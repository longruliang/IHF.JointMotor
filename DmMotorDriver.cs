using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

namespace DmCanDriver
{
    /// <summary>
    /// 达妙 DM-J4340 系列减速电机的串口-CAN 通讯驱动（配达妙 USB2CAN 调试模块）。
    ///
    /// 职责分层：
    ///   1) Usb2CanProtocol —— 串口帧封装/解析 + CRC-8 校验；
    ///   2) DmMotorDriver —— 串口收发、后台接收线程、请求-应答关联、电机命令；
    ///   3) DmMotor —— 电机对象（CAN_ID / MST_ID / 模式 / 型号限幅 / 反馈与参数缓存）。
    ///
    /// 使用前请确保：
    ///   - 串口参数 921600-8-N-1；
    ///   - 模块 CAN 波特率与电机一致（默认 1Mbps）；
    ///   - 电机上电自检完成（红灯常亮=失能）后再使能。
    /// </summary>
    public sealed class DmMotorDriver : IDisposable
    {
        public const int DefaultBaudRate = 921600;

        /// <summary>收到一条从总线来的反馈帧。</summary>
        public event Action<DmMotor, DmFeedback>? FeedbackReceived;

        /// <summary>收到任意一条非回显的 CAN 帧（含寄存器应答）。</summary>
        public event Action<DmMotor?, Usb2CanProtocol.CanFrame>? FrameReceived;

        /// <summary>串口或协议错误。</summary>
        public event Action<Exception>? ErrorOccurred;

        private readonly SerialPort _port;
        private readonly Usb2CanProtocol.FrameParser _parser = new Usb2CanProtocol.FrameParser();
        private readonly object _writeLock = new object();
        private readonly object _pendingLock = new object();

        // 电机表：按 CanId 与 MasterId 均可索引
        private readonly Dictionary<uint, DmMotor> _motorsByCanId = new Dictionary<uint, DmMotor>();
        private readonly Dictionary<uint, DmMotor> _motorsByMasterId = new Dictionary<uint, DmMotor>();

        // 读/写寄存器请求-应答表：key = (canId << 8) | rid
        private readonly Dictionary<int, TaskCompletionSource<(bool IsWrite, byte[] Data)>> _pending = new();

        private Thread? _rxThread;
        private volatile bool _running;
        private byte[] _rxBuffer = new byte[1024];

        public bool IsOpen => _port.IsOpen;

        public DmMotorDriver(string portName, int baudRate = DefaultBaudRate)
        {
            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = SerialPort.InfiniteTimeout,
                WriteTimeout = 1000,
                DtrEnable = true,
                RtsEnable = true,
            };
        }

        #region 连接管理

        public void Open()
        {
            if (_port.IsOpen) return;
            _port.Open();
            _running = true;
            _rxThread = new Thread(RxLoop) { IsBackground = true, Name = "DM-CAN-Rx" };
            _rxThread.Start();
        }

        public void Close()
        {
            _running = false;
            try { _port.Close(); } catch { /* 忽略关闭异常 */ }
            if (_rxThread != null)
            {
                try { if (_rxThread.IsAlive) _rxThread.Join(500); } catch { }
                _rxThread = null;
            }
            CompleteAllPending(false);
        }

        public void Dispose() => Close();

        /// <summary>注册一台电机，使其进入按 ID 索引的表。</summary>
        public void AddMotor(DmMotor motor)
        {
            if (motor == null) throw new ArgumentNullException(nameof(motor));
            _motorsByCanId[motor.CanId] = motor;
            _motorsByMasterId[motor.MasterId] = motor;
        }

        private void RxLoop()
        {
            try
            {
                while (_running && _port.IsOpen)
                {
                    int n = _port.Read(_rxBuffer, 0, _rxBuffer.Length);
                    if (n <= 0) continue;
                    var frames = _parser.Push(new ReadOnlySpan<byte>(_rxBuffer, 0, n));
                    foreach (var f in frames) Dispatch(f);
                }
            }
            catch (Exception ex)
            {
                if (_running) ErrorOccurred?.Invoke(ex);
            }
        }

        private void Dispatch(Usb2CanProtocol.CanFrame f)
        {
            if (f.IsEcho) return; // 模块回显本机发出的帧，忽略

            // 寄存器应答帧：data[2]==0x33（读应答）或 0x55（写应答）
            if (f.Data.Length == 8 && (f.Data[2] == 0x33 || f.Data[2] == 0x55))
            {
                uint canId = (uint)(f.Data[0] | (f.Data[1] << 8));
                byte rid = f.Data[3];
                var key = PendingKey(canId, rid);
                TaskCompletionSource<(bool, byte[])>? tcs;
                lock (_pendingLock)
                {
                    if (_pending.TryGetValue(key, out tcs)) _pending.Remove(key);
                    else tcs = null;
                }
                // 应答数据：[canid_l, canid_h, 0x33/0x55, rid, val0..3]，值取 [4..8]
                tcs?.TrySetResult((f.Data[2] == 0x55, f.Data[4..8].ToArray()));
                FrameReceived?.Invoke(null, f);
                return;
            }

            // 反馈帧：通过仲裁 ID（=MST_ID）定位电机
            if (_motorsByMasterId.TryGetValue(f.Id, out var motor))
            {
                var fb = ParseFeedback(motor, f.Data);
                motor.LastFeedback = fb;
                FeedbackReceived?.Invoke(motor, fb);
            }
            FrameReceived?.Invoke(null, f);
        }

        #endregion

        #region 底层发送

        /// <summary>向 CAN 总线发送一条 8 字节数据帧。</summary>
        public void SendCanFrame(uint canId, byte[] data)
        {
            byte[] frame = Usb2CanProtocol.EncodeSendFrame(canId, data);
            lock (_writeLock)
            {
                if (!_port.IsOpen) throw new InvalidOperationException("串口未打开。");
                _port.Write(frame, 0, frame.Length);
            }
        }

        #endregion

        #region 寄存器读/写

        /// <summary>
        /// 读取指定寄存器并等待应答。返回解码后的 double 值。
        /// </summary>
        public async Task<double> ReadRegisterAsync(DmMotor motor, DmRegister reg, int timeoutMs = 300)
        {
            byte rid = (byte)reg;
            var data = new byte[8] { (byte)(motor.CanId & 0xFF), (byte)((motor.CanId >> 8) & 0xFF), 0x33, rid, 0, 0, 0, 0 };
            var tcs = RegisterPending(motor.CanId, rid, isWrite: false);
            SendCanFrame(0x7FF, data);
            var (isWrite, value) = await tcs.Task.WaitAsync(TimeSpan.FromMilliseconds(timeoutMs));
            double result = DecodeRegisterValue(rid, value);
            motor.CacheRegister(rid, result);
            return result;
        }



        /// <summary>读取寄存器（同步阻塞版）。</summary>
        public double ReadRegister(DmMotor motor, DmRegister reg, int timeoutMs = 300) =>
            ReadRegisterAsync(motor, reg, timeoutMs).GetAwaiter().GetResult();

        /// <summary>
        /// 写寄存器并等待写应答。写后立即生效，掉电丢失；如需保存请调 SaveParams。
        /// </summary>
        public async Task<bool> WriteRegisterAsync(DmMotor motor, DmRegister reg, double value, int timeoutMs = 300)
        {
            byte rid = (byte)reg;
            byte[] enc = EncodeRegisterValue(rid, value);
            var data = new byte[8] {
                (byte)(motor.CanId & 0xFF), (byte)((motor.CanId >> 8) & 0xFF), 0x55, rid,
                enc[0], enc[1], enc[2], enc[3]
            };
            var tcs = RegisterPending(motor.CanId, rid, isWrite: true);
            SendCanFrame(0x7FF, data);
            var (isWrite, _) = await tcs.Task.WaitAsync(TimeSpan.FromMilliseconds(timeoutMs));
            motor.CacheRegister(rid, value);
            return isWrite;
        }

        /// <summary>写寄存器（同步阻塞版）。</summary>
        public bool WriteRegister(DmMotor motor, DmRegister reg, double value, int timeoutMs = 300) =>
            WriteRegisterAsync(motor, reg, value, timeoutMs).GetAwaiter().GetResult();

        /// <summary>存储全部参数到片内 flash（仅失能模式下生效，最大耗时 30ms）。</summary>
        public void SaveParams(DmMotor motor)
        {
            var data = new byte[8] { (byte)(motor.CanId & 0xFF), (byte)((motor.CanId >> 8) & 0xFF), 0xAA, 0x01, 0, 0, 0, 0 };
            SendCanFrame(0x7FF, data);
        }

        /// <summary>读取固件版本：返回 (软件版本, 子版本, Boot 版本)。</summary>
        public async Task<(string Fw, uint Sub, string Boot)> ReadVersionAsync(DmMotor motor)
        {
            double sw = await ReadRegisterAsync(motor, DmRegister.sw_ver);
            double sub = await ReadRegisterAsync(motor, DmRegister.sub_ver);
            double boot = await ReadRegisterAsync(motor, DmRegister.Boot_ver);
            return (DmMotor.FormatSwVersion((uint)sw), (uint)sub,
                    DmMotor.FormatSwVersion((uint)boot));
        }

        /// <summary>读取固件版本（同步阻塞版）。</summary>
        public (string Fw, uint Sub, string Boot) ReadVersion(DmMotor motor) =>
            ReadVersionAsync(motor).GetAwaiter().GetResult();

        private TaskCompletionSource<(bool, byte[])> RegisterPending(uint canId, byte rid, bool isWrite)
        {
            var tcs = new TaskCompletionSource<(bool, byte[])>(TaskCreationOptions.RunContinuationsAsynchronously);
            var key = PendingKey(canId, rid);
            lock (_pendingLock) _pending[key] = tcs;
            return tcs;
        }

        private static int PendingKey(uint canId, byte rid) => (int)((canId << 8) | rid);

        private void CompleteAllPending(bool result)
        {
            lock (_pendingLock)
            {
                foreach (var tcs in _pending.Values)
                    tcs.TrySetResult((result, Array.Empty<byte>()));
                _pending.Clear();
            }
        }

        // 哪些寄存器按 uint32 处理（与官方库一致），其余按 float（小端）
        private static bool IsUint32Register(byte rid) =>
            (rid >= 0x07 && rid <= 0x0A) || (rid >= 0x0D && rid <= 0x10) ||
            (rid >= 0x23 && rid <= 0x25);

        private static double DecodeRegisterValue(byte rid, byte[] v)
        {
            if (v.Length < 4) return 0;
            if (IsUint32Register(rid))
            {
                uint u = (uint)(v[0] | (v[1] << 8) | (v[2] << 16) | (v[3] << 24));
                return u;
            }
            return BitConverter.ToSingle(v, 0);
        }

        private static byte[] EncodeRegisterValue(byte rid, double value)
        {
            if (IsUint32Register(rid))
            {
                uint u = (uint)value;
                return new byte[] { (byte)u, (byte)(u >> 8), (byte)(u >> 16), (byte)(u >> 24) };
            }
            return BitConverter.GetBytes((float)value);
        }

        #endregion

        #region 电机使能 / 失能 / 控制

        /// <summary>使能电机（上电自检后发送）。发到当前控制模式的命令帧 ID。</summary>
        public void Enable(DmMotor motor, uint? frameId = null) =>
            SendControlCmd(frameId ?? motor.ControlFrameId, 0xFC);

        /// <summary>失能电机。</summary>
        public void Disable(DmMotor motor, uint? frameId = null) =>
            SendControlCmd(frameId ?? motor.ControlFrameId, 0xFD);

        /// <summary>保存当前位置为输出轴零点（失能状态下执行）。</summary>
        public void SaveZeroPosition(DmMotor motor, uint? frameId = null) =>
            SendControlCmd(frameId ?? motor.ControlFrameId, 0xFE);

        /// <summary>清除错误。</summary>
        public void ClearError(DmMotor motor, uint? frameId = null) =>
            SendControlCmd(frameId ?? motor.ControlFrameId, 0xFB);

        private void SendControlCmd(uint frameId, byte cmd)
        {
            var data = new byte[8] { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd };
            SendCanFrame(frameId, data);
        }

        /// <summary>
        /// 主动请求电机返回一帧反馈（问询式）。发 0x7FF + [canid_l, canid_h, 0xCC, 0...]。
        /// 应答通过 FeedbackReceived 事件 / motor.LastFeedback 获取。
        /// </summary>
        public void RefreshStatus(DmMotor motor)
        {
            var data = new byte[8] { (byte)(motor.CanId & 0xFF), (byte)((motor.CanId >> 8) & 0xFF), 0xCC, 0, 0, 0, 0, 0 };
            SendCanFrame(0x7FF, data);
        }

        /// <summary>MIT 模式控制帧（ID = CAN_ID）。位置 rad、速度 rad/s、力矩 N·m。</summary>
        public void ControlMit(DmMotor motor, float kp, float kd, float pos, float vel, float torque)
        {
            ushort p = FloatToUint(pos, -motor.PMax, motor.PMax, 16);
            ushort v = FloatToUint(vel, -motor.VMax, motor.VMax, 12);
            ushort kpp = FloatToUint(kp, 0, 500, 12);
            ushort kdd = FloatToUint(kd, 0, 5, 12);
            ushort t = FloatToUint(torque, -motor.TMax, motor.TMax, 12);

            var data = new byte[8]
            {
                (byte)(p >> 8), (byte)(p & 0xFF),
                (byte)(v >> 4),
                (byte)(((v & 0x0F) << 4) | ((kpp >> 8) & 0x0F)),
                (byte)(kpp & 0xFF),
                (byte)(kdd >> 4),
                (byte)(((kdd & 0x0F) << 4) | ((t >> 8) & 0x0F)),
                (byte)(t & 0xFF),
            };
            SendCanFrame(motor.CanId, data);
        }

        /// <summary>位置速度模式控制帧（ID = 0x100 + CAN_ID）。p_des rad，v_des rad/s。</summary>
        public void ControlPosVel(DmMotor motor, float pos, float vel)
        {
            var data = new byte[8];
            BitConverter.GetBytes(pos).CopyTo(data, 0);   // float 小端
            BitConverter.GetBytes(vel).CopyTo(data, 4);
            SendCanFrame(0x100 + motor.CanId, data);
        }

        /// <summary>速度模式控制帧（ID = 0x200 + CAN_ID）。v_des rad/s。</summary>
        public void ControlVel(DmMotor motor, float vel)
        {
            var data = new byte[8];
            BitConverter.GetBytes(vel).CopyTo(data, 0);
            SendCanFrame(0x200 + motor.CanId, data);
        }

        /// <summary>
        /// 力位混控模式控制帧（ID = 0x300 + CAN_ID）。
        /// pos rad(float)；vel 限速值放大 100 倍（0~10000 => 0~100 rad/s）；
        /// iDes 电流标幺放大 10000 倍（0~10000 => 0~1.0）。
        /// </summary>
        public void ControlForce(DmMotor motor, float pos, float vel, float iDes)
        {
            var data = new byte[8];
            BitConverter.GetBytes(pos).CopyTo(data, 0);
            ushort v = (ushort)Math.Min(10000, Math.Max(0, (int)(vel * 100)));
            ushort i = (ushort)Math.Min(10000, Math.Max(0, (int)(iDes * 10000)));
            data[4] = (byte)(v & 0xFF); data[5] = (byte)(v >> 8);
            data[6] = (byte)(i & 0xFF); data[7] = (byte)(i >> 8);
            SendCanFrame(0x300 + motor.CanId, data);
        }

        #endregion

        #region 编码工具

        private static ushort FloatToUint(float x, float min, float max, int bits)
        {
            x = Math.Clamp(x, min, max);
            float span = max - min;
            float norm = (x - min) / span;
            return (ushort)Math.Round(norm * ((1 << bits) - 1));
        }

        private static DmFeedback ParseFeedback(DmMotor motor, byte[] d)
        {
            byte id = (byte)(d[0] & 0x0F);
            byte err = (byte)((d[0] >> 4) & 0x0F);
            ushort qU = (ushort)((d[1] << 8) | d[2]);
            ushort vU = (ushort)((d[3] << 4) | (d[4] >> 4));
            ushort tU = (ushort)(((d[4] & 0x0F) << 8) | d[5]);
            float q = UintToFloat(qU, -motor.PMax, motor.PMax, 16);
            float v = UintToFloat(vU, -motor.VMax, motor.VMax, 12);
            float t = UintToFloat(tU, -motor.TMax, motor.TMax, 12);
            float mos = d[6];
            float rotor = d[7];
            return new DmFeedback(id, err, q, v, t, mos, rotor);
        }

        private static float UintToFloat(ushort x, float min, float max, int bits)
        {
            float norm = (float)x / ((1 << bits) - 1);
            return norm * (max - min) + min;
        }

        #endregion
    }
}
//（注：内容由AI生成）
