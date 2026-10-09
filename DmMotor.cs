using System;

namespace DmCanDriver
{
    /// <summary>电机型号（用于 MIT 模式的位置/速度/力矩映射范围 PMAX/VMAX/TMAX）。</summary>
    public enum DmMotorType
    {
        Dm4310, Dm4310_48V, Dm4340, Dm4340_48V,
        Dm6006, Dm8006, Dm8009, Dm10010L, Dm10010,
        DmH3510, DmH6215, DmG6220,
    }
    /// <summary>达妙电机反馈帧ERR状态码（反馈字节D0高4位），参考说明书</summary>
    public enum DmMotorErrCode : byte
    {
        /// <summary>0 失能状态(Disabled)</summary>
        Disabled = 0x00,
        /// <summary>1 使能状态(Enabled)</summary>
        Enabled = 0x01,

        /// <summary>8 超压(Over‑voltage)</summary>
        OverVoltage = 0x08,
        /// <summary>9 欠压(Under‑voltage)</summary>
        UnderVoltage = 0x09,
        /// <summary>A 过电流(Over‑current)</summary>
        OverCurrent = 0x0A,
        /// <summary>B MOS管过温(MOS over temperature)</summary>
        MosOverTemp = 0x0B,
        /// <summary>C 电机线圈过温(Motor coil over temperature)</summary>
        MotorCoilOverTemp = 0x0C,
        /// <summary>D 通讯丢失(Communication lost)</summary>
        CommLost = 0x0D,
        /// <summary>E 过载(Overload)</summary>
        OverLoad = 0x0E,

        /// <summary>未知错误码</summary>
        Unknown = 0xFF
    }



    /// <summary>电机控制模式（寄存器 0x0A CTRL_MODE 的取值）。</summary>
    public enum DmControlMode : uint
    {
        Mit = 1,        // MIT 模式
        PosVel = 2,     // 位置速度模式
        Vel = 3,        // 速度模式
        Force = 4,      // 力位混控模式
    }

    /// <summary>内部寄存器地址表（详见说明书第 17~18 页）。</summary>
    public enum DmRegister : byte
    {
        UV_Value = 0x00,   // 低压保护值        float
        KT_Value = 0x01,   // 扭矩系数          float
        OT_Value = 0x02,   // 过温保护值        float
        OC_Value = 0x03,   // 过流保护值        float
        ACC      = 0x04,   // 加速度            float
        DEC      = 0x05,   // 减速度            float
        MAX_SPD  = 0x06,   // 最大速度          float
        MST_ID   = 0x07,   // 反馈 ID           uint32
        ESC_ID   = 0x08,   // 接收 ID           uint32
        TIMEOUT  = 0x09,   // 超时警报时间      uint32
        CTRL_MODE= 0x0A,   // 控制模式          uint32
        Damp     = 0x0B,   // 电机粘滞系数      float
        Inertia  = 0x0C,   // 电机转动惯量      float
        hw_ver   = 0x0D,   // 硬件版本          uint32
        sw_ver   = 0x0E,   // 软件版本          uint32
        SN       = 0x0F,   // 序列号            uint32
        NPP      = 0x10,   // 电机极对数        uint32
        Rs       = 0x11,   // 相电阻            float
        Ls       = 0x12,   // 相电感            float
        Flux     = 0x13,   // 磁链值            float
        Gr       = 0x14,   // 齿轮减速比        float
        PMAX     = 0x15,   // 位置映射范围      float
        VMAX     = 0x16,   // 速度映射范围      float
        TMAX     = 0x17,   // 扭矩映射范围      float
        I_BW     = 0x18,   // 电流环带宽        float
        KP_ASR   = 0x19,   // 速度环 Kp         float
        KI_ASR   = 0x1A,   // 速度环 Ki         float
        KP_APR   = 0x1B,   // 位置环 Kp         float
        KI_APR   = 0x1C,   // 位置环 Ki         float
        OV_Value = 0x1D,   // 过压保护值        float
        GREF     = 0x1E,   // 齿轮力矩效率      float
        Deta     = 0x1F,   // 速度环阻尼系数    float
        V_BW     = 0x20,   // 速度环滤波带宽    float
        IQ_c1    = 0x21,   // 电流环增强系数    float
        VL_c1    = 0x22,   // 速度环增强系数    float
        can_br   = 0x23,   // CAN 波特率代码    uint32
        sub_ver  = 0x24,   // 子版本号          uint32
        Boot_ver = 0x25,   // Boot 版本号       uint32
        dir      = 0x37,   // 方向              float
        m_off    = 0x38,   // 电机侧角度偏移    float
        Imax     = 0x3B,   // 驱动板最大电流    float
        VBus     = 0x3C,   // 电源电压          float
        Tpcb     = 0x3D,   // 驱动板温度        float
        Tmtr     = 0x3E,   // 电机温度          float
        Iu_off   = 0x3F,   // U 相电流偏置      float
        Iv_off   = 0x40,   // V 相电流偏置      float
        Iw_off   = 0x41,   // W 相电流偏置      float
        p_m      = 0x50,   // 电机当前位置      float
        xout     = 0x51,   // 输出轴位置        float
    }

    /// <summary>电机实时反馈状态（反馈帧，见说明书第 13 页）。</summary>
    public readonly struct DmFeedback
    {
        public readonly uint Id;        // 电机 ID（反馈帧 D0 低 4 位）
        public readonly byte Err;       // 状态码（D0 高 4 位）
        public readonly DmMotorErrCode enErr; // 时间戳（D1~D4，单位 ms）
        public readonly float Pos;      // 位置 rad
        public readonly float Vel;      // 速度 rad/s
        public readonly float Torque;   // 扭矩 N·m
        public readonly float MosTemp;  // 驱动 MOS 温度 ℃
        public readonly float RotorTemp;// 电机线圈温度 ℃

        public DmFeedback(uint id, byte err, float pos, float vel, float torque, float mosTemp, float rotorTemp)
        {
            Id = id; Err = err; Pos = pos; Vel = vel;
            Torque = torque; MosTemp = mosTemp; RotorTemp = rotorTemp;
            enErr = (DmMotorErrCode)err;
        }

        public bool IsEnabled => (Err & 0x01) != 0;

        public override string ToString() =>
            $"ID=0x{Id:X} ERR=0x{Err:X2} POS={Pos:F4}rad VEL={Vel:F4}rad/s T={Torque:F4}Nm " +
            $"T_MOS={MosTemp:F1}℃ T_Rotor={RotorTemp:F1}℃";
    }

    /// <summary>
    /// 单个电机的参数对象。记录 CAN_ID（接收命令 ID）、MST_ID（反馈 ID）、
    /// 控制模式与型号限幅，以及最近一次反馈与已读取的寄存器缓存。
    /// </summary>
    public sealed class DmMotor
    {
        public uint CanId { get; }
        public uint MasterId { get; }
        public DmControlMode Mode { get; set; }
        public DmMotorType MotorType { get; }

        // MIT 模式映射范围（PMAX/VMAX/TMAX），与官方库一致
        public float PMax, VMax, TMax;

        /// <summary>最近一次收到的反馈。</summary>
        public DmFeedback? LastFeedback { get; internal set; }

        /// <summary>最近一次读寄存器得到的值缓存（key=寄存器地址）。</summary>
        private readonly System.Collections.Generic.Dictionary<byte, double> _regCache
            = new System.Collections.Generic.Dictionary<byte, double>();

        public DmMotor(DmMotorType type, uint canId, uint masterId, DmControlMode mode = DmControlMode.Mit)
        {
            MotorType = type;
            CanId = canId;
            MasterId = masterId;
            Mode = mode;
            SetLimits();
        }

        /// <summary>根据型号设置 MIT 映射限幅（PMAX/VMAX/TMAX），官方默认值。</summary>
        private void SetLimits()
        {
            switch (MotorType)
            {
                case DmMotorType.Dm4310:      (PMax, VMax, TMax) = (12.5f, 30f, 10f); break;
                case DmMotorType.Dm4310_48V:  (PMax, VMax, TMax) = (12.5f, 50f, 10f); break;
                case DmMotorType.Dm4340:      (PMax, VMax, TMax) = (12.5f, 8f, 28f); break;
                case DmMotorType.Dm4340_48V:  (PMax, VMax, TMax) = (12.5f, 10f, 28f); break;
                case DmMotorType.Dm6006:      (PMax, VMax, TMax) = (12.5f, 45f, 20f); break;
                case DmMotorType.Dm8006:      (PMax, VMax, TMax) = (12.5f, 45f, 40f); break;
                case DmMotorType.Dm8009:      (PMax, VMax, TMax) = (12.5f, 45f, 54f); break;
                case DmMotorType.Dm10010L:    (PMax, VMax, TMax) = (12.5f, 25f, 200f); break;
                case DmMotorType.Dm10010:     (PMax, VMax, TMax) = (12.5f, 20f, 200f); break;
                case DmMotorType.DmH3510:     (PMax, VMax, TMax) = (12.5f, 280f, 1f); break;
                case DmMotorType.DmH6215:     (PMax, VMax, TMax) = (12.5f, 45f, 10f); break;
                case DmMotorType.DmG6220:     (PMax, VMax, TMax) = (12.5f, 45f, 10f); break;
                default:                      (PMax, VMax, TMax) = (12.5f, 30f, 10f); break;
            }
        }

        /// <summary>记录读寄存器结果。</summary>
        internal void CacheRegister(byte rid, double value) => _regCache[rid] = value;

        /// <summary>取某寄存器的缓存值（未读到返回 null）。</summary>
        public double? GetCached(byte rid) => _regCache.TryGetValue(rid, out var v) ? v : null;

        /// <summary>当前控制模式对应的命令帧 ID（使能/失能/控制帧都发到该 ID）。</summary>
        public uint ControlFrameId => Mode switch
        {
            DmControlMode.Mit    => CanId,
            DmControlMode.PosVel => 0x100 + CanId,
            DmControlMode.Vel    => 0x200 + CanId,
            DmControlMode.Force  => 0x300 + CanId,
            _ => CanId,
        };

        /// <summary>将软件版本 uint32 格式化为 "x.y.z.w"。</summary>
        /// <remarks>
        /// 固件返回的版本分两种字节目序（均按 data[4..8] 原始字节序）：
        ///   - sw_ver  为 ASCII 数字，如 "7020"  → 7.0.2.0（截图 FW REV: 7.0.2.0）
        ///   - Boot_ver 为裸字节，如 03 02 00 06 → 3.2.0.6（截图 BL REV: 3.2.0.6）
        /// 两者都按 b[0].b[1].b[2].b[3] 顺序拼装；区别仅在 ASCII 需减 0x30。
        /// </remarks>
        public static string FormatSwVersion(uint swVer)
        {
            byte[] b = BitConverter.GetBytes(swVer); // 小端，b[0..3] 即版本字节目序
            bool ascii = true;
            for (int i = 0; i < 4; i++)
                if (b[i] < 0x30 || b[i] > 0x39) { ascii = false; break; }

            int[] d = new int[4];
            for (int i = 0; i < 4; i++) d[i] = ascii ? (b[i] - 0x30) : (int)b[i];
            return $"{d[0]}.{d[1]}.{d[2]}.{d[3]}";
        }
    }
}
//（注：内容由AI生成）
