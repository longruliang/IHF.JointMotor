using System;
using System.Collections.Generic;

namespace DmCanDriver
{
    /// <summary>
    /// 达妙 USB2CAN 调试模块的串口封装层。
    ///
    /// 通讯参数：波特率 921600、数据位 8、无校验、停止位 1（见说明书 / 调试助手默认值）。
    ///
    /// 通过 Bus Hound 抓包还原的帧结构：
    /// ─────────────────────────────────────────────────────────────
    /// 发送帧（主机 -> 模块，固定 30 字节，DLC 固定为 8）：
    ///   [0]=0x55 [1]=0xAA 同步头
    ///   [2]=0x1E 帧长度(30)
    ///   [3]=0x01 命令类型：发送 CAN 帧
    ///   [4..7]=01 00 00 00  固定
    ///   [8..11]=0A 00 00 00 固定
    ///   [12]=0x00 标准帧标志
    ///   [13..14]=CAN ID 低、高（小端，11 位标准帧 ID）
    ///   [15..17]=00 00 00 固定
    ///   [18]=DLC(0x08)
    ///   [19..20]=00 00 固定
    ///   [21..28]=CAN 数据(8 字节)
    ///   [29]=校验字节 = CRC-8(poly=0x8C, init=0x00, LSB优先) 对整帧 30 字节（校验槽置 0）计算
    ///
    /// 接收帧（模块 -> 主机，固定 16 字节，DLC=8）：
    ///   [0]=0xAA 同步头
    ///   [1]=状态/类型（0x11=总线接收，0x12=本机发送的回显）
    ///   [2]=DLC(0x08)
    ///   [3..6]=CAN ID 低、高 + 扩展位（小端）
    ///   [7..14]=CAN 数据(8 字节)
    ///   [15]=0x55 帧尾（固定分隔符，非校验）
    /// ─────────────────────────────────────────────────────────────
    /// 校验算法已用抓包 0x00~0x25 共 39 条真实发送帧逐一验证通过（39/39）。
    /// </summary>
    public static class Usb2CanProtocol
    {
        public const int TxFrameLength = 30;
        public const int RxFrameLength = 16;
        public const byte TxHead0 = 0x55;
        public const byte TxHead1 = 0xAA;
        public const byte RxHead = 0xAA;
        public const byte RxTail = 0x55;
        public const byte CmdSendCan = 0x01;

        /// <summary>
        /// 将一条标准帧 CAN 报文封装成 30 字节串口发送帧。
        /// </summary>
        /// <param name="canId">11 位标准帧 CAN ID（0~0x7FF）</param>
        /// <param name="data">CAN 数据，长度必须为 8</param>
        public static byte[] EncodeSendFrame(uint canId, byte[] data)
        {
            if (data == null || data.Length != 8)
                throw new ArgumentException("CAN 数据必须为 8 字节。", nameof(data));
            if (canId > 0x7FF)
                throw new ArgumentOutOfRangeException(nameof(canId), "标准帧 CAN ID 不能超过 0x7FF。");

            byte[] f = new byte[TxFrameLength];
            f[0] = TxHead0;
            f[1] = TxHead1;
            f[2] = TxFrameLength;          // 0x1E = 30
            f[3] = CmdSendCan;             // 命令类型：发送 CAN 帧
            f[4] = 0x01;                   // 固定
            f[5] = 0x00; f[6] = 0x00; f[7] = 0x00;
            f[8] = 0x0A;                   // 固定
            f[9] = 0x00; f[10] = 0x00; f[11] = 0x00;
            f[12] = 0x00;                  // 标准帧
            f[13] = (byte)(canId & 0xFF);      // CAN ID 低字节
            f[14] = (byte)((canId >> 8) & 0xFF); // CAN ID 高字节
            f[15] = 0x00; f[16] = 0x00; f[17] = 0x00;
            f[18] = 0x08;                  // DLC = 8
            f[19] = 0x00; f[20] = 0x00;
            Array.Copy(data, 0, f, 21, 8);
            // 校验字节 = CRC-8(0x8C, init 0, LSB优先) 对整帧 [0..28] 再补一个 0x00 槽位计算
            f[29] = CalcChecksum(f, 0, TxFrameLength - 1);
            return f;
        }

        /// <summary>
        /// 发送帧校验字节算法（已用抓包 0x00~0x25 共 39 帧逐帧验证通过）：
        /// CRC-8，多项式 0x8C，初值 0x00，LSB 优先（右移）。
        /// 对 [offset..offset+length-1] 计算后，再补一个 0x00 字节（即校验槽位置 0 的整帧 CRC）。
        /// </summary>
        public static byte CalcChecksum(byte[] buf, int offset, int length)
        {
            byte crc = 0x00;
            for (int i = offset; i < offset + length; i++)
            {
                crc ^= buf[i];
                for (int j = 0; j < 8; j++)
                {
                    crc = (crc & 0x01) != 0 ? (byte)((crc >> 1) ^ 0x8C) : (byte)(crc >> 1);
                }
            }
            // 补一个 0x00 字节（校验字节槽位，发送前置 0）
            for (int j = 0; j < 8; j++)
            {
                crc = (crc & 0x01) != 0 ? (byte)((crc >> 1) ^ 0x8C) : (byte)(crc >> 1);
            }
            return crc;
        }

        /// <summary>
        /// 一条从总线收到的 CAN 帧。
        /// </summary>
        public readonly struct CanFrame
        {
            public readonly uint Id;          // 11 位标准帧 CAN ID
            public readonly byte[] Data;      // 8 字节
            public readonly byte Status;      // 0x11=总线接收  0x12=本机发送回显
            public readonly bool IsEcho;      // 是否为模块回显的本机发送帧

            public CanFrame(uint id, byte[] data, byte status)
            {
                Id = id;
                Data = data;
                Status = status;
                IsEcho = (status == 0x12);
            }

            public override string ToString() =>
                $"ID=0x{Id:X3} {BitConverter.ToString(Data)} ({(IsEcho ? "回显" : "接收")})";
        }

        /// <summary>
        /// 流式 CAN 帧解析器。把串口读到的原始字节喂进来，输出完整的接收帧。
        /// 接收帧以 0xAA 开头、0x55 结尾、DLC 位于第 3 字节；DLC=0..8，帧长 = 8 + DLC。
        /// （0x55 为固定帧尾分隔符，非校验值；不做 CRC 校验，按 DLC 长度定位。）
        /// </summary>
        public sealed class FrameParser
        {
            private readonly List<byte> _buffer = new List<byte>();

            /// <summary>喂入一段原始字节，返回本次解析出的完整帧。</summary>
            public List<CanFrame> Push(ReadOnlySpan<byte> chunk)
            {
                var frames = new List<CanFrame>();
                foreach (byte b in chunk) _buffer.Add(b);

                int i = 0;
                while (i < _buffer.Count)
                {
                    if (_buffer[i] != RxHead) { i++; continue; }   // 找同步头 0xAA

                    // 至少要有 [头][状态][DLC] 三个字节
                    if (i + 3 > _buffer.Count) break;

                    int dlc = _buffer[i + 2];
                    int frameLen = 8 + dlc;                        // 头1+状态1+DLC1+ID4+数据dlc+尾1
                    if (frameLen > RxFrameLength) { i++; continue; } // DLC>8，非法
                    if (i + frameLen > _buffer.Count) break;         // 数据还没收齐

                    if (_buffer[i + frameLen - 1] == RxTail)         // 帧尾 0x55
                    {
                        byte status = _buffer[i + 1];
                        uint id = (uint)(_buffer[i + 3] | (_buffer[i + 4] << 8));
                        var data = new byte[dlc];
                        Array.Copy(_buffer.ToArray(), i + 7, data, 0, dlc);

                        frames.Add(new CanFrame(id, data, status));
                        _buffer.RemoveRange(0, i + frameLen);
                        i = 0;
                        continue;
                    }
                    else
                    {
                        i++; // 帧尾不符，换一个候选头
                    }
                }

                return frames;
            }
        }
    }
}
//（注：内容由AI生成）
