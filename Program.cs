using System;
using DmCanDriver;

// ============================================================
//  达妙 DM-J4340-2EC 减速电机 · 串口-CAN 通讯驱动 演示
//
//  与 Bus Hound 抓包对应：
//    - 读版本：读寄存器 0x0E(sw_ver)、0x24(sub_ver)、0x25(Boot_ver)
//      （抓包中 sw_ver 返回 37 30 32 30 = "7020" -> 7.0.2.0，
//         Boot_ver 返回 03 02 00 06 -> 3.2.0.6，与调试助手 FW/BL REV 一致）
//    - 读参数：读寄存器列表
// ============================================================

using var driver = new DmMotorDriver("COM3", 921600); // 改成你的串口号
driver.ErrorOccurred += ex => Console.WriteLine($"[错误] {ex.Message}");
driver.FeedbackReceived += (m, fb) => Console.WriteLine($"[反馈] {fb}");

// DM-J4340：CAN_ID=0x01，MST_ID=0x00（与调试助手 CAN ID:1 一致），MIT 模式
var motor = new DmMotor(DmMotorType.Dm4340, canId: 0x01, masterId: 0x00, DmControlMode.Mit);
driver.AddMotor(motor);

driver.Open();
Console.WriteLine($"串口已打开: COM3 @ {DmMotorDriver.DefaultBaudRate}");

// ---------- 1) 读版本（对应抓包） ----------
Console.WriteLine("\n== 读版本 ==");
for (int i = 0; i < 10; i++)
{
    try
    {
        var (fw, sub, boot) = driver.ReadVersion(motor);
        Console.WriteLine($"FW REV : {fw}   (子版本 {sub})");
        Console.WriteLine($"BL REV : {boot}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"读版本失败: {ex.Message}");
    }

}

// ---------- 2) 读参数（寄存器列表） ----------
Console.WriteLine("\n== 读参数 ==");
var regs = new[]
{
    (DmRegister.NPP,    "极对数"),
    (DmRegister.Rs,     "相电阻(Ω)"),
    (DmRegister.Ls,     "相电感(H)"),
    (DmRegister.Gr,     "减速比"),
    (DmRegister.KT_Value,"扭矩系数"),
    (DmRegister.MST_ID, "反馈ID"),
    (DmRegister.ESC_ID, "接收ID"),
    (DmRegister.PMAX,   "PMAX"),
    (DmRegister.VMAX,   "VMAX"),
    (DmRegister.TMAX,   "TMAX"),
    (DmRegister.VBus,   "电源电压(V)"),
    (DmRegister.Tmtr,   "电机温度(℃)"),
    (DmRegister.Tpcb,   "驱动板温度(℃)"),
};
foreach (var (reg, name) in regs)
{
    try
    {
        double v = driver.ReadRegister(motor, reg);
        Console.WriteLine($"  {name,-12}(0x{(byte)reg:X2}) = {v}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"  {name} 读取失败: {ex.Message}");
    }
}

// ---------- 3) 使能 / 控制 / 失能（示例，可注释） ----------
// 注意：使能前电机必须完成自检且处于失能状态；控制时请固定电机！
Console.WriteLine("\n== 控制示例 ==");
// driver.Enable(motor);
// Thread.Sleep(100);
// driver.ControlMit(motor, kp: 0, kd: 0.1f, pos: 0, vel: 5f, torque: 0);  // MIT 匀速 5 rad/s
// driver.RefreshStatus(motor);                                             // 主动要一帧反馈
// Thread.Sleep(200);
// driver.ControlMit(motor, kp: 0, kd: 0, pos: 0, vel: 0, torque: 0);
// driver.Disable(motor);
driver.Close();
Console.WriteLine("\n演示结束，按任意键关闭。");
Console.ReadKey();

//（注：内容由AI生成）
