# V2 硬件接口

## 电机接入

在 CubeMX 完成初始化后，对每条使用的总线调用一次 `can_bus::start(&hfdcanX)`，检查返回值，然后初始化具体型号。电机对象须在整个使用期间存活。请在应用停机时先下发失能命令，再销毁实例。大疆的失能需要再调用一次分组发送。

| 所接总线 | 达妙默认帧格式 | 仲裁段 | 数据段 | 反馈入口 |
| --- | --- | --- | --- | --- |
| FDCAN1、FDCAN3 | 经典 CAN | 1 Mbps | 1 Mbps | FIFO0 |
| FDCAN2 | CAN FD + BRS | 1 Mbps | 5 Mbps | FIFO1 |

FDCAN2 的内核时钟为 80 MHz。仲裁段每位为 `2 × (1 + 32 + 7)` 个时钟，数据段每位为 `1 × (1 + 11 + 4)` 个时钟。两种格式均使用 11 位标准 ID 和 8 字节 MIT 载荷。电调的模式和波特率须与该节点匹配；发送 FD 帧的物理总线上，其他节点也须能容忍 FD 帧。

总线进入 Bus_Off 后，`hfdcan_bsp` 在错误状态中断里自动调用 `can_bus::Soft_Restart`：先中止未发完的 Tx 请求，再清除 `CCCR.INIT`，控制器自行完成 129 次 11 个连续隐性位的恢复序列后继续收发。若软复位本身失败（`HAL_FDCAN_Stop` 超时会让 HAL 状态机进入 ERROR），总线保持静默且不再上报，需要应用层调用 `can_bus::Hard_Restart` 兜底。

- 达妙使用 `init(command_id, feedback_id)` 注册。第一个参数是发往电调的命令 ID（CAN ID），第二个是电调反馈帧的返回 ID（Master ID）。在构造和注册时都可以直接写整数字面量或 `constexpr` 整数。编译器在缩窄转换前检查两个 ID 是否都在 `0x000…0x7FF` 内。
- 帧格式默认跟随总线。要显式选择经典 CAN 或 CAN FD + BRS，第三个参数传 `DmCanMode::Classic` 或 `DmCanMode::FdBrs`。原有 `Dm4310 joint(&hfdcan2, command_id, feedback_id)` 写法仍可用，随后调用 `joint.init()` 或 `joint.init(mode)` 注册构造时给定的地址。
- 各型号直接继承 `DmMotor`。在 `Motor/DM/dm_models.hpp` 中填写与该型号电调寄存器一致的 `T_MAX`。构造该型号时编译器要求它是有限正数，未使用的型号可以保留占位值。各型号共用 `P_MAX = 12.5664 rad` 和 `V_MAX = 45 rad/s`。
- 每台电机独立声明和注册。同一总线上的命令 ID 必须唯一。共用返回 ID 时，命令 ID 的低四位须能区分反馈。运行时遇到这些冲突、重复注册时改绑、总线模式不匹配或注册表已满，`init()` 返回 `false`。不同总线可复用同一组 ID。
- 一个业务对象写入每台电机的目标，观察方读取 `feedback()` 快照。角速度为 rad/s，力矩为 N·m。首帧前 `valid` 为假。单圈角范围为 `[−π, π)`，累计角包含首帧原始角。连续解圈要求相邻有效采样之间的机械位移小于半圈。

```cpp
#include "dm_models.hpp"
#include "hfdcan_bsp.h"

static Dm4310 joint(&hfdcan2);

bool initJoint() {
    return can_bus::start(&hfdcan2) && joint.init(0x01, 0x11) && joint.enable();
}

bool sendJointTorque(float joint_nm) {
    if (!joint.setTorque(joint_nm)) return false;
    const auto dm = DmMotor::sendAll();
    return dm.queued_frames == dm.attempted_frames;
}
```

`setMit({position_rad, velocity_rad_s, kp, kd, torque_ff_nm})` 和 `setTorque()` 只更新目标。每拍分别调用 `DmMotor::sendAll()` 和 `DjiMotorGroup::sendAll()`，重新提交对应品牌的当前目标，返回本次尝试和成功入队的 CAN 帧数。使能、失能、清故障操作单独检查返回值。M3508 的 `resetAccumulatedAngle()` 立即将软件累计角归零。

## 传感器

用 `Bmi088 imu(&hspi2)` 构造板载 IMU，也可传入第二个参数 `Bmi088::Config`。上电调用 `imu.init()`，成功返回 `true`。节拍中调用 `imu.start()`，上一帧仍在传输或等待提交时返回 `StartResult::Skipped`。SPI2 DMA 串联读取加速度、温度和角速度，主循环调用 `imu.process()`，仅 `ProcessResult::Updated` 表示发布了一帧新数据。

`imu.feedback()` 按值返回传感器原生 XYZ 轴序的完整快照。加速度 `accel_m_s2` 使用 m/s²，角速度 `gyro_rad_s` 使用 rad/s，温度 `temperature_c` 使用 °C。首帧前 `valid` 为假，每次成功发布更新 `sequence` 和 `last_update_ms`。传输失败时可继续读取上一帧。`imu.online(timeout_ms)` 检查已发布帧的年龄，默认有效期为 100 ms，含边界。

FlySky SBUS 在 UART5 上使用 DMA，`snapshot()` 按值读取同一帧的全部原始通道、丢帧和失控标志，`channel(index)` 按 0 基下标读取归一化至 [-1, 1] 的单通道值。

## 验证

STM32 固件使用根目录的 Debug、Release 预设。实机验收时，对照分析仪核对 FDCAN1/3 的经典帧和 FDCAN2 的 FDF、BRS、1/5 Mbps 位时序及双向反馈。
