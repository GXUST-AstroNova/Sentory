# Sentory（Dm_02_Bsp_V2）

Sentory 是基于 STM32H723VGT6 的嵌入式机器人固件工程。外设初始化由 STM32CubeMX 管理，硬件驱动、服务组件和应用以 C/C++ 实现，使用 CMake 和 Ninja 构建。

## 仓库定位与提交范围

本仓库维护可复用的底层支持能力，并包含用于联调的应用骨架。电机、传感器、控制工具和服务组件可供其他应用复用；具体机器人或任务的业务逻辑应按项目需要维护。提交应用改动时请同时确认它不会误导使用者把未完成的示例当作可直接上机的控制程序。

## 功能概览

- 硬件驱动：达妙电机、大疆 M3508、BMI088 IMU、FlySky SBUS、W25Q Flash、WS2812 LED 和 PWM 蜂鸣器。
- 通信与服务：UART/FDCAN BSP、板间消息编解码与收发、板级启动角色配置、全向底盘组件和 M3508 功率限制。
- 控制与调试：PID、滑模控制、轨迹滑模、VOFA 数据发送，以及 Fusion 姿态融合库。

具体接口和总线配置见 [硬件接口说明](Hardware/README.md)，分层设计见 [设计思路](设计思路.md)。各驱动的接入说明另见 [控制器串口接入约定](Hardware/Controller/README.md)、[蜂鸣器说明](Hardware/Buzzer/README.md) 和 [LED 说明](Hardware/Led/README.md)。模块是否在应用中启用，以应用代码和 CubeMX 配置为准。

## 板上外设与配置状态

以下为当前 CubeMX 配置和仓库代码所体现的状态，不代表所有外设均已在当前应用中启用或完成实机验证：

| 外设或接口 | 当前配置与支持情况 |
| --- | --- |
| 主控 | STM32H723VGT6，Cortex-M7；CubeMX 配置系统时钟为 550 MHz。 |
| FDCAN1 / FDCAN3 | 配置为经典 CAN，1 Mbps；应用示例启动两条总线。 |
| FDCAN2 | 配置为 CAN FD + BRS，仲裁段 1 Mbps、数据段 5 Mbps；应用示例使用该总线控制达妙电机。 |
| BMI088 | 提供驱动；SPI2 配置为主机并使用收发 DMA，配置了加速度计和陀螺仪片选及中断引脚。 |
| W25Q Flash | 提供驱动；OCTOSPI2 配置为四线模式。 |
| 遥控器 | 提供 FlySky SBUS 接收支持；UART5 配置为 100000 baud、偶校验、2 位停止位，并配置接收 DMA。 |
| 串口调试 | UART9 配置为 921600 baud，收发均配置 DMA；当前示例用于 VOFA。UART7、USART1、USART3、USART10 也已配置。 |
| LED 与蜂鸣器 | 提供 WS2812 和 PWM 蜂鸣器组件；SPI6 配置为仅发送主机，TIM3 CH4 与 TIM12 CH2 配置为 PWM 输出。 |
| USB | CubeMX 配置了 USB Device CDC 和全速设备接口。 |

具体引脚、参数及启动方式以 [CubeMX 配置](Dm_02_Bsp_V2.ioc)、生成代码和各模块文档为准；电机总线细节见 [硬件接口说明](Hardware/README.md)。
## 环境准备

- Git（包含子模块支持）
- CMake 3.22 或更新版本
- Ninja
- Arm GNU Toolchain（`arm-none-eabi-gcc`、`arm-none-eabi-g++` 等命令需在 PATH 中）
- 修改外设配置时需要 STM32CubeMX；下载固件还需与硬件匹配的调试器和烧录工具

工程使用 C11 和 C++20；请使用支持 C++20 的 Arm 工具链。

## 获取与构建

克隆时初始化 Fusion 和 Boost.PFR 子模块：

```bash
git clone --recurse-submodules <仓库地址>
cd Sentory
```

已有工作区可执行：

```bash
git submodule update --init --recursive
```

在项目根目录使用预设构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

发布构建可将上述两处 `Debug` 替换为 `Release`。生成的 ELF 和 map 文件位于对应的 `build/Debug/` 或 `build/Release/` 目录，可使用调试器加载 ELF 或按烧录工具的要求转换格式。构建预设及工具链配置分别位于 `CMakePresets.json` 和 `cmake/gcc-arm-none-eabi.cmake`。

## 工程结构

```text
Application/     角色应用、主循环与定时回调
Hardware/        电机、传感器、板级接口及其他硬件驱动
Service/         启动配置、板间通信、底盘和功率限制等组件
Tools/           控制算法及 Fusion、Boost.PFR 子模块
Core/            CubeMX 生成的核心代码
Drivers/         STM32 HAL、CMSIS 等驱动
Middlewares/     STM32 中间件
USB_DEVICE/      USB 设备代码
cmake/           工具链与 CubeMX 构建配置
Dm_02_Bsp_V2.ioc CubeMX 工程配置
```

## 使用与调试

应用入口位于 [Application.cpp](Application/Application.cpp)。启动时固件从 W25Q Flash 的双槽配置中读取板卡角色，选择底盘或云台应用；未知或无效角色不会启动角色应用。底盘和云台应用目前是联调骨架，主循环中的业务控制尚未完成，不应视作可直接上机的完整机器人程序。

底盘应用目前启动 FDCAN1/2/3，初始化 M3508 和只读观察模式的达妙电机，建立底盘与板间通信服务，并在 TIM7 回调中调用底盘、功率限制和电机发送流程。云台应用目前启动三条 FDCAN 总线并初始化板间通信服务；其主循环仍为空。实际控制目标、配置写入工具和设备初始化流程需按硬件与任务继续完善。

**上电前请确认板卡角色配置、应用逻辑、总线模式、电机使能、接线和供电；首次验证时应确保执行器不会意外动作。** 电机接口、帧格式、初始化顺序和实机验收要点见 [硬件接口说明](Hardware/README.md)。

外设配置可通过根目录的 `.ioc` 文件在 STM32CubeMX 中查看或修改。重新生成代码后，请检查生成结果和用户代码区，并重新构建验证。

## 许可证

本项目许可证见 [LICENSE](LICENSE)。第三方组件遵循各自的许可证，例如 [Fusion 许可证](Tools/ThirdParty/Fusion/LICENSE.md)。
