# 09 Hardware

收录嵌入式计算、电机与驱动、传感器、总线、供电和机器人硬件接口资料。

## Official and maintainer documentation

| 资源 | 提供方 | 主题 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [Arduino Documentation](https://docs.arduino.cc/) | Arduino | 开发板、IDE、CLI、语言参考和官方教程 | English | Beginner | Source Checked | 2026-08-31 |
| [STM32 Step by Step](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:STM32_step_by_step_overview) | STMicroelectronics | STM32Cube 工具、GPIO、外设和调试入门 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) | STMicroelectronics | IDE、项目生成、编译和调试资料入口 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/computers/) | Raspberry Pi | 系统、GPIO、相机、远程访问和计算模块 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [NVIDIA Jetson Software Documentation](https://docs.nvidia.com/jetson/index.html) | NVIDIA | JetPack、Jetson Linux、API 和版本归档 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [ODrive Documentation](https://docs.odriverobotics.com/v/latest/index.html) | ODrive Robotics | BLDC 驱动、编码器、调参、CAN 和 ROS 2 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [SimpleFOC Getting Started](https://docs.simplefoc.com/example_from_scratch) | SimpleFOC project | 传感器、驱动器、开环与闭环 FOC 测试 | English | Intermediate | Source Checked | 2026-08-31 |
| [SocketCAN Documentation](https://docs.kernel.org/networking/can.html) | Linux kernel documentation | Linux CAN 网络栈和 socket API | English | Advanced | Source Checked | 2026-08-31 |
| [micro-ROS Documentation](https://micro.ros.org/docs/) | micro-ROS project | MCU 上的 ROS 2 客户端、RTOS 与 Agent | English | Intermediate–Advanced | Source Checked | 2026-08-31 |

## Safety notes

- 电机、电池和电源测试应先做限流、急停、保险和机械固定；软件限位不能替代硬件保护。
- 同名开发板或驱动器可能有不同硬件修订版，接线前以具体型号的原理图和数据手册为准。
- 真实机器人控制需要记录固件、参数、齿比、编码器方向、零位和单位；不要只保存上位机代码。
- 厂商的 `latest` 文档可能不适用于旧硬件或旧固件，必要时使用版本归档。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
