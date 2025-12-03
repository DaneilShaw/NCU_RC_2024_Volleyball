# NCU南昌大学机器人队 Robocon2024排球挑战赛代码

项目包含机器人的底盘控制和 Delta 机械臂控制两大部分，基于大疆A板，使用 FreeRTOS 实时操作系统。

## 项目结构 (Project Structure)

本仓库主要包含两个工程目录：

### 1. RC24_Volleyball_chassis (底盘控制)
负责机器人的移动、定位以及部分击球/垫球逻辑的协调。
- **路径**: `RC24_Volleyball_chassis/`
- **主要任务 (User Tasks)**:
    - `ChassisDrive_Task`: 底盘全向移动控制。
    - `Vision_Task`: 视觉数据处理与目标跟踪。
    - `AppRemote_Task`: 移动端 APP 遥控通信。
    - `HitBall_Task`: 击球动作控制。
    - `CushionBall_Task`: 垫球动作控制。
    - `OffLine_Check_Task`: 模块离线检测与安全保护。

### 2. RC24_Volleyball_Delta_can_control (Delta 机械臂控制)
负责 Delta 并联机械臂的运动控制，用于精准的击球和发球操作。
- **路径**: `RC24_Volleyball_Delta_can_control/`
- **主要任务 (User Tasks)**:
    - `ArmDrive_Task`: 机械臂运动学解算与驱动。
    - `Serve_Task`: 发球控制。
    - `HitBall_Task`: 击球控制。
    - `Vision_Task`: 视觉信号处理。

## 开发环境 (Development Environment)

- **微控制器**: STM32F4 系列 (STM32F4xx)
- **IDE**: Keil MDK-ARM
- **配置工具**: STM32CubeMX (工程包含 `.ioc` 文件)
- **操作系统**: FreeRTOS
- **驱动库**: STM32 HAL Driver
