# 26_robot_code

基于 STM32H723VG 的机器人控制系统

## 项目简介

本项目是一款面向 RoboMaster 比赛的机器人控制器固件，基于 STM32H723VG 微控制器开发，运行 FreeRTOS 实时操作系统。系统集成了底盘控制、舵轮控制、云台控制等多种机器人运动控制功能，并提供了丰富的算法库支持。

## 硬件平台

- **MCU**: STM32H723VG (ARM Cortex-M7, 480MHz)
- **存储**: 512KB Flash, 320KB SRAM
- **通信接口**: 
  - FDCAN × 3 (CAN1/CAN2/CAN3)
  - UART × 4 (UART1/UART2/UART7/USART10)
  - SPI × 3 (SPI1/SPI2/SPI6)
  - USB OTG HS
- **ADC**: 1 路 ADC (支持 DMA)
- **定时器**: TIM1/TIM3/TIM12 (PWM 输出)

## 项目结构

```
26_robot_code/
├── Core/                      # STM32 HAL 驱动库
│   ├── Inc/                   # 头文件
│   └── Src/                   # 源文件
├── Drivers/                   # 外设驱动
│   ├── CMSIS/                 # ARM CMSIS 核心
│   └── STM32H7xx_HAL_Driver/ # ST HAL 驱动
├── Middlewares/               # 中间件
│   ├── ST/                    # ST 官方中间件
│   │   ├── ARM/DSP/          # DSP 库
│   │   ├── FreeRTOS/         # FreeRTOS
│   │   └── USB_Device_Library/# USB 设备库
│   └── Third_Party/          # 第三方库
├── User/                     # 用户代码
│   ├── APP/                  # 应用层
│   │   ├── chassis/          # 底盘控制
│   │   ├── gimbal/          # 云台控制
│   │   └── steering_wheel/   # 舵轮控制
│   ├── Drive/                # 驱动层
│   │   ├── algorithm/         # 算法库
│   │   │   ├── basic_math/  # 基础数学
│   │   │   ├── controller/  # 控制器 (PID/Fuzzy)
│   │   │   ├── crc/         # CRC 校验
│   │   │   ├── filter/      # 滤波器 (Kalman/EKF)
│   │   │   ├── fsm/         # 状态机
│   │   │   ├── matrix/      # 矩阵运算
│   │   │   ├── pid/         # PID 控制
│   │   │   ├── quaternion/  # 四元数
│   │   │   ├── queue/       # 队列
│   │   │   ├── slope_math/  # 斜坡函数
│   │   │   ├── user_lib/    # 工具库
│   │   │   └── wave_generator/# 波形生成
│   │   ├── brush_motor/     # 有刷电机驱动
│   │   │   ├── dji_motor/  # 大疆电机
│   │   │   ├── dji_motor_new/ # 大疆电机 (新版)
│   │   │   └── dm_motor/   # DM 电机
│   │   ├── buzzer/          # 蜂鸣器
│   │   ├── direct_motor/    # 直驱电机
│   │   │   ├── a4950_drive/# A4950 驱动
│   │   │   ├── mks_drive/  # MKS 驱动
│   │   │   └── servo_drive/# 舵机驱动
│   │   ├── grayscale/       # 灰度传感器
│   │   ├── imu/             # IMU 传感器
│   │   │   ├── hi12_imu/   # HI12 IMU
│   │   │   ├── mspm0_imu/   # MSPM0 IMU
│   │   │   └── wit_imu/    # WIT IMU
│   │   ├── key/             # 按键驱动
│   │   ├── odometer/        # 里程计
│   │   ├── remote_control/  # 遥控器
│   │   │   ├── crsf/        # CRSF 协议
│   │   │   └── remote_center/# 遥控中心
│   │   ├── seggee_rtt/      # RTT 调试
│   │   ├── wheel_system_solution/# 轮子系统方案
│   │   └── ws2812/         # RGB LED
│   ├── bsp/                 # 板级支持包
│   │   ├── bsp_adc/         # ADC
│   │   ├── bsp_can/         # CAN 总线
│   │   ├── bsp_dwt/         # DWT 时钟
│   │   ├── bsp_i2c/         # I2C
│   │   ├── bsp_iwdg/       # 看门狗
│   │   ├── bsp_log/         # 日志
│   │   ├── bsp_mcu/         # MCU
│   │   ├── bsp_pwm/         # PWM
│   │   ├── bsp_spi/         # SPI
│   │   ├── bsp_uart/        # UART
│   │   └── bsp_usb/         # USB
│   └── test/                # 测试代码
├── USB_DEVICE/               # USB 设备配置
└── CMakeLists.txt            # 构建配置
```

## 功能特性

### 底盘控制 (Chassis)
- 多种运动模式：锁定模式、手动模式、自动模式
- 支持遥控器控制和自主导航
- 麦轮/舵轮里程计算
- 电机 PID 控制

### 舵轮控制 (Steering Wheel)
- 独立舵轮控制
- 手动模式操作
- 电机闭环控制

### 云台控制 (Gimbal)
- 角度控制
- 电机速度环/位置环控制

### 通信协议
- **FDCAN**: 支持 DJI 电机、DM 电机、MKS 电机
- **CRSF**: 支持 ELRS 遥控器
- **USB CDC**: 虚拟串口调试

### 算法库
- PID 控制器 (支持前馈、模糊 PID)
- 卡尔曼滤波 (Kalman Filter)
- 扩展卡尔曼滤波 (EKF)
- 矩阵运算
- 四元数运算
- CRC 校验 (CRC8/CRC16)
- 状态机框架

## 任务调度

| 任务名 | 优先级 | 说明 |
|--------|--------|------|
| defaultTask | 默认 | 系统初始化任务 |
| chassis_task | 底盘 | 底盘控制任务 |
| remote_control_task | 遥控 | 遥控器数据处理 |
| steering_wheel_task | 舵轮 | 舵轮控制任务 |
| test_task | 测试 | 测试调试任务 |

## 编译环境

- **工具链**: GCC ARM Embedded (arm-none-eabi-gcc)
- **构建系统**: CMake
- **IDE**: CLion / VSCode (推荐)

### 编译步骤

```bash
# 创建构建目录
mkdir build && cd build

# 配置项目
cmake ..

# 编译
make
```

## 配置说明

### FDCAN 引脚配置
- CAN1: FDCAN1_IT0, FDCAN1_IT1
- CAN2: FDCAN2_IT0, FDCAN2_IT1
- CAN3: FDCAN3_IT0, FDCAN3_IT1

### 电机配置
- 支持 DJI GM3508/GM2006/GM6020 电机
- 支持 DM3519 电机
- 支持 MKS 步进电机

### IMU 配置
- HI12 系列
- MSPM0 系列
- WIT 系列

## 调试

### RTT 调试
使用 SEGGER RTT 进行实时调试输出：

```c
// 使用方式
SEGGER_RTT_printf(0, "Value: %d\n", value);
```

### USB 虚拟串口
通过 USB CDC 接口进行调试数据收发。

## 依赖库

- STM32H7xx HAL
- FreeRTOS v10+
- CMSIS DSP
- USB Device Library

## 注意事项

1. 确保 CAN 总线终端电阻正确配置 (120Ω)
2. 电机电调需要正确使能后再进行控制
3. IMU 传感器需要根据实际型号配置通信参数
4. 使用看门狗防止系统死机

## 贡献指南

欢迎提交 Issue 和 Pull Request。

## 许可证

MIT License