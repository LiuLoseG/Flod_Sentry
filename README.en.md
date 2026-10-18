# 26_robot_code

Robot Control System Based on STM32H723VG

## Project Overview

This project is a firmware for a robot controller designed for the RoboMaster competition, developed on the STM32H723VG microcontroller and running the FreeRTOS real-time operating system. The system integrates various robotic motion control functions including chassis control, steering wheel control, and gimbal control, along with a rich library of algorithms.

## Hardware Platform

- **MCU**: STM32H723VG (ARM Cortex-M7, 480MHz)
- **Memory**: 512KB Flash, 320KB SRAM
- **Communication Interfaces**:
  - FDCAN × 3 (CAN1/CAN2/CAN3)
  - UART × 4 (UART1/UART2/UART7/USART10)
  - SPI × 3 (SPI1/SPI2/SPI6)
  - USB OTG HS
- **ADC**: 1 ADC channel (with DMA support)
- **Timers**: TIM1/TIM3/TIM12 (PWM output)

## Project Structure

```
26_robot_code/
├── Core/                      # STM32 HAL Driver Library
│   ├── Inc/                   # Header Files
│   └── Src/                   # Source Files
├── Drivers/                   # Peripheral Drivers
│   ├── CMSIS/                 # ARM CMSIS Core
│   └── STM32H7xx_HAL_Driver/ # ST HAL Driver
├── Middlewares/               # Middleware
│   ├── ST/                    # ST Official Middleware
│   │   ├── ARM/DSP/          # DSP Library
│   │   ├── FreeRTOS/         # FreeRTOS
│   │   └── USB_Device_Library/# USB Device Library
│   └── Third_Party/          # Third-Party Libraries
├── User/                     # User Code
│   ├── APP/                  # Application Layer
│   │   ├── chassis/          # Chassis Control
│   │   ├── gimbal/          # Gimbal Control
│   │   └── steering_wheel/   # Steering Wheel Control
│   ├── Drive/                # Driver Layer
│   │   ├── algorithm/         # Algorithm Library
│   │   │   ├── basic_math/  # Basic Mathematics
│   │   │   ├── controller/  # Controllers (PID/Fuzzy)
│   │   │   ├── crc/         # CRC Checksum
│   │   │   ├── filter/      # Filters (Kalman/EKF)
│   │   │   ├── fsm/         # Finite State Machine
│   │   │   ├── matrix/      # Matrix Operations
│   │   │   ├── pid/         # PID Control
│   │   │   ├── quaternion/  # Quaternion Operations
│   │   │   ├── queue/       # Queue
│   │   │   ├── slope_math/  # Ramp Function
│   │   │   ├── user_lib/    # Utility Library
│   │   │   └── wave_generator/# Waveform Generator
│   │   ├── brush_motor/     # Brush Motor Driver
│   │   │   ├── dji_motor/  # DJI Motor
│   │   │   ├── dji_motor_new/ # DJI Motor (New Version)
│   │   │   └── dm_motor/   # DM Motor
│   │   ├── buzzer/          # Buzzer
│   │   ├── direct_motor/    # Direct Drive Motor
│   │   │   ├── a4950_drive/# A4950 Driver
│   │   │   ├── mks_drive/  # MKS Driver
│   │   │   └── servo_drive/# Servo Driver
│   │   ├── grayscale/       # Grayscale Sensor
│   │   ├── imu/             # IMU Sensor
│   │   │   ├── hi12_imu/   # HI12 IMU
│   │   │   ├── mspm0_imu/   # MSPM0 IMU
│   │   │   └── wit_imu/    # WIT IMU
│   │   ├── key/             # Key Driver
│   │   ├── odometer/        # Odometry
│   │   ├── remote_control/  # Remote Controller
│   │   │   ├── crsf/        # CRSF Protocol
│   │   │   └── remote_center/# Remote Center
│   │   ├── seggee_rtt/      # RTT Debugging
│   │   ├── wheel_system_solution/# Wheel System Solution
│   │   └── ws2812/         # RGB LED
│   ├── bsp/                 # Board Support Package
│   │   ├── bsp_adc/         # ADC
│   │   ├── bsp_can/         # CAN Bus
│   │   ├── bsp_dwt/         # DWT Timer
│   │   ├── bsp_i2c/         # I2C
│   │   ├── bsp_iwdg/       # Watchdog
│   │   ├── bsp_log/         # Logging
│   │   ├── bsp_mcu/         # MCU
│   │   ├── bsp_pwm/         # PWM
│   │   ├── bsp_spi/         # SPI
│   │   ├── bsp_uart/        # UART
│   │   └── bsp_usb/         # USB
│   └── test/                # Test Code
├── USB_DEVICE/               # USB Device Configuration
└── CMakeLists.txt            # Build Configuration
```

## Features

### Chassis Control
- Multiple motion modes: Lock mode, Manual mode, Auto mode
- Supports remote control and autonomous navigation
- Omni-wheel/steering-wheel odometry calculation
- Motor PID control

### Steering Wheel Control
- Independent steering wheel control
- Manual operation mode
- Closed-loop motor control

### Gimbal Control
- Angle control
- Motor speed loop / position loop control

### Communication Protocols
- **FDCAN**: Supports DJI motors, DM motors, MKS motors
- **CRSF**: Supports ELRS remote controller
- **USB CDC**: Virtual serial port debugging

### Algorithm Library
- PID Controller (with feedforward and fuzzy PID support)
- Kalman Filter
- Extended Kalman Filter (EKF)
- Matrix Operations
- Quaternion Operations
- CRC Checksum (CRC8/CRC16)
- Finite State Machine Framework

## Task Scheduling

| Task Name | Priority | Description |
|-----------|----------|-------------|
| defaultTask | Default | System initialization task |
| chassis_task | Chassis | Chassis control task |
| remote_control_task | Remote | Remote controller data processing |
| steering_wheel_task | Steering | Steering wheel control task |
| test_task | Test | Debugging and testing task |

## Build Environment

- **Toolchain**: GCC ARM Embedded (arm-none-eabi-gcc)
- **Build System**: CMake
- **IDE**: CLion / VSCode (Recommended)

### Build Steps

```bash
# Create build directory
mkdir build && cd build

# Configure project
cmake ..

# Compile
make
```

## Configuration Notes

### FDCAN Pin Configuration
- CAN1: FDCAN1_IT0, FDCAN1_IT1
- CAN2: FDCAN2_IT0, FDCAN2_IT1
- CAN3: FDCAN3_IT0, FDCAN3_IT1

### Motor Configuration
- Supports DJI GM3508/GM2006/GM6020 motors
- Supports DM3519 motors
- Supports MKS stepper motors

### IMU Configuration
- HI12 Series
- MSPM0 Series
- WIT Series

## Debugging

### RTT Debugging
Use SEGGER RTT for real-time debug output:

```c
// Usage
SEGGER_RTT_printf(0, "Value: %d\n", value);
```

### USB Virtual Serial Port
Debug data transmission and reception via USB CDC interface.

## Dependencies

- STM32H7xx HAL
- FreeRTOS v10+
- CMSIS DSP
- USB Device Library

## Notes

1. Ensure correct termination resistor configuration on CAN bus (120Ω).
2. Enable motor ESCs properly before control.
3. Configure communication parameters for IMU sensors according to actual model.
4. Use watchdog timer to prevent system lockup.

## Contribution Guidelines

Issues and pull requests are welcome.

## License

MIT License