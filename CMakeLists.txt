cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME infantry)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Build CMSIS-DSP for the Cortex-M target. FASTBUILD uses the grouped source
# files provided by CMSIS-DSP and avoids compiling every function separately.
set(FASTBUILD ON CACHE BOOL "Build CMSIS-DSP using grouped Cortex-M sources" FORCE)
add_subdirectory(
    Middlewares/Third_Party/DSP/Source
    ${CMAKE_BINARY_DIR}/cmsis-dsp
)
target_link_libraries(CMSISDSP PUBLIC stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/BSP/bsp_adc/bsp_adc.cpp
    User/BSP/bsp_can/bsp_can.cpp
    User/BSP/bsp_dwt/bsp_dwt.cpp
    User/BSP/bsp_iwdg/bsp_iwdg.cpp
    User/BSP/bsp_log/bsp_log.cpp
    User/BSP/bsp_spi/bsp_spi.cpp
    User/BSP/bsp_uart/bsp_uart.cpp
    User/BSP/bsp_usb/bsp_usb.cpp
    User/BSP/bsp_pwm/bsp_pwm.cpp
    User/BSP/bsp_mcu/bsp_mcu.cpp

    User/Drive/algorithm/basic_math/basic_math.cpp
    User/Drive/algorithm/complex/complex.cpp
    User/Drive/algorithm/controller/controller.cpp
    User/Drive/algorithm/crc/crc8_crc16/crc8_crc16.cpp
    User/Drive/algorithm/crc/crc8/crc8.cpp
    User/Drive/algorithm/crc/crc16/crc16.cpp

    User/Drive/algorithm/feedforward/feedforward.cpp
    User/Drive/algorithm/filter/filter_ekf/filter_ekf.cpp
    User/Drive/algorithm/filter/frequency_filter/frequency_filter.cpp
    User/Drive/algorithm/filter/general_filter/general_filter.cpp
    User/Drive/algorithm/filter/kalman_last/kalman_last.cpp
    User/Drive/algorithm/filter/kalman_new/kalman_new.cpp


    User/Drive/algorithm/fsm/fsm.cpp
    User/Drive/algorithm/MahonyAHRS/MahonyAHRS.cpp
    User/Drive/algorithm/matrix/math_matrix.cpp
    User/Drive/algorithm/QuaternionEKF/QuaternionEKF.cpp
    User/Drive/algorithm/quaternion/quaternion.cpp
    User/Drive/algorithm/queue/queue.cpp
    User/Drive/algorithm/slope_math/slope_math.cpp
    User/Drive/algorithm/pid/pid.cpp
    User/Drive/algorithm/pid_fuse/pid_fuse.cpp
    User/Drive/algorithm/user_lib/user_lib.cpp
    User/Drive/algorithm/wave_generator/wave_generator.cpp

    User/Drive/bmi088/BMI088driver.cpp
    User/Drive/bmi088/BMI088Middleware.cpp
    User/Drive/buzzer/buzzer.cpp

    User/Drive/direct_motor/a4950_drive/a4950_drive.cpp
    User/Drive/direct_motor/tb6612_drive/tb6612_drive.cpp
    User/Drive/direct_motor/servo_drive/servo_drive.cpp
    User/Drive/direct_motor/mks_drive/mks_drive.cpp

    User/Drive/brush_motor/dji_motor/dji_motor.cpp
    User/Drive/brush_motor/dji_motor_new/dji_motor_new.cpp
    User/Drive/brush_motor/dm_motor/dm_motor.cpp
    User/Drive/brush_motor/lk_motor/lk_motor.cpp

    User/Drive/data_distribution_center/can_board_communication/can_board_communication.cpp
    User/Drive/data_distribution_center/can_distribution/can_distribution.cpp
    User/Drive/data_distribution_center/pc_communication/pc_communication.cpp

    User/Drive/debug_tool/usb_vofa/usb_vofa.cpp
    User/Drive/debug_tool/uart_vofa/uart_vofa.cpp
    User/Drive/seggee_rtt/SEGGER_RTT.c
    User/Drive/seggee_rtt/SEGGER_RTT_printf.c

    User/Drive/sys_identificate/sys_phased_freq/sys_phased_freq.cpp
    User/Drive/sys_identificate/sys_sweep_freq/sys_sweep_freq.cpp

    User/Drive/grayscale/grayscale.cpp
    User/Drive/imu/wit_imu/wit_imu.cpp
    User/Drive/imu/hi12_imu/hi12_imu.cpp
    User/Drive/imu/mspm0_imu/mspm0_imu.cpp

    User/Drive/key/adc_key/adc_key.cpp
    User/Drive/key/regular_key/regular_key.cpp

    User/Drive/lcd_oled/st7789_73x219_lcd/st7789_73x219_lcd.cpp

    User/Drive/wheel_system_solution/wheel_system_solution.cpp

    User/Drive/odometer/imu_odometer/imu_odometer.cpp
    User/Drive/odometer/fusion_odometer/fusion_odometer.cpp
    User/Drive/odometer/ops_9/ops_9.cpp

    User/Drive/remote_control/crsf/crsf.cpp
    User/Drive/remote_control/remote_center/remote_center.cpp

    User/Drive/ws2812/ws2812.cpp

    User/test/test.cpp
    # User/APP/chassis/chassis.cpp
    # User/APP/steering_wheel/steering_wheel.cpp
    User/APP/gimbal/gimbal.cpp
    User/APP/shoot/shoot.cpp
    User/APP/robot_decision/robot_decision.cpp
    User/APP/safe_daemon_process/safe_daemon_process.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/BSP/bsp_adc
    User/BSP/bsp_can
    User/BSP/bsp_dwt
    User/BSP/bsp_log
    User/BSP/bsp_iwdg
    User/BSP/bsp_spi
    User/BSP/bsp_uart
    User/BSP/bsp_usb
    User/BSP/bsp_pwm
    User/BSP/bsp_mcu

    User/Drive/algorithm/basic_math
    User/Drive/algorithm/complex
    User/Drive/algorithm/controller
    User/Drive/algorithm/crc/crc8_crc16
    User/Drive/algorithm/crc/crc8
    User/Drive/algorithm/crc/crc16

    User/Drive/algorithm/feedforward
    User/Drive/algorithm/filter/filter_ekf
    User/Drive/algorithm/filter/frequency_filter
    User/Drive/algorithm/filter/general_filter
    User/Drive/algorithm/filter/kalman_last
    User/Drive/algorithm/filter/kalman_new

    User/Drive/algorithm/fsm
    User/Drive/algorithm/MahonyAHRS
    User/Drive/algorithm/QuaternionEKF
    User/Drive/algorithm/quaternion
    User/Drive/algorithm/queue
    User/Drive/algorithm/slope_math
    User/Drive/algorithm/matrix
    User/Drive/algorithm/rls
    User/Drive/algorithm/pid
    User/Drive/algorithm/pid_fuse
    User/Drive/algorithm/user_lib
    User/Drive/algorithm/wave_generator

    User/Drive/bmi088
    User/Drive/buzzer

    User/Drive/brush_motor/dji_motor
    User/Drive/brush_motor/dji_motor_new
    User/Drive/brush_motor/dm_motor
    User/Drive/brush_motor/lk_motor

    User/Drive/data_distribution_center/can_board_communication
    User/Drive/data_distribution_center/can_distribution
    User/Drive/data_distribution_center/pc_communication

    User/Drive/debug_tool/usb_vofa
    User/Drive/debug_tool/uart_vofa
    User/Drive/seggee_rtt

    User/Drive/direct_motor/a4950_drive
    User/Drive/direct_motor/servo_drive
    User/Drive/direct_motor/mks_drive

    User/Drive/grayscale
    User/Drive/imu/wit_imu
    User/Drive/imu/hi12_imu
    User/Drive/imu/mspm0_imu

    User/Drive/key/adc_key
    User/Drive/key/regular_key

    User/Drive/lcd_oled/st7789_73x219_lcd

    User/Drive/odometer/imu_odometer
    User/Drive/odometer/fusion_odometer
    User/Drive/odometer/ops_9

    User/Drive/remote_control/crsf
    User/Drive/remote_control/remote_center

    User/Drive/sys_identificate/sys_phased_freq
    User/Drive/sys_identificate/sys_sweep_freq

    User/Drive/wheel_system_solution

    User/Drive/ws2812

    User/test
    User/APP
    # User/APP/chassis
    # User/APP/steering_wheel
    User/APP/gimbal
    User/APP/shoot
    User/APP/robot_decision
    User/APP/safe_daemon_process
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP

    # Add user defined libraries
)
