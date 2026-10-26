/**
 * @file imu_temp_ctrl.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 板载 BMI088 IMU 的 DRDY 同步采样、恒温控制与姿态解算对外接口
 * @note  1) 对外单位: 角度 rad, 角速度 rad/s, 加速度 m/s^2, 温度 ℃ (见 RoboMaster电控C++开发规范 §5.1)
 *        2) 坐标系: 机体 X=前 Y=左 Z=上(右手系), yaw 绕 Z 逆时针为正, 角度自 +X 起算
 *        3) imu_frame 由 IMU 任务写、其余任务读, 单个 float 字段的读写是原子的(32 位对齐);
 *           需要多字段严格一致时应自行关中断拷贝
 * @date 2026-10-26
 * @version 0.7
 * @copyright HNAU-Robot (c) 2026
 */
#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 上电陀螺零漂自标定开关
 *        1 = 加热到目标温度并稳定后采集 IMU_GYRO_CALIB_SAMPLES 帧求均值作为零漂(需上车验证后启用)
 *        0 = 直接使用 gyro_bias[] 中的实测补偿值(默认, 与旧版 IMU_CHECK_ENABLE = 0 行为一致)
 * @note  也可用 target_compile_definitions 覆盖, 便于分板/分车配置
 */
#ifndef IMU_GYRO_AUTO_CALIB_ENABLE
#define IMU_GYRO_AUTO_CALIB_ENABLE 0
#endif

/**
 * @brief IMU 工作状态
 */
typedef enum
{
    Imu_Status_Uninit = 0,  //!< 未初始化
    Imu_Status_Init_Fail,   //!< BMI088 初始化失败, 任务内周期性重试
    Imu_Status_Wait_Temp,   //!< 等待加热到目标温度(仅自标定模式)
    Imu_Status_Calibrating, //!< 陀螺零漂标定中
    Imu_Status_Running,     //!< 正常解算
} imu_status_e;

/**
 * @brief 欧拉角, 单位 rad
 */
typedef struct
{
    float roll;
    float pitch;
    float yaw; 
} euler_angle_t;

/**
 * @brief IMU 解算结果
 */
typedef struct
{
    // 解算输出
    float gyro[3];              // 已扣除零漂的角速度, rad/s
    float accel[3];             // 加速度, m/s^2
    float temperature;          // 当前温度, ℃
    euler_angle_t euler_angle;  // 欧拉角, rad
    float yaw_total;            // yaw 连续多圈角(不折叠到 ±pi), rad
    float dt;                   // 实测采样周期, s

    // 温控
    float ref_temperature;      // 目标温度, ℃
    float heater_out;           // 加热输出, 与 TIM3 ARR 同量纲(0 ~ ARR)

    // 状态
    imu_status_e status;        // imu_status_e
    uint8_t data_valid;         // 1 = 本周期传感器数据有效
    uint32_t sample_count;      // 采样帧计数(温控分频基准)
} imu_data_resolve_t;

extern volatile imu_data_resolve_t imu_frame;

void imu_task(void *argument);

float imu_get_roll_rad(void);
float imu_get_pitch_rad(void);
float imu_get_yaw_rad(void);
float imu_get_yaw_total_rad(void);

/**
 * @brief 姿态是否可用(传感器在线且已进入解算状态)
 * @return 1 可用, 0 不可用
 */
uint8_t imu_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_TEMP_CTRL_H
