/**
 * @file imu_temp_ctrl.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-11-23
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// IMU 零漂标定宏
#ifndef IMU_GYRO_AUTO_CALIB_ENABLE
#define IMU_GYRO_AUTO_CALIB_ENABLE 0
#endif

/**
 * @brief IMU 工作状态
 */
typedef enum
{
    Imu_Status_Uninit = 0,  // 未初始化
    Imu_Status_Init_Fail,   // BMI088 初始化失败, 任务内周期性重试
    Imu_Status_Wait_Temp,   // 等待加热到目标温度(仅自标定模式)
    Imu_Status_Calibrating, // 陀螺零漂标定中
    Imu_Status_Running,     // 正常解算
} imu_status_e;

typedef enum
{
    VQF = 0,    // VQF 算法
    MAHONY,     // Mahony算法
    EKF         // EKF 算法
} IMU_Fusion_Algo_e;

typedef struct
{
    uint8_t temp_reached;      // 到达目标温度
    uint8_t temp_stable;       // 温度稳定
    uint8_t gyro_calib_done;   // 陀螺仪零漂完成
    uint8_t fusion_enabled;    // 融合算法输出可用
} imu_ctrl_flag_t;

/**
 * @brief IMU 解算结果
 */
typedef struct
{
    // 解算输出
    float gyro[3];              // 已扣除零漂的角速度, rad/s
    float accel[3];             // 加速度, m/s^2; 解算状态下已做偏心补偿(折算到旋转中心)
    float temperature;          // 当前温度, ℃
    float q[4];                  // 姿态四元数 [w, x, y, z]
    float pitch;                 // 俯仰角, rad
    float roll;                  // 横滚角, rad
    float yaw;                   // 偏航角, rad
    float YawTotalAngle;         // yaw 连续多圈角(不折叠到 ±pi), rad
    float dt;                   // 本拍姿态积分步长, s(累计排空后的实际积分时间)

    // 温控
    float ref_temperature;      // 目标温度, ℃
    float heater_out;           // 加热输出, 与 TIM3 ARR 同量纲(0 ~ ARR)

    // 状态
    imu_status_e status;        // 工作状态
    uint8_t data_valid;         // 1 = 本周期传感器数据有效
    uint32_t sample_count;      // 采样帧计数(温控分频基准)
} imu_data_resolve_t;

extern volatile imu_data_resolve_t imu_frame;

float imu_get_roll_rad(void);
float imu_get_pitch_rad(void);
float imu_get_yaw_rad(void);
float imu_get_yaw_total_rad(void);

/**
 * @brief 姿态是否可用(传感器在线、已进入解算状态且融合器正常产出)
 * @return 1 可用, 0 不可用
 */
uint8_t imu_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_TEMP_CTRL_H
