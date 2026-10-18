#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include "pid_fuse.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CHECK_ENABLE 0

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} euler_angle_t;    //欧拉角结构体

typedef struct 
{
    float gyro[3];
    float accel[3];
    float temp;
    float ref_temp;
    euler_angle_t euler_angle;
    pid_t temperature_pid;
} imu_data_resolve_t;

extern volatile imu_data_resolve_t imu_frame;

void IMU_task(void * argument);

#ifdef __cplusplus
}
#endif

#endif // IMU_TEMP_CTRL_H
