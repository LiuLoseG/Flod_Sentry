#include "wheel_system_solution.h"
#include <cmath>
#include <cstdint>
#include "arm_math.h"
#include "dsp/fast_math_functions.h"
#include "user_lib.h"


/*-------------------------------------------------------------------------omni_wheel-----------------------------------------------------------------------------------------------------------*/

#if WHEEL_TYPE == 0 // 全向轮运动学解算函数

/**
    * @brief 全向轮逆运动学解算函数
    * @param wheel_goal 轮电机目标结构体指针，包含轮子速度设置值
    * @param vx_set 机器人在X轴方向的速度，单位m/s
    * @param vy_set 机器人在Y轴方向的速度，单位m/s
    * @param wz 机器人绕Z轴的角速度，单位rad/s
*/
void omni_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx_set, float vy_set, float wz)
{
    wheel_status->wheel_speed[0] = -(SQRT_2_2 * (-vx_set + vy_set) + wz * OMNI_WHEEL_DISTANCE_R) / WHEEL_RADIUS_S; // 轮子1速度计算
    wheel_status->wheel_speed[1] = -(SQRT_2_2 * (-vx_set - vy_set) + wz * OMNI_WHEEL_DISTANCE_R) / WHEEL_RADIUS_S; // 轮子2速度计算
    wheel_status->wheel_speed[2] = -(SQRT_2_2 * (vx_set - vy_set)  + wz * OMNI_WHEEL_DISTANCE_R) / WHEEL_RADIUS_S; // 轮子3速度计算
    wheel_status->wheel_speed[3] = -(SQRT_2_2 * (vx_set + vy_set)  + wz * OMNI_WHEEL_DISTANCE_R) / WHEEL_RADIUS_S; // 轮子4速度计算
}

/**
    * @brief 全向轮正运动学解算函数
    * @param robot_speed 机器人状态结构体指针，包含机器人在X轴方向的速度、Y轴方向的速度和绕Z轴的角速度
    * @param wheel_speed 轮子速度数组，包含4个轮子的速度值，单位m/s
 */
void omni_wheel_speed_soluction(wheel_robot_speed_t *robot_speed, float *wheel_speed)
{
    float ws[4];
    ws[0] = -wheel_speed[0];
    ws[1] = -wheel_speed[1];
    ws[2] = -wheel_speed[2];
    ws[3] = -wheel_speed[3];

    // robot_speed->vx_set = (SQRT_2_4 * (-wheel_speed[0] - wheel_speed[1] + wheel_speed[2] + wheel_speed[3])) * WHEEL_RADIUS_S; 
    // robot_speed->vy_set = (SQRT_2_4 * (wheel_speed[0] - wheel_speed[1] - wheel_speed[2] + wheel_speed[3])) * WHEEL_RADIUS_S;
    // robot_speed->wz = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / OMNI_WHEEL_DISTANCE_R * WHEEL_RADIUS_S;

    robot_speed->vx_set = (SQRT_2_4 * (-ws[0] - ws[1] + ws[2] + ws[3])) * WHEEL_RADIUS_S; 
    robot_speed->vy_set = (SQRT_2_4 * (ws[0] - ws[1] - ws[2] + ws[3])) * WHEEL_RADIUS_S;
    robot_speed->wz = (ws[0] + ws[1] + ws[2] + ws[3]) / OMNI_WHEEL_DISTANCE_R * WHEEL_RADIUS_S;
}

/*-------------------------------------------------------------------------mecanum_wheel-----------------------------------------------------------------------------------------------------------*/

#elif WHEEL_TYPE == 1 // 麦轮运动学解算函数
/**
    * @brief 麦轮逆运动学解算函数
    * @param wheel_goal 轮电机目标结构体指针，包含轮子速度设置值
    * @param vx_set 机器人在X轴方向的速度，单位m/s
    * @param vy_set 机器人在Y轴方向的速度，单位m/s
    * @param vw_set 机器人绕Z轴的角速度，单位rad/s
    * @note  外八辊子布局, 电机: [0]=FL前左, [1]=RL后左, [2]=RR后右, [3]=FR前右
    *        辊子: FL\ RL/ RR\ FR/ (小口→车内, 大口→车外)
*/
void mecanum_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx_set, float vy_set, float vw_set)
{
    wheel_status->wheel_speed[0] = ( vx_set - vy_set - vw_set * (MECANUM_WHEEL_PROJECTION_X + MECANUM_WHEEL_PROJECTION_Y)) / WHEEL_RADIUS_S; // FL
    wheel_status->wheel_speed[1] = ( vx_set + vy_set - vw_set * (MECANUM_WHEEL_PROJECTION_X + MECANUM_WHEEL_PROJECTION_Y)) / WHEEL_RADIUS_S; // RL
    wheel_status->wheel_speed[2] = (-vx_set + vy_set - vw_set * (MECANUM_WHEEL_PROJECTION_X + MECANUM_WHEEL_PROJECTION_Y)) / WHEEL_RADIUS_S; // RR
    wheel_status->wheel_speed[3] = (-vx_set - vy_set - vw_set * (MECANUM_WHEEL_PROJECTION_X + MECANUM_WHEEL_PROJECTION_Y)) / WHEEL_RADIUS_S; // FR
}

/**
    * @brief 麦轮正运动学解算函数
    * @param robot_speed 机器人状态结构体指针，包含机器人在X轴方向的速度、Y轴方向的速度和绕Z轴的角速度
    * @param wheel_speed 轮子速度数组，包含4个轮子的速度值，单位m/s
 */
void mecanum_wheel_position_soluction(wheel_robot_speed_t *robot_speed, float *wheel_speed)
{
    robot_speed->vx_set = ( wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) * WHEEL_RADIUS_S / 4.0f; 
    robot_speed->vy_set = (-wheel_speed[0] + wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) * WHEEL_RADIUS_S / 4.0f;
    robot_speed->vw_set = -(wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / MECANUM_WHEEL_SUM_XY * WHEEL_RADIUS_S / 4.0f;
}

/*-------------------------------------------------------------------------steer_wheel-----------------------------------------------------------------------------------------------------------*/
#elif WHEEL_TYPE == 2 // 舵轮运动学解算函数

#ifndef STEER_TARGET_MAX_STEP_RAD
#define STEER_TARGET_MAX_STEP_RAD 0.20f
#endif

/**
 * @brief 选择舵向电机最近的等效目标，并决定轮向电机方向
 * @param angle 单圈目标角度，单位rad
 * @param reference 当前多圈角度，单位rad
 * @param direction 输出轮向电机方向，1或-1
 * @return 连续多圈舵向目标角度，单位rad
 */
static inline float steer_select_nearest_target(float angle, float reference, float *direction)
{
    float angle_error = basic_math_modulus_normalization(angle - reference, 2.0f * PI);
    *direction = 1.0f;

    // 轮子转向超过90度时，使用反向轮速的等效舵向角。
    if (fabsf(angle_error) > PI * 0.5f)
    {
        angle_error = basic_math_modulus_normalization(angle_error + PI, 2.0f * PI);
        *direction = -1.0f;
    }

    return reference + angle_error;
}

/**
 * @brief 限制多圈舵向目标单周期变化量
 * @param target 规划得到的目标角度，单位rad
 * @param current 当前反馈总角度，单位rad
 * @param previous 上一周期目标总角度，单位rad
 * @return 限速后的多圈目标角度，单位rad
 */
static inline float steer_limit_target_step(float target, float current, float previous)
{
    // 目标未初始化或已脱离当前角度太远时，从当前反馈角度重新接管。
    if ((previous != previous) || (fabsf(previous - current) > PI))
    {
        previous = current;
    }

    const float target_step = target - previous;

    if (target_step > STEER_TARGET_MAX_STEP_RAD)
    {
        return previous + STEER_TARGET_MAX_STEP_RAD;
    }

    if (target_step < -STEER_TARGET_MAX_STEP_RAD)
    {
        return previous - STEER_TARGET_MAX_STEP_RAD;
    }

    return target;
}

void steer_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx_set, float vy_set, float vw_set, uint8_t control_mode)
{
    if(wheel_status == NULL)
    {
        return; // wheel_status 为空指针，直接返回
    }

    if(!isfinite(vx_set) || !isfinite(vy_set) || !isfinite(vw_set) || !isfinite(control_mode))
    {
        return; // 有输入值不是有限数，直接返回
    }

    // 舵轮逆运动学解算函数实现
    // wheel_status->wheel_speed[0] = sqrtf(square(vx_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R) + square(vy_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R)) / WHEEL_RADIUS_S; // 轮子1速度计算
    // wheel_status->wheel_speed[1] = sqrtf(square(vx_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R) + square(vy_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R)) / WHEEL_RADIUS_S; // 轮子2速度计算
    // wheel_status->wheel_speed[2] = sqrtf(square(vx_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R) + square(vy_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R)) / WHEEL_RADIUS_S; // 轮子3速度计算
    // wheel_status->wheel_speed[3] = sqrtf(square(vx_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R) + square(vy_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R)) / WHEEL_RADIUS_S; // 轮子4速度计算

    // wheel_status->wheel_angle[0] = atan2f(vx_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R, vy_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R); // 轮子1转向角计算
    // wheel_status->wheel_angle[1] = atan2f(vx_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R, vy_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R); // 轮子2转向角计算
    // wheel_status->wheel_angle[2] = atan2f(vx_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R, vy_set - SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R); // 轮子3转向角计算
    // wheel_status->wheel_angle[3] = atan2f(vx_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R, vy_set + SQRT_2_2 * vw_set * STEER_WHEEL_DISTANCE_R); // 轮子4转向角计算

    // 静止判定(两种优化配置共用)
    const bool cmd_zero = (fabsf(vx_set) <= MIN_LIMIT) && (fabsf(vy_set) <= MIN_LIMIT) && (fabsf(vw_set) <= MIN_LIMIT);

#if COSINE_OPTIMIZE_ENABLE == 1 // 启动余弦优化
    
    // 舵轮逆运动学解算(直接解算目标角度和速度)
    for (uint8_t i = 0; i < 4; i++)
    {
        const float vel_x = vx_set - vw_set * arm_sin_f32(Wheel_Azimuth[i]) * Wheel_To_Core_Distance[i];
        const float vel_y = vy_set + vw_set * arm_cos_f32(Wheel_Azimuth[i]) * Wheel_To_Core_Distance[i];

        wheel_status->solution_speed[i] = sqrtf(vel_x * vel_x + vel_y * vel_y) / STEER_WHEEL_RADIUS; // 轮向电机目标角速度
        wheel_status->solution_angle[i] = atan2f(vel_y, vel_x);                                  // 舵向目标角, 0 对应 +X
    }

    // 舵向电机余弦优化计算
    for(uint8_t i = 0; i < 4; i++)
    {
        // 静止或无轮速(解算矢量模长为0)时锁定当前角度，防止舵轮无意义摆动。
        // 当前角度是多圈角度，目标角选择当前角附近的等效值，避免跨圈时位置环绕远路。
        float optimize_direction = 1.0f;
        const float target = (cmd_zero || wheel_status->solution_speed[i] <= 0.0f)
                             ? wheel_status->now_angle[i]
                             : steer_select_nearest_target(wheel_status->solution_angle[i] + Enc_Zero_Angle[i], wheel_status->now_angle[i], &optimize_direction);

        // 目标已经选择为最近的等效方向，再对位置目标做斜坡限制。
        const float planned_target = steer_limit_target_step(target, wheel_status->now_angle[i], wheel_status->optimize_angle[i]);
        float error_angle = planned_target - wheel_status->now_angle[i];

        // 非小陀螺模式下，误差小于0.05rad时，直接将误差置为0，防止舵轮抖动。
        // 必须在写入多圈目标前处理死区，否则位置环仍会追踪微小角度误差。
        if (fabsf(error_angle) < 0.05f && control_mode != 2)
        {
            error_angle = 0.0f;
            wheel_status->optimize_angle[i] = wheel_status->now_angle[i];
        }

        // 舵向电机只跟踪原始多圈目标角；超过90度时仅反转轮向电机。
        // 目标角使用斜坡限制，避免新速度指令使位置环目标瞬间跳变。
        wheel_status->optimize_angle[i] = planned_target;
        wheel_status->optimize_direction[i] = optimize_direction;
        wheel_status->error_angle[i] = error_angle;

        // 取绝对值避免余弦在超过90度时与方向标志重复翻转。
        const float cos_angle = fabsf(arm_cos_f32(error_angle));
        wheel_status->cosine_optimize[i] = cos_angle * cos_angle * cos_angle;
    }

#else // 关闭余弦优化：不做 cos³ 削减，仅做就近转位
    for (uint8_t i = 0; i < 4; i++)
    {
        const float r_arm = Wheel_To_Core_Distance[i];               // 该轮心到中心的距离(力臂), m
        if (!isfinite(r_arm))
        {
            wheel_status->solution_speed[i] = 0.0f;
            continue;
        }

        const float vel_x = vx_set - vw_set * r_arm * arm_sin_f32(Wheel_Azimuth[i]);
        const float vel_y = vy_set + vw_set * r_arm * arm_cos_f32(Wheel_Azimuth[i]);

        const float vel_modulus = sqrtf(vel_x * vel_x + vel_y * vel_y);

        wheel_status->solution_speed[i] = vel_modulus / STEER_WHEEL_RADIUS;

        // 角度约定与余弦优化分支保持一致(0 对应 +X)；无速度时保持当前角
        wheel_status->solution_angle[i] = (vel_modulus <= 0.0f)
                                        ? wheel_status->now_angle[i]
                                        : atan2f(vel_y, vel_x);
    }

    // 舵向电机目标规划：只展开到当前多圈角度附近，不通过加减π改变舵向目标。
    for (uint8_t i = 0; i < 4; i++)
    {
        // 静止或无轮速时锁定当前角
        if (cmd_zero || wheel_status->solution_speed[i] <= 0.0f)
        {
            wheel_status->optimize_angle[i]     = wheel_status->now_angle[i];
            wheel_status->optimize_direction[i] = 1.0f;
            wheel_status->error_angle[i]        = 0.0f;
            wheel_status->cosine_optimize[i]    = 1.0f;
            continue;
        }

        // 当前角度为多圈角度，先将单圈目标展开到当前角度附近。
        float optimize_direction = 1.0f;
        const float target_angle = steer_select_nearest_target(
            wheel_status->solution_angle[i] + Enc_Zero_Angle[i],
            wheel_status->now_angle[i],
            &optimize_direction);
        float tmp_delta_angle = target_angle - wheel_status->now_angle[i];

        // 舵向目标和轮向速度方向使用同一组等效角规划结果。
        wheel_status->optimize_angle[i] = steer_limit_target_step(
            target_angle,
            wheel_status->now_angle[i],
            wheel_status->optimize_angle[i]);
        wheel_status->optimize_direction[i] = optimize_direction;
        wheel_status->error_angle[i]        =
        wheel_status->optimize_angle[i] - wheel_status->now_angle[i];
        wheel_status->cosine_optimize[i]    = 1.0f; // 关闭余弦优化，不削减轮速
    }
#endif

    // 轮电机实际速度设置 = 解算速度*速度方向*余弦优化系数
    wheel_status->optimize_speed[0] = wheel_status->solution_speed[0] * wheel_status->cosine_optimize[0] * wheel_status->optimize_direction[0]; // 轮组1速度计算
    wheel_status->optimize_speed[1] = wheel_status->solution_speed[1] * wheel_status->cosine_optimize[1] * wheel_status->optimize_direction[1]; // 轮组2速度计算
    wheel_status->optimize_speed[2] = wheel_status->solution_speed[2] * wheel_status->cosine_optimize[2] * wheel_status->optimize_direction[2]; // 轮组3速度计算
    wheel_status->optimize_speed[3] = wheel_status->solution_speed[3] * wheel_status->cosine_optimize[3] * wheel_status->optimize_direction[3]; // 轮组4速度计算
    // 静止死区：舵轮锁定当前角的同时，轮速一并清零，避免小幅爬行
    if(cmd_zero)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_status->solution_speed[i] = 0.0f;
            wheel_status->optimize_speed[i] = 0.0f;
        }
    }

    // 限制最大轮组速度
    float max_vector = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabsf(wheel_status->optimize_speed[i]) > max_vector)
        {
            max_vector = fabsf(wheel_status->optimize_speed[i]);
        }
    }

    // 限制最大轮组速度输出
    if(max_vector > MAX_WHEEL_SPEED)
    {
        // 计算速度缩放比例，不改变运动形态
        const float vector_rate = MAX_WHEEL_SPEED / max_vector;

        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_status->solution_speed[i] *= vector_rate; // 同步缩放，避免下游改用solution_speed时限速失效
            wheel_status->optimize_speed[i] *= vector_rate;
        }
    }
}

/**
 * @brief 舵轮速度解算（正运动学，由四个舵轮的反馈反解机器人实际速度）
 * @param robot_speed 机器人实际速度（输出：vx/vy 单位m/s，vw 单位rad/s）
 * @param wheel_angle 四个舵向电机的转向角反馈值，单位rad
 * @param wheel_speed 四个轮向电机的角速度反馈值，单位rad/s
 */
void steer_wheel_speed_soluction(wheel_robot_speed_t *robot_speed, float *wheel_angle, float *wheel_speed)
{
    if (robot_speed == nullptr || wheel_angle == nullptr || wheel_speed == nullptr) return;

    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        const float wheel_linear_speed = wheel_speed[i] * STEER_WHEEL_RADIUS; // 轮向电机角速度 → 轮子线速度, 单位m/s

        // 反向扣除零位,同时 now_angle 为多圈连续角, 归一化到 [-π,π) 防止精度退化.
        const float steer = basic_math_modulus_normalization(wheel_angle[i] - Enc_Zero_Angle[i], 2.0f * PI);

        const float sin_angle = arm_sin_f32(steer);   // sinθ
        const float cos_angle = arm_cos_f32(steer);   // cosθ

        vx_sum += wheel_linear_speed * cos_angle; // 各轮线速度在 X(前) 方向分量
        vy_sum += wheel_linear_speed * sin_angle; // 各轮线速度在 Y(左) 方向分量
        // 角速度分量: v_i·sin(θ_i−φ_i)/R_i = ω, 四轮求和再平均即底盘角速度
        vw_sum += wheel_linear_speed * arm_sin_f32(steer - Wheel_Azimuth[i]) / Wheel_To_Core_Distance[i];
    }

    robot_speed->vx = vx_sum * 0.25f;                // 四轮X分量均值即底盘X轴实际速度
    robot_speed->vy = vy_sum * 0.25f;                // 四轮Y分量均值即底盘Y轴实际速度
    robot_speed->vw = vw_sum * 0.25f;                // Σv·sin(θ−φ)/R = 4ω → 底盘角速度, 单位rad/s
}

#endif

/*-------------------------------------------------------------------------Mecanum Wheel Odometry-----------------------------------------------------------------------------------------------------------*/
#if WHEEL_TYPE == 1 // 麦轮编码器里程计 (仅麦轮车系使用)

/**
 * @brief 初始化麦轮编码器里程计状态
 * @param odom 里程计状态结构体指针
 */
void mecanum_encoder_odom_init(mecanum_encoder_odom_t *odom)
{
    if (odom == nullptr) return;

    for (int i = 0; i < 4; i++)
    {
        odom->last_angle_rad[i] = 0.0f;
    }
    odom->pose.x   = 0.0f;
    odom->pose.y   = 0.0f;
    odom->pose.yaw = 0.0f;
    odom->initialized = false;
}

/**
 * @brief 麦轮编码器里程计更新
 * @param odom          里程计状态结构体指针 (维护内部状态, 输出位姿到 odom->pose)
 * @param wheel_encoder_rad 四个麦轮的多圈累计轮转角 (float[4], 单位 rad)
 * @param imu_yaw_rad   IMU反馈的绝对航向角 (rad), 用于机体→世界坐标旋转,
 *                      替代轮式解算yaw以避免轮子打滑引起的方向混乱
 * @note  首次调用自动记录基准角度 (位移=0).
 *        轮转角正方向需与驱动车体前进的方向一致.
 */
void mecanum_encoder_odom_update(mecanum_encoder_odom_t *odom, wheel_motor_encoder_t *wheel_encoder_rad, float imu_yaw_rad)
{
    if (odom == nullptr || wheel_encoder_rad == nullptr) return;

    float dtheta[4];   // 本帧轮转角增量 (rad)
    float ds[4];       // 各轮线位移 (m), 正转→前进方向

    // 首次调用时记录基准角度, 避免里程计位姿跳变
    if (!odom->initialized)
    {
        for (int i = 0; i < 4; i++)
        {
            odom->last_angle_rad[i] = wheel_encoder_rad->wheel_angle[i];
        }
        odom->initialized = true;
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        dtheta[i] = wheel_encoder_rad->wheel_angle[i] - odom->last_angle_rad[i];
        odom->last_angle_rad[i] = wheel_encoder_rad->wheel_angle[i];
    }

    /* ---- Step 2: 轮转角增量 → 轮子线位移 (m) ---- */
    for (int i = 0; i < 4; i++)
    {
        ds[i] = dtheta[i] * WHEEL_RADIUS_S;
    }

    float dx_body = ( ds[0] + ds[1] - ds[2] - ds[3]) / 4.0f;
    float dy_body = (-ds[0] + ds[1] + ds[2] - ds[3]) / 4.0f;
    float dyaw    = -( ds[0] + ds[1] + ds[2] + ds[3]) / (4.0f * MECANUM_WHEEL_SUM_XY);

    /* ---- 忽略零位移帧 (微噪声不积分) ---- */
    if (fabsf(dx_body) < 1e-7f && fabsf(dy_body) < 1e-7f && fabsf(dyaw) < 1e-9f)
    {
        return;
    }

    float cos_yaw = arm_cos_f32(imu_yaw_rad);
    float sin_yaw = arm_sin_f32(imu_yaw_rad);

    odom->pose.x += dx_body * cos_yaw - dy_body * sin_yaw;
    odom->pose.y += dx_body * sin_yaw + dy_body * cos_yaw;

    /* ---- yaw 直接同步 IMU 反馈值, 不再累加轮式差分 ---- */
    odom->pose.yaw = imu_yaw_rad;
}

/**
 * @brief 初始化轮式速度里程计位置
 * @param position 位置结构体指针
 */
void wheel_speed_odometry_init(wheel_robot_position_t *position)
{
    if (position == nullptr) return;

    position->x   = 0.0f;
    position->y   = 0.0f;
    position->yaw = 0.0f;
}

void wheel_speed_odometry_update(wheel_robot_position_t *position, wheel_robot_speed_t *robot_speed, float dt, float imu_yaw_rad)
{
    if (dt <= 0.0f) return;

    float cos_yaw = arm_cos_f32(imu_yaw_rad);
    float sin_yaw = arm_sin_f32(imu_yaw_rad);

    position->x += (robot_speed->vx * cos_yaw - robot_speed->vy * sin_yaw) * dt;
    position->y += (robot_speed->vx * sin_yaw + robot_speed->vy * cos_yaw) * dt;

    /* ---- yaw 直接同步 IMU 反馈值, 不再累加轮式积分 ---- */
    position->yaw = imu_yaw_rad;
}

/*-------------------------------------------------------------------------Steering Wheel Odometry-----------------------------------------------------------------------------------------------------------*/
#elif WHEEL_TYPE == 2 // 舵轮编码器里程计 (仅舵轮车系使用)

/**
 * @brief 单帧轮向转角增量 → 线位移
 * @param cur  本帧轮向多圈累计转角(连续角), rad
 * @param prev 上一帧轮向转角(连续角), rad
 * @return 本帧轮子线位移, m (正转→前进方向)
 * @note  采用"多圈连续角"反馈时, 直接相减即为真实增量:
 *        1) 有多圈信息, 不存在单圈反馈在 ±π 处的过零跳变, 无需取模;
 *        2) 若再对增量做 2π 归一化, 反而会在单帧真实转角 >π 时少算一整圈(且掩盖丢帧);
 *        3) 前提: 上层喂入的 wheel_angle 必须是多圈连续角(非 now_angle 单圈值), 否则需改回归一化。
 */
static inline float steer_odom_wheel_ds(float cur, float prev)
{
    const float dtheta = cur - prev;
    return dtheta * STEER_WHEEL_RADIUS;
}

void steer_encoder_odom_init(steer_encoder_odom_t *odom, IMU_Odometry_TypeDef imu_odom)
{
    if (odom == nullptr || imu_odom < IMU_None || imu_odom > IMU_Nine_Axis) return;

    odom->imu_type = imu_odom;

    for (uint8_t i = 0; i < 4; i++)
    {
        odom->last_encoder.wheel_angle[i] = 0.0f;  
        odom->last_encoder.steer_angle[i] = 0.0f;
    }

    odom->pose.x = 0.0f;
    odom->pose.y = 0.0f;
    odom->pose.yaw = 0.0f;
    odom->initialized = false;
}

/**
 * @brief 舵轮编码器里程计更新 (实时调用, 通常 1kHz)
 *
 * 支持两种航向来源:
 *   1. odom->imu_type == IMU_None : 无 IMU, 航向角由舵轮运动学解算 (角位移 dyaw 差分积分);
 *   2. odom->imu_type != IMU_None : 有 IMU, 航向角直接采用 IMU 绝对航向角
 *                                   (抑制轮子打滑/侧滑导致的航向累积误差).
 * 两种模式共用同一套位移解算, 仅"航向来源"与"机体→世界旋转角"不同.
 *
 * @param odom              里程计状态结构体指针 (维护内部状态, 输出位姿到 odom->pose)
 * @param wheel_encoder_rad 四个舵轮的多圈累计编码器角度 (单位 rad)
 *                          wheel_angle[i] 轮向累计转角, steer_angle[i] 舵向累计转角(可多圈)
 * @param imu_yaw_rad     IMU 绝对航向角 (rad), 仅 imu_type != IMU_None 时使用
 *
 * @note 轮组布局: [0]=FL前左 [1]=RL后左 [2]=RR后右 [3]=FR前右
 *       轮心方位角 φ_i = Wheel_Azimuth[i] (45/135/225/315°), 力臂 R_i = Wheel_To_Core_Distance[i]
 *       与 steer_wheel_speed_soluction 角度约定一致: 单轮线位移在底盘系分量 = ds·(cosθ, sinθ)
 *       (θ=0 → +X前, 逆时针为正). 首次调用自动记录基准角度并对齐航向, 避免位姿跳变.
 */
void steer_encoder_odom_update(steer_encoder_odom_t *odom, wheel_system_encoder_t *wheel_encoder_rad, float imu_yaw_rad)
{
    if (odom == nullptr || wheel_encoder_rad == nullptr) return;

    // 有 IMU 时用 IMU 绝对航向; 无 IMU 时用编码器运动学解算的航向
    const bool use_imu = (odom->imu_type != IMU_None);

    // 获取编码器基准, 对齐航向角
    if (!odom->initialized)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            odom->last_encoder.wheel_angle[i] = wheel_encoder_rad->wheel_angle[i];
            odom->last_encoder.steer_angle[i] = wheel_encoder_rad->steer_angle[i];
        }
        odom->pose.yaw    = use_imu ? imu_yaw_rad : 0.0f; // 有 IMU 直接对齐绝对航向; 无则从 0 起
        odom->initialized = true;
        return;
    }

    // 各轮本帧线位移 (m), 正转→前进方向 (多圈连续角直接相减)
    float ds[4];

    for (uint8_t i = 0; i < 4; i++)
    {
        // wheel_angle 为多圈连续角: 直接差分即真实转角, 无需归一化(归一化反而会在>π时少算一圈)
        ds[i] = steer_odom_wheel_ds(wheel_encoder_rad->wheel_angle[i], odom->last_encoder.wheel_angle[i]);

        odom->last_encoder.wheel_angle[i] = wheel_encoder_rad->wheel_angle[i]; // 更新轮向缓存
        odom->last_encoder.steer_angle[i] = wheel_encoder_rad->steer_angle[i]; // 同步舵向缓存(供观察/扩展)
    }

    /* ---- Step 2: 各轮线位移投影到底盘系(X前/Y左), 四轮累加解算 dx/dy/dyaw ---- */
    //  与正解 steer_wheel_speed_soluction 完全一致:
    //    单轮线位移在底盘系分量 = ds·(cosθ, sinθ);
    //    角位移分量 = ds·sin(θ−φ)/R, φ_i = Wheel_Azimuth[i], R_i = Wheel_To_Core_Distance[i].
    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        // 扣除零位; steer_angle 为多圈累计角, 但 sinf/cosf 对整圈不敏感
        const float steer = wheel_encoder_rad->steer_angle[i] - Enc_Zero_Angle[i];
        const float sin_angle = arm_sin_f32(steer);                        // 底盘 X(前) 方向分量系数
        const float cos_angle = arm_cos_f32(steer);                        // 底盘 Y(左) 方向分量系数

        vx_sum += ds[i] * cos_angle;                               // 累加各轮 X 位移分量
        vy_sum += ds[i] * sin_angle;                               // 累加各轮 Y 位移分量
        vw_sum += ds[i] * arm_sin_f32(steer - Wheel_Azimuth[i]) / Wheel_To_Core_Distance[i]; // 角位移分量, Σ = 4·dyaw
    }

    const float dx_body = vx_sum * 0.25f;                          // 四轮 X 分量均值 → 底盘 X 位移 (m)
    const float dy_body = vy_sum * 0.25f;                          // 四轮 Y 分量均值 → 底盘 Y 位移 (m)
    const float dyaw    = vw_sum * 0.25f;                          // 底盘角位移 (rad), 与正解同号

    /* ---- 忽略零位移帧 (静止/微噪声不积分), 有 IMU 时仍同步航向 ---- */
    if (fabsf(dx_body) < 1e-7f && fabsf(dy_body) < 1e-7f && fabsf(dyaw) < 1e-9f)
    {
        // 如果所有位移都为零，则不更新位姿
        if (use_imu) odom->pose.yaw = imu_yaw_rad;
        return;
    }

    /* ---- Step 3: 确定航向角 (IMU 优先; 无 IMU 则编码器角位移积分) ---- */
    float yaw_for_rotate;                                          // 机体→世界 旋转所用的航向角
    if (use_imu)
    {
        yaw_for_rotate = imu_yaw_rad;                            // 直接采用 IMU 绝对航向
        odom->pose.yaw  = imu_yaw_rad;                          // 输出航向同步 IMU
    }
    else
    {
        // 无 IMU: 航向 = 上一帧航向 + 本帧舵轮解算角位移, 归一化到 [-π, π)
        odom->pose.yaw = basic_math_modulus_normalization(odom->pose.yaw + dyaw, 2.0f * PI);
        yaw_for_rotate = odom->pose.yaw;
    }

    /* ---- Step 4: 底盘系位移 → 世界系 (用本帧航向旋转), 累加到位姿 ---- */
    const float cos_yaw = arm_cos_f32(yaw_for_rotate);
    const float sin_yaw = arm_sin_f32(yaw_for_rotate);

    odom->pose.x += dx_body * cos_yaw - dy_body * sin_yaw;
    odom->pose.y += dx_body * sin_yaw + dy_body * cos_yaw;
}

/**
 * @brief 输出轮式里程计位姿, 供在线观察/标定 (仅在主循环低频调用, 勿放入 1kHz 控制流)
 */
void odom_debug_printf(const steer_encoder_odom_t *odom)
{
    if (odom == nullptr) return;
    printf("[ODOM] x=%.4f y=%.4f yaw=%.4f rad (%.2f deg)\r\n",
           odom->pose.x, odom->pose.y, odom->pose.yaw, odom->pose.yaw * (180.0f / PI));
}

#endif