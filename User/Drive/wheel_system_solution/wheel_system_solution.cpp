#include "wheel_system_solution.h"
#include <cmath>
#include "arm_math.h"
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
    const bool cmd_zero = (fabsf(vx_set) <= MIN_LIMIT) &&
                          (fabsf(vy_set) <= MIN_LIMIT) &&
                          (fabsf(vw_set) <= MIN_LIMIT);

#if COSINE_OPTIMIZE_ENABLE == 1 // 启动余弦优化
    float vx_plus, vx_minus, vy_plus, vy_minus;
    float rotational_speed;

    // 底盘整体旋转控制速度投影到每个轮组的线速度分量
    rotational_speed = vw_set * STEER_WHEEL_DISTANCE_R * SQRT_2_2;

    vx_plus  = vx_set + rotational_speed;
    vx_minus = vx_set - rotational_speed;
    vy_plus  = vy_set + rotational_speed;
    vy_minus = vy_set - rotational_speed;

    // 求解解算后的轮向电机角速度
    wheel_status->solution_speed[0] = sqrtf(vx_minus * vx_minus + vy_plus * vy_plus) / WHEEL_RADIUS_S;  // 轮组1速度计算
    wheel_status->solution_speed[1] = sqrtf(vx_minus * vx_minus + vy_minus * vy_minus) / WHEEL_RADIUS_S; // 轮组2速度计算
    wheel_status->solution_speed[2] = sqrtf(vx_plus * vx_plus + vy_minus * vy_minus) / WHEEL_RADIUS_S;  // 轮组3速度计算
    wheel_status->solution_speed[3] = sqrtf(vx_plus * vx_plus + vy_plus * vy_plus) / WHEEL_RADIUS_S;  // 轮组4速度计算

    // 求解解算后的轮向电机角度
    wheel_status->solution_angle[0] = atan2f(vx_minus, vy_plus);  // 轮组1转向角计算
    wheel_status->solution_angle[1] = atan2f(vx_minus, vy_minus); // 轮组2转向角计算
    wheel_status->solution_angle[2] = atan2f(vx_plus, vy_minus);  // 轮组3转向角计算
    wheel_status->solution_angle[3] = atan2f(vx_plus, vy_plus);  // 轮组4转向角计算

    // 舵向电机余弦优化计算
    for(uint8_t i = 0; i < 4; i++)
    {
        // 静止或无轮速(解算矢量模长为0)时锁定当前角度，防止舵轮无意义摆动。
        // 当前角度是多圈角度，目标角选择当前角附近的等效值，避免跨圈时位置环绕远路。
        float optimize_direction = 1.0f;
        const float target = (cmd_zero || wheel_status->solution_speed[i] <= 0.0f)
                             ? wheel_status->now_angle[i]
                             : steer_select_nearest_target(wheel_status->solution_angle[i] + Enc_Zero_Angle[i],wheel_status->now_angle[i], &optimize_direction);

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
    const float wheel_projection = STEER_WHEEL_DISTANCE_R * SQRT_2_2; // 轮组投影坐标 a

    for (uint8_t i = 0; i < 4; i++)
    {
        // 各轮组投影坐标符号: FL(+1,+1) RL(-1,+1) RR(-1,-1) FR(+1,-1)
        const float sign_x = (i == 0 || i == 3) ? 1.0f : -1.0f; // r_x / a
        const float sign_y = (i < 2) ? 1.0f : -1.0f;            // r_y / a

        // 解算到每个轮组的具体线速度 (vx - ω·r_y, vy + ω·r_x)
        const float tmp_velocity_x = vx_set - vw_set * wheel_projection * sign_y;
        const float tmp_velocity_y = vy_set + vw_set * wheel_projection * sign_x;

        const float tmp_velocity_modulus = sqrtf(tmp_velocity_x * tmp_velocity_x +
                                                 tmp_velocity_y * tmp_velocity_y);

        wheel_status->solution_speed[i] = tmp_velocity_modulus / WHEEL_RADIUS_S;

        // 角度约定与余弦优化分支保持一致(0 对应 +Y)；无速度时保持当前角
        wheel_status->solution_angle[i] = (tmp_velocity_modulus <= 0.0f)
                                        ? wheel_status->now_angle[i]
                                        : atan2f(tmp_velocity_x, tmp_velocity_y);
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
 * @note  轮组布局: [0]=FL前左, [1]=RL后左, [2]=RR后右, [3]=FR前右
 *        轮子投影坐标: FL(+a,+a) RL(-a,+a) RR(-a,-a) FR(+a,-a), a = STEER_WHEEL_DISTANCE_R * SQRT_2_2
 *        单个轮子线速度在底盘系的分量为 (v·sinθ, v·cosθ)，与逆解 atan2f(vx, vy) 的角度约定一致
 */
void steer_wheel_speed_soluction(wheel_robot_speed_t *robot_speed, float *wheel_angle, float *wheel_speed)
{
    const float projection = STEER_WHEEL_DISTANCE_R * SQRT_2_2; // 轮子投影点到几何中心的X/Y坐标，单位m
    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        const float wheel_linear_speed = wheel_speed[i] * WHEEL_RADIUS_S; // 轮向电机角速度 → 轮子线速度，单位m/s
        const float sin_angle = arm_sin_f32(wheel_angle[i]);              // 轮子线速度在底盘X轴的分量系数
        const float cos_angle = arm_cos_f32(wheel_angle[i]);              // 轮子线速度在底盘Y轴的分量系数

        vx_sum += wheel_linear_speed * sin_angle; // 累加各轮线速度的X轴分量
        vy_sum += wheel_linear_speed * cos_angle; // 累加各轮线速度的Y轴分量

        // 依据 vx - vw·r_y = v·sinθ, 以轮子投影Y坐标符号 (+a,+a,-a,-a) 为权消去vx，解出角速度
        vw_sum += (i < 2 ? 1.0f : -1.0f) * wheel_linear_speed * sin_angle;
    }

    robot_speed->vx = vx_sum * 0.25f;                // 四轮X分量均值即底盘X轴实际速度
    robot_speed->vy = vy_sum * 0.25f;                // 四轮Y分量均值即底盘Y轴实际速度
    robot_speed->vw = -vw_sum / (4.0f * projection); // 底盘实际角速度，单位rad/s
}

#endif

/*-------------------------------------------------------------------------mecanum_odometry_encoder-----------------------------------------------------------------------------------------------------------*/

/*===========================================================================
 * 麦轮编码器里程计 — 基于累计轮转角的位移解算
 *
 * 输入: 外部已转换为多圈累计轮转角 (rad), 无需编码器原始值.
 *
 * 核心流程:
 *   Step 1 — 计算轮转角增量:
 *            Δθ_i = wheel_angle_rad[i] - last_angle_rad[i]
 *   Step 2 — 轮转角增量 → 轮子线位移:
 *            ds_i = Δθ_i × WHEEL_RADIUS_S
 *   Step 3 — 麦轮正运动学 (位移形式):
 *            dx_body = ( ds0 + ds1 - ds2 - ds3) / 4
 *            dy_body = (-ds0 + ds1 + ds2 - ds3) / 4
 *            dyaw    = ( ds0 + ds1 + ds2 + ds3) / (4 × (a+b))
 *   Step 4 — 机体→世界 旋转 (半步长航向中值法):
 *            ψ_mid = ψ(k-1) + ½·Δψ
 *            x(k) = x(k-1) + dx_body·cos(ψ_mid) - dy_body·sin(ψ_mid)
 *            y(k) = y(k-1) + dx_body·sin(ψ_mid) + dy_body·cos(ψ_mid)
 *            ψ(k) = ψ(k-1) + Δψ (rad) → 转 deg 存储
 *
 * 坐标系:
 *   机体: X=前 Y=左 Z=上 (右手系)
 *   世界: 上电位置=(0,0), 上电朝向=前(+x)
 *   yaw: CCW为正, 内部 rad 计算, 输出 deg
 *
 * 轮组布局 & 轮转角索引 (俯视, 外八辊子):
 *        FL[0] \  ← 前 →  / FR[3]
 *          ↙                ↗    ← 辊子外八 (小口→车内, 大口→车外)
 *          ↖                ↘
 *        RL[1] /  ← 后 →  \ RR[2]
 *===========================================================================*/

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

    /* ---- 首次调用: 捕获基准角度, 位移=0 ---- */
    if (!odom->initialized)
    {
        for (int i = 0; i < 4; i++)
        {
            odom->last_angle_rad[i] = wheel_encoder_rad->wheel_angle[i];
        }
        odom->initialized = true;
        return;
    }

    /* ---- Step 1: 轮转角增量 (rad) ---- */
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

    /* ---- Step 3: 麦轮正运动学 (机体坐标系位移) ---- */
    /*
     * 逆运动学矩阵 M (4×3) 的 Moore-Penrose 伪逆 M⁺:
     *
     *   ω_i = M · [vx_set, vy_set, ω]ᵀ,  其中 M 的第 i 行 = f_i(vx_set,vy_set,ω)/R
     *
     *   M⁺ = (MᵀM)⁻¹Mᵀ  →  对于正交轮组简化为:
     *
     *   dx_body = ( ds0 + ds1 - ds2 - ds3) / 4
     *   dy_body = (-ds0 + ds1 + ds2 - ds3) / 4
     *   dyaw    = ( ds0 + ds1 + ds2 + ds3) / (4 × (a+b))
     *
     *   验证 (纯前向 vx_set>0): FL+RL正转 RR+FR反转 → dx>0 ✓
     *   验证 (纯左移 vy_set>0): FL+FR反转 RL+RR正转 → dy>0 ✓
     */
    float dx_body = ( ds[0] + ds[1] - ds[2] - ds[3]) / 4.0f;
    float dy_body = (-ds[0] + ds[1] + ds[2] - ds[3]) / 4.0f;
    float dyaw    = -( ds[0] + ds[1] + ds[2] + ds[3]) / (4.0f * MECANUM_WHEEL_SUM_XY);

    /* ---- 忽略零位移帧 (微噪声不积分) ---- */
    if (fabsf(dx_body) < 1e-7f && fabsf(dy_body) < 1e-7f && fabsf(dyaw) < 1e-9f)
    {
        return;
    }

    /* ---- Step 4: 机体→世界 旋转 (使用IMU反馈yaw, 避免轮式yaw累积误差) ---- */
    /*
     * 关键设计: 使用IMU反馈的绝对航向角进行坐标旋转, 而非轮式解算yaw.
     *   轮式yaw (dyaw) 由轮子编码器差分计算, 轮子打滑会导致累积误差,
     *   进而污染世界坐标系位置解算, 引起后续运动控制中轮子方向混乱.
     *
     *   IMU yaw 由陀螺仪+加速度计+磁力计融合得到, 长期精度远优于轮式yaw.
     *   轮式yaw仅作为辅助监控量保留在 odom->pose.yaw 中.
     *
     * 坐标系: 前向=(cosψ, sinψ), 左向=(-sinψ, cosψ)
     */
    float cos_yaw = cosf(imu_yaw_rad);
    float sin_yaw = sinf(imu_yaw_rad);

    odom->pose.x += dx_body * cos_yaw - dy_body * sin_yaw;
    odom->pose.y += dx_body * sin_yaw + dy_body * cos_yaw;

    /* ---- yaw 直接同步 IMU 反馈值, 不再累加轮式差分 ---- */
    odom->pose.yaw = imu_yaw_rad;
}

/*===========================================================================
 * 轮式里程计 — 正运动学位移积分 (所有轮系通用)
 *
 * 坐标系:
 *   机体: X=前 Y=左 Z=上 (右手系)
 *   世界: 上电位置=(0,0), 上电朝向=前(x+), 左=y+
 *   yaw: CCW为正 (Z轴右手定则), 单位弧度 (rad), 0=初始朝向
 *
 * 公式:
 *   yaw(k)   = imu_yaw_rad              (直接同步IMU航向, 避免轮式积分漂移)
 *   pos_x(k) = pos_x(k-1) + (vx_set*cosψ - vy_set*sinψ) * dt
 *   pos_y(k) = pos_y(k-1) + (vx_set*sinψ + vy_set*cosψ) * dt
 *   ψ = yaw 中值 (使用半步长航向提高精度)
 *===========================================================================*/
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

    /* ---- Body→World 旋转: 使用IMU反馈yaw, 避免轮式yaw累积误差 ---- */
    /*
     * 关键设计: 使用IMU反馈的绝对航向角进行坐标旋转, 而非轮式积分yaw.
     *   轮式yaw (由正运动学 vw_set 积分得到) 会因轮子打滑产生累积误差,
     *   进而污染世界坐标系位置解算, 引起后续运动控制中轮子方向混乱.
     *
     *   IMU yaw 由陀螺仪+加速度计+磁力计融合得到, 长期精度远优于轮式yaw.
     *   轮式yaw仅作为辅助监控量保留在 position->yaw 中.
     *
     * 坐标系: 前向=(cosψ, sinψ), 左向=(-sinψ, cosψ)
     */

    float cos_yaw = cosf(imu_yaw_rad);
    float sin_yaw = sinf(imu_yaw_rad);

    position->x += (robot_speed->vx * cos_yaw - robot_speed->vy * sin_yaw) * dt;
    position->y += (robot_speed->vx * sin_yaw + robot_speed->vy * cos_yaw) * dt;

    /* ---- yaw 直接同步 IMU 反馈值, 不再累加轮式积分 ---- */
    position->yaw = imu_yaw_rad;
}
#endif