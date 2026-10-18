/**
 * @file pid_fuse.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "pid_fuse.h"
#include  <math.h>
#include "user_lib.h"

// PID 初始化函数
void pid_init(pid_t* pid, pid_config* config)
{
    pid->config = *config;
}

//快速响应：大误差时使用Bang-Bang控制

//平稳过渡：中等误差时使用PD控制加积分预加载

//精确稳定：小误差时使用完整的PID控制

//抗积分饱和：在不同阶段合理管理积分项

float pid_calc(pid_t* pid, float expected, float feedback)
{
    pid->expected = expected;
    pid->feedback = feedback;

    pid->error[2] = pid->error[1];        //上上次误差
    pid->error[1] = pid->error[0];        //上次误差
    pid->error[0] = pid->expected - pid->feedback;  //本次误差
    pid->abs_error = fabsf(pid->error[0]);

    if (pid->config.mode == PID_POSITION)  //位置式PID
    {
        pid->error_sum += pid->error[0];  //积分上限判断
        if (pid->error_sum > pid->config.integral_limit) pid->error_sum = pid->config.integral_limit;
        if (pid->error_sum < -pid->config.integral_limit) pid->error_sum = -pid->config.integral_limit;

        pid->output = pid->config.Kp * pid->error[0] + pid->config.Ki * pid->error_sum + pid->config.Kd * (pid->error[0] - pid->error[1]);
    }

    else if (pid->config.mode == PID_DELTA)  //增量式PID
    {
        pid->output += pid->config.Kp * (pid->error[0] - pid->error[1]) + pid->config.Ki * pid->error[0] + pid->config.Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    }

    else if (pid->config.mode == PID_COMP_POSITION)  // PI分离模式 用于摩擦轮和拨弹
    {
        pid->error_delta = pid->error[0] - pid->error[1];
        if (pid->error[0] > pid->config.range_rough)  // 误差绝对值大于range_rough时,max_out
        {
            pid->output = pid->config.max_out;        //输出最大控制量，快速消除大误差
            pid->error_sum = 0;
        } else if (pid->error[0] < -pid->config.range_rough)  // bangbang
        {
            pid->output = -pid->config.max_out;
            pid->error_sum = 0;
        } else if (fabsf(pid->error[0]) < pid->config.range_tiny)  //细调，误差绝对值小于range_tiny时，使用较小的比例系数kp_tiny
        {
            pid->error_sum += pid->error[0];  //积分上限判断
            if (pid->error_sum > pid->config.integral_limit) pid->error_sum = pid->config.integral_limit;
            if (pid->error_sum < -pid->config.integral_limit) pid->error_sum = -pid->config.integral_limit;
            pid->output = pid->config.Kp_tiny * pid->error[0] + pid->config.Ki * pid->error_sum + pid->config.Kd * pid->error_delta;
        } else  //粗调，误差介于 range_tiny 和 range_rough 之间，使用较大的比例系数kp，添加补偿
        {
            pid->output = pid->config.Kp * (pid->error[0] + fsgn(pid->error[0]) * pid->config.Kf) + pid->config.Kd * pid->error_delta;
            pid->error_sum = fsgn(pid->error[0]) * pid->config.error_preload;
        }
    }

    else if (pid->config.mode == PID_FUSION_POSITION)  //融合式PID 前馈补偿 + 变速积分 + 微分先行 + 死区控制
    {
        static float Kd_out = 0.0f;

        /* 死区控制，小于dead_band不进行PID控制 */
        if (pid->abs_error < pid->config.dead_band)  //死区控制
        {
            pid->expected = pid->last_expected;
            pid->error[0] = 0.0f;
            pid->error_sum = 0.0f;
            pid->abs_error = 0.0f;
        }
        else
        {
                    /* 微分先行控制，根据误差变化率调整Kd */
            if (pid->config.Kd_pre == PID_PRE_ENABLE)                               //微分先行使能 
            {
                Kd_out = pid->last_feedback - pid->feedback;  //微分先行计算
            }
            else
            {
                Kd_out = pid->error[0] - pid->error[1];  //传统微分计算
            }
        }

        /* 变速积分控制，根据误差大小调整积分比例系数 */
        if (pid->config.Ki_speed_A == 0.0f && pid->config.Ki_speed_B == 0.0f)
        {
            pid->speed_ratio = 1.0f;  //无变速积分
        }
        else
        {
            if (pid->abs_error <= pid->config.Ki_speed_A)   //误差小于阈值A，积分比例系数为1
            {
                pid->speed_ratio = 1.0f;
            }
            else if (pid->abs_error >= pid->config.Ki_speed_B) //误差大于阈值B，积分比例系数为0
            {
                pid->speed_ratio = 0.0f;
            }
            else  //误差介于A和B之间，线性调整积分比例系数
            {
                pid->speed_ratio = (pid->config.Ki_speed_B - pid->abs_error) / (pid->config.Ki_speed_B - pid->config.Ki_speed_A);
            }
        }

        /* 积分分离控制，当误差过大时，清零积分项输出值 */
        if (pid->config.Ki_separate == 0.0f)    //无积分分离控制
        {
            pid->error_sum += pid->speed_ratio * pid->config.Ki * pid->error[0];
        }

        else                                    //有积分分离控制
        {
            if (pid->abs_error < pid->config.Ki_separate)  //积分分离控制，误差小于分离阈值时启用积分
            {
                pid->error_sum += pid->speed_ratio * pid->config.Ki * pid->error[0];  //积分上限判断，乘以时间片步长
                if (pid->error_sum > pid->config.integral_limit) pid->error_sum = pid->config.integral_limit;
                if (pid->error_sum < -pid->config.integral_limit) pid->error_sum = -pid->config.integral_limit;
            }
            else  //误差大于分离阈值时，清空积分项
            {
                pid->error_sum = 0.0f;
            }
        }
        
        /* PID计算 */
        pid->output = pid->config.Kp * pid->error[0] + pid->error_sum + pid->config.Kd * Kd_out + pid->config.Kf * (pid->expected - pid->last_expected);  //添加前馈补偿

    }
    
    pid->last_expected = pid->expected;
    pid->last_feedback = pid->feedback;
    /*----- 输出上限 -----*/
    pid->output_unlimited = pid->output;
    if (pid->output > pid->config.max_out) pid->output = pid->config.max_out;
    if (pid->output < -pid->config.max_out) pid->output = -pid->config.max_out;
    
    return pid->output;
}

/**
 * @brief 
 * @param config 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param integral_limit 
 * @param max_out 
 * @note 位置式PID配置函数
 */
void pid_set_config_pos(pid_config* config, float Kp, float Ki, float Kd, float integral_limit, float max_out)
{
    config->mode = PID_POSITION;
    config->Kp = Kp;
    config->Ki = Ki;
    config->Kd = Kd;
    config->integral_limit = integral_limit;
    config->max_out = max_out;
}

// PI分离
void pid_set_config_comp(pid_config* config, float Kp_rough, float Kp_tiny, float Ki, float Kd, float rangerough, float rangefine, float Kf, float errorpreload, float integral_limit,
                        float max_out) 
{
    config->mode = PID_COMP_POSITION;
    config->Kp = Kp_rough;
    config->Kp_tiny = Kp_tiny;
    config->Ki = Ki;
    config->Kd = Kd;
    config->range_rough = rangerough;
    config->range_tiny = rangefine;
    config->Kf = Kf;   //位置环补偿
    config->error_preload = errorpreload;  //位置环积分预载
    config->integral_limit = integral_limit;
    config->max_out = max_out;
}

//PID前馈补偿 + 变速积分 + 积分分离 + 微分先行 + 死区控制

/**
 * @note 添加PID前馈补偿，解决系统滞后问题，提高响应速度和控制精度。
 * @note 由于增加时间片步长会导致PID参数出现异常I*时间步长，D/时间步长，因此先去掉时间步长，时间步长的好处是：将连续的PID计算离散化，积分项*时间片补偿，得到积分增益。
 * 微分项/时间片补偿，计算微分变化率用于微分先行控制。
 * 将PID计算频率与PID控制器时间片解耦，修改任务周期只需，修改时间片步长即可。
 * @note PID前馈补偿：根据期望值变化率，提前调整输出，减少系统滞后，提高响应速度。
 * @note 变速积分：根据误差大小动态调整积分增益，偏差大时减少积分作用，偏差小时增加积分作用，防止积分饱和。
 * @note 积分分离：在大误差阶段，清空积分项，误差较小时才启用积分控制，防止积分项引起的过冲和震荡。
 * @note 微分先行：通过提前计算误差变化率，调整微分Kd参数，提高系统响应速度和适应性。
 * @note 死区控制：当误差小于一定范围时，不进行PID计算，避免抖动。
 */
void pid_set_config_fusion(pid_config* config, pid_Kd_pre_e Kd_pre, float Kp, float Ki,float kd, float Kf, float dead_band,
                           float Ki_speed_A, float Ki_speed_B, float Ki_separate, float integral_limit, float max_out) 
{
    config->mode = PID_FUSION_POSITION;
    config->Kd_pre = Kd_pre;                // 微分先行使能
    config->Kp = Kp;
    config->Ki = Ki;
    config->Kd = kd;
    config->Kf = Kf;                        // 前馈增益
    config->dead_band = dead_band;          // 死区范围
    config->Ki_speed_A = Ki_speed_A;        // 变速积分阀值A
    config->Ki_speed_B = Ki_speed_B;        // 变速积分阀值B
    config->Ki_separate = Ki_separate;      // 积分分离阀值
    config->integral_limit = integral_limit;
    config->max_out = max_out;
}

