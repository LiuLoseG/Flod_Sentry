#pragma once

#include <stdint.h>

// PID前馈开关
typedef enum
{
    PID_PRE_DISABLE = 0,
    PID_PRE_ENABLE
} pid_Kd_pre_e;

// PID工作模式
typedef enum pid_mode_enum
{
    PID_POSITION = 0,       // 位置式PID
    PID_DELTA,              // 增量式PID
    PID_COMP_POSITION,      // 位置式PID+前馈
    PID_FUSION_POSITION     // 融合式(位置式PID+前馈+变速积分)
} pid_mode;

#pragma pack(1)

typedef struct pid_config_struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;                   // 前馈增益

    // float time_step;         // 时间片步长
    float dead_band;            // 死区范围

    float Ki_speed_A;           // 变速积分阀值A
    float Ki_speed_B;           // 变速积分阀值B
    float Ki_separate;          // 积分分离阀值

        //误差大时快速响应，误差小时平稳调整
    float Kp_tiny;              // 微调参数
    float range_rough;          // 粗调范围,误差小于此值才粗调用Kp_rough
    float range_tiny;           // 微调范围,误差小于此值才调用Kp_tiny

    float max_out;              // 输出限幅
    float integral_limit;       // 积分限幅

    float error_preload;        // 预加载误差

    pid_Kd_pre_e Kd_pre;    // 微分先行使能
    pid_mode mode;              // PID工作模式

} pid_config;

typedef struct pid_struct
{
    pid_config config;
    float error[3];              // 误差
    float error_sum;             // 误差和
    float abs_error;            // 绝对误差

    float feedback;              // 反馈值
    float last_feedback;          // 上次反馈值
    float expected;              // 期望值
    float last_expected;          // 上次期望值

    float speed_ratio;          // 变速积分比例系数
    float output;                // 输出值
    float output_unlimited;      // 未限幅输出值
    float error_delta;           // 误差变化率
}pid_t;


#pragma pack()

void pid_init(pid_t* pid, pid_config* config);
float pid_calc(pid_t* pid, float expected, float feedback);
void pid_set_config_pos(pid_config* config, float Kp, float Ki, float Kd, float errormax, float max_out);
void pid_set_config_comp(pid_config* config, float Kp_rough, float Kp_tiny, float Ki, float Kd, float rangerough, float rangefine, float Kf, float errorpreload, float errormax, float max_out);
void pid_set_config_fusion(pid_config* config, pid_Kd_pre_e Kd_pre, float Kp, float Ki,float kd, float Kf, float dead_band,
                           float Ki_speed_A, float Ki_speed_B, float Ki_separate, float integral_limit, float max_out);