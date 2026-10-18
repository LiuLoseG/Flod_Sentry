#include "sys_phased_freq.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 定义辨识实例
static SysId_Instance_t sys_id_inst = {};

void SysId_Init(SysId_Config_t *config) {
    if (config == NULL) return;
    sys_id_inst.config = *config;
    sys_id_inst.state = SYSID_STATE_IDLE;
    sys_id_inst.current_time = 0.0f;
    sys_id_inst.current_freq = 0.0f;
    sys_id_inst.output_cmd = 0.0f;
    sys_id_inst.sweep_start_time_us = 0;
}

void SysId_Start(void) {
    sys_id_inst.state = SYSID_STATE_PREPARE;
    sys_id_inst.current_time = 0.0f;
    sys_id_inst.current_freq = 0.0f;
    sys_id_inst.output_cmd = 0.0f;
    sys_id_inst.sweep_start_time_us = 0;
}

void SysId_Stop(void) {
    sys_id_inst.state = SYSID_STATE_IDLE;
    sys_id_inst.current_time = 0.0f;
    sys_id_inst.current_freq = 0.0f;
    sys_id_inst.output_cmd = 0.0f;
    sys_id_inst.sweep_start_time_us = 0;
}

// 线性 Chirp 信号公式：
// freq(t) = f0 + (f1 - f0) / T * t
// phase(t) = 2*pi * integral(freq(t) dt) = 2*pi * (f0*t + (f1 - f0) / (2*T) * t^2)
float SysId_Update(float fb_angle, float fb_speed, float gravity_comp, float dt) {
    (void)dt;
    sys_id_inst.feedback_angle = fb_angle;
    sys_id_inst.feedback_speed = fb_speed;
    sys_id_inst.output_cmd = 0.0f;

    // 状态机处理
    switch (sys_id_inst.state) {
        case SYSID_STATE_IDLE:
            return 0.0f;

        case SYSID_STATE_DONE:
            // DONE 只作为单帧过渡态，避免上层一直卡在辨识分支
            SysId_Stop();
            return 0.0f;

        case SYSID_STATE_PREPARE:
            // 准备阶段：等待闭环控制将云台移动到 start_angle
            // 这里为了安全，我们需要判断是否到达目标点
            // （可以在应用层做闭环移动，当到达 start_angle 后切换为 SWEEP 状态）
            // 这里简单起见，由应用层决定何时进入 SWEEP。
            // 暂时返回0，应用层此时应执行正常的闭环控制。
            return 0.0f;

        case SYSID_STATE_SWEEP: {
            // 安全检查：如果超出角度范围，立即停止辨识
            if (fb_angle > sys_id_inst.config.max_angle ||
                fb_angle < sys_id_inst.config.min_angle) {
                SysId_Stop();
                return 0.0f; // 紧急停止
            }

            if (sys_id_inst.sweep_start_time_us == 0) {
                            sys_id_inst.sweep_start_time_us = bsp_dwt_get_timeline_us();
            }

                        uint64_t now_us = bsp_dwt_get_timeline_us();
            uint64_t elapsed_us = now_us - sys_id_inst.sweep_start_time_us;
            sys_id_inst.current_time = (float)elapsed_us * 1e-6f;
            if (sys_id_inst.current_time < 0.0f) {
                sys_id_inst.current_time = 0.0f;
            }

            // 检查是否结束
            if (sys_id_inst.current_time >= sys_id_inst.config.duration) {
                sys_id_inst.state = SYSID_STATE_DONE;
                sys_id_inst.current_time = sys_id_inst.config.duration;
                sys_id_inst.current_freq = sys_id_inst.config.end_freq;
                sys_id_inst.output_cmd = 0.0f;
                return 0.0f;
            }

            // 计算瞬时频率和相位
            float t = sys_id_inst.current_time;
            float T = sys_id_inst.config.duration;
            float f0 = sys_id_inst.config.start_freq;
            float f1 = sys_id_inst.config.end_freq;

            // f(t) = f0 + k * t，其中 k = (f1 - f0) / T
            float k = (f1 - f0) / T;
            sys_id_inst.current_freq = f0 + k * t;

            // phase(t) = 2*pi * (f0 * t + 0.5 * k * t^2)
            float phase = 2.0f * M_PI * (f0 * t + 0.5f * k * t * t);

            // 激励信号：A * sin(phase)
            float excitation = sys_id_inst.config.amplitude * sinf(phase);

            // 总输出 = 激励信号 + 重力补偿前馈
            // （因为机械无法配平，必须有重力补偿才能保证扫频是在平衡点附近的小范围震荡）
            sys_id_inst.output_cmd = excitation + gravity_comp;

            return sys_id_inst.output_cmd;
        }
        default:
            break;
    }

    return 0.0f;
}

SysId_State_e SysId_GetState(void) {
    return sys_id_inst.state;
}

// 供应用层强制修改状态（比如在 Prepare 阶段闭环到位后切换到 Sweep）
void SysId_SetState(SysId_State_e state) {
    sys_id_inst.state = state;
    if (state == SYSID_STATE_SWEEP) {
        sys_id_inst.current_time = 0.0f;
        sys_id_inst.current_freq = sys_id_inst.config.start_freq;
        sys_id_inst.output_cmd = 0.0f;
        sys_id_inst.sweep_start_time_us = bsp_dwt_get_timeline_us();
    } else if (state != SYSID_STATE_DONE) {
        sys_id_inst.sweep_start_time_us = 0;
    }
}

SysId_Instance_t* SysId_GetInstance(void) {
    return &sys_id_inst;
}
