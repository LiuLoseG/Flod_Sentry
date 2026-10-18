#ifndef SYS_ID_H
#define SYS_ID_H

#include <stdint.h>

// 辨识系统状态
typedef enum {
    SYSID_STATE_IDLE = 0,    // 停止辨识
    SYSID_STATE_PREPARE,     // 准备阶段：闭环移动到起始位置
    SYSID_STATE_SWEEP,       // 扫频阶段：开环发送chirp信号
    SYSID_STATE_DONE         // 辨识完成
} SysId_State_e;

// 辨识配置参数
typedef struct {
    float start_freq;        // 起始频率 (Hz)
    float end_freq;          // 结束频率 (Hz)
    float amplitude;         // 激励幅值 (电机电流给定单位)
    float duration;          // 扫频总时间 (s)
    float sample_rate;       // 采样频率 (Hz)

    // 安全限制
    float max_angle;         // 允许的最大角度 (rad)
    float min_angle;         // 允许的最小角度 (rad)
    float start_angle;       // 扫频起始中心角度 (rad)
} SysId_Config_t;

// 辨识系统实例
typedef struct {
    SysId_State_e state;
    SysId_Config_t config;

    // 运行状态变量
    float current_time;      // 当前已运行时间 (s)
    float current_freq;      // 当前瞬时频率 (Hz)
    float output_cmd;        // 当前输出指令
    uint64_t sweep_start_time_us; // 扫频起始 DWT 时间戳 (us)

    // 外部反馈
    float feedback_angle;    // 反馈角度 (rad)
    float feedback_speed;    // 反馈角速度 (rad/s)
} SysId_Instance_t;

/**
 * @brief 初始化系统辨识模块
 */
void SysId_Init(SysId_Config_t *config);

/**
 * @brief 启动系统辨识
 */
void SysId_Start(void);

/**
 * @brief 停止系统辨识
 */
void SysId_Stop(void);

/**
 * @brief 系统辨识任务更新
 */
float SysId_Update(float fb_angle, float fb_speed, float gravity_comp, float dt);

/**
 * @brief 获取当前辨识状态
 */
SysId_State_e SysId_GetState(void);

/**
 * @brief 强制设置当前辨识状态
 */
void SysId_SetState(SysId_State_e state);

/**
 * @brief 获取辨识系统实例指针
 */
SysId_Instance_t* SysId_GetInstance(void);

#endif // SYS_ID_H
