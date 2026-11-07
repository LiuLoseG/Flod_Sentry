//
// Created by Lenovo on 2024/3/15.
//

#pragma once

#include <cmath>
#include <cstdint>

#define SAMPLE_PERIOD 0.001
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

// 分母保护阈值: 绝对值小于该值的分母按 0 处理, 避免除零产生 NaN / inf
#define SMC_MIN_DIVISOR 1e-6f

// 滑模控制器模式
typedef enum : uint8_t {
    EXPONENT,   // 线性关系滑模面， 指数趋近率 -> 适用于Yaw(推荐)
    POWER,      // 线性关系滑模面， 幂次趋近率 -> 适用于Yaw(较推荐)
    TFSMC,      // 快速终端滑模控制 -> 适用于Yaw(较不推荐)
    VELSMC,     // 比例积分关系滑模面， 指数趋近率 -> 适用于摩擦轮、底盘电机等速度控制场景
    EISMC       // 比例积分关系滑模面， 指数趋近率 -> 适用于Pitch，拨弹轮
} Rmode;

// 滑模控制器误差结构体
typedef struct {

    float tar_now = 0.0f;//当前目标值
    float tar_last = 0.0f;//上一次目标值
    float tar_differential = 0.0f;//目标值一阶导
    float tar_differential_last = 0.0f;//上一次目标值一阶导
    float tar_differential_second = 0.0f;//目标值二阶导

    float pos_get = 0.0f;//当前位置
    float vol_get = 0.0f;//当前速度

    float p_error = 0.0f;//位置误差
    float v_error = 0.0f;//速度误差（位置误差一阶导）

    float p_error_integral = 0.0f;//位置误差积分
    float v_error_integral = 0.0f;//速度误差积分
//    float v_error_integral_max; //积分限幅

    float pos_error_eps = 0.0f;   // 位置误差死区阀值
    float vol_error_eps = 0.0f;   // 速度误差死区阀值
    float error_last = 0.0f;
}RError;

// 滑模控制器结构体
typedef struct {
    float J = 0.0f;
    float K = 0.0f;
    float c = 0.0f;

    float c1 = 0.0f;   //EIsmc参数
    float c2 = 0.0f;   //EIsmc参数

    float p = 0.0f;    //tfsmc参数，正奇数 p>q
    float q = 0.0f;    //tfsmc参数，正奇数
    float beta = 0.0f; //tfsmc参数，正数
    float epsilon = 0.0f; //ε噪声上限
}SlidingParam;

// 滑模控制器数据结构体
typedef struct {
    float u = 0.0f; //控制器输出
    float s = 0.0f; //滑模面计算储存

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max = 0.0f; //输出限幅
    Rmode flag = EXPONENT; //控制器模式选择
    float limit = 0.0f; //饱和函数上下限
}Sliding;

// 滑模控制器类
class cSMC
{
public:

    // 初始化函数
    void Init();

    // 参数设置函数
    bool SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //滑膜参数设定 EXPONENT,POWER,VELSMC 参数
    bool SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//滑膜参数设定 TFSMC 参数
    bool SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC 参数设定

    // 误差更新函数
    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜位置误差更新
    void ErrorUpdate(float target,float vol_now); //滑模速度误差更新

    // 控制器计算中间值全清空
    void Clear();

    // 积分部分误差清空
    void Integval_Clear();

    // 滑模控制器计算函数
    float SmcCalculate(); 

    // 获取控制器输出
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //参量提取

private:

    Sliding smc;
    bool param_last_valid = false; //param_last 是否已由一次成功的 SetParam 写入

    // 切换参数时输出连续化，防止因积分系数突变时导致控制器输出突变
    void OutContinuation();
    static bool CheckParam(float J, float K, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //参数合法性检查
    static float RescaleIntegral(float integral, float factor_last, float factor_now); //积分项重标定
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};

// ε 为干扰绝对值上限，对效果影响不大。
// J 为转动惯量，可以找机械哥获取后微调。
// k 值太大会导致高频抖震，c 值太大会出现大幅度缓慢震荡。提高 k 值可减小
// 超调并且提高响应速度，提高 c 值也可提高响应速度，但会导致超调加重，需平衡
// 两者影响。Pitch 中，c2 与 c1 相比会小很多。这些均为个人经验，可能会对不同结
// 构机器有些出入。
// 死亡后要清除Pitch和速度控制中的积分变量值，否则会导致其持续累积而引
// 起Pitch卡死及超射速问题！
