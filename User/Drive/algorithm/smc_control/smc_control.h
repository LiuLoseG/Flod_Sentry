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

// 固定底层类型为 uint8_t: 枚举值域为 0..255, 超出 Rmode 值域的静态转换不再落入 3 bit 值域
typedef enum : uint8_t {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

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


class cSMC
{
public:
    void Init();

    // 参数非法(含 NaN/inf 或与模式不匹配)时返回 false, 并保持上一次的有效参数不变
    bool SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //滑膜参数设定 EXPONENT,POWER,VELSMC 参数
    bool SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//滑膜参数设定 TFSMC 参数
    bool SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC 参数设定

    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜位置误差更新
    void ErrorUpdate(float target,float vol_now); //滑模速度误差更新
    void Clear();
    void Integval_Clear();

    float SmcCalculate(); //滑模控制器计算函数
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //参量提取

private:


    Sliding smc;
    bool param_last_valid = false; //param_last 是否已由一次成功的 SetParam 写入

    void OutContinuation(); //输出连续化
    static bool CheckParam(float J, float K, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //参数合法性检查
    static float RescaleIntegral(float integral, float factor_last, float factor_now); //积分项重标定
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};
