//
// Created by Lenovo on 2024/3/15.
//

#include "smc_control.h"

void cSMC::Init()
{
    smc.param = SlidingParam();
    smc.param_last = SlidingParam();
    param_last_valid = false;

    smc.error = RError();

    smc.flag = EXPONENT;
    smc.u_max = 0.0f;
    smc.limit = 0.0f;

    smc.u = 0.0f;
    smc.s = 0.0f;
}

bool cSMC::CheckParam(float J, float K, float epsilon, float limit, float u_max, Rmode flag, float pos_esp)
{
    if (flag > EISMC)
    {
        return false;
    }

    if (!std::isfinite(J) || !std::isfinite(K) || !std::isfinite(limit) || !std::isfinite(u_max))
    {
        return false;
    }

    // epsilon 在 Sat() 中作为边界层厚度(除数)使用, 必须为正
    if (!std::isfinite(epsilon) || epsilon < SMC_MIN_DIVISOR)
    {
        return false;
    }

    if (limit < 0.0f || u_max < 0.0f)
    {
        return false;
    }

    if (!std::isfinite(pos_esp) || pos_esp < 0.0f)
    {
        return false;
    }

    return true;
}

float cSMC::RescaleIntegral(float integral, float factor_last, float factor_now)
{
    if (!std::isfinite(integral) || !std::isfinite(factor_last) || !std::isfinite(factor_now))
    {
        return 0.0f;
    }

    // 新参数为 0 表示滑模面里已没有这一项, 直接清零, 同时也避免除零
    if (std::fabs(factor_now) < SMC_MIN_DIVISOR)
    {
        return 0.0f;
    }

    integral *= factor_last / factor_now;

    return std::isfinite(integral) ? integral : 0.0f;
}

bool cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EXPONENT,POWER,VELSMC 参数设定
    // 非法参数一律拒绝, 保持上一次的有效参数, 避免把 0/NaN 写进控制器
    if (!CheckParam(J, K, epsilon, limit, u_max, flag, pos_esp))
    {
        return false;
    }

    // 该重载只服务于不含 p/q/beta 与 c1/c2 的模式, 防止误用后读到未配置的参数
    if (flag != EXPONENT && flag != POWER && flag != VELSMC)
    {
        return false;
    }

    if (!std::isfinite(c))
    {
        return false;
    }

    smc.param.J = J;
    smc.param.K = K;
    smc.param.c = c;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();

    return true;
}
bool cSMC::SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///TFSMC 参数设定
    if (!CheckParam(J, K, epsilon, limit, u_max, flag, pos_esp))
    {
        return false;
    }

    if (flag != TFSMC)
    {
        return false;
    }

    // p 是幂次指数的分母, p 为 0 会让 q/p 变成 inf
    if (!std::isfinite(p) || !std::isfinite(q) || !std::isfinite(beta) || p <= 0.0f || q <= 0.0f)
    {
        return false;
    }

    smc.param.J = J;
    smc.param.K = K;
    smc.param.p = p;
    smc.param.q = q;
    smc.error.pos_error_eps = pos_esp;
    smc.param.beta = beta;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();

    return true;
}

bool cSMC::SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EISMC 参数设定
    if (!CheckParam(J, K, epsilon, limit, u_max, flag, pos_esp))
    {
        return false;
    }

    if (flag != EISMC)
    {
        return false;
    }

    if (!std::isfinite(c1) || !std::isfinite(c2))
    {
        return false;
    }

    smc.param.J = J;
    smc.param.K = K;
    smc.param.c1 = c1;
    smc.param.c2 = c2;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();

    return true;
}

void cSMC::ErrorUpdate(float target, float pos_now, float vol_now) //位置环误差更新
{
    // 单帧非法数据整帧丢弃, 避免 NaN/inf 污染微分、积分与滑模面
    if (!std::isfinite(target) || !std::isfinite(pos_now) || !std::isfinite(vol_now))
    {
        return;
    }

    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.p_error = pos_now - target;
    smc.error.v_error = vol_now - smc.error.tar_differential;
    smc.error.tar_last = smc.error.tar_now;

    smc.error.p_error_integral += (float)(smc.error.p_error * SAMPLE_PERIOD); ///位置误差积分项

    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}

void cSMC::ErrorUpdate(float target,float vol_now) //速度环误差更新
{
    if (!std::isfinite(target) || !std::isfinite(vol_now))
    {
        return;
    }

    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

//    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.v_error = vol_now - smc.error.tar_now;

    smc.error.v_error_integral += (float)(smc.error.v_error * SAMPLE_PERIOD); ///速度误差积分项
//    if(std::abs(smc.error.v_error_integral) > smc.error.v_error_integral_max) //积分限幅
//    {
//        smc.error.v_error_integral = smc.error.v_error_integral_max;
//    }

    smc.error.tar_last = smc.error.tar_now;

//    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}
void cSMC::Clear()
{
    // 只复位运行状态, 保留 SetParam 下发的死区阀值, 否则运行中调用一次 Clear 就会把死区静默关掉
    const float pos_error_eps = smc.error.pos_error_eps;
    const float vol_error_eps = smc.error.vol_error_eps;

    smc.error = RError();
    smc.error.pos_error_eps = pos_error_eps;
    smc.error.vol_error_eps = vol_error_eps;

    smc.u = 0.0f;
    smc.s = 0.0f;
}
void cSMC::Integval_Clear()
{
    smc.error.v_error_integral = 0;
    smc.error.p_error_integral = 0;
}

float cSMC::SmcCalculate()
{
    // u/fun 必须初始化: 若 smc.flag 越界导致 switch 不命中, 未初始化值会直接送进执行器
    float u = 0.0f;
    float fun = 0.0f;

    switch (smc.flag) {
        case EXPONENT:///线性滑模面，指数趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                smc.u = 0.0f; 
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun + smc.error.tar_differential_second); //控制器计算,指数趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率

            break;
        case POWER:///线性滑模面，幂次趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                smc.u = 0.0f; 
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun + smc.error.tar_differential_second); //控制器计算,幂次趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun); //控制器计算,幂次趋近率

            break;
        case TFSMC:///tfsmc
        {
            float pos_pow;//tfsmc 位置 幂

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                smc.u = 0.0f; 
                return 0;
            }

            pos_pow = std::pow(std::abs(smc.error.p_error),smc.param.q/smc.param.p);
            if(smc.error.p_error<=0) pos_pow = -pos_pow;


            smc.s = smc.param.beta * pos_pow + smc.error.v_error; //滑模面

            fun = Sat(smc.s);//饱和函数消除抖动

            // pos_pow/p_error 等价于 |p_error|^(q/p-1), 误差趋 0 时该项发散,
            // 用下限截断保证结果有限, 同时避免 p_error==0 时的除零
            const float error_abs = std::fmax(std::abs(smc.error.p_error), SMC_MIN_DIVISOR);
            const float power_gain = std::pow(error_abs, smc.param.q / smc.param.p - 1.0f);

            u = smc.param.J * (smc.error.tar_differential_second//目标值的二阶导 暂定是否删除，需测试
                             -smc.param.K * smc.s //s*K
                             -smc.param.epsilon * fun  //epsilon*SAT(S)
                             -smc.error.v_error * ((smc.param.q * smc.param.beta) / smc.param.p) * power_gain); //控制器计算
            break;
        }
        case VELSMC:///比例积分滑模面，指数趋近律，速度控制
            smc.s = smc.error.v_error + smc.param.c * smc.error.v_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * (smc.error.tar_differential - (smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算，速度控制
            break;

        case EISMC:///比例积分滑模面，指数趋近律，位置控制
            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                smc.u = 0.0f; 
                return 0;
            }

            smc.s = smc.param.c1 * smc.error.p_error + smc.error.v_error + smc.param.c2 * smc.error.p_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * ( (-smc.param.c1 * smc.error.v_error)- smc.param.c2 * smc.error.p_error - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率
            break;

        default:///非法模式, 输出 0, 不允许把未定义的控制量送进电机
            u = 0.0f;
            break;
    }

    smc.error.error_last = smc.error.p_error; //更新上一步的误差

    // 数值安全: NaN/inf 与任何数比较都为假, 下面的限幅拦不住, 必须先归零并复位积分
    if (!std::isfinite(u))
    {
        smc.error.p_error_integral = 0.0f;
        smc.error.v_error_integral = 0.0f;
        smc.s = 0.0f;
        u = 0.0f;
    }

    //控制量限幅
    if (u > smc.u_max)
    {
        u = smc.u_max;
    }
    if (u < -smc.u_max)
    {
        u = -smc.u_max;
    }
    smc.u = u;
    return u;
}

void cSMC::OutContinuation()
{
    // 参数热更新时按模式重标定积分项, 保持滑模面 s 连续
    if (param_last_valid)
    {
        switch (smc.flag)
        {
        case VELSMC:// s = v_error + c * ∫v_error, 要让 s 不变则 ∫v_error 按 c_last/c 缩放
            smc.error.v_error_integral = RescaleIntegral(smc.error.v_error_integral, smc.param_last.c, smc.param.c);
            break;

        case EISMC:// s = c1*p_error + v_error + c2 * ∫p_error, 要让 s 不变则 ∫p_error 按 c2_last/c2 缩放
            smc.error.p_error_integral = RescaleIntegral(smc.error.p_error_integral, smc.param_last.c2, smc.param.c2);
            break;

        default:// EXPONENT / POWER / TFSMC 滑模面不含积分项, 无需重标定
            break;
        }
    }

    smc.param_last = smc.param;
    param_last_valid = true;
}


// 符号函数
float cSMC::Signal(float s)
{
    if (s > 0)
        return 1;
    else if (s == 0)
        return 0;
    else
        return -1;
}

//饱和函数
float cSMC::Sat(float s)
{
    // s 非有限时给 0, 避免把 NaN 带进 epsilon 项
    if (!std::isfinite(s))
    {
        return 0.0f;
    }

    // epsilon 是边界层厚度(除数), 非法或过小时退化为符号函数, 避免除零与溢出
    if (!(smc.param.epsilon >= SMC_MIN_DIVISOR))
    {
        return Signal(s);
    }

    float y;
    y = s / smc.param.epsilon;
    if (std::abs(y) <= smc.limit)
        return y;
    else
        return Signal(y);
}

const Sliding &cSMC::getSmc() const {
    return smc;
}

float cSMC::Out() {
    return smc.u;
}

void cSMC::SetOut(float out) {
    smc.u = out;
}


