/**
 * @file user_lib.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "math.h"


uint8_t GlobalDebugMode = 7;

//快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

//快速求平方根倒数
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/**
    * @brief   速度规划斜坡函数,使目标输出值缓慢等于期望值
    * @param   target 期望最终输出值
    * @param   current 当前输出值
    * @param   increase_step 加速斜坡值,变化速度(越大越快)
    * @param   decrease_step 减速斜坡值,变化速度(越大越快)
    * @return  返回计算后的输出值
 */
float slope_plan_ramp(float target, float current, float increase_step, float decrease_step)
{
    //static float step_output = 0.0f;
        /* 当前值为正(当前在电机正转) */
    // if (fabsf(current) <= 0.08f)
    // {
    //     current = 0.0f;
    // }

    if (current > 0.0f)         
    {
        if (target > current)   //加速阶段(目标值大于当前值，电机加速正转)
        {
            if (fabsf(current - target) > increase_step)    
            {
                current += increase_step; 
            }
            else 
            {
                current = target; 
            }
        }
        else if (target < current)  //减速阶段(目标值小于当前值，电机减速正转)
        {
            if (fabsf(current - target) > decrease_step)
            {
                current -= decrease_step;
            }
            else
            {
                current = target;
            }
        }
    }
        /* 当前值为负(当前在电机反转) */
    else if (current < 0.0f)        
    {
        if (target < current)   //加速阶段(目标值小于当前值，电机加速反转)
        {
            if (fabsf(current - target) > increase_step)
            {
                current -= increase_step;
            }
            else
            {
                current = target;
            } 
        }
        else if (target > current)  //减速阶段(目标值为+时，先减速为0，再加速到目标值)
        {
            if (fabsf(current - target) > decrease_step)
            {
                current += decrease_step;
            }
            else
            {
                current = target;
            }
        }
    }
        /* 当前值为0(当前在电机不转) */
    else
    {
        if (target > current)   //加速阶段(目标值大于当前值，电机加速正转)
        {
            if (fabsf(current - target) > increase_step)
            {
                current += increase_step;
            }
            else
            {
                current = target;
            }
        }
        else if (target < current)  //加速阶段(目标值小于当前值，电机加速反转)
        {
            if (fabsf(current - target) > increase_step)
            {
                current -= increase_step;
            }
            else
            {
                current = target;
            }
        }
    }

    return current;
}

/**
 * @brief   就近转零斜坡函数,使目标输出值缓慢等于期望值
 * @param   final 期望最终输出值
 * @param   current 当前输出值
 * @param   ramp 斜坡值,变化速度(越大越快)
 * @param   range 回绕范围
 * @return  返回计算后的输出值
 */

float slope_float_ramp(float final, float current, float ramp, float range)
{
    // 计算回绕的一半范围
    float half = range * 0.5f;
    float delta = final - current;

    // 将转角规范到 [-half, half]，获得最短旋转方向
    if (delta > half)  delta -= range;
    else if (delta < -half) delta += range;

    //限幅一步变化量
    float step = (delta > ramp) ? ramp : ((delta < -ramp) ? -ramp : delta);

    float next = current + step;
    if (next >= range) next -= range;
    if (next < 0.0f)   next += range;

    return next;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @retval         返回空
  */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}

//绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

//判断符号位
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

// 浮点数四舍五入为整数运算
int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
  * @brief          最小二乘法初始化
  * @param[in]      最小二乘法结构体
  * @param[in]      样本数
  * @retval         返回空
  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    if (OLS == nullptr || order == 0) return;

    /* 防止重复调用导致内存泄漏: 先释放旧内存 */
    if (OLS->x != nullptr || OLS->y != nullptr)
    {
        OLS_Deinit(OLS);
    }

    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);

    /* pvPortMalloc 可能失败(堆耗尽) */
    if (OLS->x == nullptr || OLS->y == nullptr)
    {
        if (OLS->x != nullptr) { vPortFree(OLS->x); OLS->x = nullptr; }
        if (OLS->y != nullptr) { vPortFree(OLS->y); OLS->y = nullptr; }
        OLS->Order = 0;
        return;
    }

    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          最小二乘法反初始化, 释放动态内存
  * @param[in]      最小二乘法结构体
  */
void OLS_Deinit(Ordinary_Least_Squares_t *OLS)
{
    if (OLS == nullptr) return;

    if (OLS->x != nullptr)
    {
        vPortFree(OLS->x);
        OLS->x = nullptr;
    }

    if (OLS->y != nullptr)
    {
        vPortFree(OLS->y);
        OLS->y = nullptr;
    }

    OLS->Order = 0;
    OLS->Count = 0;
    OLS->k = 0;
    OLS->b = 0;
    OLS->StandardDeviation = 0;
}

/**
  * @brief          最小二乘法拟合
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    /* 兜底: OLS_Init 分配失败时会把 Order 置 0 且 x/y 保持 nullptr,
       此时若继续执行会出现空指针解引用与 Order-1 无符号回绕(0-1 -> 65535) */
    if (OLS == nullptr || OLS->Order < 2 || OLS->x == nullptr || OLS->y == nullptr)
        return;

    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief          最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回斜率k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    /* 兜底: 同 OLS_Update, 未成功分配时返回 0 (调用方应自行降级/舍前馈的微分项) */
    if (OLS == nullptr || OLS->Order < 2 || OLS->x == nullptr || OLS->y == nullptr)
        return 0.0f;

    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief          获取最小二乘法提取信号微分
  * @param[in]      最小二乘法结构体
  * @retval         返回斜率k
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief          最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @param[in]      信号新样本距上一个样本时间间隔
  * @param[in]      信号值
  * @retval         返回平滑输出
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief          获取最小二乘法平滑信号
  * @param[in]      最小二乘法结构体
  * @retval         返回平滑输出
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}


//快速求平方根倒数
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//整数符号函数
int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}

//浮点符号函数
int fsgn(float x) {
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0);
}

//带死区的符号函数
float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}

/*--------------------------------------------------将下面函数放在公用lib运算函数库中--------------------------------------------------------*/

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  transform the bit8 to bit32
*/
uint32_t bit8_to_bit32(uint8_t change_info[4])
{
	union
	{
    uint32_t bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}
//------------------------------------------------------------------------------
uint32_t bit8_to_bit64(uint8_t change_info[8]){
 	union
	{
    uint64_t bit32;
		uint8_t  byte[4];
	}u64val;

  u64val.byte[0] = change_info[0];
  u64val.byte[1] = change_info[1];
  u64val.byte[2] = change_info[2];
  u64val.byte[3] = change_info[3];

	return u64val.bit32;

}
/**
 * @brief  transform the bit8 to float32
*/
float bit8_to_float32(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit32 to bit8
*/
uint8_t bit32_to_bit8(uint8_t Index_need,uint32_t bit32)
{
	union
	{
    uint32_t  bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.bit32 = bit32;

	return u32val.byte[Index_need];
}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit8 to bit16
*/
int16_t bit8_to_bit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit16 to bit8
*/
uint8_t bit16_to_bit8(uint8_t Index_need,int16_t bit16)
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.bit16 = bit16;
	return u16val.byte[Index_need];
}

/**
 * @brief  transform the float to bit16 with absolute value
*/
uint16_t float_abs_to_bit16(float value, float f_max, uint16_t t_max)
{
    float abs_val = fabsf(value);
    if (abs_val > f_max) abs_val = f_max;
    return (uint16_t)((abs_val / f_max) * t_max);
}

/**
 * @brief 
 * @param value 
 * @param f_max 
 * @param t_max 
 * @retval uint32_t: 
 */
uint32_t float_abs_to_bit32(float value, float f_max, uint32_t t_max)
{
    float abs_val = fabsf(value);
    if (abs_val > f_max) abs_val = f_max;
    return (uint32_t)((abs_val / f_max) * t_max);
}

/**
 * @brief  transform the float to int16 with signed mapping
 * @note   Maps [-f_max, +f_max] linearly to [-t_max, +t_max] (signed).
 *         Unlike float_abs_to_bit16, this preserves the sign information.
 */
int16_t float_to_int16(float value, float f_max, uint16_t t_max)
{
    if (value > f_max)  value = f_max;
    if (value < -f_max) value = -f_max;
    return (int16_t)(value / f_max * t_max);
}

/**
 * @brief  inverse transform of float_to_int16
 * @note   Maps int16_t [-t_max, +t_max] linearly to [-f_max, +f_max].
 *         float_to_int16 的逆函数，转换结果互为逆运算。
 */
float int16_to_float(int16_t value, float f_max, uint16_t t_max)
{
    if (value > t_max)  value = t_max;
    if (value < -t_max) value = -t_max;

    return (float)((float)value / (float)t_max * f_max);
}

//------------------------------------------------------------------------------