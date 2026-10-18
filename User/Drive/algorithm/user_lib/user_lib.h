/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#pragma once

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

#ifndef RPM_TO_ANGLE_SEC
#define RPM_TO_ANGLE_SEC 6.0f            // ×360°/60sec
#endif

#ifndef ANGLE_TO_RPM
#define ANGLE_TO_RPM 0.166666667f        // ×60sec/360°
#endif

#ifndef RPM_TO_RAD_SEC
#define RPM_TO_RAD_SEC   0.104719755f    // ×2pi/60sec
#endif

#ifndef RAD_TO_RPM
#define RAD_TO_RPM       9.549296586f     // ×60sec/2pi
#endif  

#ifndef DEGREE_TO_RAD
#define DEGREE_TO_RAD 0.01745329252f    //角度转弧度(rad/180)
#endif

#ifndef RAD_TO_DEGREE
#define RAD_TO_DEGREE 57.295779513f     //弧度转角度(180/pi)
#endif

#ifndef SQRT_2_2
#define SQRT_2_2 0.707106781185f                //(根号2)/2，cos_sin(45°)
#endif

#ifndef SQRT_2_4
#define SQRT_2_4 0.35355339059f                 //(根号2)/4, cos_sin(45°)/2
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

//快速开方
float Sqrt(float x);

//速度规划斜坡函数，缓慢接近目标值。
float slope_plan_ramp(float target, float current, float increase_step, float decrease_step);

//就近转位斜坡函数，缓慢接近目标值
float slope_float_ramp(float final, float current, float ramp, float range);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
//斜波函数计算
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//绝对限制
float abs_limit(float num, float Limit);
//判断符号位
float sign(float value);
//浮点死区
float float_deadband(float Value, float minValue, float maxValue);
// int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
float float_constrain(float Value, float minValue, float maxValue);
//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);
//角度 °限幅 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Deinit(Ordinary_Least_Squares_t *OLS);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

float invSqrt(float x);
int sgn(int x);
int fsgn(float x);
float sgn_like(float x, float d);

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

uint32_t bit8_to_bit32(uint8_t change_info[4]);
float bit8_to_float32(uint8_t change_info[4]);
uint32_t bit8_to_bit64(uint8_t change_info[8]);
int16_t bit8_to_bit16(uint8_t change_info[2]);
uint8_t bit16_to_bit8(uint8_t Index_need,int16_t bit16);
uint8_t bit32_to_bit8(uint8_t Index_need,uint32_t bit32);
uint16_t float_abs_to_bit16(float value, float f_max, uint16_t t_max);
uint32_t float_abs_to_bit32(float value, float f_max, uint32_t t_max);

int16_t float_to_int16(float value, float f_max, uint16_t t_max);
float int16_to_float(int16_t value, float f_max, uint16_t t_max);

// 平方函数
static inline float square(float x)
{
    return x * x;
}
