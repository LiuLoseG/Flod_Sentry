#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GRAY_SENSOR_NUM
#define GRAY_SENSOR_NUM     8       // 灰度传感器数量：3 / 5 / 8
#endif

// N bit 全黑掩码，例如 N=5 时 GRAY_ALL_BLACK = 0x1F
#define GRAY_ALL_BLACK      ((uint8_t)((1 << GRAY_SENSOR_NUM) - 1))
#define GRAY_ALL_WHITE      ((uint8_t)0x00)
// 位置范围：N=5 → ±2.0，N=3 → ±1.0，N=8 → ±3.5
#define GRAY_POS_MAX        ((GRAY_SENSOR_NUM - 1) / 2.0f)

/*
 * GPIO 引脚映射类型
 *
 * 用户需在调用方定义 const 数组，**从左到右**排列传感器：
 *   static const GrayPin my_pins[GRAY_SENSOR_NUM] = {
 *       {GPIOB, GPIO_PIN_4},   // 最左传感器
 *       {GPIOB, GPIO_PIN_3},
 *       {GPIOB, GPIO_PIN_2},   // 正中传感器
 *       {GPIOB, GPIO_PIN_1},
 *       {GPIOB, GPIO_PIN_0}    // 最右传感器
 *   };
 **/
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} gray_pin;

struct gray_sensor_t;

/*
 * 用户回调函数类型
 *
 * 在脱线或检测到停止线时，库会自动调用对应的回调，
 * 用户可在其中添加自定义处理逻辑（如减速、停车、转向）。
 *
 * 若不需要回调，保持为 NULL 即可。
 */

/**
 * @brief 脱线回调
 * @param s       触发事件的传感器对象
 * @param user    用户自定义数据指针（由 on_lost_user 传入）
 * @param cnt     已连续脱线的周期数（lost_cnt）
 */
typedef void (*gray_on_lost_t)(struct gray_sensor_t *s, void *user, uint8_t cnt);

/**
 * @brief 停止线（全黑）回调
 * @param s       触发事件的传感器对象
 * @param user    用户自定义数据指针（由 on_stop_line_user 传入）
 * @param cnt     停止线累计检测次数（stop_line_cnt）
 */
typedef void (*gray_on_stop_line_t)(struct gray_sensor_t *s, void *user, uint8_t cnt);

/**
 * @brief 脱线恢复回调（从全白→正常的第一周期触发）
 * @param s       传感器对象
 * @param user    用户自定义数据指针
 * @param pos     恢复后的软切入位置
 */
typedef void (*gray_on_recover_t)(struct gray_sensor_t *s, void *user, float pos);


typedef struct gray_sensor_t
{
    const gray_pin *pins;                   // 灰度传感器引脚映射表指针，长度 GRAY_SENSOR_NUM
    uint8_t sensor_val[GRAY_SENSOR_NUM];    // 各探头值：1=黑线，0=白底
    uint8_t state;                          // n bit 掩码：bit0=最左

    float position;                         // 连续位置偏差，0=正中，正=偏右
    float backup;                           // 最近一次有效位置（脱线时沿用）

    uint32_t stop_line_cnt;                 // 全黑检测累计次数
    uint32_t lost_cnt;                      // 脱线持续周期数
    uint32_t lost_cnt_prev;                 // 上一周期 lost_cnt，用于检测脱线→恢复的上升沿

    // 回调函数及用户数据指针
    gray_on_lost_t      on_lost;            // 脱线回调
    void               *on_lost_user;       // 脱线回调用户参数

    gray_on_stop_line_t on_stop_line;        // 停止线回调
    void               *on_stop_line_user;   // 停止线回调用户参数

    gray_on_recover_t   on_recover;          // 脱线恢复回调
    void               *on_recover_user;     // 恢复回调用户参数

} gray_sensor_t;

/**
 * @brief 初始化灰度传感器，并注册用户回调
 * @param s                   传感器对象
 * @param pin_map             引脚映射表（长度 GRAY_SENSOR_NUM 的 const 数组）
 * @param on_lost             脱线回调（可 NULL）
 * @param on_lost_user        脱线回调用户参数（可 NULL）
 * @param on_stop_line        停止线回调（可 NULL）
 * @param on_stop_line_user   停止线回调用户参数（可 NULL）
 * @param on_recover          脱线恢复回调（可 NULL）
 * @param on_recover_user     恢复回调用户参数（可 NULL）
 */
void grayscale_init(gray_sensor_t *s, const gray_pin *pin_map,
                    gray_on_lost_t on_lost, void *on_lost_user,
                    gray_on_stop_line_t on_stop_line, void *on_stop_line_user,
                    gray_on_recover_t on_recover, void *on_recover_user);

/**
 * @brief 更新灰度传感器（每个控制周期调用一次）
 *
 * 流程：
 *   1. 读取 N 路 GPIO → state
 *   2. 保存上一周期 position 用作软切入
 *   3. 根据 state 决策：
 *      全白 → 脱线，沿用 backup
 *      全黑 → 停止线/十字，归中并计数
 *      其他 → 加权质心法计算 position
 *   4. 脱线→恢复的上升沿做 0.5 软切入
 */
void grayscale_update(gray_sensor_t *s);

#ifdef __cplusplus
}

#endif