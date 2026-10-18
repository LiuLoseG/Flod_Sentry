/**
 * @file safe_daemon_process.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 安全守护进程: 统一调用各电机 alive_check() 做离线检测
 * @note  设计要点:
 *        1. 一个任务: 全工程只有 safe_daemon_task() 一个任务做存活检测;
 *        2. 一个调用点: 全工程只有该任务内一处调用各设备的 check_client(= alive_check());
 *        3. 只检测不动作: 本层只维护"在线/离线"状态与统计, 不做安全动作(清零/禁止发射/锁存),
 *           上层需要时用 safe_daemon_process_is_online() / safe_daemon_process_all_online() 自行判断;
 *        4. 无动态内存: 注册表为静态数组, init 阶段注册, 运行期只读(除状态与统计字段);
 *        5. 周期可配: 掉线检测时延 = 该设备注册时的 period_ms。
 * @date 2026-09-18
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
// 以下三个驱动头均为 C++ 专有头(含 class), 用条件编译包起来,
// 使本头文件在 C 编译单元(如 freertos.c)中也能被 include(仍可拿到任务原型)
#include "dm_motor.h"
#include "dji_motor_new.h"
#include "lk_motor.h"
#endif

/*====================================== 配置项 ======================================*/

// 最大注册设备数(当前占用 6: 云台 3 + 发射 3, 余量 2; 新增设备需同步调大)
#define SAFE_DAEMON_PROCESS_MAX 8

// 任务轮询周期(ms), 决定检测周期的时间分辨率
#define SAFE_DAEMON_PROCESS_POLL_PERIOD_MS 1

// 默认检测周期(ms): 无副作用设备(DJI)用, 反馈周期约1ms, 连续100ms无新帧即判定掉线
#define SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS 100

// 掉线时会自动重发使能的设备(DM/LK)检测周期(ms), 放宽以避免刷屏占用CAN带宽
#define SAFE_DAEMON_PROCESS_PERIOD_MOTOR_RETRY_MS 20

// 上电静默期(ms): 等待各电机建立通信, 避免上电首轮把尚未反馈的电机误判为掉线
#define SAFE_DAEMON_PROCESS_STARTUP_DELAY_MS 1000

#ifdef __cplusplus
extern "C" {
#endif

/*====================================== 类型定义 ======================================*/

// 存活检测回调(类型擦除: 内部即 device->alive_check())
typedef void (*safe_daemon_process_check_client_t)(void *device);

// 在线状态查询回调(1=在线: 读驱动的 get_status())
typedef uint8_t (*safe_daemon_process_status_client_t)(void *device);

// 注册表结构体
typedef struct
{
    const char *name;                                    // 设备名(调试/Ozone 观察用)
    void *device;                                        // 设备对象指针
    safe_daemon_process_check_client_t check_client;      // 存活检测回调
    safe_daemon_process_status_client_t status_client;    // 在线状态查询回调
    uint32_t period_ms;                                  // 检测周期, ms
    uint32_t last_check_tick;                            // 上次检测时刻, ms
    uint32_t check_count;                                // 累计检测次数(为0说明守护任务没跑)
    uint32_t offline_count;                              // 累计掉线次数
    uint8_t online;                                      // 当前在线状态, 1=在线
} safe_daemon_process_table_t;

/*====================================== 接口函数 ======================================*/

/**
 * @brief 设备注册函数
 * @param name 设备名(字符串常量, 仅调试用)
 * @param device 设备对象指针
 * @param check_client 存活检测回调(一般由 SAFE_DAEMON_PROCESS_REGISTER 宏生成)
 * @param status_client 在线状态查询回调(同上)
 * @param period_ms 检测周期(ms), 0 则使用默认周期
 * @return 1=注册成功; 0=注册失败(参数为空 / 同一设备重复注册 / 超出 SAFE_DAEMON_PROCESS_MAX)
 * @note 由各模块在 init 阶段(任务上下文)调用; 运行期注册表只读。
 *       两个回调必须来自同一设备对象(二者读的是同一份 motor_status: alive_check() 写, get_status() 读),
 *       传错会出现"检测对象与判据不一致"。
 */
uint8_t safe_daemon_process_register(const char *name, void *device,
                                     safe_daemon_process_check_client_t check_client,
                                     safe_daemon_process_status_client_t status_client,
                                     uint32_t period_ms);

/**
 * @brief 安全守护任务
 * @note 函数名必须与 CubeMX 线程的入口函数一致(当前线程名 `safe_daemon` → `safe_daemon_task`),
 *       且该线程的代码生成选项必须是 `As weak`: CubeMX 生成 weak 桩, 由本文件的强定义覆盖;
 *       若选 `Default`, CubeMX 会生成强定义 → 链接期 multiple definition。
 * @note 全工程唯一的 alive_check() 调用点: 遍历注册表, 到期即调用存活检测并刷新在线状态
 */
void safe_daemon_task(void *argument);

/**
 * @brief 查询某设备当前是否在线
 * @param device 设备对象指针
 * @return 1=在线; 0=离线 或 该设备未注册
 * @note 未注册返回 0(不可用), 避免漏注册被误认为在线
 */
uint8_t safe_daemon_process_is_online(const void *device);

/**
 * @brief 获取所有已注册设备是否全部在线
 * @return 1=全部在线, 0=存在掉线设备
 */
uint8_t safe_daemon_process_all_online(void);

/**
 * @brief 获取注册表指针, 便于观察或自行遍历(只读: 状态与统计由守护任务维护)
 * @param count 输出当前已注册设备数, 可为空
 */
const safe_daemon_process_table_t *safe_daemon_process_get_table(uint8_t *count);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/*================================= C++ 适配器(类型擦除) =================================*/

/**
 * @brief 把设备成员函数 void T::alive_check() 适配成 safe_daemon_process_check_client_t
 */
template <typename T>
inline void safe_daemon_process_check_adapter(void *device)
{
    static_cast<T *>(device)->alive_check();
}

/**
 * @brief 各驱动"在线"判据的枚举值(新增驱动类型时在此补一条特化即可)
 */
template <typename StatusEnum>
struct safe_daemon_process_online_value
{
    // 未特化时给出可读的编译期提示(否则报错是一长串 incomplete type)
    static_assert(sizeof(StatusEnum) == 0,
                  "该驱动的状态枚举未特化: 请在 safe_daemon_process.h 中补 "
                  "safe_daemon_process_online_value<该枚举> 特化");
};

template <>
struct safe_daemon_process_online_value<dm_motor_status>
{
    static constexpr dm_motor_status value = Motor_DM_Status_Enable;
};

template <>
struct safe_daemon_process_online_value<dji_motor_status_e>
{
    static constexpr dji_motor_status_e value = Motor_DJI_Status_Enable;
};

template <>
struct safe_daemon_process_online_value<lk_motor_status>
{
    static constexpr lk_motor_status value = Motor_LK_Status_Enable;
};

/**
 * @brief 把设备成员函数 get_status() 适配成 safe_daemon_process_status_client_t
 * @note 以 get_status() 的返回类型自动匹配"在线"枚举值, 因此业务侧无需再写状态适配函数
 */
template <typename T>
inline uint8_t safe_daemon_process_status_adapter(void *device)
{
    typedef decltype(static_cast<T *>(nullptr)->get_status()) status_enum_t;
    return (static_cast<T *>(device)->get_status() == safe_daemon_process_online_value<status_enum_t>::value) ? 1 : 0;
}

/**
 * @brief 注册待监控设备(需具备 void alive_check() 与 get_status() const)
 * @param name 设备名
 * @param obj 设备对象(直接写对象表达式, 如 gimbal.pitch_motor)
 * @param period_ms 检测周期(ms)
 * @note 例: SAFE_DAEMON_PROCESS_REGISTER("pitch_motor", gimbal.pitch_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_RETRY_MS);
 */
#define SAFE_DAEMON_PROCESS_REGISTER(name, obj, period_ms)                                   \
    safe_daemon_process_register((name), static_cast<void *>(&(obj)),                         \
                                 &safe_daemon_process_check_adapter<decltype(obj)>,           \
                                 &safe_daemon_process_status_adapter<decltype(obj)>,          \
                                 (period_ms))
#endif




