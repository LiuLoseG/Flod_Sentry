/**
 * @file safe_daemon_process.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 安全守护进程: 统一调用各电机 alive_check() 做离线检测
 * @note  设计要点:
 *        1. 一个任务: 全工程只有 safe_daemon_task() 一个任务做存活检测;
 *        2. 一个调用点: 全工程只有该任务内一处调用各设备的 check_client(= alive_check());
 *        3. 只检测不动作: 本层只维护"在线/离线"状态与统计, 不做安全动作(清零/禁止发射/锁存),
 *           上层需要时用 safe_daemon_process_is_online() / safe_daemon_process_all_online() 自行判断;
 *        4. 无动态内存: 注册表为静态数组, 注册在临界区内完成(可在任意任务里调用), 运行期只读(除状态与统计字段);
 *        5. 周期可配: 掉线检测时延 = 该设备注册时的 period_ms;
 *        6. 注册失败可观测: 参数为空 / 重复注册 / 超出容量时计 reject 次数, 不破坏已发布表项;
 *        7. 状态可细分: is_online() 只给"此刻在线否", get_device_status() 额外给出
 *           "是否已评估过"与"已连续离线多久", 供上层做去抖/锁存而无需自己计时。
 * @date 2026-09-18
 * @version 0.4
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

// 最大注册设备数(当前占用 17: 云台 4 + 底盘 8 + 发射 3 + PC 视觉 1 + IMU 1, 余量 3; 新增设备需同步调大)
// 超出该值后注册失败并累加 safe_daemon_process_reject_count, 不会覆盖已发布的表项(漏监控不会静默)
#define SAFE_DAEMON_PROCESS_MAX 10

// 最小休眠粒度(ms): 守护任务按"最近一台设备的到期时刻"休眠, 该值仅在注册表为空时兜底
#define SAFE_DAEMON_PROCESS_POLL_PERIOD_MS 1

// 默认检测周期(ms): 反馈周期约1ms, 连续100ms无新帧即判定掉线(DJI 三个类的 alive_check() 无总线副作用)
#define SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS 100

// 非电机设备(IMU / PC 视觉等)的检测周期(ms): 与电机同窗口, 单独命名避免"电机周期"用在传感器上
#define SAFE_DAEMON_PROCESS_PERIOD_SENSOR_MS 100

// 上电静默期(ms): 等待各电机建立通信, 避免上电首轮把尚未反馈的电机误判为掉线
#define SAFE_DAEMON_PROCESS_STARTUP_DELAY_MS 1000

#ifdef __cplusplus
extern "C" {
#endif

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
    uint32_t check_count;                                // 累计检测次数(为0说明守护任务没跑/尚未评估)
    uint32_t offline_count;                              // 累计掉线次数
    uint32_t offline_since_tick;                         // 本次掉线的起始时刻, ms(仅 offline=0 时有效)
    uint8_t online;                                      // 当前在线状态, 1=在线
} safe_daemon_process_table_t;

// 单设备状态快照(供上层做去抖/锁存, 无需自己计时)
typedef struct
{
    uint8_t online;        // 1=在线, 0=离线
    uint8_t evaluated;     // 1=至少完成过一次检测; 0=仍在启动静默期或刚注册
    uint32_t offline_count; // 累计掉线次数
    uint32_t offline_ms;    // 已连续离线时长(ms), 在线时为 0
} safe_daemon_process_device_status_t;

/**
 * @brief 设备注册函数
 * @param name 设备名(字符串常量, 仅调试用)
 * @param device 设备对象指针
 * @param check_client 存活检测回调(一般由 SAFE_DAEMON_PROCESS_REGISTER 宏生成)
 * @param status_client 在线状态查询回调(同上)
 * @param period_ms 检测周期(ms), 0 则使用默认周期
 * @return 1=注册成功; 0=注册失败(参数为空 / 同一设备重复注册 / 超出 SAFE_DAEMON_PROCESS_MAX)
 * @note 各模块在自己的任务 init 里调用, 因此是跨任务并发调用; 读-改-写在临界区内完成,
 *       失败只计统计(见 safe_daemon_process_get_reject_count()), 不会破坏已发布的表项。
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
 * @brief 获取所有已注册设备是否全部在线
 * @return 1=全部在线; 0=存在掉线设备, 或注册表为空(未注册任何设备不能当作"全部在线")
 */
uint8_t safe_daemon_process_all_online(void);

/**
 * @brief 所有已注册设备是否都至少完成过一次检测(启动静默期是否已过去)
 * @return 1=已就绪; 0=注册表为空, 或仍有设备在静默期内/刚注册尚未被检测
 * @note 上电后 `online` 一律先置 1 以避免误报, 因此 is_online() 在静默期内恒为"在线"。
 *       需要区分"真在线"与"还没测过"时用本接口, 例如上层做掉线锁存前先等 is_ready()。
 */
uint8_t safe_daemon_process_is_ready(void);

/**
 * @brief 查询某设备当前是否在线
 * @param device 设备对象指针
 * @return 1=在线; 0=离线 或 该设备未注册
 * @note 未注册返回 0(不可用), 避免漏注册被误认为在线
 */
uint8_t safe_daemon_process_is_online(const void *device);

/**
 * @brief 取单个设备的状态快照(在线/是否已评估/连续离线时长/累计掉线次数)
 * @param device 设备对象指针
 * @param status 输出; 未注册或入参为空时全部字段置 0
 * @return 1=设备已注册并成功填充; 0=未注册(调用方可直接忽略返回值, 此时 status 视为"不可用")
 * @note 提供给上层做去抖与锁存: 例如 offline_ms > 300 才认为是真掉线, 避免单次窗口抖动误触发。
 *       读取过程无临界区: 各字段独立读取, 允许与守护任务并发; 若需要严格一致的多字段快照,
 *       应连续读取两次并比较 offline_count 是否变化。
 */
uint8_t safe_daemon_process_get_device_status(const void *device, safe_daemon_process_device_status_t *status);

/**
 * @brief 获取注册表指针, 便于观察或自行遍历(只读: 状态与统计由守护任务维护)
 * @param count 输出当前已注册设备数, 可为空
 */
const safe_daemon_process_table_t *safe_daemon_process_get_table(uint8_t *count);

/**
 * @brief 获取累计注册失败次数(参数为空 / 同一设备重复注册 / 超出 SAFE_DAEMON_PROCESS_MAX)
 * @return 累计失败次数, 0 表示全部注册成功
 * @note 注册失败时对应的设备名存在 safe_daemon_process_last_reject_name(Ozone 直接观察),
 *       用于定位"某个电机没被监控"这类静默漏配
 */
uint32_t safe_daemon_process_get_reject_count(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief 把设备成员函数 void T::alive_check() 适配成 safe_daemon_process_check_client_t
 * @note  模板参数为设备指针类型(TPtr = T *), 而非 T 或 T&:
 *        decltype(设备对象表达式) 得到的是引用类型, 直接用会形成"指向引用的指针"
 */
template <typename TPtr>
inline void safe_daemon_process_check_adapter(void *device)
{
    static_cast<TPtr>(device)->alive_check();
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
 * @note 模板参数同上, 为设备指针类型(TPtr = T *)
 */
template <typename TPtr>
inline uint8_t safe_daemon_process_status_adapter(void *device)
{
    typedef decltype(static_cast<TPtr>(nullptr)->get_status()) status_enum_t;
    return (static_cast<TPtr>(device)->get_status() == safe_daemon_process_online_value<status_enum_t>::value) ? 1 : 0;
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
                                 &safe_daemon_process_check_adapter<decltype(&(obj))>,        \
                                 &safe_daemon_process_status_adapter<decltype(&(obj))>,       \
                                 (period_ms))

/**
 * @brief 由调用点显式给出"在线"判据的版本, 用于不便往本头补特化的设备(IMU / PC 视觉等)
 * @note 模板参数 OnlineValue 是设备"在线"时的状态枚举值, 在宏里就地传入, 因此本头不需要 include 该驱动头,
 *       避免把一个纯逻辑模块和被监控对象(可能带 HAL/USB 依赖)绑死
 * @note 模板参数 TPtr 为设备指针类型, 用 auto 承接以兼容 volatile 对象(如 volatile hi12_gyro hi12_data)
 */
template <typename TPtr, auto OnlineValue>
inline uint8_t safe_daemon_process_status_as_adapter(void *device)
{
    typedef decltype(static_cast<TPtr>(nullptr)->get_status()) status_enum_t;
    return (static_cast<status_enum_t>(static_cast<TPtr>(device)->get_status()) ==
            static_cast<status_enum_t>(OnlineValue))
               ? 1
               : 0;
}

/**
 * @brief 注册待监控设备, 在线判据由调用点给出(需具备 void alive_check() 与 get_status())
 * @param name 设备名
 * @param obj 设备对象(直接写对象表达式, 可以是 volatile 全局对象, 如 hi12_data)
 * @param online_value 设备在线时 get_status() 的取值
 * @param period_ms 检测周期(ms)
 * @note 例: SAFE_DAEMON_PROCESS_REGISTER_AS("hi12_imu", hi12_data,
 *                                          HI12_Connection_Status_Enable, SAFE_DAEMON_PROCESS_PERIOD_SENSOR_MS);
 * @note 中间的 const volatile 转换是为了兼容 volatile 对象:
 *       volatile T* 不能直接转 void*, 因此先补成 const volatile void* 再 const_cast 掉
 */
#define SAFE_DAEMON_PROCESS_REGISTER_AS(name, obj, online_value, period_ms)                     \
    safe_daemon_process_register(                                                              \
        (name), const_cast<void *>(static_cast<const volatile void *>(&(obj))),                 \
        &safe_daemon_process_check_adapter<decltype(&(obj))>,                                  \
        &safe_daemon_process_status_as_adapter<decltype(&(obj)), (online_value)>, (period_ms))
#endif
