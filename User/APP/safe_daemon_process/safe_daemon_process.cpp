/**
 * @file safe_daemon_process.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 安全守护进程: 统一调用各电机 alive_check() 做离线检测
 * @note  实现要点:
 *        1. 全工程唯一的 alive_check() 调用点在本文件 safe_daemon_task() 内;
 *        2. 检测语义: 设备自己的 alive_check() 做"滑动窗口 flag 比较"并更新其内部状态,
 *           本层只负责"到期调用 + 读取状态 + 统计";
 *        3. 只检测不动作: 不写任何控制量, 不做安全动作; 上层用 is_online()/all_online() 自行判断;
 *        4. 未注册设备视为"不可用"返回 0, 防止漏注册被误判为在线;
 *        5. 注册由各模块在自己的任务 init 里调用(跨任务并发), 读-改-写在临界区内完成;
 *           注册失败不破坏已发布的表项, 只累加 reject 统计, 避免"漏监控却无人发现";
 *        6. 状态细分: 除 online 外还维护 offline_since_tick, 由 get_device_status() 折算成
 *           连续离线时长, 上层做去抖/锁存时不必自己计时。
 * @date 2026-09-18
 * @version 0.4
 * @copyright HNAU-Robot (c) 2026
 */

#include "safe_daemon_process.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

// 注册表(静态数组, 无动态内存; Ozone 可直接观察该表排查掉线)
static safe_daemon_process_table_t safe_daemon_process_table[SAFE_DAEMON_PROCESS_MAX] = {};

// 已注册设备数(注册时最后发布, 保证轮询侧读到的表项一定是完整的; 跨任务读写, 用 volatile)
static volatile uint8_t safe_daemon_process_count = 0;

// 注册失败统计: 失败只记录不覆盖, 由 get_reject_count() 与 Ozone 观察
static uint32_t safe_daemon_process_reject_count = 0;
static const char *safe_daemon_process_last_reject_name = nullptr;

/**
 * @brief 设备注册函数
 * @note 各模块在自己的任务里 init, 注册是跨任务并发调用: 查重、占位、发布 count 必须在同一临界区内完成,
 *       否则两个任务可能读到同一个 idx 并互相覆盖表项
 * @return 1=注册成功; 0=注册失败(参数为空 / 同一设备重复注册 / 超出 SAFE_DAEMON_PROCESS_MAX)
 */
uint8_t safe_daemon_process_register(const char *name, void *device,
                                     safe_daemon_process_check_client_t check_client,
                                     safe_daemon_process_status_client_t status_client,
                                     uint32_t period_ms)
{
    // 相位对齐到"注册时刻", 给新注册设备一个完整周期的缓冲:
    // 若置 0, 则注册晚于静默期时会立即检测, 可能把尚未建立通信的设备误判为离线
    uint32_t register_tick = osKernelGetTickCount();

    taskENTER_CRITICAL();

    // 先查重再判断容量: 重复注册与容量溢出都不允许写坏任何已发布表项
    uint8_t registered = safe_daemon_process_count;
    uint8_t duplicated = 0;

    if (device != nullptr)
    {
        for (uint8_t i = 0; i < registered; i++)
        {
            if (safe_daemon_process_table[i].device == device)
            {
                duplicated = 1;
                break;
            }
        }
    }

    if (device == nullptr || check_client == nullptr || status_client == nullptr ||
        duplicated != 0 || registered >= SAFE_DAEMON_PROCESS_MAX)
    {
        safe_daemon_process_reject_count++;
        safe_daemon_process_last_reject_name = name;
        taskEXIT_CRITICAL();
        return 0; // 无效注册
    }

    safe_daemon_process_table_t *entry = &safe_daemon_process_table[registered];

    entry->name = name;
    entry->device = device;
    entry->check_client = check_client;
    entry->status_client = status_client;
    entry->period_ms = (period_ms == 0) ? SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS : period_ms;
    entry->last_check_tick = register_tick;
    entry->check_count = 0;
    entry->offline_count = 0;
    entry->offline_since_tick = 0;
    entry->online = 1;                          // 初始视为在线, 由首次检测结果刷新
    safe_daemon_process_count = registered + 1; // 最后发布表项

    taskEXIT_CRITICAL();

    return 1;
}

/**
 * @brief 安全守护任务
 * @note 全工程唯一的 alive_check() 调用点
 */
extern "C" void safe_daemon_task(void *argument)
{
    (void)argument;

    // 上电静默期: 等待各业务模块 init 完成并建立电机通信, 避免首轮把尚未反馈的电机误判为掉线
    osDelay(SAFE_DAEMON_PROCESS_STARTUP_DELAY_MS);

    uint32_t now_tick = osKernelGetTickCount();
    uint8_t registered = safe_daemon_process_count; // 快照: 只对齐静默期内已注册的设备

    for (uint8_t i = 0; i < registered; i++)
    {
        safe_daemon_process_table[i].last_check_tick = now_tick; // 以唤醒时刻为相位基准
        safe_daemon_process_table[i].online = 1;
    }
    // 静默期之后才注册的设备保留注册时刻的相位(见 safe_daemon_process_register), 无需在此对齐

    for (;;)
    {
        now_tick = osKernelGetTickCount();
        uint32_t sleep_ms = 0xFFFFFFFFu; // 本拍之后的最小剩余时间

        // 唯一的存活检测调用点: 到期设备依次检测, 各设备周期互不干扰
        registered = safe_daemon_process_count; // 快照: 避免遍历中读到新增表项
        for (uint8_t i = 0; i < registered; i++)
        {
            safe_daemon_process_table_t *entry = &safe_daemon_process_table[i];
            uint32_t elapsed = now_tick - entry->last_check_tick; // 无符号差值, 天然抗回绕

            if (elapsed >= entry->period_ms)
            {
                entry->last_check_tick = now_tick;

                // 调用设备自身的存活检测(内部只做 flag 比较与状态更新, 无阻塞)
                entry->check_client(entry->device);
                entry->check_count++;

                // 刷新在线状态(判据来自驱动 get_status())
                uint8_t online = (entry->status_client(entry->device) != 0) ? 1 : 0;

                if (online != entry->online)
                {
                    entry->online = online;

                    if (online == 0)
                    {
                        entry->offline_count++;              // 累计掉线次数, 供上层与 Ozone 诊断
                        entry->offline_since_tick = now_tick; // 记录起始时刻, 供 offline_ms 计算
                    }
                }

                elapsed = 0; // 刚检测过, 下一周期从此刻起算
            }

            // 睡到最近一台设备到期, 而不是固定 1ms 空转: 相同相位的设备会在同一拍被唤醒
            if ((entry->period_ms - elapsed) < sleep_ms)
            {
                sleep_ms = entry->period_ms - elapsed;
            }
        }

        // 空表时没有参考周期, 用最小粒度兜底(保证新注册设备能被及时发现)
        if (sleep_ms == 0xFFFFFFFFu || sleep_ms == 0)
        {
            sleep_ms = SAFE_DAEMON_PROCESS_POLL_PERIOD_MS;
        }

        osDelay(sleep_ms);
    }
}

/**
 * @brief 所有已注册设备是否都至少完成过一次检测(启动静默期是否已过去)
 */
uint8_t safe_daemon_process_is_ready(void)
{
    uint8_t registered = safe_daemon_process_count;

    // 空表没有"就绪"可言, 与 all_online() 对空表的判据保持一致
    if (registered == 0)
    {
        return 0;
    }

    for (uint8_t i = 0; i < registered; i++)
    {
        if (safe_daemon_process_table[i].check_count == 0)
        {
            return 0; // 仍有设备未被检测过(静默期内, 或刚注册未到期)
        }
    }

    return 1;
}

/**
 * @brief 查询某设备当前是否在线
 */
uint8_t safe_daemon_process_is_online(const void *device)
{
    uint8_t registered = safe_daemon_process_count; // 快照: 与 all_online() 的遍历范围保持一致

    for (uint8_t i = 0; i < registered; i++)
    {
        if (safe_daemon_process_table[i].device == device)
        {
            return safe_daemon_process_table[i].online;
        }
    }

    // 未注册: 返回不可用, 避免漏注册被上层误认为在线
    return 0;
}

/**
 * @brief 获取所有已注册设备是否全部在线
 */
uint8_t safe_daemon_process_all_online(void)
{
    uint8_t registered = safe_daemon_process_count;

    // 空表不能当作"全部在线": 与 is_online() 对未注册设备返回 0 的判据保持一致
    if (registered == 0)
    {
        return 0;
    }

    for (uint8_t i = 0; i < registered; i++)
    {
        if (safe_daemon_process_table[i].online == 0)
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief 取单个设备的状态快照(在线/是否已评估/连续离线时长/累计掉线次数)
 */
uint8_t safe_daemon_process_get_device_status(const void *device,
                                              safe_daemon_process_device_status_t *status)
{
    if (status == nullptr)
    {
        return 0;
    }

    // 未注册或参数无效时统一给"不可用"的全零快照, 调用方可忽略返回值直接使用
    status->online = 0;
    status->evaluated = 0;
    status->offline_count = 0;
    status->offline_ms = 0;

    uint8_t registered = safe_daemon_process_count;

    for (uint8_t i = 0; i < registered; i++)
    {
        const safe_daemon_process_table_t *entry = &safe_daemon_process_table[i];

        if (entry->device != device)
        {
            continue;
        }

        status->online = entry->online;
        status->evaluated = (entry->check_count != 0) ? 1 : 0;
        status->offline_count = entry->offline_count;
        // 无符号差值, 天然抗 tick 回绕
        status->offline_ms = (entry->online == 0) ? (osKernelGetTickCount() - entry->offline_since_tick) : 0;

        return 1;
    }

    return 0;
}

/**
 * @brief 获取注册表指针(只读)
 */
const safe_daemon_process_table_t *safe_daemon_process_get_table(uint8_t *count)
{
    if (count != nullptr)
    {
        *count = safe_daemon_process_count;
    }

    return safe_daemon_process_table;
}

/**
 * @brief 获取累计注册失败次数
 */
uint32_t safe_daemon_process_get_reject_count(void)
{
    return safe_daemon_process_reject_count;
}

