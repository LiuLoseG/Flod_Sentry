/**
 * @file safe_daemon_process.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 安全守护进程: 统一调用各电机 alive_check() 做离线检测
 * @note  实现要点:
 *        1. 全工程唯一的 alive_check() 调用点在本文件 safe_daemon_task() 内;
 *        2. 检测语义: 设备自己的 alive_check() 做"滑动窗口 flag 比较"并更新其内部状态,
 *           本层只负责"到期调用 + 读取状态 + 统计";
 *        3. 只检测不动作: 不写任何控制量, 不做安全动作; 上层用 is_online()/all_online() 自行判断;
 *        4. 未注册设备视为"不可用"返回 0, 防止漏注册被误判为在线。
 * @date 2026-09-18
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "safe_daemon_process.h"

#include "freertos.h"
#include "cmsis_os2.h"

/*==================================== 静态变量 ====================================*/

// 注册表(静态数组, 无动态内存; Ozone 可直接观察该表排查掉线)
static safe_daemon_process_table_t safe_daemon_process_table[SAFE_DAEMON_PROCESS_MAX] = {0};

// 已注册设备数(注册时最后发布, 保证轮询侧读到的表项一定是完整的)
static uint8_t safe_daemon_process_count = 0;

/*==================================== 接口实现 ====================================*/

/**
 * @brief 设备注册函数
 * @note 单写者原则: 仅在 init 阶段(任务上下文)调用; 先完整填写表项, 最后才发布 count
 * @return 1=注册成功; 0=注册失败(参数为空 / 同一设备重复注册 / 超出 SAFE_DAEMON_PROCESS_MAX)
 */
uint8_t safe_daemon_process_register(const char *name, void *device,
                                     safe_daemon_process_check_client_t check_client,
                                     safe_daemon_process_status_client_t status_client,
                                     uint32_t period_ms)
{
    if (device == nullptr || check_client == nullptr || status_client == nullptr)
    {
        return 0; // 无效注册
    }

    // 重复注册检测, 防止同一设备被注册多次
    for (uint8_t i = 0; i < safe_daemon_process_count; i++)
    {
        if (safe_daemon_process_table[i].device == device)
        {
            return 0;
        }
    }

    if (safe_daemon_process_count >= SAFE_DAEMON_PROCESS_MAX)
    {
        return 0; // 超过最大注册数量
    }

    uint8_t idx = safe_daemon_process_count;
    safe_daemon_process_table_t *entry = &safe_daemon_process_table[idx];

    entry->name = name;
    entry->device = device;
    entry->check_client = check_client;
    entry->status_client = status_client;
    entry->period_ms = (period_ms == 0) ? SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS : period_ms;
    // 相位对齐到"注册时刻", 给新注册设备一个完整周期的缓冲:
    // 若置 0, 则注册晚于静默期时(如将来把注册放到任务 delay 之后)会立即检测, 可能把尚未建立通信的设备误判为离线
    entry->last_check_tick = osKernelGetTickCount();
    entry->check_count = 0;
    entry->offline_count = 0;
    entry->online = 1;          // 初始视为在线, 由首次检测结果刷新
    safe_daemon_process_count = idx + 1; // 发布表项

    return 1;
}

/**
 * @brief 安全守护任务
 * @note 全工程唯一的 alive_check() 调用点
 */
extern "C" void safe_daemon_task(void *argument)
{
    // 上电静默期: 等待各业务模块 init 完成并建立电机通信, 避免首轮把尚未反馈的电机误判为掉线
    osDelay(SAFE_DAEMON_PROCESS_STARTUP_DELAY_MS);

    uint32_t now_tick = osKernelGetTickCount();

    for (uint8_t i = 0; i < safe_daemon_process_count; i++)
    {
        safe_daemon_process_table[i].last_check_tick = now_tick; // 以唤醒时刻为相位基准
        safe_daemon_process_table[i].online = 1;
    }

    for (;;)
    {
        now_tick = osKernelGetTickCount();

        // ★ 唯一的存活检测调用点: 到期设备依次检测, 各设备周期互不干扰
        uint8_t registered = safe_daemon_process_count; // 快照: 避免遍历中读到新增表项
        for (uint8_t i = 0; i < registered; i++)
        {
            safe_daemon_process_table_t *entry = &safe_daemon_process_table[i];

            if (entry->check_client == nullptr || entry->status_client == nullptr)
            {
                continue;
            }

            if ((now_tick - entry->last_check_tick) < entry->period_ms)
            {
                continue; // 未到该设备的检测时刻
            }

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
                    entry->offline_count++; // 累计掉线次数, 供上层与 Ozone 诊断
                }
            }
        }

        osDelay(SAFE_DAEMON_PROCESS_POLL_PERIOD_MS);
    }
}

/**
 * @brief 查询某设备当前是否在线
 */
uint8_t safe_daemon_process_is_online(const void *device)
{
    for (uint8_t i = 0; i < safe_daemon_process_count; i++)
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
    for (uint8_t i = 0; i < safe_daemon_process_count; i++)
    {
        if (safe_daemon_process_table[i].online == 0)
        {
            return 0;
        }
    }

    return 1;
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

