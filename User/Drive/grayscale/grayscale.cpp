#include "grayscale.h"


// 按照灰度传感器的灯管数量定义位置偏差属性 N=3 → {-1,  0, +1}、N=5 → {-2, -1, 0, +1, +2}、N=8 → {-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5}
static float gray_pos_table[GRAY_SENSOR_NUM];
static uint8_t gray_pos_table_inited = 0;       // 位置表初始化标志,只进行一次初始化

 /**
  * @brief 初始化灰度传感器对象
  * @param s 传感器对象指针
  * @param pin_map 引脚映射表指针
  * @param on_lost 脱线回调函数
  * @param on_lost_user 回调函数用户数据
  * @param on_stop_line 停止线回调函数
  * @param on_stop_line_user 回调函数用户数据
  * @param on_recover 恢复回调函数
  * @param on_recover_user 回调函数用户数据
  * @note pos_table[i] = i - (N-1)/2, 从左到右 i=0~N-1，
          最左 i=0 位置 = -(N-1)/2，
  *        最右 i=N-1 位置 = +(N-1)/2，
  *        等间距分布，0 在正中间（N 为偶数时 0 在中间两探头之间）
  */
void grayscale_init(gray_sensor_t *s, const gray_pin *pin_map,
                    gray_on_lost_t on_lost, void *on_lost_user,
                    gray_on_stop_line_t on_stop_line, void *on_stop_line_user,
                    gray_on_recover_t on_recover, void *on_recover_user)
{
    // 填充全局位置表
    if (!gray_pos_table_inited)
    {
        for (uint8_t i = 0; i < GRAY_SENSOR_NUM; i++)
        {
            gray_pos_table[i] = (float)i - (GRAY_SENSOR_NUM - 1) / 2.0f;
        }

        // 标记位置表已初始化，避免重复计算
        gray_pos_table_inited = 1;
    }

    // gpio引脚映射
    s->pins = pin_map;

    // 清零所有状态
    s->state          = 0;
    s->position       = 0.0f;
    s->backup         = 0.0f;
    s->stop_line_cnt  = 0;
    s->lost_cnt       = 0;
    s->lost_cnt_prev  = 0;

    // 注册自定义回调函数
    s->on_lost         = on_lost;
    s->on_lost_user    = on_lost_user;
    s->on_stop_line    = on_stop_line;
    s->on_stop_line_user = on_stop_line_user;
    s->on_recover      = on_recover;
    s->on_recover_user = on_recover_user;
}

/**
  * @brief 读取灰度传感器 GPIO 状态, 读取 n 路 GPIO 并组装 n bit 状态掩码
  * @param s 传感器对象指针
  * @note 读取 GPIO_Pin引脚状态，组装成 n bit 状态掩码
  *       左→右 (i=0=最左, i=n-1=最右)
  *       state bit0 = 最左传感器，bit(n-1) = 最右传感器
  *       sensor_val[i] = 1 表示黑线(高电平)，0 表示白底(低电平)
  */
void grayscale_read_gpio(gray_sensor_t *s)
{
    for (int8_t i = GRAY_SENSOR_NUM - 1; i >= 0; i--)
    {
        s->sensor_val[i] = HAL_GPIO_ReadPin(s->pins[i].port, s->pins[i].pin);
        // 左移后合并：先读到的（最右）在高位，后读到的（最左）在低位
        s->state = (s->state << 1) | s->sensor_val[i];
    }
}

/**
 * @brief 使用加权质心法, 计算黑线的连续位置
 * @param s 传感器对象指针
 * @note 计算公式：
 *               Σ ( 第 i 个探头值 × 第 i 个探头物理位置 )
 *   position = ──────────────────────────────────────────
 *                      Σ 第 i 个探头值
 *   将每个"看到黑线"的探头按其物理位置加权平均，获得连续值
    *   age:（n=5，探头 #2、#3 看到黑线）：
    *   sensor_val = {0, 0, 1, 1, 0}    ← 1=黑线，0=白底
    *   pos_table  = {-2, -1, 0, +1, +2}
    *   position = (1×0 + 1×+1) / 2 = +0.5   → 偏右 0.5
 */
void grayscale_centroid(gray_sensor_t *s)
{
    float sum = 0.0f;
    uint8_t count = 0;  // 避免除以零

    for (uint8_t i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        if (s->sensor_val[i])   // 统计看到黑线的探头数量
        {
            count++;
        }

        sum += s->sensor_val[i] * gray_pos_table[i];    // 灰度数据加权求和
    }

    if (count == 0)         // 检测到灰度传感器反馈的位置，全在白状态，循迹脱线
    {
        s->position = 0.0f;  
    }
    
    else
    {
        s->position = sum / count;
    }
}

/**
 * @brief 灰度传感器更新函数
 * @param s 传感器对象指针
 * @note 1. 读取 GPIO 并组装状态掩码
 *       2. 根据状态掩码决策：全白→脱线、全黑→停止线、其他→正常循迹
 *       3. 脱线→恢复的软切入处理
 */
void grayscale_update(gray_sensor_t *s)
{
    // 读取 GPIO 状态并组装状态掩码
    grayscale_read_gpio(s);

    // 保存上一周期 lost_cnt，检测脱线→恢复的上升沿
    s->lost_cnt_prev = s->lost_cnt;

    // 记录脱线前的 backup，用于软切入恢复
    float prev_backup = s->backup;

    // GPIO读取的状态掩码决策，进行脱线/停止线/正常循迹处理
    if (s->state == GRAY_ALL_WHITE) // 全白：所有探头都检测到白底→脱线(state == 0x00)
    {
        s->lost_cnt++;              // 连续脱线周期计数
        s->position = s->backup;    // 沿用最近一次有效位置，避免位置突变导致失控

        if (s->on_lost != NULL)     // 触发脱线回调函数(已注册)
        {
            s->on_lost(s, s->on_lost_user, s->lost_cnt);
        }
    }

    else if (s->state == GRAY_ALL_BLACK)// 全黑：所有探头都检测到黑线→停止线、十字路口
    {
        s->stop_line_cnt++;         // 连续全黑周期计数
        s->position      = 0.0f;    // 位置归零
        s->backup        = 0.0f;    // 清零 backup，避免脱线时沿用已失效的历史值       
        s->lost_cnt      = 0;       // 清零 lost_cnt（全黑不算脱线）

        if (s->on_stop_line != NULL)    // 触发全黑回调函数(已注册)
        {
            s->on_stop_line(s, s->on_stop_line_user, s->stop_line_cnt);
        }
    }

    else    // 正常循迹：部分探头检测到黑线
    {

        grayscale_centroid(s);              // 加权质心法计算连续位置

        s->backup         = s->position;    // 更新 backup 为当前有效位置
        s->lost_cnt       = 0;              // 清零 lost_cnt
        s->stop_line_cnt  = 0;              // 清零 stop_line_cnt

        // 脱线-恢复的软切入，lost_cnt_prev > 0, (上一周期处于脱线状态)说明刚脱离全白状态、恢复正常循迹
        // position = 0.5 × 脱线前backup + 0.5 × 当前质心值相当于一阶低通滤波，避免backup值和质心值差异太大导致 position 跳变，使车体抖动
        if (s->lost_cnt_prev > 0)
        {
            s->position = 0.5f * prev_backup + 0.5f * s->position;

            if (s->on_recover != NULL)
            {
                s->on_recover(s, s->on_recover_user, s->position);
            }
        }
    }
}