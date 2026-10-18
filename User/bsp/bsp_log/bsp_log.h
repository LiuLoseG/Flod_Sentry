#ifndef __BSP_LOG_H
#define __BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>

#define BUFFER_INDEX 0 // RTT的0号缓冲区索引

void bsp_log_init(void);

/** 
* @brief 日志打印宏定义
* @param type: 日志类型(如：INFO(信息), ERROR(错误), WARNING(警告), DEBUG(调试))
* @param color: 日志颜色(如：RTT_CTRL_TEXT_RED(红色), RTT_CTRL_TEXT_GREEN(绿色), RTT_CTRL_TEXT_YELLOW(黄色), RTT_CTRL_TEXT_BLUE(蓝色))
* @param format: 格式化字符串
* @param ...: 可变参数
* @note %s(color，文本颜色)%s(type，日志类型)" format "\r\n%s(RTT_CTRL_RESET，恢复终端默认颜色)
*       ##__VA_ARGS__(允许可变参数为空时自动去除前面的逗号，避免语法错误)
**/

#define LOG_PROTO(type, color, format, ...)                       \
        SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s" format "\r\n%s", \
                          color,                                  \
                          type,                                   \
                          ##__VA_ARGS__,                          \
                          RTT_CTRL_RESET)
                                    
/*-------------------------------------------------------日志输出调用接口------------------------------------------------------------------------------------*/
//清除终端历史显示          
#define LOG_CLEAR()               SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)                    

//无颜色日志输出
#define LOG_Blank(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/** 
 * @brief 有颜色日志输出
 * @attention 注意：以下接口不支持浮点数输出，如需浮点数输出请使用float_to_str2()函数转换为字符串后再输出
 * @note
 */
// information level
#define LOG_INFO(format, ...)       LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
// warning level
#define LOG_WARNING(format, ...)    LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
// error level
#define LOG_ERROR(format, ...)      LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)

/** 
 * @brief segger RTT日志打印函数,支持格式化字符串输出,参考：printf函数
 * @param sFormat: 格式化字符串
 * @param ...: 可变参数列表
 * @param 返回值n: 成功返回实际打印字符数,失败返回负值
 * @note 支持浮点数输出
 * 
**/

int print_log(const char * sFormat, ...);

/** 
 * 
 * @brief 浮点数转换为字符串函数
 * @param str: 转换后的字符串
 * @param float_va: 需要转换的浮点数
 * @return 无
 * @note 浮点数需要转换为字符串后才能通过RTT打印
**/
void float_to_strnum(char *str, float float_va, int num);

#endif
