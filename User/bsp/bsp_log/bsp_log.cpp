#include "bsp_log.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>
#include "math.h"
// 初始化RTT
void bsp_log_init(void)
{
    SEGGER_RTT_Init();
}

//RTT日志打印函数,支持格式化字符串输出
int print_log(const char * sFormat, ...)
{	
	va_list ParamList;
	va_start(ParamList, sFormat);
	int n = SEGGER_RTT_vprintf(BUFFER_INDEX, sFormat, &ParamList);
	va_end(ParamList);
	return n;
}
//浮点数转换为字符串函数
void float_to_strnum(char *str, float float_va, int num)
{
	int flag = float_va < 0;								//判断转换的浮点数的正负号
	float abs_va = fabsf(float_va);							//获取浮点数的绝对值
	int head = (int)abs_va;									//获取整数部分
	float fractional = abs_va - head;						//获取小数部分

    int point = (int)(fractional * powf(10, num) + 0.5f); 	// 四舍五入			

	if (point >= (int)powf(10, num))						//处理小数部分进位
	{
		point = 0;
		head += 1;
	}

	if (flag)
	{
		sprintf(str, "-%d.%0*d", head, num, point);
	}
	else
	{
		sprintf(str, "%d.%0*d", head, num, point);
	}
}
