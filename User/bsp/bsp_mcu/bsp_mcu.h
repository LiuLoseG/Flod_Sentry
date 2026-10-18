#pragma once

// 让头文件同时兼容 C 和 C++

// 函数声明使用 C 链接方式；在 C 编译器下这段会被忽略，避免语法错误。
// 这样同一个头文件既能被 C 文件包含，也能被 C++ 文件包含。
#ifdef __cplusplus
extern "C" {
#endif

void bsp_mcu_init(void);

#ifdef __cplusplus
}
#endif
