#include "basic_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// rpm换算到rad/s
const float BASIC_MATH_RPM_TO_RADPS = 2.0f * PI / 60.0f;
// deg换算到rad
const float BASIC_MATH_DEG_TO_RAD = PI / 180.0f;
// 摄氏度换算到开氏度
const float BASIC_MATH_CELSIUS_TO_KELVIN = 273.15f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param Value 布尔值地址
 */

void basic_math_bool_reverse(bool *value)
{
    if (*value)
    {
        *value = false;
    }
    else
    {
        *value = true;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param address 地址
 */
void basic_math_endian_swap_16(uint16_t *address)
{
    uint8_t tmp[2];
    memcpy(tmp, address, 2);
    /* 通过 memcpy 避免 C++ 严格别名违规 (pointer-casting then dereference) */
    uint16_t result = ((uint16_t)tmp[0] << 8) | (uint16_t)tmp[1];
    memcpy(address, &result, 2);
}

/**
 * @brief 16位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t basic_math_endian_swap_16(void *source, void *destination)
{
    uint8_t *tmp_address_8;
    uint16_t tmp_value_16;
    tmp_address_8 = (uint8_t *) source;
    tmp_value_16 = tmp_address_8[0] << 8 | tmp_address_8[1];

    if (destination != nullptr)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) source;
        tmp_destination = (uint8_t *) destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return (tmp_value_16);
}

/**
 * @brief 32位大小端转换
 *
 * @param address 地址
 */
void basic_math_endian_swap_32(uint32_t *address)
{
    uint8_t *tmp_address_8;
    uint32_t *tmp_address_32;
    tmp_address_8 = (uint8_t *) address;
    tmp_address_32 = (uint32_t *) address;
    *tmp_address_32 = tmp_address_8[0] << 24 | tmp_address_8[1] << 16 | tmp_address_8[2] << 8 | tmp_address_8[3];
}

/**
 * @brief 32位大小端转换
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t basic_math_endian_swap_32(void *source, void *destination)
{
    uint8_t *tmp_address_8;
    uint32_t tmp_value_32;
    tmp_address_8 = (uint8_t *) source;
    tmp_value_32 = tmp_address_8[0] << 24 | tmp_address_8[1] << 16 | tmp_address_8[2] << 8 | tmp_address_8[3];

    if (destination != nullptr)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) source;
        tmp_destination = (uint8_t *) destination;
        tmp_destination[0] = tmp_source[3];
        tmp_destination[1] = tmp_source[2];
        tmp_destination[2] = tmp_source[1];
        tmp_destination[3] = tmp_source[0];
    }

    return (tmp_value_32);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t basic_math_sum_8(const uint8_t *address, uint32_t length)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t basic_math_sum_16(const uint16_t *address, uint32_t length)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t basic_math_sum_32(const uint32_t *address, uint32_t length)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float basic_math_sinc(float x)
{
    // 分母为0则按极限求法
    if (basic_math_abs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (arm_sin_f32(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param float_1 浮点数1
 * @param float_2 浮点数2
 * @param int_1 整型1
 * @param int_2 整型2
 * @return int32_t 整型
 */
int32_t basic_math_float_to_int(float x, float float_1, float float_2, int32_t int_1, int32_t int_2)
{
    float tmp = (x - float_1) / (float_2 - float_1);
    auto out = (int32_t)(tmp * (float) (int_2 - int_1) + (float) (int_1));
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param int_1 整型1
 * @param int_2 整型2
 * @param float_1 浮点数1
 * @param float_2 浮点数2
 * @return float 浮点数
 */
float basic_math_int_to_float(int32_t x, int32_t int_1, int32_t int_2, float float_1, float float_2)
{
    float tmp = (float) (x - int_1) / (float) (int_2 - int_1);
    float out = tmp * (float_2 - float_1) + float_1;
    return (out);
}

/**
 * @brief 判断浮点数是否为无效浮点数
 *
 * @param x 浮点数
 * @return 是否为NaN
 */
bool basic_math_is_invalid_float(float x)
{
    uint32_t exp = (*(uint32_t *) (&x) >> 23) & 0xff;
    uint32_t frac = (*(uint32_t *) (&x) & 0x7fffff);
    if (exp == 0x00)
    {
        if (frac == 0x00)
        {
            // 正负零
            return (false);
        }
        // 次正规数和非正规数
        return (true);
    }
    if (exp == 0xFF)
    {
        // 无穷大和NaN
        return (true);
    }
    // 正规数
    return (false);
}

/**
 * @brief 求取模归化
 *
 * @param x 传入数据
 * @param modulus 模数
 * @return 返回的归化数, 介于 ±modulus / 2 之间
 */
float basic_math_modulus_normalization(float x, float modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}
