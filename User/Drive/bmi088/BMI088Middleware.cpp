#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088传感器GPIO初始化函数
************************************************************************
**/
void BMI088_GPIO_init(void)
{

}

/**
 * @brief 初始化 BMI088 使用的 SPI 外设, 并尝试加速通信
 */
void BMI088_com_init(void)
{
    // CFG1(含波特率分频)在 SPE=1 时写保护, 必须先在关掉外设的情况下重新初始化
    __HAL_SPI_DISABLE(&BMI088_USING_SPI_UNIT);

    BMI088_USING_SPI_UNIT.Init.BaudRatePrescaler = BMI088_SPI_PRESCALER;
    if (HAL_SPI_Init(&BMI088_USING_SPI_UNIT) != HAL_OK)
    {
        // 加速失败就退回 CubeMX 默认分频, 绝不让通信初始化因此整体失败
        BMI088_USING_SPI_UNIT.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        (void)HAL_SPI_Init(&BMI088_USING_SPI_UNIT);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - 要延迟的毫秒数
* @retval:     	void
* @details:    	延迟指定毫秒数的函数，基于微秒延迟实现
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    // BMI088_init() 里有两段 80ms 的软复位等待. IMU 任务优先级最高, 若用 SysTick 忙等,
    // 初始化重试期间会把 CAN 收发/云台/安全任务整体饿死(单次重试可达数百毫秒).
    // 调度器已启动时必须让出 CPU.
    if (osKernelGetState() == osKernelRunning)
    {
        osDelay((uint32_t)ms);
        return;
    }

    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - 要延迟的微秒数
* @retval:     	void
* @details:    	微秒级延迟函数，使用SysTick定时器实现
************************************************************************
**/
void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    // 计算延迟所需的滴答数
    ticks = (uint32_t)us * (SystemCoreClock / 1000000U);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - 要发送的数据
* @retval:     	uint8_t - 接收到的数据
* @details:    	通过BMI088使用的SPI总线进行单字节的读写操作
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
/**
************************************************************************
* @brief:      	BMI088_transmit_receive(uint8_t *tx, uint8_t *rx, uint16_t len)
* @param:       tx - 发送缓冲区
*               rx - 接收缓冲区
*               len - 传输字节数
* @retval:     	uint8_t - 0 成功, 1 失败
* @details:    	SPI 全双工突发收发, 使用阻塞模式(非 DMA), 片选由调用者控制
************************************************************************
**/
uint8_t BMI088_transmit_receive(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((tx == 0) || (rx == 0) || (len == 0))
    {
        return 1;
    }

    if (HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx, rx, len, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

