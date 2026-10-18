#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_last.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"
#include "MahonyAHRS.h"
#include "pid_fuse.h"

#include <math.h>

#define CHEAT TRUE  					//作弊模式 去掉较小的gyro值
#define CORRECT_TIM_DEFINE 1000    		//上电去0飘 1000次取平均
#define TEMP_STABLE_TIMES 300       	//探测温度阈值
#define GYRO_ZERO_THRESHOLD 0.003f      // gyro 小值截止

volatile imu_data_resolve_t imu_frame = {0};

float gyro_correct[3]={0.001526523f, 0.00049747858f, 0.0000852212106f};  //陀螺仪0飘补偿值
// float gyro_correct[3]={0.0f, 0.0f, 0.0f};  //陀螺仪0飘补偿值
volatile uint8_t attitude_flag=0;
volatile uint32_t correct_times=0;

pid_config temperature_ctrl_init = {0};

void INS_Init(void)
{
	
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
		pid_set_config_pos(&temperature_ctrl_init, 180.0f, 5.0f, 120.00f, 32.2f, 1500.0f); //温度pid参数设置
		pid_init((pid_t *)&imu_frame.temperature_pid, &temperature_ctrl_init);
		Mahony_Init(1000);  //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while (BMI088_init())
    {
        ;  //等待陀螺仪初始化完成
    }
    imu_frame.ref_temp = 40.0f; //设定目标温度40度
}

uint32_t temp_temperature=0;
void IMU_Temperature_Ctrl()
{
	  pid_calc((pid_t *)&imu_frame.temperature_pid, imu_frame.ref_temp, imu_frame.temp); //温度pid  //需要调一下pid使得温度在40°左右
		//由于PID计算中已经做了限幅处理 这里再做一次保护（看似没必要）
		float temp_out = imu_frame.temperature_pid.output;
		if(temp_out < 0.0f) temp_out = 0.0f;
		if (temp_out > 2000.0f) temp_out = 2000.0f;
		temp_temperature = (uint32_t)roundf(temp_out); //roundf 浮点数四舍五入取整
    htim3.Instance->CCR4 = temp_temperature;
}
/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
 
*/
static uint8_t first_mahony=0; 
void INS_Task(void)
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(imu_frame.gyro, imu_frame.accel, &imu_frame.temp);
				if(first_mahony==0)
				{
					first_mahony++;
					MahonyAHRSinit(imu_frame.accel[0], imu_frame.accel[1], imu_frame.accel[2], 0, 0, 0);  
				}
				
				if(attitude_flag==2)  //状态2 进入姿态解算
				{
					imu_frame.gyro[0] -= gyro_correct[0];   //减去陀螺仪0飘
					imu_frame.gyro[1] -= gyro_correct[1];
					imu_frame.gyro[2] -= gyro_correct[2];
					
					#if CHEAT              //作弊 可以让yaw很稳定 去掉比较小的值
						if(fabsf(imu_frame.gyro[2]) < GYRO_ZERO_THRESHOLD)
							imu_frame.gyro[2]=0;
					#endif
					//===========================================================================
						//ekf姿态解算部分
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
					IMU_QuaternionEKF_Update(imu_frame.gyro[0], imu_frame.gyro[1], imu_frame.gyro[2], imu_frame.accel[0], imu_frame.accel[1], imu_frame.accel[2]);
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
					//===============================================================================	
						
					//=================================================================================
					//mahony姿态解算部分
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
					Mahony_update(imu_frame.gyro[0], imu_frame.gyro[1], imu_frame.gyro[2], imu_frame.accel[0], imu_frame.accel[1], imu_frame.accel[2], 0, 0, 0);
					Mahony_computeAngles(); //角度计算   移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
					//=============================================================================
					//ekf获取姿态角度函数
					imu_frame.euler_angle.pitch=Get_Pitch(); //获得pitch
					imu_frame.euler_angle.roll=Get_Roll();//获得roll
					imu_frame.euler_angle.yaw=Get_Yaw();//获得yaw
					//==============================================================================
				}

				#if IMU_CHECK_ENABLE == 1 
				else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
				{
						//gyro correct
						gyro_correct[0] += imu_frame.gyro[0];
						gyro_correct[1] += imu_frame.gyro[1];
						gyro_correct[2] += imu_frame.gyro[2];
						correct_times++;
						if(correct_times >= CORRECT_TIM_DEFINE)
						{
							gyro_correct[0] /= (float)CORRECT_TIM_DEFINE;
							gyro_correct[1] /= (float)CORRECT_TIM_DEFINE;
							gyro_correct[2] /= (float)CORRECT_TIM_DEFINE;
							attitude_flag=2; //go to 2 state
						}
				}
				#else

				else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
				{
						attitude_flag=2; //go to 2 state
				}

				#endif
    }

    // temperature control
    if ((count % 10) == 0)
    {
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl();
				
		#if IMU_CHECK_ENABLE == 1 
				static uint32_t temp_Ticks=0;
				if((fabsf(imu_frame.temp - imu_frame.ref_temp) < 0.8f) && attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
				{
					temp_Ticks++;
					if(temp_Ticks > TEMP_STABLE_TIMES)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
					{
						attitude_flag=1;  //go to correct state
					}
				}
		#else
				if(attitude_flag==0)  //直接进入状态1 进行0飘初始化
				{
					attitude_flag=1;  //go to correct state
				}
		#endif
    }
    count++;
}

extern "C" void IMU_task(void *argument)
{
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
		
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
