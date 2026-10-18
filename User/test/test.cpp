#include "test.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "fdcan.h"
// #include "stm32h7xx_hal_gpio.h"
// #include "usart.h"
// #include "bsp_pwm.h"
// #include "bsp_can.h"
// #include "bsp_adc.h"
// #include "mks_drive.h"
// #include "buzzer.h"
// #include "wave_generator.h"
// #include "imu_odometer.h"
// #include "hi12_imu.h"
// #include "dji_motor_new.h"
// #include "regular_key.h"

#include "bsp_dwt.h"
// #include <cstdint>

// #include "lk_motor.h"

// // #include "usb_vofa.h"

// #include "uart_vofa.h"

#include "ws2812.h"

// #include "st7789_73x219_lcd.h"
// #include "73x219_lcd_picture.h"
// #include "bsp_adc.h"
// #include "adc_key.h"

// #include "servo_drive.h"

// lcd_key_t key_state = LCD_KEY_NONE;
// lcd_key_t last_key = LCD_KEY_NONE;    

// DJI新电机驱动库测试程序
/*
dji_c610_motor_class dji_motor_test;

// pwm_class pwm_test;

// buzzer_class buzzer_test;
// wave_generator_t wave_test;

// dm_motor_normal_class dm_motor_test;

// uint8_t tx_data[5] = {0xF6, 0x01, 0x40, 0x02, 0x3A};

// navigation_class navigation;
// volatile hi12_gyro *hi12_imu;

// float adc_voltage;

// float target_value = 0.0f;

// mks_motor_drive_t mks_motor;


//  * FIR低通滤波器 — 滤除编码器量化噪声, 低延迟优化设计 (v2)
//  *
//  * 实测对比 (10阶→14阶):
//  *   - 10阶/Fc50Hz: 输出轴跳变 0.861→0.389 (-55%), 转子跳变 26.8→14.1 (-47%)
//  *   - 14阶/Fc40Hz: 过渡带 550→393Hz (收窄29%), 噪声衰减 √11→√15 (+18%)
//  *   - 延迟 5ms→7ms (+2ms), 仍远低于1kHz控制周期, 人感无差异
//  *
//  * 参数选择:
//  *   - 阶数 14 (15抽头): 延迟 7ms, 过渡带宽 ~393Hz (Blackman: Δf≈5.5×Fs/N)
//  *   - 截止频率 40Hz: 4×电机机械带宽(~10Hz), 不影响动态跟踪
//  *   - 限幅 ±500 rad/s: C610转子额定 ~1047rad/s, 安全余量充足
//  *
//  * 编码器量化 (8192 CPR, 1kHz):
//  *   量化台阶 = 0.767 rad/s → 15抽头FIR → 等效 ≈ 0.767/√15 ≈ 0.20 rad/s
 
class_filter_frequency<14> lowpass_filter;  // 14阶 FIR, Blackman窗, 7ms延迟

float filter_omega = 0.0f; // 滤波后的转子角速度, rad/s (Ozone观测变量)

float filter_angle = 0.0f; // 电机角度, rad

float filter_current = 0.0f; // 电机电流, A

void test_dji_motor_control_init(void)
{
    dji_motor_test.init(&hfdcan2, Motor_DJI_ID_0x201, DJI_Motor_Control_Method_Omega);
    dji_motor_test.pid_omega.init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    dji_motor_test.set_target_omega(3.0f);

    // 初始化 FIR 低通: 限幅±500, 低通, Fc=40Hz, Fs=1000Hz
    lowpass_filter.init(-500.0f, 500.0f, filter_frequency_type_lowpass, 40.0f, 500.0f, 1000.0f);
}

void test_dji_motor_control(void)
{
    // 1. 获取转子原始角速度 (rad/s)
    float raw_rotor_omega = dji_motor_test.get_rotor_omega();
    filter_angle = dji_motor_test.get_rotor_angle();
    filter_current = dji_motor_test.get_rotor_current();
    // 2. FIR 低通滤波 (14阶, 7ms延迟)
    lowpass_filter.set_now(raw_rotor_omega);
    lowpass_filter.frequency_filter_calculate_callback();
    filter_omega = lowpass_filter.get_out();

    // 3. 滤波后的输出轴角速度存入电机对象 (供 Ozone 观测)
    dji_motor_test.set_filter_angle(filter_angle / dji_motor_test.get_gearbox_rate());
    dji_motor_test.set_filter_omega(filter_omega / dji_motor_test.get_gearbox_rate());
    dji_motor_test.set_filter_torque(filter_current / dji_motor_test.get_current_torque_rate());


    dji_motor_test.external_calculate_callback();

    bsp_can_1_to_4_transmit(&hfdcan2, 0x200, can2_0x200_tx_data, Can_transmit_frequency_1000Hz);
}
*/
/*
key_callback_t user_key_test;

// 初始化按键触发状态
uint8_t user_key_read(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
}

// 初始化按键触发时间
uint32_t user_time_read(void)
{
    return (uint32_t)bsp_dwt_get_timeline_ms();
}

// 按键按下回调函数
void user_key_down(void)
{

}

// 按键释放回调函数
void user_key_up(void)
{

}

// 按键单击回调函数
void user_key_click(void)
{

}

// 按键长按回调函数
void user_key_longpress(void)
{

}

// 按键双击回调函数
void user_key_double_click(void)
{

}

// 按键空闲回调函数
void user_key_idle(void)
{

}

// 按键初始化函数
void user_key_init(void)
{
    key_init(&user_key_test, 
        user_key_up, user_key_down, 
        user_key_click, user_key_longpress, 
        user_key_double_click, user_key_idle, 
        user_key_read, 
        user_time_read);
}

void user_key_scan(void)
{
    key_state_scan(&user_key_test);
}
*/

// lk_motor_normal_class lk_motor_test;

// 由于: DMA无法访问DTCM(0x20000000)，必须放在DMA可访问的RAM_D1(0x24000000)
// float wave_out;
// float vofa_out;
// float key_velocity = 0.0f;
// float adc_voltage = 0.0f;
// float angle = 0.0f;
ws2812_class ws2812;
// servo_class servo_test;

// wave_generator_t wave_test;

// lcd_key_t lcd_key_state; // LCD按键状态

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

void ws2812_color_flicker(void)
{
    static uint16_t mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        ws2812.set_rgb(red, green, blue);
        ws2812.ws2812_write_color();
    }
}

// menu_page_t Q_just_shot = {0};
// menu_page_t Q_find_shot = {0};
// menu_page_t Q_track_shot = {0};
// menu_page_t Q_track_shot2 = {0};
// menu_page_t Q_track_draw = {0};

// menu_page_t Q_select = {0};
// menu_page_t Q_running = {0};

// static uint8_t test_id;

// void menu_page_init(void)
// {
//     Q_just_shot = (menu_page_t){
//         .menu_name = "Just Shot  ",
//         .next = &Q_find_shot,
//         .prev = &Q_track_draw,
//         .child = &Q_select,
//         .task_id = 1,
//     };
//     Q_find_shot = (menu_page_t){
//         .menu_name = "Find & Shot",
//         .next = &Q_track_shot,
//         .prev = &Q_just_shot,
//         .child = &Q_select,
//         .task_id = 2,
//     };
//     Q_track_shot = (menu_page_t){
//         .menu_name = "Track & Shot ",
//         .next = &Q_track_shot2,
//         .prev = &Q_find_shot,
//         .child = &Q_select,
//         .task_id = 3,
//     };
//     Q_track_shot2 = (menu_page_t){
//         .menu_name = "Track & Shot2",
//         .next = &Q_track_draw,
//         .prev = &Q_track_shot,
//         .child = &Q_select,
//         .task_id = 4,
//     };
//     Q_track_draw = (menu_page_t){
//         .menu_name = "Track & Draw",
//         .next = &Q_just_shot,
//         .prev = &Q_track_shot2,
//         .child = &Q_select,
//         .task_id = 5,
//     };

//     Q_select = (menu_page_t){
//         .menu_name = "Press to Run",
//         .next = NULL,
//         .prev = NULL,
//         .child = NULL, // 没有子菜单
//         .task_id = 0, // 0表示没有任务
//     };
//     Q_running = (menu_page_t){
//         .menu_name = "I am Running!",
//         .next = NULL,
//         .prev = NULL,
//         .parent = &Q_select,
//         .task_id = 0, // 0表示没有任务
//     };
// }

// void LcdTask_select(lcd_key_t key_press){
//     // 菜单页
//     static menu_page_t *last_page = NULL;
//     static menu_page_t *current_page = &Q_just_shot;

//     switch (key_press)
//     {
//     case LCD_KEY_RIGHT:
//         if(current_page->next != NULL){
//             current_page = current_page->next;
//         }
//         break;
    
//     case LCD_KEY_LEFT:
//         if(current_page->prev != NULL){
//             current_page = current_page->prev;
//         }
//         break;
//     case LCD_KEY_PRESS:
//         if(current_page->task_id != 0)
//         {
//             if(current_page->child != NULL){
//                 current_page->child->task_id = current_page->task_id;
//                 current_page->child->parent = current_page;
//                 current_page = current_page->child;
//             }
//             else if(current_page->child == NULL){
//                 // 执行任务
//                 test_id = current_page->task_id;
//                 Q_running.task_id = current_page->task_id; // 设置运行状态页的任务ID
//                 current_page = &Q_running; // 切换到运行状态页
//                 LCD_ShowIntNum(0, 60, current_page->task_id, 1, RED, BLACK, 32);
//             }
//         }
//         break;
//     case LCD_KEY_UP:
//         if(current_page->parent != NULL){
//             current_page = current_page->parent;
//         }
//     default:
//         break;
//     }
    
//     if (last_page != current_page)
//     {
//         last_page = current_page;
//         LCD_Fill(0, 20, LCD_W, 20 + 32, BLACK);
//         LCD_ShowString(0, 20, current_page->menu_name, RED, BLACK, 32, 0);
//     }
    

// }

// uint8_t count = 0;

extern "C" void test_task(void *argument)
{
    /* USER CODE BEGIN test_task */
    /* Infinite loop */
    // pwm_test.set_tim_handle(&htim1);
    // pwm_test.set_pwm_channel(0, TIM_CHANNEL_1);
    // pwm_test.set_duty_ration(0, 0.5f);
    // pwm_test.start_pwm();
    // test_dji_motor_control_init();
    // buzzer_test.init(4000.0f, 0.5f);
    // test_dji_motor_control_init();
    // user_key_init();
    // lk_motor_test.init(&hfdcan2, 0x01, Motor_LK_Control_Method_Normal_Torque);
    // menu_page_init();
    bsp_spi_init(&hspi6, nullptr); // 初始化SPI6
    // wave_generator_init(&wave_test, WAVE_SINE, 0.2f, 5.0f, 0.0f);
    // wave_generator_enable(&wave_test);
    osDelay(1000);
    ws2812.color_init(0, 0, 0); // 初始化为红色
    ws2812.ws2812_write_color();
    // servo_test.drv_servo_init(&htim1, TIM_CHANNEL_1, SERVO_180);
    // LCD_Init();//LCD初始化
    // LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
    
    // LcdTask_select(LCD_KEY_RIGHT); // 初始化菜单页
    // test_dji_motor_control_init();
    // mks_motor_position_ctrl(&hfdcan1, 1, POSITION_PULSE_COMMAND, CAN_COMMUNIACTION_MODE, 1.0f, 16, 360.0f, 1000.0f, 100); // 发送速度控制命令，参数根据需要调整
    for(;;)
    {
        // test_dji_motor_control();
        // navigation.update(hi12_imu);
        // adc_voltage = bsp_get_adc_voltage();
        // bsp_fdcan_send(&hfdcan1, 0x1, (uint8_t*)0x01, 1);
        // bsp_uart_transmit(&huart7, (uint8_t*)"Hello, World!\r\n", 15);
        // dji_motor_ctrl(&hfdcan1, DJI_L3508_L2006_FB_ID, 0, 0, 0, 0); // 发送控制命令，参数根据需要调整
        // dm_motor_test.set_control_omega(5.0f);
        // dm_motor_test.set_control_torque(0.0f);
        // dm_motor_test.output();
        // lowpass_filter.set_now(motor_fibpara.motor.rx_fdpara.angle_speed);
        // lowpass_filter.frequency_filter_calculate_callback();
        // motor_fibpara.motor_speed = lowpass_filter.get_out();
        // bsp_fdcan_send(&hfdcan1, 0x001, tx_data, 5);
        // target_value = wave_generator_calc(&wave_test);
        // mks_motor_read_speed_value(&hfdcan1, 1, CAN_COMMUNIACTION_MODE); // 发送读取速度值命令，参数根据需要调整
        // mks_motor_speed_ctrl(&hfdcan1, 1, CAN_COMMUNIACTION_MODE, 1.0f, 3000.0f, 100); // 发送速度控制命令，参数根据需要调整
        // user_key_scan();
        // lk_motor_test.set_control_torque(0.02f);
        // lk_motor_test.calculate_callback();
        // wave_out = wave_generator_calc(&wave_test);
        ws2812_color_flicker();
        // uart_vofa_data_rx();
        // uart_vofa_start();
        // ws2812_color_flicker();
        // bsp_uart_transmit(&huart1, (uint8_t*)&wave_out, sizeof(float));
        // usb_vofa_data_rx();
        // usb_vofa_start();
        // key_velocity = bsp_get_adc_lcd_key_voltage();
        // adc_voltage = bsp_get_adc_power_voltage();
        // lcd_key_state = get_lcd_key_state();
        // count = ((key_state != (lcd_key_t)lcd_key_state) && (lcd_key_state == last_key)) ? (count + 1) : 0;
        // if(count >= 3){
        //     key_state = lcd_key_state;
        //     if (key_state != LCD_KEY_NONE) 
        //     {
        //         LcdTask_select(key_state);
        //     }
        // }
        // LCD_ShowString(120, 72,(uint8_t *)"dmBot", BRRED, BLACK, 24, 0);
        // LCD_ShowChinese(84, 100, (uint8_t *)"达妙科技", WHITE, BLACK, 32, 0);
        // LCD_DrawLine(10, 0, 10,  280,WHITE);
        // LCD_DrawLine(270,0, 270, 280,WHITE);
        // LCD_ShowPicture(180, 150, 40, 40, gImage_1);
        // servo_test.set_angle(angle);
//        LCD_ShowIntNum(50, 170, adc_val[0], 5, WHITE, BLACK, 32);
        // last_key = lcd_key_state;
        osDelay(10);
    }
    /* USER CODE END test_task */
}

// void fdcan1_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
// {
//     if (rx_frame == NULL) {
// 		return;
// 	}

// 	/* 处理接收到的CAN消息 */
// 	uint16_t rec_id = rx_frame->header.Identifier;
// 	uint8_t rx_data[8] = {0};

// 	/* 从 volatile 数据复制到本地缓冲区（逐字节） */
// 	for (uint8_t i = 0; i < 8; ++i) {
// 		rx_data[i] = rx_frame->data[i];
// 	}

//     switch (rec_id)
//     {

//     case 0x001: // 根据实际ID进行判断
//     {
//         // 处理接收到的CAN消息，参数根据需要调整
//         mks_motor_original_feedback(&mks_motor, rx_data);
//         break;
//     }

//     case 0x201: // 根据实际ID进行判断
//     {
//         // mks_motor_fbdata(&mks_motor, rx_data);
//     }

//     default:
//         break;
//     }
// }

// void fdcan2_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
// {
//     if (rx_frame == NULL) {
//         return;
//     }

//     switch (rx_frame->header.Identifier)
//     {

//         case 0x141: // 根据实际ID进行判断
//         {
//             // 处理接收到的CAN消息，参数根据需要调整
//             // dji_motor_test.can_rx_callback();
//             // lk_motor_test.can_rx_callback();
//             break;
//         }
        
//         default:
//             break;
//     }
// }