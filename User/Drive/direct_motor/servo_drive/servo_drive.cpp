#include "servo_drive.h"

void servo_class::set_servo_form(servo_form_e __servo_form)
{
    servo_form = __servo_form;

    switch (servo_form)
    {
        case SERVO_90:
            angle_max = 90.0f;
            break;
        case SERVO_180:
            angle_max = 180.0f;
            break;
        case SERVO_270:
            angle_max = 270.0f;
            break;
        case SERVO_360:
            angle_max = 360.0f;
            break;
        default:
            angle_max = 180.0f; // 默认设置为180度
            break;
    }
}

void servo_class::set_angle(float __angle)
{
    if (__angle < 0.0f || __angle > angle_max)
    {
        return;
    }

    angle = __angle;

    float duty_ratio = 0.025f + (__angle / angle_max) * 0.10f; // 假设占空比范围为5%到15%

    set_duty_ration(duty_ratio);
}

void servo_class::drv_servo_init(TIM_HandleTypeDef *pwm_handle, uint32_t __pwm_channel, servo_form_e __servo_form)
{
    set_tim_handle(pwm_handle);
    set_pwm_channel(__pwm_channel);
    set_servo_form(__servo_form);
    start_pwm();
    set_angle(0.0f); // 初始化时设置角度为0度
}