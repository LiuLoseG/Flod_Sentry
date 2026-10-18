#pragma once

#include "bsp_pwm.h"

typedef enum {
    SERVO_90 = 0,
    SERVO_180,
    SERVO_270,
    SERVO_360
} servo_form_e;

class servo_class : public pwm_class
{
    public:

        void set_servo_form(servo_form_e __servo_form);

        void set_angle(float __angle);

        void drv_servo_init(TIM_HandleTypeDef *pwm_handle, uint32_t __pwm_channel, servo_form_e __servo_form);

    private:

        servo_form_e servo_form = SERVO_180;

        float angle = 0.0f;

        float angle_max = 180.0f;
};
