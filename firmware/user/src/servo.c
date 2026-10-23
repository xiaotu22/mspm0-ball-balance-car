/*********************************************************************************************************************
* 文件名称          servo
* 功能说明          摆杆舵机驱动
*                   TIM_A1_CH0_B4 @50Hz（与电机 TIM_A0@10kHz 不同定时器，互不干扰）
*                   占空换算：duty 0~10000 对应 0~20000µs → duty = us × 10000 / 20000
*                   （500µs=250、1500µs=750、2500µs=1250，分辨率 2µs/LSB）
*                   供电红线：舵机走独立 5V（≥3A 余量），严禁与逻辑 5V 共用，必须共地
********************************************************************************************************************/

#include "servo.h"

#define SERVO_PWM_PIN   ( PWM_TIM_A1_CH1_B3 )   // 实物接 B3，已切换
#define SERVO_FREQ      ( 50 )
#define SERVO_PERIOD_US ( 20000 )

static uint16 servo_us = SERVO_MID_US;          // 当前脉宽（上电=中位）

void servo_init (void)
{
    pwm_init(SERVO_PWM_PIN, SERVO_FREQ, (uint32)SERVO_MID_US * PWM_DUTY_MAX / SERVO_PERIOD_US);
}

void servo_set_us (uint16 us)
{
    if(SERVO_ABS_MAX_US < us)
    {
        us = SERVO_ABS_MAX_US;
    }
    else if(SERVO_ABS_MIN_US > us)
    {
        us = SERVO_ABS_MIN_US;
    }
    servo_us = us;
    pwm_set_duty(SERVO_PWM_PIN, (uint32)us * PWM_DUTY_MAX / SERVO_PERIOD_US);
}

uint16 servo_get_us (void)
{
    return servo_us;
}
