/*********************************************************************************************************************
* 文件名称          motor
* 功能说明          TB6612 双路电机驱动封装
*                   左：PWMA=A0(TIM_A0_CH0) AIN1=B13 AIN2=B12
*                   右：PWMB=A1(TIM_A0_CH1) BIN1=B2  BIN2=A13
*                   PWM 10kHz；上电默认刹车态，防止初始化间隙电机乱转
********************************************************************************************************************/

#include "motor.h"

#define MOTOR_PWM_FREQ      ( 10000 )

#define MOTOR_L_PWM         ( PWM_TIM_A0_CH0_A0 )
#define MOTOR_L_IN1         ( B13 )
#define MOTOR_L_IN2         ( B12 )

#define MOTOR_R_PWM         ( PWM_TIM_A0_CH1_A1 )
#define MOTOR_R_IN1         ( B2  )
#define MOTOR_R_IN2         ( A13 )

#define MOTOR_REVERSE_BRAKE_MS    ( 10 )        // 换向前的最短刹车时间（红线）

#define MOTOR_L_POL     ( +1 )                  // 左轮极性：+duty 应 = 车前进，A04 开机自检实测反了就改 -1
#define MOTOR_R_POL     ( +1 )                  // 右轮同上

static const uint32 motor_pwm_pin[2] = {MOTOR_L_PWM, MOTOR_R_PWM};
static const gpio_pin_enum motor_in1[2]  = {MOTOR_L_IN1, MOTOR_R_IN1};
static const gpio_pin_enum motor_in2[2]  = {MOTOR_L_IN2, MOTOR_R_IN2};
static const int8 motor_pol[2]  = {MOTOR_L_POL, MOTOR_R_POL};

static int8 motor_dir[2] = {0, 0};              // 当前驱动方向：+1 正 -1 反 0 未驱动；刹车/滑行不清，供换向判断
static volatile uint8 g_motor_enabled = 1;      // 全局电机使能：x=0 时强制关断，最高优先级（ISR/main 双上下文，volatile 防优化）

//-------------------------------------------------------------------------------------------------------------------
// 全局电机使能：0=任何 motor_set 都直接刹车；1=正常响应
//-------------------------------------------------------------------------------------------------------------------
void motor_enable (uint8 en)
{
    g_motor_enabled = (0 == en) ? 0 : 1;
    if(0 == g_motor_enabled)
    {
        motor_brake(MOTOR_CH_L);
        motor_brake(MOTOR_CH_R);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 上电先刹车：方向脚全部拉高、PWM 满占空，真值表 IN1=IN2=H 且 PWM=H 为刹车态
//-------------------------------------------------------------------------------------------------------------------
void motor_init (void)
{
    uint8 i = 0;
    for(i = 0; i < 2; i++)
    {
        gpio_init(motor_in1[i], GPO, 1, GPO_PUSH_PULL);
        gpio_init(motor_in2[i], GPO, 1, GPO_PUSH_PULL);
        pwm_init((pwm_channel_enum)motor_pwm_pin[i], MOTOR_PWM_FREQ, PWM_DUTY_MAX);
    }
}

void motor_brake (uint8 ch)
{
    if(2 <= ch)
    {
        return;
    }
    gpio_set_level(motor_in1[ch], 1);
    gpio_set_level(motor_in2[ch], 1);
    pwm_set_duty((pwm_channel_enum)motor_pwm_pin[ch], PWM_DUTY_MAX);
}

void motor_coast (uint8 ch)
{
    if(2 <= ch)
    {
        return;
    }
    pwm_set_duty((pwm_channel_enum)motor_pwm_pin[ch], 0);
}

//-------------------------------------------------------------------------------------------------------------------
// duty > 0：IN1=H IN2=L（定义为正转）；duty < 0 反之；duty = 0 刹车
// 与当前方向相反时先刹车 MOTOR_REVERSE_BRAKE_MS 再换向
//-------------------------------------------------------------------------------------------------------------------
void motor_set (uint8 ch, int16 duty)
{
    int8 target_dir = 0;

    if(2 <= ch)
    {
        return;
    }

    if(0 == g_motor_enabled)                    // x=0 最高优先级：任何控制环调用都直接刹车
    {
        motor_brake(ch);
        return;
    }

    duty = (int16)(duty * motor_pol[ch]);       // 极性统一在此翻转：对外 +duty 恒 = 车前进

    if(PWM_DUTY_MAX < duty)
    {
        duty = PWM_DUTY_MAX;
    }
    else if(-PWM_DUTY_MAX > duty)
    {
        duty = -PWM_DUTY_MAX;
    }

    if(0 == duty)
    {
        motor_brake(ch);
        return;
    }
    target_dir = (0 < duty) ? 1 : -1;

    if((0 != motor_dir[ch]) && (target_dir != motor_dir[ch]))
    {
        motor_brake(ch);
        system_delay_ms(MOTOR_REVERSE_BRAKE_MS);
    }
    motor_dir[ch] = target_dir;

    if(0 < duty)
    {
        gpio_set_level(motor_in1[ch], 1);
        gpio_set_level(motor_in2[ch], 0);
    }
    else
    {
        gpio_set_level(motor_in1[ch], 0);
        gpio_set_level(motor_in2[ch], 1);
    }
    pwm_set_duty((pwm_channel_enum)motor_pwm_pin[ch], (0 < duty) ? (uint32)duty : (uint32)(-duty));
}
