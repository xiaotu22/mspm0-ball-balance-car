/*********************************************************************************************************************
* 文件名称          motor
* 功能说明          TB6612 双路电机驱动封装（左 ch0 / 右 ch1）
*                   正转逻辑定义：IN1=H IN2=L + PWM（物理方向对不对由 A02 实测定，此后以实测为准）
*                   换向保护：目标转向与当前相反时先刹车 10ms（TB6612 红线）
********************************************************************************************************************/

#ifndef _motor_h_
#define _motor_h_

#include "zf_common_headfile.h"

#define MOTOR_CH_L      ( 0 )
#define MOTOR_CH_R      ( 1 )

void motor_init   (void);
void motor_set    (uint8 ch, int16 duty);   // duty: -10000~10000，负 = 反转，0 = 刹车
void motor_brake  (uint8 ch);               // 刹车：IN1=IN2=H 且 PWM 高（TB6612 真值表）
void motor_coast  (uint8 ch);               // 滑行：PWM=0
void motor_enable (uint8 en);               // 全局电机使能：0=强制刹车忽略所有 motor_set，1=正常响应

#endif
