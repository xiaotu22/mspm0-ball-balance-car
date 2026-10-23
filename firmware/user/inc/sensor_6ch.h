/*********************************************************************************************************************
* 文件名称          sensor_6ch
* 功能说明          幻尔六路 I2C 循迹模块驱动（AiBlock/Hiwonder LineFollower_6CH）
*                   I2C 7 位地址 0x5C；SCL=B8 SDA=B26（模块丝印 SDL=SCL），3.3V 电平直连
*                   寄存器：5=数字状态 1B（bit0~5=通道1~6，1=触发）；6=模拟量 12B（6×u16 小端）
*                   校准走模块板载 KEY 一键学习（识别高度必须=学习高度）——物理操作，小土执行
********************************************************************************************************************/

#ifndef _sensor_6ch_h_
#define _sensor_6ch_h_

#include "zf_common_headfile.h"

void   sensor_6ch_init (void);
bool   sensor_6ch_read (uint8 *state, uint16 analog[6]);    // NACK/失败返回 false，数据不更新
uint32 sensor_6ch_get_fail_cnt (void);                      // 连续失败计数（成功清零），排查用
uint8  sensor_6ch_read_thr (uint16 thr[6]);                 // 读六路学习阈值（插值归一化基准），全 0/失败返回 0
void   sensor_6ch_i2c_scan (void);                          // I2C 总线扫描（排障），串口 k 命令触发

#endif
