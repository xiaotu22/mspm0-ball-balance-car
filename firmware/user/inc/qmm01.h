/*********************************************************************************************************************
* 文件名称          qmm01.h
* 功能说明          QMM01/CY-Z 串口陀螺仪驱动头文件
*                   协议：115200 8N1，固定 16 字节数据帧（AA 55 Seq[2] AngleDeg[4] GyroDps[4] CRC16[2] 55 AA）
*                   CRC-16/MODBUS（初值 0xFFFF，多项式 0xA001），校验范围 Seq+Angle+Gyro 共 10 字节
*                   默认接 MSPM0 UART1：TX=A8  RX=A9（若拓展板布不下可改 qmm01.c 内宏）
*********************************************************************************************************************/

#ifndef _qmm01_h_
#define _qmm01_h_

#include "zf_common_headfile.h"

typedef struct
{
    float   angle_deg;          // 积分角度，单位 deg
    float   gyro_dps;           // 滤波角速度，单位 deg/s
    uint16  seq;                // 帧序号
    uint32  last_rx_ms;         // 最后收到合法帧的毫秒时基
    uint8   valid;              // 是否收到过合法帧
} qmm01_data_t;

void    qmm01_init    (void);                               // 初始化 UART1 + 1ms 时基
void    qmm01_poll    (void);                               // 主循环调用，解析一帧
uint8   qmm01_get     (qmm01_data_t *out, uint32 *age_ms);  // 读最新数据，age_ms 可传 NULL
void    qmm01_zero    (void);                               // 发送角度清零命令（静止时用）
void    qmm01_calib   (void);                               // 发送零偏校准命令（静止时用，约 2s）

#endif
