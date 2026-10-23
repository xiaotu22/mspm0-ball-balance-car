/*********************************************************************************************************************
* 文件名称          comm
* 功能说明          K230 <-> MSPM0 UART2 通信协议（规范：D:\diansai\通信协议规范.md v1.0）
*                   中断里只做逐字节入队，解析在 comm_poll（主循环）
********************************************************************************************************************/

#ifndef _comm_h_
#define _comm_h_

#include "zf_common_typedef.h"

void   comm_init (void);
void   comm_rx_byte (uint8 b);                          // 仅供 UART2 RX 回调调用
void   comm_poll (void);                                // 主循环周期调用
bool   comm_get_vision (int16 *err, uint8 *status, uint32 *age_ms);
void   comm_send_mode (uint8 mode, uint16 p1, uint16 p2);
uint32 comm_get_err_count (void);                       // 校验错帧累计，供 UART0 打印
uint32 comm_get_ms (void);                              // comm 自维护 1ms 时基（题3状态机等主循环计时用）
uint32 comm_get_frame_ms (void);                        // 最后合法帧时刻（变化=新视觉帧，帧级 KD 差分用）
void   comm_dbg_raw (uint8 on);                         // 【调试】原始字节抓取开/关（串口 y=1）
void   comm_dbg_dump (void);                            // 【调试】打印抓到的原始字节并关闭（串口 y=2）
void   comm_dbg_pin (void);                             // 【调试】B18 引脚电平诊断（串口 y=3）

#endif
