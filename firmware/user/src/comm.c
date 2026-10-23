/*********************************************************************************************************************
* 文件名称          comm
* 功能说明          K230 -> MSPM0 下行帧解析 + 上行命令帧发送（D:\diansai\通信协议规范.md v1.0）
*                   下行 $V,err,status*HH\r\n 定长 14B；上行 $C,mode,p1,p2*HH\r\n 定长 17B
*                   校验 = '$' 与 '*' 之间字节 XOR，两位大写 hex
* 时基说明          库里没有现成毫秒函数，用 PIT_TIM_G8 1ms 中断自维护 comm_ms（只用库注册机制，不改 isr.c；避开 TIM_A1，servo 用 PWM_TIM_A1）
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "comm.h"

#define COMM_RX_BUF_LEN     ( 32 )                      // 只收短帧，32 足够；超了说明流乱了
#define COMM_FRAME_LEN      ( 14 )                      // 下行帧定长（简化后只发 err+status）
#define COMM_UP_FRAME_LEN   ( 17 )                      // 上行帧定长
#define COMM_STAR_INDEX     ( 9 )                       // '*' 下标

static uint8  rx_buf[COMM_RX_BUF_LEN];
static uint8  rx_len        = 0;                        // 0 = 空闲等帧头
static uint8  frame_buf[COMM_FRAME_LEN];
static vuint8 frame_ready   = 0;

//-------------------------------------------------------------------------------------------------------------------
// 【2026-10-16 调试】原始字节抓取：串口 y=1 开、y=2 打印并关
//   为什么需要：出现过"MSPM0 收到 410 B/s 字节、但 327 秒解析不出一个合法帧"。
//   光看计分不出是"线缆干扰导致字节错"还是"帧格式/波特率不匹配"，必须看到原始字节。
//   开销：抓取开关关着时，本段只有一个 if 判断，正常跑车零成本。
//-------------------------------------------------------------------------------------------------------------------
#define COMM_DBG_RING   ( 56 )                          // 留 4 帧的窗口，够看出帧边界和错位
static uint8  dbg_ring[COMM_DBG_RING];
static uint8  dbg_len   = 0;
static uint8  dbg_on    = 0;
static uint32 dbg_total = 0;                            // 抓取期间收到的总字节数

void comm_dbg_raw (uint8 on)
{
    dbg_on    = (0 == on) ? 0 : 1;
    dbg_len   = 0;
    dbg_total = 0;
    printf("COMM RAW capture %s\r\n", (0 == on) ? "OFF" : "ON (y=2 打印)");
}

void comm_dbg_dump (void)
{
    uint8 i = 0;
    printf("COMM RAW total=%u len=%u : ", (unsigned int)dbg_total, (unsigned int)dbg_len);
    for(i = 0; i < dbg_len; i++)
    {
        printf("%02X ", dbg_ring[i]);
    }
    printf("\r\n");
    dbg_on = 0;
}

static int16  vision_err    = 0;
static uint8  vision_status = 0;
static uint8  vision_type   = 0;                        // 保留占位，简化协议中固定为 0

static vuint32 comm_ms      = 0;                        // 1ms 时基
static uint32 last_rx_ms    = 0;
static uint8  ever_valid    = 0;                        // 收到过合法帧才置 1
static uint32 err_count     = 0;

//-------------------------------------------------------------------------------------------------------------------
// 1ms PIT：自维护毫秒时基（库里无 system_get_ms，规范需要 age_ms 判掉线）
//-------------------------------------------------------------------------------------------------------------------
static void comm_tick_callback (uint32 event, void *ptr)
{
    comm_ms++;
}

//-------------------------------------------------------------------------------------------------------------------
// UART2 RX 中断回调：FIFO 里有多少取多少，逐字节喂状态机，不做解析
//-------------------------------------------------------------------------------------------------------------------
static void comm_uart_callback (uint32 event, void *ptr)
{
    uint8 dat = 0;

    if(event & UART_INTERRUPT_STATE_RX)
    {
        while(uart_query_byte(UART_2, &dat))
        {
            comm_rx_byte(dat);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 逐字节接收状态机：'$' 起头、'\n' 收尾、长度对才交帧，其余情况自恢复
//-------------------------------------------------------------------------------------------------------------------
static uint32 rx_total_cnt = 0;                         // 调试：统计 UART2 收到字节数

void comm_rx_byte (uint8 b)
{
    rx_total_cnt++;

    if(dbg_on)                                          // 【调试】原始字节抓取（关着时只多一次判断）
    {
        if(COMM_DBG_RING <= dbg_len)
        {
            memmove(dbg_ring, dbg_ring + 1, COMM_DBG_RING - 1);
            dbg_len = COMM_DBG_RING - 1;
        }
        dbg_ring[dbg_len++] = b;
        dbg_total++;
    }

    if(0 == (rx_total_cnt & 0x3FF))                     // 每 1024 字节打印一次，确认物理层有数据
    {
        printf("COMM RX bytes=%u\r\n", (unsigned int)rx_total_cnt);
    }

    if('$' == b)                                        // 任何时刻见帧头都重开，吃掉半帧/乱流
    {
        rx_len = 0;
        rx_buf[rx_len++] = b;
        return;
    }
    if(0 == rx_len)
    {
        return;
    }
    rx_buf[rx_len++] = b;
    if('\n' == b)
    {
        if((COMM_FRAME_LEN == rx_len) && !frame_ready)  // 主循环没取走就丢，不堆积
        {
            memcpy(frame_buf, rx_buf, COMM_FRAME_LEN);
            frame_ready = 1;
        }
        rx_len = 0;
    }
    if(COMM_RX_BUF_LEN <= rx_len)                       // 越界保护：超长未见过帧尾，丢弃重开
    {
        rx_len = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 两个 ASCII hex 转字节，非法字符返回 false
//-------------------------------------------------------------------------------------------------------------------
static bool hex_to_byte (uint8 hi, uint8 lo, uint8 *out)
{
    uint8 h = 0;
    uint8 l = 0;

    if     (('0' <= hi) && (hi <= '9')) h = hi - '0';
    else if(('A' <= hi) && (hi <= 'F')) h = hi - 'A' + 10;
    else return false;
    if     (('0' <= lo) && (lo <= '9')) l = lo - '0';
    else if(('A' <= lo) && (lo <= 'F')) l = lo - 'A' + 10;
    else return false;

    *out = (uint8)((h << 4) | l);
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 帧格式 + 校验检查，过则解析进 vision_*，并刷新收帧时间戳（只有合法帧才刷）
//-------------------------------------------------------------------------------------------------------------------
static void comm_parse_frame (void)
{
    uint8  i      = 0;
    uint8  chk    = 0;
    uint8  chk_rx = 0;
    int16  err    = 0;
    uint8  status = 0;
    uint8  type   = 0;

    // 先核骨架：帧种、分隔符、帧尾，位置全在规范 §2 定死
    if(('V' != frame_buf[1])  || (',' != frame_buf[2])  || (',' != frame_buf[7])  ||
       ('*' != frame_buf[COMM_STAR_INDEX]) ||
       ('\r' != frame_buf[12]) || ('\n' != frame_buf[13]) ||
       ((('+' != frame_buf[3]) && ('-' != frame_buf[3]))))
    {
        err_count++;
        return;
    }
    for(i = 4; i <= 6; i++)   if(('0' > frame_buf[i]) || (frame_buf[i] > '9')) { err_count++; return; }
    if((frame_buf[8] < '0') || (frame_buf[8] > '2'))  { err_count++; return; }  // status 0~2

    chk = 0;
    for(i = 1; i < COMM_STAR_INDEX; i++)                // 规范：'$' 与 '*' 之间逐字节 XOR
    {
        chk ^= frame_buf[i];
    }
    if(!hex_to_byte(frame_buf[10], frame_buf[11], &chk_rx) || (chk != chk_rx))
    {
        err_count++;                                    // 校验错：整帧丢弃，不刷数据不刷时间戳
        return;
    }

    err = (int16)((frame_buf[4] - '0') * 100 + (frame_buf[5] - '0') * 10 + (frame_buf[6] - '0'));
    if('-' == frame_buf[3])
    {
        err = (int16)(-err);
    }
    status = frame_buf[8]  - '0';
    type   = 0;

    vision_err    = err;
    vision_status = status;
    vision_type   = type;
    last_rx_ms    = comm_ms;
    ever_valid    = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 主循环周期调用：取走一帧解一帧
//-------------------------------------------------------------------------------------------------------------------
void comm_poll (void)
{
    if(frame_ready)
    {
        comm_parse_frame();
        frame_ready = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 读最新视觉数据；age_ms = 当前 - 最后合法帧；返回 false = 从未收到合法帧
// 掉线判定（age_ms >= 300）不在本层，交给上层（规范 §4）
//-------------------------------------------------------------------------------------------------------------------
bool comm_get_vision (int16 *err, uint8 *status, uint32 *age_ms)
{
    if((NULL == err) || (NULL == status) || (NULL == age_ms))
    {
        return false;
    }
    if(!ever_valid)
    {
        return false;
    }
    *err     = vision_err;
    *status  = vision_status;
    *age_ms  = comm_ms - last_rx_ms;                    // uint32 相减天然抗回绕
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 上行命令帧：$C,mode,p1,p2*HH\r\n（规范 §3，定长 17B）
//-------------------------------------------------------------------------------------------------------------------
void comm_send_mode (uint8 mode, uint16 p1, uint16 p2)
{
    char   buf[COMM_UP_FRAME_LEN + 1];
    char   body[12];                                    // "C,1,000,000" + '\0'
    uint8  chk = 0;
    uint8  i   = 0;

    if(3 < mode)                                        // mode 仅 0~3 合法，越界直接不发
    {
        return;
    }
    if(999 < p1) p1 = 999;                              // 钳位防超宽，保帧定长
    if(999 < p2) p2 = 999;

    snprintf(body, sizeof(body), "C,%d,%03d,%03d", mode, p1, p2);
    for(i = 0; body[i]; i++)
    {
        chk ^= body[i];
    }
    snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, chk);
    uart_write_buffer(UART_2, (uint8 *)buf, COMM_UP_FRAME_LEN);
}

uint32 comm_get_err_count (void)
{
    return err_count;
}

uint32 comm_get_ms (void)
{
    return comm_ms;
}

uint32 comm_get_frame_ms (void)
{
    return last_rx_ms;                              // 最后合法帧时刻（变化=新帧，帧级差分用）
}

//-------------------------------------------------------------------------------------------------------------------
// 【2026-10-16 调试】B18 引脚级诊断：把 UART2 RX 脚临时切成"带下拉的 GPIO 输入"读电平
//   为什么需要：出现过「K230 屏幕画面在动（=主循环在跑、每帧都在调 send_vision）
//   ── 但 MSPM0 收到 0 字节」，必须分清是【线断了/接触不良】还是【MSPM0 的 RX 脚坏了】。
//   原理：UART TX 空闲时为高电平。线接通 → 读 20 次基本全高；线悬空 → 被内部下拉成低。
//   读完立刻把 UART2 RX 恢复，不影响正常通信。
//-------------------------------------------------------------------------------------------------------------------
void comm_dbg_pin (void)
{
    uint8 i  = 0;
    uint8 hi = 0;

    gpio_init(B18, GPI, 0, GPI_PULL_DOWN);
    system_delay_ms(2);
    for(i = 0; i < 20; i++)
    {
        if(gpio_get_level(B18)) { hi++; }
        system_delay_ms(1);
    }

    uart_init(UART_2, 115200, UART2_TX_B15, UART2_RX_B18);      // 恢复 UART2 RX
    uart_set_callback(UART_2, comm_uart_callback, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);

    printf("PIN B18: high=%d/20 -> %s\r\n", hi,
           (10 <= hi) ? "线通且 K230 在驱动 -> 问题在 MSPM0 侧"
                      : "线未接通/悬空 -> 问题在线(或 K230 没在发)");
}

//-------------------------------------------------------------------------------------------------------------------
// UART2(B15/B18, 115200) + RX 回调注册 + 1ms 时基；全部走库注册机制，不改 isr.c
// 对接 K230（丝印 IO 口，勿用排针序号）：K230 IO3=TX1 -> B18(RX)；K230 IO4=RX1 <- B15(TX)；GND 共地
// ⚠️ B16 疑物理损伤（2026-08-01 排障确认），RX 已固定改走 B18，接线务必接 B18 不要接 B16
//-------------------------------------------------------------------------------------------------------------------
void comm_init (void)
{
    uart_init(UART_2, 115200, UART2_TX_B15, UART2_RX_B18);  /* 2026-08-01: B16 疑物理损伤，RX 改 B18（排障换脚）*/
    uart_set_callback(UART_2, comm_uart_callback, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);

    pit_ms_init(PIT_TIM_G8, 1, comm_tick_callback, NULL);
}
