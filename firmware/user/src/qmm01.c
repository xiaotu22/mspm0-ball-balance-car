/*********************************************************************************************************************
* 文件名称          qmm01.c
* 功能说明          QMM01/CY-Z 串口陀螺仪驱动
*                   数据帧 16 字节：AA 55 Seq[2 LE] AngleDeg[4 LE float] GyroDps[4 LE float] CRC16[2 LE] 55 AA
*                   命令帧 8  字节：A5 5A Cmd Param Seq CRC16[2 LE] 5A
*                   默认 UART1_A8/A9，波特率 115200
*********************************************************************************************************************/

#include "qmm01.h"

// ---------- 引脚/串口配置（按拓展板实际接线修改）----------
#define QMM01_UART              UART_1
#define QMM01_BAUD              115200
#define QMM01_TX_PIN            UART1_TX_A8
#define QMM01_RX_PIN            UART1_RX_A9

// ---------- 协议常量 ----------
#define QMM01_FRAME_LEN         ( 16 )
#define QMM01_RX_BUF_SIZE       ( 64 )          // 4 帧深度，足够 100Hz 数据流
#define QMM01_CMD_LEN           ( 8 )

// ---------- 环形缓冲区 ----------
static uint8  rx_buf[QMM01_RX_BUF_SIZE];
static volatile uint8 rx_head = 0;
static volatile uint8 rx_tail = 0;

static qmm01_data_t qmm01_latest = {0};
static volatile uint32 qmm01_ms = 0;

// ---------- CRC16/MODBUS ----------
static uint16 qmm01_crc16_modbus(const uint8 *data, uint32 len)
{
    uint16 crc = 0xFFFF;
    while(len--)
    {
        crc ^= *data++;
        for(int i = 0; i < 8; i++)
        {
            if(crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ---------- 1ms 时基 ----------
static void qmm01_tick_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    qmm01_ms++;
}

// ---------- UART 接收中断：只往环形缓冲区塞字节 ----------
static void qmm01_uart_callback(uint32 event, void *ptr)
{
    uint8 dat = 0;
    (void)ptr;

    if(event & UART_INTERRUPT_STATE_RX)
    {
        while(uart_query_byte(QMM01_UART, &dat))
        {
            uint8 next = (rx_head + 1) % QMM01_RX_BUF_SIZE;
            if(next != rx_tail)
            {
                rx_buf[rx_head] = dat;
                rx_head = next;
            }
        }
    }
}

// ---------- 初始化 ----------
void qmm01_init(void)
{
    rx_head = 0;
    rx_tail = 0;
    qmm01_latest.valid = 0;

    uart_init(QMM01_UART, QMM01_BAUD, QMM01_TX_PIN, QMM01_RX_PIN);
    uart_set_callback(QMM01_UART, qmm01_uart_callback, NULL);
    uart_set_interrupt_config(QMM01_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);

    pit_ms_init(PIT_TIM_G12, 1, qmm01_tick_callback, NULL);
}

// ---------- 主循环解析：找帧头、验帧尾、CRC、解析 ----------
void qmm01_poll(void)
{
    uint8  i = 0;
    uint8  frame[QMM01_FRAME_LEN];
    uint8  available = 0;
    uint16 crc_recv = 0;

    // 找帧头 AA 55，同时丢弃老旧/乱流字节
    while(rx_head != rx_tail)
    {
        if(rx_buf[rx_tail] == 0xAA)
        {
            uint8 next = (rx_tail + 1) % QMM01_RX_BUF_SIZE;
            if(next != rx_head && rx_buf[next] == 0x55)
            {
                available = (rx_head + QMM01_RX_BUF_SIZE - rx_tail) % QMM01_RX_BUF_SIZE;
                if(available >= QMM01_FRAME_LEN)
                {
                    break;      // 找到完整候选帧
                }
            }
        }
        rx_tail = (rx_tail + 1) % QMM01_RX_BUF_SIZE;
    }

    // 没帧头或不够长直接退出
    if((rx_head == rx_tail) || (available < QMM01_FRAME_LEN))
    {
        return;
    }

    // 拷贝 16 字节候选帧
    for(i = 0; i < QMM01_FRAME_LEN; i++)
    {
        frame[i] = rx_buf[(rx_tail + i) % QMM01_RX_BUF_SIZE];
    }

    // 校验尾 55 AA
    if((0x55 != frame[14]) || (0xAA != frame[15]))
    {
        rx_tail = (rx_tail + 1) % QMM01_RX_BUF_SIZE;    // 尾不对，只滑一个字节继续找
        return;
    }

    // CRC 校验：Seq[2] + Angle[4] + Gyro[4] = 10 字节
    crc_recv = (uint16)frame[12] | ((uint16)frame[13] << 8);
    if(qmm01_crc16_modbus(&frame[2], 10) != crc_recv)
    {
        rx_tail = (rx_tail + 1) % QMM01_RX_BUF_SIZE;
        return;
    }

    // 消费整帧
    rx_tail = (rx_tail + QMM01_FRAME_LEN) % QMM01_RX_BUF_SIZE;

    // 解析数据（小端，与协议一致）
    qmm01_latest.seq = (uint16)frame[2] | ((uint16)frame[3] << 8);
    memcpy(&qmm01_latest.angle_deg, &frame[4], 4);
    memcpy(&qmm01_latest.gyro_dps, &frame[8], 4);
    qmm01_latest.last_rx_ms = qmm01_ms;
    qmm01_latest.valid = 1;
}

// ---------- 读最新数据 ----------
uint8 qmm01_get(qmm01_data_t *out, uint32 *age_ms)
{
    if((NULL == out) || !qmm01_latest.valid)
    {
        return 0;
    }
    *out = qmm01_latest;
    if(NULL != age_ms)
    {
        *age_ms = qmm01_ms - qmm01_latest.last_rx_ms;
    }
    return 1;
}

// ---------- 发送命令帧 ----------
static void qmm01_send_cmd(uint8 cmd, uint8 param)
{
    uint8  frame[QMM01_CMD_LEN];
    uint16 crc = 0;

    frame[0] = 0xA5;
    frame[1] = 0x5A;
    frame[2] = cmd;
    frame[3] = param;
    frame[4] = 0x00;                            // Seq，上位机发 0 即可
    crc = qmm01_crc16_modbus(&frame[2], 3);     // Cmd+Param+Seq
    frame[5] = (uint8)(crc & 0xFF);
    frame[6] = (uint8)((crc >> 8) & 0xFF);
    frame[7] = 0x5A;

    uart_write_buffer(QMM01_UART, frame, QMM01_CMD_LEN);
}

// ---------- 角度清零（静止）----------
void qmm01_zero(void)
{
    qmm01_send_cmd(0x01, 0x01);
}

// ---------- 零偏校准（静止，约 2s）----------
void qmm01_calib(void)
{
    qmm01_send_cmd(0x01, 0x02);
}
