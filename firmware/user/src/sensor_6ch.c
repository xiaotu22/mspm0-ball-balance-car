/*********************************************************************************************************************
* 文件名称          sensor_6ch
* 功能说明          幻尔六路 I2C 循迹模块驱动
*                   库高层 soft_iic_read_8bit_registers 不回传 ACK 状态，本驱动用底层
*                   start/send_data/read_data/stop 自组时序：任一字节 NACK 即判失败，
*                   缓冲不更新（防拿旧数据/全 0 数据当有效）
*                   通信节奏：主循环 10ms 读一次（与循迹环同节奏）；软 I2C 位带延时阻塞
*                   约 1ms 级，不放 PIT ISR（ISR 不做长操作）
********************************************************************************************************************/

#include "sensor_6ch.h"

#define SENSOR_6CH_ADDR         ( 0x5C )        // 模块 7 位地址（官方协议）
#define SENSOR_6CH_DELAY        ( 50  )         // 软 I2C 时钟延时，参考 imu660ra 例程量级
#define SENSOR_6CH_SCL          ( B8  )
#define SENSOR_6CH_SDA          ( B26 )

#define SENSOR_6CH_REG_STATE    ( 5 )           // 数字状态：1 字节，bit0~5 = 通道 1~6
#define SENSOR_6CH_REG_ANALOG   ( 6 )           // 模拟量：12 字节，6×u16 小端
#define SENSOR_6CH_REG_THR      ( 0x18 )        // 学习阈值：12 字节，6×u16 小端（插值归一化基准）
#define SENSOR_6CH_CH_NUM       ( 6 )

static soft_iic_info_struct sensor_6ch_iic;
static uint32 sensor_6ch_fail_cnt = 0;          // 连续失败计数

void sensor_6ch_init (void)
{
    soft_iic_init(&sensor_6ch_iic, SENSOR_6CH_ADDR, SENSOR_6CH_DELAY, SENSOR_6CH_SCL, SENSOR_6CH_SDA);
}

//-------------------------------------------------------------------------------------------------------------------
// 自组带 NACK 检测的寄存器读：写地址/寄存器号任一步 NACK → 停止并返回 0
// send_data 返回 1=ACK 0=NACK（库实现：wait_ack 读 SDA 高=NACK 返回1，send_data 再取反）
// 7/30 排障：曾按 0=ACK 写反，表现为地址段全 NACK、fail 风暴、扫描 111 台假设备
//-------------------------------------------------------------------------------------------------------------------
static uint8 sensor_6ch_read_regs (uint8 reg, uint8 *buf, uint32 len)
{
    soft_iic_start(&sensor_6ch_iic);
    if(0 == soft_iic_send_data(&sensor_6ch_iic, SENSOR_6CH_ADDR << 1))
    {
        soft_iic_stop(&sensor_6ch_iic);
        return 0;
    }
    if(0 == soft_iic_send_data(&sensor_6ch_iic, reg))
    {
        soft_iic_stop(&sensor_6ch_iic);
        return 0;
    }
    soft_iic_start(&sensor_6ch_iic);            // repeated start
    if(0 == soft_iic_send_data(&sensor_6ch_iic, (SENSOR_6CH_ADDR << 1) | 0x01))
    {
        soft_iic_stop(&sensor_6ch_iic);
        return 0;
    }
    while(len--)
    {
        *buf++ = soft_iic_read_data(&sensor_6ch_iic, (0 == len));
    }
    soft_iic_stop(&sensor_6ch_iic);
    return 1;
}

bool sensor_6ch_read (uint8 *state, uint16 analog[6])
{
    uint8 st = 0;
    uint8 raw[SENSOR_6CH_CH_NUM * 2];
    uint8 i = 0;

    if((NULL == state) || (NULL == analog))
    {
        return false;
    }
    if(!sensor_6ch_read_regs(SENSOR_6CH_REG_STATE, &st, 1))
    {
        sensor_6ch_fail_cnt++;
        return false;
    }
    if(!sensor_6ch_read_regs(SENSOR_6CH_REG_ANALOG, raw, SENSOR_6CH_CH_NUM * 2))
    {
        sensor_6ch_fail_cnt++;
        return false;
    }
    *state = st;
    for(i = 0; i < SENSOR_6CH_CH_NUM; i++)      // 小端拼 u16
    {
        analog[i] = (uint16)raw[2 * i] | ((uint16)raw[2 * i + 1] << 8);
    }
    sensor_6ch_fail_cnt = 0;
    return true;
}

uint32 sensor_6ch_get_fail_cnt (void)
{
    return sensor_6ch_fail_cnt;
}

//-------------------------------------------------------------------------------------------------------------------
// 读六路学习阈值（卡A10 亚灯插值用）：模块板载 KEY 学习后阈值存模块内，开机读一次做归一化基准
// 返回 1=成功；失败（NACK/未学习全 0）返回 0，调用方回退数字加权版
//-------------------------------------------------------------------------------------------------------------------
uint8 sensor_6ch_read_thr (uint16 thr[6])
{
    uint8 raw[SENSOR_6CH_CH_NUM * 2];
    uint8 i = 0;
    uint8 all_zero = 1;

    if(NULL == thr)
    {
        return 0;
    }
    if(!sensor_6ch_read_regs(SENSOR_6CH_REG_THR, raw, SENSOR_6CH_CH_NUM * 2))
    {
        return 0;
    }
    for(i = 0; i < SENSOR_6CH_CH_NUM; i++)
    {
        thr[i] = (uint16)raw[2 * i] | ((uint16)raw[2 * i + 1] << 8);
        if(0 != thr[i])
        {
            all_zero = 0;
        }
    }
    return !all_zero;                           // 未学习过则全 0，按失败处理（数字版兜底）
}

//-------------------------------------------------------------------------------------------------------------------
// I2C 总线扫描（排障）：逐个地址发地址写，ACK 即打印；串口 k 命令触发
//-------------------------------------------------------------------------------------------------------------------
void sensor_6ch_i2c_scan (void)
{
    uint8 addr = 0;
    uint8 found = 0;
    printf("i2c scan start (SCL=B8 SDA=B26)...\r\n");
    for(addr = 0x08; addr < 0x78; addr++)
    {
        soft_iic_start(&sensor_6ch_iic);
        if(soft_iic_send_data(&sensor_6ch_iic, addr << 1))          // send_data 返回 1=ACK（库实现，别写反）
        {
            printf("  ACK at 0x%02X\r\n", addr);
            found++;
        }
        soft_iic_stop(&sensor_6ch_iic);
    }
    printf("i2c scan done, %d device(s)\r\n", found);
}
