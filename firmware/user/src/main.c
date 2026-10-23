/*********************************************************************************************************************
* 循迹 PID 联调固件（2026-07-28，卡A04，A06 后恢复版）
* 架构：七路灰度 pos（-3~+3，左负右正）→ 循迹外环 PD → 差速修正 → 速度内环 PI（双轮）→ motor_set
* 编码器：状态机四倍频软解码（A02 补充卡方案，抗耦合毛刺），左 A=B10 B=B9，右 A=B11 B=B7
*         每圈 1560 脉冲；ENC_x_SIGN 定案双 +1（A06 实测改案：R 通道前进即为正，不取负）
* 定时器：PIT_G0 10ms 编码器采样；PIT_G7 10ms 控制环（自检完成后才启动）
* 调参：UART0 收 `x=123\n`（p=SPD_KP i=SPD_KI d=LINE_KP e=LINE_KD b=BASE_RPM），立即生效并回显
* LOST 策略（A04 补充，方形道必需）：丢线保持——LOST 时按丢线瞬间 pos 分档：弯道(|pos|>=1.5)冻结 duty/积分续行 ≤LOST_HOLD_MS(450)，开出赛道/拎走立即停车，
*           超时仍丢线才停车清积分；打印 LOST 带已保持毫秒数
* 双参数档（A04 补充2，方/弧双赛道）：增益两套（0=方形 1=弧线），串口 m 切档、p/i/d/e 改当前档；
*           弯道自动减速 target = BASE - SLOW_KP*|pos|（下限 BASE 的 40%），SLOW_KP 串口 s
* pos EMA（A04 补充3，治台阶舞）：pos_f = pos_f*6/10 + pos*4/10（POS_EMA_NEW 可调），
*           控制环 P/D/SLOW_KP/打印全用 pos_f；LOST 判定走原始 Σv 不滤波
* 非线性转向（A04 补充5，直道软弯道狠）：diff = KP*pos/100 + KQ*pos*|pos|/10000 + KD*dpos/100，
*           KQ 串口 q（双档），diff 满舵限幅 ±DIFF_MAX(80)rpm
* 堵转保护（A04 补充6）：|duty|≥3000 且 |rpm|≤5 持续 1000ms → 该侧断电刹车清积分，每秒打印一次；
*           LOST 后找回线或 b=0 人工复位自动解除
* 目标速度斜坡（A04 补充7，治一卡一卡）：公共基准 base 按 ±RAMP_STEP(rpm/10ms，串口 r) 爬/滑，
*           转向 diff 不平滑保持即时；上电从 0 柔启动；LOST 停车期间 base_smooth 同步滑到 0
* A14 钉低电平：该脚与蜂鸣器同脚，禁止翻转心跳（全程保持低）
* 传感器源（卡A09）：LINE_SRC_6CH=1 幻尔六路 I2C 模块（0x5C，SCL=B8 SDA=B26，权重 ±5 均值 ×30
*           → ±150，与现环门槛 80/弯丢线判 150 同量纲）；=0 原七路 ADC（留档不删）
*           卡A10 B案：LINE_6CH_INTERP=1 亚灯插值（黑响应 max(0,analog-学习阈值) 加权，pos 连续
*           治台阶脉冲感；阈值开机读模块 0x18，未学习/读取失败自动回退数字加权版）
* 开机流程：白底校准 3s（带病 offset 直接 CAL FAIL 停机）→ 3s 准备 → 电机极性自检（各 +3000 转 1s
*           打印编码器符号）→ 进控制环。增益全 0 = 安全态（duty 恒 0，刹车）
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor.h"
#include "servo.h"
#include "sensor_6ch.h"
#include "comm.h"                               // K230 视觉下行帧
#include "oled_i2c.h"                           // SSD1306 OLED 计时屏（B6/B5，卡A11）

// 视觉 PID 纯净测试模式：=1 只跑 servo+视觉，不初始化电机/编码器/循迹；=0 完整整车
#define VISION_TEST_MODE    ( 0 )

#define LED_PIN         ( A14 )                 // 与蜂鸣器同脚：只初始化钉低，禁止翻转

// ====== 调参区（起步参数已给，串口 p/i/d/e/b/m/s 现场改；双参数档：0=方形档 1=弧线档）======
#define GAIN_SET_NUM    ( 2 )
static volatile int32 SPD_KP[GAIN_SET_NUM]  = {20, 12};   // 速度内环 P（台架定案；串口 p，改当前档）：m1_v1 定案 12（低速稳球档）
static volatile int32 SPD_KI[GAIN_SET_NUM]  = {1, 1};     // 速度内环 I（串口 i；已加条件积分+限幅100，台架验证零振荡）
static volatile int32 SPD_KD[GAIN_SET_NUM]  = {25, 25};    // 速度内环 D（串口 v，measurement 微分）：m=0=25 压高速超调（m0_v3 定案）/ m1_v1=25 抑低速过弯抖动
static volatile int32 LINE_KP[GAIN_SET_NUM] = {15, 10};   // 循迹外环 P（串口 d）：m0_v6.1 定案 15（v6=10，带重救弯 authority 不够）
static volatile int32 LINE_KD[GAIN_SET_NUM] = {20, 45};    // 循迹外环 D（串口 e，改当前档）：m=0=20 压入弯尖峰 / m1=45
                                                           // 【2026-10-17 写进初值】m1 由 20 改 45：149rpm 带球的所有实测
                                                           //   （题5 连跑45s不丢线、题6 四目标位）都是 e=45 跑的，142/20 那组从未带球验证
                                                           //   代价要知道：直道 pos σ 由 0.134 变 0.206（更摆）—— 是拿摆动换高速不丢线
                                                           // 【2026-10-15 定案】7 路数字循迹实跑：e=20 比 12 稳
static volatile int32 LINE_KQ[GAIN_SET_NUM] = {6, 0};     // 循迹外环二次项（串口 q，改当前档）：m=0=6 中段早转 / m=1=0 恒速纯P
#define DIFF_MAX        ( 120 )                 // 满舵限幅 rpm（10s 冲刺：b>150 高速转向差速需求大，80 不够）
static volatile int32 BASE_RPM[GAIN_SET_NUM] = {220, 149};  // 基准速度（串口 b，改当前档）：m0_vFinal=220 / m1=149
                                                           // 【2026-10-17 写进初值】m1 由 142 改 149（0.541 m/s，单圈≈11.35s）：
                                                           //   题5/题6 全部实测都在 149 跑的；初值若留 142，一断电就回到未验证的档
                                                           // 【2026-10-15 定案】7路数字循迹：105 稳但不快，142 圈速 ≈14s 且不丢线
static volatile int32 SLOW_KP[GAIN_SET_NUM]  = {0, 0};   // 弯道比例预减速（串口 s，改当前档）：双档均 0=恒速流不预减速（m0_v2 起）
static volatile int32 RAMP_STEP = 3;                     // 目标速度斜坡步长 rpm/10ms（串口 r）：【2026-10-15 定案】3（起步更柔，6 太冲）
static volatile int32 FF_RAMP_PERIOD = 4;                // 弯道前馈缓坡周期（串口 h，单位 10ms/步）：【2026-10-15 定案】4（n=15 时 600ms 爬满）
static volatile uint8 gain_set = 1;                     // 当前参数档（串口 m=0/1）：默认 m1 球稳档（比赛模式按键整档切换）
static volatile int32 CORNER_RPM[GAIN_SET_NUM] = {220, 149};  // 弯道安全速（串口 c，改当前档）：m0_vFinal=220 / m1=149 与直道同速
                                                              // 【2026-10-17 写进初值】m1 随 b 同步改 149（c=b 的定案不变）
                                                              // 【2026-10-15 定案】c=b=142：弯道识别只切前馈，不再切速度（治变速顿挫）
static volatile uint8 in_corner = 0;                    // 弯道标志：1=弯中（目标 CORNER_RPM），0=直线
static volatile uint16 corner_ms = 0, exit_ms = 0;      // 进/出弯确认计时（迟滞）
#define LOST_HOLD_MS    ( 300 )                 // 全白（丢线/出线）保持时长：冻结 duty/积分续行，超时仍全白即停车
                                                // 【2026-10-17 100->300】100ms 在 149rpm 下只有 5.4cm，
                                                //   出弯甩出去一下就 "LOST STOP" 硬停，实测中途反复顿挫（30s 抓到 1 次，
                                                //   起步猛冲 bug 未修时更频繁）。300ms = 跑 16cm 才停，容错够。
                                                //   这正是 2026-10-15 注释里预留的那句"再往上调（200/300）"。
                                                // 【2026-10-15 小土定案】先试 100ms（反应最快）：
                                                //   出弯/出线后立即截停，避免冲出赛道布造成干扰。
                                                //   若过弯途中偶发全白被误停，再往上调（200/300）。
                                                //   历史值：450（仅弯道用）→ 1000（统一）→ 100
#define LOST_CORNER_POS ( 150 )                 // 保留：丢线瞬间 pos 超此值视为弯道丢线（日志/调试参考）
#define DARK_STOP_MS    ( 500 )                 // 冲出赛道保护：武装后，全黑持续此毫秒数即截停
                                                //   真起停胶带压过 <0.3s，500ms 不会误判；
                                                //   而深色地面是持续全黑，0.5s 内必然触发。
#define START_FF_K      ( 300 )                  // 起步前馈系数：delta_ff = K * (rpm/10ms)
                                                // 【2026-10-16 新增】解决"起步前冲"：
                                                //   斜坡期间车加速度是【已知量】(目标速度/斜坡时长)，不需要编码器 -> 无噪声。
                                                //   球受惯性扰动 d = (5/7)*a_car*2600 px/s²，需要倾角 delta = d/K_plant
                                                //   a_car = (rpm每10ms)*0.3629 m/s²；实测 K_plant≈1.2 px/s² per µs
                                                //   代入得 delta_ff ≈ 560*(rpm每10ms)
                                                //   实例：循迹 147rpm/4000ms -> d=248px/s² -> delta_ff=206µs
                                                //         题4   70rpm/1500ms -> d=314px/s² -> delta_ff=262µs
                                                //   只在斜坡期间生效，巡航/弯道自动归零，不影响稳态
static volatile int32 g_start_ff = 0;            // 由 pit_ctrl_callback 每 10ms 算好，vision_hold_isr 直接扣
static volatile int32 g_stop_ff  = 0;            // 【2026-10-17】软停车减速度前馈（负值 = 与起步前馈反向），
                                                 //   0 = 不在减速段。只在最后 STOP_RAMP_MS 内非零。
#define START_DIFF_RAMP_MS  ( 600 )              // 转向 diff 起步柔化时长 ms（与速度斜坡解耦，见下）
#define START_RAMP_MS   ( 4000 )                // 起步线性加速时长 ms
                                                // 【2026-10-16 定案】800 -> 2000 -> 4000，依据是环路能力算出来的：
                                                //   实测被控对象增益 K=2.22 px/s² per µs 舵机脉宽；
                                                //   环路最大回复加速度 = K*(KP*|err|/1000 + KI*I_max/1000)
                                                //                       = 2.22*(1.4*|err|) + 2.22*75 = 3.108|err| + 166.5
                                                //   稳态误差 = (d - 166.5)/3.108，d = (5/7)*a_car*2600
                                                //     ramp 2000ms: d=478 -> err 100px（实测 -100，模型吻合）
                                                //     ramp 4000ms: d=239 -> err  23px = 0.9cm  ← 达标
                                                //   代价：整圈慢约 1s（题5 整圈限 30s，余量充足）
                                                //   ⚠️ ACC_FF 加速度前馈这条路已否决：acc_long_lpf 的编码器量化噪声
                                                //      ±4 而斜坡真实信号只有 1.42（SNR<1），乘以 150 倍后变成 ±300us
                                                //      的随机舵机指令（DELTA_NEAR 才 180），巡航段球慢速大摆实测 ±115px。
#define SLOW_MIN_PCT    ( 40 )                  // （已弃用，比例减速被弯道识别替代）
#define CORNER_POS_TH   ( 50 )                 // 进弯阈值 |pos|x100>=0.5
                                                // 【2026-10-15】80 -> 50：圆赛道上车要持续差速，
                                                //   P 控制器产生 diff 必须靠 pos 静差（diff=KP*pos/100），
                                                //   等 pos 到 80 才降速/给前馈已经太晚，车已贴近量程边缘。
                                                //   提前到 50 让弯道降速与前馈更早介入。
#define EXIT_POS_TH     ( 25 )                  // 出弯阈值 |pos|x100<=0.25（迟滞，避免阈值附近抖动）
#define CORNER_MS       ( 30 )                  // 进弯确认时长 ms
#define EXIT_MS         ( 100 )                 // 出弯确认时长 ms
// ============================================================================

// ---------- 视觉→舵机闭环（卡 B07：静止调球位）----------
#define VISION_AGE_MAX      ( 300 )             // 视觉帧超 300ms 不用，防掉线乱摆
static volatile int32 SERVO_KP = -1400;         // 视觉 PID P 增益（2026-08-01 新杆定案：-1600 震荡/-1200 偏慢，-1400 最快稳定档），串口 a 调
static volatile int32 SERVO_KI = -25;           // 条件慢积分（|err|<60px 才积，上限3000→±40us 磨死区；负号同 KP 方向），串口 j 调
                                                // 【2026-10-16 定案】-25：纯 P(j=0) 有 ~22px 静摩擦死区推不到中心，
                                                //   j=-25 后实测稳态 err=+0.0、σ=0.0，题3 外的高增益档另在 servo_gain_preset 里切
static volatile int32 SERVO_KD = -70000;        // 视觉 PID D 增益，串口 k 调
                                                // 【2026-10-16 定案】-18000 -> -70000：基线其余不动，只改这一项。
                                                //   静态阶跃实测（步长 ±120px）：
                                                //     KD=-18000: 回中 1.97/5.82/4.39/3.15s，过冲 -12/+111/-84/+85px
                                                //     KD=-70000: 回中 0.85/0.93/0.73/0.82s，过冲 -2/+6/-4/+2px，残差 0~3px
                                                //   快 2.5 倍且几乎无过冲，保持 σ 仍为 0。
                                                //   原因：KD 是帧级微分（Δe=err-err_last 按帧算），
                                                //   标定 -18000 时帧率 ~33fps，现在 46fps -> 等效阻尼掉 1.4 倍；
                                                //   再叠加机械改动（齿轮重装/高度调整），阻尼明显不足。
                                                // 【2026-10-16 定案】-18000 -> -65000：跑车实测同一次跑动内扫参
                                                //   （每档 20s 换参数，条件一致）：-18000 最大偏离 79px / -35000 102px /
                                                //   -50000 61px / -65000 26px(1.00cm 达标)。
                                                //   原因：KD 是帧级微分，帧率 33->46fps 后等效阻尼掉 1.4 倍，
                                                //   而环路固有阻尼比只有 ζ=0.246（ω=1.76rad/s），本来就欠阻尼。
static volatile int32 SERVO_MID_VISION = 1726;  // 视觉闭环舵机中位（落地精标：钉±1px；换场地/掰杆后须串口 o 重标），串口 o 标定
                                                // 【2026-10-15 重标】1726 -> 1660：传动齿轮重装后机械零点左移，1660 才是水平位
                                                // 【2026-10-16 复测】漂移过零点 ≈1646us，与 1660 差 14us 在容差内
static volatile int32 VISION_TARGET_ERR = 0;    // 目标误差（像素或 cm），串口 l 调；默认 0=目标就是视觉中点
static volatile int32 STICTION_KICK = 250;    // 静摩擦破粘脉冲（us），串口 Z 调（可在跑动中扫）
                                                // 【2026-10-16 重新启用】实测球回中时在 +26px(1cm) 处有稳定卡点，
                                                //   该点 P(36µs)+积分满幅(75µs)=111µs < 脱困阈值 ~300µs。
                                                //   250µs 叠加后约 361µs，够破粘；脉冲只有 150ms，不会把球甩飞。
                                                //   ⚠️ 必须与 servo_gain_preset(0) 里的值保持一致
                                                // 【2026-10-16 重启】0 -> 400：球卡住时 P(36us)+积分满幅(75us)=111us
                                                //   远低于实测脱困阈值 300us，导致球回中到 +26px(1cm) 就永远停住。
                                                //   vision_hold_isr 里只在 |err|>15px 才触发（稳定保持 |err|<=5，不误触），
                                                //   且脉冲在限幅【之外】叠加，否则会被 DELTA_NEAR 吃掉。
                                                //   ⚠️ 注意：这个初值必须和 servo_gain_preset(0) 里的一致，
                                                //   否则上电/烧录后没敲过 x=1 或 3=0 时用的是错的初值（踩过此坑）。
static volatile int32 DELTA_LIMIT   = 250;      // 视觉 PID 输出限幅（us，远目标段；新杆定案 250 远端拉球 authority），串口 G 调
static volatile int32 DELTA_NEAR    = 180;      // 视觉 PID 近目标限幅（us，|err|<60 段，稳停防过冲），串口 H 调
static volatile int32 DITHER_PX     = 0;        // 抗静摩擦颤振幅度（px，中点锚定时目标方波，0=关闭），串口 D 调
static volatile int32 DITHER_MS     = 500;      // 颤振半周期 ms（目标 ±DITHER_PX 切换间隔），串口 U 调
static volatile int32 KP_NEAR       = -1400;    // 近段 KP（定案=远段同值，纯净单增益；旧 -400 弱近段会粘 ±2cm），串口 N 调
static volatile int32 KP_NEG_SCALE  = 100;      // 负方向 KP 放大倍数 %（定案 100=对称），串口 E 调
static volatile int32 ACC_FF        = 0;        // 纵向加速度前馈（毫 us per rpm差分；车加速球后甩舵机预倾，0=关闭，串口 A 调）
// ---------- 题3（2026-08-01 重写版：五次多项式轨迹 + PD 跟踪，按题纸原文从零设计）----------
// 题意：小车静止，球从中心 O -> +5cm（到达后折返）-> -5cm 稳定；总时长 ≤5s，±5cm 处误差 ≤1cm
// 物理：凹槽球 ≈ 双积分器 x¨ = β·θ（θ=摆杆倾角，舵机脉宽→倾角近似线性）
// 方法：rest-to-rest 五次多项式参考轨迹 s(τ)=10τ³-15τ⁴+6τ⁵（两端速度/加速度=0，到位末速≈0，
//       天然满足"到达后折返"与"稳定"），PD 跟踪（D 项追参考速度）+ 参考加速度前馈（默认 0）
// 标定：球放 O 发 3=7 记零点 ERR_O；球放 +5cm 发 3=8 记比例 PX_CM → 两侧目标像素自动算出
static volatile int32 T3_KP        = 1200;   // 跟踪 P（毫 us/px：delta_us = (KP·e + KD·ev)/1000），串口 L 调
static volatile int32 T3_KD        = 24000;  // 跟踪 D（毫 us per px/frame；定案 24000：过中线即重刹，段2 摆回阻尼），串口 R 调
static volatile int32 T3_KFF       = 300;    // 参考加速度前馈（定案 300≈峰值 100µs；400/800/1200 均饱和甩球），串口 Y 调
static volatile int32 T3_KI        = 20;     // 条件积分（毫 us per px·拍，|e|<T3_KI_BAND 才积；磨过静摩擦到位），串口 I 调
static volatile int32 T3_VEL_NEW   = 5;      // 球速 LPF 新值权重 ×0.1（1~10，越大越灵），串口 M 调
static volatile int32 T3_T1_MS     = 1000;   // [已退役] 段1闭环版轨迹时长（段1 已改开环 bang-bang），串口 P 已改派 T3_VMAX
static volatile int32 T3_T2_MS     = 1800;   // 段2 +5->-5cm 轨迹时长 ms（定案 1800：1600 到达过冲 -201 锁太慢），串口 Q 调
static volatile int32 T3_DELTA_MAX = 450;    // 输出限幅 us（绕中位 ±；定案 450，齿条物理限位 1300/2500 内），串口 W 调
static volatile int32 T3_ERR_O     = 0;      // O 点视觉 err px（3=7 标定写入）
static volatile int32 T3_PX_CM     = 26;     // 像素比例 px/cm（定案v3 硬定 26：+5cm 在屏 x=450，(450-320)/5=26；串口 V 直设）
static volatile int32 T3_TGT_P5    = 130;    // +5cm 目标 err px（定案v3 硬定 130：屏 x=450，450-320=+130；串口 X 直设覆盖）
static volatile int32 T3_TGT_N5    = -130;   // -5cm 目标 err px（定案v3 硬定 -130：屏 x=190，190-320=-130，实测停 -5.00cm），串口 F 直设覆盖
// 段1开环 bang-bang（小土定案 2026-08-01：固定倾角固定时长，重复性优先于闭环跟踪；段2 仍闭环）
static volatile int32 T3_OL_ACC    = -260;   // 段1加速倾角增量 us（定案v3 硬定 -260：当前杆冲程比原 -250 略大一点），串口 O 调
static volatile int32 T3_OL_BRK    = 250;    // 段1刹车倾角增量 us（正=反向制动），串口 B 调
static volatile int32 T3_OL_TA_MS  = 400;    // 加速时长 ms，串口 S 调
static volatile int32 T3_OL_TB_MS  = 300;    // 刹车时长 ms，串口 T 调
static volatile int32 T3_VMAX      = 10;     // 段2限速 px/frame（超速即满刹：过中线必刹，治 -5 刹不住），串口 P 调

//-------------------------------------------------------------------------------------------------------------------
// 题4：匀速行驶球稳（题纸原文：小车置A点、球置中心O，按键启动后沿黑线顺时针行驶并通过B位置，
//       AB间行驶时间 ≤8s，过程中钢球须稳定在摆杆中心点附近，误差绝对值 ≤1cm。本题 20 分）
//   换算依据：题2 实测 m0 档 BASE_RPM=220 跑完一圈 6.142m 用 7.7s → 0.798m/s，即 1rpm ≈ 0.00363m/s
//     35rpm → 0.127m/s（1.5m 需 11.8s，**超时**）
//     60rpm → 0.218m/s（含1.5s斜坡约 7.64s，余量仅 4.5%，太薄）
//     70rpm → 0.254m/s（约 6.66s，余量 17%）← 采用，且仍只有题2车速的 1/3，球好压
//   来源：topic/三题 573e187「题3定案v3」带入；与题3共用舵机，t3_run=1 时让位给题3轨迹控制器
//   启动：串口 `4=1`（带视觉校验，无球帧拒绝）/ `4=0` 停车；按键面板 mode 4 仍为占位（未上台验证前不放开）
//-------------------------------------------------------------------------------------------------------------------
static volatile uint8  t4_run = 0;           // 0=空闲 1=题4运行中
static volatile uint32 t4_t0 = 0;            // 题4起跑时刻（1.5s 匀加速计时用）
static volatile int32  t4_target_rpm = 70;   // 目标匀速 rpm（三题原版声明了却写死60没用上；此处接上并按题纸 AB≤8s 定为70）

static volatile uint8  t3_run = 0;           // 0=空闲（锚定 PID 接管舵机）1=题3运行中（本模块独占舵机）
static volatile uint8  t3_state = 0;         // 1=O->+5  2=+5->-5  3=-5保持
static volatile uint32 t3_t0 = 0;            // 本次起跑时刻（comm 1ms 时基）
static volatile uint32 t3_seg0 = 0;          // 当前段起始时刻
static volatile int32  t3_ref0 = 0;          // 当前段参考起点 px（=切段瞬间实测位，轨迹连续）
static volatile int32  t3_ref = 0;           // 当前参考位置 px（打印用）
static volatile int32  t3_x = 0;             // 最新球位 err px（打印用）
static volatile int32  t3_vel = 0;           // 球速估计 px/frame（打印用）
static volatile uint32 t3_inband0 = 0;       // 进到位带起始时刻，0=不在带内
static volatile int32  t3_extreme = 0;       // 段1 极端位置 err px（评审判 +5cm 到达误差用）
static volatile uint32 t3_reach_ms = 0;      // 首次进 +5 带时刻（相对 t0）
#define T3_BAND_X100    ( 60 )               // 到位带 ±0.60cm（×100 cm，随 PX_CM 折算 px）
#define T3_INBAND1_MS   ( 200 )              // +5 侧：带内持续 200ms 认定"到达"（题意"到达后折返"）
#define T3_INBAND2_MS   ( 300 )              // -5 侧：带内持续 300ms 认定"稳定"
#define T3_VEL_STILL    ( 3 )                // 到位低速门限 px/frame（≈静止）
#define T3_SEG1_TIMEOUT ( 800 )              // 段1 轨迹走完后到位宽限 ms（超时强制折返）
#define T3_SEG2_TIMEOUT ( 1000 )             // 段2 轨迹走完后到位宽限 ms（超时进保持）
#define T3_LOST_ABORT   ( 1500 )             // 视觉连丢此时长 ms： abort 回中位
#define T3_KI_BAND      ( 60 )               // 条件积分：|e|<60px 才积（远段不积，防冲刺段积分饱和）
#define T3_KI_MAX       ( 3000 )             // 积分限幅 px·拍（满幅贡献 KI·3000/1000=60us@KI20，够破粘不致极限环）
#define T3_VEL_CLAMP    ( 80 )               // 球速估计钳位 px/frame（≈2m/s 物理上限，防视觉跳帧污染 KD）
#define SERVO_KI_GATE       ( 300 )             // 【2026-10-16】积分门限：|err| 超过此值不积（原为硬编码 60，太窄导致远端回不了中）
#define SERVO_KI_MAX        ( 3000 )            // 条件慢积分限幅（锚定：j=20 满幅 60us 磨死区；题3 复用同逻辑）
#define SERVO_KI_MIN        ( -3000 )
// ---------- 编码器 ----------
#define ENC_L_A         ( B10 )
#define ENC_L_B         ( B9  )
#define ENC_R_A         ( B11 )
#define ENC_R_B         ( B7  )
#define ENC_PPR         ( 1560 )                // 每圈脉冲：13 x 30 x 4 倍频
#define ENC_L_SIGN      ( +1 )                  // 定案（A06 实测）：前进计正
#define ENC_R_SIGN      ( +1 )                  // 定案（A06 实测改案）：前进计正，不取负

// ---------- 循迹传感器源 ----------
// 2026-10-15 换传感器：拟改用「七路不带 MCU 的循迹传感器」（正负 + 7 根信号线）
//   三条通路都留着，改这两行就能切换：
//     LINE_SRC_6CH=1                      → 幻尔六路 I2C（旧，已验证）
//     LINE_SRC_6CH=0 + IR7_DIGITAL=1      → 七路数字 GPIO（新）★当前
//     LINE_SRC_6CH=0 + IR7_DIGITAL=0      → 旧七路模拟 ADC（留档）
#define LINE_SRC_6CH    ( 0 )                   // 见上表
#define IR7_DIGITAL     ( 1 )                   // 七路内选：1=数字 GPIO，0=模拟 ADC
#define SENSOR_6CH_POS_SIGN ( +1 )              // 六路 pos 方向：线在右应为正，反了改 -1
#define SENSOR_6CH_SCALE    ( 30 )              // 数字权重均值(-5..+5)×30 → ±150（门槛 80 / 弯丢线判 150 同量纲）
#define LINE_6CH_INTERP   ( 1 )                 // 卡A10 B案：1=模拟量亚灯插值（pos 连续，治台阶脉冲感），0=数字状态加权
#define S6_DSUM_MIN       ( 400 )               // 插值版黑响应总量下限（线性 d 量纲）：低于此值按丢线帧
#define S6_D_DEADZONE     ( 300 )               // 黑响应死区：analog-thr 再扣 300，压白底串扰（ch2 白底残留 ~300）
#define LINE_POS_BIAS     ( 50 )                // 跟踪零点偏置（×100 定点）
                                               // 【2026-10-15 小土定案：中点挪到 ch3】
                                               //   （ch3 权重 -1 => pos_raw = -50；要让它成为零点需 BIAS = +50）
                                               //   ch3 左边只有 ch2/ch1，右边有 ch4~ch7 四级缓冲，
                                               //   顺时针右转时线往右跑，偏置把行程全铺在右侧。
                                               // 历史值：+50（幻尔6路时代）→ 0（7路居中 ch4）→ -50（零点挪 ch5）→ +50（回 ch3）
#define S6_TAPE6_RUN      ( 3 )                 // ch6+≥3路 连续触发帧数：≥3 帧(30ms) 判终点胶带/冲出，高速压胶带更易触发

// ---------- 七路灰度（LINE_SRC_6CH=0 时启用）----------
#define IR_CH_NUM       ( 7 )
#define IR_CAL_MS       ( 3000 )
#define IR_CAL_STEP_MS  ( 50 )
#define IR_CAL_MAX      ( 1000 )                // 白底 offset 上限，超限 = 校准时压线，带病不上岗
#define IR_FILTER_CNT   ( 5 )
#define IR_LOST_SUM     ( 300 )                 // Σv 低于此值判丢线
#define LOST_DEBOUNCE_CNT ( 10 )                // 丢线防抖：连续 10 帧都丢才认（单帧 glitch 无视，≈10~30ms）
#define IR_POS_SIGN     ( +1 )                  // pos 物理左负右正；装法反了改 -1
static volatile int32 POS_EMA_NEW[GAIN_SET_NUM] = {7, 7};  // pos EMA 新值权重 ×0.1（串口 f，改当前档）：m=0=7 快反应压入弯尖峰 / m1_v5=7 保留响应不拖泥
static volatile int32 DIFF_LPF_NUM[GAIN_SET_NUM] = {9, 8}; // 转向 diff 低通新值权重 ×0.1（串口 g，改当前档）：m=0=9 近乎即时 / m1_v5=8 比最大滞后稍灵，过弯更跟手
static volatile int32 FF_CORNER_DIFF[GAIN_SET_NUM] = {25, 15};  // 弯道前馈差速 rpm（串口 n，改当前档）：m0_v6 定案 25 / m1=15
                                                                // 【2026-10-15 定案】7路数字循迹实测：n<14 必掉线（n=10 → 4 秒丢线），14~15 才跑得稳

// ---------- 控制环 ----------
#define PIT_ENC_TIMER   ( PIT_TIM_G0 )
#define PIT_CTRL_TIMER  ( PIT_TIM_G7 )
#define PIT_STOP_TIMER  ( PIT_TIM_G6 )          // 外挂定时停车中断：独立计时，不占用控制环
#define PIT_PERIOD_MS   ( 10 )

#define DUTY_RUN_MAX    ( 8000 )                // 运行 duty 限幅（10s 冲刺：6000 卡在 rpm150，抬到 8000 探 rpm200；满速 350 物理顶）
#define DUTY_SLEW       ( 200 )                 // 每 10ms duty 最大变化量（防猛冲）
#define INTEG_MAX       ( 100 )                 // 积分限幅（台架定案：500 等效摆幅过大打出极限环，100 翻不了船）
#define INTEG_ERR_GATE  ( 60 )                  // 条件积分门限：|err|>60rpm 时积分冻结不增（防大误差段疯长）
                                                // 原为 15：实测目标120 稳态只到 80，err=40 超过门限导致积分
                                                // 永不累积，速度环只剩 P 项，33% 稳态误差消不掉。
                                                // 抬到 60 覆盖该工况；仍保留"特大误差冻结"防冲。

#define STALL_MS        ( 1000 )                // 堵转判定时长：条件持续这么久才锁定
#define STALL_DUTY      ( 3000 )                // 堵转条件：|duty| ≥ 此值
#define STALL_RPM       ( 5 )                   //           且 |rpm| ≤ 此值（给了劲却不转 = 卡死）

#define SELFCHK_DUTY    ( 3000 )

#if !LINE_SRC_6CH                               // 七路版：六路启用时不编，防 unused 警告

#if IR7_DIGITAL
//-------------------------------------------------------------------------------------------------------------------
// 七路【数字量】GPIO 循迹（2026-10-15 新增，接主板灰度传感器口 P14）
//   引脚取官方端口表的【后 7 个】，第 1 脚 B18 被 K230 RX 占用故跳过
//   官方原文（拓展板使用说明）：灰度传感器模块从左往右依次为
//       ADC-B18 / ADC-B17 / ADC-A17 / ADC-A16 / ADC-B20 / ADC-B25 / ADC-A24 / ADC-A25
//   本表 通道1..7 对应 B17 A17 A16 B20 B25 A24 A25（即端口的第 2~8 脚）
//   ⚠️ 权重中间那个 0 是"中心通道"：7 路奇数，正好有一路压在线上正中
//-------------------------------------------------------------------------------------------------------------------
static const gpio_pin_enum ir7_pin[IR_CH_NUM] = {
    B17, A17, A16, B20, B25, A24, A25,
};
static const int8 ir7_w[IR_CH_NUM] = {-3, -2, -1, 0, 1, 2, 3};  // 左负右正
#define IR7_ACTIVE_LOW  ( 0 )                   // 压黑线时该路输出【高】电平，故取 0
                                                // 荣洋电子 rydz 七路模块实测（2026-10-15，raw 实测铁证）：
                                                //   黑底/黑线/抬高(无反射) -> 模块 LED 【不亮】, raw=1111111 (引脚全高)
                                                //   白底                  -> 模块 LED 【亮】  , raw=0000000 (引脚全低)
                                                // 判据: 要让"黑线"算作压线, 需 on=(0!=lv) => IR7_ACTIVE_LOW=0
                                                // （此前在 1/0 之间反复改错，以此 raw 实测为准，不再变更）
#define IR7_POS_SIGN    ( +1 )                  // pos 物理左负右正；七根线接反了改 -1
#define IR7_SCALE       ( 50 )                  // 均值(-3..+3)×50 → ±150（与六路 SENSOR_6CH_SCALE 同量纲）
#define IR7_TAPE_N      ( 5 )                   // 终点胶带判定：7 路里 ≥5 路同黑（对应六路的 n≥4）
#else
static const adc_pin_enum ir_adc[IR_CH_NUM] = {
    ADC1_CH5_B18, ADC1_CH4_B17, ADC1_CH2_A17, ADC1_CH1_A16,
    ADC0_CH6_B20, ADC0_CH4_B25, ADC0_CH3_A24,
};
#endif

static const int8 ir_w[IR_CH_NUM] = {-3, -2, -1, 0, 1, 2, 3};
static uint16 ir_offset[IR_CH_NUM] = {0};
static uint8  ir7_bits = 0;                     // 七路数字：bit0~6 = 通道1~7 的黑线状态（调试打印用）
#endif

// delta[旧状态][新状态]，合法正交步进 ±1，非法跳变 = 0 丢弃；状态 = (A<<1)|B
static const int8 enc_delta[4][4] = {
    {  0,  +1,  -1,  0 },
    { -1,   0,   0, +1 },
    { +1,   0,   0, -1 },
    {  0,  -1,  +1,  0 },
};
static volatile uint8 enc_l_state = 0;
static volatile uint8 enc_r_state = 0;

volatile int32 enc_l_raw  = 0;
volatile int32 enc_r_raw  = 0;
static volatile int32 acc_long_lpf = 0;         // 纵向加速度低通值（rpm和差分/10ms；题4/5/6 球稳前馈）
static volatile int32 rpm_sum_now = 0;          // 两轮 rpm 之和（遥测打印用）
volatile int32 enc_l_10ms = 0;
volatile int32 enc_r_10ms = 0;
volatile int32 enc_l_total = 0;
volatile int32 enc_r_total = 0;

static volatile int32 line_pos_x100 = 0;        // 加权位置 ×100（-300~+300）
static volatile uint8 line_lost = 1;
static volatile int32 lost_hold_ms = 0;         // LOST 已保持毫秒数，打印用
static uint8 lost_stop_reported = 0;            // "全白超时截停"只报一次，避免每帧刷屏
static volatile int32 lost_hold_limit = 0;      // 本次 LOST 的保持上限：弯道=LOST_HOLD_MS，开出赛道/拎走=0（立即停）
static volatile uint8  lost_cnt = 0;            // 连续丢线帧计数（防抖用）
static volatile uint8  line_hold = 0;           // 全黑通过帧标志（压启停胶带/黑块）：期间限速 HOLD_CRUISE_RPM，防盲加速冲线
static void ir_update (void);                    // 前置声明：定义在后面，pit_ctrl_callback(10ms 中断) 要先用它采样
static volatile uint32 loop_tick = 0;           // 主循环计数（保留，供以后诊断用）
static volatile int32 target_l_dbg = 0;         // 诊断：左轮当前目标 rpm
static volatile int32 target_r_dbg = 0;         // 诊断：右轮当前目标 rpm
static volatile int32 diff_dbg = 0;             // 诊断：当前差速项
//-------------------------------------------------------------------------------------------------------------------
// 台架模式（串口 B=1 开 / B=0 关）：只测速度环，两轮同目标 = BASE_RPM
//   用途：架空调速度环 / 换电机验证 / 测电池。架空时传感器必然全黑会触发 line_hold
//         限速 80rpm，导致测不到真实速度环；本模式绕过循迹与 line_hold，直接给纯目标。
//   注意：本模式强制忽略 pos 差速，车不会循迹 —— 只在【架空】时使用。
//-------------------------------------------------------------------------------------------------------------------
static volatile uint8 g_bench_mode = 0;
static uint8  ir7_raw_bits = 0;                 // 七路原始引脚电平（1=引脚高），仅调试打印用

#define HOLD_CRUISE_RPM ( 80 )                  // 全黑通过帧限速：发车压胶带低速摸过去，出带再恢复加速

#if LINE_SRC_6CH
static volatile uint16 s6_analog_dbg[6] = {0};  // 六路模拟量缓存，500ms 打印排查用
static volatile uint8  s6_state_dbg = 0;        // 六路数字状态缓存
static volatile uint8  s6_n_dbg = 0;            // 触发通道数（丢线判据），排障打印用
static uint16 s6_thr[6] = {0};                  // 六路学习阈值（开机读模块，插值归一化基准）
static uint8  s6_thr_valid = 0;                 // 阈值有效标志：0=未读到/未学习 → 回退数字加权版
#endif

static volatile int32 ctrl_duty_l = 0;          // 控制环当前输出，打印用
static volatile int32 ctrl_duty_r = 0;
static volatile uint8 stall_l_flag = 0;         // 堵转锁定（主循环读，每秒提示；ISR 里不清屏打印）
static volatile uint8 stall_r_flag = 0;

// 串口启动状态机：x=1 触发后进入等待胶带模式，检测到胶带后 2s 倒计时起跑
#define RUN_STATE_IDLE      ( 0 )
#define RUN_STATE_WAIT_TAPE ( 1 )               // 等待起始胶带，最多 10s
#define RUN_STATE_COUNTDOWN ( 2 )               // 已检测到胶带，2s 倒计时
#define RUN_STATE_RUNNING   ( 3 )               // 电机启动，正常跑
static volatile uint8  g_run_state = RUN_STATE_IDLE;
static volatile int32  g_run_timer_ms = 0;
static volatile uint32 g_run_last_ms = 0;       // 上次跑表成绩（PIT/TAPE6 停车时捕获，OLED 显示 FIN 用）
static volatile uint32 g_t3_total_ms = 0;       // 题3 最近一轮总时长（REPORT 时捕获，OLED 显示用）
#define LAP_TIME_MS     ( 14800 )               // 【2026-10-17 小土定】单圈总时长 = 全程 11800 + 减速 3000
                                                //   即：0~11800ms 全速跑，11800ms 起用 3s 斜率线性减速，
                                                //   14800ms 速度正好降到 0 再断电机 -> 最后一刀不再甩球。
                                                //   现场微调直接发 t=<ms>；t=0 关闭（不限时）。
#define STOP_START_MS   ( 11800 )               // 【2026-10-17 小土定】软停车减速【起点】ms —— 只挪时刻，不改减速度
                                                //   到这一刻开始减速，斜率沿用下面 STOP_RAMP_MS（3s 那套）
                                                //   ⚠️ 12000+3000=15000 > 总时长 13500：到 13500 断电机时
                                                //   速度还剩 (3000-1500)/3000*149 ≈ 74rpm，最后仍有半程冲击
#define STOP_RAMP_MS    ( 3000 )                // 减速【斜率时长】ms（也是 3s 版的减速度）：
                                                //   149rpm 在 3000ms 内线性降到 0 = 0.18 m/s²；
                                                //   舵机反向减速度前馈按这个斜率算（START_FF_K 同比例）
static volatile uint32 g_run_stop_ms = LAP_TIME_MS;  // PIT 外挂定时停车（x=1 起跑后从 GO 开始计时）
#define RUN_WAIT_TAPE_MS    ( 10000 )           // 等待胶带超时 10s
#define RUN_COUNTDOWN_MS    ( 2000 )            // 检测到胶带后倒计时 2s

//-------------------------------------------------------------------------------------------------------------------
// 启动位置检查：只有在起始黑胶带上（连续 >=3 路黑 持续多帧）才允许起跑
// 普通赛道黑线宽 1.8cm，通常最多连续 2 路黑；5cm 起始胶带可覆盖连续 3~6 路
// 要求连续 5 帧（50ms）都满足，防止左右平移时的过渡毛刺误判
// 2026-10-15：本函数移出 #if LINE_SRC_6CH（调用点在守卫外），并支持七路数字
//-------------------------------------------------------------------------------------------------------------------
#define START_TAPE_MIN_CONSEC   ( 3 )           // 最少连续黑通道数
#define START_TAPE_MIN_FRAMES   ( 5 )           // 连续满足帧数（10ms/帧）

static uint8 is_on_start_tape (void)
{
    static uint8 ok_cnt = 0;
    uint8 st, cnt, max_cnt, i;
    if(line_lost) { ok_cnt = 0; return 0; }     // 丢线时清零
#if LINE_SRC_6CH
    st = s6_state_dbg;
    cnt = 0;
    max_cnt = 0;
    for(i = 0; i < 6; i++)
#else
    st = ir7_bits;
    cnt = 0;
    max_cnt = 0;
    for(i = 0; i < IR_CH_NUM; i++)
#endif
    {
        if(st & (1 << i))
        {
            cnt++;
            if(cnt > max_cnt) max_cnt = cnt;
        }
        else
        {
            cnt = 0;
        }
    }
    if(START_TAPE_MIN_CONSEC <= max_cnt)
    {
        if(START_TAPE_MIN_FRAMES <= ++ok_cnt) return 1;
    }
    else
    {
        ok_cnt = 0;
    }
    return 0;
}

static volatile char  cmd_buf[16];              // 串口调参行缓冲
static volatile uint8 cmd_len = 0;
static volatile uint8 cmd_ready = 0;

//-------------------------------------------------------------------------------------------------------------------
// 编码器：任一脚边沿触发后重读 (A,B) 合成新状态，查表累加；毛刺跳变查表得 0 被丢弃
//-------------------------------------------------------------------------------------------------------------------
static void enc_l_update (void)
{
    uint8 new_state = (uint8)((gpio_get_level(ENC_L_A) << 1) | gpio_get_level(ENC_L_B));
    enc_l_raw  += enc_delta[enc_l_state][new_state];
    enc_l_state = new_state;
}

static void enc_r_update (void)
{
    uint8 new_state = (uint8)((gpio_get_level(ENC_R_A) << 1) | gpio_get_level(ENC_R_B));
    enc_r_raw  += enc_delta[enc_r_state][new_state];
    enc_r_state = new_state;
}

void enc_l_a_callback (uint32 event, void *ptr) { enc_l_update(); }
void enc_l_b_callback (uint32 event, void *ptr) { enc_l_update(); }
void enc_r_a_callback (uint32 event, void *ptr) { enc_r_update(); }
void enc_r_b_callback (uint32 event, void *ptr) { enc_r_update(); }

//-------------------------------------------------------------------------------------------------------------------
// PIT_G0 10ms：编码器采样（PRIMASK 临界区取 raw 清零，符号统一到前进为正）
// 无 LED 心跳：A14 与蜂鸣器同脚，翻转会响
//
// 【2026-10-16】视觉链路（解析 + 锚定球控 PID）挂在这里，不挂 PIT_CTRL_TIMER：
//   为什么：run_stop()（串口 x=0）会 pit_disable(PIT_CTRL_TIMER) 做"终极保险"，
//   一旦关掉，挂在那个中断里的 comm_poll() 就再也不执行，而 comm.c 的收帧状态机是
//   「frame_ready 没被取走就丢帧」—— 于是 frame_ready 永远卡在 1、之后每一帧全被丢弃，
//   表现为"K230 明明在发帧、MSPM0 却 age 一直涨"，极容易被误判成 K230 挂了。
//   PIT_ENC_TIMER 全程不关（pit_disable 只关 CTRL/STOP），所以视觉链路必须挂这里。
//-------------------------------------------------------------------------------------------------------------------
static void vision_hold_isr (void);                 // 前置声明（定义在下方的 PIT_CTRL 区之前）

void pit_enc_callback (uint32 event, void *ptr)
{
    uint32 primask = __get_PRIMASK();
    __disable_irq();
    int32 l = enc_l_raw;
    enc_l_raw = 0;
    int32 r = enc_r_raw;
    enc_r_raw = 0;
    __set_PRIMASK(primask);

    l *= ENC_L_SIGN;
    r *= ENC_R_SIGN;

    enc_l_10ms  = l;
    enc_l_total += l;
    enc_r_10ms  = r;
    enc_r_total += r;

    // 纵向加速度估计（题4/5/6 球稳前馈：车加减速球前后甩，舵机预倾补偿）
    // rpm_sum=两轮 rpm 之和（差分即加速度方向量），低通去齿槽/打滑毛刺
    {
        static int32 rpm_sum_prev = 0;
        int32 rpm_sum = (l + r) * 6000 / ENC_PPR;
        int32 acc_raw = rpm_sum - rpm_sum_prev;         // rpm 差分 /10ms
        rpm_sum_prev = rpm_sum;
        // 【2026-10-16】滤波 3/10 -> 1/16（时间常数 ~160ms）：
        //   原来 α=0.3 太轻，acc_raw 的编码器量化噪声约 ±4（rpm_sum 每 ±1 计数 = ±3.9rpm），
        //   而起步斜坡的真实信号只有 1.42 —— SNR<1。喂给 ACC_FF 时噪声被放大 150 倍变成
        //   ±300us 随机舵机指令，实测把巡航段的球从 ±23px 搞成 ±115px。
        //   加重滤波后 ACC_FF 才有可用性；ACC_FF=0 时本项无副作用。
        acc_long_lpf = (acc_long_lpf * 15 + acc_raw) / 16;
        rpm_sum_now = rpm_sum;
    }

    comm_poll();                                    // 解析视觉帧（放所有 return 之前，待机也要跑）
    vision_hold_isr();                              // 题4/5/6 锚定球控 PID（题3 运行时自动让位）
}

//-------------------------------------------------------------------------------------------------------------------
// PIT_G7 10ms：双层控制环
// 外环 PD：diff = (LINE_KP*pos + LINE_KD*dpos)/100（pos 是 ×100 定点）
// 内环增量 PI：duty += SPD_KP*err + SPD_KI*∫err，err = target - 实测 rpm
// duty 钳 0~DUTY_RUN_MAX（循迹不允许反转，也避免 ISR 里触发 motor_set 的换向延时）
//-------------------------------------------------------------------------------------------------------------------
static int32 base_smooth = 0;                   // 目标速度斜坡状态：上电从 0 爬升，天然柔启动（文件作用域，便于待机清零）

static volatile uint8  t3_home = 0;                 // 启动归中：0=无 1=大力扫回 2=锚定稳定
static volatile uint32 t3_home_ms = 0;              // 阶段起始时刻
static volatile uint32 t3_home_inband = 0;          // 进带起始时刻
static volatile uint8  t3_home_next = 0;            // 归中完成后要启动的模式

static volatile uint8  t3_seq = 0;              // 0=空闲 1=去+5cm 2=去-5cm 3=已完成（停在-5cm）
static volatile uint32 t3_seq_t0 = 0;           // 序列起始时刻
static volatile uint32 t3_seq_in = 0;           // 本次进带起始时刻
static volatile int32  t3_seq_t1 = 0;           // O->+5cm 用时 ms（诊断用）
static volatile int32  t3_seq_ext = 0;          // 【2026-10-17】去+5 过程的【最大 err】= 真实过冲峰值（诊断用）
static volatile int32  t3_q_inband_px = 26;     // 【2026-10-17】到位判据带宽 px（原写死 #define 26，改可调；串口 K）
                                                //   为什么要可调：带宽 ±26px = ±1cm 太宽，球高速【穿带】时
                                                //   只要在带内待 200ms 就判"到了"，把过冲掩盖了
                                                //   （实测：判 +5 OK 那一刻球其实在 +155px = +5.96cm）。
static volatile int32  T3G_KP  = -2200;         // 【2026-10-17 定案】题3 增益提出来做成初值（串口 L/I/R 可调）
static volatile int32  T3G_KPN = -2000;         //   定案依据（题3 +5cm 峰值实测，目标 +130px）：
static volatile int32  T3G_KD  = -60000;        //     原 KP-2500 KD-48000 -> 峰值 +209px(+8.04cm) 过冲 +3.04cm ❌
static volatile int32  T3G_KI  = -60;           //     新 KP-2200 KD-60000 -> 峰值 +107~+137(+4.1~5.3cm) 过冲 ≤0.5cm ✅
                                                //     -5cm 处误差 +2~-4px(≤0.15cm，限 26px)；总时长 3.7~4.5s（限 5s）

volatile uint32 g_isr_cnt = 0;                  // 10ms 控制中断计数：【仅诊断】主循环被电机拖慢到什么程度，此计数才是真实节拍

// 【2026-10-16 调试】台架直驱：非 0 时两轮直接吃这个 duty，绕过全部状态机
//   为什么需要：架空测试时七路全白 → line_lost 分支会把车截停；题4 也被 line_lost 挡。
//   有了它才能"分级升 duty，看 K230 到哪一档开始挂"，把电机/K230 的耦合一步步定位。
static volatile int32 g_bench_duty = 0;

//-------------------------------------------------------------------------------------------------------------------
// 【2026-10-16】锚定视觉 PID（题4/5/6：中点/任意位保球）——搬进 10ms 中断
//   为什么搬：主循环实测【电机一转就从 39.5Hz 掉到 9.7Hz】（单轮最长 220ms），根因是
//     disp_poll() 跑动时计时值每 10ms 就变 → 每轮都刷 OLED，而软 I2C 一行 16px 要 ~100ms，
//     等于把 CPU 全喂了屏幕。锚定 PID 只有 4.5~9.7Hz 带宽，压不住起步/转弯的加速度扰动
//     （实测球摆 ±280px ≈ ±11cm，题目要求 ±1cm）。
//   10ms 中断实测节拍稳定（电机转不转都是 99.6~100Hz），所以整条球控链路都放这里。
//   舵机所有权：t3_run / t3_home / t3_seq 任一非 0 时归主循环的题3控制器，本函数立即返回。
//   与主循环版的差异：不做静摩擦破粘 kick —— 题4/5/6 用的中心档 STICTION_KICK=0，
//     这段本来就是空转；kick 的计数器是"帧计数"，搬进 100Hz 中断语义会变，故不搬。
//-------------------------------------------------------------------------------------------------------------------
static int32  vh_integral     = 0;              // 条件慢积分
static int32  vh_err_last     = 0;
static int32  vh_diff_hold    = 0;              // 帧级差分保持值（新视觉帧才刷新，与主循环版同语义）
static uint32 vh_frame_ms_last = 0;
static uint8  vh_first        = 1;              // 上电第一次：直接吃掉首帧差分，避免 err_diff 假脉冲
static int32  vh_stuck        = 0;              // 破粘计数（帧计数，跟视觉帧同步）
static int32  vh_kick_hold    = 0;              // 破粘脉冲剩余 ISR 拍数

static void vision_hold_isr (void)
{
    int16  v_err = 0;
    uint8  v_status = 0;
    uint32 v_age = 0;
    uint8  v_valid = 0;
    uint8  new_frame = 0;                       // 本拍是否有新视觉帧（P/I/D 与破粘计数都跟它同步）
    int32  tgt = 0, err = 0, err_diff = 0, delta = 0, kp_eff = 0, us = 0, kick = 0;

    if((0 != t3_run) || (0 != t3_home) || (0 != t3_seq))
    {
        return;                                 // 舵机让给题3控制器
    }

    v_valid = comm_get_vision(&v_err, &v_status, &v_age)
              && (2 == v_status) && (v_age < VISION_AGE_MAX);

    if((!v_valid) || ((0 == SERVO_KP) && (0 == SERVO_KI) && (0 == SERVO_KD)))
    {
        // 无有效球 / 视觉伺服关闭：清积分与差分，避免再见到球时积分饱和冲舵
        vh_integral  = 0;
        vh_err_last  = 0;
        vh_diff_hold = 0;
        vh_first     = 1;
        // 【2026-10-16】失联超过 VISION_AGE_MAX 就把摆杆回水平：
        //   原来这里直接 return，舵机保持上一个指令 —— 若上一拍是大倾角（追球中），
        //   视觉一丢球就顺着那个倾角一路滚到端头撞停（实测丢了 4 秒后球停在 +262px 回不来）。
        //   回水平后球靠静摩擦自己停住，是失联期间最好的被动状态。
        //   短暂掉帧（<300ms）不回中，免得每帧抖动。
        if(v_age >= VISION_AGE_MAX)
        {
            servo_set_us((uint16)SERVO_MID_VISION);
        }
        return;
    }

    tgt = VISION_TARGET_ERR;                    // 题6 任意位：非 0 目标；题4/5：0=中点
    if((0 == VISION_TARGET_ERR) && (0 < DITHER_PX))
    {
        tgt = (0 != ((comm_get_ms() / DITHER_MS) & 1)) ? DITHER_PX : -DITHER_PX;
    }
    err = (int32)v_err - tgt;


    // ---- 帧同步：本函数 100Hz，视觉帧只有 27~43fps ----
    //   P/I/D 全部只在【新帧】到来时推进一次，保证与定案时（主循环逐帧跑）的数学完全一致：
    //   否则积分每秒累积次数变 10 倍，调好的 KI=-25 实际效果变成 -250，中位保持立刻振起来。
    //   搬进中断换来的真正收益是【不再丢帧】：主循环 9.7Hz 时 33~43fps 的帧被丢掉 3/4，
    //   等效更新时间 100ms+；现在是帧一到就更新，等效更新率 = 帧率本身。
    {
        uint32 fms = comm_get_frame_ms();
        if(vh_first)
        {
            vh_first         = 0;
            vh_frame_ms_last = fms;
            vh_err_last      = err;
            vh_diff_hold     = 0;
            new_frame        = 1;
        }
        else if(fms != vh_frame_ms_last)
        {
            vh_frame_ms_last = fms;
            new_frame        = 1;
        }

        if(new_frame)
        {
            // 条件慢积分：|err| < SERVO_KI_GATE 才积累，远段清零防饱和（消静摩擦死区靠这一项）
            if((SERVO_KI_GATE > err) && (-SERVO_KI_GATE < err))
            {
                vh_integral += err;
                if(vh_integral > SERVO_KI_MAX) { vh_integral = SERVO_KI_MAX; }
                if(vh_integral < SERVO_KI_MIN) { vh_integral = SERVO_KI_MIN; }
            }
            else
            {
                vh_integral = 0;
            }
            // 帧级微分：帧间差分（循环级差分在 100Hz 下几乎恒为 0，KD 会完全失效）
            vh_diff_hold = err - vh_err_last;
            vh_err_last  = err;
        }
    }
    err_diff = vh_diff_hold;

    // ---------- 小破粘脉冲（2026-10-16 最小实现）----------
    // 为什么加：小土反复验证「球往负侧刨，老是停在 +1cm；往正侧刨倒能停在 0」——
    //   是个【稳定复现】的卡点，不是随机。位置约 +26px(1cm)。
    //   该点 P 项只有 1400*26/1000 = 36µs，条件积分满幅也只有 75µs，合计 111µs，
    //   而实测静摩擦脱困阈值约 300µs -> 球永远出不来。
    // 设计：只在 |err|>20px（超出 ±1cm 规格区）且连续 8 帧位置几乎不变时才给一小段脉冲；
    //   中心保持时 |err|<=5，永远不会触发（σ=0.7px 不受影响）。
    // 脉冲在限幅【之外】叠加，否则会被 DELTA_NEAR=180 吃掉（这是之前失败的教训）。
    {
        #define VH_KICK_LO        ( 20 )            // 触发内门限 px（±1cm 规格区内不干预）
        #define VH_KICK_HI        ( 400 )           // 触发外门限 px（端头挡片不是静摩擦，不踢）
        #define VH_KICK_HOLD_N    ( 25 )            // 连续 25 帧几乎不动才踢（帧同步 ≈625ms@40fps）
                                                //   8 -> 25：8 帧太短，正常回中的减速段球也会"几乎不动"，
                                                //   破粘被误触发 -> 额外注入能量、过冲反而变大（实测 -70~+102px）。
        #define VH_KICK_PULSE     ( 10 )            // 脉冲长度（ISR 次数 ×10ms = 100ms）
        int32 eabs = (0 > err) ? -err : err;

        if(0 < vh_kick_hold)
        {
            vh_kick_hold--;
            kick = (0 < err) ? STICTION_KICK : -STICTION_KICK;   // 与 P 同向：推向目标
        }
        else if((VH_KICK_LO < eabs) && (VH_KICK_HI > eabs)
                && (-5 < err_diff) && (5 > err_diff))
        {
            if(new_frame && (++vh_stuck >= VH_KICK_HOLD_N))
            {
                vh_stuck     = 0;
                vh_kick_hold = VH_KICK_PULSE - 1;
                kick = (0 < err) ? STICTION_KICK : -STICTION_KICK;
            }
        }
        else
        {
            vh_stuck = 0;
        }
    }

    // PID（符号：err>0 球偏右 → 往 +us 打；SERVO_KP 自带负号）
    kp_eff = ((-100 < err) && (err < 100)) ? KP_NEAR : SERVO_KP;
    if(0 < err) { kp_eff = kp_eff * KP_NEG_SCALE / 100; }
    delta = -((int32)kp_eff * err + (int32)SERVO_KI * vh_integral + (int32)SERVO_KD * err_diff) / 1000;

    if(0 != ACC_FF)                             // 纵向加速度前馈：车加速球后甩，舵机预倾补偿
    {
        delta -= (int32)((int64)ACC_FF * acc_long_lpf / 1000);
    }
    delta -= g_start_ff;                        // 【起步前馈】斜坡期间已知加速度的倾角补偿（球惯性）

    {                                           // 分级限幅：远段大限幅快走，近段小限幅稳停
        int32 g = ((-100 < err) && (err < 100)) ? DELTA_NEAR : DELTA_LIMIT;
        if(delta > g)  { delta = g;  }
        if(delta < -g) { delta = -g; }
    }

    // 【2026-10-16 关键修复】破粘脉冲必须在限幅【之外】叠加
    //   为什么：球卡住时 PID 输出早就顶在限幅上了（实测 us=1905 正是 DELTA_LIMIT 的位置），
    //   破粘脉冲加进去又被同一个限幅截掉 —— 一点额外权限都没有，踢多大都白搭。
    //   实测该位置从静止脱困需要 delta >= 300，而近段限幅只有 330（且被 P 项占满）。
    //   现在：限幅管"连续控制权限"（小 = 防过冲），破粘管"一次性脱困"（大 = 突破静摩擦）。

    delta += kick;                              // 破粘脉冲叠加在限幅【之外】（见上面说明）

    {                                           // 只防超程的最终保护，不参与控制
        // 齿条物理行程：题3 实测 us=1260(delta=-400) 已顶到限位，正方向约到 us=2060
        #define DELTA_ABS_MAX   ( 420 )
        if(delta > DELTA_ABS_MAX)  { delta = DELTA_ABS_MAX; }
        if(delta < -DELTA_ABS_MAX) { delta = -DELTA_ABS_MAX; }
    }

    us = (int32)SERVO_MID_VISION + delta;
    if(SERVO_ABS_MAX_US < us) { us = SERVO_ABS_MAX_US; }
    if(SERVO_ABS_MIN_US > us) { us = SERVO_ABS_MIN_US; }
    servo_set_us((uint16)us);
}

void pit_ctrl_callback (uint32 event, void *ptr)
{
    g_isr_cnt++;

    // 视觉链路（comm_poll + vision_hold_isr）已移到 pit_enc_callback —— 那个中断全程不关，
    // 见那里的注释：run_stop() 会 pit_disable(PIT_CTRL_TIMER)，挂这里会导致永久丢帧。

    // 【2026-10-16 调试】台架直驱优先于一切状态机：非 0 就直接给两轮 duty（架空排查用）
    if(0 != g_bench_duty)
    {
        ctrl_duty_l = g_bench_duty;
        ctrl_duty_r = g_bench_duty;
        motor_set(MOTOR_CH_L, (int16)g_bench_duty);
        motor_set(MOTOR_CH_R, (int16)g_bench_duty);
        return;
    }

    static int32 pos_prev = 0;
    static int32 integ_l = 0, integ_r = 0;
    static int32 stall_cnt_l = 0, stall_cnt_r = 0;
    static int32 run_ms = 0;                    // 起步后已运行时间
    static uint32 dark_ms = 0;                  // 冲出赛道保护：全黑(n>=5)已持续毫秒
    static uint8  dark_armed = 0;               // 冲出赛道保护武装位：起跑后见过一次"非全黑"才武装
    int32 pos = 0;
    int32 diff = 0, target_l = 0, target_r = 0;
    int32 err_l = 0, err_r = 0, new_l = 0, new_r = 0;
    int32 rpm_l = 0, rpm_r = 0;

    //---------------------------------------------------------------------------
    // 传感器采样移到本中断（100Hz 固定节拍）
    //   原先 ir_update() 在主循环里调用，而主循环在【电机转动时】实测掉到 4.5Hz
    //   （IDLE 63Hz / RUNNING 4.5Hz），导致：
    //     丢线防抖 10 帧 ÷ 4.5Hz = 2.2 秒才认丢线 -> 出弯出线后"刹不住"，
    //     车带着 100% duty 冲出赛道布在瓷砖上乱跑。
    //   放进 10ms 中断后采样固定 100Hz，丢线检测延迟 = 10 帧 x 10ms = 100ms。
    //---------------------------------------------------------------------------
#if !VISION_TEST_MODE
    ir_update();
#endif
    pos = line_pos_x100;

    // 串口启动状态机处理
    if(RUN_STATE_IDLE == g_run_state)           // 待机：强制停车
    {
        ctrl_duty_l = 0;
        ctrl_duty_r = 0;
        integ_l = 0;
        integ_r = 0;
        stall_cnt_l = 0;
        stall_cnt_r = 0;
        stall_l_flag = 0;
        stall_r_flag = 0;
        base_smooth = 0;                        // 待机时斜坡状态归零，防止下一圈启动前冲
        dark_armed = 0;                         // 冲出赛道保护重新武装（下一轮起跑后见过白底才计数）
        dark_ms = 0;
        motor_set(MOTOR_CH_L, 0);
        motor_set(MOTOR_CH_R, 0);
        pos_prev = pos;
        run_ms = 0;
        return;
    }
    if(RUN_STATE_WAIT_TAPE == g_run_state)      // 等待起始胶带模式
    {
        motor_set(MOTOR_CH_L, 0);
        motor_set(MOTOR_CH_R, 0);
        stall_l_flag = 0;                       // 进入等待/倒计时即解除历史堵转锁定，防止 x=1 后再起跑被旧 stall 钳住
        stall_r_flag = 0;
        g_run_timer_ms += PIT_PERIOD_MS;
        if(is_on_start_tape())
        {
            g_run_state = RUN_STATE_COUNTDOWN;
            g_run_timer_ms = 0;
            printf("tape detected, countdown %d ms...\r\n", RUN_COUNTDOWN_MS);
        }
        else if(RUN_WAIT_TAPE_MS <= g_run_timer_ms)
        {
            g_run_state = RUN_STATE_IDLE;
            g_run_timer_ms = 0;
            printf("start TIMEOUT: no tape in %d ms\r\n", RUN_WAIT_TAPE_MS);
        }
        pos_prev = pos;
        run_ms = 0;
        return;
    }
    if(RUN_STATE_COUNTDOWN == g_run_state)      // 2s 倒计时
    {
        motor_set(MOTOR_CH_L, 0);
        motor_set(MOTOR_CH_R, 0);
        stall_l_flag = 0;
        stall_r_flag = 0;
        base_smooth = 0;                        // 【2026-10-17 修】起步猛冲的真因：
        integ_l = 0; integ_r = 0;               //   run_stop()（x=0）第一件事就是 pit_disable(PIT_CTRL_TIMER)，
                                                //   而下面 IDLE 分支那句 base_smooth=0 就在这个中断里 —— 中断一关
                                                //   它永远跑不到，于是 base_smooth 冻在上一圈的 149。
                                                //   下一次 x=1 目标速度从 149rpm 起（不是从 0），RAMP_STEP=3 要 3 秒
                                                //   才降回正常斜坡 → 表现就是"起步猛冲"，且第一次开机不冲、越跑越冲。
                                                //   实测证据：x=0 后遥测 bs=149（rpm 已是 0）；发 J=0 让待机分支跑一遍后 bs=0。
        g_run_timer_ms += PIT_PERIOD_MS;
        if(RUN_COUNTDOWN_MS <= g_run_timer_ms)
        {
            g_run_state = RUN_STATE_RUNNING;
            g_run_timer_ms = 0;
            printf("run=1 (GO)\r\n");
        }
        pos_prev = pos;
        run_ms = 0;
        return;
    }

    if(line_lost)                               // 丢线保持：LOST_HOLD_MS 内冻结 duty/积分续行（过断线/直角），超时停车清积分
    {
        stall_l_flag = 0;                       // LOST = 冲出赛道/被捡车：堵转锁定同步解除，放回线上即恢复
        stall_r_flag = 0;
        stall_cnt_l  = 0;
        stall_cnt_r  = 0;
        lost_hold_ms += PIT_PERIOD_MS;
        if(lost_hold_limit < lost_hold_ms)
        {
            // 只报一次：证明"全白超时截停"这条路径确实执行了
            if(!lost_stop_reported)
            {
                lost_stop_reported = 1;
                printf("LOST STOP: %dms >= limit %d  (whiteline timeout)\r\n",
                       (int)lost_hold_ms, (int)lost_hold_limit);
            }
            ctrl_duty_l = 0;
            ctrl_duty_r = 0;
            integ_l = 0;
            integ_r = 0;
            if(0 < base_smooth)                 // 停车期间 base_smooth 同步滑到 0，防找回线时目标猛冲
            {
                base_smooth -= (RAMP_STEP < base_smooth) ? RAMP_STEP : base_smooth;
            }
            motor_set(MOTOR_CH_L, 0);
            motor_set(MOTOR_CH_R, 0);
        }
        else
        {
            lost_stop_reported = 0;
        }
        pos_prev = pos;
        return;
    }
    lost_hold_ms = 0;
    run_ms += PIT_PERIOD_MS;                    // 起步计时：前 300ms 限制 diff 抑冲

    //---------------------------------------------------------------------------------------------------------------
    // 冲出赛道保护（深色地面 / 黑块持续）
    //   背景：起跑时车压在起停胶带上（七路黑，n>=IR7_TAPE_N），此时不能判出线，
    //         否则起跑 1 秒内就被误停（曾犯此错）。
    //   判据：用"武装位"而不是时间宽限 —— 起跑后只要见过一次【非全黑】(即白色赛道布)，
    //         说明胶带已经过去、车确实在赛道上；此后若再出现【全黑持续 DARK_STOP_MS】，
    //         就不是"压过一条窄胶带"（真胶带 <0.3s 通过），而是冲出赛道布压在深色地面 → 立即截停。
    //   这样既不误停起跑胶带，也能兜住"跑到瓷砖/地面上还在开"的情况。
    //---------------------------------------------------------------------------------------------------------------
    if(!line_hold)
    {
        dark_armed = 1;                         // 见到白底（正常赛道布）→ 武装
        dark_ms = 0;
    }
    else if(dark_armed)
    {
        dark_ms += PIT_PERIOD_MS;
        if(DARK_STOP_MS <= dark_ms)
        {
            printf("OFF-TRACK: dark %ums, STOP\r\n", (unsigned int)dark_ms);
            dark_ms = 0;
            dark_armed = 0;
            line_hold = 0;
            ctrl_duty_l = 0;
            ctrl_duty_r = 0;
            integ_l = 0;
            integ_r = 0;
            base_smooth = 0;
            motor_enable(0);                    // 硬件关断电机（双保险）
            g_run_state = RUN_STATE_IDLE;       // 回待机，跑表定格
            g_run_last_ms = g_run_timer_ms;
            pos_prev = pos;
            run_ms = 0;
            return;
        }
    }
    else
    {
        dark_ms = 0;                            // 未武装（还在起跑胶带上）不计数
    }

    //---------------------------------------------------------------------------------------------------------------
    // 题4：匀速直行（跳过循迹，锚定 PID 在下方主循环仍在线保球）
    //   来源 topic/三题 573e187；三题原版把转速写死 60，此处改用 t4_target_rpm 便于串口调
    //   与循迹分支互斥：本分支 return 掉，保证"匀速"不被循迹差速污染
    //---------------------------------------------------------------------------------------------------------------
    if(t4_run)
    {
        if(0 == t4_t0) { t4_t0 = comm_get_ms(); }
        uint32 ela = comm_get_ms() - t4_t0;
        if(ela < 1500)                          // 1.5s 匀加速到目标转速（防起步冲击把球甩飞）
        {
            base_smooth = t4_target_rpm * (int32)ela / 1500;
            g_start_ff = (int32)((int64)START_FF_K * t4_target_rpm * 10 / 1500);   // 【起步前馈】题4 已知斜坡
        }
        else
        {
            base_smooth = t4_target_rpm;
            g_start_ff = 0;
        }
        target_l = base_smooth;
        target_r = base_smooth;

        rpm_l = enc_l_10ms * 100 * 60 / ENC_PPR;
        rpm_r = enc_r_10ms * 100 * 60 / ENC_PPR;
        err_l = target_l - rpm_l;
        err_r = target_r - rpm_r;
        // 条件积分：|err|<=门限才累积，大误差段【冻结】（不累积但也不清零，
        // 保留已积累量，否则大误差时积分永远建立不起来，稳态误差消不掉）
        if((INTEG_ERR_GATE >= err_l) && (-INTEG_ERR_GATE <= err_l)) { integ_l += err_l; }
        if((INTEG_ERR_GATE >= err_r) && (-INTEG_ERR_GATE <= err_r)) { integ_r += err_r; }
        if(INTEG_MAX < integ_l) { integ_l = INTEG_MAX; }
        if(-INTEG_MAX > integ_l) { integ_l = -INTEG_MAX; }
        if(INTEG_MAX < integ_r) { integ_r = INTEG_MAX; }
        if(-INTEG_MAX > integ_r) { integ_r = -INTEG_MAX; }
        // 【2026-10-16 修 bug】原来是"绝对式"：new = KP*err + KI*integ，
        //   duty 上限被锁死在 12*70+100 = 940，而 PWM_DUTY_MAX=10000（只有 9.4% 占空比），
        //   实测电机给了 duty=940 但轮子完全不转（转速 0）。这段从 topic/三题 抄来，
        //   注释自己写着"未上台验证"，所以一直没暴露。
        //   改成与循迹分支一致的"增量式"：new = 上次duty + KP*err + KI*integ，
        //   让 duty 能一路爬到能推动轮子为止（DUTY_SLEW 限速防猛冲）。
        new_l = ctrl_duty_l + SPD_KP[gain_set] * err_l + SPD_KI[gain_set] * integ_l;
        new_r = ctrl_duty_r + SPD_KP[gain_set] * err_r + SPD_KI[gain_set] * integ_r;
        // 【2026-10-16 修 bug】补速度环 D（measurement 微分），与循迹分支一致。
        //   缺 D 时 rpm 一冲过目标就猛收 duty、掉了又猛加 —— 实测车轮"脉冲式前进"。
        {
            static int32 t4_rpm_l_prev = 0, t4_rpm_r_prev = 0;
            new_l -= SPD_KD[gain_set] * (rpm_l - t4_rpm_l_prev);
            new_r -= SPD_KD[gain_set] * (rpm_r - t4_rpm_r_prev);
            t4_rpm_l_prev = rpm_l;
            t4_rpm_r_prev = rpm_r;
        }
        if(DUTY_SLEW < (new_l - ctrl_duty_l)) { new_l = ctrl_duty_l + DUTY_SLEW; }
        if(-DUTY_SLEW > (new_l - ctrl_duty_l)) { new_l = ctrl_duty_l - DUTY_SLEW; }
        if(DUTY_SLEW < (new_r - ctrl_duty_r)) { new_r = ctrl_duty_r + DUTY_SLEW; }
        if(-DUTY_SLEW > (new_r - ctrl_duty_r)) { new_r = ctrl_duty_r - DUTY_SLEW; }
        if(DUTY_RUN_MAX < new_l) { new_l = DUTY_RUN_MAX; }
        if(-DUTY_RUN_MAX > new_l) { new_l = -DUTY_RUN_MAX; }
        if(DUTY_RUN_MAX < new_r) { new_r = DUTY_RUN_MAX; }
        if(-DUTY_RUN_MAX > new_r) { new_r = -DUTY_RUN_MAX; }
        ctrl_duty_l = new_l;
        ctrl_duty_r = new_r;
        motor_set(MOTOR_CH_L, ctrl_duty_l);
        motor_set(MOTOR_CH_R, ctrl_duty_r);
        return;
    }

    {
        uint8  gs = gain_set;                   // 本拍固定用同一档，防切档瞬间半新半旧
        int32  spd_kp = SPD_KP[gs], spd_ki = SPD_KI[gs];
        int32  line_kp = LINE_KP[gs], line_kd = LINE_KD[gs], line_kq = LINE_KQ[gs];
        int32  base_eff = 0, pos_abs = (0 > pos) ? -pos : pos;

        // 非线性转向：diff = KP*pos/100 + KQ*pos*|pos|/10000 + KD*dpos/100（rpm）
        // KQ=5、pos=300 时二次项 45rpm——直道（小 pos）软、弯道（大 pos）狠
        // （弯道伺服定目标案已否决：环路靠 pos→0 自平衡找转弯半径，强加 setpoint 会把平衡点推出窗外）
        diff = line_kp * pos / 100 + line_kq * pos * pos_abs / 10000 + line_kd * (pos - pos_prev) / 100;
        pos_prev = pos;
        if(DIFF_MAX < diff) diff = DIFF_MAX;
        if(-DIFF_MAX > diff) diff = -DIFF_MAX;
        {                                       // 钢球丝滑化（A10）：diff 低通（新值 DIFF_LPF_NUM/10，串口 g 1~10 越大越灵，默认 7≈滞后5ms）
            static int32 diff_f = 0;
            diff_f = (diff_f * (10 - DIFF_LPF_NUM[gs]) + diff * DIFF_LPF_NUM[gs]) / 10;
            diff = diff_f;
        }
        {                                           // 弯道前馈缓坡（m1_v3）：消除进弯阶跃，300ms 内从 0 爬到 n
            static int32 ff_corner_ramp = 0;
            static uint8 ff_ramp_cnt = 0;
            int32 ff_target = (in_corner && (0 < pos)) ? FF_CORNER_DIFF[gs] : 0;
            if(++ff_ramp_cnt >= FF_RAMP_PERIOD)     // h*10ms 更新一次 ramp，默认 3=300ms 爬到 n=10
            {
                ff_ramp_cnt = 0;
                if(ff_corner_ramp < ff_target)
                {
                    ff_corner_ramp += 1;
                    if(ff_corner_ramp > ff_target) ff_corner_ramp = ff_target;
                }
                else if(ff_corner_ramp > ff_target)
                {
                    ff_corner_ramp -= 1;
                    if(ff_corner_ramp < ff_target) ff_corner_ramp = ff_target;
                }
            }
            diff += ff_corner_ramp;
            if(DIFF_MAX < diff) diff = DIFF_MAX;
            if(-DIFF_MAX > diff) diff = -DIFF_MAX;
        }

        base_eff = in_corner ? CORNER_RPM[gs] : BASE_RPM[gs];   // 弯道识别：进弯降安全速，出弯顶满速

        // 起步线性加速：前 350ms 目标速度从 0 线性爬到 base_eff，防起步前冲
        // 起步线性加速：目标速度从 0 线性爬到 base_eff，防起步前冲
        // 【2026-10-15】350 -> 800ms：小土反馈"起步太冲"，斜坡拉长让起步更柔
        if(run_ms < START_RAMP_MS)
        {
            base_eff = base_eff * run_ms / START_RAMP_MS;       // 只压速度
        }
        // 【2026-10-16 修】转向 diff 的起步柔化必须和速度斜坡【解耦】
        //   原来两行共用 START_RAMP_MS。我把 START_RAMP_MS 从 800 改到 4000（为了球稳）之后，
        //   转向权限也被压成 4 秒斜坡（第 1 秒只有 25%）→ 起步在弯道里转不过来直接冲出去；
        //   同时 pos 一直累积，等斜坡放开时 diff 已经很大 -> 猛地一拧 = 小土说的"起步脉冲"。
        //   转向只管 600ms：够抑制起步瞬间的大 pos 猛拧，又不影响循迹。
        if(run_ms < START_DIFF_RAMP_MS)
        {
            diff = diff * run_ms / START_DIFF_RAMP_MS;
        }
        if(line_hold && (HOLD_CRUISE_RPM < base_eff))       // 全黑通过帧（压启停胶带）：限速防盲加速冲线
        {
            base_eff = HOLD_CRUISE_RPM;
        }
        if(g_bench_mode)                                    // 台架模式：忽略 line_hold 限速，直接用 BASE_RPM
        {
            base_eff = BASE_RPM[gs];
            diff     = 0;                                   // 两轮同目标，不做差速修正
        }
        base_eff -= SLOW_KP[gs] * ((0 < pos) ? pos : 0) / 100;  // 比例预减速（顺时针写死：只右弯 pos>0 生效，直道左晃不误收油）
        // 【2026-10-17 修】40% 下限不能管起步斜坡：
        //   原来这一句无条件执行，run_ms=0 时斜坡算出 base_eff=0，立刻被抬回 59rpm，
        //   于是"4 秒线性斜坡"的前 1.6 秒实际是【直接给 59rpm 的硬起步】——球被这一下甩到 -3.00cm。
        //   斜坡期间跳过下限，让速度真的从 0 爬起来（和软停车越过下限是同一个道理）。
        if((run_ms >= START_RAMP_MS) && ((BASE_RPM[gs] * 2 / 5) > base_eff))
        {
            base_eff = BASE_RPM[gs] * 2 / 5;
        }

        // ---- 【2026-10-17】软停车：从 STOP_START_MS 起按 3s 版斜率线性减速 ----
        //   为什么：原来定时到点直接 motor_enable(0)，车从 0.54m/s 靠摩擦在 0.3~0.5s 内停住
        //   = 减速度 1.5~2 m/s²，是起步斜坡(0.135)的 11 倍；而舵机前馈 g_start_ff 只在
        //   起步斜坡期间计算，停车时没有任何补偿 -> 球按惯性直接冲到杆端（实测 +11cm）。
        //   现在减速段是【已知的】线性减速，并给舵机加【反向】前馈。
        //   【小土定】减速度 = STOP_RAMP_MS(3s) 那套，只把【起点】改到 STOP_START_MS。
        //   ⚠️ 本段必须在 40% 限幅【之后】：否则 base_eff 会被抬回 59rpm，减不下来。
        g_stop_ff = 0;
        if((0 < g_run_stop_ms) && ((uint32)STOP_START_MS < g_run_stop_ms)
           && (run_ms >= STOP_START_MS))
        {
            int32 elapsed = (int32)run_ms - (int32)STOP_START_MS;   // 已经减速了多少 ms
            if(0 > elapsed) elapsed = 0;
            if((int32)STOP_RAMP_MS < elapsed) elapsed = (int32)STOP_RAMP_MS;
            base_eff  = (int32)((int64)BASE_RPM[gs] * ((int32)STOP_RAMP_MS - elapsed) / STOP_RAMP_MS);
            g_stop_ff = -(int32)((int64)START_FF_K * BASE_RPM[gs] * 10 / STOP_RAMP_MS);
        }

        // 弯道判定（迟滞防抖动）：|pos|>=1.0 持续50ms 进弯；<=0.6 持续100ms 出弯；中间带保持
        if(CORNER_POS_TH <= pos_abs)
        {
            if(CORNER_MS <= ++corner_ms) in_corner = 1;
        }
        else if(EXIT_POS_TH >= pos_abs)
        {
            if(EXIT_MS <= ++exit_ms) in_corner = 0;
        }

        // 目标速度斜坡：公共基准一律按 ±RAMP_STEP(rpm/10ms) 爬升/下滑，变速滑行治一卡一卡
        // 转向 diff 不平滑，保持即时响应
        {
            int32 delta = base_eff - base_smooth;
            if(RAMP_STEP < delta) delta = RAMP_STEP;
            if(-RAMP_STEP > delta) delta = -RAMP_STEP;
            base_smooth += delta;

            // 【起步前馈】斜坡期间按【已知】加速度算倾角补偿（球惯性）
            if(run_ms < START_RAMP_MS)
            {
                g_start_ff = (int32)((int64)START_FF_K * BASE_RPM[gs] * 10 / START_RAMP_MS);
            }
            else if(0 != g_stop_ff)
            {
                g_start_ff = g_stop_ff;     // 【软停车】减速段：反向前馈顶住球的惯性
            }
            else
            {
                g_start_ff = 0;
            }
        }

        target_l = base_smooth + diff;          // pos 负 = 线在左 → 左轮减速右轮加速，差速左拐
        target_r = base_smooth - diff;
        target_l_dbg = target_l;                // 诊断快照（主循环遥测打印用）
        target_r_dbg = target_r;
        diff_dbg = diff;
        if(0 > target_l) target_l = 0;
        if(0 > target_r) target_r = 0;

        rpm_l = enc_l_10ms * 6000 / ENC_PPR;
        rpm_r = enc_r_10ms * 6000 / ENC_PPR;
        err_l = target_l - rpm_l;
        err_r = target_r - rpm_r;

        if(INTEG_ERR_GATE >= ((0 > err_l) ? -err_l : err_l))    // 条件积分：小误差才累积，大误差段冻结
        {
            integ_l += err_l;
        }
        if(INTEG_ERR_GATE >= ((0 > err_r) ? -err_r : err_r))
        {
            integ_r += err_r;
        }
        if(INTEG_MAX < integ_l) integ_l = INTEG_MAX;
        if(-INTEG_MAX > integ_l) integ_l = -INTEG_MAX;
        if(INTEG_MAX < integ_r) integ_r = INTEG_MAX;
        if(-INTEG_MAX > integ_r) integ_r = -INTEG_MAX;

        new_l = ctrl_duty_l + spd_kp * err_l + spd_ki * integ_l;
        new_r = ctrl_duty_r + spd_kp * err_r + spd_ki * integ_r;
        {                                       // 速度环 D（measurement 微分，串口 v）：rpm 上窜即收 duty，压转速振荡（小土点单）
            static int32 rpm_l_prev = 0, rpm_r_prev = 0;
            new_l -= SPD_KD[gs] * (rpm_l - rpm_l_prev);
            new_r -= SPD_KD[gs] * (rpm_r - rpm_r_prev);
            rpm_l_prev = rpm_l;
            rpm_r_prev = rpm_r;
        }
    }

    if(DUTY_RUN_MAX < new_l) new_l = DUTY_RUN_MAX;
    if(0 > new_l) new_l = 0;
    if(DUTY_RUN_MAX < new_r) new_r = DUTY_RUN_MAX;
    if(0 > new_r) new_r = 0;

    if(DUTY_SLEW < (new_l - ctrl_duty_l)) new_l = ctrl_duty_l + DUTY_SLEW;
    if(DUTY_SLEW < (ctrl_duty_l - new_l)) new_l = ctrl_duty_l - DUTY_SLEW;
    if(DUTY_SLEW < (new_r - ctrl_duty_r)) new_r = ctrl_duty_r + DUTY_SLEW;
    if(DUTY_SLEW < (ctrl_duty_r - new_r)) new_r = ctrl_duty_r - DUTY_SLEW;

    // 堵转保护（双轮独立）：|duty|≥STALL_DUTY 且 |rpm|≤STALL_RPM 持续 STALL_MS → 该侧断电刹车清积分
    // 恢复：LOST 后找回线（捡车放回，LOST 分支已清标志）或 b=0 人工复位
    if(0 == BASE_RPM[gain_set])
    {
        stall_l_flag = 0;
        stall_r_flag = 0;
        stall_cnt_l  = 0;
        stall_cnt_r  = 0;
    }
    if((STALL_DUTY <= new_l) && (STALL_RPM >= ((0 > rpm_l) ? -rpm_l : rpm_l)))
    {
        stall_cnt_l += PIT_PERIOD_MS;
    }
    else
    {
        stall_cnt_l = 0;
    }
    if((STALL_DUTY <= new_r) && (STALL_RPM >= ((0 > rpm_r) ? -rpm_r : rpm_r)))
    {
        stall_cnt_r += PIT_PERIOD_MS;
    }
    else
    {
        stall_cnt_r = 0;
    }
    if(STALL_MS <= stall_cnt_l)
    {
        stall_l_flag = 1;
    }
    if(STALL_MS <= stall_cnt_r)
    {
        stall_r_flag = 1;
    }
    if(stall_l_flag)
    {
        new_l = 0;
        integ_l = 0;
    }
    if(stall_r_flag)
    {
        new_r = 0;
        integ_r = 0;
    }

    ctrl_duty_l = new_l;
    ctrl_duty_r = new_r;
    motor_set(MOTOR_CH_L, (int16)new_l);
    motor_set(MOTOR_CH_R, (int16)new_r);
}

//-------------------------------------------------------------------------------------------------------------------
// PIT_G1 外挂定时停车中断：与控制环完全独立，只计时和关电机，不影响主循环
//-------------------------------------------------------------------------------------------------------------------
void pit_stop_callback(uint32 event, void *ptr)
{
    static uint32 stop_ms = 0;
    (void)event;
    (void)ptr;

    if(RUN_STATE_RUNNING == g_run_state)
    {
        stop_ms += PIT_PERIOD_MS;
        g_run_timer_ms += PIT_PERIOD_MS;        // 跑表计时（OLED 显示；RUNNING 起才开始计，题纸"按键启动时开始计时"）
        if((0 < g_run_stop_ms) && (g_run_stop_ms <= stop_ms))
        {
            printf("PIT stop: %u ms\r\n", (unsigned int)stop_ms);
            g_run_state = RUN_STATE_IDLE;
            g_run_last_ms = g_run_timer_ms;     // 捕获成绩（OLED FIN 显示）
            g_t3_total_ms = 0;                  // 最新成绩优先（清掉题3旧成绩）
            g_start_ff = 0;             // 【2026-10-17 修】软停车减速段会把 g_start_ff 留在负值，
            g_stop_ff  = 0;             //   不清零的话停车后 vision_hold_isr 每拍仍扣它 -> 舵机被固定偏置顶住把球推跑
            motor_enable(0);            // 硬件关断电机，双保险
            stop_ms = 0;
        }
    }
    else
    {
        stop_ms = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// UART0 RX 回调：攒一行置标志，主循环解析（ISR 里不解析）
// 注意：本回调先于逐飞助手接收处理执行，读走的字节助手协议看不到（A04 不用助手，无影响）
//-------------------------------------------------------------------------------------------------------------------
void tuning_rx_callback (uint32 event, void *ptr)
{
    uint8 b = 0;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }
    if(cmd_ready)
    {
        return;                                 // 上行没取走就丢（助手 handler 会兜底读走 FIFO）
    }
    if(!uart_query_byte(UART_0, &b))
    {
        return;
    }
    if(('\r' == b) || ('\n' == b))
    {
        if(0 != cmd_len)
        {
            cmd_buf[cmd_len] = '\0';
            cmd_ready = 1;
        }
    }
    else if(15 > cmd_len)
    {
        cmd_buf[cmd_len++] = (char)b;
    }
    else
    {
        cmd_len = 0;                            // 超长作废，防越界
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 比赛六题模式：按键 A30 选题+启动，蜂鸣器 A14 报数（卡A11，2026-08-01 定案）
//   短按(<800ms)=切模式 1→2→3→4→5→6→1，蜂鸣 N 声报模式
//   长按(≥800ms)=启动当前模式；运行中长按=急停（等效 x=0 + 3=0）
//   模式：1=展示怠速(图传) 2=题2速度圈(m0,t8000) 3=题3静止滚球(开环bang-bang)
//         4=题4 AB段(m1,t5000) 5=题5整圈球稳中心(m1,t14750) 6=题6整圈球稳任意位置(m1,t14750,目标=串口 l)
//-------------------------------------------------------------------------------------------------------------------
#define KEY_PIN         ( A30 )                 // 启动/急停（短按：空闲=启动当前模式，运行中=急停）
#define KEY_MODE_UP     ( B1  )                 // 模式+（短按循环 1→6）
#define KEY_MODE_DN     ( B0  )                 // 模式-（短按循环 6→1）
#define KEY_TGT         ( A31 )                 // 题6 目标位置步进（+25px/次，>+150 回到 -150 循环）
#define KEY_LONG_MS     ( 800 )
static volatile uint8 g_mode = 1;                   // 当前模式（上电=1 展示怠速）

static void beep (uint8 n)                          // 蜂鸣器 A14 短鸣 n 声（空闲期用，阻塞 ~150ms/声）
{
    for(; n > 0; n--)
    {
        gpio_set_level(LED_PIN, 1);
        system_delay_ms(60);
        gpio_set_level(LED_PIN, 0);
        system_delay_ms(90);
    }
}

// 按键通用扫描：idx 0=A30 1=B1 2=B0 3=A31；返回 1=有一次有效按键（松开沿判定，30ms 消抖）
// 按键模块：未按=高电平、按下=低电平（上拉输入）；不分长短按，30ms~5s 的按下-松开都算一次
static uint8 key_scan (uint8 idx, gpio_pin_enum pin)
{
    static uint8  down[4] = {0, 0, 0, 0};
    static uint32 t0[4] = {0, 0, 0, 0};
    uint8 key = gpio_get_level(pin);                // 0=按下（按键模块按下出低电平）

    if((0 == key) && (0 == down[idx]))
    {
        down[idx] = 1;
        t0[idx] = comm_get_ms();
    }
    else if((0 != key) && (0 != down[idx]))
    {
        uint32 dt = 0;
        down[idx] = 0;
        dt = comm_get_ms() - t0[idx];
        if((30 <= dt) && (5000 > dt))
        {
            return 1;                               // 有效按键（不分长短）
        }
    }
    return 0;
}

static void servo_gain_preset (uint8 for_q3);       // 前置声明（定义在下方题3 区）

static void run_start (void)                        // 等效串口 x=1（比赛简化版：直进 2s 倒计时，不等胶带——题纸"按键启动"）
{
    servo_gain_preset(0);                           // 【2026-10-16】题2/4/5/6 一律用中心保持档，防题3 的高增益残留
    base_smooth = 0;                                // 【2026-10-17 修】双保险：x=0 关了 PIT_CTRL 后斜坡状态没人清
    motor_enable(1);                                // 允许电机响应
    g_run_state = RUN_STATE_COUNTDOWN;
    g_run_timer_ms = 0;
    pit_enable(PIT_CTRL_TIMER);                     // 重新打开控制环中断
    pit_enable(PIT_STOP_TIMER);                     // 重新打开外挂停车中断
    printf("start COUNTDOWN: GO in %d ms\r\n", RUN_COUNTDOWN_MS);
}

static void run_stop (void)                         // 等效串口 x=0
{
    g_run_state = RUN_STATE_IDLE;
    g_run_timer_ms = 0;
    g_start_ff = 0;                                 // 【2026-10-17 修】软停车/起步前馈必须在这里清：
    g_stop_ff  = 0;                                 //   pit_disable 后控制环不再刷新它们，残留会让舵机被固定偏置顶住
    motor_enable(0);                                // 最高优先级：硬件层强制刹车
    pit_disable(PIT_CTRL_TIMER);                    // 终极保险：直接关闭 10ms 控制环中断
    pit_disable(PIT_STOP_TIMER);                    // 同时关闭外挂停车中断
    printf("run=0 (idle, PIT OFF)\r\n");
}

static void t3_start (void)                         // 等效串口 3=1
{
    int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;
    if(!(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX)))
    {
        printf("T3 REFUSED: no valid vision frame (check K230/UART)\r\n");
        beep(3);                                    // 启动失败提示音：看不到球（K230 没在找球模式/球不在视野/网传断）
        return;
    }
    t3_run = 1;
    t3_state = 1;
    t3_t0 = comm_get_ms();
    t3_seg0 = t3_t0;
    t3_ref0 = v_err0;                               // 轨迹起点=当前实测位（球不一定正好在 O）
    t3_ref = t3_ref0;
    t3_x = v_err0;
    t3_vel = 0;
    t3_inband0 = 0;
    t3_extreme = v_err0;
    t3_reach_ms = 0;
    printf("T3 start: x0=%+d -> tgt %+d -> %+d (T1=%d T2=%d)\r\n",
           t3_ref0, T3_TGT_P5, T3_TGT_N5, T3_T1_MS, T3_T2_MS);
}

static void t3_stop (void)                          // 等效串口 3=0
{
    t3_run = 0;
    t3_state = 0;
    t3_inband0 = 0;
    servo_set_us((uint16)SERVO_MID_VISION);         // 收工回水平中位
    // 【2026-10-16 修】原来这里没有 pit_enable：发过 x=0（run_stop 会 pit_disable）之后，
    //   再发 3=0 是无法把控制中断打开的 -> ir_update() 不跑 -> 七路采样冻结在 0
    //   （表现为"传感器全白、调电位器/拔排线都不变"，害人排查半天）。补上，让 3=0 真正能"复位"。
    pit_enable(PIT_CTRL_TIMER);
    pit_enable(PIT_STOP_TIMER);
    g_start_ff = 0;
    printf("T3 stop (mid %d)\r\n", SERVO_MID_VISION);
}

static void mode_apply (uint8 m)                    // 切模式：该题参数整档写入（模式清空重验制：4/5/6 未验收占位，见 mode_start）
{
    switch(m)
    {
        case 2:     // 题2[已验证]：m0 速度圈（≤20s，m0_vFinal 定案 7.7s 圈速）
            gain_set = 0; RAMP_STEP = 6; FF_RAMP_PERIOD = 2; g_run_stop_ms = 8000;
            break;
        default:    // 1=展示怠速 / 3=题3[已验证，参数在题3区] / 4/5/6=未验收占位（不动参数）
            break;
    }
}

// ---------- 题3 位置序列（2026-10-16 新增，替代已失效的开环 bang-bang + 五次轨迹）----------
// 为什么换：原控制器是 2026-08-01 按旧机械标定的（OL acc=-260/brk=250/ta=400/tb=300）。
//   传动齿轮重装 + 舵机中位 1726→1660 之后，同样的脉宽对应的实际倾角全变了，
//   实测段1 只把球推到 +0.53cm（要求 +5cm），段2 五次轨迹彻底失稳、球在整根杆上来回飞。
// 新做法：不再单独造控制器，直接用已在用的"锚定 PID"做位置伺服——
//   把目标依次给 +5cm / -5cm，到位判据 |err-tgt| <= 26px(1cm) 持续 300ms，自动折返并计时打印。
// 增益要两套（题3 与题4/5/6 需求相反）：
//   题3   = 球要"大力推到指定位置"，要高 P + 强阻尼 + 破粘脉冲 + 中等积分
//   题456 = 球要"稳在中心"，低 P 纯 PD（带积分反而磨出低频摆）
#define T3_BRAKE_K         ( 5 )                // 题3 时间最优刹车距离系数：d_brake = v²/K（px）。
                                               //   偏小=更早刹车(保守,慢)；偏大=更晚刹车(快,易过冲)。
                                               //   理论值：a≈1380px/s²÷27fps²≈1.9px/帧² → K=2a≈3.8，取 5 留余量
#define T3_Q_INBAND_PX     ( 26 )               // 到位判据 ±26px = ±1cm（题纸要求）
#define T3_Q_INBAND_MS     ( 200 )              // 【2026-10-16】300->200：球高速穿带时 300ms 等不到会漏判
// t3_seq / t3_seq_t0 / t3_seq_in / t3_seq_t1 声明已上移到文件头（pit_ctrl_callback 里的
// vision_hold_isr 要用它判断舵机所有权，必须先可见）

static void servo_gain_preset (uint8 for_q3)    // 伺服增益整组切换（题3 / 题4-5-6）
{
    if(0 != for_q3)
    {   // 题3：2026-10-16 实测 4364ms 过题（限 5s，要求 ±5cm 处误差 ≤1cm）
        // DELTA_LIMIT 250->400：远段限幅是提速的主要杠杆（O->+5 从 2344ms 降到 1276ms）；
        // 400 时 us=1260 已顶到齿条物理限位 1300，但只是瞬时顶住，到位后 PID 立刻回拉。
        // STICTION_KICK 400->250：触发上限放开到 400px 后，400us 的踢在远段太猛，
        //   实测有一轮球冲过 -5cm 一路飞到 -248px（量程外）。
        SERVO_KP = T3G_KP; KP_NEAR = T3G_KPN; SERVO_KD = T3G_KD; SERVO_KI = T3G_KI;
        // 【2026-10-16 过冲修正】近段 P 从 -2500 降到 -800、近段限幅 180->400：
        //   失败轮球稳定冲到 -250~-278（过冲 ~146px）。算下来：G=400 把球推到 ~540px/s，
        //   而 H=180 的减速度只有 ~828 px/s²，刹车距离 = 540²/(2*828) ≈ 176px —— 对得上。
        //   过冲根因是"接近目标时 P 项还在猛推、压住 D 项的刹车"。近段 P 调小 + 给刹车足够权限。
        STICTION_KICK = 150; DELTA_LIMIT = 300; DELTA_NEAR = 180;
    }
    else
    {   // 题4/5/6：中心保持 —— 【2026-10-16 回退到基线】
        // 为什么回退：这一轮在上面叠了 KP/KICK/DELTA 等一堆试验性改动，越叠越乱
        //   （KP -1400->-2800、KD -18000->-65000、KICK 0->400、限幅 250/180->380/330，
        //    外加目标切换清积分破粘脉冲在限幅外叠加两处逻辑），实测静态回中反复失败。
        // 现在全部退回这一版基线，只保留与调参无关的东西：
        //   SERVO_MID_VISION = 1726   机械零点（小土调齿轮后实测方向反转点，与代码无关）
        //   comm_poll 挂 PIT_ENC      修 x=0 关 PIT_CTRL 导致永久丢帧（真 bug）
        //   OLED 软 I2C 50->10        主循环 9.7Hz -> 46Hz（真 bug）
        //   PID 在中断 + 与视觉帧同步  相当于把主循环版原样搬进 100Hz，语义不变
        //   视觉失联 >300ms 回水平     防丢帧时舵机保持大倾角把球甩到端头
        //   起步斜坡 800->4000ms      起步太冲（实测 800ms 时球受扰动 1196px/s²，超舵机权限）
        SERVO_KP = -1400; KP_NEAR = -1400; SERVO_KD = -70000; SERVO_KI = -25;
        STICTION_KICK = 250; DELTA_LIMIT = 250; DELTA_NEAR = 180;
    }
}

static void t3_seq_start (void)                 // 等效串口 3=1
{
    int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;

    if(!(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX)))
    {
        printf("T3 REFUSED: no valid vision frame (check K230/UART)\r\n");
        beep(3);                                // 看不到球（K230 没在找球模式 / 球不在视野 / 串口断）
        return;
    }
    servo_gain_preset(1);                       // 切题3 增益
    t3_seq    = 1;
    t3_seq_ext = v_err0;                    // 【2026-10-17】峰值从起点开始记
    t3_seq_t0 = comm_get_ms();
    t3_seq_in = 0;
    t3_seq_t1 = 0;
    printf("T3Q start: x0=%+d -> %+d -> %+d (inband %dpx x %dms)\r\n",
           v_err0, T3_TGT_P5, T3_TGT_N5, T3_Q_INBAND_PX, T3_Q_INBAND_MS);
}

static void mode_start (uint8 m)
{
    switch(m)
    {
        case 1:
            printf("MODE1 idle (stream show only)\r\n");
            break;
        case 2:
        case 3:
        {
            // [已验证模式] 启动归中：先把球驱到目标位（题3=中心O），到位才起跑；题2 不用球，无视觉也直接跑
            int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;
            if(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX))
            {
                t3_home_next = m;
                t3_home = 1;
                t3_home_ms = comm_get_ms();
                t3_home_inband = 0;
                printf("HOME first: drive ball to %+d, then start MODE %d\r\n", VISION_TARGET_ERR, m);
                beep(1);
            }
            else if(2 == m)
            {
                run_start();                        // 题2 不用球：无视觉也直接跑
            }
            else
            {
                printf("HOME REFUSED: no vision (mode %d)\r\n", m);
                beep(3);
            }
            break;
        }
        case 4:
        case 5:
        case 6:
            // [未验收占位] 模式清空重验制：跑道验证合格前不起跑（验收一个录一个）
            printf("MODE %d NOT VALIDATED yet (placeholder, cleared)\r\n", m);
            beep(3);
            break;
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// OLED 计时屏（SSD1306 128x64，题纸要求：启动即计时并显示，屏 ≤2 寸）
//   page0-1(8x16)：模式行  "M4 Q4 ABSEG"      page2(6x8)：参数行 "m1 r1 h3 t=5.0s"
//   page4-5(8x16)：计时行  "T= 12.34s"        page6-7(8x16)：状态行 "RUN/FIN/RDY"
//   整行覆写（补空格到行宽）防残影且省 clear 事务；disp_poll 5Hz 限频防阻塞控制环
//-------------------------------------------------------------------------------------------------------------------
static void oled_line16 (uint8 page, const char *s)
{
    char buf[17];
    uint8 i = 0;
    for(i = 0; i < 16; i++)
    {
        buf[i] = ('\0' != s[i]) ? s[i] : ' ';
    }
    buf[16] = '\0';
    oled_i2c_str16(0, page, buf);
}

static void oled_line6 (uint8 page, const char *s)
{
    char buf[22];
    uint8 i = 0;
    for(i = 0; i < 21; i++)
    {
        buf[i] = ('\0' != s[i]) ? s[i] : ' ';
    }
    buf[21] = '\0';
    oled_i2c_str6(0, page, buf);
}

static void disp_mode (void)                        // 模式/参数/目标有变化时重刷布局（低频调用）
{
    char line[22];
    const char *name = "IDLE";
    int32 cm10 = 0;

    switch(g_mode)
    {
        case 1: name = "Q1 IDLE";   break;
        case 2: name = "Q2 SPEED";  break;
        case 3: name = "Q3 BALL";   break;
        case 4: name = "Q4 ABSEG";  break;
        case 5: name = "Q5 LAP";    break;
        case 6: name = "Q6 LAPANY"; break;
    }
    sprintf(line, "M%d %s", g_mode, name);
    oled_line16(0, line);

    if(6 == g_mode)
    {
        cm10 = VISION_TARGET_ERR * 10 / 25;
        sprintf(line, "tgt=%s%d.%dcm t=%u.%us", (0 > cm10) ? "-" : "+", (int)((0 > cm10) ? (-cm10 / 10) : (cm10 / 10)),
                (int)((0 > cm10) ? ((-cm10) % 10) : (cm10 % 10)),
                (unsigned int)(g_run_stop_ms / 1000), (unsigned int)((g_run_stop_ms % 1000) / 100));
    }
    else if((2 == g_mode) || (4 == g_mode) || (5 == g_mode))
    {
        sprintf(line, "m%d r%d h%d t=%u.%us", gain_set, RAMP_STEP, FF_RAMP_PERIOD,
                (unsigned int)(g_run_stop_ms / 1000), (unsigned int)((g_run_stop_ms % 1000) / 100));
    }
    else if(3 == g_mode)
    {
        sprintf(line, "O>+5>-5 <5s");
    }
    else
    {
        sprintf(line, "stream show");
    }
    oled_line6(2, line);
}

static void disp_poll (void)                        // 主循环调用：计时区 5Hz 刷新（内容变了才写，省 I2C 带宽）
{
    static uint32 last_ms = 0;
    static uint32 last_shown = 0xFFFFFFFF;
    static uint8  last_code = 0xFF;
    uint32 now = comm_get_ms();
    uint32 show = 0;
    uint8  code = 0;                                // 0=RDY 1=RUN 2=FIN 3=WAIT 4=CD
    char line[22];
    const char *st = "RDY";

    if(200 > (now - last_ms)) { return; }
    last_ms = now;

    if(0 != t3_home)                                // 启动归中阶段：显示 HOMING
    {
        code = 5;
        st = "HOMING";
    }
    else if(0 != t3_run)                            // 题3：t3 计时（state3=FIN 定格总成绩）
    {
        show = (3 == t3_state) ? g_t3_total_ms : (now - t3_t0);
        code = (3 == t3_state) ? 2 : 1;
        st = (3 == t3_state) ? "FIN" : "RUN";
    }
    else if(RUN_STATE_RUNNING == g_run_state)       // 跑车：跑表计时
    {
        show = (uint32)g_run_timer_ms;
        code = 1;
        st = "RUN";
    }
    else if(RUN_STATE_WAIT_TAPE == g_run_state)
    {
        code = 3;
        st = "WAIT TAPE";
    }
    else if(RUN_STATE_COUNTDOWN == g_run_state)
    {
        code = 4;
        st = "RDY-GO";
    }
    else                                            // IDLE：有成绩定格 FIN，无成绩 RDY
    {
        if(0 != g_run_last_ms)
        {
            show = g_run_last_ms;
            code = 2;
            st = "FIN";
        }
        else if(0 != g_t3_total_ms)
        {
            show = g_t3_total_ms;
            code = 2;
            st = "FIN";
        }
    }

    if((show != last_shown) || (code != last_code))
    {
        last_shown = show;
        last_code = code;
        sprintf(line, "T=%u.%02u", (unsigned int)(show / 1000), (unsigned int)((show % 1000) / 10));
        oled_line16(4, line);
        oled_line16(6, st);
    }
}

static void key_poll (void)                         // 主循环周期调用：四按键扫描（B0/B1 选题 A30 启停 A31 调目标）
{
    // ---- B1 模式+ / B0 模式-（空闲才切，运行中防误触）----
    if(key_scan(1, KEY_MODE_UP) && (0 == t3_run) && (RUN_STATE_IDLE == g_run_state))
    {
        g_mode++;
        if(6 < g_mode) { g_mode = 1; }
        mode_apply(g_mode);
        printf("KEY: MODE %d (m%d r=%d h=%d t=%u)\r\n",
               g_mode, gain_set, RAMP_STEP, FF_RAMP_PERIOD, (unsigned int)g_run_stop_ms);
        beep(g_mode);
        disp_mode();
    }
    if(key_scan(2, KEY_MODE_DN) && (0 == t3_run) && (RUN_STATE_IDLE == g_run_state))
    {
        if(1 == g_mode) { g_mode = 6; }
        else            { g_mode--; }
        mode_apply(g_mode);
        printf("KEY: MODE %d (m%d r=%d h=%d t=%u)\r\n",
               g_mode, gain_set, RAMP_STEP, FF_RAMP_PERIOD, (unsigned int)g_run_stop_ms);
        beep(g_mode);
        disp_mode();
    }

    // ---- A31 题6 目标位置步进（+25px≈+1cm/次，>+150 回 -150；串口 l 同步）----
    if(key_scan(3, KEY_TGT) && (0 == t3_run) && (RUN_STATE_IDLE == g_run_state))
    {
        VISION_TARGET_ERR += 25;
        if(150 < VISION_TARGET_ERR) { VISION_TARGET_ERR = -150; }
        printf("KEY: vision target = %+d px (~%+.1fcm)\r\n",
               VISION_TARGET_ERR, VISION_TARGET_ERR / 25.0);
        beep(1);
        disp_mode();
    }

    // ---- A30 启动/急停 ----
    if(key_scan(0, KEY_PIN))
    {
        if((0 != t3_run) || (RUN_STATE_IDLE != g_run_state))
        {
            run_stop();
            t3_stop();
            printf("KEY: emergency stop\r\n");
            beep(2);
            disp_mode();
        }
        else
        {
            printf("KEY: start MODE %d\r\n", g_mode);
            beep(1);
            mode_start(g_mode);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 主循环解析 `x=123`：p/i/d/e/b 对应五个调参量，改完回显
//-------------------------------------------------------------------------------------------------------------------
static void tuning_parse (void)
{
    char buf[16];
    uint8 i = 0;
    int32 val = 0;
    uint32 primask = 0;

    if(!cmd_ready)
    {
        return;
    }
    primask = __get_PRIMASK();                  // 拷贝期间屏蔽，防 ISR 改缓冲
    __disable_irq();
    for(i = 0; i < 16; i++)
    {
        buf[i] = cmd_buf[i];
    }
    cmd_len = 0;
    cmd_ready = 0;
    __set_PRIMASK(primask);

    if('w' == buf[0])                           // 舵机步进：w+ / w-（±10µs），标定边调边看机械
    {
        if('+' == buf[1])
        {
            servo_set_us(servo_get_us() + 10);
        }
        else if('-' == buf[1])
        {
            servo_set_us(servo_get_us() - 10);
        }
        else
        {
            printf("bad cmd (want w+ / w-)\r\n");
            return;
        }
        printf("servo us=%d\r\n", servo_get_us());
        return;
    }
    if('K' == buf[0])                           // I2C 总线扫描（六路模块排障用）
    {
        sensor_6ch_i2c_scan();
        return;
    }
    if('=' != buf[1])
    {
        printf("bad cmd (want x=123)\r\n");
        return;
    }
    {
        int32 sign = 1;
        i = 2;
        if('-' == buf[i])
        {
            sign = -1;
            i++;
        }
        for(; ('0' <= buf[i]) && ('9' >= buf[i]); i++)
        {
            val = val * 10 + (buf[i] - '0');
        }
        val = val * sign;
    }
    switch(buf[0])
    {
        case 'p':   SPD_KP[gain_set]  = val; break;
        case 'i':   SPD_KI[gain_set]  = val; break;
        case 'v':   SPD_KD[gain_set]  = val; break;   // 速度环 D（measurement 微分）
        case 'n':   FF_CORNER_DIFF[gain_set] = val; break;   // 弯道前馈差速（先看轮子后看循迹）
        case 'd':   LINE_KP[gain_set] = val; break;
        case 'e':   LINE_KD[gain_set] = val; break;
        case 'q':   LINE_KQ[gain_set] = val; break;
        case 'b':   BASE_RPM[gain_set] = val; break;
        case 's':   SLOW_KP[gain_set]  = val; break;
        case 'c':   CORNER_RPM[gain_set] = val; break;
        case 'r':   RAMP_STEP = val; break;
        case 'h':                                   // 弯道前馈缓坡周期（单位 10ms/步）
            if(1 > val) { val = 1; }
            if(100 < val) { val = 100; }
            FF_RAMP_PERIOD = val;
            break;
        case 'f':                                   // pos 滤波 ×0.1（1~10，越大越灵）
            if(1 > val) { val = 1; }
            if(10 < val) { val = 10; }
            POS_EMA_NEW[gain_set] = val;
            break;
        case 'g':                                   // 转向 diff 滤波 ×0.1（1~10，越大越灵）
            if(1 > val) { val = 1; }
            if(10 < val) { val = 10; }
            DIFF_LPF_NUM[gain_set] = val;
            break;
        case 'A':                                   // 纵向加速度前馈（毫 us per rpm差分；题4/5/6 球稳，0=关闭）
            ACC_FF = val;
            printf("ACC_FF = %d\r\n", ACC_FF);
            return;
        case 'a':                                   // 视觉 PID P 增益（us/像素或 us/cm），a=0 关闭视觉伺服
            SERVO_KP = val;
            printf("SERVO_KP = %d\r\n", SERVO_KP);
            return;
        case 'j':                                   // 视觉 PID I 增益
            SERVO_KI = val;
            printf("SERVO_KI = %d\r\n", SERVO_KI);
            return;
        case 'k':                                   // 视觉 PID D 增益
            SERVO_KD = val;
            printf("SERVO_KD = %d\r\n", SERVO_KD);
            return;
        case 'Z':                                   // 静摩擦破粘脉冲幅度（us），大写 Z 避免与 qmm01 的 z 冲突
            STICTION_KICK = val;
            printf("STICTION_KICK = %d\r\n", STICTION_KICK);
            return;
        case 'G':                                   // 视觉 PID 输出限幅 us（远目标段）
            DELTA_LIMIT = val;
            printf("DELTA_LIMIT = %d\r\n", DELTA_LIMIT);
            return;
        case 'H':                                   // 视觉 PID 近目标限幅 us（|err|<60 段）
            DELTA_NEAR = val;
            printf("DELTA_NEAR = %d\r\n", DELTA_NEAR);
            return;
        case 'D':                                   // 抗静摩擦颤振幅度 px（0=关闭）
            DITHER_PX = val;
            printf("DITHER_PX = %d\r\n", DITHER_PX);
            return;
        case 'U':                                   // 颤振半周期 ms
            DITHER_MS = val;
            printf("DITHER_MS = %d\r\n", DITHER_MS);
            return;
        case 'N':                                   // 近段 KP（|err|<60）
            KP_NEAR = val;
            printf("KP_NEAR = %d\r\n", KP_NEAR);
            return;
        case 'E':                                   // 负方向 KP 放大倍数 %（100=对称）
            KP_NEG_SCALE = val;
            printf("KP_NEG_SCALE = %d\r\n", KP_NEG_SCALE);
            return;
        case 'L':                                   // 题3：跟踪 P（毫 us/px）【2026-10-17 改调 T3G_KP，3=1 生效】
            T3G_KP = val;
            printf("T3G_KP = %d (next 3=1)\r\n", T3G_KP);
            return;
        case 'I':                                   // 题3：条件积分【2026-10-17 改调 T3G_KI】
            T3G_KI = val;
            printf("T3G_KI = %d (next 3=1)\r\n", T3G_KI);
            return;
        case 'O':                                   // 题3[开环段1]：加速倾角增量 us（负=往+err加速）
            T3_OL_ACC = val;
            printf("T3_OL_ACC = %d\r\n", T3_OL_ACC);
            return;
        case 'B':                                   // 题3[开环段1]：刹车倾角增量 us（正=反向制动）
            T3_OL_BRK = val;
            printf("T3_OL_BRK = %d\r\n", T3_OL_BRK);
            return;
        case 'S':                                   // 题3[开环段1]：加速时长 ms
            T3_OL_TA_MS = val;
            printf("T3_OL_TA_MS = %d\r\n", T3_OL_TA_MS);
            return;
        case 'T':                                   // 题3[开环段1]：刹车时长 ms
            T3_OL_TB_MS = val;
            printf("T3_OL_TB_MS = %d\r\n", T3_OL_TB_MS);
            return;
        case 'R':                                   // 题3：跟踪 D（毫 us per px/frame）【2026-10-17 改调 T3G_KD】
            T3G_KD = val;
            printf("T3G_KD = %d (next 3=1)\r\n", T3G_KD);
            return;
        case 'Y':                                   // 题3：参考加速度前馈（毫 us per px/ms^2）
            T3_KFF = val;
            printf("T3_KFF = %d\r\n", T3_KFF);
            return;
        case 'M':                                   // 题3：球速 LPF ×0.1（1~10 越大越灵）
            if(1 > val) { val = 1; }
            if(10 < val) { val = 10; }
            T3_VEL_NEW = val;
            printf("T3_VEL_NEW = %d\r\n", T3_VEL_NEW);
            return;
        case 'P':                                   // 题3[段2限速器]：速度上限 px/frame（超速满刹）
            T3_VMAX = val;
            printf("T3_VMAX = %d\r\n", T3_VMAX);
            return;
        case 'Q':                                   // 题3：段2 +5->-5cm 轨迹时长 ms
            T3_T2_MS = val;
            printf("T3_T2_MS = %d\r\n", T3_T2_MS);
            return;
        case 'W':                                   // 题3：输出限幅 us（绕中位 ±）
            T3_DELTA_MAX = val;
            printf("T3_DELTA_MAX = %d\r\n", T3_DELTA_MAX);
            return;
        case 'X':                                   // 题3：+5cm 目标 err px（直设覆盖标定值；方向反了用负值）
            T3_TGT_P5 = val;
            printf("T3_TGT_P5 = %d\r\n", T3_TGT_P5);
            return;
        case 'F':                                   // 题3：-5cm 目标 err px
            T3_TGT_N5 = val;
            printf("T3_TGT_N5 = %d\r\n", T3_TGT_N5);
            return;
        case 'V':                                   // 题3：像素比例 px/cm（直设覆盖标定值）
            if(1 > val) { val = 1; }
            T3_PX_CM = val;
            T3_TGT_P5 = T3_ERR_O + 5 * T3_PX_CM;
            T3_TGT_N5 = T3_ERR_O - 5 * T3_PX_CM;
            printf("T3_PX_CM = %d -> tgt %+d/%+d\r\n", T3_PX_CM, T3_TGT_P5, T3_TGT_N5);
            return;
        case '3':                                   // 题3：3=1 启动 / 3=0 收工 / 3=7 标O点 / 3=8 标+5cm / 3=9 查标定
            if(1 == val)
            {
                t3_seq_start();                     // 【2026-10-16】改走位置序列；原 t3_start() 的开环方案已失效
            }
            else if(0 == val)
            {
                t3_seq = 0; t3_seq_in = 0;          // 停序列
                servo_gain_preset(0);               // 增益切回题4/5/6 中心保持档
                t3_stop();
            }
            else if(7 == val)                           // 标定 O 点：球放中心 O，记录当前 err
            {
                int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;
                if(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX))
                {
                    T3_ERR_O = v_err0;
                    T3_TGT_P5 = T3_ERR_O + 5 * T3_PX_CM;
                    T3_TGT_N5 = T3_ERR_O - 5 * T3_PX_CM;
                    printf("T3 calib O: err=%+d -> tgt %+d/%+d\r\n", T3_ERR_O, T3_TGT_P5, T3_TGT_N5);
                }
                else { printf("T3 calib O FAIL: no valid frame\r\n"); }
            }
            else if(8 == val)                           // 标定 +5cm：球放杆身 +5 刻度，自动算比例和方向（含相机装反情形）
            {
                int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;
                if(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX))
                {
                    int32 ds = (int32)v_err0 - T3_ERR_O;        // 带符号：+5 刻度相对 O 的像素偏移
                    int32 d  = (0 > ds) ? -ds : ds;
                    if(10 > d) { printf("T3 calib +5 FAIL: d=%d px too small (do 3=7 first, then +5cm)\r\n", d); }
                    else
                    {
                        T3_PX_CM  = d / 5;
                        T3_TGT_P5 = v_err0;                     // +5 目标=实测点（方向自动）
                        T3_TGT_N5 = T3_ERR_O - ds;              // -5 目标=镜像点
                        printf("T3 calib +5: err=%+d -> PX_CM=%d tgt %+d/%+d\r\n", v_err0, T3_PX_CM, T3_TGT_P5, T3_TGT_N5);
                    }
                }
                else { printf("T3 calib +5 FAIL: no valid frame\r\n"); }
            }
            else if(9 == val)
            {
                printf("T3 calib: O=%+d PX_CM=%d tgt %+d/%+d | OL acc=%d brk=%d ta=%d tb=%d | KP=%d KD=%d KFF=%d KI=%d VEL_NEW=%d T2=%d W=%d mid=%d\r\n",
                       T3_ERR_O, T3_PX_CM, T3_TGT_P5, T3_TGT_N5, T3_OL_ACC, T3_OL_BRK, T3_OL_TA_MS, T3_OL_TB_MS,
                       T3_KP, T3_KD, T3_KFF, T3_KI, T3_VEL_NEW, T3_T2_MS, T3_DELTA_MAX, SERVO_MID_VISION);
            }
            return;
        case 'l':                                   // 视觉目标误差（像素或 cm），默认 0
            VISION_TARGET_ERR = val;
            printf("VISION_TARGET_ERR = %d\r\n", VISION_TARGET_ERR);
            return;
        case 'C':                                   // 调试：向 K230 发上行模式帧 $C（交叉验证 B15->IO4 链路；C=1 切循迹 C=2 切找球）
            comm_send_mode((uint8)val, 0, 0);
            printf("C frame sent mode=%d\r\n", val);
            return;
        case 'K':                                   // 【2026-10-17 新增】题3 到位判据带宽 px（原写死 26）
            if(1 > val) { val = 1; }
            if(60 < val) { val = 60; }
            t3_q_inband_px = val;
            printf("T3 inband = %d px (%d.%02d cm)\r\n", t3_q_inband_px,
                   t3_q_inband_px / 26, (t3_q_inband_px * 100 / 26) % 100);
            return;
        case 'o':                                   // 视觉闭环舵机中位（标定摆杆水平；题3同样用此中位）
            if(SERVO_ABS_MAX_US < val) { val = SERVO_ABS_MAX_US; }
            if(SERVO_ABS_MIN_US > val) { val = SERVO_ABS_MIN_US; }
            SERVO_MID_VISION = val;
            printf("SERVO_MID = %d us\r\n", SERVO_MID_VISION);
            return;
        case 'u':
            servo_set_us((uint16)val);          // 舵机直设脉宽（标定用），超界由 servo.c 钳位
            printf("servo us=%d\r\n", servo_get_us());
            return;
        case '4':                                   // 题4：4=1 启动匀速直行 / 4=0 停车
            // 未上台验证前只走串口（按键面板 mode 4 仍为占位，守住"模式清空重验制"）
            if(1 == val)
            {
                int16 v_err0 = 0; uint8 v_st0 = 0; uint32 v_ag0 = 0;
                if(!(comm_get_vision(&v_err0, &v_st0, &v_ag0) && (2 == v_st0) && (v_ag0 < VISION_AGE_MAX)))
                {
                    printf("T4 REFUSED: no valid vision frame (check K230/UART)\r\n");
                    beep(3);
                    return;
                }
                t4_run = 1;
                t4_t0 = 0;                          // 重置 1.5s 匀加速计时
                g_run_state = RUN_STATE_RUNNING;    // 直进：题4 不要 2s 倒计时
                g_run_timer_ms = 0;
                g_run_stop_ms = 8000;               // 题4：AB段 1.5m，题纸要求「AB间行驶时间≤8s」，故取满 8.0s
                                                    //（原写 5000 是照代码注释猜的，按题纸只有约 1.09m，到不了 B 点，已纠正）
                motor_enable(1);
                pit_enable(PIT_CTRL_TIMER);
                pit_enable(PIT_STOP_TIMER);
                printf("T4 start: target=%d rpm, ramp 1.5s, stop in %d ms\r\n",
                       (int)t4_target_rpm, (int)g_run_stop_ms);
            }
            else
            {
                t4_run = 0;
                run_stop();                         // 复用急停：刹车 + 关控制环/停车中断
                printf("T4 stop\r\n");
            }
            return;
        case 'x':                                   // 串口启动/停车：x=1 进入等待胶带模式，x=0 强制停车
            if(0 == val) { run_stop(); }
            else           { run_start(); }
            return;
        case 'm':
            if(GAIN_SET_NUM <= val)
            {
                printf("bad set (want m=0 or m=1)\r\n");
                return;
            }
            gain_set = (uint8)val;
            break;
        case 't':                                   // 外挂定时停车时间（ms）：x=1启动后跑够t毫秒即停，t=0禁用
            g_run_stop_ms = (uint32)val;
            printf("PIT stop time = %u ms (0=disable)\r\n", (unsigned int)g_run_stop_ms);
            return;
        case 'z':                                   // 台架模式：z=1 开 / z=0 关（只测速度环，忽略循迹与 line_hold）
            g_bench_mode = (0 == val) ? 0 : 1;
            printf("BENCH mode = %d (1=两轮同目标BASE_RPM,忽略循迹/line_hold; 仅架空用)\r\n",
                   (int)g_bench_mode);
            return;
        case 'y':                                   // 【调试】y=1 开原始字节抓取 / y=2 打印并关 / y=3 B18 引脚电平诊断
            if(2 == val)      { comm_dbg_dump(); }
            else if(3 == val) { comm_dbg_pin(); }
            else if(1 == val) { comm_dbg_raw(1); }
            else              { comm_dbg_raw(0); }
            return;
        case 'J':                                   // 【调试】台架直驱两轮 duty（架空排查电机/K230 耦合）；J=0 停
            if(DUTY_RUN_MAX < val)  { val = DUTY_RUN_MAX; }
            if(-DUTY_RUN_MAX > val) { val = -DUTY_RUN_MAX; }
            g_bench_duty = val;
            motor_enable(1);                        // x=0 会把电机总使能关掉，这里强制打开
            pit_enable(PIT_CTRL_TIMER);             // 并确保控制中断开着（x=0 会关掉它，台架直驱挂在里面）
            pit_enable(PIT_STOP_TIMER);
            printf("BENCH DRIVE duty = %d (J=0 停)\r\n", (int)g_bench_duty);
            return;
        default:    printf("unknown param '%c'\r\n", buf[0]); return;
    }
    printf("set=%d | p=%d i=%d v=%d d=%d e=%d q=%d | b=%d s=%d r=%d c=%d f=%d g=%d n=%d h=%d\r\n",
           gain_set, SPD_KP[gain_set], SPD_KI[gain_set], SPD_KD[gain_set], LINE_KP[gain_set], LINE_KD[gain_set],
           LINE_KQ[gain_set], BASE_RPM[gain_set], SLOW_KP[gain_set], RAMP_STEP, CORNER_RPM[gain_set], POS_EMA_NEW[gain_set], DIFF_LPF_NUM[gain_set], FF_CORNER_DIFF[gain_set], FF_RAMP_PERIOD);
}

#if !LINE_SRC_6CH && !IR7_DIGITAL                  // 只有模拟量七路才编（数字量不用校准，且 ir_adc 不存在）
//-------------------------------------------------------------------------------------------------------------------
// 白底校准：3 秒均值；任一路 offset 超限 = 校准时压线，停机提示重新上电（带病不上岗）
//-------------------------------------------------------------------------------------------------------------------
static void ir_calibrate (void)
{
    uint32 sum[IR_CH_NUM] = {0};
    uint8  i = 0;
    uint16 n = 0;
    uint16 rounds = IR_CAL_MS / IR_CAL_STEP_MS;

    printf("calibrating white base %d ms, keep sensors OFF the line!\r\n", IR_CAL_MS);
    for(n = 0; n < rounds; n++)
    {
        for(i = 0; i < IR_CH_NUM; i++)
        {
            sum[i] += adc_mean_filter_convert(ir_adc[i], IR_FILTER_CNT);
        }
        system_delay_ms(IR_CAL_STEP_MS);
    }
    for(i = 0; i < IR_CH_NUM; i++)
    {
        ir_offset[i] = (uint16)(sum[i] / rounds);
    }
    printf("offset:");
    for(i = 0; i < IR_CH_NUM; i++)
    {
        printf(" %4d", ir_offset[i]);
    }
    printf("\r\n");

    // 赛道上启动允许压线：白底参考取"白通道"均值，压线通道借用白底参考值
    // 只有白通道 <3 路（≥5 路全黑，学不到白底）才真正停机
    uint8  white_cnt = 0;
    uint32 white_sum = 0;

    for(i = 0; i < IR_CH_NUM; i++)
    {
        if(IR_CAL_MAX > ir_offset[i])
        {
            white_cnt++;
            white_sum += ir_offset[i];
        }
    }
    if(3 > white_cnt)
    {
        printf("CAL FAIL: only %d white ch, cannot learn base. cover? power cycle.\r\n", white_cnt);
        while(true)
        {
        }
    }
    uint16 white_ref = (uint16)(white_sum / white_cnt);
    for(i = 0; i < IR_CH_NUM; i++)
    {
        if(IR_CAL_MAX <= ir_offset[i])
        {
            printf("ch%d started ON line (offset=%d), use white_ref=%d\r\n", i + 1, ir_offset[i], white_ref);
            ir_offset[i] = white_ref;                   // 压线通道借用白底参考，正常上岗
        }
    }
}
#endif // !LINE_SRC_6CH

//-------------------------------------------------------------------------------------------------------------------
// 主循环循迹传感任务（10ms 节奏）：读传感器 → 加权位置 ×100 → EMA 平滑 → line_pos_x100
// LINE_SRC_6CH=1：幻尔六路 I2C；LINE_6CH_INTERP=1 亚灯插值（阈值归一化），=0 数字状态加权
// 丢线帧 = 全白/全黑/I2C 失败/黑响应总量过低；两版共用同一套丢线防抖/EMA/保持逻辑
//-------------------------------------------------------------------------------------------------------------------
static void ir_update (void)
{
    static int32 pos_f = 0;                     // EMA 状态（×100 定点）
    uint8  i = 0;

#if LINE_SRC_6CH
    uint8  state = 0;
    uint16 analog[6] = {0};
    uint8  n = 0;
    uint8  lost_frame = 0;
    uint8  hold_frame = 0;                      // 全黑保持帧：pos 冻结不更新，也不算丢线
    int32  pos_raw = 0;
    static const int8 w6[6] = {-5, -3, -1, 1, 3, 5};    // 左负右正（通道1~6，物理序待实测核对）

    if(!sensor_6ch_read(&state, analog))
    {
        lost_frame = 1;                         // I2C 失败：按丢线帧处理，pos 保持最后有效值
    }
    else
    {
        for(i = 0; i < 6; i++)
        {
            s6_analog_dbg[i] = analog[i];
            if(state & (1 << i))
            {
                n++;
            }
        }
        s6_state_dbg = state;
        s6_n_dbg = n;
        {                                       // 终点胶带检测：n≥4路同黑 或 ch1+ch6 同黑（5cm 胶带特征，弯道贴边 n≤2 零误判）；m0 以 PIT 8.00s 为主停车，TAPE6 做双保险
            static uint8 tape6_run = 0, tape6_armed = 0;
            static uint8 lap_timing = 0;        // 圈速自报：离带起表，TAPE6 停车报时（A11 串口版）
            static uint32 lap_ms = 0;
            if(lap_timing) { lap_ms += 10; }    // ir_update 10ms 一拍

            // 只有真正在跑圈时才允许 TAPE6 触发；x=0/待机/倒计时期间绝不误触发或自动再起跑
            if(RUN_STATE_RUNNING != g_run_state)
            {
                tape6_run = 0;
                tape6_armed = 0;                // 每圈重新起跑前必须解除武装，避免上一圈残留的 armed 导致起点误停
            }
            else if((4 <= n) || (0x21 == (state & 0x21)))
            {
                if(tape6_armed && (S6_TAPE6_RUN <= ++tape6_run))
                {
                    if(S6_TAPE6_RUN == tape6_run)   // 只报一次（停胶带上期间每帧都触发，防刷屏）
                    {
                        printf("LAP %u ms | TAPE6 stop: n=%d st=0x%02X\r\n", (unsigned int)lap_ms, n, state);
                    }
                    lap_timing = 0;
                    tape6_armed = 0;                // 触发后立刻解除武装，防止原地循环触发
                    lost_hold_limit = 0;        // 立即停车
                    line_lost = 1;
                    g_run_state = RUN_STATE_IDLE;   // 强制回待机：防止停车后 line_lost 被清除又自动起跑
                    g_run_last_ms = g_run_timer_ms; // 捕获成绩（OLED FIN 显示；跑表自 GO 起计）
                    g_t3_total_ms = 0;              // 最新成绩优先（清掉题3旧成绩）
                    motor_enable(0);                // 硬件层同时关断，双保险
                }
            }
            else
            {
                tape6_run = 0;
                tape6_armed = 1;                // 离开发车胶带（ch6 干净过）才武装，防起点原地自锁
                if(!lap_timing)                 // 离带瞬间起表
                {
                    lap_timing = 1;
                    lap_ms = 0;
                }
            }
        }
        if(0 == n)                              // 全白 = 脱线丢
        {
            lost_frame = 1;
        }
        else if(6 == n)                         // 全黑 = 压启停线/黑块：保持 pos 直接通过（巡环不停车；A 点停车检测后续单做）
        {
            hold_frame = 1;
        }
        else
        {
#if LINE_6CH_INTERP
            if(s6_thr_valid)                    // 亚灯插值（线性版）：d=max(0, analog-thr-死区)，边界按比例分摊不跳崖
            {
                int32 dsum = 0, wsum = 0;

                for(i = 0; i < 6; i++)
                {
                    int32 d = (int32)analog[i] - (int32)s6_thr[i] - S6_D_DEADZONE;
                    if(0 > d)
                    {
                        d = 0;                  // 白底/弱响应进死区归零，只留黑线方向的有效分量
                    }
                    dsum += d;
                    wsum += w6[i] * d;
                }
                if(S6_DSUM_MIN > dsum)
                {
                    lost_frame = 1;             // 黑响应总量过低：线在窗口边缘外，按丢线帧
                }
                else
                {
                    pos_raw = SENSOR_6CH_POS_SIGN * wsum * SENSOR_6CH_SCALE / dsum;   // 连续 ±150，线性滑动（int32 够，d≤4095）
                }
            }
            else                                // 阈值未读到：数字状态加权兜底（台阶版，仅应急）
#endif
            {
                int32 wv_sum = 0;
                for(i = 0; i < 6; i++)
                {
                    if(state & (1 << i))
                    {
                        wv_sum += w6[i];
                    }
                }
                pos_raw = SENSOR_6CH_POS_SIGN * wv_sum * SENSOR_6CH_SCALE / n;    // 均值(-5..+5)×30 → ±150
            }
        }
    }

    if(lost_frame)
    {
        line_hold = 0;
        lost_cnt++;                             // 丢线防抖：连续 LOST_DEBOUNCE_CNT 帧都丢才认，单帧 glitch 无视
        if((LOST_DEBOUNCE_CNT <= lost_cnt) && !line_lost)
        {
            int32 pa = (0 > pos_f) ? -pos_f : pos_f;
            lost_hold_limit = LOST_HOLD_MS;      // 统一 1s：全白超 1s 才停（不再分直道/弯道）
            line_lost = 1;
        }
    }
    else if(!hold_frame)
    {
        line_hold = 0;
        lost_cnt = 0;
        line_lost = 0;
        pos_f = pos_f * (10 - POS_EMA_NEW[gain_set]) / 10 + pos_raw * POS_EMA_NEW[gain_set] / 10;
        line_pos_x100 = pos_f + LINE_POS_BIAS;  // 零点左偏：环路线 pos→0 时物理线在 ch2/ch3（顺时针借灯）
    }
    else                                        // 全黑通过帧：清防抖计数，pos 冻结保持（启停线/黑块不扰环）
    {
        line_hold = 1;                          // 控制环据此限速 HOLD_CRUISE_RPM，防胶带上盲加速
        line_lost = 0;                          // 全黑是强信号，不是丢线；否则长黑胶带训练时 line_lost 永远清不掉，无法起跑
        lost_cnt = 0;
    }
#else
#if IR7_DIGITAL
    //---------------- 七路【数字量】GPIO ----------------------------------------
    // 每路只有 通/断（0/1），位置 = 加权均值 × 50 → ±150（与六路同量纲，下游门槛不用改）
    // 通道压在线上正中时权重 0，所以"只在正中间"时 pos=0，与六路的零点一致
    {
        uint8 n = 0;
        int32 wsum = 0;
        ir7_bits = 0;
        ir7_raw_bits = 0;
        for(i = 0; i < IR_CH_NUM; i++)
        {
            uint8 lv = (uint8)gpio_get_level(ir7_pin[i]);
            if(lv) { ir7_raw_bits |= (uint8)(1 << i); }   // 原始电平：1=引脚高
            uint8 on = IR7_ACTIVE_LOW ? (uint8)(0 == lv) : (uint8)(0 != lv);
            if(on)
            {
                ir7_bits |= (uint8)(1 << i);
                n++;
                wsum += ir7_w[i];
            }
        }
        if(0 == n)                              // 全白 = 丢线
        {
            lost_cnt++;
            if((LOST_DEBOUNCE_CNT <= lost_cnt) && !line_lost)
            {
                int32 pa = (0 > pos_f) ? -pos_f : pos_f;
                lost_hold_limit = LOST_HOLD_MS;      // 统一 1s：全白超 1s 才停（不再分直道/弯道）
                line_lost = 1;
            }
        }
        else if(IR7_TAPE_N <= n)                // 多路同黑 = 起停线/黑块：pos 冻结，不算丢线
        {
            line_hold = 1;
            line_lost = 0;
            lost_cnt = 0;
        }
        else
        {
            int32 pos_raw = IR7_POS_SIGN * wsum * IR7_SCALE / n;
            lost_cnt = 0;
            line_lost = 0;
            line_hold = 0;                      // ★ BUG FIX：正常帧必须清 line_hold
                                                //   原代码只在 n>=IR7_TAPE_N 时置 1，从不清零 ->
                                                //   车压过一次起跑胶带后 line_hold 永久为 1 ->
                                                //   base_eff 被钉在 HOLD_CRUISE_RPM(80)，全程只能跑 80rpm
                                                //   而 BASE_RPM 设 90 也上不去（实测 bs=80 而不是 90）。
            pos_f = pos_f * (10 - POS_EMA_NEW[gain_set]) / 10 + pos_raw * POS_EMA_NEW[gain_set] / 10;
            line_pos_x100 = pos_f + LINE_POS_BIAS;
        }
    }
#else
    uint16 raw = 0;
    int32  v = 0, v_sum = 0, wv_sum = 0;

    for(i = 0; i < IR_CH_NUM; i++)
    {
        raw = adc_mean_filter_convert(ir_adc[i], IR_FILTER_CNT);
        v = (int32)ir_offset[i] - (int32)raw;
        if(0 > v)
        {
            v = -v;                             // 黑线响应量取绝对值，模块极性自适应
        }
        v_sum  += v;
        wv_sum += IR_POS_SIGN * ir_w[i] * v;
    }
    if(IR_LOST_SUM > v_sum)
    {
        lost_cnt++;                             // 丢线防抖：连续 LOST_DEBOUNCE_CNT 帧都丢才认，单帧 glitch 无视
        if((LOST_DEBOUNCE_CNT <= lost_cnt) && !line_lost)
        {
            int32 pa = (0 > pos_f) ? -pos_f : pos_f;
            lost_hold_limit = LOST_HOLD_MS;      // 统一 1s：全白超 1s 才停（不再分直道/弯道）
            line_lost = 1;
        }
    }
    else
    {
        int32 pos_raw = wv_sum * 100 / v_sum;
        lost_cnt = 0;
        line_lost = 0;
        pos_f = pos_f * (10 - POS_EMA_NEW[gain_set]) / 10 + pos_raw * POS_EMA_NEW[gain_set] / 10;
        line_pos_x100 = pos_f;
    }
#endif
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 开机电机极性自检：左右各 +3000 转 1s，打印两路编码器增量符号
// 判读：被驱动轮的通道应有明显增量；符号应为正（定案 ENC_x_SIGN 双 +1，前进 = 双正）
//-------------------------------------------------------------------------------------------------------------------
static void self_check (void)
{
    int32 l0 = 0, r0 = 0;

    printf("!!! self-check: wheels WILL spin, hold the car !!!\r\n");
    system_delay_ms(1000);

    printf("L motor +%d 1s...\r\n", SELFCHK_DUTY);
    l0 = enc_l_total;
    r0 = enc_r_total;
    motor_set(MOTOR_CH_L, SELFCHK_DUTY);
    system_delay_ms(1000);
    motor_brake(MOTOR_CH_L);
    printf("  dTotal: L=%+d R=%+d\r\n", enc_l_total - l0, enc_r_total - r0);
    system_delay_ms(500);

    printf("R motor +%d 1s...\r\n", SELFCHK_DUTY);
    l0 = enc_l_total;
    r0 = enc_r_total;
    motor_set(MOTOR_CH_R, SELFCHK_DUTY);
    system_delay_ms(1000);
    motor_brake(MOTOR_CH_R);
    printf("  dTotal: L=%+d R=%+d\r\n", enc_l_total - l0, enc_r_total - r0);
    printf("self-check done. expect: driven ch counts POSITIVE.\r\n\r\n");
}

int main (void)
{
    uint8  i = 0;
    uint16 loop_cnt = 0;

    clock_init(SYSTEM_CLOCK_80M);               // 时钟配置及系统初始化<务必保留>
    debug_init();                               // 调试串口初始化（UART0: TX=A10 RX=A11, 115200）
    motor_init();                               // 最优先接管电机脚为刹车态：复位浮空窗口 TB6612 输入乱飘会上电猛转，越短越好

    gpio_init(LED_PIN, GPO, 0, GPO_PUSH_PULL);  // A14 钉低：与蜂鸣器同脚，禁翻转
    gpio_init(KEY_PIN, GPI, 0, GPI_PULL_UP);    // A30 启动/急停键（上拉；按键模块未按=高、按下=低）
    gpio_init(KEY_MODE_UP, GPI, 0, GPI_PULL_UP);  // B1 模式+
    gpio_init(KEY_MODE_DN, GPI, 0, GPI_PULL_UP);  // B0 模式-
    gpio_init(KEY_TGT, GPI, 0, GPI_PULL_UP);      // A31 题6目标位置步进
    oled_i2c_init();                                // SSD1306 OLED 计时屏（B6/B5；无屏也不死机）
    servo_init();                               // 先电机后舵机：上电输出中位脉宽，防猛冲
    comm_init();                                // K230 视觉下行帧接收（UART2 B15/B16, 115200）

#if !VISION_TEST_MODE
#if LINE_SRC_6CH
    sensor_6ch_init();                          // 六路 I2C 循迹模块（校准走板载 KEY，小土按）
#elif IR7_DIGITAL
    // 七路数字：7 个脚全做上拉输入（模块多为此类输出；拉线断开时读到高=白，不会误判压线）
    {
        uint8 k = 0;
        for(k = 0; k < IR_CH_NUM; k++)
        {
            gpio_init(ir7_pin[k], GPI, 0, GPI_PULL_UP);
        }
        printf("IR7 digital in: B17 A17 A16 B20 B25 A24 A25 (active %s)\r\n",
               IR7_ACTIVE_LOW ? "LOW" : "HIGH");
    }
#endif
#if LINE_SRC_6CH
#if LINE_6CH_INTERP
    {                                           // 读学习阈值：合法性校验（100~4000），垃圾值/失败重读 5 次
        uint8 retry = 0, sane = 0;              // （7/30 实案：总线撞车读出 65535/33315，四路被掐零直道全废）
        for(retry = 0; retry < 5; retry++)
        {
            system_delay_ms(100);
            s6_thr_valid = sensor_6ch_read_thr(s6_thr);
            if(s6_thr_valid)
            {
                sane = 1;
                for(i = 0; i < 6; i++)
                {
                    if((0 != s6_thr[i]) && ((100 > s6_thr[i]) || (4000 < s6_thr[i])))
                    {
                        sane = 0;               // 非 0 但越界 = 垃圾值，重读
                        break;
                    }
                }
                if(sane) { break; }
            }
        }
        if(!sane)
        {
            s6_thr_valid = 0;                   // 5 次都读坏：回退数字加权版（仍能跑，台阶版应急）
            printf("6ch thr: READ FAIL x5, fallback to digital mode!\r\n");
        }
    }
    if(s6_thr_valid)                            // 模块学习只存部分通道阈值（短按瞬采，没扫到的路存 0）
    {                                           // → 0 值路用有效路均值顶替，防白底响应被当成黑
        uint8 zcnt = 0, vcnt = 0;
        uint16 vsum = 0;
        for(i = 0; i < 6; i++)
        {
            if(0 == s6_thr[i]) { zcnt++; }
            else               { vcnt++; vsum += s6_thr[i]; }
        }
        if((0 != zcnt) && (0 != vcnt))
        {
            uint16 vmean = vsum / vcnt;
            printf("6ch thr: %d ch zero, fill with mean %d\r\n", zcnt, vmean);
            for(i = 0; i < 6; i++)
            {
                if(0 == s6_thr[i]) { s6_thr[i] = vmean; }
            }
        }
    }
#endif
#elif !IR7_DIGITAL
    // 只有【模拟量】七路才需要初始化 ADC；数字量已在上面 gpio_init 过
    for(i = 0; i < IR_CH_NUM; i++)
    {
        adc_init(ir_adc[i], ADC_12BIT);
    }
#endif

    printf("\r\n=== A04 line-follow PID (restored, final signs) ===\r\n");
    printf("enc: 4x state-machine, L=B10/B9 R=B11/B7, 1560/rev, fwd=+\r\n");
#if LINE_SRC_6CH
    printf("line sensor: 6ch I2C 0x5C (SCL=B8 SDA=B26), press module KEY to learn\r\n");
#if LINE_6CH_INTERP
    printf("6ch interp: thr_valid=%d thr=%d %d %d %d %d %d\r\n",
           s6_thr_valid, s6_thr[0], s6_thr[1], s6_thr[2], s6_thr[3], s6_thr[4], s6_thr[5]);
#endif
#endif
    printf("tune: x=123 t=ms (p i d e q=当前档增益 b=BASE s=SLOW_KP r=RAMP c=CORNER f=pos滤波 g=转向滤波 n=前馈 h=前馈缓坡周期 m=0/1 切档 z=台架模式 J=duty台架直驱)\r\n");
    printf("servo: u=1500 直设脉宽, w+ / w- 步进 ±10us (视觉闭环调试中暂不启用)\r\n");
    printf("T3(段1开环bang-bang+段2五次PD): 3=1启动 3=0收工 3=7标O 3=8标+5 3=9查参 | O=加速us B=刹车us S=加速ms T=刹车ms | L=KP R=KD Y=KFF I=KI M=滤波 Q=T2 W=限幅 X/F=目标 V=px/cm\r\n");
    printf("MODE按键: B1/B0=选题1~6(蜂鸣报数) A30=启动/急停 A31=题6目标步进 | 1=怠速图传 2=题2速度圈 3=题3滚球 4=题4AB段 5=题5整圈 6=题6任意位\r\n\r\n");
    disp_mode();                                // OLED 首屏：当前模式布局

#if !LINE_SRC_6CH && !IR7_DIGITAL
    ir_calibrate();                             // 只有【模拟量】七路才需要白底 offset 校准；数字量无阈值，跳过
#endif

    exti_init(ENC_L_A, EXTI_TRIGGER_BOTH, enc_l_a_callback, NULL);
    exti_init(ENC_L_B, EXTI_TRIGGER_BOTH, enc_l_b_callback, NULL);
    exti_init(ENC_R_A, EXTI_TRIGGER_BOTH, enc_r_a_callback, NULL);
    exti_init(ENC_R_B, EXTI_TRIGGER_BOTH, enc_r_b_callback, NULL);
    enc_l_state = (uint8)((gpio_get_level(ENC_L_A) << 1) | gpio_get_level(ENC_L_B));
    enc_r_state = (uint8)((gpio_get_level(ENC_R_A) << 1) | gpio_get_level(ENC_R_B));
#endif

    uart_set_callback(UART_0, tuning_rx_callback, NULL);

    printf("ready in 3s, keep car still...\r\n\r\n");
    system_delay_ms(3000);

#if !VISION_TEST_MODE
    pit_ms_init(PIT_ENC_TIMER, PIT_PERIOD_MS, pit_enc_callback, NULL);
    // self_check();                            // 极性已定案，开机不再转轮自检（留函数备查）

    pit_ms_init(PIT_CTRL_TIMER, PIT_PERIOD_MS, pit_ctrl_callback, NULL);
    pit_ms_init(PIT_STOP_TIMER, PIT_PERIOD_MS, pit_stop_callback, NULL);    // 外挂定时停车中断
    // 【2026-10-16】开机显式套一次中心档预设，不依赖"有没有敲过 x=1 / 3=0"。
    //   踩过的坑：静态测试脚本只发 x=0/J=0/A=0，从没触发 servo_gain_preset(0)，
    //   于是跑的是文件初值（当时 STICTION_KICK=0 / KD=-18000），破粘一次都没生效，
    //   表现为"球永远停在中点旁 1cm 回不来"，查了半天。初值已同步，这里再兜一道。
    servo_gain_preset(0);
    printf("center preset applied: kp=%d ki=%d kd=%d kick=%d G=%d H=%d\r\n\r\n",
           SERVO_KP, SERVO_KI, SERVO_KD, STICTION_KICK, DELTA_LIMIT, DELTA_NEAR);
#else
    printf("VISION TEST MODE ON: servo + vision PID only, motors/line/encoders disabled\r\n\r\n");
#endif

    while(true)
    {
        // 注意：ir_update() 已移入 pit_ctrl_callback（10ms 中断）固定 100Hz 采样。
        //       不要再放回主循环 —— 主循环在电机转动时只有 4.5Hz，会让丢线检测慢到 2 秒。
        loop_tick++;                                // 主循环计数（保留供诊断）
        tuning_parse();
        key_poll();                                 // 四按键：B1/B0 选题 A30 启停 A31 调目标
        disp_poll();                                // OLED 计时区 5Hz 刷新
        // comm_poll() 已搬进 pit_ctrl_callback（10ms 中断）——主循环电机转动时只有 9.7Hz，会丢 3/4 视觉帧

        // ---------- 题3（2026-08-01 重写版：五次多项式轨迹 + PD 跟踪）+ 中点锚定视觉 PID ----------
        // 舵机所有权二选一：t3_run=1 → 题3轨迹控制器独占；t3_run=0 → 锚定 PID（题4/5/6 用）
        {
            int16  v_err = 0;
            uint8  v_status = 0;
            uint32 v_age = 0;
            static int32 v_err_last = 0;
            static int32 v_err_integral = 0;
            static int32 stuck_cnt = 0;
            static int32 kick_hold = 0;
            static int32 err_diff_hold = 0;                 // 帧级差分保持值（新帧才刷新）
            static uint32 frame_ms_last = 0;                // 上次差分时的帧时刻
            static int32  t3_x_last = 0;                    // [题3] 上一帧球位（速度差分）
            static uint32 t3_fms_last = 0;                  // [题3] 上一帧时刻
            static int32  t3_fdt = 23;                      // [题3] 实测帧周期 ms（默认≈43fps）
            static uint32 t3_lost0 = 0;                     // [题3] 视觉丢失起始时刻
            uint8  v_valid = comm_get_vision(&v_err, &v_status, &v_age)
                             && (2 == v_status) && (v_age < VISION_AGE_MAX);

            // ==================== 启动归中状态机（A30 后先把球驱到目标位）====================
            if((0 != t3_home) && (0 == t3_run))
            {
                uint32 now_ms = comm_get_ms();
                int32  e = (int32)v_err - VISION_TARGET_ERR;

                if(!v_valid)
                {
                    // 视觉丢失：舵机回中位等待，状态机不推进
                    servo_set_us((uint16)SERVO_MID_VISION);
                }
                else if(1 == t3_home)
                {
                    // 阶段1：球在远端(|e|>60) → ±400us 大力往回扫；进 ±60 区或扫满 800ms → 阶段2
                    if((60 < e) || (-60 > e))
                    {
                        int32 us = (0 < e) ? ((int32)SERVO_MID_VISION + 400) : ((int32)SERVO_MID_VISION - 400);
                        if(SERVO_ABS_MAX_US < us) { us = SERVO_ABS_MAX_US; }
                        if(SERVO_ABS_MIN_US > us) { us = SERVO_ABS_MIN_US; }
                        servo_set_us((uint16)us);
                        if(800 <= (int32)(now_ms - t3_home_ms))
                        {
                            t3_home = 2;
                            t3_home_ms = now_ms;
                        }
                    }
                    else
                    {
                        t3_home = 2;
                        t3_home_ms = now_ms;
                    }
                }
                else
                {
                    // 阶段2：锚定 PID 接管（下方锚定块输出）；到位=进带 ±15px 稳 300ms；2s 超时兜底启动
                    if((15 > e) && (-15 < e))
                    {
                        if(0 == t3_home_inband) { t3_home_inband = now_ms; }
                    }
                    else
                    {
                        t3_home_inband = 0;
                    }
                    if(((0 != t3_home_inband) && (300 <= (int32)(now_ms - t3_home_inband)))
                       || (2000 <= (int32)(now_ms - t3_home_ms)))
                    {
                        uint8 m = t3_home_next;
                        t3_home = 0;
                        if(0 == t3_home_inband) { printf("HOME TIMEOUT, start anyway\r\n"); }
                        printf("HOME done, start MODE %d\r\n", m);
                        if(3 == m) { t3_start(); }
                        else       { run_start(); }
                    }
                }
            }

            if(t3_run)
            {
                // ==================== 题3 轨迹跟踪控制器 ====================
                uint32 now_ms = comm_get_ms();              // 真实毫秒时基（comm 1ms PIT）

                if(!v_valid)
                {
                    // 视觉丢失：冻结舵机（保持上一输出），连丢超时收工回中位
                    if(0 == t3_lost0) { t3_lost0 = now_ms; }
                    if(T3_LOST_ABORT <= (int32)(now_ms - t3_lost0))
                    {
                        t3_run = 0;
                        t3_state = 0;
                        servo_set_us((uint16)SERVO_MID_VISION);
                        printf("T3 ABORT: vision lost >%d ms, back to mid\r\n", T3_LOST_ABORT);
                    }
                }
                else
                {
                    int32 seg_T, ref1, refv = 0, refa = 0, ela;
                    t3_lost0 = 0;
                    t3_x = v_err;

                    // 球速估计：新帧到达才差分 + LPF（px/frame）
                    {
                        uint32 fms = comm_get_frame_ms();
                        if(fms != t3_fms_last)
                        {
                            if(0 != t3_fms_last)
                            {
                                int32 fdt = (int32)(fms - t3_fms_last);
                                if((1 <= fdt) && (200 >= fdt)) { t3_fdt = fdt; }
                                {
                                    int32 vraw = (int32)v_err - t3_x_last;
                                    if(T3_VEL_CLAMP < vraw)  { vraw = T3_VEL_CLAMP; }
                                    if(-T3_VEL_CLAMP > vraw) { vraw = -T3_VEL_CLAMP; }
                                    t3_vel = (t3_vel * (10 - T3_VEL_NEW) + vraw * T3_VEL_NEW) / 10;
                                }
                            }
                            t3_fms_last = fms;
                            t3_x_last = v_err;
                        }
                    }

                    if((1 == t3_state) || (4 == t3_state))
                    {
                        // ======== 段1开环 bang-bang（小土定案：固定倾角固定时长，重复性优先）========
                        // state1=加速段（T3_OL_ACC×T3_OL_TA_MS）→ state4=刹车段（T3_OL_BRK×T3_OL_TB_MS）→ 段2闭环
                        int32 d    = (1 == t3_state) ? T3_OL_ACC : T3_OL_BRK;
                        int32 tlim = (1 == t3_state) ? T3_OL_TA_MS : T3_OL_TB_MS;
                        int32 us   = (int32)SERVO_MID_VISION + d;
                        ela = (int32)(now_ms - t3_seg0);
                        if(SERVO_ABS_MAX_US < us) { us = SERVO_ABS_MAX_US; }
                        if(SERVO_ABS_MIN_US > us) { us = SERVO_ABS_MIN_US; }
                        servo_set_us((uint16)us);
                        t3_ref = (1 == t3_state) ? us : t3_ref;     // 打印占位（复用 ref 栏显示直写 us）

                        // 极端位置记录（加速+刹车两段，评审判 +5 到达误差；方向随标定自适应）
                        if(((T3_TGT_P5 >= T3_ERR_O) && (v_err > t3_extreme)) ||
                           ((T3_TGT_P5 <  T3_ERR_O) && (v_err < t3_extreme)))
                        {
                            t3_extreme = v_err;
                        }

                        if(ela >= tlim)
                        {
                            if(1 == t3_state)
                            {
                                t3_state = 4;                       // 加速完进刹车段
                                t3_seg0 = now_ms;
                            }
                            else
                            {
                                t3_state = 2;                       // 刹车完进段2闭环
                                t3_seg0 = now_ms;
                                t3_ref0 = v_err;                    // 段2轨迹起点=刹车末实测位（连续不跳变）
                                t3_reach_ms = now_ms - t3_t0;       // 折返时刻=段1加速+刹车总时长
                                printf("T3 rev @%ums x=%+d\r\n", (unsigned int)t3_reach_ms, v_err);
                            }
                        }
                    }
                    else
                    {
                    // ---- 参考轨迹：五次多项式 s(τ)=10τ³-15τ⁴+6τ⁵（两端速度/加速度=0，到位末速≈0）----
                    seg_T = (1 == t3_state) ? T3_T1_MS : T3_T2_MS;
                    ref1  = (1 == t3_state) ? T3_TGT_P5 : T3_TGT_N5;
                    ela   = (int32)(now_ms - t3_seg0);
                    if(3 == t3_state)
                    {
                        t3_ref = T3_TGT_N5;
                    }
                    else
                    {
                        int64 tm = (int64)ela * 1000 / seg_T;           // τ×1000
                        int64 t2, t3, t4, t5;
                        int32 s_milli, sv_milli, sa_milli, dref;
                        if(1000 < tm) { tm = 1000; }
                        t2 = tm * tm;                                   // ≤1e6
                        t3 = t2 * tm;                                   // ≤1e9
                        t4 = t3 * tm;                                   // ≤1e12
                        t5 = t4 * tm;                                   // ≤1e15
                        s_milli  = (int32)(10*t3/1000000 - 15*t4/1000000000 + 6*t5/1000000000000);
                        sv_milli = (int32)(30*t2/1000 - 60*t3/1000000 + 30*t4/1000000000);   // s'(τ)×1000
                        sa_milli = (int32)(60*tm - 180*t2/1000 + 120*t3/1000000);            // s''(τ)×1000
                        dref   = ref1 - t3_ref0;
                        t3_ref = t3_ref0 + (int32)((int64)dref * s_milli / 1000);
                        refv   = (int32)((int64)dref * sv_milli * t3_fdt / ((int64)seg_T * 1000));
                        refa   = (int32)((int64)dref * sa_milli * 1000 / ((int64)seg_T * seg_T));
                    }

                    // ---- PD 跟踪 + 加速度前馈 + 条件积分（符号实测：us 低于中位 → 球往 err 正方向走）----
                    {
                        static int32 t3_eint = 0;                   // 条件积分累加（px·拍）
                        int32 e  = t3_ref - (int32)v_err;
                        int32 ev = refv - t3_vel;
                        int32 delta, us;
                        // 条件积分：只在近目标(|e|<60)积累，远段清零防饱和；切段时由状态机清零
                        if((T3_KI_BAND > e) && (-T3_KI_BAND < e))
                        {
                            t3_eint += e;
                            if(T3_KI_MAX < t3_eint)  { t3_eint = T3_KI_MAX; }
                            if(-T3_KI_MAX > t3_eint) { t3_eint = -T3_KI_MAX; }
                        }
                        else
                        {
                            t3_eint = 0;
                        }
                        delta = (int32)(-((int64)T3_KP * e + (int64)T3_KD * ev + (int64)T3_KI * t3_eint + (int64)T3_KFF * refa) / 1000);
                        // 段2限速器（小土定案）：球速朝 -err 超 T3_VMAX 即满刹——过中线必刹，治 -5 刹不住
                        if((2 == t3_state) && (t3_vel < -T3_VMAX))
                        {
                            delta = -T3_DELTA_MAX;
                        }
                        if(delta > T3_DELTA_MAX)  { delta = T3_DELTA_MAX; }
                        if(delta < -T3_DELTA_MAX) { delta = -T3_DELTA_MAX; }
                        us = (int32)SERVO_MID_VISION + delta;
                        if(SERVO_ABS_MAX_US < us) { us = SERVO_ABS_MAX_US; }
                        if(SERVO_ABS_MIN_US > us) { us = SERVO_ABS_MIN_US; }
                        servo_set_us((uint16)us);
                    }

                    // ---- 到位判据（仅段2）：轨迹走完 + 进带(±0.6cm) + 低速持续认定；超时兜底进保持 ----
                    if(2 == t3_state)
                    {
                        int32 band = T3_PX_CM * T3_BAND_X100 / 100;
                        int32 dtgt = (int32)v_err - ref1;
                        if(8 > band) { band = 8; }
                        if(0 > dtgt) { dtgt = -dtgt; }
                        if((band >= dtgt) && (T3_VEL_STILL >= t3_vel) && (-T3_VEL_STILL <= t3_vel) && (ela >= seg_T))
                        {
                            if(0 == t3_inband0) { t3_inband0 = now_ms; }
                        }
                        else
                        {
                            t3_inband0 = 0;
                        }

                        if(((0 != t3_inband0) && (T3_INBAND2_MS <= (int32)(now_ms - t3_inband0)))
                           || ((T3_T2_MS + T3_SEG2_TIMEOUT) <= ela))
                        {
                            // 进入 -5 保持段，出评分报告（cm×100 定标打印）
                            int32 ex_cm, fx_cm, ex_a, fx_a;
                            uint32 total;
                            t3_state = 3;
                            total = now_ms - t3_t0;
                            g_t3_total_ms = total;          // OLED FIN 显示用
                            g_run_last_ms = 0;              // 最新成绩优先（清掉跑车旧成绩）
                            ex_cm = (t3_extreme - T3_ERR_O) * 100 / T3_PX_CM;
                            fx_cm = ((int32)v_err - T3_ERR_O) * 100 / T3_PX_CM;
                            ex_a = (0 > ex_cm) ? -ex_cm : ex_cm;
                            fx_a = (0 > fx_cm) ? -fx_cm : fx_cm;
                            printf("T3 REPORT: +5 reach=%ums extreme=%s%d.%02dcm | -5 settled x=%s%d.%02dcm | total=%ums %s\r\n",
                                   (unsigned int)t3_reach_ms,
                                   (0 > ex_cm) ? "-" : "+", (int)(ex_a / 100), (int)(ex_a % 100),
                                   (0 > fx_cm) ? "-" : "+", (int)(fx_a / 100), (int)(fx_a % 100),
                                   (unsigned int)total,
                                   (5000 >= total) ? "TIME-OK(<=5s)" : "TIME-OVER(>5s)");
                        }
                    }
                    }   // end 段2/保持（闭环）分支
                }
            }

            // ---------- 视觉→舵机 PID 闭环（题3位置序列专用） ----------
            // 【2026-10-16】加 (0 != t3_seq)：只有题3的位置序列（O->+5->-5）还留在主循环，
            //   题4/5/6 的中点/任意位保球已整条搬进 pit_ctrl_callback（vision_hold_isr），
            //   否则两处会同时写舵机打架。题3的 gain/kick 时序一行未动，保证已通过的 4/4 不被影响。
            if((!t3_run) && (1 != t3_home) && (0 != t3_seq))
            {
                if(v_valid && ((0 != SERVO_KP) || (0 != SERVO_KI) || (0 != SERVO_KD)))
                {
                    // 抗静摩擦颤振：仅中点锚定（target=0 且非题3）时目标叠加方波，
                    // 球在 ±DITHER_PX 间不停滚动，静摩擦失效，平均位置≈中点
                    int32 tgt = VISION_TARGET_ERR;

                    // ---------- 题3 位置序列：接管目标，自动判定到位 / 折返 / 计时 ----------
                    if(0 != t3_seq)
                    {
                        tgt = (1 == t3_seq) ? T3_TGT_P5 : T3_TGT_N5;    // seq=3 时保持停在 -5cm
                    }
                    if((1 == t3_seq) && ((int32)v_err > t3_seq_ext))    // 【2026-10-17】记录去+5 的真实峰值
                    {
                        t3_seq_ext = (int32)v_err;
                    }
                    if((1 == t3_seq) || (2 == t3_seq))                  // 只有"去+5/去-5"两态才判定到位
                    {
                        uint32 now3 = comm_get_ms();
                        int32  e3 = (int32)v_err - tgt;
                        if((t3_q_inband_px >= e3) && (-t3_q_inband_px <= e3))
                        {
                            if(0 == t3_seq_in)
                            {
                                t3_seq_in = now3;                       // 刚进带，开始计时
                            }
                            else if(T3_Q_INBAND_MS <= (int32)(now3 - t3_seq_in))
                            {
                                if(1 == t3_seq)                         // +5cm 到位 -> 折返
                                {
                                    t3_seq_t1 = (int32)(now3 - t3_seq_t0);
                                    printf("T3Q: +5cm OK @%ums (err=%+d peak=%+d)\r\n",
                                           (unsigned int)t3_seq_t1, (int)e3, (int)t3_seq_ext);
                                    t3_seq = 2;
                                    t3_seq_in = 0;
                                    tgt = T3_TGT_N5;
                                }
                                else                                    // -5cm 到位 -> 完成，保持在该点
                                {
                                    int32 total3 = (int32)(now3 - t3_seq_t0);
                                    printf("T3Q: DONE -5cm total=%ums (err=%+d) %s\r\n",
                                           (unsigned int)total3, (int)e3,
                                           (5000 >= total3) ? "TIME-OK(<=5s)" : "TIME-OVER(>5s)");
                                    t3_seq = 3;                         // 完成态：只保持目标，不再判定
                                    t3_seq_in = 0;
                                }
                            }
                        }
                        else
                        {
                            t3_seq_in = 0;                              // 出带，重新等
                        }
                    }

                    if((0 == t3_seq) && (!t3_run) && (0 == VISION_TARGET_ERR) && (0 < DITHER_PX))
                    {
                        tgt = (0 != ((comm_get_ms() / DITHER_MS) & 1)) ? DITHER_PX : -DITHER_PX;
                    }
                    int32 err = (int32)v_err - tgt;     // 相对目标点的误差

                    // 条件慢积分（2026-08-01 定案）：|err|<60px 才积累，远段清零防饱和；
                    // 上限 3000（j=20 时满幅 60us），0.3~0.6s 缓建压"磨"出静摩擦死区——
                    // 旧版 0.1s 冲顶 500 是 bang-bang 震荡源（j≥150 甩尾实案）
                    // 【2026-10-16 改】门限 60 -> SERVO_KI_GATE(300)：
                    //   原来 |err|>60 就清零，球被打到远端（实测 -194px）后积分完全帮不上忙，
                    //   回中只能靠 P 项硬推、几十秒回不来。上限 SERVO_KI_MAX=3000，j=-25 满幅 75us 不会失控。
                    if((SERVO_KI_GATE > err) && (-SERVO_KI_GATE < err))
                    {
                        v_err_integral += err;
                        if(v_err_integral > SERVO_KI_MAX) { v_err_integral = SERVO_KI_MAX; }
                        if(v_err_integral < SERVO_KI_MIN) { v_err_integral = SERVO_KI_MIN; }
                    }
                    else
                    {
                        v_err_integral = 0;
                    }

                    // 微分（帧级：新视觉帧到达才刷新差分并保持到下一帧；
                    // 主循环 ~230Hz 远快于 43Hz 帧率，循环级差分 5/6 是 0，KD 能量全散成单循环脉冲）
                    {
                        uint32 fms = comm_get_frame_ms();
                        if(fms != frame_ms_last)
                        {
                            frame_ms_last = fms;
                            err_diff_hold = err - v_err_last;
                            v_err_last = err;
                        }
                    }
                    int32 err_diff = err_diff_hold;

                    // 静摩擦破粘：误差大且长时间几乎不变时，给一段持续反向脉冲
                    // 静摩擦破粘：仅中点坑区（25<|err|<150）触发，端头不踢（端头是挡片不是静摩擦，踢了来回振荡）
                    #define STICTION_ERR_GATE   ( 6 )       // 触发门限 6px：±5px 以外都推（锁 ±5 内的要求）
                    // 【2026-10-16 改】上限 150 -> 400：题3 每段起步时 |err| 是 130~260，
                    //   原上限 150 让远段起步完全踢不到，只能靠静摩擦慢慢磨（实测某轮 +5->-5 磨了 4.9s）。
                    #define STICTION_ERR_HI     ( 400 )
                    #define STICTION_HOLD       ( 8 )       // 【2026-10-16 改】12 -> 8 帧：卡住后更快踢出去
                    #define STICTION_KICK_HOLD  ( 3 )
                    int32 kick = 0;
                    if(kick_hold > 0)
                    {
                        kick = (err > 0) ? STICTION_KICK : -STICTION_KICK;  /* 与 P 同向（与 err 同号）：破粘要把球推向目标，原反向会抵消 P */
                        kick_hold--;
                    }
                    else if((((err > STICTION_ERR_GATE) && (err < STICTION_ERR_HI))
                             || ((err < -STICTION_ERR_GATE) && (err > -STICTION_ERR_HI)))
                            && (err_diff > -5) && (err_diff < 5))
                    {
                        if(++stuck_cnt >= STICTION_HOLD)
                        {
                            kick = (err > 0) ? STICTION_KICK : -STICTION_KICK;  /* 与 P 同向（与 err 同号）：破粘要把球推向目标，原反向会抵消 P */
                            kick_hold = STICTION_KICK_HOLD - 1;
                            stuck_cnt = 0;
                        }
                    }
                    else
                    {
                        stuck_cnt = 0;
                    }

                    // PID 输出（符号：err>0 球偏右 → 需要舵机往左打，delta 符号由 SERVO_KP 正负决定）
                    // 负方向（err>0，往 -x 打）KP 按 KP_NEG_SCALE 放大：实测杆两侧死区不对称，负方向欠推
                    // KP 分级：远段 SERVO_KP 快推（>60px，出坑要快），近段 KP_NEAR 稳停（防近目标振荡）
                    int32 kp_eff = ((-100 < err) && (err < 100)) ? KP_NEAR : SERVO_KP;
                    if(err > 0) { kp_eff = kp_eff * KP_NEG_SCALE / 100; }
                    int32 delta = -((int32)kp_eff * err
                                  + (int32)SERVO_KI * v_err_integral      // 量纲修复：不再二次除1000（原 KI 满幅仅 10us 推不动死区）
                                  + (int32)SERVO_KD * err_diff) / 1000;
                    delta += kick;
                    // ---------- 题3 时间最优折返（2026-10-16）----------
                    // 为什么加：光靠 P/D 时，接近目标阶段 P 还在猛推、压住 D 的刹车，
                    //   等 D 压过 P 球已经太快刹不住 —— 实测稳定过冲到 -250~-288px，白吃 ~800ms。
                    // 做法：远段(|err|>26)直接跳过 PID，按"能不能刹住"满舵推或满舵反打：
                    //   d_brake = v²/T3_BRAKE_K（v=err_diff，px/帧）；够远就满舵朝目标，否则满舵反打。
                    // 近段(|err|<=26)交回 PID 精修，保证最后能稳住不抖。
                    // ---------- 题3 时间最优折返（已停用，2026-10-16）----------
                    // 思路对（用 err_diff 估刹车距离，够就满舵推、不够就满舵反打），但实测两版都 0/5：
                    //   ① 漏判"球是否正朝目标运动" -> 球在 err=20 但远离目标时一刹车反而推得更远 -> 失控 ±278
                    //   ② 补上方向判断后仍失控：dbrk = err_diff²/K 是平方增长，err_diff 一超过 ~40px/帧
                    //      就恒判"该刹"，满舵反打把球推向远端 -> 跑飞 ±283
                    // 结论：这个系数必须按实际帧率/减速度仔细标定（K=2a，a≈1380px/s²÷27fps²≈1.9 → K≈3.8），
                    //   不是快改能收的。当前回退到已验证 3/5 通过的 PID 配置。
                    // 要重启这条路线：先给 err_diff 加限幅，再单独标 K。
#if 0
                    if(((1 == t3_seq) || (2 == t3_seq)) && (26 < ((0 > err) ? -err : err)))
                    {
                        int32 eabs = (0 > err) ? -err : err;
                        int32 dbrk = err_diff * err_diff / T3_BRAKE_K;
                        int32 toward = (((0 < err) && (0 > err_diff)) ||
                                        ((0 > err) && (0 < err_diff))) ? 1 : 0;
                        if((0 == toward) || (eabs > dbrk))
                        {
                            delta = (0 < err) ?  DELTA_LIMIT : -DELTA_LIMIT;
                        }
                        else
                        {
                            delta = (0 < err) ? -DELTA_LIMIT :  DELTA_LIMIT;
                        }
                    }
#endif
                    // 纵向加速度前馈（题4/5/6：进弯出弯/起步刹车车加速球甩，舵机预倾补偿；符号由 A 正负定）
                    if(0 != ACC_FF)
                    {
                        delta -= (int32)((int64)ACC_FF * acc_long_lpf / 1000);
                    }
                    // 分级限幅：远目标大限幅快走，近目标小限幅稳停（kick 退役后由 KI 消静差）
                    {
                        int32 g = ((-100 < err) && (err < 100)) ? DELTA_NEAR : DELTA_LIMIT;
                        if(delta > g)  { delta = g;  }
                        if(delta < -g) { delta = -g; }
                    }

                    int32 us = (int32)SERVO_MID_VISION + delta;
                    if(SERVO_ABS_MAX_US < us) { us = SERVO_ABS_MAX_US; }
                    if(SERVO_ABS_MIN_US > us) { us = SERVO_ABS_MIN_US; }
                    servo_set_us((uint16)us);
                }
                else if((!v_valid) || ((0 == SERVO_KP) && (0 == SERVO_KI) && (0 == SERVO_KD)))
                {
                    // 无有效球或视觉伺服关闭时清零积分，避免再次见到球时积分饱和冲舵
                    v_err_integral = 0;
                    v_err_last = 0;
                    err_diff_hold = 0;
                    stuck_cnt = 0;
                    kick_hold = 0;
                }
            }
        }

        if(3 <= ++loop_cnt)                         // 调参期 3 循环一打印（实测 ~25Hz）；
                                                    // 原为 10（实测仅 7.5Hz，3 秒跑动只采到 ~22 帧）
                                                    // 定案后可回 10 或更大，降低串口占用
        {
            loop_cnt = 0;
#if !VISION_TEST_MODE
            int32 rpm_l = enc_l_10ms * 6000 / ENC_PPR;
            int32 rpm_r = enc_r_10ms * 6000 / ENC_PPR;
            if(line_lost)
            {
                printf("pos=LOST(%dms)", lost_hold_ms);            }
            else
            {
                int32 p = line_pos_x100;
                printf("pos=%s%d.%02d",
                       (0 > p) ? "-" : "+",
                       (int)((0 > p) ? -p : p) / 100, (int)((0 > p) ? -p : p) % 100);
            }
            printf("%s | rpm L=%4d R=%4d | duty L=%5d R=%5d | tgt %d/%d bs=%d d=%d | set%d p=%d i=%d d2=%d e=%d q=%d b=%d s=%d r=%d c=%d\r\n",
                   in_corner ? "C" : "S",
                   rpm_l, rpm_r, ctrl_duty_l, ctrl_duty_r,
                   (int)target_l_dbg, (int)target_r_dbg, (int)base_smooth, (int)diff_dbg,
                   gain_set, SPD_KP[gain_set], SPD_KI[gain_set], LINE_KP[gain_set], LINE_KD[gain_set],
                   LINE_KQ[gain_set], BASE_RPM[gain_set], SLOW_KP[gain_set], RAMP_STEP, CORNER_RPM[gain_set]);
            if(g_bench_mode) { printf("BENCH=ON\r\n"); }
            // 【2026-10-16 诊断】主循环实测节拍：dt=本轮打印间隔 ms（3 个循环），isr=同期 10ms 中断次数
            //   正常应 dt≈60~90 / isr≈6~9；电机一转就掉 = 主循环被拖慢，锚定 PID 必须搬进中断
            {
                static uint32 lp_t_prev = 0, lp_isr_prev = 0;
                uint32 lp_now = comm_get_ms();
                printf("LP: dt=%u isr=%u\r\n", (unsigned)(lp_now - lp_t_prev),
                       (unsigned)(g_isr_cnt - lp_isr_prev));
                lp_t_prev = lp_now;
                lp_isr_prev = g_isr_cnt;
            }
#if LINE_SRC_6CH
            printf("S6: st=0x%02X n=%d ll=%d a=%4d %4d %4d %4d %4d %4d fail=%d\r\n",
                   s6_state_dbg, s6_n_dbg, line_lost,
                   s6_analog_dbg[0], s6_analog_dbg[1], s6_analog_dbg[2],
                   s6_analog_dbg[3], s6_analog_dbg[4], s6_analog_dbg[5],
                   sensor_6ch_get_fail_cnt());
#elif IR7_DIGITAL
            // 七路数字状态：7 位从左到右 = 通道1..7，1=压黑线。逐路验证就看这一行
            printf("IR7: bits=%d%d%d%d%d%d%d (ch1..ch7) raw=%d%d%d%d%d%d%d n=%d ll=%d hl=%d\r\n",
                   (ir7_bits >> 0) & 1, (ir7_bits >> 1) & 1, (ir7_bits >> 2) & 1,
                   (ir7_bits >> 3) & 1, (ir7_bits >> 4) & 1, (ir7_bits >> 5) & 1,
                   (ir7_bits >> 6) & 1,
                   (ir7_raw_bits >> 0) & 1, (ir7_raw_bits >> 1) & 1, (ir7_raw_bits >> 2) & 1,
                   (ir7_raw_bits >> 3) & 1, (ir7_raw_bits >> 4) & 1, (ir7_raw_bits >> 5) & 1,
                   (ir7_raw_bits >> 6) & 1,
                   (int)((ir7_bits >> 0) & 1) + (int)((ir7_bits >> 1) & 1) + (int)((ir7_bits >> 2) & 1)
                   + (int)((ir7_bits >> 3) & 1) + (int)((ir7_bits >> 4) & 1) + (int)((ir7_bits >> 5) & 1)
                   + (int)((ir7_bits >> 6) & 1),
                   line_lost, line_hold);
#endif
#endif
            {
                int16  v_err = 0;
                uint8  v_status = 0;
                uint32 v_age = 0;
                static uint8 v_novision_printed = 0;    // "无视觉"提示只打一次，别每帧刷屏
                if(comm_get_vision(&v_err, &v_status, &v_age))
                {
                    v_novision_printed = 0;
                    if(t3_run)
                    {
                        printf("T3: st=%d ref=%+4d x=%+4d vel=%+3d us=%d ela=%ums | tgt%+d/%+d KP=%d KD=%d\r\n",
                               t3_state, t3_ref, t3_x, t3_vel, servo_get_us(),
                               (unsigned int)(comm_get_ms() - t3_seg0),
                               T3_TGT_P5, T3_TGT_N5, T3_KP, T3_KD);
                    }
                    else
                    {
                        printf("VISION: err=%+4d status=%d age=%3d ms | servo us=%d | kp=%d ki=%d kd=%d target=%d mid=%d | acc=%+d | ec=%u\r\n",
                               v_err, v_status, v_age, servo_get_us(), SERVO_KP, SERVO_KI, SERVO_KD, VISION_TARGET_ERR, SERVO_MID_VISION, acc_long_lpf,
                               (unsigned int)comm_get_err_count());
                    }
                }
                else if(!v_novision_printed)
                {
                    v_novision_printed = 1;
                    printf("VISION: no valid frame yet (err_cnt=%u)\r\n", (unsigned int)comm_get_err_count());
                }
            }
#if !VISION_TEST_MODE
            if(stall_l_flag || stall_r_flag)    // 堵转提示每秒一次（500ms 块隔次），不刷屏
            {
                static uint8 stall_msg_div = 0;
                stall_msg_div ^= 1;
                if(0 == stall_msg_div)
                {
                    if(stall_l_flag)
                    {
                        printf("STALL L! duty=0\r\n");
                    }
                    if(stall_r_flag)
                    {
                        printf("STALL R! duty=0\r\n");
                    }
                }
            }
#endif
        }

        system_delay_ms(10);
    }
}
