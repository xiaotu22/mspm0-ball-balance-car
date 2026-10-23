# -*- coding: utf-8 -*-
# ============================================================
# car_vision_stream.py —— 循迹/找球双模式 + UART2 全链路 + WiFi MJPEG 图传
# 平台：01Studio CanMV K230 mini + 2.4寸屏（ST7701 640x480）
# 底版：car_vision.py（B06 补充版）
#
# 【图传实现】
#   CanMV v1.8 没有 media.vencoder/MJPEGEncoder，因此采用软件 JPEG：
#   chn0 RGB565 640x480 帧 → copy(roi=CROP_ROI) → img.to_jpeg() → socket MJPEG 流
#
# 【访问方式】
#   浏览器打开 http://<K230_IP>:8080/      → 实时视频页
#   Python/OpenCV 打开 http://<K230_IP>:8080/  → 录像
#
# 【图传 vs 识别 资源分配】
#   - chn0 RGB565 640x480：本地屏显示 + 图传源（发送完整画面）
#   - chn1 RGB888P 320x240：YOLOv11n 推理（主任务，保帧率）
#   - 软件 JPEG 编码：图传（限帧，减小 CPU 负担）
# ============================================================

USE_DUAL_MODEL = False
THRESHOLD      = (0, 100)
MIN_MAGNITUDE  = 1.0
IMG_W, IMG_H   = 640, 480
AI_W, AI_H     = 320, 240
SCALE          = 2
LOCK_DIST_PX   = 60         # 锁球半径减小，跟球更紧
LOST_RELEASE   = 5          # 丢球判定更快释放，重新搜索更积极
EMA_NEW        = 0.85       # 降低平滑，十字线跟随更实时
SEND_EVERY     = 1          # 每帧都向 MSPM0 发视觉偏差，保证舵机控制实时性
SEND_MAX_MS    = 50
TO_IDE         = True
DISP_CROP      = False
PRINT_EVERY    = 10         # 调参时打印频率高，方便观察
GC_EVERY       = 10
KMODEL_PATH    = "/sdcard/yolo11n_11279.kmodel" if USE_DUAL_MODEL \
                 else "/sdcard/yolo11n_det_12422.kmodel"
LABELS         = {0: 'slider', 1: 'steel ball'} if USE_DUAL_MODEL else {0: 'steel ball'}
MODEL_INPUT    = [320, 320]
CONF_TH        = 0.6
NMS_TH         = 0.45
MAX_BOXES      = 10

CROP_ROI      = (0, 205, 640, 70)
AI_CROP_TOP   = int(CROP_ROI[1] * AI_H / IMG_H)
AI_CROP_BOT   = int((CROP_ROI[1] + CROP_ROI[3]) * AI_H / IMG_H)

# ---------- 图传配置 ----------
ENABLE_STREAM  = False         # 【2026-10-17 小土定：关】图传开着会压 K230 帧率（Wi-Fi + 每帧 to_jpeg），
                               #   控制环延迟大半拍：题3 +5cm 过冲从 ±0.2cm 恶化到 +0.81cm。
                               #   实测：开流后 +5 峰值 5.81cm；关流后 4.88~5.19cm。
                               #   要开回来：改 True 再上传，同时把题3 KD 按新帧率重标。
                               #   注：没人连流时 stream_poll 会提前 return，但 Wi-Fi 本身仍占 CPU。
WIFI_SSID      = "xiaotu"      # 【2026-10-15 小土改】原 "679"，换成自己的热点
WIFI_PASSWORD  = "liuyue22"
STREAM_PORT    = 8080
STREAM_FPS     = 15            # 图传目标帧率上限：回退到稳定版
JPEG_QUALITY   = 40            # 1~99：回退到稳定版
# ============================================================

import time, os, sys, gc

from media.sensor import *
from media.display import *
from media.media import *
from libs.YOLO import YOLO11
from libs.Utils import *
from machine import UART
from machine import FPIOA
import image
import ulab.numpy as np

# ---------- 条件导入图传模块 ----------
if ENABLE_STREAM:
    import network, socket

CX = IMG_W // 2
TARGET_X = CX
PX_PER_CM = 25.2
LIMIT_CM  = 5
LIMIT_PX_L = 130          # 左绿线（物理 -5cm）= TARGET_X-130 → x=190（2026-08-01 落地实测）
LIMIT_PX_R = 129          # 右绿线（物理 +5cm）= TARGET_X+129 → x=449（透视不对称，左右分别标定）
CLS_BALL   = 1 if USE_DUAL_MODEL else 0
CLS_SLIDER = 0
LOCK_DIST_SQ = LOCK_DIST_PX * LOCK_DIST_PX

def dbg_print(fmt, *args):
    if frame_cnt % PRINT_EVERY == 0:
        print(fmt % args)

# ---------- UART1 初始化 ----------
# 接线（K230 mini 丝印 IO 口，官方引脚图 2026-08-01 核；勿用排针序号，以 IO 丝印为准）：
#   K230 IO3 = TX1 → MSPM0 B16 (UART2 RX)   视觉帧下行，必须接
#   K230 IO4 = RX1 → MSPM0 B15 (UART2 TX)   上行 $C 命令，可不接
#   K230 GND       → MSPM0 GND               共地必须
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, baudrate=115200)

# ---------- 下行状态帧 ----------
# 简化：只发 err + status，不再发 x/y/type，主控只关心横向偏差
def send_vision(err, status, ttype=0, x=0, y=0):
    err = max(-999, min(999, err))
    body = "V,%+04d,%d" % (err, status)
    chk = 0
    for c in body:
        chk ^= ord(c)
    uart.write("$%s*%02X\r\n" % (body, chk))

# ---------- 上行命令帧接收 ----------
rx_buf = b""
mode = 2

def poll_command():
    global rx_buf, mode
    # 【2026-10-16 修 A】原为 uart.read(64)：UART.read(n) 要等满 n 个字节才返回，
    #   IO4(RX) 悬空时噪声喂进半截字节就阻塞等剩下 63 个 -> 主循环停摆 ->
    #   串口不再发帧、屏幕冻住，表现成"K230 硬挂"。改成先 any() 再按可用长度读。
    _n = uart.any()
    if not _n:
        return
    data = uart.read(_n)          # 只读当前可用字节，永不阻塞
    if data:
        rx_buf += data
        if not rx_buf.startswith(b"$"):
            i = rx_buf.rfind(b"$")
            rx_buf = rx_buf[i:] if i >= 0 else b""
        if len(rx_buf) > 32:
            i = rx_buf.rfind(b"$")
            rx_buf = rx_buf[i:] if i >= 0 else b""
    while True:
        i = rx_buf.find(b"\n")
        if i < 0:
            break
        frame = rx_buf[:i + 1]
        rx_buf = rx_buf[i + 1:]
        if len(frame) == 17 and frame[0:1] == b"$" and \
           frame[1:2] == b"C" and frame[12:13] == b"*":
            chk = 0
            for k in range(1, 12):
                chk ^= frame[k]
            try:
                want = int(frame[13:15], 16)
            except ValueError:
                continue
            if chk == want:
                m = frame[3] - 0x30
                p1 = int(frame[5:8])
                p2 = int(frame[9:12])
                if m == 3:
                    m = 2
                if 0 <= m <= 2 and m != mode:
                    mode = m
                    dbg_print("MODE -> %d (p1=%03d p2=%03d)", mode, p1, p2)
                else:
                    dbg_print("CMD mode=%d p1=%03d p2=%03d", m, p1, p2)

# ---------- WiFi STA + MJPEG 图传初始化 ----------
stream_listen = None
stream_client = None
stream_last_frame_ms = 0
STREAM_FRAME_INTERVAL = 1000 // STREAM_FPS

def stream_init():
    global stream_listen
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting WiFi %s ..." % WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > 15000:
                print("WiFi connect timeout")
                return False
            time.sleep_ms(100)
    ip = wlan.ifconfig()[0]
    print("WiFi connected, IP: %s" % ip)
    print("Open http://%s:%d/ in browser" % (ip, STREAM_PORT))

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except:
        pass
    s.setblocking(False)
    addr = socket.getaddrinfo("0.0.0.0", STREAM_PORT)[0][-1]
    s.bind(addr)
    s.listen(1)
    print("MJPEG server listening on :%d" % STREAM_PORT)
    stream_listen = s
    return True

def _close_client():
    global stream_client
    try:
        stream_client.close()
    except:
        pass
    stream_client = None

def _send_all(sock, data, max_ms=5):
    """非阻塞发送全部数据；遇到 EAGAIN/EINPROGRESS 立即丢帧，避免阻塞主循环。"""
    mv = memoryview(data)
    sent = 0
    t0 = time.ticks_ms()
    while sent < len(data):
        if time.ticks_diff(time.ticks_ms(), t0) > max_ms:
            return False
        try:
            n = sock.send(mv[sent:sent + 16384])
            if n == 0:
                return False
            sent += n
        except OSError as e:
            err = e.args[0]
            if err == 11 or err == 115:  # EAGAIN / EINPROGRESS
                return False
            raise
    return True

def stream_poll(img_rgb):
    """非阻塞处理 MJPEG 客户端连接与发送。
    img_rgb: chn0 RGB565 640x480 帧；直接 crop ROI 后用软件 JPEG 编码。
    """
    global stream_client, stream_last_frame_ms
    if stream_listen is None:
        return

    # 接受新客户端（只保留最新一个）
    if stream_client is None:
        try:
            client, addr = stream_listen.accept()
            client.setblocking(False)
            stream_client = client
            header = ("HTTP/1.1 200 OK\r\n"
                      "Server: K230\r\n"
                      "Content-Type: multipart/x-mixed-replace;boundary=frame\r\n"
                      "Cache-Control: no-cache\r\n"
                      "Pragma: no-cache\r\n\r\n").encode()
            if not _send_all(stream_client, header, max_ms=50):
                _close_client()
                return
            stream_last_frame_ms = 0   # 新客户端首帧立即发送，避免白屏
            print("Client connected: %s" % str(addr))
        except OSError:
            pass
        except Exception as e:
            print("accept err:", e)

    if stream_client is None:
        return

    # 限帧发送
    now = time.ticks_ms()
    if time.ticks_diff(now, stream_last_frame_ms) < STREAM_FRAME_INTERVAL:
        return
    stream_last_frame_ms = now

    try:
        # 软件 JPEG 编码：只传红框 ROI 区域
        roi_img = img_rgb.copy(roi=CROP_ROI)
        jpeg = roi_img.to_jpeg(quality=JPEG_QUALITY)
        header = ("--frame\r\n"
                  "Content-Type: image/jpeg\r\n"
                  "Content-Length: %d\r\n\r\n" % len(jpeg)).encode()
        if not _send_all(stream_client, header + jpeg):
            _close_client()
            print("stream client lost: send 0")
    except Exception as e:
        _close_client()
        print("stream client lost:", e)

# ---------- 传感器：双通道 ----------

clock = time.clock()

frame_cnt = 0
last_send_ms = time.ticks_ms()
last_err = 0

locked = False
sx = 0.0; sy = 0.0
lost_cnt = 0

# ============================================================
# 【2026-10-16 自恢复改造】原版：一次性初始化 + 裸 while True。
#   任何异常 / 相机占用 / 串口阻塞 都会让脚本退出或停摆到 REPL，串口从此永远不再输出，
#   只能人工断电 —— 实测多次。现在四层防护：初始化重试 + 主循环兜异常 + 非阻塞串口 + 看门狗。
# ============================================================
import machine
from machine import WDT

_wdt = None

def _feed():
    """喂狗（WDT 没建起来时静默跳过）"""
    try:
        if _wdt is not None:
            _wdt.feed()
    except Exception:
        pass

def _send_raw(err, status):
    """直接发一帧，不依赖 main_loop 里的局部变量"""
    err = max(-999, min(999, err))
    body = "V,%+04d,%d" % (err, status)
    chk = 0
    for c in body:
        chk ^= ord(c)
    try:
        uart.write("$%s*%02X\r\n" % (body, chk))
    except Exception as e:
        print("mark send err:", e)

def boot_marker():
    """开机标记：err=±899 不可能是真球位（球机械行程 ±130px），主控侧看到就知道 K230 重启了"""
    for _ in range(3):
        _send_raw(899, 1)
        time.sleep_ms(20)
        _send_raw(-899, 1)
        time.sleep_ms(20)
    print("== K230 BOOT MARKER SENT ==")

def hw_bringup():
    """相机 + 显示 + 媒体 + YOLO 初始化；失败先把上次占用释放掉再重试"""
    global sensor, yolo
    for attempt in range(3):
        _feed()                                  # 每次尝试前喂狗：3 次重试也不会超时
        try:
            sensor = Sensor(width=1280, height=960)
            sensor.reset()
            # chn0 → 显示（RGB565 640x480），也作为图传源
            sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
            sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
            # chn1 → AI 推理（RGB888P 320x240）
            sensor.set_framesize(width=AI_W, height=AI_H, chn=CAM_CHN_ID_1)
            sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_1)

            Display.init(Display.ST7701, width=IMG_W, height=IMG_H, to_ide=TO_IDE)
            MediaManager.init()
            sensor.run()

            # ---------- 图传服务启动 ----------
            if ENABLE_STREAM:
                stream_init()

            # ---------- YOLO11 ----------
            yolo = YOLO11(task_type="detect", mode="video",
                          kmodel_path=KMODEL_PATH, labels=LABELS,
                          rgb888p_size=[AI_W, AI_H], model_input_size=MODEL_INPUT,
                          display_size=[AI_W, AI_H],
                          conf_thresh=CONF_TH, nms_thresh=NMS_TH,
                          max_boxes_num=MAX_BOXES, debug_mode=0)
            yolo.config_preprocess()
            print("hw_bringup OK (attempt %d)" % attempt)
            return True
        except Exception as e:
            print("hw_bringup FAIL(%d): %s" % (attempt, e))
            try:
                sys.print_exception(e)
            except Exception:
                pass
            try:
                sensor.deinit()
            except Exception:
                pass
            try:
                MediaManager.deinit()
            except Exception:
                pass
            gc.collect()
            time.sleep_ms(800)
    return False


def main_loop():
    global frame_cnt, last_send_ms, last_err, locked, sx, sy, lost_cnt, status, err, xc, yc
    frame_cnt = 0
    last_send_ms = time.ticks_ms()
    last_err = 0
    locked = False
    sx = 0.0
    sy = 0.0
    lost_cnt = 0
    status = 0
    err = 0
    xc = TARGET_X
    yc = 0

    while True:
        clock.tick()

        # 【2026-10-16 补丁B】喂狗：主循环 46fps，正常每 ~20ms 喂一次；真卡死 20s 后硬件复位
        try:
            _wdt.feed()
        except Exception:
            pass

        # 显示通道先取帧（保留全帧，供图传使用）
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_full = img

        if mode == 1:
            gray = img.to_grayscale()
            bin_img = gray.binary([THRESHOLD])
            line = bin_img.get_regression([(255, 255)])
            if line and line.magnitude() >= MIN_MAGNITUDE:
                status = 1
                mx = (line.x1() + line.x2()) // 2
                err = mx - CX
                last_err = err
                bin_img.draw_line(line.line(), color=127, thickness=4)
                my = (line.y1() + line.y2()) // 2
                bin_img.draw_line(mx - 8, my, mx + 8, my, color=255, thickness=2)
                bin_img.draw_line(mx, my - 8, mx, my + 8, color=255, thickness=2)
                bin_img.draw_string_advanced(4, 4, 32, "MODE:LINE err=%+d FPS=%.1f" %
                                             (err, clock.fps()), color=255)
                dbg_print("FPS %.1f  err=%+d  theta=%d  mag=%.1f",
                      clock.fps(), err, line.theta(), line.magnitude())
            else:
                status = 0
                bin_img.draw_string_advanced(4, 4, 32,
                                             "MODE:LINE LOST last=%+d FPS=%.1f" %
                                             (last_err, clock.fps()), color=255)
                dbg_print("FPS %.1f  LOST (last_err=%+d)", clock.fps(), last_err)
            Display.show_image(bin_img)

        elif mode == 2:
            img_ai = sensor.snapshot(chn=CAM_CHN_ID_1)
            res = yolo.run(img_ai.to_numpy_ref())
            boxes, classes, scores = res[0], res[1], res[2]

            balls = []
            slider = None
            for i in range(len(boxes)):
                bx = int(boxes[i][0]) * SCALE
                by = int(boxes[i][1]) * SCALE
                bw = int(boxes[i][2]) * SCALE
                bh = int(boxes[i][3]) * SCALE
                cls = int(classes[i]); sc = scores[i]
                if cls == CLS_BALL:
                    cy = by + bh // 2
                    if CROP_ROI[1] <= cy <= CROP_ROI[1] + CROP_ROI[3]:
                        balls.append((bx, by, bw, bh, sc))
                        img.draw_rectangle(bx, by, bw, bh,
                                           color=(128, 128, 128), thickness=1)
                elif USE_DUAL_MODEL and cls == CLS_SLIDER:
                    if slider is None or sc > slider[5]:
                        slider = [bx + bw // 2, bx, by, bw, bh, sc]
                    img.draw_rectangle(bx, by, bw, bh,
                                       color=(255, 200, 0), thickness=2)

            ball_found = False
            if len(balls) > 0:
                if not locked:
                    best = 0
                    for i in range(1, len(balls)):
                        if balls[i][4] > balls[best][4]:
                            best = i
                    bx, by, bw, bh, sc = balls[best]
                    sx = bx + bw / 2.0
                    sy = by + bh / 2.0
                    locked = True
                    lost_cnt = 0
                    ball_found = True
                    pick = best
                else:
                    best = -1
                    best_d = LOCK_DIST_SQ
                    for i in range(len(balls)):
                        cx_i = balls[i][0] + balls[i][2] / 2.0
                        cy_i = balls[i][1] + balls[i][3] / 2.0
                        dx = cx_i - sx; dy = cy_i - sy
                        d = dx * dx + dy * dy
                        if d < best_d:
                            best_d = d
                            best = i
                    if best >= 0:
                        lost_cnt = 0
                        ball_found = True
                        pick = best
                    else:
                        lost_cnt += 1
                        if lost_cnt >= LOST_RELEASE:
                            locked = False

            if ball_found:
                bx, by, bw, bh, sc = balls[pick]
                cx_raw = bx + bw / 2.0
                cy_raw = by + bh / 2.0
                sx = sx * (1.0 - EMA_NEW) + cx_raw * EMA_NEW
                sy = sy * (1.0 - EMA_NEW) + cy_raw * EMA_NEW
                xc = int(sx); yc = int(sy)
                cx_i = int(cx_raw); cy_i = int(cy_raw)
                img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0), thickness=2)
                img.draw_line(cx_i - 12, cy_i, cx_i + 12, cy_i, color=(255, 0, 0), thickness=2)
                img.draw_line(cx_i, cy_i - 12, cx_i, cy_i + 12, color=(255, 0, 0), thickness=2)

            img.draw_line(TARGET_X, CROP_ROI[1], TARGET_X,
                          CROP_ROI[1] + CROP_ROI[3], color=(0, 0, 255), thickness=2)
            img.draw_line(TARGET_X - LIMIT_PX_L, CROP_ROI[1],
                          TARGET_X - LIMIT_PX_L, CROP_ROI[1] + CROP_ROI[3],
                          color=(0, 255, 0), thickness=2)
            img.draw_line(TARGET_X + LIMIT_PX_R, CROP_ROI[1],
                          TARGET_X + LIMIT_PX_R, CROP_ROI[1] + CROP_ROI[3],
                          color=(0, 255, 0), thickness=2)

            if USE_DUAL_MODEL:
                target_ok = ball_found and (slider is not None)
                if target_ok:
                    pipe_cx = slider[0]
                    pipe_w  = slider[3]
                    err = xc - pipe_cx
                    img.draw_line(pipe_cx, slider[2] - 10, pipe_cx,
                                  slider[2] + slider[4] + 10,
                                  color=(255, 0, 0), thickness=2)
                    hud = "MODE:BALL* err=%+d pw=%d FPS=%.1f" % \
                          (err, pipe_w, clock.fps())
                    dbg_print("FPS %.1f  err=%+d ball=(%d,%d) pipe_cx=%d pipe_w=%d",
                          clock.fps(), err, xc, yc, pipe_cx, pipe_w)
                else:
                    miss = ""
                    if not ball_found:
                        miss += "ball "
                    if slider is None:
                        miss += "pipe"
                    hud = "MODE:BALL* LOST(%s) last=%+d FPS=%.1f" % \
                          (miss, last_err, clock.fps())
                    dbg_print("FPS %.1f  LOST(%s) last_err=%+d",
                          clock.fps(), miss, last_err)
            else:
                target_ok = ball_found
                if target_ok:
                    err = xc - TARGET_X
                    hud = "MODE:BALL err=%+d (%d,%d) FPS=%.1f" % \
                          (err, xc, yc, clock.fps())
                    dbg_print("FPS %.1f  ball err=%+d xc=%d yc=%d",
                          clock.fps(), err, xc, yc)
                else:
                    hud = "MODE:BALL NONE last=%+d FPS=%.1f" % \
                          (last_err, clock.fps())
                    dbg_print("FPS %.1f  no ball (last_err=%+d)",
                          clock.fps(), last_err)

            if target_ok:
                last_err = err
                status = 2
            else:
                status = 0

            if DISP_CROP:
                img = img.crop(CROP_ROI)
            else:
                img.draw_rectangle(CROP_ROI, color=(255, 0, 0), thickness=2)
            img.draw_string_advanced(4, 4, 32, hud, color=(0, 255, 0))
            Display.show_image(img)
            if frame_cnt % GC_EVERY == 0:
                gc.collect()

        else:
            status = 0
            img.draw_string_advanced(4, 4, 32, "MODE:STOP FPS=%.1f" % clock.fps(),
                                     color=(0, 255, 0))
            Display.show_image(img)

        # 每 SEND_EVERY 帧发一次；同时保证周期 ≤ SEND_MAX_MS
        frame_cnt += 1
        now_ms = time.ticks_ms()
        if frame_cnt % SEND_EVERY == 0 or \
           time.ticks_diff(now_ms, last_send_ms) >= SEND_MAX_MS:
            if mode == 2 and status == 2:
                send_vision(err, 2, 1, xc, yc)
            else:
                send_vision(last_err, status, 0, 0, 0)
            last_send_ms = now_ms

        # 图传：用原始全帧 crop ROI 后软件 JPEG 编码发送
        if ENABLE_STREAM:
            stream_poll(img_full)

        poll_command()


# ============================================================
# 自恢复监督循环
# ============================================================
_bringup_dead = 0
while True:
    try:
        # 【补丁B】20 秒看门狗：正常初始化 3~5s，重试期间 hw_bringup 内也会喂狗
        try:
            _wdt = WDT(0, 20000)
            print("WDT armed 20s")
        except Exception as e:
            print("WDT init FAIL:", e)
            _wdt = None
        if not hw_bringup():
            _bringup_dead += 1
            print("CAM BRINGUP DEAD x%d -> machine.reset()" % _bringup_dead)
            time.sleep_ms(1500)
            machine.reset()
        _bringup_dead = 0
        boot_marker()
        main_loop()                  # 正常不返回；返回即视为异常退出，重来
        print("main_loop RETURNED -> restart")
    except Exception as e:
        print("APP ERROR: %s" % e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
        try:
            sensor.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        gc.collect()
        time.sleep_ms(500)

