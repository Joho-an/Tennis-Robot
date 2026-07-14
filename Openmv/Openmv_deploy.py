# OpenMV — 色块定位 + 模型确认 + UART输出 (X坐标 + 距离)
# 标签: out[0]=ball, out[1]=background

import sensor, image, time, tf, gc
from pyb import UART

# === UART 初始化 (P4=TX, P5=RX, 115200bps) ===
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320×240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
time.sleep_ms(2000)

print("Loading...")
gc.collect()
net = tf.load("mobilenet_v2_int8.tflite")
print("OK")

THRESHOLD = [(68, 100, -45, 15, 34, 113)]
MIN_AREA = 200
MAX_AREA = 30000
MIN_ROUNDNESS = 0.4
MIN_DENSITY = 0.2
CONF_THRESHOLD = 0.6

# 距离估算常数 (需实机标定)
# 焦距约 228px (70° FOV), 网球直径 6.7cm
FOCAL_PX = 228
BALL_DIAM_CM = 6.7

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    gc.collect()

    # 色块扫描
    candidates = []
    for b in img.find_blobs(THRESHOLD, pixels_threshold=MIN_AREA,
                            area_threshold=MIN_AREA, merge=True):
        if b.area() < MIN_AREA or b.area() > MAX_AREA:
            continue
        if b.roundness() < MIN_ROUNDNESS:
            continue
        if b.density() < MIN_DENSITY:
            continue
        candidates.append(b)

    best_x, best_y, best_w = 0, 0, 0
    best_conf = 0.0
    found = False

    for b in candidates:
        cx, cy = b.cx(), b.cy()
        x1 = max(cx - 24, 0)
        y1 = max(cy - 24, 0)
        if x1 + 48 > img.width():
            x1 = img.width() - 48
        if y1 + 48 > img.height():
            y1 = img.height() - 48

        roi = img.copy(roi=(x1, y1, 48, 48))
        for obj in net.classify(roi):
            out = obj.output()
            ball_s = out[0]
            if ball_s > out[1] and ball_s > best_conf:
                best_conf = ball_s
                best_x, best_y = x1, y1
                best_w = b.w()  # blob 像素宽度
                found = True

    # 画框 + UART输出
    if found and best_conf > CONF_THRESHOLD:
        # X坐标: 框中心 (0=最左, 320=最右)
        ball_cx = best_x + 24

        # 距离估算: distance = (focal * real_diam) / pixel_width
        if best_w > 0:
            dist_cm = (FOCAL_PX * BALL_DIAM_CM) / best_w
        else:
            dist_cm = 0

        # UART 打包输出
        uart.write("X:%d,D:%.1f\n" % (ball_cx, dist_cm))

        # 画面标注
        img.draw_rectangle((best_x, best_y, 48, 48),
                           color=(0, 255, 0), thickness=2)
        img.draw_string(best_x + 2, max(best_y - 10, 0),
                        "ball %.2f" % best_conf, color=(0, 255, 0))
        img.draw_string(best_x + 2, best_y + 50,
                        "X:%d D:%.1fcm" % (ball_cx, dist_cm),
                        color=(255, 255, 0))
    else:
        img.draw_string(10, 100, "no ball", color=(255, 0, 0), scale=2)

    gc.collect()

    if clock.fps() > 0:
        print("%.1f fps | ball:%.2f" % (clock.fps(), best_conf))
