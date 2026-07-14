import sensor, image, time, gc
from pyb import UART
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.__write_reg(0x13, 0x00)
sensor.__write_reg(0x10, (700 >> 8) & 0xFF)
sensor.__write_reg(0x04, 0x40 | (700 & 0x03))
sensor.__write_reg(0x00, 0x30)
sensor.__write_reg(0x8C, 0x80)
sensor.__write_reg(0x2F, 0x50)
sensor.__write_reg(0x30, 0x48)
sensor.__write_reg(0x31, 0x60)
time.sleep_ms(2000)
print("Sensor locked")
BLACK_THRESHOLD  = [(0, 10, -128, 127, -128, 127)]
GREEN_THRESHOLD  = [(47, 100, -128, -7, 25, 121)]
LINE_ROI	   = (0, 120, 320, 120)
LINE_MIN_AREA  = 80
LINE_DEADZONE  = 6
BALL_MIN_AREA  = 100
FOCAL_PX	   = 228
BALL_DIAM_CM   = 6.7
clock = time.clock()
frame_cnt = 0
while True:
	clock.tick()
	img = sensor.snapshot()
	gc.collect()
	line_dev = 0
	blobs = img.find_blobs(BLACK_THRESHOLD,
						   pixels_threshold=LINE_MIN_AREA,
						   area_threshold=LINE_MIN_AREA,
						   merge=True, roi=LINE_ROI)
	if blobs:
		best = max(blobs, key=lambda b: b.pixels())
		line_cx = best.cx()
		line_dev = line_cx - 160
		if abs(line_dev) <= LINE_DEADZONE:
			line_dev = 0
		img.draw_line((line_cx, 120, line_cx, 240),
					  color=(255, 0, 0), thickness=2)
		img.draw_cross(line_cx, 180, color=(255, 0, 0), size=8)
	img.draw_line((160, 120, 160, 240), color=(128, 128, 128), thickness=1)
	ball_x  = 0
	dist_cm = 0
	blobs = img.find_blobs(GREEN_THRESHOLD,
						   pixels_threshold=BALL_MIN_AREA,
						   area_threshold=BALL_MIN_AREA,
						   merge=True, margin=5)
	if blobs:
		best = max(blobs, key=lambda b: b.pixels())
		ball_cx = best.cx()
		ball_x  = min(ball_cx * 255 // 320, 255)
		w = best.w()
		if w > 0:
			dist_cm = int((FOCAL_PX * BALL_DIAM_CM) / w)
			if dist_cm > 255:
				dist_cm = 255
		img.draw_rectangle(best.rect(), color=(0, 255, 0), thickness=2)
		img.draw_cross(best.cx(), best.cy(), color=(0, 255, 0))
		img.draw_string(best.x(), max(best.y() - 10, 0),
					   "%dc" % dist_cm, color=(0, 255, 0))
	line_byte = line_dev if line_dev >= 0 else (256 + line_dev)
	uart.write(b'\x55' + bytearray([ball_x, dist_cm, line_byte, 0]) + b'\xAA')
	gc.collect()
	frame_cnt += 1
	if frame_cnt % 10 == 0:
		print("%.1ffps | X:%d D:%d L:%d" %
			  (clock.fps(), ball_x, dist_cm, line_dev))