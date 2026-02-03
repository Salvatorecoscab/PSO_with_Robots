import cv2
import matplotlib.pyplot as plt
import pandas as pd

VIDEO_NAME = 'led2.mp4'
cap = cv2.VideoCapture(VIDEO_NAME)
assert cap.isOpened()

FPS = 960

PADDING_FRAME_CNT = 50

values = []
times = []

for i in range(PADDING_FRAME_CNT):
    ret, frame = cap.read()

count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print('error at %d' % count)
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mean_val = cv2.mean(v)
    values.append(mean_val[0])
    times.append(count / FPS)
    count = count + 1

cap.release()

times = times[:-PADDING_FRAME_CNT]
values = values[:-PADDING_FRAME_CNT]
count -= PADDING_FRAME_CNT


meanv = sum(values) / len(values)
ma = max(values)
mi = min(values)
hi_cnt = 0
lo_cnt = 0
now = None
for v in values:
    if v > meanv:
        if now != 'hi':
            hi_cnt += 1
        now = 'hi'
    else:
        if now != 'lo':
            lo_cnt += 1
        now = 'lo'


real_duration = count / FPS
print('duration: %.4fs' % real_duration)
freq = ((lo_cnt + hi_cnt) / real_duration / 2)
per_diff = (ma-mi)*2/(ma+mi)*100
print('estimated frequency: %.4fhz' % freq)
print('percentage diff: %.4f%%' % per_diff)


df = pd.DataFrame({'x': times, 'y': values})

plt.figure(figsize=(20, 5))
plt.title(VIDEO_NAME)
plt.plot('x', 'y', data=df, linestyle='-', marker='o')
plt.show()