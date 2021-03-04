# import関連
import math
import signal
import sys
import threading
import time
import numpy as np
import serial
import bmx055
import micropyGPS
import motor
import mycamera
import cv2

# インスタンス生成
bmx = bmx055.Bmx055()
gps = micropyGPS.MicropyGPS(9, 'dd')
mtr = motor.Motor(20, 21, 12, 19, 26, 13)
camera = mycamera.MyCamera()

# シリアル通信のためのインスタンス生成
ser = serial.Serial('/dev/serial0', 9600, timeout=10)

# GPSオブジェクトを更新するスレッドを起動
def rungps():
    ser.readline()
    while True:
        a = ser.readline()
        if sys.getsizeof(a) < 100:
            sentence = a.decode('utf-8')
            if sentence[0] != '$':
                continue
            for x in sentence:
                gps.update(x)

gpsthread = threading.Thread(target=rungps, args=())
gpsthread.daemon = True
gpsthread.start()

# 落下検知
def is_falling_began(path='./pictures/0.jpg', grayscale_threshold=128, size_threshold=0.9):
    camera.take_a_picture(path)
    img = cv2.imread(path)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_thresh = cv2.threshold(img_gray, grayscale_threshold, 255, cv2.THRESH_BINARY)
    count = np.count_nonzero(img_thresh == 0)
    ratio = count / (camera.height * camera.width)
    return True if ratio < size_threshold else False

# 着地検出
def is_landed(loop=10, error=1):
    accs = np.zeros((loop, 3))
    for i in range(loop):
        accs[i] = bmx.get_acc_value()
        time.sleep(0.5)
    acc = bmx.get_acc_value()
    return True if np.all(abs(accs - acc) < error) else False

# パラシュート切り離し検出
def is_released_parachute():
    return

# クォータニオンの値がが安定しているか確認
def is_quaternion_stable(loop=10, error=0.01, deltat=0.1):
    quaternions = np.zeros((loop, 4))
    for i in range(loop):
        bmx.update_quaternion_with_Madgwick_filter(deltat)
        quaternions[i] = bmx.q
        time.sleep(deltat)
    bmx.update_quaternion_with_Madgwick_filter(deltat)
    qua = bmx.q
    return True if np.all(abs(quaternions - qua) < error) else False

# Tweliteによるメッセージ送信
def send_message_by_twelite(txt):
    # Tweliteは、App_Uartに書き換えて、透過モードに設定
    txt = txt + '\r\n'
    enc = txt.encode()
    ser.write(enc)

# ターミナルへの表示とTweliteのメッセージ送信
def print_and_send(txt):
    send_message_by_twelite(txt)
    print(txt)

# 緯度経度の取得
def get_latitude_and_longitude():
    while True:
        if gps.clean_sentences > 20:
            latitude_and_longitude = np.array([gps.latitude[0], gps.longitude[0]])
            break
        time.sleep(2)
    return latitude_and_longitude

# 現在地と目標地との距離を取得
def get_distance(current_position, destination):
    EQUATORIAL_RADIUS = 6378.137    # (km)
    y1, x1 = np.deg2rad(current_position)
    y2, x2 = np.deg2rad(destination)
    r = EQUATORIAL_RADIUS
    dx = x2 - x1
    sy1, cy1 = math.sin(y1), math.cos(y1)
    sy2, cy2 = math.sin(y2), math.cos(y2)
    distance = r * math.acos(sy1*sy2 + cy1*cy2*math.cos(dx))
    return distance

# 方位角を取得（緯度経度から）
def get_azimuth_using_GPS(current_position, destination):
    y1, x1 = np.deg2rad(current_position)
    y2, x2 = np.deg2rad(destination)
    dx = x2 - x1
    sy1, cy1 = math.sin(y1), math.cos(y1)
    ty2 = math.tan(y2)
    sdx, cdx = math.sin(dx), math.cos(dx)
    azimuth = (450 - np.rad2deg(math.atan2(cy1*ty2-sy1*cdx, sdx))) % 360
    return azimuth

# 方位角を取得（オイラー角から）
def get_azimuth_using_9axes(euler_angle):
    # 度数法で表された角度を引数に持つことに注意
    euler_angle = (euler_angle + 450) % 360
    azimuth = -euler_angle % 360
    return azimuth

# 現在向いている方向と目標地の方向との差を取得
def get_azimuth_difference(destination):
    euler_angle = np.rad2deg(bmx.qua2eul(bmx.q)[2])
    azimuth_9axes = get_azimuth_using_9axes(euler_angle)
    current_position = get_latitude_and_longitude()
    azimuth_GPS = get_azimuth_using_GPS(current_position, destination)
    difference = azimuth_9axes - azimuth_GPS
    return difference

# 目標地方向に向く
def head_toward_distination(destination, error=5, delta=0.5):
    diff = get_azimuth_difference(destination)
    diff %= 360
    mtr.spin_turn('left') if 0 <= diff <= 180 else mtr.spin_turn('right')
    time.sleep(int(diff))
    mtr.stop()

# カメラを用いて目標方向に向く
def head_toward_distination_using_camera(i):
    filepath = './pictures/p{:03}.jpg'.format(i)
    camera.take_a_picture(filepath)
    width, _, area = camera.analysis(filepath, progress=True)
    mtr.spin_turn('left') if width < 0.5 else mtr.spin_turn('right')
    time.sleep(abs(width-0.5)*2)
    mtr.stop()
    return area


#　目的地の設定
destination = np.array([0, 0])

# 開始
print_and_send('--- Started ---')

# 落下検知
while not is_falling_began():
    pass
print_and_send('--- Began to fall ---')
time.sleep(10)

# 着地検知
while not is_landed():
    time.sleep(1)
print_and_send('--- Landed ---')

# パラシュートの切り離し検知
while not is_released_parachute():
    time.sleep(1)
print_and_send('--- Released parachute ---')

# GPSを用いて目的地に近づく
while get_distance(get_latitude_and_longitude(), destination) > 5:
    while not is_quaternion_stable():
        pass
    head_toward_distination(destination)
    mtr.forward()
    time.sleep(int(get_distance(get_latitude_and_longitude(), destination)))
    mtr.stop()
    time.sleep(3)
print_and_send('--- Got closer to destination ---')

# カメラを用いて目的地に近づく
for i in range(1, 10**6):
    area = head_toward_distination_using_camera(i)
    if area > 0.9:
        print_and_send('--- Arrived ---')
        break
    mtr.forward()
    time.sleep((1-area)*10)
    mtr.stop()
    time.sleep(3)