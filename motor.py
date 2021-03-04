# import関連
import pigpio

# クラスの定義
class Motor():
    # コンストラクタ
    def __init__(self, left_in1, left_in2, left_vref, right_in1, right_in2, right_vref):
        self.pi = pigpio.pi()
        self.pins = [[left_in1, left_in2, left_vref], [right_in1, right_in2, right_vref]]
        for i in range(2):
            for j in range(3):
                self.pi.set_mode(self.pins[i][j], pigpio.OUTPUT)

    # モーターの回転
    def rotate(self, left_right, stop_forward_reverse_brake, duty=1.0):
        # 0 <= duty <= 1.0
        lr = 0 if left_right == 'left' else 1
        self.pi.hardware_PWM(self.pins[lr][2], 1000, int(duty*10**6))
        if stop_forward_reverse_brake == 'stop':
            self.pi.write(self.pins[lr][0], 0)
            self.pi.write(self.pins[lr][1], 0)
        elif stop_forward_reverse_brake == 'forward':
            self.pi.write(self.pins[lr][0], 0)
            self.pi.write(self.pins[lr][1], 1)
        elif stop_forward_reverse_brake == 'reverse':
            self.pi.write(self.pins[lr][0], 1)
            self.pi.write(self.pins[lr][1], 0)
        elif stop_forward_reverse_brake == 'brake':
            self.pi.write(self.pins[lr][0], 1)
            self.pi.write(self.pins[lr][1], 1)
        return

    # ストップ
    def stop(self):
        self.rotate('left', 'stop')
        self.rotate('right', 'stop')

    # 前進
    def forward(self):
        self.rotate('left', 'forward')
        self.rotate('right', 'forward')
    
    # 後進
    def backward(self):
        self.rotate('left', 'reverse')
        self.rotate('right', 'reverse')

    # ブレーキ
    def brakes(self):
        self.rotate('left', 'brake')
        self.rotate('right', 'brake')
    
    # 信地旋回
    def pivot_turn(self, left_right):
        if left_right == 'left':
            self.rotate('left', 'forward')
            self.rotate('right', 'brake')
        if left_right == 'right':
            self.rotate('left', 'brake')
            self.rotate('right', 'forward')

    # 超信地旋回
    def spin_turn(self, left_right):
        if left_right == 'left':
            self.rotate('left', 'forward')
            self.rotate('right', 'reverse')
        if left_right == 'right':
            self.rotate('left', 'reverse')
            self.rotate('right', 'forward')

if __name__ == "__main__":
    motor = Motor(20, 21, 12, 19, 26, 13)
    import time
    while True:
        motor.forward()
        time.sleep(5)
        motor.stop()
        time.sleep(5)
        motor.backward()
        time.sleep(5)
        motor.brakes()
        time.sleep(5)