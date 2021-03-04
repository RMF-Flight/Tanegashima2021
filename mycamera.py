import os
import shutil
import time
import cv2
import picamera


class MyCamera():
    # 初期化
    def __init__(self, height=256, width=256, makefolder=True):
        self.height = height
        self.width = width
        if makefolder:
            path = './pictures/'
            os.makedirs(path, exist_ok=True)
            shutil.rmtree(path)
            os.makedirs(path, exist_ok=True)

    # 拡張子分離
    def _separate(self, filepath):
        return filepath[:-4], filepath[-4:]

    # 撮影
    def take_a_picture(self, filepath):
        with picamera.PiCamera() as camera:
            camera.resolution = (self.height, self.width)
            #camera.start_preview()
            time.sleep(2)
            camera.capture(filepath)

    # コーン位置推定  (theresh=閾値, progress=画像処理の途中経過を記録)
    def analysis(self, filepath, thresh=150, progress=False):
        img = cv2.imread(filepath)
        name = self._separate(filepath)

        # 色調変換
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ガウス変換
        gauss = cv2.GaussianBlur(hsv,(9, 9),3)
        
        # 色調分割
        img_H, img_S, img_V = cv2.split(gauss)

        # 2値化
        _thre, img_mask = cv2.threshold(img_H, thresh, 255, cv2.THRESH_BINARY)

        # 変換途中の画像の保存
        if progress:
            # Hだけ分離した画像の準備
            img_S.fill(255)
            img_V.fill(255)
            hsv = cv2.merge([img_H, img_S, img_V])
            out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

            # 保存
            cv2.imwrite('_1_hsv'.join(name), hsv)
            cv2.imwrite('_2_gauss'.join(name), gauss)
            cv2.imwrite('_3_img_H'.join(name), img_H)
            cv2.imwrite('_4_img_HSV'.join(name), out)
            cv2.imwrite('_5_img_mask'.join(name), img_mask)

        # 輪郭抽出 & 最大輪郭の情報取得
        contours, _ = cv2.findContours(img_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        ind = -1
        m = 0
        cx, cy, area = 0, 0, 0
        for i, c in enumerate(contours):
            if len(c) > 0:
                cv2.polylines(img, c, True, (255, 255, 255), 1)
                if cv2.contourArea(c) > m:
                    ind = i
                    m = cv2.contourArea(c)
        if ind != -1:
            cnt = contours[ind]
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            area = cv2.contourArea(cnt)
            img = cv2.line(img, (int(cx),0), (int(cx),self.height), (255, 255, 255), 1)
            img = cv2.line(img, (0,int(cy)), (self.width,int(cy)), (255, 255, 255), 1)
        cv2.imwrite('_process'.join(name), img)

        return (cx / self.width), (cy / self.height), (area / (self.height*self.width))


if __name__ == "__main__":
    mycamera = MyCamera()
    for i in range(1, 10**6):
        print('Enter', end='')
        while True:
            if input() == '':
                break
        filepath = './pictures/p{:03}.jpg'.format(i)
        mycamera.take_a_picture(filepath)
        print(i, mycamera.analysis(filepath, progress=True))
