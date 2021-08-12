# imports
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

ZONES = 250
CORRECT = 300
GREEN = ((70, 107, 0), (93, 255, 255))
RED = ((129, 59, 6), (255, 255, 255))
BOARD = ((0, 0, 0), (255, 127, 74))
YELLOW = ((0, 87, 95), (25, 155, 145))
BLUE = ((112, 19, 68), (121, 255, 137))
LEFT = ((0, ZONES), (0, 210))
CENTER = ((ZONES, 640 - ZONES), (0, 210))
RIGHT = ((640 - ZONES, 640), (0, 210))
board_correct = 10000
rotate_moment = 9000
cor_angle = 5
# получаем от пользователя скорость
speed = int(input('Enter speed #> ').rstrip())
correction_speed = speed - 10 if speed > 70 else speed
rotate_speed = speed - 15 if speed > 70 else speed
bricks_speed = speed - 7 if speed > 70 else speed


def is_in(rng, value):
    if min(rng) <= value <= max(rng):
        return True
    return False


class Drive:
    forward = (True, False)
    backward = (False, True)


class Motor:
    def __init__(self, in1=27, in2=22, pwm=17):
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm

        # настраиваем порты
        self.setup_pins()
        self.pwr = GPIO.PWM(self.pwm, 1000)
        self.pwr.start(0)

    def setup_pins(self):
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm, GPIO.OUT)

    def drive(self, direction, speed):
        GPIO.output(self.in1, direction[0])
        GPIO.output(self.in2, direction[1])
        self.pwr.ChangeDutyCycle(speed)


class Servo:
    def __init__(self, pin=4, rng=()):
        self.pin = pin
        self.minimum, self.maximum = rng

        # настраиваем порты
        self.setup_pins()
        self.servo1 = GPIO.PWM(self.pin, 50)
        self.servo1.start(0)
        self.calc()

    def calc_flip(self):
        k = self.maximum - self.minimum
        self.center = int(int(((k / 2) + self.minimum)) - k / 2 / 4)
        self.full_left = self.minimum
        self.half_left = int(self.center - 10)
        self.full_right = self.maximum
        self.half_right = int(self.center + k / 2 / 4)

    def calc(self):
        k = self.maximum - self.minimum
        self.center = int(((k / 2) + self.minimum))
        self.full_left = self.minimum
        self.half_left = int(self.center - k / 2 / 4)
        self.full_right = self.maximum
        self.half_right = int(self.center + k / 2 / 4)

    def setup_pins(self):
        GPIO.setup(self.pin, GPIO.OUT)

    def angle(self, angle):
        duty = angle / 18 + 2
        angle = angle / 10
        self.servo1.ChangeDutyCycle(angle)
        self.servo1.ChangeDutyCycle(angle)


#    m""                  "           ""m
#    #    mmmmm   mmm   mmm    m mm     #
#  mm"    # # #  "   #    #    #"  #    "mm
#    #    # # #  m"""#    #    #   #    #
#    #    # # #  "mm"#  mm#mm  #   #    #
#     ""                              ""

class Camera:
    def __init__(self, cam: int):
        self.cam = cv2.VideoCapture(cam)

    def capture_img(self):
        _, frame = self.cam.read()
        return frame

    def crop(self, frame, x, y):
        return frame[y[0]:y[1], x[0]:x[1]]

    def to_hsv(self, frame):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def show(self, mask=None, name=None):
        if name:
            cv2.imshow(name, mask)
        else:
            cv2.imshow('xxx', mask)

    def mask(self, hsv_image, range):
        start, stop = range
        return cv2.inRange(hsv_image, start, stop)

    def moment(self, mask):
        moments = cv2.moments(mask, 1)
        m01 = moments['m01']
        m10 = moments['m10']
        area = moments['m00']
        return m01, m10, area

    def center(self, moment):
        return int(moment[1] / moment[2])

    def line(self, frame, color: tuple, thickness: int, x=0, y=0):
        if x:
            cv2.line(frame, (x, 0), (x, 210), color, thickness)
        if y:
            cv2.line(frame, (0, y), (640, y), color, thickness)
        return 0

    def point(self, frame, center, color=(255, 255, 255), radius=1):
        cv2.circle(frame, center, radius=1, color=color, thickness=-1)

    def draw_box(self, frame, mask, min_range, max_range, draw=False, color=()):
        if self.moment(mask)[2] >= min_range:
            contours0, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            center = ()
            output = []
            # перебираем все найденные контуры в цикле
            for cnt in contours0:
                ar = cv2.contourArea(cnt)
                # print(ar)
                if ar >= max_range:
                    rect = cv2.minAreaRect(cnt)  # пытаемся вписать прямоугольник
                    output.append((rect, ar))
            if len(output):
                top = max(output, key=lambda x: x[1])
                if draw:
                    box = cv2.boxPoints(top[0])  # поиск четырех вершин прямоугольника
                    box = np.int0(box)  # округление координат
                    cv2.drawContours(frame, [box], 0, color, 2)  # рисуем прямоугольник
                return top[0][0], top[0][-1], top[1]
            return None

    def stop(self):
        self.cam.release()
        cv2.destroyAllWindows()


# камера и все остальное железо
cam = Camera(0)
servo = Servo(rng=(50, 120))
driver = Motor()
dr = Drive()
# цикл
fps = 0
fps_start = time.time()
rotates = 0
rotate_logic = [False, False]
start_time = time.time()
while True:
    DIR = None
    # получаем изоюражение
    frame = cam.capture_img()

    # обрезаеи изображение
    frame = frame[60:270, 0:640]

    # переводим в hsv
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # задаем маски
    board_mask = cam.mask(frame_hsv, BOARD)
    red_mask = cam.mask(frame_hsv, BOARD)
    green_mask = cam.mask(frame_hsv, BOARD)
    blue_mask = cam.mask(frame_hsv, BOARD)
    yellow_mask = cam.mask(frame_hsv, BOARD)

    # обрадатываем маски и обводим фигуры => (((x, y), angle, area), color)
    blue = cam.draw_box(frame, board_mask, min_range=200, max_range=220, draw=True, color=(255, 0, 0)), 'blue'
    yellow = cam.draw_box(frame, board_mask, min_range=200, max_range=220, draw=True, color=(0, 255, 255)), 'yellow'
    left_zone = cam.draw_box(cam.crop(frame, LEFT[0], LEFT[1]), cam.crop(board_mask, LEFT[0], LEFT[1]),
                             min_range=1000, max_range=1500, draw=True, color=(255, 255, 255)), 'left'
    center_zone = cam.draw_box(cam.crop(frame, CENTER[0], CENTER[1]), cam.crop(board_mask, CENTER[0], CENTER[1]),
                               min_range=1000, max_range=1500, draw=True, color=(255, 255, 255)), 'center'
    right_zone = cam.draw_box(cam.crop(frame, RIGHT[0], RIGHT[1]), cam.crop(board_mask, RIGHT[0], RIGHT[1]),
                              min_range=1000, max_range=1500, draw=True, color=(255, 255, 255)), 'right'

    # выводим изображение
    cam.show(frame)

    # едем отталкиваясь от забора
    if left_zone and right_zone and center_zone:
        if left_zone[0][2] - right_zone[0][2] >= board_correct:
            # uncomment to debug
            # print('correct left zone')
            # сбросить скорость
            # driver.drive(dr.forward, correction_speed)
            servo.angle(servo.half_right + cor_angle)
        elif left_zone[0][2] - right_zone[0][2] <= -board_correct:
            # uncomment to debug
            # print('correct right zone')
            # сбросить скорость
            # driver.drive(dr.forward, correction_speed)
            servo.angle(servo.half_left - cor_angle)
        elif center_zone[0][2] >= rotate_moment:
            driver.drive(dr.forward, rotate_speed)
            servo.angle(servo.full_right)

    # считаем повороты
    if yellow:
        if yellow[0][1] >= 150:
            rotate_logic[0] = True
    if blue:
        if blue[0][1] >= 180:
            rotate_logic[1] = True

    if rotate_logic == [True, True]:
        rotates += 1
        rotate_logic = [False, False]

    # рассичтываем и печатаем фпс
    # fps = cam.cam.get(cv2.CAP_PROP_FPS)
    # print(f'{fps=}')
    fps += 1
    cur_time = time.time()
    if cur_time - fps_start >= 1:
        print(f'fps={fps}')
        fps = 0
        fps_start = cur_time
    if rotates == 12:
        print("--- %s seconds ---" % (time.time() - start_time))
        start_time = time.time()
        rotates = 0
    # читаем кнопку выхода и выходим
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# отпускаем камеру
# удаляем все окна
cam.stop()
