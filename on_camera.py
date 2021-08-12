# imports
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import inspect


GREEN = ((70, 107, 0), (93, 255, 255))
RED = ((129, 59, 6), (255, 255, 255))
BOARD = ((0, 0, 0), (255, 127, 74))
YELLOW = ((0, 87, 95), (25, 155, 145))
BLUE = ((112, 19, 68), (121, 255, 137))

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
        self.calc_flip()
        
    def calc_flip(self):
        k = self.maximum - self.minimum
        self.center = int(int(((k / 2) + self.minimum)) - k/2/4)
        self.full_left = self.minimum
        self.half_left = int(self.center-10)
        self.full_right = self.maximum
        self.half_right = int(self.center + k/2/4)
        
    def calc(self):
        k = self.maximum - self.minimum
        self.center = int(((k / 2) + self.minimum))
        self.full_left = self.minimum
        self.half_left = int(self.center - k/2/4)
        self.full_right = self.maximum
        self.half_right = int(self.center + k/2/4)

    def setup_pins(self):
        GPIO.setup(self.pin, GPIO.OUT)

    def angle(self, angle):
        duty = angle / 18 + 2
        angle = angle / 10
        self.servo1.ChangeDutyCycle(angle)
        self.servo1.ChangeDutyCycle(angle)


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

    def draw_box(self, frame, mask, min_range, max_range, draw=True, color=()):
        if self.moment(mask)[2] >= min_range:
            contours0, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            center = ()
            # перебираем все найденные контуры в цикле
            for cnt in contours0:
                ar = cv2.contourArea(cnt)
                # print(ar)
                if ar >= max_range:
                    rect = cv2.minAreaRect(cnt)  # пытаемся вписать прямоугольник
                    if rect[0][1] > 0:
                        center = tuple(rect[0])
                        if draw:
                            box = cv2.boxPoints(rect)  # поиск четырех вершин прямоугольника
                            box = np.int0(box)  # округление координат
                            cv2.drawContours(frame, [box], 0, color, 2)  # рисуем прямоугольник
                    return center, ar

    def stop(self):
        self.cam.release()
        cv2.destroyAllWindows()


class AI(Camera):  # класс автопилота
    def bricks(self, frame, hsv, min_area=1000, max_area=1500, draw_box=(True, True), draw_lines=(True, True)):
        global RED, GREEN
        # red
        mask = self.mask(hsv, RED)

        red = self.draw_box(frame, mask, min_area, max_area, draw=draw_box[0], color=(0, 0, 255))
        # print(red)
        if red:
            red_center, red_area = red
            if red_center and draw_lines[0]:
                self.line(frame, (0, 0, 255), 1, x=red_center[0], y=red_center[1])  # рисуем линии по центру
        # green
        mask = self.mask(hsv, GREEN)
        green = self.draw_box(frame, mask, min_area, max_area, draw=draw_box[1], color=(0, 255, 0))  # рисуем коробку кубика
        if green:
            green_center, green_area = green
            if green_center and draw_lines[1]:
                self.line(frame, (0, 255, 0), 1, x=green_center[0], y=green_center[1])  # рисуем линии по центру
        return red, green

    def create_cursor(self, frame, centers: tuple):
        global correct
        if centers:
            #print('centers=', centers)
            if centers[0] and not centers[1]:
                # green
                if centers[0]:
                    curse = centers[0][0][0] - correct
                    self.line(frame, (0, 255, 0), 2, x=curse)  # рисуем линию курса
                    return curse
            elif centers[1] and not centers[0]:
                # red
                if centers[1]:
                    curse = centers[1][0][0] + correct
                    self.line(frame, (0, 0, 255), 2, x=curse)  # рисуем линию курса
                    return curse
            elif centers[1] and centers[0]:
                return False
            elif not centers[1] and not centers[0]:
                return False

    def lines(self, frame, hsv, min_area=200, max_area=200, draw_box=(True, True), draw_lines=(True, True)):
        global YELLOW, BLUE
        # blue
        mask = self.mask(hsv, BLUE)
        blue = self.draw_box(frame, mask, min_area, max_area, draw=draw_box[0], color=(255, 20, 20))
        if blue:
            blue_center, blue_area = blue
            if blue_center and draw_lines[0]:
                self.line(frame, (255, 20, 20), 1, x=blue_center[0], y=blue_center[1])  # рисуем линии по центру
        # yellow
        mask = self.mask(hsv, YELLOW)
        yellow = self.draw_box(frame, mask, min_area, max_area, draw=draw_box[1], color=(20, 255, 255))
        if yellow:
            yellow_center, yellow_area = yellow
            if yellow_center and draw_lines[1]:
                self.line(frame, (255, 20, 20), 1, x=yellow_center[0], y=yellow_center[1])  # рисуем линии по центру
        return blue, yellow

    def borders(self, frame, hsv, min_area=1000, max_area=1500, draw_points=True, color=(255, 255, 255)):
        global BOARD, zones
        mask = self.mask(hsv, BOARD)
        left_zone = self.crop(mask, (0, zones), (0, 480))
        left = self.draw_box(frame, left_zone, min_area, max_area, draw=False)
        if left:
            left_center, left_area = left
            if left_center and draw_points:
                self.point(frame, left_center, color, 5)
        center_zone = self.crop(mask, (zones, 640 - zones), (0, 350))
        center = self.draw_box(frame, center_zone, min_area, max_area, draw=False)
        if center:
            center_center, center_area = center
            if center_center and draw_points:
                x, y = center_center
                x += zones
                center_center = x, y
                self.point(frame, center_center, color, 5)
        right_zone = self.crop(mask, (640 - zones, 640), (0, 210))
        right = self.draw_box(frame, right_zone, min_area, max_area, draw=False)
        if right:
            right_center, right_area = right
            if right_center and draw_points:
                x, y = right_center
                x += 640-zones
                right_center = x, y
                self.point(frame, right_center, color, 5)
        return left, center, right


class AutoPilot(AI, Motor, Drive, Servo):
    def avoid_borders(self, current_areas, half_correct=7, board_correct=13000, speed=90):
        global correction_speed
        if current_areas:
            left, _, right = current_areas
            if left and right:
                if left[1] and right[1]:
                    if left[1] - right[1] > board_correct:
                        driver.drive(dir.forward, speed)
                        servo.angle(servo.half_right+half_correct)
                        return 1
                    elif left[1] - right[1] < -board_correct:
                        servo.angle(servo.half_left-half_correct)
                        return -1
            return 0

    def rotate(self, speed):
        func = inspect.stack()[1].function
        if 'clockwise' in func:
            driver.drive(dir.forward, speed)
            servo.angle(servo.full_right+5)
            return 1
        elif 'counterclockwise' in func:
            servo.angle(servo.full_left-5)
            return 1
        return 0

    def park(self):
        global speed
        speed = 0

    def clockwise_qual(self, speed=100, rot_speed=85, cor_speed=0):
        rotates = 0
        rotate_logic = [False, False]
        start_time = time.time()
        while True:
            frame = self.capture_img()
            cropped = self.crop(frame, (0, 640), (60, 210+60))
            hsv = self.to_hsv(cropped)
            blue, yellow = self.lines(cropped, hsv, draw_lines=(False, False), draw_box=(False, False))
            # print('colors=', blue, yellow)
            board = self.borders(cropped, hsv, draw_points=False)
            # ssprint('board=', board)
            bort = self.avoid_borders(board, 7, 13000, speed-cor_speed)
            # print(bort)
            if not bort:
                if board[1]:
                    if board[1][1] > 9000:
                        self.rotate(rot_speed)
                    else:
                        servo.angle(servo.center)
                        driver.drive(dir.forward, speed)
            if yellow:
                if len(yellow) and len(yellow[0]) and yellow[0][1] > 100:
                    rotate_logic[0] = True
            if blue:
                if len(blue) and len(blue[0]) and blue[0][1] > 100:
                    rotate_logic[1] = True

            if rotate_logic == [True, True]:
                rotates += 1
                rotate_logic = [False, False]
            # print(rotates)
            if rotates == 12:
                print(print("--- %s seconds ---" % (time.time() - start_time)))
                start_time = time.time()
                rotates = 0
                break
        self.park()

    def clockwise_final(self, speed=100, rot_speed=85, cor_speed=0, cor_angle=7):
        rotates = 0
        rotate_logic = [False, False]
        start_time = time.time()
        fps_start = start_time
        fps = 0
        while True:
            frame = self.capture_img()
            cropped = self.crop(frame, (0, 640), (60, 210 + 60))
            hsv = self.to_hsv(cropped)
            blue, yellow = self.lines(cropped, hsv, draw_lines=(False, False), draw_box=(False, False))
            # print('colors=', blue, yellow)
            board = self.borders(cropped, hsv, draw_points=False)
            # print('board=', board)
            bort = self.avoid_borders(board, 7, 10000, speed - cor_speed)
            # print(bort)
            red, green = self.bricks(cropped, hsv, draw_lines=(False, False), draw_box=(False, False), max_area=3000)
            cursor = self.create_cursor(frame, (red, green))
            if cursor:
                if is_in((zones, 640-zones), cursor):
                    driver.drive(dir.forward, speed)
                    servo.angle(servo.center)
                elif is_in((-1000, 0), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.full_right)
                elif is_in((0, zones//2), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.half_right+cor_angle)
                elif is_in((zones//2, zones), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.half_right)
                elif is_in((640-zones, 640-zones+zones//2), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.half_left)
                elif is_in((640-zones, 640), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.half_left-cor_angle)
                elif is_in((640, 2000), cursor):
                    driver.drive(dir.forward, speed-15)
                    servo.angle(servo.full_left)

            elif not bort:
                if board[1]:
                    if board[1][1] > 9000:
                        self.rotate(rot_speed)
                    else:
                        servo.angle(servo.center)
                        driver.drive(dir.forward, speed)
            if yellow:
                if len(yellow) and len(yellow[0]) and yellow[0][1] > 150:
                    rotate_logic[0] = True
            if blue:
                if len(blue) and len(blue[0]) and blue[0][1] > 180:
                    rotate_logic[1] = True

            if rotate_logic == [True, True]:
                rotates += 1
                rotate_logic = [False, False]
            # print(rotates)
            if rotates == 12:
                print("--- %s seconds ---" % (time.time() - start_time))
                start_time = time.time()
                rotates = 0
            fps += 1
            if time.time() - fps_start >= 1:
                print(fps)
                fps = 0
                fps_start = time.time()

        #self.park()

    def test(self):
        while True:
            frame = ap.capture_img()
            cropped = ap.crop(frame, (0, 640), (60, 210 + 60))
            hsv = ap.to_hsv(cropped)

            print(ap.bricks(cropped, hsv))
            print(ap.borders(cropped, hsv))
            print(ap.lines(cropped, hsv, 50, 100))
            ap.show(cropped)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break


if __name__ == '__main__':
    cam_num = 0
    ap = AutoPilot(cam_num)

    GPIO.setmode(GPIO.BCM)
    servo = Servo(rng=(65, 120))
    dir = Drive()
    driver = Motor()

    speed = int(input('speed #> '))
    correction_speed = speed - 10 if speed > 70 else speed
    rotate_speed = speed-15 if speed > 70 else speed
    zones = 250
    correct = 300

    # запоминаем стартовые параметры
    frame = ap.capture_img()
    start_bricks = ap.bricks(frame, ap.to_hsv(frame), draw_box=(False, False), draw_lines=(False, False))
    start_boards = ap.borders(frame, ap.to_hsv(frame), draw_points=False)

    # road zone
    #ap.clockwise_final(speed=speed, rot_speed=rotate_speed, cor_speed=correction_speed)
    ap.test()
    # parking
    ap.stop()
