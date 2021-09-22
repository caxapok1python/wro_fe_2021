from wiringpi import *

wiringPiSetupGpio()

def pwr(proc: int):
    return int((255*proc) / 100)


def inRange(rang, val):
    start, stop = rang
    return start <= val <= stop


class Motor:
    def __init__(self, conf):
        self.IN_A, self.IN_B, self.PWR = conf

        self.setup_pins()

    def setup_pins(self):
        pinMode(self.IN_A, GPIO.PWM_OUTPUT)
        pinMode(self.IN_B, GPIO.PWM_OUTPUT)
        pinMode(self.PWR, GPIO.PWM_OUTPUT)

    def forward(self, power: int):
        pass


class HCSR04:
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

    def setup(self):
        pinMode(self.trig, GPIO.PWM_OUTPUT)
        pinMode(self.echo, GPIO.PWM_INPUT)

    def get_dist(self):

        digitalWrite(sel.trig, 0)
        delayMicroseconds(5)
        digitalWrite(self.trig, 1)


        delayMicroseconds(10)
        digitalWrite(self.trig, 0)

        duration = pulseIn(self.echo, 1)

        dist = (duration / 2) / 29.1
        return int(dist)

class Servo:
    def __init__(self, pin):
        self.logic = pin
        self.max_angle = 180
        self.center = 60

    def angle_to_shim(self, angle):
        return (255 * angle) / self.max_angle

    def setup(self):
        pinMode(self.logic, GPIO.PWM_OUTPUT)
        pwmWrite(self.logic, self.angle_to_shim(self.center))

    def angle(self, angle):
        pwmWrite(self.logic, self.angle_to_shim(angle))


class AI:
    def __init__(self):
        self.stright = (220, 430)
        self.conditions()

    def conditions(self):
        road = ((60, 150), (0, 70), (0, 70))
        rotate = ((0, 60), (70, 300), (70, 300))
        out = ((150, 300), (70, 300), (70, 300))
        self.pos = (out, road, rotate)

    def sonic_range(self, sonic, position, mode):
        out = False
        if mode == 'front':
            if inRange(position[0], sonic[0]):
                out = True
        elif mode == 'left':
            if inRange(position[1], sonic[1]):
                out = True
        elif mode == 'right':
            if inRange(position[2], sonic[2]):
                out = True
        return out

    def on_road(self, cam):
        ax = (cam[-2], cam[-1])
        for i in range(2):
            if inRange(self.stright, ax[i]):
                return True
        return False


    def detect_zone(self, sonic, cam):
        # cam = (red, green, red_x, green_x)
        # sonic = (front, left, right)
        positions = ('out', 'road', 'rotate')
        current = positions[0]
        for position in range(3):
            if position == 1 and not self.on_road(cam):
                if self.sonic_range(sonic, self.pos[position][0], 'front'):
                    current = positions[position]
        return current

    def line(self, zone, sonic):
        lines = ('left', 'center', 'right')
        # if zone == 'road':
        #     for i in range(3):
        #         if self.sonic_range(sonic)


def setup_controls():
    global servo, front, left, right
    servo.setup()
    front.setup()
    left.setup()
    right.setup()





def main():
    servo = Servo(pin=14)
    front = HCSR04(trig=17, echo=27)
    left = HCSR04(trig=23, echo=24)
    right = HCSR04(trig=20, echo=21)
    setup_controls()
    while True:
        f_cm = front.get_dist()
        l_cm = left.get_dist()
        r_cm = right.get_dist()
        print(":".join(map(str, (f_cm, l_cm, r_cm))))



if __name__ == '__main__':


