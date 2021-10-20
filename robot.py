import time

import wiringpi

# Left motor GPIO number
L_MOTOR_FIRST = 23
L_MOTOR_SECOND = 24
# Right motor GPIO number
R_MOTOR_FIRST = 25
R_MOTOR_SECOND = 26
# Ultrasonic sensor GPIO number
TRIG = 16
ECHO = 17


class RobotController:

    # Constructor
    def __init__(self):
        # Init wiringpi
        wiringpi.wiringPiSetupGpio()
        # Motor pin set as an output
        wiringpi.pinMode(L_MOTOR_FIRST, wiringpi.OUTPUT)
        wiringpi.pinMode(L_MOTOR_SECOND, wiringpi.OUTPUT)
        wiringpi.pinMode(R_MOTOR_FIRST, wiringpi.OUTPUT)
        wiringpi.pinMode(R_MOTOR_SECOND, wiringpi.OUTPUT)
        # PWM setup
        wiringpi.softPwmCreate(L_MOTOR_FIRST, 0, 100)
        wiringpi.softPwmCreate(L_MOTOR_SECOND, 0, 100)
        wiringpi.softPwmCreate(R_MOTOR_FIRST, 0, 100)
        wiringpi.softPwmCreate(R_MOTOR_SECOND, 0, 100)
        # Each motor set as zero
        wiringpi.softPwmWrite(L_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(L_MOTOR_SECOND, 0)
        wiringpi.softPwmWrite(R_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(R_MOTOR_SECOND, 0)
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(TRIG, wiringpi.OUTPUT)
        wiringpi.pinMode(ECHO, wiringpi.INPUT)

    def stop(self):
        wiringpi.softPwmWrite(L_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(L_MOTOR_SECOND, 0)
        wiringpi.softPwmWrite(R_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(R_MOTOR_SECOND, 0)

    def move_forward(self):
        wiringpi.softPwmWrite(L_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(L_MOTOR_SECOND, 80)
        wiringpi.softPwmWrite(R_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(R_MOTOR_SECOND, 80)

    def move_backward(self):
        wiringpi.softPwmWrite(L_MOTOR_FIRST, 80)
        wiringpi.softPwmWrite(L_MOTOR_SECOND, 0)
        wiringpi.softPwmWrite(R_MOTOR_FIRST, 80)
        wiringpi.softPwmWrite(R_MOTOR_SECOND, 0)

    def read_distance(self):
        wiringpi.digitalWrite(TRIG, wiringpi.GPIO.HIGH)
        # Wait 10 micor sec.
        time.sleep(0.00001)
        wiringpi.digitalWrite(TRIG, wiringpi.GPIO.LOW)

        signalOff = 0
        signalOn = 0

        while wiringpi.digitalRead(ECHO) == wiringpi.GPIO.LOW:
            signalOff = time.time()

        while wiringpi.digitalRead(ECHO) == wiringpi.GPIO.HIGH:
            signalOn = time.time()

        duration = signalOn - signalOff
        distance = duration * 34000 / 2

        time.sleep(0.1)

        return distance


def main():
    robotController = RobotController()
    try:
        while True:
            distance = robotController.read_distance()
            print("Distance: {}".format(distance))
            robotController.move_forward()

            if distance < 15.0:
                robotController.stop()
    finally:
        robotController.stop()


if __name__ == "__main__":
    main()
