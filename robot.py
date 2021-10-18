import time

import wiringpi

# Left motor GPIO number
L_MOTOR_FIRST = 23
L_MOTOR_SECOND = 24
# Right motor GPIO number
R_MOTOR_FIRST = 25
R_MOTOR_SECOND = 26


class RobotController:

    # Constructor
    def __init__(self):
        # Init wiringpi
        wiringpi.wiringPiSetupGpio()
        # Pin mode set as an output
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

    def stop(self):
        wiringpi.softPwmWrite(L_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(L_MOTOR_SECOND, 0)
        wiringpi.softPwmWrite(R_MOTOR_FIRST, 0)
        wiringpi.softPwmWrite(R_MOTOR_SECOND, 0)

    def move_forward(self):
        startTime = time.time()
        elapsedTime = time.time()
        try:
            # Move forward 3 sec
            while elapsedTime - startTime < 3:
                wiringpi.softPwmWrite(L_MOTOR_FIRST, 0)
                wiringpi.softPwmWrite(L_MOTOR_SECOND, 80)
                wiringpi.softPwmWrite(R_MOTOR_FIRST, 0)
                wiringpi.softPwmWrite(R_MOTOR_SECOND, 80)
                elapsedTime = time.time()

        except KeyboardInterrupt:
            print("Keyboard interrupt.")
            self.stop()

        self.stop()

        # Waiting stop the motor completely
        time.sleep(1)

    def move_backward(self):
        startTime = time.time()
        elapsedTime = time.time()
        try:
            # Move backward 3 sec
            while elapsedTime - startTime < 3:
                wiringpi.softPwmWrite(L_MOTOR_FIRST, 80)
                wiringpi.softPwmWrite(L_MOTOR_SECOND, 0)
                wiringpi.softPwmWrite(R_MOTOR_FIRST, 80)
                wiringpi.softPwmWrite(R_MOTOR_SECOND, 0)
                elapsedTime = time.time()

        except KeyboardInterrupt:
            print("Keyboard interrupt.")
            self.stop()

        self.stop()

        # Waiting stop the motor completely
        time.sleep(1)


def main():
    robotController = RobotController()
    robotController.move_backward()
    robotController.move_forward()

    print("done")


if __name__ == "__main__":
    main()
