import RPi.GPIO as GPIO
import time, tty, termios, sys, smbus, numpy, argparse, math
from picamera import PiCamera

# SERVO CONTROL FUNCTIONS for DS-SCX18.S Servo controller board
# Each servo has two registers, where the index of the register for servo N:
# N * 2 - 1 = settings
# N * 2 = position control

# Servo settings - set the parameters for the servo
# [A B C D S S S S] - Register legend [ 0 - disabled, 1 - enabled]
# A - power
# B - reverse
# C - Soft start
# D - Speed
# S_n - Speed value [0-15]

class DS_SCX18S:
    def __init__(self, address=0x74, execute_delay=0.01):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.execute_delay = execute_delay

    # Executes all the changes in servo registers
    def execute(self):
        self.bus.write_byte_data(self.address, 37, 0)
        time.sleep(self.execute_delay)


class Servo:
    # Base parameters - [power on, not-reverse, no soft-start, speed control on]
    def __init__(self, controller, index, position, params=[1,0,0,1], speed=14):
        self.params = params
        self.position = position # [0-255]
        self.initial_position = position
        self.speed = speed       # Rotational speed
        self.controller = controller
        self.control_register = index * 2
        self.position_register = index * 2 - 1
        self.index = index
    
    # Calculates the current position based on params [0-127]
    def calc(self):
        value = 0
        for i, bit in enumerate(self.params):
            value += bit * 2**(7-i) 
        value += self.speed # speed control
        return value

    # Update position by a specific value [0-255]
    def update_position(self, increment):
        self.position += increment
        if self.position > 240: self.position = 240
        if self.position < 10:  self.position = 10

    # Move servo to the current position by writing to the first register
    def move(self):
        self.controller.bus.write_byte_data(self.controller.address, self.control_register, self.position)
    
    # Change the parameters in the second control register
    def control(self):
        self.controller.bus.write_byte_data(self.controller.address, self.position_register, self.calc())

    def shutdown(self):
        self.controller.bus.write_byte_data(self.controller.address, self.control_register, 0)


# Distance sensor classs
class Sensor:
    def __init__(self, trig=20, echo=21):
        self.trig = trig
        self.echo = echo
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trig,GPIO.OUT)
        GPIO.setup(echo,GPIO.IN)
        GPIO.output(trig,False)
        time.sleep(0.01)

    def getDistance(self):
        GPIO.output(self.trig,True)
        time.sleep(0.001)
        GPIO.output(self.trig,False)
        pulse_end, pulse_start = (0, 0)
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()
        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()

        distance = round((pulse_end - pulse_start) * 17150, 2)
        return distance


# RaspberryPi Camera module
class Camera(PiCamera):
    def __init__(self):
        PiCamera.__init__(self)
        #self.tilt = Servo()
        #self.rotation = Servo()
    

class Leg:
    def __init__(self, controller, leg_number):
        self.controller = controller

        self.coxa  = Servo(controller, (leg_number-1)*3, 122)
        self.femur = Servo(controller, (leg_number-1)*3+1, 122)
        self.tibia = Servo(controller, (leg_number-1)*3+2, 122)
        self.servos = [self.coxa, self.femur, self.tibia]

    # Increment/decrement current servo positions
    def move_servos(self, increments):
        for i, servo in enumerate(self.servos):
            servo.update_position(increments[i])
            servo.move()
        self.controller.execute()

    def set_servos(self, leg_index, positions):
        for i, servo in enumerate(self.servos):
            servo.position = positions[i]
            servo.move()
        self.controller.execute()

    def servo_status(self):
        line = ''
        for servo in self.servos:
            line += "Servo {}: ".format(servo.index) + "{:3d}".format(servo.position) + "  |  "
        return line


class Body:
    def __init__(self, controller, n_legs=4):
        self.legs = [Leg(controller, x+1) for x in range(n_legs)]

    


class Robot:
    def __init__(self):

        self.camera = Camera()
        self.controller = DS_SCX18S()
        self.sensor = Sensor()
        self.body = Body(self.controller)
        self.step = 3


    def initialize(self):
        for leg in self.body.legs:
            for servo in leg.servos:
                servo.move(servo.initial_position)
        self.controller.execute()

    def manual_control(self):
        self.body.set_servos([122, 122, 122])
        print("Manual Control Mode\n")
        leg_no = input("Enter the leg number [1-4] or 0 for all: \n")
        while True:
            
            char = getch()
            if char == "w": self.body.legs[leg_no].move_servos([0,  self.step, 0])
            if char == "s": self.body.legs[leg_no].move_servos([0, -self.step, 0])
            if char == "a": self.body.legs[leg_no].move_servos([ self.step, 0, 0])
            if char == "d": self.body.legs[leg_no].move_servos([-self.step, 0, 0])
            if char == "q": self.body.legs[leg_no].move_servos([0, 0,  self.step])
            if char == "e": self.body.legs[leg_no].move_servos([0, 0, -self.step])
            if char == "r": self.step += 1
            if char == "f": self.step -= 1
            print(self.body.legs[leg_no-1].servo_status(), end="\r")
            if char == "p":
                print("Exiting")
                break

## Read keyboard inputs
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        char = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
    return char



if __name__ == "__main__":
    robot = Robot()
    robot.control(init=True)
    robot.run()
