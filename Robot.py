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
    def __init__(self, address=0x74, commit_delay=0.01):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.commit_delay = commit_delay

    def commit(self):
        self.bus.write_byte_data(self.address, 37, 0)
        time.sleep(self.commit_delay)


class Servo:
    # Base parameters - [power on, not-reverse, no soft-start, speed control on]
    def __init__(self, controller, index, position, params=[1,0,0,1], speed=14):
        self.params = params
        self.position = position # [0-255]
        self.speed = speed       # Rotational speed
        self.controller = controller
        self.control_register = index * 2
        self.position_register = index * 2 - 1
    
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
    
    # Implement functions through opencv to do object tracking and recognition

