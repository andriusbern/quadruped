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
