"""
@author: Andrius Bernatavicius, 2018

This file contains custom classes and methods for:
     DS_SCX18S servo controller, 
     HC-SR04 Ultrasonic sound sensor,
     Servo control using inverse kinematics,
     Several quadrupedal gaits
     Keyboard control through SSH connection
"""

import RPi.GPIO as GPIO
import time, tty, termios, sys, smbus, numpy, argparse, math
import numpy as np
from picamera import PiCamera
import matplotlib.pyplot as plt
# Servo controller
class DS_SCX18S(object):
    """
    # SERVO CONTROL FUNCTIONS for DS-SCX18.S Servo controller board
    # Each connected servo has two registers, where the index of the register for servo N:
    # N * 2 - 1 = settings
    # N * 2 = position control

    # Servo settings - set the parameters for the servo
    # [A B C D S S S S] - Register legend [ 0 - disabled, 1 - enabled]
    # A - power
    # B - reverse
    # C - Soft start
    # D - Speed
    # S_n - 4bit Speed value [0-15] 
    """
    def __init__(self, address=0x74, commit_delay=0.01):
        self.bus = smbus.SMBus(1)
        self.address = 0x74
        self.commit_delay = commit_delay

    def commit(self):
        """
        Send a command to the main register (37) to execute all the changes made in servo registers
        """
        self.bus.write_byte_data(self.address, 37, 0)
        if args.verbose:
            print('### Writing the global register ###')

    def write_control_register(self, servo_index, value):
        """
        Writes the control register of the indicated servo
        """

        self.bus.write_byte_data(self.address, (servo_index-1) * 2, value)

    def write_position_register(self, servo_index, value):
        """
        Writes the position register of the indicated servo
        """
        self.bus.write_byte_data(self.address, (servo_index-1) * 2 + 1, value)

class Servo(object):
    """
    Base parameters - [power on, not-reverse, no soft-start, speed control on]
    """
    def __init__(self, controller, index, params=[1,0,0,1], speed=0):
        self.params = params
        self.position = 0        # [0-255]
        self.speed = speed       # Rotational speed
        self.controller = controller
        self.control_register = index * 2 
        self.position_register = index * 2 - 1  
        self.index = index
        if args.verbose:
            print(' Initialized servo {} using registers {} and {}'.format(index, self.position_register, self.control_register))
            print(' params: {}'.format(self.decimal_to_binary()))
    
    def decimal_to_binary(self):
        """
        Converts the binary parameters into decimal format [0-255]
        """
        value = 0
        for i, bit in enumerate(self.params):
            value += bit * 2**(7-i) 
        value += self.speed # speed control
        return value

    # Update position by a specific value [0-255]
    def update_position(self, increment):
        """
        """
        self.position += increment
        self.position = np.clip(self.position, 10, 240)

    def move(self):
        """
        Move servo to the current position by writing to the first register
        """
        self.controller.write_position_register(self.index, self.position)
        self.control()
        if args.verbose:
            print(' Moved servo {} ({}) to position {}'.format(self.index, self.position_register, self.position))
    
    def control(self):
        """
        Change the parameters in the second control register
        """
        self.controller.write_control_register(self.index, self.decimal_to_binary())
        if args.verbose:
            print(' Writing control register of servo {}, value {}'.format(self.index, self.decimal_to_binary()))

    def shutdown(self):
        """
        Disable the servo
        """
        self.controller.bus.write_byte_data(self.controller.address, self.control_register, 0)


class Leg(object):
    def __init__(self, controller, leg_number, socket, rev):
        self.reversed = rev
        self.controller = controller
        self.leg_number = leg_number # [1 - 4]
        self.socket = socket # [1 - 6]
        speed = 0
        control = 0
        # Servos
        self.coxa  = Servo(self.controller, (socket - 1) * 3 + 1, [1, 0, 0, control], speed)
        self.femur = Servo(self.controller, (socket - 1) * 3 + 2, [1, rev, 0, control], speed)
        self.tibia = Servo(self.controller, (socket - 1) * 3 + 3, [1, rev, 0, control], speed)
        self.servos = [self.coxa, self.femur, self.tibia]

        if args.verbose:
            print('Initialized Leg {}, at socket {}.'.format(leg_number, socket))

        # Dimensions
        self.cox_l = 0
        self.fem_l = 0
        self.tib_l = 0
        self.angle_offset = 0

        # Coxa position
        self.x = 0
        self.y = 0
        self.z = 0

        # Foot position
        self.fx = math.cos(60/180 * math.pi) * (self.cox_l + self.fem_l)
        self.fy = self.tib_l
        self.fz = math.sin(60/180 * math.pi) * (self.cox_l + self.fem_l)
        

    # Moves servos by a certain amount
    def move_servos(self, increments):
        for i, servo in enumerate(self.servos):
            servo.update_position(increments[i])
            servo.move()

    # Sets servos to a specific position
    def set_servos(self, positions):
        for i, servo in enumerate(self.servos):
            servo.position = positions[i]
            servo.move()

    # Print the status of servos currently
    def servo_status(self):
        line = ''
        for i, servo in enumerate(self.servos):
            line += "Servo {}: ".format(i+1) + "{:3d}".format(servo.position) + "  |  "
        return line


class Body:
    """
    Contains all the methods for motor control
    """
    def __init__(self, controller, leg_sockets):
        self.reversed = [0, 1, 0, 1]
        self.controller = controller
        self.legs = [Leg(self.controller, x, leg_sockets[x], self.reversed[x]) for x in range(len(leg_sockets))]

        # Translation
        self.trans = [0, 0, 0]

        # Roll / pitch / yaw
        self.rot = [0, 0, 0]

        # Positions
        self.total_x = [self.legs[x].fx + self.legs[x].x + self.trans[0] for x in range(len(leg_sockets))]
        self.total_z = [self.legs[x].fz + self.legs[x].z + self.trans[1] for x in range(len(leg_sockets))]
        self.total_y = [self.legs[x].fy for x in range(len(leg_sockets))]

    ################
    # IK functions #
    ################

    def sin(self, axis):
        """
        Returns the sine of the rotation angle on the specified axis
        """
        return math.sin(self.rot[axis] * math.pi / 180)

    def cos(self, axis):
        """
        Returns the cosine of the rotation angle on the specified axis
        """
        return math.cos(self.rot[axis] * math.pi / 180)

    # X component of the body inverse kinematics
    def ik_x(self):
        a = self.total_x * self.cos(2) * self.cos(1)
        b = self.total_z * self.cos(2) * self.sin(1)
        c = self.total_y * self.sin(2) - self.total_x

        return a + b - c

    # Z component of the body inverse kinematics
    def ik_z(self):
        a = self.total_x * self.cos(0) * self.sin(1)
        b = self.total_x * self.cos(2) * self.sin(2) * self.sin(0)
        c = self.total_z * self.cos(2) * self.cos(0) * self.sin(0)
        d = self.total_y * self.sin(2) * self.sin(0) - self.total_z

        return a + b + c - d

    # Move individual joints of all legs
    def move_all(self, target, increment):
        for leg in self.legs:
            leg.servos[target].update_position(increment)
            leg.servos[target].move()
        # self.controller.commit()
        
    # Set all legs to a certain position
    def set_all(self, positions):
        for leg in self.legs:
            leg.set_servos(positions)
        # self.controller.commit()


class Sensor:
    """
    HC-SR04 distance sensor class with base methods for getting the current distance.
    Default R-Pi pins - 20 for sending the pulse, 21 for getting the input
    """
    def __init__(self, trig=20, echo=21):
        self.trig = trig
        self.echo = echo
        GPIO.setup(trig,GPIO.OUT)
        GPIO.setup(echo,GPIO.IN)
        GPIO.output(trig,False)
        time.sleep(0.01)

    def getDistance(self):
        """
        Get the current distance to the sensor
        """
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


class Quadruped(object):
    """
    The class for containing all methods for:
        Unified servo control
        Gaits
        Keyboard input
        Camera and sensors
    """
    def __init__(self):
        self.controller = DS_SCX18S()
        self.body = Body(self.controller, [1, 2, 5, 6])
        self.shutdown_positions = [0, 0, 0]
        self.initial_position   = [0, 0, 0]
        
    def initialize(self):
        pass

    # Shut all servos down
    def shutdown(self):
        for leg in self.body.legs:
            for servo in leg.servos:
                servo.shutdown()
        self.controller.commit()

    # Keyboard control for 1 leg
    def manual_control(self):
        self.body.set_all([122, 70, 70])
        print("Manual Control Mode\n")

        step = 5
        def change_step(val):
            step += val

        while True:
            fn = self.body.legs[0].move_servos
            # Mapping to move servo angles to specific positions based on keyboard inputs
            mapping = {'w': fn([0,  step, 0]),
                       "w": fn([0,  step, 0]),
                       "s": fn([0, -step, 0]),
                       "a": fn([ step, 0, 0]),
                       "d": fn([-step, 0, 0]),
                       "q": fn([0, 0,  step]),
                       "e": fn([0, 0, -step]),
                       "r": change_step( 1),
                       "f": change_step(-1)}

            char = getch()
            try:
                mapping[char]
                self.controller.commit()
            except:
                pass

    # Enable all servos
    def enable(self):
        for leg in self.body.legs:
            for servo in leg.servos:
                servo.control()
        self.controller.commit()

    # Keyboard control for all legs simultaneously
    def move_all(self):
        self.body.set_all([122, 70, 70])
        step = 5
        print("All legs\n")
        while True:
            char = getch()
            if char == "w": self.body.move_all(0,  step)
            if char == "s": self.body.move_all(0, -step)
            if char == "a": self.body.move_all(1,  step)
            if char == "d": self.body.move_all(1, -step)
            if char == "q": self.body.move_all(2,  step)
            if char == "e": self.body.move_all(2, -step)
            if char == "r": step += 1
            if char == "f": step -= 1
            self.controller.commit()
            print(self.body.legs[0].servo_status(), end="\r")
            if char == "p":
                print("Exiting")
                break

    def walk(self):
        # Positions for the gait
        self.body.set_all([122, 70, 70])
        pos = [[182, 120,  60],
               [32,  120,  60],
               [100,  30,  80]]

        try:
            step = 0
            while True:
                for i in range(2):
                    self.body.legs[ i ].set_servos(pos[(step+i)%3])
                    self.body.legs[i+2].set_servos(pos[(step+i)%3])
                step += 1

                self.controller.commit()
                time.sleep(.2)
        except KeyboardInterrupt:
            pass

    def crawl(self, delay, offset=0, speed_increase=False):
        # Positions for the gait
        self.body.set_all([122, 70, 70])

        pos = np.array([ [132, 140,  100],
                         [122, 140,  100],
                         [112, 140,  100],
                         [122, 110,  140]])
        
        if offset != 0:
            pos[0,0] += offset
            pos[2,0] -= offset
            pos[3,1] -= offset*2

        try:
            step = 0
            while True:
                for i in range(4):
                    # pos = 
                    self.body.legs[ i ].set_servos(pos[(step+i)%4])
                step += 1

                if speed_increase:
                    if step%16 == 0:
                        pos[0,0] += 5
                        pos[2,0] -= 5 
                        pos[3,1] -= 5
                        delay += .005
                self.controller.commit()
                time.sleep(delay)
        except KeyboardInterrupt:
            pass

    def geometric_control(self, leg, n_steps):
        """
        Given a start and end position, interpolate the intermediate positions using a geometric progression
        """
        start = [122, 122, 122]
        end   = [200, 200, 200]
        self.body.legs[leg].set_servos(start)
        coxa  = np.geomspace(start[0], end[0], n_steps)  
        femur = np.geomspace(start[1], end[1], n_steps)
        tibia = np.geomspace(start[2], end[2], n_steps)
        for i in range(n_steps):
            self.body.legs[leg].set_servos([coxa[i], femur[i],tibia[i]])

    def run(self):
        """
        Main loop for keyboard control
        """
        print("M - manual control; R - distance, A - Control all legs")
        try:
            while True:
                char = getch()
                # if char == "m": self.manual_control()
                if char == "a": self.move_all()
                if char == "f": self.walk()
                if char == "g": self.crawl(.02, 0, True)
                if char == "z": self.crawl(.03, 20)
                if char == "x": self.crawl(.05, 40)
                if char == "c": self.crawl(.07, 60)
                if char == "v": self.crawl(.09, 70)
                if char == "b": self.crawl(.15, 60)
                if char == "n": self.crawl(.3, 50)
                if char == "m": self.crawl(.3, 25)
                if char == "t": self.geometric_control(0, 10)
                if char == 'p':
                    break
                else:
                    pass
        except KeyboardInterrupt:
            self.shutdown()
        self.shutdown()

def getch():
    """
    Read keyboard inputs
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        char = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
    return char


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true")
    args = parser.parse_args()

    robot = Quadruped()
    robot.enable()
    robot.run()
    plt.plot(np.array([1,2,3,4]))
    plt.show()
