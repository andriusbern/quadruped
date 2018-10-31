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

class Servo:
    # Base parameters - [power on, not-reverse, no soft-start, speed control on]
    def __init__(self, index, position, params=[1,0,0,1], speed=14):
        self.params = params
        self.index = index       # Socket number on DS-SCX18.S [1-18]
        self.position = position # [0-255]
        self.parent_address = 0x74 # SMBus number of the controller
        self.speed = speed       # Rotational speed
    
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
        if self.position > 240:
            self.position = 240
        if self.position < 10:
            self.position = 10

    # Move servo to the current position by writing to the first register
    def move(self):
        bus.write_byte_data(address, self.index * 2 - 1, self.position)
    
    # Change the parameters in the second control register
    def control(self):
        bus.write_byte_data(address, self.index * 2, self.calc())

    def shutdown(self):
        bus.write_byte_data(address, self.index * 2, 0)

    def sin(self):
        pass

    def cos(self):
        pass



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


class Leg:
    def __init__(self, leg_number):
        
        # Positions
        self.angular_offset = 0
        self.position = [0,0,0] # XYZ Position
        self.leg_number = 0

        # Lengths
        self.coxa = Servo((self.leg_number-1)*3, 122)
        self.coxa_length = 0  # Need to measure
        self.coxa_start = 0
        self.coxa_end = 0

        self.femur = Servo((self.leg_number-1)*3+1, 122)
        self.femur_length = 0
        self.femur_start = 0
        self.femur_end = 0

        self.tibia = Servo((self.leg_number-1)*3+2, 122)
        self.tibia_length = 0
        self.tibia_start = 0
        self.tibia_end = 0
        self.servos = [self.tibia, self.femur, self.coxa]
    

    # Increment/decrement current servo positions
    def move_servos(self, leg_index, increments):
        for i, servo in enumerate(self.body.legs[leg_index].servos):
            servo.update_position(increments[i])
            servo.move()
        self.commit()

    def set_servos(self, leg_index, positions):
        for i, servo in enumerate(self.servos):
            servo.position = positions[i]
            servo.move()
            servo.control()
        self.commit()


    # LEG IK
    def coxa_angle(self):
        pass

    def servo_status(self):
        line = ''
        for servo in self.servos:
            line += "Servo {}: ".format(servo.index) + "{:3d}".format(servo.position) + "  |  "
        return line


class Body:
    def __init__(self, n_legs=2):

        # Rotations
        self.rot_x = 0
        self.rot_y = 0
        self.rot_z = 0

        # Translation
        self.trans_x = 0
        self.trans_y = 0
        self.trans_z = 0

        self.center_to_leg = 0

        self.body_coxa_offset_x = self.center_to_leg/2
        self.body_coxa_offset_z = math.sqrt()
        self.body_coxa_offset_y = 0

        self.legs = [Leg(i) for i in range(1,2)]
        self.center = [0, 0, 0]

    def move(self):
        pass
    
    # BODY IK
    # Roll, pitch, yaw, translate


class Agent:
    def __init__(self, initial_positions=[122,122,122], n_servos=3, address=0x74):

        self.initial_positions = initial_positions
        self.servos = [Servo(x+1, initial_positions[x-3], address) for x in range(0, 3)]
        self.sensor = Sensor()
        self.address = address
        self.commit_delay = 0.01
        self.step = 3
        self.body = Body()

    def commit(self):
        bus.write_byte_data(self.address, 37, 0)
        time.sleep(self.commit_delay)

    def control(self, speed=5, control=[1, 0, 0, 1], init=False):
        for servo in self.servos:
            servo.control()
            if init:
                servo.move()
            self.commit()

    def speed_control(self, direction):
        start = self.servos[0].speed
        for servo in self.servos:
            servo.speed += direction
            servo.control()
        self.commit()
        print("Speed: {} -> {}".format(start, self.servos[0].speed))




    def shutdown(self):
        for servo in self.servos:
            servo.shutdown()
            self.commit()


    def manual_control(self):
        self.set_servos([122, 122, 122])
        print("Manual Control Mode\n")
        leg_no = input("Enter the leg number [1-4] or 0 for all: \n")
        while True:
            
            char = getch()
            if char == "w": self.move_servos(leg_no, [0,  self.step, 0])
            if char == "s": self.move_servos(leg_no, [0, -self.step, 0])
            if char == "a": self.move_servos(leg_no, [ self.step, 0, 0])
            if char == "d": self.move_servos(leg_no, [-self.step, 0, 0])
            if char == "q": self.move_servos(leg_no, [0, 0,  self.step])
            if char == "e": self.move_servos(leg_no, [0, 0, -self.step])
            if char == "r": self.step += 1
            if char == "f": self.step -= 1
            print(self.body.legs[leg_no-1].servo_status(), end="\r")
            if char == "p":
                print("Exiting")
                break
    
    def test(self):
        try:
            while True:
                print(self.sensor.getDistance(), end="\r")
        except KeyboardInterrupt:
            pass


    def reactive(self):
        s = 3
        a = 0
        angles = [82, 207, 162]

        self.set_servos(angles)
        # Keep a history of distances
        try:
            while True:
                d = self.sensor.getDistance()
                if d > 2 and d < 25:
                    if d > 2 and d < 6:
                        a = 75
                        print(self.servo_status() + "    Fleeing" + "   Distance: {}".format(d), end="\r")
                    elif d > 4 and d < 10:
                        a = s
                        print(self.servo_status() + "    Backing off" + "   Distance: {}".format(d), end="\r")
                    elif d < 25 and d > 15:
                        a = -s
                        print(self.servo_status() + "    Approaching" + "   Distance: {}".format(d), end="\r")
                    else:
                        continue
                        print(self.body.legs[leg_no-1].servo_status(), end="\r")
                self.move_servos([0, a, a])
                time.sleep(0.01)


        except KeyboardInterrupt:
            self.shutdown()
            print("\nENDING...")
           
    def run(self):
        print("M - manual control; R - distance")
        while True:
            char = getch()
            if char == "m": self.manual_control()
            if char == "r": self.reactive()
            if char == "v": self.reverse_all()
            if char == "t": self.test()
            if char == "w": self.speed_control(1)
            if char == "s": self.speed_control(-1)
            if char == 'p':
                break
        self.shutdown()
        GPIO.cleanup()


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

    # DS-SCX18.S
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(20,GPIO.OUT)
    GPIO.setup(21,GPIO.IN)
    GPIO.output(20,False)
    time.sleep(0.01)

    bus = smbus.SMBus(1)
    address = 0x74 # DS-SCX18.S address (i2c)

    robot = Agent()
    robot.control(init=True)
    robot.run()
