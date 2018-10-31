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


# Servo controller
class DS_SCX18S:
    def __init__(self, address=0x74, commit_delay=0.1):
        self.bus = smbus.SMBus(1)
        self.address = 0x74
        self.commit_delay = commit_delay

    def commit(self):
        self.bus.write_byte_data(self.address, 37, 0)
        time.sleep(self.commit_delay)
        if args.verbose:
            print('### Writing the global register ###')


class Servo:
    # Base parameters - [power on, not-reverse, no soft-start, speed control on]
    def __init__(self, controller, index, params=[1,0,0,0], speed=0):
        self.params = params
        self.position = 0        # [0-255]
        self.speed = speed       # Rotational speed
        self.controller = controller
        self.control_register = index * 2 
        self.position_register = index * 2 - 1  
        self.index = index
        if args.verbose:
            print(' Initialized servo {} using registers {} and {}'.format(index, self.position_register, self.control_register))
    
    # Converts the binary parameters into decimal [0-127]
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
        self.controller.bus.write_byte_data(self.controller.address, self.position_register, self.position)
        self.control()
        if args.verbose:
            print(' Moved servo {} ({}) to position {}'.format(self.index, self.position_register, self.position))
    
    # Change the parameters in the second control register
    def control(self):
        self.controller.bus.write_byte_data(self.controller.address, self.control_register, self.calc())
        if args.verbose:
            print(' Writing control register of servo {}, value {}'.format(self.index, self.calc()))

    def shutdown(self):
        self.controller.bus.write_byte_data(self.controller.address, self.control_register, 0)


class Leg:
    def __init__(self, controller, leg_number, socket, rev):
        self.reversed = rev
        self.controller = controller
        self.leg_number = leg_number # [1 - 4]
        self.socket = socket # [1 - 6]

        # Servos
        self.coxa = Servo(self.controller, (socket - 1) * 3 + 1, [1, rev, 0, 0])
        self.femur = Servo(self.controller, (socket - 1) * 3 + 2, [1, rev, 0, 0])
        self.tibia = Servo(self.controller, (socket - 1) * 3 + 3, [1, rev, 0, 0])
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

    def sin(self, axis):
        return math.sin(self.rot[axis] * math.pi / 180)

    def cos(self, axis):
        return math.cos(self.rot[axis] * math.pi / 180)

    # Use np.dot 
    # And use numpy arrays in general 
    def ik_x(self):
        a = self.total_x * self.cos(2) * self.cos(1)
        b = self.total_z * self.cos(2) * self.sin(1)
        c = self.total_y * self.sin(2) - self.total_x

        return a + b - c

    def ik_z(self):
        a = self.total_x * self.cos(0) * self.sin(1)
        b = self.total_x * self.cos(2) * self.sin(2) * self.sin(0)
        c = self.total_z * self.cos(2) * self.cos(0) * self.sin(0)
        d = self.total_y * self.sin(2) * self.sin(0) - self.total_z

        return a + b + c - d


    ################
    ## TODO : 
    # Create the links to matplotlib
    # Finish IK for legs
    
    # Gaits

    def crawl(self, n_steps):
        step = 0
        while step < n_steps:
            pass

    # Move individual joints of all legs
    def move_all(self, target, increment):
        for leg in self.legs:
            leg.servos[target].update_position(increment)
            leg.servos[target].move()
        self.controller.commit()
        
    # Set all legs to a certain position
    def set_all(self, positions):
        for leg in self.legs:
            leg.set_servos(positions)
        self.controller.commit()



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


# # RaspberryPi Camera module
# class Camera(PiCamera):
#     def __init__(self):
#         PiCamera.__init__(self)
#         #self.tilt = Servo()
#         #self.rotation = Servo()


class Robot:
    def __init__(self):
        self.controller = DS_SCX18S()
        #self.camera = Camera()
        self.body = Body(self.controller, [1, 2, 5, 6])
        self.shutdown_positions = [0, 0, 0]
        self.initial_position = [0, 0, 0]
        
    
    def initialize(self):
        pass

    # Shut all servos down
    def shutdown(self):
        for leg in self.body.legs:
            for servo in leg.servos:
                servo.shutdown()
        self.controller.commit()

    # Control one leg
    def manual_control(self):
        self.body.set_all([122, 70, 70])
        print("Manual Control Mode\n")
        step = 5
        while True:
            
            char = getch()
            if char == "w": self.body.legs[0].move_servos([0,  step, 0])
            if char == "s": self.body.legs[0].move_servos([0, -step, 0])
            if char == "a": self.body.legs[0].move_servos([ step, 0, 0])
            if char == "d": self.body.legs[0].move_servos([-step, 0, 0])
            if char == "q": self.body.legs[0].move_servos([0, 0,  step])
            if char == "e": self.body.legs[0].move_servos([0, 0, -step])
            if char == "r": step += 1
            if char == "f": step -= 1
            print(self.body.legs[0].servo_status(), end="\r")
            self.controller.commit()
            if char == "p":
                print("Exiting")
                break

    # Enable all servos
    def enable(self):
        for leg in self.body.legs:
            for servo in leg.servos:
                servo.control()
        self.controller.commit()


    # Control all legs
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
            print(self.body.legs[0].servo_status(), end="\r")
            if char == "p":
                print("Exiting")
                break


        # Control all legs
    def set_all(self):
        self.body.set_all([122, 70, 70])
        print("All legs\n")
        while True:
            char = getch()
            if char == "w": self.body.set_all([122,70,70])
            if char == "s": self.body.set_all([100,80,80])
            if char == "a": self.body.set_all([50,100,50])
            # if char == "d": self.body.move_all(1, -step)
            # if char == "q": self.body.move_all(2,  step)
            # if char == "e": self.body.move_all(2, -step)
            print(self.body.legs[0].servo_status(), end="\r")
            if char == "p":
                print("Exiting")
                break

    def run(self):
        print("M - manual control; R - distance, A - Control all legs")
        while True:
            char = getch()
            if char == "m": self.manual_control()
            if char == "a": self.move_all()
            if char == "s": self.set_all()
            # if char == "r": self.reactive()
            # if char == "v": self.reverse_all()
            # if char == "t": self.test()
            # if char == "w": self.speed_control(1)
            # if char == "s": self.speed_control(-1)
            if char == 'p':
                break
        self.shutdown()
        #GPIO.cleanup()


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

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                    action="store_true")
    args = parser.parse_args()

    robot = Robot()
    robot.enable()
    robot.run()
