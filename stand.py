import smbus
import time

bus = smbus.SMBus(1)

address = 0x74

def writeRegister(register, value):
    bus.write_byte_data(address, register, value)

def writeServo(servo_n, pos, rev, speed):
    val = 128 + speed
    if rev:
        val += 64 
    bus.write_byte_data(address, servo_n * 2 - 1, pos)
    bus.write_byte_data(address, servo_n * 2, val)

# Leg parameters
rev = [0, 0, 0, 0]
coxas = [1, 4, 13, 16]
femurs = [2, 5, 14, 17]
tibias = [3, 6, 15, 18]

speed = [0, 0, 0]
coxa_angles  = [50, 128, 200]
femur_angles = [128, 128, 128]
tibia_angles = [128, 128, 128]



try:
    while True:
        for i in range(len(coxa_angles)):
            for j in range(4):
                writeServo(coxas[j], coxa_angles[i], rev[j], speed[i])
                writeServo(femurs[j], femur_angles[i], rev[j], speed[i])
                writeServo(tibias[j], tibia_angles[i], rev[j], speed[i])

            writeRegister(37, 0)
            time.sleep(0.75)

except KeyboardInterrupt:
    for k in range(len(coxa_angles)):
        writeServo(coxas[k], -128, 0, 0)
        writeServo(femurs[k], -128, 0, 0)
        writeServo(tibias[k], -128, 0, 0)
        writeRegister(37,0)
