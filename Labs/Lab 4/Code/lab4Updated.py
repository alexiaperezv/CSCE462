import smbus            #import SMBus module of I2C
from time import sleep, perf_counter
import matplotlib.pyplot as plt

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

stepCount = 0
possibleStep = 0
array = []
arrayFiltered = []
past = perf_counter()
restingTotal = 0.0
num = 0
i=-1

while stepCount < 50:
    #Read Accelerometer raw value
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Az = acc_z/16384.0
    
    array.append(Az)
    i = i+1
    if (i < 2):
        continue
    arrayFiltered.append((array[i] + array[i-1] + array[i-2]) / 3.0)
    Az = arrayFiltered[i-2]
    
    if (i == 2):
        restingTotal = restingTotal + Az
        num = num + 1
    
    restingAverage = restingTotal/num
    tol = 0.03 * restingAverage
    
    if(Az > restingAverage + tol and possibleStep == 0 and num > 20):
        possibleStep += 1
    elif(Az < restingAverage - tol and possibleStep == 1): #and perf_counter() - past > 0.7):
        stepCount += 1
        possibleStep = 0
        print("Steps=%d" %stepCount, "\tAcc_Z: %.2f" %Az)
        past = perf_counter()
    else:
        restingTotal += Az
        num += 1

    sleep(0.05)