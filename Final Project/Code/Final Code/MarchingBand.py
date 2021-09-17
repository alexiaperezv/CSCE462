import smbus  # import SMBus module of I2C
import time  # import time module for all time-sensitive calculations
import matplotlib.pyplot as plt  # import plotting library to visualize filter
import csv
from math import *  # import math module for trig and other math calculations
from time import sleep  # import module for delaying execution

# Next two imports allow the pi to recognize the button and the buzzer
import RPi.GPIO as GPIO
from gpiozero import Button, Buzzer

# Useful MPU6050 Registers and their Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Module Variables
accAngleX = 0
accAngleY = 0

gyroAngleX = 0
gyroAngleY = 0

roll = 0

elapsedTime = 0
currentTime = time.time()
previousTime = 0

accArray = []
gyroArray = []
rollArray = []

i = 0

initial_angle = 0


def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelerometer and Gyroscope values are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # Concatenate higher and lower value
    value = ((high << 8) | low)

    # To get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value


def calculate_IMU_error():
    # This function will help calibrate the device before it begins recrding any data #
    a = 0
    error_Ax = 0
    error_Ay = 0
    error_Gx = 0
    error_Gy = 0
    error_Gz = 0

    print("Calculating IMU error, please don't move.")

    while (a < 300):
        # Read accelerometer raw data
        Acc_x = read_raw_data(ACCEL_XOUT_H)
        Acc_y = read_raw_data(ACCEL_YOUT_H)
        Acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Sum all readings
        error_Ax = error_Ax + (180 / pi) * (atan(Acc_y / sqrt(pow(Acc_x, 2) + pow(Acc_z, 2))))
        error_Ay = error_Ay + (180 / pi) * (atan((-1 * Acc_x) / sqrt(pow(Acc_y, 2) + pow(Acc_z, 2))))

        # Read gyroscope raw data
        Gyro_x = read_raw_data(GYRO_XOUT_H)
        Gyro_y = read_raw_data(GYRO_YOUT_H)
        Gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Sum all readings
        error_Gx = error_Gx + (Gyro_x / 131.0)
        error_Gy = error_Gy + (Gyro_y / 131.0)
        error_Gz = error_Gz + (Gyro_z / 131.0)

        # Increase the iterator
        a += 1

    error_Ax = error_Ax / 300
    error_Ay = error_Ay / 300
    error_Gx = error_Gx / 300
    error_Gy = error_Gy / 300
    error_Gz = error_Gz / 300

    print("IMU error successfully calculated.")
    return error_Ax, error_Ay, error_Gx, error_Gy, error_Gz;


''' End of functions section '''

''' MAIN CODE BELOW '''
# Set up button & buzzer
GPIO.setmode(GPIO.BCM)
button = Button(22)
buzzer = Buzzer(26)

# Set up & initialize MPU
bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

# No MPU data will be collected until the button is pressed
print("Waiting for button press...")

button.wait_for_press()

# Calculate the IMU error
Ax_error, Ay_error, Gx_error, Gy_error, Gz_error = calculate_IMU_error()
time.sleep(3)

print("Reading Data of Gyroscope and Accelerometer...")

# Begin collecting data
while True:
    # Read Accelerometer raw values
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Accelerometer full scale range +/- 250 degree/C (per sensitivity scale factor)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0  # gravity

    # Convert Accelerometer readings to corresponding angle of rotation
    accAngleX = degrees(atan(Ay / sqrt(pow(Ax, 2) + pow(Az, 2)))) - Ax_error
    accAngleY = degrees(atan((-1 * Ax) / sqrt(pow(Ay, 2) + pow(Az, 2)))) - Ay_error

    # Time operations for gyroscope calculations
    previousTime = currentTime
    currentTime = time.time()
    elapsedTime = (currentTime - previousTime)

    # Read Gyroscope raw values
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Gyroscope full scale range +/- 250 degree/C (per sensitivity scale factor)
    Gx = gyro_x / 131.0 - Gx_error
    Gy = gyro_y / 131.0 - Gy_error
    Gz = gyro_z / 131.0 - Gz_error

    # Convert Gyroscope readings to corresponding angle of rotation
    gyroAngleX = -1 * (gyroAngleX + Gx * elapsedTime)  # multiply by -1 since the chip is placed upside down in the case
    gyroAngleY = gyroAngleY + Gy * elapsedTime

    # Apply Complementary Filter to get roll value
    roll = 0.94 * gyroAngleX + 0.06 * accAngleX  # rotation around x-axis (angle with respect to x-axis)

    # Calculate the average starting angle by using the first 5 roll (angle with respect to x-axis) values
    if i < 5:
        # Add roll values for the first 5 readings
        initial_angle += roll
    elif i == 5:
        # Once first 5 roll vals have been added, calculate average (dividing by 5) and set threshold angles
        initial_angle = initial_angle / 5
        low_thr = initial_angle - 5.5
        high_thr = initial_angle + 5.5
    else:
        # Determine if current roll value is within the calculated threshold
        if (roll > high_thr) or (roll < low_thr):
            buzzer.on()
        else:
            buzzer.off()

    # Append all data from reading to corresponding arrays
    accArray.append(accAngleX)
    gyroArray.append(gyroAngleX)
    rollArray.append(roll)

    i = i + 1

    if button.is_pressed:
        break;

#Uncomment to view graph with each angle
'''plt.plot(accArray)
plt.plot(gyroAngleX)
plt.plot(roll)
plt.legend(['Acc X Angle', 'Gyro X Angle', 'Roll'])
plt.show()'''

# Uncomment to store filter results in csv
'''print("Data collection & plotting completed successfully.")
print("Exporting to CSV file...")

with open("data.csv", mode="w") as csv_file:
    csv_file.write("i val," + "Acc X," + "Gyro X," + "Roll" + "\n")
    for i in range(len(accArray)):
        csv_file.write(str(i) + "," + str(accArray[i]) + "," + str(accArrayFilt[i]) + "," + str(filtRoll[i]) + "\n")
print("Successfully exported data to CSV.")'''