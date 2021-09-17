from gpiozero import Button
import Adafruit_MCP4725
import RPi.GPIO as GPIO
import math
import time

GPIO.setmode(GPIO.BCM)

dac = Adafruit_MCP4725.MCP4725()
button = Button(4)

try:
    while True:
        if button.is_pressed:
            shape = input("Enter 1 for a square wave, 2 for a triangle wave, and 3 for a sin wave: ")
            f = float(input("Enter the desired frequency (Hz, max of 20): "))
            v = int(input("Enter the desired maximum output voltage (mV): "))
            if shape == 1: #square
                while not button.is_pressed:
                    dac.set_voltage(v)
                    time.sleep(1/f/2)
                    dac.set_voltage(0)
                    time.sleep(1/f/2)
            elif shape == 2: #triangle
                while not button.is_pressed:
                    temp = 0
                    numSteps = 25
                    stepSize = v/numSteps #fixed amount of steps per voltage input
                    for i in range (numSteps):
                        dac.set_voltage(temp)
                        temp += stepSize
                        time.sleep(1/f/50)
                    temp = v
                    for j in range (numSteps):
                        dac.set_voltage(temp)
                        temp -= stepSize
                        time.sleep(1/f/50)
            elif shape == 3: #sin
                while not button.is_pressed:
                    temp = 0
                    numSteps = 50
                    stepSize = 2 * math.pi/numSteps
                    for i in range (numSteps):
                        dac.set_voltage(v/2+ int(v/2*math.sin(temp)))
                        temp += stepSize
                        time.sleep(1/f/50)
                    
            else:
                print ("Invalid input")
        
except KeyboardInterrupt:
    GPIO.cleanup()
            
