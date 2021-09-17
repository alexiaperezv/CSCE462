from gpiozero import LED, Button
from time import sleep
import time
import threading
import RPi.GPIO as GPIO
import sys

GPIO.setmode(GPIO.BCM)

# Global Variables
elapsed = sys.maxint
startTime = 0

# Traffic Light 2
red2 = LED(4)
green2 = LED(5)
blue2 = LED(6)

#Traffic Light 1
red = LED(20)
green = LED(13)
blue = LED(19)

# Seven Segment Display
pins = [18, 23, 26, 27, 22, 12, 25]

GPIO.setup(18, GPIO.OUT) # Intialize GPIO Pins as output
GPIO.setup(23, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(21, GPIO.OUT) # dot

arrSequence = [[1,1,1,1,1,1,0], #  0
                [0,1,1,0,0,0,0], # 1
                [1,1,0,1,1,0,1], # 2
                [1,1,1,1,0,0,1], # 3
                [0,1,1,0,0,1,1], # 4
                [1,0,1,1,0,1,1], # 5
                [1,0,1,1,1,1,1], # 6
                [1,1,1,0,0,0,0], # 7
                [1,1,1,1,1,1,1], # 8
                [1,1,1,0,0,1,1] # 9
                ]

# Button
button = Button(17)

def blink():
    while countdown > 0:
        blue.on()
        sleep(0.3)
        blue.off()
        sleep(0.3)

try:
    while True:
        green2.on() # Traffic Light 2 starts green
        if elapsed < 20: # Make sure to wait 20 seconds before button can be pressed
            elapsed = time.time() - startTime
            continue

        if button.is_pressed and elapsed >= 20:
            startTime = time.time() # Get starting time of pressing button

            green2.off()
            sleep(1)
            for i in range(3): # blink blue three times
                blue2.on()
                sleep(1)
                blue2.off()
                sleep(0.5)

            red2.on() # Turn red on and wait for countdown to reach zero
            green.on() # Traffic Light 1 turns green

            countdown = 9
            while countdown >= 0:
                for i in range(7): # get number sequence
                    GPIO.output(pins[i], arrSequence[countdown][i]) # activate pin[i] on high or low (0 - 1) depending on arrSequence
                sleep(1)

                if countdown == 5: # If countdown is less than 5
                    green.off()
                    #if countdown == 4:
                    t1 = threading.Thread(target=blink)
                    t1.start()

                countdown -= 1 # Decrement the countdown shown

            red2.off()
            red.on() # After timer Traffic Light 1 becomes red and
            green2.on() # Traffic Light 2 becomes green
            sleep(1)
            red.off()
            green2.off()

            elapsed = time.time() - startTime


except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on Ctrl+C exit

GPIO.cleanup() # clean up at end of program
