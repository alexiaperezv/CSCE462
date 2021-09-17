from gpiozero import LED, Button
from time import sleep
import time
import threading
import RPi.GPIO as GPIO
from signal import pause
import sys


GPIO.setmode(GPIO.BCM)

# Traffic Light 2 using GPIOzero library
red2 = LED(4)
green2 = LED(5)
blue2 = LED(6)

#Traffic Light 1 using GPIOzero library
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
# GPIOzero library
button = Button(17) 

# RPi.GPIO way of doing it. GPIO 17 set as input. Pulled up to stop false signals
# GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Makes a light blink blue. Note: This functionality can be reached with the blink() function
# from the GPIOzero library
def blink(countdown):
    while countdown > 0:
        blue.on()
        sleep(0.3)
        blue.off()
        sleep(0.3)
        countdown -=1
# END OF BLINK

# Function for when button is pressed
def pressed():
    try:
        startTime = time.time()
        
        # GPIO.wait_for_edge(17, GPIO.FALLING) # RPi.GPIO way of doing it
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
            sleep(0.7)
                        
            if countdown == 5: # If countdown is less than 5
                green.off()
                t1 = threading.Thread(target=blink, args=(countdown,)) # Create new thread to blink first light blue
                t1.start()
                    
            countdown -= 1 # Decrement the countdown shown
                        
        red2.off()
        red.on() # After timer Traffic Light 1 becomes red and
        green2.on() # Traffic Light 2 becomes green
        sleep(1)
        red.off()
        green2.off()
        
        while time.time() - startTime < 19.5:
            continue
        
        green2.on()        
        
    except KeyboardInterrupt:
        GPIO.cleanup() # clean up GPIO on Ctrl+C exit
    # GPIO.cleanup()
# END OF PRESSED

green2.on() # start program with Traffic Light 2 green    
button.when_pressed = pressed # Run pressed() when the button is pressed

    
# pause() # pauses the script so it can wait for the button press
while True:
    time.sleep(1)
