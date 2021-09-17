import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import time
import matplotlib.pyplot as plt

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D22)
mcp = MCP.MCP3008(spi, cs)
chan0 = AnalogIn(mcp, MCP.P0)

def gatherData():
    voltageList = []
    timeList = []
    startTime = time.time()
    while time.time() - startTime < 1: #gather 1s of data points
        voltageList.append(chan0.voltage)
        timeList.append(time.time()-startTime)
        
    smoothedVoltageList = []
    smoothedVoltageList.append(voltageList[0])
    for i in range(1, len(voltageList)-1): #smooth data with rolling average spanning 3 numbers
        avg = (voltageList[i-1] + voltageList[i] + voltageList[i+1]) / 3
        smoothedVoltageList.append(avg)
    smoothedVoltageList.append(voltageList[len(voltageList)-1])
    
    #plot the 1s of data to prove the actual frequency output by the function generator (we used our generator from lab 2 on another Pi)
    plt.plot(timeList, voltageList, 'o', color="black")
    plt.show()

    return voltageList, smoothedVoltageList, timeList

def characterizeWaveform(rawVoltageList, voltageList, timeList):
    length = len(voltageList)
    max = 0.0
    min = 100000000.0
    for i in range(length): #obain max and min from smoothed voltage list
        if (voltageList[i] > max):
            max = voltageList[i]
        if (voltageList[i] < min):
            min = voltageList[i]
        if (i == length - 1):
            continue
        
    #test if it is a square wave
    numMin = 0
    numMax = 0
    tol = (max-min) * 0.1 #tolerance to account for noise (experimentally determined)
    for i in range(length): #determine amount of points that lie on extremes
        if max - voltageList[i] < tol:
            numMax = numMax + 1
        elif voltageList[i] - min < tol:
            numMin = numMin + 1
    if numMin + numMax > (length * 0.8): #if at 80% of points lie on extremes it is a square wave
        print("Square")
        squareFreq(rawVoltageList, timeList, min, max)
        return

    #test if its a triangle or sin wave   
    Edge = []
    start = -1
    end = -1
    side = -1 #0 is falling side, 1 is rising
    consOpp = 0 #account for fluctuations
    for i in range(1, length): #find one continuous edge (rising or falling) and store the start and end indices
        if (voltageList[i] - voltageList[i-1] >= 0):
            if (side == -1):
                side = 1
            elif (side == 0):
                consOpp = consOpp + 1
                if (consOpp >= 5): #account for noise causing fluctuations
                    if (start == -1):
                        start = i-1 - consOpp
                        side = 1
                    else:
                        end = i-1 - consOpp
                        break           
            else:
                consOpp = 0
        else:
            if (side == -1):
                side = 0
            elif (side == 1):
                consOpp = consOpp + 1
                if (consOpp >= 5): #account for noise causing fluctuations
                    if (start == -1):
                        start = i-1 - consOpp
                        side = 0
                    else:
                        end = i-1 - consOpp
                        break
            else:
                consOpp = 0
    for i in range(start+1, end): #use the indices to store the change in voltage between each point
        Edge.append(voltageList[i-1]-voltageList[i])
    sum = 0
    #obtain average change between points
    for i in range(len(Edge)):
        sum = sum + Edge[i]
    IncrAvg = sum / len(Edge)
    #count the number of changes in two consecutive points that equal the average for the edge
    nIncrAvg = 0
    tol = (max-min) * 0.0075 #tolerance to account for noise (experimentally determined)
    for i in range(len(Edge)):
        if abs(IncrAvg - Edge[i]) < tol:
            nIncrAvg = nIncrAvg + 1
    if nIncrAvg > len(Edge) * 0.5: #if 50% (experimentally determined) of the increases were equal to the average it is a triangle
        print("Triangle")
        sinTriangleFreq(voltageList, timeList, min, max)
    else:
        print("Sin")
        sinTriangleFreq(voltageList, timeList, min, max)

def squareFreq(voltageList, timeList, min, max):
    length = len(voltageList)
    tol = (max-min) * 0.2
    maxStart = -1
    maxFinish = -1
    minStart = -1
    minFinish = -1
    lastExtreme = -1 #-1 for no last extreme, 0 for min, 1 for max
    for i in range (length): #find a series of consecutive maxes and mins - the longer of which is half a cycle
        if max - voltageList[i] < tol:
            if lastExtreme == 0:
                minFinish = i-1
                if maxStart != -1:
                    break
            lastExtreme = 1
            if maxStart == -1:
                maxStart = i
        elif voltageList[i] - min < tol:
            if lastExtreme == 1:
                maxFinish = i-1
                if minStart != -1:
                    break
            lastExtreme = 0
            if minStart == -1:
                minStart = i
        
    maxTime = timeList[maxFinish] - timeList[maxStart]
    minTime = timeList[minFinish] - timeList[minStart]
    if maxTime > minTime:
        freq = 1.0 / 2.0 / maxTime
    else:
        freq = 1.0 / 2.0 / minTime
    print(str(freq) + "Hz")

def sinTriangleFreq (voltageList, timeList, min, max):
    length = len(voltageList)
    tol= (max-min) * 0.05
    min1 = -1
    min2 = -1
    maxSeen = False
    for i in range (length): #find 2 minimum points, a maximum must be inbetween - indicates one full cycle
        if voltageList[i] - min < tol:
            minSeen = True
            if min1 == -1:
                min1 = i
            elif maxSeen:
                min2 = i
                break
        elif max - voltageList[i] < tol:
            if min1 == -1:
                continue
            maxSeen = True
    freq = 1 / (timeList[min2] - timeList[min1])
    print(str(freq) + "Hz")
    
while True:
    rawVoltageList, smoothedVoltageList, timeList = gatherData()
    characterizeWaveform(rawVoltageList, smoothedVoltageList, timeList)

    

    