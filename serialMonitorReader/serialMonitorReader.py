import serial
import matplotlib.pyplot as plt
import numpy as np
from drawnow import *


def figure(y):
    plt.plot(y,'ro-'

def main():

    ser = serial.Serial('/dev/cu.usbmodem1411',115200)
    x=list()
    y=list()
    plt.ion() ## Note this correction
    # fig=plt.figure()        
    # plt.axis([0,1000,-2,2])    
    i=0
    while(1):
        while ser.inWaiting==0:
            pass
        q = ser.readline()

        temp_y = r[j]
        # x.append(i+5)
        y.append(temp_y)
        drawnow(figure)
        # plt.show()
        plt.pause(0.000001)
        i+=1
        if i>50:
            # x.pop(0)
            y.pop(0)


main()