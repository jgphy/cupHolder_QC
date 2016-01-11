import serial
import matplotlib.pyplot as plt
import numpy as np



def main():

    ser = serial.Serial('/dev/cu.usbmodem1421', 9600)
    x=list()
    y=list()
    plt.ion() ## Note this correction
    fig=plt.figure()        
    plt.axis([0,1000,-2,2])    
    i=0
    while (1):

        temp_y = ser.readline()
        x.append(i+5)
        y.append(temp_y)
        plt.scatter(i,temp_y)
        plt.scatter(i,1,color='red')
        plt.show()
        plt.pause(0.0001) #Note this correction
        i+=1

main()