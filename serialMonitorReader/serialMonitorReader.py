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

    q = ""
    r = []
    s = []

    while(1):
        q = ser.readline()
        r = q.split(';')
        for j in range(len(r)):
            temp_y = r[i]
            x.append(i+5)
            y.append(temp_y)
            plt.scatter(i,temp_y, (j*.15, j*.15, j*.15))
            plt.show()
            plt.pause(0.0001) #Note this correction
            i+=1

main()