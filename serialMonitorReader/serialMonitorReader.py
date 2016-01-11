import serial
import matplotlib.pyplot as plt
import numpy as np



def main():

    ser = serial.Serial('/dev/cu.usbmodem1411', 9600)
    x=list()
    y=list()
    plt.ion() ## Note this correction
    fig=plt.figure()        
    plt.axis([0,1000,-2,2])    
    i=0

    q = ""
    r = []
    s = []

    color_list = plt.cm.Set3(np.linspace(0, 1, 12))


    for butts in range(20):

        ser.readline()

    while(1):
        q = ser.readline()
        r = q.split(';')
        #print(r)
        for j in range(len(r)):
            try:
                temp_y = r[j]
                x.append(i+5)
                y.append(temp_y)
                plt.scatter(i,temp_y, color = color_list[j])
                plt.show()
                plt.pause(0.0001) #Note this correction
                i+=1
            except ValueError:
                print "Oops!  That was no valid number.  Try again..."


main()