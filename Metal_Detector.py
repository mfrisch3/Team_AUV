
import piplates.ADCplate as ADC
import time

import numpy as np  
import matplotlib.pyplot as plt
plt.ion() # turn on interactive mode
 
addr=0    # This is the address of the ADC plate. Only need to change this if you are controlling multiple plates.
N=1000     # This is the number of data points that you collect before plotting it on the graph. Max value of 4096 points 
data=list()   #initialize the list
 
print(ADC.getID(addr))
ADC.setMODE(addr,'ADV')  


ADC.configINPUT(addr,'D0',13,True)  #Configure D0 input for 1007 sps. See documentation to adjust the samples per second.



while True:
    ADC.startSTREAM(addr,N)    #Start ADC in streaming mode. Flag an event when N data points are collected
    t0=time.time()             #start time tracking
       
    go=True
    while(go):
        while(ADC.check4EVENTS(addr)!=True):   #Check for events
        # Start of foreground tasks
            pass
        # End of foreground tasks
        if (ADC.getEVENTS(addr) & 0x80):       #read event register when event detected. Use 0x80 to ignore possible events from digital pins 
            go=False                           #if the event is == ADCcomplete then get out of while loop and fetch data
    data.extend(ADC.getSTREAM(addr))   
    ADC.stopSTREAM(addr)               #stop the STREAM

    T= time.time()-t0   # Total aquisition time
    Ntot = len(data)   # Total number of readings 
    t = np.linspace(0, T, num=Ntot)
    avg_val = sum(data) / Ntot
    avg = np.full(
        shape = Ntot,
        fill_value = avg_val,
        dtype=np.float64
        )
    
    if avg_val > 0.4:
        print("Metal detected!")
    
    plt.figure(1)
    plt.clf()
    plt.plot(t,data)
    plt.plot(t,avg)
    plt.xlabel('Time [s]')
    plt.ylabel('Voltage [V]')

          
    plt.show()
    plt.pause(0.0001) # Pause for a bit to have a enough time to show the plot.  
    
    data[0:(N)]= []     # Reset data
    
    



