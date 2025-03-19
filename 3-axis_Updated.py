import piplates.ADCplate as ADC
import time
import numpy as np  
import matplotlib.pyplot as plt
plt.ion() # turn on interactive mode
 
addr=0
N=3115
data=list()   #initialize the list
D0 = list()
D1 = list()
D2 = list()
 
print(ADC.getID(addr))
ADC.setMODE(addr,'ADV')  

ADC.configINPUT(addr,'D0',15,True)  #Configure S0 input for 1007 sps
ADC.configINPUT(addr,'D1',15,True)  #Configure S1 input for 1007 sps
ADC.configINPUT(addr,'D2',15,True)  #Configure S2 input for 1007 sps

while True: 
    ADC.startSTREAM(addr,N)             #start the STREAM
    t0=time.time()
    for i in range(1):              
        go=True
        while(go):
            while(ADC.check4EVENTS(addr)!=True):   #Check for events
            # Start of foreground tasks
                pass
            # End of foreground tasks
            if (ADC.getEVENTS(addr) & 0x80):       #read event register when event detected
                go=False                           #if the event is == ADCcomplete then get out of while loop and fetch data
        data.extend(ADC.getSTREAM(addr))   
    ADC.stopSTREAM(addr)               #stop the STREAM

    T= time.time()-t0   # Total aquisition time
    Ntot = len(data)   # Total number of readings 

    t = np.linspace(0, T, num=int(Ntot/3))

    # Extract separate readings:
    for i in range(int(Ntot/3)):
        D0.extend([data[3*i]])
        
    for i in range(int(Ntot/3)):
        D1.extend([data[3*i+1]])
        
    for i in range(int(Ntot/3)):
        D2.extend([data[3*i+2]])
     
     # Compute FFT: 
    FFT = np.abs(np.fft.fft(D0)/(Ntot/3))   # Compute abs value of FFT. Double sided
    V1 = FFT[0:int(Ntot/6)]   # Convert to single sided
    V1[1:-2]=2*V1[1:-2] # Multiply by 2 except for DC component 
    freq = 1/T*np.linspace(0, Ntot/6, num=int(Ntot/6))
                 
    plt.figure(1)
    plt.clf()
    plt.plot(freq,V1)
    plt.ylim(0, 2)  # Limit Y-axis from 0 to 2
    
    FFT = np.abs(np.fft.fft(D1)/(Ntot/3))   # Compute abs value of FFT. Double sided
    V1 = FFT[0:int(Ntot/6)]   # Convert to single sided
    V1[1:-2]=2*V1[1:-2] # Multiply by 2 except for DC component

    plt.figure(2)
    plt.clf()
    plt.plot(freq,V1)
    plt.ylim(0, 2)  # Limit Y-axis from 0 to 2
    
    FFT = np.abs(np.fft.fft(D2)/(Ntot/3))   # Compute abs value of FFT. Double sided
    V1 = FFT[0:int(Ntot/6)]   # Convert to single sided
    V1[1:-2]=2*V1[1:-2] # Multiply by 2 except for DC component

    plt.figure(3)
    plt.clf()
    plt.plot(freq,V1)
    plt.ylim(0, 2)  # Limit Y-axis from 0 to 2
    
    plt.show()
    plt.pause(0.00002) # Pause for a bit to have a enough time to show the plot.   
        
    data[0:(N)]= []     # Reset data
