import piplates.ADCplate as ADC
import time
import numpy as np  
import matplotlib.pyplot as plt
plt.ion()  # turn on interactive mode
 
addr = 0    # This is the address of the ADC plate.
N = 1000    # Number of data points to collect before plotting. Max value is 4096
data = list()  # initialize the list
 
print(ADC.getID(addr))
ADC.setMODE(addr, 'ADV')  
ADC.configINPUT(addr, 'D0', 13, True)  # Configure D0 for 1007 sps

while True:
    ADC.startSTREAM(addr, N)    # Start streaming
    t0 = time.time()            # start time tracking
    
    go = True
    while go:
        while not ADC.check4EVENTS(addr):
            pass
        if ADC.getEVENTS(addr) & 0x80:  # check for ADC complete
            go = False

    data.extend(ADC.getSTREAM(addr))   
    ADC.stopSTREAM(addr)  # stop streaming

    T = time.time() - t0   # total acquisition time
    Ntot = len(data)       # total number of readings 
    
    FFT = np.abs(np.fft.fft(data)) / Ntot  # Compute FFT and normalize
    V1 = FFT[0:int(Ntot/2+1)]               # Single-sided FFT
    V1[1:-2] = 2 * V1[1:-2]                 # Double amplitude (except DC and Nyquist)

    t = np.linspace(0, T, num=Ntot)  # Time axis
    freq = 1/T * np.linspace(0, int(Ntot/2+1), int(Ntot/2+1))  # Frequency axis

    # Plot time-domain signal
    plt.figure(1)
    plt.clf()
    plt.plot(t, data)
    plt.xlabel('Time [s]')
    plt.ylabel('Voltage [V]')

    # Plot frequency-domain (zoomed into 100?130 Hz)
    plt.figure(2)
    plt.clf()
    plt.plot(freq, V1)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Voltage [V]')
    plt.xlim((100, 130))  # Focus on 100?130 Hz

    plt.show()
    plt.pause(0.0001)  # Short pause to update plot
    
    data[0:N] = []  # Clear data for next loop
