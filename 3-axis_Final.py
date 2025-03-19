import piplates.ADCplate as ADC
import time
import numpy as np  
import matplotlib.pyplot as plt
plt.ion() # turn on interactive mode

addr = 0  # ADC plate address
N = 1000  # Number of data points per channel

data_D0 = list()
data_D1 = list()
data_D2 = list()

print(ADC.getID(addr))
ADC.setMODE(addr, 'ADV')  

# Configure inputs for D0, D1, and D2
ADC.configINPUT(addr, 'D0', 13, True)
ADC.configINPUT(addr, 'D1', 13, True)
ADC.configINPUT(addr, 'D2', 13, True)

while True:
    ADC.startSTREAM(addr, N)  # Start ADC in streaming mode
    t0 = time.time()  # Start time tracking
    
    go = True
    while go:
        while ADC.check4EVENTS(addr) != True:  # Check for events
            pass
        if (ADC.getEVENTS(addr) & 0x80):  # Read event register when event detected
            go = False  # Exit loop when event detected
    
    data = ADC.getSTREAM(addr)  # Fetch data
    ADC.stopSTREAM(addr)  # Stop streaming
    
    T = time.time() - t0  # Total acquisition time
    Ntot = len(data)  # Total number of readings
    
    # Separate data into D0, D1, and D2
    for i in range(int(Ntot / 3)):
        data_D0.append(data[3 * i])
        data_D1.append(data[3 * i + 1])
        data_D2.append(data[3 * i + 2])
    
    t = np.linspace(0, T, num=int(Ntot / 3))  # Time axis
    freq = 1 / T * np.linspace(0, int(Ntot / 6), int(Ntot / 6))  # Frequency axis
    
    # Function to compute FFT
    def compute_fft(data):
        FFT = np.abs(np.fft.fft(data)) / len(data)
        V1 = FFT[:int(len(data) / 2)]
        V1[1:-1] = 2 * V1[1:-1]
        return V1
    
    # Compute FFTs
    FFT_D0 = compute_fft(data_D0)
    FFT_D1 = compute_fft(data_D1)
    FFT_D2 = compute_fft(data_D2)
    
    # Plot time-domain data
    plt.figure(1)
    plt.clf()
    plt.plot(t, data_D0, label='D0')
    plt.plot(t, data_D1, label='D1')
    plt.plot(t, data_D2, label='D2')
    plt.xlabel('Time [s]')
    plt.ylabel('Voltage [V]')
    plt.legend()
    
    # Plot frequency-domain data (FFT) in separate figures
    plt.figure(2)
    plt.clf()
    plt.plot(freq, FFT_D0)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Voltage [V]')
    plt.title('FFT of D0')
    
    plt.figure(3)
    plt.clf()
    plt.plot(freq, FFT_D1)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Voltage [V]')
    plt.title('FFT of D1')
    
    plt.figure(4)
    plt.clf()
    plt.plot(freq, FFT_D2)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Voltage [V]')
    plt.title('FFT of D2')
    
    plt.show()
    plt.pause(0.0001)
    
    # Reset data
    data_D0.clear()
    data_D1.clear()
    data_D2.clear()
