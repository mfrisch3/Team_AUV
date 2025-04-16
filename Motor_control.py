import piplates.ADCplate as ADC 
import time
import numpy as np  
import matplotlib.pyplot as plt
from gpiozero import PWMOutputDevice
import plotext


plt.ion()  # Enable interactive mode for real-time plotting
PIN= 26
motor = PWMOutputDevice(PIN, frequency=50)
addr = 0  # ADC plate address
N = 2000  # Number of data points per channel

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
    # Frequency axis for FFT; note that the FFT result length is int(Ntot/6) for one channel
    freq = 1 / T * np.linspace(0, int(Ntot / 6), int(Ntot / 6))
    
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
    
    
    # -------------------------------
    # NEW: Compute and print the magnitude in the 110-125Hz range
    # Create a boolean mask for frequencies between 110 and 125 Hz
    mask = (freq >= 120) & (freq <= 150)
    # Compute the average magnitude in this frequency band for channel D0.
    magnitude_voltage = np.max(FFT_D0[mask])
    print("Magnitude of voltage between 120Hz and 150Hz: {:.4f}".format(magnitude_voltage))
    
    



    if(magnitude_voltage >= 0.1):
        duty_s = 10
        motor.value = duty_s / 100  # Convert to range 0-1
        #motor_speed = set_motor_speed(motor, duty_s)
    #elif magnitude_voltage >= 0.12:
       # duty_s = 6
       # motor.value = duty_s / 100
    #elif magnitude_voltage >= 0.15:
     #   duty_s = 7
     #   motor.value = duty_s / 100
    #elif magnitude_voltage >= 0.2:
    #     motor.value = 8
    #elif magnitude_voltage >= 0.25:
      #  duty_s = 9
     #   motor.value = duty_s / 100
    #elif magnitude_voltage >= 0.3: 
      #  duty_s = 10
      #  motor.value = duty_s / 100
    elif magnitude_voltage < 0.1:
        duty_s = 4
        motor.value = duty_s / 100  # Convert to range 0-1
    # Clear the data buffers for the next iteration
    data_D0 = list()
    data_D1 = list()
    data_D2 = list()


     