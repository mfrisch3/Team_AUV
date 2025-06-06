import piplates.ADCplate as ADC 
import time
import numpy as np  
import matplotlib.pyplot as plt
from gpiozero import PWMOutputDevice
import plotext
from time import sleep

plt.ion()  # Enable interactive mode for real-time plotting
# Get GPIO pin number for the motors
# NEED TO FIGURE OUT WHICH PIN IS MAPPED TO WHICH MOTOR
M1_PIN = 21 # vertical 
M2_PIN = 16 # back right
M3_PIN = 12 #back left
M4_PIN = 13 # tbd
M5_PIN = 6 # vertical
M6_PIN = 5 # front left

# initialize motors for PWM output
#motor = PWMOutputDevice(PIN, frequency=50)
motor_1 = PWMOutputDevice(M1_PIN, frequency=150)
motor_2 = PWMOutputDevice(M2_PIN, frequency=150)
motor_3 = PWMOutputDevice(M3_PIN, frequency=150)
motor_4 = PWMOutputDevice(M4_PIN, frequency=150)
motor_5 = PWMOutputDevice(M5_PIN, frequency=150)
motor_6 = PWMOutputDevice(M6_PIN, frequency=150)
# Function to set motor speed
addr = 0  # ADC plate address
N = 2000  # Number of data points per channel

# maximum duty cycle (maximum speed)
MAX_DUTY = 17

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
    mask = (freq >= 10) & (freq <= 150)
    
    max_index = np.argmax(FFT_D0[mask])
    max_freq = (freq[mask])[max_index]
    print("Max Frequency: ", max_freq)
    # --------------------------------
    # Freq: 89-Left 96-Right 107-Back 116-Forward
    # --------------------------------
    duty_s = 1
    motor_1.value = duty_s/100
    motor_2.value = duty_s/100
    motor_3.value = duty_s/100
    motor_4.value = duty_s/100
    motor_5.value = duty_s/100
    motor_6.value = duty_s/100

    # Forward
    if(max_freq <= 115 and max_freq >= 108):
        duty_s = 10
        spin_motor(motor1=motor_1, motor2=motor_2)
        print("Forward")

    # Back
    elif(max_freq >= 100 and max_freq <= 107):
        duty_s = 10
        spin_motor(motor2=motor_2, motor3=motor_3)
        print("Back")

    # Left
    elif(max_freq <= 93 and max_freq >= 85):
        duty_s = 10
        spin_motor(motor2=motor_2, motor4=motor_4)
        print("Left")

    # Right
    elif(max_freq <= 99 and max_freq >= 94):
        duty_s = 10
        spin_motor(motor3=motor_3,motor6=motor_6)
        print("Right")

def spin_motor(motor1=None, motor2=None, motor3=None, motor4=None, motor5=None, motor6=None):
    for duty in range(3,MAX_DUTY):
        motor1.value = duty/100
        motor2.value = duty/100
        motor3.value = duty/100
        motor4.value = duty/100
        motor5.value = duty/100
        motor6.value = duty/100
        time.sleep(0.5)
