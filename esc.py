from gpiozero import PWMOutputDevice
#gpiozero is a python library that allows you to control the GPIO pins on a Raspberry Pi.
# It provides an easy-to use interface

import time

PIN = 26
motor1 = PWMOutputDevice(PIN, frequency=50)

PIN2 = 19
motor2 = PWMOutputDevice(PIN2, frequency=50)

PIN3 = 13
motor3 = PWMOutputDevice(PIN3, frequency=50)

PIN4= 21
motor4 = PWMOutputDevice(PIN4, frequency=50)

PIN5 = 6

motor5 = PWMOutputDevice(PIN5, frequency=50)

def set_motor_speed(motor, duty_cycle):
    motor.value = duty_cycle / 100  # Convert to range 0-1

    return

while True:
    try:
        duty_s = float(input("Enter duty cycle for motor 1: "))  # Convert input to float
        motor1_speed = set_motor_speed(motor1, duty_s)
        duty_s2 = float(input("Enter duty cycle for motor 2: "))  # Convert input to float
        motor2_speed = set_motor_speed(motor2, duty_s2)

        duty_s3 = float(input("Enter duty cycle for motor 3: "))  # Convert input to float
        motor3_speed = set_motor_speed(motor3, duty_s3)
        
        duty_s4 = float(input("Enter duty cycle for motor 4: "))  # Convert input to float
        motor4_speed = set_motor_speed(motor4, duty_s4)
        #duty_s2 = float(input("Enter duty cycle for motor 2:"))
        #motor2_speed = set_motor_speed(motor2, duty_s2)
        duty_s5 = float(input("Enter duty cycle for motor 5: "))  # Convert input to float
        motor5_speed = set_motor_speed(motor5, duty_s5)
    except ValueError:
        print("Invalid input! Please enter a numerical value.")
