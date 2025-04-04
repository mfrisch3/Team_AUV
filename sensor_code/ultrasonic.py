import serial
import time
import RPi.GPIO as GPIO

# Constants
COM = 0x55
LED_PIN = 6  # GPIO6 (physical pin 31)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

# Setup Serial
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2)  # Let serial settle

def calculate_distance(buffer):
    CS = buffer[0] + buffer[1] + buffer[2]
    if buffer[3] == CS:
        distance = (buffer[1] << 8) + buffer[2]
        return distance
    return None

try:
    while True:
        ser.write(bytes([COM]))
        time.sleep(0.1)

        if ser.in_waiting > 0:
            time.sleep(0.004)

            if ser.read() == b'\xff':
                buffer = [0xff]
                for _ in range(3):
                    byte = ser.read()
                    if byte:
                        buffer.append(ord(byte))

                if len(buffer) == 4:
                    distance = calculate_distance(buffer)
                    if distance is not None:
                        print(f"Distance: {distance}mm")

                        if distance < 100:
                            GPIO.output(LED_PIN, GPIO.HIGH)
                        else:
                            GPIO.output(LED_PIN, GPIO.LOW)

except KeyboardInterrupt:
    print("Stopping script...")

finally:
    GPIO.cleanup()
    ser.close()
