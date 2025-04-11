import serial
import time
from gpiozero import LED

# Constants
COM = 0x55
LED_PIN = 6  # GPIO6 (BCM numbering)

# Setup LED
led = LED(LED_PIN)

# Setup Serial communication
ser = serial.Serial('/dev/tty0', 115200, timeout=1)
time.sleep(2)  # Allow time for serial to settle

def calculate_distance(buffer):
    """
    Calculate distance from the sensor data.
    """
    CS = buffer[0] + buffer[1] + buffer[2]
    if buffer[3] == CS:
        # Combine the 2nd and 3rd byte for the distance
        distance = (buffer[1] << 8) + buffer[2]
        return distance
    return None

try:
    while True:
        # Send the COM command
        ser.write(bytes([COM]))
        time.sleep(0.1)

        if ser.in_waiting > 0:
            time.sleep(0.004)  # Delay for data to be ready

            if ser.read() == b'\xff':
                buffer = [0xff]
                for _ in range(3):
                    byte = ser.read()
                    if byte:
                        buffer.append(ord(byte))

                if len(buffer) == 4:
                    # Calculate distance
                    distance = calculate_distance(buffer)
                    if distance is not None:
                        print(f"Distance: {distance}mm")

                        # Turn on/off the LED based on the distance
                        if distance < 100:
                            led.on()
                        else:
                            led.off()

except KeyboardInterrupt:
    print("Stopping script...")

finally:
    # Ensure the LED is turned off before exiting
    led.off()
    ser.close()
