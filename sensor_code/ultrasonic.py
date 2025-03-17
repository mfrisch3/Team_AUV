import serial
import time

# Define the COM command
COM = 0x55

# Initialize the serial connection
# Use the appropriate serial port (e.g., '/dev/ttyUSB0' or '/dev/ttyAMA0' or '/dev/serial0')
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Adjust the port as needed
time.sleep(2)  # Wait for the serial connection to initialize

def calculate_distance(buffer):
    # Calculate the checksum
    CS = buffer[0] + buffer[1] + buffer[2]
    
    # Verify the checksum
    if buffer[3] == CS:
        # Calculate the distance
        distance = (buffer[1] << 8) + buffer[2]
        return distance
    return None

def main():
    while True:
        # Send the COM command
        ser.write(bytes([COM]))
        time.sleep(0.1)  # Delay for 100ms

        # Check if data is available
        if ser.in_waiting > 0:
            time.sleep(0.004)  # Delay for 4ms

            # Read the first byte
            if ser.read() == b'\xff':
                buffer = [0xff]
                for _ in range(3):
                    buffer.append(ord(ser.read()))
                
                # Calculate and print the distance
                distance = calculate_distance(buffer)
                if distance is not None:
                    print(f"Distance: {distance}mm")

if __name__ == "__main__":
    main()