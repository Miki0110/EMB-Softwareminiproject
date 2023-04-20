import serial
import time
import os

# Change the serial port and baud rate according to your setup
serial_port = 'COM3'  # Use '/dev/ttyACM0' or similar for Linux, '/dev/tty.usbserial' or similar for macOS
baud_rate = 31250

# Create a timestamped file to store the data
timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
file_name = f'arduino_data_{timestamp}.csv'

# Connect to the Arduino via the serial port
ser = serial.Serial(serial_port, baud_rate)

# Open the file to store the data
with open(file_name, 'w') as f:
    print(f'Starting data logging. Saving data to {file_name}')

    try:
        while True:
            # Read a line of data from the serial port
            line = ser.readline().decode('utf-8').strip()

            # Write the data to the file
            f.write(line + '\n')

            # Print the data to the console (optional)
            print(line)
    except KeyboardInterrupt:
        print('Data logging stopped.')

# Close the serial port
ser.close()