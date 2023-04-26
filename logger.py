import serial
import time


def write_into_file(file_name, data):
    # Open the file to store the data
    with open(file_name, 'w') as f:
        # Write the data to the file
        f.write(data + '\n')


def send_pid_values(ser, pid_values):
    # Send PID values to Arduino
    ser.write(f"{pid_values[0]},{pid_values[1]},{pid_values[2]}\n".encode('utf-8'))
    time.sleep(0.1)


# Change the serial port and baud rate according to your setup
serial_port = 'COM5'  # Use '/dev/ttyACM0' or similar for Linux, '/dev/tty.usbserial' or similar for macOS
baud_rate = 31250

# List of PID values to test
pid_values_list = [
    (1.0, 1.0, 1.0),
    (5.0, 2.0, 2.0),
    (25.0, 3.0, 3.0),
    (50.0, 4.0, 4.0),
    (100.0, 5.0, 5.0),
    (500.0, 6.0, 6.0),
    (1000.0, 7.0, 7.0),
    (1500.0, 8.0, 8.0),
    (2000.0, 9.0, 9.0),
    (3000.0, 10.0, 10.0),
]

# Connect to the Arduino via the serial port
ser = serial.Serial(serial_port, baud_rate)

try:
    for pid_values in pid_values_list:
        send_pid_values(ser, pid_values)

        # Read a line of data from the serial port
        line = ser.readline().decode('utf-8').strip()

        # Create a timestamped file to store the data
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        file_name = f'arduino_data_{timestamp}_P{pid_values[0]}_I{pid_values[1]}_D{pid_values[2]}.csv'

        start_time = time.time()
        while True:
            # Read a line of data from the serial port
            line = ser.readline().decode('utf-8').strip()

            # Save the response in the file
            write_into_file(file_name, line)

            # Print the data to the console (optional)
            print(line)

            elapsed_time = time.time() - start_time
            if elapsed_time >= 10:
                # Send the new PID values after 10 seconds
                break

except KeyboardInterrupt:
    print('Data logging stopped.')

finally:
    # Close the serial port
    ser.close()