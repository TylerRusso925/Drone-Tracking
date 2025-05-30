import serial

def main():
    try:
        # Initialize the serial connection
        serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        print("Connected to /dev/ttyACM0 at 9600 baud rate.")
        
        # Write the command to the serial port
        command = "MANUAL_ON\n"
        serial_port.write(command.encode('utf-8'))
        print(f"Sent command: {command.strip()}")
        
        # Close the serial connection
        serial_port.close()
        print("Closed the serial connection.")
        
    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
