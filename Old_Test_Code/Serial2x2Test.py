import serial
import time

# Open UART port
uart_port = serial.Serial('/dev/ttyTHS1', baudrate=9600, timeout=1)

# Open USB serial port
usb_serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

while True:
    if uart_port.in_waiting > 0:
        # Read command from UART
        command = uart_port.readline().decode('utf-8').rstrip()

        # Process the command (e.g., just echo it back for this example)
        response = f"Jetson received: {command}"

        # Send response back over USB serial
        usb_serial_port.write(response.encode('utf-8'))

        print(f"Sent to Arduino: {response}")

    time.sleep(1)
