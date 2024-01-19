import serial

# Open the serial port at 9600 baud
ser = serial.Serial('/dev/ttyTHS1', 9600)  # Adjust the port if necessary

try:
    while True:
        if ser.inWaiting() > 0:
            ser.flushInput()
            incoming = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino says: {incoming}")
            # Respond to the request
            if incoming == "Request":
                ser.write("Data from Jetson\n".encode())

except KeyboardInterrupt:
    ser.close()
