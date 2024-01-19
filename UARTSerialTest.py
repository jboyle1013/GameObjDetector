import serial

# Open the serial port at 9600 baud
ser = serial.Serial(
    port='/dev/ttyTHS1',  # Replace with your serial port
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
try:
    while True:
        if ser.inWaiting() > 0:
            ser.flushInput()
            incoming = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino says: {incoming}")
            # Respond to the request
            if incoming == "Request":
                str_to_send = "Data From Jetson\n"
                print(f"Sending String: {str_to_send}")
                ser.write(str_to_send.encode())

except KeyboardInterrupt:
    ser.close()
