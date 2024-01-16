import spidev
import time

# Set up SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Using SPI bus 0 and device (CS) 0
spi.mode = 0b00
spi.max_speed_hz = 1000000

try:
    while True:
        # Send a byte
        sentData = [0x01]
        print("Sending:", sentData[0])

        # SPI transaction
        receivedData = spi.xfer(sentData)

        # Print received data
        print("Received:", receivedData[0])

        time.sleep(1)  # Wait for a second
except KeyboardInterrupt:
    spi.close()
