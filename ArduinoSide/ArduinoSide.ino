#include <SPI.h>

// GPIO pin for sending signal
const int signalPin = 9;

// SPI data buffer
volatile byte spiBuffer[512];
volatile int bufferIndex = 0;

// Flag to indicate data reception
volatile bool dataReceived = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Configure pin modes
  pinMode(signalPin, OUTPUT);

  // Set MISO output, all others input
  pinMode(MISO, OUTPUT);

  // Turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // Register SPI service routine
  SPI.attachInterrupt();
  Serial.print("Code Running");
}

// SPI interrupt routine
ISR(SPI_STC_vect) {
  byte receivedByte = SPDR;                 // Get the received data
  spiBuffer[bufferIndex++] = receivedByte;  // Store data in buffer

  // Send a response back to the master
  byte responseByte = receivedByte + 1;     // Simple response (increment received byte)
  SPDR = responseByte;                      // Load the response byte into the SPI Data Register

  // Check for end of data transmission or buffer overflow
  if (receivedByte == '\n' || bufferIndex >= sizeof(spiBuffer)) {
    dataReceived = true;
    bufferIndex = 0;  // Reset buffer index for next message
  }
}

void loop() {
  // Send signal periodically
  digitalWrite(signalPin, HIGH);
  delay(50);  // Signal duration
  digitalWrite(signalPin, LOW);

  // Wait before sending the signal again
  delay(5000);  // Adjust as needed

  // Check if data has been received
  if (dataReceived) {
    // Process received data
    processReceivedData();
    dataReceived = false;  // Reset flag for next reception
  }
}

void processReceivedData() {
  // Print the received data to the serial monitor
  Serial.print("Received SPI Data: ");
  for (int i = 0; i < sizeof(spiBuffer); ++i) {
    if (spiBuffer[i] == '\0' || spiBuffer[i] == '\n') break;  // End of data
    Serial.print((char)spiBuffer[i]);
  }
  Serial.println();

  // Reset the buffer for the next message
  memset(spiBuffer, 0, sizeof(spiBuffer));
}
