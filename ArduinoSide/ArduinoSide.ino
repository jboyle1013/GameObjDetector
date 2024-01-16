#include <SPI.h>

// Pin definitions
const int requestPin = 9; // GPIO pin used for sending the request signal
const int ssPin = 10;     // Slave Select pin for SPI

// SPI data buffer
volatile byte spiBuffer[128]; // Adjust size as needed
volatile int bufferIndex = 0;

// Flag to indicate data reception
volatile bool dataReceived = false;

void setup() {
    // Configure pin modes
    pinMode(requestPin, OUTPUT);
    pinMode(ssPin, INPUT_PULLUP);

    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8); // Adjust clock speed as needed
    SPI.attachInterrupt();  // Attach interrupt for SPI communication

    // Initialize serial communication for debugging
    Serial.begin(9600);
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
    byte data = SPDR; // Read the received data
    spiBuffer[bufferIndex++] = data; // Store data in buffer

    // Check for end of data transmission (e.g., newline character)
    if (data == '\n') {
        dataReceived = true;
        bufferIndex = 0; // Reset buffer index
    }
}

void loop() {
    // Send request signal periodically
    digitalWrite(requestPin, HIGH);
    delay(50); // Short delay to ensure signal is read
    digitalWrite(requestPin, LOW);

    // Wait for some time before next request
    delay(5000); // Adjust delay as needed

    // Check if data has been received
    if (dataReceived) {
        // Process received data
        processReceivedData();
        dataReceived = false; // Reset flag
    }
}

void processReceivedData() {
    // Convert the received bytes back into string
    String receivedString = "";
    for (int i = 0; i < sizeof(spiBuffer); i++) {
        if (spiBuffer[i] == '\0' || spiBuffer[i] == '\n') break;
        receivedString += (char)spiBuffer[i];
    }

    // Debug: Print the received string
    Serial.println(receivedString);

    // Reset the buffer
    memset(spiBuffer, 0, sizeof(spiBuffer));
    
    // TODO: Add your code here to split and parse the received string
    // Example: Split the string using the delimiter and interpret each part
}
