#include <SPI.h>

volatile byte receivedData;
volatile bool dataReceived = false;

void setup() {
  // Set MISO output, all others input
  pinMode(MISO, OUTPUT);

  // Turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // Register SPI service routine
  SPI.attachInterrupt();
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
  receivedData = SPDR; // Get the received data
  dataReceived = true;
}

void loop() {
  if (dataReceived) {
    SPDR = receivedData + 1; // Increment received data and send it back
    dataReceived = false;
  }
}
