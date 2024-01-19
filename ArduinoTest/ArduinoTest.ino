#include <SPI.h>

volatile byte receivedByte;
volatile bool dataReceived = false;

void setup() {
  Serial.begin(9600);
  pinMode(MISO, OUTPUT);  // MISO pin set as output

  // Turn on SPI in slave mode
  SPCR |= _BV(SPE);   // SPI Enable
  SPCR |= _BV(SPIE);  // SPI Interrupt Enable

  Serial.println("SPI Slave Initialized");
}

ISR(SPI_STC_vect) {         // SPI interrupt routine
  receivedByte = SPDR;      // Read received byte
  dataReceived = true;      // Set flag indicating data was received
  SPDR = receivedByte + 1;  // Increment byte and write back to SPI Data Register
}

void loop() {
  if (dataReceived) {
    Serial.print("Received: ");
    Serial.println(receivedByte);
    dataReceived = false;  // Reset the flag
  }
}