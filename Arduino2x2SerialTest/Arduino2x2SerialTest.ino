#include <SoftwareSerial.h>

SoftwareSerial uartSerial(10, 11); // RX, TX

void setup() {
  // Start the built-in serial port, for USB communication
  Serial.begin(9600);
  // Start the software serial port, for UART communication
  uartSerial.begin(9600);
}

void loop() {
  // Send a command to the Jetson
  uartSerial.println("Hello Jetson");

  // Check if the Jetson has sent a response
  if (Serial.available()) {
    String response = Serial.readStringUntil('\n');
    Serial.print("Received from Jetson: ");
    Serial.println(response);
  }

  // Wait for a bit before sending the next command
  delay(2000);
}
