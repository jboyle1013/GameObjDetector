void setup() {
  Serial.begin(9600, SERIAL_8N1);
}

void loop() {
  Serial.println("Request"); // Send a request to the Jetson Nano

  unsigned long startTime = millis(); // Record the start time
  bool timeoutOccurred = false;

  // Wait for a response or until timeout
  while (!Serial.available()) {
    if (millis() - startTime > 2000) { // 2 seconds timeout
      timeoutOccurred = true;
      break; // Break the loop if timeout occurs
    }
  }

  if (!timeoutOccurred) {
    // Read the response
    String response = Serial.readStringUntil('\n');
    Serial.print("Received from Jetson: ");
    Serial.println(response);
  } else {
    Serial.println("No response received, sending another request.");
  }

  delay(3000); // Delay between requests
}
