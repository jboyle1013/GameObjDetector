void setup() {
  Serial.begin(9600); // Start the serial communication
}

void loop() {
  Serial.println("Request"); // Send a request to the Jetson Nano

  // Wait for a response
  while (!Serial.available()) {
    // Optional: add a delay or a timeout if needed
  }

  // Read the response
  String response = Serial.readStringUntil('\n');
  Serial.print("Received from Jetson: ");
  Serial.println(response);

  delay(2000); // Delay between requests
}
