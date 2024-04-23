#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX pins

void setup() {
  // Open serial communications:
  Serial.begin(9600);
  // Set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
}

void loop() {
  // Send a request every 10 seconds
  delay(5000);
  mySerial.println("REQUEST");

  // Wait for a response with a timeout
  unsigned long startTime = millis();
  while (!mySerial.available() && millis() - startTime < 5000) {
    // Waiting for response with 5 seconds timeout
  }

  // Read and parse the response
  if (mySerial.available()) {
    String data = mySerial.readStringUntil('\n');
    processDetections(data);
  }
}

void parseDetectionData(String data) {
  // The data is expected to be in the format:
  // "class_name,confidence,depth_mm,depth_in,x,y,z,horizontal_angle,direction\n"

  int index = 0;
  while (index != -1) {
    // Find the next comma in the string
    int nextIndex = data.indexOf(',', index);
    if (nextIndex == -1) nextIndex = data.length(); // Last item in the string

    String item = data.substring(index, nextIndex);
    processDataItem(index / 9, index % 9, item); // Assumes 9 items per detection

    if (nextIndex == data.length()) break; // Exit loop if end of string is reached
    index = nextIndex + 1;
  }
}

void processDetections(String data) {
  // Split the data string into individual detections
  int startIdx = 0;
  int endIdx = data.indexOf(';');

  while (endIdx != -1) {
    String detection = data.substring(startIdx, endIdx);
    parseDetectionData(detection);
    startIdx = endIdx + 1;
    endIdx = data.indexOf(';', startIdx);
  }
  // Process the last detection (after the last semicolon)
  parseDetectionData(data.substring(startIdx));
}


void processDataItem(int detectionNumber, int itemNumber, String item) {
  switch (itemNumber) {
    case 0: Serial.print("Class Name: "); break;
    case 1: Serial.print("Confidence: "); break;
    case 2: Serial.print("Depth (mm): "); break;
    case 3: Serial.print("Depth (in): "); break;
    case 4: Serial.print("X: "); break;
    case 5: Serial.print("Y: "); break;
    case 6: Serial.print("Z: "); break;
    case 7: Serial.print("Horizontal Angle: "); break;
    case 8: Serial.print("Direction: "); break;
  }
  Serial.println(item);
}
