#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX pins

unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;       // Interval at which to make requests (10 seconds)

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000); // Wait for everything to stabilize
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

void processDetections(String data) {
  // Split the data string into individual detections
  int startIdx = 0;
  int endIdx = data.indexOf(';');

  while (endIdx != -1) {
    String detection = data.substring(startIdx, endIdx);
    parseDetection(detection);
    startIdx = endIdx + 1;
    endIdx = data.indexOf(';', startIdx);
  }
  // Process the last detection (after the last semicolon)
  parseDetection(data.substring(startIdx));
}

void parseDetection(String detection) {
  // Split the detection string into its components
  int numFields = 9; // Number of fields in the Detection class
  String fields[numFields];
  int startIdx = 0;
  int endIdx;

  for (int i = 0; i < numFields; i++) {
    endIdx = detection.indexOf(',', startIdx);
    fields[i] = (endIdx == -1) ? detection.substring(startIdx) : detection.substring(startIdx, endIdx);
    startIdx = endIdx + 1;
  }

  // Process the detection fields
  Serial.println("Detection received:");
  Serial.print("Class Name: ");
  Serial.println(fields[0]);
  Serial.print("Confidence: ");
  Serial.println(fields[1]);
  Serial.print("Depth MM: ");
  Serial.println(fields[2]);
  Serial.print("Depth IN: ");
  Serial.println(fields[3]);
  Serial.print("X Component: ");
  Serial.println(fields[4]);
  Serial.print("Y Component: ");
  Serial.println(fields[5]);
  Serial.print("Z Component: ");
  Serial.println(fields[6]);
  Serial.print("Horizontal Angle: ");
  Serial.println(fields[7]);
  Serial.print("Direction: ");
  Serial.println(fields[8]);
}