#include <DetectionsBuffer.h>

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX pins

unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;       // Interval at which to make requests (10 seconds)

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  clearBuffer();
  delay(1000); // Wait for everything to stabilize
}

void loop() {
  // Send a request every 10 seconds
  delay(10000);
  mySerial.println("REQUEST");

  // Wait for a response with a timeout
  unsigned long startTime = millis();
  while (!mySerial.available() && millis() - startTime < 5000) {
    // Waiting for response with 5 seconds timeout
  }

  // Read and parse the response
  if (mySerial.available()) {
    Serial.println("Detections received:");
    String data = mySerial.readStringUntil('\n');
    // Serial.println(data);
    processDetections(data);
  }

  // delay(2000); // Wait for everything to stabilize
  // printDetections();
}

void processDetections(String data) {
  // Split the data string into individual detections
  int startIdx = 0;
  int endIdx = data.indexOf(';');

  while (endIdx != -1) {
    String detection = data.substring(startIdx, endIdx);
    // Serial.print("Detection ");
    // Serial.println(startIdx);
    Serial.println(detection);
    parseDetection(detection);
    startIdx = endIdx + 1;
    endIdx = data.indexOf(';', startIdx);
  }
  // Process the last detection (after the last semicolon)
  // Serial.print("Detection ");
  // Serial.println(startIdx);
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
    // Serial.println(fields[i]);
    startIdx = endIdx + 1;

  }

  Serial.println("Test Print Begin");
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
  Serial.println("Test Print End");

  // Detection newDetection(class_name, confidence, depth_mm, depth_in, x, y, z, horizontal_angle, direction);
  // Serial.println("Newest Detection");
  // printDetection(newDetection);
  // addDetectionToBuffer(newDetection);

}

long stringToLong(String s)
{
    char arr[12];
    s.toCharArray(arr, sizeof(arr));
    return atol(arr);
}

long stringToFloat(String s)
{
    char arr[12];
    s.toCharArray(arr, sizeof(arr));
    return atof(arr);
}
long stringToChar(String s)
{
    char arr[12];
    s.toCharArray(arr, sizeof(arr));
    return arr;
}

void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // Print the latest detection - Not Right Now - Working on it
    // Detection latest = getLatestDetection();
    // Serial.println("Latest Detection:");
    // printDetection(latest);

    // Loop through and print all detections
    // Serial.println("All Detections:");
    // for (int i = 0; i < getBufferSize(); i++) {
    //     Detection d = getDetectionFromBuffer(i);
    //     printDetection(d);
    // }
}

void printDetection(const Detection& d) {
    if (strlen(d.class_name) > 0) { // Check if the detection is valid
        Serial.print("Class Name: ");
        Serial.println(d.class_name);
        Serial.print("Confidence: ");
        Serial.println(d.confidence, 2);
        Serial.print("Depth MM: ");
        Serial.println(d.depth_mm);
        Serial.print("Depth IN: ");
        Serial.println(d.depth_in);
        Serial.print("X Component: ");
        Serial.println(d.x);
        Serial.print("Y Component: ");
        Serial.println(d.y);
        Serial.print("Z Component: ");
        Serial.println(d.z);
        Serial.print("Horizontal Angle: ");
        Serial.println(d.horizontal_angle);
        Serial.print("Direction: ");
        Serial.println(d.direction);
        Serial.println("-------------------");
    } else {
        Serial.println("No Detection Data");
    }
}