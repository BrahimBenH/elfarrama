#include <Arduino.h>

void readNumber() {
  // --- Tell Python to start ---
  Serial.println("START");

  // --- Wait for Python's response ---
  while (!Serial.available()) {
    ; // wait until something comes
  }
  String response = Serial.readStringUntil('\n');
  Serial.print("Python said: ");
  Serial.println(response);
}

void setup() {
  Serial.begin(9600);
  delay(2000); // give Python time to connect

  moveDistance(100, 50);
  readNumber();
}

void loop() {
  // empty
}
