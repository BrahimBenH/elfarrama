void requestFromPython(String command) {
  Serial.println(command);  // send the command to Python
  while (!Serial.available()); // wait for a response
  String response = Serial.readStringUntil('\n');
  Serial.print("Python responded: ");
  Serial.println(response);
}

void getNumber() { requestFromPython("GET_NUMBER"); }
void getColor()  { requestFromPython("GET_COLOR"); }
void getShape()  { requestFromPython("GET_SHAPE"); }

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  getNumber();
  getColor();
  getShape();
}

void loop() {}
