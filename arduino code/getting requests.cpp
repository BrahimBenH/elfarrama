String lastResponse = "";

String requestFromPython(String command) {
  Serial.println(command);           // send the command to Python
  Serial.setTimeout(1000);           // blocking read timeout (1s)

  String response = "";
  // read one (may block up to timeout) then drain remaining available lines
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() > 0) response = line;

  while (Serial.available()) {
    line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) response = line; // keep last non-empty line
  }

  if (response.length() == 0) {
    Serial.println("No response");
    return String("");
  }

  if (response == lastResponse) {
    Serial.println("Same as last response, ignoring");
    return String("");
  }

  lastResponse = response;
  Serial.print("Python responded: ");
  Serial.println(response);
  return response;
}

void getNumber() { requestFromPython("GET_NUMBER"); }
void getColor()  { requestFromPython("GET_COLOR"); }
void getShape()  { requestFromPython("GET_SHAPE"); }

void setup() {
  Serial.begin(9600);
  while (!Serial.available()); // wait for a response
  String response = Serial.readStringUntil('\n');
  Serial.print("OK");
  delay(2000);
  
  getNumber();
  delay(1000);
  getColor();
    delay(1000);

  getShape();
}

void loop() {}
