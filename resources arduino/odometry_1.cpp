//MATHEMATICAL RELATIONS TO ESTIMATE POSITION AND ORIENTATION

/*
r: Radius of the wheels (in meters).
L: Distance between the two wheels (wheelbase) in meters.
Δθ: Change in orientation (heading angle) in radians.
Δx: Change in x-position.
Δy: Change in y-position.
ΔsL: Distance traveled by the left wheel (in meters).
ΔsR: Distance traveled by the right wheel (in meters).
Δs: Distance traveled by the center of the robot.
θ: Robot's orientation (heading) with respect to a reference frame (in radians).
𝑥 and y: Robot's position in the global coordinate system (in meters).
Δt: Time interval during which the movement occurred (in seconds) 
*/
#define ENCODER_LEFT_PIN_A 18  // Left encoder pin A
#define ENCODER_LEFT_PIN_B 19  // Left encoder pin B
#define ENCODER_RIGHT_PIN_A 20 // Right encoder pin A
#define ENCODER_RIGHT_PIN_B 21 // Right encoder pin B
#define PI 3.1415926535897932384626433832795

int nb_ticks= 1024;
float wheel_raduis = 0.08; 
float encoder_distance = 0.32; //mazelet bch tetbadel

long long encoderLeftCount = 0;
long long encoderRightCount = 0;

float x = 0.0;  
float y = 0.0;  
float theta = 0.0;  

long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

unsigned long lastTime = 0;
const float DELTA_T = ....; //kol 9dech nhebou ne7sbou odometry

void leftEncoder() {
  if (digitalRead(ENCODER_LEFT_PIN_A) == digitalRead(ENCODER_LEFT_PIN_B)) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void rightEncoder() {
  if (digitalRead(ENCODER_RIGHT_PIN_A) == digitalRead(ENCODER_RIGHT_PIN_B)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_LEFT_PIN_A, INPUT);
  pinMode(ENCODER_LEFT_PIN_B, INPUT);
  pinMode(ENCODER_RIGHT_PIN_A, INPUT);
  pinMode(ENCODER_RIGHT_PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_A), rightEncoder, CHANGE);
  
  lastTime = millis();
}

void loop() {

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= DELTA_T * 1000) {
    lastTime = currentTime;
    
    // 3dad pulses fi kol update
    long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
    long deltaRightCount = encoderRightCount - lastEncoderRightCount;
    
    // n updaytiwhee bch matarje3ch li zero fi kol loop
    lastEncoderLeftCount = encoderLeftCount;
    lastEncoderRightCount = encoderRightCount;
    
    // distance ili ya3melha kol encodeur
    float dsL = (deltaLeftCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;
    float dsR = (deltaRightCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;

    float dS = (deltaSLeft + deltaSRight) / 2.0; // average mte3 position
    float dTheta = (dsR - dsL) / encoder_distance; //average me3 orientation
  
   // ntal3ou beha position w angle d'orientation jdida 
    x += dS * cos(theta + dTheta / 2.0);
    y += dS * sin(theta + deTheta / 2.0);
    theta += deltaTheta;

    // nsal7ou beha angle bch nkhaliwhee entre pi et -pi
    if (theta > PI) {
      theta -= 2.0 * PI;
    } else if (theta < -PI) {
      theta += 2.0 * PI;
    }

    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" m, y: ");
    Serial.print(y);
    Serial.print(" m, theta: ");
    Serial.print(theta * 180.0 / PI);
    Serial.println(" degrees");
  }
}

