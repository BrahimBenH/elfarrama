bool code1 = true;
bool code2 = false;

#include <TimerFive.h>
// Motor pins
#define IN1 11
#define IN2 10
#define IN3 12
#define IN4 13

// Encoder pins
#define interruptPinRA 3
#define interruptPinRB 2
#define interruptPinLB 19
#define interruptPinLA 18

#define tickcmR 58.6
#define tickcmL 58.9

#define tickZR_P 882
#define tickZL_N 886
#define tickZL_P 887
#define tickZR_N 882


#define maxAcc 300
#define PI 3.14159265358979323846

long prevCountR = 0;
long prevCountL = 0;
int nb_ticks = 800;

float cmdV = 0;
float dsR = 0;
float dsL = 0;
float dS;
float totalL = 0;
float totalR = 0;
float dTheta = 0.0;
float theta = 0.0;

float cmdPwmRight = 0;
float cmdPwmLeft = 0;
float wheel_radius = 39;
float entreaxe = 320;

int i;
int incomingByte = 0;

unsigned long previousMillis;
unsigned long chrono;
long encoderLeftCount = 0;
long encoderRightCount = 0;
float currentvelocityRight = 0.0;
float currentvelocityLeft = 0.0;

long t = 0;
int speed_ech = 10;

float total_ech_l = 0;
float total_ech_r = 0;

long lastEncoderLeftCount = 0;
long lastEncoderRightCount = 0;

float dS_total = 0;

// Separate PID constants
float kpR =5, kiR = 0.0000001;
float kpL = 10, kiL = 0.00000001;
float kTheta = 4;

float kp_dour = 0.001;
float k_position =0.001;

float PWM_R = 0;
float PWM_L = 0;

float right_erreur = 0;
float left_erreur = 0;
float i_right_erreur = 0;
float i_left_erreur = 0;

float orientation = 0;
float i_orientation = 0;
float orientation_erreur = 0;
float i_orientation_erreur = 0;
float Theta_correction = 0;
float position_erreur = 0;

int sens = 1;

float PWM_MIN_R = 70;
float PWM_MAX_R = 250;
float PWM_MIN_L = 130;
float PWM_MAX_L = 250;

float PWM_MIN_DOURA = 85;
float PWM_MAX_DOURA = 150;

bool reachedLimitR = false;
bool reachedLimitL = false;

float current_speed;

// Encoder interrupts
void interruptR() {
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) encoderRightCount--;
  else encoderRightCount++;
}

void interruptL() {
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) encoderLeftCount++;
  else encoderLeftCount--;
}

float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks) {
  dsL = (deltaLeftCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;
  dsR = (deltaRightCount / (float)nb_ticks) * 2.0 * PI * wheel_radius;
  return (dsL + dsR) / 2.0;
}

void speed_calcul(void) {
  float right_encoder_speed = float(total_ech_r / (float(speed_ech) * 15 / 1000));
  float left_encoder_speed = float(total_ech_l / (float(speed_ech) * 15 / 1000));
  float alpha_speed = (float(total_ech_r - total_ech_l)) / (float(speed_ech) * 15 / 1000);

  currentvelocityRight = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * wheel_radius / 2;
  currentvelocityLeft = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * wheel_radius / 2;

  total_ech_l = 0;
  total_ech_r = 0;
}

void updateOdometrie() {
  t++;

  long deltaLeftCount = encoderLeftCount - lastEncoderLeftCount;
  long deltaRightCount = encoderRightCount - lastEncoderRightCount;

  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;

  dS = calculDistance(deltaLeftCount, deltaRightCount, wheel_radius, nb_ticks);

  totalL += dsL;
  totalR += dsR;
  dS_total += dS;

  total_ech_l += dsL;
  total_ech_r += dsR;

  dTheta = (dsR - dsL) / entreaxe;
  theta += dTheta;

  if (t % speed_ech == 0) speed_calcul();

  dsR = 0;

}

void setup() {
  Serial.begin(9600);

  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(8, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);

  Timer5.initialize(5000);
  Timer5.attachInterrupt(updateOdometrie);

  previousMillis = millis();
moveDistance(500, 300);

}

void stopmotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

float RadToDeg(float radians) { return radians * (180.0 / PI); }
float DegToRad(float degrees) { return degrees * (PI / 180.0); }

float ramp(int time) { return time * 0.6; }

void applyMotorCommand(float cmdPwmRight, float cmdPwmLeft) {
  digitalWrite(IN1, cmdPwmRight);
  digitalWrite(IN2, cmdPwmRight);
  digitalWrite(IN3, cmdPwmLeft);
  digitalWrite(IN4, cmdPwmLeft);
}

int constraint(float a, int min, int max) {
  if (a < min) return min;
  if (a > max) return max;
  return a;
}

float getcurrentVelocity(float dist, float t) { return dist / t; }

void run() {
  if (PWM_R > 0) {
    analogWrite(IN1, PWM_R);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, -PWM_R);
  }
  if (PWM_L > 0) {
    analogWrite(IN3, PWM_L);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, -PWM_L);
  }
 /* Serial.print ("pwm R   "); 
  Serial.print (PWM_R); 
   Serial.print ("     pwm L   "); 
  Serial.println (PWM_L); */
}

float erreur(float PWM, float min, float max) {
  if (PWM < min) PWM = min;
  if (PWM > max) PWM = max;
  return PWM;
}

void iniiit() {
  totalR = 0;
  totalL = 0;
  dS_total = 0;
  i_right_erreur = 0;
  i_left_erreur = 0;
  right_erreur = 0;
  left_erreur = 0;
  position_erreur = 0;
  orientation_erreur = 0;
  reachedLimitR = false;
  reachedLimitL = false;

}

// --- moveDistance with requested changes ---
float acceleration(float speed, float distance, float accel, float decel) {
  if (abs(dS_total) < accel) current_speed = (speed / accel) * abs(dS_total);
  else if (distance - abs(dS_total) < decel) current_speed = (speed / -decel) * abs(dS_total) + speed - ((distance - decel) * (speed / -decel));
  else current_speed=speed;
  return current_speed;
}

void moveDistance(float distance, float speed) {
  iniiit();

  float accel = speed * speed / (2 * maxAcc);
  float decel = accel;

  // Special case: small distance
  while (distance < accel + decel) {
    accel *= 0.8;
    decel *= 0.8;
    speed=sqrt(2*maxAcc*accel);

  }

  while (abs(dS_total - distance) > 10) {
    if ((dS_total - distance) < 0) sens = 1;
    else sens = -1;

    current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));

    // Right PID
    right_erreur = current_speed - currentvelocityRight;
    if (!reachedLimitR) i_right_erreur += right_erreur;
    PWM_R = kpR * right_erreur + kiR * i_right_erreur;

    if (abs(PWM_R) >= PWM_MAX_R && currentvelocityRight < current_speed) {
      reachedLimitR = true;
      i_right_erreur = 0;
      kiR = 0;
    }

    if (sens == 1) PWM_R = erreur(PWM_R, PWM_MIN_R, PWM_MAX_R);
    else PWM_R = erreur(PWM_R, -PWM_MAX_R, -PWM_MIN_R);

    // Left PID
    left_erreur = current_speed - currentvelocityLeft;
    if (!reachedLimitL) i_left_erreur += left_erreur;
    PWM_L = kpL * left_erreur + kiL * i_left_erreur;

    if (abs(PWM_L) >= PWM_MAX_L && currentvelocityLeft < current_speed) {
      reachedLimitL = true;
      i_left_erreur = 0;
      kiL = 0;
    }

    if (sens == 1) PWM_L = erreur(PWM_L, PWM_MIN_L, PWM_MAX_L);
    else PWM_L = erreur(PWM_L, -PWM_MAX_L, -PWM_MIN_L);

    // Orientation correction
    float orientation_erreur = totalR - totalL;
    float Theta_correction = kTheta * orientation_erreur;
    PWM_R -= Theta_correction;
    PWM_L += Theta_correction;

    if (sens == 1) {
      PWM_R = erreur(PWM_R, PWM_MIN_R, PWM_MAX_R);
      PWM_L = erreur(PWM_L, PWM_MIN_L, PWM_MAX_L);
    } else {
      PWM_R = erreur(PWM_R, -PWM_MAX_R, -PWM_MIN_R);
      PWM_L = erreur(PWM_L, -PWM_MAX_L, -PWM_MIN_L);
    }

    run();
    Serial.print(speed);
        Serial.print(" ");

    Serial.print(currentvelocityLeft);
            Serial.print(" ");
                Serial.print(current_speed);

        Serial.print(" ");

    Serial.println(currentvelocityRight);


  }
  stopmotors();
  //Serial.print(dS_total);
}

// --- Turn / dour function ---
void dour(float angle, float speed, bool stop) {
  iniiit();

  float distance = angle * PI * entreaxe / 180;  // target distance for turning
  float accel = speed * speed / (2 * maxAcc);
  float decel = accel;

  // Special case: small distance
  while (distance < accel + decel) {
    accel *= 0.8;
    decel *= 0.8;
  }

  while (abs((theta * 180) / PI - angle) > 2) {
    sens = (((theta * 180) / PI) - angle < 0) ? 1 : -1;
    if (stop && sens == -1) break;

    float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));

    // Right wheel PID
    right_erreur = current_speed - currentvelocityRight;
    if (!reachedLimitR) i_right_erreur += right_erreur;
    PWM_R = kpR * right_erreur + kiR * i_right_erreur;

    if (abs(PWM_R) >= PWM_MAX_R && current_speed < speed) {
      reachedLimitR = true;
      i_right_erreur = 0;
      kiR = 0;
    }

    PWM_R = (sens == 1) ? erreur(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA) : erreur(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);

    // Left wheel PID
    left_erreur = -current_speed - currentvelocityLeft;
    if (!reachedLimitL) i_left_erreur += left_erreur;
    PWM_L = kpL * left_erreur + kiL * i_left_erreur;

    if (abs(PWM_L) >= PWM_MAX_L && current_speed < speed) {
      reachedLimitL = true;
      i_left_erreur = 0;
      kiL = 0;
    }
    PWM_L = (sens == 1) ? erreur(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA) : erreur(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA);

    // Orientation correction
    orientation_erreur = totalR - totalL;
    Theta_correction = kTheta * orientation_erreur;
    PWM_R -= Theta_correction;
    PWM_L += Theta_correction;

    // Limit PWM after correction
    PWM_R = (sens == 1) ? erreur(PWM_R, PWM_MIN_DOURA, PWM_MAX_DOURA) : erreur(PWM_R, -PWM_MAX_DOURA, -PWM_MIN_DOURA);
    PWM_L = (sens == 1) ? erreur(PWM_L, PWM_MIN_DOURA, PWM_MAX_DOURA) : erreur(PWM_L, -PWM_MAX_DOURA, -PWM_MIN_DOURA);



    run();
  }
  stopmotors();
}


// Loop
void loop() {






}
