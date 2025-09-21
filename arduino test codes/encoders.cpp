#define encoder_L_A 2
#define encoder_L_B 3

 long int nb_ticks_L = 0;

void ticks_L_A(){
  if(digitalRead(encoder_L_A)==digitalRead(encoder_L_B)){
    nb_ticks_L++;
  }else {
  nb_ticks_L--;
  } 
}
void ticks_L_B(){
  if(digitalRead(encoder_L_B)==digitalRead(encoder_L_A)){
    nb_ticks_L--;
  }else {
  nb_ticks_L++;
  } 
}
void setup() {
  Serial.begin(9600);

  pinMode(encoder_L_A, INPUT_PULLUP);
  pinMode(encoder_L_B, INPUT_PULLUP);

  attachInterrupt( digitalPinToInterrupt(encoder_L_A) , ticks_L_A, CHANGE);
  attachInterrupt( digitalPinToInterrupt(encoder_L_B) , ticks_L_B, CHANGE);

}
void loop() {
  Serial.println(nb_ticks_L);
}
