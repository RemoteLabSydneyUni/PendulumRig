// ************************************************************************************
//   Motor Tester
// ----------------------------------------------------------------------------------
//   Version    :   0.0
//   Date       :   17 Feb 2017
//   Created by :   YR DENG
// ----------------------------------------------------------------------------------
//   Program Description
//  - To test the stepper motor
// ----------------------------------------------------------------------------------
//   Version Notes
//  0.0     Initial Build. 
//
// ----------------------------------------------------------------------------------
// ************************************************************************************

// --------------- Library --------------------------------------


// --------------- Pin Definition -------------------------------

#define STEPPER_PUL 12
#define STEPPER_DIR 13

int count = 0;
// --------------- Main: System Setup & Initializtion -----------
void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  pinMode(STEPPER_PUL,OUTPUT);
  pinMode(STEPPER_DIR,OUTPUT);


  digitalWrite(STEPPER_PUL,LOW);
  digitalWrite(STEPPER_DIR,LOW);


  Serial.begin(115200);
}
// --------------- Main: Controller Main Processing -------------
void loop() {
  for(int i=0;i<3;i++){
    digitalWrite(STEPPER_DIR,LOW);
    for (int j=0; j<20000;j++){
      digitalWrite(STEPPER_PUL,HIGH);
      delayMicroseconds(10);
      digitalWrite(STEPPER_PUL,LOW);
      delay(2);
    }
    delay(10000);
    count= count+400;
    Serial.println(count);
  } 
  for(int i=0;i<3;i++){
    digitalWrite(STEPPER_DIR,HIGH);
    for (int j=0; j<20000;j++){
      digitalWrite(STEPPER_PUL,HIGH);
      delayMicroseconds(10);
      digitalWrite(STEPPER_PUL,LOW);
      delay(2);
    }
    delay(10000);
    count= count+400;
    Serial.println(count);
  } 
}

