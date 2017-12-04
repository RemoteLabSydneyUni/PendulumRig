int voltage = 0;
int trigger_in = 0;
int lockout_in = 0;
int lockout_out = 0;
unsigned long initialStart = 0L;

int clock_sig = 0;
void setup() {
  // put your setup code here, to run once:
  voltage = 0;
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  initialStart = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((millis() - initialStart) < 100){
    digitalWrite(3, LOW);
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
  }else{
    digitalWrite(3, HIGH);
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
  }
  voltage = analogRead(A0);
  trigger_in = analogRead(A2);
  lockout_in = analogRead(A3);
  lockout_out = analogRead(A4);
//  if ((millis() - initialStart) < 10000){
  Serial.println(voltage);
//  Serial.print(" ");
//  Serial.print(trigger_in);
//  Serial.print(" ");
//  Serial.print(lockout_in);
//  Serial.print(" ");
//  Serial.println(lockout_out);
  delay(5);
//  }
}
