// ************************************************************************************
//   Pendulum Controller
// ----------------------------------------------------------------------------------
//   Version    :   1.1
//   Date       :   07 Feb 2017
//   Created by :   YR DENG
// ----------------------------------------------------------------------------------
//   Program Description
//  - To measure the period of the pendulum
//  - To control the length of the pendulum
// ----------------------------------------------------------------------------------
//   Version Notes
//  0.0     Initial Build. 
//          - Basic infrastructure for Arduino-Sahara Interface, functionality placeholders
//  1.0     Pendulum Period Measurement, Length Change Function, Manual Operation
//  1.1     Change period scan input to analog from digital
//          Change the condition of auto-mode measuring stage complete to using number of swing
//          feedback, instead of time.
//
// ----------------------------------------------------------------------------------
// ************************************************************************************

// --------------- Library --------------------------------------


// --------------- Pin Definition -------------------------------

#define STEPPER_PUL 12
#define STEPPER_DIR 13

#define PERIOD_SCAN A0

#define COIL_ACTIVATE 3
#define COIL_OVERRIDE 4
#define COIL_BRAKE 5

// Operation:
// Activate: force current to flow into coil
//  - Activate PNP: ON (0VDC - LOW)
//  - Override PNP: any state
//  - Brake relay: forward (0VDC - LOW)
// Acceleration: Circuit operating alone
//  - Activate PNP: OFF (5VDC - HIGH)
//  - Override PNP: Allow Current, ON (0VDC - LOW)
//  - Brake relay: forward (0VDC - LOW)
// Measure: prevent acceleration circuit operating, no current through
//  - Activate PNP: OFF (5VDC - HIGH)
//  - Override PNP: Prevent Current, OFF (5VDC - HIGH)
//  - Brake relay: forward (0VDC - LOW)
// Braking: Force current to the coil reversely
//  - Activate PNP: ON (0VDC - LOW)
//  - Override PNP: any state
//  - Brake relay: Reverse (5VDC - HIGH)
// Length Changing: same as measure, no circuit operation
// Idle, same as measure

// --------------- Variable Declaration -------------------------

// Mode
// - To define the operating condition of the pendulum setup
// - Automatic Mode (Experiment mode): execution of the experiment routine
// - Manual Mode (Calibration mode): For maintenance & calibration
// - Convention: integer (N); 
// - N: 1 = Auto Mode; 2 = Manual Mode;
// State
// - To define the operating status
// - Convention: integer(n)
// - n : 0 = idle; 1 = activate and acceleration; 2 = measurement; 3 = brake; 4 = length adjustment
// modeStart to capture the clock when enter the current mode (for timer)

int mode = 0;
int state = 0;
unsigned long modeStart = 0;

// Length Adjustment related
// - Current Position (in terms of steps) - mark the current positions of the platform (pivot point)
// - Target Position (in terms of steps)
// - Stepper will run towards the target (i.e. direction decided by the current position vs. target position) by 1 step per scan
//      - to avoid jamming up the controller computational resources

long curStep = 0L;
long tgtStep = 0L;

// Period Measurement
// - Counter to capture the number of time the pendulum passes the coil
// - Starting timer to capture the initial clock 

unsigned long periodCounter = 0L;
unsigned long lastCounter = 0L;
unsigned long periodTimer = 0;
float period = 0.0;
boolean previousScan = false;
boolean currentScan = false;

// Communication

char inputString[64] = "";           // Serial Comm - a string to hold incoming data
int inputPointer;                    // Serial Comm - input store pointer
boolean stringComplete = false;      // Serial Comm - whether the string is complete
unsigned long wdTimeComm = 0;
unsigned long wdTimer = 0;
boolean wdDisabled = true;

// Debug
unsigned long debugTimer = 0;
unsigned long debugTime = 0;



// --------------- Function: Motor Driving ----------------------
long runStepper(long cur,long tgt){
  long delta = 0;
  if (cur > tgt){
    digitalWrite(STEPPER_DIR,HIGH);
    delta = -1;
  }
  else{
    digitalWrite(STEPPER_DIR,LOW);
    delta = 1;
  }
  digitalWrite(STEPPER_PUL,HIGH);
  delayMicroseconds(10);
  digitalWrite(STEPPER_PUL,LOW);
  delayMicroseconds(500);
  return delta;
}

// --------------- Function: Input Scan -------------------------
boolean updateScan(){
  if (analogRead(PERIOD_SCAN)>700){
    return true;
  }else if (analogRead(PERIOD_SCAN)<500){
    return false;
  }
}

// --------------- Function: Period Calculation -----------------
void updatePeriod(){
  if ((periodCounter - lastCounter) >= 2L){
    period = ((float)(millis()-periodTimer))/1000;
    periodTimer = millis();
    lastCounter = periodCounter;
  }
}

// --------------- Function: Communication Processing -----------
// requestValue: request Arduino to provide data
// The following data would be provided for display / calculation
// - period
// - current location (step)
// - Target Location (step)
// - Operating mode

void requestValue(){
  Serial.print("Cpl-");
  Serial.print(period);
  Serial.print(";");
  Serial.print(curStep);
  Serial.print(";");
  Serial.print(tgtStep);
  Serial.print(";");
  Serial.print(mode);
  Serial.print(";");
  Serial.print(state);
  Serial.print(";");
  Serial.println(0);
}

// setValue: write target value to arduino local registers (for arduino to process)
// The following value accepted from the remote end
// - Target Location (steps): address 11

void setValue(int addr, float val){
  long intVal=(long)val;
  switch (addr){
    case 11:
      if (mode == 2 && state == 0){
        tgtStep = intVal;
        mode = 2;
        state = 4;
      }
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr"); 
  }
}

// setCommand: Receive command from remote end (Sahara)
// The following commands are accepted
// - STOP        (command index 11): stop automatic cycle, i.e. switch to manual mode
//    - accepted when running in auto mode
// - START       (command index 12): start automatic cycle, i.e. switch to automatic mode
//    - accepted when running in manual mode
// - MOVEUP      (command index 21): increase pendulum length by 1mm 
//    - accepted when running in manual mode and idle state
// - MOVEDOWN    (command index 22): increase pendulum length by 1mm 
//    - accepted when running in manual mode and idle state
// - FASTUP      (command index 23): increase pendulum length by 5mm (1 round)
//    - accepted when running in manual mode and idle state
// - FASTDOWN    (command index 24): increase pendulum length by 5mm (1 round)
//    - accepted when running in manual mode and idle state
// - ACCEL       (command index 31): activate and accelerate the pendulum swing
//    - accepted when running in manual mode
// - MEASURE     (command index 32): stop acceleration and start recording the period
//    - accepted when running in manual mode 
// - FORCESTOP   (command index 33): stop period recording and brake the swing
//    - accepted when running in manual mode
// - CALIBRATION (command index 41): set the current location to be zero point
//    - accepted when running in manual mode and idle state
// - CLEANUP     (command index 99): stop activities and back to zero position
//    - accepted in any mode

void setCommand(int commandIndex){
  switch (commandIndex){
    case 11:
      if (mode == 1){
        digitalWrite(STEPPER_PUL,LOW);
        digitalWrite(STEPPER_DIR,LOW);
        
        digitalWrite(COIL_ACTIVATE,HIGH);
        digitalWrite(COIL_OVERRIDE,HIGH);
        digitalWrite(COIL_BRAKE,LOW);
        mode = 2;
        state = 0;
      }
      Serial.println("Cpl");
      break;
    case 12:
      if (mode == 2){
        digitalWrite(STEPPER_PUL,LOW);
        digitalWrite(STEPPER_DIR,LOW);

        digitalWrite(COIL_ACTIVATE,LOW);
        digitalWrite(COIL_OVERRIDE,HIGH);
        digitalWrite(COIL_BRAKE,LOW);
        delay(1000);
        digitalWrite(COIL_ACTIVATE,HIGH);
        digitalWrite(COIL_OVERRIDE,LOW);
        digitalWrite(COIL_BRAKE,LOW);
        mode = 1;
        state = 1;
        modeStart = millis();
      }
      Serial.println("Cpl");
      break;
    case 21:
      if (mode == 2 && (state == 0 || state == 4)){
        tgtStep = tgtStep +2000L;
        mode = 2;
        state = 4;
      }
      Serial.println("Cpl");
      break;
    case 22:
      if (mode == 2 && (state == 0 || state == 4)){
        tgtStep = tgtStep -2000L;
        mode = 2;
        state = 4;
      }
      Serial.println("Cpl");
      break;
    case 23:
      if (mode == 2 && (state == 0 || state == 4)){
        tgtStep = tgtStep + 20000L;
        mode = 2;
        state = 4;
      }
      Serial.println("Cpl");
      break;
    case 24:
      if (mode == 2 && (state == 0 || state == 4)){
        tgtStep = tgtStep - 20000L;
        mode = 2;
        state = 4;
      }
      Serial.println("Cpl");
      break;
    case 31:
      if (mode == 2){
        digitalWrite(STEPPER_PUL,LOW);
        digitalWrite(STEPPER_DIR,LOW);

        digitalWrite(COIL_ACTIVATE,LOW);
        digitalWrite(COIL_OVERRIDE,HIGH);
        digitalWrite(COIL_BRAKE,LOW);
        delay(1000);
        digitalWrite(COIL_ACTIVATE,HIGH);
        digitalWrite(COIL_OVERRIDE,LOW);
        digitalWrite(COIL_BRAKE,LOW);
        mode = 2;
        state = 1;
      }
      Serial.println("Cpl");
      break;
    case 32:
      if (mode == 2){
        mode = 2;
        state = 2;
        periodCounter = 0L;
        lastCounter = 0L;
        periodTimer = millis();
        period = 0.0;
      }
      Serial.println("Cpl");
      break;
    case 33:
      if (mode == 2){
        mode = 2;
        state = 3;
        modeStart = millis();
      }
      Serial.println("Cpl");
      break;

    // debug command
    case 37:
      if (mode == 2){
        mode = 2;
        state = 7;
        periodCounter = 0L;
        lastCounter = 0L;
        periodTimer = millis();
        period = 0.0;
      }
      Serial.println("Cpl");
      break;
    case 41:
      if (mode == 2 && state == 0){
        curStep = 0L;
        tgtStep = 0L;
      }
      Serial.println("Cpl");
      break;
    case 99:
      digitalWrite(STEPPER_PUL,LOW);
      digitalWrite(STEPPER_DIR,LOW);
 
      digitalWrite(COIL_ACTIVATE,HIGH);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,LOW);
      delay(1000);
      tgtStep = 0;
      mode = 2;
      state = 4;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr"); 
  }
}

// --------------- Main: System Setup & Initializtion -----------
void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  pinMode(STEPPER_PUL,OUTPUT);
  pinMode(STEPPER_DIR,OUTPUT);
  pinMode(PERIOD_SCAN,INPUT);
  pinMode(COIL_ACTIVATE,OUTPUT);
  pinMode(COIL_OVERRIDE,OUTPUT);
  pinMode(COIL_BRAKE,OUTPUT);

  digitalWrite(STEPPER_PUL,LOW);
  digitalWrite(STEPPER_DIR,LOW);
  digitalWrite(COIL_ACTIVATE,HIGH);
  digitalWrite(COIL_OVERRIDE,HIGH);
  digitalWrite(COIL_BRAKE,LOW);

  mode = 2;
  state = 0;
  modeStart = 0;
  curStep = 0L;
  tgtStep = 0L;

  periodCounter = 0L;
  lastCounter = 0L;
  periodTimer = 0;
  period = 0.0;
  previousScan = false;
  currentScan = false;
  debugTimer = millis();
  debugTime = 0;
  
  Serial.begin(115200);
}
// --------------- Main: Controller Main Processing -------------
void loop() {
  currentScan = updateScan();
  if ((currentScan== true)&&(previousScan==false)){
    periodCounter = periodCounter +1L;
    previousScan = true;
    debugTimer = millis();
  }else if ((currentScan== false)&&(previousScan==true)){
    previousScan = false;
    debugTime = millis()-debugTimer;
  }

  if (mode == 1 && state == 1){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,LOW);
    digitalWrite(COIL_BRAKE,LOW);
    
    // Process complete condition clear, actions to move to next state
    if (millis() - modeStart > 10000){
      mode = 1;
      state = 2;
      modeStart = millis();
      periodCounter = 0;
      periodTimer = millis();
      period = 0.0;
    }
    
  }
  if (mode == 1 && state == 2){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,LOW);
    updatePeriod();
    // Process complete condition clear, actions to move to next state
    // if 200 swing detected, move to next stage
    // if 30 second without swing detected, consider malfunction, break to manual mode
    if (periodCounter >= 50){
      mode = 1;
      state = 3;
      modeStart = millis();
    }else if ((millis() - modeStart > 30000)&&(periodCounter <= 10)){
      mode = 2;
      state = 0;
    }    
  }
  if (mode == 1 && state == 3){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,LOW);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,HIGH);
    // Process complete condition clear, actions to move to next state
    if (millis() - modeStart > 10000){
      if ((curStep >= 158000L) &&(curStep < 160000L)){
        tgtStep = 160000L;
      }else if (curStep == 160000L){
        tgtStep = 0;
      }else{
        tgtStep = tgtStep + 20000L;
      }
      mode = 1;
      state = 4;
    }
  }
  if (mode == 1 && state == 4){
    // Process actions
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,LOW);
    if (curStep != tgtStep){
      curStep = curStep + runStepper(curStep, tgtStep);      
    }
    // Process complete condition clear, actions to move to next state
    else{
      digitalWrite(STEPPER_PUL,LOW);
      digitalWrite(STEPPER_DIR,LOW);

      digitalWrite(COIL_ACTIVATE,LOW);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,LOW);
      delay(1000);
      digitalWrite(COIL_ACTIVATE,HIGH);
      digitalWrite(COIL_OVERRIDE,LOW);
      digitalWrite(COIL_BRAKE,LOW);
      mode = 1;
      state = 1;
      modeStart = millis();
    }
  }
  if (mode == 2 && state == 0){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,LOW);
    
    // Process complete condition clear, actions to move to next state
    // Manual mode - move to other state based on manual input, no automatic condition        
  }
  if (mode == 2 && state == 1){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,LOW);
    digitalWrite(COIL_BRAKE,LOW);
    
    // Process complete condition clear, actions to move to next state
    // Manual mode - move to other state based on manual input, no automatic condition        
  }
  if (mode == 2 && state == 2){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,LOW);
    updatePeriod();
    // Process complete condition clear, actions to move to next state
    // Manual mode - move to other state based on manual input, no automatic condition            
  }
  if (mode == 2 && state == 3){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,LOW);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,HIGH);
    // Process complete condition clear, actions to move to next state
    if (millis() - modeStart > 10000){
      mode = 2;
      state = 0;
    }        
  }
  if (mode == 2 && state == 4){
    // Process actions
    if (curStep != tgtStep){
      curStep = curStep + runStepper(curStep, tgtStep);      
    }
    // Process complete condition clear, actions to move to next state
    else{
      mode = 2;
      state = 0;         
    }
  }
  // debug mode
  if (mode == 2 && state == 7){
    // Process actions
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
    
    digitalWrite(COIL_ACTIVATE,HIGH);
    digitalWrite(COIL_OVERRIDE,HIGH);
    digitalWrite(COIL_BRAKE,LOW);

    // Process complete condition clear, actions to move to next state
    // Manual mode - move to other state based on manual input, no automatic condition            
  }
    

}

// --------------- Event: Serial Comm Event Handling ------------
void serialEvent() {
    
    while (Serial.available() && !stringComplete) {  
      // get the new byte:
      char inChar = (char)Serial.read();
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if ((inChar == '\n')||(inputPointer == 63)) {
        stringComplete = true;
        inputPointer = 0;
      }else if (inputPointer < 63){
      // allocate to the inputString buffer
      // move pointer to next location
        inputString[inputPointer] = inChar;
        inputPointer++;
      }
    }
    
    // *************************
    // Interpret input
    if (stringComplete) {
        char msgHeader[8] = "";
        char msgReq[5] = "";
        char msgAddr[3] = "";
        char msgVal[40] = "";
        for (int i = 0; i<7;i++){
          msgHeader[i] = inputString[i];
        }
        for (int i = 0; i<4;i++){
          msgReq[i] = inputString[i+7];
        }
        for (int i = 0; i<2;i++){
          if ((inputString[i+17] < '+') || (inputString[i+17] > '9') ||(inputString[i+17] == '/') ) {
            break; 
          }
          msgAddr[i] = inputString[i+17];
        }
        for (int i = 0; i<40;i++){
          if ((inputString[i+24] < '+') || (inputString[i+24] > 'z') ||((inputString[i+24] > ';')&&(inputString[i+24] < 'A'))||(inputString[i+24] == '/')||((inputString[i+24] > 'Z')&&(inputString[i+24] < 'a'))) {
            break; 
          }
          msgVal[i] = inputString[i+24];
        }
        
        int intAddr = atoi(msgAddr);
        float flVal = atof(msgVal);
        if(strcmp(msgHeader,"rlab://") != 0) {                                           
          Serial.println("Err-input");
          Serial.flush();
        }else{
          if (strcmp(msgReq,"REQV") == 0){
            requestValue();
            Serial.flush();
          }else if (strcmp(msgReq,"SETV") == 0){
            setValue(intAddr,flVal);
            Serial.flush();
          }else if (strcmp(msgReq,"COMD") == 0){
            setCommand(intAddr);
            Serial.flush();
          }else if (strcmp(msgReq,"DEBG") == 0){
            Serial.println(debugTime);
            Serial.println(periodCounter);
            Serial.println(digitalRead(2)); 
            Serial.println(digitalRead(3));
            Serial.println(digitalRead(4));
            Serial.println(digitalRead(5));
            Serial.flush();
          }else{
            Serial.println("Err-command");
            Serial.flush();
          }              
        }
        for (int i=0;i<47;i++){
          inputString[i] = '\0';
        }
        stringComplete = false;

        // Watchdog
        if (wdDisabled == true){
          wdDisabled = false;
        }
        wdTimeComm = millis();
      }
}
