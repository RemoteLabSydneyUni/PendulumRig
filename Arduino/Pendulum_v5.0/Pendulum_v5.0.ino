// ************************************************************************************
//   Pendulum Controller
// ----------------------------------------------------------------------------------
//   Version    :   5.0
//   Date       :   22 Mar 2017
//   Created by :   YR DENG
//   Modify Date:   17 Nov 2017
//   Modified by:   YR DENG
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
//  2.0     Re-structure the program.
//          - Activation become an independant state
//          - Activation & acceleration state ends based on feedback, instead of time
//          - Coil control profile and motor control profile managed centralized, and call by
//            individual states, instead of commands.
//          - Bulky descriptive comments relocated to separate file (flowchart)
//  3.0     Relocate the recipe management to Rig Client
//          - Add new state in Auto mode to handle handshake
//          - Dynamically calculate measurement count required
//          - remove hard-coded recipe
//          - change address for setValue
//  4.0     Modify cleanup strategy
//          - record last position at cleanup
//          - load last position at startup
//  5.0     Add Pendulum Length feedback with VL53L0X time of flight sensor
//          - Take average of 5 distance samples
//          - Fine tune after every length change
//          - Add handler when distance sensor fails
//
// ----------------------------------------------------------------------------------
// ************************************************************************************

// --------------- Library --------------------------------------
#include <Wire.h>
#include <VL53L0X.h>

// --------------- Pin Definition -------------------------------

#define STEPPER_PUL 12          // Motor control, triggering pulse      (OUT)
#define STEPPER_DIR 13          // Motor control, direction             (OUT)

#define PERIOD_SCAN A0          // Pendulum Detection input             (IN)

#define COIL_ACTIVATE 3         // Coil Control, activation             (OUT)
#define COIL_OVERRIDE 4         // Coil Control, override acceleration  (OUT)
#define COIL_BRAKE 5            // Coil Control, force stop             (OUT)

// --------------- Variable Declaration -------------------------

// Mode Info
int mode = 0;                     // Mode indication (Auto = 1; Manual = 2)
int state = 0;                    // State indication (Refer to flowchart)

// Mode Manager
unsigned long modeStart = 0;      // Mode timer (System time when enter current state)

// Motor Controller
long curStep = 0L;                // Current Step Count
long tgtStep = 0L;                // Target Step Count

// Input Acquisition
boolean previousScan = false;     // Current Detected Indicator
boolean currentScan = false;      // Previous Detected Indicator, for tracing the rising edge

// Input Processing
unsigned long periodCounter = 0L; // Passing-by Counter, times the pendulum passes by the coil
unsigned long lastCounter = 0L;   // Previous Counter, used to mark the starting point of period calculation
unsigned long periodTimer = 0;    // Period Timer, used to mark the starting point of period calculation

// Calculated Value Register
double period = 0.0;

// Communication
char inputString[64] = "";        // Serial Comm - a string to hold incoming data
int inputPointer;                 // Serial Comm - input store pointer
boolean stringComplete = false;   // Serial Comm - whether the string is complete

// Experiment Parameter
unsigned long expActTime = 500;   // Activation Time for corresponding experiment
int expAccelCount = 15;           // Acceleration counts for corresponding experiment
int expMeasureCount = 50;         // Measurement Counts for corresponding experiment

boolean loadCheck = false;

// Length Feedback
VL53L0X sensor;                       
int sampleCount = 0;              // Samples collected (for average)
int sampleBuffer = 0;             // Buffer to store sample collected
int curLength;                    // Length feedback (in mm)
int tgtLength;                    // Target length (specified by Rig)
int minLength = 280;              // Minimum length (at 0 step)
int sensorFault = 0;              // Sensor health flag (1 = faulty)
int sensorErrCount = 0;           // Sensor error count
int sensorValid = 0;              // Validated Sensor reading
                                  
// Debug

// --------------- Function: Sensor Health Check  ---------------
void sensorCheck(){
  int reading = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred() || (reading >= 2000) || (reading <= minLength - 100)){
     sensorErrCount++;
  }else{
    sensorErrCount = 0;
    sensorValid = reading;
  }
  if (sensorErrCount>6){
    sensorFault = 1;
  }else{
    sensorFault = 0;
  }
}


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

// --------------- Function: Input Interpretation ---------------
void updateScan(){
  if (analogRead(PERIOD_SCAN)>700){
    currentScan = true;
  }else if (analogRead(PERIOD_SCAN)<500){
    currentScan = false;
  }
  if ((currentScan== true)&&(previousScan==false)){
    periodCounter = periodCounter +1L;
    previousScan = true;
  }else if ((currentScan== false)&&(previousScan==true)){
    previousScan = false;
  }
}

// --------------- Function: Period Calculation -----------------
void updatePeriod(){
  if ((periodCounter - lastCounter) >= 2L){
    period = ((double)(millis()-periodTimer))/1000.0;
    periodTimer = millis();
    lastCounter = periodCounter;
  }
}

// --------------- Function: Communication Processing -----------
// Data Fetch (REQV)
void requestValue(){
  Serial.print("Cpl-");
  Serial.print(period);
  Serial.print(";");
  Serial.print(curStep);
  Serial.print(";");
  Serial.print(tgtStep);
  Serial.print(";");
  Serial.print(curLength);
  Serial.print(";");
  Serial.print(tgtLength);
  Serial.print(";");
  Serial.print(mode);
  Serial.print(";");
  Serial.print(state);
  Serial.print(";");
  Serial.println(0);
}

// Data Write (SETV)
void setValue(int addr, long val){
  switch (addr){
    case 11:
      if (mode == 9 && state == 0){
        tgtLength = (int)val;
        tgtStep = (long)(tgtLength - curLength)*200L + curStep;
        mode = 9;
        state = 1;
      }
      Serial.println("Cpl");
      break;
    case 21:
      if ((mode == 2 && state == 0) || (mode == 2 && state == 5)){
        tgtLength = (int)val;
        tgtStep = max(min((long)(tgtLength - curLength) *200L +curStep, 160000L),0L);
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    case 91:
      if (mode == 0 && state == 1){
        curStep = val;
        loadCheck = true;
      }
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr"); 
  }
}

// Commands (COMD)
void setCommand(int commandIndex){
  switch (commandIndex){
    case 11:
//      if (mode == 1){
        mode = 2;
        state = 0;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
//      }                           // change stop button to ESD button: can be used in any case
      Serial.println("Cpl");
      break;
    
    case 12:
      if ((mode == 2)&&(state == 0)){
        mode = 1;
        state = 1;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
      }
      Serial.println("Cpl");
      break;
    
    case 13:
      if ((mode == 9)&&(state == 1)){
        mode = 1;
        state = 5;
      }
      Serial.println("Cpl");
      break;
      
    case 21:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep +2000L;
        tgtLength = tgtLength + 10;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 22:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep -2000L;
        tgtLength = tgtLength - 10;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 23:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep + 20000L;
        tgtLength = tgtLength + 100;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 24:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep - 20000L;
        tgtLength = tgtLength - 100;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 25:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep + 200L;
        tgtLength = tgtLength + 1;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 26:
      if (mode == 2 && (state == 0 || state == 5)){
        tgtStep = tgtStep - 200L;
        tgtLength = tgtLength - 1;
        mode = 2;
        state = 5;
      }
      Serial.println("Cpl");
      break;
    
    case 31:
      if ((mode == 2)&&(state == 0)){
        mode = 2;
        state = 1;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
      }
      Serial.println("Cpl");
      break;
    
    case 32:
      if (mode == 2 && (state == 0 || state == 2)){
        mode = 2;
        state = 3;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
        periodTimer = millis();
        period = 0.0;
      }
      Serial.println("Cpl");
      break;
    
    case 33:
      if (mode == 2 && (state == 0 || state == 2 || state == 3)){
        mode = 2;
        state = 4;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
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
      
    case 91:
      if (mode == 0 && state == 0){
        mode = 0;
        state = 1;
      }
      Serial.println("Cpl");
      break;
    
    case 99:
      mode = 0;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;      
      Serial.println("Cpl");
      break;
    
    default:
      Serial.println("Err-addr"); 
  }
}

// Enforce Coil Control Profile
void updateCoilCtrl(int setNumber){
  switch (setNumber){
    case 0:
      digitalWrite(COIL_ACTIVATE,HIGH);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,LOW);
      break;
    case 1:
      digitalWrite(COIL_ACTIVATE,LOW);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,LOW);
      break;
    case 2:
      digitalWrite(COIL_ACTIVATE,HIGH);
      digitalWrite(COIL_OVERRIDE,LOW);
      digitalWrite(COIL_BRAKE,LOW);
      break;
    case 3:
      digitalWrite(COIL_ACTIVATE,LOW);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,HIGH);
      break;
    default:
      digitalWrite(COIL_ACTIVATE,HIGH);
      digitalWrite(COIL_OVERRIDE,HIGH);
      digitalWrite(COIL_BRAKE,LOW);
  }
}

// Enforce Motor Control Profile
void updateMotorCtrl(int setNumber){
  if (setNumber == 1){
    digitalWrite(STEPPER_PUL,LOW);
    digitalWrite(STEPPER_DIR,LOW);
  }
}


// --------------- Main: System Setup & Initializtion -----------
void setup() {
  // Comm Init
  Serial.begin(115200);
  
  // Pin Init
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

  // Variable Init
  curStep = 0L;
  tgtStep = 0L;

  periodCounter = 0L;
  lastCounter = 0L;
  periodTimer = 0;
  period = 0.0;
  previousScan = false;
  currentScan = false;
  loadCheck = false;
  expActTime = 500;       // Default activation time: 500ms
  expAccelCount = 15;     // Default acceleration count: 15 times
  expMeasureCount = 50;   // Default measure count : 50 times

  // Default Mode
  mode = 0;
  state = 0;
  modeStart = millis();

  // Sensor Configuration
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(200000);
  
  sampleCount = 0;
  sampleBuffer = 0;
  curLength = 0;
  tgtLength = 0;
  minLength = 280;
  sensorFault = 0;
  sensorErrCount = 0;
  sensorValid = 0;
}

// --------------- Main: Controller Main Processing -------------
void loop() {
  // General Input Update
  updateScan();

  if (mode == 0 && state == 1){
    sensorCheck();
    if (sensorValid != 0){
      if (loadCheck == true){
        mode = 2;
        state = 0;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
        loadCheck = false;
        tgtStep = curStep;
        curLength = sensorValid;
        tgtLength = curLength;
      }
    }else if (sensorFault == 1){
      if (loadCheck == true){
        mode = 2;
        state = 0;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
        loadCheck = false;
        tgtStep = curStep;
        curLength = curStep/200+280;
        tgtLength = curLength;
      }
    }
  }

  // States Implementation
  if (mode == 1 && state == 1){
    updateCoilCtrl(1);
    updateMotorCtrl(1);
    
    // Auto Transition Conditions
    if (millis() - modeStart > expActTime){
      // Auto Transition Actions
      mode = 1;
      state = 2;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }    
  }
  
  if (mode == 1 && state == 2){
    updateCoilCtrl(2);
    updateMotorCtrl(1);
    
    // Auto Transition Conditions with error handling
    if (periodCounter >= expAccelCount){
      mode = 1;
      state = 3;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
      periodTimer = 0;
      period = 0.0;
    }else if ((millis() - modeStart > 60000)&&(periodCounter <= 10)){
      mode = 2;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }    
  }
  
  if (mode == 1 && state == 3){
    updateCoilCtrl(0);
    updateMotorCtrl(1);
    
    // Process actions
    updatePeriod();

    // Calculate measure count based on current length
    expMeasureCount = (int)map(curStep, 0L, 160000L, 50, 20);
    // Auto Transition Conditions with error handling
    if (periodCounter >= expMeasureCount){
      mode = 1;
      state = 4;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }else if ((millis() - modeStart > 60000)&&(periodCounter <= 10)){
      mode = 2;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }
  
  if (mode == 1 && state == 4){
    updateCoilCtrl(3);
    updateMotorCtrl(1);
    // Auto Transition Conditions
    if (millis() - modeStart > 45000){
      // Auto Transition Actions
      
      mode = 9;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }

  if (mode == 1 && state == 5){
    updateCoilCtrl(3);
    updateMotorCtrl(0);
    // Auto Transition Conditions
    if (curStep != tgtStep){
      // Process actions
      curStep = curStep + runStepper(curStep, tgtStep);      
    }else{
      // Auto Transition Actions
      mode = 9;
      state = 6;
      sampleCount = 0;
      sampleBuffer = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }
  if (mode == 9 && state == 6){
    // stablization
    if (millis() - modeStart > 1000){
      mode = 1;
      state = 6;
      sampleCount = 0;
      sampleBuffer = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }
  if (mode == 1 && state == 6){
    updateCoilCtrl(3);
    updateMotorCtrl(0);
    sensorCheck();
    // Auto Transition Conditions
    if (sensorFault == 0){
      if (sampleCount < 10){
        sampleBuffer = sampleBuffer + sensorValid;
        sampleCount++;
      }else{
        curLength = sampleBuffer / 10;
        if ((curLength < (tgtLength - 10))||(curLength > (tgtLength + 10))){
          // Process actions
          tgtStep = max(min(curStep - (long)(curLength - tgtLength) *200L, 160000L),0L);
          mode = 1;
          state = 5;
          modeStart = millis();
          periodCounter = 0L;
          lastCounter = 0L;      
        }else{
          // Auto Transition Actions
          mode = 1;
          state = 1;
          modeStart = millis();
          periodCounter = 0L;
          lastCounter = 0L;
        }
      }
    }else{
      mode = 1;
      state = 1;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }
  
  if (mode == 9 && state == 0){
    updateCoilCtrl(0);
    updateMotorCtrl(1);
  }
  if (mode == 9 && state == 1){
    updateCoilCtrl(0);
    updateMotorCtrl(1);
  }
  
  if (mode == 2 && state == 0){
    updateCoilCtrl(0);
    updateMotorCtrl(1);
  }
  
  if (mode == 2 && state == 1){
    updateCoilCtrl(1);
    updateMotorCtrl(1);
    // Auto Transition Conditions
    if (millis() - modeStart > expActTime){
      // Auto Transition Actions
      mode = 2;
      state = 2;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }        
  }
  
  if (mode == 2 && state == 2){
    updateCoilCtrl(2);
    updateMotorCtrl(1);          
  }
  
  if (mode == 2 && state == 3){
    updateCoilCtrl(0);
    updateMotorCtrl(1);
    // Process actions
    updatePeriod();       
  }

  if (mode == 2 && state == 4){
    updateCoilCtrl(3);
    updateMotorCtrl(1);
    // Auto Transition Conditions
    if (millis() - modeStart > 45000){
      mode = 2;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }

  if (mode == 2 && state == 5){
    updateCoilCtrl(3);
    updateMotorCtrl(0);
    // Auto Transition Conditions
    if (curStep != tgtStep){
      // Process actions
      curStep = curStep + runStepper(curStep, tgtStep);      
    }else{
      // Auto Transition Actions
      mode = 2;
      state = 6;
      sampleCount = 0;
      sampleBuffer = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
  }
  
  if (mode == 2 && state == 6){
    updateCoilCtrl(3);
    updateMotorCtrl(0);
    sensorCheck();
    // Auto Transition Conditions
    if (sensorFault == 0){
      if (sampleCount < 10){
        sampleBuffer = sampleBuffer + sensorValid;
        sampleCount++;
      }else{
        curLength = sampleBuffer / 10;
        tgtLength = curLength;
        mode = 2;
        state = 0;
        modeStart = millis();
        periodCounter = 0L;
        lastCounter = 0L;
      }
    }else{
      mode = 2;
      state = 0;
      modeStart = millis();
      periodCounter = 0L;
      lastCounter = 0L;
    }
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
        long flVal = atol(msgVal);
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
          }else{
            Serial.println("Err-command");
            Serial.flush();
          }              
        }
        for (int i=0;i<47;i++){
          inputString[i] = '\0';
        }
        stringComplete = false;
      }
}
