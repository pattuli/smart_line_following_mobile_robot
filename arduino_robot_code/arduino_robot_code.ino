// ------------ PIN DEFINITIONS ------------

// IR Sensors (0=White/No Line, 1 = Black Line)
#define IR_LEFT 4
#define IR_RIGHT 13

// Ultrasonic
#define TRIG_PIN 11
#define ECHO_PIN 12

// Motor Driver (L293D)
#define PWMA 5
#define AIN1 7
#define AIN2 8
#define PWMB 6
#define BIN1 9
#define BIN2 10

// Encoders
#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3

// ------------ PID CONSTANTS ------------
//Refined from your MATLAB/Simulink values
double Kp = 0.6256; 
double Ki = 25.02; 
double Kd = 0;

// ------------ GLOBAL VARIABLES ------------
//volatile is required for variables modified in ISRs
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

double leftPWM = 0, rightPWM = 0; // Analog Inputs for two motors
double leftError = 0, leftLastError = 0, leftIntegral = 0;
double rightError = 0, rightLastError = 0, rightIntegral = 0;

unsigned long lastPIDTime = 0; // To check sample time
const int sampleTime = 50; // Calculate PID every 50ms

// Targets
int baseSpeed = 25; // Target speed (ticks per sample period)
int minPWM =150; //Minimum power to overcome motor friction

// ------------ INTERRUPT SERVICE ROUTINES ------------
void leftEncoderISR() { leftEncoderCount++; }
void rightEncoderISR() { rightEncoderCount++; }

// ------------ SETUP ------------
void setup() {
  Serial.begin(9600);

  // Sensor Pins
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor Pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Encoder Pins with Internal Pullup
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);

  // Attach Interrupts for precise wheel tracking
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);

  lastPIDTime = millis();
}

// ------------ MAIN LOOP ------------
void loop() {
  // 1. Safety Check
  if (checkObstacle()) {
    stopMotors();
    return; //Skip the rest of the loop
  }

  // 2. Line Following Logic (Determines Target Speeds)
  int leftSens = digitalRead(IR_LEFT);
  int rightSens = digitalRead(IR_RIGHT);

  int targetL = 0;
  int targetR = 0;

  // Logic: Adjust target speeds based on sensor input
  if (rightSens == 0 && leftSens == 0) { // On Track

    targetL = baseSpeed;
    targetR = baseSpeed;
  } 
  else if (rightSens == 1 && leftSens == 0) { // Turn Right
    targetL = baseSpeed;
    targetR = -baseSpeed;
  } 
  else if (rightSens == 0 && leftSens == 1) { // Turn Left
    targetL = -baseSpeed;
    targetR = baseSpeed;
  } 
  else { // Both black / Stop
    targetL = 0;
    targetR = 0;
  }

  // 3. PID Computation
  if (millis() - lastPIDTime >= sampleTime) {
    computePID(targetL, targetR);
    lastPIDTime = millis();
  }

  // 4. Apply calculated PWM to motors
  applyMotorSignals(targetL, targetR);
}

// ------------ PID CALCULATION ------------
void computePID(int targetL, int targetR) {

  // ATOMIC READ: Temporarily diable interrupts to copy encoder values safely
  noInterrupts();
  long currentL = leftEncoderCount;
  long currentR = rightEncoderCount;
  
  // Reset counts for next sample
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();

  // --- Left Motor PID ---
  leftError = abs(targetL) - currentL;
  leftIntegral += leftError;
  leftIntegral = constrain(leftIntegral, -100, 100); // Prevent Integral Windup
  double leftDeriv = leftError - leftLastError;

  // Update PWM (Velocity Form)
  leftPWM += (Kp * leftError) + (Ki * leftIntegral) + (Kd * leftDeriv);
  leftLastError = leftError;

  // --- Right Motor PID ---
  rightError = abs(targetR) - currentR;
  rightIntegral += rightError;
  rightIntegral = constrain(leftIntegral, -100, 100); //Prevent Integral Windup
  double rightDeriv = rightError - rightLastError;

  //Update PWM (Velocity Form)
  rightPWM += (Kp * rightError) + (Ki * rightIntegral) + (Kd * rightDeriv);
  rightLastError = rightError;

  //Handle Left Motor minPWM
  if(targetL!=0){
    leftPWM =constrain(leftPWM, minPWM, 255);
  }else{
    leftPWM = 0;
    leftIntegral = 0; //Clear integral when stopped to prevent jump-start
  }

  //Handle Right Motor minPWM
  if(targetR!=0){
    rightPWM = constrain(rightPWM, minPWM, 255);
  }else{
    rightPWM= 0;
    rightIntegral =0;
  }
  
  // If target is 0, reset PID
  if(targetL == 0) { leftPWM = 0; leftIntegral = 0; }
  if(targetR == 0) { rightPWM = 0; rightIntegral = 0; }
}


// ------------ MOTOR OUTPUT ------------
void applyMotorSignals(int targetL, int targetR) {
  // Left Motor Direction
  if (targetL > 0) { 
    digitalWrite(AIN1, HIGH); 
    digitalWrite(AIN2, LOW); 
    }
  else if (targetL < 0) { 
    digitalWrite(AIN1, LOW); 
    digitalWrite(AIN2, HIGH); 
    }
  else { 
    digitalWrite(AIN1, LOW); 
    digitalWrite(AIN2, LOW); 
    }

  // Right Motor Direction
  if (targetR > 0) { 
    digitalWrite(BIN1, HIGH); 
    digitalWrite(BIN2, LOW); 
    }
  else if (targetR < 0) {
     digitalWrite(BIN1, LOW); 
     digitalWrite(BIN2, HIGH);
      }
  else { 
    digitalWrite(BIN1, LOW);
   digitalWrite(BIN2, LOW); 
   }

  analogWrite(PWMA, leftPWM);
  analogWrite(PWMB, rightPWM);
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  leftIntegral = 0;
  rightIntegral = 0;
}

// ------------ ULTRASONIC ------------
bool checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  float distance = duration * 0.034 / 2;
  return (distance > 0 && distance < 15);
}