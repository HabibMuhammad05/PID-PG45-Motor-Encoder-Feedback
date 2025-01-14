/*---------------------------------------------------------------------------------------------------------*/
/*--------------------ARDUINO MEGA PG45 MOTOR WITH PID CONTROLLER CLOSED LOOP FEEDBACK---------------------*/
/*----------------------DUAL DIRECTION WITH MULTIPLE MOTOR CALCULATION CAPABILITIES------------------------*/
/*------------------------7PPR ENCODER @1:19.2 GEAR RATIO -- BTS7960 MOTOR DRIVER--------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-24 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-24 TEAM---------------------------------*/
/*----------------------------------------------------V4.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 17:30:00, 14 JAN 25-----------------------------------*/

// Define DEBUG to enable debugging; comment it out to disable
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINT(...)    
  #define DEBUG_PRINTLN(...)  
  #define DEBUG_BEGIN(baud)   
#endif


//======================================= MOTOR DRIVER PINS =======================================//
#define MOTOR_A_FL 49
#define MOTOR_B_FL 47
#define MOTOR_PWM_FL 8

#define MOTOR_A_FR 43
#define MOTOR_B_FR 45
#define MOTOR_PWM_FR 9

#define MOTOR_A_RL 41
#define MOTOR_B_RL 39
#define MOTOR_PWM_RL 4

#define MOTOR_A_RR 35
#define MOTOR_B_RR 37
#define MOTOR_PWM_RR 5


//========================================= ENCODER PINS =========================================//
#define ENCA_FL 21
#define ENCB_FL 28
#define ENCA_FR 20
#define ENCB_FR 26
#define ENCA_RL 3
#define ENCB_RL 24
#define ENCA_RR 2
#define ENCB_RR 22

//================================== PID CALCULATION VARIABLES ===================================//
uint8_t maxPWM = 200;                         //amount of max PWM sent to motor driver
    
const int encoderPPR = 326;                   //PG45 = 17PPR @1:19.2 Ratio = 17x19.2 ~ 326
const unsigned long sampleTime = 25;          //PID Calculation Interval
const unsigned long debugInterval = 1000;     //Serial Print Debug interval, avoid excessive RAM&Overflow

volatile long encoderCount[4] = {0, 0, 0, 0}; //num of Encoder
float currentRPM[4] = {0.0, 0.0, 0.0, 0.0};   //Encoder RPM Reading
float targetRPM[4] = {0.0, 0.0, 0.0, 0.0};    //Target RPM as reference for PID
float error[4] = {0.0, 0.0, 0.0, 0.0};        //PID - Proportional
float lastError[4] = {0.0, 0.0, 0.0, 0.0};    //PID - Derivative
float integral[4] = {0.0, 0.0, 0.0, 0.0};     //PID - Integral
float Kp = 0.45, Ki = 0.8, Kd = 0.01;         //PID Times Factor
int motorPWM[4] = {0, 0, 0, 0};               //Amount of PWM sent to driver
unsigned long lastTime[4] = {0, 0, 0, 0};     //store millis for calculation Interval
unsigned long lastDebugTime = 0;              //store millis for Debug Interval


void setup() {
  pinMode(MOTOR_A_FL, OUTPUT);
  pinMode(MOTOR_B_FL, OUTPUT);
  pinMode(MOTOR_PWM_FL, OUTPUT);

  pinMode(ENCA_FL, INPUT_PULLUP);
  pinMode(ENCB_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoder_FL, RISING);

  pinMode(MOTOR_A_FR, OUTPUT);
  pinMode(MOTOR_B_FR, OUTPUT);
  pinMode(MOTOR_PWM_FR, OUTPUT);

  pinMode(ENCA_FR, INPUT_PULLUP);
  pinMode(ENCB_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoder_FR, RISING);

  pinMode(MOTOR_A_RL, OUTPUT);
  pinMode(MOTOR_B_RL, OUTPUT);
  pinMode(MOTOR_PWM_RL, OUTPUT);

  pinMode(ENCA_RL, INPUT_PULLUP);
  pinMode(ENCB_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_RL), readEncoder_RL, RISING);

  pinMode(MOTOR_A_RR, OUTPUT);
  pinMode(MOTOR_B_RR, OUTPUT);
  pinMode(MOTOR_PWM_RR, OUTPUT);

  pinMode(ENCA_RR, INPUT_PULLUP);
  pinMode(ENCB_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoder_RR, RISING);

  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < 4; i++) {
    drivePID(i);
  }

  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    for (int i = 0; i < 4; i++) {
      Serial.print("M ");
      Serial.print(i);
      Serial.print(" | Target: ");
      Serial.print(targetRPM[i]);
      Serial.print(" | Cur: ");
      Serial.print(currentRPM[i]);
      Serial.print(" | Err: ");
      Serial.print(error[i]);
      Serial.print(" | PWM: ");
      Serial.println(motorPWM[i]);
    }
    Serial.println("------------------------");
  }
}


//===================================PID Feedback Calculation=====================================//
void drivePID(int motorIndex) {
  const int motorA[4] = {MOTOR_A_FL, MOTOR_A_FR, MOTOR_A_RL, MOTOR_A_RR};
  const int motorB[4] = {MOTOR_B_FL, MOTOR_B_FR, MOTOR_B_RL, MOTOR_B_RR};
  const int motorPWM_Pin[4] = {MOTOR_PWM_FL, MOTOR_PWM_FR, MOTOR_PWM_RL, MOTOR_PWM_RR};

  unsigned long startTime = micros();

  if (millis() - lastTime[motorIndex] >= sampleTime) {
    lastTime[motorIndex] = millis();

    currentRPM[motorIndex] = (encoderCount[motorIndex] / (float)encoderPPR) * (60000.0 / sampleTime);
    encoderCount[motorIndex] = 0;

    error[motorIndex] = targetRPM[motorIndex] - currentRPM[motorIndex];
    integral[motorIndex] += error[motorIndex] * (sampleTime / 1000.0);
    float derivative = (error[motorIndex] - lastError[motorIndex]) / (sampleTime / 1000.0);
    lastError[motorIndex] = error[motorIndex];

    motorPWM[motorIndex] = Kp * error[motorIndex] + Ki * integral[motorIndex] + Kd * derivative;

    if (motorPWM[motorIndex] > 0) {
      motorPWM[motorIndex] = constrain(motorPWM[motorIndex], 0, maxPWM);
      analogWrite(motorPWM_Pin[motorIndex], motorPWM[motorIndex]);
      digitalWrite(motorA[motorIndex], HIGH);
      digitalWrite(motorB[motorIndex], LOW);
    } else {
      motorPWM[motorIndex] = constrain(abs(motorPWM[motorIndex]), 0, maxPWM);
      analogWrite(motorPWM_Pin[motorIndex], motorPWM[motorIndex]);
      digitalWrite(motorA[motorIndex], LOW);
      digitalWrite(motorB[motorIndex], HIGH);
    }
  }

  
//==================================== ENCODER ISR FUNCTIONS =====================================//
void readEncoder_FL() {
  if (digitalRead(ENCB_FL)) encoderCount[0]++;
  else encoderCount[0]--;
}

void readEncoder_FR() {
  if (digitalRead(ENCB_FR)) encoderCount[1]--;
  else encoderCount[1]++;
}

void readEncoder_RL() {
  if (digitalRead(ENCB_RL)) encoderCount[2]++;
  else encoderCount[2]--;
}

void readEncoder_RR() {
  if (digitalRead(ENCB_RR)) encoderCount[3]--;
  else encoderCount[3]++;
}


//  unsigned long computationTime = micros() - startTime;
//  Serial.print("Motor ");
//  Serial.print(motorIndex);
//  Serial.print(" PID computation time: ");
//  Serial.print(computationTime);
//  Serial.println(" us");
}



//#define MOTOR_A_FL 49
//#define MOTOR_B_FL 47
//#define MOTOR_PWM_FL 6
//
//#define MOTOR_A_FR 43
//#define MOTOR_B_FR 45
//#define MOTOR_PWM_FR 7
//
//#define MOTOR_A_RL 41
//#define MOTOR_B_RL 39
//#define MOTOR_PWM_RL 4
//
//#define MOTOR_A_RR 35
//#define MOTOR_B_RR 37
//#define MOTOR_PWM_RR 5
//
//#define ENCA_FL 20
//#define ENCB_FL 22
//#define ENCA_FR 21
//#define ENCB_FR 24
//#define ENCA_RL 2
//#define ENCB_RL 26
//#define ENCA_RR 3
//#define ENCB_RR 28
//
//float targetRPM_FL = 100.0, targetRPM_FR = 100.0, targetRPM_RL = 100.0, targetRPM_RR = 100.0;
//const int encoderPPR = 326;
//const unsigned long sampleTime = 50;
//
//volatile long encoderCount_FL = 0, encoderCount_FR = 0, encoderCount_RL = 0, encoderCount_RR = 0;
//float currentRPM_FL = 0.0, currentRPM_FR = 0.0, currentRPM_RL = 0.0, currentRPM_RR = 0.0;
//float error_FL = 0.0, error_FR = 0.0, error_RL = 0.0, error_RR = 0.0;
//float lastError_FL = 0.0, lastError_FR = 0.0, lastError_RL = 0.0, lastError_RR = 0.0;
//float integral_FL = 0.0, integral_FR = 0.0, integral_RL = 0.0, integral_RR = 0.0;
//float Kp = 0.45, Ki = 0.8, Kd = 0.01;
//int motorPWM_FL = 0, motorPWM_FR = 0, motorPWM_RL = 0, motorPWM_RR = 0;
//unsigned long lastTime_FL = 0, lastTime_FR = 0, lastTime_RL = 0, lastTime_RR = 0;
//
//void readEncoder_FL() {
//  if (digitalRead(ENCB_FL)) encoderCount_FL--;
//  else encoderCount_FL++;
//}
//
//void readEncoder_FR() {
//  if (digitalRead(ENCB_FR)) encoderCount_FR--;
//  else encoderCount_FR++;
//}
//
//void readEncoder_RL() {
//  if (digitalRead(ENCB_RL)) encoderCount_RL--;
//  else encoderCount_RL++;
//}
//
//void readEncoder_RR() {
//  if (digitalRead(ENCB_RR)) encoderCount_RR--;
//  else encoderCount_RR++;
//}
//
//void setup() {
//  pinMode(MOTOR_A_FL, OUTPUT);
//  pinMode(MOTOR_B_FL, OUTPUT);
//  pinMode(MOTOR_PWM_FL, OUTPUT);
//
//  pinMode(ENCA_FL, INPUT_PULLUP);
//  pinMode(ENCB_FL, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoder_FL, RISING);
//
//  pinMode(MOTOR_A_FR, OUTPUT);
//  pinMode(MOTOR_B_FR, OUTPUT);
//  pinMode(MOTOR_PWM_FR, OUTPUT);
//
//  pinMode(ENCA_FR, INPUT_PULLUP);
//  pinMode(ENCB_FR, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoder_FR, RISING);
//
//  pinMode(MOTOR_A_RL, OUTPUT);
//  pinMode(MOTOR_B_RL, OUTPUT);
//  pinMode(MOTOR_PWM_RL, OUTPUT);
//
//  pinMode(ENCA_RL, INPUT_PULLUP);
//  pinMode(ENCB_RL, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(ENCA_RL), readEncoder_RL, RISING);
//
//  pinMode(MOTOR_A_RR, OUTPUT);
//  pinMode(MOTOR_B_RR, OUTPUT);
//  pinMode(MOTOR_PWM_RR, OUTPUT);
//
//  pinMode(ENCA_RR, INPUT_PULLUP);
//  pinMode(ENCB_RR, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoder_RR, RISING);
//
//  Serial.begin(115200);
//}
//
//void loop() {
//  drivePID_FL();
//  drivePID_FR();
//  drivePID_RL();
//  drivePID_RR();
//}
//
//void drivePID_FL() {
//  if (millis() - lastTime_FL >= sampleTime) {
//    lastTime_FL = millis();
//
//    currentRPM_FL = (encoderCount_FL / (float)encoderPPR) * (60000.0 / sampleTime);
//    encoderCount_FL = 0;
//
//    error_FL = targetRPM_FL - currentRPM_FL;
//    integral_FL += error_FL * (sampleTime / 1000.0);
//    float derivative_FL = (error_FL - lastError_FL) / (sampleTime / 1000.0);
//    lastError_FL = error_FL;
//
//    motorPWM_FL = Kp * error_FL + Ki * integral_FL + Kd * derivative_FL;
//
//    if (motorPWM_FL > 0) {
//      motorPWM_FL = constrain(motorPWM_FL, 0, 255);
//      analogWrite(MOTOR_PWM_FL, motorPWM_FL);
//      digitalWrite(MOTOR_A_FL, HIGH);
//      digitalWrite(MOTOR_B_FL, LOW);
//    } else {
//      motorPWM_FL = constrain(abs(motorPWM_FL), 0, 255);
//      analogWrite(MOTOR_PWM_FL, motorPWM_FL);
//      digitalWrite(MOTOR_A_FL, LOW);
//      digitalWrite(MOTOR_B_FL, HIGH);
//    }
//  }
//}
//
//void drivePID_FR() {
//  if (millis() - lastTime_FR >= sampleTime) {
//    lastTime_FR = millis();
//
//    currentRPM_FR = (encoderCount_FR / (float)encoderPPR) * (60000.0 / sampleTime);
//    encoderCount_FR = 0;
//
//    error_FR = targetRPM_FR - currentRPM_FR;
//    integral_FR += error_FR * (sampleTime / 1000.0);
//    float derivative_FR = (error_FR - lastError_FR) / (sampleTime / 1000.0);
//    lastError_FR = error_FR;
//
//    motorPWM_FR = Kp * error_FR + Ki * integral_FR + Kd * derivative_FR;
//
//    if (motorPWM_FR > 0) {
//      motorPWM_FR = constrain(motorPWM_FR, 0, 255);
//      analogWrite(MOTOR_PWM_FR, motorPWM_FR);
//      digitalWrite(MOTOR_A_FR, HIGH);
//      digitalWrite(MOTOR_B_FR, LOW);
//    } else {
//      motorPWM_FR = constrain(abs(motorPWM_FR), 0, 255);
//      analogWrite(MOTOR_PWM_FR, motorPWM_FR);
//      digitalWrite(MOTOR_A_FR, LOW);
//      digitalWrite(MOTOR_B_FR, HIGH);
//    }
//  }
//}
//
//void drivePID_RL() {
//  if (millis() - lastTime_RL >= sampleTime) {
//    lastTime_RL = millis();
//
//    currentRPM_RL = (encoderCount_RL / (float)encoderPPR) * (60000.0 / sampleTime);
//    encoderCount_RL = 0;
//
//    error_RL = targetRPM_RL - currentRPM_RL;
//    integral_RL += error_RL * (sampleTime / 1000.0);
//    float derivative_RL = (error_RL - lastError_RL) / (sampleTime / 1000.0);
//    lastError_RL = error_RL;
//
//    motorPWM_RL = Kp * error_RL + Ki * integral_RL + Kd * derivative_RL;
//
//    if (motorPWM_RL > 0) {
//      motorPWM_RL = constrain(motorPWM_RL, 0, 255);
//      analogWrite(MOTOR_PWM_RL, motorPWM_RL);
//      digitalWrite(MOTOR_A_RL, HIGH);
//      digitalWrite(MOTOR_B_RL, LOW);
//    } else {
//      motorPWM_RL = constrain(abs(motorPWM_RL), 0, 255);
//      analogWrite(MOTOR_PWM_RL, motorPWM_RL);
//      digitalWrite(MOTOR_A_RL, LOW);
//      digitalWrite(MOTOR_B_RL, HIGH);
//    }
//  }
//}
//
//void drivePID_RR() {
//  if (millis() - lastTime_RR >= sampleTime) {
//    lastTime_RR = millis();
//
//    currentRPM_RR = (encoderCount_RR / (float)encoderPPR) * (60000.0 / sampleTime);
//    encoderCount_RR = 0;
//
//    error_RR = targetRPM_RR - currentRPM_RR;
//    integral_RR += error_RR * (sampleTime / 1000.0);
//    float derivative_RR = (error_RR - lastError_RR) / (sampleTime / 1000.0);
//    lastError_RR = error_RR;
//
//    motorPWM_RR = Kp * error_RR + Ki * integral_RR + Kd * derivative_RR;
//
//    if (motorPWM_RR > 0) {
//      motorPWM_RR = constrain(motorPWM_RR, 0, 255);
//      analogWrite(MOTOR_PWM_RR, motorPWM_RR);
//      digitalWrite(MOTOR_A_RR, HIGH);
//      digitalWrite(MOTOR_B_RR, LOW);
//    } else {
//      motorPWM_RR = constrain(abs(motorPWM_RR), 0, 255);
//      analogWrite(MOTOR_PWM_RR, motorPWM_RR);
//      digitalWrite(MOTOR_A_RR, LOW);
//      digitalWrite(MOTOR_B_RR, HIGH);
//    }
//  }
//}
