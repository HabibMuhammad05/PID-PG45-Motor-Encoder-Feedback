/*---------------------------------------------------------------------------------------------------------*/
/*--------------------ARDUINO MEGA PG45 MOTOR WITH PID CONTROLLER CLOSED LOOP FEEDBACK---------------------*/
/*-------------------------------DUAL DIRECTION CALCULATION CAPABILITIES-----------------------------------*/
/*------------------------7PPR ENCODER @1:19.2 GEAR RATIO -- BTS7960 MOTOR DRIVER--------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-24 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-24 TEAM---------------------------------*/
/*----------------------------------------------------V3.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 17:25:00, 14 JAN 25-----------------------------------*/

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
#define MOTOR_A 49
#define MOTOR_B 47
#define MOTOR_PWM 6


//========================================= ENCODER PINS =========================================//
#define ENCA 18
#define ENCB 22


//================================== PID CALCULATION VARIABLES ===================================//
float targetRPM = 100.0;              //Target RPM as reference for PID
const int encoderPPR = 326;           //PG45 = 17PPR @1:19.2 Ratio = 17x19.2 ~ 326
const unsigned long sampleTime = 50;  //PID Calculation Interval

volatile long encoderCount = 0;       //Encoder pulse reading
float currentRPM = 0.0;               //Encoder RPM Reading
float error = 0.0;                    //PID - Proportional
float lastError = 0.0;                //PID - Derivative
float integral = 0.0;                 //PID - Integral
float Kp = 0.45, Ki = 0.8, Kd = 0.01; //PID tuning Times Factor
int motorPWM = 0;                     //Amount of PWM sent to driver
unsigned long lastTime = 0;           //store millis for calculation Interval

unsigned long waktuNaik=0;                          //store millis for speed change
float target[5] = {50.0, 100.0, 30.0, 80.0, 150.0}; //speed value to be calculated
int ind = 0;                                        //speed array index


void setup() {
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  DEBUG_BEGIN(115200);
}

void loop() {
  if(millis() - waktuNaik >= 5000){
    ind++;
    if(ind > 4){
      ind = 0;
    }
    DEBUG_PRINTLN(ind);
    waktuNaik=millis();
  }
  targetRPM = target[ind];
  drivePID();
}


//===================================PID Feedback Calculation=====================================//
void drivePID() {
  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();

    currentRPM = (encoderCount / (float)encoderPPR) * (60000.0 / sampleTime);
    encoderCount = 0;

    error = targetRPM - currentRPM;
    integral += error * (sampleTime / 1000.0);
    float derivative = (error - lastError) / (sampleTime / 1000.0);
    lastError = error;

    motorPWM = Kp * error + Ki * integral + Kd * derivative;

    DEBUG_PRINT("Target: ");
    DEBUG_PRINT(targetRPM);
    DEBUG_PRINT(" | Cur: ");
    DEBUG_PRINT(currentRPM);
    DEBUG_PRINT(" | PWM: ");
    DEBUG_PRINT(motorPWM);
    DEBUG_PRINT(" | DIR: ");
    
    if (motorPWM > 0) {
      motorPWM = constrain(motorPWM, 0, 50);
      analogWrite(MOTOR_PWM, motorPWM);
      digitalWrite(MOTOR_A, HIGH); 
      digitalWrite(MOTOR_B, LOW);
      DEBUG_PRINTLN("CW");
    } else {
      motorPWM = constrain(abs(motorPWM), 0, 50);
      analogWrite(MOTOR_PWM, motorPWM);
      digitalWrite(MOTOR_A, LOW);
      digitalWrite(MOTOR_B, HIGH);
      DEBUG_PRINTLN("CCW");
    }
  }
}


void readEncoder() {
  if (digitalRead(ENCB)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}
