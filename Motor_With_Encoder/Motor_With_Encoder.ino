/*---------------------------------------------------------------------------------------------------------*/
/*-----------------------ARDUINO MEGA PG45 MOTOR WITH INTERNAL ENCODER AS FEEDBACK-------------------------*/
/*------------------------7PPR ENCODER @1:19.2 GEAR RATIO -- BTS7960 MOTOR DRIVER--------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-24 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-24 TEAM---------------------------------*/
/*----------------------------------------------------V1.1-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 11:50:00, 27 MAY 25-----------------------------------*/

// Define DEBUG to enable debugging; comment it out to disable
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINT(...)    
  #define DEBUG_PRINTLN(...)  
  #define DEBUG_BEGIN(baud)   
#endif


//========================================= ENCODER PINS =========================================//
uint8_t ENCA_FL = 20;
uint8_t ENCB_FL = 22;
uint8_t ENCA_FR = 21;
uint8_t ENCB_FR = 24;
uint8_t ENCA_RL = 2;
uint8_t ENCB_RL = 26;
uint8_t ENCA_RR = 3;
uint8_t ENCB_RR = 28;

int encoderCount[4] = {0};

//======================================= MOTOR DRIVER PINS =======================================//
//idc 1 connector, FRONT LEFT
uint8_t FL_A = 49;
uint8_t FL_B = 47;
uint8_t pwm1 = 4;

//idc 2 connector, FRONT RIGHT
uint8_t FR_A = 43;
uint8_t FR_B = 45;
uint8_t pwm2 = 5;

//idc 3 connector, REAR LEFT
uint8_t RL_A = 41;
uint8_t RL_B = 39;
uint8_t pwm3 = 6;

//idc 4 connector, REAR RIGHT
uint8_t RR_A = 35;
uint8_t RR_B = 37;
uint8_t pwm4 = 7;

void setup() {
  DEBUG_BEGIN(115200);
  
  pinMode(ENCA_FL, INPUT_PULLUP);
  pinMode(ENCB_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoder_FL, RISING);
  pinMode(ENCA_FR, INPUT_PULLUP);
  pinMode(ENCB_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoder_FR, RISING);
  pinMode(ENCA_RL, INPUT_PULLUP);
  pinMode(ENCB_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_RL), readEncoder_RL, RISING);
  pinMode(ENCA_RR, INPUT_PULLUP);
  pinMode(ENCB_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoder_RR, RISING); 
  
  pinMode(pwm1, OUTPUT);
  pinMode(FL_A, OUTPUT);
  pinMode(FL_B, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(FR_A, OUTPUT);
  pinMode(FR_B, OUTPUT);  
  pinMode(pwm3, OUTPUT);
  pinMode(RL_A, OUTPUT);
  pinMode(RL_B, OUTPUT);  
  pinMode(pwm4, OUTPUT);
  pinMode(RR_A, OUTPUT);
  pinMode(RR_B, OUTPUT);
}

void loop() {
//  bool a = digitalRead(ENCA);
//  bool b = digitalRead(ENCB);
//  DEBUG_PRINT(a); 
//  DEBUG_PRINT(" ");
//  DEBUG_PRINT(b);
//  DEBUG_PRINTLN();

//  if(encoderCount[3] < 800){
//    
//    analogWrite(pwm4, 30);
//    digitalWrite(RR_A, HIGH);
//    digitalWrite(RR_B, LOW);
//  }else{
//    analogWrite(pwm4, 0);
//  }

    analogWrite(pwm1, 30);
    analogWrite(pwm2, 30);
    analogWrite(pwm3, 30);
    analogWrite(pwm4, 30);
    
    digitalWrite(FL_B, HIGH);
    digitalWrite(FL_A, LOW);
//    digitalWrite(FR_B, HIGH);
//    digitalWrite(FR_A, LOW);
//    digitalWrite(RL_B, HIGH);
//    digitalWrite(RL_A, LOW);
//    digitalWrite(RR_B, HIGH);
//    digitalWrite(RR_A, LOW);

  DEBUG_PRINT(encoderCount[0]);  DEBUG_PRINT(" | ");
  DEBUG_PRINT(encoderCount[1]);  DEBUG_PRINT(" | ");
  DEBUG_PRINT(encoderCount[2]);  DEBUG_PRINT(" | ");
  DEBUG_PRINTLN(encoderCount[3]);  
  delay(15);
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
