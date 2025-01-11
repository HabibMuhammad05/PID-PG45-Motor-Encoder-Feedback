#define MOTOR_A 49
#define MOTOR_B 47
#define MOTOR_PWM 6

#define ENCA 18
#define ENCB 22

float targetRPM = 100.0; 
const int encoderPPR = 326;  
const unsigned long sampleTime = 50; 

volatile long encoderCount = 0; 
float currentRPM = 0.0;         
float error = 0.0;              
float lastError = 0.0;          
float integral = 0.0;       
float Kp = 0.45, Ki = 0.8, Kd = 0.01; 
int motorPWM = 0;             
unsigned long lastTime = 0;

unsigned long waktuNaik = 0;
float target[6] = {50.0, 100.0, 10.0, -80.0, -150.0, -10}; 
int ind = 0;

void readEncoder() {
  if (digitalRead(ENCB)) encoderCount--;
  else encoderCount++;
}

void setup() {
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  Serial.begin(115200);
}

void loop() {
  if (millis() - waktuNaik >= 5000) {
    ind++;
    if (ind > 5) ind = 0;
    Serial.println(ind);
    waktuNaik = millis();
  }
  targetRPM = target[ind];
  drivePID();
}

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

    Serial.print("Target: ");
    Serial.print(targetRPM);
    Serial.print(" | Cur: ");
    Serial.print(currentRPM);
    Serial.print(" | PWM: ");
    Serial.print(motorPWM);
    Serial.print(" | DIR: ");
    
    if (motorPWM > 0) {
      motorPWM = constrain(motorPWM, 0, 50);
      analogWrite(MOTOR_PWM, motorPWM);
      digitalWrite(MOTOR_A, HIGH); 
      digitalWrite(MOTOR_B, LOW);
      Serial.println("CW");
    } else {
      motorPWM = constrain(abs(motorPWM), 0, 50);
      analogWrite(MOTOR_PWM, motorPWM);
      digitalWrite(MOTOR_A, LOW);
      digitalWrite(MOTOR_B, HIGH);
      Serial.println("CCW");
    }
  }
}
