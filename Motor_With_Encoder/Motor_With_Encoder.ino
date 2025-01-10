#define ENCA 2 // Yellow
#define ENCB 24 // White
int pos; 

//idc 1 connector, FRONT LEFT
uint8_t FL_A = 49;
uint8_t FL_B = 47;
uint8_t pwm1 = 6;
uint8_t pwm2 = 7;

////idc 2 connector, FRONT RIGHT
//uint8_t FR_A = 43;
//uint8_t FR_B = 45;
//uint8_t pwm2 = 5;
//
////idc 3 connector, REAR LEFT
//uint8_t RL_A = 41;
//uint8_t RL_B = 39;
//uint8_t pwm3 = 4;
//
////idc 4 connector, REAR RIGHT
//uint8_t RR_A = 35;
//uint8_t RR_B = 37;
//uint8_t pwm4 = 5;
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  
  pinMode(pwm1, OUTPUT);
  pinMode(FL_A, OUTPUT);
  pinMode(FL_B, OUTPUT);
  pinMode(pwm2, OUTPUT);
//  pinMode(FR_A, OUTPUT);
//  pinMode(FR_B, OUTPUT);
//  pinMode(pwm3, OUTPUT);
//  pinMode(RL_A, OUTPUT);
//  pinMode(RL_B, OUTPUT);
//  pinMode(pwm4, OUTPUT);
//  pinMode(RR_A, OUTPUT);
//  pinMode(RR_B, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

}

void loop() {
//  int a = digitalRead(ENCA);
//  int b = digitalRead(ENCB);
//  Serial.print(a*5); 
//  Serial.print(" ");
//  Serial.print(b*5);
//  Serial.println();
  if(pos < 800){
    
    analogWrite(pwm1, 20);
    analogWrite(pwm2, 20);
    digitalWrite(FL_A, HIGH);
    digitalWrite(FL_B, LOW);
  }else{
    analogWrite(pwm1, 70);
    analogWrite(pwm2, 70);
    digitalWrite(FL_A, LOW);
    digitalWrite(FL_B, LOW);
  }

  Serial.println(pos);
  delay(15);
}


void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos--;
  }
  else{
    pos++;
  }
}
