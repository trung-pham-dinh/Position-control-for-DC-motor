#include<PID_v1.h>

#define DIRA1 4 // use for IN3
#define DIRA2 5 // use for IN4
#define ENA 9 // use to control speed
#define ENCODERA 2
#define ENCODERB 3

// These variables are for ISR
volatile int encoderCount = 0; // use in ISR
volatile uint8_t oldState = 0;
int encodeTable[16] = {
  0,  1, -1,  2,
 -1,  0, -2,  1,
  1, -2,  0, -1,
  2, -1,  1,  0
};


int CPC = 8600; // counts per cycle
int maxAngle = 360;

// These variables are for PID
double kp = 2.8, ki = 0, kd = 0.2;
double setPoint = 0, input = 0, output = 0;
int outputPWM = 0;
PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT);


int i = 1;
int dir = 0;
void setup() {
  Serial.begin(9600);
  
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENCODERA, INPUT_PULLUP); 
  pinMode(ENCODERB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODERA), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERB), encoder, CHANGE);

  if(digitalRead(ENCODERA)) oldState |= 1;
  if(digitalRead(ENCODERB)) oldState |= 2;

  setPoint = 180;
  input = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255,255);
}

void loop() {
  input = map(encoderCount, 0, 8600, 0, 360);
  pid.Compute();
  if(output < 0) {
    outputPWM = map(output, -255, 0 , -255, -30);// 30 is minimum PWM that motor can rotate
    setMotor(-1, outputPWM);
  }
  else {
    outputPWM = map(output, 0, 255 , 30, 255);
    setMotor(1, outputPWM);
  }
  Serial.print(input);
  Serial.print(' ');
  Serial.println(outputPWM);
}

void setMotor(int d, int s) {
  if(abs(s) < 36) { // acctually at 35, the motor stop
    analogWrite(ENA, 0);
  }
  else {
    analogWrite(ENA, abs(s));
  }
  if(d == 1) {
    digitalWrite(DIRA1, HIGH);
    digitalWrite(DIRA2, LOW);
    dir = 1;
  }
  else if (d == -1) {
    digitalWrite(DIRA1, LOW);
    digitalWrite(DIRA2, HIGH);
    dir = -1;
  }
  else {
    digitalWrite(DIRA1, LOW);
    digitalWrite(DIRA2, LOW);
    dir = 0;
  }
}

// reference: https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h
//https://www.best-microcontroller-projects.com/rotary-encoder.html
void encoder(void) {  // 8600 counts/cycle
  uint8_t A = digitalRead(ENCODERA);
  uint8_t B = digitalRead(ENCODERB);
  uint8_t thisState = oldState & 3;
  if(A) thisState |= 4; // 0b-1--
  if(B) thisState |= 8; // 0b1---
  
  oldState = (thisState>>2);
  encoderCount += encodeTable[thisState];
}

//void encoder(void) {
//  int8_t A = digitalRead(ENCODERA);
//  int8_t B = digitalRead(ENCODERB);
//
//  int8_t thisState = (A<<1)|B;
//  
//  int8_t thisCode = (thisState<<2) | oldState;
//    
//  if(thisCode != oldCode) {
//    if(dir == 1 && encodeTable[thisCode] == 1) {
//      Serial.print(encoderCount);
//      Serial.print('\n');
//      encoderCount++;
//      oldCode = thisCode;
//    }
//    else if(dir == -1 && encodeTable[thisCode] == -1) {
//      Serial.print(encoderCount);
//      Serial.print('\n');
//      encoderCount--;  
//      oldCode = thisCode;
//    }
//  }
//  oldState = thisState;
//}
