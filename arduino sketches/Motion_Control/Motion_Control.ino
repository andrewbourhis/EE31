#include "TimerOne.h"

//motorA = right motor
//motorB = left motor

int pinA0 = 4, pinA1 = 5, pinB0 = 6, pinB1 = 7;
int pinChangeDirA = 2, pinChangeDirB = 3;
int pinMotorASpeed = A0, pinMotorBSpeed = A1;
int MotorASpeed = 100, MotorBSpeed = 100;
int serialPacket = 0;
float calibrateA = 1.3,  calibrateB = 1;
bool motorAFWD = 0;
bool motorBFWD = 0;
long lastDebounceTime = 0;
bool motorAStopped = 0, motorBStopped = 0;
bool stopMotors = 0, goFWD = 0, goBCK = 0, turnL = 0, turnR = 0, turn180 = 0;


void setup() {
  //init pins
  Serial.begin(9600);
  while(!Serial);
  Timer1.initialize(50000);
  Timer1.attachInterrupt(timer1_ISR);
  MotorASpeed = 100*calibrateA;
  MotorBSpeed = 100*calibrateB;
  pinMode(pinA1, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinA0, OUTPUT);
  pinMode(pinB0, OUTPUT);
  
}

void timer1_ISR(){
  if(Serial.available())
  {
    serialPacket = (char)Serial.read();
    //packet is received in ascii format
    switch(serialPacket){
      case 48://forward
        goFWD = 1;
        break;
      case 49://backwards
        goBCK = 1;
        break;
      case 50://left
        turnL = 1;
        break;
      case 51://right
        turnR = 1;
        break;
      case 52://180 degree turn
        turn180 = 1;
        break;
      case 53://stop motors
        stopMotors = 1;
        break;    
    }
  }
}

void changeDirA(){
    if(motorAFWD){
      digitalWrite(pinA1, LOW);
      delay(10);
      analogWrite(pinA0, MotorASpeed);
      motorAFWD = 0;
    }
    else
    {
      digitalWrite(pinA0, LOW);
      delay(10);
      analogWrite(pinA1, MotorASpeed); 
      motorAFWD = 1;     
    }
}

void changeDirB(){
    if(motorBFWD){
      digitalWrite(pinB1, LOW);
      delay(10);
      analogWrite(pinB0, MotorBSpeed);
      motorBFWD = 0;
    }
    else
    {
      digitalWrite(pinB0, LOW);
      delay(10);
      analogWrite(pinB1, MotorBSpeed); 
      motorBFWD = 1;     
    }
}


void stopMotorA()
{
  digitalWrite(pinA0, LOW);
  digitalWrite(pinA1, LOW);
  motorAStopped = 1;
}


void stopMotorB()
{
  digitalWrite(pinB0, LOW);
  digitalWrite(pinB1, LOW);
  motorBStopped = 1;
}

void moveforward(){
  if(!motorAFWD)
    changeDirA();
  else
    {
      digitalWrite(pinA0, LOW);
      delay(10);
      analogWrite(pinA1, MotorASpeed);
    }
  if(!motorBFWD)
    changeDirB();
  else
  {
      digitalWrite(pinB0, LOW);
      delay(10);
      analogWrite(pinB1, MotorBSpeed); 
  }
}

void movebackwards(){
  if(motorAFWD)
    changeDirA();
  else
    {
      digitalWrite(pinA1, LOW);
      delay(10);
      analogWrite(pinA0, MotorASpeed);
    }
  if(motorBFWD)
    changeDirB();
  else
  {
      digitalWrite(pinB1, LOW);
      delay(10);
      analogWrite(pinB0, MotorBSpeed); 
  }
}

//turn left 90 deg
void turnLeft(){
  moveforward();//move forwards, and turn off motorB
  changeDirB(); //pivot
  delay(900);
  stopMotors = 1;
}

//turn right 90 deg
void turnRight(){
  moveforward();
  changeDirA();
  delay(900);
  stopMotors = 1;
}
/*
      case 0://forward
      case 1://backwards
      case 2://left
      case 3://right
      case 4://180 degree turn
      case 5://stop
 */
void loop() {
  
  
  if(stopMotors){
    stopMotorA();
    stopMotorB();
    stopMotors = 0;
  }
  else if(goFWD)
  {
    moveforward();
    goFWD = 0;
  }
  else if(goBCK)
  {
    movebackwards();
    goBCK = 0;
  }
  else if(turnL)
  {
    turnLeft();
    turnL = 0;
  }
  else if(turnR)
  {
    turnRight();
    turnR = 0;
  }
  else if(turn180)
  {
    turnRight();
    turnRight();
    turn180 = 0;
  }

 
}

