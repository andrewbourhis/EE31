//----------------------
//--01A--State-Machine--
//----------------------
#include "TimerOne.h"

int LEDREDpin = 9;
int LEDBLUEpin = 4;
int LEDGREENpin = 5;
int SW1 = 3, SW2 = 2, SW_on = 6;
int nextState = 2; //state 1 is on, state 2 is off, state 3 is run, state 4 is sleep, state 5 is diagnstic
int blueRate = 10, redRate = 1;//1 hz by default
int bcnt = 0, rcnt = 0;
int curGreenVal = 255;
int lastTimeMilli = 0;
int N = 5;
int pot1 = A0, pot2 = A1;
int blueBrightness = 255;
bool toggleBlue = 0, disableRedFlash = 0;
int redOffCount = 0;
int sw1Or2 = 1; 
void setup() {
  Serial.begin(9600);
  while(!Serial);
  Timer1.initialize(50000);
  pinMode(LEDREDpin, OUTPUT);
  pinMode(LEDBLUEpin, OUTPUT);
  pinMode(LEDGREENpin, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  Timer1.attachInterrupt(timer1_ISR);
  
}

void timer1_ISR()
{
  if(!disableRedFlash)
  {
    Serial.println("test");
    redRate = map(analogRead(pot1), 0, 1024, 0, 10);
    rcnt++;
    if(rcnt>=redRate)
    {
      digitalWrite(LEDREDpin,!digitalRead(LEDREDpin));
      rcnt=0;
    }
  }
  else{
    redOffCount++;
    if(sw1Or2 == 1)
      digitalWrite(LEDREDpin, HIGH);
    if(sw1Or2 == 2)
      digitalWrite(LEDREDpin, LOW);  
  }
  if(redOffCount >= 50)//re-enable flashing
  {
    disableRedFlash = 0;
    redOffCount = 0;
  }
  
  bcnt++;
  if(bcnt==blueRate)
  {
    Serial.println("ISR");
    blueBrightness = map(analogRead(pot2), 0, 1024, 0, 255);
    if(toggleBlue){
      analogWrite(LEDBLUEpin, blueBrightness);
      }
      else{
        digitalWrite(LEDBLUEpin, LOW);
        }
    toggleBlue = !toggleBlue;
    bcnt = 0;
  }
}

void loop() {
  switch(nextState){
    case 1:
      on_state();//start off in this state
    break;
    case 2:
      off_state();
    break;
    case 3:
      run_state();
      run_state();
      delay(4000);
    break;
    case 4:
      sleep_state();
    break;
    case 5:
      diagnostic_state();
    break;
  }
}

void SW1_ISR()
{
  sw1Or2 = 1;
  blueRate = 1;//10 hz
  bcnt = 0;//ensure we aren't outside our range
  disableRedFlash = 1;//show we've changed rate with red ON for a second
  
  attachInterrupt(digitalPinToInterrupt(SW2),SW2_ISR,FALLING);//now we can start looking for sw2
}

void SW2_ISR()
{
  sw1Or2 = 2;
  blueRate = 10;
  disableRedFlash = 1;
  detachInterrupt(digitalPinToInterrupt(SW2));
}

void on_state(){
  Timer1.attachInterrupt(timer1_ISR);
  delay(5000);
  nextState = 3;
}

void off_state(){
  //turn off everything
  Timer1.detachInterrupt();
  detachInterrupt(digitalPinToInterrupt(SW2));
  detachInterrupt(digitalPinToInterrupt(SW1));
  digitalWrite(LEDREDpin, 0);
  digitalWrite(LEDBLUEpin, 0);
  digitalWrite(LEDGREENpin, 0);
  
  //wait for on switch
  while(!digitalRead(SW_on));
  nextState = 1;
}

void run_state(){
  attachInterrupt(digitalPinToInterrupt(SW1),SW1_ISR,RISING);
 
  //decay with time constant of 6 sec. flash twice for duty cycle of 0.5 sec then repeats while in state 
  for(int i = 255; i>0; i--)
  {
    analogWrite(LEDGREENpin, i);
    delay(24);
  }
  
  //begin blinking with 0.5 second period
  digitalWrite(LEDGREENpin, 1);
  delay(500);
  digitalWrite(LEDGREENpin, 0);
  delay(500);
  digitalWrite(LEDGREENpin, 1);
  delay(500);
  digitalWrite(LEDGREENpin, 0);
  delay(500);
  
  nextState = 4; //go to bed
}

void sleep_state(){
  //disable everything
  Timer1.detachInterrupt();
  detachInterrupt(digitalPinToInterrupt(SW2));
  detachInterrupt(digitalPinToInterrupt(SW1));
  digitalWrite(LEDGREENpin, LOW);
  digitalWrite(LEDREDpin, LOW);
  
  for(int i = 0; i<4; i++)
  {
    digitalWrite(LEDBLUEpin, 1);
    delay(125);
    if(i!=3)
    {
      digitalWrite(LEDBLUEpin, 0);
      delay(125);
    }
  }
  for(int i = 255; i>0; i--)
  {
    analogWrite(LEDBLUEpin, i);
    delay(4);
  }
  delay(1000);//hold your horses
  nextState = 5;//diag
}

void diagnostic_state(){
  digitalWrite(LEDBLUEpin, 0);//turn off blue!
  
  for(int i = 0; i<N; i++)
  {
    digitalWrite(LEDREDpin, 1);
    delay(200);
    digitalWrite(LEDREDpin, 0);
    delay(200);
  }
  if(N==5) {N = 8;}//update this dawg
  else{ N = 5;}

  delay(3000);
  nextState = 2;//TURN OFF
  
}
