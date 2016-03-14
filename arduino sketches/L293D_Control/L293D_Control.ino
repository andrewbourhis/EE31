int pinA0 = 4, pinA1 = 5, pinB0 = 6, pinB1 = 7;
int pinChangeDirA = 2, pinChangeDirB = 3;
int pinMotorASpeed = A0, pinMotorBSpeed = A1;
int MotorASpeed = 100, MotorBSpeed = 100;
int serialPacket = 0;
bool motorAFWD = 0;
bool motorBFWD = 0;
long lastDebounceTime = 0;


void setup() {
  //init pins
  Serial.begin(9600);
  while(!Serial);
  pinMode(pinA1, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinA0, OUTPUT);
  pinMode(pinB0, OUTPUT);
  pinMode(pinChangeDirA, INPUT);
  pinMode(pinChangeDirB, INPUT);
  pinMode(pinMotorASpeed, INPUT);
  pinMode(pinMotorBSpeed, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinChangeDirA), changeDirA,FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChangeDirB), changeDirB,FALLING);
  
}

void changeDirA(){
  if((millis() - lastDebounceTime) > 500){//only allow change in direction every .1 seconds
    Serial.println(MotorBSpeed);
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
  lastDebounceTime = millis();
}

void changeDirB(){
  if((millis() - lastDebounceTime) > 500){//only allow change in direction every .1 seconds
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
    lastDebounceTime = millis();
    
  }
}

void loop() {
  MotorASpeed = map(analogRead(pinMotorASpeed),0,1023,0,255);
  MotorBSpeed = map(analogRead(pinMotorBSpeed),0,1023,0,255);
  if(motorAFWD)
  {
    digitalWrite(pinA1, LOW);
    delay(10);
    analogWrite(pinA0, MotorASpeed);
  }
  else{
    digitalWrite(pinA0, LOW);
    delay(10);
    analogWrite(pinA1, MotorASpeed);
  }
  
  if(motorBFWD)
  {
    digitalWrite(pinB1, LOW);
    delay(10);
    analogWrite(pinB0, MotorBSpeed);
    
  }
  else{
    digitalWrite(pinB0, LOW);
    delay(10);
    analogWrite(pinB1, MotorBSpeed);
  }
  
}

