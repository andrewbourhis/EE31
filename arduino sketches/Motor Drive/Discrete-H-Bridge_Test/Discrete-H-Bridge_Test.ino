int pinAgnd = 5, pinAvcc = 6, pinBgnd = 7, pinBvcc = 8;
int serialPacket = 0;
bool motorFWD = 1;
void setup() {
  //init serial COM
  Serial.begin(9600);
  while(!Serial);
  //init pins
  pinMode(pinAgnd, OUTPUT);
  pinMode(pinBgnd, OUTPUT);
  pinMode(pinAvcc, OUTPUT);
  pinMode(pinBvcc, OUTPUT);
  //init direction A
  digitalWrite(pinBgnd, LOW);
  digitalWrite(pinBvcc,LOW);
  delay(10);//don't want to fry anything, let mosfets switch
  digitalWrite(pinAgnd, HIGH);
  digitalWrite(pinAvcc, HIGH);
}

void loop() {
  if(Serial.available() > 0)
  {
    serialPacket = Serial.read();
  }
  //we want to change direction
  if(serialPacket != 0){
    changeDirection();
    serialPacket = 0; //reset packet to 0
  }
}
void changeDirection()
{
  if(digitalRead(pinAgnd))//we're in direction A
  {
    digitalWrite(pinAgnd, LOW);
    digitalWrite(pinAvcc,LOW);
    delay(10);
    digitalWrite(pinBgnd, HIGH);
    digitalWrite(pinBvcc, HIGH);
  }
  else
  {
    digitalWrite(pinBgnd, LOW);
    digitalWrite(pinBvcc,LOW);
    delay(10);
    digitalWrite(pinAgnd, HIGH);
    digitalWrite(pinAvcc, HIGH);
  }
}

