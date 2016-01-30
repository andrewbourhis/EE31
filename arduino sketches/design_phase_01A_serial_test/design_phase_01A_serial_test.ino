#include <SoftwareSerial.h>

SoftwareSerial theSerialPort(10,11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){
    ;
  }

  Serial.println("Ready to go...");

  theSerialPort.begin(2400);
  theSerialPort.println("Hello, world?");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(theSerialPort.available())
    Serial.write(theSerialPort.read());
  if(Serial.available())
    theSerialPort.write(Serial.read());
    
}
