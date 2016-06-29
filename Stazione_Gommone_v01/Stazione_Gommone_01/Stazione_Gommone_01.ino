#include <Wire.h>

/*
##########################################################
STAZIONE GOMMONE METIS VELA UNIPD
##########################################################
      Andrea     Mastrangelo
Dott. Marco Tarantino
      Stefano    Pieretti
##########################################################
*/


#define WSPIN 2
#define LEDPIN  8
#define ADD 0x25
byte data[4];

int wd = 920;
byte ws = 200;
byte bat = 100;

void setup() {
Wire.begin(ADD);
Serial.begin(57600);
Wire.onRequest(requestEvent);

pinMode(WSPIN,INPUT_PULLUP);
pinMode(LEDPIN,OUTPUT);
attachInterrupt(digitalPinToInterrupt(WSPIN), ISR_FUNC, FALLING);
  digitalWrite(LEDPIN,HIGH);
  delay(1000);
  digitalWrite(LEDPIN,LOW);

}
void loop() {
  wd = analogRead(A0);
  //Serial.println(wd);
  //bat= analogRead(A1);
  data[0]=wd; //LSB wd
  data[1]=wd>>8; //MSB wd
  data[2]=ws;
  data[3]=bat;
  delay(500);
  

}

void requestEvent(){
  digitalWrite(LEDPIN,HIGH);
  Wire.write(data,4);
  ws=0;
  digitalWrite(LEDPIN,LOW);
}
void ISR_FUNC(){
  ++ws;
}

