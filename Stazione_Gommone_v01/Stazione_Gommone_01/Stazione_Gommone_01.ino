/*
##########################################################
STAZIONE GOMMONE METIS VELA UNIPD
##########################################################
      Andrea     Mastrangelo
Dott. Marco Tarantino
      Stefano    Pieretti
##########################################################
*/

#include<Wire.h>
#define WSPIN 2;
#define ADD 0x25
byte data[4];

int wd;
byte ws, bat;

void setup() {
Wire.begin(ADD);
Wire.onRequest(requestevent);

pinMode(WSPIN,INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(WSPIN), ISR_FUNC, RISING);

}
void loop() {
  wd = analogRead(A0);
  bat=analogRead(A1);
  data[0]=wd; //LSB wd
  data[1]=wd>>8; //MSB wd
  

}

void requestEvent(){
  Wire.write(data,4)
  ws=0;
}

void ISR_FUNC{
  ++ws;
}

