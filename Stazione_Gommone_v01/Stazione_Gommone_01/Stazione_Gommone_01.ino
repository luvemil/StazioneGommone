/*
##########################################################
STAZIONE GOMMONE METIS VELA UNIPD
##########################################################
      Andrea     Mastrangelo
Dott. Marco Tarantino
      Stefano    Pieretti
##########################################################
*/

//INSERT LIBRARIES
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include "nRF24L01.h"
#include "RF24.h"

//includere sempre per ultimi e in questo ordine
//#include <mastraSerial.h>
#include <Mille_MEGA.h>

float wind[2];            //speed, direction

unsigned int timer_counter;   //valore di partenza della variabile 'overflow'
unsigned long connectionTimer; //controlla quanto tempo è passato dall'ultima informazione del computer

boolean save = FALSE;
boolean fix = 0;
boolean nLoop = 1; //flag per il salvataggio dell'intestazione del file
boolean changeNameFlag = 0;

float tempDS = 0.0;
byte Wspeed, vale_1, vale_2;
float left, right;

byte cyclecounter = 0;

long delayTime;

//lascio qui la definizione per definire solo quella necessaria
//typedef struct Mvupc_t mvupc_t;
//mvupc_t mvupc;
typedef struct Mvup_t mvup_t;
mvup_t mvup;



//INIT
LCD_I2C lcd(LCD_ADD, 16, 2); //indirizzo, colonne, righe
//LCD_CLASSIC lcd(41, 43, 45, 47, 49, 48, 16, 2); //rs, en, d4-7, colonne, righe
SDCARD sd (SD_SELECT);
GPS gps(2); //1: EM406a, 2: G.top013 //3: Ublox
//GPS gps(3, 1);//second value is frequence in hz
TEMP ds(DS_PIN);




void setup() {
//Starting libraries
  Wire.begin();
  Serial.begin(BAUD);
  Serial.println();
  lcd.start(); //initialize lcd (classic and i2c)
  ds.check();
  sd.init();
  timer_counter = gps.begin(); //default 4800 baud Em406a, 9600 for G.top, write baudrate to modify
  gps.autoset();
  sd.setName(sd.getFreeName (NOMEFILE, 4, 1)); //set name with the first free index (1 for three numbers, 0 for two numbers)
  
  pinMode(SAVELED, OUTPUT);
  pinMode(SAVEBUTTON,INPUT);


  Serial.println("calibration");
  Serial.println(sd.publicName);
  lcd.print(sd.publicName);

  //SETUP Timer1
  noInterrupts();    //Disabilita Interrupt
  //Calcolo il valore iniziale considerando un prescaler di 256
  if (SKIP_GPS){
      timer_counter=65536-16000000/256/2;
    }
  //Azzero i registri
  TCCR1A = 0;
  TCCR1B = 0;
  //Impostazioni dei registri
  TCNT1 = timer_counter;
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts(); //Enable Interrupt

  //Last thing to do in setup!
  AHRS.start(); //initialize time variable
  Serial.println("Everything OK, press button or send '1' to start save data and '0' to stop saving");// 0 to stop
  if (TELEMETRY) Serial3.println("Everything OK, now you can send command");// 0 to stop
  lcd.setCursor(0,0);
  lcd.print("OK ");
  lcd.print("Press Save Bt");
  mvup.tempDS = ds.getTemp();
}


ISR(TIMER1_OVF_vect){ //Interrupt function
  TCNT1 = timer_counter; //'azzera' il contatore
  if (digitalRead(SAVEBUTTON)) {
    if (millis()-delayTime>750){//evita i 'doppi-click'
      save= !save; //activate with button
      delayTime=millis();
      if(save) changeNameFlag = 1;
      }
    }
  if(Serial.available()){
    save = Serial.read()-48; //activate with serial (1-on, 2-off)
    }
  if(telemetry && Serial3.available()){
    //save = Serial3.read()-48; //activate with serial (1-on, 2-off)
    leggiComando();//funzione locale
    }
  
  digitalWrite(SAVELED,save); //led-saveState
  if (save){

    //fix = gps.readGPS(&mvupc.vel, &mvupc.gradi, &mvupc.date, &mvupc.times, &mvupc.lat, &mvupc.lon); //gps reading function
    fix = gps.readGPS(&mvup.vel, &mvup.gradi, &mvup.date, &mvup.times, &mvup.lat, &mvup.lon); //gps reading function
    Serial.println(mvup.times);
    if (SKIP_GPS){
      fix = 1;
      }
  }  
}//end_ISR


void loop() {
  if (changeNameFlag){//cambio nome
    sd.setName(sd.getFreeName (NOMEFILE, 4, 1)); //set name with the first free index (1 for three numbers, 0 for two numbers)
    changeNameFlag = 0;
    if (telemetry) Serial3.println(sd.publicName);
    Serial.println(sd.publicName);
    
    nLoop = 1;
  }
  
  //IMU request
  MPU.readMPU(offAcc, gainAcc, offGyr, gainGyr, acc, gyr); //get calibrated values
  //MPU.readMPU(acc, gyr); //get raw values
  HMC.readHMC(offMag, gainMag, mag);//get calibrated values values
  //HMC.readHMC(mag); //get raw values
  //AHRS.Filter(gyr, acc, mag, BETA, mvupc.attitude); //Filter
  AHRS.Filter(gyr, acc, mag, BETA, mvup.attitude); //Filter
  
  
  if (fix && !changeNameFlag){
    //serialGPS(mvup.vel, mvup.gradi, mvup.date, mvup.times, mvup.lat, mvup.lon, nLoop);
    //Serial.println(mvup.tempDS);
    //serialGPS(mvupc.vel, mvupc.gradi, mvupc.date, mvupc.times, mvupc.lat, mvupc.lon, nLoop);
    printMVUP(mvup);
    
  
    sd.openFile('w');//opening file in write mode
    if (nLoop){
      sd.printFile(sd.getName());sd.printFile(" - ");
      sd.printFile(mvup.date);sd.printFile(" - ");sd.printFile(mvup.times);
      sd.newLineFile();
      
      nLoop = 0;
    }
  
  
    //lettura temperatura
    if (cyclecounter==60){//1 aggiornamento di T al minuto circa
      mvup.tempDS = ds.getTemp();
      cyclecounter = 0;
    }

    //sd.printMVUPC(mvupc); //frase senza condizioni ambientali ed estensimetro
    sd.printMVUP(mvup); //stringa completa
    sd.closeFile(); //close file
  
    fix = 0;
    ++cyclecounter;
  
  }
  
  //WIND.readTiny(5, wind); //Read AtTiny85 wind station
  
  checkConnection(); //check the time passed from the last connection signal and send savestatus and telemetry status
}


void leggiComando(){
  byte command[LENGTH+1];
  byte error = 100;
  if(Serial3.available()){
    error = readCommand(command, LENGTH+1);//it will return datas in command buffer
    if (!error){
      if(!command[0]){
        if(command[1]==5) save = 1;//save on
        if(command[1]==6) save = 0;//save off
        connectionTimer = millis();
        telemetry = 1;
      }
    }
    sendCommand(0, &error, sizeof(error)/sizeof(byte), '$', '\n');//send acknowledge
  }
}

void checkConnection(){
  if (connectionTimer && connectionTimer + CONNECTION_TIMEOUT < millis()) telemetry = 0;
  Serial.print(save);Serial.print('\t');Serial.println(telemetry);
}


