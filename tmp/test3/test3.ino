

#include<SoftwareSerial.h>

char INBYTE;
int  LED = 10; // LED on pin 13

SoftwareSerial slaveRobo(10,11); 


void setup() {
  Serial.begin(9600); 
  while(!Serial);
  slaveRobo.begin(9600);
  pinMode(LED, OUTPUT);
  
}

void loop(){ 
  
   
  slaveRobo.write('1');
  delay(500);
  slaveRobo.write('0');
  delay(500);       
  }
 

