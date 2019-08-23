// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 8;
int in2 = 7;
// motor two
int enB = 9;
int in3 = 6;
int in4 = 5;
int sensorValL = A0;
int sensorValF = A1;
int sensorValR = A2;
//int sensorVal4 = A3;
int spd = 80;
int distL = 0;
int distF = 0;
int distR = 0;
//int digIn = 0;
//int total = 0;
void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(sensorValL, INPUT);
  pinMode(sensorValF, INPUT);
  pinMode(sensorValR, INPUT);
  //pinMode(sensorVal4, INPUT);
  Serial.begin(9600);
}
 void Adjust(int dL,int dR)
{
  if((dist>13) && (dist)<16)
  {
    AdjustLeft();
  }
  if(dist<11)
  {
    AdjustRight();
  }
  
}
int distance_calF(int anl)
{
  float volts = anl*0.0048828125;
  int dist = 12.21*pow(volts, -1.1221);
  //value_scaled = ((float)value/615)*1024;
  if(dist>30)
  {
    dist = 30;
  }
  return dist;
}
int distance_calL(int ann)
{
  float volts = ann*0.0048828125;  // value from sensor * (5/1024)
  int distL = 13*pow(volts, -1); // worked out from datasheet graph
  if(distL>30)
  {
    distL = 30;
  }
  return distL;
}
int distance_calR(int anv)
{
  float volts = anv*0.0048828125;  // value from sensor * (5/1024)
  int distR = 13*pow(volts, -1); // worked out from datasheet graph
  if(distR>30)
  {
    distR = 30;
  }
  return distR;
}
void AdjustLeft()
{
  //delay(20);
  //this function will run the motors in the left direction at a fixed speed
  // turn on motor A;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enA, 0);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enB, spd);
}
void AdjustRight()
{
  //delay(20);
 //this function will run the motors in the right direction at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enA, spd);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enB, 0); 
}
void Straight()
{
  // this function will run the motors in one direction at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enA, spd);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enB, spd);
}
void Left()
{
  //Straight();
  //delay(20);
  //this function will run the motors in the left direction at a fixed speed
  // turn on motor A;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enA, 0);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enB, spd);
}
void Right()
{
  //Straight();
  //this function will run the motors in the right direction at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enA, spd);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enB, 0);
}
void sharpRight()
{
  //Straight();
  //this function will run the motors in the right direction at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enA, 150);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 50 out of possible range 0~255
  analogWrite(enB, 80);
}
void Stop()
  {
    // now turn off motors
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(500);
  }
void Back()
{
  //turn the motors in the opposite direction
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  // set speed to 80 out of possible range 0~255
  analogWrite(enA, 85);
  // turn on motor B
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enB, 85);  
}
void Uturn()
{
  //Straight();
  //delay(20);
  //this function will run the motors in the left direction at a fixed speed
  // turn on motor A;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 50 out of possible range 0~255
  analogWrite(enA, 100);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 80 out of possible range 0~255
  analogWrite(enB, 100);
  delay(600);
}
  /*delay(2000);
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);*/
void loop()
{
  
  int sensorL = analogRead(sensorValL);
  int sensorF = analogRead(sensorValF);
  int sensorR = analogRead(sensorValR);
  //int sensor4 = analogRead(sensorVal4);
  /*
  if (sensor1 > 900)
  {
    Left();
  }
  else
  {
    Stop();
  }
  
  if ((sensor4 > 900) && (sensor1 < 900) && (sensor2 > 900) && (sensor3 < 900))
  {
    Straight();
  }
  else if ((sensor4 > 900) && (sensor1 > 900) && (sensor2 > 900) && (sensor3 < 900))
  {
    delay(30);
    Left();
  }
  else if ((sensor4 > 900) && (sensor1 < 900) && (sensor2 > 900) && (sensor3 > 900)) 
  {
    delay(35);
    Right();
  }
  else if ((sensor4 > 900) && (sensor1 < 900) && (sensor2 < 900) && (sensor3 < 900))
  {
    delay(35);
    Right();
  }
  else if ((sensor4 > 900) && (sensor1 > 900) && (sensor2 > 900) && (sensor3 > 900))
  {
    delay(35);
    Left();
  }
  else if ((sensor4 < 900) && (sensor1 < 900) && (sensor2 < 900) && (sensor3 < 900))
  {
    Back();
    delay(300);
    Stop();
    delay(300);
    Right();
    delay(300);
    //Left();
    //delay(300);
  }
   //intln(sensor1);
  Serial.println(sensor4);
  //Serial.println(total);
  //delay(50);        // delay in between reads for stability
*/
distF = distance_calF(sensorF);
distL = distance_calL(sensorL);
distR = distance_calR(sensorR);
if((distL<=13) && (distF>=6) && (distR<=13))
{
  Straight();
  Adjust(distL,distR);
}
else if((distL>13) && (distF<25) && (distR>13))
{
  Left();
}
else if((distL>13) && (distF>=6) && (distR>13))
{
  Left();
  Adjust(distL);
}
else if((distL<=13) && (distF>=6) && (distR>13))
{
  Straight();
}
else if((distL>13) && (distF<25) && (distR<=13))
{
  Left();
}
else if((distL<=13) && (distF<12) && (distR>13))
{
  //Stop();
  sharpRight();
  delay(200);
}
else if((distL<=13) && (distF<25) && (distR<=13))
{
  Stop();
  Uturn();
  Stop();
}
Serial.println(distR);
delay(50);

/*
if(distF <= 7)
{
  Stop();
}
else
{
Straight();
Adjust(distL);
}
Serial.println(sensorF);
delay(50);

*/
}


