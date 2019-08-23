/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*
 * rajnikanth_robot.c
 *
 * Created: 8/22/2016 6:28:23 PM
 *  Author: Rabing
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//Beginning of Auto generated function prototypes by Atmel Studio
int main(void);
//End of Auto generated function prototypes by Atmel Studio

#ifndef _BV(val) 
#define _BV(val) 1<<val
#endif

#define Left_front_motor_output OCR3B
#define Right_front_motor_output OCR3A
#define Left_back_motor_output OCR4B
#define Right_back_motor_output OCR4C
#define Servo_motor_output OCR5A

#define Left_front_Motor_Encoder_Interrupt() ISR( INT2_vect )
#define Right_front_Motor_Encoder_Interrupt() ISR( INT3_vect )
#define Left_back_Motor_Encoder_Interrupt() ISR( INT5_vect )
#define Right_back_Motor_Encoder_Interrupt() ISR( INT1_vect )

#define Left_front_Motor_Encoder_Interrupt_Pin 19
#define Right_front_Motor_Encoder_Interrupt_Pin 18
#define Left_back_Motor_Encoder_Interrupt_Pin 3
#define Right_back_Motor_Encoder_Interrupt_Pin 20
#define Servo_motor_Pin 46

#define Left_front_Motor_direction_IN1  1
#define Left_front_Motor_direction_IN2  0
#define Right_front_Motor_direction_IN1 4
#define Right_front_Motor_direction_IN2 5
#define Left_back_Motor_direction_IN1   9
#define Left_back_Motor_direction_IN2  10
#define Right_back_Motor_direction_IN1 11
#define Right_back_Motor_direction_IN2 12


#define Left_front_Encoder_direction 21
#define Right_front_Encoder_direction 13
#define Left_back_Encoder_direction 14
#define Right_back_Encoder_direction 21


#define TimeSlot1() ISR( TIMER1_COMPA_vect, ISR_BLOCK )
#define TimeSlot2() ISR( TIMER1_COMPB_vect, ISR_BLOCK )
#define TimeSlot3() ISR( TIMER1_COMPC_vect, ISR_BLOCK )


typedef struct 
{
  float kp,kd;
  float Proportional,Derivative,error,lasterror,totalerror;
  float Setpoint;
  float outMin,outMax;
}PID;

typedef struct 
{
  int Pulse_count;
  unsigned int Time;
  float measure_RPM;
}Encoder;

typedef struct  
{
  PID pid;
  Encoder encoder;
}Motor;

volatile Motor Left_front_Motor, Right_front_Motor, Left_back_Motor, Right_back_Motor;

volatile unsigned int numoftimer0_overflow;


void clearRegister( void );
void SetTuningConstants( PID *pid, float p,  float d );
inline int CalculatePID(  PID *pid );
void PWM_init( void );
void IO_port_init( void );
void servo_target_angle( char angle );
void Encoder_Interrupt_init( void );
void Encoder_Reference_timer_Init(void);
void Time_Slot_init(void);



int main(void)
{
  /*clearRegister();
  
 // PWM_init(  );
  Encoder_Interrupt_init();
  Encoder_Reference_timer_Init();
  Time_Slot_init();
  Serial.begin(9600);
    sei();*/
    //Left_front_motor_output=4000;
 //   IO_port_init();
    pinMode(2,OUTPUT);
    pinMode(1,OUTPUT);
    pinMode(0,OUTPUT);
    digitalWrite(2,HIGH);
    digitalWrite(1,HIGH);
  //digitalWrite(Left_front_Motor_direction_IN2,LOW);
  
    while(1)
    {
      
      _delay_ms(100);
    }
}

TimeSlot1()
{
  
}

TimeSlot2()
{
  
  if( Left_front_Motor.encoder.Pulse_count<0 )
  Left_front_Motor.encoder.Pulse_count= Left_front_Motor.encoder.Pulse_count+1;
  else if( Left_front_Motor.encoder.Pulse_count>0 )
  Left_front_Motor.encoder.Pulse_count=Left_front_Motor.encoder.Pulse_count-1;

  if( Left_front_Motor.encoder.Pulse_count==0)
  Left_front_Motor.encoder.measure_RPM=0;
  else
  Left_front_Motor.encoder.measure_RPM=(83333.333f*(float)Left_front_Motor.encoder.Pulse_count)/(float)Left_front_Motor.encoder.Time;
  Left_front_Motor.encoder.Pulse_count=0;
  Left_front_Motor.encoder.Time=0;
  
  
  if( Right_front_Motor.encoder.Pulse_count<0 )
  Right_front_Motor.encoder.Pulse_count= Right_front_Motor.encoder.Pulse_count+1;
  else if( Right_front_Motor.encoder.Pulse_count>0 )
  Right_front_Motor.encoder.Pulse_count=Right_front_Motor.encoder.Pulse_count-1;

  if( Right_front_Motor.encoder.Pulse_count==0)
  Right_front_Motor.encoder.measure_RPM=0;
  else
  Right_front_Motor.encoder.measure_RPM=(83333.333f*(float)Right_front_Motor.encoder.Pulse_count)/(float)Right_front_Motor.encoder.Time;
  Right_front_Motor.encoder.Pulse_count=0;
  Right_front_Motor.encoder.Time=0;
  
  if( Left_back_Motor.encoder.Pulse_count<0 )
  Left_back_Motor.encoder.Pulse_count= Left_back_Motor.encoder.Pulse_count+1;
  else if( Left_back_Motor.encoder.Pulse_count>0 )
  Left_back_Motor.encoder.Pulse_count=Left_back_Motor.encoder.Pulse_count-1;

  if( Left_back_Motor.encoder.Pulse_count==0)
  Left_back_Motor.encoder.measure_RPM=0;
  else
  Left_back_Motor.encoder.measure_RPM=(83333.333f*(float)Left_back_Motor.encoder.Pulse_count)/(float)Left_back_Motor.encoder.Time;
  Left_back_Motor.encoder.Pulse_count=0;
  Left_back_Motor.encoder.Time=0;
  
  if( Right_back_Motor.encoder.Pulse_count<0 )
  Right_back_Motor.encoder.Pulse_count= Right_back_Motor.encoder.Pulse_count+1;
  else if( Right_back_Motor.encoder.Pulse_count>0 )
  Right_back_Motor.encoder.Pulse_count=Right_back_Motor.encoder.Pulse_count-1;

  if( Right_back_Motor.encoder.Pulse_count==0)
  Right_back_Motor.encoder.measure_RPM=0;
  else
  Right_back_Motor.encoder.measure_RPM=(83333.333f*(float)Right_back_Motor.encoder.Pulse_count)/(float)Right_back_Motor.encoder.Time;
  Right_back_Motor.encoder.Pulse_count=0;
  Right_back_Motor.encoder.Time;

  TCNT0=0;
  numoftimer0_overflow=0;
  Serial.println(Left_front_Motor.encoder.measure_RPM);
  
  
}

TimeSlot3()
{
  
}

Left_front_Motor_Encoder_Interrupt()
{
  static unsigned int start_timeL;
  int pinstate=digitalRead(Left_front_Encoder_direction);

  if( Left_front_Motor.encoder.Pulse_count==0 )
  Left_front_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256;
  else
  Left_front_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256-start_timeL;

  if( pinstate==HIGH )
  Left_front_Motor.encoder.Pulse_count=Left_front_Motor.encoder.Pulse_count+1;
  else
  Left_front_Motor.encoder.Pulse_count=Left_front_Motor.encoder.Pulse_count-1;
  
}

Right_front_Motor_Encoder_Interrupt()
{
  static unsigned int start_timeL;
  int pinstate=digitalRead(Right_front_Encoder_direction);

  if( Right_front_Motor.encoder.Pulse_count==0 )
  Right_front_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256;
  else
  Right_front_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256-start_timeL;

  if( pinstate==HIGH )
  Right_front_Motor.encoder.Pulse_count=Right_front_Motor.encoder.Pulse_count+1;
  else
  Right_front_Motor.encoder.Pulse_count=Right_front_Motor.encoder.Pulse_count-1;
  
  
}

Left_back_Motor_Encoder_Interrupt()
{
  static unsigned int start_timeL;
  int pinstate=digitalRead(Left_back_Encoder_direction);

  if( Left_front_Motor.encoder.Pulse_count==0 )
  Left_back_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256;
  else
  Left_back_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256-start_timeL;

  if( pinstate==HIGH )
  Left_back_Motor.encoder.Pulse_count=Left_back_Motor.encoder.Pulse_count+1;
  else
  Left_back_Motor.encoder.Pulse_count=Left_back_Motor.encoder.Pulse_count-1;
}

Right_back_Motor_Encoder_Interrupt()
{
  static unsigned int start_timeL;
  int pinstate=digitalRead(Right_back_Encoder_direction);

  if( Right_back_Motor.encoder.Pulse_count==0 )
  Right_back_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256;
  else
  Right_back_Motor.encoder.Time=TCNT0+numoftimer0_overflow*256-start_timeL;

  if( pinstate==HIGH )
  Right_back_Motor.encoder.Pulse_count=Right_back_Motor.encoder.Pulse_count+1;
  else
  Right_back_Motor.encoder.Pulse_count=Right_back_Motor.encoder.Pulse_count-1;
  
}

void IO_port_init( void )
{
  pinMode(2,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  
  pinMode(Left_back_Motor_Encoder_Interrupt_Pin,INPUT);
  pinMode(Left_front_Motor_Encoder_Interrupt_Pin,INPUT);
  pinMode(Right_back_Motor_Encoder_Interrupt_Pin,INPUT);
  pinMode(Right_front_Motor_Encoder_Interrupt_Pin,INPUT);
  
  digitalWrite(Left_back_Motor_Encoder_Interrupt_Pin,HIGH);
  digitalWrite(Left_front_Motor_Encoder_Interrupt_Pin,HIGH);
  digitalWrite(Right_back_Motor_Encoder_Interrupt_Pin,HIGH);
  digitalWrite(Right_front_Motor_Encoder_Interrupt_Pin,HIGH);
  
  
  pinMode(Left_front_Encoder_direction,INPUT);
  pinMode(Left_back_Encoder_direction,INPUT);
  pinMode(Right_back_Encoder_direction,INPUT);
  pinMode(Right_front_Encoder_direction,INPUT);
  
  digitalWrite(Left_front_Encoder_direction,HIGH);
  digitalWrite(Left_back_Encoder_direction,HIGH);
  digitalWrite(Right_back_Encoder_direction,HIGH);
  digitalWrite(Right_front_Encoder_direction,HIGH);
  
  pinMode(Left_back_Motor_direction_IN1,OUTPUT);
  pinMode(Left_back_Motor_direction_IN2,OUTPUT);
  pinMode(Right_back_Motor_direction_IN1,OUTPUT);
  pinMode(Right_back_Motor_direction_IN2,OUTPUT);
  pinMode(Left_front_Motor_direction_IN1,OUTPUT);
  pinMode(Left_front_Motor_direction_IN2,OUTPUT);
  pinMode(Right_front_Motor_direction_IN1,OUTPUT);
  pinMode(Right_front_Motor_direction_IN2,OUTPUT);
  pinMode(Servo_motor_Pin,OUTPUT);

  Left_front_Motor.encoder.Pulse_count=0;
  
  
}

void clearRegister( void )
{
  TCCR1A=0x00;TCCR3A=0x00;TCCR4A=0x00;TCCR5A=0x00;
  TCCR1B=0x00;TCCR3B=0x00;TCCR4B=0x00;TCCR5B=0x00;
  TIMSK1=0x00;TIMSK3=0x00;TIMSK4=0x00;TIMSK5=0x00;
  TCCR1C=0x00;TCCR3C=0x00;TCCR4C=0x00;TCCR5C=0x00;
  EICRA=0x00; EICRB=0x00;TCCR0B=0x00; TCCR0A=0x00;
  TIMSK0=0x00;
 
  
}

void PWM_init( void )
{
  TCCR3A|=_BV(COM3A1)|_BV(COM3B1)|_BV(WGM31);
  TCCR3B|=_BV(WGM32)|_BV(WGM33)|_BV(CS31)|_BV(CS30);
  ICR3=4999;
  
  TCCR4A|=_BV(COM4C1)|_BV(COM4B1)|_BV(WGM41);
  TCCR4B|=_BV(WGM42)|_BV(WGM43)|_BV(CS41)|_BV(CS40);
  ICR4=4999;
  
  TCCR5A|=_BV(COM5A1)|_BV(COM5B1)|_BV(WGM51);
  TCCR5B|=_BV(WGM52)|_BV(WGM53)|_BV(CS51)|_BV(CS50);
  ICR5=4999;
  
}

void Encoder_Reference_timer_Init(void)
{
  TCCR0B|=(1<<CS00)|(1<<CS01);
  TIMSK0|=(1<<TOIE0);
  numoftimer0_overflow=0;
}

void Time_Slot_init(void)
{
  TCCR1A|=_BV(WGM11);
  TCCR1B|=_BV(WGM12)|_BV(WGM13)|_BV(CS11);
  ICR1=39999;
  TIMSK1|=_BV(OCIE1A)|_BV(OCIE1B)|_BV(OCR1C);
  
  OCR1A=12000;
  OCR1B=24000;
  OCR1C=36000;  
}

void Encoder_Interrupt_init( void )
{
  EICRA|=_BV(ISC11)|_BV(ISC21)|_BV(ISC31);
  EICRB|=_BV(ISC51);
  EIMSK|=_BV(INT1)|_BV(INT2)|_BV(INT3)|_BV(INT5);
  EIFR=_BV(INTF1)|_BV(INTF2)|_BV(INTF3)|_BV(INTF5);
}

ISR( TIMER0_OVF_vect )
{
  numoftimer0_overflow=numoftimer0_overflow+1;
}





