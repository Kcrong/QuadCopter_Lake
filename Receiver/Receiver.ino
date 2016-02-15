#include <PinChangeInt.h>
#include "Wire.h"

#define RC_Ch1 3
#define RC_Ch2 4
#define RC_Ch3 5
#define RC_Ch4 6

#define ROLL_MIN -30
#define ROLL_MAX 30
#define PITCH_MIN -30
#define PITCH_MAX 30

#define ESC_MIN 800
#define ESC_MAX 2200 


//Rx interrupt
boolean InterruptLock = false;

//calculate Rx pulse
volatile unsigned long LastCh1 = micros();
volatile unsigned long LastCh2 = micros();
volatile unsigned long LastCh3 = micros();
volatile unsigned long LastCh4 = micros();


//Rx raw data
volatile float Ch1;
volatile float Ch2;
volatile float Ch3;
volatile float Ch4;

uint16_t Throttle,  LastThrottle=0;
int8_t TargetROLL, TargetPITCH;
int8_t LastTargetROLL, LastTargetPITCH;




void setup() {
  Wire.begin(4);
  Wire.onRequest(Send);
 
  Serial.begin(9600);

  PCintPort::attachInterrupt(RC_Ch1, Ch1_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch2, Ch2_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch3, Ch3_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch4, Ch4_Interrupt, CHANGE);

}

void loop() {

    TargetROLL = map(Ch1, 1100, 1900, ROLL_MIN, ROLL_MAX);
    TargetPITCH = map(Ch2, 1100, 1900, PITCH_MIN, PITCH_MAX);
    Throttle = map(Ch3, 1130, 1800, ESC_MIN, ESC_MAX);
    
    if (Throttle < ESC_MIN || Throttle > ESC_MAX)
      Throttle = LastThrottle;
    
    if (TargetROLL < ROLL_MIN || TargetROLL > ROLL_MAX)
      TargetROLL = LastTargetROLL;

    if (TargetPITCH < PITCH_MIN || TargetPITCH > PITCH_MAX)
      TargetPITCH = LastTargetPITCH;
      
    if (TargetROLL >= -3 && TargetROLL <= 3)
      TargetROLL = 0;

    if (TargetPITCH >= -3 && TargetPITCH <= 3 )
      TargetPITCH = 0;

    LastThrottle = Throttle;
    LastTargetROLL = TargetROLL;
    LastTargetPITCH = TargetPITCH;

  
     
     
}


void Ch1_Interrupt() {
    Ch1 = micros() - LastCh1;
  LastCh1 = micros();
}

void Ch2_Interrupt() {
    Ch2 = micros() - LastCh2;
  LastCh2 = micros();
}

void Ch3_Interrupt() {
    Ch3 = micros() - LastCh3;
  LastCh3 = micros();
}

void Ch4_Interrupt() {
    Ch4 = micros() - LastCh4;
  LastCh4 = micros();
}


void Send() {

  byte bufferArr[4];
  
  bufferArr[0] = (Throttle >> 8) &0xFF;
  bufferArr[1] = Throttle & 0xFF;
  bufferArr[2] = TargetROLL & 0xFF;
  bufferArr[3] = TargetPITCH & 0xFF;
  
  Wire.write(bufferArr,4);
  
}

