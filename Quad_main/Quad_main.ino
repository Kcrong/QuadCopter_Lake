#include <PinChangeInt.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"




MPU6050 mpu;

//Rx channel pin
#define RC_Ch1 4
#define RC_Ch2 7
#define RC_Ch3 8
#define RC_Ch4 12

#define ROLL_MIN -30
#define ROLL_MAX 30
#define PITCH_MIN -30
#define PITCH_MAX 30

//ESC pin
#define ESC5 5
#define ESC6 6
#define ESC9 9
#define ESC10 10

#define ESC_MIN 800
#define ESC_MAX 2200 


#define SamplingTime 0.01


//Gain value
#define OuterPgain 4.7
#define Pgain 1
#define Igain 0.4
#define Dgain 0.335

Servo M1, M2, M3, M4;


uint8_t mpuIntStatus;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float yrp[3];
int16_t gyro[3];
float ROLL_Angle, ROLL_Rate;
float PITCH_Angle, PITCH_Rate;
float ROLL_AngleError, PITCH_AngleError;
float ROLL_RateError, PITCH_RateError;

float ROLL_P, ROLL_D, ROLL_I = 0;
float PITCH_P, PITCH_D, PITCH_I = 0;

float ROLL_LastRateError ;
float PITCH_LastRateError ;

float ROLL_PID = 0;
float PITCH_PID = 0;

float M1Out, M2Out, M3Out , M4Out;

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



volatile bool mpuInterrupt = false;


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



uint16_t Throttle,  LastThrottle;
int8_t TargetROLL, TargetPITCH;
int8_t LastTargetROLL, LastTargetPITCH;



void dmpDataReady() {
  mpuInterrupt = true;
}

unsigned long dmpReady = 0;

void setup() {

  Wire.begin();
  TWBR = 24;

  Serial.begin(115200);
  mpu.initialize();
  mpu.dmpInitialize();

  mpu.setXGyroOffset(220); //220
  mpu.setYGyroOffset(76); //76
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();


  packetSize = mpu.dmpGetFIFOPacketSize();


  InterruptAttach();
  InitESC();

  pinMode(13,OUTPUT);

}

void loop() {

  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();

  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
    // ypr[0] : YAW
    // yrp[1]  gyro[1] : ROLL
    // yrp[2]  gyro[2] : PITCH
    ROLL_Angle = yrp[1] * 180 / M_PI - 0.3;
    PITCH_Angle = yrp[2] * 180 / M_PI - 1.1;

    ROLL_Rate = (float)gyro[1];
    PITCH_Rate = (float)gyro[0];

    AcquireLock();

    Ch3 = floor(Ch3 / 50) * 50;

    TargetROLL = map(Ch1, 1100, 1900, ROLL_MIN, ROLL_MAX);
    TargetPITCH = map(Ch2, 1100, 1900, PITCH_MIN, PITCH_MAX);
    Throttle = map(Ch3, 1130, 1800, ESC_MIN, ESC_MAX);

    if (TargetROLL < ROLL_MIN || TargetROLL > ROLL_MAX)
      TargetROLL = LastTargetROLL;

    if (TargetPITCH < PITCH_MIN || TargetPITCH > PITCH_MAX)
      TargetPITCH = LastTargetPITCH;

    if (Throttle < ESC_MIN || Throttle > ESC_MAX)
      Throttle = LastThrottle;

    if (TargetROLL >= -3 && TargetROLL <= 3)
      TargetROLL = 0;

    if (TargetPITCH >= -3 && TargetPITCH <= 3 )
      TargetPITCH = 0;


    LastThrottle = Throttle;
    LastTargetROLL = TargetROLL;
    LastTargetPITCH = TargetPITCH;

    ReleaseLock();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(Serial.available())
      dmpReady = 2000;
      
    if(dmpReady > 1700){
      PIDcontrol();
      digitalWrite(13,HIGH);
    }
    else{
      digitalWrite(13,LOW);
      Serial.print(ROLL_Angle);
      Serial.print("   ");
      Serial.println(PITCH_Angle);

    }



    
    if(Throttle > 1300)
      Throttle = 1300;

    


  //dmpReady++;
  }

}

void PIDcontrol(){
  //Double-loop Proportional, Integral, Differential Control (PID)
    //ROLL CONTROL////////////////////////////////////////////////
    ROLL_AngleError = TargetROLL - ROLL_Angle;
    ROLL_RateError = (ROLL_AngleError * OuterPgain) - ROLL_Rate;

    ROLL_P = ROLL_RateError * Pgain;
    ROLL_I += (ROLL_RateError * SamplingTime) * Igain;
    LIMIT(&ROLL_I , -100, 100);
    ROLL_D = ((ROLL_RateError - ROLL_LastRateError) / SamplingTime) * Dgain;

    ROLL_LastRateError = ROLL_RateError;

    ROLL_PID = ROLL_P + PITCH_I + ROLL_D;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //PITCH CONTROL//////////////////////////////////////////////////
    PITCH_AngleError = TargetPITCH - PITCH_Angle;
    PITCH_RateError = (PITCH_AngleError * OuterPgain) - PITCH_Rate;

    PITCH_P = PITCH_RateError * Pgain;
    PITCH_I += (PITCH_RateError * SamplingTime) * Igain;
    LIMIT(&PITCH_I , -100, 100);
    PITCH_D = ((PITCH_RateError - PITCH_LastRateError) / SamplingTime) * Dgain;

    PITCH_LastRateError = PITCH_RateError;

    PITCH_PID = PITCH_P + PITCH_I + PITCH_D;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    M1Out = (ROLL_PID - PITCH_PID)*0.5 + (float)Throttle;
    M2Out = (-ROLL_PID - PITCH_PID)*0.5 + (float)Throttle;
    M3Out = (-ROLL_PID + PITCH_PID)*0.5 + (float)Throttle;
    M4Out = (ROLL_PID + PITCH_PID)*0.5 + (float)Throttle;

    if(Throttle<900){
      M1Out = ESC_MIN;
      M2Out = ESC_MIN;
      M3Out = ESC_MIN;
      M4Out = ESC_MIN;
    }


    ///////////////////////////////ESC OUTPUT//////////////////////////////////////////
   // M1.writeMicroseconds(M1Out);
    M2.writeMicroseconds(M2Out);
   // M3.writeMicroseconds(M3Out);
    M4.writeMicroseconds(M4Out);
    
}



void Ch1_Interrupt() {
  if (InterruptLock == false)
    Ch1 = micros() - LastCh1;
  LastCh1 = micros();
}

void Ch2_Interrupt() {
  if (InterruptLock == false)
    Ch2 = micros() - LastCh2;
  LastCh2 = micros();
}

void Ch3_Interrupt() {
  if (InterruptLock == false)
    Ch3 = micros() - LastCh3;
  LastCh3 = micros();
}

void Ch4_Interrupt() {
  if (InterruptLock == false)
    Ch4 = micros() - LastCh4;
  LastCh4 = micros();
}

void AcquireLock() {
  InterruptLock = true;
}

void ReleaseLock() {
  InterruptLock = false;
}

void InitESC() {
  M1.attach(ESC5);
  M2.attach(ESC6);
  M3.attach(ESC9);
  M4.attach(ESC10);

  //set ESC
    M1.writeMicroseconds(ESC_MIN);
    M2.writeMicroseconds(ESC_MIN);
    M3.writeMicroseconds(ESC_MIN);
    M4.writeMicroseconds(ESC_MIN);

    delay(5000);
  
}


void InterruptAttach() {
  PCintPort::attachInterrupt(RC_Ch1, Ch1_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch2, Ch2_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch3, Ch3_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch4, Ch4_Interrupt, CHANGE);
}

void LIMIT(float *value, int MIN, int MAX)
{
    if(*value < MIN){
        *value = MIN;
    }
    else if(*value > MAX){
        *value = MAX;
    }
} 

