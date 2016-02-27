#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


MPU6050 mpu;



#define ROLL_MIN -30
#define ROLL_MAX 30
#define PITCH_MIN -30
#define PITCH_MAX 30
#define YAW_MIN -20
#define YAW_MAX 20


//ESC pin
#define ESC5 5
#define ESC6 6
#define ESC10 10
#define ESC11 11 

#define ESC_MIN 800
#define ESC_MAX 2200 

#define SamplingTime 0.01


//Gain value
#define OuterPgain 4.7
#define Pgain 2.35
#define Igain 0.43
#define Dgain 0.344

#define YAW_Pgain 2.0
#define YAW_Igain 0.5

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
float YAW_Rate;
float ROLL_AngleError, PITCH_AngleError;
float ROLL_RateError, PITCH_RateError;
float YAW_RateError;

float ROLL_P, ROLL_D, ROLL_I = 0;
float PITCH_P, PITCH_D, PITCH_I = 0;
float YAW_P, YAW_I = 0;

float ROLL_LastRateError ;
float PITCH_LastRateError ;

float ROLL_PID = 0;
float PITCH_PID = 0;
float YAW_PI;

 
float M1Out, M2Out, M3Out , M4Out;


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;

uint16_t Throttle,  LastThrottle=0;
int8_t TargetROLL;
int8_t TargetPITCH;
int8_t TargetYAW;
int8_t LastTargetROLL, LastTargetPITCH,LastYAW;

byte a,b,c,d,e;

void dmpDataReady() {
  mpuInterrupt = true;
}

unsigned long dmpReady = 0;


void setup() {
  Wire.begin();
  TWBR = 24;

  //Serial.begin(115200);
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
    YAW_Rate = (float)gyro[2];
    
    RC_Command();

    if(Throttle>1800)
      Throttle = 1800;

    
    if(dmpReady > 1700){
      PIDcontrol();
      digitalWrite(13,HIGH);
    }
    else{
      digitalWrite(13,LOW);
    }
   

    M1Out = (ROLL_PID - PITCH_PID)*0.5 + YAW_PI + (float)Throttle;
    M2Out = (-ROLL_PID - PITCH_PID)*0.5 - YAW_PI + (float)Throttle;
    M3Out = (-ROLL_PID + PITCH_PID)*0.5 + YAW_PI + (float)Throttle;
    M4Out = (ROLL_PID + PITCH_PID)*0.5 - YAW_PI + (float)Throttle;

    //Serial.print(M1Out);
    //Serial.print("   ");
    //Serial.println(M3Out);
   
    if(Throttle<900){
      M1Out = ESC_MIN;
      M2Out = ESC_MIN;
      M3Out = ESC_MIN;
      M4Out = ESC_MIN;
      ROLL_I = PITCH_I = YAW_I = 0;
      
    }

   
    
   

    ///////////////////////////////Motor OUTPUT//////////////////////////////////////////

      M1.writeMicroseconds(M1Out);
      M2.writeMicroseconds(M2Out);
      M3.writeMicroseconds(M3Out);
      M4.writeMicroseconds(M4Out);
  
  dmpReady++;
  }

}

void PIDcontrol(){
  //Double-loop Proportional, Integral, Differential Control (PID)
    //ROLL CONTROL////////////////////////////////////////////////
    ROLL_AngleError = TargetROLL - ROLL_Angle;
    ROLL_RateError = (ROLL_AngleError * OuterPgain) + ROLL_Rate;

    ROLL_P = ROLL_RateError * Pgain;
    ROLL_I += (ROLL_RateError * SamplingTime) * Igain;
    LIMIT(&ROLL_I , -200, 200);
    ROLL_D = ((ROLL_RateError - ROLL_LastRateError) / SamplingTime) * Dgain;

    ROLL_LastRateError = ROLL_RateError;

    ROLL_PID = ROLL_P + PITCH_I + ROLL_D;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////


    //PITCH CONTROL//////////////////////////////////////////////////
    PITCH_AngleError = TargetPITCH - PITCH_Angle;
    PITCH_RateError = (PITCH_AngleError * OuterPgain) - PITCH_Rate;   

    PITCH_P = PITCH_RateError * Pgain;
    PITCH_I += (PITCH_RateError * SamplingTime) * Igain;
    LIMIT(&PITCH_I , -200, 200);
    PITCH_D = ((PITCH_RateError - PITCH_LastRateError) / SamplingTime) * Dgain; 
    
    PITCH_LastRateError = PITCH_RateError;
    
    PITCH_PID = PITCH_P + PITCH_I + PITCH_D;


    //YAW CONTROLL/////////////////////////////////////////////////
    YAW_RateError = TargetYAW - YAW_Rate;
    YAW_P = YAW_RateError * YAW_Pgain;
    YAW_I += (YAW_RateError * SamplingTime) * YAW_Igain ;
    LIMIT( &YAW_I, -200, 200);

    YAW_PI = YAW_P + YAW_I;
    
}



void InitESC() {
  M1.attach(ESC5);
  M2.attach(ESC6);
  M3.attach(ESC10);
  M4.attach(ESC11);

  //set ESC
    M1.writeMicroseconds(ESC_MIN);
    M2.writeMicroseconds(ESC_MIN);
    M3.writeMicroseconds(ESC_MIN);
    M4.writeMicroseconds(ESC_MIN);

    delay(3000);
  
}

void RC_Command() {
  Wire.requestFrom(4, 5);

  
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();
  e = Wire.read();
  
  Throttle = a;
  Throttle = (Throttle << 8) | b;
  TargetROLL = c;
  TargetPITCH = d;
  TargetYAW = e;

  
    if (Throttle < ESC_MIN || Throttle > ESC_MAX)
      Throttle = LastThrottle;

    if (TargetROLL < ROLL_MIN || TargetROLL > ROLL_MAX)
      TargetROLL = LastTargetROLL;

    if (TargetPITCH < PITCH_MIN || TargetPITCH > PITCH_MAX)
      TargetPITCH = LastTargetPITCH;

    if (TargetYAW < YAW_MIN || TargetYAW > YAW_MAX)
     TargetYAW = LastYAW;
      
    if (TargetROLL >= -3 && TargetROLL <= 3)
      TargetROLL = 0;
    
    if (TargetPITCH >= -3 && TargetPITCH <= 3 )
      TargetPITCH = 0;

    if(TargetYAW >= -3 && TargetYAW <= 3)
      TargetYAW = 0;

     
  
    LastThrottle = Throttle;
    LastTargetROLL = TargetROLL;
    LastTargetPITCH = TargetPITCH;
    LastYAW = TargetYAW;

    
  }
  
void LIMIT(float *Value, int MIN, int MAX){
    if(*Value < MIN){
        *Value = MIN;
    }
    else if(*Value > MAX){
        *Value = MAX;
    }
} 

