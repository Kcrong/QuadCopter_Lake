#include <PinChangeInt.h>
#include <Servo.h>
#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"


#define pi 3.141592
#define RADIANS_TO_DEGREES 180/3.14159
#define fs 131.0; 
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

Servo M1,M2,M3,M4;


int16_t ax,ay,az;
int16_t gx,gy,gz;
int16_t mx,my,mz;


float base_gx=0, base_gy=0, base_gz=0;            //gyro bias
float Accel_PITCH, Accel_ROLL;                         //acceleration
float Gyro_PITCH = 0, Gyro_ROLL = 0 ;             //gyro
float Rate_PITCH, Rate_ROLL;                           //angular velocity
float Angle_PITCH = 0;                                        //final angle
float Angle_ROLL = 0;

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
int8_t LastROLL, LastPITCH;

void calibrate(){  
   
  int loop =10;
  for (int i=0;i<loop;i++)
  {
    mpu.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);
    base_gx += gx;
    base_gy += gy;  
    base_gz += gz;
    delay(80);
  }
  
  base_gx /=loop;
  base_gy /=loop;
  base_gz /=loop;
}
int count=0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  TWBR = 24;
  //calibrate();
  InterruptAttach();
  InitESC();

}

void loop() {
/*count +=1;
if(count ==100)
  Serial.println(millis());*/
///////////////////////////////////////////////////get angle///////////////////////////////////////////////////
   mpu.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);

   Accel_PITCH = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RADIANS_TO_DEGREES;
   Accel_ROLL = atan(-1*ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES;

  //angular velocity
   Rate_PITCH = (gx-base_gx)/fs;  
   Rate_ROLL = (gy-base_gy)/fs;

   Gyro_PITCH = Angle_PITCH + (Rate_PITCH*SamplingTime);
   Gyro_ROLL = Angle_ROLL + (Rate_ROLL*SamplingTime);


  //complementary filter
   Angle_PITCH = (0.98*Gyro_PITCH) + (0.02*Accel_PITCH);
   Angle_ROLL = (0.98*Gyro_PITCH) + (0.02*Accel_ROLL);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////Interrupt///////////////////////////////////////////////////
  //get Targetangle, Throttle
  AcquireLock();
  
  Ch3 = floor(Ch3/50)*50;
  
  TargetROLL = map(Ch1, 1100, 1900, ROLL_MIN, ROLL_MAX);
  TargetPITCH = map(Ch2, 1100, 1900, PITCH_MIN, PITCH_MAX);
  Throttle = map(Ch3, 1130, 1800, ESC_MIN, ESC_MAX); 

  if(TargetROLL < ROLL_MIN || TargetROLL > ROLL_MAX)
    TargetROLL = LastROLL;
    
  if(TargetPITCH < PITCH_MIN || TargetPITCH > PITCH_MAX)
    TargetPITCH = LastPITCH;
  
  if(Throttle < ESC_MIN || Throttle > ESC_MAX)
    Throttle = LastThrottle;

  if(TargetROLL >= -3 && TargetROLL <= 3)
    TargetROLL = 0;

  if(TargetPITCH >= -3 && TargetPITCH <= 3 )
    TargetPITCH = 0;
  
  
  LastThrottle = Throttle;
  LastROLL = TargetROLL;
  LastPITCH = TargetPITCH;
  
  ReleaseLock();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Double-loop Proportional, Integral, Differential Control (PID)
  
  //Serial.print(TargetROLL);
  //Serial.print("      ");
  //Serial.println(TargetPITCH);
 // Serial.print("  ");
 

 
  
  M1.writeMicroseconds(Throttle);

}



void Ch1_Interrupt() {
  if(InterruptLock==false) 
    Ch1 = micros() -LastCh1;
  LastCh1 = micros();
}

void Ch2_Interrupt() {
  if(InterruptLock==false) 
    Ch2 = micros() -LastCh2;
  LastCh2 = micros();
}

void Ch3_Interrupt() {
  if(InterruptLock==false) 
    Ch3 = micros() -LastCh3;
  LastCh3 = micros();
}

void Ch4_Interrupt() {
  if(InterruptLock==false) 
    Ch4 = micros() -LastCh4;
  LastCh4 = micros();
}

void AcquireLock(){
  InterruptLock = true;
}

void ReleaseLock(){ 
  InterruptLock = false;
}

void InitESC(){
  M1.attach(ESC5);
  M2.attach(ESC6);
  M3.attach(ESC9);
  M4.attach(ESC10);

  //set ESC 
  /*M1.writeMicroseconds(ESC_min);
  M2.writeMicroseconds(ESC_min);
  M3.writeMicroseconds(ESC_min);
  M4.writeMicroseconds(ESC_min);

  //delay(5000);
*/
 }


void InterruptAttach(){
  PCintPort::attachInterrupt(RC_Ch1, Ch1_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch2, Ch2_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch3, Ch3_Interrupt, CHANGE);
  PCintPort::attachInterrupt(RC_Ch4, Ch4_Interrupt, CHANGE);
}
 
