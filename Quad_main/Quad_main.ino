#include <PinChangeInt.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL


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

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high



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


void dmpDataReady() {
    mpuInterrupt = true;
}

int count=0;

void setup() {
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(115200);
  mpu.initialize();
  mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
    
  packetSize = mpu.dmpGetFIFOPacketSize();

  
  InterruptAttach();
  InitESC();

}

void loop() {
/*count +=1;
if(count ==100)
  Serial.println(millis());*/
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

        //mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // ypr[0] : YAW
        // ypr[1] : PITCH
        // ypr[2] : ROLL
        float ROLL = ypr[1] * 180/M_PI-0.3;
        float PITCH = ypr[2] * 180/M_PI-1.1; 

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

  Serial.print(ROLL);
  Serial.print("   ");
  Serial.print(PITCH);
  Serial.print("   ");
  Serial.print(TargetROLL);
  Serial.print("   ");
  Serial.print(TargetPITCH);
  Serial.print("   ");
  Serial.println(Throttle);

 
  
  M1.writeMicroseconds(Throttle);

      
    }
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
 
