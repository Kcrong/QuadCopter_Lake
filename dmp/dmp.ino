#include "Wire.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include "I2Cdev.h"

MPU6050 mpu;

boolean dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t FIFOcount;
uint8_t FIFObuffer[64];

float ypr[3];
float yprLast[3];
int16_t gyro[3];

Quaternion q;
VectorFloat gravity;

volatile boolean mpuInterrupt = false;
void dmpDataReady(){
  mpuInterrupt = true;
}
 
void setup() {
  Serial.begin(9600);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; 
   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
   #endif
   
   mpu.initialize();

   mpu.dmpInitialize();
   mpu.setXGyroOffset(220);
   mpu.setYGyroOffset(76);
   mpu.setZGyroOffset(-85);
   mpu.setZAccelOffset(1788);
   
   mpu.setDMPEnabled(true);

   attachInterrupt(0, dmpDataReady, RISING);
   mpuIntStatus = mpu.getIntStatus();

   dmpReady = true;

 
   packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
  if(dmpReady==false) return;

  while (!mpuInterrupt && FIFOcount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    FIFOcount = mpu.getFIFOCount();

    if((mpuIntStatus & 0x10) || FIFOcount == 1024){
      mpu.resetFIFO();
   
    } else if (mpuIntStatus & 0x02){
      
        while (FIFOcount < packetSize) {
          FIFOcount = mpu.getFIFOCount();
        }
        
        mpu.getFIFOBytes(FIFObuffer, packetSize);
        
        FIFOcount -= packetSize;
     
        mpu.dmpGetGyro(gyro, FIFObuffer);
        mpu.dmpGetQuaternion(&q, FIFObuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // ypr[0] : YAW
        // ypr[1] : PITCH
        // ypr[2] : ROLL
        Serial.println(ypr[1]);
    }
 

}
