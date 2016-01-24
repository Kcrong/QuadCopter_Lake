#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include <SoftwareSerial.h>


int pwm = 126;
int apc;
int Motor[] = {3,9,10,11};
float m[4] = {0,};

#define pi 3.141592
#define RADIANS_TO_DEGREES 180/3.14159
#define fs 131.0; 
MPU6050 mpu;


int16_t ax,ay,az;
int16_t gx,gy,gz;
int16_t mx,my,mz;

//pid게인값
float kp = 2.0;//p gian
float ki = 0.1; //i gain
float kd = 28.0; //d gain
float tau = 0.005; //LPF시정수

//자이로
float gyro_x, gyro_y;

//최종 가속도,자이로 각도
float accel_x, accel_y;
float gyro_angle_x=0, gyro_angle_y=0;

//상보필터 최종각도
float angle_x=0,angle_y=0,angle_z=0;

//자이로센서 바이어스값
float base_gx=0, base_gy=0, base_gz=0;


float pid_y, pid_x; //pid 조작량
float integ_y =0.0; // error 누적값 초기화
float integ_x =0.0; 
float last_d_error_y, last_d_error_x; //전회 error값
float differ_y, differ_x; //전회error와 현재error와의 차이
float last_differ_y, last_differ_x; //for Low Pass Filter
float d_y, d_x; //LPF 필터링 된 값

unsigned long pre_msec=0;



float error_x; //전회각도와 현재각도와의 차이
float error_y; 
  
float d_error_x;//미분제어값 구하기위한 error
float d_error_y;

void calibrate(){  //자이로센서 바이어스
   
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



void setup() {
  Wire.begin();
  
  Serial.begin(9600);
  mpu.initialize();

  for(int i=0;i<4;i++)
    pinMode(Motor[i], OUTPUT);
  calibrate();

   //모터 시동
   /*while(!APC.available()){
     for(int i=0;i<4;i++){
      analogWrite(Motor[i],pwm);
     }
   }*/
   
}


void get_angle(){ //mpu9250 
  mpu.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);

  //가속도값 아크탄젠트->각도변환
  accel_x = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*RADIANS_TO_DEGREES;
  accel_y = atan(-1*ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES;
  

  //자이로 -32766~+32766을 실제 250degree/s로 변환
  //앞에서 계산한 오차의 평균값을 빼줌 
  gyro_x = (gx-base_gx)/fs;  
  gyro_y = (gy-base_gy)/fs;

  //gyro 적분 
  gyro_angle_x = angle_x + dt*gyro_x;
  gyro_angle_y = angle_y + dt*gyro_y;
  
}

void complementary_filter(){   //상보필터
  
   angle_x = 0.97*gyro_angle_x + 0.03*accel_x;
   angle_y = 0.97*gyro_angle_y + 0.03*accel_y;
  
}


void LPF_d(){  //d제어에만 적용되는 저역통과필터
  d_error_x = (target_x - gyro_angle_x);//미분제어값 구하기위한 error(use only gyro)
  d_error_y = (target_y - gyro_angle_y);
  
  differ_y = d_error_y - last_d_error_y; //변화량
  differ_x = d_error_x - last_d_error_x; 
  
  d_y = (tau*last_differ_y + dt*differ_y)/(tau+dt);//d제어에 lpf적용
  d_x = (tau*last_differ_x + dt*differ_x)/(tau+dt);
}


void pid(){ //p+i+lpf_d
  
  integ_y += dt*error_y; //error 적분
  integ_x += dt*error_x;
  
  pid_y = kp*error_y + ki*integ_y + kd*d_y; 
  pid_x = kp*error_x + ki*integ_x + kd*d_x;
}


void throttle_output(){ 
  //pid+throttle
  m[0] = pid_y + pwm;
  m[1] = pid_x + pwm;
  m[2] = -1*pid_y + pwm;
  m[3] = -1*pid_x + pwm;

   for(int i=0;i<4;i++){
    /*if(m[i]<pwm) //pid적용된 값이 throttle 신호보다 작으면 throttle로 고정
      m[i]=pwm;*/
    analogWrite(Motor[i],m[i]);
  }
  
}

void loop() {
  //단위시간 변화량
  float dt = (millis()-pre_msec)/1000.0;
  pre_msec = millis();

  get_angle();
  complementary_filter();
  
  if(APC.available()){
    apc=(int)APC.read();
    /*if (apc > 160)
      apc = 160;*/
    pwm = apc;
  }
  
  float target_x = 0; //목표 각도
  float target_y = 0;
  
  error_x = (target_x - angle_x)/10 + 0.06; //전회각도와 현재각도와의 차이
  error_y = (target_y - angle_y)/10 - 0.06; 

  LPF_d();
  pid();
  
  last_d_error_y = d_error_y; //미분을 위한 전회값
  last_d_error_x = d_error_x;
  
  last_differ_y = d_y; //전회값 for LPF
  last_differ_x = d_x;


  throttle_output();

}
