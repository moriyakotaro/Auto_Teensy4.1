#include <Arduino.h>
#include "DualShock.h"
// #include "Drive_.h"
#include "function.h"
// #include <mpu6050.h>
#include <Metro.h>
#include <Encoder.h>
// #include <teensy.h>
// #include "Sonoterm_forArduino.h"
#include <FlexCAN_T4.h>
#include "CAN.h"
#include "RobomasMotor.h"
#include "define.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include "drive.h"
// #include "omuni_driver.h"
// #include "manual.h"
// #include "mars_bone.h"
// #include "pwm_poscontrol.h"

#define HWSERIAL Serial4
// #define PI 3.14159265358979
#define GYAIRO_LED 24
#define GYAIRO_CYCLE 1
#define DISP_CYCLE 10
#define LED_CYCLE 500

short drive_gain[MOTOR_COUNT][3] = {
  { 1,-1,-1},
  { 1, 1,-1},
  {-1, 1,-1},
  {-1,-1,-1},
};
double drive_PIDx_gain[3] = {9.0,0.0,0};
double drive_PIDy_gain[3] = {9.0,0.0,0};
double drive_PIDr_gain[3] = {1.7,0.01,0.5};

double M_Pg = 5.342;
double M_Ig = 4.21;
double M_Dg = 2.4;

double velocity[4] = {0};
double targets_rpm[4] = {-5000,5000,-5000,5000};//速さ(Hz)
int data[10] = {0};
int monitoring = 0; 
const double motor_control_cycle = 2;/*ms*/
const int debugLED = 25;
int ID = 4;
int I = 0;
int debug = 0;
/*
double Pg = 1.12;
double Ig = 0.3;
double Dg = 0.2;*/
double Pg = 0.342;
double Ig = 3.21;
double Dg = 5.4;
int spd = 0;
// static int No = 0;
double gyro_sence;
double x_val;
double y_val;
double x_keep[2];
double y_keep[2];

bool rotation_sign;
int8_t rotation_num=0;
double rotation_delta=0;
float rotation_keep[10];

uint8_t led_count = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

//////////////////////////

////////////////////////

// CanControl DriveCan1(1);//robomas
CanControl DriveCan2(2);//robomas
// CanControl DriveCan3(3);//bonebone
Motor motor(MOTOR_COUNT,SMOOTH_SLOW,SMOOTH_NORMAL,RISING_PWM,SMALL_PWM);
Drive AutoDrive(motor,CONTROL_CYCLE/1000);
DJIMotor SpdControl(&DriveCan2,CONTROL_CYCLE/1000,MOTOR_COUNT);

// Encoder driveX_enc(38,39);
// Encoder drivey_enc(40,41);
Encoder enc_x(ENCODER_X_A,ENCODER_X_B);
Encoder enc_y(ENCODER_Y_A,ENCODER_Y_B);

//Sonoterm term(115200);
Metro debugTiming = Metro(5);
Metro ControlTiming = Metro((motor_control_cycle));
// Metro gyroTiming = Metro(10);
Metro LEDTiming = Metro(500);
// Metro Serialtiming = Metro(1);//マイコン間通信
//速度が速すぎるためわざと遅くしている
Metro gyaroTimming(GYAIRO_CYCLE);
Metro ledTimming(LED_CYCLE);
Metro encoderTimming(ENCODER_CYCLE);
Metro idoutiming(4000);

void debugDisp();
void timer(void);
void control_data_receive(int recive);
void getpalam();
void printD(int a);
void getEncorder();

void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  pinMode(GYAIRO_LED,OUTPUT);
  if(!bno.begin())while(1);
  delay(500);
  bno.setExtCrystalUse(true);
  AutoDrive.setDriveGain(drive_gain);
  AutoDrive.setDrivePID(drive_PIDx_gain,drive_PIDy_gain,drive_PIDr_gain);
  for(int i=1;i<=MOTOR_COUNT;i++){
    SpdControl.setPIDgain(i,M_Pg,M_Ig,M_Dg);
  }
  delay(500);
  SpdControl.init();
  pinMode(24,OUTPUT);
  pinMode(debugLED,OUTPUT);
  delay(100);



}
double idou[5][3] = {{0,0,0},{45,0,PI/2},{45,45,PI},{0,45,PI/2},{0,0,0}};
int c=0;

uint8_t tx_data[8] = {0};
CAN_message_t sendM;
int ss=0;

void loop() {
  // DriveCan1.CANAllDataRead();
  int incomingByte;
  // DriveCan1.CANAllDataRead();
  // if(1){
     //Serial.write(10);
  
    // if (Serial.available() > 0) {
    //         //incomingByte = Serial.read();
    //       //  control_data_receive(incomingByte);
    //         // Serial.write(10);
    //         // Serial.print("USB received: ");
    //         // Serial.print(incomingByte, DEC);
    //         // HWSERIAL.print("USB received:");
    //         // HWSERIAL.print(incomingByte, DEC);

    // }
    if (HWSERIAL.available() > 0) {
            incomingByte = HWSERIAL.read();
            
            control_data_receive(incomingByte);
            // Serial.print(HWSERIAL.available());
            // Serial.write(10);
            // Serial.print("USB received: ");
            // Serial.print(incomingByte, DEC);
            // Serial.print(incomingByte);
            // HWSERIAL.write(ss++);
            // Serial.println();
            // if(ss>10000)ss=0;
            
    }
  // }
//////////////////////////////////////

  // monitoring = 0;
  AutoDrive.now.r = gyro_sence;
	AutoDrive.searchPosition(x_val,y_val);
  AutoDrive.to = Point(idou[c][0],idou[c][1],idou[c][2]);
  if(idoutiming.check()){
    if(c == 4)c=0;
    if(c<4)c++;
  }
  AutoDrive.absoluteMove();
  AutoDrive.update();

// anomalyDriveing(joystick_lx, -joystick_ly, joystick_rx, targets_rpm, gyro_sence);


  targets_rpm[0] = 1000;
  targets_rpm[1] = 1000;
  targets_rpm[2] = 1000;
  targets_rpm[3] = 1000;


// for(int i=0;i<WHEEL_COUNT;i++){
//   // targets_rpm[i] = 100;
//   if(targets_rpm[i]<2000){
//     targets_rpm[i] += 10;
//   }
// }

// if(Button_switch(triangle_button) == 1){
// //   for(int i=0;i<WHEEL_COUNT;i++){
// // //     targets_rpm[i] = targets_rpm[i]/10;
// //       targets_rpm[i] = 1000;
// //   }

//   targets_rpm[0] -= 1;
//   targets_rpm[1] += 1;
//   targets_rpm[2] -= 1;
//   targets_rpm[3] += 1;
// }
// if(Button_switch(cross_button) == 1){


//   targets_rpm[0] += 1;
//   targets_rpm[1] -= 1;
//   targets_rpm[2] += 1;
//   targets_rpm[3] -= 1;
// }




for(int i=1;i<=WHEEL_COUNT;i++){
  SpdControl.setPIDgain(i,Pg,Ig,Dg);
}
        
if(debugTiming.check()){                
  printD(1);
  // for(int i=0;i<8;i++){
  //   Serial.print(msg1.buf[i]);
  //   Serial.print(",");
  //   Serial.print(msg2.buf[i]);
		
	// }
  //  Serial.print(HWSERIAL.available());
  //  Serial.print(",");
  // Serial.print(circle_button);
  // Serial.print(",");
  // Serial.print(triangle_button);
  // Serial.print(",");
  // Serial.print(square_button);
  // Serial.print(",");
  // Serial.print(cross_button);
  // Serial.print(",");
  // Serial.print(joystick_lx);
  // Serial.print(",");
  // Serial.print(joystick_ly);
  // Serial.print(",");
  // Serial.print(joystick_rx);
  // Serial.print(",");
  // Serial.print(joystick_ry);
  // Serial.print(gyro_sence);
  // Serial.print(",");
  // Serial.print(targets_rpm[0]);
  // Serial.print(",");
  // Serial.print(targets_rpm[1]);
  // Serial.print(",");
  // Serial.print(targets_rpm[2]);
  // Serial.print(",");
  // Serial.print(targets_rpm[3]);
  // Serial.println();
 // debugDisp();
  // printMotor();
  //term.dnl();
}

for(int i=1;i<=WHEEL_COUNT;i++){
  SpdControl.setTargetRPM(i,targets_rpm[i-1]);
}

if(ControlTiming.check()){
  // Serial.print(",");
  // targets_rpm[1] = 100;
  
          
  SpdControl.speedControl();
  // spd = 0;
  // tx_data[6] = spd>>8;
  // tx_data[7] = spd&0xff;
  // //DriveCan->CANDataPush(0x200,tx_data);
  // //DriveCan->CANAllDataWrite();
          
  // sendM.id = 0x200;
  // sendM.len = 8;
  // sendM.buf[0] = 0;
  // sendM.buf[1] = 0;
  // sendM.buf[2] = 0;
  // sendM.buf[3] = 0;
  // sendM.buf[4] = 0;
  // sendM.buf[5] = 0;
  // sendM.buf[6] = tx_data[6];
  // sendM.buf[7] = tx_data[7];
  // //motor.rote(sendM);
}




  // for(int i=1;i<=4;i++){
  //   motor.setPIDgain(i,Pg,Ig,Dg);
  // }
  
  // if(debugTiming.check()){
  //   debugDisp();
  //   // printMotor();
  //   term.dnl();
  // }
  // if(ControlTiming.check()){


  //   for(int i=1;i<=4;i++){
  //     motor.setTargetRPM(i,targets_rpm[i-1]);
  //   }
    








  getpalam();
  getEncorder();





  //   motor.speedControl();
  //   spd = 0;
  //   tx_data[6] = spd>>8;
  //   tx_data[7] = spd&0xff;
  //   //DriveCan->CANDataPush(0x200,tx_data);
  //   //DriveCan->CANAllDataWrite();
    
  //   sendM.id = 0x200;
  //   sendM.len = 8;
  //   sendM.buf[0] = 0;
  //   sendM.buf[1] = 0;
  //   sendM.buf[2] = 0;
  //   sendM.buf[3] = 0;
  //   sendM.buf[4] = 0;
  //   sendM.buf[5] = 0;
  //   sendM.buf[6] = tx_data[6];
  //   sendM.buf[7] = tx_data[7];
  //   //motor.rote(sendM);
  // }
  //if(gyroTiming.check())gyro.update();


  //digitalWrite(24,term.key_l.press);
  if(LEDTiming.check())digitalWrite(debugLED,I++%2);
  if(I>1000000)I=0;
  //term.DebugPID(&Pg,&Ig,&targets_rpm);
  //term.DebugEnd();
  //DriveCan->CANAllDataRead();
  // put your main code here, to run repeatedly:
  if(ledTimming.check()){
    digitalWrite(GYAIRO_LED,led_count++%2);
    if(led_count>1000)led_count=0;
  }
}

void getpalam(void){
  //Calibration status values: 0=uncalibrated, 3=fully calibrated
  imu::Vector<3> euler;
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  imu::Quaternion quat = bno.getQuat();
  euler = quat.toEuler();
  //get Axis Data

  double r=euler[0];
  rotation_keep[0] = r;
  rotation_sign = rotation_keep[9] <= rotation_keep[0];
  if(rotation_keep[0]*rotation_keep[9]<0){
    if(PI - fabs(rotation_keep[9]) < 0.1)rotation_sign = rotation_keep[9] > 0;
    //else if(fabs((rotation_num)*PI-gyro_sence) < 0.1)rotation_num += rotation_keep[1] < 0 ? 1:-1;
  }
  if(r*gyro_sence < 0 && fabs(r)>0.1){
    if(r>0)rotation_delta = -2*PI;
    else   rotation_delta = 2*PI ;
  }else{
    rotation_delta = 0;
  }
  
  gyro_sence = r + rotation_delta + 2*PI*rotation_num ;
  for(int i=9;i>0;i--){
    rotation_keep[i] = rotation_keep[i-1];
  }
}

void getEncorder(){
    x_keep[0] = -enc_x.read()/1000.0;
    x_val = x_keep[0] - x_keep[1];
    x_keep[1] = x_keep[0];
    y_keep[0] = enc_y.read()/1000.0;
    y_val = y_keep[0] - y_keep[1];
    y_keep[1] = y_keep[0];

}


void printMotor(){//not use
  //term.disp(motor.rpm[ID]);
  for(int i=0;i<4;i++){
    //term.disp(targets_rpm[i]);
  }
  
  /*
  Serial.print(" ");
  Serial.print(motor.x);*//*
  term.advancedDisp(Pg,5);
  term.advancedDisp(Ig,5);
  term.advancedDisp(Dg,5);*/
  /*
  Serial.print(" ");
  Serial.print(motor.y);*/
  //term.disp(spd);
/*
  if(avarage_count % 10 != 0)smp_avarage10_rpm = (smp_avarage10_rpm + motor.rpm_data[ID])/2;
  else avarage10_rpm = smp_avarage10_rpm;smp_avarage10_rpm = 0;
  if(avarage_count <= 10000)avarage100_rpm = (avarage100_rpm + motor.rpm_data[ID])/2;
  else avarage100_rpm = smp_avarage100_rpm;avarage_count = smp_avarage100_rpm = 0;
  avarage_count++;
  term.disp(avarage10_rpm);
  term.disp(avarage100_rpm);*/
 /*
  uint8_t data[8];
  DriveCan->CANDataPull(0x204,data);
  term.disp((data[0]<<8)+(data[1]&0xff));
  term.disp((data[2]<<8)+(data[3]&0xff));
  term.disp((data[4]<<8)+(data[5]&0xff));
  term.disp(data[6]);*/
  //term.dnl();
  return;
}
void debugDisp(){//not use
  //term.disp(gyro.angle);
  //Serial.print(term.key_k.press);
}

void timer(void){
    monitoring++;
    if(monitoring>=200){
        circle_button   = 0;
        cross_button    = 0;
        triangle_button = 0;
        square_button   = 0;
        left_button  = 0;
        right_button = 0;
        up_button    = 0;
        down_button  = 0;
        l1_button = 0;
        l2_button = 0;
        l3_button = 0;
        r1_button = 0;
        r2_button = 0;
        r3_button = 0;
        ps_button = 0;
        start_button  = 0;
        select_button = 0;
        joystick_rx = 0;
        joystick_ry = 0;
        joystick_lx = 0;
        joystick_ly = 0;
    }
}


void printD(int a){/*
    Serial.print(roll);
    
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");*/
    Serial.print(a);
    Serial.print(",");
    Serial.print(AutoDrive.now.x);
    Serial.print(" ");
    Serial.print(AutoDrive.now.y);
    Serial.print(" ");

    // Serial.print(lxbf);
    // Serial.print(",");
    // Serial.print(lybf);
    // Serial.print(",");
    // Serial.print(rxbf);
    // Serial.print(",,,");
    // Serial.print(gyro_sence);
    // Serial.print(",");
    // Serial.print(targets_rpm[0]);
    // Serial.print(",");
    // Serial.print(targets_rpm[1]);
    // Serial.print(",");
    // Serial.print(targets_rpm[2]);
    // Serial.print(",");
    // Serial.print(targets_rpm[3]);
    // Serial.print(lxaf);
    // Serial.print(",");
    // Serial.print(lyaf);
    // Serial.print(",");
    // Serial.print(rxaf);
    // Serial.print(",,,");

    // Serial.print(lx);
    // Serial.print(",");
    // Serial.print(ly);
    // Serial.print(",");
    // Serial.print(rx);
    // Serial.print(",");
    // Serial.print(con);
    // Serial.print(" ");
    // Serial.print(gyro_sence);
    // Serial.print(" ");
    // Serial.print(AutoDrive.now.x);
    // Serial.print(" ");
    // Serial.print(AutoDrive.now.y);
    // Serial.print(" ");
    // Serial.print(AutoDrive.to.x);
    // Serial.print(" ");
    // Serial.print(AutoDrive.to.y);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.x,6);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.y,6);
    // Serial.print(" ");
    // Serial.print(AutoDrive.nm.r,6);
    // Serial.print(" ");
    // for(int i=1;i<=4;i++){
    //   Serial.print(AutoDrive.motor[i-1]);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.angle[i]);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.ampare[i],10);
    //   Serial.print(" ");
    //   Serial.print(SpdControl.ampare[i],10);
    //   Serial.print(" ");
    // }
      // Serial.print(AutoDrive.spdd[0],6);
      // Serial.print(" ");
    Serial.println();
}

int No = 0;
void control_data_receive(int recive){
  // static int No;
// Serial.print(No);
// Serial.print(monitoring);

// Serial.print(recive);
// Serial.print(" ");
// Serial.print("\n");
  if(recive == 0x80){
    No = 0;
    data[No++] = 0x80;
  }else if(No > 0){
    data[No++] = recive;
    if(No > 8){
      updataState(data);
      // Serial.print("\n");
    }
  }
}

