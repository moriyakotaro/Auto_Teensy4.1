#ifndef _ROBOMASMOTOR_H_
#define _ROBOMASMOTOR_H_

#include "CAN.h"
#include "PIDcontroller.h"
#include <arduino.h>

#define M3508 
#define M2006
#define GM6020

#define CAN_BITRATE 1000000
#define MAXRPM 9300//9300
#define MAXANPARE 16384

#define CUTOFF 2.5

typedef struct{
	
}RobomasMotor;

typedef struct cmp{
	
	double target_rpm;
	double error;

}ComputeVal;

class DJIMotor : public CanControl{
	public:
		
		DJIMotor(CanControl* _can ,double cycle ,uint8_t used_motor ); //CanControlオブジェクト,制御周期(s),使用するモータの数

		void init(); //bitrateを設定しCAN通信を始める setup()に書くこと
		
		int16_t input_ampare_data[9];

		/*モーターのデータ　※angle[1] はID_1のモーターの角度*/

		int16_t angle[9];
		int16_t rpm[9];
		int16_t ampare[9];
		int8_t temperature[9];

		bool is_use[9];

		void setTargetRPM(uint8_t id,double _rpm); //目標rpmの設定
		void setPIDgain(uint8_t id, double Kp ,double Ki ,double Kd); //PIDゲインの設定
		void speedControl(int bus,double *angle0); //速度制御関数 設定した制御周期で呼び出すこと
		void updateMotorData(int bus,uint8_t id); //モーターのデータを更新
		static void send();
		bool checkIsContanct(uint8_t id); //CANの電源が入っているか
		void rote(uint8_t,int16_t); //指定電流でモータを回す
		double positioningConrol(uint8_t id ,double position);
	private:

		CanControl* canforDrive;
		
		void membaInit();

		const double feed_forward_bias = 200;
		
		static DJIMotor* interrupt_func_pass;

		PID PIDcmp[9];

		double check_ampare[9][10];
		double check_rpm[9][10];

		
		
		double cycle;
		
};

#endif