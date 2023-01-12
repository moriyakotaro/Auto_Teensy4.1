#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "PIDcontroller.h"

#define MAX_PWM 255        //PWMの最大値
#define DRIVEN_MOTOR_NUM 4 //駆動のモーターの数

#define MAX_SPEED 6000

#define MAX_INPUT_x 500
#define MAX_INPUT_Y 500
#define MAX_INPUT_R 2*PI

#define MAX_OUTPUT 100

struct Point{
public:
	double x;
	double y;
	double r;
	Point(double x_,double y_,double r_){
		x = x_;
		y = y_;
		r = r_;
	}
};

class Motor{
public:
	float motor[30];
	float speed[30];
	int motor_num;
	float smooth_slow;
	float smooth_normal;
	float rising_threshold;
	float small_pwm;
	bool stop_flag = 0;
	void setDriveGain(short _gain[DRIVEN_MOTOR_NUM][3]);
	void rising();
	void update();
	Motor(int motor_num_,float smooth_slow_,float smooth_normal_,float rising_threshold_,float small_pwm_){
		this->motor_num = motor_num_;
		this->smooth_slow = smooth_slow_;
		this->smooth_normal = smooth_normal_;
		this->rising_threshold = rising_threshold_;
		this->small_pwm = small_pwm_;
	}
    private:
        int smooth(float *n,float target,float degree);
        int smoothRising(float *n,float target,float degree_normal,float degree_slow,float threshold);
};
class Drive{
public:
	Motor driven_motor = Motor(DRIVEN_MOTOR_NUM,0,0,0,0);
	double motor[DRIVEN_MOTOR_NUM];

	Point to = Point(0,0,0);
	Point now = Point(0,0,0);
	Point diff = Point(0,0,0);
	Point sum = Point(0,0,0);
	
	void setDriveGain(short _gain[DRIVEN_MOTOR_NUM][3]);
	void setDrivePID(double _gx[3],double _gy[3],double _gr[3]);
	void setTargetGrid(double _x,double _y,double _r);
	void setNowGrid(double _x,double _y,double _r);
	void move(Point p);
	void relativeMove(Point p);
	double absoluteMove();
	void searchPosition(double enc_x,double enc_y,double radd);
	void update();
 	double pid_x[3];
 	double pid_y[3];
	double pid_r[3];
	PID x_cmp;
	PID y_cmp;
	PID r_cmp;
	Point nm = Point(0,0,0);
	short drive_gain[DRIVEN_MOTOR_NUM][3];
	Drive(Motor motor_,double cycle){
			this->driven_motor = motor_;
			this->cycle = cycle;
			this->pidSetup();
	}
	double spdd[4];
private:
	void pidSetup();
	double cycle;
};
double map(int x,int from_min,int from_max,int to_min,int to_max);
int isBetween(int a,int b,int c);
void anomalyDriveing(double x, double y, double r, double* velocity, double rad);

#endif 