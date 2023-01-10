#include "drive.h"
#include "math.h"
#include <algorithm>
#include "function.h"

void Motor::rising(){
	for(int i=0; i<motor_num; i++){
		this->speed[i] *= MAX_SPEED/100;
		/*
		if(motor[i]>0){
			this->motor[i] = map(speed[i],0,MAX_PWM,0,MAX_PWM);
		}else if(motor[i]<0){
			this->motor[i] = map(speed[i],0,-MAX_PWM,-0,-MAX_PWM);
		}else{
			this->motor[i] = 0;
		}*/
	}
}
void Motor::update(){
	//rising();
	if(stop_flag){
		for(int i=0; i<motor_num; i++){
			this->motor[i] = 0;
		}
	}else{
		for(int i=0; i<motor_num; i++){
			if(fabs(speed[i]-motor[i])>0)smoothRising(&(this->motor[i]),speed[i],smooth_normal,smooth_slow,small_pwm);
			else motor[i] = speed[i];
		}
	}
}

int Motor::smooth(float *n,float target,float degree){
    if(*n < target){
        *n = std::min(*n + degree,target);
    }
    if(*n > target){
        *n = std::max(*n - degree,target);
    }
    return *n;
}

int Motor::smoothRising(float *n,float target,float degree_normal,float degree_slow,float threshold){
    if(/*abs(*n) < threshold*/0)smooth(n,threshold,degree_slow);
    else{
        float t=*n;
        int sign = t/abs(t);
        smooth(&t,target,degree_normal);/*
        if(isBetween(sign*threshold,*n,t)){
            *n = sign*threshold;
            return 0;
        }*/
        *n = t;
        return *n == target;

    }
    return 0;
}
void Drive::setDriveGain(short _gain[DRIVEN_MOTOR_NUM][3]){
	for(int i=0; i<DRIVEN_MOTOR_NUM; i++){
		drive_gain[i][0] = _gain[i][0];
		drive_gain[i][1] = _gain[i][1];
		drive_gain[i][2] = _gain[i][2];
	}
}
void Drive::setDrivePID(double _gx[3],double _gy[3],double _gr[3]){
	for(int i=0;i<3;i++){
		pid_x[i] = _gx[i];
		pid_y[i] = _gy[i];
		pid_r[i] = _gr[i];
	}	
	x_cmp.setGain(_gx[0],_gx[1],_gx[2]);
	y_cmp.setGain(_gy[0],_gy[1],_gy[2]);
	r_cmp.setGain(_gr[0],_gr[1],_gr[2]);
}
void Drive::move(Point p){
	//p.r *= ROTATION_GAIN;
	for(int i=0; i<DRIVEN_MOTOR_NUM; i++){
		this->driven_motor.speed[i] = drive_gain[i][0] * p.x + drive_gain[i][1] * p.y + drive_gain[i][2] * p.r;
		this->driven_motor.speed[i] *= MAX_SPEED/100;
		spdd[i]=this->driven_motor.speed[i];
	}nm=p;
}
void Drive::update(){
	this->driven_motor.update();
	for(int i=0; i<DRIVEN_MOTOR_NUM; i++){
		this->motor[i] = driven_motor.motor[i];
	}
	x_cmp.setSetPoint(now.x);
	y_cmp.setSetPoint(now.y);
	r_cmp.setSetPoint(now.r);
}
void Drive::relativeMove(Point p){
	Point tem_p = Point(0,0,0); //座標
	double cosR = cos(-now.r);
	double sinR = sin(-now.r);
	tem_p.x = p.x*cosR - p.y*sinR;
	tem_p.y = p.x*sinR + p.y*cosR;
	tem_p.r = p.r;
	move(tem_p);
}
double Drive::absoluteMove(){
	// to.x = 0;
	// to.y = 45;
	// to.r = -PI/2;
	Point now_diff = Point(to.x-now.x,to.y-now.y,to.r-now.r);
	double distance;/*
	if(fabs(now_diff.x)<100){
		sum.x += now_diff.x;
	}else{
		sum.x = 0;
	}
	if(fabs(now_diff.y)<100){
		sum.y += now_diff.y;
	}else{
		sum.y = 0;
	}
	if(fabs(now_diff.r)<0.02){
		sum.r += now_diff.r;
	}else{
		sum.r = 0;
*/
	Point power = Point(0,0,0);
	x_cmp.setProcessValue(to.x);
	y_cmp.setProcessValue(to.y);
	r_cmp.setProcessValue(to.r);

	power.x = -x_cmp.compute();
	power.y = -y_cmp.compute();
	power.r = -r_cmp.compute();nm=power;
	relativeMove(power);
	/*
	power.x = pid_x[0]*(now_diff.x) + pid_x[1]*sum.x - pid_x[2]*(diff.x - now_diff.x);
	power.y = pid_y[0]*(now_diff.y) + pid_y[1]*sum.y - pid_y[2]*(diff.y - now_diff.y);
	power.r = pid_r[0]*(now_diff.r) + pid_r[1]*sum.r - pid_r[2]*(diff.r - now_diff.r);
	relativeMove(power);
	diff.x = now_diff.x;
	diff.y = now_diff.y;
	diff.r = now_diff.r;*/
	distance = sqrt(diff.x*diff.x+diff.y*diff.y);
	return distance;
}
void Drive::searchPosition(double enc_x,double enc_y,double radd){
	double cosR = cos(radd);//+PI/4
	double sinR = sin(radd);
	now.x += enc_x*cosR - enc_y*sinR;
	now.y += enc_x*sinR + enc_y*cosR;
}
void Drive::pidSetup(){
		this->x_cmp.setTSample(cycle);
		this->x_cmp.setInputLimits(-MAX_INPUT_x , MAX_INPUT_x);
		this->x_cmp.setOutputLimits(-MAX_OUTPUT ,MAX_OUTPUT);
		//this->x_cmp.setIncompleteDifferential(CUTOFF_D);
		//this->x_cmp.setBias(THRESHOLD);

		this->y_cmp.setTSample(cycle);
		this->y_cmp.setInputLimits(-MAX_INPUT_Y , MAX_INPUT_Y);
		this->y_cmp.setOutputLimits(-MAX_OUTPUT ,MAX_OUTPUT);
		//this->y_cmp.setIncompleteDifferential(CUTOFF_D);
		//this->y_cmp.setBias(THRESHOLD);

		this->r_cmp.setTSample(cycle);
		this->r_cmp.setInputLimits(-MAX_INPUT_R , MAX_INPUT_R);
		this->r_cmp.setOutputLimits(-MAX_OUTPUT ,MAX_OUTPUT);
		//this->r_cmp.setIncompleteDifferential(CUTOFF_D);
		//this->r_cmp.setBias(THRESHOLD);
}
double map(int x,int from_min,int from_max,int to_min,int to_max){
	return ((x - from_min)*(to_max - to_min))/(from_max - from_min) + to_min;
}
int isBetween(int a,int b,int c){
    return (a>c&&a<b)||(a>b&&a<c);
}

/*///////

      [0]
   [3]   [1]
      [2]

      //////*/


void anomalyDriveing(double x, double y, double r, double* velocity, double rad){
    #define WHEEL_COUNT 4
    
    #define TRANSLATIONAL_VELOCITY_RATIO    40.0
    #define ROTETIONAL_VELOCITY_RATIO       20.0

    #define SPEED_LIMIT 4000.0
    double diff;
    double sita,sita1;
    
    diff = Controll(x ,y);
    sita = Sita(x, y);
    sita1 = (PI/4) - Sita(x, y) - rad;
    if(sita1>PI)sita1 = sita1 - 2*PI;

    const double gainxy[WHEEL_COUNT] = { cos(sita1), sin(sita1), -cos(sita1), -sin(sita1)};
    const double gainr[WHEEL_COUNT] = { 1, 1, 1, 1};

    double spd[WHEEL_COUNT]={0};
    double max_spd = 0;
    int i;

    for(i=0; i<WHEEL_COUNT; i++){
        double fast;
        spd[i] = diff * gainxy[i] * TRANSLATIONAL_VELOCITY_RATIO + r * gainr[i] * ROTETIONAL_VELOCITY_RATIO;
        fast = (spd[i]>0) ? spd[i] : -spd[i];
    if(fast>max_spd) max_spd = fast;
    }
    if(SPEED_LIMIT < max_spd){
        for(i=0;i<WHEEL_COUNT;i++){
            spd[i] *= SPEED_LIMIT / max_spd;
        }
    }
    for(i=0; i<WHEEL_COUNT; i++){
        velocity[i] = spd[i];
    }
}