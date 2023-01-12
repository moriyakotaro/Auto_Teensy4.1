#include "RobomasMotor.h"
#include <Metro.h>
#include "PIDcontroller.h"
#include "main.h"

//Metro motorControl();

ComputeVal Motor_cmp_val[9] = {};
DJIMotor* DJIMotor::interrupt_func_pass = 0;
DJIMotor::DJIMotor(CanControl* _canN ,double _cycle /*単位は秒*/,uint8_t motor_cnt ){
	canforDrive = _canN;
	cycle = _cycle;
	/*
	if(canforDrive->is_can_open){
	}else{
		//abort();
	}*/
	/*
	//void (*p)(); 
	//p = std::bind(&DJIMotor::sendMotorData);
    //std::function<void()> pi = [&]() { return DJIMotor::sendMotorData(); };
    //std::function<void()> pa = std::bind(&DJIMotor::sendMotorData,this);
	//DJIMotor::sendMotorData;
	//void (DJIMotor::*pf)() = DJIMotor::sendMotorData;
	//void (*ps)() = [&]()->void { return DJIMotor::sendMotorData(); };
	//void (*ps)() = pa;
    //boost::function<void ()> func = boost::bind(&DJIMotor::sendMotorData, this);
	*/
    //func();
	//interrupt_func_pass = this;
    //MsTimer2::set(_cycle,send);
}

void DJIMotor::membaInit(){
	for(int i=0;i<9;i++){
		this->angle[i] = 0;
		this->rpm[i] = 0;
		this->ampare[i] = 0;
		this->temperature[i] = 0;
		this->is_use[i] = false;
		for(int j=0;j<10;j++){
			this->check_ampare[i][j] = 0;
			this->check_rpm[i][j] = 0;
		}
		this->PIDcmp[i].setTSample(cycle);
		this->PIDcmp[i].setGain(1.0,0,0);
		this->PIDcmp[i].setInputLimits(-MAXRPM , MAXRPM);
		this->PIDcmp[i].setOutputLimits(-MAXANPARE ,MAXANPARE);
		this->PIDcmp[i].setIncompleteDifferential(CUTOFF);
		this->PIDcmp[i].setBias(feed_forward_bias);
	}
}

void DJIMotor::send(){
	//interrupt_func_pass->MotorControl();
}

void DJIMotor::init(){
	membaInit();
	canforDrive->init(CAN_BITRATE);
}

bool DJIMotor::checkIsContanct(uint8_t id){
	int dif_check = 0;
	for(int i=0;i<9;i++){
		if(check_ampare[id][i] != check_ampare[id][i+1]){
			dif_check++;
		}
		if(check_rpm[id][i] != check_rpm[id][i+1]){
			dif_check++;
		}
	}
	for(int i=0;i<9;i++){
		check_ampare[id][i+1] = check_ampare[id][i];
		check_rpm[id][i+1] = check_rpm[id][i];
	}
	check_ampare[id][0] = this->ampare[id];
	check_rpm[id][0] = this->rpm[id];
	if(dif_check == 0)return false;
	else return true;
}

void DJIMotor::setTargetRPM(uint8_t id ,double _rpm){
	if( id <= 0 || id > 8)return;
	_rpm = (_rpm > MAXRPM) ? MAXRPM : _rpm;
	_rpm = (_rpm < -MAXRPM) ? -MAXRPM : _rpm;

	is_use[id] = true;
	PIDcmp[id].setSetPoint(_rpm);
	// printD(is_use[5]);
}

void DJIMotor::setPIDgain(uint8_t id ,double _kp ,double _ki ,double _kd){
	if( id <= 0 || id > 8)return;
	PIDcmp[id].setGain(_kp,_ki,_kd);
	return;
}

void DJIMotor::updateMotorData(int bus,uint8_t id){
	if( id <= 0 || id > 8)return;
	uint8_t buff[8];
	CANDataPull(0x200+id,buff, bus);
	this->angle[id] = (buff[0]<<8) + buff[1];
	this->rpm[id] = (buff[2]<<8) + buff[3];
	this->ampare[id] = (buff[4]<<8) + buff[5];
	this->temperature[id] = buff[6];
	return;
}

void DJIMotor::speedControl(int bus,double *angle0){
	if((canforDrive->CANAllDataRead()) == -1)return;

	static CAN_message_t msg1;
	static CAN_message_t msg2;
	
	bool is_msg1_data = false;
	bool is_msg2_data = false;

	msg1.id = 0x200;
	msg2.id = 0x1FF;
	msg1.len = msg2.len = 8;
	for(int i=0;i<8;i++){
		msg1.buf[i] = 0;
		msg2.buf[i] = 0;
	}
	
	for(int i=1;i<=8;i++){
		if(is_use[i]){
			updateMotorData(bus,i);
			if(!checkIsContanct(i))continue;
			PIDcmp[i].setProcessValue(this->rpm[i]);
			int16_t input_ampare = (int16_t)constrain(PIDcmp[i].compute(),-MAXANPARE,MAXANPARE);
			if(i <= 4){
				is_msg1_data = true;
				msg1.buf[(i-1)*2] = input_ampare >> 8;
				msg1.buf[(i-1)*2+1] = input_ampare & 0xFF;
			}else if(i <= 8){
				is_msg2_data = true;
				msg2.buf[(i-5)*2] = input_ampare >> 8;
				msg2.buf[(i-5)*2+1] = input_ampare & 0xFF;
			}
			this->input_ampare_data[i] = input_ampare;
		}
	}
	
	if(is_msg1_data)canforDrive->CANMsgWrite(msg1);
	if(is_msg2_data)canforDrive->CANMsgWrite(msg2);

	for(int pp=0;pp<4;pp++){
		angle0[pp] = angle[pp+1];
	}

	return;
}

void DJIMotor::rote(uint8_t id,int16_t comm_ampare){
	if( id <= 0 || id > 8)return;
	comm_ampare = (comm_ampare > MAXANPARE) ? MAXANPARE : comm_ampare;
	comm_ampare = (comm_ampare < -MAXANPARE) ? -MAXANPARE : comm_ampare;
	
	static CAN_message_t msg_rote;

	msg_rote.len = 8;
	for(int i=0;i<8;i++)msg_rote.buf[i] = 0;
	if(id >= 4){
		msg_rote.id = 0x200;
		msg_rote.buf[(id-1)*2] = comm_ampare >> 8;
		msg_rote.buf[(id-1)*2+1] = comm_ampare & 0xFF;
	}
	else if(id >= 8){
		msg_rote.id = 0x1FF;
		msg_rote.buf[(id-5)*2] = comm_ampare >> 8;
		msg_rote.buf[(id-5)*2+1] = comm_ampare & 0xFF;
	}

	canforDrive->CANMsgWrite(msg_rote);
}

double DJIMotor::positioningConrol(uint8_t id ,double position){
	return 0;
}
