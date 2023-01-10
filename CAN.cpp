#include <CAN.h>
#include <FlexCAN_T4.h>
#include "main.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

std::map<uint32_t ,CAN_message_t> buffer_table1;
std::vector<CAN_message_t> msg_stack1;
std::map<uint32_t ,CAN_message_t> buffer_table2;
std::vector<CAN_message_t> msg_stack2;
std::map<uint32_t ,CAN_message_t> buffer_table3;
std::vector<CAN_message_t> msg_stack3;

CanControl::CanControl(uint8_t _canbus){//ok
	if(_canbus == 1){
		_can1 = true;
	}else if(_canbus == 2){
		_can2 = true;
	}else if(_canbus == 3){
		_can3 = true;
	}
	is_can_open = is_start_Com = I_have_buffer_data = false;
}


void CanControl::init(int baudrate){//ok
	if(_can1){
		Can1.begin();
		Can1.setBaudRate(baudrate);
		is_can_open = true;
	}else if(_can2){
		Can2.begin();
		Can2.setBaudRate(baudrate);
		is_can_open = true;
	}else if(_can3){
		Can3.begin();
		Can3.setBaudRate(baudrate);
		is_can_open = true;
	}
}

void CanControl::resetTable(){//ok
	if(_can1){
		buffer_table1.clear();
	}else if(_can2){
		buffer_table2.clear();
	}else if(_can3){
		buffer_table3.clear();
	}
	I_have_buffer_data = false;
}

void CanControl::CANDataPull(uint32_t id ,uint8_t data[8],uint8_t bus){//ok
	CAN_message_t msg_;
	switch(bus){
		case 1:
			msg_ = buffer_table1[id];
			memcpy(data,msg_.buf,8);
			break;
		case 2:
			msg_ = buffer_table2[id];
			memcpy(data,msg_.buf,8);
			break;
		case 3:
			msg_ = buffer_table3[id];
			memcpy(data,msg_.buf,8);
			break;
		default:
			break;
	}
}

int8_t CanControl::CANAllDataRead(){//ok
	// printD(1);
	if(!is_can_open)return -1;
	// printD(2);
	static uint8_t data_vanishing;
	static CAN_message_t msg;
	data_vanishing++;
	if(_can1){
		
		if(Can1.read(msg)){
			I_have_buffer_data = true;
			buffer_table1[msg.id] = msg;
			data_vanishing = 0;
			// printD(3);
		}
	}
	if(_can2){
		// printD(Can2.read(msg));
		if(Can2.read(msg)){
			I_have_buffer_data = true;
			buffer_table2[msg.id] = msg;
			data_vanishing = 0;
		}
	}
	if(_can3){
		if(Can3.read(msg)){
			I_have_buffer_data = true;
			buffer_table3[msg.id] = msg;
			data_vanishing = 0;
		}
	}
	is_start_Com = true;
	if(data_vanishing == 0){
		return 0;
	}else if(data_vanishing < 16){
		return data_vanishing;
	}else {
		is_start_Com = false;
		return -1;
	}
	
}


void CanControl::CANDataPush(uint32_t id ,uint8_t data[8]){//ok
	MSG.id = id;
	if(_can1){
		memcpy(MSG.buf,data,8);
		msg_stack1.push_back(MSG);
	}else if(_can2){
		memcpy(MSG.buf,data,8);
		msg_stack2.push_back(MSG);
	}else if(_can3){
		memcpy(MSG.buf,data,8);
		msg_stack3.push_back(MSG);
	}
	return;
}

void CanControl::MsgStackClear(){//ok
	msg_stack1.clear();
	msg_stack2.clear();
	msg_stack3.clear();
}

void CanControl::CANMsgWrite(CAN_message_t msg){//ok
	if(!is_can_open)return;
	if(_can1){
		Can1.write(msg);
	}else if(_can2){
		Can2.write(msg);
	}else if(_can3){
		Can3.write(msg);
	}
	return;
}

int8_t CanControl::CANAllDataWrite(){//ok
	if(!is_can_open)return -1;
	int8_t s = 0;
	if(_can1){
		for(CAN_message_t buff : msg_stack1){
			Can1.write(buff);
		s++;
		}
	}else if(_can2){
		for(CAN_message_t buff : msg_stack2){
			Can2.write(buff);
		s++;
		}
	}else if(_can3){
		for(CAN_message_t buff : msg_stack3){
			Can3.write(buff);
		s++;
		}
	}
	msg_stack1.clear();
	msg_stack2.clear();
	msg_stack3.clear();
	return s;
}