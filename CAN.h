#ifndef CAN_H
#define CAN_H

#include <vector>
#include <map>
#include <FlexCAN_T4.h>

class CanControl{
	public:
		CanControl(uint8_t _canbus);

		bool is_can_open;
		bool is_start_Com;
		bool I_have_buffer_data;
		
		void init(int baudrate);
		void resetTable();
		void CANDataPull(uint32_t id ,uint8_t data[8] ,uint8_t bus);
		int8_t CANAllDataRead();
		void CANDataPush(uint32_t id ,uint8_t data[8]);
		void MsgStackClear();
		void CANMsgWrite(CAN_message_t msg);
		int8_t CANAllDataWrite();

	private:
		CAN_message_t MSG;
		
		bool _can1 = false;
		bool _can2 = false;
		bool _can3 = false;
		
		//CanControl(uint8_t);
	protected:
		CanControl(){};
};

#endif