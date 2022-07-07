#include "atmel_start.h"
#include "hpl_pmc.h"

#include "driver/usb/usb.h"
#include "driver/usb/usb_drive.h"
#include "driver/usb/usb_console.h"
#include "driver/log_msg.h"
#include "driver/adc.h"
#include "driver/eeprom_emu.h"
#include "framework/sensor/sensor.h"
#include "framework/sensor/sensor_db.h"
#include "app/motor_controller.h"
#include "app/analog_poll.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//SETTINGS
#define MAX_THR 10 // the maximum throttle 0% - 100%
#define SYS_DELAY 100 // how many times a second does VCU task receive inputs TODO, NOT IMPLEMETED YET

#define RET_CHECK(x) if (x < 0) return x;
#define BUFFSIZE 256
#define MOTOR_STREAM_SIZE 128 // for both read and write. If update frequency is increased to 100Hz, i.e. 10ms, increase stream size to 512

//critical parameters
#define PRECHARGE_MODE 1
#define AIR_MODE 0
#define ASS_PIN PIN_PC19
#define ASS_OPEN 0
#define ASS_CLOSED 1

static QueueHandle_t read_buf;
static QueueHandle_t pedalQUEUE;
static QueueHandle_t inputQUEUE;
static QueueHandle_t MotortoVCUL;// Left motor communication
static QueueHandle_t VCUtoMotorL;
static QueueHandle_t MotortoVCUR;// Right motor communication
static QueueHandle_t VCUtoMotorR;
static QueueHandle_t bmsQUEUE;		// bms data like voltage, current etc
static QueueHandle_t btRelayQUEUE;	// battery state queue, AUX 1, 2, 3 TODO
static QueueHandle_t VCUtoSpeaker;	// data to speaker, TODO

static char *stok; // strtok state

//VCU <-> Motor Controller messages
struct motorStream{
	uint8_t buf[MOTOR_STREAM_SIZE];
	int length; // string in buffer length
};

//State of Motor Controller
struct motorState{
	char inputmethod;
	int pwm;
	float input_voltage;
	float phase_current;
	int rpm;
	int power_stage_temp;
	int motor_temp;	
};

//VCU_Task input structure
struct inputStruct{
	bool pedalboard_data_comingthru;
	int thr;
	bool goodThr;
	
	bool bms_data_comingthru;
	float batVoltage;				// battery voltage
	float batCurrent;			// battery current
	
	bool btRelay_data_comingthru;
	int relayState;
	
	bool mcl_data_comingthru;
	struct motorState motorstateL;
	bool mcr_data_comingthru;
	struct motorState motorstateR;
	
	bool brakePress_valid;
	float brakePress;
	
	bool assClosed; // true is closed, false is open
};

//VCU_Task output structure. currently not used, as VCU_task directly handles outputs
struct outputStruct{
	bool SetassState;
	//motorStream SetMotorL;
	struct motorStream SetMotorR;
};


/*
	I/O interrupts, tasks etc
	Handles interactions between I/O and input_all_task.
	STATELESS 
*
*
*
*/
// blink when msg is sent
static void CAN_0_tx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8);
	(void)descr;
}


//uint8_t data[8];
uint64_t data; // more general data structure comparing to uint8_t data[8], same size
// blink and store the msg
static void CAN_0_rx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8); // blink when msg is received
	struct can_message msg;
	msg.data = &data;
	can_async_read(descr, &msg);
	
	//Format & send data to input_all_task
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	switch (msg.id)
	{
		case 0x100: // pedal data
			//log_debug("pedal data valid %lx", data);
			xQueueOverwriteFromISR(pedalQUEUE, (void * )msg.data, &xHigherPriorityTaskWoken); // pxHigherPriorityTaskWoken needs reevaluation
			if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			break;
		case 0x6b0: // BMS cell broadcast
			xQueueOverwriteFromISR(bmsQUEUE, (void * )msg.data, &xHigherPriorityTaskWoken);
			if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			log_debug("msg from BMS!");
			break;
		case 0x9: // AUX states
			xQueueOverwriteFromISR(btRelayQUEUE, (void * )msg.data, &xHigherPriorityTaskWoken);
			if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			log_debug("msg about AUX states");
			break;
		default:
			break;
		
	}	
}

/*
Left motor control input & output
controlled through thread safe Queues MotortoVCUL, VCUtoMotorL
*/
static void motorControlLeft(void* p){
	(void) p;
	int r;
	struct mc_inst_t *mc = mc_get_inst(0); // MC_LEFT socket 0
	struct mc_inst_t *mcOther = mc_get_inst(1); // MC_RIGHT socket 1
	
	static uint8_t readbuf[MOTOR_STREAM_SIZE];
	static uint8_t writebuf[MOTOR_STREAM_SIZE];
	struct motorStream readStream;
	struct motorStream writeStream;
	
	log_debug("in motor left");

	mc_passthru_enable(mc);
	mc_passthru_enable(mcOther);
	while (1) {
		if ((r = mc_uart_read(mc, readbuf, sizeof(readbuf)-1))) { // Handles MC to VCU msgs
			readbuf[r] = '\0';
			//log_info("in read: %s", readbuf);
			strncpy(readStream.buf, readbuf, r+1);
			readStream.length = r+1;
			xQueueSendToBack(MotortoVCUL, &readStream, portMAX_DELAY); // review this portMAX_DELAY use
		} else if (xQueueReceive(VCUtoMotorL, &writeStream, 0) == pdTRUE) {	// Handles VCU to MC msgs
			//log_info("write: ENDOFCMD, %d, %s", writeStream.length, writeStream.buf);
			strncpy(writebuf, writeStream.buf, writeStream.length);
			mc_uart_write(mc, (uint8_t *)&writebuf, writeStream.length);
		} else {
			vTaskDelay(1);
		}
	}
	mc_passthru_disable(mc);
	mc_passthru_disable(mcOther);
}

/*
	Right motor control input & output
	controlled through thread safe Queue MotortoVCUR, VCUtoMotorR
*/
static void motorControlRight(void* p){
	(void) p;
	int r;
	struct mc_inst_t *mc = mc_get_inst(1); // MC_RIGHT socket 1
	struct mc_inst_t *mcOther = mc_get_inst(0); // MC_LEFT socket 0
	
	static uint8_t readbuf[MOTOR_STREAM_SIZE];
	static uint8_t writebuf[MOTOR_STREAM_SIZE];
	struct motorStream readStream;
	struct motorStream writeStream;

	log_debug("in motor right");

	mc_passthru_enable(mc);
	mc_passthru_enable(mcOther);
	while (1) {
		if ((r = mc_uart_read(mc, readbuf, sizeof(readbuf)-1))) { // Handles MC to VCU msgs
			readbuf[r] = '\0';
			//log_info("in read: %s", readbuf);
			strncpy(readStream.buf, readbuf, r+1);
			readStream.length = r+1;
			xQueueSendToBack(MotortoVCUR, &readStream, portMAX_DELAY); // review this portMAX_DELAY use
		} else if (xQueueReceive(VCUtoMotorR, &writeStream, 0) == pdTRUE) {	// Handles VCU to MC msgs
			//log_info("write: ENDOFCMD, %d, %s", writeStream.length, writeStream.buf);
			strncpy(writebuf, writeStream.buf, writeStream.length);
			mc_uart_write(mc, (uint8_t *)&writebuf, writeStream.length);
		} else {
			vTaskDelay(1);
		}
	}
	mc_passthru_disable(mc);
	mc_passthru_disable(mcOther);
}

/* 
	decode message coming from motor. remove 0 since 10 of them seems to show up without much reason
	reason found! some VCU -> MC msgs are leaking to MC -> VCU chanel, might be a hardware problem, 
	as the extra msgs are found at mc_uart_read in MotorControlLeft() and MotorControlRight.
	
	Software temporary fix: increase buffer size as VCU update frequency increases.
	
	Example inputs(excluding chars that shouldn't be there): 
	T=3.649V,a=0.000V,PWM= 787,U= 34.9V,I= 3.7A,RPM= 1482,con= 28°C,mot= 26°C
	S=3.649V,a=0.000V,PWM= 787,U= 34.9V,I= 3.7A,RPM= 1482,con= 28°C,mot= 26°C
*/
void decodeMotorMSG(struct motorStream ms, struct motorState *mst){
	int i = 0;
	while(i < ms.length && ms.buf[i] != 's' && ms.buf[i] != 'S' && ms.buf[i] != 't' && ms.buf[i] != 'T' && ms.buf[i] != '*') i++;
	log_debug("MC mode: %c", ms.buf[i]);
	char imode = ms.buf[i];
	if (imode == '*' || ms.length - i < 36) return; // if the data is not valid
	if(imode == 's' || imode == 'S') mst->inputmethod = 's'; 
	if(imode == 't' || imode == 'T') mst->inputmethod = 't';
	
	char* curr = ms.buf+i;
	//S=3.649V,a=0.000V,PWM= 787,U= 34.9V,I= 3.7A,RPM= 1482,con= 28°C,mot= 26°C
#define ERROR_CHECK if ((uintptr_t)curr > (uintptr_t)ms.buf+ms.length) goto error;
	strtof(curr+2, &curr);//analogue throttle input
	ERROR_CHECK
	strtof(curr+4, &curr);//aux/brake input
	ERROR_CHECK
	mst->pwm = strtol(curr+6, &curr, 10);
	ERROR_CHECK
	mst->input_voltage = strtof(curr+3, &curr);
	ERROR_CHECK
	mst->phase_current = strtof(curr+4, &curr);
	ERROR_CHECK
	mst->rpm = strtol(curr+6, &curr, 10);
	ERROR_CHECK
	mst->power_stage_temp = strtol(curr+5, &curr, 10);
	ERROR_CHECK
	mst->motor_temp = strtol(curr+6, &curr, 10);
	ERROR_CHECK
#undef ERROR_CHECK

	error:
	log_warn("mc_parser: read beyond the end of the string!!!");
	return;	
}

// CAN msg decoding tool: get 8 bit msg starting from int position
uint8_t getCanInfo8(uint64_t canMsg, int position){
	uint64_t canMsg1 = canMsg >> position;
	uint64_t canMsg2 = (canMsg1 >> 8) << 8;
	return (uint8_t)(canMsg1 - canMsg2);
}

// CAN msg decoding tool: get 16 bit msg starting from int position
// needs more testing concerning endianess
uint16_t getCanInfo16(uint64_t canMsg, int position){
	uint64_t canMsg1 = canMsg >> position;
	uint64_t canMsg2 = (canMsg1 >> 16) << 16;
	return (uint16_t)(canMsg1 - canMsg2);
}

int getCanInfo1(uint64_t canMsg, int position){
	uint64_t canMsg1 = canMsg >> position;
	uint64_t canMsg2 = (canMsg1 >> 1) << 1;
	return (int)(canMsg1 - canMsg2);
}

/* 
	task to handle all the inputs to VCU module, which output onto output_all_task, 
	STATELESS
*/
static void input_all_task(void *p){
	(void)p;
	//communication channels
	pedalQUEUE = xQueueCreate(1, sizeof(uint64_t));
	MotortoVCUL = xQueueCreate(100, sizeof(struct motorStream));
	MotortoVCUR = xQueueCreate(100, sizeof(struct motorStream));
	inputQUEUE = xQueueCreate(1, sizeof(struct inputStruct));
	bmsQUEUE = xQueueCreate(1, sizeof(uint64_t));
	btRelayQUEUE = xQueueCreate(1, sizeof(uint64_t));
	
	while(1){
		//PEDAL SECTION
		bool pedalboard_data_comingthru;	// between last input_all_task loop and this, are there pedalboard msgs coming in?
		bool pedalboard_live;				// if there's msg coming thru, are the sensors valid?
		int pedalboard_thrpos;				// if there's msg coming thru and sensors are vlid, what is the throttle position?
		uint64_t thr_pos;
		if(xQueueReceive(pedalQUEUE, &(thr_pos), 0) == pdTRUE) //if we have data coming through pedalQUEUE
		{
			pedalboard_data_comingthru = true;
			uint8_t valid = getCanInfo8(thr_pos, 0);	// thr_pos[0], validity
			uint8_t pos1 = getCanInfo8(thr_pos, 8);		// thr_pos[1], throttle position
			pedalboard_live = valid == 1;
			if(pedalboard_live && pos1 >= 0 && pos1 <= 100){ // double check throttle for being valid
				pedalboard_thrpos = pos1;
			}else{
				pedalboard_live = false;
			}
			log_debug("pedal pos: %d, pedal validity: %d", pedalboard_thrpos, pedalboard_live);
		}else{
			pedalboard_data_comingthru = false;
		}
		//PEDAL SECTION ENDS
	
	
		//BMS SECTION
		bool bms_data_comingthru = false;	// Are there bms can msgs coming thru in this time frame?
		float batVoltage;				// battery voltage
		float batCurrent;			// battery current
		uint64_t packinfo;
		while(xQueueReceive(bmsQUEUE, &(packinfo), 0) == pdTRUE)
		{
			bms_data_comingthru = true;
			batVoltage = getCanInfo16(packinfo, 23) * 0.1;
			batCurrent = getCanInfo16(packinfo, 7) * 0.1;
		}
		if(bms_data_comingthru) 
			log_debug("Battery Voltage: %f, Battery Current: %f", batVoltage, batCurrent);
		//BMS SECTION ENDS
		
		
		//BT RELAY SECTION
		bool btRelay_data_comingthru = false;
		int relayState;
		uint64_t relayinfo;
		while(xQueueReceive(btRelayQUEUE, &(relayinfo), 0) == pdTRUE){
			btRelay_data_comingthru = true;
			relayState = (getCanInfo1(relayinfo, 15)+1)%2;//flipping the bit so that 1 is precharge, 0 is air
		}
		if(btRelay_data_comingthru)
			log_debug("Battery Relay State: %d (1 is PRECHARGE CLOSED, 0 is AIR CLOSED)", relayState);
		//BT RELAY ENDS
		
		
		//MC LEFT SECTION
		bool someMsgl = false;			// are there some MC left rs232 msgs coming thru in this time frame?
		struct motorState motorStateL;	// if there's msgs coming thru, what is the current MC left state?
		motorStateL.inputmethod = 'o';	// input method initialized as 'o'. 'o' doesn't represents anything and if there's somemsg, it'll usually be replaced by 's' or 'l'
		struct motorStream msl;
		while(xQueueReceive(MotortoVCUL, &(msl), 0) == pdTRUE){ // this loop will keep updating motor state untill the latest msg
			someMsgl = true;
			log_debug("Motor left says: %s", msl.buf);
			decodeMotorMSG(msl, &motorStateL);
		}
		//log_debug("motor left in method: %c, somemsgl: %d", motorStateL.inputmethod, someMsgl);
		//MC LEFT SECTION ENDS
		
		
		//MC RIGHT SECTION 
		bool someMsgr = false;			// are there some MC rights rs232 msgs coming thru in this time frame?
		struct motorState motorStateR;	// if there's msgs coming thru, what is the current MC left state?
		motorStateR.inputmethod = 'o';	// input method initialized as 'o'. 'o' doesn't represents anything and if there's somemsg, it'll usually be replaced by 's' or 'l'
		struct motorStream msr;
		while(xQueueReceive(MotortoVCUR, &(msr), 0) == pdTRUE){ // this loop will keep updating motor state untill the latest msg
			someMsgr = true;
			log_debug("Motor Right says: %s", msr.buf);
			decodeMotorMSG(msr, &motorStateR);
		}
		//log_debug("motor right in method: %c, somemsgr: %d", motorStateR.inputmethod, someMsgr);
		//MC RIGHT SECTION ENDS


		//VCU ONBOARD SENSOR SECTION
		bool assClosed = gpio_get_pin_level(ASS_PIN);
		float br_pres_val = sensor_get_f(SENS_PRES_R_IN); // SENS_PRES_R_IN is where brake data coming thru
		uint32_t br_pres_ts = sensor_get_ts(SENS_PRES_R_IN);
		bool br_pres_valid = br_pres_val>3000; //nominal data around 5200
		//log_info("PRESSURE SENSOR READING: %d %f, %b", br_pres_ts, br_pres_val, br_pres_valid);
		//VCU ONBOARD SENSOR SECTION ENDS


		//DATA PACKING AND SENDING
		struct inputStruct is;
		is.pedalboard_data_comingthru = pedalboard_data_comingthru;
		is.thr = pedalboard_thrpos;
		is.goodThr = pedalboard_live;
		
		is.bms_data_comingthru = bms_data_comingthru;
		is.batVoltage = batVoltage;
		is.batCurrent = batCurrent;
		
		is.btRelay_data_comingthru = btRelay_data_comingthru;
		is.relayState = relayState;
		
		is.mcl_data_comingthru = someMsgl;
		is.motorstateL = motorStateL;
		is.mcr_data_comingthru = someMsgr;
		is.motorstateR = motorStateR;
		
		is.assClosed = assClosed;
		is.brakePress_valid = br_pres_valid;
		is.brakePress = br_pres_val;
		xQueueOverwrite(inputQUEUE, &is);
		//DATA SENDING ENDS
		vTaskDelay(SYS_DELAY); // updating frequency, defines time frame length. Now its around 100ms
	}
}

// convert floating thr to string commands for motor controllers
static void get_thr_cmd(float desired_thr, struct motorStream *output){
	if(desired_thr > 99.9) desired_thr = 99.9;	// caps throttle at 99.9, cause why not
	if(desired_thr < 0.1){// if throttle is 0 or lower, stop the car
		output->buf[0] = '0';
		output->buf[1] = '\0';
		output->length = 1; // important, length of buffer cannot contain the '\0' char in the end
		return;
	}
	
	int length = 0;
	//decide first char: '1' - '9'
	int firstdec = (int)(desired_thr / 10);
	if(firstdec > 0){		// if 10.0% <= desired_thr <= 99.9%
		output->buf[0] = '0' + firstdec;
		desired_thr -= 10 * firstdec;
		length += 1;
		
		// decide number of '+'
		int seconddec = (int)(desired_thr);
		for(int i = length; i < length+seconddec; i++){
			output->buf[i] = '+';
		}
		desired_thr -= seconddec;
		length += seconddec;
		
		// decide number of 'g' or 0.1% increase throttle
		int thirddec = (int)((desired_thr+0.05) * 10);
		for(int i = length; i < length+thirddec; i++){
			output->buf[i] = 'g';
		}
		length += thirddec;
		
		output->length = length;
		}else{							// if 0.1% <= desired_thr <= 9.9%
		//first letter must be '1'
		output->buf[0] = '1';
		length += 1;
		
		// decide number of '+'
		int seconddec = (int)desired_thr;
		for(int i = length; i < length + (10-seconddec); i++){
			output->buf[i] = '-';
		}
		desired_thr -= seconddec;
		length += (10-seconddec);
		
		int thirddec =  (int)((desired_thr+0.05) * 10);
		for(int i = length; i < length + thirddec; i++){
			output->buf[i] = 'g';
		}
		length += thirddec;
		
		output->length = length;
	}
	
}

/*
	VCU_task, handles the logics
	King task, lads of the lads
	STATEFUL
*/
static void VCU_task(void *p){
	(void)p;
	VCUtoMotorL = xQueueCreate(10, sizeof(struct motorStream));
	VCUtoMotorR = xQueueCreate(10, sizeof(struct motorStream));
	VCUtoSpeaker = xQueueCreate(10, sizeof(8));
	const int pedal_maxTimeNoInfo = 200; // in ms
	const int pedal_maxTimeFailInput = 200; // in ms
	const int mc_maxTimeNoInfo = 500; // in ms
	const int bms_maxTimeNoInfo = 500; // in ms
	const int btRelay_maxTimeNoInfo = 500; // in ms

	bool carSafeToIgnite = false;
	bool carPrecharged = false; // after each ts power up, we can have one precharge. This is used to record that, and don't precharge the car multiple time
	bool carSafeToRun = false;
	
	struct inputStruct is;
	
	bool pedalLive = false;
	int pedalNoInfoCount = 0;
	int pedalFailCount = 0;
	
	bool dashLive = false; // assume dash is not live, so turn on the car asap
	bool wantToIgnite = true; // assume dash is not live, so turn on the car asap
	
	bool bmsLive = false;
	int bmsNoInfoCount = 0;
	
	bool btRelayLive = false;
	int btRelayNoInfoCount = 0;
	
	bool mc_leftLive = false;
	int mclNoInfoCount = 0;
	bool mc_rightLive = false;	
	int mcrNoInfoCount = 0;
	
	// actual datas
	int pedalboard_thrpos = 0;
	bool pedalboard_valid = false;
	
	float batV = 0;
	float batC = 0;
		
	struct motorState motorStateLeft;
	motorStateLeft.inputmethod = "o";
	struct motorState motorStateRight;
	motorStateRight.inputmethod = "o";
	
	//battery state
	bool batteryPrecharged = false;
	int relayState = -1;//battery relay state 1 = precharge, 0 = air+, -1 means no info

	bool assClosed = false;
	
	bool brake_valid = false;
	float brake_press = 0;

	//speaker pwm control
	int period = 333; // assume unit is in us
	float perc = 0.5; // duty cycle period * perc
	//ends
	
	while(1){
		//receiving and updating states
		xQueueReceive(inputQUEUE, &is, portMAX_DELAY); // max delay because we want the VCU_task to not exceed speed of previous call
		// pedal
		if(is.pedalboard_data_comingthru){
			pedalNoInfoCount = 0;//reset
			if(is.goodThr){
				pedalFailCount = 0;//reset
				pedalLive = true;
				pedalboard_thrpos = is.thr;
				pedalboard_valid = is.goodThr;
			}else{
				pedalFailCount++;
				if(pedalFailCount >= pedal_maxTimeFailInput/SYS_DELAY)
					pedalLive = false;
			}
		}else{
			pedalNoInfoCount++;
			if(pedalNoInfoCount >= pedal_maxTimeNoInfo/SYS_DELAY)
				pedalLive = false;
		}
		// battery management system
		if(is.bms_data_comingthru){
			bmsNoInfoCount = 0;
			bmsLive = true;
			batV = is.batVoltage;
			batC = is.batCurrent;
		}else{
			bmsNoInfoCount++;
			if(bmsNoInfoCount > bms_maxTimeNoInfo/SYS_DELAY)
				bmsLive = false;
		}
		// battery state
		if(is.btRelay_data_comingthru){
			btRelayNoInfoCount = 0;
			btRelayLive = true;
			relayState = is.relayState;
		}else{
			btRelayNoInfoCount++;
			if(btRelayNoInfoCount >= btRelay_maxTimeNoInfo/SYS_DELAY)
				btRelayLive = false;
		}
		// motor controller left
		if(is.mcl_data_comingthru){
			mclNoInfoCount = 0; // reset
			if(is.motorstateL.inputmethod == 't' || is.motorstateL.inputmethod == 's'){
				mc_leftLive = true;
				motorStateLeft.inputmethod = is.motorstateL.inputmethod;
			}else{
				mc_leftLive = false;
			}
		}else{
			mclNoInfoCount++;
			if(mclNoInfoCount >= mc_maxTimeNoInfo/SYS_DELAY)
				mc_leftLive = false;
		}
		// motor controller right
		if(is.mcl_data_comingthru){
			mcrNoInfoCount = 0; //reset
			if(is.motorstateR.inputmethod == 't' || is.motorstateR.inputmethod == 's'){
				mc_rightLive = true;
				motorStateRight.inputmethod = is.motorstateR.inputmethod;
			}else{
				mc_rightLive = false;
			}
		}else{
			mcrNoInfoCount++;
			if(mcrNoInfoCount >= mc_maxTimeNoInfo/SYS_DELAY)
				mc_rightLive = false;
		}
		// onboard sensors
		assClosed = is.assClosed;
		brake_valid = is.brakePress_valid;
		brake_press = is.brakePress;
		//log_info("throttle pos: %d, validity: %u, motor state left: %c",pedalboard_thrpos, pedalboard_valid, motorStateLeft);

		// car state
		bool carIntact = pedalLive && bmsLive && btRelayLive && brake_valid; // && dashLive
		bool mcPrechargeOverThreshold = motorStateLeft.input_voltage > 0.95*batV && motorStateRight.input_voltage > 0.95*batV && batV > 30; // batV > 30 to make sure battery voltage is a sensible value
		log_debug("pedalLive: %d, bmsLive: %d, btRelayLive: %d, brake_valid: %d", pedalLive, bmsLive, btRelayLive, brake_valid);
		
		// relay msg to battery
		uint8_t data[8] = {1,0,0,0,0,0,0,0}; // default state is precharge, only goes into AIR if motorcontrollers ahve finished precharge
		if(carIntact && wantToIgnite && mc_leftLive && mc_rightLive && (mcPrechargeOverThreshold || carPrecharged)){
			data[0] = AIR_MODE;
			carPrecharged = true;
		}else{
			data[0] = PRECHARGE_MODE;
		}
		struct can_message msg;
		msg.id   = 0x8;
		msg.type = CAN_TYPE_DATA;
		msg.data = &data;
		msg.len  = 8;
		msg.fmt  = CAN_FMT_STDID;
		int32_t status = can_async_write(&CAN_0, &msg);
		// relay msg to battery ends
	
		if(carIntact && wantToIgnite && (relayState == PRECHARGE_MODE || carPrecharged)){ 
			gpio_set_pin_level(ASS_PIN, ASS_CLOSED);//ASS pin PIN_PC19 close pin
		}else{
			gpio_set_pin_level(ASS_PIN, ASS_OPEN);
			carPrecharged = false;
		}
		

		if(carIntact && mc_leftLive && mc_rightLive && wantToIgnite && relayState == AIR_MODE)
		{
			//MCLEFT
			if(motorStateLeft.inputmethod != 's'){
				struct motorStream amsg;
				strcpy(amsg.buf, "s\r\n");
				amsg.length = 3;
				xQueueSendToBack(VCUtoMotorL, &amsg, portMAX_DELAY);
			}else{
				if(pedalboard_valid){
					struct motorStream command;
					float desired_thr = pedalboard_thrpos*MAX_THR/100.0;
					//int actual_thr = pedalboard_thrpos*MAX_THR/100;
					if(pedalboard_thrpos <= 0){
						command.buf[0] = '0';
						command.buf[1] = '\0';
						command.length = 1; // important, length of buffer cannot contain the '\0' char in the end
					}else{
						get_thr_cmd(desired_thr, &command);
						command.buf[command.length] = 'r';
						command.buf[command.length+1] = '\0';
						command.length += 1;
						log_info("throttle: %f, command left: %s", desired_thr, command.buf);
					}
					xQueueSendToBack(VCUtoMotorL, &command, portMAX_DELAY);
				}
			}
		
			//MCRIGHT
			if(motorStateRight.inputmethod != 's'){
				struct motorStream amsg;
				strcpy(amsg.buf, "s\r\n");
				amsg.length = 3;
				xQueueSendToBack(VCUtoMotorR, &amsg, portMAX_DELAY);
			}else{
				if(pedalboard_valid){
					struct motorStream command;
					float desired_thr = pedalboard_thrpos*MAX_THR/100.0;
					//int actual_thr = pedalboard_thrpos*MAX_THR/100;
					if(pedalboard_thrpos <= 0){
						command.buf[0] = '0';
						command.buf[1] = '\0';
						command.length = 1; // important, length of buffer cannot contain the '\0' char in the end
					}else{
						get_thr_cmd(desired_thr, &command);
						command.buf[command.length] = 'r';
						command.buf[command.length+1] = '\0';
						command.length += 1;
						log_info("throttle: %f, command right: %s", desired_thr, command.buf);
					}
					xQueueSendToBack(VCUtoMotorR, &command, portMAX_DELAY);
				}
			}
		
		}
		//speaker
		float fr = pedalboard_thrpos/100.0; // alternatively real throttle position percentage
		//pwm_set_parameters(&PWM_0, (uint32_t)period, (uint32_t)(period*perc)); //setting speaker control
		//log_info("fr: %d", fr);
		
		if(brake_press > 5600){
			gpio_set_pin_level(PIN_PC9, 1);
		}else{
			gpio_set_pin_level(PIN_PC9, 0);
		}
	}
}

static void speaker_task(void *p){
	(void)p;
	
	gpio_set_pin_direction(PIN_PA2, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(PIN_PA2, GPIO_PULL_UP);
	gpio_set_pin_function(PIN_PA2, GPIO_PIN_FUNCTION_OFF);
	
	int freq = 0;
	while(1){			
		gpio_set_pin_level(PIN_PA2, 1); // closed, 3.1v, 300mv, 4.5MegaOhm to ground
		vTaskDelay(1000);
		gpio_set_pin_level(PIN_PA2, 0); // open, 0v, 0mv, 335ohm to ground
		vTaskDelay(1000);
	}
	
}
	

extern uint8_t *can0_rx_fifo;
static void setup_can(void) {

	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, CAN_0_rx_callback);
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, CAN_0_tx_callback);
	int32_t status = can_async_enable(&CAN_0);
	
	// get out of stand by mode
	// IMPORTANT: PIN_PD11 may put can chip in stand by mode
	gpio_set_pin_direction(PIN_PD11, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PD11, 0);

	log_debug("enable status: %d", status);
	struct can_filter filter;
	filter.id   = 0;
	filter.mask = 0;
	status = can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
	log_debug("filter status: %d", status);
}

static void do_can_tx(void) {
	static uint32_t counter;

	struct can_message msg;
	uint32_t data = __builtin_bswap32(counter);
	//uint8_t data[8];
	//data[0] = counter + 42;

	msg.id   = 0x246;
	msg.type = CAN_TYPE_DATA;
	msg.data = &data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	
	int32_t status = can_async_write(&CAN_0, &msg);
	//log_info("SENT, message_status: %d", status);
	
	counter++;
}

static void can_task(void *p) {
	(void)p;
	
	log_debug("start can_test");
	setup_can();
	log_debug("setup finished");

	//hri_mcan_set_CCCR_TEST_bit(MCAN0);
	//hri_mcan_set_TEST_LBCK_bit(MCAN0);
	
	
	while (1) {
		
		//do_can_tx();
		vTaskDelay(100);
		
	}
}

static void test_task(void *p) {
	(void)p;

	while (1) {
		vTaskDelay(789);
		float val = sensor_get_f(SENS_PRES_R_OUT);
		uint32_t ts = sensor_get_ts(SENS_PRES_R_OUT);
		log_info("%d %f", ts, val);
		
	}
}

static void read_task(void *p){
	(void)p;
	struct io_descriptor *io;
	usart_os_get_io(&USART_EDBG, &io);
	
	read_buf = xQueueCreate(4, 1);
	
	log_debug("read task started2");
		
	while(1){
		uint8_t charbuf[1];
		io_read(io, charbuf, 1);
		//log_debug("readtask received: %c", charbuf[0]);
		xQueueSendToBack(read_buf, &(charbuf[0]), portMAX_DELAY);
	}
}

static int read_async(){
	uint8_t c = 0;
	xQueueReceive(read_buf, &c, 0);
	if(c == NULL){
		//log_debug("no char");
		return 0;
	}else{
		//log_debug("HASCHAR %u", c);
		return c;
	}
}

int main(void) {
	// TODO don't hardcode address
	hri_matrix_write_CCFG_CAN0_CAN0DMABA_bf(MATRIX, (uint32_t)0x2042);

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	NVIC_SetPriority(HSMCI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USBHS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(MCAN0_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(MCAN0_INT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(UART1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(UART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(AFEC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(AFEC1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

	_pmc_enable_periph_clock(ID_PIOB);

	gpio_set_pin_direction(PIN_PC8, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PC8, 0);
	
	log_init();
	log_debug("startup 3");

	eeprom_emu_init();

	mc_init();
	analog_poll_init();
	
	usb_drive_init();
	usb_console_init();
	usb_start();
	setup_can();
	
	log_debug("finished configs");
	
	//----
	/*
	int pin = PIN_PB3;

	gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
	gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_OFF);

	while (1) {
		int x = gpio_get_pin_level(pin);
		gpio_set_pin_level(PIN_PC8, x);
	}
	*/
	//----
	
	
	//ASS LOOP CLOSED
	gpio_set_pin_direction(ASS_PIN, GPIO_DIRECTION_OUT);// ASS latch pin
	gpio_set_pin_pull_mode(ASS_PIN, GPIO_PULL_UP);
	gpio_set_pin_level(ASS_PIN, ASS_OPEN); // 0 is open 1 is closed
	//ASS LOOP CODE FINISHED
	
	//brake light
	gpio_set_pin_direction(PIN_PC9, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(PIN_PC9, GPIO_PULL_UP);
	gpio_set_pin_level(PIN_PC9, 1); // 0 is open 1 is closed	
	
	//pwm pin is set in driver_init.c, now the pin is PIN PA2
	//pwm_enable(&PWM_0);
	//pwm_set_parameters(&PWM_0, (uint32_t)333, (uint32_t)150);
	//pwm ends
	
	xTaskCreate(can_task, "can_task", 1024, NULL, 1, NULL);
	xTaskCreate(read_task, "read_task", 1024, NULL, 1, NULL);
	//xTaskCreate(latchtask, "latchtask", 1024, NULL, 1, NULL); // just for testing latching
	
	//xTaskCreate(motortest_task, "motortest_task", 1024, NULL, 1, NULL);
	xTaskCreate(input_all_task, "input_all_task", 1024, NULL, 1, NULL);
	xTaskCreate(VCU_task, "VCU_task", 1024, NULL, 1, NULL);
	xTaskCreate(motorControlLeft, "motorControlLeft", 1024, NULL, 1, NULL);
	xTaskCreate(motorControlRight, "motorControlRight", 1024, NULL, 1, NULL);
	xTaskCreate(speaker_task, "speaker_task", 1024, NULL, 1, NULL);



	vTaskStartScheduler();

	while (1);
}
