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

#define RET_CHECK(x) if (x < 0) return x;

#define BUFFSIZE 256
#define MOTOR_STREAM_SIZE 128 // for both read and write
#define MAX_THR 10 // the maximum throttle 0% - 100%

static QueueHandle_t read_buf;
static QueueHandle_t pedalQUEUE;
static QueueHandle_t inputQUEUE;
static QueueHandle_t MotortoVCUL;// Left motor communication
static QueueHandle_t VCUtoMotorL;
static QueueHandle_t MotortoVCUR;// Right motor communication
static QueueHandle_t VCUtoMotorR;
static char *stok; // strtok state

struct motorStream{
	uint8_t buf[MOTOR_STREAM_SIZE];
	int length; // string in buffer length
};

struct motorState{
	char inputmethod;
};

struct inputStruct{
	int thr;
	bool goodThr;
	bool assState;
	struct motorState motorstateL;
	struct motorState motorstateR;
};

struct outputStruct{
	bool SetassState;
	//motorStream SetMotorL;
	struct motorStream SetMotorR;
};


/*
	I/O interrupts, tasks etc
*
*
*
*/
// blink when msg is sent
static void CAN_0_tx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8);
	(void)descr;
}


uint8_t data[8];
// blink and store the msg
static void CAN_0_rx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8); // blink when msg is received
	struct can_message msg;
	msg.data = data;
	can_async_read(descr, &msg);
	
	/*
	char datas[100] = "";
	strcpy(datas, "");
	for(int i = 0; i < msg.len; i++){
		char tmp[3] = "";
		sprintf(tmp, "%02x ", *(msg.data + i));
		strcat(datas, tmp);		
	}
	log_debug("RECEIVED: id: %x, data: %s", msg.id, datas);
	*/
	//TODO: format & send data
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	switch (msg.id)
	{
		case 0x100: // pedal data
			xQueueOverwriteFromISR(pedalQUEUE, (void * )&msg.data, &xHigherPriorityTaskWoken); // pxHigherPriorityTaskWoken needs reevaluation
			if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			break;
		default:
			break;
		
	}	
	/*
	if( xHigherPriorityTaskWoken == pdTRUE )
	{
		portYIELD_FROM_ISR(); // or portEND_SWITCHING_ISR() depending on the port.
	}*/
	//format & send data end
}

/*
Left motor control input & output
controlled through MotortoVCUL, VCUtoMotorL
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


	mc_passthru_enable(mc);
	mc_passthru_enable(mcOther);
	while (1) {
		if ((r = mc_uart_read(mc, readbuf, sizeof(readbuf)-1))) {
			readbuf[r] = '\0';
			//log_info("in read: %s", readbuf);
			strncpy(readStream.buf, readbuf, r+1);
			readStream.length = r+1;
			xQueueSendToBack(MotortoVCUL, &readStream, portMAX_DELAY); // review this portMAX_DELAY use
		} else if (xQueueReceive(VCUtoMotorL, &writeStream, 0) == pdTRUE) {	
			// gotta make sure writebuf ends in a certain way i.e. \r\n\0
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
controlled through MotortoVCUR, VCUtoMotorR
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


	mc_passthru_enable(mc);
	mc_passthru_enable(mcOther);
	while (1) {
		if ((r = mc_uart_read(mc, readbuf, sizeof(readbuf)-1))) {
			readbuf[r] = '\0';
			//log_info("in read: %s", readbuf);
			strncpy(readStream.buf, readbuf, r+1);
			readStream.length = r+1;
			xQueueSendToBack(MotortoVCUR, &readStream, portMAX_DELAY); // review this portMAX_DELAY use
		} else if (xQueueReceive(VCUtoMotorR, &writeStream, 0) == pdTRUE) {	
			// gotta make sure writebuf ends in a certain way i.e. \r\n\0
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

// get the first byte that is not 0
char decodeMotorMSG(struct motorStream ms){
	char c;
	int i = 0;
	while(ms.buf[i] == '0') i++;
	log_debug("first byte is: %c", ms.buf[i]);
	return ms.buf[i];
}

// task to handle all the inputs to VCU module, which output onto output_all_task
static void input_all_task(void *p){
	(void)p;

	pedalQUEUE = xQueueCreate(1, 8*sizeof(uint8_t));
	MotortoVCUL = xQueueCreate(10, sizeof(struct motorStream));
	MotortoVCUR = xQueueCreate(10, sizeof(struct motorStream));
	//VCUtoMotorL = xQueueCreate(10, sizeof(struct motorStream));
	inputQUEUE = xQueueCreate(1, sizeof(struct inputStruct));
	// initial settings
	//motors
	struct motorState motorStateL;
	struct motorState motorStateR;
	motorStateL.inputmethod = 't';
	motorStateR.inputmethod = 't';
	//log_debug("163: %c", motorStateL.inputmethod);
	//pedalboard
	int pedalboard_thrpos = 0;
	bool pedalboard_live = false;
	//ASS state
	bool ASS_closed = false;
	
	while(1){
		//pedal board section
		uint8_t *thr_pos;// = (uint8_t *) malloc (8);//[8];// = {0,0,0,0,0,0,0,0};
		if(xQueueReceive(pedalQUEUE, &(thr_pos), 0) > 0) //if we have data coming through pedalQUEUE
		{
			//log_debug("length: %i, validity: %u, thr_pos: %u, %u, %u, %u, %u, %u, %u", len, thr_pos[0], thr_pos[1], thr_pos[2], thr_pos[3], thr_pos[4], thr_pos[5], thr_pos[6], thr_pos[7]); // check all messages coming through
			pedalboard_live = thr_pos[0] == 1;
			if(pedalboard_live && thr_pos[1] >= 0 && thr_pos[1] <= 100){
				pedalboard_thrpos = thr_pos[1];
			}
		}else{
			//log_debug("unsucessful Pedalboard communication");
		}
		//pedal board section end
		
		//MC left section
		struct motorStream msl;
		bool someMsgl = false;
		while(xQueueReceive(MotortoVCUL, &(msl), 0) > 0){ // data coming from mc left
			someMsgl = true;
			log_debug("Motor left says: %s", msl.buf);
			char m = decodeMotorMSG(msl);
			//log_debug("motormsg %c", m);
			if(m == 't' || m == 'T'){
				motorStateL.inputmethod = 't'; // analogue
			}else if(m == 's' || m == 'S'){
				motorStateL.inputmethod = 's'; // serial
			}else{
				log_info("interesting data coming through motor left");
			}
		}

		//log_debug("motor left in method: %c, somemsgl: %d", motorStateL.inputmethod, someMsgl);
		//MC left section ENDS
		
		//MC right section
		struct motorStream msr;
		bool someMsgr = false;
		while(xQueueReceive(MotortoVCUR, &(msr), 0) > 0){ // data coming through mc right
			someMsgr = true;
			log_debug("Motor Right says: %s", msr.buf);
			char m = decodeMotorMSG(msr);
			//log_debug("motormsg %c", m);
			if(m == 't' || m == 'T'){
				motorStateR.inputmethod = 't'; // mc in analogue mode
			}else if(m == 's' || m == 'S'){
				motorStateR.inputmethod = 's'; //  mc in serial mode
			}else{
				log_info("interesting data coming through motor right");
			}
		}

		//log_debug("motor right in method: %c, somemsgr: %d", motorStateR.inputmethod, someMsgr);
		//MC right section ENDS

		//data sending
		struct inputStruct is;
		is.thr = pedalboard_thrpos;
		is.goodThr = pedalboard_live;
		is.assState = gpio_get_pin_level(PIN_PC19);
		is.motorstateL = motorStateL;
		is.motorstateR = motorStateR;
		//log_debug("thr pos is: %i", pedalboard_thrpos);
		xQueueOverwrite(inputQUEUE, &is);
		vTaskDelay(100);
	}
}

static void VCU_task(void *p){
	VCUtoMotorL = xQueueCreate(10, sizeof(struct motorStream));
	VCUtoMotorR = xQueueCreate(10, sizeof(struct motorStream));

	(void)p;
	struct inputStruct is;
	int pedalboard_thrpos = 0;
	bool pedalboard_valid = false;
	bool assClosed = false;
	char motorStateLeft = "";
	char motorStateRight = "";
	
	while(1){
		//recieving and updating states
		xQueueReceive(inputQUEUE, &is, portMAX_DELAY); // max delay because we want the state to not exceed speed of previous call
		pedalboard_thrpos = is.thr;
		pedalboard_valid = is.goodThr;
		motorStateLeft = is.motorstateL.inputmethod;
		motorStateRight = is.motorstateR.inputmethod;
		//log_info("throttle pos: %d, validity: %u, motor state left: %c",pedalboard_thrpos, pedalboard_valid, motorStateLeft);
		//MCLEFT
		if(motorStateLeft != 's'){
			struct motorStream amsg;
			strcpy(amsg.buf, "s\r\n");
			//amsg.buf = "s\r\n";
			amsg.length = 3;
			xQueueSendToBack(VCUtoMotorL, &amsg, portMAX_DELAY);
		}else{
			if(pedalboard_valid){
				struct motorStream command;
				int actual_thr = pedalboard_thrpos/10;
				if(actual_thr <= 0){
					command.buf[0] = '0';
					command.buf[1] = '\0';
					command.length = 1; // important, length of buffer cannot contain the '\0' char in the end
				}else{
					command.buf[0] = '1';
					for(int i = 0; i < 10-actual_thr; i++){
						command.buf[i+1] = '-';
					}
					command.buf[11-actual_thr] = 'r';
					command.buf[12-actual_thr] = '\0';
					command.length = 12-actual_thr; // important, length of buffer cannot contain the '\0' char in the end
				}
				xQueueSendToBack(VCUtoMotorL, &command, portMAX_DELAY);
			}
		}
		//MCRIGHT
		if(motorStateRight != 's'){
			struct motorStream amsg;
			strcpy(amsg.buf, "s\r\n");
			//amsg.buf = "s\r\n";
			amsg.length = 3;
			xQueueSendToBack(VCUtoMotorR, &amsg, portMAX_DELAY);
		}else{
			if(pedalboard_valid){
				struct motorStream command;
				int actual_thr = pedalboard_thrpos/10;
				if(pedalboard_thrpos <= 0){
					command.buf[0] = '0';
					command.buf[1] = '\0';
					command.length = 1; // important, length of buffer cannot contain the '\0' char in the end
				}else{
					command.buf[0] = '1';
					for(int i = 0; i < 10-actual_thr; i++){
						command.buf[i+1] = '-';
					}
					command.buf[11-actual_thr] = 'r';
					command.buf[12-actual_thr] = '\0';
					command.length = 12-actual_thr; // important, length of buffer cannot contain the '\0' char in the end
				}
				xQueueSendToBack(VCUtoMotorR, &command, portMAX_DELAY);
			}
		}
		//setmotorspeed(pedalboard_thrpos);
		// no vdelay cause inputQueue speed limits
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



//vehicle startup test
static void start_up_test(void){
	// call all Units to make sure they are online
	
	
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
		
		do_can_tx();
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



#define CMD_MC_HELP "mc <num> <command>"
static int cmd_mc(void) {
	int r;
	struct mc_inst_t *mcOther = mc_get_inst(0); // MC_LEFT socket 0, MC_RIGHT socket 1
	struct mc_inst_t *mc = mc_get_inst(1); // MC_LEFT socket 0, MC_RIGHT socket 1
	
	struct io_descriptor *io;
	usart_os_get_io(&USART_EDBG, &io);
	//char *inst = strtok_r(NULL, " ", &stok);
	//if (!inst) return 0;

	char *subcmd = strtok_r(stok, " ", &stok);
	log_debug("subcmd: %s", subcmd);
	if (!subcmd){
		log_debug("in return");
		return 0;
	} else if (!strcmp(subcmd, "passthru")) {
		log_debug("in passthrough");
		static uint8_t buf[128];
		static uint8_t writebuf[128];
		int writel = 0;

		mc_passthru_enable(mc);
		mc_passthru_enable(mcOther);
		
		while (1) {
			if ((r = mc_uart_read(mc, buf, sizeof(buf)-1))) {
				buf[r] = '\0';
				log_info("in read: %s", buf);
				//io_write(io, buf, r);
			} else if ((r = read_async())) {
				RET_CHECK(r);
				//log_just("%c", r);
				/*
				if (r == 'q') { // esc
					//cwrite("\n", 1);
					log_info("session out");
					break;
				} else {
					log_info("write: %c", r);
					mc_uart_write(mc, (uint8_t *)&r, 1);
				}*/
				
				if (r == 'q') { // esc
					//io_write(io, "\n", 1);
					log_info("session out");
					break;
				} else if(r != 13 && r != 10) {
					writebuf[writel] = r;
					writel++;
					log_info("write: %c", r);
					//mc_uart_write(mc, (uint8_t *)&r, 1);
				} else if(r == '\n'){
					//buf[writel] = '\r';
					writebuf[writel] = '\r';
					writebuf[writel+1] = '\n';
					writebuf[writel+2] = '\0';
					log_info("write: ENDOFCMD, %d, %s", writel, writebuf);
					mc_uart_write(mc, (uint8_t *)&writebuf, writel+2);
					writel = 0;
				}
			} else {
				//log_debug("in delay");
				vTaskDelay(1);
			}
		}
		mc_passthru_disable(mc);
		mc_passthru_disable(mcOther);
	} else{
		log_debug("in leftover");
	}

	return 0;
}

static void motortest_task(void *p) {
	(void)p;
	struct io_descriptor *io;
	usart_os_get_io(&USART_EDBG, &io);
	log_debug("motortest task started");
	
	while (1) {
		log_debug("looping iteration");
		int r;
		bool nextlined = false;
		int bufidx = 0;
		uint8_t buf[256];
		while(!nextlined){
			uint8_t c;
			xQueueReceive(read_buf, &c, portMAX_DELAY);
			//log_debug("received for buf: %c", c);
			if(c == '\n'){
				buf[bufidx] = '\0';
				nextlined = true;
			}else{
				buf[bufidx] = c;
				bufidx++;
			}
		}
		log_debug("received: %s", buf);
		char *cmd = strtok_r(buf, " ", &stok);
		if (!cmd) {
		} else if (!strcmp(cmd, "mc")) {
			log_info("mc: %s\n", stok);

			r = cmd_mc();
			RET_CHECK(r);
		} else {
			log_info("unrecognized command: %s\n", cmd);
		}
		//io_write(io, buf, bufidx);
	}
}


static void latchtask(void *p){
	(void)p;
	
	int pin = PIN_PC19;
	gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
	gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
	
	bool state = true;

	while(1){
		gpio_set_pin_level(pin, state);
		state = !state;
		vTaskDelay(1000);
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
	
	
	xTaskCreate(can_task, "can_task", 1024, NULL, 1, NULL);
	xTaskCreate(read_task, "read_task", 1024, NULL, 1, NULL);
	//xTaskCreate(latchtask, "latchtask", 1024, NULL, 1, NULL); // just for testing latching
	
	//xTaskCreate(motortest_task, "motortest_task", 1024, NULL, 1, NULL);
	xTaskCreate(input_all_task, "input_all_task", 1024, NULL, 1, NULL);
	xTaskCreate(VCU_task, "VCU_task", 1024, NULL, 1, NULL);
	xTaskCreate(motorControlLeft, "motorControlLeft", 1024, NULL, 1, NULL);
	xTaskCreate(motorControlRight, "motorControlRight", 1024, NULL, 1, NULL);

	vTaskStartScheduler();

	while (1);
}
