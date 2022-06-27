#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define MC_BUF_SIZE 256

struct mc_cmd_t {
	enum {
		MC_WAKE,
		MC_ENABLE,
		MC_DISABLE,
		MC_PASSTHRU_ON,
		MC_PASSTHRU_OFF,
		MC_THROTTLE,
	} op;
	int d;
};

struct mc_inst_t {
	volatile bool enabled;
	volatile bool passthru;

	QueueHandle_t cmd_q;
	TaskHandle_t task;

	// uart

	SemaphoreHandle_t uart_sem;

	struct _usart_async_device *uart;

	volatile uint8_t *tx_ptr;
	volatile int tx_cnt;
	SemaphoreHandle_t tx_sem;

	volatile int rx_rdy;
	struct ringbuffer rb;
	uint8_t buffer[MC_BUF_SIZE];
};

void mc_init(void);

struct mc_inst_t *mc_get_inst(unsigned int i);

int mc_uart_read(struct mc_inst_t *mc, uint8_t *dst, int size);
void mc_uart_write(struct mc_inst_t *mc, uint8_t *src, int count);

void mc_passthru_enable(struct mc_inst_t *mc);
void mc_passthru_disable(struct mc_inst_t *mc);

#endif
