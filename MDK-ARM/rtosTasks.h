#include "stm32f4xx_hal.h"
#ifndef _RTOSTASKS_H_
#define _RTOSTASKS_H_

#define MAX_BUF_SIZE		 3072
#define TX_RX_MAX_BUF_SIZE	2048
#define TX_BUF	100
#define RX_BUF	100


#define MOTOR_INIT		0
#define MOTOR_START		1
#define MOTOR_STOP		2
#define MOTOR_DO_NOTHING 3
#define MOTOR_QUEUE_WAIT 4
#define SCAN			5

#define MOTOR_LEFT		0U
#define MOTOR_RIGHT		1U
#define MOTOR_ENABLE	1
#define MOTOR_DISABLE	0


#define MOTOR_PORT		GPIOC
#define MOTOR_PIN		GPIO_PIN_6

#define MOTOR_DIR_PORT	GPIOC
#define MOTOR_DIR_PIN	GPIO_PIN_5

#define MOTOR_ENA_PORT	GPIOC
#define MOTOR_ENA_PIN	GPIO_PIN_9

#define NUMBER_OF_CYCLES 1

#ifndef MOTOR_PORT

	#error Missing MOTOR_PORT: YOu need to include MOTOR_PORT in rtosTask.h before use
#endif

#ifndef MOTOR_PIN
	#error Missing MOTOR_PIN: YOu need to include MOTOR_PIN in rtosTask..h before use
#endif
void motorTask(void * pvParameters);
void SetMotorDirection(uint8_t DIR);
void motorInit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void step(void);



void cs_sel(void);
void cs_desel(void);
uint8_t spi_rb(void);
void spi_wb(uint8_t byte);
void MainTask(void * pvParameters);
void msgBufor(uint8_t* msg, uint16_t length);
uint16_t ASCII_convert(uint16_t ascii);
void GetAndSend(void);
//ZMIANY
void motorSet(uint8_t mStatus, uint16_t count);
uint8_t motorGet();

void setMotorEnable(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState GPIO_status);
#endif
