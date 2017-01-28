#include "rtosTasks.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "wizchip_conf.h"
#include "socket.h"
#include <stdint.h>
	uint8_t bufor[RX_BUF];
	uint16_t motorCycle = 0;
	uint8_t val;
	uint16_t len;
	extern uint8_t status;
	extern SPI_HandleTypeDef hspi2;
	extern UART_HandleTypeDef huart2;
	extern uint32_t diffInCM;
	const uint8_t start[] = {"start"};
	const uint8_t motor_set[] = {"SET"};
	uint16_t PERIOD = 6490; //6490
	uint8_t CycleNumber;
	uint8_t motorEnable = MOTOR_ENABLE;
	uint8_t motorStatus = MOTOR_STOP;
	//QUEUES
	xQueueHandle MotorStepHandle;
	xQueueHandle DistanceHandle;
	portBASE_TYPE MotorQueueStatus;
	portBASE_TYPE DistanceStatus;
	
	
void motorInit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
	
void motorTask(void * pvParameters){
	CycleNumber = NUMBER_OF_CYCLES;
	uint8_t cycleCounter = 0;
	uint8_t motorDir = 0;
	for(;;){
		switch(motorStatus){
			case MOTOR_INIT:
				motorInit(GPIOC, GPIO_PIN_6);
				motorInit(GPIOC, GPIO_PIN_5);
				motorStatus = MOTOR_START;
				cycleCounter = 0;
				motorCycle = 0;
				printf("\nMotorInit");
				break;
			case MOTOR_START:
				SetMotorDirection(MOTOR_RIGHT);
				if(motorCycle<PERIOD/2)
					SetMotorDirection(MOTOR_RIGHT);
				if(motorCycle>=PERIOD/2){
					SetMotorDirection(MOTOR_LEFT);
				}
				step();
				if(motorCycle >= PERIOD){
					motorCycle = 0;
					motorStatus = MOTOR_STOP;
				}
				motorCycle++;
				osDelay(10);
				MotorQueueStatus = xQueueSendToFront( MotorStepHandle, &motorCycle, 0);
//				printf("\nMotorCycle");
				
				break;
			case MOTOR_STOP:
				if(cycleCounter < CycleNumber-1){
					motorStatus = MOTOR_START;
					cycleCounter++;
				}				
				break;
			case MOTOR_DO_NOTHING:
					
				break;
			case MOTOR_QUEUE_WAIT:				
				break;
			case SCAN:
				motorStatus = MOTOR_INIT;
				break;
			default:
				printf("\nSomething went wrong");
			
		}
	}
}


void SetMotorDirection(uint8_t DIR){
	if(DIR == MOTOR_RIGHT) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	if(DIR == MOTOR_LEFT) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

//void WriteMotor(uint8_t motor_ena){
//	if(motor_ena = )
//}

void step(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

void cs_sel(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void cs_desel(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

uint8_t spi_rb(void){
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi2, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void spi_wb(uint8_t byte){
	HAL_SPI_Transmit(&hspi2, &byte, 1, 0xFFFFFFFF); 
}

void MainTask(void * pvParameters){
	const uint8_t data_buf[TX_BUF]= {"TEKST: "};
	
	
    uint8_t bufSize[] = {4, 2, 1, 1};
	//Inicjalizacja funckji w chipie wiznet
	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);	
	wizchip_init(bufSize, bufSize);
	wiz_NetInfo netInfo = { .mac = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05}, 
							.ip  = { 192, 168, 1, 110},
							.sn  = { 255, 255, 255, 0 },
							.gw  = { 192, 168, 1, 100}};

	wizchip_setnetinfo(&netInfo);
	
	uint8_t bchannel_start[] = {0, 0, 0, 0}; /* 0: close, 1: ready, 2: connect*/
	uint8_t ch = 0;
	uint16_t receivedStep;
	uint8_t receivedStepBuf[10];

	for(;;){
		switch(status = getSn_SR(0))
		{
			case SOCK_ESTABLISHED:
				if(motorGet() == MOTOR_DO_NOTHING){ 
					motorSet(MOTOR_INIT, 0);
				}
				if(bchannel_start[ch] == 1){
					uint8_t remoteIP[4];
					uint16_t remotePort;
					getsockopt(ch, SO_DESTIP, remoteIP);
					getsockopt(ch, SO_DESTPORT, &remotePort);
					printf("Connected to %d.%d.%d.%d:%d \n", remoteIP[0],
															remoteIP[1],
															remoteIP[2],
															remoteIP[3],
															remotePort);
					bchannel_start[ch] = 2;
				}
				osDelay(10);
				//CZESC KODU ODPOWIEDZIALNA ZA ODBIERANIE PRZEZ SERWER KOMEND
				//CZESC UZYTA DO TESTOWANIA SKANERA
				if((len = getSn_RX_RSR(ch)) >  0){
					memset(bufor, 0, sizeof(bufor));
					if(len > TX_RX_MAX_BUF_SIZE) len = TX_RX_MAX_BUF_SIZE;					
					recv(ch, (uint8_t *)bufor, len);
					osDelay(20);
					msgBufor(bufor,len);					
					printf("%d\n", len);	
				}
				//WYSYLANIE DANYCH PODCZAS GDY SKANOWANIE JEST AKTYWNE
				if(motorStatus == MOTOR_START)
					GetAndSend();
				break;
			case SOCK_LISTEN:
				printf("socket listening\r\n");
				motorCycle  = 0;
				osDelay(200);
				break;
					
			case SOCK_CLOSE_WAIT:
				printf("socket waiting\r\n");

				motorCycle  = 0;
				disconnect(ch);
				bchannel_start[ch] = 0;				
				break;
			
			case SOCK_CLOSED:
				motorSet(MOTOR_DO_NOTHING, 0);
				printf("socket closed\n");
				bchannel_start[ch] = 0;
				if(!bchannel_start[ch]){
					printf("Server Starting \n\n");
					bchannel_start[ch] = 1;
				}
				if(socket(ch, Sn_MR_TCP, 5000, 0) == 0){
					printf("Failed to create socket.\r\n");
					bchannel_start[ch] = 0;
				} else {
					printf("Entering listen mode\r\n");
					listen(ch);
				}
				break;
			case SOCK_INIT:
				bchannel_start[ch] = 1;
				listen(ch);
			break;

			default:
				printf("Something went wrong\n");
			break;
			
		}
	}
}
uint16_t val2;
uint16_t val1;
extern uint8_t valBuf[10];
uint16_t PERIOD;

void msgBufor(uint8_t* msg, uint16_t length){
	uint16_t counter;
	
	uint8_t msg_flag;
	for(counter = 0; counter < length-2; counter++){
		if(msg[counter] == start[counter]){
			val2++;
				if(val2 == sizeof(start)-1){
					motorStatus = SCAN;
					val2 = 0;
					memset(msg, 0,length);
				}
		}else{
			val2 = 0;	
		}
		if(msg[counter] == motor_set[counter]){
			val1++;
				if(msg[2] == 'T'){
					PERIOD = ASCII_convert(msg[3])*100+ASCII_convert(msg[4])*10+ASCII_convert(msg[5]);
					memset(msg, 0,length);
					break;
				}
		}
	}
}

uint16_t ASCII_convert(uint16_t ascii){
	if(ascii == 48) return 0;
	if(ascii == 49) return 1;
	if(ascii == 50) return 2;
	if(ascii == 51) return 3;
	if(ascii == 52) return 4;
	if(ascii == 53) return 5;
	if(ascii == 54) return 6;
	if(ascii == 55) return 7;
	if(ascii == 56) return 8;
	if(ascii == 57) return 9;
	return '\0';
}

void GetAndSend(void){
	uint16_t receivedStep;
	uint8_t receivedStepBuf[30];
	do{
		MotorQueueStatus = xQueueReceive (MotorStepHandle, &receivedStep, 0);
				if( MotorQueueStatus == pdPASS ){
					memset(receivedStepBuf , 0, sizeof(receivedStepBuf));
					sprintf(receivedStepBuf, "A%06dD%06d", receivedStep,  diffInCM);
					

					send(0, (char *)receivedStepBuf, sizeof(receivedStepBuf));	

					printf("%d%d%d%d, %d\n", (char)receivedStepBuf[0],
											 (char)receivedStepBuf[1],
											 (char)receivedStepBuf[2],
											 (char)receivedStepBuf[3], 
											 receivedStep );
				}else{
			
				}
	}while(motorStatus != MOTOR_START);
}

void motorSet(uint8_t mStatus, uint16_t count){
	motorStatus = mStatus;
	motorCycle = count;
}

uint8_t motorGet(){
	return motorStatus;
}