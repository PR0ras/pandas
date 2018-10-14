#include "bsp_uart.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
#include "bsp_pit.h"
#define DEMO_LPUART_IRQHandler LPUART5_IRQHandler

uint8_t g_tipString[] ="Lpuart";
uint8_t datainx = 0,rxindex = 0,rxflag = 0,rxflag1 = 0;		
uint8_t lpuartrx[8];
uint8_t data=0;

float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;

void uart_Init(void)
{
	//BLUE LED Init
	gpio_pin_config_t led_config;
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,0x10B0);
	led_config.direction=kGPIO_DigitalOutput;	//输出
	led_config.interruptMode=kGPIO_NoIntmode;	//不使用中断功能
	led_config.outputLogic=1;					//默认高电平，LED灯关闭
	GPIO_PinInit(GPIO1,9,&led_config); 
	//digitalLo(GPIO1,9);
	
	lpuart_config_t config;
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200U;
	config.enableTx = true;
	config.enableRx = true;
	config.txFifoWatermark = 0;
	config.rxFifoWatermark = 0;
	LPUART_Init(LPUART5, &config, BOARD_DEBUG_UART_CLK_FREQ);
	LPUART_Init(LPUART3, &config, BOARD_DEBUG_UART_CLK_FREQ);
	//LPUART_WriteBlocking(LPUART5, g_tipString, sizeof(g_tipString)-1);
	LPUART_EnableInterrupts(LPUART5, kLPUART_RxDataRegFullInterruptEnable);
	RT1052_NVIC_SetPriority(LPUART5_IRQn,5,0);
	EnableIRQ(LPUART5_IRQn);
	//LPUART_EnableInterrupts(LPUART1, kLPUART_RxDataRegFullInterruptEnable);
	//RT1052_NVIC_SetPriority(LPUART1_IRQn,5,0);
	//EnableIRQ(LPUART1_IRQn);
}

void DEMO_LPUART_IRQHandler(void)
{
	/* If new data arrived. */

	static union
	{
		uint8_t data[24];
		float ActVal[6];
	}posture;

	if ((kLPUART_RxDataRegFullFlag)&&LPUART_GetStatusFlags(LPUART5))
	{
		data=LPUART_ReadByte(LPUART5);
	//LPUART_WriteByte(LPUART1,data);
		//digitalToggle(GPIO1,9);
		switch(rxindex)
		{
			case 0:
				if(data == 0x0d)
					rxindex++;
				else
					rxindex = 0;
				break;
			case 1:
				if(data == 0x0a)
				{
					datainx = 0;//数组索引
					rxindex++;//数据索引
				}
				else if(data == 0x0d)
					rxindex = 1;
				else
					rxindex = 0;
				break;
			case 2:
				posture.data[datainx++]=data;
				if(datainx >= 24)
				{
					datainx = 0;
					rxindex++;
				}
				break;
			case 3:
				if(data == 0x0a)
					rxindex++;
				else
					rxindex = 0;
				break; 
			case 4:
				if(data == 0x0d)
				{
					zangle=posture.ActVal[0];
					xangle=posture.ActVal[1];
					yangle=posture.ActVal[2];
					pos_x =posture.ActVal[3];
					pos_y =posture.ActVal[4];
					w_z   =posture.ActVal[5];
					rxflag=1;
					digitalLo(GPIO1,9);
				}
				rxindex=0;
				break; 
			}
		}
}

void LPUART1_IRQHandler(void)
{
//	if ((kLPUART_RxDataRegFullFlag)&&LPUART_GetStatusFlags(LPUART1))
//		data=LPUART_ReadByte(LPUART1);
//	LPUART_WriteByte(LPUART5,data);
//	
//	 lpuartrx[datainx++]=data;
//	if(datainx>=8)
//	{
//		rxflag1=1;
//		datainx=0;
//	}
}

void END_SEND(void)
{
	LPUART_WriteByte(LPUART1,0xff);
	LPUART_WriteByte(LPUART1,0xff);
	LPUART_WriteByte(LPUART1,0xff);
	//delay_ms(5);
}