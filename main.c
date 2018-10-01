#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "bsp_flexcan.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_flexpwm.h"
#include "bsp_pit.h"
#include "arm_math.h"
#include "fsl_gpt.h"
#include "control.h"	

/*******************************************************************
 * Prototypes
 *******************************************************************/
	uint8_t cnt=0;
	uint8_t canbuf[8];
	uint8_t rx_canbuf[8];
	uint8_t res,key;
	uint32_t txIdentifier[20];
	uint8_t cmd[]="t1.txt=\"完结\"";
	extern uint8_t lpuartrx[8];
	uint16_t baseLeft_v,  baseRight_v,  baseFront_v;
	int32_t move_x,move_y,move_z,move_v,t_0;
	extern float vx,vy,vz;
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	extern uint8_t rxflag,rxflag1,data;
/*******************************************************************
 * Code
 *******************************************************************/
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    /* 初始化内存保护单元 */
    BOARD_ConfigMPU();
    /* 初始化开发板引脚 */
    BOARD_InitPins();
    /* 初始化开发板时钟 */
    BOARD_BootClockRUN();
    /* 初始化调试串口 */
    BOARD_InitDebugConsole();
    /* 打印系统时钟 */
	PRINTF("\r\n");
    PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    /* 在这里添加你的代码^_^. */
	
		vx=0.0;
		vy=0.0;
		vz=1000;//2370.0;
		uart_Init();
		uint8_t i=0,key_flag=0,node_type=0;
		uint16_t vel=0;
		PIT_CH0_Int_Init(750000);
		KEY_Init();
		//DUO_Init();
		//delay_ms(1500);
		uint8_t frc=0,led1pwmval=0,dir=1;
		//change_hight(0,50000);
		
//		PRINTF("*****欢迎使用 野火i.MX RT1052 开发板*****\r\n");

		flexcanInit();
		CAN_RoboModule_DRV_Reset(0); 
		delay_ms(500);
		CAN_RoboModule_DRV_Mode_Choice(0,Velocity_Mode);
		delay_ms(200);
		//PIT_CH0_Int_Init(750000);
		//CAN_RoboModule_DRV_Velocity_Mode(0,5000,1000);
		
//		CAN_RoboModule_DRV_Velocity_Position_Mode(0,5000,2000,891600);
		//delay_ms(10000);
		//CAN_RoboModule_DRV_Velocity_Position_Mode(0,5000,-2000,0);
//		PRINTF("TEST\r\n");
		
    //flexcanInit();
		Analysis(vx,vy,vz);
		//PIT_StartTimer(PIT,kPIT_Chnl_0);
		//delay_ms(4000);
		//CAN_RoboModule_DRV_Velocity_Mode(0,5000,0);
		//rxflag1=1;
    while(1) 
    {
		//LPUART_WriteByte(LPUART5,0x11);
		
			if(rxflag)
			{ 
				updateWD();
				//PIT_StartTimer(PIT,kPIT_Chnl_0);        //打开PIT			
//				Analysis(vx,vy,vz);
//				Kinematic_Analysis();
				PRINTF("%f ",wd_x);
				PRINTF("%f ",wd_y);
//				PRINTF("pos_x=%f ",pos_x);
//				PRINTF("pos_y=%f ",pos_y);
//				PRINTF("sin=%f ",arm_sin_f32(zangle*PI/180));
//				PRINTF("cos=%f ",arm_cos_f32(zangle*PI/180));
				PRINTF("%f ",zangle);
//				PRINTF("%f,",pos_y);
//				PRINTF("%f,",arm_sin_f32(zangle*PI/180));
//				PRINTF("%f",arm_cos_f32(zangle*PI/180));

				
//				if(zangle>=0&&zangle<=0.5)
//				{
//					PRINTF("COUNT=%d",PIT_GetCurrentTimerCount(PIT,kPIT_Chnl_0));
					PRINTF("\r\n");
//				}
				//PRINTF("Zangle=%f \r\n",zangle);		
				rxflag=0;
				delay_ms(20);
			}
			key=KEY_Scan(0);
//			if(rxflag1)
//			{
//				LPUART_WriteBlocking(LPUART5, lpuartrx, sizeof(lpuartrx) - 1);
//				for(i=0;i<8;i++)
//				{
//					PRINTF("SUCCESS:%c \r\n",lpuartrx[i]);
//					rxflag1=0; 
//				}
//			}
			
			if(key==WKUP_PRES)
			{
				PRINTF("TEST\r\n");
				CAN_RoboModule_DRV_Velocity_Mode(0,5000,2000);
				//change_hight(0,50000);
			}
			
				if(key==KEY0_PRES)
				{
					//change_hight(1,50000);
				}
				
//				key=CAN2_Receive_Msg(canbuf);
//			if(key)//接收到有数据
//			{			
//				for(i=0;i<key;i++)
//				{
//					PRINTF("%x ",canbuf[i]);				
//				}
//				PRINTF("\r\n");			
//			}
		
		}
		
}
/****************************END OF FILE**********************/
