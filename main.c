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
#include "math.h"
/*******************************************************************
 * Prototypes
 *******************************************************************/
 #define assign(x,y,z) vx=x;vy=y;vz=z;vx_o+=x;vy_o+=y;vz_o+=z;
 #define threshould_xy(a,b) ((a>b-7)&&(a<b+7))
 #define threshould_z(a,b)  ((a>b-1)&&(a<b+1))
 #define threshould_s(a,b)  ((a>b-50)&&(a<b+50)) 
 
	uint8_t res,rxff=0;
	extern uint8_t key;
	extern uint8_t lpuartrx[8];
	int32_t move_x,move_y,move_z,move_v,t_0;
	uint16_t run_node=0,roa_node=0;
	extern uint32_t sec;	
	uint16_t gear=2000;
	float vx,vy,vz;
	float vx_o=0,vy_o=0,vz_o=0;
	int32_t height=0,d_height=0,height_o=0;
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	extern uint8_t rxflag,rxflag1,data;
	uint8_t once=1;
	float px,py,pz;
	static float
		  DST_X[100]={ -1376.0 , -2327.0 , -3613.0 , -4064.0 ,-3335 ,-1416 },
		  DST_Y[100]={ -340.0  ,  225.0  ,  1284.0 ,  2369.0 , 1949 ,  487 },
		  DST_Z[100]={    0    ,    53.0 ,    0.0  ,  136.0  , -160 ,  -24 };
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
    //PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    /* 在这里添加你的代码^_^. */
	
//		vx=0.0;
//		vy=0.0;
//		vz=1000;//2370.0;
		uart_Init();
		uint8_t i=0,key_flag=0,node_type=0;
		uint16_t vel=0;
		PIT_CH0_Int_Init(500);
//		change_hight(-100000);
		KEY_Init();
//		DUO_Init();
//		delay_ms(1500);	

	my_canInit();

//	PIT_StartTimer(PIT,kPIT_Chnl_1);        //打开PIT
    while(1) 
    {
			if(rxflag)
			{ 
				updateWD();
				//PRINTF("t0.txt=\"%f\"",height);
				PRINTF("t0.txt=\"%f\"",wd_x);
				END_SEND();
				PRINTF("t1.txt=\"%f\"",wd_y);
				END_SEND();
				PRINTF("t2.txt=\"%f\"",zangle);
				END_SEND();
				PRINTF("t7.txt=\"%d\"",run_node);
				END_SEND();
				PRINTF("t8.txt=\"%f\"",DST_X[run_node]);
				END_SEND();
				PRINTF("t9.txt=\"%f\"",DST_Y[run_node]);
				END_SEND();
				PRINTF("t10.txt=\"%f\"",DST_Z[run_node]);
				END_SEND();
				//PRINTF("x= %f ",wd_x);
				//PRINTF("y= %f ",wd_y);
				//PRINTF("z= %f ",zangle);
				//PRINTF("node= %d ",run_node);
				//PRINTF("dX= %f ", vx);
				//PRINTF("dy= %f ", vy);
				
				//PRINTF("\r\n");
				rxflag=0;
				rxff=1;
			}
			if(key==WKUP_PRES)
			{
				height+=10000;
				digitalToggle(GPIO1,9);
				key=0;
			}
			
			if(key==KEY0_PRES)
			{
				height-=10000;
				digitalToggle(GPIO1,9);
				key=0;
			}
		d_height=height-height_o;
		change_hight(d_height);
		height_o=height;
				
//				key=CAN2_Receive_Msg(canbuf);
//			if(key)//接收到有数据
//			{			
//				for(i=0;i<key;i++)
//				{
//					PRINTF("%x ",canbuf[i]);				
//				}
//				PRINTF("\r\n");			
//			}
//	if(threshould_s(sec,2000))
//		run_node=1;
//	if(threshould_s(sec,9000))
//		run_node=2;	
	if(rxff==1)
	{
		
	aaa(DST_X[run_node],DST_Y[run_node]);
		switch(run_node)//需要停止的点
			{
				case 0:
					vz=0;
					break;
				case 2:
					vz=0;
					break;
				default :
					if(once)
					{
						vz=60.0;
						once=0;
					}
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;
					break;
			}
		if(threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node]))
		{
			switch(run_node)//需要暂停的点
			{
//				case 10:
//					if(threshould_s(sec,2000))
//					{
//						once=1;
//						run_node++;
//					}
//						break;
//					vx=0;
//					vy=0;
//					vz=0;
//					break;
				default :
					once=1;
					run_node++;
			}
			
		}
		
//		switch(run_node)
//		{
//			case 0:
//					aaa(-1400.0,-423.0);
//					if(threshould_xy(wd_x,-1400.0f)&&threshould_xy(wd_y,-423.0f))
//					run_node++;
//					break;
//			case 1:
//					aaa(-2370.0,168.0);
//					if(threshould_xy(wd_x,-2370.0)&&threshould_xy(wd_y,168.0))
//					run_node++;
//					break;
//			case 2:
//					vx=0.0;
//					vy=0.0;
//					vz=0.0;
//		//				aaa(-1000.0,1000.0,-60.0);
//		//				if(threshould_xy(wd_x,-1000.0)&&threshould_xy(wd_y,1000.0)&&threshould_z(zangle,90.0))
//		//					run_node++;
//					break;
//			case 3:
//					vx=0.0;
//					vy=0.0;
//					vz=0.0;
//					break;
//		}	
//		switch(roa_node)
//		{
//			case 0:
//				vz=0;
//				if(threshould_z(zangle,0.0f)&&(run_node == 1))
//					roa_node++;
//			case 1:
//				vz=60.0;
//				if(threshould_z(zangle,51.0f))
//					roa_node++;
//			case 2:
//				vz=0;
//			
//		}
			Analysis(vx,vy,vz);
			rxff=0;
		}
	}
		
}
/****************************END OF FILE**********************/
