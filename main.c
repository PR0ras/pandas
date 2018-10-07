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
#include "bsp_led.h"
#include "bsp_key_it.h"
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
	int32_t move_x,move_y,move_z,move_v,t_0,Hei;
	uint16_t run_node=0,roa_node=0;
	extern uint32_t sec;
	uint32_t now_time;
	uint8_t tim;
	uint16_t gear=1000;
	float vx,vy,vz;
	float vx_o=0,vy_o=0,vz_o=0;
	
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	extern uint8_t rxflag,rxflag1,data;
	uint8_t once=1,twice=1,onceaaa=1;
	float px,py,pz;
	static float
		  DST_X[100]={ -1376.0 , -2327.0 , -3613.0 , -4166.0 ,-3335.0 ,-1416.0 ,	-452,	1244,	1771,	-245},
		  DST_Y[100]={ -340.0  ,  225.0  ,  1284.0 ,  2298.0 , 1949.0 ,  487.0 ,	2528,	3013,	5017,	5382},
		  DST_Z[100]={    0    ,    53.0 ,    0.0  ,  136.0  , -160.0 ,  -24.0 ,	-16 ,	45  ,      0,     60};
	static uint32_t 
		  DST_H[100]={    0    ,    0    ,    0    ,  150000 , 9000 ,  -24 },
		  DST_S[100]={    0    ,    0    ,    0    ,  0 , 3000 ,  3000 };
/*******************************************************************
 * Code
 *******************************************************************/
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
void yigedunzi()
{
if(twice)
	{
		now_time = sec;
	}
	if(threshould_s(sec,now_time + DST_S[4]))
	{
		once=1;
		twice=1;
		run_node++;	
	}
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(1000);
			change_height(90000);
			delay_ms(1000);
			change_height(150000);
			delay_ms(1000);						
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

void lianggedunzi()
{
if(twice)
	{
		now_time = sec;
	}
	if(threshould_s(sec,now_time + DST_S[4]))
	{
		once=1;
		twice=1;
		run_node++;	
	}
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(1000);
			change_height(130000);
			delay_ms(1000);
			change_height(150000);
			delay_ms(1000);						
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

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
		
//		change_hight(-100000);
		Key_IT_GPIO_Config();
		LED_GPIO_Config();
		PIT_CH0_Int_Init(500);
//		DUO_Init();
//		delay_ms(500);	

	my_canInit();

	PIT_StartTimer(PIT,kPIT_Chnl_1);        //打开PIT
    while(1) 
    {
	if(rxflag)
{ 
updateWD();

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
PRINTF("t10.txt=\"%d\"",sec);
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

		if(g_KeyDown[CORE_BOARD_WAUP_KEY_ID])
      {
          /* 稍微延时 */
          delay(100);
          /* 等待至按键被释放 （高电平）*/
          if(1 == GPIO_PinRead(CORE_BOARD_WAUP_KEY_GPIO, CORE_BOARD_WAUP_KEY_GPIO_PIN))
          {
              /* 翻转LED灯，串口输出信息 */
            CORE_BOARD_LED_TOGGLE;
			 Hei=130000;
			change_height(Hei);
          }
          /* 重新设置标志位 */
          g_KeyDown[CORE_BOARD_WAUP_KEY_ID] = false; 
      }
      
      /* MODE按键的标志 */
      /* 若g_KeyDown为true表明按键被按下 */
      if(g_KeyDown[CORE_BOARD_MODE_KEY_ID])
      {
          delay(100);
          if(1 == GPIO_PinRead(CORE_BOARD_MODE_KEY_GPIO, CORE_BOARD_MODE_KEY_GPIO_PIN))
          {
			  CORE_BOARD_LED_TOGGLE;
			  Hei=220000;
			  change_height(Hei);
          }
          g_KeyDown[CORE_BOARD_MODE_KEY_ID] = false; 
	 }
			


				

//	if(threshould_s(sec,2000))
//		run_node=1;
//	if(threshould_s(sec,9000))
//		run_node=2;	
	if(rxff==1)
	{
		aaa(DST_X[run_node],DST_Y[run_node]);
//	if(threshould_z(zangle,DST_Z[run_node]))
//		vz=0;	
//	else if(once)
//	{
//		if(DST_Z[run_node]-zangle>0)
//			vz=60.0;
//		else
//			vz=-60.0;
//		once=0;		
//	}
//	else;
		switch(run_node)//开始动作
			{
				case 0:
					vz=0;
					break;
				case 2:			
					vz=0;
					break;
				case 4:
					if(once)
					{
					vz=60.0;
					once=0;
				    change_height(150000);
					}					
					if(threshould_z(zangle,DST_Z[run_node]))
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
			switch(run_node)//暂停动作
			{
				case 4:
					if(twice)
						{
							now_time = sec;
						}
						if(threshould_s(sec,now_time + DST_S[4]))
						{
							once=1;
							twice=1;
							run_node++;	
						}
						else
						{
							if(!twice)
							{
								change_height(50000);
								delay_ms(1000);
								change_height(90000);
								delay_ms(1000);
								change_height(150000);
								delay_ms(1000);						
							}
							
							vx=0;
							vy=0;
							vz=0;
							twice=0;
						}
					break;
				case 5:
					if(twice)
						{
							now_time = sec;
						}
						if(threshould_s(sec,now_time + DST_S[4]))
						{
							once=1;
							twice=1;
							run_node++;	
						}
						else
						{
							if(!twice)
							{
								change_height(50000);
								delay_ms(1000);
								change_height(90000);
								delay_ms(1000);
								change_height(150000);
								delay_ms(1000);						
							}
							
							vx=0;
							vy=0;
							vz=0;
							twice=0;
						}
					break;
				case 6:
					if(twice)
						{
							now_time = sec;
						}
						if(threshould_s(sec,now_time + DST_S[4]))
						{
							once=1;
							twice=1;
							run_node++;	
						}
						else
						{
							if(!twice)
							{
								change_height(50000);
								delay_ms(1000);
								change_height(130000);
								delay_ms(1000);
								change_height(150000);
								delay_ms(1000);						
							}
							
							vx=0;
							vy=0;
							vz=0;
							twice=0;
						}
					break;
				case 7:
					if(twice)
						{
							now_time = sec;
						}
						if(threshould_s(sec,now_time + DST_S[4]))
						{
							once=1;
							twice=1;
							run_node++;	
						}
						else
						{
							if(!twice)
							{
								change_height(50000);
								delay_ms(1000);
								change_height(130000);
								delay_ms(1000);
								change_height(150000);
								delay_ms(1000);						
							}
							
							vx=0;
							vy=0;
							vz=0;
							twice=0;
						}
					break;
				case 9:
					if(twice)
						{
							now_time = sec;
						}
						if(threshould_s(sec,now_time + DST_S[4]))
						{
							twice=1;
							run_node++;	
						}
						else
						{
							if(!twice)
							{
								change_height(50000);
								delay_ms(1000);
								change_height(130000);
								delay_ms(1000);
								change_height(150000);
								delay_ms(1000);						
							}
							vx=0;
							vy=0;
							vz=0;
							twice=0;
						}
					break;
				default :
					once=1;
					twice=1;
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
