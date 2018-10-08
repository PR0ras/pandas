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
 void yigedunzi(void);
 void lianggedunzi(void);
	uint8_t res,rxff=0;
	extern uint8_t key;
	extern uint8_t lpuartrx[8];
	int32_t move_x,move_y,move_z,move_v,t_0,Hei;
	uint16_t run_node=0,roa_node=0;
	extern uint32_t sec;
	uint32_t now_time=0;
	uint32_t bk_tim=700;
	
	uint16_t gear0=1000;
	float vx,vy,vz;
	float vx_o=0,vy_o=0,vz_o=0;
	
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	float d_z;
	extern uint8_t rxflag,rxflag1,data;
	uint8_t once=1,twice=1,thrice=1,back=1;
	float px,py,pz;
	static float
		  DST_X[30]={ -1467   , -2353.0 , -3730.0 , -4093.0 ,-2841.0 ,-2979.0  ,	-1460,	  -455,	 1561,	 1265, 1997,  802,   49},
		  DST_Y[30]={ -307.0  ,  236.0  ,  1285.0 ,  2376.0 , 2063.0 ,  1911.0 ,      451,	  2508,  2737,	 3000, 4071, 5714,5141,},
		  DST_Z[30]={    0    ,    53.0 ,    53  ,  138.0  , 138.0  ,  138.0   ,	-137 ,	 -26  ,   170,    168,  168, -120, -120};
	static int32_t 
		  DST_H[30]={    0    ,    0    ,    0    ,  150000 , 9000 ,  -24 },
		  DST_S[30]={    0    ,    0    ,    0    ,  0 , 3000 ,  3000 };
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
//	vx=0.0;
//	vy=0.0;
//	vz=1000;//2370.0;
	uart_Init();
	uint8_t i=0,key_flag=0;
	uint16_t vel=0;		
	Key_IT_GPIO_Config();
	LED_GPIO_Config();
	PIT_CH0_Int_Init(500);
//		delay_ms(500);	
	my_canInit();
	PIT_StartTimer(PIT,kPIT_Chnl_1);        //打开PIT
    while(1) 
    {
		if(rxflag)
		{ 
			updateWD();
			PRINTF("t0.txt=\"%f\"",wd_x );
			END_SEND();
			PRINTF("t1.txt=\"%f\"",wd_y);
			END_SEND();
			PRINTF("t2.txt=\"%f\"",zangle);
			END_SEND();
			PRINTF("t7.txt=\"%d\"",run_node);
			END_SEND();
//			PRINTF("t8.txt=\"%d\"",back);
//			END_SEND();
//			PRINTF("t9.txt=\"%d\"",now_time);
//			END_SEND();
//			PRINTF("t10.txt=\"%d\"",sec);
//			END_SEND();
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
		if(rxff==1)
		{
			if(back)
				aaa(DST_X[run_node],DST_Y[run_node]);
			else
				aaa(DST_X[run_node-1],DST_Y[run_node-1]);
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
//						d_z=DST_Z[run_node ]-DST_Z[run_node -1];
//						if(d_z>0)
//						{
//							if(abs(d_z)>180)
//								vz=-60;
//							else
//								vz=60;
//						}
//						else
//						{
//							if(abs(d_z)>180)
//								vz=60;
//							else
//								vz=-60;
//						}
						once=0;
						change_height(150000);
					}					
//					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;
					break;
				case 5:
					vz=0;
					break;
				case 10:
					vz=0;
					break;
				case 12:
					vz=0;
					break;
				
				default :
					if(once)
					{
						d_z=DST_Z[run_node ]-DST_Z[run_node -1];
						if(d_z>0)
						{
							if(abs(d_z)>180)
								vz=-60;
							else
								vz=60;
						}
						else
						{
							if(abs(d_z)>180)
								vz=60;
							else
								vz=-60;
						}
						once=0;
					}
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;
					break;
			}
			if(!back||(threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node])))
			{
				switch(run_node)//暂停动作
				{
					case 5:
						yigedunzi();
						break;
					case 6:
						yigedunzi();
						break;
					case 7:
						lianggedunzi();
						break;
					case 9:
						lianggedunzi();
						break;
					case 12:
						lianggedunzi();
						break;
					case 13:
						back=0;
						once=1;
						twice=1;
						thrice=1;
						vx=0;
						vy=0;
						vz=0;
						break;
					default :
						back=1;
						once=1;
						twice=1;
						thrice=1;
						run_node++;
						break;
				}
			}
			Analysis(vx,vy,vz);
			rxff=0;
		}
	}
		
}
void yigedunzi(void)
{
	if(!thrice)
	{
		if(!twice)
		{
			sec=0;
			now_time = sec;
			twice=1;
			back=0;
		}
		else if(threshould_s(sec,now_time + bk_tim))//延时500ms
		{
			once=1;
			twice=1;
			thrice=1;
			back=1;
			run_node++;	
		}
	}//回退一点
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(500);
			change_height(90000);
			delay_ms(500);
			change_height(150000);
			delay_ms(100);	
			thrice=0;	
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

void lianggedunzi()
{
	if(!thrice)
	{
		if(!twice)
		{
			sec=0;
			now_time = sec;
			twice=1;
			back=0;
		}
		else if(threshould_s(sec,now_time + bk_tim))//延时500ms
		{
		once=1;
		twice=1;
		thrice=1;
		back=1;
		run_node++;	
		}
	}//回退一点
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(500);
			change_height(130000);
			delay_ms(500);
			change_height(150000);
			delay_ms(100);		
			thrice=0;	
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

/****************************END OF FILE**********************/
