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
#include "bsp_red.h"
/*******************************************************************
 * Prototypes
 *******************************************************************/
 #define assign(x,y,z) vx=x;vy=y;vz=z;vx_o+=x;vy_o+=y;vz_o+=z;
 #define threshould_xy(a,b) ((a>b-7)&&(a<b+7))
 #define threshould_z(a,b)  ((a>b-1)&&(a<b+1))
 #define threshould_s(a,b)  ((a>b-50)&&(a<b+50)) 
 void yigedunzi(void);
 void lianggedunzi(void);
 void END_node(void);
	uint8_t res,rxff=0;
	extern uint8_t key;
	extern uint8_t lpuartrx[8];
	int32_t move_x,move_y,move_z,move_v,t_0,Hei;
	uint16_t run_node=0,roa_node=0;
	extern uint32_t sec;
	uint32_t now_time=0;
	uint32_t bk_tim=700;
	uint8_t dog1=0,dog2=0,dog3=0;
	uint16_t gear0=2000;
	float vx,vy,vz;
	float vx_o=0,vy_o=0,vz_o=0;
	extern uint32_t mm1,mm2;
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	
	extern uint8_t key1,key2,key3;
	float d_z;
	extern uint8_t rxflag,rxflag1,data;
	uint8_t once=1,twice=1,thrice=1,back=1;
	float px,py,pz;
	float
		  DST_X[30]={ -1486  , -2397.0 , -3694.0 , -4119.0 ,-2644.0 , -2993.0 , -1390.0, -458.0,  1550, 1162, 2010,  -70 },
		  DST_Y[30]={ -148.0 ,   464.0 ,  1479.0 ,  2590.0 , 2492.0 ,  2109.0 ,   717.0, 2801.0,  2644, 3263, 5620, 5374},
		  DST_Z[30]={    0   ,    53.5 ,    53.5 ,  -101.0 , 137.2  ,  137.2  ,  -143.0, -136.0, -79.7, -79.7,  99 ,  98 };
		  //XMDZ[2][10] ={{},{}};
	int32_t
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
	uint8_t hha[7]={0x0d,0x0a,0x00,0x00,0x00,0x0a,0x0d};
	uart_Init();
	uint8_t i=0,key_flag=0;
	uint16_t vel=0;		
	Key_IT_GPIO_Config();
	LED_GPIO_Config();
	PIT_CH0_Int_Init(500);
	red_gpio_init();
//		delay_ms(500);	
//	my_canInit();
	//舵机
	hha[2]=0;
	hha[3]=0;
	hha[4]=0;
	LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
	
    while(1) 
    {
			
			if(key1==1)
			{
				PRINTF("t0.txt=\"%d\"",key1 );
				END_SEND();
			}
			
			if(key2==1)
			{
				PRINTF("t1.txt=\"%d\"",key2 );
				END_SEND();
			}
			
			if(key3==1)
			{
				PRINTF("t2.txt=\"%d\"",key3 );
				END_SEND();
			}
			PRINTF("t7.txt=\"%d\"",mm1);
			END_SEND();
		if(rxflag)
		{ 
			updateWD();		
//			PRINTF("t0.txt=\"%f\"",wd_x );
//			END_SEND();
//			PRINTF("t1.txt=\"%f\"",wd_y);
//			END_SEND();
//			PRINTF("t2.txt=\"%f\"",zangle);
//			END_SEND();
//			PRINTF("t7.txt=\"%d\"",run_node);
//			END_SEND();
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
			  hha[2]=0;
			  hha[3]=0;
			  hha[4]=0;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  delay(20);
//			  /* 稍微延时 */
//			  delay(100);
//			  /* 等待至按键被释放 （高电平）*/
//			  if(1 == GPIO_PinRead(CORE_BOARD_WAUP_KEY_GPIO, CORE_BOARD_WAUP_KEY_GPIO_PIN))
//			  {
//				  /* 翻转LED灯，串口输出信息 */
//				CORE_BOARD_LED_TOGGLE;
//				 Hei=130000;
//				change_height(Hei);
//			  }
			  
			  /* 重新设置标志位 */
			  g_KeyDown[CORE_BOARD_WAUP_KEY_ID] = false; 
		  }
		  /* MODE按键的标志 */
		  /* 若g_KeyDown为true表明按键被按下 */
		  if(g_KeyDown[CORE_BOARD_MODE_KEY_ID])
		  {
			  hha[2]=1;
			  hha[3]=1;
			  hha[4]=1;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  delay(20);
//			  delay(100);
//			  if(1 == GPIO_PinRead(CORE_BOARD_MODE_KEY_GPIO, CORE_BOARD_MODE_KEY_GPIO_PIN))
//			  {
//				  CORE_BOARD_LED_TOGGLE;
//				  Hei=220000;
//				  change_height(Hei);
//			  }
			  g_KeyDown[CORE_BOARD_MODE_KEY_ID] = false; 
		 }
		if(rxff==1)
		{
			if(back)
				bbb(DST_X[run_node],DST_Y[run_node]);
			else
				aaa(DST_X[run_node-1],DST_Y[run_node-1]);
			
			switch(run_node)//开始动作
			{
				case 3:
					if(once)
					{
						d_z=DST_Z[run_node ]-zangle;
						if(d_z>0)
						{
							if(abs(d_z)>180)
								vz=-100;
							else
								vz=100;
						}
						else
						{
							if(abs(d_z)>180)
								vz=100;
							else
								vz=-100;
						}
						once=0;
					}
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;		
				case 4:
					if(once)
					{
						d_z=DST_Z[run_node ]-zangle;
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
						change_height(150000);
					}
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;						
					break;
			}	
			
	     	if(once)
					{
						d_z=DST_Z[run_node ]-zangle;
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


			if(!back||(threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node])))
			{
				switch(run_node)//暂停动作
				{
					case 1:
						cloneD();
						END_node();
						break;
					case 4:
						cltwoD();
						delay_ms(200);
						END_node();
						break;
					case 5:
						//dog1=1;
						yigedunzi();
						break;
					case 6:
						//dog1=1;
						yigedunzi();
						break;
					case 7:
						//dog1=1;
						lianggedunzi();
						break;
					case 9:
						//dog2=1;
						lianggedunzi();
						break;
					case 11:
						lianggedunzi();
						break;
					case 12:
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
			//Analysis(vx,vy,vz);
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
			END_node();
		}
	}//回退一点
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(500);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
			delay_ms(100);
			change_height(90000);
			delay_ms(500);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
			delay_ms(100);
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
			END_node();
		}
	}//回退一点
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(500);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
			delay_ms(100);
			change_height(130000);
			delay_ms(500);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
			delay_ms(100);
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

void END_node(void)
{
	once=1;
	twice=1;
	thrice=1;
	back=1;
	dog1=0;
	dog2=0;
	dog3=0;
	run_node++;	
}
	

/****************************END OF FILE**********************/
