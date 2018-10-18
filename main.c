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
#define threshould_z(a,b)  ((a>b-0.5)&&(a<b+1))
#define threshould_s(a,b)  ((a>b-50)&&(a<b+50)) 
void yigedunzi(void);
void lianggedunzi(void);
void END_node(void);

uint8_t res,rxff=0;
extern uint8_t key;
uint8_t key1_queren=0,KEY1=0,key2_queren=0,KEY2=0;
extern uint8_t lpuartrx[8];
int32_t move_x,move_y,move_z,move_v,t_0,Hei;
uint16_t run_node=0,roa_node=0;
extern uint32_t sec;
uint32_t now_time=0;
uint32_t bk_tim=700;
uint8_t dog1=0,dog2=0,dog3=0;
float gear0=3000.0;
float vx,vy,vz,vx_old=0,vy_old=0,keyflag=1;
float vx_o=0,vy_o=0,vz_o=0;
extern uint32_t mm1,mm2;
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float xangle;
extern float yangle;
extern float w_z;
extern float wd_x,wd_y;
uint8_t key1,key2,key3;
extern uint8_t rxflag,rxflag1,data;
float d_z;
uint8_t once=1,twice=1,thrice=1,back=1,once1=1;
float px,py,pz;
float
	  DST_X[30]={ -1486  , -2397.0 , -3694.0 , -4119.0 ,-2644.0 , -2993.0 , -1377.0, -458.0,  1550, 1162, 2010,  -70 ,0,-4720},
	  DST_Y[30]={ -148.0 ,   464.0 ,  1479.0 ,  2590.0 , 2492.0 ,  2109.0 ,   716.0, 2801.0,  2644, 3263, 5620, 5374,0,-172},
	  DST_Z[30]={    0   ,    53.5 ,    53.5 ,  -101.0 , 137.2  ,  137.2  ,  -145.0, -158.0, -79.7, -79.7,  99 ,  98 ,0,175};
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
	my_canInit();
	//CAN_RoboModule_DRV_Velocity_Mode(0,5000,0 );
	
	//change_height(195000);
	//舵机
	hha[2]=0;
	hha[3]=0;
	hha[4]=0;
	LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
	LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
	key=0;
    while(1) 
    {
		key1=GPIO_PinRead(GPIO2, 27);
		key2=GPIO_PinRead(GPIO2, 3);
		
		if(!key1)
		{
			if(!key1&&(key1_queren++)>100)
			{
				KEY1=1;
				key1_queren=0;
			}
		}
		else
			key1_queren=0;
		
		if(!key2)
		{
			if(!key2&&(key2_queren++)>100)
			{
				KEY1=1;
				key1_queren=0;
			}
		}
		else
			key2_queren=0;
		
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
			PRINTF("t8.txt=\"%d\"",back);
			END_SEND();
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

			if(key==1)
		  {
			  hha[2]=0;
			  hha[3]=0;
			  hha[4]=0;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  //delay(20);
//			  if(1 == GPIO_PinRead(CORE_BOARD_WAUP_KEY_GPIO, CORE_BOARD_WAUP_KEY_GPIO_PIN))
//			  {
//				  /* 翻转LED灯，串口输出信息 */
//				CORE_BOARD_LED_TOGGLE;
//				 Hei=130000;
//				change_height(Hei);
//			  }
			  
			  /* 重新设置标志位 */
			  key = 0; 
		  }
		  /* MODE按键的标志 */
		  /* 若g_KeyDown为true表明按键被按下 */
		  if(key==2)
		  {
			  hha[2]=1;
			  hha[3]=1;
			  hha[4]=1;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  //delay(20);
//			  if(1 == GPIO_PinRead(CORE_BOARD_MODE_KEY_GPIO, CORE_BOARD_MODE_KEY_GPIO_PIN))
//			  {
//				  CORE_BOARD_LED_TOGGLE;
//				  Hei=220000;
//				  change_height(Hei);
//			  }
			  key = 0; 
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
						change_height(180000);
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
			if(run_node&&once1)
				{
//					vx_old=DST_X[run_node]-DST_X[run_node-1];
//					vy_old=DST_Y[run_node]-DST_Y[run_node-1];
					vx_old=vx;
					vy_old=vy;
					once1=0;
				}
			switch(run_node)
			{
//				case 5:
//					if(!KEY1)
//						{
//							vy=vy_old;
//							vx=vx_old;
//							keyflag=0;
//						}
//					break;
				case 6:
					if(!KEY1)
						{
							vy=vy_old;
							vx=vx_old;
							keyflag=0;
						}
					break;
				case 7:
					if(!KEY1)
						{
							vy=vy_old;
							vx=vx_old;
							keyflag=0;
						}
					break;
				case 9:
					if(!KEY1)
						{
							vy=vy_old;
							vx=vx_old;
							keyflag=0;
						}
					break;
				case 11:
					if(!KEY1)
						{
							vy=vy_old;
							vx=vx_old;
							keyflag=0;
						}
					break;						
			}
			if(!keyflag||!back||(threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node])))
			{
				switch(run_node)//暂停动作
				{
					case 1:
						clthrD();
						END_node();
						break;
					case 3 :
						cltwoD();
						delay_ms(200);
						END_node();
						break;
					case 5:
						dog3=1;
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
								change_height(70000);
								delay_ms(500);
								if(dog1)
									oponeD();
								if(dog2)
									optwoD();
								if(dog3)
									opthrD();
								delay_ms(100);
								change_height(85000);
								delay_ms(500);
								if(dog1)
									cloneD();
								if(dog2)
									cltwoD();
								if(dog3)
									clthrD();
								delay_ms(200);
								change_height(180000);
								delay_ms(100);	
								thrice=0;			
							}
								twice=0;
								vx=0;
								vy=0;
								vz=0;
						}
						break;
					case 6:
						dog3=1;
						yigedunzi();
						break;
					case 7:
						dog2=1;
						lianggedunzi();
						break;
					case 9:
						dog2=1;
						lianggedunzi();
						break;
					case 11:
						dog3=1;
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
						once1=1;
						keyflag=1;
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
			END_node();
		}
	}//回退一点
	else
	{
		if(!twice)
		{
			change_height(70000);
			delay_ms(500);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
			delay_ms(100);
			change_height(85000);
			delay_ms(700);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
			delay_ms(200);
			change_height(180000);
			delay_ms(100);	
			thrice=0;			
		}
		
		if(KEY1)
		{
			twice=0;
			vx=0;
			vy=0;
			vz=0;
		}
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
			change_height(70000);
			delay_ms(500);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
			delay_ms(100);
			change_height(140000);
			delay_ms(700);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
			delay_ms(200);
			change_height(180000);
			delay_ms(300);		
			thrice=0;	
		}
		if(KEY1)
			{
				twice=0;
				vx=0;
				vy=0;
				vz=0;
			}
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
	KEY1=0;
	once1=1;
	keyflag=1;
	run_node++;	
}
	

/****************************END OF FILE**********************/
