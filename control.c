#include "control.h"	
#include "arm_math.h"
#include "bsp_flexcan.h"
#include "fsl_debug_console.h"

extern  float ax,ay,rx,ry,juli,jdc;
extern float pos_x;
extern float pos_y;
extern float zangle;
float wd1_x=0.0,wd1_y=0.0,kx=129.8,bx=0.0,ky=-129.9,by=130.0,wd_x=0,wd_y=0;
uint8_t Flag_Target,Flag_Change;                             //相关标志位
uint8_t temp1;                                               //临时变量
float Voltage_Count,Voltage_All;  //电压采样相关变量
float Gyro_K=0.004;       //陀螺仪比例系数
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER           (1364.0f)  
#define Kxy_mm                (217.2996f)
#define k_Vz									(13.66f)
void updateWD(void)
{
	wd_x=pos_x-(arm_sin_f32(zangle*PI/180)*kx+bx);
	wd_y=pos_y-(arm_cos_f32(zangle*PI/180)*ky+by);
}
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Analysis(float Vx,float Vy,float Vz)
{
	float rad=zangle*PI/180;
	static float Lx=0,Ly=0,Lz=0,dx=0,dy=0;
	static	int32_t Target_1=0,Target_2=0,Target_3=0,TT=0;
	dx=Vx-wd1_x;
	dy=Vy-wd1_y;
	Lx=dx*arm_cos_f32(rad)+dy*arm_sin_f32(rad);
	Ly=dy*arm_cos_f32(rad)-dx*arm_sin_f32(rad); 
	
//	float hg=80.0;
//	int32_t lk=0;
//	lk=hg;
	
	Target_1   = (int32_t)(Lx + Vz);//+gyroz*Gyro_K;
	Target_2   = (int32_t)(-X_PARAMETER*Lx + Y_PARAMETER*Ly + Vz);//+gyroz*Gyro_K;
	Target_3   = (int32_t)(-X_PARAMETER*Lx - Y_PARAMETER*Ly + Vz);//+gyroz*Gyro_K;
	
//	PRINTF("Vx=%f ",Vx);
//	PRINTF("Vy=%f ",Vy);
//	PRINTF("dx=%f ",dx);
//	PRINTF("dy=%f ",dy);
//	PRINTF("COS=%f ",arm_cos_f32(rad));
//	PRINTF("SIN=%f ",arm_sin_f32(rad));
//	PRINTF("Lx=%f ",Lx);
//	PRINTF("Ly=%f ",Ly);
//	PRINTF("Target_1=%d ",Target_1);
//	PRINTF("Target_2=%d ",Target_2);
//	PRINTF("Target_3=%d ",Target_3);
//	PRINTF("lk=%d ",lk);
	
	CAN_RoboModule_DRV_Velocity_Mode(1,5000,Target_1);
	CAN_RoboModule_DRV_Velocity_Mode(2,5000,Target_2);
	CAN_RoboModule_DRV_Velocity_Mode(3,5000,Target_3);
}	 				


void move(int32_t target_1,int32_t target_2,int32_t target_3,int16_t tt)
{
 	int32_t V1=0,V2=0,V3=0;
	static  int32_t Len_Sum1=0,Len_Sum2=0,Len_Sum3=0;
	
	V1=target_1/tt;
	V2=target_2/tt; 
	V3=target_3/tt;
//  	PRINTF("V1:      %d \r\n",V1);  
//		PRINTF("V2:      %d \r\n",V2);  
//		PRINTF("V3:      %d \r\n",V3);  
	
	Len_Sum1+=target_1;
	Len_Sum2+=target_2;
	Len_Sum3+=target_3;
	
		CAN_RoboModule_DRV_Velocity_Position_Mode(1,5000,V1,Len_Sum1);
		//delay_ms(200);
		CAN_RoboModule_DRV_Velocity_Position_Mode(2,5000,V2,Len_Sum2);
		//delay_ms(200);
		CAN_RoboModule_DRV_Velocity_Position_Mode(3,5000,V3,Len_Sum3);
		//delay_ms(200);



//						PRINTF("V1:      %d \r\n",V1);  
//						PRINTF("V2:      %d \r\n",V2);  
//						PRINTF("V3:      %d \r\n",V3); 
//						
//						PRINTF("Len_Sum1:      %d \r\n",Len_Sum1);  
//						PRINTF("Len_Sum2:      %d \r\n",Len_Sum2);  
//						PRINTF("Len_Sum3:      %d \r\n",Len_Sum3); 	
}



/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{ 		   
	  uint32_t temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
