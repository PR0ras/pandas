#include "control.h"	
#include "arm_math.h"
#include "bsp_flexcan.h"
#include "fsl_debug_console.h"

extern  float ax,ay,rx,ry,juli,jdc;
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float vx,vy,vz;
extern uint16_t gear;
float wd1_x=0.0,wd1_y=0.0,kx=129.8,bx=0.0,ky=-129.9,by=130.0,wd_x=0,wd_y=0;
uint8_t Flag_Target,Flag_Change;                             //相关标志位
uint8_t temp1;                                               //临时变量
float Voltage_Count,Voltage_All;  //电压采样相关变量
float Gyro_K=0.004;       //陀螺仪比例系数
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER           (1364.0f)  
#define Kxy_mm                (217.2996f)
#define k_Vz									(20.38f)//(13.66f)
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
	static float Lx=0,Ly=0,Lz=0,dx=0,dy=0,Vx1,Vy1;
	static	int32_t Target_1=0,Target_2=0,Target_3=0;
	//Vx、Vy距离，Vx1、Vy1根据gear所得速度
	if((Vx==0)&&(Vy==0))
	{Vx1=0;Vy1=0;}
	else if((Vx==0)&&(Vy!=0))
	{
		if(Vy>0)
			Vy1=gear;
		else if(Vy<0)
			Vy1=-gear;
	}
	else
	{	
		Vy1=arm_sin_f32(atan(Vy/Vx))*gear;
		Vx1=arm_cos_f32(atan(Vy/Vx))*gear;
		if(Vx<0)
		{
			Vy1=-Vy1;
			Vx1=-Vx1;
		}
	}
	dx=Vx1-wd1_x;
	dy=Vy1-wd1_y;
	Lx=dx*arm_cos_f32(rad)+dy*arm_sin_f32(rad);
	Ly=dy*arm_cos_f32(rad)-dx*arm_sin_f32(rad); 
	
//	float hg=80.0;
//	int32_t lk=0;
//	lk=hg;
	
	Target_1   = (int32_t)(Lx + k_Vz*Vz);//+gyroz*Gyro_K;
	Target_2   = (int32_t)(-X_PARAMETER*Lx + Y_PARAMETER*Ly + k_Vz*Vz);//+gyroz*Gyro_K;
	Target_3   = (int32_t)(-X_PARAMETER*Lx - Y_PARAMETER*Ly + k_Vz*Vz);//+gyroz*Gyro_K;
	
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

void aaa(float x,float y)
{
	static float err_x=0,l_err_x=0,ln_err_x=0,dX=0,
				 err_y=0,l_err_y=0,ln_err_y=0,dY=0;
	static float kp,ki,kd,index;
		kp=2.25;
		ki=0.025;
		kd=0.32;
		if((abs(err_x)+abs(err_y))>200.0)
		{
			index=0.0;
			gear=1000;
		}
		else if(((abs(err_x)+abs(err_y))>50.0)&&((abs(err_x)+abs(err_y))<100.0))
		{
			index=0.0;
			gear=(abs(err_x)+abs(err_y))+500.0;
		}
		else if((abs(err_x)+abs(err_y))<25.0)
		{
			index=1.0;
			gear=(abs(err_x)+abs(err_y))*2;
		}		
		
		err_x=x-wd_x ;               
		ln_err_x+=index*err_x;
		dX=kd*err_x+ki*index*ln_err_x+kd*(err_x-l_err_x);
		l_err_x=err_x;
		
		err_y=y-wd_y;               
		ln_err_y+=index*err_y;		
		dY=kd*err_y+ki*index*ln_err_y+kd*(err_y-l_err_y);		
		l_err_y=err_y;

	    vx=dX;
		vy=dY;
		delay_ms(2);
//		PRINTF("\r\n");
//		PRINTF("err_x= %f \r\n",err_x);
//		PRINTF("ln_err_x= %f \r\n",ln_err_x);
//		PRINTF("dX= %f \r\n",dX);
//		PRINTF("ln_err_y= %f \r\n",ln_err_y);
//		PRINTF("dY= %f \r\n", dY);
//		PRINTF("vz= %f \r\n", vz);
//		PRINTF("\r\n");
}

//void bbb(float z)
//{
//	static float err_z=0,l_err_z=0,ln_err_z=0,dZ=0,index,
//	kp1=5.25,
//	ki1=0.015,
//	kd1=0.2;
//	if(abs(err_z)>200.0)
//		{
//			index=0.0;
//		}
//	else if(abs(err_z)<5.0)
//		{
//			index=1.0;
//		}

//	err_z=z-zangle  ;               
//	ln_err_z+=index*err_z;
//	dZ=kd1*err_z+ki1*index*ln_err_z+kd1*(err_z-l_err_z);
//	l_err_z=err_z;
//}

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
