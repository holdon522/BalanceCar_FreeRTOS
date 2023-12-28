#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "control.h"
#include "stdio.h"
#include "cartask.h"

#define Mechanical_balance 0.1

int   Dead_Zone=1;     //电机死区
int   control_turn=0;                             //转向控制
extern uint8_t Flag_Stop;
//PID调节参数
struct pid_arg PID = {
	.Balance_Kp= 36,
	.Balance_Kd=0.2,
	
	.Velocity_Kp= 12,    
	.Velocity_Ki= 0.06,
	.Turn_Kp = 10,
	.Turn_Kd = 0.1,
};

/**************************************************************************************************************
*函数名:Read_Encoder()
*功能:读取编码器值(当作小车当前前进的速度)
*形参:(u8 TIMX):x为编码器1或者2
*返回值:无
*************************************************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;  
		
   switch(TIMX)
	 {
	   case 1:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
		 case 2:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}



/**************************************************************************************************************
*函数名:Vertical_Ring_PD()
*功能:直立环PD控制
*形参:(float Angle):x轴的角度/(float Gyro):x轴的角速度
*返回值:经过PID转换之后的PWM值
**************************************************************************************************************/
//直立环的PD


int	Vertical_Ring_PD(float Angle,float Gyro)
{
	 float Bias;
	 int balance;
   Bias=Angle-Mechanical_balance;
   balance=PID.Balance_Kp*Bias+ Gyro*PID.Balance_Kd;
//	 printf("balance = %d\r\n",balance);
	 return balance;
		
}

/**************************************************************************************************************
*函数名:Vertical_speed_PI()
*功能；速度环PI控制
*形参:(int encoder_left):左轮编码器值/(int encoder_right):编码器右轮的值/(float Angle):x轴角度值
*返回值:
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement)
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral,Target_Velocity;
		
	//============遥控控制部分====================//
//	if(FS_MODE!=0)	Movement=30;
//	if(Distance<12&&Distance>2)
//	{
////		flag_UltrasonicWave=1;
//		Movement=0;		
//	}


	Encoder_Least =(encoder_left+encoder_right)-0;    //获取最新速度偏差=测量速度（左右编码器之和）-目标速度（此处为零）
	Encoder *= 0.8f;																	//一阶低通滤波器 ，上次的速度占85%
	Encoder += Encoder_Least*0.2f;                   //一阶低通滤波器， 本次的速度占15% 
	Encoder_Integral +=Encoder;                       //积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //积分限幅
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //积分限幅

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //速度控制
	
	
	if(Turn_off(Angle)==1||Flag_Stop==1)   Encoder_Integral=0;            //电机关闭后清除积分
	return Velocity;
}


/**************************************************************************************************************
*函数名:Vertical_turn_PD()
*功能:转向环PD
*形参:无  CCD小于64左转、CCD大于64右转。 yaw = z轴陀螺仪数值
*返回值:无
***************************************************************************************************************/
int Vertical_turn_PD(short CCD,short yaw)
{
		float Turn;     
    float Bias;	  
	  Bias=CCD;
	  Turn=-Bias*PID.Turn_Kp-yaw*PID.Turn_Kd;
	  return Turn;
}



/**************************************************************************************************************
*函数名:PWM_Limiting()
*功能:PWM限幅函数
*形参:无
*返回值:无
***************************************************************************************************************/
void PWM_Limiting(int *motor1,int *motor2)
{
	int Amplitude=4000;
	if(*motor1<-Amplitude) *motor1=-Amplitude;	
	if(*motor1>Amplitude)  *motor1=Amplitude;	
	if(*motor2<-Amplitude) *motor2=-Amplitude;	
	if(*motor2>Amplitude)  *motor2=Amplitude;		
}

/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
//	if(flag==0)                                                                   //第一步
//	 {
//	      if(abs(encoder_left)+abs(encoder_right)<40)                         //条件1，小车接近静止
//				count0++;
//        else 
//        count0=0;		
//        if(count0>5)				
//		    flag=1,count0=0; 
//	 } 
//	 if(flag==1)                                                                  //进入第二步
//	 {
//		    if(++count1>100)       count1=0,flag=0;                                 //超时不再等待2000ms
//	      if(Acceleration>12000&&(Angle>(-20+Mechanical_balance))&&(Angle<(20+Mechanical_balance)))   //条件2，小车是在0度附近被拿起
//		    flag=2; 
//	 } 
//	 if(flag==2)                                                                  //第三步
//	 {
//		  if(++count2>50)       count2=0,flag=0;                                   //超时不再等待1000ms
//	    if(abs(encoder_left+encoder_right)>120)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
//      {
//				flag=0;                                                                                     
//				return 1;                                                               //检测到小车被拿起
//			}
//	 }

		if((abs(encoder_left)+abs(encoder_right)>120)&&(Angle>(-20+Mechanical_balance))&&(Angle<(20+Mechanical_balance))){
			count0++;
		}else{
			count0=0;
		}
		if(count0>10)
			return 1;

	return 0;
}

/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                           //防止误检      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+Mechanical_balance)&&Angle<(10+Mechanical_balance)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>25)                                          //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(abs(encoder_left)+abs(encoder_right)>5)                //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                                             //检测到小车被放下
			}
	 }
	return 0;
}


/**************************************************************************************************************
*函数名:Turn_off()
*功能:关闭电机
*形参:(const float Angle):x轴角度值
*返回值:1:小车当前处于停止状态/0:小车当前处于正常状态
***************************************************************************************************************/
u8 FS_state;

u8 Turn_off(const float Angle)
{
	u8 temp;
	if(fabs(Angle)>40||1==Flag_Stop){
		FS_state=1;
		temp=1;
		AIN2(0),			AIN1(0);
		BIN1(0),			BIN2(0);
	}
	else 
		temp=0;
		FS_state=0;
	return temp;
}

/**************************************************************************************************************
*函数名:Set_PWM()
*功能:输出PWM控制电机
*形参；(int motor1):电机1对应的PWM值/(int motor2):电机2对应的PWM值
*返回值:无
*************************************************************************************************************/


void Set_PWM(int motor1,int motor2)
{
	if(motor1>0)			AIN2(1),			AIN1(0);
	else 	          	AIN2(0),			AIN1(1);
	PWMA=Dead_Zone+(abs(motor1));
	
	
	if(motor2>0)			BIN1(1),			BIN2(0);
	else       		 		BIN1(0),			BIN2(1);
	PWMB=Dead_Zone+(abs(motor2));	
	
//	printf("PWMA = %d\n",PWMA);
//	printf("PWMB = %d\n",PWMB);
}



