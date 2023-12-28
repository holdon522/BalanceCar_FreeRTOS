#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "control.h"
#include "stdio.h"
#include "cartask.h"

#define Mechanical_balance 0.1

int   Dead_Zone=1;     //�������
int   control_turn=0;                             //ת�����
extern uint8_t Flag_Stop;
//PID���ڲ���
struct pid_arg PID = {
	.Balance_Kp= 36,
	.Balance_Kd=0.2,
	
	.Velocity_Kp= 12,    
	.Velocity_Ki= 0.06,
	.Turn_Kp = 10,
	.Turn_Kd = 0.1,
};

/**************************************************************************************************************
*������:Read_Encoder()
*����:��ȡ������ֵ(����С����ǰǰ�����ٶ�)
*�β�:(u8 TIMX):xΪ������1����2
*����ֵ:��
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
*������:Vertical_Ring_PD()
*����:ֱ����PD����
*�β�:(float Angle):x��ĽǶ�/(float Gyro):x��Ľ��ٶ�
*����ֵ:����PIDת��֮���PWMֵ
**************************************************************************************************************/
//ֱ������PD


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
*������:Vertical_speed_PI()
*���ܣ��ٶȻ�PI����
*�β�:(int encoder_left):���ֱ�����ֵ/(int encoder_right):���������ֵ�ֵ/(float Angle):x��Ƕ�ֵ
*����ֵ:
**************************************************************************************************************/

int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement)
{
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral,Target_Velocity;
		
	//============ң�ؿ��Ʋ���====================//
//	if(FS_MODE!=0)	Movement=30;
//	if(Distance<12&&Distance>2)
//	{
////		flag_UltrasonicWave=1;
//		Movement=0;		
//	}


	Encoder_Least =(encoder_left+encoder_right)-0;    //��ȡ�����ٶ�ƫ��=�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
	Encoder *= 0.8f;																	//һ�׵�ͨ�˲��� ���ϴε��ٶ�ռ85%
	Encoder += Encoder_Least*0.2f;                   //һ�׵�ͨ�˲����� ���ε��ٶ�ռ15% 
	Encoder_Integral +=Encoder;                       //���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement; 
	
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;           //�����޷�
	if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //�����޷�

	Velocity=Encoder*PID.Velocity_Kp+Encoder_Integral*PID.Velocity_Ki;      //�ٶȿ���
	
	
	if(Turn_off(Angle)==1||Flag_Stop==1)   Encoder_Integral=0;            //����رպ��������
	return Velocity;
}


/**************************************************************************************************************
*������:Vertical_turn_PD()
*����:ת��PD
*�β�:��  CCDС��64��ת��CCD����64��ת�� yaw = z����������ֵ
*����ֵ:��
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
*������:PWM_Limiting()
*����:PWM�޷�����
*�β�:��
*����ֵ:��
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
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
//	if(flag==0)                                                                   //��һ��
//	 {
//	      if(abs(encoder_left)+abs(encoder_right)<40)                         //����1��С���ӽ���ֹ
//				count0++;
//        else 
//        count0=0;		
//        if(count0>5)				
//		    flag=1,count0=0; 
//	 } 
//	 if(flag==1)                                                                  //����ڶ���
//	 {
//		    if(++count1>100)       count1=0,flag=0;                                 //��ʱ���ٵȴ�2000ms
//	      if(Acceleration>12000&&(Angle>(-20+Mechanical_balance))&&(Angle<(20+Mechanical_balance)))   //����2��С������0�ȸ���������
//		    flag=2; 
//	 } 
//	 if(flag==2)                                                                  //������
//	 {
//		  if(++count2>50)       count2=0,flag=0;                                   //��ʱ���ٵȴ�1000ms
//	    if(abs(encoder_left+encoder_right)>120)                                 //����3��С������̥��Ϊ�������ﵽ����ת��   
//      {
//				flag=0;                                                                                     
//				return 1;                                                               //��⵽С��������
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
�������ܣ����С���Ƿ񱻷���
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                           //��ֹ���      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+Mechanical_balance)&&Angle<(10+Mechanical_balance)&&encoder_left==0&&encoder_right==0)         //����1��С������0�ȸ�����
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>25)                                          //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(abs(encoder_left)+abs(encoder_right)>5)                //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				flag=0;
				return 1;                                             //��⵽С��������
			}
	 }
	return 0;
}


/**************************************************************************************************************
*������:Turn_off()
*����:�رյ��
*�β�:(const float Angle):x��Ƕ�ֵ
*����ֵ:1:С����ǰ����ֹͣ״̬/0:С����ǰ��������״̬
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
*������:Set_PWM()
*����:���PWM���Ƶ��
*�βΣ�(int motor1):���1��Ӧ��PWMֵ/(int motor2):���2��Ӧ��PWMֵ
*����ֵ:��
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



