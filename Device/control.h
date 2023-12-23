#ifndef _CONTRIL_H_
#define _CONTRIL_H_

#include "sys.h"



//机械0点
#define Mechanical_balance 0


#define AIN1(PinState)    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, (GPIO_PinState)PinState)
#define AIN2(PinState)    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_15, (GPIO_PinState)PinState)

#define BIN1(PinState)    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_12, (GPIO_PinState)PinState)
#define BIN2(PinState)    HAL_GPIO_WritePin( GPIOB, GPIO_PIN_13, (GPIO_PinState)PinState)

#define PWMA   TIM2->CCR3 
#define PWMB   TIM2->CCR4


extern int Flag_Stop;
extern volatile int Encoder_Left,Encoder_Right;		      //编码器左右速度值
struct pid_arg{
	
	float Balance_Kp;
	float Balance_Ki;
	float Balance_Kd;
	
	float Velocity_Kp;
	float Velocity_Ki;
	float Velocity_Kd;
	
	float  Turn_Kp;
	float  Turn_Ki;
	float  Turn_Kd;

};
extern struct pid_arg PID;

int Read_Encoder(u8 TIMX);
int	Vertical_Ring_PD(float Angle,float Gyro);
int Vertical_speed_PI(int encoder_left,int encoder_right,float Angle,float Movement );
int Vertical_turn_PD(u8 CCD,short yaw);
void Set_Motor(int motor1,int motor2);
void PWM_Limiting(int *motor1,int *motor2);
u8 Turn_off(const float Angle);
void Set_PWM(int motor1,int motor2);


#endif
