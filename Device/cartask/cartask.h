#ifndef __CARTASK_H
#define __CARTASK_H
#include "main.h"
extern int  FS_MODE  ;                      //0、遥控模式   1、蔽障模式  2、巡线模式 
extern int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID计算的PWM值
extern int  Motor1, Motor2;                  //左右电机PWM值
extern int  Encoder_left, Encoder_right;     //检测速度
extern float Movement ;                       //速度调节  
extern int  Contrl_Turn ;                     //转向调节变量
extern int  Distence ;                       //小车和前方障碍物之间的距离
extern uint8_t   power;                       //定义电池电量
extern int  Distance;
extern float Speed ; 
extern uint8_t Flag_Stop;

#define REMOTE_MODE 0				//蓝牙遥控模式
#define ULTRA_AVOID_MODE 1	//超声波避障模式
#define ULTRA_FOLLOW_MODE 2	//超声波跟随模式

struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //俯仰角
	  float roll;     //翻滚角
	  float yaw;      //偏航角
};

extern struct mpu6050_data OutMpu;

struct tCCD
{
	uint16_t middle;      //中间位置值
	uint16_t threshold;   //像素ad阈值
	uint16_t left;        //左跳变的位置
	uint16_t right;       //右跳变的位置
};

extern struct tCCD  CCD;
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed);
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len);
static void demo_niming_report_senser(  int16_t  acc_x, int16_t  acc_y, int16_t  acc_z,
                                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                        int16_t  mag_x, int16_t  mag_y, int16_t  mag_z);
void demo_run(void);
void Car_Task_System(void);
void Car_Task_Motor(void);
void Car_Task_IMU(void);
void  HC05_Start(void);

#endif
