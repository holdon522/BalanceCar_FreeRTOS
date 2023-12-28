#ifndef __CARTASK_H
#define __CARTASK_H
#include "main.h"
extern int  FS_MODE  ;                      //0��ң��ģʽ   1������ģʽ  2��Ѳ��ģʽ 
extern int  Balance_Pwm,Velocity_Pwm,Turn_Pwm;        //PID�����PWMֵ
extern int  Motor1, Motor2;                  //���ҵ��PWMֵ
extern int  Encoder_left, Encoder_right;     //����ٶ�
extern float Movement ;                       //�ٶȵ���  
extern int  Contrl_Turn ;                     //ת����ڱ���
extern int  Distence ;                       //С����ǰ���ϰ���֮��ľ���
extern uint8_t   power;                       //�����ص���
extern int  Distance;
extern float Speed ; 
extern uint8_t Flag_Stop;

#define REMOTE_MODE 0				//����ң��ģʽ
#define ULTRA_AVOID_MODE 1	//����������ģʽ
#define ULTRA_FOLLOW_MODE 2	//����������ģʽ

struct mpu6050_data{
	
		short acc_x;
		short acc_y;
		short acc_z;
		
		short gyro_x;
		short gyro_y;
		short gyro_z;
	
		float pitch;    //������
	  float roll;     //������
	  float yaw;      //ƫ����
};

extern struct mpu6050_data OutMpu;

struct tCCD
{
	uint16_t middle;      //�м�λ��ֵ
	uint16_t threshold;   //����ad��ֵ
	uint16_t left;        //�������λ��
	uint16_t right;       //�������λ��
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
