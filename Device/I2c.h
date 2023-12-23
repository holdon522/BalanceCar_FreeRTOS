#ifndef _IIC_H
#define _IIC_H
#include "sys.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

//IO��������
#define SDA_IN()  {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=8<<20;} //PA5����ģʽ  
#define SDA_OUT() {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=3<<20;} //PA5���ģʽ
//IO����

#define IIC_SCL    PAout(4) 
//IIC SDAΪPC11
#define IIC_SDA    PAout(5)  
//��ȡSDA��Ӧ���ŵĵ�ƽ
#define READ_SDA   PAin(5) 


//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif

