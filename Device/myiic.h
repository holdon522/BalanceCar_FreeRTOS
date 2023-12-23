/**
 ****************************************************************************************************
 * @file        atk_ms6050_iic.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS6050ģ��IIC�ӿ���������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __MYIIC_H
#define __MYIIC_H

#include "sys.h"

/* ���Ŷ��� */
#define ATK_MS6050_IIC_SCL_GPIO_PORT            GPIOA
#define ATK_MS6050_IIC_SCL_GPIO_PIN             GPIO_PIN_4
#define ATK_MS6050_IIC_SCL_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define ATK_MS6050_IIC_SDA_GPIO_PORT            GPIOA
#define ATK_MS6050_IIC_SDA_GPIO_PIN             GPIO_PIN_5
#define ATK_MS6050_IIC_SDA_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/* IO���� */
#define ATK_MS6050_IIC_SCL(x)                   do{ x ?                                                                                             \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SCL_GPIO_PORT, ATK_MS6050_IIC_SCL_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SCL_GPIO_PORT, ATK_MS6050_IIC_SCL_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define ATK_MS6050_IIC_SDA(x)                   do{ x ?                                                                                             \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define ATK_MS6050_IIC_READ_SDA()               HAL_GPIO_ReadPin(ATK_MS6050_IIC_SDA_GPIO_PORT, ATK_MS6050_IIC_SDA_GPIO_PIN)

/* �������� */
void IIC_Start(void);                /* ����IIC��ʼ�ź� */
void IIC_Stop(void);                 /* ����IICֹͣ�ź� */
uint8_t IIC_Wait_Ack(void);          /* �ȴ�IICӦ���ź� */
void IIC_Ack(void);                  /* ����ACKӦ���ź� */
void IIC_NAck(void);                 /* ������ACKӦ���ź� */
void IIC_Send_Byte(uint8_t dat);     /* IIC����һ���ֽ� */
uint8_t IIC_Read_Byte(uint8_t ack);  /* IIC����һ���ֽ� */
void IIC_Init(void);                 /* ��ʼ��IIC�ӿ� */

#endif
