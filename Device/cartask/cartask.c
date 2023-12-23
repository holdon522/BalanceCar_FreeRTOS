#include "main.h"
#include "oled.h"
#include "cartask.h"
#include "delay.h"
#include "mpu6050.h"
#include "atk_ms6050.h"
#include "atk_ms6050_iic.h"
#include "car_system.h"
#include "atk_ms6050.h"
#include "inv_mpu.h"
#include "stdio.h"
#include "usart.h"
#include "control.h"
struct mpu6050_data OutMpu; 
int  FS_MODE = 0 ;                      //0、遥控模式   1、蔽障模式  2、巡线模式 
int  Balance_Pwm=0,Velocity_Pwm=0,Turn_Pwm=0;        //PID计算的PWM值
int  Motor1=0, Motor2=0;                  //左右电机PWM值
int  Encoder_left=0, Encoder_right=0;     //检测速度
float Movement = 0;                  //速度调节  
int  Contrl_Turn = 84;                //转向调节变量
struct tCCD  CCD;                      //摄像头的数据
uint8_t   power;                       //定义电池电量
#define imu_report 0	//imu上位机上报
/*************************************************************************************************************
*函数名:Task_200HZ()
*功能:运行主频为200Hz的任务
*形参:无
*返回值:无
*************************************************************************************************************/
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    
    if (len > 28)
    {
        return;
    }
    
    send_buf[len+4] = 0;            /* 校验位清零 */
    send_buf[0] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[1] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[2] = fun;              /* 功能字 */
    send_buf[3] = len;              /* 数据长度 */
    for (i=0; i<len; i++)           /* 复制数据 */
    {
        send_buf[4 + i] = dat[i];
    }
    for (i=0; i<(len + 4); i++)     /* 计算校验和 */
    {
        send_buf[len + 4] += send_buf[i];
    }
    
    /* 发送数据 */
    HAL_UART_Transmit(&huart1, send_buf, len + 5, HAL_MAX_DELAY);
}
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed)
{
    uint8_t send_buf[12];
    
    /* 横滚角 */
    send_buf[0] = (rol >> 8) & 0xFF;
    send_buf[1] = rol & 0xFF;
    /* 俯仰角 */
    send_buf[2] = (pit >> 8) & 0xFF;
    send_buf[3] = pit & 0xFF;
    /* 航向角 */
    send_buf[4] = (yaw >> 8) & 0xFF;
    send_buf[5] = yaw & 0xFF;
    /* 飞行高度 */
    send_buf[6] = (alt >> 24) & 0xFF;
    send_buf[7] = (alt >> 16) & 0xFF;
    send_buf[8] = (alt >> 8) & 0xFF;
    send_buf[9] = alt & 0xFF;
    /* 飞行模式 */
    send_buf[10] = fly_mode;
    /* 锁定状态 */
    send_buf[11] = armed;
    
    /* 状态帧的功能字为0x01 */
    demo_usart1_niming_report(0x01, send_buf, 12);
}
static void demo_niming_report_senser(  int16_t  acc_x, int16_t  acc_y, int16_t  acc_z,
                                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                        int16_t  mag_x, int16_t  mag_y, int16_t  mag_z)
{
    uint8_t send_buf[18];
    
    /* x轴上的加速度值 */
    send_buf[0] = (acc_x >> 8) & 0xFF;
    send_buf[1] = acc_x & 0xFF;
    /* y轴上的加速度值 */
    send_buf[2] = (acc_y >> 8) & 0xFF;
    send_buf[3] = acc_y & 0xFF;
    /* z轴上的加速度值 */
    send_buf[4] = (acc_z >> 8) & 0xFF;
    send_buf[5] = acc_z & 0xFF;
    /* x轴上的陀螺仪值 */
    send_buf[6] = (gyro_x >> 8) & 0xFF;
    send_buf[7] = gyro_x & 0xFF;
    /* y轴上的陀螺仪值 */
    send_buf[8] = (gyro_y >> 8) & 0xFF;
    send_buf[9] = gyro_y & 0xFF;
    /* z轴上的陀螺仪值 */
    send_buf[10] = (gyro_z >> 8) & 0xFF;
    send_buf[11] = gyro_z & 0xFF;
    /* x轴上的磁力计值 */
    send_buf[12] = (mag_x >> 8) & 0xFF;
    send_buf[13] = mag_x & 0xFF;
    /* y轴上的磁力计值 */
    send_buf[14] = (mag_y >> 8) & 0xFF;
    send_buf[15] = mag_y & 0xFF;
    /* z轴上的磁力计值 */
    send_buf[16] = (mag_z >> 8) & 0xFF;
    send_buf[17] = mag_z & 0xFF;
    
    /* 传感器的功能字为0x02 */
    demo_usart1_niming_report(0x02, send_buf, 18);
}
//环境数据采集任务
void Car_Task_IMU(void)
{
			static struct mpu6050_data Last_Data;
			uint8_t ret;
//			float pitch, roll, yaw;
//			int16_t acc_x, acc_y, acc_z;
//			int16_t gyr_x, gyr_y, gyr_z;
//			int16_t temp;
			ret  = atk_ms6050_dmp_get_data(&OutMpu.pitch, &OutMpu.roll, &OutMpu.yaw);
			/* 获取ATK-MS6050加速度值 */
			ret += atk_ms6050_get_accelerometer(&OutMpu.acc_x, &OutMpu.acc_y, &OutMpu.acc_z);
			/* 获取ATK-MS6050陀螺仪值 */
			ret += atk_ms6050_get_gyroscope(&OutMpu.gyro_x, &OutMpu.gyro_y, &OutMpu.gyro_z);
			/* 获取ATK-MS6050温度值 */
//			ret += atk_ms6050_get_temperature(&temp);
			
			if (ret == 0)
			{
						Last_Data=OutMpu;
					/* 上传相关数据信息至串口调试助手 */
//					printf("pit: %.2f, rol: %.2f, yaw: %.2f,\n ", pitch, roll, yaw);
//					printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
//					printf("gyr_x: %d, gyr_y: %d, gyr_z: %d,\n", gyr_x, gyr_y, gyr_z);
//					printf("temp: %d\r\n", temp);
					#if imu_report
					demo_niming_report_status((int16_t)(OutMpu.roll * 100), (int16_t)((OutMpu.pitch) * 100), (int16_t)(OutMpu.yaw * 100), 0, 0, 0);
					demo_niming_report_senser(OutMpu.acc_x, OutMpu.acc_y, OutMpu.acc_z, OutMpu.gyro_x, OutMpu.gyro_y, OutMpu.gyro_z, 0, 0, 0);
					#endif
					
			}else{
					OutMpu=Last_Data;
			}
}


/**************************************************************************************************************
*函数名:Task_100HZ()
*功能:运行主频为100Hz的任务
*形参:无
*返回值:无
**************************************************************************************************************/


void Car_Task_Motor(void)
{

		Encoder_left  = -Read_Encoder(1);
		Encoder_right = Read_Encoder(2);
//		printf("Encoder_left:%d,Encoder_right:%d\r\n",Encoder_left,Encoder_right);
		//1、确定直立环PWM
	
//		printf("pitch:%f\r\n",OutMpu.pitch);

		Balance_Pwm = Vertical_Ring_PD(OutMpu.pitch, OutMpu.gyro_y);

	//2、确定速度环PWM
	
	  Velocity_Pwm = Vertical_speed_PI(Encoder_left,Encoder_right,OutMpu.pitch, Movement );
	
	
	//3、确定转向环PWM
	
//	  if(FS_MODE == 0)       //遥控模式
			Turn_Pwm = Vertical_turn_PD(Contrl_Turn, OutMpu.gyro_z);
//		else if(FS_MODE == 1)  //蔽障模式
//		{
//			if(Distance < 20)
//					Turn_Pwm = Vertical_turn_PD(20, OutMpu.gyro_z);
//			else
//				 Turn_Pwm = 0;
//		}
//		else if(FS_MODE == 2)  //巡线模式
//		{
//			   Turn_Pwm = Vertical_turn_PD(CCD.middle, OutMpu.gyro_z);
//		}
	
	//4、确定最终左右电机的PWM
		Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
	  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
//		Motor1 = Balance_Pwm;
//		Motor2 = Balance_Pwm;
		PWM_Limiting(&Motor1,&Motor2);

		if(Turn_off(OutMpu.pitch)==0){
	//输出PWM控制电机
			Set_PWM(Motor1,Motor2);
		}
}

/**************************************************************************************************************
*函数名:Task_100HZ()
*功能:运行主频为100Hz的任务
*形参:无
*返回值:无
**************************************************************************************************************/

void Car_Task_System(void)
{
		OLED_Show();
}

void  HC05_Start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	delay_us(25);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
}
