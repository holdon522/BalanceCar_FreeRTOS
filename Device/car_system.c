#include "main.h"
#include "car_system.h"
#include "adc.h"
#include "oled.h"
#include "cartask.h"
#include "control.h"
#include "math.h"
/***************************************************************************************************************
*������:Get_battery_volt()
*����:��ȡADCԭʼֵ������ԭʼֵ����������ص�ѹ
*�β�:��
*����ֵ:����ת����ĵ�ص�ѹ,��λ mv 
****************************************************************************************************************/
int Get_battery_volt(void)   
{  
	int  Volt,i,sum=0;//��ص�ѹ
	for(i = 0;i<3;i++)
	{
		HAL_ADC_Start(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1,100);
		
		Volt = HAL_ADC_GetValue(&hadc1)*3300*5/4096;   //�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
		
		sum += Volt;
	}
	
	
	Volt = sum/3;
	//printf("Volt = %d\n",Volt);
		
	return Volt;
}


/***************************************************************************************************************
*������:Task_State()
*����:��ʾС��������״̬
*�β�:��
*����ֵ:��
****************************************************************************************************************/


void Task_State(void)
{
	int voltage;
	
//	switch(FS_MODE)
//	{
//		case 0:              //ң��ģʽ�� R����˸
//		  Led_Contrl(2);
//			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
//			break;
//		
//		case 1:
//			Led_Contrl(4);    //����ģʽ�� B����˸
//			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//			break;
//		
//		case 2:
//			Led_Contrl(3);     //Ѳ��ģʽ�� G����˸
//			HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
//			break;
//				

//		default:
//			break;
//	}
	
	
	//��ȡС������
	voltage=Get_battery_volt();
//	if(voltage>=12000)	{power=100;}	                //����ʣ��100%
//	if((voltage<12000)&&(voltage>=11800))	{power=75;}	//����ʣ��75%
//	if((voltage<11800)&&(voltage>=11400)) {power=50;}	//����ʣ��50%
//	if((voltage<11400)&&(voltage>=11200))	{power=25;}	//����ʣ��25%
//	if(voltage<11200){
//			power=0;
//			if(voltage>9000){
//						//��ѹ����
//					BUZZ=1;//������������
//			}
//	}	
}

/***************************************************************************************************************
*������:OLED_Show()
*����:��ʾ�������
*�β�:��
*����ֵ:��
****************************************************************************************************************/
int  Distance=0 ; 
void OLED_Show(void)
{
		OLED_ShowString(1,1,"Mode:");
		if(FS_MODE==REMOTE_MODE){
			OLED_ShowString(1,6,"REMOTE");
		}else if(FS_MODE==ULTRA_AVOID_MODE){
			OLED_ShowString(1,6,"AUTO  ");
		}else{
			OLED_ShowString(1,6,"FOLLOW ");
		}
//		uint8_t power=Get_battery_volt();
		OLED_ShowString(2,1,"distance:");
			OLED_ShowNum(2,10,Distance,3);
//		OLED_ShowString(2,1,"power:");
//		OLED_ShowNum(2,10,power,3);
//		OLED_ShowNum(2,10,power,3);


		OLED_ShowString(3,1,"p:");
		OLED_ShowString(3,7,"r:");
		OLED_ShowString(3,11,"y");
		if(OutMpu.pitch>0){
			OLED_ShowChar(3,3,'+');
			OLED_ShowNum(3,4,OutMpu.pitch,2);
		}else{
			OLED_ShowChar(3,3,'-');
			OLED_ShowNum(3,4,OutMpu.pitch,2);
		}
		OLED_ShowNum(3,9,OutMpu.roll,2);
		OLED_ShowNum(3,13,OutMpu.yaw,2);
		OLED_ShowNum(4,1,OutMpu.gyro_x,2);
		OLED_ShowNum(4,6,OutMpu.gyro_y,2);
		OLED_ShowNum(4,12,OutMpu.gyro_z,2);
		
//		OLED_ShowString(1,1,"l:");
//		OLED_ShowString(1,8,"r:");
//		OLED_ShowNum(1,4,abs(Encoder_left),4);
//		OLED_ShowNum(1,11,abs(Encoder_right),4);
		
//		OLED_ShowNum(2,1,Motor1,4);
//		OLED_ShowNum(2,10,Motor2,4);
}