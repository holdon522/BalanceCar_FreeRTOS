#include "main.h"
#include "car_system.h"
#include "adc.h"
#include "oled.h"
#include "cartask.h"
#include "control.h"
#include "math.h"
/***************************************************************************************************************
*函数名:Get_battery_volt()
*功能:获取ADC原始值，并对原始值做处理代表电池电压
*形参:无
*返回值:经过转换后的电池电压,单位 mv 
****************************************************************************************************************/
int Get_battery_volt(void)   
{  
	int  Volt,i,sum=0;//电池电压
	for(i = 0;i<3;i++)
	{
		HAL_ADC_Start(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1,100);
		
		Volt = HAL_ADC_GetValue(&hadc1)*3300*5/4096;   //电阻分压，具体根据原理图简单分析可以得到	
		
		sum += Volt;
	}
	
	
	Volt = sum/3;
	//printf("Volt = %d\n",Volt);
		
	return Volt;
}


/***************************************************************************************************************
*函数名:Task_State()
*功能:显示小车的运行状态
*形参:无
*返回值:无
****************************************************************************************************************/


void Task_State(void)
{
	int voltage;
	
//	switch(FS_MODE)
//	{
//		case 0:              //遥控模式， R灯闪烁
//		  Led_Contrl(2);
//			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
//			break;
//		
//		case 1:
//			Led_Contrl(4);    //蔽障模式， B灯闪烁
//			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//			break;
//		
//		case 2:
//			Led_Contrl(3);     //巡线模式， G灯闪烁
//			HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
//			break;
//				

//		default:
//			break;
//	}
	
	
	//获取小车电量
	voltage=Get_battery_volt();
//	if(voltage>=12000)	{power=100;}	                //电量剩余100%
//	if((voltage<12000)&&(voltage>=11800))	{power=75;}	//电量剩余75%
//	if((voltage<11800)&&(voltage>=11400)) {power=50;}	//电量剩余50%
//	if((voltage<11400)&&(voltage>=11200))	{power=25;}	//电量剩余25%
//	if(voltage<11200){
//			power=0;
//			if(voltage>9000){
//						//低压报警
//					BUZZ=1;//蜂鸣器哔哔响
//			}
//	}	
}

/***************************************************************************************************************
*函数名:OLED_Show()
*功能:显示各项参数
*形参:无
*返回值:无
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