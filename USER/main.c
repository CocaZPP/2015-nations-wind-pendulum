#include "stm32f10x.h"
#include "delay.h"
#include "led.h"

#include "motor_control.h"
#include "motor_pwm.h"
#include "motor_pid.h"
#include "chuanjiPID.h"
#include "stdlib.h"
#include "mpu6050.h"
#include "math.h"
#include "indkey.h"
#include "lcd12864.h"
/*------------------------------------------
 				全局变量				
------------------------------------------*/ 
extern uint8_t Q1_Start;
extern uint8_t Q2_Start;
extern uint8_t Q3_Start;
extern uint8_t Q4_Start;
extern uint8_t Q5_Start;
extern uint8_t Q6_Start;
extern uint8_t CurMode; 
/*-----------------------------------------
			    初始化外设
------------------------------------------*/ 


void BSP_Init(void)
{
	DelayInit(); 
	
	Key_IO_Init();
	PID_M1_Init();
	PID_M2_Init();
	PID_M1RA_Init();
	PID_M2PA_Init();
	PID_M1RV_Init();
	PID_M2PV_Init();
	M1TypeDef_Init();
	M2TypeDef_Init();
	PWM_Init();	
	TIM1_Config1(10000-1,71);  /* TIM1 10ms Inturrupt  */
	TIM5_Config1(5000-1,71);   /* TIM5 5ms Inturrupt 采样率200Hz 不能更改*/
	USART1_Config1();
	TIM_Cmd(TIM5,ENABLE);  
	TIM_Cmd(TIM1,ENABLE);
	
  LED_GPIO_Config();
	DelayInit(); 
	LCD_IOinit_OUT();
  LCD_init();	 //初始化与LED连接的硬件接口
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	TIM5_Config1(5000-1,71);   /* TIM5 5ms Inturrupt 采样率200Hz 不能更改*/
	TIM_Cmd(TIM5,ENABLE);  	  //电机控制都在TIM5完成
	

}
/*-----------------------------------------
				主 函 数
------------------------------------------*/ 
int main(void)    
{ 	
BSP_Init();

	
	//角度采样和运动控制都在TIM5_IRQHandler()中完成
	while(1)  
	{  		


// Mode_4();
//   Mode_4();
		if(Q1_Start == 1) 
		{	
		CurMode = 1;
		}
		else if(Q2_Start == 1)
		{ 
		CurMode = 2;
		}
		else if(Q3_Start == 1)
		{
		 	CurMode = 3;
		}
		else if(Q4_Start == 1)
		{
		 	CurMode = 4;
		}
		else if(Q5_Start == 1)
		{
			CurMode = 5;
		}
		else if(Q6_Start == 1)
		{
			CurMode = 6;
		}
		else
		{
			CurMode = 0;
		}	
	}     
} 

