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
 				ȫ�ֱ���				
------------------------------------------*/ 
extern uint8_t Q1_Start;
extern uint8_t Q2_Start;
extern uint8_t Q3_Start;
extern uint8_t Q4_Start;
extern uint8_t Q5_Start;
extern uint8_t Q6_Start;
extern uint8_t CurMode; 
/*-----------------------------------------
			    ��ʼ������
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
	TIM5_Config1(5000-1,71);   /* TIM5 5ms Inturrupt ������200Hz ���ܸ���*/
	USART1_Config1();
	TIM_Cmd(TIM5,ENABLE);  
	TIM_Cmd(TIM1,ENABLE);
	
  LED_GPIO_Config();
	DelayInit(); 
	LCD_IOinit_OUT();
  LCD_init();	 //��ʼ����LED���ӵ�Ӳ���ӿ�
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	TIM5_Config1(5000-1,71);   /* TIM5 5ms Inturrupt ������200Hz ���ܸ���*/
	TIM_Cmd(TIM5,ENABLE);  	  //������ƶ���TIM5���
	

}
/*-----------------------------------------
				�� �� ��
------------------------------------------*/ 
int main(void)    
{ 	
BSP_Init();

	
	//�ǶȲ������˶����ƶ���TIM5_IRQHandler()�����
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

