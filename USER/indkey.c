#include "stm32f10x.h"
#include "delay.h"
#include "led.h"

#include "motor_control.h"
#include "motor_pwm.h"
#include "motor_pid.h"
#include "stdlib.h"
#include "mpu6050.h"
#include "math.h"
#include "indkey.h"
#include "lcd12864.h"
//#define CMD       (uint32_t)0xf8000000 //���� д���������Ҫ��д��0xf8 
//#define DATE      (uint32_t)0xfa000000 // ���� д������Ҫ��д��0xfa
/*-----------------------------------------
				ȫ�ֱ���
------------------------------------------*/
uint8_t Item = 0;
uint8_t Q1_Start = 0;
uint8_t Q2_Start = 0;
uint8_t Q3_Start = 0;
uint8_t Q4_Start = 0;
uint8_t Q5_Start = 0;
uint8_t Q6_Start = 0;
uint8_t Q7_Start = 0;

extern float R;
extern float angle;
extern uint8_t RoundDir;
extern uint8_t CurMode; 
/*-----------------------------------------
				KEY IO����
------------------------------------------*/
void Key_IO_Init(void)	 //����IO����
{
    GPIO_InitTypeDef IO_Init;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);	
  IO_Init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;				    
  IO_Init.GPIO_Mode = GPIO_Mode_IPU;	
  GPIO_Init(GPIOA, &IO_Init);
}
/*-----------------------------------------------
	��������:	�����������
	��������:	�˿�����GPIOx,������GPIO_Pin
	������ֵ:	INT8U���� ����ֵ0,1
------------------------------------------------*/
void KeyScan(void)
{	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == KEY_PRESSED) //K1
	{	
		switch(Item)
		{
			case 2:R+=5.0;
				   if(R >= 30.0) R = 30.0;
			
					  Display(0x90,"length:");
				    displaylength(R);	
			
			       break;  //��2�ʰ���S4���Ӿ���

			case 3:angle+=10.0;
				   if(angle >= 180.0) 
				   	  angle = 180.0;
				    Display(0x88,"angel:");
					  displayangle(angle);	
			       break;  //��3�ʰ���S4���ӽǶ�;  
			
			case 5:R+=5.0;
				   if(R >= 35.0) R = 35.0;
				   Display(0x98,"radius:");
					 displayradius(R);	
				   break;

			case 6:R+=5.0;
				   if(R >= 35.0) R = 35.0;
			     Display(0x98,"radius:");
					 displayradius(R);	
				   break;
				   

			default:break;
		}				
	}

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == KEY_PRESSED) //K2
	{
		switch(Item)
		{
			case 2:R-=5.0;
				   if(R <= 15.0) R = 15.0;
			 Display(0x90,"length:");
				    displaylength(R);	
			       break;  //��2�ʰ���S4���پ���
			case 3:angle-=10.0;
				   if(angle <= 0.0) 
				   	  angle = 0.0;
			 Display(0x88,"angel:");
					  displayangle(angle);	
			       break;  //��3�ʰ���S4���ٽǶ�
			
			case 5:R-=5.0;
				   if(R <= 15.0) R = 15.0;
				Display(0x98,"radius:");
					 displayradius(R);	
				   break;

			case 6:R-=5.0;
				   if(R <= 15.0) R = 15.0;
			Display(0x98,"radius:");
					 displayradius(R);	
				   break;
				   

			default:break;
		}		
	}
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == KEY_PRESSED) //K3��������
	{
		switch(Item)
		{
			case 1:Q1_Start = 1;
//			  LCD_Write(CMD,0x01);
				   	Display(0x80,"startT1");
				   break; 

			case 2:Q2_Start = 1;
//				  LCD_Write(CMD,0x01);
				Display(0x80,"startT2");
				   break;

			case 3:Q3_Start = 1;
//				  LCD_Write(CMD,0x01);
				Display(0x80,"startT3");
				   break;  

			case 4:Q4_Start = 1;
//				  LCD_Write(CMD,0x01);
				Display(0x80,"startT4");
			      break;
				   
			case 5:Q5_Start = 1;
				   RoundDir = !RoundDir;
				   if(RoundDir == 1)
					 {
//						 	  LCD_Write(CMD,0x01);
					Display(0x80,"startR5");
					 }
				   else
						 {
//							 	  LCD_Write(CMD,0x01);
					     Display(0x80,"startL5");
					   }
				   	
				  
					 break;
				   
			case 6:Q6_Start = 1;	       
//			    	  LCD_Write(CMD,0x01); 
				  Display(0x80,"startT6");
					 break;   

			default:break;
		}	
	} 	

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) == KEY_PRESSED)  //K4ѡ��ģʽ
	{
		Item++;
		if(Item>6)		//����7��
			Item = 0;
		Display(0x86,"T");
		diplay_num(0x87,Item);
		
	}
}

void Display_Title(void)
{
Display(0x80,"Windpendulum");
}


