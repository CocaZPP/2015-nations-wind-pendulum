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
//#define CMD       (uint32_t)0xf8000000 //串行 写入的是命令要先写入0xf8 
//#define DATE      (uint32_t)0xfa000000 // 串行 写入数据要先写入0xfa
/*-----------------------------------------
				全局变量
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
				KEY IO配置
------------------------------------------*/
void Key_IO_Init(void)	 //按键IO配置
{
    GPIO_InitTypeDef IO_Init;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);	
  IO_Init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;				    
  IO_Init.GPIO_Mode = GPIO_Mode_IPU;	
  GPIO_Init(GPIOA, &IO_Init);
}
/*-----------------------------------------------
	函数功能:	独立按键检测
	函数参数:	端口组名GPIOx,引脚名GPIO_Pin
	函数回值:	INT8U类型 按键值0,1
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
			
			       break;  //第2问按下S4增加距离

			case 3:angle+=10.0;
				   if(angle >= 180.0) 
				   	  angle = 180.0;
				    Display(0x88,"angel:");
					  displayangle(angle);	
			       break;  //第3问按下S4增加角度;  
			
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
			       break;  //第2问按下S4减少距离
			case 3:angle-=10.0;
				   if(angle <= 0.0) 
				   	  angle = 0.0;
			 Display(0x88,"angel:");
					  displayangle(angle);	
			       break;  //第3问按下S4减少角度
			
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
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == KEY_PRESSED) //K3启动按键
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

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) == KEY_PRESSED)  //K4选择模式
	{
		Item++;
		if(Item>6)		//共有7问
			Item = 0;
		Display(0x86,"T");
		diplay_num(0x87,Item);
		
	}
}

void Display_Title(void)
{
Display(0x80,"Windpendulum");
}


