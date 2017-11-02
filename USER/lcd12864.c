#include "lcd12864.h"
#include "delay.h"
#include "led.h"

#define LCD_IO  GPIOE //我用的是 E.2 E.3 E.4 E.5 E.6 
#define CS      GPIO_Pin_0  //rs
#define RW      GPIO_Pin_1   //wr
#define CLK     GPIO_Pin_2    //en
#define PSB     GPIO_Pin_3    //rd
#define RST     GPIO_Pin_4         //
#define SET(n)    GPIO_SetBits(GPIOE,n) //将对应管脚输出高电平 
#define RESET(n)  GPIO_ResetBits(GPIOE,n)//输出低电平  
#define CMD1       (uint32_t)0xf8000000 //串行 写入的是命令要先写入0xf8 
#define DATE1      (uint32_t)0xfa000000 // 串行 写入数据要先写入0xfa
u8 datas[8];
void LCD_IOinit_OUT() //推挽输出模式 ，管脚配置，不多解释，库函数有
{    
	
  GPIO_InitTypeDef GPIO_InitStructure;      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
	
  GPIO_InitStructure.GPIO_Pin =CS|RW|CLK|PSB|RST;     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
  GPIO_Init(LCD_IO, &GPIO_InitStructure);   
}

void LCD_Write(uint32_t cmd,uint8_t ddata)//LCD 写函数 
	{  
		uint32_t temp=cmd;  
		uint32_t i;  RESET(CS); //片选拉低      
		temp|=((uint32_t)(ddata&(uint8_t)0xf0)<<16)+((uint32_t)(ddata&(uint8_t)0x0f)<<12);  
		SET(CS); //片选拉高，开始传输数据
		for(i=0;i<24;i++)  
		{   
			if(temp&0x80000000)SET(RW); //取出最高位，如果是1，那么RW就写1   
			else RESET(RW);  //如果是0 RW就写0   
			SET(CLK);//向液晶写数据  是在下降沿写入的   
			delay_us(2);//稍作延时   
			RESET(CLK);//拉低产生下降沿，写入数据   
			temp=temp<<1;//左移一位 ，写入下一位  
			}
		RESET(CS); //拉低片选，写入数据完毕 
}

void Display(uint8_t addr,uint8_t *hz)
{     
  LCD_Write(CMD1,addr);    
  delay_us(3);     
  while(*hz!='\0')    
   {   
     LCD_Write(DATE1,*hz);   
     hz++;   
     delay_us(3);
   }
}



void LCD_init()//液晶初始化 
	{ 
		
		RESET(CS); //拉低片选  
		RESET(PSB);//PSB拉低，表示是串行，拉高则是并行  
		RESET(RST);//拉低RST  
	
		delay_us(100);
		SET(RST);  
		delay_us(40);  
		LCD_Write(CMD1,0x30);//8位数据传输      
		delay_us(40);      
		LCD_Write(CMD1,0x0c);//显示开，游标开      
		delay_us(40);  
		LCD_Write(CMD1,0x01);//清屏        
		delay_us(40);  
	  delay_us(40);  
	}

void displaylength(int temp) 	 
{
  
	datas[0] = temp / 1000;
	datas[1] = temp % 1000 / 100;
    datas[2] = temp % 100 / 10 ;
    datas[3] = temp % 10;
 
  LCD_Write(CMD1,0x94);
	LCD_Write(DATE1,'0'+datas[0]); 
	
		 
 LCD_Write(CMD1,0x95);
	LCD_Write(DATE1,'0'+datas[1]); 
	

	 LCD_Write(CMD1,0x96);
	LCD_Write(DATE1,'0'+datas[2]); 
	
		 
 LCD_Write(CMD1,0x97);
	LCD_Write(DATE1,'0'+datas[3]); 
}
void displayangle(int temp) 	 
{
  
	datas[0] = temp / 1000;
	datas[1] = temp % 1000 / 100;
    datas[2] = temp % 100 / 10 ;
    datas[3] = temp % 10;
 
  LCD_Write(CMD1,0x8c);
	LCD_Write(DATE1,'0'+datas[0]); 
	
		 
 LCD_Write(CMD1,0x8d);
	LCD_Write(DATE1,'0'+datas[1]); 
	

	 LCD_Write(CMD1,0x8e);
	LCD_Write(DATE1,'0'+datas[2]); 
	
		 
 LCD_Write(CMD1,0x8f);
	LCD_Write(DATE1,'0'+datas[3]); 
}
void displayradius(int temp) 	 
{
  
	datas[0] = temp / 1000;
	datas[1] = temp % 1000 / 100;
    datas[2] = temp % 100 / 10 ;
    datas[3] = temp % 10;
 
  LCD_Write(CMD1,0x9c);
	LCD_Write(DATE1,'0'+datas[0]); 
	
		 
 LCD_Write(CMD1,0x9d);
	LCD_Write(DATE1,'0'+datas[1]); 
	

	 LCD_Write(CMD1,0x9e);
	LCD_Write(DATE1,'0'+datas[2]); 
	
		 
 LCD_Write(CMD1,0x9f);
	LCD_Write(DATE1,'0'+datas[3]); 
}

void diplay_num(u8 x,u8 z)
{
	LCD_Write(CMD1,x);
	LCD_Write(DATE1,'0'+z); 

}



