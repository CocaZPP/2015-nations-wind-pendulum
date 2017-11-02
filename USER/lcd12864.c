#include "lcd12864.h"
#include "delay.h"
#include "led.h"

#define LCD_IO  GPIOE //���õ��� E.2 E.3 E.4 E.5 E.6 
#define CS      GPIO_Pin_0  //rs
#define RW      GPIO_Pin_1   //wr
#define CLK     GPIO_Pin_2    //en
#define PSB     GPIO_Pin_3    //rd
#define RST     GPIO_Pin_4         //
#define SET(n)    GPIO_SetBits(GPIOE,n) //����Ӧ�ܽ�����ߵ�ƽ 
#define RESET(n)  GPIO_ResetBits(GPIOE,n)//����͵�ƽ  
#define CMD1       (uint32_t)0xf8000000 //���� д���������Ҫ��д��0xf8 
#define DATE1      (uint32_t)0xfa000000 // ���� д������Ҫ��д��0xfa
u8 datas[8];
void LCD_IOinit_OUT() //�������ģʽ ���ܽ����ã�������ͣ��⺯����
{    
	
  GPIO_InitTypeDef GPIO_InitStructure;      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
	
  GPIO_InitStructure.GPIO_Pin =CS|RW|CLK|PSB|RST;     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       
  GPIO_Init(LCD_IO, &GPIO_InitStructure);   
}

void LCD_Write(uint32_t cmd,uint8_t ddata)//LCD д���� 
	{  
		uint32_t temp=cmd;  
		uint32_t i;  RESET(CS); //Ƭѡ����      
		temp|=((uint32_t)(ddata&(uint8_t)0xf0)<<16)+((uint32_t)(ddata&(uint8_t)0x0f)<<12);  
		SET(CS); //Ƭѡ���ߣ���ʼ��������
		for(i=0;i<24;i++)  
		{   
			if(temp&0x80000000)SET(RW); //ȡ�����λ�������1����ôRW��д1   
			else RESET(RW);  //�����0 RW��д0   
			SET(CLK);//��Һ��д����  �����½���д���   
			delay_us(2);//������ʱ   
			RESET(CLK);//���Ͳ����½��أ�д������   
			temp=temp<<1;//����һλ ��д����һλ  
			}
		RESET(CS); //����Ƭѡ��д��������� 
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



void LCD_init()//Һ����ʼ�� 
	{ 
		
		RESET(CS); //����Ƭѡ  
		RESET(PSB);//PSB���ͣ���ʾ�Ǵ��У��������ǲ���  
		RESET(RST);//����RST  
	
		delay_us(100);
		SET(RST);  
		delay_us(40);  
		LCD_Write(CMD1,0x30);//8λ���ݴ���      
		delay_us(40);      
		LCD_Write(CMD1,0x0c);//��ʾ�����α꿪      
		delay_us(40);  
		LCD_Write(CMD1,0x01);//����        
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



