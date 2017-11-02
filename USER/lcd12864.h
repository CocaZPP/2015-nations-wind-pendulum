#ifndef _lcd12864__H
#define _lcd12864__H
#include "sys.h"

#define Line1   0x80//Һ����һ�� 
#define Line2   0x90//Һ���ڶ��� 
#define Line3   0x88//Һ�������� 
#define Line4   0x98//Һ��������

void LCD_IOinit_OUT(void);
void LCD_Write(uint32_t cmd,uint8_t ddata);
void Display(uint8_t addr,uint8_t *hz);
void LCD_init(void);
void diplay_num(u8 x,u8 z);
void displaylength(int temp); 	 
void displayangle(int temp); 	 
void displayradius(int temp);
void diplay_num(u8 x,u8 z);
#endif
