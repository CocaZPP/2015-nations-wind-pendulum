#ifndef __MPU6050_H
#define __MPU6050_H
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   

void USART1_Config1(void);
void TIM1_Config1(unsigned short int Period,unsigned short int Prescaler);
void TIM5_Config1(unsigned short int Period,unsigned short int Prescaler);
void   Read_angle(void);
 
#endif
