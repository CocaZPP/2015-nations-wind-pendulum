#include "string.h"
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
float    t;
unsigned char Re_buf[11],temp_buf[11];
unsigned char    Temp[11],sign,counter=0;
float angle0[3],speed[3],accel[3],temperature;//�Ƕ�,���ٶȣ��Ǽ��ٶ�,�¶�

extern M1TypeDef  M1;
extern M2TypeDef  M2;
extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;
extern PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;

uint8_t CurMode = 0;
static uint16_t LED6Cnt = 0;


//���ڽ�������
void USART1_Config1(void)
{
GPIO_InitTypeDef    GPIO_InitStructure;	   //���ڶ˿����ýṹ�����
	USART_InitTypeDef   USART_InitStructure;   //���ڲ������ýṹ�����
NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ�� USART1 ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//�򿪴��ڸ���ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   //��PA�˿�ʱ��

	//��USART1 Tx�����ͽţ���GPIO����Ϊ���츴��ģʽ   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				  //ѡ���ĸ�IO�� ��ѡ��PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		  //�趨IO�ڵ�����ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);    				  //��ʼ��GPIOA

	//��USART1 Rx�����սţ���GPIO����Ϊ��������ģʽ														  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				  //ѡ���ĸ�IO�� ��ѡ��PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //��ʼ��GPIOA
	  
	//����USART1����
	USART_InitStructure.USART_BaudRate = 115200;	                    //���������ã�115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //����λ�����ã�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //ֹͣλ���ã�1λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //�Ƿ���żУ�飺��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//Ӳ��������ģʽ���ã�û��ʹ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�����뷢�Ͷ�ʹ��
	USART_Init(USART1, &USART_InitStructure);                       //��ʼ��USART1
	
	//�򿪷����жϺͽ����ж�(�����Ҫ�ж�)
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  // ʹ��ָ����USART1�����ж�
//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ʹ��ָ����USART1�����ж�

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//ʹ�� USART1�� �������
    USART_Cmd(USART1, ENABLE);  
	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������
    USART_ClearFlag(USART1, USART_FLAG_TC);                //�崮��1���ͱ�־

}

void USART1_IRQHandler(void)       
{	 //�����ж���Ч,���������ݼĴ�����
	 if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)  
   { 
      Temp[counter] = USART_ReceiveData(USART1);   //��������

			if(counter == 0 && Temp[0] != 0x55) 
					return;      //�� 0 �����ݲ���֡ͷ������
			counter++;
			if(counter==11) //���յ� 11 ������
      { 
         memcpy(Re_buf,Temp,11);
         counter=0; //���¸�ֵ��׼����һ֡���ݵĽ���
         sign=1;
      }
   }			
}

//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // �ر�USART1�����жϣ�

void   Read_angle(void)
{

  if(sign)
  {   
			 memcpy(Temp,Re_buf,11);
			 sign=0;
			 if(Re_buf[0]==0x55)       //���֡ͷ
			 {  LED1=1;	
					switch(Re_buf[1])
					{
             case 0x51: //��ʶ������Ǽ��ٶȰ�
								accel[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X����ٶ�
								accel[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y����ٶ�
								accel[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z����ٶ�
								t = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //�¶�
								break;
						 case 0x52: //��ʶ������ǽ��ٶȰ�
								speed[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X����ٶ�
								speed[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y����ٶ�
								speed[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z����ٶ�
								t  = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;      //�¶�
								break;
						 case 0x53: //��ʶ������ǽǶȰ�
								angle0[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X���ת�ǣ�x �ᣩRoll
								angle0[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y�ḩ���ǣ�y �ᣩPitch
								angle0[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z��ƫ���ǣ�z �ᣩyaw
								t = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //�¶�

								//printf("X��Ƕȣ�%.2f   Y��Ƕȣ�%.2f   Z��Ƕȣ�%.2f\r\n",angle[0],angle[1],angle[2]);
								break;
						 default:  break;
					}
			 } 	 
	 }
	
}

/*-----------------------------------------------
 ��������: TIM1��ʱ��ΪPID���������ṩ�ȶ��ж�
 ��������: ARR�Ĵ���ֵ0-65535,Ԥ��Ƶֵ0-65535
 �� �� ֵ: TIM1_Config(9999,71)
	       ����Ƶ��1MHz,�ж�Ƶ��100Hz
		   ������ÿ1us��1,�ж�ÿ10ms����һ��		  			  
-----------------------------------------------*/
void TIM1_Config1(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = Period;			        //�Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //ʱ�ӷ�Ƶ1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  					
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  	    //��Ӧ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);	
}
/*-----------------------------------------------

*-----------------------------------------------
 ��������:TIM1�жϷ������
 ����˵��:10ms��һ���ж�
-----------------------------------------------*/
void TIM1_UP_IRQHandler(void)
{		
	LED6Cnt++;

	if(LED6Cnt%10 == 0)	   //100msɨ��һ��
	{	
	
		KeyScan();
	}
	
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}


/*-----------------------------------------------
 ��������: TIM5��ʱ��ΪPID���������ṩ�ȶ��ж�
 ��������: ARR�Ĵ���ֵ0-65535,Ԥ��Ƶֵ0-65535
 �� �� ֵ: TIM5_Config(999,71)
	       ����Ƶ��1MHz,�ж�Ƶ��1000Hz
		   ������ÿ1us��1,�ж�ÿ1ms����һ��		  			  
-----------------------------------------------*/
void TIM5_Config1(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//ʱ��Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = Period;			        //�Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //ʱ�ӷ�Ƶ1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  					
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	    //��Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);		
}


/*-----------------------------------------------
 ��������:TIM5�жϷ������
 ����˵��:ÿ5ms����һ���ж�,������200Hz
 ʵ������ʱ��: 3.93ms
-----------------------------------------------*/
#define H (0.36f)  //����ھ����ĸ߶�(��)
void TIM5_IRQHandler(void)
{  		
	uint8_t i = 0;	
	float pitch_temp1 = 0.0;
	float roll_temp1 = 0.0;
//	float pitch_temp2 = 0.0;
//	float roll_temp2 = 0.0;
	static float pitch_sum = 0.0;
	static float roll_sum = 0.0;
		
	
    GPIOE->BSRR = GPIO_Pin_5;            //������ʱ��
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
	{			
 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ʹ��ָ����USART1�����ж� ��

			for(i=0;i<3;i++)
		{  
		  Read_angle();
			roll_temp1  = angle0[0]; //����Roll�Ƕ� X���ת��
			pitch_temp1 = angle0[1];//����Pitch�Ƕ� Y�ḩ����
			
			roll_sum  += roll_temp1;
			pitch_sum += pitch_temp1;
		
		}
		pitch_temp1 = pitch_sum / 3.0;	 //ȡ��ƽ��ֵ
		roll_temp1  = roll_sum  / 3.0;	 //ȡ��ƽ��ֵ

		pitch_sum = 0.0;
		roll_sum = 0.0;
		M1.CurPos = roll_temp1;         //x���ת�� 
		M2.CurPos =pitch_temp1;         //Y�ḩ����		
   	//�����ٶ�	
		M1.CurSpeed = M1.CurPos - M1.PrevPos;
		M1.PrevPos = M1.CurPos;				
		
		M2.CurSpeed = M2.CurPos - M2.PrevPos;
		M2.PrevPos = M2.CurPos;	
		switch(CurMode)	//������Ŀѡ����
		{	
			case 1: Mode_1(); break;
			case 2: Mode_2(); break;
			case 3: Mode_3(); break;
			case 4: Mode_4(); break;
			case 5: Mode_5(); break;
			case 6: Mode_6(); break;
			default:break;
		}
				
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);		
	}
  	GPIOE->BRR = GPIO_Pin_5;		//		������ʱ��	 	
}


