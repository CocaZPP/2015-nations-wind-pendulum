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
float angle0[3],speed[3],accel[3],temperature;//角度,角速度，角加速度,温度

extern M1TypeDef  M1;
extern M2TypeDef  M2;
extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;
extern PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;

uint8_t CurMode = 0;
static uint16_t LED6Cnt = 0;


//串口接收数据
void USART1_Config1(void)
{
GPIO_InitTypeDef    GPIO_InitStructure;	   //串口端口配置结构体变量
	USART_InitTypeDef   USART_InitStructure;   //串口参数配置结构体变量
NVIC_InitTypeDef NVIC_InitStructure;
	//使能 USART1 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//打开串口复用时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);   //打开PA端口时钟

	//将USART1 Tx（发送脚）的GPIO配置为推挽复用模式   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				  //选定哪个IO口 现选定PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		  //设定IO口的输出速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);    				  //初始化GPIOA

	//将USART1 Rx（接收脚）的GPIO配置为浮空输入模式														  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				  //选定哪个IO口 现选定PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);                    //初始化GPIOA
	  
	//配置USART1参数
	USART_InitStructure.USART_BaudRate = 115200;	                    //波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	    //数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	        //停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收与发送都使能
	USART_Init(USART1, &USART_InitStructure);                       //初始化USART1
	
	//打开发送中断和接收中断(如果需要中断)
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  // 使能指定的USART1发送中断
//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能指定的USART1接收中断

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//使能 USART1， 配置完毕
    USART_Cmd(USART1, ENABLE);  
	//如下语句解决第1个字节无法正确发送出去的问题
    USART_ClearFlag(USART1, USART_FLAG_TC);                //清串口1发送标志

}

void USART1_IRQHandler(void)       
{	 //接收中断有效,若接收数据寄存器满
	 if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)  
   { 
      Temp[counter] = USART_ReceiveData(USART1);   //接收数据

			if(counter == 0 && Temp[0] != 0x55) 
					return;      //第 0 号数据不是帧头，跳过
			counter++;
			if(counter==11) //接收到 11 个数据
      { 
         memcpy(Re_buf,Temp,11);
         counter=0; //重新赋值，准备下一帧数据的接收
         sign=1;
      }
   }			
}

//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // 关闭USART1接收中断；

void   Read_angle(void)
{

  if(sign)
  {   
			 memcpy(Temp,Re_buf,11);
			 sign=0;
			 if(Re_buf[0]==0x55)       //检查帧头
			 {  LED1=1;	
					switch(Re_buf[1])
					{
             case 0x51: //标识这个包是加速度包
								accel[0] = ((short)(Temp[3]<<8 | Temp[2]))/32768.0*16;      //X轴加速度
								accel[1] = ((short)(Temp[5]<<8 | Temp[4]))/32768.0*16;      //Y轴加速度
								accel[2] = ((short)(Temp[7]<<8 | Temp[6]))/32768.0*16;      //Z轴加速度
								t = ((short)(Temp[9]<<8 | Temp[8]))/340.0+36.25;      //温度
								break;
						 case 0x52: //标识这个包是角速度包
								speed[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*2000;      //X轴角速度
								speed[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*2000;      //Y轴角速度
								speed[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*2000;      //Z轴角速度
								t  = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;      //温度
								break;
						 case 0x53: //标识这个包是角度包
								angle0[0] = ((short)(Temp[3]<<8| Temp[2]))/32768.0*180;   //X轴滚转角（x 轴）Roll
								angle0[1] = ((short)(Temp[5]<<8| Temp[4]))/32768.0*180;   //Y轴俯仰角（y 轴）Pitch
								angle0[2] = ((short)(Temp[7]<<8| Temp[6]))/32768.0*180;   //Z轴偏航角（z 轴）yaw
								t = ((short)(Temp[9]<<8| Temp[8]))/340.0+36.25;   //温度

								//printf("X轴角度：%.2f   Y轴角度：%.2f   Z轴角度：%.2f\r\n",angle[0],angle[1],angle[2]);
								break;
						 default:  break;
					}
			 } 	 
	 }
	
}

/*-----------------------------------------------
 函数功能: TIM1定时器为PID采样计算提供稳定中断
 函数参数: ARR寄存器值0-65535,预分频值0-65535
 参 考 值: TIM1_Config(9999,71)
	       计数频率1MHz,中断频率100Hz
		   计数器每1us加1,中断每10ms产生一次		  			  
-----------------------------------------------*/
void TIM1_Config1(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//时钟预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = Period;			        //自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //时钟分频1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  					
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  	    //响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);	
}
/*-----------------------------------------------

*-----------------------------------------------
 函数功能:TIM1中断服务程序
 函数说明:10ms进一次中断
-----------------------------------------------*/
void TIM1_UP_IRQHandler(void)
{		
	LED6Cnt++;

	if(LED6Cnt%10 == 0)	   //100ms扫描一次
	{	
	
		KeyScan();
	}
	
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}


/*-----------------------------------------------
 函数功能: TIM5定时器为PID采样计算提供稳定中断
 函数参数: ARR寄存器值0-65535,预分频值0-65535
 参 考 值: TIM5_Config(999,71)
	       计数频率1MHz,中断频率1000Hz
		   计数器每1us加1,中断每1ms产生一次		  			  
-----------------------------------------------*/
void TIM5_Config1(unsigned short int Period,unsigned short int Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; 			//时钟预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = Period;			        //自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	    //时钟分频1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  					
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	    //响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	            
	NVIC_Init(&NVIC_InitStructure);		
}


/*-----------------------------------------------
 函数功能:TIM5中断服务程序
 函数说明:每5ms进入一次中断,采样率200Hz
 实测运行时间: 3.93ms
-----------------------------------------------*/
#define H (0.36f)  //万向节距地面的高度(米)
void TIM5_IRQHandler(void)
{  		
	uint8_t i = 0;	
	float pitch_temp1 = 0.0;
	float roll_temp1 = 0.0;
//	float pitch_temp2 = 0.0;
//	float roll_temp2 = 0.0;
	static float pitch_sum = 0.0;
	static float roll_sum = 0.0;
		
	
    GPIOE->BSRR = GPIO_Pin_5;            //测运行时间
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
	{			
 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能指定的USART1接收中断 ；

			for(i=0;i<3;i++)
		{  
		  Read_angle();
			roll_temp1  = angle0[0]; //计算Roll角度 X轴滚转角
			pitch_temp1 = angle0[1];//计算Pitch角度 Y轴俯仰角
			
			roll_sum  += roll_temp1;
			pitch_sum += pitch_temp1;
		
		}
		pitch_temp1 = pitch_sum / 3.0;	 //取出平均值
		roll_temp1  = roll_sum  / 3.0;	 //取出平均值

		pitch_sum = 0.0;
		roll_sum = 0.0;
		M1.CurPos = roll_temp1;         //x轴滚转角 
		M2.CurPos =pitch_temp1;         //Y轴俯仰角		
   	//计算速度	
		M1.CurSpeed = M1.CurPos - M1.PrevPos;
		M1.PrevPos = M1.CurPos;				
		
		M2.CurSpeed = M2.CurPos - M2.PrevPos;
		M2.PrevPos = M2.CurPos;	
		switch(CurMode)	//根据题目选择函数
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
  	GPIOE->BRR = GPIO_Pin_5;		//		测运行时间	 	
}


