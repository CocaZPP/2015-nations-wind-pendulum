#include "motor_control.h"
#include "motor_pwm.h"
#include "motor_pid.h"
#include "chuanjiPID.h"
#include "stdlib.h"
#include "stdio.h"
#include "delay.h"
#include "math.h"
#include "mpu6050.h"
#include "led.h"
#include "lcd12864.h"
/*------------------------------------------
 				ȫ�ֱ���				
------------------------------------------*/
 M1TypeDef  M1;
 M2TypeDef  M2;

#define CMD1       (uint32_t)0xf8000000
extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;
extern PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;

float R = 0; 					 //�뾶����(cm)
float angle = 40.0;					 //�ڶ��Ƕ�����(��)
uint8_t RoundDir = 0; 				 //����ת����
/*------------------------------------------
 ��������:������������λ
 ����˵��:ǿ�Ƹ�λ			
------------------------------------------*/
void MCU_Reset(void) 
{  
	__set_FAULTMASK(1);   // �ر������ж�
 	NVIC_SystemReset();   // ��λ
}
/*------------------------------------------
 ��������:��ʼ��M1�ṹ�����
 ����˵��:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
	M1.CurPos    = 0.0;
	M1.PrevPos   = 0.0;
	M1.CurAcc    = 0.0;
	M1.PrevSpeed = 0.0;
 	M1.Offset    = 0.1;   //����ƫ����
	M1.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M1.PWM = 0;	         //PWM
}
/*------------------------------------------
 ��������:��ʼ��M2�ṹ�����
 ����˵��:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
	M2.CurPos    = 0.0;
	M2.PrevPos   = 0.0;
	M2.CurAcc    = 0.0;
	M2.PrevSpeed = 0.0;
 	M2.Offset    = 0.1;   //����ƫ����
	M2.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M2.PWM = 0;	         //PWM		
}
/*------------------------------------------
 ��������:
------------------------------------------*/
void Mode_0(void)
{
	
		
}
/*------------------------------------------
 ��������:��1��PID����
 ����˵��:
------------------------------------------*/
void Mode_1(void)
{
	const float priod = 1410.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float set_y = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
				LED=1;
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan((30.0f/36.0f))*57.2958f;				 //���ݰڷ�����Ƕ�A,88Ϊ�ڸ˾�����泤��cm
	set_y = A*sin(Omega);                        //�������ǰ�ڽ� 	
		
	PID_M1_SetPoint(0);			//X����PID��λĿ��ֵ0
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(50);
	
	PID_M2_SetPoint(set_y);		//Y����PID����Ŀ��ֵsin
	PID_M2_SetKp(46);    
	PID_M2_SetKi(0.15);		
	PID_M2_SetKd(50); 	 

	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//Roll
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Pitch
	
	if(M1.PWM > 1500)   M1.PWM =  POWER_MAX;
	if(M1.PWM < -1500) M1.PWM = -POWER_MAX;	
	
	if(M2.PWM > 1500)  M2.PWM = POWER_MAX;
	if(M2.PWM < -1500) M2.PWM = -POWER_MAX;		
	
	MotorMove(  M1.PWM ,M2.PWM);//������
}
/*------------------------------------------
 ��������:��2��PID����
 ����˵��:
------------------------------------------*/
void Mode_2(void)
{
	const float priod = 1410.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;				
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan((R/36.0f))*24.2958f;//���ݰڷ�����Ƕ�A,88Ϊ�ڸ���ظ߶�
LED=1;
	set_x = A*sin(Omega);                        //�������ǰ�ڽ� 
LCD_Write(CMD1,0x01);	
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0.2);	 
	PID_M1_SetKd(10);	
	PID_M2_SetPoint(3);		//Y����PID��λĿ��ֵ0
	PID_M2_SetKp(102);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(50); 	 	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//X����PID����
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Y����PID����	
	if(M1.PWM > POWER_MAX) M1.PWM  =  0;//����޷�
	if(M1.PWM < -POWER_MAX) M1.PWM = -0; 	
	if(M2.PWM > POWER_MAX) M2.PWM  =  0;
	if(M2.PWM < -POWER_MAX) M2.PWM = -0;			
	MotorMove( M1.PWM,M2.PWM);//������
}
/*------------------------------------------
 ��������:��3��PID����
 ����˵��:
------------------------------------------*/ 
void Mode_3(void)
{
	const float priod = 1410.0;  //��������(����)
	             //��λ���� 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
	const float Phase[19]= {0,-0.1,-0.05,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.05,0.05,0.05,0.07,0};
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float set_y = 0.0;
	float Ax = 0.0;
	float Ay = 0.0;
	float A = 0.0;
	uint32_t pOffset = 0;
	float Normalization = 0.0;
	float Omega = 0.0;
	LED=1;
	pOffset = (uint32_t)(angle/10.0f);			 //��λ���������±�
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan((20/36.0f))*24.2958f;//���ݰڷ�����Ƕ�A,88Ϊ�ڸ���ظ߶�                   						
	Ax = A*cos(angle*0.017453);	 //�����X����ڷ�����0.017453Ϊ����ת��
	Ay = A*sin(angle*0.017453);	 //�����Y����ڷ�����
	set_x = Ax*sin(Omega); 		 //�����X����ǰ�ڽ�
	set_y = Ay*sin(Omega+Phase[pOffset]); //�����Y����ǰ�ڽ�
		
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0.2);	 
	PID_M1_SetKd(10);

	PID_M2_SetPoint(set_y);	//Y����PID����Ŀ��ֵsin
	PID_M2_SetKp(102);    
	PID_M2_SetKi(0.2);		
	PID_M2_SetKd(10); 	 
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//Roll
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Pitch
	
	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
			 	
	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		

	MotorMove(M1.PWM,M2.PWM);	
}
/*------------------------------------------
 ��������:��4��PID����
 ����˵��:
------------------------------------------*/ 
void Mode_4(void)
{	
	if(abs(M1.CurPos)<45.0 && abs(M2.CurPos)<45.0)	//С��45�ȲŽ����ƶ�
	{		LED=1;
		PID_M1RA_SetPoint(0);	  //X����PID��λĿ��ֵ0
		PID_M1RA_SetKp(0); 		
		PID_M1RA_SetKi(0);     
		PID_M1RA_SetKd(0);     //�⻷
		
		PID_M1RV_SetKp(100); 		
		PID_M1RV_SetKi(0);     
		PID_M2RV_SetKd(0);    //�ڻ�

		PID_M2PA_SetPoint(0);	  //Y����PID��λĿ��ֵ0
		PID_M2PA_SetKp(0);  		
		PID_M1PA_SetKi(0);    
		PID_M1PA_SetKd(0);   // �⻷
		
		PID_M2PV_SetKp(0);  		
		PID_M1PV_SetKi(0);    
		PID_M2PV_SetKd(0);   // �ڻ�
			
		M1.PWM = PID_M1_chuanji(M1.CurPos);//Roll 
		M2.PWM = PID_M2_chuanji(M2.CurPos); //Pitch
		
		if(M1.PWM > 2000)  M1.PWM =  POWER_MAX;
		if(M1.PWM < -2000) M1.PWM = -POWER_MAX;

		if(M2.PWM > 2000)  M2.PWM =  POWER_MAX;
		if(M2.PWM < -2000) M2.PWM = -POWER_MAX;
	}
	else	
	{
	 	M1.PWM = 0;
		M2.PWM = 0;	
	}
	
	MotorMove(M1.PWM,0);
}
/*------------------------------------------
 ��������:��5��PID����
 ����˵��:
------------------------------------------*/
void Mode_5(void)
{
	const float priod = 1410.0;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float set_y = 0.0;
	float A = 0.0;
	float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	LED=1;
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������				
	A = atan((25/36.0f))*57.2958f;    //���ݰ뾶�����Ӧ�����A
	
	if(RoundDir == 0)       	  
		phase = 3.141592/2.0;		 //��ʱ����ת��λ��90�� 
	else if(RoundDir == 1)  
		phase = (3.0*3.141592)/2.0;	 //˳ʱ����ת��λ��270��
	
	set_x = A*sin(Omega);			 //�����X����ǰ�ڽ�
	set_y = A*sin(Omega+phase); 	 //�����Y����ǰ�ڽ�
	 
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(50);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(0);

	PID_M2_SetPoint(set_y);	//Y����PID����Ŀ��ֵcos
	PID_M2_SetKp(50);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(0); 		 
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Roll 
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);//Pitch
	
	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
			 	
	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		

	MotorMove(M1.PWM,M2.PWM);
	
}
/*------------------------------------------
 ��������:��6��PID����
 ����˵��:
------------------------------------------*/
void Mode_6(void)
{

}
/*------------------------------------------
 ��������:����ײ���������
 ����˵��:
------------------------------------------*/
void MotorMove(int32_t pwm1,int32_t pwm2)
{
	if(pwm1 > 0)
	{
	 PWM_M2_Forward(abs(pwm1));
		PWM_M4_Forward(abs(pwm1));	
	}
	else if(pwm1 < 0)
	{
	 	PWM_M2_Backward(abs(pwm1));
		PWM_M4_Backward(abs(pwm1));	
	}

	if(pwm2 > 0)
	{
	 	PWM_M1_Forward(pwm2);
		PWM_M3_Backward(pwm2);
	}
	else if(pwm2 < 0)
	{
	 	PWM_M1_Backward(abs(pwm2));
		PWM_M3_Forward(abs(pwm2));	
	} 	
}

