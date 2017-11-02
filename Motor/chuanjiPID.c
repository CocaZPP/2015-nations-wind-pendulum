#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "led.h"
#include "chuanjiPID.h"
#include "motor_control.h"
#include "motor_pwm.h"
/*------------------------------------------
 				��������				
------------------------------------------*/
extern M1TypeDef  M1;
extern M2TypeDef  M2;

PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;



/*------------------------------------------
 ��������:��ʼ��M1RA�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M1RA_Init(void)
{
    PID_ROLL_ANGLE.LastError  = 0;			//Error[-1]
    PID_ROLL_ANGLE.PrevError  = 0;			//Error[-2]
	  PID_ROLL_ANGLE.Proportion = 0;			//�������� Proportional Const
    PID_ROLL_ANGLE.Integral   = 0;			//���ֳ��� Integral Const
    PID_ROLL_ANGLE.Derivative = 0;			//΢�ֳ��� Derivative Const
    PID_ROLL_ANGLE.SetPoint   = 0;
	  PID_ROLL_ANGLE.SumError   = 0;
	  PID_ROLL_ANGLE.IMAX=1000;           //�����޷�
}
/*------------------------------------------
 ��������:��ʼ��M2PA�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M2PA_Init(void)
{
    PID_PITCH_ANGLE.LastError  = 0;			//Error[-1]
    PID_PITCH_ANGLE.PrevError  = 0;			//Error[-2]
	  PID_PITCH_ANGLE.Proportion = 0;			//�������� Proportional Const
    PID_PITCH_ANGLE.Integral   = 0;			//���ֳ��� Integral Const
    PID_PITCH_ANGLE.Derivative = 0;			//΢�ֳ��� Derivative Const
    PID_PITCH_ANGLE.SetPoint   = 0;
	  PID_PITCH_ANGLE.SumError   = 0;
		PID_PITCH_ANGLE.IMAX=1000;           //�����޷�
}

/*------------------------------------------
 ��������:��ʼ��M1RV�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M1RV_Init(void)
{
    PID_ROLL_RATE.LastError  = 0;			//Error[-1]
    PID_ROLL_RATE.PrevError  = 0;			//Error[-2]
	  PID_ROLL_RATE.Proportion = 0;			//�������� Proportional Const
    PID_ROLL_RATE.Integral   = 0;			//���ֳ��� Integral Const
    PID_ROLL_RATE.Derivative = 0;			//΢�ֳ��� Derivative Const
    PID_ROLL_RATE.SetPoint   = 0;
	  PID_ROLL_RATE.SumError   = 0;
		PID_ROLL_RATE.IMAX=1000;           //�����޷�
}
/*------------------------------------------
 ��������:��ʼ��M2PV�ṹ�����
 ����˵��:			
------------------------------------------*/
void PID_M2PV_Init(void)
{
    PID_PITCH_RATE.LastError  = 0;			//Error[-1]
    PID_PITCH_RATE.PrevError  = 0;			//Error[-2]
	  PID_PITCH_RATE.Proportion = 0;			//�������� Proportional Const
    PID_PITCH_RATE.Integral   = 0;			//���ֳ��� Integral Const
    PID_PITCH_RATE.Derivative = 0;			//΢�ֳ��� Derivative Const
    PID_PITCH_RATE.SetPoint   = 0;
	  PID_PITCH_RATE.SumError   = 0;
	  PID_PITCH_RATE.IMAX=1000;           //�����޷�
}
/*------------------------------------------
 ��������:����M1RA����ֵ
 ����˵��:			
------------------------------------------*/
void PID_M1RA_SetPoint(float setpoint)
{	
	PID_ROLL_ANGLE.SetPoint = setpoint;	
}
/*------------------------------------------
 ��������:����M2PA����ֵ
 ����˵��:			
------------------------------------------*/
void PID_M2PA_SetPoint(float setpoint)
{	
	PID_PITCH_ANGLE.SetPoint = setpoint;	
}
/*------------------------------------------
 ��������:����M1RA����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1RA_SetKp(float dKpp)
{	
	PID_ROLL_ANGLE.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M2RV����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1RV_SetKp(float dKpp)
{	
	PID_ROLL_RATE.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M1PA����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2PA_SetKp(float dKpp)
{	
	PID_PITCH_ANGLE.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M2PV����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2PV_SetKp(float dKpp)
{	
	PID_PITCH_RATE.Proportion = dKpp;	
}
/*------------------------------------------
 ��������:����M1RA����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1RA_SetKi(float dKii)
{	
	PID_ROLL_ANGLE.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M2RV����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1RV_SetKi(float dKii)
{	
	PID_ROLL_RATE.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M1PA����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1PA_SetKi(float dKii)
{	
	PID_PITCH_ANGLE.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M2PV����ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1PV_SetKi(float dKii)
{	
	PID_PITCH_RATE.Integral = dKii;	
}
/*------------------------------------------
 ��������:����M1RA΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1RA_SetKd(float dKdd)
{	
	PID_ROLL_ANGLE.Derivative = dKdd;
}
/*------------------------------------------
 ��������:����M2RV΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2RV_SetKd(float dKdd)
{	
	PID_ROLL_RATE.Derivative = dKdd;
}
/*------------------------------------------
 ��������:����M1PA΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M1PA_SetKd(float dKdd)
{	
	PID_PITCH_ANGLE.Derivative = dKdd;
}
/*------------------------------------------
 ��������:����M2PV΢��ϵ��
 ����˵��:������			
------------------------------------------*/
void PID_M2PV_SetKd(float dKdd)
{	
	PID_PITCH_RATE.Derivative = dKdd;
}

/*------------------------------------------
 ��������:���1����PID����
 ����˵��:		
------------------------------------------*/

int32_t PID_M1_chuanji(float NextPoint)
{
    register float  iError,dError,iError1,dError1;
	
	iError = PID_ROLL_ANGLE.SetPoint - NextPoint;        // �⻷ƫ��
	PID_ROLL_ANGLE.SumError += iError;				    // �⻷����
	if(PID_ROLL_ANGLE.SumError > PID_ROLL_ANGLE.IMAX)					//�⻷�����޷�
		PID_ROLL_ANGLE.SumError = PID_ROLL_ANGLE.IMAX;
	else if(PID_ROLL_ANGLE.SumError < -PID_ROLL_ANGLE.IMAX)
		PID_ROLL_ANGLE.SumError = -PID_ROLL_ANGLE.IMAX;	
	
	dError = iError - PID_ROLL_ANGLE.LastError; 			//�⻷΢��
	PID_ROLL_ANGLE.LastError = iError;
	PID_ROLL_ANGLE.OUT=PID_ROLL_ANGLE.Proportion * iError + PID_ROLL_ANGLE.Integral   * PID_ROLL_ANGLE.SumError + PID_ROLL_ANGLE.Derivative * dError;
//�⻷��� ���ڻ�Ϊ����ֵ
	PID_ROLL_RATE.SetPoint=2000;
	iError1 = PID_ROLL_RATE.SetPoint - NextPoint;        // �ڻ�ƫ��
	PID_ROLL_RATE.SumError += iError1;				    // �ڻ�����
		if(PID_ROLL_RATE.SumError >PID_ROLL_RATE.IMAX)					//�ڻ������޷�
		PID_ROLL_RATE.SumError = PID_ROLL_RATE.IMAX;
	else if(PID_ROLL_RATE.SumError < -PID_ROLL_RATE.IMAX)
		PID_ROLL_RATE.SumError = -PID_ROLL_RATE.IMAX;	
	
		dError1 = iError1 - PID_ROLL_RATE.LastError; 			// �ڻ�΢��
	PID_ROLL_RATE.LastError = iError1;
	PID_ROLL_RATE.OUT=PID_ROLL_RATE.Proportion * iError1 + PID_ROLL_RATE.Integral   * PID_ROLL_RATE.SumError + PID_ROLL_RATE.Derivative * dError1;
	
	return(int32_t)( PID_ROLL_RATE.OUT );//����ڻ�ֵΪ����ֵ
}

int32_t PID_M2_chuanji(float NextPoint)
{
    register float  iError,dError,iError1,dError1;
	
	iError = PID_PITCH_ANGLE.SetPoint - NextPoint;        // �⻷ƫ��
	PID_PITCH_ANGLE.SumError += iError;				    // �⻷����
	if(PID_PITCH_ANGLE.SumError > PID_PITCH_ANGLE.IMAX)					//�⻷�����޷�
		PID_PITCH_ANGLE.SumError = PID_PITCH_ANGLE.IMAX;
	else if(PID_PITCH_ANGLE.SumError < -PID_PITCH_ANGLE.IMAX)
		PID_PITCH_ANGLE.SumError = -PID_PITCH_ANGLE.IMAX;	
	
	dError = iError - PID_PITCH_ANGLE.LastError; 			// �⻷΢��
	PID_PITCH_ANGLE.LastError = iError;
	PID_PITCH_ANGLE.OUT=PID_PITCH_ANGLE.Proportion * iError + PID_PITCH_ANGLE.Integral   * PID_PITCH_ANGLE.SumError + PID_PITCH_ANGLE.Derivative * dError;
//�⻷��� ���ڻ�Ϊ����ֵ	
	PID_PITCH_RATE.SetPoint=2000;
	iError1 = PID_PITCH_ANGLE.SetPoint - NextPoint;        // �ڻ�ƫ��
	PID_PITCH_RATE.SumError += iError1;				    // �ڻ�����
		if(PID_PITCH_RATE.SumError >PID_PITCH_RATE.IMAX)					//�ڻ������޷�
		PID_PITCH_RATE.SumError = PID_PITCH_RATE.IMAX;
	else if(PID_PITCH_RATE.SumError < -PID_PITCH_RATE.IMAX)
		PID_PITCH_RATE.SumError = -PID_PITCH_RATE.IMAX;	
	
		dError1 = iError1 - PID_PITCH_RATE.LastError; 			// �ڻ�΢��
	PID_PITCH_RATE.LastError = iError1;
	PID_ROLL_RATE.OUT=PID_PITCH_RATE.Proportion * iError1 + PID_PITCH_RATE.Integral   * PID_PITCH_RATE.SumError + PID_PITCH_RATE.Derivative * dError1;
	return(int32_t)( PID_PITCH_RATE.OUT );//����ڻ�ֵΪ����ֵ
}
