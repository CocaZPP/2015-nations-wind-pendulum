#ifndef __CHUANJIPID_H
#define __CHUANJIPID_H
#include "sys.h"
typedef struct
{
	float  SetPoint; 	//  �趨Ŀ�� Desired Value 
	double  SumError;		//	����ۼ� 
		
	float  Proportion;      //  �������� Proportional Const 
	float  Integral;        //  ���ֳ��� Integral Const
	float  Derivative;      //  ΢�ֳ��� Derivative Const
	float IMAX;             //I����ֵ
	
	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]
	
	float OUT;

}PID;

void PID_M1RA_Init(void);
void PID_M2PA_Init(void);
void PID_M1RV_Init(void);
void PID_M2PV_Init(void);
void PID_M1RA_SetPoint(float setpoint);
void PID_M2PA_SetPoint(float setpoint);
void PID_M1RA_SetKp(float dKpp);
void PID_M1RV_SetKp(float dKpp);
void PID_M2PA_SetKp(float dKpp);
void PID_M2PV_SetKp(float dKpp);
void PID_M1RA_SetKi(float dKii);
void PID_M1RV_SetKi(float dKii);
void PID_M1PA_SetKi(float dKii);
void PID_M1PV_SetKi(float dKii);
void PID_M1RA_SetKd(float dKdd);
void PID_M2RV_SetKd(float dKdd);
void PID_M1PA_SetKd(float dKdd);
void PID_M2PV_SetKd(float dKdd);
int32_t PID_M1_chuanji(float NextPoint);
int32_t PID_M2_chuanji(float NextPoint);

#endif




