#ifndef __CHUANJIPID_H
#define __CHUANJIPID_H
#include "sys.h"
typedef struct
{
	float  SetPoint; 	//  设定目标 Desired Value 
	double  SumError;		//	误差累计 
		
	float  Proportion;      //  比例常数 Proportional Const 
	float  Integral;        //  积分常数 Integral Const
	float  Derivative;      //  微分常数 Derivative Const
	float IMAX;             //I限制值
	
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




