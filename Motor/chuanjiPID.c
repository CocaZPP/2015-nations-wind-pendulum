#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "led.h"
#include "chuanjiPID.h"
#include "motor_control.h"
#include "motor_pwm.h"
/*------------------------------------------
 				声明变量				
------------------------------------------*/
extern M1TypeDef  M1;
extern M2TypeDef  M2;

PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;



/*------------------------------------------
 函数功能:初始化M1RA结构体参数
 函数说明:			
------------------------------------------*/
void PID_M1RA_Init(void)
{
    PID_ROLL_ANGLE.LastError  = 0;			//Error[-1]
    PID_ROLL_ANGLE.PrevError  = 0;			//Error[-2]
	  PID_ROLL_ANGLE.Proportion = 0;			//比例常数 Proportional Const
    PID_ROLL_ANGLE.Integral   = 0;			//积分常数 Integral Const
    PID_ROLL_ANGLE.Derivative = 0;			//微分常数 Derivative Const
    PID_ROLL_ANGLE.SetPoint   = 0;
	  PID_ROLL_ANGLE.SumError   = 0;
	  PID_ROLL_ANGLE.IMAX=1000;           //积分限幅
}
/*------------------------------------------
 函数功能:初始化M2PA结构体参数
 函数说明:			
------------------------------------------*/
void PID_M2PA_Init(void)
{
    PID_PITCH_ANGLE.LastError  = 0;			//Error[-1]
    PID_PITCH_ANGLE.PrevError  = 0;			//Error[-2]
	  PID_PITCH_ANGLE.Proportion = 0;			//比例常数 Proportional Const
    PID_PITCH_ANGLE.Integral   = 0;			//积分常数 Integral Const
    PID_PITCH_ANGLE.Derivative = 0;			//微分常数 Derivative Const
    PID_PITCH_ANGLE.SetPoint   = 0;
	  PID_PITCH_ANGLE.SumError   = 0;
		PID_PITCH_ANGLE.IMAX=1000;           //积分限幅
}

/*------------------------------------------
 函数功能:初始化M1RV结构体参数
 函数说明:			
------------------------------------------*/
void PID_M1RV_Init(void)
{
    PID_ROLL_RATE.LastError  = 0;			//Error[-1]
    PID_ROLL_RATE.PrevError  = 0;			//Error[-2]
	  PID_ROLL_RATE.Proportion = 0;			//比例常数 Proportional Const
    PID_ROLL_RATE.Integral   = 0;			//积分常数 Integral Const
    PID_ROLL_RATE.Derivative = 0;			//微分常数 Derivative Const
    PID_ROLL_RATE.SetPoint   = 0;
	  PID_ROLL_RATE.SumError   = 0;
		PID_ROLL_RATE.IMAX=1000;           //积分限幅
}
/*------------------------------------------
 函数功能:初始化M2PV结构体参数
 函数说明:			
------------------------------------------*/
void PID_M2PV_Init(void)
{
    PID_PITCH_RATE.LastError  = 0;			//Error[-1]
    PID_PITCH_RATE.PrevError  = 0;			//Error[-2]
	  PID_PITCH_RATE.Proportion = 0;			//比例常数 Proportional Const
    PID_PITCH_RATE.Integral   = 0;			//积分常数 Integral Const
    PID_PITCH_RATE.Derivative = 0;			//微分常数 Derivative Const
    PID_PITCH_RATE.SetPoint   = 0;
	  PID_PITCH_RATE.SumError   = 0;
	  PID_PITCH_RATE.IMAX=1000;           //积分限幅
}
/*------------------------------------------
 函数功能:设置M1RA期望值
 函数说明:			
------------------------------------------*/
void PID_M1RA_SetPoint(float setpoint)
{	
	PID_ROLL_ANGLE.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M2PA期望值
 函数说明:			
------------------------------------------*/
void PID_M2PA_SetPoint(float setpoint)
{	
	PID_PITCH_ANGLE.SetPoint = setpoint;	
}
/*------------------------------------------
 函数功能:设置M1RA比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1RA_SetKp(float dKpp)
{	
	PID_ROLL_ANGLE.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M2RV比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1RV_SetKp(float dKpp)
{	
	PID_ROLL_RATE.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M1PA比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2PA_SetKp(float dKpp)
{	
	PID_PITCH_ANGLE.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M2PV比例系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2PV_SetKp(float dKpp)
{	
	PID_PITCH_RATE.Proportion = dKpp;	
}
/*------------------------------------------
 函数功能:设置M1RA积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1RA_SetKi(float dKii)
{	
	PID_ROLL_ANGLE.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M2RV积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1RV_SetKi(float dKii)
{	
	PID_ROLL_RATE.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M1PA积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1PA_SetKi(float dKii)
{	
	PID_PITCH_ANGLE.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M2PV积分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1PV_SetKi(float dKii)
{	
	PID_PITCH_RATE.Integral = dKii;	
}
/*------------------------------------------
 函数功能:设置M1RA微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1RA_SetKd(float dKdd)
{	
	PID_ROLL_ANGLE.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M2RV微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2RV_SetKd(float dKdd)
{	
	PID_ROLL_RATE.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M1PA微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M1PA_SetKd(float dKdd)
{	
	PID_PITCH_ANGLE.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M2PV微分系数
 函数说明:浮点型			
------------------------------------------*/
void PID_M2PV_SetKd(float dKdd)
{	
	PID_PITCH_RATE.Derivative = dKdd;
}

/*------------------------------------------
 函数功能:电机1串级PID计算
 函数说明:		
------------------------------------------*/

int32_t PID_M1_chuanji(float NextPoint)
{
    register float  iError,dError,iError1,dError1;
	
	iError = PID_ROLL_ANGLE.SetPoint - NextPoint;        // 外环偏差
	PID_ROLL_ANGLE.SumError += iError;				    // 外环积分
	if(PID_ROLL_ANGLE.SumError > PID_ROLL_ANGLE.IMAX)					//外环积分限幅
		PID_ROLL_ANGLE.SumError = PID_ROLL_ANGLE.IMAX;
	else if(PID_ROLL_ANGLE.SumError < -PID_ROLL_ANGLE.IMAX)
		PID_ROLL_ANGLE.SumError = -PID_ROLL_ANGLE.IMAX;	
	
	dError = iError - PID_ROLL_ANGLE.LastError; 			//外环微分
	PID_ROLL_ANGLE.LastError = iError;
	PID_ROLL_ANGLE.OUT=PID_ROLL_ANGLE.Proportion * iError + PID_ROLL_ANGLE.Integral   * PID_ROLL_ANGLE.SumError + PID_ROLL_ANGLE.Derivative * dError;
//外环输出 给内环为期望值
	PID_ROLL_RATE.SetPoint=2000;
	iError1 = PID_ROLL_RATE.SetPoint - NextPoint;        // 内环偏差
	PID_ROLL_RATE.SumError += iError1;				    // 内环积分
		if(PID_ROLL_RATE.SumError >PID_ROLL_RATE.IMAX)					//内环积分限幅
		PID_ROLL_RATE.SumError = PID_ROLL_RATE.IMAX;
	else if(PID_ROLL_RATE.SumError < -PID_ROLL_RATE.IMAX)
		PID_ROLL_RATE.SumError = -PID_ROLL_RATE.IMAX;	
	
		dError1 = iError1 - PID_ROLL_RATE.LastError; 			// 内环微分
	PID_ROLL_RATE.LastError = iError1;
	PID_ROLL_RATE.OUT=PID_ROLL_RATE.Proportion * iError1 + PID_ROLL_RATE.Integral   * PID_ROLL_RATE.SumError + PID_ROLL_RATE.Derivative * dError1;
	
	return(int32_t)( PID_ROLL_RATE.OUT );//输出内环值为最终值
}

int32_t PID_M2_chuanji(float NextPoint)
{
    register float  iError,dError,iError1,dError1;
	
	iError = PID_PITCH_ANGLE.SetPoint - NextPoint;        // 外环偏差
	PID_PITCH_ANGLE.SumError += iError;				    // 外环积分
	if(PID_PITCH_ANGLE.SumError > PID_PITCH_ANGLE.IMAX)					//外环积分限幅
		PID_PITCH_ANGLE.SumError = PID_PITCH_ANGLE.IMAX;
	else if(PID_PITCH_ANGLE.SumError < -PID_PITCH_ANGLE.IMAX)
		PID_PITCH_ANGLE.SumError = -PID_PITCH_ANGLE.IMAX;	
	
	dError = iError - PID_PITCH_ANGLE.LastError; 			// 外环微分
	PID_PITCH_ANGLE.LastError = iError;
	PID_PITCH_ANGLE.OUT=PID_PITCH_ANGLE.Proportion * iError + PID_PITCH_ANGLE.Integral   * PID_PITCH_ANGLE.SumError + PID_PITCH_ANGLE.Derivative * dError;
//外环输出 给内环为期望值	
	PID_PITCH_RATE.SetPoint=2000;
	iError1 = PID_PITCH_ANGLE.SetPoint - NextPoint;        // 内环偏差
	PID_PITCH_RATE.SumError += iError1;				    // 内环积分
		if(PID_PITCH_RATE.SumError >PID_PITCH_RATE.IMAX)					//内环积分限幅
		PID_PITCH_RATE.SumError = PID_PITCH_RATE.IMAX;
	else if(PID_PITCH_RATE.SumError < -PID_PITCH_RATE.IMAX)
		PID_PITCH_RATE.SumError = -PID_PITCH_RATE.IMAX;	
	
		dError1 = iError1 - PID_PITCH_RATE.LastError; 			// 内环微分
	PID_PITCH_RATE.LastError = iError1;
	PID_ROLL_RATE.OUT=PID_PITCH_RATE.Proportion * iError1 + PID_PITCH_RATE.Integral   * PID_PITCH_RATE.SumError + PID_PITCH_RATE.Derivative * dError1;
	return(int32_t)( PID_PITCH_RATE.OUT );//输出内环值为最终值
}
