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
 				全局变量				
------------------------------------------*/
 M1TypeDef  M1;
 M2TypeDef  M2;

#define CMD1       (uint32_t)0xf8000000
extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;
extern PID PID_ROLL_ANGLE,PID_PITCH_ANGLE,PID_ROLL_RATE,PID_PITCH_RATE;

float R = 0; 					 //半径设置(cm)
float angle = 40.0;					 //摆动角度设置(°)
uint8_t RoundDir = 0; 				 //正反转控制
/*------------------------------------------
 函数功能:控制器软件复位
 函数说明:强制复位			
------------------------------------------*/
void MCU_Reset(void) 
{  
	__set_FAULTMASK(1);   // 关闭所有中断
 	NVIC_SystemReset();   // 复位
}
/*------------------------------------------
 函数功能:初始化M1结构体参数
 函数说明:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
	M1.CurPos    = 0.0;
	M1.PrevPos   = 0.0;
	M1.CurAcc    = 0.0;
	M1.PrevSpeed = 0.0;
 	M1.Offset    = 0.1;   //允许偏差量
	M1.CurSpeed  = 0.0;  //当前速度矢量
	M1.PWM = 0;	         //PWM
}
/*------------------------------------------
 函数功能:初始化M2结构体参数
 函数说明:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
	M2.CurPos    = 0.0;
	M2.PrevPos   = 0.0;
	M2.CurAcc    = 0.0;
	M2.PrevSpeed = 0.0;
 	M2.Offset    = 0.1;   //允许偏差量
	M2.CurSpeed  = 0.0;  //当前速度矢量
	M2.PWM = 0;	         //PWM		
}
/*------------------------------------------
 函数功能:
------------------------------------------*/
void Mode_0(void)
{
	
		
}
/*------------------------------------------
 函数功能:第1问PID计算
 函数说明:
------------------------------------------*/
void Mode_1(void)
{
	const float priod = 1410.0;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	float set_y = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
				LED=1;
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理
	A = atan((30.0f/36.0f))*57.2958f;				 //根据摆幅求出角度A,88为摆杆距离地面长度cm
	set_y = A*sin(Omega);                        //计算出当前摆角 	
		
	PID_M1_SetPoint(0);			//X方向PID定位目标值0
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(50);
	
	PID_M2_SetPoint(set_y);		//Y方向PID跟踪目标值sin
	PID_M2_SetKp(46);    
	PID_M2_SetKi(0.15);		
	PID_M2_SetKd(50); 	 

	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//Roll
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Pitch
	
	if(M1.PWM > 1500)   M1.PWM =  POWER_MAX;
	if(M1.PWM < -1500) M1.PWM = -POWER_MAX;	
	
	if(M2.PWM > 1500)  M2.PWM = POWER_MAX;
	if(M2.PWM < -1500) M2.PWM = -POWER_MAX;		
	
	MotorMove(  M1.PWM ,M2.PWM);//电机输出
}
/*------------------------------------------
 函数功能:第2问PID计算
 函数说明:
------------------------------------------*/
void Mode_2(void)
{
	const float priod = 1410.0;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;				
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理
	A = atan((R/36.0f))*24.2958f;//根据摆幅求出角度A,88为摆杆离地高度
LED=1;
	set_x = A*sin(Omega);                        //计算出当前摆角 
LCD_Write(CMD1,0x01);	
	PID_M1_SetPoint(set_x);	//X方向PID跟踪目标值sin
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0.2);	 
	PID_M1_SetKd(10);	
	PID_M2_SetPoint(3);		//Y方向PID定位目标值0
	PID_M2_SetKp(102);    
	PID_M2_SetKi(0);		
	PID_M2_SetKd(50); 	 	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos);	//X方向PID计算
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Y方向PID计算	
	if(M1.PWM > POWER_MAX) M1.PWM  =  0;//输出限幅
	if(M1.PWM < -POWER_MAX) M1.PWM = -0; 	
	if(M2.PWM > POWER_MAX) M2.PWM  =  0;
	if(M2.PWM < -POWER_MAX) M2.PWM = -0;			
	MotorMove( M1.PWM,M2.PWM);//电机输出
}
/*------------------------------------------
 函数功能:第3问PID计算
 函数说明:
------------------------------------------*/ 
void Mode_3(void)
{
	const float priod = 1410.0;  //单摆周期(毫秒)
	             //相位补偿 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
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
	pOffset = (uint32_t)(angle/10.0f);			 //相位补偿数组下标
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理
	A = atan((20/36.0f))*24.2958f;//根据摆幅求出角度A,88为摆杆离地高度                   						
	Ax = A*cos(angle*0.017453);	 //计算出X方向摆幅分量0.017453为弧度转换
	Ay = A*sin(angle*0.017453);	 //计算出Y方向摆幅分量
	set_x = Ax*sin(Omega); 		 //计算出X方向当前摆角
	set_y = Ay*sin(Omega+Phase[pOffset]); //计算出Y方向当前摆角
		
	PID_M1_SetPoint(set_x);	//X方向PID跟踪目标值sin
	PID_M1_SetKp(102);	
	PID_M1_SetKi(0.2);	 
	PID_M1_SetKd(10);

	PID_M2_SetPoint(set_y);	//Y方向PID跟踪目标值sin
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
 函数功能:第4问PID计算
 函数说明:
------------------------------------------*/ 
void Mode_4(void)
{	
	if(abs(M1.CurPos)<45.0 && abs(M2.CurPos)<45.0)	//小于45度才进行制动
	{		LED=1;
		PID_M1RA_SetPoint(0);	  //X方向PID定位目标值0
		PID_M1RA_SetKp(0); 		
		PID_M1RA_SetKi(0);     
		PID_M1RA_SetKd(0);     //外环
		
		PID_M1RV_SetKp(100); 		
		PID_M1RV_SetKi(0);     
		PID_M2RV_SetKd(0);    //内环

		PID_M2PA_SetPoint(0);	  //Y方向PID定位目标值0
		PID_M2PA_SetKp(0);  		
		PID_M1PA_SetKi(0);    
		PID_M1PA_SetKd(0);   // 外环
		
		PID_M2PV_SetKp(0);  		
		PID_M1PV_SetKi(0);    
		PID_M2PV_SetKd(0);   // 内环
			
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
 函数功能:第5问PID计算
 函数说明:
------------------------------------------*/
void Mode_5(void)
{
	const float priod = 1410.0;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float set_y = 0.0;
	float A = 0.0;
	float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	LED=1;
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理				
	A = atan((25/36.0f))*57.2958f;    //根据半径求出对应的振幅A
	
	if(RoundDir == 0)       	  
		phase = 3.141592/2.0;		 //逆时针旋转相位差90° 
	else if(RoundDir == 1)  
		phase = (3.0*3.141592)/2.0;	 //顺时针旋转相位差270°
	
	set_x = A*sin(Omega);			 //计算出X方向当前摆角
	set_y = A*sin(Omega+phase); 	 //计算出Y方向当前摆角
	 
	PID_M1_SetPoint(set_x);	//X方向PID跟踪目标值sin
	PID_M1_SetKp(50);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(0);

	PID_M2_SetPoint(set_y);	//Y方向PID跟踪目标值cos
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
 函数功能:第6问PID计算
 函数说明:
------------------------------------------*/
void Mode_6(void)
{

}
/*------------------------------------------
 函数功能:电机底层驱动函数
 函数说明:
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


