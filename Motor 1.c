#include "main.h"
#include "tim.h"
#include "Motor.h"
uint16_t Val_A=0;
uint16_t Val_B=0;
extern tPid MotorA_pid;
extern tPid MotorB_pid;
extern float MotorA_Speed;
extern float MotorB_Speed;
void MotorA_Run(int16_t PWM_Val){
	if(PWM_Val>=0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); 
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM_Val);
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); 
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM_Val);
	}
	
}
void MotorB_Run(int16_t PWM_Val){
	if(PWM_Val>=0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,PWM_Val);
	}else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,PWM_Val);
	}
}
/*
void MotorA_Speed_Set(float rps){
    if (MotorA_Speed<=rps-0.1)
    {
      Val_A+=1;
    }
    else if (MotorA_Speed>=rps+0.1)
    {
      Val_A-=1;
    }
	MotorA_Run(Val_A);
}
void MotorB_Speed_Set(float rps){
	if (MotorB_Speed<=rps-0.1)
    {
      Val_B++;
    }
    else if (MotorB_Speed>=rps+0.1)
    {
      Val_B--;
    }
	
     
     MotorB_Run(Val_B);
	
}*/
void PIDA_Init(tPid *PID){
	PID->actual_val=0.0;
	PID->target_val=2.0;
	PID->err=0.0;
	PID->err_last=0.0;
	PID->err_sum=0.0;
	PID->Kp=100.0;
	PID->Ki=20.0;
	PID->Kd=1.0;

}
void PIDB_Init(tPid *PID){
	PID->actual_val=0.0;
	PID->target_val=2.0;
	PID->err=0.0;
	PID->err_last=0.0;
	PID->err_sum=0.0;
	PID->Kp=100.0;
	PID->Ki=20.0;
	PID->Kd=1.0;

}

// PID¿ØÖÆº¯Êý
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//ä¼ é€’çœŸå®žå€¼
	pid->err = pid->target_val - pid->actual_val;////å½“å‰è¯¯å·®=ç›®æ ‡å€¼-çœŸå®žå€¼
	pid->err_sum += pid->err;//è¯¯å·®ç´¯è®¡å€¼ = å½“å‰è¯¯å·®ç´¯è®¡å’Œ
	//ä½¿ç”¨PIDæŽ§åˆ¶ è¾“å‡º = Kp*å½“å‰è¯¯å·®  +  Ki*è¯¯å·®ç´¯è®¡å€¼ + Kd*(å½“å‰è¯¯å·®-ä¸Šæ¬¡è¯¯å·®)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//ä¿å­˜ä¸Šæ¬¡è¯¯å·®: è¿™æ¬¡è¯¯å·®èµ‹å€¼ç»™ä¸Šæ¬¡è¯¯å·®
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

void motorPidSetSpeed(float MotorASetSpeed,float MotorBSetSpeed)
{
	//¸Ä±äµç»úPID²ÎÊýµÄÄ¿±êËÙ¶È
	MotorA_pid.target_val = MotorASetSpeed;
	MotorB_pid.target_val = MotorBSetSpeed;
	//¸ù¾ÝPID¼ÆËã Êä³ö×÷ÓÃÓÚµç»ú
	MotorA_Run(PID_realize(&MotorA_pid,MotorA_Speed));
	MotorB_Run(PID_realize(&MotorB_pid,MotorB_Speed));
}













