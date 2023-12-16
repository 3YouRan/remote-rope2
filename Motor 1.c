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

// PID���ƺ���
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

void motorPidSetSpeed(float MotorASetSpeed,float MotorBSetSpeed)
{
	//�ı���PID������Ŀ���ٶ�
	MotorA_pid.target_val = MotorASetSpeed;
	MotorB_pid.target_val = MotorBSetSpeed;
	//����PID���� ��������ڵ��
	MotorA_Run(PID_realize(&MotorA_pid,MotorA_Speed));
	MotorB_Run(PID_realize(&MotorB_pid,MotorB_Speed));
}













