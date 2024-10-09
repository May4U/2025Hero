#ifndef __FSM_CHASSIS_H
#define __FSM_CHASSIS_H

#include "bsp_dr16.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "stdint.h"

#define K_FULL_SPEED_SET    13.1212f//ʵ������Ϊ8660rpm����ϵ��Ϊ8660/660
#define SPEED_DEADBAND      5*K_FULL_SPEED_SET
#define X                   0
#define Y                   1
#define Z                   2

    

typedef enum
{
		NO_FOLLOW,
		FOLLOW,
		SPIN,
	    NO_FORCE,
}Chassis_Mode_t;

typedef enum
{
	LOCK_POSITION,
	SPEED
}Chassis_State_t;

typedef struct 
{
	Chassis_Mode_t Chassis_Mode;
	Chassis_State_t Chassis_State;
	
	//��������Ϊ�ʵ㽨����ƽ������ϵ������X��������Y�����Ե���ͷΪY����
    fp32 plane_x_speed_set;//���ӵ�ƽ����ᣬ����Ϊ��
    fp32 plane_y_speed_set;//���ӵ�ƽ�����ᣬ����Ϊ��
	fp32 plane_z_speed_set;//���ӵ�ƽ����ת�ᣬ��ʱ��Ϊ��

	fp32 LF_speed_set;
	fp32 RF_speed_set;
	fp32 LB_speed_set;
	fp32 RB_speed_set;
	
	fp32 Difference_Angle_between_Chassis_Gimbal;
	
	const RC_ctrl_t *RC;
	REFEREE_t *referee;
	const INS_t *INS;
	
	int clear_lag;
	
}FSM_Chassis_t;

FSM_Chassis_t *Get_FSM_Chassis_Address(void);
void Mode_Deal(void);
void State_Deal(void);
void FSM_Chassis_Init(void);
void Chassis_X_Y_Speed_Calc(void);

void ALL_OK_Motion_Calc(void);
void Miss_RF_Motion_Calc(void);
void Miss_LF_Motion_Calc(void);
void Miss_LB_Motion_Calc(void);
void Miss_RB_Motion_Calc(void);
#endif



