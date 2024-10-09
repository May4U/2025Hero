#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <struct_typedef.h>
#include "robot_define.h"
#include "bsp_dr16.h"

/* ����robot_def.h�е�macro�Զ�����Ĳ��� */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // �����
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // ���־�
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // �����ܳ�

#define K_FULL_SPEED_SET    (9000.0f/660.0f)//9000���� shit������9000
#define SPEED_DEADBAND      5*K_FULL_SPEED_SET

#define DEGREE_2_RAD 0.01745329252f // pi/180
//��һ���õ�Ծ¹���������ģ��
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

typedef enum
{
    Ready_Fire,   //��Ħ����
    NO_Fire_FORCE,//��Ħ����
    On_Fire,      //������
    On_Empty      //�˵���
}Fire_State_e;

typedef enum
{
    Gimbal_TC,
    Chassis_RC
}Control_State_e;//ʵ�ʲٿ�Դ

typedef enum
{
		NO_FOLLOW,
		FOLLOW,
		SPIN,
        RESPIN,
	    NO_FORCE,
    #ifdef USE_NAVIGATION
        NAVIGATION
    #endif
}Chassis_Mode_t;

typedef enum
{
	LOCK_POSITION,
	SPEED
}Chassis_State_t;

typedef struct{
    fp32  gambal_x;
    fp32  gambal_y;
    
    fp32  chassis_x;
    fp32  chassis_y;
    fp32  chassis_z;
    
    fp32  RF_motor;
    fp32  RB_motor;
    fp32  LB_motor;
    fp32  LF_motor;
}Speed_set_t;

typedef struct{
    uint8_t         AUTO_LOCK_Flag;
    uint8_t         AUTO_Fire_Flag;
    uint8_t         Pitch_Lock_Encoder_Flag;
    fp32            Fire_auto_delay;//���鿪���ӳ�
    fp32            Pitch_compensate;//P�����鲹���Ƕ�
    fp32            Pitch_angle;
    uint8_t         Fire_ready;
    uint8_t         Open_Auto;//��������
    
    uint16_t         LU_FireMoterSpeed;
    uint16_t         RU_FireMoterSpeed;
    uint16_t         LD_FireMoterSpeed;
    uint16_t         RD_FireMoterSpeed;
    int16_t          FireMotorSetRpm;
}Gimbal_Info_t;
typedef struct{
    Chassis_Mode_t  Chassis_Mode;
	Chassis_State_t Chassis_State;

    uint8_t         Power_Mode;
    
    fp32            Difference_Angle_between_Chassis_Gimbal;
    Speed_set_t     Speed_set;
    Fire_State_e    Fire_State;
    Control_State_e Control_State;
    RC_ctrl_t       *RC;
    uint8_t         increat_time;
    Gimbal_Info_t   Gimbal_Info;
}Robot_cmd_t;

void CMD_Init(void);
void Chassis_cmd_set(void);
void Fire_Set(void);
void Send_Dr16_to_Gimbal(void);
void Send_Shoot_Speed_to_Gimbal(void);
void Send_Pitch_Limit_More(void);

Robot_cmd_t* get_Robot_cmd_point(void);
Control_State_e Get_Chassis_Control_State(void);
uint8_t Get_fire_ready(void);
uint8_t Get_gimbal_open_auto(void);
uint8_t Get_AUTO_LOCK_Flag(void);
fp32 Get_Pitch_compensate(void);
fp32 Get_Fire_auto_delay(void);
fp32 Get_Gimbal_Pitch_Angle(void);
fp32 Get_TOF_Distance(void);
uint8_t Get_Gimbal_Pitch_Mode(void);
uint16_t Get_Gimbal_Fire_Motor_Speed(uint8_t idx);
fp32 Get_dif_angle(void);
int16_t Get_Fire_Motor_Set_Rpm(void);
#endif
