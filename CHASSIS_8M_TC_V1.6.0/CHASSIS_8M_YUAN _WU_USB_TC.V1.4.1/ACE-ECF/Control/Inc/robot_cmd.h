#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <struct_typedef.h>
#include "robot_define.h"
#include "bsp_dr16.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define K_FULL_SPEED_SET    (9000.0f/660.0f)//9000拉满 shit键解锁9000
#define SPEED_DEADBAND      5*K_FULL_SPEED_SET

#define DEGREE_2_RAD 0.01745329252f // pi/180
//这一段用的跃鹿里面的物理模型
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

typedef enum
{
    Ready_Fire,   //开摩擦轮
    NO_Fire_FORCE,//关摩擦轮
    On_Fire,      //发射中
    On_Empty      //退弹中
}Fire_State_e;

typedef enum
{
    Gimbal_TC,
    Chassis_RC
}Control_State_e;//实际操控源

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
    fp32            Fire_auto_delay;//自瞄开火延迟
    fp32            Pitch_compensate;//P轴自瞄补偿角度
    fp32            Pitch_angle;
    uint8_t         Fire_ready;
    uint8_t         Open_Auto;//开启自瞄
    
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
