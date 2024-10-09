#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#define CHASSIS 0
#define GIMBAL  1
#define BOARD GIMBAL 
//#define BOARD GIMBAL
#if BOARD == CHASSIS
#include <struct_typedef.h>
#include "robot_define.h"
#include "bsp_dr16.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define K_FULL_SPEED_SET    1.0f//待测最大位移速度/660
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
    Chassis_Mode_t  Chassis_Mode;
	Chassis_State_t Chassis_State;
    fp32            *Difference_Angle_between_Chassis_Gimbal;
    Speed_set_t     Speed_set;
    Fire_State_e      Fire_State;
}Robot_cmd_t;

void CMD_Init(void);
void Chassis_cmd_set(void);
void Fire_Set(void);

Robot_cmd_t* get_Robot_cmd_point(void);
#elif BOARD == GIMBAL
#include <struct_typedef.h>
#include "gimbal_config.h"
#include "virtual_task.h"
#include "bsp_dr16.h"

//目前虚拟串口用来干嘛
typedef enum
{
    IMU,
    ENCODER
}Lock_type_e;

typedef enum
{
    NO_Work,
    MANUAL,              //手动操作
    LONG_SHOOT,//前哨自动开火
    AUTO_ATTACK,         //自瞄
    AUTO_Delay_Fire      //飞行时间延迟发射
}Gimbal_Work_State_e;//云台操控状态

typedef enum
{
    Gimbal_TC,
    Chassis_RC
}Control_State_e;//实际操控者：guojia源

typedef struct{
    fp32 Pitch_Motor_down_encoder;
    Gimbal_Work_State_e Gimbal_Work_State;
    
    #if USB_USE_PURPOSE == KUN_VISION
    KUN_Vision_Control_t*   Vision_control;//kun瞄接口
    #elif USB_USE_PURPOSE == OU_VISION
    fp32 *AUTO_Pitch_Angle_Set_IMU;
    fp32 *AUTO_Yaw_Angle_Set_IMU;
    #elif USB_USE_PURPOSE == YUAN_VISION
    NBNBNB
    #endif
    RC_ctrl_t   *rc_ctl;
    REFEREE_t   *referee_cmd;
    
    Lock_type_e               Pitch_Lock_type;
    Lock_type_e               Yaw_Lock_type;
    
    fp32 Pitch_Set_IMU;
    fp32 Pitch_Set_Encoder;
    fp32 Yaw_Set_IMU;
    int64_t Yaw_Set_Encoder;
    
    Control_State_e     Control_State;
    uint8_t shooter_id1_42mm_speed_limit;
    int16_t Fire_Set_Rpm;
    uint8_t Fire_Ready;
    uint8_t Fire_Open;//摩擦轮实际上开关状态
    uint8_t Pitch_Limit_Max_Flag;
}Gimbal_CMD_t;

const Gimbal_CMD_t* Get_Gimbal_CMD_point(void);
void Gimbal_Deal_TLcontrol(void);
void Gimbal_Choice_Lock_type(void);

void Gimbal_CMD_Set(fp32 Pitch_Up_Angle_Limit, fp32 Pitch_Down_Angle_Limit, fp32 Pitch_Angle, fp32 Pitch_encoder_Angle);
void Gimbal_Angle_Set(fp32 Pitch_Angle_Change, fp32 Yaw_Angle_Change);
void Gimbal_Fire_State_Set(void);
void Deal_Fire_Set_Rpm(void);
void Gimbal_Pitch_Set_Encoder(fp32 Set_Encoder);
void Gimbal_Pitch_Set_IMU(fp32 Set_IMU);
void Gimbal_Yaw_Set_Encoder(int64_t Set_Encoder);
void Gimbal_Yaw_Set_IMU(fp32 Set_IMU);

void Gimbal_Pitch_Lock(Lock_type_e Lock_type);
void Gimbal_Yaw_Lock(Lock_type_e   Lock_type);
void Set_Pitch_Motor_down_encoder(fp32 Pitch_Motor_down_encoder);
void Set_Fire_Open_State(uint8_t State);
//给视觉任务用的
uint8_t get_fire_ready();
fp32* get_Gimbal_pitch_point();
fp32* get_Gimbal_yaw_point();
uint8_t get_rc_fire_control(void);
Gimbal_Work_State_e get_open_auto_control(void);
Gimbal_Work_State_e get_open_auto_delay_control(void);

Gimbal_Work_State_e* get_gimbal_behaviour_point();

void Send_Fire_Set_Rpm(void);
void Send_Fire_Motor_Speed(uint16_t LF, uint16_t RF, uint16_t LB, uint16_t RB);
void Send_TC_to_Chassis(void);
void Deal_Dr16_form_chassis(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data);
void Send_Chassis_vision_fire_static(uint8_t lock_flag, uint8_t fire_flag, uint8_t fire_ready, uint8_t open_auto, fp32 Pitch_Angle, uint8_t encoder_lock);
#endif

#endif
