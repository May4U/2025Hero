/*
_ooOoo_
o8888888o
88" . "88
(| -_- |)
 O\ = /O
___/`---'\____
.   ' \\| |// `.
/ \\||| : |||// \
/ _||||| -:- |||||- \
| | \\\ - /// | |
| \_| ''\---/'' | |
\ .-\__ `-` ___/-. /
___`. .' /--.--\ `. . __
."" '< `.___\_<|>_/___.' >'"".
| | : `- \`.;`\ _ /`;.`/ - ` : | |
\ \ `-. \_ __\ /__ _/ .-` / /
======`-.____`-.___\_____/___.-`____.-'======
`=---='

.............................................
佛曰：bug泛滥，我已瘫痪！
*/

//  #include "gimbal_struct_variables.h"
#ifndef __GIMBAL_STRUCT_VARIABLES_H
#define __GIMBAL_STRUCT_VARIABLES_H

#include "main.h"
#include "struct_typedef.h"

/*  系统头文件 */
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
#include <stdbool.h>
// #include "stdint.h"

///* ************************FreeRTOS******************** */
// #include "freertos.h"
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"
// #include "cmsis_os.h"

/******************** BSP ********************/
#include "bsp_dr16.h"
//#include "bsp_motor.h"
#include "bsp_referee.h"
/********************ALGORITHM********************/
#include "pid.h"
#include "fifo.h"
#include "lqr.h"
#include "filter.h"
/********************REFEREE********************/
// #include "crc.h"
// #include "referee_deal.h"
#include "imu_task.h"

typedef enum
{
	GIMBAL_MANUAL,		 // 手动状态
	GIMBAL_AUTOATTACK,	 // 自瞄状态
	GIMBAL_SMALL_AUTOBUFF,	 // 小符状态
	GIMBAL_BIG_AUTOBUFF,	//大符状态
	GIMBAL_REPLENISHMEN, // 补给状态
    GIMBAL_ZERO_FORCE//无力状态
} gimbal_behaviour_e;

typedef enum
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
} ARMOR_ID;

typedef enum
{
    ARMOR_NUM_NOT = 0,
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
} ARMOR_NUM;


//用于存储目标装甲板的信息
typedef struct
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
    float angle_yaw;   //角度制下的装甲板坐标系相对于世界坐标系的yaw角
    float distance;    //装甲板坐标系相对于世界坐标系的直线距离
    uint8_t idx;
} tar_pos;

typedef enum
{
	FIRE_OFF,		 
	FIRE_FULL_AUTO,
} fire_behaviour_e;

//typedef enum
//{
//	GIMBAL_ZERO_FORCE, // 云台无力
//	GIMBAL_NORMAL,	   //
//} gimbal_state_e;


//TODO:修改pitch？好像也可以不用修
typedef struct
{
	uint8_t visual_buff_send[29];
	fifo_s_t *usb_fifo;
	float auto_pitch_speed;
	
	float history_gimbal_data[2]; //第一个pitch，第二个yaw
	float gimbal_use_control[2];

	const gimbal_behaviour_e *gimbal_behaviour;
	const INS_t *Imu_c;
	const float *gimbal_yaw;
	const float *gimbal_pitch;
	const REFEREE_t *referee;
	//const gimbal_pitch_control_t *Pitch_c;
	
	//淐喵Pro
	bool reset_tracker : 1;
		
	uint8_t tracking;
  ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                              //1-英雄 2-工程 3-4-5-步兵 
  ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
	uint8_t reserved;
    
  float xw,yw,zw;       //ROS坐标系下的x,y,z
  float yaw;
  float vxw,vyw,vzw;    //ROS坐标系下的vx,vy,vz
  float v_yaw;          //目标yaw速度
  float r1;             //目标中心到前后装甲板的距离
  float r2;             //目标中心到左右装甲板的距离
  float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
	
  int bias_time;        //偏置时间
  float s_bias;         //枪口前推的距离
  float z_bias;         //yaw轴电机到枪口水平面的垂直距离
	float auto_yaw;
	float auto_pitch;
	
  tar_pos Target_Position;
	
	//重力补偿	
	float bullet_speed;
	float max_iter;
	float R_K_iter;
	float stop_error;
	float k;
	float g;
    
    const   RC_ctrl_t   *RC;
    uint8_t Get_out_post;
    uint8_t fire_flag;
	fp32    fly_time_ms;
    fp32    auto_fire_delay_time_ms;
    fp32    Next_Can_Fire_time;
    fp32    pitch_compensate;
    fp32    yaw_compensate;
    fp32    last_receive_time_ms;
} gimbal_auto_control_t;

// 火控//TODO:修改左右摩擦轮和开火的返回数据
typedef struct
{
	fire_behaviour_e fire_behaviour;
//	Motor3508_t right_motor;
//	Motor3508_t left_motor;
//	Motor3508_t fire_motor;
//	Encoder_t *fire_motor_encoder;
	pid_parameter_t right_motor_speed_pid;
	pid_parameter_t left_motor_speed_pid;
	pid_parameter_t fire_motor_speed_pid;
	pid_parameter_t fire_motor_position_pid;
	
	bool full_automatic;
	volatile bool replenish_flag;
	
	uint8_t *fire;
	uint8_t *tracking;
	const RC_ctrl_t *fire_rc;
	const REFEREE_t *referee;
} gimbal_fire_control_t;

#endif
