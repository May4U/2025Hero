#ifndef CHASSIS_MOVEMENT_H
#define CHASSIS_MOVEMENT_H
/*********FreeRTOS头文件包含*********/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include <struct_typedef.h>
#include "filter.h"
#include "CAN1.h"
#include "CAN2.h"
#include "pid.h"
#include "bsp_Motor_Encoder.h"
#include "PID_Data.h"
#include "robot_cmd.h"



typedef struct{
    uint8_t CanId;//Can通信id
    int8_t  reverse_flag;//反转正负号
    Encoder_Type_e  Encoder_type; //电机种类
    const Motor_2006_3508_6020_t    *Motor_Information;//电机CAN回传信息
    Encoder_t           *Motor_encoder;//电机编码器信息
    pid_parameter_t     Speed_PID;//速度环PID
	pid_parameter_t	    Position_PID;//位置环PID
    int16_t             current_input;//电调输出电流，仅2006和3508有,-10000到100000，对应-10A到10A
    int16_t             voltage_input;//驱动器的电压输出，仅6020有
}Motor_information_t;

typedef struct{
    const Robot_cmd_t*  Robot_cmd;
    Motor_information_t RF_MOTOR;
    Motor_information_t RB_MOTOR;
    Motor_information_t LB_MOTOR;
    Motor_information_t LF_MOTOR;
}Chassis_t;
#endif