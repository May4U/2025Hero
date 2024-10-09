#ifndef CHASSIS_MOVEMENT_H
#define CHASSIS_MOVEMENT_H
/*********FreeRTOS头文件包含*********/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include <struct_typedef.h>
#include "filter.h"
#include "PID_Data.h"
#include "robot_cmd.h"
#include "DJI_Motor.h"
#include "bsp_referee.h"
#include "pm01_api.h"

typedef struct{
    const REFEREE_t     *referee;
    Robot_cmd_t*  Robot_cmd;
    pm01_od_t           *pm01_od;
    
    uint8_t Max_Power;
    uint8_t New_Power;

    
    DJIMotor_object_t *RF_MOTOR;
    DJIMotor_object_t *RB_MOTOR;
    DJIMotor_object_t *LB_MOTOR;
    DJIMotor_object_t *LF_MOTOR;
    DJIMotor_object_t *FIRE_MOTOR;
}Chassis_t;
void Chassis_Movement(void const *argument);
const fp32* Get_Difference_Angle_between_Chassis_Gimbal(void);  
Chassis_t *get_chassis_point(void);
int64_t    get_fire_motor_encoder(void);
#endif