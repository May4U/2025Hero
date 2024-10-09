#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H
//#include "Chassis_movement_task.h"
#include "Chassis_task.h"
#include "DJI_Motor.h"
#include "PID_Data.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct{
    Chassis_t   *Chassis;
    int64_t     set_position;   
    int64_t     last_set_position;
    int64_t     Empty_position;//启动退弹时设定位置
}fire_task_t;
void Fire_Task(void const *argument);

#endif

