#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H
#include "gimbal_struct_variables.h"
#include "DJI_Motor.h"
#include "robot_cmd.h"
typedef struct{
    DJIMotor_object_t*   right_motor_up;
    DJIMotor_object_t*   left_motor_up;
    DJIMotor_object_t*   left_motor;
    DJIMotor_object_t*   right_motor;
    const Gimbal_CMD_t*  Gimbal_CMD;
}Fire_t;
void FIRE_TASK(void const *argument);


#define FIRE_SPEED_0 0
#define FIRE_SPEED_10 3270
#define FIRE_SPEED_16 4900

#endif
