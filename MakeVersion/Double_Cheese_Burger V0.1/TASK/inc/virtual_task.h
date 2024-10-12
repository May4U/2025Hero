#ifndef __TASK_VIRTUAL_H  //如果未定义
#define __TASK_VIRTUAL_H  //那么定义
#include "gimbal_config.h"
#if USB_USE_PURPOSE == OU_VISION
#include "gimbal_struct_variables.h"

extern void VIRTUAL_TASK(void const * argument);

void gimbal_clear_virtual_recive(void);
uint8_t *get_auto_fire_point(void);
uint8_t *get_tracking_point(void);
gimbal_auto_control_t *get_auto_control_point(void);
float* get_AutoPitch(void);
float* get_AutoYaw(void);
uint8_t Get_Virtual_task_init_falg(void);
void Flash_fire_bias_time(uint8_t Start_0_End_1);
#endif

#if USB_USE_PURPOSE == KUN_VISION
#include "struct_typedef.h"
typedef union   
{
    float f;
    uint8_t u8[4];
}u8_and_float;

typedef struct{
    uint8_t tracking;
    uint8_t id;
    uint8_t fire_control;
    u8_and_float x;
    u8_and_float y;
    u8_and_float z;
    uint8_t in_place_falg;
}KUN_Vision_Receive_t;

typedef struct{
    fp32    AutoPitch;
    fp32    AutoYaw;
    uint8_t Fire_flag;
}KUN_Vision_Control_t;

extern void Virtual_Task(void const * argument);
KUN_Vision_Control_t *get_auto_control_point(void);
uint8_t Get_Virtual_task_init_falg(void);
#endif

#endif


