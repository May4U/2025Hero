#ifndef __BSP_Parabloic_Calc
#define __BSP_Parabloic_Calc
#define PLAN_A 0    //����A��ˮƽ��ֱ�����ǿ������������η���ϵ
#define PLAN_B 1    //����B����ˮƽ�����ǿ������������η���ϵ
#define PLAN PLAN_B
#include "struct_typedef.h"
float Calc_Pitch_Angle(float Y_, float X_, float *Fly_time);
#if PLAN == PLAN_A
float fly_time(float Distance, float cos);
#endif
float Yaw_Angle_close(float X_, float Y_);
float Calc_The_Distance(float X_,float Y_);
float calc_fire_time(float now_in_place_time, float fly_time);
uint8_t JudgeCanFire(float now_in_place_time, float *next_in_place_time);
#endif
