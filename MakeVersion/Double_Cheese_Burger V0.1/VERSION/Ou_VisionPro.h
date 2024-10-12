#ifndef __PC_VISIONPRO_H
#define __PC_VISIONPRO_H

#include "gimbal_struct_variables.h"

////单方向空气阻力模型
//extern float monoDirectionalAirResistanceModel(float Distance, float speed, float rad);
//完全空气阻力模型
extern float completeAirResistanceModel(gimbal_auto_control_t *auto_control_p);
//pitch弹道补偿
extern float pitchTrajectoryCompensation(float s, float y, float v);
//根据最优决策得出被击打装甲板 自动解算弹道
extern void autoSolveTrajectory(gimbal_auto_control_t *auto_control_p ,uint8_t receive_virtual_flag);

int PC_VisionPro_Init(void);

float PC_Calc_Pitch_Angle(float Y_, float X_, float *Fly_time, float speed_bullet);
#endif
