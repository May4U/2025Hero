#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

#include "main.h"
#include "tim.h"

void Servo_Init(TIM_HandleTypeDef *htim,uint32_t ch);
void Servo_SetAngle(TIM_HandleTypeDef* htim,uint32_t ch,float angle);

#endif
