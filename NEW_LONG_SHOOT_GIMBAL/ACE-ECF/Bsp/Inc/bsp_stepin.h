#ifndef __BSP_STEPIN_H__
#define __BSP_STEPIN_H__

#include "main.h"

#define STEPIN_MODE 0//0为速度模式，1为位置模式

#define STEPIN_TIM TIM2
#define STEPIN_HTIM htim2
#define CH TIM_CHANNEL_1
#define STEPIN_PORT GPIOE
#define STEPIN_PIN_ENABLE GPIO_PIN_13
#define STEPIN_PIN_DIRT GPIO_PIN_14

extern long Pulse_CNT;

void StepIn_MotorInit(TIM_HandleTypeDef *htim);
void StepIn_MotorAngle(uint8_t dir,long set);
void StepIn_MotorSet(uint8_t dir,uint8_t enable);
void StepIn_Callback(TIM_HandleTypeDef *htim);



#endif
