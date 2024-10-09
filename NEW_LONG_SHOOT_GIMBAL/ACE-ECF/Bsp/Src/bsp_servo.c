/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_servo.c
 * @author  cayon
 * @version V0.1.0
 * @date    2023/10/24
 * @brief
 ******************************************************************************
 * @verbatim
 *  驱动舵机
 *  使用方法：
 *  初始化舵机
 *
 *  设定角度
 *
 *  demo：
 *        Servo_Init(&htim1,1);//初始化tim1的定时器通道1
 *       角度设定：Servo_SetAngle(&htim1,TIM_CHANNEL_1,50)//表示舵机旋转50度
 *       
 * @attention
 *      
 *      
 * @version
 * v0.1.0 完成基本功能，所有功能已验证
 ************************** Dongguan-University of Technology -ACE***************************/

#include "bsp_servo.h"


/**
  * @brief  舵机PWM初始化.
  * @param  htim TIM handle
  * @param  Channel TIM Channels to be enabled
  * @retval 无
  */
void Servo_Init(TIM_HandleTypeDef *htim,uint32_t ch)
{
    switch(ch)
    {
        case 1:
            HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
        break;
        
        case 2:
            HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);
        break;
        
        case 3:
            HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);
        break;
        
        case 4:
            HAL_TIM_PWM_Start(htim,TIM_CHANNEL_4);
        break;
            
        default:
        break;
    }
}

/**
  * @brief  舵机设定旋转角度
  * @param  htim TIM handle
  *         demo：TIM1\TIM2...
  * @param  __CHANNEL__ TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  angle
  *          旋转角度设定
  *           0~180度
  * @retval 无
  */

void Servo_SetAngle(TIM_HandleTypeDef* htim,uint32_t ch,float angle)
{
    static uint32_t temp;
     switch(ch)
    {
        case 1:
            temp=TIM_CHANNEL_1;
        break;
        
        case 2:
            temp=TIM_CHANNEL_2;
        break;
        
        case 3:
            temp=TIM_CHANNEL_3;
        break;
        
        case 4:
            temp=TIM_CHANNEL_4;
        break;
            
        default:
        break;
    }
    __HAL_TIM_SET_COMPARE(htim,temp,((angle/270.0f)*2000+500));
}
