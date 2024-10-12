/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_stepin.c
 * @author  cayon
 * @version V0.2.0
 * @date    2023/11/01
 * @brief
 ******************************************************************************
 * @verbatim
 *  驱动步进电机
 *  使用方法：
 *  在.h文件中修改所用到的定时器与GPIO输出口相关宏定义
 *  共有两种操作方式
 *  在.h文件中修改操作方式
 *  1为位置模式
 *  0为速度模式
 *  若为位置模式
 *  调用StepIn_MotorAngle(方向，脉冲数);
 *  若为速度模式StepIn_MotorSet(方向，是否使能);
 *  初始化：
 *
 *  demo：
 *       StepIn_MotorInit(&htim2);
 *       StepIn_MotorAngle(1,100);//位置模式
 *       StepIn_MotorSet(1,1);//速度模式
 *       
 * @attention
 *      
 *      
 * @version
 * v0.1.0 完成基本功能，所有功能已验证
 ************************** Dongguan-University of Technology -ACE***************************/


#include "bsp_stepin.h"
#include "tim.h"

long Pulse_CNT=0;

static void StepIn_MotorENABLE(uint8_t enable);
static void StepIn_MotorDirection(uint8_t dir);

/**
  * @brief  步进电机初始化
  * @param  htim TIM handle
  *   开启所用步进电机的对应定时器中断与pwm输出
  * @retval 无
  */
void StepIn_MotorInit(TIM_HandleTypeDef *htim)
{
    if(STEPIN_MODE==1)//如果为位置模式开启定时器中断
    {
         HAL_TIM_Base_Start_IT(htim);//开启定时器中断 
    }
    __HAL_TIM_SET_COMPARE(&STEPIN_HTIM,CH,0);//默认关闭PWM输出
    HAL_TIM_PWM_Start(htim,CH);//步进电机所用通道在.h里改
    StepIn_MotorENABLE(0);//初始化后使能步进电机
    StepIn_MotorDirection(1);//默认正向
}

/**
  * @brief  步进电机使能
  * @param  是否使能，1为使能/0为不使能
  *   
  * @retval 无
  */
void StepIn_MotorENABLE(uint8_t enable)
{
    if(enable)
    {
        HAL_GPIO_WritePin(STEPIN_PORT,STEPIN_PIN_ENABLE,GPIO_PIN_SET);//步进电机使能引脚在.h里改
    }
    else
    {
        HAL_GPIO_WritePin(STEPIN_PORT,STEPIN_PIN_ENABLE,GPIO_PIN_RESET);
    }
}

/**
  * @brief  步进电机方向设定
  * @param  1为正向  0为逆向
  *   
  * @retval 无
  */
void StepIn_MotorDirection(uint8_t dir)
{
    if(dir)
    {
        HAL_GPIO_WritePin(STEPIN_PORT,STEPIN_PIN_DIRT,GPIO_PIN_SET);//步进电机使能引脚在.h里改
    }
    else
    {
        HAL_GPIO_WritePin(STEPIN_PORT,STEPIN_PIN_DIRT,GPIO_PIN_RESET);
    }
}

/*位置模式设定位置*/
/**
  * @brief  步进电机旋转角度设定
  * @param  dir:1为正向  0为逆向
  *          set:旋转度数
  * @retval 无
  */
//100个脉冲转11度
void StepIn_MotorAngle(uint8_t dir,long set)
{
    StepIn_MotorENABLE(1);
    StepIn_MotorDirection(dir);
    Pulse_CNT=set;
    __HAL_TIM_SET_COMPARE(&STEPIN_HTIM,CH,625);//设定完后开启PWM输出，使中断产生
}

/*速度模式设定速度*/
void StepIn_MotorSet(uint8_t dir,uint8_t enable)
{
     StepIn_MotorDirection(dir);
     if(enable)
     {
         __HAL_TIM_SET_COMPARE(&STEPIN_HTIM,CH,625);//开启PWM输出
     }
     else
     {
         __HAL_TIM_SET_COMPARE(&STEPIN_HTIM,CH,0);//关闭PWM输出
     }
}

/**
  * @brief  步进电机定时器中断回调函数
  * @param  htim TIM handle
  * @retval 无
  */
//此处设定TB6600为3200个脉冲步进电机转1圈
//100个脉冲转11度
void StepIn_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == STEPIN_TIM)
    {
        if(Pulse_CNT!=0)
        {
            Pulse_CNT--;         //步进电机计数脉冲
        }
        else
        {
            StepIn_MotorENABLE(0);//当没有输入信号产生时，不使能步进电机
            __HAL_TIM_SET_COMPARE(&STEPIN_HTIM,CH,0);//同时关闭PWM产生输出
        }
    }
}
