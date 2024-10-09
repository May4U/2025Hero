/*code is far away from bug with the animal protecting
 *  ┏┓　　　┏┓
 *┏┛┻━━━┛┻┓
 *┃　　　　　　　┃ 　
 *┃　　　━　　　┃
 *┃　┳┛　┗┳　┃
 *┃　　　　　　　┃
 *┃　　　┻　　　┃
 *┃　　　　　　　┃
 *┗━┓　　　┏━┛
 *　　┃　　　┃PC、BJ保佑
 *　　┃　　　┃代码无BUG！
 *　　┃　　　┗━━━┓
 *　　┃　　　　　　　┣┓
 *　　┃　　　　　　　┏┛
 *　　┗┓┓┏━┳┓┏┛
 *　　　┃┫┫∞┃┫┫
 *　　　┗┻┛||┗┻┛
 *　　　       u
 */

// #include "gimbal_config.h"
#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#define FIRE_WORK

//#define VIRTUAL_DELAY_COMPENSATE

#define YAW_USE_PID 			0
#define YAW_USE_LQR 			1
#define YAW_CONTROLER 		YAW_USE_LQR

#define PITCH_USE_M2006			0
#define PITCH_USE_MI		    1
#define PITCH_USE_G6020         3
#define PITCH_MOTOR 		PITCH_USE_G6020

#define PITCH_ZERO_OFFSET 100.0f //p轴中位偏移值
#define YAW_ZERO_OFFSET 1541.0f //p轴中位偏移值

/**********************pitch轴PID参数**********************/
#define GIMBAL_PITCH_P_P 1000.0f
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 100.0f

#define GIMBAL_PITCH_S_P 10.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************pitch轴自瞄PID参数**********************/
#define GIMBAL_PITCH_visual_P_P 800.0f
#define GIMBAL_PITCH_visual_P_I 0.08f
#define GIMBAL_PITCH_visual_P_D 0.0f

#define GIMBAL_PITCH_visual_S_P 35.0f
#define GIMBAL_PITCH_visual_S_I 0.0f
#define GIMBAL_PITCH_visual_S_D 0.0f

/**********************Yaw轴PID参数**********************/
#define GIMBAL_YAW_P_P 300.0f
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 100.0f

#define GIMBAL_YAW_S_P 10.0f
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

#define GIMBAL_YAW_visual_P_P 0.0f
#define GIMBAL_YAW_visual_P_I 5.0f
#define GIMBAL_YAW_visual_P_D 0.0f

#define GIMBAL_YAW_visual_S_P 1.0f
#define GIMBAL_YAW_visual_S_I 0.0f
#define GIMBAL_YAW_visual_S_D 0.0f
/**********************云台pitch角度限制**********************/
#define Using_MI_Motor_Init
#define PITCH_ANGLE_RANGE    (56.0f)
#define PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE     (-43.0f)
#define PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE   (14.51f)
#define MOTOR_PITCH_ANGLE_LIMIT_UP      WHILE_PITCH_0_MOTOR_ACTUAL_ANGLE - PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE
#define MOTOR_PITCH_ANGLE_LIMIT_DOWN    WHILE_PITCH_0_MOTOR_ACTUAL_ANGLE - PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE

#define PITCH_MOTOR_UP_MIN_ACTUAL_ENCODER 5000
#define PITCH_MOTOR_DOWN_MAX_ACTUAL_ENCODER 5700
/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED 0.006f   //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED 0.005f //鼠标pitch轴速度增益
#define RC_YAW_SPEED 0.0003f     //遥控器yaw轴速度增益
#define RC_PITCH_SPEED 0.0003f   //遥控器pitch轴速度增益
/**********************虚拟串口用法**********************/
#define KUN_VISION 0
#define OU_VISION 1
#define NAVIGATION 2
#define YUAN_VISION 3
#define USB_USE_PURPOSE OU_VISION
#endif
