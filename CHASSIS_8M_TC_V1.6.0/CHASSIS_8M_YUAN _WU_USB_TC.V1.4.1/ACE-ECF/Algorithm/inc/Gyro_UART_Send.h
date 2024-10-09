#ifndef __Gyro_UART_Send_H
#define __Gyro_UART_Send_H
//#include "usart.h"

#include "stdint.h"

//返回数组元素的个数
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )



typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

/*****视觉相关结构体****/
typedef struct
{
    float auto_pitch_angle; //视觉传回P轴差角
    float auto_yaw_angle;   //视觉传回Y轴差角

    float auto_kalman_pitch_angle; //视觉传回P轴差角
    float auto_kalman_yaw_angle;   //视觉传回Y轴差角
    float last_auto_pitch_angle;   //上一次视觉传回P轴差角
    float last_auto_yaw_angle;     //上一次视觉传回Y轴差角

    float len;   //视觉传回距离
    int8_t fire; //是否开火

    float auto_yaw_angular_velocity;     //视觉回传数据算出的角速度
    float auto_last_yaw_angular_velocity;     //视觉回传数据算出的上一次的角速度
    float auto_yaw_angular_acceleration; //视觉回传数据算出的角加速度

    float auto_pitch_sum;      //传回P轴角度积分
    float auto_pitch_angle_kf; //电控卡尔曼处理后的P轴差角
    float auto_yaw_angle_kf;   //电控卡尔曼处理后的Y轴差角
		float auto_pitch_speed;
	
	
    int16_t auto_lost_data_pit_count; //丢失目标计数
    int16_t auto_lost_data_pit_flag;  //丢失目标标志位
    int16_t auto_lost_data_yaw_count; //丢失目标计数
    int16_t auto_lost_data_yaw_flag;  //丢失目标标志位
    int16_t auto_yaw_zero_flag;   //yaw轴0值标志位

    float pitch_control_data;      //P轴视觉控制量
    float yaw_control_data;        //y轴视觉控制量
    float last_pitch_control_data; //上一次P轴视觉控制量
    float last_yaw_control_data;   //上一次y轴视觉控制量
		
		float xxx_speed; 

		float auto_pitch_angle_speed;
    int32_t world_time1;
		int32_t last_world_time1;
} vision_auto_data_t;



void USART_Send(void);
void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed,float yaw,float pitch);
void Gyro_Send_Task(void *pvParameters);
float invSqrt(float x) ;
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
extern float hex2Float(uint8_t HighByte, uint8_t LowByte);
extern void visual_data_reception(void);
vision_auto_data_t *get_auto_control_point(void);
#endif

