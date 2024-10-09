#ifndef _CAN1_H
#define _CAN1_H

#include "stm32f4xx.h"

#define CHASSIS_MOTOR_GENERAL_ID 0x200
#define CHASSIS_MOTOR_LF_ID      0x201
#define CHASSIS_MOTOR_RF_ID      0x202
#define CHASSIS_MOTOR_LB_ID      0x203
#define CHASSIS_MOTOR_RB_ID		 0x204

#define FIRE_MOTOR_GENERAL_ID    0x1FF
#define FIRE_MOTOR_ID            0x205

#define SUPERCAP_ID							 0x211


typedef struct 
{
	int16_t speed_data;
	int16_t position_data;
	int16_t last_position_data;
	uint8_t temperature;
	int16_t current;
} Motor_2006_3508_6020_t;

typedef struct
{
	float input_voltage;	   //输入电压
	float Capacitance_voltage; //电容电压
	float Input_current;	   //输入电流
	float Set_power;		   //设定功率
} Supercapacitor_receive_t;	   //超级电容器

void CAN1_Init(void);
void CAN1_filter_config(void);

void CAN1_Chassis_Tx( int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor);
void CAN1_Fire_Tx( int16_t FIRE_Motor);
void Can1_Cap_Setmsg(int16_t Chassis_power);
void Motor_Online_Detection(void);

uint32_t *Get_last_call_back_time(void);

const Motor_2006_3508_6020_t *Get_Measure_Address(uint32_t i);
Supercapacitor_receive_t *get_supercap_control_point(void);

void CAN1_Send_2006_3508(int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor);
void CAN1_Send_6020(int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor);

#endif 

