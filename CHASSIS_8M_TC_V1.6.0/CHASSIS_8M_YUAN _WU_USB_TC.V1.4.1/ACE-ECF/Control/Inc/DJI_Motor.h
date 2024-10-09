#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include <struct_typedef.h>
#include "pid.h"
#include "bsp_Motor_Encoder.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "bsp_can.h"
typedef enum{
    Position_Speed_PID,
    Speed_PID,
    No_Current
}Using_PID_e;

typedef struct{
    int16_t position;//转子位置
    int16_t speed;//rpm (转/分钟) 
    int16_t Torque_current;//仅C620电调M3508 M6020电机有
	int16_t Output_torque; //仅C610电调M2006电机有 
	uint8_t temperature;
}DJIMotor_feedback_data_t;

typedef struct{
    uint8_t CanId;//Can通信id
    CAN_HandleTypeDef *hcan;
    bool  reverse_flag;//偏置反转
    Encoder_Type_e  Encoder_type; //电机种类
    
    void (*CanRxCallBack)(void);//中断调用函数
    DJIMotor_feedback_data_t    Motor_Information;//电机CAN回传信息
    Encoder_t                   Motor_encoder;//电机编码器信息
    
    pid_parameter_t     Speed_PID;    //速度环PID
	pid_parameter_t	    Position_PID; //位置环PID
    Using_PID_e         Using_PID;    //使用哪个环
    int64_t             set_position;
    fp32                set_angel;       //设定角度
    fp32                set_speed;       //设定线速度
    int16_t             current_input;//电调输出力矩电流，仅2006和3508有,-10000到100000，对应-10A到10A
    int16_t             voltage_input;//驱动器的电压输出，仅6020有  -30000到30000 对应-24V到24V
}DJIMotor_object_t;

typedef void (*Callback)(void);

DJIMotor_object_t *DJIMotor_Init(uint8_t hcan, uint8_t CanId, bool reverse_flag, Encoder_Type_e  Encoder_type, uint16_t radius);
void DJIMotor_Call_Back(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data);
void DJIMotor_Set_val(DJIMotor_object_t *DJIMotor, fp32 val);
void DJIMotor_PID_Calc(DJIMotor_object_t *DJIMotor);
void DJIMotor_Send_Group(uint8_t Group_index);
void DJIMotor_Send(DJIMotor_object_t *DJIMotor);
void DJMotor_Send_only_one(DJIMotor_object_t *DJIMotor);
float torque_to_voltage_6020(float torque);
void DJIMotor_CanRx_Callback(DJIMotor_object_t *DJIMotor, Callback Callback_function);
#endif
