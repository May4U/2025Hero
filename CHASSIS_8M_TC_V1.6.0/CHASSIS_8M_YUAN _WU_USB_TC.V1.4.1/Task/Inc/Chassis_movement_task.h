#ifndef CHASSIS_MOVEMENT_H
#define CHASSIS_MOVEMENT_H
/*********FreeRTOSͷ�ļ�����*********/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include <struct_typedef.h>
#include "filter.h"
#include "CAN1.h"
#include "CAN2.h"
#include "pid.h"
#include "bsp_Motor_Encoder.h"
#include "PID_Data.h"
#include "robot_cmd.h"



typedef struct{
    uint8_t CanId;//Canͨ��id
    int8_t  reverse_flag;//��ת������
    Encoder_Type_e  Encoder_type; //�������
    const Motor_2006_3508_6020_t    *Motor_Information;//���CAN�ش���Ϣ
    Encoder_t           *Motor_encoder;//�����������Ϣ
    pid_parameter_t     Speed_PID;//�ٶȻ�PID
	pid_parameter_t	    Position_PID;//λ�û�PID
    int16_t             current_input;//��������������2006��3508��,-10000��100000����Ӧ-10A��10A
    int16_t             voltage_input;//�������ĵ�ѹ�������6020��
}Motor_information_t;

typedef struct{
    const Robot_cmd_t*  Robot_cmd;
    Motor_information_t RF_MOTOR;
    Motor_information_t RB_MOTOR;
    Motor_information_t LB_MOTOR;
    Motor_information_t LF_MOTOR;
}Chassis_t;
#endif