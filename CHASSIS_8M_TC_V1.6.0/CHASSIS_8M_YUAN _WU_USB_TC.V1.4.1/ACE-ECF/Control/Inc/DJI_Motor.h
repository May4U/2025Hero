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
    int16_t position;//ת��λ��
    int16_t speed;//rpm (ת/����) 
    int16_t Torque_current;//��C620���M3508 M6020�����
	int16_t Output_torque; //��C610���M2006����� 
	uint8_t temperature;
}DJIMotor_feedback_data_t;

typedef struct{
    uint8_t CanId;//Canͨ��id
    CAN_HandleTypeDef *hcan;
    bool  reverse_flag;//ƫ�÷�ת
    Encoder_Type_e  Encoder_type; //�������
    
    void (*CanRxCallBack)(void);//�жϵ��ú���
    DJIMotor_feedback_data_t    Motor_Information;//���CAN�ش���Ϣ
    Encoder_t                   Motor_encoder;//�����������Ϣ
    
    pid_parameter_t     Speed_PID;    //�ٶȻ�PID
	pid_parameter_t	    Position_PID; //λ�û�PID
    Using_PID_e         Using_PID;    //ʹ���ĸ���
    int64_t             set_position;
    fp32                set_angel;       //�趨�Ƕ�
    fp32                set_speed;       //�趨���ٶ�
    int16_t             current_input;//���������ص�������2006��3508��,-10000��100000����Ӧ-10A��10A
    int16_t             voltage_input;//�������ĵ�ѹ�������6020��  -30000��30000 ��Ӧ-24V��24V
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
