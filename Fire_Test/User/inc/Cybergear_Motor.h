#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include "stm32f4xx_hal.h"
#include <struct_typedef.h>
#include <stdbool.h>


/*---ID值宏定义---*/
#define MASTER_ID 0
#define MOTOR_ID 127 //pitch电机127号

/*---写入参数地址宏定义---*/
#define SET_MODE_INDEX 0x7005
#define POSTION_MODE_SET_POSTION_INDEX 0x7016
#define POSTION_MODE_SET_SPEED_INDEX 0x7017
#define SPEED_MODE_SET_CURRENT_INDEX 0x7018

/*---CAN拓展标识符宏操作---*/
#define txCanIdEx (((struct ExtId)&(txMsg.tx_extid)))
#define rxCanIdEx (((struct ExtId)&(rxMsg.rx_efid))) //将扩展帧id解析为定义数据结构
/*电机参数限制*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define Temp_Gain 0.1
#define MAX_P 720
#define MIN_P -720
#define MAX_S 30
#define MIN_S -30
#define MAX_T 12
#define MIN_T -12
    
typedef enum{
    RESET_MODE = 0,//Reset[模式]
    CALI_MODE  = 1,//Cali 模式[标定]
    RUN_MODE   = 2 //Motor模式[运行]
} motor_mode_e;//电机模式

typedef enum{
    control = 0,//运控模式
    postion = 1,//位置模式
    speed   = 2,//速度模式
    current = 3 //电流模式
}Mode_rum_mode_t;//模式状态

//电机参数列表
typedef struct{
    Mode_rum_mode_t Mode_rum_mode;
    float iq_ref;
    float spd_ref;
    float imit_torque;
    float cur_kp;
    float cur_ki;
    float cur_filt_gain;
    float loc_ref;
    float limit_spd;
    float limit_cur;
}motor_parameter_list_t;

//电机故障及警告列表
typedef struct{
    bool Over_temperature_warning;  //过温警告
    
    bool Over_temperature_fault;    //过温故障
    bool Over_voltage_fault;        //过压故障
    bool Over_current_fault;        //过流故障
    bool Overload_fault;            //过载故障
    bool Under_voltage_fault;       //欠压故障
    bool Encoder_not_calibrated_fault;//编码器未标定故障
    bool Magnetic_encoding_fault;   //磁编码故障
    bool HALL_encoding_fault;       //HALL编码故障
    bool Drive_core_fault;          //驱动芯片故障
    bool A_phase_current_sampling_overcurrent;//A相电流采用过流
    bool B_phase_current_sampling_overcurrent;//B相电流采用过流
    bool C_phase_current_sampling_overcurrent;//C相电流采用过流   
}motor_error_list_t;//电机故障及警告列表

typedef enum{
    run_mode        = 0x7005,
    iq_ref          = 0x7006,
    spd_ref         = 0x700A,
    imit_torque     = 0x700B,
    cur_kp          = 0x7010,
    cur_ki          = 0x7011,
    cur_filt_gain   = 0x7014,
    loc_ref         = 0x7016,
    limit_spd       = 0x7017,
    limit_cur       = 0x7018
}parameter_index_e;//电机参数下标

//CAN拓展标识符结构体  motor_id、data、mode共29位
typedef struct {
    uint32_t motor_id:8;    //bit7~0        电机canID
    uint32_t data:16;       //bit15~23       信息位
    uint32_t mode:5;        //bit28~24      通信类型
    uint32_t res:3;         //占空位，无信息
}txExtId_t;
 
typedef struct {
    uint32_t motor_id:8;                //bit7~0    电机canID
    uint32_t motor_now_id:8;            //bit15~8   当前电机canID
    uint32_t Under_voltage_fault:1;     //bit16     欠压故障
    uint32_t Over_current:1;            //bit17     过流
    uint32_t Over_temperature:1;        //bit18     过流
    uint32_t Magnetic_encoding_fault:1; //bit19     磁编码故障
    uint32_t HALL_encoding_failt:1;     //bit20     HALL编码故障
    uint32_t Uncalibrated:1;            //bit21     未标定
    uint32_t Mode_status:2;             //bit22~23  模式状态
    uint32_t mode:5;                    //bit28~24  通信类型
    uint32_t res:3;                     //占空位，无信息
}rxExtId_t;

//CAN通信结构体 主控板->电机
typedef struct {
    txExtId_t     ExtId;//CAN 29位拓展标识符
    uint8_t       Data[8];//发送数据
}txMsg_t;

//CAN通信结构体 电机->主控板
typedef struct {
    rxExtId_t     ExtId;//CAN 29位拓展标识符
    uint8_t       Data[8];//接收数据
}rxMsg_t;

typedef struct{
    uint8_t master_id;
    uint8_t motor_id;
    uint64_t MCU;
    fp32 postion;
    fp32 speed;     
    fp32 torque;    //力矩
    fp32 temperture;//温度
    bool    error_falg;     //故障标志位
    
    motor_parameter_list_t motor_parameter_list;//参数列表
    motor_error_list_t    motor_error_list;     //状态列表
    
    CAN_HandleTypeDef *phcan;//电机注册使用的CAN邮箱
    txMsg_t            txMsg;//发送
    rxMsg_t            rxMsg;//接收
    
}MI_Motor_t;

void MI_motor_get_ID(MI_Motor_t* hmotor);
MI_Motor_t* MI_motor_init(CAN_HandleTypeDef *phcan);
void MI_motor_enable(MI_Motor_t *hmotor);
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd);
void MI_motor_stop(MI_Motor_t *hmotor);
void MI_motor_setMechPosition2Zero(MI_Motor_t *hmotor);
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Target_ID);
void MI_motor_recive_callback(MI_Motor_t* MI_motor, uint32_t receive_ExtId, uint8_t Rx_Data[8]);
void MI_motor_Write_One_Para(MI_Motor_t* MI_motor, parameter_index_e index , float Value);
void MI_motor_Read_One_Para(MI_Motor_t* MI_motor,parameter_index_e index);
void MI_motor_Set_Mode(MI_Motor_t *MI_motor,Mode_rum_mode_t Mode);
#endif
