#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "struct_typedef.h"

/*
     编码器序号
     
     3---0  车头
       |
       4    拨弹
       |
     2---1  车尾
*/
#define CHASSIS_MOTOR_RF_ENCODER   0
#define CHASSIS_MOTOR_RB_ENCODER   1
#define CHASSIS_MOTOR_LB_ENCODER   2
#define CHASSIS_MOTOR_LF_ENCODER   3
#define CHASSIS_MOTOR_FIRE_ENCODER 4


#define PI 3.14159f

/*数据状态枚举*/
typedef enum {
    NORM,  //正常
    BLOCK, //堵转
    WRONG  //异常
} STATE_e;

typedef enum {
    M3508,//如果有改减速比的，在初始化之后自己硬改
    GM6020,
    M2006,
    Type_End
} Encoder_Type_e;


typedef enum {
    Read_Value = 0x01,
    Set_Encoder_ID = 0x02,
    Set_Baud_Rate = 0x03,
    Set_Encoder_Mode = 0x04,
    Set_Auto_Return_Time = 0x05,
    Set_ZeroPoint = 0x06,
    Set_Direction = 0x07,
    Set_MidPoint = 0x0C,
    Set_Current_Value = 0x0D,
    Set_Five_Loops = 0x0F,
} Briter_Encoder_Code_e;

/*CAN数据处理-码盘处理*/
typedef __packed struct {
    int32_t Encode_Record_Val;//累积码盘值(从0开始记)
    int32_t Encode_Actual_Val;//真实码盘值(从当前电机编码器值开始记）
    
    float radius;       //轮胎半径 mm
    float linear_speed[2]; //线速度 0为旧速度，1为新速度
    float Acc_linear_speed;//线加速度
    float Radian;       //真实弧度
    float Actual_Angle;        //圆心角
    float Record_Angle;

    uint8_t Init_Flag;
    int16_t Speed[2];    //rpm (转/分钟) Can回传速度 0为旧速度，1为新速度
    int16_t AccSpeed;    //加速度
    int16_t position;    //未处理的Can原始码盘
    int16_t last_position;    //未处理的上次的Can原始码盘
    int16_t lap_encoder;      //编码器单圈码盘值（8192=12bit）
    Encoder_Type_e Encoder_Type;//编码器种类
    bool_t Block_Detect_Enable; //堵转检测开启否
    STATE_e State;              //电机状态
    int16_t gear_Ratio;         //减速比
    int16_t Max_Block_Angle_Num;//堵转最大角度容忍值，可以调整这个来控制堵转灵敏度
    int16_t Max_Block_Num;      //堵转计数器最大值
    int32_t Block_Count;
    void (*User_Fun)(void);     //用户自定义函数
} Encoder_t;



Encoder_t * Get_Chassis_motor_encoder_point (uint16_t CHASSIS_MOTOR_ENCODER, Encoder_Type_e Encoder_Type, uint16_t radius);

extern STATE_e Block_Detect(int16_t error, Encoder_t *Encoder );

void Encoder_Init(Encoder_t * Temp_Encoder, Encoder_Type_e Encoder_Type, uint16_t radius);

/*CAN返回码盘值处理*/
void CAN_DATA_Encoder_Deal(int16_t position, int16_t speed, Encoder_t * Encoder);

/*码盘值数值清零处理*/
extern void EncoderValZero(Encoder_t *Encoder);

extern void Briter_Encoder_Code_Set(uint8_t CAN_STD_ID, Briter_Encoder_Code_e code, uint32_t data);

#endif
