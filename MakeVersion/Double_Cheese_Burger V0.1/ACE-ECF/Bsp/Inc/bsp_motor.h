#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "main.h"

#define M3508_RATIO 19
#define GM6020_RATIO 1
#define M2006_RATIO 36

#define LAP_ENCODER 8192

#define M3508_MAX 16000
#define GM6020_MAX 30000
#define M3508_SPEED_MAX 9000
#define GM6020_SPEED_MAX 320

typedef enum {
    M3508,
    GM6020,
    M2006,
}Encoder_Type_e;

typedef struct{
    int16_t erro;//偏差值记录
    int32_t total_encoder;//记录总共转动的编码值
    int32_t relative_total_encoder;//记录总共转动的编码值（从0记）
    uint16_t rotor_mechanical;
    uint16_t rotor_last_mechanical;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  motor_temp;
}Encoder_Raw_t;

typedef struct{
    Encoder_Type_e motor_type;
    Encoder_Raw_t motor_raw;
    float actual_angle;//真实角度(-180°~180°)//真实编码值，会减去OFFSET
    float total_angle;//相对角度,上电时为0
    float speed;//真实速度(rpm)
    int16_t zero_offset;//供绝对编码使用
    uint8_t ratio;//电机减速比
}Encoder_t;

typedef struct{
    float pos_set;
    float spd_set;
    float pos_real;
    float spd_real;
    float output;
}motor_t;

void Motor_Init(Encoder_t *encoder,uint8_t encoder_type,int16_t offset);
void Motor_InfoProc(Encoder_t *encoder,uint8_t* data);
void Motor_ZeroTotal(Encoder_t *encoder);
float torque_to_voltage_6020(float torque);

#endif
