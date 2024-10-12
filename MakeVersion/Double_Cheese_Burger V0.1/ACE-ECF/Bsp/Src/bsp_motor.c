/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_motor.c
 * @author  cayon
 * @version V0.1.0
 * @date    2023/9/23
 * @brief
 ******************************************************************************
 * @verbatim
 *  电机数据接收与处理
 *  使用方法：
 *  创建好电机结构体
 *  初始化电机结构体，设置好电机种类，offset
 *  将接收处理函数扔can接收中断中
 *      
 *  demo：
 *      Encoder_t encoder;
 *      Motor_Init(&encoder,M3508,0);
 *      在can接收中，加入Motor_InfoProc
 *       Motor_InfoProc(&encoder,rx_buf);
 *       
 * @attention
 *      数据会自动接收，需要时调用即可
 *      
 * @version
 * v0.1.0 完成基本接收功能，所有功能成功验证
 ************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_motor.h"
#include <string.h>
#include "maths.h"
#include "user_lib.h"
/****************************************内部函数声明**********************************************/
static void motor_rawproc(Encoder_t *encoder);
static int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);
static int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);

/**
  * @brief  电机初始化
  * @param  encoder结构体，电机种类，偏移角度，若无则写0即可
  * 
  * @retval None
  * @note 该句放在can初始化之前
  */
void Motor_Init(Encoder_t *encoder,uint8_t encoder_type,int16_t offset)
{
    memset((void *)encoder, 0x0, sizeof(Encoder_t));
    
    switch(encoder_type)
    {
        case M3508:
            encoder->motor_type=M3508;
            encoder->ratio=M3508_RATIO;
        break;
        
        case GM6020:
            encoder->motor_type=GM6020;
            encoder->ratio=GM6020_RATIO;
        break;
        
        case M2006:
            encoder->motor_type=M2006;
            encoder->ratio=M2006_RATIO;
        break;
        
        default:
            encoder->ratio=0;
        break;
    }
    
    encoder->motor_raw.relative_total_encoder=0;
    encoder->motor_raw.total_encoder=0;
    encoder->motor_raw.erro=0.0f;
    encoder->zero_offset=offset;
    encoder->actual_angle=0.0f;
    encoder->total_angle=0.0f;
    encoder->motor_raw.total_encoder-=((encoder->zero_offset/360.0f)*8192);
}

/**
  * @brief  电机接收数据处理
  * @param  encoder结构体
  *         data为can总线上收到的数据，配合can接收回调函数使用风味更佳
  * @retval None
  */
void Motor_InfoProc(Encoder_t *encoder,uint8_t* data)
{
    encoder->motor_raw.rotor_last_mechanical=encoder->motor_raw.rotor_mechanical;
    encoder->motor_raw.rotor_mechanical = ((data[0] << 8) | data[1]);
    encoder->motor_raw.rotor_speed      = ((data[2] << 8) | data[3]);
    encoder->motor_raw.torque_current   = ((data[4] << 8) | data[5]);  
    encoder->motor_raw.motor_temp =   data[6];
    motor_rawproc(encoder);//原生数据处理
}

/**
  * @brief  电机原始数据处理，获得角度与速度
  * @param  encoder结构体
  * @retval None
  */
void motor_rawproc(Encoder_t *encoder)
{
    static uint8_t isFirst=0;//TODO:待优化QAQ
    
    encoder->speed=encoder->motor_raw.rotor_speed;//转子速度赋值给编码器
    encoder->motor_raw.erro=//获取上次与本次的偏差
    angle_limiting_int16(((encoder->motor_raw.rotor_mechanical)-(encoder->motor_raw.rotor_last_mechanical)),LAP_ENCODER);
    encoder->motor_raw.total_encoder+=encoder->motor_raw.erro;//从电机反馈值记录
    
    if(isFirst<10)//若为第一次进入,相对编码值先不记录
    {
        encoder->motor_raw.relative_total_encoder=0;
        isFirst++;//待稳定//TODO:待优化QAQ
    }
    else//此后进入，相对编码值开始累加
    {
        isFirst=11;
        encoder->motor_raw.relative_total_encoder+=encoder->motor_raw.erro;
    }
    
    //获取绝对角度,限幅到0~360
    encoder->actual_angle=(encoder->motor_raw.total_encoder/(LAP_ENCODER*encoder->ratio*1.0f))*360.0f;
    encoder->actual_angle=loop_fp32_constrain(encoder->actual_angle,-180.0f,180.0f);
    
    //获取相对角度
    encoder->total_angle=(encoder->motor_raw.relative_total_encoder/(LAP_ENCODER*encoder->ratio*1.0f))*360.0f;
}

void Motor_ZeroTotal(Encoder_t *encoder)
{
    encoder->motor_raw.relative_total_encoder=0;
}

/**
  * @brief  临角处理16位
  * @param  传入当前值与上一次值，编码器转一圈圈数
  * @retval 实际增加/减少的编码值
  */
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder) {
   //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
   if (Angl_Err < -(lap_encoder / 2))
   {
       Angl_Err += (lap_encoder - 1);
   }
   if (Angl_Err > (lap_encoder / 2)) {
       Angl_Err -= (lap_encoder - 1);
   }
   return Angl_Err;
}

//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder) {
   if (value > (gear_Ratio * lap_encoder) / 2) {
       value = value - (gear_Ratio * lap_encoder);
   }
   if (value < (-(gear_Ratio * lap_encoder) / 2)) {
       value = (gear_Ratio * lap_encoder) - value;
   }
   return value;
}

//6020参数转换
float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = abs_limit(voltage,25000);
	
	return voltage;
		
}