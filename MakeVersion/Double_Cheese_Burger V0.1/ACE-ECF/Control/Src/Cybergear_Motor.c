
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    cybergear.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/10/1
 * @brief
 ******************************************************************************
 * @verbatim
 *  米狗电机驱动库,支持自定义选择can邮箱,支持多电机同时使用
 *  使用方法：
 *      先创建一个米狗电机结构体
 *      初始化：
 *            使用MI_motor_init申请数据结构体，注意这里设置后面使用的can网络
 *            使用MI_motor_get_ID(MI_Motor);//获取电机id值
 *            使用changeID(MI_Motor,num);//更改ID
 *      使用：多种运行模式,具体流程参考用户手册
 *  demo：
 *       extern MI_Motor_t MI_Motor;//引用一个米狗电机结构体,实际上在can接收相关.c中声明定义
 *       MI_Motor = MI_motor_init(hcan1);//挂载在can1
 *       MI_motor_get_ID(MI_Motor);//获取电机id值,储存到结构体
 *       MI_motor_changeID(MI_Motor,127);//更改ID为127
 *       MI_motor_enable(MI_Motor)；//电机使能
 *       MI_motor_controlmode(MI_Motor, torque, MechPosition , speed , kp , kd);//运控模式
 *       void MI_motor_stop(MI_Motor_t* hmotor);//电机失能
 * @attention
 *      请确保CAN已经start
 *      请先判断基于大端通信还是小段通信,在MI_motor_recive_callback函数内修改宏定义
 *      每创建一个米狗电机结构体,就丢一个处理函数到对应的can中断里面,例如
 *      MI_Motor_t YAW_Motor;
 *      MI_Motor_t PITCH_Motor;
 *      MI_motor_recive_callback(YAW_Motor, Rxmessage.ExtId, Rx_Data)丢到中断处理函数
 *      MI_motor_recive_callback(PITCH_Motor, Rxmessage.ExtId, Rx_Data)丢到中断处理函数
 * @version
 * v1.0   基础版本
 * v1.1   添加接收处理函数
 * v1.2   集成速度位置环PID
 ************************** Dongguan-University of Technology -ACE***************************/
#include "Cybergear_Motor.h"
#include "bsp_can.h"
#include <string.h>
#include <stdlib.h>

//共用体float转型uint8_t
union FloatAndBytes_t {
    float floatValue;
    uint8_t byteValue[4];
}FloatAndBytes;

bool get_can_id = false;
uint8_t MI_id = 0;
/**
  * @brief         浮点数转4字节
  * @param         浮点数
  * @return        4字节数组
  * @description   IEEE 754 协议
  */
uint8_t byte[4];
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}
/**
  * @brief         4字节转浮点数
  * @param         4字节数组
  * @return        浮点数
  * @description   IEEE 754 协议
  */
float Byte_to_Float(uint8_t* byte)
{
    unsigned long longdata = 0;

    // 将字节按位组装成一个无符号长整数
    longdata |= (unsigned long)byte[0] << 24;
    longdata |= (unsigned long)byte[1] << 16;
    longdata |= (unsigned long)byte[2] << 8;
    longdata |= (unsigned long)byte[3];

    // 将长整数重新解释为浮点数
    float result = *(float*)&longdata;

    return result;
}

/**
  * @brief  float转int，数据打包用
  * @param  x float数值
  * @param  x_min float数值的最小值
  * @param  x_max float数值的最大值
  * @param  bits  int的数据位数
  * @retval null
  */
int float_to_uint(float x, float x_min, float x_max, int bits) 
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief  小米电机回文16位数据转浮点
  * @param  16位回文 
  * @param  对应参数下限 
  * @param  对应参数上限 
  * @param  参数位数
  * @return 参数对应浮点数
  */
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief  小米电机CAN通信发送
  * @param  hmotor 电机结构体
  * @retval null
  */
CAN_TxHeaderTypeDef CAN_TxHeader_MI;
void MI_Motor_CanTx(MI_Motor_t* MI_motor) 
{
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(MI_motor->txMsg.ExtId));
	/*CAN_TxHeader_MI.ExtId = hmotor->EXT_ID.motor_id<<24 | hmotor->EXT_ID.data << 8 | hmotor->EXT_ID.mode << 5;*/
    uint32_t mailbox;
    
    uint32_t ret = HAL_CAN_AddTxMessage(MI_motor->phcan, &CAN_TxHeader_MI, MI_motor->txMsg.Data, &mailbox);
//    if (ret != HAL_OK) {
//        /* Transmission request Error */
//        while(1);
//    }
    while (ret != HAL_OK) {
        /* Transmission request Error */
        ret = HAL_CAN_AddTxMessage(MI_motor->phcan, &CAN_TxHeader_MI, MI_motor->txMsg.Data, &mailbox);
    }
}

/**
  * @brief  申请小米电机结构体并初始化
  * @param  phcan can总线句柄
  * @retval 申请的小米电机结构体指针
  */
MI_Motor_t* Now_Malloc_Can1_MI_Motor_Groud[10] = {NULL};
MI_Motor_t* Now_Malloc_Can2_MI_Motor_Groud[10] = {NULL};
void MI_Motor_CAN_Recive_init(void);

MI_Motor_t* MI_motor_init(CAN_HandleTypeDef *phcan)
{
    MI_Motor_t *MI_motor = (MI_Motor_t *)malloc(sizeof(MI_Motor_t));
    memset(MI_motor, 0, sizeof(MI_Motor_t));
    
    MI_motor->have_pos_flag = 0;
    MI_motor->master_id = MASTER_ID;
    MI_motor->motor_id = MOTOR_ID;
    MI_motor->phcan = phcan;
    MI_motor->CanRxCallBack = NULL;
    
    if (phcan == &hcan1)
    {
        for (int i = 0; i < 10; i++)
        {
            if (Now_Malloc_Can1_MI_Motor_Groud[i] == NULL)
            {
                Now_Malloc_Can1_MI_Motor_Groud[i] = MI_motor;
                break;
            }                
        }
    }else if (phcan == &hcan2)
    {
        for (int i = 0; i < 10; i++)
        {
            if (Now_Malloc_Can2_MI_Motor_Groud[i] == NULL)   
            {
                Now_Malloc_Can2_MI_Motor_Groud[i] = MI_motor;
                break;
            }
        }
    }
    
    static uint8_t MI_Motor_CallBack_Init_Flag = 0;
    if (MI_Motor_CallBack_Init_Flag == 0)
    {
        MI_Motor_CallBack_Init_Flag++;
        MI_Motor_CAN_Recive_init();
    }
    return MI_motor;
}

/*内部函数，不对外开放*/
void MI_Motor_Can1_Call_Back(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *MI_Motor_Rx_Data)
{
    for (int i = 0; i < 10; i++)
    {
        if (Now_Malloc_Can1_MI_Motor_Groud[i] == NULL)   break;
        MI_motor_recive_callback(Now_Malloc_Can1_MI_Motor_Groud[i], CAN_Rxmessage->ExtId, MI_Motor_Rx_Data);
        if (Now_Malloc_Can1_MI_Motor_Groud[i]->CanRxCallBack != NULL)
            Now_Malloc_Can1_MI_Motor_Groud[i]->CanRxCallBack();
    }
}
void MI_Motor_Can2_Call_Back(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *MI_Motor_Rx_Data)
{
    for (int i = 0; i < 10; i++)
    {
        if (Now_Malloc_Can2_MI_Motor_Groud[i] == NULL)   break;
        MI_motor_recive_callback(Now_Malloc_Can2_MI_Motor_Groud[i], CAN_Rxmessage->ExtId, MI_Motor_Rx_Data);
        if (Now_Malloc_Can2_MI_Motor_Groud[i]->CanRxCallBack != NULL)
            Now_Malloc_Can2_MI_Motor_Groud[i]->CanRxCallBack();
    }
}
void MI_Motor_CAN_Recive_init(void)
{
    ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Extid, 0, MI_Motor_Can1_Call_Back);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Extid, 0, MI_Motor_Can2_Call_Back);
}

/**
  *@brief MI_Motor_CanRx_Callback   设定接收中断中调用的函数
  *@parm  Callback                  void返回值void参数的函数指针
  */
void MI_Motor_CanRx_Callback(MI_Motor_t *MI_motor, Callback Callback_function)
{
    MI_motor->CanRxCallBack = Callback_function;
}

/**
  * @brief  小米电机使能
  * @param  hmotor 电机结构体
  * @param  id 电机id
  * @retval null
  */
void MI_motor_enable(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 3;
    MI_motor->txMsg.ExtId.motor_id = MI_motor->motor_id;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  获取设备ID （通信类型0），需在电机使能前使用
  * @param  hmotor 电机结构体
  * @retval null 
  */
void MI_motor_get_ID(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 0;
    MI_motor->txMsg.ExtId.data = 0;
    MI_motor->txMsg.ExtId.motor_id = 0;
    MI_motor->txMsg.ExtId.res = 0;
    
    get_can_id = true;
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
    while(get_can_id);//等待can中断接收到电机目前id
    MI_motor->motor_id = MI_id;
    
}

/**
  * @brief  运控模式电机控制指令（通信类型1）
  * @param  hmotor 电机结构体
  * @param  torque 力矩
  * @param  MechPosition 目标位置
  * @param  speed 转速
  * @param  kp 
  * @param  kd 
  * @retval null
  */
void MI_motor_controlmode(MI_Motor_t* MI_motor, float torque, float MechPosition , float speed , float kp , float kd)
{
    MI_motor->txMsg.ExtId.mode = 1;
    MI_motor->txMsg.ExtId.data = float_to_uint(torque,T_MIN,T_MAX,16);
    MI_motor->txMsg.ExtId.res = 0;
 
    MI_motor->txMsg.Data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    MI_motor->txMsg.Data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    MI_motor->txMsg.Data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    MI_motor->txMsg.Data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    MI_motor->txMsg.Data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    MI_motor->txMsg.Data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    MI_motor->txMsg.Data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    MI_motor->txMsg.Data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  电机停止运行帧（通信类型4）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_stop(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 4;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_setMechPosition2Zero(MI_Motor_t* MI_motor)
{
    MI_motor->txMsg.ExtId.mode = 6;
    MI_motor->txMsg.ExtId.data = MASTER_ID;
    MI_motor->txMsg.ExtId.res = 0;
    MI_motor->txMsg.Data[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param  MI_motor 电机结构体
  * @param  Target_ID 想要改成的电机ID
  * @retval null
  */
void MI_motor_changeID(MI_Motor_t* MI_motor,uint8_t Target_ID)
{
    MI_motor->txMsg.ExtId.mode = 7;	
    MI_motor->txMsg.ExtId.motor_id = MI_motor->motor_id;
    MI_motor->txMsg.ExtId.data = Target_ID << 8 | MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    
    MI_motor->motor_id = Target_ID;//更改本电机canID
 
    for(uint8_t i=0; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  单个参数读取（通信类型17）
  * @param  MI_motor 电机结构体指针
  * @param  parameter_index_e 读取的参数
  * @retval null
  * @note   目前反馈接收存在小bug
  */
void MI_motor_Read_One_Para(MI_Motor_t* MI_motor, parameter_index_e index)
{
    MI_motor->txMsg.ExtId.mode = 17;
    MI_motor->txMsg.ExtId.data = MASTER_ID;
    MI_motor->txMsg.ExtId.res = 0;
    MI_motor->txMsg.Data[0]=index;
    memcpy(&MI_motor->txMsg.Data[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        MI_motor->txMsg.Data[i]=0;
    }
    MI_Motor_CanTx(MI_motor);
}

/**
  * @brief  单个参数写入（通信类型18） （掉电丢失）
  * @param  电机结构体指针
  * @param  写入的参数
  * @param  写入的值
  * @retval null
  * @note   电流 位置 速度模式第一次写入无反应？
  */
void MI_motor_Write_One_Para(MI_Motor_t* MI_motor, parameter_index_e index , float Value)
{
    MI_motor->txMsg.ExtId.mode = 0x12;
    MI_motor->txMsg.ExtId.data = MI_motor->master_id;
    MI_motor->txMsg.ExtId.res = 0;
    for(uint8_t i=2; i<8; i++)
    {
        MI_motor->txMsg.Data[i] = 0;
    }

    memcpy(&MI_motor->txMsg.Data[0],&index,2);
//    memcpy(&MI_motor->txMsg.Data[4],data, 4);
    if (index == run_mode)
    {
        MI_motor->txMsg.Data[4]=(uint8_t)Value;
		MI_motor->txMsg.Data[5]=0x00;
		MI_motor->txMsg.Data[6]=0x00;
		MI_motor->txMsg.Data[7]=0x00;	
    }
    else
    {
		Float_to_Byte(Value);
		MI_motor->txMsg.Data[4]=byte[3];
		MI_motor->txMsg.Data[5]=byte[2];
		MI_motor->txMsg.Data[6]=byte[1];
		MI_motor->txMsg.Data[7]=byte[0];		
	}
    MI_Motor_CanTx(MI_motor);
}

/**
  * @function     : 设置电机控制模式
  * @param        : 电机控制模式
  * @return       : null
  */
void MI_motor_Set_Mode(MI_Motor_t *MI_motor,Mode_rum_mode_t Mode)
{
	MI_motor_Write_One_Para(MI_motor,run_mode,Mode);
}


/**
  * @brief  小米电机接收处理函数
  * @param  MI_motor 电机结构体指针
  * @param  receive_ExtId 接收的can拓展标识符
  * @param  Rx_Data[8] 接收的8字节数据
  * @retval null
  * @note   判断的数据结构体内本电机id需要自己手动设置
  */
void MI_motor_recive_callback(MI_Motor_t* MI_motor, uint32_t receive_ExtId, uint8_t Rx_Data[8])
{
//    uint16_t temp;
    MI_motor->rxMsg.ExtId = *((rxExtId_t*)&receive_ExtId);
    if (MI_motor->rxMsg.ExtId.motor_id != MI_motor->master_id)       return;//非本电机
    if (MI_motor->rxMsg.ExtId.motor_now_id != MI_motor->motor_id)    return;//非本主机
    //以上操作方便了can网络上搭载多个米狗电机的情况,每使用一个米狗电机就丢一个这个到can接收中断处理里面,接收到标识符本电机id值自动忽略
    
    switch (MI_motor->rxMsg.ExtId.mode)
    {
        case 0:{//获取设备的ID和64位MCU唯一标识符(通信类型0)
            if (MI_motor->rxMsg.ExtId.motor_id == 0xFE){
                get_can_id = true;//MI_motor_get_ID用得到
                MI_id = MI_motor->rxMsg.ExtId.motor_id;
            };     //后面的是64位MCU唯一标识符,暂不添加
            break;
        }
        case 2:{//电机反馈数据帧,用来向主机反馈电机运行状态(通信类型2)
            
            //拓展帧bit16~23信息
            MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t)MI_motor->rxMsg.ExtId.Mode_status;
            MI_motor->motor_error_list.Encoder_not_calibrated_fault = MI_motor->rxMsg.ExtId.Uncalibrated;
            MI_motor->motor_error_list.HALL_encoding_fault = MI_motor->rxMsg.ExtId.HALL_encoding_failt;
            MI_motor->motor_error_list.Magnetic_encoding_fault = MI_motor->rxMsg.ExtId.Magnetic_encoding_fault;
            MI_motor->motor_error_list.Over_temperature_fault = MI_motor->rxMsg.ExtId.Over_temperature;
            MI_motor->motor_error_list.Over_current_fault = MI_motor->rxMsg.ExtId.Over_current;
            MI_motor->motor_error_list.Under_voltage_fault = MI_motor->rxMsg.ExtId.Under_voltage_fault;

            MI_motor->postion=uint16_to_float(Rx_Data[0]<<8|Rx_Data[1],MIN_P,MAX_P,16);

            MI_motor->speed=uint16_to_float(Rx_Data[2]<<8|Rx_Data[3],MIN_S,MAX_S,16);			

            MI_motor->torque=uint16_to_float(Rx_Data[4]<<8|Rx_Data[5],MIN_T,MAX_T,16);				

            MI_motor->temperture=(Rx_Data[6]<<8|Rx_Data[7])*Temp_Gain;
            if (MI_motor->have_pos_flag < 1000) MI_motor->have_pos_flag++;
            break;
        }
        case 17:{//发送读取单个参数的应答帧,根据index参数下标判断回传参数类型(通信类型17)
            parameter_index_e index = (parameter_index_e)(Rx_Data[1]<<8| Rx_Data[0]);//

            //共用体4个u8转float
            FloatAndBytes.byteValue[0] = Rx_Data[4];
            FloatAndBytes.byteValue[1] = Rx_Data[5];
            FloatAndBytes.byteValue[2] = Rx_Data[6];
            FloatAndBytes.byteValue[3] = Rx_Data[7];

            switch (index){
                case run_mode:
                    MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t) Rx_Data[4];//
                    break;
                case iq_ref:
                    MI_motor->motor_parameter_list.iq_ref = FloatAndBytes.floatValue;
                    break;
                case spd_ref:
                    MI_motor->motor_parameter_list.spd_ref = FloatAndBytes.floatValue;
                    break;
                case imit_torque:
                    MI_motor->motor_parameter_list.imit_torque = FloatAndBytes.floatValue;
                    break;
                case cur_kp:
                    MI_motor->motor_parameter_list.cur_kp = FloatAndBytes.floatValue;
                    break;
                case cur_ki:
                    MI_motor->motor_parameter_list.cur_ki = FloatAndBytes.floatValue;
                    break;
                case cur_filt_gain:
                    MI_motor->motor_parameter_list.cur_filt_gain = FloatAndBytes.floatValue;
                    break;
                case loc_ref:
                    MI_motor->motor_parameter_list.loc_ref = FloatAndBytes.floatValue;
                    break;
                case limit_spd:
                    MI_motor->motor_parameter_list.limit_spd = FloatAndBytes.floatValue;
                    break;
                case limit_cur:
                    MI_motor->motor_parameter_list.limit_cur = FloatAndBytes.floatValue;
                    break;
                default :
                    break;
            }
        }
        case 21:{//故障反馈帧
            //Byte0
            MI_motor->motor_error_list.Over_temperature_fault = Rx_Data[0] & 0x01;//取bit0
            MI_motor->motor_error_list.Drive_core_fault = (Rx_Data[0]>>1) & 0x01;//取bit1
            MI_motor->motor_error_list.Under_voltage_fault = (Rx_Data[0]>>2) & 0x01;//取bit2
            MI_motor->motor_error_list.Over_voltage_fault =  (Rx_Data[0]>>3) & 0x01;//取bit3
            MI_motor->motor_error_list.B_phase_current_sampling_overcurrent = (Rx_Data[0]>>4) & 0x01;//取bit4
            MI_motor->motor_error_list.C_phase_current_sampling_overcurrent = (Rx_Data[0]>>5) & 0x01;//取bit5
            MI_motor->motor_error_list.Magnetic_encoding_fault = (Rx_Data[0]>>7) ;//取bit7
            
            uint16_t Byte1_2 = ((uint16_t)Rx_Data[1] << 8) | Rx_Data[2];//取bit8~bit15
            if (Byte1_2 != 0)   MI_motor->motor_error_list.Overload_fault = true;
            else                MI_motor->motor_error_list.Overload_fault = false;
            
            MI_motor->motor_error_list.A_phase_current_sampling_overcurrent = Rx_Data[3] & 0x01;//取bit16
        }
    }
}
