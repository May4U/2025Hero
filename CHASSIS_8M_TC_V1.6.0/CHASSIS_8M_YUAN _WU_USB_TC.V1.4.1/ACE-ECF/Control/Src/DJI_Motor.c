/************************** Dongguan-University of Technology -ACE**************************
* @file DJI_Motor.c
* @brief  大疆电机集成驱动库
* @author zhengNannnn
* @version 1.0
* @date 2023-11-29
* @note 依赖前置bsp_Motor_Encoder.c且有修改
*
* @history
* Date       Version Author Description
* 2023-11-29   1.0   郑楠   完成最基本的大疆电机驱动集成
* 2023-12-04   1.1   郑楠   添加can接收中断调用的回调函数，可用于Safe任务喂狗
* 2023-12-28   1.2   郑楠   适配最新的bsp_can文件，使用其提供的ECF_CAN_Rx_Callback_Register()接口
* @verbatim
* ==============================================================================
*  目标值采用真实物理量
*  简化原先的大疆电机使用方式
*  所有大疆电机的消息接收集成在一个函数里面，与原来每使用一个电机都需要在CAN中断里面加一个处理相比方便了不少
*  优化了原来需要自己手动将多个电机打包发送的情况，现在只需一个电机便会发送它所处的那一组四个电机，当然也可以指定一个组，也可以单独只发一个
*  保留了原来PID库的使用方式，只需要调用提供的计算函数就可以完成PID计算
*  严谨了电调的输入量和电机回调的接收信息，GM6020与M2006，M3508的输入物理量区分开来，在接收中，区分了M2006接收为输出力矩
*
*  Demo:
*      
*      DJIMotor_object_t *DJI_Motor;
*      DJI_Motor = DJIMotor_Init(1,1,false,GM6020,168);//挂载can1，id为1，轮组半径168mm
*      DJI_Motor->Using_PID = Position_Speed_PID;//位置速度环
*      DJIMotor_Set_val(DJI_Motor,0);//锁0°
*      //PID初始化
*      PidInit(&DJI_Motor->Speed_PID,200,0,0,Output_Limit);//使用输出限幅
*      PidInitMode(&DJI_Motor->Speed_PID,Output_Limit,5000,0);//输出限幅模式设置
*      PidInit(&DJI_Motor->Position_PID,10,0,0,Output_Limit);//使用输出限幅
*      PidInitMode(&DJI_Motor->Position_PID,Output_Limit,1000,0);//输出限幅模式设置
*      
*      while (1)
*      {
*          DJIMotor_PID_Calc(DJI_Motor);
*          //三选一
*          DJIMotor_Send_Group(0);//查该文件内得到此电机位于Ground[0];
*          DJIMotor_Send(DJI_Motor);//发送此电机所处组别
*          DJMotor_Send_only_one(DJI_Motor);//只发送这一个电机
*      }
*   
*
*  注意事项：
*      
*      
*
* ==============================================================================
* @TODO 继续集成编码器清理，任务托管PID计算,发送,只需输入更新的目标值，添加LQR功能
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "DJI_Motor.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/*--------------------文件内私密变量-----------------------*/
static uint8_t  DJIMotor_CallBack_Flag = 0;
static uint8_t  DJI_Motor_Rx_Data[8];    							//接受数据的数组
static CAN_TxHeaderTypeDef CAN_TxHeader_DJI;
static uint8_t DJI_Motor_Tx_Data[8];
static uint32_t send_mail_box;

DJIMotor_object_t ALL_Motor[2][8];//[2]{0：can1；1：can2}  [8]{电机ID号}
// 6020的id 1-4和2006/3508的id 5-8会发生冲突
// 开一个数组特别标记 丑陋且有用
static Encoder_Type_e RX_ID_205_208_type[2][4] = {{Type_End,Type_End,Type_End,Type_End},
                                                  {Type_End,Type_End,Type_End,Type_End}};
/*
 * C610(M2006)/C620(M3508): 0x200,0x1FF;
 * GM6020:                  0x1FF,0x2FF
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
typedef struct{
    int16_t* Wait_Send[4];//指针数组 指向连续的四个id 1-4或5-8
}Ground_t;
static Ground_t Ground[6];
/*--------------------函数声明-----------------------*/
DJIMotor_object_t *DJIMotor_Init(uint8_t hcan, uint8_t CanId, bool reverse_flag, Encoder_Type_e  Encoder_type, uint16_t radius);
void DJIMotor_Call_Back(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data);
void DJIMotor_Set_val(DJIMotor_object_t *DJIMotor, fp32 val);
void DJIMotor_PID_Calc(DJIMotor_object_t *DJIMotor);
void DJIMotor_Send_Group(uint8_t Group_index);
void DJIMotor_Send(DJIMotor_object_t *DJIMotor);
void DJMotor_Send_only_one(DJIMotor_object_t *DJIMotor);
void DJIMotor_CAN_Recive_init(void);
/*----------------------------------------------------*/

/**
  *@brief DJIMotor_Init      大疆电机申请并初始化
  *@parm  hcan               挂载的can网    1、2
  *@parm  CanId              挂载的Can id号 1-8
  *@parm  reverse_flag       是否偏置安装
  *@parm  Encoder_Type_e     大疆电机型号
  *@reval DJIMotor_object_t  初始化后的大疆电机地址
  *@note  reverse_flag 偏置标志位的意思是 当取正电流时，电机转动方向与实际运动方向相反，通常左侧电机需要取反
  */
DJIMotor_object_t *DJIMotor_Init(uint8_t hcan, uint8_t CanId, bool reverse_flag, Encoder_Type_e  Encoder_type, uint16_t radius)
{
    if (DJIMotor_CallBack_Flag == 0)
    {
        DJIMotor_CAN_Recive_init();
        DJIMotor_CallBack_Flag++;
    }
    if (hcan != 1 && hcan!=2)   return NULL;
    if (CanId > 8 || CanId < 1) return NULL;
    //6020的id 1-4和2006/3508的id 5-8冲突区间 检查是否ID冲突
    if (Encoder_type == GM6020 && CanId <= 4)
    {
        if (RX_ID_205_208_type[hcan==1?0:1][CanId-1] != Type_End)   return NULL;
        else    RX_ID_205_208_type[hcan==1?0:1][CanId-1] = Encoder_type;
    }
    if (Encoder_type != GM6020 && CanId >= 5)
    {
        if (RX_ID_205_208_type[hcan==1?0:1][CanId-5] != Type_End)   return NULL;
        else    RX_ID_205_208_type[hcan==1?0:1][CanId-5] = Encoder_type;
    }
    
    DJIMotor_object_t *DJI_Motor = &(ALL_Motor[hcan==1?0:1][CanId-1]);//指向电机池对应的电机
    memset(DJI_Motor, 0, sizeof(DJIMotor_object_t));
   /*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
    */
    switch (Encoder_type)
    {
        case GM6020:{//6020发送 电调输出电压
            if (CanId <= 4) Ground[hcan==1?0:3].Wait_Send[CanId-1] = &DJI_Motor->voltage_input;
            else            Ground[hcan==1?2:5].Wait_Send[CanId-5] = &DJI_Motor->voltage_input;
        }break;
        
        case M2006:
        case M3508:{//这两发送 力矩输出电流
            if (CanId <= 4) Ground[hcan==1?1:4].Wait_Send[CanId-1] = &DJI_Motor->current_input;
            else            Ground[hcan==1?0:3].Wait_Send[CanId-5] = &DJI_Motor->current_input;
        }break;
        
        case Type_End://不为什么 看warn不顺眼
        default:{
        }break;
    }
    //初始化参数
    DJI_Motor->hcan = (hcan==1? &hcan1:&hcan2);
    DJI_Motor->CanId = CanId;
    DJI_Motor->reverse_flag = reverse_flag;
    DJI_Motor->Encoder_type = Encoder_type;
    Encoder_Init(&DJI_Motor->Motor_encoder, DJI_Motor->Encoder_type, radius);

    return DJI_Motor;
}

/**
  *@brief DJIMotor_CanRx_Callback   设定接收中断中调用的函数
  *@parm  Callback                  void返回值void参数的函数指针
  */
void DJIMotor_CanRx_Callback(DJIMotor_object_t *DJIMotor, Callback Callback_function)
{
    DJIMotor->CanRxCallBack = Callback_function;
}

/**
  *@brief DJIMotor_Set_val   电机设定目标值
  *@parm  val                设定的目标值cm/s或°
  */
void DJIMotor_Set_val(DJIMotor_object_t *DJIMotor, fp32 val)
{
    if (DJIMotor->reverse_flag == true)  val = -val;
    if (DJIMotor->Using_PID == Speed_PID)   DJIMotor->set_speed = val;
    else                                    DJIMotor->set_position = val;
}

void DJIMotor_Set_speed_val(DJIMotor_object_t *DJIMotor, int16_t val)
{
    DJIMotor->set_speed = val;
}

void DJIMotor_PID_Calc(DJIMotor_object_t *DJIMotor)
{
    switch (DJIMotor->Using_PID) {
        
        case Position_Speed_PID: {
            int16_t temp_PID_ret = motor_position_speed_control(&DJIMotor->Speed_PID,
                                                                &DJIMotor->Position_PID,
                                                                DJIMotor->set_angel,
                                                                DJIMotor->Motor_encoder.Record_Angle,
                                                                DJIMotor->Motor_encoder.linear_speed[1]);
            
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = temp_PID_ret;
            else                                    DJIMotor->current_input = temp_PID_ret;
        }break;
        
        case Speed_PID:{
            int16_t temp_PID_ret = PidCalculate(&DJIMotor->Speed_PID,
                                                DJIMotor->set_speed, 
                                                DJIMotor->Motor_Information.speed);
            
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = temp_PID_ret;
            else                                    DJIMotor->current_input = temp_PID_ret;
        }break;
        
        case No_Current:{
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = 0;
            else                                    DJIMotor->current_input = 0;
        }break;
        
        default :{
        }break;
    }
}

/**
  *@brif 大疆电机四个一起打包发送
  *@parm Group_index 发送的Can集合
  *@note can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
  *@note can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
  */
void DJIMotor_Send_Group(uint8_t Group_index)
{
    if (Group_index > 5)    return;
                
    Ground_t * Send_Group;
    CAN_HandleTypeDef *Send_phcan;
    
    switch (Group_index)
    {
        case 0:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[0];
            CAN_TxHeader_DJI.StdId 	= 0x1FF;
            break;
        }
        case 3:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[3];
            CAN_TxHeader_DJI.StdId 	= 0x1FF;
            break;
        }
        case 1:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[1];
            CAN_TxHeader_DJI.StdId 	= 0x200;
            break;
        }
        case 4:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[4];
            CAN_TxHeader_DJI.StdId 	= 0x200;
            break;
        }
        case 2:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[2];
            CAN_TxHeader_DJI.StdId 	= 0x2FF;
            break;
        }
        case 5:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[5];
            CAN_TxHeader_DJI.StdId 	= 0x2FF;
            break;
        }
        
        default:
            Send_Group = NULL;
            break;
    }
	CAN_TxHeader_DJI.IDE = CAN_ID_STD;
	CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
	CAN_TxHeader_DJI.DLC = 0x08;
    
    if (Send_Group->Wait_Send[0] != NULL){//防止空指针
            DJI_Motor_Tx_Data[0] = (*Send_Group->Wait_Send[0]) >> 8;
            DJI_Motor_Tx_Data[1] = (*Send_Group->Wait_Send[0]);
    }else   DJI_Motor_Tx_Data[0] = DJI_Motor_Tx_Data[1] = 0;
    
    if (Send_Group->Wait_Send[1] != NULL){
            DJI_Motor_Tx_Data[2] = (*Send_Group->Wait_Send[1]) >> 8;
            DJI_Motor_Tx_Data[3] = (*Send_Group->Wait_Send[1]);
    }else   DJI_Motor_Tx_Data[2] = DJI_Motor_Tx_Data[3] = 0;    

    if (Send_Group->Wait_Send[2] != NULL){
            DJI_Motor_Tx_Data[4] = (*Send_Group->Wait_Send[2]) >> 8;
            DJI_Motor_Tx_Data[5] = (*Send_Group->Wait_Send[2]);
    }else   DJI_Motor_Tx_Data[4] = DJI_Motor_Tx_Data[5] = 0;

    if (Send_Group->Wait_Send[3] != NULL){//防止空指针
            DJI_Motor_Tx_Data[6] = (*Send_Group->Wait_Send[3]) >> 8;
            DJI_Motor_Tx_Data[7] = (*Send_Group->Wait_Send[3]);
    }else   DJI_Motor_Tx_Data[6] = DJI_Motor_Tx_Data[7] = 0;

	HAL_CAN_AddTxMessage(Send_phcan, &CAN_TxHeader_DJI, DJI_Motor_Tx_Data, &send_mail_box);			
}
/*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
*/
void DJIMotor_Send(DJIMotor_object_t *DJIMotor)
{
    uint8_t Motor_Gounp_index = 1;
    if (DJIMotor->hcan == &hcan2)    Motor_Gounp_index+=3;
    if (DJIMotor->CanId > 4)
    {
        if (DJIMotor->Encoder_type == GM6020)   Motor_Gounp_index+=2;
        else                                    Motor_Gounp_index-=1;
    }
    else
    {
        if (DJIMotor->Encoder_type == GM6020)   Motor_Gounp_index-=2;
        else                                    Motor_Gounp_index+=0;
    }
    DJIMotor_Send_Group(Motor_Gounp_index);
}
/*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
*/
void DJMotor_Send_only_one(DJIMotor_object_t *DJIMotor)
{
    DJI_Motor_Tx_Data[0] = 0;
    DJI_Motor_Tx_Data[1] = 0;
    DJI_Motor_Tx_Data[2] = 0;
    DJI_Motor_Tx_Data[3] = 0;
    DJI_Motor_Tx_Data[4] = 0;
    DJI_Motor_Tx_Data[5] = 0;
    DJI_Motor_Tx_Data[6] = 0;
    DJI_Motor_Tx_Data[7] = 0;
    int16_t Send_Message;
    if (DJIMotor->Encoder_type == GM6020)   Send_Message = DJIMotor->voltage_input;
    else                                    Send_Message = DJIMotor->current_input;
    
    if (DJIMotor->CanId <= 4)
    {
        if (DJIMotor->Encoder_type == GM6020)   CAN_TxHeader_DJI.StdId = 0x1FF;
        else                                    CAN_TxHeader_DJI.StdId = 0x200;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-1)*2] = Send_Message >> 8;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-1)*2+1] = Send_Message >> 8;
    }
    else
    {
        if (DJIMotor->Encoder_type == GM6020)   CAN_TxHeader_DJI.StdId = 0x2FF;
        else                                    CAN_TxHeader_DJI.StdId = 0x1FF;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-5)*2] = Send_Message >> 8;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-5)*2+1] = Send_Message >> 8;
    }
	CAN_TxHeader_DJI.IDE = CAN_ID_STD;
	CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
	CAN_TxHeader_DJI.DLC = 0x08;
    
	HAL_CAN_AddTxMessage(DJIMotor->hcan, &CAN_TxHeader_DJI, DJI_Motor_Tx_Data, &send_mail_box);	
}


/**
  *@brief DJIMotor_Data_Deal 电机数据处理
  *@parm  Motor              待处理的电机指针
  *@parm  Rx_Data            CAN接收的8个U8数据
  */
void DJIMotor_Data_Deal(DJIMotor_object_t *Motor,uint8_t DJI_Motor_Rx_Data[8])
{
    //电机基本数据处理
    Motor->Motor_Information.position	    = (int16_t)(DJI_Motor_Rx_Data[0] << 8 | DJI_Motor_Rx_Data[1]);
    Motor->Motor_Information.speed			= (int16_t)(DJI_Motor_Rx_Data[2] << 8 | DJI_Motor_Rx_Data[3]);
    //不同电机接收的信息不一样
    if(Motor->Encoder_type == M2006)Motor->Motor_Information.Output_torque	= (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
    else                            Motor->Motor_Information.Torque_current = (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
	Motor->Motor_Information.temperature	=  DJI_Motor_Rx_Data[6]; 
    if (Motor->CanRxCallBack != NULL)   Motor->CanRxCallBack();//调用接收回调函数
    //编码盘处理
    CAN_DATA_Encoder_Deal(Motor->Motor_Information.position, Motor->Motor_Information.speed, &Motor->Motor_encoder);
}

/**
  *@brief DJIMotor_Call_Back 大疆电机中断回调函数
  *@parm  hcan               接收到消息的can邮箱
  *@note  需要将该函数丢到CAN的接收中断函数里面去,目前该函数写死了接收消息为FIFO0队列,若有配置有改变,需要修改
  */
uint16_t can_std = 0;
uint8_t a = 0;
void chang (void)
{
    a = 0;
}
void DJIMotor_Call_Back(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    switch(CAN_Rxmessage->StdId)//用switch方便后期特殊处理？例如对应的塞进去一个safe任务
    {
        can_std = CAN_Rxmessage->StdId;
        if (can_std == 0x205)
        {
            chang();
            a = 1;
        }
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
            DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN_Rxmessage->StdId - 0x201], DJI_Motor_Rx_Data);
            break;
        case 0x205://6020的id 1-4和2006/3508的id 5-8冲突区间
        case 0x206:
        case 0x207:
        case 0x208:
        {
            //检查注册该ID的电机型号
            if (RX_ID_205_208_type[hcan==&hcan1?0:1][CAN_Rxmessage->StdId - 0x205] == GM6020)
                //6020该区间id对应为1-4，二维下标为 StdId-0x205
                DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN_Rxmessage->StdId - 0x205], DJI_Motor_Rx_Data);
            else 
                //3508 2006 该区间id对应为5-8，二维下标为 StdId-0x201
                DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN_Rxmessage->StdId - 0x201], DJI_Motor_Rx_Data);
            break;
        }
        case 0x209://6020 id 5-7区间
        case 0x20A:
        case 0x20B:
            DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN_Rxmessage->StdId - 0x205], DJI_Motor_Rx_Data);
            break;
    }
}

/*内部函数，不对外开放*/
void DJIMotor_Can1_Call_Back(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    DJIMotor_Call_Back(&hcan1, CAN_Rxmessage, DJI_Motor_Rx_Data);
}
void DJIMotor_Can2_Call_Back(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    DJIMotor_Call_Back(&hcan2, CAN_Rxmessage, DJI_Motor_Rx_Data);
}
void DJIMotor_CAN_Recive_init(void)
{
    ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Stdid, 0, DJIMotor_Can1_Call_Back);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0, DJIMotor_Can2_Call_Back);
}
//6020参数转换
float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	
	if (voltage > 25000)
    {
        voltage = 25000;
    }
    else if (voltage < -25000)
    {
        voltage = -25000;
    }
	return voltage;
		
}