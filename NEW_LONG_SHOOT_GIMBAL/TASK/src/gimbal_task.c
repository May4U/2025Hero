/*--------------------- FIRMWARE --------------------*/
#include "string.h"
#include "usbd_cdc_if.h"
/*--------------------- TASK --------------------*/
#include "imu_task.h"
#include "gimbal_task.h"
#include "fire_Task.h"
#include "virtual_task.h"
#include "safe_task.h"
#include "music_task.h"
/*--------------------- COMMUINICATE --------------------*/
#include "bsp_can.h"
/*--------------------- CONTROL --------------------*/
#include "robot_cmd.h"
#include "DJI_Motor.h"
#include "Cybergear_Motor.h"
/*--------------------- ALGORITHM --------------------*/
#include "pid.h"
#include "maths.h"
#include "filter.h"
#include "lqr.h"
#include "rgb.h"

#define ABS(x) (((x)>0) ? (x) : (-x))
/*--------------------- BSP --------------------*/



static void Photo_Control(void);
static void RGB_LED(void);
static void IMU_PITCH_Angle_Set_Limit(float* ,float*);
static void Gimbal_Work();
static void Gimbal_Init();
static void Gimbal_Not_Work();



typedef struct{

    const Gimbal_CMD_t* Gimbal_CMD;
    const INS_t         *ins;
   
    DJIMotor_object_t   *YAW_Motor;
    DJIMotor_object_t   *Photo_Motor;
    MI_Motor_t   *PITCH_Motor;
    //陀螺仪LQR
    LQR_t lqr_pitch;
    LQR_t lqr_pitch_litter;
    LQR_t lqr_yaw;
    
    LQR_t lqr_encoder_pitch;
    LQR_t lqr_encoder_yaw;
    pid_parameter_t     yaw_only_i_pid;
    //编码值PID
    pid_parameter_t      PITCH_Motor_Speed_PID;
    pid_parameter_t      PITCH_Motor_Position_PID;   
    pid_parameter_t      YAW_Motor_SPEED_PID;
    pid_parameter_t      YAW_Motor_Position_PID;
    pid_parameter_t      Photo_Motor_SPEED_PID;
    pid_parameter_t      Photo_Motor_Position_PID;
    
    fp32 Phtot_Angle;
}Gimbal_t;
Gimbal_t Gimbal;



/*电机在线状态*/
uint16_t pitch_motor_init_flag = 0;
safe_task_t *Pitch_Motor_Safe;

uint8_t pitch_motor_is_online = 0;
void pitch_motor_online(void){}
void pitch_motor_disconnect(void)
{
    pitch_motor_init_flag = 0;
    Gimbal.PITCH_Motor->have_pos_flag = 0;
}
void pitch_can_rx_CallBack(void)
{
    Safe_Task_online_ptr_(Pitch_Motor_Safe);
    pitch_motor_is_online = 1;
}

uint8_t yaw_motor_is_online = 0;
void yaw_motor_online(void){yaw_motor_is_online = 1;}
void yaw_motor_disconnect(void){yaw_motor_is_online = 0;}
void yaw_can_rx_CallBack(void){//Safe_Task_online_name_("Yaw_Motor_Safe");
    }

uint8_t Photo_motor_online = 0;
void Photo_motor_CanRx_callback(void){Photo_motor_online = 1;}

void Send_Photo_Current()
{
    static CAN_TxHeaderTypeDef CAN_TxHeader;
    static uint8_t Can_Tx_Data[8];
    Can_Tx_Data[0] = 0;
    Can_Tx_Data[1] = 0;
    Can_Tx_Data[2] = 0;
    Can_Tx_Data[3] = 0;
    Can_Tx_Data[4] = 0;
    Can_Tx_Data[5] = 0;
    Can_Tx_Data[6] = Gimbal.Photo_Motor->current_input >> 8;
    Can_Tx_Data[7] = Gimbal.Photo_Motor->current_input ;

    static uint32_t send_mail_box;
    CAN_TxHeader.StdId = 0x1FF;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = 0x08;
    HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeader, Can_Tx_Data, &send_mail_box);
}

/*LQR参数列表*/
//手瞄lqr参数
float k_pitch_lqr[2] = {1.5f, 2.0f};

float k_pitch_lqr_down[2] = {0.8f, 2.0f};
float k_yaw_lqr[2] = {-80.0f, -1.0f};
//自瞄lqr参数
float k_virtual_pitch_lqr[2] = {60.0f, 1.2f};
float k_virtual_yaw_lqr[2] = {-40.0f, -1.0f};
//狙击lqr参数
float k_encoder_pitch_lqr[2] = {6,1};
float k_encoder_yaw_lqr[2] = {500,100};
/**
 * @brief  云台主任务
 * @param
 * @note 6020电机反馈信息波特率 135.25bit*1000hz = 135,250 bps
         6020电机控制信息波特率 135.25bit*1000hz = 135,250 bps
         转发图传控制信息波特率 85.25 bit*  36hz  = 3069   bps
         转发视觉开火信息波特率 65.25 bit* 100hz  = 6925   bps    
         转发四摩擦轮转速波特率 135.25bit*  10hz  = 1352.5 bps   
 * @retval void
 */
fp32 UP_ANGLE = 0;
fp32 DOWN_ANGLE = 0;
fp32 Pitch_output_torque;
fp32 Pitch_gravity_feedforward_torque = 1.0f;//P轴重力前馈补偿
uint8_t  Enable_pitch_motor = 100;//连续发送100次使能后就不发送了
void GIMBAL_TASK(void const * argument)
{    
    static uint16_t time = 0;

    while(Get_Virtual_task_init_falg() == 0)    vTaskDelay(1);//等待视觉任务初始化
    while(Gimbal.ins->imu_ok_falg == 0)         vTaskDelay(1);//等待IMU任务运行

    Gimbal_Init();
    
    while (1)
    {

    time ++;
    if (time > 1000)    time = 0;
        
    #ifdef Using_MI_Motor_Init
        
        if (Gimbal.PITCH_Motor->have_pos_flag < 100)//收集1k次位置信息
        {
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,1000000);
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
            
            MI_motor_enable(Gimbal.PITCH_Motor);
            vTaskDelay(1);
            continue;
        }
        
        if (pitch_motor_init_flag < 500)//堵转1000次任务周期作为初始化
        {
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,1000000);
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
            
            MI_motor_controlmode(Gimbal.PITCH_Motor, Pitch_gravity_feedforward_torque, 0, 0, 0, 0);
            if (abs(Gimbal.PITCH_Motor->speed) < 0.03f) pitch_motor_init_flag++;
            vTaskDelay(1);
            continue;
        }
        
        if (pitch_motor_init_flag ==  500)
        {
            pitch_motor_init_flag ++;
                        Set_Pitch_Motor_down_encoder(Gimbal.PITCH_Motor->postion);//初始化校验完成，设定最低限位

        }

    #endif
    
        
        taskENTER_CRITICAL();         // 进入临界区
        
        Gimbal_Deal_TLcontrol();//图传信息处理及转发
        
        IMU_PITCH_Angle_Set_Limit(&UP_ANGLE,&DOWN_ANGLE);//俯仰角限制
        Gimbal_CMD_Set(UP_ANGLE,DOWN_ANGLE, Gimbal.ins->Pitch, Gimbal.Gimbal_CMD->Pitch_Motor_down_encoder - Gimbal.PITCH_Motor->postion);//陀螺仪上下极限
        
        
        if (Gimbal.Gimbal_CMD->Fire_Ready == 0)//Gimbal.Gimbal_CMD->rc_ctl->rc.s2 == RC_SW_DOWN)
        {
            Enable_pitch_motor = 100;
            Gimbal_Not_Work();
            Gimbal_Pitch_Set_Encoder(Gimbal.PITCH_Motor->postion);
            Gimbal_Yaw_Set_Encoder(Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val);
            Gimbal_Pitch_Set_IMU(Gimbal.ins->Pitch);
            Gimbal_Yaw_Set_IMU(Gimbal.ins->Yaw);
        }
        else
        {
            Gimbal_Work();      //云台正常工作
        }
        
        RGB_LED();//炫彩rgb
        Photo_Control();//开镜时保持图传平行
        
        
        if (time % 2 == 0)    
            Send_Photo_Current();
        
        DJMotor_Send_only_one(Gimbal.YAW_Motor);
        MI_motor_controlmode(Gimbal.PITCH_Motor, Pitch_output_torque, 0, 0, 0, 0);

		taskEXIT_CRITICAL();              // 退出临界区
        vTaskDelay(1); // 绝对延时//vTaskDelay(2);
        
    }
}


void Gimbal_Init()
{
    Gimbal.Phtot_Angle = 85.0f;
    Gimbal.Gimbal_CMD = Get_Gimbal_CMD_point();
    Gimbal.ins = get_imu_control_point();
    Gimbal.Photo_Motor = DJIMotor_Init(1,8,false,M2006,3600/2/PI);
    Gimbal.YAW_Motor = DJIMotor_Init(2,1,false,GM6020,3600/2/PI);
    Gimbal.PITCH_Motor = MI_motor_init(&hcan1);
    
    PidInit(&Gimbal.PITCH_Motor_Speed_PID,0.f,0.f,0.0f,Integral_Limit | Output_Limit);
    PidInitMode(&Gimbal.PITCH_Motor_Speed_PID,Integral_Limit,0,0);//积分限幅
    PidInitMode(&Gimbal.PITCH_Motor_Speed_PID,Output_Limit,4,0);//输出限幅
    
    PidInit(&Gimbal.PITCH_Motor_Position_PID,0.0f,0,0.0f,Integral_Limit | Output_Limit | StepIn);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, Integral_Limit, 0, 0);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, Output_Limit, 0, 0);
    PidInitMode(&Gimbal.PITCH_Motor_Position_PID, StepIn, 30, 0);
    
    PidInit(&Gimbal.YAW_Motor_SPEED_PID,200.0f,0.0f,15.0f,Output_Limit | Integral_Limit );//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.YAW_Motor_SPEED_PID,Integral_Limit,1000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.YAW_Motor_SPEED_PID,Output_Limit,25000,0);//输出限幅模式设置
    
    PidInit(&Gimbal.YAW_Motor_Position_PID,2.0f,0,0.0f,Output_Limit | Integral_Limit);//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.YAW_Motor_Position_PID,Output_Limit,200,0);//输出限幅模式设置
    PidInitMode(&Gimbal.YAW_Motor_Position_PID,Integral_Limit,200,0);//输出限幅模式设置

    PidInit(&Gimbal.yaw_only_i_pid,0.0f,1.5f,0.0f,Output_Limit | Integral_Limit | Separated_Integral);//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.yaw_only_i_pid,Integral_Limit,10000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.yaw_only_i_pid,Output_Limit,10000,0);//输出限幅模式设置
    PidInitMode(&Gimbal.yaw_only_i_pid,Separated_Integral,20,-20);
   
    PidInit(&Gimbal.Photo_Motor_SPEED_PID,10.0f,0.0f,0.0f,Output_Limit | Integral_Limit );//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.Photo_Motor_SPEED_PID,Integral_Limit,1000,0);//积分限幅模式设置
    PidInitMode(&Gimbal.Photo_Motor_SPEED_PID,Output_Limit,5000,0);//输出限幅模式设置
    
    PidInit(&Gimbal.Photo_Motor_Position_PID,0.1f,0,0.0f,Output_Limit | Integral_Limit);//使用输出限幅和积分限幅模式
    PidInitMode(&Gimbal.Photo_Motor_Position_PID,Output_Limit,1000,0);//输出限幅模式设置
    PidInitMode(&Gimbal.Photo_Motor_Position_PID,Integral_Limit,200,0);//输出限幅模式设置
    
    //LQR初始化
    LQR_Init(&Gimbal.lqr_pitch, 2, 1, k_pitch_lqr);
    LQR_Init(&Gimbal.lqr_pitch_litter, 2, 1, k_pitch_lqr_down);
    LQR_Init(&Gimbal.lqr_yaw, 2, 1, k_yaw_lqr);
    LQR_Init(&Gimbal.lqr_encoder_pitch, 2, 1, k_encoder_pitch_lqr);
    LQR_Init(&Gimbal.lqr_encoder_yaw, 2, 1, k_encoder_yaw_lqr);
    
    
    //创建安全任务
    Pitch_Motor_Safe = Safe_task_add("Pitch_Motor_Safe", 30, pitch_motor_disconnect, pitch_motor_online);
    //电机回调函数设置
    MI_Motor_CanRx_Callback(Gimbal.PITCH_Motor, pitch_can_rx_CallBack);
    DJIMotor_CanRx_Callback(Gimbal.YAW_Motor, yaw_can_rx_CallBack);
    DJIMotor_CanRx_Callback(Gimbal.Photo_Motor, Photo_motor_CanRx_callback);

    MI_motor_enable(Gimbal.PITCH_Motor);
}

/**
  *@brief 根据电机绝对编码值判断IMU的Pitch目标角度可以增加减少多少
  */
void IMU_PITCH_Angle_Set_Limit(fp32 *IMU_Pitch_Up_Limit, fp32 *IMU_Pitch_Down_Limit)
{
    #ifdef Using_MI_Motor_Init
    //计算电机绝对角度可以还能增加减少多少


     fp32 Motor_Can_reduced_Pitch_Angle   = Gimbal.Gimbal_CMD->Pitch_Motor_down_encoder - Gimbal.PITCH_Motor->postion ;//根据上电时记录的低头最大值推算
     fp32 Motor_Can_increased_Pitch_Angle = Gimbal.PITCH_Motor->postion - (Gimbal.Gimbal_CMD->Pitch_Motor_down_encoder - PITCH_ANGLE_RANGE);
    #else
     fp32 Motor_Can_increased_Pitch_Angle = Gimbal.PITCH_Motor->postion - PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE ;
     fp32 Motor_Can_reduced_Pitch_Angle   = PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE - Gimbal.PITCH_Motor->postion ;
    #endif
    *IMU_Pitch_Up_Limit   = Gimbal.ins->Pitch + Motor_Can_increased_Pitch_Angle;
    *IMU_Pitch_Down_Limit = Gimbal.ins->Pitch - Motor_Can_reduced_Pitch_Angle;

    if (pitch_motor_is_online == 0)
    {
        *IMU_Pitch_Up_Limit = 0;
        *IMU_Pitch_Down_Limit = 0;
    }   
    static Recursive_ave_filter_type_t filter_IMU_Pitch_Up_Limit;
    static Recursive_ave_filter_type_t filter_IMU_Pitch_Down_Limit;

    static uint8_t filter_init_flag = 1;
    if (filter_init_flag)
    {
        Recursive_ave_filter_init(&filter_IMU_Pitch_Up_Limit);
        Recursive_ave_filter_init(&filter_IMU_Pitch_Down_Limit);

    }
    //平滑imu限幅 防止震荡发散
    *IMU_Pitch_Up_Limit =  Recursive_ave_filter(&filter_IMU_Pitch_Up_Limit,*IMU_Pitch_Up_Limit,30);
    *IMU_Pitch_Down_Limit =  Recursive_ave_filter(&filter_IMU_Pitch_Down_Limit,*IMU_Pitch_Down_Limit,30);
}


void Motor_Choice_Lock_Type(void)
{
    if (Gimbal.Gimbal_CMD->Gimbal_Work_State == LONG_SHOOT)//前哨站模式，改为锁编码值
    {
        if (ABS(Gimbal.ins->Pitch - Gimbal.Gimbal_CMD->Pitch_Set_IMU) < 8.0f && Gimbal.Gimbal_CMD->Pitch_Lock_type == IMU)
        {//达到目标IMU PITCH
            Gimbal_Pitch_Lock(ENCODER);
            Gimbal_Pitch_Set_Encoder(Gimbal.PITCH_Motor->postion);
        }
        if (ABS(Gimbal.ins->Yaw - Gimbal.Gimbal_CMD->Yaw_Set_IMU) < 8.0f && Gimbal.Gimbal_CMD->Yaw_Lock_type == IMU)
        {//达到目标IMU PITCH
            Gimbal_Yaw_Lock(ENCODER);
            Gimbal_Yaw_Set_Encoder(Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val);
        }
    }
    else//改为锁IMU
    {
        if (Gimbal.Gimbal_CMD->Pitch_Lock_type == ENCODER)
        {
            Gimbal_Pitch_Lock(IMU);
            Gimbal_Pitch_Set_IMU(Gimbal.ins->Pitch);
        }
        if (Gimbal.Gimbal_CMD->Yaw_Lock_type == ENCODER)
        {
            Gimbal_Yaw_Lock(IMU);
            Gimbal_Yaw_Set_IMU(Gimbal.ins->Yaw);
        }    
    }
}

#define PITCH_TORQUE_LIMIT 8
void Pitch_Calc(void)
{
    if (Gimbal.Gimbal_CMD->Pitch_Lock_type == ENCODER)//锁电机
    {
        Gimbal.lqr_encoder_pitch.k[0] = k_encoder_pitch_lqr[0];
        Gimbal.lqr_encoder_pitch.k[1] = k_encoder_pitch_lqr[1];

        float Pitch_motortarget = Gimbal.Gimbal_CMD->Pitch_Set_Encoder - Gimbal.PITCH_Motor->postion;
        float Pitch_system_state[2] = {Pitch_motortarget, -Gimbal.PITCH_Motor->speed};
        

        LQR_Data_Update(&Gimbal.lqr_encoder_pitch, Pitch_system_state);
        LQR_Calculate(&Gimbal.lqr_encoder_pitch);
        
        Pitch_output_torque = Gimbal.lqr_encoder_pitch.Output[0];
        value_limit(Pitch_output_torque,-PITCH_TORQUE_LIMIT,PITCH_TORQUE_LIMIT);
        
        //俯角限幅
        if((Gimbal.Gimbal_CMD->Pitch_Set_Encoder > Gimbal.PITCH_Motor->postion) && (Gimbal.PITCH_Motor->postion > PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE+2))
        {
            //MI_motor_controlmode(Gimbal.PITCH_Motor, 0, 0, 0, 0, 0);
        }
        //仰角限幅
        else if( (Gimbal.Gimbal_CMD->Pitch_Set_Encoder < Gimbal.PITCH_Motor->postion) && (Gimbal.PITCH_Motor->postion < PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE - 2 ))
        {
            //MI_motor_controlmode(Gimbal.PITCH_Motor, 0, 0, 0, 0, 0);
        }
        else
        {
            //Pitch_output_torque = 0;
            //MI_motor_controlmode(Gimbal.PITCH_Motor, Pitch_output_torque, 0, 0, 0, 0);
        }
    }

    else//锁imu
    {
        Gimbal.lqr_pitch.k[0] = k_pitch_lqr[0];
        Gimbal.lqr_pitch.k[1] = k_pitch_lqr[1];

        float Pitch_motortarget = Gimbal.Gimbal_CMD->Pitch_Set_IMU - Gimbal.ins->Pitch;
        float Pitch_system_state[2] = {-Pitch_motortarget, -Gimbal.ins->Gyro[0]};
        
        static fp32 lastPitch_Set_IMU = 0;
        static uint8_t litter_lqr_time = 0;
        if (Gimbal.Gimbal_CMD->Pitch_Set_IMU < lastPitch_Set_IMU)
        {
            litter_lqr_time = 32;//低速32次
        }
        if ( litter_lqr_time > 0)
        {
            litter_lqr_time -- ;
            Gimbal.lqr_pitch_litter.k[0] = k_pitch_lqr_down[0];
            Gimbal.lqr_pitch_litter.k[1] = k_pitch_lqr_down[1];
            
            LQR_Data_Update(&Gimbal.lqr_pitch_litter, Pitch_system_state);
            LQR_Calculate(&Gimbal.lqr_pitch_litter);
            
            Pitch_output_torque = Gimbal.lqr_pitch_litter.Output[0];
                
        }
        else
        {
            LQR_Data_Update(&Gimbal.lqr_pitch, Pitch_system_state);
            LQR_Calculate(&Gimbal.lqr_pitch);
            
            Pitch_output_torque = Gimbal.lqr_pitch.Output[0];
        }
        lastPitch_Set_IMU = Gimbal.Gimbal_CMD->Pitch_Set_IMU;
        value_limit(Pitch_output_torque,-PITCH_TORQUE_LIMIT,PITCH_TORQUE_LIMIT);
        
        //仰角限幅
        if((Gimbal.Gimbal_CMD->Pitch_Set_IMU > Gimbal.ins->Pitch) && (Gimbal.PITCH_Motor->postion < PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE - 2 ))
        {
            //MI_motor_controlmode(Gimbal.PITCH_Motor, 0, 0, 0, 0, 0);
        }
        //俯角限幅
        else if( (Gimbal.Gimbal_CMD->Pitch_Set_IMU < Gimbal.ins->Pitch) && (Gimbal.PITCH_Motor->postion > PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE + 2 ))
        {
            //MI_motor_controlmode(Gimbal.PITCH_Motor, 0, 0, 0, 0, 0);
        }
        else
        {
            //Pitch_output_torque = 0;
            //MI_motor_controlmode(Gimbal.PITCH_Motor, Pitch_output_torque, 0, 0, 0, 0);
        }
    }
}

void Yaw_Calc(void)
{
    if (Gimbal.Gimbal_CMD->Yaw_Lock_type == ENCODER)//锁电机
    {   
        Gimbal.lqr_encoder_yaw.k[0] = k_encoder_yaw_lqr[0];
        Gimbal.lqr_encoder_yaw.k[1] = k_encoder_yaw_lqr[1];

        float Yaw_motortarget = Gimbal.Gimbal_CMD->Yaw_Set_Encoder - Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val;
        float Yaw_system_state[2] = {Yaw_motortarget, Gimbal.YAW_Motor->Motor_Information.speed};

        LQR_Data_Update(&Gimbal.lqr_encoder_yaw, Yaw_system_state);
        LQR_Calculate(&Gimbal.lqr_encoder_yaw);
        
        int64_t Yaw_Motor_Output =  Gimbal.lqr_encoder_yaw.Output[0] + PidCalculate(&Gimbal.yaw_only_i_pid, Gimbal.Gimbal_CMD->Yaw_Set_Encoder, Gimbal.YAW_Motor->Motor_encoder.Encode_Record_Val);;
        value_limit(Yaw_Motor_Output,-25000,25000);//LQR运算有很大概率超int16
        Gimbal.YAW_Motor->voltage_input = Yaw_Motor_Output;
    }
    else//锁IMU
    {
        Gimbal.lqr_yaw.k[0] = k_yaw_lqr[0];
        Gimbal.lqr_yaw.k[1] = k_yaw_lqr[1];
        
        float Yaw_motortarget = float_min_distance(Gimbal.Gimbal_CMD->Yaw_Set_IMU,Gimbal.ins->Yaw, -180, 180);
        float Yaw_system_state[2] = {((-Yaw_motortarget) / 57.295779513f), Gimbal.ins->Gyro[2]};
        LQR_Data_Update(&Gimbal.lqr_yaw, Yaw_system_state);
        LQR_Calculate(&Gimbal.lqr_yaw);
    
        float Yaw_output_torque =  Gimbal.lqr_yaw.Output[0];
        Gimbal.YAW_Motor->voltage_input = torque_to_voltage_6020(Yaw_output_torque);
    }
}

/**
*@brief 图传小云台控制
*@param Shoot
*@note  向下角度为增加
*/
int64_t lock_position_encoder = 0;
int64_t move = 9500;
float spin = 0.6f;

/**
  *@brief 舵机角度转脉冲
  */
uint16_t GetCCR(float Inputangle)
{
	uint16_t RET = ((Inputangle/180 * 2000) + 500);
	return RET;
}

void Photo_Control(void)
{
        /*按键操控*/
    /*
    F 舵机↑
    V 舵机↓
    C 开镜     
    B 关镜
    */
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,GetCCR(Gimbal.Phtot_Angle));
    
    //动作组 
    //吊射 倍镜到位 舵机摇下
    //正常 舵机摇上 倍镜侧置
    static int16_t Set_Photo_Angle_delay = 300;//等待舵机向上到位后再把倍镜挪开的时间延迟
    static uint8_t Set_MG995_Angle_flag = 1;//仅开关镜时自动切换一次舵机角度
    
    static int64_t last_Photo_Motor_encoder = 0;
    static int16_t to_lock = 300;//校准限位堵转计时器
    static int16_t to_unlock = 300;//堵转后会有形变，此时设定为限位编码值不准
    static uint8_t Reset_flag = 1;//校准标志位
   
    static int64_t Set_encoder = 0;//锁的编码
    static int8_t  Status = 0;//开关镜情况
    static uint16_t On_set_time = 0;//长按开镜计时器
    static uint16_t Off_set_time = 0;//长按关镜计时器
    static uint8_t  Left_move_flag = 0;//向左微调标志
    static uint8_t  Right_move_flag = 0;//向右微调标志
    static uint8_t  FUCK = 1;

    if (Photo_motor_online == 0) return;//等待2006上电后再校准
    
    if (Reset_flag)
    {
        Set_MG995_Angle_flag = 1;
        Status = 0;
        Gimbal.Phtot_Angle = 92;
        
        Gimbal.Photo_Motor->current_input = to_lock > 0? (-2000):0 ;//转动到限位
        
        if (abs(Gimbal.Photo_Motor->Motor_Information.speed) < 10)//到位后堵转
        {
            if (to_lock > 0)    to_lock--;//晚上好夜之城 
        }
        if (to_lock == 0)//堵转计时到
        {
            to_unlock--;//电流设定为0，释放堵转产生的弹性形变
            if (to_unlock == 0)
            {
                Reset_flag = 0;//已完成校准
                lock_position_encoder = Gimbal.Photo_Motor->Motor_encoder.Encode_Record_Val;//记录堵转编码值
                Set_encoder = lock_position_encoder + move;//堵转位和开镜位间有差异
            }
            
        }
    }
    else 
    {
    /*短按微调*/
    /*长按开镜*/
        
        
    uint8_t unlook_C_flag = 1;
    static uint8_t B_set = 1;
    if (Gimbal.Gimbal_CMD->rc_ctl->kb.bit.B == 1)
    {
        unlook_C_flag = 0;
        Off_set_time++;//长按计时器
        if (B_set)//按下即微调
        {
            B_set = 0;
            if (Status == -1) Right_move_flag = 1;//向右微调
        }
    }
    else
    {
        B_set = 1;
        if (Off_set_time > 0) Off_set_time = 0;//清零长按计时器
    }

    static uint8_t C_set = 1;
    if (Gimbal.Gimbal_CMD->rc_ctl->kb.bit.C == 1 && unlook_C_flag) 
    {
        On_set_time++;//长按计时器
        if (C_set)//按下即微调
        {
            C_set = 0;
            if (Status == -1)   Left_move_flag = 1;//开倍镜时才允许向左微调
        }
    }
    else
    {
        C_set = 1;//按键松开检测
        if (On_set_time > 0) On_set_time = 0;//清零长按计时器
    }
    
    if (On_set_time > 500)//长按一秒开镜
    {
        if (Status != -1)
        {
            FUCK = 1;
            to_lock = 300;
            to_unlock = 300;
            Set_Photo_Angle_delay = 200;
            Set_MG995_Angle_flag = 1;
            Status = -1;
        }
    }
    
    if (Off_set_time > 500)//长按一秒关镜
    {
        if (Status != 1)
        {
            Left_move_flag = 1;//补偿长按关镜被误识别的短按
            Set_Photo_Angle_delay = 200;
            Set_MG995_Angle_flag = 1;
            Status = 1;
        }

    }

    if (Left_move_flag == 1)//向左微调
    {
        Left_move_flag = 0;
        move-=500;
    }
    if (Right_move_flag == 1)//向右微调
    {
        Right_move_flag = 0;
        move+=500;
    }
    
    
    
        /*F短按*/
        static uint8_t F_set = 1;
        if (Gimbal.Gimbal_CMD->rc_ctl->kb.bit.F == 1)
        {
            if (F_set)
            {
                F_set = 0;
                Gimbal.Phtot_Angle--;
            }
        }
        else
            F_set = 1;
        
        /*V短按*/
        static uint8_t V_set = 1;
        if (Gimbal.Gimbal_CMD->rc_ctl->kb.bit.V == 1)
        {
            if (V_set)
            {
                V_set = 0;
                Gimbal.Phtot_Angle++;
            }
        }
        else
            V_set = 1;
        
        
        Gimbal.Photo_Motor->current_input = motor_position_speed_control(&Gimbal.Photo_Motor_SPEED_PID, &Gimbal.Photo_Motor_Position_PID, Set_encoder, Gimbal.Photo_Motor->Motor_encoder.Encode_Record_Val, Gimbal.Photo_Motor->Motor_Information.speed);
    }
        
    //执行动作
    if (Status == 1)//关镜
    {
        if (Set_MG995_Angle_flag)//首次执行动作，一键设定舵机角度
        {
            Set_MG995_Angle_flag = 0;
            Gimbal.Phtot_Angle = 85;
        }
        value_limit(Gimbal.Phtot_Angle,85,130);
        
        
        if (Set_Photo_Angle_delay > 0)   Set_Photo_Angle_delay--;//使用简单延时等待舵机到位后切镜，防止倍镜撞相机
        if (Set_Photo_Angle_delay == 0)
        {
            Set_encoder = lock_position_encoder + move + 90000;
        }
    }        
    
    if (Status == -1) //由于编码值会飘，所以每次开镜都轻撞限位重置
    {
        Set_encoder = lock_position_encoder + move;//开镜
        
        if (FUCK == 1)//操蛋开镜校准
        {
            Gimbal.Photo_Motor->current_input = to_lock > 0? (-2000):0 ;//转动到限位
            if (abs(Gimbal.Photo_Motor->Motor_Information.speed) < 10)//到位后堵转
            {
                if (to_lock > 0)    to_lock--;//晚上好夜之城 
            }
            if (to_lock == 0)//堵转计时到
            {
                if (to_unlock > 0) to_unlock--;
                if (to_unlock == 0)
                {
                    //已完成校准
                    lock_position_encoder = Gimbal.Photo_Motor->Motor_encoder.Encode_Record_Val;//记录堵转编码值
//                    Set_encoder = lock_position_encoder + move;//开镜
                    FUCK = 0;
                }
            }
            last_Photo_Motor_encoder = Gimbal.Photo_Motor->Motor_encoder.Encode_Record_Val;
        }
        else
        {
            if (Set_MG995_Angle_flag && (abs(Gimbal.Photo_Motor->Motor_encoder.Encode_Record_Val - Set_encoder) < 10000))//开镜到位后舵机自动到位
            {
                Set_MG995_Angle_flag = 0;
                Gimbal.Phtot_Angle = 85 + Gimbal.ins->Pitch * spin;
            }
            value_limit(Gimbal.Phtot_Angle,85,130);
        }
    }//放大舵机角度   
    
}
void RGB_LED(void)
{
    static uint16_t idx_1 = 0;
    static uint16_t idx_2 = 0;
    static uint16_t idx_3 = 0;

        __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,sin_value[idx_1]*1000);
        __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,sin_value[idx_2]*1000);
        __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,sin_value[idx_3]*1000);
    
    //三色灯以不同周期运行正弦波
   if (idx_1 < 990)  idx_1+=1;
    else    idx_1 = 0;
    
       if (idx_2 < 990)  idx_2+=2;
    else    idx_2 = 0;
    
       if (idx_3 < 990)  idx_3+=3;
    else    idx_3 = 0;
}

void Gimbal_Work()
{
    if (Enable_pitch_motor > 0)
    {
        MI_motor_enable(Gimbal.PITCH_Motor);
        Enable_pitch_motor--;
    }
    Motor_Choice_Lock_Type();//锁电机或陀螺仪
    Pitch_Calc();//P轴计算
    Yaw_Calc();//轴计算
}

void Gimbal_Not_Work()
{
    Gimbal.YAW_Motor->voltage_input = 0;
    Pitch_output_torque = 0;
    MI_motor_stop(Gimbal.PITCH_Motor);
}

