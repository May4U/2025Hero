#include "Chassis_movement_task.h"
#include "imu_task.h"
/*该文件内私密变量*/
static const INS_t  *ins;
Chassis_t Chassis = {
        .RF_MOTOR = {
            .CanId = 1,
            .reverse_flag = 1,
            .Encoder_type = M3508,
            .current_input = 0
        },
        .RB_MOTOR = {
            .CanId = 2,
            .reverse_flag = 1,
            .Encoder_type = M3508,
            .current_input = 0
        },
        .LB_MOTOR = {
            .CanId = 3,
            .reverse_flag = -1,
            .Encoder_type = M3508,
            .current_input = 0
        },
        .LF_MOTOR = {
            .CanId = 4,
            .reverse_flag = -1,
            .Encoder_type = M3508,
            .current_input = 0
        }
    };
void Chassis_Init(void)
{
    Chassis.Robot_cmd = get_Robot_cmd_point();
    
    Chassis.RF_MOTOR.Motor_Information = Get_Measure_Address(CHASSIS_MOTOR_RF_ID);
    Chassis.RB_MOTOR.Motor_Information = Get_Measure_Address(CHASSIS_MOTOR_RB_ID);
    Chassis.LB_MOTOR.Motor_Information = Get_Measure_Address(CHASSIS_MOTOR_LB_ID);
    Chassis.LF_MOTOR.Motor_Information = Get_Measure_Address(CHASSIS_MOTOR_LF_ID);
    
    Chassis.RF_MOTOR.Motor_encoder   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RF_ENCODER,Chassis.RF_MOTOR.Encoder_type, 0);
    Chassis.RB_MOTOR.Motor_encoder   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RB_ENCODER,Chassis.RB_MOTOR.Encoder_type, 0);
    Chassis.LB_MOTOR.Motor_encoder   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LB_ENCODER,Chassis.LB_MOTOR.Encoder_type, 0);
    Chassis.LF_MOTOR.Motor_encoder   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LF_ENCODER,Chassis.LF_MOTOR.Encoder_type, 0);
    
    PidInit(&Chassis.LF_MOTOR.Speed_PID, LF_Speed_PID_P,	LF_Speed_PID_I,	LF_Speed_PID_D,	Output_Limit | StepIn);
	PidInit(&Chassis.RF_MOTOR.Speed_PID, RF_Speed_PID_P,	RF_Speed_PID_I,	RF_Speed_PID_D,	Output_Limit | StepIn);	
	PidInit(&Chassis.LB_MOTOR.Speed_PID, LB_Speed_PID_P,	LB_Speed_PID_I,	LB_Speed_PID_D,	Output_Limit | StepIn);		
	PidInit(&Chassis.RB_MOTOR.Speed_PID, RB_Speed_PID_P,	RB_Speed_PID_I,	RB_Speed_PID_D,	Output_Limit | StepIn);
//  PidInit(&New_Chassis_Data.PID_information.z_speed, 40, 0.1 ,0, Output_Limit | Integral_Limit | Deadzone);
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Output_Limit, 10000, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Integral_Limit, 800, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Deadzone, 5, 0);

	PidInitMode(&Chassis.LF_MOTOR.Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.RF_MOTOR.Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.LB_MOTOR.Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.RB_MOTOR.Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    
	PidInitMode(&Chassis.LF_MOTOR.Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.RF_MOTOR.Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.LB_MOTOR.Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.RB_MOTOR.Speed_PID, StepIn, StepIn_Data, 0);
    
	PidInit(&Chassis.LF_MOTOR.Position_PID, LF_Position_PID_P, LF_Position_PID_I, LF_Position_PID_D, NONE);
	PidInit(&Chassis.RF_MOTOR.Position_PID, RF_Position_PID_P, RF_Position_PID_I, RF_Position_PID_D, NONE);
	PidInit(&Chassis.LB_MOTOR.Position_PID, LB_Position_PID_P, LB_Position_PID_I, LB_Position_PID_D, NONE);
	PidInit(&Chassis.RB_MOTOR.Position_PID, RB_Position_PID_P, RB_Position_PID_I, RB_Position_PID_D, NONE);
}
void PID_Calc(void)
{
	if(Chassis.Robot_cmd->Chassis_Mode == NO_FORCE)
	{
		Chassis.RF_MOTOR.current_input = 0;
        Chassis.RB_MOTOR.current_input = 0;
        Chassis.LB_MOTOR.current_input = 0;
        Chassis.LF_MOTOR.current_input = 0;
	}
    else if(Chassis.Robot_cmd->Chassis_State == LOCK_POSITION)
	{
        Chassis.RF_MOTOR.current_input = motor_position_speed_control(&Chassis.RF_MOTOR.Speed_PID,&Chassis.RF_MOTOR.Position_PID,0,Chassis.RF_MOTOR.Motor_encoder->Encode_Record_Val,Chassis.RF_MOTOR.Motor_Information->speed_data);
        Chassis.RB_MOTOR.current_input = motor_position_speed_control(&Chassis.RB_MOTOR.Speed_PID,&Chassis.RB_MOTOR.Position_PID,0,Chassis.RB_MOTOR.Motor_encoder->Encode_Record_Val,Chassis.RB_MOTOR.Motor_Information->speed_data);
        Chassis.LB_MOTOR.current_input = motor_position_speed_control(&Chassis.LB_MOTOR.Speed_PID,&Chassis.LB_MOTOR.Position_PID,0,Chassis.LB_MOTOR.Motor_encoder->Encode_Record_Val,Chassis.LB_MOTOR.Motor_Information->speed_data);
        Chassis.LF_MOTOR.current_input = motor_position_speed_control(&Chassis.LF_MOTOR.Speed_PID,&Chassis.LF_MOTOR.Position_PID,0,Chassis.LF_MOTOR.Motor_encoder->Encode_Record_Val,Chassis.LF_MOTOR.Motor_Information->speed_data);
	}
	else if(Chassis.Robot_cmd->Chassis_State == SPEED)
	{		
        Chassis.RF_MOTOR.current_input = PidCalculate(&Chassis.RF_MOTOR.Speed_PID, Chassis.Robot_cmd->Speed_set.RF_motor, Chassis.RF_MOTOR.Motor_Information->speed_data);
        Chassis.RB_MOTOR.current_input = PidCalculate(&Chassis.RB_MOTOR.Speed_PID, Chassis.Robot_cmd->Speed_set.RB_motor, Chassis.RB_MOTOR.Motor_Information->speed_data);	
        Chassis.LB_MOTOR.current_input = PidCalculate(&Chassis.LB_MOTOR.Speed_PID, Chassis.Robot_cmd->Speed_set.LB_motor, Chassis.LB_MOTOR.Motor_Information->speed_data);
        Chassis.LF_MOTOR.current_input = PidCalculate(&Chassis.LF_MOTOR.Speed_PID, Chassis.Robot_cmd->Speed_set.LF_motor, Chassis.LF_MOTOR.Motor_Information->speed_data);
    }
}
/**
  *@brif 打滑检测
  *@note 通过陀螺仪和轮组的加速度差检测
  */
void Slip_detection(void)
{
//    static float filted_accel[2];
//    static Recursive_ave_filter_type_t filter_IMU;
//    static first_order_filter_type_t ditong_filter_IMU;
//    static Recursive_ave_filter_type_t filter_Encoder;
//    static first_order_filter_type_t ditong_filter_Encoder; 
//    static float Not_Slip_Acc = 0;
//    
//    filted_accel[0] = Recursive_ave_filter(&filter_IMU, ins->Accel[Y], 100);
//	filted_accel[0] = first_order_filter(&ditong_filter_IMU,filted_accel[0]);
//    
//	filted_accel[1] = Recursive_ave_filter(&filter_Encoder, Chassis.RF_MOTOR.Motor_encoder->Acc_linear_speed * 0.7071067811865475f, 100);//分解成向前的速度
//	filted_accel[1] = first_order_filter(&ditong_filter_Encoder,	filted_accel[1]);
//    #define Slip_num 1.5f
//    if (abs(filted_accel[1] - filted_accel[0]) > Slip_num) {//打滑
//        
//    }
//    else {
//        Not_Slip_Acc = Chassis.RF_MOTOR.Motor_encoder->Acc_linear_speed;
//    }
}
//void Power_Limit();
void Chassis_Send_current()
{
    int16_t send[4];
    send[Chassis.RF_MOTOR.CanId-1] = Chassis.RF_MOTOR.current_input;
    send[Chassis.RB_MOTOR.CanId-1] = Chassis.RB_MOTOR.current_input;
    send[Chassis.LB_MOTOR.CanId-1] = Chassis.LB_MOTOR.current_input;
    send[Chassis.LF_MOTOR.CanId-1] = Chassis.LF_MOTOR.current_input;
    CAN1_Chassis_Tx(send[0],send[1],send[2],send[3]);
}
void Chassis_acion(void)
{
    PID_Calc();
    Slip_detection();	                        //打滑检测
    //Power_Limit();
    Chassis_Send_current();
}

/**
  *@brif 发送超点充电功率
  *@note 最小30W
  */
void Send_power_to_Supercap(void)
{
    static int16_t power = 30;
    Can1_Cap_Setmsg(power);//发送超电充能功率
}

uint32_t currentTime;
void Chassis_Movement(void const *argument)
{
	Chassis_Init();
	vTaskDelay(500);
	while(1)
	{
        taskENTER_CRITICAL(); 
        /*--------------------------------临界区--------------------------------*/
        currentTime = xTaskGetTickCount();                              //当前系统时间
        
        Chassis_cmd_set();
        
        Chassis_acion();
        
//        a_limit();														//加速度限制
//        CAN1_Chassis_Tx(New_Chassis_Data.Set_information.LF_current_input,
//                        New_Chassis_Data.Set_information.RF_current_input,
//						New_Chassis_Data.Set_information.LB_current_input,
//						New_Chassis_Data.Set_information.RB_current_input);	
//        Can1_Cap_Setmsg(power);//发送超电充能功率
        /*--------------------------------临界区--------------------------------*/
        taskEXIT_CRITICAL(); 
        vTaskDelayUntil(&currentTime, 1);//绝对延时//vTaskDelay(2);	
    }
}