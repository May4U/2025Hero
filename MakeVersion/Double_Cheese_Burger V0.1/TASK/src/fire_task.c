#include "fire_Task.h"
#include "bsp_dr16.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"


#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "gimbal_config.h"
#include "bsp_referee.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
//#include "PC_VisionPro.h"

Fire_t Fire;

static void fire_task_init(void);
static void fire_pid_calculate(void);
static void fire_behaviour_choose(void);
static void Shoot_Check(void);

int16_t left_speed = 0;
int16_t right_speed = 0;
int16_t left_speed_up = 0;
int16_t right_speed_up = 0;
int8_t speed_error = 0;
void Send_Fire_Current()
{
    static CAN_TxHeaderTypeDef CAN_TxHeader;
    static uint8_t Can_Tx_Data[8];
    Can_Tx_Data[0] = Fire.right_motor_up->current_input >> 8;
    Can_Tx_Data[1] = Fire.right_motor_up->current_input ;
    Can_Tx_Data[2] = Fire.right_motor->current_input >> 8;
    Can_Tx_Data[3] = Fire.right_motor->current_input ;
    Can_Tx_Data[4] = Fire.left_motor->current_input >> 8;
    Can_Tx_Data[5] = Fire.left_motor->current_input ;
    Can_Tx_Data[6] = Fire.left_motor_up->current_input >> 8;
    Can_Tx_Data[7] = Fire.left_motor_up->current_input ;

    static uint32_t send_mail_box;
    CAN_TxHeader.StdId = 0x200;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = 0x08;
    HAL_CAN_AddTxMessage(&hcan1, &CAN_TxHeader, Can_Tx_Data, &send_mail_box);
}

void FIRE_TASK(void const *argument)
{
    static uint16_t time = 0;


	fire_task_init();
	while (1)
	{
        time ++;
        if (time > 1000)    time = 0;
                taskENTER_CRITICAL();         // 进入临界区
        
        left_speed = Fire.left_motor->Motor_Information.speed;
        right_speed = -Fire.right_motor->Motor_Information.speed;
        left_speed_up = Fire.left_motor_up->Motor_Information.speed;
        right_speed_up = -Fire.right_motor_up->Motor_Information.speed;
        speed_error = left_speed - right_speed;
        
        Deal_Fire_Set_Rpm();//改变摩擦轮转速
        Send_Fire_Set_Rpm();//发送底盘摩擦轮设定转速
        Send_Fire_Motor_Speed(left_speed_up, right_speed_up, left_speed, right_speed);//发送底盘摩擦轮实际转速
				//Send_Fire_Motor_Speed(4800, 4800, 4800, 4800);//发送底盘摩擦轮实际转速
        
        Gimbal_Fire_State_Set();
		fire_behaviour_choose();
		fire_pid_calculate();
        
        Shoot_Check();//发射检查，发弹延迟校准
        
//        if (time % 2 ==0)   Send_Fire_Current();//500hz控制频率
        
		DJIMotor_Send(Fire.left_motor);
        taskEXIT_CRITICAL();              // 退出临界区


        
		vTaskDelay(1);
	}
}

void fire_task_init(void)
{
	memset(&Fire, 0, sizeof(Fire_t));
	// 获得拨弹指针
    Fire.left_motor_up = DJIMotor_Init(1,2,false,M3508,30);
    Fire.right_motor_up = DJIMotor_Init(1,4,true,M3508,30);
    Fire.left_motor  = DJIMotor_Init(1,1,false,M3508,30);
    Fire.right_motor = DJIMotor_Init(1,3,true,M3508,30);

    Fire.left_motor->Using_PID = Speed_PID;
    Fire.left_motor_up->Using_PID = Speed_PID;
    Fire.right_motor_up->Using_PID = Speed_PID;
    Fire.right_motor->Using_PID = Speed_PID;
    
    PidInit(&Fire.left_motor->Speed_PID,1.0f,0,0,Output_Limit|StepIn|Integral_Limit);
 	PidInitMode(&Fire.left_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.left_motor->Speed_PID,StepIn,10,0);//过大的加减速会导致发射机构超电流断电
    PidInitMode(&Fire.left_motor->Speed_PID,Integral_Limit,1000,0);

    PidInit(&Fire.right_motor->Speed_PID,2.75f,0,0,Output_Limit|StepIn|Integral_Limit);
 	PidInitMode(&Fire.right_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.right_motor->Speed_PID,StepIn,10,0);//逐渐减速，实测发现最大速度减速会导致发射机构超电流断电
    PidInitMode(&Fire.left_motor->Speed_PID,Integral_Limit,1000,0);

    PidInit(&Fire.left_motor_up->Speed_PID,1.0f,0,0,Output_Limit|StepIn);
 	PidInitMode(&Fire.left_motor_up->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.left_motor_up->Speed_PID,StepIn,10,0);//过大的加减速会导致发射机构超电流断电
    PidInit(&Fire.right_motor_up->Speed_PID,1.0f,0,0,Output_Limit|StepIn);
 	PidInitMode(&Fire.right_motor_up->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.right_motor_up->Speed_PID,StepIn,10,0);//逐渐减速，实测发现最大速度减速会导致发射机构超电流断电
    
	Fire.Gimbal_CMD = Get_Gimbal_CMD_point();
}

void fire_behaviour_choose(void)
{
    if (Fire.Gimbal_CMD->Fire_Ready == 0)
    {
        DJIMotor_Set_val(Fire.left_motor_up, 0);
        DJIMotor_Set_val(Fire.right_motor_up, 0);
        DJIMotor_Set_val(Fire.left_motor, 0);
        DJIMotor_Set_val(Fire.right_motor, 0);
        HAL_GPIO_WritePin(fire_set_GPIO_Port, fire_set_Pin, GPIO_PIN_SET);
    }
    else
    {
        DJIMotor_Set_val(Fire.left_motor_up, Fire.Gimbal_CMD->Fire_Set_Rpm);
        DJIMotor_Set_val(Fire.right_motor_up,Fire.Gimbal_CMD->Fire_Set_Rpm);
        DJIMotor_Set_val(Fire.left_motor,    Fire.Gimbal_CMD->Fire_Set_Rpm);
        DJIMotor_Set_val(Fire.right_motor,   Fire.Gimbal_CMD->Fire_Set_Rpm);
        HAL_GPIO_WritePin(fire_set_GPIO_Port, fire_set_Pin, GPIO_PIN_RESET);
    }

}

void fire_pid_calculate(void)
{
    Fire.left_motor->current_input = motor_speed_control(&Fire.left_motor->Speed_PID,
                                                          Fire.left_motor->set_speed,
                                                          Fire.left_motor->Motor_Information.speed);
    Fire.right_motor->current_input = motor_speed_control(&Fire.right_motor->Speed_PID,
                                                          Fire.right_motor->set_speed,
                                                          Fire.right_motor->Motor_Information.speed);
    Fire.left_motor_up->current_input = motor_speed_control(&Fire.left_motor_up->Speed_PID,
                                                          Fire.left_motor_up->set_speed,
                                                          Fire.left_motor_up->Motor_Information.speed);
    Fire.right_motor_up->current_input = motor_speed_control(&Fire.right_motor_up->Speed_PID,
                                                          Fire.right_motor_up->set_speed,
                                                          Fire.right_motor_up->Motor_Information.speed);
}
uint16_t fla_speed = 100;
void Shoot_Check(void)
{
    if (abs(left_speed) > 2000 || abs(right_speed) > 2000 || abs(left_speed_up) > 2000 || abs(right_speed_up) > 2000)
        Set_Fire_Open_State(1);
    else
        Set_Fire_Open_State(0);
    
    if (Fire.right_motor->set_speed != Fire.Gimbal_CMD->Fire_Set_Rpm)   return;
    // if (Fire.right_motor->Motor_Information.speed < Fire.Gimbal_CMD->Fire_Set_Rpm - fla_speed)
    // {
    //     Flash_fire_bias_time(1);
    // }
}