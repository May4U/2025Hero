#include "fire_task.h"
/* ************************freertos******************** */

#include "CAN2.h"
fire_task_t fire_task;

void fire_init(void)
{  
    fire_task.Chassis = get_chassis_point();
    fire_task.set_position = 0;
    fire_task.last_set_position = 0;
    fire_task.Empty_position = 0;
}

static Fire_State_e ON_OR_OFF = NO_Fire_FORCE;//摩擦轮开关状态


int16_t speed = 100;
/**
  *@brief 开火任务
  *@note  启动退弹指令后，不会主动归位到退弹指令前设定的位置，但当下次进弹指令到来时会转到退弹指令前设定的位置
  */
void fire_work(void)
{
    
    static uint8_t Block = 0;//堵转标志
    static uint16_t On_Fire_Block_time = 0;//进弹堵转时长
    static uint16_t Return = 0;
    
    if (fire_task.Chassis->Robot_cmd->increat_time > 0)
    {
        fire_task.Chassis->Robot_cmd->increat_time--;
        if (fire_task.Empty_position != 0)//启动退弹指令后，未到达的设定位置
        {
            fire_task.set_position = fire_task.Empty_position;
            fire_task.Empty_position = 0;
        }
        else
            fire_task.set_position = fire_task.set_position - (69632); //拨盘6颗弹，打一发转60°，8192*51（减速比）/6 
        
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;//设定到电机实例
    } 
        
   
    //进弹中
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Fire)
    {
        if(fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val < fire_task.set_position + 500)//检查进弹行程
        {
            fire_task.Chassis->Robot_cmd->Fire_State = Ready_Fire;
        }
    }


    if (fire_task.Chassis->Robot_cmd->Fire_State == NO_Fire_FORCE)
    {//无力

        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;
        EncoderValZero(&fire_task.Chassis->FIRE_MOTOR->Motor_encoder);
        fire_task.set_position = 0;
        fire_task.Empty_position = 0;
        fire_task.Chassis->FIRE_MOTOR->set_position = 0;
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Speed_PID);
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Position_PID);
    }else
        fire_task.Chassis->FIRE_MOTOR->Using_PID = Position_Speed_PID;
    
    static uint8_t Save_Set_Position = 1;//保存执行退弹指令时未完成的目标位置
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Empty)
    {
        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;//退蛋，无力
        
        fire_task.set_position = fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val;
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
        
        if (Save_Set_Position)//保存未完成的目标位置
        {       
            Save_Set_Position = 0;            
            fire_task.Empty_position = fire_task.set_position;
        }
    }
    else
    {
        Save_Set_Position = 1;
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
    }
}

void Fire_Task(void const *argument)
{
	fire_init();
	while(1)
	{
		taskENTER_CRITICAL(); //进入临界区
        Fire_Set();
		fire_work();
		taskEXIT_CRITICAL(); //退出临界区
		vTaskDelay(1);
	}
}

