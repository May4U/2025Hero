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

static Fire_State_e ON_OR_OFF = NO_Fire_FORCE;//Ħ���ֿ���״̬


int16_t speed = 100;
/**
  *@brief ��������
  *@note  �����˵�ָ��󣬲���������λ���˵�ָ��ǰ�趨��λ�ã������´ν���ָ���ʱ��ת���˵�ָ��ǰ�趨��λ��
  */
void fire_work(void)
{
    
    static uint8_t Block = 0;//��ת��־
    static uint16_t On_Fire_Block_time = 0;//������תʱ��
    static uint16_t Return = 0;
    
    if (fire_task.Chassis->Robot_cmd->increat_time > 0)
    {
        fire_task.Chassis->Robot_cmd->increat_time--;
        if (fire_task.Empty_position != 0)//�����˵�ָ���δ������趨λ��
        {
            fire_task.set_position = fire_task.Empty_position;
            fire_task.Empty_position = 0;
        }
        else
            fire_task.set_position = fire_task.set_position - (69632); //����6�ŵ�����һ��ת60�㣬8192*51�����ٱȣ�/6 
        
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;//�趨�����ʵ��
    } 
        
   
    //������
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Fire)
    {
        if(fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val < fire_task.set_position + 500)//�������г�
        {
            fire_task.Chassis->Robot_cmd->Fire_State = Ready_Fire;
        }
    }


    if (fire_task.Chassis->Robot_cmd->Fire_State == NO_Fire_FORCE)
    {//����

        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;
        EncoderValZero(&fire_task.Chassis->FIRE_MOTOR->Motor_encoder);
        fire_task.set_position = 0;
        fire_task.Empty_position = 0;
        fire_task.Chassis->FIRE_MOTOR->set_position = 0;
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Speed_PID);
        pid_clear(&fire_task.Chassis->FIRE_MOTOR->Position_PID);
    }else
        fire_task.Chassis->FIRE_MOTOR->Using_PID = Position_Speed_PID;
    
    static uint8_t Save_Set_Position = 1;//����ִ���˵�ָ��ʱδ��ɵ�Ŀ��λ��
    if (fire_task.Chassis->Robot_cmd->Fire_State == On_Empty)
    {
        fire_task.Chassis->FIRE_MOTOR->Using_PID = No_Current;//�˵�������
        
        fire_task.set_position = fire_task.Chassis->FIRE_MOTOR->Motor_encoder.Encode_Record_Val;
        fire_task.Chassis->FIRE_MOTOR->set_position = fire_task.set_position;
        
        if (Save_Set_Position)//����δ��ɵ�Ŀ��λ��
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
		taskENTER_CRITICAL(); //�����ٽ���
        Fire_Set();
		fire_work();
		taskEXIT_CRITICAL(); //�˳��ٽ���
		vTaskDelay(1);
	}
}

