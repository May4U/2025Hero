/*************************** Dongguan-University of Technology -ACE**************************
 * @file    safe_task.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/11/5
 * @brief
 ******************************************************************************
 * @verbatim
 *  ��ȫ��������֧���û��Զ������ơ�ʧ�����ʱ�䡢ʧ���ص�����
 *  ʹ�÷�����
 *      ����һ����ȫ���񣬶������ּ��Զ���ʧ���ص�����
 *      ������ι�����ƣ����յ���Ϣʱˢ������״̬
 *      ���ﵽʧ����ֵʱִ���Զ�������
 *  demo��
 *       ��Init_task.c�е�init_Task()���������
 *       Safe_Init(1);//��������1ms
 *       //������ȫ����
 *       osThreadDef(SAFE_TASK, Safe_Task, osPriorityHigh, 0, 128);
 *		 Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);

 *       void online(void){enable�׹����}
 *       void disconnect(void){unable�׹����}
 *       //���밲ȫ����
 *       safe_task_t* demo_safe_task_ptr = Safe_task_add(demo,10,disconnect,online);
 *       //�����жϺ�����ι�������¶�ѡһ
 *       Safe_Task_online_ptr_(demo_safe_task_ptr);//����ָ��
 *       Safe_Task_online_name_(demo);             //��������
 * @attention
 *      ���Ҫʹ�û������ֵ�ι������ȷ�����ֲ��ظ������Բ���ֻ��ι�������ĵ�һ��
 *      �ǻ���ָ��������ν
 *      �����C++�������ؾͲ����ر���������׺����
 *      ɾ������Ҳ�����
 * @version
 * v1.0   �����汾
 ************************** Dongguan-University of Technology -ACE***************************/
#include "safe_task.h"
#include <string.h>
/*������������*/
#define TASK_INTERVAL_MS 1
uint32_t RUN_cycle_MS;
Head_t* Head;
uint8_t init_falg = 1;

/**
  * @brief  Safe_Init     ��ȫ�����ʼ��
  * @param  RUN_cycle_ms  ÿ���������ڣ�ms��
  * @retval null
  */
void Safe_Init(uint32_t RUN_cycle_ms)
{
    if (init_falg)
    {
        init_falg--;
        RUN_cycle_MS = RUN_cycle_ms;
        Head = (Head_t*)malloc(sizeof(Head_t));
        Head->first_task = NULL;
    }
}
/**
  * @brief  Safe_task_add  ���һ����ȫ����
  * @param  name[20]       ����
  * @param  Discon_ms      ���ʧ��ʱ�� 
  * @param  DisconnetCallBack     ʧ��ʱ���ûص�����
  * @param  OnlineCallback        ι��ʱ�������ߺ���
  * @retval temp_task_ptr  ��ȫ����ָ��
  */

safe_task_t* Safe_task_add(const char* name, uint32_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback)
{
    Safe_Init(TASK_INTERVAL_MS);
    safe_task_t* temp_task_ptr = (safe_task_t*)malloc(sizeof(safe_task_t));
    safe_task_t* last_task_ptr = Head->first_task;
    
    //д����ز���
    strcpy(temp_task_ptr->name, name);
    temp_task_ptr->Disconnection_threshold = Discon_ms;
    temp_task_ptr->Disconnection_count = 0;
    temp_task_ptr->Disconnection_falg = 0;
    temp_task_ptr->First_Disconnect = 0;
    temp_task_ptr->DisconnetCallBack = DisconnetCallBack;
    temp_task_ptr->OnlineCallBack = OnlineCallback;
    temp_task_ptr->next_task = NULL;
    //����β�巨
    if(Head->first_task == NULL)
    {
        Head->first_task = temp_task_ptr;
        return temp_task_ptr;
    }        
    while(last_task_ptr->next_task != NULL)     last_task_ptr = last_task_ptr->next_task;
    last_task_ptr->next_task = temp_task_ptr;
    
    return temp_task_ptr;
}

void SAFE_TASK(void const *argument)
{
	Safe_Init(TASK_INTERVAL_MS);//��Ȼ�����can���պ�dr16���յȻ���FreeRTOS����Safe_Taskǰ���ó�ʼ���������Է���һ
	while(1)
	{
		taskENTER_CRITICAL(); //�����ٽ���
        //������ѯ
        safe_task_t* temp_task_ptr = Head->first_task;
        while(temp_task_ptr != NULL)
        {
            temp_task_ptr->Disconnection_count += RUN_cycle_MS;//�����һ�������ʧ��ʱ��
            if (temp_task_ptr->Disconnection_count >= temp_task_ptr->Disconnection_threshold)//�ﵽʧ����ֵ
            {
                if(temp_task_ptr->First_Disconnect == 0)//��ֹ��ν���ʧ���ص�����
                {
                    temp_task_ptr->First_Disconnect++;
                    temp_task_ptr->Disconnection_falg = 1;//ʧ����־
                    temp_task_ptr->DisconnetCallBack();//ִ���û��Զ���ʧ���ص�����
                }
            }
            temp_task_ptr = temp_task_ptr->next_task;
        }
        
		taskEXIT_CRITICAL(); //�˳��ٽ���
		vTaskDelay(RUN_cycle_MS);
	}
}

/**
  * @brief  Safe_Task_online_name_  ��ȫ��������ˢ��
  * @param  temp_task_ptr           ˢ�µ�����ָ��
  * @return uint8_t                 ˢ�½��
  * @retval 1���ɹ�ˢ�£�0��ˢ��ʧ��
  */
uint8_t Safe_Task_online_ptr_(safe_task_t* temp_task_ptr)
{
    if(temp_task_ptr == NULL)   return 0;
    temp_task_ptr->Disconnection_count = 0;
    temp_task_ptr->Disconnection_falg = 0;
    temp_task_ptr->First_Disconnect = 0;
    temp_task_ptr->OnlineCallBack();
    return 1;
}

/**
  * @brief  Safe_Task_online_name_  ��ȫ��������ˢ��
  * @param  name                    ˢ�µ���������
  * @return uint8_t                 ˢ�½��
  * @retval 1���ɹ�ˢ�£�0��ˢ��ʧ��
  */
uint8_t Safe_Task_online_name_(char * name)
{
    safe_task_t* temp_task_ptr = Head->first_task;
    while(temp_task_ptr != NULL)
    {
        if (strcmp(temp_task_ptr->name,name) == 0)
        {
            temp_task_ptr->Disconnection_count = 0;
            temp_task_ptr->Disconnection_falg = 0;
            temp_task_ptr->First_Disconnect = 0;
            temp_task_ptr->OnlineCallBack();
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}

/**
  * @brief  Safe_task_delect_ptr_   ��ȫ����ɾ��
  * @param  name                    ɾ��������ָ��
  * @return uint8_t                 ɾ�����
  * @retval 1���ɹ�ɾ����0��ɾ��ʧ��
  */
uint8_t Safe_task_delect_ptr_(safe_task_t* delect_task_ptr)
{
    safe_task_t* temp_task_ptr = Head->first_task;
    
    if(temp_task_ptr == delect_task_ptr)
    {
        Head->first_task = NULL;
        free(delect_task_ptr);
        return 1;
    }
    
    while(temp_task_ptr->next_task != NULL)
    {
        if(temp_task_ptr->next_task == delect_task_ptr)
        {
            temp_task_ptr->next_task = delect_task_ptr->next_task;
            free(delect_task_ptr);
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}
/**
  * @brief  Safe_task_delect_name_  ��ȫ����ɾ��
  * @param  name                    ɾ������������
  * @return uint8_t                 ɾ�����
  * @retval 1���ɹ�ɾ����0��ɾ��ʧ��
  */
uint8_t Safe_Task_delect_name_(char * delect_task_name)
{
    safe_task_t* temp_task_ptr = Head->first_task;
    safe_task_t* delect_task_ptr;
    
    if(strcmp(temp_task_ptr->name,delect_task_name) == 0)
    {
        Head->first_task = NULL;
        free(delect_task_ptr);
        return 1;
    }
    
    while(temp_task_ptr->next_task != NULL)
    {
        if(strcmp(temp_task_ptr->next_task->name,delect_task_name) == 0)
        {
            delect_task_ptr = temp_task_ptr->next_task;
            temp_task_ptr->next_task = delect_task_ptr->next_task;
            free(delect_task_ptr);
            return 1;
        }
        temp_task_ptr = temp_task_ptr->next_task;
    }   
    return 0;
}