#ifndef __TASK_SAFE_H  //���δ����
#define __TASK_SAFE_H  //��ô����
#include <stdlib.h>
/*************************freertos*************************/
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

typedef void (*Callback)(void);

struct safe_task{
    char      name[50];                 //��������
    uint32_t  Disconnection_count;      //ʧ��������
    uint32_t  Disconnection_threshold;  //ʧ����ֵ
    uint8_t   Disconnection_falg;       //ʧ����־λ
    uint8_t   First_Disconnect;         //�״�ʧ����־λ����ֹ��ν���ʧ���ص�����
    void (*DisconnetCallBack)(void);    //�û��Զ���ʧ���ص�����
    void (*OnlineCallBack)(void); 
    struct safe_task *next_task;
};
typedef struct safe_task safe_task_t;
typedef struct safe_task_Head{
    struct safe_task *first_task;
}Head_t;

void Safe_Init(uint32_t RUN_cycle_ms);
void Safe_Task(void const *argument);

safe_task_t* Safe_task_add(const char* name, uint32_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback);
uint8_t Safe_Task_online_ptr_(safe_task_t* temp_task_ptr);
uint8_t Safe_Task_online_name_(char * name);
uint8_t Safe_task_delect_ptr_(safe_task_t* delect_task_ptr);
uint8_t Safe_Task_delect_name_(char * delect_task_name);
#endif