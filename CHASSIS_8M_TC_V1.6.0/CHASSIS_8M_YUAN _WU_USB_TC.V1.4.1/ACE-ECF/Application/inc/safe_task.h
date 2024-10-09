#ifndef __TASK_SAFE_H  //如果未定义
#define __TASK_SAFE_H  //那么定义
#include <stdlib.h>
/*************************freertos*************************/
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

typedef void (*Callback)(void);

struct safe_task{
    char      name[50];                 //任务名称
    uint32_t  Disconnection_count;      //失联计数器
    uint32_t  Disconnection_threshold;  //失联阈值
    uint8_t   Disconnection_falg;       //失联标志位
    uint8_t   First_Disconnect;         //首次失联标志位，防止多次进入失联回调函数
    void (*DisconnetCallBack)(void);    //用户自定义失联回调函数
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