#include "Init_task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

/************************* bsp ************************/
#include "bsp_can.h"
#include "bsp_dr16.h"
#include "bsp_referee.h"
/************************* Task ************************/
#include "chassis_task.h"
#include "fire_task.h"
#include "imu_task.h"
#include "safe_task.h"
#include "referee_task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

extern osThreadId Init_TASKHandle;
extern osThreadId defaultTaskHandle;

//osThreadId Safe_TASKHandle;
//osThreadId UI_TASKHandle;
osThreadId Chassis_TASKHandle;
osThreadId Fire_TaskHandler;
osThreadId TASK_CHASSISHandle;
osThreadId IMUTask_Handler;
osThreadId UITASK_Handler;

void init_Task(void const *argument)
{
		taskENTER_CRITICAL(); //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区
	
		// CAN初始化
        ECF_CAN_Init();
	
		//遥控器初始化
		ECF_RC_Init();
	
        //裁判系统初始化
		ECF_referee_uart_init();
	
		//创建底盘任务
		osThreadDef(Chassis_task, Chassis_Movement, osPriorityRealtime, 0, 512);
		Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);
	
		//创建火控任务
		osThreadDef(FIRE_TASK, Fire_Task, osPriorityNormal, 0, 128);
 		Fire_TaskHandler = osThreadCreate(osThread(FIRE_TASK), NULL);

//		// IMU
//        osThreadDef(IMU_TASK, ECF_IMU_Task, osPriorityNormal, 0, 512);
//        IMUTask_Handler = osThreadCreate(osThread(IMU_TASK), NULL);
//        
//        //创建安全任务
        osThreadDef(SAFE_TASK, Safe_Task, osPriorityNormal, 0, 256);
		Chassis_TASKHandle = osThreadCreate(osThread(SAFE_TASK), NULL);
//        
        osThreadDef(UI_Task, Referee_Task, osPriorityNormal, 0, 1024);
		Chassis_TASKHandle = osThreadCreate(osThread(UI_Task), NULL);
        
		vTaskDelete(Init_TASKHandle); //删除开始任务
		vTaskDelete(defaultTaskHandle);
		taskEXIT_CRITICAL();		  //退出临界区
}

