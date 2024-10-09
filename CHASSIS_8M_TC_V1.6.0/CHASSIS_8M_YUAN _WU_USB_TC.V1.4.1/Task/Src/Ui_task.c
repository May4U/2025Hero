#include "Ui_task.h"
#include "chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "rm_cilent_ui.h"

#define MAX_X 1920
#define MAX_Y 1080

const static char *chassis_mode_ui[4] = {
    "FOLLOW",      //跟随
    "NO_FOLLOW",   //不跟随   
    "ROTATION",    //小陀螺
    "Battery"};    //炮台模式

chassis_control_t *Chassis_Control_p = NULL;
Graph_Data diff_ang;

Float_Data CAP_VOLTAGE;
String_Data CH_MODE,voltage,replenish;
		
static void ui_init(void);           //ui初始化
static void print_mode(void);        //底盘模式显示
static void print_chassis_gimbal_angle(void);
static void print_cap_voltage(void);
/**
  * @brief      ui显示任务
  * @param[in]  *pvParameters
  * @retval     void    
  * @attention  
  */
void UI_Task(void const * argument)
{
	int count =0;
	vTaskDelay(10);
	Chassis_Control_p = Get_Chassis_Control_p();
	ui_init();
	while (1)
	{
		print_cap_voltage();
		print_chassis_gimbal_angle();
		print_mode();

		count++;
		if(count == 100)
		{
			ui_init();
			count = 0;
		}
		vTaskDelay(1);
	}
}


	
/** 
  * @brief      ui初始化
  * @param[in]  void
  * @retval     void    
  * @attention  没有的话，重启一下
  */
static void ui_init(void)
{
		
		/*--------------------电容ui加入--------------------*/
		Char_Draw(&voltage, "078", UI_Graph_ADD, 7, UI_Color_Green, 20, 4, 1080, 300, "Voltage:");
		My_Char_Refresh(voltage);
		Float_Draw(&CAP_VOLTAGE, "075", UI_Graph_ADD, 6, UI_Color_Green, 20, 32, 3, 1230, 300, (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000));		//x的偏移为150左右比较正常
		My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);

		/*--------------------底盘云台差角ui加入--------------------*/
		//以中点为圆心，云台方向为前，距离200为半径，画底盘灯条方位、
		uint32_t start_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle -45 -180 ,0,360);
		uint32_t end_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle +45 -180,0,360);
		Arc_Draw(&diff_ang,"001",UI_Graph_ADD ,6 ,UI_Color_Main ,start_angle,end_angle,5,MAX_X/2,MAX_Y/2,200,200);
		My_Graph_Refresh(&diff_ang);

		/*--------------------底盘模式ui加入--------------------*/
		Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 8, UI_Color_Main, 25, 5, 80, 880-80, chassis_mode_ui[Chassis_Control_p->behaviour]);
		My_Char_Refresh(CH_MODE);
	
		/*--------------------云台补弹ui加入--------------------*/
		if(Chassis_Control_p->gimbal_data_p->replenish_flag)
			Char_Draw(&replenish, "002", UI_Graph_ADD, 7, UI_Color_Green, 20, 4, 80, 600, "replenish:");
		else
			Char_Draw(&replenish, "002", UI_Graph_ADD, 7, UI_Color_Orange, 20, 4, 80, 600, "replenish:");
		My_Char_Refresh(replenish);
}

/**
  * @brief      云台、底盘模式显示
  */
static void print_mode(void)
{
	//底盘
    Char_Draw(&CH_MODE, "093", UI_Graph_Change, 8, UI_Color_Main, 25, 5, 80, 880-80, chassis_mode_ui[Chassis_Control_p->behaviour]);
    My_Char_Refresh(CH_MODE);
		if(Chassis_Control_p->gimbal_data_p->replenish_flag)
			Char_Draw(&replenish, "002", UI_Graph_Change, 7, UI_Color_Green, 20, 4, 80, 600, "replenish:");
		else
			Char_Draw(&replenish, "002", UI_Graph_Change, 7, UI_Color_Orange, 20, 4, 80, 600, "replenish:");
		My_Char_Refresh(replenish);
	//云台
//	GI_MODE.Graph_Control.end_angle = strlen(gimbal_mode_ui[re_gimbal_behaviour()]);
//	strcpy(GI_MODE.show_Data, gimbal_mode_ui[re_gimbal_behaviour()]);
//	GI_MODE.Graph_Control.operate_tpye = UI_Graph_Change;
//	My_Char_Refresh(GI_MODE);
}

/**
  * @brief      电容电压显示
  */
static void print_cap_voltage(void)
{
    CAP_VOLTAGE.graph_Float = (int)(Chassis_Control_p->super_cap_c->Capacitance_voltage * 1000);
    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}
/**
  * @brief      地盘云台角度显示
  */
static void print_chassis_gimbal_angle(void)
{
		uint32_t start_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle -45 -180,0,360);
		uint32_t end_angle = loop_fp32_constrain(Chassis_Control_p->Chassis_Gimbal_Diference_Angle +45 -180,0,360);
		Arc_Draw(&diff_ang,"001",UI_Graph_Change ,6 ,UI_Color_Main ,start_angle,end_angle,5,MAX_X/2,MAX_Y/2,200,200);
		My_Graph_Refresh(&diff_ang);
}
