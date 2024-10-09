#include "CAN1.h"
#include "CAN2.h"
#include "maths.h"
#include "stdlib.h"
//#include "FSM_Chassis.h"
#include "chassis_task.h"
#include "PID_Data.h"
#include "stm32f4xx_hal.h"

void (*can2_callback)(CAN_HandleTypeDef *);			//回调函数
extern CAN_HandleTypeDef hcan2;

static CAN_RxHeaderTypeDef CAN2_Rxmessage;					//接收信息结构体
uint8_t  CAN2_Rx_Data[8];    										//接受数据的数组

static CAN_TxHeaderTypeDef CAN2_Txmessage;						//发送的信息
static uint32_t send_mail_box;									//发送邮箱
uint8_t  CAN2_Tx_Data[8];												//发送数据的数组

//extern New_Chassis_Task_t New_Chassis_Data;
//extern Chassis_Task_t Chassis_Data;
//FSM_Chassis_t  *can_FSM_Chassis;
#include "bsp_referee.h"
#include "bsp_dr16.h"

static const REFEREE_t   *referee_cmd;
const RC_ctrl_t   *rc_ctl;

void CAN2_filter_config(void)
{
  CAN_FilterTypeDef CAN2_FIilter_InitStruct;

  //开启滤波器
  CAN2_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;  //掩码模式
  CAN2_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32位工作
  CAN2_FIilter_InitStruct.FilterIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
  CAN2_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
  CAN2_FIilter_InitStruct.FilterBank = 14; // CAN2 滤波器组为14
  CAN2_FIilter_InitStruct.SlaveStartFilterBank = 14;
  CAN2_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0; //指定接收邮箱
  CAN2_FIilter_InitStruct.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &CAN2_FIilter_InitStruct);            //根据指定配置CAN接收过滤器
  HAL_CAN_Start(&hcan2);                                             //开启can2
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //启动中断
}

void CAN2_Init(void)
{
	CAN2_filter_config();
	can2_callback = CAN2_RX_Deal;
	//can_FSM_Chassis = Get_FSM_Chassis_Address();
	rc_ctl = RC_Get_RC_Pointer();
}

void CAN2_RX_Deal(CAN_HandleTypeDef *hcan)
{
		if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_Rxmessage, CAN2_Rx_Data) == HAL_OK)	//读取接收的信息
	{
	if(CAN2_Rxmessage.StdId==NULL)
	{
		return;
	}
	switch (CAN2_Rxmessage.StdId)
	{
//		case 0x300:
//						__asm__ 
//			(
//				"LDR r7 , [CAN2_Rx_Data] \n"
//				"STR r7 , [&can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal] \n"
//			);
//			break;
		case 0x205: // Y轴
		{
//			can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal = (int16_t)(CAN2_Rx_Data[0] << 8 | CAN2_Rx_Data[1]);
//			can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal = (can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal - YAW_ZERO_OFFSET) * 360.0f / 8192.0f;
//			can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal = loop_fp32_constrain(can_FSM_Chassis->Difference_Angle_between_Chassis_Gimbal, -180.0f, 180.0f);

//			New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal = (int16_t)(CAN2_Rx_Data[0] << 8 | CAN2_Rx_Data[1]);
//			New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal = (New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal - YAW_ZERO_OFFSET) * 360.0f / 8192.0f;
//			New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal = loop_fp32_constrain(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal, -180.0f, 180.0f);

			break;
		}
		default:
			break;
		}
	}
}

#include "robot_define.h"
void  Send_to_Gimbal(bool fire_on_off)
{
    CAN2_Txmessage.StdId 	= 0x401;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x08;

	CAN2_Tx_Data[0] = (rc_ctl->rc.ch[2]) >> 8;
	CAN2_Tx_Data[1] = (rc_ctl->rc.ch[2]);
    
    #ifdef USE_NAVIGATION//导航模式，此遥控值作为Z旋转，云台电机锁死，不改变YAW角度
    CAN2_Tx_Data[0] = 0;
	CAN2_Tx_Data[1] = 0;
    #endif
    
	CAN2_Tx_Data[2] = (rc_ctl->rc.ch[3]) >> 8;
	CAN2_Tx_Data[3] = (rc_ctl->rc.ch[3]);
    if (fire_on_off == true)    CAN2_Tx_Data[4] = 1;
	else                        CAN2_Tx_Data[4] = 0;
	CAN2_Tx_Data[5] = (rc_ctl->kb.key_code) >> 8;
	CAN2_Tx_Data[6] = (rc_ctl->kb.key_code);
	CAN2_Tx_Data[7] = 0xAA;
	
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
    
    CAN2_Txmessage.StdId 	= 0x407;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x08;
	CAN2_Tx_Data[0] = (rc_ctl->mouse.x) >> 8;
	CAN2_Tx_Data[1] = (rc_ctl->mouse.x);
	CAN2_Tx_Data[2] = (rc_ctl->mouse.y) >> 8;
	CAN2_Tx_Data[3] = (rc_ctl->mouse.y);
	CAN2_Tx_Data[4] = (rc_ctl->mouse.press_l);
	CAN2_Tx_Data[5] = (rc_ctl->mouse.press_r);
	CAN2_Tx_Data[6] = (rc_ctl->rc.s2);
	CAN2_Tx_Data[7] = 0;
	
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
}

//void CAN2_Send(void)
//{
//	CAN2_Txmessage.StdId 	= 0x401;
//	CAN2_Txmessage.IDE		= CAN_ID_STD;
//	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
//	CAN2_Txmessage.DLC 		= 0x08;
//	
//	CAN2_Tx_Data[0] = (rc_ctl->rc.ch[2]) >> 8;
//	CAN2_Tx_Data[1] = (rc_ctl->rc.ch[2]);
//	CAN2_Tx_Data[2] = (rc_ctl->rc.ch[3]) >> 8;
//	CAN2_Tx_Data[3] = (rc_ctl->rc.ch[3]);
//	CAN2_Tx_Data[4] = Chassis_Data.fire_on_flag;
//	CAN2_Tx_Data[5] = (rc_ctl->kb.key_code) >> 8;
//	CAN2_Tx_Data[6] = (rc_ctl->kb.key_code);
//	CAN2_Tx_Data[7] = 0xAA;
//	
//	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送

//	CAN2_Tx_Data[0] = (rc_ctl->mouse.x) >> 8;
//	CAN2_Tx_Data[1] = (rc_ctl->mouse.x);
//	CAN2_Tx_Data[2] = (rc_ctl->mouse.y) >> 8;
//	CAN2_Tx_Data[3] = (rc_ctl->mouse.y);
//	CAN2_Tx_Data[4] = (rc_ctl->mouse.press_l);
//	CAN2_Tx_Data[5] = (rc_ctl->mouse.press_r);
//	CAN2_Tx_Data[6] = (rc_ctl->rc.s2);
//	CAN2_Tx_Data[7] = Chassis_Data.referee->Robot_Status.shooter_id1_42mm_speed_limit;
//	
//	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送

//}

//void CAN2_Send_Gambal(void)
//{
//    CAN2_Send();
//}
