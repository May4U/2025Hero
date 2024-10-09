#include "CAN1.h"
#include "CAN2.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "bsp_Motor_Encoder.h"
#include "safe_task.h"

#define Get_Motor_Measure_Data(ptr, CAN1_Rx_Data)                                        	  \
{                                                                                           \
		(ptr)->last_position_data		 = (ptr)->position_data;                                    \
		(ptr)->position_data				 = (int16_t)(CAN1_Rx_Data[0] << 8 | CAN1_Rx_Data[1]);       \
		(ptr)->speed_data						 = (int16_t)(CAN1_Rx_Data[2] << 8 | CAN1_Rx_Data[3]);       \
		(ptr)->current 	 						 = (int16_t)(CAN1_Rx_Data[4] << 8 | CAN1_Rx_Data[5]);				\
		(ptr)->temperature					 =  CAN1_Rx_Data[6];                                        \
}																																														

Motor_2006_3508_6020_t LF_Motor;
Motor_2006_3508_6020_t RF_Motor;
Motor_2006_3508_6020_t LB_Motor;
Motor_2006_3508_6020_t RB_Motor;
Motor_2006_3508_6020_t FIRE_Motor;

//CAN1_t CAN_Motor;

Supercapacitor_receive_t Supercap_receive;

pid_parameter_t jiasudu;
float speed;
float add_speed;

void (*can1_callback)(CAN_HandleTypeDef *);			//回调函数
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static CAN_RxHeaderTypeDef CAN1_Rxmessage;					//接收信息结构体
uint8_t  CAN1_Rx_Data[8];    										//接受数据的数组

static CAN_TxHeaderTypeDef CAN1_Txmessage;						//发送的信息
static uint32_t send_mail_box;									//发送邮箱
uint8_t  CAN1_Tx_Data[8];												//发送数据的数组

static void CAN1_RX_Deal(CAN_HandleTypeDef *hcan);
extern void CAN2_RX_Deal(CAN_HandleTypeDef *hcan);
void CAN1_Clear(Motor_2006_3508_6020_t *Motor_3508);

uint32_t last_call_back_time[5];
extern Encoder_t CAN_Encoder[5];//在bsp_Motor_Encoder中声明
/**
 * @brief		CAN1滤波器配置
 * @param		none
 *	@retval		none
 */
void CAN1_filter_config(void)
{
	CAN_FilterTypeDef CAN1_FIilter_InitStruct;

	CAN1_FIilter_InitStruct.FilterActivation = ENABLE;			 //开启滤波器
	CAN1_FIilter_InitStruct.FilterMode = CAN_FILTERMODE_IDMASK;	 //掩码模式
	CAN1_FIilter_InitStruct.FilterScale = CAN_FILTERSCALE_32BIT; // 32位工作
	CAN1_FIilter_InitStruct.FilterIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdHigh = 0x0000;
	CAN1_FIilter_InitStruct.FilterMaskIdLow = 0x0000;
	CAN1_FIilter_InitStruct.FilterBank = 0;
	CAN1_FIilter_InitStruct.SlaveStartFilterBank = 14;
	CAN1_FIilter_InitStruct.FilterFIFOAssignment = CAN_RX_FIFO0;	   //指定接收邮箱
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_FIilter_InitStruct);			   //根据指定配置CAN接收过滤器
	HAL_CAN_Start(&hcan1);											   //开启can1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //启动中断
}

uint8_t Motor_online[4];



void CAN1_Init(void)
{
	CAN1_filter_config();
//	CAN_Motor.CAN1_Chassis_Tx = CAN1_Chassis_Tx;
//	CAN_Motor.CAN1_Fire_Tx	  = CAN1_Fire_Tx;
//	CAN_Motor.can1_cap_setmsg = Can1_Cap_Setmsg;
//    CAN_Motor.last_call_back_time = last_call_back_time;
	can1_callback 			  		= CAN1_RX_Deal;
	CAN1_Clear(&LF_Motor);
	CAN1_Clear(&RF_Motor);
	CAN1_Clear(&LB_Motor);
	CAN1_Clear(&RB_Motor);	
	CAN1_Clear(&FIRE_Motor);	
	PidInit(&jiasudu,0,0,1,NULL);
}

/**
 * @brief		HAl库can1的回调函数
 * @param		传入参数： CAN的句柄
 * @retval   none
 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	if (hcan == &hcan1)
//	{
//		CAN1_RX_Deal(hcan);
//	}
//	if (hcan == &hcan2)
//	{
//		CAN2_RX_Deal(hcan);
//	}
//}

//const CAN1_t *Get_CAN1_Address(void)
//{
//	return &CAN_Motor;
//}
uint32_t *Get_last_call_back_time(void)
{
    return &last_call_back_time[0];
}
//电容
Supercapacitor_receive_t *get_supercap_control_point(void)
{
	return &Supercap_receive;
}


const Motor_2006_3508_6020_t *Get_Measure_Address(uint32_t i)
{
	switch(i)
	{
		case CHASSIS_MOTOR_LF_ID:
		{
			return &LF_Motor;
		}
		case CHASSIS_MOTOR_RF_ID:
		{
			return &RF_Motor;
		}
		case CHASSIS_MOTOR_LB_ID:
		{
			return &LB_Motor;
		}
		case CHASSIS_MOTOR_RB_ID:
		{
			return &RB_Motor;
		}
		case FIRE_MOTOR_ID:
		{
			return &FIRE_Motor;
		}
		default:
		return NULL;
		
	}

}
void CAN1_Send_2006_3508(int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor)
{
    CAN1_Txmessage.StdId 	= CHASSIS_MOTOR_GENERAL_ID;
	CAN1_Txmessage.IDE		= CAN_ID_STD;
	CAN1_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN1_Txmessage.DLC 		= 0x08;
	
	CAN1_Tx_Data[0] = ID_1_Motor >> 8;
	CAN1_Tx_Data[1] = ID_1_Motor;
	CAN1_Tx_Data[2] = ID_2_Motor >> 8;
	CAN1_Tx_Data[3] = ID_2_Motor;
	CAN1_Tx_Data[4] = ID_3_Motor >> 8;
	CAN1_Tx_Data[5] = ID_3_Motor;
	CAN1_Tx_Data[6] = ID_4_Motor >> 8;
	CAN1_Tx_Data[7] = ID_4_Motor;

	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
}

void CAN1_Send_6020(int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor)
{
    CAN1_Txmessage.StdId 	= FIRE_MOTOR_GENERAL_ID;
	CAN1_Txmessage.IDE		= CAN_ID_STD;
	CAN1_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN1_Txmessage.DLC 		= 0x08;
	
	CAN1_Tx_Data[0] = ID_1_Motor >> 8;
	CAN1_Tx_Data[1] = ID_1_Motor;
	CAN1_Tx_Data[2] = ID_2_Motor >> 8;
	CAN1_Tx_Data[3] = ID_2_Motor;
	CAN1_Tx_Data[4] = ID_3_Motor >> 8;
	CAN1_Tx_Data[5] = ID_3_Motor;
	CAN1_Tx_Data[6] = ID_4_Motor >> 8;
	CAN1_Tx_Data[7] = ID_4_Motor;

	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
}

void CAN1_Chassis_Tx(int16_t ID_1_Motor, int16_t ID_2_Motor, int16_t ID_3_Motor, int16_t ID_4_Motor)
{
	CAN1_Txmessage.StdId 	= CHASSIS_MOTOR_GENERAL_ID;
	CAN1_Txmessage.IDE		= CAN_ID_STD;
	CAN1_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN1_Txmessage.DLC 		= 0x08;
	
	CAN1_Tx_Data[0] = ID_1_Motor >> 8;
	CAN1_Tx_Data[1] = ID_1_Motor;
	CAN1_Tx_Data[2] = ID_2_Motor >> 8;
	CAN1_Tx_Data[3] = ID_2_Motor;
	CAN1_Tx_Data[4] = ID_3_Motor >> 8;
	CAN1_Tx_Data[5] = ID_3_Motor;
	CAN1_Tx_Data[6] = ID_4_Motor >> 8;
	CAN1_Tx_Data[7] = ID_4_Motor;

	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
}

void CAN1_Fire_Tx( int16_t FIRE_Motor)
{
	CAN1_Txmessage.StdId	= FIRE_MOTOR_GENERAL_ID;
	CAN1_Txmessage.IDE		= CAN_ID_STD;
	CAN1_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN1_Txmessage.DLC 		= 0x08;
	
	CAN1_Tx_Data[0] = FIRE_Motor >> 8;
	CAN1_Tx_Data[1] = FIRE_Motor;
	CAN1_Tx_Data[2] = 0;
	CAN1_Tx_Data[3] = 0;
	CAN1_Tx_Data[4] = 0;
	CAN1_Tx_Data[5] = 0;
	CAN1_Tx_Data[6] = 0;
	CAN1_Tx_Data[7] = 0;

	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box);			//将一段数据通过 CAN 总线发送
}

/**
 * @brief		CAN1 发送电容的值
 * @param		none
 *	@retval		none
 */
void Can1_Cap_Setmsg(int16_t Chassis_power)
{
	Chassis_power = Chassis_power * 100;

	CAN1_Txmessage.StdId = 0x210;	  //根据820r设置标识符   （根据情况修改）
	CAN1_Txmessage.IDE = CAN_ID_STD;	  //指定将要传输的消息的标识符的类型
	CAN1_Txmessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
	CAN1_Txmessage.DLC = 2;

	CAN1_Tx_Data[0] = (Chassis_power >> 8);
	CAN1_Tx_Data[1] = Chassis_power;
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0));
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}

static void CAN1_RX_Deal(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_Rxmessage, CAN1_Rx_Data) == HAL_OK)	//读取接收的信息
	{
	if(CAN1_Rxmessage.StdId==NULL)
	{
		return;
	}
	switch (CAN1_Rxmessage.StdId)
	{
		case CHASSIS_MOTOR_LF_ID:
		{
            Safe_Task_online_name_("LF_Motor");
			Get_Motor_Measure_Data(&LF_Motor,CAN1_Rx_Data);
			CAN_DATA_Encoder_Deal(LF_Motor.position_data, LF_Motor.speed_data, &CAN_Encoder[CHASSIS_MOTOR_LF_ENCODER]);
			break;
		}
		case CHASSIS_MOTOR_RF_ID:
		{
            Safe_Task_online_name_("RF_Motor");
//			Get_Motor_Measure_Data(&RF_Motor,CAN1_Rx_Data);
//			CAN_DATA_Encoder_Deal(LF_Motor.position_data, LF_Motor.speed_data, &CAN_Encoder[CHASSIS_MOTOR_RF_ENCODER]);
			break;
		}
		case CHASSIS_MOTOR_LB_ID:
		{
            Safe_Task_online_name_("LB_Motor");
			break;
		}
		case CHASSIS_MOTOR_RB_ID:
		{
            Safe_Task_online_name_("RB_Motor");
			Get_Motor_Measure_Data(&RB_Motor,CAN1_Rx_Data);
			CAN_DATA_Encoder_Deal(LF_Motor.position_data, LF_Motor.speed_data, &CAN_Encoder[CHASSIS_MOTOR_RB_ENCODER]);	
			break;
		}
		case FIRE_MOTOR_ID:
		{
			Get_Motor_Measure_Data(&FIRE_Motor,CAN1_Rx_Data);
			CAN_DATA_Encoder_Deal(FIRE_Motor.position_data, FIRE_Motor.speed_data, &CAN_Encoder[CHASSIS_MOTOR_FIRE_ENCODER]);
			break;
		}
		case SUPERCAP_ID: //超级电容接收
		{
			Supercap_receive.input_voltage = (float)((CAN1_Rx_Data[1] << 8 | CAN1_Rx_Data[0])/ 100.0f);
			Supercap_receive.Capacitance_voltage = (float)((CAN1_Rx_Data[3] << 8 | CAN1_Rx_Data[2])/ 100.0f);
			Supercap_receive.Input_current = (float)((CAN1_Rx_Data[5] << 8 | CAN1_Rx_Data[4])/ 100.0f);
			Supercap_receive.Set_power = (float)((CAN1_Rx_Data[7] << 8 | CAN1_Rx_Data[6])/ 100.0f);
			break;
		}
		default:
			break;
		}
	}
}

void CAN1_Clear(Motor_2006_3508_6020_t *Motor_3508)
{
	Motor_3508->last_position_data = 0;
	Motor_3508->position_data   	 = 0;
	Motor_3508->speed_data  			 = 0;
	Motor_3508->temperature				 = 0;
	Motor_3508->current 				   = 0;
}
