/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_can.c
 * @brief
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-24

 * @history
 * Date       Version Author Description
 *                     dji
 * 2022-07-24   1.0   侯文辉
 * 2022-12-06   2.0   侯文辉
 * 2023-12-26   3.0   郭嘉源
 * 2023-12-27   4.0   郑  楠
 * @verbatim
 * ==============================================================================
 * @verbatim
 *  完成CAN的初始化配置，包括开CAN发送 HAL_CAN_Start()
 *  接管CAN接收，对外提供统一接口，支持用户自定义接收的CAN，ID类型，回调函数
 *  使用方法：
 *      初始化函数ECF_CAN_Init()丢到Init_task.c中的init_Task()函数
 *      注册CAN接收回调函数
 *  demo：
 *       //在Init_task.c中的init_Task()函数里添加
 *       ECF_CAN_Init();
 *       //注册CAN接收
 *       ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Stdid, 0, DJIMotor_Can1_Call_Back);
 *       
 * @attention
 *      
 * @version
 * 1.0   基础版本
 * 2.0   升级版本 回环发送
 * 3.0   删除了接收相关函数，拉了发送相关的屎
 * 4.0   删除了发送相关函数，拉了接收相关的屎
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_can.h"
#define CAN_RX_CALLBACK_SIZE 20
/*
    CAN1 StdID 下标 0
    CAN1 ExtID 下标 1
    CAN2 StdID 下标 2
    CAN2 ExtID 下标 3
*/
typedef struct{
    uint32_t receive_id;//全0为不屏蔽
    can_rx_callback_fun_t Rx_Callback_fun;
}CAN_rx_callback_t;
static CAN_rx_callback_t CAN_rx_callback[4][CAN_RX_CALLBACK_SIZE] = {NULL};
uint8_t NUM_of_can_and_id_type[4] = {0,0,0,0};

/**
* @brief  CAN初始化
* @note   接收写死为FIFO0队列
*/
void ECF_CAN_Init(void) {
    // CAN过滤器初始化
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.SlaveStartFilterBank = 0;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    //开启CAN1传输
    HAL_CAN_Start(&hcan1);
    //开启对应的CAN中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);

    //CAN2部分
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_LAST_ERROR_CODE);
}


/**
* @brief  设置对应的CAN接收回调函数
* @param  hcan        接收的CAN
* @param  can_id_type 接收的ID类型(拓展帧或标准帧)
* @param  canID       接收的CanID
* @param  fun         接收的回调函数
* @return uint8_t     注册结果
* @retval 1：成功注册；0：注册失败(数组溢出，改CAN_RX_CALLBACK_SIZE大小)
*/
uint8_t ECF_CAN_Rx_Callback_Register(bsp_can_e hcan, bsp_can_id_type_e can_id_type, uint32_t canID, can_rx_callback_fun_t fun) 
{
    uint8_t temp_index = 0;
    if (hcan == Bsp_Can2)           temp_index = 2;
    if (can_id_type == Bsp_Extid)   temp_index++;
    if (NUM_of_can_and_id_type[temp_index] == CAN_RX_CALLBACK_SIZE) return 0;
    
    CAN_rx_callback[temp_index][NUM_of_can_and_id_type[temp_index]].receive_id = canID;
    CAN_rx_callback[temp_index][NUM_of_can_and_id_type[temp_index]].Rx_Callback_fun = fun;
    NUM_of_can_and_id_type[temp_index]++;
    return 1;
}

//接收到数据就进入中断的回调函数
#include "robot_cmd.h"
#include "Cybergear_Motor.h"
MI_Motor_t* MI_Motor;
MI_Motor_t* Get_MI_motor(void)
{
    MI_Motor = MI_motor_init(&hcan1);
    return MI_Motor;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef Rxmessage;
    uint8_t rx_data[8];
    uint8_t temp_index = 0;
    uint32_t Recive_CanID = 0;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxmessage, rx_data) == HAL_OK) // 读取接收的信息
    {
//        if (hcan == &hcan2 && Rxmessage.StdId == 0x401)
//            Deal_Dr16_form_chassis(&Rxmessage, rx_data);

        if (hcan == &hcan2)                 temp_index = 2;
        if (Rxmessage.IDE == CAN_ID_EXT)   //判断Recive_CanID是拓展id还是标准id  
        {
            //MI_motor_recive_callback(MI_Motor, Rxmessage.ExtId, rx_data);
            Recive_CanID = Rxmessage.ExtId;
            temp_index++;
        }            
        else    Recive_CanID = Rxmessage.StdId;
        
        for (int i = 0; i < NUM_of_can_and_id_type[temp_index]; i++)//遍历注册的回调函数
        {
            if (CAN_rx_callback[temp_index][i].receive_id == 0)//0则全盘接收
                CAN_rx_callback[temp_index][i].Rx_Callback_fun(&Rxmessage, rx_data);
            else if (CAN_rx_callback[temp_index][i].receive_id == Recive_CanID)
                CAN_rx_callback[temp_index][i].Rx_Callback_fun(&Rxmessage, rx_data);
        }
    }
}
