/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_can.h
* @brief
* @author pansyhou侯文辉 (1677195845lyb@gmail.com)
* @version 1.0
* @date 2022-07-24
*
*
* @history
* <table>
* Date       Version Author Description
* 2022-07-24   1.0   侯文辉
* 2022-12-06   2.0   侯文辉
* 2023-12-26   3.0   郭嘉源
* 2023-12-27   4.0   郑  楠
* @verbatim
* ==============================================================================
* 文件内容捏
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/

#ifndef _ECF_BSP_CAN
#define _ECF_BSP_CAN
#include "can.h"

//用Bsp前缀防止奇奇怪怪的同名
typedef enum{
    Bsp_Can1,
    Bsp_Can2
}bsp_can_e;
typedef enum{
    Bsp_Stdid,
    Bsp_Extid
}bsp_can_id_type_e;
typedef void (*can_rx_callback_fun_t)(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *data);//数据处理指针

void ECF_CAN_Init(void);
uint8_t ECF_CAN_Rx_Callback_Register(bsp_can_e hcan, bsp_can_id_type_e can_id_type, uint32_t canID, can_rx_callback_fun_t fun);

#endif

