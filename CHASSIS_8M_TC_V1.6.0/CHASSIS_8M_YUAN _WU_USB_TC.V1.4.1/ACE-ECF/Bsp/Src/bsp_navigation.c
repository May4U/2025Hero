/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_navigation.c
* @brief
* @author love_sorrow 吴锴泓 (2567715886@qq.com)
* @version 1.0
* @date 2024-1-20
*
*
* @history
* <table>
* Date       Version Author Description
* 2024-1-20  1.0   吴锴泓  接收导航数据
* 使用方法：使用Get函数获取速度结构体指针,处理函数记得丢进任务里面实时进行
* 注意事项：需要自己配置虚拟串口并且到 usbd_cdc_if.c 里更换接收函数，并且引用一些变量
* @verbatim
* ==============================================================================
*/

#include "bsp_navigation.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

VcpRx_t temp1 = {
    .rxlen =0,
    .flag = 0
};

navi_t navi_data;

extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

Chassis_Speed Speed;

extern USBD_HandleTypeDef hUsbDeviceFS;

//这个是速度死区停下来
int count = 0;
//这个没有接收到数据停下来
int Receive_count = 0;

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief简介 速度结构体指针获取函数，z轴速度是°/s
 * 注意事项： 对 x，y 速度放大到 0~9000，请注意限幅
 ************************** Dongguan-University of Technology*-ACE************************** */
Chassis_Speed* Get_Navigation_Point(void)
{
	return &Speed;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief简介 导航数据解算函数
 * 功能： 加入死区限制和掉线检测
 ************************** Dongguan-University of Technology*-ACE************************** */
void USB_DataRecive(void)
{
	Receive_count++;
  if(temp1.flag)
  {
   // usb_printf("%s\r\n", UserRxBufferFS);//若无需回发，屏蔽即可
    temp1.flag = 0;
    temp1.rxlen = 0;
    for(uint8_t i=0;i<4;i++)
    {
      navi_data.x.byte[0+i]=UserRxBufferFS[1+i];
    }
   for(uint8_t i=0;i<4;i++)
    {
      navi_data.y.byte[0+i]=UserRxBufferFS[5+i];
    }
    for(uint8_t i=0;i<4;i++)
    {
      navi_data.z.byte[0+i]=UserRxBufferFS[9+i];
    }
    memset(UserRxBufferFS, 0, APP_RX_DATA_SIZE);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS); 
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		
//		//看看发过来的数据是否都在0附近
//		if(abs(navi_data.x.speed ) < USB_Dead && abs(navi_data.y.speed ) < USB_Dead)
//      count++;
//		else 
//			count = 0;
//		
//		//超过0.5s，则判定为停下来锁死
//		if(count > 500)
//		{
//			//让计数器不要无限增大
//			count = 501;

//			navi_data.x.speed = 0;
//			navi_data.y.speed = 0;
//			navi_data.z.speed = 0;
//		}
//  }
//	
//	  if(Receive_count > 500)
//		{
//			Receive_count = 501;
//			navi_data.x.speed = 0;
//			navi_data.y.speed = 0;
//			navi_data.z.speed = 0;
//		}
		
		Speed_Enlarge();
    }
}

void Speed_Enlarge(void)
{
	Speed.x = - navi_data.y.speed * Move_Speed_Factor;
	Speed.y = navi_data.x.speed * Move_Speed_Factor;
//	Speed.z = navi_data.z.speed * Rotate_Speed_Factor;
	Speed.z = navi_data.z.speed * Gimbal_Speed_Factor;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief简介 USB 中断函数
 * 注意事项： 到 usbd_cdc_if.c 里替换这个同名函数
 ************************** Dongguan-University of Technology*-ACE************************** */

//static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
//   temp1.rxlen = temp1.rxlen + (*Len);
//    
//    if(temp1.rxlen < APP_RX_DATA_SIZE && UserRxBufferFS[0] != 0xFF  //必须以0XFF开头
//        &&  UserRxBufferFS[temp1.rxlen - 1] != 0xFE)//***发送时每次发送必须以0xFE结尾***
//    {
//       //---继续接收---------------
//       USBD_CDC_SetRxBuffer(&hUsbDeviceFS,UserRxBufferFS  + temp1.rxlen); 
//       USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//    }
//    else 
//    {
//       temp1.flag = 1;    //接收完成
//       UserRxBufferFS[temp1.rxlen] = 0;
//			 Receive_count = 0;
//    }
//    return (USBD_OK);	
//  /* USER CODE END 6 */
//}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief简介 引用变量
 * 注意事项： 在 usbd_cdc_if.c 里加入以下部分
 ************************** Dongguan-University of Technology*-ACE************************** */
 
//#include "bsp_navigation.h" 
//extern VcpRx_t temp1;
//extern navi_t navi_data;
//extern int Receive_count;
