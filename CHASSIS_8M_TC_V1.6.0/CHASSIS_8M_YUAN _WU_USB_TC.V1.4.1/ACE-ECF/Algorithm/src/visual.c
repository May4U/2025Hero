#include "visual.h"
#include "imu_task.h"
#include "usbd_cdc_if.h"

/*--------------------变量-----------------------*/
vision_auto_data_t vision_auto_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
int c=0,Time=0,last_time=0;

/*float转char 使用共联体*/
typedef union  
{
    unsigned char  uc[4];   
    float          f;
}Float4Byte;

vision_auto_data_t *get_auto_control_point(void)
{
    return &vision_auto_data;
}

/**
  * @brief      发送数据给视觉
  * @param[in]  data: 敌方装甲板  红：0 | 蓝：1
  * @param[in]  mode: 模式选择    0：装甲模式  1：能量机关模式  2：陀螺模式
  * @param[in]  shoot_speed: 射速，目前是平均实际速度，其实就是当前上限射速-2
  * @retval     none
  * @attention  协议：帧头0xFF  数据  帧尾0xFE
  *             通过放入数据到fifo缓冲区，
  *             然后在调用函数将缓冲区的数据放进DMA的数组里面，开启DMA传输
  */
//volatile float nnnnn = 0;
 void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed,float yaw,float pitch)
{
    uint8_t SendBuff[29];
	  Float4Byte fp[6];

		for(int i=0;i<4;i++)
		{
			fp[i].f=INS.q[i];
		}
		fp[4].f = -yaw; 
		fp[5].f = -pitch;

    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
		
		//Append_CRC8_Check_Sum(SendBuff,3);
    SendBuff[2] = mode;
    SendBuff[3] = shoot_speed;
	  SendBuff[4] = fp[0].uc[0];
		SendBuff[5] = fp[0].uc[1];
		SendBuff[6] = fp[0].uc[2];
		SendBuff[7] = fp[0].uc[3];  
		SendBuff[8] = fp[1].uc[0];
		SendBuff[9] = fp[1].uc[1];
		SendBuff[10] = fp[1].uc[2];
		SendBuff[11] = fp[1].uc[3]; 
		SendBuff[12] = fp[2].uc[0];
		SendBuff[13] = fp[2].uc[1];
		SendBuff[14] = fp[2].uc[2];
		SendBuff[15] = fp[2].uc[3]; 
		SendBuff[16] = fp[3].uc[0];
		SendBuff[17] = fp[3].uc[1];
		SendBuff[18] = fp[3].uc[2];
		SendBuff[19] = fp[3].uc[3]; 		
		SendBuff[20] = fp[4].uc[0];
		SendBuff[21] = fp[4].uc[1];
		SendBuff[22] = fp[4].uc[2];
		SendBuff[23] = fp[4].uc[3]; 		
		SendBuff[24] = fp[5].uc[0];
		SendBuff[25] = fp[5].uc[1];
		SendBuff[26] = fp[5].uc[2];
		SendBuff[27] = fp[5].uc[3]; 		
    SendBuff[28] = 0xFE;
		
		CDC_Transmit_FS(SendBuff, ARR_SIZE(SendBuff));
}

/**
  * @brief      接收视觉回传数据
  * @param[in]  none
  * @retval     none
  * @attention  0xFF   0xFE
  *             通过接收fifo队列的数据接收（具体看相对应的串口文件）
  */
float show_yaw;
uint8_t visual_buff_read[32];
void visual_data_reception(void)
{
    

//    uint32_t length = fifo_read_buff(pfifo_visual, visual_buff_read, ARR_SIZE(visual_buff_read)); //将pfifo_visual的数据存到buff_read内
		uint32_t length=8;
    if (length)
    {
        for (uint16_t i = 0; i < length; i++)
        {
            if (visual_buff_read[i] == 0xFF && visual_buff_read[i + 7] == 0xFE)
            {
								vision_auto_data.auto_yaw_angle = (hex2Float(visual_buff_read[i + 2], visual_buff_read[i + 1]) / 100);        //自动打击的y轴角度计算/100
								show_yaw=(hex2Float(visual_buff_read[i + 2], visual_buff_read[i + 1]) / 100); 
                vision_auto_data.auto_pitch_angle = (hex2Float(visual_buff_read[i + 4], visual_buff_read[i + 3]) / 100);      //自动打击的p轴角度计算
                vision_auto_data.auto_pitch_angle_speed = (hex2Float(visual_buff_read[i + 6], visual_buff_read[i + 5]) / 10000); //自动打击的y轴角度计算/100
							  c++;
//								vision_auto_data.world_time1 = xTaskGetTickCount();
//							  vision_auto_data.auto_pitch_angle_speed = ((vision_auto_data.auto_pitch_angle - vision_auto_data.last_auto_pitch_angle)/ (vision_auto_data.world_time1 - vision_auto_data.last_world_time1))*1000;
//							  
	
//							  vision_auto_data.last_auto_pitch_angle = vision_auto_data.auto_pitch_angle;
//							  vision_auto_data.last_world_time1 = vision_auto_data.world_time1;
                //vision_auto_data.len = (hex2Float(MSDataBuffer[i + 6], MSDataBuffer[i + 5]) / 100);              //距离
	
                i = i + 7;
            }
        }
    }
    else
    {
      
			// 没有数据
    }
		
		Time = xTaskGetTickCount() * portTICK_PERIOD_MS;
		if(last_time - Time >= 1000)
		{
			last_time = Time;
			c-=c;
		}
		

}


/**
  * @brief      高低八位数据整合
  * @param[in]  HighByte: 高八位 
  * @param[in]  LowByte: 低八位
  * @retval     合并后的数据
  * @attention  
  */
float hex2Float(uint8_t HighByte, uint8_t LowByte)
{
    float high = (float)(HighByte & 0x7f);
    float low = (float)LowByte;

    if (HighByte & 0x80) //MSB is 1 means a negative number
    {
        return (high * 256.0f + low) - 32768;
    }
    else
    {
        return (high * 256.0f + low);
    }
}
