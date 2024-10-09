#include "virtual_task.h"
#include "struct_typedef.h"
uint8_t Virtual_task_init_falg = 0;
uint8_t Get_Virtual_task_init_falg(){return Virtual_task_init_falg;}

#if USB_USE_PURPOSE == OU_VISION
#include "Ou_VisionPro.h"
#include "robot_cmd.h"
#include "bsp_dr16.h"
/*--------------------- TASK --------------------*/

//include获取云台状态，获取pitch实际值
#include "imu_task.h"
/*--------------------- FIRMWARE --------------------*/
#include "usbd_cdc_if.h"

#include "stdlib.h"
#include "string.h"
#include "filter.h"
#include "fifo.h"
#include "bsp_referee.h"
uint8_t virtual_task_init_falg = 0;
uint8_t get_virtual_task_init_falg(){return virtual_task_init_falg;}

gimbal_auto_control_t *auto_control_p;
uint8_t fire_flag;
uint8_t tracking;
extern INS_t INS;

#define PC_Vision 					0
#define PC_VisionPro				1
#define VISION_CONTROLLER   PC_VisionPro

static gimbal_auto_control_t *virtual_task_init(void);
static void Virtual_send(gimbal_auto_control_t *Virtual_send_p);
static void Virtual_recive(gimbal_auto_control_t *Virtual_recive_p);
static void Virtual_RC_Deal(gimbal_auto_control_t *Virtual_recive_p);
#ifdef VIRTUAL_DELAY_COMPENSATE
void gimbal_data_log(gimbal_auto_control_t *gimbal_data_log_p);
#endif
fp32 auto_yaw_Debug = 0;
fp32 auto_pitch_Debug = 0;
uint8_t receive_virtual_flag = 0;
void VIRTUAL_TASK(void const * argument)
{

 	auto_control_p = virtual_task_init();
	while(1)
	{
        Virtual_RC_Deal(auto_control_p);//键鼠微调
        
        if (get_open_auto_control() ==  MANUAL)
            auto_control_p->Get_out_post = 0;
        
//        auto_yaw_Debug = auto_control_p->auto_yaw;
//        auto_pitch_Debug = auto_control_p->auto_pitch;
        Virtual_recive(auto_control_p);
        Virtual_send(auto_control_p);
        taskENTER_CRITICAL();         // 进入临界区
        
        taskEXIT_CRITICAL();              // 退出临界区
		vTaskDelay(1);
	}
}

void Virtual_recive(gimbal_auto_control_t *Virtual_recive_p)
{
    receive_virtual_flag = 0;
	if(fifo_s_isempty(Virtual_recive_p->usb_fifo) != true)
	{
		uint8_t read_buff[50];

		fifo_s_gets(Virtual_recive_p->usb_fifo,(char*)read_buff,50);
		if(read_buff[0] == 0xFF && read_buff[49] == 0xFE)
			{
				#if(VISION_CONTROLLER == PC_Vision)
				Virtual_recive_p->auto_yaw = -(((float)((int16_t)(read_buff[2]<<8 | read_buff[1]))) / 100.0f);        //自动打击的y轴角度计算/100
				Virtual_recive_p->auto_pitch = -(((float)((int16_t)(read_buff[4]<<8 | read_buff[3]))) / 100.0f);      //自动打击的p轴角度计算
				Virtual_recive_p->auto_pitch_speed = (read_buff[5]<<8 | read_buff[6]) / 10000; //自动打击的p轴角度计算/100
				#elif(VISION_CONTROLLER == PC_VisionPro)
				memcpy(&Virtual_recive_p->tracking,&read_buff[1],1);//tracking标志位
				memcpy(&Virtual_recive_p->armor_id,&read_buff[2],1);
				memcpy(&Virtual_recive_p->armor_num,&read_buff[3],1);
				memcpy(&Virtual_recive_p->reserved,&read_buff[4],1);
				memcpy(&Virtual_recive_p->xw,&read_buff[5],44);//从xw后赋值视觉结算后的目标值
//                Virtual_recive_p->xw -= 0.1f;
//                Virtual_recive_p->zw -= z;
                //Virtual_recive_p->v_yaw = 0;
                if (Virtual_recive_p->xw != 0 && Virtual_recive_p->yw != 0 && Virtual_recive_p->zw != 0)
                {
                    autoSolveTrajectory(Virtual_recive_p, 1);
                    receive_virtual_flag = 1;
                }

				#endif
//				if(Virtual_recive_p->xw != 0 && Virtual_recive_p->yw != 0 && Virtual_recive_p->zw != 0)
//				{
//					autoSolveTrajectory();
//				}
				#ifdef VIRTUAL_DELAY_COMPENSATE
				Virtual_recive_p->gimbal_use_control[0] = Virtual_recive_p->history_gimbal_data[0] + Virtual_recive_p->auto_pitch;
				Virtual_recive_p->gimbal_use_control[1] = Virtual_recive_p->history_gimbal_data[1] + Virtual_recive_p->auto_yaw;
				gimbal_data_log(auto_control_p);
				#endif
			}
	}
    if (receive_virtual_flag == 0 && Virtual_recive_p->Get_out_post == 1)
        autoSolveTrajectory(Virtual_recive_p, 0);
}
/**
  * @brief      发送数据给视觉
  * @retval     none
  * @attention  协议：帧头0xFF  数据帧尾0xFE
  *             第二帧 data: 敌方装甲板  红：0 | 蓝：1
	* 						第三帧 mode: 模式选择    0：装甲模式  1：能量机关模式  2：陀螺模式
	* 						第四帧 shoot_speed: 射速，目前是平均实际速度，其实就是当前上限射速-2
  */
Recursive_ave_filter_type_t filter_shoot_speed;
extKalman_t Kalman_shoot_speed;                  //卡尔曼滤波器结构体
fp32 Send_Pitch_Compensate;
fp32 Pitch_Compensate = 1.8f;
static uint8_t filter_shoot_speed_init_flag = 0;
#define KALMAN    0//卡尔曼预测
#define RECURSIVE 1//递推预测
#define USING_KALEMAN RECURSIVE
void Virtual_send(gimbal_auto_control_t *Virtual_send_p)
{
    
        //射速取样
    if (filter_shoot_speed_init_flag == 0)
    {
        filter_shoot_speed_init_flag = 1;
        #if (USING_KALEMAN == KALMAN)
        KalmanCreate(&Kalman_shoot_speed,20,200);      //初始化该滤波器的Q=20 R=200参数
        #elif (USING_KALEMAN == RECURSIVE)
        Recursive_ave_filter_init(&filter_shoot_speed);
        #endif

    }
    static float Last_Shoot_Speed = 0;
    //加入射速样本
    if (auto_control_p->referee->Shoot_Data.bullet_speed>10 && auto_control_p->referee->Shoot_Data.bullet_speed < 18 && auto_control_p->referee->Shoot_Data.bullet_speed != Last_Shoot_Speed)
    {
        Last_Shoot_Speed = auto_control_p->referee->Shoot_Data.bullet_speed;
        #if (USING_KALEMAN == KALMAN)
        auto_control_p->bullet_speed = KalmanFilter(&Kalman_shoot_speed,auto_control_p->referee->Shoot_Data.bullet_speed);   //对数据进行卡尔曼滤波
        #elif (USING_KALEMAN == RECURSIVE)
        auto_control_p->bullet_speed = Recursive_ave_filter(&filter_shoot_speed, auto_control_p->referee->Shoot_Data.bullet_speed, 2);//取样最近2次
        #endif
    }
    //取最近五次射速均值
    
//	switch (*Virtual_send_p->gimbal_behaviour)
//	{
//    case GIMBAL_AUTOATTACK:
//			  //暂用
//				Virtual_send_p->visual_buff_send[2] = 0;
//        break;
////				return;
//    case GIMBAL_SMALL_AUTOBUFF:
//				Virtual_send_p->visual_buff_send[2] = 1;
//        break;
//    case GIMBAL_BIG_AUTOBUFF:
//				Virtual_send_p->visual_buff_send[2] = 2;
//        break;
//    default:
//        break;
//	}
//		//裁判系统接收 阵营，射速。
//		//敌方阵营 0red 1bule
		Virtual_send_p->visual_buff_send[1] = 0;
//		//射速
//		Virtual_send_p->visual_buff_send[3] = Virtual_send_p->referee->Robot_Status.shooter_id1_17mm_speed_limit;
		Virtual_send_p->visual_buff_send[2] = Virtual_send_p->reset_tracker;
		Virtual_send_p->visual_buff_send[3] = Virtual_send_p->reserved;//原本是读取弹速限制发送给视觉，现改为保留位
        Send_Pitch_Compensate = Virtual_send_p->Imu_c->Pitch + Pitch_Compensate;
			#if(VISION_CONTROLLER == PC_Vision)
			{
				//浮点转字节
				{
					float register *Register1 = INS.q;
					uint8_t register *Register2 = &Virtual_send_p->visual_buff_send[4];
					int register Register3;
					__asm__ volatile
					{
						LDR Register3 , [Register1],#4
						STR	Register3 , [Register2],#4
						LDR Register3 , [Register1],#4
						STR	Register3 , [Register2],#4
						LDR Register3 , [Register1],#4
						STR	Register3 , [Register2],#4
						LDR Register3 , [Register1],#4
						STR	Register3 , [Register2],#4
						LDR Register3 , [&INS.Yaw]
						STR Register3 , [Register2],#4
						LDR Register3 , [&INS.Pitch]
						STR Register3 , [Register2]
					}
				}
				#elif(VISION_CONTROLLER == PC_VisionPro)
				{
					{
							float *x = &Virtual_send_p->Target_Position.x;//发送当前计算后瞄准目标值
							float *y = &Virtual_send_p->Target_Position.y;
							float *z = &Virtual_send_p->Target_Position.z;
							uint8_t register *Register2 = &Virtual_send_p->visual_buff_send[4];
							int register Register3;
							__asm__ volatile
							{ // 汇编指令，LDR读，STR写//发送当前陀螺仪数据
								LDR Register3 , [&INS.Roll]
								STR Register3 , [Register2],#4
								LDR Register3 , [&Send_Pitch_Compensate]
								STR Register3 , [Register2],#4
								LDR Register3 , [&INS.Yaw]
								STR Register3 , [Register2],#4
								LDR Register3 , [x],#4
								STR	Register3 , [Register2],#4
								LDR Register3 , [y],#4
								STR	Register3 , [Register2],#4
								LDR Register3 , [z],#4
								STR	Register3 , [Register2]
							}
					}
				}
			#endif
		CDC_Transmit_FS(Virtual_send_p->visual_buff_send, sizeof(Virtual_send_p->visual_buff_send));
}

			
#ifdef VIRTUAL_DELAY_COMPENSATE
void gimbal_data_log(gimbal_auto_control_t *gimbal_data_log_p)
{
	#if(PITCH_ANGLE_SENSOR == PITCH_USE_IMU)
	gimbal_data_log_p->history_gimbal_data[0] = gimbal_data_log_p->Imu_c->Roll;
	#elif(PITCH_ANGLE_SENSOR == PITCH_USE_ENCODER)
	//TODO:未测试
//	gimbal_data_log_p->Pitch_c->pitch_motor.actPositon_360 = ((float)gimbal_data_log_p->Pitch_c->pitch_motor_encoder->Encode_Actual_Val * 360.0f / 8192.0f - PITCH_ZERO_OFFSET);
	Gimbal_pitch_positon360_update();
	gimbal_data_log_p->history_gimbal_data[0] = gimbal_data_log_p->Pitch_c->pitch_motor.actPositon_360;
	#endif
	gimbal_data_log_p->history_gimbal_data[1] = gimbal_data_log_p->Imu_c->Yaw;
}
#endif

gimbal_auto_control_t *virtual_task_init(void)
{
	gimbal_auto_control_t *virtual_task_init_p;
	virtual_task_init_p = malloc(sizeof(gimbal_auto_control_t));
	memset(virtual_task_init_p, 0, sizeof(gimbal_auto_control_t));

	// 获取陀螺仪指针
	virtual_task_init_p->Imu_c = get_imu_control_point();
	virtual_task_init_p->referee = Get_referee_Address();//裁判系统指针
	virtual_task_init_p->usb_fifo = fifo_s_create(96);//创建长度为96的fifo
	virtual_task_init_p->gimbal_pitch = get_Gimbal_pitch_point();//云台pitch设定值
	virtual_task_init_p->gimbal_yaw = get_Gimbal_yaw_point();//云台yaw设定值
	virtual_task_init_p->gimbal_behaviour = NULL;//这个用不着，屏蔽了 get_gimbal_behaviour_point();//云台状态获取
	
	virtual_task_init_p->visual_buff_send[0]  = 0xFF;//帧头
	virtual_task_init_p->visual_buff_send[28] = 0xFE;//帧尾
	//virtual_task_init_p->Pitch_c = get_gimbal_pitch_point();//获取pitch轴参数
	
	    //定义参数
  virtual_task_init_p->k = 0.042;
	virtual_task_init_p->xw = 3.0;
// virtual_task_init_p->yw = 0.0159;
  virtual_task_init_p->yw = 0;
// virtual_task_init_p->zw = -0.2898;
  virtual_task_init_p->zw = 1.5;
  virtual_task_init_p->vxw = 0;
  virtual_task_init_p->vyw = 0;
  virtual_task_init_p->vzw = 0;
  virtual_task_init_p->v_yaw = 0;
//virtual_task_init_p->tar_yaw = 0.09131;
  virtual_task_init_p->r1 = 0.5;
  virtual_task_init_p->r2 = 0.5;
  virtual_task_init_p->dz = 0.1;
  virtual_task_init_p->bias_time = 125;
  virtual_task_init_p->s_bias = 0.0f;
  virtual_task_init_p->z_bias = 0.0f;
  virtual_task_init_p->armor_id = ARMOR_INFANTRY3;
  virtual_task_init_p->armor_num = 0;
	virtual_task_init_p->bullet_speed = 15.5f;
	virtual_task_init_p->max_iter = 10;
	virtual_task_init_p->R_K_iter = 50;
	virtual_task_init_p->g = 9.78;
	virtual_task_init_p->stop_error = 0.001;

    virtual_task_init_p->fire_flag = 0;
    virtual_task_init_p->Get_out_post = 0;
    virtual_task_init_p->auto_fire_delay_time_ms = 0;
    virtual_task_init_p->fly_time_ms = 0;
    virtual_task_init_p->Next_Can_Fire_time = 0;
    virtual_task_init_p->yaw_compensate = -0.50;
    virtual_task_init_p->pitch_compensate = -0.50;
    virtual_task_init_p->RC = RC_Get_RC_Pointer();
    virtual_task_init_p->last_receive_time_ms = 0;
    Virtual_task_init_falg = 1;
    virtual_task_init_falg = 1;
	return virtual_task_init_p;
}

#include "bsp_dwt.h"
fp32    masure_bias_time = 125.0f;
float start_time_ms = 0;
void Flash_fire_bias_time(uint8_t Start_0_End_1)
{
    int bias_time;
    if (Start_0_End_1 == 0 && start_time_ms == 0)
        start_time_ms = DWT_GetTimeline_ms();//防止覆盖
    
    if (Start_0_End_1 == 1 && start_time_ms != 0)
    {
        bias_time = DWT_GetTimeline_ms() -  start_time_ms;
        start_time_ms = 0;
    }
    
    if (bias_time > 0 && bias_time < 1000)
    {
        masure_bias_time = bias_time;
        //
        //auto_control_p->auto_fire_delay_time_ms = masure_bias_time;
    }
}

void gimbal_clear_virtual_recive(void)
{
	if(auto_control_p == NULL)
	{
		return;
	}
	auto_control_p->auto_pitch = 0;
	auto_control_p->auto_yaw = 0;
}
gimbal_auto_control_t *get_auto_control_point(void)
{
	return auto_control_p;
}

uint8_t *get_auto_fire_point(void)
{
	return &fire_flag;
}
uint8_t *get_tracking_point(void)
{
	return &tracking;
}
void Virtual_RC_Deal(gimbal_auto_control_t *Virtual_recive_p)
{
    static uint8_t mouse_z_up = 1;
    static uint8_t mouse_z_down = 1;
    if (auto_control_p->RC->kb.bit.SHIFT == 1)//调节pitch
    {
        if (auto_control_p->RC->mouse.z>0)
        {
            if (mouse_z_up == 1)    auto_control_p->pitch_compensate+=0.1f;
            mouse_z_up = 0;
        }        
        else
            mouse_z_up = 1;
        
        if (auto_control_p->RC->mouse.z<0)
        {
            if (mouse_z_down == 1)    auto_control_p->pitch_compensate-=0.1f;
            mouse_z_down = 0;
        }        
        else
            mouse_z_down = 1;
    }
    
    if (auto_control_p->RC->kb.bit.CTRL == 1)//调节发弹延迟
    {
        if (auto_control_p->RC->mouse.z>0)
        {
            if (mouse_z_up == 1)            auto_control_p->auto_fire_delay_time_ms+=50.0f;
            mouse_z_up = 0;
        }        
        else
            mouse_z_up = 1;
        
        if (auto_control_p->RC->mouse.z<0)
        {
            if (mouse_z_down == 1)            auto_control_p->auto_fire_delay_time_ms-=50.0f;
            mouse_z_down = 0;
        }        
        else
            mouse_z_down = 1;
    }
    auto_control_p->bias_time = 125 - auto_control_p->auto_fire_delay_time_ms;
}
float* get_AutoPitch(void){return &auto_control_p->auto_pitch;}
float* get_AutoYaw(void){return &auto_control_p->auto_yaw;}
#endif
#if USB_USE_PURPOSE == KUN_VISION
#include <string.h>
#include "imu_task.h"
#include "usbd_cdc_if.h"
#include "bsp_Parabolic_calculation.h"
KUN_Vision_Receive_t    KUN_Vision_Receive;//视觉接收信息结构体
KUN_Vision_Control_t    KUN_Vision_Control;

static const INS_t       *ins;
u8_and_float pitch_imu;
u8_and_float yaw_imu;
u8_and_float roll_imu;
uint8_t send_buff[29];

float* get_AutoPitch(void)
    {return &KUN_Vision_Control.AutoPitch;}
    
KUN_Vision_Control_t *get_auto_control_point(void)
    {return &KUN_Vision_Control;}

void virtual_task_init()
{
    ins = get_imu_control_point();
    
    memset(send_buff, 0, sizeof(send_buff));
    send_buff[0] = 0XFF;
    send_buff[28] = 0XFE;
    
    memset(&KUN_Vision_Receive, 0, sizeof(KUN_Vision_Receive_t));
    memset(&KUN_Vision_Control, 0, sizeof(KUN_Vision_Control_t));
    
    Virtual_task_init_falg = 1;
}
#include "PC_VisionPro.h"
fp32 bullet_speed_k = 15;
fp32 now_time;
fp32 Next_Fire_time;
fp32 Fly_Time = 0.12f;//单位 s
void Virtual_recive()
{
    fp32 distance = sqrt(KUN_Vision_Receive.x.f*KUN_Vision_Receive.x.f + KUN_Vision_Receive.y.f*KUN_Vision_Receive.y.f);
    
    KUN_Vision_Control.AutoPitch = (float)(pitchTrajectoryCompensation(distance,KUN_Vision_Receive.z.f, bullet_speed_k )* 180.f / PI);
    KUN_Vision_Control.AutoYaw = (float)(atan2(KUN_Vision_Receive.y.f,KUN_Vision_Receive.x.f)*180.0f/PI);
    if (KUN_Vision_Receive.in_place_falg != 1)  return;
    
    KUN_Vision_Receive.in_place_falg = 0;//接收清零标志位
//    fp32 Fly_Time;//单位 s
//    fp32 Distance = Calc_The_Distance(KUN_Vision_Receive.x.f, KUN_Vision_Receive.y.f);//计算平面距离
//    KUN_Vision_Control.AutoPitch = Calc_Pitch_Angle(-0.25, 5, &Fly_Time);
    
    //目前没有写入计算预判开火时间
    if (Next_Fire_time == 0)//Next_Fire_time值在JudgeCanFire()函数中处理
    {
        KUN_Vision_Receive.in_place_falg = 0;
        now_time = DWT_GetTimeline_ms();
        Next_Fire_time = calc_fire_time(now_time, Fly_Time*1000);
        //Next_Fire_time = now_time + 417;//单位ms
    }    
}

void Fire_Judge(void)
{
    now_time = DWT_GetTimeline_ms();
    uint8_t JudgeCanFire_num = JudgeCanFire(now_time, &Next_Fire_time);//判断是否到达预判开火时期并处理预开火时间戳
    if (JudgeCanFire_num == 1)   
        KUN_Vision_Control.Fire_flag = 1;
    if (JudgeCanFire_num == 2)//发火时机已过，清零火控标志
        KUN_Vision_Control.Fire_flag = 0;
}

void Virtual_send()
{
    pitch_imu.f = ins->Pitch;
    yaw_imu.f   = ins->Yaw;
    roll_imu.f  = ins->Roll;
    send_buff[4] = roll_imu.u8[0];
    send_buff[5] = roll_imu.u8[1];
    send_buff[6] = roll_imu.u8[2];
    send_buff[7] = roll_imu.u8[3];
    
    send_buff[8] = pitch_imu.u8[0];
    send_buff[9] = pitch_imu.u8[1];
    send_buff[10] = pitch_imu.u8[2];
    send_buff[11] = pitch_imu.u8[3];
    
    send_buff[12] = yaw_imu.u8[0];
    send_buff[13] = yaw_imu.u8[1];
    send_buff[14] = yaw_imu.u8[2];
    send_buff[15] = yaw_imu.u8[3];
    
    CDC_Transmit_FS(send_buff, 29);//发送数据给视觉
}
void Virtual_Task(void const * argument)
{
 	virtual_task_init();
	while(1)
	{
        taskENTER_CRITICAL();         // 进入临界区
        Virtual_recive();
        Fire_Judge();
        Virtual_send();
        taskEXIT_CRITICAL();              // 退出临界区
		vTaskDelay(2);
	}
}
#endif