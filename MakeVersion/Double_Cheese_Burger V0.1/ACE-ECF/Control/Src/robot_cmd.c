#include "robot_cmd.h"
#if BOARD == CHASSIS
#include "bsp_dr16.h"
#include "maths.h"
#include "pid.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "bsp_Motor_Encoder.h"
#include <stdbool.h>
Robot_cmd_t Robot_cmd;

/*�����˶�����ĸ�������*/
static const REFEREE_t   *referee_cmd;
static const RC_ctrl_t   *RC_ctrl;  
fp32         Difference_Angle_between_Chassis_Gimbal;
static const INS_t       *ins;
static pid_parameter_t   chassis_follow_gambal_speed;


/**
  *@brief ���ص����˶���������ָ��
  */
Robot_cmd_t* get_Robot_cmd_point(void)
{
    static uint8_t init_flag = 0;
    if (init_flag++ == 0) CMD_Init();
    return &Robot_cmd;
}

/**
  *@brief ��ȡ�����˶���������ĸ�������ָ��
  */
void CMD_Init(void)
{
    RC_ctrl = RC_Get_RC_Pointer();
    Robot_cmd.Chassis_Mode = NO_FOLLOW;
    ins = get_imu_control_point();
}

void Mode_Set(void)
{
    static Chassis_Mode_t last_Chassis_Mode = NO_FOLLOW;
    static Chassis_Mode_t RC_Chassis_Mode = NO_FOLLOW;//ģʽ���ݣ������ж�����״̬�ı�
    
    if (RC_ctrl->rc.s2 == RC_SW_DOWN){//ң������࿪��״̬Ϊ[��]
        Robot_cmd.Chassis_Mode = NO_FORCE;//���ж�ģʽ
        return;
    }
    switch(RC_ctrl->rc.s1)//ң����࿪��
    {
        case RC_SW_DOWN://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = NO_FOLLOW;//���̲�������̨
            break;
        case RC_SW_MID://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = FOLLOW;//���̸�����̨
            break;
        case RC_SW_UP://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = SPIN;//����С����ģʽ
            break;
        default://ɶģʽ��û������ֹ���������ж�ģʽ
            RC_Chassis_Mode = NO_FORCE;
            break;
    }
    //��⵽�ı��������Robot_cmd.Chassis_Mode�������ü������õ�ģʽ�������ɿ��������ֱ���ԭ��ģʽ���ݸ���
    if(last_Chassis_Mode != RC_Chassis_Mode)
    {
        Robot_cmd.Chassis_Mode = RC_Chassis_Mode;
    }
    last_Chassis_Mode = RC_Chassis_Mode;
    
    if(RC_ctrl->kb.bit.Q == 1)  Robot_cmd.Chassis_Mode = FOLLOW;
    if(RC_ctrl->kb.bit.E == 1)  Robot_cmd.Chassis_Mode = SPIN;
    if(RC_ctrl->kb.bit.R == 1)  Robot_cmd.Chassis_Mode = NO_FOLLOW;
}

static void NO_FORCE_Mode()
{
	Robot_cmd.Speed_set.chassis_x	=	0;
    Robot_cmd.Speed_set.chassis_y	=	0;
	Robot_cmd.Speed_set.chassis_z	=	0;
}

static void NO_FOLLOW_Mode()
{
	Robot_cmd.Speed_set.chassis_z = 0;

	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Difference_Angle_between_Chassis_Gimbal);
}

static void FOLLOW_Mode()
{
	Robot_cmd.Speed_set.chassis_z = PidCalculate(&chassis_follow_gambal_speed, 0, Difference_Angle_between_Chassis_Gimbal);
    
    Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Difference_Angle_between_Chassis_Gimbal);
}

static void SPIN_Mode()
{
    //����Ӣ�۵ȼ�ѡ�������ٶ�
	switch(referee_cmd->Robot_Status.robot_level)
	{
		case 1:
			Robot_cmd.Speed_set.chassis_z	=	6000;
			break;
		case 2:
			Robot_cmd.Speed_set.chassis_z	=	4000;
			break;
		case 3:
			Robot_cmd.Speed_set.chassis_z	=	6000;
            break;
		default:
			Robot_cmd.Speed_set.chassis_z	=	3000 ;
			break;
	}
	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Difference_Angle_between_Chassis_Gimbal);
}
/**
  *@note ��Ҫע������ٶ��趨Ϊ���ӵ���ͷǰ�����ٶ�,ʵ�������ƫ�ð�װ��Ҫ����ȡ�����ں����ĵ��������д���
  */
/*
  LF  /   \  RF
        ^
        |
  LB  \   /  RB
*/
void Speed_Set(void)
{
    //��ǰΪY����������ΪX������
    //��̨����ϵ
    Robot_cmd.Speed_set.gambal_x =  ((RC_ctrl->rc.ch[0] ) + (-RC_ctrl->kb.bit.A + RC_ctrl->kb.bit.D) * 660 ) * K_FULL_SPEED_SET;
    Robot_cmd.Speed_set.gambal_y = -((RC_ctrl->rc.ch[1] ) + (-RC_ctrl->kb.bit.S + RC_ctrl->kb.bit.W) * 660 ) * K_FULL_SPEED_SET;
    //��̨����ϵ�ٶȷֽ⵽��������ϵ
    switch(Robot_cmd.Chassis_Mode)
	{
        case NO_FOLLOW:
			NO_FOLLOW_Mode();//���̲�������̨
			break;
        case FOLLOW:
			FOLLOW_Mode();//���̸�����̨
			break;
		case SPIN:
			SPIN_Mode();  //��������
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//����
			break;
	}
    //���ֽ���
//    ��һ�����ĵ����ֵȡ��
//    Robot_cmd.Speed_set.LF_motor = -Robot_cmd.Speed_set.chassis_x - Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * LF_CENTER;
//    Robot_cmd.Speed_set.RF_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RF_CENTER;
//	  Robot_cmd.Speed_set.LB_motor =  Robot_cmd.Speed_set.chassis_x - Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * LB_CENTER;
//    Robot_cmd.Speed_set.RB_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RB_CENTER;
    Robot_cmd.Speed_set.LF_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z * LF_CENTER;
    Robot_cmd.Speed_set.RF_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RF_CENTER;
    Robot_cmd.Speed_set.LB_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z * LB_CENTER;
    Robot_cmd.Speed_set.RB_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RB_CENTER;
}

void State_Set(void)
{
    Robot_cmd.Chassis_State = SPEED;  // Ĭ�ϲ�����
    if ( Robot_cmd.Chassis_Mode == FOLLOW || Robot_cmd.Chassis_Mode == NO_FOLLOW )
    {
        //ҡ���ٶ���������
        if ( abs(Robot_cmd.Speed_set.chassis_x) > SPEED_DEADBAND || abs(Robot_cmd.Speed_set.chassis_y) > SPEED_DEADBAND )
            return;
        //�����Ǽ��ٶȼ�⣬��������ֹ������ң��ͻȻ��������ֱ��ɲ������
        if( abs(ins->E_Accel[X]) >= 0.1f || abs(ins->E_Accel[Y]) >= 0.1f) 
            return;
        
        Robot_cmd.Chassis_State = LOCK_POSITION;
    }      
}


void Chassis_cmd_set(void)
{
    Mode_Set();
    Speed_Set();
    State_Set();
}

void Fire_Set(void)
{
    static bool rc_4_reset = true;
    if (Robot_cmd.Fire_State == On_Fire ||  Robot_cmd.Fire_State == On_Empty)  return;//��ֹ��ϲ����̶���

    switch(RC_ctrl->rc.s2){//ң���Ҳ࿪��
        case RC_SW_DOWN://����״̬Ϊ[��]
            Robot_cmd.Fire_State = NO_Fire_FORCE;//��Ħ����
            break;
        case RC_SW_MID://����״̬Ϊ[��]
            Robot_cmd.Fire_State = Ready_Fire;   //��Ħ����
            break;
        case RC_SW_UP: //����״̬Ϊ[��]
            Robot_cmd.Fire_State = Ready_Fire;   //��Ħ����
            break;
        default:
            Robot_cmd.Fire_State = NO_Fire_FORCE;//��Ħ����
            break;
        }
    //���˻���
    if ((RC_ctrl->rc.ch[4] > -200 &&  RC_ctrl->rc.ch[4] <200)|| RC_ctrl->mouse.press_l == 1)    rc_4_reset = true;
    
    if (rc_4_reset == false)    return;//��ֹ����
    if (Robot_cmd.Fire_State == Ready_Fire && (RC_ctrl->rc.ch[4] >= 650 || RC_ctrl->mouse.press_l == 1))
    {
        rc_4_reset = false;
        Robot_cmd.Fire_State = On_Fire;
    }  
    if (RC_ctrl->rc.ch[4] <= -650)   
    {
        rc_4_reset = false;
        Robot_cmd.Fire_State = On_Empty;
    }
}
#elif BOARD == GIMBAL
#include "imu_task.h"
#include "bsp_dr16.h"
#include "bsp_can.h"
#include "gimbal_config.h"
#include "usbd_cdc_if.h"
//#include "virtual_task.h"
#include "maths.h"
#include "safe_task.h"
#include "bsp_Parabolic_calculation.h"

Gimbal_CMD_t    Gimbal_CMD;
// #if USB_USE_PURPOSE == OU_VISION
// extern gimbal_auto_control_t *auto_control_p;
// #endif
#define CONTROL_SEND_HZ(HZ)\
{\
    static int16_t hz = 0;\
    hz++;\
    if(hz < HZ)   return;\
    hz = 0;\
}

/*�ļ���˽�ܱ���*/
static const INS_t       *ins;

// float calc_can_fire_time(float FIRE_DELAY_MS, float now_in_place_time, float fly_time, gimbal_auto_control_t *auto_control_p);
// uint8_t JudgeCanFire(float now_in_place_time, float *next_in_place_time);

/*
���͸�����
���͸�����
���͸�����
���͸�����
���͸�����
���͸�����
���͸�����
���͸�����
*/


/**
  *@brief ת��ͼ��ң����Ϣ������
  *@note  ͨѶ��bit 48+8N+(29+8N)/4 = 85.25bit 36hz   3069bps
  */
void  Send_TC_to_Chassis(void)
{
    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[3];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x402;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x03;

	CAN2_Tx_Data[0] = (Gimbal_CMD.rc_ctl->kb.key_code) >> 8;
	CAN2_Tx_Data[1] = (Gimbal_CMD.rc_ctl->kb.key_code);
    CAN2_Tx_Data[2] = 0;
	CAN2_Tx_Data[2] |= (Gimbal_CMD.rc_ctl->mouse.press_l);
    CAN2_Tx_Data[2] |= (Gimbal_CMD.rc_ctl->mouse.press_r<<1);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//��һ������ͨ�� CAN ���߷���
}

/**
  *@brief ���͸�����Ħ�����Լ��Ӿ���Ϣ
  *@note ͨѶ��bit 48+8N+(29+8N)/4 = 65.25bit  90hz   
                   48+8N+(29+8N)/4 = 105.25bit 10hz   
                   6,925 bps
  *@parm lock_state ����ʶ��״̬
  *@parm fire_flag  ���鿪��
  *@parm fire_ready Ħ����״̬
  *@parm open_auto  ���鿪��
  */
void Send_Chassis_vision_fire_static(uint8_t lock_flag, uint8_t fire_flag, uint8_t fire_ready, uint8_t open_auto, fp32 Pitch_Angle, uint8_t encoder_lock)
{
    CONTROL_SEND_HZ(10);
    
    static uint8_t send_time = 0;
    static CAN_TxHeaderTypeDef CAN_TxHeader;
    static uint8_t Can_Tx_Data[5];
    Can_Tx_Data[0] = 0;
    Can_Tx_Data[0] |= (lock_flag);
    Can_Tx_Data[0] |= (fire_flag  << 1);
    Can_Tx_Data[0] |= (fire_ready << 2);
    Can_Tx_Data[0] |= (open_auto  << 3);
    Can_Tx_Data[0] |= (encoder_lock << 4);
    
    if (send_time == 9)
    {//ÿʮ�η���һ��P��Ƕ�
        send_time = 0;
        union float_to_uint8_t{
            float f;
            uint8_t u8[4];
        }float_to_uint8;
        
        float_to_uint8.f = Pitch_Angle;
        Can_Tx_Data[1] = float_to_uint8.u8[0];
        Can_Tx_Data[2] = float_to_uint8.u8[1];
        Can_Tx_Data[3] = float_to_uint8.u8[2];
        Can_Tx_Data[4] = float_to_uint8.u8[3];
        
        CAN_TxHeader.DLC = 0x05;
    }
    else
    {
        send_time++;
        CAN_TxHeader.DLC = 0x01;
    }
    static uint32_t send_mail_box;
    CAN_TxHeader.StdId = 0x403;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = 0x05;
    HAL_CAN_AddTxMessage(&hcan2, &CAN_TxHeader, Can_Tx_Data, &send_mail_box);
}


/**
  *@brief ת��Ԥ��ʱ���Լ�pitch�Ჹ��
  *@note  ͨѶ��bit 48+8N+(29+8N)/4 = 135.25bit
  */
// void  Send_Fire_Auto_Delay_And_Pitch_compensate_to_Chassis(void)
// {
//     static fp32 last_pitch_compensate = 0;
//     static fp32 last_auto_fire_delay_time_ms = 0;
//     static uint8_t send_flag = 0;
//     if (last_pitch_compensate != auto_control_p->pitch_compensate || last_auto_fire_delay_time_ms != auto_control_p->auto_fire_delay_time_ms)   
//         send_flag = 10;
//     last_pitch_compensate = auto_control_p->pitch_compensate;
//     last_auto_fire_delay_time_ms = auto_control_p->auto_fire_delay_time_ms;

//     if (send_flag == 0) return;
//     send_flag--;
    
//     CAN_TxHeaderTypeDef CAN2_Txmessage;
//     uint8_t CAN2_Tx_Data[8];
//     uint32_t send_mail_box;
    
//     union float_to_uint8_t{
//         float f;
//         uint8_t u8[4];
//     }float_to_uint8;
//     float_to_uint8.f = auto_control_p->pitch_compensate;
//     CAN2_Tx_Data[0] = float_to_uint8.u8[0];
//     CAN2_Tx_Data[1] = float_to_uint8.u8[1];
//     CAN2_Tx_Data[2] = float_to_uint8.u8[2];
//     CAN2_Tx_Data[3] = float_to_uint8.u8[3];
    
//     float_to_uint8.f = auto_control_p->auto_fire_delay_time_ms;
//     CAN2_Tx_Data[4] = float_to_uint8.u8[0];
//     CAN2_Tx_Data[5] = float_to_uint8.u8[1];
//     CAN2_Tx_Data[6] = float_to_uint8.u8[2];
//     CAN2_Tx_Data[7] = float_to_uint8.u8[3];
    
//     CAN2_Txmessage.StdId 	= 0x405;
// 	CAN2_Txmessage.IDE		= CAN_ID_STD;
// 	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
// 	CAN2_Txmessage.DLC 		= 0x08;
// 	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);			//��һ������ͨ�� CAN ���߷���
// }

/**
  *@brief ת��Ħ����ת��
  *@note  ͨѶ��bit 48+8N+(29+8N)/4 = 135.25bit 10hz
  */
void Send_Fire_Motor_Speed(uint16_t LF, uint16_t RF, uint16_t LB, uint16_t RB)
{
    CONTROL_SEND_HZ(100);

    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[8];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x406;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x08;

	CAN2_Tx_Data[0] = (LF) >> 8;
	CAN2_Tx_Data[1] = (LF);
    CAN2_Tx_Data[2] = (RF) >> 8;
	CAN2_Tx_Data[3] = (RF);
    CAN2_Tx_Data[4] = (LB) >> 8;
	CAN2_Tx_Data[5] = (LB);
    CAN2_Tx_Data[6] = (RB) >> 8;
	CAN2_Tx_Data[7] = (RB);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
}
void Send_Fire_Set_Rpm(void)
{
    CONTROL_SEND_HZ(100);

    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[2];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x407;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x02;

	CAN2_Tx_Data[0] = Gimbal_CMD.Fire_Set_Rpm >> 8;
	CAN2_Tx_Data[1] = Gimbal_CMD.Fire_Set_Rpm;
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
}

#if USB_USE_PURPOSE == NAVIGATION
#include "bsp_navigation.h"
extern Chassis_Speed Speed;
void Send_Chassis_Move(void)
{
    static CAN_TxHeaderTypeDef CAN_TxHeader;
    static uint8_t Can_Tx_Data[8];
    Can_Tx_Data[0] = Speed.x>>8;
    Can_Tx_Data[1] = Speed.x;
    Can_Tx_Data[2] = Speed.y>>8;
    Can_Tx_Data[3] = Speed.y;
    Can_Tx_Data[4] = Speed.z>>8;
    Can_Tx_Data[5] = Speed.z;
    Can_Tx_Data[6] = 0;
    Can_Tx_Data[7] = 0;
    static uint32_t send_mail_box;
    CAN_TxHeader.StdId = 0x77;
	CAN_TxHeader.IDE = CAN_ID_STD;
	CAN_TxHeader.RTR = CAN_RTR_DATA;
	CAN_TxHeader.DLC = 0x08;
    HAL_CAN_AddTxMessage(&hcan2, &CAN_TxHeader, Can_Tx_Data, &send_mail_box);
}
#endif

//dr16����תͼ������ǰ������ң������
uint8_t first_disconnect_clear_rc = 0;
void dr16_online(void){Gimbal_CMD.Control_State = Chassis_RC;first_disconnect_clear_rc = 1;}
void dr16_disconnect(void)
{
    Gimbal_CMD.Control_State = Gimbal_TC; 
    if (first_disconnect_clear_rc == 1)
    {
        first_disconnect_clear_rc = 0;
        Gimbal_CMD.rc_ctl->rc.ch[0] = 0;
        Gimbal_CMD.rc_ctl->rc.ch[1] = 0;
        Gimbal_CMD.rc_ctl->rc.ch[2] = 0;
        Gimbal_CMD.rc_ctl->rc.ch[3] = 0;
        Gimbal_CMD.rc_ctl->rc.ch[4] = 0;
        Gimbal_CMD.rc_ctl->mouse.x = 0;
        Gimbal_CMD.rc_ctl->mouse.y = 0;
        Gimbal_CMD.rc_ctl->mouse.z = 0;
        Gimbal_CMD.rc_ctl->mouse.press_l = 0;
        Gimbal_CMD.rc_ctl->mouse.press_r = 0;
        Gimbal_CMD.rc_ctl->kb.key_code = 0;
    }
}

/*
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
���մ����ص�����
*/


/**
  *@brief ����dr16��Ϣ
  */
safe_task_t *Dr16_Safe;
void Deal_Dr16_form_chassis(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{
    Safe_Task_online_ptr_(Dr16_Safe);
    //                               ��ȡλ��bit3-bit1�ĸ�3λ    ��ȡλ��bit8-bit1�ĵ�8λ
    Gimbal_CMD.rc_ctl->rc.ch[0] = (((Rx_Data[1] & 0x07) << 8) | Rx_Data[0]) - 660;
    //                               ��ȡλ��bit6-bit1�ĸ�6λ    ��ȡλ��bit8-bit4�ĵ�5λ
    Gimbal_CMD.rc_ctl->rc.ch[1] = (((Rx_Data[2] & 0x3F) << 5) | (Rx_Data[1] >> 3)) - 660;
    //                            ��ȡλ��bit8-bit7�ĸ�2λ
    Gimbal_CMD.rc_ctl->rc.s2    = Rx_Data[2] >> 6;
    //                               ��ȡλ��bit3-bit1�ĸ�3λ    ��ȡλ��bit8-bit1�ĵ�8λ
    Gimbal_CMD.rc_ctl->mouse.x  = (((Rx_Data[4] & 0x07) << 8) | Rx_Data[3]) - 660;
    //                               ��ȡλ��bit6-bit1�ĸ�6λ    ��ȡλ��bit8-bit4�ĵ�5λ
    Gimbal_CMD.rc_ctl->mouse.y  = (((Rx_Data[5] & 0x3F) << 5) | (Rx_Data[4] >> 3)) - 660;
    //                                   ��ȡbit7
    Gimbal_CMD.rc_ctl->mouse.press_r = ((Rx_Data[5] & 0x40) >> 6);
    //                                  ��ȡbit8
    uint8_t RC_Mouse_Fire_Set               = (Rx_Data[5]  >> 8);
    //                               ��ȡλ��bit3-bit1�ĸ�3λ      ��ȡλ��bit8-bit1�ĵ�8λ
    Gimbal_CMD.rc_ctl->kb.key_code = ((Rx_Data[7] << 8) | Rx_Data[6]);
    
    //ң��ֵ���ϲ��˺�����������һ��bit
    if (RC_Mouse_Fire_Set == 1)
    {
        Gimbal_CMD.rc_ctl->mouse.press_l = 1;
        Gimbal_CMD.rc_ctl->rc.ch[4] = 0;
    }else
    {
        Gimbal_CMD.rc_ctl->mouse.press_l = 0;
        Gimbal_CMD.rc_ctl->rc.ch[4] = 660;
    }
}

/**
  *@brief ���մ�����ת��ͼ����Ϣ
  *@brief ֻ�е�����dr16����ʱ�Ż�ʹ��ͼ����·
  */
extern uint8_t TCreceive_Flag;
void Gimbal_Deal_TLcontrol(void)
{
    if (Gimbal_CMD.Control_State == Gimbal_TC && TCreceive_Flag == 1)
    {
        memcpy(&Gimbal_CMD.rc_ctl->mouse.x, &Gimbal_CMD.referee_cmd->Remote_control, 10);//����ͼ�����ݵ�ң�����ṹ��
        TCreceive_Flag--;
        Send_TC_to_Chassis();
    }
}

/**
  *@note ����can2 Yaw����ĽǶȲ�ֵ
  */
int16_t zero = 3917;
fp32 Difference_Angle_between_Chassis_Gimbal = 0;
void Deal_Gimbal_Chassis_Angle_between(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_MOTOR_Rx_Data)
{
    int16_t yaw_encoder_value = (int16_t)(DJI_MOTOR_Rx_Data[0] << 8 | DJI_MOTOR_Rx_Data[1]);
    Difference_Angle_between_Chassis_Gimbal = (yaw_encoder_value - zero) * 360.0f / 8192.0f;
    Difference_Angle_between_Chassis_Gimbal = loop_fp32_constrain(Difference_Angle_between_Chassis_Gimbal, -180.0f, 180.0f);

}
/**
  *@brief ����������Ϣ
  */
void Deal_Shoot_Speed_form_chassis(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{
    union float_to_uint8_t{
        float f;
        uint8_t u8[4];
    }float_to_uint8;
    
    float_to_uint8.u8[0] = Rx_Data[0];	
    float_to_uint8.u8[1] = Rx_Data[1];
    float_to_uint8.u8[2] = Rx_Data[2];
    float_to_uint8.u8[3] = Rx_Data[3];
    Gimbal_CMD.referee_cmd->Shoot_Data.bullet_speed = float_to_uint8.f;
}

/**
  *@brief ���յ��̷��͵�P���Ƿ�����������
  */
safe_task_t *Deal_Pitch_Limit_More_Safe;
void Deal_Pitch_Limit_More(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{
    Safe_Task_online_ptr_(Deal_Pitch_Limit_More_Safe);
    if (Rx_Data[0] == 1)    Gimbal_CMD.Pitch_Limit_Max_Flag = 1;
    else                    Gimbal_CMD.Pitch_Limit_Max_Flag = 0;
}
void Deal_Pitch_Limit_More_disconnect(void){Gimbal_CMD.Pitch_Limit_Max_Flag = 0;}
void Deal_Pitch_Limit_More_online(void){}

void CMD_Init(void)
{
    memset(&Gimbal_CMD, 0, sizeof(Gimbal_CMD_t));
    
    Dr16_Safe = Safe_task_add("Dr16_From_Chassis_Safe", 60, dr16_disconnect, dr16_online);
    Deal_Pitch_Limit_More_Safe = Safe_task_add("Deal_Pitch_Limit_More_Safe", 500, Deal_Pitch_Limit_More_disconnect, Deal_Pitch_Limit_More_online);
    //ע������CAN�����жϣ���������ת����dr16��Ϣ
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x01,  Deal_Dr16_form_chassis);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x404, Deal_Shoot_Speed_form_chassis);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x666, Deal_Pitch_Limit_More);
    //ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x205, Deal_Gimbal_Chassis_Angle_between);

    
    // #if USB_USE_PURPOSE == OU_VISION
    //     Gimbal_CMD.AUTO_Pitch_Angle_Set_IMU = get_AutoPitch();
    //     Gimbal_CMD.AUTO_Yaw_Angle_Set_IMU   = get_AutoYaw();
    // #elif USB_USE_PURPOSE == KUN_VISION
    //     Gimbal_CMD.Vision_control = get_auto_control_point();
    // #endif
    Gimbal_CMD.rc_ctl = RC_Get_RC_Pointer();
    Gimbal_CMD.rc_ctl->rc.s2 = RC_SW_DOWN;//��ֹ���̰�δ����ת��
    
    Gimbal_CMD.referee_cmd = Get_referee_Address();
    Gimbal_CMD.Pitch_Lock_type = IMU;
    Gimbal_CMD.Yaw_Lock_type = IMU;
    Gimbal_CMD.Control_State = Chassis_RC;
    Gimbal_CMD.Fire_Ready = 0;
    Gimbal_CMD.Fire_Open = 0;
    Gimbal_CMD.Pitch_Limit_Max_Flag = 0;
    Gimbal_CMD.Pitch_Motor_down_encoder = 0;
    Gimbal_CMD.Fire_Set_Rpm = 5000;
    ins = get_imu_control_point();

}
void Gimbal_Pitch_Set_Encoder(fp32 Set_Encoder)
{
    Gimbal_CMD.Pitch_Lock_type = ENCODER;
    Gimbal_CMD.Pitch_Set_Encoder = Set_Encoder;
}
void Gimbal_Pitch_Set_IMU(fp32 Set_IMU)
{
    Gimbal_CMD.Pitch_Lock_type = IMU;
    Gimbal_CMD.Pitch_Set_IMU = Set_IMU;
}
void Gimbal_Yaw_Set_Encoder(int64_t Set_Encoder)
{
    Gimbal_CMD.Yaw_Lock_type = ENCODER;
    Gimbal_CMD.Yaw_Set_Encoder = Set_Encoder;
}
void Gimbal_Yaw_Set_IMU(fp32 Set_IMU)
{
    Gimbal_CMD.Yaw_Lock_type = IMU;
    Gimbal_CMD.Yaw_Set_IMU = Set_IMU;
}


#define ABS(x) (((x)>0) ? (x) : (-x))
void Gimbal_Angle_Set(fp32 Pitch_Angle_Change, fp32 Yaw_Angle_Change)
{
    static fp32 p = 0;
    static fp32 y = 0;
    if (Gimbal_CMD.Pitch_Lock_type == IMU)
    {
        Gimbal_CMD.Pitch_Set_IMU += Pitch_Angle_Change;
        
        /*
        static uint8_t Key_F_reset = 1;
        if (Gimbal_CMD.referee_cmd->Remote_control.Key_F == 1)
        {
            if (Key_F_reset == 1)   Gimbal_CMD.Pitch_Set_IMU+=0.5;
            Key_F_reset = 0;
        }else
            Key_F_reset = 1;
        
        static uint8_t Key_V_reset = 1;
        if (Gimbal_CMD.referee_cmd->Remote_control.Key_V == 1)
        {
            if (Key_V_reset == 1)   Gimbal_CMD.Pitch_Set_IMU-=0.5;
            Key_V_reset = 0;
        }else
            Key_V_reset = 1;
        */
    }
    if (Gimbal_CMD.Pitch_Lock_type == ENCODER)
    {
        Gimbal_CMD.Pitch_Set_Encoder -= Pitch_Angle_Change*2.0f;
    }
    
    if (Gimbal_CMD.Yaw_Lock_type == IMU)
    {
        Gimbal_CMD.Yaw_Set_IMU += Yaw_Angle_Change;
        /*
        static uint8_t Key_C_reset = 1;
        if (Gimbal_CMD.referee_cmd->Remote_control.Key_C == 1)
        {
            if (Key_C_reset == 1)   Gimbal_CMD.Yaw_Set_IMU+=0.5;
            Key_C_reset = 0;
        }else
            Key_C_reset = 1;
        
        static uint8_t Key_B_reset = 1;
        if (Gimbal_CMD.referee_cmd->Remote_control.Key_B == 1)
        {
            if (Key_B_reset == 1)   Gimbal_CMD.Yaw_Set_IMU-=0.5;
            Key_B_reset = 0;
        }else
            Key_B_reset = 1;
        */
    }

    if (Gimbal_CMD.Yaw_Lock_type == ENCODER)
    {
        y += Yaw_Angle_Change * 20;
        if ( ABS(y) >= 1 )
        {
            Gimbal_CMD.Yaw_Set_Encoder += y;
            y = 0;
        }
    }
    Gimbal_CMD.Yaw_Set_IMU = loop_fp32_constrain(Gimbal_CMD.Yaw_Set_IMU, -180.0f, 180.0f);
}


uint8_t open_x = 0;
void Gimbal_Work_State_Set(void)
{
    static Gimbal_Work_State_e Last_RC_Gimbal_Work_State = MANUAL;
    static Gimbal_Work_State_e RC_Gimbal_Work_State = MANUAL;

    switch(Gimbal_CMD.rc_ctl->rc.s2)//ң���Ҳ࿪��
    {
        case RC_SW_DOWN://����״̬Ϊ[��]
            RC_Gimbal_Work_State =  MANUAL;//�ֶ�����;
            break;
        case RC_SW_MID://����״̬Ϊ[��]
            RC_Gimbal_Work_State =  MANUAL;//�ֶ�����;
            break;
        case RC_SW_UP: //����״̬Ϊ[��]
            RC_Gimbal_Work_State =  LONG_SHOOT;//����;
            break;
        default:
            RC_Gimbal_Work_State =  MANUAL;//�ֶ�����;
            break;
    }
    if (Last_RC_Gimbal_Work_State != RC_Gimbal_Work_State)
    {
        Gimbal_CMD.Gimbal_Work_State = RC_Gimbal_Work_State;
    }
    Last_RC_Gimbal_Work_State = RC_Gimbal_Work_State;
    
    // if (Gimbal_CMD.rc_ctl->mouse.press_r == 1)  
    // {
    //     if (auto_control_p->armor_num != 0) Gimbal_CMD.Gimbal_Work_State = AUTO_ATTACK;//����Ҽ�ǿ�ƿ�������
    //     else                                Gimbal_CMD.Gimbal_Work_State = LONG_SHOOT;//������ʶ����Ϊ�����
    // }
    // else
    //     Gimbal_CMD.Gimbal_Work_State = RC_Gimbal_Work_State;//�ع�ԭ��ģʽ
}
fp32 now_time_ms;

float debug_RC_PITCH_SPEED = 0.0003f;
float debug_RC_YAW_SPEED = 0.0003f;


void Deal_Fire_Set_Rpm(void)
{
    static uint8_t X_set = 1;
    if (Gimbal_CMD.rc_ctl->kb.bit.N)
    {
        if (X_set)
        {
            X_set = 0;
            Gimbal_CMD.Fire_Set_Rpm+=100;
        }
    }
    else
        X_set = 1;
        
//    Gimbal_CMD.Fire_Set_Rpm = 0;
    if (Gimbal_CMD.Fire_Set_Rpm > 5100) Gimbal_CMD.Fire_Set_Rpm = 4500;

}
void Gimbal_CMD_Set(fp32 Pitch_Up_Angle_Limit, fp32 Pitch_Down_Angle_Limit, fp32 Pitch_Angle, fp32 Pitch_encoder_Angle)
{

    Gimbal_Work_State_Set();
    uint8_t manual_fire_delay_flag = 0;
    
    if (Gimbal_CMD.rc_ctl->rc.ch[0] == 48)  Gimbal_CMD.rc_ctl->rc.ch[0] = 0;//Dr16��ӳ�磿���δ�����
    
    static Gimbal_Work_State_e Last_Gimbal_Work_State = MANUAL;
    switch(Gimbal_CMD.Gimbal_Work_State)
    {//��̨�ٿ�״̬
        case MANUAL://�ֶ�����,��̨�Ƕ��ɲ����ָı�
            Last_Gimbal_Work_State = MANUAL;
            Gimbal_Angle_Set(-Gimbal_CMD.rc_ctl->mouse.y * MOUSE_PITCH_SPEED, -Gimbal_CMD.rc_ctl->mouse.x * MOUSE_YAW_SPEED);
            Gimbal_Angle_Set(Gimbal_CMD.rc_ctl->rc.ch[1] * debug_RC_PITCH_SPEED, -Gimbal_CMD.rc_ctl->rc.ch[0] * debug_RC_YAW_SPEED);
            break;
        case LONG_SHOOT://��ǰ��վ���ֲ٣����ٲ���
          
            Last_Gimbal_Work_State = LONG_SHOOT;
            Gimbal_Angle_Set(-Gimbal_CMD.rc_ctl->mouse.y * 0.0002f, -Gimbal_CMD.rc_ctl->mouse.x * 0.0002f);
            Gimbal_Angle_Set(Gimbal_CMD.rc_ctl->rc.ch[1] *  0.00001f , -Gimbal_CMD.rc_ctl->rc.ch[0] * 0.00001f);
            break;
       
        // case AUTO_ATTACK://����
        //     Gimbal_Angle_Set(-Gimbal_CMD.rc_ctl->mouse.y * MOUSE_PITCH_SPEED, -Gimbal_CMD.rc_ctl->mouse.x * MOUSE_YAW_SPEED);
        //     Gimbal_Angle_Set(Gimbal_CMD.rc_ctl->rc.ch[1] * RC_PITCH_SPEED, -Gimbal_CMD.rc_ctl->rc.ch[0] * RC_YAW_SPEED);
        //     //�Ӿ�ʶ�𵽣�����Ƕ�
        //     if (auto_control_p->armor_num != 0)
        //     {
        //         Gimbal_CMD.Pitch_Set_IMU = *Gimbal_CMD.AUTO_Pitch_Angle_Set_IMU;
        //         Gimbal_CMD.Yaw_Set_IMU   = *Gimbal_CMD.AUTO_Yaw_Angle_Set_IMU; 
        //     }
        //     Last_Gimbal_Work_State = AUTO_ATTACK;
        //     break;
        default:
            Gimbal_Angle_Set(-Gimbal_CMD.rc_ctl->mouse.y * MOUSE_PITCH_SPEED, -Gimbal_CMD.rc_ctl->mouse.x * MOUSE_YAW_SPEED);
            Gimbal_Angle_Set(Gimbal_CMD.rc_ctl->rc.ch[1] * RC_PITCH_SPEED, -Gimbal_CMD.rc_ctl->rc.ch[0] * RC_YAW_SPEED);
    }
    
    //��������λ
    if (Gimbal_CMD.Pitch_Lock_type == IMU)  
        value_limit(Gimbal_CMD.Pitch_Set_IMU, Pitch_Down_Angle_Limit, Pitch_Up_Angle_Limit);
    if (Gimbal_CMD.Pitch_Lock_type == ENCODER)
    #ifdef Using_MI_Motor_Init
        value_limit(Gimbal_CMD.Pitch_Set_Encoder, Gimbal_CMD.Pitch_Motor_down_encoder-PITCH_ANGLE_RANGE, Gimbal_CMD.Pitch_Motor_down_encoder);//�������̧����ֵ��С
    #else
        value_limit(Gimbal_CMD.Pitch_Set_Encoder, PITCH_MOTOR_UP_MIN_ACTUAL_ANGLE, PITCH_MOTOR_DOWN_MAX_ACTUAL_ANGLE);//�������̧����ֵ��С
    #endif
    //����������Ϣ������
    // Send_Chassis_vision_fire_static(auto_control_p->armor_num != 0? 1 : 0,//�����Ƿ�ʶ��
    //                                 Gimbal_CMD.Gimbal_Work_State == AUTO_Delay_Fire? manual_fire_delay_flag : auto_control_p->fire_flag,//�Զ������͵Ŀ���ָ��
    //                                 Gimbal_CMD.Fire_Open,//�Ƿ���Ħ����
    //                                (Gimbal_CMD.Gimbal_Work_State == MANUAL || Gimbal_CMD.Gimbal_Work_State == LONG_SHOOT)?0:1,//�ִ�
    //                                 Gimbal_CMD.Pitch_Lock_type == ENCODER? Pitch_encoder_Angle:Pitch_Angle,
    //                                 Gimbal_CMD.Pitch_Lock_type == ENCODER? 1:0);
    Send_Chassis_vision_fire_static(0,//�����Ƿ�ʶ��
                                    Gimbal_CMD.Gimbal_Work_State == manual_fire_delay_flag,//�Զ������͵Ŀ���ָ��
                                    Gimbal_CMD.Fire_Open,//�Ƿ���Ħ����
                                   (Gimbal_CMD.Gimbal_Work_State == MANUAL || Gimbal_CMD.Gimbal_Work_State == LONG_SHOOT)?0:1,//�ִ�
                                    Gimbal_CMD.Pitch_Lock_type == ENCODER? Pitch_encoder_Angle:Pitch_Angle,
                                    Gimbal_CMD.Pitch_Lock_type == ENCODER? 1:0);
}
    
void Gimbal_Fire_State_Set(void)
{
    static uint8_t first_down = 1;
    if (Gimbal_CMD.rc_ctl->kb.bit.F == 1 || Gimbal_CMD.rc_ctl->rc.s2 == RC_SW_UP || Gimbal_CMD.rc_ctl->rc.s2 == RC_SW_MID)
        Gimbal_CMD.Fire_Ready = 1;
    
    if (Gimbal_CMD.rc_ctl->rc.s2 == RC_SW_DOWN && first_down == 1)//�״β���ʱ�ر�Ħ���֣����Է�ֹ���ش�������£���F����Ħ���ֱ����˸��ǹر�
    {
        first_down = 0;
        Gimbal_CMD.Fire_Ready = 0;
    }
    
    if (Gimbal_CMD.rc_ctl->rc.s2 != RC_SW_DOWN)
        first_down = 1;
}

const Gimbal_CMD_t* Get_Gimbal_CMD_point(void)
{
    // while(Get_Virtual_task_init_falg() == 0)
    //     vTaskDelay(1);//�ȴ��Ӿ������ʼ��
    
    static uint8_t init_flag = 0;
    if (init_flag++ == 0)   CMD_Init();
    return &Gimbal_CMD;
}

void Gimbal_Pitch_Lock(Lock_type_e Lock_type)
{
    Gimbal_CMD.Pitch_Lock_type = Lock_type;
}
void Gimbal_Yaw_Lock(Lock_type_e Lock_type)
{
    Gimbal_CMD.Yaw_Lock_type = Lock_type;
}
void Set_Pitch_Motor_down_encoder(fp32 Pitch_Motor_down_encoder)
{
    Gimbal_CMD.Pitch_Motor_down_encoder = Pitch_Motor_down_encoder;
}

//���������Ӿ������õ�
fp32* get_Gimbal_pitch_point()
{
    return &Gimbal_CMD.Pitch_Set_IMU;
}
fp32* get_Gimbal_yaw_point()
{
    return &Gimbal_CMD.Yaw_Set_IMU;
}
Gimbal_Work_State_e* get_gimbal_behaviour_point()
{
    return &Gimbal_CMD.Gimbal_Work_State;
}
uint8_t get_fire_ready(void)
{
    return Gimbal_CMD.Fire_Ready;
}
uint8_t get_rc_fire_control(void)
{
    if (Gimbal_CMD.rc_ctl->rc.ch[4] == 660) return 1;
    if (Gimbal_CMD.rc_ctl->mouse.press_l == 1)  return 1;
    return 0;
}
Gimbal_Work_State_e get_open_auto_control(void)
{
    return Gimbal_CMD.Gimbal_Work_State == AUTO_ATTACK? 1 : 0;
}
Gimbal_Work_State_e get_open_auto_delay_control(void)
{
    return Gimbal_CMD.Gimbal_Work_State == AUTO_Delay_Fire? 1 : 0;
}

/**
  *@brief ������ʵķ���ʱ���뷢���ӳ�
  *@parm  ��λms
  */
#define ROTATE_TIME_MS 833 //ms 833.333
// float calc_can_fire_time(float FIRE_DELAY_MS, float now_in_place_time, float fly_time, gimbal_auto_control_t *auto_control_p)
// {
//     if (fly_time == 0)  return 0;
//     int64_t increat_time = ROTATE_TIME_MS;
//     float next_in_place_time = now_in_place_time + ROTATE_TIME_MS;//��һ��ת�����ĵ�ʱ���
    
//     while ((FIRE_DELAY_MS + fly_time) > increat_time)
//     {   //����ʱ�䣫����ʱ�䳬����һ��װ�װ�ת��λ��ʱ�䣬���Ӧ��Ԥ�����¿�װ�װ嵽λʱ��
//         next_in_place_time += ROTATE_TIME_MS;
//         increat_time += ROTATE_TIME_MS;
//     }
//     float   can_fire_time = next_in_place_time-FIRE_DELAY_MS-fly_time;
// //    auto_control_p->fly_time_ms = fly_time;
// //    auto_control_p->auto_fire_delay_time_ms = can_fire_time - now_in_place_time;
//     return  can_fire_time;
// }
/**
  *@brief �ж��Ƿ���Ͽ���ʵ��
  *@note  �����������Ԥ����һ�ε�λʱ��
  *@reval 0 ����ʱ��δ�� | 1 ����ʱ���ѵ� | 2 ����ʱ���ѹ� | 3 ���δ���
  */
// float fla = 20;
// uint8_t JudgeCanFire(float now_in_place_time, float *next_in_place_time)
// {
//     if (now_in_place_time == 0) return 3;
//     if (*next_in_place_time == 0) return 3;
//     if (now_in_place_time > *next_in_place_time)
//     {
//         *next_in_place_time = 0;
//         return 2;
//     }        
//     if (abs(now_in_place_time - *next_in_place_time) < fla)
//     {
//         *next_in_place_time = 0;
//         return 1;    
//     }                                                       
//     return 0;
// }

void Set_Fire_Open_State(uint8_t State)
{
    Gimbal_CMD.Fire_Open = State;
}
#endif
