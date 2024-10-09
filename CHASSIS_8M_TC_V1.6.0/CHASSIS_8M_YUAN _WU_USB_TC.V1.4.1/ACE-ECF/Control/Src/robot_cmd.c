#include "robot_cmd.h"
#include "bsp_dr16.h"
#include "maths.h"
#include "pid.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "bsp_Motor_Encoder.h"
#include <stdbool.h>
#include "safe_task.h"
#include "DJI_Motor.h"

#define CONTROL_SEND_HZ(HZ)\
{\
    static int16_t hz = 0;\
    hz++;\
    if(hz < HZ)   return;\
    hz = 0;\
}
#define abs(x) ((x) > (0) ? (x) : (-(x)))
Robot_cmd_t Robot_cmd;
DJIMotor_object_t *YAW_MOTOR;
/*���ļ���˽�ܱ���*/
static uint8_t init_flag = 0;
/*�����˶�����ĸ�������*/
static const REFEREE_t   *referee_cmd;
static const INS_t       *ins;
static pid_parameter_t   chassis_follow_gambal_speed;
acceleration_control_type_t chassis_x_acc_control;//����
acceleration_control_type_t chassis_y_acc_control;

acceleration_control_type_t chassis_x_moderate_control;
acceleration_control_type_t chassis_y_moderate_control;
/**
  *@brief ���ص����˶���������ָ��
  */
Robot_cmd_t* get_Robot_cmd_point(void)
{
    if (init_flag++ == 0) CMD_Init();
    return &Robot_cmd;
}

/**
  *@brief ������̨ͼ��ң��
  *@note  ����Dr16����ʱ����
  */
void Deal_TC_from_Gimbal(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{    
    if (Robot_cmd.Control_State == Chassis_RC)   return;
    Robot_cmd.RC->kb.key_code = ((DJI_Motor_Rx_Data[0] << 8) | DJI_Motor_Rx_Data[1]);
    Robot_cmd.RC->mouse.press_l = DJI_Motor_Rx_Data[2] & 1;//������һλ
    Robot_cmd.RC->mouse.press_r = (DJI_Motor_Rx_Data[2]>>1) & 1;//�����ڶ�λ
}

/**
  *@brief ������̨�Ӿ���Ϣ
  *@note  �ֳ�4�ֽ�
  */
uint8_t Get_fire_ready(void){return Robot_cmd.Gimbal_Info.AUTO_Fire_Flag;}
uint8_t Get_gimbal_open_auto(void){return Robot_cmd.Gimbal_Info.Open_Auto;}
uint8_t Get_AUTO_LOCK_Flag(void){return Robot_cmd.Gimbal_Info.AUTO_LOCK_Flag;}//0 ������ 1 �Ӿ��� 2 TOF��
void Deal_Vision_Fire_Pitch_Angle_from_Gimbal(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{    
    Robot_cmd.Gimbal_Info.AUTO_LOCK_Flag = Rx_Data[0] & 1;//���1λ���Ӿ��Ƿ�ʶ��
    Robot_cmd.Gimbal_Info.AUTO_Fire_Flag = (Rx_Data[0]>>1) & 1;//���2λ���Զ������͵Ŀ���ָ��
    Robot_cmd.Gimbal_Info.Fire_ready = (Rx_Data[0]>>2) & 1;//���3λ��Ħ���ֿ�����
    Robot_cmd.Gimbal_Info.Open_Auto = (Rx_Data[0]>>3) & 1;//���4λ���ִ�
    
    Robot_cmd.Gimbal_Info.Pitch_Lock_Encoder_Flag = (Rx_Data[0]>>4) & 1;
    if (CAN_Rxmessage->DLC == 5)
    {
        union float_to_uint8_t{
            float f;
            uint8_t u8[4];
        }float_to_uint8;
        
        float_to_uint8.u8[0] = Rx_Data[1];	
        float_to_uint8.u8[1] = Rx_Data[2];
        float_to_uint8.u8[2] = Rx_Data[3];
        float_to_uint8.u8[3] = Rx_Data[4];
        Robot_cmd.Gimbal_Info.Pitch_angle = float_to_uint8.f;
    }
}
/**
  *@brief ����Ԥ��ʱ���Լ�pitch�Ჹ��
  *@note  ͨѶ�ֳ�8�ֽ� 1hz
  */
fp32 Get_Pitch_compensate(void){return Robot_cmd.Gimbal_Info.Pitch_compensate;}
fp32 Get_Fire_auto_delay(void){return Robot_cmd.Gimbal_Info.Fire_auto_delay;}

void Deal_Fire_Auto_Delay_And_Pitch_compensate_from_Gimbal(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{    
    union float_to_uint8_t{
        float f;
        uint8_t u8[4];
    }float_to_uint8;
    
    float_to_uint8.u8[0] = Rx_Data[0];	
    float_to_uint8.u8[1] = Rx_Data[1];
    float_to_uint8.u8[2] = Rx_Data[2];
    float_to_uint8.u8[3] = Rx_Data[3];
    Robot_cmd.Gimbal_Info.Pitch_compensate = float_to_uint8.f;
    
    float_to_uint8.u8[0] = Rx_Data[4];	
    float_to_uint8.u8[1] = Rx_Data[5];
    float_to_uint8.u8[2] = Rx_Data[6];
    float_to_uint8.u8[3] = Rx_Data[7];
    Robot_cmd.Gimbal_Info.Fire_auto_delay = float_to_uint8.f;
    
}
/**
  *@brief ����TOF���
  *@note  ͨѶ�ֳ�8�ֽ� 200
  */
fp32 TOF_Distance;
fp32 Get_TOF_Distance(void){return TOF_Distance;}
void Deal_TOF_Distance(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{    
    union float_to_uint8_t{
        float f;
        uint8_t u8[4];
    }float_to_uint8;
    
    float_to_uint8.u8[0] = Rx_Data[0];	
    float_to_uint8.u8[1] = Rx_Data[1];
    float_to_uint8.u8[2] = Rx_Data[2];
    float_to_uint8.u8[3] = Rx_Data[3];
    TOF_Distance = float_to_uint8.f;  
}

void Deal_Fire_Motor_Speed(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{    
    Robot_cmd.Gimbal_Info.LU_FireMoterSpeed = Rx_Data[0] << 8 | Rx_Data[1];	
    Robot_cmd.Gimbal_Info.RU_FireMoterSpeed = Rx_Data[2] << 8 | Rx_Data[3];	
    Robot_cmd.Gimbal_Info.LD_FireMoterSpeed = Rx_Data[4] << 8 | Rx_Data[5];	
    Robot_cmd.Gimbal_Info.RD_FireMoterSpeed = Rx_Data[6] << 8 | Rx_Data[7];	 
}

void Deal_Fire_Motor_Set_Rpm(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *Rx_Data)
{
    Robot_cmd.Gimbal_Info.FireMotorSetRpm = Rx_Data[0] << 8 | Rx_Data[1];
}

uint8_t send_to_gimbal_Dr16 = 0;
uint8_t first_disconnect_clear_rc = 1;
void Dr16_DisconnetCallback(void)
{
    Robot_cmd.Control_State = Gimbal_TC;
    send_to_gimbal_Dr16 = 0;
    if (first_disconnect_clear_rc == 1)
    {//����s1s2����,����ԭ��ң�������ݣ�����ͼ������
        first_disconnect_clear_rc = 0;
        Robot_cmd.RC->rc.ch[0] = 0;
        Robot_cmd.RC->rc.ch[1] = 0;
        Robot_cmd.RC->rc.ch[2] = 0;
        Robot_cmd.RC->rc.ch[3] = 0;
        Robot_cmd.RC->rc.ch[4] = 0;
        Robot_cmd.RC->mouse.x = 0;
        Robot_cmd.RC->mouse.y = 0;
        Robot_cmd.RC->mouse.z = 0;
        Robot_cmd.RC->mouse.press_l = 0;
        Robot_cmd.RC->mouse.press_r = 0;
        Robot_cmd.RC->kb.key_code = 0;
    }
}
void Dr16_OnlineCallback(void)
{
    send_to_gimbal_Dr16 = 2;
    first_disconnect_clear_rc = 1;
    Robot_cmd.Control_State = Chassis_RC;
}
void Dr16_RxCallback(void){Safe_Task_online_name_("dr16");}
/**
  *@brief ת��dr16����̨��
  *@note  ����Dr16����ʱת�� ��bit 48+8N+(29+8N)/4 = 135.25bit 72hz 
  */
uint8_t Tx_Data_SendTime_And_IsOpenFllow = 0;
void  Send_Dr16_to_Gimbal(void)
{
    if (Robot_cmd.Control_State == Gimbal_TC)    return;
    if (send_to_gimbal_Dr16 == 0)   return;

    static uint8_t send_time = 1;
    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[8];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x01;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x08;

    uint8_t RC_Mouse_Fire_Set = 0;
    if (Robot_cmd.RC->rc.ch[4] >= 550)  RC_Mouse_Fire_Set = 1;
    if (Robot_cmd.RC->mouse.press_l == 1) RC_Mouse_Fire_Set = 1;
    
    //ͨ��ֵ��Χ-660~660������ƫ����660��Ϊ0~1320��2^11 = 2,048����Ҫ��11bit���
    CAN2_Tx_Data[0] = (Robot_cmd.RC->rc.ch[0]+660);
    CAN2_Tx_Data[1] = (Robot_cmd.RC->rc.ch[0]+660) >> 8 | (Robot_cmd.RC->rc.ch[1]+660) << 3;
    CAN2_Tx_Data[2] = (Robot_cmd.RC->rc.ch[1]+660) >> 5 | (Robot_cmd.RC->rc.s2) << 6;
    CAN2_Tx_Data[3] = (Robot_cmd.RC->mouse.x+660);
    CAN2_Tx_Data[4] = (Robot_cmd.RC->mouse.x+660) >> 8 | (Robot_cmd.RC->mouse.y+660) << 3;
    CAN2_Tx_Data[5] = (Robot_cmd.RC->mouse.y+660) >> 5 | (Robot_cmd.RC->mouse.press_r << 6) | RC_Mouse_Fire_Set << 7;//ң��ֵ���ϲ��˺�����������һ��bit��������
    CAN2_Tx_Data[6] = (Robot_cmd.RC->kb.key_code);
    CAN2_Tx_Data[7] = (Robot_cmd.RC->kb.key_code) >> 8;
    
    HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
    send_to_gimbal_Dr16--;
}

void Send_Dr16_to_Gimbal_Ext(void)
{
    if (Robot_cmd.Control_State == Gimbal_TC)    return;

    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[15];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.ExtId = 0x01;
	CAN2_Txmessage.IDE	 = CAN_ID_EXT;
	CAN2_Txmessage.RTR	 = CAN_RTR_DATA;
	CAN2_Txmessage.DLC   = 0xF;
    
    CAN2_Tx_Data[0] = (Robot_cmd.RC->rc.ch[0]) >> 8;
    CAN2_Tx_Data[1] = (Robot_cmd.RC->rc.ch[0]);
    CAN2_Tx_Data[2] = (Robot_cmd.RC->rc.ch[1]) >> 8;
    CAN2_Tx_Data[3] = (Robot_cmd.RC->rc.ch[1]);
    CAN2_Tx_Data[4] = (Robot_cmd.RC->rc.ch[4]) >> 8;
    CAN2_Tx_Data[5] = (Robot_cmd.RC->rc.ch[4]);
    CAN2_Tx_Data[6] = (Robot_cmd.RC->mouse.x) >> 8;
    CAN2_Tx_Data[7] = (Robot_cmd.RC->mouse.x);
    CAN2_Tx_Data[8] = (Robot_cmd.RC->mouse.y) >> 8;
    CAN2_Tx_Data[9] = (Robot_cmd.RC->mouse.y);
    CAN2_Tx_Data[10] = (Robot_cmd.RC->kb.key_code) >> 8;
    CAN2_Tx_Data[11] = (Robot_cmd.RC->kb.key_code);
    CAN2_Tx_Data[12] = (Robot_cmd.RC->rc.s2);	
    CAN2_Tx_Data[13] = (Robot_cmd.RC->mouse.press_r);
    CAN2_Tx_Data[14] = Robot_cmd.Chassis_Mode == FOLLOW? 1 : 0;	
	
    HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
}
/**
  *@brief ת�����ٸ���̨��
  *@note  ��bit 48+8N+(29+8N)/4 = 95.25bit
  */
void  Send_Shoot_Speed_to_Gimbal(void)
{
    static fp32 last_shoot_speed = 0;
    static uint8_t send_flag = 0;
    if (last_shoot_speed != referee_cmd->Shoot_Data.bullet_speed)   send_flag = 10;
    last_shoot_speed = referee_cmd->Shoot_Data.bullet_speed;
    
    if (send_flag == 0) return;
    send_flag--;
//    CONTROL_SEND_HZ(10);//0.01s ��� 
    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[4];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x404;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x04;

    union float_to_uint8_t{
        float f;
        uint8_t u8[4];
    }float_to_uint8;

    float_to_uint8.f = referee_cmd->Shoot_Data.bullet_speed;
    CAN2_Tx_Data[0] = float_to_uint8.u8[0];	
    CAN2_Tx_Data[1] = float_to_uint8.u8[1];
    CAN2_Tx_Data[2] = float_to_uint8.u8[2];
    CAN2_Tx_Data[3] = float_to_uint8.u8[3];
    HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
}
/**
  *@brief �����Ƿ��������PITCH��������
  */
void  Send_Pitch_Limit_More(void)
{
    CONTROL_SEND_HZ(200);
    
    CAN_TxHeaderTypeDef CAN2_Txmessage;
    uint8_t CAN2_Tx_Data[1];
    uint32_t send_mail_box;
    
    CAN2_Txmessage.StdId 	= 0x666;
	CAN2_Txmessage.IDE		= CAN_ID_STD;
	CAN2_Txmessage.RTR	  = CAN_RTR_DATA;
	CAN2_Txmessage.DLC 		= 0x01;
    
    if (abs(Robot_cmd.Difference_Angle_between_Chassis_Gimbal) < 20 && Robot_cmd.Chassis_Mode == FOLLOW)
        CAN2_Tx_Data[0] = 1;
    else
        CAN2_Tx_Data[0] = 0;
    
    HAL_CAN_AddTxMessage(&hcan2, &CAN2_Txmessage, CAN2_Tx_Data, &send_mail_box);
}
/**
  *@brief ��ȡ�����˶���������ĸ�������ָ��
  */
fp32 Z_Fllow_Deadzone = 5.0f;
void CMD_Init(void)
{
    memset(&Robot_cmd.Gimbal_Info, 0 ,sizeof(Gimbal_Info_t));
    
    Robot_cmd.Power_Mode = 0;
    Robot_cmd.increat_time = 0;
    Robot_cmd.RC = RC_Get_RC_Pointer();
    Robot_cmd.Chassis_Mode = NO_FOLLOW;
    Robot_cmd.Control_State = Chassis_RC;

    referee_cmd = Get_referee_Address();
    ins = get_imu_control_point();
 
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x402, Deal_TC_from_Gimbal);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x403, Deal_Vision_Fire_Pitch_Angle_from_Gimbal);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x405, Deal_Fire_Auto_Delay_And_Pitch_compensate_from_Gimbal);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x406, Deal_Fire_Motor_Speed);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0x407, Deal_Fire_Motor_Set_Rpm);

    
    YAW_MOTOR = DJIMotor_Init(2,1,true,GM6020,1000);
    PidInit(&chassis_follow_gambal_speed, 5.5, 0 ,0, Output_Limit | Integral_Limit | Deadzone);
    PidInitMode(&chassis_follow_gambal_speed, Output_Limit, 660, 0);
	PidInitMode(&chassis_follow_gambal_speed, Integral_Limit, 300, 0);
	PidInitMode(&chassis_follow_gambal_speed, Deadzone, Z_Fllow_Deadzone, 0);
    //���ٶ�����
    motion_acceleration_control_init(&chassis_x_acc_control);
    motion_acceleration_control_init(&chassis_y_acc_control);
    
    Dr16RxCallBack_Init(Dr16_RxCallback);//bug��
    
    //��ӵ��߼��
    Safe_task_add("dr16",60,Dr16_DisconnetCallback, Dr16_OnlineCallback);
}

int16_t z = 400;
void Mode_Set(void)
{
    static Chassis_Mode_t last_RC_Chassis_Mode = NO_FOLLOW;
    static Chassis_Mode_t RC_Chassis_Mode   = NO_FOLLOW;//ģʽ���ݣ������ж�����״̬�ı�
    
    switch(Robot_cmd.RC->rc.s1)//ң����࿪��
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
        default://ɶģʽ��û������ֹ�������ж�ģʽ
            RC_Chassis_Mode = NO_FOLLOW;
            break;
    }
    //��⵽�ı��������Robot_cmd.Chassis_Mode�������ü������õ�ģʽ�������ɿ��������ֱ���ԭ��ģʽ���ݸ���
    if(last_RC_Chassis_Mode != RC_Chassis_Mode)
    {
        Robot_cmd.Chassis_Mode = RC_Chassis_Mode;
        if (Robot_cmd.Chassis_Mode == SPIN) z = -z;
    }
    last_RC_Chassis_Mode = RC_Chassis_Mode;
    
    static bool Q_reset = true;     
    if (Robot_cmd.RC->kb.bit.Q) { 
        Robot_cmd.Chassis_Mode = SPIN;
        if (Q_reset)
        {
            z = -z;
            Q_reset = false;
        }
    }
    else   Q_reset = true;
    
    if (Robot_cmd.RC->kb.bit.E == 1)  Robot_cmd.Chassis_Mode = FOLLOW;
    if (Robot_cmd.RC->kb.bit.R == 1)  Robot_cmd.Chassis_Mode = NO_FOLLOW;
   
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

	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
}

static void FOLLOW_Mode()
{
    fp32 Z_close = Robot_cmd.Difference_Angle_between_Chassis_Gimbal;
    if (Z_close > 90)
    {
        Z_close = -180 + Z_close;
    }
    else if (Z_close < -90)
    {
        Z_close = 180 + Z_close;
    }
        
	Robot_cmd.Speed_set.chassis_z = -PidCalculate(&chassis_follow_gambal_speed, 0, Z_close);//˫ͷ��
    
    Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
}




static void SPIN_Mode()
{
    //�����������z�ٶ�
    static uint16_t last_Robot_HP = 0;
    static uint64_t hight_spin_time = 0;
    fp32 speed_spin = 1;
    if (referee_cmd->Robot_Status.current_HP < last_Robot_HP)
    {
        hight_spin_time = 5000;
    }
    
    if (hight_spin_time > 0)
    {
        speed_spin = 1.1f;
        hight_spin_time--;
    }
    else  
        speed_spin = 1;
    
    last_Robot_HP = referee_cmd->Robot_Status.current_HP;

    Robot_cmd.Speed_set.chassis_z	=	z * speed_spin;

	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(Robot_cmd.Difference_Angle_between_Chassis_Gimbal);
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
    if (Robot_cmd.RC->kb.bit.SHIFT == 1)
    {
        Robot_cmd.Power_Mode = 1;
        chassis_follow_gambal_speed.deadband = 0.25f*Z_Fllow_Deadzone;
    }        
    else 
    {
        Robot_cmd.Power_Mode = 0;
        chassis_follow_gambal_speed.deadband = Z_Fllow_Deadzone;
    }        
    
    //��ǰΪY����������ΪX������
    //��̨����ϵ
    Robot_cmd.Speed_set.gambal_x =  ((Robot_cmd.RC->rc.ch[2] ) + (-Robot_cmd.RC->kb.bit.A + Robot_cmd.RC->kb.bit.D) * 660 ) ;
    Robot_cmd.Speed_set.gambal_y = -((Robot_cmd.RC->rc.ch[3] ) + (-Robot_cmd.RC->kb.bit.S + Robot_cmd.RC->kb.bit.W) * 660 ) ;

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
        case RESPIN:
			SPIN_Mode();  //��������
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//����
			break;
	}
//���ֽ���
//    /*
//        / \
//        \ /
//    */
//    
    Robot_cmd.Speed_set.LF_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z ;
    Robot_cmd.Speed_set.RF_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z ;
    Robot_cmd.Speed_set.LB_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z ;
    Robot_cmd.Speed_set.RB_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z ;
    
    Robot_cmd.Speed_set.LF_motor *= 10.0f + 5.0f*Robot_cmd.Power_Mode;
    Robot_cmd.Speed_set.RF_motor *= 10.0f + 5.0f*Robot_cmd.Power_Mode;
    Robot_cmd.Speed_set.LB_motor *= 10.0f + 5.0f*Robot_cmd.Power_Mode;
    Robot_cmd.Speed_set.RB_motor *= 10.0f + 5.0f*Robot_cmd.Power_Mode;
    
    //�������ٶ�9000����������ȱ������ŵ�9000����ֹ�˶�ʧ��
    float divisor = 1.0f;
    uint16_t max_speed_1 = max_abs(Robot_cmd.Speed_set.LF_motor, Robot_cmd.Speed_set.RF_motor);
    uint16_t max_speed_2 = max_abs(Robot_cmd.Speed_set.LB_motor, Robot_cmd.Speed_set.RB_motor);
    uint16_t max_speed = max_abs(max_speed_1, max_speed_2);
    if (max_speed > 9000)
    {
        divisor = 9000.0f/((float)max_speed);
        
        Robot_cmd.Speed_set.LF_motor *=  divisor ;
        Robot_cmd.Speed_set.RF_motor *=  divisor ;
        Robot_cmd.Speed_set.LB_motor *=  divisor ;
        Robot_cmd.Speed_set.RB_motor *=  divisor ;
    }
}

#define ABS_FP(num) ((num>0?num:-num))
void State_Set(void)
{
    Robot_cmd.Chassis_State = SPEED;  // Ĭ�ϲ�����
    
    if ((ABS_FP(Robot_cmd.Speed_set.chassis_x) == 0) &&  (ABS_FP(Robot_cmd.Speed_set.chassis_y) == 0) && (ABS_FP(Robot_cmd.Speed_set.chassis_z) == 0))
    {
        Robot_cmd.Chassis_State = LOCK_POSITION;// ����
    }  
}

int16_t zero = -2740;
void Chassis_cmd_set(void)
{
    //������̨���̲��
    int16_t yaw_encoder_value = YAW_MOTOR->Motor_Information.position;
    Robot_cmd.Difference_Angle_between_Chassis_Gimbal = (yaw_encoder_value - zero) * 360.0f / 8192.0f;
    Robot_cmd.Difference_Angle_between_Chassis_Gimbal = loop_fp32_constrain(Robot_cmd.Difference_Angle_between_Chassis_Gimbal, -180.0f, 180.0f);
    
    Mode_Set();
    Speed_Set();
    State_Set();
}

/**
  *@brief ������״̬�� ��������״̬  ���� ���� ���� �˵�  
  */
void Fire_Set(void)
{
    static bool rc_4_reset = true;     
    if (Robot_cmd.RC->rc.ch[4] <200 && Robot_cmd.RC->mouse.press_l == 0)    rc_4_reset = true;//�������ּ��
    
    if (Robot_cmd.Gimbal_Info.Fire_ready == 0)                 Robot_cmd.Fire_State = NO_Fire_FORCE;//��̨Ħ����δ����������
    else if(Robot_cmd.Fire_State != On_Fire)    Robot_cmd.Fire_State = Ready_Fire;   //��Ħ���֣�����
    
    if (Robot_cmd.RC->rc.ch[4] <-200 || Robot_cmd.RC->kb.bit.M == 1)   Robot_cmd.Fire_State = On_Empty;//�˵�
    
    if (Robot_cmd.Fire_State == NO_Fire_FORCE || Robot_cmd.Fire_State == On_Empty)  return;//δ����ת���ɽ���״̬����
        
//    if (Robot_cmd.Fire_State == On_Fire)  return;//�ϳ�ʱ�������ʵ�����޻���
    
    if (Robot_cmd.Gimbal_Info.Open_Auto == 1)//����״̬�¶���һ��Ҫ���Ӿ����������
    {
         if ( (Robot_cmd.RC->rc.ch[4] >= 550 || Robot_cmd.RC->mouse.press_l == 1) && rc_4_reset && Robot_cmd.Gimbal_Info.AUTO_Fire_Flag == 1)
        {
            Robot_cmd.Fire_State = On_Fire;//����
            rc_4_reset = false;//�ȴ��������֣���ֹ����
            Robot_cmd.increat_time ++;//��һ�ŵ�
        }
    }
    else//����
    {
        if ( (Robot_cmd.RC->rc.ch[4] >= 550 || Robot_cmd.RC->mouse.press_l == 1) && rc_4_reset)
        {
            Robot_cmd.Fire_State = On_Fire;
            rc_4_reset = false;
            Robot_cmd.increat_time ++;
        }
    }
}


Control_State_e Get_Chassis_Control_State(void)
{
    return Robot_cmd.Control_State;
}
fp32 Get_Gimbal_Pitch_Angle(void)
{
    return Robot_cmd.Gimbal_Info.Pitch_angle;
}
uint8_t Get_Gimbal_Pitch_Mode(void)
{
    return Robot_cmd.Gimbal_Info.Pitch_Lock_Encoder_Flag;
}
uint16_t Get_Gimbal_Fire_Motor_Speed(uint8_t idx)
{
    switch (idx)
    {
        case 0:
            return Robot_cmd.Gimbal_Info.LU_FireMoterSpeed;
        case 1:
            return Robot_cmd.Gimbal_Info.RU_FireMoterSpeed;
        case 2:
            return Robot_cmd.Gimbal_Info.LD_FireMoterSpeed;
        case 3:
            return Robot_cmd.Gimbal_Info.RD_FireMoterSpeed;
        default:
            return 0;
    }
}
fp32 Get_dif_angle(void)
{
    return Robot_cmd.Difference_Angle_between_Chassis_Gimbal;
}
int16_t Get_Fire_Motor_Set_Rpm(void)
{
    return Robot_cmd.Gimbal_Info.FireMotorSetRpm;
}