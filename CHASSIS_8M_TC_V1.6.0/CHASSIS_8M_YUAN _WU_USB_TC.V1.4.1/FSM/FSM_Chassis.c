#include "chassis_task.h"
#include "FSM_Chassis.h"
#include "pid.h"
#include "maths.h"

static void NO_FOLLOW_Mode(void);
static void NO_FORCE_Mode(void);
static void FOLLOW_Mode(void);
static void SPIN_Mode(void);

pid_parameter_t z_speed;
FSM_Chassis_t FSM_Chassis;
extern New_Chassis_Task_t New_Chassis_Data;//���õ������ݽṹ��


#define Mode_same_or_not(last_mode,change_mode)     (last_mode != change_mode)?   change_mode:last_mode
#define Encoder_clear_or_not(last_mode,change_mode) (last_mode != change_mode)?           1:0

void FSM_Chassis_Init(void)
{
	FSM_Chassis.RC       = RC_Get_RC_Pointer();
	FSM_Chassis.INS      = get_imu_control_point();
	FSM_Chassis.referee  = Get_referee_Address();
	FSM_Chassis.Difference_Angle_between_Chassis_Gimbal = 0;
	PidInit(&z_speed, 40, 0.1 ,0, Output_Limit | Integral_Limit | Deadzone);
	PidInitMode(&z_speed, Output_Limit, 10000, 0);
	PidInitMode(&z_speed, Integral_Limit, 800, 0);
	PidInitMode(&z_speed, Deadzone, 5, 0);
}

FSM_Chassis_t *Get_FSM_Chassis_Address(void)
{
	return &FSM_Chassis;
}

/*
 *�������ƣ�Mode_Deal
 *����������void
 *�������أ�void
 *�������ܣ�����ģʽ����
 *�ر�˵����FSM״̬������������ģʽ
 */
void Mode_Deal()
{
    Chassis_Mode_t last_Chassis_Mode;
    static Chassis_Mode_t RC_Chassis_Mode = NO_FOLLOW;//�ֱ�ģʽ���ݣ������ж�����״̬�ı�
    if(New_Chassis_Data.Necessary_information.RC->rc.s2 != RC_SW_DOWN)
    {   
        //����01����660��ģ��ҡ����ֵ������Ϊ��������
        New_Chassis_Data.Set_information.plane_x_speed_set =  ((New_Chassis_Data.Necessary_information.RC->rc.ch[0] ) + (-New_Chassis_Data.Necessary_information.RC->kb.bit.A + New_Chassis_Data.Necessary_information.RC->kb.bit.D) * 660 ) * K_FULL_SPEED_SET;
		New_Chassis_Data.Set_information.plane_y_speed_set = -((New_Chassis_Data.Necessary_information.RC->rc.ch[1] ) + (-New_Chassis_Data.Necessary_information.RC->kb.bit.S + New_Chassis_Data.Necessary_information.RC->kb.bit.W) * 660 ) * K_FULL_SPEED_SET;
        
        last_Chassis_Mode = RC_Chassis_Mode;
        switch(New_Chassis_Data.Necessary_information.RC->rc.s1)
		{
			case RC_SW_DOWN:
				RC_Chassis_Mode = NO_FOLLOW;
				break;
			case RC_SW_MID:
				RC_Chassis_Mode = FOLLOW;
				break;
			case RC_SW_UP:
				RC_Chassis_Mode = SPIN;
                break;
			default://ɶģʽ��û������ֹ��������ģʽ
                RC_Chassis_Mode = NO_FORCE;
				break;
		}
        //���Ӹı����ֱ�Ӹı�Chassis_Mode�ĺ���ǣ��ü��̸ı��ģʽ���ݻ����ɿ��������ֱ���ģʽ���ݸ���
        if(last_Chassis_Mode != RC_Chassis_Mode)
		{
			New_Chassis_Data.Set_information.Mode = RC_Chassis_Mode;
		}
        
        /*����Ϊ�ֱ����ݣ�����Ϊ��������*/
        
        if(FSM_Chassis.RC->kb.bit.Q == 1)
		{
			New_Chassis_Data.Set_information.Mode = FOLLOW;
		}
		if(FSM_Chassis.RC->kb.bit.E == 1)
		{
			New_Chassis_Data.Set_information.Mode = SPIN;
		}
		if(FSM_Chassis.RC->kb.bit.R == 1)
		{
			New_Chassis_Data.Set_information.Mode = NO_FOLLOW;
		}                
    }
    else
    {
        New_Chassis_Data.Set_information.Mode = NO_FORCE;//����ģʽ
    }
}

/*
 *�������ƣ�State_Deal
 *����������void
 *�������أ�void
 *�������ܣ�����״̬����
 *�ر�˵�����������˶�
 */
void State_Deal()
{   
    Chassis_State_t     last_Chassis_State = New_Chassis_Data.Set_information.State;
    New_Chassis_Data.Set_information.State= SPEED;  // Ĭ�ϲ�����
    if ( New_Chassis_Data.Set_information.Mode == FOLLOW || New_Chassis_Data.Set_information.Mode == NO_FOLLOW )
    {
        //ҡ���ٶ���������
        if ( abs(New_Chassis_Data.Set_information.plane_x_speed_set) > SPEED_DEADBAND || abs(New_Chassis_Data.Set_information.plane_y_speed_set) > SPEED_DEADBAND )
            return;
        //�����Ǽ��ٶȼ�⣬��������ֹ������ң��ͻȻ��������ֱ��ɲ������
        if( abs(FSM_Chassis.INS->E_Accel[X]) >= 0.1f || abs(FSM_Chassis.INS->E_Accel[Y]) >= 0.1f) 
            return;
        
        New_Chassis_Data.Set_information.State = LOCK_POSITION;
        if (last_Chassis_State != New_Chassis_Data.Set_information.State)    New_Chassis_Data.Motor_information.Motor_encoder_clear_flag = 1;
    }          
}

/*
 *�������ƣ�Chassis_X_Y_Speed_Calc
 *����������void
 *�������أ�void
 *�������ܣ������ٶȼ���
 *�ر�˵����xy�������̨����ϵ
 */
void Chassis_X_Y_Speed_Calc()
{
    switch(New_Chassis_Data.Set_information.Mode)
	{
        case NO_FOLLOW:
			NO_FOLLOW_Mode();//���̲�������̨
			break;
        case FOLLOW:
			FOLLOW_Mode();//���̸�����̨
			break;
		case SPIN:
			SPIN_Mode();//��������
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//����
			break;
	}
}
static void NO_FORCE_Mode()
{
	New_Chassis_Data.Set_information.plane_x_speed_set	=	0;
    New_Chassis_Data.Set_information.plane_y_speed_set	=	0;
	New_Chassis_Data.Set_information.plane_z_speed_set	=	0;
}

static void NO_FOLLOW_Mode()
{
	New_Chassis_Data.Set_information.plane_z_speed_set = 0;

	float x_speed_set = New_Chassis_Data.Set_information.plane_x_speed_set;
	float y_speed_set = New_Chassis_Data.Set_information.plane_y_speed_set;
    
	New_Chassis_Data.Set_information.plane_x_speed_set = - y_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal)
                                                         - x_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
    New_Chassis_Data.Set_information.plane_y_speed_set =   y_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal) 
                                                         - x_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
}

static void FOLLOW_Mode()
{
	New_Chassis_Data.Set_information.plane_z_speed_set = PidCalculate(&z_speed, 0, New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
    
    float x_speed_set = New_Chassis_Data.Set_information.plane_x_speed_set;
	float y_speed_set = New_Chassis_Data.Set_information.plane_y_speed_set;
	New_Chassis_Data.Set_information.plane_x_speed_set = - y_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal)
                                                         - x_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
    New_Chassis_Data.Set_information.plane_y_speed_set =   y_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal) 
                                                         - x_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
}

static void SPIN_Mode()
{
    //����Ӣ�۵ȼ�ѡ�������ٶ�
	switch(New_Chassis_Data.Necessary_information.referee->Robot_Status.robot_level)
	{
		case 1:
			New_Chassis_Data.Set_information.plane_z_speed_set	=	6000;
			break;
		case 2:
			New_Chassis_Data.Set_information.plane_z_speed_set	=	4000;
			break;
		case 3:
			New_Chassis_Data.Set_information.plane_z_speed_set	=	6000;
            break;
		default:
			New_Chassis_Data.Set_information.plane_z_speed_set	=	3000 ;
			break;
	}
	float x_speed_set = New_Chassis_Data.Set_information.plane_x_speed_set;
	float y_speed_set = New_Chassis_Data.Set_information.plane_y_speed_set;
	New_Chassis_Data.Set_information.plane_x_speed_set = - y_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal)
                                                         - x_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
    New_Chassis_Data.Set_information.plane_y_speed_set =   y_speed_set * cos_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal) 
                                                         - x_speed_set * sin_calculate(New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal);
}

void ALL_OK_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = -New_Chassis_Data.Set_information.plane_x_speed_set - New_Chassis_Data.Set_information.plane_y_speed_set - New_Chassis_Data.Set_information.plane_z_speed_set;
    New_Chassis_Data.Set_information.RF_speed_set = -New_Chassis_Data.Set_information.plane_x_speed_set + New_Chassis_Data.Set_information.plane_y_speed_set - New_Chassis_Data.Set_information.plane_z_speed_set;
	New_Chassis_Data.Set_information.LB_speed_set =  New_Chassis_Data.Set_information.plane_x_speed_set - New_Chassis_Data.Set_information.plane_y_speed_set - New_Chassis_Data.Set_information.plane_z_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set =  New_Chassis_Data.Set_information.plane_x_speed_set + New_Chassis_Data.Set_information.plane_y_speed_set - New_Chassis_Data.Set_information.plane_z_speed_set;
}

void Miss_RF_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z�����ٶ��� LF �ֽ� �����Y�����ٶȺ���
    New_Chassis_Data.Set_information.LF_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y�����ٶ��� LB RB �ϳ�
    New_Chassis_Data.Set_information.LB_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X�����ٶ��� LF LB �ϳ�
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.LB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}

void Miss_LF_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z�����ٶ��� RF �ֽ� �����Y�����ٶȺ���
    New_Chassis_Data.Set_information.RF_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y�����ٶ��� LB RB �ϳ�
    New_Chassis_Data.Set_information.LB_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X�����ٶ��� RF RB �ϳ�
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    
    /* ����֤ ����С����Ӧ��ûë��
    //���� RF Ϊ�ṩ Speed_z ���� X ����ת���ٶ� �� ��������� Y �����ٶ�
    FSM_Chassis.LB_speed_set += FSM_Chassis.plane_z_speed_set;
    FSM_Chassis.RB_speed_set -= FSM_Chassis.plane_z_speed_set;
    */
}

void Miss_RB_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z�����ٶ��� LB �ֽ� �����Y�����ٶȺ���
    New_Chassis_Data.Set_information.LB_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y�����ٶ��� LF RF �ϳ�
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X�����ٶ��� LF LB �ϳ�
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.LB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}

void Miss_LB_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z�����ٶ��� RB �ֽ� �����Y�����ٶȺ���
    New_Chassis_Data.Set_information.RB_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y�����ٶ��� LF RF �ϳ�
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X�����ٶ��� RF RB �ϳ�
    New_Chassis_Data.Set_information.RF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}
