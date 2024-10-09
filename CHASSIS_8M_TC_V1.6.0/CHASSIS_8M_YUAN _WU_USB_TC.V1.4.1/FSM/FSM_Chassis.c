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
extern New_Chassis_Task_t New_Chassis_Data;//引用底盘数据结构体


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
 *函数名称：Mode_Deal
 *函数参数：void
 *函数返回：void
 *函数功能：底盘模式处理
 *特别说明：FSM状态机处理，共三个模式
 */
void Mode_Deal()
{
    Chassis_Mode_t last_Chassis_Mode;
    static Chassis_Mode_t RC_Chassis_Mode = NO_FOLLOW;//手柄模式数据，用于判断有无状态改变
    if(New_Chassis_Data.Necessary_information.RC->rc.s2 != RC_SW_DOWN)
    {   
        //按键01乘上660后模拟摇杆满值，向右为横轴正向
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
			default://啥模式都没给，防止出错，不动模式
                RC_Chassis_Mode = NO_FORCE;
				break;
		}
        //不加改变检测而直接改变Chassis_Mode的后果是，用键盘改变的模式数据会在松开按键后被手柄的模式数据覆盖
        if(last_Chassis_Mode != RC_Chassis_Mode)
		{
			New_Chassis_Data.Set_information.Mode = RC_Chassis_Mode;
		}
        
        /*以上为手柄数据，以下为键盘数据*/
        
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
        New_Chassis_Data.Set_information.Mode = NO_FORCE;//不动模式
    }
}

/*
 *函数名称：State_Deal
 *函数参数：void
 *函数返回：void
 *函数功能：底盘状态处理
 *特别说明：锁车或运动
 */
void State_Deal()
{   
    Chassis_State_t     last_Chassis_State = New_Chassis_Data.Set_information.State;
    New_Chassis_Data.Set_information.State= SPEED;  // 默认不锁车
    if ( New_Chassis_Data.Set_information.Mode == FOLLOW || New_Chassis_Data.Set_information.Mode == NO_FOLLOW )
    {
        //摇杆速度死区限制
        if ( abs(New_Chassis_Data.Set_information.plane_x_speed_set) > SPEED_DEADBAND || abs(New_Chassis_Data.Set_information.plane_y_speed_set) > SPEED_DEADBAND )
            return;
        //陀螺仪加速度检测，这样做防止高速下遥控突然回正导致直接刹车锁死
        if( abs(FSM_Chassis.INS->E_Accel[X]) >= 0.1f || abs(FSM_Chassis.INS->E_Accel[Y]) >= 0.1f) 
            return;
        
        New_Chassis_Data.Set_information.State = LOCK_POSITION;
        if (last_Chassis_State != New_Chassis_Data.Set_information.State)    New_Chassis_Data.Motor_information.Motor_encoder_clear_flag = 1;
    }          
}

/*
 *函数名称：Chassis_X_Y_Speed_Calc
 *函数参数：void
 *函数返回：void
 *函数功能：底盘速度计算
 *特别说明：xy轴基于云台坐标系
 */
void Chassis_X_Y_Speed_Calc()
{
    switch(New_Chassis_Data.Set_information.Mode)
	{
        case NO_FOLLOW:
			NO_FOLLOW_Mode();//底盘不跟随云台
			break;
        case FOLLOW:
			FOLLOW_Mode();//底盘跟随云台
			break;
		case SPIN:
			SPIN_Mode();//底盘自旋
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//不动
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
    //根据英雄等级选择自旋速度
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
    //Z方向速度由 LF 分解 多余的Y方向速度忽略
    New_Chassis_Data.Set_information.LF_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y方向速度由 LB RB 合成
    New_Chassis_Data.Set_information.LB_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X方向速度由 LF LB 合成
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.LB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}

void Miss_LF_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z方向速度由 RF 分解 多余的Y方向速度忽略
    New_Chassis_Data.Set_information.RF_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y方向速度由 LB RB 合成
    New_Chassis_Data.Set_information.LB_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X方向速度由 RF RB 合成
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    
    /* 待验证 不开小陀螺应该没毛病
    //补偿 RF 为提供 Speed_z 所需 X 方向转动速度 而 额外产生的 Y 方向速度
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
    //Z方向速度由 LB 分解 多余的Y方向速度忽略
    New_Chassis_Data.Set_information.LB_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y方向速度由 LF RF 合成
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X方向速度由 LF LB 合成
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.LB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}

void Miss_LB_Motion_Calc()
{
    New_Chassis_Data.Set_information.LF_speed_set = 0;
    New_Chassis_Data.Set_information.RF_speed_set = 0;
	New_Chassis_Data.Set_information.LB_speed_set = 0;
    New_Chassis_Data.Set_information.RB_speed_set = 0;
    //Z方向速度由 RB 分解 多余的Y方向速度忽略
    New_Chassis_Data.Set_information.RB_speed_set = 2 * New_Chassis_Data.Set_information.plane_z_speed_set;
    //Y方向速度由 LF RF 合成
    New_Chassis_Data.Set_information.LF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    New_Chassis_Data.Set_information.RF_speed_set += 2 * New_Chassis_Data.Set_information.plane_y_speed_set;
    //X方向速度由 RF RB 合成
    New_Chassis_Data.Set_information.RF_speed_set -= 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
    New_Chassis_Data.Set_information.RB_speed_set += 2 * New_Chassis_Data.Set_information.plane_x_speed_set;
}
