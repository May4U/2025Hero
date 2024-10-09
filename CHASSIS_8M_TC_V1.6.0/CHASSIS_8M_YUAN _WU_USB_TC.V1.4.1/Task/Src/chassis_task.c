#include "Chassis_task.h"
#include "imu_task.h"
#include "safe_task.h"
#include "stm32f4xx.h"
#define ABS(x) ((x) > (0) ? (x) : (-(x)))

#define CONTROL_SEND_HZ(HZ)\
{\
    static int16_t hz = 0;\
    hz++;\
    if(hz < HZ)   return;\
    hz = 0;\
}

Chassis_t Chassis;
static uint16_t time = 0;
uint8_t chassis_power_limit = 55;//Ѫ������ģʽ��һ����������
fp32 Cap_Input_Power = 55;

static void Chassis_Init(void);
static void True_power_limit(uint8_t Chassis_Power_Limit);
static void Calc_SuperPower_energy(void);
static void Read_SuperPower_Vout(void);
static void Chassis_Motor_Set_PID(void);
static void PID_Calc(void);
static void Chassis_Send_current(void);
static uint8_t Calc_SuperPower_Out_power_limit(void);

uint8_t cmd = 0x02;
void Chassis_Movement(void const *argument)
{
	Chassis_Init();
	vTaskDelay(500);
	while(1)
	{
        taskENTER_CRITICAL(); 
        /*--------------------------------�ٽ���--------------------------------*/
        
        if (Chassis.referee->Robot_Status.chassis_power_limit != 0)//���µ��̹�������
            chassis_power_limit = Chassis.referee->Robot_Status.chassis_power_limit;
                    
        time++;
        if (time % 10 == 0)//100hz ��ȡ�����ѹ
            Read_SuperPower_Vout();
        if (time > 100)//10hz ����ʣ�໺�������趨�������빦��
        {
            Cap_Input_Power = chassis_power_limit + (Chassis.referee->Power_Heat.chassis_power_buffer - 10.0f)/10.0f;//������������
            pm01_power_set((Cap_Input_Power)*100,0x00);
            pm01_cmd_send( cmd, 0x01);
            time = 0;
        }
        

        
        Chassis_cmd_set();
        Chassis_Motor_Set_PID();
        PID_Calc();//�������е���ĵ�������
        Calc_SuperPower_energy();//������������
        Chassis.Max_Power = Calc_SuperPower_Out_power_limit();
        True_power_limit(Chassis.Max_Power);


        Chassis_Send_current();
        Send_Dr16_to_Gimbal();//135.25bit * 72hz = 9738 bps 
        Send_Shoot_Speed_to_Gimbal();
        /*--------------------------------�ٽ���--------------------------------*/
        taskEXIT_CRITICAL(); 
        vTaskDelay(1);//������ʱ//vTaskDelay(2);	
    }
}


/*
void RF_DisconnetCallback(void){;}//Safeʧ��ʱ����
void RF_OnlineCallback(void){;}//Safeι��ʱ����
void RF_CanRxCallback(void){Safe_Task_online_name_("RF_Motor");};//Can����ʱ����

void RB_DisconnetCallback(void){;}
void RB_OnlineCallback(void){;}
void RB_CanRxCallback(void){Safe_Task_online_name_("RB_Motor");};

void LB_DisconnetCallback(void){;}
void LB_OnlineCallback(void){;}
void LB_CanRxCallback(void){Safe_Task_online_name_("LB_Motor");};

void LF_DisconnetCallback(void){;}
void LF_OnlineCallback(void){;}
void LF_CanRxCallback(void){Safe_Task_online_name_("LF_Motor");};
*/

/**
  *@brif �����ѹ��Ϣ���ջص�����
  *@note ��Chassis_Init�е���ECF_CAN_Rx_Callback_Registerע��
  */
void SuperPower_V_responed(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *can_rx_data)
{
    uint16_t m_tmp; 
    if (CAN_Rxmessage->StdId != 0x612)  return;
    
    m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	
    Chassis.pm01_od->p_out = m_tmp;

    m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	
    Chassis.pm01_od->v_out = m_tmp;	

    m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	
    Chassis.pm01_od->i_out = m_tmp;
}
/**
  *@brif ���̳�ʼ��
  */
void Chassis_Init(void)
{
    Chassis.Robot_cmd = get_Robot_cmd_point();
    Chassis.referee = Get_referee_Address();
    Chassis.pm01_od = get_superpower_point();
    //�����ʼ��
    Chassis.FIRE_MOTOR = DJIMotor_Init(1,5,false,M3508,178);
    Chassis.FIRE_MOTOR->Motor_encoder.gear_Ratio = 51;
    Chassis.RF_MOTOR = DJIMotor_Init(1,4,true,M3508,178);
    Chassis.RB_MOTOR = DJIMotor_Init(1,2,true,M3508,178);
    Chassis.LB_MOTOR = DJIMotor_Init(1,3,false,M3508,178);
    Chassis.LF_MOTOR = DJIMotor_Init(1,1,false,M3508,178);
    
    //PID������ʼ��
    PidInit(&Chassis.FIRE_MOTOR->Speed_PID, 1.0f,	0,	0.0f,	Output_Limit);
    PidInit(&Chassis.LF_MOTOR->Speed_PID, 15.0f,	LF_Speed_PID_I,	LF_Speed_PID_D,	Output_Limit | StepIn);
	PidInit(&Chassis.RF_MOTOR->Speed_PID, 15.0f,	RF_Speed_PID_I,	RF_Speed_PID_D,	Output_Limit | StepIn);	
	PidInit(&Chassis.LB_MOTOR->Speed_PID, 15.0f,	LB_Speed_PID_I,	LB_Speed_PID_D,	Output_Limit | StepIn);		
	PidInit(&Chassis.RB_MOTOR->Speed_PID, 15.0f,	RB_Speed_PID_I,	RB_Speed_PID_D,	Output_Limit | StepIn);

    PidInitMode(&Chassis.FIRE_MOTOR->Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.LF_MOTOR->Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.RF_MOTOR->Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.LB_MOTOR->Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
	PidInitMode(&Chassis.RB_MOTOR->Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    
	PidInitMode(&Chassis.LF_MOTOR->Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.RF_MOTOR->Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.LB_MOTOR->Speed_PID, StepIn, StepIn_Data, 0);
	PidInitMode(&Chassis.RB_MOTOR->Speed_PID, StepIn, StepIn_Data, 0);
    
    PidInit(&Chassis.FIRE_MOTOR->Position_PID, 0.8f, 0.3f, 100.0f, Integral_Limit | Output_Limit);
	PidInit(&Chassis.LF_MOTOR->Position_PID, LF_Position_PID_P, LF_Position_PID_I, LF_Position_PID_D, NONE);
	PidInit(&Chassis.RF_MOTOR->Position_PID, RF_Position_PID_P, RF_Position_PID_I, RF_Position_PID_D, NONE);
	PidInit(&Chassis.LB_MOTOR->Position_PID, LB_Position_PID_P, LB_Position_PID_I, LB_Position_PID_D, NONE);
	PidInit(&Chassis.RB_MOTOR->Position_PID, RB_Position_PID_P, RB_Position_PID_I, RB_Position_PID_D, NONE);
    
    PidInitMode(&Chassis.FIRE_MOTOR->Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
	PidInitMode(&Chassis.FIRE_MOTOR->Position_PID,Integral_Limit, 150 , 0);
    
    ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Stdid, 0x612, SuperPower_V_responed);
    pm01_cmd_send( 0x02, 0x01);//
    pm01_voltage_set(2400,0x0001);//24V
    pm01_current_set(750, 0x0001 );//7.5A
    pm01_power_set((50)*100-50,0x0001);//49.5w
}

/**
  *@brif ���̵������PID�⻷����Ŀ��ֵ
  */
#define abs(x) ((x) > (0) ? (x) : (-(x)))
void Chassis_Motor_Set_PID(void)
{
    static uint8_t Move = 1;//�����Ƿ��������ƶ�״̬
    static uint8_t Set_Lock = 1;//����ͣ�º��״��趨����λ��
    if (Chassis.Robot_cmd->Chassis_State != LOCK_POSITION)
        Move = 1;

    if(Chassis.Robot_cmd->Chassis_State == LOCK_POSITION &&  (abs(Chassis.LF_MOTOR->Motor_Information.speed < 10) && abs(Chassis.RF_MOTOR->Motor_Information.speed < 10) && abs(Chassis.LB_MOTOR->Motor_Information.speed < 10) && abs(Chassis.RB_MOTOR->Motor_Information.speed < 10)))
        Move = 0;
    
    if (Move)
    {
        Set_Lock = 1;
        Chassis.RF_MOTOR->Using_PID = Speed_PID;
        Chassis.RB_MOTOR->Using_PID = Speed_PID;
        Chassis.LB_MOTOR->Using_PID = Speed_PID;
        Chassis.LF_MOTOR->Using_PID = Speed_PID;    
        
        DJIMotor_Set_val( Chassis.RF_MOTOR, Chassis.Robot_cmd->Speed_set.RF_motor);
        DJIMotor_Set_val( Chassis.RB_MOTOR, Chassis.Robot_cmd->Speed_set.RB_motor);
        DJIMotor_Set_val( Chassis.LB_MOTOR, Chassis.Robot_cmd->Speed_set.LB_motor);
        DJIMotor_Set_val( Chassis.LF_MOTOR, Chassis.Robot_cmd->Speed_set.LF_motor);
    }
    else
    {
        if (Set_Lock)
        {
            Set_Lock = 0;
            Chassis.RF_MOTOR->Using_PID = Position_Speed_PID;
            Chassis.RB_MOTOR->Using_PID = Position_Speed_PID;
            Chassis.LB_MOTOR->Using_PID = Position_Speed_PID;
            Chassis.LF_MOTOR->Using_PID = Position_Speed_PID;
            
            DJIMotor_Set_val( Chassis.RF_MOTOR, -Chassis.RF_MOTOR->Motor_encoder.Encode_Record_Val);
            DJIMotor_Set_val( Chassis.RB_MOTOR, -Chassis.RB_MOTOR->Motor_encoder.Encode_Record_Val);
            DJIMotor_Set_val( Chassis.LB_MOTOR, Chassis.LB_MOTOR->Motor_encoder.Encode_Record_Val);
            DJIMotor_Set_val( Chassis.LF_MOTOR, Chassis.LF_MOTOR->Motor_encoder.Encode_Record_Val);
            pid_clear(&Chassis.RF_MOTOR->Position_PID);
            pid_clear(&Chassis.RB_MOTOR->Position_PID);
            pid_clear(&Chassis.LB_MOTOR->Position_PID);
            pid_clear(&Chassis.LF_MOTOR->Position_PID);
        }
    }
}
/**
  *@brif ���̵��PID����
  *@note ���ڲ����̵����Fire_task�з��䷢񲣬���Ƿ������﷢��
  */
void PID_Calc(void)
{
Chassis.FIRE_MOTOR->current_input = motor_position_speed_control(&Chassis.FIRE_MOTOR->Speed_PID, &Chassis.FIRE_MOTOR->Position_PID, Chassis.FIRE_MOTOR->set_position, Chassis.FIRE_MOTOR->Motor_encoder.Encode_Record_Val ,Chassis.FIRE_MOTOR->Motor_Information.speed);	
    if (Chassis.FIRE_MOTOR->Using_PID == No_Current)
        Chassis.FIRE_MOTOR->current_input = 0;
    if (Chassis.Robot_cmd->Fire_State == On_Empty)  
        Chassis.FIRE_MOTOR->current_input = 1000;
    
    if (Chassis.RF_MOTOR->Using_PID == SPEED)
    {

        DJIMotor_PID_Calc(Chassis.RF_MOTOR);
        DJIMotor_PID_Calc(Chassis.RB_MOTOR);
        DJIMotor_PID_Calc(Chassis.LB_MOTOR);
        DJIMotor_PID_Calc(Chassis.LF_MOTOR);
    }
    else
    {
        Chassis.RF_MOTOR->current_input = motor_position_speed_control(&Chassis.RF_MOTOR->Speed_PID, &Chassis.RF_MOTOR->Position_PID, Chassis.RF_MOTOR->set_position, Chassis.RF_MOTOR->Motor_encoder.Encode_Record_Val, Chassis.RF_MOTOR->Motor_Information.speed);
        Chassis.RB_MOTOR->current_input = motor_position_speed_control(&Chassis.RB_MOTOR->Speed_PID, &Chassis.RB_MOTOR->Position_PID, Chassis.RB_MOTOR->set_position, Chassis.RB_MOTOR->Motor_encoder.Encode_Record_Val, Chassis.RB_MOTOR->Motor_Information.speed);
        Chassis.LB_MOTOR->current_input = motor_position_speed_control(&Chassis.LB_MOTOR->Speed_PID, &Chassis.LB_MOTOR->Position_PID, Chassis.LB_MOTOR->set_position, Chassis.LB_MOTOR->Motor_encoder.Encode_Record_Val, Chassis.LB_MOTOR->Motor_Information.speed);
        Chassis.LF_MOTOR->current_input = motor_position_speed_control(&Chassis.LF_MOTOR->Speed_PID, &Chassis.LF_MOTOR->Position_PID, Chassis.LF_MOTOR->set_position, Chassis.LF_MOTOR->Motor_encoder.Encode_Record_Val, Chassis.LF_MOTOR->Motor_Information.speed);
    }
//    if (Chassis.Robot_cmd->Chassis_Mode == NO_FORCE)
//    {
//        Chassis.RF_MOTOR->current_input = 0;
//        Chassis.RB_MOTOR->current_input = 0;
//        Chassis.LF_MOTOR->current_input = 0;
//        Chassis.LB_MOTOR->current_input = 0;
//    }

}

/**
  *@brif �򻬼��
  *@note ͨ�������Ǻ�����ļ��ٶȲ���
  */
void Slip_detection(void)
{
//    static float filted_accel[2];
//    static Recursive_ave_filter_type_t filter_IMU;
//    static first_order_filter_type_t ditong_filter_IMU;
//    static Recursive_ave_filter_type_t filter_Encoder;
//    static first_order_filter_type_t ditong_filter_Encoder; 
//    static float Not_Slip_Acc = 0;
//    
//    filted_accel[0] = Recursive_ave_filter(&filter_IMU, ins->Accel[Y], 100);
//	filted_accel[0] = first_order_filter(&ditong_filter_IMU,filted_accel[0]);
//    
//	filted_accel[1] = Recursive_ave_filter(&filter_Encoder, Chassis.RF_MOTOR.Motor_encoder->Acc_linear_speed * 0.7071067811865475f, 100);//�ֽ����ǰ���ٶ�
//	filted_accel[1] = first_order_filter(&ditong_filter_Encoder,	filted_accel[1]);
//    #define Slip_num 1.5f
//    if (abs(filted_accel[1] - filted_accel[0]) > Slip_num) {//��
//        
//    }
//    else {
//        Not_Slip_Acc = Chassis.RF_MOTOR.Motor_encoder->Acc_linear_speed;
//    }
}

static void CAN_Send_3508_5_8_current( int16_t LF_Motor, int16_t RF_Motor, int16_t LB_Motor, int16_t RB_Motor)
{
    CAN_TxHeaderTypeDef CANTxmessage;
    uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;
	CANTxmessage.StdId 	= 0x1FF;
	CANTxmessage.IDE		= CAN_ID_STD;
	CANTxmessage.RTR	  = CAN_RTR_DATA;
	CANTxmessage.DLC 		= 0x08;
	
	CAN_Send_Data[0] = LF_Motor >> 8;
	CAN_Send_Data[1] = LF_Motor;
	CAN_Send_Data[2] = RF_Motor >> 8;
	CAN_Send_Data[3] = RF_Motor;
	CAN_Send_Data[4] = LB_Motor >> 8;
	CAN_Send_Data[5] = LB_Motor;
	CAN_Send_Data[6] = RB_Motor >> 8;
	CAN_Send_Data[7] = RB_Motor;

	HAL_CAN_AddTxMessage(&hcan1, &CANTxmessage, CAN_Send_Data, &send_mail_box);			//��һ������ͨ�� CAN ���߷���
}

/**
  *@brif ���͵������е������ֵ
  */

void Chassis_Send_current()
{
    CAN_Send_3508_5_8_current(Chassis.FIRE_MOTOR->current_input, 0, 0, 0);
    CONTROL_SEND_HZ(2);//����Ƶ��1000hz��ÿ���η���һ�μ�500hz
    DJIMotor_Send(Chassis.RF_MOTOR);
}

/**
  *@brif ������ȡ�����������
  */
void Read_SuperPower_Vout(void)
{
    CAN_TxHeaderTypeDef CAN1_Txmessage;
    uint8_t CAN1_Tx_Data[8]={0};
    uint32_t send_mail_box;
    CAN1_Txmessage.StdId =0x612;	  
    CAN1_Txmessage.IDE = CAN_ID_STD;
    CAN1_Txmessage.RTR = CAN_RTR_REMOTE; //Զ��֡��ȡ����
    CAN1_Txmessage.DLC   = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box); //��һ������ͨ�� CAN ���߷���
}

/**
  *@brif ��ʵ����ģ�͹�������
  */
void True_power_limit(uint8_t Chassis_Power_Limit)
{
    fp32 Chassis_Power_All = 0;//�����ܹ���
    int16_t speed[4];//ת��ת��
    int16_t I_out[4];//���ص���
    fp32 P_out[4];//����������
    fp32 P_in[4];//������빦��
    fp32 P_back_electromotive_force[4];//������綯�����
    fp32 P_heat[4];//����������
        
    uint8_t RF = 0;
    uint8_t RB = 1;
    uint8_t LB = 2;
    uint8_t LF = 3;
        
    fp32 P_C620 = 1.25f;//����������
    fp32 K = 1.99688994e-6f; // (20/16384)  �����������ʵ������     *(0.3)*(187/3591)/9.55   M3509Ť�ر�������   ���ֵת��Ϊ����
    fp32 K_1 = 1.453e-07;                 // k1  �Ѿ��� K
    fp32 a = 1.23e-07;                     // k2

    /*---ת��ת��---*/
    //ǰ�� L�� R��
    speed[RF] = Chassis.RF_MOTOR->Motor_Information.speed;
    speed[RB] = Chassis.RB_MOTOR->Motor_Information.speed;
    speed[LB] = Chassis.LB_MOTOR->Motor_Information.speed;
    speed[LF] = Chassis.LF_MOTOR->Motor_Information.speed;
    /*---��ǰ����������ص���---*/
    I_out[RF] = Chassis.RF_MOTOR->Speed_PID.out;
    I_out[RB] = Chassis.RB_MOTOR->Speed_PID.out;
    I_out[LB] = Chassis.LB_MOTOR->Speed_PID.out;
    I_out[LF] = Chassis.LF_MOTOR->Speed_PID.out;
    /*---�������---*/
    P_out[RF] = K * I_out[RF] * speed[RF];
    P_out[RB] = K * I_out[RB] * speed[RB];
    P_out[LB] = K * I_out[LB] * speed[LB];
    P_out[LF] = K * I_out[LF] * speed[LF];
    /*---���綯�����---*/
    P_back_electromotive_force[RF] = K_1*Chassis.RF_MOTOR->Motor_Information.speed*Chassis.RF_MOTOR->Motor_Information.speed;
    P_back_electromotive_force[RB] = K_1*Chassis.RB_MOTOR->Motor_Information.speed*Chassis.RB_MOTOR->Motor_Information.speed;
    P_back_electromotive_force[LB] = K_1*Chassis.LB_MOTOR->Motor_Information.speed*Chassis.LB_MOTOR->Motor_Information.speed;
    P_back_electromotive_force[LF] = K_1*Chassis.LF_MOTOR->Motor_Information.speed*Chassis.LF_MOTOR->Motor_Information.speed;
    /*---�����������---*/
    P_heat[RF] = a*I_out[RF]*I_out[RF];
    P_heat[RB] = a*I_out[RB]*I_out[RB];
    P_heat[LB] = a*I_out[LB]*I_out[LB];
    P_heat[LF] = a*I_out[LF]*I_out[LF];
    /*---���빦��---*/
    P_in[RF] = P_out[RF] + P_back_electromotive_force[RF]  + P_heat[RF] + P_C620;
    P_in[RB] = P_out[RB] + P_back_electromotive_force[RB]  + P_heat[RB] + P_C620;
    P_in[LB] = P_out[LB] + P_back_electromotive_force[LB]  + P_heat[LB] + P_C620;
    P_in[LF] = P_out[LF] + P_back_electromotive_force[LF]  + P_heat[LF] + P_C620;
    /*---������������ܺ�---*/
    Chassis_Power_All = 0;
    for(int i = RF;i <= LF; i++)
    {
        if ( P_in[i] > 0 )  Chassis_Power_All += P_in[i];//����˲ʱ������ �����
    }    

    if ( Chassis_Power_All > Chassis_Power_Limit )//���ܺͳ����ʣ�������
    {
        fp32 divisor = Chassis_Power_Limit / Chassis_Power_All;//��������
        /*---Ŀ�����빦�ʵȱ�����С---*/
        P_in[RF] = P_in[RF] * divisor;
        P_in[RB] = P_in[RB] * divisor;
        P_in[LB] = P_in[LB] * divisor;
        P_in[LF] = P_in[LF] * divisor;
        /*---���������ż�������빦���µ��������---*/
        for (int i = RF; i <= LF; i++)
        {
            if ( P_in[i] < 0 )    continue;
            fp32 b = K * speed[i];
            fp32 c = K_1 * speed[i] * speed[i] - P_in[i] + P_C620;
            if ( I_out[i] > 0 )
            {
                fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);//ά��ԭ���˶�״̬
                if (temp > MOTOR_3508_CURRENT_LIMIT)    I_out[i] = MOTOR_3508_CURRENT_LIMIT;
                else                                    I_out[i] = temp;
            }
            else 
            {
                fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);//����
                if (temp < -MOTOR_3508_CURRENT_LIMIT)    I_out[i] = -MOTOR_3508_CURRENT_LIMIT;
                else                                     I_out[i] = temp;
            }
        }
        /*---���ķ��ͽo���������---*/
        Chassis.RF_MOTOR->current_input =  I_out[RF];
        Chassis.RB_MOTOR->current_input =  I_out[RB];
        Chassis.LB_MOTOR->current_input =  I_out[LB];
        Chassis.LF_MOTOR->current_input =  I_out[LF];
    }
}
/**
  *@brif ���㳬��ʣ������
  */
void Calc_SuperPower_energy(void)
{
    float superPower_v;
    superPower_v = ((float) Chassis.pm01_od->v_out) / 100.0f;
    Chassis.pm01_od->energy = ((float)(superPower_v-12)/(float)(24-12)) * 100;//13v�������״��������ѹ����
}
/**
  *@brif ���ݳ���ʣ���������������������
  */
uint8_t Calc_SuperPower_Out_power_limit(void)
{

    if (Chassis.pm01_od->energy >= 30)   return Chassis.Robot_cmd->Power_Mode == 1?160:120;;//160w����
    
    if (Chassis.pm01_od->energy >= 20 && Chassis.pm01_od->energy < 30)   return chassis_power_limit;//���빦��

    if (Chassis.pm01_od->energy < 20 && Chassis.pm01_od->energy >= 0)  return chassis_power_limit + Chassis.pm01_od->energy - 20;//���빦������

    return  chassis_power_limit-20;


}

Chassis_t *get_chassis_point(void){
    return &Chassis;
}
int64_t get_fire_motor_encoder(void)
{
    return Chassis.FIRE_MOTOR->Motor_encoder.Record_Angle;
}