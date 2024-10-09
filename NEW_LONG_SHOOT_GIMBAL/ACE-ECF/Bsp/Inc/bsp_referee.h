#ifndef __BSP_REFEREE_H
#define __BSP_REFEREE_H

#include "CRC.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define NULL 0
#define Referee_Data_len 128

//֡ͷ����
#define HEADER_LEN 5
//ָ���
#define CMDID_LEN 2
//CRC�����볤��1
#define CRC16_LEN 2

//���ݶγ��� and ������ID and ���ݽṹ��
#define DATA_STATUS_LEN													11						//!����״̬���ݳ���(�ٷ�����)
#define ID_STATE 																0x0001				//����״̬����
/*����״̬����*/
typedef __packed struct
{
	uint8_t	 game_type : 4; //��������
	uint8_t  game_progress : 4; //��ǰ�����׶�
	uint16_t stage_remain_time; //��ǰ�׶�ʣ��ʱ�䣬��λ����
	uint64_t SyncTimeStamp; //UNIX ʱ�䣬����������ȷ���ӵ�����ϵͳ�� NTP ����������Ч
	uint8_t  error;
} Game_Type_Data;

#define DATA_RESULT_LEN													1						 	//����������ݳ���
#define ID_RESULT 															0x0002				//�����������
/*�����������*/
typedef __packed struct
{
	uint8_t winner;
	uint8_t error;
}	Game_Result;

#define DATA_ROBOT_HP_LEN												32					 	//!����������Ѫ�����ݳ���(�ٷ�����)
#define ID_ROBOT_HP 														0x0003				//���������˻�����Ѫ������
/*Ѫ������*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP; //�췽ǰ��վѪ��
	uint16_t red_base_HP;  //�췽����Ѫ��
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
	uint8_t error;
} Robot_Hp_Data;


#define DATA_EVENT_DATA_LEN											4					 		//�����¼����ݳ���
#define ID_EVENT_DATA 													0x0101				//�����¼�����
/*�����¼����� after*/
typedef __packed struct
{
  uint8_t Depot_FrontBloodPoint : 1; //��������վǰ��Ѫ���ռ��״̬
	uint8_t Depot_InBloodPoint : 1; //��������վ�ڲ���Ѫ���ռ��״̬
	uint8_t Depot : 1;              //������������ռ��״̬
	uint8_t Energy_Point : 1; //�����������ؼ�����ռ��״̬
	uint8_t Smail_Energy_Act : 1; //����С�������صļ���״̬
	uint8_t Large_Energy_Act : 1; //�������������صļ���״̬
	uint8_t Annular_HighLand_2 : 2; //���� 2 �Ż��θߵص�ռ��״̬
  uint8_t Ladder_HighLand_3 : 2; //���� 3 �����θߵص�ռ��״̬
	uint8_t Ladder_HighLand_4 : 2; //���� 4 �����θߵص�ռ��״̬
  uint8_t Base_Shield : 7; //�����������⻤�ܵ�ֵ
	uint16_t Dart_Hit_Last_Time : 9; //���һ�λ��м���ǰ��վ����صķ��ڻ���ʱ��
  uint8_t Dart_Hit_ID : 2; //���һ�λ��м���ǰ��վ����صķ��ڻ��о���Ŀ�꣬����Ĭ��Ϊ 0��1 Ϊ����ǰ��վ��2 Ϊ���л��ع̶�Ŀ�꣬3 Ϊ���л������Ŀ��
	uint8_t Center_Gain_Point : 2;
  uint8_t error;
} Area_Data;

#define DATA_SUPPLY_PROJECTILE_ACTION_LEN				4		 					//����״̬���ݳ���
#define ID_SUPPLY_PROJECTILE_ACTION 						0x0102	   		//����״̬����
typedef __packed struct
{
	uint8_t other;
	uint8_t supply_robot_id; //���������� ID
	uint8_t supply_projectile_step; //�����ڿ���״̬
	uint8_t supply_projectile_num; //��������
	uint8_t error;/*�˴�����*/
} Supply_Data;

#define DATA_REFEREE_WARNING_LEN								3				 			//���о������ݳ���
#define ID_REFEREE_WARNING		 									0x0104			  //���о�������
/*���о�����Ϣ after*/
typedef __packed struct
{
	uint8_t level; //�������һ���ܵ��з��ĵȼ�
	uint8_t foul_robot_id; //�������һ���ܵ��з���Υ������� ID
	uint8_t count; //�������һ���ܵ��з���Υ������˶�Ӧ�з��ȼ���Υ�����
	uint8_t error;
}Referee_Warning;

#define DATA_DART_REMAINING_TIME_LEN						3			 				//���ڷ���ڵ���ʱ
#define ID_DART_REMAINING_TIME 									0x0105		   	//���ڷ���ڵ���ʱ
/*���ڷ�������� after*/
typedef __packed struct
{
	uint8_t dart_remaining_time; //�������ڷ���ʣ��ʱ�䣬��λ����
	uint8_t Dart_Hit_ID : 2; //���һ�μ������ڻ��е�Ŀ�꣬����Ĭ��Ϊ 0��1 Ϊ����ǰ��վ��2 Ϊ���л��ع̶�Ŀ�꣬3 Ϊ���л������Ŀ��
	uint8_t Hit_Count : 3; //�Է���������е�Ŀ���ۼƱ����м���
	uint8_t Dart_Choose_ID : 2; //���ڴ�ʱѡ���Ļ���Ŀ�꣬����Ĭ�ϻ�δѡ��/ѡ��ǰ��վʱΪ 0��ѡ�л��ع̶�Ŀ�� 1��ѡ�л������Ŀ��Ϊ 2
	uint16_t other : 9;
	uint8_t error;
} Dart_Launch_Data;

#define DATA_ROBOT_STATUS_LEN										13				 		//������״̬����
#define ID_ROBOT_STATE 													0x0201				//������״̬����
/*������״̬����*/
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value;   //������ǹ������ÿ����ȴֵ
  uint16_t shooter_barrel_heat_limit;      //������ǹ����������
  uint16_t chassis_power_limit;            //�����˵��̹�������
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
	uint8_t other : 5;
	uint8_t error;
} Robot_Situation_Data;

#define DATA_POWER_HEAT_DATA_LEN								16						//ʵʱ������������
#define ID_POWER_HEAT_DATA	 										0x0202			  //ʵʱ������������
/*������������*/
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer; //������������λ��J��
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
	uint8_t error;
} Robot_Power_Heat_Data;

#define DATA_ROBOT_POS_LEN											12					 	//������λ������
#define ID_ROBOT_POS 														0x0203				//������λ������
/*������λ�� after*/
typedef __packed struct
{
	float x;
	float y;
	float yaw; //�������˲���ģ�鳯�򣬵�λ���ȡ�����Ϊ 0 ��
	uint8_t error;
} Robot_Position_Data;

#define DATA_BUFF_LEN														6							//��������������
#define ID_BUFF 																0x0204				//��������������
/*�������������� after*/
typedef __packed struct
{
  uint8_t recovery_buff; //�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��Ϊÿ��ظ� 10%���Ѫ����
  uint8_t cooling_buff; //������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ζ�� 5 ����ȴ��
  uint8_t defence_buff; //�����˷������棨�ٷֱȣ�ֵΪ 50 ��Ϊ 50%�������棩
  uint16_t attack_buff; //�����˹������棨�ٷֱȣ�ֵΪ 50 ��Ϊ 50%�������棩
	uint8_t error;
} Area_Buff_Data;

#define DATA_AERIAL_ROBOT_ENERGY_LEN						2							//���л���������״̬����
#define ID_AERIAL_ROBOT_ENERGY 									0x0205		   	//���л���������״̬����
/*���л���������״̬ after*/
typedef __packed struct
{
  uint8_t airforce_status; //���л�����״̬��0 Ϊ������ȴ��1 Ϊ��ȴ��ϣ�2 Ϊ����֧Ԯ�ڼ䣩
  uint8_t time_remain; //��״̬��ʣ��ʱ�䣨��λΪ s������ȡ��������ȴʱ��ʣ�� 1.9s ʱ����ֵΪ 1��
	uint8_t error;
} UAV_Data;

#define DATA_ROBOT_HURT_LEN											1							//�˺�״̬����
#define ID_ROBOT_HURT 													0x0206				//�˺�״̬����
/*�˺�״̬*/
typedef __packed struct
{
	uint8_t armor_id : 4; //����Ѫԭ��Ϊװ��ģ������ģ��ʱ���� 4bit ��ɵ���ֵΪװ��ģ������ģ��� ID ���
	uint8_t hurt_type : 4; //Ѫ���仯����
// 0 װ�ױ����蹥����Ѫ
// 1 ����ϵͳ��Ҫģ�����߿�Ѫ
// 2 ������ٶȳ��޿�Ѫ
// 3 ǹ���������޿�Ѫ
// 4 ���̹��ʳ��޿�Ѫ
// 5 װ��ģ���ܵ�ײ����Ѫ
	uint8_t error;
} Robot_Hurt_Data;

#define DATA_SHOOT_DATA_LEN											7					 		//ʵʱ�������
#define ID_SHOOT_DATA 													0x0207				//ʵʱ�������
/*ʵʱ�����Ϣ*/
typedef __packed struct
{
	uint8_t bullet_type; //�������ͣ� 1��17mm ���� 2��42mm ����
	uint8_t shooter_id; //������� ID�� 1���� 1 �� 17mm ������� 2���� 2 �� 17mm ������� 3��42mm �������
	uint8_t bullet_freq; //�������٣���λ��Hz��
	float bullet_speed; //������ٶȣ���λ��m/s��
	uint8_t error;
} Robot_Shoot_Data;

#define DATA_BULLET_REMAINING_LEN								6							//!�ӵ�ʣ�෢����(�ٷ�����)
#define ID_BULLET_REMAINING											0x0208			  //�ӵ�ʣ�෢����
/*�ӵ�ʣ�෢����*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm �ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm; //42mm �ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;		//ʣ��������
	uint8_t  error;
} Robot_RaminingBullet_Data;

#define DATA_RFID_STATUS_LEN										4							//������ RFID ״̬
#define ID_RFID_STATUS 													0x0209				//������ RFID ״̬
/*RFID״̬ after*/
typedef __packed struct
{
	//�Ƿ��Ѽ�⵽������� RFID ��
	uint8_t Our_BaseLand : 1;
	uint8_t Our_Annular_HighLand : 1;
	uint8_t Enermy_Annular_HighLand : 1;
	uint8_t Our_R3_B3_Ladder_HighLand_3 : 1;
	uint8_t Enermy_R3_B3_Ladder_HighLand_3 : 1;
	uint8_t Our_R4_B4_Ladder_HighLand_4 : 1;
	uint8_t Enermy_R4_B4_Ladder_HighLand_4 : 1;
	uint8_t Our_Energy : 1;
	uint8_t Our_Fly_Near : 1;
	uint8_t Our_Fly_Far : 1;
	uint8_t Enermy_Fly_Near : 1;
	uint8_t Enermy_Fly_Far : 1;
	uint8_t Our_Outpost : 1;
	uint8_t Our_Blood_Enrich : 1;
	uint8_t Our_Sentinel_Patrol : 1;
	uint8_t Enermy_Sentinel_Patrol : 1;
	uint8_t Our_Large_Resource_island : 1;
	uint8_t Enermy_Large_Resource_island : 1;
	uint8_t Our_Exchange : 1;
	uint8_t Center_Gain_Point : 1;
	uint16_t other : 12;
	uint8_t error;
} RFID_Situation_Data;

#define DATA_DART_CLIENT_CMD_LEN				6			  //���ڻ����˿ͻ���ָ������
#define ID_DART_CLIENT_CMD                      0x020A        //���ڻ����˿ͻ���ָ������
/*���ڻ����˿ͻ���ָ������*/
typedef __packed struct
{
  uint8_t dart_launch_opening_status;
  uint8_t other;
  uint16_t target_change_time;  //��ʣ��ʱ��
  uint16_t operate_launch_cmd_time;   //��ʣ��ʱ��
	uint8_t error;
} Dart_Client_Cmd;

#define DATA_GROUND_ROBOT_POSITION_CMD_LEN         40     //������λ���������ݳ���
#define ID_GROUND_ROBOT_POSITION_CMD               0x020B   //������λ����������
/*������λ����������*/
typedef __packed struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float standard_5_x;  
  float standard_5_y; 
	uint8_t  error;
}ground_robot_position_t; 

#define DATA_RADAR_MARK_CMD_LEN             6      //�з������˱���ǽ������ݳ���
#define ID_RADAR_MARK_CMD               0x020C   //�з������˱���ǽ�������
/*�з������˱���ǽ�������*/
typedef __packed struct 
{ 
  uint8_t mark_hero_progress;  
  uint8_t mark_engineer_progress;  
  uint8_t mark_standard_3_progress;  
  uint8_t mark_standard_4_progress; 
  uint8_t mark_standard_5_progress; 
  uint8_t mark_sentry_progress; 
	uint8_t  error;
}radar_mark_data_t; 

//#define DATA_STUDENT_INTERACTIVE_HEADER_DATA  6             //UI
//#define ID_STUDENT_INTERACTIVE_HEADER_DATA    0x0301       // UI

#define DATA_SENTRY_INFO_CMD_LEN             4          //�ڱ������˶һ�������ݳ���
#define ID_SENTRY_INFO_CMD                   0x020D     //�ڱ������˶һ��������
/*�ڱ������˶һ��������*/
typedef __packed struct 
{ 
  	uint16_t Num_of_bullets_exchanged : 11;  //��Զ�̶һ��⣬�ڱ��ɹ��һ��ķ�����
	uint8_t  Num_of_times_to_exchange_bullets_remotely : 4;
	uint8_t  Num_of_times_to_exchange_HP_remotely : 4;
	uint16_t  other : 13;
	uint8_t  error;
}sentry_info_t; 

#define DATA_RADAR_INFO_CMD_LEN             1   
#define ID_RADAR_INFO_CMD                   0x020E
/*�״�˫�������������*/
typedef __packed struct 
{ 
  uint8_t Buff_Num : 2; //�״��Ƿ�ӵ�д���˫�����˵Ļ��ᣬ����Ϊ0����ֵΪ�״�ӵ�д���˫�����˵Ļ��ᣬ����Ϊ2
	uint8_t Enemy_Buff : 1;  //�Է��Ƿ����ڱ�����˫������
	uint8_t other : 5;
	uint8_t  error;
}radar_info_t; 

#define DATA_ROBOT_INTERACTION_DATA_LEN             121
#define ID_ROBOT_INTERACTION_DATA                   0x0301
typedef __packed struct 
{ 
  uint16_t data_cmd_id;		//������ ID����Ϊ���ŵ������� ID
  uint16_t sender_id;       //������ ID���������� ID ƥ�䣬�鿴����ϵͳЭ�鸽¼����
  uint16_t receiver_id;     //������ ID
  uint8_t user_data[113];     //�������ݶ�
  uint8_t error;
}robot_interaction_data_t; 

#define DATA_DIY_CONTROLLER                     30            //�Զ��������
#define ID_DIY_CONTROLLER  											0x0302 				//�Զ��������
/*�������ݽ�����Ϣ*/
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
//uint16_t data[];	
	uint8_t error;
} student_interactive_header_data_t;

#define DATA_SENTRY_CMD_LEN					4         
#define ID_SENTRY_CMD						0x0120
typedef __packed struct //�ڱ���������ָ��
{
	uint8_t confirm_resurrection : 1;//1:ȷ�ϸ�� 0��ȷ�ϲ�����
	uint8_t confirm_purchase_resurrection : 1;//1:�ڱ�������ȷ�϶һ���������; 0:�ڱ�������ȷ�ϲ��һ���������
	uint16_t Num_of_bullets_will_exchanged : 11;//�ڱ���Ҫ�һ��ķ�����ֵ������Ϊ 0���޸Ĵ�ֵ���ڱ��ڲ�Ѫ�㼴�ɶһ���������
	uint8_t Num_of_requests_for_exchange_bullets_remotely : 4;//�ڱ�Զ�̶һ����������������,����Ϊ 0
	uint8_t Num_of_requests_for_exchange_HP_remotely      : 4;//�ڱ�Զ�̶һ�Ѫ�����������,����Ϊ 0
	uint16_t other : 11;
	uint8_t error;
}sentry_cmd_t;

#define DATA_RADAR_CMD_LEN 1                
#define ID_RADAR_CMD       0x0121
typedef __packed struct //�״���������ָ��
{
	uint8_t confirm_to_start_double_damage;//����Ϊ 0���޸Ĵ�ֵ�������󴥷�˫�����ˣ�����ʱ�״�ӵ�д���˫�����˵Ļ��ᣬ��ɴ���
	uint8_t error;
}radar_cmd_t;


#define DATA_CLIENT_DOWMLOAD_LEN                    12            //С��ͼ�·�λ����Ϣ
#define ID_CLIENT_DOWMLOAD  										0x0303     		//С��ͼ�·�λ����Ϣ
/*�ͻ����·���Ϣ*/
typedef __packed struct
{
  float target_position_x; 
  float target_position_y; 
  uint8_t cmd_keyboard; 
  uint8_t target_robot_id; 
  uint16_t Information_source_ID; 
	uint8_t error;
} Robot_Command;


#define DATA_PICTURE_TRANSMISSION_LEN               12            //ͼ��ң����Ϣ
#define ID_PICTURE_TRANSMISSION 								0x0304				//ͼ��ң����Ϣ
/*����ң������*/
typedef __packed struct 
{ 
  int16_t mouse_x; 
  int16_t mouse_y; 
  int16_t mouse_z;   //�������ƶ��ٶ�
  uint8_t left_button_down; 
  uint8_t right_button_down; 
  uint8_t Key_W : 1;
	uint8_t Key_S : 1;
	uint8_t Key_A : 1;
	uint8_t Key_D : 1;
	uint8_t Key_Shift : 1;
	uint8_t Key_Ctil : 1;
	uint8_t Key_Q : 1;
	uint8_t Key_E : 1;
	uint8_t Key_R : 1;
	uint8_t Key_F : 1;
	uint8_t Key_G : 1;
	uint8_t Key_Z : 1;
	uint8_t Key_X : 1;
	uint8_t Key_C : 1;
	uint8_t Key_V : 1;
	uint8_t Key_B : 1;
  uint16_t other; 
	uint8_t error;
}remote_control_t; 

#define DATA_CLIENT_RECEIVE_LEN                     10            //С��ͼ����λ����Ϣ
#define ID_CLIENT_RECEIVE  											0x0305     		//С��ͼ����λ����Ϣ
/*�ͻ��˽�����Ϣ*/
//�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ������
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	uint8_t error;
} Client_Map_Command_Data;

/*����ϵͳ����*/
typedef __packed struct
{
	uint8_t RefereeData[256];
	uint8_t RealData[45];
	int16_t DataLen;
	int16_t RealLen;
	int16_t Cmd_ID;
	uint8_t RECEIVE_FLAG;
	Game_Type_Data 															Game_Status;
	Game_Result                 							  Game_Result;
	Robot_Hp_Data 															Robot_HP;
	Area_Data 																	Event_Data;
	Supply_Data 																Supply_Action;
	Referee_Warning             							  Referee_Warning;
	Dart_Launch_Data 														Dart_Remaining_Time;
	Robot_Situation_Data 												Robot_Status;
	Robot_Power_Heat_Data 											Power_Heat;
	Robot_Position_Data         							  Robot_Position;
	Area_Buff_Data 															Buff;
	UAV_Data 																		Aerial_Energy;
	Robot_Hurt_Data 														Robot_Hurt;
	Robot_Shoot_Data 														Shoot_Data;
	Robot_RaminingBullet_Data 									Bullet_Num;
	RFID_Situation_Data 												RFID_Status;
	Dart_Client_Cmd             							  Dart_Client;
	ground_robot_position_t                     Ground_robot_position;
	radar_mark_data_t                           Radar_mark;
	sentry_info_t                               Sentry_info;
	radar_info_t                                Radar_info;
	sentry_cmd_t								Sentry_Cmd;
	radar_cmd_t									Radar_Cmd;
	Robot_Command         			     					  Client_Data;
	remote_control_t                            Remote_control;
	Client_Map_Command_Data          					  ClientMapData;
    student_interactive_header_data_t           Interact_Header;
} REFEREE_t;

//����ϵͳ��ʼ��
void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
void ECF_referee_uart_init(void);
void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart);
REFEREE_t *Get_referee_Address(void);

#endif
