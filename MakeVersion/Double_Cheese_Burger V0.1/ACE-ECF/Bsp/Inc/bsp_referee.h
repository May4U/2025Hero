#ifndef __BSP_REFEREE_H
#define __BSP_REFEREE_H

#include "CRC.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define NULL 0
#define Referee_Data_len 128

//帧头长度
#define HEADER_LEN 5
//指令长度
#define CMDID_LEN 2
//CRC冗余码长度1
#define CRC16_LEN 2

//数据段长度 and 命令码ID and 数据结构体
#define DATA_STATUS_LEN													11						//!比赛状态数据长度(官方有误)
#define ID_STATE 																0x0001				//比赛状态数据
/*比赛状态数据*/
typedef __packed struct
{
	uint8_t	 game_type : 4; //比赛类型
	uint8_t  game_progress : 4; //当前比赛阶段
	uint16_t stage_remain_time; //当前阶段剩余时间，单位：秒
	uint64_t SyncTimeStamp; //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
	uint8_t  error;
} Game_Type_Data;

#define DATA_RESULT_LEN													1						 	//比赛结果数据长度
#define ID_RESULT 															0x0002				//比赛结果数据
/*比赛结果数据*/
typedef __packed struct
{
	uint8_t winner;
	uint8_t error;
}	Game_Result;

#define DATA_ROBOT_HP_LEN												32					 	//!比赛机器人血量数据长度(官方有误)
#define ID_ROBOT_HP 														0x0003				//比赛机器人机器人血量数据
/*血量数据*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP; //红方前哨站血量
	uint16_t red_base_HP;  //红方基地血量
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


#define DATA_EVENT_DATA_LEN											4					 		//场地事件数据长度
#define ID_EVENT_DATA 													0x0101				//场地事件数据
/*场地事件数据 after*/
typedef __packed struct
{
  uint8_t Depot_FrontBloodPoint : 1; //己方补给站前补血点的占领状态
	uint8_t Depot_InBloodPoint : 1; //己方补给站内部补血点的占领状态
	uint8_t Depot : 1;              //己方补给区的占领状态
	uint8_t Energy_Point : 1; //己方能量机关激活点的占领状态
	uint8_t Smail_Energy_Act : 1; //己方小能量机关的激活状态
	uint8_t Large_Energy_Act : 1; //己方大能量机关的激活状态
	uint8_t Annular_HighLand_2 : 2; //己方 2 号环形高地的占领状态
  uint8_t Ladder_HighLand_3 : 2; //己方 3 号梯形高地的占领状态
	uint8_t Ladder_HighLand_4 : 2; //己方 4 号梯形高地的占领状态
  uint8_t Base_Shield : 7; //己方基地虚拟护盾的值
	uint16_t Dart_Hit_Last_Time : 9; //最后一次击中己方前哨站或基地的飞镖击中时间
  uint8_t Dart_Hit_ID : 2; //最后一次击中己方前哨站或基地的飞镖击中具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
	uint8_t Center_Gain_Point : 2;
  uint8_t error;
} Area_Data;

#define DATA_SUPPLY_PROJECTILE_ACTION_LEN				4		 					//补给状态数据长度
#define ID_SUPPLY_PROJECTILE_ACTION 						0x0102	   		//补给状态数据
typedef __packed struct
{
	uint8_t other;
	uint8_t supply_robot_id; //补弹机器人 ID
	uint8_t supply_projectile_step; //出弹口开闭状态
	uint8_t supply_projectile_num; //补弹数量
	uint8_t error;/*此处有误？*/
} Supply_Data;

#define DATA_REFEREE_WARNING_LEN								3				 			//裁判警告数据长度
#define ID_REFEREE_WARNING		 									0x0104			  //裁判警告数据
/*裁判警告信息 after*/
typedef __packed struct
{
	uint8_t level; //己方最后一次受到判罚的等级
	uint8_t foul_robot_id; //己方最后一次受到判罚的违规机器人 ID
	uint8_t count; //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
	uint8_t error;
}Referee_Warning;

#define DATA_DART_REMAINING_TIME_LEN						3			 				//飞镖发射口倒计时
#define ID_DART_REMAINING_TIME 									0x0105		   	//飞镖发射口倒计时
/*飞镖发射口数据 after*/
typedef __packed struct
{
	uint8_t dart_remaining_time; //己方飞镖发射剩余时间，单位：秒
	uint8_t Dart_Hit_ID : 2; //最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
	uint8_t Hit_Count : 3; //对方最近被击中的目标累计被击中计数
	uint8_t Dart_Choose_ID : 2; //飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基地固定目标 1，选中基地随机目标为 2
	uint16_t other : 9;
	uint8_t error;
} Dart_Launch_Data;

#define DATA_ROBOT_STATUS_LEN										13				 		//机器人状态数据
#define ID_ROBOT_STATE 													0x0201				//机器人状态数据
/*机器人状态数据*/
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value;   //机器人枪口热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;      //机器人枪口热量上限
  uint16_t chassis_power_limit;            //机器人底盘功率上限
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
	uint8_t other : 5;
	uint8_t error;
} Robot_Situation_Data;

#define DATA_POWER_HEAT_DATA_LEN								16						//实时功率热量数据
#define ID_POWER_HEAT_DATA	 										0x0202			  //实时功率热量数据
/*功率热量数据*/
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer; //缓冲能量（单位：J）
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
	uint8_t error;
} Robot_Power_Heat_Data;

#define DATA_ROBOT_POS_LEN											12					 	//机器人位置数据
#define ID_ROBOT_POS 														0x0203				//机器人位置数据
/*机器人位置 after*/
typedef __packed struct
{
	float x;
	float y;
	float yaw; //本机器人测速模块朝向，单位：度。正北为 0 度
	uint8_t error;
} Robot_Position_Data;

#define DATA_BUFF_LEN														6							//机器人增益数据
#define ID_BUFF 																0x0204				//机器人增益数据
/*机器人增益数据 after*/
typedef __packed struct
{
  uint8_t recovery_buff; //机器人回血增益（百分比，值为 10 意为每秒回复 10%最大血量）
  uint8_t cooling_buff; //机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
  uint8_t defence_buff; //机器人防御增益（百分比，值为 50 意为 50%防御增益）
  uint16_t attack_buff; //机器人攻击增益（百分比，值为 50 意为 50%攻击增益）
	uint8_t error;
} Area_Buff_Data;

#define DATA_AERIAL_ROBOT_ENERGY_LEN						2							//空中机器人能量状态数据
#define ID_AERIAL_ROBOT_ENERGY 									0x0205		   	//空中机器人能量状态数据
/*空中机器人能量状态 after*/
typedef __packed struct
{
  uint8_t airforce_status; //空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为空中支援期间）
  uint8_t time_remain; //此状态的剩余时间（单位为 s，向下取整，即冷却时间剩余 1.9s 时，此值为 1）
	uint8_t error;
} UAV_Data;

#define DATA_ROBOT_HURT_LEN											1							//伤害状态数据
#define ID_ROBOT_HURT 													0x0206				//伤害状态数据
/*伤害状态*/
typedef __packed struct
{
	uint8_t armor_id : 4; //当扣血原因为装甲模块或测速模块时，该 4bit 组成的数值为装甲模块或测速模块的 ID 编号
	uint8_t hurt_type : 4; //血量变化类型
// 0 装甲被弹丸攻击扣血
// 1 裁判系统重要模块离线扣血
// 2 射击初速度超限扣血
// 3 枪口热量超限扣血
// 4 底盘功率超限扣血
// 5 装甲模块受到撞击扣血
	uint8_t error;
} Robot_Hurt_Data;

#define DATA_SHOOT_DATA_LEN											7					 		//实时射击数据
#define ID_SHOOT_DATA 													0x0207				//实时射击数据
/*实时射击信息*/
typedef __packed struct
{
	uint8_t bullet_type; //弹丸类型： 1：17mm 弹丸 2：42mm 弹丸
	uint8_t shooter_id; //发射机构 ID： 1：第 1 个 17mm 发射机构 2：第 2 个 17mm 发射机构 3：42mm 发射机构
	uint8_t bullet_freq; //弹丸射速（单位：Hz）
	float bullet_speed; //弹丸初速度（单位：m/s）
	uint8_t error;
} Robot_Shoot_Data;

#define DATA_BULLET_REMAINING_LEN								6							//!子弹剩余发送数(官方有误)
#define ID_BULLET_REMAINING											0x0208			  //子弹剩余发送数
/*子弹剩余发射数*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm 子弹剩余发射数目
	uint16_t bullet_remaining_num_42mm; //42mm 子弹剩余发射数目
	uint16_t coin_remaining_num;		//剩余金币数量
	uint8_t  error;
} Robot_RaminingBullet_Data;

#define DATA_RFID_STATUS_LEN										4							//机器人 RFID 状态
#define ID_RFID_STATUS 													0x0209				//机器人 RFID 状态
/*RFID状态 after*/
typedef __packed struct
{
	//是否已检测到该增益点 RFID 卡
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

#define DATA_DART_CLIENT_CMD_LEN				6			  //飞镖机器人客户端指令数据
#define ID_DART_CLIENT_CMD                      0x020A        //飞镖机器人客户端指令数据
/*飞镖机器人客户端指令数据*/
typedef __packed struct
{
  uint8_t dart_launch_opening_status;
  uint8_t other;
  uint16_t target_change_time;  //是剩余时间
  uint16_t operate_launch_cmd_time;   //是剩余时间
	uint8_t error;
} Dart_Client_Cmd;

#define DATA_GROUND_ROBOT_POSITION_CMD_LEN         40     //机器人位置坐标数据长度
#define ID_GROUND_ROBOT_POSITION_CMD               0x020B   //机器人位置坐标数据
/*机器人位置坐标数据*/
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

#define DATA_RADAR_MARK_CMD_LEN             6      //敌方机器人被标记进度数据长度
#define ID_RADAR_MARK_CMD               0x020C   //敌方机器人被标记进度数据
/*敌方机器人被标记进度数据*/
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

#define DATA_SENTRY_INFO_CMD_LEN             4          //哨兵机器人兑换相关数据长度
#define ID_SENTRY_INFO_CMD                   0x020D     //哨兵机器人兑换相关数据
/*哨兵机器人兑换相关数据*/
typedef __packed struct 
{ 
  	uint16_t Num_of_bullets_exchanged : 11;  //除远程兑换外，哨兵成功兑换的发弹量
	uint8_t  Num_of_times_to_exchange_bullets_remotely : 4;
	uint8_t  Num_of_times_to_exchange_HP_remotely : 4;
	uint16_t  other : 13;
	uint8_t  error;
}sentry_info_t; 

#define DATA_RADAR_INFO_CMD_LEN             1   
#define ID_RADAR_INFO_CMD                   0x020E
/*雷达双倍易伤相关数据*/
typedef __packed struct 
{ 
  uint8_t Buff_Num : 2; //雷达是否拥有触发双倍易伤的机会，开局为0，数值为雷达拥有触发双倍易伤的机会，至多为2
	uint8_t Enemy_Buff : 1;  //对方是否正在被触发双倍易伤
	uint8_t other : 5;
	uint8_t  error;
}radar_info_t; 

#define DATA_ROBOT_INTERACTION_DATA_LEN             121
#define ID_ROBOT_INTERACTION_DATA                   0x0301
typedef __packed struct 
{ 
  uint16_t data_cmd_id;		//子内容 ID，需为开放的子内容 ID
  uint16_t sender_id;       //发送者 ID，需与自身 ID 匹配，查看裁判系统协议附录内容
  uint16_t receiver_id;     //接收者 ID
  uint8_t user_data[113];     //内容数据段
  uint8_t error;
}robot_interaction_data_t; 

#define DATA_DIY_CONTROLLER                     30            //自定义控制器
#define ID_DIY_CONTROLLER  											0x0302 				//自定义控制器
/*交互数据接收信息*/
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
typedef __packed struct //哨兵自主决策指令
{
	uint8_t confirm_resurrection : 1;//1:确认复活； 0：确认不复活
	uint8_t confirm_purchase_resurrection : 1;//1:哨兵机器人确认兑换立即复活; 0:哨兵机器人确认不兑换立即复活
	uint16_t Num_of_bullets_will_exchanged : 11;//哨兵将要兑换的发弹量值，开局为 0，修改此值后，哨兵在补血点即可兑换允许发弹量
	uint8_t Num_of_requests_for_exchange_bullets_remotely : 4;//哨兵远程兑换发弹量的请求次数,开局为 0
	uint8_t Num_of_requests_for_exchange_HP_remotely      : 4;//哨兵远程兑换血量的请求次数,开局为 0
	uint16_t other : 11;
	uint8_t error;
}sentry_cmd_t;

#define DATA_RADAR_CMD_LEN 1                
#define ID_RADAR_CMD       0x0121
typedef __packed struct //雷达自主决策指令
{
	uint8_t confirm_to_start_double_damage;//开局为 0，修改此值即可请求触发双倍易伤，若此时雷达拥有触发双倍易伤的机会，则可触发
	uint8_t error;
}radar_cmd_t;


#define DATA_CLIENT_DOWMLOAD_LEN                    12            //小地图下发位置信息
#define ID_CLIENT_DOWMLOAD  										0x0303     		//小地图下发位置信息
/*客户端下发信息*/
typedef __packed struct
{
  float target_position_x; 
  float target_position_y; 
  uint8_t cmd_keyboard; 
  uint8_t target_robot_id; 
  uint16_t Information_source_ID; 
	uint8_t error;
} Robot_Command;


#define DATA_PICTURE_TRANSMISSION_LEN               12            //图传遥控信息
#define ID_PICTURE_TRANSMISSION 								0x0304				//图传遥控信息
/*键鼠遥控数据*/
typedef __packed struct 
{ 
  int16_t mouse_x; 
  int16_t mouse_y; 
  int16_t mouse_z;   //鼠标滚轮移动速度
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

#define DATA_CLIENT_RECEIVE_LEN                     10            //小地图接收位置信息
#define ID_CLIENT_RECEIVE  											0x0305     		//小地图接收位置信息
/*客户端接受信息*/
//雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	uint8_t error;
} Client_Map_Command_Data;

/*裁判系统数据*/
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

//裁判系统初始化
void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
void ECF_referee_uart_init(void);
void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart);
REFEREE_t *Get_referee_Address(void);

#endif
