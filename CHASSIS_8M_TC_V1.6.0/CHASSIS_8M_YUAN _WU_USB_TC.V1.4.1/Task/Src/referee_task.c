#include "referee_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "rm_cilent_ui.h"
#include "bsp_referee.h"
#include "maths.h"
#include "Chassis_task.h"
#include "robot_cmd.h"
#include "stdio.h"
extern Chassis_t Chassis;
Chassis_t* CH_P;

//圆心OFFSET
const uint32_t CIRCLE_X0=700;
const uint32_t CIRCLE_Y0=180;
const uint32_t CIRCLE_R0=80;
const uint8_t THETA=40;
String_Data Print_Cap;
String_Data Print_Control;
String_Data Print_auto_fire_info;
String_Data chassis_mode,fric_state,aim_id;
String_Data chassis_mode_set,fric_state_set,aim_id_set;
Graph_Data mid_rectangle;
Graph_Data circle;
Graph_Data line_0,line_1,line_2,line_3,line_4,line_5,line_6,line_7;
Float_Data CAP_VOLTAGE;
String_Data CH_MODE,voltage,replenish,GM_MODE;
String_Data Cap_ENERGE;
String_Data Control_mode;
String_Data Cap;
String_Data Control;
Float_Data Fire_motor_encoder;
Graph_Data diff_ang;
String_Data test;


static void Drow_Motor_Angle_BackGoroud(void);
static void Drow_Super_Power_BackGoroud(void);
//右侧电机角度背景线
static void Drow_Motor_Angle(void);
//左侧超点容量背景线
static void Drow_Super_Power(void);


//左侧超点容量背景线
Graph_Data Super_Power_BackGoroud_Arc[11];
//左侧超点容量
Graph_Data Super_Power_Arc;

//右侧电机角度线
Graph_Data Pitch_Motor_Angle_Arc;
//右侧电机角度值
String_Data Pitch_Motor_Angle_Num;


/*图层0 P轴*/
Graph_Data Motor_Angle_BackGoroud_Arc[9];//P轴电机角度背景线
String_Data Pitch_Motor_Mode_Text;//P轴电机模式文字
Graph_Data Pitch_Motor_Mode_Rect;//P轴电机模式选择框
float debugangle = 50;
void Drow_Pitch_Motor_Angle_BackGoroud(void)
{
    //12点钟位置为0°，顺时针增加
    char imagename[3];
    for (int i = 0; i < 9; i++)
    {
        uint16_t start_angle = 50 + 10*i;
        uint16_t end_angle = start_angle+1;
        uint16_t Rx = 380;
        uint16_t Ry = 380;
        uint16_t width = i%2 == 0? 25:12;
        sprintf(imagename,"00%d",i);
        Arc_Draw(&Motor_Angle_BackGoroud_Arc[i], imagename, UI_Graph_ADD, 0, UI_Color_White, start_angle, end_angle, width, 960, 540, Rx, Ry);
    }
    
//    Char_Draw(&Pitch_Motor_Mode_Text,"011",UI_Graph_ADD,0,UI_Color_White,2,22,1400,850,"ENC\nIMU");
//    Rectangle_Draw(&Pitch_Motor_Mode_Rect, "012", UI_Graph_ADD, 0, UI_Color_White, 4, 1400, 800, 1500, 820);
    
}
void Drow_Pitch_Motor_Angle(void)
{
    Arc_Draw(&Pitch_Motor_Angle_Arc, "009", UI_Graph_ADD, 0, UI_Color_Main, 45, 45+1, 40, 960, 540, 380, 380);
    Char_Draw(&Pitch_Motor_Angle_Num,"010",UI_Graph_ADD,0,UI_Color_Pink,2,22,1240,550,"NULL");
}
void Update_Pitch_Motor_Angle(void)
{
    fp32 Motor_Angle = Get_Gimbal_Pitch_Angle();
    fp32 start_angle_fp;
    uint8_t start_angle;
    if (Get_Gimbal_Pitch_Mode() == 1)
    {
        start_angle_fp =  Motor_Angle;
        start_angle =  90  - Motor_Angle;
//        start_angle_fp = 134.3f + Motor_Angle - 90.0f;
//        start_angle = 134.3f + Motor_Angle ;
    }
    else
    {
        start_angle_fp =  Motor_Angle;
        start_angle =  90  - Motor_Angle;
    } 
    uint8_t Color = Get_Gimbal_Pitch_Mode() == 1? UI_Color_Pink:UI_Color_Cyan;
//    Motor_Angle = loop_fp32_constrain(Motor_Angle, 0, 360); //0-360循环限制
    Arc_Draw(&Pitch_Motor_Angle_Arc, "009", UI_Graph_Change, 0, Color, start_angle, start_angle+1, 40, 960, 540, 380, 380);
//    My_Graph_Refresh(&Pitch_Motor_Angle_Arc);

    char Pitch_Angle[7];
    sprintf(Pitch_Angle,"%.1f",start_angle_fp);
    Char_Draw(&Pitch_Motor_Angle_Num,"010",UI_Graph_Change,0,Color,2,22,1240,550,Pitch_Angle);
//    My_Char_Refresh(Pitch_Motor_Angle_Num);
//    if (Pitch_Lock_Encoder == 0)
//    
//        Rectangle_Draw(&Pitch_Motor_Mode_Rect, "012", UI_Graph_Change, 0, UI_Color_White, 4, 1400, 800, 1500, 820);
//    else
//        Rectangle_Draw(&Pitch_Motor_Mode_Rect, "012", UI_Graph_Change, 0, UI_Color_White, 4, 1400, 800, 1500, 820);


}
/*图层0*/



/*图层1 摩擦轮*/
//摩擦轮速度框
Graph_Data Fire_Motor_Speed_BackGoroud_Rect[4];
//摩擦轮速度刻度
String_Data Fire_Motor_Speed_Num;
void Drow_Fire_Motor_Speed_BackGoroud(void)
{
    //矩形宽度90个像素，每45个像素代表1000转，从4000转起记录
    Rectangle_Draw(&Fire_Motor_Speed_BackGoroud_Rect[0], "100", UI_Graph_ADD, 1, UI_Color_White, 4, 1500, 850, 1590, 870);//左上
    Rectangle_Draw(&Fire_Motor_Speed_BackGoroud_Rect[1], "101", UI_Graph_ADD, 1, UI_Color_White, 4, 1600, 850, 1690, 870);//右上
    Rectangle_Draw(&Fire_Motor_Speed_BackGoroud_Rect[2], "102", UI_Graph_ADD, 1, UI_Color_White, 4, 1500, 800, 1590, 820);//左下
    Rectangle_Draw(&Fire_Motor_Speed_BackGoroud_Rect[3], "103", UI_Graph_ADD, 1, UI_Color_White, 4, 1600, 800, 1690, 820);//右下
    Char_Draw(&Fire_Motor_Speed_Num,"104",UI_Graph_ADD,1,UI_Color_Yellow,3,22,1500,846,"6 5 4 5 6");
}
//摩擦轮速度条
Graph_Data Fire_Motor_Speed[4];
void Drow_Fire_Motor_Speed(void)
{
    Rectangle_Draw(&Fire_Motor_Speed[0], "105", UI_Graph_ADD, 1, UI_Color_Yellow, 3, 1500, 850, 1500, 870);//左上
    Rectangle_Draw(&Fire_Motor_Speed[1], "106", UI_Graph_ADD, 1, UI_Color_Yellow, 3, 1600, 850, 1600, 870);//右上
    Rectangle_Draw(&Fire_Motor_Speed[2], "107", UI_Graph_ADD, 1, UI_Color_Yellow, 3, 1500, 800, 1500, 820);//左下
    Rectangle_Draw(&Fire_Motor_Speed[3], "108", UI_Graph_ADD, 1, UI_Color_Yellow, 3, 1600, 800, 1600, 820);//右下
}
void Update_Fire_Motor_Speed(uint16_t LF, uint16_t RF, uint16_t LB, uint16_t RB)
{
    if (LF < 4000)  Rectangle_Draw(&Fire_Motor_Speed[0], "105", UI_Graph_Change, 1, UI_Color_Yellow, 3, 1500, 860, 1500, 860);//左上
    else            Rectangle_Draw(&Fire_Motor_Speed[0], "105", UI_Graph_Change, 1, UI_Color_Yellow, 16, 1590 - 45*((LF-4000.0f)/1000.0f), 860, 1590, 860);//左上
    
    if (RF < 4000)  Rectangle_Draw(&Fire_Motor_Speed[1], "106", UI_Graph_Change, 1, UI_Color_Yellow, 3, 1600, 860, 1600, 860);//右上
    else            Rectangle_Draw(&Fire_Motor_Speed[1], "106", UI_Graph_Change, 1, UI_Color_Yellow, 16, 1600, 860, 1600 + 45*((RF-4000.0f)/1000.0f), 860);//右上
    
    if (LB < 4000)  Rectangle_Draw(&Fire_Motor_Speed[2], "107", UI_Graph_Change, 1, UI_Color_Yellow, 3, 1500, 810, 1500, 810);//左下
    else            Rectangle_Draw(&Fire_Motor_Speed[2], "107", UI_Graph_Change, 1, UI_Color_Yellow, 16, 1590 - 45*((LB-4000.0f)/1000.0f), 810, 1590, 810);//左下
    
    if (RB < 4000)  Rectangle_Draw(&Fire_Motor_Speed[3], "108", UI_Graph_Change, 1, UI_Color_Yellow, 3, 1600, 810, 1600, 810);//右下
    else            Rectangle_Draw(&Fire_Motor_Speed[3], "108", UI_Graph_Change, 1, UI_Color_Yellow, 16, 1600, 810, 1600 + 45*((RB-4000.0f)/1000.0f), 810);//右下

//    My_Graph_Refresh(&Fire_Motor_Speed[0]);
//    My_Graph_Refresh(&Fire_Motor_Speed[1]);
//    My_Graph_Refresh(&Fire_Motor_Speed[2]);
//    My_Graph_Refresh(&Fire_Motor_Speed[3]);
}
/*图层1*/


/*图层2 超电*/
void Drow_Super_Power_BackGoroud(void)
{
        //12点钟位置为0°，顺时针增加
    char imagename[3];
    for (int i = 0; i < 11; i++)
    {
        uint16_t start_angle = 230 + 8*i;
        uint16_t end_angle = start_angle+1;
        uint16_t Rx = 380;
        uint16_t Ry = 380;
        uint16_t width = i%2 == 0? 25:12;
        sprintf(imagename,"2%.2d",i);
        Arc_Draw(&Super_Power_BackGoroud_Arc[i], imagename, UI_Graph_ADD, 2, UI_Color_White, start_angle, end_angle, width, 960, 540, Rx, Ry);
    }
}
void Drow_Super_Power(void)
{
    Arc_Draw(&Super_Power_Arc, "211", UI_Graph_ADD, 2, UI_Color_White, 230, 230, 20, 960, 540, 380, 380);
}

void Update_Super_Power(float num, uint8_t Super_Power_state)
{
    uint16_t end_angle = 231 + num/10.0f*8.0f;
    if (Super_Power_state == 1)
        Arc_Draw(&Super_Power_Arc, "211", UI_Graph_Change, 2, UI_Color_Orange, 230, end_angle, 20, 961, 541, 380, 380);
    else
        Arc_Draw(&Super_Power_Arc, "211", UI_Graph_Change, 2, UI_Color_Green, 230, end_angle, 20, 961, 541, 380, 380);
//    My_Graph_Refresh(&Super_Power_Arc);
}
/*图层2*/


/*图层3 自瞄框*/
Graph_Data  Left_Circle,Right_Circle,Left_Rect,Right_Rect,Mid_Rect;
void Drow_AimMode_BackGoroud(void)
{
    Rectangle_Draw(&Left_Rect, "301", UI_Graph_ADD, 3, UI_Color_White, 20, 900, 100, 940, 100);
    Rectangle_Draw(&Right_Rect, "303", UI_Graph_ADD, 3, UI_Color_White, 20, 980, 100, 1020, 100);
    Rectangle_Draw(&Mid_Rect, "302", UI_Graph_ADD, 3, UI_Color_White, 20, 940, 100, 980, 100);
}
void Drow_AimMode(void)
{
    Rectangle_Draw(&Left_Rect, "301", UI_Graph_ADD, 3, UI_Color_Yellow, 20, 900, 100, 940, 100);
}

    uint8_t debug = 0;

void Update_AimMode(void)
{
    if(debug == 0)//未识别 左侧点亮
    {
            Rectangle_Draw(&Left_Rect, "301", UI_Graph_Change, 3, UI_Color_Yellow, 20, 900, 100, 940, 100);
    Rectangle_Draw(&Right_Rect, "303", UI_Graph_Change, 3, UI_Color_White, 20, 980, 100, 1020, 100);
    Rectangle_Draw(&Mid_Rect, "302", UI_Graph_Change, 3, UI_Color_White, 20, 940, 100, 980, 100);  
    }
    else if(debug == 1)//已经识别 中间点亮
    {
            Rectangle_Draw(&Left_Rect, "301", UI_Graph_Change, 3, UI_Color_White, 20, 900, 100, 940, 100);
    Rectangle_Draw(&Right_Rect, "303", UI_Graph_Change, 3, UI_Color_White, 20, 980, 100, 1020, 100);
    Rectangle_Draw(&Mid_Rect, "302", UI_Graph_Change, 3, UI_Color_Green, 20, 940, 100, 980, 100);
        

    }
    else if(debug == 2)//开锁 右间点亮
    {
            Rectangle_Draw(&Left_Rect, "301", UI_Graph_Change, 3, UI_Color_White, 20, 900, 100, 940, 100);
    Rectangle_Draw(&Right_Rect, "303", UI_Graph_Change, 3, UI_Color_Orange, 20, 980, 100, 1020, 100);
    Rectangle_Draw(&Mid_Rect, "302", UI_Graph_Change, 3, UI_Color_White, 20, 940, 100, 980, 100);
        

    }
    My_Graph_Refresh(&Mid_Rect);
    My_Graph_Refresh(&Left_Rect);
    My_Graph_Refresh(&Right_Rect);
//    if(Get_AUTO_LOCK_Flag() == 0)//未识别 左侧点亮
//    {
//        Circle_Draw(&Left_Circle, "300", UI_Graph_ADD, 3, UI_Color_White, 10, 1400, 430, 5);
//        Rectangle_Draw(&Left_Rect, "300", UI_Graph_ADD, 3, UI_Color_White, 20, 1400, 430, 1400, 430);    
//    }
//    else if(Get_gimbal_open_auto() == 0 && Get_AUTO_LOCK_Flag() == 1)//已经识别 中间点亮
//    {
//        Rectangle_Draw(&Mid_Rect, "300", UI_Graph_ADD, 3, UI_Color_White, 20, 1400, 430, 1400, 430);
//        My_Graph_Refresh(&mid_rectangle);//初始框
//    }
//    else if(Get_gimbal_open_auto() == 1 && Get_AUTO_LOCK_Flag() == 1)//开锁 右间点亮
//    {
//        Rectangle_Draw(&Right_Rect, "300", UI_Graph_ADD, 3, UI_Color_White, 20, 1400, 430, 1400, 430);
//        Circle_Draw(&Right_Circle, "300", UI_Graph_ADD, 3, UI_Color_White, 10, 1400, 430, 5);
//    }
}

/*图层4 自瞄框*/
Graph_Data  Diff_Angle_Circle,Diff_Angle_Arc;
void Drow_Gimbal_Chassis_Angle_BackGoroud(void)
{
    Circle_Draw(&Diff_Angle_Circle, "400", UI_Graph_ADD, 4, UI_Color_White, 5, 960, 540, 340);
}
    uint8_t range = 30;
void Drow_Gimbal_Chassis_Angle(void)
{

    uint32_t start_angle = loop_fp32_constrain(0 -range -180,0,360);
    uint32_t end_angle = loop_fp32_constrain(0 +range -180,0,360);
    Arc_Draw(&Diff_Angle_Arc,"401",UI_Graph_ADD , 4, UI_Color_Main ,start_angle,end_angle,5,960,540,340,340);
}
void Update_Gimbal_Chassis_Angle(void)
{
    fp32 dif_Angle = Get_dif_angle() - 180;
    uint32_t start_angle = loop_fp32_constrain(dif_Angle -range -180,0,360);
    uint32_t end_angle = loop_fp32_constrain(dif_Angle +range -180,0,360);
    Arc_Draw(&Diff_Angle_Arc,"401",UI_Graph_Change , 4, UI_Color_Main ,start_angle,end_angle,5,960,540,340,340);
//    My_Graph_Refresh(&Diff_Angle_Arc);
}

/*图层5 摩擦轮设定rpm*/
String_Data Fire_Motor_Set_Rpm;
void Drow_Fire_Motor_Set_Rpm(void)
{
    Char_Draw(&Fire_Motor_Set_Rpm, "500", UI_Graph_ADD, 5, UI_Color_White, 3, 20, 10, 750, "NULL");
}
void Update_Fire_Motor_Set_Rpm(int16_t Rpm)
{
    char Fire_Motor_Rpm[4];
    sprintf(Fire_Motor_Rpm, "%d", Rpm);
    Char_Draw(&Fire_Motor_Set_Rpm, "500", UI_Graph_Change, 5, UI_Color_White, 3, 20, 10, 750, Fire_Motor_Rpm);
//    My_Char_Refresh(Fire_Motor_Set_Rpm);

}
/*图层6 歪把子线*/
uint32_t YYJ_Start_x[7] = {900,1020,920,920,920,920,920};
uint32_t YYJ_Start_y[7] = {300,300,460,430,400,370,340};
uint32_t YYJ_End_x[7] =   {900,1020,1000,1000,1000,1000,1000};
uint32_t YYJ_End_y[7] =   {800,800,460,430,400,370,340};
Graph_Data line_1,line_2,line_3,line_4,line_5,line_6,line_7;
void Drow_All(void)
{
    vTaskDelay(10);
    My_Char_Refresh(Fire_Motor_Set_Rpm);
    vTaskDelay(10);
    My_Char_Refresh(Fire_Motor_Speed_Num);
    vTaskDelay(10);
    My_Char_Refresh(Pitch_Motor_Angle_Num);
    vTaskDelay(10);
    My_Graph_Refresh_seven(&Motor_Angle_BackGoroud_Arc[0] , &Motor_Angle_BackGoroud_Arc[1], &Motor_Angle_BackGoroud_Arc[2], &Motor_Angle_BackGoroud_Arc[3], &Motor_Angle_BackGoroud_Arc[4], &Motor_Angle_BackGoroud_Arc[5], &Motor_Angle_BackGoroud_Arc[6]);
    vTaskDelay(10);
    My_Graph_Refresh_seven(&Motor_Angle_BackGoroud_Arc[7] , &Motor_Angle_BackGoroud_Arc[8], &Fire_Motor_Speed_BackGoroud_Rect[0], &Fire_Motor_Speed_BackGoroud_Rect[1], &Fire_Motor_Speed_BackGoroud_Rect[2], &Fire_Motor_Speed_BackGoroud_Rect[3], &Pitch_Motor_Angle_Arc);
    vTaskDelay(10);
    My_Graph_Refresh_seven(&Super_Power_BackGoroud_Arc[0] , &Super_Power_BackGoroud_Arc[1], &Super_Power_BackGoroud_Arc[2], &Super_Power_BackGoroud_Arc[3], &Super_Power_BackGoroud_Arc[4], &Super_Power_BackGoroud_Arc[5], &Super_Power_BackGoroud_Arc[6]);
    vTaskDelay(10);
    My_Graph_Refresh_seven(&Super_Power_BackGoroud_Arc[7] , &Super_Power_BackGoroud_Arc[8], &Super_Power_BackGoroud_Arc[9], &Super_Power_BackGoroud_Arc[10], &Super_Power_BackGoroud_Arc[11], &Fire_Motor_Speed[0], &Fire_Motor_Speed[1]);
    vTaskDelay(10);
        My_Graph_Refresh_seven(&Fire_Motor_Speed[2] , &Fire_Motor_Speed[3], &Super_Power_Arc, &Diff_Angle_Arc, &Diff_Angle_Arc, &Diff_Angle_Arc, &Diff_Angle_Circle);

//    My_Graph_Refresh_seven(&Fire_Motor_Speed[2] , &Fire_Motor_Speed[3], &Super_Power_Arc, &Mid_Rect, &Left_Rect, &Right_Rect, &Diff_Angle_Circle);
    vTaskDelay(10);
    My_Graph_Refresh(&Diff_Angle_Arc);
    
    Line_Draw(&line_1,"601",UI_Graph_ADD,6,UI_Color_Green,2,YYJ_Start_x[0],YYJ_Start_y[0],YYJ_End_x[0],YYJ_End_y[0]);
    Line_Draw(&line_2,"602",UI_Graph_ADD,6,UI_Color_Green,2,YYJ_Start_x[1],YYJ_Start_y[1],YYJ_End_x[1],YYJ_End_y[1]);
    Line_Draw(&line_3,"603",UI_Graph_ADD,6,UI_Color_Yellow,2,YYJ_Start_x[2],YYJ_Start_y[2],YYJ_End_x[2],YYJ_End_y[2]);
    Line_Draw(&line_4,"604",UI_Graph_ADD,6,UI_Color_Orange,2,YYJ_Start_x[3],YYJ_Start_y[3],YYJ_End_x[3],YYJ_End_y[3]);
    Line_Draw(&line_5,"605",UI_Graph_ADD,6,UI_Color_Cyan,2,YYJ_Start_x[4],YYJ_Start_y[4],YYJ_End_x[4],YYJ_End_y[4]);
    Line_Draw(&line_6,"606",UI_Graph_ADD,6,UI_Color_Pink,2,YYJ_Start_x[5],YYJ_Start_y[5],YYJ_End_x[5],YYJ_End_y[5]);
    Line_Draw(&line_7,"607",UI_Graph_ADD,6,UI_Color_Green,2,YYJ_Start_x[6],YYJ_Start_y[6],YYJ_End_x[6],YYJ_End_y[6]);
    //My_Graph_Refresh(&line_1);
    vTaskDelay(10);
    My_Graph_Refresh_seven(&line_1 , &line_2, &line_3, &line_4, &line_5, &line_6, &line_7);
//        My_Graph_Refresh(&Pitch_Motor_Mode_Rect);
    
}
void Update_All(void)
{
    
//        My_Graph_Refresh(&Mid_Rect);
//    My_Graph_Refresh(&Left_Rect);
//    My_Graph_Refresh(&Right_Rect);
;
        
        My_Char_Refresh(Pitch_Motor_Angle_Num);
        My_Char_Refresh(Fire_Motor_Set_Rpm);
        My_Graph_Refresh_seven(&Pitch_Motor_Angle_Arc , &Fire_Motor_Speed[0], &Fire_Motor_Speed[1], &Fire_Motor_Speed[2], &Fire_Motor_Speed[3], &Super_Power_Arc, &Diff_Angle_Arc);

}

//void Drow_Super_Power(void)
//{
//    Arc_Draw(&Super_Power_BackGoroud_Arc, "002", UI_Graph_ADD, 1, UI_Color_Green, 315, 315, 12, 960, 540, 400, 400);
//}
//void Update_Super_Power(float Power_of_100)
//{
//    Power_of_100 = Power_of_100 / 100.0f * 90.0f;//刻度范围90格
//    uint16_t start_angle = 315;
//    start_angle -= Power_of_100;
//    Arc_Draw(&Super_Power_BackGoroud_Arc, "001", UI_Graph_Change, 1, UI_Color_Main, start_angle, 315, 12, 960, 540, 400, 400);
//}
static void UI_Init(void);
static void print_chassis_mode(void);
static void print_fric_state(void);
static void print_cap_power_control(void);
static void print_now_aimmode(void);
static void print_chassis_gimbal_angle(void);
static void print_control_mode(void);
static void print_pitch_and_fire_info(void);
static void print_fire_encoder(void);
//指示状态
extern uint8_t id;
extern uint8_t fric_onoff;
extern uint8_t gimbal_mode;


uint16_t LF = 4000;
uint16_t RF = 4000;
uint16_t LB = 4000;
uint16_t RB = 4000;
    uint8_t time = 0;
void Referee_Task(void const *argument)
{

    uint16_t cnt=200;
    CH_P = get_chassis_point();
//    UI_Init();
    while (1)
    {
        //建议不要使用临界区，因为函数包含HAL_Delay
       // taskENTER_CRITICAL(); //进入临界区


        if(time ==10) 
        {
            Drow_Pitch_Motor_Angle_BackGoroud();
            Drow_Pitch_Motor_Angle();
            Drow_Fire_Motor_Speed_BackGoroud();
            Drow_Fire_Motor_Speed();
            Drow_Super_Power_BackGoroud();
            Drow_Super_Power();
            Drow_AimMode_BackGoroud();
            Drow_AimMode();
            Drow_Gimbal_Chassis_Angle_BackGoroud();
            Drow_Gimbal_Chassis_Angle();
            
            Drow_Fire_Motor_Set_Rpm();
            Drow_All();


//            UI_Init();
            time=0;
        }
        else//打印刷新
        {
            Update_Pitch_Motor_Angle();
            Update_Fire_Motor_Speed(Get_Gimbal_Fire_Motor_Speed(0), Get_Gimbal_Fire_Motor_Speed(1), Get_Gimbal_Fire_Motor_Speed(2), Get_Gimbal_Fire_Motor_Speed(3));
            Update_Super_Power(Chassis.pm01_od->energy, Chassis.Robot_cmd->Power_Mode);
            Update_AimMode();
            Update_Gimbal_Chassis_Angle();
            Update_Fire_Motor_Set_Rpm(Get_Fire_Motor_Set_Rpm());
            
            
            Update_All();
//            print_cap_power_control();
//            print_chassis_mode();
//            print_now_aimmode();
//            print_fric_state();
//            print_control_mode();
//            print_pitch_and_fire_info();
//            print_fire_encoder();
            time++;
        }
     //   taskEXIT_CRITICAL(); //退出临界区
		vTaskDelay(1);
    }
}

int16_t X_1 = 680;
int16_t Y_1 = 480;
int16_t X_2 = 1240;
int16_t Y_2 = 750;//600

//x横坐标
//y纵坐标
uint32_t Start_x[7] = {960,920,920,920,920,920,920};
uint32_t Start_y[7] = {600,460,440,420,400,380,360};
uint32_t End_x[7] =   {960,1000,1000,1000,1000,1000,1000};
uint32_t End_y[7] =   {480,460,440,420,400,380,360};



char* ChassisMode[] = {"No_FOLLOW","FOLLOW","SPIN"};

static void UI_Init(void)
{
    //底盘模式
    Char_Draw(&chassis_mode_set, "000", UI_Graph_ADD, 1, UI_Color_Purplish_red, 3, 30, 10, 890, "Warnnig");
    My_Char_Refresh(chassis_mode_set);
    //超电
    Char_Draw(&Print_Cap, "010", UI_Graph_ADD, 2, UI_Color_Green, 3, 30, 10, 850, "Cap:");
    My_Char_Refresh(Print_Cap);
    //瓦数 控制模式
    Char_Draw(&Cap_ENERGE, "080", UI_Graph_ADD, 3, UI_Color_Green, 3, 30, 110, 850, "NULL");
    My_Char_Refresh(Cap_ENERGE);
    
    Char_Draw(&Print_Control, "020", UI_Graph_ADD, 4, UI_Color_Green, 3, 30, 10, 800, "Ctrl:");
    My_Char_Refresh(Print_Control);
//    
//    Char_Draw(&Print_Control, "020", UI_Graph_ADD, 1, UI_Color_Green, 3, 30, 10, 850, "Control:");
//    My_Char_Refresh(Print_Control);
    //摩擦轮状态
    Char_Draw(&fric_state,"122",UI_Graph_ADD,5,UI_Color_Purplish_red,3,20,10,750,"Fire:");
    My_Char_Refresh(fric_state);
    //自瞄显示框
    Rectangle_Draw(&mid_rectangle,"101",UI_Graph_ADD,6,UI_Color_White,2,X_1,Y_1,X_2,Y_2);
    My_Graph_Refresh(&mid_rectangle);//初始框
    
    Char_Draw(&Print_auto_fire_info,"030",UI_Graph_ADD,7,UI_Color_Green,3,20,400,550,"PITCH\nFire_Delay:");
    My_Char_Refresh(Print_auto_fire_info);
    
    Float_Draw(&Fire_motor_encoder,"040",UI_Graph_ADD,8,UI_Color_Main,20,0,2,1600,750, (-get_fire_motor_encoder()*1000)/60);
    My_Graph_Refresh((Graph_Data *)&Fire_motor_encoder);
    //枪管中线
    Line_Draw(&line_1,"061",UI_Graph_ADD,9,UI_Color_Green,2,Start_x[0],Start_y[0],End_x[0],End_y[0]);
    Line_Draw(&line_2,"062",UI_Graph_ADD,9,UI_Color_Green,2,Start_x[1],Start_y[1],End_x[1],End_y[1]);
    Line_Draw(&line_3,"063",UI_Graph_ADD,9,UI_Color_Yellow,2,Start_x[2],Start_y[2],End_x[2],End_y[2]);
    Line_Draw(&line_4,"064",UI_Graph_ADD,9,UI_Color_Orange,2,Start_x[3],Start_y[3],End_x[3],End_y[3]);
    Line_Draw(&line_5,"065",UI_Graph_ADD,9,UI_Color_Cyan,2,Start_x[4],Start_y[4],End_x[4],End_y[4]);
    Line_Draw(&line_6,"066",UI_Graph_ADD,9,UI_Color_Pink,2,Start_x[5],Start_y[5],End_x[5],End_y[5]);
    Line_Draw(&line_7,"067",UI_Graph_ADD,9,UI_Color_Green,2,Start_x[6],Start_y[6],End_x[6],End_y[6]);
    //My_Graph_Refresh(&line_1);
    My_Graph_Refresh_seven(&line_1 , &line_2, &line_3, &line_4, &line_5, &line_6, &line_7);
    
//        //圆心坐标，绘制圆
//        Circle_Draw(&circle,"222",UI_Graph_ADD,5,UI_Color_Green,3,CIRCLE_X0,CIRCLE_Y0,CIRCLE_R0);
//        My_Graph_Refresh(&circle);
//        //画线//test use
//        Line_Draw(&line_1,"023",UI_Graph_ADD,6,UI_Color_Cyan,2,600,800,1000,1400);
//        My_Graph_Refresh(&line_1);
//        //画线
//        Line_Draw(&line_0,"066",UI_Graph_ADD,6,UI_Color_Cyan,2,CIRCLE_R0*sin_calculate(loop_fp32_constrain((cc_ctrl.chassis_yaw_diff_angle-THETA),-180,180))+CIRCLE_X0,CIRCLE_R0*cos_calculate(loop_fp32_constrain((cc_ctrl.chassis_yaw_diff_angle-THETA),-180,180))+CIRCLE_Y0,CIRCLE_R0*sin_calculate(loop_fp32_constrain((cc_ctrl.chassis_yaw_diff_angle+THETA),-180,180))+CIRCLE_X0,CIRCLE_R0*sin_calculate(loop_fp32_constrain((cc_ctrl.chassis_yaw_diff_angle+THETA),-180,180))+CIRCLE_Y0);
//        My_Graph_Refresh(&line_0);
        		/*--------------------底盘云台差角ui加入--------------------*/
		//以中点为圆心，云台方向为前，距离200为半径，画底盘灯条方位、
        

//        uint32_t start_angle = Chassis.Robot_cmd->Difference_Angle_between_Chassis_Gimbal;
//		uint32_t end_angle = Chassis.Robot_cmd->Difference_Angle_between_Chassis_Gimbal;
//        Arc_Draw(&diff_ang,"003",UI_Graph_ADD ,6 ,UI_Color_Main ,start_angle,end_angle,5,CIRCLE_X0,CIRCLE_Y0,CIRCLE_R0,CIRCLE_R0);
//		My_Graph_Refresh(&diff_ang);
}

/**
  * @brief      超电电压容量
  */
uint8_t max_power ;
uint8_t connum;
char print_out[10];
static void print_cap_power_control(void)
{

    int32_t energe = Chassis.pm01_od->energy;
    sprintf(print_out,"%.3d%% %.3dw",energe,Chassis.Max_Power);

    Char_Draw(&Cap_ENERGE, "080", UI_Graph_Change, 3, UI_Color_Green, 3, 30, 110, 850, print_out);
    My_Char_Refresh(Cap_ENERGE);
}
int64_t x = 10;
int64_t y = 880;
static void print_control_mode(void)
{
    if (Get_Chassis_Control_State() == Gimbal_TC)
    {
        Char_Draw(&Print_Control, "020", UI_Graph_Change, 4, UI_Color_Green, 3, 30, 10, 800, "Ctrl:T");
        My_Char_Refresh(Print_Control);
    }else
    {
        Char_Draw(&Print_Control, "020", UI_Graph_Change, 4, UI_Color_Green, 3, 30, 10, 800, "Ctrl:R");
        My_Char_Refresh(Print_Control);
    }

}
//打印当前底盘模式

static void print_chassis_mode(void)
{
    switch(Chassis.Robot_cmd->Chassis_Mode)
    {
        case NO_FOLLOW:
            Char_Draw(&chassis_mode_set,"000",UI_Graph_Change,1,UI_Color_Yellow,4,20,10,890,"NO_FOLLOW");
        break;
        case FOLLOW:
            Char_Draw(&chassis_mode_set,"000",UI_Graph_Change,1,UI_Color_Orange,4,20,10,890,"FOLLOW");
        break;
        case SPIN:
            Char_Draw(&chassis_mode_set,"000",UI_Graph_Change,1,UI_Color_Pink,4,20,10,890,"SPIN");
        break;
        default:
            Char_Draw(&chassis_mode_set,"000",UI_Graph_Change,1,UI_Color_Purplish_red,4,20,10,890,"WARNING");
        break;
    }
    My_Char_Refresh(chassis_mode_set);
}

//打印当前摩擦轮状态
static void print_fric_state(void)
{
    if(Get_fire_ready() == 1) 
        Char_Draw(&fric_state,"122",UI_Graph_Change,5,UI_Color_Green,3,20,10,750,"Fire:ON");
    else  
        Char_Draw(&fric_state,"122",UI_Graph_Change,5,UI_Color_Purplish_red,3,20,10,750,"Fire:OFF");
    My_Char_Refresh(fric_state);
}
//打印手瞄自瞄状态
static void print_now_aimmode(void)
{
    if(Get_gimbal_open_auto() == 0 && Get_AUTO_LOCK_Flag() == 0)//手动且自瞄未锁到
    {
        //这框
        Rectangle_Draw(&mid_rectangle,"101",UI_Graph_Change,6,UI_Color_White,2,X_1,Y_1,X_2,Y_2);
        My_Graph_Refresh(&mid_rectangle);//初始框
    }
    else if(Get_gimbal_open_auto() == 0 && Get_AUTO_LOCK_Flag() == 1)//手动但已经识别
    {
        //这框
        Rectangle_Draw(&mid_rectangle,"101",UI_Graph_Change,6,UI_Color_Green,2,X_1,Y_1,X_2,Y_2);
        My_Graph_Refresh(&mid_rectangle);//初始框
    }
    else if(Get_gimbal_open_auto() == 1 && Get_AUTO_LOCK_Flag() == 1)//开锁
    {
        //这框
        Rectangle_Draw(&mid_rectangle,"101",UI_Graph_Change,6,UI_Color_Purplish_red,2,X_1,Y_1,X_2,Y_2);
        My_Graph_Refresh(&mid_rectangle);//初始框
    }
    else if((Get_gimbal_open_auto() == 1) && (Get_AUTO_LOCK_Flag() == 0))//锁个寂寞
    {
         Rectangle_Draw(&mid_rectangle,"101",UI_Graph_Change,6,UI_Color_Black,2,X_1,Y_1,X_2,Y_2);
         My_Graph_Refresh(&mid_rectangle);//初始框
    }
    else if(Get_AUTO_LOCK_Flag() == 2)//TOF准备就绪
    {
        Rectangle_Draw(&mid_rectangle,"101",UI_Graph_Change,6,UI_Color_Yellow,2,X_1,Y_1,X_2,Y_2);
        My_Graph_Refresh(&mid_rectangle);//初始框
    }
}
char auto_fire_info[30];    
int16_t auto_x  = 600;
int16_t auto_y  = 600;

static void print_pitch_and_fire_info(void)
{
    sprintf(auto_fire_info,"PITCH:%.1f\nFire:%.3f\n%.2f\n%.3f",Get_Pitch_compensate(),Get_Fire_auto_delay(),Get_Gimbal_Pitch_Angle(),Get_TOF_Distance());

    Char_Draw(&Print_auto_fire_info, "030", UI_Graph_Change, 7,UI_Color_Green,3,20,1600,900, auto_fire_info);
    My_Char_Refresh(Print_auto_fire_info);
}
/**
  * @brief      地盘云台角度显示
  */
static void print_chassis_gimbal_angle(void)
{
//        float temp_angle=(float)(cc_ctrl.chassis_yaw_diff_angle-180.0f);
		uint32_t start_angle = Chassis.Robot_cmd->Difference_Angle_between_Chassis_Gimbal;
		uint32_t end_angle = Chassis.Robot_cmd->Difference_Angle_between_Chassis_Gimbal;
		Arc_Draw(&diff_ang,"003",UI_Graph_Change ,6 ,UI_Color_Main ,start_angle,end_angle,5,CIRCLE_X0,CIRCLE_Y0,CIRCLE_R0,CIRCLE_R0);
		My_Graph_Refresh(&diff_ang);
}
void print_fire_encoder(void)
{
        Float_Draw(&Fire_motor_encoder,"040",UI_Graph_Change,8,UI_Color_Main,20,0,2,1600,750, (-get_fire_motor_encoder()*1000)/60);
        My_Graph_Refresh((Graph_Data *)&Fire_motor_encoder);
}
/*

*/