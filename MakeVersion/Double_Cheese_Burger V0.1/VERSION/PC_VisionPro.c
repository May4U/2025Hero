/*
@brief: 弹道解算 适配彭淐的VisionPro
@author: PC 东莞理工学院ACE战队 请关注我的Github:Pengchang
*/
// 近点只考虑水平方向的空气阻力

//TODO 完整弹道模型

#include <math.h>
//#include "arm_math.h"
#include <stdio.h>

#include "maths.h"
#include "PC_VisionPro.h"
#include "gimbal_struct_variables.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "imu_task.h"
#include "robot_cmd.h"

#define judge(a, b) {a>b ? a:b }

float erro_temp;
extern gimbal_auto_control_t *auto_control_p;
tar_pos tar_position[4]; //最多只有四块装甲板

float t=0;
float bullet_speed = 19.0f;

int number[4] = {0,1,2,3};

float min(float a,float b)
{
    return a > b ? b : a;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(auto_control_p->k * s) - 1) / (auto_control_p->k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - auto_control_p->g * t * t / 2);
    return z;
}


/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}
/*
@brief 完整弹道模型
@param s:m 距离
@return z:m
*/
//float completeAirResistanceModel(gimbal_auto_control_t *auto_control_p)
////{
//	{
//    // TODO:根据陀螺仪安装位置调整距离求解方式
//    // 降维，坐标系Y轴以垂直向上为正方向
//    float dist_vertical =  auto_control_p->Target_Position.z; //- 0.1f;//目标与实际垂直距离差值
//    float vertical_tmp = dist_vertical;
//    float dist_horizonal = sqrt(auto_control_p->Target_Position.x * auto_control_p->Target_Position.x + auto_control_p->Target_Position.y * auto_control_p->Target_Position.y);//目标与实际水平距离差值
//    // auto dist_vertical = xyz[2];
//    // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
//    float pitch = atan(dist_vertical / dist_horizonal) * 180 / PI;//得到当前pitch轴差的角度
//    float pitch_new = pitch;//设定更新后设定的pitch轴
//    // auto pitch_offset = 0.0;
//    // 开始使用龙格库塔法求解弹道补偿
//    for (int i = 0; i < auto_control_p->max_iter; i++)
//    {
//        // TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
//        // 初始化
//        float x = 0.0;
//        float y = 0.0;
//        float p = tan(pitch_new / 180 * PI);
//        float v = auto_control_p->bullet_speed;//设定弹速
//        float u = v / sqrt(1 + pow(p, 2));
//        float delta_x = dist_horizonal / auto_control_p->R_K_iter;
//        int j = 0;
//        for(j = 0; j < auto_control_p->R_K_iter; j++)
//        {
//            float k1_u = -auto_control_p->k * u * sqrt(1 + pow(p, 2));
//            float k1_p = -auto_control_p->g / pow(u, 2);
//            float k1_u_sum = u + k1_u * (delta_x / 2);
//            float k1_p_sum = p + k1_p * (delta_x / 2);

//            float k2_u = -auto_control_p->k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
//            float k2_p = -auto_control_p->g / pow(k1_u_sum, 2);
//            float k2_u_sum = u + k2_u * (delta_x / 2);
//            float k2_p_sum = p + k2_p * (delta_x / 2);

//            float k3_u = -auto_control_p->k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
//            float k3_p = -auto_control_p->g / pow(k2_u_sum, 2);
//            float k3_u_sum = u + k3_u * (delta_x / 2);
//            float k3_p_sum = p + k3_p * (delta_x / 2);

//            float k4_u = -auto_control_p->k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
//            float k4_p = -auto_control_p->g / pow(k3_u_sum, 2);

//            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
//            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

//            x += delta_x;
//            y += p * delta_x;
//        }
//        // 评估迭代结果,若小于迭代精度需求则停止迭代
//        float error = dist_vertical - y;
//         erro_temp=error;
//        if (abs(error) <= auto_control_p->stop_error)
//        {
//           
//            break;
//        }
//        else
//        {
//            vertical_tmp += error;
//            // xyz_tmp[1] -= error;
//            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / PI;
//        }
//        
//    }
//    return pitch_new;// - pitch;
////	}

//}

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
//float fly_time;

float max_close = 0.2f;

Recursive_ave_filter_type_t Vyaw_Speed_filter;//Vyaw滑窗,当Vyaw长时间过快时切换定头打法

uint8_t Shake_Fire  = 1;//摇头跟板打法
uint8_t Vyaw_filtet_init_flag = 0;
void autoSolveTrajectory(gimbal_auto_control_t *auto_control_p)
{
    if (Vyaw_filtet_init_flag == 0)
    {
        Vyaw_filtet_init_flag = 1;
        Recursive_ave_filter_init(&Vyaw_Speed_filter);
    }

    //取样最近30次,决定是否使用摇头打法
    Shake_Fire = Recursive_ave_filter(&Vyaw_Speed_filter, abs(auto_control_p->v_yaw), 30) < 4.0f? 1 : 0;
    
        
    // 线性预测
    //PC_Calc_Pitch_Angle(auto_control_p->zw,sqrtf(auto_control_p->xw*auto_control_p->xw+auto_control_p->yw*auto_control_p->yw),&fly_time,auto_control_p->bullet_speed);
            
    float timeDelay = auto_control_p->bias_time/1000.0+t;//计算发出发射指令到命中装甲板的时间
    auto_control_p->yaw += auto_control_p->v_yaw * timeDelay;//计算yaw轴需要旋转的角度
    //计算装甲板的位置；
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0,n = 0,tem = 0;
    int idx = 0; // 选择的装甲板
    
    
    if (auto_control_p->armor_num ==	ARMOR_NUM_BALANCE)//发现当前的兵种是平衡步兵
    {
             for (i = 0; i<2; i++)//计算
            {
                     float tmp_yaw = auto_control_p->yaw + i * PI;
                     float r = auto_control_p->r1;
                     tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
                     tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
                     tar_position[i].z = auto_control_p->zw;
                     tar_position[i].yaw = tmp_yaw;//计算两块装甲板和当前yaw轴的距离
            }
            float yaw_diff_min = fabsf(auto_control_p->auto_yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(auto_control_p->auto_yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
        //选0 1//找最小的打
    }
    
    else if (auto_control_p->armor_num ==	 ARMOR_NUM_OUTPOST)//发现当前是前哨战
    {
         for (i = 0; i<3; i++) {
            float tmp_yaw = auto_control_p->yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (auto_control_p->r1 + auto_control_p->r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
            tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
            tar_position[i].z = auto_control_p->zw;
            tar_position[i].yaw = tmp_yaw;//同理：计算三块装甲板设定距离
        }
    }
		
    else //当前为普通步兵
    {
        for(i = 0;i < 4; i++)//计算旋转后的
        {
            float tmp_yaw = auto_control_p->yaw + i * PI/2.0; //角度转弧度
            float r =  use_1 ?  auto_control_p->r1: auto_control_p->r2; 
            //float r =  use_1 ? 0.235fauto_control_p->r1 : auto_control_p->r2;   
            tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
            tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? auto_control_p->zw : auto_control_p->zw + auto_control_p->dz;
            tar_position[i].yaw = tmp_yaw;
            tar_position[i].angle_yaw = tar_position[i].yaw * 180.f / PI ;//弧度转角度
                        use_1 = !use_1;
        }
                    
        tar_position[0].angle_yaw -= auto_control_p->Imu_c->Yaw;
        tar_position[1].angle_yaw = tar_position[0].angle_yaw + 90;
        tar_position[3].angle_yaw = tar_position[0].angle_yaw - 90;
        //摇头打法
        if (Shake_Fire == 1)
        {
            if(min(abs(tar_position[1].angle_yaw),abs(tar_position[3].angle_yaw)) <= 45 && abs(tar_position[0].angle_yaw) <= 45){//两块装甲板都45°面对自己时强制0号板防止跳变
                idx = 0;//0号板角度＞45° 且存在有板＜45°                                                                                                
            }
            else if(!(abs(tar_position[0].angle_yaw) <= 45 ) && min(abs(tar_position[1].angle_yaw),abs(tar_position[3].angle_yaw)) <= 45){   
                if(abs(tar_position[1].angle_yaw) < abs(tar_position[3].angle_yaw))
                    idx = 1;
                else
                    idx = 3;
            }
            else{ //锁中心
                idx = 0;
            }
            auto_control_p->Target_Position.z = tar_position[idx].z + auto_control_p->vzw * timeDelay;
            auto_control_p->Target_Position.x = tar_position[idx].x + auto_control_p->vxw * timeDelay;
            auto_control_p->Target_Position.y = tar_position[idx].y + auto_control_p->vyw * timeDelay;	
            auto_control_p->Target_Position.distance = sqrtf(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y);
   
            float sqrt1 = sqrtf((auto_control_p->Target_Position.x) * (auto_control_p->Target_Position.x) + (auto_control_p->Target_Position.y) * (auto_control_p->Target_Position.y)) - auto_control_p->s_bias;
            auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(sqrt1,auto_control_p->Target_Position.z + auto_control_p->z_bias,auto_control_p->bullet_speed )* 180.f / PI);
            auto_control_p->auto_yaw = (float)(atan2(auto_control_p->Target_Position.y,auto_control_p->Target_Position.x)*180.0f/PI);//把设定的yaw轴转换到imu设定值
            //判断开火条件
            if (abs(auto_control_p->auto_yaw - auto_control_p->Imu_c->Yaw) < max_close) 
                auto_control_p->fire_flag = 1;
            else  
                auto_control_p->fire_flag = 0;
                
        }
        else if (Shake_Fire == 0)//定头打法
        {
            float close_r = 0;
            float max_distance = 0;
            uint8_t idx = 0;
            float tar_yaw[4];
            float tar_yaw_close[4];
            for (int i = 0; i < 4; i++)
            {
                tar_position[i].z = tar_position[i].z + auto_control_p->vzw * timeDelay;
                tar_position[i].x = tar_position[i].x + auto_control_p->vxw * timeDelay;
                tar_position[i].y = tar_position[i].y + auto_control_p->vyw * timeDelay;	
                tar_position[i].distance = sqrtf(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);

                if (tar_position[i].distance > max_distance)//选出背后板的序号
                {
                    max_distance = tar_position[i].distance;
                    idx = i;
                    if (i == 0||i == 2) close_r = auto_control_p->r1;//背后板与面前板半径相同
                    else                close_r = auto_control_p->r2;
                }
            }
            float Rota_center_x = auto_control_p->xw;
            float Rota_center_y = auto_control_p->yw;
            //旋转中心-面前板到旋转中心半径，得到当 面前板旋转到 枪口与旋转中心连线时的水平距离
            float distance = sqrtf(Rota_center_x*Rota_center_x + Rota_center_y*Rota_center_y) - close_r;
            
            auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(distance, tar_position[idx].z + auto_control_p->z_bias, auto_control_p->bullet_speed )* 180.f / PI) - 3.0f;
            auto_control_p->auto_yaw = (float)(atan2(Rota_center_y,Rota_center_x)*180.0f/PI);//把设定的yaw轴转换到imu设定值
            uint8_t fire_control = 0;
            
            //判断是否有板到连线中心，即正对枪口
            int i;
            for (i = 0; i < 4; i++)
            {
                if (i == idx)   continue;//跳过距离最远的背后板
                tar_yaw[i] = (float)(atan2(tar_position[i].y,tar_position[i].x)*180.0f/PI);
                tar_yaw_close[i] = abs(tar_yaw[i] - auto_control_p->Imu_c->Yaw);
                if (abs(tar_yaw[i] - auto_control_p->auto_yaw) < max_close)
                {
                    fire_control = 1;
                    break;
                }
            }
            auto_control_p->fire_flag = fire_control;
            
            if (Get_Gimbal_CMD_point()->rc_ctl->mouse.press_l == 1 && Get_Gimbal_CMD_point()->Gimbal_Work_State == AUTO_ATTACK && fire_control == 1 && get_fire_ready()==1)//图床允许自瞄发射，校准发弹延迟
                Flash_fire_bias_time(0);//实现更新发弹延迟
            //发送的是预判的那块装甲板位置，实际上预瞄的是最佳击打位置
            auto_control_p->Target_Position.z = tar_position[i].z;
            auto_control_p->Target_Position.x = tar_position[i].x;
            auto_control_p->Target_Position.y = tar_position[i].y;	
            auto_control_p->Target_Position.distance = tar_position[i].distance;
        }
}
//float min_float(float a,float b){return a>b? b:a;}

//void autoSolveTrajectory(gimbal_auto_control_t *auto_control_p)
//{
////    auto_control_p->bias_time =1000;//TODO:修改拨蛋盘旋转到发射出去的时间
//    auto_control_p->k = 0.042;
//    // 线性预测
//    float timeDelay = auto_control_p->bias_time/1000.0+t;//计算发出发射指令到命中装甲板的时间
//    auto_control_p->yaw += auto_control_p->v_yaw * timeDelay;//计算yaw轴需要旋转的角度

//    //计算四块装甲板的位置；
//    //装甲板id顺序，以四块装甲板为例，逆时针编号
//    //      2
//    //   3     1
//    //      0
//	int use_1 = 1;
//	int i = 0;
////    int n = 0,tem = 0;
//    int idx = 0; // 选择的装甲板
//    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
//		if (auto_control_p->armor_num ==	ARMOR_NUM_BALANCE)//发现当前的兵种是平衡步兵
//		{
//                for (i = 0; i<2; i++)//计算
//                {
//                         float tmp_yaw = auto_control_p->yaw + i * PI;
//                         float r = auto_control_p->r1;
//                         tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
//                         tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
//                         tar_position[i].z = auto_control_p->zw;
//                         tar_position[i].yaw = tmp_yaw;//计算两块装甲板和当前yaw轴的距离
//                }
//                float yaw_diff_min = fabsf(auto_control_p->auto_yaw - tar_position[0].yaw);

//            //因为是平衡步兵 只需判断两块装甲板即可
//            float temp_yaw_diff = fabsf(auto_control_p->auto_yaw - tar_position[1].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = 1;
//            }
//            //选0 1//找最小的打
//		}
//		else if (auto_control_p->armor_num ==	 ARMOR_NUM_OUTPOST)//发现当前是前哨战
//		{
//			for (i = 0; i<3; i++) 
//            {
//                float tmp_yaw = auto_control_p->yaw + i * 2.0 * PI/3.0;  // 2/3PI
//                float r =  (auto_control_p->r1 + auto_control_p->r2)/2;   //理论上r1=r2 这里取个平均值
//                tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
//                tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
//                tar_position[i].z = auto_control_p->zw;
//                tar_position[i].yaw = tmp_yaw;//同理：计算三块装甲板设定距离
//            }
//		}
//		
//		else //当前为普通步兵
//		{
//			for(i = 0;i < 4; i++)
//			{
//                float tmp_yaw = auto_control_p->yaw + i * PI/2.0f; 
//                float r =  use_1 ? auto_control_p->r1 : auto_control_p->r2;   
//                tar_position[i].x = auto_control_p->xw - r*cos(tmp_yaw);
//                tar_position[i].y = auto_control_p->yw - r*sin(tmp_yaw);
//                tar_position[i].z = use_1 ? auto_control_p->zw : auto_control_p->zw + auto_control_p->dz;
//                tar_position[i].yaw = tmp_yaw;
//                tar_position[i].angle_yaw = tmp_yaw * 180.f / PI ;
//                            use_1 = !use_1;
//			}
//        tar_position[0].angle_yaw-=auto_control_p->Imu_c->Yaw;
//        tar_position[1].angle_yaw = tar_position[0].angle_yaw + 90;
//        tar_position[3].angle_yaw = tar_position[0].angle_yaw - 90;
//        if(min_float(abs(tar_position[1].angle_yaw),abs(tar_position[3].angle_yaw)) <= 20 && abs(tar_position[0].angle_yaw) <= 20)
//        {
//            idx = 0;
//        }
//        else if(!(abs(tar_position[0].angle_yaw) <= 20) && min_float(abs(tar_position[1].angle_yaw),abs(tar_position[3].angle_yaw)) <= 20)
//        {
//            if(abs(tar_position[1].angle_yaw) < abs(tar_position[3].angle_yaw))
//				idx = 1;
//            else
//				idx = 3;
//        }
//        else
//        {
//            idx = 0;
//        }
//        
//	auto_control_p->Target_Position.z = tar_position[idx].z + auto_control_p->vzw * timeDelay;
//	auto_control_p->Target_Position.x = tar_position[idx].x + auto_control_p->vxw * timeDelay;
//	auto_control_p->Target_Position.y = tar_position[idx].y + auto_control_p->vyw * timeDelay;	
//	auto_control_p->Target_Position.distance = sqrtf(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y);
//   
//	//这里符号给错了
//	float sqrt1 = sqrtf((auto_control_p->Target_Position.x) * (auto_control_p->Target_Position.x) + (auto_control_p->Target_Position.y) * (auto_control_p->Target_Position.y)) - auto_control_p->s_bias;
//    auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(sqrt1,auto_control_p->Target_Position.z + auto_control_p->z_bias,auto_control_p->bullet_speed )* 180.f / PI);
//    auto_control_p->auto_yaw = (float)(atan2(auto_control_p->Target_Position.y,auto_control_p->Target_Position.x)*180.0f/PI);//把设定的yaw轴转换到imu设定值
//}
	
// 从坐标轴正向看向原点，逆时针方向为正

//int PC_VisionPro_Init()
//{
//    float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
//    float pitch = 0; //输出控制量 pitch绝对角度 弧度
//    float yaw = 0;   //输出控制量 yaw绝对角度 弧度

//		auto_control_p = get_auto_control_point();
//	
//    //定义参数
//    auto_control_p->k = 0.038;
//		auto_control_p.current_pitch = 0;
//    auto_control_p.current_yaw = 0;
//    auto_control_p->xw = 3.0;
//    // auto_control_p.yw = 0.0159;
//    auto_control_p->yw = 0;
//    // auto_control_p.zw = -0.2898;
//    auto_control_p->zw = 1.5;

//    auto_control_p->vxw = 0;
//    auto_control_p->vyw = 0;
//    auto_control_p->vzw = 0;
//    auto_control_p->v_yaw = 0;
//  auto_control_p.tar_yaw = 0.09131;
//    auto_control_p->r1 = 0.5;
//    auto_control_p->r2 = 0.5;
//    auto_control_p->dz = 0.1;
//    auto_control_p->bias_time = 100;
//    auto_control_p->s_bias = 0.0;
//    auto_control_p->z_bias = 0.0;
//    auto_control_p->armor_id = ARMOR_INFANTRY3;
//    auto_control_p->armor_num = ARMOR_NUM_NORMAL;


//    autoSolveTrajectory();

//    return 0;
//}

//}
}

float root;
float cos_Y_X(float _Y, float _X)
{
    if (_X == 0 && _Y == 0) return 0;

    float s = _X*_X+_Y*_Y;
    root = sqrtf(s);
    //arm_sqrt_f32(X*X+Y*Y,&root);
    return _X/root;
}
float sin_Y_X(float _Y, float _X)
{
    if (_X == 0 && _Y == 0) return 0;
    double s = _X*_X+_Y*_Y;
    root = sqrtf(s);
    //arm_sqrt_f32(X*X+Y*Y,&root);
    return _Y/root;
}
/**
  *@brief 向前迭代求仰角
  *@param Y 竖直距离
  *@param X 水平距离
  */
#define Vo 15//  m/s
#define k_1 0.1f// CpS/2m
#define G 9.8f
#define e 2.71828183f//自然数e
float abs_f(float NUM){return NUM>=0?NUM:-NUM;}

float PC_Calc_Pitch_Angle(float Y_, float X_, float *Fly_time, float speed_bullet)
{
    if (Y_ == 0 || X_ == 0) return 0;
    uint8_t calc_time = 0;
	float Temp_cos_Y_X = cos_Y_X(Y_, X_);//首次迭代仰角指向前哨站
	float Temp_sin_Y_X = sin_Y_X(Y_, X_);//计算sin
	float V_x = Temp_cos_Y_X*speed_bullet;
	float V_y = Temp_sin_Y_X*speed_bullet;
	float Angle = acos(Temp_cos_Y_X)*57.3;

	float t = (pow(e,k_1*X_)-1)/(k_1*V_x);//计算飞行时间
	float Temp_Y = V_y*t-0.5*G*t*t;//不考虑空气阻力的匀减速运动

	float Y_error = Temp_Y - Y_;//弹道偏高为正
	float try_change = 1;
	
	while(abs_f(Y_error) > 0.01f)//误差过大向前迭代,根据实际经验判断，<45°的情况下，仰角 up，落点Y up
	{
        calc_time++;
        if (calc_time > 20)
        {
            *Fly_time = 0;
            return Angle;//计算迭代轮数上限50次
        }            
        
		static float last_Y_error = 0;
		if(Y_error < 0)//落点低，仰角增加
		{
			if (last_Y_error>0)    try_change /= 2;//上一次运算为落点过高，但这次确实落点过低，证明仰角减太多了，因此增加一半上次减少的仰角
			
            Angle+=try_change;
            
			Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X*speed_bullet;
			V_y = Temp_sin_Y_X*speed_bullet;
            
			t = (pow(e,k_1*X_)-1)/(k_1*V_x);
			Temp_Y = V_y*t-0.5*G*t*t;//不考虑空气阻力的匀减速运动
			
			last_Y_error = Y_error;			//更新落点Y值
			Y_error = Temp_Y - Y_;

		}
		else//落点高，仰角减小
		{
			if (last_Y_error<0)    try_change /= 2;//上一次运算为落点过低，但这次确实落点过高，证明仰角加太多了，因此减小一半上次增加的仰角
			
			Angle-=try_change;
            
			Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X*speed_bullet;
			V_y = Temp_sin_Y_X*speed_bullet;
            
			t = (pow(e,k_1*X_)-1)/(k_1*V_x);
			Temp_Y = V_y*t-0.5*G*t*t;//不考虑空气阻力的匀减速运动
			
			last_Y_error = Y_error;			//更新落点Y值
			Y_error = Temp_Y - Y_;
		}
	}
    *Fly_time = t;//成功拟合弹道，返回飞行时间
	return Angle;
}
