/*
@brief: 弹道解算 适配彭淐的VisionPro
@author: PC 东莞理工学院ACE战队 请关注我的Github:Pengchang
*/
// 近点只考虑水平方向的空气阻力

// TODO 完整弹道模型

#include <math.h>
// #include "arm_math.h"
#include <stdio.h>
#include "bsp_dwt.h"
#include "maths.h"
#include "Ou_VisionPro.h"
#include "gimbal_struct_variables.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "imu_task.h"
#include "robot_cmd.h"

#define judge(a, b)   \
    {                 \
        a > b ? a : b \
    }

float erro_temp;
extern gimbal_auto_control_t *auto_control_p;
tar_pos tar_position[4]; // 最多只有四块装甲板
tar_pos Wait_position;   // 前哨站等板位置
float t = 0;
float bullet_speed = 19.0f;

int number[4] = {0, 1, 2, 3};

float min(float a, float b)
{
    return a > b ? b : a;
}
float PC_Calc_Pitch_Angle(float Y_, float X_, float *Fly_time, float speed_bullet);
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
    // t为给定v与angle时的飞行时间
    t = (float)((exp(auto_control_p->k * s) - 1) / (auto_control_p->k * v * cos(angle)));
    // z为给定v与angle时的高度
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
        dz = 0.3 * (z - z_actual);
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
// float completeAirResistanceModel(gimbal_auto_control_t *auto_control_p)
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
// float fly_time;
uint8_t Shake_Yaw_Fire = 0;

float max_close = 0.2f;
float max_yaw_close = 1.0f;
float max_pitch_close = 1.0f;
Recursive_ave_filter_type_t Vyaw_Speed_filter; // Vyaw滑窗,当Vyaw长时间过快时切换定头打法

uint8_t in_armor_range = 0;
tar_pos last_Target;

uint8_t Shake_Fire = 1; // 摇头跟板打法
uint8_t Vyaw_filtet_init_flag = 0;
fp32 pitch_compensate = 0;

float ARMOR_RANGE = 30.0f;
float OUTPOST_ARMOR_RANGE = 23.0f;
float debounce_threshold = 5.5f;
float last_xw = 0, last_yw = 0, last_zw = 0, last_yaw = 0; // ROS坐标系下的x,y,z
float Get_Lost_outpost_time(gimbal_auto_control_t *auto_control_p)
{
    float lost_time = DWT_GetTimeline_ms() - auto_control_p->last_receive_time_ms;
    auto_control_p->xw = last_xw;
    auto_control_p->yw = last_yw;
    auto_control_p->zw = last_zw;
    auto_control_p->yaw = last_yaw;
    return lost_time;
}
/**
 *@parm receive_virtual_flag 0代表视觉丢失，此时前哨站进入自预测模式
 */
float timeDelay = 0;
uint8_t Tracking_control = 0;
fp32 YawAngleToRotaCenter;
float Wait_Track_Yaw;
uint8_t try_wait;

void autoSolveTrajectory(gimbal_auto_control_t *auto_control_p, uint8_t receive_virtual_flag)
{
    // 延迟击打计算部分
    // PC_Calc_Pitch_Angle(auto_control_p->zw,sqrtf(auto_control_p->xw*auto_control_p->xw+auto_control_p->yw*auto_control_p->yw)-0.2765f,&auto_control_p->fly_time_ms,auto_control_p->bullet_speed);

    if (Vyaw_filtet_init_flag == 0)
    {
        Vyaw_filtet_init_flag = 1;
        Recursive_ave_filter_init(&Vyaw_Speed_filter);
    }

    // 取样最近30次,决定是否使用摇头打法
    Shake_Yaw_Fire = Recursive_ave_filter(&Vyaw_Speed_filter, abs(auto_control_p->v_yaw), 30) < 4.0f ? 1 : 0;

    // 线性预测
    timeDelay = t; // 计算发出发射指令到命中装甲板的时间
    // auto_control_p->auto_fire_delay_time_ms = auto_control_p->bias_time + t*1000;
    auto_control_p->fly_time_ms = t * 1000;

    if (receive_virtual_flag == 0) // 丢失目标，根据历史位置推算
        timeDelay += Get_Lost_outpost_time(auto_control_p) / 1000.0f;
    else if (auto_control_p->armor_num == ARMOR_NUM_OUTPOST)
    { // 保存已知位置
        auto_control_p->Get_out_post = 1;
        auto_control_p->last_receive_time_ms = DWT_GetTimeline_ms();
        last_yaw = auto_control_p->yaw;
        last_xw = auto_control_p->xw;
        last_yw = auto_control_p->yw;
        last_zw = auto_control_p->zw;
    }
    else
        auto_control_p->Get_out_post = 0;

    // 计算装甲板的位置；
    // 装甲板id顺序，以四块装甲板为例，逆时针编号
    //       2
    //    3     1
    //       0
    int use_1 = 1;
    int i = 0, n = 0, tem = 0;
    int idx = 0; // 选择的装甲板

    if (auto_control_p->armor_id == ARMOR_BASE) // BASE基地
    {
        auto_control_p->Get_out_post = 0;
        float tmp_yaw = auto_control_p->yaw; // 2/3PI
        float tmp_r = (auto_control_p->r1 + auto_control_p->r2) / 2;
        auto_control_p->Target_Position.x = auto_control_p->xw - tmp_r * cos(tmp_yaw);
        auto_control_p->Target_Position.y = auto_control_p->yw - tmp_r * sin(tmp_yaw);
        auto_control_p->Target_Position.z = auto_control_p->zw;
        auto_control_p->Target_Position.distance = sqrtf(auto_control_p->Target_Position.x * auto_control_p->Target_Position.x + auto_control_p->Target_Position.y * auto_control_p->Target_Position.y);

        auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(auto_control_p->Target_Position.distance, auto_control_p->Target_Position.z + auto_control_p->z_bias, auto_control_p->bullet_speed) * 180.f / PI) + auto_control_p->pitch_compensate;
        auto_control_p->auto_yaw = (float)(atan2(auto_control_p->Target_Position.y, auto_control_p->Target_Position.x) * 180.0f / PI); // 把设定的yaw轴转换到imu设定值
    }
    else if (auto_control_p->armor_num == ARMOR_NUM_BALANCE) // 发现当前的兵种是平衡步兵
    {
        auto_control_p->Get_out_post = 0;
        auto_control_p->yaw += auto_control_p->v_yaw * (t + auto_control_p->bias_time / 1000.0f);
        for (i = 0; i < 2; i++) // 计算
        {
            float tmp_yaw = auto_control_p->yaw + i * PI;
            float r = auto_control_p->r1;
            tar_position[i].x = auto_control_p->xw - r * cos(tmp_yaw);
            tar_position[i].y = auto_control_p->yw - r * sin(tmp_yaw);
            tar_position[i].z = auto_control_p->zw;
            tar_position[i].yaw = tmp_yaw; // 计算两块装甲板和当前yaw轴的距离
        }
        float yaw_diff_min = fabsf(auto_control_p->auto_yaw - tar_position[0].yaw);

        // 因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(auto_control_p->auto_yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
        auto_control_p->Target_Position.x = tar_position[idx].x + auto_control_p->vxw * timeDelay;
        auto_control_p->Target_Position.y = tar_position[idx].y + auto_control_p->vyw * timeDelay;
        auto_control_p->Target_Position.z = tar_position[idx].z + auto_control_p->vzw * timeDelay;
        auto_control_p->Target_Position.distance = sqrtf(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y);

        auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(auto_control_p->Target_Position.distance, tar_position[idx].z + auto_control_p->z_bias, auto_control_p->bullet_speed) * 180.f / PI) + auto_control_p->pitch_compensate;
        auto_control_p->auto_yaw = (float)(atan2(auto_control_p->Target_Position.y, auto_control_p->Target_Position.x) * 180.0f / PI); // 把设定的yaw轴转换到imu设定值

        // 选0 1//找最小的打
    }
    else if (auto_control_p->armor_num == ARMOR_NUM_OUTPOST) // 发现当前是前哨战,定头打法
    {
        float r = 0.2765; // 前哨站半径
        // ojh:位置估算是准的，速度估算可能错的，操作手给个按钮选择预测速度
        // 因此一下转速阈值化作废
        // 转速阈值化
        if (auto_control_p->v_yaw > 1.0f)
            auto_control_p->v_yaw = 2.51327408; // PI*0.8
        else if (auto_control_p->v_yaw < -1.0f)
            auto_control_p->v_yaw = -2.51327408;
        else
            auto_control_p->v_yaw = 0;

        auto_control_p->yaw += auto_control_p->v_yaw * timeDelay; // 计算yaw轴需要旋转的角度

        for (i = 0; i < 3; i++)
        {
            float tmp_yaw = auto_control_p->yaw + i * 2.0 * PI / 3.0; // 2/3PI

            tar_position[i].x = auto_control_p->xw - r * cos(tmp_yaw);
            tar_position[i].y = auto_control_p->yw - r * sin(tmp_yaw);
            tar_position[i].z = auto_control_p->zw;
            tar_position[i].distance = sqrtf(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
            tar_position[i].yaw = tmp_yaw; // 同理：计算三块装甲板设定距离
        }

        // 选出距离最近的板子作为击打板
        float min_distance = tar_position[0].distance;
        uint8_t min_distance_idx = 0;
        for (int i = 1; i < 3; i++)
        {
            if (tar_position[i].distance < min_distance)
            {
                min_distance = tar_position[i].distance;
                min_distance_idx = i;
            }
        }

        // 减去 指向前哨站旋转中心时的陀螺仪Yaw值，得到三个<板子朝向> 与 <云台到前哨站连线> 的夹角大小
        YawAngleToRotaCenter = (float)(atan2(auto_control_p->yw, auto_control_p->xw) * 180.0f / PI); // 把设定的yaw轴转换到imu设定值

        tar_position[0].angle_yaw = auto_control_p->yaw * 180.f / PI;
        tar_position[0].angle_yaw -= YawAngleToRotaCenter;
        tar_position[1].angle_yaw = tar_position[0].angle_yaw + 120;
        tar_position[2].angle_yaw = tar_position[0].angle_yaw - 120;

        float tar_yaw_close = abs(tar_position[min_distance_idx].angle_yaw);
        // 根据转动方向选择等待点
        Wait_Track_Yaw = YawAngleToRotaCenter + auto_control_p->v_yaw > 0 ? (OUTPOST_ARMOR_RANGE) : (-OUTPOST_ARMOR_RANGE);
        Wait_position.x = auto_control_p->xw - r * cos(Wait_Track_Yaw);
        Wait_position.y = auto_control_p->yw - r * sin(Wait_Track_Yaw);
        Wait_position.z = auto_control_p->zw;
        Wait_position.distance = sqrtf(Wait_position.x * Wait_position.x + Wait_position.y * Wait_position.y);

        if (try_wait)
        {
            auto_control_p->Target_Position.x = Wait_position.x;
            auto_control_p->Target_Position.y = Wait_position.y;
            auto_control_p->Target_Position.z = Wait_position.z;
            auto_control_p->Target_Position.distance = Wait_position.distance;
        }
        else
        {
            if (tar_yaw_close < OUTPOST_ARMOR_RANGE) // 跟踪区间
            {
                auto_control_p->Target_Position.x = tar_position[min_distance_idx].x + auto_control_p->vxw * timeDelay;
                auto_control_p->Target_Position.y = tar_position[min_distance_idx].y + auto_control_p->vyw * timeDelay;
                auto_control_p->Target_Position.z = tar_position[min_distance_idx].z + auto_control_p->vzw * timeDelay;
                auto_control_p->Target_Position.distance = sqrtf(auto_control_p->Target_Position.x * auto_control_p->Target_Position.x + auto_control_p->Target_Position.y * auto_control_p->Target_Position.y);
            }
            else // 等板来
            {
                auto_control_p->Target_Position.x = Wait_position.x;
                auto_control_p->Target_Position.y = Wait_position.y;
                auto_control_p->Target_Position.z = Wait_position.z;
                auto_control_p->Target_Position.distance = Wait_position.distance;
            }
        }
        /*
                static uint8_t in_armor_range = 0;
                if (tar_yaw_close < OUTPOST_ARMOR_RANGE) {//消抖阈值
                    if (in_armor_range == 0) {
                        in_armor_range = 1;
                        last_Target.x = tar_position[min_distance_idx].x;
                        last_Target.y = tar_position[min_distance_idx].y;
                        last_Target.z = tar_position[min_distance_idx].z;
                        last_Target.distance = tar_position[min_distance_idx].distance;
                    }
                }else if (tar_yaw_close > OUTPOST_ARMOR_RANGE + debounce_threshold) {
                    in_armor_range = 0;

                }
                if (last_Target.x == 0 && last_Target.y == 0 && last_Target.z == 0) {
                    last_Target.x = tar_position[min_distance_idx].x;
                    last_Target.y = tar_position[min_distance_idx].y;
                    last_Target.z = tar_position[min_distance_idx].z;
                    in_armor_range = 1;
                }
                if (tar_yaw_close > OUTPOST_ARMOR_RANGE && abs(auto_control_p->v_yaw) < 1.0f) {
                    in_armor_range = 1;
                }

                if (in_armor_range == 0) {
                    auto_control_p->Target_Position.x = last_Target.x + auto_control_p->vxw * timeDelay;
                    auto_control_p->Target_Position.y = last_Target.y + auto_control_p->vyw * timeDelay;
                    auto_control_p->Target_Position.z = last_Target.z + auto_control_p->vzw * timeDelay;
                    auto_control_p->Target_Position.distance = sqrtf(auto_control_p->Target_Position.x * auto_control_p->Target_Position.x + auto_control_p->Target_Position.y * auto_control_p->Target_Position.y);
                }else {
                    auto_control_p->Target_Position.x = tar_position[min_distance_idx].x + auto_control_p->vxw * timeDelay;
                    auto_control_p->Target_Position.y = tar_position[min_distance_idx].y + auto_control_p->vyw * timeDelay;
                    auto_control_p->Target_Position.z = tar_position[min_distance_idx].z + auto_control_p->vzw * timeDelay;
                    auto_control_p->Target_Position.distance = sqrtf(auto_control_p->Target_Position.x * auto_control_p->Target_Position.x + auto_control_p->Target_Position.y * auto_control_p->Target_Position.y);
                }

                auto_control_p->auto_pitch = (float)(pitchTrajectoryCompensation(auto_control_p->Target_Position.distance, auto_control_p->zw + auto_control_p->z_bias, auto_control_p->bullet_speed )* 180.f / PI) + auto_control_p->pitch_compensate;
                auto_control_p->auto_yaw = (float)(atan2(auto_control_p->Target_Position.y,auto_control_p->Target_Position.x)*180.0f/PI) + auto_control_p->yaw_compensate;//把设定的yaw轴转换到imu设定值
                */
        // 判断开火条件
        if (in_armor_range == 1 && (abs(auto_control_p->auto_yaw - auto_control_p->Imu_c->Yaw) < max_yaw_close) && (abs(auto_control_p->auto_pitch - auto_control_p->Imu_c->Pitch) < max_pitch_close))
            auto_control_p->fire_flag = 1;
        else
            auto_control_p->fire_flag = 0;
    }

    else // 当前为普通步兵
    {
        Tracking_control = 0;
        uint8_t fire_control = 0;

        auto_control_p->yaw += auto_control_p->v_yaw * (t + auto_control_p->bias_time / 1000.0f); // 装甲板yaw朝向经过飞行时间旋转的角度

        for (i = 0; i < 4; i++)
        {
            tar_position[i].idx = i;
            float tmp_yaw = auto_control_p->yaw + i * PI / 2.0;
            float r = use_1 ? auto_control_p->r1 : auto_control_p->r2;
            tar_position[i].x = auto_control_p->xw - r * cos(tmp_yaw);
            tar_position[i].y = auto_control_p->yw - r * sin(tmp_yaw);
            tar_position[i].z = use_1 ? auto_control_p->zw : auto_control_p->zw + auto_control_p->dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }
        tar_position[0].angle_yaw = auto_control_p->yaw * 180.f / PI;
        tar_position[0].angle_yaw -= auto_control_p->Imu_c->Yaw;
        tar_position[1].angle_yaw = tar_position[0].angle_yaw + 90;
        tar_position[2].angle_yaw = tar_position[0].angle_yaw + 180 > 180 ? 180 - tar_position[0].angle_yaw : tar_position[0].angle_yaw + 180;
        tar_position[3].angle_yaw = tar_position[0].angle_yaw - 90;
        // 摇头打法
        for (int i = 0; i < 4; i++)
        {
            if (abs(tar_position[i].angle_yaw) < ARMOR_RANGE) // 改角度在这里改//选距离枪管角度＜30的
            {
                idx = i;
                Tracking_control = 1;
                fire_control = 1;
                break;
            }
        }
        // 定头打法
        Shake_Fire = 0;
        if (!Shake_Fire && Tracking_control)
        {
            float temp_r = tar_position[idx].idx % 2 == 0 ? auto_control_p->r1 : auto_control_p->r2;
            // reg_test=temp_r*100;
            tar_position[idx].x = auto_control_p->xw - temp_r * cos(auto_control_p->Imu_c->Yaw / 180.f * PI);
            tar_position[idx].y = auto_control_p->yw - temp_r * sin(auto_control_p->Imu_c->Yaw / 180.f * PI);
            uint8_t range = 0;
            if (auto_control_p->armor_id == 1)
                range = 7;
            else
                range = 5;
            if (abs(tar_position[idx].angle_yaw) <= range)
                fire_control = 1;
            else
            {
                Tracking_control = 0;
                fire_control = 0;
            }
        }

        uint8_t isClose = 0;
        float x_temp = (tar_position[idx].x + auto_control_p->vxw * timeDelay);
        float y_temp = (tar_position[idx].y + auto_control_p->vyw * timeDelay);
        float x_camera = x_temp * cos(auto_control_p->Imu_c->Yaw / 360.f * PI) + y_temp * sin(auto_control_p->Imu_c->Yaw / 360.f * PI);
        float y_camera = y_temp * cos(auto_control_p->Imu_c->Yaw / 360.f * PI) - x_temp * sin(auto_control_p->Imu_c->Yaw / 360.f * PI);
        if (sqrtf(x_temp * x_temp + y_temp * y_temp) < 0.20f || (x_camera <= 0 && y_camera <= 0) || (Tracking_control == 0 && auto_control_p->armor_num != ARMOR_NUM_OUTPOST && auto_control_p->armor_num != ARMOR_NUM_BALANCE))
            isClose = 1;
        else
        {
            auto_control_p->Target_Position.x = tar_position[idx].x + auto_control_p->vxw * timeDelay;
            auto_control_p->Target_Position.y = tar_position[idx].y + auto_control_p->vyw * timeDelay;
            auto_control_p->Target_Position.z = tar_position[idx].z + auto_control_p->vzw * timeDelay;
            auto_control_p->Target_Position.distance = sqrtf(tar_position[idx].x * tar_position[idx].x + tar_position[idx].y * tar_position[idx].y);
        }

        float sqrt1 = sqrtf((auto_control_p->Target_Position.x) * (auto_control_p->Target_Position.x) + (auto_control_p->Target_Position.y) * (auto_control_p->Target_Position.y));
        if (!isClose)
        {
            auto_control_p->auto_pitch = (float)((pitchTrajectoryCompensation(sqrt1, auto_control_p->Target_Position.z, auto_control_p->bullet_speed) * 180.f / PI) + auto_control_p->pitch_compensate);
            auto_control_p->auto_yaw = (float)((atan2(auto_control_p->Target_Position.y, auto_control_p->Target_Position.x) * 180.0f / PI)); // 把设定的yaw轴转换到imu设定值//右- 左+
        }
        else
        {
            fire_control = 0;
        }

        auto_control_p->fire_flag = fire_control;
    }
}
// float min_float(float a,float b){return a>b? b:a;}

// void autoSolveTrajectory(gimbal_auto_control_t *auto_control_p)
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

// int PC_VisionPro_Init()
//{
//     float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
//     float pitch = 0; //输出控制量 pitch绝对角度 弧度
//     float yaw = 0;   //输出控制量 yaw绝对角度 弧度

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

float root;
float cos_Y_X(float _Y, float _X)
{
    if (_X == 0 && _Y == 0)
        return 0;

    float s = _X * _X + _Y * _Y;
    root = sqrtf(s);
    // arm_sqrt_f32(X*X+Y*Y,&root);
    return _X / root;
}
float sin_Y_X(float _Y, float _X)
{
    if (_X == 0 && _Y == 0)
        return 0;
    double s = _X * _X + _Y * _Y;
    root = sqrtf(s);
    // arm_sqrt_f32(X*X+Y*Y,&root);
    return _Y / root;
}
/**
 *@brief 向前迭代求仰角
 *@param Y 竖直距离
 *@param X 水平距离
 */
#define Vo 15    //  m/s
#define k_1 0.1f // CpS/2m
#define G 9.8f
#define e 2.71828183f // 自然数e
float abs_f(float NUM) { return NUM >= 0 ? NUM : -NUM; }

float PC_Calc_Pitch_Angle(float Y_, float X_, float *Fly_time, float speed_bullet)
{
    if (Y_ == 0 || X_ == 0)
        return 0;
    uint8_t calc_time = 0;
    float Temp_cos_Y_X = cos_Y_X(Y_, X_); // 首次迭代仰角指向前哨站
    float Temp_sin_Y_X = sin_Y_X(Y_, X_); // 计算sin
    float V_x = Temp_cos_Y_X * speed_bullet;
    float V_y = Temp_sin_Y_X * speed_bullet;
    float Angle = acos(Temp_cos_Y_X) * 57.3;

    float t = (pow(e, k_1 * X_) - 1) / (k_1 * V_x); // 计算飞行时间
    float Temp_Y = V_y * t - 0.5 * G * t * t;       // 不考虑空气阻力的匀减速运动

    float Y_error = Temp_Y - Y_; // 弹道偏高为正
    float try_change = 1;

    while (abs_f(Y_error) > 0.01f) // 误差过大向前迭代,根据实际经验判断，<45°的情况下，仰角 up，落点Y up
    {
        calc_time++;
        if (calc_time > 20)
        {
            *Fly_time = 0;
            return Angle; // 计算迭代轮数上限50次
        }

        static float last_Y_error = 0;
        if (Y_error < 0) // 落点低，仰角增加
        {
            if (last_Y_error > 0)
                try_change /= 2; // 上一次运算为落点过高，但这次确实落点过低，证明仰角减太多了，因此增加一半上次减少的仰角

            Angle += try_change;

            Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X * speed_bullet;
            V_y = Temp_sin_Y_X * speed_bullet;

            t = (pow(e, k_1 * X_) - 1) / (k_1 * V_x);
            Temp_Y = V_y * t - 0.5 * G * t * t; // 不考虑空气阻力的匀减速运动

            last_Y_error = Y_error; // 更新落点Y值
            Y_error = Temp_Y - Y_;
        }
        else // 落点高，仰角减小
        {
            if (last_Y_error < 0)
                try_change /= 2; // 上一次运算为落点过低，但这次确实落点过高，证明仰角加太多了，因此减小一半上次增加的仰角

            Angle -= try_change;

            Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X * speed_bullet;
            V_y = Temp_sin_Y_X * speed_bullet;

            t = (pow(e, k_1 * X_) - 1) / (k_1 * V_x);
            Temp_Y = V_y * t - 0.5 * G * t * t; // 不考虑空气阻力的匀减速运动

            last_Y_error = Y_error; // 更新落点Y值
            Y_error = Temp_Y - Y_;
        }
    }
    *Fly_time = t; // 成功拟合弹道，返回飞行时间
    return Angle;
}
