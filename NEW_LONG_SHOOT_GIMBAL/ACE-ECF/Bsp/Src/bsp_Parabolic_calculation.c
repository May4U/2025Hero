#include <struct_typedef.h>
#include "bsp_Parabolic_calculation.h"
//#include "arm_math.h"
#include "maths.h"
#include "bsp_dwt.h"

#define m 0.041f//kg
#define e 2.71828183f//自然数e
#define Vo 15//  m/s
#define G 9.8f
#ifndef PI
  #define PI               3.14159265358979f
#endif



#if  PLAN == PLAN_A
    #define k 0.001f//空气阻力系数
    /**
      *@brief 计算42mm弹丸飞行时间
      *@param Distance  水平距离
      *@param cos       仰角余弦值
      */
    float fly_time(float Distance, float cos)
    {
        float time = m/(k*Vo*cos)*(pow(e,k/m*Distance)-1);
        return time;
    }
#elif PLAN == PLAN_B
    #define k_1 0.1f// CpS/2m
#endif
double abs_d(double NUM)
{
    return NUM>=0?NUM:-NUM;
}
float abs_f(float NUM)
{
    return NUM>=0?NUM:-NUM;
}
/*
    Z /| Y
    /__|
      X
*/
float s;
float cos_Y_X(float Y, float X)
{
    if (X == 0 && Y == 0) return 0;
    float root;
    s = X*X+Y*Y;
    root = sqrtf(s);
    //arm_sqrt_f32(X*X+Y*Y,&root);
    return X/root;
}
double root;
float sin_Y_X(float Y, float X)
{
    if (X == 0 && Y == 0) return 0;
    double s = X*X+Y*Y;
    root = sqrt(s);
    //arm_sqrt_f32(X*X+Y*Y,&root);
    return Y/root;
}
/**
  *@brief 向前迭代求仰角
  *@param Y 竖直距离
  *@param X 水平距离
  */
float Calc_Pitch_Angle(float Y_, float X_, float *Fly_time)
{
    if (Y_ == 0 || X_ == 0) return 0;
    #if PLAN == PLAN_A
    float Temp_cos_Y_X = cos_Y_X(Y_, X_);//首次迭代仰角指向前哨站
    float Temp_sin_Y_X;
    arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X, &Temp_sin_Y_X);//计算sin
    
                                    //ln||内分子
    float Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                   /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));
                                    //ln||内分母
    float Y_error = Temp_Y - Y_;//弹道偏高为正
    float try_change = 1;
    while(abs_f(Y_error) > 0.01f)//误差过大向前迭代,根据实际经验判断，<45°的情况下，仰角 up，落点Y up
    {
        static float last_Y_error = 0;
        if(Y_error < 0)//落点低，仰角增加
        {
            if (last_Y_error>0)    try_change /= 2;//上一次运算为落点过高，但这次确实落点过低，证明仰角减太多了，因此增加一半上次减少的仰角

            Temp_cos_Y_X+=try_change;
            arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X ,&Temp_sin_Y_X);//计算sin
            Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                     /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));

        }
        else//落点高，仰角减小
        {
            if (last_Y_error<0)    try_change /= 2;//上一次运算为落点过低，但这次确实落点过高，证明仰角加太多了，因此减小一半上次增加的仰角
            Temp_cos_Y_X-=try_change;
            arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X ,&Temp_sin_Y_X);//计算sin
            Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                     /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));
        }
        //更新落点Y值
        last_Y_error = Y_error;
        Y_error = Temp_Y - Y_;
    }
    return acos(Temp_cos_Y_X);
    #elif PLAN == PLAN_B
    uint8_t calc_time = 0;
	float Temp_cos_Y_X = cos_Y_X(Y_, X_);//首次迭代仰角指向前哨站
	float Temp_sin_Y_X = sin_Y_X(Y_, X_);//计算sin
	float V_x = Temp_cos_Y_X*Vo;
	float V_y = Temp_sin_Y_X*Vo;
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
            V_x = Temp_cos_Y_X*Vo;
			V_y = Temp_sin_Y_X*Vo;
            
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
            V_x = Temp_cos_Y_X*Vo;
			V_y = Temp_sin_Y_X*Vo;
            
			t = (pow(e,k_1*X_)-1)/(k_1*V_x);
			Temp_Y = V_y*t-0.5*G*t*t;//不考虑空气阻力的匀减速运动
			
			last_Y_error = Y_error;			//更新落点Y值
			Y_error = Temp_Y - Y_;
		}
	}
    *Fly_time = t;//成功拟合弹道，返回飞行时间
	return Angle;
    #endif
}

/**
  *@brief YAW与前哨站偏转的角度
  *@param X 水平X轴距离,镜头坐标系右为正
  *@param Y 水平Y轴距离,镜头坐标系前为正
  *@note  逆时针正
  */
/*
    Z /| Y
    /__|
      X
*/
float Yaw_Angle_close(float X_, float Y_)
{
    float Z;
    Z = sqrt(X_*X_+Y_*Y_);
    //arm_sqrt_f32(X_*X_+Y_*Y_, &Z);//直角斜边
    float cos = (X_*X_+Z*Z-Y_*Y_)/2*X_*Z;//余弦定理
    float Angle_close = acos(cos);//余弦值求弧度
    Angle_close = Angle_close*180/PI;//弧度转角度
    return Angle_close;
}
/**
  *@brief 计算前哨站装甲版旋转到英雄与前哨站中心连线的时间
  */
#define FIRE_DELAY_MS  200.0f//待测
#define ROTATE_TIME_MS 325.0f//833.3333f 靶车13秒10圈 0.77rad
//10m/s 5m远 1m高 飞行时间0.7s 加上FIRE_DELAY_MS拨弹盘延迟，已超过
/**
  *@parm 单位ms
  */
float calc_fire_time(float now_in_place_time, float fly_time)
{
    if (fly_time == 0)  return 0;
    int64_t increat_time = ROTATE_TIME_MS;
    float next_in_place_time = now_in_place_time + ROTATE_TIME_MS;//下一次转到中心的时间戳
    while ((FIRE_DELAY_MS + fly_time) > increat_time)
    {   //发弹时间＋飞行时间超过下一块装甲板转到位的时间，因此应该预判下下块装甲板到位时间
        next_in_place_time += ROTATE_TIME_MS;
        increat_time += ROTATE_TIME_MS;
    }
    float fire_time = next_in_place_time-FIRE_DELAY_MS-fly_time;
    return  fire_time;
}
/**
  *@brief 判断是否符合开火实际
  *@note  如果符合清零预计下一次到位时间
  *@reval 0 发火时机未到 | 1 发火时机已到 | 2 发火时机已过 | 3 传参错误
  */
float fla = 20;
int64_t Fire_delay = 200;
uint8_t JudgeCanFire(float now_in_place_time, float *next_in_place_time)
{
    if (now_in_place_time == 0) return 3;
    if (*next_in_place_time == 0) return 3;
    if (now_in_place_time > *next_in_place_time)
    {
        *next_in_place_time = 0;
        return 2;
    }        
    if (abs_f(now_in_place_time - *next_in_place_time) < fla)
    {
        *next_in_place_time = 0;
        return 1;    
    }                                                       
    return 0;
}

fp32 Calc_The_Distance(float X_,float Y_)
{
    if (X_ == 0 && Y_ == 0) return 0;
    float Z;
    Z = sqrt(X_*X_+Y_*Y_);
    //arm_sqrt_f32(X_*X_+Y_*Y_,&Z);
    return Z;
}
float in_place_time[2];


