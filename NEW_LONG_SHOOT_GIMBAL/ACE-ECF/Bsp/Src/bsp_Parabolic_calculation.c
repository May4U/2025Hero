#include <struct_typedef.h>
#include "bsp_Parabolic_calculation.h"
//#include "arm_math.h"
#include "maths.h"
#include "bsp_dwt.h"

#define m 0.041f//kg
#define e 2.71828183f//��Ȼ��e
#define Vo 15//  m/s
#define G 9.8f
#ifndef PI
  #define PI               3.14159265358979f
#endif



#if  PLAN == PLAN_A
    #define k 0.001f//��������ϵ��
    /**
      *@brief ����42mm�������ʱ��
      *@param Distance  ˮƽ����
      *@param cos       ��������ֵ
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
  *@brief ��ǰ����������
  *@param Y ��ֱ����
  *@param X ˮƽ����
  */
float Calc_Pitch_Angle(float Y_, float X_, float *Fly_time)
{
    if (Y_ == 0 || X_ == 0) return 0;
    #if PLAN == PLAN_A
    float Temp_cos_Y_X = cos_Y_X(Y_, X_);//�״ε�������ָ��ǰ��վ
    float Temp_sin_Y_X;
    arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X, &Temp_sin_Y_X);//����sin
    
                                    //ln||�ڷ���
    float Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                   /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));
                                    //ln||�ڷ�ĸ
    float Y_error = Temp_Y - Y_;//����ƫ��Ϊ��
    float try_change = 1;
    while(abs_f(Y_error) > 0.01f)//��������ǰ����,����ʵ�ʾ����жϣ�<45�������£����� up�����Y up
    {
        static float last_Y_error = 0;
        if(Y_error < 0)//���ͣ���������
        {
            if (last_Y_error>0)    try_change /= 2;//��һ������Ϊ�����ߣ������ȷʵ�����ͣ�֤�����Ǽ�̫���ˣ��������һ���ϴμ��ٵ�����

            Temp_cos_Y_X+=try_change;
            arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X ,&Temp_sin_Y_X);//����sin
            Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                     /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));

        }
        else//���ߣ����Ǽ�С
        {
            if (last_Y_error<0)    try_change /= 2;//��һ������Ϊ�����ͣ������ȷʵ�����ߣ�֤�����Ǽ�̫���ˣ���˼�Сһ���ϴ����ӵ�����
            Temp_cos_Y_X-=try_change;
            arm_sqrt_f32(1-Temp_cos_Y_X*Temp_cos_Y_X ,&Temp_sin_Y_X);//����sin
            Temp_Y = (m/k)*log(abs_d( cos( m/(k*Vo*Temp_cos_Y_X)*(pow(e, k/m*X_)-1)*sqrt(G*k/m)-atan(Vo*Temp_sin_Y_X*sqrt(k/m/G)))
                                     /cos( atan(Vo*Temp_sin_Y_X*sqrt(k/m/G))                                                    ) ));
        }
        //�������Yֵ
        last_Y_error = Y_error;
        Y_error = Temp_Y - Y_;
    }
    return acos(Temp_cos_Y_X);
    #elif PLAN == PLAN_B
    uint8_t calc_time = 0;
	float Temp_cos_Y_X = cos_Y_X(Y_, X_);//�״ε�������ָ��ǰ��վ
	float Temp_sin_Y_X = sin_Y_X(Y_, X_);//����sin
	float V_x = Temp_cos_Y_X*Vo;
	float V_y = Temp_sin_Y_X*Vo;
	float Angle = acos(Temp_cos_Y_X)*57.3;

	float t = (pow(e,k_1*X_)-1)/(k_1*V_x);//�������ʱ��
	float Temp_Y = V_y*t-0.5*G*t*t;//�����ǿ����������ȼ����˶�

	float Y_error = Temp_Y - Y_;//����ƫ��Ϊ��
	float try_change = 1;
	
	while(abs_f(Y_error) > 0.01f)//��������ǰ����,����ʵ�ʾ����жϣ�<45�������£����� up�����Y up
	{
        calc_time++;
        if (calc_time > 20)
        {
            *Fly_time = 0;
            return Angle;//���������������50��
        }            
        
		static float last_Y_error = 0;
		if(Y_error < 0)//���ͣ���������
		{
			if (last_Y_error>0)    try_change /= 2;//��һ������Ϊ�����ߣ������ȷʵ�����ͣ�֤�����Ǽ�̫���ˣ��������һ���ϴμ��ٵ�����
			
            Angle+=try_change;
            
			Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X*Vo;
			V_y = Temp_sin_Y_X*Vo;
            
			t = (pow(e,k_1*X_)-1)/(k_1*V_x);
			Temp_Y = V_y*t-0.5*G*t*t;//�����ǿ����������ȼ����˶�
			
			last_Y_error = Y_error;			//�������Yֵ
			Y_error = Temp_Y - Y_;

		}
		else//���ߣ����Ǽ�С
		{
			if (last_Y_error<0)    try_change /= 2;//��һ������Ϊ�����ͣ������ȷʵ�����ߣ�֤�����Ǽ�̫���ˣ���˼�Сһ���ϴ����ӵ�����
			
			Angle-=try_change;
            
			Temp_cos_Y_X = cos_calculate(Angle);
            Temp_sin_Y_X = sin_calculate(Angle);
            V_x = Temp_cos_Y_X*Vo;
			V_y = Temp_sin_Y_X*Vo;
            
			t = (pow(e,k_1*X_)-1)/(k_1*V_x);
			Temp_Y = V_y*t-0.5*G*t*t;//�����ǿ����������ȼ����˶�
			
			last_Y_error = Y_error;			//�������Yֵ
			Y_error = Temp_Y - Y_;
		}
	}
    *Fly_time = t;//�ɹ���ϵ��������ط���ʱ��
	return Angle;
    #endif
}

/**
  *@brief YAW��ǰ��վƫת�ĽǶ�
  *@param X ˮƽX�����,��ͷ����ϵ��Ϊ��
  *@param Y ˮƽY�����,��ͷ����ϵǰΪ��
  *@note  ��ʱ����
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
    //arm_sqrt_f32(X_*X_+Y_*Y_, &Z);//ֱ��б��
    float cos = (X_*X_+Z*Z-Y_*Y_)/2*X_*Z;//���Ҷ���
    float Angle_close = acos(cos);//����ֵ�󻡶�
    Angle_close = Angle_close*180/PI;//����ת�Ƕ�
    return Angle_close;
}
/**
  *@brief ����ǰ��վװ�װ���ת��Ӣ����ǰ��վ�������ߵ�ʱ��
  */
#define FIRE_DELAY_MS  200.0f//����
#define ROTATE_TIME_MS 325.0f//833.3333f �г�13��10Ȧ 0.77rad
//10m/s 5mԶ 1m�� ����ʱ��0.7s ����FIRE_DELAY_MS�������ӳ٣��ѳ���
/**
  *@parm ��λms
  */
float calc_fire_time(float now_in_place_time, float fly_time)
{
    if (fly_time == 0)  return 0;
    int64_t increat_time = ROTATE_TIME_MS;
    float next_in_place_time = now_in_place_time + ROTATE_TIME_MS;//��һ��ת�����ĵ�ʱ���
    while ((FIRE_DELAY_MS + fly_time) > increat_time)
    {   //����ʱ�䣫����ʱ�䳬����һ��װ�װ�ת��λ��ʱ�䣬���Ӧ��Ԥ�����¿�װ�װ嵽λʱ��
        next_in_place_time += ROTATE_TIME_MS;
        increat_time += ROTATE_TIME_MS;
    }
    float fire_time = next_in_place_time-FIRE_DELAY_MS-fly_time;
    return  fire_time;
}
/**
  *@brief �ж��Ƿ���Ͽ���ʵ��
  *@note  �����������Ԥ����һ�ε�λʱ��
  *@reval 0 ����ʱ��δ�� | 1 ����ʱ���ѵ� | 2 ����ʱ���ѹ� | 3 ���δ���
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


