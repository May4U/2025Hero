#ifndef __BSP_IMU_H  //如果未定义
#define __BSP_IMU_H  //那么定义
/********基本头文件********/
#include "main.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "cmsis_os.h"
/********应用层头文件,位于ACE-ECF\Bsp\Inc\IMU_Driver********/
#include "./IMU_Driver/BMI088driver.h"
#include "./IMU_Driver/BMI088Middleware.h"
#include "./IMU_Driver/kalman_filter.h"
#include "./IMU_Driver/QuaternionEKF.h"
#include "./IMU_Driver/user_lib.h"
#include "./IMU_Driver/bsp_dwt.h"
#include "./IMU_Driver/bsp_pwm.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];//陀螺仪
    float Accel[3];//加速度
	float E_Accel[3];//地球坐标系加速度
    float MotionAccel_b[3];
    float MotionAccel_n[3];

	float Accel_filte[3];
	
    float AccelLPF;

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

extern void ECF_INS_Init(void);
extern void ECF_INS_Task(void);
void ECF_IMU_Task(void const *argument);
extern void IMU_Temperature_Ctrl(void);

extern void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
extern void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
extern void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
extern void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
extern void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
extern const INS_t *get_imu_control_point(void);
extern void imu_app_init(void);
extern void imu_Task(void const * argument);
//extern TaskHandle_t *get_imu_task_handle(void);


/**
 * @brief      返回陀螺仪结构体
 * @param[in]  none
 * @retval     imu_data_t *
 */
const INS_t *get_imu_control_point(void);

/**
 * @brief      陀螺仪初始化
 * @param[in]  none
 * @retval     void
 */
void ECF_INS_Init(void);

/**
 * @brief      陀螺仪初始化
 * @param[in]  none
 * @retval     void
 */
#endif