/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @file lqr.h
 * @brief 
 * @author 洪张茹
 * @version 1.0
 * @date 2022-12-05
 * 
 * ************************* Dongguan-University of Technology -ACE**************************
 */
#ifndef LQR_H
#define LQR_H

/* Include */
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <string.h>
#include "math.h"


/* Define */
#ifndef lqr_abs
#define lqr_abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif




/* Private Body */
typedef struct 
{
  uint8_t System_State_Size;     //对应u与input
  uint8_t Control_Size;          //对应Output

  //非线性控制量
  float Control_Variable;
  float Control_Area;      //
	
  float *Input;
  float *Output;
  float *k;
  
  float *target;
	
  void (*User_Func_f)(void);

}LQR_t;

typedef struct 
{
  uint8_t System_State_Size;     //对应u与input
  uint8_t Control_Size;          //对应Output

  //非线性控制量
  int64_t Control_Variable;
  int64_t Control_Area;      //
	
  int64_t *Input;
  int64_t *Output;
  float *k;
  
  int64_t *target;
	
  void (*User_Func_f)(void);

}LQR_int_t;

/* Funtion */
//extern void LQR_Calculate(LQR_t *lqr);
void LQR_Init(LQR_t *lqr, uint8_t system_state_size, uint8_t control_size, float *k);
void LQR_Data_Update(LQR_t *lqr, float* system_state);
void LQR_Calculate(LQR_t *lqr);

void LQR_Int_Init(LQR_int_t *lqr, uint8_t system_state_size, uint8_t control_size, float *k);
void LQR_Int_Data_Update(LQR_int_t *lqr, int64_t* system_state);
void LQR_Int_Calculate(LQR_int_t *lqr);
#endif


