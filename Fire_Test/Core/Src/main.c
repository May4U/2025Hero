/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "bsp_can.h"
#include "bsp_Motor_Encoder.h"
#include "DJI_Motor.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct{
    DJIMotor_object_t*   left_motor;
    DJIMotor_object_t*   right_motor;
}Fire_t;
Fire_t Fire;
int16_t left_speed = 0;
int16_t right_speed = 0;
uint8_t fire_set = 0;

void fire_task_init(void)
{
    memset(&Fire, 0, sizeof(Fire_t));
	  // 获得拨弹指针
    Fire.left_motor  = DJIMotor_Init(1,2,false,M3508,30);
    Fire.right_motor = DJIMotor_Init(1,4,true,M3508,30);

    Fire.left_motor->Using_PID = Speed_PID;
    Fire.right_motor->Using_PID = Speed_PID;
    
    PidInit(&Fire.left_motor->Speed_PID,1.45f,0,0,Output_Limit|StepIn);
 	  PidInitMode(&Fire.left_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.left_motor->Speed_PID,StepIn,10,0);//过大的加减�?�会导致发射机构超电流断�?
    PidInit(&Fire.right_motor->Speed_PID,1.0f,0,0,Output_Limit|StepIn);
 	  PidInitMode(&Fire.right_motor->Speed_PID,Output_Limit,16000,0);//输出限幅模式设置
    PidInitMode(&Fire.right_motor->Speed_PID,StepIn,10,0);//逐渐减�?�，实测发现�?大�?�度减�?�会导致发射机构超电流断�?
}


void fire_behaviour_choose(void)
{
    // if (Fire.Gimbal_CMD->Fire_Ready == 0)
    // {
    //     DJIMotor_Set_val(Fire.left_motor_up, 0);
    //     DJIMotor_Set_val(Fire.right_motor_up, 0);
    //     DJIMotor_Set_val(Fire.left_motor, 0);
    //     DJIMotor_Set_val(Fire.right_motor, 0);
    // }
    // else
    // {
    //     DJIMotor_Set_val(Fire.left_motor_up, Fire.Gimbal_CMD->Fire_Set_Rpm);
    //     DJIMotor_Set_val(Fire.right_motor_up,Fire.Gimbal_CMD->Fire_Set_Rpm);
    //     DJIMotor_Set_val(Fire.left_motor,    Fire.Gimbal_CMD->Fire_Set_Rpm);
    //     DJIMotor_Set_val(Fire.right_motor,   Fire.Gimbal_CMD->Fire_Set_Rpm);
    // }
    fire_set = HAL_GPIO_ReadPin(fire_set_GPIO_Port, fire_set_Pin);
    if(fire_set == 1)
    {
      DJIMotor_Set_val(Fire.left_motor, 0);
      DJIMotor_Set_val(Fire.right_motor, 0);
    }
    else
    {
      DJIMotor_Set_val(Fire.left_motor, 4800);
      DJIMotor_Set_val(Fire.right_motor, 4800);
    }
    
}

void fire_pid_calculate(void)
{
    Fire.left_motor->current_input = motor_speed_control(&Fire.left_motor->Speed_PID,
                                                          Fire.left_motor->set_speed,
                                                          Fire.left_motor->Motor_Information.speed);
    Fire.right_motor->current_input = motor_speed_control(&Fire.right_motor->Speed_PID,
                                                          Fire.right_motor->set_speed,
                                                          Fire.right_motor->Motor_Information.speed);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  ECF_CAN_Init();
  //DWT_Init(168);
  fire_task_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    left_speed = Fire.left_motor->Motor_Information.speed;
    right_speed = -Fire.right_motor->Motor_Information.speed;
    fire_behaviour_choose();
		fire_pid_calculate();
    DJIMotor_Send(Fire.left_motor);
    // DJMotor_Send_only_one(Fire.left_motor);
    // DJMotor_Send_only_one(Fire.right_motor);
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
