/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	MOTOR_RUNNING,
	MOTOR_STOPPING,
	MOTOR_WAITING
}MotorState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_REVERSE_DELAY_MS 500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t brake_pwm = 100;
uint8_t steering_pwm = 100;
uint8_t accelerator_pwm = 100;

uint8_t brake_enable = 0;
uint8_t steering_enable = 0;
uint8_t accelerator_enable = 0;

uint8_t servo_angles[6] = {90, 90, 90, 90, 90, 90};

MotorState acceleratorState = MOTOR_WAITING;
MotorState brakingState = MOTOR_WAITING;
MotorState steeringState = MOTOR_WAITING;

int8_t acceleratorDirection = 0;
int8_t brakingDirection = 0;
int8_t steeringDirection = 0;

uint32_t acceleratorStopTime = 0;
uint32_t brakingStopTime = 0;
uint32_t steeringStopTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void disableAccelerating(void);
void disableBraking(void);
void disableSteering(void);
void disableMotors(void);
void enableAccelerating(void);
void enableBraking(void);
void enableSteering(void);
void setAcceleratorEnable(uint8_t enableCommand);
void setBrakingEnable(uint8_t enableCommand);
void setSteeringEnable(uint8_t enableCommand);


void startMotorPWM(void);
void initializeMotorPWM(void);
uint16_t motorCommandToCCR(uint8_t pwmCommand);
int8_t motorDirection(unit_8 pwmCommand);
void setAcceleratorPWM(uint8_t pwmCommand);
void setBrakingPWM(uint8_t pwmCommand);
void setSteeringPWM(uint8_t pwmCommand);

bool hasTimeElapsed(uint32_t startTime, uint32_t delayMs);
bool isDirectionReversed(int8_t previousDirection, int8_t requestedDirection);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool hasTimeElapsed(uint32_t startTime, uint32_t delayMs){
	return (delayMs <= (HAL_GetTick() - startTime));
}
bool isDirectionReversed(int8_t previousDirection, int8_t requestedDirection){
	if (previousDirection == 0 || requestedDirection == 0){
		return false;
	}
	if (previousDirection == requestedDirection){
		return false;
	}
	else{
		return true;
	}
}

void disableAccelerating(void){
	HAL_GPIO_WritePin(GPIOA, ACC_EN1_Pin|ACC_EN2_Pin, GPIO_PIN_RESET);
}
void disableBraking(void){
	HAL_GPIO_WritePin(GPIOB, BRA_EN1_Pin|BRA_EN2_Pin, GPIO_PIN_RESET);
}
void disableSteering(void){
	HAL_GPIO_WritePin(GPIOB, STE_EN1_Pin|STE_EN2_Pin, GPIO_PIN_RESET);
}
void disableMotors(void){
	disableAccelerating();
	disableSteering();
	disableBraking();
}
void enableAccelerating(void){
	HAL_GPIO_WritePin(GPIOA, ACC_EN1_Pin|ACC_EN2_Pin, GPIO_PIN_SET);
}
void enableBraking(void){
	HAL_GPIO_WritePin(GPIOB, BRA_EN1_Pin|BRA_EN2_Pin, GPIO_PIN_SET);
}
void enableSteering(void){
	HAL_GPIO_WritePin(GPIOB, STE_EN1_Pin|STE_EN2_Pin, GPIO_PIN_SET);
}
void setAcceleratorEnable(uint8_t enableCommand){
	if (enableCommand == 1){
		enableAccelerating();
	}
	else{
		disableAccelerating();
	}
}
void setBrakingEnable(uint8_t enableCommand){
	if (enableCommand == 1){
		enableBraking();
	}
	else{
		disableBraking();
	}
}
void setSteeringEnable(uint8_t enableCommand){
	if (enableCommand == 1){
		enableSteering();
	}
	else{
		disableSteering();
	}
}


void startMotorPWM(void){
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK){
		disableMotors();
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK){
			disableMotors();
			Error_Handler();
		}
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK){
			disableMotors();
			Error_Handler();
		}
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK){
			disableMotors();
			Error_Handler();
		}
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK){
			disableMotors();
			Error_Handler();
		}
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK){
			disableMotors();
			Error_Handler();
		}
}
void initializeMotorPWM(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}
uint16_t motorCommandToCCR(uint8_t pwmCommand){
    if (pwmCommand > 200) {
        return 0;
    }

    if (pwmCommand < 100) {
        pwmCommand = 100 - pwmCommand;
    }
    else {
        pwmCommand = pwmCommand - 100;
    }

    return pwmCommand * 50;
}
int8_t motorDirection(uint8_t pwmCommand){
	if (pwmCommand > 200){
		return 0;
	}
	if (pwmCommand > 100){
		return 1;
	}
	else if (pwmCommand < 100){
		return -1;
	}
	else{
		return 0;
	}
}
void setAcceleratorPWM(uint8_t pwmCommand){
	uint16_t acceleratorCCR = motorCommandToCCR(pwmCommand);
	int8_t acceleratorDirection = motorDirection(pwmCommand);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//set to 0
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	if (acceleratorDirection == 1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, acceleratorCCR);	//PA8 = positive acceleration
	}
	else if (acceleratorDirection == -1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, acceleratorCCR);	//PA9 = negative acceleration
	}
}
void setBrakingPWM(uint8_t pwmCommand){
	uint16_t brakingCCR = motorCommandToCCR(pwmCommand);
	int8_t brakingDirection = motorDirection(pwmCommand);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//set to 0
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	if (brakingDirection == 1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, brakingCCR);	//PA10 = positive braking
	}
	else if (brakingDirection == -1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, brakingCCR);	//PA11 = negative braking
	}
}
void setSteeringPWM(uint8_t pwmCommand){
	uint16_t steeringCCR = motorCommandToCCR(pwmCommand);
	int8_t steeringDirection = motorDirection(pwmCommand);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	//set to 0
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	if (steeringDirection == 1){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steeringCCR);	//PA0 = positive steering (right, clockwise)
	}
	else if (steeringDirection == -1){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, steeringCCR);	//PA1 = negative steering (left, cc)
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  disableMotors();
  initializeMotorPWM();
  startMotorPWM();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
