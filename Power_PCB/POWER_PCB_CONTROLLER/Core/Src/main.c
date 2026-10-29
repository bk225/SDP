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
#include "firmware_types.h"
#include "firmware_tests.h"
/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */



void setAcceleratorEnable(uint8_t enableCommand);

void setBrakingEnable(uint8_t enableCommand);

void setSteeringEnable(uint8_t enableCommand);

void setAcceleratorPWM(uint8_t pwmCommand);

void setBrakingPWM(uint8_t pwmCommand);

void setSteeringPWM(uint8_t pwmCommand);






/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* The FIRMWARE_SELF_TEST switch is in Core/Inc/firmware_tests.h. */





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



uint8_t servo_commands[5] = {0, 0, 0, 0, 0};

uint8_t servo_angles[5] = {90, 90, 90, 90, 90};

uint32_t lastServoStepTime[5] = {0};


SafetyState safetyState = SAFETY_DISARMED;

bool motorAuthorized = false;



Motor accelerator = {

    .state = MOTOR_WAITING,

    .direction = 0,

    .stopTime = 0,

	.setPWM = setAcceleratorPWM,

	.setEnable = setAcceleratorEnable,

	.reverseDelayMs = 2000

};



Motor braking = {

    .state = MOTOR_WAITING,

    .direction = 0,

    .stopTime = 0,

	.setPWM = setBrakingPWM,

	.setEnable = setBrakingEnable,

	.reverseDelayMs = 1000

};



Motor steering = {

    .state = MOTOR_WAITING,

    .direction = 0,

    .stopTime = 0,

	.setPWM = setSteeringPWM,

	.setEnable = setSteeringEnable,

	.reverseDelayMs = 1000

};



uint8_t rxByte = 0;

uint8_t rxPacket[14];

uint8_t rxIndex = 0;

uint8_t completedPacket[14];

volatile bool packetReady = false;

uint32_t lastValidPacketTime = 0;

uint8_t validCommandProlonged = 250;

bool hasReceivedValidPacket = false;



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







void startMotorPWM(void);

void initializeMotorPWM(void);

uint16_t motorCommandToCCR(uint8_t pwmCommand);

int8_t motorDirection(uint8_t pwmCommand);



bool hasTimeElapsed(uint32_t startTime, uint32_t delayMs);

bool isDirectionReversed(int8_t previousDirection, int8_t requestedDirection);



void updateMotor(Motor *motor, uint8_t pwmCommand, uint8_t enableCommand, bool motorAuthorized);

void updateAllMotors(bool motorAuthorized);



void processReceivedByte(uint8_t byte);

bool validateChecksum(uint8_t *packet);

bool validateCommands(uint8_t *packet);

bool validatePacket(uint8_t *packet);

bool applyPacket(uint8_t *packet);

bool isCommunicationProlonged(void);



void updateSafetyState(void);


void updateServoAngles(void);

uint16_t servoAngleToPulseUs(uint8_t angle);

void updateServoPWM(void);

void startServoPWM(void);


/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

bool validateCommands(uint8_t *packet){

	for (uint8_t i = 2; i < 5; i++ ){

		if (packet[i] != 0 && packet[i] != 1){

			return false;

		}

	}

	for (uint8_t i = 5; i < 8; i++ ){

		if (packet[i] > 200){

			return false;

		}

	}

	for(uint8_t i = 8; i < 13; i++){

		if((packet[i] != 100) && (packet[i] != 108) && (packet[i] != 117) && (packet[i] != 114) && (packet[i] != 0)){

			return false;

		}

	}

	return true;

}



bool validateChecksum(uint8_t *packet){

	uint16_t sum = 0;

	uint8_t checkSum = 0;

	for (uint8_t i = 2; i < 13; i++){

		uint8_t value = packet[i];

		sum += value;

	}

	checkSum = sum%256;

	return (checkSum == packet[13]);

}



bool validatePacket(uint8_t *packet){

	return (packet[0] == 0xAA && packet[1] == 0x55 && validateChecksum(packet) && validateCommands(packet));

}



bool applyPacket(uint8_t *packet){

	if (validatePacket(packet)){

		brake_enable = packet[2];

		steering_enable = packet[3];

		accelerator_enable = packet[4];

		brake_pwm = packet[5];

		steering_pwm = packet[6];

		accelerator_pwm = packet[7];

		for (uint8_t i = 0; i < 5; i++){

			servo_commands[i] = packet[i+8];

		}

		return true;

	}

	return false;

}



void processReceivedByte(uint8_t rxByte){

	if (rxIndex == 0){

		if(rxByte == 0xAA){

			rxPacket[0] = rxByte;

			rxIndex ++;

		}

		return;

	}

	if (rxIndex == 1){

		if(rxByte == 0x55){

			rxPacket[1] = rxByte;

			rxIndex ++;

		}

		else if(rxByte == 0xAA){

			rxPacket[0] = rxByte;

			rxIndex = 1;

		}

		else {rxIndex = 0;}

		return;

	}

	if ((rxIndex > 1) && (rxIndex < 14)){

		rxPacket[rxIndex] = rxByte;

		rxIndex++;

	}

	if (rxIndex == 14){

		rxIndex = 0;

		if (!packetReady){

			for (uint8_t i = 0; i < 14; i++){

				completedPacket[i] = rxPacket[i];

			}

		packetReady = true;

		}

	}

	return;

}

void updateServoAngles(void){

    uint32_t currentTime = getFirmwareTick();

    if (isCommunicationProlonged()){

        for (uint8_t i = 0; i < 5; i++){

            lastServoStepTime[i] = currentTime;

        }

        return;
    }

    for (uint8_t i = 0; i < 5; i++){

    	int8_t direction = 0;

    	if (i == 0 || i == 2 || i == 4){

            if (servo_commands[i] == 114){        // Right

                direction = 1;

            }

            else if (servo_commands[i] == 108){   // Left

                direction = -1;

            }

    	}

    	else {

            if (servo_commands[i] == 117){        // up

                direction = 1;

            }

            else if (servo_commands[i] == 100){   // down

                direction = -1;

            }

    	}

        if (direction == 0){

            lastServoStepTime[i] = currentTime;

        }

        if (currentTime - lastServoStepTime[i] >= 100){



            if (direction == 1 && servo_angles[i] < 180){

                servo_angles[i]++;

            }

            else if (direction == -1 && servo_angles[i] > 0 ){

            	servo_angles[i]--;

            }

            lastServoStepTime[i] = currentTime;

        }

    }

}


uint16_t servoAngleToPulseUs(uint8_t angle){

	if (angle > 180){

		angle = 180;

	}

	uint32_t convert1 = angle * 1000;

	uint16_t convert2 = convert1 / 180;

	uint16_t pulse = 1000 + convert2;

	return pulse;

}


void updateServoPWM(void){

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servoAngleToPulseUs(servo_angles[0]));

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servoAngleToPulseUs(servo_angles[1]));

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servoAngleToPulseUs(servo_angles[2]));

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servoAngleToPulseUs(servo_angles[3]));

	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, servoAngleToPulseUs(servo_angles[4]));

}


void startServoPWM(void){

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK){

			Error_Handler();

		}

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK){

			Error_Handler();

		}

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK){

			Error_Handler();

		}

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK){

			Error_Handler();

		}

	if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK){

			Error_Handler();

		}

}


bool isCommunicationProlonged(void){

	if (!hasReceivedValidPacket){

		return true;

	}

	if ((getFirmwareTick() - lastValidPacketTime) > validCommandProlonged){

		return true;

	}

	return false;

}



bool hasTimeElapsed(uint32_t startTime, uint32_t delayMs){

	return (delayMs <= (getFirmwareTick() - startTime));

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



void updateMotor(Motor *motor, uint8_t pwmCommand, uint8_t enableCommand, bool motorAuthorized)

{

    int8_t requestedDirection = motorDirection(pwmCommand);

    if (!motorAuthorized || enableCommand != 1 || requestedDirection == 0)	//Stop immediately if motor operation is not authorized, disabled, or commanded neutral.

    {

        motor->setEnable(0);

        motor->setPWM(100);



        if (motor->state == MOTOR_RUNNING)

        {

        	motor->stopTime = getFirmwareTick();

        	motor->state = MOTOR_WAITING;

        }



        return;

    }



    if (motor->state == MOTOR_RUNNING && isDirectionReversed(motor->direction, requestedDirection))    //Detect a reversal while running.

    {

        motor->setEnable(0);

        motor->setPWM(100);

        motor->stopTime = getFirmwareTick();

        motor->state = MOTOR_STOPPING;

        return;

    }



    if (motor->state == MOTOR_RUNNING){

        motor->setPWM(pwmCommand);

        return;

    }



    if (motor->state == MOTOR_STOPPING) //Handle the stopping interval.

    {

        motor->setEnable(0);

        motor->setPWM(100);

        if (hasTimeElapsed(motor->stopTime, motor->reverseDelayMs) || !isDirectionReversed(motor->direction, requestedDirection))

        {

            motor->state = MOTOR_WAITING;

        }



        return;

    }



    if (motor->state == MOTOR_WAITING) //Check whether a waiting motor may restart.

    {

        if (!isDirectionReversed(motor->direction,requestedDirection) ||hasTimeElapsed(motor->stopTime,motor->reverseDelayMs))

        {

            motor->direction = requestedDirection;

            motor->setPWM(pwmCommand);

            motor->setEnable(1);

            motor->state = MOTOR_RUNNING;

        }

        else

        {

            motor->setEnable(0);

            motor->setPWM(100);

            return;

        }

    }

}



void updateAllMotors(bool motorAuthorized)

{

	updateMotor(&accelerator, accelerator_pwm, accelerator_enable, motorAuthorized);

	updateMotor(&braking, brake_pwm, brake_enable, motorAuthorized);

	updateMotor(&steering, steering_pwm, steering_enable, motorAuthorized);

}



void updateSafetyState(void){

	if (safetyState == SAFETY_DISARMED){

		motorAuthorized = false;

		disableMotors();

		updateAllMotors(motorAuthorized);

	}

	else if (safetyState == SAFETY_ARMED){

		if (!isCommunicationProlonged()){

			motorAuthorized = true;

			updateAllMotors(motorAuthorized);

		}

		else{

			motorAuthorized = false;

			disableMotors();

			updateAllMotors(motorAuthorized);

		}

	}

	else if (safetyState == SAFETY_FAULT){

		motorAuthorized = false;

		disableMotors();

		updateAllMotors(motorAuthorized);

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
#if FIRMWARE_SELF_TEST //To enable or disable testing, change this line in firmware_tests.h
  /* Do not start motor PWM or arm UART receive in test mode. */
  runFirmwareTests();
  while (1) {
      __NOP(); /* Inspect testChecks/testFailures/testFailureLog. */
  }
#else
  initializeMotorPWM();
  startMotorPWM();
  updateServoPWM();
  startServoPWM();
  if (HAL_UART_Receive_IT(&huart3, &rxByte, 1) != HAL_OK) {
      disableMotors();
      Error_Handler();
  }
#endif

/* USER CODE END 2 */



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

    /* USER CODE END WHILE */



    /* USER CODE BEGIN 3 */



	  if (packetReady){

		  if (applyPacket(completedPacket)){

			  lastValidPacketTime = getFirmwareTick();

			  hasReceivedValidPacket = true;

		  }

		  packetReady = false;

	  }

	  updateSafetyState();
	  updateServoAngles();
	  updateServoPWM();


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



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{

    if (huart == &huart3){

    	processReceivedByte(rxByte);

    	if (HAL_UART_Receive_IT(&huart3, &rxByte, 1) != HAL_OK){

    		disableMotors();

    		Error_Handler();

    	}

    }

}



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
