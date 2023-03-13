/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "APDS9930.h"
#include "DHT.h"
#include "utils.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint8_t type;
	GPIO_TypeDef* port;
	uint16_t pin;
} device;

extern sensor sensors[SENSORS_COUNT];
extern device_state devices_states[DEVICES_COUNT];
extern mode current_mode;

uint8_t tx[BUFFER_SIZE];
uint8_t rx[BUFFER_SIZE];

DHT_sensor dht11 = { GPIOB, GPIO_PIN_6, DHT11, GPIO_NOPULL };

uint8_t fl_btn = 0;
uint8_t fl_uart = 0;

uint8_t periodCount = 0;

device devices[DEVICES_COUNT];

sensor* psensor;
device* pdevice;

void initDevices() {
	devices[0].type = DEVICE_LED_RED;
	devices[0].port = GPIOB;
	devices[0].pin = GPIO_PIN_4;
	//
	devices[1].type = DEVICE_LED_BLUE;
	devices[1].port = GPIOB;
	devices[1].pin = GPIO_PIN_5;
}

void getSensorsData() {

	psensor = NULL;

	FctERR status = APDS9930_handler(&APDS9930[0]);
	if (status != ERROR_OK) {
		__NOP();
	}
	psensor = getSensorByType(SENSOR_AMBIENT);
	if (psensor) {
		psensor->previous_value = psensor->value;
		psensor->value = APDS9930[0].Lux & 0xFF;
	}
	//
	DHT_data d = DHT_getData(&dht11);
	psensor = getSensorByType(SENSOR_TEMPERATURE);
	if (psensor) {
		psensor->previous_value = psensor->value;
		psensor->value = (int)d.temp;
	}
	psensor = getSensorByType(SENSOR_HUMIDITY);
	if (psensor) {
		psensor->previous_value = psensor->value;
		psensor->value = (int)d.hum;
	}

}

void DoUartReceive() {
	HAL_UART_Receive_IT (&huart1, rx, BUFFER_SIZE);
}

void DoUartTransmit() {
	HAL_UART_Transmit_IT (&huart1, tx, BUFFER_SIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	fl_btn = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		fl_uart = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		__NOP();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (current_mode.type == MODE_PERIODIC)
		periodCount++;
}

void fillTxSensorData() {

	memset(tx, 0, BUFFER_SIZE);

	tx[0] = MSG_SENSORS;

	getSensorsData();

	uint8_t i = 2;

	checkSensorsAlert();

	for (int j = 0; j < SENSORS_COUNT; j++) {
		tx[i++] = sensors[j].type;
		tx[i++] = sensors[j].value;
		tx[i++] = sensors[j].alert_check;
		tx[i++] = sensors[j].alert_compare;
		tx[i++] = sensors[j].alert_value;
		tx[i++] = sensors[j].alert_flag;
	}

	tx[1] = i;

	fillTxCRC(tx);

}

void fillTxModeData() {

	memset(tx, 0, BUFFER_SIZE);
	//
	tx[0] = MSG_MODE;
	tx[1] = 5;
	tx[2] = current_mode.type;
	tx[3] = current_mode.period;
	tx[4] = current_mode.percents;
	//
	fillTxCRC(tx);

}

void fillTxDevicesData() {

	memset(tx, 0, BUFFER_SIZE);

	tx[0] = MSG_DEVICES;

	uint8_t i = 2;

	for (int j = 0; j < DEVICES_COUNT; j++) {
		tx[i++] = devices_states[j].type;
		tx[i++] = devices_states[j].value;
	}

	tx[1] = i;

	fillTxCRC(tx);

}

void HandleButton() {
	fillTxSensorData();
	DoUartTransmit();
}

void sendCompleteStatus() {
	fillTxModeData();
	DoUartTransmit();
	HAL_Delay(300);
	//
	fillTxSensorData();
	DoUartTransmit();
	HAL_Delay(300);
	//
	fillTxDevicesData();
	DoUartTransmit();
}

void HandleUART() {

	DoUartReceive();

	uint8_t rx0 = rx[0];
	uint8_t rx1 = rx[1];

	uint8_t idx;
	uint8_t fl_transmit = 0;

	uint8_t crc = getCRC(rx1, rx);

	if (crc != rx[rx1]) {

		/* Handle CRC error */
		__NOP();
		//
		return;

	}

	memset(tx, 0, BUFFER_SIZE);

	if (rx0 == CMD_GET_SENSORS) {
		fillTxSensorData();
		fl_transmit = 1;
	} else
	//
	if (rx0 == CMD_SET_MODE) {
		current_mode.type = rx[2];
		if (current_mode.type == MODE_PERIODIC)
			current_mode.period = rx[3];
		if (current_mode.type == MODE_IFCHANGED)
			current_mode.percents = rx[3];
		//
		fillTxModeData();
		fl_transmit = 1;
	} else
	//
	if (rx0 == CMD_GET_MODE) {
		fillTxModeData();
		fl_transmit = 1;
	}
	//
	if (rx0 == CMD_SET_ALERTS) {
		for (int i = 0; i < rx[1] / 4; i++) {
			idx = 2 + i * 4;
			psensor = getSensorByType(rx[idx]);
			if (!psensor) continue;
			psensor->alert_check = rx[idx + 1];
			psensor->alert_compare = rx[idx + 2];
			psensor->alert_value = rx[idx + 3];
		}
		//
		fillTxSensorData();
		fl_transmit = 1;
	} else
	//
    if (rx0 == CMD_SET_DEVICES) {
    	for (int i = 0; i < rx[1] / 2; i++)
    		for (int j = 0; j < DEVICES_COUNT; j++) {
    			idx = 2 + i * 2;
    			if (devices[j].type == rx[idx]) {
    				devices_states[j].value = rx[idx + 1];
    				HAL_GPIO_WritePin(
    					devices[j].port,
						devices[j].pin,
						devices_states[j].value
    				);
    				break;
    			}
    		}
    	//
    	fillTxDevicesData();
    	fl_transmit = 1;
    } else
    //
    if (rx0 == CMD_GET_DEVICES) {
    	fillTxDevicesData();
    	fl_transmit = 1;
    } else
    //
    if (rx0 == CMD_GET_STATUS) {
    	sendCompleteStatus();
    }

	if (fl_transmit) DoUartTransmit();

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  initMode();
  initSensors();
  initDevices();
  initDevicesStates();

  HAL_TIM_Base_Start_IT(&htim3);

  FctERR status = APDS9930_Init(0, &hi2c2, APDS9930_ADDR);
  if (status != ERROR_OK) {
	  __NOP();
  }

  DoUartReceive();

  uint8_t fl_send_data = 0;

  sendCompleteStatus();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  getSensorsData();

	  fl_send_data = 0;

	  if (fl_btn) {

		  HandleButton();
		  //
		  fl_btn = 0;

	  }

	  if (fl_uart) {

		  HandleUART();
		  //
		  fl_uart = 0;

	  }

	  if ((current_mode.type == MODE_PERIODIC) &&
		  (periodCount >= current_mode.period)) {

		  fl_send_data = 1;
		  //
		  periodCount = 0;

	  }

	  if (current_mode.type  == MODE_IFCHANGED) {

		  fl_send_data = checkSensorsPercents(current_mode.percents);

	  }

	  if (fl_send_data) {

		  fillTxSensorData();
		  DoUartTransmit();

	  }

	  HAL_Delay(500);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
