/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ts.h"
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
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
#define READY 0
#define BUSY 1
#define RECEIVED 2

uint8_t rxBuff[12] { 0 };
uint8_t txBuff[12] { 0, 0, 0, 0, 2, 0,
					 0, 0, 0, 0, 2, 0};

uint8_t *rx[2] { &rxBuff[0], &rxBuff[6] };
uint8_t *tx[2] { &txBuff[0], &txBuff[6] };

uint8_t rxSaved[4] { 0 };
uint8_t spiState[2] { 0 };
uint8_t spiBuffNum { 0 };

uint8_t tuState { 0 };
uint8_t tu[2] { 0 };
uint8_t shiftRegNum { 0 };
uint8_t sendTuNum { 0 };

Ts ts[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi2) {
		spiInterrupt(spiBuffNum);
	}
}

void spiInterrupt(uint8_t num) {
	spiState[num] = RECEIVED;
	uint8_t invNum = num ^ (1 << 0);
	if (spiState[invNum] == READY) {
		spiBuffNum = invNum;
		spiState[spiBuffNum] = BUSY;
		HAL_SPI_TransmitReceive_DMA(&hspi2, tx[spiBuffNum], rx[spiBuffNum], 6);
	}
}

void spiProcessing(uint8_t num) {
	uint8_t invNum = num ^ (1 << 0);
	if (spiState[invNum] == RECEIVED) {
		uint8_t rxSumm { 0 };
		for (uint8_t i = 0; i <= 4; i++) {
			rxSumm += rx[invNum][i];
		}
		if (rxSumm == rx[invNum][5] && tuState != BUSY) {
			HAL_IWDG_Refresh(&hiwdg);
			tx[invNum][2] = rx[invNum][2];
			tx[invNum][3] = rx[invNum][3];
			for (uint8_t i = 0; i <= 3; i++) {
				if (rx[invNum][i] != rxSaved[i]) {
					rxSaved[i] = rx[invNum][i];
					sendTuNum = 3;
				}
			}
		}
	}

	uint8_t tsNum { 0 };
	while (tsNum <= 7) {
		ts[tsNum].read(&tx[invNum][0], tsNum);
		++tsNum;
	}
	while (tsNum <= 15) {
		ts[tsNum].read(&tx[invNum][1], tsNum - 8);
		++tsNum;
	}

	tx[invNum][5] =  (tx[invNum][0] + tx[invNum][1] + tx[invNum][2] + tx[invNum][3] + tx[invNum][4]);
	spiState[invNum] = READY;
	if (spiState[num] == RECEIVED) {
		spiBuffNum = invNum;
		spiState[spiBuffNum] = BUSY;
		HAL_SPI_TransmitReceive_DMA(&hspi2, tx[spiBuffNum], rx[spiBuffNum], 6);
	}
}

void sendTu() {
	if (sendTuNum != 0 && tuState == READY) {
		tuState = BUSY;
		--sendTuNum;
		for (uint8_t n = 0; n <= 1; n++) {
			for (uint8_t i = 0; i <= 7; i++) {
				if (rxSaved[n + 2] & (1 << i)) {
					tu[n] |= (1 << (7 - i));
				} else {
					tu[n] &= ~(1 << (7 - i));
				}
			}
		}
		GPIOA->BRR = (1 << 4);
		HAL_SPI_Transmit_DMA(&hspi1, &tu[0], 1);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		switch (shiftRegNum) {
		case 0:
			GPIOA->BSRR = (1 << 4);
			GPIOA->BRR = (1 << 4);
			GPIOA->BSRR = (1 << 4);

			GPIOF->BRR = (1 << 5);
			HAL_SPI_Transmit_DMA(&hspi1, &tu[1], 1);
			break;
		case 1:
			GPIOF->BSRR = (1 << 5);
			GPIOF->BRR = (1 << 5);
			GPIOF->BSRR = (1 << 5);
			tuState = READY;
		}
		shiftRegNum ^= (1 << 0);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	HAL_SPI_Abort(hspi);
	__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
	__HAL_SPI_CLEAR_FREFLAG(hspi);
	__HAL_SPI_CLEAR_MODFFLAG(hspi);
	__HAL_SPI_CLEAR_OVRFLAG(hspi);
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	if (hspi == &hspi2) {
		HAL_SPI_TxRxCpltCallback(hspi);
	}
	if (hspi == &hspi1) {
		HAL_SPI_TxCpltCallback(hspi);
	}
}

void HAL_SPI_DMAErrorCallback(SPI_HandleTypeDef *hspi) {
	HAL_SPI_Abort(hspi);
	__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
	__HAL_SPI_CLEAR_FREFLAG(hspi);
	__HAL_SPI_CLEAR_MODFFLAG(hspi);
	__HAL_SPI_CLEAR_OVRFLAG(hspi);
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	if (hspi == &hspi2) {
		HAL_SPI_TxRxCpltCallback(hspi);
	}
	if (hspi == &hspi1) {
		HAL_SPI_TxCpltCallback(hspi);
	}
}

void tsInit() {
	for (int i = 0; i <= 15; i++) {
		ts[i].setOut(GPIOB, 0);
		ts[i].setSw(GPIOB, 1, GPIOB, 10, GPIOC, 5, GPIOC, 4);
	}
	ts[0].setPinNum(0);
		ts[1].setPinNum(1);
		ts[2].setPinNum(2);
		ts[3].setPinNum(3);
		ts[4].setPinNum(8);
		ts[5].setPinNum(9);
		ts[6].setPinNum(10);
		ts[7].setPinNum(11);
		ts[8].setPinNum(15);
		ts[9].setPinNum(14);
		ts[10].setPinNum(13);
		ts[11].setPinNum(12);
		ts[12].setPinNum(7);
		ts[13].setPinNum(6);//!!!!!!!!!!!
		ts[14].setPinNum(5);
		ts[15].setPinNum(4);
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  tsInit();

  spiState[0] = BUSY;
  spiBuffNum = 0;
  HAL_SPI_TransmitReceive_DMA(&hspi2, tx[0], rx[0], 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* Обработка приема по spi */
		spiProcessing(spiBuffNum);
		/* Отправка сигналов ТУ */
		sendTu();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 1500;
  hiwdg.Init.Reload = 1500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
