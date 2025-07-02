/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLING_FREQUENCY_HZ 47991.0f

#define UINT16_TO_FLOAT 0.00001525878f
#define INT16_TO_FLOAT 	0.00003051757f
#define FLOAT_TO_INT16 	32768.0f

#define BUFFER_SIZE 	128

#define LED_Pin        GPIO_PIN_6
#define LED_GPIO_Port  GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

/* USER CODE BEGIN PV */

int16_t adcBuff[BUFFER_SIZE];
int16_t dacBuff[BUFFER_SIZE];

static volatile int16_t *inBufferPtr = &adcBuff[0];
static volatile int16_t *outBufferPtr = &dacBuff[0];

uint8_t dataReadyFlag;

uint8_t playAudio = 1;
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 250;  // 50 ms

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    inBufferPtr  = &(adcBuff[0]);
    outBufferPtr = &(dacBuff[0]);

    processData();
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
    inBufferPtr  = &(adcBuff[BUFFER_SIZE / 2]);
    outBufferPtr = &(dacBuff[BUFFER_SIZE / 2]);

    processData();
}


/**
 * @brief Main audio processing function
 */
void processData() {

	// Input samples
	static float leftIn		= 0.0f;
	static float rightIn	= 0.0f;

	// Output samples
	static float leftOut	= 0.0f;
	static float rightOut	= 0.0f;

	for (uint16_t n = 0; n < BUFFER_SIZE/2; n += 2) {

		leftIn  = INT16_TO_FLOAT * inBufferPtr[n];
		rightIn = INT16_TO_FLOAT * inBufferPtr[n+1];

		if (playAudio)
		{
			leftOut  = leftIn;
			rightOut = rightIn;
		}

		outBufferPtr[n]   = (int16_t)(FLOAT_TO_INT16 * leftOut);
		outBufferPtr[n+1] = (int16_t)(FLOAT_TO_INT16 * rightOut);

		//outBufferPtr[n] = inBufferPtr[n];
		//outBufferPtr[n+1] = inBufferPtr[n+1];
	}

	dataReadyFlag = 0;
}

void Process_HalfBuffer() {

	// Input samples
	static float leftIn		= 0.0f;
	static float rightIn	= 0.0f;

	// Output samples
	static float leftOut	= 0.0f;
	static float rightOut	= 0.0f;

	// Loop through half of audio buffer (double buffering), convert int->float, apply processing, convert float->int, set output buffers
	for (uint16_t sampleIndex = 0; sampleIndex < (BUFFER_SIZE/2) - 1; sampleIndex += 2) {

		/*
		 * Convert current input samples (24-bits) to floats (two I2S data lines, two channels per data line)
		 */

		// Extract 24-bits via bit mask
		inBufferPtr[sampleIndex]		&= 0xFFFFFF;
		inBufferPtr[sampleIndex + 1]	&= 0xFFFFFF;

		// Check if number is negative (sign bit)
		if (inBufferPtr[sampleIndex] & 0x800000) {
			inBufferPtr[sampleIndex] |= ~0xFFFFFF;
		}

		if (inBufferPtr[sampleIndex + 1] & 0x800000) {
			inBufferPtr[sampleIndex + 1] |= ~0xFFFFFF;
		}

		// Normalise to float (-1.0, +1.0)
		leftIn  = (float) inBufferPtr[sampleIndex]     / (float) (0x7FFFFF);
		rightIn = (float) inBufferPtr[sampleIndex + 1] / (float) (0x7FFFFF);

		/*
		 * Apply processing
		 */
		leftOut  = leftIn;
		rightOut = rightIn;



		/*
		 * Convert floats to 32-bit output samples
		 */

		// Ensure output samples are within [-1.0,+1.0] range
		if (leftOut < -1.0f) {
			leftOut = -1.0f;
		} else if (leftOut > 1.0f) {
			leftOut =  1.0f;
		}

		if (rightOut < -1.0f) {
			rightOut = -1.0f;
		} else if (rightOut > 1.0f) {
			rightOut =  1.0f;
		}

		// Scale to 24-bit signed integer and set output buffer
		outBufferPtr[sampleIndex]	   = (int32_t) (leftOut  * 0x7FFFFF);
		outBufferPtr[sampleIndex + 1] = (int32_t) (rightOut * 0x7FFFFF);
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  //HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *) buffer_audio, 2 * NUM_SAMPLES);

  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, (uint16_t *) dacBuff, (uint16_t *) adcBuff, BUFFER_SIZE);

//  if (halStatus == HAL_OK)
//  {
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // vžgi LED
//  }
//  else
//  {
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // ostane ugasnjena
//  }

  uint8_t playbackActive = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && (HAL_GetTick() - lastDebounceTime) > debounceDelay) {
		  playAudio = !playAudio;
		  lastDebounceTime = HAL_GetTick();
	  }

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
