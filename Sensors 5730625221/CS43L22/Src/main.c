/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2559 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t soundArg[2];
uint16_t Istr[1];

/* USER CODE END 0 */

void InitSound() {

	Istr[0] = 0;
	if (HAL_I2S_Transmit(&hi2s3, Istr, 0x10, 10) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);

	HAL_Delay(500);

	//Initialization Setting...
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	HAL_Delay(500);
	soundArg[0] = 0x00;
	soundArg[1] = 0x99;
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	}

	soundArg[0] = 0x47;
	soundArg[1] = 0x80;
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	}

	soundArg[0] = 0x32;
	soundArg[1] = 0x80; // 0xBB or 0x80
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	}

	soundArg[0] = 0x32;
	soundArg[1] = 0x00; // 0x3B or 0x00
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	}
	soundArg[0] = 0x00;
	soundArg[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	}

	soundArg[0] = 0x06;
	soundArg[1] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	}

	soundArg[0] = 0x02;
	soundArg[1] = 0x9E;
	if (HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	}
}

void playSound(int note) {

	// Disable...
	soundArg[0] = 0x1E;
	soundArg[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50);

	// Change note...
	soundArg[0] = 0x1C;
	soundArg[1] = note;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50);

	// Enable...
	soundArg[0] = 0x1E;
	soundArg[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, soundArg, 2, 50);

	int i;
	for (i = 0; i < 100; i++)
		HAL_I2S_Transmit(&hi2s3, Istr, 200, 10);
}



int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2S3_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	InitSound();
	int count = 0;
	char note;
	int octave;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		uint8_t pData;
		if (HAL_UART_Receive(&huart2, &pData, 1, 10000) == HAL_OK) {
			HAL_UART_Transmit(&huart2, &pData, 1, 10000);
			if (pData == 13 && count != 2) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
				HAL_UART_Transmit(&huart2, (uint8_t*) ("\n\r"), 2, 10000);
				count = 0;
				continue;
			}

			if (count == 2 && pData == 13) {
				HAL_UART_Transmit(&huart2, (uint8_t*) ("\n\r"), 2, 10000);
				uint8_t n;
				switch (note) {
				case 'C':
					if (octave == 4)			//C4
						n = 0x0F;
					else if (octave == 5)		//C5
						n = 0x1F;
					else if (octave == 6)		//C6
						n = 0x8F;
					else if (octave == 7)		//C7
						n = 0xFF;
					break;
				case 'D':
					if (octave == 5)			//D5
						n = 0x2F;
					else
						n = 0x9F;
					break;
				case 'E':
					if (octave == 5)			//E5
						n = 0x3F;
					else
						n = 0xAF;
					break;
				case 'F':
					if (octave == 5)			//F5
						n = 0x4F;
					else
						n = 0xBF;
					break;
				case 'G':
					if (octave == 5)			//G5
						n = 0x5F;
					else
						n = 0xCF;
					break;
				case 'A':
					if (octave == 5)			//A5
						n = 0x6F;
					else
						n = 0xDF;
					break;
				case 'B':
					if (octave == 5)			//B5
						n = 0x7F;
					else
						n = 0xEF;
					break;
				default:						// No note...
					n = 0xFF;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
					break;
				}
				playSound(n);
				count = 0;
			} else {

				if (count == 1 && (pData >= '4' && pData <= '7')) {
					if ((pData == '4' || pData == '7') && note != 'C') {
						count = 0;
						continue;
					}
					count++;
					octave = pData - '0';
				} else {
					if ((pData == 'C' || pData == 'D' || pData == 'E'
							|| pData == 'F' || pData == 'G' || pData == 'A'
							|| pData == 'B') && count == 0) {
						count++;
						note = pData;
					} else
						count = 0;
				}

			}

		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

}

/* I2S3 init function */
static void MX_I2S3_Init(void) {

	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PC3   ------> I2S2_SD
 PA5   ------> SPI1_SCK
 PA6   ------> SPI1_MISO
 PA7   ------> SPI1_MOSI
 PB10   ------> I2S2_CK
 PA9   ------> USB_OTG_FS_VBUS
 PA10   ------> USB_OTG_FS_ID
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT1_Pin */
	GPIO_InitStruct.Pin = MEMS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
