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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MHZ19B_CMD_READ_CO2 0x86
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart4, p, len, 10);
	return len;
}

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
uint8_t pms[32];
uint16_t adcval[5];
uint8_t str[128];

void read_Pms()
{
	for (uint8_t i = 0; i < sizeof(pms); i++)
	{
		if (HAL_UART_Receive(&huart5, &pms[i], 1, 1000) != HAL_OK) break;
	}
}

void send_Data()
{
	uint8_t txData[14];

	float no2 = (22000/(5/(2*adcval[0]/1227.6)-1));
	float no2_ppm = no2*0.000008-0.0194;
	int no2_int = (int)no2_ppm;
	int no2_float = (int)(no2_ppm * 100);

	float co_ppm = no2_ppm*20;
	uint8_t co_int = (int)co_ppm;
	uint8_t co_float = (int)(co_ppm * 100);


	float so2_concentration = 341.064; // 1/(sensitivity code(29.32) * TIA gain(100) * 10^-9 * 10^3)
	float so2_ppb = so2_concentration*(3.3/1024*(adcval[4]-adcval[3]));
	if(so2_ppb < 0) so2_ppb = 0;

	uint8_t so2_int = (int)so2_ppb;
	uint8_t so2_float = (int)(so2_ppb * 100);

	float so2_temp = (24.86*3.3/1024*adcval[2])-20;
	uint8_t so2Temp_int = (int)so2_temp;
	uint8_t so2Temp_float = (int)(so2_temp * 100);

	txData[0] = 77;
	txData[1] = 99;

	txData[2] = pms[10];
	txData[3] = pms[11];
	txData[4] = pms[12];
	txData[5] = pms[13];
	txData[6] = pms[14];
	txData[7] = pms[15];

	txData[8] = no2_int;
	txData[9] = no2_float;

	txData[10] = so2_int;
	txData[11] = so2_float;

	txData[12] = so2Temp_int;
	txData[13] = so2Temp_float;

	sprintf(str, "Data,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", txData[0], txData[1], txData[2], txData[3], txData[4], txData[5], txData[6], txData[7], txData[8], txData[9], txData[10], txData[11], txData[12], txData[13]);
	printf(str);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_ADC_Start_DMA(&hadc1, &adcval[0], 5);


    uint8_t str[20];
    uint8_t a = 0x7;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  read_Pms();
	  send_Data();
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	  HAL_Delay(1300);


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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
