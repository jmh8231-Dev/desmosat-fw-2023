/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CustomA_GPIO_Pin GPIO_PIN_0
#define CustomA_GPIO_GPIO_Port GPIOA
#define MiCS_4514_VNOX_Pin GPIO_PIN_1
#define MiCS_4514_VNOX_GPIO_Port GPIOA
#define MiCS_4514_VRED_Pin GPIO_PIN_2
#define MiCS_4514_VRED_GPIO_Port GPIOA
#define MQ_7_AOut_Pin GPIO_PIN_3
#define MQ_7_AOut_GPIO_Port GPIOA
#define DGS_SO2_Vtemp_Pin GPIO_PIN_4
#define DGS_SO2_Vtemp_GPIO_Port GPIOA
#define DGS_SO2_Vref_Pin GPIO_PIN_5
#define DGS_SO2_Vref_GPIO_Port GPIOA
#define DGS_SO2_Vgas_Pin GPIO_PIN_6
#define DGS_SO2_Vgas_GPIO_Port GPIOA
#define MiCS_4514_PRE_Pin GPIO_PIN_7
#define MiCS_4514_PRE_GPIO_Port GPIOA
#define CCS811_Wake_Pin GPIO_PIN_1
#define CCS811_Wake_GPIO_Port GPIOB
#define CCS811_Reset_Pin GPIO_PIN_2
#define CCS811_Reset_GPIO_Port GPIOB
#define DGS_SO2_Rx_Pin GPIO_PIN_10
#define DGS_SO2_Rx_GPIO_Port GPIOB
#define DGS_SO2_Tx_Pin GPIO_PIN_11
#define DGS_SO2_Tx_GPIO_Port GPIOB
#define PMS7003_Set_Pin GPIO_PIN_12
#define PMS7003_Set_GPIO_Port GPIOB
#define PMS7003_Reset_Pin GPIO_PIN_13
#define PMS7003_Reset_GPIO_Port GPIOB
#define MH_Z19_Rx_Pin GPIO_PIN_9
#define MH_Z19_Rx_GPIO_Port GPIOA
#define MH_Z19_Tx_Pin GPIO_PIN_10
#define MH_Z19_Tx_GPIO_Port GPIOA
#define Custom_Debug_Tx_Pin GPIO_PIN_10
#define Custom_Debug_Tx_GPIO_Port GPIOC
#define Custom_Debug_Rx_Pin GPIO_PIN_11
#define Custom_Debug_Rx_GPIO_Port GPIOC
#define PMS7003_Rx_Pin GPIO_PIN_12
#define PMS7003_Rx_GPIO_Port GPIOC
#define PMS7003_Tx_Pin GPIO_PIN_2
#define PMS7003_Tx_GPIO_Port GPIOD
#define CCS811_SCL_Pin GPIO_PIN_6
#define CCS811_SCL_GPIO_Port GPIOB
#define CCS811_SDA_Pin GPIO_PIN_7
#define CCS811_SDA_GPIO_Port GPIOB
#define Custom_Debug_L1_Pin GPIO_PIN_8
#define Custom_Debug_L1_GPIO_Port GPIOB
#define Custom_Debug_L2_Pin GPIO_PIN_9
#define Custom_Debug_L2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
