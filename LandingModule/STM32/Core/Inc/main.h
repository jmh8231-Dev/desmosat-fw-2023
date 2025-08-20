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
#include "stm32l4xx_hal.h"

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
#define shutdown_Pin GPIO_PIN_14
#define shutdown_GPIO_Port GPIOC
#define TOF_XSHUT_b_Pin GPIO_PIN_15
#define TOF_XSHUT_b_GPIO_Port GPIOC
#define CDS1_Pin GPIO_PIN_1
#define CDS1_GPIO_Port GPIOA
#define Debug_UART_Tx_Pin GPIO_PIN_2
#define Debug_UART_Tx_GPIO_Port GPIOA
#define Debug_UART_Rx_Pin GPIO_PIN_3
#define Debug_UART_Rx_GPIO_Port GPIOA
#define CDS2_Pin GPIO_PIN_4
#define CDS2_GPIO_Port GPIOA
#define CDS3_Pin GPIO_PIN_5
#define CDS3_GPIO_Port GPIOA
#define CDS4_Pin GPIO_PIN_6
#define CDS4_GPIO_Port GPIOA
#define ToF_SCL_Pin GPIO_PIN_7
#define ToF_SCL_GPIO_Port GPIOA
#define Landing_GPIO_Pin GPIO_PIN_0
#define Landing_GPIO_GPIO_Port GPIOB
#define L_Debug_L1_Pin GPIO_PIN_1
#define L_Debug_L1_GPIO_Port GPIOB
#define S_T_Pin GPIO_PIN_10
#define S_T_GPIO_Port GPIOA
#define ToF_SDA_Pin GPIO_PIN_4
#define ToF_SDA_GPIO_Port GPIOB
#define IR_CAM_SCL_Pin GPIO_PIN_6
#define IR_CAM_SCL_GPIO_Port GPIOB
#define IR_CAM_CDA_Pin GPIO_PIN_7
#define IR_CAM_CDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
