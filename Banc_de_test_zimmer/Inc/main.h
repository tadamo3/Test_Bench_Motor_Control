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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define brake_1_Pin GPIO_PIN_0
#define brake_1_GPIO_Port GPIOC
#define brake_2_Pin GPIO_PIN_1
#define brake_2_GPIO_Port GPIOC
#define brake_3_Pin GPIO_PIN_2
#define brake_3_GPIO_Port GPIOC
#define brake_4_Pin GPIO_PIN_3
#define brake_4_GPIO_Port GPIOC
#define moteur_2_PUL_Pin GPIO_PIN_0
#define moteur_2_PUL_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_4
#define LD2_GPIO_Port GPIOA
#define limitswitch_1_Pin GPIO_PIN_5
#define limitswitch_1_GPIO_Port GPIOA
#define limitswitch_2_Pin GPIO_PIN_6
#define limitswitch_2_GPIO_Port GPIOA
#define limitswitch_3_Pin GPIO_PIN_7
#define limitswitch_3_GPIO_Port GPIOA
#define limitswitch_4_Pin GPIO_PIN_4
#define limitswitch_4_GPIO_Port GPIOC
#define limitswitch_5_Pin GPIO_PIN_5
#define limitswitch_5_GPIO_Port GPIOC
#define limitswitch_6_Pin GPIO_PIN_0
#define limitswitch_6_GPIO_Port GPIOB
#define moteur_4_ENA_Pin GPIO_PIN_1
#define moteur_4_ENA_GPIO_Port GPIOB
#define moteur_4_DIR_Pin GPIO_PIN_10
#define moteur_4_DIR_GPIO_Port GPIOB
#define moteur_3_DIR_Pin GPIO_PIN_12
#define moteur_3_DIR_GPIO_Port GPIOB
#define moteur_2_DIR_Pin GPIO_PIN_13
#define moteur_2_DIR_GPIO_Port GPIOB
#define moteur_1_DIR_Pin GPIO_PIN_14
#define moteur_1_DIR_GPIO_Port GPIOB
#define moteur_1_PUL_Pin GPIO_PIN_6
#define moteur_1_PUL_GPIO_Port GPIOC
#define moteur_3_PUL_Pin GPIO_PIN_7
#define moteur_3_PUL_GPIO_Port GPIOC
#define amp_4_Pin GPIO_PIN_8
#define amp_4_GPIO_Port GPIOC
#define amp_1_Pin GPIO_PIN_9
#define amp_1_GPIO_Port GPIOC
#define amp_3_Pin GPIO_PIN_8
#define amp_3_GPIO_Port GPIOA
#define amp_2_Pin GPIO_PIN_9
#define amp_2_GPIO_Port GPIOA
#define moteur_3_ENA_Pin GPIO_PIN_10
#define moteur_3_ENA_GPIO_Port GPIOA
#define moteur_2_ENA_Pin GPIO_PIN_11
#define moteur_2_ENA_GPIO_Port GPIOA
#define moteur_1_ENA_Pin GPIO_PIN_12
#define moteur_1_ENA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define encoder_4_tx_Pin GPIO_PIN_15
#define encoder_4_tx_GPIO_Port GPIOA
#define encoder_3_tx_Pin GPIO_PIN_10
#define encoder_3_tx_GPIO_Port GPIOC
#define encoder_2_tx_Pin GPIO_PIN_11
#define encoder_2_tx_GPIO_Port GPIOC
#define encoder_1_tx_Pin GPIO_PIN_12
#define encoder_1_tx_GPIO_Port GPIOC
#define encoder_4_rx_Pin GPIO_PIN_2
#define encoder_4_rx_GPIO_Port GPIOD
#define encoder_3_rx_Pin GPIO_PIN_3
#define encoder_3_rx_GPIO_Port GPIOB
#define encoder_2_rx_Pin GPIO_PIN_4
#define encoder_2_rx_GPIO_Port GPIOB
#define encoder_1_rx_Pin GPIO_PIN_5
#define encoder_1_rx_GPIO_Port GPIOB
#define moteur_4_PUL_Pin GPIO_PIN_6
#define moteur_4_PUL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
