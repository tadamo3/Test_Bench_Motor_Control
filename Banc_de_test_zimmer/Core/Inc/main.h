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
#include "stm32h7xx_hal.h"

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
#define encoder_4_A_Pin GPIO_PIN_0
#define encoder_4_A_GPIO_Port GPIOF
#define encoder_4_B_Pin GPIO_PIN_1
#define encoder_4_B_GPIO_Port GPIOF
#define encoder_4_X_Pin GPIO_PIN_2
#define encoder_4_X_GPIO_Port GPIOF
#define amp_4_adc_Pin GPIO_PIN_5
#define amp_4_adc_GPIO_Port GPIOF
#define amp_3_adc_Pin GPIO_PIN_7
#define amp_3_adc_GPIO_Port GPIOF
#define amp_2_adc_Pin GPIO_PIN_9
#define amp_2_adc_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define amp_1_adc_Pin GPIO_PIN_3
#define amp_1_adc_GPIO_Port GPIOC
#define encoder_3_X_Pin GPIO_PIN_2
#define encoder_3_X_GPIO_Port GPIOB
#define encoder_3_A_Pin GPIO_PIN_11
#define encoder_3_A_GPIO_Port GPIOF
#define encoder_3_B_Pin GPIO_PIN_12
#define encoder_3_B_GPIO_Port GPIOF
#define moteur_3_4_DIR_Pin GPIO_PIN_7
#define moteur_3_4_DIR_GPIO_Port GPIOE
#define encoder_1_A_Pin GPIO_PIN_9
#define encoder_1_A_GPIO_Port GPIOE
#define encoder_1_X_Pin GPIO_PIN_10
#define encoder_1_X_GPIO_Port GPIOE
#define encoder_1_B_Pin GPIO_PIN_11
#define encoder_1_B_GPIO_Port GPIOE
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define moteur_2_DIR_Pin GPIO_PIN_10
#define moteur_2_DIR_GPIO_Port GPIOD
#define moteur_1_DIR_Pin GPIO_PIN_11
#define moteur_1_DIR_GPIO_Port GPIOD
#define moteur_1_PUL_Pin GPIO_PIN_12
#define moteur_1_PUL_GPIO_Port GPIOD
#define moteur_2_PUL_Pin GPIO_PIN_13
#define moteur_2_PUL_GPIO_Port GPIOD
#define moteur_3_PUL_Pin GPIO_PIN_14
#define moteur_3_PUL_GPIO_Port GPIOD
#define moteur_4_PUL_Pin GPIO_PIN_15
#define moteur_4_PUL_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define encoder_2_X_Pin GPIO_PIN_8
#define encoder_2_X_GPIO_Port GPIOG
#define encoder_2_A_Pin GPIO_PIN_6
#define encoder_2_A_GPIO_Port GPIOC
#define encoder_2_B_Pin GPIO_PIN_7
#define encoder_2_B_GPIO_Port GPIOC
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define limit_switch_6_Pin GPIO_PIN_0
#define limit_switch_6_GPIO_Port GPIOD
#define limit_switch_5_Pin GPIO_PIN_1
#define limit_switch_5_GPIO_Port GPIOD
#define limit_switch_4_Pin GPIO_PIN_2
#define limit_switch_4_GPIO_Port GPIOD
#define limit_switch_3_Pin GPIO_PIN_3
#define limit_switch_3_GPIO_Port GPIOD
#define limit_switch_2_Pin GPIO_PIN_4
#define limit_switch_2_GPIO_Port GPIOD
#define limit_switch_1_Pin GPIO_PIN_5
#define limit_switch_1_GPIO_Port GPIOD
#define motor_brake_1_Pin GPIO_PIN_6
#define motor_brake_1_GPIO_Port GPIOD
#define motor_brake_2_Pin GPIO_PIN_7
#define motor_brake_2_GPIO_Port GPIOD
#define motor_brake_3_Pin GPIO_PIN_9
#define motor_brake_3_GPIO_Port GPIOG
#define motor_brake_4_Pin GPIO_PIN_10
#define motor_brake_4_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define moteur_4_DIR_Pin GPIO_PIN_1
#define moteur_4_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
