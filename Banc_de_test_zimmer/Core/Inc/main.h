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
#define encoder_vertical_right_A_Pin GPIO_PIN_0
#define encoder_vertical_right_A_GPIO_Port GPIOF
#define encoder_vertical_right_B_Pin GPIO_PIN_1
#define encoder_vertical_right_B_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define encoder_vertical_left_A_Pin GPIO_PIN_0
#define encoder_vertical_left_A_GPIO_Port GPIOA
#define encoder_vertical_left_B_Pin GPIO_PIN_1
#define encoder_vertical_left_B_GPIO_Port GPIOA
#define motor_vertical_left_right_pulse_Pin GPIO_PIN_5
#define motor_vertical_left_right_pulse_GPIO_Port GPIOA
#define encoder_horizontal_Z_Pin GPIO_PIN_2
#define encoder_horizontal_Z_GPIO_Port GPIOB
#define encoder_horizontal_A_Pin GPIO_PIN_11
#define encoder_horizontal_A_GPIO_Port GPIOF
#define encoder_horizontal_B_Pin GPIO_PIN_12
#define encoder_horizontal_B_GPIO_Port GPIOF
#define motor_vertical_left_right_dir_Pin GPIO_PIN_7
#define motor_vertical_left_right_dir_GPIO_Port GPIOE
#define encoder_vertical_right_Z_Pin GPIO_PIN_10
#define encoder_vertical_right_Z_GPIO_Port GPIOE
#define motor_horizontal_pulse_Pin GPIO_PIN_10
#define motor_horizontal_pulse_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_OVCR_EXTI_IRQn EXTI9_5_IRQn
#define motor_horizontal_pul_Pin GPIO_PIN_6
#define motor_horizontal_pul_GPIO_Port GPIOC
#define encoder_vertical_left_Z_Pin GPIO_PIN_8
#define encoder_vertical_left_Z_GPIO_Port GPIOC
#define motor_horizontal_dir_Pin GPIO_PIN_8
#define motor_horizontal_dir_GPIO_Port GPIOA
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
#define limit_switch_hor_2_Pin GPIO_PIN_2
#define limit_switch_hor_2_GPIO_Port GPIOD
#define limit_switch_hor_2_EXTI_IRQn EXTI2_IRQn
#define limit_switch_hor_1_Pin GPIO_PIN_3
#define limit_switch_hor_1_GPIO_Port GPIOD
#define limit_switch_hor_1_EXTI_IRQn EXTI3_IRQn
#define limit_switch_vert_2_Pin GPIO_PIN_4
#define limit_switch_vert_2_GPIO_Port GPIOD
#define limit_switch_vert_2_EXTI_IRQn EXTI4_IRQn
#define limit_switch_vert_1_Pin GPIO_PIN_5
#define limit_switch_vert_1_GPIO_Port GPIOD
#define limit_switch_vert_1_EXTI_IRQn EXTI9_5_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define mototr_adapt_dir_Pin GPIO_PIN_1
#define mototr_adapt_dir_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
