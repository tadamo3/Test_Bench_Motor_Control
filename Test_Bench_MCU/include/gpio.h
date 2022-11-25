/**
 * @file gpio.h
 * @brief
 * Header file for gpio.c 
 * 
 * @copyright Copyright Zimmer Biomet (c) 2022
 * 
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f4xx_hal.h"

/* Defines */
#define LED_PIN                     GPIO_PIN_5
#define LED_GPIO_PORT               GPIOA
#define LED_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()

/* Function prototypes */
void gpio_init_struct(uint32_t pin, uint32_t mode, uint32_t pull, 
                        uint32_t speed, GPIO_TypeDef * periph_to_connect, GPIO_InitTypeDef * gpio_struct);

#endif /* _GPIO_H_ */
