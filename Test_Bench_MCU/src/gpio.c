/**
 * @file gpio.c
 * @brief
 * GPIO handling file used to implement GPIO behavior
 * 
 * @copyright Copyright Zimmer Biomet (c) 2022
 * 
 */

#include "gpio.h"

/**
 * @brief Initialise a GPIO structure
 * 
 * @param pin                   GPIO pin to be configured
 * @param mode                  Operating mode for the selected pins
 * @param pull                  Pull-up or pull-down activation
 * @param speed                 Speed of selected pin
 * @param periph_to_connect     Peripheral to connect to the GPIO
 * @param gpio_struct           GPIO structure to be initialised
 */
void gpio_init_struct(uint32_t pin, uint32_t mode, uint32_t pull, 
                        uint32_t speed, GPIO_TypeDef * periph_to_connect, GPIO_InitTypeDef * gpio_struct)
{
    gpio_struct->Pin = pin;
    gpio_struct->Mode = mode;
    gpio_struct->Pull = pull;
    gpio_struct->Speed = speed;

    HAL_GPIO_Init(periph_to_connect, gpio_struct);
}