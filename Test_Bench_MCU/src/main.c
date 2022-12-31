/**
 * @file main.c
 * @brief 
 * Main file for MCU control in the Zimmer Biomet Test Bench project
 * 
 * @copyright Copyright Zimmer Biomet (c) 2022
 * 
 */

#include "gpio.h"

/**
 * @brief Main process - calls multiple other processes to execute motor control loop
 */
int main(void)
{
  HAL_Init();
  
  LED_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio_led_user_struct;
  gpio_init_struct(LED_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 
                      GPIO_SPEED_FREQ_HIGH, LED_GPIO_PORT, &gpio_led_user_struct);

  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

    HAL_Delay(200);
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}
