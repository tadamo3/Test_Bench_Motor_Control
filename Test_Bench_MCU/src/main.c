/**
 * @file main.c
 * @brief 
 * Main file for MCU control in the Zimmer Biomet Test Bench project
 * 
 * @copyright Copyright Zimmer Biomet (c) 2022
 * 
 */

#include "gpio.h"

#define DIR_Pin   GPIO_PIN_10 //Direction pin to motor driver
#define STEP_Pin  GPIO_PIN_11 //Step pin to motor driver
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

    CW_rotation();
    
    HAL_Delay(2000);
  }
}

//Completes 1 full rotation
void CW_rotation(void){
  HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_SET);//Clock wise rotation (SET=CW,RESET=CCW)
		//Moving stepper motor forward for 1 full rotation 
		for(int i=1;i<=200;i++){  //200 steps of 1.8°
			HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
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
