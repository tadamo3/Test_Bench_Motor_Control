/**
 * @file
 * motor_control.c
 * 
 * @brief
 * Stepper motor control functions for position and spped control.
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* INCLUDES */
#include "Motor_Control/motor_control.h"
#include "tim.h"

/*Microcontroler and driver properties*/
int Fclock = 72000000;        //clock frequency
int DP = 2;                   //distance per rotation
int PULSE = 400;              //Pulse/revolution


/* FUNCTIONS */

void motor_control(int Pd, int efinal, int ARRmax, ControlMotor * motor)
{
  int KP = 1;
  int KI = 1;
  int KD = 1;         
  
  
  //INSERT ENCODER READING

  int e = Pd - motor->P;
  float dT = motor->timer_val_us - motor->timer_old_val_us;
  HAL_TIM_PWM_Start(motor->TIMER, motor->TIM_CHANNEL); //Starts PWM for motor X

  while (e >= efinal)             //loops until convergence
  {
    
    //INSERT ENCODE READING
    motor->timer_val_us = __HAL_TIM_GET_COUNTER(motor->TIMER)/72;
    motor->e = Pd - motor->P;
    motor->eint = (motor->eint + e) * dT;
    float X = KP*e + KI * motor->eint + KD * ((e - motor->e) / dT);
    motor->DIR = 1;
    if ((X<0) && (motor->DIR == 1)) 
    {
        motor->DIR = -1;
        HAL_GPIO_TogglePin(GPIOE,7);
    } 
    else if ((X>0) && (motor->DIR == -1))
    {
        motor->DIR = 1;
        HAL_GPIO_TogglePin(GPIOE,7);
    }
    motor->ARR = ((motor->DIR*DP * (1/PULSE) * Fclock) / X) - 1;
    
    if (motor->ARR < ARRmax) 
    {
      motor->ARR = ARRmax; 
    }
    motor->P = 2*(TIM1->CNT >>2)/2048;
    motor->timer_old_val_us = motor->timer_val_us;
 /*---------------------------------------------------*/
    TIM2->ARR = motor->ARR; //comment rendre modulaire??
    TIM2->CCR1 = (TIM2->ARR)/2;
    /*----------------------------------------------*/
  }

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); //Stops PWM for motor X after convergence
} 

void motor_control_manual(int Direction, int STOP, float V, ControlMotor * motor)
{
    
    if ((Direction == -1) && (motor->DIR == 1)) 
    {
        motor->DIR = -1;
        HAL_GPIO_TogglePin(GPIOE,7);
    } 
    else if ((Direction == 1) && (motor->DIR == -1))
    {
        motor->DIR = 1;
        HAL_GPIO_TogglePin(GPIOE,7);
    }   

    if ( !(TIM2->CR1 & TIM_CR1_CEN)) //Checks if PWM is NOT started
    {
        /*----------------------------------------------*/
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        TIM2->ARR = (Fclock * DP / (V * PULSE)) - 1;
        TIM2->CCR1 = (TIM2->ARR)/2;
        /*----------------------------------------------*/
	}
    
    //Add end trajectory sensor condition
    if (STOP == 1){
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }

/* EXTERN VARIABLES */
extern Encoder * encoder_array;

/* FUNCTIONS */
void motor_control_change_speed(uint8_t motor_id, uint16_t speed)
{
    
}