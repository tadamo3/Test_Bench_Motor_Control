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
#include "Serial_Communication/serial_com.h"
#include "tim.h"

/*Microcontroler and driver properties*/
int Fclock = 72000000;        //clock frequency
int DP = 5;                   //distance per rotation
int PULSE = 400;              //Pulse/revolution

/* EXTERN VARIABLES */
extern Motor motor_vertical_left;

/* FUNCTIONS */
float_t convert_encoder_position_to_mm(int32_t encoder_position)
{
    float_t position_mm = (float)encoder_position;
    position_mm = position_mm / (2048/5);

    return position_mm;
}

void motor_control_position(float position_to_reach_mm, int32_t current_position, float error_final, int max_arr_value, Motor * motor)
{
    int KP = 1;
    int KI = 1;
    int KD = 1;

    float_t current_position_mm = convert_encoder_position_to_mm(current_position);
    motor->motor_position_error_mm = position_to_reach_mm - current_position_mm;
    //float current_error = position_to_reach_mm - motor->motor_position_mm;
    
    float time_difference_us = motor->motor_timer_val_us - motor->motor_timer_old_val_us;

    /* Loop until convergence of error values */
    while (current_position_mm >= error_final)
    {
        uint32_t encoder_position = encoder_read_value(motor->motor_encoder) & 0xFFFFFFFF;
        current_position_mm = convert_encoder_position_to_mm(encoder_position);
        motor->motor_position_error_mm = position_to_reach_mm - motor->motor_current_position;

        motor->motor_timer_val_us = __HAL_TIM_GET_COUNTER(motor->motor_htim) / 72;
        motor->motor_error_integral = (motor->motor_error_integral + motor->motor_position_error_mm) * time_difference_us;
        float X = (KP * motor->motor_position_error_mm) + (KI * motor->motor_error_integral) + (KD * ((motor->motor_position_error_mm - motor->motor_position_error_mm) / time_difference_us));

        if ((X < 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_UP))
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
            HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
        } 
        else if ((X > 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN))
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
            HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
        }
        else
        {
        }

        motor->motor_arr_value = (Fclock / X) - 1;
        
        if (motor->motor_arr_value < max_arr_value) 
        {
            motor->motor_arr_value = max_arr_value; 
        }

        motor->motor_timer_old_val_us = motor->motor_timer_val_us;
        TIM2->ARR = motor->motor_arr_value; //comment rendre modulaire??
        TIM2->CCR1 = (TIM2->ARR)/2;
    }
    
     //Stops PWM for motor X after convergence
     
    if (motor->motor_position_error_mm <= error_final)
    {
       HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
       HAL_Delay(10000);
    }
    

} 

void motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor)
{
    if (direction == COMMAND_MOTOR_VERTICAL_UP)
    {
        if (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN)
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
            HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
        }

        *is_stop_activated = false; 
    } 
    else if (direction == COMMAND_MOTOR_VERTICAL_DOWN)
    {
        if (motor->motor_direction == MOTOR_STATE_VERTICAL_UP)
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
            HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
        }

        *is_stop_activated = false;
    }

    if ((!(TIM2->CR1 & TIM_CR1_CEN)) && (*is_stop_activated == false)) //Checks if PWM is NOT started
    {
        /*----------------------------------------------
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        TIM2->ARR = (Fclock * DP / (speed * PULSE)) - 1;
        TIM2->CCR1 = (TIM2->ARR)/2;
        ----------------------------------------------*/
        HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
        motor->motor_timer->ARR = 4 * 28000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
	}
    
    //Add end trajectory sensor condition
    if (direction == COMMAND_MOTOR_VERTICAL_STOP)
    {
        *is_stop_activated = true;
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
}

/* FUNCTIONS */
void motor_control_change_speed(uint8_t motor_id, uint16_t speed, Motor * motor)
{
    if (motor_id == ID_MOTOR_VERTICAL_LEFT)
    {
        motor->motor_timer->ARR = 9 * 28000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    }
}