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

/* FUNCTIONS */
void motor_control_position(float position_to_reach_mm, int32_t current_position, float error_final, int max_arr_value, Motor * motor)
{
    int KP = 1;
    int KI = 1;
    int KD = 1;

    float_t current_position_mm = convert_encoder_position_to_mm(current_position);

    /* Update position errors for adjustement of PID */
    motor->motor_previous_position_error_mm = motor->motor_current_position_error_mm;
    motor->motor_current_position_error_mm = position_to_reach_mm - current_position_mm;
    
    float time_difference_us = motor->motor_timer_val_us - motor->motor_timer_old_val_us;

    /* PID processing */
    if (current_position_mm >= error_final)
    {
        motor->motor_timer_val_us = __HAL_TIM_GET_COUNTER(motor->motor_htim) / 72;
        motor->motor_error_integral = (motor->motor_error_integral + motor->motor_current_position_error_mm) * time_difference_us;

        float pid_non_converted_speed = (KP * motor->motor_current_position_error_mm) + 
                                        (KI * motor->motor_error_integral) + 
                                        (KD * ((motor->motor_current_position_error_mm - motor->motor_previous_position_error_mm) / time_difference_us));

        verify_change_direction(pid_non_converted_speed, motor);

        motor->motor_arr_value = (CLOCK_FREQUENCY / pid_non_converted_speed) - 1;
        if (motor->motor_arr_value < max_arr_value)
        {
            motor->motor_arr_value = max_arr_value;
        }

        motor->motor_timer_old_val_us = motor->motor_timer_val_us;
        TIM2->ARR = motor->motor_arr_value;
        TIM2->CCR1 = (TIM2->ARR)/2;
    }
    
    /* Stop condition */
    if (motor->motor_current_position_error_mm <= error_final)
    {
       HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
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
        TIM2->ARR = (CLOCK_FREQUENCY * DP / (speed * PULSE)) - 1;
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

/**
 * @brief
 * Verifies if a change of direction is needed according to the PID speed and the motors current position
 * 
 * @param[in] pid_speed     The current PID speed   
 * @param[inout] motor      Current motor structure
 */
void verify_change_direction(float_t pid_speed, Motor * motor)
{
    /* Change directions if needed to reach desired position */
    if ((pid_speed < 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_UP))
    {
        motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
        HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    } 
    else if ((pid_speed > 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN))
    {
        motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
        HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    }
    else
    {
        /* Do nothing here */
    }
}

void motor_control_change_speed(uint8_t motor_id, uint16_t speed, Motor * motor)
{
    if (motor_id == ID_MOTOR_VERTICAL_LEFT)
    {
        motor->motor_timer->ARR = 9 * 28000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    }
}