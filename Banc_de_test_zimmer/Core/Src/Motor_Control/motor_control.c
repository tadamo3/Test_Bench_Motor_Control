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

/* EXTERN VARIABLES */
extern Motor motor_vertical_left;

/* FUNCTIONS */
void motor_control(float position_to_reach_mm, int error_final, int max_arr_value, Motor * motor)
{
    int KP = 1;
    int KI = 1;
    int KD = 1;

    motor->motor_position_mm = encoder_read_value(motor->motor_encoder);
    //motor->motor_position_error_mm = position_to_reach_mm - motor->motor_position_mm;
    //float current_error = position_to_reach_mm - motor->motor_position_mm;
    
    //float time_difference_us = motor->motor_timer_val_us - motor->motor_timer_old_val_us;

    float to_compare = (float)motor->motor_position_mm / 1024;
    if (to_compare > position_to_reach_mm)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }

    /* Loop until convergence of error values */
    /*
    while (current_error >= error_final)
    {
        motor->motor_position_mm = (encoder_read_value(motor->motor_encoder) / ENCODER_FULL_TURN_SHIFT) * MOTOR_COMPLETE_TURN_DISTANCE_MM;
        motor->motor_position_error_mm = position_to_reach_mm - motor->motor_position_mm;

        motor->motor_timer_val_us = __HAL_TIM_GET_COUNTER(motor->motor_htim) / 72;
        motor->motor_error_integral = (motor->motor_error_integral + current_error) * time_difference_us;
        float X = (KP * current_error) + (KI * motor->motor_error_integral) + (KD * ((current_error - motor->motor_position_error_mm) / time_difference_us));

        if ((X < 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_UP))
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
        } 
        else if ((X > 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN))
        {
            motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
        }
        else
        {
        }
        HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);

        motor->motor_arr_value = ((motor->motor_direction * DP * (1 / PULSE) * Fclock) / X) - 1;
        
        if (motor->motor_arr_value < max_arr_value) 
        {
            motor->motor_arr_value = max_arr_value; 
        }

        motor->motor_timer_old_val_us = motor->motor_timer_val_us;
        TIM2->ARR = motor->motor_arr_value; //comment rendre modulaire??
        TIM2->CCR1 = (TIM2->ARR)/2;
    }
    */
     //Stops PWM for motor X after convergence
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