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
#include <math.h>

#include "tim.h"
#include "Motor_Control/motor_control.h"

/* Global flags */
bool g_is_stop_activated = true;

/* FUNCTIONS */
static inline uint8_t verify_motor_id(SerialDataIn * serial_data_in)
{
    uint8_t is_motor_id_valid = MOTOR_FAULT_INVALID_ID;
    if (serial_data_in->id >= OFFSET_INDEX_MOTOR_ARRAY)
    {
        is_motor_id_valid = MOTOR_FAULT_NONE;
    }

    return is_motor_id_valid;
}

void motor_control_dispatch(SerialDataIn * serial_data_in, SerialDataOut * serial_data_out, Motor * motor_array)
{
    /* Set reference states and faults */
    uint8_t motor_index_to_control = ID_RESERVED;
    uint8_t motor_state = MOTOR_STATE_RESERVED;

    uint8_t motor_status_component = verify_motor_id(serial_data_in);
    if (motor_status_component == MOTOR_FAULT_NONE)
    {
        motor_index_to_control = serial_data_in->id - OFFSET_INDEX_MOTOR_ARRAY;

        if (serial_data_in->mode == MODE_MANUAL_CONTROL)
        {
            motor_state = motor_control_manual(serial_data_in->command, &g_is_stop_activated, &motor_array[motor_index_to_control]);
        }
        else if (serial_data_in->mode == MODE_POSITION_CONTROL)
        {   
            motor_state = motor_control_position((float_t)serial_data_in->data, motor_array[motor_index_to_control].motor_current_position, 0.25f, 4*28000, &motor_array[motor_index_to_control]);
        }
        else if (serial_data_in->mode == MODE_CHANGE_PARAMS)
        {
            motor_state = motor_change_params(serial_data_in->command, serial_data_in->data, &motor_array[motor_index_to_control]);
        }
        else
        {
            /* Do nothing */
        }
    }

    serial_build_message(serial_data_in->id, motor_state, motor_status_component, motor_array[motor_index_to_control].motor_current_position, serial_data_out);
}

uint8_t motor_control_position(float position_to_reach_mm, uint32_t current_position, float error_final, int max_arr_value, Motor * motor)
{
    uint8_t motor_status_movement = MOTOR_STATE_AUTO_END_OF_TRAJ;

    max_arr_value = motor->motor_speed;
    float_t pwm_frequency_hz_stage1 = DISTANCE_PER_REVOLUTION_MM * (CLOCK_FREQUENCY / (1 + (max_arr_value*4)));
    pwm_frequency_hz_stage1 = pwm_frequency_hz_stage1 / 400;
    
    float_t pwm_frequency_hz_stage2 = DISTANCE_PER_REVOLUTION_MM * (CLOCK_FREQUENCY / (1 + (max_arr_value*2)));
    pwm_frequency_hz_stage2 = pwm_frequency_hz_stage2 / 400;

    float_t pwm_frequency_hz = DISTANCE_PER_REVOLUTION_MM * (CLOCK_FREQUENCY / (1 + max_arr_value));
    pwm_frequency_hz = pwm_frequency_hz / 400;

    float speed_mm_per_sec = pwm_frequency_hz;
    float_t run_time_ms = 1000 * (50 - 10) / ((0.20*pwm_frequency_hz_stage1)+(0.20*pwm_frequency_hz_stage2)+(0.6*speed_mm_per_sec));

    /* Start of trajectory */
    HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);


    /*Acceleration stage 1*/   
    motor->motor_timer->ARR = max_arr_value*4;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)(0.10*run_time_ms));

    
    /*Acceleration stage 2*/
    motor->motor_timer->ARR = max_arr_value*2;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)(0.10*run_time_ms));


    /*steady speed*/
    motor->motor_timer->ARR = max_arr_value;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)(0.6*run_time_ms));

    /*Decceleration stage 1*/
    motor->motor_timer->ARR = max_arr_value*2;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)(0.10*run_time_ms));

    
    /*Acceleration stage 2*/
    motor->motor_timer->ARR = max_arr_value*4;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)(0.10*run_time_ms));
    
    

    /* End of trajectory */
    HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);
    HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    HAL_Delay(300);

    

    return motor_status_movement;
}

uint8_t motor_change_params(uint8_t command, uint16_t data, Motor * motor)
{
    uint8_t motor_status = MOTOR_STATE_RESERVED;

    if (command == COMMAND_MOTOR_CHANGE_SPEED)
    {
        motor->motor_speed = MINIMUM_MOTOR_SPEED - (INTERVAL_SPEED * data);
        motor_status = MOTOR_STATE_CHANGE_PARAMS;
    }

    return motor_status;
}

/**
 * @brief
 * Verifies if a change of direction is needed according to the PID speed and the motors current position
 * 
 * @param[in] pid_speed     The current PID speed   
 * @param[inout] motor      Current motor structure
 */
void verify_change_direction(float_t motor_position_error, Motor * motor)
{
    /* Change directions if needed to reach desired position */
    if ((motor_position_error < 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_UP))
    {
        motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
        HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    } 
    else if ((motor_position_error > 0) && (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN))
    {
        motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
        HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    }
    else
    {
        /* Do nothing */
    }
}

/**
 * @brief
 * Manual control function controlling every motor seperately
 * 
 * @param direction 
 * @param is_stop_activated 
 * @param motor 
 * @return uint8_t 
 */
uint8_t motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor)
{
    switch (direction)
    {
        case COMMAND_MOTOR_VERTICAL_UP:
            if (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN)
            {
                motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
                HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_VERTICAL_DOWN:
            if (motor->motor_direction == MOTOR_STATE_VERTICAL_UP)
            {
                motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
                HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_HORIZONTAL_RIGHT:
            if (motor->motor_direction == MOTOR_STATE_HORIZONTAL_LEFT)
            {
                motor->motor_direction = MOTOR_STATE_HORIZONTAL_RIGHT;
                HAL_GPIO_TogglePin(GPIOA, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_HORIZONTAL_LEFT:
            if (motor->motor_direction == MOTOR_STATE_HORIZONTAL_RIGHT)
            {
                motor->motor_direction = MOTOR_STATE_HORIZONTAL_LEFT;
                HAL_GPIO_TogglePin(GPIOA, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;
        
        case COMMAND_MOTOR_VERTICAL_STOP:
            *is_stop_activated = true;
            HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);

            break;
        
        case COMMAND_MOTOR_HORIZONTAL_STOP:
            *is_stop_activated = true;
            HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);

            break;
    }

    /* Activate PWM if stop is not activated */
    if ((*is_stop_activated == false))
    {
        HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
        motor->motor_timer->ARR = motor->motor_speed;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
	}

    return direction;
}