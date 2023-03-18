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
    uint8_t motor_status_movement = MOTOR_STATE_RESERVED;

    uint8_t motor_status_component = verify_motor_id(serial_data_in);
    if (motor_status_component == MOTOR_FAULT_NONE)
    {
        motor_index_to_control = serial_data_in->id - OFFSET_INDEX_MOTOR_ARRAY;

        if (serial_data_in->mode == MODE_MANUAL_CONTROL)
        {
            motor_status_movement = motor_control_manual(serial_data_in->command, &g_is_stop_activated, &motor_array[motor_index_to_control]);
        }
        else if (serial_data_in->mode == MODE_POSITION_CONTROL)
        {
            motor_status_movement = motor_control_position((float_t)serial_data_in->data, motor_array[motor_index_to_control].motor_current_position, 0.25f, 4*28000, &motor_array[motor_index_to_control]);
        }
        else
        {
            /* Nothing to do here */
        }
    }

    serial_build_message(serial_data_in->id, motor_status_component, motor_status_movement, motor_array[motor_index_to_control].motor_current_position, serial_data_out);
}

uint8_t motor_control_position(float position_to_reach_mm, uint32_t current_position, float error_final, int max_arr_value, Motor * motor)
{
    int KP = 10;
    int KI = 5;
    int KD = 1;

    float_t current_position_mm = convert_encoder_position_to_mm(current_position);
    uint8_t motor_status_movement = MOTOR_STATE_RESERVED;

    /* Update position errors for adjustement of PID */
    motor->motor_current_position_error_mm = position_to_reach_mm - (current_position_mm + 0.5);
    
    float time_difference_us = motor->motor_timer_val_us - motor->motor_timer_old_val_us;

    /* PID processing */
    if (fabs(motor->motor_current_position_error_mm) > error_final)
    {
        motor->motor_timer_val_us = __HAL_TIM_GET_COUNTER(motor->motor_htim) / 72;
        motor->motor_error_integral = (motor->motor_error_integral + motor->motor_current_position_error_mm) * time_difference_us;

        float pid_non_converted_speed = (KP * motor->motor_current_position_error_mm) +
                                        (KI * motor->motor_error_integral) +
                                        (KD * ((motor->motor_current_position_error_mm - motor->motor_previous_position_error_mm) / (time_difference_us / 100000)));

        verify_change_direction(motor->motor_current_position_error_mm, motor);

        motor->motor_arr_value = (CLOCK_FREQUENCY / pid_non_converted_speed) - 1;
        if (motor->motor_arr_value < max_arr_value)
        {
            motor->motor_arr_value = max_arr_value;
        }

        motor->motor_timer_old_val_us = motor->motor_timer_val_us;

        HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
        motor->motor_timer->ARR = motor->motor_arr_value;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;

        motor_status_movement = MOTOR_STATE_AUTO_IN_TRAJ;
    }
    else if (fabs(motor->motor_current_position_error_mm) <= error_final)
    {
        //HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);
        motor->motor_timer->ARR = 69000000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
        motor_status_movement = MOTOR_STATE_AUTO_END_OF_TRAJ;
    }
    else
    {
        /* Do nothing here */
    }

    /* Update previous error to keep track of progression of trajectory */
    motor->motor_previous_position_error_mm = motor->motor_current_position_error_mm;

    return motor_status_movement;
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
        /* Do nothing here */
    }
}

uint8_t motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor)
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
    else
    {
        /* Nothing to do here */
    }

    if ((!(TIM2->CR1 & TIM_CR1_CEN)) && (*is_stop_activated == false)) //Checks if PWM is NOT started
    {
        HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
        motor->motor_timer->ARR = 4 * 28000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
	}
    
    //Add end trajectory sensor condition
    if (direction == COMMAND_MOTOR_VERTICAL_STOP)
    {
        *is_stop_activated = true;
        HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);
    }

    return motor->motor_direction;
}

void motor_control_change_speed(uint8_t motor_id, uint16_t speed, Motor * motor)
{
    if (motor_id == ID_MOTOR_VERTICAL_LEFT)
    {
        motor->motor_timer->ARR = 9 * 28000;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    }
}