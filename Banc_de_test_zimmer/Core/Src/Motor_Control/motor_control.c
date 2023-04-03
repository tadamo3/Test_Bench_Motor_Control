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
bool g_is_limit_reached = false;

/**
 * @brief
 * External interrupt function when a limit switch is activated - stops all motors and sends an update to the GUI
 * 
 * @param GPIO_Pin The pin number of the affected limit switch
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == limit_switch_vert_1_Pin) || 
        (GPIO_Pin == limit_switch_vert_2_Pin) || 
        (GPIO_Pin == limit_switch_hor_1_Pin) || 
        (GPIO_Pin == limit_switch_hor_2_Pin))
    {
        /* Stops all motors */
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

        g_is_limit_reached = true;
    }
}

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
    if (motor_status_component == MOTOR_FAULT_NONE && (g_is_limit_reached == false))
    {
        motor_index_to_control = serial_data_in->id - OFFSET_INDEX_MOTOR_ARRAY;

        if (serial_data_in->mode == MODE_MANUAL_CONTROL)
        {
            motor_state = motor_control_manual(serial_data_in->command, &g_is_stop_activated, &motor_array[motor_index_to_control]);
        }
        else if (serial_data_in->mode == MODE_POSITION_CONTROL)
        {
            /* Bench test is not supposed to repeat a same movement for safety reasons */
            if (serial_data_in->command != serial_data_in->previous_command)
            {
                /* Send updated trajectory message to GUI */
                serial_build_message(serial_data_in->id, MOTOR_STATE_AUTO_IN_TRAJ, motor_status_component, motor_array[motor_index_to_control].motor_current_position, serial_data_out);
                
                motor_state = motor_control_position(serial_data_in->command, serial_data_in->data, &motor_array[motor_index_to_control]);
            }
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

    serial_data_in->previous_command = serial_data_in->command;
    serial_build_message(serial_data_in->id, motor_state, motor_status_component, motor_array[motor_index_to_control].motor_current_position, serial_data_out);
}

uint8_t motor_control_position(uint8_t direction, uint16_t position_to_reach_mm, Motor * motor)
{
    uint8_t motor_status_movement = MOTOR_STATE_AUTO_END_OF_TRAJ;

    uint32_t max_arr_value = motor->motor_speed;

    float_t pwm_frequency_hz = DISTANCE_PER_REVOLUTION_MM * (CLOCK_FREQUENCY / (1 + max_arr_value));
    float speed_mm_per_sec = pwm_frequency_hz / PULSE;
    float speed_hz_initial = DISTANCE_PER_REVOLUTION_MM * (CLOCK_FREQUENCY / (1 + 65000));
    float speed_mm_per_sec_initial = speed_hz_initial / PULSE;
    float speed_mm_per_sec_stage_mean_value = speed_mm_per_sec_initial + (speed_mm_per_sec - speed_mm_per_sec_initial) * 0.5;

    float_t run_time_ms = 1000 * (50 - 10) / ((2 * ACCELERATION_RATIO * speed_mm_per_sec_stage_mean_value) + ((1 - (2* ACCELERATION_RATIO)) * speed_mm_per_sec));

    /* Start of trajectory */
    verify_change_direction(direction, &g_is_stop_activated, motor);
    HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);

    /*Acceleration in a stages*/
    for(uint8_t a = 1; a <= ACCELERATION_STAGE; a++) //IF THERE IS A DEVIATION, ADD 1 TO ACCELERATION STAGE
    {
        float speed_stage = speed_mm_per_sec * (a / ACCELERATION_STAGE);
        motor->motor_timer->ARR = DISTANCE_PER_REVOLUTION_MM * CLOCK_FREQUENCY / (PULSE * speed_stage) - 1;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
        HAL_Delay((int)((ACCELERATION_RATIO / ACCELERATION_STAGE) * run_time_ms));
        
    }

    /*steady speed*/
    motor->motor_timer->ARR = max_arr_value;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
    HAL_Delay((int)((1 - (2 * ACCELERATION_RATIO)) * run_time_ms));
    

   /*Decceleration in a stages*/
    for(uint8_t a = ACCELERATION_STAGE; a > 0; a--)
    {
        float speed_stage = speed_mm_per_sec * (a / ACCELERATION_STAGE);
        motor->motor_timer->ARR = DISTANCE_PER_REVOLUTION_MM * CLOCK_FREQUENCY / (PULSE * speed_stage) - 1;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
        HAL_Delay((int)((ACCELERATION_RATIO / ACCELERATION_STAGE) * run_time_ms));
    }
    
    /* End of trajectory */
    HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);
    HAL_GPIO_TogglePin(GPIOE, motor->motor_pin_direction);
    HAL_Delay(1000);

    return MOTOR_STATE_AUTO_END_OF_TRAJ;
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
void verify_change_direction(uint8_t direction, bool * is_stop_activated, Motor * motor)
{
    switch (direction)
    {
        case COMMAND_MOTOR_VERTICAL_UP:
            if (motor->motor_direction == MOTOR_STATE_VERTICAL_DOWN)
            {
                motor->motor_direction = MOTOR_STATE_VERTICAL_UP;
                HAL_GPIO_TogglePin(motor->motor_gpio_channel, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_VERTICAL_DOWN:
            if (motor->motor_direction == MOTOR_STATE_VERTICAL_UP)
            {
                motor->motor_direction = MOTOR_STATE_VERTICAL_DOWN;
                HAL_GPIO_TogglePin(motor->motor_gpio_channel, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_HORIZONTAL_RIGHT:
            if (motor->motor_direction == MOTOR_STATE_HORIZONTAL_LEFT)
            {
                motor->motor_direction = MOTOR_STATE_HORIZONTAL_RIGHT;
                HAL_GPIO_TogglePin(motor->motor_gpio_channel, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;

        case COMMAND_MOTOR_HORIZONTAL_LEFT:
            if (motor->motor_direction == MOTOR_STATE_HORIZONTAL_RIGHT)
            {
                motor->motor_direction = MOTOR_STATE_HORIZONTAL_LEFT;
                HAL_GPIO_TogglePin(motor->motor_gpio_channel, motor->motor_pin_direction);
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
    verify_change_direction(direction, is_stop_activated, motor);

    if ((*is_stop_activated == false))
    {
        /* Start PWM if previous command was a stop */
        if ((motor->motor_timer->CR1 & TIM_CR1_CEN) == 0)
        {
            HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
        }

        motor->motor_timer->ARR = motor->motor_speed;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;
	}

    return motor->motor_direction;
}