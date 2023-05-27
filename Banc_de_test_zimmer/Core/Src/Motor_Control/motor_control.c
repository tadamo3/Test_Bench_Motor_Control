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
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

        g_is_limit_reached = true;
    }
}

/* FUNCTIONS */
/**
 * @brief
 * External interrupt function when a limit switch is activated - stops all motors and sends an update to the GUI
 * 
 * @param[in] direction             Direction state
 * @param[in] position_to_reach_mm  Distance to travel from current position (mm)
 * @param[inout] motor              Current motor structure
 */
static inline uint8_t verify_motor_id(SerialDataIn * serial_data_in)
{
    uint8_t is_motor_id_valid = MOTOR_FAULT_INVALID_ID;
    if (serial_data_in->id >= OFFSET_INDEX_MOTOR_ARRAY)
    {
        is_motor_id_valid = MOTOR_FAULT_NONE;
    }

    return is_motor_id_valid;
}

/**
 * @brief
 * Dispatches the incoming motor control insctruction from GUI.
 * 
 * @param[in] serial_data_in      Current serial data input structure
 * @param[out] serial_data_out    Current serial data output structure
 * @param[inout] motor_array      Current motor structure
 */
void motor_control_dispatch(SerialDataIn * serial_data_in, SerialDataOut * serial_data_out, Motor * motor_array)
{
    /* Set reference states and faults */
    uint8_t motor_index_to_control = ID_RESERVED;
    uint8_t motor_state = MOTOR_STATE_RESERVED;

    uint8_t motor_status_component = verify_motor_id(serial_data_in);
    if ((motor_status_component == MOTOR_FAULT_NONE) && (g_is_limit_reached == false))
    {
        motor_index_to_control = serial_data_in->id - OFFSET_INDEX_MOTOR_ARRAY;

        if (serial_data_in->mode == MODE_MANUAL_CONTROL)
        {
            motor_state = motor_control_manual(serial_data_in->command, &g_is_stop_activated, &motor_array[motor_index_to_control]);
        }
        else if (serial_data_in->mode == MODE_POSITION_CONTROL)
        {
            /* Bench test cannot repeat the same direction of movements for safety reasons */
            if (serial_data_in->command != serial_data_in->previous_command)
            {
                /* Send updated trajectory message to GUI */
                motor_state = MOTOR_STATE_AUTO_IN_TRAJ;
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

    if (motor_state != MOTOR_STATE_RESERVED)
    {
        serial_data_in->previous_command = serial_data_in->command;

        serial_build_message(serial_data_in->id, motor_state, motor_status_component, motor_array[motor_index_to_control].motor_current_position, serial_data_out);
    }
}

/**
 * @brief
 * Controls the motor speed with acceleration and decceleration stages to reach a desired distance at 
 * a certain cruising speed. 
 * 
 * @param[in] direction             Direction state
 * @param[in] position_to_reach_mm  Distance to travel from current position (mm)
 * @param[inout] motor              Current motor structure
 */
uint8_t motor_control_position(uint8_t direction, uint16_t position_to_reach_mm, Motor * motor)
{
    uint8_t screw_pitch_mm = DISTANCE_PER_TURN_MM;
    float_t amplitude_movement_mm = position_to_reach_mm;

    /* Convert correctly number of turns received in amplitude */
    if (motor->motor_id == ID_MOTOR_ADAPT)
    {
        screw_pitch_mm = 2;

        /* 
            Divide by 10 because of the factor 100 in the GUI that is applied to the number of turns sent
            The remaining factor 10 is to account for the gearbox ratio of 10:1
        */
        amplitude_movement_mm = position_to_reach_mm / 10;
    }

    float_t peak_pwm_frequency_hz = CLOCK_FREQUENCY / ((motor->motor_speed + 1) * (PRESCALER + 1));
    float_t peak_speed_mm_per_sec = ((screw_pitch_mm) * peak_pwm_frequency_hz) / STEPS_PER_TURN;
    
    float_t lowest_pwm_frequency_hz = CLOCK_FREQUENCY / ((MINIMUM_MOTOR_SPEED + 1) * (PRESCALER + 1));
    float_t lowest_speed_mm_per_sec = ((screw_pitch_mm) * lowest_pwm_frequency_hz) / STEPS_PER_TURN;
    
    float_t median_speed_mm_per_sec = lowest_speed_mm_per_sec + ((peak_speed_mm_per_sec - lowest_speed_mm_per_sec) * 0.5);

    float_t run_time_ms = (FACTOR_CONVERSTION_SEC_TO_MS * amplitude_movement_mm) / ((2 * RAMPUP_RATIO * median_speed_mm_per_sec) + ((1 - (2 * RAMPUP_RATIO)) * peak_speed_mm_per_sec));
    float_t speed_increments = (peak_speed_mm_per_sec - lowest_speed_mm_per_sec) / NUMBER_OF_STAGES;

    /* Start of trajectory */
    verify_change_direction(direction, &g_is_stop_activated, motor);
    HAL_TIM_PWM_Start(motor->motor_htim, motor->motor_timer_channel);
    
     float_t delay_in_rampup_ms = (RAMPUP_RATIO / NUMBER_OF_STAGES) * run_time_ms;
    
    /*Acceleration in stages*/
    float_t delay_in_rampup_ms = (RAMPUP_RATIO / NUMBER_OF_STAGES) * run_time_ms;

    for (uint8_t i = 1; i <= NUMBER_OF_STAGES; i++)
    {
        float_t current_speed = lowest_speed_mm_per_sec + (i * speed_increments);
        uint32_t new_arr = ((CLOCK_FREQUENCY * screw_pitch_mm) / (STEPS_PER_TURN * (PRESCALER + 1) * current_speed)) - 1;

        motor->motor_timer->ARR = new_arr;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;

        HAL_Delay((int)delay_in_rampup_ms);
    }

    /* Steady speed */
    float_t current_speed = peak_speed_mm_per_sec;
    uint32_t new_arr = ((CLOCK_FREQUENCY * screw_pitch_mm) / (STEPS_PER_TURN * (PRESCALER + 1) * current_speed)) - 1;

    motor->motor_timer->ARR = new_arr;
    motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;

    float_t delay_in_steady_ms = (1 - (2 * RAMPUP_RATIO)) * run_time_ms;
    HAL_Delay((uint32_t)delay_in_steady_ms);
    
    /*Decceleration in stages*/
    for (uint8_t i = NUMBER_OF_STAGES; i > 0; i--)
    {
        float_t current_speed = lowest_speed_mm_per_sec + (i * speed_increments);
        uint32_t new_arr = ((CLOCK_FREQUENCY * screw_pitch_mm) / (STEPS_PER_TURN * (PRESCALER + 1) * current_speed)) - 1;

        motor->motor_timer->ARR = new_arr;
        motor->motor_timer->CCR1 = motor->motor_timer->ARR / 2;

        HAL_Delay((int)delay_in_rampup_ms);
    }
    
    /* End of trajectory */
    HAL_TIM_PWM_Stop(motor->motor_htim, motor->motor_timer_channel);
    HAL_Delay(300);

    return MOTOR_STATE_AUTO_END_OF_TRAJ;
}

/**
 * @brief
 * Changes motor parameters and status
 * 
 * @param[in] command       Serial command status
 * @param[in] data          Corresponding data to modify a parameter with
 * @param[inout] motor      Current motor structure
 * @param[out] motor_status Motor status    
 */
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
        
        case COMMAND_MOTOR_ADAPT_UP:
            if (motor->motor_direction == MOTOR_STATE_ADAPT_DOWN)
            {
                motor->motor_direction = MOTOR_STATE_ADAPT_UP;
                HAL_GPIO_TogglePin(motor->motor_gpio_channel, motor->motor_pin_direction);
            }
            *is_stop_activated = false;

            break;
        
        case COMMAND_MOTOR_ADAPT_DOWN:
            if (motor->motor_direction == MOTOR_STATE_ADAPT_UP)
            {
                motor->motor_direction = MOTOR_STATE_ADAPT_DOWN;
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
        
        case COMMAND_MOTOR_ADAPT_STOP:
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