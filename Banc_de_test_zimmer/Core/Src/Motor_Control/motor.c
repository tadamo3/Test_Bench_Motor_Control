/**
 * @file
 * motor.c
 * 
 * @brief
 * Motor related functions to manage the motors.
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* INCLUDES */
#include "Motor_Control/motor.h"

/* FUNCTIONS */
/**
 * @brief 
 * Generates all motor structures for main application
 * 
 * @param[inout] motor_array  Array of motor structures
 * @param[in] encoder_array   Array of encoder structures to link to specific motors
 */
void motor_init(Motor * motor_array, Encoder * encoder_array)
{
  Motor motor_vertical_left = {
    .motor_arr_value = 72000u,
    .motor_position_mm = 0u,
    .motor_position_error_mm = 0u,
    .motor_error_integral = 0u,
    .motor_htim = &htim2,
    .motor_timer = TIM2,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_timer_old_val_us = 0u,
    .motor_timer_val_us = 0u,
    .motor_direction = MOTOR_STATE_VERTICAL_DOWN,
    .motor_pin_direction = moteur_3_4_DIR_Pin,
    .motor_encoder = &encoder_array[INDEX_ENCODER_1],
  };

  Motor motor_vertical_right = {
    // check arguments (for now are the same as motor_vertical_left)
    .motor_arr_value = 72000u,
    .motor_position_mm = 0u,
    .motor_position_error_mm = 0u,
    .motor_error_integral = 0u,
    .motor_htim = &htim2,
    .motor_timer = TIM2,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_timer_old_val_us = 0u,
    .motor_timer_val_us = 0u,
    .motor_direction = MOTOR_STATE_VERTICAL_DOWN,
    .motor_pin_direction = moteur_3_4_DIR_Pin,
    .motor_encoder = &encoder_array[INDEX_ENCODER_1],
  };

  Motor motor_horizontal = {
    // check arguments (for now are the same as motor_vertical_left)
    .motor_arr_value = 72000u,
    .motor_position_mm = 0u,
    .motor_position_error_mm = 0u,
    .motor_error_integral = 0u,
    .motor_htim = &htim2,
    .motor_timer = TIM2,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_timer_old_val_us = 0u,
    .motor_timer_val_us = 0u,
    .motor_direction = MOTOR_STATE_VERTICAL_DOWN,
    .motor_pin_direction = moteur_3_4_DIR_Pin,
    .motor_encoder = &encoder_array[INDEX_ENCODER_1],
  };

    /* Fill the array of structures */
    motor_array[INDEX_MOTOR_VERTICAL_LEFT]  = motor_vertical_left;
    motor_array[INDEX_MOTOR_VERTICAL_RIGHT] = motor_vertical_right; 
    motor_array[INDEX_MOTOR_HORIZONTAL]     = motor_horizontal; 
}
