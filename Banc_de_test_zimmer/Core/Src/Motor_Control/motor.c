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
#include "Motor_Control/motor_control.h"
#include "Serial_Communication/serial_com.h"
#include "Encoders/encoder.c"
#include "Encoders/encoder.h"

/* FUNCTIONS */
/**
 * @brief 
 * Generates all motor structures for main application
 * 
 * @param[out] motor_array Array of motor structures
 */
void motor_init(Motor * motor_array)
{
  // Initialisation of encoder 
  Encoder encoder_array[NUMBER_MAX_ENCODERS];
  encoder_init(encoder_array);

  // Initialisation of the different motor structures

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
    motor_array[0] = motor_vertical_left;
    motor_array[1] = motor_vertical_right; 
    motor_array[2] = motor_horizontal; 
}
