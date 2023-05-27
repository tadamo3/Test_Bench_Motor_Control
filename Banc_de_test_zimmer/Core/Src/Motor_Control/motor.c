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
    .motor_id = ID_MOTOR_VERTICAL_LEFT,
    .motor_current_position = 0u,
    .motor_htim = &htim2,
    .motor_timer = TIM2,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_direction = MOTOR_STATE_VERTICAL_UP,
    .motor_speed = INITIAL_MOTOR_SPEED,
    .motor_gpio_channel = GPIOE,
    .motor_pin_direction = motor_vertical_left_right_dir_Pin,
    .motor_encoder = &encoder_array[INDEX_ENCODER_VERTICAL_LEFT],
  };

  motor_vertical_left.motor_timer->ARR = motor_vertical_left.motor_speed;
  motor_vertical_left.motor_timer->CCR1 = motor_vertical_left.motor_timer->ARR / 2;

  Motor motor_vertical_right = {
    .motor_id = ID_MOTOR_VERTICAL_RIGHT,
    .motor_current_position = 0u,
    .motor_htim = &htim2,
    .motor_timer = TIM2,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_direction = MOTOR_STATE_VERTICAL_UP,
    .motor_speed = INITIAL_MOTOR_SPEED,
    .motor_gpio_channel = GPIOE,
    .motor_pin_direction = motor_vertical_left_right_dir_Pin,
    .motor_encoder = &encoder_array[INDEX_ENCODER_VERTICAL_RIGHT],
  };

  motor_vertical_right.motor_timer->ARR = motor_vertical_right.motor_speed;
  motor_vertical_right.motor_timer->CCR1 = motor_vertical_right.motor_timer->ARR / 2;

  Motor motor_horizontal = {
    .motor_id = ID_MOTOR_HORIZONTAL,
    .motor_current_position = 0u,
    .motor_htim = &htim8,
    .motor_timer = TIM8,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_direction = MOTOR_STATE_HORIZONTAL_LEFT,
    .motor_speed = INITIAL_MOTOR_SPEED,
    .motor_gpio_channel = GPIOA,
    .motor_pin_direction = motor_horizontal_dir_Pin,
    .motor_encoder = &encoder_array[INDEX_MOTOR_HORIZONTAL],
  };

  motor_horizontal.motor_timer->ARR = motor_horizontal.motor_speed;
  motor_horizontal.motor_timer->CCR1 = motor_horizontal.motor_timer->ARR / 2;

  Motor motor_adapt = {
    .motor_id = ID_MOTOR_ADAPT,
    .motor_current_position = 0u,
    .motor_htim = &htim4,
    .motor_timer = TIM4,
    .motor_timer_channel = TIM_CHANNEL_1,
    .motor_direction = MOTOR_STATE_ADAPT_UP,
    .motor_speed = INITIAL_MOTOR_SPEED,
    .motor_gpio_channel = GPIOE,
    .motor_pin_direction = mototr_adapt_dir_Pin,
    .motor_encoder = NULL,
  };

  motor_adapt.motor_timer->ARR = motor_adapt.motor_speed;
  motor_adapt.motor_timer->CCR1 = motor_adapt.motor_timer->ARR / 2;

    /* Fill the array of structures */
    motor_array[INDEX_MOTOR_VERTICAL_LEFT]  = motor_vertical_left;
    motor_array[INDEX_MOTOR_VERTICAL_RIGHT] = motor_vertical_right; 
    motor_array[INDEX_MOTOR_HORIZONTAL]     = motor_horizontal;
    motor_array[INDEX_MOTOR_ADAPT]          = motor_adapt;
}
