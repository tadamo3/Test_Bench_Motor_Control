/**
 * @file
 * motor_control.h
 * 
 * @brief
 * Header file for motor control related functions.
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

/* INCLUDES */
#include "Motor_Control/motor.h"

/* CONSTANTS */
#define CLOCK_FREQUENCY 72000000
#define DISTANCE_PER_REVOLUTION_MM 5
#define PULSE 400

/* FUNCTIONS PROTOTYPES */
void motor_control_position(float position_to_reach_mm, int32_t current_position, float error_final, int max_arr_value, Motor * motor);
void motor_control_change_speed(uint8_t motor_id, uint16_t speed, Motor * motor);
void motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor);
void verify_change_direction(float_t pid_speed, Motor * motor);

#endif /* _MOTOR_CONTROL_H_ */