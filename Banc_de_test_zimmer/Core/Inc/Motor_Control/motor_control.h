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
#define OFFSET_INDEX_MOTOR_ARRAY 4

/* FUNCTIONS PROTOTYPES */
uint8_t motor_control_position(float position_to_reach_mm, uint32_t current_position, float error_final, int max_arr_value, Motor * motor);
uint8_t motor_change_params(uint8_t command, uint16_t data, Motor * motor);
uint8_t motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor);
void verify_change_direction(float_t pid_speed, Motor * motor);
void motor_control_dispatch(SerialDataIn * serial_data_in, SerialDataOut * serial_data_out, Motor * motor_array);

#endif /* _MOTOR_CONTROL_H_ */