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
#define DISTANCE_PER_TURN_MM 5
#define STEPS_PER_TURN 400
#define RAMPUP_RATIO 0.1
#define NUMBER_OF_STAGES 15
#define PRESCALER 10
#define FACTOR_CONVERSTION_SEC_TO_MS 1000

#define OFFSET_INDEX_MOTOR_ARRAY 4

/* FUNCTIONS PROTOTYPES */
uint8_t motor_control_position(uint8_t direction, uint16_t position_to_reach_mm, Motor * motor);
uint8_t motor_change_params(uint8_t command, uint16_t data, Motor * motor);
uint8_t motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor);
void verify_change_direction(uint8_t direction, bool * is_stop_activated, Motor * motor);
void motor_control_dispatch(SerialDataIn * serial_data_in, SerialDataOut * serial_data_out, Motor * motor_array);

#endif /* _MOTOR_CONTROL_H_ */