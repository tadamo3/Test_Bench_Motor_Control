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
#include <stdio.h>
#include <stdbool.h>
#include "gpio.h"
#include "tim.h"
#include "Encoders/encoder.h"
#include "Serial_Communication/serial_com.h"

/* CONSTANTS */
#define MOTOR_COMPLETE_TURN_DISTANCE_MM 5

/* ENUMS */
/**
 * @brief Enum to represent any motors state
 * 
 */
enum motor_state
{
    MOTOR_STATE_FAULT = -1,
    MOTOR_STATE_RESERVED = 0,
    MOTOR_STATE_VERTICAL_UP = 1,
    MOTOR_STATE_VERTICAL_DOWN = 2,
    MOTOR_STATE_HORIZONTAL_RIGHT = 3,
    MOTOR_STATE_HORIZONTAL_LEFT = 4,
    MOTOR_STATE_VERTICAL_STOP = 5,
    MOTOR_STATE_HORIZONTAL_STOP = 6,
};

/* STRUCTURES */
/**
 * @brief Structure to represent a motor's parameter 
 * 
 */
typedef struct Motor
{
    int32_t motor_arr_value;
    float motor_position_mm;
    int32_t motor_position_error_mm;
    int32_t motor_error_integral;
    TIM_HandleTypeDef * motor_htim;
    TIM_TypeDef * motor_timer;
    uint16_t motor_timer_channel;
    float_t motor_timer_old_val_us;
    float_t motor_timer_val_us;
    int32_t motor_direction;
    uint16_t motor_pin_direction;
    Encoder * motor_encoder;
} Motor;

/* FUNCTIONS PROTOTYPES */
void motor_control(float position_to_reach_mm, float error_final, int max_arr_value, Motor * motor);
void motor_control_change_speed(uint8_t motor_id, uint16_t speed, Motor * motor);
void motor_control_manual(uint8_t direction, bool * is_stop_activated, Motor * motor);

#endif /* _MOTOR_CONTROL_H_ */