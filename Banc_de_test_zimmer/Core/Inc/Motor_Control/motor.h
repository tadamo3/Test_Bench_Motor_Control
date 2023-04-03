/**
 * @file
 * motor.h
 * 
 * @brief
 * Header file for motor functions 
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _MOTOR_H_
#define _MOTOR_H_

/* INCLUDES */
#include <stdio.h>
#include <stdbool.h>
#include "stm32h723xx.h"
#include "gpio.h"
#include "tim.h"
#include "Encoders/encoder.h"
#include "Serial_Communication/serial_com.h"


/* CONSTANTS */
#define NUMBER_MOTOR 3
#define INDEX_MOTOR_VERTICAL_LEFT 0
#define INDEX_MOTOR_VERTICAL_RIGHT 1
#define INDEX_MOTOR_HORIZONTAL 2

#define MINIMUM_MOTOR_SPEED 65000
#define MAXIMUM_MOTOR_SPEED 20000
#define INITIAL_MOTOR_SPEED 42500
#define INTERVAL_SPEED 450

/**
 * @brief Enum to represent any motors state
 * 
 */
enum motor_state
{
    MOTOR_STATE_RESERVED            = 0,
    MOTOR_STATE_VERTICAL_UP         = 1,
    MOTOR_STATE_VERTICAL_DOWN       = 2,
    MOTOR_STATE_HORIZONTAL_RIGHT    = 3,
    MOTOR_STATE_HORIZONTAL_LEFT     = 4,
    MOTOR_STATE_VERTICAL_STOP       = 5,
    MOTOR_STATE_HORIZONTAL_STOP     = 6,
    MOTOR_STATE_AUTO_IN_TRAJ        = 7,
    MOTOR_STATE_AUTO_END_OF_TRAJ    = 8,
    MOTOR_STATE_CHANGE_PARAMS       = 9,
};

enum motor_fault
{
    MOTOR_FAULT_NONE        = 0,
    MOTOR_FAULT_INVALID_ID  = 1,
};

/* STRUCTURES */
/**
 * @brief Structure to represent a motor's parameter 
 * 
 */
typedef struct Motor
{
    uint8_t motor_id;
    uint32_t motor_current_position;
    TIM_HandleTypeDef * motor_htim;
    TIM_TypeDef * motor_timer;
    uint16_t motor_timer_channel;
    int32_t motor_direction;
    uint32_t motor_speed;
    GPIO_TypeDef * motor_gpio_channel;
    uint16_t motor_pin_direction;
    Encoder * motor_encoder;
} Motor;

/* FUNCTIONS PROTOTYPES */
void motor_init(Motor * motor_array, Encoder * encoder_array);

#endif /* _MOTOR_H_ */