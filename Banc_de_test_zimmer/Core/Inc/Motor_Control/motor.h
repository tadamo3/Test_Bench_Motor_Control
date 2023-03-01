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
#include "stm32h723xx.h"
#include "Motor_Control/motor_control.h"

/* CONSTANTS */
#define NUMBER_MOTOR 3
#define INDEX_MOTOR_VERTICAL_LEFT 0
#define INDEX_MOTOR_VERTICAL_RIGHT 1
#define INDEX_MOTOR_HORIZONTAL 2

/* STRUCTURES */

/* FUNCTIONS PROTOTYPES */
void motor_init(Motor * motor_array, Encoder * encoder_array);

#endif /* _MOTOR_H_ */