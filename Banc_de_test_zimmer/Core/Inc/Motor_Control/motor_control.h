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
#include "gpio.h"
#include "tim.h"

/* STRUCTURES */
/**
 * @brief Structure to represent a motor's parameter 
 * 
 */
typedef struct
{
  int ARR;
  int e;
  int DIR;
  int eint;
  float timer_old_val_us;
  float timer_val_us;
  float P;
  TIM_HandleTypeDef * TIMER;
  uint TIM_CHANNEL;
} ControlMotor;

/* FUNCTIONS PROTOTYPES */
void motor_control(int Pd, int efinal, int ARRmax, ControlMotor * motor);
#include "Encoders/encoder.h"

/* FUNCTIONS PROTOTYPES */
void motor_control_change_speed(uint8_t motor_id, uint16_t speed);

#endif /* _MOTOR_CONTROL_H_ */