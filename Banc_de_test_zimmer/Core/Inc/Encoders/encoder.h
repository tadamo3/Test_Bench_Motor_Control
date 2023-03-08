/**
 * @file
 * encoder.h
 * 
 * @brief
 * Header file for encoder functions to read values coming from encoders.
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _ENCODER_H_
#define _ENCODER_H_

/* INCLUDES */
#include <stdio.h>
#include <float.h>
#include <math.h>
#include "stm32h723xx.h"

/* CONSTANTS */
#define NUMBER_OF_ENCODERS 4
#define INDEX_ENCODER_1 0
#define INDEX_ENCODER_2 1
#define INDEX_ENCODER_3 2
#define INDEX_ENCODER_4 3
#define ENCODER_FULL_TURN_SHIFT 2048

/* STRUCTURES */
/**
 * @brief Structure to represent a stepper motor encoder
 * 
 */
typedef struct Encoder
{
    TIM_TypeDef * encoder_timer;
    uint8_t encoder_id;
    int32_t encoder_current_value;
    int32_t encoder_past_value;
    uint32_t encoder_number_of_turns;
} Encoder;

/* FUNCTIONS PROTOTYPES */
int32_t encoder_read_value(Encoder * encoder);
void encoder_init(Encoder * encoder_array);
float_t convert_encoder_position_to_mm(int32_t encoder_position);

#endif /* _ENCODER_H_ */