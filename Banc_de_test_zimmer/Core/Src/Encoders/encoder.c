/**
 * @file
 * encoder.c
 * 
 * @brief
 * Encoder related functions to manage the stepper motor encoders.
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* INCLUDES */
#include "Encoders/encoder.h"

/* FUNCTIONS */
/**
 * @brief
 * Reads the current value of the desired encoder. Updates past and current encoder values. 
 * 
 * @param[inout] encoder Current encoder structure
 * @return int32_t Current value of the encoder
 */
int32_t encoder_read_value(Encoder * encoder)
{
    int32_t encoder_value = encoder->encoder_timer->CNT >> 2;

    /* Update encoder values */
    encoder->encoder_past_value = encoder->encoder_current_value;
    encoder->encoder_current_value = encoder_value;

    return encoder_value;
}