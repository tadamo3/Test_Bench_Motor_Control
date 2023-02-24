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
#include "Serial_Communication/serial_com.h"

/* FUNCTIONS */
/**
 * @brief 
 * Generates all encoder structures for main application
 * 
 * @param[out] encoder_array Array of encoder structures
 */
void encoder_init(Encoder * encoder_array)
{
    Encoder encoder_1 = {
        .encoder_timer            = TIM1,
        .encoder_id               = ID_ENCODER_VERTICAL_LEFT,
        .encoder_current_value    = 0u,
        .encoder_past_value       = 0u,
        .encoder_number_of_turns  = 0u
    };

    /* Fill the array of structures */
    encoder_array[INDEX_ENCODER_1] = encoder_1;
}

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
    encoder_value = encoder_value + (encoder->encoder_id << 24);

    /* Update encoder values */
    encoder->encoder_past_value = encoder->encoder_current_value;
    encoder->encoder_current_value = encoder_value;

    return encoder_value;
}