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
    Encoder encoder_vertical_left = {
        .encoder_timer            = TIM8,
        .encoder_id               = ID_ENCODER_VERTICAL_LEFT,
        .encoder_current_value    = 0u,
        .encoder_past_value       = 0u,
        .encoder_number_of_turns  = 0u
    };

    Encoder encoder_vertical_right = {
        .encoder_timer            = TIM1,
        .encoder_id               = ID_ENCODER_VERTICAL_RIGHT,
        .encoder_current_value    = 0u,
        .encoder_past_value       = 0u,
        .encoder_number_of_turns  = 0u
    };

    Encoder encoder_horizontal = {
        .encoder_timer            = TIM24,
        .encoder_id               = ID_ENCODER_HORIZONTAL,
        .encoder_current_value    = 0u,
        .encoder_past_value       = 0u,
        .encoder_number_of_turns  = 0u
    };

    /* Fill the array of structures */
    encoder_array[INDEX_ENCODER_VERTICAL_LEFT]   = encoder_vertical_left;
    encoder_array[INDEX_ENCODER_VERTICAL_RIGHT]  = encoder_vertical_right;
    encoder_array[INDEX_ENCODER__HORIZONTAL]     = encoder_horizontal;
}

/**
 * @brief
 * Reads the current value of the desired encoder. Updates past and current encoder values. 
 * 
 * @param[inout] encoder Current encoder structure
 * 
 * @return int32_t Current value of the encoder
 */
uint32_t encoder_read_value(Encoder * encoder)
{
    int32_t encoder_value = encoder->encoder_timer->CNT >> 2;

    encoder_value = encoder_value + (encoder->encoder_id << 24);

    /* Update encoder values */
    encoder->encoder_past_value = encoder->encoder_current_value;
    encoder->encoder_current_value = encoder_value;

    return encoder_value;
}

/**
 * @brief 
 * Converts an absolute encoder position in its mm equivalent
 * 
 * @param[in] encoder_position  The absolute position of the encoder
 * 
 * @return The converted encoder position in mm 
 */
float_t convert_encoder_position_to_mm(int32_t encoder_position)
{
    float_t position_mm = (float)encoder_position;
    position_mm = position_mm / (2048 / 5);

    return position_mm;
}