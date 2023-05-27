/**
 * @file
 * serial_com.c
 * 
 * @brief
 * Function implementation for transmitting data over serial communication to the GUI.
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* INCLUDES */
#include "Serial_Communication/serial_com.h"

/* FUNCTIONS */
/**
 * @brief
 * Sends via the serial bus data to transmit to the GUI 
 * 
 * @param[in] uart_channel      UART channel to send the information through
 * @param[in] data_to_transmit  Pointer to the transmit buffer to send to the GUI
 */
void serial_data_transmit(UART_HandleTypeDef * uart_channel, uint32_t * data_to_transmit, size_t size)
{
    HAL_UART_Transmit(uart_channel, (uint8_t *)data_to_transmit, size, 0xF);
}

void serial_build_message(uint8_t motor_id, uint8_t status_motor, uint8_t status_movement_motor, uint32_t position, SerialDataOut * serial_data_out)
{
    uint32_t message_to_send = status_motor + (status_movement_motor << 8) + (motor_id << 16);
    serial_data_out->buffer[0] = message_to_send;

    serial_data_transmit(serial_data_out->uart_channel, serial_data_out->buffer, serial_data_out->size_buffer);
}

/**
 * @brief
 * Parses the serial input buffer data received from the GUI
 * 
 * @param[inout] serial_data_in Serial data structure to parse the serial input buffer
 */
void serial_data_parser(SerialDataIn * serial_data_in)
{
    serial_data_in->id = serial_data_in->buffer[INDEX_ID_BYTE];
    serial_data_in->mode = (serial_data_in->buffer[INDEX_COMMAND_BYTE] & MASK_MODE) >> 5;
    serial_data_in->command = serial_data_in->buffer[INDEX_COMMAND_BYTE] & MASK_COMMAND;
    serial_data_in->data = (serial_data_in->buffer[INDEX_DATA_FIRST_BYTE] + (serial_data_in->buffer[INDEX_DATA_SECOND_BYTE] << 8));
}