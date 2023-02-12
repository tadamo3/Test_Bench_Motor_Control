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

/* CONSTANTS */
#define SIZE_BUFFER_OUT 100

/* FUNCTIONS */
/**
 * @brief
 * Sends via the serial bus data to transmit to the GUI 
 * 
 * @param[in] uart_channel      UART channel to send the information through
 * @param[in] data_to_transmit  Pointer to the data to send to the GUI
 */
void transmit_serial_data(UART_HandleTypeDef * uart_channel, void * data_to_transmit)
{
    char serial_buffer_out[SIZE_BUFFER_OUT];

    sprintf(serial_buffer_out, "%d\r\n", *(int32_t *)data_to_transmit);
    HAL_UART_Transmit(uart_channel, (uint8_t *)serial_buffer_out, strlen(serial_buffer_out), 0xF);
}