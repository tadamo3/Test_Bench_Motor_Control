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
#include "Motor_Control/motor_control.h"

void serial_data_dispatch(SerialDataIn * parsed_serial_data, Motor * motor);

/* FUNCTIONS */
/**
 * @brief
 * Sends via the serial bus data to transmit to the GUI 
 * 
 * @param[in] uart_channel      UART channel to send the information through
 * @param[in] data_to_transmit  Pointer to the transmit buffer to send to the GUI
 */
void serial_data_transmit(UART_HandleTypeDef * uart_channel, uint32_t * data_to_transmit)
{
    HAL_UART_Transmit(uart_channel, (uint8_t *)data_to_transmit, sizeof(data_to_transmit), 0xF);
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
    serial_data_in->command = serial_data_in->buffer[INDEX_COMMAND_BYTE];
    serial_data_in->data = (serial_data_in->buffer[INDEX_DATA_FIRST_BYTE] + (serial_data_in->buffer[INDEX_DATA_SECOND_BYTE] << 8));
}

void serial_data_dispatch(SerialDataIn * parsed_serial_data, Motor * motor)
{
    if (parsed_serial_data->command == COMMAND_MOTOR_VERTICAL_UP)
    {
       motor_control_manual(parsed_serial_data->command, 0, motor);
    }
    else if (parsed_serial_data->command == COMMAND_MOTOR_VERTICAL_DOWN)
    {

    }
    else if (parsed_serial_data->command == COMMAND_MOTOR_VERTICAL_STOP)
    {
        
    }
}
