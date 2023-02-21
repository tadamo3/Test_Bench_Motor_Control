/**
 * @file
 * serial_com.h
 * 
 * @brief
 * Header file for serial communication related functions.
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _SERIAL_COM_H_
#define _SERIAL_COM_H_

/* INCLUDES */
#include <stdio.h>
#include "string.h"
#include "usart.h"

/* FUNCTIONS PROTOTYPES */
void transmit_serial_data(UART_HandleTypeDef * uart_channel, void * data_to_transmit);

#endif /* _SERIAL_COM_H_ */