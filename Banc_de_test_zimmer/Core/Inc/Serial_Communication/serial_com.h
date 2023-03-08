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

/* CONSTANTS */
#define SIZE_BUFFER 10
#define INDEX_DATA_FIRST_BYTE   0
#define INDEX_DATA_SECOND_BYTE  1
#define INDEX_COMMAND_BYTE      2
#define INDEX_ID_BYTE           3

#define MASK_MODE       0x80
#define MASK_COMMAND    0x7F


/* STRUCTURES */
typedef struct SerialDataIn
{
    uint8_t * buffer;
    uint8_t mode;
    uint8_t id;
    uint8_t command;
    uint16_t data;
} SerialDataIn;

/* ENUMS */
enum ID
{
    ID_RESERVED                 = 0,
    ID_ENCODER_VERTICAL_LEFT    = 1,
    ID_ENCODER_VERTICAL_RIGHT   = 2,
    ID_ENCODER_HORIZONTAL       = 3,
    ID_MOTOR_VERTICAL_LEFT      = 4,
    ID_MOTOR_VERTICAL_RIGHT     = 5,
    ID_MOTOR_HORIZONTAL         = 6,
};

enum COMMANDS
{
    COMMAND_RESERVED                    = 0,
    COMMAND_MOTOR_VERTICAL_UP           = 1,
    COMMAND_MOTOR_VERTICAL_DOWN         = 2,
    COMMAND_MOTOR_VERTICAL_STOP         = 3,
    COMMAND_MOTOR_HORIZONTAL_RIGHT      = 4,
    COMMAND_MOTOR_HORIZONTAL_LEFT       = 5,
    COMMAND_MOTOR_CHANGE_SPEED          = 6,
    COMMAND_READ_ENCODER_VERTICAL_LEFT  = 7,
    COMMAND_READ_ENCODER_VERTICAL_RIGHT = 8,
    COMMAND_READ_ENCODER_HORIZONTAL     = 9,
    COMMAND_ENABLE_MANUAL_MODE          = 10,
    COMMAND_ENABLE_AUTOMATIC_MODE       = 11,
};

enum MODES
{
    MODE_MANUAL_CONTROL     = 0,
    MODE_POSITION_CONTROL   = 1,
};

/* FUNCTIONS PROTOTYPES */
void serial_data_transmit(UART_HandleTypeDef * uart_channel, uint32_t * data_to_transmit);
void serial_data_parser(SerialDataIn * serial_data_in);

#endif /* _SERIAL_COM_H_ */