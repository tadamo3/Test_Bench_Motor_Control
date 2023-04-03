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

#define MASK_MODE       0xE0
#define MASK_COMMAND    0x1F


/* STRUCTURES */
typedef struct SerialDataIn
{
    uint8_t * buffer;
    uint8_t mode;
    uint8_t id;
    uint8_t command;
    uint8_t previous_command;
    uint16_t data;
} SerialDataIn;

typedef struct SerialDataOut
{
    uint32_t * buffer;
    uint32_t message_to_send;
    UART_HandleTypeDef * uart_channel;
    size_t size_buffer;
} SerialDataOut;

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
    COMMAND_MOTOR_HORIZONTAL_STOP       = 6,
    COMMAND_MOTOR_CHANGE_SPEED          = 7,
};

enum MODES
{
    MODE_RESERVED           = 0,
    MODE_MANUAL_CONTROL     = 1,
    MODE_POSITION_CONTROL   = 2,
    MODE_RESET              = 3,
    MODE_CHANGE_PARAMS      = 4,
};

/* FUNCTIONS PROTOTYPES */
void serial_data_transmit(UART_HandleTypeDef * uart_channel, uint32_t * data_to_transmit, size_t size);
void serial_data_parser(SerialDataIn * serial_data_in);
void serial_build_message(uint8_t motor_id, uint8_t status_motor, uint8_t status_movement_motor, uint32_t position, SerialDataOut * serial_data_out);

#endif /* _SERIAL_COM_H_ */