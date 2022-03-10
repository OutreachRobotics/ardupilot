
// del_winch.h

#ifndef DEL_WINCH_H
#define DEL_WINCH_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <stdint.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define WINCH_UART 2
#define WINCH_HEADER 0xFE
#define WINCH_FOOTER 0xDE

#define TX_BUFFER_LEN 4
#define RX_BUFFER_LEN 7

#define WINCH_MID_CHANNEL 1500.0f
#define WINCH_DEADBAND 0.02f
#define WINCH_RANGE 500.0f

#define WINCH_MAX_SPEED 50


/***************************************************************************
	Enumerations :
***************************************************************************/

enum Direction
{
    Neutral,
    Up,
    Down
};

/***************************************************************************
	Class :
***************************************************************************/

class DelWinch
{
public:
    DelWinch();
    void manage();

private:
    AP_HAL::UARTDriver *_winch_port;
    uint8_t tx_buffer[TX_BUFFER_LEN];
    uint16_t position_read;
    uint8_t speed_read;
    uint8_t direction_read;
    uint8_t error;
    uint8_t speed;
    Direction direction;
    float winch_input;

};

#endif