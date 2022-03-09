
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


/***************************************************************************
	Enumerations :
***************************************************************************/


/***************************************************************************
	Class :
***************************************************************************/

class DelWinch
{
public:
    DelWinch();
    void init();
    void manage();

private:
    AP_HAL::UARTDriver *_winch_port;
    uint8_t* send_buffer;
    uint16_t position;
    uint8_t speed;
    uint8_t direction;
    uint8_t error;
};

#endif