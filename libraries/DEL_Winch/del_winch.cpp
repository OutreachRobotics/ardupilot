// del_ekf.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_winch.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

DelWinch::DelWinch()
{
    _winch_port = hal.serial(WINCH_UART);
    
    position_read.pos = 0;
    speed_read = 0;
    direction_read = Neutral;
    speed = 0;
    direction = Neutral;
    command = 0;
    error = 0;
    winch_input = 0;

    tx_buffer[0] = WINCH_HEADER;
    tx_buffer[1] = direction;
    tx_buffer[2] = speed;
    tx_buffer[3] = command;
    tx_buffer[4] = WINCH_FOOTER;
}

void DelWinch::init()
{
    _winch_port->begin(57600,50,50);
}

void DelWinch::manage()
{
    // Receiving status from the winch
    while(_winch_port->available()>6)
    {
        if(_winch_port->read() == WINCH_HEADER)
        {
            position_read.byte[1] = _winch_port->read();
            position_read.byte[0] = _winch_port->read(); 
            speed_read = _winch_port->read();
            direction_read = _winch_port->read();
            current = _winch_port->read();
            error = _winch_port->read();
        }
    }

    uint16_t winch_rc_in = hal.rcin->read(WINCH_CHANNEL)<1000 ? WINCH_MID_CHANNEL : hal.rcin->read(WINCH_CHANNEL);
    winch_input = (winch_rc_in-WINCH_MID_CHANNEL)/WINCH_RANGE;

    uint16_t command_rc_in = hal.rcin->read(COMMAND_CHANNEL)<1000 ? WINCH_MID_CHANNEL : hal.rcin->read(COMMAND_CHANNEL);
    command = command_rc_in>WINCH_MID_CHANNEL;

    if(abs(winch_input)>WINCH_DEADBAND)
    {
        direction = winch_input>0.0f ? Up : Down;
        speed = uint8_t(abs(winch_input) * WINCH_MAX_SPEED);
    }
    else
    {
        direction = Neutral;
        speed = 0;
    }

    // Sending commands to the winch
    tx_buffer[1] = (uint8_t)direction;
    tx_buffer[2] = speed;
    tx_buffer[3] = command;
    _winch_port->write(tx_buffer,TX_BUFFER_LEN);   

}

    int16_t DelWinch::getPosition()
    {
        return position_read.pos;
    }

    uint8_t DelWinch::getSpeed()
    {
        return speed_read;
    }

    uint8_t DelWinch::getSpeedCommand()
    {
        return speed;
    }

    uint8_t DelWinch::getDirection()
    {
        return direction_read;
    }

    uint8_t DelWinch::getCurrent()
    {
        return current;
    }

    uint8_t DelWinch::getError()
    {
        return error;
    }