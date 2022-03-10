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
    _winch_port->begin(57600,50,50);

    position_read = 0;
    speed_read = 0;
    direction_read = Neutral;
    speed = 0;
    direction = Neutral;
    error = 0;
    winch_input = 0;

    tx_buffer[0] = WINCH_HEADER;
    tx_buffer[1] = direction;
    tx_buffer[2] = speed;
    tx_buffer[3] = WINCH_FOOTER;
}

void DelWinch::manage()
{
    // Receiving status from the winch
    while(_winch_port->available() > RX_BUFFER_LEN)
    {
        if(_winch_port->read() == WINCH_HEADER)
        {
            uint16_t temp;
            temp = _winch_port->read();
            position_read = _winch_port->read();
            position_read = (temp<<8) | position_read;
            speed_read = _winch_port->read();
            direction_read = _winch_port->read();
            error = _winch_port->read();
            error |= _winch_port->read() == WINCH_FOOTER;
            _winch_port->flush();
        }
    }

    winch_input = (hal.rcin->read(CH_3)-WINCH_MID_CHANNEL)/WINCH_RANGE;

    if(abs(winch_input)>WINCH_DEADBAND)
    {
        direction = winch_input>0.0f ? Up : Down;
        direction = direction==Up && position_read<10 ? Neutral : direction;
        speed = uint8_t(abs(winch_input) * WINCH_MAX_SPEED);
    }
    else
    {
        direction = Neutral;
        speed = 0;
    }

    // Sending commands to the winch
    tx_buffer[1] = direction;
    tx_buffer[2] = speed;
    _winch_port->write(tx_buffer,TX_BUFFER_LEN);   

}