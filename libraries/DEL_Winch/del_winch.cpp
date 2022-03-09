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
}

void DelWinch::init()
{
    _winch_port->begin(57600,50,50);
}

void DelWinch::manage()
{
    // Sending commands to the winch
    
    // Receiving status from the winch
}