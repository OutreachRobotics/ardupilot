// del_comm.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_comm.h"
#include <AP_Logger/AP_Logger.h>

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

DEL_Comm::DEL_Comm()
{
	sampler_port = hal.serial(UART_SAMPLER);
    fcu_port = hal.serial(UART_FCU);
}

void DEL_Comm::init()
{
	memset(comMsg,0,sizeof(comMsg));
	memset(com2Msg,0,sizeof(com2Msg));
	memset(statusMsg,0,sizeof(statusMsg));

	sampler_port->begin(57600,10,10);
	fcu_port->begin(57600,10,10);

    calib = 0;
    landMode = 0;
    calibPrevious = 0;
    calibTimer = AP_HAL::millis();
}

uint8_t DEL_Comm::manageSamplerInput()
{
    while(sampler_port->available()>1)
    {
        uint8_t temp = sampler_port->read();
        if(temp == StatusMessage && sampler_port->available()>=SAMPLER_MSG_SIZE-1)
        {
            for(uint8_t i=0;i<SAMPLER_MSG_SIZE-1;i++)
			{
				statusMsg[i] = sampler_port->read();
			}    
        }
        else if(temp == TextMessage && sampler_port->available())
        {
            uint8_t status_message = sampler_port->read();
            return status_message;           
        }
    }
    return NoMessage;
}

uint8_t DEL_Comm::manageFCUInput()
{
    while(fcu_port->available()>1)
    {
        uint8_t temp = fcu_port->read();
        if(temp == StatusMessage && fcu_port->available()>=FCU_MSG_SIZE-1)
        {
            for(uint8_t i=0;i<FCU_MSG_SIZE-1;i++)
			{
				statusMsg[i+FCU_MSG_OFFSET] = fcu_port->read();
			}    
        }
        else if(temp == TextMessage && fcu_port->available())
        {
            uint8_t status_message = fcu_port->read();
            return status_message;           
        }
    }
    return NoMessage;
}

void DEL_Comm::sendCommand()
{

    comMsg[0] = COM_SAMPLER_HEADER;
    comMsg[1] = hal.rcin->read(SEQUENCE_CHANNEL)>MID_PPM_VALUE;
    comMsg[2] = hal.rcin->read(STEALTH_CHANNEL)>MID_PPM_VALUE;
    comMsg[3] = hal.rcin->read(CALIB_CHANNEL)>MID_PPM_VALUE;
    comMsg[4] = hal.rcin->read(TAXI_CHANNEL)>MID_PPM_VALUE;
    sampler_port->write(comMsg,COM_MSG_SIZE);

    // com2Msg[0] = COM_HEADER;
    // com2Msg[1] = camera;
    // fcu_port->write(com2Msg,COM2_MSG_SIZE);
}

uint8_t* DEL_Comm::getStatus()
{
    return statusMsg;
}