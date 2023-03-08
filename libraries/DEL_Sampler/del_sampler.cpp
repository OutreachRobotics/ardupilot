// del_sampler.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_sampler.h"
#include <AP_Logger/AP_Logger.h>

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

Sampler::Sampler()
{
	sampler_port = hal.serial(UART_DELEAVES);
}

void Sampler::init()
{
	memset(comMsg,0,sizeof(comMsg));
	memset(statusMsg,0,sizeof(statusMsg));
	sampler_port->begin(57600,10,10);
    calib = 0;
    landMode = 0;
    calibPrevious = 0;
    calibTimer = AP_HAL::millis();
}

uint8_t Sampler::manageInput()
{
    while(sampler_port->available()>1)
    {
        uint8_t temp = sampler_port->read();
        if(temp == StatusMessage && sampler_port->available()>=STATUS_MSG_SIZE-1)
        {
            for(uint8_t i=0;i<STATUS_MSG_SIZE-1;i++)
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

void Sampler::sendCommand()
{
    uint8_t dyna1Speed = 0;
    uint8_t dyna2Speed = 0;

    if(hal.rcin->read(CH_14)>RC_MID_VALUE && abs(hal.rcin->read(CH_3)-RC_MID_VALUE)>50)
    {
        dyna1Speed = ((hal.rcin->read(CH_3)-RC_MID_VALUE) / 17) + MAMBA_DYNA_OFFSET;
    }
    else 
    {
        dyna1Speed = 0 + MAMBA_DYNA_OFFSET;
    }
    
    if(hal.rcin->read(CH_14)>RC_MID_VALUE && abs(hal.rcin->read(CH_4)-RC_MID_VALUE)>50)
    {
        dyna2Speed = ((hal.rcin->read(CH_4)-RC_MID_VALUE) / 17) + MAMBA_DYNA_OFFSET;
    }
    else 
    {
        dyna2Speed = 0 + MAMBA_DYNA_OFFSET;
    }
    
    if(hal.rcin->read(CH_15)>RC_MID_VALUE && !calibPrevious)
    {
        calibTimer = AP_HAL::millis();
    }
    else if(hal.rcin->read(CH_15)<RC_MID_VALUE && calibPrevious)
    {
        if(AP_HAL::millis() - calibTimer > CALIB_TIME)
        {
            calib = calib ? 0 : 1;
        }
        else
        {
            landMode = landMode ? 0 : 1;
        }
    }
    calibPrevious = hal.rcin->read(CH_15)>RC_MID_VALUE;

    comMsg[0] = COM_HEADER;
    comMsg[1] = hal.rcin->read(CH_12)>RC_MID_VALUE;
    comMsg[2] = hal.rcin->read(CH_11)>RC_MID_VALUE;
    comMsg[3] = calib;
    comMsg[4] = landMode;
    comMsg[5] = dyna1Speed;
    comMsg[6] = dyna2Speed;
    sampler_port->write(comMsg,COM_MSG_SIZE);
}


uint8_t* Sampler::getStatus()
{
    return statusMsg;
}