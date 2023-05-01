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
            hal.console->printf("\r\nSampler Msg\r\n");
            for(uint8_t i=0;i<SAMPLER_MSG_SIZE-1;i++)
			{
				statusMsg[i] = sampler_port->read();
                hal.console->printf("%d: %d\r\n",i,statusMsg[i]);
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
            // hal.console->printf("\r\nFCU Msg\r\n");
            for(uint8_t i=0;i<FCU_MSG_SIZE-1;i++)
			{
				statusMsg[i+FCU_MSG_OFFSET] = fcu_port->read();
                // hal.console->printf("%d: %d\r\n",i,statusMsg[i+FCU_MSG_OFFSET]);
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

void DEL_Comm::sendCommand(uint8_t camera)
{
    uint8_t dyna1Speed = 0;
    uint8_t dyna2Speed = 0;

    if(hal.rcin->read(WRIST_CHANNEL)>MID_PPM_VALUE && abs(hal.rcin->read(CH_3)-MID_PPM_VALUE)>WRIST_DEADBAND)
    {
        dyna1Speed = ((hal.rcin->read(CH_3)-MID_PPM_VALUE) / WRIST_SPEED_FACTOR) + MAMBA_DYNA_OFFSET;
    }
    else 
    {
        dyna1Speed = 0 + MAMBA_DYNA_OFFSET;
    }
    
    if(hal.rcin->read(WRIST_CHANNEL)>MID_PPM_VALUE && abs(hal.rcin->read(CH_4)-MID_PPM_VALUE)>WRIST_DEADBAND)
    {
        dyna2Speed = ((hal.rcin->read(CH_4)-MID_PPM_VALUE) / WRIST_SPEED_FACTOR) + MAMBA_DYNA_OFFSET;
    }
    else 
    {
        dyna2Speed = 0 + MAMBA_DYNA_OFFSET;
    }
    
    if(hal.rcin->read(CALIB_CHANNEL)>MID_PPM_VALUE && !calibPrevious)
    {
        calibTimer = AP_HAL::millis();
    }
    else if(hal.rcin->read(CALIB_CHANNEL)<MID_PPM_VALUE && calibPrevious)
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
    calibPrevious = hal.rcin->read(CALIB_CHANNEL)>MID_PPM_VALUE;

    comMsg[0] = COM_SAMPLER_HEADER;
    comMsg[1] = hal.rcin->read(SEQUENCE_CHANNEL)>MID_PPM_VALUE;
    comMsg[2] = hal.rcin->read(STEALTH_CHANNEL)>MID_PPM_VALUE;
    comMsg[3] = calib;
    comMsg[4] = landMode;
    comMsg[5] = dyna1Speed;
    comMsg[6] = dyna2Speed;
    sampler_port->write(comMsg,COM_MSG_SIZE);
    hal.console->printf("\r\nCommand: %d %d %d %d %d %d\r\n", comMsg[1],comMsg[2],comMsg[3],comMsg[4],comMsg[5],comMsg[6]);

    com2Msg[0] = COM_HEADER;
    com2Msg[1] = camera;
    fcu_port->write(com2Msg,COM2_MSG_SIZE);
}


uint8_t* DEL_Comm::getStatus()
{
    return statusMsg;
}