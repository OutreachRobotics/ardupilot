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

    SRV_Channels::set_output_pwm(GRASPING_CH, GRASPING_CLOSE);
    graspingCnt = 0;
    graspingStatus = true;

}

uint8_t DEL_Comm::manageSamplerInput()
{
    graspingCnt = hal.rcin->read(GRASPING_RC) > MID_VALUE ? graspingCnt+1 : 0;
    graspingStatus = graspingCnt==GRASPING_DEBOUNCE ? !graspingStatus : graspingStatus;
    SRV_Channels::set_output_pwm(GRASPING_CH, graspingStatus ? GRASPING_CLOSE : GRASPING_OPEN);
    hal.console->printf("Count: %d\r\n", graspingCnt);
    hal.console->printf("Status: %d\r\n", graspingStatus);
    hal.console->printf("RC: %d\r\n", hal.rcin->read(GRASPING_RC));
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

    com2Msg[0] = COM_HEADER;
    com2Msg[1] = camera;
    fcu_port->write(com2Msg,COM2_MSG_SIZE);
}


uint8_t* DEL_Comm::getStatus()
{
    return statusMsg;
}