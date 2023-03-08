
// del_smapler.h

#ifndef DEL_SAMPLER_H
#define DEL_SAMPLER_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define UART_DELEAVES       4

#define COM_SEQUENCE        0
#define COM_STEALTH         1 
#define COM_CALIB           2
#define COM_LANDMODE        3
#define COM_DYNA1           4
#define COM_DYNA2           5

#define STATUS_CUTTING      0
#define STATUS_WRIST1       1
#define STATUS_WRIST2       2
#define STATUS_SAW          3
#define STATUS_GRASP        4
#define STATUS_ARM          5
#define STATUS_BATT_VOLT    6
#define STATUS_BATT_SOC     7

#define STATUS_MSG_SIZE     9
#define COM_MSG_SIZE        7
#define COM_HEADER          255

#define RC_MID_VALUE        1500
#define CALIB_TIME          3000

#define MAMBA_DYNA_OFFSET   50


/***************************************************************************
	Enumerations :
***************************************************************************/

enum MessageID
{
    StatusMessage = 252,
    TextMessage
};

enum TextMessageID
{
    SamplingCompleted,
    CalibrationStarted,
    LowBattery,
    SawNotConnected,
    SawJammed,
    SawHighCurrent,
    SamplingStucked,    
    NoCalibration,
    NoMessage = 195   
};

/***************************************************************************
	Class :
***************************************************************************/

class Sampler
{
public:
    Sampler();
    void init();
    uint8_t manageInput();
    void sendCommand();

    uint8_t* getStatus();

private:
    uint8_t comMsg[COM_MSG_SIZE];
    uint8_t statusMsg[STATUS_MSG_SIZE];
    uint8_t calib;
    uint8_t landMode;
    uint64_t calibTimer;
    uint8_t calibPrevious;

    AP_HAL::UARTDriver *sampler_port;
};

#endif