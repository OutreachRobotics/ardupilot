
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

#define STATUS_MSG_SIZE     8
#define COM_MSG_SIZE        7
#define COM_HEADER          255

/***************************************************************************
	Enumerations :
***************************************************************************/

enum MessageID
{
    StatusMessage = 252,
    TextMessage
};

// enum TextMessageID
// {
//     SamplingCompleted,
//     CalibrationStarted,
//     LowBattery,
//     SawNotConnected,
//     SawJammed,
//     SawHighCurrent,
//     SamplingStucked,    
//     NoCalibration    
// };

/***************************************************************************
	Class :
***************************************************************************/

class Sampler
{
public:
    Sampler();
    void init();
    void manageInput();
    void receiveMsg();
    void sendMsg();

private:
    uint8_t cuttingPercentage = 0;
    uint8_t armStatus  = 0;
    uint8_t batteryVoltage = 0;
    uint8_t batterySOC = 0;
    uint8_t wrist1 = 0;
    uint8_t wrist2 = 0;
    uint8_t grasp = 0;
    uint8_t saw = 0;

    int8_t comMsg[COM_MSG_SIZE];
    uint8_t buf[STATUS_MSG_SIZE];
};

#endif