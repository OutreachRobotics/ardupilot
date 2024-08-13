
// del_smapler.h

#ifndef DEL_COMM_H
#define DEL_COMM_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <DEL_Helper/del_helper.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define UART_SAMPLER        4
#define UART_FCU            2

#define COM_MSG_SIZE        7
#define COM_HEADER          255
#define COM_SAMPLER_HEADER  251
#define COM_SEQUENCE        0
#define COM_STEALTH         1 
#define COM_CALIB           2
#define COM_LANDMODE        3
#define COM_DYNA1           4
#define COM_DYNA2           5
#define COM2_MSG_SIZE       2
#define COM_HDMI            1

#define STATUS_MSG_SIZE     9
#define SAMPLER_MSG_SIZE    7
#define FCU_MSG_SIZE        4
#define FCU_MSG_OFFSET      6
#define STATUS_CUTTING      0
#define STATUS_WRIST1       1
#define STATUS_WRIST2       2
#define STATUS_SAW          3
#define STATUS_GRASP        4
#define STATUS_ARM          5
#define STATUS_BATT_LOW     6
#define STATUS_BATT_HIGH    7
#define STATUS_BATT_SOC     8

#define CALIB_TIME          3000
// Need to offset the received dyna speed to fit in uint8_t
#define MAMBA_DYNA_OFFSET   50
#define WRIST_DEADBAND      50
#define WRIST_SPEED_FACTOR  17

#define GRASPING_CH         SRV_Channel::k_scripting1
#define GRASPING_CLOSE      1200
#define GRASPING_OPEN       1900
#define GRASPING_RC         9
#define MID_VALUE           1500
#define GRASPING_DEBOUNCE   5


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

class DEL_Comm
{
public:
    DEL_Comm();
    void init();
    uint8_t manageSamplerInput();
    uint8_t manageFCUInput();
    void sendCommand(uint8_t camera);

    uint8_t* getStatus();

private:
    uint8_t comMsg[COM_MSG_SIZE];
    uint8_t com2Msg[COM2_MSG_SIZE];
    uint8_t statusMsg[STATUS_MSG_SIZE];
    uint8_t calib;
    uint8_t landMode;
    uint64_t calibTimer;
    uint8_t calibPrevious;
    uint16_t graspingPWM;
    uint16_t graspingCnt;
    uint8_t graspingStatus;

    AP_HAL::UARTDriver *sampler_port;
    AP_HAL::UARTDriver *fcu_port;
};

#endif