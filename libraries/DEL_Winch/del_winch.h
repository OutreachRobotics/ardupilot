
// del_winch.h

#ifndef DEL_WINCH_H
#define DEL_WINCH_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define WINCH_UART              2
#define WINCH_HEADER            0xFE
#define WINCH_FOOTER            0xDE

#define TX_BUFFER_LEN           5
#define RX_BUFFER_LEN           18

#define WINCH_UP_MASK           0x0800
#define WINCH_DOWN_MASK         0x1000
#define COMMAND_MASK            0x0200

#define WINCH_MAX_SPEED         50

#define COMMMAND_COUNT          5


/***************************************************************************
	Enumerations :
***************************************************************************/

enum Direction
{
    Neutral,
    Up,
    Down
};

union uint16_union
{
    int16_t value;
    uint8_t byte[2];
};

enum AddOnType
{
    None,
    WaterSampling,
    Grasping,
    PlantRubbing
};

/***************************************************************************
	Struct :
***************************************************************************/

struct DataQGC
{
    int16_t position;
    uint8_t addOn;
    uint8_t addOnState;
    uint16_t waterQty;
    uint16_t waterTime;
};

/***************************************************************************
	Class :
***************************************************************************/

class DelWinch
{
public:
    DelWinch();
    void init();
    void manage(uint16_t buttons);
    void receiveSerial();
    void setFailsafe(bool setter);
    int16_t getPosition();
    uint8_t getSpeed();
    uint8_t getSpeedCommand();
    uint8_t getDirection();
    uint8_t getError();
    uint8_t getCurrent();
    DataQGC getDataQGC();
    void printStatus();

private:
    AP_HAL::UARTDriver *_winch_port;
    uint8_t tx_buffer[TX_BUFFER_LEN];

    uint16_union position_read;
    uint8_t speed_read;
    uint8_t direction_read;
    uint8_t current;
    uint8_t error;
    uint8_t speed;
    Direction direction;
    uint8_t command;
    uint32_t command_ctr;
    uint16_t last_command;

    AddOnType addOnType;
    uint8_t graspingState;
    uint8_t rubbingState;
    uint8_t waterReady;
    uint8_t waterState;
    uint16_union waterQty; // dL
    uint8_t waterRate; // dL/m
    uint16_union waterTime; // s

    uint8_t gcs_failsafe;

    DataQGC dataQGC;

};

#endif