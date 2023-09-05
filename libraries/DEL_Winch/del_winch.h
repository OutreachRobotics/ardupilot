
// del_winch.h

#ifndef DEL_WINCH_H
#define DEL_WINCH_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <stdint.h>
#include <SRV_Channel/SRV_Channel.h>


/***************************************************************************
    Macro :
***************************************************************************/

#define WINCH_MID_CHANNEL       1500
#define WINCH_DEADBAND          0.05f
#define WINCH_RANGE             500.0f

#define WINCH_MAX_SPEED         50

#define WINCH_CH                SRV_Channel::k_winch

/***************************************************************************
	Enumerations :
***************************************************************************/


/***************************************************************************
	Class :
***************************************************************************/

class DEL_Winch
{
public:
    DEL_Winch();
    void init();
    void manage();

private:
    uint8_t speed;
    float winch_input;

};

#endif