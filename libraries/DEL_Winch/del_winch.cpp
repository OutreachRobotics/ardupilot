// del_winch.cpp

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

DEL_Winch::DEL_Winch()
{

}

void DEL_Winch::init()
{
    SRV_Channels::set_output_pwm(WINCH_CH, WINCH_MID_CHANNEL);
}

void DEL_Winch::manage()
{
    uint16_t winch_rc_in = hal.rcin->read(CH_3)<1000 ? WINCH_MID_CHANNEL : hal.rcin->read(CH_3);
    winch_input = (winch_rc_in-WINCH_MID_CHANNEL)/WINCH_RANGE;


    if(abs(winch_input)>WINCH_DEADBAND)
    {
        SRV_Channels::set_output_pwm(WINCH_CH, winch_rc_in);
    }
    else
    {
        SRV_Channels::set_output_pwm(WINCH_CH, WINCH_MID_CHANNEL);
    }    
}
