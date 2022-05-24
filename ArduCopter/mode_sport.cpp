#include "Copter.h"

#if MODE_SPORT_ENABLED == ENABLED

#define MAX_INPUT 100.0f
#define MID_INPUT 50.0f

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    counter = 0;
    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{

    // Only call controller each 8 timestep to have 50Hz
    if (counter>7){
        attitude_control->tuneMotor(motors->armed());
        counter=0;
    }
    counter++;


}

#endif
