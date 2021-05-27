#include "Copter.h"

#define MAX_INPUT 100.0f
#define MID_INPUT 50.0f
/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool ModeBrake::init(bool ignore_checks)
{
    counter = 0;
    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    float yaw_input;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    yaw_input = (float(channel_yaw->percent_input()) - MID_INPUT) / MID_INPUT;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Only call controller each 8 timestep to have 50Hzs
    if (counter>7){
        attitude_control->deleaves_controller_taxi(yaw_input, motors->armed());
        counter=0;
    }
    counter++;
}