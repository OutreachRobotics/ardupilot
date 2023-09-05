#include "Copter.h"
#include <DEL_Helper/del_helper.h>

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool ModeBrake::init(bool ignore_checks)
{
    counter = 0;
    motors->set_motors_tuning(true);
    return true;
}

void ModeBrake::exit()
{
    motors->set_motors_tuning(false);
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    // float lateral_input, pitch_input, yaw_input, thrust_input;
    // bool taxi_mode;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    // taxi_mode = hal.rcin->read(TAXI_CHANNEL) > MID_PPM_VALUE;

    // lateral_input = -(float(channel_roll->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT; // Exemple: channel=0.3 range -1 to 1 so 1.3/2=65% 65-50/50=0.3
    // pitch_input = -(float(channel_pitch->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    // yaw_input = (float(channel_yaw->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    // thrust_input = float(channel_throttle->percent_input()) / MAX_RC_INPUT;

    // Only call controller each 8 timestep to have 50Hz
    // if (counter>7){

    // }
    // counter++;
}
