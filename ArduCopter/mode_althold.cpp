#include "Copter.h"

#define MAX_INPUT 100.0f
#define MID_INPUT 50.0f
#define RESET_REF_VALUE 1500

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{
    counter = 0;
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    float lateral_input, pitch_input, yaw_input, thrust_input;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    lateral_input = -(float(channel_roll->percent_input()) - MID_INPUT) / MID_INPUT; // range -1 to 1
    pitch_input = -(float(channel_pitch->percent_input()) - MID_INPUT) / MID_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_INPUT) / MID_INPUT;
    thrust_input = float(channel_throttle->percent_input()) / MAX_INPUT;

    float forward = (pitch_input * sqrtf(2)/2 - lateral_input * sqrtf(2)/2) / (sqrtf(2)); //range -1 to 1
    float lateral = (pitch_input * sqrtf(2)/2 + lateral_input * sqrtf(2)/2) / (sqrtf(2));
    float yaw_moment = yaw_input;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Only call controller each 8 timestep to have 50Hzs
    if (counter>7){
        attitude_control->deleaves_controller_angVelHold_LQR(lateral, forward, yaw_moment, thrust_input, motors->armed());
        counter=0;
    }
    counter++;
}
