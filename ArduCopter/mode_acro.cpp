#include "Copter.h"
#include <DEL_Helper/del_helper.h>

/*
 * Init and run calls for stabilize flight mode
 */
bool ModeAcro::init(bool ignore_checks)
{
    motors->set_coax_enable(false);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeAcro::run()
{
    float lateral_input, pitch_input, yaw_input, thrust_input;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw are between -1 and 1
    // lateral = 1 -> move to the right
    // Pitch = 1 -> pitch backward
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1
    lateral_input = -(float(channel_roll->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    pitch_input = -(float(channel_pitch->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    thrust_input = float(channel_throttle->percent_input()) / MAX_RC_INPUT;

    //Add a deadband to inputs
    lateral_input = abs(lateral_input)<DEADBAND ? 0.0f : lateral_input;
    pitch_input = abs(pitch_input)<DEADBAND ? 0.0f : pitch_input;
    yaw_input = abs(yaw_input)<DEADBAND ? 0.0f : yaw_input;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    attitude_control->deleaves_controller_acro(lateral_input, pitch_input, yaw_input, thrust_input);
}

void ModeAcro::exit()
{
    motors->set_coax_enable(false);
}

void ModeAcro::air_mode_aux_changed()
{
    
}
