#include "Copter.h"
#include <DEL_Helper/del_helper.h>


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{
    counter = 0;
    motors->set_coax_enable(false);
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    float lateral_input, pitch_input, yaw_input, thrust_input;
    bool taxi_mode;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    taxi_mode = hal.rcin->read(TAXI_CHANNEL) > MID_PPM_VALUE;

    lateral_input = -(float(channel_roll->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT; // Exemple: channel=0.3 range -1 to 1 so 1.3/2=65% 65-50/50=0.3
    pitch_input = -(float(channel_pitch->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    yaw_input = hal.rcin->read(WRIST_CHANNEL) < MID_PPM_VALUE ? (float(channel_yaw->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT : 0.0f;
    thrust_input = float(channel_throttle->percent_input()) / MAX_RC_INPUT;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }



    // Only call controller each 8 timestep to have 50Hz
    if (counter>7){
        if(taxi_mode)
        {
            motors->set_coax_enable(false);
            attitude_control->deleaves_controller_taxi_LQR(yaw_input, motors->armed());
        }
        else
        {
            if(motors->get_coax_enable() && attitude_control->getPitchCommand()<0.1f)
            {
                motors->set_coax_enable(false);
            }
            else if(!motors->get_coax_enable() && attitude_control->getDelEKFOrientation().y>0.1f && attitude_control->getPitchCommand()>0.2f)
            {
                motors->set_coax_enable(true);
            }
            attitude_control->deleaves_controller_angVelHold_LQR(lateral_input, pitch_input, yaw_input, thrust_input, motors->armed());
        }
        counter=0;
    }
    counter++;
}
