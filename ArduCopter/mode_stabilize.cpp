#include "Copter.h"

#define MAX_INPUT 100.0f
#define MID_INPUT 50.0f

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // // apply simple mode transform to pilot inputs
    // update_simple_mode();

    // // convert pilot input to lean angles
    // float target_roll, target_pitch;
    // get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // // get pilot's desired yaw rate
    // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    float roll_input, pitch_input, yaw_input, thrust_input;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw are between -1 and 1
    // Roll = 1 -> move to the right
    // Pitch = 1 -> pitch backward
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1
    roll_input = (float(channel_roll->percent_input()) - MID_INPUT) / MID_INPUT;
    pitch_input = (float(channel_pitch->percent_input()) - MID_INPUT) / MID_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_INPUT) / MID_INPUT;
    thrust_input = float(channel_throttle->percent_input()) / MAX_INPUT;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    //  else if (copter.ap.throttle_zero) {
    //     // Attempting to Land
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    // } 
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // attitude_control->set_yaw_target_to_current_heading();
        // attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // attitude_control->set_yaw_target_to_current_heading();
        // attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        // if (!motors->limit.throttle_lower) {
        //     set_land_complete(false);
        // }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    attitude_control->set_commands(roll_input, pitch_input, yaw_input, thrust_input);

    // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw);

    // output pilot's throttle
    // attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                    true,
    //                                    g.throttle_filt);
}
