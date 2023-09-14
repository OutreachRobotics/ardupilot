#include "Copter.h"
#include <DEL_Helper/del_helper.h>


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{
    counter = 0;
    probingState = Standby;
    motors->set_coax_enable(false);
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    float lateral_input, pitch_input, yaw_input, thrust_input;
    bool alt_hold_mode, sequence_on;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    alt_hold_mode = !(hal.rcin->read(TAXI_CHANNEL) > MID_PPM_VALUE);
    sequence_on = hal.rcin->read(WRIST_CHANNEL) > MID_PPM_VALUE;

    lateral_input = -(float(channel_roll->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT; // Exemple: channel=0.3 range -1 to 1 so 1.3/2=65% 65-50/50=0.3
    pitch_input = -(float(channel_pitch->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_RC_INPUT) / MID_RC_INPUT;
    thrust_input = float(channel_throttle->percent_input()) / MAX_RC_INPUT;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Only call controller each 8 timestep to have 50Hz
    if (counter>7)
    {
        ProbingState nextProbingState = Standby;
        switch (probingState)
        {
        case Standby:
            {
                if(alt_hold_mode)
                {
                    motors->set_coax_enable(true);
                    attitude_control->deleaves_controller_angVelHold_LQR(lateral_input, pitch_input, yaw_input, thrust_input, motors->armed());
                }
                else
                {
                    motors->set_coax_enable(false);
                    attitude_control->deleaves_controller_stabilize(lateral_input, pitch_input, yaw_input, thrust_input,motors->armed());
                }
                nextProbingState = sequence_on ? MoveForward : Standby;
            }
            break;

        case MoveForward:
            {
                float des_pitch = attitude_control->get_att_target_euler_rad().y;
                motors->set_coax_enable(true);
                attitude_control->deleaves_controller_angVelHold_LQR(lateral_input, SEQ_MOVING_PITCH_COMMAND, yaw_input, thrust_input, motors->armed());
                attachCommand = (motors->get_forward()/PMAX_ACTUATOR_THRUST) + 0.1f;
                nextProbingState = sequence_on ? des_pitch>=SEQ_MAX_ANGLE ? Attach : MoveForward : MoveBackward;
            }
            break;

        case Attach:
            {
                motors->set_coax_enable(true);
                attachCommand += SEQ_DETACH_INCREMENT;
                attitude_control->deleaves_controller_acro(0.0f, attachCommand, yaw_input, thrust_input);
                nextProbingState = sequence_on ? attachCommand>=SEQ_PROBING_PITCH_COMMAND ? Probing : Attach : Detach;
            }   
            break;

        case Probing:
            {
                motors->set_coax_enable(true);
                attitude_control->deleaves_controller_acro(0.0f, SEQ_PROBING_PITCH_COMMAND, yaw_input, thrust_input);
                attitude_control->setForwardTarget(constrain_float(attitude_control->getDelEKFOrientation().y-SEQ_PITCH_OFFSET,0.0f,MAX_PITCH));
                detachCommand = SEQ_PROBING_PITCH_COMMAND;
                nextProbingState = sequence_on ? Probing : Detach;
            }
            break;

        case Detach:
            {
                motors->set_coax_enable(true);
                detachCommand -= SEQ_DETACH_INCREMENT;
                attitude_control->deleaves_controller_acro(0.0f, detachCommand, yaw_input, thrust_input);
                nextProbingState = detachCommand <= SEQ_DETACH_LIMIT ? MoveBackward : Detach;
            }
            break;

        case MoveBackward:
            {
                float des_pitch = attitude_control->get_att_target_euler_rad().y;
                motors->set_coax_enable(true);
                attitude_control->deleaves_controller_angVelHold_LQR(lateral_input, -SEQ_MOVING_PITCH_COMMAND, yaw_input, thrust_input, motors->armed());
                nextProbingState = des_pitch<=SEQ_MIN_ANGLE ? Standby : MoveBackward;
            }
            break;
        
        default:
            nextProbingState = Standby;
            break;
        }
        probingState = nextProbingState;
        counter=0;
    }
    counter++;
}
