#include "Copter.h"

#define MAX_INPUT 100.0f
#define MID_INPUT 50.0f
#define DEADBAND 0.01f //0 to 1

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{
    counter = 0;
    sequenceArmed = 0;
    sequenceStart = AP_HAL::micros();
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    float lateral_input, pitch_input, yaw_input, thrust_input;
    float forward_target = 0.0f;
    float lateral_target = 0.0f;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> move to the right
    // Pitch = 1 -> pitch backward
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    lateral_input = (float(channel_roll->percent_input()) - MID_INPUT) / MID_INPUT; // Exemple: channel=0.3 range -1 to 1 so 1.3/2=65% 65-50/50=0.3
    pitch_input = -(float(channel_pitch->percent_input()) - MID_INPUT) / MID_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_INPUT) / MID_INPUT;
    thrust_input = float(channel_throttle->percent_input()) / MAX_INPUT;

    //Add a deadband to inputs
    lateral_input = abs(lateral_input)<DEADBAND ? 0.0f : lateral_input;
    pitch_input = abs(pitch_input)<DEADBAND ? 0.0f : pitch_input;
    yaw_input = abs(yaw_input)<DEADBAND ? 0.0f : yaw_input;

    if(sequenceArmed && pitch_input<-0.8)
    {
        forward_target = 0.0f;
        lateral_target = 0.0f;
        sequenceArmed = false;
        sequenceStart = AP_HAL::millis();
    }
    else if(!sequenceArmed && pitch_input>0.8)
    {
        forward_target = 0.0f;
        lateral_target = 0.0f;
        sequenceArmed = true;
        sequenceStart = AP_HAL::millis();
    }
    else if(sequenceArmed)
    {
        uint32_t now = AP_HAL::millis();
        if(now-sequenceStart<10000)
        {
            forward_target = 0.0f;
            lateral_target = 0.0f;
        }
        else if(now-sequenceStart<20000)
        {
            forward_target = (L1+L2)*sinf(0.2f);
            lateral_target = 0.0f;
        }
        else if(now-sequenceStart<30000)
        {
            forward_target = (L1+L2)*sinf(0.25f);
            lateral_target = 0.0f;
        }
        else if(now-sequenceStart<40000)
        {
            forward_target = (L1+L2)*sinf(0.25f);
            lateral_target = -(L1+L2)*sinf(0.1f);
        }
        else if(now-sequenceStart<50000)
        {
            forward_target = (L1+L2)*sinf(0.25f);
            lateral_target = -(L1+L2)*sinf(-0.2f);
        }
        else
        {
            forward_target = (L1+L2)*sinf(0.25f);
            lateral_target = -(L1+L2)*sinf(-0.25f);
        }       
    }
    else
    {
        forward_target = 0.0f;
        lateral_target = 0.0f;
        sequenceArmed = false;
        sequenceStart = AP_HAL::millis();
    }

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    
    else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Only call controller each 8 timestep to have 50Hz
    if (counter>7){
        attitude_control->deleaves_controller_angHold_PD(lateral_target, forward_target, yaw_input, thrust_input, sequenceArmed, motors->armed());
        counter=0;
    }
    counter++;

}
