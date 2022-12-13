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
    timeStarted = AP_HAL::millis();
    sequenceStarted = false;
    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    float pitch_input, yaw_input;
    float lateral, forward;

    // We use a NED frame as per the UAV standard
    // Roll, pitch, yaw channel are between -1 and 1
    // lateral = 1 -> joystick right
    // Pitch = 1 -> joystick down
    // Yaw = 1 -> turn clockwise
    // Thrust is between 0 and 1

    pitch_input = -(float(channel_pitch->percent_input()) - MID_INPUT) / MID_INPUT;
    yaw_input = (float(channel_yaw->percent_input()) - MID_INPUT) / MID_INPUT;

    float yaw_moment = yaw_input;

    if(!sequenceStarted && pitch_input>0.5)
    {
        sequenceStarted = true;
        timeStarted = AP_HAL::millis();
        lateral = 0.0f;
        forward = 0.0f;
    }
    else if(sequenceStarted && pitch_input<-0.5)
    {
        sequenceStarted = false;
        lateral = 0.0f;
        forward = 0.0f;
    }
    else if(sequenceStarted)
    {
        uint32_t now = AP_HAL::millis();
        if(now-timeStarted<5000)
        {
            lateral = 0.0f;
            forward = 0.0f;
        }
        else if(now-timeStarted<15000)
        {
            lateral = 0.0f;
            forward = 0.10f;
        } 
        else if(now-timeStarted<25000)
        {
            lateral = 0.0f;
            forward = 0.0f;
        } 
        else if(now-timeStarted<35000)
        {
            lateral = 0.1f;
            forward = 0.1f;
        } 
        else
        {
            sequenceStarted = false;
            lateral = 0.0f;
            forward = 0.0f;
        }
    }
    else
    {
        lateral = 0.0f;
        forward = 0.0f;
    }

    // Only call controller each 8 timestep to have 50Hzs
    if (counter>7){
        attitude_control->deleaves_controller_forHold_LQR(lateral,forward,yaw_moment,0.0f,motors->armed());
        counter=0;
    }
    counter++;
}