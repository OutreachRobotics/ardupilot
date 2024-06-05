// del_gimbal.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_gimbal.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

DelGimbal::DelGimbal()
{

}

void DelGimbal::init()
{
    pitch_command = PITCH_INITIAL_VALUE;
    zoom_command = ZOOM_INITIAL_VALUE;
    center_command = CENTER_INITIAL_VALUE;
    focus_command = FOCUS_INITIAL_VALUE;
    record_command = RECORD_INITIAL_VALUE;
    yaw_command = YAW_INITIAL_VALUE;

    SRV_Channels::set_output_pwm(PITCH_CH, pitch_command);
    SRV_Channels::set_output_pwm(ZOOM_CH, zoom_command);
    SRV_Channels::set_output_pwm(CENTER_CH, center_command);
    SRV_Channels::set_output_pwm(FOCUS_CH, focus_command);
    SRV_Channels::set_output_pwm(RECORD_CH, record_command);
    SRV_Channels::set_output_pwm(YAW_CH, yaw_command);
    
}

void DelGimbal::manage(uint16_t buttons, bool armStatus)
{
    if(!gcs_failsafe)
    {
        if(buttons & PITCH_UP_MASK)
        {
            pitch_command = PITCH_UP;
        }
        else if(buttons & PITCH_DOWN_MASK)
        {
            pitch_command = PITCH_DOWN;
        }
        else
        {
            pitch_command = MID_PPM;
        }

        // if(buttons & YAW_LEFT_MASK)
        // {
        //     yaw_command = YAW_LEFT;
        // }
        // else if(buttons & YAW_RIGHT_MASK)
        // {
        //     yaw_command = YAW_RIGHT;
        // }
        // else
        // {
        //     yaw_command = MID_PPM;
        // }

        if(buttons & ZOOM_IN_MASK)
        {
            zoom_command = MAX_PPM;
        }
        else if(buttons & ZOOM_OUT_MASK)
        {
            zoom_command = MIN_PPM;
        }
        else
        {
            zoom_command = MID_PPM;
        }

        if(buttons & CENTER_MASK)
        {
            center_command = MAX_PPM;
        }
        else
        {
            center_command = MIN_PPM;
        }

        if(buttons & FOCUS_MASK)
        {
            focus_command = MAX_PPM;
        }
        else
        {
            focus_command = MIN_PPM;
        }

        if(armStatus && !lastArmStatus)
        {
            record_command = MAX_PPM;
        }
        else if(!armStatus && lastArmStatus)
        {
            record_command = MIN_PPM;
        }
        else
        {
            record_command = MID_PPM;
        }
        lastArmStatus = armStatus;    
    }
    else
    {
        pitch_command = MID_PPM;
        yaw_command = MID_PPM;
        zoom_command = MID_PPM;
        center_command = MAX_PPM;
        focus_command = MID_PPM;
        record_command = MID_PPM;
    }

    // SRV_Channels::set_output_pwm(PITCH_CH, pitch_command);
    // SRV_Channels::set_output_pwm(YAW_CH, yaw_command);
    // SRV_Channels::set_output_pwm(ZOOM_CH, zoom_command);
    // SRV_Channels::set_output_pwm(CENTER_CH, center_command);
    // SRV_Channels::set_output_pwm(FOCUS_CH, focus_command);
    // SRV_Channels::set_output_pwm(RECORD_CH, record_command);  

    RC_Channels::set_override(5, pitch_command, AP_HAL::millis()); 
    RC_Channels::set_override(6, zoom_command, AP_HAL::millis()); 
    RC_Channels::set_override(8, center_command, AP_HAL::millis()); 
    RC_Channels::set_override(9, focus_command, AP_HAL::millis()); 
}

void DelGimbal::setFailsafe(uint8_t setter)
{
    gcs_failsafe = setter;
}
