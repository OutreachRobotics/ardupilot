#include "AP_Sony.h"

extern const AP_HAL::HAL& hal;

AP_Sony::AP_Sony()
{
    is_started = false;
    is_on = false;

    here_zoom_in = false;
    here_zoom_out = false;
    here_photo_btn = false;
    here_movie_btn = false;
    here_zoom_btn = false;
    here_focus_btn = false;
    servo_up = false;
    servo_down = false;

    rec_ch1_pwm = SONY_MID_VALUE;
    rec_ch2_pwm = SONY_MID_VALUE;
    rec_ch3_pwm = SONY_MAN_MID_VALUE;
    rec_ch4_pwm = SONY_MID_VALUE;
    servo_pwm = SONY_MID_VALUE;

    power_time = AP_HAL::millis();
    photo_time = AP_HAL::millis();
    movie_time = AP_HAL::millis();

    SRV_Channels::set_output_pwm(SONY_REC_CH, rec_ch1_pwm);
    SRV_Channels::set_output_pwm(SONY_ZOOM_CH, rec_ch2_pwm);
    SRV_Channels::set_output_pwm(SONY_MAN_CH, rec_ch3_pwm);
    SRV_Channels::set_output_pwm(SONY_PWR_CH, rec_ch4_pwm);
    SRV_Channels::set_output_pwm(SONY_SERVO_CH, servo_pwm);
}
void AP_Sony::manage()
{
    handle_rc_input();
    set_pwm_action();
    handle_pwm_output();
}

void AP_Sony::handle_rc_input()
{
    here_zoom_in = hal.rcin->read(HERE_ZOOM_CH) > HERE_MID_VALUE && hal.rcin->read(HERE_ZOOM_SERVO_CH) > HERE_ZOOM_SERVO_UP;
    here_zoom_out = hal.rcin->read(HERE_ZOOM_CH) > HERE_MID_VALUE && hal.rcin->read(HERE_ZOOM_SERVO_CH) < HERE_ZOOM_SERVO_DOWN;

    servo_up = !(hal.rcin->read(HERE_ZOOM_CH) > HERE_MID_VALUE) && hal.rcin->read(HERE_ZOOM_SERVO_CH) > HERE_ZOOM_SERVO_UP;
    servo_down = !(hal.rcin->read(HERE_ZOOM_CH) > HERE_MID_VALUE) && hal.rcin->read(HERE_ZOOM_SERVO_CH) < HERE_ZOOM_SERVO_DOWN; 
    
    if(hal.rcin->read(HERE_PHOTO_CH) > HERE_MID_VALUE && !here_focus_btn && !here_photo_btn)
    {
        here_focus_btn = true;
    }
    else if(hal.rcin->read(HERE_PHOTO_CH) > HERE_MID_VALUE)
    {   
        // Wait for the focus
    }
    else if(here_focus_btn)
    {
        here_photo_btn = true;
        here_focus_btn = false;
        photo_time = AP_HAL::millis();
    }
    else if(AP_HAL::millis() - photo_time > 500)
    {
        here_photo_btn = false;
    }

    if(hal.rcin->read(HERE_MOVIE_CH) > HERE_MID_VALUE && !here_movie_btn)
    {
        here_movie_btn = true;
        movie_time = AP_HAL::millis();
    }
    else if(AP_HAL::millis() - movie_time > 1500)
    {   
        here_movie_btn = false;
    }
}

void AP_Sony::set_pwm_action()
{
    // Handling the camera power at startup (CH4)
    if(!is_started)
    {
        power_time = AP_HAL::millis();
        is_started = true;
        rec_ch4_pwm = SONY_ON_OFF_COM;     
    }
    else if(!is_on)
    {
        if(AP_HAL::millis() - power_time < 1500) 
        {
            rec_ch4_pwm = SONY_ON_OFF_COM; 
        }    
        else
        {
            is_on = true;
        } 
    }
    else
    {
        rec_ch4_pwm = SONY_MID_VALUE; 
    }

    // Handling the camera zoom with the RC (wheel for zoom, A button for speed) (CH2)
    if(here_zoom_in)
    {
        rec_ch2_pwm = SONY_LOW_ZOOM_IN;
    }
    else if(here_zoom_out)
    {
        rec_ch2_pwm = SONY_LOW_ZOOM_OUT;
    }  
    else
    {
        rec_ch2_pwm = SONY_MID_VALUE;
    } 

    // Handling the MANUAL photo channel (CH3)
    if(here_focus_btn && !here_photo_btn)
    {
        rec_ch3_pwm = SONY_FOCUS;
    }
    else
    {
        rec_ch3_pwm = SONY_MAN_MID_VALUE;
;
    }

    // Handling the recording channel (CH1)
    if(here_photo_btn)
    {
        rec_ch1_pwm = SONY_PHOTO;
    }
    else if(here_movie_btn)
    {
        rec_ch1_pwm = SONY_MOVIE;
    }
    else
    {
        rec_ch1_pwm = SONY_MID_VALUE;
    }

    servo_pwm += servo_up?SERVO_INCREMENT:0 + servo_down?-SERVO_INCREMENT:0;
    servo_pwm = constrain_float(servo_pwm, MIN_SERVO_PWM, MAX_SERVO_PWM);
}

void AP_Sony::handle_pwm_output()
{
    SRV_Channels::set_output_pwm(SONY_REC_CH, rec_ch1_pwm);
    SRV_Channels::set_output_pwm(SONY_ZOOM_CH, rec_ch2_pwm);
    SRV_Channels::set_output_pwm(SONY_MAN_CH, rec_ch3_pwm);
    SRV_Channels::set_output_pwm(SONY_PWR_CH, rec_ch4_pwm);
    SRV_Channels::set_output_pwm(SONY_SERVO_CH, servo_pwm);
}

uint16_t AP_Sony::get_camera_angle()
{
    return ((servo_pwm-MIN_SERVO_PWM)*180)/(MAX_SERVO_PWM-MIN_SERVO_PWM);
}