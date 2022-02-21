/// @file	AP_Sonyh
/// @brief	Photo or video camera manager for the Sony RX100
#pragma once

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Relay/AP_Relay.h>

#define SONY_REC_CH                     SRV_Channel::k_cam_trigger
#define SONY_ZOOM_CH                    SRV_Channel::k_cam_iso
#define SONY_MAN_CH                     SRV_Channel::k_cam_aperture
#define SONY_PWR_CH                     SRV_Channel::k_cam_focus
#define SONY_SERVO_CH                   SRV_Channel::k_mount2_pan
#define SONY_ON_OFF_COM                 1800
#define SONY_MAN_MID_VALUE              1200
#define SONY_MID_VALUE                  1500
#define SONY_LOW_ZOOM_IN                1692
#define SONY_LOW_ZOOM_OUT               1308
#define SONY_FOCUS                      1500
#define SONY_PHOTO                      1325
#define SONY_MOVIE                      1800


#define HERE_ZOOM_SERVO_CH              6
#define HERE_ZOOM_SERVO_UP              1600
#define HERE_ZOOM_SERVO_DOWN            1400
#define HERE_MID_VALUE                  1500
#define HERE_PHOTO_CH                   8
#define HERE_MOVIE_CH                   9
#define HERE_ZOOM_CH                    10

#define MIN_SERVO_PWM                   600
#define MAX_SERVO_PWM                   2380
#define SERVO_INCREMENT                 10

class AP_Sony
{
public:
    AP_Sony();
    void manage();
    void handle_rc_input();
    void set_pwm_action();
    void handle_pwm_output();
    uint16_t get_camera_angle();

private:
    bool is_started;
    bool is_on;

    bool here_zoom_in;
    bool here_zoom_out;
    bool here_photo_btn;
    bool here_movie_btn;
    bool here_zoom_btn;
    bool here_focus_btn;
    bool servo_up;
    bool servo_down;

    uint16_t rec_ch1_pwm;
    uint16_t rec_ch2_pwm;
    uint16_t rec_ch3_pwm;
    uint16_t rec_ch4_pwm;

    uint32_t power_time;
    uint32_t photo_time;
    uint32_t movie_time;

    uint16_t servo_pwm;
};