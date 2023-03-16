#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <DEL_EKF/del_ekf.h>
#include <DEL_Helper/del_helper.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    void deleaves_controller_acro(float lateral, float forward, float yaw, float throttle);
    void deleaves_controller_stabilize(float lateral, float forward, float yaw, float throttle, bool armed);
    void deleaves_controller_latHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed);
    void deleaves_controller_forHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed);
    void deleaves_controller_approachHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed);
    void deleaves_controller_angVelHold_PD(float lateral, float forward, float yaw, float throttle, bool armed);
    void deleaves_controller_angVelHold_LQR(float lateral, float forward, float yaw, float throttle, bool armed);
    void deleaves_controller_step_LQR(float lateral, float forward, float yaw, float throttle, bool armed);
    void deleaves_controller_taxi(float yaw, bool armed);
    void deleaves_controller_taxi_LQR(float yaw, bool armed);
    void constrainCommand();

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;
    void downSamplingDataFilter();
    void lowPassDataFilter();
    void lowPassSetPointFilter();

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    void updateDelEKF(Vector3f F_in, Vector3f measure, uint8_t rope_length);
    Vector3f getDelEKFOrientation();
    Vector2f getMaxReach();
    Mat getDelEKFStates();

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);
  

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll;
    AC_PID                _pid_rate_pitch;
    AC_PID                _pid_rate_yaw;

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    DelEKF delEKF;

    Vector3f ahrs_ang, last_ahrs_ang, ds_filtered_ang, last_ds_filtered_ang, ctrl_ang;
    Vector3f ang_vel, last_ang_vel, ds_filtered_ang_vel, last_ds_filtered_ang_vel, ctrl_ang_vel;

    float target_yaw, yaw_angle_error, yaw_angle_error_last, yaw_angle_error_dt, yaw_input;
    float target_forward, forward_error, forward_error_last, forward_error_dt, forward_command;
    float target_lateral, lateral_error, lateral_error_last, lateral_error_dt, lateral_command;
    float last_target_lateral, last_target_forward;
    float filtered_target_lateral, filtered_target_forward;
    float roll_kp, roll_kd, pitch_kp, pitch_kd, yaw_kp, yaw_kd;
    float pitch_sensitivity, roll_sensitivity;
};
