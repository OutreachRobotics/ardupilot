#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    target_yaw=0;
    yaw_angle_error=0;
    yaw_angle_error_last=0;
    yaw_angle_error_dt=0;
    yaw_input=0;
    target_forward = 0;
    forward_error = 0;
    forward_error_last = 0;
    forward_error_dt = 0;
    forward_command = 0;
    target_lateral = 0;
    lateral_error = 0;
    lateral_error_last = 0;
    lateral_error_dt = 0;
    lateral_command = 0;

    roll_kp = 0.0f;
    roll_kd = 0.0f;
    pitch_kp = 0.0f;
    pitch_kd = 0.0f;
    yaw_kp = 0.0f;
    yaw_kd = 0.0f;
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f * cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f / constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run()
{

}

void AC_AttitudeControl_Multi::downSamplingDataFilter()
{
    // Low pass filter on orientation
    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    attitude_vehicle_quat.to_euler(ahrs_ang.x, ahrs_ang.y, ahrs_ang.z);
    ds_filtered_ang.x = B1_DS*ahrs_ang.x + B0_DS*last_ahrs_ang.x - A0_DS*ds_filtered_ang.x;
    ds_filtered_ang.y = B1_DS*ahrs_ang.y + B0_DS*last_ahrs_ang.y - A0_DS*ds_filtered_ang.y;
    ds_filtered_ang.z = B1_DS*ahrs_ang.z + B0_DS*last_ahrs_ang.z - A0_DS*ds_filtered_ang.z;

    // Low pass filter on gyroscope data
    ang_vel = _ahrs.get_gyro_latest();
    ds_filtered_ang_vel.x = B1_DS*ang_vel.x + B0_DS*last_ang_vel.x - A0_DS*ds_filtered_ang_vel.x;
    ds_filtered_ang_vel.y = B1_DS*ang_vel.y + B0_DS*last_ang_vel.y - A0_DS*ds_filtered_ang_vel.y;
    ds_filtered_ang_vel.z = B1_DS*ang_vel.z + B0_DS*last_ang_vel.z - A0_DS*ds_filtered_ang_vel.z;

    last_ahrs_ang = ahrs_ang;
    last_ang_vel = ang_vel;

    yaw_kp = 20.0f;
    yaw_kd = 12.0f;

    if(get_mamba_length()<6.0f)
    {
        roll_kp = 28.0f;
        roll_kd = 46.0f;
        pitch_kp = 50.0;
        pitch_kd = 50.0f;
        roll_sensitivity = 0.007f;
        pitch_sensitivity = 0.007f;
    }
    else if(get_mamba_length()<12.0f)
    {
        roll_kp = 47.0f;
        roll_kd = 72.0f;
        pitch_kp = 89.0;
        pitch_kd = 89.0f;
        roll_sensitivity = 0.0038f;
        pitch_sensitivity = 0.0038f;
    }
    else
    {
        roll_kp = 80.0f;
        roll_kd = 122.0f;
        pitch_kp = 166.0;
        pitch_kd = 166.0f;
        roll_sensitivity = 0.00188f;
        pitch_sensitivity = 0.00188f;
    }
}

void AC_AttitudeControl_Multi::lowPassDataFilter()
{
    ctrl_ang.x = B1_LP*ds_filtered_ang.x + B0_LP*last_ds_filtered_ang.x - A0_LP*ctrl_ang.x;
    ctrl_ang.y = B1_LP*ds_filtered_ang.y + B0_LP*last_ds_filtered_ang.y - A0_LP*ctrl_ang.y;
    ctrl_ang.z = B1_LP*ds_filtered_ang.z + B0_LP*last_ds_filtered_ang.z - A0_LP*ctrl_ang.z;

    ctrl_ang_vel.x = B1_LP*ds_filtered_ang_vel.x + B0_LP*last_ds_filtered_ang_vel.x - A0_LP*ctrl_ang_vel.x;
    ctrl_ang_vel.y = B1_LP*ds_filtered_ang_vel.y + B0_LP*last_ds_filtered_ang_vel.y - A0_LP*ctrl_ang_vel.y;
    ctrl_ang_vel.z = B1_LP*ds_filtered_ang_vel.z + B0_LP*last_ds_filtered_ang_vel.z - A0_LP*ctrl_ang_vel.z;

    last_ds_filtered_ang = ds_filtered_ang;
    last_ds_filtered_ang_vel = ds_filtered_ang_vel;
}


// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > 4.0f) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(AC_ATTITUDE_CONTROL_MAN_DEFAULT);
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > 0.25f) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

void AC_AttitudeControl_Multi::deleaves_controller_acro(float lateral, float forward, float yaw, float throttle)
{
    _motors.set_lateral(lateral*RMAX_ACTUATOR_THRUST);
    _motors.set_forward(forward*PMAX_ACTUATOR_THRUST);
    _motors.set_yaw(yaw*PMAX_ACTUATOR_THRUST);
    _motors.set_throttle(throttle);
}



void AC_AttitudeControl_Multi::deleaves_controller_stabilize(float lateral, float forward, float yaw, float throttle, bool armed)
{
    //run at 50Hz

    //Initialize target yaw to the value of yaw when not armed or update it with joystick when armed
    target_yaw = !armed ? ahrs_ang.z : target_yaw + yaw*YAW_SENSITIVITY;
    yaw_angle_error= target_yaw-ahrs_ang.z ;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ahrs_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ahrs_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // For logging purpose
    _attitude_target_euler_angle.x = 0.0f;
    _attitude_target_euler_angle.y = 0.0f;
    _attitude_target_euler_angle.z = target_yaw;

    constrainCommand();

    _motors.set_lateral(lateral*RMAX_ACTUATOR_THRUST);
    _motors.set_forward(forward*PMAX_ACTUATOR_THRUST);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(throttle);

    // For logging purpose
    _rate_target_ang_vel.x = lateral*RMAX_ACTUATOR_THRUST;
    _rate_target_ang_vel.y = forward*PMAX_ACTUATOR_THRUST;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
    
}

void AC_AttitudeControl_Multi::deleaves_controller_forHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed)
{
    // Control runs at 50Hz
    lowPassDataFilter();

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    target_yaw = !armed ? ctrl_ang.z : target_yaw + yaw*YAW_SENSITIVITY;

    // Yaw PD control here
    yaw_angle_error= target_yaw-ctrl_ang.z;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // Pitch PD control here
    target_forward = forward;   
    forward_error= target_forward-ctrl_ang.y;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command= pitch_kp*forward_error+pitch_kd*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
    forward_error_last=forward_error; //assign new error to last
            
    
    // Convert force command to motor command (0 to 1)
    constrainCommand();
    forward_command=sequenceArmed?forward_command:0.0f;

    // For logging purpose
    _attitude_target_euler_angle.x = 0.0f;
    _attitude_target_euler_angle.y = target_forward;
    _attitude_target_euler_angle.z = target_yaw;

    _motors.set_lateral(lateral*RMAX_ACTUATOR_THRUST);
    _motors.set_forward(forward_command);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(throttle);

    // For logging purpose
    _rate_target_ang_vel.x = lateral*RMAX_ACTUATOR_THRUST;
    _rate_target_ang_vel.y = forward_command;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
}

void AC_AttitudeControl_Multi::deleaves_controller_latHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed)
{
    // Control runs at 50Hz

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    target_yaw = !armed ? ctrl_ang.z : target_yaw + yaw*YAW_SENSITIVITY;

    lowPassDataFilter();

    // Yaw PD control here
    yaw_angle_error= target_yaw-ctrl_ang.z;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    target_lateral = lateral;

    lateral_error= target_lateral-ctrl_ang.x;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= roll_kp*lateral_error+roll_kd*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last

    // Convert force command to motor command (0 to 1)
    constrainCommand();
    lateral_command=sequenceArmed?lateral_command:0.0f;

    // For logging purpose
    _attitude_target_euler_angle.x = target_lateral;
    _attitude_target_euler_angle.y = 0.0f;
    _attitude_target_euler_angle.z = target_yaw;

    _motors.set_lateral(lateral_command);
    _motors.set_forward(forward*PMAX_ACTUATOR_THRUST);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(throttle);

    // For logging purpose
    _rate_target_ang_vel.x = lateral_command;
    _rate_target_ang_vel.y = forward*PMAX_ACTUATOR_THRUST;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
}

void AC_AttitudeControl_Multi::deleaves_controller_approachHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed)
{
    // Control runs at 50Hz
    lowPassDataFilter();

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    target_yaw = !armed ? ctrl_ang.z : target_yaw + yaw*YAW_SENSITIVITY;

    // Yaw PD control here
    yaw_angle_error= target_yaw-ctrl_ang.z;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    target_lateral = lateral;
    target_forward = forward;
    
    lateral_error= target_lateral-ctrl_ang.x;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= roll_kp*lateral_error+roll_kd*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last


    forward_error= target_forward-ctrl_ang.y;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command= roll_kp*forward_error+roll_kd*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
    forward_error_last=forward_error; //assign new error to last

    // Convert force command to motor command (0 to 1)
    constrainCommand();
    lateral_command=sequenceArmed?lateral_command:0.0f;
    forward_command=sequenceArmed?forward_command:0.0f;

    // For logging purpose
    _attitude_target_euler_angle.x = target_lateral;
    _attitude_target_euler_angle.y = target_forward;
    _attitude_target_euler_angle.z = target_yaw;

    _motors.set_lateral(lateral_command);
    _motors.set_forward(forward_command);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(throttle);

    // For logging purpose
    _rate_target_ang_vel.x = lateral_command;
    _rate_target_ang_vel.y = forward_command;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
}

void AC_AttitudeControl_Multi::deleaves_controller_angVelHold_PD(float lateral, float forward, float yaw, float throttle, bool armed, bool reset_command)
{
    // Control runs at 50Hz
    // Low pass filter to reduce vibration in data
    lowPassDataFilter();

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    if(!armed)
    {
        target_yaw = ctrl_ang.z;
        target_forward = 0.0f;
        target_lateral = 0.0f;
    }
    else
    {
        // Yaw control, simple deadband check
        if(abs(yaw) > DEADBAND)
        {
            target_yaw += yaw*YAW_SENSITIVITY;
        }

        // Forward control, velocity on move, angular on hold
        if(reset_command)
        {
            target_forward = 0.0f;
        }
        else if(abs(forward) > DEADBAND)
        {
            target_forward += forward*pitch_sensitivity;
        }
        
        // Lateral control, based on the same principle as forward control
        if(reset_command)
        {
            target_lateral = 0.0f;
        }
        else if(abs(lateral) > DEADBAND)
        {
            target_lateral += lateral*roll_sensitivity;
        }
    }    
    target_forward = constrain_value(target_forward, MIN_PITCH, MAX_PITCH);
    target_lateral = constrain_value(target_lateral, MIN_ROLL, MAX_ROLL);

    // Yaw PD control
    yaw_angle_error= target_yaw-ctrl_ang.z;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // Roll PD control
    lateral_error= target_lateral-ctrl_ang.x;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= roll_kp*lateral_error+roll_kd*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last

    // Pitch PD control
    forward_error= target_forward-ctrl_ang.y;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command = pitch_kp*forward_error+pitch_kd*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
    forward_error_last=forward_error; //assign new error to last

    constrainCommand();

    // For logging purpose
    _attitude_target_euler_angle.x = target_lateral;
    _attitude_target_euler_angle.y = target_forward;
    _attitude_target_euler_angle.z = target_yaw;

    _motors.set_lateral(lateral_command);
    _motors.set_forward(forward_command);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(throttle);

    // For logging purpose
    _rate_target_ang_vel.x = lateral_command;
    _rate_target_ang_vel.y = forward_command;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
}

void AC_AttitudeControl_Multi::deleaves_controller_taxi(float yaw, bool armed)
{
    // Control runs at 50Hz
    // Low pass filter to reduce vibration in data
    lowPassDataFilter();

    target_forward = 0.0f;
    target_lateral = 0.0f;

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    if(!armed)
    {
        target_yaw = ctrl_ang.z;
    }
    // Yaw control, simple deadband check
    else if(abs(yaw) > DEADBAND)
    {
        target_yaw += yaw*YAW_SENSITIVITY;
    }   

    // Yaw PD control
    yaw_angle_error= target_yaw-ctrl_ang.z;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ctrl_ang.z;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yaw_kp*yaw_angle_error+yaw_kd*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    float roll_kp_taxi = 28.0f;
    float roll_kd_taxi = 46.0f;
    float pitch_kp_taxi = 50.0f;
    float pitch_kd_taxi = 50.0f;

    // Roll PD control
    lateral_error= target_lateral-ctrl_ang.x;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= roll_kp_taxi*lateral_error+roll_kd_taxi*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last

    // Pitch PD control
    forward_error= target_forward-ctrl_ang.y;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command = pitch_kp_taxi*forward_error+pitch_kd_taxi*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
    forward_error_last=forward_error; //assign new error to last

    constrainCommand();

    // For logging purpose
    _attitude_target_euler_angle.x = target_lateral;
    _attitude_target_euler_angle.y = target_forward;
    _attitude_target_euler_angle.z = target_yaw;

    _motors.set_lateral(lateral_command);
    _motors.set_forward(forward_command);
    _motors.set_yaw(yaw_input);
    _motors.set_throttle(0.0f);

    // For logging purpose
    _rate_target_ang_vel.x = lateral_command;
    _rate_target_ang_vel.y = forward_command;
    _rate_target_ang_vel.z = yaw_input;
    _rate_sysid_ang_vel = ctrl_ang;
    _attitude_target_ang_vel = ds_filtered_ang;
}

void AC_AttitudeControl_Multi::constrainCommand()
{
    yaw_input=constrain_float(yaw_input,-PMAX_ACTUATOR_THRUST,PMAX_ACTUATOR_THRUST);
    lateral_command=constrain_float(lateral_command,-RMAX_ACTUATOR_THRUST,RMAX_ACTUATOR_THRUST);
    forward_command=constrain_float(forward_command,-PMAX_ACTUATOR_THRUST,PMAX_ACTUATOR_THRUST);
}

float AC_AttitudeControl_Multi::get_mamba_length()
{
    return _pid_rate_pitch.kI();
}
