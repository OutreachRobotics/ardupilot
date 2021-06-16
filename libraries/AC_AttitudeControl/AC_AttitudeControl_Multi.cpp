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
    // Vector3f ang_vel = _ahrs.get_gyro_latest();
    // Quaternion attitude_vehicle_quat;
    // _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    // float ahrs_roll, ahrs_pitch, ahrs_yaw;
    // attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

    _motors.set_lateral(lateral*RMAX_ACTUATOR_THRUST);
    _motors.set_forward(forward*PMAX_ACTUATOR_THRUST);
    _motors.set_yaw(yaw*PMAX_ACTUATOR_THRUST);
    _motors.set_throttle(throttle);
}



void AC_AttitudeControl_Multi::deleaves_controller_stabilize(float lateral, float forward, float yaw, float throttle, bool armed)
{
    //run at 50Hz
        //Vector3f ang_vel = _ahrs.get_gyro_latest();
        Quaternion attitude_vehicle_quat;
        _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);

        float ahrs_roll, ahrs_pitch, ahrs_yaw;
        attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

        //Initialize target yaw to the value of yaw when not armed or update it with joystick when armed
        target_yaw = !armed ? ahrs_yaw : target_yaw + yaw*YAW_SENSITIVITY;


        // Yaw PD control here
        float GainP = _pid_rate_yaw.kP();
        float GainD = _pid_rate_yaw.kD();
        yaw_angle_error= target_yaw-ahrs_yaw ;

        //Correction for target angle more than half-turn away
        if (yaw_angle_error>M_PI){
            target_yaw=target_yaw-2*M_PI;
            yaw_angle_error= target_yaw-ahrs_yaw ;
            yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
        }

        if (yaw_angle_error<-M_PI){
            target_yaw=target_yaw+2*M_PI;
            yaw_angle_error= target_yaw-ahrs_yaw;
            yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
        }

        yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
        yaw_input= GainP*yaw_angle_error+GainD*yaw_angle_error_dt;
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
        _rate_sysid_ang_vel.x = 0.0f;
        _rate_sysid_ang_vel.y = 0.0f;
        _rate_sysid_ang_vel.z = 0.0f;
    
}

void AC_AttitudeControl_Multi::deleaves_controller_forHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed)
{
    // Control runs at 50Hz

    // Vector3f ang_vel = _ahrs.get_gyro_latest();

    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    float ahrs_roll, ahrs_pitch, ahrs_yaw;
    attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    target_yaw = !armed ? ahrs_yaw : target_yaw + yaw*YAW_SENSITIVITY;

    // Yaw PD control here
    float yawGainP = _pid_rate_yaw.kP();
    float yawGainD = _pid_rate_yaw.kD();
    yaw_angle_error= target_yaw-ahrs_yaw;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yawGainP*yaw_angle_error+yawGainD*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // Pitch PD control here
    target_forward = forward;

    float forwardGainP = _pid_rate_pitch.kP();
    float forwardGainD = _pid_rate_pitch.kD();
    forward_error= target_forward-ahrs_pitch;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command= forwardGainP*forward_error+forwardGainD*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
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
    _rate_sysid_ang_vel.x = 0.0f;
    _rate_sysid_ang_vel.y = 0.0f;
    _rate_sysid_ang_vel.z = 0.0f;
}

void AC_AttitudeControl_Multi::deleaves_controller_latHold(float lateral, float forward, float yaw, float throttle, bool sequenceArmed, bool armed)
{
    // Control runs at 50Hz
    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    float ahrs_roll, ahrs_pitch, ahrs_yaw;
    attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    target_yaw = !armed ? ahrs_yaw : target_yaw + yaw*YAW_SENSITIVITY;

    // Yaw PD control here
    float yawGainP = _pid_rate_yaw.kP();
    float yawGainD = _pid_rate_yaw.kD();
    yaw_angle_error= target_yaw-ahrs_yaw;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yawGainP*yaw_angle_error+yawGainD*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    target_lateral = lateral;

    // Roll PD control
    float lateralGainP = _pid_rate_roll.kP();
    float lateralGainD = _pid_rate_roll.kD();
    lateral_error= target_lateral-ahrs_roll;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= lateralGainP*lateral_error+lateralGainD*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last

    // Filtered roll PD control
    // float lateralGainP = _pid_rate_roll.kP();
    // float lateralGainD = _pid_rate_roll.kD();
    // float b1 = 0.5792f;
    // float b0 = 0.5792f;
    // float a0 = 0.1584f;
    // lateral_error= target_lateral-ahrs_roll;
    // filtered_lateral_error = b1*lateral_error + b0*lateral_error_last - a0*last_filtered_lateral_error;
    // lateral_error_dt=(filtered_lateral_error-last_filtered_lateral_error)*50; //50 Hz
    // lateral_command= lateralGainP*lateral_error+lateralGainD*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    // lateral_error_last=lateral_error; //assign new error to last
    // last_filtered_lateral_error = filtered_lateral_error;

    // Tachymeter feedback
    // Vector3f ang_vel = _ahrs.get_gyro_latest();
    // float lateralGainP = _pid_rate_roll.kP();
    // float lateralGainD = _pid_rate_roll.kD();
    // lateral_error = target_lateral-ahrs_roll;
    // lateral_command = lateralGainP*(lateral_error - lateralGainD/lateralGainP*ang_vel.x) + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);

    // LQT control
    // Vector3f ang_vel = _ahrs.get_gyro_latest();
    // float k1 = 87.5302f;
    // float k2 = -199.5141f;
    // lateral_error = target_lateral-ahrs_roll;
    // lateral_command = -k1*ang_vel.x - k2*lateral_error + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);

    // Filtered LQT control
    // float k1 = 30.7898f;
    // float k2 = -64.6907f;
    // float b1 = 0.5792f;
    // float b0 = 0.5792f;
    // float a0 = 0.1584f;
    // Vector3f ang_vel = _ahrs.get_gyro_latest();
    // lateral_error= target_lateral-ahrs_roll;
    // filtered_lateral_error = b1*lateral_error + b0*lateral_error_last - a0*last_filtered_lateral_error;
    // filtered_angVel = b1*ang_vel.x + b0*last_angVel.x - a0*last_filtered_angVel;
    // lateral_command = -k1*filtered_angVel - k2*filtered_lateral_error + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    // lateral_error_last=lateral_error; //assign new error to last
    // last_filtered_lateral_error = filtered_lateral_error;
    // last_angVel = ang_vel;
    // last_filtered_angVel = filtered_angVel;

    // Filtered Tachymeter feedback
    // float b1 = 0.5792f;
    // float b0 = 0.5792f;
    // float a0 = 0.1584f;
    // float lateralGainP = _pid_rate_roll.kP();
    // float lateralGainD = _pid_rate_roll.kD();
    // lateral_error = target_lateral-ahrs_roll;
    // Vector3f ang_vel = _ahrs.get_gyro_latest();
    // filtered_lateral_error = b1*lateral_error + b0*lateral_error_last - a0*last_filtered_lateral_error;
    // filtered_angVel = b1*ang_vel.x + b0*last_angVel.x - a0*last_filtered_angVel;
    // lateral_command = lateralGainP*(filtered_lateral_error - lateralGainD/lateralGainP*filtered_angVel) + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    // lateral_error_last=lateral_error; //assign new error to last
    // last_filtered_lateral_error = filtered_lateral_error;
    // last_angVel = ang_vel;
    // last_filtered_angVel = filtered_angVel;

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
    _rate_sysid_ang_vel.x = 0.0f;
    _rate_sysid_ang_vel.y = 0.0f;
    _rate_sysid_ang_vel.z = 0.0f;
}

void AC_AttitudeControl_Multi::deleaves_controller_angVelHold_PD(float lateral, float forward, float yaw, float throttle, bool armed, bool reset_command)
{
    // Control runs at 50Hz

    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    float ahrs_roll, ahrs_pitch, ahrs_yaw;
    attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    if(!armed)
    {
        target_yaw = ahrs_yaw;
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
            target_forward += forward*PITCH_SENSITIVITY;
        }
        
        // Lateral control, based on the same principle as forward control
        if(reset_command)
        {
            target_lateral = 0.0f;
        }
        else if(abs(lateral) > DEADBAND)
        {
            target_lateral += lateral*ROLL_SENSITIVITY;
        }
    }    
    target_forward = constrain_value(target_forward, MIN_PITCH, MAX_PITCH);
    target_lateral = constrain_value(target_lateral, MIN_ROLL, MAX_ROLL);

    // Yaw PD control
    float yawGainP = _pid_rate_yaw.kP();
    float yawGainD = _pid_rate_yaw.kD();
    yaw_angle_error= target_yaw-ahrs_yaw;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yawGainP*yaw_angle_error+yawGainD*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // Roll PD control
    float lateralGainP = _pid_rate_roll.kP();
    float lateralGainD = _pid_rate_roll.kD();
    lateral_error= target_lateral-ahrs_roll;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= lateralGainP*lateral_error+lateralGainD*lateral_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_lateral);
    lateral_error_last=lateral_error; //assign new error to last

    // Pitch PD control
    float forwardGainP = _pid_rate_pitch.kP();
    float forwardGainD = _pid_rate_pitch.kD();
    forward_error= target_forward-ahrs_pitch;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command = forwardGainP*forward_error+forwardGainD*forward_error_dt + M_PLATFORM*GRAVITY_MSS*sinf(target_forward);
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
    _rate_sysid_ang_vel.x = 0.0f;
    _rate_sysid_ang_vel.y = 0.0f;
    _rate_sysid_ang_vel.z = 0.0f;

}

void AC_AttitudeControl_Multi::deleaves_controller_taxi(float yaw, bool armed)
{
    // Control runs at 50Hz

    Vector3f ang_vel = _ahrs.get_gyro_latest();

    Quaternion attitude_vehicle_quat;
    _ahrs.get_quat_body_to_ned(attitude_vehicle_quat);
    float ahrs_roll, ahrs_pitch, ahrs_yaw;
    attitude_vehicle_quat.to_euler(ahrs_roll, ahrs_pitch, ahrs_yaw);

    target_forward = 0.0f;
    target_lateral = 0.0f;

    //Initialize target angle to the value of angle when not armed or update it with joystick when armed
    if(!armed)
    {
        target_yaw = ahrs_yaw;
    }
    else if(abs(yaw) > DEADBAND)
    {
        target_yaw += yaw*YAW_SENSITIVITY;
    }

    // Yaw PD control here
    float yawGainP = _pid_rate_yaw.kP();
    float yawGainD = _pid_rate_yaw.kD();
    yaw_angle_error= target_yaw-ahrs_yaw;

    //Correction for target angle more than half-turn away
    if (yaw_angle_error>M_PI){
        target_yaw=target_yaw-2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw ;
        yaw_angle_error_last=yaw_angle_error_last-2*M_PI;
    }

    if (yaw_angle_error<-M_PI){
        target_yaw=target_yaw+2*M_PI;
        yaw_angle_error= target_yaw-ahrs_yaw;
        yaw_angle_error_last=yaw_angle_error_last+2*M_PI;
    }

    yaw_angle_error_dt=(yaw_angle_error-yaw_angle_error_last)*50; //50 Hz
    yaw_input= yawGainP*yaw_angle_error+yawGainD*yaw_angle_error_dt;
    yaw_angle_error_last=yaw_angle_error; //assign new error to last

    // Roll PD control here
    float lateralGainP = _pid_rate_roll.kP();
    float lateralGainD = _pid_rate_roll.kD();
    lateral_error=  target_lateral - ang_vel.x;
    lateral_error_dt=(lateral_error-lateral_error_last)*50; //50 Hz
    lateral_command= lateralGainP*lateral_error+lateralGainD*lateral_error_dt;
    lateral_error_last=lateral_error; //assign new error to last

    // Pitch PD control here
    float forwardGainP = _pid_rate_pitch.kP();
    float forwardGainD = _pid_rate_pitch.kD();
    forward_error= target_forward-ang_vel.y;
    forward_error_dt=(forward_error-forward_error_last)*50; //50 Hz
    forward_command= forwardGainP*forward_error+forwardGainD*forward_error_dt;
    forward_error_last=forward_error; //assign new error to last

    // Convert force command to motor command (-1 to 1)
    constrainCommand();

    _motors.set_lateral(lateral_command);
    _motors.set_forward(forward_command);
    _motors.set_yaw(target_yaw);
}

void AC_AttitudeControl_Multi::constrainCommand()
{
    yaw_input=constrain_float(yaw_input,-PMAX_ACTUATOR_THRUST,PMAX_ACTUATOR_THRUST);
    lateral_command=constrain_float(lateral_command,-RMAX_ACTUATOR_THRUST,RMAX_ACTUATOR_THRUST);
    forward_command=constrain_float(forward_command,-PMAX_ACTUATOR_THRUST,PMAX_ACTUATOR_THRUST);
}
