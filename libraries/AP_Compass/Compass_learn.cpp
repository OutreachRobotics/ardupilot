#include <AP_AHRS/AP_AHRS.h>

#include <AP_Compass/AP_Compass.h>

#include "Compass_learn.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/EKFGSF_yaw.h>

#if COMPASS_LEARN_ENABLED

extern const AP_HAL::HAL &hal;

// constructor
CompassLearn::CompassLearn(Compass &_compass) :
    compass(_compass)
{
    gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: Initialised");
}

// accuracy threshold applied for GSF yaw estimate
#define YAW_ACCURACY_THRESHOLD_DEG 5.0

/*
  update when new compass sample available
 */
void CompassLearn::update(void)
{
    const AP_Vehicle *vehicle = AP::vehicle();
    if (compass.get_learn_type() != Compass::LEARN_INFLIGHT ||
        !hal.util->get_soft_armed() ||
        vehicle->get_time_flying_ms() < 3000) {
        // only learn when flying and with enough time to be clear of
        // the ground
        return;
    }

    const auto &ahrs = AP::ahrs();
    const auto *gsf = ahrs.get_yaw_estimator();
    if (gsf == nullptr) {
        // no GSF available
        return;
    }
    if (degrees(fabsf(ahrs.get_pitch())) > 50) {
        // we don't want to be too close to nose up, or yaw gets
        // problematic. Tailsitters need to wait till they are in
        // forward flight
        return;
    }

    AP_Notify::flags.compass_cal_running = true;

    ftype yaw_rad, yaw_variance;
    uint8_t n_clips;
    if (!gsf->getYawData(yaw_rad, yaw_variance, &n_clips) ||
        !is_positive(yaw_variance) ||
        n_clips > 1 ||
        yaw_variance >= sq(radians(YAW_ACCURACY_THRESHOLD_DEG))) {
        // not converged
        return;
    }

<<<<<<< HEAD
    Vector3f field = compass.get_field(0);
    Vector3f field_change = field - last_field;
    if (field_change.length() < min_field_change) {
        return;
    }

    {
        WITH_SEMAPHORE(sem);
        // give a sample to the backend to process
        new_sample.field = field;
        new_sample.offsets = compass.get_offsets(0);
        new_sample.attitude = Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw);
        sample_available = true;
        last_field = field;
        num_samples++;
    }

    if (sample_available) {
        // @LoggerMessage: COFS
        // @Description: Current compass learn offsets
        // @Field: TimeUS: Time since system startup
        // @Field: OfsX: best learnt offset, x-axis
        // @Field: OfsY: best learnt offset, y-axis
        // @Field: OfsZ: best learnt offset, z-axis
        // @Field: Var: error of best offset vector
        // @Field: Yaw: best learnt yaw
        // @Field: WVar: error of best learn yaw
        // @Field: N: number of samples used
        AP::logger().Write("COFS", "TimeUS,OfsX,OfsY,OfsZ,Var,Yaw,WVar,N", "QffffffI",
                                               AP_HAL::micros64(),
                                               (double)best_offsets.x,
                                               (double)best_offsets.y,
                                               (double)best_offsets.z,
                                               (double)best_error,
                                               (double)best_yaw_deg,
                                               (double)worst_error,
                                               num_samples);
    }

    if (!converged) {
        WITH_SEMAPHORE(sem);

        // set offsets to current best guess
        compass.set_offsets(0, best_offsets);

        // set non-primary offsets to match primary
        Vector3f field_primary = compass.get_field(0);
        for (uint8_t i=1; i<compass.get_count(); i++) {
            if (!compass._use_for_yaw[Compass::Priority(i)]) {
                continue;
            }
            Vector3f field2 = compass.get_field(i);
            Vector3f new_offsets = compass.get_offsets(i) + (field_primary - field2);
            compass.set_offsets(i, new_offsets);
        }

        // stop updating the offsets once converged
        if (num_samples > COMPASS_LEARN_NUM_SAMPLES &&
            best_error < COMPASS_LEARN_BEST_ERROR_THRESHOLD &&
            worst_error > COMPASS_LEARN_WORST_ERROR_THRESHOLD) {
            // set the offsets and enable compass for EKF use. Let the
            // EKF learn the remaining compass offset error
            for (uint8_t i=0; i<compass.get_count(); i++) {
                if (compass._use_for_yaw[Compass::Priority(i)]) {
                    compass.save_offsets(i);
                    compass.set_and_save_scale_factor(i, 0.0);
                }
            }
            compass.set_learn_type(Compass::LEARN_NONE, true);
            // setup so use can trigger it again
            converged = false;
            sample_available = false;
            num_samples = 0;
            have_earth_field = false;
            for (auto &v : predicted_offsets) {
                v.zero();
            }
            worst_error = 0;
            best_error = 0;
            best_yaw_deg = 0;
            best_offsets.zero();
            gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: finished");
            AP_Notify::flags.compass_cal_running = false;
            AP_Notify::events.compass_cal_saved = true;
        }
    }
}

/*
  we run the math intensive calculations in the IO thread
 */
void CompassLearn::io_timer(void)
{
    if (!sample_available) {
        return;
    }

    struct sample s;

    {
        WITH_SEMAPHORE(sem);
        s = new_sample;
        sample_available = false;
    }

    process_sample(s);
}

/*
  process a new compass sample
 */
void CompassLearn::process_sample(const struct sample &s)
{
    uint16_t besti = 0;
    float bestv = 0, worstv=0;

    /*
      we run through the 72 possible yaw error values, and for each
      one we calculate a value for the compass offsets if that yaw
      error is correct. 
     */
    for (uint16_t i=0; i<num_sectors; i++) {
        float yaw_err_deg = i*(360/num_sectors);

        // form rotation matrix for the euler attitude
        Matrix3f dcm;
        dcm.from_euler(s.attitude.x, s.attitude.y, wrap_2PI(s.attitude.z + radians(yaw_err_deg)));

        // calculate the field we would expect to get if this yaw error is correct
        Vector3f expected_field = dcm.transposed() * mag_ef;

        // calculate a value for the compass offsets for this yaw error
        Vector3f v1 = mat * s.field;
        Vector3f v2 = mat * expected_field;
        Vector3f offsets = (v2 - v1) + s.offsets;
        float delta = (offsets - predicted_offsets[i]).length();

        if (num_samples == 1) {
            predicted_offsets[i] = offsets;
        } else {
            // lowpass the predicted offsets and the error
            const float learn_rate = 0.92f;
            predicted_offsets[i] = predicted_offsets[i] * learn_rate + offsets * (1-learn_rate);
            errors[i] = errors[i] * learn_rate + delta * (1-learn_rate);
        }

        // keep track of the current best prediction
        if (i == 0 || errors[i] < bestv) {
            besti = i;
            bestv = errors[i];
        }
        // also keep the worst error. This is used as part of the convergence test
        if (i == 0 || errors[i] > worstv) {
            worstv = errors[i];
        }
    }

    WITH_SEMAPHORE(sem);

    // pass the current estimate to the front-end
    best_offsets = predicted_offsets[besti];
    best_error = bestv;
    worst_error = worstv;
    best_yaw_deg = wrap_360(degrees(s.attitude.z) + besti * (360/num_sectors));

    // send current learn state to gcs
    const uint32_t now = AP_HAL::millis();
    if (!converged && now - last_learn_progress_sent_ms >= 5000) {
        float percent = (MIN(num_samples / COMPASS_LEARN_NUM_SAMPLES, 1.0f) + 
                         MIN(COMPASS_LEARN_BEST_ERROR_THRESHOLD / (best_error + 1.0f), 1.0f) + 
                         MIN(worst_error / COMPASS_LEARN_WORST_ERROR_THRESHOLD, 1.0f)) / 3.0f * 100.f;
        gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: %d%%", (int) percent);
        last_learn_progress_sent_ms = now;
=======
    const auto result = compass.mag_cal_fixed_yaw(degrees(yaw_rad), (1U<<HAL_COMPASS_MAX_SENSORS)-1, 0, 0, true);
    if (result == MAV_RESULT_ACCEPTED) {
        AP_Notify::flags.compass_cal_running = false;
        compass.set_learn_type(Compass::LEARN_NONE, true);
        gcs().send_text(MAV_SEVERITY_INFO, "CompassLearn: Finished");
>>>>>>> Copter-4.2.3
    }
}

#endif // COMPASS_LEARN_ENABLED
