/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_ModernDevice.h"
// read wind speed from Modern Device rev p wind sensor
// https://moderndevice.com/news/calibrating-rev-p-wind-sensor-new-regression/

// constructor
AP_ModernDevice::AP_ModernDevice()
{
    initialized = false;
}

void AP_ModernDevice::init()
{
    initialized = true;

    _speed_sensor_speed_pin = WINDSPEED_SPEED_PIN;
    _speed_sensor_temp_pin = -1;

    _speed_analog_source = hal.analogin->channel(WINDSPEED_SPEED_PIN);
    // _temp_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);
    _calibration_time = AP_HAL::millis();
    _calibration = 0;
    _speed_sensor_voltage_offset = WINDSPEED_DEFAULT_VOLT_OFFSET;
    _speed_filt_hz = -1.0f;

    _speed_true = 0.0f;                       
    _temp = 28.0f;
    _wind_voltage = 0.0f;
    _temp_voltage = 0.0f;
}

// update wind vane, expected to be called at 20hz
void AP_ModernDevice::update()
{
    if(!initialized)
    {
        init();
    }
    if (AP_HAL::millis() - _calibration_time < 15000) {
        calibrate();
    } 
    else if((AP_HAL::millis() - _calibration_time > 15000) && !_calibration)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "WindVane: rev P. zero wind voltage offset set to %.3f",double(_speed_sensor_voltage_offset));
        _calibration = 1;
    }


    // read wind speed
    update_speed();
    send_wind(MAVLINK_COMM_0);
}

void AP_ModernDevice::update_speed()
{
    // only read temp pin if defined, sensor will do OK assuming constant temp
    float temp_ambient = 28.0f; // equations were generated at this temp in above data sheet
    if (_speed_sensor_temp_pin > 0) {
        _temp_analog_source->set_pin(_speed_sensor_temp_pin);
        _temp_voltage = _temp_analog_source->voltage_average();
        temp_ambient = (_temp_voltage - 0.4f) / 0.0195f; // deg C
        // constrain to reasonable range to avoid deviating from calibration too much and potential divide by zero
        temp_ambient = constrain_float(temp_ambient, 10.0f, 40.0f);
    }

    _speed_analog_source->set_pin(_speed_sensor_speed_pin);
    _wind_voltage = _speed_analog_source->voltage_average();

    float analog_voltage;
    // apply voltage offset and make sure not negative
    // by default the voltage offset is the number provide by the manufacturer
    analog_voltage = _wind_voltage - _speed_sensor_voltage_offset;
    if (is_negative(analog_voltage)) {
        analog_voltage = 0.0f;
    }

    // simplified equation from data sheet, converted from mph to m/s
    // _speed_true = (24.254896f * powf((analog_voltage / powf(temp_ambient, 0.115157f)), 3.009364f));
    _speed_true = 0.1504759105*expf(2.10908635 * analog_voltage);
}

void AP_ModernDevice::calibrate()
{
    _speed_analog_source->set_pin(_speed_sensor_speed_pin);
    _speed_sensor_voltage_offset = _speed_analog_source->voltage_average();
}

// send mavlink wind message
void AP_ModernDevice::send_wind(mavlink_channel_t chan)
{
    // send wind
    mavlink_msg_wind_send(
        chan,
        _speed_sensor_voltage_offset,
        _speed_true,
        _wind_voltage);
}

float AP_ModernDevice::get_wind_speed()
{
    return _speed_true;
}

float AP_ModernDevice::get_wind_voltage()
{
    return _wind_voltage;
}

float AP_ModernDevice::get_temp()
{
    return _temp;
}

float AP_ModernDevice::get_temp_voltage()
{
    return _temp_voltage;
}

float AP_ModernDevice::get_voltage_offset()
{
    return _speed_sensor_voltage_offset;
}
