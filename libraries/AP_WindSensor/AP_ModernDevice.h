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
#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>

#define WINDSPEED_SPEED_PIN 18              // default pin for reading speed from ModernDevice rev p wind sensor
#define WINDSPEED_DEFAULT_TEMP_PIN 13               // default pin for reading temperature from ModernDevice rev p wind sensor
#define WINDSPEED_DEFAULT_VOLT_OFFSET 1.346f        // default voltage offset between speed and temp pins from ModernDevice rev p wind sensor


extern const AP_HAL::HAL& hal;

class AP_ModernDevice
{
public:
    // constructor
    AP_ModernDevice();

    // update state
    void init();
    void update();
    void update_speed();
    void calibrate();
    void send_wind(mavlink_channel_t chan);

    float get_wind_speed();
    float get_wind_voltage();
    float get_temp();
    float get_temp_voltage();
    float get_voltage_offset();
    

private:
    // parameters
    uint64_t _calibration_time;
    int8_t _calibration;                           // enter calibration
    int8_t _speed_sensor_speed_pin;                // speed sensor analog pin for reading speed
    int8_t _speed_sensor_temp_pin;                // speed sensor analog pin for reading temp, -1 if disable
    float _speed_sensor_voltage_offset;          // analog speed zero wind voltage offset
    float _speed_filt_hz;                        // speed sensor low pass filter frequency

    float _speed_true;                              // wind's true estimated speed in m/s
    float _temp;
    float _wind_voltage;
    float _temp_voltage;

    bool initialized;

    // pin for reading analog voltage
    AP_HAL::AnalogSource *_speed_analog_source;
    AP_HAL::AnalogSource *_temp_analog_source;

    // low pass filters for speed
    LowPassFilterFloat _speed_filt = LowPassFilterFloat(2.0f);
};
