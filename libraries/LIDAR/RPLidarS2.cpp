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

/*
 * ArduPilot device driver for SLAMTEC RPLIDAR A2 (16m range version)
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "RPLidarS2.h"
#include <ctype.h>
#include <stdio.h>

#define RP_DEBUG_LEVEL 2

#if RP_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200
#define RESET_RPA2_WAIT_MS              8
#define RESYNC_TIMEOUT                  5000

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_PREAMBLE               0xA5
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52
#define RPLIDAR_CMD_GET_SAMPLERATE     0x59
#define RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL 0xA8
#define DEFAULT_MOTOR_SPEED         (0xFFFFu)

// Commands with payload but no response
#define RPLIDAR_CMD_NEW_BAUDRATE_CONFIRM 0x90

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82
#define RPLIDAR_CMD_HQ_SCAN            0x83

extern const AP_HAL::HAL& hal;

//Constructor
RPLidarS2::RPLidarS2(void)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (_uart != nullptr) {
        // start uart with larger receive buffer
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0), rxspace(), 0);
    }
}

// update the _rp_state of the sensor
void RPLidarS2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    if (!_initialised) {
        _initialised = init();    //returns true if everything initialized properly
    }

    // if LIDAR in known state
    if (_initialised) {
        get_readings();
    }

}


bool RPLidarS2::init()
{


    if (!_initialised) {
        reset_rplidar();            // set to a known state
        Debug(1, "LIDAR initialised");
        set_motor_speed(DEFAULT_MOTOR_SPEED);          // Start motor to default speed
        _initialised = true;
    }

    return true;
}

void RPLidarS2::reset_rplidar()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    _resetted = true;   ///< be aware of extra 63 bytes coming after reset containing FW information
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    _rp_state = rp_resetted;

}

void RPLidarS2::set_motor_speed(uint16_t speed = DEFAULT_MOTOR_SPEED)
{
    if (_uart == nullptr) {
        return;
    }

    // Split speed to 2 uint8
    uint8_t speedarray[2];
    speedarray[0] = speed & 0xFF;
    speedarray[1] = (speed >> 8);

    uint8_t checksum = 0;
    uint8_t payloadsize = sizeof(speed);
    checksum ^= RPLIDAR_PREAMBLE;
    checksum ^= RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL;
    checksum ^= (payloadsize & 0xFF);

    uint8_t tx_buffer[6] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL, payloadsize, speedarray[0], speedarray[1], checksum};
    _uart->write(tx_buffer, 6);
    _last_request_ms = AP_HAL::millis();
    Debug(1, "MOTOR STARTED");
}

// set Lidar into SCAN mode
void RPLidarS2::set_scan_mode()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(1, "LIDAR SCAN MODE ACTIVATED");
    _rp_state = rp_responding;
}

// send request for sensor health
void RPLidarS2::send_request_for_health()                                    //not called yet
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    _rp_state = rp_health;
}

void RPLidarS2::get_readings()
{
    if (_uart == nullptr) {
        return;
    }
    Debug(2, "             CURRENT STATE: %d ", _rp_state);
    uint32_t nbytes = _uart->available();

    while (nbytes-- > 0) {

        uint8_t c = _uart->read();
        Debug(2, "UART READ %x <%c>", c, c); //show HEX values

        STATE:
        switch(_rp_state){

            case rp_resetted:
                Debug(3, "                  BYTE_COUNT %d", _byte_count);
                if ((c == 0x52 || _information_data) && _byte_count < 62) {
                    if (c == 0x52) {
                        _information_data = true;
                    }
                    _rp_systeminfo[_byte_count] = c;
                    Debug(3, "_rp_systeminfo[%d]=%x",_byte_count,_rp_systeminfo[_byte_count]);
                    _byte_count++;
                    break;
                } else {

                    if (_information_data) {
                        Debug(1, "GOT RPLIDAR INFORMATION");
                        _information_data = false;
                        _byte_count = 0;
                        set_scan_mode();
                        break;
                    }

                    if (_cnt>5) {
                        _rp_state = rp_unknown;
                        _cnt=0;
                        break;
                    }
                    _cnt++;
                    break;
                }
                break;

            case rp_responding:
                Debug(2, "RESPONDING");
                if (c == RPLIDAR_PREAMBLE || _descriptor_data) {
                    _descriptor_data = true;
                    _descriptor[_byte_count] = c;
                    _byte_count++;
                    // descriptor packet has 7 byte in total
                    if (_byte_count == sizeof(_descriptor)) {
                        Debug(2,"LIDAR DESCRIPTOR CATCHED");
                        _response_type = ResponseType_Descriptor;
                        // identify the payload data after the descriptor
                        parse_response_descriptor();
                        _byte_count = 0;
                    }
                } else {
                    _rp_state = rp_unknown;
                }
                break;

            case rp_measurements:
                if (_sync_error) {
                    // out of 5-byte sync mask -> catch new revolution
                    Debug(1, "       OUT OF SYNC");
                    // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
                    if ((c & 0x03) == 0x01) {
                        _sync_error = 0;
                        Debug(1, "                  RESYNC");
                    } else {
                        if (AP_HAL::millis() - _last_distance_received_ms > RESYNC_TIMEOUT) {
                            reset_rplidar();
                        }
                        break;
                    }
                }
                Debug(3, "READ PAYLOAD");

                // Load received bytes to payload struct
                payload[_byte_count] = c;
                _byte_count++;

                // When fully loaded, parse data to usable measures
                if (_byte_count == _payload_length) {
                    Debug(2, "LIDAR MEASUREMENT CATCHED");
                    parse_response_data();
                    _byte_count = 0;
                }
                break;

            case rp_health:
                Debug(1, "state: HEALTH");
                break;

            case rp_unknown:
                Debug(1, "state: UNKNOWN");
                if (c == RPLIDAR_PREAMBLE) {
                    _rp_state = rp_responding;
                    goto STATE;
                    break;
                }
                _cnt++;
                if (_cnt>10) {
                    reset_rplidar();
                    _rp_state = rp_resetted;
                    _cnt=0;
                }
                break;

            default:
                Debug(1, "UNKNOWN LIDAR STATE");
                break;
        }
    }
}

void RPLidarS2::parse_response_descriptor()
{
    // check if descriptor packet is valid
    if (_descriptor[0] == RPLIDAR_PREAMBLE && _descriptor[1] == 0x5A) {

        if (_descriptor[2] == 0x05 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x40 && _descriptor[6] == 0x81) {
            // payload is SCAN measurement data
            _payload_length = sizeof(payload.sensor_scan);
            static_assert(sizeof(payload.sensor_scan) == 5, "Unexpected payload.sensor_scan data structure size");
            _response_type = ResponseType_SCAN;
            Debug(2, "Measurement response detected");
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state = rp_measurements;
        }
        if (_descriptor[2] == 0x03 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x06) {
            // payload is health data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_health) == 3, "Unexpected payload.sensor_health data structure size");
            _response_type = ResponseType_Health;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_health;
        }
        return;
    }
    Debug(1, "Invalid response descriptor");
    _rp_state = rp_unknown;
}

void RPLidarS2::parse_response_data()
{
    switch (_response_type){
        case ResponseType_SCAN:
            Debug(2, "UART %02x %02x%02x %02x%02x", payload[0], payload[2], payload[1], payload[4], payload[3]); //show HEX values
            // check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
            if ((payload.sensor_scan.startbit == !payload.sensor_scan.not_startbit) && payload.sensor_scan.checkbit) {
                const float angle_deg = (payload.sensor_scan.angle_q14*90.f)/16384.f;
                const float distance_mm = (payload.sensor_scan.distance_q2/4.0f);
#if RP_DEBUG_LEVEL >= 2
                const float quality = payload.sensor_scan.quality;
                Debug(2, "                                       D%02.2f A%03.1f Q%02f", distance_mm, angle_deg, quality);
#endif
                _last_distance_received_ms = AP_HAL::millis();
                
            } else {
                // not valid payload packet
                Debug(1, "Invalid Payload");
                _sync_error++;
            }
            break;

        case ResponseType_Health:
            // health issue if status is "3" ->HW error
            if (payload.sensor_health.status == 3) {
                Debug(1, "LIDAR Error");
            }
            break;

        default:
            // no valid payload packets recognized: return payload data=0
            Debug(1, "Unknown LIDAR packet");
            break;
    }
}
