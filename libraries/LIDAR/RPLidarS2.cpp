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


// Protocol
//-----------------------------------------

#define SL_LIDAR_CMD_SYNC_BYTE              0xA5
#define SL_LIDAR_CMDFLAG_HAS_PAYLOAD        0x80

#define SL_LIDAR_ANS_SYNC_BYTE1             0xA5
#define SL_LIDAR_ANS_SYNC_BYTE2             0x5A

#define SL_LIDAR_ANS_PKTFLAG_LOOP           0x1

#define SL_LIDAR_ANS_HEADER_SIZE_MASK       0x3FFFFFFF

#define  MAX_SCAN_NODES  (8192)

#define DEFAULT_MOTOR_SPEED         (0xFFFFu)
// ------------------------------------------


// Commands
//-----------------------------------------

#define SL_LIDAR_AUTOBAUD_MAGICBYTE         0x41

// Commands without payload and response
#define SL_LIDAR_CMD_STOP                   0x25
#define SL_LIDAR_CMD_SCAN                   0x20
#define SL_LIDAR_CMD_FORCE_SCAN             0x21
#define SL_LIDAR_CMD_RESET                  0x40

// Commands with payload but no response
#define SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM   0x90 //added in fw 1.30

// Commands without payload but have response
#define SL_LIDAR_CMD_GET_DEVICE_INFO        0x50
#define SL_LIDAR_CMD_GET_DEVICE_HEALTH      0x52

#define SL_LIDAR_CMD_GET_SAMPLERATE         0x59 //added in fw 1.17

#define SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL    0xA8

// Commands with payload and have response
#define SL_LIDAR_CMD_EXPRESS_SCAN           0x82 //added in fw 1.17
#define SL_LIDAR_CMD_HQ_SCAN                0x83 //added in fw 1.24
#define SL_LIDAR_CMD_GET_LIDAR_CONF         0x84 //added in fw 1.24
#define SL_LIDAR_CMD_SET_LIDAR_CONF         0x85 //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define SL_LIDAR_CMD_SET_MOTOR_PWM          0xF0
#define SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     0xFF
// ------------------------------------------

// Response
// ------------------------------------------
#define SL_LIDAR_ANS_TYPE_DEVINFO          0x4
#define SL_LIDAR_ANS_TYPE_DEVHEALTH        0x6

#define SL_LIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x82
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ            0x83
#define SL_LIDAR_ANS_TYPE_MEASUREMENTT_ULTRA_DENSE_CAPSULED 0x86

// Added in FW ver 1.17
#define SL_LIDAR_ANS_TYPE_SAMPLE_RATE      0x15
//added in FW ver 1.23alpha
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA  0x84
//added in FW ver 1.24
#define SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF     0x20
#define SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF     0x21
#define SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED        0x85
#define SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

#define SL_LIDAR_STATUS_OK                 0x0
#define SL_LIDAR_STATUS_WARNING            0x1
#define SL_LIDAR_STATUS_ERROR              0x2

#define SL_LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2

#define SL_LIDAR_RESP_HQ_FLAG_SYNCBIT               (0x1<<0)

#define SL_LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
// ------------------------------------------

extern const AP_HAL::HAL& hal;

//Constructor
RPLidarS2::RPLidarS2(void)
{
    _state = initial;
    _isConnected = false;
}

// update the _state of the sensor, run at 10Hz
void RPLidarS2::update(void)
{
    switch (_state)
    {
    case initial:
        // Wait for UART to be established
        break;
    
    case connected:
        // Request a reset
        reset();
        _last_reset_ms =  AP_HAL::millis();
        _state = resetting;
        break;

    case resetting: 
        // wait 600 ms
        if(AP_HAL::millis() - _last_reset_ms == 600) _state = idle;
        break;

    case idle:
        // Request scan mode
        startScanNormal();
        setMotorSpeed(DEFAULT_MOTOR_SPEED);
        _state = scanning;
        break;

    case scanning:
        break;

    default:
        break;
    }

    if ( (AP_HAL::millis() - _update_debug) > 2000){
        Debug(1, "Still doing Lidar.update()");
        Debug(1, "CURRENT STATE: %d ", _state);
        _update_debug = AP_HAL::millis();
    }

}

sl_result RPLidarS2::connect()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);

    if (_uart != nullptr) {
        // start uart with larger receive buffer
        _uart->begin(1000000, 0, 0);
        _isConnected = true;
    }
    else {
        return SL_RESULT_FAIL_BIT;
    }

    return SL_RESULT_OK;
}

sl_result RPLidarS2::_sendCommand(uint8_t cmd, const void * cmdPayload = NULL, size_t payloadsize = 0 )
{
    uint8_t checksum = 0;

    uint8_t cmd_packet[60];
    uint8_t packet_size = 0;

    if (payloadsize && cmdPayload) {
        cmd |= SL_LIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    // Load to packet sync byte and command
    _uart->flush();
    cmd_packet[0] = SL_LIDAR_CMD_SYNC_BYTE ;
    packet_size++;
    cmd_packet[1] = cmd;
    packet_size++;
    
    // If there is a payload, add it to the cmd
    if (cmd & SL_LIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= SL_LIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // send size
        uint8_t sizebyte = payloadsize;
        cmd_packet[3] = sizebyte;
        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((uint8_t *)cmdPayload)[pos];
            cmd_packet[5 + pos] = ((uint8_t *)cmdPayload)[pos];
            packet_size++;
        }
        cmd_packet[packet_size - 1] = checksum;
        packet_size++;

    }

    // Load cmd to uart packet
    Debug(1, "Sending : ");
    uint8_t packet[1024];
    for (uint32_t pos = 0; pos < packet_size; pos++) {
        packet[pos] = cmd_packet[pos];
        Debug(1, "%x", packet[pos]);
    }
    
    _uart->write(packet, packet_size);
    return SL_RESULT_OK;
}

sl_result RPLidarS2::setMotorSpeed(uint16_t speed = DEFAULT_MOTOR_SPEED)
        {
            sl_result ans = SL_RESULT_OK;
            
            ans = _sendCommand(SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL, (const uint8_t *)&speed, sizeof(speed));
            if (!ans) return ans;
            
            return SL_RESULT_OK;
        }
    
sl_result RPLidarS2::startScanNormal()
{
    sl_result ans = SL_RESULT_OK;

    ans = _sendCommand( SL_LIDAR_CMD_SCAN );
    if (!ans) return ans;

        
    return SL_RESULT_OK;
}

sl_result RPLidarS2::reset()
{   
    sl_result ans = SL_RESULT_OK;
    ans = _sendCommand( SL_LIDAR_CMD_RESET );
    if (!ans) {
        return ans;
    }
    return SL_RESULT_OK;
}

sl_result RPLidarS2::stop()
{
    sl_result ans = SL_RESULT_OK;

    ans = _sendCommand(SL_LIDAR_CMD_STOP);
    if (!ans) return ans;

    setMotorSpeed(0);

    return SL_RESULT_OK;
}

// Manage UART connection and reading, run at 50Hz
void RPLidarS2::get_readings()
{
    switch (_state)
    {
        case initial:
            {
                // Connect to the UART port
                sl_result ans = SL_RESULT_OK;
                ans = connect();
                if(ans == SL_RESULT_OK)
                {
                    _state = connected;
                }
                
            }
            break;

        case scanning:
            // Read available bytes from the UART
            // Eliminate the descriptor message and save the data packets
            {
                uint32_t nbytes = _uart->available();

                while (nbytes-- > 0) {

                    // Discard what is left of a descriptor
                    if(_next_read_flush > 0)
                    {
                        for(int i = 0; i<_next_read_flush;i++)
                        {
                            _uart->read();
                            nbytes--;
                        }
                        _next_read_flush = 0;
                    }

                    uint8_t c = _uart->read();
        
                    // Check for response descriptor
                    if ( c == SL_LIDAR_ANS_SYNC_BYTE1)
                    {
                        // If every bytes of the descriptor is available
                        if(nbytes >= 6)
                        {   
                            // Discard the descriptor
                            for(int i = 0; i<6; i++){
                                _uart->read();
                                nbytes--;
                            }       
                        }
                        else
                        {
                            // Discard the available descriptor and save the number of bytes left
                            for(int i = 0; i<nbytes; i++){
                                _uart->read();
                                nbytes--;
                            } 
                            _next_read_flush = 6-nbytes;
                        }
                        
                    }

                    // Check for data packet
                    else if( (c & 0b00000011) == 0b00000010)
                    {
                        // Load received bytes to payload struct
                        payload[_byte_count] = c;
                        _byte_count++;

                        // When fully loaded, parse data to usable measures
                        if (_byte_count == _payload_length) {
                            Debug(2, "LIDAR MEASUREMENT CATCHED");
                            parse_response();
                            _byte_count = 0;
                        }
                    }

                    // Check for data packet start of new revolution
                    else if( (c & 0b00000011) == 0b00000001)
                    {
                        // Add revolution to count
                        _rev_cnt++;

                        // Load received bytes to payload struct
                        payload[_byte_count] = c;
                        _byte_count++;

                        // When fully loaded, parse data to usable measures
                        if (_byte_count == _payload_length) {
                            Debug(2, "LIDAR MEASUREMENT CATCHED");
                            parse_response();
                            _byte_count = 0;
                        }
                    }
                }
            }
            break;
        
        default:
            break;
    }

}

void RPLidarS2::parse_response()
{
    // Parse angle from data packet
    float response_angle = static_cast<float>(payload.sensor_scan.angle_q6)/64.0;

    // Only save 200 points of the 360 degree scan
    float pointNumber = response_angle / 1.8;
    if( (pointNumber- std::roundf(response_angle / 1.8)) < 0.001 && (pointNumber- std::roundf(response_angle / 1.8)) > -0.001)
    {
        int pos = static_cast<int>(pointNumber);
        pointArray[pos].angle = response_angle;
        pointArray[pos].distance = static_cast<float>(payload.sensor_scan.distance_q2)/4.0;
    }

}
