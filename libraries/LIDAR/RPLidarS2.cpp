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
#include <vector>

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
    _sync_error = false;
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
        if(AP_HAL::millis() - _last_reset_ms >= 800) _state = idle;
        //Debug(1, "Waiting for reset");
        break;

    case idle:
        // Request scan mode
        startScanNormal();
        setMotorSpeed(DEFAULT_MOTOR_SPEED);
        _state = scanning;
        Debug(1, "Starting scan and motor");
        break;

    case scanning:

        run_algo();

        if ( (AP_HAL::millis() - _update_debug) > 300){
            _update_debug = AP_HAL::millis();
            hal.console->printf("\n");
            for(int i = 0 ; i < coord_lenght ; i+=10)
            {
                //Debug(1, "%f , %f", polarArray[i].angle, polarArray[i].distance);
                hal.console->printf("%f , %f\n", polarArray[i].angle, polarArray[i].distance);
                
            }
            hal.console->printf("Goal, %f , %f\n", center_goal.x, center_goal.y);
            hal.console->printf("\n");
            Debug(1, "Still running algo");
        }
        
        break;

    default:
        break;
    }

}

sl_result RPLidarS2::connect()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    int8_t uartNo = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_Lidar360, 0);
    uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0);
    
    Debug(1, "UART port : %d , Baud : %lu", uartNo, baud);
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

// Manage UART connection and reading, run at 200Hz
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
            {
                uint32_t nbytes = 0;
                // uint32_t sBytes = 0;
                nbytes = _uart->available();
                
                while (nbytes-- > 0) {
                    
                    uint8_t c = _uart->read();

                    if (_sync_error) {
                        // out of 5-byte sync mask -> catch new revolution
                        // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
                        if ((c & 0x03) == 0x01) 
                        {
                            _sync_error = false;
                        }
                        _break_count++;
                        // else
                        // {
                        //     _break_count++;
                        //     continue;
                        // }
                    }
                    else {

                        // Load received bytes to payload struct
                        payload[_byte_count] = c;
                        _byte_count++;

                        // When fully loaded, parse data to usable measures
                        if (_byte_count == _payload_length) {
                            parse_response();
                            _byte_count = 0;
                            _total_msg_count++;
                        }
                    }

                    
                }

                // uint32_t end_byte = nbytes;
                // if(AP_HAL::millis() - _update_debug2 >= 500)
                // {
                //     Debug(1, "-----bytes available : %lu", sBytes);
                //     Debug(1, "-----bytes remaining : %lu", ++end_byte);
                //     Debug(1, "-------bad msg count : %lu", _bad_msg_count);
                //     Debug(1, "------good msg count : %lu", _msg_count);
                //     Debug(1, "-----total msg count : %lu", _total_msg_count);
                //     Debug(1, "-----------Rev count : %u", _rev_cnt);
                //     Debug(1, "-----------Break count : %lu", _break_count);
                //     _bad_msg_count = 0;
                //     _msg_count = 0;
                //     _total_msg_count = 0;
                //     _rev_cnt = 0;
                //     _break_count = 0;
                //     _update_debug2 = AP_HAL::millis();
                // }
            }
            break;
        
        default:
            break;


    }



}

// Decode angle and distance from a data packet
void RPLidarS2::parse_response()
{

    // Check for valid payload
    if ((payload.sensor_scan.startbit == !payload.sensor_scan.not_startbit) && payload.sensor_scan.checkbit)
    {

        // Check for new turn
        if( payload.sensor_scan.startbit == 1 )
        {
            // Add revolution to count
            _rev_cnt++;
            _last_rev = AP_HAL::millis();


        }
 

        // Parse angle from data packet
        const float response_angle = payload.sensor_scan.angle_q6/64.0f;
        const float response_distance = payload.sensor_scan.distance_q2/4.0f;
        
        if(response_angle > 360.0) // Don't know why this happens
        {
            _angle_too_big++;
        }

        // Skip if the distance is 0. Don't know why this happens
        if(response_distance < 1.0) 
        {
            return;
        }

        // Only save 100 points of the 360 degree scan
        const float divider = 360.0/coord_lenght;
        const int sectionNumber = static_cast<int>(response_angle/divider);

        // Add to array if it is not already there
        if( frequencyArray[sectionNumber] != _rev_cnt)
        {
            polarArray[sectionNumber].angle = response_angle;
            polarArray[sectionNumber].distance = response_distance;
            frequencyArray[sectionNumber]++;
        } 

    }
    else
    {
        _sync_error = true;
        _bad_msg_count++;
    }

    

}

// Convert polar coordinates to Cartesian coordinates
cartesian_coord RPLidarS2::polar_to_cartesian(const polar_coord& polar) 
{
    cartesian_coord cartesian;
    cartesian.x = polar.distance * cosf(M_PI * polar.angle / 180.0);
    cartesian.y = polar.distance * sinf(M_PI * polar.angle / 180.0);
    return cartesian;
}

// Calculate the distance between two Cartesian points
float RPLidarS2::calculate_distance(const cartesian_coord& p1, const cartesian_coord& p2) 
{
    return safe_sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Calculates num_points points with the same linear distance along the perimeter 
void RPLidarS2::equidistant_points(struct polar_coord* polar_coords, size_t num_coord, struct cartesian_coord* points, size_t num_points) 
{
    cartesian_coord cartesian_coords[num_coord];
    
    // Convert polar coordinates to Cartesian coordinates
    for (int i = 0 ; i < num_coord ; i++) {
        cartesian_coords[i] = polar_to_cartesian(polar_coords[i]);
    }
    
    // Calculate total perimeter
    float total_perimeter = 0.0;
    float segment_distances[num_coord-1];
    for (size_t i = 0; i < num_coord - 1; ++i) {
        segment_distances[i] = calculate_distance(cartesian_coords[i], cartesian_coords[i + 1]);
        total_perimeter += segment_distances[i];
    }

    // Calculate equidistant points
    const float spacing = total_perimeter / static_cast<float>(num_points);
    int incrementor = 0;
    float dist = 0.0;

    for (size_t i = 0; i < num_coord - 1; ++i) {
        while (dist < segment_distances[i] && incrementor < num_points) {
            float ratio = dist / segment_distances[i];
            points[incrementor].x = cartesian_coords[i].x + ratio * (cartesian_coords[i + 1].x - cartesian_coords[i].x);
            points[incrementor].y = cartesian_coords[i].y + ratio * (cartesian_coords[i + 1].y - cartesian_coords[i].y);
            incrementor++;
            dist += spacing;
        }
        dist -= segment_distances[i];
    }

}

// Find the centroid of a xy points array
cartesian_coord RPLidarS2::find_centroid(struct cartesian_coord* points, size_t num_points) 
{
    cartesian_coord centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    float total_x = 0.0;
    float total_y = 0.0;

    for (int i = 0 ; i < num_points ; i++) {
        total_x += points[i].x;
        total_y += points[i].y;
    }

    centroid.x = total_x / static_cast<float>(num_points);
    centroid.y = total_y / static_cast<float>(num_points);

    return centroid;
}

// Find the center of the shape read by lidar
void RPLidarS2::run_algo()
{
    equidistant_points(polarArray, coord_lenght, pointsArray, points_lenght);
    center_goal = find_centroid(pointsArray, points_lenght);
}