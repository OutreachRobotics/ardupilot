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


#pragma once

#include <AP_HAL/AP_HAL.h>                   ///< for UARTDriver

typedef uint32_t sl_result;

#define SL_RESULT_OK                     (sl_result)0
#define SL_RESULT_FAIL_BIT               (sl_result)0x80000000

struct polar_coord
{
    float angle;
    float distance;
};

struct cartesian_coord 
{
    float x;
    float y;
};

class RPLidarS2 
{

public:
    RPLidarS2();

    // update the _state of the sensor, run at 10Hz
    void update(void);

    // Manage UART connection and reading, run at 50Hz
    void get_readings();

    // Output for localisation
    cartesian_coord center_goal;

protected:

    AP_HAL::UARTDriver *_uart;              // uart for communicating with sensor

private:
    enum _state {
            initial = 0,
            connected,
            resetting,
            idle,
            scanning
        }_state;


    // Types
    //-----------------------------------------

    struct sl_lidar_ans_header_t
    {
        uint8_t  syncByte1; // must be SL_LIDAR_ANS_SYNC_BYTE1
        uint8_t  syncByte2; // must be SL_LIDAR_ANS_SYNC_BYTE2
        uint32_t size_q30_subtype; // see _u32 size:30; _u32 subType:2;
        uint8_t  type;
    };

    struct PACKED _sensor_scan {
        uint8_t startbit      : 1;            ///< on the first revolution 1 else 0
        uint8_t not_startbit  : 1;            ///< complementary to startbit
        uint8_t quality       : 6;            ///< Related the reflected laser pulse strength
        uint8_t checkbit      : 1;            ///< always set to 1
        uint16_t angle_q6    : 15;           ///< Actual heading = angle_q6/64.0 Degree
        uint32_t distance_q2  : 16;           ///< Actual Distance = distance_q2/4.0 mm
    };

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        _sensor_scan sensor_scan;
    } payload;


    
    // Methods
    //-----------------------------------------

    // Request
    sl_result startScanNormal();
    sl_result setMotorSpeed(uint16_t speed );
    sl_result stop();
    sl_result reset();

    // Message handling
    sl_result connect();
    sl_result _sendCommand(uint8_t cmd, const void * cmdPayload, size_t payloadsize);
    void parse_response();

    // Algorithm
    float calculate_distance(const cartesian_coord& p1, const cartesian_coord& p2);
    cartesian_coord polar_to_cartesian(const polar_coord& polar);
    void equidistant_points(struct polar_coord* polar_coords, size_t num_coord, struct cartesian_coord* points, size_t num_points);
    cartesian_coord find_centroid(struct cartesian_coord* points, size_t num_points);
    void run_algo();



    // Attributes
    //-----------------------------------------

    bool _isConnected;

    // reply related variables
    uint8_t _payload_length = 5;
    uint8_t _rev_cnt;
    uint16_t _byte_count;

    uint8_t _next_read_flush;

    // request related variables
    uint32_t  _last_reset_ms;
    uint32_t   _update_debug;
    uint32_t   _update_debug2;

    // List of selected points to use for localisation
    
    static const size_t coord_lenght = 200;
    polar_coord polarArray[coord_lenght];

    static const size_t points_lenght = 100;
    cartesian_coord pointsArray[points_lenght];

};
