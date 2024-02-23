
// del_gimbal.h

#ifndef DEL_GIMBAL_H
#define DEL_GIMBAL_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define PITCH_CH                SRV_Channel::k_mount_tilt // CH10
#define YAW_CH                  SRV_Channel::k_mount_pan  // CH15
#define ZOOM_CH                 SRV_Channel::k_mount_roll // CH11
#define CENTER_CH               SRV_Channel::k_mount_open // CH12
#define FOCUS_CH                SRV_Channel::k_cam_focus  // CH13
#define RECORD_CH               SRV_Channel::k_cam_trigger// CH14

#define PITCH_INITIAL_VALUE     1500
#define YAW_INITIAL_VALUE       1500
#define ZOOM_INITIAL_VALUE      1500
#define CENTER_INITIAL_VALUE    1500
#define FOCUS_INITIAL_VALUE     1500
#define RECORD_INITIAL_VALUE    1500

#define PITCH_UP_MASK           0x0008
#define PITCH_DOWN_MASK         0x0001
#define YAW_LEFT_MASK           0x0004
#define YAW_RIGHT_MASK          0x0002
#define ZOOM_IN_MASK            0x0400
#define ZOOM_OUT_MASK           0x0200
#define CENTER_MASK             0x0010
#define FOCUS_MASK              0x0040
#define RECORD_MASK             0x0020

#define PITCH_UP                1700
#define PITCH_DOWN              1300

#define YAW_LEFT                1300
#define YAW_RIGHT               1700

#define ZOOM_SLEW               5

#define MIN_PPM                 1000
#define MID_PPM                 1500
#define MAX_PPM                 2000

/***************************************************************************
	Enumerations :
***************************************************************************/



/***************************************************************************
	Class :
***************************************************************************/

class DelGimbal
{
public:
    DelGimbal();
    void init();
    void manage(uint16_t receivedButtons);

private:
    uint16_t pitch_command;
    uint16_t yaw_command;
    uint16_t zoom_command;
    uint16_t center_command;
    uint16_t focus_command;
    uint16_t record_command;
};

#endif