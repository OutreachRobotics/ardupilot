
// del_led.h

#ifndef DEL_LED_H
#define DEL_LED_H

/***************************************************************************
    Include headers :
***************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

/***************************************************************************
    Macro :
***************************************************************************/

#define I2C_BUS 1
#define I2C_ADDRESS 0x14

#define LED_COUNT 6

/***************************************************************************
	Enumerations :
***************************************************************************/


/***************************************************************************
	Class :
***************************************************************************/

class DEL_LED
{
public:
    DEL_LED();
    void init();
    void setLedPower(uint8_t* ledValue);
    void manage();


private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
    uint8_t ledPower[LED_COUNT];
};

#endif