
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

#define LED_COUNT 8

/***************************************************************************
	Enumerations :
***************************************************************************/

union BatteryVoltage
{
    uint16_t voltage;
    struct 
    {
        uint8_t lsb;
        uint8_t msb;
    } bytes;
    
};

/***************************************************************************
	Class :
***************************************************************************/

class DelLed
{
public:
    DelLed();
    void init();
    void setLedPower(uint8_t* ledValue);
    int16_t getBatteryVoltage();
    void manage();


private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
    BatteryVoltage batteryVoltage;
    uint8_t ledPower[LED_COUNT];
};

#endif