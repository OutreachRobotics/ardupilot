// del_ekf.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_led.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

DEL_LED::DEL_LED()
{
    
}

void DEL_LED::init()
{
    _dev = hal.i2c_mgr->get_device(I2C_BUS, I2C_ADDRESS);
    if (!_dev) {
        return;
    }
    if(!_dev->get_semaphore()->take(10)){
        return;
    }
    _dev->set_retries(10);
    _dev->get_semaphore()->give();

    for(uint8_t i=0;i<LED_COUNT;i++)
    {
        ledPower[i] = 0;
    }
}

void DEL_LED::setLedPower(uint8_t* ledCommand)
{
    for(uint8_t i=0;i<7;i++)
    {
        ledPower[i] = ledCommand[i];
    }
}

void DEL_LED::manage()
{
    if(!_dev->get_semaphore()->take(2) || !_dev)
    {
        return;
    }
    _dev->transfer(ledPower,LED_COUNT,nullptr,0);
}