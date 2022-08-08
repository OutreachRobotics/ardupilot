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

DelLed::DelLed()
{
    
}

void DelLed::init()
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
    batteryVoltage.voltage = 0;
}

void DelLed::setLedPower(uint8_t* ledCommand)
{
    for(uint8_t i=0;i<7;i++)
    {
        ledPower[i] = ledCommand[i];
    }
}

int16_t DelLed::getBatteryVoltage()
{
    return int16_t(batteryVoltage.voltage*10);
}

void DelLed::manage()
{
    if(!_dev->get_semaphore()->take(2) || !_dev){
        return;
    }
    _dev->transfer(ledPower,LED_COUNT,nullptr,0);
    _dev->transfer(nullptr,0,&batteryVoltage.bytes.lsb,sizeof(batteryVoltage.voltage));
}
