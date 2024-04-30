// del_winch.cpp

/***************************************************************************
    Include headers :
***************************************************************************/

#include "del_winch.h"

/***************************************************************************
	Global variables declaration :
***************************************************************************/

extern const AP_HAL::HAL& hal;

/***************************************************************************
	Function definition :
***************************************************************************/

DelWinch::DelWinch()
{
    _winch_port = hal.serial(WINCH_UART);
    
    position_read.value = 0;
    speed_read = 0;
    direction_read = Neutral;
    speed = 0;
    direction = Neutral;
    command = 0;
    error = 0;
    last_command = 0;
    command_ctr = 0;

    addOnType = None;
    graspingState = 10;
    rubbingState = 0;
    waterReady = 0;
    waterState = 0;
    waterQty.value = 123;
    waterRate = 0;
    waterTime.value = 126;

    tx_buffer[0] = WINCH_HEADER;
    tx_buffer[1] = direction;
    tx_buffer[2] = speed;
    tx_buffer[3] = command;
    tx_buffer[4] = WINCH_FOOTER;
}

void DelWinch::init()
{
    _winch_port->begin(57600,50,50);
}

void DelWinch::manage(uint16_t buttons)
{
    if(!gcs_failsafe)
    {
        if(buttons & WINCH_UP_MASK)
        {
            direction = Up;
            speed = WINCH_MAX_SPEED;
        }
        else if(buttons & WINCH_DOWN_MASK)
        {
            direction = Down;
            speed = WINCH_MAX_SPEED;
        }
        else
        {
            direction = Neutral;
            speed = 0;
        }

        if(buttons & COMMAND_MASK)
        {
            command_ctr++;
            if(command_ctr == COMMMAND_COUNT)
            {
               command = !command;
            }
        }
        else
        {
            command_ctr = 0;
        }
    }
    else
    {
        direction = Neutral;
        speed = 0;
        command = 0;
    }

    // Sending commands to the winch
    tx_buffer[1] = (uint8_t)direction;
    tx_buffer[2] = speed;
    tx_buffer[3] = command;
    _winch_port->write(tx_buffer,TX_BUFFER_LEN);   
}

void DelWinch::receiveSerial()
{
    // Receiving status from the winch
    while(_winch_port->available()>RX_BUFFER_LEN-1)
    {
        if(_winch_port->read() == WINCH_HEADER)
        {
            position_read.byte[1] = _winch_port->read();
            position_read.byte[0] = _winch_port->read(); 
            speed_read = _winch_port->read();
            direction_read = _winch_port->read();
            current = _winch_port->read();
            error = _winch_port->read();

            addOnType = AddOnType(_winch_port->read());
            graspingState = _winch_port->read();
            rubbingState = _winch_port->read();
            waterReady = _winch_port->read();
            waterState = _winch_port->read();
            waterQty.byte[1] = _winch_port->read();
            waterQty.byte[0] = _winch_port->read();
            waterRate = _winch_port->read();
            waterTime.byte[1] = _winch_port->read();
            waterTime.byte[0] = _winch_port->read();

            _winch_port->read();
        }
    }    
}

void DelWinch::setFailsafe(bool setter)
{
    gcs_failsafe = setter;
}

int16_t DelWinch::getPosition()
{
    return position_read.value;
}

uint8_t DelWinch::getSpeed()
{
    return speed_read;
}

uint8_t DelWinch::getSpeedCommand()
{
    return speed;
}

uint8_t DelWinch::getDirection()
{
    return direction_read;
}

uint8_t DelWinch::getCurrent()
{
    return current;
}

uint8_t DelWinch::getError()
{
    return error;
}

DataQGC DelWinch::getDataQGC()
{
    dataQGC.position = position_read.value;
    dataQGC.addOn = uint8_t(addOnType);
    dataQGC.addOnState = graspingState || waterState || rubbingState;
    dataQGC.waterQty = waterQty.value;
    dataQGC.waterTime = waterTime.value;

    return dataQGC;
}

void DelWinch::printStatus()
{
    hal.console->printf("\r\nWinch PCB Status\r\n");
    hal.console->printf("Position: %f m\r\n", float(position_read.value/100.0f));
    hal.console->printf("Speed: %f m/s\r\n", float(speed));
    hal.console->printf("Direction: %s\r\n",direction==Neutral ? "Neutral" : direction==Up ? "Up" : "Down");
    hal.console->printf("Add On Type: %s\r\n", addOnType==None ? "None" : addOnType==WaterSampling ? "Water" : addOnType==Grasping ? "Grasping" : "Rubbing");
    hal.console->printf("Grasping state: %s\r\n", graspingState?"On":"Off");
    hal.console->printf("Rubbing state: %s\r\n", rubbingState?"On":"Off");
    hal.console->printf("Water ready: %s\r\n", waterReady?"On":"Off");
    hal.console->printf("Water state: %s\r\n", waterState?"On":"Off");
    hal.console->printf("Water Quantity: %f L\r\n", float(waterQty.value/10.0f));
    hal.console->printf("Water Rate: %f\r\n", float(waterRate/10.0f));
    hal.console->printf("Water time: %d s\r\n\r\n", waterTime.value);
}