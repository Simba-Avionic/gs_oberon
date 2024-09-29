#include "ArduinoZawory.hpp"

ArduinoZawory::ArduinoZawory(std::string serialPort, std::function<void()> newZaworyPosCallback,
                                                     std::function<void()> newTemperatureCallback,
                                                     std::function<void()> newPressureCallback,
                                                     std::function<void()> newUARTStatsCallback)
    :   messenger(serialPort),
        newZaworyPosCallback(newZaworyPosCallback),
        newTemperatureCallback(newTemperatureCallback),
        newPressureCallback(newPressureCallback),
        newUARTStatsCallback(newUARTStatsCallback)
{
    readT = std::thread(&ArduinoZawory::readingLoop, this);
}

ArduinoZawory::~ArduinoZawory()
{
    
}

void ArduinoZawory::readingLoop()
{
    while (true)
    {
        GSUART::Message* msg = nullptr;
        while (!msg)
        {
            msg = messenger.receive();
        }
        
        switch (msg->getID())
        {
            case GSUART::MsgID::ZAWORY_POZYCJA:
            {
                GSUART::MsgZaworyPozycja* msgZaworyPos = dynamic_cast<GSUART::MsgZaworyPozycja*>(msg);
                zaworyPos.feed_percent = msgZaworyPos->valve_feed;
                zaworyPos.vent_percent = msgZaworyPos->valve_vent;
                if (newZaworyPosCallback)
                    newZaworyPosCallback();
                break;
            }
            case GSUART::MsgID::TEMPERATURE:
            {
                GSUART::MsgTemperature* msgTemperature = dynamic_cast<GSUART::MsgTemperature*>(msg);
                temperature = msgTemperature->temperature_celsius;
                if (newTemperatureCallback)
                    newTemperatureCallback();
                break;
            }
            case GSUART::MsgID::PRESSURE:
            {
                GSUART::MsgPressure* msgPressure = dynamic_cast<GSUART::MsgPressure*>(msg);
                pressure = msgPressure->pressure_bar;
                if (newPressureCallback)
                    newPressureCallback();
                break;
            }
            case GSUART::MsgID::UART_STATS:
            {
                GSUART::MsgUartStats* msgUARTStats = dynamic_cast<GSUART::MsgUartStats*>(msg);
                remoteUartStats = msgUARTStats->stats;
                if (newUARTStatsCallback)
                    newUARTStatsCallback();
                break;
            }
            default:
                printf("Unknown message type\n");
                break;
        }
    }
}

void ArduinoZawory::openZawory(int8_t feed_percent, int8_t vent_percent)
{
    GSUART::MsgZaworySterowanie msg;
    msg.valve_feed = feed_percent;
    msg.valve_vent = vent_percent;
    messenger.send(msg);
}

const ArduinoZawory::ZaworyPos& ArduinoZawory::getZaworyPos()
{
    return zaworyPos;
}

const float& ArduinoZawory::getTemperature()
{
    return temperature;
}

const double& ArduinoZawory::getPressure()
{
    return pressure;
}

const GSUART::UARTStatistics::Stats& ArduinoZawory::getUartStats()
{
    return messenger.getStats().stats;
}

const GSUART::UARTStatistics::Stats& ArduinoZawory::getRemoteUartStats()
{
    return remoteUartStats;
}