#pragma once

#include <thread>
#include <functional>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

class PowerMonitor
{
public:
    struct Watomierz
    {
        
    };

    PowerMonitor(std::function<void()> newReadingCallback);
    ~PowerMonitor();

    const Watomierz& getBatteryWatomierz();

private:
    Watomierz batteryWatomierz;
    std::thread readT;
    void readingLoop();

    std::function<void()> newReadingCallback;
};