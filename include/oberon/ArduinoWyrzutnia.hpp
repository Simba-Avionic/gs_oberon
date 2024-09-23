#pragma once

#include <unistd.h>
#include <termio.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <thread>
#include <vector>
#include <functional>

#include "GSUART.hpp"

#define RAMKA_META_SIZE     4
#define RAMKA_START         0xCC    // 11001100   204(10) S: 172(10)
#define RAMKA_STOP          0x33    // 00110011   51(10)  S: 19(10)
#define RAMKA_SPECIAL       0xF0    // 11110000   240(10) S: 208(10)
#define RAMKA_SPECIAL_DIF   0x20    // 00100000   32(10)
#define RAMKA_TENSO         0x01
#define RAMKA_TEMPERATURE   0x02

class ArduinoWyrzutnia
{
public:
    struct tenso
    {
        int32_t last_values[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // 30 ostatnich wartosci do sredniej dla tarowania
        int32_t last_values_idx = 0;
        int32_t raw_value = 0;                  // bez tarowania
        int32_t rocket_point = 0;               // tarujemy i kladziemy rakiete
        int32_t empty_rocket_point = 0;         // tarujemy i tankujemy paliwo
        double scale = 1.0;
        float raw_kg = 0.0;
        float rocket_kg = 0.0;
        float fuel_kg = 0.0;

        int32_t getAvgValue30()
        {
            int32_t sum = 0;
            for (int i=0; i<30; i++)
                sum += last_values[i];
            return sum / 30;
        }
    };

    ArduinoWyrzutnia(std::function<void()> newTensoCallback, std::function<void()> newSensorsCallback, std::string serialPort);
    ~ArduinoWyrzutnia();

    tenso& getTensoL();
    tenso& getTensoR();
    const float& getLeanAngle();
    const float& getTemperature();

    const GSUART::UARTStatistics& getUartStats();

    void tareRocketPoint();
    void tareEmptyRocketPoint();
    void setScaleLeft(double scale);
    void setScaleRight(double scale);
    void setLeanAngle(float angle);

private:
    tenso tensoL, tensoR;

    struct Lean {
        float angle = 0.0;
        float cosinus = 1.0;
    } lean;

    float temperature = 0.0;

    GSUART::UARTStatistics::Stats remoteUARTStats;

    GSUART::Messenger messenger;
    std::thread readT;
    void readingLoop();

    std::function<void()> newTensoCallback;
    std::function<void()> newTemperatureCallback;
    std::function<void()> newUARTStatsCallback;
};