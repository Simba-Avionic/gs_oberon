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

#define RAMKA_META_SIZE     4
#define RAMKA_START         0xCC    // 11001100   204(10) S: 172(10)
#define RAMKA_STOP          0x33    // 00110011   51(10)  S: 19(10)
#define RAMKA_SPECIAL       0xF0    // 11110000   240(10) S: 208(10)
#define RAMKA_SPECIAL_DIF   0x20    // 00100000   32(10)
#define RAMKA_TENSO         0x01

class ArduinoWyrzutnia
{
public:
    struct tenso
    {
        int32_t raw_value = 0;
        int32_t rocket_point = 0;
        int32_t empty_rocket_point = 0;
        double scale = 1.0;
        float raw_kg = 0.0;
        float rocket_kg = 0.0;
        float fuel_kg = 0.0;
    };
    struct uartStatistics
    {
        unsigned long long totalMessagesSent = 0;
        unsigned long long totalMessagesReceived = 0;
        unsigned long long goodMessagesReceived = 0;
        unsigned long long totalBytesReceived = 0;
        unsigned long long totalBytesSent = 0;
        unsigned int goodMessagesReceivedLastSec = 0;
        unsigned int messagesRecLastSec = 0;
        unsigned int messagesSentLastSec = 0;
        unsigned int bytesRecLastSec = 0;
        unsigned int bytesSentLastSec = 0;
        float goodMessagesReceivedPerSecRatio = 0.0;
    private:
        unsigned long long lastSecTotalMessagesSent = 0;
        unsigned long long lastSecTotalMessagesReceived = 0;
        unsigned long long lastSecGoodMessagesReceived = 0;
        unsigned long long lastSecTotalBytesReceived = 0;
        unsigned long long lastSecTotalBytesSent = 0;
        void secondPassed()
        {
            goodMessagesReceivedLastSec = goodMessagesReceived - lastSecGoodMessagesReceived;
            messagesRecLastSec = totalMessagesReceived - lastSecTotalMessagesReceived;
            messagesSentLastSec = totalMessagesSent - lastSecTotalMessagesSent;
            bytesRecLastSec = totalBytesReceived - lastSecTotalBytesReceived;
            bytesSentLastSec = totalBytesSent - lastSecTotalBytesSent;
            lastSecTotalMessagesSent = totalMessagesSent;
            lastSecTotalMessagesReceived = totalMessagesReceived;
            lastSecGoodMessagesReceived = goodMessagesReceived;
            lastSecTotalBytesReceived = totalBytesReceived;
            lastSecTotalBytesSent = totalBytesSent;
            goodMessagesReceivedPerSecRatio = (float)goodMessagesReceivedLastSec / (float)messagesRecLastSec;
        }
        friend class ArduinoWyrzutnia;
    };

    ArduinoWyrzutnia(std::function<void()> newTensoCallback, std::function<void()> newSensorsCallback, std::string serialPort);
    ~ArduinoWyrzutnia();

    const tenso& getTensoL();
    const tenso& getTensoR();
    const uartStatistics& getUartStats();

    void secondPassedUpdateStats();
private:
    tenso tensoL, tensoR;
    std::thread readT;
    void readingLoop();
    std::string serialPort;
    int serialPortFD;
    void openSerialPort();
    unsigned char read_buff[256];
    void decodeRamka(unsigned char* ramka, unsigned int size);

    std::function<void()> newTensoCallback;
    std::function<void()> newSensorsCallback;

    uartStatistics uartStats;
};