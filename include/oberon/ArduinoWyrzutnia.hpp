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
    ArduinoWyrzutnia(std::string serialPort = "/dev/ttyS0");
    ~ArduinoWyrzutnia();
private:
    struct tenso
    {
        int32_t raw_value = 0;
        int32_t zero_point = 0;
        int32_t empty_rocket_point = 0;
    } tensoL, tensoR;
    std::thread readT;
    void readingLoop();
    std::string serialPort;
    int serialPortFD;
    void openSerialPort();
    unsigned char read_buff[256];
    void decodeRamka(unsigned char* ramka, unsigned int size);

    std::function<void()> newTensoCallback;
    std::function<void()> newSensorsCallback;
};