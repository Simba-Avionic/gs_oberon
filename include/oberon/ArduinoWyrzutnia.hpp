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

#define R_META_SIZE   4
#define RAMKA_START   0xCC    // 11001100   204(10) S: 172(10)
#define RAMKA_STOP    0x33    // 00110011   51(10)  S: 19(10)
#define RAMKA_SPECIAL 0xF0    // 11110000   240(10) S: 208(10)
#define R_SPECIAL_DIF 0x20    // 00100000   32(10)
#define RAMKA_TENSO   0x01

class ArduinoWyrzutnia
{
public:
    ArduinoWyrzutnia();
    ~ArduinoWyrzutnia();
private:
    int32_t offset_1_t1 = 809;
    int32_t offset_2_t1 = 0;
    int32_t offset_1_t2 = 597;
    int32_t offset_2_t2 = 0;
    std::vector<int32_t> srednia_100_t_1;
    std::vector<int32_t> srednia_100_t_2;
    unsigned int srednia_idx = 0;
    std::thread readT;
    void readingLoop();
    int serial_port;
    void openSerialPort();
    unsigned char read_buff[128];
    void decodeRamka(unsigned char* ramka, unsigned int size);
};