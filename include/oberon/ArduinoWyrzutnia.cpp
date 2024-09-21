#include "ArduinoWyrzutnia.hpp"

#include <cstring>
#include <cmath>

ArduinoWyrzutnia::ArduinoWyrzutnia(std::function<void()> newTensoCallback, std::function<void()> newSensorsCallback, std::string serialPort)
    : serialPort(serialPort), newTensoCallback(newTensoCallback), newSensorsCallback(newSensorsCallback)
{
    openSerialPort();
    readT = std::thread(&ArduinoWyrzutnia::readingLoop, this);
}

ArduinoWyrzutnia::~ArduinoWyrzutnia()
{
    
}

void ArduinoWyrzutnia::openSerialPort()
{
    serialPortFD = open(serialPort.c_str(), O_RDWR);
    struct termios tty;
    if (tcgetattr(serialPortFD, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(1);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serialPortFD, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
    printf("Serial port opened: %s\n", serialPort.c_str());
}

void ArduinoWyrzutnia::readingLoop()
{
    #define BUFF_SIZE 512
    unsigned char ramka_buff[BUFF_SIZE];
    unsigned int ramka_size = 0;       // obecna ilosc bajtow w buforze ramki
    while (true)
    {
        int n = read(serialPortFD, &read_buff, sizeof(read_buff));
        // printf("n: %d\n", n);
        if (n < 0)
        {
            printf("Error %i from read: %s\n", errno, strerror(errno));
            exit(1);
        }
        uartStats.totalBytesReceived += n;
        for (int i=0; i<n; i++)
        {
            unsigned char b = read_buff[i];
            ramka_buff[ramka_size++] = b;

            if (ramka_size >= BUFF_SIZE)    // zabezpieczenie przed overflowem
            {
                ramka_size = 0;
                continue;
            }

            if (b == 0xcc)
            {
                ramka_size = 0;
                ramka_buff[ramka_size++] = b;
            }
            else if (b == 0x33)
                decodeRamka(ramka_buff, ramka_size);
        }
    }
}

void ArduinoWyrzutnia::decodeRamka(unsigned char* ramka, unsigned int size)
{
    if (size < 3)
        return;
    uartStats.totalMessagesReceived++;
    // remove special values
    unsigned char conv_ramka[128];
    unsigned int conv_idx = 0;
    for (unsigned int i=1; i<size-1; i++) // bez bajt start i stop
    {
        unsigned char b = ramka[i];
        if (b == RAMKA_SPECIAL)
        {
            i++;
            ramka[i] += RAMKA_SPECIAL_DIF;
            conv_ramka[conv_idx++] = ramka[i];
        }
        else
            conv_ramka[conv_idx++] = b;
    }

    // sprawdzenie checksumy
    unsigned char checksum = 0;
    for (unsigned int i = 0; i < conv_idx - 1; i++) // exclude last byte (checksum)
    {
        checksum += conv_ramka[i];
    }

    if (checksum != conv_ramka[conv_idx - 1])
    {
        return;
    }
    uartStats.goodMessagesReceived++;

    unsigned char typ_ramki = conv_ramka[0];
    if (typ_ramki == RAMKA_TENSO)
    {
        int32_t tenso_1 = (conv_ramka[1] << 24) | (conv_ramka[2] << 16) | (conv_ramka[3] << 8) | conv_ramka[4];
        int32_t tenso_2 = (conv_ramka[5] << 24) | (conv_ramka[6] << 16) | (conv_ramka[7] << 8) | conv_ramka[8];
        tensoL.raw_value = tenso_1;
        tensoL.last_values[(tensoL.last_values_idx++) % 30] = tenso_1; 
        tensoR.raw_value = tenso_2;
        tensoR.last_values[(tensoR.last_values_idx++) % 30] = tenso_2;

        tensoL.raw_kg = tensoL.raw_value * tensoL.scale / 1000.0;
        tensoL.rocket_kg = (tensoL.raw_value - tensoL.rocket_point) * tensoL.scale / 1000.0;
        tensoL.fuel_kg = (tensoL.raw_value - tensoL.empty_rocket_point) * tensoL.scale / 1000.0;
        
        tensoR.raw_kg = tensoR.raw_value * tensoR.scale / 1000.0;
        tensoR.rocket_kg = (tensoR.raw_value - tensoR.rocket_point) * tensoR.scale / 1000.0;
        tensoR.fuel_kg = (tensoR.raw_value - tensoR.empty_rocket_point) * tensoR.scale / 1000.0;

        // uwzglednienie kata nachylenia
        tensoL.raw_kg /= lean.cosinus;      
        tensoL.rocket_kg /= lean.cosinus;
        tensoL.fuel_kg /= lean.cosinus;
        tensoR.raw_kg /= lean.cosinus;
        tensoR.rocket_kg /= lean.cosinus;
        tensoR.fuel_kg /= lean.cosinus;
        
        if (newTensoCallback)
            newTensoCallback();
    }
    else if (typ_ramki == RAMKA_TEMPERATURE)
    {
        uint8_t temperature_buff[4] = {conv_ramka[1], conv_ramka[2], conv_ramka[3], conv_ramka[4]};
        std::memcpy(&temperature, temperature_buff, 4);
        if (newSensorsCallback)
            newSensorsCallback();
    }
}

ArduinoWyrzutnia::tenso& ArduinoWyrzutnia::getTensoL()
{
    return tensoL;
}

ArduinoWyrzutnia::tenso& ArduinoWyrzutnia::getTensoR()
{
    return tensoR;
}

const float& ArduinoWyrzutnia::getLeanAngle()
{
    return lean.angle;
}

const float& ArduinoWyrzutnia::getTemperature()
{
    return temperature;
}

const ArduinoWyrzutnia::uartStatistics& ArduinoWyrzutnia::getUartStats()
{
    return uartStats;
}

void ArduinoWyrzutnia::secondPassedUpdateStats()
{
    uartStats.secondPassed();
}

void ArduinoWyrzutnia::tareRocketPoint()
{
    tensoL.rocket_point = tensoL.getAvgValue30();
    tensoR.rocket_point = tensoR.getAvgValue30();
}

void ArduinoWyrzutnia::tareEmptyRocketPoint()
{
    tensoL.empty_rocket_point = tensoL.getAvgValue30();
    tensoR.empty_rocket_point = tensoR.getAvgValue30();
}

void ArduinoWyrzutnia::setScaleLeft(double scale)
{
    tensoL.scale = scale;
}

void ArduinoWyrzutnia::setScaleRight(double scale)
{
    tensoR.scale = scale;
}

void ArduinoWyrzutnia::setLeanAngle(float angle)
{
    lean.angle = angle;
    lean.cosinus = cos(angle * 3.14159265359 / 180.0);
}