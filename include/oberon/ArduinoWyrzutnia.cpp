#include "ArduinoWyrzutnia.hpp"

ArduinoWyrzutnia::ArduinoWyrzutnia()
{
    srednia_100_t_1.reserve(100);
    srednia_100_t_2.reserve(100);
    for(int i=0; i<100; i++)
    {
        srednia_100_t_1.push_back(0);
        srednia_100_t_2.push_back(0);
    }
    openSerialPort();
    readT = std::thread(&ArduinoWyrzutnia::readingLoop, this);
}

ArduinoWyrzutnia::~ArduinoWyrzutnia()
{
    
}

void ArduinoWyrzutnia::openSerialPort()
{
    serial_port = open("/dev/ttyS0", O_RDWR);
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
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
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
}

void ArduinoWyrzutnia::readingLoop()
{
    unsigned char ramka_buff[128];
    unsigned int ramka_size = 0;       // obecna ilosc bajtow w buforze ramki
    while (true)
    {
        int n = read(serial_port, &read_buff, sizeof(read_buff));
        for (int i=0; i<n; i++)
        {
            unsigned char b = read_buff[i];
            ramka_buff[ramka_size++] = b;
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
    // remove special values
    unsigned char conv_ramka[128];
    unsigned int conv_idx = 0;
    for (unsigned int i=1; i<size-1; i++) // bez bajt start i stop
    {
        unsigned char b = ramka[i];
        if (b == RAMKA_SPECIAL)
        {
            i++;
            ramka[i] += R_SPECIAL_DIF;
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
        // printf("Checksum error\n");
        return;
    }

    unsigned char typ_ramki = conv_ramka[0];
    if (typ_ramki == RAMKA_TENSO)
    {
        int32_t tenso_1 = (conv_ramka[1] << 24) | (conv_ramka[2] << 16) | (conv_ramka[3] << 8) | conv_ramka[4];
        int32_t tenso_2 = (conv_ramka[5] << 24) | (conv_ramka[6] << 16) | (conv_ramka[7] << 8) | conv_ramka[8];
        srednia_100_t_1[srednia_idx] = tenso_1;
        srednia_100_t_2[srednia_idx] = tenso_2;
        srednia_idx++;
        srednia_idx %= 100;
        long long int s1 = 0, s2 = 0;
        for (int i=0; i<100; i++)
        {
            s1 += srednia_100_t_1[i];
            s2 += srednia_100_t_2[i];
        }
        s1 /= 100;
        s2 /= 100;
        printf("Tenso 1: %d  Tenso 2: %d  Srednia 1: %lld  Srednia 2: %lld  S1_kg: %f  S2_kg: %f\n", tenso_1, tenso_2, s1, s2, 1.078*(s1-offset_1_t1)/1000, 1.098*(s2-offset_1_t2)/1000);
    }
    // for (unsigned int i=0; i<conv_idx; i++)
    // {
    //     printf("%02X ", conv_buf[i]);
    // }
    // printf("\n");

}