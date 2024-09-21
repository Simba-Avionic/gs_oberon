#include "GSUART.hpp"

#include <unistd.h>
#include <termio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

namespace GSUART
{
    MsgID Message::getID() const
    {
        return id;
    }

    bool Message::isValid() const
    {
        return valid;
    }

    void UARTStatistics::calculatePerSecValues()
    {
        // TODO
    }

    // arduino is different here
    Messenger::Messenger(std::string uart_device)
        : serialPort(uart_device)
    {
        openSerialPort();
    }

    Messenger::~Messenger()
    {
        // TODO
    }

    void Messenger::send(const Message& msg)
    {
        Byte frame[READ_WRITE_BUFF_SIZE];
        frame[0] = RAMKA_START;
        frame[1] = (Byte)msg.getID();
        
        size_t idx = 2;
        Byte msgSerializedBytes[READ_WRITE_BUFF_SIZE];
        size_t msgSerializedSize;
        msg.serialize(msgSerializedBytes, &msgSerializedSize);
        for (size_t i = 0; i < msgSerializedSize; i++)
            putByteIntoFrame(msgSerializedBytes[i], frame, idx);
        
        Byte checksum = 0;
        for (size_t i = 1; i < idx; i++)
            checksum += frame[i];
        putByteIntoFrame(checksum, frame, idx);
        frame[idx++] = RAMKA_STOP;

        writeToSerialPort(frame, idx);
    }
    
    // arduino is different here, bo serial.read
    Message* Messenger::receive(bool blocking)
    {
        Byte read_buff[READ_WRITE_BUFF_SIZE];
        int n = readFromSerialPort(read_buff, READ_WRITE_BUFF_SIZE);
        if (n < 0)
        {
            printf("Error %i from read: %s\n", errno, strerror(errno));
            return nullptr;
        }
        uartStats.totalBytesReceived += n;
        for (int i=0; i<n; i++)
        {
            Byte b = read_buff[i];
            receive_buff[read_buff_idx++] = b;

            if (read_buff_idx >= READ_WRITE_BUFF_SIZE)    // zabezpieczenie przed overflowem
            {
                read_buff_idx = 0;
                continue;
            }

            if (b == RAMKA_START)
            {
                read_buff_idx = 0;
                receive_buff[read_buff_idx++] = b;
            }
            else if (b == RAMKA_STOP)
            {
                decodeMsg(receive_buff, read_buff_idx);
            }       
        }
        return receivedMsg;
    }

    const UARTStatistics& Messenger::getStats() const
    {
        return uartStats;
    }

    void Messenger::decodeMsg(Byte* msg_bytes, const size_t size)
    {
        if (size < 3)
            return;
        uartStats.totalMessagesReceived++;
        // remove special values
        Byte conv_msg[READ_WRITE_BUFF_SIZE];
        size_t conv_idx = 0;
        for (size_t i=1; i<size-1; i++) // bez bajt start i stop
        {
            Byte b = msg_bytes[i];
            if (b == RAMKA_SPECIAL)
            {
                i++;
                msg_bytes[i] += R_SPECIAL_DIF;
                conv_msg[conv_idx++] = msg_bytes[i];
            }
            else
                conv_msg[conv_idx++] = b;
        }

        // sprawdzenie checksumy
        Byte checksum = 0;
        for (size_t i = 0; i < conv_idx - 1; i++) // exclude last byte (checksum)
        {
            checksum += conv_msg[i];
        }

        if (receivedMsg)
            delete receivedMsg;
        receivedMsg = nullptr;

        if (checksum != conv_msg[conv_idx - 1])
        {
            return;
        }      
        uartStats.goodMessagesReceived++;

        MsgID msgID = (MsgID)conv_msg[0];
        switch (msgID)
        {
            case MsgID::TENSO:
                receivedMsg = new MsgTenso();
                break;
            case MsgID::TEMPERATURE:
                receivedMsg = new MsgTemperature();
                break;
            case MsgID::ZAWORY_STEROWANIE:
                receivedMsg = new MsgZaworySterowanie();
                break;
            case MsgID::ZAWORY_POZYCJA:
                receivedMsg = new MsgZaworyPozycja();
                break;
            case MsgID::PRESSURE:
                receivedMsg = new MsgPressure();
                break;
        }
        receivedMsg->deserialize(conv_msg+1, conv_idx-2);
    }

    // arduino is different here
    void Messenger::openSerialPort()
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

    // arduino is different here
    void Messenger::writeToSerialPort(const Byte* bytes, const size_t size)
    {
        // TODO
    }

    ssize_t Messenger::readFromSerialPort(Byte* bytes, const size_t size)
    {
        // TODO
    }


    void MsgTenso::serialize(const Byte* bytes_out, const size_t* size_out) const
    {
        // TODO
    }

    void MsgTenso::deserialize(const Byte* bytes_in, const size_t size_in)
    {
        // TODO
    }

    void MsgTemperature::serialize(const Byte* bytes_out, const size_t* size_out) const
    {
        // TODO
    }

    void MsgTemperature::deserialize(const Byte* bytes_in, const size_t size_in)
    {
        // TODO
    }

    void MsgZaworySterowanie::serialize(const Byte* bytes_out, const size_t* size_out) const
    {
        // TODO
    }

    void MsgZaworySterowanie::deserialize(const Byte* bytes_in, const size_t size_in)
    {
        // TODO
    }

    void MsgZaworyPozycja::serialize(const Byte* bytes_out, const size_t* size_out) const
    {
        // TODO
    }

    void MsgZaworyPozycja::deserialize(const Byte* bytes_in, const size_t size_in)
    {
        // TODO
    }

    void MsgPressure::serialize(const Byte* bytes_out, const size_t* size_out) const
    {
        // TODO
    }

    void MsgPressure::deserialize(const Byte* bytes_in, const size_t size_in)
    {
        // TODO
    }

    void putByteIntoFrame(Byte byte, Byte* bytes, size_t& idx)
    {
        if (byte == RAMKA_STOP || byte == RAMKA_START || byte == RAMKA_SPECIAL) {
            bytes[idx++] = RAMKA_SPECIAL;
            bytes[idx++] = byte - R_SPECIAL_DIF;
        } else bytes[idx++] = byte;
    }
}