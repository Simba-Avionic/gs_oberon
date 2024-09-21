#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>

namespace GSUART
{
    #define READ_WRITE_BUFF_SIZE 128
    
    #define RAMKA_META_SIZE     4
    #define RAMKA_START         0xCC    // 11001100   204(10) S: 172(10)
    #define RAMKA_STOP          0x33    // 00110011   51(10)  S: 19(10)
    #define RAMKA_SPECIAL       0xF0    // 11110000   240(10) S: 208(10)
    #define R_SPECIAL_DIF       0x20    // 00100000   32(10)


    typedef uint8_t         Byte;


    // can't use RAMKA_START, RAMKA_STOP, RAMKA_SPECIAL
    enum class MsgID : Byte
    {
        TENSO = 0x01,
        TEMPERATURE = 0x02,
        ZAWORY_STEROWANIE = 0x03,
        ZAWORY_POZYCJA = 0x04,
        PRESSURE = 0x05
    };


    class Message
    {
    public:
        MsgID getID() const;
        bool isValid() const;
    protected:
        MsgID id;
        Byte checksum;
        bool valid = false;
        
        virtual void serialize(const Byte* bytes_out, const size_t* size_out) const = 0;
        virtual void deserialize(const Byte* bytes_in, const size_t size_in) = 0;
        friend class Messenger;
    };


    struct UARTStatistics
    {
        unsigned long long totalMessagesSent = 0;
        unsigned long long totalMessagesReceived = 0;
        unsigned long long goodMessagesReceived = 0;
        unsigned long long totalBytesReceived = 0;
        unsigned long long totalBytesSent = 0;
        unsigned int goodMessagesReceivedPerSec = 0;
        unsigned int messagesRecPerSec = 0;
        unsigned int messagesSentPerSec = 0;
        unsigned int bytesRecPerSec = 0;
        unsigned int bytesSentPerSec = 0;
        float goodMessagesReceivedPerSecRatio = 0.0;
    private:
        unsigned long long lastSecTotalMessagesSent = 0;
        unsigned long long lastSecTotalMessagesReceived = 0;
        unsigned long long lastSecGoodMessagesReceived = 0;
        unsigned long long lastSecTotalBytesReceived = 0;
        unsigned long long lastSecTotalBytesSent = 0;

        std::chrono::high_resolution_clock::time_point lastCalcTime = std::chrono::high_resolution_clock::now();
        void calculatePerSecValues();
        friend class Messenger;
    };


    class Messenger
    {
    public:
        Messenger(std::string uart_device);
        // ARDUINO VERSION
        // Messenger(Serial& serial);
        ~Messenger();

        void send(const Message& msg);
        Message* receive(bool blocking = true);

        const UARTStatistics& getStats() const;
    private:
        std::string serialPort;
        int serialPortFD;

        Byte receive_buff[READ_WRITE_BUFF_SIZE];
        size_t read_buff_idx = 0;
        Message* receivedMsg = nullptr;

        UARTStatistics uartStats;

        void decodeMsg(Byte* msg_bytes, const size_t size);
        void openSerialPort();
        void writeToSerialPort(const Byte* bytes, const size_t size);
        ssize_t readFromSerialPort(Byte* bytes, const size_t size);
    };


    class MsgTenso : public Message
    {
    public:
        int tenso_left_raw = 0;
        int tenso_right_raw = 0;
    private:
        void serialize(const Byte* bytes_out, const size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgTemperature : public Message
    {
    public:
        float temperature_celsius = 0.0;
    private:
        void serialize(const Byte* bytes_out, const size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgZaworySterowanie : public Message
    {
    public:
        signed char valve_vent = 0;
        signed char valve_feed = 0;
    private:
        void serialize(const Byte* bytes_out, const size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgZaworyPozycja : public Message
    {
    public:
        signed char valve_vent = 0;
        signed char valve_feed = 0;
    private:
        void serialize(const Byte* bytes_out, const size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgPressure : public Message
    {
    public:
        float pressure_bar = 0.0;
    private:
        void serialize(const Byte* bytes_out, const size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };


    void putByteIntoFrame(Byte byte, Byte* bytes, size_t& idx);
}