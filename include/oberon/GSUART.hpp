#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>

namespace GSUART
{
    #define WRITE_BUFF_SIZE     256
    #define RECEIVE_BUFF_SIZE   256
    #define READ_BUFF_SIZE      64
    
    #define RAMKA_META_SIZE     4
    #define RAMKA_START         0xCC    // 11001100   204(10) S: 172(10)
    #define RAMKA_STOP          0x33    // 00110011   51(10)  S: 19(10)
    #define RAMKA_SPECIAL       0xF0    // 11110000   240(10) S: 208(10)
    #define R_SPECIAL_DIF       0x20    // 00100000   32(10)


    typedef uint8_t         Byte;


    void putByteIntoFrame(Byte byte, Byte* bytes, size_t& idx);
    uint64_t getCurrentTimeMillis();


    // DON'T USE RAMKA_START, RAMKA_STOP, RAMKA_SPECIAL
    enum class MsgID : Byte
    {
        TENSO = 0x01,
        TEMPERATURE = 0x02,
        ZAWORY_STEROWANIE = 0x03,
        ZAWORY_POZYCJA = 0x04,
        PRESSURE = 0x05,
        UART_STATS = 0x06
    };


    class Message
    {
    public:
        Message(MsgID id);
        virtual ~Message() = default;
        MsgID getID() const;
        bool isValid() const;
    protected:
        MsgID id;
        Byte checksum;
        
        virtual void serialize(Byte* bytes_out, size_t* size_out) const = 0;
        virtual void deserialize(const Byte* bytes_in, const size_t size_in) = 0;
        friend class Messenger;
    };


    struct UARTStatistics
    {
        struct Stats
        {
            uint32_t totalMessagesSent = 0;
            uint32_t totalMessagesReceived = 0;
            uint32_t goodMessagesReceived = 0;
            uint64_t totalBytesReceived = 0;
            uint64_t totalBytesSent = 0;
            uint32_t goodMessagesReceivedPerSec = 0;
            uint32_t messagesRecPerSec = 0;
            uint32_t messagesSentPerSec = 0;
            uint32_t bytesRecPerSec = 0;
            uint32_t bytesSentPerSec = 0;
            float goodMessagesReceivedPerSecRatio = 0.0;
            uint32_t messagesOverwritten = 0;
            uint32_t bufforOverflows = 0;
        } stats;
    private:
        uint32_t lastSecTotalMessagesSent = 0;
        uint32_t lastSecTotalMessagesReceived = 0;
        uint32_t lastSecGoodMessagesReceived = 0;
        uint32_t lastSecTotalBytesReceived = 0;
        uint32_t lastSecTotalBytesSent = 0;

        uint64_t lastCalcTime = getCurrentTimeMillis();
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
        Message* receive();

        const UARTStatistics& getStats();
    private:
        std::string serialPort;
        int serialPortFD;

        Byte receive_buff[READ_BUFF_SIZE];
        size_t receive_buff_idx = 0;
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
        MsgTenso() : Message(MsgID::TENSO) {}
        int tenso_left_raw = 0;
        int tenso_right_raw = 0;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgTemperature : public Message
    {
    public:
        MsgTemperature() : Message(MsgID::TEMPERATURE) {}
        float temperature_celsius = 0.0;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgZaworySterowanie : public Message
    {
    public:
        MsgZaworySterowanie() : Message(MsgID::ZAWORY_STEROWANIE) {}
        signed char valve_vent = 0;
        signed char valve_feed = 0;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgZaworyPozycja : public Message
    {
    public:
        MsgZaworyPozycja() : Message(MsgID::ZAWORY_POZYCJA) {}
        signed char valve_vent = 0;
        signed char valve_feed = 0;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgPressure : public Message
    {
    public:
        MsgPressure() : Message(MsgID::PRESSURE) {}
        float pressure_bar = 0.0;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };

    class MsgUartStats : public Message
    {
    public:
        MsgUartStats() : Message(MsgID::UART_STATS) {}
        UARTStatistics::Stats stats;
    private:
        void serialize(Byte* bytes_out, size_t* size_out) const override;
        void deserialize(const Byte* bytes_in, const size_t size_in) override;
    };
}