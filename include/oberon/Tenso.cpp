#include "Tenso.hpp"

Tenso::Tenso() : nau_(1, 0x2A)
{
    if (nau_.begin() == false)
    {
        // TODO: logger
        nau_connected_ = false;
        return;
    }
    nau_.setSampleRate(NAU7802_SPS_10);
    read_thread_ = std::thread(&Tenso::readTenso, this);
    nau_connected_ = true;
}

Tenso::~Tenso()
{

}

const bool& Tenso::tensoConnected()
{
    return nau_connected_;
}

const gs_interfaces::msg::LoadCellStamped& Tenso::getLastReadings()
{
    return last_readings_;
}

void Tenso::readTenso()
{
    while (true)
    {
        if (nau_.available())
        {
            long currentReading = nau_.getReading();
            // last_readings_.header.stamp 
            last_readings_.load_cell.raw_val = currentReading;
        }
        usleep(11E3);
    }
}