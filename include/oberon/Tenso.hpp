#pragma once

#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "gs_interfaces/msg/load_cell_stamped.hpp"

#include "nau7802/NAU7802.h"

class Tenso
{
public:
    Tenso();
    ~Tenso();
    const gs_interfaces::msg::LoadCellStamped& getLastReadings();
    const bool& tensoConnected();
private:
    NAU7802 nau_;
    bool nau_connected_;
    gs_interfaces::msg::LoadCellStamped last_readings_;
    std::thread read_thread_;
    void readTenso();
};