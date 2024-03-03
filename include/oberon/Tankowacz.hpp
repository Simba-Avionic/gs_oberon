#pragma once

#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cell_stamped.hpp"

#include "Tenso.hpp"

class Tankowacz : public rclcpp::Node
{
public:
    Tankowacz();
    ~Tankowacz();
private:
    Tenso tenso_;
    rclcpp::Publisher<gs_interfaces::msg::LoadCellStamped>::SharedPtr load_cell_pub_;
    rclcpp::TimerBase::SharedPtr load_cell_timer_;
    void publishLoadCell();
};