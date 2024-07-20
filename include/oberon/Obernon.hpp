#pragma once

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cells.hpp"

#include "ArduinoWyrzutnia.hpp"

class Oberon : public rclcpp::Node
{
public:
    Oberon();
    ~Oberon();
private:
    rclcpp::Publisher<gs_interfaces::msg::LoadCells>::SharedPtr loadCellsPublisher;
    std::unique_ptr<ArduinoWyrzutnia> arduinoWyrzutnia;
    void arduinoWyrzutniaTensoCallback();
    void arduinoWyrzutniaSensorsCallback();
};