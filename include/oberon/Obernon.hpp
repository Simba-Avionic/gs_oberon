#pragma once

#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "ArduinoWyrzutnia.hpp"

class Oberon : public rclcpp::Node
{
public:
    Oberon();
    ~Oberon();
private:
    std::unique_ptr<ArduinoWyrzutnia> arduinoWyrzutnia;
};