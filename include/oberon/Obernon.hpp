#pragma once

#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cells.hpp"
#include "gs_interfaces/msg/load_cells_tare.hpp"
#include "gs_interfaces/msg/temperature.hpp"
#include "gs_interfaces/msg/uart_statistics.hpp"

#include "ArduinoWyrzutnia.hpp"

class Oberon : public rclcpp::Node
{
public:
    Oberon();
    ~Oberon();
private:
    rclcpp::Publisher<gs_interfaces::msg::LoadCells>::SharedPtr loadCellsLaunchPadPublisher;
    rclcpp::Publisher<gs_interfaces::msg::Temperature>::SharedPtr temperatureLaunchPadPublisher;
    rclcpp::Subscription<gs_interfaces::msg::LoadCellsTare>::SharedPtr loadCellsLaunchPadTareSubscription;
    std::unique_ptr<ArduinoWyrzutnia> arduinoWyrzutnia;
    void arduinoWyrzutniaTareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg);
    void arduinoWyrzutniaTensoCallback();
    void arduinoWyrzutniaTemperatureCallback();

    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr wyrzutniaUartStatsPub;
    rclcpp::TimerBase::SharedPtr wyrzutniaUartStatsTimer;
    void publishWyrzutniaUartStats();

    void createLiveConfigIfDoesNotExist();
    void loadLiveConfig();
    void saveLiveConfig();
};