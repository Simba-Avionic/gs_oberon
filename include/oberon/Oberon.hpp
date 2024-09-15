#pragma once

#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cells.hpp"
#include "gs_interfaces/msg/load_cells_params.hpp"
#include "gs_interfaces/msg/load_cells_tare.hpp"
#include "gs_interfaces/msg/temperature.hpp"
#include "gs_interfaces/msg/uart_statistics.hpp"
#include "gs_interfaces/msg/power.hpp"

#include "ArduinoWyrzutnia.hpp"
#include "PowerMonitor.hpp"

class Oberon : public rclcpp::Node
{
public:
    Oberon();
    ~Oberon();
private:
    rclcpp::Publisher<gs_interfaces::msg::LoadCells>::SharedPtr loadCellsLaunchPadPublisher;
    rclcpp::Publisher<gs_interfaces::msg::UartStatistics>::SharedPtr wyrzutniaUartStatsPub;
    rclcpp::Publisher<gs_interfaces::msg::Temperature>::SharedPtr temperatureLaunchPadPublisher;
    rclcpp::Publisher<gs_interfaces::msg::LoadCellsParams>::SharedPtr loadCellsParamsPublisher;
    rclcpp::Publisher<gs_interfaces::msg::Power>::SharedPtr powerMonitorPublisher;
    rclcpp::Subscription<gs_interfaces::msg::LoadCellsTare>::SharedPtr loadCellsLaunchPadTareSubscription;
    std::unique_ptr<ArduinoWyrzutnia> arduinoWyrzutnia;
    void arduinoWyrzutniaTareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg);
    void arduinoWyrzutniaTensoCallback();
    void arduinoWyrzutniaTemperatureCallback();

    std::unique_ptr<PowerMonitor> powerMonitor;
    void powerMonitorCallback();

    rclcpp::TimerBase::SharedPtr oneSecondTimer;
    void oneSecondTimerCallback();

    void publishWyrzutniaUartStats();
    void publishLoadCellsParams();

    void createLiveConfigIfDoesNotExist();
    void loadLiveConfig();
    void saveLiveConfig();
};