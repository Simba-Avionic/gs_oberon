#include "Obernon.hpp"

Oberon::Oberon()
    : Node("oberon")
{
    loadCellsLaunchPadPublisher = this->create_publisher<gs_interfaces::msg::LoadCells>("/oberon/launch_tower/tenso", 10);
    temperatureLaunchPadPublisher = this->create_publisher<gs_interfaces::msg::Temperature>("/oberon/launch_tower/temperature", 3);
    loadCellsLaunchPadTareSubscription = this->create_subscription<gs_interfaces::msg::LoadCellsTare>("/oberon/launch_tower/tenso_tare", 3, std::bind(&Oberon::arduinoWyrzutniaTareCallback, this, std::placeholders::_1)); 
    arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>(std::bind(&Oberon::arduinoWyrzutniaTensoCallback, this), std::bind(&Oberon::arduinoWyrzutniaTemperatureCallback, this), "/dev/ttyS0");        // RPI - always /dev/ttyS0  
    // arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>(std::bind(&Oberon::arduinoWyrzutniaTensoCallback, this), std::bind(&Oberon::arduinoWyrzutniaTemperatureCallback, this), "/dev/ttyUSB0");   // ubuntu - /dev/ttyUSB0 / /dev/ttyUSB1 ...
    wyrzutniaUartStatsPub = this->create_publisher<gs_interfaces::msg::UartStatistics>("/oberon/launch_tower/uart_stats", 10);
    wyrzutniaUartStatsTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Oberon::publishWyrzutniaUartStats, this));

    createLiveConfigIfDoesNotExist();
    loadLiveConfig();
}

Oberon::~Oberon()
{
}

void Oberon::arduinoWyrzutniaTensoCallback()
{
    gs_interfaces::msg::LoadCells msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    auto tL = arduinoWyrzutnia->getTensoL();
    auto tR = arduinoWyrzutnia->getTensoR();

    msg.tenso_l.raw_val = tL.raw_value;
    msg.tenso_l.vehicle_point = tL.rocket_point;
    msg.tenso_l.fuel_point = tL.empty_rocket_point;
    msg.tenso_l.raw_kg = tL.raw_kg;
    msg.tenso_l.vehicle_kg = tL.rocket_kg;
    msg.tenso_l.fuel_kg = tL.fuel_kg;

    msg.tenso_r.raw_val = tR.raw_value;
    msg.tenso_r.vehicle_point = tR.rocket_point;
    msg.tenso_r.fuel_point = tR.empty_rocket_point;
    msg.tenso_r.raw_kg = tR.raw_kg;
    msg.tenso_r.vehicle_kg = tR.rocket_kg;
    msg.tenso_r.fuel_kg = tR.fuel_kg;

    msg.combined_raw_kg = tL.raw_kg + tR.raw_kg;
    msg.combined_vehicle_kg = tL.rocket_kg + tR.rocket_kg;
    msg.combined_fuel_kg = tL.fuel_kg + tR.fuel_kg;

    loadCellsLaunchPadPublisher->publish(msg);
}

void Oberon::arduinoWyrzutniaTemperatureCallback()
{
    gs_interfaces::msg::Temperature msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    msg.temperature = arduinoWyrzutnia->getTemperature();
    temperatureLaunchPadPublisher->publish(msg);
}

void Oberon::publishWyrzutniaUartStats()
{
    arduinoWyrzutnia->secondPassedUpdateStats();
    gs_interfaces::msg::UartStatistics msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_fully_qualified_name();
    auto stats = arduinoWyrzutnia->getUartStats();
    msg.total_messages_received = stats.totalMessagesReceived;
    msg.total_messages_sent = stats.totalMessagesSent;
    msg.good_messages_received = stats.goodMessagesReceived;
    msg.total_bytes_received = stats.totalBytesReceived;
    msg.total_bytes_sent = stats.totalBytesSent;
    msg.messages_received_per_second = stats.messagesRecLastSec;
    msg.messages_sent_per_second = stats.messagesSentLastSec;
    msg.bytes_received_per_second = stats.bytesRecLastSec;
    msg.bytes_sent_per_second = stats.bytesSentLastSec;
    msg.good_messages_received_per_second = stats.goodMessagesReceivedLastSec;
    msg.good_messages_ratio_received_per_second = stats.goodMessagesReceivedPerSecRatio;
    wyrzutniaUartStatsPub->publish(msg);
}

void Oberon::arduinoWyrzutniaTareCallback(const gs_interfaces::msg::LoadCellsTare::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Wyrzutnia tare msg from %s", msg->header.frame_id.c_str());
    if (msg->tare_rocket_point)
        arduinoWyrzutnia->tareRocketPoint();
    if (msg->tare_empty_rocket_point)
        arduinoWyrzutnia->tareEmptyRocketPoint();
    if (msg->set_scale_left)
        arduinoWyrzutnia->setScaleLeft(msg->scale_left);
    if (msg->set_scale_right)
        arduinoWyrzutnia->setScaleRight(msg->scale_right);
    saveLiveConfig();
}

void Oberon::createLiveConfigIfDoesNotExist()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::in);
    if (!file.is_open())
    {
        file.close();
        saveLiveConfig();
    }
}

void Oberon::loadLiveConfig()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::in);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file %s", filename.c_str());
        return;
    }

    auto tensoL = arduinoWyrzutnia->getTensoL();
    file >> tensoL.rocket_point;
    file >> tensoL.empty_rocket_point;
    file >> tensoL.scale;
    auto tensoR = arduinoWyrzutnia->getTensoR();
    file >> tensoR.rocket_point;
    file >> tensoR.empty_rocket_point;
    file >> tensoR.scale;
    file.close();
}

void Oberon::saveLiveConfig()
{
    std::string filename = "." + std::string(this->get_fully_qualified_name()) + ".live.cfg";
    std::fstream file(filename, std::ios::out);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file %s", filename.c_str());
        return;
    }

    auto tensoL = arduinoWyrzutnia->getTensoL();
    file << tensoL.rocket_point << std::endl;
    file << tensoL.empty_rocket_point << std::endl;
    file << tensoL.scale << std::endl;
    auto tensoR = arduinoWyrzutnia->getTensoR();
    file << tensoR.rocket_point << std::endl;
    file << tensoR.empty_rocket_point << std::endl;
    file << tensoR.scale << std::endl;
    file.close();
}