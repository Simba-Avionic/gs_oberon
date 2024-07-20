#include "Obernon.hpp"

Oberon::Oberon()
    : Node("oberon")
{
    loadCellsPublisher = this->create_publisher<gs_interfaces::msg::LoadCells>("load_cells", 10);
    arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>(std::bind(&Oberon::arduinoWyrzutniaTensoCallback, this), std::bind(&Oberon::arduinoWyrzutniaSensorsCallback, this), "/dev/ttyS0");
}

Oberon::~Oberon()
{
}

void Oberon::arduinoWyrzutniaTensoCallback()
{
    gs_interfaces::msg::LoadCells msg;
    msg.header.stamp = this->now();
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

    loadCellsPublisher->publish(msg);
}

void Oberon::arduinoWyrzutniaSensorsCallback()
{

}