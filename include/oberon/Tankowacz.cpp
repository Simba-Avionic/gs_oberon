#include "Tankowacz.hpp"

Tankowacz::Tankowacz() : Node("oberon"), tenso_()
{
    using namespace std::chrono_literals;
    load_cell_pub_ = this->create_publisher<gs_interfaces::msg::LoadCellStamped>("/tankowanie/LoadCell", 10);
    load_cell_timer_ = this->create_wall_timer(500ms, std::bind(&Tankowacz::publishLoadCell, this));
}

Tankowacz::~Tankowacz()
{

}

void Tankowacz::publishLoadCell()
{
    load_cell_pub_->publish(tenso_.getLastReadings());
}