#include "Obernon.hpp"

Oberon::Oberon()
    : Node("oberon")
{
    arduinoWyrzutnia = std::make_unique<ArduinoWyrzutnia>();
}

Oberon::~Oberon()
{
}