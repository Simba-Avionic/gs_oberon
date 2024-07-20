#include "Obernon.hpp"

Oberon::Oberon()
    : Node("oberon")
{
    arduinoWyrzutnia = new ArduinoWyrzutnia();
}

Oberon::~Oberon()
{
    delete arduinoWyrzutnia;
}