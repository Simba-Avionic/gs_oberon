#include <Servo.h>
#define GSUART_PLATFORM_ARDUINO     0
#define GSUART_PLATFORM_RPI_UBUNTU  1
#define GSUART_PLATFORM GSUART_PLATFORM_ARDUINO
#include "GSUART.hpp"
GSUART::Messenger messenger(&Serial);
#define SEND_INTERVAL_UART_STATS  4000
#define FEED_PIN 8
#define VENT_PIN 9
#define RESET_TIME 5000
void readServoCtrl();
Servo serwo_vent;
Servo serwo_feed;
int8_t pos_vent = 0, pos_feed = 0;
unsigned long last_update = 0;

void setup() {
  serwo_feed.attach(FEED_PIN);
  serwo_vent.attach(VENT_PIN);
  Serial.begin(9600);
}

void loop() {
  static unsigned long uart_stats_read_time = millis();
  unsigned long tic = millis();
  if (millis() >= uart_stats_read_time)
    {
      uart_stats_read_time = millis() + SEND_INTERVAL_UART_STATS;
      messenger.sendUartStats();      
    } 

  readServoCtrl();
  if (tic - last_update >= RESET_TIME){
    pos_vent = 0;
    pos_feed = 0;
    last_update = millis();
  }
  int value_vent = map(static_cast<int>(pos_vent), 0, 100, 0, 180);
  serwo_vent.write(value_vent);
  int value_feed = map(static_cast<int>(pos_feed), 0, 100, 0, 180);
  serwo_feed.write(value_feed);
}

void readServoCtrl(){

  GSUART::Message* msg = nullptr;
  msg = messenger.receive();

  if (msg == nullptr)
    return;

  switch (msg->getID()){
    case GSUART::MsgID::ZAWORY_STEROWANIE:
       GSUART::MsgZaworySterowanie* msgZaworyS = static_cast<GSUART::MsgZaworySterowanie*>(msg);
       pos_vent = msgZaworyS->valve_vent;
       pos_feed = msgZaworyS->valve_feed;
       last_update = millis();

    default:
      break;
  }
}
