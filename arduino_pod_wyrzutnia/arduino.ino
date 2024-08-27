#include "Wire.h"
#include <OneWire.h>
#include <Adafruit_NAU7802.h>

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
byte temp_addr[8] = {0x28, 0xC2, 0x18, 0x3A, 0x0A, 0x00, 0x00, 0x94};
float temperature = 0.0;

#define TCAADDR 0x70
Adafruit_NAU7802 nau;
Adafruit_NAU7802 nau2;
bool nau_connected;
bool nau2_connected;

int32_t tenso_1_val = 0;
int32_t tenso_2_val = 0;

#define RAMKA_META_SIZE     4
#define RAMKA_START         0xCC    // 11001100   204(10) S: 172(10)
#define RAMKA_STOP          0x33    // 00110011   51(10)  S: 19(10)
#define RAMKA_SPECIAL       0xF0    // 11110000   240(10) S: 208(10)
#define R_SPECIAL_DIF       0x20    // 00100000   32(10)
#define RAMKA_TENSO         0x01
#define RAMKA_TEMPERATURE   0x02

void connectTenso(Adafruit_NAU7802& nau, bool& nau_x_connected);
void sendTenso(int tenso_1, int tenso_2);
void sendTensoText(int32_t t1, int32_t t2);

bool readTemperature();
void sendTemperature();
void sendTemperatureText();

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(2);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  
  tcaselect(4);
  connectTenso(nau, nau_connected);
  
  tcaselect(6);
  connectTenso(nau2, nau2_connected);
  
  digitalWrite(LED_BUILTIN, LOW);
  delay(666);

  ds.reset();
  ds.select(temp_addr);
  ds.write(0x44);
}

void loop()
{
  static unsigned long t1_reconnect_time = millis();
  static unsigned long t2_reconnect_time = millis();
  static unsigned long temp_read_time = millis() + 1000;

  unsigned long tic = millis();
  if (millis() > temp_read_time)
  { 
    temp_read_time = millis() + 1000;
    if (readTemperature())
    {
      sendTemperature();
    }
    // ~17 ms
  }
  
  bool got_data = false;
  tcaselect(4);
  if (!nau_connected && millis() > t1_reconnect_time + 1000)
  {
    t1_reconnect_time = millis();
    connectTenso(nau, nau_connected);
  }
  if (nau_connected && nau.available()) {
    tenso_1_val = nau.read();
    got_data = true;
  }
  tcaselect(6);
  if (!nau2_connected && millis() > t2_reconnect_time + 1000)
  {
    t2_reconnect_time = millis();
    connectTenso(nau2, nau2_connected);
  }
  if (nau2_connected && nau2.available()) {
    tenso_2_val = nau2.read();
    got_data = true;
  }
  if (got_data)
  {
    sendTenso(tenso_1_val, tenso_2_val);
    // sendTensoText(tenso_1_val, tenso_2_val);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  unsigned long duration = millis()-tic;
  if (duration < 100)
    delay(100 - duration);

  digitalWrite(LED_BUILTIN, LOW);
}

void putByteIntoRamka(byte b, unsigned char* ramka, int& idx)
{
  if (b == RAMKA_STOP || b == RAMKA_START || b == RAMKA_SPECIAL) {
    ramka[idx++] = RAMKA_SPECIAL;
    ramka[idx++] = b - R_SPECIAL_DIF;
  } else ramka[idx++] = b;
}

void sendTensoText(int32_t t1, int32_t t2)
{
  Serial.print(t1);
  Serial.print(" ");
  Serial.print(t2);
  Serial.println();
}

void connectTenso(Adafruit_NAU7802& nau_x, bool& nau_x_connected)
{
  if (! nau_x.begin()) {
    nau_x_connected = false;
  } else {
    nau_x_connected = true;
    nau_x.setLDO(NAU7802_4V5);
    nau_x.setGain(NAU7802_GAIN_1);
    nau_x.setRate(NAU7802_RATE_10SPS);
    while (! nau_x.calibrate(NAU7802_CALMOD_INTERNAL)) {
      delay (100);
    }
  }
}

void sendTenso(int32_t tenso_1, int32_t tenso_2)
{
  unsigned char ramka[RAMKA_META_SIZE + 16];
  ramka[0] = RAMKA_START;
  ramka[1] = RAMKA_TENSO;
  int idx = 2;
  
  putByteIntoRamka((tenso_1 >> 24) & 0xFF, ramka, idx);    // 2
  putByteIntoRamka((tenso_1 >> 16) & 0xFF, ramka, idx);    // 3
  putByteIntoRamka((tenso_1 >> 8) & 0xFF, ramka, idx);    // 4
  putByteIntoRamka(tenso_1 & 0xFF, ramka, idx);    // 5

  putByteIntoRamka((tenso_2 >> 24) & 0xFF, ramka, idx);    // 6
  putByteIntoRamka((tenso_2 >> 16) & 0xFF, ramka, idx);    // 7
  putByteIntoRamka((tenso_2 >> 8) & 0xFF, ramka, idx);    // 8
  putByteIntoRamka(tenso_2 & 0xFF, ramka, idx);    // 9
  
  unsigned char checksum = 0;
  for (int i = 1; i < idx; i++)
    checksum += ramka[i];
  putByteIntoRamka(checksum, ramka, idx);
  ramka[idx++] = RAMKA_STOP;

  Serial.write(ramka, idx);
}

void sendTemperatureText()
{
  Serial.println(temperature);
}

void sendTemperature()
{
  unsigned char ramka[RAMKA_META_SIZE + 16];
  ramka[0] = RAMKA_START;
  ramka[1] = RAMKA_TEMPERATURE;
  int idx = 2;
  byte temp_bytes[4];
  memcpy(temp_bytes, &temperature, 4);
  putByteIntoRamka(temp_bytes[0], ramka, idx);    // 2
  putByteIntoRamka(temp_bytes[1], ramka, idx);    // 3
  putByteIntoRamka(temp_bytes[2], ramka, idx);    // 4
  putByteIntoRamka(temp_bytes[3], ramka, idx);    // 5
  
  unsigned char checksum = 0;
  for (int i = 1; i < idx; i++)
    checksum += ramka[i];
  putByteIntoRamka(checksum, ramka, idx);
  ramka[idx++] = RAMKA_STOP;

  Serial.write(ramka, idx);
}

bool readTemperature()
{
  if (!ds.reset())
    return false;
  
  ds.select(temp_addr);                 // ~5ms
  ds.write(0xBE);                       // 0-1ms
  
  byte data[9];
  for (byte i = 0; i < 9; i++) {        // ~5ms
    data[i] = ds.read();
  }

  if (OneWire::crc8(data, 8) != data[8])                              // 0ms FROM HERE
  {
    return false;
  }
    
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  temperature = (float)raw / 16.0;      // celsius                    // 0ms TO HERE

  // start next conversion - will take ~750 ms (on the termometer hardware)
  ds.reset();
  ds.select(temp_addr);                 // ~5ms
  ds.write(0x44);                       // 0-1ms
  return true;
}