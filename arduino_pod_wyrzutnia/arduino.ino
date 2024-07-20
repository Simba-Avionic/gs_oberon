#include "Wire.h"
#include <Adafruit_NAU7802.h>

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

void connectTenso(Adafruit_NAU7802& nau, bool& nau_x_connected);
void sendTenso(int tenso_1, int tenso_2);
void sendTensoText(int32_t t1, int32_t t2);

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
}

void loop()
{
  static unsigned long t1_reconnect_time = millis();
  static unsigned long t2_reconnect_time = millis();
  
  bool got_data = false;
  tcaselect(4);                                               // + delay(2)
  if (!nau_connected && millis() > t1_reconnect_time + 1000)
  {
    t1_reconnect_time = millis();
    connectTenso(nau, nau_connected);
  }
  if (nau_connected && nau.available()) {
    tenso_1_val = nau.read();
    got_data = true;
  }
  tcaselect(6);                                               // + delay(2)
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
//    sendTensoText(tenso_1_val, tenso_2_val);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(6);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else 
    delay(5);
}

void putByteIntoRamka(unsigned char b, unsigned char* ramka, int& idx)
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
