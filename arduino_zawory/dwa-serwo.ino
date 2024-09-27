#include <Servo.h>

Servo serwo1;
Servo serwo2;
char pos1 = 0, pos2 = 0;
unsigned long last_update = 0;
unsigned long time = 0;
void ramka_in();

void setup() {
  serwo1.attach(9);
  serwo2.attach(10);
  Serial.begin(9600);
}

void loop() {
if (Serial.available())
  ramka_read();
int value = map(static_cast<int>(pos1), 0, 100, 0, 180);
serwo1.write(value);
value = map(static_cast<int>(pos2), 0, 100, 0, 180);
serwo2.write(value);

time = millis();
if (time - last_update > 5000){
  serwo1.write(0);
  serwo2.write(0);
  last_update = millis();
}
}

void ramka_read(){
unsigned char ramka[128];
unsigned int ramka_size = 0;
bool special = false;
if (Serial.read() == 0x03){
  while (true){
    unsigned char buf = Serial.read();
    if (!special){
      if (buf == 0x33){
        ramka_decode(ramka, ramka_size);
        return;
      }
      else if (buf == 0x03){  // bajt startu resetuje ramkę 
        for (int i = 0; i < ramka_size; ++i){
          ramka[i] = NULL;
        }
        ramka_size = 0;
        continue; // to moze nie dzialac
      }
    }
    else
      special = false;
    if (buf == 0xF0)
      special = true;
    if (ramka_size > 127)
      return;
    ramka[ramka_size] = buf;
    ++ramka_size;
  }
}
}

void ramka_decode(unsigned char* ramka, unsigned int size){  //do funkcji wysyłamy ramkę bez bajtu startu i stopu, bo po co
unsigned char ramka_r[128];
unsigned int id_r = 0;
for (unsigned int i = 0; i<size; ++i){
  if (ramka[i] == 0xF0)
    ramka[++i] += 0x20;
  ramka_r[id_r] = ramka[i];
  ++id_r;
}
unsigned char checksuma = 0;
for (unsigned int i = 0; i<id_r-1; ++i){
  checksuma += ramka_r[i];
}
if (checksuma != ramka_r[id_r - 1])
  return;
if (ramka_r[0] == 0x03){
  if (ramka_r[4] != NULL) //warunek do odrzucania zbyt długich ramek
    return;
  pos1 = static_cast<char>(ramka_r[1]);
  pos2 = static_cast<char>(ramka_r[2]);
  last_update = millis();
}
}
