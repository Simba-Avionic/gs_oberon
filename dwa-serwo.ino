#include <Servo.h>
#include <iostream>

Servo serwo1;
Servo serwo2;
int pos1 = 0, pos2 = 0;
bool s1 = true;
bool info = true;

void ramka_in();

void setup() {
  serwo1.attach(9);
  serwo2.attach(10);
  Serial.begin(9600);
}
// na razie program pozwala sterować serwami z komputera
void loop() {
if (Serial.available() > 0 and !info){
  int val = Serial.parseInt();
  info = true;
  if (s1){
  pos1 = val;
  s1 = false;
  }
  else{
  pos2 = val;
  s1 = true;
  }
  Serial.parseInt(); // do usunięcia zera
}
  if (pos1 <= 180) {
    serwo1.write(pos1);
    Serial.println("Nowa pozycja serwo 1: " + String(pos1));
  }
  if (pos2 <= 180) {
    serwo2.write(pos2);
    Serial.println("Nowa pozycja serwo 2: " + String(pos2));
  }
  if (info){
  if (s1)
    Serial.println("Pozycja serwo 1:");
  else
    Serial.println("Pozycja serwo 2:");
  info = false;
  }
}

//TODO: odbieranie ramki

void ramka_decode(unsigned char* ramka, insigned int size){  //do funkcji wysyłamy ramkę bez bajtu startu i stopu, bo po co
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
  pos1 = static_cast<int>(ramka_r[1]);
  pos2 = static_cast<int>(ramka_r[2]);
}
}