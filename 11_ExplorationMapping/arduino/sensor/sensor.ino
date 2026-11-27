// Tek HC-SR04, robotun onunde (Ornek 11 - Exploration Mapping).
// Robot etrafi yerinde 360 derece donerek olcer.
//
// Varsayilan kablolama 07/08'deki kartin ON kanaliyla AYNIDIR:
//   on : Trig = D11, Echo = D12, VCC = 5V, GND = GND
// Sensor robot merkezinden ~11 cm onde, yere paralel ve tam ileri bakar.
//
// Pinler degistirilebilir (ExploreLauncher -> 3. adim "Sensor kalibrasyonu"
// -> Trig / Echo -> "Karta yaz"). Secim EEPROM'a yazilir, kart yeniden
// baslasa da kalir. Seri komutlar (satir sonu \n):
//   ?             -> {"ver":2,"trig":11,"echo":12}      ayar sorgusu
//   PINS 7 8      -> Trig = D7, Echo = D8 (D2..D13, farkli) ve ayar satiri
//                    gecersizse {"err":"pins"}
//   PINS 11 12    -> varsayilana donus
//
// Olcumler FILTRESIZ (ham) gonderilir: deneysel kalibrasyon (Gaussian
// gurultu modeli, P(olay|Bos)) ve Bayes guncellemesi her olcumu ayri bir
// gozlem olarak kullanir. (08'deki "son 2 olcumun kucugu" filtresi yok.)
//
// Seri cikti (~10 Hz, 115200 baud), 07/08 ile ayni JSON bicimi + ham yanki suresi:
//   {"l":-1,"f":40,"r":-1,"us":2332}
//   f  = cm (343 m/s ile, yuvarlanmis)   us = yanki suresi, mikrosaniye
// Yan kanallar takili olmadigi icin hep -1; ana uygulama yalniz "f"yi kullanir.
// Yanki alinamazsa (zaman asimi / menzil disi, ~4 m) "f" ve "us" -1 olur.
// 40 cm kalibrasyonu (launcher) PC tarafinda uygulanir: cm = ham_cm * scale
// (sensor_calibration.txt) - bu sketch hep ham degeri gonderir.

#include <EEPROM.h>

const int SKETCH_VERSION = 2;
const uint8_t DEFAULT_TRIG = 11;
const uint8_t DEFAULT_ECHO = 12;
const uint8_t EEPROM_MAGIC = 0xA7;   // EEPROM[0]; [1] = trig, [2] = echo

const unsigned long READ_TIMEOUT_US = 25000;   // ~4.3 m karsiligi
const unsigned long INTERVAL_MS = 100;         // ~10 Hz

uint8_t trigPin = DEFAULT_TRIG;
uint8_t echoPin = DEFAULT_ECHO;
unsigned long lastCycle = 0;

char command[24];
uint8_t commandLength = 0;

bool validPins(int trig, int echo) {
  return trig >= 2 && trig <= 13 && echo >= 2 && echo <= 13 && trig != echo;   // D0/D1 = USB seri
}

void applyPins(uint8_t trig, uint8_t echo) {
  pinMode(trigPin, INPUT);   // eski pini birak
  pinMode(echoPin, INPUT);
  trigPin = trig;
  echoPin = echo;
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
}

void printConfig() {
  Serial.print("{\"ver\":");
  Serial.print(SKETCH_VERSION);
  Serial.print(",\"trig\":");
  Serial.print(trigPin);
  Serial.print(",\"echo\":");
  Serial.print(echoPin);
  Serial.println("}");
}

void handleCommand() {
  command[commandLength] = '\0';
  if (command[0] == '?') {
    printConfig();
  } else if (strncmp(command, "PINS", 4) == 0) {
    int trig = -1, echo = -1;
    if (sscanf(command + 4, "%d %d", &trig, &echo) == 2 && validPins(trig, echo)) {
      EEPROM.update(0, EEPROM_MAGIC);
      EEPROM.update(1, (uint8_t)trig);
      EEPROM.update(2, (uint8_t)echo);
      applyPins((uint8_t)trig, (uint8_t)echo);
      printConfig();
    } else {
      Serial.println("{\"err\":\"pins\"}");
    }
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (commandLength > 0) handleCommand();
      commandLength = 0;
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = c;
    }
  }
}

long readEchoUs() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, READ_TIMEOUT_US);
  if (duration == 0) {
    return -1;  // zaman asimi: yanki yok / menzil disi
  }
  return (long)duration;
}

void setup() {
  Serial.begin(115200);
  uint8_t trig = DEFAULT_TRIG, echo = DEFAULT_ECHO;
  if (EEPROM.read(0) == EEPROM_MAGIC && validPins(EEPROM.read(1), EEPROM.read(2))) {
    trig = EEPROM.read(1);
    echo = EEPROM.read(2);
  }
  trigPin = trig;
  echoPin = echo;
  applyPins(trig, echo);
}

void loop() {
  readCommands();
  if (millis() - lastCycle < INTERVAL_MS) {
    return;
  }
  lastCycle = millis();

  const long us = readEchoUs();
  const long cm = us < 0 ? -1 : (long)(us * 0.0343 / 2.0 + 0.5);
  Serial.print("{\"l\":-1,\"f\":");
  Serial.print(cm);
  Serial.print(",\"r\":-1,\"us\":");
  Serial.print(us);
  Serial.println("}");
}
