// 3 adet HC-SR04 ultrasonik sensor: sol on, on, sag on (Mapping, Ornek 09).
//
// Donanim, pinler ve seri protokol 08_TangentBugAlgorithm ile AYNIDIR (bu dosya
// onun kopyasidir): ayni sensor karti ve ana uygulamadaki ArduinoTriSensor.h
// degistirilmeden kullanilir. Her kanal icin son 2 olcumun KUCUGU gonderilir.
// HC-SR04 egik / yumusak yuzeylerde yankiyi sik sik kacirir; bu tek
// seferlik sahte "uzak" (veya -1) okuma haritada duvarin icinden "bos"
// isaretlenmesine yol acabilir. Son 2 olcumun kucugu bunu bastirir ve yeni
// beliren YAKIN bir engeli ayni dongude bildirir (gecikme yok).
//
// Pin atamasi (gerekirse degistirin):
//   sol on : Trig = 9,  Echo = 10
//   on     : Trig = 11, Echo = 12
//   sag on : Trig = 7,  Echo = 8
//
// Seri cikti (~10 Hz, 115200 baud), tek satir JSON:
//   {"l":32,"f":40,"r":27}
// Yanki alinamazsa (zaman asimi / menzil disi) ilgili alan -1 olur; haritalama
// uygulamasi -1'de yalnizca kisa bir mesafeyi "bos" sayar (egik duz duvar da
// yanki vermez). Kablo/paket hatalari ise alanin hic gelmemesiyle (gecersiz)
// ayirt edilir.

const uint8_t TRIG_PINS[3] = {9, 11, 7};   // sol, on, sag
const uint8_t ECHO_PINS[3] = {10, 12, 8};

const unsigned long READ_TIMEOUT_US = 25000;    // ~4.3 m karsiligi, guvenlik payiyla
const unsigned long INTER_SENSOR_GAP_MS = 10;   // sensorler arasi bekleme (capraz yanki onlemi)
const unsigned long INTERVAL_MS = 100;          // tam dongu periyodu -> ~10 Hz
const long NO_ECHO = 100000;                    // filtre icin "yanki yok" = cok uzak

long history[3][2];          // [kanal][son 2 olcum]
uint8_t historyIndex = 0;
unsigned long lastCycle = 0;

long readCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, READ_TIMEOUT_US);
  if (duration == 0) {
    return NO_ECHO;  // zaman asimi: yanki yok / menzil disi
  }
  return (long)(duration * 0.0343 / 2.0);
}

long filtered(uint8_t ch) {
  long m = min(history[ch][0], history[ch][1]);
  return (m >= NO_ECHO) ? -1 : m;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    for (uint8_t k = 0; k < 2; ++k) history[i][k] = NO_ECHO;
  }
}

void loop() {
  if (millis() - lastCycle < INTERVAL_MS) {
    return;
  }
  lastCycle = millis();

  // Sirayla oku: bir sensorun yankisi beklenirken digeri tetiklenmez.
  for (uint8_t i = 0; i < 3; ++i) {
    history[i][historyIndex] = readCm(TRIG_PINS[i], ECHO_PINS[i]);
    if (i < 2) delay(INTER_SENSOR_GAP_MS);
  }
  historyIndex = (historyIndex + 1) % 2;

  Serial.print("{\"l\":");
  Serial.print(filtered(0));
  Serial.print(",\"f\":");
  Serial.print(filtered(1));
  Serial.print(",\"r\":");
  Serial.print(filtered(2));
  Serial.println("}");
}
