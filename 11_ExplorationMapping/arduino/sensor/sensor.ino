// Tek HC-SR04, robotun onunde (Ornek 11 - Exploration Mapping).
// Robot etrafi yerinde 360 derece donerek olcer.
//
// Kablolama 07/08'deki kartin ON kanaliyla AYNIDIR:
//   on : Trig = D11, Echo = D12, VCC = 5V, GND = GND
// Sensor robot merkezinden ~11 cm onde, yere paralel ve tam ileri bakar.
//
// Olcumler FILTRESIZ (ham) gonderilir: deneysel kalibrasyon (Gaussian
// gurultu modeli, P(olay|Bos)) ve Bayes guncellemesi her olcumu ayri bir
// gozlem olarak kullanir. (08'deki "son 2 olcumun kucugu" filtresi yok.)
//
// Seri cikti (~10 Hz, 115200 baud), 07/08 ile ayni JSON bicimi:
//   {"l":-1,"f":40,"r":-1}
// Yan kanallar takili olmadigi icin hep -1; ana uygulama yalniz "f"yi kullanir.
// Yanki alinamazsa (zaman asimi / menzil disi, ~4 m) "f" de -1 olur.

const uint8_t TRIG_PIN = 11;
const uint8_t ECHO_PIN = 12;

const unsigned long READ_TIMEOUT_US = 25000;   // ~4.3 m karsiligi
const unsigned long INTERVAL_MS = 100;         // ~10 Hz

unsigned long lastCycle = 0;

long readCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, READ_TIMEOUT_US);
  if (duration == 0) {
    return -1;  // zaman asimi: yanki yok / menzil disi
  }
  return (long)(duration * 0.0343 / 2.0);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  if (millis() - lastCycle < INTERVAL_MS) {
    return;
  }
  lastCycle = millis();

  Serial.print("{\"l\":-1,\"f\":");
  Serial.print(readCm());
  Serial.println(",\"r\":-1}");
}
