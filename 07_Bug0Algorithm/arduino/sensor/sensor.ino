// 3 adet HC-SR04 ultrasonik sensor: sol on, on, sag on.
// Her sensor icin ayri Trig/Echo pini kullanilir; sensorler CAPRAZ YANKI
// riskini azaltmak icin SIRAYLA (biri bitmeden digeri tetiklenmez) okunur.
//
// Pin atamasi (gerekirse degistirin):
//   sol on : Trig = 7,  Echo = 8
//   on     : Trig = 11, Echo = 12
//   sag on : Trig = 9,  Echo = 10
//
// Seri cikti (~10 Hz, 115200 baud), tek satir JSON:
//   {"l":32,"f":40,"r":27}
// Yankı alinamazsa (zaman asimi / kablo kopuk / menzil disi) ilgili alan -1
// olur. Ana uygulama -1 degerini "gecersiz olcum" olarak ele alir, "acik
// yol" olarak YORUMLAMAZ.

const uint8_t TRIG_LEFT  = 7;
const uint8_t ECHO_LEFT  = 8;
const uint8_t TRIG_FRONT = 11;
const uint8_t ECHO_FRONT = 12;
const uint8_t TRIG_RIGHT = 9;
const uint8_t ECHO_RIGHT = 10;

const unsigned long READ_TIMEOUT_US = 25000;    // ~4.3 m karsiligi, guvenlik payiyla
const unsigned long INTER_SENSOR_GAP_MS = 10;   // sensorler arasi bekleme (capraz yanki onlemi)
const unsigned long INTERVAL_MS = 100;          // tam dongu periyodu -> ~10 Hz

unsigned long lastCycle = 0;

long readCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, READ_TIMEOUT_US);

  if (duration == 0) {
    return -1;  // zaman asimi: yankı yok / menzil disi
  }

  return (long)(duration * 0.0343 / 2.0);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

void loop() {
  if (millis() - lastCycle < INTERVAL_MS) {
    return;
  }
  lastCycle = millis();

  // Sirayla oku: bir sensorun yankisi beklenirken digeri tetiklenmez.
  long left = readCm(TRIG_LEFT, ECHO_LEFT);
  delay(INTER_SENSOR_GAP_MS);
  long front = readCm(TRIG_FRONT, ECHO_FRONT);
  delay(INTER_SENSOR_GAP_MS);
  long right = readCm(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("{\"l\":");
  Serial.print(left);
  Serial.print(",\"f\":");
  Serial.print(front);
  Serial.print(",\"r\":");
  Serial.print(right);
  Serial.println("}");
}
