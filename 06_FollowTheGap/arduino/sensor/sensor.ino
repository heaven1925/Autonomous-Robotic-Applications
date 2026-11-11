// 1 adet ultrasonik sensör (HC-SR04), robotun merkezine bakacak şekilde.
// center : Trig = 4, Echo = 3
//
// Seri çıktı örneği (her ~100ms'de bir satır):
// {"center":40}
// Sensör hiçbir şey algılamazsa (yankı zaman aşımına uğrarsa, kablo kopuksa
// vb.) center değeri -1 olur.

const uint8_t TRIG_CENTER = 4;
const uint8_t ECHO_CENTER = 3;

const unsigned long READ_TIMEOUT_US = 25000;
const unsigned long INTERVAL_MS = 100;  // 100 ms -> saniyede ~10 okuma

unsigned long lastRead = 0;

long readCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, READ_TIMEOUT_US);

  if (duration == 0) {
    return -1;
  }

  return (long)(duration * 0.0343 / 2.0);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_CENTER, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
}

void loop() {
  if (millis() - lastRead < INTERVAL_MS) {
    return;
  }

  lastRead = millis();

  long center = readCm(TRIG_CENTER, ECHO_CENTER);

  Serial.print("{\"center\":");
  Serial.print(center);
  Serial.println("}");
}
