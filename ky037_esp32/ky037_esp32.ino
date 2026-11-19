// KY-037 sound sensor -> ESP32 -> Serial output (more sensitive version)
//
// Wiring:
//   KY-037 AO  -> GPIO34 (ADC1, input only)
//   KY-037 DO  -> GPIO27 (optional, digital threshold output)
//   KY-037 +   -> 3V3
//   KY-037 G   -> GND
//
// Output format is unchanged: raw,level,trigger

const int ANALOG_PIN  = 27;
const int DIGITAL_PIN = 14;

const int   SAMPLE_WINDOW_MS = 50;    // longer window = catches slower/lower sounds
const float GAIN             = 8.0;   // software amplification: raise for more sensitivity
const int   CALIBRATION_MS   = 1500;  // keep it quiet at startup

int noiseFloor = 0;  // peak-to-peak of background noise, measured at startup

int measurePeakToPeak(int windowMs, int &lastRaw) {
  int signalMax = 0;
  int signalMin = 4095;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)windowMs) {
    lastRaw = analogRead(ANALOG_PIN);
    if (lastRaw > signalMax) signalMax = lastRaw;
    if (lastRaw < signalMin) signalMin = lastRaw;
  }
  return signalMax - signalMin;
}

void setup() {
  Serial.begin(115200);
  pinMode(DIGITAL_PIN, INPUT);
  analogReadResolution(12);  // 0-4095

  // Calibrate: measure background noise so it can be subtracted later.
  int raw = 0;
  int worst = 0;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)CALIBRATION_MS) {
    int p2p = measurePeakToPeak(SAMPLE_WINDOW_MS, raw);
    if (p2p > worst) worst = p2p;
  }
  noiseFloor = worst;

  Serial.println("raw,level,trigger");
}

void loop() {
  int raw = 0;
  int p2p = measurePeakToPeak(SAMPLE_WINDOW_MS, raw);

  // Subtract the noise floor, then amplify what's left.
  int level = (int)(max(0, p2p - noiseFloor) * GAIN);
  if (level > 4095) level = 4095;

  Serial.print(raw);
  Serial.print(",");
  Serial.print(level);
  Serial.print(",");
  Serial.println(digitalRead(DIGITAL_PIN));
}
