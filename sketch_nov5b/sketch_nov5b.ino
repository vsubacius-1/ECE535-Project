#include <Wire.h>

// ---- Pin config: change only these if you rewire ----
#define SDA_PIN  23
#define SCL_PIN  22
#define INT1_PIN 13   // data ready
#define INT2_PIN 12   // activity (shake)
// ------------------------------------------------------

#define REG_DEVID       0x00
#define REG_THRESH_ACT  0x24
#define REG_ACT_INACT   0x27
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_INT_ENABLE  0x2E
#define REG_INT_MAP     0x2F
#define REG_INT_SOURCE  0x30
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32

uint8_t addr = 0;
volatile bool drdy = false, activity = false;
volatile uint32_t int1Count = 0;

void IRAM_ATTR onInt1() { drdy = true; int1Count++; }
void IRAM_ATTR onInt2() { activity = true; }

bool probe(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

bool readXYZ(float &gx, float &gy, float &gz) {
  Wire.beginTransmission(addr);
  Wire.write(REG_DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() != 6) return false;
  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);
  const float S = 0.0039f;  // 3.9 mg/LSB, full-res
  gx = x * S; gy = y * S; gz = z * S;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (probe(0x53)) addr = 0x53;
  else if (probe(0x1D)) addr = 0x1D;
  else {
    Serial.println("No ADXL345 found. Check SDA/SCL pins, 3V3 power, CS->3V3.");
    while (true) delay(1000);
  }
  Serial.printf("Found device at 0x%02X\n", addr);

  uint8_t id = readReg(REG_DEVID);
  Serial.printf("DEVID = 0x%02X (expected 0xE5)\n", id);
  if (id != 0xE5) { Serial.println("Wrong ID, stopping."); while (true) delay(1000); }

  writeReg(REG_POWER_CTL, 0x00);    // standby while configuring
  writeReg(REG_DATA_FORMAT, 0x08);  // full-res, +/-2g, INT active HIGH
  writeReg(REG_BW_RATE, 0x0A);      // 100 Hz
  writeReg(REG_THRESH_ACT, 0x08);   // ~0.5 g shake threshold
  writeReg(REG_ACT_INACT, 0xF0);    // activity on X/Y/Z, AC-coupled
  writeReg(REG_INT_MAP, 0x10);      // activity -> INT2, data ready -> INT1
  writeReg(REG_INT_ENABLE, 0x90);   // data ready + activity
  writeReg(REG_POWER_CTL, 0x08);    // measurement mode

  pinMode(INT1_PIN, INPUT);
  pinMode(INT2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), onInt1, RISING);
  attachInterrupt(digitalPinToInterrupt(INT2_PIN), onInt2, RISING);

  float a, b, c;
  readXYZ(a, b, c);
  readReg(REG_INT_SOURCE);
  Serial.println("Collecting data. Shake it to trigger INT2.\n");
}

void loop() {
  static uint32_t lastRead = 0, lastPrint = 0;
  static float gx = 0, gy = 0, gz = 0;

  // Read when INT1 fires (or every 50 ms as a fallback if INT1 isn't wired right)
  if (drdy || millis() - lastRead > 50) {
    drdy = false;
    lastRead = millis();
    readXYZ(gx, gy, gz);
  }

  if (millis() - lastPrint >= 100) {   // print at 10 Hz
    lastPrint = millis();
    float mag = sqrtf(gx*gx + gy*gy + gz*gz);
    Serial.printf("X=%6.3f  Y=%6.3f  Z=%6.3f  |a|=%5.3f g  INT1 count=%lu\n",
                  gx, gy, gz, mag, (unsigned long)int1Count);
  }

  if (activity) {
    activity = false;
    readReg(REG_INT_SOURCE);   // clears the flag
    Serial.println(">>> ACTIVITY detected (INT2)");
  }
}