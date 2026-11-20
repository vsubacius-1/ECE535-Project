
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_timer.h"
#include <math.h>

// ---- Pin Configuration ----
#define SDA_PIN  23
#define SCL_PIN  22
#define INT1_PIN 13
#define INT2_PIN 27

// ---- ADXL345 Registers ----
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

// ---- BLE Configuration ----
// Must be the same UUIDs as the sound sketch: ble_rx.py subscribes to one
// characteristic UUID on both nodes.
const char* BLE_DEVICE_NAME = "TSN-ACU";

#define SERVICE_UUID   "CA89474F-D5F8-44BB-8926-DD4B58A44E2D"
#define DATA_CHAR_UUID "0E76AAE8-1EE4-456A-9708-DE09E78F4BCA"

// ---- Packet Configuration ----
const uint8_t PACKET_VERSION = 1;
const uint8_t NODE_ID = 1;  // 1 = ADXL345

// Same 33-byte packet structure as KY-037
struct __attribute__((packed)) SensorPacket {
  uint8_t  version;
  uint8_t  node_id;
  uint16_t seq;
  uint64_t t_edge_us;
  uint64_t t_peak_us;
  uint64_t tx_us;
  uint16_t raw;
  uint16_t level;
  uint8_t  trigger;
};

static_assert(sizeof(SensorPacket) == 33,
              "SensorPacket must be 33 bytes");

// ---- BLE Variables ----
BLECharacteristic* dataChar = nullptr;

volatile bool clientConnected = false;
bool wasConnected = false;

uint16_t seq = 0;

// ---- Sensor Variables ----
uint8_t addr = 0;

volatile bool drdy = false;
volatile uint32_t int1Count = 0;

// ---- Activity Timestamp ----
portMUX_TYPE edgeMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t firstEdgeUs = 0;

// ---- INT1: Data Ready ----
void IRAM_ATTR onInt1() {
  drdy = true;
  int1Count++;
}

// ---- INT2: Activity Detection ----
void IRAM_ATTR onInt2() {

  uint64_t now = (uint64_t)esp_timer_get_time();

  portENTER_CRITICAL_ISR(&edgeMux);

  if (firstEdgeUs == 0) {
    firstEdgeUs = now;
  }

  portEXIT_CRITICAL_ISR(&edgeMux);
}

// Get first activity timestamp and clear it
uint64_t takeFirstEdge() {

  portENTER_CRITICAL(&edgeMux);

  uint64_t t = firstEdgeUs;
  firstEdgeUs = 0;

  portEXIT_CRITICAL(&edgeMux);

  return t;
}

// ---- ADXL345 Functions ----
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

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

// Read X, Y, Z acceleration
bool readXYZ(float &gx, float &gy, float &gz) {

  Wire.beginTransmission(addr);
  Wire.write(REG_DATAX0);
  Wire.endTransmission(false);

  Wire.requestFrom(addr, (uint8_t)6);

  if (Wire.available() != 6) return false;

  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);

  const float S = 0.0039f;

  gx = x * S;
  gy = y * S;
  gz = z * S;

  return true;
}

// Convert g to milligravity (mg)
uint16_t toMilliG(float value) {

  long result = lroundf(value * 1000.0f);

  if (result < 0) result = 0;
  if (result > 65535) result = 65535;

  return (uint16_t)result;
}

// ---- BLE Connection Callbacks ----
class ServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer*) override {
    clientConnected = true;
  }

  void onDisconnect(BLEServer*) override {
    clientConnected = false;
  }
};

// ---- BLE Initialization ----
void setupBle() {

  BLEDevice::init(BLE_DEVICE_NAME);

  BLEDevice::setMTU(247);

  BLEServer* server = BLEDevice::createServer();

  server->setCallbacks(new ServerCallbacks());

  BLEService* service =
    server->createService(SERVICE_UUID);

  dataChar = service->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  dataChar->addDescriptor(new BLE2902());

  service->start();

  // Advertising
  BLEAdvertising* adv = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setName(BLE_DEVICE_NAME);

  adv->setAdvertisementData(advData);

  BLEAdvertisementData scanData;
  scanData.setCompleteServices(BLEUUID(SERVICE_UUID));

  adv->setScanResponseData(scanData);

  BLEDevice::startAdvertising();
}

// ---- Collect ADXL345 Data for 50 ms ----
void measureAccelWindow(
  float &gx,
  float &gy,
  float &gz,
  float &mag,
  float &magMin,
  float &magMax,
  uint64_t &tPeakUs
) {

  const uint32_t SAMPLE_WINDOW_MS = 50;

  uint32_t start = millis();
  uint32_t lastRead = start;

  mag = sqrtf(gx*gx + gy*gy + gz*gz);

  magMin = mag;
  magMax = mag;

  tPeakUs = (uint64_t)esp_timer_get_time();

  while (millis() - start < SAMPLE_WINDOW_MS) {

    if (drdy || millis() - lastRead >= 10) {

      drdy = false;
      lastRead = millis();

      if (readXYZ(gx, gy, gz)) {

        float currentMag = sqrtf(
          gx*gx + gy*gy + gz*gz
        );

        uint64_t sampleTime =
          (uint64_t)esp_timer_get_time();

        if (currentMag > magMax) {

          magMax = currentMag;
          tPeakUs = sampleTime;
        }

        if (currentMag < magMin) {
          magMin = currentMag;
        }

        mag = currentMag;
      }
    }
  }
}

// ---- Setup ----
void setup() {

  Serial.begin(115200);
  delay(500);

  Serial.println("\n# ADXL345 BLE starting");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Find ADXL345
  if (probe(0x53)) {
    addr = 0x53;
  }
  else if (probe(0x1D)) {
    addr = 0x1D;
  }
  else {

    Serial.println("No ADXL345 found.");

    while (true) delay(1000);
  }

  Serial.printf("Found device at 0x%02X\n", addr);

  uint8_t id = readReg(REG_DEVID);

  Serial.printf(
    "DEVID = 0x%02X (expected 0xE5)\n", id
  );

  if (id != 0xE5) {

    Serial.println("Wrong ID, stopping.");

    while (true) delay(1000);
  }

  // Configure ADXL345
  writeReg(REG_POWER_CTL, 0x00);
  writeReg(REG_DATA_FORMAT, 0x08);
  // 3200 Hz: the activity interrupt is checked every 312.5 us.
  // At 100 Hz (0x0A) it could fire up to 10 ms late, which breaks the 1 ms goal.
  writeReg(REG_BW_RATE, 0x0F);
  writeReg(REG_THRESH_ACT, 0x08);
  writeReg(REG_ACT_INACT, 0xF0);
  writeReg(REG_INT_MAP, 0x10);
  // Activity only. Data-ready at 3200 Hz would interrupt the CPU 3200 times/s.
  // The loop still reads X/Y/Z every 10 ms for the level value.
  writeReg(REG_INT_ENABLE, 0x10);
  writeReg(REG_POWER_CTL, 0x08);

  // Configure interrupt pins
  pinMode(INT1_PIN, INPUT);
  pinMode(INT2_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(INT1_PIN),
    onInt1,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(INT2_PIN),
    onInt2,
    RISING
  );

  // Initialize sensor reading
  float a, b, c;

  readXYZ(a, b, c);
  readReg(REG_INT_SOURCE);

  // Initialize BLE
  setupBle();

  Serial.printf(
    "Advertising as %s, MAC %s\n",
    BLE_DEVICE_NAME,
    BLEDevice::getAddress().toString().c_str()
  );

  Serial.println("raw,level,trigger");
}

// ---- Main Loop ----
void loop() {

  // Restart advertising after Pi disconnects
  if (wasConnected && !clientConnected) {

    BLEDevice::startAdvertising();

    Serial.println(
      "# Pi disconnected, advertising again"
    );
  }

  if (!wasConnected && clientConnected) {
    Serial.println("# Pi connected");
  }

  wasConnected = clientConnected;

  // Collect acceleration data over 50 ms
  static float gx = 0;
  static float gy = 0;
  static float gz = 0;

  float mag = 0;
  float magMin = 0;
  float magMax = 0;

  uint64_t tPeak = 0;

  measureAccelWindow(
    gx, gy, gz,
    mag, magMin, magMax, tPeak
  );

  // Get activity interrupt timestamp
  uint64_t tEdge = takeFirstEdge();

  // Clear the latched activity flag so INT2 can rise again. Also clear it if
  // INT2 is high with no timestamp (an edge was missed), or INT2 stays stuck high.
  if (tEdge != 0 || digitalRead(INT2_PIN) == HIGH) {
    readReg(REG_INT_SOURCE);
  }

  uint8_t trigger = (tEdge != 0) ? 1 : 0;

  // Prepare packet
  SensorPacket pkt;

  pkt.version = PACKET_VERSION;
  pkt.node_id = NODE_ID;
  pkt.seq = seq++;

  pkt.t_edge_us = tEdge;
  pkt.t_peak_us = tPeak;

  // Acceleration magnitude in mg
  pkt.raw = toMilliG(mag);

  // Peak-to-peak acceleration variation in mg
  pkt.level = toMilliG(magMax - magMin);

  pkt.trigger = trigger;

  // Timestamp immediately before BLE transmission
  pkt.tx_us = (uint64_t)esp_timer_get_time();

  // Send packet through BLE
  if (clientConnected) {

    dataChar->setValue(
      (uint8_t*)&pkt,
      sizeof(pkt)
    );

    dataChar->notify();
  }

  // Serial Monitor output
  Serial.print(pkt.raw);
  Serial.print(",");
  Serial.print(pkt.level);
  Serial.print(",");
  Serial.println(pkt.trigger);

}
