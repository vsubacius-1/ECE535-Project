// KY-037 sound sensor -> ESP32 -> BLE notify -> Raspberry Pi
//
// Wiring (matches the pin constants below):
//   KY-037 AO  -> GPIO27 (analog level)
//   KY-037 DO  -> GPIO14 (comparator output, timestamped by interrupt)
//   KY-037 +   -> 3V3   (keep at 3.3 V, so DO stays within the ESP32 input limit)
//   KY-037 G   -> GND
//
// What it does:
//   1. Measures peak-to-peak sound level in 50 ms windows (same as before).
//   2. Timestamps the first DO edge in each window in an interrupt, using the
//      ESP32 64-bit microsecond clock. This is the sync timestamp for the Pi.
//   3. Sends one 33-byte packet per window to the Pi as a BLE notification.
//   4. Still prints "raw,level,trigger" on USB serial for debugging.
//
// Board package: "esp32" by Espressif, version 3.x. Board: ESP32 Dev Module.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>  // Bluedroid needs this for notifications. NimBLE adds it itself.
#endif
#include "esp_timer.h"

// ---- Pins ---------------------------------------------------------------
const int ANALOG_PIN  = 27;   // AO. ADC2 pin: OK with BLE, but NOT with Wi-Fi.
const int DIGITAL_PIN = 14;   // DO

// ---- Sound processing (unchanged) --------------------------------------
const int   SAMPLE_WINDOW_MS = 50;    // longer window = catches slower/lower sounds
const float GAIN             = 8.0;   // software amplification: raise for more sensitivity
const int   CALIBRATION_MS   = 1500;  // keep it quiet at startup

// ---- BLE ----------------------------------------------------------------
// The Pi script (pi/ble_rx.py) uses the same name and UUIDs. Change both together.
const char* BLE_DEVICE_NAME = "TSN-Sound";
#define SERVICE_UUID   "CA89474F-D5F8-44BB-8926-DD4B58A44E2D"
#define DATA_CHAR_UUID "0E76AAE8-1EE4-456A-9708-DE09E78F4BCA"

const uint8_t PACKET_VERSION = 1;
const uint8_t NODE_ID        = 2;     // 1 = ACU (ADXL345), 2 = sound (KY-037)

// One notification per sample window. Little-endian, no padding.
// Python: struct.unpack('<BBHQQQHHB', data)
struct __attribute__((packed)) SoundPacket {
  uint8_t  version;    // PACKET_VERSION
  uint8_t  node_id;    // NODE_ID
  uint16_t seq;        // +1 per packet, wraps at 65535. Gaps = lost packets.
  uint64_t t_edge_us;  // ESP32 clock at the first DO edge in this window. 0 = no edge.
  uint64_t t_peak_us;  // ESP32 clock at the loudest AO sample in this window
  uint64_t tx_us;      // ESP32 clock just before sending. Pi uses it for delay/drift.
  uint16_t raw;        // last AO reading, 0-4095
  uint16_t level;      // noise-subtracted, amplified peak-to-peak, 0-4095
  uint8_t  trigger;    // 1 if DO changed at least once in this window
};
static_assert(sizeof(SoundPacket) == 33, "SoundPacket must be 33 bytes");

BLECharacteristic* dataChar = nullptr;
volatile bool clientConnected = false;
bool wasConnected = false;
uint16_t seq = 0;

int noiseFloor = 0;  // peak-to-peak of background noise, measured at startup

// ---- DO edge capture ----------------------------------------------------
// The ISR keeps only the first edge per window. A 64-bit value is not atomic
// on the ESP32, so a spinlock protects it.
portMUX_TYPE edgeMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t firstEdgeUs = 0;

void IRAM_ATTR onDoEdge() {
  uint64_t now = (uint64_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&edgeMux);
  if (firstEdgeUs == 0) firstEdgeUs = now;
  portEXIT_CRITICAL_ISR(&edgeMux);
}

// Returns the first edge time since the last call (0 if none), and clears it.
uint64_t takeFirstEdge() {
  portENTER_CRITICAL(&edgeMux);
  uint64_t t = firstEdgeUs;
  firstEdgeUs = 0;
  portEXIT_CRITICAL(&edgeMux);
  return t;
}

// ---- BLE connection state -----------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { clientConnected = true; }
  void onDisconnect(BLEServer*) override { clientConnected = false; }
};

// ---- Sampling -----------------------------------------------------------
int measurePeakToPeak(int windowMs, int &lastRaw, uint64_t &tPeakUs) {
  int signalMax = 0;
  int signalMin = 4095;
  tPeakUs = 0;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)windowMs) {
    lastRaw = analogRead(ANALOG_PIN);
    if (lastRaw > signalMax) {
      signalMax = lastRaw;
      tPeakUs = (uint64_t)esp_timer_get_time();
    }
    if (lastRaw < signalMin) signalMin = lastRaw;
  }
  return signalMax - signalMin;
}

void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);  // the 33-byte packet needs more than the default 20-byte payload

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  dataChar = service->createCharacteristic(
      DATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  dataChar->addDescriptor(new BLE2902());
#endif
  service->start();

  // Advertising packet (max 31 bytes): flags + name, so every scanner sees
  // "TSN-Sound", including passive scanners. The 128-bit service UUID does not
  // also fit, so it goes in the scan response.
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setFlags(0x06);  // general discoverable, BLE only
  advData.setName(BLE_DEVICE_NAME);
  adv->setAdvertisementData(advData);
  BLEAdvertisementData scanData;
  scanData.setCompleteServices(BLEUUID(SERVICE_UUID));
  adv->setScanResponseData(scanData);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n# ky037_esp32_ble starting, calibrating (keep quiet)");
  pinMode(DIGITAL_PIN, INPUT);
  analogReadResolution(12);  // 0-4095

  // Calibrate first, before the radio starts: measure background noise so it
  // can be subtracted later.
  int raw = 0;
  uint64_t tPeak = 0;
  int worst = 0;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)CALIBRATION_MS) {
    int p2p = measurePeakToPeak(SAMPLE_WINDOW_MS, raw, tPeak);
    if (p2p > worst) worst = p2p;
  }
  noiseFloor = worst;

  // CHANGE catches the onset whatever the DO polarity of your module is.
  attachInterrupt(digitalPinToInterrupt(DIGITAL_PIN), onDoEdge, CHANGE);
  takeFirstEdge();  // discard edges from calibration

  setupBle();

  Serial.printf("noiseFloor=%d, advertising as %s, MAC %s\n", noiseFloor, BLE_DEVICE_NAME,
                BLEDevice::getAddress().toString().c_str());
  Serial.println("raw,level,trigger");
}

void loop() {
  // Restart advertising after the Pi disconnects, so it can reconnect.
  if (wasConnected && !clientConnected) {
    BLEDevice::startAdvertising();
    Serial.println("# Pi disconnected, advertising again");
  }
  if (!wasConnected && clientConnected) {
    Serial.println("# Pi connected");
  }
  wasConnected = clientConnected;

  int raw = 0;
  uint64_t tPeak = 0;
  int p2p = measurePeakToPeak(SAMPLE_WINDOW_MS, raw, tPeak);
  uint64_t tEdge = takeFirstEdge();

  // Subtract the noise floor, then amplify what's left.
  int level = (int)(max(0, p2p - noiseFloor) * GAIN);
  if (level > 4095) level = 4095;
  uint8_t trigger = (tEdge != 0) ? 1 : 0;

  if (clientConnected) {
    SoundPacket pkt;
    pkt.version   = PACKET_VERSION;
    pkt.node_id   = NODE_ID;
    pkt.seq       = seq++;
    pkt.t_edge_us = tEdge;
    pkt.t_peak_us = tPeak;
    pkt.raw       = (uint16_t)raw;
    pkt.level     = (uint16_t)level;
    pkt.trigger   = trigger;
    pkt.tx_us     = (uint64_t)esp_timer_get_time();  // last, as close to send as possible
    dataChar->setValue((uint8_t*)&pkt, sizeof(pkt));
    dataChar->notify();
  }

  Serial.print(raw);
  Serial.print(",");
  Serial.print(level);
  Serial.print(",");
  Serial.println(trigger);
}
