#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#include "ICM42670P.h" 

// -------------------
// USER CONFIG
// -------------------
uint8_t PEER_MAC[] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC };

// Seeed XIAO ESP32C3 default I2C pins are usually SDA=6, SCL=7
static const int I2C_SDA = 6;
static const int I2C_SCL = 7;
static const uint32_t I2C_FREQ = 400000;

// I2C address LSB: 0 -> 0x68, 1 -> 0x69 (depends on SA0 pin)
static const bool ICM_ADDR_LSB = 0;

// Strategy params
static const uint32_t LOOP_PERIOD_MS = 20;      // 50 Hz
static const float MOTION_DELTA_MPS2 = 2.0f;    // tune
static const int CONFIRM_COUNT = 3;
static const uint32_t HEARTBEAT_EVERY_N = 250;  // every ~5s

// Speed estimate knobs
static const float ACC_LPF_ALPHA = 0.20f;
static const float G = 9.80665f;
static const float ACC_CLAMP = 30.0f;
static const float SPEED_CLAMP = 40.0f;

// If your accel values look like "g", set this true to convert g->m/s^2.
static const bool ACCEL_IS_IN_G = false;

// -------------------
// ESP-NOW packet
// -------------------
#pragma pack(push, 1)
struct TelemetryPacket {
  uint32_t magic;
  uint32_t seq;
  uint32_t ms;

  float ax, ay, az;     // m/s^2 (expected)
  float acc_mag;        // filtered magnitude
  float speed_mps;      // naive integrated estimate

  bool motion;
  uint8_t _pad[3];

  uint32_t crc32;
};
#pragma pack(pop)

static const uint32_t PACKET_MAGIC = 0x54534E57;

// -------------------
// Globals / RTC state
// -------------------
ICM42670 IMU(Wire, ICM_ADDR_LSB, I2C_FREQ);

static uint32_t last_ms = 0;
static uint32_t seq_num = 0;
static float acc_mag_lpf = 0.0f;

RTC_DATA_ATTR float speed_mps = 0.0f;
RTC_DATA_ATTR bool motion_state = false;
RTC_DATA_ATTR int motion_hits = 0;
RTC_DATA_ATTR int no_motion_hits = 0;
RTC_DATA_ATTR uint32_t loop_count = 0;

// -------------------
// CRC32
// -------------------
uint32_t crc32_le(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

// -------------------
// ESP-NOW
// -------------------
void onSent(const uint8_t*, esp_now_send_status_t) {}

bool espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Wristband MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  return esp_now_add_peer(&peer) == ESP_OK;
}

// -------------------
// IMU init + read (matches library examples)
// -------------------
bool imuInit() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  int ret = IMU.begin();
  if (ret != 0) return false;

  // Configure accel + gyro like the official Polling_I2C example
  IMU.startAccel(100, 16);     // ODR 100 Hz, FSR 16G
  IMU.startGyro(100, 2000);    // ODR 100 Hz, FSR 2000 dps

  delay(100);
  return true;
}

bool readImu(float &ax, float &ay, float &az) {
  inv_imu_sensor_event_t evt{};
  IMU.getDataFromRegisters(evt);

  // Library docs: "Raw data can be translated to SI using configured FSR"
  // Example prints evt.accel[0..2] directly. :contentReference[oaicite:4]{index=4}
  ax = evt.accel[0];
  ay = evt.accel[1];
  az = evt.accel[2];

  if (ACCEL_IS_IN_G) {
    ax *= G; ay *= G; az *= G;
  }
  return true;
}

void sendTelemetry(float ax, float ay, float az, float acc_mag, float speed, bool motion, bool heartbeat) {
  TelemetryPacket pkt{};
  pkt.magic = PACKET_MAGIC;
  pkt.seq = seq_num++;
  pkt.ms = millis();
  pkt.ax = ax; pkt.ay = ay; pkt.az = az;
  pkt.acc_mag = acc_mag;
  pkt.speed_mps = speed;
  pkt.motion = motion;

  pkt.crc32 = 0;
  pkt.crc32 = crc32_le(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));

  esp_now_send(PEER_MAC, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));

  if (heartbeat) {
    Serial.printf("[HB] seq=%lu speed=%.2f motion=%d ax=%.2f ay=%.2f az=%.2f\n",
                  (unsigned long)pkt.seq, pkt.speed_mps, pkt.motion, pkt.ax, pkt.ay, pkt.az);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!espNowInit()) {
    Serial.println("ERROR: ESP-NOW init failed");
    while (true) delay(1000);
  }

  if (!imuInit()) {
    Serial.println("ERROR: ICM-42670-P init failed (wiring/address)");
    while (true) delay(1000);
  }

  Serial.println("Wristband TX ready (ICM-42670-P).");
}

void loop() {
  const uint32_t now = millis();
  if (now - last_ms < LOOP_PERIOD_MS) return;

  const float dt = (last_ms == 0) ? (LOOP_PERIOD_MS / 1000.0f) : ((now - last_ms) / 1000.0f);
  last_ms = now;

  float ax, ay, az;
  if (!readImu(ax, ay, az)) return;

  float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
  acc_mag_lpf = (1.0f - ACC_LPF_ALPHA) * acc_mag_lpf + ACC_LPF_ALPHA * acc_mag;

  // crude linear accel proxy (starter)
  float linear_acc = acc_mag_lpf - G;
  if (linear_acc > ACC_CLAMP) linear_acc = ACC_CLAMP;
  if (linear_acc < -ACC_CLAMP) linear_acc = -ACC_CLAMP;

  // integrate speed-like estimate
  speed_mps += linear_acc * dt;
  if (speed_mps < 0) speed_mps = 0;
  if (speed_mps > SPEED_CLAMP) speed_mps = SPEED_CLAMP;

  // debounced motion detection (lab vibe)
  bool motion_detected = fabsf(linear_acc) >= MOTION_DELTA_MPS2;
  if (motion_detected) { motion_hits++; no_motion_hits = 0; }
  else { no_motion_hits++; motion_hits = 0; }

  bool publish_change = false;
  bool new_state = motion_state;

  if (!motion_state && motion_hits >= CONFIRM_COUNT) { new_state = true; publish_change = true; }
  if ( motion_state && no_motion_hits >= CONFIRM_COUNT) { new_state = false; publish_change = true; }

  loop_count++;
  bool do_heartbeat = (loop_count % HEARTBEAT_EVERY_N) == 0;

  if (publish_change || do_heartbeat) {
    motion_state = new_state;
    sendTelemetry(ax, ay, az, acc_mag_lpf, speed_mps, motion_state, do_heartbeat);
  }
}