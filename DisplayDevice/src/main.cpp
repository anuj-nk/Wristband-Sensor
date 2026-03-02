#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AccelStepper.h>

// -------------------
// USER CONFIG
// -------------------
// ULN2003 inputs (pick 4 GPIOs you are actually using)
static const int IN1 = 2;
static const int IN2 = 3;
static const int IN3 = 4;
static const int IN4 = 5;

// Gauge tuning
static const float SPEED_MIN_MPS = 0.0f;
static const float SPEED_MAX_MPS = 25.0f;     // adjust to your expected max
static const float SWEEP_DEG = 240.0f;         // how much your needle should move

// 28BYJ-48 steps per output shaft revolution depends on variant.
// Common “starter” assumption: 2048 steps/rev (half-step) or 4096 (depending on drive mode/gear).
// We'll use 4096 for half-step style smoothness equivalently.
// With AccelStepper FULL4WIRE, we’ll treat it as 4096 steps per 360° as a starting point.
static const float STEPS_PER_REV = 4096.0f;

// Signal timeout
static const uint32_t SIGNAL_TIMEOUT_MS = 1200;

// Smoothing
static const float SPEED_LPF_ALPHA = 0.25f;

// Optional: peak-hold behavior
static const bool USE_PEAK_HOLD = true;
static const uint32_t PEAK_HOLD_MS = 800;
static const float PEAK_DECAY_MPS_PER_S = 10.0f;  // after hold, decay peak down

// -------------------
// Packet (must match wristband)
// -------------------
#pragma pack(push, 1)
struct TelemetryPacket {
  uint32_t magic;
  uint32_t seq;
  uint32_t ms;

  float ax, ay, az;
  float acc_mag;
  float speed_mps;

  bool motion;
  uint8_t _pad[3];

  uint32_t crc32;
};
#pragma pack(pop)

static const uint32_t PACKET_MAGIC = 0x54534E57;

// CRC32 must match transmitter
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
// Stepper
// -------------------
// FULL4WIRE gives 4-wire stepping; for smoother motion you can also try HALF4WIRE
// but FULL4WIRE tends to be fine with the ULN2003 board.
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

long degreesToSteps(float deg) {
  if (deg < 0) deg = 0;
  if (deg > SWEEP_DEG) deg = SWEEP_DEG;

  float steps_per_deg = STEPS_PER_REV / 360.0f;
  return (long)lroundf(deg * steps_per_deg);
}

float speedToDegrees(float speed_mps) {
  float s = speed_mps;
  if (s < SPEED_MIN_MPS) s = SPEED_MIN_MPS;
  if (s > SPEED_MAX_MPS) s = SPEED_MAX_MPS;

  float t = (s - SPEED_MIN_MPS) / (SPEED_MAX_MPS - SPEED_MIN_MPS);
  return t * SWEEP_DEG;
}

// -------------------
// RX state
// -------------------
volatile bool has_new = false;
TelemetryPacket latest{};
uint32_t last_rx_ms = 0;

float speed_lpf = 0.0f;

// Peak hold state
float peak_speed = 0.0f;
uint32_t peak_set_ms = 0;

void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != (int)sizeof(TelemetryPacket)) return;

  TelemetryPacket pkt{};
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.magic != PACKET_MAGIC) return;

  uint32_t crc = pkt.crc32;
  pkt.crc32 = 0;
  uint32_t computed = crc32_le(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
  if (computed != crc) return;

  memcpy(&latest, data, sizeof(latest));
  has_new = true;
  last_rx_ms = millis();
}

bool espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Display MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Copy this MAC into wristband PEER_MAC.");

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!espNowInit()) {
    Serial.println("ERROR: ESP-NOW init failed");
    while (true) delay(1000);
  }

  // Tune these for 28BYJ-48. Too high = missed steps / buzzing.
  stepper.setMaxSpeed(800);        // steps/sec
  stepper.setAcceleration(2000);   // steps/sec^2
  stepper.setCurrentPosition(0);

  Serial.println("Display RX ready (28BYJ-48 gauge).");
}

void loop() {
  const uint32_t now = millis();

  // Always run stepper
  stepper.run();

  if (has_new) {
    has_new = false;

    // Smooth input
    speed_lpf = (1.0f - SPEED_LPF_ALPHA) * speed_lpf + SPEED_LPF_ALPHA * latest.speed_mps;

    float display_speed = speed_lpf;

    // Optional: peak-hold looks great for “throws”
    if (USE_PEAK_HOLD) {
      if (display_speed > peak_speed) {
        peak_speed = display_speed;
        peak_set_ms = now;
      } else {
        // hold then decay
        if (now - peak_set_ms > PEAK_HOLD_MS) {
          float dt = (now - peak_set_ms - PEAK_HOLD_MS) / 1000.0f;
          float decayed = peak_speed - PEAK_DECAY_MPS_PER_S * dt;
          if (decayed < display_speed) decayed = display_speed;
          peak_speed = decayed;
        }
      }
      display_speed = peak_speed;
    }

    float deg = speedToDegrees(display_speed);
    long targetSteps = degreesToSteps(deg);
    stepper.moveTo(targetSteps);

    static uint32_t dbg_t = 0;
    if (now - dbg_t > 250) {
      dbg_t = now;
      Serial.printf("seq=%lu speed=%.2f lpf=%.2f disp=%.2f -> deg=%.1f -> steps=%ld\n",
                    (unsigned long)latest.seq, latest.speed_mps, speed_lpf, display_speed, deg, targetSteps);
    }
  }

  if (now - last_rx_ms > SIGNAL_TIMEOUT_MS) {
    peak_speed = 0.0f;
    stepper.moveTo(0);
  }
}