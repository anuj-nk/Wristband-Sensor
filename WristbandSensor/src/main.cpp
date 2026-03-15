#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// =====================================================
// MPU-6050 wiring
// =====================================================
const int IMU_SDA = D4;
const int IMU_SCL = D5;
const int IMU_INT = D7;

const uint8_t MPU_ADDR = 0x68;
uint8_t imuAddr = 0;

// =====================================================
// MPU-6050 registers
// =====================================================
const uint8_t REG_WHO_AM_I      = 0x75;
const uint8_t REG_PWR_MGMT_1    = 0x6B;
const uint8_t REG_SMPLRT_DIV    = 0x19;
const uint8_t REG_CONFIG        = 0x1A;
const uint8_t REG_GYRO_CONFIG   = 0x1B;
const uint8_t REG_ACCEL_CONFIG  = 0x1C;
const uint8_t REG_INT_PIN_CFG   = 0x37;
const uint8_t REG_INT_ENABLE    = 0x38;
const uint8_t REG_INT_STATUS    = 0x3A;
const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

// =====================================================
// BLE settings - MUST match display code
// =====================================================
static BLEUUID serviceUUID("6f1c0001-4b1f-4f6c-9c79-1234567890ab");
static BLEUUID charUUID("6f1c0002-4b1f-4f6c-9c79-1234567890ab");

static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEClient* pClient = nullptr;

bool doConnect = false;
bool connected = false;
bool doScan = false;

// =====================================================
// Timing
// =====================================================
unsigned long lastSampleMs = 0;
unsigned long lastDebugMs = 0;
unsigned long lastThrowSentMs = 0;

const unsigned long sampleIntervalMs = 10;   // 100 Hz
const unsigned long debugIntervalMs  = 200;
const unsigned long minResendGapMs   = 1200;

unsigned long lastScanAttemptMs = 0;
const unsigned long SCAN_RETRY_MS = 3000;
bool isScanning = false;

// =====================================================
// Filtering
// =====================================================
const float ACC_LPF_ALPHA  = 0.25f;
const float GYRO_LPF_ALPHA = 0.25f;

float filtAccMag = 1.0f;
float filtGyroMag = 0.0f;

// =====================================================
// Throw detection parameters
// Tune these after testing
// =====================================================
const float THROW_START_ACC_G   = 0.70f;   // dynamic accel threshold
const float THROW_START_GYRO_DPS= 180.0f;  // gyro threshold
const float THROW_END_ACC_G     = 0.12f;   // end threshold
const float THROW_END_GYRO_DPS  = 70.0f;   // end threshold
const unsigned long THROW_END_HOLD_MS = 180;
const unsigned long MAX_THROW_MS      = 1200;
const unsigned long MIN_THROW_MS      = 80;
unsigned long throwCooldownUntilMs = 0;
const unsigned long THROW_COOLDOWN_MS = 500;

// =====================================================
// Final MPH gating
// =====================================================
const float MIN_SENDABLE_THROW_MPH = 10.0f;

// =====================================================
// Sensor data structure
// =====================================================
struct ImuSample {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float accMag_g;
  float gyroMag_dps;
};

// =====================================================
// Throw feature state
// =====================================================
bool throwActive = false;
unsigned long throwStartMs = 0;
unsigned long throwQuietStartMs = 0;

float peakDynAcc_g = 0.0f;
float peakGyro_dps = 0.0f;
float accIntegral_gs = 0.0f;
float gyroIntegral_ds = 0.0f;

float lastComputedThrowMph = 0.0f;

// =====================================================
// Low-level I2C helpers
// =====================================================
bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool readBytes(uint8_t addr, uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

int16_t makeInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

bool probeAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// =====================================================
// MPU init
// =====================================================
bool initMPU6050() {
  uint8_t who = 0;

  if (!probeAddress(MPU_ADDR)) {
    Serial.println("No I2C response at 0x68");
    return false;
  }

  imuAddr = MPU_ADDR;

  Serial.print("MPU found at 0x");
  Serial.println(imuAddr, HEX);

  if (!readReg(imuAddr, REG_WHO_AM_I, who)) {
    Serial.println("Failed to read WHO_AM_I");
    return false;
  }

  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x68) {
    Serial.println("Unexpected WHO_AM_I");
    return false;
  }

  if (!writeReg(imuAddr, REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  if (!writeReg(imuAddr, REG_SMPLRT_DIV, 0x09)) return false;
  if (!writeReg(imuAddr, REG_CONFIG, 0x03)) return false;
  if (!writeReg(imuAddr, REG_GYRO_CONFIG, 0x00)) return false;   // ±250 dps
  if (!writeReg(imuAddr, REG_ACCEL_CONFIG, 0x00)) return false;  // ±2g
  if (!writeReg(imuAddr, REG_INT_PIN_CFG, 0x00)) return false;
  if (!writeReg(imuAddr, REG_INT_ENABLE, 0x01)) return false;

  return true;
}

// =====================================================
// Read IMU
// =====================================================
bool readImu(ImuSample &s) {
  uint8_t buf[14];
  uint8_t intStatus = 0;

  if (!readReg(imuAddr, REG_INT_STATUS, intStatus)) return false;
  if (!readBytes(imuAddr, REG_ACCEL_XOUT_H, buf, 14)) return false;

  int16_t axRaw = makeInt16(buf[0],  buf[1]);
  int16_t ayRaw = makeInt16(buf[2],  buf[3]);
  int16_t azRaw = makeInt16(buf[4],  buf[5]);
  int16_t gxRaw = makeInt16(buf[8],  buf[9]);
  int16_t gyRaw = makeInt16(buf[10], buf[11]);
  int16_t gzRaw = makeInt16(buf[12], buf[13]);

  s.ax_g = axRaw / 16384.0f;
  s.ay_g = ayRaw / 16384.0f;
  s.az_g = azRaw / 16384.0f;

  s.gx_dps = gxRaw / 131.0f;
  s.gy_dps = gyRaw / 131.0f;
  s.gz_dps = gzRaw / 131.0f;

  s.accMag_g = sqrtf(s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
  s.gyroMag_dps = sqrtf(s.gx_dps * s.gx_dps + s.gy_dps * s.gy_dps + s.gz_dps * s.gz_dps);

  return true;
}

// =====================================================
// BLE client callbacks
// =====================================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    Serial.println("Connected to display.");
  }

  void onDisconnect(BLEClient* pclient) override {
    connected = false;
    Serial.println("Disconnected from display.");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    bool hasService = advertisedDevice.haveServiceUUID() &&
                      advertisedDevice.isAdvertisingService(serviceUUID);

    if (hasService) {
      Serial.print("Found display: ");
      Serial.println(advertisedDevice.toString().c_str());

      BLEDevice::getScan()->stop();

      if (myDevice != nullptr) {
        delete myDevice;
        myDevice = nullptr;
      }

      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

bool connectToServer() {
  if (myDevice == nullptr) return false;

  Serial.println("Connecting to BLE display...");

  if (pClient != nullptr) {
    delete pClient;
    pClient = nullptr;
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("Failed to connect to display.");
    return false;
  }

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find display service.");
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find display characteristic.");
    pClient->disconnect();
    return false;
  }

  if (!pRemoteCharacteristic->canWrite()) {
    Serial.println("Display characteristic is not writable.");
    pClient->disconnect();
    return false;
  }

  connected = true;
  Serial.println("BLE display ready.");
  return true;
}

void startScan() {
  if (isScanning) return;

  Serial.println("Scanning for ThrowSense display...");
  isScanning = true;

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  isScanning = false;
  lastScanAttemptMs = millis();
}

bool sendMphToDisplay(float mph) {
  if (!connected || pRemoteCharacteristic == nullptr) return false;

  char payload[16];
  snprintf(payload, sizeof(payload), "%.1f", mph);

  try {
    pRemoteCharacteristic->writeValue((uint8_t*)payload, strlen(payload), false);
    Serial.print("Sent final throw MPH to display: ");
    Serial.println(payload);
    return true;
  } catch (...) {
    Serial.println("BLE write failed.");
    connected = false;
    return false;
  }
}

// =====================================================
// Filtering helpers
// =====================================================
float lpf(float input, float prev, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

// =====================================================
// Throw estimator
// =====================================================
float estimateThrowMph(float peakDynAcc, float peakGyro, float accIntegral, float durationSec) {
  float mph =
      10.0f * peakDynAcc +
      0.035f * peakGyro +
      8.0f  * accIntegral +
      3.0f  * durationSec;

  if (mph < 0.0f) mph = 0.0f;
  if (mph > 80.0f) mph = 80.0f;
  return mph;
}

void resetThrowState() {
  throwActive = false;
  throwStartMs = 0;
  throwQuietStartMs = 0;
  peakDynAcc_g = 0.0f;
  peakGyro_dps = 0.0f;
  accIntegral_gs = 0.0f;
  gyroIntegral_ds = 0.0f;
}

void beginThrow(unsigned long now) {
  throwActive = true;
  throwStartMs = now;
  throwQuietStartMs = 0;
  peakDynAcc_g = 0.0f;
  peakGyro_dps = 0.0f;
  accIntegral_gs = 0.0f;
  gyroIntegral_ds = 0.0f;
  Serial.println("Throw started.");
}

void finishThrow(unsigned long now) {
  throwCooldownUntilMs = now + THROW_COOLDOWN_MS;
  float durationSec = (now - throwStartMs) / 1000.0f;
  if (durationSec < 0.001f) durationSec = 0.001f;

  if ((now - throwStartMs) < MIN_THROW_MS) {
    Serial.println("Throw discarded: too short.");
    resetThrowState();
    return;
  }

  float mph = estimateThrowMph(peakDynAcc_g, peakGyro_dps, accIntegral_gs, durationSec);
  lastComputedThrowMph = mph;

  Serial.println("Throw finished.");
  Serial.print("  durationSec: "); Serial.println(durationSec, 3);
  Serial.print("  peakDynAcc_g: "); Serial.println(peakDynAcc_g, 3);
  Serial.print("  peakGyro_dps: "); Serial.println(peakGyro_dps, 1);
  Serial.print("  accIntegral_gs: "); Serial.println(accIntegral_gs, 3);
  Serial.print("  gyroIntegral_ds: "); Serial.println(gyroIntegral_ds, 1);
  Serial.print("  estimated MPH: "); Serial.println(mph, 1);

  if (connected && mph >= MIN_SENDABLE_THROW_MPH && (now - lastThrowSentMs >= minResendGapMs)) {
    if (sendMphToDisplay(mph)) {
      lastThrowSentMs = now;
    }
  } else {
    if (!connected) Serial.println("Not sent: display not connected.");
    else if (mph < MIN_SENDABLE_THROW_MPH) Serial.println("Not sent: throw MPH below send threshold.");
  }

  resetThrowState();
}

void processThrowPipeline(const ImuSample &sample, float dtSec, unsigned long now) {
  // 1) Filter magnitudes
  filtAccMag = lpf(sample.accMag_g, filtAccMag, ACC_LPF_ALPHA);
  filtGyroMag = lpf(sample.gyroMag_dps, filtGyroMag, GYRO_LPF_ALPHA);

  // 2) Compute dynamic accel relative to ~1g baseline
  float dynAcc_g = fabsf(filtAccMag - 1.0f);

  // 3) Debug
  if (now - lastDebugMs >= debugIntervalMs) {
    lastDebugMs = now;
    Serial.print("dynAcc_g: ");
    Serial.print(dynAcc_g, 3);
    Serial.print("  filtGyro_dps: ");
    Serial.print(filtGyroMag, 1);
    Serial.print("  throwActive: ");
    Serial.println(throwActive ? "yes" : "no");
  }

  if (!throwActive && now < throwCooldownUntilMs) {
    return;
  }

  // 4) Detect start
  bool startCondition = (dynAcc_g > THROW_START_ACC_G) || (filtGyroMag > THROW_START_GYRO_DPS);

  if (!throwActive && startCondition) {
    beginThrow(now);
  }

  if (!throwActive) return;

  // 5) Accumulate throw features
  if (dynAcc_g > peakDynAcc_g) peakDynAcc_g = dynAcc_g;
  if (filtGyroMag > peakGyro_dps) peakGyro_dps = filtGyroMag;

  accIntegral_gs += dynAcc_g * dtSec;
  gyroIntegral_ds += filtGyroMag * dtSec;

  // 6) Detect end
  bool quietCondition = (dynAcc_g < THROW_END_ACC_G) && (filtGyroMag < THROW_END_GYRO_DPS);

  if (quietCondition) {
    if (throwQuietStartMs == 0) {
      throwQuietStartMs = now;
    } else if (now - throwQuietStartMs >= THROW_END_HOLD_MS) {
      finishThrow(now);
      return;
    }
  } else {
    throwQuietStartMs = 0;
  }

  // 7) Safety max duration
  if (now - throwStartMs >= MAX_THROW_MS) {
    Serial.println("Throw ended by max duration timeout.");
    finishThrow(now);
    return;
  }
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\nMPU-6050 BLE throw pipeline starting...");

  pinMode(IMU_INT, INPUT);

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);

  if (!initMPU6050()) {
    Serial.println("MPU init failed.");
    Serial.println("Check: SDA/SCL pins, power, GND, AD0 wiring, and solder joints.");
    while (true) delay(1000);
  }

  Serial.println("MPU init OK.");

  BLEDevice::init("ThrowSenseWrist");
  startScan();
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  // BLE reconnect flow
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to ThrowSenseDisplay.");
    } else {
      Serial.println("Connection failed. Will rescan.");
      doScan = true;
    }
    doConnect = false;
  }

  if (!connected && (doScan || millis() - lastScanAttemptMs >= SCAN_RETRY_MS)) {
    doScan = false;
    startScan();
  }

  // Sensor sampling / throw pipeline
  unsigned long now = millis();
  static ImuSample sample;

  if (now - lastSampleMs >= sampleIntervalMs) {
    float dtSec = (now - lastSampleMs) / 1000.0f;
    lastSampleMs = now;

    if (!readImu(sample)) {
      Serial.println("Sensor read failed");
      delay(20);
      return;
    }

    processThrowPipeline(sample, dtSec, now);
  }
}