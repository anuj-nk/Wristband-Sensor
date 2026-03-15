#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// -------------------------
// Pin definitions
// -------------------------
const int IN1 = D2;
const int IN2 = D1;
const int IN3 = D0;
const int IN4 = D7;

const int BUTTON_PIN = D4;
const int LED_PIN    = D6;

// -------------------------
// BLE UUIDs
// -------------------------
// Keep these the same on the sender/client side.
#define DEVICE_NAME         "ThrowSenseDisplay"
#define SERVICE_UUID        "6f1c0001-4b1f-4f6c-9c79-1234567890ab"
#define CHARACTERISTIC_UUID "6f1c0002-4b1f-4f6c-9c79-1234567890ab"

// -------------------------
// BLE globals
// -------------------------
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// When BLE data arrives, loop() will process it
bool newMphPending = false;
float pendingMph = 0.0f;

// -------------------------
// Stepper sequence
// -------------------------
const uint8_t seq[4][4] = {
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1}
};

// -------------------------
// Calibration settings
// -------------------------
const int NUM_ANCHORS = 9;
const int anchorMPH[NUM_ANCHORS] = {0, 10, 20, 30, 40, 50, 60, 70, 80};

long anchorSteps[NUM_ANCHORS] = {0, -33, -76, -114, -157, -204, -251, -279, -314};
bool anchorSet[NUM_ANCHORS]   = {true, true, true, true, true, true, true, true, true};

// -------------------------
// Stepper state
// -------------------------
long currentStep = 0;
int currentSeqIdx = 0;

// -------------------------
// LED/UI state
// -------------------------
bool displayedValueReady = false;
bool motorMoving = false;
bool ledState = false;
unsigned long lastBlinkMs = 0;
unsigned long rxBlinkUntilMs = 0;
float displayedMph = 0.0f;         // what the needle is currently showing
const float MIN_VALID_MPH = 15.0f; // ignore values below this
const float MPH_HYSTERESIS = 3.0f; // require this much increase before moving

// -------------------------
// Button state
// -------------------------
bool lastButtonReading = HIGH;
bool buttonPressed = false;
unsigned long buttonPressStart = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceMs = 30;
const unsigned long longPressMs = 1200;

// -------------------------
// BLE callbacks
// -------------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() == 0) return;

    String s = String(rxValue.c_str());
    s.trim();

    float mph = s.toFloat();

    Serial.print("BLE write received MPH: ");
    Serial.println(mph, 1);

    pendingMph = mph;
    newMphPending = true;
    rxBlinkUntilMs = millis() + 300;
  }
};

// -------------------------
// Stepper helpers
// -------------------------
void setStep(int a, int b, int c, int d) {
  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
  digitalWrite(IN3, c);
  digitalWrite(IN4, d);
}

void applyStep(int idx) {
  idx = (idx + 4) % 4;
  setStep(seq[idx][0], seq[idx][1], seq[idx][2], seq[idx][3]);
}

void allOff() {
  setStep(0, 0, 0, 0);
}

// -------------------------
// LED helpers
// -------------------------
void ledOff() {
  digitalWrite(LED_PIN, LOW);
}

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void blinkLED(unsigned long intervalMs = 100) {
  unsigned long now = millis();
  if (now - lastBlinkMs >= intervalMs) {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

// -------------------------
// Calibration helpers
// -------------------------
bool allAnchorsSet() {
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (!anchorSet[i]) return false;
  }
  return true;
}

void printCalibration() {
  Serial.println("Calibration table:");
  for (int i = 0; i < NUM_ANCHORS; i++) {
    Serial.print("  ");
    Serial.print(anchorMPH[i]);
    Serial.print(" mph -> ");
    Serial.println(anchorSteps[i]);
  }
}

// -------------------------
// Stepper motion
// -------------------------
void stepMotor(int dir, int delayMs = 8) {
  currentSeqIdx = (currentSeqIdx + dir + 4) % 4;
  applyStep(currentSeqIdx);
  delay(delayMs);
  currentStep += dir;
}

void moveRelative(long deltaSteps, int delayMs = 8) {
  if (deltaSteps == 0) {
    motorMoving = false;
    displayedValueReady = true;
    ledOn();
    return;
  }

  motorMoving = true;
  displayedValueReady = false;

  long stepsToMove = labs(deltaSteps);
  int dir = (deltaSteps > 0) ? +1 : -1;

  for (long i = 0; i < stepsToMove; i++) {
    blinkLED(60);
    stepMotor(dir, delayMs);
  }

  allOff();
  motorMoving = false;
  displayedValueReady = true;
  ledOn();
}

void moveToStep(long targetStep, int delayMs = 8) {
  long delta = targetStep - currentStep;
  Serial.print("Moving from step ");
  Serial.print(currentStep);
  Serial.print(" to ");
  Serial.println(targetStep);
  moveRelative(delta, delayMs);
}

// -------------------------
// MPH interpolation
// -------------------------
bool getStepForMPH(float mph, long &targetStep) {
  if (!allAnchorsSet()) return false;

  if (mph <= anchorMPH[0]) {
    targetStep = anchorSteps[0];
    return true;
  }

  if (mph >= anchorMPH[NUM_ANCHORS - 1]) {
    targetStep = anchorSteps[NUM_ANCHORS - 1];
    return true;
  }

  for (int i = 0; i < NUM_ANCHORS - 1; i++) {
    if (mph >= anchorMPH[i] && mph <= anchorMPH[i + 1]) {
      float mph0 = anchorMPH[i];
      float mph1 = anchorMPH[i + 1];
      long s0 = anchorSteps[i];
      long s1 = anchorSteps[i + 1];

      float t = (mph - mph0) / (mph1 - mph0);
      targetStep = lround(s0 + t * (s1 - s0));
      return true;
    }
  }

  return false;
}

void goToMPH(float mph) {
  long targetStep;
  if (!getStepForMPH(mph, targetStep)) {
    Serial.println("Cannot move to MPH: calibration incomplete.");
    return;
  }

  Serial.print("MPH ");
  Serial.print(mph, 1);
  Serial.print(" -> target step ");
  Serial.println(targetStep);

  moveToStep(targetStep, 8);
  displayedMph = mph;
}

void handleIncomingMph(float mph) {
  Serial.print("Detected MPH from wristband: ");
  Serial.println(mph, 1);

  // Ignore tiny/noisy values
  if (mph < MIN_VALID_MPH) {
    Serial.println("Ignored: below minimum threshold.");
    return;
  }

  // Ignore very small changes to avoid jitter
  float diff = fabs(mph - displayedMph);
  if (diff < MPH_HYSTERESIS) {
    Serial.print("Ignored: change too small. Current displayed MPH = ");
    Serial.print(displayedMph, 1);
    Serial.print(", diff = ");
    Serial.println(diff, 1);
    return;
  }

  Serial.print("Accepted MPH: ");
  Serial.println(mph, 1);

  goToMPH(mph);
}
// -------------------------
// Button handling
// -------------------------
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceMs) {
    if (reading == LOW && !buttonPressed) {
      buttonPressed = true;
      buttonPressStart = millis();
    }

    if (reading == HIGH && buttonPressed) {
      unsigned long pressDuration = millis() - buttonPressStart;
      buttonPressed = false;

      if (pressDuration >= longPressMs) {
        Serial.println("Long press: moving to 0 MPH and resetting logical zero.");
        goToMPH(0);
        currentStep = 0;
        displayedMph = 0.0f;
      } else {
        currentStep = 0;
        displayedMph = 0.0f;
        displayedValueReady = true;
        ledOn();
        Serial.println("Short press: logical zero and displayed MPH reset.");
      }
    }
  }

  lastButtonReading = reading;
}

// -------------------------
// Serial command handling
// -------------------------
void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  f <steps>     move forward");
  Serial.println("  b <steps>     move backward");
  Serial.println("  zero          set logical zero");
  Serial.println("  go <mph>      move needle to mph");
  Serial.println("  ble <mph>     simulate BLE write");
  Serial.println("  print         print calibration table");
  Serial.println("  help          show commands");
}

void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  if (cmd.equalsIgnoreCase("zero")) {
    currentStep = 0;
    displayedValueReady = true;
    ledOn();
    Serial.println("Logical zero set.");
    return;
  }

  if (cmd.equalsIgnoreCase("print")) {
    printCalibration();
    return;
  }

  if (cmd.startsWith("f ")) {
    int steps = cmd.substring(2).toInt();
    Serial.print("Forward ");
    Serial.println(steps);
    moveRelative(steps, 8);
    return;
  }

  if (cmd.startsWith("b ")) {
    int steps = cmd.substring(2).toInt();
    Serial.print("Backward ");
    Serial.println(steps);
    moveRelative(-steps, 8);
    return;
  }

  if (cmd.startsWith("go ")) {
    float mph = cmd.substring(3).toFloat();
    goToMPH(mph);
    return;
  }

  if (cmd.startsWith("ble ")) {
    float mph = cmd.substring(4).toFloat();
    Serial.print("Simulated BLE MPH: ");
    Serial.println(mph, 1);
    pendingMph = mph;
    newMphPending = true;
    rxBlinkUntilMs = millis() + 300;
    return;
  }

  Serial.println("Unknown command.");
  printHelp();
}

// -------------------------
// BLE setup
// -------------------------
void setupBLE() {
  BLEDevice::init(DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("0");
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started.");
  Serial.print("Device name: ");
  Serial.println(DEVICE_NAME);
}

// -------------------------
// LED state machine
// -------------------------
void updateLED() {
  if (motorMoving) {
    blinkLED(60);
    return;
  }

  if (millis() < rxBlinkUntilMs) {
    blinkLED(40);
    return;
  }

  if (displayedValueReady) {
    ledOn();
  } else {
    ledOff();
  }
}

// -------------------------
// Setup / loop
// -------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  allOff();
  ledOff();

  Serial.println("ThrowSense display ready.");
  printCalibration();
  printHelp();

  setupBLE();
}

void loop() {
  handleButton();
  handleSerial();

  if (newMphPending) {
    float mph = pendingMph;
    newMphPending = false;
    handleIncomingMph(mph);
  }

  // reconnect advertising if client disconnects
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  updateLED();
}