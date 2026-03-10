#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include "MadgwickFilter.h"

// =============================================================
//  Madgwick-based Hip Drop Detector for Climbing (with BLE)
//  Device is hip-mounted on a belt:
//    Z-axis → straight out from the climber (forward)
//    X-axis → horizontal (left/right)
//    Y-axis → vertical (up/down)
// =============================================================

// --- BLE UUIDs ---
#define BLE_UUID_IMU_SERVICE   "1101"
#define BLE_UUID_ROLL          "2101"
#define BLE_UUID_PITCH         "2102"
#define BLE_UUID_DEVIATION     "2103"
#define BLE_UUID_ALERT         "2104"

#define BLE_DEVICE_NAME "Elfo"
#define BLE_LOCAL_NAME  "Elfo"

BLEService imuService(BLE_UUID_IMU_SERVICE);
BLEFloatCharacteristic rollCharacteristic(BLE_UUID_ROLL, BLERead | BLENotify);
BLEFloatCharacteristic pitchCharacteristic(BLE_UUID_PITCH, BLERead | BLENotify);
BLEFloatCharacteristic deviationCharacteristic(BLE_UUID_DEVIATION, BLERead | BLENotify);
BLEBoolCharacteristic  alertCharacteristic(BLE_UUID_ALERT, BLERead | BLENotify);

// --- Madgwick Filter ---
MadgwickFilter filter;
const float SAMPLE_RATE = 100.0;  // Hz (match your loop timing)

// --- Timing ---
unsigned long lastTime = 0;
unsigned long lastBleUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL_MS = 200; // 5 Hz BLE updates

// --- Orientation from filter ---
float roll  = 0.0;
float pitch = 0.0;
float yaw   = 0.0;

// --- Calibration ---
float baselineRoll  = 0.0;
float baselinePitch = 0.0;
bool  calibrated    = false;

const int CAL_SAMPLES = 200;  // ~2 seconds at 100 Hz

// --- Hip Drop Detection (rolling-window debounce) ---
const float ALERT_THRESHOLD = 20.0;  // degrees of deviation

// Rolling window: 50 samples at 100 Hz = 0.5 seconds
// Trigger when >= 40 of the last 50 samples exceed threshold (~80%)
const int DEBOUNCE_WINDOW        = 50;
const int DEBOUNCE_TRIGGER_COUNT = 40;

bool  sampleBuffer[DEBOUNCE_WINDOW];  // circular buffer (true = above threshold)
int   bufferIndex    = 0;              // current write position
int   aboveCount     = 0;              // running count of 'true' entries
bool  hipDropActive  = false;          // latched alert state

const unsigned long MIN_ALERT_HOLD_MS = 250;  // alert stays on for at least 0.25 seconds
unsigned long alertStartTime = 0;               // when alert was first triggered

// =============================================================
//  Calibration
// =============================================================
void runCalibration() {
  Serial.println(">> Calibration started. Hold good position...");
  delay(1000);

  // --- Compute baseline from accelerometer (instant, any angle) ---
  Serial.println(">> Capturing baseline from accelerometer...");
  float sumRoll = 0.0, sumPitch = 0.0;
  int count = 0;

  while (count < CAL_SAMPLES) {
    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);

      sumRoll  += atan2(ay, az) * RAD_TO_DEG;
      sumPitch += atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
      count++;

      delay(10);
    }
  }

  baselineRoll  = sumRoll  / CAL_SAMPLES;
  baselinePitch = sumPitch / CAL_SAMPLES;
  calibrated    = true;

  Serial.print(">> Baseline Roll: ");
  Serial.print(baselineRoll, 1);
  Serial.print("°  |  Baseline Pitch: ");
  Serial.print(baselinePitch, 1);
  Serial.println("°");

  // --- Pre-seed the Madgwick filter so it starts at the right angle ---
  Serial.println(">> Seeding and stabilizing filter...");
  while (!IMU.accelerationAvailable()) { delay(1); }
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  filter.begin(SAMPLE_RATE);
  filter.setFromAccelerometer(ax, ay, az);

  // Brief stabilization (~1 second)
  int warmup = 0;
  while (warmup < 100) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      warmup++;
      delay(10);
    }
  }

  Serial.println(">> Calibration complete. Ready.");
}

// =============================================================
//  Setup
// =============================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized (Madgwick filter).");

  // --- Initialize BLE ---
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1);
  }

  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(rollCharacteristic);
  imuService.addCharacteristic(pitchCharacteristic);
  imuService.addCharacteristic(deviationCharacteristic);
  imuService.addCharacteristic(alertCharacteristic);
  BLE.addService(imuService);

  rollCharacteristic.writeValue(0.0f);
  pitchCharacteristic.writeValue(0.0f);
  deviationCharacteristic.writeValue(0.0f);
  alertCharacteristic.writeValue(false);

  BLE.advertise();
  Serial.println("BLE advertising, waiting for connections...");

  lastTime = micros();

  // Run calibration once at startup
  runCalibration();
}

// =============================================================
//  Main loop
// =============================================================
void loop() {
  BLE.poll();

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

    // --- Read sensors ---
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // --- Update Madgwick filter at full rate (100 Hz) ---
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll  = filter.getRoll();
    pitch = filter.getPitch();
    yaw   = filter.getYaw();

    // --- Hip drop detection with rolling-window debounce (100 Hz) ---
    float deviation = 0.0;
    if (calibrated) {
      float deltaRoll  = roll  - baselineRoll;
      float deltaPitch = pitch - baselinePitch;
      deviation = sqrt(deltaRoll * deltaRoll + deltaPitch * deltaPitch);

      // Determine if this sample is above threshold
      bool aboveNow = (deviation > ALERT_THRESHOLD);

      // Update the rolling window: subtract the old sample, add the new one
      if (sampleBuffer[bufferIndex]) aboveCount--;
      sampleBuffer[bufferIndex] = aboveNow;
      if (aboveNow) aboveCount++;
      bufferIndex = (bufferIndex + 1) % DEBOUNCE_WINDOW;

      // Trigger/clear alert with minimum hold time
      if (aboveCount >= DEBOUNCE_TRIGGER_COUNT) {
        if (!hipDropActive) {
          alertStartTime = millis();  // record when alert first triggered
        }
        hipDropActive = true;
      } else if (hipDropActive) {
        // Only clear if the minimum hold time has elapsed
        if (millis() - alertStartTime >= MIN_ALERT_HOLD_MS) {
          hipDropActive = false;
        }
      }
    }

    // --- Throttled BLE + Serial output at ~5 Hz ---
    unsigned long nowMs = millis();
    if (nowMs - lastBleUpdate >= BLE_UPDATE_INTERVAL_MS) {
      lastBleUpdate = nowMs;

      // Write BLE characteristics
      rollCharacteristic.writeValue(roll);
      pitchCharacteristic.writeValue(pitch);
      deviationCharacteristic.writeValue(deviation);
      alertCharacteristic.writeValue(hipDropActive);

      // Serial output
      Serial.print("Roll: ");
      Serial.print(roll, 1);
      Serial.print("°  |  Pitch: ");
      Serial.print(pitch, 1);
      Serial.print("°  |  Yaw: ");
      Serial.print(yaw, 1);
      Serial.print("°");

      if (calibrated) {
        Serial.print("  |  Dev: ");
        Serial.print(deviation, 1);
        Serial.print("°");

        if (hipDropActive) {
          Serial.print("  *** HIP DROP ALERT ***");
          digitalWrite(LED_BUILTIN, HIGH);
        } else {
          Serial.print("  (no hip drop)");
          digitalWrite(LED_BUILTIN, LOW);
        }
      }

      Serial.println();
    }
  }

  delay(1);
}
