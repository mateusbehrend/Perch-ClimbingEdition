#include <Arduino_LSM6DSOX.h>
#include <MadgwickAHRS.h>

// =============================================================
//  Madgwick-based Hip Drop Detector for Climbing
//  Device is hip-mounted on a belt:
//    Z-axis → straight out from the climber (forward)
//    X-axis → horizontal (left/right)
//    Y-axis → vertical (up/down)
// =============================================================

// --- Madgwick Filter ---
Madgwick filter;
const float SAMPLE_RATE = 50.0;  // Hz (match your loop timing)

// --- Timing ---
unsigned long lastTime = 0;

// --- Orientation from filter ---
float roll  = 0.0;  // rotation around X-axis (degrees)
float pitch = 0.0;  // rotation around Y-axis (degrees)
float yaw   = 0.0;  // rotation around Z-axis (degrees)

// --- Calibration ---
float baselineRoll  = 0.0;
float baselinePitch = 0.0;
bool  calibrated    = false;

const int CAL_SAMPLES = 100;  // ~2 seconds at 50 Hz

// --- Hip Drop Detection (rolling-window debounce) ---
const float ALERT_THRESHOLD = 20.0;  // degrees of deviation

// Rolling window: 25 samples at 50 Hz = 0.5 seconds
// Trigger when >= 20 of the last 25 samples exceed threshold (~80%)
const int DEBOUNCE_WINDOW        = 25;
const int DEBOUNCE_TRIGGER_COUNT = 20;

bool  sampleBuffer[DEBOUNCE_WINDOW];  // circular buffer (true = above threshold)
int   bufferIndex    = 0;              // current write position
int   aboveCount     = 0;              // running count of 'true' entries
bool  hipDropActive  = false;          // latched alert state

// =============================================================
//  Calibration: average orientation over CAL_SAMPLES readings
// =============================================================
void runCalibration() {
  Serial.println(">> Calibration started. Hold good position...");
  delay(1000);

  float sumRoll = 0.0, sumPitch = 0.0;
  int   count   = 0;

  while (count < CAL_SAMPLES) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      float ax, ay, az, gx, gy, gz;
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      // Feed the Madgwick filter (6-DOF, no magnetometer)
      filter.updateIMU(gx, gy, gz, ax, ay, az);

      sumRoll  += filter.getRoll();
      sumPitch += filter.getPitch();
      count++;

      delay(20);  // ~50 Hz
    }
  }

  baselineRoll  = sumRoll  / CAL_SAMPLES;
  baselinePitch = sumPitch / CAL_SAMPLES;
  calibrated    = true;

  Serial.print(">> Calibration complete!  Baseline Roll: ");
  Serial.print(baselineRoll, 1);
  Serial.print("°  |  Baseline Pitch: ");
  Serial.print(baselinePitch, 1);
  Serial.println("°");
}

// =============================================================
//  Setup
// =============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized (Madgwick filter).");

  filter.begin(SAMPLE_RATE);
  lastTime = micros();

  // run calibration once during setup
  runCalibration();
}

// =============================================================
//  Main loop
// =============================================================
void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

    // --- Read sensors ---
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // --- Update Madgwick filter ---
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll  = filter.getRoll();
    pitch = filter.getPitch();
    yaw   = filter.getYaw();

    // --- Output orientation ---
    Serial.print("Roll: ");
    Serial.print(roll, 1);
    Serial.print("°  |  Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("°  |  Yaw: ");
    Serial.print(yaw, 1);
    Serial.print("°");

    // --- Hip drop detection with rolling-window debounce ---
    if (calibrated) {
      float deltaRoll  = roll  - baselineRoll;
      float deltaPitch = pitch - baselinePitch;
      float deviation  = sqrt(deltaRoll * deltaRoll + deltaPitch * deltaPitch);

      Serial.print("  |  Dev: ");
      Serial.print(deviation, 1);
      Serial.print("°");

      // Determine if this sample is above threshold
      bool aboveNow = (deviation > ALERT_THRESHOLD);

      // Update the rolling window: subtract the old sample, add the new one
      if (sampleBuffer[bufferIndex]) aboveCount--;
      sampleBuffer[bufferIndex] = aboveNow;
      if (aboveNow) aboveCount++;
      bufferIndex = (bufferIndex + 1) % DEBOUNCE_WINDOW;

      // Trigger alert when enough samples in the window exceeded threshold
      if (aboveCount >= DEBOUNCE_TRIGGER_COUNT) {
        hipDropActive = true;
        Serial.print("  *** HIP DROP ALERT ***");
      } else if (hipDropActive) {
        // Cleared: majority of window is now below threshold
        hipDropActive = false;
        Serial.print("  (hip drop cleared)");
      } else {
        Serial.print("  (no hip drop)");
      }
    }

    Serial.println();
    delay(20);  // ~50 Hz to match SAMPLE_RATE
  }

  delay(1);
}