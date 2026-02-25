#include <Arduino_LSM6DSOX.h>

// --- Complementary Filter Coefficient ---
// 0.98 = trust gyro 98%, accel 2% (tune to your needs)
const float alpha = 0.98;

// --- Timing ---
float imu_dt = 0.0;
unsigned long lastTime = 0;

// --- Angles ---
float roll  = 0.0;  // rotation around X-axis (degrees)
float pitch = 0.0;  // rotation around Y-axis (degrees)

// --- Calibration ---
float baselineRoll  = 0.0;
float basePitch     = 0.0;
bool  calibrated    = false;

const int   CAL_SAMPLES  = 100;   // number of samples to average (~2 seconds)
const float ALERT_THRESHOLD = 20.0; // degrees of deviation before alert

// --- Run calibration: average CAL_SAMPLES readings ---
void runCalibration() {
  Serial.println(">> Calibration started. Hold good position...");
  delay(1000); // give the climber a moment to get set

  float sumRoll = 0.0, sumPitch = 0.0;
  int   count   = 0;

  while (count < CAL_SAMPLES) {
    float ax, ay, az, gx, gy, gz;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      unsigned long now = micros();
      imu_dt = (now - lastTime) / 1000000.0;
      lastTime = now;

      // Accelerometer angle estimates
      float accelRoll  = atan2(ay, az) * RAD_TO_DEG;
      float accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

      // Complementary filter
      roll  = alpha * (roll  + gx * imu_dt) + (1.0 - alpha) * accelRoll;
      pitch = alpha * (pitch + gy * imu_dt) + (1.0 - alpha) * accelPitch;

      sumRoll  += roll;
      sumPitch += pitch;
      count++;

      delay(20); // ~50Hz during calibration
    }
  }

  baselineRoll = sumRoll  / CAL_SAMPLES;
  basePitch    = sumPitch / CAL_SAMPLES;
  calibrated   = true;

  Serial.print(">> Calibration complete! Baseline Roll: ");
  Serial.print(baselineRoll, 1);
  Serial.print("°  |  Baseline Pitch: ");
  Serial.print(basePitch, 1);
  Serial.println("°");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized.");

  lastTime = micros();

  // Run calibration once at startup
  runCalibration();
}

void loop() {
  float ax, ay, az;   // Accelerometer (g)
  float gx, gy, gz;   // Gyroscope (degrees/sec)

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

    // --- Read sensors ---
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // --- Compute delta time in seconds ---
    unsigned long now = micros();
    imu_dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // --- Accelerometer-based angle estimates ---
    float accelRoll  = atan2(ay, az) * RAD_TO_DEG;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // --- Complementary Filter ---
    roll  = alpha * (roll  + gx * imu_dt) + (1.0 - alpha) * accelRoll;
    pitch = alpha * (pitch + gy * imu_dt) + (1.0 - alpha) * accelPitch;

    // --- Output to Serial ---
    Serial.print("Roll: ");
    Serial.print(roll, 1);
    Serial.print("°  |  Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("°");

    // --- Deviation from baseline (if calibrated) ---
    if (calibrated) {
      float deltaRoll  = roll  - baselineRoll;
      float deltaPitch = pitch - basePitch;
      float deviation  = sqrt(deltaRoll * deltaRoll + deltaPitch * deltaPitch);

      Serial.print("  |  Deviation: ");
      Serial.print(deviation, 1);
      Serial.print("°");

      if (deviation > ALERT_THRESHOLD) {
        Serial.print("  *** HIP DROP ALERT ***");
      }
    }

    Serial.println();
    delay(200); // 5 updates per second
  }

  delay(10);
}