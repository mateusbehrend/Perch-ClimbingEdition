// =============================================================
//  MadgwickFilter.h — Custom Madgwick AHRS with initial seeding
//  Based on Sebastian Madgwick's open algorithm.
//  Place this file in the same folder as your .ino sketch.
// =============================================================
#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <math.h>

class MadgwickFilter {
public:
  float beta;

  MadgwickFilter() : beta(0.2f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), invSampleFreq(0.01f) {}

  void begin(float sampleFrequency) {
    invSampleFreq = 1.0f / sampleFrequency;
  }

  // Pre-seed the quaternion from accelerometer (gravity) data.
  // Sets the initial orientation instantly — no convergence needed.
  // Yaw is set to 0 (accelerometer can't determine heading).
  void setFromAccelerometer(float ax, float ay, float az) {
    float rollRad  = atan2f(ay, az);
    float pitchRad = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(rollRad  * 0.5f);
    float sr = sinf(rollRad  * 0.5f);
    float cp = cosf(pitchRad * 0.5f);
    float sp = sinf(pitchRad * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
  }

  // 6-DOF update (accelerometer + gyroscope, no magnetometer)
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
    float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      _2q0 = 2.0f * q0;  _2q1 = 2.0f * q1;  _2q2 = 2.0f * q2;  _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;  _4q1 = 4.0f * q1;  _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;  _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;  q1q1 = q1 * q1;  q2q2 = q2 * q2;  q3q3 = q3 * q3;

      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;  s1 *= recipNorm;  s2 *= recipNorm;  s3 *= recipNorm;

      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;  q1 *= recipNorm;  q2 *= recipNorm;  q3 *= recipNorm;
  }

  float getRoll() {
    return atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29578f;
  }

  float getPitch() {
    float arg = 2.0f * (q0 * q2 - q3 * q1);
    if (arg > 1.0f) arg = 1.0f;
    if (arg < -1.0f) arg = -1.0f;
    return asinf(arg) * 57.29578f;
  }

  float getYaw() {
    return atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29578f;
  }

private:
  float q0, q1, q2, q3;
  float invSampleFreq;
};

#endif
