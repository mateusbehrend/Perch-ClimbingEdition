# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Perch-ClimbingEdition is a climbing safety monitoring system that uses an Arduino Nano 33 BLE Sense with IMU (LSM6DSOX) to detect "hip drops" - a warning sign of poor climbing posture. The system consists of:

1. **Arduino firmware** (`imu/`) - Runs on hip-mounted device, detects orientation changes via Madgwick filter
2. **Node.js BLE central** (`perch/central.js`) - Receives sensor data via Bluetooth Low Energy
3. **Web dashboard** (`perch/views/index.ejs`) - Real-time visualization of roll/pitch data

## Architecture

### IMU Firmware (`imu/imu.ino`)

The active firmware uses a **Madgwick AHRS filter** for orientation estimation:

- **Calibration**: Captures baseline roll/pitch by averaging 200 accelerometer samples (~2s at 100Hz)
- **Detection**: Uses rolling window debouncing (50 samples, trigger at 80% above threshold) to detect sustained hip drops (>20° deviation from baseline)
- **Output**: Serial prints roll/pitch/deviation at 100Hz (~10ms loop delay)

**Deprecated version** (`imu/peripheral_roll_pitch`): Uses complementary filter instead of Madgwick, includes BLE peripheral code with 4 characteristics (roll, pitch, deviation, alert).

### BLE Communication

**Service UUID**: `1101`
**Characteristics**:
- `2101` - Roll (float32, degrees)
- `2102` - Pitch (float32, degrees)
- `2103` - Deviation (float32, degrees) [deprecated firmware only]
- `2104` - Alert (boolean) [deprecated firmware only]

**Note**: Current `imu.ino` does NOT include BLE - it's serial-only. The BLE code is in the deprecated `peripheral_roll_pitch` file.

### Node.js Central (`perch/central.js`)

- Uses `@abandonware/noble` for BLE scanning/connection
- Polls roll/pitch characteristics at ~5Hz (200ms interval)
- Exposes Express server on port 3000 with:
  - `GET /` - Serves dashboard
  - `GET /api/sensor` - Returns `{ attitude: { roll, pitch } }`
  - `POST /` - Also returns sensor data (legacy endpoint)

### Web Dashboard (`perch/views/index.ejs`)

- EJS template with Chart.js for real-time line graphs
- Polls `/api/sensor` every 50ms
- Displays last 120 samples of roll/pitch data
- Dark theme optimized for monitoring

## Common Commands

### Arduino Development

**Upload firmware to Arduino Nano 33 BLE Sense**:
```bash
# Use Arduino IDE or CLI
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble imu/
arduino-cli upload -p /dev/tty.usbmodem* --fqbn arduino:mbed_nano:nano33ble imu/
```

**Monitor serial output**:
```bash
screen /dev/tty.usbmodem* 115200
# or
arduino-cli monitor -p /dev/tty.usbmodem*
```

### Node.js Server

**Install dependencies**:
```bash
cd perch && npm install
```

**Run BLE central + web server**:
```bash
cd perch && node central.js
```

**Access dashboard**:
```
http://localhost:3000
```

## Important Implementation Notes

### Coordinate System (Hip-Mounted Device)

From `imu.ino` comments:
- **Z-axis**: Forward (straight out from climber)
- **X-axis**: Horizontal (left/right)
- **Y-axis**: Vertical (up/down)

### Filter Comparison

**Madgwick filter** (current `imu.ino`):
- Smooth orientation for display
- Calibration uses raw accelerometer angles (instant, no convergence delay)
- Detection also uses raw accelerometer angles for consistency

**Complementary filter** (deprecated `peripheral_roll_pitch`):
- `alpha = 0.98` weighting
- Simpler but drifts more than Madgwick

### BLE Considerations

- Noble requires platform-specific Bluetooth stack (works on macOS/Linux/Windows with different backends)
- On macOS, may need Xcode command-line tools for native module compilation
- BLE connection can be flaky - the dashboard shows "Connection lost" if polling fails

## Key Calibration Flow

1. Device powers on → waits for serial connection
2. Runs `runCalibration()`: averages 200 accelerometer samples to establish baseline
3. Continuously monitors deviation from baseline
4. Triggers alert when 40+ of last 50 samples exceed 20° deviation

## File Structure

```
imu/
  imu.ino                    # Active Madgwick-based firmware (serial only)
  peripheral_roll_pitch      # Deprecated complementary filter firmware (with BLE)

perch/
  central.js                 # BLE central + Express server
  package.json               # Dependencies: noble, express, ejs
  views/
    index.ejs                # Dashboard UI with Chart.js
```
