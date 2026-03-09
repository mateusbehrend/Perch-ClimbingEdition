# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Perch-ClimbingEdition is a climbing safety monitoring system that uses an Arduino Nano 33 BLE Sense with IMU (LSM6DSOX) to detect "hip drops" - a warning sign of poor climbing posture. The system consists of:

1. **Arduino firmware** (`imu/`) - Runs on hip-mounted device, detects orientation changes via Madgwick filter
2. **Node.js BLE central** (`perch/central.js`) - Receives sensor data via Bluetooth Low Energy
3. **Web dashboard** (`perch/views/index.ejs`) - Real-time visualization of roll/pitch data

## Architecture

### IMU Firmware (`imu/imu.ino`)

The active firmware uses a **custom Madgwick AHRS filter** (`MadgwickFilter.h`) with quaternion seeding for instant orientation lock:

- **BLE Peripheral**: Advertises as "Elfo" with service `1101` and 4 characteristics (roll, pitch, deviation, alert)
- **Calibration**: Captures baseline roll/pitch by averaging 200 accelerometer samples (~2s at 100Hz), then seeds filter quaternion from accelerometer
- **Detection**: Uses rolling window debouncing (50 samples, trigger at 80% above threshold) to detect sustained hip drops (>20° deviation from baseline). Uses Madgwick-filtered roll/pitch for detection.
- **Output**: Filter runs at 100Hz; BLE writes + serial prints throttled to ~5Hz (200ms interval)
- **LED**: Built-in LED lights during active hip drop alert

**Legacy version** (`imu/peripheral_roll_pitch`): Uses complementary filter instead of Madgwick, simpler detection without rolling-window debounce.

### BLE Communication

**Service UUID**: `1101`
**Characteristics**:
- `2101` - Roll (float32, degrees)
- `2102` - Pitch (float32, degrees)
- `2103` - Deviation (float32, degrees)
- `2104` - Alert (boolean)

### Node.js Central (`perch/central.js`)

- Uses `@abandonware/noble` for BLE scanning/connection
- Polls all 4 characteristics at ~5Hz (200ms interval)
- Exposes Express server on port 3000 with:
  - `GET /` - Serves dashboard
  - `GET /api/sensor` - Returns `{ attitude: { roll, pitch, deviation, alert } }`
  - `POST /` - Also returns sensor data (legacy endpoint)

### Web Dashboard (`perch/views/index.ejs`)

- Deviation gauge with green/yellow/red zones (0-15°/15-20°/20°+)
- 2D hip position dot showing roll/pitch on a bullseye
- Chart.js time-series with threshold annotation lines at ±20°
- Session recording with start/stop, live stats (duration, samples, max deviation, alerts), and CSV download
- Polls `/api/sensor` every 50ms, displays last 200 samples
- Dark theme optimized for demo/monitoring

### Serial Logger (`perch/logger.js`)

- Standalone CLI tool for recording IMU data directly from Arduino serial output
- Auto-detects Arduino serial port, parses firmware output format
- Writes timestamped CSV with roll/pitch/yaw/deviation/alert columns
- Keyboard controls: SPACE pause/resume, Q stop and save

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
  imu.ino                    # Active firmware: Madgwick filter + BLE peripheral
  MadgwickFilter.h           # Custom Madgwick AHRS with quaternion seeding
  peripheral_roll_pitch      # Legacy: complementary filter version

perch/
  central.js                 # BLE central + Express server
  logger.js                  # Standalone serial IMU logger (CSV output)
  package.json               # Dependencies: noble, express, ejs, serialport
  views/
    index.ejs                # Dashboard UI with gauge, hip dot, chart, session recording
```
