#!/usr/bin/env node

// ============================================================
//  Perch IMU Logger
//  Records roll/pitch/deviation from the Arduino serial output
//  during an active climbing session and saves to CSV.
//
//  Usage:
//    node logger.js                       # auto-detect serial port
//    node logger.js /dev/tty.usbmodem1401 # specify port
//    node logger.js --list                # list available ports
//
//  Output: climb-log-<timestamp>.csv in the current directory
// ============================================================

const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const fs = require('fs');
const path = require('path');
const readline = require('readline');

const BAUD_RATE = 115200;

// ---- Helpers ----
function timestamp() {
  return new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
}

function formatDuration(ms) {
  const s = Math.floor(ms / 1000);
  const m = Math.floor(s / 60);
  const h = Math.floor(m / 60);
  if (h > 0) return `${h}h ${m % 60}m ${s % 60}s`;
  if (m > 0) return `${m}m ${s % 60}s`;
  return `${s}s`;
}

// ---- Parse a single IMU line ----
// Expected format from imu.ino:
//   Roll: 2.3°  |  Pitch: -1.5°  |  Yaw: 45.2°  |  Dev: 3.1°  (no hip drop)
function parseLine(line) {
  const result = {};
  const rollMatch = line.match(/Roll:\s*([-\d.]+)/);
  const pitchMatch = line.match(/Pitch:\s*([-\d.]+)/);
  const yawMatch = line.match(/Yaw:\s*([-\d.]+)/);
  const devMatch = line.match(/Dev:\s*([-\d.]+)/);

  if (rollMatch) result.roll = parseFloat(rollMatch[1]);
  if (pitchMatch) result.pitch = parseFloat(pitchMatch[1]);
  if (yawMatch) result.yaw = parseFloat(yawMatch[1]);
  if (devMatch) result.deviation = parseFloat(devMatch[1]);

  result.alert = line.includes('HIP DROP ALERT');

  // Only return if we got at least roll and pitch
  if (result.roll != null && result.pitch != null) return result;
  return null;
}

// ---- List available serial ports ----
async function listPorts() {
  const ports = await SerialPort.list();
  if (ports.length === 0) {
    console.log('No serial ports found.');
    return;
  }
  console.log('\nAvailable serial ports:');
  for (const p of ports) {
    const label = p.manufacturer ? ` (${p.manufacturer})` : '';
    console.log(`  ${p.path}${label}`);
  }
  console.log('');
}

// ---- Auto-detect Arduino port ----
async function findArduinoPort() {
  const ports = await SerialPort.list();
  // Prefer ports that look like Arduino / USB modem
  const candidates = ports.filter(p =>
    /arduino|usbmodem|usbserial|acm/i.test(p.path + (p.manufacturer || ''))
  );
  if (candidates.length > 0) return candidates[0].path;
  // Fall back to first available port
  if (ports.length > 0) return ports[0].path;
  return null;
}

// ---- Main ----
async function main() {
  const args = process.argv.slice(2);

  if (args.includes('--list') || args.includes('-l')) {
    await listPorts();
    process.exit(0);
  }

  let portPath = args[0];
  if (!portPath) {
    portPath = await findArduinoPort();
    if (!portPath) {
      console.error('No serial port found. Connect Arduino or specify port path.');
      console.error('Usage: node logger.js [port_path]');
      console.error('       node logger.js --list');
      process.exit(1);
    }
    console.log(`Auto-detected port: ${portPath}`);
  }

  // ---- Open serial port ----
  const port = new SerialPort({ path: portPath, baudRate: BAUD_RATE });
  const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

  // ---- Prepare CSV output ----
  const filename = `climb-log-${timestamp()}.csv`;
  const filepath = path.resolve(filename);
  const csvStream = fs.createWriteStream(filepath);
  csvStream.write('timestamp_ms,elapsed_s,roll_deg,pitch_deg,yaw_deg,deviation_deg,alert\n');

  let sampleCount = 0;
  let startTime = null;
  let maxDev = 0;
  let alertCount = 0;
  let lastAlertState = false;
  let paused = false;

  console.log(`\n  Perch IMU Logger`);
  console.log(`  ================`);
  console.log(`  Port:   ${portPath}`);
  console.log(`  Baud:   ${BAUD_RATE}`);
  console.log(`  Output: ${filename}`);
  console.log(`\n  Waiting for IMU data...\n`);
  console.log('  Controls: [SPACE] pause/resume  [Q] stop & save\n');

  // ---- Keyboard controls (raw mode) ----
  if (process.stdin.isTTY) {
    process.stdin.setRawMode(true);
    process.stdin.resume();
    process.stdin.on('data', (key) => {
      const ch = key.toString();
      if (ch === 'q' || ch === 'Q' || ch === '\u0003') { // q or Ctrl+C
        finish();
      } else if (ch === ' ') {
        paused = !paused;
        process.stdout.write(paused ? '\r  >> PAUSED (press SPACE to resume)        ' : '\r  >> RECORDING                              ');
      }
    });
  }

  // ---- Process incoming serial data ----
  parser.on('data', (line) => {
    // Skip calibration messages
    if (line.startsWith('>>') || line.startsWith('IMU') || line.startsWith('Failed')) {
      process.stdout.write(`  [info] ${line}\n`);
      return;
    }

    const data = parseLine(line);
    if (!data) return;
    if (paused) return;

    if (startTime == null) {
      startTime = Date.now();
      console.log('  Recording started!\n');
    }

    const now = Date.now();
    const elapsed = (now - startTime) / 1000;
    sampleCount++;

    const dev = data.deviation != null ? data.deviation : 0;
    if (dev > maxDev) maxDev = dev;
    if (data.alert && !lastAlertState) alertCount++;
    lastAlertState = data.alert;

    // Write CSV row
    csvStream.write([
      now,
      elapsed.toFixed(3),
      (data.roll || 0).toFixed(3),
      (data.pitch || 0).toFixed(3),
      (data.yaw || 0).toFixed(3),
      dev.toFixed(3),
      data.alert ? 1 : 0
    ].join(',') + '\n');

    // Live status line
    if (sampleCount % 10 === 0) {
      const dur = formatDuration(now - startTime);
      const status = data.alert ? ' !! HIP DROP !!' : (dev > 15 ? ' ~ watch posture' : '');
      process.stdout.write(
        `\r  [${dur}] samples: ${sampleCount} | roll: ${data.roll.toFixed(1)} | pitch: ${data.pitch.toFixed(1)} | dev: ${dev.toFixed(1)} | alerts: ${alertCount}${status}    `
      );
    }
  });

  port.on('error', (err) => {
    console.error(`\n  Serial error: ${err.message}`);
    process.exit(1);
  });

  port.on('close', () => {
    console.log('\n  Serial port closed.');
    finish();
  });

  function finish() {
    csvStream.end();
    const dur = startTime ? formatDuration(Date.now() - startTime) : '0s';
    console.log(`\n\n  Session complete`);
    console.log(`  ================`);
    console.log(`  Duration:    ${dur}`);
    console.log(`  Samples:     ${sampleCount}`);
    console.log(`  Max dev:     ${maxDev.toFixed(1)}\u00B0`);
    console.log(`  Hip alerts:  ${alertCount}`);
    console.log(`  Saved to:    ${filepath}`);
    console.log('');
    process.exit(0);
  }
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
