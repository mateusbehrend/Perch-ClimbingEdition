// based on the example on https://www.npmjs.com/package/@abandonware/noble

const noble = require('@abandonware/noble');

const uuid_service = "1101";
// 2101 → roll, 2102 → pitch, 2103 → deviation, 2104 → alert (float32/bool)
const uuid_roll = "2101", uuid_pitch = "2102";
const uuid_deviation = "2103", uuid_alert = "2104";

const uuid_characteristics = [uuid_roll, uuid_pitch, uuid_deviation, uuid_alert];

let sensorData = {
    attitude: { roll: 0, pitch: 0, deviation: 0, alert: false }
};

noble.on('stateChange', async (state) => {
    if (state === 'poweredOn') {
        console.log("start scanning");
        await noble.startScanningAsync([uuid_service], false);
    }
});

noble.on('discover', async (peripheral) => {
    await noble.stopScanningAsync();
    await peripheral.connectAsync();
    const { characteristics } = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
        [uuid_service],
        uuid_characteristics
    );
    const byUuid = {};
    characteristics.forEach(c => { byUuid[c.uuid] = c; });
    readData(byUuid);
});

function parseFloatFromBuffer(buf) {
    if (!buf || buf.length < 4) return 0;
    return buf.readFloatLE(0);
}

async function readData(byUuid) {
    const read = async (uuid) => {
        const c = byUuid[uuid];
        if (!c) return null;
        try {
            const value = await c.readAsync();
            return parseFloatFromBuffer(value);
        } catch (e) {
            return null;
        }
    };

    if (byUuid[uuid_roll]) {
        const v = await read(uuid_roll);
        if (v != null) sensorData.attitude.roll = v;
    }
    if (byUuid[uuid_pitch]) {
        const v = await read(uuid_pitch);
        if (v != null) sensorData.attitude.pitch = v;
    }
    if (byUuid[uuid_deviation]) {
        const v = await read(uuid_deviation);
        if (v != null) sensorData.attitude.deviation = v;
    }
    if (byUuid[uuid_alert]) {
        const c = byUuid[uuid_alert];
        try {
            const buf = await c.readAsync();
            sensorData.attitude.alert = buf && buf.length > 0 && buf[0] !== 0;
        } catch (e) { /* ignore */ }
    }

    console.log('attitude', sensorData.attitude);

    setTimeout(() => readData(byUuid), 200); // ~5 Hz to match BLE update rate
}

//
// hosting a web-based front-end and respond requests with sensor data
// based on example code on https://expressjs.com/
//
const express = require('express');
const path = require('path');
const app = express();
const port = 3000;

app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));

app.get('/', (req, res) => {
    res.render('index');
});

app.get('/api/sensor', (req, res) => {
    res.set('Content-Type', 'application/json');
    res.json(sensorData);
});

app.post('/', (req, res) => {
    res.set('Content-Type', 'application/json');
    res.json(sensorData);
});

app.listen(port, () => {
    console.log(`IMU dashboard: http://localhost:${port}`);
});
