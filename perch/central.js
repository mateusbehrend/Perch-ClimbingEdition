// based on the example on https://www.npmjs.com/package/@abandonware/noble

const noble = require('@abandonware/noble');

const uuid_service = "1101";
// 2101 → roll, 2102 → pitch (float32, degrees)
const uuid_roll = "2101", uuid_pitch = "2102";

const uuid_characteristics = [uuid_roll, uuid_pitch];

// { attitude: { roll, pitch } } — degrees
let sensorData = {
    attitude: { roll: 0, pitch: 0 }
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
