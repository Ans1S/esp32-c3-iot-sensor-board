// Render the real embedded pages with deterministic public example data.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');

function loadPackage(name) {
  try {
    return require(name);
  } catch (error) {
    if (!process.env.CODEX_NODE_MODULES) throw error;
    return require(require.resolve(name, { paths: [process.env.CODEX_NODE_MODULES] }));
  }
}
const { chromium } = loadPackage('playwright');
const sharp = loadPackage('sharp');
const repoRoot = path.resolve(__dirname, '..');
const dashboardSource = fs.readFileSync(
  path.join(repoRoot, 'Firmware/station/src/web_pages.cpp'), 'utf8');
const otaSource = fs.readFileSync(
  path.join(repoRoot, 'Firmware/station/include/ota_page.h'), 'utf8');
const dashboardHtml = dashboardSource.match(
  /const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
const otaHtml = otaSource.match(/R"HTML\((.*?)\)HTML"/s)[1];
const origin = 'http://station.test';
const fixedNow = Date.UTC(2026, 9, 25, 12, 0, 0);

const common = {
  provisioned: true, seen: true, online: true, pcbVersion: 4,
  revision: 3, appliedRevision: 3, historyRevision: 1, operatingMode: 2,
  ageMs: 18000, rssi: -56, profileSlot: 255, uploadEnabled: false,
  version: '4.4.0', build: 'demo-build-v4', live: false,
  normalHistory: true, temperatureField: 1, humidityField: 2,
  pressureField: 3, batteryField: 5
};
const demoSensors = [
  {
    ...common, mac: '02:00:00:00:10:01', name: 'Climate node',
    sensorType: 1, configuredSensorType: 1, capabilities: 23,
    sleepSeconds: 900, batteryMv: 3940, temperature: 21.6,
    humidity: 48.3, pressure: 1012.4
  },
  {
    ...common, mac: '02:00:00:00:10:02', name: 'Air-quality node',
    sensorType: 2, configuredSensorType: 2, capabilities: 127,
    sleepSeconds: 600, batteryMv: 3820, temperature: 22.2,
    humidity: 46.1, pressure: 1012.8, iaq: 42, iaqAccuracy: 3,
    gasResistanceOhms: 126400, iaqCalibrationPhase: 3
  },
  {
    ...common, mac: '02:00:00:00:10:03', name: 'Temperature node',
    sensorType: 4, configuredSensorType: 4, capabilities: 80,
    sleepSeconds: 10, batteryMv: 3910, temperature: 23.125,
    operatingMode: 4, recording: { state: 1, pending: 0, capacity: 44352 }
  },
  {
    ...common, mac: '02:00:00:00:10:04', name: 'Motion node',
    sensorType: 3, configuredSensorType: 3, capabilities: 144,
    sleepSeconds: 10, batteryMv: 3900, operatingMode: 4,
    ax: 0.12, ay: -0.24, az: 1.0, gx: 12, gy: -24, gz: 0,
    peakAcceleration: 1.4, peakAngularRate: 30,
    recording: { state: 1, pending: 0, capacity: 22176 }
  },
  {
    ...common, mac: '02:00:00:00:10:05', name: 'Optical node',
    sensorType: 5, configuredSensorType: 5, capabilities: 16,
    pcbVersion: 3, build: 'demo-build-v3', sleepSeconds: 10,
    batteryMv: 3800, operatingMode: 4, normalHistory: false,
    recording: { state: 1, pending: 0, capacity: 10912 }
  }
];
const demoConfig = {
  csrfToken: 'demo-csrf-token', defaultSleepSeconds: 600,
  wifiSsid: 'Demo Home Network', adminPasswordSet: true,
  apActive: false, apSsid: 'W-Charger-DEMO', apIp: '192.0.2.1',
  fallbackChannel: 6, userApiKey: '', accessPointPassword: '',
  dashboard: {
    columns: 3, comfortable: false, order: demoSensors.map(sensor => sensor.mac),
    hidden: demoSensors.slice(3).map(sensor => sensor.mac)
  },
  thingSpeakChannels: []
};

function normalHistory(mac) {
  const sensor = demoSensors.find(item => item.mac === mac) || demoSensors[0];
  const step = sensor.sleepSeconds * 1000;
  const span = sensor.sleepSeconds < 60 ? 3600000 : 2 * 86400000;
  const count = Math.floor(span / step) + 1;
  return Array.from({ length: count }, (_, index) => {
    const wave = Math.sin(index / 9) * 0.55 + Math.cos(index / 23) * 0.2;
    return {
      t: fixedNow - (count - 1 - index) * step,
      ax: wave * 0.1, ay: wave * 0.2, az: 1 + wave * 0.02,
      gx: wave * 3, gy: wave * 2, gz: wave,
      temperature: (sensor.temperature || 22) + wave,
      humidity: (sensor.humidity || 48) - wave * 3.2,
      pressure: (sensor.pressure || 1012) + Math.cos(index / 8) * 1.3,
      iaq: 42 + Math.round(16 + wave * 26), gas: 126.4 + wave * 7,
      battery: sensor.batteryMv / 1000, sensorType: sensor.sensorType,
      pcbVersion: sensor.pcbVersion
    };
  });
}
for (const sensor of demoSensors.filter(item => item.normalHistory)) {
  const points = normalHistory(sensor.mac);
  sensor.history = { oldest: points[0].t / 1000, newest: fixedNow / 1000,
    points: points.length, durablePoints: points.length, pendingPoints: 0,
    capacityPoints: 23000 };
}
const recording = {
  session: '0000000000001001', epochMs: fixedNow - 3600000,
  durationMs: 120000, availableMs: 120000,
  expected: 2400, stored: 2400, complete: true, sensorType: 3
};
const recordingPoints = Array.from({ length: recording.stored }, (_, index) => {
  const t = (index + 1) * 50, wave = Math.sin(t / 420);
  return { sampleMs: t, t, estimateT: t, ax: wave * 0.6,
    ay: Math.cos(t / 420) * 0.25, az: 1 + wave * 0.1,
    gx: wave * 20, gy: Math.cos(t / 420) * 8, gz: wave * 3,
    steps: Math.floor(t / 600), activeSeconds: Math.floor(t / 1000),
    peakAcceleration: 1.4, wave: [] };
});

function demoResponse(url) {
  if (url.pathname === '/dashboard' || url.pathname === '/') {
    return { contentType: 'text/html; charset=utf-8', body: dashboardHtml };
  }
  if (url.pathname === '/updates') {
    return { contentType: 'text/html; charset=utf-8', body: otaHtml };
  }
  let body;
  if (url.pathname === '/api/config') body = demoConfig;
  else if (url.pathname === '/api/status') body = {
    sensors: demoSensors, storageAvailable: true, uptimeMs: 3600000,
    wifi: { connected: true, ssid: 'Demo Home Network', ip: '192.0.2.42', channel: 6 },
    cloud: {}
  };
  else if (url.pathname === '/api/history') {
    const points = normalHistory(url.searchParams.get('mac'));
    const after = Number(url.searchParams.get('after') || 0);
    const until = Number(url.searchParams.get('until')) || fixedNow / 1000;
    const selected = points.filter(point => point.t / 1000 > after && point.t / 1000 <= until).slice(0, 128);
    body = { paged: true, until, oldest: points[0].t / 1000,
      bucketSeconds: (points[1].t - points[0].t) / 1000,
      durablePoints: points.length, pendingPoints: 0, capacityPoints: 23000,
      nextAfter: selected.length === 128 && selected.at(-1).t < points.at(-1).t
        ? selected.at(-1).t / 1000 : 0, points: selected };
  } else if (url.pathname === '/api/recordings') body = {
    freeBytes: 786432,
    sessions: url.searchParams.get('mac') === demoSensors[3].mac ? [recording] : []
  };
  else if (url.pathname === '/api/recording') {
    const offset = Number(url.searchParams.get('offset') || 0);
    const points = recordingPoints.slice(offset, offset + 128);
    body = { ...recording, nextOffset: offset + points.length, points };
  } else if (url.pathname === '/api/ota') body = {
    available: true, mac: demoSensors[1].mac, targetVersion: '4.4.1',
    state: 'Waiting for next contact', bytes: 0, total: 1107104,
    reason: 'The signed package is stored on the station. The sensor will request it at its next scheduled contact.',
    nodes: demoSensors.map(sensor => ({ mac: sensor.mac, pcb: sensor.pcbVersion,
      version: sensor.version, release: 40400, build: sensor.build }))
  };
  else if (url.pathname === '/favicon.ico') return { contentType: 'image/x-icon', body: '' };
  else throw new Error('Missing public fixture for ' + url.pathname);
  return { contentType: 'application/json', body: JSON.stringify(body) };
}

function servePreview(port = 4173) {
  http.createServer((request, response) => {
    try {
      const result = demoResponse(new URL(request.url, 'http://127.0.0.1:' + port));
      response.writeHead(200, { 'Content-Type': result.contentType, 'Cache-Control': 'no-store' });
      response.end(result.body);
    } catch {
      response.writeHead(404);
      response.end();
    }
  }).listen(port, '127.0.0.1', () => console.log('Example dashboard: http://127.0.0.1:' + port));
}

async function saveWebp(page, name, options = {}) {
  const viewport = page.viewportSize();
  if (options.fullPage) {
    // Keep fixed elements outside the capture when they are below the viewport.
    const height = await page.evaluate(() => document.documentElement.scrollHeight);
    await page.setViewportSize({ width: viewport.width, height });
  }
  const png = await page.screenshot({ animations: 'disabled', ...options });
  const output = path.join(__dirname, name + '-4.4.1.webp');
  await sharp(png).webp({ quality: 88, effort: 6 }).toFile(output);
  if (options.fullPage) await page.setViewportSize(viewport);
  console.log('Created ' + path.relative(repoRoot, output));
}

async function generateScreenshots() {
  const browser = await chromium.launch({ headless: true });
  const errors = [];
  try {
    const context = await browser.newContext({
      viewport: { width: 1280, height: 1000 }, colorScheme: 'light',
      locale: 'en-US', timezoneId: 'UTC'
    });
    const page = await context.newPage();
    page.on('pageerror', error => errors.push(error.message));
    await page.addInitScript(now => {
      const NativeDate = Date;
      globalThis.Date = class extends NativeDate {
        constructor(...args) { super(...(args.length ? args : [now])); }
        static now() { return now; }
      };
      // A fixed display snapshot keeps polling from racing screenshot capture.
      window.setInterval = () => 0;
    }, fixedNow);
    await context.route('**/*', async route => {
      const url = new URL(route.request().url());
      assert.equal(url.origin, origin, 'Screenshots must never contact a device or cloud service');
      return route.fulfill(demoResponse(url));
    });

    await page.goto(origin + '/dashboard');
    await page.locator('.sensorSummary').first().waitFor();
    assert.equal(await page.locator('.sensorSummary').count(), 3);
    const grid = await page.locator('#sensorGrid').boundingBox();
    await saveWebp(page, 'station-overview', {
      clip: { x: 0, y: 0, width: 1280, height: Math.ceil(grid.y + grid.height + 48) }
    });

    await page.locator('.sensorSummary').nth(1).locator('.meta').click();
    await page.locator('.historySvg').waitFor();
    await page.waitForFunction(mac => (stationHistory[mac] || []).length > 0
      && historyRevisions[mac] != null, demoSensors[1].mac);
    await page.getByLabel('History metric').selectOption('iaq');
    await page.evaluate(() => window.scrollTo(0, 0));
    await saveWebp(page, 'station-history', { fullPage: true });

    await page.goto(origin + '/dashboard');
    await page.getByRole('button', { name: 'Settings', exact: true }).click();
    await page.locator('#settings.view.on').waitFor();
    await saveWebp(page, 'station-settings', { clip: { x: 0, y: 0, width: 1280, height: 1000 } });

    await page.goto(origin + '/dashboard');
    await page.locator('.sensorSummary').first().waitFor();
    await page.evaluate(mac => openSensorDetail(mac), demoSensors[3].mac);
    await page.getByLabel('Recording selection').selectOption(recording.session);
    await page.waitForFunction(() => document.querySelector('.recordingCharts svg.liveSvg')
      && !recordingViews['02:00:00:00:10:04'].busy);
    await page.locator('.currentReadings > summary').click();
    await page.getByLabel('Recording view', { exact: true }).selectOption('minute');
    await page.evaluate(() => { document.documentElement.dataset.theme = 'dark'; window.scrollTo(0, 0); });
    await saveWebp(page, 'station-recording', { fullPage: true });

    await page.goto(origin + '/updates?mac=' + encodeURIComponent(demoSensors[1].mac));
    await page.locator('#stateBadge').filter({ hasText: 'Waiting for next contact' }).waitFor();
    await page.locator('#file').setInputFiles(path.join(repoRoot, 'Firmware/releases/sensor-v4-4.4.1.ota'));
    await page.waitForFunction(() => document.getElementById('compatibilityValue').textContent === 'Compatible');
    await page.evaluate(() => { document.documentElement.dataset.theme = 'light'; window.scrollTo(0, 0); });
    await saveWebp(page, 'station-ota', { fullPage: true });
    assert.deepEqual(errors, [], 'The embedded example pages must not have script errors');
  } finally {
    await browser.close();
  }
}

if (process.argv.includes('--serve')) servePreview(Number(process.env.PORT || 4173));
else generateScreenshots().catch(error => { console.error(error); process.exitCode = 1; });
