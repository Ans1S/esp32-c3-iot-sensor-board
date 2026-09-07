// Generate public README images from the real embedded station pages.
// All fixture values are deliberately synthetic and safe to publish.
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

const demoSensors = [
  {
    mac: '02:00:00:00:10:01', name: 'Living room', provisioned: true,
    seen: true, online: true, sensorType: 2, configuredSensorType: 2,
    capabilities: 127, pcbVersion: 4, sleepSeconds: 600, operatingMode: 3,
    batteryMv: 3820, temperature: 22.7, humidity: 48.3, pressure: 1008.4,
    iaq: 42, iaqAccuracy: 3, gasResistanceOhms: 126400,
    iaqCalibrationElapsedMinutes: 510, iaqCalibrationPhase: 3,
    historyRevision: 4, revision: 3, appliedRevision: 3, ageMs: 18000,
    rssi: -56, profileSlot: 0, uploadEnabled: true,
    temperatureField: 1, humidityField: 2, pressureField: 3,
    iaqField: 4, gasResistanceField: 5, batteryField: 6,
    version: '4.1.3', build: 'demo-v4'
  },
  {
    mac: '02:00:00:00:10:02', name: 'Workshop', provisioned: true,
    seen: true, online: true, sensorType: 1, configuredSensorType: 1,
    capabilities: 71, pcbVersion: 3, sleepSeconds: 1800, operatingMode: 2,
    batteryMv: 3650, temperature: 20.9, humidity: 53.1, pressure: 1007.8,
    historyRevision: 7, revision: 2, appliedRevision: 2, ageMs: 94000,
    rssi: -68, profileSlot: 255, uploadEnabled: false,
    version: '4.1.3', build: 'demo-v3'
  }
];

const demoConfig = {
  csrfToken: 'demo-csrf-token', defaultSleepSeconds: 600,
  wifiSsid: 'Demo Home Network', adminPasswordSet: true,
  apActive: false, apSsid: 'W-Charger-DEMO', apIp: '192.0.2.1',
  fallbackChannel: 6, userApiKey: '', accessPointPassword: '',
  thingSpeakChannels: [{
    slot: 0, channelId: 1234567, name: 'Indoor climate demo',
    readApiKey: '', writeApiKey: '', publicChannel: false
  }]
};

const fixedNow = Date.UTC(2026, 8, 7, 12, 0, 0);
function historyFor(mac) {
  const sensor = demoSensors.find(item => item.mac === mac) || demoSensors[0];
  const points = Array.from({ length: 48 }, (_, index) => {
    const wave = Math.sin(index / 6);
    return {
      t: fixedNow - (47 - index) * 30 * 60 * 1000,
      temperature: sensor.temperature + wave * 0.8,
      humidity: sensor.humidity - wave * 3.2,
      pressure: sensor.pressure + Math.cos(index / 8) * 1.3,
      iaq: sensor.sensorType === 2 ? 42 + Math.round(wave * 5) : 0,
      gasResistance: sensor.sensorType === 2 ? 126.4 + wave * 7 : 0,
      battery: sensor.batteryMv / 1000 - index * 0.0004,
      sensorType: sensor.sensorType
    };
  });
  return { bucketSeconds: 1800, points };
}

async function routeDemo(route) {
  const url = new URL(route.request().url());
  if (url.pathname === '/dashboard') {
    return route.fulfill({ contentType: 'text/html', body: dashboardHtml });
  }
  if (url.pathname === '/updates') {
    return route.fulfill({ contentType: 'text/html', body: otaHtml });
  }
  let body = { success: true, sensors: [] };
  if (url.pathname === '/api/config') body = demoConfig;
  else if (url.pathname === '/api/status') body = {
    sensors: demoSensors,
    wifi: { connected: true, ssid: 'Demo Home Network', ip: '192.0.2.42', channel: 6 },
    cloud: { lastHttpStatus: 200, lastEntryId: 4242, successCount: 128, ageMs: 65000 }
  };
  else if (url.pathname === '/api/history') body = historyFor(url.searchParams.get('mac'));
  else if (url.pathname === '/api/ota') body = {
    available: true,
    mac: demoSensors[0].mac,
    targetVersion: '4.2.0',
    state: 'Transferring',
    bytes: 655360,
    total: 1092000,
    reason: 'The sensor is receiving the signed update. Transfer resumes automatically after every sleep cycle.',
    nodes: demoSensors.map(sensor => ({
      mac: sensor.mac, pcb: sensor.pcbVersion, version: sensor.version,
      release: 40103, build: sensor.build
    }))
  };
  else if (url.pathname.includes('thingspeak')) body = { success: true, channels: [] };
  return route.fulfill({ contentType: 'application/json', body: JSON.stringify(body) });
}

function servePreview(port = 4173) {
  const server = http.createServer((request, response) => {
    const url = new URL(request.url, `http://127.0.0.1:${port}`);
    let body;
    let contentType = 'application/json';
    if (url.pathname === '/dashboard') {
      body = dashboardHtml;
      contentType = 'text/html; charset=utf-8';
    } else if (url.pathname === '/updates') {
      body = otaHtml;
      contentType = 'text/html; charset=utf-8';
    } else if (url.pathname === '/api/config') body = JSON.stringify(demoConfig);
    else if (url.pathname === '/api/status') body = JSON.stringify({
      sensors: demoSensors,
      wifi: { connected: true, ssid: 'Demo Home Network', ip: '192.0.2.42', channel: 6 },
      cloud: { lastHttpStatus: 200, lastEntryId: 4242, successCount: 128, ageMs: 65000 }
    });
    else if (url.pathname === '/api/history') {
      body = JSON.stringify(historyFor(url.searchParams.get('mac')));
    } else if (url.pathname === '/api/ota') body = JSON.stringify({
      available: true, mac: demoSensors[0].mac, targetVersion: '4.2.0',
      state: 'Transferring', bytes: 655360, total: 1092000,
      reason: 'The sensor is receiving the signed update. Transfer resumes automatically after every sleep cycle.',
      nodes: demoSensors.map(sensor => ({
        mac: sensor.mac, pcb: sensor.pcbVersion, version: sensor.version,
        release: 40103, build: sensor.build
      }))
    });
    else body = JSON.stringify({ success: true, sensors: [], channels: [] });
    response.writeHead(200, { 'Content-Type': contentType, 'Cache-Control': 'no-store' });
    response.end(body);
  });
  server.listen(port, '127.0.0.1', () => {
    console.log(`Demo UI available at http://127.0.0.1:${port}/dashboard`);
  });
}

async function saveWebp(page, name, options = {}) {
  const pngPath = path.join(__dirname, `${name}.png`);
  const webpPath = path.join(__dirname, `${name}.webp`);
  await page.screenshot({ path: pngPath, ...options });
  await sharp(pngPath).webp({ quality: 88, effort: 6 }).toFile(webpPath);
  fs.unlinkSync(pngPath);
  console.log(`Created ${path.relative(repoRoot, webpPath)}`);
}

async function generateScreenshots() {
  const browser = await chromium.launch({ headless: true });
  try {
    const context = await browser.newContext({
      viewport: { width: 1440, height: 1000 },
      colorScheme: 'light',
      locale: 'en-US',
      timezoneId: 'Europe/Berlin'
    });
    const page = await context.newPage();
    page.on('pageerror', error => console.error('Page error:', error.message));
    await page.route('http://station.demo/**', routeDemo);

    await page.goto('http://station.demo/dashboard');
    await page.locator('.sensor h3').first().waitFor();
    await page.locator('[data-firmware]').first().filter({ hasText: 'Software 4.1.3' }).waitFor();
    await page.evaluate(() => window.scrollTo(0, 0));
    await saveWebp(page, 'station-overview', { clip: { x: 0, y: 0, width: 1440, height: 1000 } });

    await page.getByRole('button', { name: 'Settings' }).click();
    await page.locator('#settings.view.on').waitFor();
    await page.evaluate(() => window.scrollTo(0, 0));
    await saveWebp(page, 'station-settings', { clip: { x: 0, y: 0, width: 1440, height: 1000 } });

    await page.goto('http://station.demo/updates');
    await page.locator('#stateBadge').filter({ hasText: 'Transferring' }).waitFor();
    await page.evaluate(() => window.scrollTo(0, 0));
    await saveWebp(page, 'station-ota', { clip: { x: 0, y: 0, width: 1440, height: 960 } });
  } finally {
    await browser.close();
  }
}

if (process.argv.includes('--serve')) {
  servePreview(Number(process.env.PORT || 4173));
} else generateScreenshots().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
