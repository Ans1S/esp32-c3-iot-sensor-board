// Run the embedded captive wizard against a local station API with controlled faults.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
const html = source.match(/const char kSetupWizardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
const origin = 'http://station.test';
const token = require('node:crypto').randomBytes(16).toString('hex');
const baseNode = { mac: '02:00:00:00:00:01', name: 'New node', provisioned: false,
  sensorType: 0, pcbVersion: 4, operatingMode: 1, confirmed: false, measured: false };

async function exercise(browser, run) {
  const page = await browser.newPage({ viewport: { width: 390, height: 844 } });
  const state = { nodes: [{ ...baseNode }], capacity: 19, invitations: 0, additions: [],
    saves: 0, deletes: 0, configFailures: 0, wifiSsid: '', interval: 60, connected: false };
  const errors = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.route(origin + '/**', async route => {
    const url = new URL(route.request().url());
    const reply = value => route.fulfill({ json: value });
    if (url.pathname === '/setup') return route.fulfill({ contentType: 'text/html', body: html });
    if (url.pathname === '/dashboard') return route.fulfill({ contentType: 'text/html', body: 'Station settings' });
    if (url.pathname === '/api/config') {
      if (state.configFailures > 0) { --state.configFailures; return route.abort(); }
      return reply({ csrfToken: token, wifiSsid: state.wifiSsid, defaultSleepSeconds: state.interval });
    }
    if (url.pathname === '/api/status') return reply(url.searchParams.has('setupSensors') ?
      { sensors: state.nodes, capacity: state.capacity } : { wifi: { connected: state.connected } });
    if (url.pathname === '/api/wifi-scan') return reply({ scanning: false, networks: [
      { ssid: 'test-network', secure: true, channel: 6, rssi: -48 }] });
    const args = Object.fromEntries(new URLSearchParams(route.request().postData()));
    assert.equal(args.csrf, token);
    if (url.pathname === '/api/enrollment') { ++state.invitations; return reply({success: true}); }
    if (url.pathname === '/api/setup-sensor') {
      state.additions.push(args);
      Object.assign(state.nodes[0], { provisioned: true, name: args.name,
        configuredSensorType: Number(args.sensorType), sleepSeconds: Number(args.sleepSeconds) });
      if (state.loseAddReply) return route.abort();
      return reply({success: true});
    }
    if (url.pathname === '/api/delete-sensor') {
      assert.equal(args.pendingOnly, '1');
      state.nodes = []; ++state.deletes; return reply({success: true});
    }
    if (url.pathname === '/api/setup') {
      ++state.saves;
      if (state.stallFirstSave && state.saves === 1) {
        await new Promise(resolve => setTimeout(resolve, 5500));
        return route.abort().catch(() => {});
      }
      state.wifiSsid = args.ssid; state.interval = Number(args.defaultSleep);
      assert.equal(args.wifiPassword, 'fixture password');
      if (state.loseSaveReply) return route.abort();
      return reply({success: true});
    }
    throw new Error('Unexpected request to ' + url.pathname);
  });
  try { await run(page, state); assert.deepEqual(errors, []); }
  finally { await page.close(); }
}
async function next(page) { await page.locator('#next').click(); }
async function sensors(page) {
  await page.goto(origin + '/setup');
  await next(page); await next(page); await next(page);
  await page.locator('.setupSensor').waitFor();
}
async function refresh(page) { await page.evaluate(() => loadSetupSensors(true)); }

(async () => {
  const browser = await chromium.launch({headless: true});
  try {
    await exercise(browser, async (page, state) => {
      state.loseAddReply = state.loseSaveReply = true;
      await sensors(page);
      const card = page.locator('.setupSensor');
      assert.match(await card.locator('code').innerText(), /Module not identified yet/);
      await card.locator('.setupSensorName').fill('Kitchen');
      state.nodes[0].sensorType = 7;
      await refresh(page);
      assert.equal(await card.locator('.setupSensorType').inputValue(), '0');
      assert.equal(await card.locator('.setupSensorInterval').inputValue(), '300');
      assert.equal(await card.locator('.setupSensorName').inputValue(), 'Kitchen');
      await card.getByRole('button', {name:'Add sensor'}).click();
      await page.getByText(/Saved on station · waiting for node/).waitFor();
      assert.equal(state.additions.length, 1);
      assert.equal(state.additions[0].sensorType, '0');
      state.nodes[0].confirmed = true; await refresh(page);
      await page.getByText(/Confirmed by node · waiting for first measurement/).waitFor();
      state.nodes[0].measured = true; await refresh(page);
      await page.getByText(/First measurement received/).waitFor();
      assert.equal(state.invitations, 1);
      await next(page);
      await page.locator('#scanBtn').click();
      await page.getByRole('button', {name:/test-network/}).click();
      await page.locator('#wifiPassword').fill('fixture password');
      await next(page); await next(page); await next(page);
      await page.locator('#done').waitFor();
      assert.equal(state.saves, 1);
      assert.equal(await page.locator('#done a').first().getAttribute('href'), 'http://w-charger.local/');
      await page.getByRole('link', {name:'local station settings'}).waitFor();
      assert.equal(new URL(page.url()).pathname, '/setup');
      await page.getByRole('link', {name:'local station settings'}).click();
      await page.waitForURL('**/dashboard');
    });
    await exercise(browser, async (page, state) => {
      state.configFailures = 1; state.capacity = 1;
      await sensors(page);
      await page.getByText(/All sensor slots are in use/).waitFor();
      const card = page.locator('.setupSensor');
      await card.locator('.setupSensorType').selectOption('4');
      await card.locator('.setupSensorInterval').selectOption('10');
      state.nodes[0].sensorType = 2; await refresh(page);
      assert.equal(await card.locator('.setupSensorType').inputValue(), '4');
      assert.equal(await card.locator('.setupSensorInterval').inputValue(), '10');
      page.once('dialog', dialog => dialog.accept());
      await card.getByRole('button', {name:'Remove from list'}).click();
      await card.waitFor({state:'detached'}); assert.equal(state.deletes, 1);
      state.nodes = [{...baseNode, resetPending:true}]; await refresh(page);
      assert(await page.getByRole('button', {name:'Waiting for reset'}).isDisabled());
      state.nodes[0].resetPending = false; await refresh(page);
      await card.getByRole('button', {name:'Add sensor'}).click();
      await page.locator('.sensorReady').waitFor();
      assert.equal(state.additions.length, 1);
      assert.equal(state.additions[0].csrf, token);
    });
    await exercise(browser, async (page, state) => {
      state.stallFirstSave = true; state.connected = true;
      await page.goto(origin + '/setup'); await next(page);
      await page.locator('#intervals [data-value="custom"]').click();
      await page.locator('#customSetupMinutes').fill('17');
      await next(page); await next(page); await next(page);
      await next(page);
      await page.getByText('Please select your Wi-Fi network.').waitFor();
      await page.locator('#ssid').fill('test-network');
      await page.locator('#wifiPassword').fill('fixture password');
      await next(page);
      await page.locator('#adminPassword').fill('test setup secret');
      await page.locator('#confirmAdminPassword').fill('different');
      await next(page);
      await page.getByText('The website passwords do not match.').waitFor();
      await page.locator('#confirmAdminPassword').fill('test setup secret');
      await next(page);
      assert.equal(await page.locator('#reviewInterval').innerText(), '17 minutes');
      await next(page);
      await page.getByText(/Save could not be confirmed/).waitFor();
      assert.equal(state.saves, 1);
      assert.equal(await page.locator('#ssid').inputValue(), 'test-network');
      await next(page);
      await page.waitForURL('**/dashboard');
      assert.equal(state.saves, 2); assert.equal(state.interval, 1020);
    });
    console.log('Captive onboarding: Auto and explicit types, draft retention, token recovery, lost POST replies, confirmation, capacity, reset, removal and Wi-Fi recovery passed');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
