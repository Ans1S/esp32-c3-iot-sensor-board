const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  const html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const browser = await chromium.launch({headless: true,
    ...(process.env.PLAYWRIGHT_CHANNEL ? {channel: process.env.PLAYWRIGHT_CHANNEL} : {})});
  try {
    const page = await browser.newPage();
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    let starts = 0, polls = 0, statusCalls = 0, legacyCalls = 0;
    let listReady = false, busy = false;
    const mutations = new Set(['/api/create-channel', '/api/thingspeak/channel-create',
      '/api/thingspeak/channel-update', '/api/thingspeak/channel-clear', '/api/thingspeak/channel-delete']);
    const submitted = [];
    let mutationReady = false;
    await page.route('http://station.test/**', async route => {
      const request = route.request(), url = new URL(request.url());
      let status = 200, body = {success: true};
      if (url.pathname === '/dashboard') return route.fulfill({contentType: 'text/html', body: html});
      if (url.pathname === '/api/config') {
        body = {csrfToken: 'fixture-token', userApiKey: 'test-key', defaultSleepSeconds: 600,
          thingSpeakChannels: [], dashboard: {}, storageAvailable: true};
      } else if (url.pathname === '/api/status') {
        statusCalls++;
        body = {sensors: [], wifi: {connected: true}, cloud: {}};
      } else if (mutations.has(url.pathname)) {
        assert.equal(request.method(), 'POST');
        const form = new URLSearchParams(request.postData());
        assert.equal(form.get('csrf'), 'fixture-token');
        assert(Number(form.get('requestId')) > 0);
        submitted.push(url.pathname);
        status = 202;
        body = {success: true, ticket: 40 + submitted.length};
      } else if (url.pathname === '/api/thingspeak/operation') {
        assert.equal(Number(url.searchParams.get('ticket')), 40 + submitted.length);
        status = mutationReady ? 200 : 202;
        body = mutationReady ? {success: submitted.length !== 5, message: submitted.length === 5 ? 'Local save failed' : 'Applied', channelId: 42, slot: 2}
          : {success: true, pending: true};
      } else if (url.pathname === '/api/thingspeak/channels') {
        if (request.method() === 'POST') {
          starts++;
          assert.equal(new URLSearchParams(request.postData()).get('csrf'), 'fixture-token');
          status = busy ? 400 : 202;
          body = busy ? {success: false, message: 'Channel list is busy'} : {success: true, ticket: 19};
        } else if (url.searchParams.get('ticket') === '19') {
          polls++;
          status = listReady ? 200 : 202;
          body = listReady ? {success: true, channels: [{id: 42, name: 'Fixture channel', fields: [], localSlot: -1}]}
            : {success: true, pending: true};
        } else {
          legacyCalls++;
          body = {success: true, channels: []};
        }
      }
      return route.fulfill({status, contentType: 'application/json', body: JSON.stringify(body)});
    });
    await page.goto('http://station.test/dashboard');
    await page.waitForFunction(() => config.userApiKey === 'test-key');
    const firstPoll = page.waitForResponse(r => r.url().endsWith('?ticket=19') && r.status() === 202);
    await page.evaluate(() => { void loadRemoteChannels(); void loadRemoteChannels(true); });
    await firstPoll;
    assert(await page.locator('#loadCloudChannels').isDisabled());
    const beforeStatus = statusCalls;
    await page.evaluate(() => requestJson('/api/status'));
    assert(statusCalls > beforeStatus, 'Local status responds while the account job is pending');
    assert.equal(starts, 1, 'Concurrent dashboard callers share one background request');
    listReady = true;
    await page.waitForFunction(() => document.getElementById('cloudAccountMessage').textContent.includes('1 ThingSpeak channel loaded'));
    assert.equal(await page.evaluate(() => remoteChannels[0].id), 42);
    assert(!(await page.locator('#loadCloudChannels').isDisabled()));
    assert.equal(legacyCalls, 0);
    assert(polls >= 2);

    busy = true;
    await page.evaluate(() => loadRemoteChannels());
    assert.match(await page.locator('#cloudAccountMessage').textContent(), /Channel list is busy/);
    assert(!(await page.locator('#loadCloudChannels').isDisabled()));
    assert.equal(starts, 2);

    const pendingMutation = page.waitForResponse(r => r.url().includes('/api/thingspeak/operation') && r.status() === 202);
    await page.evaluate(() => {
      window.firstMutation = thingSpeakMutation('/api/create-channel', new URLSearchParams({name: 'Fixture'}));
    });
    await pendingMutation;
    const duringMutation = statusCalls;
    await page.evaluate(() => requestJson('/api/status'));
    assert(statusCalls > duringMutation);
    mutationReady = true;
    assert.equal(await page.evaluate(async () => (await window.firstMutation).slot), 2);
    for (const path of [...mutations].slice(1, 4)) {
      const result = await page.evaluate(path => thingSpeakMutation(path, new URLSearchParams({channelId: '42'})), path);
      assert.equal(result.message, 'Applied');
    }
    await assert.rejects(page.evaluate(() => thingSpeakMutation('/api/thingspeak/channel-delete',
      new URLSearchParams({channelId: '42'}))), /Local save failed/);
    assert.deepEqual(submitted, [...mutations], 'Result polling does not resend a mutation');
    assert.deepEqual(errors, []);
    console.log(`ThingSpeak dashboard: one list for concurrent callers, ${polls} list polls, five mutation routes, concurrent status and failure handling passed`);
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exit(1); });
