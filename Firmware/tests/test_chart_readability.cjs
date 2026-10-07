// Exercise the embedded chart renderers with deterministic sparse, noisy and gapped data.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  let html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  // Render the real page without starting network polling; fixtures exercise the same functions.
  html = html.replace('setInterval(()=>{if(!document.hidden)loadFirmwareVersions()},2500);', '');
  html = html.replace(/\bboot\(\);(?=\s*<\/script>)/, '');
  const browser = await chromium.launch({
    headless: true,
    ...(process.env.PLAYWRIGHT_CHANNEL ? {channel: process.env.PLAYWRIGHT_CHANNEL} : {})
  });
  try {
    const page = await browser.newPage({viewport: {width: 1280, height: 1000}});
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.setContent(html);
    await page.evaluate(() => {
      document.querySelector('main').style.display = 'none';
      document.body.insertAdjacentHTML('beforeend', '<main id="chartQa"><article class="card sensor" id="chartFixture"></article></main>');
      window.fixtureNow = Date.now();
    });

    await page.evaluate(() => {
      document.getElementById('chartFixture').innerHTML = chartSvg(
        [{t: fixtureNow - 4000, v: 22.123}], chartDefinitions.temperature, 900, 900, 900);
    });
    assert.equal(await page.locator('.historySvg').count(), 1, 'A single point must still have a chart');
    assert.equal(await page.locator('.historySvg .sample').count(), 1);
    assert.equal(await page.locator('.historySvg .sample').getAttribute('r'), '4');
    assert(await page.locator('.historySvg .grid').count() >= 3);
    await page.locator('.historySvg').hover();
    assert((await page.locator('.chartReadout').textContent()).includes('22.123 °C'));
    assert(/\d{2}:\d{2}:\d{2}\.\d{3}/.test(await page.locator('.chartReadoutTime').textContent()));
    await page.locator('.historySvg').focus();
    await page.keyboard.press('Home');
    assert.equal(Number(await page.locator('.historySvg').getAttribute('data-cursor-time')),
      await page.evaluate(() => fixtureNow - 4000));

    await page.evaluate(() => {
      document.getElementById('chartFixture').innerHTML = chartSvg(
        [{t: fixtureNow - 4000, v: 22.123}], chartDefinitions.temperature, 900, 900, 1);
    });
    assert.equal(await page.locator('.historySvg').count(), 0,
      'A point outside a one-second window must not be projected into it');
    assert((await page.locator('.chartEmpty').textContent()).includes('No measurements'));

    const gapChecks = await page.evaluate(() => {
      const history = [{t: 100, ax: 1}, {t: 150, ax: 0, gap: true}, {t: 200, ax: 2}];
      const series = liveSeries(history, 'ax', 'var(--series-blue)', 'X');
      const segments = chartSegments(series.points, 250);
      return {lengths: segments.map(s => s.length), input: history.map(p => p.ax),
        smooth: segments.map(s => chartDisplayPoints(s, 'smooth').map(p => p.v))};
    });
    assert.deepEqual(gapChecks.lengths, [1, 1], 'An explicit invalid report must break the trace');
    assert.deepEqual(gapChecks.smooth, [[1], [2]], 'Filtering must not mix measurements across a gap');
    assert.deepEqual(gapChecks.input, [1, 0, 2], 'Filtering must not modify acquired measurements');

    await page.evaluate(() => {
      Date.now = () => fixtureNow;
      const sensor = {mac: 'history-test', sensorType: 1, capabilities: 17,
        provisioned: true, sleepSeconds: 900, live: false};
      statusData.sensors = [sensor];
      stationHistory[sensor.mac] = [
        {t: fixtureNow - 23 * 3600000, temperature: 20},
        {t: fixtureNow - 20 * 60000, temperature: 21},
        {t: fixtureNow - 4000, temperature: 22.123}
      ];
      historyBuckets[sensor.mac] = 900;
      renderSensors = () => {
        document.getElementById('chartFixture').innerHTML =
          environmentChartPanel(sensor, stationHistory[sensor.mac]);
      };
      renderSensors();
    });
    assert.equal(await page.getByLabel('Chart time window').inputValue(), '86400');
    assert.equal(await page.locator('.historySvg .sample').count(), 3);
    await page.getByLabel('Chart time window').selectOption('900');
    assert.equal(await page.locator('.historySvg .sample').count(), 1,
      'Switching from 24 hours to 15 minutes must show a single recent measurement');
    await page.getByLabel('Chart time window').selectOption('1');
    assert.equal(await page.locator('.historySvg').count(), 0,
      'A one-second view may be empty when no reading was acquired in that second');
    await page.getByLabel('Chart time window').selectOption('900');
    await page.getByRole('button', {name: 'Previous window', exact: true}).click();
    const selectedEnd = Number(await page.locator('.historySvg').getAttribute('data-end'));
    assert.equal(selectedEnd, await page.evaluate(() => fixtureNow - 900000));
    assert.equal(await page.locator('.historySvg .sample').count(), 1);
    await page.evaluate(() => {
      fixtureNow += 10000;
      stationHistory['history-test'].push({t: fixtureNow, temperature: 22.2});
      renderSensors();
    });
    assert.equal(Number(await page.locator('.historySvg').getAttribute('data-end')), selectedEnd,
      'Polling new measurements must keep a selected historical window fixed');
    assert.equal(await page.getByLabel('Chart time window').inputValue(), '900');
    await page.getByRole('button', {name: 'Now', exact: true}).click();
    assert.equal(Number(await page.locator('.historySvg').getAttribute('data-end')),
      await page.evaluate(() => fixtureNow));
    assert.equal(await page.getByRole('button', {name: 'Next window', exact: true}).isEnabled(), false);
    await page.evaluate(() => {
      for (let i = 0; i < 120; i++) moveChartHistory('history-test', -1);
    });
    assert.equal(await page.getByRole('button', {name: 'Previous window', exact: true}).isEnabled(), false,
      'Navigation must stop at the retained history boundary');
    assert(Number(await page.locator('.historySvg').getAttribute('data-start')) >=
      await page.evaluate(() => stationHistory['history-test'][0].t));

    await page.evaluate(() => {
      const history = [0, 9, 0].map((ax, i) => ({
        t: fixtureNow - 3000 + i * 1000, ax, ay: i / 10, az: 1
      }));
      stationHistory['graph-test'] = history;
      document.getElementById('chartFixture').innerHTML = livePlot('Acceleration X / Y / Z', 'g',
        ['ax', 'ay', 'az'].map((key, i) => liveSeries(history, key,
          ['var(--series-blue)', 'var(--series-orange)', 'var(--series-purple)'][i], ['X', 'Y', 'Z'][i])),
        60, 2, 0, 1500, 3, fixtureNow, 'smooth');
    });
    assert.equal(await page.locator('.seriesTrace').count(), 3);
    assert.equal(await page.locator('.rawTrace').count(), 3);
    assert.deepEqual(await page.locator('.seriesTrace').evaluateAll(paths => paths.map(p => p.getAttribute('stroke-dasharray'))),
      ['', '7 3', '2 3']);
    assert.notEqual(await page.locator('.seriesTrace').first().getAttribute('d'),
      await page.locator('.rawTrace').first().getAttribute('d'));
    const rawSeries = JSON.parse(await page.locator('.liveSvg').getAttribute('data-series'));
    assert.equal(rawSeries[0].points[1].v, 9, 'Cursor values must use raw acquired samples');
    await page.locator('.liveSvg').focus();
    await page.keyboard.press('Home');
    await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('.chartReadoutValue').count(), 3);
    assert((await page.locator('.chartReadoutValue').first().textContent()).includes('9.000 g'));

    const downloadPromise = page.waitForEvent('download');
    await page.evaluate(() => exportHistory('graph-test'));
    const download = await downloadPromise;
    const csv = fs.readFileSync(await download.path(), 'utf8');
    const rows = csv.split('\n').map(line => line.split(','));
    const axColumn = rows[0].indexOf('ax');
    assert.equal(rows[2][axColumn], '9', 'CSV must retain the raw spike even with visual smoothing');

    await page.setViewportSize({width: 390, height: 844});
    await page.locator('.liveSvg').focus();
    await page.keyboard.press('ArrowRight');
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth),
      'Chart legend and exact values must fit a narrow screen');
    assert(await page.locator('.chartReadout').isVisible());
    if (process.env.QA_SCREENSHOT_DIR) {
      fs.mkdirSync(process.env.QA_SCREENSHOT_DIR, {recursive: true});
      await page.screenshot({path: path.join(process.env.QA_SCREENSHOT_DIR, 'chart-mobile.png'), fullPage: true});
      await page.setViewportSize({width: 1280, height: 1000});
      await page.screenshot({path: path.join(process.env.QA_SCREENSHOT_DIR, 'chart-desktop.png'), fullPage: true});
    }
    assert.deepEqual(errors, []);
    console.log('Chart readability: single samples, time windows, raw cursor/CSV, gap-safe smoothing, XYZ styles and mobile layout passed');
  } finally {
    await browser.close();
  }
})().catch(error => {console.error(error);process.exitCode = 1});
