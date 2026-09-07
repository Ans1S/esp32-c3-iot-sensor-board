// Render the real embedded dashboard against deterministic local API fixtures.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  const html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now = Math.floor(Date.now()/1000)*1000;
  const node = {mac:'02:03:04:05:06:07',name:'Motion test',provisioned:true,seen:true,online:true,
    sensorType:3,configuredSensorType:3,capabilities:144,pcbVersion:4,sleepSeconds:1,
    operatingMode:4,batteryMv:3900,historyRevision:1,revision:1,appliedRevision:1,
    ax:.12,ay:-.24,az:1,gx:12,gy:-24,gz:0,peakAcceleration:1.4,peakAngularRate:30,
    motionSamples:104,ageMs:0,rssi:-55,profileSlot:255};
  const points = Array.from({length:60},(_,i)=>({t:now-(59-i)*1000,ax:Math.sin(i/4),ay:0,az:1,gx:0,gy:0,gz:0,peakAcceleration:1.4,peakAngularRate:30,battery:3.9}));
  const browser = await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try {
    const page = await browser.newPage({viewport:{width:1280,height:1100}});
    const errors=[]; page.on('pageerror',e=>errors.push(e.message));
    await page.route('http://station.test/**', route => {
      const url = new URL(route.request().url());
      let body;
      if(url.pathname==='/dashboard') return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config') body={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],wifiSsid:'test',adminPasswordSet:true};
      else if(url.pathname==='/api/status') body={sensors:[node],wifi:{connected:true,ssid:'test',ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/history') body={bucketSeconds:1,points};
      else body={sensors:[],success:true};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensor h3').waitFor();
    assert.equal(await page.locator('.sensor .value').count(),9);
    assert.equal(await page.locator('[aria-label="History metric"] option').count(),9);
    await page.locator('[aria-label="Chart time window"]').selectOption('30');
    assert(await page.locator('svg.chart .sample').count() <= 31);
    await page.locator('.sensorActions button').click();
    assert.equal(await page.locator('#wizardInterval .intervalSeconds').inputValue(),'1');
    await page.locator('#wizardInterval .intervalPreset').selectOption('custom');
    await page.locator('#wizardInterval .customMinutes').fill('7');
    assert.equal(await page.locator('#wizardInterval .intervalSeconds').inputValue(),'7');
    await page.evaluate(()=>closeWizard());
    const download = page.waitForEvent('download');
    await page.getByRole('button',{name:'CSV',exact:true}).click();
    assert((await download).suggestedFilename().endsWith('.csv'));
    if(process.env.QA_SCREENSHOT_DIR) {
      fs.mkdirSync(process.env.QA_SCREENSHOT_DIR,{recursive:true});
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-desktop.png'),fullPage:true});
    }
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    if(process.env.QA_SCREENSHOT_DIR)
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-mobile.png'),fullPage:true});
    assert.deepEqual(errors,[]);
    console.log('Motion dashboard: axes, seconds, chart window, CSV and mobile layout passed');
  } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exitCode=1});
