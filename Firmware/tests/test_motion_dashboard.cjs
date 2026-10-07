// Render the real embedded dashboard against deterministic local API fixtures.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  const html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now = Math.floor(Date.now()/1000)*1000;
  const saved={session:'0000000000001234',epochMs:now-7200000,durationMs:1800000,expected:21000,stored:21000,complete:true,sensorType:3};
  const savedPoints=Array.from({length:21000},(_,i)=>{const t=i<6000?i*50:300000+(i-6000)*100;return{sampleMs:t,t,estimateT:t,ax:Math.sin(i/12),ay:.3,az:1,gx:5,gy:2,gz:0,steps:Math.floor(t/500),activeSeconds:Math.floor(t/1000),peakAcceleration:1.8,wave:[]}});
  let submittedInterval = null;
  let failHistory = false;
  const node = {mac:'02:03:04:05:06:07',name:'Motion test',provisioned:true,seen:true,online:true,
    timingKnown:true,acquisitionAgeMs:5,receivedIntervalMs:100,live:true,liveRevision:1,sensorType:3,configuredSensorType:3,capabilities:144,pcbVersion:4,sleepSeconds:1,
    operatingMode:4,batteryMv:3900,historyRevision:1,revision:1,appliedRevision:1,
    ax:.12,ay:-.24,az:1,gx:12,gy:-24,gz:0,peakAcceleration:1.4,peakAngularRate:30,
    motionSamples:104,ageMs:0,rssi:-55,profileSlot:255};
  function historyFixture() {
    // A type transition starts a fresh acquisition window, even when earlier
    // UI assertions took several seconds on a busy browser host.
    const now=Date.now(),type=node.sensorType,step=type===3?100:type===5?200:1000;
    return Array.from({length:60000/step},(_,i)=>{
      const t=now-60000+(i+1)*step,id=type*100000+i*step;
      const p={id,sensorType:type,t,ageMs:Date.now()-t,measurementAgeMs:Date.now()-t+5};
      if(type===3)Object.assign(p,{ax:Math.sin(i/8),ay:Math.cos(i/8)*.4,az:1,gx:30*Math.sin(i/8),gy:10,gz:0,steps:Math.floor(i/5),activeSeconds:Math.floor(i/10),peakAcceleration:1.5});
      if(type===4)p.temperature=37.10+.02*Math.sin(i/20);
      if(type===5){p.pulseStatus=2;p.quality=95;p.wave=Array.from({length:5},(_,j)=>({ageMs:Date.now()-t+(4-j)*40,red:100000+800*Math.sin((i*5+j)*.5),infrared:120000+900*Math.sin((i*5+j)*.5)}));if(i%5===0){p.heartRate=72+Math.sin(i/20);p.estimateAgeMs=Date.now()-t}}
      return p;
    });
  }
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
      else if(url.pathname==='/api/history') {if(failHistory)return route.abort('failed');body={live:true,bucketSeconds:node.sensorType===3?.1:node.sensorType===5?.2:1,points:historyFixture().filter(p=>!url.searchParams.has('afterMs')||p.id>Number(url.searchParams.get('afterMs')))};}
      else if(url.pathname==='/api/recordings') body={freeBytes:100000,sessions:[saved]};
      else if(url.pathname==='/api/recording') {let offset=Number(url.searchParams.get('offset')||0);if(url.searchParams.has('fromMs'))offset=savedPoints.findIndex(p=>p.sampleMs>=Number(url.searchParams.get('fromMs')));if(offset<0)offset=savedPoints.length;const points=savedPoints.slice(offset,offset+128);body={...saved,nextOffset:offset+points.length,points};}
      else if(url.pathname==='/api/sensor') {submittedInterval=new URLSearchParams(route.request().postData()).get('sleepSeconds');body={success:true,message:'Saved'};}
      else body={sensors:[],success:true};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensor h3').waitFor();
    assert.equal(await page.locator('.sensor .value').count(),9);
    assert.equal(await page.locator('.sensorFeedback').count(),1);
    assert.equal(await page.locator('.feedbackValue').count(),4);
    const feedbackChecks=await page.evaluate(()=>{
      const end=100000;
      const temperatures=Array.from({length:11},(_,i)=>({t:end-10000+i*1000,temperature:37.125}));
      const stable=temperatureFeedback(temperatures,end);
      const drift=temperatureFeedback(temperatures.map((p,i)=>({...p,temperature:37+i*.01})),end);
      const gap=temperatureFeedback(temperatures.filter((p,i)=>i!==5),end);
      const stale=temperatureFeedback(temperatures,end+3000);
      const pulses=Array.from({length:61},(_,i)=>({t:end-60000+i*1000,estimateT:end-60000+i*1000,heartRate:120-i/2,pulseStatus:2,quality:95}));
      return {stable,drift,gap,stale,recovery:pulseRecovery(pulses,120,end),broken:pulseRecovery(pulses.filter((p,i)=>i<20||i>30),120,end),invalid:validPulsePoints([{t:1,heartRate:null},{t:2,heartRate:80,quality:20},{t:3,heartRate:80,gap:true}]).length};
    });
    assert(feedbackChecks.stable.stable&&feedbackChecks.stable.mean===37.125);
    assert(!feedbackChecks.drift.stable&&Math.abs(feedbackChecks.drift.slope-.6)<.00001);
    assert(!feedbackChecks.gap.stable&&!feedbackChecks.stale.stable);
    assert(Math.abs(feedbackChecks.recovery-28.75)<.001);
    assert(Number.isNaN(feedbackChecks.broken)&&feedbackChecks.invalid===0);
    assert.equal(await page.locator('svg.liveSvg').count(),2);
    const loadedPoints=await page.evaluate(()=>getHistory(statusData.sensors[0].mac).length);
    failHistory=true;
    await page.evaluate(async()=>{statusData.sensors[0].liveRevision++;await syncHistories(statusData.sensors)});
    failHistory=false;
    assert.equal(await page.evaluate(()=>getHistory(statusData.sensors[0].mac).length),loadedPoints,
      'A failed refresh must retain the previously loaded graph');
    assert.equal(await page.locator('svg[aria-label="Acceleration X / Y / Z"] path').count(),3);
    assert.equal(await page.locator('svg[aria-label="Angular rate X / Y / Z"] path').count(),3);
    await page.locator('svg.liveSvg').first().hover();
    assert(await page.locator('.liveCursor text').first().textContent());
    await page.locator('.sensorActions button').click();
    assert.equal(await page.locator('#wizardInterval').isVisible(),true);
    assert((await page.locator('#sensorTypeHint').textContent()).includes('50-ms'));
    await page.locator('#wizardSensorType').selectOption('4');
    assert((await page.locator('#sensorTypeHint').textContent()).includes('8-conversion'));
    await page.locator('#wizardSensorType').selectOption('5');
    assert((await page.locator('#sensorTypeHint').textContent()).includes('100 Hz'));
    assert.equal(await page.locator('#wizardInterval').isVisible(),false);
    await page.locator('#wizardSensorType').selectOption('4');
    await page.locator('#wizardInterval .intervalPreset').selectOption('10');
    await page.evaluate(()=>saveSensorWizard());
    assert.equal(submittedInterval,'10'); // Normal cadence is not forced back to 1 s.
    const download = page.waitForEvent('download');
    await page.getByRole('button',{name:'CSV',exact:true}).click();
    assert((await download).suggestedFilename().endsWith('.csv'));
    if(process.env.QA_SCREENSHOT_DIR) {
      fs.mkdirSync(process.env.QA_SCREENSHOT_DIR,{recursive:true});
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-desktop.png'),fullPage:true});
      await page.screenshot({fullPage:true,path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-charts.png')});
    }
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    if(process.env.QA_SCREENSHOT_DIR)
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-mobile.png'),fullPage:true});
    Object.assign(node,{sensorType:4,configuredSensorType:4,capabilities:17,temperature:37.125,liveRevision:2});
    await page.waitForFunction(()=>document.querySelector('.sensorIdentity').textContent.includes('TMP117'));
    assert.equal(await page.locator('.sensor .value').count(),2);
    assert.equal(await page.locator('.feedbackValue').count(),3);
    assert.equal(await page.locator('svg[aria-label="Temperature trend"]').count(),1);
    assert((await page.locator('.sensor .values').textContent()).includes('37.125'));
    Object.assign(node,{sensorType:5,configuredSensorType:5,recording:{state:2,session:'pulse-session',elapsedMs:60000,pending:300,capacity:10912},capabilities:784,heartRate:72,red:100000,infrared:120000,pulseStatus:2,pulseQuality:90,liveRevision:3});
    await page.waitForFunction(()=>document.querySelector('.sensorIdentity').textContent.includes('MAX30102'));
    assert.equal(await page.locator('.sensor .value').count(),4);
    assert.equal(await page.locator('.feedbackValue').count(),5);
    await page.evaluate(()=>{const mac=statusData.sensors[0].mac,t=Date.now();stationHistory[mac]=stationHistory[mac].concat(Array.from({length:5},(_,i)=>({t:t-i*1000,estimateT:t-i*1000,heartRate:120,pulseStatus:2,quality:95,receivedT:t-i*1000})));});
    await page.getByRole('button',{name:'Start recovery',exact:true}).click();
    assert((await page.locator('.sensorFeedback').textContent()).includes('Measuring'));
    assert.equal(await page.locator('svg[aria-label="Optical pulse waveform"] path').count(),2);
    const waveform=await page.locator('svg[aria-label="Optical pulse waveform"]').getAttribute('data-series');
    assert(JSON.parse(waveform)[0].points.length>150);
    if(process.env.QA_SCREENSHOT_DIR) await page.screenshot({fullPage:true,path:path.join(process.env.QA_SCREENSHOT_DIR,'pulse-charts.png')});
    await page.locator('[aria-label="Pulse waveform window"]').selectOption('60');
    assert(JSON.parse(await page.locator('svg[aria-label="Optical pulse waveform"]').getAttribute('data-series'))[0].points.length>1200);
    if(process.env.QA_SCREENSHOT_DIR) await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'pulse-mobile.png'),fullPage:true});
    assert((await page.locator('.sensor .values').textContent()).includes('72.0'));
    Object.assign(node,{recording:{state:1},liveRevision:4});
    await page.getByText('Measurement stopped · press SW2 on the sensor to start',{exact:true}).waitFor();
    assert(!(await page.locator('.sensor .values').textContent()).includes('72.0'));
    Object.assign(node,{recording:{state:2},capabilities:272,heartRate:0,pulseStatus:0,liveRevision:5});
    await page.waitForFunction(()=>document.querySelector('.sensor .values').textContent.includes('No usable infrared contact signal'));
    assert(!(await page.locator('.sensor .values').textContent()).includes('0.0 bpm'));
    Object.assign(node,{recording:{state:1,pending:0,capacity:22176,elapsedMs:0,dropped:0},telemetryFlags:1,liveRevision:5});
    await page.getByText('Ready · press SW2 to record',{exact:true}).waitFor();
    assert(!(await page.locator('.sensor').textContent()).includes('Environmental sensor could not be read'));
    await page.getByRole('button',{name:'Recordings',exact:true}).click();
    await page.getByLabel('Recording selection').selectOption(saved.session);
    await page.waitForFunction(()=>document.querySelector('svg[aria-label="Acceleration X / Y / Z"]')&& !document.querySelector('.recordingPanel').textContent.includes('Loading'));
    const savedChart=page.locator('svg[aria-label="Acceleration X / Y / Z"]');
    assert.equal(Number(await savedChart.getAttribute('data-start')),saved.epochMs);
    assert.equal(Number(await savedChart.getAttribute('data-window')),saved.durationMs);
    await page.getByLabel('Recording view',{exact:true}).selectOption('minute');
    await page.getByRole('button',{name:'Previous minute',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('.recordingPanel').textContent.includes('1680–1740'));
    const savedDownload=page.waitForEvent('download');
    await page.getByRole('button',{name:'Export recording',exact:true}).click();
    const exported=await savedDownload;
    assert.equal(exported.suggestedFilename(),'recording-'+saved.session+'.csv');
    const csv=fs.readFileSync(await exported.path(),'utf8');
    assert.equal(csv.split('\n').length,21001);
    assert(csv.split('\n')[0].includes('steps,active_seconds'));
    assert.equal(csv.split('\n')[1].split(',')[16],'0');
    if(process.env.QA_SCREENSHOT_DIR){await page.evaluate(()=>document.documentElement.dataset.theme='dark');await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'recording-mobile-dark.png'),fullPage:true});await page.setViewportSize({width:1280,height:1100});await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'recording-desktop-dark.png'),fullPage:true});}
    await page.getByLabel('Recording selection').selectOption('');
    assert.equal(await page.locator('svg[aria-label="Optical pulse waveform"]').count(),1);
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    Object.assign(node,{live:false,operatingMode:6,telemetryFlags:256,capabilities:48,batteryMv:2790,recording:undefined,liveRevision:6});
    await page.getByText('Battery protection active',{exact:true}).waitFor();
    assert.equal(await page.locator('.sensor .values .value').count(),1);
    assert((await page.locator('.sensor .values').textContent()).includes('2.790 V'));
    assert(!(await page.locator('.sensor').textContent()).includes('Environmental sensor could not be read'));
    assert.equal(await page.locator('.sensor .modeBadge').textContent(),'Battery protection');
    Object.assign(node,{telemetryFlags:258,capabilities:32,batteryMv:4000});
    await page.waitForFunction(()=>!document.querySelector('.sensor .values').textContent.includes('2.790'));
    assert(!(await page.locator('.sensor .values').textContent()).includes('4.000'));
    assert.deepEqual(errors,[]);
    console.log('Live dashboard: XYZ overlays, acquisition timing, temperature scale, 25 Hz waveform, pulse window, CSV and mobile layout passed');
  } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exitCode=1});
