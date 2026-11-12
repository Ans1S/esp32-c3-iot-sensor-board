// Controlled browser benchmark; API delays emulate latency, not station timings.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
(async () => {
  const source = fs.readFileSync(process.env.DASHBOARD_SOURCE || path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  const html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const pageSize = Number(process.env.HISTORY_PAGE_SIZE || 512), expectedPoints=Number(process.env.EXPECTED_POINTS||1440), mac = '02:03:04:05:06:01';
  const now = Math.floor(Date.now()/1000)-1;
  const points = Array.from({length:4400}, (_, i) => ({t:(now-(4399-i)*60)*1000,battery:3.7+i/100000,sensorType:255}));
  const sensors = Array.from({length:19}, (_, i) => ({mac:i ? `02:03:04:05:06:${String(i+1).padStart(2,'0')}` : mac,
    name:`Sensor ${i+1}`,sensorType:255,configuredSensorType:255,provisioned:true,seen:true,online:true,
    capabilities:16,batteryMv:3800,sleepSeconds:60,historyRevision:1,normalHistory:true,
    history:{oldest:points[0].t/1000,newest:now,points:4400,durablePoints:4400,pendingPoints:0}}));
  let historyCalls=0, statusStarted=0, configFinished=0;
  const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try {
    const page=await browser.newPage();
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url()); let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config'){
        await new Promise(r=>setTimeout(r,300)); configFinished=Date.now();
        body={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],dashboard:{}};
      }else if(url.pathname==='/api/status'){
        statusStarted ||= Date.now(); await new Promise(r=>setTimeout(r,200));
        body={sensors,wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      }else if(url.pathname==='/api/history'){
        if(!process.env.DASHBOARD_SOURCE)assert.equal(url.searchParams.get('limit'),'512');
        historyCalls++; await new Promise(r=>setTimeout(r,40));
        const after=Number(url.searchParams.get('after')), until=Number(url.searchParams.get('until'))||now;
        const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,pageSize);
        body={paged:true,until,oldest:points[0].t/1000,bucketSeconds:60,
          nextAfter:selected.length===pageSize&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};
      }else body={nodes:[],sessions:[],success:true};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    const start=Date.now(); await page.goto('http://station.test/dashboard');
    await page.waitForFunction(()=>document.querySelectorAll('.sensorSummary').length===19);
    const overviewMs=Date.now()-start;
    const graphStart=Date.now(); await page.locator('.sensorSummary').first().click();
    await page.waitForFunction(({mac,expectedPoints})=>stationHistory[mac]?.length===expectedPoints&&!historyLoadController,{mac,expectedPoints});
    const graphMs=Date.now()-graphStart;
    assert.equal(historyCalls,Math.ceil(expectedPoints/pageSize));
    assert.equal(await page.evaluate(mac=>stationHistory[mac].at(-1).battery,mac),points.at(-1).battery);
    if(!process.env.DASHBOARD_SOURCE)assert(statusStarted<configFinished,'Status request starts while configuration is loading');
    console.log(JSON.stringify({pageSize,expectedPoints,overviewMs,graphMs,historyCalls,statusStartedBeforeConfig:statusStarted<configFinished}));
  } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exit(1)});
