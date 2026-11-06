// Verify that the selected normal-history window appears before older pages.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-5,mac='02:03:04:05:06:07';
  const points=Array.from({length:1884},(_,i)=>({t:(now-(1883-i)*600)*1000,
    sensorType:1,temperature:20+i/1000,battery:3.8}));
  const node={mac,name:'History test',sensorType:1,configuredSensorType:1,provisioned:true,
    seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:600,
    historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,
      points:points.length,durablePoints:points.length,pendingPoints:0}};
  let releaseOlder,notifyOlder,failOlder=true;const requests=[];
  const olderStarted=new Promise(resolve=>notifyOlder=resolve);
  const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage();
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],dashboard:{}};
      else if(url.pathname==='/api/status')body={sensors:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/history'){
        const after=Number(url.searchParams.get('after')),until=Number(url.searchParams.get('until'))||now;
        requests.push({after,until});
        if(requests.length===2){notifyOlder();await new Promise(resolve=>releaseOlder=resolve);}
        if(failOlder&&requests.length===2)return route.fulfill({status:503,body:'unavailable'}).catch(()=>{});
        const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,512);
        body={paged:true,until,oldest:points[0].t/1000,bucketSeconds:600,
          nextAfter:selected.length===512&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};
      }else body={success:true,nodes:[],sessions:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensorSummary').first().click();
    await page.waitForFunction(mac=>historyPartial[mac]&&stationHistory[mac]?.length>0,mac);
    await olderStarted;
    assert(requests[0].after>points[0].t/1000&&requests[1].after===0);
    const preview=await page.evaluate(mac=>stationHistory[mac],mac);
    assert(preview.length>=140&&preview.length<=146,'The complete 24-hour period is loaded');
    assert.equal(preview.at(-1).t,points.at(-1).t);
    assert.equal(await page.getByLabel('Chart time window').isDisabled(),true);
    assert.equal(await page.locator('button').filter({hasText:'Export full history CSV'}).isDisabled(),true);
    await page.getByText('Selected period loaded', {exact:false}).waitFor();
    releaseOlder();
    await page.getByText('History request failed',{exact:false}).waitFor();
    assert.equal(await page.evaluate(mac=>stationHistory[mac].length,mac),preview.length,
      'A failed older page keeps the complete selected period');
    assert.equal(await page.locator('button').filter({hasText:'Export full history CSV'}).isDisabled(),true);
    failOlder=false;
    await page.locator('.historyError button').click();
    await page.waitForFunction(mac=>stationHistory[mac]?.length===1884&&!historyPartial[mac]&&!historyLoadController,mac);
    assert.equal(await page.getByLabel('Chart time window').isEnabled(),true);
    assert.equal(await page.locator('button').filter({hasText:'Export full history CSV'}).isEnabled(),true);
    assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),points.map(p=>p.t));
    console.log('History priority: complete visible window first, fixed snapshot, failure retry and full export gating passed');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
