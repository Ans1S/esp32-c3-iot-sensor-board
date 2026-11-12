// Window loading, failure recovery and complete on-demand CSV export.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-5,mac='02:03:04:05:06:07';
  const points=Array.from({length:1884},(_,i)=>({t:(now-(1883-i)*600)*1000,sensorType:1,temperature:20+i/1000,battery:3.8}));
  const node={mac,name:'History test',sensorType:1,configuredSensorType:1,provisioned:true,seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:600,historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,points:points.length,durablePoints:points.length,pendingPoints:0,intervalSeconds:1}};
  let failNext=false;const requests=[];
  const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage({acceptDownloads:true});
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],dashboard:{}};
      else if(url.pathname==='/api/status')body={sensors:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/history'){
        const after=Number(url.searchParams.get('after')),until=Number(url.searchParams.get('until'))||now;
        requests.push({after,until});
        if(failNext){failNext=false;return route.fulfill({status:503,body:'unavailable'}).catch(()=>{});}
        const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,512);
        assert.equal(url.searchParams.get('format'),'array-v1');
        const fields=['t','sensorType','temperature','battery','iaq','gap'];
        body={paged:true,format:'array-v1',fields,until,oldest:points[0].t/1000,bucketSeconds:600,nextAfter:selected.length===512&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected.map(p=>fields.map(k=>p[k]??null))};
      }else body={success:true,nodes:[],sessions:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensorSummary').first().click();
    await page.waitForFunction(mac=>stationHistory[mac]?.length===144&&!historyLoadController,mac);
    assert.equal(requests.length,1,'Only the selected 24-hour period is loaded');
    assert.equal(await page.getByLabel('Chart time window').isEnabled(),true);
    const preview=await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac);
    failNext=true;
    await page.getByLabel('Chart time window').selectOption('604800');
    await page.getByText('History request failed',{exact:false}).waitFor();
    assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),preview,'A failed request keeps existing points');
    await page.locator('.historyError button').click();
    await page.waitForFunction(mac=>stationHistory[mac]?.length===1008&&!historyLoadController,mac);
    assert.equal(await page.getByLabel('Chart time window').inputValue(),'604800');
    assert.equal(await page.evaluate(mac=>historyPartial[mac],mac),true);
    assert.equal(await page.evaluate(mac=>stationHistory[mac].at(-1).temperature,mac),points.at(-1).temperature);
    assert.equal(await page.evaluate(mac=>'iaq' in stationHistory[mac].at(-1),mac),false,'Absent values remain absent');
    const recentFirst=await page.evaluate(mac=>stationHistory[mac][0].t,mac);
    await page.getByRole('button',{name:'Previous window',exact:true}).click();
    await page.waitForFunction(({mac,recentFirst})=>stationHistory[mac]?.[0]?.t<recentFirst&&!historyLoadController,{mac,recentFirst});
    assert(requests.some(x=>x.until&&x.until<now),'Earlier navigation requests its own bounded time window');
    await page.getByRole('button',{name:'Now',exact:true}).click();
    await page.waitForFunction(mac=>stationHistory[mac]?.length===1008&&!historyLoadController,mac);
    await page.locator('.chartOptions summary').click();
    const downloadPromise=page.waitForEvent('download',{timeout:6000});
    await page.locator('button').filter({hasText:'Export full history CSV'}).click();
    const download=await downloadPromise;
    const csv=fs.readFileSync(await download.path(),'utf8');
    assert.equal(csv.trim().split('\n').length,points.length+1,'Full export includes every retained point');
    assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),points.map(p=>p.t));
    assert.equal(await page.evaluate(mac=>historyPartial[mac],mac),false);
    console.log('History priority: selected window, failure retry and complete on-demand CSV export passed');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
