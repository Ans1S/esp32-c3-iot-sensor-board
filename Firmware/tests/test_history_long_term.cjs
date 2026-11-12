// Two simulated years: selected ranges, raw samples, cache reuse and rollover.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-5,mac='02:03:04:05:06:01';
  let points=Array.from({length:17520},(_,i)=>({t:(now-(17519-i)*3600)*1000,sensorType:1,temperature:i===10000?99:20+i/100000,battery:3.8}));
  const node={mac,name:'Long history',sensorType:1,configuredSensorType:1,provisioned:true,seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:3600,historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,points:points.length,durablePoints:points.length,pendingPoints:0,capacityPoints:18000,intervalSeconds:1}};
  let calls=0;const requests=[];const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage();let errors=0;page.on('pageerror',()=>errors++);
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:3600,thingSpeakChannels:[],dashboard:{}};
      else if(url.pathname==='/api/status')body={sensors:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/history'){
        calls++;const after=Number(url.searchParams.get('after')),until=Number(url.searchParams.get('until'))||now;requests.push({after,until,oldest:Number(url.searchParams.get('oldest'))});
        const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,512);
        const fields=['t','sensorType','temperature','battery'];
        body={format:'array-v1',fields,paged:true,until,oldest:points[0].t/1000,bucketSeconds:3600,nextAfter:selected.length===512&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected.map(p=>fields.map(k=>p[k]??null))};
      }else body={success:true,nodes:[],sessions:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');await page.locator('.sensorSummary').first().click();
    await page.waitForFunction(mac=>stationHistory[mac]?.length===24&&!historyLoadController,mac);
    assert.equal(calls,1,'Opening a long-lived sensor loads only 24 hours');
    const firstRange=await page.evaluate(mac=>({...historyRanges[mac]}),mac),beforeYear=calls;
    await page.getByLabel('Chart time window').selectOption('31536000');
    await page.waitForFunction(mac=>stationHistory[mac]?.length===8760&&!historyLoadController,mac);
    assert.equal(requests[beforeYear].until,firstRange.after,'Year loading requests only the missing older prefix');
    assert.equal(requests[beforeYear].oldest,firstRange.oldest,'The extension validates the original retained boundary');
    assert.equal(await page.evaluate(mac=>Math.max(...stationHistory[mac].map(p=>p.temperature)),mac),99,'Spikes remain in the raw data');
    const yearRange=await page.evaluate(mac=>({...historyRanges[mac]}),mac),beforeTwoYears=calls;
    await page.getByLabel('Chart time window').selectOption('63072000');
    await page.waitForFunction(mac=>stationHistory[mac]?.length===17520&&!historyLoadController,mac);
    assert.equal(requests[beforeTwoYears].until,yearRange.after,'Two years reuses the previously loaded year');
    assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),points.map(p=>p.t),'Expanding retains every timestamp exactly once');
    const callsAfterTwoYears=calls;
    await page.getByLabel('Chart time window').selectOption('86400');
    await page.getByLabel('Chart time window').selectOption('31536000');
    await page.getByLabel('Chart time window').selectOption('63072000');
    assert.equal(calls,callsAfterTwoYears,'Changing to covered periods reuses the complete cached snapshot');
    assert((await page.locator('.historyStorageInfo').textContent()).includes('Oldest history is removed'));
    const gaps=await page.evaluate(()=>{
      const now=Date.now(),samples=Array.from({length:12000},(_,i)=>({t:now-(11999-i)*3600000,v:i===5000?99:20,breakBefore:i>0}));
      const box=document.createElement('div');document.body.append(box);
      const start=performance.now();box.innerHTML=chartSvg(samples,chartDefinitions.temperature,3600,3600,12000*3600,'raw',now);
      const svg=box.querySelector('.historySvg'),result={ms:Math.round(performance.now()-start),markers:svg.querySelectorAll('.sample').length,
        traces:svg.querySelectorAll('.seriesTrace').length,raw:chartData(svg,'samples').length,
        peak:Math.max(...chartReducePoints(samples).map(p=>p.v))};box.remove();return result;
    });
    assert(gaps.markers<=900,'Many gaps must not create thousands of SVG nodes');
    assert.equal(gaps.traces,0,'Gaps must never become connecting lines');
    assert.equal(gaps.raw,12000,'Cursor data retains all raw samples');
    assert.equal(gaps.peak,99,'Visual reduction retains a spike');
    assert.equal(await page.evaluate(()=>{try{decodeHistoryPage({format:'array-v1',fields:['t','t'],points:[[1,2]]});return false}catch(e){return true}}),true,'Invalid columns are rejected');
    assert.equal(errors,0);
    console.log(`Long-term history: 17,520 hourly points, 1/2-year windows, exact spike and cached switches passed (${calls} API pages); 12,000 gaps rendered with ${gaps.markers} markers in ${gaps.ms} ms`);
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
