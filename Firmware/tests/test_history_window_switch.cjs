// A slow older-history page must not prevent selecting a different period.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-5,mac='02:03:04:05:06:07';
  const points=Array.from({length:2000},(_,i)=>({t:(now-(1999-i)*600)*1000,sensorType:1,temperature:20+i/1000,battery:3.8}));
  const node={mac,name:'History test',sensorType:1,configuredSensorType:1,provisioned:true,seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:600,historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,points:points.length,durablePoints:points.length,pendingPoints:0}};
  let releaseOlder,notifyOlder;const olderStarted=new Promise(resolve=>notifyOlder=resolve),requests=[];
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
        if(requests.length===1){notifyOlder();await new Promise(resolve=>releaseOlder=resolve);}
        const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,512);
        body={paged:true,until,oldest:points[0].t/1000,bucketSeconds:600,nextAfter:selected.length===512&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};
      }else body={success:true,nodes:[],sessions:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensorSummary').first().click();
    await olderStarted;
    assert.equal(await page.getByLabel('Chart time window').isEnabled(),true,'Period must remain selectable while a history page is pending');
    await page.getByLabel('Chart time window').selectOption('604800',{timeout:2000});
    await page.waitForFunction(()=>Number(document.querySelector('[aria-label="Chart time window"]').value)===604800);
    await page.waitForFunction(()=>!historyLoadController);
    assert(requests.some(x=>x.after>0&&x.after<=now-604800+10),'Seven-day request must start without waiting for the previous request');
    assert.equal(await page.evaluate(mac=>stationHistory[mac].length,mac),1008,'The complete selected window is retained');
    releaseOlder();
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(mac=>stationHistory[mac].length,mac),1008,'An obsolete response cannot replace the new window');
    assert.equal(await page.getByLabel('Chart time window').inputValue(),'604800');
    console.log('History window switch: seven-day selection is available while the previous background page is stalled');
  }finally{releaseOlder?.();await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
