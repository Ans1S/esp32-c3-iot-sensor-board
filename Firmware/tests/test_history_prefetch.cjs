// Bounded background prefetch should speed up seven-day selection safely.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-5,mac='02:03:04:05:06:07';
  const points=Array.from({length:2000},(_,i)=>({t:(now-(1999-i)*600)*1000,sensorType:1,temperature:20+i/1000,battery:3.8}));
  const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    for(const mode of ['inflight','ready','large','live','export','fail','cap','cancel']){
      const page=await browser.newPage({acceptDownloads:true}),requests=[];let release,notify;
      const prefetchStarted=new Promise(resolve=>notify=resolve);
      const node={mac,name:'Prefetch test',sensorType:1,configuredSensorType:1,provisioned:true,seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:600,historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,points:points.length,durablePoints:points.length,pendingPoints:0,intervalSeconds:mode==='large'?1:600}};
      await page.route('http://station.test/**',async route=>{
        const url=new URL(route.request().url());let body;
        if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
        if(url.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],dashboard:{}};
        else if(url.pathname==='/api/status')body={sensors:mode==='live'?[node,{...node,mac:'02:03:04:05:06:08',name:'Live sensor',live:true,normalHistory:false}]:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
        else if(url.pathname==='/api/history'){
          const after=Number(url.searchParams.get('after')),until=Number(url.searchParams.get('until'))||now;
          requests.push({after,until});
          const firstPrefetch=mode==='cap'?3:2;
          if(requests.length===firstPrefetch){notify();if(mode==='inflight'||mode==='export'||mode==='cap'||mode==='cancel')await new Promise(resolve=>release=resolve);if(mode==='fail')return route.fulfill({status:503,body:'unavailable'}).catch(()=>{});}
          const limit=mode==='cap'?128:512,selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,limit);
          body={paged:true,until,oldest:points[0].t/1000,bucketSeconds:600,nextAfter:selected.length===limit&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};
        }else body={success:true,nodes:[],sessions:[]};
        return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
      });
      await page.goto('http://station.test/dashboard');await page.locator('.sensorSummary').first().click();
      await page.waitForFunction(mac=>stationHistory[mac]?.length===144,mac);
      if(mode==='large'||mode==='live'){
        await page.waitForTimeout(300);
        assert.equal(requests.length,1,'High-rate histories and active live sensors must not trigger background prefetch');
      }else{
        await prefetchStarted;
        if(mode==='cancel'){
          await page.getByLabel('Chart time window').selectOption('31536000');release();
          await page.waitForFunction(mac=>stationHistory[mac]?.length===2000&&!historyLoadController,mac);
          assert.equal(await page.getByLabel('Chart time window').inputValue(),'31536000');
          await page.close();continue;
        }else if(mode==='export'){
          await page.locator('.chartOptions summary').click();
          const pending=page.waitForEvent('download',{timeout:5000});
          await page.getByRole('button',{name:'Export full history CSV',exact:true}).click();
          release();const download=await pending;
          assert.equal(fs.readFileSync(await download.path(),'utf8').trim().split('\n').length,points.length+1,'Export cancels prefetch and includes the entire retained history');
          await page.close();continue;
        }else if(mode==='fail'){
          await page.waitForFunction(()=>!historyLoadController);
          assert.equal(await page.evaluate(mac=>historyLoadErrors[mac]??null,mac),null,'A failed background prefetch is silent');
          await page.getByLabel('Chart time window').selectOption('604800');
        }else if(mode==='inflight'||mode==='cap'){
          assert.equal(await page.evaluate(mac=>Boolean(historyLoading[mac]),mac),false,'Background prefetch must not mark the current view as loading');
          await page.getByLabel('Chart time window').selectOption('604800');
          assert.equal(requests.length,mode==='cap'?3:2,'Selecting seven days adopts the in-flight request');
          release();
        }else await page.waitForFunction(mac=>stationHistory[mac]?.length===1008&&!historyLoadController,mac);
        await page.waitForFunction(mac=>stationHistory[mac]?.length===1008&&!historyLoadController,mac);
        const completed=requests.length;
        await page.getByLabel('Chart time window').selectOption('604800');
        assert.equal(requests.length,completed,'A prefetched period needs no extra request');
        assert.equal(await page.evaluate(mac=>stationHistory[mac].at(-1).temperature,mac),points.at(-1).temperature);
      }
      release?.();await page.close();
    }
    console.log('History prefetch: bounded load, adoption in flight, cached selection, high-rate guard, failed prefetch and full export passed');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
