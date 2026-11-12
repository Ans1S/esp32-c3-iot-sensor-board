// A wider view reuses its snapshot; a recycled oldest page must fail safely.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
 const src=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
 const html=src.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
 const now=Math.floor(Date.now()/1000),mac='02:03:04:05:06:07';
 let points=Array.from({length:2000},(_,i)=>({t:(now-(1999-i)*3600)*1000,sensorType:1,temperature:i,battery:3.8}));
 const node={mac,name:'Range test',sensorType:1,configuredSensorType:1,provisioned:true,seen:true,online:true,capabilities:17,batteryMv:3800,temperature:21,sleepSeconds:3600,historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,points:points.length,durablePoints:points.length,pendingPoints:0,intervalSeconds:1}};
 let staleNext=false;const requests=[];
 const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
 try{
  const page=await browser.newPage();await page.addInitScript(time=>{Date.now=()=>time},now*1000);
  await page.route('http://station.test/**',async route=>{
   const u=new URL(route.request().url());let body;
   if(u.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
   if(u.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:3600,thingSpeakChannels:[],dashboard:{}};
   else if(u.pathname==='/api/status')body={sensors:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
   else if(u.pathname==='/api/history'){
    const after=Number(u.searchParams.get('after')),until=Number(u.searchParams.get('until'))||now,oldest=Number(u.searchParams.get('oldest'));
    requests.push({after,until,oldest});
    if(staleNext){staleNext=false;body={success:false,message:'History changed while loading. Retry the selected window.'};}
    else {const selected=points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,512);body={paged:true,until,oldest:oldest||points[0].t/1000,bucketSeconds:3600,nextAfter:selected.length===512&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};}
   }else body={success:true,nodes:[],sessions:[]};
   return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
  });
  await page.goto('http://station.test/dashboard');await page.locator('.sensorSummary').first().click();
  await page.waitForFunction(mac=>stationHistory[mac]?.length===25&&!historyLoadController,mac);
  const original=await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),range=await page.evaluate(mac=>({...historyRanges[mac]}),mac);
  staleNext=true;await page.getByLabel('Chart time window').selectOption('2592000');
  await page.getByText('History changed while loading.',{exact:false}).waitFor();
  assert.equal(requests[1].until,range.after);assert.equal(requests[1].oldest,range.oldest);
  assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),original,'A stale snapshot must not replace the previous graph');
  points=points.slice(25);node.history.oldest=points[0].t/1000;node.history.points=points.length;node.historyRevision++;
  await page.waitForFunction(()=>!statusRequestActive&&!historyLoadController);
  const next=requests.length;await page.evaluate(()=>loadStatus());
  await page.waitForFunction(mac=>stationHistory[mac]?.length===721&&!historyLoadController,mac);
  assert.equal(requests[next].oldest,0,'Changed retention reloads a fresh snapshot');
  assert.equal(requests[next].until,now,'Changed retention does not reuse the old ceiling');
  assert.deepEqual(await page.evaluate(mac=>stationHistory[mac].map(p=>p.t),mac),points.filter(p=>p.t>=(now-2592000)*1000).map(p=>p.t));
  console.log('History expansion: exact boundary, stale-page recovery and refreshed snapshot passed');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
