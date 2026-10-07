// Exercise zero references and sensor-type transitions in the real embedded UI.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const mac='02:03:04:05:06:07',now=Date.now(),session='0000000000000042';
  const axes={ax:.125,ay:-.25,az:1,gx:.2,gy:-.3,gz:1};
  const node={mac,name:'Table sensor',provisioned:true,seen:true,online:true,live:true,
    sensorType:3,configuredSensorType:3,capabilities:144,revision:1,appliedRevision:1,
    liveRevision:1,pcbVersion:4,sleepSeconds:10,batteryMv:3900,ageMs:0,profileSlot:255,
    ...axes,peakAcceleration:1.04,peakAngularRate:1.06};
  const saved={session,epochMs:now-3600000,durationMs:10000,availableMs:10000,
    stored:10,expected:10,complete:true,sensorType:3};
  const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:1280,height:1100}}),errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.route('http://station.test/**',route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',thingSpeakChannels:[],defaultSleepSeconds:600};
      else if(url.pathname==='/api/status')body={sensors:[node],wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/recordings')body={sessions:[saved],freeBytes:100000};
      else if(url.pathname==='/api/recording')body={...saved,nextOffset:10,points:Array.from({length:10},(_,i)=>({sampleMs:(i+1)*1000,t:(i+1)*1000,...axes,wave:[]}))};
      else if(url.pathname==='/api/history'){
        const points=Array.from({length:5},(_,i)=>({sensorType:node.sensorType,id:node.sensorType*1000+i,
          ageMs:(4-i)*10000,measurementAgeMs:(4-i)*10000,
          ...(node.sensorType===3?axes:{temperature:23.456})}));
        body={live:true,bucketSeconds:10,points:points.filter(p=>!url.searchParams.has('afterMs')||p.id>Number(url.searchParams.get('afterMs')))};
      }else if(url.pathname==='/api/motion-reference'){
        const params=new URLSearchParams(route.request().postData());
        assert.equal(params.get('csrf'),'test');
        node.motionReference={enabled:params.get('clear')!=='1',...axes};
        body={success:true,message:'Reference saved'};
      }else body={success:true,sensors:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)});
    });
    await page.goto('http://station.test/dashboard');
    await page.getByRole('button',{name:'Zero axes here',exact:true}).waitFor();
    await page.getByRole('button',{name:'Zero axes here',exact:true}).click();
    await page.getByRole('button',{name:'Clear reference',exact:true}).waitFor();
    if(process.env.QA_SCREENSHOT_DIR){
      fs.mkdirSync(process.env.QA_SCREENSHOT_DIR,{recursive:true});
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-reference-desktop.png'),fullPage:true});
      await page.setViewportSize({width:390,height:844});
      assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'motion-reference-mobile.png'),fullPage:true});
      await page.setViewportSize({width:1280,height:1100});
    }
    const values=await page.locator('.sensor .values .value b').allTextContents();
    assert(values.slice(0,3).every(v=>v.startsWith('0.000')));
    assert(values.slice(3,6).every(v=>v.startsWith('0.0')));
    let series=JSON.parse(await page.locator('svg[aria-label="Acceleration X / Y / Z"]').getAttribute('data-series'));
    assert(series.every(s=>s.points.length&&s.points.every(p=>Math.abs(p.v)<1e-6)));
    await page.getByLabel('Recording selection').selectOption(session);
    await page.waitForFunction(()=>document.querySelector('.recordingCharts svg')&&!document.querySelector('.recordingPanel').textContent.includes('Loading'));
    series=JSON.parse(await page.locator('svg[aria-label="Angular rate X / Y / Z"]').getAttribute('data-series'));
    assert(series.every(s=>s.points.length===10&&s.points.every(p=>Math.abs(p.v)<1e-6)));
    const download=page.waitForEvent('download');
    await page.getByRole('button',{name:'Export recording',exact:true}).click();
    const csv=fs.readFileSync(await (await download).path(),'utf8');
    const row=csv.split('\n')[1].split(',');
    assert(row.includes('0.125')&&row.includes('-0.25'));
    await page.getByLabel('Recording selection').selectOption('');
    await page.getByRole('button',{name:'Clear reference',exact:true}).click();
    await page.waitForFunction(()=>!document.querySelector('button[onclick*="setMotionZero"][onclick*="true"]'));
    assert((await page.locator('.sensor .values').textContent()).includes('0.125'));
    Object.assign(node,{configuredSensorType:4,configPending:true,revision:2,telemetryFlags:9});
    await page.waitForFunction(()=>document.querySelector('.sensorIdentity').textContent.includes('TMP117'));
    assert((await page.locator('.sensor').textContent()).includes('last detected: LSM6DSOX'));
    assert(!(await page.locator('.sensor').textContent()).includes('Configured sensor type does not match'),
      'Pending sensor changes cannot inherit a mismatch from the previous driver');
    assert(!(await page.locator('.sensor').textContent()).includes('Environmental sensor could not be read'));
    assert((await page.locator('.sensor').textContent()).includes('initialize the selected sensor'));
    assert.equal(await page.locator('svg[aria-label="Acceleration X / Y / Z"]').count(),0);
    // A genuine mismatch after application must remain visible.
    node.configPending=false;node.appliedRevision=2;
    await page.getByText('Configured sensor type does not match',{exact:true}).waitFor();
    Object.assign(node,{sensorType:4,capabilities:17,temperature:23.456,appliedRevision:2,configPending:false,liveRevision:2,telemetryFlags:0});
    await page.waitForFunction(()=>document.querySelector('.sensor .values').textContent.includes('23.456'));
    assert.equal(await page.locator('svg[aria-label="Temperature trend"]').count(),1);
    assert(!(await page.locator('.sensor').textContent()).includes('last detected: LSM6DSOX'));
    // A name/type change also renders when the node has no new live telemetry.
    Object.assign(node,{name:'Renamed again',seen:false,configuredSensorType:5,configPending:true,revision:3});
    await page.waitForFunction(()=>document.querySelector('.sensor h3').textContent==='Renamed again');
    assert((await page.locator('.sensorIdentity').textContent()).includes('MAX30102'));
    assert.deepEqual(errors,[]);
    console.log('Sensor settings UI: six-axis zero, saved plots, raw CSV, clear reference, pending type, applied type and offline names passed');
  }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
