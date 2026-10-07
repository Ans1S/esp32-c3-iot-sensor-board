// Exercise archive/live selection while the embedded dashboard receives telemetry.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/src/web_pages.cpp'), 'utf8');
  const html = source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const mac = '02:03:04:05:06:07', now = Date.now(), errors = [];
  const motion = {mac,name:'Motion test',provisioned:true,seen:true,online:true,
    sensorType:3,configuredSensorType:3,capabilities:144,pcbVersion:4,
    live:true,liveRevision:1,historyRevision:1,operatingMode:4,sleepSeconds:10,
    revision:1,appliedRevision:1,batteryMv:3900,ax:.1,ay:.2,az:1,gx:2,gy:3,gz:4,
    ageMs:0,rssi:-50,profileSlot:255,
    recording:{state:2,session:'active-session',elapsedMs:12000,pending:240,capacity:22176}};
  const climate = {...motion,mac:'02:03:04:05:06:08',name:'Climate test',
    sensorType:1,configuredSensorType:1,capabilities:23,live:false,sleepSeconds:900,
    temperature:22.5,humidity:50,pressure:1010,recording:undefined};
  const saved = {session:'0000000000001234',epochMs:now-3600000,durationMs:90000,
    availableMs:90000,expected:1800,stored:1800,complete:true,sensorType:3};
  const partial = {...saved,session:'0000000000005678',durationMs:1800000,
    complete:false,expected:21000,stored:280,availableMs:14000};
  const archived = Array.from({length:1800},(_,i)=>({sampleMs:(i+1)*50,
    t:(i+1)*50,estimateT:(i+1)*50,ax:Math.sin(i/10),ay:.2,az:1,gx:2,gy:3,gz:4,wave:[]}));
  let statusCalls=0,recordingCalls=0,slowNextRecording=false,failedNextRecording=false;
  const browser = await chromium.launch({headless:true,
    ...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try {
    const page = await browser.newPage({viewport:{width:1280,height:1000}});
    page.on('pageerror',e=>errors.push(e.message));
    await page.route('http://station.test/**',async route => {
      const url = new URL(route.request().url());
      let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',defaultSleepSeconds:600,
        thingSpeakChannels:[],wifiSsid:'test',adminPasswordSet:true};
      else if(url.pathname==='/api/status') {
        statusCalls++;
        body={sensors:[motion,climate],wifi:{connected:true,ssid:'test',ip:'192.0.2.1'},cloud:{}};
      } else if(url.pathname==='/api/history') {
        if(url.searchParams.get('mac')===mac) {
          const end=Date.now(),base=motion.liveRevision*100000;
          body={live:true,bucketSeconds:.1,points:Array.from({length:20},(_,i)=>({
            id:base+i,sensorType:3,ageMs:(19-i)*100,measurementAgeMs:(19-i)*100,
            ax:motion.ax,ay:.2,az:1,gx:2,gy:3,gz:4
          })).filter(p=>p.id>Number(url.searchParams.get('afterMs')||0))};
        } else body={bucketSeconds:900,points:[{t:Date.now()-300000,temperature:22.5}]};
      } else if(url.pathname==='/api/recordings')body={freeBytes:100000,sessions:url.searchParams.get('mac')===mac?[saved,partial]:[]};
      else if(url.pathname==='/api/recording') {
        recordingCalls++;
        if(slowNextRecording){slowNextRecording=false;await new Promise(resolve=>setTimeout(resolve,4500))}
        if(failedNextRecording){failedNextRecording=false;body={success:false,message:'Recording read unavailable'}}
        else {
          let offset=Number(url.searchParams.get('offset')||0);
          if(url.searchParams.has('fromMs'))offset=archived.findIndex(p=>p.sampleMs>=Number(url.searchParams.get('fromMs')));
          if(offset<0)offset=archived.length;
          const info=url.searchParams.get('session')===partial.session?partial:saved;
          const points=archived.slice(offset,Math.min(offset+128,info.stored));
          body={...info,
            nextOffset:offset+points.length,points};
        }
      } else body={success:true,sensors:[]};
      // A superseded archive request can be aborted before this fixture replies.
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.waitForFunction(()=>document.querySelector('[aria-label="Recording selection"]')?.options.length===3);
    const selector=page.getByLabel('Recording selection');
    assert.equal(await page.locator('.recordingPanel').count(),1,'BME normal history does not show manual recordings');
    await selector.focus();
    await page.evaluate(()=>{window.savedSelect=document.querySelector('[aria-label="Recording selection"]');window.savedCard=document.querySelector('.sensor');});
    motion.ax=.876;motion.liveRevision++;
    await page.waitForFunction(()=>document.querySelector('.sensor .values').textContent.includes('0.876'));
    assert(await page.evaluate(()=>savedSelect===document.querySelector('[aria-label="Recording selection"]')&&savedSelect===document.activeElement&&savedCard===document.querySelector('.sensor')),
      'Telemetry keeps the selector and its focus instead of replacing the card');

    await page.locator('svg[aria-label="Acceleration X / Y / Z"]').hover();
    await page.evaluate(()=>window.savedSvg=document.querySelector('svg[aria-label="Acceleration X / Y / Z"]'));
    motion.ax=.222;motion.liveRevision++;
    await page.waitForFunction(()=>document.querySelector('.sensor .values').textContent.includes('0.222'));
    assert(await page.evaluate(()=>savedSvg===document.querySelector('svg[aria-label="Acceleration X / Y / Z"]')&&savedSvg.parentElement.querySelector('.chartReadoutTime')),
      'A visible hover readout survives telemetry updates');

    slowNextRecording=true;
    await selector.selectOption(saved.session);
    await page.waitForFunction(()=>document.querySelector('.recordingPanel').textContent.includes('Loading recording'));
    const callsDuringLoad=statusCalls;
    await page.waitForTimeout(1100);
    assert(statusCalls<=callsDuringLoad+1,'Archive reads reserve bandwidth while an existing status request can finish');
    assert.equal(await page.locator('.recordingCharts svg.liveSvg').count(),0,
      'No partial chart is displayed before the full snapshot has loaded');
    await page.waitForFunction(()=>document.querySelector('.recordingCharts svg.liveSvg')&&!document.querySelector('.recordingPanel').textContent.includes('Loading recording'),null,{timeout:10000});
    assert.equal(await selector.inputValue(),saved.session);
    assert.equal(await page.locator('.recordingCharts svg.liveSvg').count(),2);
    assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].points.length),1800,
      'A response slower than the former 4 s timeout still displays the saved recording');
    assert.equal(Number(await page.locator('.recordingCharts svg.liveSvg').first().getAttribute('data-window')),90000,
      'The default graph spans the entire recording, including data older than one minute');
    const callsBeforeDetail=recordingCalls;
    await page.getByLabel('Recording view',{exact:true}).selectOption('minute');
    await page.getByRole('button',{name:'Previous minute',exact:true}).click();
    assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].endMs),60000);
    await page.getByLabel('Recording view',{exact:true}).selectOption('full');
    assert.equal(recordingCalls,callsBeforeDetail,'Detail navigation uses the loaded snapshot without refetching');

    await selector.selectOption('');
    motion.ax=.654;motion.liveRevision++;
    await page.waitForFunction(()=>document.querySelector('.sensor .values').textContent.includes('0.654'));
    assert.equal(await page.locator('.recordingCharts').count(),0);
    assert(await page.evaluate(()=>JSON.parse(document.querySelector('svg[aria-label="Acceleration X / Y / Z"]').dataset.series)[0].points.some(p=>p.v===.654)),
      'Live charts resume while SW2 recording is still active');

    await selector.selectOption(partial.session);
    await page.getByText('No partial graph is drawn automatically.',{exact:false}).waitFor();
    assert.equal(await page.locator('.recordingCharts svg.liveSvg').count(),0,
      'Incomplete transfers show a waiting state instead of progressively redrawing a partial graph');
    await page.getByRole('button',{name:'Show available data',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('.recordingCharts svg.liveSvg')&&!recordingViews[statusData.sensors[0].mac].busy);
    assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].endMs),14000,
      'Partially synchronized recordings open the data already stored, rather than an empty future minute');
    assert.equal(await page.locator('.recordingCharts svg.liveSvg').count(),2);

    // Reproduce the reported flicker: synchronization metadata grows while a
    // selected graph is being inspected. No archive reload/DOM replacement.
    await page.locator('.recordingCharts svg.liveSvg').first().hover();
    await page.evaluate(()=>{window.archiveSvg=document.querySelector('.recordingCharts svg.liveSvg');window.archiveSeries=archiveSvg.dataset.series;window.archiveTime=archiveSvg.dataset.cursorTime;window.archiveReadout=archiveSvg.parentElement.querySelector('.chartReadout').textContent;});
    const callsBeforeGrowth=recordingCalls;
    for(let i=0;i<3;i++){
      partial.stored+=40;partial.availableMs=partial.stored*50;
      motion.recording.state=3;motion.recording.pending-=40;motion.liveRevision++;
      await page.evaluate(()=>loadRecordings(statusData.sensors[0].mac,true));
      await page.evaluate(()=>loadStatus());
      assert(await page.evaluate(()=>archiveSvg===document.querySelector('.recordingCharts svg.liveSvg')&&archiveSvg.dataset.series===archiveSeries&&archiveSvg.dataset.cursorTime===archiveTime&&archiveSvg.parentElement.querySelector('.chartReadout').textContent===archiveReadout));
      assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].endMs),14000);
      assert.equal(await page.locator('.recordingPanel [data-ui-key="recording-progress"]').count(),0);
    }
    assert.equal(recordingCalls,callsBeforeGrowth,'Growing synchronization never reloads the selected graph');
    await page.getByText('New synchronized data available.',{exact:false}).waitFor();
    await page.getByRole('button',{name:'Load latest data',exact:true}).click();
    await page.waitForFunction(()=>!recordingViews[statusData.sensors[0].mac].busy&&recordingViews[statusData.sensors[0].mac].points.length===400);
    assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].endMs),20000);

    // Default behavior waits, then publishes all points atomically once the
    // station marks the transfer complete. No progressively growing chart.
    await selector.selectOption('');await selector.selectOption(partial.session);
    await page.waitForFunction(()=>recordingViews[statusData.sensors[0].mac].waitingForSync);
    const callsBeforeComplete=recordingCalls;
    Object.assign(partial,{complete:true,expected:400,durationMs:20000});
    await page.evaluate(()=>loadRecordings(statusData.sensors[0].mac,true));
    await page.waitForFunction(()=>document.querySelector('.recordingCharts svg.liveSvg')&&!recordingViews[statusData.sensors[0].mac].busy);
    assert.equal(await page.evaluate(()=>recordingViews[statusData.sensors[0].mac].points.length),400);
    assert.equal(recordingCalls,callsBeforeComplete+4,'A completed transfer triggers exactly one full paginated load');

    slowNextRecording=true;
    await selector.selectOption(saved.session);
    await page.waitForFunction(()=>recordingViews[statusData.sensors[0].mac].busy);
    await selector.selectOption('');
    await selector.selectOption(partial.session);
    await page.waitForFunction(()=>recordingViews[statusData.sensors[0].mac].selected==='0000000000005678'&&!recordingViews[statusData.sensors[0].mac].busy);
    assert.equal(await selector.inputValue(),partial.session,'A cancelled older request cannot replace the current selection');
    assert.equal(await page.evaluate(()=>recordingTransfers),0);

    failedNextRecording=true;
    await page.evaluate(()=>selectRecording(statusData.sensors[0].mac,'0000000000001234'));
    await page.getByText('Recording read unavailable',{exact:false}).waitFor();
    assert.equal(await page.getByRole('button',{name:'Retry',exact:true}).count(),1);
    await selector.selectOption('');
    assert.equal(await page.getByRole('button',{name:'Retry',exact:true}).count(),0);

    await selector.selectOption(partial.session);
    await page.waitForFunction(()=>!recordingViews[statusData.sensors[0].mac].busy);
    slowNextRecording=true;const callsBeforeExport=recordingCalls;
    await page.evaluate(()=>{window.pendingExport=exportRecording(statusData.sensors[0].mac);exportRecording(statusData.sensors[0].mac)});
    await page.getByRole('button',{name:'Cancel export',exact:true}).waitFor();
    await page.waitForFunction(()=>recordingViews[statusData.sensors[0].mac].exportBusy);
    assert.equal(await page.getByRole('button',{name:'Export recording',exact:true}).isEnabled(),false);
    assert.equal(await page.getByRole('button',{name:'Delete recording',exact:true}).isEnabled(),false);
    assert.equal(recordingCalls,callsBeforeExport+1,'Repeated exports cannot compete for the station archive');
    await page.getByRole('button',{name:'Cancel export',exact:true}).click();
    await page.evaluate(()=>pendingExport);
    assert.equal(await page.evaluate(()=>recordingTransfers),0,'Cancellation releases archive bandwidth for telemetry');
    assert.equal(await page.getByRole('button',{name:'Export recording',exact:true}).isEnabled(),true);
    assert.equal(await page.getByRole('button',{name:'Cancel export',exact:true}).count(),0);
    assert.deepEqual(errors,[]);
    console.log('Recording dashboard: full snapshots, fixed graph/cursor during synchronization, explicit refresh, cached minute navigation, slow responses and cancellation passed');
  } finally {await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
