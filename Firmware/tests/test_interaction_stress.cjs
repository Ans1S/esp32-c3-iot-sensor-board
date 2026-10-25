// Exercise the embedded dashboard under overlapping actions and failed pages.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');

(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const browser=await chromium.launch({headless:true,
    ...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:390,height:844},hasTouch:true});
    const errors=[];page.on('pageerror',error=>errors.push(error.message));
    const epoch=Math.floor(Date.now()/1000)-1000;
    const mac=index=>'02:03:04:05:06:'+index.toString(16).padStart(2,'0');
    const nodes=Array.from({length:19},(_,i)=>({mac:mac(i+1),name:'Node '+(i+1),
      sensorType:255,configuredSensorType:255,pcbVersion:i%2?3:4,provisioned:true,
      seen:true,online:true,capabilities:16,batteryMv:3800,sleepSeconds:1,
      normalHistory:true,historyRevision:1,history:{oldest:epoch,newest:epoch+256}}));
    let samples=Array.from({length:257},(_,i)=>({t:(epoch+i)*1000,battery:3.8+i/100000,sensorType:255}));
    const otherSamples=[{t:epoch*1000,battery:4.001,sensorType:255}];
    let uptimeMs=100000,statusFails=false,historyMode='ok',historyReads=0,layoutWrites=0;
    let prefs={columns:0,comfortable:false,order:[],hidden:[]};
    let releaseLayout=null,releaseHistory=null;
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',thingSpeakChannels:[],dashboard:prefs};
      else if(url.pathname==='/api/status'){
        if(statusFails)return route.abort('failed');
        body={uptimeMs,sensors:nodes,wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      }else if(url.pathname==='/api/dashboard-layout'){
        layoutWrites++;
        const form=new URLSearchParams(route.request().postData());
        assert.equal(form.get('csrf'),'test');
        const submitted={columns:Number(form.get('columns')),comfortable:form.get('comfortable')==='1',
          order:form.get('order').split(',').filter(Boolean),hidden:form.get('hidden').split(',').filter(Boolean)};
        await new Promise(resolve=>{releaseLayout=resolve;});
        prefs=submitted;body={success:true};
      }else if(url.pathname==='/api/history'){
        historyReads++;
        const after=Number(url.searchParams.get('after')),selectedMac=url.searchParams.get('mac');
        const mode=historyMode;
        if(mode==='hold'&&selectedMac===mac(1))await new Promise(resolve=>{releaseHistory=resolve;});
        if(mode==='http-error')return route.fulfill({status:503,body:'Temporarily unavailable'});
        if(mode==='crc-error'&&after)body={success:false,message:'History page failed CRC validation.'};
        else{
          const rows=selectedMac===mac(1)?samples:otherSamples;
          const until=Number(url.searchParams.get('until'))||rows.at(-1).t/1000;
          const points=rows.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,128);
          body={paged:true,until,oldest:rows[0].t/1000,bucketSeconds:1,
            nextAfter:mode==='stuck-cursor'&&after?after:
              points.length===128&&points.at(-1).t/1000<until?points.at(-1).t/1000:0,points};
        }
      }else body={success:true,nodes:[],sessions:[]};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.waitForFunction(()=>document.querySelectorAll('.sensorSummary').length===19);
    assert.equal(historyReads,0,'A full overview does not load nineteen histories');

    // A delayed old save must not close a newly opened layout or erase its draft.
    await page.getByRole('button',{name:'Arrange',exact:true}).click();
    await page.locator('#layoutColumns').selectOption('3');
    await page.getByRole('button',{name:'Save layout',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('#saveDashboardLayoutButton').disabled);
    assert.equal(layoutWrites,1);
    await page.getByRole('button',{name:'Cancel',exact:true}).click();
    await page.getByRole('button',{name:'Arrange',exact:true}).click();
    await page.locator('#layoutColumns').selectOption('2');
    await page.getByLabel('Show Node 1',{exact:true}).uncheck();
    releaseLayout();releaseLayout=null;
    await page.waitForFunction(()=>config.dashboard.columns===3);
    assert(await page.locator('#dashboardLayout').isVisible(),'An old save retains the reopened layout');
    assert.equal(await page.locator('#layoutColumns').inputValue(),'2');
    assert.equal(await page.getByLabel('Show Node 1',{exact:true}).isChecked(),false);
    await page.getByRole('button',{name:'Cancel',exact:true}).click();

    // A failed later page must not publish a partial snapshot. Retry via the UI.
    historyMode='crc-error';
    await page.locator('.sensorSummary').first().focus();await page.keyboard.press('Enter');
    await page.getByText('History page failed CRC validation.',{exact:false}).waitFor();
    assert.equal(await page.evaluate(()=>stationHistory[statusData.sensors[0].mac]?.length||0),0);
    historyMode='ok';
    await page.getByRole('button',{name:'Retry',exact:true}).click();
    await page.waitForFunction(()=>stationHistory[statusData.sensors[0].mac]?.length===257);
    const original=await page.evaluate(()=>JSON.stringify(stationHistory[statusData.sensors[0].mac]));

    // An unsuccessful incremental load keeps every original value intact.
    samples.push(...Array.from({length:129},(_,i)=>({t:(epoch+257+i)*1000,battery:4.1,sensorType:255})));
    nodes[0].historyRevision++;
    historyMode='http-error';
    await page.waitForFunction(()=>!statusRequestActive&&!historyLoadController);
    await page.evaluate(()=>loadStatus());
    await page.getByText('History request failed',{exact:false}).waitFor();
    assert.equal(await page.evaluate(()=>JSON.stringify(stationHistory[statusData.sensors[0].mac])),original);
    historyMode='ok';
    await page.getByRole('button',{name:'Retry',exact:true}).click();
    await page.waitForFunction(()=>stationHistory[statusData.sensors[0].mac]?.length===386);

    // Rapid navigation aborts old work; its delayed response cannot replace B.
    historyMode='hold';
    await page.evaluate(()=>{delete historyRevisions[statusData.sensors[0].mac];openSensorDetail(statusData.sensors[0].mac);});
    await page.waitForFunction(()=>Boolean(historyLoadController));
    await page.waitForTimeout(50);
    assert(releaseHistory,'The fixture is holding a real history request');
    await page.getByRole('button',{name:'All sensors',exact:false}).click();
    await page.getByRole('button',{name:'Open Node 2',exact:true}).tap();
    await page.waitForFunction(()=>stationHistory[statusData.sensors[1].mac]?.length===1&&!historyLoadController);
    releaseHistory();releaseHistory=null;
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(()=>selectedSensorMac),mac(2));
    assert.equal(await page.evaluate(()=>stationHistory[selectedSensorMac][0].battery),4.001);

    // A malformed cursor is bounded and leaves an explicit retry action.
    historyMode='stuck-cursor';
    await page.evaluate(()=>{delete stationHistory[statusData.sensors[0].mac];delete historyRevisions[statusData.sensors[0].mac];openSensorDetail(statusData.sensors[0].mac);});
    await page.getByText('History cursor did not advance',{exact:false}).waitFor();
    assert.equal(await page.evaluate(()=>stationHistory[statusData.sensors[0].mac]?.length||0),0);
    historyMode='ok';
    await page.getByRole('button',{name:'Retry',exact:true}).click();
    await page.waitForFunction(()=>stationHistory[statusData.sensors[0].mac]?.length===386&&!historyLoadController);

    // A timeout is a visible fault; only user navigation is a silent abort.
    historyMode='hold';
    await page.evaluate(()=>{delete historyRevisions[statusData.sensors[0].mac];openSensorDetail(statusData.sensors[0].mac);});
    await page.waitForTimeout(50);assert(releaseHistory);
    await page.getByText('History request timed out.',{exact:false}).waitFor({timeout:18000});
    assert.equal(await page.evaluate(()=>stationHistory[statusData.sensors[0].mac].length),386);
    historyMode='ok';releaseHistory();releaseHistory=null;
    await page.getByRole('button',{name:'Retry',exact:true}).click();
    await page.waitForFunction(()=>!historyLoadController&&!historyLoadErrors[statusData.sensors[0].mac]);

    // A station reboot with unchanged revisions replaces the browser cache.
    samples=[{t:(epoch+500)*1000,battery:3.111,sensorType:255}];
    uptimeMs=100;nodes[0].history.oldest=epoch+500;
    await page.waitForFunction(()=>!statusRequestActive);
    await page.evaluate(()=>loadStatus());
    await page.waitForFunction(()=>stationHistory[statusData.sensors[0].mac]?.length===1);
    assert.equal(await page.evaluate(()=>stationHistory[statusData.sensors[0].mac][0].battery),3.111);
    statusFails=true;await page.evaluate(()=>loadStatus());
    await page.getByText('Station unreachable',{exact:true}).first().waitFor();
    statusFails=false;await page.evaluate(()=>loadStatus());
    await page.getByText('Wi-Fi connected',{exact:false}).first().waitFor();
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    assert.deepEqual(errors,[]);
    if(process.env.QA_SCREENSHOT_DIR){fs.mkdirSync(process.env.QA_SCREENSHOT_DIR,{recursive:true});
      await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'interaction-stress-mobile.png'),fullPage:true});}
    console.log('Interaction stress: 19 nodes, reopened drafts, partial/HTTP/timeout/cursor faults, rapid navigation, retry, reboot and outage recovery passed');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1});
