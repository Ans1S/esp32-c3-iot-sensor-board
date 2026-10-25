// Exercise real overview/detail navigation, station-backed layout and paged history.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const html=source.match(/const char kDashboardPage\[\].*?R"WCH\((.*?)\)WCH";/s)[1];
  const now=Math.floor(Date.now()/1000)-3,mac='02:03:04:05:06:07',other='02:03:04:05:06:08';
  const points=Array.from({length:4400},(_,i)=>({t:(now-(4399-i)*900)*1000,
    battery:3.7+i/100000,sensorType:255,pcbVersion:4}));
  points[2100].battery=4.123;
  const nodes=[{mac,name:'Battery node',sensorType:255,configuredSensorType:255,pcbVersion:4,
    provisioned:true,seen:true,online:true,capabilities:16,batteryMv:3805,sleepSeconds:900,
    historyRevision:1,normalHistory:true,history:{oldest:points[0].t/1000,newest:now,
      points:4400,durablePoints:4400,pendingPoints:0,capacityPoints:23000}},
    {mac:other,name:'Climate node',sensorType:1,configuredSensorType:1,pcbVersion:3,
      provisioned:true,seen:true,online:true,capabilities:23,temperature:21.4,humidity:45,
      pressure:1013,batteryMv:3900,sleepSeconds:600,normalHistory:true,historyRevision:1}];
  let prefs={columns:0,comfortable:false,order:[],hidden:[]},historyCalls=[],savedLayout=null;
  const errors=[],browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:1280,height:1000},hasTouch:true});
    const checkBackSpacing=async()=>{
      const button=await page.locator('#sensorDetailBack button').boundingBox();
      const card=await page.locator('#sensorGrid.detailGrid > .sensor').first().boundingBox();
      assert(button&&card&&card.y-button.y-button.height>=15.9,
        'The back button must have at least 16 CSS pixels of separation from the sensor card');
    };
    page.on('pageerror',e=>errors.push(e.message));
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test',thingSpeakChannels:[],defaultSleepSeconds:600,dashboard:prefs};
      else if(url.pathname==='/api/status')body={sensors:nodes,wifi:{connected:true,ip:'192.0.2.1'},cloud:{}};
      else if(url.pathname==='/api/dashboard-layout'){
        const form=new URLSearchParams(route.request().postData());assert.equal(form.get('csrf'),'test');
        prefs={columns:Number(form.get('columns')),comfortable:form.get('comfortable')==='1',
          order:form.get('order').split(',').filter(Boolean),hidden:form.get('hidden').split(',').filter(Boolean)};
        savedLayout=prefs;body={success:true};
      }else if(url.pathname==='/api/history'){
        historyCalls.push(url.searchParams.get('mac'));assert.equal(url.searchParams.get('source'),'normal');
        const after=Number(url.searchParams.get('after')),until=Number(url.searchParams.get('until'))||now;
        const selected=url.searchParams.get('mac')===mac?points.filter(p=>p.t/1000>after&&p.t/1000<=until).slice(0,128):[];
        body={paged:true,until,oldest:points[0].t/1000,bucketSeconds:900,
          durablePoints:4400,pendingPoints:0,capacityPoints:23000,
          nextAfter:selected.length===128&&selected.at(-1).t/1000<until?selected.at(-1).t/1000:0,points:selected};
      }else body={nodes:[],sessions:[],success:true};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)}).catch(()=>{});
    });
    await page.goto('http://station.test/dashboard');
    await page.locator('.sensorSummary').first().waitFor();
    assert.equal(await page.locator('.sensorSummary').count(),2);
    assert.equal(await page.locator('.historySvg,.recordingPanel').count(),0);
    await page.waitForTimeout(1100);
    assert.equal(historyCalls.length,0,'Overview loads no history for visible or hidden sensors');
    assert.equal(await page.locator('#securityBanner').isVisible(),false);
    assert.equal(await page.locator('#cloudMessage').isVisible(),false);
    assert.equal(await page.locator('.sensorSummary').first().locator('.value').count(),1);
    if(process.env.QA_SCREENSHOT_DIR){fs.mkdirSync(process.env.QA_SCREENSHOT_DIR,{recursive:true});await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'dashboard-overview-desktop.png'),fullPage:true});}
    // Use the card background, outside the name, tile and arrow buttons.
    await page.locator('.sensorSummary').first().locator('.meta').click();
    await page.locator('.historySvg').waitFor();
    await checkBackSpacing();
    assert.equal(await page.locator('.sensorTop .sensorIdentity').count(),1);
    assert.equal(await page.locator('.sensorTop .sensorActions a').filter({hasText:'Firmware'}).count(),1);
    assert.equal(await page.locator('.sensorTop .sensorActions button').filter({hasText:'Edit'}).count(),1);
    assert.equal(await page.locator('.deviceInformation .sensorIdentity,.deviceInformation .sensorActions').count(),0,
      'Device identity and actions appear only in the detail header');
    assert(await page.locator('.sensorTop').evaluate(top=>{
      const name=top.querySelector('h3').getBoundingClientRect(),identity=top.querySelector('.sensorIdentity').getBoundingClientRect(),actions=top.querySelector('.sensorActions').getBoundingClientRect();
      return identity.left>name.right&&Math.abs(identity.top-name.top)<20&&Math.abs(actions.right-top.getBoundingClientRect().right)<2;
    }),'Identity follows the name and header actions align with the right edge');
    assert.equal(await page.getByLabel('History metric').inputValue(),'battery');
    assert(historyCalls.length>1&&historyCalls.every(m=>m===mac),'Only the opened sensor is paged');
    await page.getByLabel('Chart time window').selectOption('all');
    const svg=page.locator('.historySvg');
    assert.equal((await svg.getAttribute('data-samples')),'[]');
    assert((await svg.getAttribute('data-cache-key')),'Long data stays out of repeated DOM attributes');
    assert.equal(await svg.evaluate(el=>chartData(el,'samples').length),4400);
    assert.equal(await svg.evaluate(el=>Math.max(...chartData(el,'samples').map(p=>p.v))),4.123);
    const pathSize=await svg.locator('.seriesTrace').first().getAttribute('d');
    assert((pathSize.match(/[ML]/g)||[]).length<=900,'Visual paths have bounded point counts');
    assert(Number(await svg.getAttribute('data-window'))>30*86400000);
    await svg.tap();
    await svg.dispatchEvent('pointerleave',{pointerType:'touch'});
    assert((await page.locator('.chartReadout').textContent()).includes('Pinned'));
    await page.getByRole('button',{name:'All sensors',exact:false}).click();
    assert.equal(await page.locator('.historySvg').count(),0);
    // Both parts of the brand and keyboard activation return to the overview.
    for(const target of ['#homeLogo .mark','#homeLogo b']){
      await page.locator('.sensorSummary').first().locator('.meta').click();
      await page.locator('.historySvg').waitFor();
      await page.getByRole('button',{name:'Settings',exact:true}).click();
      await page.locator(target).click();
      assert.deepEqual(errors,[]);
      assert.equal(await page.locator('#overview.on .sensorSummary').count(),2);
      assert.equal(await page.locator('.historySvg').count(),0);
      assert.equal(await page.evaluate(()=>location.hash),'');
    }
    for(const key of ['Enter','Space']){
      await page.getByRole('button',{name:'Arrange',exact:true}).click();
      await page.getByRole('button',{name:'Settings',exact:true}).click();
      await page.locator('#homeLogo').focus();await page.keyboard.press(key);
      assert.equal(await page.locator('#overview.on .sensorSummary').count(),2);
      assert.equal(await page.locator('#dashboardLayout').isVisible(),false);
    }
    await page.locator('.sensorSummary').first().locator('.meta').click();
    await page.getByRole('button',{name:'Overview',exact:true}).click();
    assert.equal(await page.locator('#overview.on .sensorSummary').count(),2);
    // The entire voltage tile must navigate, including its bottom edge.
    const tile=page.locator('.sensorSummary').first().locator('.value');
    const box=await tile.boundingBox();await page.touchscreen.tap(box.x+box.width/2,box.y+box.height-8);
    await page.locator('.historySvg').waitFor();
    assert.equal(await page.getByLabel('History metric').inputValue(),'battery');
    await page.getByRole('button',{name:'All sensors',exact:false}).click();
    await page.getByRole('button',{name:'Arrange',exact:true}).click();
    await page.getByRole('button',{name:'Move Climate node up',exact:true}).click();
    await page.getByLabel('Show Battery node',{exact:true}).uncheck();
    await page.locator('#layoutColumns').selectOption('3');
    assert(await page.locator('#dashboardRefresh').isVisible(),'Refresh setting remains reachable');
    await page.getByRole('button',{name:'Save layout',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('dashboardLayout').hidden);
    assert.deepEqual(savedLayout.order,[other,mac]);assert.deepEqual(savedLayout.hidden,[mac]);
    assert.equal(await page.locator('.sensorSummary').count(),1);
    await page.reload();await page.locator('.sensorSummary').waitFor();
    assert.equal(await page.locator('.sensorSummary').count(),1,'Station layout survives a fresh browser page');
    assert.equal(await page.locator('#sensorGrid').getAttribute('data-columns'),'3');
    await page.getByRole('button',{name:'Show hidden',exact:true}).click();
    assert.deepEqual(await page.locator('.sensorName h3').allTextContents(),['Climate node','Battery node']);
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    if(process.env.QA_SCREENSHOT_DIR)await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'dashboard-overview-mobile.png'),fullPage:true});
    await page.locator('.sensorSummary').last().locator('.value').tap();
    await page.locator('.historySvg').waitFor();
    await checkBackSpacing();
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    const font=await page.locator('.historySvg text').first().evaluate(t=>parseFloat(getComputedStyle(t).fontSize)*t.ownerSVGElement.getBoundingClientRect().width/600);
    assert(font>=11.9);
    await page.locator('.deviceInformation > summary').tap();
    assert(await page.getByRole('button',{name:'Edit',exact:true}).isVisible());
    assert(await page.locator('.sensorActions a[href^="/updates"]').isVisible());
    if(process.env.QA_SCREENSHOT_DIR)await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'dashboard-detail-mobile.png'),fullPage:true});
    await page.setViewportSize({width:320,height:760});
    await page.emulateMedia({colorScheme:'dark'});
    await checkBackSpacing();
    await page.setViewportSize({width:390,height:844});
    await checkBackSpacing();
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.evaluate(()=>window.scrollTo(0,0));
    if(process.env.QA_SCREENSHOT_DIR)await page.screenshot({path:path.join(process.env.QA_SCREENSHOT_DIR,'dashboard-back-spacing-mobile-dark.png')});
    assert.deepEqual(errors,[]);
    const referenceCheck=await page.evaluate(()=>{
      const n={mac:'normal-motion-reference',sensorType:3,capabilities:144,sleepSeconds:1,
        motionReference:{enabled:true,ax:.125}},history=[{t:Date.now()-1000,ax:.125}];
      chartMetrics[n.mac]='ax';const holder=document.createElement('div');
      holder.innerHTML=environmentChartPanel(n,history);
      return {value:chartData(holder.querySelector('svg'),'samples')[0].v,raw:history[0].ax};
    });
    assert.equal(referenceCheck.value,0);assert.equal(referenceCheck.raw,.125);
    console.log('Compact dashboard: whole card/tile touch, lazy paged history, long periods, bounded rendering, persistent layout, mobile controls and no startup notices passed');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1});
