// Exercise setup drafts and invalid/stale values in the actual embedded pages.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');

(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../station/src/web_pages.cpp'),'utf8');
  const extract=name=>source.match(new RegExp('const char '+name+'\\[\\].*?R"WCH\\((.*?)\\)WCH";','s'))[1];
  const browser=await chromium.launch({headless:true,
    ...(process.env.PLAYWRIGHT_CHANNEL?{channel:process.env.PLAYWRIGHT_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:390,height:844}}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    const mac='02:03:04:05:06:07';
    let setupNodes=[{mac,name:'Detected sensor',sensorType:1,pcbVersion:4,operatingMode:1,provisioned:false}];
    let statusFails=false,statusReads=0,maxStatusReads=0,configFails=false,sensorWrites=0,savedName='';
    const cloud={},stationConfig={csrfToken:'test',defaultSleepSeconds:600,thingSpeakChannels:[],wifiSsid:'test'};
    const node={mac,name:'Climate',sensorType:2,configuredSensorType:2,provisioned:true,
      seen:true,online:true,capabilities:95,temperature:null,humidity:null,pressure:null,
      iaq:42,iaqAccuracy:3,batteryMv:3900,gasResistanceOhms:50000,telemetryFlags:0,
      revision:1,appliedRevision:1,sleepSeconds:600,profileSlot:255,historyRevision:1};
    await page.route('http://station.test/**',async route=>{
      const url=new URL(route.request().url());let body;
      if(url.pathname==='/setup')return route.fulfill({contentType:'text/html',body:extract('kSetupWizardPage')});
      if(url.pathname==='/dashboard')return route.fulfill({contentType:'text/html',body:extract('kDashboardPage')});
      if(url.pathname==='/api/config'){
        if(configFails)return route.abort('failed');
        body=stationConfig;
      }else if(url.pathname==='/api/status'){
        if(url.searchParams.has('setupSensors')){
          statusReads++;maxStatusReads=Math.max(maxStatusReads,statusReads);
          await new Promise(resolve=>setTimeout(resolve,100));statusReads--;
          if(statusFails)return route.abort('failed');
          body={sensors:setupNodes};
        }else body={storageAvailable:false,sensors:[node],wifi:{connected:true,ssid:'test',ip:'192.0.2.1'},cloud};
      }else if(url.pathname==='/api/sensor'){
        sensorWrites++;savedName=new URLSearchParams(route.request().postData()).get('name');
        await new Promise(resolve=>setTimeout(resolve,1000));
        body={success:true,message:'Settings saved'};
      }else if(url.pathname==='/api/history')body={points:[],bucketSeconds:600};
      else body={nodes:[],success:true};
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)});
    });
    await page.goto('http://station.test/setup');
    await page.evaluate(()=>{step=3;draw()});
    const name=page.locator('.setupSensorName').first();await name.waitFor();
    await name.fill('Kitchen draft');await name.focus();
    await page.locator('.setupSensorType').first().selectOption('4');
    await page.locator('.setupSensorInterval').first().selectOption('10');
    await name.focus();
    await page.evaluate(()=>window.savedName=document.querySelector('.setupSensorName'));
    setupNodes.push({...setupNodes[0],mac:'02:03:04:05:06:08',name:'Second sensor'});
    await page.evaluate(()=>Promise.all([loadSetupSensors(true),loadSetupSensors(true),loadSetupSensors(true)]));
    assert.equal(await page.locator('.setupSensor').count(),2);
    assert.equal(await name.inputValue(),'Kitchen draft');
    assert.equal(await page.locator('.setupSensorType').first().inputValue(),'4');
    assert.equal(await page.locator('.setupSensorInterval').first().inputValue(),'10');
    assert(await page.evaluate(()=>savedName===document.querySelector('.setupSensorName')&&savedName===document.activeElement),
      'Incoming discovery retains the actual input and its keyboard focus');
    assert.equal(maxStatusReads,1,'Manual and scheduled discovery requests cannot overlap');
    statusFails=true;await page.evaluate(()=>loadSetupSensors(true));
    assert.equal(await name.inputValue(),'Kitchen draft','A failed refresh retains pending setup drafts');
    await page.getByText('Station status is temporarily unavailable.',{exact:false}).waitFor();
    statusFails=false;await page.evaluate(()=>loadSetupSensors(true));
    assert.equal(await page.locator('.discoveryError').count(),0);
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));

    configFails=true;await page.goto('http://station.test/dashboard');
    await page.getByText('Station configuration unavailable · retrying',{exact:true}).waitFor();
    configFails=false;await page.locator('.sensor h3').waitFor({timeout:7000});
    assert(await page.locator('#securityBanner').isVisible());
    assert(await page.locator('#cloudMessage').isVisible());
    await page.waitForTimeout(5200);
    assert.equal(await page.locator('#securityBanner').isVisible(),false);
    assert.equal(await page.locator('#cloudMessage').isVisible(),false);
    await page.evaluate(async()=>{await loadConfig();await loadStatus()});
    assert.equal(await page.locator('#securityBanner').isVisible(),false,'Config reload cannot resurrect a dismissed warning');
    assert.equal(await page.locator('#cloudMessage').isVisible(),false,'Status polling cannot resurrect a dismissed warning');
    assert.equal(await page.locator('#passwordState').textContent(),'No website password is set');
    assert.equal(await page.locator('#cloudState').textContent(),'Not configured');
    stationConfig.adminPasswordSet=true;
    await page.evaluate(()=>loadConfig());
    assert(await page.locator('#securityBanner').isVisible(),'Changed password protection displays the new notice');
    Object.assign(cloud,{uploadEnabledSensors:1,failedUploads:0});
    await page.evaluate(()=>loadStatus());
    assert(await page.locator('#cloudMessage').isVisible(),'A changed cloud condition displays a new notice');
    Object.assign(cloud,{failedUploads:1,lastHttpStatus:401});
    await page.evaluate(()=>loadStatus());
    await page.waitForTimeout(5200);
    assert(await page.locator('#cloudMessage').isVisible(),'An earlier yellow notice timer cannot hide a current upload error');
    assert((await page.locator('#cloudMessage').textContent()).includes('Last upload failed'));
    assert.equal(await page.locator('.sensor .iaqTile').count(),1,'BME680 retains its IAQ tile');
    assert.equal(await page.locator('.sensor .values').getByText('Gas resistance',{exact:true}).count(),1);
    Object.assign(node,{sensorType:1,configuredSensorType:1,capabilities:23,iaq:0,gasResistanceOhms:null});
    await page.evaluate(()=>loadStatus());
    assert.equal(await page.locator('.sensor .values .value').count(),4,'BME280 has temperature, humidity, pressure and battery only');
    assert.equal(await page.locator('.sensor .iaqTile').count(),0);
    assert.equal(await page.locator('.sensor .values').getByText('Gas resistance',{exact:true}).count(),0);
    assert(!(await page.locator('.sensor .values').textContent()).includes('IAQ'));
    Object.assign(node,{sensorType:2,configuredSensorType:2,capabilities:95,iaq:42,gasResistanceOhms:50000});
    await page.evaluate(()=>loadStatus());
    assert((await page.locator('.sensor .values').textContent()).includes('– °C'),
      'JSON null measurements must not appear as zero');
    await page.locator('#storageMessage').getByText('Local storage unavailable',{exact:true}).waitFor();
    Object.assign(node,{telemetryFlags:1,temperature:22,humidity:50,pressure:1010});
    await page.getByText('No valid environmental measurement available.',{exact:true}).waitFor();
    assert(!(await page.locator('.sensor .values').textContent()).includes('Excellent'),
      'A sensor-read failure must not present stale IAQ health advice');
    Object.assign(node,{telemetryFlags:256,operatingMode:6,batteryMv:2790});
    await page.getByText('Battery protection active',{exact:true}).waitFor();
    assert.equal(await page.locator('.sensor .values .value').count(),1,
      'Battery-only BME680 protection packets cannot retain an IAQ tile');

    await page.locator('.sensorActions button').click();
    await page.locator('#wizardName').fill('First submitted name');
    await page.evaluate(()=>{window.pendingSave=saveSensorWizard();saveSensorWizard()});
    assert.equal(await page.locator('#wizardNext').isEnabled(),false);
    await page.evaluate(()=>{closeWizard();openSensorWizard(statusData.sensors[0].mac)});
    await page.locator('#wizardName').fill('New unsaved draft');
    await page.evaluate(()=>pendingSave);
    assert.equal(sensorWrites,1,'Double-clicking Save must not apply calibration twice');
    assert.equal(savedName,'First submitted name','The submitted request uses the original form snapshot');
    assert.equal(await page.locator('#sensorModal').isVisible(),true,
      'A completed older save cannot close a newly opened wizard');
    assert.equal(await page.locator('#wizardName').inputValue(),'New unsaved draft');
    await page.evaluate(()=>closeWizard());
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    assert.deepEqual(errors,[]);
    console.log('UI reliability: setup drafts/focus, serialized saves, outage recovery, timed non-repeating notices, persistent errors, null values, battery-only state and storage diagnostic passed');
  }finally{await browser.close()}
})().catch(error=>{console.error(error);process.exitCode=1});
