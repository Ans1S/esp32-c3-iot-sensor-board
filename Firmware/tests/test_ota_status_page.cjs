const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
(async () => {
  const source = fs.readFileSync(path.join(__dirname, '../station/include/ota_page.h'), 'utf8');
  const html = source.match(/R"HTML\((.*?)\)HTML"/s)[1];
  const browser = await chromium.launch({headless:true});
  try {
    const page = await browser.newPage();
    await page.addInitScript(()=>{window.setInterval=()=>0});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    let fail=false, otaReads=0, sensorReads=0;
    const mac='02:03:04:05:06:07';
    await page.route('http://station.test/**',route=>{
      const url=new URL(route.request().url());let body={};
      if(url.pathname==='/updates')return route.fulfill({contentType:'text/html',body:html});
      if(url.pathname==='/api/config')body={csrfToken:'test'};
      if(url.pathname==='/api/ota') {
        ++otaReads;
        if(fail)return route.abort('failed');
        body={available:true,mac,targetVersion:'4.1.1',state:'Waiting for next contact',bytes:0,total:1000000,nodes:[{mac,pcb:4,version:'4.1.0',release:40100}]};
      }
      if(url.pathname==='/api/status') {
        ++sensorReads;
        body={sensors:[{mac,name:'Test',provisioned:true,pcbVersion:4,sleepSeconds:300}]};
      }
      return route.fulfill({contentType:'application/json',body:JSON.stringify(body)});
    });
    await page.goto('http://station.test/updates');
    await page.waitForFunction(()=>document.getElementById('jobReason').textContent.includes('300 seconds'));
    await page.evaluate(()=>message('uploadMessage','info','Update scheduled','Saved successfully'));
    const before=otaReads;
    await page.evaluate(()=>Promise.all([refresh(),refresh(),refresh()]));
    assert.equal(otaReads,before+1);assert.equal(sensorReads,1);
    fail=true;await page.evaluate(()=>refresh());
    assert((await page.locator('#statusMessage').textContent()).includes('does not mean the sensor update failed'));
    assert((await page.locator('#uploadMessage').textContent()).includes('Update scheduled'));
    assert.equal(await page.locator('#stateBadge').textContent(),'Waiting for next contact');
    fail=false;await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#statusMessage').textContent(),'');
    assert.deepEqual(errors,[]);
    console.log('OTA page: non-overlapping polling, cached sensor status, network error recovery and sleep explanation passed');
  } finally {await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
