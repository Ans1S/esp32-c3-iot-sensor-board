#include "web_pages.h"

namespace station {

const char kLoginPage[] PROGMEM = R"WCH(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#edf6f0"><title>Sign in · W-Charger</title>
<script>(()=>{let m='system';try{m=localStorage.getItem('wcharger-theme')||'system'}catch(e){}const q=matchMedia('(prefers-color-scheme: dark)'),a=()=>{const t=m==='system'?(q.matches?'dark':'light'):m;document.documentElement.dataset.theme=t;document.documentElement.dataset.themeMode=m;document.querySelector('meta[name="theme-color"]').content=t==='dark'?'#101813':'#edf6f0';document.querySelectorAll('.themeMode').forEach(s=>s.value=m)};window.wchargerTheme={bind(){document.querySelectorAll('.themeMode').forEach(s=>{s.value=m;s.onchange=()=>{m=s.value;try{m==='system'?localStorage.removeItem('wcharger-theme'):localStorage.setItem('wcharger-theme',m)}catch(e){}a()}});q.addEventListener?.('change',()=>{if(m==='system')a()});a()}};a()})();</script>
<style>
:root{--ink:#14231a;--muted:#657269;--line:#d9e4dc;--brand:#126b42;--soft:#e3f4e9;--danger:#b42318;--shadow:0 28px 80px #143a251f;color-scheme:light}*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;padding:20px;background:radial-gradient(circle at 15% 10%,#dff3e7 0,transparent 36%),radial-gradient(circle at 90% 90%,#e8edf8 0,transparent 35%),#f7f8f5;color:var(--ink);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}.shell{width:min(430px,100%)}.shellTop{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px}.brand{display:flex;align-items:center;gap:11px}.mark{display:grid;place-items:center;width:44px;height:44px;border-radius:15px;background:linear-gradient(145deg,#168453,var(--brand));color:#fff;font-size:22px;font-weight:900;box-shadow:0 10px 24px #126b4230}.brand b{font-size:19px}.brand small{display:block;color:var(--muted)}.themeControl{display:inline-flex;align-items:center;min-height:42px;border-radius:12px;color:var(--muted)}.themeControl:hover{background:#ffffffb8;color:var(--ink)}.themeControl select{width:auto;min-width:112px;padding:9px 28px 9px 10px;border:1px solid var(--line);border-radius:12px;background:#ffffffd9;color:inherit;font:inherit;cursor:pointer}.card{padding:clamp(25px,7vw,38px);border:1px solid #fff;border-radius:26px;background:#ffffffed;box-shadow:var(--shadow);backdrop-filter:blur(16px)}.lock{display:grid;place-items:center;width:54px;height:54px;margin-bottom:20px;border-radius:17px;background:var(--soft);font-size:24px}h1{margin:0 0 8px;font-size:30px;line-height:1.1}.lead{margin:0 0 24px;color:var(--muted)}label{display:block;margin:0 0 7px;font-weight:700}input{width:100%;padding:14px 15px;border:1px solid var(--line);border-radius:13px;background:#fbfcfa;color:var(--ink);font:inherit;outline:0}input:focus{border-color:var(--brand);box-shadow:0 0 0 4px #126b4216}.btn{width:100%;margin-top:13px;padding:13px 18px;border:0;border-radius:13px;background:var(--brand);color:#fff;font:inherit;font-weight:800;cursor:pointer}.btn:disabled{opacity:.55;cursor:wait}.error{display:none;margin-top:13px;padding:11px 12px;border-radius:11px;background:#fff0ee;color:var(--danger)}.note{margin:18px 0 0;color:var(--muted);font-size:12px;text-align:center}html[data-theme=dark]{--ink:#edf6ef;--muted:#a8b8ad;--line:#34473a;--brand:#48c986;--soft:#193b29;--danger:#ff9b92;--shadow:0 28px 80px #0007;color-scheme:dark}html[data-theme=dark] body{background:radial-gradient(circle at 15% 10%,#173525 0,transparent 38%),radial-gradient(circle at 90% 90%,#1c2940 0,transparent 36%),#0d1510}html[data-theme=dark] .card{border-color:#2b3d31;background:#152019ed}html[data-theme=dark] input,html[data-theme=dark] .themeControl select{background:#0f1812}html[data-theme=dark] .themeControl:hover{background:#223128}html[data-theme=dark] .error{background:#451d1b}@media(max-width:390px){.shellTop{align-items:flex-start;flex-direction:column}.themeControl,.themeControl select{width:100%}}
.liveSvg text{font-size:11px}@media(max-width:600px){.liveSvg text{font-size:17px}}
</style></head><body><main class="shell"><div class="shellTop"><div class="brand"><div class="mark">⚡</div><div><b>W-Charger</b><small>Station</small></div></div><label class="themeControl" for="loginTheme"><select id="loginTheme" class="themeMode" aria-label="Color theme"><option value="system">🖥️ System</option><option value="light">☀️ Light</option><option value="dark">🌙 Dark</option></select></label></div><section class="card"><div class="lock">🔒</div><h1>Welcome back</h1><p class="lead">Enter the website password to view sensors, settings and API keys.</p><form id="loginForm"><label for="password">Website password</label><input id="password" name="password" type="password" maxlength="64" autocomplete="current-password" autofocus required><button id="submit" class="btn" type="submit">Sign in</button><div id="error" class="error" role="alert"></div></form><p class="note">Access is protected locally on this W-Charger station. Sessions expire after 30 minutes.</p></section></main><script>
wchargerTheme.bind();const form=document.getElementById('loginForm'),button=document.getElementById('submit'),error=document.getElementById('error');form.onsubmit=async e=>{e.preventDefault();button.disabled=true;error.style.display='none';try{const response=await fetch('/login',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(new FormData(form))}),data=await response.json();if(!response.ok||!data.success)throw new Error(data.message||'Sign-in failed');location.replace('/dashboard')}catch(exception){error.textContent=exception.message;error.style.display='block';button.disabled=false;document.getElementById('password').select()}};
</script></body></html>
)WCH";

const char kSetupWizardPage[] PROGMEM = R"WCH(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#f7f8f5"><title>Set up W-Charger</title>
<script>(()=>{let m='system';try{m=localStorage.getItem('wcharger-theme')||'system'}catch(e){}const q=matchMedia('(prefers-color-scheme: dark)'),a=()=>{const t=m==='system'?(q.matches?'dark':'light'):m;document.documentElement.dataset.theme=t;document.documentElement.dataset.themeMode=m;document.querySelector('meta[name="theme-color"]').content=t==='dark'?'#101813':'#f7f8f5';document.querySelectorAll('.themeMode').forEach(s=>s.value=m)};window.wchargerTheme={bind(){document.querySelectorAll('.themeMode').forEach(s=>{s.value=m;s.onchange=()=>{m=s.value;try{m==='system'?localStorage.removeItem('wcharger-theme'):localStorage.setItem('wcharger-theme',m)}catch(e){}a()}});q.addEventListener?.('change',()=>{if(m==='system')a()});a()}};a()})();</script>
<style>
:root{--card:#fff;--ink:#172019;--muted:#667168;--line:#dce3dc;--brand:#146c43;--brand2:#e4f4ea;--danger:#b42318;--shadow:0 18px 55px #18352218}*{box-sizing:border-box}body{margin:0;background:linear-gradient(145deg,#eef7f1,#f8f7f1 55%,#edf1f8);color:var(--ink);font:16px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;min-height:100vh}.shell{max-width:760px;margin:auto;padding:clamp(18px,4vw,44px)}.brand{display:flex;align-items:center;gap:12px;margin-bottom:24px}.mark{display:grid;place-items:center;width:44px;height:44px;border-radius:14px;background:var(--brand);color:#fff;font-weight:800}.brand b{font-size:20px}.brand small{display:block;color:var(--muted)}.card{background:var(--card);border:1px solid #fff;border-radius:24px;padding:clamp(22px,5vw,42px);box-shadow:var(--shadow)}.progress{display:flex;gap:7px;margin-bottom:28px}.dot{height:7px;flex:1;border-radius:9px;background:#e5e9e5}.dot.on{background:var(--brand)}.step{display:none;animation:in .25s ease}.step.on{display:block}@keyframes in{from{opacity:0;transform:translateY(5px)}}h1{font-size:clamp(28px,7vw,43px);line-height:1.05;margin:0 0 12px}h2{font-size:27px;margin:0 0 9px}.lead{color:var(--muted);font-size:17px;margin:0 0 26px}.feature{display:flex;gap:13px;padding:13px 0}.icon{display:grid;place-items:center;flex:0 0 38px;height:38px;border-radius:12px;background:var(--brand2);color:var(--brand)}.feature b{display:block}.feature span,.hint{color:var(--muted);font-size:14px}label{display:block;font-weight:650;margin:18px 0 7px}input,select{width:100%;padding:14px 15px;border:1px solid var(--line);border-radius:13px;background:#fbfcfa;color:var(--ink);font:inherit;outline:0}input:focus,select:focus{border-color:var(--brand);box-shadow:0 0 0 3px #1a7b4c1a}.choices{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}.choice{padding:15px;border:1px solid var(--line);border-radius:14px;background:#fff;text-align:left;color:var(--ink);cursor:pointer}.choice.on{border-color:var(--brand);background:var(--brand2)}.choice b,.choice span{display:block}.choice span{color:var(--muted);font-size:13px}.customTime{display:none;margin-top:14px;padding:15px;border-radius:14px;background:var(--brand2)}.customTime.on{display:grid;grid-template-columns:1fr auto;align-items:end;gap:10px}.customTime label{grid-column:1/-1;margin:0}.customTime span{padding:14px 0;color:var(--muted)}.keyGrid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.wide{grid-column:1/-1}.networks{display:grid;gap:9px;max-height:260px;overflow:auto}.network{display:flex;align-items:center;justify-content:space-between;width:100%;padding:13px;border:1px solid var(--line);border-radius:13px;background:#fff;color:var(--ink);text-align:left;cursor:pointer}.network.on{border-color:var(--brand);background:var(--brand2)}.network small{display:block;color:var(--muted)}.sensorScan{display:grid;gap:10px}.setupSensor{padding:15px;border:1px solid var(--line);border-radius:15px;background:#fbfcfa}.setupSensorTop{display:flex;align-items:center;justify-content:space-between;gap:10px}.setupSensorTop code{display:block;color:var(--muted);font-size:11px}.setupSensorGrid{display:grid;grid-template-columns:1.2fr 1fr 1fr;gap:8px}.setupSensorGrid label{margin-top:11px}.sensorReady{padding:14px;border-radius:14px;background:var(--brand2);color:var(--brand)}.securityChoice{padding:18px;border:1px solid var(--line);border-radius:17px;background:linear-gradient(145deg,#fbfdfb,#f1f8f3)}.securityChoice .icon{font-size:20px;margin-bottom:12px}.passwordGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.summary{border:1px solid var(--line);border-radius:15px;overflow:hidden}.summary div{display:flex;justify-content:space-between;gap:20px;padding:13px 15px;border-bottom:1px solid var(--line)}.summary div:last-child{border:0}.summary span{color:var(--muted)}.actions{display:flex;justify-content:space-between;gap:12px;margin-top:28px}.btn{border:0;border-radius:13px;padding:13px 18px;font:inherit;font-weight:750;cursor:pointer}.primary{background:var(--brand);color:#fff}.ghost{background:#eef1ee;color:var(--ink)}.btn:disabled{opacity:.5;cursor:wait}.error{display:none;margin-top:15px;padding:12px;border-radius:12px;background:#fff0ee;color:var(--danger)}.done{text-align:center;padding:30px 0}.spinner{width:48px;height:48px;border:5px solid var(--brand2);border-top-color:var(--brand);border-radius:50%;margin:0 auto 20px;animation:spin 1s linear infinite}@keyframes spin{to{transform:rotate(360deg)}}@media(max-width:540px){.shell{padding:14px}.card{border-radius:20px;padding:22px}.choices,.keyGrid,.setupSensorGrid,.passwordGrid{grid-template-columns:1fr}.wide{grid-column:auto}.actions .btn{flex:1}}
.modeBadge{display:inline-flex;align-items:center;padding:4px 8px;border-radius:999px;font-size:11px;font-weight:750;background:#eef1ee;color:var(--muted)}.modeBadge.setup{background:#fff5d8;color:#946200}.modeBadge.energy{background:var(--brand2);color:var(--brand)}.modeBadge.recovery{background:#e8efff;color:#315fa8}
.liveSvg text{font-size:11px}@media(max-width:600px){.liveSvg text{font-size:17px}}
</style>
<style>
html{color-scheme:light}.shellTop{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:24px}.shellTop .brand{margin-bottom:0}.shellTop .mark{font-size:22px}.themeControl{display:inline-flex;align-items:center;min-height:42px;border-radius:12px;color:var(--muted)}.themeControl:hover{background:#ffffffb8;color:var(--ink)}.themeControl select{width:auto;min-width:112px;padding:9px 28px 9px 10px;border-radius:12px;cursor:pointer}html[data-theme=dark]{--card:#17231b;--ink:#edf6ef;--muted:#a8b8ad;--line:#34473a;--brand:#48c986;--brand2:#193b29;--danger:#ff9b92;--shadow:0 18px 55px #0007;color-scheme:dark}html[data-theme=dark] body{background:linear-gradient(145deg,#101b14,#0d1510 55%,#121a28)}html[data-theme=dark] .card{border-color:#2b3d31}html[data-theme=dark] input,html[data-theme=dark] select,html[data-theme=dark] .choice,html[data-theme=dark] .network,html[data-theme=dark] .setupSensor{background:#101913}html[data-theme=dark] .themeControl:hover{background:#223128}html[data-theme=dark] .securityChoice{background:linear-gradient(145deg,#132019,#162a1d)}html[data-theme=dark] .ghost{background:#26342b}html[data-theme=dark] .error{background:#451d1b}@media(max-width:420px){.shellTop{align-items:flex-start;flex-direction:column}.themeControl,.themeControl select{width:100%}}
.liveSvg text{font-size:11px}@media(max-width:600px){.liveSvg text{font-size:17px}}
</style></head><body><main class="shell"><div class="shellTop"><div class="brand"><div class="mark">⚡</div><div><b>W-Charger</b><small>Station</small></div></div><label class="themeControl" for="setupTheme"><select id="setupTheme" class="themeMode" aria-label="Color theme"><option value="system">🖥️ System</option><option value="light">☀️ Light</option><option value="dark">🌙 Dark</option></select></label></div><section class="card"><div id="progress" class="progress"></div>
<div class="step on"><h1>Welcome to W-Charger</h1><p class="lead">Set up your station in a few clear steps.</p><div class="feature"><div class="icon">1</div><div><b>Choose a measurement interval</b><span>Defines how often sensors send new data.</span></div></div><div class="feature"><div class="icon">2</div><div><b>Optionally link ThingSpeak</b><span>Only the account User API Key is needed here.</span></div></div><div class="feature"><div class="icon">3</div><div><b>Connect devices and Wi-Fi</b><span>Add nearby sensors and select your 2.4 GHz network.</span></div></div><div class="feature"><div class="icon">4</div><div><b>Protect private settings</b><span>An optional website password keeps API keys inside your household.</span></div></div></div>
<div class="step"><h2>How often should sensors measure?</h2><p class="lead">A longer interval saves energy. Each sensor can be adjusted later.</p><div class="choices" id="intervals"><button class="choice" data-value="60"><b>1 minute</b><span>Direct functional testing</span></button><button class="choice" data-value="300"><b>5 minutes</b><span>Fast testing</span></button><button class="choice on" data-value="600"><b>10 minutes</b><span>Recommended starting point</span></button><button class="choice" data-value="900"><b>15 minutes</b><span>Balanced</span></button><button class="choice" data-value="1800"><b>30 minutes</b><span>Longer battery life</span></button><button class="choice" data-value="3600"><b>60 minutes</b><span>Maximum runtime</span></button><button class="choice" data-value="custom"><b>Custom</b><span>Set your own time</span></button></div><div id="customSetupTime" class="customTime"><label for="customSetupMinutes">Custom interval</label><input id="customSetupMinutes" type="number" min="1" max="1440" step="1" value="10"><span>minutes</span></div></div>
<div class="step"><h2>Link ThingSpeak</h2><p class="lead">The optional User API Key is enough to load, create and manage your account channels after setup.</p><div class="keyGrid"><div class="wide"><label for="apiKey">ThingSpeak User API Key</label><input id="apiKey" type="text" maxlength="40" autocomplete="off" placeholder="Optional · can also be added later"></div></div><p class="hint">Channel selection, Read and Write API Keys and field mapping remain available under Settings and when editing a sensor.</p></div>
<div class="step"><h2>Nearby sensors</h2><p class="lead">Turn on your sensors. New devices check in every ten seconds for their first ten minutes and remain in this list after first detection.</p><button id="scanSensorsBtn" class="btn ghost" type="button">Search for sensors now</button><div id="setupSensors" class="sensorScan" style="margin-top:14px"><p class="hint">Automatic discovery starts when you open this step.</p></div><p class="hint">Adding is optional. Name, sensor board, cloud fields and calibration can be changed later in the dashboard.</p></div>
<div class="step"><h2>Connect to your Wi-Fi</h2><p class="lead">Select your 2.4 GHz network and enter its password.</p><button id="scanBtn" class="btn ghost" type="button">Search Wi-Fi networks</button><div id="networks" class="networks" style="margin-top:12px"></div><label for="ssid">Wi-Fi name</label><input id="ssid" maxlength="32" placeholder="Select a network or enter its name"><label for="wifiPassword">Wi-Fi password</label><input id="wifiPassword" type="password" maxlength="64" autocomplete="new-password" placeholder="Leave blank to keep the saved password"><p class="hint">W-Charger supports 2.4 GHz Wi-Fi only.</p></div>
<div class="step"><h2>Protect the website</h2><p class="lead">Optional, but recommended when other people or devices share your home network.</p><div class="securityChoice"><div class="icon">🔒</div><b>Website password</b><p class="hint">This protects the dashboard, settings, local APIs and visible ThingSpeak keys. The password is stored as a device-specific hash.</p><div class="passwordGrid"><div><label for="adminPassword">Password</label><input id="adminPassword" type="password" minlength="8" maxlength="64" autocomplete="new-password" placeholder="Leave blank for no login"></div><div><label for="confirmAdminPassword">Confirm password</label><input id="confirmAdminPassword" type="password" minlength="8" maxlength="64" autocomplete="new-password" placeholder="Repeat password"></div></div></div><p class="hint">You can enable, change or remove protection later in Settings.</p></div>
<div class="step"><h2>Everything is ready</h2><p class="lead">Review your choices. The station will save them and restart.</p><div class="summary"><div><span>Station name</span><b>W-Charger</b></div><div><span>Measurement interval</span><b id="reviewInterval"></b></div><div><span>ThingSpeak</span><b id="reviewCloud"></b></div><div><span>Home Wi-Fi</span><b id="reviewSsid"></b></div><div><span>Website access</span><b id="reviewSecurity"></b></div></div></div>
<div id="error" class="error" role="alert"></div><div id="done" class="done" style="display:none"><div class="spinner"></div><h2>W-Charger is restarting</h2><p id="doneStatus" class="lead">The station will be found automatically. Keep this page open.</p><button id="retrySetup" class="btn ghost" style="display:none" type="button">Try again</button></div><div id="actions" class="actions"><button id="back" class="btn ghost">Back</button><button id="next" class="btn primary">Get started</button></div></section></main>
<script>
let step=0,interval=600,csrf='',sensorScanTimer=0,sensorSignature='',setupSensorsLoading=false,setupSensorRefreshRequested=false;const $=id=>document.getElementById(id),steps=[...document.querySelectorAll('.step')],progress=$('progress'),back=$('back'),next=$('next'),ssid=$('ssid'),scanBtn=$('scanBtn'),networks=$('networks'),apiKey=$('apiKey'),wifiPassword=$('wifiPassword'),adminPassword=$('adminPassword'),confirmAdminPassword=$('confirmAdminPassword'),customSetupTime=$('customSetupTime'),customSetupMinutes=$('customSetupMinutes'),error=$('error');wchargerTheme.bind();const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));async function requestJson(url,timeout=4000){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeout);try{const response=await fetch(url,{cache:'no-store',signal:controller.signal});if(response.status===401){location.replace('/login');throw new Error('Authentication required')}if(!response.ok)throw new Error(`HTTP ${response.status}`);return await response.json()}finally{clearTimeout(timer)}}
function draw(){steps.forEach((s,i)=>s.classList.toggle('on',i===step));progress.innerHTML=steps.map((_,i)=>`<i class="dot ${i<=step?'on':''}"></i>`).join('');back.style.visibility=step?'visible':'hidden';next.textContent=step===steps.length-1?'Finish setup':step?'Next':'Get started';clearTimeout(sensorScanTimer);if(step===3)loadSetupSensors(true);if(step===steps.length-1)review()}
function move(d){if(d>0&&step===4&&!ssid.value.trim()){showError('Please select your Wi-Fi network.');return}if(d>0&&step===5&&((adminPassword.value&&adminPassword.value.length<8)||adminPassword.value!==confirmAdminPassword.value)){showError(adminPassword.value!==confirmAdminPassword.value?'The website passwords do not match.':'Use at least 8 characters for the website password.');return}if(step===steps.length-1&&d>0){finish();return}error.style.display='none';step=Math.max(0,Math.min(steps.length-1,step+d));draw()}
document.querySelectorAll('#intervals .choice').forEach(b=>b.onclick=()=>{document.querySelectorAll('#intervals .choice').forEach(x=>x.classList.remove('on'));b.classList.add('on');const custom=b.dataset.value==='custom';customSetupTime.classList.toggle('on',custom);interval=custom?Math.max(1,Number(customSetupMinutes.value)||1)*60:Number(b.dataset.value)});customSetupMinutes.oninput=()=>interval=Math.max(1,Math.min(1440,Number(customSetupMinutes.value)||1))*60;back.onclick=()=>move(-1);next.onclick=()=>move(1);scanBtn.onclick=scanWifi;
async function boot(){try{const c=await(await fetch('/api/config')).json();csrf=c.csrfToken;interval=c.defaultSleepSeconds||600;const selected=document.querySelector(`#intervals [data-value="${interval}"]`);if(selected)selected.click();else{customSetupMinutes.value=Math.max(1,Math.round(interval/60));document.querySelector('#intervals [data-value="custom"]').click()}ssid.value=c.wifiSsid||'';apiKey.value=c.userApiKey||''}catch(e){}draw()}
async function scanWifi(){scanBtn.disabled=true;scanBtn.textContent='Searching …';networks.innerHTML='<p class="hint">Scanning Wi-Fi networks without blocking sensor reception.</p>';try{let data=await requestJson('/api/wifi-scan?refresh=1');for(let attempt=0;data.scanning&&attempt<20;attempt++){await new Promise(resolve=>setTimeout(resolve,400));data=await requestJson('/api/wifi-scan')}networks.innerHTML=data.networks.length?data.networks.map(n=>`<button type="button" class="network" data-ssid="${esc(n.ssid)}"><span><b>${esc(n.ssid)}</b><small>Channel ${n.channel} · ${n.secure?'secured':'open'}</small></span><span>${n.rssi>-55?'●●●':n.rssi>-70?'●●○':'●○○'}</span></button>`).join(''):'<p class="hint">No networks found. Search again or enter the name.</p>';document.querySelectorAll('.network').forEach(b=>b.onclick=()=>{document.querySelectorAll('.network').forEach(x=>x.classList.remove('on'));b.classList.add('on');ssid.value=b.dataset.ssid})}catch(e){networks.innerHTML='<p class="hint">Scan failed. Please try again.</p>'}scanBtn.disabled=false;scanBtn.textContent='Search again'}
function sensorTypeLabel(type){return({1:'BME280',2:'BME680',3:'LSM6DSOX',4:'TMP117',5:'MAX30102',6:'BMP280',7:'BME688',8:'CCS811',9:'TOF400C-VL53L1X',10:'AS7341',255:'No environmental sensor'})[Number(type)]||'Detected automatically'}
function sensorModeInfo(n){const mode=Number(n?.operatingMode),type=Number(n?.sensorType);if(mode===6)return{label:'Battery protection',css:'energy',detail:'Sensors off; deep sleep with battery report once every 24 hours'};if(mode===5&&type===8)return{label:'Continuous gas sensing',css:'recovery',detail:'Sensor rail stays powered for MOX heater warm-up and continuous eCO2/TVOC measurement'};if(mode===5)return{label:'Live precision measurement',css:'recovery',detail:type===5?'Active optical recording (25 Hz waveform / 1 s pulse); sensor and CPU stay powered':'Active temperature recording (1 s cycle); sensor and CPU stay powered'};if(mode===4)return{label:'Continuous motion',css:'recovery',detail:'Active 104 Hz FIFO recording; sensor and CPU stay powered'};if(mode===1)return{label:'Setup mode',css:'setup',detail:'Broadcast discovery on all 13 channels'};if(mode===2&&type===5)return{label:'Standby (light sleep)',css:'energy',detail:'Sensor rail off; light sleep with GPIO9 (SW2) wake and 5-second station check-in'};if(mode===2&&[3,4].includes(type))return{label:'Interval measurement',css:'energy',detail:'Sensor rail off between intervals; light sleep with GPIO9 (SW2) wake'};if(mode===2)return{label:'Energy-saving mode',css:'energy',detail:'Sensor rail off, ESP32-C3 deep sleep and unicast on the saved channel'};if(mode===3)return{label:'Channel recovery',css:'recovery',detail:'Searching for the station on another channel'};return{label:'Mode unknown',css:'unknown',detail:'Waiting for a current sensor report'}}function sensorModeBadge(n){const mode=sensorModeInfo(n);return `<span class="modeBadge ${mode.css}" title="${mode.detail}">${mode.label}</span>`}
function setupIntervalOptions(sensorType){if(Number(sensorType)===5)return '<option value="1">200 milliseconds · SW2 recording</option>';const isGas=[2,7].includes(Number(sensorType)),minimum=isGas?300:1,values=[1,2,5,10,30,60,300,600,1800,3600].filter(v=>v>=minimum),selected=isGas?Math.max(300,Math.ceil(interval/300)*300):[3,4].includes(Number(sensorType))?1:Math.max(1,interval);if(!values.includes(selected))values.push(selected);return [...new Set(values)].sort((a,b)=>a-b).map(v=>{const amount=v<60?v:v<3600?v/60:v/3600,unit=v<60?(amount===1?'second':'seconds'):v<3600?(amount===1?'minute':'minutes'):(amount===1?'hour':'hours');return `<option value="${v}" ${v===selected?'selected':''}>${amount} ${unit}</option>`}).join('')}
async function loadSetupSensors(force=false){clearTimeout(sensorScanTimer);if(step!==3)return;if(setupSensorsLoading){setupSensorRefreshRequested ||= force;return}setupSensorsLoading=true;const box=$('setupSensors');try{const data=await requestJson('/api/status?setupSensors='+Date.now());if(step!==3)return;box.querySelector('.discoveryError')?.remove();const list=data.sensors||[],signature=list.map(n=>`${n.mac}:${n.provisioned}:${n.sensorType}:${n.operatingMode}`).join('|');if(force||signature!==sensorSignature){sensorSignature=signature;const previous=new Map([...box.querySelectorAll('.setupSensor')].map(card=>[card.dataset.mac,card])),pending=list.filter(n=>!n.provisioned),ready=list.filter(n=>n.provisioned),next=document.createElement('div');next.innerHTML=(pending.length?pending.map(n=>`<div class="setupSensor" data-mac="${esc(n.mac)}" data-detected-type="${Number(n.sensorType)}"><div class="setupSensorTop"><div><b>New sensor detected</b><code>${esc(n.mac)} · PCB V${n.pcbVersion} · ${sensorTypeLabel(n.sensorType)}</code></div>${sensorModeBadge(n)}</div><div class="setupSensorGrid"><div><label>Name</label><input class="setupSensorName" maxlength="24" value="${esc(n.name)}"></div><div><label>Sensor board</label><select class="setupSensorType"><option value="${Number(n.sensorType)}">${sensorTypeLabel(n.sensorType)} · detected</option><option value="3">LSM6DSOX · acceleration and angular rate</option><option value="4">TMP117 · precision temperature</option><option value="5">MAX30102 · heart rate</option><option value="0">Auto detect</option><option value="1">BME280</option><option value="2">BME680</option><option value="6">BMP280</option><option value="7">BME688</option><option value="8">CCS811 · eCO₂ / TVOC</option><option value="9">TOF400C-VL53L1X · distance</option><option value="10">AS7341 · 10-ch spectrum</option><option value="255">No environmental sensor</option></select></div><div><label>Interval</label><select class="setupSensorInterval">${setupIntervalOptions(n.sensorType)}</select></div></div><button type="button" class="btn primary" style="margin-top:12px" onclick="quickAddSetupSensor(this)">Add sensor</button></div>`).join(''):'<p class="hint">Listening for sensors… New sensors check in every ten seconds during their first ten minutes.</p>')+ready.map(n=>`<div class="sensorReady"><b>✓ ${esc(n.name)}</b> ${sensorModeBadge(n)}<div class="hint">${esc(n.mac)} · already added · Details can be changed later in the dashboard</div></div>`).join('');const pendingMacs=new Set(pending.map(n=>n.mac));for(const [mac,card] of previous)if(!pendingMacs.has(mac))card.remove();box.querySelectorAll(':scope > :not(.setupSensor)').forEach(el=>el.remove());for(const replacement of [...next.children]){const existing=replacement.classList.contains('setupSensor')?previous.get(replacement.dataset.mac):null;if(existing){existing.dataset.detectedType=replacement.dataset.detectedType;existing.querySelector('code').textContent=replacement.querySelector('code').textContent;existing.querySelector('.modeBadge').replaceWith(replacement.querySelector('.modeBadge'))}else box.append(replacement)}box.querySelectorAll('.setupSensorType').forEach(select=>select.onchange=()=>{const card=select.closest('.setupSensor'),effective=Number(select.value)===0?card.dataset.detectedType:select.value;card.querySelector('.setupSensorInterval').innerHTML=setupIntervalOptions(effective)})}}catch(e){if(step===3){let notice=box.querySelector('.discoveryError');if(!notice){notice=document.createElement('p');notice.className='hint discoveryError';box.append(notice)}notice.textContent='Station status is temporarily unavailable. Discovery will retry automatically.'}}finally{setupSensorsLoading=false;if(step===3){const forceNext=setupSensorRefreshRequested;setupSensorRefreshRequested=false;sensorScanTimer=setTimeout(()=>loadSetupSensors(forceNext),1000)}}}
async function quickAddSetupSensor(button){const card=button.closest('.setupSensor'),name=card.querySelector('.setupSensorName').value.trim(),sensorType=card.querySelector('.setupSensorType').value,sleepSeconds=card.querySelector('.setupSensorInterval').value;if(!name){showError('Please give the sensor a name.');return}button.disabled=true;button.textContent='Adding …';try{const body=new URLSearchParams({csrf,mac:card.dataset.mac,name,sensorType,sleepSeconds}),r=await fetch('/api/setup-sensor',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}),j=await r.json();if(!r.ok||!j.success)throw new Error(j.message||'Sensor could not be added');sensorSignature='';await loadSetupSensors(true)}catch(e){showError(e.message);button.disabled=false;button.textContent='Add sensor'}}
$('scanSensorsBtn').onclick=()=>{sensorSignature='';loadSetupSensors(true)};
function review(){const minutes=interval/60;$('reviewInterval').textContent=minutes===1?'1 minute':minutes+' minutes';$('reviewCloud').textContent=apiKey.value?'User API Key saved':'set up later';$('reviewSsid').textContent=ssid.value||'not selected';$('reviewSecurity').textContent=adminPassword.value?'Password protected':'No password'}function showError(t){error.style.display='block';error.textContent=t}
const pause=ms=>new Promise(r=>setTimeout(r,ms));async function reconnectSetup(){const status=$('doneStatus'),retry=$('retrySetup');retry.style.display='none';status.textContent='W-Charger is restarting …';await pause(3500);const started=Date.now();while(Date.now()-started<60000){status.textContent=`Looking for station … ${Math.floor((Date.now()-started)/1000)+1} s`;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),4000);try{const r=await fetch('/api/status?reconnect='+Date.now(),{cache:'no-store',signal:controller.signal});if(r.status===401){status.textContent='Station is protected. Opening sign-in …';await pause(400);location.replace('/login');return}if(r.ok){status.textContent='Station is reachable again. Opening dashboard …';await pause(500);location.replace('/dashboard');return}}catch(e){}finally{clearTimeout(timer)}await pause(1200)}status.textContent='The station is not reachable yet. Check the Wi-Fi connection and try again.';retry.style.display='inline-flex'}$('retrySetup').onclick=reconnectSetup;
async function finish(){next.disabled=true;back.disabled=true;error.style.display='none';const body=new URLSearchParams({csrf,ssid:ssid.value.trim(),wifiPassword:wifiPassword.value,defaultSleep:String(interval),userApiKey:apiKey.value,adminPassword:adminPassword.value,confirmAdminPassword:confirmAdminPassword.value});try{const r=await fetch('/api/setup',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();if(!r.ok||!j.success)throw new Error(j.message||'Save failed');steps.forEach(s=>s.style.display='none');progress.style.display='none';$('actions').style.display='none';$('done').style.display='block';reconnectSetup()}catch(e){showError(e.message);next.disabled=false;back.disabled=false}}
boot();
</script></body></html>
)WCH";

const char kDashboardPage[] PROGMEM = R"WCH(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#f7f8f5"><title>W-Charger Dashboard</title><script>(()=>{let m='system';try{m=localStorage.getItem('wcharger-theme')||'system'}catch(e){}const q=matchMedia('(prefers-color-scheme: dark)'),a=()=>{const t=m==='system'?(q.matches?'dark':'light'):m;document.documentElement.dataset.theme=t;document.documentElement.dataset.themeMode=m;document.querySelector('meta[name="theme-color"]').content=t==='dark'?'#101813':'#f7f8f5';document.querySelectorAll('.themeMode').forEach(s=>s.value=m)};window.wchargerTheme={bind(){document.querySelectorAll('.themeMode').forEach(s=>{s.value=m;s.onchange=()=>{m=s.value;try{m==='system'?localStorage.removeItem('wcharger-theme'):localStorage.setItem('wcharger-theme',m)}catch(e){}a()}});q.addEventListener?.('change',()=>{if(m==='system')a()});a()}};a()})();</script>
<style>
:root{--bg:#f4f7f4;--card:#ffffff;--surface:#f7faf7;--ink:#142118;--muted:#5c6b61;--line:#d9e3dc;--brand:#146c43;--brand2:#e3f3e9;--danger:#b42318;--warn:#946200;--warnbg:#fff5d8;--chart-bg:#ffffff;--series-blue:#186ac2;--series-orange:#c25229;--series-purple:#7256bb;--shadow:0 10px 30px #15352010;color-scheme:light}
html{scrollbar-gutter:stable}
html[data-theme=dark]{--bg:#0d1510;--card:#152019;--surface:#101913;--ink:#edf6ef;--muted:#9fb1a5;--line:#2b3d31;--brand:#48c986;--brand2:#193b29;--danger:#ff9b92;--warn:#ffd479;--warnbg:#3b3119;--chart-bg:#152019;--series-blue:#80b7ff;--series-orange:#ffb087;--series-purple:#c4acff;--shadow:0 14px 36px #0007;color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;background:linear-gradient(155deg,#edf6f0,#f7f8f5 52%,#eef2f8);color:var(--ink);font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}
html[data-theme=dark] body{background:linear-gradient(155deg,#101b14,#0d1510 55%,#121a28)}
button,input,select{font:inherit}
h1{font-size:clamp(26px,4.5vw,38px);line-height:1.1;letter-spacing:-.03em;margin:0 0 6px}
h2{font-size:20px;line-height:1.2;margin:0 0 8px}
h3{font-size:17px;line-height:1.25;margin:0}
h4{font-size:15px;margin:0 0 4px}
.muted{color:var(--muted)}
.top{position:sticky;top:0;z-index:6;display:flex;align-items:center;justify-content:space-between;gap:12px;min-height:66px;padding:10px max(18px,calc((100vw - 1160px)/2));background:#ffffffeb;backdrop-filter:blur(16px);border-bottom:1px solid var(--line)}
html[data-theme=dark] .top{background:#121c16eb;border-color:#26372c}
.logo{display:flex;align-items:center;gap:10px;cursor:pointer}
.mark{display:grid;place-items:center;width:40px;height:40px;border-radius:13px;background:linear-gradient(145deg,#198955,#0f5e3a);color:#fff;font-weight:900;box-shadow:0 6px 18px #146c4330}
.logo b{font-size:17px;display:block;line-height:1.1}.logo small{display:block;color:var(--muted);font-size:12px}
.nav{display:flex;align-items:center;gap:4px}
.nav button,.navLink,.themeControl{display:inline-flex;align-items:center;gap:6px;height:40px;min-height:40px;margin:0;border:0;border-radius:11px;padding:0 12px;background:transparent;color:var(--muted);font-size:14px;font-weight:600;text-decoration:none;cursor:pointer;line-height:1}
.nav button:hover,.navLink:hover,.themeControl:hover{background:var(--brand2);color:var(--ink)}
.nav button.on{background:var(--brand2);color:var(--brand);box-shadow:inset 0 0 0 1px #cce5d4}
html[data-theme=dark] .nav button.on{box-shadow:inset 0 0 0 1px #2f5b42}
.themeControl{padding:0;overflow:hidden}
.themeControl select{display:block;width:auto;min-width:108px;height:40px;margin:0;padding:0 26px 0 10px;border:0;border-radius:11px;background:transparent;color:inherit;font-size:14px;font-weight:600;cursor:pointer}
main{max-width:1160px;margin:auto;padding:22px 18px 64px}
.view{display:none}.view.on{display:block}
.eyebrow{margin-bottom:4px;color:var(--brand);font-size:11px;font-weight:800;letter-spacing:.1em;text-transform:uppercase}
.card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:20px;box-shadow:var(--shadow)}
.badge{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:999px;padding:6px 12px;font-size:12px;font-weight:600;color:var(--muted);background:var(--card)}
.badge i{width:8px;height:8px;border-radius:50%;background:#d49a45;flex:0 0 auto}.badge.ok i{background:#2fb46e}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:6px;border:0;border-radius:11px;padding:10px 14px;background:var(--brand);color:#fff;font-size:14px;font-weight:700;text-decoration:none;cursor:pointer;transition:opacity .15s}
.btn.secondary{background:#e9efea;color:var(--ink)}
html[data-theme=dark] .btn.secondary,html[data-theme=dark] .close{background:#24332a;color:var(--ink)}
.btn.danger{background:#fff0ee;color:var(--danger)}
html[data-theme=dark] .btn.danger{background:#401e1c;color:#ffaaa2}
.btn:disabled{opacity:.48;cursor:not-allowed}
.cloudMessage{display:flex;gap:11px;align-items:flex-start;padding:12px 15px;border-radius:13px;background:var(--brand2);color:#204c33;margin:0 0 14px;font-size:13px}
.cloudMessage.warn{background:var(--warnbg);color:var(--warn)}
.cloudMessage.bad{background:#fff0ee;color:var(--danger)}
html[data-theme=dark] .cloudMessage{background:#193b29;color:#aee5c4}
html[data-theme=dark] .cloudMessage.warn{background:#3b3119;color:#ffd479}
html[data-theme=dark] .cloudMessage.bad{background:#3d1e1c;color:#ffaaa2}
.cloudMessage b{display:block;font-size:13px}.cloudMessage small{display:block;margin-top:2px}
#securityBanner{display:none!important}
.overviewTop{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:10px 14px;margin:0 0 16px}
.dashboardTools{display:flex;flex-wrap:wrap;justify-content:space-between;align-items:center;gap:10px 14px;flex:1 1 auto;min-width:0}
.dashboardToolsLead{display:flex;align-items:center;flex-wrap:wrap;gap:8px 12px}
.dashboardTools h2{margin:0;font-size:21px}
.dashboardTools .muted{font-size:12px}
.dashboardTools .actionRow{margin:0;gap:7px}
.dashboardTools[hidden],#sensorDetailBack[hidden],#dashboardLayout[hidden],[hidden]{display:none!important}
#sensorDetailBack{display:flex;align-items:center;gap:10px}
.sensorGrid{display:grid;grid-template-columns:1fr;gap:14px}
#sensorGrid.overviewGrid{grid-template-columns:repeat(auto-fit,minmax(min(100%,290px),1fr));gap:14px}
#sensorGrid.overviewGrid[data-columns="1"]{grid-template-columns:1fr}
#sensorGrid.overviewGrid[data-columns="2"]{grid-template-columns:repeat(2,minmax(0,1fr))}
#sensorGrid.overviewGrid[data-columns="3"]{grid-template-columns:repeat(3,minmax(0,1fr))}
#sensorGrid.detailGrid{grid-template-columns:1fr}
.sensor{padding:18px;transition:border-color .15s,box-shadow .15s}
.sensorSummary{display:flex;flex-direction:column;gap:12px;padding:16px;cursor:pointer}
.sensorSummary:hover{border-color:#9bc7aa}
html[data-theme=dark] .sensorSummary:hover{border-color:#446b53}
.comfortable .sensorSummary{padding:22px;gap:16px}
.comfortable .sensorSummary .value{min-height:86px}
.sensorTop{display:flex;align-items:center;justify-content:space-between;gap:10px}
.sensorTop code{font-size:11px;color:var(--muted)}
.sensorName{border:0;padding:0;min-height:40px;flex:1;min-width:0;text-align:left;background:transparent;color:var(--ink);cursor:pointer;display:flex;align-items:center;gap:9px}
.sensorNameText{min-width:0}
.sensorName h3{margin:0;font-size:16px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.sensorName .muted{display:block;font-size:12px}
.sensorOpen{min-width:36px;min-height:36px;padding:0;font-size:19px;border-radius:10px}
.sensorActions{display:flex;align-items:center;gap:7px;flex-wrap:wrap}
.fresh{width:9px;height:9px;border-radius:50%;background:#2fb46e;box-shadow:0 0 0 3px #2fb46e22;flex:0 0 auto}
.stale{background:#d49a45;box-shadow:0 0 0 3px #d49a4522}
.detailGrid .sensorTop{align-items:center;flex-wrap:wrap;gap:8px 12px;padding-bottom:10px;border-bottom:1px solid var(--line)}
.detailGrid .sensorTop>div:first-child{display:flex;align-items:center;flex-wrap:wrap;gap:6px 12px;min-width:0}
.detailGrid .sensorTop h3{font-size:21px}
.detailGrid .sensorIdentity{display:flex;align-items:center;flex-wrap:wrap;gap:6px;min-width:0}
.detailGrid .sensorIdentity code{overflow-wrap:anywhere}
.detailGrid .sensorIdentity small{font-size:11px;color:var(--muted)}
.detailGrid .sensorActions{margin-left:auto;justify-content:flex-end}
.detailMetaBar{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:6px 14px;padding:8px 0 4px;font-size:12px;color:var(--muted)}
.modeBadge,.accuracyBadge{display:inline-flex;align-items:center;padding:2px 8px;border-radius:999px;font:700 10px/1.5 system-ui,-apple-system,Segoe UI,sans-serif;background:var(--surface);color:var(--muted);border:1px solid var(--line)}
.modeBadge.setup{background:var(--warnbg);color:var(--warn);border-color:transparent}
.modeBadge.energy,.accuracyBadge{background:var(--brand2);color:var(--brand);border-color:transparent}
.modeBadge.recovery{background:#e8efff;color:#315fa8;border-color:transparent}
html[data-theme=dark] .modeBadge.recovery{background:#1b2c4a;color:#9cc0ff}
/* Unified measurement tile grid across all 11 sensors */
.values{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,142px),1fr));gap:9px;margin:10px 0 4px;align-items:stretch}
.sensorSummary .values{flex:1;grid-template-columns:repeat(auto-fit,minmax(min(100%,96px),1fr));gap:7px;margin:0}
.sensorSummary:has(.values>.value:only-child) .values{grid-template-columns:1fr}
.value{appearance:none;display:flex;flex-direction:column;justify-content:space-between;gap:4px;text-align:left;color:var(--ink);background:var(--surface);border:1px solid var(--line);border-radius:13px;padding:11px 12px;min-width:0;min-height:76px;cursor:pointer;transition:border-color .15s}
.value:hover{border-color:#8dc2a0}
.value.active{border-color:var(--brand);background:var(--brand2);box-shadow:inset 0 0 0 1px var(--brand)}
.value>span,.tileHead{display:flex;align-items:center;justify-content:space-between;gap:5px;color:var(--muted);font-size:11.5px;line-height:1.25}
.value b{display:block;font-size:21px;font-weight:750;line-height:1.15;font-variant-numeric:tabular-nums;margin:2px 0 0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.value b small{font-size:12px;font-weight:500;color:var(--muted)}
.sensorSummary .value{min-height:68px;padding:9px 10px}
.sensorSummary .value b{font-size:19px}
.tileSub,.fieldBadge{display:inline-flex;align-items:center;align-self:flex-start;padding:2px 7px;border-radius:7px;background:var(--brand2);color:var(--brand);font-size:10.5px;font-weight:650;line-height:1.35;max-width:100%;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.tileSub.mutedTag,.fieldBadge.off{background:#e9efea;color:var(--muted)}
html[data-theme=dark] .tileSub.mutedTag,html[data-theme=dark] .fieldBadge.off{background:#24332a}
.tileSub.warnTag{background:var(--warnbg);color:var(--warn)}
.iaqTile .iaqTop{display:flex;align-items:baseline;justify-content:space-between;gap:6px}
.iaqCategory{flex:0 0 auto;padding:2px 7px;border-radius:999px;background:var(--iaq-color);color:var(--iaq-ink,#fff);font-size:10px;font-weight:800;line-height:1.3}
.iaqAdvice{display:block;color:var(--muted);font-size:11px;line-height:1.3}
.sensorSummary .fieldBadge,.sensorSummary .iaqAdvice,.sensorSummary .iaqRange,.sensorSummary .helper,.sensorSummary .pulseStatus{display:none}
.pulseStatus{grid-column:1/-1;display:flex;align-items:center;justify-content:space-between;gap:8px;padding:8px 11px;border-radius:11px;background:var(--brand2);color:var(--brand);font-size:12px;font-weight:600}
.pulseStatus.warn{background:var(--warnbg);color:var(--warn)}
.meta{display:flex;justify-content:space-between;gap:8px;color:var(--muted);font-size:11.5px;font-variant-numeric:tabular-nums}
.sensorFoot{display:grid;grid-template-columns:minmax(0,1fr);gap:12px;margin-top:10px;padding-top:12px;border-top:1px solid var(--line)}
.recordingSection{min-width:0;width:100%}
.channelTag{display:inline-flex;gap:6px;align-items:center;padding:5px 10px;border-radius:9px;background:var(--surface);border:1px solid var(--line);color:var(--muted);font-size:12px;text-decoration:none}
.channelTag.ok{background:var(--brand2);color:var(--brand);border-color:transparent}
/* Collapsible sections: compact, flat, no jarring bloat */
.currentReadings,.deviceInformation{margin:8px 0 0}
.currentReadings>summary,.deviceInformation>summary{min-height:36px;display:flex;align-items:center;gap:8px;cursor:pointer;font-size:13px;font-weight:650;color:var(--muted);list-style:none;user-select:none}
.currentReadings>summary::-webkit-details-marker,.deviceInformation>summary::-webkit-details-marker,.chartOptions>summary::-webkit-details-marker{display:none}
.currentReadings>summary:after,.deviceInformation>summary:after,.chartOptions>summary:after{content:'+';margin-left:auto;font-size:17px;font-weight:500;line-height:1}
.currentReadings[open]>summary:after,.deviceInformation[open]>summary:after,.chartOptions[open]>summary:after{content:'−'}
.deviceInformation{margin-top:12px;padding-top:8px;border-top:1px solid var(--line)}
.deviceContent{display:grid;gap:8px;padding-top:6px}
.chartOptions{margin:8px 0 0;border:1px solid var(--line);border-radius:12px;padding:0 12px;background:var(--surface)}
.chartOptions>summary{min-height:38px;display:flex;align-items:center;gap:8px;cursor:pointer;font-size:13px;font-weight:650;color:var(--muted);list-style:none}
.chartOptionsBody{display:flex;flex-wrap:wrap;align-items:center;gap:8px 12px;padding:6px 0 10px}
/* Charts: crisp, responsive, zero-jump */
.chartPanel{min-width:0}
.chartPrimary{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(0,1fr);gap:10px;margin-bottom:8px}
.chartPrimary label{margin:0;font-size:12px;font-weight:650;color:var(--muted)}
.chartPrimary select{margin-top:4px;padding:9px 10px}
.chartNavigation{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:6px 10px;margin:6px 0}
.chartNavigationButtons{display:flex;flex-wrap:wrap;gap:6px}
.chartNavigation .btn{padding:7px 11px;font-size:12px;min-height:36px}
.chartNavigation>span{font-size:12px;font-variant-numeric:tabular-nums}
.chartStatusSlot{min-height:22px;display:flex;align-items:center;flex-wrap:wrap;gap:6px;margin:2px 0 4px;font-size:12px;color:var(--muted)}
.chartStatusSlot p{margin:0}
.historyError{color:var(--danger);display:inline-flex;align-items:center;gap:8px;flex-wrap:wrap}
.historyError .btn{padding:5px 10px;font-size:12px}
.chartStage{min-height:208px}
.chartLoadingBox{display:grid;place-items:center;min-height:190px;border:1px dashed var(--line);border-radius:12px;background:var(--surface);color:var(--muted);font-size:13px}
.livePlot,.historyPlot{min-width:0;margin:8px 0 14px}
.plotHeading{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:6px 12px;margin-bottom:4px}
.plotStats{display:inline-flex;align-items:center;flex-wrap:wrap;gap:6px;font-size:11.5px;color:var(--muted);font-variant-numeric:tabular-nums}
.plotStatPill{padding:2px 7px;border-radius:7px;background:var(--surface);border:1px solid var(--line)}
.chartLegendGroup{display:flex;flex-wrap:wrap;gap:12px}
.chartLegend{display:inline-flex;align-items:center;gap:5px;font-size:12.5px;font-weight:650}
.liveSvg,.chart.historySvg{display:block;width:100%;height:auto;min-height:176px;fill:var(--muted);font:inherit;overflow:visible;touch-action:pan-y pinch-zoom}
.spectrumSvg{display:block;width:100%;height:auto;max-height:220px;color:var(--ink);overflow:visible}
.liveSvg text,.chart.historySvg text{font-size:var(--chart-label-size,11px);font-variant-numeric:tabular-nums;fill:var(--muted)}
.chart .grid,.liveSvg .grid{stroke:var(--line);stroke-width:1}
.chart .minor{stroke:var(--line);stroke-width:.6;opacity:.55}
.liveSvg .cursor,.chart .cursor{stroke:var(--ink);stroke-width:1;stroke-dasharray:3 3;opacity:.75}
.chart .cursorDot{fill:var(--chart-bg);stroke:var(--brand);stroke-width:2.5}
.rawTrace{opacity:.24}
.seriesArea{pointer-events:none}
.cursorSummary{display:none}
.sampleDot{stroke:var(--chart-bg);stroke-width:.7;pointer-events:none}
.chartReadout{display:flex;align-items:center;flex-wrap:wrap;gap:4px 14px;min-height:48px;margin-top:6px;padding:7px 11px;border:1px solid var(--line);border-radius:11px;background:var(--surface);font-size:13px;font-variant-numeric:tabular-nums;line-height:1.4}
.chartReadoutTime{font-weight:650;color:var(--ink)}
.chartReadoutValue{display:inline-flex;align-items:center;gap:5px;white-space:nowrap;color:var(--ink)!important}
.chartReadoutSwatch{width:8px;height:8px;border-radius:50%;flex:0 0 auto}
.chartReadoutValue b{font-size:14.5px}
.chartCursorActions{display:inline-flex;gap:4px;margin-left:auto}
.chartCursorActions .btn{padding:2px 9px;min-width:34px;min-height:34px;font-size:17px}
.chartToolbar{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px;margin:6px 0}
.chartFilter{display:inline-flex;align-items:center;gap:7px;margin:0;font-size:12px;color:var(--muted)}
.chartFilter select{width:auto;min-width:150px;padding:7px 9px}
.chartFilterNote{font-size:12px;color:var(--muted)}
.chartEmpty{display:grid;place-items:center;min-height:176px;font-size:13px;padding:18px 12px;border:1px dashed var(--line);border-radius:12px;text-align:center}
.historyStorageInfo{padding-top:8px;font-size:11.5px;color:var(--muted);line-height:1.45}
.liveSvg:focus-visible,.chart.historySvg:focus-visible{outline:2px solid var(--brand);outline-offset:2px;border-radius:6px}
/* Recordings & live-to-archive sync panel */
.recordingPanel{width:100%;margin-bottom:12px;padding:12px 14px;border:1px solid var(--line);border-radius:14px;background:var(--surface)}
.chartHead{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:8px;margin-bottom:6px}
.chartHead:last-child{margin-bottom:0}
.recordingPanel select{width:auto;max-width:100%;min-width:160px;padding:8px 10px;background:var(--card)}
.recordingPanel .btn{padding:7px 11px;font-size:13px}
.recordingPanel p{margin:4px 0 0;font-size:12px}
.recordingNav{display:flex;flex-wrap:wrap;align-items:center;gap:6px 8px;margin-top:8px;padding-top:8px;border-top:1px solid var(--line)}
.recordingNav label{display:inline-flex;align-items:center;gap:6px;margin:0;font-size:12px}
.recordingNav span{font-size:12px;font-variant-numeric:tabular-nums;color:var(--muted)}
.syncBar{height:6px;border-radius:999px;background:var(--line);overflow:hidden;margin:6px 0 2px}
.syncBar i{display:block;height:100%;background:var(--brand);border-radius:inherit;transition:width .25s ease}
/* Quick feedback */
.sensorFeedback{border:1px solid var(--line);border-radius:14px;padding:12px 14px;margin:8px 0 12px;background:var(--surface)}
.feedbackGrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:8px;margin-top:8px}
.feedbackValue{min-width:0;padding:10px 11px;border:1px solid var(--line);border-radius:11px;background:var(--card)}
.feedbackValue span{display:block;color:var(--muted);font-size:11px;margin-bottom:3px}
.feedbackValue b{display:block;font-size:18px;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.sensorFeedback p{font-size:11.5px;margin:8px 0 0}
/* Arrange, Pending, Settings, Modal */
.empty{text-align:center;padding:38px 18px}
.pulse{width:50px;height:50px;border-radius:50%;background:var(--brand2);margin:0 auto 12px;display:grid;place-items:center;color:var(--brand);font-size:22px}
.pendingPanel{display:none;margin-bottom:14px}.pendingPanel.on{display:block}
.pendingList{display:grid;gap:9px}
.pendingItem{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:12px 14px;border:1px solid var(--line);border-radius:13px;background:var(--surface)}
.pendingItem code{display:block;color:var(--muted);font-size:11px}
.dashboardLayout{margin:0 0 16px}
.layoutRows{margin:12px 0}
.layoutRow{display:flex;align-items:center;gap:6px;padding:8px 0;border-bottom:1px solid var(--line)}
.layoutName{flex:1;min-width:0;font-size:14px}
.layoutRow button{min-width:38px;min-height:40px;padding:6px}
.layoutVisibility{display:flex;align-items:center;justify-content:center;min-width:42px;min-height:42px;margin:0}
.layoutVisibility input{width:18px;height:18px;accent-color:var(--brand)}
.sectionHead{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:22px 0 10px}
.settingsHero{margin-bottom:16px}
.settings{display:grid;grid-template-columns:repeat(12,1fr);gap:14px;align-items:start}
.settingsCard{grid-column:span 6;padding:20px}
.settingsCard.wide{grid-column:span 7}.settingsCard.narrow{grid-column:span 5}.settingsCard.full{grid-column:1/-1}
.settingsCardHead{display:flex;align-items:flex-start;gap:11px;margin-bottom:12px}
.settingsCardHead h2{margin-bottom:2px;font-size:18px}
.cardIcon{display:grid;place-items:center;flex:0 0 38px;height:38px;border-radius:11px;background:var(--brand2);font-size:18px}
.settingsDescription{color:var(--muted);font-size:12.5px}
.securityStatus{display:flex;align-items:center;gap:8px;padding:9px 11px;border-radius:10px;background:var(--warnbg);color:var(--warn);font-size:13px;font-weight:600}
.securityStatus.protected{background:var(--brand2);color:var(--brand)}
.securityStatus:before{content:"●"}
.saveBar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:14px 18px}
.saveActions{display:flex;flex-wrap:wrap;gap:8px}
.settingsDivider{display:flex;align-items:end;justify-content:space-between;gap:12px;margin:28px 0 12px;padding-top:20px;border-top:1px solid var(--line)}
.dangerCard{border-color:#f1d5d2}
html[data-theme=dark] .dangerCard{border-color:#5a2b27;background:#261615}
.dangerCard .cardIcon{background:#fff0ee}
html[data-theme=dark] .dangerCard .cardIcon{background:#401e1c}
.checkLine{display:flex;align-items:center;gap:8px;margin-top:10px;font-size:13px;color:var(--ink)}
.checkLine input{width:auto}
.checkLine.removePassword{padding:9px 11px;border-radius:10px;background:#fff4f2;color:var(--danger)}
html[data-theme=dark] .checkLine.removePassword{background:#3d1e1c;color:#ffaaa2}
label{display:block;color:var(--muted);font-size:12.5px;font-weight:600;margin:11px 0 4px}
input,select{width:100%;padding:10px 12px;border:1px solid var(--line);border-radius:10px;background:var(--surface);color:var(--ink);outline:0}
input:focus,select:focus{border-color:var(--brand);box-shadow:0 0 0 3px #146c431f}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.formActions,.actionRow{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}
.intervalControl{display:grid;gap:7px}
.customInterval{display:none;grid-template-columns:1fr auto;align-items:center;gap:8px}
.customInterval.on{display:grid}
.customInterval span{color:var(--muted);font-size:13px}
.channelGrid,.remoteGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}
.channelCard,.remoteCard{padding:16px;border:1px solid var(--line);border-radius:15px;background:var(--card)}
.channelTitle,.remoteSummary{display:flex;align-items:center;justify-content:space-between;gap:10px}
.remoteCard summary{list-style:none;cursor:pointer}.remoteCard summary::-webkit-details-marker{display:none}
.remoteMeta{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.urlBox{display:flex;gap:6px;align-items:center;padding:7px 9px;margin-top:8px;border-radius:9px;background:var(--surface);overflow:hidden}
.urlBox code{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-size:11px;flex:1}
.remoteForm{padding-top:12px;margin-top:12px;border-top:1px solid var(--line)}
.fieldNames{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.dangerZone{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px;padding-top:12px;border-top:1px solid #f1d2cf}
.keyText{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:13px}
.notice{display:none;padding:11px 14px;border-radius:11px;margin-bottom:14px;background:var(--brand2);color:var(--brand);font-size:13px}
.helper{padding:9px 11px;border-radius:10px;background:var(--brand2);color:#204c33;font-size:12.5px;margin-top:8px}
html[data-theme=dark] .helper{color:#b7e5c8}
.toast{position:fixed;right:16px;bottom:16px;z-index:20;max-width:360px;padding:12px 15px;border-radius:12px;background:#143e27;color:#fff;box-shadow:var(--shadow);transform:translateY(140px);transition:.22s}
.toast.on{transform:none}.toast.bad{background:#7b241e}
.modal{display:none;position:fixed;inset:0;z-index:15;background:#101a148c;padding:16px;overflow:auto}
.modal.on{display:grid;place-items:center}
.modalCard{width:min(680px,100%);background:var(--card);border:1px solid var(--line);border-radius:20px;padding:clamp(18px,4vw,28px);box-shadow:0 24px 80px #0006}
.modalHead{display:flex;justify-content:space-between;align-items:flex-start;gap:12px}
.close{border:0;background:#e9efea;width:36px;height:36px;border-radius:10px;cursor:pointer;font-size:18px;flex:0 0 auto}
.wizardProgress{display:flex;gap:6px;margin:16px 0}
.wizardProgress i{height:5px;flex:1;background:var(--line);border-radius:8px}
.wizardProgress i.on{background:var(--brand)}
.wizardStep{display:none}.wizardStep.on{display:block}
.fieldGrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.fieldRow{padding:10px 12px;border:1px solid var(--line);border-radius:11px;background:var(--surface)}
.fieldRow label{margin:0 0 5px;color:var(--ink)}
.quickChannel{display:none;margin-top:10px;padding:12px;border-radius:12px;background:var(--surface);border:1px solid var(--line)}
.quickChannel.on{display:block}
.wizardNav{display:flex;justify-content:space-between;gap:8px;margin-top:20px}
.calibrationBox{margin-top:12px;padding:12px;border:1px solid var(--line);border-radius:12px;background:var(--surface)}
.calibrationBox summary{cursor:pointer;font-weight:650;font-size:13.5px}
.calSection{margin-top:12px;padding-top:10px;border-top:1px solid var(--line)}
.restartOverlay{display:none;position:fixed;inset:0;z-index:50;background:#f7f8f5ee;backdrop-filter:blur(12px);place-items:center;padding:20px}
html[data-theme=dark] .restartOverlay{background:#0d1510ee}
.restartOverlay.on{display:grid}
.restartBox{width:min(440px,100%);text-align:center;background:var(--card);border:1px solid var(--line);padding:28px;border-radius:20px;box-shadow:var(--shadow)}
.restartSpinner{width:46px;height:46px;margin:0 auto 16px;border:4px solid var(--brand2);border-top-color:var(--brand);border-radius:50%;animation:turn 1s linear infinite}
@keyframes turn{to{transform:rotate(360deg)}}
@media(max-width:900px){.channelGrid,.remoteGrid{grid-template-columns:1fr}.settingsCard.wide,.settingsCard.narrow,.settingsCard{grid-column:span 6}.saveBar{grid-column:1/-1}}
@media(max-width:760px){.top{padding:8px 12px;min-height:58px}main{padding:16px 12px 56px}.nav button span,.navLink span{display:none}.nav button,.navLink{padding:0 9px}.themeControl select{width:56px;min-width:56px;padding:0 16px 0 8px}.settings{grid-template-columns:1fr}.settingsCard.wide,.settingsCard.narrow,.settingsCard,.settingsCard.full,.saveBar{grid-column:auto}.saveBar{align-items:stretch;flex-direction:column}.saveActions .btn{flex:1}.row,.fieldGrid,.fieldNames{grid-template-columns:1fr}#sensorGrid.overviewGrid[data-columns="3"]{grid-template-columns:repeat(2,minmax(0,1fr))}.detailGrid .sensor{padding:14px}.detailGrid .sensorTop h3{font-size:19px}.chartPrimary{gap:8px}.chartNavigation .btn{padding:7px 9px;font-size:12px}}
@media(max-width:560px){#sensorGrid.overviewGrid,#sensorGrid.overviewGrid[data-columns="2"],#sensorGrid.overviewGrid[data-columns="3"]{grid-template-columns:1fr}.dashboardTools{flex-direction:column;align-items:stretch}.dashboardTools .actionRow{justify-content:flex-start}.sensorSummary .values{grid-template-columns:repeat(auto-fit,minmax(min(100%,94px),1fr))}.detailGrid .values{grid-template-columns:repeat(2,minmax(0,1fr))}.chartPrimary select{padding:10px 8px;font-size:16px}.chartReadout{min-height:78px;font-size:12.5px;gap:4px 10px}.chartReadoutTime{flex-basis:100%}.feedbackGrid{grid-template-columns:repeat(2,minmax(0,1fr))}.feedbackValue b{font-size:16px}.pendingItem{align-items:flex-start;flex-direction:column}.pendingItem .btn{width:100%}.wizardNav .btn{flex:1}.recordingPanel select{width:100%}}
@media(pointer:coarse){.sensorOpen,.dashboardTools .btn,.chartNavigation .btn,.chartCursorActions .btn{min-height:44px}.chartCursorActions .btn{min-width:44px}.sensorSummary .value{min-height:76px}}
</style></head><body><header class="top"><div class="logo" id="homeLogo" role="button" tabindex="0" aria-label="Open overview"><div class="mark">⚡</div><div><b>W-Charger</b><small>Station</small></div></div><nav class="nav" aria-label="Main navigation"><button class="on" data-view="overview" aria-label="Overview">🏠 <span>Overview</span></button><button data-view="settings" aria-label="Settings">⚙️ <span>Settings</span></button><button id="logoutLink" class="navLink" type="button" style="display:none" aria-label="Sign out" onclick="signOut()">↪ <span>Sign out</span></button><label class="themeControl" for="dashboardTheme"><select id="dashboardTheme" class="themeMode" aria-label="Color theme"><option value="system">🖥️ System</option><option value="light">☀️ Light</option><option value="dark">🌙 Dark</option></select></label></nav></header><main>
<section id="overview" class="view on"><span id="sensorCount" hidden>0</span><span id="onlineCount" hidden>0</span><span id="pendingCount" hidden>0</span><span id="cloudState" hidden>–</span><div id="storageMessage" class="cloudMessage bad" role="status" style="display:none"></div><div id="securityBanner" style="display:none"></div><div id="cloudMessage" style="display:none"></div><div id="pendingPanel" class="card pendingPanel"><div class="sectionHead" style="margin:0 0 10px"><div><h2>Add sensor</h2><div class="muted">Detected nearby devices ready to configure.</div></div><button class="btn secondary" id="closePending">Close</button></div><div id="pendingList" class="pendingList"></div></div><div class="overviewTop"><div id="dashboardTools" class="dashboardTools"><div class="dashboardToolsLead"><h2 id="dashboardTitle">Your sensors</h2><span id="visibleSensorCount" class="muted"></span></div><div class="actionRow"><button class="btn secondary" onclick="openDashboardLayout()">Arrange</button><button id="toggleHiddenBtn" class="btn secondary" hidden onclick="showHiddenSensors=!showHiddenSensors;renderSensors(statusData.sensors.filter(n=>n.provisioned))">Show hidden</button><button class="btn" id="addSensorButton">+ Find sensors</button></div></div><div id="sensorDetailBack" hidden><button class="btn secondary" onclick="closeSensorDetail()">← All sensors</button></div><div id="connection" class="badge"><i></i><span>Checking connection</span></div></div><section id="dashboardLayout" class="card dashboardLayout" hidden></section><div id="sensorGrid" class="sensorGrid"></div></section>
<section id="settings" class="view">
<div class="settingsHero"><div class="eyebrow">Station control</div><h1>Settings</h1><div class="muted">Network, access, measurement defaults and optional ThingSpeak cloud.</div></div><div id="notice" class="notice"></div>
<div id="settingsStorageMessage" class="cloudMessage bad" role="status" style="display:none;margin-bottom:14px"></div><form id="settingsForm" class="settings" data-api="/api/station"><div class="card settingsCard wide"><div class="settingsCardHead"><div class="cardIcon">📶</div><div><h2>Home Wi-Fi</h2><div class="settingsDescription">Network used for the dashboard and optional cloud uploads.</div></div></div><div id="currentWifi" class="helper">Loading connection status …</div><button type="button" id="wifiScanButton" class="btn secondary" style="margin-top:10px">Search networks</button><label for="settingsSsid">Wi-Fi network</label><select id="settingsSsid" name="ssid"></select><label for="settingsWifiPassword">New Wi-Fi password</label><input id="settingsWifiPassword" name="wifiPassword" type="password" maxlength="64" autocomplete="new-password" placeholder="Blank keeps current password"></div><div class="card settingsCard narrow"><div class="settingsCardHead"><div class="cardIcon">⏱️</div><div><h2>Measurement</h2><div class="settingsDescription">Defaults for newly added sensors.</div></div></div><label>Default interval</label><div id="defaultInterval" class="intervalControl"></div><label for="fallbackChannel">Fallback radio channel</label><input id="fallbackChannel" name="fallbackChannel" type="number" min="1" max="13"><div class="settingsDescription" style="margin-top:6px">Match your 2.4 GHz home Wi-Fi channel if offline recovery is needed.</div></div><div class="card settingsCard wide"><div class="settingsCardHead"><div class="cardIcon">🔒</div><div><h2>Website access</h2><div class="settingsDescription">Protect settings, APIs and saved cloud keys on your network.</div></div></div><div id="passwordState" class="securityStatus">Loading protection status …</div><div class="row"><div><label for="newAdminPassword">New website password</label><input id="newAdminPassword" name="newAdminPassword" type="password" minlength="8" maxlength="64" autocomplete="new-password" placeholder="Blank keeps current setting"></div><div><label for="confirmAdminPasswordSettings">Confirm password</label><input id="confirmAdminPasswordSettings" name="confirmAdminPassword" type="password" minlength="8" maxlength="64" autocomplete="new-password" placeholder="Repeat password"></div></div><label class="checkLine removePassword"><input id="removeAdminPassword" name="removeAdminPassword" type="checkbox"> Remove website password protection</label><div class="settingsDescription" style="margin-top:8px">Minimum 8 characters. Changing protection signs this browser out.</div></div><div class="card settingsCard narrow"><div class="settingsCardHead"><div class="cardIcon">📡</div><div><h2>Recovery Wi-Fi</h2><div class="settingsDescription">Automatic fallback access point during Wi-Fi outages.</div></div></div><div style="margin-bottom:4px"><b id="apName">W-Charger</b></div><div id="apState" class="helper">Loading status …</div><label for="apPassword">Setup Wi-Fi password</label><input id="apPassword" name="apPassword" type="password" minlength="8" maxlength="63" autocomplete="new-password"></div><div class="card settingsCard full"><div class="settingsCardHead"><div class="cardIcon">☁️</div><div><h2>ThingSpeak account (optional)</h2><div class="settingsDescription">Only needed if you manage ThingSpeak channels from this station.</div></div></div><label for="userApiKey">User API Key</label><input id="userApiKey" name="userApiKey" class="keyText" type="text" maxlength="40" autocomplete="off" placeholder="Optional · leave blank for local-only use"></div><div class="card saveBar full"><div><b>Apply station settings</b><div class="settingsDescription">Restarts automatically only when Wi-Fi or radio settings change.</div></div><div class="saveActions"><button class="btn" type="submit">Save settings</button><button class="btn secondary" type="submit" data-restart="1">Save and restart</button></div></div></form>
<div class="settingsDivider"><div><div class="eyebrow">Optional cloud</div><h2>ThingSpeak channels</h2><div class="muted">Load or create channels from your ThingSpeak account.</div></div><button id="loadCloudChannels" class="btn secondary" type="button">↻ Load channels</button></div>
<div id="cloudAccountMessage" class="helper">Save your User API Key first, then load channels.</div><div id="remoteChannelGrid" class="remoteGrid" style="margin-top:12px"></div>
<details class="card" style="margin-top:12px"><summary class="channelTitle"><div><h3>Create new cloud channel</h3><div class="muted" style="font-size:12px">Name, description and up to eight fields.</div></div><span class="fieldBadge">+</span></summary><form id="cloudCreateForm" class="remoteForm"><label>Name</label><input name="name" maxlength="80" required placeholder="e.g. Home climate"><label>Description</label><input name="description" maxlength="255" placeholder="Optional"><div class="fieldNames"><div><label>Field 1</label><input name="field1" maxlength="80" placeholder="Temperature"></div><div><label>Field 2</label><input name="field2" maxlength="80" placeholder="Humidity"></div><div><label>Field 3</label><input name="field3" maxlength="80"></div><div><label>Field 4</label><input name="field4" maxlength="80"></div><div><label>Field 5</label><input name="field5" maxlength="80"></div><div><label>Field 6</label><input name="field6" maxlength="80"></div><div><label>Field 7</label><input name="field7" maxlength="80"></div><div><label>Field 8</label><input name="field8" maxlength="80"></div></div><div class="row"><div><label>Tags</label><input name="tags" maxlength="160" placeholder="home, climate"></div><div><label>External URL</label><input name="url" maxlength="200" placeholder="https://..."></div></div><div class="fieldNames"><div><label>Latitude</label><input name="latitude" type="number" min="-90" max="90" step="any" placeholder="Optional"></div><div><label>Longitude</label><input name="longitude" type="number" min="-180" max="180" step="any" placeholder="Optional"></div><div><label>Elevation (m)</label><input name="elevation" type="number" min="-10000" max="100000" step="any" placeholder="Optional"></div></div><label>Metadata</label><input name="metadata" maxlength="512" placeholder="Optional"><label class="checkLine"><input name="publicFlag" type="checkbox"> Show channel publicly</label><button class="btn" style="margin-top:12px" type="submit">Create channel</button></form></details>
<div class="sectionHead"><div><h2>Local channel profiles</h2><div class="muted" style="font-size:12px">Up to six reusable channel assignments.</div></div></div><div id="channelGrid" class="channelGrid"></div>
<div class="card dangerCard" style="margin-top:16px"><div class="settingsCardHead"><div class="cardIcon">⚠️</div><div><h2>Factory reset</h2><div class="settingsDescription">Clears local Wi-Fi, password, sensors and stored history. ThingSpeak channels are untouched.</div></div></div><form data-api="/api/factory-reset"><label>Enter RESET to confirm</label><input name="confirm" autocomplete="off"><button class="btn danger" style="margin-top:12px" type="submit">Reset W-Charger</button></form></div></section>
</main>
<div id="sensorModal" class="modal"><div class="modalCard"><div class="modalHead"><div><h2 id="wizardTitle">Configure sensor</h2><div id="wizardSubtitle" class="muted" style="font-size:12px"></div></div><button type="button" class="close" id="closeWizard" aria-label="Close">×</button></div><div id="wizardProgress" class="wizardProgress"></div><form id="sensorWizardForm"><input id="wizardMac" name="mac" type="hidden"><input name="channelId" type="hidden" value="0"><input name="writeKey" type="hidden" value="">
<div class="wizardStep"><label for="wizardName">Name</label><input id="wizardName" name="name" maxlength="24"><label for="wizardSensorType">Sensor board</label><select id="wizardSensorType" name="sensorType"><option value="1">BME280 · temperature, humidity, pressure</option><option value="2">BME680 · climate + IAQ</option><option value="6">BMP280 · temperature, pressure</option><option value="7">BME688 · climate + IAQ (BSEC2)</option><option value="8">CCS811 · eCO₂ and TVOC</option><option value="9">TOF400C-VL53L1X · laser distance (up to 4 m)</option><option value="10">AS7341 · 10-channel optical spectrum</option><option value="3">LSM6DSOX · acceleration &amp; angular rate</option><option value="4">TMP117 · precision temperature</option><option value="5">MAX30102 · heart rate &amp; optical</option><option value="0">Auto detect</option><option value="255">Battery only (no I²C sensor)</option></select><div id="sensorTypeHint" class="helper"></div><div id="wizardIntervalGroup"><label>Measurement interval</label><div id="wizardInterval" class="intervalControl"></div></div><details class="calibrationBox"><summary>Calibration &amp; correction</summary><div id="wizardTempOffsetGroup" class="calSection"><label for="wizardTemperatureOffset">BME680/BME688 temperature offset (°C)</label><input id="wizardTemperatureOffset" name="temperatureOffsetC" type="number" min="-10" max="10" step="0.1" value="0.466"></div><div id="iaqCalibrationGuide" class="calSection"><b>IAQ learning state (BSEC ULP · 5 min)</b><div id="iaqCalibrationState" class="helper"></div><button id="resetIaqCalibration" class="btn secondary" type="button" style="margin-top:8px">Restart IAQ learning state</button></div><div class="calSection"><b>Battery voltage calibration</b><div id="batteryCalibrationState" class="helper"></div><label for="batteryReferenceVoltage">Multimeter reference voltage (V)</label><input id="batteryReferenceVoltage" name="batteryReferenceVoltage" type="number" min="2" max="5" step="0.001" placeholder="e.g. 3.842"><label class="checkLine"><input id="resetBatteryCalibration" name="resetBatteryCalibration" type="checkbox"> Reset calibration to default</label></div></details></div>
<div class="wizardStep"><h3>Cloud upload (optional)</h3><label class="checkLine"><input id="wizardUpload" name="uploadEnabled" type="checkbox"> Upload measurements to ThingSpeak</label><label for="wizardProfile">ThingSpeak channel</label><select id="wizardProfile" name="profileSlot"></select><div id="profileHint" class="helper"></div><button type="button" id="showQuickChannel" class="btn secondary" style="margin-top:10px">+ Add another channel</button><div id="quickChannel" class="quickChannel"><div class="row"><div><label>Channel name</label><input id="quickName" maxlength="24"></div><div><label>Channel ID</label><input id="quickId" type="number" min="1"></div></div><div class="row"><div><label>Read API Key</label><input id="quickRead" type="text" maxlength="32"></div><div><label>Write API Key</label><input id="quickWrite" type="text" maxlength="32"></div></div><div class="actionRow"><button type="button" id="saveQuickChannel" class="btn">Save existing channel</button><button type="button" id="autoCreateChannel" class="btn secondary">Create channel automatically</button></div></div></div>
<div class="wizardStep"><h3>ThingSpeak field mapping</h3><p class="muted" style="font-size:12.5px;margin:4px 0 10px">Only fields supported by this sensor are shown.</p><div class="fieldGrid"><div class="fieldRow" data-ts-field="temperature"><label>Temperature</label><select id="wizardTemperature" name="temperatureField"></select></div><div class="fieldRow" data-ts-field="humidity"><label>Humidity</label><select id="wizardHumidity" name="humidityField"></select></div><div class="fieldRow" data-ts-field="pressure"><label>Pressure</label><select id="wizardPressure" name="pressureField"></select></div><div class="fieldRow" data-ts-field="iaq"><label>IAQ</label><select id="wizardIaq" name="iaqField"></select></div><div class="fieldRow" data-ts-field="gas"><label>Gas resistance</label><select id="wizardGasResistance" name="gasResistanceField"></select></div><div class="fieldRow" data-ts-field="battery"><label>Battery</label><select id="wizardBattery" name="batteryField"></select></div></div></div><div class="wizardNav"><button id="deleteSensor" type="button" class="btn danger">Delete sensor</button><button id="resetSensor" type="button" class="btn danger">Reset sensor</button><button id="wizardBack" type="button" class="btn secondary">Back</button><button id="wizardNext" type="button" class="btn">Next</button></div></form></div></div><div id="toast" class="toast"></div>
<script>
let csrf='',config={},statusData={sensors:[],cloud:{}},remoteChannels=[],wizardStep=0,currentSensor=null,wizardGeneration=0,sensorSaveActive=false,chartMetrics={},chartWindows={},stationHistory={},historyRevisions={},historyBuckets={},statusRequestActive=false,statusRequestStartedAt=0,statusFailureCount=0,sensorRenderSignature='';const $=id=>document.getElementById(id),esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])),intervalPresets=[1,2,5,10,30,60,300,600,900,1800,3600];async function requestJson(url,timeout=8000){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeout);try{const response=await fetch(url,{cache:'no-store',signal:controller.signal});if(response.status===401){location.replace('/login');throw new Error('Authentication required')}if(!response.ok)throw new Error(`HTTP ${response.status}`);return await response.json()}finally{clearTimeout(timer)}}document.body.insertAdjacentHTML('beforeend','<div id="restartOverlay" class="restartOverlay"><div class="restartBox"><div class="restartSpinner"></div><h2>W-Charger is restarting</h2><p id="restartStatus" class="muted">Applying settings. The station will be detected automatically.</p><button id="retryRestart" class="btn secondary" style="display:none" type="button">Try again</button></div></div>');wchargerTheme.bind();
document.querySelectorAll('.nav button[data-view]').forEach(b=>b.onclick=()=>{if(b.dataset.view==='overview'){openDashboardHome();return}history.replaceState(null,'','#settings');showView('settings')});function showView(id){document.querySelectorAll('.nav button[data-view]').forEach(b=>b.classList.toggle('on',b.dataset.view===id));document.querySelectorAll('.view').forEach(v=>v.classList.toggle('on',v.id===id))}if(location.hash==='#settings')showView('settings');addEventListener('hashchange',()=>showView(location.hash==='#settings'?'settings':'overview'));
function age(ms){if(!ms&&ms!==0)return'–';if(ms<15000)return'just now';const m=Math.round(ms/60000);if(m<60)return`${m} min ago`;const h=Math.round(m/60);return h<48?`${h} h ago`:`${Math.round(h/24)} d ago`}function metric(v,d=1){return v!==null&&v!==''&&Number.isFinite(Number(v))?Number(v).toFixed(d):'–'}function fieldOptions(value){let html='<option value="0">Do not upload</option>';for(let i=1;i<=8;i++)html+=`<option value="${i}" ${Number(value)===i?'selected':''}>Field ${i}</option>`;return html}function fieldName(v){return Number(v)?`Field ${v}`:'Do not upload'}
function sensorTypeName(type){return({0:'Auto detect',1:'BME280',2:'BME680',3:'LSM6DSOX',4:'TMP117',5:'MAX30102',6:'BMP280',7:'BME688',8:'CCS811',9:'TOF400C-VL53L1X',10:'AS7341',255:'No environmental sensor'})[Number(type)]||'Unknown'}function iaqAccuracyText(value){return['0 · learning','1 · low','2 · medium','3 · high'][Number(value)]||'unknown'}function iaqBand(value){const v=Number(value);if(!Number.isFinite(v))return null;if(v<=50)return{range:'0–50',category:'Excellent',advice:'Indoor air quality is excellent.',color:'#169bd5'};if(v<=100)return{range:'51–100',category:'Good',advice:'Indoor air quality is good.',color:'#2db84d'};if(v<=150)return{range:'101–150',category:'Lightly polluted',advice:'Consider ventilating the room.',color:'#f1df16',ink:'#172019'};if(v<=200)return{range:'151–200',category:'Moderately polluted',advice:'Ventilation is recommended.',color:'#ff6b16'};if(v<=250)return{range:'201–250',category:'Heavily polluted',advice:'Ventilate the room and check for pollutant sources.',color:'#9b1464'};if(v<=350)return{range:'251–350',category:'Severely polluted',advice:'Ventilate thoroughly and reduce pollutant sources.',color:'#df101b'};return{range:'351–500',category:'Extremely polluted',advice:'Ventilate immediately and investigate pollutant sources.',color:'#8b0000'}}function iaqCalibrationText(n){if(![2,7].includes(Number(n?.sensorType)))return'No BME680/BME688 reading available yet.';const accuracy=Number(n.iaqAccuracy||0),elapsed=Number(n.iaqCalibrationElapsedMinutes||0),normalOperation=Number(n.iaqCalibrationPhase||0)>=3,flags=Number(n.telemetryFlags||0),rawFallback=Boolean(flags&32),elapsedText=elapsed===1?'1 minute':`${elapsed} minutes`,source=rawFallback?'Direct Bosch raw reading; the last valid BSEC IAQ is retained when available':'Bosch BSEC Static IAQ',state=rawFallback?'No fresh IAQ was available in this cycle; BSEC retries at the next 5-minute measurement':!normalOperation?`Learning has run for ${elapsedText} and ends automatically when BSEC accuracy first reaches 1; its duration depends on changing air conditions`:'Normal IAQ operation is active; BSEC continues refining the baseline in the background',radio=Math.max(1,Math.round(Number(n.sleepSeconds||300)/60));return `Accuracy ${iaqAccuracyText(accuracy)} · ${state}. Source: ${source}. Internal ULP measurement every 5 min; radio and ThingSpeak strictly every ${radio} min.`}
function sensorModeInfo(n){const mode=Number(n?.operatingMode),type=Number(n?.sensorType);if(mode===6)return{label:'Battery protection',css:'energy',detail:'Sensors off; deep sleep with battery report once every 24 hours'};if(mode===5&&type===8)return{label:'Continuous gas sensing',css:'recovery',detail:'Sensor rail stays powered for MOX heater warm-up and continuous eCO2/TVOC measurement'};if(mode===5)return{label:'Live precision measurement',css:'recovery',detail:type===5?'Active optical recording (25 Hz waveform / 1 s pulse); sensor and CPU stay powered':'Active temperature recording (1 s cycle); sensor and CPU stay powered'};if(mode===4)return{label:'Continuous motion',css:'recovery',detail:'Active 104 Hz FIFO recording; sensor and CPU stay powered'};if(mode===1)return{label:'Setup mode',css:'setup',detail:'Broadcast discovery on all 13 channels'};if(mode===2&&type===5)return{label:'Standby (light sleep)',css:'energy',detail:'Sensor rail off; light sleep with GPIO9 (SW2) wake and 5-second station check-in'};if(mode===2&&[3,4].includes(type))return{label:'Interval measurement',css:'energy',detail:'Sensor rail off between intervals; light sleep with GPIO9 (SW2) wake'};if(mode===2)return{label:'Energy-saving mode',css:'energy',detail:'Sensor rail off, ESP32-C3 deep sleep and unicast on the saved channel'};if(mode===3)return{label:'Channel recovery',css:'recovery',detail:'Saved channel failed; searching for the station'};return{label:'Mode unknown',css:'unknown',detail:'Waiting for a current sensor report'}}function sensorModeBadge(n){const mode=sensorModeInfo(n);return `<span class="modeBadge ${mode.css}" title="${mode.detail}">${mode.label}</span>`}
function intervalOptions(value){const labels={1:'1 second',2:'2 seconds',5:'5 seconds',10:'10 seconds',30:'30 seconds',60:'1 minute',300:'5 minutes',600:'10 minutes',900:'15 minutes',1800:'30 minutes',3600:'60 minutes'};return intervalPresets.map(v=>`<option value="${v}" ${value===v?'selected':''}>${labels[v]}</option>`).join('')+`<option value="custom" ${intervalPresets.includes(value)?'':'selected'}>Custom</option>`}function intervalControl(name,value){const seconds=Math.max(1,Math.min(86400,Math.round(Number(value)||600))),custom=!intervalPresets.includes(seconds);return `<select class="intervalPreset">${intervalOptions(seconds)}</select><div class="customInterval ${custom?'on':''}"><input class="customMinutes" type="number" min="1" max="86400" step="1" value="${seconds}"><span>seconds</span></div><input class="intervalSeconds" type="hidden" name="${name}" value="${seconds}">`}function bindIntervals(root=document){root.querySelectorAll('.intervalControl').forEach(c=>{if(c.dataset.bound)return;const p=c.querySelector('.intervalPreset'),box=c.querySelector('.customInterval'),minutes=c.querySelector('.customMinutes'),hidden=c.querySelector('.intervalSeconds');if(!p||!box||!minutes||!hidden)return;const sync=()=>{const custom=p.value==='custom';box.classList.toggle('on',custom);hidden.value=custom?Math.max(1,Math.min(86400,Math.round(Number(minutes.value)||1))):p.value};p.onchange=sync;minutes.oninput=sync;c.dataset.bound='1';sync()})}
async function apiPost(path,body){
  body.set('csrf',csrf);const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),20000);
  try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,signal:controller.signal});if(r.status===401){location.replace('/login');throw new Error('Authentication required')}const j=await r.json();if(!r.ok||!j.success)throw new Error(j.message||'Action failed');return j}
  catch(e){if(e.name==='AbortError')throw new Error('The station did not confirm the action in time. Check its current status before retrying.');throw e}
  finally{clearTimeout(timer)}
}function notify(text,bad=false){$('toast').textContent=text;$('toast').className='toast on'+(bad?' bad':'');clearTimeout(window.toastTimer);window.toastTimer=setTimeout(()=>$('toast').className='toast',4000)}
async function loadConfig(){config=await requestJson('/api/config');renderStorageStatus(config.storageAvailable);csrf=config.csrfToken;$('defaultInterval').innerHTML=intervalControl('defaultSleep',config.defaultSleepSeconds);bindIntervals();$('fallbackChannel').value=config.fallbackChannel||1;$('userApiKey').value=config.userApiKey||'';$('apName').textContent=config.apSsid||'W-Charger';$('apState').textContent=config.apActive?`Active · ${config.apIp||'192.168.4.1'}`:'Disabled · available only during initial setup or network recovery';$('apPassword').value=config.accessPointPassword||'';const protectedAccess=Boolean(config.adminPasswordSet),state=$('passwordState'),banner=$('securityBanner');state.className='securityStatus'+(protectedAccess?' protected':'');state.textContent=protectedAccess?'Password protection is active':'No website password is set';banner.style.display='none';$('logoutLink').style.display=protectedAccess?'inline-flex':'none';$('newAdminPassword').value='';$('newAdminPassword').disabled=false;$('confirmAdminPasswordSettings').value='';$('confirmAdminPasswordSettings').disabled=false;$('removeAdminPassword').checked=false;$('removeAdminPassword').disabled=!protectedAccess;setSsidOptions([{ssid:config.wifiSsid}],config.wifiSsid);renderChannels()}
function renderChannels(){const profiles=config.thingSpeakChannels||[];$('channelGrid').innerHTML=profiles.map(p=>`<form class="channelCard" data-api="/api/channel-profile"><input type="hidden" name="slot" value="${p.slot}"><div class="channelTitle"><h3>${esc(p.name)}</h3><span class="fieldBadge">ID ${p.channelId}</span></div><label>Name</label><input name="name" maxlength="24" value="${esc(p.name)}"><label>Channel ID</label><input name="channelId" type="number" min="1" value="${p.channelId}"><div class="row"><div><label>Read API Key</label><input class="keyText" name="readApiKey" type="text" maxlength="32" value="${esc(p.readApiKey)}"></div><div><label>Write API Key</label><input class="keyText" name="writeApiKey" type="text" maxlength="32" value="${esc(p.writeApiKey)}"></div></div><div class="actionRow"><button class="btn" type="submit">Save</button><button class="btn danger deleteProfile" type="button" data-slot="${p.slot}">Remove</button></div></form>`).join('')+(profiles.length<6?`<form class="channelCard" data-api="/api/channel-profile"><div class="channelTitle"><h3>Add new channel</h3></div><label>Name</label><input name="name" maxlength="24" placeholder="e.g. Upstairs"><label>Channel ID</label><input name="channelId" type="number" min="1" required><div class="row"><div><label>Read API Key</label><input class="keyText" name="readApiKey" type="text" maxlength="32"></div><div><label>Write API Key</label><input class="keyText" name="writeApiKey" type="text" maxlength="32"></div></div><button class="btn" type="submit" style="margin-top:15px">Add channel</button></form>`:'');bindForms();document.querySelectorAll('.deleteProfile').forEach(b=>b.onclick=()=>deleteProfile(Number(b.dataset.slot)))}
function remoteById(id){return remoteChannels.find(c=>c.id===Number(id))}function remoteFieldInputs(c){return(c.fields||Array(8).fill('')).map((v,i)=>`<div><label>Field ${i+1}</label><input name="field${i+1}" maxlength="80" value="${esc(v)}"></div>`).join('')}
function renderRemoteChannels(){const grid=$('remoteChannelGrid');if(!remoteChannels.length){grid.innerHTML='<div class="card empty"><div class="pulse">☁</div><h3>No account channels loaded</h3><p class="muted">Check the User API Key or create your first channel.</p></div>';return}grid.innerHTML=remoteChannels.map(c=>`<details class="remoteCard"><summary><div class="remoteSummary"><div><h3>${esc(c.name)}</h3><code>Channel ${c.id}</code></div><span class="fieldBadge">${c.publicFlag?'Public':'Private'}</span></div><div class="remoteMeta"><span class="channelTag">${c.localSlot>=0?'✓ Sensor profile':'Not stored locally'}</span><span class="channelTag">Latest entry ${c.lastEntryId||'–'}</span></div><div class="urlBox"><code>${esc(c.channelUrl)}</code><button type="button" class="btn secondary copyRemote" data-url="${esc(c.channelUrl)}">Copy</button></div><div class="urlBox"><code>${esc(c.feedUrl)}</code><button type="button" class="btn secondary copyRemote" data-url="${esc(c.feedUrl)}">API URL</button></div></summary><form class="remoteForm remoteEdit"><input type="hidden" name="channelId" value="${c.id}"><label>Name</label><input name="name" maxlength="80" required value="${esc(c.name)}"><label>Description</label><input name="description" maxlength="255" value="${esc(c.description)}"><div class="fieldNames">${remoteFieldInputs(c)}</div><div class="row"><div><label>Tags</label><input name="tags" maxlength="160" value="${esc(c.tags)}"></div><div><label>External URL</label><input name="url" maxlength="200" value="${esc(c.url)}"></div></div><div class="fieldNames"><div><label>Latitude</label><input name="latitude" type="number" min="-90" max="90" step="any" value="${esc(c.latitude)}"></div><div><label>Longitude</label><input name="longitude" type="number" min="-180" max="180" step="any" value="${esc(c.longitude)}"></div><div><label>Elevation in metres</label><input name="elevation" type="number" min="-10000" max="100000" step="any" value="${esc(c.elevation)}"></div></div><label>Metadata</label><input name="metadata" maxlength="512" value="${esc(c.metadata)}"><label><input name="publicFlag" type="checkbox" style="width:auto" ${c.publicFlag?'checked':''}> Show channel publicly</label><div class="row"><div><label>Read API Key</label><input class="keyText" type="text" readonly value="${esc(c.readApiKey)}"></div><div><label>Write API Key</label><input class="keyText" type="text" readonly value="${esc(c.writeApiKey)}"></div></div><div class="actionRow"><button class="btn" type="submit">Save cloud settings</button><button class="btn secondary importRemote" type="button" data-id="${c.id}">${c.localSlot>=0?'Update local profile':'Use for sensors'}</button><a class="btn secondary" target="_blank" rel="noopener" href="${esc(c.channelUrl)}">Open in ThingSpeak ↗</a></div><div class="dangerZone"><button class="btn danger clearRemote" type="button" data-id="${c.id}">Clear all measurements</button><button class="btn danger deleteRemote" type="button" data-id="${c.id}">Permanently delete channel</button></div></form></details>`).join('');document.querySelectorAll('.remoteEdit').forEach(f=>f.onsubmit=submitRemoteEdit);document.querySelectorAll('.importRemote').forEach(b=>b.onclick=async()=>{try{await importRemoteChannel(Number(b.dataset.id),false)}catch(e){notify(e.message,true)}});document.querySelectorAll('.clearRemote').forEach(b=>b.onclick=()=>clearRemoteChannel(Number(b.dataset.id)));document.querySelectorAll('.deleteRemote').forEach(b=>b.onclick=()=>deleteRemoteChannel(Number(b.dataset.id)));document.querySelectorAll('.copyRemote').forEach(b=>b.onclick=e=>{e.preventDefault();copyRemoteUrl(b.dataset.url)})}
let pendingChannelList=null;
async function waitForChannelList(){
  const path='/api/thingspeak/channels';
  const {ticket}=await apiPost(path,new URLSearchParams());
  return waitForThingSpeakResult(path,ticket);
}
async function waitForThingSpeakResult(path,ticket){
  if(!Number.isInteger(ticket)||ticket<1||ticket>0xffffffff)throw new Error('Invalid ThingSpeak operation ticket.');
  const deadline=performance.now()+30000;
  for(;;){
    const remaining=deadline-performance.now();
    if(remaining<=0)throw new Error('ThingSpeak is still busy. Reload account channels before retrying.');
    const result=await requestJson(`${path}?ticket=${ticket}`,Math.min(4000,remaining));
    if(!result.pending)return result;
    await new Promise(resolve=>setTimeout(resolve,300));
  }
}
async function thingSpeakMutation(path,body){
  const nonce=crypto.getRandomValues(new Uint32Array(1))[0]||1;
  body.set('requestId',String(nonce));
  const {ticket}=await apiPost(path,body);
  const result=await waitForThingSpeakResult('/api/thingspeak/operation',ticket);
  if(!result.success)throw new Error(result.message||'ThingSpeak action failed.');
  return result;
}
function fetchChannelList(){
  if(!pendingChannelList)pendingChannelList=waitForChannelList().finally(()=>{pendingChannelList=null});
  return pendingChannelList;
}
async function loadRemoteChannels(silent=false){const button=$('loadCloudChannels'),message=$('cloudAccountMessage');if(!config.userApiKey){remoteChannels=[];renderRemoteChannels();message.textContent='Save your ThingSpeak User API Key first.';return}if(button){button.disabled=true;button.textContent='Loading channels …'}if(!silent)message.textContent='Connecting to ThingSpeak …';try{const j=await fetchChannelList();if(!j.success)throw new Error(j.message||'Channels could not be loaded');remoteChannels=j.channels||[];renderRemoteChannels();message.textContent=`${remoteChannels.length} ThingSpeak channel${remoteChannels.length===1?'':'s'} loaded. Select a channel to edit its settings.`;if($('sensorModal').classList.contains('on')){const selected=$('wizardProfile').value;$('wizardProfile').innerHTML=channelOptions(selected);updateProfileHint()}}catch(e){remoteChannels=[];renderRemoteChannels();message.textContent=e.message;if(!silent)notify(e.message,true)}if(button){button.disabled=false;button.textContent='↻ Load channels'}}
async function importRemoteChannel(id,selectAfter=true){const c=remoteById(id);if(!c)throw new Error('Cloud channel not found.');let local=(config.thingSpeakChannels||[]).find(p=>p.channelId===c.id);if(!local){const j=await apiPost('/api/channel-profile',new URLSearchParams({name:(c.name||`Channel ${c.id}`).slice(0,24),channelId:String(c.id),readApiKey:c.readApiKey||'',writeApiKey:c.writeApiKey||''}));notify(j.message);await loadConfig();local=(config.thingSpeakChannels||[]).find(p=>p.channelId===c.id)}else{const j=await apiPost('/api/channel-profile',new URLSearchParams({slot:String(local.slot),name:(c.name||`Channel ${c.id}`).slice(0,24),channelId:String(c.id),readApiKey:c.readApiKey||local.readApiKey||'',writeApiKey:c.writeApiKey||local.writeApiKey||''}));notify(j.message);await loadConfig();local=(config.thingSpeakChannels||[]).find(p=>p.channelId===c.id)}if(!local)throw new Error('The local channel profile could not be created.');c.localSlot=local.slot;if(selectAfter&&$('sensorModal').classList.contains('on')){$('wizardProfile').innerHTML=channelOptions(local.slot);updateProfileHint()}renderRemoteChannels();return local.slot}
async function submitRemoteEdit(e){e.preventDefault();const f=e.currentTarget,b=f.querySelector('[type=submit]');b.disabled=true;try{const j=await thingSpeakMutation('/api/thingspeak/channel-update',new URLSearchParams(new FormData(f)));notify(j.message);await loadConfig();await loadRemoteChannels(true);await loadStatus()}catch(err){notify(err.message,true)}b.disabled=false}
async function clearRemoteChannel(id){const c=remoteById(id);if(!confirm(`Really clear ALL measurements from “${c?c.name:id}”? The channel and its settings will remain.`))return;try{const j=await thingSpeakMutation('/api/thingspeak/channel-clear',new URLSearchParams({channelId:String(id)}));notify(j.message);await loadRemoteChannels(true)}catch(e){notify(e.message,true)}}
async function deleteRemoteChannel(id){const c=remoteById(id),name=c?c.name:String(id);if(!confirm(`Permanently delete channel “${name}” from ThingSpeak? All measurements and local sensor assignments will be lost.`))return;if(prompt(`Enter the channel name to confirm:\n${name}`)!==name){notify('Deletion cancelled: the name does not match.',true);return}try{const j=await thingSpeakMutation('/api/thingspeak/channel-delete',new URLSearchParams({channelId:String(id)}));notify(j.message);await loadConfig();await loadRemoteChannels(true);await loadStatus()}catch(e){notify(e.message,true)}}
async function createRemoteChannel(e){e.preventDefault();const f=e.currentTarget,b=f.querySelector('[type=submit]');b.disabled=true;try{const j=await thingSpeakMutation('/api/thingspeak/channel-create',new URLSearchParams(new FormData(f)));notify(j.message);f.reset();await loadConfig();await loadRemoteChannels(true)}catch(err){notify(err.message,true)}b.disabled=false}
async function copyRemoteUrl(url){try{await navigator.clipboard.writeText(url);notify('Channel URL copied.')}catch(e){prompt('Copy channel URL:',url)}}
async function deleteProfile(slot){if(!confirm('Remove this channel profile? Assigned sensor uploads will be disabled.'))return;try{const j=await apiPost('/api/delete-channel-profile',new URLSearchParams({slot:String(slot)}));notify(j.message);await loadConfig();await loadStatus()}catch(e){notify(e.message,true)}}
async function syncLegacyHistories(sensors,serial=historyLoadSerial){await Promise.all(sensors.map(async n=>{const revision=Number(n.live?n.liveRevision:n.historyRevision||0);if(n.live){if(n.recording?.session&&feedbackSessions[n.mac]!==n.recording.session){feedbackSessions[n.mac]=n.recording.session;stationHistory[n.mac]=[];delete historyRevisions[n.mac];delete recoveryMeasurements[n.mac]}if(historyRevisions[n.mac]===revision&&stationHistory[n.mac]){stationHistory[n.mac]=stationHistory[n.mac].filter(p=>p.receivedT>=Date.now()-60000);return}try{const cutoff=Date.now()-60000,previous=(stationHistory[n.mac]||[]).filter(p=>(p.sensorType==null||p.sensorType===n.sensorType)&&p.receivedT>=cutoff),last=previous.at(-1),url='/api/history?mac='+encodeURIComponent(n.mac)+'&source=live';let data=await requestJson(url+(last?.id!=null?'&afterMs='+last.id:'')),now=Date.now(),base=previous;if(last?.id!=null&&!(data.points||[]).length&&(n.recording?.state===2||[4,5].includes(Number(n.operatingMode)))&&Number(n.ageMs)<5000){data=await requestJson(url);now=Date.now();base=[];}if(serial!==historyLoadSerial||selectedSensorMac!==n.mac)return;stationHistory[n.mac]=[...base,...(data.points||[]).map(p=>({...p,t:now-Number(p.measurementAgeMs??p.ageMs),receivedT:now-Number(p.ageMs),estimateT:now-Number(p.estimateAgeMs??p.ageMs),wave:(p.wave||[]).map(w=>({...w,t:now-Number(w.ageMs)}))}))].filter(p=>p.receivedT>=now-60000).slice(-1201);historyBuckets[n.mac]=Number(data.bucketSeconds);historyRevisions[n.mac]=revision}catch(e){if(!stationHistory[n.mac])stationHistory[n.mac]=[]}return;}if(stationHistory[n.mac]&&historyRevisions[n.mac]===revision)return;try{const previous=stationHistory[n.mac]||[],incremental=Number(n.sleepSeconds)<60&&historyBuckets[n.mac]===Number(n.sleepSeconds)&&previous.length&&revision>=Number(historyRevisions[n.mac]||0),since=incremental?Math.floor(previous[previous.length-1].t/1000):0,data=await requestJson('/api/history?mac='+encodeURIComponent(n.mac)+'&since='+since);if(serial!==historyLoadSerial||selectedSensorMac!==n.mac)return;stationHistory[n.mac]=incremental?[...new Map([...previous,...(data.points||[])].map(p=>[p.t,p])).values()].sort((a,b)=>a.t-b.t).slice(-900):data.points||[];historyBuckets[n.mac]=Number(data.bucketSeconds||n.sleepSeconds||1800);historyRevisions[n.mac]=revision}catch(e){if(!stationHistory[n.mac])stationHistory[n.mac]=[]}}))}
let dashboardRefreshSeconds=1,selectUpdateDeferred=false;function setDashboardRefresh(value){dashboardRefreshSeconds=Number(value);window._scheduleDashboardPoll?.()}
async function loadStatus(initialStatus=null){if(statusRequestActive&&Date.now()-statusRequestStartedAt<15000)return;if(recordingTransfers)return;statusRequestActive=true;statusRequestStartedAt=Date.now();try{const s=initialStatus?await initialStatus:await requestJson('/api/status?ts='+Date.now());statusFailureCount=0;if(!csrf||Number(s.uptimeMs)<Number(statusData.uptimeMs)){if(Number(s.uptimeMs)<Number(statusData.uptimeMs)){stationHistory={};historyRevisions={};historyBuckets={};for(const mac of Object.keys(historyPartial)){delete historyPartial[mac];delete historyRanges[mac]}}loadConfig().catch(()=>{})}for(const n of s.sensors||[]){const old=(statusData.sensors||[]).find(x=>x.mac===n.mac);if(old&&(Number(old.sensorType)!==Number(n.sensorType)||Number(old.configuredSensorType)!==Number(n.configuredSensorType))){delete stationHistory[n.mac];delete historyRevisions[n.mac];delete historyBuckets[n.mac];delete historyPartial[n.mac];delete historyRanges[n.mac];delete chartMetrics[n.mac];delete chartEnds[n.mac];delete feedbackSessions[n.mac];delete recoveryMeasurements[n.mac];const v=recordingViews[n.mac];if(v){v.controller?.abort();v.request=(v.request||0)+1;v.selected='';v.points=[];v.busy=false;v.error='';v.listChecked=0;}}}statusData=s;const refresh=$('dashboardRefresh');if(refresh){const live=(s.sensors||[]).some(n=>n.live&&n.provisioned);refresh.disabled=live;if(live){refresh.innerHTML=`<option>Automatic · ${livePollMs()} ms</option>`}else if(refresh.options.length===1){refresh.innerHTML=[1,2,5,10].map(v=>`<option value="${v}" ${v===dashboardRefreshSeconds?'selected':''}>${v} second${v===1?'':'s'}</option>`).join('')}}await syncHistories(s.sensors||[]);$('connection').classList.toggle('ok',s.wifi.connected);$('connection').querySelector('span').textContent=s.wifi.connected?`Wi-Fi connected · ${s.wifi.ip}`:(s.wifi.apActive?'Setup Wi-Fi active':'Wi-Fi offline');$('currentWifi').textContent=s.wifi.connected?`Connected to ${s.wifi.ssid} · ${s.wifi.ip} · radio channel ${s.wifi.channel}`:`Not connected · saved: ${config.wifiSsid||'no home Wi-Fi'}`;const configured=(s.sensors||[]).filter(n=>n.provisioned),pending=(s.sensors||[]).filter(n=>!n.provisioned);$('sensorCount').textContent=configured.length;$('onlineCount').textContent=configured.filter(n=>n.online).length;$('pendingCount').textContent=pending.length;renderStorageStatus(s.storageAvailable);renderCloud(s.cloud||{});renderPending(pending);syncRecordingLists(configured.filter(n=>n.mac===selectedSensorMac));const renderSignature=JSON.stringify(configured.map(n=>[n.mac,n.name,n.sensorType,n.configuredSensorType,n.configPending,n.motionReference,n.online,n.revision,n.appliedRevision,n.temperature,n.humidity,n.pressure,n.iaq,n.iaqAccuracy,n.gasResistanceOhms,n.batteryMv,n.telemetryFlags,n.operatingMode,n.sleepSeconds,n.reportIntervalMs,n.historyRevision,n.history?.points,n.history?.pendingPoints,n.ax,n.ay,n.az,n.gx,n.gy,n.gz,n.peakAcceleration,n.peakAngularRate,n.motionOverrun,n.liveRevision,n.heartRate,n.pulseStatus,n.pulseQuality,n.eco2,n.tvoc,n.ccsStatus,n.ccsError,n.distanceMm,n.signalRateKcps,n.ambientRateKcps,n.rangeStatus,n.spectral,n.spectralGain,n.spectralStatus,n.recording,n.seen?(n.live?Math.floor(Date.now()/Math.max(1000,livePollMs())):age(n.ageMs)):0]));if(!$('sensorModal').classList.contains('on')&&renderSignature!==sensorRenderSignature){sensorRenderSignature=renderSignature;renderSensors(configured)}}catch(e){statusFailureCount++;$('connection').classList.remove('ok');$('connection').querySelector('span').textContent='Station unreachable';$('currentWifi').textContent='Connection status could not be loaded.';$('cloudMessage').style.display='flex';$('cloudMessage').className='cloudMessage bad';$('cloudMessage').innerHTML='<div>!</div><div><b>Station unreachable</b><small>Status could not be loaded. Retrying automatically.</small></div>'}finally{statusRequestActive=false;statusRequestStartedAt=0}}
function renderStorageStatus(available){for(const id of ['storageMessage','settingsStorageMessage']){const box=$(id);box.style.display=available===false?'flex':'none';if(available===false)box.innerHTML='<div>!</div><div><b>Local storage unavailable</b><small>Existing files were preserved. New recordings and persistent history cannot be saved. Settings and live measurements remain available. Export accessible data and check the station storage before resetting it.</small></div>'}}
function renderCloud(c){
  const box=$('cloudMessage'),nodes=(statusData.sensors||[]).filter(n=>n.provisioned);
  box.style.display='none';
  if(!c.uploadEnabledSensors){$('cloudState').textContent=nodes.length&&nodes.every(n=>n.live)?'Not used':config.userApiKey?'Account ready':'Not configured';return;}
  const failed=c.lastUploadFailed!=null?Boolean(c.lastUploadFailed):(c.failedUploads>0&&(!c.lastSuccessAgeMs||c.lastAttemptAgeMs<c.lastSuccessAgeMs));
  $('cloudState').textContent=failed?'Error':c.lastEntryId>0?'Connected':'Waiting';
  if(!failed)return;
  box.style.display='flex';box.className='cloudMessage bad';
  const detail=c.lastHttpStatus===200?'ThingSpeak returned no entry ID. Check the channel and Write API Key.':c.lastHttpStatus===-2?'None of the assigned measurements were valid or available for this cycle.':c.lastHttpStatus===-1?'The secure ThingSpeak connection could not be established.':c.lastHttpStatus===-3?'A queued upload was cancelled after the sensor settings changed. New readings use the current settings.':`HTTP status ${c.lastHttpStatus}. Check Wi-Fi, channel and API key.`;
  box.innerHTML=`<div>!</div><div><b>Last upload failed</b><small>${detail} Failed uploads since restart: ${c.failedUploads}.</small></div>`;
}
function renderPending(list){if(list.length)$('pendingPanel').classList.add('on');$('pendingList').innerHTML=list.length?list.map(n=>`<div class="pendingItem"><div><b>${esc(n.name)}</b> ${sensorModeBadge(n)}<code>${esc(n.mac)} · PCB V${n.pcbVersion} · ${sensorTypeName(n.sensorType)} · ${n.seen?age(n.ageMs):'no measurement yet'}</code></div><button class="btn" onclick="openSensorWizard('${esc(n.mac)}')">Configure now</button></div>`).join(''):'<div class="empty"><div class="pulse">⌁</div><h3>Listening for new sensors</h3><p class="muted">Keep this panel open. New sensors check in every ten seconds during their first ten minutes and will appear automatically.</p></div>';$('addSensorButton').textContent=list.length?`+ ${list.length} sensor${list.length===1?'':'s'} to configure`:'+ Find sensors'}
function sensorDomKey(node){return [node.nodeType,node.nodeName,node.nodeType===1?node.getAttribute('data-ui-key')||node.id||node.getAttribute('aria-label')||node.getAttribute('data-key')||'':''].join(':')}
function patchSensorDom(current,next){
  if(current.nodeType!==next.nodeType||current.nodeName!==next.nodeName){current.replaceWith(next.cloneNode(true));return}
  if(current.nodeType!==1){if(current.nodeValue!==next.nodeValue)current.nodeValue=next.nodeValue;return}
  // Keep native menus, keyboard focus and chart cursors alive during telemetry updates.
  if(current.matches('select')&&document.activeElement===current){selectUpdateDeferred=true;return}
  // A loaded archive is immutable until the user changes its view or reloads.
  if(current.classList.contains('recordingCharts')&&current.hasChildNodes()&&next.dataset.chartVersion&&current.dataset.chartVersion===next.dataset.chartVersion)return;
  if(current.classList.contains('liveCursor')||current.classList.contains('hoverCursor'))return;
  for(const attr of [...current.attributes])if(!(current.matches('details')&&attr.name==='open')&&!next.hasAttribute(attr.name))current.removeAttribute(attr.name);
  for(const attr of [...next.attributes])if(!(current.matches('details')&&attr.name==='open')&&current.getAttribute(attr.name)!==attr.value)current.setAttribute(attr.name,attr.value);
  const old=[...current.childNodes],used=new Set,pools=new Map;let position=current.firstChild;
  for(const child of old){const key=sensorDomKey(child);if(!pools.has(key))pools.set(key,{nodes:[],index:0});pools.get(key).nodes.push(child)}
  for(const child of [...next.childNodes]){
    const pool=pools.get(sensorDomKey(child)),match=pool?.nodes[pool.index++];
    if(match){used.add(match);if(match!==position)current.insertBefore(match,position);patchSensorDom(match,child);position=match.nextSibling}
    else{const added=child.cloneNode(true);current.insertBefore(added,position);position=added.nextSibling}
  }
  for(const child of old)if(!used.has(child))child.remove();
  if(current.matches('select'))current.value=next.value;
}
function updateSensorGrid(html){
  const grid=$('sensorGrid'),cursors=[...grid.querySelectorAll('svg[data-cursor-time]')].map(svg=>({svg,time:Number(svg.dataset.cursorTime),active:svg._cursorPinned||svg.querySelector('.liveCursor,.hoverCursor')?.style.display!=='none',snapshot:svg._cursorSnapshot})),next=document.createElement('div');
  for(const attr of grid.attributes)next.setAttribute(attr.name,attr.value);next.innerHTML=html;patchSensorDom(grid,next);
  for(const {svg,time,active,snapshot} of cursors)if(active&&svg.isConnected&&Number.isFinite(time)){
    const points=svg.dataset.series?chartData(svg,'series').flatMap(s=>s.points):chartData(svg,'samples');
    if(svg._cursorPinned&&snapshot&&!points.some(p=>p.t===time)){
      svg.dataset.cursorTime=time;svg.querySelector('.liveCursor,.hoverCursor').style.display='none';
      chartReadout(svg,time,snapshot.values,true);continue;
    }
    const box=svg.getBoundingClientRect();
    if(!(box.width>0))continue;
    const padLeft=Number(svg.dataset.padLeft||64),plotW=Number(svg.dataset.plotWidth||520);
    const x=padLeft+(time-Number(svg.dataset.start))*plotW/Number(svg.dataset.window);
    (svg.classList.contains('liveSvg')?liveHover:chartMove)({currentTarget:svg,clientX:box.left+x/600*box.width});
  }
  fitChartLabels();
}
const observedCharts=new Set(),chartLabelObserver=typeof ResizeObserver==='function'?new ResizeObserver(entries=>{for(const {target} of entries)fitChartLabel(target)}):null;
function fitChartLabel(svg){const width=svg.getBoundingClientRect().width;if(width>0)svg.style.setProperty('--chart-label-size',(12*600/width).toFixed(2)+'px')}
function fitChartLabels(){
  for(const svg of observedCharts)if(!svg.isConnected){chartLabelObserver?.unobserve(svg);observedCharts.delete(svg)}
  for(const svg of document.querySelectorAll('.historySvg,.liveSvg')){fitChartLabel(svg);if(chartLabelObserver&&!observedCharts.has(svg)){observedCharts.add(svg);chartLabelObserver.observe(svg)}}
}
$('sensorGrid').addEventListener('focusout',()=>{if(!selectUpdateDeferred)return;selectUpdateDeferred=false;setTimeout(()=>redrawRecordings(),0)});
function tileActiveClass(mac,key){return selectedSensorMac===mac&&chartMetrics[mac]===key&&!recordingViews[mac]?.selected?' active':''}
function renderFullSensors(list){
if(!list.length){$('sensorGrid').innerHTML='<div class="card empty"><div class="pulse">⌁</div><h3>No sensor configured yet</h3><p class="muted">Detected devices can be configured with “+ Find sensors”.</p></div>';fitChartLabels();return}
const html=list.map(source=>{
const history=getHistory(source.mac),last=history.length?history[history.length-1]:null,restored=!source.seen&&(Boolean(source.restored)||Boolean(last)),restoredCaps=Number(source.capabilities||0)|(last?.eco2!=null?4096:0)|(last?.distanceMm!=null?2048:0)|(Array.isArray(last?.spectral)?1024:0)|(last?.battery!=null?16:0),base=!source.seen&&last?{...source,sensorType:Number(last.sensorType??source.configuredSensorType??source.sensorType),pcbVersion:Number(last.pcbVersion??source.pcbVersion),capabilities:restoredCaps,temperature:last.temperature,humidity:last.humidity,pressure:last.pressure,iaq:Number(last.iaq??0),iaqAccuracy:Number(last.iaqAccuracy??(last.iaq!==undefined?1:0)),gasResistanceOhms:Number(last.gas??0)*1000,eco2:last.eco2??source.eco2,tvoc:last.tvoc??source.tvoc,distanceMm:last.distanceMm??source.distanceMm,signalRateKcps:last.signalRate??source.signalRateKcps,ambientRateKcps:last.ambientRate??source.ambientRateKcps,rangeStatus:last.rangeStatus??source.rangeStatus,spectral:last.spectral??source.spectral,spectralGain:last.spectralGain??source.spectralGain,spectralIntMs:last.spectralIntMs??source.spectralIntMs,batteryMv:Number(last.battery??0)*1000}:source,n=sensorForDisplay(base);
const p=(config.thingSpeakChannels||[]).find(x=>x.slot===n.profileSlot),battery=n.batteryMv?metric(n.batteryMv/1000,3):'–',gas=n.gasResistanceOhms?metric(n.gasResistanceOhms/1000,1):'–',cloudOk=n.uploadEnabled&&p&&statusData.cloud&&Number(statusData.cloud.lastEntryId)>0&&Number(statusData.cloud.lastHttpStatus)===200,cloud=n.uploadEnabled&&p?`<a class="channelTag ${cloudOk?'ok':''}" target="_blank" rel="noopener" href="https://thingspeak.com/channels/${p.channelId}">${cloudOk?'●':'○'} ${esc(p.name)} · Channel ${p.channelId} ↗</a>`:'',flags=Number(n.telemetryFlags||0),isBoschGas=[2,7].includes(Number(n.sensorType)),iaqValue=isBoschGas&&Number(n.iaqAccuracy)===0?'–':metric(n.iaq,0);
const calText=iaqCalibrationText(n),calibrationReady=isBoschGas&&Number(n.iaqCalibrationPhase||0)>=3,ccsErr=Number(n.sensorType)===8?Number(n.ccsError)||0:0,ccsMaxResOnly=ccsErr===8,ccsErrText=ccsErr?` CCS811 ERROR_ID 0x${ccsErr.toString(16).toUpperCase().padStart(2,'0')} (${ccs811ErrorLabel(ccsErr)}).`:'',health=flags&256?'<div class="cloudMessage warn" style="margin:10px 0 0"><div>!</div><div><b>Battery protection active</b><small>Sensors are off. Battery is checked once every 24 hours. Recharge to resume.</small></div></div>':n.sensorInitializing?'':flags&8?'<div class="cloudMessage bad" style="margin:10px 0 0"><div>!</div><div><b>Configured sensor type does not match</b><small>Check I²C wiring and selected sensor type in Edit.</small></div></div>':(flags&1)&&ccsMaxResOnly?'<div class="cloudMessage warn" style="margin:10px 0 0"><div>○</div><div><b>CCS811 resistance above range (MAX_RESISTANCE)</b><small>I²C is responding and the MOX heater stays powered. Allow up to 20 min warm-up (48 h burn-in on new units); if persistent, the MOX sensing element may be degraded or open-circuit.</small></div></div>':(flags&1)&&!(Number(n.sensorType)===5&&n.recording&&n.recording.state!==2)?`<div class="cloudMessage bad" style="margin:10px 0 0"><div>!</div><div><b>Environmental sensor could not be read</b><small>${ccsErrText?esc(ccsErrText)+' ':''}Check power, SDA/SCL and I²C address (AS7341: 0x39 · VL53L1X: 0x29 · CCS811: 0x5A/0x5B · MAX30102: 0x57 · TMP117: 0x48–0x4B · LSM6DSOX: 0x6A/0x6B · Bosch: 0x76/0x77).</small></div></div>`:flags&32?`<div class="cloudMessage warn" style="margin:10px 0 0"><div>○</div><div style="flex:1"><b>${sensorTypeName(n.sensorType)} direct measurement active</b><small>${calText}</small></div></div>`:flags&4&&!calibrationReady?`<div class="cloudMessage" style="margin:10px 0 0"><div>●</div><div style="flex:1"><b>${sensorTypeName(n.sensorType)} is learning its IAQ baseline</b><small>${calText}</small></div></div>`:'';
const accuracy=calibrationReady?`<span class="accuracyBadge">IAQ accuracy ${esc(iaqAccuracyText(n.iaqAccuracy))}</span>`:'',modeBadge=sensorModeBadge(n),contactState=n.seen?age(n.ageMs):restored?'Last saved value · waiting for contact':'No measurement yet',intervalMeta=Number(n.sensorType)===5?'Manual (SW2)':`Interval ${historyStepLabel(n.sleepSeconds||1)}`;
return `<article data-ui-key="${esc(n.mac)}" class="card sensor ${[3,4,5].includes(Number(n.sensorType))?'motion':''}"><div class="sensorTop"><div><h3>${esc(n.name)}</h3><div class="sensorIdentity"><code>${esc(n.mac)} · PCB V${n.pcbVersion} · ${sensorTypeName(n.sensorType)}</code>${modeBadge}${accuracy}<small data-firmware="${esc(n.mac)}">${esc(firmwareLabel(n.mac))}</small></div></div><div class="sensorActions"><a class="btn secondary" href="/updates?mac=${esc(n.mac)}">Firmware</a><button type="button" class="btn secondary" onclick="openSensorWizard('${esc(n.mac)}')">Edit</button><i class="${n.online?'fresh':'fresh stale'}" title="${n.online?'Online':'Offline / sleeping'}"></i></div></div><div class="detailMetaBar"><span>${esc(contactState)} · ${esc(intervalMeta)}</span><span>${n.seen&&n.rssi!=null&&Number.isFinite(Number(n.rssi))?`RSSI ${n.rssi} dBm`:'RSSI –'}</span></div>${n.detectedSensorType!=null?`<div class="helper" role="status">Selected: ${sensorTypeName(n.sensorType)} · last detected: ${sensorTypeName(n.detectedSensorType)}. ${n.configPending?'Waiting for the node to stop the previous driver and initialize the selected sensor at its next contact.':'The selected sensor did not respond. Check its connection and power.'}</div>`:n.configPending?'<div class="helper" role="status">Settings queued · will apply at next sensor contact.</div>':''}${health}${n.sensorType===3?motionReferenceControls(n):''}${n.motionOverrun?'<div class="helper">FIFO overflow: this interval contains a recording gap.</div>':''}${n.live&&!n.sensorInitializing?liveTiming(n):''}<div class="values">${n.sensorType===3?motionTiles(n):n.sensorType===5?pulseTiles(n):n.sensorType===8?ccs811Tiles(n):n.sensorType===9?tofTiles(n):n.sensorType===10?spectralTiles(n):''}${valueTile(n,'Temperature',metric(n.temperature,n.sensorType===4?3:1),'°C','temperatureField')}${valueTile(n,'Humidity',metric(n.humidity),'%','humidityField')}${valueTile(n,'Pressure',metric(n.pressure,0),'hPa','pressureField')}${isBoschGas?iaqTile(n,iaqValue):''}${isBoschGas?valueTile(n,'Gas resistance',gas,'kΩ','gasResistanceField'):''}${valueTile(n,'Battery',battery,'V','batteryField')}</div>${n.sensorType===10?spectralSpectrumCard(n):''}<div class="sensorFoot">${cloud?`<div data-ui-key="cloud-tag"><b>ThingSpeak</b> ${cloud}</div>`:''}${chartPanel(n,history)}</div></article>`
}).join('');updateSensorGrid(decorateSensorDetail(html))}
function valueTile(n,label,value,unit,key){if((Number(n.telemetryFlags)&256)&&key!=='batteryField')return '';if((key==='batteryField'&&(Number(n.telemetryFlags)&2))||(key!=='batteryField'&&(Number(n.telemetryFlags)&1)))value='–';const t=Number(n.sensorType);if([3,5,8,9,10].includes(t)&&key!=='batteryField')return '';if(t===6&&key==='humidityField')return '';if(t===4&&!['temperatureField','batteryField'].includes(key))return '';if(t===255&&key!=='batteryField')return '';const field=n[key],selectedMetric={temperatureField:'temperature',humidityField:'humidity',pressureField:'pressure',iaqField:'iaq',gasResistanceField:'gas',batteryField:'battery'}[key],badge=n.uploadEnabled&&Number(field)?`<span class="fieldBadge">${fieldName(field)}</span>`:'';return `<button type="button" class="value${tileActiveClass(n.mac,selectedMetric)}" onclick="openMetricChart('${esc(n.mac)}','${selectedMetric}')"><span>${label}</span><b>${value}${unit?` <small>${unit}</small>`:''}</b>${badge}</button>`}
function ccs811ErrorLabel(code){const b=Number(code)||0,names=['WRITE_REG_INVALID','READ_REG_INVALID','MEASMODE_INVALID','MAX_RESISTANCE','HEATER_FAULT','HEATER_SUPPLY'],out=[];for(let i=0;i<names.length;i++)if(b&(1<<i))out.push(names[i]);return out.join(', ')||'unknown'}
function eco2Band(v){const x=Number(v);if(!Number.isFinite(x)||x<400)return null;if(x<=800)return{category:'Excellent',color:'#169bd5',advice:'Fresh indoor air.'};if(x<=1200)return{category:'Good',color:'#2db84d',advice:'Normal indoor CO₂.'};if(x<=2000)return{category:'Moderate',color:'#f1df16',ink:'#172019',advice:'Consider ventilating.'};return{category:'High CO₂',color:'#df101b',advice:'Ventilate immediately.'}}
function ccs811Tiles(n){if(Number(n.telemetryFlags)&256)return '';const valid=!(Number(n.telemetryFlags)&1)&&(Number(n.capabilities)&4096)&&n.eco2!=null,band=valid?eco2Band(n.eco2):null;const eco2Html=band?`<button type="button" class="value iaqTile${tileActiveClass(n.mac,'eco2')}" style="--iaq-color:${band.color};--iaq-ink:${band.ink||'#fff'}" onclick="openMetricChart('${esc(n.mac)}','eco2')"><span>eCO₂</span><div class="iaqTop"><b>${metric(n.eco2,0)} <small>ppm</small></b><span class="iaqCategory">${band.category}</span></div><small class="iaqAdvice">${band.advice}</small></button>`:`<button type="button" class="value iaqTile${tileActiveClass(n.mac,'eco2')}" onclick="openMetricChart('${esc(n.mac)}','eco2')"><span>eCO₂</span><b>– <small>ppm</small></b><small class="iaqAdvice">Waiting for CCS811 reading (20 min warm-up).</small></button>`;return eco2Html+`<button type="button" class="value${tileActiveClass(n.mac,'tvoc')}" onclick="openMetricChart('${esc(n.mac)}','tvoc')"><span>TVOC</span><b>${valid?metric(n.tvoc,0):'–'} <small>ppb</small></b></button>`}
function tofStatusLabel(s){return({0:'Valid',1:'Sigma warn',2:'Low signal',4:'Out of range',7:'Wraparound'})[Number(s)]||'No target'}
function tofTiles(n){if(Number(n.telemetryFlags)&256)return '';const valid=!(Number(n.telemetryFlags)&1)&&(Number(n.capabilities)&2048)&&n.distanceMm!=null,mm=Number(n.distanceMm),st=tofStatusLabel(n.rangeStatus),ok=valid&&Number(n.rangeStatus)===0;return `<button type="button" class="value${tileActiveClass(n.mac,'distanceMm')}" onclick="openMetricChart('${esc(n.mac)}','distanceMm')"><span>Distance</span><b>${valid?`${mm} <small>mm</small>`:'–'}</b><span class="tileSub ${ok?'':'mutedTag'}">${valid?`${(mm/1000).toFixed(2)} m · ${st}`:'Unavailable'}</span></button><button type="button" class="value${tileActiveClass(n.mac,'signalRate')}" onclick="openMetricChart('${esc(n.mac)}','signalRate')"><span>Signal rate</span><b>${valid?metric(n.signalRateKcps,0):'–'} <small>kcps</small></b></button><button type="button" class="value${tileActiveClass(n.mac,'ambientRate')}" onclick="openMetricChart('${esc(n.mac)}','ambientRate')"><span>Ambient rate</span><b>${valid?metric(n.ambientRateKcps,0):'–'} <small>kcps</small></b></button>`}
const spectralBands=[{k:'F1',nm:'415 nm',c:'#6d28d9'},{k:'F2',nm:'445 nm',c:'#2563eb'},{k:'F3',nm:'480 nm',c:'#0284c7'},{k:'F4',nm:'515 nm',c:'#0d9488'},{k:'F5',nm:'555 nm',c:'#16a34a'},{k:'F6',nm:'590 nm',c:'#ca8a04'},{k:'F7',nm:'630 nm',c:'#ea580c'},{k:'F8',nm:'680 nm',c:'#dc2626'},{k:'CLR',nm:'Clear',c:'#64748b'},{k:'NIR',nm:'NIR',c:'#7f1d1d'}];
const as7341Gains=['0.5×','1×','2×','4×','8×','16×','32×','64×','128×','256×','512×'];
function spectralTiles(n){if(Number(n.telemetryFlags)&256)return '';const sp=Array.isArray(n.spectral)&&n.spectral.length===10&&!(Number(n.telemetryFlags)&1)?n.spectral:null;let peakIdx=0,peakVal=-1;if(sp)for(let i=0;i<8;i++)if(sp[i]>peakVal){peakVal=sp[i];peakIdx=i}const gain=as7341Gains[Number(n.spectralGain)]||'–',sat=Boolean(sp&&(Number(n.spectralStatus)&1));return `<button type="button" class="value${tileActiveClass(n.mac,'spectralClear')}" onclick="openMetricChart('${esc(n.mac)}','spectralClear')"><span>Peak band</span><b>${sp?spectralBands[peakIdx].nm:'–'}</b><span class="tileSub ${sp?'':'mutedTag'}">${sp?`${spectralBands[peakIdx].k} · ${peakVal} cnt`:'Unavailable'}</span></button><button type="button" class="value${tileActiveClass(n.mac,'spectralClear')}" onclick="openMetricChart('${esc(n.mac)}','spectralClear')"><span>Clear channel</span><b>${sp?sp[8]:'–'} <small>cnt</small></b></button><button type="button" class="value${tileActiveClass(n.mac,'spectralNir')}" onclick="openMetricChart('${esc(n.mac)}','spectralNir')"><span>NIR channel</span><b>${sp?sp[9]:'–'} <small>cnt</small></b></button><button type="button" class="value" onclick="openMetricChart('${esc(n.mac)}','spectralClear')"><span>Gain · Integration</span><b>${sp?`${gain} · ${n.spectralIntMs||50} ms`:'–'}</b><span class="tileSub ${sat?'warnTag':sp?'':'mutedTag'}">${!sp?'Unavailable':sat?'Saturated':'In range'}</span></button>`}
function spectralSpectrumCard(n){const sp=Array.isArray(n.spectral)&&n.spectral.length===10&&!(Number(n.telemetryFlags)&1)?n.spectral:null;if(!sp)return '';const maxV=Math.max(1,...sp),bars=sp.map((v,i)=>{const h=Math.max(3,Math.round(v/maxV*106)),x=18+i*56,y=128-h,b=spectralBands[i];return `<rect x="${x}" y="${y}" width="38" height="${h}" rx="5" fill="${b.c}" opacity=".9"><title>${b.k} (${b.nm}): ${v}</title></rect><text x="${x+19}" y="${Math.max(13,y-5)}" text-anchor="middle" font-size="10" fill="currentColor">${v}</text><text x="${x+19}" y="145" text-anchor="middle" font-size="10.5" font-weight="650" fill="currentColor">${b.k}</text><text x="${x+19}" y="159" text-anchor="middle" font-size="9.5" fill="var(--muted)">${b.nm.replace(' nm','')}</text>`}).join('');return `<div class="livePlot" style="margin-top:10px"><div class="plotHeading"><b>AS7341 10-Channel Optical Spectrum</b><span class="muted">415–680 nm · Clear · NIR</span></div><svg class="spectrumSvg" viewBox="0 0 580 168" role="img" aria-label="10-channel optical spectrum">${bars}</svg></div>`}
function iaqTile(n,value){if(Number(n.telemetryFlags)&256)return '';const field=n.iaqField,badge=n.uploadEnabled&&Number(field)?`<span class="fieldBadge">${fieldName(field)}</span>`:'',valid=!(Number(n.telemetryFlags)&1)&&n.iaq!=null&&Number.isFinite(Number(n.iaq));if(!valid)return `<button type="button" class="value iaqTile${tileActiveClass(n.mac,'iaq')}" onclick="openMetricChart('${esc(n.mac)}','iaq')"><span>Indoor Air Quality</span><b>–</b><small class="iaqAdvice">No valid environmental measurement available.</small>${badge}</button>`;const band=Number(n.iaqAccuracy)>0?iaqBand(n.iaq):null;if(!band)return `<button type="button" class="value iaqTile${tileActiveClass(n.mac,'iaq')}" onclick="openMetricChart('${esc(n.mac)}','iaq')"><span>Indoor Air Quality</span><div class="iaqTop"><b>${value}</b><span class="iaqCategory" style="--iaq-color:#eef1ee;--iaq-ink:var(--muted)">Learning</span></div><small class="iaqAdvice">BSEC is learning room air baseline.</small>${badge}</button>`;return `<button type="button" class="value iaqTile${tileActiveClass(n.mac,'iaq')}" style="--iaq-color:${band.color};--iaq-ink:${band.ink||'#fff'}" onclick="openMetricChart('${esc(n.mac)}','iaq')"><span>Indoor Air Quality</span><div class="iaqTop"><b>${value}</b><span class="iaqCategory">${band.category}</span></div><small class="iaqAdvice">${band.range} · ${band.advice}</small>${badge}</button>`}
function pulseTiles(n){if(Number(n.telemetryFlags)&256)return '';const stopped=n.recording&&Number(n.recording.state)!==2,failed=Number(n.telemetryFlags)&1,text=n.sensorInitializing?'Waiting for sensor initialization':stopped?'Measurement stopped · press SW2 on the sensor to start':failed?'Optical sensor unavailable · check module power and I2C':['No usable infrared contact signal','Settling or poor signal · keep still','Periodic signal detected','Recording gap · reacquiring'][Number(n.pulseStatus)]||'Waiting';const warn=stopped||failed||Number(n.pulseStatus)!==2;return `<div class="pulseStatus ${warn?'warn':''}"><span>${text}</span>${stopped||failed?'':`<small>Correlation ${Number(n.pulseQuality)||0}%</small>`}</div>`+['heartRate','red','infrared'].map(k=>{const d=chartDefinitions[k],valid=!stopped&&!failed&&(k==='heartRate'?(Number(n.capabilities)&512):(Number(n.capabilities)&256));return `<button type="button" class="value${tileActiveClass(n.mac,k)}" onclick="openMetricChart('${esc(n.mac)}','${k}')"><span>${d.label}</span><b>${valid?metric(n[k],d.digits):'–'} <small>${d.unit}</small></b></button>`}).join('')}
function sensorForDisplay(source){
  const selected=Number(source.configuredSensorType),actual=Number(source.sensorType);
  if(selected>0&&selected!==255&&selected!==actual){
    const pending=Boolean(source.configPending);
    return {...source,detectedSensorType:actual,sensorType:selected,sensorInitializing:pending,
      capabilities:Number(source.capabilities)&16,live:[3,4,5].includes(selected),
      telemetryFlags:pending?(Number(source.telemetryFlags)&(2|256)):(Number(source.telemetryFlags)|1),
      temperature:null,humidity:null,pressure:null,iaq:null,iaqAccuracy:0,gasResistanceOhms:null,
      eco2:null,tvoc:null,distanceMm:null,signalRateKcps:null,ambientRateKcps:null,rangeStatus:null,
      spectral:null,spectralGain:null,spectralIntMs:null,spectralStatus:0,
      heartRate:null,red:null,infrared:null,pulseQuality:0,pulseStatus:0,motionOverrun:false,
      recording:null,normalSampling:true};
  }
  return source;
}
function motionValue(n,key,value){const offset=n.motionReference?.enabled&&['ax','ay','az','gx','gy','gz'].includes(key)?Number(n.motionReference[key]||0):0;return Number(value)-offset}
function motionReferenceControls(n){const enabled=n.motionReference?.enabled,ready=n.seen&&(Number(n.capabilities)&128)&&!(Number(n.telemetryFlags)&257);return `<div class="chartToolbar" data-ui-key="motion-reference"><button type="button" class="btn secondary" ${ready?'':'disabled'} onclick="setMotionZero('${esc(n.mac)}',false)">Zero axes here</button>${enabled?`<button type="button" class="btn secondary" onclick="setMotionZero('${esc(n.mac)}',true)">Clear reference</button>`:''}<span class="muted">${enabled?'Axes relative to resting reference (peaks & CSV keep raw values).':'Keep sensor still on a flat surface, then zero axes.'}</span></div>`}
async function setMotionZero(mac,clear){try{const result=await apiPost('/api/motion-reference',new URLSearchParams({mac,clear:clear?'1':'0'}));notify(result.message);sensorRenderSignature='';await loadStatus()}catch(e){notify(e.message,true)}}
function motionTiles(n){if(Number(n.telemetryFlags)&256)return '';return ['ax','ay','az','gx','gy','gz','peakAcceleration','peakAngularRate'].map(k=>{const d=chartDefinitions[k],relative=n.motionReference?.enabled&&k.length===2;return `<button type="button" class="value${tileActiveClass(n.mac,k)}" onclick="openMetricChart('${esc(n.mac)}','${k}')"><span>${d.label}${relative?' · rel':n.motionReference?.enabled?' · raw':''}</span><b>${(Number(n.capabilities)&128)&&!(Number(n.telemetryFlags)&1)&&!n.motionOverrun&&n[k]!=null?metric(motionValue(n,k,n[k]),relative&&k[0]==='g'?2:d.digits):'–'} <small>${d.unit}</small></b></button>`}).join('')}

function getHistory(mac){return stationHistory[mac]||[]}
function livePollMs(){const nodes=(statusData.sensors||[]).filter(n=>n.live&&n.provisioned);return nodes.length?Math.max(250,Math.min(...nodes.map(n=>n.recording?.state===2||[4,5].includes(Number(n.operatingMode))?(Number(n.sensorType)===3?250:Number(n.sensorType)===5?250:1000):Number(n.sensorType)===5?1000:Math.max(1000,Math.min(10000,Number(n.sleepSeconds||1)*1000))))):dashboardRefreshSeconds*1000}
function liveTiming(n){if(n.normalSampling||(n.recording&&n.recording.state!==2)){const normal=[3,4].includes(Number(n.sensorType));return `<div class="helper liveTiming">${normal?`Normal interval: every ${historyStepLabel(n.sleepSeconds||1)} · press SW2 for fast recording.`:'Standby · press SW2 on sensor to start recording and sync.'}</div>`;}const type=Number(n.sensorType),age=Number(n.acquisitionAgeMs),known=n.timingKnown&&age>=0,acquisition=type===3?'104 Hz FIFO · 50–100 ms means':type===4?'8-avg · 1 s cycle':'100 Hz → 25 Hz wave · 200 ms packets',estimate=type===5?(Number(n.pulseStatus)===2?` · pulse age ${Math.max(0,Number(n.estimateAgeMs)||0)} ms`:` · warmup ${Math.max(0,Number(n.warmupMs)||0)/1000} s`):'',measured=Number(n.receivedIntervalMs)>0?` · spacing ${Number(n.receivedIntervalMs)} ms`:'';return `<div class="helper liveTiming">${acquisition}${known?` · age ~${age} ms`:''}${estimate}${measured}${Number(n.processingMs)>0?` · proc ${Number(n.processingMs)} ms`:''}</div>`}
const pulseWindows={};
const recoveryMeasurements={};
function feedbackNumber(v,d=1,unit=''){return Number.isFinite(v)?v.toFixed(d)+(unit?' '+unit:''):'–'}
function feedbackTile(label,value){return `<div class="feedbackValue"><span>${esc(label)}</span><b>${esc(value)}</b></div>`}
function feedbackPoints(history,end){return history.filter(p=>Number.isFinite(p.t)&&p.t>=end-60000&&p.t<=end).sort((a,b)=>a.t-b.t)}
function temperatureFeedback(points,end){
  const recent=points.filter(p=>!p.gap&&p.temperature!=null&&Number.isFinite(Number(p.temperature))&&p.t>=end-10000);
  let continuous=recent.length>=8&&recent.at(-1).t-recent[0].t>=8000&&end-recent.at(-1).t<=1800;
  for(let i=1;i<recent.length;i++)if(recent[i].t-recent[i-1].t>1800)continuous=false;
  if(points.some(p=>p.gap&&p.t>=end-10000))continuous=false;
  if(!continuous)return {stable:false,mean:NaN,slope:NaN};
  const mean=recent.reduce((s,p)=>s+Number(p.temperature),0)/recent.length;
  const xs=recent.map(p=>(p.t-recent[0].t)/60000),mx=xs.reduce((s,x)=>s+x,0)/xs.length;
  const variance=xs.reduce((s,x)=>s+(x-mx)**2,0);
  const slope=variance?recent.reduce((s,p,i)=>s+(xs[i]-mx)*(Number(p.temperature)-mean),0)/variance:NaN;
  const values=recent.map(p=>Number(p.temperature));
  return {mean,slope,stable:Math.max(...values)-Math.min(...values)<=.03&&Math.abs(slope)<=.06};
}
function validPulsePoints(points){return [...new Map(points.filter(p=>!p.gap&&p.heartRate!=null&&Number.isFinite(Number(p.heartRate))&&p.heartRate>=30&&p.heartRate<=240&&(p.pulseStatus==null||p.pulseStatus===2)&&(p.quality==null||p.quality>=75)).map(p=>[p.estimateT??p.t,{...p,t:p.estimateT??p.t}])).values()].sort((a,b)=>a.t-b.t)}
function pulseRecovery(points,baseline,target){
  const valid=validPulsePoints(points).filter(p=>p.t>target-60000&&p.t<=target+2000);
  const tail=valid.filter(p=>p.t>=target-5000);
  if(valid.length<45||tail.length<3||tail.at(-1).t<target-2000||points.some(p=>p.gap&&p.t>target-60000&&p.t<=target))return NaN;
  for(let i=1;i<valid.length;i++)if(valid[i].t-valid[i-1].t>2500)return NaN;
  return baseline-tail.reduce((s,p)=>s+Number(p.heartRate),0)/tail.length;
}
function startRecovery(mac){
  const n=statusData.sensors.find(n=>n.mac===mac);if(!n||n.recording?.state!==2)return;
  const now=Date.now(),points=validPulsePoints(getHistory(mac)).filter(p=>p.t>=now-5000&&p.t<=now);
  if(points.length<3||now-points.at(-1).t>2000){notify('Wait for at least three recent valid pulse estimates.',true);return}
  recoveryMeasurements[mac]={session:n.recording.session,started:now,baseline:points.reduce((s,p)=>s+Number(p.heartRate),0)/points.length};
  redrawRecordings();
}
function sensorFeedback(n,history,end){
  const points=feedbackPoints(history,end),last=points.at(-1),type=Number(n.sensorType),saved=n._archiveEnd!=null;
  const scope=saved?'Last minute of selected view':'Last minute';let tiles='',note='',control='';
  if(type===3){
    const measured=(!saved&&n.recording?.state!==2)?[]:points.filter(p=>p.steps!=null&&!p.gap),latest=measured.at(-1),recent=measured.filter(p=>p.t>=end-10000),first=recent[0];
    const fresh=latest&&end-latest.t<=500;
    const cadence=fresh&&first&&latest.t-first.t>=8000&&latest.steps>=first.steps&&!points.some(p=>p.gap&&p.t>=end-10000)?(latest.steps-first.steps)*60000/(latest.t-first.t):NaN;
    const peaks=points.filter(p=>!p.gap&&Number.isFinite(p.peakAcceleration)).map(p=>p.peakAcceleration);
    tiles=feedbackTile('Steps · session',latest?String(latest.steps):'–')+feedbackTile('Cadence · 10 s',feedbackNumber(cadence,0,'steps/min'))+feedbackTile('Active time',latest?feedbackNumber(latest.activeSeconds,0,'s'):'–')+feedbackTile('Accel peak',peaks.length?feedbackNumber(Math.max(...peaks),2,'g'):'–');
    note=!saved&&n.recording?.state!==2?'Press SW2 for continuous 104 Hz step/cadence tracking. Acceleration includes gravity.':'104 Hz estimates (gravity included; requires fixed placement).';
  }else if(type===4){
    const result=temperatureFeedback(points,end),duration=saved?n._durationMs:n.recording?.state===2?n.recording.elapsedMs:null;
    tiles=feedbackTile(result.stable?'Stable · 10 s':'Settling',result.stable?feedbackNumber(result.mean,3,'°C'):Number.isFinite(result.mean)?'Changing':!saved&&Number(n.sleepSeconds)>1&&n.recording?.state!==2?'Use 1 s or SW2':'Collecting')+feedbackTile('Rate · 10 s',feedbackNumber(result.slope,3,'°C/min'))+feedbackTile('Duration',duration!=null?feedbackNumber(duration/1000,0,'s'):'–');
    note='Stable = ≤0.03 °C spread & ≤0.06 °C/min drift over ≥8 s (sensor temperature, not medical).';
  }else if(type===5){
    const valid=validPulsePoints(points),values=valid.map(p=>Number(p.heartRate)),fresh=last&&end-last.t<=1500;
    const quality=!saved&&n.recording&&n.recording.state!==2?'Not recording':!fresh?'No recent signal':last.gap||last.pulseStatus===3?'Signal gap':last.pulseStatus===0?'No contact':last.pulseStatus===2?`Periodic · ${last.quality??'–'}/100`:'Settling / poor';
    tiles=feedbackTile('Signal quality',quality)+feedbackTile('Pulse min',values.length?feedbackNumber(Math.min(...values),0,'bpm'):'–')+feedbackTile('Pulse avg',values.length?feedbackNumber(values.reduce((s,v)=>s+v,0)/values.length,0,'bpm'):'–')+feedbackTile('Pulse max',values.length?feedbackNumber(Math.max(...values),0,'bpm'):'–');
    let recovery='Mark end of exercise';
    if(saved){
      const base=valid.filter(p=>p.t>=end-60000&&p.t<=end-55000),baseline=base.length>=3?base.reduce((s,p)=>s+Number(p.heartRate),0)/base.length:NaN;
      recovery=feedbackNumber(pulseRecovery(points,baseline,end),1,'bpm');
      note='Drop between first and last 5 s of this minute (valid estimates only; non-medical).';
    }else{
      const r=recoveryMeasurements[n.mac];
      if(r&&r.session===n.recording?.session){
        if(r.result==null&&n.recording?.state!==2&&Date.now()<r.started+60000)r.failed=true;
        if(!r.failed&&r.result==null&&Date.now()>=r.started+60000){const value=pulseRecovery(points,r.baseline,r.started+60000);r.result=Number.isFinite(value)?value:null;if(Date.now()>r.started+62500||n.recording?.state!==2)r.failed=r.result==null}
        recovery=r.failed?'Insufficient signal':r.result!=null?feedbackNumber(r.result,1,'bpm'):`Measuring · ${Math.max(0,Math.ceil((r.started+60000-Date.now())/1000))} s`;
      }
      control=`<button type="button" class="btn secondary" ${n.recording?.state===2?'':'disabled'} onclick="startRecovery('${esc(n.mac)}')">Start recovery</button>`;
      note='Press Start recovery after exercise and keep recording for 60 s (valid estimates only; non-medical).';
    }
    tiles+=feedbackTile(saved?'Pulse drop':'Recovery · 60 s',recovery);
  }else return '';
  return `<section class="sensorFeedback" aria-label="Sensor feedback"><div class="chartHead"><b>Quick feedback</b><small class="muted">${scope}</small>${control}</div><div class="feedbackGrid">${tiles}</div><p class="muted">${note}</p></section>`;
}
function setPulseWindow(mac,value){pulseWindows[mac]=Number(value);renderSensors(statusData.sensors.filter(n=>n.provisioned))}

const recordingViews={};
const feedbackSessions={};
let recordingTransfers=0;
function redrawRecordings(){renderSensors((statusData.sensors||[]).filter(n=>n.provisioned))}
function supportsRecordings(n){return [3,4,5].includes(Number(n.sensorType))||[3,4,5].includes(Number(n.configuredSensorType))||Boolean(recordingViews[n.mac]?.sessions?.length)}
function syncRecordingLists(sensors){
  for(const n of sensors){
    const v=recordingViews[n.mac]??={};if(!supportsRecordings(n)&&v.sessions)continue;
    const state=Number(n.recording?.state),signature=JSON.stringify([state,n.recording?.session,n.recording?.pending]);
    const fastSync=v.waitingForSync||[3,5,6,7].includes(state)||(v.lastRecState===2&&state!==2)||(v.lastRecState===3&&state===4);
    const interval=fastSync?2000:30000;
    if(!v.listLoading&&((!v.sessions&&!v.listChecked)||Date.now()-(v.listChecked||0)>=interval||(signature!==v.statusSignature&&state!==2))){
      v.statusSignature=signature;v.lastRecState=state;loadRecordings(n.mac,true);
    }else v.lastRecState=state;
  }
}
function recordingControls(n){
  if(!supportsRecordings(n))return '';
  const r=n.recording,v=recordingViews[n.mac]||{},labels=['USB setup required','Ready · press SW2 to record','Recording · press SW2 to stop','Synchronizing to station','Synchronized','Storage full · stopped','Storage fault · stopped','Preparing'],duration=v.info?recordingAvailableMs(v.info):0;
  const sessions=v.sessions||[],latestComplete=!v.selected&&sessions.find(x=>x.complete);
  const syncPct=v.waitingForSync&&v.waitingInfo?.expected?Math.min(100,Math.round(100*(v.waitingInfo.stored||0)/Math.max(1,v.waitingInfo.expected))):v.busy&&v.loadingRecords?Math.min(100,Math.round(100*(v.loadedRecords||0)/Math.max(1,v.loadingRecords))):r?.state===3&&r.capacity?Math.max(5,Math.min(100,Math.round(100*(1-Number(r.pending||0)/Math.max(1,Number(r.capacity)))))):null;
  return `<div class="recordingPanel" data-ui-key="recordings"><div class="chartHead"><b>${r?labels[r.state]||'Recording status':'Manual recordings'}</b><div class="actionRow" style="margin:0">${latestComplete?`<button type="button" class="btn" onclick="selectRecording('${esc(n.mac)}','${esc(latestComplete.session)}')">View latest recording</button>`:''}<button type="button" class="btn secondary" onclick="loadRecordings('${esc(n.mac)}')">Recordings</button></div></div>${syncPct!=null?`<div class="syncBar" role="progressbar" aria-valuenow="${syncPct}" aria-valuemin="0" aria-valuemax="100"><i style="width:${syncPct}%"></i></div>`:''}${r?`<p class="muted">${r.pending} / ${r.capacity} buffered on sensor${r.dropped?` · ${r.dropped} lost`:''}.</p>`:''}<div class="chartHead" data-ui-key="recording-selection"><select aria-label="Recording selection" onchange="selectRecording('${esc(n.mac)}',this.value)"><option value="">Live · last minute</option>${sessions.map(x=>`<option value="${x.session}" ${v.selected===x.session?'selected':''}>${x.epochMs?new Date(x.epochMs).toLocaleString():'Session '+x.session.slice(-6)} · ${(x.durationMs/60000).toFixed(1)} min · ${x.complete?'complete':x.stored+'/'+x.expected+' records'}</option>`).join('')}</select><small class="muted">${v.sessions?`Free space: ${Math.floor((v.freeBytes||0)/1024)} KiB`:'Loading saved recordings …'}</small></div>${v.sessions&&!sessions.length?'<p class="muted">No saved recordings yet. Press SW2 to start/stop; completed recordings sync automatically.</p>':''}${v.selected?`<div class="recordingNav" data-ui-key="recording-navigation"><label>View <select aria-label="Recording view" onchange="setRecordingView('${esc(n.mac)}',this.value)"><option value="full" ${v.view!=='minute'?'selected':''}>Full recording</option><option value="minute" ${v.view==='minute'?'selected':''}>Minute detail</option></select></label><button type="button" class="btn secondary" ${v.view!=='minute'||v.endMs<=Math.min(60000,duration)?'disabled':''} onclick="moveRecording('${esc(n.mac)}',-60000)">Previous minute</button><span>${v.view==='minute'?Math.max(0,(v.endMs-60000)/1000).toFixed(0):'0'}–${(v.endMs/1000).toFixed(0)} s</span><button type="button" class="btn secondary" ${v.view!=='minute'||v.endMs>=duration?'disabled':''} onclick="moveRecording('${esc(n.mac)}',60000)">Next minute</button><button type="button" class="btn secondary" ${v.busy||v.refreshing||v.missing?'disabled':''} onclick="reloadRecording('${esc(n.mac)}')">${v.waitingForSync?'Show available data':'Load latest data'}</button><button type="button" class="btn secondary" ${v.exportBusy?'disabled':''} onclick="exportRecording('${esc(n.mac)}')">Export recording</button><button type="button" class="btn danger" ${v.exportBusy?'disabled':''} onclick="deleteRecording('${esc(n.mac)}')">Delete recording</button></div>`:''}${v.waitingForSync?`<p class="muted" role="status">Waiting for synchronization · ${v.waitingInfo?.stored||0} / ${v.waitingInfo?.expected||0} records. No partial graph is drawn automatically.</p>`:''}${v.selected&&!v.waitingForSync&&v.updatesAvailable?'<p class="muted" role="status">New synchronized data available. The graph stays fixed until you choose Load latest data.</p>':''}${v.selected&&v.missing?'<p class="muted" role="status">Recording no longer on station. The loaded graph remains available.</p>':''}${v.exportBusy?`<p class="muted" role="status" data-ui-key="recording-export">Exporting recording · ${Number(v.exportProgress)||0} records loaded <button type="button" class="btn secondary" onclick="cancelRecordingExport('${esc(n.mac)}')">Cancel export</button></p>`:''}${v.busy?`<p class="muted" data-ui-key="recording-progress">Loading recording …${v.loadingRecords?` (${v.loadedRecords||0}/${v.loadingRecords})`:''}</p>`:''}${v.error?`<p class="muted" role="status" data-ui-key="recording-error">${esc(v.error)} <button type="button" class="btn secondary" onclick="selectRecording('${esc(n.mac)}','${esc(v.selected)}',${Number(v.endMs)||0})">Retry</button></p>`:''}</div>`;
}
function recordingAvailableMs(info){return info.complete?Number(info.durationMs)||0:Math.min(Number(info.durationMs)||0,Number(info.availableMs??info.durationMs)||0)}
async function loadRecordings(mac,silent=false){
  const v=recordingViews[mac]??={};if(v.listLoading)return;v.listLoading=true;v.listChecked=Date.now();
  try{
    const data=await requestJson('/api/recordings?mac='+encodeURIComponent(mac),15000);if(recordingViews[mac]!==v)return;if(data.success===false)throw new Error(data.message);
    Object.assign(v,{sessions:(data.sessions||[]).sort((a,b)=>Number(b.epochMs)-Number(a.epochMs)||b.session.localeCompare(a.session)),freeBytes:data.freeBytes});
    const info=v.sessions.find(x=>x.session===v.selected);
    // Poll synchronization metadata without modifying the selected snapshot.
    v.missing=Boolean(v.selected&&!info);
    if(v.waitingForSync&&info){v.waitingInfo=info;if(info.complete&&!v.busy){v.waitingForSync=false;await selectRecording(mac,v.selected,null,true)}}
    v.updatesAvailable=Boolean(info&&!info.readError&&v.info&&(info.stored!==v.info.stored||info.complete!==v.info.complete));
    redrawRecordings();
  }catch(e){if(!silent)notify(e.name==='AbortError'?'The recording list timed out. Try again.':e.message,true)}
  finally{v.listLoading=false}
}
async function requestRecordingJson(url,signal){
  const controller=new AbortController(),abort=()=>controller.abort(),timer=setTimeout(abort,15000);signal?.addEventListener('abort',abort,{once:true});
  try{if(signal?.aborted)controller.abort();const response=await fetch(url,{cache:'no-store',signal:controller.signal});if(response.status===401){location.replace('/login');throw new Error('Authentication required')}if(!response.ok)throw new Error(`HTTP ${response.status}`);return await response.json()}
  finally{clearTimeout(timer);signal?.removeEventListener('abort',abort)}
}
async function selectRecording(mac,session,endMs,refresh=false){
  const v=recordingViews[mac]??={};v.controller?.abort();v.request=(v.request||0)+1;const request=v.request;
  if(!session){Object.assign(v,{selected:'',points:[],busy:false,error:'',info:null,waitingForSync:false});redrawRecordings();return}
  const available=(v.sessions||[]).find(x=>x.session===session);if(!available){v.busy=false;v.error='This recording is no longer available. Refresh the recording list.';redrawRecordings();return}
  const same=v.selected===session,info={...available},stored=Math.max(0,Number(info.stored)||0),controller=new AbortController();
  const canAppend=Boolean(same&&refresh&&Array.isArray(v.points)&&v.points.length>0&&v.points.length<stored&&v.info?.session===session);
  // An explicit archive selection takes precedence over a previously opened metric.
  metricFocus[mac]=false;
  Object.assign(v,{selected:session,busy:true,error:'',controller,loadedRecords:canAppend?v.points.length:0,loadingRecords:stored,missing:false});
  if(!same){Object.assign(v,{info,points:[],view:'full',endMs:recordingAvailableMs(info)});delete pulseWindows[mac]}
  if(!refresh&&!same&&!info.readError&&!info.complete&&stored<Number(info.expected)){v.waitingForSync=true;v.waitingInfo=info;v.busy=false;redrawRecordings();return}
  v.waitingForSync=false;redrawRecordings();recordingTransfers++;
  try{
    let offset=canAppend?v.points.length:0,lastProgressPaint=Date.now();
    const points=canAppend?[...v.points]:[];
    // Fix the record count at selection time. Appends during synchronization
    // cannot extend this request indefinitely or change the displayed window.
    while(offset<stored){
      const data=await requestRecordingJson('/api/recording?mac='+encodeURIComponent(mac)+'&session='+encodeURIComponent(session)+'&offset='+offset+'&limit=512',controller.signal);
      if(v.request!==request)return;if(data.success===false)throw new Error(data.message);
      if(data.session!==session||!Number.isInteger(data.stored)||data.stored<stored)throw new Error('The recording changed while loading. Try again.');
      const page=data.points,next=Number(data.nextOffset);
      if(!Array.isArray(page)||!page.length||page.length>512||!Number.isInteger(next)||next!==offset+page.length||next>data.stored)throw new Error('Recording page is incomplete or its cursor is invalid. Try again.');
      points.push(...page.slice(0,stored-offset));offset=Math.min(stored,next);
      v.loadedRecords=offset;
      if(offset<stored&&Date.now()-lastProgressPaint>=250){lastProgressPaint=Date.now();redrawRecordings();}
    }
    if(v.request===request){
      Object.assign(v,{points,info,endMs:recordingAvailableMs(info),updatesAvailable:false});
      if(endMs!=null)v.endMs=Math.min(v.endMs,Math.max(Math.min(60000,v.endMs),endMs));
      v.plotRevision=(v.plotRevision||0)+1;
    }
  }catch(e){if(v.request===request)v.error=e.name==='AbortError'?'Loading this recording timed out. Try again.':e.message}
  finally{recordingTransfers--;if(v.request===request){v.busy=false;redrawRecordings()}}
}
function setRecordingView(mac,value){const v=recordingViews[mac];if(!v?.selected)return;v.view=value==='minute'?'minute':'full';v.endMs=recordingAvailableMs(v.info);redrawRecordings()}
function moveRecording(mac,delta){const v=recordingViews[mac];if(!v?.selected)return;const duration=recordingAvailableMs(v.info);v.view='minute';v.endMs=Math.min(duration,Math.max(Math.min(60000,duration),v.endMs+delta));redrawRecordings()}
async function reloadRecording(mac){const v=recordingViews[mac];if(!v?.selected||v.busy||v.refreshing)return;const session=v.selected;v.waitingForSync=false;v.refreshing=true;redrawRecordings();try{while(v.listLoading)await new Promise(resolve=>setTimeout(resolve,25));await loadRecordings(mac,true);if(v.selected===session)await selectRecording(mac,session,null,true)}finally{v.refreshing=false;redrawRecordings()}}
function recordingCharts(n,history){
  const v=recordingViews[n.mac];if(!v?.selected)return `<div data-ui-key="live-charts">${sensorFeedback(n,history,Date.now())+liveCharts(n,history)}</div>`;
  if(!v.info)return '';
  if(v.waitingForSync)return `<div class="recordingCharts" data-ui-key="archive-${n.mac}-${v.selected}"><div class="chartLoadingBox">Waiting for synchronization to finish before drawing the complete graph.</div></div>`;
  const key='archive-'+n.mac+'-'+v.selected,signature=JSON.stringify([v.plotRevision,v.view,v.endMs,v.info,Boolean(v.busy&&!v.points?.length),chartFilters[n.mac],pulseWindows[n.mac],n.motionReference]);
  // Cache complete markup: an ancestor can be replaced while the previous
  // graph is still connected. An empty placeholder would then erase the graph.
  if(v.chartSignature===signature&&v.chartHtml)return v.chartHtml;
  v.chartSignature=signature;v.chartVersion=(v.chartVersion||0)+1;
  const epoch=Number(v.info.epochMs)||0,points=(v.points||[]).map(p=>({...p,t:Number(p.t)+epoch,estimateT:Number(p.estimateT??p.t)+epoch,wave:(p.wave||[]).map(w=>({...w,t:Number(w.t)+epoch}))})),context={...n,sensorType:v.info.sensorType,_archiveEnd:epoch+v.endMs,_archiveWindowSeconds:v.view==='minute'?60:Math.max(1,v.endMs/1000),_durationMs:v.info.durationMs};
  let charts=sensorFeedback(context,points,epoch+v.endMs)+liveCharts(context,points);if(!epoch)charts=charts.replaceAll('class="liveSvg"','class="liveSvg" data-relative="1" data-epoch="0"');
  v.chartHtml=`<div class="recordingCharts" data-ui-key="${key}" data-chart-version="${v.chartVersion}"><p class="muted" style="font-size:12px;margin:0 0 6px">${epoch?'Acquired '+new Date(epoch).toLocaleString():'Relative timestamps (station clock unavailable)'} · ${v.info.complete?'Complete':'Partial sync'} · ${v.info.stored} records</p>${v.busy&&!points.length?'<div class="chartLoadingBox">Loading the selected recording …</div>':charts}</div>`;
  return v.chartHtml;
}
function cancelRecordingExport(mac){recordingViews[mac]?.exportController?.abort()}
async function exportRecording(mac){
  const v=recordingViews[mac],session=v?.selected;if(!session||v.exportBusy)return;
  const controller=new AbortController();Object.assign(v,{exportBusy:true,exportProgress:0,exportController:controller});
  recordingTransfers++;redrawRecordings();
  try{
    const lines=['session,epoch_ms,kind,sample_ms,time_ms,temperature_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,heart_rate_bpm,red,infrared,gap,steps,active_seconds,signal_correlation,pulse_status'];let offset=0,lastRender=0;
    for(;;){
      const data=await requestRecordingJson('/api/recording?mac='+encodeURIComponent(mac)+'&session='+encodeURIComponent(session)+'&offset='+offset+'&limit=512',controller.signal);
      if(data.success===false)throw new Error(data.message);
      for(const p of data.points){
        lines.push([session,data.epochMs,'report',p.sampleMs,p.t,p.temperature,p.ax,p.ay,p.az,p.gx,p.gy,p.gz,p.heartRate,'','',p.gap?1:0,p.steps,p.activeSeconds,p.quality,p.pulseStatus].join(','));
        for(const w of p.wave||[])lines.push([session,data.epochMs,'optical',p.sampleMs,w.t,'','','','','','','','',w.red,w.infrared,p.gap?1:0,'','','',''].join(','));
      }
      v.exportProgress=data.nextOffset;if(Date.now()-lastRender>=250){redrawRecordings();lastRender=Date.now()}
      if(data.nextOffset>=data.stored)break;if(data.nextOffset<=offset)throw new Error('Recording page did not advance');offset=data.nextOffset;
    }
    if(controller.signal.aborted)return;
    const url=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/csv'})),a=document.createElement('a');a.href=url;a.download='recording-'+session+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }catch(e){const cancelled=controller.signal.aborted;notify(cancelled?'Export cancelled.':e.name==='AbortError'?'Export timed out. Try again.':e.message,!cancelled)}
  finally{recordingTransfers--;v.exportBusy=false;v.exportController=null;redrawRecordings()}
}

async function deleteRecording(mac){const v=recordingViews[mac];if(!v?.selected||!confirm('Delete this recording from the station? Export it first if you want to keep it.'))return;try{await apiPost('/api/recording/delete',new URLSearchParams({mac,session:v.selected}));await selectRecording(mac,'');await loadRecordings(mac)}catch(e){notify(e.message,true)}}
const chartFilters={};
const chartEnds={};
function setChartFilter(mac,value){
  chartFilters[mac]=value==='smooth'?'smooth':'raw';
  renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));
}
function chartFilterControl(n){
  const selected=chartFilters[n.mac]||'raw';
  return '<label class="chartFilter">Display filter <select aria-label="Display filter" onchange="setChartFilter(\''+esc(n.mac)+'\',this.value)"><option value="raw" '+(selected==='raw'?'selected':'')+'>Raw samples</option><option value="smooth" '+(selected==='smooth'?'selected':'')+'>Smooth · 3 samples</option></select></label>';
}
function chartSegments(points,gapMs){
  const segments=[];let segment=[];
  for(const p of points){
    if(segment.length&&(p.breakBefore||p.t-segment.at(-1).t>gapMs)){segments.push(segment);segment=[]}
    segment.push(p);
  }
  if(segment.length)segments.push(segment);
  return segments;
}
function chartDisplayPoints(segment,filter){
  if(filter!=='smooth')return segment;
  return segment.map((p,i)=>{
    const neighbours=segment.slice(Math.max(0,i-1),Math.min(segment.length,i+2));
    return {...p,v:neighbours.reduce((sum,q)=>sum+q.v,0)/neighbours.length};
  });
}
function chartScale(values,minSpan=0,reference=null){
  let low=Infinity,high=-Infinity;for(const value of values){low=Math.min(low,value);high=Math.max(high,value)}
  if(reference!==null){low=Math.min(low,reference);high=Math.max(high,reference)}
  const centre=(low+high)/2,span=Math.max(high-low,minSpan,Math.abs(centre)*.001,.02)*1.16;
  low=centre-span/2;high=centre+span/2;
  const rough=span/4,power=10**Math.floor(Math.log10(rough)),step=[1,2,2.5,5,10].map(v=>v*power).find(v=>v>=rough)||power*10;
  return {low:Math.floor(low/step)*step,high:Math.ceil(high/step)*step,step};
}
function chartTimestamp(svg,time){
  if(svg.dataset.relative==='1')return ((time-Number(svg.dataset.epoch||0))/1000).toFixed(3)+' s into recording';
  const date=new Date(time);
  return date.toLocaleString([],{day:'2-digit',month:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit'})+'.'+String(date.getMilliseconds()).padStart(3,'0');
}
function chartReadout(svg,time,values,outside=false){
  const readout=svg.parentElement.querySelector('.chartReadout');
  if(!readout)return;
  svg._cursorSnapshot={time,values};
  readout.innerHTML='<span class="chartReadoutTime">'+esc(chartTimestamp(svg,time))+(svg._cursorPinned?(outside?' · Pinned outside current window':' · Pinned'):'')+'</span>'+values.map(v=>'<span class="chartReadoutValue"><i class="chartReadoutSwatch" style="background:'+v.color+'"></i>'+esc(v.label)+' <b>'+Number(v.v).toFixed(Number(svg.dataset.digits))+(svg.dataset.unit?' '+esc(svg.dataset.unit):'')+'</b></span>').join('')+'<span class="chartCursorActions"><button type="button" class="btn secondary" aria-label="Previous sample" onclick="chartStep(this,-1)">‹</button><button type="button" class="btn secondary" aria-label="Next sample" onclick="chartStep(this,1)">›</button><button type="button" class="btn secondary" aria-label="Clear selected sample" onclick="chartClear(this)">×</button></span>';
}
function chartLeave(svg){
  if(svg._cursorPinned)return;
  const cursor=svg.querySelector('.liveCursor,.hoverCursor');
  if(cursor)cursor.style.display='none';
}
function chartPointerDown(e,handler){
  const svg=e.currentTarget;svg._chartGesture={x:e.clientX,y:e.clientY,vertical:false};
  if(e.pointerType==='mouse'){svg._cursorPinned=true;handler(e)}
}
function chartPointerMove(e,handler){
  const svg=e.currentTarget,g=svg._chartGesture;
  if(g&&e.pointerType==='mouse'&&!e.buttons){delete svg._chartGesture;if(!svg._cursorPinned)handler(e);return}
  if(g){
    const dx=Math.abs(e.clientX-g.x),dy=Math.abs(e.clientY-g.y);
    if(dy>9&&dy>dx)g.vertical=true;
    if(!g.vertical&&dx>6){svg._cursorPinned=true;handler(e)}
  }else if(e.pointerType==='mouse'&&!svg._cursorPinned)handler(e);
}
function chartPointerUp(e,handler){
  const svg=e.currentTarget,g=svg._chartGesture;
  if(g&&!g.vertical){svg._cursorPinned=true;handler(e)}
  delete svg._chartGesture;
}
function chartClear(button){
  const plot=button.closest('.livePlot,.historyPlot'),svg=plot.querySelector('svg');
  svg._cursorPinned=false;delete svg._chartGesture;delete svg._cursorSnapshot;delete svg.dataset.cursorTime;
  svg.querySelector('.liveCursor,.hoverCursor').style.display='none';
  plot.querySelector('.chartReadout').innerHTML='<span class="muted">Hover, tap a point, or use arrow keys for exact sample values.</span>';
  svg.focus({preventScroll:true});
}
function chartStep(button,direction){
  const svg=button.closest('.livePlot,.historyPlot').querySelector('svg');
  chartKey({key:direction<0?'ArrowLeft':'ArrowRight',currentTarget:svg,preventDefault(){}},svg.classList.contains('liveSvg')?liveHover:chartMove);
}
function chartData(svg,key){
  const source=(svg.dataset.cacheKey||'')+(svg.dataset[key]||'');
  if(svg._chartDataSource!==source){svg._chartDataSource=source;svg._chartData=svg.dataset.cacheKey?(historyPayloads.get(svg.dataset.cacheKey)||[]):JSON.parse(svg.dataset[key]||'[]')}
  return svg._chartData;
}
function chartLayout(scale,digits){
  let maxLen=0;
  for(let v=scale.low;v<=scale.high+scale.step*.1;v+=scale.step)maxLen=Math.max(maxLen,v.toFixed(digits).length);
  const padLeft=Math.min(104,Math.max(68,Math.round(maxLen*11+12))),plotW=588-padLeft;
  return {padLeft,plotW,right:588,xFractions:window.innerWidth<=480?[0,.5,1]:[0,.25,.5,.75,1]};
}
function chartKey(e,handler){
  if(!['ArrowLeft','ArrowRight','Home','End'].includes(e.key))return;
  const svg=e.currentTarget,series=svg.dataset.series?chartData(svg,'series'):[{points:chartData(svg,'samples')}],times=[...new Set(series.flatMap(s=>s.points.map(p=>p.t)))].sort((a,b)=>a-b);
  if(!times.length)return;
  e.preventDefault();
  svg._cursorPinned=true;
  let index=Math.max(0,times.indexOf(Number(svg.dataset.cursorTime)));
  index=e.key==='Home'?0:e.key==='End'?times.length-1:Math.max(0,Math.min(times.length-1,index+(e.key==='ArrowRight'?1:-1)));
  const box=svg.getBoundingClientRect(),padLeft=Number(svg.dataset.padLeft||64),plotW=Number(svg.dataset.plotWidth||520),x=padLeft+(times[index]-Number(svg.dataset.start))*plotW/Number(svg.dataset.window);
  handler({currentTarget:svg,clientX:box.left+x/600*box.width});
}
function liveSeries(history,key,color,label,timeKey='t'){
  const points=[];let breakBefore=false;
  for(const p of history){
    if(p.gap||p[key]==null||!Number.isFinite(Number(p[key]))){breakBefore=true;continue}
    const t=Number(p[timeKey]);if(!Number.isFinite(t)){breakBefore=true;continue}
    points.push({t,v:Number(p[key]),breakBefore});breakBefore=false;
  }
  return {key,color,label,points};
}
function livePlot(title,unit,series,seconds,minSpan,reference=null,gapMs=1500,digits=2,end=Date.now(),filter='raw'){
  const start=end-seconds*1000;
  series=series.map((s,i)=>({...s,dash:['','7 3','2 3'][i%3],points:s.points.filter(p=>p.t>=start&&p.t<=end&&Number.isFinite(p.v)).sort((a,b)=>a.t-b.t)}));
  const values=series.flatMap(s=>s.points.map(p=>p.v));
  if(!values.length)return '<div class="livePlot"><b>'+esc(title)+'</b><p class="muted">No valid measurements in this window. Gaps are not interpolated.</p></div>';
  const scale=chartScale(values,minSpan,reference),span=scale.high-scale.low,lay=chartLayout(scale,digits),x=t=>lay.padLeft+(t-start)*lay.plotW/(seconds*1000),y=v=>156-(v-scale.low)*132/span;
  const paths=series.map(s=>chartSegments(s.points,gapMs).map(segment=>{
    const path=points=>chartReducePoints(points).map((p,i)=>(i?'L':'M')+x(p.t).toFixed(2)+','+y(p.v).toFixed(2)).join('');
    const raw=filter==='smooth'&&segment.length>1?'<path class="rawTrace" d="'+path(segment)+'" stroke="'+s.color+'" fill="none" stroke-width="1"/>':'';
    const trace='<path class="seriesTrace" d="'+path(chartDisplayPoints(segment,filter))+'" stroke="'+s.color+'" stroke-dasharray="'+s.dash+'" fill="none" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round"/>';
    const marked=segment.length<=90?segment:segment.filter((p,i)=>i===0||i===segment.length-1);
    return raw+trace+marked.map(p=>'<circle class="sampleDot" cx="'+x(p.t)+'" cy="'+y(p.v)+'" r="'+(segment.length===1?3.7:2)+'" fill="'+s.color+'"/>').join('');
  }).join('')).join('');
  const ticks=[];
  for(let v=scale.low;v<=scale.high+scale.step*.1;v+=scale.step)ticks.push('<line class="grid" x1="'+lay.padLeft+'" x2="'+lay.right+'" y1="'+y(v)+'" y2="'+y(v)+'"/><text x="'+(lay.padLeft-8)+'" y="'+(y(v)+4)+'" text-anchor="end">'+v.toFixed(digits)+'</text>');
  const xticks=lay.xFractions.map(f=>'<text x="'+(lay.padLeft+lay.plotW*f)+'" y="177" text-anchor="'+(f===0?'start':f===1?'end':'middle')+'">'+(f===1?(end<Date.now()-2000?'end':'now'):'−'+Math.round(seconds*(1-f))+' s')+'</text>').join('');
  const legend=series.map(s=>'<span class="chartLegend" style="color:'+s.color+'"><svg width="24" height="12" aria-hidden="true"><line x1="0" x2="24" y1="6" y2="6" stroke="currentColor" stroke-width="2.4" stroke-dasharray="'+s.dash+'"/></svg>'+esc(s.label)+'</span>').join('');
  const circles=series.map(s=>'<circle class="cursorDot" data-key="'+esc(s.key)+'" r="4" stroke="'+s.color+'" fill="var(--chart-bg,#fff)" stroke-width="2"/>').join('');
  return '<div class="livePlot"><div class="plotHeading"><b>'+esc(title)+' <small class="muted">('+esc(unit)+')</small></b><span class="chartLegendGroup">'+legend+'</span></div><svg class="liveSvg" viewBox="0 0 600 190" role="img" tabindex="0" aria-label="'+esc(title)+'" '+chartSeriesAttribute(series)+' data-pad-left="'+lay.padLeft+'" data-plot-width="'+lay.plotW+'" data-start="'+start+'" data-window="'+seconds*1000+'" data-low="'+scale.low+'" data-span="'+span+'" data-gap="'+gapMs+'" data-digits="'+digits+'" data-unit="'+esc(unit)+'" data-filter="'+filter+'" onpointermove="chartPointerMove(event,liveHover)" onpointerdown="chartPointerDown(event,liveHover)" onpointerup="chartPointerUp(event,liveHover)" onpointercancel="delete this._chartGesture" onpointerleave="chartLeave(this)" onkeydown="chartKey(event,liveHover)">'+ticks.join('')+xticks+(reference===null?'':'<line x1="'+lay.padLeft+'" x2="'+lay.right+'" y1="'+y(reference)+'" y2="'+y(reference)+'" stroke="var(--muted)" stroke-dasharray="4 4"/>')+paths+'<g class="liveCursor" style="display:none"><line class="cursor" y1="24" y2="156"/>'+circles+'<text class="cursorSummary" x="'+lay.padLeft+'" y="188"></text></g></svg><div class="chartReadout"><span class="muted">Hover, tap a point, or use arrow keys for exact sample values.</span></div></div>';
}
function liveHover(e){
  const svg=e.currentTarget,box=svg.getBoundingClientRect();
  if(!(box.width>0))return;
  const padLeft=Number(svg.dataset.padLeft||64),plotW=Number(svg.dataset.plotWidth||520);
  const px=(e.clientX-box.left)/box.width*600,time=Number(svg.dataset.start)+(Math.max(padLeft,Math.min(padLeft+plotW,px))-padLeft)/plotW*Number(svg.dataset.window),series=chartData(svg,'series');
  let nearest=null;
  for(const s of series){if(!s.points.length)continue;const p=nearestHistoryPoint(s.points,time);if(p&&(!nearest||Math.abs(p.t-time)<Math.abs(nearest.t-time)))nearest=p}
  if(!nearest)return;
  const values=[],cursor=svg.querySelector('.liveCursor'),x=padLeft+(nearest.t-Number(svg.dataset.start))*plotW/Number(svg.dataset.window);
  svg.dataset.cursorTime=nearest.t;cursor.style.display='block';cursor.querySelector('line').setAttribute('x1',x);cursor.querySelector('line').setAttribute('x2',x);
  for(const s of series){
    const best=s.points.length?nearestHistoryPoint(s.points,nearest.t):null;
    const dot=cursor.querySelector('[data-key="'+s.key+'"]'),valid=best&&Math.abs(best.t-nearest.t)<=Math.max(1,Number(svg.dataset.gap)/2);
    if(dot){dot.style.display=valid?'':'none';if(valid){dot.setAttribute('cx',padLeft+(best.t-Number(svg.dataset.start))*plotW/Number(svg.dataset.window));dot.setAttribute('cy',156-(best.v-Number(svg.dataset.low))*132/Number(svg.dataset.span))}}
    if(valid)values.push({...s,...best});
  }
  chartReadout(svg,nearest.t,values);
  cursor.querySelector('text').textContent=chartTimestamp(svg,nearest.t)+' · '+values.map(v=>v.label+' '+v.v.toFixed(Number(svg.dataset.digits))+' '+svg.dataset.unit).join(' · ');
}
function liveCharts(n,history){const endTime=n._archiveEnd??Date.now(),type=Number(n.sensorType),chartSeconds=n._archiveWindowSeconds??60,footer=`<div class="muted" style="font-size:12px;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px"><span>${n._archiveEnd!=null?'Saved recording snapshot':'Live RAM window (1 min)'} · gaps retained</span>${n._archiveEnd!=null?'':`<button type="button" class="btn secondary" onclick="exportHistory('${esc(n.mac)}')">CSV</button>`}</div>`,colors=['var(--series-blue)','var(--series-orange)','var(--series-purple)'],filter=chartFilters[n.mac]||'raw',toolbar='<div class="chartToolbar">'+chartFilterControl(n)+'<span class="chartFilterNote">'+(n.motionReference?.enabled&&type===3?'Axes relative to resting reference; CSV keeps raw samples':filter==='smooth'?'3-sample visual smooth · cursor & CSV keep raw samples':'Raw samples')+'</span></div>',fast=n._archiveEnd!=null||n.recording?.state===2||[4,5].includes(Number(n.operatingMode)),gapMs=n._archiveEnd!=null?250:fast?500:Math.max(250,Number(n.sleepSeconds||1)*1500);if(type===3){const plotted=history.map(p=>{const q={...p};for(const key of ['ax','ay','az','gx','gy','gz'])if(p[key]!=null)q[key]=motionValue(n,key,p[key]);return q});return toolbar+`<div style="width:100%">${livePlot('Acceleration X / Y / Z','g',['ax','ay','az'].map((k,i)=>liveSeries(plotted,k,colors[i],['X','Y','Z'][i])),chartSeconds,n.motionReference?.enabled ? .05 : 2,0,gapMs,3,endTime,filter)}${livePlot('Angular rate X / Y / Z','°/s',['gx','gy','gz'].map((k,i)=>liveSeries(plotted,k,colors[i],['X','Y','Z'][i])),chartSeconds,n.motionReference?.enabled?1:20,0,gapMs,n.motionReference?.enabled?2:1,endTime,filter)}${footer}</div>`}if(type===4){const points=history.filter(p=>p.temperature!=null&&Number.isFinite(Number(p.temperature))),values=points.map(p=>Number(p.temperature)),stable=temperatureFeedback(points,endTime).stable;return toolbar+`<div style="width:100%">${livePlot('Temperature trend','°C',[liveSeries(history,'temperature','var(--brand)','TMP117')],chartSeconds,.1,null,fast?1800:Math.max(1800,Number(n.sleepSeconds||1)*1500),3,endTime,filter)}<p class="muted" style="font-size:12px;margin:0 0 8px">${values.length?`Range ${Math.min(...values).toFixed(3)}–${Math.max(...values).toFixed(3)} °C · Δ ${(values.at(-1)-values[0]).toFixed(3)} °C · `:''}${stable?'Stable over last 10 s':'Settling'}</p>${footer}</div>`}const detail=pulseWindows[n.mac]??(n._archiveEnd!=null?0:10),windowSeconds=detail===0?chartSeconds:detail,wave=history.flatMap(p=>(p.wave||[]).map(w=>({...w,gap:p.gap}))).filter(w=>w.t>=endTime-windowSeconds*1000&&w.t<=endTime),series=['red','infrared'].map((key,i)=>{const valid=wave.filter(w=>!w.gap&&Number(w[key])>0),mean=valid.reduce((sum,w)=>sum+Number(w[key]),0)/Math.max(1,valid.length);return{key,label:key==='red'?'Red':'Infrared',color:i?'var(--series-blue)':'var(--series-orange)',points:valid.map(w=>({t:w.t,v:100*(Number(w[key])/mean-1)}))}});return toolbar+`<div style="width:100%">${livePlot('Heart rate trend','bpm',[liveSeries(history,'heartRate','var(--series-purple)','Pulse','estimateT')],chartSeconds,20,null,1800,0,endTime,filter)}<label class="chartFilter" style="margin-bottom:6px">Waveform window <select aria-label="Pulse waveform window" onchange="setPulseWindow('${esc(n.mac)}',this.value)">${(n._archiveEnd!=null?[0,10,60]:[10,60]).map(v=>`<option value="${v}" ${v===detail?'selected':''}>${v===0?'Full recording view':v+' seconds'}</option>`).join('')}</select></label>${livePlot('Optical pulse waveform','% from window mean',series,windowSeconds,.1,0,100,2,endTime,filter)}${footer}</div>`}
const chartDefinitions={heartRate:{label:'Heart rate',unit:'bpm',cap:512,digits:1},red:{label:'Red optical signal',unit:'counts',cap:256,digits:0},infrared:{label:'Infrared optical signal',unit:'counts',cap:256,digits:0},ax:{label:'Acceleration X',unit:'g',cap:128,digits:3},ay:{label:'Acceleration Y',unit:'g',cap:128,digits:3},az:{label:'Acceleration Z',unit:'g',cap:128,digits:3},gx:{label:'Angular rate X',unit:'°/s',cap:128,digits:1},gy:{label:'Angular rate Y',unit:'°/s',cap:128,digits:1},gz:{label:'Angular rate Z',unit:'°/s',cap:128,digits:1},peakAcceleration:{label:'Peak acceleration magnitude',unit:'g',cap:128,digits:3},peakAngularRate:{label:'Peak angular rate magnitude',unit:'°/s',cap:128,digits:1},temperature:{label:'Temperature',unit:'°C',cap:1,digits:3},humidity:{label:'Humidity',unit:'%',cap:2,digits:1},pressure:{label:'Pressure',unit:'hPa',cap:4,digits:0},iaq:{label:'Static IAQ',unit:'',cap:8,digits:0},gas:{label:'Gas resistance',unit:'kΩ',cap:64,digits:1},eco2:{label:'eCO₂ (CCS811)',unit:'ppm',cap:4096,digits:0},tvoc:{label:'TVOC (CCS811)',unit:'ppb',cap:4096,digits:0},distanceMm:{label:'Distance (VL53L1X)',unit:'mm',cap:2048,digits:0},signalRate:{label:'Signal rate',unit:'kcps',cap:2048,digits:0},ambientRate:{label:'Ambient rate',unit:'kcps',cap:2048,digits:0},spectralClear:{label:'Clear spectrum',unit:'counts',cap:1024,digits:0},spectralNir:{label:'NIR spectrum',unit:'counts',cap:1024,digits:0},battery:{label:'Battery voltage',unit:'V',cap:16,digits:3}};
function availableChartMetrics(n){if(Number(n.telemetryFlags)&256)return ['battery'];const t=Number(n.sensorType),caps=Number(n.capabilities||16)|(t===3?128:t===4?1:t===5?768:t===6?5:t===8?4096:t===9?2048:t===10?1024:0);return Object.keys(chartDefinitions).filter(key=>(caps&chartDefinitions[key].cap)!==0)}
function refreshChartHistory(mac){
  const n=(statusData.sensors||[]).find(sensor=>sensor.mac===mac);
  if(!n?.normalHistory||selectedSensorMac!==mac)return;
  const requested=chartWindows[mac]??86400,range=historyRanges[mac];
  if(historyPrefetch?.mac===mac&&historyPrefetch.controller===historyLoadController&&!historyLoadController.signal.aborted&&chartEnds[mac]==null&&requested!=='all'&&Number(requested)<=604800){
    const after=Math.max(0,Math.floor((Date.now()-Number(requested)*1000)/1000)-1);
    if(range&&after>=range.after)delete historyLoading[mac];else historyLoading[mac]=true;
    return;
  }
  historyLoadController?.abort();historyLoadController=null;historyLoadSerial++;delete historyLoading[mac];
  syncHistories(statusData.sensors||[]).then(()=>{if(selectedSensorMac===mac)renderSensors((statusData.sensors||[]).filter(sensor=>sensor.provisioned));});
}
function setChartWindow(mac,value){chartWindows[mac]=value==='all'?'all':Number(value);refreshChartHistory(mac);renderSensors(statusData.sensors.filter(n=>n.provisioned))}
async function exportHistory(mac){
  if(historyPartial[mac]){
    if(historyLoadController&&historyPrefetch?.controller===historyLoadController){historyLoadController.abort();historyLoadController=null;historyLoadSerial++;delete historyLoading[mac];}
    else if(historyLoadController){notify('History is loading. Please try export again shortly.',true);return}
    await syncHistories(statusData.sensors||[],{full:true});
    if(selectedSensorMac!==mac||historyPartial[mac]||historyLoadErrors[mac]){if(selectedSensorMac===mac)renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));return}
    renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));
  }
  const records=getHistory(mac),points=records.flatMap(p=>[{...p,kind:'report',spectralClear:Array.isArray(p.spectral)?p.spectral[8]:'',spectralNir:Array.isArray(p.spectral)?p.spectral[9]:'',spectralAll:Array.isArray(p.spectral)?p.spectral.join(';'):''},...(p.wave||[]).map(w=>({...w,kind:'optical'}))]).sort((a,b)=>a.t-b.t),keys=['kind','t','receivedT','estimateT','windowMs','heartRate','red','infrared','ax','ay','az','gx','gy','gz','peakAcceleration','peakAngularRate','steps','activeSeconds','quality','pulseStatus','gap','temperature','humidity','pressure','iaq','iaqAccuracy','gas','eco2','tvoc','distanceMm','signalRate','ambientRate','rangeStatus','spectralClear','spectralNir','spectralAll','battery'];const csv=keys.join(',')+'\n'+points.map(p=>keys.map(k=>p[k]??'').join(',')).join('\n'),url=URL.createObjectURL(new Blob([csv],{type:'text/csv'})),a=document.createElement('a');a.href=url;a.download='sensor-'+mac.replaceAll(':','')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)
}
function setChartMetric(mac,key){chartMetrics[mac]=key;renderSensors(statusData.sensors.filter(n=>n.provisioned))}
function historyStepLabel(seconds){const value=Math.max(.2,Number(seconds)||1);if(value<1)return`${Math.round(value*1000)} milliseconds`;if(value%31536000===0){const years=value/31536000;return`${years} ${years===1?'year':'years'}`}if(value%86400===0){const days=value/86400;return`${days} ${days===1?'day':'days'}`}if(value%3600===0){const hours=value/3600;return`${hours} ${hours===1?'hour':'hours'}`}if(value%60===0){const minutes=value/60;return`${minutes} ${minutes===1?'minute':'minutes'}`}return`${value} ${value===1?'second':'seconds'}`}function historyContinuitySeconds(history,bucketSeconds){const base=Math.max(.2,Number(bucketSeconds)||1),cutoff=0,times=history.map(p=>Number(p.t)).filter(t=>Number.isFinite(t)&&t>=cutoff).sort((a,b)=>a-b),occurrences=new Map;for(let i=1;i<times.length;i++){const gap=Math.round((times[i]-times[i-1])/1000);if(gap<base||gap>86400)continue;occurrences.set(gap,(occurrences.get(gap)||0)+1)}let continuity=base;occurrences.forEach((count,gap)=>{if(gap>continuity&&(count>=2||(times.length<6&&gap<=3600)))continuity=gap});return continuity}function moveChartHistory(mac,direction){
  if(direction===0){delete chartEnds[mac];refreshChartHistory(mac);renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));return}
  const n=(statusData.sensors||[]).find(n=>n.mac===mac);if(!n)return;
  const now=Date.now(),bounds=historyChartBounds(getHistory(mac),now),oldest=Number(n.history?.oldest)?Number(n.history.oldest)*1000:bounds.oldest,desired=chartWindows[mac]??86400,seconds=desired==='all'?Math.max(1,Math.ceil((now-oldest)/1000)):Number(desired),windowMs=seconds*1000,end=chartEnds[mac]??now,minimumEnd=Math.min(now,oldest+windowMs),next=Math.max(minimumEnd,Math.min(now,end+Number(direction)*windowMs));
  if(next>=now-1)delete chartEnds[mac];else chartEnds[mac]=next;
  refreshChartHistory(mac);renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));
}
function historyRangeLabel(start,end){
  const format=time=>new Date(time).toLocaleString([],{year:'numeric',day:'2-digit',month:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit'});
  return format(start)+' – '+format(end);
}
function chartMove(e){
  const svg=e.currentTarget,box=svg.getBoundingClientRect();
  if(!(box.width>0))return;
  const padLeft=Number(svg.dataset.padLeft||64),plotW=Number(svg.dataset.plotWidth||520);
  const x=(e.clientX-box.left)/box.width*600,time=Number(svg.dataset.start)+(Math.max(padLeft,Math.min(padLeft+plotW,x))-padLeft)/plotW*Number(svg.dataset.window),samples=chartData(svg,'samples');
  if(!samples.length)return;
  const best=nearestHistoryPoint(samples,time);
  const cx=padLeft+(best.t-Number(svg.dataset.start))*plotW/Number(svg.dataset.window),cy=156-(best.v-Number(svg.dataset.low))*132/Number(svg.dataset.span),g=svg.querySelector('.hoverCursor');
  svg.dataset.cursorTime=best.t;g.style.display='block';g.querySelector('.cursor').setAttribute('x1',cx);g.querySelector('.cursor').setAttribute('x2',cx);g.querySelector('.cursorDot').setAttribute('cx',cx);g.querySelector('.cursorDot').setAttribute('cy',cy);
  chartReadout(svg,best.t,[{label:svg.dataset.label,v:best.v,color:'var(--brand)'}]);
  g.querySelector('.tooltip').textContent=chartTimestamp(svg,best.t)+' · '+best.v.toFixed(Number(svg.dataset.digits))+' '+svg.dataset.unit;
}
let chartGradSeq=0;
function chartSvg(samples,definition,bucketSeconds,continuitySeconds,windowSeconds=86400,filter='raw',end=Date.now(),loading=false){
  const now=end,windowMs=Math.max(1,windowSeconds)*1000,start=now-windowMs;
  samples=samples.filter(p=>Number.isFinite(p.t)&&Number.isFinite(p.v)&&p.t>=start&&p.t<=now).sort((a,b)=>a.t-b.t);
  if(!samples.length)return loading?'<div class="chartLoadingBox">Loading selected history …</div>':'<div class="muted chartEmpty">No measurements in the selected '+historyStepLabel(windowSeconds)+' window ('+esc(historyRangeLabel(start,now))+'). Sensor readings arrive every '+historyStepLabel(bucketSeconds)+'.</div>';
  let minV=Infinity,maxV=-Infinity,sumV=0;
  for(const p of samples){if(p.v<minV)minV=p.v;if(p.v>maxV)maxV=p.v;sumV+=p.v}
  const avgV=sumV/samples.length,scale=chartScale(samples.map(p=>p.v)),span=scale.high-scale.low,lay=chartLayout(scale,definition.digits),x=t=>lay.padLeft+(t-start)*lay.plotW/windowMs,y=v=>156-(v-scale.low)*132/span,maximumGap=Math.max(350,Number(continuitySeconds||bucketSeconds||1800)*1500),segments=chartSegments(samples,maximumGap),visible=samples.length>1200?new Set(chartReducePoints(samples)):null,simplified=samples.length>1200,gradId='cg'+(++chartGradSeq),grid=[];
  for(let v=scale.low;v<=scale.high+scale.step*.1;v+=scale.step)grid.push('<line class="grid" x1="'+lay.padLeft+'" x2="'+lay.right+'" y1="'+y(v)+'" y2="'+y(v)+'"/><text x="'+(lay.padLeft-8)+'" y="'+(y(v)+4)+'" text-anchor="end">'+v.toFixed(definition.digits)+'</text>');
  for(const f of lay.xFractions){
    const t=start+f*windowMs,gx=x(t),label=f===1&&now>=Date.now()-1500?'now':windowMs>86400000?new Date(t).toLocaleDateString([],{day:'2-digit',month:'2-digit',...(windowMs>=31536000000?{year:'2-digit'}:{})}):new Date(t).toLocaleTimeString([],{hour:'2-digit',minute:'2-digit',...(windowMs<=900000?{second:'2-digit'}:{})});
    grid.push('<line class="minor" x1="'+gx+'" y1="24" x2="'+gx+'" y2="156"/><text x="'+gx+'" y="177" text-anchor="'+(f===0?'start':f===1?'end':'middle')+'">'+esc(label)+'</text>');
  }
  const lines=segments.map(segment=>{
    const selected=visible?segment.filter(p=>visible.has(p)):segment;if(!selected.length)return '';
    const display=filter==='smooth'?chartDisplayPoints(segment,filter).filter((_,i)=>!visible||visible.has(segment[i])):selected;
    const reduced=chartReducePoints(display);
    const path=points=>points.map((p,i)=>(i?'L':'M')+x(p.t).toFixed(2)+','+y(p.v).toFixed(2)).join('');
    const area=reduced.length>1?'<path class="seriesArea" d="'+path(reduced)+'L'+x(reduced[reduced.length-1].t).toFixed(2)+',156L'+x(reduced[0].t).toFixed(2)+',156Z" fill="url(#'+gradId+')"/>':'';
    const raw=filter==='smooth'&&selected.length>1?'<path class="rawTrace" d="'+path(chartReducePoints(selected))+'" stroke="var(--brand)" fill="none" stroke-width="1"/>':'';
    const trace=reduced.length>1?'<path class="seriesTrace" d="'+path(reduced)+'" stroke="var(--brand)" fill="none" stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"/>':'';
    const marked=selected.length<=120?selected:selected.filter((p,i)=>i===0||i===selected.length-1);
    return area+raw+trace+marked.map(p=>'<circle class="sample sampleDot" cx="'+x(p.t)+'" cy="'+y(p.v)+'" r="'+(selected.length===1?4:2.6)+'" fill="var(--brand)"/>').join('');
  }).join(''),unit=definition.unit||'',u=unit?' '+esc(unit):'',d=definition.digits;
  const stats=`<span class="plotStats"><span class="plotStatPill">Min ${minV.toFixed(d)}${u}</span><span class="plotStatPill">Avg ${avgV.toFixed(d)}${u}</span><span class="plotStatPill">Max ${maxV.toFixed(d)}${u}</span><span>${samples.length} pt${samples.length===1?'':'s'}${simplified?' · simplified':''}</span></span>`;
  return '<div class="historyPlot"><div class="plotHeading"><b>'+esc(definition.label)+(unit?' <small class="muted">('+esc(unit)+')</small>':'')+'</b>'+stats+'</div><svg class="chart historySvg" viewBox="0 0 600 190" role="img" tabindex="0" aria-label="'+esc(definition.label)+' history for the selected time window" data-pad-left="'+lay.padLeft+'" data-plot-width="'+lay.plotW+'" data-digits="'+d+'" data-unit="'+esc(unit)+'" data-label="'+esc(definition.label)+'" '+chartSamplesAttribute(samples)+' data-start="'+start+'" data-end="'+end+'" data-window="'+windowMs+'" data-low="'+scale.low+'" data-span="'+span+'" data-filter="'+filter+'" onpointermove="chartPointerMove(event,chartMove)" onpointerdown="chartPointerDown(event,chartMove)" onpointerup="chartPointerUp(event,chartMove)" onpointercancel="delete this._chartGesture" onpointerleave="chartLeave(this)" onkeydown="chartKey(event,chartMove)"><defs><linearGradient id="'+gradId+'" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stop-color="var(--brand)" stop-opacity=".18"/><stop offset="100%" stop-color="var(--brand)" stop-opacity="0"/></linearGradient></defs>'+grid.join('')+lines+'<g class="hoverCursor" style="display:none"><line class="cursor" y1="24" y2="156"/><circle class="cursorDot" r="4"/><text class="tooltip cursorSummary" x="'+lay.padLeft+'" y="188"></text></g></svg><div class="chartReadout"><span class="muted">Hover, tap a point, or use arrow keys for exact sample values.</span></div></div>';
}
function channelOptions(selected=-1){const value=String(selected),local=config.thingSpeakChannels||[],localIds=new Set(local.map(p=>p.channelId));return `<option value="-1" ${value==='-1'?'selected':''}>Without ThingSpeak</option>`+local.map(p=>`<option value="${p.slot}" ${String(p.slot)===value?'selected':''}>Local: ${esc(p.name)} · ID ${p.channelId}</option>`).join('')+remoteChannels.filter(c=>!localIds.has(c.id)).map(c=>`<option value="r${c.id}" ${`r${c.id}`===value?'selected':''}>ThingSpeak: ${esc(c.name)} · ID ${c.id}</option>`).join('')}
function openSensorWizard(mac,start=0){wizardGeneration++;currentSensor=statusData.sensors.find(n=>n.mac===mac);if(!currentSensor)return;wizardStep=start;$('wizardTitle').textContent=currentSensor.provisioned?'Edit sensor':'Add new sensor';$('wizardSubtitle').textContent=`${currentSensor.mac} · PCB V${currentSensor.pcbVersion} · detected: ${sensorTypeName(currentSensor.sensorType)}`;$('wizardMac').value=currentSensor.mac;$('wizardName').value=currentSensor.name;const selectedType=!currentSensor.provisioned&&Number(currentSensor.configuredSensorType)===0&&Number(currentSensor.sensorType)>0?currentSensor.sensorType:currentSensor.configuredSensorType;$('wizardSensorType').value=String(selectedType??0);$('wizardTemperatureOffset').value=Number(currentSensor.temperatureOffsetC??0.466).toFixed(3);$('wizardInterval').innerHTML=intervalControl('sleepSeconds',!currentSensor.provisioned&&[3,4].includes(Number(selectedType))?1:currentSensor.sleepSeconds);delete $('wizardInterval').dataset.bound;$('batteryReferenceVoltage').value='';$('resetBatteryCalibration').checked=false;$('batteryCalibrationState').textContent=`Latest: ${currentSensor.batteryMv?metric(currentSensor.batteryMv/1000,3)+' V':'unavailable'} · factor ${metric(currentSensor.batteryCalibrationFactor??1,4)}`;$('iaqCalibrationState').textContent=iaqCalibrationText(currentSensor);$('wizardProfile').innerHTML=channelOptions(currentSensor.profileSlot);$('wizardUpload').checked=currentSensor.uploadEnabled;[['wizardTemperature','temperatureField'],['wizardHumidity','humidityField'],['wizardPressure','pressureField'],['wizardIaq','iaqField'],['wizardGasResistance','gasResistanceField'],['wizardBattery','batteryField']].forEach(([id,key])=>$(id).innerHTML=fieldOptions(currentSensor[key]));$('quickChannel').classList.remove('on');bindIntervals($('sensorModal'));updateSensorTypeHint();updateProfileHint();drawWizard();$('sensorModal').classList.add('on')}
function wizardEffectiveType(){const type=Number($('wizardSensorType').value);return type===0?Number(currentSensor?.sensorType||0):type}
function drawWizard(){const live=[3,4,5].includes(wizardEffectiveType()),existing=currentSensor&&currentSensor.provisioned;if(live&&wizardStep>0)wizardStep=0;document.querySelectorAll('.wizardStep').forEach((s,i)=>s.classList.toggle('on',i===wizardStep));$('wizardProgress').style.display=live?'none':'flex';$('wizardProgress').innerHTML=[0,1,2].map(i=>`<i class="${i<=wizardStep?'on':''}"></i>`).join('');$('resetSensor').style.display=existing&&wizardStep===0?'inline-flex':'none';$('deleteSensor').style.display=existing&&wizardStep===0?'inline-flex':'none';$('wizardBack').style.visibility=wizardStep?'visible':'hidden';$('wizardNext').textContent=live||wizardStep===2?(existing?'Save changes':'Add sensor'):wizardStep===1&&!$('wizardUpload').checked?(existing?'Save without cloud':'Add locally'):wizardStep===1?'Check fields':'Next'}function closeWizard(){wizardGeneration++;$('sensorModal').classList.remove('on');currentSensor=null}
function moveWizard(d){if(d>0&&wizardStep===0&&$('wizardName').value.trim()&&[3,4,5].includes(wizardEffectiveType())){saveSensorWizard();return}if(d>0&&wizardStep===0&&!$('wizardName').value.trim()){notify('Please enter a sensor name.',true);return}if(d>0&&wizardStep===1&&$('wizardUpload').checked&&Number($('wizardProfile').value)<0){notify('Select a channel for cloud upload.',true);return}if(d>0&&wizardStep===1&&!$('wizardUpload').checked){saveSensorWizard();return}if(d>0&&wizardStep===2){saveSensorWizard();return}wizardStep=Math.max(0,Math.min(2,wizardStep+d));drawWizard()}
function updateProfileHint(){const choice=$('wizardProfile').value;if(choice.startsWith('r')){const c=remoteById(choice.slice(1));$('wizardUpload').disabled=!c||!c.writeApiKey;$('profileHint').textContent=c&&c.writeApiKey?`Cloud channel ${c.id} will be imported when you save. Write API Key: ${c.writeApiKey}`:'This cloud channel has no available Write API Key.';if(!c||!c.writeApiKey)$('wizardUpload').checked=false;return}const slot=Number(choice),p=(config.thingSpeakChannels||[]).find(x=>x.slot===slot);if(!p){$('profileHint').textContent=remoteChannels.length?'Select a local channel or one loaded directly from ThingSpeak.':'The sensor remains local. With a saved User API Key, your account channels are listed here too.';$('wizardUpload').checked=false;$('wizardUpload').disabled=true;return}$('wizardUpload').disabled=!p.writeApiKey;$('profileHint').textContent=p.writeApiKey?`Channel ${p.channelId} · Write API Key ${p.writeApiKey}`:`Channel ${p.channelId} has no Write API Key, so upload is unavailable.`;if(!p.writeApiKey)$('wizardUpload').checked=false}
function supportedThingSpeakFields(t){if([2,7].includes(t))return new Set(['temperature','humidity','pressure','iaq','gas','battery']);if(t===1)return new Set(['temperature','humidity','pressure','battery']);if(t===6)return new Set(['temperature','pressure','battery']);if([8,9,10,255].includes(t))return new Set(['battery']);return new Set(['temperature','humidity','pressure','iaq','gas','battery'])}
function updateSensorTypeHint(){const type=Number($('wizardSensorType').value),actual=type===0?Number(currentSensor?.sensorType):type,isBme680=[2,7].includes(actual),hint=$('sensorTypeHint');$('wizardTemperatureOffset').disabled=!isBme680;$('wizardTempOffsetGroup').style.display=isBme680?'':'none';$('iaqCalibrationGuide').style.display=isBme680?'':'none';$('wizardIntervalGroup').style.display=actual===5?'none':'';$('wizardInterval').style.display=actual===5?'none':'';const preset=$('wizardInterval').querySelector('.intervalPreset'),hidden=$('wizardInterval').querySelector('.intervalSeconds');if(preset){for(const opt of preset.options){const v=Number(opt.value);const blocked=isBme680&&opt.value!=='custom'&&(v<300||v%300!==0);opt.disabled=blocked;opt.hidden=blocked}if(isBme680&&hidden&&(Number(hidden.value)<300||Number(hidden.value)%300!==0)){preset.value='300';preset.dispatchEvent(new Event('change'))}}const allowed=supportedThingSpeakFields(actual);document.querySelectorAll('#sensorModal [data-ts-field]').forEach(row=>{const ok=allowed.has(row.dataset.tsField);row.style.display=ok?'':'none';if(!ok){const sel=row.querySelector('select');if(sel)sel.value='0'}});hint.textContent=actual===10?'AS7341: 10-channel optical spectrum (415–680 nm, Clear, NIR) at 0x39. Powers down between intervals.':actual===9?'TOF400C-VL53L1X: 940 nm laser distance (up to 4 m, Long Mode) at 0x29. Remove protective lens film.':actual===8?'CCS811: eCO₂ (400–8192 ppm) & TVOC (0–1187 ppb) at 0x5A/0x5B. Allow 20 min warm-up on cold start.':actual===7?'BME688: BSEC2 Static IAQ (5-min ULP cadence) + climate & gas at 0x76/0x77. Interval must be a 5-min multiple.':actual===6?'BMP280: forced temperature & pressure at 0x76/0x77 (no humidity). Rail off between intervals.':actual===4?'TMP117: interval controls normal temperature snapshots. SW2 starts continuous recording with 8-conversion averaging (124 ms typical) and 1 s reports.':actual===5?'MAX30102: press SW2 on sensor to start/stop & sync. Recording uses 100 Hz red/IR acquisition, 25 Hz waveform in 200-ms packets and 1 s experimental pulse estimates.':actual===3?'LSM6DSOX: interval controls normal snapshots. SW2 starts continuous 104 Hz FIFO (±4 g, ±500 °/s) with 50-ms means for 5 min, then 100-ms means; peaks retained.':type===2?'BME680: Bosch BSEC Static IAQ in 5-min ULP mode; radio & ThingSpeak follow the configured 5-min multiple interval.':type===1?'BME280: forced climate measurement at the reporting interval, then sensor rail powers off.':type===255?'Battery-only node: I²C sensor path stays off.':'Auto-detect identifies BME280, BMP280, BME680, BME688, LSM6DSOX, TMP117, MAX30102, AS7341, VL53L1X and CCS811 at startup.';drawWizard()}
async function saveSensorWizard(){
  if(sensorSaveActive||!currentSensor)return;
  const effectiveType=wizardEffectiveType(),allowed=supportedThingSpeakFields(effectiveType);
  document.querySelectorAll('#sensorModal [data-ts-field]').forEach(row=>{if(!allowed.has(row.dataset.tsField)){const sel=row.querySelector('select');if(sel)sel.value='0'}});
  const fields=['wizardTemperature','wizardHumidity','wizardPressure','wizardIaq','wizardGasResistance','wizardBattery'].map(id=>Number($(id).value)).filter(Boolean);
  if(new Set(fields).size!==fields.length){notify('Each ThingSpeak field can only be used once.',true);return}
  const seconds=Number($('wizardInterval').querySelector('.intervalSeconds').value);
  if([2,7].includes(effectiveType)&&(seconds<300||seconds%300!==0)){notify('The BME680/BME688 interval must be 5 minutes or a multiple of 5 minutes.',true);return}
  const generation=wizardGeneration,choice=$('wizardProfile').value,body=new URLSearchParams(new FormData($('sensorWizardForm')));
  if([3,4,5].includes(effectiveType)){if(effectiveType===5)body.set('sleepSeconds','1');body.delete('uploadEnabled')}
  sensorSaveActive=true;$('wizardNext').disabled=true;
  try{
    if(choice.startsWith('r')){const slot=await importRemoteChannel(Number(choice.slice(1)),true);body.set('profileSlot',String(slot));if(generation===wizardGeneration)$('wizardProfile').value=String(slot)}
    const j=await apiPost('/api/sensor',body);notify(j.message);if(generation===wizardGeneration)closeWizard();await loadStatus();
  }catch(e){notify(e.message,true)}
  finally{sensorSaveActive=false;$('wizardNext').disabled=false}
}

async function saveQuickChannel(){const previous=new Set((config.thingSpeakChannels||[]).map(p=>p.slot)),body=new URLSearchParams({name:$('quickName').value,channelId:$('quickId').value,readApiKey:$('quickRead').value,writeApiKey:$('quickWrite').value});try{const j=await apiPost('/api/channel-profile',body);notify(j.message);await loadConfig();const added=(config.thingSpeakChannels||[]).find(p=>!previous.has(p.slot));$('wizardProfile').innerHTML=channelOptions(added?added.slot:-1);updateProfileHint();$('quickChannel').classList.remove('on')}catch(e){notify(e.message,true)}}
async function autoCreateChannel(){const button=$('autoCreateChannel');button.disabled=true;try{const j=await thingSpeakMutation('/api/create-channel',new URLSearchParams({mac:$('wizardMac').value,name:$('wizardName').value.trim()}));notify(j.message);await loadConfig();if(config.userApiKey)await loadRemoteChannels(true);$('wizardProfile').innerHTML=channelOptions(Number(j.slot));$('wizardUpload').checked=true;const chosen=Number($('wizardSensorType').value),effective=chosen===0?Number(currentSensor?.sensorType):chosen,boschGas=[2,7].includes(effective);if(effective===6)$('wizardHumidity').value='0';$('wizardIaq').value=boschGas?'4':'0';$('wizardBattery').value='5';$('wizardGasResistance').value=boschGas?'6':'0';updateProfileHint();$('quickChannel').classList.remove('on')}catch(e){notify(e.message,true)}button.disabled=false}
async function resetCurrentSensor(){if(!currentSensor||!confirm('Reset this sensor to factory settings? The command will be sent at its next check-in.'))return;try{const j=await apiPost('/api/reset-sensor',new URLSearchParams({mac:currentSensor.mac}));notify(j.message);closeWizard();await loadStatus()}catch(e){notify(e.message,true)}}
async function resetCurrentIaqCalibration(){if(!currentSensor||!confirm('Clear the learned BME680/BME688 IAQ state? Accuracy will restart at zero.'))return;try{const j=await apiPost('/api/reset-sensor-iaq',new URLSearchParams({mac:currentSensor.mac}));notify(j.message);$('iaqCalibrationState').textContent='Calibration restart queued – it will be sent at the next sensor check-in.'}catch(e){notify(e.message,true)}}
async function deleteCurrentSensor(){if(!currentSensor||!confirm('Remove this sensor and its local history from the station? If it remains powered, it will appear as a new sensor at its next check-in.'))return;const mac=currentSensor.mac;try{const j=await apiPost('/api/delete-sensor',new URLSearchParams({mac}));delete stationHistory[mac];delete historyRevisions[mac];delete historyBuckets[mac];delete historyPartial[mac];delete historyRanges[mac];delete chartMetrics[mac];delete chartWindows[mac];delete chartEnds[mac];delete chartFilters[mac];delete feedbackSessions[mac];delete recoveryMeasurements[mac];const v=recordingViews[mac];if(v){v.controller?.abort();v.exportController?.abort();v.request=(v.request||0)+1;delete recordingViews[mac]}sensorRenderSignature='';notify(j.message);closeWizard();await loadStatus()}catch(e){notify(e.message,true)}}
const wait=ms=>new Promise(r=>setTimeout(r,ms));async function reconnectDashboard(target='/dashboard'){const overlay=$('restartOverlay'),status=$('restartStatus'),retry=$('retryRestart'),previousUptime=statusData.uptimeMs||Number.MAX_SAFE_INTEGER;retry.dataset.target=target;retry.style.display='none';overlay.classList.add('on');status.textContent='Applying settings. W-Charger is restarting …';await wait(3500);const started=Date.now();while(Date.now()-started<60000){status.textContent=`Looking for station automatically … ${Math.floor((Date.now()-started)/1000)+1} s`;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),4000);try{const r=await fetch('/api/status?reconnect='+Date.now(),{cache:'no-store',signal:controller.signal});if(r.status===401){status.textContent='Station is protected. Opening sign-in …';await wait(400);location.replace('/login');return}if(r.ok){const s=await r.json();if(s.uptimeMs<previousUptime||Date.now()-started>5000){status.textContent='W-Charger is reachable again. Reloading page …';await wait(500);location.replace(target);return}}}catch(e){}finally{clearTimeout(timer)}await wait(1200)}status.textContent='W-Charger is not reachable yet. Check the Wi-Fi connection and try again.';retry.style.display='inline-flex'}
async function signOut(){const button=$('logoutLink');button.disabled=true;try{await apiPost('/logout',new URLSearchParams());location.replace('/login')}catch(e){button.disabled=false;notify(e.message,true)}}
function bindForms(){document.querySelectorAll('form[data-api]').forEach(f=>{if(f.dataset.bound)return;f.addEventListener('submit',submitApi);f.dataset.bound='1'})}async function submitApi(e){e.preventDefault();const f=e.currentTarget,b=e.submitter||f.querySelector('[type=submit]');if(f.dataset.api==='/api/factory-reset'&&!confirm('Delete all local W-Charger settings? ThingSpeak channels remain unchanged.'))return;if(f.id==='settingsForm'){const password=$('newAdminPassword').value,confirmation=$('confirmAdminPasswordSettings').value,remove=$('removeAdminPassword').checked;if(remove&&password){notify('Choose either a new password or removal, not both.',true);return}if(password&&password.length<8){notify('The website password must contain at least 8 characters.',true);return}if(password!==confirmation){notify('The new website passwords do not match.',true);return}}if(b)b.disabled=true;try{const body=new URLSearchParams(new FormData(f));if(e.submitter?.dataset.restart==='1')body.set('restartNow','1');const j=await apiPost(f.dataset.api,body);notify(j.message);if(j.restart){reconnectDashboard(f.dataset.api==='/api/factory-reset'?'/':'/dashboard');return}if(j.authenticationChanged){location.replace('/login');return}await loadConfig();await loadStatus();if(f.dataset.api==='/api/station'&&config.userApiKey)await loadRemoteChannels(false)}catch(err){notify(err.message,true)}if(b)b.disabled=false}
async function scanSettingsWifi(){$('wifiScanButton').disabled=true;$('wifiScanButton').textContent='Scanning …';try{let data=await requestJson('/api/wifi-scan?refresh=1');for(let attempt=0;data.scanning&&attempt<20;attempt++){await wait(400);data=await requestJson('/api/wifi-scan')}setSsidOptions(data.networks||[],config.wifiSsid)}catch(e){notify('Wi-Fi scan failed.',true)}$('wifiScanButton').disabled=false;$('wifiScanButton').textContent='Search networks again'}function setSsidOptions(list,current){const names=[...new Set([current,...list.map(x=>x.ssid)].filter(Boolean))];$('settingsSsid').innerHTML=names.map(s=>`<option value="${esc(s)}" ${s===current?'selected':''}>${esc(s)}</option>`).join('')||'<option value="">No networks found</option>'}
$('addSensorButton').onclick=()=>$('pendingPanel').classList.add('on');$('closePending').onclick=()=>$('pendingPanel').classList.remove('on');$('wifiScanButton').onclick=scanSettingsWifi;$('loadCloudChannels').onclick=()=>loadRemoteChannels(false);$('cloudCreateForm').onsubmit=createRemoteChannel;$('retryRestart').onclick=()=>reconnectDashboard($('retryRestart').dataset.target||'/dashboard');$('closeWizard').onclick=closeWizard;$('resetSensor').onclick=resetCurrentSensor;$('resetIaqCalibration').onclick=resetCurrentIaqCalibration;$('deleteSensor').onclick=deleteCurrentSensor;$('wizardBack').onclick=()=>moveWizard(-1);$('wizardNext').onclick=()=>moveWizard(1);$('wizardProfile').onchange=()=>{updateProfileHint();drawWizard()};$('wizardUpload').onchange=drawWizard;$('wizardSensorType').onchange=updateSensorTypeHint;$('showQuickChannel').onclick=()=>$('quickChannel').classList.toggle('on');$('saveQuickChannel').onclick=saveQuickChannel;$('autoCreateChannel').onclick=autoCreateChannel;$('removeAdminPassword').onchange=()=>{const remove=$('removeAdminPassword').checked;$('newAdminPassword').disabled=remove;$('confirmAdminPasswordSettings').disabled=remove;if(remove){$('newAdminPassword').value='';$('confirmAdminPasswordSettings').value=''}};$('sensorModal').onclick=e=>{if(e.target===$('sensorModal'))closeWizard()};const goHome=openDashboardHome;$('homeLogo').onclick=goHome;$('homeLogo').onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();goHome()}};
let firmwareVersions=[];function firmwareLabel(mac){const n=firmwareVersions.find(n=>n.mac===mac);return n?`Software ${n.version} · build ${n.build}`:'Software not reported / OTA base image required'}
let selectedSensorMac=null,dashboardDraft=null,showHiddenSensors=false,historyLoadController=null,historyLoadSerial=0,historyPrefetch=null;
let firmwareRequestActive=false;async function loadFirmwareVersions(){if(recordingTransfers||firmwareRequestActive||statusRequestActive||historyLoadController)return;firmwareRequestActive=true;try{const o=await requestJson('/api/ota');firmwareVersions=o.nodes||[];for(const el of document.querySelectorAll('[data-firmware]')){const n=(o.nodes||[]).find(n=>n.mac===el.dataset.firmware);el.textContent=n?`Software ${n.version} · build ${n.build}`:'Software not reported / OTA base image required'}}catch(e){}finally{firmwareRequestActive=false}}
document.getElementById('logoutLink').insertAdjacentHTML('beforebegin','<a class="navLink" href="/updates" aria-label="Firmware updates">📦 <span>Firmware updates</span></a>');
loadFirmwareVersions();
setInterval(()=>{if(!document.hidden)loadFirmwareVersions()},15000);
const metricFocus={};
const historyMetadata={},historyLoadErrors={},historySources={},historyPartial={},historyRanges={},historyLoading={},readingsExpanded={},deviceExpanded={},historyPayloads=new Map();let historyPayloadSequence=0;
function dashboardPreferences(){return {order:[],hidden:[],columns:0,comfortable:false,...config.dashboard};}
function orderedSensors(list){const prefs=dashboardPreferences(),positions=new Map(prefs.order.map((mac,i)=>[mac,i]));return [...list].sort((a,b)=>(positions.get(a.mac)??1000)-(positions.get(b.mac)??1000));}
function dashboardChrome(){}
function openSensorDetail(mac,metricKey){
  if(selectedSensorMac===mac&&metricKey){
    metricFocus[mac]=true;chartMetrics[mac]=metricKey;
    renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));
    return;
  }
  historyLoadController?.abort();historyLoadController=null;historyLoadSerial++;if(selectedSensorMac)delete historyLoading[selectedSensorMac];selectedSensorMac=mac;
  metricFocus[mac]=Boolean(metricKey);if(metricKey)chartMetrics[mac]=metricKey;
  $('dashboardLayout')?.setAttribute('hidden','');
  renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));
  syncHistories(statusData.sensors||[]).then(()=>{if(selectedSensorMac===mac)renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));});
  syncRecordingLists((statusData.sensors||[]).filter(n=>n.mac===mac&&n.provisioned));
  if(window.scrollY>80)window.scrollTo({top:0,behavior:'instant'});
}
function closeSensorDetail(){historyLoadController?.abort();historyLoadController=null;historyLoadSerial++;if(selectedSensorMac)delete historyLoading[selectedSensorMac];selectedSensorMac=null;renderSensors((statusData.sensors||[]).filter(n=>n.provisioned));if(window.scrollY>80)window.scrollTo({top:0,behavior:'instant'});}
function openDashboardHome(){history.replaceState(null,'',location.pathname);showView('overview');cancelDashboardLayout();closeSensorDetail();}
function sensorCardClick(event,mac){if(!event.target.closest('button,a,input,select,summary'))openSensorDetail(mac);}
function openMetricChart(mac,key){openSensorDetail(mac,key);}
function overviewSensor(source){
  const n=sensorForDisplay(source),battery=n.batteryMv?metric(n.batteryMv/1000,3):'–',isBoschGas=[2,7].includes(Number(n.sensorType));
  let values=n.sensorType===3?motionTiles(n):n.sensorType===5?pulseTiles(n):n.sensorType===8?ccs811Tiles(n):n.sensorType===9?tofTiles(n):n.sensorType===10?spectralTiles(n):'';
  values+=valueTile(n,'Temperature',metric(n.temperature,n.sensorType===4?3:1),'°C','temperatureField')+valueTile(n,'Humidity',metric(n.humidity),'%','humidityField')+valueTile(n,'Pressure',metric(n.pressure,0),'hPa','pressureField');
  if(isBoschGas)values+=iaqTile(n,Number(n.iaqAccuracy)?metric(n.iaq,0):'–')+valueTile(n,'Gas resistance',n.gasResistanceOhms?metric(n.gasResistanceOhms/1000,1):'–','kΩ','gasResistanceField');
  values+=valueTile(n,'Battery',battery,'V','batteryField');
  const flags=Number(n.telemetryFlags||0),state=flags&256?'Battery protection active':n.configPending?'Settings pending':flags&1?'Measurement unavailable':n.seen?age(n.ageMs):'Waiting for sensor',intervalText=Number(n.sensorType)===5?'Manual (SW2)':`${historyStepLabel(n.sleepSeconds||1)} interval`;
  return `<article class="card sensor sensorSummary" tabindex="0" onkeydown="if(event.target===this&&(event.key==='Enter'||event.key===' ')){event.preventDefault();openSensorDetail(this.dataset.uiKey)}" data-ui-key="${esc(n.mac)}" onclick="sensorCardClick(event,'${esc(n.mac)}')"><div class="sensorTop"><button type="button" class="sensorName" onclick="openSensorDetail('${esc(n.mac)}')"><i class="${n.online?'fresh':'fresh stale'}" aria-hidden="true"></i><span class="sensorNameText"><h3>${esc(n.name)}</h3><span class="muted">${sensorTypeName(n.sensorType)}</span></span></button><button type="button" class="btn secondary sensorOpen" aria-label="Open ${esc(n.name)}" onclick="openSensorDetail('${esc(n.mac)}')">›</button></div><div class="values">${values}</div><div class="meta"><span>${esc(state)}</span><span>${esc(intervalText)}</span></div></article>`;
}
function decorateSensorDetail(html){
  const container=document.createElement('div');container.innerHTML=html;
  const article=container.querySelector('.sensor');if(!article)return html;
  const mac=selectedSensorMac,values=article.querySelector(':scope > .values');
  if(values){const details=document.createElement('details');details.className='currentReadings';details.dataset.uiKey='current-readings';details.open=readingsExpanded[mac]??matchMedia('(min-width:700px)').matches;details.innerHTML='<summary>Current readings</summary>';values.before(details);details.append(values);}
  const info=document.createElement('details');info.className='deviceInformation';info.dataset.uiKey='device-information';info.open=Boolean(deviceExpanded[mac]);info.innerHTML='<summary>Device details</summary><div class="deviceContent"></div>';const content=info.lastElementChild;
  for(const node of [...article.children])if(node.matches('.helper.liveTiming'))content.append(node);
  const foot=article.querySelector('.sensorFoot'),cloud=foot?.querySelector(':scope > [data-ui-key="cloud-tag"]');
  if(cloud){if((statusData.sensors||[]).find(n=>n.mac===mac)?.uploadEnabled)content.append(cloud);else cloud.remove();}
  if(!content.children.length){const sensor=(statusData.sensors||[]).find(n=>n.mac===mac);content.innerHTML=`<span class="muted" style="font-size:12px">PCB V${sensor?.pcbVersion||4} · ${sensor?.online?'Online':'Sleeping / offline'} · ${esc(sensorModeInfo(sensor).detail)}</span>`;}
  article.append(info);return container.innerHTML;
}
$('sensorGrid').addEventListener('toggle',event=>{
  if(!selectedSensorMac)return;
  if(event.target.matches('.currentReadings'))readingsExpanded[selectedSensorMac]=event.target.open;
  if(event.target.matches('.deviceInformation'))deviceExpanded[selectedSensorMac]=event.target.open;
},true);
function renderSensors(list){
  dashboardChrome();const ordered=orderedSensors(list),prefs=dashboardPreferences();
  if(selectedSensorMac&&!ordered.some(n=>n.mac===selectedSensorMac))selectedSensorMac=null;
  const visible=ordered.filter(n=>!prefs.hidden.includes(n.mac)),hiddenCount=ordered.length-visible.length;
  $('dashboardTitle').textContent=selectedSensorMac?'Sensor details':'Your sensors';
  $('visibleSensorCount').textContent=visible.length+' visible'+(hiddenCount?' · '+hiddenCount+' hidden':'');
  const toggleBtn=$('toggleHiddenBtn');if(toggleBtn){toggleBtn.hidden=hiddenCount===0;toggleBtn.textContent=showHiddenSensors?'Hide hidden':'Show hidden';}
  $('sensorDetailBack').hidden=!selectedSensorMac;$('dashboardTools').hidden=Boolean(selectedSensorMac);
  $('sensorGrid').className='sensorGrid'+(selectedSensorMac?' detailGrid':' overviewGrid')+(prefs.comfortable?' comfortable':'');
  $('sensorGrid').dataset.columns=String(prefs.columns||0);
  if(selectedSensorMac){renderFullSensors(ordered.filter(n=>n.mac===selectedSensorMac));}
  else{updateSensorGrid((showHiddenSensors?ordered:visible).map(overviewSensor).join('')||'<div class="card empty">No visible sensors. Open Arrange to show sensors.</div>');}
}
function openDashboardLayout(){
  dashboardDraft=structuredClone(dashboardPreferences());dashboardDraft.order=orderedSensors((statusData.sensors||[]).filter(n=>n.provisioned)).map(n=>n.mac);drawDashboardLayout();
}
function drawDashboardLayout(){
  const panel=$('dashboardLayout');panel.hidden=false;const d=dashboardDraft;
  d.order=d.order.filter(mac=>(statusData.sensors||[]).some(n=>n.mac===mac&&n.provisioned));
  const live=(statusData.sensors||[]).some(n=>n.live&&n.provisioned),refreshOptions=live?`<option>Automatic · ${livePollMs()} ms</option>`:[1,2,5,10].map(v=>`<option value="${v}" ${v===dashboardRefreshSeconds?'selected':''}>${v} second${v===1?'':'s'}</option>`).join('');
  panel.innerHTML='<h2>Arrange dashboard</h2><div class="row"><label>Columns<select id="layoutColumns">'+[[0,'Automatic'],[1,'1 column'],[2,'2 columns'],[3,'3 columns']].map(([v,label])=>`<option value="${v}" ${Number(d.columns)===v?'selected':''}>${label}</option>`).join('')+'</select></label><label>Spacing<select id="layoutComfort"><option value="0">Compact</option><option value="1" '+(d.comfortable?'selected':'')+'>Comfortable</option></select></label></div><label class="dashboardRefreshControl">Page refresh<select id="dashboardRefresh" aria-label="Dashboard refresh interval" '+(live?'disabled':'')+' onchange="setDashboardRefresh(this.value)">'+refreshOptions+'</select></label><div class="layoutRows">'+d.order.map((mac,i)=>{const n=statusData.sensors.find(n=>n.mac===mac);if(!n)return '';return `<div class="layoutRow" draggable="true" data-layout-mac="${esc(mac)}"><span class="layoutName">${esc(n.name)}</span><button type="button" class="btn secondary" aria-label="Move ${esc(n.name)} up" onclick="moveDashboardSensor('${esc(mac)}',-1)" ${i?'':'disabled'}>↑</button><button type="button" class="btn secondary" aria-label="Move ${esc(n.name)} down" onclick="moveDashboardSensor('${esc(mac)}',1)" ${i===d.order.length-1?'disabled':''}>↓</button><label class="layoutVisibility"><input type="checkbox" aria-label="Show ${esc(n.name)}" data-visible-mac="${esc(mac)}" ${d.hidden.includes(mac)?'':'checked'}></label></div>`;}).join('')+'</div><p class="muted" style="font-size:12px">Hidden sensors continue measuring and saving data. Layout is shared on this station.</p><div class="actionRow"><button type="button" class="btn secondary" onclick="cancelDashboardLayout()">Cancel</button><button type="button" class="btn" id="saveDashboardLayoutButton" onclick="saveDashboardLayout()">Save layout</button></div>';
  panel.querySelectorAll('[data-visible-mac]').forEach(c=>c.onchange=()=>{d.hidden=d.hidden.filter(mac=>mac!==c.dataset.visibleMac);if(!c.checked)d.hidden.push(c.dataset.visibleMac);});
  $('layoutColumns').onchange=()=>d.columns=Number($('layoutColumns').value);$('layoutComfort').onchange=()=>d.comfortable=$('layoutComfort').value==='1';
  let dragged=null;panel.querySelectorAll('[data-layout-mac]').forEach(row=>{row.ondragstart=e=>{if(e.target.closest('button,input,select')){e.preventDefault();return;}dragged=row.dataset.layoutMac;e.dataTransfer.setData('text/plain',dragged);};row.ondragover=e=>e.preventDefault();row.ondrop=e=>{e.preventDefault();if(!dragged)return;const from=d.order.indexOf(dragged),to=d.order.indexOf(row.dataset.layoutMac);d.order.splice(from,1);d.order.splice(to,0,dragged);drawDashboardLayout();};});
}
function moveDashboardSensor(mac,step){const order=dashboardDraft.order,i=order.indexOf(mac),j=Math.max(0,Math.min(order.length-1,i+step));[order[i],order[j]]=[order[j],order[i]];drawDashboardLayout();}
function cancelDashboardLayout(){dashboardDraft=null;$('dashboardLayout').hidden=true;}
async function saveDashboardLayout(){
  const button=$('saveDashboardLayoutButton');if(button.disabled)return;button.disabled=true;
  const draft=dashboardDraft,submitted=structuredClone(draft);
  try{await apiPost('/api/dashboard-layout',new URLSearchParams({columns:String(submitted.columns),comfortable:submitted.comfortable?'1':'0',order:submitted.order.join(','),hidden:submitted.hidden.filter(mac=>submitted.order.includes(mac)).join(',')}));config.dashboard=submitted;if(dashboardDraft===draft)cancelDashboardLayout();renderSensors(statusData.sensors.filter(n=>n.provisioned));notify('Dashboard layout saved.');}
  catch(error){notify(error.message,true);button.disabled=false;}
}
const historyColumnNames=new Set(['t','sensorType','pcbVersion','iaqAccuracy','gap','temperature','humidity','pressure','iaq','gas','battery','ax','ay','az','gx','gy','gz','peakAcceleration','peakAngularRate','eco2','tvoc','distanceMm','signalRate','ambientRate','rangeStatus','spectral','spectralGain','spectralIntMs']);
function decodeHistoryPage(data){
  if(!Array.isArray(data.points))throw new Error('History page is invalid');
  if(data.format!=='array-v1'){
    if(data.format||data.points.some(point=>!point||typeof point!=='object'||Array.isArray(point)))throw new Error('History page format is invalid');
    return data.points.map(p=>Array.isArray(p.spectral)?{...p,spectralClear:p.spectral[8],spectralNir:p.spectral[9]}:p);
  }
  const fields=data.fields;
  if(!Array.isArray(fields)||!fields.includes('t')||!fields.includes('sensorType')||new Set(fields).size!==fields.length||fields.some(field=>!historyColumnNames.has(field)))throw new Error('History columns are invalid');
  return data.points.map(values=>{
    if(!Array.isArray(values)||values.length!==fields.length)throw new Error('History row is invalid');
    const point={};for(let i=0;i<fields.length;i++)if(values[i]!==null)point[fields[i]]=values[i];
    if(Array.isArray(point.spectral)){point.spectralClear=point.spectral[8];point.spectralNir=point.spectral[9];}
    if(!Number.isFinite(point.t))throw new Error('History timestamp is invalid');return point;
  });
}
function prefetchRecentHistory(n){
  const mac=n.mac,interval=Number(n.history?.intervalSeconds||n.sleepSeconds||0),count=Number(n.history?.points||0),oldest=Number(n.history?.oldest||0);
  const estimated=Math.min(count,Math.ceil(604800/Math.max(1,interval)));
  if(!interval||!count||estimated>1536||!oldest||oldest>=Math.floor((Date.now()-86400000)/1000))return;
  setTimeout(()=>{
    const current=(statusData.sensors||[]).find(sensor=>sensor.mac===mac),desired=chartWindows[mac]??86400,range=historyRanges[mac];
    const after=Math.max(0,Math.floor((Date.now()-604800000)/1000)-1);
    if(document.hidden||recordingTransfers||(statusData.sensors||[]).some(sensor=>sensor.provisioned&&sensor.live)||selectedSensorMac!==mac||!current?.normalHistory||chartEnds[mac]!=null||desired==='all'||Number(desired)>86400||historyLoadController||!range||range.full||range.after<=after)return;
    const promise=syncHistories(statusData.sensors||[],{prefetch:true});
    const state={mac,controller:historyLoadController};historyPrefetch=state;
    promise.finally(()=>{if(historyPrefetch===state)historyPrefetch=null;if(selectedSensorMac===mac)renderSensors((statusData.sensors||[]).filter(sensor=>sensor.provisioned));});
  },0);
}
async function syncHistories(sensors,options={}){
  const selected=sensors.filter(n=>n.mac===selectedSensorMac&&n.provisioned);if(!selected.length)return;
  const n=selected[0],source=n.normalHistory?'normal':n.live?'live':'legacy';
  if(historySources[n.mac]&&historySources[n.mac]!==source){historyLoadController?.abort();historyLoadController=null;historyLoadSerial++;delete stationHistory[n.mac];delete historyRevisions[n.mac];delete historyMetadata[n.mac];delete historyPartial[n.mac];delete historyRanges[n.mac];}
  historySources[n.mac]=source;
  if(!n.normalHistory){await syncLegacyHistories(selected,++historyLoadSerial);return;}
  const revision=Number(n.historyRevision||0),desired=options.full?'all':options.prefetch?604800:(chartWindows[n.mac]??86400),end=chartEnds[n.mac]??Date.now();
  const until=options.full||chartEnds[n.mac]==null?0:Math.floor(end/1000);
  const after=desired==='all'?0:Math.max(0,Math.floor((end-Number(desired)*1000)/1000)-1);
  const key=desired+':'+(until||'live'),range=historyRanges[n.mac],previous=stationHistory[n.mac]||[];
  const covered=Boolean(range&&(range.full||range.key===key||(after>=range.after&&((!until&&range.live)||(until&&until<=range.until)))));
  if(covered&&historyRevisions[n.mac]===revision&&(!options.full||range.full))return;
  if(historyLoadController)return;
  const controller=new AbortController(),serial=++historyLoadSerial;historyLoadController=controller;let timedOut=false,retryForeground=false;
  const stillSelected=()=>serial===historyLoadSerial&&selectedSensorMac===n.mac&&!controller.signal.aborted;
  try{
    const incremental=Boolean(!options.full&&!until&&range&&range.live&&after>=range.after&&previous.length&&revision>=Number(historyRevisions[n.mac]||0)&&Number(n.history?.oldest||0)<=Math.floor(previous.at(-1).t/1000));
    // A wider current window needs only the missing older prefix (plus any new
    // tail if a measurement arrived). Require the oldest retained page to match.
    const expanding=Boolean(!options.full&&!until&&range&&range.live&&!range.full&&after<range.after&&range.after>0&&previous.length&&range.oldest>0&&Number(n.history?.oldest)===range.oldest);
    if(!options.prefetch)historyLoading[n.mac]=true;if(!range||historyRevisions[n.mac]!==revision)historyPartial[n.mac]=true;delete historyLoadErrors[n.mac];
    let pages=0;
    async function readRange(startAfter,startUntil,startOldest){
      let after=startAfter,until=startUntil,oldest=startOldest,points=[],data;
      do{
        const timer=setTimeout(()=>{timedOut=true;controller.abort();},15000);
        try{const response=await fetch('/api/history?mac='+encodeURIComponent(n.mac)+'&source=normal&after='+after+'&until='+until+'&oldest='+oldest+'&limit=512&format=array-v1',{cache:'no-store',signal:controller.signal});if(response.status===401){location.replace('/login');throw new Error('Authentication required')}if(!response.ok)throw new Error('History request failed');data=await response.json();if(data.success===false)throw new Error(data.message||'History is unavailable');}
        finally{clearTimeout(timer);}
        const pagePoints=decodeHistoryPage(data);
        const nextUntil=Number(data.until||0),nextOldest=Number(data.oldest||0);
        if((until&&nextUntil!==until)||(oldest&&nextOldest!==oldest))throw new Error('History snapshot changed while loading');
        points.push(...pagePoints);until=nextUntil;oldest=nextOldest;
        const next=Number(data.nextAfter||0);if(!Number.isSafeInteger(next)||next<0||(next&&next<=after))throw new Error('History cursor did not advance');after=next;
        if(++pages>1024)throw new Error('History exceeded its bounded storage capacity');
        if(options.prefetch&&pages>=3&&after){const error=new Error('Background history limit reached');error.name='HistoryPrefetchLimit';throw error;}
        if(options.prefetch&&after&&Number(chartWindows[n.mac])!==604800)await new Promise(resolve=>setTimeout(resolve,100));
      }while(after&&!controller.signal.aborted);
      return {points,until,oldest,data};
    }
    let result=await readRange(incremental?Math.floor(previous.at(-1).t/1000):after,expanding?range.after:until,expanding?range.oldest:0);if(!stillSelected())return;
    let tailPoints=[],finalUntil=expanding?range.until:result.until;
    if(expanding&&historyRevisions[n.mac]!=null&&revision!==historyRevisions[n.mac]){
      const tail=await readRange(Math.floor(previous.at(-1).t/1000),0,result.oldest);if(!stillSelected())return;
      tailPoints=tail.points;finalUntil=tail.until;result.data=tail.data;
    }
    const retainedSpan=desired==='all'?0:Math.max(Number(desired)||0,604800);
    const maxRetainedAfter=retainedSpan?Math.max(0,Math.floor((end-retainedSpan*1000)/1000)-1):0;
    const cachedAfter=incremental?(range.full?range.after:Math.max(range.after,maxRetainedAfter)):after;
    stationHistory[n.mac]=[...new Map([...((incremental||expanding)?previous:[]),...result.points,...tailPoints].filter(p=>p.t>=result.oldest*1000&&p.t>cachedAfter*1000).map(p=>[p.t,p])).values()].sort((a,b)=>a.t-b.t);
    const full=!until&&(cachedAfter===0||(result.oldest>0&&cachedAfter<result.oldest));
    historyRanges[n.mac]={key,full,after:cachedAfter,until:finalUntil,oldest:result.oldest,live:!until};historyPartial[n.mac]=!full;
    historyMetadata[n.mac]=result.data;historyBuckets[n.mac]=Number(result.data.bucketSeconds||n.sleepSeconds||1);historyRevisions[n.mac]=revision;delete historyLoadErrors[n.mac];
    if(!options.prefetch&&!options.full&&desired!=='all'&&Number(desired)<=86400)prefetchRecentHistory(n);
  }catch(error){
    if(error.name==='HistoryPrefetchLimit'&&options.prefetch)retryForeground=Number(chartWindows[n.mac])===604800;
    else if(serial===historyLoadSerial&&selectedSensorMac===n.mac&&(timedOut||!controller.signal.aborted)&&(!options.prefetch||Number(chartWindows[n.mac])===604800))historyLoadErrors[n.mac]=timedOut?'History request timed out. Retry the selected window.':error.message;
  }
  finally{
    if(historyLoadController===controller){historyLoadController=null;delete historyLoading[n.mac];}
    if(retryForeground&&serial===historyLoadSerial&&selectedSensorMac===n.mac)queueMicrotask(()=>syncHistories(statusData.sensors||[]).then(()=>{if(selectedSensorMac===n.mac)renderSensors((statusData.sensors||[]).filter(sensor=>sensor.provisioned));}));
  }
}
function historyBounds(history,now=Date.now()){let oldest=now,newest=0,count=0;for(const p of history){const t=Number(p.t);if(Number.isFinite(t)&&t<=now){oldest=Math.min(oldest,t);newest=Math.max(newest,t);count++;}}return {oldest:count?oldest:now,newest:count?newest:now,count};}
function environmentChartPanel(n,history){
  const available=availableChartMetrics(n),selected=available.includes(chartMetrics[n.mac])?chartMetrics[n.mac]:(available[0]||'battery');chartMetrics[n.mac]=selected;
  const relative=n.motionReference?.enabled&&['ax','ay','az','gx','gy','gz'].includes(selected),definition=relative?{...chartDefinitions[selected],label:chartDefinitions[selected].label+' · relative'}:chartDefinitions[selected],bounds=historyBounds(history),now=Date.now(),oldest=Number(n.history?.oldest)>0?Number(n.history.oldest)*1000:bounds.oldest,full=Math.max(1,Math.ceil((now-oldest)/1000));
  const desired=chartWindows[n.mac]??86400,windowSeconds=desired==='all'?full:Number(desired),end=chartEnds[n.mac]??now,start=end-windowSeconds*1000;
  const bucketSeconds=Number(historyBuckets[n.mac]||n.sleepSeconds||1),continuity=historyContinuitySeconds(history,bucketSeconds),samples=liveSeries(history,selected,'','').points.map(p=>relative?{...p,v:motionValue(n,selected,p.v)}:p).filter(p=>p.t>=start&&p.t<=end).sort((a,b)=>a.t-b.t),saved=chartEnds[n.mac]!=null;
  const presets=[1,10,30,60,300,900,3600,86400,604800,2592000,7776000,15552000,31536000,63072000].filter(v=>v<=Math.max(63072000,full)||v===windowSeconds);
  const choices=[...new Set([...presets,...(desired==='all'?[]:[windowSeconds])])].sort((a,b)=>a-b);
  const metricOptions=available.map(key=>`<option value="${key}" ${key===selected?'selected':''}>${chartDefinitions[key].label}</option>`).join('');
  const windows=choices.map(v=>`<option value="${v}" ${desired!=='all'&&v===windowSeconds?'selected':''}>${historyStepLabel(v)}</option>`).join('')+`<option value="all" ${desired==='all'?'selected':''}>All retained data</option>`;
  const metadata=n.history||{},mac=esc(n.mac),error=historyLoadErrors[n.mac],partial=Boolean(historyPartial[n.mac]),loading=Boolean(historyLoading[n.mac]);
  const statusLine=error?`<div class="historyError" role="status">${esc(error)} <button type="button" class="btn secondary" onclick="delete historyRevisions['${mac}'];syncHistories(statusData.sensors).then(()=>renderSensors(statusData.sensors.filter(n=>n.provisioned)))">Retry</button></div>`:loading?'<p role="status">Loading selected history …</p>':partial?'<p role="status">Selected period loaded · full history loads on export.</p>':'';
  return `<div class="chartPanel"><div class="chartPrimary"><label>Measurement<select aria-label="History metric" onchange="setChartMetric('${mac}',this.value)">${metricOptions}</select></label><label>Period<select aria-label="Chart time window" onchange="setChartWindow('${mac}',this.value)">${windows}</select></label></div><div class="chartNavigation"><div class="chartNavigationButtons"><button type="button" class="btn secondary" ${(Number(metadata.points)||bounds.count)&&start>oldest+1?'':'disabled'} onclick="moveChartHistory('${mac}',-1)">Previous window</button><button type="button" class="btn secondary" ${saved?'':'disabled'} onclick="moveChartHistory('${mac}',1)">Next window</button><button type="button" class="btn secondary" ${saved?'':'disabled'} onclick="moveChartHistory('${mac}',0)">Now</button></div><span class="muted">${esc(historyRangeLabel(start,end))}</span></div><div class="chartStatusSlot">${statusLine}</div><div class="chartStage">${chartSvg(samples,definition,bucketSeconds,continuity,windowSeconds,chartFilters[n.mac]||'raw',end,loading)}</div><details class="chartOptions" data-ui-key="history-options"><summary>Display options &amp; CSV export</summary><div class="chartOptionsBody">${chartFilterControl(n)}<button type="button" class="btn secondary" ${loading?'disabled':''} onclick="exportHistory('${mac}')">Export full history CSV</button></div><p class="muted" style="font-size:11.5px;margin:0 0 10px">Display smoothing affects the curve only; cursor values and CSV exports retain original samples.</p></details><div class="historyStorageInfo">${metadata.durablePoints??history.length} saved · ${metadata.pendingPoints||0} buffered · ${historyStepLabel(bucketSeconds)} steps${metadata.capacityPoints?' · capacity ~'+metadata.capacityPoints+' pts':''}${metadata.oldest?' · since '+esc(new Date(Number(metadata.oldest)*1000).toLocaleDateString()):''}${metadata.dropped?' · '+Number(metadata.dropped)+' dropped':''}${metadata.writeFailed?' · Storage full or unavailable':''} · Oldest history is removed when space is needed.</div></div>`;
}
function chartPanel(n,history){const manual=n.live&&!n.normalHistory&&!metricFocus[n.mac];return `<div class="recordingSection">${recordingControls(n)}${manual||(!metricFocus[n.mac]&&recordingViews[n.mac]?.selected)?recordingCharts(n,history):environmentChartPanel(n,history)}</div>`;}
function chartSamplesAttribute(samples){if(samples.length<=2000)return 'data-samples="'+esc(JSON.stringify(samples))+'"';const key=String(++historyPayloadSequence);historyPayloads.set(key,samples);while(historyPayloads.size>32)historyPayloads.delete(historyPayloads.keys().next().value);return 'data-samples="[]" data-cache-key="'+key+'"';}
function chartSeriesAttribute(series){let total=0;for(const s of series)total+=s.points.length;if(total<=4000)return 'data-series="'+esc(JSON.stringify(series))+'"';const key=String(++historyPayloadSequence);historyPayloads.set(key,series);while(historyPayloads.size>32)historyPayloads.delete(historyPayloads.keys().next().value);return 'data-series="[]" data-cache-key="'+key+'"';}
function chartReducePoints(points,maximum=900){
  if(points.length<=maximum)return points;const result=[],size=Math.ceil(points.length/Math.floor(maximum/4));
  for(let start=0;start<points.length;start+=size){const end=Math.min(points.length,start+size);let low=start,high=start;for(let i=start+1;i<end;i++){if(points[i].v<points[low].v)low=i;if(points[i].v>points[high].v)high=i;}for(const i of [...new Set([start,low,high,end-1])].sort((a,b)=>a-b))result.push(points[i]);}return result;
}
function nearestHistoryPoint(points,time){let low=0,high=points.length;while(low<high){const middle=(low+high)>>1;if(points[middle].t<time)low=middle+1;else high=middle;}const a=points[Math.max(0,low-1)],b=points[Math.min(points.length-1,low)];return !a?b:!b?a:Math.abs(a.t-time)<=Math.abs(b.t-time)?a:b;}

function historyChartBounds(history,now=Date.now()){return historyBounds(history,now)}

async function boot(){const configuration=loadConfig(),initialStatus=requestJson('/api/status?ts='+Date.now()).then(value=>({value}),error=>({error}));try{await configuration}catch(e){$('connection').classList.remove('ok');$('connection').querySelector('span').textContent='Station configuration unavailable · retrying';setTimeout(boot,2500);return}bindForms();await loadStatus(initialStatus.then(result=>{if(result.error)throw result.error;return result.value}));renderRemoteChannels();const nextPollMs=()=>statusFailureCount>0?Math.min(5000,Math.round(1500*Math.pow(1.5,Math.min(statusFailureCount-1,3)))):Math.max(100,livePollMs());const schedulePoll=()=>{clearTimeout(window._dashboardPollTimer);if(document.hidden)return;window._dashboardPollTimer=setTimeout(async()=>{if(!document.hidden){window.lastDashboardPoll=Date.now();await loadStatus();}schedulePoll();},nextPollMs());};window._scheduleDashboardPoll=schedulePoll;schedulePoll();const wakePoll=()=>{if(!document.hidden){statusFailureCount=0;loadStatus().finally(schedulePoll)}};document.addEventListener('visibilitychange',()=>{if(!document.hidden)wakePoll();else clearTimeout(window._dashboardPollTimer)});window.addEventListener('online',wakePoll);window.addEventListener('focus',wakePoll)}boot();
</script></body></html>
)WCH";

}  // namespace station
