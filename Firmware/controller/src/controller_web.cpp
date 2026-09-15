#include "controller_web.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <stdlib.h>
#include <string.h>

namespace {

WebServer server(80);
bool serverStarted = false;
ControllerWebDataSource source{};
ControllerWebActions action{};

const char DEBUG_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Laser Parkour Debug</title>
  <style>body{font-family:system-ui,sans-serif;margin:0;background:#111827;color:#f9fafb}header,main{max-width:70rem;margin:auto;padding:1.25rem}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(18rem,1fr));gap:1rem}.card{background:#1f2937;border-radius:.8rem;padding:1rem;box-shadow:0 .5rem 2rem #0005}h1,h2{margin-top:0}.ok{color:#4ade80}.bad{color:#f87171}pre{white-space:pre-wrap;overflow-wrap:anywhere;color:#dbeafe}small{color:#9ca3af}</style>
</head>
<body><header><h1>Laser Parkour Debug</h1><p><small>Read-only polling dashboard</small></p></header><main>
<section class="card"><h2>System</h2><pre id="system">Loading…</pre></section>
<section class="card"><h2>Game</h2><pre id="game">Loading…</pre></section>
<section class="card"><h2>Nodes</h2><pre id="nodes">Loading…</pre></section>
<section class="card"><h2>Settings</h2><pre id="settings">Loading…</pre></section>
<section class="card"><h2>Recent attempts</h2><pre id="recent">Loading…</pre></section>
<section class="card"><h2>Top 10</h2><pre id="top">Loading…</pre></section>
</main><script>
async function update(id,path){const el=document.getElementById(id);try{const r=await fetch(path,{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);el.textContent=JSON.stringify(await r.json(),null,2)}catch(e){el.textContent='Unavailable: '+e.message}}
function refreshFast(){update('system','/api/system');update('game','/api/game')}
function refreshNodes(){update('nodes','/api/nodes')}
function refreshSlow(){update('settings','/api/settings');update('recent','/api/results/recent');update('top','/api/results/top')}
refreshFast();refreshNodes();refreshSlow();setInterval(refreshFast,1000);setInterval(refreshNodes,2000);setInterval(refreshSlow,5000);
</script></body></html>)HTML";

const char SETUP_PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Laser Parkour Setup</title>
<style>:root{color-scheme:dark}*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;background:#111827;color:#f9fafb}header,main{max-width:75rem;margin:auto;padding:1.2rem}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(20rem,1fr));gap:1rem}.card{background:#1f2937;border-radius:.8rem;padding:1rem;margin-bottom:1rem;box-shadow:0 .4rem 1.5rem #0004}h1,h2,h3{margin-top:0}label{display:block;margin:.65rem 0}input{width:100%;padding:.55rem;border:1px solid #4b5563;border-radius:.35rem;background:#111827;color:#fff}button{padding:.55rem .8rem;border:0;border-radius:.4rem;background:#2563eb;color:white;cursor:pointer;margin:.25rem .3rem .25rem 0}button.secondary{background:#4b5563}button.danger{background:#b91c1c}button:disabled{opacity:.45;cursor:not-allowed}.status{padding:.65rem;border-radius:.4rem;background:#374151;margin:.7rem 0}.ok{color:#4ade80}.bad{color:#f87171}.warn{color:#fbbf24}.nodes{border:1px solid #4b5563;border-radius:.5rem;overflow:hidden}.node-head,.node{display:grid;grid-template-columns:5rem 7rem minmax(8rem,1fr) 5rem 7rem 17rem;align-items:center;gap:.7rem;padding:.45rem .7rem}.node-head{background:#374151;color:#d1d5db;font-size:.85rem}.node{border-bottom:1px solid #4b5563}.node:last-child{border-bottom:0}.node-state{font-weight:600}.counter{font-weight:700;font-variant-numeric:tabular-nums}.counter.changed{color:#ff4d61;text-shadow:0 0 .5rem #ff263f;animation:counter-alert 1s ease-in-out 2}@keyframes counter-alert{50%{background:#7f1d1d;color:#fff}}.node-actions{display:flex;white-space:nowrap}.node-actions button{padding:.4rem .6rem}.facts{display:grid;grid-template-columns:auto 1fr;gap:.25rem .7rem;margin:.5rem 0}.facts dt{color:#9ca3af}.facts dd{margin:0;overflow-wrap:anywhere}.password{display:flex;gap:.4rem}.password input{flex:1}.password button{white-space:nowrap}.message{position:sticky;top:.5rem;z-index:2}dialog{width:min(34rem,calc(100% - 2rem));border:1px solid #4b5563;border-radius:.8rem;background:#1f2937;color:#f9fafb;padding:1.2rem}dialog::backdrop{background:#000a}.dialog-actions{text-align:right}small,.muted{color:#9ca3af}@media(max-width:60rem){.node-head,.node{grid-template-columns:4.5rem 6rem minmax(7rem,1fr) 4rem 14rem}.node-head span:nth-child(5),.node .availability{display:none}}@media(max-width:45rem){.node-head{display:none}.node{grid-template-columns:4.5rem 1fr 4rem auto}.node-state{grid-column:1/3}.node-actions{grid-column:4;grid-row:1/3}}</style></head>
<body><header><h1>Laser Parkour Setup</h1><div id="message" class="status message">Loading controller…</div></header><main>
<section class="card"><h2>System</h2><dl id="system" class="facts"></dl><button id="setupMode">Return system to Setup</button><button id="gameMode">Switch to Game</button><button id="refresh" class="secondary">Refresh status</button><button id="rescan" class="secondary">Rescan nodes</button><button id="clearTop" class="danger">Reset Top 10</button><p><small>Returning to Setup aborts an active attempt. Node commissioning remains available through the serial CLI.</small></p></section>
<div class="grid"><section class="card"><h2>Game configuration</h2><form id="gameConfig"><label>Interruption penalty (ms)<input name="penalty_ms" type="number" min="0" max="3600000" required></label><label>Maximum run time (ms)<input name="maximum_run_ms" type="number" min="1" max="4294967295" required></label><button type="submit">Save game configuration</button></form></section>
<section class="card"><h2>Wi-Fi configuration</h2><form id="wifiConfig"><label>Wi-Fi SSID<input name="ssid" maxlength="32" required></label><label>Wi-Fi password<div class="password"><input name="password" type="password" minlength="8" maxlength="63" required><button id="showPassword" type="button" class="secondary">Show</button></div></label><button type="submit">Save Wi-Fi configuration</button><p><small>Changes take effect after restarting the controller.</small></p></form></section></div>
<section class="card"><h2>Configure all lasers</h2><div id="sensorConsistency" class="status warn">Checking laser configurations…</div><form id="allSensors"><div class="grid"><label>Threshold<input name="threshold" type="number" min="0" max="1023" required value="240"></label><label>Hysteresis<input name="hysteresis" type="number" min="0" max="1023" required value="16"></label><label>Stable time (ms)<input name="stable_time_ms" type="number" min="0" max="1000" required value="30"></label><label>Cooldown (ms)<input name="cooldown_ms" type="number" min="0" max="5000" required value="500"></label></div><button type="submit">Apply to all lasers</button><button id="resetLaserCounters" type="button" class="secondary">Reset all laser event counters</button></form></section>
<section class="card"><h2>Node inventory</h2><div class="nodes"><div class="node-head"><span>Address</span><span>Type</span><span>State</span><span>Events</span><span>Availability</span><span>Actions</span></div><div id="nodes"><p>Loading…</p></div></div></section></main><dialog id="nodeDialog"><h2 id="dialogTitle"></h2><div id="dialogContent"></div><div class="dialog-actions"><button id="closeDialog" class="secondary">Close</button></div></dialog>
<script>
const $=s=>document.querySelector(s), msg=$('#message'),previousCounters=new Map();let setup=true,currentSettings=null,currentNodes=[],busy=false;
function say(text,kind='ok'){msg.textContent=text;msg.className='status message '+kind}
async function request(path,options={}){const r=await fetch(path,{cache:'no-store',...options});let data={};try{data=await r.json()}catch(e){}if(!r.ok)throw new Error(data.message||data.error||('HTTP '+r.status));return data}
function formBody(form){return new URLSearchParams(new FormData(form))}
function facts(el,items){el.replaceChildren();for(const [k,v] of items){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;el.append(dt,dd)}}
function sensorForm(node){const f=document.createElement('form');f.className='sensor';for(const [name,label,max] of [['threshold','Threshold',1023],['hysteresis','Hysteresis',1023],['stable_time_ms','Stable time (ms)',1000],['cooldown_ms','Cooldown (ms)',5000]]){const l=document.createElement('label'),i=document.createElement('input');l.textContent=label;i.name=name;i.type='number';i.min=0;i.max=max;i.required=true;i.value=node.sensor_config?.[name]??'';l.append(i);f.append(l)}const b=document.createElement('button');b.textContent='Save sensor settings';b.disabled=!setup||!node.available;f.append(b);f.onsubmit=async e=>{e.preventDefault();if(await act('/api/nodes/'+node.address_hex+'/sensor',{method:'POST',body:formBody(f)},'Sensor configuration saved'))$('#nodeDialog').close()};return f}
function inputState(n){const d=n.diagnostics;if(!d)return['Unknown','warn'];if(n.role==='laser')return d.input_state?['Beam interrupted','bad']:['Beam clear','ok'];if(n.role==='start'||n.role==='finish')return d.input_state?['Button pressed','warn']:['Button released','ok'];return['Not configured','warn']}
function showInfo(n){const d=n.diagnostics,c=n.sensor_config;$('#dialogTitle').textContent='Node 0x'+n.address_hex.toUpperCase()+' information';const content=$('#dialogContent');content.replaceChildren();facts(content.appendChild(document.createElement('dl')),[['Type',n.role],['Availability',n.available?'online':'unavailable'],['Protocol version',n.protocol],['Firmware version',n.firmware],['Input state',inputState(n)[0]],['Raw ADC',d?d.raw_adc:'—'],['Filtered ADC',d?d.filtered_adc:'—'],['Event counter',n.event_counter],['Boot counter',n.boot_counter],['Status flags','0x'+n.status_flags.toString(16).toUpperCase()],['Threshold',c?c.threshold:'—'],['Hysteresis',c?c.hysteresis:'—'],['Stable time',c?c.stable_time_ms+' ms':'—'],['Cooldown',c?c.cooldown_ms+' ms':'—']]);$('#nodeDialog').showModal()}
function showConfig(n){$('#dialogTitle').textContent='Configure laser 0x'+n.address_hex.toUpperCase();const content=$('#dialogContent');content.replaceChildren();content.append(sensorForm(n));$('#nodeDialog').showModal()}
function updateSensorConsistency(nodes){const el=$('#sensorConsistency'),lasers=nodes.filter(n=>n.role==='laser');if(!lasers.length){el.textContent='No laser nodes discovered.';el.className='status warn';return}const unreadable=lasers.filter(n=>!n.sensor_config);if(unreadable.length){el.textContent='Warning: active configuration unavailable for '+unreadable.length+' of '+lasers.length+' laser nodes; consistency cannot be verified.';el.className='status warn';return}const key=c=>[c.threshold,c.hysteresis,c.stable_time_ms,c.cooldown_ms].join('/'),reference=key(lasers[0].sensor_config),different=lasers.filter(n=>key(n.sensor_config)!==reference);if(different.length){el.textContent='Warning: laser nodes do not all use the same sensor configuration.';el.className='status bad'}else{el.textContent='All '+lasers.length+' laser nodes use the same sensor configuration.';el.className='status ok'}}
function renderNodes(data){currentNodes=data.nodes;updateSensorConsistency(data.nodes);const box=$('#nodes');box.replaceChildren();if(!data.nodes.length){box.textContent='No nodes discovered.';return}for(const n of data.nodes){const row=document.createElement('div');row.className='node';const address=document.createElement('strong'),role=document.createElement('span'),state=document.createElement('span'),counter=document.createElement('span'),available=document.createElement('span'),actions=document.createElement('span');address.textContent='0x'+n.address_hex.toUpperCase();role.textContent=n.role;const stateInfo=inputState(n);state.textContent=stateInfo[0];state.className='node-state '+stateInfo[1];const previous=previousCounters.get(n.address);counter.textContent=n.event_counter;counter.className='counter'+(previous!==undefined&&n.event_counter>previous?' changed':'');previousCounters.set(n.address,n.event_counter);available.textContent=n.available?'online':'unavailable';available.className='availability '+(n.available?'ok':'bad');actions.className='node-actions';const info=document.createElement('button');info.textContent='Info';info.className='secondary';info.onclick=()=>showInfo(n);actions.append(info);if(n.role==='laser'){const config=document.createElement('button');config.textContent='Configure';config.disabled=!setup||!n.available;config.onclick=()=>showConfig(n);actions.append(config)}const id=document.createElement('button');id.textContent='Identify';id.disabled=!setup||!n.available;id.onclick=()=>act('/api/nodes/'+n.address_hex+'/identify',{method:'POST'},'Identify started');actions.append(id);row.append(address,role,state,counter,available,actions);box.append(row)}}
function renderSystem(health,system,game,nodes){setup=game.state==='SETUP';facts($('#system'),[['Health',health.ok&&!system.internal_fault?'OK':'Fault'],['AP',health.ok?'running':'unavailable'],['SSID',currentSettings?.wifi_ssid??'—'],['IP address',health.ip],['Uptime',Math.floor(system.uptime_ms/1000)+' s'],['Game state',game.state],['Nodes',nodes.count],['Bus read failures',system.bus_reads_failed]]);$('#setupMode').disabled=setup;$('#gameMode').disabled=!setup;$('#clearTop').disabled=!setup;document.querySelectorAll('#gameConfig input,#gameConfig button,#wifiConfig input,#wifiConfig button,#allSensors input,#allSensors button').forEach(x=>x.disabled=!setup);renderNodes(nodes)}
async function refresh(announce=true){try{const [health,system,game,settings,nodes]=await Promise.all(['/api/health','/api/system','/api/game','/api/settings','/api/nodes'].map(p=>request(p)));currentSettings=settings;renderSystem(health,system,game,nodes);const g=$('#gameConfig'),w=$('#wifiConfig');g.penalty_ms.value=settings.penalty_ms;g.maximum_run_ms.value=settings.maximum_run_ms;w.ssid.value=settings.wifi_ssid;w.password.value=settings.wifi_password;if(announce)say(setup?'Setup controls ready':'Game is active; return to Setup to change configuration','warn')}catch(e){say('Refresh failed: '+e.message,'bad')}}
async function refreshNodesLive(){if(document.hidden||busy)return;try{renderNodes(await request('/api/nodes'))}catch(e){say('Node refresh failed: '+e.message,'bad')}}
async function refreshSystemLive(){if(document.hidden||busy)return;try{const [health,system,game,nodes]=await Promise.all(['/api/health','/api/system','/api/game','/api/nodes'].map(p=>request(p)));renderSystem(health,system,game,nodes)}catch(e){say('System refresh failed: '+e.message,'bad')}}
async function act(path,options,label){if(busy)return false;busy=true;try{say('Working…','warn');const d=await request(path,options);await refresh(false);say(d.message||label);return true}catch(e){say(e.message,'bad');return false}finally{busy=false}}
async function configureAll(form){if(busy)return;const lasers=currentNodes.filter(n=>n.role==='laser'&&n.available);if(!lasers.length){say('No available laser nodes to configure','bad');return}busy=true;let saved=0;try{for(const n of lasers){say('Configuring laser 0x'+n.address_hex.toUpperCase()+' ('+(saved+1)+'/'+lasers.length+')…','warn');await request('/api/nodes/'+n.address_hex+'/sensor',{method:'POST',body:formBody(form)});saved++}await refresh(false);say(saved+' laser configurations saved')}catch(e){say(saved+' of '+lasers.length+' saved; '+e.message,'bad')}finally{busy=false}}
async function switchToGame(){if(busy)return;busy=true;try{say('Starting game mode…','warn');await request('/api/system/game',{method:'POST'});location.replace('/game')}catch(e){say(e.message,'bad');busy=false}}$('#setupMode').onclick=()=>{if(confirm('Return to Setup? An active attempt will be aborted.'))act('/api/system/setup',{method:'POST'},'System returned to Setup')};$('#gameMode').onclick=switchToGame;$('#refresh').onclick=()=>refresh();$('#rescan').onclick=()=>act('/api/nodes/rescan',{method:'POST'},'Node rescan complete');$('#clearTop').onclick=()=>{if(confirm('Really clear the complete Top 10? This cannot be undone.'))act('/api/results/top',{method:'DELETE'},'Top 10 cleared')};
$('#showPassword').onclick=()=>{const p=$('#wifiConfig').password;p.type=p.type==='password'?'text':'password';$('#showPassword').textContent=p.type==='password'?'Show':'Hide'};
function settingsBody(){const p=new URLSearchParams(new FormData($('#gameConfig')));for(const [k,v] of new FormData($('#wifiConfig')))p.set(k,v);return p}$('#gameConfig').onsubmit=e=>{e.preventDefault();act('/api/settings',{method:'POST',body:settingsBody()},'Game configuration saved')};$('#wifiConfig').onsubmit=e=>{e.preventDefault();act('/api/settings',{method:'POST',body:settingsBody()},'Wi-Fi configuration saved')};$('#allSensors').onsubmit=e=>{e.preventDefault();if(confirm('Apply these settings to every discovered laser node?'))configureAll(e.target)};$('#resetLaserCounters').onclick=()=>{if(confirm('Reset the event counters of all discovered laser nodes?'))act('/api/nodes/counters/reset',{method:'POST'},'Laser event counters reset')};$('#closeDialog').onclick=()=>$('#nodeDialog').close();refresh();setInterval(refreshNodesLive,2000);setInterval(refreshSystemLive,10000);
</script></body></html>)HTML";

const char GAME_PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Laser Parkour Game</title><style>:root{color-scheme:dark;--cyan:#31f5ff;--green:#66ff9a;--pink:#ff3cac;--line:#155e75}*{box-sizing:border-box}body{font-family:"Courier New",ui-monospace,monospace;margin:0;background:radial-gradient(circle at 50% -20%,#123044 0,#050a0f 52%,#020405 100%);color:#e6fcff;overflow:auto}.page{height:100dvh;width:100%;padding:.8rem 1.25rem;display:grid;grid-template-rows:auto auto minmax(0,1fr);gap:.8rem}.top,.play,.panel{background:linear-gradient(145deg,#09141ddd,#050b11ee);border:1px solid var(--line);border-radius:.35rem;padding:.8rem 1rem;box-shadow:0 0 1rem #00d9ff18,inset 0 0 1.5rem #00d9ff08}.top{display:flex;align-items:center;gap:.8rem;border-top:2px solid var(--cyan)}.top h1{font-size:clamp(1.4rem,2vw,2rem);letter-spacing:.08em;text-transform:uppercase;color:var(--cyan);text-shadow:0 0 .7rem #00e5ff88;margin:0}.state{flex:1;font-size:clamp(1rem,1.5vw,1.35rem);font-weight:700;padding:.55rem .8rem;border:1px solid #164e63;background:#07131b;text-transform:uppercase}.ok{color:var(--green);text-shadow:0 0 .5rem #35ff7977}.warn{color:#ffd84a}.bad{color:#ff526d}button{font:700 1rem "Courier New",monospace;padding:.6rem .9rem;border:1px solid #168aa0;border-radius:.2rem;background:#073544;color:var(--cyan);cursor:pointer;text-transform:uppercase}button:hover{background:#0b4a5c;box-shadow:0 0 .7rem #23d9ff55}button.danger{background:#421120;border-color:#a72545;color:#ff8095}button:disabled{opacity:.38;cursor:not-allowed;box-shadow:none}.player-control{min-width:19rem}.player-control label,.latest h2,.panel h2{color:var(--cyan);font-weight:700;text-transform:uppercase;letter-spacing:.06em}.name-row{display:flex;gap:.4rem}.name-row input{min-width:0;flex:1;font:700 1.05rem "Courier New",monospace;padding:.55rem;border:1px solid #168aa0;border-radius:.2rem;background:#02080d;color:white;outline:none}.name-row input:focus{border-color:var(--cyan);box-shadow:0 0 .7rem #31f5ff55}.message{position:absolute;margin:.2rem 0;font-size:.8rem}.play{display:block}.latest h2,.panel h2{font-size:clamp(1.1rem,1.5vw,1.4rem);margin:0 0 .5rem}.latest-grid{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:.7rem}.metric{background:#06121a;border-left:3px solid var(--pink);padding:.8rem}.metric small{display:block;color:#7dcbd3;font-size:.9rem;text-transform:uppercase}.metric strong{display:block;font-size:clamp(1.2rem,1.8vw,1.7rem);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.scoreboards{display:grid;grid-template-columns:1fr 1fr;gap:.8rem;min-height:0}.panel{min-width:0;overflow:hidden;display:flex;flex-direction:column;border-top:2px solid var(--pink)}table{width:100%;height:100%;border-collapse:collapse;font-size:clamp(.9rem,1.3vw,1.2rem);table-layout:fixed}th,td{text-align:left;padding:.32rem .5rem;border-bottom:1px solid #123340;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}th{height:2rem;color:#68c4cf;text-transform:uppercase;font-size:.82em}th:first-child,td:first-child{width:2.7rem;text-align:center}#top tr:nth-child(-n+3),#recent tr:first-child{font-weight:700}#top tr:nth-child(1){color:#ffd700;text-shadow:0 0 .45rem #ffd70066}#top tr:nth-child(2){color:#dce7ed;text-shadow:0 0 .4rem #dce7ed44}#top tr:nth-child(3){color:#e6a267;text-shadow:0 0 .4rem #cd7f3266}@media(max-width:1100px){.top{flex-wrap:wrap}.state{order:3;flex-basis:100%}.player-control{flex:1}}@media(max-width:900px){body{overflow:auto}.page{height:auto;min-height:100dvh;padding:.6rem}.latest-grid{grid-template-columns:repeat(3,1fr)}.scoreboards{grid-template-columns:1fr}.panel{min-height:32rem}}@media(max-height:700px) and (min-width:901px){.page{padding:.4rem .7rem;gap:.4rem}.top,.play,.panel{padding:.45rem .7rem}th,td{padding:.15rem .35rem}.metric{padding:.45rem}}</style></head>
<body><main class="page"><header class="top"><h1>Laser Parkour</h1><div id="state" class="state warn">Loading game state…</div><div class="player-control"><form id="playerForm"><label for="player">Next player</label><div class="name-row"><input id="player" name="name" maxlength="32" pattern="[A-Za-z0-9_-](?:[A-Za-z0-9 _-]*[A-Za-z0-9_-])?" title="Use letters, numbers, spaces, dashes, and underscores; do not start or end with a space" autocomplete="off" required><button id="submitPlayer" disabled>Submit</button></div></form><p id="message" class="message warn"></p></div><button id="returnSetup" class="danger">Return to Setup</button></header><section class="play"><div class="latest"><h2>Latest result</h2><div id="latest" class="latest-grid"></div></div></section><section class="scoreboards"><article class="panel"><h2>Top 10</h2><table><thead><tr><th>#</th><th>Player</th><th>Score</th><th>Raw</th><th>Hits</th></tr></thead><tbody id="top"></tbody></table></article><article class="panel"><h2>Latest runs</h2><table><thead><tr><th>#</th><th>Player</th><th>Status</th><th>Score</th><th>Hits</th></tr></thead><tbody id="recent"></tbody></table></article></section></main><script>
const $=s=>document.querySelector(s);let previousState=null,busy=false;
async function get(path,options={}){const r=await fetch(path,{cache:'no-store',...options});let d={};try{d=await r.json()}catch(e){}if(!r.ok)throw new Error(d.message||d.error||('HTTP '+r.status));return d}
function time(us){if(us===null||us===undefined)return '—';const ms=Math.round(Number(us)/1000),minutes=Math.floor(ms/60000),seconds=((ms%60000)/1000).toFixed(3);return minutes?minutes+':'+seconds.padStart(6,'0')+' min':seconds+' s'}
function statusName(s){return['—','Finished','Aborted','Timeout','Fault'][s]||'Unknown'}
function stateText(g){if(g.state==='SETUP')return['Game mode is not active','warn'];if(g.state==='WAIT_PLAYER')return['Waiting for a player name','ok'];if(g.state==='WAIT_START')return[g.start_enabled?g.player+' ready — press Start':'Start blocked — clear all lasers','warn'];if(g.state==='WAIT_FINISH')return[g.player+' running — waiting for Finish · interruptions: '+g.interruptions,'ok'];return['Game fault — return to Setup','bad']}
function metric(label,value){const box=document.createElement('div'),small=document.createElement('small'),strong=document.createElement('strong');box.className='metric';small.textContent=label;strong.textContent=value;box.append(small,strong);return box}
function renderLatest(result){const box=$('#latest');box.replaceChildren();if(!result){box.append(metric('Status','No result yet'));return}for(const item of [['Player',result.player],['Status',statusName(result.status)],['Score',time(result.score_time_us)],['Raw time',time(result.raw_time_us)],['Interruptions',result.interruptions],['Penalty',time(result.penalty_time_us)]])box.append(metric(item[0],item[1]))}
function renderTable(id,results,top){const body=$(id);body.replaceChildren();for(let i=0;i<10;i++){const tr=document.createElement('tr'),r=results[i];for(const value of r?[i+1,r.player,top?time(r.score_time_us):statusName(r.status),top?time(r.raw_time_us):time(r.score_time_us),r.interruptions]:[i+1,'—','—','—','—']){const td=document.createElement('td');td.textContent=value;tr.append(td)}body.append(tr)}}
async function loadResults(){try{const [recent,top]=await Promise.all([get('/api/results/recent'),get('/api/results/top')]);renderTable('#recent',recent.results,false);renderTable('#top',top.results,true);renderLatest(recent.results[0])}catch(e){$('#message').textContent='Results unavailable: '+e.message}}
async function pollGame(){if(busy)return;try{const g=await get('/api/game'),info=stateText(g);$('#state').textContent=info[0];$('#state').className='state '+info[1];const accepts=g.state==='WAIT_PLAYER';$('#player').disabled=!accepts;$('#submitPlayer').disabled=!accepts||!$('#player').checkValidity();if(previousState!==null&&previousState!==g.state&&g.state==='WAIT_PLAYER')loadResults();previousState=g.state}catch(e){$('#state').textContent='Controller unavailable: '+e.message;$('#state').className='state bad'}}
$('#player').oninput=()=>$('#submitPlayer').disabled=previousState!=='WAIT_PLAYER'||!$('#player').checkValidity();$('#playerForm').onsubmit=async e=>{e.preventDefault();if(busy)return;busy=true;$('#submitPlayer').disabled=true;try{const body=new URLSearchParams(new FormData(e.target));await get('/api/game/player',{method:'POST',body});$('#message').textContent='';$('#player').value=''}catch(e){$('#message').textContent=e.message}finally{busy=false;pollGame()}};
$('#returnSetup').onclick=async()=>{if(!confirm('Return to Setup? An active attempt will be aborted.'))return;busy=true;$('#message').textContent='Returning to Setup…';try{await get('/api/system/setup',{method:'POST'});location.replace('/setup')}catch(e){$('#message').textContent=e.message;busy=false}};
loadResults();pollGame();setInterval(pollGame,750);
</script></body></html>)HTML";

void addNoCacheHeaders() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
}

void handleRoot() {
  server.sendHeader("Location", "/setup", true);
  addNoCacheHeaders();
  server.send(302, "text/plain; charset=utf-8", "Redirecting to setup");
}

void handleDebug() {
  addNoCacheHeaders();
  server.send_P(200, "text/html; charset=utf-8", DEBUG_PAGE);
}

void handleSetup() {
  addNoCacheHeaders();
  server.send_P(200, "text/html; charset=utf-8", SETUP_PAGE);
}

void handleGamePage() {
  addNoCacheHeaders();
  server.send_P(200, "text/html; charset=utf-8", GAME_PAGE);
}

void sendJson(String (*provider)()) {
  if (provider == nullptr) {
    server.send(503, "application/json", "{\"error\":\"unavailable\"}");
    return;
  }
  addNoCacheHeaders();
  server.send(200, "application/json; charset=utf-8", provider());
}

void handleSystem() { sendJson(source.systemJson); }
void handleGame() { sendJson(source.gameJson); }
void handleNodes() { sendJson(source.nodesJson); }
void handleRecent() { sendJson(source.recentResultsJson); }
void handleTop() { sendJson(source.topResultsJson); }
void handleSettings() { sendJson(source.settingsJson); }

void sendAction(int (*callback)(String &)) {
  String response;
  const int status = callback == nullptr ? 503 : callback(response);
  if (response.length() == 0U) response = F("{\"error\":\"unavailable\"}");
  addNoCacheHeaders();
  server.send(status, "application/json; charset=utf-8", response);
}

bool parseUnsignedArgument(const char *name, unsigned long maximum,
                           unsigned long &value) {
  if (!server.hasArg(name)) return false;
  const String input = server.arg(name);
  if (input.length() == 0U) return false;
  char *end = nullptr;
  value = strtoul(input.c_str(), &end, 10);
  return end != input.c_str() && *end == '\0' && value <= maximum;
}

bool parseSensorArguments(uint16_t &threshold, uint16_t &hysteresis,
                          uint16_t &stableTimeMs, uint16_t &cooldownMs) {
  unsigned long values[4];
  if (!parseUnsignedArgument("threshold", 65535UL, values[0]) ||
      !parseUnsignedArgument("hysteresis", 65535UL, values[1]) ||
      !parseUnsignedArgument("stable_time_ms", 65535UL, values[2]) ||
      !parseUnsignedArgument("cooldown_ms", 65535UL, values[3])) return false;
  threshold = values[0]; hysteresis = values[1];
  stableTimeMs = values[2]; cooldownMs = values[3];
  return true;
}

void handleSetupMode() { sendAction(action.setSetupMode); }
void handleGameMode() { sendAction(action.setGameMode); }
void handleRescan() { sendAction(action.rescanNodes); }
void handleClearTop() { sendAction(action.clearTopResults); }

void handleSubmitPlayer() {
  String response;
  int status = 400;
  if (!server.hasArg("name")) {
    response = F("{\"error\":\"missing_name\",\"message\":\"Enter a player name\"}");
  } else if (action.submitPlayer == nullptr) {
    status = 503; response = F("{\"error\":\"unavailable\"}");
  } else {
    status = action.submitPlayer(server.arg("name").c_str(), response);
  }
  addNoCacheHeaders();
  server.send(status, "application/json; charset=utf-8", response);
}

void handleResetLaserCounters() { sendAction(action.resetLaserCounters); }

void handleSaveSettings() {
  unsigned long penalty, maximumRun;
  String response;
  int status = 400;
  if (!parseUnsignedArgument("penalty_ms", 3600000UL, penalty) ||
      !parseUnsignedArgument("maximum_run_ms", 0xFFFFFFFFUL, maximumRun) ||
      maximumRun == 0UL || !server.hasArg("ssid") ||
      !server.hasArg("password")) {
    response = F("{\"error\":\"invalid_settings\",\"message\":\"Check all configuration values\"}");
  } else if (action.saveSettings == nullptr) {
    status = 503; response = F("{\"error\":\"unavailable\"}");
  } else {
    status = action.saveSettings(penalty, maximumRun,
                                 server.arg("ssid").c_str(),
                                 server.arg("password").c_str(), response);
  }
  addNoCacheHeaders();
  server.send(status, "application/json; charset=utf-8", response);
}

void handleConfigureAllSensors() {
  uint16_t threshold, hysteresis, stable, cooldown;
  String response;
  int status = 400;
  if (!parseSensorArguments(threshold, hysteresis, stable, cooldown)) {
    response = F("{\"error\":\"invalid_sensor_config\"}");
  } else if (action.configureAllSensors == nullptr) {
    status = 503; response = F("{\"error\":\"unavailable\"}");
  } else {
    status = action.configureAllSensors(threshold, hysteresis, stable,
                                        cooldown, response);
  }
  addNoCacheHeaders();
  server.send(status, "application/json; charset=utf-8", response);
}

void handleHealth() {
  addNoCacheHeaders();
  const String response = String("{\"ok\":") +
                          (controllerWebHealthy() ? "true" : "false") +
                          ",\"ip\":\"" + WiFi.softAPIP().toString() +
                          "\"}";
  server.send(200, "application/json; charset=utf-8", response);
}

void handleNotFound() {
  const String uri = server.uri();
  const String prefix = "/api/nodes/";
  if (server.method() == HTTP_GET && uri.startsWith(prefix) &&
      source.nodeJson != nullptr) {
    const String addressText = uri.substring(prefix.length());
    char *end = nullptr;
    const unsigned long address = strtoul(addressText.c_str(), &end, 16);
    if (addressText.length() == 2U && end != addressText.c_str() &&
        *end == '\0' && address <= 0x7FU) {
      const String response = source.nodeJson(static_cast<uint8_t>(address));
      addNoCacheHeaders();
      server.send(response.length() == 0U ? 404 : 200,
                  "application/json; charset=utf-8",
                  response.length() == 0U ? "{\"error\":\"node_not_found\"}"
                                          : response);
      return;
    }
  }
  if (server.method() == HTTP_POST && uri.startsWith(prefix)) {
    const int slash = uri.indexOf('/', prefix.length());
    const String addressText = slash < 0 ? String() :
                               uri.substring(prefix.length(), slash);
    const String operation = slash < 0 ? String() : uri.substring(slash + 1);
    char *end = nullptr;
    const unsigned long address = strtoul(addressText.c_str(), &end, 16);
    if (addressText.length() == 2U && end != addressText.c_str() &&
        *end == '\0' && address <= 0x7FU) {
      String response;
      int status = 404;
      if (operation == "identify" && action.identifyNode != nullptr) {
        status = action.identifyNode(address, response);
      } else if (operation == "sensor" && action.configureSensor != nullptr) {
        uint16_t threshold, hysteresis, stable, cooldown;
        if (parseSensorArguments(threshold, hysteresis, stable, cooldown)) {
          status = action.configureSensor(address, threshold, hysteresis,
                                          stable, cooldown, response);
        } else {
          status = 400;
          response = F("{\"error\":\"invalid_sensor_config\"}");
        }
      }
      if (response.length() == 0U) response = F("{\"error\":\"not_found\"}");
      addNoCacheHeaders();
      server.send(status, "application/json; charset=utf-8", response);
      return;
    }
  }
  addNoCacheHeaders();
  server.send(404, "application/json; charset=utf-8",
              "{\"error\":\"not_found\"}");
}

}  // namespace

bool controllerWebBegin(const lp_controller_config_t &config,
                        const ControllerWebDataSource &dataSource,
                        const ControllerWebActions &actions) {
  serverStarted = false;
  source = dataSource;
  action = actions;
  // Arduino-Pico applies the regulatory country at compile time through
  // WIFICC. The production environment is compiled for Germany.
  if (strcmp(config.wifi_country, "DE") != 0) {
    return false;
  }
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(config.wifi_ssid, config.wifi_password)) {
    return false;
  }
  server.on("/", HTTP_GET, handleRoot);
  server.on("/debug", HTTP_GET, handleDebug);
  server.on("/setup", HTTP_GET, handleSetup);
  server.on("/game", HTTP_GET, handleGamePage);
  server.on("/api/health", HTTP_GET, handleHealth);
  server.on("/api/system", HTTP_GET, handleSystem);
  server.on("/api/game", HTTP_GET, handleGame);
  server.on("/api/nodes", HTTP_GET, handleNodes);
  server.on("/api/results/recent", HTTP_GET, handleRecent);
  server.on("/api/results/top", HTTP_GET, handleTop);
  server.on("/api/settings", HTTP_GET, handleSettings);
  server.on("/api/settings", HTTP_POST, handleSaveSettings);
  server.on("/api/system/setup", HTTP_POST, handleSetupMode);
  server.on("/api/system/game", HTTP_POST, handleGameMode);
  server.on("/api/nodes/rescan", HTTP_POST, handleRescan);
  server.on("/api/nodes/sensors", HTTP_POST, handleConfigureAllSensors);
  server.on("/api/results/top", HTTP_DELETE, handleClearTop);
  server.on("/api/game/player", HTTP_POST, handleSubmitPlayer);
  server.on("/api/nodes/counters/reset", HTTP_POST,
            handleResetLaserCounters);
  server.onNotFound(handleNotFound);
  server.begin();
  serverStarted = true;
  return controllerWebHealthy();
}

void controllerWebHandle(void) {
  if (serverStarted) {
    server.handleClient();
  }
}

bool controllerWebHealthy(void) {
  return serverStarted && WiFi.getMode() == WIFI_AP &&
         WiFi.softAPIP() != IPAddress(0U, 0U, 0U, 0U);
}

void controllerWebPrintStatus(void) {
  Serial.print("AP SSID: ");
  Serial.println(WiFi.softAPSSID());
  Serial.print("Web interface: http://");
  Serial.println(WiFi.softAPIP());
}
