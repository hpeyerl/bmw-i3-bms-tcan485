// =============================================================================
// WiFiManager.cpp  - BMW i3 / T-CAN485
//
// Stripped compared to M5Dial version:
//   - No LVGL display push
//   - No Nextion comms
//   - Single-page dark HTML dashboard (auto-refreshes every 2 s)
//
// Routes:
//   GET  /            -> HTML dashboard
//   GET  /api/data    -> JSON pack + per-module data
//   GET  /api/can     -> JSON array of last 50 CAN frames (rolling)
//   GET  /api/settings-> current settings JSON
//   POST /api/settings-> update + save settings
//   POST /api/cantx   -> transmit a raw CAN frame from the browser
//   POST /api/reboot  -> software reboot
// =============================================================================
#include "WiFiManager.h"
#include "CANManager.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <driver/twai.h>

extern BMSModuleManager bms;
extern EEPROMSettings   settings;
extern CANManager       can;

static AsyncWebServer *server = nullptr;

// ---------------------------------------------------------------------------
// CAN frame rolling log (populated by wifiLogCAN, served via /api/can)
// ---------------------------------------------------------------------------
#define CAN_LOG_SIZE 50
struct CANLogEntry {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
    uint32_t ts;
};
static CANLogEntry canLog[CAN_LOG_SIZE];
static int canLogHead  = 0;
static int canLogCount = 0;

void wifiLogCAN(uint32_t id, uint8_t *data, uint8_t len)
{
    canLog[canLogHead] = { id, {0}, len, millis() };
    memcpy(canLog[canLogHead].data, data, len < 8 ? len : 8);
    canLogHead = (canLogHead + 1) % CAN_LOG_SIZE;
    if (canLogCount < CAN_LOG_SIZE) canLogCount++;
}

// ---------------------------------------------------------------------------
// HTML dashboard (single file, auto-refresh)
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMW i3 BMS</title>
<style>
:root{--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;--bg:#0f172a;--card:#1e293b;--text:#f1f5f9;--sub:#94a3b8;--border:#334155}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
h1{text-align:center;font-size:1.2rem;padding:.8rem;color:var(--ok)}
.tabs{display:flex;border-bottom:2px solid var(--border);padding:0 1rem}
.tab{padding:.5rem 1rem;cursor:pointer;font-size:.82rem;color:var(--sub);border-bottom:2px solid transparent;margin-bottom:-2px}
.tab.active{color:var(--ok);border-bottom-color:var(--ok)}
.panel{display:none;padding:1rem}.panel.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:.5rem;margin-bottom:1rem}
.stat{background:var(--card);border-radius:.5rem;padding:.7rem;text-align:center}
.stat .val{font-size:1.4rem;font-weight:700}.stat .lbl{font-size:.68rem;color:var(--sub);margin-top:.2rem}
.ok{color:var(--ok)}.warn{color:var(--warn)}.err{color:var(--err)}
.modules{display:grid;grid-template-columns:repeat(auto-fill,minmax(340px,1fr));gap:.7rem}
.module{background:var(--card);border-radius:.5rem;padding:.7rem;border:2px solid var(--border)}
.module.faulted{border-color:var(--err)}
.module h3{font-size:.8rem;margin-bottom:.5rem;display:flex;justify-content:space-between}
.cells{display:grid;grid-template-columns:1fr 1fr;gap:.25rem}
.cell{background:#0f172a;border-radius:.25rem;padding:.25rem .4rem;display:flex;align-items:center;gap:.3rem;font-size:.74rem}
.cn{width:1.6rem;color:var(--sub)}.cv{width:3.2rem;font-weight:600}
.bar{flex:1;height:5px;background:var(--border);border-radius:3px;overflow:hidden}
.bf{height:100%;background:var(--ok);transition:width .3s}
.temps{margin-top:.4rem;font-size:.72rem;color:var(--sub)}
#canTable{width:100%;border-collapse:collapse;font-size:.76rem;font-family:monospace}
#canTable th{text-align:left;padding:.3rem .5rem;color:var(--sub);border-bottom:1px solid var(--border)}
#canTable td{padding:.25rem .5rem;border-bottom:1px solid #1e293b}
.sf{background:var(--card);border-radius:.5rem;padding:.7rem;margin-bottom:.8rem}
.sf h4{font-size:.75rem;color:var(--sub);margin-bottom:.4rem;text-transform:uppercase}
.srow{display:flex;gap:.4rem;align-items:flex-end;flex-wrap:wrap}
.srow label{font-size:.72rem;color:var(--sub);display:block;margin-bottom:.2rem}
.srow input{background:#0f172a;border:1px solid var(--border);color:var(--text);padding:.3rem .4rem;border-radius:.25rem;font-family:monospace;font-size:.8rem}
.fi{width:80px}.fd{flex:1;min-width:160px}
button{background:#1d4ed8;color:#fff;border:none;border-radius:.3rem;padding:.35rem .8rem;cursor:pointer;font-size:.8rem}
button:hover{background:#2563eb}
button.red{background:#b91c1c}.button.red:hover{background:#dc2626}
.sgrid{display:grid;grid-template-columns:1fr 1fr;gap:.4rem 1rem}
@media(max-width:480px){.sgrid{grid-template-columns:1fr}}
.sfield{display:flex;flex-direction:column;gap:.15rem}
.sfield label{font-size:.7rem;color:var(--sub)}
.sfield input{background:#0f172a;border:1px solid var(--border);color:var(--text);padding:.28rem .4rem;border-radius:.25rem;font-size:.8rem}
</style>
</head>
<body>
<h1>&#9889; BMW i3 BMS</h1>
<div class="tabs">
  <div class="tab active" onclick="show('pack',this)">Pack</div>
  <div class="tab" onclick="show('can',this)">CAN</div>
  <div class="tab" onclick="show('settings',this)">Settings</div>
</div>

<div id="pack" class="panel active">
  <div class="grid" id="stats"></div>
  <div class="modules" id="modules"></div>
</div>

<div id="can" class="panel">
  <div class="sf">
    <h4>Send CAN Frame</h4>
    <div class="srow">
      <div><label>ID (hex)</label><input class="fi" id="txid" placeholder="0x351"></div>
      <div class="fd"><label>Data (hex bytes, space-sep)</label><input id="txdata" placeholder="00 01 02 03 04 05 06 07"></div>
      <button onclick="sendCAN()">TX</button>
    </div>
    <div id="txresult" style="font-size:.72rem;margin-top:.3rem;color:var(--sub)"></div>
  </div>
  <table id="canTable">
    <thead><tr><th>ms</th><th>ID</th><th>Len</th><th>Data</th></tr></thead>
    <tbody id="canBody"></tbody>
  </table>
</div>

<div id="settings" class="panel">
  <div class="sf">
    <h4>Pack Topology</h4>
    <div class="sgrid">
      <div class="sfield"><label>Modules in Series</label><input id="numSeries" type="number"></div>
      <div class="sfield"><label>Strings in Parallel</label><input id="numParallel" type="number"></div>
      <div class="sfield"><label>SoC 0% V/module</label><input id="socLo" type="number" step="0.1"></div>
      <div class="sfield"><label>SoC 100% V/module</label><input id="socHi" type="number" step="0.1"></div>
    </div>
  </div>
  <div class="sf">
    <h4>Thresholds</h4>
    <div class="sgrid">
      <div class="sfield"><label>OV limit (V/cell)</label><input id="ovSetpoint" type="number" step="0.01"></div>
      <div class="sfield"><label>UV limit (V/cell)</label><input id="uvSetpoint" type="number" step="0.01"></div>
      <div class="sfield"><label>Over-temp (C)</label><input id="otSetpoint" type="number" step="0.5"></div>
      <div class="sfield"><label>Under-temp (C)</label><input id="utSetpoint" type="number" step="0.5"></div>
    </div>
  </div>
  <div class="sf">
    <h4>Connectivity</h4>
    <div class="sgrid">
      <div class="sfield"><label>SimpBMS Battery ID</label><input id="batteryID" type="number"></div>
      <div class="sfield"><label>Log Level (0=debug 4=off)</label><input id="logLevel" type="number"></div>
      <div class="sfield"><label>Charger CAN ID (hex)</label><input id="chargerHeartbeatID"></div>
      <div class="sfield"><label>CAN Inhibit (0/1)</label><input id="canInhibitEnabled" type="number"></div>
    </div>
  </div>
  <div style="display:flex;gap:.5rem;margin-top:.5rem">
    <button onclick="saveSettings()">Save Settings</button>
    <button class="red" onclick="reboot()">Reboot</button>
  </div>
  <div id="saveresult" style="font-size:.72rem;margin-top:.4rem;color:var(--sub)"></div>
</div>

<script>
function show(id,el){
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  el.classList.add('active');
}

function vc(v){
  if(v>=4.1)  return 'ok';
  if(v>=3.3)  return 'ok';
  if(v>=3.0)  return 'warn';
  return 'err';
}
function barPct(v){
  let p=((v-2.8)/(4.2-2.8))*100;
  return Math.min(100,Math.max(0,p));
}

async function fetchData(){
  try{
    const r=await fetch('/api/data'); const d=await r.json();
    const s=document.getElementById('stats');
    const fault=d.faulted;
    s.innerHTML=`
      <div class="stat"><div class="val ${fault?'err':'ok'}">${fault?'FAULT':'OK'}</div><div class="lbl">Status</div></div>
      <div class="stat"><div class="val ok">${d.packV.toFixed(2)}V</div><div class="lbl">Pack Voltage</div></div>
      <div class="stat"><div class="val">${d.soc.toFixed(0)}%</div><div class="lbl">SoC</div></div>
      <div class="stat"><div class="val ${vc(d.lowCell)}">${d.lowCell.toFixed(3)}V</div><div class="lbl">Low Cell</div></div>
      <div class="stat"><div class="val">${d.highCell.toFixed(3)}V</div><div class="lbl">High Cell</div></div>
      <div class="stat"><div class="val">${((d.highCell-d.lowCell)*1000).toFixed(0)}mV</div><div class="lbl">Delta</div></div>
      <div class="stat"><div class="val">${d.avgTemp.toFixed(1)}C</div><div class="lbl">Avg Temp</div></div>
      <div class="stat"><div class="val">${d.numModules}</div><div class="lbl">Modules</div></div>
    `;
    const m=document.getElementById('modules');
    m.innerHTML='';
    (d.modules||[]).forEach(mod=>{
      let cells='';
      (mod.cells||[]).forEach((v,i)=>{
        cells+=`<div class="cell"><span class="cn">C${i+1}</span><span class="cv ${vc(v)}">${v.toFixed(3)}</span><div class="bar"><div class="bf" style="width:${barPct(v)}%;background:${v<3.0?'#ef4444':v<3.3?'#f59e0b':'#22c55e'}"></div></div></div>`;
      });
      m.innerHTML+=`<div class="module${mod.faulted?' faulted':''}">
        <h3><span>Module #${mod.addr}</span><span>${mod.voltage.toFixed(3)}V</span></h3>
        <div class="cells">${cells}</div>
        <div class="temps">T1: ${mod.t1.toFixed(1)}C &nbsp; T2: ${mod.t2.toFixed(1)}C${mod.faulted?' &nbsp; <span class="err">[FAULT]</span>':''}</div>
      </div>`;
    });
  } catch(e){}
}

async function fetchCAN(){
  try{
    const r=await fetch('/api/can'); const d=await r.json();
    const b=document.getElementById('canBody');
    b.innerHTML=d.map(f=>`<tr><td>${f.ts}</td><td style="color:#38bdf8">0x${f.id.toString(16).toUpperCase().padStart(3,'0')}</td><td>${f.len}</td><td>${f.data}</td></tr>`).join('');
  }catch(e){}
}

async function sendCAN(){
  const id=parseInt(document.getElementById('txid').value,16);
  const raw=document.getElementById('txdata').value.trim().split(/\s+/).map(x=>parseInt(x,16));
  const r=await fetch('/api/cantx',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,data:raw})});
  const d=await r.json();
  document.getElementById('txresult').textContent=d.ok?'TX OK':('Error: '+d.error);
}

async function fetchSettings(){
  const r=await fetch('/api/settings'); const d=await r.json();
  ['numSeries','numParallel','socLo','socHi','ovSetpoint','uvSetpoint','otSetpoint','utSetpoint','batteryID','logLevel','canInhibitEnabled'].forEach(k=>{
    const el=document.getElementById(k); if(el) el.value=d[k]!==undefined?d[k]:'';
  });
  const chg=document.getElementById('chargerHeartbeatID');
  if(chg) chg.value='0x'+((d.chargerHeartbeatID||0x305).toString(16).toUpperCase());
}

async function saveSettings(){
  const body={
    numSeries:    parseInt(document.getElementById('numSeries').value),
    numParallel:  parseInt(document.getElementById('numParallel').value),
    socLo:        parseFloat(document.getElementById('socLo').value),
    socHi:        parseFloat(document.getElementById('socHi').value),
    ovSetpoint:   parseFloat(document.getElementById('ovSetpoint').value),
    uvSetpoint:   parseFloat(document.getElementById('uvSetpoint').value),
    otSetpoint:   parseFloat(document.getElementById('otSetpoint').value),
    utSetpoint:   parseFloat(document.getElementById('utSetpoint').value),
    batteryID:    parseInt(document.getElementById('batteryID').value),
    logLevel:     parseInt(document.getElementById('logLevel').value),
    canInhibitEnabled: parseInt(document.getElementById('canInhibitEnabled').value),
    chargerHeartbeatID: parseInt(document.getElementById('chargerHeartbeatID').value,16),
  };
  const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const d=await r.json();
  document.getElementById('saveresult').textContent=d.ok?'Saved OK':('Error: '+d.error);
}

async function reboot(){
  if(!confirm('Reboot BMS?')) return;
  await fetch('/api/reboot',{method:'POST'});
}

fetchData(); fetchCAN(); fetchSettings();
setInterval(fetchData, 2000);
setInterval(fetchCAN, 2000);
</script>
</body>
</html>
)rawhtml";

// ---------------------------------------------------------------------------
// WiFiManager
// ---------------------------------------------------------------------------
WiFiManager::WiFiManager() : running(false), ipAddr("") {}

bool WiFiManager::begin()
{
    WiFi.softAP(settings.wifiSSID[0] ? settings.wifiSSID : WIFI_SSID_DEFAULT,
                settings.wifiPass[0]  ? settings.wifiPass  : WIFI_PASS_DEFAULT);
    ipAddr = WiFi.softAPIP().toString();
    Logger::info("WiFi AP: SSID=%s  IP=%s", settings.wifiSSID, ipAddr.c_str());

    if (server) { server->end(); delete server; server = nullptr; }
    server = new AsyncWebServer(80);

    // --- GET / ---
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    // --- GET /api/data ---
    server->on("/api/data", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["packV"]      = bms.getPackVoltage();
        doc["lowCell"]    = bms.getLowCellVolt();
        doc["highCell"]   = bms.getHighCellVolt();
        doc["avgTemp"]    = bms.getAvgTemperature();
        doc["soc"]        = bms.getSoCPercent();
        doc["numModules"] = bms.getNumModules();
        doc["faulted"]    = bms.isFaultedState();
        doc["chargerActive"] = can.getChargerActive();
        doc["currentA"]   = can.getCanCurrentA();

        JsonArray mods = doc["modules"].to<JsonArray>();
        for (int i = 1; i <= BMW_I3_MAX_MODS; i++) {
            BMSModule &m = bms.getModule(i);
            if (!m.isExisting()) continue;
            JsonObject mo = mods.add<JsonObject>();
            mo["addr"]    = i;
            mo["voltage"] = m.getModuleVoltage();
            mo["t1"]      = m.getTemperature(0);
            mo["t2"]      = m.getTemperature(1);
            mo["faulted"] = (m.getFaults() || m.getAlerts()) ? true : false;
            JsonArray cells = mo["cells"].to<JsonArray>();
            for (int c = 0; c < BMW_I3_CELLS_PER_MOD; c++)
                cells.add(m.getCellVoltage(c));
        }
        String json; serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- GET /api/can ---
    server->on("/api/can", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        // Walk from oldest to newest
        int start = (canLogCount < CAN_LOG_SIZE) ? 0 : canLogHead;
        for (int n = 0; n < canLogCount; n++) {
            int idx = (start + n) % CAN_LOG_SIZE;
            JsonObject e = arr.add<JsonObject>();
            e["id"]  = canLog[idx].id;
            e["len"] = canLog[idx].len;
            e["ts"]  = canLog[idx].ts;
            char hex[32] = {0};
            for (int b = 0; b < canLog[idx].len; b++)
                snprintf(hex + b * 3, 4, "%02X ", canLog[idx].data[b]);
            e["data"] = String(hex);
        }
        String json; serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- GET /api/settings ---
    server->on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["numSeries"]          = (int)settings.numSeries;
        doc["numParallel"]        = (int)settings.numParallel;
        doc["socLo"]              = settings.socLo;
        doc["socHi"]              = settings.socHi;
        doc["ovSetpoint"]         = settings.OverVSetpoint;
        doc["uvSetpoint"]         = settings.UnderVSetpoint;
        doc["otSetpoint"]         = settings.OverTSetpoint;
        doc["utSetpoint"]         = settings.UnderTSetpoint;
        doc["batteryID"]          = (int)settings.batteryID;
        doc["logLevel"]           = (int)settings.logLevel;
        doc["wifiEnabled"]        = (bool)settings.wifiEnabled;
        doc["canInhibitEnabled"]  = (bool)settings.canInhibitEnabled;
        doc["chargerHeartbeatID"] = (int)settings.chargerHeartbeatID;
        String json; serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- POST /api/settings ---
    server->on("/api/settings", HTTP_POST,
        [](AsyncWebServerRequest *req) {},
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
            JsonDocument jdoc;
            if (deserializeJson(jdoc, data, len)) {
                req->send(400, "application/json", "{\"ok\":false,\"error\":\"json\"}"); return;
            }
            JsonObject j = jdoc.as<JsonObject>();
            bool changed = false;
            auto setU8 = [&](const char *k, uint8_t &f, int lo, int hi){
                if(!j[k].isNull()){int v=j[k];if(v>=lo&&v<=hi){f=(uint8_t)v;changed=true;}}
            };
            auto setF = [&](const char *k, float &f, float lo, float hi){
                if(!j[k].isNull()){float v=j[k];if(v>=lo&&v<=hi){f=v;changed=true;}}
            };
            setU8("numSeries",   settings.numSeries,   1, 20);
            setU8("numParallel", settings.numParallel,  1, 4);
            setF("socLo",        settings.socLo,        10.0f, 40.0f);
            setF("socHi",        settings.socHi,        20.0f, 60.0f);
            setF("ovSetpoint",   settings.OverVSetpoint, 3.0f, 5.0f);
            setF("uvSetpoint",   settings.UnderVSetpoint,2.0f, 4.0f);
            setF("otSetpoint",   settings.OverTSetpoint, 20.0f,80.0f);
            setF("utSetpoint",   settings.UnderTSetpoint,-40.0f,20.0f);
            setU8("batteryID",   settings.batteryID,   1, 14);
            setU8("logLevel",    settings.logLevel,     0, 4);
            if(!j["canInhibitEnabled"].isNull()){ settings.canInhibitEnabled=(j["canInhibitEnabled"].as<int>()!=0)?1:0; changed=true; }
            if(!j["chargerHeartbeatID"].isNull()){ int v=j["chargerHeartbeatID"]; if(v>0&&v<=0x7FF){settings.chargerHeartbeatID=(uint32_t)v;changed=true;} }
            if(changed){ EEPROM.put(EEPROM_PAGE,settings); EEPROM.commit(); }
            req->send(200,"application/json",changed?"{\"ok\":true}":"{\"ok\":false,\"error\":\"no valid fields\"}");
        }
    );

    // --- POST /api/cantx ---
    server->on("/api/cantx", HTTP_POST,
        [](AsyncWebServerRequest *req) {},
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
            JsonDocument jdoc;
            if (deserializeJson(jdoc, data, len)) {
                req->send(400,"application/json","{\"ok\":false,\"error\":\"json\"}"); return;
            }
            uint32_t id  = jdoc["id"] | 0;
            JsonArray arr = jdoc["data"].as<JsonArray>();
            if (!arr || arr.size() < 1 || arr.size() > 8 || id > 0x7FF) {
                req->send(400,"application/json","{\"ok\":false,\"error\":\"bad params\"}"); return;
            }
            uint8_t txdata[8]={};
            uint8_t txlen=0;
            for(JsonVariant b: arr) txdata[txlen++]=b.as<uint8_t>();
            twai_message_t msg={};
            msg.identifier=id; msg.extd=0; msg.data_length_code=txlen;
            memcpy(msg.data, txdata, txlen);
            esp_err_t res=twai_transmit(&msg, pdMS_TO_TICKS(10));
            if(res==ESP_OK){
                wifiLogCAN(id, txdata, txlen);
                req->send(200,"application/json","{\"ok\":true}");
            } else {
                char buf[64]; snprintf(buf,sizeof(buf),"{\"ok\":false,\"error\":\"TWAI 0x%02X\"}",res);
                req->send(200,"application/json",buf);
            }
        }
    );

    // --- POST /api/reboot ---
    server->on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest *req) {
        req->send(200, "application/json", "{\"ok\":true}");
        delay(500);
        ESP.restart();
    });

    server->onNotFound([](AsyncWebServerRequest *req) {
        req->send(404, "text/plain", "Not found");
    });

    server->begin();
    running = true;
    Logger::console("HTTP server started: http://%s", ipAddr.c_str());
    return true;
}

void WiFiManager::end()
{
    if (server) { server->end(); delete server; server = nullptr; }
    WiFi.softAPdisconnect(true);
    running = false;
    Logger::info("WiFi stopped");
}

bool   WiFiManager::isRunning() const { return running; }
String WiFiManager::getIP()     const { return ipAddr; }
void   WiFiManager::loop()            {}
