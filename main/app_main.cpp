#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <algorithm>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_check.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_netif.h>
#include <esp_system.h> 
#include <esp_timer.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_heap_caps.h>

#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/AttributeList.h>
#include <platform/DeviceInfoProvider.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>

#include <cJSON.h>

namespace {

static const char *kTag = "roehn_bridge";
static const char *kNvsNamespace = "roehnmatter";
static const char *kNvsConfigKey = "config";
static const char *kWifiHostname = "roehn-bridge";
static const uint8_t kHeader[] = "HSN_S-UDP";

// Increase discovery capacity to allow discovering many devices on the gateway.
// Kept within uint8_t limits and aligned with ESP-Matter dynamic endpoint limits.
// Lowered to 128 to avoid DRAM BSS overflow on ESP32-S3 while still
// providing a large discovery capacity. If you need more, convert the
// static arrays to heap allocations so they can live in PSRAM.
static constexpr uint8_t kMaxLights = 128;
static constexpr size_t kMaxBodyBytes = 16384;
static constexpr uint16_t kCommissioningTimeoutSec = 300;
static constexpr uint16_t kDefaultGatewayPort = 2006;
static constexpr uint16_t kDefaultCommandPort = 23;
static constexpr uint16_t kDefaultScanIntervalSec = 30;
static constexpr int kDefaultUdpTimeoutMs = 700;
static constexpr int kDefaultCommandConnectTimeoutMs = 3000;
static constexpr int kDefaultCommandResponseTimeoutMs = 600;

extern const uint8_t kModuleDriversJsonStart[] asm("_binary_module_drivers_json_start");
extern const uint8_t kModuleDriversJsonEnd[] asm("_binary_module_drivers_json_end");

struct ProcessorInfo {
    char source_ip[16];
    char name[32];
    char version[20];
    char serial[24];
    char ip[16];
    char mask[16];
    char gateway[16];
    char mac[20];
};

struct DeviceInfo {
    char processor_ip[16];
    uint8_t port;
    uint16_t hsnet_id;
    uint16_t device_id;
    uint8_t dev_model;
    char fw[16];
    char model[16];
    char extended_model[24];
    char serial_hex[24];
    uint8_t status;
    uint16_t crc;
    uint16_t eeprom_address;
    uint16_t bitmap;
};

struct SlotInfo {
    uint8_t initial_port;
    uint8_t capacity;
    uint8_t slot_type;
    char slot_name[12];
};

struct ModuleDriverInfo {
    char model_base_name[48];
    char firmware_name[48];
    char firmware_extended_name[48];
    int dev_model;
    uint8_t slot_count;
    SlotInfo slots[16];
};

struct ResourcesIndex {
    bool loaded;
    uint16_t module_count;
    ModuleDriverInfo modules[256];
};

struct FallbackDriverSpec {
    int dev_model;
    const char *firmware_name;
    const char *firmware_extended_name;
    uint8_t slot_type;
    uint8_t initial_port;
    uint8_t capacity;
};

struct LightConfig {
    char id[40];
    char name[64];
    char serial_hex[24];
    char model[16];
    char extended_model[24];
    uint16_t device_id;
    uint16_t hsnet_id;
    uint8_t dev_model;
    uint8_t channel;
    bool supports_brightness;
};

struct AppConfig {
    char gateway_host[64];
    uint16_t gateway_port;
    uint16_t scan_interval_sec;
    char gateway_name[32];
    char gateway_ip[16];
    uint8_t light_count;
    LightConfig *lights; // allocated on heap (preferably PSRAM)
    uint8_t module_count;
    uint16_t module_parent_ids[32]; // stable bridged_node endpoint IDs per module
};

struct LightState {
    bool on;
    uint8_t level_percent;
    esp_err_t last_err;
};

struct GatewayRuntime {
    bool connected;
    bool discovery_ok;
    char last_error[64];
    ProcessorInfo processor;
    uint16_t last_device_count;
    int64_t last_refresh_us;
};

struct PendingLoadCommand {
    uint8_t light_index;
    uint8_t level_percent;
    uint32_t sequence;
};

uint32_t g_command_sequence = 0;

AppConfig g_config = {};
LightState *g_light_states = nullptr;
uint16_t *g_endpoint_ids = nullptr;
httpd_handle_t g_httpd = nullptr;
TaskHandle_t g_gateway_task = nullptr;
TaskHandle_t g_command_task = nullptr;
QueueHandle_t g_command_queue = nullptr;
bool g_wifi_hostname_set = false;
EXT_RAM_BSS_ATTR ResourcesIndex g_resources;
EXT_RAM_BSS_ATTR GatewayRuntime g_gateway;
bool g_matter_started = false;

// Persistent TCP connection for Roehn gateway command/event channel (port 23).
// Mirrors the architecture in the roehn_dinplug reference: a single long-lived
// telnet connection is used for both sending commands and receiving real-time
// feedback events such as R:LOAD, R:SHADE, R:BTN etc.
static int g_event_sock = -1;
static SemaphoreHandle_t g_event_lock = nullptr;  // protects g_event_sock writes
static TaskHandle_t g_event_task_handle = nullptr;
static constexpr int kEventReconnectDelayMs = 5000;
static constexpr int kEventRecvTimeoutMs = 250;   // poll interval while idle

static const FallbackDriverSpec kFallbackLightDrivers[] = {
    {59, "RL12", "ADP-RL12", 1, 1, 12},
    {60, "DIM8", "ADP-DIM8", 2, 1, 8},
};


static const char kSetupPageHtml[] = R"html(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>ROEHN Matter Bridge</title>
<style>
:root{color-scheme:light;--ink:#132033;--muted:#617085;--line:rgba(19,32,51,.12);--card:rgba(255,255,255,.78);--blue:#1273ea;--green:#18a058;--amber:#c57b00;--red:#cf2e2e;--shadow:0 24px 80px rgba(19,32,51,.16)}
*{box-sizing:border-box}body{margin:0;min-height:100vh;font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","SF Pro Text","Helvetica Neue",sans-serif;color:var(--ink);background:radial-gradient(circle at 15% 10%,#fff3d8 0 14%,transparent 36%),radial-gradient(circle at 88% 12%,#d8efff 0 16%,transparent 38%),linear-gradient(145deg,#f8f4ea 0%,#eef6ff 46%,#edf7f0 100%);display:grid;place-items:center;padding:24px}
body:before{content:"";position:fixed;inset:0;background-image:linear-gradient(rgba(255,255,255,.34) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.28) 1px,transparent 1px);background-size:44px 44px;mask-image:linear-gradient(to bottom,rgba(0,0,0,.45),transparent 72%);pointer-events:none}
.shell{width:min(1100px,100%)}.hero{display:grid;grid-template-columns:1.02fr .98fr;gap:22px;align-items:stretch}.panel,.preview{background:var(--card);border:1px solid rgba(255,255,255,.72);box-shadow:var(--shadow);backdrop-filter:blur(24px) saturate(1.15);-webkit-backdrop-filter:blur(24px) saturate(1.15);border-radius:34px}
.panel{padding:30px}.preview{padding:24px;display:flex;flex-direction:column;gap:18px;min-height:620px;position:relative;overflow:hidden;overflow-y:auto;max-height:calc(100vh - 48px)}.preview:before{content:"";position:absolute;inset:auto -90px -120px auto;width:320px;height:320px;border-radius:50%;background:linear-gradient(135deg,#f7c66a,#7ed7a8 48%,#56a4ff);filter:blur(8px);opacity:.35;pointer-events:none;z-index:0}
.eyebrow{display:inline-flex;align-items:center;gap:8px;margin:0 0 18px;padding:7px 12px;border-radius:999px;background:rgba(255,255,255,.72);border:1px solid var(--line);font-size:13px;color:var(--muted);font-weight:650}.dot{width:8px;height:8px;border-radius:999px;background:var(--amber);box-shadow:0 0 0 5px rgba(197,123,0,.13)}.dot.good{background:var(--green);box-shadow:0 0 0 5px rgba(24,160,88,.13)}.dot.bad{background:var(--red);box-shadow:0 0 0 5px rgba(207,46,46,.13)}
h1{font-size:clamp(34px,6vw,64px);line-height:.94;letter-spacing:-.06em;margin:0 0 14px}.sub{font-size:18px;line-height:1.45;color:var(--muted);margin:0 0 28px;max-width:600px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}.field{display:grid;gap:10px;margin:0 0 18px}.label{font-size:13px;text-transform:uppercase;letter-spacing:.12em;color:#8b95a5;font-weight:800}
input{width:100%;border:1px solid var(--line);background:rgba(255,255,255,.78);border-radius:20px;padding:16px 18px;font:700 18px/1.1 -apple-system,BlinkMacSystemFont,"SF Pro Display",sans-serif;color:var(--ink);outline:none;box-shadow:inset 0 1px 0 rgba(255,255,255,.8)}
.actions{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:6px}button{border:0;border-radius:20px;padding:16px 18px;font-weight:800;font-size:16px;cursor:pointer;transition:.18s ease;box-shadow:0 12px 30px rgba(19,32,51,.12)}button:active{transform:scale(.98)}button:disabled{opacity:.55;cursor:not-allowed}.primary{background:var(--blue);color:white}.secondary{background:rgba(255,255,255,.8);color:#141a24;border:1px solid var(--line)}
.status{min-height:48px;margin-top:18px;padding:14px 16px;border-radius:18px;background:rgba(255,255,255,.58);border:1px solid var(--line);color:var(--muted);line-height:1.35}.status.good{color:#14783c;background:rgba(24,160,88,.11)}.status.bad{color:#b42318;background:rgba(207,46,46,.11)}.status.info{color:var(--blue);background:rgba(18,115,234,.11)}
.card{position:relative;border-radius:28px;background:linear-gradient(180deg,#fdfdfd,#f2f5f8);border:1px solid rgba(255,255,255,.92);box-shadow:inset 0 1px 0 #fff,0 18px 42px rgba(19,32,51,.12);padding:20px}.pill{display:inline-flex;gap:7px;align-items:center;border-radius:999px;background:#132033;color:white;padding:9px 12px;font-size:13px;font-weight:750}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:18px}.tile{background:rgba(255,255,255,.76);border:1px solid var(--line);border-radius:20px;padding:14px}.tile b{display:block;font-size:24px;letter-spacing:-.05em}.tile span{display:block;margin-top:4px;color:var(--muted);font-size:13px}
.meta{display:grid;gap:8px;margin-top:16px;font-size:14px;color:var(--muted)}.meta strong{color:var(--ink)}
.list{display:grid;gap:10px;position:relative;z-index:1}
.module-group{border-radius:20px;background:rgba(255,255,255,.5);border:1px solid var(--line);overflow:hidden;position:relative;z-index:1}
.module-header{display:grid;grid-template-columns:1fr auto auto;gap:10px;align-items:center;padding:14px 16px;cursor:pointer;user-select:none;background:rgba(255,255,255,.72);transition:background .15s}
.module-header:hover{background:rgba(255,255,255,.92)}
.module-header b{font-size:16px}.module-header .arrow{font-size:14px;color:var(--muted);transition:transform .2s;min-width:20px;text-align:center}
.module-header.open .arrow{transform:rotate(90deg)}
.module-channels{display:none;border-top:1px solid var(--line);padding:6px 8px}
.module-channels.open{display:grid;gap:4px}
.module-meta{font-size:12px;color:var(--muted);padding:0 16px 10px;margin-top:-4px}
.entity{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;padding:10px 14px;border-radius:14px;background:rgba(255,255,255,.6);border:1px solid transparent;margin:0 4px}.entity:hover{background:rgba(255,255,255,.92);border-color:var(--line)}
.entity b{display:block;font-size:15px}.entity span{display:block;margin-top:2px;color:var(--muted);font-size:12px}
.badge{display:inline-flex;align-items:center;justify-content:center;min-width:76px;padding:6px 10px;border-radius:999px;font-size:11px;font-weight:800;text-transform:uppercase;letter-spacing:.08em;background:#132033;color:#fff}.badge.dim{background:#1273ea;color:#fff}.badge.relay{background:#6f42c1;color:#fff}.badge.empty{background:#8b95a5;color:#fff}
.empty-state{text-align:center;padding:24px;color:var(--muted)}
@keyframes spin{to{transform:rotate(360deg)}}
.spinner{display:inline-block;width:20px;height:20px;border:2.5px solid var(--line);border-top-color:var(--blue);border-radius:50%;animation:spin .7s linear infinite;vertical-align:middle;margin-right:6px}
.overlay{position:fixed;inset:0;background:rgba(19,32,51,.6);display:flex;flex-direction:column;align-items:center;justify-content:center;z-index:100;gap:16px}
.overlay .msg{color:#fff;font-size:20px;font-weight:700}.overlay .timer{color:rgba(255,255,255,.7);font-size:16px}.overlay .spinner{width:36px;height:36px;border-width:4px;border-top-color:#fff;border-color:rgba(255,255,255,.2);border-top-color:#fff}
@media(max-width:860px){body{padding:14px;place-items:start center}.hero{grid-template-columns:1fr}.panel,.preview{border-radius:28px}.panel{padding:22px}.actions,.grid,.stats{grid-template-columns:1fr}.module-header{grid-template-columns:1fr auto auto}}
</style>
</head>
<body>
<div class="overlay" id="rebootOverlay" style="display:none">
<span class="spinner"></span>
<span class="msg">Discovery complete!</span>
<span class="timer" id="rebootTimer">Rebooting in 5 seconds...</span>
</div>
<main class="shell">
<section class="hero">
<div class="panel">
<p class="eyebrow"><span id="dot" class="dot"></span><span id="connectivity">Gateway status unknown</span></p>
<h1>ROEHN Matter Bridge</h1>
<p class="sub">Configure the ROEHN M16 gateway, discover connected DIN modules, and expose their light channels as Matter endpoints.</p>
<div class="grid">
<div class="field">
<span class="label">Gateway Host</span>
<input id="host" placeholder="192.168.1.50" autocomplete="off">
</div>
<div class="field">
<span class="label">Gateway Port</span>
<input id="port" type="number" min="1" max="65535" placeholder="2006">
</div>
</div>
<div class="field">
<span class="label">Scan Interval (seconds)</span>
<input id="scanInterval" type="number" min="5" max="3600" placeholder="30">
</div>
<div class="actions">
<button class="secondary" id="saveBtn">Save Gateway</button>
<button class="primary" id="discoverBtn">Discover Lights</button>
</div>
<div class="status" id="status">Loading current bridge configuration...</div>
</div>
<aside class="preview">
<div class="card">
<span class="pill">ROEHN Matter Bridge</span>
<div class="stats">
<div class="tile"><b id="lightCount">0</b><span>Discovered lights</span></div>
<div class="tile"><b id="moduleCount">0</b><span>Modules</span></div>
</div>
<div class="meta">
<div><strong>Gateway:</strong> <span id="gatewayName">Not configured</span></div>
<div><strong>Gateway IP:</strong> <span id="gatewayIp">-</span></div>
<div><strong>Version:</strong> <span id="gatewayVersion">-</span></div>
<div><strong>Last refresh:</strong> <span id="lastRefresh">Never</span></div>
</div>
</div>
<div class="list" id="entities"></div>
</aside>
</section>
</main>
<script>
const els={dot:document.querySelector("#dot"),connectivity:document.querySelector("#connectivity"),host:document.querySelector("#host"),port:document.querySelector("#port"),scanInterval:document.querySelector("#scanInterval"),saveBtn:document.querySelector("#saveBtn"),discoverBtn:document.querySelector("#discoverBtn"),status:document.querySelector("#status"),lightCount:document.querySelector("#lightCount"),moduleCount:document.querySelector("#moduleCount"),gatewayName:document.querySelector("#gatewayName"),gatewayIp:document.querySelector("#gatewayIp"),gatewayVersion:document.querySelector("#gatewayVersion"),lastRefresh:document.querySelector("#lastRefresh"),entities:document.querySelector("#entities"),rebootOverlay:document.querySelector("#rebootOverlay"),rebootTimer:document.querySelector("#rebootTimer")};
let config=null,statusData=null,busy=false,formDirty=false,rebooting=false,lastLightsJson="";
function setStatus(text,type=""){els.status.innerHTML=text;els.status.className=`status ${type}`;}
function stampToText(ms){if(!ms)return"Never";const sec=Math.max(0,Math.round(ms/1000));if(sec<5)return"Just now";if(sec<60)return`${sec}s ago`;const min=Math.round(sec/60);if(min<60)return`${min}m ago`;const hr=Math.round(min/60);return`${hr}h ago`;}
function syncField(el,value){
 if(document.activeElement===el||formDirty)return;
 el.value=value;
}
function render(){
 if(rebooting)return;
 const gw=config?.gateway||{}, stat=statusData?.gateway||{}, lights=statusData?.lights||config?.lights||[];
 syncField(els.host,gw.host||"");syncField(els.port,String(gw.port||2006));syncField(els.scanInterval,String(gw.scan_interval||30));
 els.lightCount.textContent=String(lights.length||0);
 // Only rebuild entity list when lights data changes (prevents collapsible reset)
 const lightsJson=JSON.stringify(lights);
 if(lightsJson!==lastLightsJson){
  lastLightsJson=lightsJson;
  // Group lights by module address
  const modules={};
  lights.forEach(l=>{const a=l.control_address||l.name?.split('-')[0]||'?';if(!modules[a]){modules[a]={addr:a,name:l.name?.replace(/-\\d+$/,'')||`Module ${a}`,model:l.model||"",lights:[]};}modules[a].lights.push(l);});
  els.moduleCount.textContent=String(Object.keys(modules).length);
  if(lights.length){
   let html="";
   for(const [addr,mod] of Object.entries(modules)){
    const nChannels=mod.lights.length;
    html+=`<div class="module-group"><div class="module-header" onclick="this.classList.toggle('open');this.nextElementSibling.classList.toggle('open')"><b>${mod.name}</b><span style="font-size:12px;color:var(--muted)">${nChannels} ch</span><span class="arrow">▶</span></div><div class="module-channels">`;
    mod.lights.forEach(l=>{const badgeCls=l.supports_brightness?"dim":"relay";const badgeTxt=l.supports_brightness?"Dimmer":"Relay";html+=`<div class="entity"><div><b>${l.name}</b><span>Ch ${l.channel}${l.endpoint_id?` • EP ${l.endpoint_id}`:''}</span></div><span class="badge ${badgeCls}">${badgeTxt}</span></div>`;});
    html+=`</div></div>`;
   }
   els.entities.innerHTML=html;
  }else{
   els.entities.innerHTML='<div class="empty-state"><b>No lights discovered yet</b><p>Save the gateway, then run discovery to build Matter light endpoints.</p></div>';
  }
 }
 const connected=!!stat.connected;
 els.connectivity.textContent=connected?"Gateway reachable":"Gateway offline or not configured";
 els.dot.className=`dot ${connected?"good":(gw.host?"bad":"")}`;
 els.gatewayName.textContent=stat.name||gw.name||"Not configured";
 els.gatewayIp.textContent=stat.ip||gw.gateway_ip||"-";
 els.gatewayVersion.textContent=stat.version||"-";
 els.lastRefresh.textContent=stampToText(stat.last_refresh_ms||0);
}
let reconnectAttempts=0;
async function fetchWithRetry(url,opts={},maxRetries=2){
 for(let i=0;i<=maxRetries;i++){
  try{
   const ctrl=new AbortController();const t=setTimeout(()=>ctrl.abort(),4000);
   const res=await fetch(url,{...opts,signal:ctrl.signal,cache:"no-store"});clearTimeout(t);
   return res;
  }catch(e){if(i<maxRetries){await new Promise(r=>setTimeout(r,800));}}
 }
 throw new Error("fetch failed");
}
async function loadAll(announce=false){
 if(rebooting)return;
 let ok=false;
 try{
  const cfgRes=await fetchWithRetry("/api/config");
  if(cfgRes.ok){config=await cfgRes.json();ok=true;}
 }catch(e){}
 try{
  const statRes=await fetchWithRetry("/api/status");
  if(statRes.ok){statusData=await statRes.json();ok=true;}
 }catch(e){}
 if(ok){
  render();reconnectAttempts=0;
  if(announce)setStatus("Bridge configuration loaded.");
 }else{
  reconnectAttempts++;
  if(reconnectAttempts>=8)setStatus(`<span class="spinner"></span> Waiting for bridge… (${reconnectAttempts})`,"info");
 }
}
function payload(){return{gateway:{host:els.host.value.trim(),port:Number(els.port.value||2006),scan_interval:Number(els.scanInterval.value||30)}}}
async function saveConfig(){
 busy=true;setStatus("Saving gateway configuration...");
 try{
  const res=await fetch("/api/config",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(payload())});
  const json=await res.json();if(!res.ok||!json.ok)throw new Error(json.error||"save_failed");
  formDirty=false;await loadAll();setStatus("Gateway configuration saved.","good");
 }catch(e){setStatus(`Save failed: ${e.message}`,"bad");}
 finally{busy=false;}
}
async function discover(){
 busy=true;els.discoverBtn.disabled=true;els.saveBtn.disabled=true;
 setStatus('<span class="spinner"></span> Talking to the ROEHN gateway and discovering modules...',"info");
 try{
  const res=await fetch("/api/discover",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(payload())});
  const json=await res.json();if(!res.ok||!json.ok)throw new Error(json.error||"discover_failed");
  const count=json.light_count||0;
  setStatus(`Discovery completed. Found ${count} light channels. Restarting…`,"good");
  // Show reboot overlay with countdown — board will restart in ~1s
  rebooting=true;
  els.rebootOverlay.style.display="flex";
  let sec=8;els.rebootTimer.textContent=`Rebooting in ${sec} seconds...`;
  const iv=setInterval(()=>{sec--;if(sec<=0){els.rebootTimer.textContent="Waiting for board...";}else{els.rebootTimer.textContent=`Restarting — back in ${sec}s...`;}},1000);
  // Poll for the board to come back online
  setTimeout(async()=>{
   for(let i=0;i<45;i++){
    await new Promise(r=>setTimeout(r,1500));
    try{const r=await fetch("/api/status",{cache:"no-store",signal:AbortSignal.timeout(2000)});if(r.ok){rebooting=false;els.rebootOverlay.style.display="none";clearInterval(iv);els.discoverBtn.disabled=false;els.saveBtn.disabled=false;await loadAll(true);setStatus("Reboot complete! Bridge is back online.","good");return;}}catch(e){}
   }
   rebooting=false;els.rebootOverlay.style.display="none";clearInterval(iv);els.discoverBtn.disabled=false;els.saveBtn.disabled=false;setStatus("Board did not come back online within 60s. Please check power and Wi-Fi.","bad");
  },8000);
 }catch(e){els.discoverBtn.disabled=false;els.saveBtn.disabled=false;setStatus(`Discovery failed: ${e.name==="AbortError"?"gateway_timeout":e.message}`,"bad");}
 finally{busy=false;}
}
["input","change"].forEach(evt=>[els.host,els.port,els.scanInterval].forEach(el=>el.addEventListener(evt,()=>{formDirty=true;})));
els.saveBtn.addEventListener("click",saveConfig);
els.discoverBtn.addEventListener("click",discover);
loadAll(true);setInterval(()=>{if(!busy&&!rebooting)loadAll(false);},3000);
</script>
</body>
</html>)html";

void copy_string(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, dst_size, "%s", src);
}

void uppercase_alnum_token(const char *src, char *dst, size_t dst_size)
{
    if (!dst || dst_size == 0) {
        return;
    }
    size_t out = 0;
    for (; src && *src != '\0' && out + 1 < dst_size; ++src) {
        if (isalnum(static_cast<unsigned char>(*src))) {
            dst[out++] = static_cast<char>(toupper(static_cast<unsigned char>(*src)));
        }
    }
    dst[out] = '\0';
}

const char *c_string_at(const uint8_t *data, size_t length, size_t start, size_t max_len)
{
    static char buffer[64];
    memset(buffer, 0, sizeof(buffer));
    if (!data || start >= length) {
        return buffer;
    }
    const size_t limit = std::min(length, start + max_len);
    size_t out = 0;
    for (size_t i = start; i < limit && out + 1 < sizeof(buffer); ++i) {
        if (data[i] == 0) {
            break;
        }
        buffer[out++] = static_cast<char>(data[i]);
    }
    buffer[out] = '\0';
    return buffer;
}

int read_json_int(cJSON *object, const char *name, int fallback)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    return cJSON_IsNumber(item) ? item->valueint : fallback;
}

const char *read_json_string(cJSON *object, const char *name)
{
    return cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(object, name));
}

uint8_t level_percent_to_matter(uint8_t percent)
{
    if (percent == 0) {
        return 0;
    }
    return static_cast<uint8_t>(std::max(1, std::min(254, static_cast<int>((percent * 254 + 50) / 100))));
}

uint8_t matter_to_level_percent(uint8_t matter_level)
{
    if (matter_level == 0) {
        return 0;
    }
    return static_cast<uint8_t>(std::max(1, std::min(100, static_cast<int>((matter_level * 100 + 127) / 254))));
}

uint16_t resolve_control_address(const LightConfig &light)
{
    return light.hsnet_id > 0 ? light.hsnet_id : light.device_id;
}

bool level_cluster_exists(uint16_t endpoint_id)
{
    return esp_matter::attribute::get(endpoint_id, chip::app::Clusters::LevelControl::Id,
                                      chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id) != nullptr;
}

void load_default_config()
{
    // Preserve lights pointer if already allocated; otherwise allocate in PSRAM.
    LightConfig *saved_lights = g_config.lights;
    memset(&g_config, 0, sizeof(g_config));
    g_config.lights = saved_lights;
    if (!g_config.lights) {
        g_config.lights = static_cast<LightConfig *>(heap_caps_malloc(kMaxLights * sizeof(LightConfig),
                                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!g_config.lights) {
            g_config.lights = static_cast<LightConfig *>(calloc(kMaxLights, sizeof(LightConfig)));
        } else {
            memset(g_config.lights, 0, kMaxLights * sizeof(LightConfig));
        }
    }
    g_config.gateway_port = kDefaultGatewayPort;
    g_config.scan_interval_sec = kDefaultScanIntervalSec;
}

void load_default_states()
{
    if (!g_light_states) {
        g_light_states = static_cast<LightState *>(heap_caps_malloc(kMaxLights * sizeof(LightState),
                                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!g_light_states) {
            g_light_states = static_cast<LightState *>(calloc(kMaxLights, sizeof(LightState)));
        }
    }
    if (!g_endpoint_ids) {
        g_endpoint_ids = static_cast<uint16_t *>(heap_caps_malloc(kMaxLights * sizeof(uint16_t),
                                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!g_endpoint_ids) {
            g_endpoint_ids = static_cast<uint16_t *>(calloc(kMaxLights, sizeof(uint16_t)));
        }
    }
    memset(g_light_states, 0, kMaxLights * sizeof(LightState));
    memset(g_endpoint_ids, 0, kMaxLights * sizeof(uint16_t));
    for (uint8_t i = 0; i < kMaxLights; ++i) {
        g_light_states[i].last_err = ESP_OK;
    }
}

char *config_to_json()
{
    cJSON *root = cJSON_CreateObject();
    cJSON *gateway = cJSON_AddObjectToObject(root, "gateway");
    cJSON_AddStringToObject(gateway, "host", g_config.gateway_host);
    cJSON_AddNumberToObject(gateway, "port", g_config.gateway_port);
    cJSON_AddNumberToObject(gateway, "scan_interval", g_config.scan_interval_sec);
    cJSON_AddStringToObject(gateway, "name", g_config.gateway_name);
    cJSON_AddStringToObject(gateway, "gateway_ip", g_config.gateway_ip);

    cJSON *lights = cJSON_AddArrayToObject(root, "lights");
    for (uint8_t i = 0; i < g_config.light_count; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "id", g_config.lights[i].id);
        cJSON_AddStringToObject(item, "name", g_config.lights[i].name);
        cJSON_AddStringToObject(item, "serial_hex", g_config.lights[i].serial_hex);
        cJSON_AddStringToObject(item, "model", g_config.lights[i].model);
        cJSON_AddStringToObject(item, "extended_model", g_config.lights[i].extended_model);
        cJSON_AddNumberToObject(item, "device_id", g_config.lights[i].device_id);
        cJSON_AddNumberToObject(item, "hsnet_id", g_config.lights[i].hsnet_id);
        cJSON_AddNumberToObject(item, "dev_model", g_config.lights[i].dev_model);
        cJSON_AddNumberToObject(item, "channel", g_config.lights[i].channel);
        cJSON_AddBoolToObject(item, "supports_brightness", g_config.lights[i].supports_brightness);
        cJSON_AddItemToArray(lights, item);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

bool parse_config_json(const char *json, AppConfig *out)
{
    if (!json || !out) {
        return false;
    }
    // Preserve lights pointer (allocated by caller) across zeroing the struct
    LightConfig *saved_lights = out->lights;
    memset(out, 0, sizeof(*out));
    out->lights = saved_lights;
    out->gateway_port = kDefaultGatewayPort;
    out->scan_interval_sec = kDefaultScanIntervalSec;

    cJSON *root = cJSON_Parse(json);
    if (!root) {
        return false;
    }

    cJSON *gateway = cJSON_GetObjectItemCaseSensitive(root, "gateway");
    if (cJSON_IsObject(gateway)) {
        copy_string(out->gateway_host, sizeof(out->gateway_host), read_json_string(gateway, "host"));
        copy_string(out->gateway_name, sizeof(out->gateway_name), read_json_string(gateway, "name"));
        copy_string(out->gateway_ip, sizeof(out->gateway_ip), read_json_string(gateway, "gateway_ip"));
        out->gateway_port = static_cast<uint16_t>(std::clamp(read_json_int(gateway, "port", kDefaultGatewayPort), 1, 65535));
        out->scan_interval_sec =
            static_cast<uint16_t>(std::clamp(read_json_int(gateway, "scan_interval", kDefaultScanIntervalSec), 5, 3600));
    }

    cJSON *lights = cJSON_GetObjectItemCaseSensitive(root, "lights");
    if (cJSON_IsArray(lights)) {
        cJSON *item = nullptr;
        cJSON_ArrayForEach(item, lights) {
            if (out->light_count >= kMaxLights) {
                break;
            }
            LightConfig &light = out->lights[out->light_count++];
            copy_string(light.id, sizeof(light.id), read_json_string(item, "id"));
            copy_string(light.name, sizeof(light.name), read_json_string(item, "name"));
            copy_string(light.serial_hex, sizeof(light.serial_hex), read_json_string(item, "serial_hex"));
            copy_string(light.model, sizeof(light.model), read_json_string(item, "model"));
            copy_string(light.extended_model, sizeof(light.extended_model), read_json_string(item, "extended_model"));
            light.device_id = static_cast<uint16_t>(read_json_int(item, "device_id", 0));
            light.hsnet_id = static_cast<uint16_t>(read_json_int(item, "hsnet_id", 0));
            light.dev_model = static_cast<uint8_t>(read_json_int(item, "dev_model", 0));
            light.channel = static_cast<uint8_t>(std::clamp(read_json_int(item, "channel", 1), 1, 255));
            light.supports_brightness = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(item, "supports_brightness"));
        }
    }

    cJSON_Delete(root);
    return true;
}

esp_err_t save_config()
{
    char *json = config_to_json();
    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        const size_t json_size = strlen(json) + 1;
        err = nvs_set_blob(handle, kNvsConfigKey, json, json_size);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }

    cJSON_free(json);
    return err;
}

void load_config()
{
    load_default_config();

    nvs_handle_t handle;
    if (nvs_open(kNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }

    size_t size = 0;
    esp_err_t err = nvs_get_blob(handle, kNvsConfigKey, nullptr, &size);
    if (err == ESP_OK && size >= 2) {
        char *json = static_cast<char *>(calloc(1, size));
        if (!json) {
            nvs_close(handle);
            return;
        }

        err = nvs_get_blob(handle, kNvsConfigKey, json, &size);
        if (err == ESP_OK) {
            AppConfig *parsed = static_cast<AppConfig *>(calloc(1, sizeof(AppConfig)));
            if (parsed) {
                parsed->lights = static_cast<LightConfig *>(heap_caps_malloc(kMaxLights * sizeof(LightConfig),
                                                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
                if (!parsed->lights) {
                    parsed->lights = static_cast<LightConfig *>(calloc(kMaxLights, sizeof(LightConfig)));
                } else {
                    memset(parsed->lights, 0, kMaxLights * sizeof(LightConfig));
                }
                if (parse_config_json(json, parsed)) {
                    if (g_config.lights) {
                        free(g_config.lights);
                    }
                    g_config = *parsed; // transfer pointer ownership
                }
                free(parsed); // do not free parsed->lights; ownership moved to g_config
            }
        }

        free(json);
        nvs_close(handle);
        return;
    }

    size = 0;
    err = nvs_get_str(handle, kNvsConfigKey, nullptr, &size);
    if (err == ESP_OK && size >= 2) {
        char *json = static_cast<char *>(calloc(1, size));
        if (!json) {
            nvs_close(handle);
            return;
        }

        err = nvs_get_str(handle, kNvsConfigKey, json, &size);
        if (err == ESP_OK) {
            AppConfig *parsed = static_cast<AppConfig *>(calloc(1, sizeof(AppConfig)));
            if (parsed) {
                parsed->lights = static_cast<LightConfig *>(heap_caps_malloc(kMaxLights * sizeof(LightConfig),
                                                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
                if (!parsed->lights) {
                    parsed->lights = static_cast<LightConfig *>(calloc(kMaxLights, sizeof(LightConfig)));
                } else {
                    memset(parsed->lights, 0, kMaxLights * sizeof(LightConfig));
                }
                if (parse_config_json(json, parsed)) {
                    if (g_config.lights) {
                        free(g_config.lights);
                    }
                    g_config = *parsed; // transfer pointer ownership
                }
                free(parsed); // do not free parsed->lights; ownership moved to g_config
            }
        }

        free(json);
    }

    nvs_close(handle);
}

bool parse_processor_response(const uint8_t *data, size_t length, const char *source_ip, ProcessorInfo *out)
{
    if (!data || !out || length < 15 || memcmp(data, kHeader, 9) != 0 || data[9] != 3 || data[10] != 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    copy_string(out->source_ip, sizeof(out->source_ip), source_ip);
    copy_string(out->name, sizeof(out->name), c_string_at(data, length, 12, 20));
    if (length >= 82) {
        snprintf(out->version, sizeof(out->version), "%u.%u.%u.%u", data[32], data[33], data[34], data[35]);
        copy_string(out->serial, sizeof(out->serial), c_string_at(data, length, 42, 16));
        snprintf(out->ip, sizeof(out->ip), "%u.%u.%u.%u", data[62], data[63], data[64], data[65]);
        snprintf(out->mask, sizeof(out->mask), "%u.%u.%u.%u", data[66], data[67], data[68], data[69]);
        snprintf(out->gateway, sizeof(out->gateway), "%u.%u.%u.%u", data[70], data[71], data[72], data[73]);
        snprintf(out->mac, sizeof(out->mac), "%02X:%02X:%02X:%02X:%02X:%02X", data[74], data[75], data[76], data[77],
                 data[78], data[79]);
    }
    return true;
}

bool parse_devices_response(const uint8_t *data, size_t length, const char *source_ip, DeviceInfo *devices, size_t capacity,
                            size_t *out_count, uint16_t *out_qty, uint16_t *out_read_index)
{
    if (!data || !devices || !out_count || !out_qty || !out_read_index || length <= 22 || memcmp(data, kHeader, 9) != 0 ||
        data[9] != 100 || data[10] != 1) {
        return false;
    }

    const uint8_t header_len = data[12];
    const uint8_t register_len = data[13];
    const uint8_t registers_qty = data[14];
    const uint16_t read_index = static_cast<uint16_t>(data[16] | (data[17] << 8));
    const size_t base = 12 + header_len;

    size_t count = 0;
    for (uint8_t i = 0; i < registers_qty && count < capacity; ++i) {
        const size_t pos = base + static_cast<size_t>(i) * register_len;
        if (pos + 40 > length) {
            break;
        }
        DeviceInfo &device = devices[count++];
        memset(&device, 0, sizeof(device));
        copy_string(device.processor_ip, sizeof(device.processor_ip), source_ip);
        device.status = data[pos];
        device.port = data[pos + 1];
        device.hsnet_id = static_cast<uint16_t>(data[pos + 3] | (data[pos + 4] << 8));
        device.device_id = static_cast<uint16_t>(data[pos + 5] | (data[pos + 6] << 8));
        device.dev_model = data[pos + 7];
        snprintf(device.fw, sizeof(device.fw), "%u.%u.%u", data[pos + 8], data[pos + 9], data[pos + 10]);
        copy_string(device.model, sizeof(device.model), c_string_at(data, length, pos + 11, 7));
        copy_string(device.extended_model, sizeof(device.extended_model), c_string_at(data, length, pos + 18, 10));
        snprintf(device.serial_hex, sizeof(device.serial_hex), "%02X:%02X:%02X:%02X:%02X:%02X", data[pos + 28],
                 data[pos + 29], data[pos + 30], data[pos + 31], data[pos + 32], data[pos + 33]);
        device.crc = static_cast<uint16_t>((data[pos + 34] << 8) | data[pos + 35]);
        device.eeprom_address = static_cast<uint16_t>(data[pos + 36] | (data[pos + 37] << 8));
        device.bitmap = static_cast<uint16_t>(data[pos + 38] | (data[pos + 39] << 8));
    }

    *out_count = count;
    *out_qty = registers_qty;
    *out_read_index = read_index;
    return true;
}

bool udp_request(const char *host, uint16_t port, const uint8_t *payload, size_t payload_len, uint8_t *response,
                 size_t response_capacity, size_t *response_len, char *source_ip, size_t source_ip_size, int timeout_ms)
{
    if (!host || !host[0] || !payload || payload_len == 0 || !response || response_capacity == 0 || !response_len) {
        return false;
    }

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    char port_text[8];
    snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(port));

    struct addrinfo *result = nullptr;
    if (getaddrinfo(host, port_text, &hints, &result) != 0 || !result) {
        return false;
    }

    int sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock < 0) {
        freeaddrinfo(result);
        return false;
    }

    struct timeval timeout = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    bool ok = false;
    if (sendto(sock, payload, payload_len, 0, result->ai_addr, result->ai_addrlen) >= 0) {
        struct sockaddr_in from_addr = {};
        socklen_t from_len = sizeof(from_addr);
        const ssize_t received = recvfrom(sock, response, response_capacity, 0, reinterpret_cast<struct sockaddr *>(&from_addr), &from_len);
        if (received > 0) {
            *response_len = static_cast<size_t>(received);
            if (source_ip && source_ip_size > 0) {
                inet_ntop(AF_INET, &from_addr.sin_addr, source_ip, source_ip_size);
            }
            ok = true;
        }
    }

    close(sock);
    freeaddrinfo(result);
    return ok;
}

bool query_processor_info(const char *host, uint16_t port, ProcessorInfo *out)
{
    const uint8_t packet[] = {'H', 'S', 'N', '_', 'S', '-', 'U', 'D', 'P', 3, 0, 0, 0};
    uint8_t *response = static_cast<uint8_t *>(calloc(1, 512));
    if (!response) {
        return false;
    }
    size_t response_len = 0;
    char source_ip[16] = {};
    if (!udp_request(host, port, packet, sizeof(packet), response, 512, &response_len, source_ip, sizeof(source_ip),
                     kDefaultUdpTimeoutMs)) {
        free(response);
        return false;
    }
    const bool ok = parse_processor_response(response, response_len, source_ip, out);
    free(response);
    return ok;
}

size_t query_devices(const char *host, uint16_t port, DeviceInfo *devices, size_t capacity)
{
    if (!devices || capacity == 0) {
        return 0;
    }

    size_t total = 0;
    uint16_t read_index = 0;
    for (uint8_t page = 0; page < 32 && total < capacity; ++page) {
        const uint16_t previous_index = read_index;
        const uint8_t packet[] = {'H', 'S', 'N', '_', 'S', '-', 'U', 'D', 'P', 100, 1,
                                  static_cast<uint8_t>(read_index & 0xFF), static_cast<uint8_t>((read_index >> 8) & 0xFF), 0};
        uint8_t *response = static_cast<uint8_t *>(calloc(1, 2048));
        DeviceInfo *page_devices = static_cast<DeviceInfo *>(calloc(24, sizeof(DeviceInfo)));
        if (!response || !page_devices) {
            free(response);
            free(page_devices);
            break;
        }
        size_t response_len = 0;
        char source_ip[16] = {};
        if (!udp_request(host, port, packet, sizeof(packet), response, 2048, &response_len, source_ip, sizeof(source_ip),
                         kDefaultUdpTimeoutMs)) {
            free(response);
            free(page_devices);
            break;
        }

        size_t page_count = 0;
        uint16_t qty = 0;
        uint16_t current_index = 0;
        if (!parse_devices_response(response, response_len, source_ip, page_devices, 24, &page_count, &qty, &current_index)) {
            free(response);
            free(page_devices);
            break;
        }

        for (size_t i = 0; i < page_count && total < capacity; ++i) {
            devices[total++] = page_devices[i];
        }
        free(response);
        free(page_devices);
        if (qty < 24 || page_count == 0 || current_index == previous_index) {
            break;
        }
        read_index = current_index + qty;
    }

    return total;
}

bool collect_socket_lines(int sock, char *lines, size_t lines_size, int timeout_ms)
{
    if (sock < 0 || !lines || lines_size == 0) {
        return false;
    }

    lines[0] = '\0';
    size_t offset = 0;
    bool saw_data = false;
    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
    while (offset + 1 < lines_size && esp_timer_get_time() < deadline_us) {
        const ssize_t received = recv(sock, lines + offset, lines_size - offset - 1, 0);
        if (received > 0) {
            offset += static_cast<size_t>(received);
            lines[offset] = '\0';
            saw_data = true;
            continue;
        }
        if (received == 0) {
            break;
        }
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            break;
        }
        return false;
    }
    return saw_data;
}

bool send_socket_line(int sock, const char *command)
{
    if (sock < 0 || !command || !command[0]) {
        return false;
    }

    char payload[96];
    snprintf(payload, sizeof(payload), "%s\r\n", command);
    return send(sock, payload, strlen(payload), 0) >= 0;
}

bool connect_with_timeout(int sock, const struct addrinfo *result, int timeout_ms)
{
    if (sock < 0 || !result) {
        return false;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        flags = 0;
    }
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    const int connect_result = connect(sock, result->ai_addr, result->ai_addrlen);
    if (connect_result == 0) {
        fcntl(sock, F_SETFL, flags);
        return true;
    }
    if (errno != EINPROGRESS) {
        return false;
    }

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock, &write_fds);

    struct timeval timeout = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    const int select_result = select(sock + 1, nullptr, &write_fds, nullptr, &timeout);
    if (select_result <= 0) {
        return false;
    }

    int socket_error = 0;
    socklen_t socket_error_len = sizeof(socket_error);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) != 0 || socket_error != 0) {
        return false;
    }

    fcntl(sock, F_SETFL, flags);
    return true;
}

bool send_text_command_lines(const char *host, uint16_t port, const char *command, char *lines, size_t lines_size)
{
    if (!host || !host[0] || !command || !lines || lines_size == 0) {
        return false;
    }

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char port_text[8];
    snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(port));

    struct addrinfo *result = nullptr;
    if (getaddrinfo(host, port_text, &hints, &result) != 0 || !result) {
        return false;
    }

    int sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock < 0) {
        freeaddrinfo(result);
        return false;
    }

    // Use the response timeout for socket I/O, not the (much longer) connect timeout.
    struct timeval rw_timeout = {
        .tv_sec = kDefaultCommandResponseTimeoutMs / 1000,
        .tv_usec = (kDefaultCommandResponseTimeoutMs % 1000) * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rw_timeout, sizeof(rw_timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &rw_timeout, sizeof(rw_timeout));

    bool ok = false;
    if (connect_with_timeout(sock, result, kDefaultCommandConnectTimeoutMs)) {
        ESP_LOGI(kTag, "TCP SEND: %s", command);

        if (send_socket_line(sock, command)) {
            ok = true; // command was accepted by TCP stack
            collect_socket_lines(sock, lines, lines_size, kDefaultCommandResponseTimeoutMs);
            ESP_LOGI(kTag, "TCP RX: %s", lines[0] ? lines : "(none)");
        }
    }

    close(sock);
    freeaddrinfo(result);
    return ok;
}

bool parse_load_line(const char *line, uint16_t *address, uint8_t *channel, uint8_t *level)
{
    if (!line || strncmp(line, "R:LOAD ", 7) != 0) {
        return false;
    }
    int parsed_address = 0;
    int parsed_channel = 0;
    int parsed_level = 0;
    if (sscanf(line, "R:LOAD %d %d %d", &parsed_address, &parsed_channel, &parsed_level) != 3) {
        return false;
    }
    if (address) {
        *address = static_cast<uint16_t>(parsed_address);
    }
    if (channel) {
        *channel = static_cast<uint8_t>(parsed_channel);
    }
    if (level) {
        *level = static_cast<uint8_t>(std::clamp(parsed_level, 0, 100));
    }
    return true;
}

// ── Persistent TCP connection helpers (mirrors roehn_dinplug architecture) ────────
// The Roehn gateway accepts only ONE TCP connection on port 23.  All commands AND
// event feedback flow through this single persistent socket.  g_event_lock (mutex)
// serialises all socket I/O so the event-listener task and the command task never
// race on recv().

static void close_event_socket()
{
    if (g_event_lock) {
        xSemaphoreTake(g_event_lock, portMAX_DELAY);
    }
    if (g_event_sock >= 0) {
        close(g_event_sock);
        g_event_sock = -1;
    }
    if (g_event_lock) {
        xSemaphoreGive(g_event_lock);
    }
}

// Send a CRLF-terminated line and read the response through the persistent socket.
// Returns true if the command was written to the socket; *out_lines receives any
// text that came back before the deadline.
static bool send_command_persistent(const char *command, char *out_lines, size_t out_size)
{
    if (!command || !out_lines || out_size == 0) {
        return false;
    }
    out_lines[0] = '\0';

    // Acquire the socket lock — the event listener will wait.
    if (!g_event_lock || xSemaphoreTake(g_event_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ESP_LOGW(kTag, "Persistent socket lock timeout for '%s'", command);
        return false;
    }

    bool ok = false;
    if (g_event_sock >= 0) {
        // Send
        char payload[96];
        snprintf(payload, sizeof(payload), "%s\r\n", command);
        if (send(g_event_sock, payload, strlen(payload), 0) >= 0) {
            ok = true;

            // Read response with a short deadline
            struct timeval orig_tv;
            socklen_t optlen = sizeof(orig_tv);
            getsockopt(g_event_sock, SOL_SOCKET, SO_RCVTIMEO, &orig_tv, &optlen);

            struct timeval short_tv = {
                .tv_sec = kDefaultCommandResponseTimeoutMs / 1000,
                .tv_usec = (kDefaultCommandResponseTimeoutMs % 1000) * 1000,
            };
            setsockopt(g_event_sock, SOL_SOCKET, SO_RCVTIMEO, &short_tv, sizeof(short_tv));

            size_t offset = 0;
            const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(kDefaultCommandResponseTimeoutMs) * 1000;
            while (offset + 1 < out_size && esp_timer_get_time() < deadline_us) {
                const ssize_t n = recv(g_event_sock, out_lines + offset, out_size - offset - 1, 0);
                if (n > 0) {
                    offset += static_cast<size_t>(n);
                    out_lines[offset] = '\0';
                    continue;
                }
                if (n == 0 || errno == ECONNRESET || errno == ENOTCONN) {
                    break;
                }
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    break;
                }
                break; // hard error
            }

            setsockopt(g_event_sock, SOL_SOCKET, SO_RCVTIMEO, &orig_tv, sizeof(orig_tv));
        }
    }

    xSemaphoreGive(g_event_lock);

    if (!ok || g_event_sock < 0) {
        // Persistent path not available — try a short-lived connection as fallback
        ESP_LOGW(kTag, "Persistent socket unavailable, falling back to short-lived TCP for '%s'", command);
        return send_text_command_lines(g_config.gateway_host, kDefaultCommandPort, command, out_lines, out_size);
    }
    return ok;
}

esp_err_t roehn_set_load(const LightConfig &light, uint8_t level_percent)
{
    char command[64];
    const uint8_t level = std::min<uint8_t>(100, level_percent);
    snprintf(command, sizeof(command), "LOAD %u %u %u",
             static_cast<unsigned>(resolve_control_address(light)),
             static_cast<unsigned>(light.channel),
             static_cast<unsigned>(level));
    ESP_LOGI(kTag, "ROEHN TX: %s", command);

    char lines[1024] = {};
    if (!send_command_persistent(command, lines, sizeof(lines))) {
        ESP_LOGW(kTag, "ROEHN LOAD failed: cmd='%s' rx='%s'", command, lines);
        return ESP_FAIL;
    }
    ESP_LOGI(kTag, "ROEHN LOAD sent: cmd='%s' rx='%s'", command, lines[0] ? lines : "(none)");
    return ESP_OK;
}

// ── Event listener task ──────────────────────────────────────────────────────────

static void sync_light_state_to_matter(uint8_t index);
static bool roehn_query_load(const LightConfig &light, uint8_t *out_level);

// Parse a single feedback line received from the gateway and update state.
static void process_event_line(const char *line)
{
    if (!line || !line[0]) {
        return;
    }

    // R:LOAD <device_address> <channel> <level>
    if (strncmp(line, "R:LOAD ", 7) == 0) {
        uint16_t address = 0;
        uint8_t channel = 0;
        uint8_t level = 0;
        if (!parse_load_line(line, &address, &channel, &level)) {
            return;
        }

        for (uint8_t i = 0; i < g_config.light_count; ++i) {
            if (resolve_control_address(g_config.lights[i]) == address &&
                g_config.lights[i].channel == channel) {
                g_light_states[i].level_percent = level;
                g_light_states[i].on = level > 0;
                sync_light_state_to_matter(i);
                ESP_LOGI(kTag, "Event R:LOAD addr=%u ch=%u level=%u → light[%u] '%s'",
                         static_cast<unsigned>(address),
                         static_cast<unsigned>(channel),
                         static_cast<unsigned>(level),
                         static_cast<unsigned>(i),
                         g_config.lights[i].name);
                break;
            }
        }
    }

    // R:MODULE STATUS <hsnet_id> <status> — trigger immediate load query for this module
    if (strncmp(line, "R:MODULE STATUS ", 16) == 0) {
        int hsnet_id = 0, status = 0;
        if (sscanf(line, "R:MODULE STATUS %d %d", &hsnet_id, &status) == 2 && hsnet_id > 0) {
            for (uint8_t i = 0; i < g_config.light_count; ++i) {
                if (resolve_control_address(g_config.lights[i]) == static_cast<uint16_t>(hsnet_id)) {
                    uint8_t level = 0;
                    if (roehn_query_load(g_config.lights[i], &level)) {
                        g_light_states[i].level_percent = level;
                        g_light_states[i].on = level > 0;
                        sync_light_state_to_matter(i);
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }
    }
}

static void event_listener_task(void *)
{
    ESP_LOGI(kTag, "Roehn event listener task started");

    while (true) {
        // Ensure clean state before connecting.
        close_event_socket();

        if (!g_config.gateway_host[0]) {
            ESP_LOGI(kTag, "Event listener: no gateway configured, waiting...");
            vTaskDelay(pdMS_TO_TICKS(kEventReconnectDelayMs));
            continue;
        }

        // Connect to the Roehn gateway telnet port.
        struct addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        char port_text[8];
        snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(kDefaultCommandPort));

        struct addrinfo *result = nullptr;
        if (getaddrinfo(g_config.gateway_host, port_text, &hints, &result) != 0 || !result) {
            ESP_LOGW(kTag, "Event listener: DNS resolution failed for %s, retrying in %d ms",
                     g_config.gateway_host, kEventReconnectDelayMs);
            vTaskDelay(pdMS_TO_TICKS(kEventReconnectDelayMs));
            continue;
        }

        int sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        if (sock < 0) {
            freeaddrinfo(result);
            ESP_LOGW(kTag, "Event listener: socket() failed, retrying in %d ms", kEventReconnectDelayMs);
            vTaskDelay(pdMS_TO_TICKS(kEventReconnectDelayMs));
            continue;
        }

        if (!connect_with_timeout(sock, result, kDefaultCommandConnectTimeoutMs)) {
            close(sock);
            freeaddrinfo(result);
            ESP_LOGW(kTag, "Event listener: connect to %s:%u failed, retrying in %d ms",
                     g_config.gateway_host, static_cast<unsigned>(kDefaultCommandPort), kEventReconnectDelayMs);
            vTaskDelay(pdMS_TO_TICKS(kEventReconnectDelayMs));
            continue;
        }

        freeaddrinfo(result);

        // Publish the socket for command use.
        if (g_event_lock) {
            xSemaphoreTake(g_event_lock, portMAX_DELAY);
        }
        g_event_sock = sock;
        if (g_event_lock) {
            xSemaphoreGive(g_event_lock);
        }

        ESP_LOGI(kTag, "Event listener connected to %s:%u",
                 g_config.gateway_host, static_cast<unsigned>(kDefaultCommandPort));

        // Read lines indefinitely — use MSG_DONTWAIT so we never block while
        // holding the socket lock.  The command path acquires the same lock,
        // sends a command, reads the response, and releases.
        char buffer[1024];
        size_t buf_len = 0;

        while (true) {
            // Try to acquire the lock.  If a command is in flight we wait briefly.
            if (!g_event_lock || xSemaphoreTake(g_event_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            char ch;
            const ssize_t n = recv(sock, &ch, 1, MSG_DONTWAIT);

            if (n > 0) {
                xSemaphoreGive(g_event_lock);
                if (ch == '\n') {
                    if (buf_len > 0 && buffer[buf_len - 1] == '\r') {
                        buf_len--;
                    }
                    buffer[buf_len] = '\0';
                    if (buf_len > 0) {
                        ESP_LOGI(kTag, "EVENT RX: %s", buffer);
                        process_event_line(buffer);
                    }
                    buf_len = 0;
                } else if (buf_len + 1 < sizeof(buffer)) {
                    buffer[buf_len++] = ch;
                }
            } else {
                xSemaphoreGive(g_event_lock);
                if (n == 0 || errno == ECONNRESET || errno == ENOTCONN) {
                    ESP_LOGW(kTag, "Event listener: connection closed by gateway");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
                } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
                    ESP_LOGW(kTag, "Event listener: recv error %d, reconnecting", errno);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50)); // idle
            }
        }

        // Socket will be cleaned up at top of loop.
    }
}

bool roehn_query_load(const LightConfig &light, uint8_t *out_level)
{
    char command[64];
    snprintf(command, sizeof(command), "GETLOAD %u %u", static_cast<unsigned>(resolve_control_address(light)),
             static_cast<unsigned>(light.channel));

    char lines[1024] = {};
    if (!send_command_persistent(command, lines, sizeof(lines))) {
        return false;
    }

    char *saveptr = nullptr;
    for (char *line = strtok_r(lines, "\r\n", &saveptr); line; line = strtok_r(nullptr, "\r\n", &saveptr)) {
        uint16_t address = 0;
        uint8_t channel = 0;
        uint8_t level = 0;
        if (!parse_load_line(line, &address, &channel, &level)) {
            continue;
        }
        if (address == resolve_control_address(light) && channel == light.channel) {
            if (out_level) {
                *out_level = level;
            }
            return true;
        }
    }
    return false;
}

bool load_resources_index()
{
    if (g_resources.loaded) {
        return g_resources.module_count > 0;
    }

    const size_t json_len = static_cast<size_t>(kModuleDriversJsonEnd - kModuleDriversJsonStart);
    if (json_len == 0) {
        return false;
    }

    cJSON *root = cJSON_ParseWithLength(reinterpret_cast<const char *>(kModuleDriversJsonStart), json_len);
    if (!root || !cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return false;
    }

    g_resources = {};
    cJSON *item = nullptr;
    cJSON_ArrayForEach(item, root) {
        if (g_resources.module_count >= sizeof(g_resources.modules) / sizeof(g_resources.modules[0])) {
            break;
        }

        ModuleDriverInfo &driver = g_resources.modules[g_resources.module_count++];
        copy_string(driver.model_base_name, sizeof(driver.model_base_name), read_json_string(item, "modelBaseName"));
        copy_string(driver.firmware_name, sizeof(driver.firmware_name), read_json_string(item, "FirmwareName"));
        copy_string(driver.firmware_extended_name, sizeof(driver.firmware_extended_name), read_json_string(item, "FirmwareExtendedName"));
        driver.dev_model = read_json_int(item, "Type", -1);

        cJSON *slots = cJSON_GetObjectItemCaseSensitive(item, "Slots");
        if (cJSON_IsArray(slots)) {
            cJSON *slot_item = nullptr;
            cJSON_ArrayForEach(slot_item, slots) {
                if (driver.slot_count >= sizeof(driver.slots) / sizeof(driver.slots[0])) {
                    break;
                }
                SlotInfo &slot = driver.slots[driver.slot_count++];
                slot.initial_port = static_cast<uint8_t>(std::max(0, read_json_int(slot_item, "InitialPort", 0)));
                slot.capacity = static_cast<uint8_t>(std::max(0, read_json_int(slot_item, "SlotCapacity", 0)));
                slot.slot_type = static_cast<uint8_t>(std::max(0, read_json_int(slot_item, "SlotType", 0)));
                switch (slot.slot_type) {
                case 1:
                    copy_string(slot.slot_name, sizeof(slot.slot_name), "relay");
                    break;
                case 2:
                    copy_string(slot.slot_name, sizeof(slot.slot_name), "dimmer");
                    break;
                case 3:
                    copy_string(slot.slot_name, sizeof(slot.slot_name), "key");
                    break;
                case 7:
                    copy_string(slot.slot_name, sizeof(slot.slot_name), "shade");
                    break;
                default:
                    copy_string(slot.slot_name, sizeof(slot.slot_name), "unknown");
                    break;
                }
            }
        }
    }

    g_resources.loaded = true;
    cJSON_Delete(root);
    ESP_LOGI(kTag, "Loaded %u Roehn module driver definitions", g_resources.module_count);
    return g_resources.module_count > 0;
}

const ModuleDriverInfo *lookup_driver(const DeviceInfo &device)
{
    if (!load_resources_index()) {
        return nullptr;
    }

    char model_token[64];
    char extended_token[64];
    uppercase_alnum_token(device.model, model_token, sizeof(model_token));
    uppercase_alnum_token(device.extended_model, extended_token, sizeof(extended_token));

    for (uint16_t i = 0; i < g_resources.module_count; ++i) {
        const ModuleDriverInfo &driver = g_resources.modules[i];
        char token[64];
        uppercase_alnum_token(driver.firmware_extended_name, token, sizeof(token));
        if (extended_token[0] && token[0] && strcmp(extended_token, token) == 0) {
            return &driver;
        }
        uppercase_alnum_token(driver.firmware_name, token, sizeof(token));
        if (model_token[0] && token[0] && strcmp(model_token, token) == 0) {
            return &driver;
        }
        uppercase_alnum_token(driver.model_base_name, token, sizeof(token));
        if (model_token[0] && token[0] && strcmp(model_token, token) == 0) {
            return &driver;
        }
        if (driver.dev_model >= 0 && driver.dev_model == device.dev_model) {
            return &driver;
        }
    }
    return nullptr;
}

void log_driver_slots(const ModuleDriverInfo *driver)
{
    if (!driver) {
        return;
    }

    ESP_LOGI(kTag, "Driver match: type=%d fw=%s ext=%s slots=%u", driver->dev_model, driver->firmware_name,
             driver->firmware_extended_name, static_cast<unsigned>(driver->slot_count));
    for (uint8_t slot_index = 0; slot_index < driver->slot_count; ++slot_index) {
        const SlotInfo &slot = driver->slots[slot_index];
        ESP_LOGI(kTag, "  slot[%u]: type=%u name=%s initial=%u capacity=%u", static_cast<unsigned>(slot_index),
                 static_cast<unsigned>(slot.slot_type), slot.slot_name, static_cast<unsigned>(slot.initial_port),
                 static_cast<unsigned>(slot.capacity));
    }
}

const FallbackDriverSpec *lookup_fallback_light_driver(const DeviceInfo &device)
{
    for (const auto &spec : kFallbackLightDrivers) {
        if (spec.dev_model == device.dev_model) {
            return &spec;
        }
    }
    return nullptr;
}

size_t describe_light_entities(const DeviceInfo *devices, size_t device_count, LightConfig *out_lights, size_t capacity)
{
    size_t total = 0;
    for (size_t i = 0; i < device_count && total < capacity; ++i) {
        const DeviceInfo &device = devices[i];
        ESP_LOGI(kTag, "Device[%u]: dev_model=%u model=%s ext=%s device_id=%u hsnet_id=%u serial=%s",
                 static_cast<unsigned>(i), static_cast<unsigned>(device.dev_model), device.model, device.extended_model,
                 static_cast<unsigned>(device.device_id), static_cast<unsigned>(device.hsnet_id), device.serial_hex);
        const ModuleDriverInfo *driver = lookup_driver(device);
        if (!driver) {
            const FallbackDriverSpec *fallback = lookup_fallback_light_driver(device);
            if (!fallback) {
                ESP_LOGW(kTag, "  No driver match for device[%u]", static_cast<unsigned>(i));
                continue;
            }

            ESP_LOGW(kTag, "  Using fallback driver for dev_model=%u fw=%s ext=%s", static_cast<unsigned>(device.dev_model),
                     fallback->firmware_name, fallback->firmware_extended_name);

            size_t device_light_count = 0;
            for (uint8_t offset = 0; offset < fallback->capacity && total < capacity; ++offset) {
                const uint8_t channel = static_cast<uint8_t>(fallback->initial_port + offset);
                LightConfig &light = out_lights[total++];
                device_light_count++;
                memset(&light, 0, sizeof(light));
                light.device_id = device.device_id;
                light.hsnet_id = device.hsnet_id;
                light.dev_model = device.dev_model;
                light.channel = channel;
                light.supports_brightness = fallback->slot_type == 2;
                copy_string(light.serial_hex, sizeof(light.serial_hex), device.serial_hex);
                copy_string(light.model, sizeof(light.model), device.model);
                copy_string(light.extended_model, sizeof(light.extended_model), device.extended_model);

                char serial_token[24];
                size_t token_len = 0;
                for (size_t j = 0; device.serial_hex[j] != '\0' && token_len + 1 < sizeof(serial_token); ++j) {
                    if (device.serial_hex[j] != ':') {
                        serial_token[token_len++] = static_cast<char>(tolower(static_cast<unsigned char>(device.serial_hex[j])));
                    }
                }
                serial_token[token_len] = '\0';

                snprintf(light.id, sizeof(light.id), "%s-%u", serial_token[0] ? serial_token : "light",
                         static_cast<unsigned>(channel));
                snprintf(light.name, sizeof(light.name), "%u-%s-%u",
                         static_cast<unsigned>(device.hsnet_id > 0 ? device.hsnet_id : device.device_id),
                         device.model[0] ? device.model : "LIGHT",
                         static_cast<unsigned>(channel));
                ESP_LOGI(kTag, "  Added fallback light entity: id=%s channel=%u brightness=%s addr=%u", light.id,
                         static_cast<unsigned>(light.channel), light.supports_brightness ? "yes" : "no",
                         static_cast<unsigned>(resolve_control_address(light)));
            }
            ESP_LOGI(kTag, "  Device[%u] produced %u fallback light entities", static_cast<unsigned>(i),
                     static_cast<unsigned>(device_light_count));
            continue;
        }
        log_driver_slots(driver);

        bool seen_channels[256] = {};
        size_t device_light_count = 0;
        for (uint8_t slot_index = 0; slot_index < driver->slot_count && total < capacity; ++slot_index) {
            const SlotInfo &slot = driver->slots[slot_index];
            const bool is_dimmer = strcmp(slot.slot_name, "dimmer") == 0;
            const bool is_relay = strcmp(slot.slot_name, "relay") == 0;
            if ((!is_dimmer && !is_relay) || slot.capacity == 0) {
                continue;
            }

            const uint8_t start_channel = slot.initial_port > 0 ? slot.initial_port : 1;
            for (uint8_t offset = 0; offset < slot.capacity && total < capacity; ++offset) {
                const uint8_t channel = static_cast<uint8_t>(start_channel + offset);
                if (seen_channels[channel]) {
                    continue;
                }
                seen_channels[channel] = true;

                LightConfig &light = out_lights[total++];
                device_light_count++;
                memset(&light, 0, sizeof(light));
                light.device_id = device.device_id;
                light.hsnet_id = device.hsnet_id;
                light.dev_model = device.dev_model;
                light.channel = channel;
                light.supports_brightness = is_dimmer;
                copy_string(light.serial_hex, sizeof(light.serial_hex), device.serial_hex);
                copy_string(light.model, sizeof(light.model), device.model);
                copy_string(light.extended_model, sizeof(light.extended_model), device.extended_model);

                char serial_token[24];
                size_t token_len = 0;
                for (size_t j = 0; device.serial_hex[j] != '\0' && token_len + 1 < sizeof(serial_token); ++j) {
                    if (device.serial_hex[j] != ':') {
                        serial_token[token_len++] = static_cast<char>(tolower(static_cast<unsigned char>(device.serial_hex[j])));
                    }
                }
                serial_token[token_len] = '\0';

                snprintf(light.id, sizeof(light.id), "%s-%u", serial_token[0] ? serial_token : "light",
                         static_cast<unsigned>(channel));
                snprintf(light.name, sizeof(light.name), "%u-%s-%u",
                         static_cast<unsigned>(device.hsnet_id > 0 ? device.hsnet_id : device.device_id),
                         device.model[0] ? device.model : "LIGHT",
                         static_cast<unsigned>(channel));
                ESP_LOGI(kTag, "  Added light entity: id=%s channel=%u brightness=%s addr=%u", light.id,
                         static_cast<unsigned>(light.channel), light.supports_brightness ? "yes" : "no",
                         static_cast<unsigned>(resolve_control_address(light)));
            }
        }
        ESP_LOGI(kTag, "  Device[%u] produced %u light entities", static_cast<unsigned>(i),
                 static_cast<unsigned>(device_light_count));
    }
    return total;
}

int endpoint_index_from_id(uint16_t endpoint_id)
{
    for (uint8_t i = 0; i < g_config.light_count; ++i) {
        if (g_endpoint_ids[i] == endpoint_id) {
            return i;
        }
    }
    return -1;
}

void sync_light_state_to_matter(uint8_t index)
{
    if (index >= g_config.light_count || g_endpoint_ids[index] == 0) {
        return;
    }

    const uint16_t endpoint_id = g_endpoint_ids[index];
    esp_matter_attr_val_t onoff = esp_matter_bool(g_light_states[index].on);
    esp_matter::attribute::report(endpoint_id, chip::app::Clusters::OnOff::Id,
                                  chip::app::Clusters::OnOff::Attributes::OnOff::Id, &onoff);

    if (g_config.lights[index].supports_brightness && level_cluster_exists(endpoint_id)) {
        const uint8_t matter_level = level_percent_to_matter(g_light_states[index].level_percent);
        esp_matter_attr_val_t level = esp_matter_nullable_uint8(nullable<uint8_t>(matter_level));
        esp_matter::attribute::report(endpoint_id, chip::app::Clusters::LevelControl::Id,
                                      chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, &level);
    }
}

void format_light_node_label(uint8_t index, char *buffer, size_t size)
{
    if (!buffer || size == 0) {
        return;
    }
    if (index >= g_config.light_count) {
        buffer[0] = '\0';
        return;
    }
    copy_string(buffer, size, g_config.lights[index].name);
}

void apply_light_endpoint_metadata(uint8_t index, esp_matter::endpoint_t *endpoint)
{
    if (!endpoint || index >= g_config.light_count) {
        return;
    }

    esp_matter::cluster_t *bridge_cluster =
        esp_matter::cluster::get(endpoint, chip::app::Clusters::BridgedDeviceBasicInformation::Id);
    if (!bridge_cluster) {
        esp_matter::cluster::bridged_device_basic_information::config_t bridge_info = {};
        bridge_cluster =
            esp_matter::cluster::bridged_device_basic_information::create(endpoint, &bridge_info, esp_matter::CLUSTER_FLAG_SERVER);
    }
    if (!bridge_cluster) {
        ESP_LOGW(kTag, "Failed to find Bridged Device Basic Information cluster for %s", g_config.lights[index].name);
        return;
    }

    char node_label[33] = {};
    format_light_node_label(index, node_label, sizeof(node_label));
    if (node_label[0]) {
        esp_matter::cluster::bridged_device_basic_information::attribute::create_node_label(
            bridge_cluster, node_label, strlen(node_label));
        esp_matter_attr_val_t value = esp_matter_char_str(node_label, strlen(node_label));
        esp_matter::attribute::update(esp_matter::endpoint::get_id(endpoint),
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id, &value);
    }

    // Also set Reachable=true so HA marks the bridged endpoint as available
    esp_matter_attr_val_t reachable_val = esp_matter_bool(true);
    esp_matter::attribute::update(esp_matter::endpoint::get_id(endpoint),
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id, &reachable_val);

    // Use the same user-friendly name for ProductName so HA entity names
    // match the new naming convention (e.g. "104-DIM8-1").
    if (node_label[0]) {
        esp_matter::cluster::bridged_device_basic_information::attribute::create_product_name(
            bridge_cluster, node_label, strlen(node_label));
        esp_matter_attr_val_t value = esp_matter_char_str(node_label, strlen(node_label));
        esp_matter::attribute::update(esp_matter::endpoint::get_id(endpoint),
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::ProductName::Id, &value);
    }

    if (g_config.lights[index].serial_hex[0]) {
        esp_matter::cluster::bridged_device_basic_information::attribute::create_serial_number(
            bridge_cluster, g_config.lights[index].serial_hex, strlen(g_config.lights[index].serial_hex));
        esp_matter_attr_val_t value =
            esp_matter_char_str(g_config.lights[index].serial_hex, strlen(g_config.lights[index].serial_hex));
        esp_matter::attribute::update(esp_matter::endpoint::get_id(endpoint),
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                      chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::SerialNumber::Id, &value);
    }

    esp_matter::cluster::fixed_label::config_t fixed_label_config = {};
    if (!esp_matter::cluster::fixed_label::create(endpoint, &fixed_label_config, esp_matter::CLUSTER_FLAG_SERVER)) {
        ESP_LOGW(kTag, "Failed to create Fixed Label cluster for %s", g_config.lights[index].name);
    }
}

void apply_user_labels()
{
    using chip::DeviceLayer::AttributeList;
    using chip::DeviceLayer::DeviceInfoProvider;
    using chip::DeviceLayer::GetDeviceInfoProvider;

    DeviceInfoProvider *provider = GetDeviceInfoProvider();
    if (!provider) {
        return;
    }

    for (uint8_t i = 0; i < g_config.light_count; ++i) {
        if (g_endpoint_ids[i] == 0) {
            continue;
        }
        AttributeList<DeviceInfoProvider::UserLabelType, chip::DeviceLayer::kMaxUserLabelListLength> labels;
        DeviceInfoProvider::UserLabelType name_label = {
            .label = chip::CharSpan::fromCharString("name"),
            .value = chip::CharSpan::fromCharString(g_config.lights[i].name),
        };
        DeviceInfoProvider::UserLabelType role_label = {
            .label = chip::CharSpan::fromCharString("role"),
            .value = chip::CharSpan::fromCharString(g_config.lights[i].supports_brightness ? "Dimmer" : "Light"),
        };
        if (labels.add(name_label) != CHIP_NO_ERROR || labels.add(role_label) != CHIP_NO_ERROR) {
            continue;
        }
        provider->SetUserLabelList(g_endpoint_ids[i], labels);
    }
}

bool refresh_gateway_runtime(bool refresh_loads)
{
    GatewayRuntime next = {};
    copy_string(next.last_error, sizeof(next.last_error), "not_configured");

    if (!g_config.gateway_host[0]) {
        g_gateway = next;
        return false;
    }

    ProcessorInfo processor = {};
    const bool processor_ok = query_processor_info(g_config.gateway_host, g_config.gateway_port, &processor);
    DeviceInfo *devices = static_cast<DeviceInfo *>(calloc(kMaxLights, sizeof(DeviceInfo)));
    if (!devices) {
        copy_string(next.last_error, sizeof(next.last_error), "oom");
        g_gateway = next;
        return false;
    }
    const size_t device_count = query_devices(g_config.gateway_host, g_config.gateway_port, devices, kMaxLights);

    if (processor_ok) {
        next.processor = processor;
        next.connected = true;
        next.discovery_ok = true;
        copy_string(next.last_error, sizeof(next.last_error), "");
        copy_string(g_config.gateway_name, sizeof(g_config.gateway_name), processor.name);
        copy_string(g_config.gateway_ip, sizeof(g_config.gateway_ip), processor.ip);
    } else if (device_count > 0) {
        next.connected = true;
        next.discovery_ok = true;
        copy_string(next.last_error, sizeof(next.last_error), "");
    } else {
        copy_string(next.last_error, sizeof(next.last_error), "unreachable");
    }

    next.last_device_count = static_cast<uint16_t>(device_count);
    next.last_refresh_us = esp_timer_get_time();
    g_gateway = next;

    if (refresh_loads && next.connected) {
        for (uint8_t i = 0; i < g_config.light_count; ++i) {
            uint8_t level = 0;
            if (!roehn_query_load(g_config.lights[i], &level)) {
                continue;
            }
            g_light_states[i].level_percent = level;
            g_light_states[i].on = level > 0;
            sync_light_state_to_matter(i);
        }
    }
    free(devices);
    return next.connected;
}

void gateway_poll_task(void *)
{
    ESP_LOGI(kTag, "Gateway poll task started");
    int64_t last_full_refresh_us = 0;
    const int64_t full_refresh_interval_us = static_cast<int64_t>(std::max<uint16_t>(5, g_config.scan_interval_sec)) * 1000000LL;
    constexpr int64_t kFastPollIntervalUs = 5000000LL;  // query loads every 5 seconds

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const bool do_full = (now_us - last_full_refresh_us) >= full_refresh_interval_us;

        if (do_full) {
            last_full_refresh_us = now_us;
            refresh_gateway_runtime(false);  // full discovery refresh (no load queries — fast poll handles it)
        }

        // Fast poll: query load levels for all configured lights
        if (g_gateway.connected && g_config.light_count > 0) {
            for (uint8_t i = 0; i < g_config.light_count; ++i) {
                uint8_t level = 0;
                if (roehn_query_load(g_config.lights[i], &level)) {
                    const bool prev_on = g_light_states[i].on;
                    const uint8_t prev_level = g_light_states[i].level_percent;
                    g_light_states[i].level_percent = level;
                    g_light_states[i].on = level > 0;
                    if (prev_on != g_light_states[i].on || prev_level != g_light_states[i].level_percent) {
                        sync_light_state_to_matter(i);
                        ESP_LOGI(kTag, "Fast poll updated %s: %u%%", g_config.lights[i].name, level);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(30));  // small gap between queries
            }
        }

        // Sleep until next fast poll (1 second at a time to stay responsive)
        const int64_t next_poll_us = now_us + kFastPollIntervalUs;
        while (esp_timer_get_time() < next_poll_us) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void command_task(void *)
{
    ESP_LOGI(kTag, "Roehn command task started");

    PendingLoadCommand command = {};
    while (true) {
        if (xQueueReceive(g_command_queue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        PendingLoadCommand newer = {};
        while (xQueueReceive(g_command_queue, &newer, pdMS_TO_TICKS(80)) == pdTRUE) {
            if (newer.light_index == command.light_index) {
                command = newer; // keep newest command for this light
            } else {
                xQueueSendToFront(g_command_queue, &newer, 0);
                break;
            }
        }

        if (command.light_index >= g_config.light_count) {
            continue;
        }

        const LightConfig &light = g_config.lights[command.light_index];

        ESP_LOGI(kTag, "Queued Roehn LOAD: light=%s addr=%u ch=%u level=%u",
                 light.name,
                 static_cast<unsigned>(resolve_control_address(light)),
                 static_cast<unsigned>(light.channel),
                 static_cast<unsigned>(command.level_percent));

        const esp_err_t err = roehn_set_load(light, command.level_percent);
        g_light_states[command.light_index].last_err = err;

        if (err != ESP_OK) {
            ESP_LOGW(kTag, "Queued Roehn LOAD failed for %s: %s",
                     light.name,
                     esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(120)); // XML uses 100 ms between commands
    }
}

struct DiscoveryOutcome {
    bool ok;
    bool restart_required;
    uint8_t light_count;
    char error[64];
};

DiscoveryOutcome perform_discovery(AppConfig *config)
{
    DiscoveryOutcome outcome = {};
    copy_string(outcome.error, sizeof(outcome.error), "unreachable");

    if (!config || !config->gateway_host[0]) {
        copy_string(outcome.error, sizeof(outcome.error), "missing_host");
        return outcome;
    }

    ProcessorInfo processor = {};
    DeviceInfo *devices = static_cast<DeviceInfo *>(calloc(kMaxLights, sizeof(DeviceInfo)));
    if (!devices) {
        copy_string(outcome.error, sizeof(outcome.error), "oom");
        return outcome;
    }
    const bool processor_ok = query_processor_info(config->gateway_host, config->gateway_port, &processor);
    const size_t device_count = query_devices(config->gateway_host, config->gateway_port, devices, kMaxLights);
    ESP_LOGI(kTag, "Discovery probe: processor_ok=%s device_count=%u host=%s port=%u", processor_ok ? "true" : "false",
             static_cast<unsigned>(device_count), config->gateway_host, static_cast<unsigned>(config->gateway_port));
    if (!processor_ok && device_count == 0) {
        free(devices);
        return outcome;
    }

    if (processor_ok) {
        copy_string(config->gateway_name, sizeof(config->gateway_name), processor.name);
        copy_string(config->gateway_ip, sizeof(config->gateway_ip), processor.ip);
    }

    const size_t previous_light_count = g_config.light_count;

    memset(config->lights, 0, sizeof(config->lights));
    const size_t light_count = describe_light_entities(devices, device_count, config->lights, kMaxLights);
    ESP_LOGI(kTag, "Discovery result: %u light entities", static_cast<unsigned>(light_count));
    config->light_count = static_cast<uint8_t>(light_count);
    free(devices);

    outcome.ok = true;
    outcome.light_count = static_cast<uint8_t>(light_count);
    outcome.restart_required = (config->light_count != previous_light_count);
    if (!outcome.restart_required) {
        for (uint8_t i = 0; i < config->light_count; ++i) {
            const LightConfig &lhs = config->lights[i];
            const LightConfig &rhs = g_config.lights[i];
            if (strcmp(lhs.id, rhs.id) != 0 || lhs.supports_brightness != rhs.supports_brightness || lhs.channel != rhs.channel ||
                resolve_control_address(lhs) != resolve_control_address(rhs)) {
                outcome.restart_required = true;
                break;
            }
        }
    }

    g_config = *config;
    if (save_config() != ESP_OK) {
        outcome.ok = false;
        copy_string(outcome.error, sizeof(outcome.error), "save_failed");
        return outcome;
    }

    refresh_gateway_runtime(false);
    copy_string(outcome.error, sizeof(outcome.error), "");
    return outcome;
}

void log_onboarding_codes()
{
    chip::PayloadContents payload;
    CHIP_ERROR err = GetPayloadContents(payload, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(kTag, "Failed to build onboarding payload: %" CHIP_ERROR_FORMAT, err.Format());
        return;
    }

    char code_buffer[chip::QRCodeBasicSetupPayloadGenerator::kMaxQRCodeBase38RepresentationLength + 1] = {};
    chip::MutableCharSpan manual_code(code_buffer);
    err = GetManualPairingCode(manual_code, payload);
    if (err == CHIP_NO_ERROR) {
        ESP_LOGI(kTag, "Matter manual pairing code: %s", code_buffer);
    }

    memset(code_buffer, 0, sizeof(code_buffer));
    chip::MutableCharSpan qr_code(code_buffer);
    err = GetQRCode(qr_code, payload);
    if (err == CHIP_NO_ERROR) {
        ESP_LOGI(kTag, "Matter QR payload: %s", code_buffer);
    }

    ESP_LOGI(kTag, "Matter discriminator: %u", payload.discriminator.GetLongValue());
    ESP_LOGI(kTag, "Matter setup passcode: %u", static_cast<unsigned>(payload.setUpPINCode));
}

const char *wifi_disconnect_reason_name(uint8_t reason)
{
    switch (reason) {
    case 2:
        return "auth_expired";
    case 3:
        return "auth_leave";
    case 4:
        return "assoc_expired";
    case 8:
        return "assoc_leave";
    case 15:
        return "4way_timeout";
    case 201:
        return "no_ap_found";
    case 202:
        return "auth_fail";
    default:
        return "unknown";
    }
}

void set_wifi_hostname()
{
    if (g_wifi_hostname_set) {
        return;
    }
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return;
    }
    if (esp_netif_set_hostname(netif, kWifiHostname) == ESP_OK) {
        g_wifi_hostname_set = true;
    }
}

void prepare_wifi_hostname()
{
    set_wifi_hostname();
}

void log_forwarded_esp_system_event(const ChipDeviceEvent *event)
{
    if (!event) {
        return;
    }

    if (event->Platform.ESPSystemEvent.Base == WIFI_EVENT) {
        switch (event->Platform.ESPSystemEvent.Id) {
        case WIFI_EVENT_STA_START:
            set_wifi_hostname();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(kTag, "Wi-Fi connected");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(kTag, "Wi-Fi disconnected: %s",
                     wifi_disconnect_reason_name(event->Platform.ESPSystemEvent.Data.WiFiStaDisconnected.reason));
            break;
        default:
            break;
        }
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(kTag, "Matter commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(kTag, "Commissioning window opened");
        log_onboarding_codes();
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            if (!mgr.IsCommissioningWindowOpen()) {
                mgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(kCommissioningTimeoutSec),
                                                 chip::CommissioningWindowAdvertisement::kDnssdOnly);
            }
        }
        break;
    case chip::DeviceLayer::DeviceEventType::kESPSystemEvent:
        log_forwarded_esp_system_event(event);
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(esp_matter::identification::callback_type_t, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *)
{
    ESP_LOGI(kTag, "Identify endpoint=%u effect=%u variant=%u", endpoint_id, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(esp_matter::attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *)
{
    if (type != esp_matter::attribute::PRE_UPDATE || !val) {
        return ESP_OK;
    }

    const int index = endpoint_index_from_id(endpoint_id);
    if (index < 0) {
        return ESP_OK;
    }

    LightState next = g_light_states[index];
    bool changed = false;

    if (cluster_id == chip::app::Clusters::OnOff::Id &&
        attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id) {
        next.on = val->val.b;
        if (!g_config.lights[index].supports_brightness) {
            next.level_percent = next.on ? 100 : 0;
        } else if (!next.on) {
            next.level_percent = 0;
        } else if (next.level_percent == 0) {
            next.level_percent = 100;
        }
        changed = true;
    } else if (cluster_id == chip::app::Clusters::LevelControl::Id &&
               attribute_id == chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id &&
               g_config.lights[index].supports_brightness) {
        const uint8_t percent = matter_to_level_percent(val->val.u8);
        // The ZCL OnOff-coupling side-effect always writes CurrentLevel=0 when
        // turning on (then restores it) and CurrentLevel=stored when turning
        // off. Both are spurious — the OnOff callback already queued the
        // correct command. Detect and ignore both cases:
        //   - percent==0 while on==true  : ZCL coupling "on" side-effect
        //   - percent>0  while on==false : ZCL coupling "off" side-effect
        // Only act on CurrentLevel when it agrees with the current on-state,
        // which is the case for genuine MoveToLevel commands.
        if (percent == 0 && next.on) {
            return ESP_OK;
        }
        if (percent > 0 && !next.on) {
            return ESP_OK;
        }
        next.level_percent = percent;
        next.on = percent > 0;
        changed = true;
    }

    if (!changed) {
        return ESP_OK;
    }

    g_light_states[index] = next;
    const uint8_t outbound_level = g_config.lights[index].supports_brightness ? next.level_percent : (next.on ? 100 : 0);
    if (g_command_queue) {
        PendingLoadCommand command = {
            .light_index = static_cast<uint8_t>(index),
            .level_percent = outbound_level,
            .sequence = ++g_command_sequence,
        };

        ESP_LOGI(kTag, "Queue Roehn LOAD: endpoint=%u light=%s level=%u seq=%u",
                endpoint_id,
                g_config.lights[index].name,
                static_cast<unsigned>(outbound_level),
                static_cast<unsigned>(command.sequence));

        if (xQueueSend(g_command_queue, &command, 0) != pdTRUE) {
            ESP_LOGW(kTag, "Roehn command queue full for %s", g_config.lights[index].name);
        }
    }
    sync_light_state_to_matter(static_cast<uint8_t>(index));
    return ESP_OK;
}

// Creates a minimal dimmable or on/off light endpoint without the optional
// Matter "Lighting" feature attributes (StartUpOnOff, StartUpCurrentLevel,
// GlobalSceneControl, OnTime, OffWaitTime) that HA surfaces as unwanted
// "Power On Behavior" and "On Level" entities.
static esp_matter::endpoint_t *create_minimal_light_endpoint(
    esp_matter::node_t *node, bool dimmable, bool on, uint8_t level)
{
    using namespace esp_matter;
    using namespace esp_matter::cluster;

    endpoint_t *ep = endpoint::create(node, ENDPOINT_FLAG_BRIDGE, nullptr);
    if (!ep) return nullptr;

    // Descriptor cluster is created by common::create in the presets — must add
    // it manually here since we use the raw endpoint::create base.
    descriptor::config_t desc_cfg = {};
    if (!descriptor::create(ep, &desc_cfg, CLUSTER_FLAG_SERVER)) {
        return nullptr;
    }

    uint32_t device_type_id = dimmable
        ? ESP_MATTER_DIMMABLE_LIGHT_DEVICE_TYPE_ID
        : ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_ID;
    uint8_t device_type_version = dimmable
        ? ESP_MATTER_DIMMABLE_LIGHT_DEVICE_TYPE_VERSION
        : ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_VERSION;
    endpoint::add_device_type(ep, device_type_id, device_type_version);

    // Identify cluster (mandatory)
    identify::config_t id_cfg = {};
    cluster_t *identify_cluster = identify::create(ep, &id_cfg, CLUSTER_FLAG_SERVER);
    identify::command::create_trigger_effect(identify_cluster);

    // Groups cluster (mandatory for lights)
    groups::config_t grp_cfg = {};
    groups::create(ep, &grp_cfg, CLUSTER_FLAG_SERVER);

    // OnOff cluster — no lighting feature, so no StartUpOnOff/GlobalSceneControl/OnTime/OffWaitTime
    on_off::config_t onoff_cfg = {};
    onoff_cfg.on_off = on;
    cluster_t *onoff_cluster = on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
    on_off::command::create_on(onoff_cluster);
    on_off::command::create_off(onoff_cluster);
    on_off::command::create_toggle(onoff_cluster);

    if (dimmable) {
        // LevelControl cluster — on_off feature only, no lighting feature (no StartUpCurrentLevel)
        level_control::config_t lvl_cfg = {};
        lvl_cfg.current_level = nullable<uint8_t>(level);
        cluster_t *lvl_cluster = level_control::create(ep, &lvl_cfg, CLUSTER_FLAG_SERVER);
        level_control::feature::on_off::add(lvl_cluster);
        level_control::command::create_move_to_level(lvl_cluster);
        level_control::command::create_move(lvl_cluster);
        level_control::command::create_step(lvl_cluster);
        level_control::command::create_stop(lvl_cluster);
        level_control::command::create_move_to_level_with_on_off(lvl_cluster);
        level_control::command::create_move_with_on_off(lvl_cluster);
        level_control::command::create_step_with_on_off(lvl_cluster);
        level_control::command::create_stop_with_on_off(lvl_cluster);
    }

    // Scenes Management cluster (mandatory per spec)
    scenes_management::config_t scn_cfg = {};
    cluster_t *scenes_cluster = scenes_management::create(ep, &scn_cfg, CLUSTER_FLAG_SERVER);
    scenes_management::command::create_copy_scene(scenes_cluster);
    scenes_management::command::create_copy_scene_response(scenes_cluster);

    return ep;
}

void create_light_endpoints(esp_matter::node_t *node)
{
    // ── Group lights by module (serial_hex) ──────────────────────────────
    struct ModuleGroup {
        const char *serial;
        const char *model_name;
        uint8_t start_index;
        uint8_t count;
    };

    ModuleGroup modules[32] = {};
    uint8_t module_count = 0;

    for (uint8_t i = 0; i < g_config.light_count; ++i) {
        const char *serial = g_config.lights[i].serial_hex;
        uint8_t m = 0;
        for (; m < module_count; ++m) {
            if (strcmp(modules[m].serial, serial) == 0) {
                modules[m].count++;
                break;
            }
        }
        if (m == module_count && module_count < 32) {
            modules[module_count].serial = serial;
            modules[module_count].model_name = g_config.lights[i].model;
            modules[module_count].start_index = i;
            modules[module_count].count = 1;
            module_count++;
        }
    }

    g_config.module_count = module_count;

    // ── Get the aggregator endpoint (EP1) ────────────────────────────────
    esp_matter::endpoint_t *agg_ep = esp_matter::endpoint::get(node, 1);
    if (!agg_ep) {
        ESP_LOGE(kTag, "Aggregator endpoint (EP1) not found");
        return;
    }

    // ── Create one bridged_node per module, then child light endpoints ───
    // Use create() (auto-assign IDs) so ESP-Matter handles persistence.
    for (uint8_t m = 0; m < module_count; ++m) {

        // ── Module parent: create a bridged_node ─────────────────────────
        esp_matter::endpoint::bridged_node::config_t bn_cfg = {};
        esp_matter::endpoint_t *module_ep =
            esp_matter::endpoint::bridged_node::create(node, &bn_cfg, esp_matter::ENDPOINT_FLAG_BRIDGE, nullptr);
        if (!module_ep) {
            ESP_LOGE(kTag, "Failed to create bridged_node for module %u", m);
            continue;
        }

        const uint16_t parent_ep_id = esp_matter::endpoint::get_id(module_ep);
        g_config.module_parent_ids[m] = parent_ep_id;

        char module_label[33] = {};
        snprintf(module_label, sizeof(module_label), "%u-%s",
                 static_cast<unsigned>(resolve_control_address(g_config.lights[modules[m].start_index])),
                 modules[m].model_name[0] ? modules[m].model_name : "Module");

        // Set NodeLabel / ProductName / Reachable / Serial on the bridged_node
        esp_matter::cluster_t *bn_cluster =
            esp_matter::cluster::get(module_ep, chip::app::Clusters::BridgedDeviceBasicInformation::Id);
        if (bn_cluster) {
            esp_matter::cluster::bridged_device_basic_information::attribute::create_node_label(
                bn_cluster, module_label, strlen(module_label));
            esp_matter_attr_val_t nl_val = esp_matter_char_str(module_label, strlen(module_label));
            esp_matter::attribute::update(parent_ep_id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                                          &nl_val);

            esp_matter::cluster::bridged_device_basic_information::attribute::create_product_name(
                bn_cluster, module_label, strlen(module_label));
            esp_matter_attr_val_t pn_val = esp_matter_char_str(module_label, strlen(module_label));
            esp_matter::attribute::update(parent_ep_id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::ProductName::Id,
                                          &pn_val);

            esp_matter_attr_val_t reach = esp_matter_bool(true);
            esp_matter::attribute::update(parent_ep_id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id,
                                          &reach);

            esp_matter::cluster::bridged_device_basic_information::attribute::create_serial_number(
                bn_cluster, const_cast<char *>(modules[m].serial), strlen(modules[m].serial));
            esp_matter_attr_val_t sn_val = esp_matter_char_str(const_cast<char *>(modules[m].serial), strlen(modules[m].serial));
            esp_matter::attribute::update(parent_ep_id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::SerialNumber::Id,
                                          &sn_val);
        }

        // Fixed Label on module parent
        {
            esp_matter::cluster::fixed_label::config_t fl_cfg = {};
            esp_matter::cluster::fixed_label::create(module_ep, &fl_cfg, esp_matter::CLUSTER_FLAG_SERVER);
        }

        // Module parent is a child of the aggregator
        esp_matter::endpoint::set_parent_endpoint(module_ep, agg_ep);

        ESP_LOGI(kTag, "Module EP%u '%s' (%u channels)",
                 parent_ep_id, module_label, modules[m].count);

        // ── Child light endpoints ────────────────────────────────────────
        for (uint8_t c = 0; c < modules[m].count; ++c) {
            const uint8_t light_index = modules[m].start_index + c;

            esp_matter::endpoint_t *child_ep =
                esp_matter::endpoint::create(node, esp_matter::ENDPOINT_FLAG_BRIDGE, nullptr);
            if (!child_ep) {
                ESP_LOGE(kTag, "Failed to create child endpoint for light[%u]", light_index);
                continue;
            }

            const uint16_t child_ep_id = esp_matter::endpoint::get_id(child_ep);
            g_endpoint_ids[light_index] = child_ep_id;

            // Add device type
            uint32_t dt_id = g_config.lights[light_index].supports_brightness
                                 ? ESP_MATTER_DIMMABLE_LIGHT_DEVICE_TYPE_ID
                                 : ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_ID;
            uint8_t dt_ver = g_config.lights[light_index].supports_brightness
                                 ? ESP_MATTER_DIMMABLE_LIGHT_DEVICE_TYPE_VERSION
                                 : ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_VERSION;
            esp_matter::endpoint::add_device_type(child_ep, dt_id, dt_ver);

            using namespace esp_matter;
            using namespace esp_matter::cluster;

            // Descriptor
            descriptor::config_t desc_cfg = {};
            descriptor::create(child_ep, &desc_cfg, CLUSTER_FLAG_SERVER);

            // Identify
            identify::config_t id_cfg = {};
            cluster_t *id_cluster = identify::create(child_ep, &id_cfg, CLUSTER_FLAG_SERVER);
            identify::command::create_trigger_effect(id_cluster);

            // Groups
            groups::config_t grp_cfg = {};
            groups::create(child_ep, &grp_cfg, CLUSTER_FLAG_SERVER);

            // OnOff
            on_off::config_t onoff_cfg = {};
            onoff_cfg.on_off = g_light_states[light_index].on;
            cluster_t *onoff_cluster = on_off::create(child_ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
            on_off::command::create_on(onoff_cluster);
            on_off::command::create_off(onoff_cluster);
            on_off::command::create_toggle(onoff_cluster);

            if (g_config.lights[light_index].supports_brightness) {
                level_control::config_t lvl_cfg = {};
                lvl_cfg.current_level = nullable<uint8_t>(
                    level_percent_to_matter(g_light_states[light_index].level_percent));
                cluster_t *lvl_cluster = level_control::create(child_ep, &lvl_cfg, CLUSTER_FLAG_SERVER);
                level_control::feature::on_off::add(lvl_cluster);
                level_control::command::create_move_to_level(lvl_cluster);
                level_control::command::create_move(lvl_cluster);
                level_control::command::create_step(lvl_cluster);
                level_control::command::create_stop(lvl_cluster);
                level_control::command::create_move_to_level_with_on_off(lvl_cluster);
                level_control::command::create_move_with_on_off(lvl_cluster);
                level_control::command::create_step_with_on_off(lvl_cluster);
                level_control::command::create_stop_with_on_off(lvl_cluster);
            }

            // Scenes
            scenes_management::config_t scn_cfg = {};
            cluster_t *scenes_cluster = scenes_management::create(child_ep, &scn_cfg, CLUSTER_FLAG_SERVER);
            scenes_management::command::create_copy_scene(scenes_cluster);
            scenes_management::command::create_copy_scene_response(scenes_cluster);

            // Fixed Label with channel name
            {
                fixed_label::config_t fl_cfg = {};
                fixed_label::create(child_ep, &fl_cfg, CLUSTER_FLAG_SERVER);
            }

            // Attach child to module parent
            esp_matter::endpoint::set_parent_endpoint(child_ep, module_ep);

            ESP_LOGI(kTag, "  Light EP%u '%s' addr=%u ch=%u %s",
                     child_ep_id, g_config.lights[light_index].name,
                     static_cast<unsigned>(resolve_control_address(g_config.lights[light_index])),
                     static_cast<unsigned>(g_config.lights[light_index].channel),
                     g_config.lights[light_index].supports_brightness ? "dimmer" : "relay");
        }
    }
}

esp_err_t send_json_response(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json ? json : "{}");
}

char *read_request_body(httpd_req_t *req)
{
    if (!req || req->content_len <= 0 || req->content_len > static_cast<int>(kMaxBodyBytes)) {
        return nullptr;
    }

    char *body = static_cast<char *>(calloc(1, req->content_len + 1));
    if (!body) {
        return nullptr;
    }

    int offset = 0;
    while (offset < req->content_len) {
        const int read = httpd_req_recv(req, body + offset, req->content_len - offset);
        if (read <= 0) {
            free(body);
            return nullptr;
        }
        offset += read;
    }
    body[offset] = '\0';
    return body;
}

esp_err_t setup_page_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, kSetupPageHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t config_get_handler(httpd_req_t *req)
{
    char *json = config_to_json();
    esp_err_t err = send_json_response(req, json);
    if (json) {
        cJSON_free(json);
    }
    return err;
}

esp_err_t status_get_handler(httpd_req_t *req)
{
    refresh_gateway_runtime(false);

    cJSON *root = cJSON_CreateObject();
    cJSON *gateway = cJSON_AddObjectToObject(root, "gateway");
    cJSON_AddBoolToObject(gateway, "connected", g_gateway.connected);
    cJSON_AddBoolToObject(gateway, "discovery_ok", g_gateway.discovery_ok);
    cJSON_AddStringToObject(gateway, "name", g_gateway.processor.name[0] ? g_gateway.processor.name : g_config.gateway_name);
    cJSON_AddStringToObject(gateway, "ip", g_gateway.processor.ip[0] ? g_gateway.processor.ip : g_config.gateway_ip);
    cJSON_AddStringToObject(gateway, "version", g_gateway.processor.version);
    cJSON_AddStringToObject(gateway, "serial", g_gateway.processor.serial);
    cJSON_AddStringToObject(gateway, "last_error", g_gateway.last_error);
    cJSON_AddNumberToObject(gateway, "device_count", g_gateway.last_device_count);
    const int64_t now_us = esp_timer_get_time();
    cJSON_AddNumberToObject(gateway, "last_refresh_ms",
                            g_gateway.last_refresh_us > 0 && now_us >= g_gateway.last_refresh_us
                                ? (now_us - g_gateway.last_refresh_us) / 1000
                                : 0);

    cJSON *matter = cJSON_AddObjectToObject(root, "matter");
    cJSON_AddNumberToObject(matter, "fabric_count",
                            g_matter_started ? chip::Server::GetInstance().GetFabricTable().FabricCount() : 0);

    cJSON *lights = cJSON_AddArrayToObject(root, "lights");
    for (uint8_t i = 0; i < g_config.light_count; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "id", g_config.lights[i].id);
        cJSON_AddStringToObject(item, "name", g_config.lights[i].name);
        cJSON_AddStringToObject(item, "model", g_config.lights[i].model);
        cJSON_AddNumberToObject(item, "channel", g_config.lights[i].channel);
        cJSON_AddBoolToObject(item, "supports_brightness", g_config.lights[i].supports_brightness);
        cJSON_AddNumberToObject(item, "endpoint_id", g_endpoint_ids[i]);
        cJSON_AddNumberToObject(item, "control_address", resolve_control_address(g_config.lights[i]));
        cJSON_AddBoolToObject(item, "on", g_light_states[i].on);
        cJSON_AddNumberToObject(item, "level_percent", g_light_states[i].level_percent);
        cJSON_AddStringToObject(item, "last_error", esp_err_to_name(g_light_states[i].last_err));
        cJSON_AddItemToArray(lights, item);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_err_t err = send_json_response(req, json);
    if (json) {
        cJSON_free(json);
    }
    return err;
}

bool apply_gateway_fields_from_json(cJSON *root, AppConfig *config)
{
    if (!root || !config) {
        return false;
    }
    cJSON *gateway = cJSON_GetObjectItemCaseSensitive(root, "gateway");
    if (!cJSON_IsObject(gateway)) {
        return false;
    }
    copy_string(config->gateway_host, sizeof(config->gateway_host), read_json_string(gateway, "host"));
    config->gateway_port = static_cast<uint16_t>(std::clamp(read_json_int(gateway, "port", config->gateway_port), 1, 65535));
    config->scan_interval_sec =
        static_cast<uint16_t>(std::clamp(read_json_int(gateway, "scan_interval", config->scan_interval_sec), 5, 3600));
    return true;
}

esp_err_t config_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (!body) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_body\"}");
    }

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_json\"}");
    }

    AppConfig *next = static_cast<AppConfig *>(calloc(1, sizeof(AppConfig)));
    if (!next) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
    }
    *next = g_config;
    if (!apply_gateway_fields_from_json(root, next)) {
        free(next);
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"missing_gateway\"}");
    }
    cJSON_Delete(root);

    g_config = *next;
    free(next);
    if (save_config() != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"save_failed\"}");
    }
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

esp_err_t discover_post_handler(httpd_req_t *req)
{
    char *body = read_request_body(req);
    if (!body) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_body\"}");
    }

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_json\"}");
    }

    AppConfig *pending = static_cast<AppConfig *>(calloc(1, sizeof(AppConfig)));
    if (!pending) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
    }
    *pending = g_config;
    if (!apply_gateway_fields_from_json(root, pending)) {
        free(pending);
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"missing_gateway\"}");
    }
    cJSON_Delete(root);

    DiscoveryOutcome outcome = perform_discovery(pending);
    free(pending);
    if (!outcome.ok) {
        httpd_resp_set_status(req, "502 Bad Gateway");
        char response[128];
        snprintf(response, sizeof(response), "{\"error\":\"%s\"}", outcome.error[0] ? outcome.error : "discover_failed");
        return httpd_resp_sendstr(req, response);
    }

    const uint8_t light_count = outcome.light_count;
    char response[192];
    snprintf(response, sizeof(response), "{\"ok\":true,\"light_count\":%u,\"restart_required\":true}",
             static_cast<unsigned>(light_count));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);

    // Give the TCP stack ~500ms to flush the response, then restart.
    // httpd_resp_sendstr() queues the response but it may not have been
    // fully transmitted yet. A short delay ensures the browser gets it.
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

void start_http_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 12288;
    if (httpd_start(&g_httpd, &config) != ESP_OK) {
        ESP_LOGE(kTag, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t setup_page = {.uri = "/", .method = HTTP_GET, .handler = setup_page_get_handler, .user_ctx = nullptr};
    httpd_register_uri_handler(g_httpd, &setup_page);

    httpd_uri_t config_get = {.uri = "/api/config", .method = HTTP_GET, .handler = config_get_handler, .user_ctx = nullptr};
    httpd_register_uri_handler(g_httpd, &config_get);

    httpd_uri_t config_post = {.uri = "/api/config", .method = HTTP_POST, .handler = config_post_handler, .user_ctx = nullptr};
    httpd_register_uri_handler(g_httpd, &config_post);

    httpd_uri_t status_get = {.uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = nullptr};
    httpd_register_uri_handler(g_httpd, &status_get);

    httpd_uri_t discover_post = {
        .uri = "/api/discover",
        .method = HTTP_POST,
        .handler = discover_post_handler,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(g_httpd, &discover_post);
}

}  // namespace

extern "C" void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    load_default_states();
    load_config();
    load_resources_index();

    esp_matter::node::config_t node_config = {};
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ESP_ERROR_CHECK(node ? ESP_OK : ESP_FAIL);

    // Create the aggregator endpoint (device type 0x000E) on endpoint 1.
    // Home Assistant uses this to detect is_bridge=true and discover all
    // child bridged endpoints via the Descriptor Parts List.
    {
        esp_matter::endpoint::aggregator::config_t agg_config = {};
        esp_matter::endpoint_t *agg_ep = esp_matter::endpoint::aggregator::create(
            node, &agg_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
        if (!agg_ep) {
            ESP_LOGE(kTag, "Failed to create aggregator endpoint");
        } else {
            ESP_LOGI(kTag, "Aggregator endpoint created id=%u", esp_matter::endpoint::get_id(agg_ep));
        }
    }

    // Create endpoints BEFORE starting Matter so they are visible during
    // the initial HA interview. ESP_MATTER_MEM_ALLOC_MODE_EXTERNAL ensures
    // these go to PSRAM, leaving internal RAM free for WiFi DMA init.
    create_light_endpoints(node);
    ESP_LOGI(kTag, "DEBUG A: endpoints created");

    ESP_LOGI(kTag, "Starting Matter stack");
    ESP_ERROR_CHECK(esp_matter::start(app_event_cb));
    g_matter_started = true;
    ESP_LOGI(kTag, "DEBUG C: Matter stack started");

    g_command_queue = xQueueCreate(8, sizeof(PendingLoadCommand));
    g_event_lock = xSemaphoreCreateMutex();
    if (!g_command_queue) {
        ESP_LOGE(kTag, "Failed to create Roehn command queue; Matter state will update but gateway commands will be dropped");
    } else if (!g_command_task) {
        const BaseType_t task_created = xTaskCreate(command_task, "roehn_cmd", 4096, nullptr, 5, &g_command_task);
        if (task_created != pdPASS) {
            ESP_LOGE(kTag, "Failed to create Roehn command task");
            g_command_task = nullptr;
        }
    }

    // Start persistent event listener for real-time R:LOAD feedback from the gateway.
    if (!g_event_task_handle) {
        const BaseType_t task_created = xTaskCreate(event_listener_task, "roehn_evt", 6144, nullptr, 4, &g_event_task_handle);
        if (task_created != pdPASS) {
            ESP_LOGE(kTag, "Failed to create Roehn event listener task");
            g_event_task_handle = nullptr;
        }
    }

    if (!g_gateway_task) {
        const BaseType_t task_created = xTaskCreate(gateway_poll_task, "gw_poll", 6144, nullptr, 4, &g_gateway_task);
        if (task_created != pdPASS) {
            ESP_LOGE(kTag, "Failed to create gateway poll task; light states will not update automatically");
            g_gateway_task = nullptr;
        }
    }

    start_http_server();
    ESP_LOGI(kTag, "DEBUG D: HTTP setup UI ready");

    ESP_LOGI(kTag, "Configured gateway host=%s port=%u lights=%u",
            g_config.gateway_host,
            static_cast<unsigned>(g_config.gateway_port),
            static_cast<unsigned>(g_config.light_count));

    apply_user_labels();
    log_onboarding_codes();

    ESP_LOGI(kTag, "Gateway poll task started; light states sync to Matter every scan_interval_sec");
}
