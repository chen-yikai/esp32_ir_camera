/*
 * ESP32-S3 + MLX90640 thermal camera -> live web page (high refresh version)
 *
 * Libraries (Arduino Library Manager):
 *   - Adafruit MLX90640   (+ Adafruit BusIO)
 *   - WebSockets          by Markus Sattler (Links2004)
 *
 * Wiring:
 *   MLX90640 VIN -> 3V3, GND -> GND, SDA -> GPIO 8, SCL -> GPIO 9
 *   Keep I2C wires short (< 10-15 cm) for 1 MHz.
 *
 *   Vibration motors (left / right half of the view):
 *     GPIO 4 -> left motor driver IN, GPIO 5 -> right motor driver IN
 *     Do NOT drive a bare motor from a GPIO (it pulls ~60-100 mA). Use a vibration
 *     motor module (has its own transistor), or an N-MOSFET / NPN (1k base resistor)
 *     low-side switch with a 1N4148/1N5819 flyback diode across the motor.
 *
 * Haptics: each half of the image vibrates its motor when any pixel there is
 * > the trigger temp (default 50 °C, adjustable live on the dashboard). The IR camera can't measure range, so "closeness" is estimated from
 * how big the hot spot looks (apparent area ~ 1/distance^2) and how hot its
 * peak reads (far objects get averaged with background) - stronger = closer.
 *
 * Usage: open Serial Monitor (115200) for the IP, then http://<ip>/ or http://thermal.local/
 * Fallback AP: "ESP32-Thermal" -> http://192.168.4.1/
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <Adafruit_MLX90640.h>
#include <Preferences.h>

// ---------- User config ----------
const char* WIFI_SSID = "pixel_kitten";
const char* WIFI_PASS = "hellokitty";

#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 1000000               // 1 MHz needed for 16 Hz
#define SENSOR_RATE MLX90640_16_HZ     // try MLX90640_8_HZ if you get read errors

#define MOTOR_L_PIN 4
#define MOTOR_R_PIN 5
#define MIRROR_VIEW true               // match the page's default "Mirror": left motor = left of the image
#define HOT_C_DEFAULT 50.0f            // start vibrating above this (adjustable live from the dashboard)
#define HOT_C_MIN 25.0f                // allowed range for the dashboard setting
#define HOT_C_MAX 150.0f
#define HOT_HYSTERESIS_C 2.0f          // stop 2 °C below the trigger (avoids buzzing on/off at the edge)
#define NEAR_PIXELS 96                 // hot pixels in one half that count as "right in front" (25% of half)
#define NEAR_TEMP_SPAN_C 70.0f         // peak this far above the trigger counts as "right in front"
#define MOTOR_MIN_DUTY 90              // 0-255, lowest duty that reliably spins your motor
#define MOTOR_PWM_FREQ 20000           // Hz, above hearing range
#define MOTOR_TIMEOUT_MS 500           // stop motors if the sensor stops delivering frames
// ---------------------------------

#define W 32
#define H 24
#define N (W * H)

Adafruit_MLX90640 mlx;
WebServer server(80);
WebSocketsServer ws(81);

static int16_t sharedFrame[N];          // temperature * 100 (°C)
static volatile uint32_t frameSeq = 0;
static SemaphoreHandle_t frameMutex;
static volatile uint8_t motorLevel[2] = {0, 0};   // current duty, 0 = left, 1 = right
static volatile int16_t sideHot[2] = {0, 0};      // hot pixels per half (for the dashboard)
static volatile int16_t sidePeak[2] = {0, 0};     // peak temp per half * 100
static volatile float hotC = HOT_C_DEFAULT;       // live trigger temperature
static volatile bool motorEnabled[2] = {true, true};   // switchable from the dashboard
static Preferences prefs;

// ---------------- Web page ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Thermal Haptics</title>
<style>
  /* shadcn/ui-style tokens (zinc, dark) */
  :root {
    color-scheme: dark;
    --background: 240 10% 3.9%;
    --foreground: 0 0% 98%;
    --card: 240 10% 3.9%;
    --muted: 240 3.7% 15.9%;
    --muted-foreground: 240 5% 64.9%;
    --border: 240 3.7% 15.9%;
    --input: 240 3.7% 15.9%;
    --primary: 0 0% 98%;
    --primary-foreground: 240 5.9% 10%;
    --destructive: 0 72% 51%;
    --success: 142 71% 45%;
    --ring: 240 4.9% 83.9%;
    --radius: 0.5rem;
  }
  * { box-sizing:border-box; border-color:hsl(var(--border)); }
  body { margin:0; background:hsl(var(--background)); color:hsl(var(--foreground)); padding:24px 16px;
         font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
         font-size:14px; line-height:1.5; -webkit-font-smoothing:antialiased; }
  .container { max-width:1080px; margin:0 auto; display:flex; flex-direction:column; gap:16px; }

  header { display:flex; align-items:flex-end; justify-content:space-between; gap:12px; flex-wrap:wrap;
           padding-bottom:16px; border-bottom:1px solid hsl(var(--border)); }
  h1 { font-size:24px; line-height:1.2; margin:0; font-weight:600; letter-spacing:-.025em; }
  .desc { color:hsl(var(--muted-foreground)); font-size:14px; margin:4px 0 0; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }

  /* Badge */
  .badge { display:inline-flex; align-items:center; gap:6px; border-radius:calc(var(--radius) - 2px);
           border:1px solid hsl(var(--border)); padding:2px 10px; font-size:12px; font-weight:600;
           font-variant-numeric:tabular-nums; white-space:nowrap; transition:background .15s, color .15s; }
  .badge.secondary { background:hsl(var(--muted)); border-color:transparent; }
  .badge.destructive { background:hsl(var(--destructive)); color:#fff; border-color:transparent; }
  .badge.outline { background:transparent; }

  /* Status dot with ping */
  .dot { position:relative; width:8px; height:8px; border-radius:50%; background:hsl(var(--muted-foreground)); }
  .live .dot, .dot.red { background:hsl(var(--success)); }
  .dot.red { background:#fff; }
  .live .dot::after, .dot.red::after { content:""; position:absolute; inset:0; border-radius:50%; background:inherit;
                                       animation:ping 1s cubic-bezier(0,0,.2,1) infinite; }
  @keyframes ping { 75%,100% { transform:scale(2.2); opacity:0; } }
  @media (prefers-reduced-motion: reduce) { .dot::after { animation:none !important; } }

  /* Card */
  .card { background:hsl(var(--card)); border:1px solid hsl(var(--border)); border-radius:calc(var(--radius) + 4px);
          box-shadow:0 1px 2px rgba(0,0,0,.3); min-width:0; transition:border-color .15s; }
  .card-header { padding:20px 20px 0; display:flex; justify-content:space-between; align-items:flex-start; gap:8px; }
  .card-title { font-size:16px; font-weight:600; letter-spacing:-.01em; margin:0; line-height:1.3; }
  .card-desc { color:hsl(var(--muted-foreground)); font-size:13px; margin:2px 0 0; }
  .card-content { padding:16px 20px 20px; }

  .grid { display:grid; grid-template-columns:minmax(0,1fr) 320px; gap:16px; }
  @media (max-width: 860px) { .grid { grid-template-columns:minmax(0,1fr); } }

  /* Thermal view */
  #wrap { position:relative; width:100%; aspect-ratio:4/3; border-radius:var(--radius); overflow:hidden;
          background:#000; border:1px solid hsl(var(--border)); }
  canvas#c { display:block; width:100%; height:100%; cursor:crosshair; }
  #cross { position:absolute; left:50%; top:50%; width:14px; height:14px; margin:-7px 0 0 -7px;
           border:2px solid #fff; border-radius:50%; pointer-events:none; mix-blend-mode:difference; }
  #split { position:absolute; left:50%; top:0; bottom:0; border-left:1px dashed rgba(255,255,255,.35); pointer-events:none; }
  .zone { position:absolute; top:0; bottom:0; width:50%; pointer-events:none; transition:box-shadow .15s; }
  .zone.left { left:0; } .zone.right { right:0; }
  .zone.on { box-shadow: inset 0 0 0 2px hsl(var(--destructive)), inset 0 0 48px hsl(var(--destructive) / .35); }
  .zone .badge { position:absolute; top:10px; background:rgba(9,9,11,.75); border-color:rgba(255,255,255,.12);
                 backdrop-filter:blur(4px); }
  .zone.left .badge { left:10px; } .zone.right .badge { right:10px; }
  .zone.on .badge { background:hsl(var(--destructive)); color:#fff; border-color:transparent; }
  #bar { display:block; width:100%; height:8px; border-radius:999px; margin-top:16px; }
  #scale { display:flex; justify-content:space-between; font-size:12px; color:hsl(var(--muted-foreground));
           margin-top:6px; font-variant-numeric:tabular-nums; }

  /* Motor cards */
  .motors { display:flex; flex-direction:column; gap:16px; }
  @media (max-width: 860px) { .motors { flex-direction:row; } .motors .card { flex:1; } }
  @media (max-width: 480px) { .motors { flex-direction:column; } }
  .motor.on { border-color:hsl(var(--destructive) / .6); }
  .motor.off .card-content { opacity:.45; }
  .motor .head-right { display:flex; flex-direction:column; align-items:flex-end; gap:8px; }
  .motor .head-right label { font-size:12px; color:hsl(var(--muted-foreground)); gap:8px; }
  .big { font-size:36px; font-weight:700; letter-spacing:-.03em; line-height:1; font-variant-numeric:tabular-nums; }
  .big small { font-size:16px; color:hsl(var(--muted-foreground)); font-weight:500; margin-left:2px; }
  .progress { height:8px; width:100%; border-radius:999px; background:hsl(var(--muted)); overflow:hidden; margin:12px 0 16px; }
  .progress > div { height:100%; width:0; background:hsl(var(--primary)); border-radius:999px; transition:width .15s, background .15s; }
  .motor.on .progress > div { background:hsl(var(--destructive)); }
  .kv { display:grid; grid-template-columns:1fr 1fr; gap:12px; padding-top:16px; border-top:1px solid hsl(var(--border)); }
  .kv span { display:block; font-size:12px; color:hsl(var(--muted-foreground)); }
  .kv b { font-size:14px; font-weight:600; font-variant-numeric:tabular-nums; white-space:nowrap; }

  /* Stat cards */
  .stats { display:grid; grid-template-columns:repeat(5, minmax(0,1fr)); gap:16px; }
  @media (max-width: 760px) { .stats { grid-template-columns:repeat(2, minmax(0,1fr)); } }
  .stat { padding:16px 20px; }
  .stat span { display:block; font-size:13px; color:hsl(var(--muted-foreground)); font-weight:500; }
  .stat b { display:block; font-size:22px; font-weight:700; letter-spacing:-.02em; margin-top:4px;
            font-variant-numeric:tabular-nums; white-space:nowrap; }

  /* Trigger control */
  .trigger { display:flex; gap:16px 24px; align-items:center; flex-wrap:wrap; }
  .trigger .big { min-width:140px; }
  .trigger input[type=range] { flex:1; min-width:180px; width:auto; }
  .hint { font-size:12px; color:hsl(var(--muted-foreground)); }
  .sync { font-size:12px; color:hsl(var(--muted-foreground)); display:inline-flex; gap:6px; align-items:center; }
  .sync.saved { color:hsl(var(--success)); }

  /* Switch / input / slider */
  .controls { display:flex; gap:20px 28px; flex-wrap:wrap; align-items:center; }
  label { display:inline-flex; gap:10px; align-items:center; font-size:14px; font-weight:500; cursor:pointer; }
  input[type=checkbox] { appearance:none; -webkit-appearance:none; margin:0; width:36px; height:20px; border-radius:999px;
    background:hsl(var(--input)); position:relative; cursor:pointer; transition:background .15s; flex:none; }
  input[type=checkbox]::before { content:""; position:absolute; top:2px; left:2px; width:16px; height:16px; border-radius:50%;
    background:hsl(var(--background)); box-shadow:0 1px 3px rgba(0,0,0,.4); transition:transform .15s; }
  input[type=checkbox]:checked { background:hsl(var(--primary)); }
  input[type=checkbox]:checked::before { transform:translateX(16px); }
  input[type=number] { width:72px; height:36px; padding:0 10px; border-radius:var(--radius); background:transparent;
    color:inherit; border:1px solid hsl(var(--input)); font:inherit; }
  input:focus-visible { outline:2px solid hsl(var(--ring)); outline-offset:2px; }
  input[type=range] { -webkit-appearance:none; appearance:none; width:140px; height:6px; border-radius:999px;
    background:hsl(var(--muted)); cursor:pointer; }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:16px; height:16px; border-radius:50%;
    background:hsl(var(--background)); border:2px solid hsl(var(--primary)); }
  input[type=range]::-moz-range-thumb { width:12px; height:12px; border-radius:50%;
    background:hsl(var(--background)); border:2px solid hsl(var(--primary)); }
</style>
</head>
<body>
<div class="container">
  <header>
    <div>
      <h1>Thermal Haptics</h1>
      <p class="desc">MLX90640 · ESP32-S3 — left/right vibration when something is hotter than <span id="thr">--</span></p>
    </div>
    <div class="row">
      <span class="badge outline" id="statusPill"><span class="dot"></span><span id="status">Connecting…</span></span>
      <span class="badge secondary"><span id="fps">--</span>&nbsp;FPS</span>
    </div>
  </header>

  <div class="grid">
    <div class="card">
      <div class="card-header">
        <div><h3 class="card-title">Thermal view</h3><p class="card-desc">Split down the middle — each half drives one motor</p></div>
      </div>
      <div class="card-content">
        <div id="wrap">
          <canvas id="c" width="640" height="480"></canvas>
          <div class="zone left" id="zoneLeft"><span class="badge" id="tagLeft">Left</span></div>
          <div class="zone right" id="zoneRight"><span class="badge" id="tagRight">Right</span></div>
          <div id="split"></div><div id="cross"></div>
        </div>
        <canvas id="bar" width="256" height="1"></canvas>
        <div id="scale"><span id="lo">--</span><span id="hi">--</span></div>
      </div>
    </div>

    <div class="motors">
      <div class="card motor" id="motor0"></div>
      <div class="card motor" id="motor1"></div>
    </div>
  </div>

  <div class="card">
    <div class="card-header">
      <div><h3 class="card-title">Vibration trigger</h3><p class="card-desc">A motor vibrates when its half sees anything hotter than this. Applied live and saved on the device.</p></div>
      <span class="sync" id="sync">--</span>
    </div>
    <div class="card-content trigger">
      <div class="big" id="thrBig">--</div>
      <input type="range" id="thrRange" min="25" max="150" step="0.5" value="50" aria-label="Trigger temperature">
      <label>°C <input type="number" id="thrNum" min="25" max="150" step="0.5" value="50"></label>
      <div class="hint" id="thrHint">Turns off again 2 °C below the trigger</div>
    </div>
  </div>

  <div class="stats">
    <div class="card stat"><span>Center</span><b id="center">--</b></div>
    <div class="card stat"><span>Min</span><b id="min">--</b></div>
    <div class="card stat"><span>Max</span><b id="max">--</b></div>
    <div class="card stat"><span>Cursor</span><b id="cursor">--</b></div>
    <div class="card stat"><span>Motors active</span><b id="active">--</b></div>
  </div>

  <div class="card">
    <div class="card-header"><div><h3 class="card-title">Display settings</h3><p class="card-desc">Only affect this page, not the motors</p></div></div>
    <div class="card-content controls">
      <label><input type="checkbox" id="mirror" checked> Mirror</label>
      <label><input type="checkbox" id="interp" checked> Interpolate</label>
      <label><input type="checkbox" id="fahr"> °F</label>
      <label><input type="checkbox" id="auto" checked> Auto range</label>
      <label>Min <input type="number" id="rmin" value="20"></label>
      <label>Max <input type="number" id="rmax" value="40"></label>
      <label>Smoothing <input type="range" id="smooth" min="0" max="0.9" step="0.1" value="0.3"></label>
    </div>
  </div>
</div>

<script>
const W = 32, H = 24, N = W * H;
let hotC = 50, sock = null, editUntil = 0, sendTimer = 0;
const $ = id => document.getElementById(id);
const cv = $('c'), ctx = cv.getContext('2d');
const small = document.createElement('canvas'); small.width = W; small.height = H;
const sctx = small.getContext('2d'), simg = sctx.createImageData(W, H);

// Motor cards
['Left', 'Right'].forEach((name, i) => {
  $('motor' + i).innerHTML = `
    <div class="card-header">
      <div><h3 class="card-title">${name} motor</h3><p class="card-desc">${name} half of the view</p></div>
      <div class="head-right">
        <span class="badge secondary" id="badge${i}">Idle</span>
        <label>Enabled <input type="checkbox" id="en${i}" checked aria-label="Enable ${name.toLowerCase()} motor"></label>
      </div>
    </div>
    <div class="card-content">
      <div class="big" id="pct${i}">0<small>%</small></div>
      <div class="progress"><div id="fill${i}"></div></div>
      <div class="kv">
        <div><span>Peak temp</span><b id="peak${i}">--</b></div>
        <div><span>Hot pixels</span><b id="hot${i}">--</b></div>
      </div>
    </div>`;
});

// Motor enable switches: optimistic update, then follow what the device reports
const enEditUntil = [0, 0];
[0, 1].forEach(i => $('en' + i).addEventListener('change', e => {
  enEditUntil[i] = performance.now() + 1500;
  if (sock && sock.readyState === 1) sock.send(`en:${i}:${e.target.checked ? 1 : 0}`);
}));

// Iron palette LUT
const stops = [[0,0,0],[32,0,96],[128,0,128],[220,40,40],[255,150,0],[255,230,60],[255,255,255]];
const lut = [];
for (let i = 0; i < 256; i++) {
  const t = i / 255 * (stops.length - 1), k = Math.min(Math.floor(t), stops.length - 2), f = t - k;
  lut.push(stops[k].map((v, j) => Math.round(v + (stops[k+1][j] - v) * f)));
}
(() => {
  const g = $('bar').getContext('2d'), d = g.createImageData(256, 1);
  for (let i = 0; i < 256; i++) d.data.set([...lut[i], 255], i * 4);
  g.putImageData(d, 0, 0);
})();

const temps = new Float32Array(N);
let have = false, dirty = false, lo = 20, hi = 40, cursorPos = null;
let frames = 0, fpsT = performance.now();
const fmt = c => $('fahr').checked ? (c * 9 / 5 + 32).toFixed(1) + ' °F' : c.toFixed(1) + ' °C';

// Extra values after the pixels: [levelL, levelR, hotL, hotR, peakL*100, peakR*100, mirrorView, trigger*100, enabled bits]
function updateMotors(raw) {
  if (raw.length < N + 7) return;
  const active = [];
  for (let i = 0; i < 2; i++) {
    const level = raw[N + i], hot = raw[N + 2 + i], peak = raw[N + 4 + i] / 100, on = level > 0;
    const pct = Math.round(level / 2.55);
    if (raw.length >= N + 9 && performance.now() > enEditUntil[i]) $('en' + i).checked = !!(raw[N + 8] & (1 << i));
    const enabled = $('en' + i).checked;
    $('motor' + i).classList.toggle('on', on);
    $('motor' + i).classList.toggle('off', !enabled);
    const b = $('badge' + i);
    b.className = 'badge ' + (on ? 'destructive' : enabled ? 'secondary' : 'outline');
    b.innerHTML = on ? '<span class="dot red"></span>Vibrating' : !enabled ? (hot ? 'Disabled · hot' : 'Disabled') : 'Idle';
    $('pct' + i).innerHTML = pct + '<small>%</small>';
    $('fill' + i).style.width = pct + '%';
    $('peak' + i).textContent = peak > -200 ? fmt(peak) : '--';
    $('hot' + i).textContent = hot;
    if (on) active.push(i ? 'Right' : 'Left');
  }
  $('active').textContent = active.length === 2 ? 'Both' : active[0] || 'None';
  // Highlight the half of the picture each motor covers (depends on the Mirror switch)
  const leftIsMotor0 = $('mirror').checked === !!raw[N + 6];
  const zones = leftIsMotor0 ? [['zoneLeft','tagLeft'], ['zoneRight','tagRight']] : [['zoneRight','tagRight'], ['zoneLeft','tagLeft']];
  zones.forEach(([z, t], i) => {
    const on = raw[N + i] > 0;
    $(z).classList.toggle('on', on);
    $(t).textContent = (i ? 'Right' : 'Left') + ' motor' + (on ? ' · ' + Math.round(raw[N + i] / 2.55) + '%' : $('en' + i).checked ? '' : ' · off');
  });
}

// ---- Trigger temperature (always °C on the wire) ----
function showTrigger(c) {
  $('thr').textContent = fmt(c);
  $('thrBig').innerHTML = fmt(c).replace(/ (°[CF])/, '<small>$1</small>');
}
function syncTrigger(raw) {
  if (raw.length < N + 8) return;
  const dev = raw[N + 7] / 100;
  if (performance.now() < editUntil) return;   // don't fight the user while dragging
  const s = $('sync');
  if (Math.abs(dev - hotC) < 0.01 && s.dataset.state === 'pending') { s.textContent = '✓ Saved on device'; s.className = 'sync saved'; s.dataset.state = 'saved'; }
  else if (!s.dataset.state) { s.textContent = 'Synced with device'; s.dataset.state = 'synced'; }
  hotC = dev;
  $('thrRange').value = dev; $('thrNum').value = dev;
  showTrigger(dev);
}
function setTrigger(c) {
  if (!isFinite(c)) return;
  c = Math.min(150, Math.max(25, Math.round(c * 2) / 2));
  hotC = c;
  editUntil = performance.now() + 1500;
  $('thrRange').value = c; $('thrNum').value = c;
  showTrigger(c);
  const s = $('sync'); s.textContent = 'Saving…'; s.className = 'sync'; s.dataset.state = 'pending';
  clearTimeout(sendTimer);
  sendTimer = setTimeout(() => { if (sock && sock.readyState === 1) sock.send('thr:' + c.toFixed(1)); }, 120);
}
$('thrRange').addEventListener('input', e => setTrigger(+e.target.value));
$('thrNum').addEventListener('change', e => setTrigger(+e.target.value));
$('fahr').addEventListener('change', () => showTrigger(hotC));

function onFrame(buf) {
  const raw = new Int16Array(buf);
  if (raw.length < N) return;
  updateMotors(raw);
  syncTrigger(raw);
  const a = +$('smooth').value;
  let mn = 1e9, mx = -1e9;
  for (let i = 0; i < N; i++) {
    const t = raw[i] / 100;
    temps[i] = have ? temps[i] * a + t * (1 - a) : t;
    if (temps[i] < mn) mn = temps[i];
    if (temps[i] > mx) mx = temps[i];
  }
  have = true;
  if ($('auto').checked) { lo += (mn - lo) * 0.3; hi += (mx - hi) * 0.3; }
  else { lo = +$('rmin').value; hi = +$('rmax').value; }

  const c = (temps[11*W+15] + temps[11*W+16] + temps[12*W+15] + temps[12*W+16]) / 4;
  $('center').textContent = fmt(c); $('min').textContent = fmt(mn); $('max').textContent = fmt(mx);
  $('lo').textContent = fmt(lo); $('hi').textContent = fmt(hi);
  if (cursorPos) updateCursor();

  frames++;
  const now = performance.now();
  if (now - fpsT >= 1000) { $('fps').textContent = (frames * 1000 / (now - fpsT)).toFixed(1); frames = 0; fpsT = now; }
  dirty = true;
}

function draw() {
  if (dirty) {
    dirty = false;
    const span = Math.max(hi - lo, 0.5);
    for (let i = 0; i < N; i++) {
      const v = Math.max(0, Math.min(255, ((temps[i] - lo) / span * 255) | 0));
      const col = lut[v], p = i * 4;
      simg.data[p] = col[0]; simg.data[p+1] = col[1]; simg.data[p+2] = col[2]; simg.data[p+3] = 255;
    }
    sctx.putImageData(simg, 0, 0);
    ctx.save();
    ctx.imageSmoothingEnabled = $('interp').checked;
    ctx.imageSmoothingQuality = 'high';
    if ($('mirror').checked) { ctx.translate(cv.width, 0); ctx.scale(-1, 1); }
    ctx.drawImage(small, 0, 0, cv.width, cv.height);   // GPU-scaled
    ctx.restore();
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function sample(x, y) {
  const x0 = Math.min(Math.floor(x), W - 2), y0 = Math.min(Math.floor(y), H - 2);
  const fx = x - x0, fy = y - y0, i = y0 * W + x0;
  return temps[i]*(1-fx)*(1-fy) + temps[i+1]*fx*(1-fy) + temps[i+W]*(1-fx)*fy + temps[i+W+1]*fx*fy;
}
function updateCursor() {
  let sx = cursorPos.x * (W - 1), sy = cursorPos.y * (H - 1);
  if ($('mirror').checked) sx = (W - 1) - sx;
  $('cursor').textContent = fmt(sample(sx, sy));
}
cv.addEventListener('pointermove', e => {
  const r = cv.getBoundingClientRect();
  cursorPos = { x: (e.clientX - r.left) / r.width, y: (e.clientY - r.top) / r.height };
  updateCursor();
});

function connect() {
  sock = new WebSocket(`ws://${location.hostname}:81/`);
  sock.binaryType = 'arraybuffer';
  const setStatus = (txt, live) => { $('status').textContent = txt; $('statusPill').classList.toggle('live', live); };
  sock.onopen = () => setStatus('Live', true);
  sock.onmessage = e => onFrame(e.data);
  sock.onclose = () => { setStatus('Reconnecting…', false); setTimeout(connect, 1000); };
  sock.onerror = () => sock.close();
}
connect();
</script>
</body>
</html>
)rawliteral";

// ---------------- Vibration motors ----------------
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

static const uint8_t motorPins[2] = {MOTOR_L_PIN, MOTOR_R_PIN};

void motorsBegin() {
  for (int s = 0; s < 2; s++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(motorPins[s], MOTOR_PWM_FREQ, 8);
#else
    ledcSetup(s, MOTOR_PWM_FREQ, 8);
    ledcAttachPin(motorPins[s], s);
#endif
  }
}

void motorWrite(int side, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motorPins[side], duty);
#else
  ledcWrite(side, duty);
#endif
  motorLevel[side] = duty;
}

void motorsOff() {
  motorWrite(0, 0);
  motorWrite(1, 0);
}

// Startup check: buzz left, then right, so you can feel both motors work and are on the right sides.
void motorsSelfTest() {
  const char* names[2] = {"left", "right"};
  for (int s = 0; s < 2; s++) {
    Serial.printf("Motor test: %s\n", names[s]);
    motorWrite(s, 255);
    delay(400);
    motorWrite(s, 0);
    delay(1000);   // 1 s gap so left and right are easy to tell apart
  }
}

// Split the frame down the middle; vibrate the side that sees something > hotC,
// stronger the closer the hot object appears.
void updateMotors(const float* frame) {
  static bool active[2] = {false, false};
  static float level[2] = {0, 0};
  const float trigger = hotC, release = trigger - HOT_HYSTERESIS_C;
  int hot[2] = {0, 0};
  float peak[2] = {-273, -273};

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      float t = frame[y * W + x];
      if (isnan(t)) continue;
      bool rawLeft = x < W / 2;
      int side = (rawLeft != MIRROR_VIEW) ? 0 : 1;   // with mirroring, raw left = displayed right
      if (t > peak[side]) peak[side] = t;
      if (t > (active[side] ? release : trigger)) hot[side]++;
    }
  }

  for (int s = 0; s < 2; s++) {
    sideHot[s] = hot[s];
    sidePeak[s] = (int16_t)(peak[s] * 100.0f);
    active[s] = hot[s] > 0;
    if (!active[s] || !motorEnabled[s]) {   // disabled motors still report hot pixels, they just stay still
      level[s] = 0;
      motorWrite(s, 0);
      continue;
    }
    // Apparent area falls with 1/d^2, so sqrt(area) tracks distance roughly linearly.
    float bySize = sqrtf((float)hot[s] / NEAR_PIXELS);
    float byTemp = (peak[s] - trigger) / NEAR_TEMP_SPAN_C;
    float closeness = constrain(max(bySize, byTemp), 0.0f, 1.0f);
    float target = MOTOR_MIN_DUTY + (255 - MOTOR_MIN_DUTY) * closeness;
    level[s] = level[s] == 0 ? target : level[s] * 0.5f + target * 0.5f;   // light smoothing
    motorWrite(s, (uint8_t)level[s]);
  }
}

// ---------------- Sensor task (core 0) ----------------
void sensorTask(void*) {
  static float frame[N];
  static int16_t local[N];
  uint32_t count = 0, errors = 0, t0 = millis(), lastGood = millis();

  for (;;) {
    if (mlx.getFrame(frame) == 0) {
      lastGood = millis();
      updateMotors(frame);
      for (int i = 0; i < N; i++) {
        local[i] = isnan(frame[i]) ? local[i] : (int16_t)(frame[i] * 100.0f);
      }
      xSemaphoreTake(frameMutex, portMAX_DELAY);
      memcpy(sharedFrame, local, sizeof(local));
      frameSeq++;
      xSemaphoreGive(frameMutex);
      count++;
    } else {
      errors++;
      if (millis() - lastGood > MOTOR_TIMEOUT_MS) motorsOff();   // don't stay stuck buzzing
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint32_t now = millis();
    if (now - t0 >= 2000) {
      Serial.printf("Sensor: %.1f FPS, %lu errors\n", count * 1000.0f / (now - t0), errors);
      count = 0; errors = 0; t0 = now;
    }
    vTaskDelay(1);   // let the idle task run (watchdog)
  }
}

// ---------------- HTTP / WebSocket ----------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// Kept for compatibility / scripts: single frame over HTTP
void handleFrame() {
  static int16_t copy[N];
  xSemaphoreTake(frameMutex, portMAX_DELAY);
  memcpy(copy, sharedFrame, sizeof(copy));
  xSemaphoreGive(frameMutex);
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "application/octet-stream", (const char*)copy, sizeof(copy));
}

void setHotC(float c) {
  c = constrain(c, HOT_C_MIN, HOT_C_MAX);
  if (c == hotC) return;
  hotC = c;
  prefs.putFloat("hotC", c);           // remembered across reboots
  Serial.printf("Vibrate trigger set to %.1f C\n", c);
}

void setMotorEnabled(int side, bool en) {
  if (motorEnabled[side] == en) return;
  motorEnabled[side] = en;
  if (!en) motorWrite(side, 0);        // stop immediately, don't wait for the next frame
  prefs.putBool(side ? "enR" : "enL", en);
  Serial.printf("%s motor %s\n", side ? "Right" : "Left", en ? "enabled" : "disabled");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) Serial.printf("WS client %u connected\n", num);
  if (type == WStype_DISCONNECTED) Serial.printf("WS client %u disconnected\n", num);
  // Dashboard command: "thr:55.5" sets the vibrate trigger temperature (°C)
  if (type == WStype_TEXT && length > 4 && length < 32 && strncmp((char*)payload, "thr:", 4) == 0) {
    float c = atof((char*)payload + 4);
    if (!isnan(c)) setHotC(c);
  }
  // Dashboard command: "en:0:1" enables (1) / disables (0) motor 0 = left, 1 = right
  if (type == WStype_TEXT && length == 6 && strncmp((char*)payload, "en:", 3) == 0) {
    int side = payload[3] - '0';
    if ((side == 0 || side == 1) && payload[4] == ':') setMotorEnabled(side, payload[5] == '1');
  }
}

void pushFrameIfNew() {
  static uint32_t lastSent = 0;
  // pixels + [dutyL, dutyR, hotL, hotR, peakL*100, peakR*100, MIRROR_VIEW, trigger*100, enabled bits]
  static int16_t copy[N + 9];
  if (frameSeq == lastSent || ws.connectedClients() == 0) return;

  xSemaphoreTake(frameMutex, portMAX_DELAY);
  memcpy(copy, sharedFrame, sizeof(sharedFrame));
  lastSent = frameSeq;
  xSemaphoreGive(frameMutex);
  copy[N] = motorLevel[0];
  copy[N + 1] = motorLevel[1];
  copy[N + 2] = sideHot[0];
  copy[N + 3] = sideHot[1];
  copy[N + 4] = sidePeak[0];
  copy[N + 5] = sidePeak[1];
  copy[N + 6] = MIRROR_VIEW ? 1 : 0;
  copy[N + 7] = (int16_t)(hotC * 100.0f);
  copy[N + 8] = (motorEnabled[0] ? 1 : 0) | (motorEnabled[1] ? 2 : 0);

  ws.broadcastBIN((uint8_t*)copy, sizeof(copy));
}

// ---------------- Setup / loop ----------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // power save adds big latency spikes
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open: http://");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Thermal");
    Serial.print("Wi-Fi failed, AP mode. Open: http://");
    Serial.println(WiFi.softAPIP());
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  setCpuFrequencyMhz(240);

  motorsBegin();
  motorsOff();
  motorsSelfTest();

  prefs.begin("haptics", false);
  hotC = constrain(prefs.getFloat("hotC", HOT_C_DEFAULT), HOT_C_MIN, HOT_C_MAX);
  motorEnabled[0] = prefs.getBool("enL", true);
  motorEnabled[1] = prefs.getBool("enR", true);
  Serial.printf("Vibrate trigger: %.1f C, motors L=%s R=%s\n", hotC,
                motorEnabled[0] ? "on" : "off", motorEnabled[1] ? "on" : "off");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found! Check wiring (SDA/SCL/3V3/GND).");
    while (true) delay(1000);
  }
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(SENSOR_RATE);
  Serial.println("MLX90640 ready");

  frameMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "mlx", 8192, nullptr, 2, nullptr, 0);

  connectWiFi();
  if (MDNS.begin("thermal")) Serial.println("mDNS: http://thermal.local/");

  server.on("/", handleRoot);
  server.on("/frame", handleFrame);
  server.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
}

void loop() {
  server.handleClient();
  ws.loop();
  pushFrameIfNew();
  delay(1);
}