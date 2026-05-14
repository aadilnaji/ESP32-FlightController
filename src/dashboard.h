#ifndef DASHBOARD_H
#define DASHBOARD_H

/** Web debug dashboard served by the ESP32 (HTML/CSS/JS in a raw string literal). */

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FC Debug</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0d1117;color:#c9d1d9;font-size:13px;padding:8px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:8px;max-width:1400px;margin:0 auto}
.c{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px}
.c h3{color:#58a6ff;font-size:12px;margin-bottom:8px;padding-bottom:4px;border-bottom:1px solid #21262d}
.sb{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:8px}
.st{padding:3px 10px;border-radius:12px;font-size:11px;font-weight:600}
.armed{background:#da3633;color:#fff}
.disarmed{background:#238636;color:#fff}
.acro{background:#8957e5;color:#fff}
.stab{background:#1f6feb;color:#fff}
.ok{background:#238636;color:#fff}
.warn{background:#d29922;color:#fff}
.bad{background:#da3633;color:#fff}
.headless{background:#a855f7;color:#fff}
.neu{background:#30363d;color:#8b949e}
.r{display:flex;justify-content:space-between;padding:2px 0;font-size:12px}
.r span:first-child{color:#8b949e}
.r span:last-child{font-family:monospace;color:#e6edf3}
.r.h{color:#58a6ff;font-weight:600;border-bottom:1px solid #21262d;margin:4px 0}
.bar{height:6px;background:#21262d;border-radius:3px;margin:2px 0;overflow:hidden}
.bf{height:100%;transition:width .1s}
.bf.m{background:linear-gradient(90deg,#238636,#d29922,#da3633)}
.bf.e{background:#58a6ff}
.bf.p{background:#3fb950}
.bf.i{background:#a371f7}
.bf.d{background:#f78166}
.bf.sat{background:#da3633!important}
.hz{width:100%;height:100px;background:#0d1117;border-radius:4px;position:relative;overflow:hidden}
.hz-i{position:absolute;width:100%;height:200%;background:linear-gradient(#1f6feb 50%,#6e3c00 50%);transform-origin:center 25%}
.hz-l{position:absolute;top:50%;left:0;right:0;height:1px;background:#fff}
.hz-m{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:50px;height:2px;background:#f0883e}
.stk{display:flex;gap:12px;justify-content:center}
.stk-c{text-align:center}
.stk-b{width:90px;height:90px;background:#0d1117;border-radius:6px;position:relative;border:1px solid #30363d}
.stk-h,.stk-v{position:absolute;background:#30363d}
.stk-h{top:50%;left:10%;right:10%;height:1px}
.stk-v{left:50%;top:10%;bottom:10%;width:1px}
.stk-d{position:absolute;width:12px;height:12px;background:#58a6ff;border-radius:50%;transform:translate(-50%,-50%);left:50%;top:50%;box-shadow:0 0 6px #58a6ff80}
.stk-l{font-size:10px;color:#8b949e;margin-bottom:4px}
.stk-vals{display:flex;justify-content:space-between;font-size:10px;margin-top:4px;color:#8b949e}
.stk-vals span{font-family:monospace;color:#c9d1d9}
.err-b{display:flex;align-items:center;gap:6px;margin:2px 0}
.err-b label{color:#8b949e;font-size:11px;width:35px}
.err-t{flex:1;height:4px;background:#21262d;border-radius:2px;position:relative}
.err-f{position:absolute;height:100%;background:#d29922;transition:all .1s}
.err-c{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#58a6ff}
.err-v{font-family:monospace;font-size:10px;width:45px;text-align:right}
.tabs{display:flex;gap:4px;margin-bottom:8px;flex-wrap:wrap}
.tab{padding:4px 10px;background:#21262d;border:none;color:#8b949e;border-radius:4px;cursor:pointer;font-size:11px}
.tab.a{background:#1f6feb;color:#fff}
.tc{display:none}
.tc.a{display:block}
.pg{margin-bottom:10px}
.pg label{display:block;color:#8b949e;font-size:10px;margin-bottom:2px;text-transform:uppercase}
.pr{display:flex;gap:4px;margin-bottom:4px}
.pr input{flex:1;background:#0d1117;border:1px solid #30363d;color:#c9d1d9;padding:5px;border-radius:4px;font-size:12px;width:55px}
.pr input:focus{outline:none;border-color:#58a6ff}
.btn{background:#238636;color:#fff;border:none;padding:5px 12px;border-radius:4px;cursor:pointer;font-size:11px;font-weight:600}
.btn:hover{background:#2ea043}
.btn-s{padding:3px 8px}
.btn-w{background:#9e6a03}
.btn-w:hover{background:#bb8009}
.btn-d{background:#da3633}
.btn-d:hover{background:#f85149}
.btn-b{background:#1f6feb}
.btn-b:hover{background:#388bfd}
.info{color:#8b949e;font-size:10px;font-style:italic;margin:4px 0}
.pid-brk{display:grid;grid-template-columns:repeat(4,1fr);gap:2px;margin-top:4px;font-size:10px}
.pid-brk>div{text-align:center;padding:2px;background:#0d1117;border-radius:2px}
.pid-brk .lbl{color:#8b949e}
.w2{grid-column:span 2}
input[type=checkbox]{width:14px;height:14px;margin-right:6px}
.ft{display:flex;align-items:center;margin-bottom:6px}
.ft label{color:#8b949e;font-size:12px;flex:1}
.ft input[type=number]{width:60px;background:#0d1117;border:1px solid #30363d;color:#c9d1d9;padding:4px;border-radius:4px}
.trim-ctrl{display:flex;align-items:center;gap:8px;margin:4px 0}
.trim-ctrl label{color:#8b949e;font-size:11px;width:40px}
.trim-ctrl input[type=range]{flex:1;height:6px;-webkit-appearance:none;background:#21262d;border-radius:3px;outline:none}
.trim-ctrl input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:14px;height:14px;background:#58a6ff;border-radius:50%;cursor:pointer}
.trim-ctrl .trim-val{font-family:monospace;font-size:11px;width:50px;text-align:right;color:#e6edf3}
.trim-num{width:60px!important;text-align:center}
.trim-box{background:#0d1117;border:1px solid #30363d;border-radius:4px;padding:8px;margin-top:8px}
.trim-box h4{color:#8b949e;font-size:10px;margin-bottom:6px;text-transform:uppercase}
.cal-result{background:#0d1117;border:1px solid #30363d;border-radius:4px;padding:8px;margin-top:8px;font-family:monospace;font-size:11px;display:none}
.cal-result.show{display:block}
.cal-result code{color:#7ee787;word-break:break-all}
</style>
</head>
<body>
<div class="sb">
<span id="arm" class="st disarmed">DISARMED</span>
<span id="mode" class="st stab">STABILIZE</span>
<span id="headless" class="st neu" style="display:none">HEADLESS</span>
<span id="rc" class="st neu">RC: ---</span>
<span id="fuse" class="st warn">CONVERGING</span>
<span id="loop" class="st neu">Loop: ---µs</span>
<span id="acc" class="st neu">Acc: ?</span>
<span id="mag" class="st neu">Mag: ?</span>
<span id="air" class="st neu">Air: OFF</span>
</div>

<div class="g">
<!-- Attitude -->
<div class="c">
<h3>Attitude</h3>
<div class="hz"><div id="hzI" class="hz-i"></div><div class="hz-l"></div><div class="hz-m"></div></div>
<div class="r"><span>Roll</span><span id="roll">0.0°</span></div>
<div class="r"><span>Pitch</span><span id="pitch">0.0°</span></div>
<div class="r"><span>Yaw</span><span id="yaw">0.0°</span></div>
<div class="r h">Angle Error</div>
<div class="err-b"><label>Roll</label><div class="err-t"><div class="err-c"></div><div id="errRF" class="err-f"></div></div><span id="errR" class="err-v">0.0°</span></div>
<div class="err-b"><label>Pitch</label><div class="err-t"><div class="err-c"></div><div id="errPF" class="err-f"></div></div><span id="errP" class="err-v">0.0°</span></div>
<div class="trim-box">
<h4>Level Trim (Stabilize Mode)</h4>
<div class="trim-ctrl">
<label>Roll</label>
<input type="range" id="trimR" min="-5" max="5" step="0.1" value="0">
<span id="trimRV" class="trim-val">0.0°</span>
</div>
<div class="trim-ctrl">
<label>Pitch</label>
<input type="range" id="trimP" min="-5" max="5" step="0.1" value="0">
<span id="trimPV" class="trim-val">0.0°</span>
</div>
<div style="display:flex;gap:4px;margin-top:6px">
<button class="btn btn-s" onclick="applyTrim()">Apply</button>
<button class="btn btn-s btn-w" onclick="resetTrim()">Reset</button>
<button class="btn btn-s btn-b" onclick="loadTrim()">Load</button>
</div>
</div>
</div>

<!-- RC Input -->
<div class="c">
<h3>RC Input</h3>
<div class="stk">
<div class="stk-c"><div class="stk-l">Left (Thr/Yaw)</div><div class="stk-b"><div class="stk-h"></div><div class="stk-v"></div><div id="stkL" class="stk-d"></div></div><div class="stk-vals"><span id="rcT">1000</span><span id="rcY">0.00</span></div></div>
<div class="stk-c"><div class="stk-l">Right (Roll/Pitch)</div><div class="stk-b"><div class="stk-h"></div><div class="stk-v"></div><div id="stkR" class="stk-d"></div></div><div class="stk-vals"><span id="rcR">0.00</span><span id="rcP">0.00</span></div></div>
</div>
<div class="r h" style="margin-top:8px">Raw Values</div>
<div class="r"><span>Throttle (µs)</span><span id="rcTr">1000</span></div>
<div class="r"><span>Roll</span><span id="rcRr">0.000</span></div>
<div class="r"><span>Pitch</span><span id="rcPr">0.000</span></div>
<div class="r"><span>Yaw</span><span id="rcYr">0.000</span></div>
<div class="r"><span>Arm Switch</span><span id="rcArm">1000</span></div>
<div class="r"><span>Mode Switch</span><span id="rcMode">1000</span></div>
</div>

<!-- Gyro & Rates -->
<div class="c">
<h3>Gyro & Rate Loop</h3>
<div class="r h">Gyro (°/s)</div>
<div class="r"><span>Raw X</span><span id="gRx">0.0</span></div>
<div class="r"><span>Raw Y</span><span id="gRy">0.0</span></div>
<div class="r"><span>Raw Z</span><span id="gRz">0.0</span></div>
<div class="r"><span>Filt X</span><span id="gFx">0.0</span></div>
<div class="r"><span>Filt Y</span><span id="gFy">0.0</span></div>
<div class="r"><span>Filt Z</span><span id="gFz">0.0</span></div>
<div class="r h">Rate Setpoint (°/s)</div>
<div class="r"><span>Roll</span><span id="rsR">0.0</span></div>
<div class="r"><span>Pitch</span><span id="rsP">0.0</span></div>
<div class="r"><span>Yaw</span><span id="rsY">0.0</span></div>
<div class="r h">Rate Error (°/s)</div>
<div class="r"><span>Roll</span><span id="reR">0.0</span></div>
<div class="r"><span>Pitch</span><span id="reP">0.0</span></div>
<div class="r"><span>Yaw</span><span id="reY">0.0</span></div>
</div>

<!-- PID Output Breakdown -->
<div class="c">
<h3>PID Debug</h3>
<div class="r h">Roll PID</div>
<div class="pid-brk">
<div><span class="lbl">P</span><br><span id="prR">0</span></div>
<div><span class="lbl">I</span><br><span id="irR">0</span></div>
<div><span class="lbl">D</span><br><span id="drR">0</span></div>
<div><span class="lbl">F</span><br><span id="frR">0</span></div>
</div>
<div class="r"><span>Total</span><span id="trR">0.0</span></div>
<div class="r h">Pitch PID</div>
<div class="pid-brk">
<div><span class="lbl">P</span><br><span id="prP">0</span></div>
<div><span class="lbl">I</span><br><span id="irP">0</span></div>
<div><span class="lbl">D</span><br><span id="drP">0</span></div>
<div><span class="lbl">F</span><br><span id="frP">0</span></div>
</div>
<div class="r"><span>Total</span><span id="trP">0.0</span></div>
<div class="r h">Yaw PID</div>
<div class="pid-brk">
<div><span class="lbl">P</span><br><span id="prY">0</span></div>
<div><span class="lbl">I</span><br><span id="irY">0</span></div>
<div><span class="lbl">D</span><br><span id="drY">0</span></div>
<div><span class="lbl">F</span><br><span id="frY">0</span></div>
</div>
<div class="r"><span>Total</span><span id="trY">0.0</span></div>
<div class="r" style="margin-top:6px"><span>TPA Factor</span><span id="tpa">1.00</span></div>
</div>

<!-- Motors -->
<div class="c">
<h3>Motors</h3>
<div class="r"><span>Throttle In</span><span id="thrIn">1000</span></div>
<div class="r h">Output (µs)</div>
<div class="r"><span>M1 (FR)</span><span id="m1v">1000</span></div>
<div class="bar"><div id="m1" class="bf m" style="width:0"></div></div>
<div class="r"><span>M2 (RR)</span><span id="m2v">1000</span></div>
<div class="bar"><div id="m2" class="bf m" style="width:0"></div></div>
<div class="r"><span>M3 (RL)</span><span id="m3v">1000</span></div>
<div class="bar"><div id="m3" class="bf m" style="width:0"></div></div>
<div class="r"><span>M4 (FL)</span><span id="m4v">1000</span></div>
<div class="bar"><div id="m4" class="bf m" style="width:0"></div></div>
<div class="r h">Mix Commands</div>
<div class="r"><span>Roll</span><span id="mixR">0.0</span></div>
<div class="r"><span>Pitch</span><span id="mixP">0.0</span></div>
<div class="r"><span>Yaw</span><span id="mixY">0.0</span></div>
<div class="r"><span id="satLbl" style="color:#8b949e">Saturation</span><span id="sat">None</span></div>
</div>

<!-- Accelerometer -->
<div class="c">
<h3>Accelerometer (m/s²)</h3>
<div class="r"><span>X</span><span id="ax">0.0</span></div>
<div class="r"><span>Y</span><span id="ay">0.0</span></div>
<div class="r"><span>Z</span><span id="az">0.0</span></div>
<div class="r"><span>Magnitude</span><span id="amag">9.81</span></div>
<div class="r h">Magnetometer</div>
<div class="r"><span>X</span><span id="mx">0.0</span></div>
<div class="r"><span>Y</span><span id="my">0.0</span></div>
<div class="r"><span>Z</span><span id="mz">0.0</span></div>
<div class="r"><span>Heading</span><span id="hdg">0°</span></div>
</div>

<!-- Configuration -->
<div class="c w2">
<h3>Configuration</h3>
<div class="tabs">
<button class="tab a" onclick="stab('rate')">Rate PID</button>
<button class="tab" onclick="stab('angle')">Angle PID</button>
<button class="tab" onclick="stab('trim')">Trim & Cal</button>
<button class="tab" onclick="stab('ahrs')">AHRS</button>
<button class="tab" onclick="stab('filt')">Filters</button>
<button class="tab" onclick="stab('rates')">Rates</button>
</div>

<div id="t-rate" class="tc a">
<p class="info">Inner loop @ 1kHz - Rate error → Motor command</p>
<div class="pg"><label>Roll</label><div class="pr">
<input type="number" id="rr_p" step="0.1" placeholder="P">
<input type="number" id="rr_i" step="0.01" placeholder="I">
<input type="number" id="rr_d" step="0.001" placeholder="D">
<input type="number" id="rr_f" step="0.01" placeholder="F">
<button class="btn btn-s" onclick="setPID('rr')">Set</button>
</div></div>
<div class="pg"><label>Pitch</label><div class="pr">
<input type="number" id="rp_p" step="0.1" placeholder="P">
<input type="number" id="rp_i" step="0.01" placeholder="I">
<input type="number" id="rp_d" step="0.001" placeholder="D">
<input type="number" id="rp_f" step="0.01" placeholder="F">
<button class="btn btn-s" onclick="setPID('rp')">Set</button>
</div></div>
<div class="pg"><label>Yaw</label><div class="pr">
<input type="number" id="ry_p" step="0.1" placeholder="P">
<input type="number" id="ry_i" step="0.01" placeholder="I">
<input type="number" id="ry_d" step="0.001" placeholder="D">
<input type="number" id="ry_f" step="0.01" placeholder="F">
<button class="btn btn-s" onclick="setPID('ry')">Set</button>
</div></div>
</div>

<div id="t-angle" class="tc">
<p class="info">Outer loop @ 250Hz - Angle error → Rate setpoint</p>
<div class="pg"><label>Roll</label><div class="pr">
<input type="number" id="ar_p" step="0.1" placeholder="P">
<input type="number" id="ar_i" step="0.01" placeholder="I">
<input type="number" id="ar_d" step="0.01" placeholder="D">
<button class="btn btn-s" onclick="setPID('ar')">Set</button>
</div></div>
<div class="pg"><label>Pitch</label><div class="pr">
<input type="number" id="ap_p" step="0.1" placeholder="P">
<input type="number" id="ap_i" step="0.01" placeholder="I">
<input type="number" id="ap_d" step="0.01" placeholder="D">
<button class="btn btn-s" onclick="setPID('ap')">Set</button>
</div></div>
<div class="pg"><label>Max Angle (deg)</label><div class="pr">
<input type="number" id="maxAng" step="5" value="40" style="width:80px">
<button class="btn btn-s" onclick="setMaxAngle()">Set</button>
</div></div>
</div>

<div id="t-trim" class="tc">
<p class="info">Level trim adjusts hover attitude. Accel calibration should be done once on a level surface.</p>

<div class="pg">
<label>Level Trim (degrees)</label>
<div class="pr">
<input type="number" id="trimRollIn" step="0.1" placeholder="Roll" class="trim-num">
<input type="number" id="trimPitchIn" step="0.1" placeholder="Pitch" class="trim-num">
<button class="btn btn-s" onclick="setTrimFromInputs()">Set</button>
<button class="btn btn-s btn-w" onclick="resetTrim()">Zero</button>
</div>
<p class="info">Positive roll = drone rolls right to hover. Positive pitch = nose down to hover.</p>
</div>

<div class="pg">
<label>Current Trim</label>
<div class="r"><span>Roll Trim</span><span id="curTrimR">0.0°</span></div>
<div class="r"><span>Pitch Trim</span><span id="curTrimP">0.0°</span></div>
</div>

<div class="pg" style="margin-top:16px;padding-top:12px;border-top:1px solid #21262d">
<label>One-Time Accelerometer Calibration</label>
<p class="info">Place drone perfectly level, then click calibrate. Copy the output values to your code as ACCEL_OFFSET_X/Y/Z defines.</p>
<button class="btn btn-d" onclick="runAccelCal()">Calibrate Accel (Level Surface Only!)</button>
<div id="calResult" class="cal-result">
<p style="color:#8b949e;margin-bottom:4px">Copy these values to main.cpp:</p>
<code id="calCode"></code>
</div>
</div>

<div class="pg" style="margin-top:16px;padding-top:12px;border-top:1px solid #21262d">
<label>Quick Trim Adjustment</label>
<p class="info">Use these during hover testing. Watch the drone and adjust until it holds position.</p>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px">
<div>
<p style="color:#8b949e;font-size:10px;margin-bottom:4px">Roll (left/right drift)</p>
<div style="display:flex;gap:4px">
<button class="btn btn-s" onclick="nudgeTrim('roll',-0.5)">◀ -0.5</button>
<button class="btn btn-s" onclick="nudgeTrim('roll',-0.1)">-0.1</button>
<button class="btn btn-s" onclick="nudgeTrim('roll',0.1)">+0.1</button>
<button class="btn btn-s" onclick="nudgeTrim('roll',0.5)">+0.5 ▶</button>
</div>
</div>
<div>
<p style="color:#8b949e;font-size:10px;margin-bottom:4px">Pitch (forward/back drift)</p>
<div style="display:flex;gap:4px">
<button class="btn btn-s" onclick="nudgeTrim('pitch',-0.5)">▼ -0.5</button>
<button class="btn btn-s" onclick="nudgeTrim('pitch',-0.1)">-0.1</button>
<button class="btn btn-s" onclick="nudgeTrim('pitch',0.1)">+0.1</button>
<button class="btn btn-s" onclick="nudgeTrim('pitch',0.5)">+0.5 ▲</button>
</div>
</div>
</div>
</div>
</div>

<div id="t-ahrs" class="tc">
<p class="info">Mahony AHRS sensor fusion tuning</p>
<div class="pg"><label>Accelerometer Correction</label><div class="pr">
<input type="number" id="kpA" step="0.1" placeholder="Kp" title="Proportional gain">
<input type="number" id="kiA" step="0.01" placeholder="Ki" title="Integral gain">
</div></div>
<div class="pg"><label>Magnetometer Correction</label><div class="pr">
<input type="number" id="kpM" step="0.01" placeholder="Kp">
<input type="number" id="kiM" step="0.001" placeholder="Ki">
</div></div>
<div class="pg"><label>Integral Limit</label><div class="pr">
<input type="number" id="iLim" step="0.1" value="0.5" style="width:80px">
</div></div>
<button class="btn" onclick="setAHRS()">Apply AHRS</button>
<button class="btn btn-w" onclick="resetAHRS()">Reset Filter</button>
<button class="btn btn-d" onclick="if(confirm('Rotate drone in all directions for 30s'))startMagCal()">Cal Mag</button>
</div>

<div id="t-filt" class="tc">
<p class="info">Gyro filter for rate PID. AHRS uses raw gyro.</p>
<div class="ft"><input type="checkbox" id="gfEn"><label>Gyro Lowpass</label><input type="number" id="gfHz" value="100"></div>
<div class="ft"><input type="checkbox" id="d1En"><label>D-term LP1</label><input type="number" id="d1Hz" value="100"></div>
<div class="ft"><input type="checkbox" id="d2En"><label>D-term LP2</label><input type="number" id="d2Hz" value="200"></div>
<div class="ft"><input type="checkbox" id="dnEn"><label>D-term Notch</label><input type="number" id="dnHz" value="150"></div>
<div class="ft"><input type="checkbox" id="irEn" checked><label>I-term Relax</label><input type="number" id="irCut" value="40"></div>
<button class="btn" onclick="setFilters()">Apply Filters</button>
</div>

<div id="t-rates" class="tc">
<p class="info">Default rate curves (Acro mode)</p>
<div class="pg"><label>Roll (RC / Super / Expo / Limit)</label><div class="pr">
<input type="number" id="rRc" placeholder="RC">
<input type="number" id="rSu" placeholder="Super">
<input type="number" id="rEx" placeholder="Expo">
<input type="number" id="rLm" placeholder="Limit">
<button class="btn btn-s" onclick="setRates(0)">Set</button>
</div></div>
<div class="pg"><label>Pitch</label><div class="pr">
<input type="number" id="pRc" placeholder="RC">
<input type="number" id="pSu" placeholder="Super">
<input type="number" id="pEx" placeholder="Expo">
<input type="number" id="pLm" placeholder="Limit">
<button class="btn btn-s" onclick="setRates(1)">Set</button>
</div></div>
<div class="pg"><label>Yaw</label><div class="pr">
<input type="number" id="yRc" placeholder="RC">
<input type="number" id="ySu" placeholder="Super">
<input type="number" id="yEx" placeholder="Expo">
<input type="number" id="yLm" placeholder="Limit">
<button class="btn btn-s" onclick="setRates(2)">Set</button>
</div></div>
</div>

<div style="margin-top:10px;padding-top:8px;border-top:1px solid #21262d">
<button class="btn" onclick="loadPIDs()">Load All</button>
<button class="btn btn-w" onclick="if(confirm('Reset PIDs to defaults?'))resetPIDs()">Reset PIDs</button>
<button class="btn btn-d" onclick="if(confirm('Emergency stop?'))emergencyStop()">E-STOP</button>
<button class="btn btn-b" id="headlessBtn" onclick="toggleHeadless()">Headless: OFF</button>
</div>
</div>

<!-- Loop Stats -->
<div class="c">
<h3>Loop Statistics</h3>
<div class="r"><span>Current (µs)</span><span id="lpCur">0</span></div>
<div class="r"><span>Min (µs)</span><span id="lpMin">0</span></div>
<div class="r"><span>Max (µs)</span><span id="lpMax">0</span></div>
<div class="r"><span>Avg (µs)</span><span id="lpAvg">0</span></div>
<div class="r"><span>Jitter (µs)</span><span id="lpJit">0</span></div>
<div class="r"><span>Target (µs)</span><span>1000</span></div>
<div class="r h">Timing Health</div>
<div class="r"><span>Overruns</span><span id="ovr">0</span></div>
<div class="r"><span>Uptime</span><span id="upt">0s</span></div>
<button class="btn btn-s" style="margin-top:8px" onclick="resetLoopStats()">Reset Stats</button>
</div>
</div>

<script>
// Local trim state mirrored from /trim
let currentTrimRoll = 0;
let currentTrimPitch = 0;

function stab(n){document.querySelectorAll('.tc').forEach(t=>t.classList.remove('a'));document.querySelectorAll('.tab').forEach(t=>t.classList.remove('a'));document.getElementById('t-'+n).classList.add('a');event.target.classList.add('a')}
function setPID(a){const p=document.getElementById(a+'_p').value,i=document.getElementById(a+'_i').value,d=document.getElementById(a+'_d').value,f=document.getElementById(a+'_f')?.value||0;fetch(`/pid?axis=${a}&kp=${p}&ki=${i}&kd=${d}&kf=${f}`)}
function setRates(a){const n=['r','p','y'][a];fetch(`/rates?axis=${a}&rc=${document.getElementById(n+'Rc').value}&super=${document.getElementById(n+'Su').value}&expo=${document.getElementById(n+'Ex').value}&limit=${document.getElementById(n+'Lm').value}`)}
function setAHRS(){fetch(`/mahony?kpa=${document.getElementById('kpA').value}&kia=${document.getElementById('kiA').value}&kpm=${document.getElementById('kpM').value}&kim=${document.getElementById('kiM').value}&limit=${document.getElementById('iLim').value}`)}
function resetAHRS(){fetch('/mahony?reset=1')}
function startMagCal(){fetch('/magcal')}
function setFilters(){const p=new URLSearchParams();p.append('gf_en',document.getElementById('gfEn').checked?1:0);p.append('gf_hz',document.getElementById('gfHz').value);p.append('d1_en',document.getElementById('d1En').checked?1:0);p.append('d1_hz',document.getElementById('d1Hz').value);p.append('d2_en',document.getElementById('d2En').checked?1:0);p.append('d2_hz',document.getElementById('d2Hz').value);p.append('dn_en',document.getElementById('dnEn').checked?1:0);p.append('dn_hz',document.getElementById('dnHz').value);p.append('ir_en',document.getElementById('irEn').checked?1:0);p.append('ir_cut',document.getElementById('irCut').value);fetch('/filters?'+p)}
function setMaxAngle(){fetch('/maxangle?deg='+document.getElementById('maxAng').value)}
function resetPIDs(){fetch('/resetpids');setTimeout(loadPIDs,500)}
function emergencyStop(){fetch('/estop')}
function resetLoopStats(){fetch('/resetstats')}

// ===== Headless =====
function toggleHeadless(){
    fetch('/headless?toggle=1').then(r => r.json()).then(d => {
        updateHeadlessUI(d.active, d.refYaw);
    });
}
function updateHeadlessUI(active, refYaw){
    const badge = document.getElementById('headless');
    const btn = document.getElementById('headlessBtn');
    if(active){
        badge.style.display = 'inline';
        badge.textContent = 'HEADLESS: ' + refYaw.toFixed(0) + '°';
        badge.className = 'st headless';
        btn.textContent = 'Headless: ON';
        btn.className = 'btn headless';
    } else {
        badge.style.display = 'none';
        btn.textContent = 'Headless: OFF';
        btn.className = 'btn btn-b';
    }
}

// ===== Trim =====
function applyTrim(){
    const r = parseFloat(document.getElementById('trimR').value) || 0;
    const p = parseFloat(document.getElementById('trimP').value) || 0;
    fetch(`/trim?roll=${r}&pitch=${p}`).then(()=>{
        currentTrimRoll = r;
        currentTrimPitch = p;
        updateTrimDisplay();
    });
}

function setTrimFromInputs(){
    const r = parseFloat(document.getElementById('trimRollIn').value) || 0;
    const p = parseFloat(document.getElementById('trimPitchIn').value) || 0;
    fetch(`/trim?roll=${r}&pitch=${p}`).then(()=>{
        currentTrimRoll = r;
        currentTrimPitch = p;
        document.getElementById('trimR').value = r;
        document.getElementById('trimP').value = p;
        updateTrimDisplay();
    });
}

function resetTrim(){
    fetch('/trim?roll=0&pitch=0').then(()=>{
        currentTrimRoll = 0;
        currentTrimPitch = 0;
        document.getElementById('trimR').value = 0;
        document.getElementById('trimP').value = 0;
        document.getElementById('trimRollIn').value = '';
        document.getElementById('trimPitchIn').value = '';
        updateTrimDisplay();
    });
}

async function loadTrim(){
    try {
        const r = await fetch('/trim');
        const d = await r.json();
        currentTrimRoll = d.roll || 0;
        currentTrimPitch = d.pitch || 0;
        document.getElementById('trimR').value = currentTrimRoll;
        document.getElementById('trimP').value = currentTrimPitch;
        document.getElementById('trimRollIn').value = currentTrimRoll;
        document.getElementById('trimPitchIn').value = currentTrimPitch;
        updateTrimDisplay();
    } catch(e) {
        console.error('Failed to load trim:', e);
    }
}

function nudgeTrim(axis, delta){
    if(axis === 'roll'){
        currentTrimRoll = Math.max(-10, Math.min(10, currentTrimRoll + delta));
        document.getElementById('trimR').value = currentTrimRoll;
    } else {
        currentTrimPitch = Math.max(-10, Math.min(10, currentTrimPitch + delta));
        document.getElementById('trimP').value = currentTrimPitch;
    }
    fetch(`/trim?roll=${currentTrimRoll}&pitch=${currentTrimPitch}`);
    updateTrimDisplay();
}

function updateTrimDisplay(){
    document.getElementById('trimRV').textContent = currentTrimRoll.toFixed(1) + '°';
    document.getElementById('trimPV').textContent = currentTrimPitch.toFixed(1) + '°';
    document.getElementById('curTrimR').textContent = currentTrimRoll.toFixed(1) + '°';
    document.getElementById('curTrimP').textContent = currentTrimPitch.toFixed(1) + '°';
}

document.getElementById('trimR').addEventListener('input', function(){
    currentTrimRoll = parseFloat(this.value);
    document.getElementById('trimRV').textContent = currentTrimRoll.toFixed(1) + '°';
});
document.getElementById('trimP').addEventListener('input', function(){
    currentTrimPitch = parseFloat(this.value);
    document.getElementById('trimPV').textContent = currentTrimPitch.toFixed(1) + '°';
});

// ===== Accel calibration =====
async function runAccelCal(){
    if(!confirm('IMPORTANT: Drone must be on a PERFECTLY LEVEL surface!\n\nThis will calibrate the accelerometer and show you the offset values to hardcode.\n\nContinue?')){
        return;
    }
    try {
        const r = await fetch('/calaccel?confirm=yes');
        if(r.ok){
            const d = await r.json();
            const code = `#define ACCEL_OFFSET_X  ${d.x.toFixed(6)}f\n#define ACCEL_OFFSET_Y  ${d.y.toFixed(6)}f\n#define ACCEL_OFFSET_Z  ${d.z.toFixed(6)}f`;
            document.getElementById('calCode').textContent = code;
            document.getElementById('calResult').classList.add('show');
        } else {
            alert('Calibration failed: ' + await r.text());
        }
    } catch(e) {
        alert('Calibration error: ' + e);
    }
}

async function loadPIDs(){
    try{
        const r=await fetch('/getpid'),d=await r.json();
        ['rr','rp','ry'].forEach(a=>{
            if(d[a]){
                document.getElementById(a+'_p').value=d[a].kp;
                document.getElementById(a+'_i').value=d[a].ki;
                document.getElementById(a+'_d').value=d[a].kd;
                if(document.getElementById(a+'_f'))document.getElementById(a+'_f').value=d[a].kf||0;
            }
        });
        ['ar','ap'].forEach(a=>{
            if(d[a]){
                document.getElementById(a+'_p').value=d[a].kp;
                document.getElementById(a+'_i').value=d[a].ki;
                document.getElementById(a+'_d').value=d[a].kd;
            }
        });
    }catch(e){}
    loadTrim();
}

function updErr(id,v,max){const f=document.getElementById(id+'F'),p=Math.min(Math.abs(v)/max*50,50);f.style.left=v>=0?'50%':(50-p)+'%';f.style.width=p+'%'}
function mPct(v){return((v-1000)/940*100).toFixed(0)+'%'}

async function poll(){
    try{
        const r=await fetch('/telem'),d=await r.json();
        document.getElementById('roll').textContent=d.r.toFixed(1)+'°';
        document.getElementById('pitch').textContent=d.p.toFixed(1)+'°';
        document.getElementById('yaw').textContent=d.y.toFixed(1)+'°';
        document.getElementById('hzI').style.transform=`rotate(${-d.r}deg) translateY(${d.p*1.5}px)`;
        
        if(d.ae){
            document.getElementById('errR').textContent=d.ae[0].toFixed(1)+'°';
            document.getElementById('errP').textContent=d.ae[1].toFixed(1)+'°';
            updErr('errR',d.ae[0],30);
            updErr('errP',d.ae[1],30);
        }
        
        // Sync trim display from telemetry, but skip while the user is dragging
        if(d.trim){
            if(document.activeElement !== document.getElementById('trimR') &&
               document.activeElement !== document.getElementById('trimP')){
                currentTrimRoll = d.trim[0];
                currentTrimPitch = d.trim[1];
                document.getElementById('trimR').value = currentTrimRoll;
                document.getElementById('trimP').value = currentTrimPitch;
                updateTrimDisplay();
            }
        }
        
        if(d.rc){
            document.getElementById('rcT').textContent=d.rc[3];
            document.getElementById('rcR').textContent=d.rc[0].toFixed(2);
            document.getElementById('rcP').textContent=d.rc[1].toFixed(2);
            document.getElementById('rcY').textContent=d.rc[2].toFixed(2);
            document.getElementById('rcTr').textContent=d.rc[3];
            document.getElementById('rcRr').textContent=d.rc[0].toFixed(3);
            document.getElementById('rcPr').textContent=d.rc[1].toFixed(3);
            document.getElementById('rcYr').textContent=d.rc[2].toFixed(3);
            const sR=document.getElementById('stkR'),sL=document.getElementById('stkL');
            sR.style.left=50+d.rc[0]*40+'%';
            sR.style.top=50-d.rc[1]*40+'%';
            sL.style.left=50+d.rc[2]*40+'%';
            sL.style.top=90-((d.rc[3]-1000)/1000)*80+'%';
        }
        
        if(d.rcs){
            document.getElementById('rcArm').textContent=d.rcs[0];
            document.getElementById('rcMode').textContent=d.rcs[1];
        }
        
        if(d.gr){
            document.getElementById('gRx').textContent=d.gr[0].toFixed(1);
            document.getElementById('gRy').textContent=d.gr[1].toFixed(1);
            document.getElementById('gRz').textContent=d.gr[2].toFixed(1);
        }
        
        if(d.gf){
            document.getElementById('gFx').textContent=d.gf[0].toFixed(1);
            document.getElementById('gFy').textContent=d.gf[1].toFixed(1);
            document.getElementById('gFz').textContent=d.gf[2].toFixed(1);
        }
        
        if(d.rs){
            document.getElementById('rsR').textContent=d.rs[0].toFixed(1);
            document.getElementById('rsP').textContent=d.rs[1].toFixed(1);
            document.getElementById('rsY').textContent=d.rs[2].toFixed(1);
        }
        
        if(d.re){
            document.getElementById('reR').textContent=d.re[0].toFixed(1);
            document.getElementById('reP').textContent=d.re[1].toFixed(1);
            document.getElementById('reY').textContent=d.re[2].toFixed(1);
        }
        
        if(d.pp){
            document.getElementById('prR').textContent=d.pp[0].toFixed(0);
            document.getElementById('prP').textContent=d.pp[1].toFixed(0);
            document.getElementById('prY').textContent=d.pp[2].toFixed(0);
        }
        
        if(d.pi){
            document.getElementById('irR').textContent=d.pi[0].toFixed(0);
            document.getElementById('irP').textContent=d.pi[1].toFixed(0);
            document.getElementById('irY').textContent=d.pi[2].toFixed(0);
        }
        
        if(d.pd){
            document.getElementById('drR').textContent=d.pd[0].toFixed(0);
            document.getElementById('drP').textContent=d.pd[1].toFixed(0);
            document.getElementById('drY').textContent=d.pd[2].toFixed(0);
        }
        
        if(d.pf){
            document.getElementById('frR').textContent=d.pf[0].toFixed(0);
            document.getElementById('frP').textContent=d.pf[1].toFixed(0);
            document.getElementById('frY').textContent=d.pf[2].toFixed(0);
        }
        
        if(d.pt){
            document.getElementById('trR').textContent=d.pt[0].toFixed(1);
            document.getElementById('trP').textContent=d.pt[1].toFixed(1);
            document.getElementById('trY').textContent=d.pt[2].toFixed(1);
        }
        
        document.getElementById('tpa').textContent=d.tpa.toFixed(2);
        document.getElementById('thrIn').textContent=d.thr;
        
        document.getElementById('m1v').textContent=d.m1;
        document.getElementById('m1').style.width=mPct(d.m1);
        document.getElementById('m2v').textContent=d.m2;
        document.getElementById('m2').style.width=mPct(d.m2);
        document.getElementById('m3v').textContent=d.m3;
        document.getElementById('m3').style.width=mPct(d.m3);
        document.getElementById('m4v').textContent=d.m4;
        document.getElementById('m4').style.width=mPct(d.m4);
        
        [['m1',d.m1],['m2',d.m2],['m3',d.m3],['m4',d.m4]].forEach(x=>{
            const e=document.getElementById(x[0]);
            e.classList.toggle('sat',x[1]>=1930||x[1]<=1010);
        });
        
        if(d.mix){
            document.getElementById('mixR').textContent=d.mix[0].toFixed(1);
            document.getElementById('mixP').textContent=d.mix[1].toFixed(1);
            document.getElementById('mixY').textContent=d.mix[2].toFixed(1);
        }
        
        document.getElementById('sat').textContent=d.sat||'None';
        document.getElementById('satLbl').style.color=d.sat&&d.sat!='None'?'#da3633':'#8b949e';
        
        if(d.acc){
            document.getElementById('ax').textContent=d.acc[0].toFixed(2);
            document.getElementById('ay').textContent=d.acc[1].toFixed(2);
            document.getElementById('az').textContent=d.acc[2].toFixed(2);
            const m=Math.sqrt(d.acc[0]**2+d.acc[1]**2+d.acc[2]**2);
            document.getElementById('amag').textContent=m.toFixed(2);
        }
        
        if(d.mag){
            document.getElementById('mx').textContent=d.mag[0].toFixed(1);
            document.getElementById('my').textContent=d.mag[1].toFixed(1);
            document.getElementById('mz').textContent=d.mag[2].toFixed(1);
            document.getElementById('hdg').textContent=(Math.atan2(d.mag[1],d.mag[0])*180/Math.PI).toFixed(0)+'°';
        }
        
        const arm=document.getElementById('arm');
        arm.textContent=d.armed?'ARMED':'DISARMED';
        arm.className='st '+(d.armed?'armed':'disarmed');
        
        const mode=document.getElementById('mode');
        mode.textContent=d.mode==0?'ACRO':'STABILIZE';
        mode.className='st '+(d.mode==0?'acro':'stab');
        
        const rc=document.getElementById('rc');
        rc.textContent='RC: '+(d.rcOk?'OK':'LOST');
        rc.className='st '+(d.rcOk?'ok':'bad');
        
        const fuse=document.getElementById('fuse');
        fuse.textContent=d.conv?'CONVERGED':'CONVERGING';
        fuse.className='st '+(d.conv?'ok':'warn');
        
        document.getElementById('loop').textContent='Loop: '+d.lp+'µs';
        
        const accS=document.getElementById('acc');
        accS.textContent='Acc: '+(d.accOk?'OK':'BAD');
        accS.className='st '+(d.accOk?'ok':'bad');
        
        const magS=document.getElementById('mag');
        magS.textContent='Mag: '+(d.magOk?'OK':'---');
        magS.className='st '+(d.magOk?'ok':'neu');
        
        const air=document.getElementById('air');
        air.textContent='Air: '+(d.air?'ON':'OFF');
        air.className='st '+(d.air?'ok':'neu');
        
        document.getElementById('gfEn').checked=d.gfOn;
        
        if(d.ls){
            document.getElementById('lpCur').textContent=d.ls[0];
            document.getElementById('lpMin').textContent=d.ls[1];
            document.getElementById('lpMax').textContent=d.ls[2];
            document.getElementById('lpAvg').textContent=d.ls[3];
            document.getElementById('lpJit').textContent=d.ls[4];
            document.getElementById('ovr').textContent=d.ls[5];
        }
        
        if(d.up)document.getElementById('upt').textContent=d.up+'s';
                
        if(d.hl !== undefined){
            updateHeadlessUI(d.hl, d.hlRef || 0);
        }
    }catch(e){}
}

loadPIDs();
setInterval(poll,100);
</script>
</body>
</html>
)rawliteral";

#endif // DASHBOARD_H