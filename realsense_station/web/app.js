/* Edge AI Perception Station — UI controller */
const video = document.getElementById('video');
const cloudCanvas = document.getElementById('cloud');
const detectbar = document.getElementById('detectbar');
const hint = document.getElementById('hint');
const notice = document.getElementById('notice');
const assistantPanel = document.getElementById('assistant');
const stage = document.querySelector('.stage');
const perfbadge = document.getElementById('perfbadge');
const valcap = document.getElementById('valcap');
const attractEl = document.getElementById('attract');
const statEl = document.getElementById('stat');
let mode = 'safety';
let enrollOpen = false, skipUntil = 0;

/* Product_RAG_ voice assistant (Flask+SocketIO, HTTPS :5001). Derive its URL
   from however the station was opened so the self-signed cert (CN=192.168.3.224)
   matches. Override with ?assistant=https://host:port if ever needed. */
const ASSISTANT = new URLSearchParams(location.search).get('assistant')
  || window.ASSISTANT_URL   /* set by the server on the Nano -> the AGX assistant */
  || ('https://' + (location.hostname || '192.168.3.224') + ':5001');
const PRODUCTS = [
  {brand:'NVIDIA Jetson', name:'Jetson AGX Orin',              key:'Jetson_AGX_Orin',         thumb:'jetsonagxorin.png'},
  {brand:'NVIDIA Jetson', name:'Jetson Orin NX',              key:'Jetson_Orin_NX',          thumb:'jetsonorinnx.png'},
  {brand:'NVIDIA Jetson', name:'Jetson Orin Nano',            key:'Jetson_Orin_Nano',        thumb:'jetsonorinnano.png'},
  {brand:'NVIDIA Jetson', name:'Jetson Nano 4GB',             key:'Jetson_Nano_4Gb',         thumb:'jetsonnano4.png'},
  {brand:'Elmos', name:'E521.39 Knob Illumination',          key:'Elmos_E521.39_Knob_Illumination',                    thumb:'e521.39.png'},
  {brand:'Elmos', name:'E522.88 Sequential Light Demo',      key:'Elmos_E522.88_Automotive_Sequential_Light_Demo',     thumb:'e52288.png'},
  {brand:'Elmos', name:'E522.95 CAN Sequential Light',       key:'Elmos_E522.95_CAN_Based_Automotive_Sequential_Light',thumb:'e522.95.png'},
  {brand:'Elmos', name:'E523.06 300W BLDC Fan Control',      key:'Elmos_E523.06_300W_BLDC_Motor_Control_For_Cooling_Fan',thumb:'e52306.png'},
  {brand:'Elmos', name:'E523.63 Coolant Control Valve',      key:'Elmos_E523.63_Coolant_Control_Valve',                thumb:'e523.63.png'},
  {brand:'Elmos', name:'E533.06 2-Wheeler Radiator',         key:'Elmos_E533.06_Automotive_2-Wheeler_Radiator',        thumb:'e533.06.png'},
  {brand:'Holtek', name:'Ceiling Fan Controller',            key:'Holtek_Ceiling_fan_Controller',                      thumb:'holtekfan3.png'},
  {brand:'Holtek', name:'High-Voltage BLDC Motor Driver',    key:'Holtek_high_voltage_BLDC_Motor_driver',              thumb:'HighvoltageBLDC.png'},
  {brand:'Holtek', name:'Low-Voltage High-Power BLDC Driver',key:'Holtek_Low_voltage_High_Power_BLDC_Motor_Driver',     thumb:'lowvolthighpower.png'},
];
function buildAssistant() {
  document.getElementById('launchBtn').href = ASSISTANT + '/';
  const grid = document.getElementById('prodgrid'); grid.innerHTML = '';
  ['NVIDIA Jetson', 'Elmos', 'Holtek'].forEach(brand => {   // one column per brand
    const col = document.createElement('div'); col.className = 'brandcol';
    const h = document.createElement('div'); h.className = 'brand'; h.textContent = brand; col.appendChild(h);
    const cards = document.createElement('div'); cards.className = 'cards';
    PRODUCTS.filter(p => p.brand === brand).forEach(p => {
      const a = document.createElement('a'); a.className = 'pcard'; a.target = '_blank'; a.rel = 'noopener';
      a.href = ASSISTANT + '/qa/' + encodeURIComponent(p.key);
      a.innerHTML = '<span class="pthumb"><img src="/thumbs/' + p.thumb + '" alt="" loading="lazy"></span>'
                  + '<span class="pn">' + p.name + '</span><span class="pgo">Ask ↗</span>';
      cards.appendChild(a);
    });
    col.appendChild(cards); grid.appendChild(col);
  });
  // QR takeaway — visitors scan to open the assistant on their own phone
  const qc = document.getElementById('qrcode');
  if (qc && window.QRCode && !qc.hasChildNodes()) {
    new QRCode(qc, {text: ASSISTANT + '/', width: 72, height: 72,
      colorDark: '#0a0b0e', colorLight: '#ffffff', correctLevel: QRCode.CorrectLevel.M});
  }
}
buildAssistant();

const HINTS = {
  safety: 'Walk toward the camera — the border reacts to how close you are.',
  dimension: 'Hold a flat object (phone, box, panel) square-on — it measures size and checks the surface for dents.',
  boxscan: 'Hold a box in the frame and slowly rotate it to build up L×W×H + volume.',
  face: 'Look at the camera. New faces get a name box; returning faces are greeted.',
  cloud: 'Keep the camera still to map the room; step out of frame once for a clean map. People are slotted in live as they enter.',
  detect: 'Type any object; the AI finds it and tells you how far away it is.',
  about: ''
};
const PRESETS = ['person','laptop','bottle','cup','cell phone','keyboard','backpack','chair','book','scissors'];

/* per-mode value / ROI line — turns each demo into a buyer-facing use case */
const VALUES = {
  safety:    'Virtual safety zone — people-aware machine guarding with configurable danger/caution distances. A software alternative to fixed light-curtains.',
  dimension: 'Non-contact depth metrology — size plus surface QC (flatness & dents in mm), no jig. Inline inspection on the line.',
  boxscan:   'Automatic parcel dimensioning — L×W×H + volume from a rotation scan. Feeds WMS, shipping-rate & cartonization.',
  face:      'On-device recognition with depth anti-spoofing. Encrypted vectors, no photos, auto-wipe — private by design.',
  cloud:     'Live 3D reconstruction — persistent room map with people slotted in. The basis for AMR/robot navigation & digital-twin capture.',
  detect:    'Open-vocabulary detection — find any object by text, zero retraining. Flexible sorting, counting & visual search.',
  assistant: '', about: ''
};

/* ---- tab switching ---- */
document.querySelectorAll('.tab').forEach(t => {
  t.onclick = () => setMode(t.dataset.mode, t);
});

function setMode(m, tabEl) {
  mode = m;
  document.querySelectorAll('.tab').forEach(x => x.classList.remove('on'));
  if (tabEl) tabEl.classList.add('on');

  if (m === 'about') { openAbout(); return; }           // overlay; keep prior view under it

  // Product AI is a UI-only panel (links out to the assistant); no camera needed.
  assistantPanel.style.display = (m === 'assistant') ? 'block' : 'none';
  if (m === 'assistant') {
    stage.classList.remove('split');
    video.style.display = 'none'; cloudCanvas.style.display = 'none';
    detectbar.style.display = 'none'; hint.textContent = ''; stopCloud();
    valcap.textContent = ''; perfbadge.style.display = 'none';
    fetch('/mode', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
                    body:'mode=safety'});   // keep the GPU light while nobody's watching the feed
    return;
  }

  fetch('/mode', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
                  body:'mode='+encodeURIComponent(m)});

  if (m === 'boxscan') {   // clicking the tab (re)starts a fresh scan
    fetch('/boxreset', {method:'POST'});
  }
  const is3D = (m === 'cloud');
  stage.classList.toggle('split', is3D);          // 3D scan = point cloud | depth heatmap
  cloudCanvas.style.display = is3D ? 'block' : 'none';
  video.style.display = 'block';                   // shown in every camera mode (right panel in 3D)
  detectbar.style.display = (m === 'detect') ? 'flex' : 'none';
  hint.textContent = HINTS[m] || '';
  // value/ROI caption (hidden in 3D split so it doesn't clash with the split labels)
  valcap.textContent = is3D ? '' : (VALUES[m] || '');
  perfbadge.style.display = '';
  if (is3D) startCloud(); else stopCloud();
}

/* ---- attract / idle mode (draw people in when the booth is empty) ---- */
const ATTRACT_IDLE = 25;                       // s with nobody present AND no touch
const ATTRACT_DWELL = 9000;                    // ms per demo while attracting
/* only cycle tabs that actually exist (AI Detect is removed on the RealSense-only Nano) */
const ATTRACT_TABS = ['boxscan', 'cloud', 'detect']
  .filter(m => document.querySelector('.tab[data-mode="' + m + '"]'));
let attract = false, lastInteract = Date.now(), attractTimer = null, attractIdx = 0;

function cycleAttract() {
  const m = ATTRACT_TABS[attractIdx++ % ATTRACT_TABS.length];
  setMode(m, document.querySelector('.tab[data-mode="' + m + '"]'));
}
function enterAttract() {
  if (attract) return;
  attract = true; attractIdx = 0;
  attractEl.classList.add('show');
  cycleAttract();                                        // begin on a demo immediately
  attractTimer = setInterval(cycleAttract, ATTRACT_DWELL);
}
function exitAttract(toMode) {
  if (!attract) return;
  attract = false;
  clearInterval(attractTimer); attractTimer = null;
  attractEl.classList.remove('show');
  if (toMode) setMode(toMode, document.querySelector('.tab[data-mode="' + toMode + '"]'));
}
// any intentional interaction ends attract at once and resets the idle timer
['pointerdown', 'keydown', 'wheel', 'touchstart'].forEach(ev =>
  window.addEventListener(ev, () => { lastInteract = Date.now(); exitAttract(); }, {passive: true}));
// a backgrounded/hidden tab must never drive the shared camera mode
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState !== 'visible') exitAttract();
});

/* ---- status poll (safety redirect + face enroll prompt + notices) ---- */
let redirectCooldown = 0;
async function poll() {
  try {
    const s = await (await fetch('/status')).json();
    // live on-device performance badge
    if (s.cam_ok === false) {
      perfbadge.textContent = '● camera offline'; perfbadge.classList.add('off');
    } else if (s.fps != null) {
      perfbadge.classList.remove('off');
      perfbadge.textContent = '⚡ ' + Math.round(s.fps) + ' fps · ' +
        Math.round(s.latency_ms || 0) + ' ms · on-device';
    }
    // live visitor counter (people + dwell analytics) — hidden until the first visit
    if (s.visitors) {
      const avg = Math.round(s.dwell_avg_s || 0);
      statEl.textContent = '👥 ' + s.visitors + ' visitor' + (s.visitors === 1 ? '' : 's') +
        ' today' + (avg ? ' · avg dwell ' + avg + 's' : '');
    } else statEl.textContent = '';
    // Keep the VISIBLE tab authoritative. The camera stream is global (one server
    // mode); if the server drifted to another mode — a background tab's attract
    // cycle, or a second open window — re-assert this tab's mode so the burned-in
    // overlay always matches the selected tab (fixes Greet showing the depth map,
    // 3D Scan showing Box Vol, "AI detect running" in the background, etc.).
    const visible = document.visibilityState === 'visible';
    if (visible && !attract && mode !== 'about' && mode !== 'assistant' && s.mode && s.mode !== mode) {
      fetch('/mode', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
                      body:'mode=' + encodeURIComponent(mode)});
    }
    // attract mode: engage ONLY on the visible tab, when nobody's present + untouched
    const idle = s.idle_s || 0, sinceTouch = (Date.now() - lastInteract) / 1000;
    if (!attract) {
      if (visible && idle > ATTRACT_IDLE && sinceTouch > ATTRACT_IDLE && mode !== 'about' && !enrollOpen)
        enterAttract();
    } else if (s.present || !visible) {
      exitAttract(s.present ? 'safety' : undefined);   // stepped up, or tab hidden -> stop cycling
    }
    // Safety zone: a person within ~40cm auto-switches to Greet.
    if (mode === 'safety' && s.redirect === 'greet' && Date.now() > redirectCooldown) {
      redirectCooldown = Date.now() + 9000;
      setMode('face', document.querySelector('.tab[data-mode="face"]'));
      return;
    }
    // enroll modal
    const em = document.getElementById('enroll');
    if (mode === 'face' && s.pending && !enrollOpen && Date.now() > skipUntil) {
      enrollOpen = true; em.style.display = 'flex'; document.getElementById('name').focus();
    } else if ((!s.pending || mode !== 'face') && enrollOpen) {
      enrollOpen = false; em.style.display = 'none';
    }
    // small live notice
    if (mode === 'face') notice.textContent = (s.captured ? 'Scanning ' + s.captured + '/' + s.views + ' angles · ' : '') +
        'Known people: ' + (s.known ?? 0) + ' · encrypted locally · auto-wipes end of day';
    else if (mode === 'safety') notice.textContent = s.person ?
        ('Person at ' + (s.dist ? (s.dist*100).toFixed(0) + ' cm' : '—')) : '';
    else if (mode === 'detect' && s.dets) notice.textContent =
        s.dets.length + ' object(s) found' + (s.dets[0] && s.dets[0].dist ?
        ' · nearest ' + s.dets[0].dist.toFixed(2) + ' m' : '');
    else if (mode === 'cloud') notice.textContent =
        'Room map ' + (s.map_cov ?? 0) + '% · ' + (s.points ?? 0).toLocaleString() + ' pts' +
        (s.people ? ' · person slotted in' : '');
    else if (mode === 'dimension') notice.textContent = s.measuring ?
        ((s.object ? s.object + ' · ' : '') + (s.w_cm != null ? s.w_cm + '×' + s.h_cm + ' cm · ' : '') +
         'flatness ' + s.flatness_mm + ' mm · ' + s.verdict) : '';
    else if (mode === 'boxscan') notice.textContent = s.locked ?
        ('Volume ' + s.volume_l + ' L  (' + s.L_cm + '×' + s.W_cm + '×' + s.H_cm + ' cm)' +
         (s.hold_left != null ? ' · resetting in ' + Math.round(s.hold_left) + 's' : '')) :
        ('Rotate the box · dimensions found ' + (s.distinct ?? 0) + '/3 · faces ' + (s.faces ?? 0));
    else notice.textContent = '';
  } catch (e) {}
}
setInterval(poll, 500);

/* ---- face enroll ---- */
async function save() {
  const n = document.getElementById('name').value.trim(); if (!n) return;
  const co = document.getElementById('company').value.trim();
  const em = document.getElementById('email').value.trim();
  const ph = document.getElementById('phone').value.trim();
  await fetch('/enroll', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'name='+encodeURIComponent(n)+'&company='+encodeURIComponent(co)+'&email='+encodeURIComponent(em)+'&phone='+encodeURIComponent(ph)});
  document.getElementById('name').value=''; document.getElementById('company').value='';
  document.getElementById('email').value=''; document.getElementById('phone').value=''; enrollOpen=false;
  document.getElementById('enroll').style.display='none';
}
function skip() { skipUntil = Date.now()+15000; enrollOpen=false;
  document.getElementById('enroll').style.display='none'; fetch('/skip',{method:'POST'}); }
async function forget() { if (confirm('Delete the face-print of the person on screen?'))
  await fetch('/forget',{method:'POST'}); }
async function wipe() { if (confirm('Erase ALL stored face-prints?')) await fetch('/wipe',{method:'POST'}); }

/* ---- detect query ---- */
const chips = document.getElementById('chips');
PRESETS.forEach(p => { const c=document.createElement('span'); c.className='chip'; c.textContent=p;
  c.onclick=()=>{ const q=document.getElementById('q'); q.value=(q.value?q.value+', ':'')+p; }; chips.appendChild(c); });
function setQuery() {
  const t = document.getElementById('q').value.trim(); if (!t) return;
  fetch('/query', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
                   body:'text='+encodeURIComponent(t)});
}
document.getElementById('q').addEventListener('keydown', e => { if (e.key==='Enter') setQuery(); });

/* ---- about panel + live Jetson stats ---- */
let sysTimer = null;
async function pollSys() {
  try {
    const s = await (await fetch('/sysstats')).json();
    const set = (id, t) => { const e = document.getElementById(id); if (e) e.textContent = t; };
    const bar = (id, pct) => { const e = document.getElementById(id);
      if (e) e.style.width = Math.max(0, Math.min(100, pct || 0)) + '%'; };
    set('sysGpu', s.gpu_pct != null ? s.gpu_pct + '%' : '—'); bar('barGpu', s.gpu_pct);
    set('sysRam', s.ram_used_gb != null ? s.ram_used_gb + ' / ' + s.ram_total_gb + ' GB' : '—');
    bar('barRam', s.ram_pct);
    const t = s.temp_c;
    set('sysTemp', t != null ? t + ' °C' : '—'); bar('barTemp', t);   // 0–100 °C scale
    const bt = document.getElementById('barTemp');
    if (bt) bt.style.background = t >= 85 ? '#ff5c5c' : (t >= 78 ? '#ffb020' : 'var(--nv)');
    set('sysStack', s.platform + ' · CUDA ' + s.cuda + ' · TensorRT ' + s.trt +
        ' · Vision: ' + s.vision + ' · LLM: ' + s.llm);
    const tw = document.getElementById('thermwarn');
    if (tw) {
      if (t >= 85) { tw.textContent = '⚠ Thermal throttling likely (' + t + ' °C) — improve airflow / lower fan curve'; tw.className = 'thermwarn hot'; }
      else if (t >= 78) { tw.textContent = '⚠ Running warm (' + t + ' °C) — keep the vents clear'; tw.className = 'thermwarn warm'; }
      else { tw.textContent = ''; tw.className = 'thermwarn'; }
    }
  } catch (e) {}
}
function openAbout(){ document.getElementById('about').style.display='flex';
  pollSys(); clearInterval(sysTimer); sysTimer = setInterval(pollSys, 1500); }
function closeAbout(){ document.getElementById('about').style.display='none';
  clearInterval(sysTimer); sysTimer = null;
  document.querySelector('.tab[data-mode="'+mode+'"]').classList.add('on'); }

/* ---- 3D point cloud (three.js) ---- */
let renderer, scene, camera, controls, points, cloudTimer, rafId, cloudInited=false, framed=false, lastW=0, lastH=0;
function initCloud() {
  renderer = new THREE.WebGLRenderer({canvas:cloudCanvas, antialias:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  scene = new THREE.Scene(); scene.background = new THREE.Color(0x0a0b0e);
  camera = new THREE.PerspectiveCamera(55, 1.6, 0.01, 100);
  camera.position.set(0, 0, 1.6);
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  controls.autoRotate = true; controls.autoRotateSpeed = 0.7;   // gentle turntable
  controls.target.set(0, 0, 0);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3), 3));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3), 3));
  points = new THREE.Points(geo, new THREE.PointsMaterial({size:0.013, vertexColors:true, sizeAttenuation:true}));
  scene.add(points);
  cloudInited = true;
}
function resizeCloud() {   // runs every frame; only acts when the canvas size changed
  const w = cloudCanvas.clientWidth, h = cloudCanvas.clientHeight;
  if (!w || !h || (w === lastW && h === lastH)) return;
  lastW = w; lastH = h;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
  framed = false;          // re-centre once the viewport is correctly sized
}
function frameCloud(g) {   // point the camera at the cloud's centre and fit it
  if (framed || !g.boundingSphere || !lastW) return;
  const s = g.boundingSphere, r = s.radius || 1;
  controls.target.copy(s.center);
  camera.near = Math.max(0.01, r * 0.05); camera.far = r * 40;
  camera.position.set(s.center.x, s.center.y, s.center.z + r * 2.6);
  camera.updateProjectionMatrix(); controls.update();
  framed = true;
}
async function fetchCloud() {
  try {
    const buf = await (await fetch('/pointcloud')).arrayBuffer();
    if (buf.byteLength < 4) return;
    const n = new Uint32Array(buf, 0, 1)[0];
    if (!n) return;
    const pos = new Float32Array(buf, 4, n*3);
    const rgb = new Uint8Array(buf, 4 + n*12, n*3);
    const col = new Float32Array(n*3);
    for (let i=0;i<n*3;i++) col[i] = rgb[i]/255;
    const g = points.geometry;
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true;
    g.computeBoundingSphere();
    frameCloud(g);
  } catch (e) {}
}
function animate() {
  if (mode !== 'cloud') return;
  resizeCloud();
  controls.update();
  renderer.render(scene, camera);
  rafId = requestAnimationFrame(animate);
}
function startCloud() {
  if (!cloudInited) initCloud();
  fetch('/cloudreset', {method:'POST'});  // fresh accumulation each time the tab opens
  lastW = 0; lastH = 0; framed = false;   // force a fresh size + centre on open
  fetchCloud();
  cloudTimer = setInterval(fetchCloud, 180);  // bigger accumulated cloud -> gentler refresh
  cancelAnimationFrame(rafId); animate();
}
function stopCloud() { clearInterval(cloudTimer); cancelAnimationFrame(rafId); }

/* start — open the deep-linked demo (/?tab=<demo>, used by the printed QR codes
   next to each demo), else default to Safety; either way sync the server. */
const VALID_TABS = ['safety','dimension','boxscan','face','cloud','detect','assistant','about'];
const wantTab = new URLSearchParams(location.search).get('tab');
if (wantTab && VALID_TABS.includes(wantTab)) {
  setMode(wantTab, document.querySelector('.tab[data-mode="' + wantTab + '"]'));
} else {
  hint.textContent = HINTS[mode];
  fetch('/mode', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
                  body:'mode=' + encodeURIComponent(mode)});
}
