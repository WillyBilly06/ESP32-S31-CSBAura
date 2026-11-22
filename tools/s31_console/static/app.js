'use strict';
/* S31 Console UI — vanilla JS, no build step, works offline. */
const $ = (s) => document.querySelector(s);
const store = {
  get(k, d) { try { const v = localStorage.getItem('s31.' + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('s31.' + k, JSON.stringify(v)); } catch { /* ignore */ } },
};
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const pad2 = (n) => String(n).padStart(2, '0');
const clock = (t) => { const d = new Date(t * 1000); return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`; };
const clockMs = (t) => { const d = new Date(t * 1000); return `${clock(t)}.${String(d.getMilliseconds()).padStart(3, '0').slice(0, 2)}`; };
const fmtN = (n) => (n == null ? '—' : Number(n).toLocaleString());

let ST = null;              // last status from server
let clockOffset = 0;        // server_time - client_time (s)
let serverId = null;
let lastSeq = 0;
const latest = {};
const updated = {};
const series = {};
const SERIES_KEEP_S = 900;
let lastCrashCount = 0;
const CRASH_RE = /Guru Meditation Error|assert failed|task_wdt: Task watchdog|abort\(\) was called|Rebooting\.\.\.|rst:0x|CORRUPT HEAP|Stack smashing/;
const srvNow = () => Date.now() / 1000 + clockOffset;

/* ================= theme / sound / notifications ================= */
function applyTheme(t) {
  document.documentElement.dataset.theme = t;
  store.set('theme', t);
  $('#btnTheme use').setAttribute('href', t === 'dark' ? '#i-sun' : '#i-moon');
  readColors(); drawCharts();
}
let soundOn = store.get('sound', true);
let notifOn = store.get('notif', true);
let actx = null;
function ensureAudio() {
  if (!actx) { try { actx = new (window.AudioContext || window.webkitAudioContext)(); } catch { /* none */ } }
  if (actx && actx.state === 'suspended') actx.resume();
}
const SOUNDS = {
  ok: [[523, 0, 0.12], [659, 0.12, 0.12], [784, 0.24, 0.25]],
  fail: [[311, 0, 0.22, 'sawtooth'], [233, 0.25, 0.45, 'sawtooth']],
  log: [[880, 0, 0.09], [1175, 0.1, 0.14]],
  crash: [[196, 0, 0.16, 'square'], [196, 0.22, 0.16, 'square'], [147, 0.44, 0.4, 'square']],
};
function play(name) {
  if (!soundOn || !SOUNDS[name]) return;
  ensureAudio(); if (!actx) return;
  const t0 = actx.currentTime + 0.02;
  for (const [f, dt, dur, type = 'sine'] of SOUNDS[name]) {
    const o = actx.createOscillator(), g = actx.createGain();
    o.type = type; o.frequency.value = f;
    const peak = type === 'sine' ? 0.25 : 0.07;
    g.gain.setValueAtTime(0.0001, t0 + dt);
    g.gain.exponentialRampToValueAtTime(peak, t0 + dt + 0.015);
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + dt + dur);
    o.connect(g).connect(actx.destination); o.start(t0 + dt); o.stop(t0 + dt + dur + 0.05);
  }
}
function renderPrefs() {
  $('#btnSound use').setAttribute('href', soundOn ? '#i-vol' : '#i-mute');
  $('#btnSound').classList.toggle('on', soundOn);
  const perm = 'Notification' in window ? Notification.permission : 'denied';
  $('#btnNotif').classList.toggle('on', notifOn && perm === 'granted');
  $('#btnNotif').title = perm === 'granted' ? `Desktop notifications ${notifOn ? 'on' : 'off'} (when this tab is not focused)`
    : perm === 'denied' ? 'Desktop notifications blocked by the browser' : 'Click to enable desktop notifications';
}
function askNotifOnce() {
  if (!('Notification' in window) || Notification.permission !== 'default' || store.get('notifAsked', false)) return;
  store.set('notifAsked', true);
  Notification.requestPermission().then(renderPrefs);
}
function notify(title, body) {
  if (!notifOn || !('Notification' in window) || Notification.permission !== 'granted') return;
  if (!document.hidden && document.hasFocus()) return; // the toast is enough when you're looking
  try { new Notification(title, { body, tag: 's31-' + title, silent: true }); } catch { /* ignore */ }
}

/* ================= toasts ================= */
function toast({ level = 'info', title, text = '', timeout }) {
  const icon = level === 'ok' ? 'i-check' : (level === 'error' || level === 'crash' || level === 'warn') ? 'i-alert' : 'i-info';
  const el = document.createElement('div');
  el.className = `toast ${level}`;
  el.innerHTML = `<svg><use href="#${icon}"/></svg><div><div class="tt">${esc(title)}</div>${text ? `<div class="tx">${esc(text)}</div>` : ''}</div>`;
  const kill = () => { el.classList.add('out'); setTimeout(() => el.remove(), 300); };
  el.onclick = kill;
  const box = $('#toasts');
  box.prepend(el);
  while (box.children.length > 6) box.lastChild.remove();
  setTimeout(kill, timeout || (level === 'crash' ? 15000 : level === 'error' ? 10000 : 6000));
}

/* ================= API ================= */
async function api(path, body) {
  try {
    const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) toast({ level: r.status === 409 ? 'warn' : 'error', title: r.status === 409 ? 'Busy — request rejected' : 'Request failed', text: j.message || r.statusText });
    return j;
  } catch (e) {
    toast({ level: 'error', title: 'Server unreachable', text: String(e) });
    return {};
  }
}

/* ================= status header ================= */
let stepPlan = null;   // {seq, op, steps:[...]}
function planFor(st) {
  const logStep = st.auto_log || st.logging;
  if (st.op === 'build') return ['Build'];
  if (st.op === 'reset') return ['Reset', st.logging ? 'Logging' : null].filter(Boolean);
  const s = ['Build', 'Flash', 'Reboot'];
  if (logStep) s.push('Logging');
  return s;
}
function renderSteps(st) {
  if (st.op && (!stepPlan || stepPlan.seq !== st.op_seq)) stepPlan = { seq: st.op_seq, op: st.op, steps: planFor(st), skipBuild: st.op === 'flash' };
  const box = $('#steps');
  if (!stepPlan) { box.innerHTML = ''; return; }
  const cur = { BUILDING: 'Build', FLASHING: 'Flash', REBOOTING: stepPlan.op === 'reset' ? 'Reset' : 'Reboot' }[st.state];
  const lr = st.last_result && st.last_result.op_seq === stepPlan.seq ? st.last_result : null;
  const running = st.op && st.op_seq === stepPlan.seq;
  let reachedCur = false;
  const failStep = lr && !lr.ok ? (lr.op === 'build' ? 'Build' : lr.op === 'reset' ? 'Reset' : 'Flash') : null;
  box.innerHTML = stepPlan.steps.map((name) => {
    let cls = '';
    if (name === 'Build' && stepPlan.skipBuild) cls = 'skip';
    else if (failStep) {
      const order = stepPlan.steps.indexOf(name), fo = stepPlan.steps.indexOf(failStep);
      cls = order < fo ? 'done' : order === fo ? 'fail' : 'skip';
    } else if (running) {
      if (name === cur) { cls = 'active'; reachedCur = true; } else cls = reachedCur ? '' : 'done';
      if (!cur) cls = '';
    } else cls = (name === 'Logging' && !st.logging) ? 'skip' : 'done';
    return `<span class="step ${cls}">${name}</span>`;
  }).join('<span class="muted">›</span>');
}

const STATE_COLORS = { IDLE: '#6b7686', BUILDING: '#4f8cff', FLASHING: '#ff8a3d', REBOOTING: '#a77bff', LOGGING: '#2fbf71', ERROR: '#f2545b' };
function setFavicon(state) {
  const c = encodeURIComponent(STATE_COLORS[state] || '#6b7686');
  $('#favicon').href = `data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Crect rx='7' width='32' height='32' fill='${c}'/%3E%3Cpath d='M18 4 8 18h7l-2 10 11-15h-7z' fill='white'/%3E%3C/svg%3E`;
}

function renderStatus(st) {
  const hero = $('#hero');
  hero.dataset.state = st.state;
  $('#pillText').textContent = st.state;
  $('#pill').title = st.state === 'ERROR' ? 'Click to dismiss the error state' : 'Current state';
  $('#devSub').textContent = `ESP32-S31 · ${st.port} · ${st.baud} baud${st.mock ? '' : ''}`;
  $('#mockBadge').hidden = !st.mock;

  // phase + progress
  const prog = $('#progress'), fill = $('#progFill'), lbl = $('#progLabel');
  prog.className = 'progress';
  let phase = st.phase || '';
  if (st.state === 'BUILDING' || st.state === 'FLASHING') {
    prog.classList.add('active');
    const p = st.progress == null ? 0 : st.progress;
    fill.style.width = p + '%';
    lbl.textContent = `${st.state === 'FLASHING' ? 'Flashing' : 'Building'} ${p.toFixed(0)}%`;
  } else if (st.state === 'REBOOTING') {
    prog.classList.add('indet'); fill.style.width = '30%'; lbl.textContent = 'Rebooting…';
  } else {
    const lr = st.last_result;
    const recent = lr && srvNow() - lr.at < 20;
    if (recent) { prog.classList.add(lr.ok ? 'ok' : 'fail'); fill.style.width = '100%'; lbl.textContent = lr.ok ? `${lr.text} ✓` : lr.text; }
    else { fill.style.width = '0'; lbl.textContent = ''; }
    phase = st.state === 'LOGGING' ? 'Monitoring serial output' : st.state === 'ERROR' ? (lr ? `${lr.text}: ${lr.detail || ''}` : 'Error') : 'Ready';
  }
  $('#phase').textContent = phase;
  renderSteps(st);

  // last result
  const lr = st.last_result, last = $('#lastResult');
  if (lr) {
    last.className = 'last ' + (lr.ok ? 'ok' : 'fail');
    last.innerHTML = `<svg><use href="#${lr.ok ? 'i-check' : 'i-alert'}"/></svg><b>${esc(lr.text)}</b> at ${clock(lr.at)} — ${lr.duration.toFixed(1)} s${lr.detail ? ` · <span>${esc(lr.detail)}</span>` : ''}`;
  }
  renderLogInd(st);

  // buttons
  const busy = !!st.op;
  for (const id of ['#btnBuild', '#btnFlash', '#btnBuildFlash', '#btnReset']) $(id).disabled = busy;
  $('#btnCancel').hidden = !st.cancellable;
  $('#btnLog').classList.toggle('on', st.logging);
  $('#btnLogText').textContent = st.logging ? 'Stop logging' : 'Start logging';
  $('#btnLog use').setAttribute('href', st.logging ? '#i-stop' : '#i-rec');
  $('#tgAutoLog').checked = st.auto_log;

  // crash tile
  latest.__crashes = st.crash_count; latest.__last_crash = st.last_crash;

  document.title = `${st.state}${(st.state === 'FLASHING' || st.state === 'BUILDING') && st.progress != null ? ' ' + st.progress.toFixed(0) + '%' : ''} · S31 Console`;
  setFavicon(st.state);
  tickTimer();
}

function renderLogInd(st) {
  const el = $('#logInd'), tx = $('#logText');
  el.className = 'logind';
  if (st.logging && st.serial_paused) {
    el.classList.add('paused');
    tx.textContent = `Logging paused — ${st.port} released for ${st.serial_paused}, resumes automatically`;
  } else if (st.logging && st.port_error) {
    el.classList.add('waiting');
    tx.textContent = `Waiting for ${st.port}: ${st.port_error}`;
  } else if (st.logging && !st.port_open) {
    el.classList.add('waiting');
    tx.textContent = `Opening ${st.port}…`;
  } else if (st.logging) {
    el.classList.add('on');
    tx.textContent = `REC · Logging to ${st.session_file} · ${st.lines_per_s.toFixed(1)} lines/s · ${fmtN(st.session_lines)} lines`;
  } else {
    tx.textContent = st.session_file ? `Not logging · last session ${st.session_file}` : 'Not logging';
  }
}

function tickTimer() {
  const st = ST; if (!st) return;
  const lab = $('#timerLabel'), val = $('#timerVal');
  if (st.op && st.op_started) {
    const e = Math.max(0, srvNow() - st.op_started);
    lab.textContent = 'Elapsed'; val.textContent = `${Math.floor(e / 60)}:${pad2(Math.floor(e % 60))}.${Math.floor((e * 10) % 10)}`;
  } else if (st.logging && st.session_started) {
    const e = Math.max(0, srvNow() - st.session_started);
    lab.textContent = 'Session'; val.textContent = `${Math.floor(e / 3600)}:${pad2(Math.floor(e / 60) % 60)}:${pad2(Math.floor(e % 60))}`;
  } else if (st.last_result) {
    lab.textContent = 'Last op'; val.textContent = `${st.last_result.duration.toFixed(1)} s`;
  } else { lab.textContent = 'Elapsed'; val.textContent = '—'; }
}

function onStatus(st, initial = false) {
  const prev = ST;
  clockOffset = st.now - Date.now() / 1000;
  ST = st;
  renderStatus(st);
  if (initial || !prev) return;
  // client-side transition effects
  if (st.state === 'FLASHING' && prev.state !== 'FLASHING') {
    toast({ level: 'info', title: 'Flashing started', text: `${st.port} @ ${st.flash_baud}${prev.logging ? ' — logging paused' : ''}` });
  } else if (st.state === 'BUILDING' && prev.state !== 'BUILDING') {
    toast({ level: 'info', title: st.op === 'build+flash' ? 'Build + Flash started' : 'Build started', text: 'idf.py build' });
  }
  const lr = st.last_result, plr = prev.last_result;
  if (lr && (!plr || lr.at !== plr.at) && lr.op !== 'reset') {
    const hero = $('#hero');
    hero.classList.remove('celebrate-ok', 'celebrate-fail'); void hero.offsetWidth;
    hero.classList.add(lr.ok ? 'celebrate-ok' : 'celebrate-fail');
  }
  if (st.logging && st.port_open && !(prev.logging && prev.port_open)) {
    const li = $('#logInd'); li.classList.remove('pop'); void li.offsetWidth; li.classList.add('pop');
  }
}

/* ================= metric tiles ================= */
/* Fed by StatsParser in server.py from the s31-btstack-speaker 2 s status lines
 * (A2DP / AURA / PARTY / JBL_* / V5_EQ / S31 HEAP) plus event lines.  "<key>_d" =
 * increase of a firmware total since the previous 2 s line. */
const L = (k) => latest[k];
const has = (k) => latest[k] !== undefined && latest[k] !== null;
const n0 = (k) => (has(k) ? fmtN(latest[k]) : '—');
const dlt = (k) => (has(k + '_d') && latest[k + '_d'] > 0 ? ` (+${fmtN(latest[k + '_d'])})` : '');
const at = (k) => (has(k) ? clock(latest[k]) : '');
const join = (...p) => p.filter((x) => x !== '' && x != null && x !== false).join(' · ');
const lvl3 = (v, warn, err) => (v == null ? '' : v >= err ? 'err' : v >= warn ? 'warn' : 'ok');
const worst = (...c) => (c.includes('err') ? 'err' : c.includes('warn') ? 'warn' : c.includes('ok') ? 'ok' : c.includes('info') ? 'info' : '');
const streaming = () => L('a2dp_state') === 'streaming';
const ROLE_CLS = { HOST: 'ok', LISTEN: 'info', HOST_STOPPING: 'warn', LISTEN_STOPPING: 'warn' };

const TILES = [
  // ---- phone -> S31 (A2DP) ----
  { g: 'Phone · A2DP', id: 'a2dp', label: 'A2DP', val: () => L('a2dp_state'), unit: () => '',
    cls: (v) => ({ streaming: 'ok', connected: 'info', paused: 'warn' }[v] || ''),
    sub: () => (L('a2dp_state') === 'no phone'
      ? join(L('a2dp_disc_reason'), L('a2dp_event') ? `${L('a2dp_event')} ${at('a2dp_event_t')}` : '')
      : join(L('a2dp_peer'), L('bredr_role'), has('a2dp_since') ? 'since ' + clock(L('a2dp_since')) : '')), stale: 'a2dp_link' },
  { id: 'volume', label: 'Volume / AVRCP', val: () => L('volume'), unit: () => '/127',
    cls: () => (L('avrcp') === false ? 'warn' : 'info'),
    sub: () => {
      const v = L('volume'); const pct = has('volume_pct') ? L('volume_pct') : (v != null ? Math.round((v * 100) / 127) : null);
      if (L('avrcp') === false) return join('no AVRCP — phone volume not synced', L('avrcp_event') && L('avrcp_event') !== 'AVRCP disconnected' ? L('avrcp_event') : '');
      return join(has('avrcp') ? 'AVRCP ✓' : '', pct != null ? pct + '%' : '', has('volume_db') ? `${L('volume_db').toFixed(1)} dB` : '',
        has('volume_src') ? `← ${L('volume_src')} ${at('volume_t')}` : '');
    } },
  { id: 'latency', label: 'Latency (PCM buffer)', val: () => L('pcm_ms'), unit: () => 'ms',
    cls: (v) => (!streaming() ? '' : worst(v < 60 ? 'err' : v < 150 ? 'warn' : 'ok', L('catchups_d') > 0 ? 'warn' : '')),
    sub: () => join(has('catchups') ? `catch-ups ${n0('catchups')}${dlt('catchups')}` : '',
      has('catchup_ms') ? `${n0('catchup_ms')} ms skipped` : '', has('catchup_expired') ? `${n0('catchup_expired')} expired` : '',
      has('clock_ppm') ? `corr ${L('clock_ppm')} ppm` : '',
      has('delay_report_ms') ? `delay report ${L('delay_report_ms')} ms${L('delay_report_status') && L('delay_report_status') !== '0x00' ? ' → ' + L('delay_report_status') : ''}` : ''),
    stale: 'pcm_ms' },
  { id: 'underruns', label: 'A2DP underruns', val: () => L('a2dp_underruns_d'), unit: () => '/2 s',
    cls: (v) => worst(lvl3(v, 1, 3), (L('seq_gaps_d') || 0) > 0 ? 'warn' : '', (L('q_drops_d') || 0) > 0 ? 'warn' : ''),
    sub: () => join(`total ${n0('a2dp_underruns')}`, `seq gaps ${n0('seq_gaps')}${dlt('seq_gaps')}`,
      `q drops ${n0('q_drops')}${dlt('q_drops')}`, `oldest dropped ${n0('oldest_dropped')}${dlt('oldest_dropped')}`), stale: 'a2dp_underruns' },
  { id: 'bitpool', label: 'SBC bitpool', val: () => L('bitpool'), unit: () => '',
    cls: (v) => (!streaming() ? '' : v >= 51 ? 'ok' : v >= 35 ? 'warn' : 'err'),
    sub: () => join(has('bitpool_min') ? `min ${L('bitpool_min')} max ${L('bitpool_max')}` : '',
      has('media_pkts_2s') ? `+${L('media_pkts_2s')} pkts` : '', has('sbc_fpp') ? `${L('sbc_fpp')} fr/pkt` : '',
      has('pkt_max_b') ? `max ${L('pkt_max_b')} B` : '', L('sbc_config')), stale: 'bitpool' },
  { id: 'role', label: 'BR/EDR link', val: () => L('bredr_role') ?? (has('link_last') ? 'role ?' : null), unit: () => '',
    cls: (v) => (v === 'MASTER' ? 'ok' : v === 'SLAVE' ? 'err' : 'info'),
    sub: () => join(L('bredr_role') === 'SLAVE' ? 'phone schedules the S31 — bad' : '',
      has('link_poll_slots') ? `QoS poll ${L('link_poll_slots')} slots` : '', has('link_qos_latency_us') ? `latency ${fmtN(L('link_qos_latency_us'))} µs` : '',
      has('link_max_slots') ? `max slots ${L('link_max_slots')}` : '', has('link_phone_edr') ? `phone EDR ${L('link_phone_edr')}` : '') || (L('link_last') || '') },
  // ---- S31 -> speakers (Auracast) ----
  { g: 'Auracast broadcast', id: 'aura', label: 'Auracast', val: () => L('aura_state'), unit: () => '',
    cls: (v) => (v === 'ON' ? 'ok' : v === 'off' ? '' : 'warn'),
    sub: () => join(has('sdu_sent_l') ? `sent L/R ${L('sdu_sent_l')}/${L('sdu_sent_r')}` : '', has('sdu_done_l') ? `done ${L('sdu_done_l')}/${L('sdu_done_r')}` : '',
      has('aura_iso_us') ? `ISO ${L('aura_iso_us') / 1000} ms` : '', has('aura_since') ? 'since ' + clock(L('aura_since')) : ''), stale: 'aura_state' },
  { id: 'not_sent', label: 'Not sent / late', val: () => L('not_sent'), unit: () => '/2 s',
    cls: (v) => worst(lvl3(v, 1, 5), lvl3(L('late_silence'), 1, 5), (L('trimmed') || 0) > 0 ? 'warn' : ''),
    sub: () => join(`host ${n0('not_sent_host')} ctrl ${n0('not_sent_ctrl')}`, `late ${n0('late_silence')}`, `trimmed ${n0('trimmed')}`,
      has('rq_ready') ? `ready ${L('rq_ready')}/${L('rq_depth')}` : '', `in flight ${n0('in_flight')}`, `ctrl free ${n0('ctrl_free')}`), stale: 'not_sent' },
  { id: 'enc', label: 'LC3 encode avg', val: () => L('enc_avg'), unit: () => 'µs',
    cls: (v) => { const m = L('enc_max') || 0; return worst(v < 3000 && m < 6000 ? 'ok' : v < 6000 && m < 9000 ? 'warn' : 'err', (L('aura_underruns') || 0) + (L('aura_wdog') || 0) > 0 ? 'warn' : ''); },
    sub: () => join(`max ${n0('enc_max')} µs`, `${n0('enc_frames')} fr`, `PCM underruns ${n0('aura_underruns')}`, `wdog ${n0('aura_wdog')}`,
      `rq full ${n0('rq_full')}${dlt('rq_full')}`, has('enc_stack_free') ? `stack ${L('enc_stack_free')} B` : ''), stale: 'enc_avg' },
  { id: 'big', label: 'BIG timing (BIS0)', val: () => (has('big_gap_avg_ms') ? Number(L('big_gap_avg_ms')).toFixed(3) : null), unit: () => 'ms gap',
    cls: () => worst((L('big_long') || 0) > 0 ? 'warn' : 'ok', (L('partly_unsent_d') || 0) > 0 ? 'warn' : '', (L('big_ctl_late_d') || 0) > 0 ? 'warn' : ''),
    sub: () => join(`min ${n0('big_gap_min_us')} max ${n0('big_gap_max_us')} µs`, `>1.5 itvl ${n0('big_long')}`,
      has('big_sdus_per_ev') ? `${L('big_sdus_per_ev').toFixed(2)} SDU/ev` : '', `drift ${n0('ts_drift_us')} µs`,
      `re-anchors ${n0('reanchors')}`, `partly unsent ${n0('partly_unsent')}${dlt('partly_unsent')}`), stale: 'big_gap_avg_ms' },
  { id: 'lr', label: 'L/R align', val: () => L('lr_tx_offset_us'), unit: () => 'µs tx offset',
    cls: (v) => worst(Math.abs(v) === 0 ? 'ok' : Math.abs(v) < 1000 ? 'warn' : 'err', has('lr_seq0') && L('lr_seq0') !== L('lr_seq1') ? 'err' : ''),
    sub: () => join(`ref offset ${n0('lr_ref_offset_us')} µs`, has('lr_pkt_cnt') ? `pkt_cnt ${L('lr_pkt_cnt')}` : '',
      has('lr_seq0') ? (L('lr_seq0') === L('lr_seq1') ? 'seq L = R' : `seq L ${L('lr_seq0')} R ${L('lr_seq1')}`) : ''), stale: 'lr_tx_offset_us' },
  // ---- party mode / JBL ----
  { g: 'Party · JBL', id: 'party', label: 'Party role', wide: true, val: () => L('party_role'), unit: () => (has('party_listen') ? `listen ${L('party_listen')}` : ''),
    cls: (v) => ROLE_CLS[v] || 'info',
    sub: () => join(has('party_since') ? 'since ' + clock(L('party_since')) : '',
      has('party_to_listen') ? `switches →listen ${L('party_to_listen')}, →host ${L('party_to_host')}` : '', has('party_syncs') ? `syncs ${L('party_syncs')}` : '',
      L('party_event') ? `last: ${L('party_event')} ${at('party_event_t')}` : '') || 'no PARTY line yet', stale: 'party_role' },
  { id: 'listen', label: 'Party listen RX', val: () => L('listen_lost'), unit: () => 'lost /2 s',
    cls: (v) => worst(lvl3(v, 1, 5), (L('listen_errors_d') || 0) > 0 ? 'err' : '', (L('listen_i2s_late_d') || 0) > 0 ? 'warn' : ''),
    sub: () => (!has('listen_rx1') ? 'not listening' : join(L('listen_name') ? `'${L('listen_name')}'` : '',
      `BIS1 rx ${n0('listen_rx1')} lost ${n0('listen_lost1')}`, `BIS2 rx ${n0('listen_rx2')} lost ${n0('listen_lost2')}`,
      `PLC ${n0('listen_plc')}`, `errors ${n0('listen_errors')}`, `gaps ${n0('listen_gaps')}`, `q drops ${n0('listen_qdrops')}`, `I2S late ${n0('listen_i2s_late')}`)),
    stale: 'listen_rx1' },
  { id: 'assist', label: 'JBL speakers (assistant)', val: () => L('assist_state'), unit: () => '',
    cls: (v) => (v == null ? '' : /scanning|connect|probing|searching|joining/.test(v) ? 'warn' : 'info'),
    sub: () => join(`speakers ${n0('assist_speakers')}`, `joins ${n0('assist_joins_ok')}/${n0('assist_joins_sent')} ok`, `native ${n0('assist_native')}`,
      L('assist_speaker') ? `${L('assist_speaker')} ${at('assist_speaker_t')}` : '', L('assist_event') ? `last: ${L('assist_event')}` : ''), stale: 'assist_state' },
  { id: 'beacon', label: 'JBL beacon partyInfo', val: () => L('party_info'), unit: () => '',
    cls: () => { const s = L('beacon_state'); return s == null ? '' : s === 'on' ? 'ok' : s === 'OFF' ? 'err' : 'warn'; },
    sub: () => join(L('party_info_str'), L('beacon_state') !== 'on' ? L('beacon_state') : '', has('beacon_ms') ? `${L('beacon_ms')} ms` : '',
      has('beacon_scanners') ? `scanners ${L('beacon_scanners')}` : '', L('beacon_name_crc') ? '+phone name' : '',
      L('beacon_event') ? `last: ${L('beacon_event')}` : ''), stale: 'party_info' },
  { id: 'app', label: 'JBL app (GATT / V5)', val: () => L('v5_link'), unit: () => '',
    cls: (v) => (v === 'up' ? 'ok' : worst((L('gatt_dropped_d') || 0) > 0 ? 'warn' : '', (L('v5_tx_dropped_d') || 0) > 0 ? 'warn' : '')),
    sub: () => join(`GATT links ${n0('gatt_links')}`, `notify ${n0('gatt_notify')}`, `dropped ${n0('gatt_dropped')}`, `V5 rx ${n0('v5_rx')} tx ${n0('v5_tx')}`,
      has('eq_state') ? `EQ ${L('eq_state')} (${L('eq_bands')} bands)` : '', L('v5_event') || ''), stale: 'v5_link' },
  // ---- system ----
  { g: 'System', id: 'heap', label: 'Free internal RAM', val: () => L('free_internal_kb'), unit: () => 'KB',
    cls: (v) => worst(v > 60 ? 'ok' : v > 30 ? 'warn' : 'err', has('lowest_kb') && L('lowest_kb') < 12 ? 'warn' : ''),
    sub: () => join(has('largest_kb') ? `largest ${L('largest_kb')} KB` : '', has('lowest_kb') ? `lowest ${L('lowest_kb')} KB` : '',
      has('free_psram_kb') ? `PSRAM ${fmtN(L('free_psram_kb'))} KB` : ''), stale: 'free_internal_kb' },
  { id: 'radio', label: 'VHCI / ISO tx errors', val: () => (has('vhci_rx_drops') ? L('vhci_rx_drops') + (L('iso_tx_errors') || 0) : null), unit: () => 'total',
    cls: (v) => ((L('vhci_rx_drops_d') || 0) + (L('iso_tx_errors_d') || 0) > 0 ? 'err' : v > 0 ? 'warn' : 'ok'),
    sub: () => join(`VHCI rx drops ${n0('vhci_rx_drops')}${dlt('vhci_rx_drops')}`, `ISO tx errors ${n0('iso_tx_errors')}${dlt('iso_tx_errors')}`,
      has('iso_tx_last') && L('iso_tx_last') ? `last ${L('iso_tx_last')}` : ''), stale: 'vhci_rx_drops' },
  { id: 'crash', label: 'Crashes / unexpected reboots', val: () => (ST ? ST.crash_count : null), unit: () => '',
    cls: (v) => (v === 0 ? 'ok' : 'err'), sub: () => (ST && ST.last_crash ? `last ${clock(ST.last_crash.t)} · click to filter` : 'none this server run'), click: () => setFilter('/Guru Meditation|assert failed|task_wdt|Rebooting|rst:0x|Backtrace/') },
];
function buildTiles() {
  let html = '', open = false;
  for (const t of TILES) {
    if (t.g) { if (open) html += '</div>'; html += `<div class="section-title sub">${t.g}</div><div class="tiles">`; open = true; }
    html += `<div class="tile${t.click ? ' clickable' : ''}${t.wide ? ' wide' : ''}" id="tile-${t.id}"><div class="t-label">${t.label}</div><div class="t-val"><span class="v">—</span><span class="u"></span></div><div class="t-sub"></div></div>`;
  }
  $('#tiles').innerHTML = html + (open ? '</div>' : '');
  for (const t of TILES) if (t.click) $(`#tile-${t.id}`).onclick = t.click;
}
function renderTiles() {
  const now = srvNow();
  for (const t of TILES) {
    const el = $(`#tile-${t.id}`); if (!el) continue;
    const v = t.val();
    const hv = v !== undefined && v !== null;
    let c = '';
    try { c = hv ? t.cls(v) : ''; } catch { c = ''; }
    const stale = t.stale && (!updated[t.stale] || now - updated[t.stale] > 10) && hv;
    el.className = `tile${t.click ? ' clickable' : ''}${t.wide ? ' wide' : ''}${c ? ' s-' + c : ''}${stale ? ' stale' : ''}${el.classList.contains('flash') ? ' flash' : ''}`;
    el.querySelector('.v').textContent = hv ? (typeof v === 'number' ? fmtN(v) : String(v)) : '—';
    el.querySelector('.u').textContent = hv ? t.unit() : '';
    let s = '';
    try { s = t.sub() || ''; } catch { s = ''; }
    const se = el.querySelector('.t-sub');
    if (se.textContent !== s) { se.textContent = s; el.title = s; }
  }
  const tl = Math.max(updated.a2dp_link || 0, updated.aura_state || 0);
  $('#statsAge').textContent = tl ? `updated ${Math.max(0, Math.round(now - tl))} s ago` : 'waiting for 2 s status lines';
}

/* ================= charts (self-drawn canvas) ================= */
const CHARTS = [
  { id: 'c1', title: 'Auracast not sent / late (2 s)', keys: ['not_sent', 'late_silence'], thr: 5 },
  { id: 'c2', title: 'A2DP underruns / seq gaps (2 s)', keys: ['a2dp_underruns_d', 'seq_gaps_d'], thr: 1 },
  { id: 'c3', title: 'A2DP PCM buffer ms', keys: ['pcm_ms'], thr: 60 },
  { id: 'c4', title: 'LC3 encode µs (avg / max)', keys: ['enc_avg', 'enc_max'] },
  { id: 'c5', title: 'BIG gap max µs', keys: ['big_gap_max_us'] },
  { id: 'c6', title: 'Free internal RAM KB', keys: ['free_internal_kb'] },
];
const CHART_KEYS = new Set(CHARTS.flatMap((c) => c.keys));
let COL = {};
function readColors() {
  const cs = getComputedStyle(document.documentElement);
  const g = (n) => cs.getPropertyValue(n).trim();
  COL = { line: [g('--accent'), g('--violet')], grid: g('--border'), text: g('--dim'), thr: g('--err') };
}
function buildCharts() {
  $('#charts').innerHTML = CHARTS.map((c) => `<div class="chart"><div class="chart-head"><span>${c.title}</span><b id="${c.id}-v">—</b></div><canvas id="${c.id}"></canvas></div>`).join('');
}
function drawCharts() {
  const now = srvNow(), t0 = now - 300;
  for (const c of CHARTS) {
    const cv = document.getElementById(c.id); if (!cv) continue;
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    if (!w) continue;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
    const data = c.keys.map((k) => (series[k] || []).filter((p) => p[0] >= t0));
    let max = 0; for (const d of data) for (const p of d) max = Math.max(max, p[1]);
    if (c.thr) max = Math.max(max, c.thr * 1.25);
    max = max <= 0 ? 1 : max * 1.08;
    const L0 = 30, R = w - 4, T = 4, B = h - 12;
    const X = (t) => L0 + ((t - t0) / 300) * (R - L0), Y = (v) => B - (v / max) * (B - T);
    g.font = '10px ui-monospace, Consolas, monospace'; g.fillStyle = COL.text; g.strokeStyle = COL.grid; g.lineWidth = 1;
    for (const f of [0, 0.5, 1]) {
      const y = Math.round(Y(max * f)) + 0.5; g.beginPath(); g.moveTo(L0, y); g.lineTo(R, y); g.stroke();
      const v = max * f; g.fillText(v >= 1000 ? (v / 1000).toFixed(1) + 'k' : v.toFixed(v < 10 && f ? 1 : 0), 2, y + 3);
    }
    g.fillText('-5m', L0, h - 1); g.fillText('now', R - 20, h - 1);
    if (c.thr) { g.save(); g.setLineDash([4, 3]); g.strokeStyle = COL.thr; g.globalAlpha = 0.7; const y = Y(c.thr); g.beginPath(); g.moveTo(L0, y); g.lineTo(R, y); g.stroke(); g.restore(); }
    data.forEach((d, i) => {
      if (!d.length) return;
      const col = COL.line[i % COL.line.length];
      g.beginPath(); d.forEach((p, j) => (j ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))));
      g.strokeStyle = col; g.lineWidth = 1.6; g.stroke();
      if (i === 0) { g.lineTo(X(d[d.length - 1][0]), B); g.lineTo(X(d[0][0]), B); g.closePath(); g.globalAlpha = 0.14; g.fillStyle = col; g.fill(); g.globalAlpha = 1; }
      const lp = d[d.length - 1]; g.beginPath(); g.arc(X(lp[0]), Y(lp[1]), 2.5, 0, 7); g.fillStyle = col; g.fill();
    });
    const lastVals = c.keys.map((k) => { const s = series[k]; return s && s.length ? fmtN(s[s.length - 1][1]) : '—'; });
    $(`#${c.id}-v`).textContent = lastVals.join(' / ');
  }
}
let chartQueued = false;
function queueCharts() { if (!chartQueued) { chartQueued = true; requestAnimationFrame(() => { chartQueued = false; drawCharts(); }); } }

function onStats(d) {
  for (const [k, v] of Object.entries(d.v)) {
    latest[k] = v; updated[k] = d.t;
    if ((k in series || CHART_KEYS.has(k)) && typeof v === 'number') {
      (series[k] = series[k] || []).push([d.t, v]);
      const cut = d.t - SERIES_KEEP_S; while (series[k].length && series[k][0][0] < cut) series[k].shift();
    }
  }
  renderTiles(); queueCharts();
}

/* ================= log pane ================= */
const logEl = $('#log');
const MAX_LINES = 5000;
let atBottom = true, newWhilePaused = 0;
let pendingLines = [], logQueued = false;
let fText = '', fRe = null;
const fLevels = Object.assign({ E: true, W: true, I: true, O: true, T: true }, store.get('levels', {}));

function hiLine(text) {
  let h = esc(text);
  const hot = (m) => `<span class="hot">${m}</span>`, warm = (m) => `<span class="warm">${m}</span>`;
  // AURA 2s (per-2 s counts)
  h = h.replace(/not sent: host (\d+) ctrl (\d+)/, (m, a, b) => (+a + +b >= 5 ? hot(m) : +a + +b > 0 ? warm(m) : m));
  h = h.replace(/late \(silence\) (\d+)/, (m, n) => (+n > 0 ? hot(m) : m));
  h = h.replace(/PCM underruns (\d+)/, (m, n) => (+n > 0 ? hot(m) : m));
  h = h.replace(/&gt;1\.5 itvl (\d+)/, (m, n) => (+n > 0 ? warm(m) : m));
  // PARTY LISTEN 2s
  h = h.replace(/ lost (\d+)/g, (m, n) => (+n > 0 ? warm(m) : m));
  h = h.replace(/\(no AVRCP\)|role SLAVE|now SLAVE|ESP_FAIL/, (m) => warm(m));
  h = h.replace(/A2DP disconnected|ACL disconnected, reason 0x[0-9a-f]+/, (m) => hot(m));
  return h;
}
function lineEl(e) {
  const d = document.createElement('div');
  let cls = `ln k-${e.k}${e.l ? ' l-' + e.l : ''}`;
  if (e.k === 'm') {
    if (/FAILED|lost/.test(e.x)) cls += ' mk-err'; else if (/finished \(ok/.test(e.x)) cls += ' mk-ok';
    else if (/flash started|pausing logging|build started|reset/.test(e.x)) cls += ' mk-flash'; else if (/logging/.test(e.x)) cls += ' mk-log';
    d.innerHTML = `<span class="ts">${clock(e.t)}</span><span>${esc(e.x.replace(/^── | ──$/g, ''))}</span>`;
  } else {
    if (e.k === 'd' && CRASH_RE.test(e.x)) cls += ' crash';
    d.innerHTML = `<span class="ts">${clockMs(e.t)}</span>${e.k === 'd' ? hiLine(e.x) : esc(e.x)}`;
  }
  if (e.rel != null) d.title = `[${e.rel.toFixed(2)}] in session`;
  d.className = cls; d._e = e;
  if (!visible(e)) d.classList.add('hide');
  return d;
}
function visible(e) {
  if (e.k === 'm') return true;
  if (e.k === 't') { if (!fLevels.T) return false; }
  else if (e.k === 'd') {
    const g = e.l === 'E' || e.l === 'W' || e.l === 'I' ? e.l : 'O';
    if (!fLevels[g]) return false;
  }
  if (fRe) return fRe.test(e.x);
  if (fText) return e.x.toLowerCase().includes(fText);
  return true;
}
function applyFilter() {
  let shown = 0;
  for (const n of logEl.children) { const v = visible(n._e); n.classList.toggle('hide', !v); if (v) shown++; }
  updateCount(shown);
  if (atBottom) scrollBottom();
}
function updateCount(shown) {
  const total = logEl.children.length;
  if (shown == null) shown = fText || fRe || Object.values(fLevels).some((v) => !v) ? logEl.querySelectorAll('.ln:not(.hide)').length : total;
  $('#logCount').textContent = shown === total ? `${fmtN(total)} lines` : `${fmtN(shown)} / ${fmtN(total)} lines`;
}
function setFilter(v) { $('#filter').value = v; onFilterInput(); }
function onFilterInput() {
  const v = $('#filter').value.trim(), inp = $('#filter');
  fRe = null; fText = ''; inp.classList.remove('bad');
  const m = v.match(/^\/(.+)\/([a-z]*)$/);
  if (m) { try { fRe = new RegExp(m[1], m[2].includes('i') ? m[2] : m[2] + 'i'); } catch { inp.classList.add('bad'); } }
  else fText = v.toLowerCase();
  applyFilter();
}
function scrollBottom() { logEl.scrollTop = logEl.scrollHeight; atBottom = true; newWhilePaused = 0; $('#jump').hidden = true; }
function onLines(lines) {
  for (const e of lines) { if (e.seq > lastSeq) { lastSeq = e.seq; pendingLines.push(e); } }
  if (!logQueued && pendingLines.length) { logQueued = true; requestAnimationFrame(flushLines); }
}
function flushLines() {
  logQueued = false;
  const batch = pendingLines; pendingLines = [];
  if (!batch.length) return;
  const follow = $('#tgScroll').checked && atBottom;
  const frag = document.createDocumentFragment();
  for (const e of batch.slice(-MAX_LINES)) frag.appendChild(lineEl(e));
  logEl.appendChild(frag);
  let over = logEl.children.length - MAX_LINES;
  if (over > 0) {
    let removedH = 0;
    while (over-- > 0) { const n = logEl.firstChild; if (!follow) removedH += n.offsetHeight; n.remove(); }
    if (!follow && removedH) logEl.scrollTop -= removedH;
  }
  if (follow) scrollBottom();
  else { newWhilePaused += batch.length; $('#jump').hidden = false; $('#jumpN').textContent = `(${fmtN(newWhilePaused)} new)`; }
  updateCount();
}
logEl.addEventListener('scroll', () => {
  atBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 40;
  if (atBottom) { newWhilePaused = 0; $('#jump').hidden = true; }
});

/* ================= SSE ================= */
function setConn(up) {
  $('#conn').classList.toggle('up', up);
  $('#connText').textContent = up ? 'live' : 'reconnecting…';
}
function onHello(d) {
  const st = d.status;
  if (st.server_id !== serverId) {
    serverId = st.server_id; lastSeq = 0; logEl.innerHTML = ''; stepPlan = null;
    for (const k of Object.keys(series)) delete series[k];
    for (const k of Object.keys(latest)) delete latest[k];
  }
  Object.assign(latest, d.stats.latest); Object.assign(updated, d.stats.updated);
  for (const [k, pts] of Object.entries(d.stats.history || {})) series[k] = pts;
  lastCrashCount = st.crash_count;
  onLines(d.lines);
  onStatus(st, true);
  renderTiles(); queueCharts();
}
function connect() {
  const es = new EventSource('/events');
  const on = (name, fn) => es.addEventListener(name, (ev) => { try { fn(JSON.parse(ev.data)); } catch (err) { console.error(name, err); } });
  on('hello', (d) => { onHello(d); setConn(true); });
  on('status', (d) => onStatus(d));
  on('log', onLines);
  on('stats', onStats);
  on('rate', (d) => { if (ST) { ST.lines_per_s = d.lps; ST.session_lines = d.session_lines; clockOffset = d.now - Date.now() / 1000; renderLogInd(ST); } });
  on('alert', (a) => {
    toast({ level: a.level, title: a.title, text: a.text });
    if (a.sound) play(a.sound);
    if (a.notify) notify(a.title, a.text);
  });
  on('crash', (c) => { const t = $('#tile-crash'); if (t) { t.classList.remove('flash'); void t.offsetWidth; t.classList.add('flash'); setTimeout(() => t.classList.remove('flash'), 1300); } });
  es.onerror = () => setConn(false);
  es.onopen = () => setConn(true);
}

/* ================= wiring ================= */
function gesture() { ensureAudio(); askNotifOnce(); }
document.addEventListener('pointerdown', gesture, { once: false, passive: true });

$('#btnBuildFlash').onclick = () => api('/api/flash', { build: true });
$('#btnFlash').onclick = () => api('/api/flash', { build: false });
$('#btnBuild').onclick = () => api('/api/build');
$('#btnCancel').onclick = () => api('/api/cancel');
$('#btnReset').onclick = () => api('/api/reset');
$('#btnLog').onclick = () => api(ST && ST.logging ? '/api/log/stop' : '/api/log/start');
$('#pill').onclick = () => { if (ST && ST.state === 'ERROR') api('/api/ack'); };
$('#tgAutoLog').onchange = (e) => api('/api/config', { auto_log: e.target.checked });
$('#btnClear').onclick = () => { logEl.innerHTML = ''; updateCount(0); scrollBottom(); };
$('#jump').onclick = scrollBottom;
$('#tgScroll').onchange = (e) => { store.set('autoscroll', e.target.checked); if (e.target.checked) scrollBottom(); };
$('#tgScroll').checked = store.get('autoscroll', true);
$('#filter').addEventListener('input', onFilterInput);
$('#markForm').onsubmit = (e) => { e.preventDefault(); const t = $('#markText').value.trim(); if (t) { api('/api/mark', { text: t }); $('#markText').value = ''; } };
for (const ch of document.querySelectorAll('.chip')) {
  ch.classList.toggle('on', fLevels[ch.dataset.f]);
  ch.onclick = () => { fLevels[ch.dataset.f] = !fLevels[ch.dataset.f]; ch.classList.toggle('on', fLevels[ch.dataset.f]); store.set('levels', fLevels); applyFilter(); };
}
$('#btnTheme').onclick = () => applyTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
$('#btnSound').onclick = () => { soundOn = !soundOn; store.set('sound', soundOn); renderPrefs(); if (soundOn) play('log'); };
$('#btnNotif').onclick = () => {
  if (!('Notification' in window)) return toast({ level: 'warn', title: 'Notifications not supported' });
  if (Notification.permission === 'default') { store.set('notifAsked', true); Notification.requestPermission().then(renderPrefs); return; }
  if (Notification.permission === 'denied') return toast({ level: 'warn', title: 'Notifications are blocked', text: 'Allow them for 127.0.0.1 in the browser site settings.' });
  notifOn = !notifOn; store.set('notif', notifOn); renderPrefs();
};
window.addEventListener('resize', queueCharts);
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === 'End') scrollBottom();
  if (e.key === '/') { e.preventDefault(); $('#filter').focus(); }
});

applyTheme(store.get('theme', 'dark'));
buildTiles(); buildCharts(); renderPrefs(); renderTiles();
setInterval(() => { tickTimer(); renderTiles(); drawCharts(); if (ST && !ST.op) renderStatus(ST); }, 1000);
setInterval(tickTimer, 100);
connect();
