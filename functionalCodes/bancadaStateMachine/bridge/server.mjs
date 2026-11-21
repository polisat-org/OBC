#!/usr/bin/env node
// Bridge da bancada do OBC (somente leitura): lê a USART2 do STM32 (ST-Link, COM18), que traz o debug
// da máquina de estados e a telemetria espelhada (variante de bancada), e serve o painel em
// http://localhost:8780. O ESP32 (esp32_ttc_mock) simula o TT&C sozinho na USART1; o PC não fala com ele.
//
//   node bridge/server.mjs                    # COM18, painel na porta 8780
//   node bridge/server.mjs --stm COM18 --http 8780
//   node bridge/server.mjs --no-relay         # relay iniciado à mão (bridge/relay.ps1 -TcpPort 8772)
//
// Sem dependências: só Node >= 18. O WSL não enxerga portas COM, então a porta é lida no Windows por
// bridge/relay.ps1 e repassada por TCP (127.0.0.1:8772).
import fs from 'node:fs';
import http from 'node:http';
import net from 'node:net';
import path from 'node:path';
import { spawn, execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import {
  MODES, MODE_INFO, TASKS, TC_BITS, TELEMETRY_SIZE, TELEMETRY_FIELDS, TELEMETRY_GROUPS, TTC_SCRIPT, TTC_STEP_SECONDS,
  decodeTelemetry, DEBUG_RULES, computeChecks,
} from './obc.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '..');
const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf(`--${n}`); return i >= 0 && args[i + 1] ? args[i + 1] : d; };
const STM_COM = opt('stm', 'COM18');
const HTTP_PORT = Number(opt('http', 8780));
const TCP_PORT = Number(opt("tcp", 8772));
const AUTO_RELAY = !args.includes('--no-relay');
const ADCS_COM = opt('adcs', null);          // ex.: --adcs COM10 (USB do Arduino Nano do ADCS, debug a 115200)
const ADCS_TCP = 8774;
const SENSORES = args.includes('--sensores'); // sem a flag: bancada sem BME280/DS3231 (sensores não instalados)
const now = () => Date.now();

// ------------------------------------------------------------------ estado
const freshTasks = () => Object.fromEntries(TASKS.map((t) => [t.id, { ...t, count: 0, lastAt: null }]));
const S = {
  startedAt: now(),
  link: { com: STM_COM, connected: false, lines: 0, lastLineAt: null },
  firmware: { bancada: null, build: readBuildInfo(), tcVisivel: false },
  config: { sensores: SENSORES },
  respostas: 0,          // firmware original: grupos de resposta do OBC a comandos
  lastRespAt: null,
  obc: { mode: null, missionActive: null, deployInProgress: null, antennaDeployed: null,
    tick: null, tickAt: null, boots: 0, lastBootAt: null, uartErrors: 0, lastHkTick: null },
  tasks: freshTasks(),
  telemetry: { last: null, lastAt: null, count: 0, partial: 0, hex: null, history: [] },
  housekeepPeriods: [],
  payload: { gpio: null, lastOnAt: null, lastOffAt: null, blocks: 0, lastBlock: null, lastBlockAt: null, descartadosTotal: 0,
    resyncs: 0, uartErros: 0, semDados: null, downlinks: 0, lastDownlinkAt: null, lastDownlink: null,
    planes: [], planesAt: null, framesP: 0, tracks: {}, tcs: null, tcsFrames: 0, tcsHist: [], tcsRuins: 0, continua: false, icaosVistos: {} },
  adcs: { detumbling: false, busca: false, buscaDesde: null, cmds: 0, ultimoCmd: null, az: null, azAt: null, azN: 0, azHist: [], zFrames: 0,
    est: null, estAt: null, estN: 0, falhas: 0, reinicios: 0, reinicioAt: null, falhaAdcs: 0,
    usb: { com: ADCS_COM, connected: false, lines: 0, lastAt: null, cmdsRecebidos: 0, ultimoCmd: null, det: null, busca: null,
      omega: null, omegaHist: [], faixaS: null, erro: null, enviados: 0, msg: null } },
  tcsReceived: 0,
  lastTcAt: null,
  script: { pos: 0, current: null, done: [], byIndex: {}, synced: false },
  timeline: [],
  checks: [],
};
const RAW = []; // log técnico

function readBuildInfo() {
  try {
    const [commit, variant] = fs.readFileSync(path.join(ROOT, 'stm32', 'build', 'COMMIT'), 'utf8').trim().split(/\s+/);
    const at = fs.statSync(path.join(ROOT, 'stm32', 'build', 'stateMachine.bin')).mtimeMs;
    return { commit, variant: variant ?? 'original', at };
  } catch { return null; }
}

// ------------------------------------------------------------- log da sessão
const stamp = new Date().toISOString().replace(/[-:]/g, '').replace('T', '-').slice(0, 15);
const LOG_DIR = path.join(ROOT, 'logs', stamp);
fs.mkdirSync(LOG_DIR, { recursive: true });
const rawFile = fs.createWriteStream(path.join(LOG_DIR, 'debug.log'), { flags: 'a' });
const evFile = fs.createWriteStream(path.join(LOG_DIR, 'eventos.jsonl'), { flags: 'a' });
const tmFile = fs.createWriteStream(path.join(LOG_DIR, 'telemetria.csv'), { flags: 'a' });
tmFile.write(`pc_time_iso,${TELEMETRY_FIELDS.join(',')},hex\n`);

// --------------------------------------------------------------------- SSE
const clients = new Set();
// Cliente que parou de ler (aba em segundo plano, conexão morta) não pode acumular estado na
// memória: atrasado pula atualizações; travado (> 8 MB na fila) é desconectado e o navegador reconecta.
const send = (res, event, data) => {
  if (res.writableLength > 8e6) { clients.delete(res); res.destroy(); return; }
  if (res.writableLength > 1e6 && event === 'state') return;
  res.write(`event: ${event}\ndata: ${typeof data === 'string' ? data : JSON.stringify(data)}\n\n`);
};
function event(text, tone = 'info') {
  const e = { t: now(), text, tone };
  S.timeline = [...S.timeline, e].slice(-400);
  evFile.write(JSON.stringify(e) + '\n');
}
function raw(text, kind = 'line') {
  const e = { t: now(), text, kind };
  RAW.push(e);
  if (RAW.length > 2000) RAW.splice(0, RAW.length - 2000);
  rawFile.write(`${new Date(e.t).toISOString()} ${text}\n`);
  for (const c of clients) send(c, 'raw', e);
}
let pending = null;
function changed() {
  if (pending) return;
  pending = setTimeout(() => {
    pending = null;
    S.checks = computeChecks(S, now());
    const snap = JSON.stringify(snapshot());
    for (const c of clients) send(c, 'state', snap);
  }, 80);
}
function snapshot() {
  const { current, ...script } = S.script;
  return {
    ...S, now: now(), logDir: LOG_DIR,
    script: { ...script, current: current ? { ...current, timer: undefined, exp: undefined } : null },
    meta: { MODES, MODE_INFO, TC_BITS, TELEMETRY_GROUPS, TELEMETRY_SIZE, TTC_STEP_SECONDS,
      TTC_SCRIPT: TTC_SCRIPT.map((s) => ({ tc: s.tc, desc: s.desc })) },
  };
}
setInterval(changed, 1000);

// ------------------------------------------------- previsão da resposta a um TC
// Reproduz handleTelecommand() de main.c (com as tasks esqueleto, que terminam na hora).
function expectFor(st, tc) {
  const e = [], f = [];
  if (tc & ~0x0f) return { expect: [/rx rejected: invalid mask/], forbid: [] };
  if (st.deployInProgress) return { expect: [/rx ignored: deploy in progress/], forbid: [] };
  if ((tc & 0x02) && !st.antennaDeployed) return { expect: [/rx accepted: deploy/, /deploy task ran/, /deploy done/], forbid: [] };
  if (tc & 0x02) f.push(/rx accepted: deploy/);
  let mode = st.mode;
  if (tc & 0x04) { if (mode !== 2) { e.push(/detumbling ON/, /adcs tx 1/); mode = 2; } }
  else if (mode === 2) { e.push(/detumbling STOP/, /adcs tx 2/); mode = 0; }
  if ((tc & 0x08) && !(tc & 0x04)) { if (mode !== 3) { e.push(/pointing ON/, /adcs tx 3/); mode = 3; } }
  else if (mode === 3) { e.push(/pointing STOP/, /adcs tx 4/); mode = 0; }
  if ((tc & 0x08) && (tc & 0x04)) f.push(/pointing ON/);
  if (tc & 0x01) { if (!st.missionActive) e.push(/mission ON/, /pld ligar/); }
  else if (st.missionActive) e.push(/mission OFF/, /pld desligar/);
  return { expect: e, forbid: f };
}
const HUMAN = [
  [/adcs tx 1/, 'mandar 1 ao ADCS'], [/adcs tx 2/, 'mandar 2 ao ADCS'], [/adcs tx 3/, 'mandar 3 ao ADCS'], [/adcs tx 4/, 'mandar 4 ao ADCS'],
  [/detumbling ON/, 'ligar detumbling'], [/detumbling task ran/, 'rodar task Detumbling'], [/detumbling done/, 'concluir detumbling'],
  [/detumbling STOP/, 'parar detumbling'], [/pointing ON/, 'ligar pointing'], [/pointing STOP/, 'parar pointing'],
  [/mission ON/, 'ligar missão'], [/pld ligar/, 'mandar LIGAR ao payload'], [/mission OFF/, 'desligar missão'], [/pld desligar/, 'mandar DESLIGAR ao payload'],
  [/rx accepted: deploy/, 'iniciar deploy'], [/deploy task ran/, 'rodar task Deploy'], [/deploy done/, 'concluir deploy'],
  [/rx rejected: invalid mask/, 'rejeitar o comando'], [/rx ignored: deploy in progress/, 'ignorar o comando'],
];
const human = (re) => HUMAN.find(([r]) => String(r) === String(re))?.[1] ?? re.source;

// ------------------------------------------------------------ roteiro do TT&C
function openStep(tc) {
  closeStep();
  const sc = S.script;
  let idx = null;
  for (let k = 0; k < TTC_SCRIPT.length; k++) {
    const i = (sc.pos + k) % TTC_SCRIPT.length;
    if (TTC_SCRIPT[i].tc === tc) { idx = i; break; }
  }
  if (idx !== null) sc.pos = (idx + 1) % TTC_SCRIPT.length;
  const o = S.obc;
  const known = o.mode !== null && o.missionActive !== null && o.antennaDeployed !== null;
  const exp = known ? expectFor(o, tc) : null;
  sc.current = {
    id: (sc.done.at(-1)?.id ?? 0) + 1, idx, tc, desc: idx !== null ? TTC_SCRIPT[idx].desc : 'fora do roteiro',
    at: now(), lines: [], known, exp,
    esperado: exp ? exp.expect.map(human) : [], status: 'running',
  };
  sc.current.timer = setTimeout(closeStep, 3000);
}
function closeStep() {
  const sc = S.script;
  const c = sc.current;
  if (!c) return;
  clearTimeout(c.timer);
  sc.current = null;
  let status, nota;
  if (!c.known) { status = 'info'; nota = 'estado anterior desconhecido; aperte RESET no Nucleo para conferir desde o início'; }
  else {
    const faltou = c.exp.expect.filter((re) => !c.lines.some((l) => re.test(l))).map(human);
    const sobrou = c.exp.forbid.filter((re) => c.lines.some((l) => re.test(l))).map(human);
    if (faltou.length || sobrou.length) {
      status = 'fail';
      nota = [faltou.length && `esperava ${faltou.join(', ')}`, sobrou.length && `não devia ${sobrou.join(', ')}`].filter(Boolean).join(' · ');
    } else {
      status = 'ok';
      nota = c.exp.expect.length ? c.exp.expect.map(human).join(' → ') : 'nada a mudar, e o OBC não mudou nada';
    }
  }
  const done = { id: c.id, idx: c.idx, tc: c.tc, desc: c.desc, at: c.at, status, nota, lines: c.lines.slice(0, 10) };
  sc.done = [...sc.done, done].slice(-60);
  if (c.idx !== null) sc.byIndex[c.idx] = done;
  event(`${status === 'ok' ? 'Resposta certa' : status === 'fail' ? 'Resposta errada' : 'Resposta'} ao 0x${hex2(c.tc)}: ${nota}`,
    status === 'ok' ? 'good' : status === 'fail' ? 'bad' : 'muted');
  changed();
}

// ----------------------------------------- firmware original: inferir o TC pela resposta
// Sem o patch "rx tc", o OBC só imprime quando um TC muda algo. Agrupa as linhas de resposta
// (rx accepted/rejected/ignored + task ran + done) e procura quais bytes 0x00..0x0F produziriam
// exatamente aquela resposta a partir do estado anterior; depois encaixa no roteiro do ESP32.
let group = null;
const snap = (o) => ({ mode: o.mode, missionActive: o.missionActive, deployInProgress: o.deployInProgress, antennaDeployed: o.antennaDeployed });
function groupLine(text) {
  if (S.firmware.tcVisivel) return;
  const isRx = /^rx (accepted|rejected|ignored)/.test(text);
  if (!group && !isRx) return;
  if (!group) {
    group = { at: now(), prev: snap(S.obc), lines: [] };
    S.respostas++; S.lastRespAt = now();
  }
  group.lines.push(text);
  clearTimeout(group.timer);
  group.timer = setTimeout(closeGroup, 1200);
}
function fits(prev, tc, lines) {
  const e = expectFor(prev, tc);
  if (!e.expect.length) return false;
  if (!e.expect.every((re) => lines.some((l) => re.test(l)))) return false;
  if (e.forbid.some((re) => lines.some((l) => re.test(l)))) return false;
  return lines.filter((l) => /^rx /.test(l)).every((l) => e.expect.some((re) => re.test(l)));
}
function closeGroup() {
  const g = group;
  group = null;
  if (!g) return;
  const sc = S.script;
  const { prev, lines } = g;
  const known = prev.mode !== null && prev.missionActive !== null && prev.antennaDeployed !== null;
  const add = (d) => { d.id = (sc.done.at(-1)?.id ?? 0) + 1; sc.done = [...sc.done, d].slice(-60); if (d.idx != null) sc.byIndex[d.idx] = d; };
  if (!known) {
    add({ idx: null, tc: null, desc: 'resposta do OBC', at: g.at, status: 'info', nota: `estado anterior desconhecido; aperte RESET no Nucleo para conferir. Resposta: ${lines.join(' | ')}`, lines });
    event('Resposta do OBC recebida, mas sem estado anterior para conferir (aperte RESET)', 'muted');
    changed();
    return;
  }
  const cands = [...Array(16).keys()].filter((tc) => fits(prev, tc, lines));
  const N = TTC_SCRIPT.length;
  let found = null;
  for (let k = 0; k < N; k++) {
    const i = (sc.pos + k) % N;
    const step = TTC_SCRIPT[i];
    if (cands.includes(step.tc)) { found = { i, skipped: k }; break; }
    if (sc.synced && expectFor(prev, step.tc).expect.length) break; // um passo que devia responder não respondeu
  }
  if (found) {
    for (let k = 0; k < found.skipped && sc.synced; k++) {
      const j = (sc.pos + k) % N;
      add({ idx: j, tc: TTC_SCRIPT[j].tc, desc: TTC_SCRIPT[j].desc, at: g.at, status: 'ok', silent: true,
        nota: 'sem mensagem, como esperado (nada a mudar)', lines: [] });
    }
    const step = TTC_SCRIPT[found.i];
    const exp = expectFor(prev, step.tc).expect.map(human);
    add({ idx: found.i, tc: step.tc, desc: step.desc, at: g.at, status: 'ok', nota: exp.join(' → '), lines, inferido: true });
    sc.pos = (found.i + 1) % N;
    sc.synced = true;
    event(`Resposta certa ao 0x${hex2(step.tc)} (${step.desc}): ${exp.join(' → ')}`, 'good');
  } else {
    const expected = (() => { for (let k = 0; k < N; k++) { const i = (sc.pos + k) % N; if (expectFor(prev, TTC_SCRIPT[i].tc).expect.length) return TTC_SCRIPT[i]; } return null; })();
    add({ idx: null, tc: cands[0] ?? null, desc: 'resposta fora do esperado', at: g.at, status: 'fail', lines,
      nota: `o OBC respondeu "${lines.join(' | ')}"${cands.length ? ` (bate com 0x${hex2(cands[0])})` : ' (nenhum comando explica isso)'}; o próximo do roteiro era ${expected ? `0x${hex2(expected.tc)} (${expected.desc})` : '—'}` });
    event(`Resposta inesperada do OBC: ${lines.join(' | ')}`, 'bad');
  }
  changed();
}

// ------------------------------------------------- parser da USART2 (texto + binário)
// Linhas de debug começam com '['; um quadro de telemetria é 'T' + 85 bytes no início de uma linha.
let lineBuf = [];
let frame = null;
let pFrame = null;   // quadro 'P' espelhado: { count: null | n, bytes: [] }
let hFrame = null;   // quadro 'H' espelhado: 32 B do trailer térmico
let zFrame = null;   // quadro 'Z' espelhado: 5 caracteres do azimute
let lastByteAt = 0;
function onStmData(chunk) {
  lastByteAt = now();
  for (const b of chunk) {
    if (frame) {
      frame.push(b);
      if (frame.length === TELEMETRY_SIZE) { const f = Buffer.from(frame); frame = null; onTelemetry(f); }
      continue;
    }
    if (zFrame) {
      zFrame.push(b);
      if (zFrame.length === 5) { S.adcs.zFrames++; raw(`Z + "${Buffer.from(zFrame).toString('latin1')}" (azimute ao TT&C)`, 'tm'); zFrame = null; }
      continue;
    }
    if (hFrame) {
      hFrame.push(b);
      if (hFrame.length === 32) { raw(`H + 32 B (trailer térmico): ${Buffer.from(hFrame).toString('hex')}`, 'tm'); hFrame = null; }
      continue;
    }
    if (pFrame) {
      if (pFrame.count === null) { pFrame.count = b; if (b > 20) { raw(`quadro 'P' com count inválido (${b})`, 'error'); pFrame = null; } else if (b === 0) { onPlanes(Buffer.alloc(0), 0); pFrame = null; } continue; }
      pFrame.bytes.push(b);
      if (pFrame.bytes.length === pFrame.count * 28) { const c = pFrame.count, buf = Buffer.from(pFrame.bytes); pFrame = null; onPlanes(buf, c); }
      continue;
    }
    if (lineBuf.length === 0 && b === 0x54) { frame = []; continue; }
    if (lineBuf.length === 0 && b === 0x50) { pFrame = { count: null, bytes: [] }; continue; }
    if (lineBuf.length === 0 && b === 0x48) { hFrame = []; continue; }
    if (lineBuf.length === 0 && b === 0x5A) { zFrame = []; continue; }
    if (b === 0x0a) { const l = Buffer.from(lineBuf).toString('latin1'); lineBuf = []; onStmLine(l); continue; }
    if (lineBuf.length < 512) lineBuf.push(b);
  }
}
setInterval(() => {
  if (frame && now() - lastByteAt > 400) {
    S.telemetry.partial++;
    raw(`quadro de telemetria incompleto: ${frame.length} de ${TELEMETRY_SIZE} B`, 'error');
    event('Telemetria chegou incompleta', 'bad');
    frame = null;
    changed();
  }
}, 200);

function onPlanes(buf, count) {
  const p = S.payload;
  const t = now();
  const planes = [];
  for (let k = 0; k < count; k++) {
    const o = k * 28;
    const pl = { icao: buf.readUInt32LE(o).toString(16).toUpperCase().padStart(6, '0'), lat: buf.readFloatLE(o + 4), lon: buf.readFloatLE(o + 8),
      alt: buf.readFloatLE(o + 12), gs: buf.readFloatLE(o + 16), track: buf.readFloatLE(o + 20), posMetMs: buf.readUInt32LE(o + 24) };
    for (const f of ['lat', 'lon', 'alt', 'gs', 'track']) pl[f] = Number.isFinite(pl[f]) ? Math.round(pl[f] * 10000) / 10000 : null;
    planes.push(pl);
    if (pl.lat != null && pl.lon != null) {
      const tr = (p.tracks[pl.icao] ??= []);
      const lastPt = tr.at(-1);
      if (!lastPt || lastPt[0] !== pl.lat || lastPt[1] !== pl.lon) tr.push([pl.lat, pl.lon, t]);
      if (tr.length > 60) tr.splice(0, tr.length - 60);
    }
  }
  for (const pl of planes) {
    if (!p.icaosVistos[pl.icao]) { p.icaosVistos[pl.icao] = t; event(`✈ Avião novo: ${pl.icao}${pl.alt != null ? `, ${Math.round(pl.alt)} ft` : ''}${pl.lat != null ? ` em ${pl.lat.toFixed(3)}, ${pl.lon.toFixed(3)}` : ' (sem posição ainda)'}`, 'good'); }
  }
  p.planes = planes; p.planesAt = t; p.framesP++;
  raw(`P + ${count} aviões (${2 + count * 28} B)`, 'tm');
  changed();
}

const touchTask = (id) => { const t = S.tasks[id]; t.count++; t.lastAt = now(); };

function onTelemetry(buf) {
  const d = decodeTelemetry(buf);
  const tm = S.telemetry;
  const t = now();
  tm.last = d; tm.lastAt = t; tm.count++; tm.hex = buf.toString('hex');
  tm.history = [...tm.history, { t, clock: d.hora * 3600 + d.minuto * 60 + d.segundo,
    temp: d.temperatura_bateria, pressao: d.pressao, umidade: d.umidade, tick: null }].slice(-120);
  S.firmware.bancada = true;
  touchTask('housekeep');
  tmFile.write(`${new Date(t).toISOString()},${TELEMETRY_FIELDS.map((f) => JSON.stringify(d[f])).join(',')},${tm.hex}\n`);
  raw(`T + ${buf.length} B: ${tm.hex}`, 'tm');
  event(`Telemetria #${tm.count}: relógio ${pad(d.hora)}:${pad(d.minuto)}:${pad(d.segundo)}, ${fmt(d.temperatura_bateria)} °C, ${fmt(d.pressao / 100, 0)} hPa`, 'tm');
  changed();
}

function onStmLine(rawLine) {
  const line = rawLine.replace(/[^\x20-\x7e]/g, '').trim();
  if (!line) return;
  const L = S.link;
  L.lines++; L.lastLineAt = now();
  raw(line);
  const m = line.match(/^\[(\d{8})\]\s*(.*)$/);
  const tick = m ? Number(m[1]) : null;
  const text = m ? m[2] : line;
  const o = S.obc;
  if (tick !== null) {
    if (o.tick !== null && tick + 1000 < o.tick && !/main ready/.test(text)) event('O OBC reiniciou sem mensagem de boot (o tick voltou)', 'bad');
    o.tick = tick; o.tickAt = now();
  }
  if (S.script.current && !/rx tc=/.test(text)) S.script.current.lines.push(text);
  if (/main ready/.test(text) && group) closeGroup();
  groupLine(text);

  const rule = DEBUG_RULES.find((r) => r.re.test(text));
  if (!rule) { changed(); return; }
  const mm = text.match(rule.re);
  if (rule.kind === 'boot') {
    closeStep();
    S.script.synced = false;
    Object.assign(o, { mode: Number(mm[1]), missionActive: false, deployInProgress: false, antennaDeployed: false,
      boots: o.boots + 1, lastBootAt: now(), lastHkTick: null });
    S.tasks = freshTasks();
    S.housekeepPeriods = [];
    Object.assign(S.payload, { gpio: null, lastOnAt: null, lastOffAt: null, semDados: null });
    Object.assign(S.adcs, { detumbling: false, busca: false });
  }
  if (rule.kind === 'tc') {
    S.tcsReceived++; S.lastTcAt = now(); S.firmware.bancada = true; S.firmware.tcVisivel = true;
    event(rule.say(mm), rule.tone);
    openStep(parseInt(mm[1], 16));
    touchTask('main');
    changed();
    return;
  }
  if (rule.set) Object.assign(o, rule.set);
  if (rule.fn) rule.fn(o);
  if (rule.mode && mm[1] !== undefined) o.mode = Number(mm[1]);
  if (rule.kind === 'uart_error') o.uartErrors++;
  if (rule.pl) rule.pl(S.payload, mm, now(), S);
  if (rule.kind === 'tx') {
    if (tick !== null && o.lastHkTick !== null && tick > o.lastHkTick) S.housekeepPeriods = [...S.housekeepPeriods, (tick - o.lastHkTick) / 1000].slice(-20);
    if (tick !== null) o.lastHkTick = tick;
    // o "tx done" vem logo depois do quadro: guarda o tick do OBC naquele pacote (o PC pode receber vários juntos)
    const h = S.telemetry.history.at(-1);
    if (h && h.tick == null && tick !== null && now() - h.t < 3000) h.tick = tick;
    const tmFresh = S.telemetry.lastAt && now() - S.telemetry.lastAt < 3000;
    if (!tmFresh) { touchTask('housekeep'); if (S.firmware.bancada === null) S.firmware.bancada = false; }
  }
  touchTask(rule.task);
  if (rule.say) event(rule.say(mm), rule.tone);
  changed();
}

// ---------------------------------------------- USB do Nano (debug do ADCS)
function onAdcsLine(rawLine) {
  const line = rawLine.replace(/[^\x20-\x7e]/g, '').trim();
  if (!line) return;
  const u = S.adcs.usb;
  const t = now();
  u.lines++; u.lastAt = t;
  let m;
  if ((m = line.match(/^(\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$/))) {           // t[s],omega[rpm],torque[mN.m]
    u.omega = parseFloat(m[2]);
    u.omegaHist = [...u.omegaHist, { t, w: u.omega, tau: parseFloat(m[3]) }].slice(-600);
    changed();
    return;
  }
  raw(`[ADCS] ${line}`, 'adcs');
  if ((m = line.match(/# detumbling=(\d) busca_az=(\d) giro_ok=(\d) omega_rpm=(-?[\d.]+) na_faixa=(\d) t_faixa_s=([\d.]+) escon_ready=(\d) ultimo_az=(\S+)/))) {
    Object.assign(u, { det: m[1] === '1', busca: m[2] === '1', giroOk: m[3] === '1', omega: parseFloat(m[4]), escon: m[7] === '1',
      ultimoAzNano: m[8], estadoAt: t, vivo: true });
    changed();
    return;
  }
  if ((m = line.match(/# OBC: comando (\d+)/))) { u.cmdsRecebidos++; u.ultimoCmd = { c: +m[1], at: t }; event(`ADCS confirma pelo USB: recebeu o comando ${m[1]} do OBC`, 'good'); }
  else if (/detumbling ATIVO/.test(line)) { u.det = true; u.erro = null; }
  else if ((m = line.match(/detumbling PARADO(?: \((.*)\))?/))) { u.det = false; if (m[1] && !/comando 2/.test(m[1])) { u.erro = m[1]; event(`ADCS parou o detumbling: ${m[1]}`, 'bad'); } }
  else if (/busca de azimute ATIVA/.test(line)) u.busca = true;
  else if (/busca de azimute PARADA/.test(line)) u.busca = false;
  else if ((m = line.match(/faixa de 5 rpm atingida em ([\d.]+) s/))) { u.faixaS = parseFloat(m[1]); event(`Detumbling: abaixo de 5 rpm em ${m[1]} s`, 'good'); }
  else if ((m = line.match(/azimute enviado: (\d{3}\.\d)/))) u.enviados++;
  else if (/ERRO/.test(line)) { u.erro = line; event(`ADCS: ${line}`, 'bad'); }
  u.msg = line;
  changed();
}

// ------------------------------------------------------------- relay (TCP)
let sock = null;
net.createServer((s) => {
  if (sock) sock.destroy();
  sock = s;
  S.link.connected = true;
  event(`Conectado ao STM32 (${STM_COM})`, 'good');
  s.on('data', onStmData);
  const close = () => {
    if (sock !== s) return;
    sock = null; S.link.connected = false;
    event(`Conexão com ${STM_COM} perdida`, 'bad');
    changed();
  };
  s.on('close', close);
  s.on('error', close);
  changed();
}).listen(TCP_PORT, '127.0.0.1');

if (ADCS_COM) {
  let asock = null;
  net.createServer((s) => {
    if (asock) asock.destroy();
    asock = s; S.adcs.usb.connected = true;
    event(`Conectado ao USB do ADCS (${ADCS_COM})`, 'good');
    // '?' só imprime o estado no USB do Nano (não aciona nada): confirma que ele está vivo
    const pergunta = () => { if (asock === s) s.write('?'); };
    setTimeout(pergunta, 1500);
    const iv = setInterval(() => (asock === s ? pergunta() : clearInterval(iv)), 20000);
    let buf = '';
    s.setEncoding('latin1');
    s.on('data', (d) => { buf += d; let i; while ((i = buf.indexOf('\n')) >= 0) { const l = buf.slice(0, i); buf = buf.slice(i + 1); onAdcsLine(l); } if (buf.length > 4096) buf = ''; });
    const fim = () => { if (asock !== s) return; asock = null; S.adcs.usb.connected = false; event(`USB do ADCS (${ADCS_COM}) desconectado`, 'bad'); changed(); };
    s.on('close', fim); s.on('error', fim);
    changed();
  }).listen(ADCS_TCP, '127.0.0.1');
}

const children = new Set();
let shuttingDown = false;
function startRelay(com = STM_COM, port = TCP_PORT) {
  const ps1 = execFileSync('wslpath', ['-w', path.join(HERE, 'relay.ps1')]).toString().trim();
  const p = spawn('powershell.exe', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ps1,
    '-Port', com, '-Baud', '115200', '-TcpPort', String(port)], { stdio: ['ignore', 'pipe', 'pipe'] });
  children.add(p);
  const out = (d) => d.toString().split(/\r?\n/).filter(Boolean).forEach((l) => {
    raw(l.trim(), 'relay');
    if (/erro ao abrir/i.test(l)) { event(`Não consegui abrir ${com}: ${l.split(':').slice(1).join(':').trim()}`, 'bad'); changed(); }
  });
  p.stdout.on('data', out);
  p.stderr.on('data', out);
  p.on('exit', () => { children.delete(p); if (!shuttingDown) setTimeout(() => startRelay(com, port), 3000); });
}
function shutdown() {
  shuttingDown = true;
  sock?.destroy();
  for (const c of children) c.kill();
  setTimeout(() => process.exit(0), 300);
}
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

// -------------------------------------------------------------------- HTTP
const WEB = path.join(ROOT, 'web');
const TYPES = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml',
  '.mp3': 'audio/mpeg', '.ogg': 'audio/ogg', '.wav': 'audio/wav' };
http.createServer((req, res) => {
  const url = new URL(req.url, 'http://x');
  if (url.pathname === '/events') {
    res.writeHead(200, { 'content-type': 'text/event-stream', 'cache-control': 'no-cache', connection: 'keep-alive' });
    clients.add(res);
    send(res, 'state', snapshot());
    send(res, 'rawlog', RAW.slice(-800));
    req.on('close', () => clients.delete(res));
    return;
  }
  const f = path.join(WEB, url.pathname === '/' ? 'index.html' : path.normalize(url.pathname));
  if (!f.startsWith(WEB) || !fs.existsSync(f) || fs.statSync(f).isDirectory()) { res.writeHead(404); return res.end('404'); }
  res.writeHead(200, { 'content-type': TYPES[path.extname(f)] ?? 'application/octet-stream', 'cache-control': 'no-cache' });
  fs.createReadStream(f).pipe(res);
}).listen(HTTP_PORT, () => {
  console.log(`[bancada] painel em http://localhost:${HTTP_PORT}`);
  console.log(`[bancada] lendo ${STM_COM}; logs em ${LOG_DIR}`);
  if (AUTO_RELAY) { startRelay(); if (ADCS_COM) startRelay(ADCS_COM, ADCS_TCP); }
  else console.log(`[bancada] --no-relay: rode relay.ps1 -Port ${STM_COM} -TcpPort ${TCP_PORT}`);
});

// ------------------------------------------------------------------ utils
function hex2(v) { return v.toString(16).toUpperCase().padStart(2, '0'); }
function pad(n) { return String(n).padStart(2, '0'); }
function fmt(v, d = 1) { return typeof v === 'number' ? v.toFixed(d) : String(v); }
