// Painel da bancada do OBC (somente leitura). O estado chega do bridge por SSE (/events).
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const hex2 = (v) => v.toString(16).toUpperCase().padStart(2, '0');
const pad = (n) => String(n).padStart(2, '0');
const dur = (s) => (s < 60 ? `${Math.max(0, Math.round(s))} s` : s < 3600 ? `${Math.floor(s / 60)} min ${pad(Math.round(s % 60) % 60)} s` : `${Math.floor(s / 3600)} h ${pad(Math.floor((s % 3600) / 60))} min`);
const ago = (t, now) => (t ? `há ${dur((now - t) / 1000)}` : '—');
const hhmmss = (t) => new Date(t).toLocaleTimeString('pt-BR', { hour12: false });
const LABEL = { ok: 'ok', fail: 'problema', warn: 'atenção', pending: 'aguardando', info: 'informativo', running: 'conferindo' };
const ICON = { ok: '✓', fail: '✕', warn: '!', pending: '…', info: 'i', running: '' };
const st = (s, cls = '') => `<span class="st ${s} ${cls}" role="img" aria-label="${LABEL[s]}" title="${LABEL[s]}">${ICON[s]}</span>`;
const check = (label) => S.checks.find((c) => c.label === label);

let S = null;

// --------------------------------------------------------------------- abas
const TABS = ['visao', 'ttc', 'payload', 'adcs', 'termico', 'telemetria', 'ligacoes', 'log'];
function showTab() {
  const t = TABS.includes(location.hash.slice(1)) ? location.hash.slice(1) : 'visao';
  for (const id of TABS) $(`tab-${id}`).hidden = id !== t;
  document.querySelectorAll('#tabs a').forEach((a) => a.classList.toggle('on', a.dataset.tab === t));
}
window.addEventListener('hashchange', () => { showTab(); window.scrollTo({ top: 0 }); });
showTab();

// ---------------------------------------------------------------- derivados
function obcTick() { const o = S.obc; return o.tick != null ? o.tick + (S.now - o.tickAt) : null; }
function sinceStart() { return (S.now - Math.max(S.startedAt, S.obc.lastBootAt ?? 0)) / 1000; }
// Firmware original (código real): o OBC só imprime quando um comando muda algo, e a telemetria só vai para o TT&C.
const isOriginal = () => S.firmware.build?.variant === 'original' && !S.firmware.tcVisivel;
const nResp = () => (isOriginal() ? S.respostas : S.tcsReceived);
const lastResp = () => (isOriginal() ? S.lastRespAt : S.lastTcAt);

// Status de cada fio, a partir do que o STM32 mostra
function wires() {
  const n = S.now;
  const tcs = nResp();
  const waitedTc = sinceStart() > (isOriginal() ? 60 : 25) && S.link.connected;
  const tcSt = tcs ? 'ok' : waitedTc ? 'fail' : 'pending';
  const rtc = check('Relógio DS3231');
  const bme = check('Sensor BME280');
  const sens = [rtc, bme].filter(Boolean);
  const nFail = sens.filter((c) => c.status === 'fail').length;
  const i2cSt = !S.config.sensores ? 'info' : !sens.length ? 'pending' : nFail === sens.length ? 'fail' : nFail ? 'warn' : sens.every((c) => c.status === 'ok') ? 'ok' : 'pending';
  const i2cTxt = !S.config.sensores ? 'sensores não instalados nesta bancada (esperado)' : !sens.length ? 'aguardando a 1ª telemetria'
    : nFail === sens.length ? 'nenhum sensor responde: confira SCL/SDA, alimentação e GND dos módulos'
      : nFail ? `${sens.filter((c) => c.status === 'fail').map((c) => c.label.replace('Sensor ', '').replace('Relógio ', '')).join(' e ')} não responde` : 'sensores respondendo';
  const tx = S.tasks.transmit.count;
  const pl = S.payload;
  return [
    { sec: 'UART do TT&C (USART1, 9600 8N1): Nucleo ↔ ESP32' },
    { a: 'D2', aSub: 'PA10 · USART1_RX', dir: '←', b: 'GPIO17', bSub: 'TX2 do ESP32', sig: 'telecomandos', color: '#9085e9', st: tcSt,
      txt: tcs ? `${isOriginal() ? `o OBC reagiu a ${tcs} comandos` : `${tcs} comandos chegaram`}, último ${ago(lastResp(), n)}` : waitedTc ? 'nenhum comando chegou: TX/RX trocados, fio no D9 em vez do D2, sem GND comum ou ESP32 desligado' : 'aguardando o 1º comando (o ESP32 manda um a cada 15 s)' },
    { a: 'D8', aSub: 'PA9 · USART1_TX', dir: '→', b: 'GPIO16', bSub: 'RX2 do ESP32', sig: 'telemetria', color: '#3987e5', st: tx ? 'info' : 'pending',
      txt: tx ? `o STM32 enviou ${tx}x. Só o ESP32 sabe se recebeu: o LED azul dele dá uma piscada curta a cada 60 s.` : 'o STM32 envia a cada 60 s' },
    { a: 'GND', aSub: 'qualquer GND', dir: '—', b: 'GND', bSub: 'qualquer GND', sig: 'referência', color: '#8c8b83', st: tcs ? 'ok' : 'info',
      txt: tcs ? 'ok (os comandos estão chegando)' : 'obrigatório: sem GND comum nada passa' },
    { a: '5V', aSub: 'CN6 / header Arduino', dir: '→', b: 'VIN', bSub: '5V do ESP32', sig: 'alimentação', color: '#e66767', st: tcs ? 'ok' : 'info',
      txt: tcs ? 'ESP32 ligado (está mandando comandos)' : 'necessário quando o ESP32 não está no USB do PC' },
    { sec: 'Payload (USART3, 115200 8N1 + 2 GPIOs): Nucleo ↔ Raspberry Pi Zero W' },
    { a: 'PB11', aSub: 'USART3_RX · CN10 pino 18', dir: '←', b: 'pino 8', bSub: 'GPIO14 · TX0 do Pi', sig: 'blocos de 4080 B', color: '#1baf7a',
      st: pl.blocks ? 'ok' : pl.semDados ? 'fail' : 'pending',
      txt: pl.blocks ? `${pl.blocks} blocos recebidos, último ${ago(pl.lastBlockAt, n)}` : pl.semDados ? 'missão ligada e nenhum byte: Pi sem missão (sem SDR?), fio no pino errado ou sem GND' : 'o Pi só transmite com a missão ligada' },
    { a: 'GND', aSub: 'CN10 pino 20', dir: '—', b: 'pino 6', bSub: 'GND do Pi', sig: 'referência', color: '#8c8b83', st: pl.blocks ? 'ok' : 'info', txt: pl.blocks ? 'ok' : 'obrigatório' },
    { a: 'D6 · PB10', aSub: 'USART3_TX · CN10 pino 25', dir: '→', b: 'pino 10', bSub: 'GPIO15 · RX0 do Pi', sig: 'LIGAR / DESLIGAR', color: '#eb6834',
      st: pl.blocks && pl.lastOnAt ? 'ok' : pl.semDados ? 'fail' : pl.gpio ? 'info' : 'pending',
      txt: pl.blocks && pl.lastOnAt ? 'ok: o Pi começou a mandar depois do LIGAR' : pl.semDados ? 'LIGAR enviado e nada voltou: confira este fio, o pino 8 → PB11 e o GND' : pl.gpio === 'ligado' ? `LIGAR enviado ${ago(pl.lastOnAt, n)}; o 1º bloco leva até ~20 s (o dump1090 sobe primeiro)` : 'comando A5 5A 01 FE quando o TT&C manda o bit Missão' },
    { sec: 'ADCS (UART4 na bancada, 9600 8N1): Nucleo ↔ Arduino Nano (USART2 PA2/PA3 na PCB)' },
    { a: 'A0 · PA0', aSub: 'UART4_TX', dir: '→', b: 'D12', bSub: 'RX do Nano (SoftwareSerial)', sig: 'comandos 1–4', color: '#9085e9',
      st: S.adcs.cmds ? 'info' : 'pending', txt: S.adcs.cmds ? `${S.adcs.cmds} comandos enviados; confirme no USB do Nano ("# OBC: comando N")` : 'sai quando o TT&C liga/desliga Detumbling ou Pointing' },
    { a: 'A1 · PA1', aSub: 'UART4_RX', dir: '←', b: 'D11', bSub: 'TX do Nano (5 V)', sig: 'azimute "ddd.d"', color: '#199e70',
      st: S.adcs.azN ? 'ok' : S.adcs.busca && n - S.adcs.buscaDesde > 5000 ? 'warn' : 'pending',
      txt: S.adcs.azN ? `${S.adcs.azN} valores recebidos` : S.adcs.busca ? 'busca ativa e nada chegou: sem luz nos LDRs, fio trocado ou sem GND' : 'chega durante a busca (Pointing), com luz' },
    { a: 'GND', aSub: 'qualquer GND', dir: '—', b: 'GND', bSub: 'do Nano', sig: 'referência', color: '#8c8b83', st: S.adcs.azN ? 'ok' : 'info', txt: 'obrigatório' },
    { sec: 'Sensores (I2C1): Nucleo ↔ BME280 e DS3231' },
    { a: 'D15', aSub: 'PB8 · I2C1_SCL', dir: '→', b: 'SCL', bSub: 'BME280 e DS3231', sig: 'clock I2C', color: '#199e70', st: i2cSt, txt: i2cTxt },
    { a: 'D14', aSub: 'PB9 · I2C1_SDA', dir: '↔', b: 'SDA', bSub: 'BME280 e DS3231', sig: 'dados I2C', color: '#1baf7a', st: i2cSt, txt: i2cTxt },
    { a: '3V3 / GND', aSub: 'header Arduino', dir: '→', b: 'VCC / GND', bSub: 'módulos dos sensores', sig: 'alimentação', color: '#e66767', st: i2cSt === 'fail' ? 'warn' : i2cSt, txt: !S.config.sensores ? 'sensores não instalados nesta bancada' : i2cSt === 'ok' ? 'ok' : 'confira se os módulos estão alimentados' },
    { sec: 'PC' },
    { a: 'USB', aSub: 'ST-Link (USART2: PA2/PA3)', dir: '→', b: S.link.com, bSub: 'porta COM no PC', sig: 'debug + telemetria', color: '#c3c2b7', st: S.link.connected ? 'ok' : 'fail',
      txt: S.link.connected ? `${S.link.lines} linhas lidas` : 'porta fechada: cabo, ou COM aberta em outro programa' },
  ];
}

// --------------------------------------------------------------------- topo
function renderTop() {
  const L = S.link;
  const b = S.firmware.build;
  $('chips').innerHTML = [
    `<span class="chip"><span class="dot ${L.connected ? 'on live' : 'off'}"></span>STM32 <b>${esc(L.com)}</b> ${L.connected ? 'conectado' : 'desconectado'}</span>`,
    b ? `<span class="chip">firmware <b>${esc(b.commit)}</b> ${b.variant === 'bancada' ? 'variante bancada' : 'original'}</span>` : '',
  ].join('');
  const w = wires().filter((x) => !x.sec);
  const wf = w.filter((x) => x.st === 'fail').length;
  badge('b-pins', wf ? 'fail' : '', wf || '');
  const bad = S.script.done.filter((d) => d.status === 'fail').length;
  badge('b-ttc', bad ? 'fail' : S.script.done.length ? 'ok' : '', bad || (S.script.done.length ? S.script.done.length : ''));
  badge('b-tm', S.telemetry.count ? 'ok' : '', S.telemetry.count || '');
  const pl = S.payload;
  badge('b-pld', pl.semDados && !pl.blocks ? 'fail' : pl.blocks ? 'ok' : '', pl.semDados && !pl.blocks ? '!' : pl.blocks || '');
  const ad = S.adcs;
  badge('b-adcs', ad.azN ? 'ok' : '', ad.azN ? ad.az.toFixed(0) + '°' : '');
  const tf = S.checks.filter((c) => c.group === 'Controle térmico (TCS)' && c.status === 'fail').length;
  badge('b-tcs', tf ? 'fail' : pl.tcs ? 'ok' : '', tf ? '!' : pl.tcs ? '✓' : '');
}
function badge(id, cls, txt) { const e = $(id); e.className = `badge ${cls}`; e.textContent = txt; }

// ------------------------------------------------------------ visão geral
function renderSummary() {
  const o = S.obc;
  const n = S.now;
  const alive = check('OBC dando sinal de vida');
  const mode = o.mode != null ? S.meta.MODES[o.mode] : null;
  const cards = [];
  // OBC
  if (!S.link.connected) cards.push(['fail', 'OBC', 'sem conexão', `${S.link.com} fechada`, 'ligacoes']);
  else if (alive?.status === 'fail') cards.push(['fail', 'OBC', 'sem resposta', alive.detail, 'log']);
  else if (mode) cards.push(['ok', 'OBC', mode, `${S.meta.MODE_INFO[mode].titulo} · ligado há ${dur(obcTick() / 1000)}`, 'visao']);
  else cards.push(['pending', 'OBC', 'ligado', 'modo desconhecido: aperte RESET no Nucleo', 'visao']);
  // TT&C
  const done = S.script.done;
  const bad = done.filter((d) => d.status === 'fail').length;
  if (nResp()) cards.push([bad ? 'warn' : 'ok', 'Comandos do TT&C', isOriginal() ? `${nResp()} reações` : `${nResp()} recebidos`, done.length ? `${done.length - bad} conferidos certos, ${bad} errados` : `último ${ago(lastResp(), n)}`, 'ttc']);
  else if (sinceStart() > (isOriginal() ? 60 : 25) && S.link.connected) cards.push(['fail', 'Comandos do TT&C', 'nenhum', 'não chegam ao OBC: confira os fios', 'ligacoes']);
  else cards.push(['pending', 'Comandos do TT&C', 'aguardando', 'o ESP32 manda um a cada 15 s', 'ttc']);
  // Telemetria
  const tm = S.telemetry;
  const tick = obcTick();
  const left = tick != null ? (60000 - ((o.lastHkTick != null ? tick - o.lastHkTick : tick) % 60000)) / 1000 : null;
  if (isOriginal() && S.tasks.transmit.count) cards.push(['ok', 'Telemetria', `${S.tasks.transmit.count} enviadas`, `para o TT&C · próxima em ~${dur(left)}`, 'telemetria']);
  else if (tm.count) cards.push(['ok', 'Telemetria', `${tm.count} pacotes`, `último ${ago(tm.lastAt, n)} · próximo em ~${dur(left)}`, 'telemetria']);
  else if (S.tasks.transmit.count) cards.push(['fail', 'Telemetria', 'não aparece', 'o OBC envia, mas o pacote não chega na USB', 'telemetria']);
  else cards.push(['pending', 'Telemetria', 'aguardando', left != null ? `primeiro em ~${dur(left)}` : 'a cada 60 s', 'telemetria']);
  // Sensores
  const sens = ['Relógio DS3231', 'Sensor BME280'].map(check).filter(Boolean);
  const sf = sens.filter((c) => c.status === 'fail');
  if (!S.config.sensores) cards.push(['info', 'Sensores', 'não instalados', 'BME280 e DS3231 ausentes nesta bancada', 'telemetria']);
  else if (!sens.length) cards.push(['pending', 'Sensores', 'aguardando', 'precisa de telemetria', 'telemetria']);
  else if (sf.length) cards.push(['fail', 'Sensores', `${sf.length} sem resposta`, sf.map((c) => c.label.replace('Sensor ', '').replace('Relógio ', '')).join(' e ') + ' · ver pinos I2C', 'ligacoes']);
  else cards.push([sens.every((c) => c.status === 'ok') ? 'ok' : 'pending', 'Sensores', 'respondendo', 'DS3231 e BME280', 'telemetria']);

  const pl = S.payload;
  if (pl.blocks) cards.splice(2, 0, ['ok', 'Payload', `${pl.lastBlock.validos} aviões`, `${pl.blocks} blocos do Pi · ${pl.downlinks} repasses ao TT&C`, 'payload']);
  else if (pl.semDados) cards.splice(2, 0, ['fail', 'Payload', 'sem dados', 'missão ligada e o Pi não mandou nada', 'payload']);
  else if (pl.gpio === 'ligado') cards.splice(2, 0, ['pending', 'Payload', 'ligando', `LIGAR enviado ${ago(pl.lastOnAt, n)}; 1º bloco em até ~20 s`, 'payload']);
  else cards.splice(2, 0, ['pending', 'Payload', 'desligado', 'liga com o bit Missão do TT&C', 'payload']);
  const ad = S.adcs;
  cards.splice(3, 0, ad.azN
    ? ['ok', 'ADCS', `${ad.az.toFixed(1)}°`, `azimute ${ago(ad.azAt, n)} · detumbling ${ad.detumbling ? 'ligado' : 'desligado'}`, 'adcs']
    : ad.cmds ? ['pending', 'ADCS', ad.busca ? 'buscando' : ad.detumbling ? 'detumbling' : 'parado', `último comando ${ad.ultimoCmd.c} ${ago(ad.ultimoCmd.at, n)}`, 'adcs']
      : ['pending', 'ADCS', 'aguardando', 'comandos saem com Detumbling/Pointing', 'adcs']);
  const tc = pl.tcs;
  if (tc && S.now - tc.at < 15000) {
    const v = tc.sondas.filter((x) => x != null);
    const tmin = v.length ? Math.min(...v) : null;
    cards.splice(3, 0, [tc.valido && !Math.max(...tc.sit) ? 'ok' : tc.valido ? 'warn' : 'fail', 'Baterias (TCS)',
      tmin != null ? `${tmin.toFixed(1)} °C` : 'sem sonda', `sonda mais fria · aquecedores ${tc.duty[0]} % / ${tc.duty[1]} % (observação)`, 'termico']);
  } else cards.splice(3, 0, ['pending', 'Baterias (TCS)', 'aguardando', 'trailer térmico a cada 5 s', 'termico']);
  $('summary').innerHTML = cards.map(([s, k, big, d, tab]) => `<a class="sum ${s}" href="#${tab}">
    <div class="top-row"><span class="k">${k}</span>${st(s)}</div><div class="big">${esc(big)}</div><div class="d">${esc(d)}</div></a>`).join('');

  const probs = S.checks.filter((c) => c.status === 'fail');
  const where = (c) => (/Comandos|conectado|sensor|Relógio|BME|STM32/i.test(c.label) ? ['ligacoes', 'ver pinos'] : /Telemetria/.test(c.label) ? ['telemetria', 'ver telemetria'] : ['log', 'ver log']);
  $('todo').innerHTML = probs.length ? `<h2>${st('fail')} O que fazer agora</h2><ol>${probs.map((c, i) => {
    const [tab, lbl] = where(c);
    return `<li><span class="n">${i + 1}</span><div><div class="w">${esc(c.label)}: ${esc(c.detail)}</div>${c.fix ? `<div class="f">${esc(c.fix)}</div>` : ''}</div><a href="#${tab}">${lbl} →</a></li>`;
  }).join('')}</ol>` : '';
}

function renderStateMachine() {
  const o = S.obc;
  const mode = o.mode != null ? S.meta.MODES[o.mode] : null;
  const info = S.meta.MODE_INFO;
  $('mode-hint').textContent = mode ? `modo atual: ${mode}` : 'modo desconhecido: aperte RESET no Nucleo';
  const W = 150, H = 56;
  const N = {
    MISSION: [205, 12], DETUMBLING: [15, 137], IDLE: [205, 137], POINTING: [395, 137], DEPLOY: [205, 262],
  };
  const edges = [
    ['MISSION', 280, 137, 280, 68, '0x01', 290, 106],
    ['DETUMBLING', 205, 165, 165, 165, '0x04', 166, 155],
    ['POINTING', 355, 165, 395, 165, '0x08', 357, 155],
    ['DEPLOY', 280, 193, 280, 262, '0x02', 290, 232],
  ];
  const node = (name, t2, cls) => {
    const [x, y] = N[name];
    const on = cls.includes('on');
    return `<g class="node ${cls}">${on ? `<rect class="halo" x="${x}" y="${y}" width="${W}" height="${H}" rx="12"/>` : ''}
      <rect x="${x}" y="${y}" width="${W}" height="${H}" rx="12"/>
      <text class="t1" x="${x + W / 2}" y="${y + 24}" text-anchor="middle">${name === 'DEPLOY' ? 'ANTENA' : name}</text>
      <text class="t2" x="${x + W / 2}" y="${y + 42}" text-anchor="middle">${esc(t2)}</text></g>`;
  };
  const antenna = o.antennaDeployed == null ? 'estado desconhecido' : o.antennaDeployed ? 'implantada ✓' : o.deployInProgress ? 'deploy em andamento' : 'recolhida';
  $('sm').innerHTML = `<defs>
      <marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="#4a4d55"/></marker>
      <marker id="ahh" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="#3987e5"/></marker></defs>
    ${edges.map(([to, x1, y1, x2, y2, lbl, lx, ly]) => {
      const hot = to === mode || (to === 'DEPLOY' && o.deployInProgress);
      return `<line class="edge ${hot ? 'hot' : ''}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" marker-start="url(#${hot ? 'ahh' : 'ah'})" marker-end="url(#${hot ? 'ahh' : 'ah'})"/>
        <text class="elabel" x="${lx}" y="${ly}">${lbl}</text>`;
    }).join('')}
    ${['MISSION', 'DETUMBLING', 'IDLE', 'POINTING'].map((m) => node(m, info[m].titulo, m === mode ? 'on' : '')).join('')}
    ${node('DEPLOY', antenna, `flag ${o.antennaDeployed ? 'done' : ''} ${o.deployInProgress ? 'on' : ''}`)}`;

  const yn = (v, yes, no) => (v == null ? '<div class="v muted">desconhecido</div>' : `<div class="v">${v ? yes : no}</div>`);
  const tick = obcTick();
  $('sm-side').innerHTML = `
    <div class="fact"><div class="k">Ligado há</div><div class="v">${tick != null ? dur(tick / 1000) : '—'}</div></div>
    <div class="fact"><div class="k">Missão</div>${yn(o.missionActive, 'ativa', 'parada')}</div>
    <div class="fact"><div class="k">Antena</div>${yn(o.antennaDeployed, 'implantada', 'recolhida')}</div>
    <div class="fact"><div class="k">Bloqueio por deploy</div>${yn(o.deployInProgress, 'sim', 'não')}</div>`;

  const timers = [];
  if (tick != null) {
    const since = (o.lastHkTick != null ? tick - o.lastHkTick : tick) % 60000;
    timers.push(timer('Próxima telemetria', `em ~${dur((60000 - since) / 1000)}`, since / 60000));
  } else timers.push(timer('Próxima telemetria', 'aguardando o OBC', null));
  const step = S.meta.TTC_STEP_SECONDS * 1000;
  if (isOriginal()) {
    timers.push(lastResp() ? timer('Última reação do OBC a um comando', ago(lastResp(), S.now), Math.min(1, (S.now - lastResp()) / 30000))
      : timer('Última reação do OBC a um comando', 'nenhuma ainda', null));
  } else if (S.lastTcAt) {
    const el = S.now - S.lastTcAt;
    timers.push(timer('Próximo comando do TT&C', el > step + 3000 ? `atrasado ${dur((el - step) / 1000)}` : `em ~${dur((step - Math.min(el, step)) / 1000)}`, Math.min(el, step) / step));
  } else timers.push(timer('Próximo comando do TT&C', 'nenhum recebido ainda', null));
  $('timers').innerHTML = timers.join('');
}
const timer = (label, value, frac) => `<div class="timer"><div class="row"><span>${label}</span><b>${value}</b></div>
  <div class="bar ${frac == null ? 'idle' : ''}"><div style="width:${frac == null ? 0 : Math.round(Math.min(1, Math.max(0, frac)) * 100)}%"></div></div></div>`;

function feedItems(list) {
  return list.length ? list.map((e) => `<li class="ev ${e.tone}"><span class="ts">${hhmmss(e.t)}</span><span class="mk"></span><span class="tx">${esc(e.text)}</span></li>`).join('')
    : '<li class="empty">Nada ainda.</li>';
}
function renderFeed() {
  const ev = [...S.timeline].reverse();
  $('feed').innerHTML = feedItems(ev.slice(0, 9));
  $('timeline').innerHTML = feedItems(ev.slice(0, 300));
}
function renderTasks() {
  const n = S.now;
  $('tasks').innerHTML = Object.values(S.tasks).map((t) => {
    const fresh = t.lastAt && n - t.lastAt < 65000;
    return `<div class="task ${fresh ? 'fresh' : ''}"><div class="n"><span class="dot ${fresh ? 'on' : ''}"></span>${esc(t.name)}</div>
      <div class="p">${esc(t.papel)}</div><div class="c"><b>${t.count}</b>× · ${t.lastAt ? ago(t.lastAt, n) : 'nunca'}</div></div>`;
  }).join('');
}

// ------------------------------------------------------------------- TT&C
function renderTtc() {
  const sc = S.script;
  const n = S.now;
  const done = sc.done;
  const ok = done.filter((d) => d.status === 'ok').length;
  const bad = done.filter((d) => d.status === 'fail').length;
  $('script-now').innerHTML = sc.current ? `<div class="script-now">${st('running')}<span>Conferindo a resposta ao <b class="mono">0x${hex2(sc.current.tc)}</b> (${esc(sc.current.desc)})${sc.current.esperado.length ? `. Espero: ${esc(sc.current.esperado.join(' → '))}` : ''}</span></div>` : '';
  $('score').innerHTML = done.length
    ? `<span>${st('ok')} <b>${ok}</b> respostas certas</span><span>${st('fail')} <b>${bad}</b> erradas</span><span>${isOriginal() ? `${nResp()} reações do OBC (firmware original: o comando é deduzido pela resposta; comandos sem efeito aparecem como silenciosos)` : `${S.tcsReceived} comandos recebidos no total`}</span>`
    : `<span class="hint">${S.tcsReceived ? '' : `Nenhum comando recebido ainda. Se passar de 25 s, veja a aba Ligações e pinos.`}</span>`;
  $('steps').innerHTML = S.meta.TTC_SCRIPT.map((s, i) => {
    const r = sc.byIndex[i];
    const isNext = i === sc.pos && !sc.current && (sc.synced || S.tcsReceived > 0);
    const status = sc.current?.idx === i ? 'running' : r ? r.status : 'pending';
    const res = r ? `<div class="r ${r.status === 'fail' ? 'bad' : ''}">${esc(r.nota)}</div>` : `<div class="r">${isNext ? 'próximo do roteiro' : 'ainda não recebido'}</div>`;
    return `<li class="step ${isNext ? 'next' : ''} ${r?.status === 'fail' ? 'fail' : ''}">${st(status, 'lg')}<span class="tc">0x${hex2(s.tc)}</span>
      <span class="d">${i + 1}. ${esc(s.desc)}</span><span class="when">${r ? ago(r.at, n) : ''}</span>${res}</li>`;
  }).join('');
  const fora = done.filter((d) => d.idx == null).slice(-6).reverse();
  if (fora.length) $('steps').innerHTML += `<li class="hint" style="margin-top:8px">Respostas que não se encaixam no roteiro:</li>` + fora.map((d) =>
    `<li class="step ${d.status === 'fail' ? 'fail' : ''}">${st(d.status, 'lg')}<span class="tc">${d.tc != null ? `0x${hex2(d.tc)}` : '?'}</span><span class="d">${esc(d.desc)}</span><span class="when">${ago(d.at, n)}</span><div class="r ${d.status === 'fail' ? 'bad' : ''}">${esc(d.nota)}</div></li>`).join('');
  $('bits-help').innerHTML = S.meta.TC_BITS.map((b) => `<div class="bit"><b>0x${hex2(b.bit)}</b> · bit ${Math.log2(b.bit)}<div>${esc(b.name)}</div></div>`).join('') +
    `<p class="hint" style="grid-column:1/-1">O comando é um vetor de estado: cada bit ligado mantém a função ativa, e bit desligado manda parar. Ex.: 0x09 = Pointing + Missão; 0x00 = desliga tudo. Deploy é feito uma vez só por boot.</p>`;
}

// -------------------------------------------------------------- telemetria
let prevTm = null, curTm = null, shownCount = -1;
function renderTelemetry() {
  const tm = S.telemetry;
  const n = S.now;
  if (tm.count !== shownCount) { prevTm = curTm; curTm = tm.last; shownCount = tm.count; }
  const d = tm.last;
  const h = tm.history;
  $('tm-meta').textContent = isOriginal() && !tm.count ? `firmware original: o pacote vai só para o TT&C (USART1), então o PC não vê o conteúdo. ${S.tasks.transmit.count} envios ("tx done"). Para ver os campos, grave a variante de bancada.` : tm.count ? `${tm.count} pacotes · último ${ago(tm.lastAt, n)} · um a cada 60 s` : 'o primeiro pacote chega 60 s depois do boot';
  const rtc = check('Relógio DS3231');
  const bme = check('Sensor BME280');
  const rtcBad = rtc?.status === 'fail', bmeBad = bme?.status === 'fail';
  const clock = d ? `${pad(d.hora)}:${pad(d.minuto)}:${pad(d.segundo)}` : '—';
  $('tiles').innerHTML = [
    `<div class="tile ${rtcBad ? 'bad' : ''}"><div class="t"><span>Relógio do OBC</span><span>DS3231</span></div><div class="v mono">${clock}</div>
      <div class="s ${rtcBad ? 'bad' : ''}">${rtcBad ? 'sem resposta: valor inválido' : d ? `data ${pad(d.dia)}/${pad(d.mes)}/${pad(d.ano)}` : 'sem dados'}</div></div>`,
    tile('Temperatura', 'BME280', d?.temperatura_bateria, '°C', h.map((x) => [x.t, x.temp]), 1, bmeBad),
    tile('Pressão', 'BME280', d ? d.pressao / 100 : null, 'hPa', h.map((x) => [x.t, x.pressao / 100]), 0, bmeBad),
    tile('Umidade', 'BME280', d?.umidade, '%', h.map((x) => [x.t, x.umidade]), 0, bmeBad),
    `<div class="tile"><div class="t"><span>Antena</span><span>deploy_antena</span></div><div class="v">${d ? (d.deploy_antena ? 'implantada' : 'recolhida') : '—'}</div><div class="s">${d ? `valor ${d.deploy_antena}` : 'sem dados'}</div></div>`,
  ].join('');
  bindSpark();

  $('groups').innerHTML = S.meta.TELEMETRY_GROUPS.map((g) => {
    const zero = d && g.fields.every((f) => !d[f]);
    return `<div class="grp ${zero ? 'dim' : ''}"><h4>${esc(g.name)} <span>${esc(g.sensor)}${zero ? ' · sempre 0' : ''}</span></h4>${g.fields.map((f) => {
      const v = d ? d[f] : null;
      const chg = prevTm && d && prevTm[f] !== v;
      return `<div class="fld"><span class="k" title="${f}">${f}</span><span class="val ${chg ? 'chg' : ''}">${v == null ? '—' : esc(typeof v === 'string' ? JSON.stringify(v) : v)}</span></div>`;
    }).join('')}</div>`;
  }).join('');
  $('hex').textContent = tm.hex ? `'T' + ${S.meta.TELEMETRY_SIZE} B: ${tm.hex.replace(/(..)/g, '$1 ').trim()}` : 'nenhum pacote ainda';
}
// Sparkline: uma série, linha de 2px, sem eixo; o título do bloco nomeia a série; hover mostra valor e horário.
function tile(title, sensor, value, unit, pts, dec, bad) {
  const valid = pts.filter(([, v]) => typeof v === 'number' && Number.isFinite(v));
  let svg = '';
  if (valid.length >= 2 && !bad) {
    const xs = valid.map((p) => p[0]), ys = valid.map((p) => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs);
    let y0 = Math.min(...ys), y1 = Math.max(...ys);
    if (y1 - y0 < 1e-6) { y0 -= 1; y1 += 1; }
    const X = (x) => ((x - x0) / (x1 - x0 || 1)) * 296 + 2;
    const Y = (y) => 36 - ((y - y0) / (y1 - y0)) * 32;
    const path = valid.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('');
    const [lx, ly] = valid.at(-1);
    svg = `<svg viewBox="0 0 300 40" preserveAspectRatio="none" data-pts='${JSON.stringify(valid.map((p) => [X(p[0]), Y(p[1]), p[0], p[1]]))}' data-unit="${unit}" data-dec="${dec}">
      <path d="${path}" fill="none" stroke="var(--series-1)" stroke-width="2" vector-effect="non-scaling-stroke" stroke-linejoin="round"/>
      <circle cx="${X(lx)}" cy="${Y(ly)}" r="4" fill="var(--series-1)" stroke="var(--surface-2)" stroke-width="2" vector-effect="non-scaling-stroke"/>
      <line class="x" y1="0" y2="40" stroke="var(--text-muted)" vector-effect="non-scaling-stroke" visibility="hidden"/></svg>`;
  }
  const v = typeof value === 'number' && Number.isFinite(value) ? value.toFixed(dec) : '—';
  const sub = bad ? '<div class="s bad">sensor sem resposta: valor inválido</div>' : valid.length >= 2 ? '' : `<div class="s">${valid.length ? 'tendência a partir do 2º pacote' : 'sem dados'}</div>`;
  return `<div class="tile ${bad ? 'bad' : ''}"><div class="t"><span>${title}</span><span>${sensor}</span></div><div class="v">${v} <small>${unit}</small></div>${sub}${svg}<div class="tip" hidden></div></div>`;
}
function bindSpark() {
  document.querySelectorAll('.tile svg[data-pts]').forEach((svg) => {
    const pts = JSON.parse(svg.dataset.pts);
    const tip = svg.parentElement.querySelector('.tip');
    const x = svg.querySelector('.x');
    svg.addEventListener('pointermove', (e) => {
      const r = svg.getBoundingClientRect();
      const px = ((e.clientX - r.left) / r.width) * 300;
      const p = pts.reduce((a, b) => (Math.abs(b[0] - px) < Math.abs(a[0] - px) ? b : a));
      x.setAttribute('x1', p[0]); x.setAttribute('x2', p[0]); x.setAttribute('visibility', 'visible');
      tip.hidden = false;
      tip.style.left = `${(p[0] / 300) * r.width + svg.offsetLeft}px`;
      tip.style.top = `${svg.offsetTop + (p[1] / 40) * r.height}px`;
      tip.textContent = `${p[3].toFixed(Number(svg.dataset.dec))} ${svg.dataset.unit} · ${hhmmss(p[2])}`;
    });
    svg.addEventListener('pointerleave', () => { tip.hidden = true; x.setAttribute('visibility', 'hidden'); });
  });
}

// ----------------------------------------------------------------- payload
function renderPayload() {
  const pl = S.payload;
  const n = S.now;
  const node = (st_, title, sub, value, detail, fix) => `<div class="node ${st_}"><div class="nh"><div><div class="nt">${title}</div><div class="ns">${sub}</div></div>${st(st_)}</div>
    <div class="nv">${value}</div><div class="nd">${detail}</div>${fix ? `<div class="nf">${fix}</div>` : ''}</div>`;
  const piSt = pl.blocks ? 'ok' : pl.semDados ? 'fail' : pl.gpio === 'ligado' ? 'running' : 'pending';
  const obcSt = pl.blocks ? (pl.descartadosTotal || pl.resyncs ? 'warn' : 'ok') : 'pending';
  const ttcSt = pl.downlinks ? 'ok' : 'pending';
  $('pld-flow').innerHTML = [
    node(piSt === 'running' ? 'pending' : piSt, 'Raspberry Pi', 'HERMES Simples · ADS-B',
      pl.gpio === 'ligado' ? 'missão ligada' : pl.gpio === 'desligado' ? 'missão desligada' : '—',
      pl.gpio === 'ligado' ? `LIGAR enviado ${ago(pl.lastOnAt, n)} (repete a cada 2 s)` : 'aguardando o bit Missão do TT&C',
      pl.semDados && !pl.blocks ? 'Nada chegou do Pi. Veja journalctl -u hermes-simples no Pi (o dump1090 precisa subir com o SDR) e confira PB10 → pino 10, pino 8 → PB11 e o GND.' : ''),
    `<div class="arrow"><b>⇄</b>USART3 · 115200<br>← LIGAR/DESLIGAR<br>→ 4080 B a cada 5 s</div>`,
    node(obcSt, 'OBC', 'recebe por DMA e valida',
      pl.blocks ? `${pl.blocks} blocos` : '0 blocos',
      pl.lastBlock ? `último ${ago(pl.lastBlockAt, n)}: ${pl.lastBlock.validos} aviões, ${pl.lastBlock.vazios} vazios, ${pl.lastBlock.descartados} descartados` : 'nenhum bloco ainda',
      pl.resyncs || pl.uartErros ? `${pl.resyncs} realinhamentos, ${pl.uartErros} erros de UART` : ''),
    `<div class="arrow"><b>→</b>USART1 · 9600<br>'P' (28 B/avião) a cada 5 s</div>`,
    node(ttcSt, 'TT&C', 'ESP32 → LoRa → solo',
      pl.downlinks ? `${pl.downlinks} quadros 'P'` : "0 quadros 'P'",
      pl.lastDownlink ? `último ${ago(pl.lastDownlinkAt, n)}: ${pl.lastDownlink.n} aviões, ${pl.lastDownlink.bytes} B` : 'sai a cada 5 s com a missão ligada e dados do Pi', ''),
  ].join('');

  const planes = pl.planes;
  $('pld-meta').textContent = pl.planesAt ? `${planes.length} aviões · quadro de ${ago(pl.planesAt, n)}` : 'nenhum quadro ainda';
  const f = (v, d, u = '') => (v == null ? '<td class="na">—</td>' : `<td>${v.toFixed(d)}${u}</td>`);
  $('pld-table').innerHTML = `<tr><th>ICAO</th><th>Latitude</th><th>Longitude</th><th>Altitude (ft)</th><th>Velocidade (kt)</th><th>Rumo (°)</th><th>Posição (MET)</th></tr>` +
    (planes.length ? planes.map((p) => `<tr><td>${p.icao}</td>${f(p.lat, 4)}${f(p.lon, 4)}${f(p.alt, 0)}${f(p.gs, 0)}${f(p.track, 0)}<td>${p.posMetMs ? (p.posMetMs / 1000).toFixed(1) + ' s' : '—'}</td></tr>`).join('')
      : '<tr><td colspan="7" class="na" style="text-align:left">Os aviões aparecem aqui quando o OBC repassar o primeiro quadro ao TT&C.</td></tr>');
  renderMap();
}
function renderMap() {
  const tr = S.payload.tracks;
  const pts = Object.values(tr).flat();
  const W = 420, H = 300, M = 34;
  if (!pts.length) { $('pld-map').innerHTML = `<text x="${W / 2}" y="${H / 2}" text-anchor="middle" fill="#8c8b83" font-size="13">sem posições ainda</text>`; return; }
  let la0 = Math.min(...pts.map((p) => p[0])), la1 = Math.max(...pts.map((p) => p[0]));
  let lo0 = Math.min(...pts.map((p) => p[1])), lo1 = Math.max(...pts.map((p) => p[1]));
  const pad = Math.max(0.01, (la1 - la0) * 0.15, (lo1 - lo0) * 0.15);
  la0 -= pad; la1 += pad; lo0 -= pad; lo1 += pad;
  const k = Math.min((W - 2 * M) / (lo1 - lo0), (H - 2 * M) / (la1 - la0));
  const X = (lo) => M + (lo - lo0) * k, Y = (la) => H - M - (la - la0) * k;
  let g = '';
  for (let i = 0; i <= 4; i++) {
    const lo = lo0 + (lo1 - lo0) * i / 4, la = la0 + (la1 - la0) * i / 4;
    g += `<line class="grid" x1="${X(lo)}" y1="${M / 2}" x2="${X(lo)}" y2="${H - M / 2}"/><text class="glabel" x="${X(lo)}" y="${H - 6}" text-anchor="middle">${lo.toFixed(2)}</text>`;
    g += `<line class="grid" x1="${M / 2}" y1="${Y(la)}" x2="${W - M / 2}" y2="${Y(la)}"/><text class="glabel" x="4" y="${Y(la) - 3}">${la.toFixed(2)}</text>`;
  }
  for (const [icao, t] of Object.entries(tr)) {
    if (t.length > 1) g += `<polyline class="trk" points="${t.map((p) => `${X(p[1]).toFixed(1)},${Y(p[0]).toFixed(1)}`).join(' ')}"/>`;
    const [la, lo] = t.at(-1);
    g += `<circle class="pt" cx="${X(lo)}" cy="${Y(la)}" r="5"><title>${icao} · ${la.toFixed(4)}, ${lo.toFixed(4)}</title></circle><text class="plabel" x="${X(lo) + 8}" y="${Y(la) - 6}">${icao}</text>`;
  }
  $('pld-map').innerHTML = g;
}

// -------------------------------------------------------------------- ADCS
function renderAdcs() {
  const a = S.adcs;
  const n = S.now;
  $('adcs-meta').textContent = a.azN ? `${a.azN} valores · último ${ago(a.azAt, n)} · ${a.zFrames} quadros 'Z' ao TT&C` : 'nenhum valor ainda';
  const C = 150, R = 118;
  const P = (deg, r) => [C + r * Math.sin(deg * Math.PI / 180), C - r * Math.cos(deg * Math.PI / 180)];
  let g = `<circle class="aro" cx="${C}" cy="${C}" r="${R + 14}"/>`;
  for (let d = 0; d < 360; d += 15) {
    const [x1, y1] = P(d, R + 12), [x2, y2] = P(d, d % 90 ? R + 4 : R - 6);
    g += `<line class="tick ${d % 90 ? '' : 'm'}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
  }
  [['0° FRONTAL', 0], ['90° DIREITO', 90], ['180° TRASEIRO', 180], ['270° ESQUERDO', 270]].forEach(([t, d]) => {
    const [x, y] = P(d, R - 22);
    g += `<text class="lbl" x="${x}" y="${y + 4}" text-anchor="middle">${t}</text>`;
  });
  for (const h of a.azHist.slice(-30)) { const [x, y] = P(h.az, R - 4); g += `<circle class="rastro" cx="${x}" cy="${y}" r="5"/>`; }
  if (a.az != null) {
    const [x, y] = P(a.az, R - 8);
    g += `<line class="agulha" x1="${C}" y1="${C}" x2="${x}" y2="${y}"/><circle class="ponta" cx="${x}" cy="${y}" r="7"/><circle cx="${C}" cy="${C}" r="6" fill="#3987e5"/>`;
  }
  $('bussola').innerHTML = g;
  $('az-lado').innerHTML = a.az != null
    ? `<div class="v">${a.az.toFixed(1)}<small>°</small></div><div class="d">recebido ${ago(a.azAt, n)}</div><div class="d">0° = luz de frente para a face FRONTAL; os pontos claros são os últimos 30 valores.</div>`
    : `<div class="v">—</div><div class="d">${a.busca ? 'busca ativa: o ADCS só manda com luz suficiente nos LDRs' : 'o OBC manda 3 quando o TT&C liga o bit Pointing (0x08)'}</div>`;
  const fact = (k, v, on) => `<div class="fact"><div class="k">${k}</div><div class="v ${on ? 'on' : ''}">${v}</div></div>`;
  $('adcs-est').innerHTML = fact('Detumbling (pedido pelo OBC)', a.detumbling ? 'ligado (1)' : 'desligado (2)', a.detumbling)
    + fact('Busca de azimute', a.busca ? 'ativa (3)' : 'parada (4)', a.busca)
    + fact('Último comando', a.ultimoCmd ? `${a.ultimoCmd.c} · ${ago(a.ultimoCmd.at, n)}` : '—')
    + fact('Comandos enviados', a.cmds)
    + fact('Estado do ADCS (SDBF!)', a.est ? `det ${a.est.det ? 1 : 0} · busca ${a.est.busca ? 1 : 0} · falha ${a.est.falha} · ${ago(a.estAt, n)}` : 'nenhum ainda', a.est && n - a.estAt < 6000 && !a.est.falha)
    + fact('Falhas seguidas / reinícios', `${a.falhas ?? 0}/5 · ${a.reinicios ?? 0} reinícios`, !(a.falhas) && !(a.reinicios));
  const u = a.usb;
  if (u.com) {
    $('adcs-est').innerHTML += fact(`USB do Nano (${u.com})`, u.connected ? (u.lastAt && n - u.lastAt < 15000 ? 'conectado' : 'conectado, em silêncio') : 'desconectado', u.connected)
      + fact('Comandos que o Nano confirmou', `${u.cmdsRecebidos}${u.ultimoCmd ? ` · último ${u.ultimoCmd.c} ${ago(u.ultimoCmd.at, n)}` : ''}`, u.cmdsRecebidos > 0)
      + fact('Detumbling no Nano', u.det == null ? '—' : u.det ? 'ATIVO' : 'parado', u.det)
      + fact('Rotação do corpo', u.omega == null ? '—' : `${u.omega.toFixed(1)} rpm`, u.omega != null && Math.abs(u.omega) < 5);
    if (u.erro) $('adcs-est').innerHTML += `<div class="fact" style="grid-column:1/-1"><div class="k">Erro informado pelo Nano</div><div class="v" style="color:#ff9b9b;font-size:14px">${esc(u.erro)}</div></div>`;
  }
  renderOmega();
  const h = [...a.azHist].reverse().slice(0, 12);
  $('az-hist').innerHTML = h.length ? h.map((x) => `<li class="ev tm"><span class="ts">${hhmmss(x.t)}</span><span class="mk"></span><span class="tx">${x.az.toFixed(1)}°</span></li>`).join('') : '<li class="empty">Nada ainda.</li>';
}

function renderOmega() {
  const u = S.adcs.usb, h = u.omegaHist;
  const W = 600, H = 200, L = 40, R = 10, T = 10, B = 24;
  $('omega-meta').textContent = u.faixaS != null ? `(abaixo de 5 rpm em ${u.faixaS.toFixed(2)} s; meta da prova < 10 s)` : '(USB do Nano, 10 Hz durante o detumbling)';
  if (h.length < 2) { $('omega-graf').innerHTML = `<text x="${W / 2}" y="${H / 2}" text-anchor="middle" fill="#8c8b83" font-size="13">${u.com ? 'aparece quando o detumbling estiver ativo' : 'rode o bridge com --adcs COMxx para ver o giroscópio'}</text>`; return; }
  const ws = h.map((p) => p.w), y1 = Math.max(10, ...ws.map(Math.abs)) * 1.1, y0 = Math.min(0, ...ws) * 1.1;
  const t0 = h[0].t, t1 = h.at(-1).t;
  const X = (t) => L + ((t - t0) / Math.max(1, t1 - t0)) * (W - L - R), Y = (v) => T + (1 - (v - y0) / (y1 - y0)) * (H - T - B);
  let g = '';
  for (const v of [0, 5, Math.round(y1 / 2), Math.round(y1 * 0.9)]) g += `<line class="grid" x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}"/><text class="gl" x="${L - 6}" y="${Y(v) + 4}" text-anchor="end">${v}</text>`;
  g += `<line class="corte" x1="${L}" x2="${W - R}" y1="${Y(5)}" y2="${Y(5)}"/><text class="gl" x="${W - R}" y="${Y(5) - 4}" text-anchor="end">5 rpm</text>`;
  g += `<path class="s" stroke="#3987e5" d="${h.map((p, i) => `${i ? 'L' : 'M'}${X(p.t).toFixed(1)},${Y(p.w).toFixed(1)}`).join('')}"/>`;
  g += `<text class="gl" x="${L}" y="${H - 6}">${hhmmss(t0)}</text><text class="gl" x="${W - R}" y="${H - 6}" text-anchor="end">${hhmmss(t1)} · rpm</text>`;
  $('omega-graf').innerHTML = g;
}

// ----------------------------------------------------------------- térmico
const CORES = ['#3987e5', '#d95926', '#199e70', '#c98500'];   // paleta categórica (dark), ordem fixa
const SIT = [[1, 'sem sonda'], [2, 'corte por temperatura alta'], [4, 'uma sonda falhou'], [8, 'sem calibração'],
  [16, 'leitura lenta'], [32, 'sonda discorda do outro bloco'], [64, 'aquecedor sem resposta']];
const sitTexto = (c) => (c ? SIT.filter(([b]) => c & b).map(([, t]) => t).join(', ') : 'normal');
function renderTermico() {
  const tc = S.payload.tcs;
  const n = S.now;
  $('tcs-meta').textContent = tc ? `trailer ${ago(tc.at, n)} · linha do TCS com ${tc.idade == null ? '—' : tc.idade.toFixed(1) + ' s'} · modo observação (aquecedores não acionados)` : 'aguardando o primeiro trailer (a cada 5 s)';
  const blocos = [['A', 0, [0, 1]], ['B', 1, [2, 3]]];
  $('tcs-blocos').innerHTML = blocos.map(([b, i, ss]) => {
    const sit = tc ? tc.sit[i] : 0;
    const cls = !tc ? '' : (sit & (1 | 32 | 64)) ? 'fail' : sit ? 'warn' : '';
    const stt = !tc ? 'pending' : (sit & (1 | 32 | 64)) ? 'fail' : sit ? 'warn' : 'ok';
    return `<div class="bloco ${cls}"><h3>Bloco ${b} ${st(stt)}</h3><div class="sondas">${ss.map((k) => {
      const v = tc ? tc.sondas[k] : null;
      return `<div class="sonda"><div class="k"><i style="display:inline-block;width:10px;height:3px;background:${CORES[k]};vertical-align:middle;margin-right:6px"></i>Sonda ${k + 1}</div><div class="v">${v == null ? '—' : v.toFixed(2)} <small>°C</small></div></div>`;
    }).join('')}</div>
      <div class="duty"><div class="row"><span>Aquecedor (decisão do LQI)</span><b>${tc ? tc.duty[i] : 0} %</b></div><div class="bar"><div style="width:${tc ? tc.duty[i] : 0}%"></div></div></div>
      <div class="sit">Situação ${sit}: ${sitTexto(sit)}</div></div>`;
  }).join('');
  renderTGraf();
}
function renderTGraf() {
  const h = S.payload.tcsHist ?? [];
  const W = 900, H = 260, L = 44, R = 12, T = 12, B = 28;
  if (h.length < 2) { $('tcs-graf').innerHTML = `<text x="${W / 2}" y="${H / 2}" text-anchor="middle" fill="#8c8b83" font-size="13">o gráfico começa no 2º trailer</text>`; $('tcs-leg').innerHTML = ''; return; }
  const vals = h.flatMap((p) => p.s.filter((x) => x != null));
  let y0 = Math.min(18, ...vals) - 1, y1 = Math.max(31, ...vals) + 1;
  const t0 = h[0].t, t1 = h.at(-1).t;
  const X = (t) => L + ((t - t0) / Math.max(1, t1 - t0)) * (W - L - R), Y = (v) => T + (1 - (v - y0) / (y1 - y0)) * (H - T - B);
  let g = '';
  for (let v = Math.ceil(y0 / 5) * 5; v <= y1; v += 5) g += `<line class="grid" x1="${L}" x2="${W - R}" y1="${Y(v)}" y2="${Y(v)}"/><text class="gl" x="${L - 6}" y="${Y(v) + 4}" text-anchor="end">${v}°</text>`;
  g += `<line class="ref" x1="${L}" x2="${W - R}" y1="${Y(20)}" y2="${Y(20)}"/><line class="corte" x1="${L}" x2="${W - R}" y1="${Y(30)}" y2="${Y(30)}"/>`;
  g += `<text class="gl" x="${W - R}" y="${Y(30) - 4}" text-anchor="end">corte 30 °C</text><text class="gl" x="${W - R}" y="${Y(20) - 4}" text-anchor="end">referência 20 °C</text>`;
  g += `<text class="gl" x="${L}" y="${H - 8}">${hhmmss(t0)}</text><text class="gl" x="${W - R}" y="${H - 8}" text-anchor="end">${hhmmss(t1)}</text>`;
  for (let k = 0; k < 4; k++) {
    let d = '', pen = false;
    for (const p of h) { const v = p.s[k]; if (v == null) { pen = false; continue; } d += `${pen ? 'L' : 'M'}${X(p.t).toFixed(1)},${Y(v).toFixed(1)}`; pen = true; }
    if (d) g += `<path class="s" d="${d}" stroke="${CORES[k]}"/>`;
  }
  g += `<line id="tg-x" x1="0" x2="0" y1="${T}" y2="${H - B}" stroke="#8c8b83" visibility="hidden"/><rect id="tg-hit" x="${L}" y="${T}" width="${W - L - R}" height="${H - T - B}" fill="transparent"/>`;
  $('tcs-graf').innerHTML = g;
  const last = h.at(-1).s;
  $('tcs-leg').innerHTML = [0, 1, 2, 3].map((k) => `<span><i style="background:${CORES[k]}"></i>Sonda ${k + 1} (bloco ${k < 2 ? 'A' : 'B'}) <b>${last[k] == null ? '—' : last[k].toFixed(2) + ' °C'}</b></span>`).join('') + '<span id="tg-tip" class="hint"></span>';
  const svg = $('tcs-graf');
  $('tg-hit').addEventListener('pointermove', (e) => {
    const r = svg.getBoundingClientRect();
    const px = ((e.clientX - r.left) / r.width) * W;
    const p = h.reduce((a, b) => (Math.abs(X(b.t) - px) < Math.abs(X(a.t) - px) ? b : a));
    const x = $('tg-x'); x.setAttribute('x1', X(p.t)); x.setAttribute('x2', X(p.t)); x.setAttribute('visibility', 'visible');
    $('tg-tip').textContent = `${hhmmss(p.t)} · ` + p.s.map((v, k) => `S${k + 1} ${v == null ? '—' : v.toFixed(2)} °C`).join(' · ');
  });
  $('tg-hit').addEventListener('pointerleave', () => { $('tg-x').setAttribute('visibility', 'hidden'); $('tg-tip').textContent = ''; });
}

// ------------------------------------------------------ aviso sonoro (avião)
let audio = null;
// Aviso sonoro LIGADO por padrão (a escolha fica salva no navegador).
let somLigado = localStorage.getItem('somAviao') !== 'off';
let conhecidos = null;          // ICAOs já vistos quando o painel abriu (não avisa o passado)
let validosAntes = null, blocoAntes = null;
const tituloBase = document.title;
// Som do aviso: se existir web/sons/aviao.mp3 (ou .ogg/.wav), toca esse arquivo (coloque ali o áudio que
// quiser, de uso seu); senão, toca uma fanfarra curta de vitória composta aqui (original).
const SONS = ['/sons/aviao.mp3', '/sons/aviao.ogg', '/sons/aviao.wav'];
let somArquivo = null;
(async () => {
  for (const u of SONS) {
    try { const r = await fetch(u, { method: 'HEAD' }); if (r.ok) { somArquivo = u; break; } } catch {}
  }
})();
let tocando = null;
function fanfarra() {
  if (!audio) return;
  const t0 = audio.currentTime + 0.02;
  // [frequência (Hz), início (s), duração (s)]: arpejo ascendente em Dó maior e acorde final sustentado
  const notas = [[523.25, 0, 0.16], [659.25, 0.16, 0.16], [783.99, 0.32, 0.16], [1046.5, 0.48, 0.42],
    [783.99, 0.95, 0.14], [1046.5, 1.1, 0.8], [659.25, 1.1, 0.8], [783.99, 1.1, 0.8]];
  for (const [f, ini, dur] of notas) {
    const o = audio.createOscillator(), g = audio.createGain();
    o.type = 'triangle'; o.frequency.value = f;
    g.gain.setValueAtTime(0.0001, t0 + ini);
    g.gain.exponentialRampToValueAtTime(0.25, t0 + ini + 0.03);
    g.gain.setValueAtTime(0.25, t0 + ini + dur * 0.7);
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + ini + dur);
    o.connect(g).connect(audio.destination); o.start(t0 + ini); o.stop(t0 + ini + dur + 0.02);
  }
}
function tocar() {
  if (!audio || audio.state !== 'running') liberarAudio();
  if (somArquivo) {
    if (tocando) { tocando.pause(); }
    tocando = new Audio(somArquivo);
    tocando.play().catch(() => fanfarra());
    return;
  }
  fanfarra();
}
function avisar(titulo, detalhe) {
  if (somLigado) tocar();
  const a = $('alerta');
  a.hidden = false;
  a.innerHTML = `<button class="x" aria-label="fechar">×</button><b>✈ ${esc(titulo)}</b><div class="d">${esc(detalhe)}</div><div class="d">${new Date().toLocaleTimeString('pt-BR')} · <a href="#payload">ver na aba Payload →</a></div>`;
  a.querySelector('.x').onclick = () => { a.hidden = true; };
  document.title = `✈ ${titulo} · ${tituloBase}`;
  if (window.Notification && Notification.permission === 'granted' && document.hidden) new Notification(`✈ ${titulo}`, { body: detalhe });
}
window.addEventListener('focus', () => { document.title = tituloBase; });
// O navegador só libera áudio depois de um gesto do usuário, a menos que a janela tenha sido aberta com
// --autoplay-policy=no-user-gesture-required. Tenta na carga e destrava no 1º clique/tecla em qualquer lugar.
function botaoSom() {
  const bloqueado = somLigado && (!audio || audio.state !== 'running');
  $('som-btn').classList.toggle('on', somLigado && !bloqueado);
  $('som-btn').textContent = !somLigado ? '🔕 Aviso sonoro desligado' : bloqueado ? '🔇 Clique na página para liberar o som' : '🔔 Aviso sonoro ligado';
}
async function liberarAudio() {
  try {
    if (!audio) audio = new (window.AudioContext || window.webkitAudioContext)();
    if (audio.state !== 'running') await audio.resume();
  } catch {}
  botaoSom();
}
liberarAudio();
['pointerdown', 'keydown'].forEach((ev) => window.addEventListener(ev, liberarAudio, { once: false, passive: true }));
setInterval(botaoSom, 2000);
$('som-btn').onclick = async () => {
  await liberarAudio();
  somLigado = !somLigado;
  localStorage.setItem('somAviao', somLigado ? 'on' : 'off');
  botaoSom();
  if (somLigado) { tocar(); if (window.Notification && Notification.permission === 'default') Notification.requestPermission(); }
};
$('som-teste').onclick = async () => {
  if (!audio) audio = new (window.AudioContext || window.webkitAudioContext)();
  await audio.resume();
  tocar();
};
function checarAvioes() {
  const pl = S.payload;
  const vistos = pl.icaosVistos ?? {};
  if (conhecidos === null) { conhecidos = new Set(Object.keys(vistos)); validosAntes = pl.lastBlock?.validos ?? 0; blocoAntes = pl.blocks; return; }
  const novos = Object.keys(vistos).filter((k) => !conhecidos.has(k));
  if (novos.length) {
    novos.forEach((k) => conhecidos.add(k));
    const p = pl.planes.find((x) => x.icao === novos[0]);
    avisar(novos.length > 1 ? `${novos.length} aviões novos` : `Avião ${novos[0]}`,
      p ? `${p.alt != null ? Math.round(p.alt) + ' ft' : 'altitude ?'} · ${p.gs != null ? Math.round(p.gs) + ' kt' : 'velocidade ?'} · ${p.lat != null ? p.lat.toFixed(3) + ', ' + p.lon.toFixed(3) : 'sem posição ainda'}` : novos.join(', '));
  } else if (pl.blocks !== blocoAntes && (pl.lastBlock?.validos ?? 0) > 0 && validosAntes === 0) {
    avisar(`${pl.lastBlock.validos} avião(ões) no bloco do Pi`, `bloco #${pl.blocks}; detalhes chegam no próximo quadro 'P'`);
  }
  if (pl.blocks !== blocoAntes) { validosAntes = pl.lastBlock?.validos ?? 0; blocoAntes = pl.blocks; }
}

// ---------------------------------------------------------------- ligações
function renderPins() {
  const rows = wires();
  $('pins').innerHTML = `<tr><th>Nucleo-L476RG</th><th></th><th>Liga em</th><th>Sinal</th><th>Status agora</th></tr>` + rows.map((r) => r.sec
    ? `<tr class="sec"><td colspan="5">${esc(r.sec)}</td></tr>`
    : `<tr><td class="pin-a">${esc(r.a)}<small>${esc(r.aSub)}</small></td><td class="arrow">${r.dir}</td>
        <td class="pin-b"><b>${esc(r.b)}</b><small>${esc(r.bSub)}</small></td>
        <td><span class="swatch" style="background:${r.color}"></span><span class="sig">${esc(r.sig)}</span></td>
        <td><div class="state">${st(r.st)}<div><b>${LABEL[r.st]}</b><div class="d">${esc(r.txt)}</div></div></div></td></tr>`).join('');

  // diagrama
  const w = Object.fromEntries(rows.filter((r) => !r.sec).map((r) => [r.a, r]));
  const mark = (x, y, s) => {
    const fill = { ok: '#0ca30c', fail: '#d03b3b', warn: '#fab219' }[s];
    return fill ? `<circle cx="${x}" cy="${y}" r="11" fill="${fill}"/><text x="${x}" y="${y + 4.5}" text-anchor="middle" font-size="13" font-weight="800" fill="${s === 'fail' ? '#fff' : '#111'}">${ICON[s]}</text>`
      : `<circle cx="${x}" cy="${y}" r="10" fill="#0f1012" stroke="#8c8b83" stroke-width="1.5"/><text x="${x}" y="${y + 4.5}" text-anchor="middle" font-size="12" font-weight="700" fill="#8c8b83">${ICON[s]}</text>`;
  };
  const pinL = (y, l, sub) => `<circle class="pin" cx="290" cy="${y}" r="6"/><text class="plabel" x="276" y="${y - 2}" text-anchor="end">${l}</text><text class="psub" x="276" y="${y + 12}" text-anchor="end">${sub}</text>`;
  const pinR = (y, l, sub) => `<circle class="pin" cx="610" cy="${y}" r="6"/><text class="plabel" x="624" y="${y - 2}">${l}</text><text class="psub" x="624" y="${y + 12}">${sub}</text>`;
  const wire = (y1, y2, r, label, arrow) => `<path class="wire" d="M296 ${y1} C 450 ${y1}, 450 ${y2}, 604 ${y2}" stroke="${r.color}" ${arrow ? `marker-end="url(#w-${arrow})"` : ''}/>
    <text class="wlabel" x="530" y="${(y1 + y2) / 2 - 9}" text-anchor="middle" fill="${r.color}">${label}</text>${mark(370, (y1 + y2) / 2, r.st)}`;
  $('wiring').innerHTML = `<defs>
      <marker id="w-r" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0 0L10 5L0 10z" fill="context-stroke"/></marker></defs>
    <rect class="board" x="30" y="20" width="266" height="325" rx="14"/>
    <text class="bname" x="50" y="50">NUCLEO-L476RG</text><text class="bsub" x="50" y="68">OBC · máquina de estados</text>
    <rect class="board" x="604" y="20" width="266" height="210" rx="14"/>
    <text class="bname" x="850" y="50" text-anchor="end">ESP32</text><text class="bsub" x="850" y="68" text-anchor="end">TT&amp;C simulado</text>
    <rect class="board" x="604" y="250" width="266" height="95" rx="14"/>
    <text class="bname" x="850" y="276" text-anchor="end">Sensores I2C</text><text class="bsub" x="850" y="293" text-anchor="end">BME280 · DS3231</text>
    ${pinL(95, 'D8', 'PA9 · TX')}${pinL(140, 'D2', 'PA10 · RX')}${pinL(185, '5V', '')}${pinL(220, 'GND', '')}${pinL(290, 'D15', 'PB8 · SCL')}${pinL(325, 'D14', 'PB9 · SDA')}
    ${pinR(95, 'GPIO16', 'RX2')}${pinR(140, 'GPIO17', 'TX2')}${pinR(185, 'VIN', '5V')}${pinR(220, 'GND', '')}${pinR(305, 'SCL', '')}${pinR(330, 'SDA', '')}
    ${wire(95, 95, w.D8, 'telemetria →', 'r')}
    <path class="wire" d="M604 140 C 450 140, 450 140, 296 140" stroke="${w.D2.color}" marker-end="url(#w-r)"/>
    <text class="wlabel" x="530" y="131" text-anchor="middle" fill="${w.D2.color}">← telecomandos</text>${mark(370, 140, w.D2.st)}
    ${wire(185, 185, w['5V'], 'alimentação', '')}${wire(220, 220, w.GND, 'GND comum', '')}
    ${wire(290, 305, w.D15, 'SCL', '')}${wire(325, 330, w.D14, '', '')}`;
}

// -------------------------------------------------------------- log técnico
let rawN = 0;
function addRaw(e) {
  const box = $('raw');
  const div = document.createElement('div');
  div.className = `rl ${e.kind}`;
  div.innerHTML = `<span class="ts">${hhmmss(e.t)}</span>${esc(e.text)}`;
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
  box.appendChild(div);
  while (box.childElementCount > 1500) box.firstChild.remove();
  if (atBottom) box.scrollTop = box.scrollHeight;
  $('rawcount').textContent = `USART2 crua · ${++rawN} linhas`;
}

// ---------------------------------------------------------------------- SSE
function render() {
  renderTop(); renderSummary(); renderStateMachine(); renderFeed(); renderTasks();
  renderTtc(); renderPayload(); renderAdcs(); renderTermico(); renderTelemetry(); renderPins();
  checarAvioes();
  $('foot').textContent = `Painel aberto às ${new Date(S.startedAt).toLocaleTimeString('pt-BR')} · logs da sessão em ${S.logDir}`;
}
const es = new EventSource('/events');
es.addEventListener('state', (m) => { S = JSON.parse(m.data); render(); });
es.addEventListener('rawlog', (m) => { $('raw').innerHTML = ''; rawN = 0; JSON.parse(m.data).forEach(addRaw); });
es.addEventListener('raw', (m) => addRaw(JSON.parse(m.data)));
es.onerror = () => { $('chips').innerHTML = '<span class="chip"><span class="dot off"></span>sem conexão com o bridge, tentando de novo…</span>'; };
