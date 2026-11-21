// Conhecimento sobre o firmware da máquina de estados (branch code_state_machine + variante de bancada):
// layout da telemetria, mensagens de debug, roteiro do TT&C simulado e checklist.
// Se o firmware ou o roteiro do ESP32 mudarem, é aqui que se ajusta.

export const MODES = ['IDLE', 'MISSION', 'DETUMBLING', 'POINTING']; // SatMode_t (customTypes.h)
export const MODE_INFO = {
  IDLE: { titulo: 'Repouso', texto: 'Só Main, Housekeeping e Receive ativas, esperando comandos.' },
  MISSION: { titulo: 'Missão ADS-B', texto: 'Task Mission ativa (ainda é um esqueleto: roda e termina).' },
  DETUMBLING: { titulo: 'Detumbling', texto: 'ADCS reduzindo a rotação (task esqueleto: roda e termina).' },
  POINTING: { titulo: 'Apontamento', texto: 'Busca de azimute. Ainda não tem task: só muda o modo.' },
};

export const TC_BITS = [
  { bit: 0x01, name: 'Missão' },
  { bit: 0x02, name: 'Deploy' },
  { bit: 0x04, name: 'Detumbling' },
  { bit: 0x08, name: 'Pointing' },
];
export const describeTc = (v) => ((v & ~0x0f) ? 'inválido' : TC_BITS.filter((b) => v & b.bit).map((b) => b.name).join(' + ') || 'tudo desligado');

export const TASKS = [
  { id: 'main', name: 'Main', papel: 'gerencia a máquina de estados', prio: 'Normal' },
  { id: 'housekeep', name: 'Housekeeping', papel: 'lê os sensores a cada 60 s', prio: 'Low' },
  { id: 'transmit', name: 'Transmit', papel: 'envia a telemetria ao TT&C', prio: 'Low' },
  { id: 'receive', name: 'Receive', papel: 'trata erros da UART do TT&C', prio: 'Low' },
  { id: 'deploy', name: 'Deploy', papel: 'abre a antena (esqueleto)', prio: 'Low' },
  { id: 'detumbling', name: 'Detumbling', papel: 'liga o ADCS (esqueleto)', prio: 'Low' },
  { id: 'mission', name: 'Mission', papel: 'liga a carga útil (esqueleto)', prio: 'Low' },
];

// ------------------------------------------------------------ roteiro do TT&C
// Igual ao ROTEIRO de esp32_ttc_mock.ino. "expect" segue o handleTelecommand() de main.c, partindo
// do estado deixado pelo passo anterior. Deploy: na 1ª vez executa; depois deve ser ignorado.
export const TTC_STEP_SECONDS = 15;
export const TTC_SCRIPT = [
  { tc: 0x00, desc: 'Solo: tudo desligado', expect: [], info: 'nada a mudar' },
  { tc: 0x04, desc: 'Liga detumbling', expect: [/detumbling ON/, /detumbling task ran/, /detumbling done, mode=0/] },
  { tc: 0x00, desc: 'Desliga tudo', expect: [], info: 'o detumbling já tinha terminado (task esqueleto)' },
  { tc: 0x08, desc: 'Liga pointing (azimute)', expect: [/pointing ON/] },
  { tc: 0x00, desc: 'Desliga pointing', expect: [/pointing STOP/] },
  { tc: 0x01, desc: 'Liga missão ADS-B', expect: [/mission ON/, /mission task ran/, /mission done, mode=0/] },
  { tc: 0x00, desc: 'Desliga tudo', expect: [], info: 'a missão já tinha terminado (task esqueleto)' },
  { tc: 0x0c, desc: 'Detumbling + pointing: detumbling tem prioridade', expect: [/detumbling ON/, /detumbling done/], forbid: [/pointing ON/] },
  { tc: 0x09, desc: 'Pointing + missão ao mesmo tempo', expect: [/pointing ON/, /mission ON/, /mission task ran/] },
  { tc: 0x00, desc: 'Desliga tudo', expect: [/pointing STOP/] },
  { tc: 0x02, desc: 'Deploy da antena', deploy: true },
];
export function stepExpectation(step, antennaDeployedBefore) {
  if (!step.deploy) return step;
  return antennaDeployedBefore
    ? { ...step, expect: [], forbid: [/rx accepted: deploy/], info: 'antena já implantada: o OBC deve ignorar' }
    : { ...step, expect: [/rx accepted: deploy/, /deploy task ran/, /deploy done/] };
}

// ---------------------------------------------------------------- telemetria
// struct Telemetry (customTypes.h), packed, little-endian, 85 bytes.
export const TELEMETRY_SIZE = 85;
const FIELDS = [
  ['hora', 'u8'], ['minuto', 'u8'], ['segundo', 'u8'], ['dia', 'u8'], ['mes', 'u8'], ['ano', 'u8'],
  ['temperatura_bateria', 'f32'], ['temperatura_estrutura', 'f32'], ['umidade', 'f32'], ['pressao', 'f32'],
  ['deploy_antena', 'u8'],
  ['tensao_13v8', 'f32'], ['tensao_5v', 'f32'], ['corrente_13v8', 'f32'], ['corrente_5v', 'f32'],
  ['tensao_painel_solar_1', 'f32'], ['tensao_painel_solar_2', 'f32'],
  ['tensao_painel_solar_3', 'f32'], ['tensao_painel_solar_4', 'f32'],
  ['status_carregador_1', 'u8'], ['status_carregador_2', 'u8'], ['falha_carregador_1', 'u8'], ['falha_carregador_2', 'u8'],
  ['adc_tensao_sistema_1', 'u8'], ['adc_tensao_sistema_2', 'u8'], ['adc_tensao_ntc_1', 'u8'], ['adc_tensao_ntc_2', 'u8'],
  ['adc_corrente_carga_1', 'u8'], ['adc_corrente_carga_2', 'u8'], ['adc_corrente_sistema_1', 'u8'], ['adc_corrente_sistema_2', 'u8'],
  ['angulo_apontamento', 'str6'],
  ['tensao_bateria_1', 'u16'], ['tensao_bateria_2', 'u16'], ['carga_bateria_1', 'u16'], ['carga_bateria_2', 'u16'],
  ['taxa_carga_descarga_bateria_1', 'i16'], ['taxa_carga_descarga_bateria_2', 'i16'],
];
const SIZES = { u8: 1, u16: 2, i16: 2, f32: 4, str6: 6 };
export const TELEMETRY_FIELDS = FIELDS.map(([name]) => name);

export const TELEMETRY_GROUPS = [
  { name: 'Relógio do OBC', sensor: 'DS3231', fields: ['hora', 'minuto', 'segundo', 'dia', 'mes', 'ano'] },
  { name: 'Ambiente e mecânica', sensor: 'BME280', fields: ['temperatura_bateria', 'temperatura_estrutura', 'umidade', 'pressao', 'deploy_antena'] },
  { name: 'Barramentos 13,8 V e 5 V', sensor: 'INA3221', fields: ['tensao_13v8', 'tensao_5v', 'corrente_13v8', 'corrente_5v'] },
  { name: 'Painéis solares', sensor: 'INA3221', fields: ['tensao_painel_solar_1', 'tensao_painel_solar_2', 'tensao_painel_solar_3', 'tensao_painel_solar_4'] },
  { name: 'Carregadores', sensor: 'MP2733', fields: ['status_carregador_1', 'status_carregador_2', 'falha_carregador_1', 'falha_carregador_2',
    'adc_tensao_sistema_1', 'adc_tensao_sistema_2', 'adc_tensao_ntc_1', 'adc_tensao_ntc_2',
    'adc_corrente_carga_1', 'adc_corrente_carga_2', 'adc_corrente_sistema_1', 'adc_corrente_sistema_2'] },
  { name: 'Atitude', sensor: 'ADCS', fields: ['angulo_apontamento'] },
  { name: 'Baterias', sensor: 'MAX17048', fields: ['tensao_bateria_1', 'tensao_bateria_2', 'carga_bateria_1', 'carga_bateria_2',
    'taxa_carga_descarga_bateria_1', 'taxa_carga_descarga_bateria_2'] },
];

export function decodeTelemetry(buf) {
  if (buf.length !== TELEMETRY_SIZE) return null;
  const out = {};
  let o = 0;
  for (const [name, type] of FIELDS) {
    switch (type) {
      case 'u8': out[name] = buf.readUInt8(o); break;
      case 'u16': out[name] = buf.readUInt16LE(o); break;
      case 'i16': out[name] = buf.readInt16LE(o); break;
      case 'f32': out[name] = round(buf.readFloatLE(o)); break;
      case 'str6': out[name] = buf.subarray(o, o + 6).toString('latin1').replace(/\0.*$/s, ''); break;
    }
    o += SIZES[type];
  }
  return out;
}
const round = (v) => (Number.isFinite(v) ? Math.round(v * 1000) / 1000 : String(v));

// ------------------------------------------------------------ debug (USART2)
// Linhas "[%08lu] texto". Cada regra: efeito no estado inferido + frase em português para a linha do tempo.
export const DEBUG_RULES = [
  { re: /main ready, mode=(\d+)/, kind: 'boot', task: 'main', say: () => 'OBC ligou e está pronto, em repouso (IDLE)', tone: 'good' },
  { re: /rx tc=0x([0-9A-F]{2})/i, kind: 'tc', task: 'main', say: (m) => `Comando do TT&C recebido: 0x${m[1].toUpperCase()} (${describeTc(parseInt(m[1], 16))})`, tone: 'tc' },
  { re: /rx rejected: invalid mask 0x([0-9A-F]{2})/i, kind: 'rx', task: 'main', say: (m) => `Comando 0x${m[1]} rejeitado: bits inválidos`, tone: 'warn' },
  { re: /rx ignored: deploy in progress/, kind: 'rx', task: 'main', say: () => 'Comando ignorado: deploy em andamento', tone: 'warn' },
  { re: /rx accepted: deploy/, kind: 'rx', task: 'main', set: { deployInProgress: true }, say: () => 'Deploy da antena iniciado', tone: 'state' },
  { re: /rx accepted: detumbling ON/, kind: 'rx', task: 'main', set: { mode: 2 }, say: () => 'Modo → DETUMBLING', tone: 'state' },
  { re: /rx accepted: detumbling STOP/, kind: 'rx', task: 'main', set: { mode: 0 }, say: () => 'Detumbling parado → IDLE', tone: 'state' },
  { re: /rx accepted: pointing ON/, kind: 'rx', task: 'main', set: { mode: 3 }, say: () => 'Modo → POINTING', tone: 'state' },
  { re: /rx accepted: pointing STOP/, kind: 'rx', task: 'main', set: { mode: 0 }, say: () => 'Pointing parado → IDLE', tone: 'state' },
  { re: /rx accepted: mission ON/, kind: 'rx', task: 'main', fn: (o) => { o.missionActive = true; if (o.mode === 0) o.mode = 1; }, say: () => 'Missão ligada', tone: 'state' },
  { re: /rx accepted: mission OFF/, kind: 'rx', task: 'main', fn: (o) => { o.missionActive = false; if (o.mode === 1) o.mode = 0; }, say: () => 'Missão desligada', tone: 'state' },
  // Tasks esqueleto: imprimem e já mandam o *_DONE. No EVT_MISSION_DONE o firmware zera missionActive
  // sempre, mas só imprime "mission done" se o modo era MISSION.
  { re: /mission task ran/, kind: 'task', task: 'mission', set: { missionActive: false }, say: () => 'Task Mission rodou e terminou', tone: 'task' },
  { re: /detumbling task ran/, kind: 'task', task: 'detumbling', say: () => 'Task Detumbling rodou e terminou', tone: 'task' },
  { re: /deploy task ran/, kind: 'task', task: 'deploy', say: () => 'Task Deploy rodou', tone: 'task' },
  { re: /mission done, mode=(\d+)/, kind: 'done', task: 'main', mode: 1, say: () => 'Missão concluída → IDLE', tone: 'state' },
  { re: /detumbling done, mode=(\d+)/, kind: 'done', task: 'main', mode: 1, say: () => 'Detumbling concluído → IDLE', tone: 'state' },
  { re: /pointing done, mode=(\d+)/, kind: 'done', task: 'main', mode: 1, say: () => 'Pointing concluído → IDLE', tone: 'state' },
  { re: /deploy done/, kind: 'done', task: 'main', set: { deployInProgress: false, antennaDeployed: true }, say: () => 'Antena implantada (deploy concluído)', tone: 'good' },
  { re: /tx done/, kind: 'tx', task: 'transmit', say: () => 'Telemetria enviada ao TT&C', tone: 'muted' },
  { re: /tx queue error/, kind: 'error', task: 'main', say: () => 'Erro: fila de transmissão cheia', tone: 'bad' },
  { re: /usart1 error=0x([0-9A-F]+) count=(\d+)/i, kind: 'uart_error', task: 'receive', say: (m) => `Erro na UART do TT&C (código 0x${m[1]}, ${m[2]} no total)`, tone: 'bad' },
  { re: /unknown event=(\d+)/, kind: 'error', task: 'main', say: (m) => `Evento desconhecido ${m[1]} na fila`, tone: 'bad' },
];

// ------------------------------------------------------------------ checklist
// Status: ok | warn | fail | pending. Cada item diz o que significa e o que fazer.
const inRange = (v, lo, hi) => typeof v === 'number' && v >= lo && v <= hi;

export function computeChecks(s, now) {
  const out = [];
  const add = (group, label, status, detail, fix) => out.push({ group, label, status, detail, fix });
  const secs = (t) => Math.round((now - t) / 1000);
  const tm = s.telemetry;
  const last = tm.last;
  const o = s.obc;

  // Conexão
  if (!s.link.connected) add('Conexão', 'STM32 conectado ao PC', 'fail', `${s.link.com} não está aberta`, 'Confira o cabo USB do Nucleo e se a COM não está aberta em outro programa (PuTTY, Arduino IDE).');
  else add('Conexão', 'STM32 conectado ao PC', 'ok', `${s.link.com}, ${s.link.lines} mensagens lidas`);

  const lastAlive = Math.max(o.lastBootAt ?? 0, s.tasks.transmit.lastAt ?? 0, tm.lastAt ?? 0, s.link.lastLineAt ?? 0);
  if (!lastAlive) add('Conexão', 'OBC dando sinal de vida', 'pending', 'nenhuma mensagem ainda', 'Aperte o RESET (botão preto) do Nucleo.');
  else if (now - lastAlive > 75000) add('Conexão', 'OBC dando sinal de vida', 'fail', `silêncio há ${secs(lastAlive)} s (o housekeeping deveria aparecer a cada 60 s)`, 'O OBC pode ter travado (HardFault ou Error_Handler). Aperte o RESET e veja o que aconteceu antes no log técnico.');
  else add('Conexão', 'OBC dando sinal de vida', 'ok', `última mensagem há ${secs(lastAlive)} s`);

  const original = s.firmware.build?.variant === 'original' && !s.firmware.tcVisivel;
  if (original) add('Conexão', 'Firmware', 'ok', `código real da branch (${s.firmware.build.commit}), sem nenhuma alteração`);
  else if (s.firmware.bancada === true) add('Conexão', 'Firmware', 'ok', 'variante de bancada (telemetria e "rx tc" na USART2)');
  else add('Conexão', 'Firmware', 'pending', 'aguardando a 1ª telemetria ou o 1º comando', '');

  // Máquina de estados
  if (o.boots) add('Máquina de estados', 'Inicialização', 'ok', `"main ready" visto ${o.boots}x`);
  else add('Máquina de estados', 'Inicialização', 'pending', 'o boot aconteceu antes do painel abrir', 'Aperte o RESET do Nucleo para ver a inicialização.');

  const p = s.housekeepPeriods;
  if (p.length) {
    const avg = p.reduce((a, b) => a + b, 0) / p.length;
    add('Máquina de estados', 'Housekeeping a cada 60 s', inRange(avg, 55, 65) ? 'ok' : 'warn', `média de ${avg.toFixed(1)} s em ${p.length} ciclos`);
  } else add('Máquina de estados', 'Housekeeping a cada 60 s', 'pending', 'precisa de 2 ciclos (até 2 min)', '');

  // Enlace com o TT&C (ESP32)
  const tcs = s.tcsReceived;
  const resp = original ? s.respostas : tcs;
  const lastResp = original ? s.lastRespAt : s.lastTcAt;
  const limite = original ? 60000 : 25000; // no original, comandos que não mudam nada não aparecem
  if (resp) add('TT&C simulado (ESP32)', 'Comandos do TT&C chegando ao OBC', 'ok',
    original ? `${resp} respostas do OBC a comandos; última há ${secs(lastResp)} s` : `${resp} comandos recebidos; último há ${secs(lastResp)} s`);
  else if (now - Math.max(s.startedAt, o.lastBootAt ?? 0) > limite && s.link.connected) add('TT&C simulado (ESP32)', 'Comandos do TT&C chegando ao OBC', 'fail',
    `nenhuma reação a comandos em ${Math.round((now - Math.max(s.startedAt, o.lastBootAt ?? 0)) / 1000)} s (o ESP32 manda um a cada ${TTC_STEP_SECONDS} s; no máximo 30 s entre comandos que mudam algo)`,
    'Ligue ESP32 GPIO17 (TX2) → Nucleo D2 (PA10). D9 não é UART. GND comum e ESP32 alimentado (VIN ← 5V do Nucleo).');
  else add('TT&C simulado (ESP32)', 'Comandos do TT&C chegando ao OBC', 'pending', original ? 'aguardando a 1ª reação do OBC (até 30 s)' : `aguardando (um a cada ${TTC_STEP_SECONDS} s)`, '');

  const steps = s.script.done;
  if (steps.length) {
    const bad = steps.filter((x) => x.status === 'fail');
    add('TT&C simulado (ESP32)', 'OBC responde certo aos comandos', bad.length ? 'fail' : 'ok',
      bad.length ? `${bad.length} de ${steps.length} comandos com resposta errada (veja o roteiro)` : `${steps.length} comandos conferidos`);
  }
  for (const t of ['detumbling', 'mission', 'deploy']) {
    if (s.tasks[t].count) add('TT&C simulado (ESP32)', `Task ${s.tasks[t].name} acionada por comando`, 'ok', `${s.tasks[t].count}x (ainda é esqueleto: só imprime e termina)`);
  }

  // Telemetria e sensores
  if (tm.count) add('Telemetria e sensores', 'Telemetria completa (85 B)', tm.partial ? 'warn' : 'ok', `${tm.count} quadros${tm.partial ? `, ${tm.partial} incompletos` : ''}`);
  else if (original && s.tasks.transmit.count) add('Telemetria e sensores', 'Telemetria enviada ao TT&C', 'ok', `${s.tasks.transmit.count}x ("tx done"). No firmware original o pacote só sai pela USART1, para o ESP32; o PC não vê o conteúdo.`);
  else if (s.tasks.transmit.count) add('Telemetria e sensores', 'Telemetria completa (85 B)', 'fail', 'o OBC envia ("tx done"), mas o quadro não aparece na USB', 'Grave a variante da bancada: stm32/build_flash.sh.');
  else add('Telemetria e sensores', 'Telemetria enviada ao TT&C', 'pending', 'a 1ª sai 60 s após o boot', '');
  if (!s.config.sensores) add('Telemetria e sensores', 'Sensores BME280 e DS3231', 'info', 'não instalados nesta bancada: os campos de relógio e ambiente vão com zeros ou lixo, e isso é esperado aqui');

  if (last && s.config.sensores) {
    const timeOk = inRange(last.hora, 0, 23) && inRange(last.minuto, 0, 59) && inRange(last.segundo, 0, 59);
    // Compara o avanço do relógio com o tick do OBC entre pacotes (o PC pode receber vários pacotes juntos).
    const h = tm.history.filter((x) => x.tick != null);
    let advancing = null;
    if (h.length >= 2) {
      const a = h[h.length - 2], b = h[h.length - 1];
      advancing = Math.abs(((b.clock - a.clock + 86400) % 86400) - (b.tick - a.tick) / 1000) < 5;
    }
    let rtcOk = false;
    if (!timeOk) add('Telemetria e sensores', 'Relógio DS3231', 'fail', `hora inválida (${last.hora}:${last.minuto}:${last.segundo})`, 'DS3231 sem resposta no I2C1 (D15/D14 = PB8/PB9)? Confira ligação e alimentação.');
    else if (advancing === false) add('Telemetria e sensores', 'Relógio DS3231', 'fail', `parado em ${String(last.hora).padStart(2, '0')}:${String(last.minuto).padStart(2, '0')}:${String(last.segundo).padStart(2, '0')}: o relógio não avança entre pacotes`, 'DS3231 sem resposta no I2C1 (D15/D14 = PB8/PB9): o firmware não confere o retorno do I2C e manda lixo. Confira ligação e alimentação do módulo.');
    else { rtcOk = !!advancing; add('Telemetria e sensores', 'Relógio DS3231', advancing ? 'ok' : 'pending', advancing ? 'avançando junto com o OBC' : 'precisa de 2 pacotes', ''); }
    if (rtcOk && !inRange(last.mes, 1, 12)) add('Telemetria e sensores', 'Data do relógio', 'warn', `dia ${last.dia}, mês ${last.mes}, ano ${last.ano}`, 'Bug conhecido: Set_Time(0,…) no boot grava dia e mês 0, e o campo "dia" recebe o dia da semana.');

    const bmeOk = inRange(last.temperatura_bateria, -40, 85) && inRange(last.pressao, 30000, 110000) && inRange(last.umidade, 0, 100);
    add('Telemetria e sensores', 'Sensor BME280', bmeOk ? 'ok' : 'fail',
      bmeOk ? `${last.temperatura_bateria.toFixed(1)} °C, ${(last.pressao / 100).toFixed(0)} hPa, ${last.umidade.toFixed(0)} %` : 'valores impossíveis',
      bmeOk ? '' : 'BME280 sem resposta no I2C1 (D15/D14 = PB8/PB9): o firmware ignora a falha no init e manda zeros. Confira ligação e alimentação do módulo.');

    const zero = [...new Set(TELEMETRY_GROUPS.filter((g) => !['DS3231', 'BME280'].includes(g.sensor) && g.fields.every((f) => !last[f])).map((g) => g.sensor))];
    if (zero.length) add('Telemetria e sensores', 'Dados de EPS, ADCS e baterias', 'warn', `sempre zero: ${zero.join(', ')}`, 'Ainda não implementado no firmware.');
  }
  return out;
}
