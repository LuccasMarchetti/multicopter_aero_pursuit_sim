// Topics tab: inspect (echo), publish (once or at a fixed rate) and call services.

import { h, button, section, slider, textInput, toast } from './ui.js';

export function topicsPanel(ctl) {
  const filter = textInput({ placeholder: 'filter topics/services' });
  const topicList = h('div', { class: 'list' });
  const srvList = h('div', { class: 'list' });
  let topics = [], services = [];

  // Echo.
  const echoTitle = h('div', { class: 'mono' }, '—');
  const echo = h('pre', { class: 'echo' });
  let echoSid = null, echoRate = 5;
  const stopEcho = () => { if (echoSid) ctl.req('unsub', { sid: echoSid }).catch(() => {}); echoSid = null; };
  async function startEcho(name) {
    stopEcho();
    echoSid = 'echo-' + Math.random().toString(36).slice(2);
    echoTitle.textContent = name;
    echo.textContent = 'waiting for message...';
    try {
      const r = await ctl.req('sub', { sid: echoSid, topic: name, rate: echoRate });
      echoTitle.textContent = `${name} [${r.type}]`;
      pubTopic.input.value = name;
      pubType.input.value = r.type;
    } catch (e) { echo.textContent = e.message; echoSid = null; }
  }
  ctl.on('msg', (m) => { if (m.sid === echoSid) echo.textContent = JSON.stringify(m.data, null, 2); });

  // Publishing.
  const pubTopic = textInput({ label: 'Topic', placeholder: '/X4/gazebo/command/twist' });
  const pubType = textInput({ label: 'Type', placeholder: 'geometry_msgs/msg/Twist' });
  const pubData = h('textarea', { class: 'json', rows: 8, placeholder: '{"linear": {"x": 1.0}}' });
  let pubTimer = null;
  const pubRate = slider({ label: 'Rate (0 = once)', min: 0, max: 50, step: 1, value: 0, unit: 'Hz', digits: 0 });
  const publishOnce = async () => {
    let data = {};
    try { data = pubData.value.trim() ? JSON.parse(pubData.value) : {}; } catch (e) { toast('Invalid JSON', 'error'); return false; }
    try { await ctl.req('pub', { topic: pubTopic.get(), type: pubType.get(), data }); return true; } catch (e) { toast(e.message, 'error'); return false; }
  };
  const pubBtn = button('Publish', async () => {
    const rate = Number(pubRate.querySelector('input').value);
    if (pubTimer) { clearInterval(pubTimer); pubTimer = null; pubBtn.textContent = 'Publish'; return; }
    if (!(await publishOnce())) return;
    if (rate > 0) {
      pubTimer = setInterval(publishOnce, 1000 / rate);
      pubBtn.textContent = 'Stop publishing';
    } else toast('Published', 'good');
  }, 'primary');
  const useEcho = button('Use last message as template', () => { if (echo.textContent.startsWith('{')) pubData.value = echo.textContent; }, 'small');

  // Services.
  const srvName = textInput({ label: 'Service', placeholder: '/pursuit_controller_node/enable' });
  const srvType = textInput({ label: 'Type', placeholder: 'std_srvs/srv/SetBool' });
  const srvReq = h('textarea', { class: 'json', rows: 4, placeholder: '{"data": true}' });
  const srvOut = h('pre', { class: 'echo' });
  const callBtn = button('Call', async () => {
    let req = {};
    try { req = srvReq.value.trim() ? JSON.parse(srvReq.value) : {}; } catch (e) { toast('Invalid JSON', 'error'); return; }
    srvOut.textContent = 'chamando...';
    try {
      const r = await ctl.req('srv', { service: srvName.get(), type: srvType.get() || undefined, request: req }, 15000);
      srvOut.textContent = JSON.stringify(r, null, 2);
    } catch (e) { srvOut.textContent = e.message; }
  }, 'primary');

  const render = () => {
    const q = filter.get().trim().toLowerCase();
    topicList.innerHTML = '';
    for (const t of topics.filter((t) => !q || t.name.toLowerCase().includes(q))) {
      topicList.append(h('div', { class: 'item', onclick: () => startEcho(t.name), title: t.types.join(', ') },
        h('span', { class: 'mono' }, t.name), h('span', { class: 'muted' }, t.types[0].replace('/msg/', '/'))));
    }
    srvList.innerHTML = '';
    for (const s of services.filter((s) => (!q || s.name.toLowerCase().includes(q)) && !/\/(describe|get|set|list)_parameters|get_parameter_types|set_parameters_atomically|get_type_description/.test(s.name))) {
      srvList.append(h('div', { class: 'item', onclick: () => { srvName.input.value = s.name; srvType.input.value = s.types[0]; } },
        h('span', { class: 'mono' }, s.name), h('span', { class: 'muted' }, s.types[0].replace('/srv/', '/'))));
    }
  };
  filter.input.addEventListener('input', render);
  const refresh = async () => {
    try {
      [topics, services] = await Promise.all([ctl.req('topics'), ctl.req('services')]);
      render();
    } catch (e) { toast(e.message, 'error'); }
  };
  ctl.on('open', refresh);

  return {
    el: h('div', {},
      section(null, h('div', { class: 'row' }, filter, button('↻', refresh, 'small', 'Refresh'))),
      section('Topics (click to inspect)', topicList),
      section('Inspection', echoTitle,
        slider({ label: 'Update rate', min: 1, max: 30, step: 1, value: echoRate, unit: 'Hz', digits: 0, live: false,
          onChange: (v) => { echoRate = v; if (echoSid) startEcho(echoTitle.textContent.split(' ')[0]); } }),
        h('div', { class: 'row' }, button('Stop', stopEcho, 'small'), useEcho), echo),
      section('Publish', pubTopic, pubType, pubData, pubRate, pubBtn),
      section('Services (click to fill)', srvList, srvName, srvType, srvReq, callBtn, srvOut)),
    refresh,
  };
}
