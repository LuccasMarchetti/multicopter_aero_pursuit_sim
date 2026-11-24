// Simulation tab -> batch studies (run_study.py), SITL tuning (tune_study.py) and the cascade
// queue (several studies in sequence on the same simulation).

import { h, button, select, textInput, toast, section } from './ui.js';

export function studiesPanel(ctl, { run, showLog }) {
  // Batch studies (run_study.py) and SITL tuning (tune_study.py).
  const studySel = select({ label: 'File', options: [] });
  const methodSel = select({ label: 'Optimizer', options: [['nm', 'Nelder–Mead'], ['bo', 'Bayesian']] });
  const evalsIn = textInput({ label: 'Evaluations (0 = from the file)', value: '0', type: 'number' });
  const refreshStudies = () => ctl.req('study.list').then((l) => studySel.setOptions(l)).catch(() => {});
  const runStudy = () => {
    showLog('study');
    run('study.run', { path: studySel.get(), method: methodSel.get(), max_evals: Number(evalsIn.get()) || 0 });
  };
  // Cascade: several studies in sequence on the same simulation (run_study.py a.yaml b.yaml ...).
  let queue = [];
  try { queue = JSON.parse(localStorage.getItem('gcs.studyQueue') || '[]'); } catch { queue = []; }
  const queueBox = h('ol', { class: 'queue' });
  const saveQueue = () => { try { localStorage.setItem('gcs.studyQueue', JSON.stringify(queue)); } catch { /* no storage */ } };
  function renderQueue() {
    queueBox.innerHTML = '';
    if (!queue.length) queueBox.append(h('li', { class: 'muted' }, 'empty queue'));
    queue.forEach((p, i) => queueBox.append(h('li', {}, p, ' ',
      button('↑', () => { if (i) { [queue[i - 1], queue[i]] = [queue[i], queue[i - 1]]; saveQueue(); renderQueue(); } }, 'small'),
      button('×', () => { queue.splice(i, 1); saveQueue(); renderQueue(); }, 'small'))));
  }
  renderQueue();
  const addQueue = () => {
    const p = studySel.get();
    if (!p) return;
    if (p.includes('/tuning/')) { toast('tuning (tune_*.yaml) runs alone, outside the cascade', 'error'); return; }
    queue.push(p); saveQueue(); renderQueue();
  };
  const runQueue = () => {
    if (!queue.length) { toast('empty queue', 'error'); return; }
    showLog('study');
    run('study.run', { paths: queue });
  };
  ctl.on('open', () => setTimeout(refreshStudies, 500));

  // Progress of the running (or last) study: runs done / total, current run, estimated end.
  const bar = h('div', { class: 'study-bar' }, h('div', {}));
  const progText = h('div', { class: 'muted' }, 'no study yet');
  const hms = (s) => (s == null ? '—' : `${Math.floor(s / 3600)}h${String(Math.floor(s / 60) % 60).padStart(2, '0')}m`);
  async function poll() {
    try {
      const { progress: p, sims } = await ctl.req('study.progress');
      if (!p) { progText.textContent = 'no study yet'; return; }
      bar.firstChild.style.width = `${p.percent}%`;
      const stale = p.state === 'running' && p.age_s > 300 ? ` (no update for ${Math.round(p.age_s / 60)} min)` : '';
      progText.innerHTML = '';
      progText.append(...[
        h('div', {}, h('b', {}, `${p.done}/${p.total} runs (${p.percent} %)`), ` · ${p.total - p.done} left · ${p.state}${stale}`),
        h('div', {}, `${p.study || ''}${p.tag ? ` [${p.tag}]` : ''} · elapsed ${hms(p.elapsed_s)}` +
          (p.eta_s != null ? ` · ETA ${hms(p.eta_s)} (ends ≈ ${new Date(Date.now() + (p.eta_s - p.age_s) * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })})` : '')),
        p.state === 'running' ? h('div', {}, `combination ${p.combination}: ${p.scenario} / ${p.variant}, run ${p.run || '—'} (${p.phase || ''})`) : null,
        sims ? h('div', {}, `simulators: ${sims.sims.join(' → ')}, now ${sims.current} (${sims.state})`) : null].filter(Boolean));
    } catch { /* GCS without the route */ }
  }
  setInterval(() => { if (document.visibilityState === 'visible') poll(); }, 5000);
  ctl.on('open', poll);


  return section('Batch studies and tuning',
    h('p', { class: 'muted' }, 'scenarios/studies/*.yaml runs run_study.py; scenarios/tuning/tune_*.yaml runs batch tuning (tune_study.py) in SITL. Output in the "study" log and in datasets/.'),
    studySel, h('div', { class: 'row wrap' }, methodSel, evalsIn),
    h('h4', {}, 'Progress'), bar, progText,
    h('div', { class: 'row wrap' }, button('▶ Run study', runStudy, 'primary'),
      button('■ Interrupt', () => run('study.cancel'), 'danger'), button('↻', refreshStudies, 'small')),
    h('h4', {}, 'Cascade (study queue)'),
    h('p', { class: 'muted' }, 'The queued studies run one after the other on the same simulation, each with its own folder and summary.csv. A file with "studies: [...]" (e.g. cascata_exemplo.yaml) is already a cascade.'),
    h('div', { class: 'row wrap' }, button('+ Add to queue', addQueue),
      button('▶ Run queue', runQueue, 'primary'), button('Clear queue', () => { queue = []; saveQueue(); renderQueue(); }, 'small')),
    queueBox);
}
