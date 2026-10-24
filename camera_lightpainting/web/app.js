import { api } from './api.js';

const $ = id => document.getElementById(id);
const settingInputs = [...document.querySelectorAll('[data-setting]')];
let state = null;
let revision = -1;
let drag = null;
let toastTimer;
let settingsTimer;
let pending = {};
let inFlight = 0;
let settingsChain = Promise.resolve();

function toast(message, error = false) {
  $('toast').textContent = message;
  $('toast').classList.toggle('error', error);
  $('toast').style.display = 'block';
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $('toast').style.display = 'none'; }, 4500);
}

function sliderFeedback(input) {
  const output = $(input.dataset.setting + 'Value');
  if (output) output.value = input.value;
  if (input.type === 'range') {
    const progress = (Number(input.value) - Number(input.min)) / (Number(input.max) - Number(input.min));
    input.style.setProperty('--progress', `${progress * 100}%`);
  }
}

function paintRoi(coords) {
  const box = $('roi');
  box.style.display = coords ? 'block' : 'none';
  if (!coords) return;
  const [x0, y0, x1, y1] = coords;
  Object.assign(box.style, {
    left: `${x0 * 100}%`, top: `${y0 * 100}%`,
    width: `${(x1 - x0) * 100}%`, height: `${(y1 - y0) * 100}%`,
  });
}

function render(s) {
  state = s;
  $('dot').classList.toggle('online', s.connected);
  $('connection').textContent = s.connected ? '相机已连接' : '相机未连接';
  $('message').textContent = s.message;
  $('drawLabel').textContent = s.drawing ? '暂停绘制' : '开始绘制';
  $('draw').querySelector('use').setAttribute('href', s.drawing ? '#i-pause' : '#i-play');
  $('draw').classList.toggle('is-drawing', s.drawing);
  $('draw').disabled = !s.connected && !s.drawing;
  $('background').disabled = !s.connected;
  $('save').disabled = !s.frames;
  $('drawingState').textContent = s.drawing ? '● 正在绘制' : '已暂停';
  $('drawingState').style.color = s.drawing ? 'var(--mint)' : 'var(--muted)';
  $('fps').textContent = s.connected ? s.fps.toFixed(1) : '—';
  $('frames').textContent = s.frames.toLocaleString();
  $('points').textContent = s.points.toLocaleString();
  $('resolution').textContent = s.resolution ? s.resolution.join(' × ') : '—';
  $('backgroundStatus').textContent = s.background_captured ? '已校准' : '待采集';
  $('backgroundStatus').classList.toggle('ready', s.background_captured);
  $('canvasMode').textContent = s.settings.mode === 'neon' ? '彩虹光轨' : '取亮长曝光';
  $('emptyCanvas').hidden = s.points > 0 || s.settings.overlay || (s.settings.mode === 'exposure' && s.frames > 0);
  $('emptyTitle').textContent = s.drawing ? '正在等待光源' : '等待第一道光';
  $('emptyHint').textContent = s.drawing ? '将光源移入识别区域，慢慢移动' : s.background_captured ? '背景已就绪，点击开始绘制' : '采集背景后，开始你的创作';
  $('stepBackground').classList.toggle('active', !s.background_captured);
  $('stepBackground').classList.toggle('done', s.background_captured);
  $('stepDraw').classList.toggle('active', s.drawing || (s.background_captured && !s.frames));
  $('stepSave').classList.toggle('active', s.frames > 0 && !s.drawing);
  if (s.resolution) {
    for (const id of ['sourceFrame', 'resultFrame']) $(id).style.aspectRatio = s.resolution.join('/');
  }

  // Keep a local adjustment intact while it is pending or focused.
  if (revision !== s.revision && !inFlight && !Object.keys(pending).length) {
    let skipped = false;
    settingInputs.forEach(input => {
      if (input === document.activeElement) { skipped = true; return; }
      const value = s.settings[input.dataset.setting];
      if (input.type === 'checkbox') input.checked = value;
      else input.value = value;
      sliderFeedback(input);
    });
    if (!skipped) revision = s.revision;
  }
  if (!drag) {
    const r = s.settings.roi;
    paintRoi(r && s.resolution ? [r[0] / s.resolution[0], r[1] / s.resolution[1], r[2] / s.resolution[0], r[3] / s.resolution[1]] : null);
  }

  const rec = s.recording;
  $('diagnostics').classList.toggle('recording', rec.active);
  $('record').disabled = rec.finalizing || (!s.connected && !rec.active);
  $('record').textContent = rec.active ? '停止录制' : rec.finalizing ? '正在打包…' : '开始录制';
  $('record').classList.toggle('rec', rec.active);
  $('recordInfo').textContent = rec.active ? `录制中 · ${rec.seconds.toFixed(1)} 秒`
    : rec.finalizing ? '正在保存视频并打包，请稍候…'
    : rec.result ? (rec.result.errors.length ? `录制错误：${rec.result.errors.join('；')}` : '录制完成，可下载诊断包。')
    : '记录原始画面与处理结果，帮助检查背景残留。';
  $('download').hidden = !rec.result;
  if (rec.result) $('download').href = rec.result.url;
}

function flushSettings() {
  clearTimeout(settingsTimer);
  if (!Object.keys(pending).length) return settingsChain;
  const changes = pending;
  pending = {};
  inFlight++;
  settingsChain = settingsChain.then(async () => {
    try { const s = await api.settings(changes); inFlight--; render(s); }
    catch (error) { inFlight--; revision = -1; toast(error.message, true); }
  });
  return settingsChain;
}

async function action(name, details = {}) {
  await flushSettings();
  try {
    const payload = await api.action(name, details);
    if (payload.version) render(payload);
    return payload;
  } catch (error) { toast(error.message, true); return null; }
}

async function poll() {
  try { render(await api.state()); }
  catch {
    $('dot').classList.remove('online');
    $('connection').textContent = '连接中断，正在重试';
    for (const id of ['draw', 'background', 'save', 'record']) $(id).disabled = true;
  }
  setTimeout(poll, 600);
}

function preview(id, view) {
  const img = $(id);
  let timer;
  const load = () => { img.src = api.preview(view()); };
  const next = delay => { clearTimeout(timer); timer = setTimeout(load, delay); };
  img.onload = () => next(document.hidden ? 1500 : 100);
  img.onerror = () => next(1500);
  load();
}

$('draw').onclick = () => action(state?.drawing ? 'pause' : 'start');
$('background').onclick = async () => { if (await action('background')) toast('背景已校准，可以开始绘制'); };
$('clear').onclick = () => action('clear');
$('save').onclick = async () => {
  const file = await action('save');
  if (!file) return;
  const a = document.createElement('a');
  a.href = file.url; a.download = file.name; a.click();
  toast('图片已保存，可在下载列表中查看');
};
$('record').onclick = () => action(state?.recording.active ? 'record_stop' : 'record_start');
$('connect').onclick = () => action('connect', { source: $('device').value });
$('removeRoi').onclick = event => { event.preventDefault(); action('roi', { coordinates: null }); };
settingInputs.forEach(input => {
  input.oninput = () => sliderFeedback(input);
  input.onchange = () => {
    pending[input.dataset.setting] = input.type === 'checkbox' ? input.checked : input.type === 'range' ? Number(input.value) : input.value;
    clearTimeout(settingsTimer);
    settingsTimer = setTimeout(flushSettings, 120);
  };
});

const sourceFrame = $('sourceFrame');
function point(event) {
  const rect = sourceFrame.getBoundingClientRect();
  return [Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width)), Math.max(0, Math.min(1, (event.clientY - rect.top) / rect.height))];
}
function rectangle(a, b) { return [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.max(a[0], b[0]), Math.max(a[1], b[1])]; }
sourceFrame.onpointerdown = event => {
  if (!state?.connected || event.button !== 0) return;
  drag = point(event); sourceFrame.setPointerCapture(event.pointerId); event.preventDefault();
};
sourceFrame.onpointermove = event => { if (drag) paintRoi(rectangle(drag, point(event))); };
sourceFrame.onpointerup = event => {
  if (!drag) return;
  const coordinates = rectangle(drag, point(event)); drag = null;
  action('roi', { coordinates });
};
sourceFrame.onpointercancel = () => { drag = null; };
document.onkeydown = event => {
  if (/INPUT|SELECT|TEXTAREA|BUTTON/.test(document.activeElement.tagName)) return;
  if (event.code === 'Space') { event.preventDefault(); $('draw').click(); }
  if (event.key === 'Escape') action('pause');
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 's') { event.preventDefault(); $('save').click(); }
};

api.devices().then(data => {
  data.devices.forEach(device => {
    const option = document.createElement('option');
    option.value = device.value; option.textContent = device.name;
    $('device').appendChild(option);
  });
  if (state) $('device').value = state.source;
}).catch(error => toast(error.message, true));
poll();
preview('camera', () => $('view').value);
preview('result', () => 'result');
