// ---------------- shareable URL state (added by allenye.me, scripts/patch-viewer.py) ----------------
// Mirrors collectState() into readable query params, e.g.
//   ?layout=B&variant=1&mode=top&walls=54&show=dims,doors,plumb&top=...
// A link with params wins over this browser's saved settings, so a shared link opens exactly as it was copied.
// Only the active mode's camera is written, to keep links short.
const chkName = (id) => id[3].toLowerCase() + id.slice(4); // chkDims -> dims
const num = (n) => String(Number(n.toFixed(3)));
const list = (a) => a.map(num).join(',');
function nums(s, len) {
  const a = s.split(',').map(Number);
  if (a.length < len || a.some((n) => !Number.isFinite(n))) throw new Error('bad list: ' + s);
  return a;
}

function stateToQuery(st) {
  const q = new URLSearchParams();
  const p = st.plumb;
  q.set('layout', p.layout);
  if (p.layout === 'B') q.set('variant', p.bVariant);
  q.set('mode', st.mode);
  q.set('walls', num(st.cut));
  q.set('show', Object.entries(st.checks).filter(([, on]) => on).map(([id]) => chkName(id)).join(','));
  if (st.mode === 'orbit') q.set('cam', list([...st.cam.pos, ...st.cam.target]));
  if (st.mode === 'top') q.set('top', list([...st.top.pos, ...st.top.target, st.top.zoom]));
  if (st.mode === 'walk') {
    const w = st.walk;
    q.set('walk', list([w.pos.x, w.pos.z, w.yaw, w.pitch, w.eye]));
    q.set('view', w.view);
    if (w.view === 'follow') q.set('follow', list([w.follow.dist, w.follow.pitch]));
  }
  q.set('tie', p.tieIn);
  q.set('depthTo', p.depthTo);
  q.set('tieDepth', num(p.tieInDepth));
  q.set('slope', num(p.slopePerFt));
  q.set('mainX', num(p.mainX));
  return q.toString().replace(/%2C/g, ',');
}

function queryToState() {
  const q = new URLSearchParams(location.search);
  if (!q.has('layout') && !q.has('mode')) return null;
  const st = { plumb: {} }, p = st.plumb;
  if (q.has('layout')) p.layout = q.get('layout');
  if (q.has('variant')) p.bVariant = Number(q.get('variant'));
  if (q.has('tie')) p.tieIn = q.get('tie');
  if (q.has('depthTo')) p.depthTo = q.get('depthTo');
  for (const [k, key] of [['tieDepth', 'tieInDepth'], ['slope', 'slopePerFt'], ['mainX', 'mainX']]) {
    if (q.has(k)) p[key] = nums(q.get(k), 1)[0];
  }
  if (q.has('mode')) st.mode = q.get('mode');
  if (q.has('walls')) st.cut = nums(q.get('walls'), 1)[0];
  if (q.has('show')) {
    const on = new Set(q.get('show').split(','));
    st.checks = Object.fromEntries(STATE_CHECKS.map((id) => [id, on.has(chkName(id))]));
  }
  if (q.has('cam')) { const a = nums(q.get('cam'), 6); st.cam = { pos: a.slice(0, 3), target: a.slice(3, 6) }; }
  if (q.has('top')) { const a = nums(q.get('top'), 7); st.top = { pos: a.slice(0, 3), target: a.slice(3, 6), zoom: a[6] }; }
  if (q.has('walk')) {
    const a = nums(q.get('walk'), 5);
    st.walk = { pos: { x: a[0], z: a[1] }, yaw: a[2], pitch: a[3], eye: a[4], view: q.get('view') || 'first' };
    if (q.has('follow')) { const f = nums(q.get('follow'), 2); st.walk.follow = { dist: f[0], pitch: f[1] }; }
  }
  return st;
}

let LAST_QUERY = null;
function syncUrl(st) {
  const qs = stateToQuery(st);
  if (qs === LAST_QUERY) return;
  LAST_QUERY = qs;
  history.replaceState(null, '', location.pathname + '?' + qs + location.hash);
}

function copyViewLink(btn) {
  saveState(); // capture the latest camera before copying
  const done = (text) => { btn.textContent = text; setTimeout(() => { btn.textContent = 'Copy link to this view'; }, 1500); };
  navigator.clipboard.writeText(location.href).then(() => done('Copied!'), () => done('Copy the address bar'));
}

