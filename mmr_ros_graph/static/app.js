const COLORS = ['#2f9e44', '#e07a2f', '#7048c6', '#1c7ed6', '#0ca678', '#d9480f', '#3b5bdb'];
const NODE_W = 220;
const STORAGE_KEY = 'mmr-ros-graph-state-v1';
const GRID = 24;

const canvas = document.getElementById('canvas');
const viewport = document.getElementById('viewport');
const groupsEl = document.getElementById('groups');
const wires = document.getElementById('wires');
const nodesEl = document.getElementById('nodes');
const counts = document.getElementById('counts');
const live = document.getElementById('live');
const diagPill = document.getElementById('diag-pill');
const srStatus = document.getElementById('sr-status');

const viewModeEl = document.getElementById('view-mode');
const systemBox = document.getElementById('system');
const hideLeafBox = document.getElementById('hide-leaf');
const hideDeadBox = document.getElementById('hide-dead');
const groupNsBox = document.getElementById('group-ns');

const searchInput = document.getElementById('search');
const searchResults = document.getElementById('search-results');
const exportJsonBtn = document.getElementById('export-json');
const exportSvgBtn = document.getElementById('export-svg');
const toggleInspectorBtn = document.getElementById('toggle-inspector');

const workspaceEl = document.getElementById('workspace');
const sidebarResizer = document.getElementById('sidebar-resizer');
const sidebarCollapseBtn = document.getElementById('sidebar-collapse');
const sidebarPeekBtn = document.getElementById('sidebar-peek');

const zoomOutBtn = document.getElementById('zoom-out');
const zoomResetBtn = document.getElementById('zoom-reset');
const zoomInBtn = document.getElementById('zoom-in');
const fitViewBtn = document.getElementById('fit-view');
const autoLayoutBtn = document.getElementById('auto-layout');
const toggleMapBtn = document.getElementById('toggle-map');
const minimapBox = document.getElementById('minimap');
const minimapSvg = document.getElementById('minimap-svg');

const tabButtons = Array.from(document.querySelectorAll('[role="tab"]'));
const tabPanels = Array.from(document.querySelectorAll('[role="tabpanel"]'));
const detailEl = document.getElementById('detail');
const healthListEl = document.getElementById('health-list');
const healthBadgeEl = document.getElementById('health-badge');
const logBadgeEl = document.getElementById('log-badge');
const logListEl = document.getElementById('log-list');
const logLevelEl = document.getElementById('log-level');
const logNodeOnlyEl = document.getElementById('log-node-only');
const logPauseBtn = document.getElementById('log-pause');
const logClearBtn = document.getElementById('log-clear');

const positions = new Map();
const nodeCards = new Map();
const portSignatures = new Map();
const paramCache = new Map();
const nodeLogCounts = new Map();

let graph = { nodes: [], topics: [], links: [], diagnostics: [], error: '' };
let selected = null;
let hovered = null;
let view = { x: 32, y: 28, scale: 1 };
let showMinimap = false;
let sidebarOpen = true;
let sidebarWidth = 380;
let lastRenderState = { mode: 'all', ids: [], nodeMap: new Map(), topics: [], leafTopics: [], links: [], ports: { outs: {}, ins: {} } };

let logs = [];
let logPaused = false;
let topicMonitor = { topic: '', type: '', active: false, hz: 0, kbps: 0, count: 0, sample: null, error: '' };
let topicStatusNode = null;
let topicSamplePre = null;

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  Object.entries(attrs).forEach(([key, value]) => {
    if (value === null || value === undefined || value === false) return;
    if (key === 'className') node.className = value;
    else if (key === 'text') node.textContent = value;
    else if (key === 'dataset') Object.assign(node.dataset, value);
    else if (key.startsWith('on') && typeof value === 'function') {
      node.addEventListener(key.slice(2).toLowerCase(), value);
    } else if (key === 'checked' || key === 'disabled') {
      node[key] = Boolean(value);
    } else {
      node.setAttribute(key, String(value));
    }
  });
  (Array.isArray(children) ? children : [children]).forEach((child) => {
    if (child === null || child === undefined) return;
    node.appendChild(typeof child === 'string' ? document.createTextNode(child) : child);
  });
  return node;
}

function clearEl(node) {
  while (node.firstChild) node.removeChild(node.firstChild);
}

function announce(text) {
  srStatus.textContent = text;
}

function colorFor(id) {
  let hash = 0;
  for (let i = 0; i < id.length; i += 1) hash = (hash * 33 + id.charCodeAt(i)) >>> 0;
  return COLORS[hash % COLORS.length];
}

function loadState() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return;
    const saved = JSON.parse(raw);
    if (saved.positions && typeof saved.positions === 'object') {
      Object.entries(saved.positions).forEach(([id, point]) => {
        if (point && typeof point.x === 'number' && typeof point.y === 'number') {
          positions.set(id, { x: Math.max(16, point.x), y: Math.max(16, point.y) });
        }
      });
    }
    if (saved.viewMode) viewModeEl.value = saved.viewMode;
    if (typeof saved.system === 'boolean') systemBox.checked = saved.system;
    if (typeof saved.hideLeaf === 'boolean') hideLeafBox.checked = saved.hideLeaf;
    if (typeof saved.hideDead === 'boolean') hideDeadBox.checked = saved.hideDead;
    if (typeof saved.groupNs === 'boolean') groupNsBox.checked = saved.groupNs;
    if (typeof saved.sidebarOpen === 'boolean') sidebarOpen = saved.sidebarOpen;
    if (typeof saved.sidebarWidth === 'number') {
      sidebarWidth = Math.max(280, Math.min(680, saved.sidebarWidth));
    }
  } catch (_err) {
    // Ignore storage access errors in private contexts.
  }
}

function saveState() {
  try {
    const posObj = {};
    positions.forEach((pt, id) => {
      posObj[id] = { x: Math.round(pt.x), y: Math.round(pt.y) };
    });
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      positions: posObj,
      viewMode: viewModeEl.value,
      system: systemBox.checked,
      hideLeaf: hideLeafBox.checked,
      hideDead: hideDeadBox.checked,
      groupNs: groupNsBox.checked,
      sidebarOpen,
      sidebarWidth,
    }));
  } catch (_err) {
    // Ignore storage quota/privacy failures.
  }
}

function applySidebarState() {
  workspaceEl.style.setProperty('--sidebar-w', `${sidebarWidth}px`);
  workspaceEl.classList.toggle('sidebar-collapsed', !sidebarOpen);
  toggleInspectorBtn.setAttribute('aria-pressed', sidebarOpen ? 'true' : 'false');
  sidebarPeekBtn.hidden = sidebarOpen;
}

function setSidebarOpen(nextOpen) {
  sidebarOpen = Boolean(nextOpen);
  applySidebarState();
  saveState();
}

function nodeOf(endpoint) {
  return typeof endpoint === 'string' ? endpoint : endpoint.node;
}

function splitNodeName(id) {
  const idx = id.lastIndexOf('/');
  if (idx <= 0) return { ns: '/', short: id.replace(/^\//, '') };
  return { ns: id.slice(0, idx), short: id.slice(idx + 1) };
}

function shortType(msgType) {
  const parts = (msgType || '').split('/');
  return parts[parts.length - 1] || 'topic';
}

function qosText(endpointOrQos) {
  const qos = endpointOrQos && endpointOrQos.qos ? endpointOrQos.qos : (endpointOrQos || {});
  if (!qos.reliability && !qos.durability) return 'QoS not announced';
  const parts = [qos.reliability || '?', qos.durability || '?'];
  if (qos.liveliness && qos.liveliness !== 'SYSTEM_DEFAULT' && qos.liveliness !== 'AUTOMATIC') {
    parts.push(qos.liveliness);
  }
  if (qos.deadline_ns) parts.push(`deadline ${Math.round(qos.deadline_ns / 1e6)}ms`);
  return parts.join(' / ');
}

function qosProblem(publisher, subscriber) {
  const offered = (publisher.qos || {});
  const asked = (subscriber.qos || {});
  const reasons = [];
  if (!offered.reliability || !asked.reliability) return reasons;
  if (asked.reliability === 'RELIABLE' && offered.reliability !== 'RELIABLE') {
    reasons.push(`subscriber is RELIABLE, publisher is ${offered.reliability}`);
  }
  if (asked.durability === 'TRANSIENT_LOCAL' && offered.durability !== 'TRANSIENT_LOCAL') {
    reasons.push(`subscriber is TRANSIENT_LOCAL, publisher is ${offered.durability || 'VOLATILE'}`);
  }
  return reasons;
}

function filteredGraphData() {
  const showSystem = systemBox.checked;
  const hideLeaf = hideLeafBox.checked;
  const hideDead = hideDeadBox.checked;
  const mode = viewModeEl.value;
  const effectiveHideLeaf = mode !== 'all' || hideLeaf;
  const effectiveHideDead = mode === 'active' || hideDead;

  const rawTopics = (graph.topics || []).filter((topic) => {
    if (!showSystem && (topic.system || topic.action_internal)) return false;
    const pubs = topic.publishers || [];
    const subs = topic.subscribers || [];
    if (effectiveHideLeaf && (pubs.length === 0 || subs.length === 0)) {
      return false;
    }
    return true;
  });

  const allowedTopics = new Set(rawTopics.map((topic) => topic.name));
  let links = [];
  const fromServer = (graph.links || []).filter((link) => allowedTopics.has(link.topic));

  if (fromServer.length > 0) {
    links = fromServer.map((link) => ({
      ...link,
      id: link.id || `${link.publisher}->${link.subscriber}|${link.topic}`,
      topics: [link.topic],
    }));
  } else {
    rawTopics.forEach((topic) => {
      (topic.publishers || []).forEach((pub) => {
        (topic.subscribers || []).forEach((sub) => {
          const pubNode = nodeOf(pub);
          const subNode = nodeOf(sub);
          if (!pubNode || !subNode || pubNode === subNode) return;
          const problems = qosProblem(pub, sub);
          links.push({
            id: `${pubNode}->${subNode}|${topic.name}`,
            topic: topic.name,
            topics: [topic.name],
            type: topic.type,
            publisher: pubNode,
            subscriber: subNode,
            compatibility: problems.length ? 'error' : 'ok',
            reason: problems.join('. '),
            pub_qos: pub.qos || {},
            sub_qos: sub.qos || {},
          });
        });
      });
    });
  }

  links = links.filter((link) => link.publisher && link.subscriber && link.publisher !== link.subscriber);

  if (mode === 'nodes') {
    const grouped = new Map();
    const rankWeight = { ok: 0, warning: 1, error: 2 };
    links.forEach((link) => {
      const key = `${link.publisher}->${link.subscriber}`;
      const existing = grouped.get(key);
      if (!existing) {
        grouped.set(key, {
          ...link,
          id: key,
          topic: '__node__',
          topics: [link.topic],
        });
        return;
      }
      if (!existing.topics.includes(link.topic)) existing.topics.push(link.topic);
      if ((rankWeight[link.compatibility] || 0) > (rankWeight[existing.compatibility] || 0)) {
        existing.compatibility = link.compatibility;
        existing.reason = link.reason;
      } else if (link.reason && existing.reason && !existing.reason.includes(link.reason)) {
        existing.reason += `; ${link.reason}`;
      }
    });
    links = Array.from(grouped.values()).map((item) => ({
      ...item,
      topic: item.topics.length > 1 ? `${item.topics.length} topics` : '',
    }));
  }

  const nodeMap = new Map();
  (graph.nodes || []).forEach((item) => {
    const obj = typeof item === 'string' ? { id: item, services: [], clients: [], actions: [], count: 1 } : item;
    if (obj.id) nodeMap.set(obj.id, obj);
  });
  rawTopics.forEach((topic) => {
    (topic.publishers || []).forEach((ep) => {
      const id = nodeOf(ep);
      if (id && !nodeMap.has(id)) nodeMap.set(id, { id, services: [], clients: [], actions: [], count: 1 });
    });
    (topic.subscribers || []).forEach((ep) => {
      const id = nodeOf(ep);
      if (id && !nodeMap.has(id)) nodeMap.set(id, { id, services: [], clients: [], actions: [], count: 1 });
    });
  });

  const connectedNodes = new Set();
  links.forEach((link) => {
    connectedNodes.add(link.publisher);
    connectedNodes.add(link.subscriber);
  });

  const leafTopics = mode === 'all' && !effectiveHideLeaf
    ? rawTopics.filter((topic) => {
      const pubs = topic.publishers || [];
      const subs = topic.subscribers || [];
      return (pubs.length > 0 && subs.length === 0) || (subs.length > 0 && pubs.length === 0);
    })
    : [];

  leafTopics.forEach((topic) => {
    (topic.publishers || []).forEach((ep) => connectedNodes.add(nodeOf(ep)));
    (topic.subscribers || []).forEach((ep) => connectedNodes.add(nodeOf(ep)));
  });

  let ids = Array.from(nodeMap.keys()).sort();
  if (effectiveHideDead) {
    ids = ids.filter((id) => connectedNodes.has(id));
  }

  return { mode, ids, nodeMap, topics: rawTopics, leafTopics, links };
}

function computeTopologicalLayout(ids, links) {
  const adj = new Map();
  const indeg = new Map();
  ids.forEach((id) => {
    adj.set(id, []);
    indeg.set(id, 0);
  });
  links.forEach((link) => {
    if (!adj.has(link.publisher) || !adj.has(link.subscriber)) return;
    if (!adj.get(link.publisher).includes(link.subscriber)) {
      adj.get(link.publisher).push(link.subscriber);
      indeg.set(link.subscriber, indeg.get(link.subscriber) + 1);
    }
  });

  const rank = new Map();
  const queue = [];
  ids.forEach((id) => {
    rank.set(id, 0);
    if (indeg.get(id) === 0) queue.push(id);
  });

  const visited = new Set();
  while (queue.length) {
    const cur = queue.shift();
    visited.add(cur);
    const curRank = rank.get(cur) || 0;
    (adj.get(cur) || []).forEach((next) => {
      rank.set(next, Math.max(rank.get(next) || 0, curRank + 1));
      indeg.set(next, indeg.get(next) - 1);
      if (indeg.get(next) === 0) queue.push(next);
    });
  }

  // Fallback for cyclic members
  ids.forEach((id) => {
    if (!visited.has(id) && !rank.has(id)) rank.set(id, 1);
  });

  const connected = new Set();
  links.forEach((link) => {
    connected.add(link.publisher);
    connected.add(link.subscriber);
  });

  const columns = new Map();
  const isolated = [];
  ids.forEach((id) => {
    if (!connected.has(id)) {
      isolated.push(id);
      return;
    }
    const r = rank.get(id) || 0;
    if (!columns.has(r)) columns.set(r, []);
    columns.get(r).push(id);
  });

  const nextCoords = new Map();
  let maxY = 48;
  Array.from(columns.keys()).sort((a, b) => a - b).forEach((colIdx) => {
    const colIds = columns.get(colIdx).sort();
    colIds.forEach((id, row) => {
      const y = 48 + row * 108;
      maxY = Math.max(maxY, y + 108);
      nextCoords.set(id, { x: 48 + colIdx * 310, y });
    });
  });

  isolated.sort().forEach((id, idx) => {
    const col = idx % 3;
    const row = Math.floor(idx / 3);
    nextCoords.set(id, { x: 48 + col * 310, y: maxY + 24 + row * 92 });
  });

  return nextCoords;
}

function overlapsAny(x, y, ignoreId) {
  for (const [otherId, pt] of positions.entries()) {
    if (otherId === ignoreId) continue;
    if (Math.abs(pt.x - x) < NODE_W + 24 && Math.abs(pt.y - y) < 64) return true;
  }
  return false;
}

function ensurePositions(ids, links) {
  const known = new Set(ids);
  Array.from(positions.keys()).forEach((id) => {
    if (!known.has(id)) positions.delete(id);
  });

  const missing = ids.filter((id) => !positions.has(id));
  if (!missing.length) return;

  const suggested = computeTopologicalLayout(ids, links);
  missing.forEach((id) => {
    const base = suggested.get(id) || { x: 48, y: 48 };
    let y = base.y;
    while (overlapsAny(base.x, y, id)) {
      y += 96;
    }
    positions.set(id, { x: base.x, y });
  });
}

function autoLayout() {
  const { ids, links } = lastRenderState;
  const fresh = computeTopologicalLayout(ids, links);
  fresh.forEach((pt, id) => positions.set(id, pt));
  saveState();
  render();
  fitView();
  announce('Graph nodes auto-arranged');
}

function portMap(links, leafTopics, mode) {
  const outs = {};
  const ins = {};
  if (mode === 'nodes') {
    links.forEach((link) => {
      outs[link.publisher] = ['__node__'];
      ins[link.subscriber] = ['__node__'];
    });
    return { outs, ins };
  }
  links.forEach((link) => {
    outs[link.publisher] = outs[link.publisher] || [];
    ins[link.subscriber] = ins[link.subscriber] || [];
    if (!outs[link.publisher].includes(link.topic)) outs[link.publisher].push(link.topic);
    if (!ins[link.subscriber].includes(link.topic)) ins[link.subscriber].push(link.topic);
  });
  (leafTopics || []).forEach((topic) => {
    (topic.publishers || []).forEach((ep) => {
      const id = nodeOf(ep);
      outs[id] = outs[id] || [];
      if (!outs[id].includes(topic.name)) outs[id].push(topic.name);
    });
    (topic.subscribers || []).forEach((ep) => {
      const id = nodeOf(ep);
      ins[id] = ins[id] || [];
      if (!ins[id].includes(topic.name)) ins[id].push(topic.name);
    });
  });
  return { outs, ins };
}

function nodeHeight(id, ports) {
  const count = Math.max((ports.outs[id] || []).length, (ports.ins[id] || []).length, 1);
  return Math.max(40, count * 22 + 18);
}

function portAnchor(id, topic, side, ports) {
  const point = positions.get(id);
  if (!point) return null;
  const key = lastRenderState.mode === 'nodes' ? '__node__' : topic;
  const list = (side === 'out' ? ports.outs[id] : ports.ins[id]) || [key];
  const index = Math.max(0, list.indexOf(key));
  const height = nodeHeight(id, ports);
  const y = point.y + 20 + index * ((height - 20) / Math.max(list.length, 1));
  return { x: point.x + (side === 'out' ? NODE_W : 0), y };
}

function isSelectedWire(link) {
  return selected && selected.kind === 'wire' && selected.id === link.id;
}

function applyViewportTransform() {
  viewport.style.transform = `translate(${view.x}px, ${view.y}px) scale(${view.scale})`;
  const step = GRID * view.scale;
  canvas.style.backgroundSize = `${step}px ${step}px`;
  canvas.style.backgroundPosition = `${view.x}px ${view.y}px`;
  zoomResetBtn.textContent = `${Math.round(view.scale * 100)}%`;
}

function screenToWorld(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (clientX - rect.left - view.x) / view.scale,
    y: (clientY - rect.top - view.y) / view.scale,
  };
}

function zoomAround(factor, clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const cx = clientX !== undefined ? clientX : rect.left + rect.width / 2;
  const cy = clientY !== undefined ? clientY : rect.top + rect.height / 2;
  const worldBefore = screenToWorld(cx, cy);
  const nextScale = Math.max(0.25, Math.min(2.2, view.scale * factor));
  view.scale = nextScale;
  view.x = cx - rect.left - worldBefore.x * nextScale;
  view.y = cy - rect.top - worldBefore.y * nextScale;
  applyViewportTransform();
  drawMinimap();
}

function fitView() {
  const { ids, ports } = lastRenderState;
  if (!ids.length) {
    view = { x: 32, y: 28, scale: 1 };
    applyViewportTransform();
    drawMinimap();
    return;
  }
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  ids.forEach((id) => {
    const pt = positions.get(id);
    if (!pt) return;
    const h = nodeHeight(id, ports);
    minX = Math.min(minX, pt.x);
    minY = Math.min(minY, pt.y);
    maxX = Math.max(maxX, pt.x + NODE_W);
    maxY = Math.max(maxY, pt.y + h);
  });
  const pad = 48;
  const bw = Math.max(100, maxX - minX + pad * 2);
  const bh = Math.max(100, maxY - minY + pad * 2);
  const scale = Math.max(0.25, Math.min(1.25, Math.min(canvas.clientWidth / bw, canvas.clientHeight / bh)));
  view.scale = scale;
  view.x = (canvas.clientWidth - bw * scale) / 2 - (minX - pad) * scale;
  view.y = (canvas.clientHeight - bh * scale) / 2 - (minY - pad) * scale;
  applyViewportTransform();
  drawMinimap();
}

function centerOnNode(id) {
  const pt = positions.get(id);
  if (!pt) return;
  const h = nodeHeight(id, lastRenderState.ports);
  view.x = canvas.clientWidth / 2 - (pt.x + NODE_W / 2) * view.scale;
  view.y = canvas.clientHeight / 2 - (pt.y + h / 2) * view.scale;
  applyViewportTransform();
  drawMinimap();
}

function updatePortsOnCard(card, id, ports, height) {
  const outs = ports.outs[id] || [];
  const ins = ports.ins[id] || [];
  const sig = `${height}|${ins.join(',')}|${outs.join(',')}`;
  if (portSignatures.get(id) === sig) return;
  portSignatures.set(id, sig);

  card.querySelectorAll('.port').forEach((node) => node.remove());
  const addSide = (topicNames, side) => {
    const count = Math.max(topicNames.length, 1);
    topicNames.forEach((topic, index) => {
      const dot = document.createElement('span');
      dot.className = `port ${side}`;
      const center = 20 + index * ((height - 20) / count);
      dot.style.top = `${center - 5.5}px`;
      dot.title = topic;
      card.appendChild(dot);
    });
  };
  addSide(ins, 'in');
  addSide(outs, 'out');
}

function nodeDiagnostics(id, nodeInfo, links) {
  const badges = [];
  if (nodeInfo && nodeInfo.count > 1) {
    badges.push({ cls: 'warn', text: `x${nodeInfo.count}`, title: `${nodeInfo.count} duplicate nodes share ${id}` });
  }
  const hasErr = links.some((l) => (l.publisher === id || l.subscriber === id) && l.compatibility === 'error');
  const hasWarn = links.some((l) => (l.publisher === id || l.subscriber === id) && l.compatibility === 'warning');
  if (hasErr) badges.push({ cls: 'error', text: 'QoS!', title: 'QoS mismatch on attached topic' });
  else if (hasWarn) badges.push({ cls: 'warn', text: 'QoS', title: 'QoS warning on attached topic' });

  const logStats = nodeLogCounts.get(id);
  if (logStats && logStats.error > 0) {
    badges.push({ cls: 'error', text: `${logStats.error} err`, title: `${logStats.error} error log(s)` });
  } else if (logStats && logStats.warn > 0) {
    badges.push({ cls: 'warn', text: `${logStats.warn} warn`, title: `${logStats.warn} warning log(s)` });
  }
  return badges;
}

function createNodeCard(id) {
  const { ns, short } = splitNodeName(id);
  const titleEl = el('div', { className: 'title', text: short || id });
  const headEl = el('div', { className: 'node-head' }, [titleEl]);
  const nsEl = ns !== '/' ? el('div', { className: 'ns-sub', text: ns }) : null;
  const badgesEl = el('div', { className: 'node-badges' });

  const card = el('article', {
    className: 'node',
    tabindex: '0',
    role: 'button',
    'aria-label': `ROS node ${id}`,
  }, [headEl, nsEl, badgesEl]);
  card.style.background = colorFor(id);

  let dragState = null;

  card.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    if (event.target.closest('button, input, select, textarea, a')) return;
    event.stopPropagation();
    const pt = positions.get(id) || { x: 48, y: 48 };
    const world = screenToWorld(event.clientX, event.clientY);
    dragState = {
      pointerId: event.pointerId,
      dx: world.x - pt.x,
      dy: world.y - pt.y,
      startX: pt.x,
      startY: pt.y,
      moved: false,
    };
    card.setPointerCapture(event.pointerId);
  });

  card.addEventListener('pointermove', (event) => {
    if (!dragState || !card.hasPointerCapture(event.pointerId)) return;
    const world = screenToWorld(event.clientX, event.clientY);
    let nx = Math.max(16, world.x - dragState.dx);
    let ny = Math.max(16, world.y - dragState.dy);
    if (event.shiftKey) {
      nx = Math.max(16, Math.round(nx / GRID) * GRID);
      ny = Math.max(16, Math.round(ny / GRID) * GRID);
    }
    if (Math.abs(nx - dragState.startX) + Math.abs(ny - dragState.startY) > 4) {
      dragState.moved = true;
      card.classList.add('dragging');
    }
    positions.set(id, { x: nx, y: ny });
    card.style.left = `${nx}px`;
    card.style.top = `${ny}px`;
    drawDynamicLayers();
  });

  const endDrag = (event, canceled) => {
    if (!dragState) return;
    const wasMoved = dragState.moved;
    dragState = null;
    card.classList.remove('dragging');
    if (wasMoved) {
      saveState();
      announce(`Moved ${id}`);
    } else if (!canceled) {
      selectNode(id);
    }
  };

  card.addEventListener('pointerup', (event) => {
    if (card.hasPointerCapture(event.pointerId)) {
      card.releasePointerCapture(event.pointerId);
    }
    endDrag(event, false);
  });

  card.addEventListener('lostpointercapture', (event) => endDrag(event, true));

  card.addEventListener('keydown', (event) => {
    const step = event.shiftKey ? GRID : 15;
    const pt = positions.get(id);
    if (!pt) return;
    if (event.key === 'ArrowLeft') pt.x = Math.max(16, pt.x - step);
    else if (event.key === 'ArrowRight') pt.x += step;
    else if (event.key === 'ArrowUp') pt.y = Math.max(16, pt.y - step);
    else if (event.key === 'ArrowDown') pt.y += step;
    else if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      selectNode(id);
      return;
    } else {
      return;
    }
    event.preventDefault();
    card.style.left = `${pt.x}px`;
    card.style.top = `${pt.y}px`;
    saveState();
    drawDynamicLayers();
    announce(`Moved ${id}`);
  });

  card.addEventListener('pointerenter', () => {
    hovered = { kind: 'node', id };
    updateHighlightClasses();
  });

  card.addEventListener('pointerleave', () => {
    if (hovered && hovered.kind === 'node' && hovered.id === id) {
      hovered = null;
      updateHighlightClasses();
    }
  });

  card.addEventListener('click', (event) => event.stopPropagation());

  return card;
}

function syncNodes(ids, nodeMap, ports, links) {
  const activeSet = new Set(ids);
  Array.from(nodeCards.keys()).forEach((id) => {
    if (!activeSet.has(id)) {
      nodeCards.get(id).remove();
      nodeCards.delete(id);
      portSignatures.delete(id);
    }
  });

  ids.forEach((id) => {
    let card = nodeCards.get(id);
    if (!card) {
      card = createNodeCard(id);
      nodeCards.set(id, card);
      nodesEl.appendChild(card);
    }
    const pt = positions.get(id);
    const height = nodeHeight(id, ports);
    card.style.left = `${pt.x}px`;
    card.style.top = `${pt.y}px`;
    card.style.height = `${height}px`;
    card.classList.toggle('selected', Boolean(selected && selected.kind === 'node' && selected.id === id));

    const badgesEl = card.querySelector('.node-badges');
    clearEl(badgesEl);
    nodeDiagnostics(id, nodeMap.get(id), links).forEach((badge) => {
      badgesEl.appendChild(el('span', {
        className: `node-badge ${badge.cls}`,
        text: badge.text,
        title: badge.title,
      }));
    });

    updatePortsOnCard(card, id, ports, height);
  });
}

function drawGroups(ids, ports) {
  clearEl(groupsEl);
  if (!groupNsBox.checked) return;

  const byNs = new Map();
  ids.forEach((id) => {
    const { ns } = splitNodeName(id);
    if (!ns || ns === '/') return;
    if (!byNs.has(ns)) byNs.set(ns, []);
    byNs.get(ns).push(id);
  });

  byNs.forEach((memberIds, ns) => {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    memberIds.forEach((id) => {
      const pt = positions.get(id);
      if (!pt) return;
      const h = nodeHeight(id, ports);
      minX = Math.min(minX, pt.x);
      minY = Math.min(minY, pt.y);
      maxX = Math.max(maxX, pt.x + NODE_W);
      maxY = Math.max(maxY, pt.y + h);
    });
    if (!Number.isFinite(minX)) return;

    const gx = minX - 18;
    const gy = minY - 34;
    const gw = maxX - minX + 36;
    const gh = maxY - minY + 52;

    const header = el('div', { className: 'ns-group-header' }, [
      el('span', { text: ns }),
      el('span', { text: `${memberIds.length}` }),
    ]);
    const box = el('div', { className: 'ns-group' }, [header]);
    box.style.left = `${gx}px`;
    box.style.top = `${gy}px`;
    box.style.width = `${gw}px`;
    box.style.height = `${gh}px`;

    let groupDrag = null;
    header.addEventListener('pointerdown', (event) => {
      if (event.button !== 0) return;
      event.stopPropagation();
      const world = screenToWorld(event.clientX, event.clientY);
      const snapshotPts = new Map();
      memberIds.forEach((id) => {
        const pt = positions.get(id);
        if (pt) snapshotPts.set(id, { ...pt });
      });
      groupDrag = { startWorld: world, snapshotPts };
      header.setPointerCapture(event.pointerId);
    });

    header.addEventListener('pointermove', (event) => {
      if (!groupDrag || !header.hasPointerCapture(event.pointerId)) return;
      const world = screenToWorld(event.clientX, event.clientY);
      const dx = world.x - groupDrag.startWorld.x;
      const dy = world.y - groupDrag.startWorld.y;
      groupDrag.snapshotPts.forEach((orig, id) => {
        const nx = Math.max(16, orig.x + dx);
        const ny = Math.max(16, orig.y + dy);
        positions.set(id, { x: nx, y: ny });
        const card = nodeCards.get(id);
        if (card) {
          card.style.left = `${nx}px`;
          card.style.top = `${ny}px`;
        }
      });
      // Update group frame + wires without tearing down dragged header.
      box.style.left = `${gx + dx}px`;
      box.style.top = `${gy + dy}px`;
      drawWires(lastRenderState.links, lastRenderState.ports);
      drawMinimap();
    });

    const stopGroupDrag = () => {
      if (!groupDrag) return;
      groupDrag = null;
      saveState();
      drawDynamicLayers();
    };
    header.addEventListener('pointerup', stopGroupDrag);
    header.addEventListener('lostpointercapture', stopGroupDrag);

    groupsEl.appendChild(box);
  });
}

function wireDefs() {
  const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
  const specs = [
    ['arrow-ok', '#7e95ad'],
    ['arrow-warn', '#d97706'],
    ['arrow-error', '#c92a2a'],
    ['arrow-sel', '#e07a2f'],
  ];
  specs.forEach(([id, fill]) => {
    const marker = document.createElementNS('http://www.w3.org/2000/svg', 'marker');
    marker.setAttribute('id', id);
    marker.setAttribute('viewBox', '0 0 10 10');
    marker.setAttribute('refX', '8');
    marker.setAttribute('refY', '5');
    marker.setAttribute('markerWidth', '6');
    marker.setAttribute('markerHeight', '6');
    marker.setAttribute('orient', 'auto-start-reverse');
    const tri = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    tri.setAttribute('d', 'M 0 1.5 L 8.5 5 L 0 8.5 z');
    tri.setAttribute('fill', fill);
    marker.appendChild(tri);
    defs.appendChild(marker);
  });
  return defs;
}

function wirePathGeometry(start, end, laneOffset, pubHeight, subHeight) {
  if (end.x >= start.x + 28) {
    const bend = Math.max(60, Math.abs(end.x - start.x) * 0.45);
    const cy1 = start.y + laneOffset * 0.35;
    const cy2 = end.y + laneOffset * 0.35;
    const d = `M ${start.x} ${start.y} C ${start.x + bend} ${cy1}, ${end.x - bend} ${cy2}, ${end.x} ${end.y}`;
    return {
      d,
      lx: (start.x + end.x) / 2,
      ly: (start.y + end.y) / 2 - 7 + laneOffset,
    };
  }
  // Route backward or stacked-column links smoothly below both boxes.
  const drop = Math.max(pubHeight, subHeight) + 34 + Math.abs(laneOffset) * 1.4;
  const routeY = Math.max(start.y, end.y) + drop;
  const cx1 = start.x + 70;
  const cx2 = end.x - 70;
  const d = `M ${start.x} ${start.y} C ${cx1} ${routeY}, ${cx2} ${routeY}, ${end.x} ${end.y}`;
  return {
    d,
    lx: (start.x + end.x) / 2,
    ly: routeY - 10,
  };
}

function drawWires(links, ports, leafTopics = lastRenderState.leafTopics, mode = lastRenderState.mode) {
  clearEl(wires);
  wires.appendChild(wireDefs());

  const pairCounts = new Map();
  const pairSeen = new Map();
  links.forEach((link) => {
    const key = `${link.publisher}->${link.subscriber}`;
    pairCounts.set(key, (pairCounts.get(key) || 0) + 1);
  });

  links.forEach((link) => {
    const start = portAnchor(link.publisher, link.topic, 'out', ports);
    const end = portAnchor(link.subscriber, link.topic, 'in', ports);
    if (!start || !end) return;

    const pairKey = `${link.publisher}->${link.subscriber}`;
    const total = pairCounts.get(pairKey) || 1;
    const idx = pairSeen.get(pairKey) || 0;
    pairSeen.set(pairKey, idx + 1);
    const laneOffset = (idx - (total - 1) / 2) * 18;

    const geom = wirePathGeometry(
      start,
      end,
      laneOffset,
      nodeHeight(link.publisher, ports),
      nodeHeight(link.subscriber, ports),
    );

    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', geom.d);
    const classes = ['wire'];
    let markerId = 'arrow-ok';
    if (link.compatibility === 'error') {
      classes.push('mismatch');
      markerId = 'arrow-error';
    } else if (link.compatibility === 'warning') {
      classes.push('warning');
      markerId = 'arrow-warn';
    }
    if (isSelectedWire(link)) {
      classes.push('selected');
      markerId = 'arrow-sel';
    }
    path.setAttribute('class', classes.join(' '));
    path.setAttribute('marker-end', `url(#${markerId})`);
    path.dataset.linkId = link.id;
    path.dataset.publisher = link.publisher;
    path.dataset.subscriber = link.subscriber;

    path.addEventListener('click', (event) => {
      event.stopPropagation();
      selectWire(link);
    });
    path.addEventListener('pointerenter', () => {
      hovered = { kind: 'wire', id: link.id, publisher: link.publisher, subscriber: link.subscriber };
      updateHighlightClasses();
    });
    path.addEventListener('pointerleave', () => {
      if (hovered && hovered.kind === 'wire' && hovered.id === link.id) {
        hovered = null;
        updateHighlightClasses();
      }
    });
    wires.appendChild(path);

    if (link.topic) {
      const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      label.setAttribute('class', 'wire-label');
      label.setAttribute('x', geom.lx);
      label.setAttribute('y', geom.ly);
      label.setAttribute('text-anchor', 'middle');
      label.dataset.linkId = link.id;
      label.dataset.publisher = link.publisher;
      label.dataset.subscriber = link.subscriber;
      label.textContent = link.topic;
      wires.appendChild(label);
    }
  });

  if (mode === 'all' && leafTopics && leafTopics.length) {
    leafTopics.forEach((topic) => {
      const isPubOnly = (topic.publishers || []).length > 0 && (topic.subscribers || []).length === 0;
      const endpoints = isPubOnly ? topic.publishers : topic.subscribers;
      endpoints.forEach((ep) => {
        const nodeId = nodeOf(ep);
        const side = isPubOnly ? 'out' : 'in';
        const anchor = portAnchor(nodeId, topic.name, side, ports);
        if (!anchor) return;
        const dx = isPubOnly ? 58 : -58;
        const endX = anchor.x + dx;

        const leafId = `leaf:${topic.name}:${nodeId}`;
        const stub = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        stub.setAttribute('d', `M ${anchor.x} ${anchor.y} L ${endX} ${anchor.y}`);
        stub.setAttribute('class', 'wire leaf-stub');
        stub.dataset.linkId = leafId;
        stub.dataset.owner = nodeId;
        if (isPubOnly) stub.setAttribute('marker-end', 'url(#arrow-ok)');
        else stub.setAttribute('marker-start', 'url(#arrow-ok)');
        stub.addEventListener('click', (event) => {
          event.stopPropagation();
          selectTopic(topic.name);
        });
        wires.appendChild(stub);

        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.setAttribute('class', 'wire-label leaf-label');
        label.setAttribute('x', endX + (isPubOnly ? 6 : -6));
        label.setAttribute('y', anchor.y + 4);
        label.setAttribute('text-anchor', isPubOnly ? 'start' : 'end');
        label.dataset.linkId = leafId;
        label.dataset.owner = nodeId;
        label.textContent = topic.name;
        label.addEventListener('click', (event) => {
          event.stopPropagation();
          selectTopic(topic.name);
        });
        wires.appendChild(label);
      });
    });
  }

  updateHighlightClasses();
}

function updateHighlightClasses() {
  const { links } = lastRenderState;
  const activeNodes = new Set();
  const activeLinks = new Set();

  if (hovered) {
    if (hovered.kind === 'node') {
      activeNodes.add(hovered.id);
      links.forEach((l) => {
        if (l.publisher === hovered.id || l.subscriber === hovered.id) {
          activeLinks.add(l.id);
          activeNodes.add(l.publisher);
          activeNodes.add(l.subscriber);
        }
      });
    } else if (hovered.kind === 'wire') {
      activeLinks.add(hovered.id);
      activeNodes.add(hovered.publisher);
      activeNodes.add(hovered.subscriber);
    }
  }

  nodeCards.forEach((card, id) => {
    const dim = Boolean(hovered) && !activeNodes.has(id);
    const hi = Boolean(hovered) && activeNodes.has(id);
    card.classList.toggle('dimmed', dim);
    card.classList.toggle('highlighted', hi);
  });

  wires.querySelectorAll('.wire, .wire-label').forEach((node) => {
    const lid = node.dataset.linkId;
    const owner = node.dataset.owner;
    const isHit = activeLinks.has(lid) || (hovered && hovered.kind === 'node' && owner === hovered.id);
    const dim = Boolean(hovered) && !isHit;
    const hi = Boolean(hovered) && isHit;
    node.classList.toggle('dimmed', dim);
    node.classList.toggle('highlighted', hi);
  });
}

function drawMinimap() {
  if (!showMinimap) return;
  clearEl(minimapSvg);
  const { ids, ports } = lastRenderState;
  if (!ids.length) return;

  let minX = 0;
  let minY = 0;
  let maxX = 1000;
  let maxY = 650;
  ids.forEach((id) => {
    const pt = positions.get(id);
    if (!pt) return;
    minX = Math.min(minX, pt.x - 40);
    minY = Math.min(minY, pt.y - 40);
    maxX = Math.max(maxX, pt.x + NODE_W + 40);
    maxY = Math.max(maxY, pt.y + nodeHeight(id, ports) + 40);
  });

  const vw = Math.max(200, maxX - minX);
  const vh = Math.max(130, maxY - minY);
  minimapSvg.setAttribute('viewBox', `${minX} ${minY} ${vw} ${vh}`);

  ids.forEach((id) => {
    const pt = positions.get(id);
    if (!pt) return;
    const rect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    rect.setAttribute('x', pt.x);
    rect.setAttribute('y', pt.y);
    rect.setAttribute('width', NODE_W);
    rect.setAttribute('height', nodeHeight(id, ports));
    rect.setAttribute('rx', '12');
    rect.setAttribute('fill', colorFor(id));
    minimapSvg.appendChild(rect);
  });

  const box = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
  box.setAttribute('x', -view.x / view.scale);
  box.setAttribute('y', -view.y / view.scale);
  box.setAttribute('width', canvas.clientWidth / view.scale);
  box.setAttribute('height', canvas.clientHeight / view.scale);
  box.setAttribute('fill', 'rgba(28, 126, 214, 0.12)');
  box.setAttribute('stroke', '#1c7ed6');
  box.setAttribute('stroke-width', String(Math.max(4, vw * 0.012)));
  minimapSvg.appendChild(box);
}

function drawDynamicLayers() {
  drawGroups(lastRenderState.ids, lastRenderState.ports);
  drawWires(lastRenderState.links, lastRenderState.ports);
  drawMinimap();
}

function render() {
  const { mode, ids, nodeMap, topics, leafTopics, links } = filteredGraphData();
  ensurePositions(ids, links);
  const ports = portMap(links, leafTopics, mode);
  lastRenderState = { mode, ids, nodeMap, topics, leafTopics, links, ports };

  hideLeafBox.disabled = mode !== 'all';
  hideDeadBox.disabled = mode === 'active';

  const error = graph.error ? ` · ${graph.error}` : '';
  const summary = mode === 'nodes'
    ? `${ids.length} nodes · ${links.length} links (nodes only)`
    : `${ids.length} nodes · ${topics.length} topics`;
  counts.textContent = `${summary}${error}`;
  live.classList.toggle('down', Boolean(graph.error));
  live.textContent = graph.error ? 'error' : 'live';

  syncNodes(ids, nodeMap, ports, links);
  drawDynamicLayers();
  applyViewportTransform();
  renderHealthPanel();
  refreshSelectedInspector();
}

function switchTab(tabId) {
  const map = {
    'tab-selection': 'panel-selection',
    'tab-console': 'panel-console',
    'tab-health': 'panel-health',
  };
  tabButtons.forEach((btn) => {
    const active = btn.id === tabId;
    btn.setAttribute('aria-selected', active ? 'true' : 'false');
    btn.tabIndex = active ? 0 : -1;
  });
  tabPanels.forEach((panel) => {
    const visible = panel.id === map[tabId];
    panel.hidden = !visible;
    if (visible) panel.tabIndex = 0;
    else panel.removeAttribute('tabindex');
  });
}

tabButtons.forEach((btn, idx) => {
  btn.addEventListener('click', () => switchTab(btn.id));
  btn.addEventListener('keydown', (event) => {
    let nextIdx = idx;
    if (event.key === 'ArrowRight') nextIdx = (idx + 1) % tabButtons.length;
    else if (event.key === 'ArrowLeft') nextIdx = (idx - 1 + tabButtons.length) % tabButtons.length;
    else if (event.key === 'Home') nextIdx = 0;
    else if (event.key === 'End') nextIdx = tabButtons.length - 1;
    else return;
    event.preventDefault();
    tabButtons[nextIdx].focus();
    switchTab(tabButtons[nextIdx].id);
  });
});

function selectNode(id) {
  selected = { kind: 'node', id };
  setSidebarOpen(true);
  switchTab('tab-selection');
  syncNodes(lastRenderState.ids, lastRenderState.nodeMap, lastRenderState.ports, lastRenderState.links);
  drawWires(lastRenderState.links, lastRenderState.ports);
  renderNodeDetail(id);
  if (logNodeOnlyEl.checked) renderLogs();
}

function selectWire(link) {
  selected = {
    kind: 'wire',
    id: link.id,
    topic: link.topic || (link.topics && link.topics[0]) || '',
    topics: link.topics || [link.topic],
    publisher: link.publisher,
    subscriber: link.subscriber,
  };
  setSidebarOpen(true);
  switchTab('tab-selection');
  syncNodes(lastRenderState.ids, lastRenderState.nodeMap, lastRenderState.ports, lastRenderState.links);
  drawWires(lastRenderState.links, lastRenderState.ports);
  renderWireDetail(link);
}

function selectTopic(topicName) {
  selected = { kind: 'topic', name: topicName };
  setSidebarOpen(true);
  switchTab('tab-selection');
  syncNodes(lastRenderState.ids, lastRenderState.nodeMap, lastRenderState.ports, lastRenderState.links);
  drawWires(lastRenderState.links, lastRenderState.ports);
  renderTopicDetail(topicName);
}

function refreshSelectedInspector() {
  if (!selected) return;
  if (selected.kind === 'node') {
    if (!lastRenderState.nodeMap.has(selected.id)) {
      selected = null;
      clearEl(detailEl);
      detailEl.appendChild(el('p', { className: 'muted-note', text: 'Selected node is no longer active.' }));
      return;
    }
    // Avoid rebuilding active parameter input focus while live graph updates arrive;
    // update node detail if parameters aren't focused.
    if (!detailEl.contains(document.activeElement)) {
      renderNodeDetail(selected.id);
    }
  } else if (selected.kind === 'wire') {
    const match = lastRenderState.links.find((item) => item.id === selected.id);
    if (!match) {
      selected = null;
      clearEl(detailEl);
      detailEl.appendChild(el('p', { className: 'muted-note', text: 'Selected wire is no longer active.' }));
      return;
    }
    if (!(topicMonitor.active && topicMonitor.topic === match.topic)) renderWireDetail(match);
  } else if (selected.kind === 'topic') {
    const match = lastRenderState.topics.find((item) => item.name === selected.name);
    if (!match) {
      selected = null;
      clearEl(detailEl);
      detailEl.appendChild(el('p', { className: 'muted-note', text: 'Selected topic is no longer active.' }));
    }
  }
}

function fetchNodeParameters(nodeId, force = false) {
  const existing = paramCache.get(nodeId);
  if (!force && existing && (existing.loading || existing.loaded)) return;

  const state = {
    loading: true,
    loaded: false,
    error: '',
    parameters: existing ? existing.parameters : [],
    showSystem: existing ? existing.showSystem : false,
  };
  paramCache.set(nodeId, state);

  fetch(`/api/params?node=${encodeURIComponent(nodeId)}`)
    .then((res) => res.json())
    .then((data) => {
      paramCache.set(nodeId, {
        loading: false,
        loaded: true,
        error: data.error || '',
        parameters: data.parameters || [],
        showSystem: state.showSystem,
      });
      if (selected && selected.kind === 'node' && selected.id === nodeId) {
        renderNodeDetail(nodeId);
      }
    })
    .catch((err) => {
      paramCache.set(nodeId, {
        loading: false,
        loaded: true,
        error: String(err),
        parameters: [],
        showSystem: state.showSystem,
      });
      if (selected && selected.kind === 'node' && selected.id === nodeId) {
        renderNodeDetail(nodeId);
      }
    });
}

function saveParameter(nodeId, param, rawValue, statusNode, applyBtn) {
  statusNode.className = 'param-status';
  statusNode.textContent = 'Saving...';
  if (applyBtn) applyBtn.disabled = true;

  fetch('/api/params', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      node: nodeId,
      name: param.name,
      type: param.type,
      value: rawValue,
    }),
  })
    .then((res) => res.json())
    .then((data) => {
      if (applyBtn) applyBtn.disabled = false;
      if (data.ok) {
        param.value = data.value;
        statusNode.className = 'param-status ok';
        statusNode.textContent = 'Saved';
      } else {
        statusNode.className = 'param-status error';
        statusNode.textContent = data.reason || 'Rejected';
      }
    })
    .catch((err) => {
      if (applyBtn) applyBtn.disabled = false;
      statusNode.className = 'param-status error';
      statusNode.textContent = String(err);
    });
}

function buildParameterRow(nodeId, param) {
  const statusEl = el('div', { className: 'param-status' });
  const top = el('div', { className: 'param-top' }, [
    el('span', { className: 'param-name', text: param.name, title: param.description || '' }),
    el('span', { className: 'pill', text: param.type }),
  ]);

  if (param.read_only) {
    top.appendChild(el('span', { className: 'pill warn', text: 'read-only' }));
  }

  const controls = el('div', { className: 'param-controls' });

  if (param.type === 'bool') {
    const checkbox = el('input', {
      type: 'checkbox',
      checked: Boolean(param.value),
      disabled: param.read_only,
    });
    const label = el('span', { text: checkbox.checked ? 'true' : 'false' });
    checkbox.addEventListener('change', () => {
      label.textContent = checkbox.checked ? 'true' : 'false';
      saveParameter(nodeId, param, checkbox.checked, statusEl, null);
    });
    controls.appendChild(checkbox);
    controls.appendChild(label);
  } else {
    const isNumeric = param.type === 'int' || param.type === 'double';
    const range = param.type === 'int' ? param.integer_range : param.floating_point_range;
    const formatted = Array.isArray(param.value)
      ? JSON.stringify(param.value)
      : (param.value === null || param.value === undefined ? '' : String(param.value));

    const input = el('input', {
      type: isNumeric ? 'number' : 'text',
      value: formatted,
      disabled: param.read_only,
    });
    if (isNumeric) {
      input.step = param.type === 'int' ? '1' : 'any';
    }

    if (range && !param.read_only && range.to > range.from) {
      const slider = el('input', {
        type: 'range',
        min: range.from,
        max: range.to,
        step: range.step > 0 ? range.step : (param.type === 'int' ? 1 : 0.01),
        value: Number(param.value || range.from),
      });
      slider.addEventListener('input', () => {
        input.value = slider.value;
      });
      input.addEventListener('input', () => {
        slider.value = input.value;
      });
      controls.appendChild(slider);
    }

    controls.appendChild(input);

    if (!param.read_only) {
      const applyBtn = el('button', {
        type: 'button',
        className: 'btn-small btn-primary',
        text: 'Set',
      });
      const submit = () => {
        const val = isNumeric ? Number(input.value) : input.value;
        saveParameter(nodeId, param, val, statusEl, applyBtn);
      };
      applyBtn.addEventListener('click', submit);
      input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') submit();
      });
      controls.appendChild(applyBtn);
    }
  }

  const card = el('div', { className: 'param-card' }, [top]);
  if (param.description) {
    card.appendChild(el('div', { className: 'item-sub', text: param.description }));
  }
  card.appendChild(controls);
  card.appendChild(statusEl);
  return card;
}

function renderNodeDetail(id) {
  clearEl(detailEl);
  const nodeInfo = lastRenderState.nodeMap.get(id) || { id, services: [], clients: [], actions: [], count: 1 };
  const topics = lastRenderState.topics;
  const outgoing = topics.filter((topic) => (topic.publishers || []).some((ep) => nodeOf(ep) === id));
  const incoming = topics.filter((topic) => (topic.subscribers || []).some((ep) => nodeOf(ep) === id));

  detailEl.appendChild(el('strong', { text: id }));
  if (nodeInfo.count > 1) {
    detailEl.appendChild(el('div', {
      className: 'alert-box',
      text: `Warning: ${nodeInfo.count} active nodes share the exact name ${id}.`,
    }));
  }

  const renderTopicSection = (title, list, roleKey) => {
    detailEl.appendChild(el('div', { className: 'sec-head', text: `${title} (${list.length})` }));
    if (!list.length) {
      detailEl.appendChild(el('div', { className: 'muted-note', text: 'none' }));
      return;
    }
    const wrap = el('div', { className: 'item-list' });
    list.forEach((topic) => {
      const ep = (topic[roleKey] || []).find((entry) => nodeOf(entry) === id);
      const row = el('div', { className: 'item-row clickable', title: 'Click to inspect & monitor topic' }, [
        el('div', {}, [el('code', { text: topic.name }), ' ', el('span', { className: 'pill', text: shortType(topic.type) })]),
        el('div', { className: 'item-sub', text: qosText(ep) }),
      ]);
      row.addEventListener('click', () => selectTopic(topic.name));
      wrap.appendChild(row);
    });
    detailEl.appendChild(wrap);
  };

  renderTopicSection('Publishes', outgoing, 'publishers');
  renderTopicSection('Subscribes', incoming, 'subscribers');

  // Actions
  const actions = nodeInfo.actions || [];
  if (actions.length) {
    detailEl.appendChild(el('div', { className: 'sec-head', text: `Actions (${actions.length})` }));
    const wrap = el('div', { className: 'item-list' });
    actions.forEach((act) => {
      wrap.appendChild(el('div', { className: 'item-row' }, [
        el('div', {}, [el('code', { text: act.name }), ' ', el('span', { className: 'pill', text: act.role })]),
        el('div', { className: 'item-sub', text: act.type }),
      ]));
    });
    detailEl.appendChild(wrap);
  }

  // Services & Clients
  const userServices = (nodeInfo.services || []).filter((s) => !s.system);
  const sysServices = (nodeInfo.services || []).filter((s) => s.system);
  const userClients = (nodeInfo.clients || []).filter((c) => !c.system);

  detailEl.appendChild(el('div', { className: 'sec-head', text: `Services (${userServices.length})` }));
  if (!userServices.length && !userClients.length) {
    detailEl.appendChild(el('div', { className: 'muted-note', text: sysServices.length ? `none (${sysServices.length} std param services)` : 'none' }));
  } else {
    const wrap = el('div', { className: 'item-list' });
    userServices.forEach((srv) => {
      wrap.appendChild(el('div', { className: 'item-row' }, [
        el('div', {}, [el('code', { text: srv.name }), ' ', el('span', { className: 'pill', text: 'server' })]),
        el('div', { className: 'item-sub', text: srv.type }),
      ]));
    });
    userClients.forEach((cli) => {
      wrap.appendChild(el('div', { className: 'item-row' }, [
        el('div', {}, [el('code', { text: cli.name }), ' ', el('span', { className: 'pill', text: 'client' })]),
        el('div', { className: 'item-sub', text: cli.type }),
      ]));
    });
    detailEl.appendChild(wrap);
  }

  // Parameters
  fetchNodeParameters(id, false);
  const pState = paramCache.get(id) || { loading: true, parameters: [], error: '', showSystem: false };
  const refreshBtn = el('button', {
    type: 'button',
    className: 'btn-small',
    text: pState.loading ? 'Loading...' : 'Refresh',
    onClick: () => fetchNodeParameters(id, true),
  });
  const paramHead = el('div', { className: 'sec-head' }, [
    el('span', { text: `Parameters (${(pState.parameters || []).length})` }),
    refreshBtn,
  ]);
  detailEl.appendChild(paramHead);

  if (pState.error) {
    detailEl.appendChild(el('div', { className: 'alert-box error', text: pState.error }));
  } else if (pState.loading && !(pState.parameters || []).length) {
    detailEl.appendChild(el('div', { className: 'muted-note', text: 'Fetching node parameters...' }));
  } else {
    const allParams = pState.parameters || [];
    const userParams = allParams.filter((p) => !p.name.startsWith('qos_overrides.') && p.name !== 'use_sim_time');
    const sysParams = allParams.filter((p) => p.name.startsWith('qos_overrides.') || p.name === 'use_sim_time');

    const shown = pState.showSystem ? allParams : (userParams.length ? userParams : allParams);
    if (!shown.length) {
      detailEl.appendChild(el('div', { className: 'muted-note', text: 'No parameters declared.' }));
    } else {
      const list = el('div', { className: 'item-list' });
      shown.forEach((param) => list.appendChild(buildParameterRow(id, param)));
      detailEl.appendChild(list);
    }

    if (sysParams.length && userParams.length) {
      const toggleSysBtn = el('button', {
        type: 'button',
        className: 'btn-small',
        style: 'margin-top: 6px',
        text: pState.showSystem ? 'Hide system params' : `Show ${sysParams.length} system/qos params`,
        onClick: () => {
          pState.showSystem = !pState.showSystem;
          renderNodeDetail(id);
        },
      });
      detailEl.appendChild(toggleSysBtn);
    }
  }
}

function buildTopicMonitorSection(topicName, msgType) {
  const isWatching = topicMonitor.active && topicMonitor.topic === topicName;
  const toggleBtn = el('button', {
    type: 'button',
    className: isWatching ? 'btn-small' : 'btn-small btn-primary',
    text: isWatching ? 'Stop monitor' : 'Monitor Live (Hz / Echo)',
  });

  topicStatusNode = el('div', { className: 'item-sub', text: isWatching ? `${topicMonitor.hz} Hz · ${topicMonitor.kbps} KB/s · ${topicMonitor.count} msgs` : 'Click Monitor Live to inspect publish rate and messages.' });
  topicSamplePre = el('pre', { className: 'topic-preview', text: isWatching && topicMonitor.sample !== null ? JSON.stringify(topicMonitor.sample, null, 2) : '(waiting for message)' });
  if (!isWatching) topicSamplePre.hidden = true;

  toggleBtn.addEventListener('click', () => {
    const nextTopic = isWatching ? '' : topicName;
    toggleBtn.disabled = true;
    fetch('/api/topic/watch', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ topic: nextTopic, type: nextTopic ? (msgType || '') : '' }),
    })
      .then((res) => res.json())
      .then((data) => {
        topicMonitor = data;
        toggleBtn.disabled = false;
        if (selected && selected.kind === 'wire') {
          const match = lastRenderState.links.find((l) => l.id === selected.id);
          if (match) renderWireDetail(match);
        } else if (selected && selected.kind === 'topic') {
          renderTopicDetail(topicName);
        }
      })
      .catch((err) => {
        toggleBtn.disabled = false;
        topicStatusNode.textContent = String(err);
      });
  });

  const wrap = el('div', {}, [
    el('div', { className: 'sec-head' }, [
      el('span', { text: 'Live Monitor' }),
      toggleBtn,
    ]),
    topicStatusNode,
    topicSamplePre,
  ]);
  if (isWatching && topicMonitor.error) {
    wrap.appendChild(el('div', { className: 'alert-box error', text: topicMonitor.error }));
  }
  return wrap;
}

function renderWireDetail(link) {
  clearEl(detailEl);
  const titleText = (link.topics && link.topics.length === 1) ? link.topics[0] : (link.topic || `${link.publisher} → ${link.subscriber}`);
  detailEl.appendChild(el('strong', { text: titleText }));
  detailEl.appendChild(el('div', { className: 'item-sub' }, [el('code', { text: link.type || 'unknown' })]));

  const routeBox = el('div', { className: 'item-row', style: 'margin-top: 8px' }, [
    el('div', { text: `${link.publisher} → ${link.subscriber}` }),
  ]);
  if (link.topics && link.topics.length > 1) {
    routeBox.appendChild(el('div', { className: 'item-sub', text: `Topics: ${link.topics.join(', ')}` }));
  }
  detailEl.appendChild(routeBox);

  if (link.compatibility === 'error') {
    detailEl.appendChild(el('div', {
      className: 'alert-box error',
      text: `Silent wire (QoS incompatible): ${link.reason || 'QoS mismatch'}`,
    }));
  } else if (link.compatibility === 'warning') {
    detailEl.appendChild(el('div', {
      className: 'alert-box',
      text: `QoS warning: ${link.reason || 'Potential QoS compatibility warning'}`,
    }));
  } else {
    detailEl.appendChild(el('div', { style: 'margin-top: 8px' }, [
      el('span', { className: 'pill ok', text: 'QoS compatible' }),
    ]));
  }

  if (link.pub_qos || link.sub_qos) {
    const pq = link.pub_qos || {};
    const sq = link.sub_qos || {};
    const table = el('table', { className: 'qos-table' }, [
      el('thead', {}, [el('tr', {}, [el('th', { text: 'Policy' }), el('th', { text: 'Publisher' }), el('th', { text: 'Subscriber' })])]),
      el('tbody', {}, [
        el('tr', {}, [el('td', { text: 'Reliability' }), el('td', { text: pq.reliability || '—' }), el('td', { text: sq.reliability || '—' })]),
        el('tr', {}, [el('td', { text: 'Durability' }), el('td', { text: pq.durability || '—' }), el('td', { text: sq.durability || '—' })]),
        el('tr', {}, [el('td', { text: 'Liveliness' }), el('td', { text: pq.liveliness || '—' }), el('td', { text: sq.liveliness || '—' })]),
        el('tr', {}, [
          el('td', { text: 'Deadline' }),
          el('td', { text: pq.deadline_ns ? `${Math.round(pq.deadline_ns / 1e6)} ms` : '∞' }),
          el('td', { text: sq.deadline_ns ? `${Math.round(sq.deadline_ns / 1e6)} ms` : '∞' }),
        ]),
      ]),
    ]);
    detailEl.appendChild(table);
  }

  const monitorTopic = (link.topics && link.topics.length === 1) ? link.topics[0] : link.topic;
  if (!link.topics || link.topics.length === 1) {
    detailEl.appendChild(buildTopicMonitorSection(monitorTopic, link.type));
  }
}

function renderTopicDetail(topicName) {
  clearEl(detailEl);
  const topic = lastRenderState.topics.find((item) => item.name === topicName);
  if (!topic) {
    detailEl.appendChild(el('p', { className: 'muted-note', text: 'Topic not found.' }));
    return;
  }
  detailEl.appendChild(el('strong', { text: topic.name }));
  detailEl.appendChild(el('div', { className: 'item-sub' }, [el('code', { text: topic.type || 'unknown' })]));

  const addEndList = (title, list) => {
    detailEl.appendChild(el('div', { className: 'sec-head', text: `${title} (${list.length})` }));
    if (!list.length) {
      detailEl.appendChild(el('div', { className: 'muted-note', text: 'none' }));
      return;
    }
    const wrap = el('div', { className: 'item-list' });
    list.forEach((ep) => {
      const id = nodeOf(ep);
      const row = el('div', { className: 'item-row clickable' }, [
        el('div', { text: id }),
        el('div', { className: 'item-sub', text: qosText(ep) }),
      ]);
      row.addEventListener('click', () => {
        selectNode(id);
        centerOnNode(id);
      });
      wrap.appendChild(row);
    });
    detailEl.appendChild(wrap);
  };

  addEndList('Publishers', topic.publishers || []);
  addEndList('Subscribers', topic.subscribers || []);
  detailEl.appendChild(buildTopicMonitorSection(topic.name, topic.type));
}

function renderHealthPanel() {
  const diags = graph.diagnostics || [];
  const errCount = diags.filter((d) => d.level === 'error').length;
  const warnCount = diags.length - errCount;

  if (diags.length > 0) {
    diagPill.hidden = false;
    diagPill.classList.toggle('has-error', errCount > 0);
    diagPill.textContent = errCount > 0 ? `${errCount} err · ${warnCount} warn` : `${warnCount} warn`;
    healthBadgeEl.hidden = false;
    healthBadgeEl.className = `tab-badge ${errCount > 0 ? 'error' : 'warn'}`;
    healthBadgeEl.textContent = String(diags.length);
  } else {
    diagPill.hidden = true;
    healthBadgeEl.hidden = true;
  }

  clearEl(healthListEl);
  if (!diags.length) {
    healthListEl.appendChild(el('p', { className: 'muted-note', text: 'No QoS mismatches or duplicate node warnings detected.' }));
    return;
  }

  const wrap = el('div', { className: 'item-list' });
  diags.forEach((diag) => {
    const row = el('div', { className: `alert-box ${diag.level === 'error' ? 'error' : ''}` }, [
      el('div', { style: 'font-weight: 700', text: `${diag.kind} · ${diag.target}` }),
      el('div', { text: diag.message }),
    ]);
    row.style.cursor = 'pointer';
    row.addEventListener('click', () => {
      if (diag.kind === 'duplicate_node' && lastRenderState.nodeMap.has(diag.target)) {
        selectNode(diag.target);
        centerOnNode(diag.target);
      } else if (lastRenderState.topics.some((t) => t.name === diag.target)) {
        selectTopic(diag.target);
      }
    });
    wrap.appendChild(row);
  });
  healthListEl.appendChild(wrap);
}

function renderLogs() {
  const minLevel = Number(logLevelEl.value || 0);
  const nodeOnly = logNodeOnlyEl.checked && selected && selected.kind === 'node' ? selected.id : '';

  const filtered = logs.filter((entry) => {
    if (minLevel && Number(entry.level || 0) < minLevel) return false;
    if (nodeOnly && entry.name !== nodeOnly) return false;
    return true;
  });

  const warnErrTotal = logs.filter((e) => Number(e.level || 0) >= 30).length;
  logBadgeEl.hidden = warnErrTotal === 0;
  logBadgeEl.textContent = String(warnErrTotal);
  logBadgeEl.className = logs.some((e) => Number(e.level || 0) >= 40) ? 'tab-badge error' : 'tab-badge warn';

  clearEl(logListEl);
  if (!filtered.length) {
    logListEl.appendChild(el('p', { className: 'muted-note', text: 'No matching /rosout logs yet.' }));
    return;
  }

  const recent = filtered.slice(-120).reverse();
  recent.forEach((entry) => {
    const ts = entry.stamp ? new Date(entry.stamp * 1000).toLocaleTimeString() : '';
    const card = el('div', {
      className: `log-entry ${entry.level_name || 'INFO'}`,
      title: entry.file ? `${entry.file}:${entry.line} (${entry.function})` : '',
    }, [
      el('div', { className: 'log-meta' }, [
        el('span', { text: `${entry.level_name || 'INFO'} · ${entry.name || 'unknown'}` }),
        el('span', { text: ts }),
      ]),
      el('div', { text: entry.msg || '' }),
    ]);
    logListEl.appendChild(card);
  });
}

function pushLogEntry(entry) {
  if (!entry || logPaused) return;
  logs.push(entry);
  if (logs.length > 300) logs.shift();
  const lvl = Number(entry.level || 0);
  if (entry.name && lvl >= 30) {
    const cur = nodeLogCounts.get(entry.name) || { warn: 0, error: 0 };
    if (lvl >= 40) cur.error += 1;
    else cur.warn += 1;
    nodeLogCounts.set(entry.name, cur);
  }
  renderLogs();
}

function updateSearchResults() {
  const query = searchInput.value.trim().toLowerCase();
  clearEl(searchResults);
  if (!query) {
    searchResults.hidden = true;
    return;
  }

  const matches = [];
  lastRenderState.ids.forEach((id) => {
    if (id.toLowerCase().includes(query)) matches.push({ kind: 'node', label: id, action: () => { selectNode(id); centerOnNode(id); } });
    const info = lastRenderState.nodeMap.get(id);
    if (info) {
      (info.services || []).filter((s) => !s.system).forEach((s) => {
        if (s.name.toLowerCase().includes(query)) matches.push({ kind: 'service', label: s.name, action: () => { selectNode(id); centerOnNode(id); } });
      });
      (info.actions || []).forEach((a) => {
        if (a.name.toLowerCase().includes(query)) matches.push({ kind: 'action', label: a.name, action: () => { selectNode(id); centerOnNode(id); } });
      });
    }
  });
  lastRenderState.topics.forEach((topic) => {
    if (topic.name.toLowerCase().includes(query)) {
      matches.push({ kind: 'topic', label: topic.name, action: () => selectTopic(topic.name) });
    }
  });

  if (!matches.length) {
    searchResults.appendChild(el('div', { className: 'search-item', text: 'No matches' }));
    searchResults.hidden = false;
    return;
  }

  matches.slice(0, 14).forEach((item) => {
    const btn = el('button', {
      type: 'button',
      className: 'search-item',
      onClick: () => {
        item.action();
        searchResults.hidden = true;
      },
    }, [
      el('span', { text: item.label }),
      el('span', { className: 'search-kind', text: item.kind }),
    ]);
    searchResults.appendChild(btn);
  });
  searchResults.hidden = false;
}

function exportSvgDiagram() {
  const { ids, links, ports } = lastRenderState;
  let minX = 0;
  let minY = 0;
  let maxX = 900;
  let maxY = 560;
  ids.forEach((id) => {
    const pt = positions.get(id);
    if (!pt) return;
    minX = Math.min(minX, pt.x - 40);
    minY = Math.min(minY, pt.y - 50);
    maxX = Math.max(maxX, pt.x + NODE_W + 40);
    maxY = Math.max(maxY, pt.y + nodeHeight(id, ports) + 60);
  });

  const width = Math.round(maxX - minX);
  const height = Math.round(maxY - minY);
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
  svg.setAttribute('viewBox', `${minX} ${minY} ${width} ${height}`);
  svg.setAttribute('width', String(width));
  svg.setAttribute('height', String(height));
  svg.style.fontFamily = '"Segoe UI", "Helvetica Neue", sans-serif';

  const bg = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
  bg.setAttribute('x', minX);
  bg.setAttribute('y', minY);
  bg.setAttribute('width', width);
  bg.setAttribute('height', height);
  bg.setAttribute('fill', '#f7f8fb');
  svg.appendChild(bg);
  svg.appendChild(wireDefs());

  wires.querySelectorAll('.wire, .wire-label').forEach((child) => {
    const clone = child.cloneNode(true);
    if (clone.classList.contains('wire')) {
      clone.setAttribute('fill', 'none');
      clone.setAttribute('stroke', clone.classList.contains('mismatch') ? '#c92a2a' : (clone.classList.contains('warning') ? '#d97706' : '#7e95ad'));
      clone.setAttribute('stroke-width', '2.4');
      if (clone.classList.contains('mismatch') || clone.classList.contains('warning')) {
        clone.setAttribute('stroke-dasharray', '7 4');
      }
    } else {
      clone.setAttribute('font-size', '11');
      clone.setAttribute('fill', '#273445');
    }
    svg.appendChild(clone);
  });

  ids.forEach((id) => {
    const pt = positions.get(id);
    if (!pt) return;
    const h = nodeHeight(id, ports);
    const rect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    rect.setAttribute('x', pt.x);
    rect.setAttribute('y', pt.y);
    rect.setAttribute('width', NODE_W);
    rect.setAttribute('height', h);
    rect.setAttribute('rx', '6');
    rect.setAttribute('fill', colorFor(id));
    svg.appendChild(rect);

    const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    text.setAttribute('x', pt.x + 14);
    text.setAttribute('y', pt.y + h / 2 + 4);
    text.setAttribute('fill', '#ffffff');
    text.setAttribute('font-size', '13');
    text.setAttribute('font-weight', '650');
    text.textContent = id;
    svg.appendChild(text);
  });

  const blob = new Blob([new XMLSerializer().serializeToString(svg)], { type: 'image/svg+xml' });
  const url = URL.createObjectURL(blob);
  const link = el('a', { href: url, download: 'mmr-ros-graph.svg' });
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

// Canvas panning & wheel zoom
let panState = null;

canvas.addEventListener('pointerdown', (event) => {
  if (event.button !== 0 && event.button !== 1) return;
  if (event.target.closest('.node, .ns-group-header, .canvas-controls, .minimap, .wire')) return;
  panState = {
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    viewX: view.x,
    viewY: view.y,
    moved: false,
  };
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener('pointermove', (event) => {
  if (!panState || !canvas.hasPointerCapture(event.pointerId)) return;
  const dx = event.clientX - panState.startX;
  const dy = event.clientY - panState.startY;
  if (Math.abs(dx) + Math.abs(dy) > 4) {
    panState.moved = true;
    canvas.classList.add('panning');
  }
  view.x = panState.viewX + dx;
  view.y = panState.viewY + dy;
  applyViewportTransform();
  drawMinimap();
});

const stopPan = (canceled) => {
  if (!panState) return;
  const moved = panState.moved;
  panState = null;
  canvas.classList.remove('panning');
  if (!moved && !canceled) {
    selected = null;
    syncNodes(lastRenderState.ids, lastRenderState.nodeMap, lastRenderState.ports, lastRenderState.links);
    drawWires(lastRenderState.links, lastRenderState.ports);
    clearEl(detailEl);
    detailEl.appendChild(el('p', {
      className: 'muted-note',
      text: 'Click a node or wire to inspect QoS, services, actions, parameters, or live traffic.',
    }));
  }
};

canvas.addEventListener('pointerup', (event) => {
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  stopPan(false);
});
canvas.addEventListener('lostpointercapture', () => stopPan(true));

canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  const factor = Math.exp(-event.deltaY * 0.0015);
  zoomAround(factor, event.clientX, event.clientY);
}, { passive: false });

// Minimap click/drag navigation
minimapSvg.addEventListener('pointerdown', (event) => {
  const vb = minimapSvg.viewBox.baseVal;
  if (!vb || !vb.width) return;
  const rect = minimapSvg.getBoundingClientRect();
  const wx = vb.x + ((event.clientX - rect.left) / rect.width) * vb.width;
  const wy = vb.y + ((event.clientY - rect.top) / rect.height) * vb.height;
  view.x = canvas.clientWidth / 2 - wx * view.scale;
  view.y = canvas.clientHeight / 2 - wy * view.scale;
  applyViewportTransform();
  drawMinimap();
});

// Controls & shortcuts
[viewModeEl, systemBox, hideLeafBox, hideDeadBox, groupNsBox].forEach((control) => {
  control.addEventListener('change', () => {
    saveState();
    render();
  });
});

zoomInBtn.addEventListener('click', () => zoomAround(1.18));
zoomOutBtn.addEventListener('click', () => zoomAround(1 / 1.18));
zoomResetBtn.addEventListener('click', () => {
  view.scale = 1;
  applyViewportTransform();
  drawMinimap();
});
fitViewBtn.addEventListener('click', fitView);
autoLayoutBtn.addEventListener('click', autoLayout);
toggleMapBtn.addEventListener('click', () => {
  showMinimap = !showMinimap;
  minimapBox.hidden = !showMinimap;
  toggleMapBtn.setAttribute('aria-pressed', showMinimap ? 'true' : 'false');
  drawMinimap();
});

diagPill.addEventListener('click', () => {
  setSidebarOpen(true);
  switchTab('tab-health');
});
exportJsonBtn.addEventListener('click', () => { window.location.href = '/api/export'; });
exportSvgBtn.addEventListener('click', exportSvgDiagram);

toggleInspectorBtn.addEventListener('click', () => setSidebarOpen(!sidebarOpen));
sidebarCollapseBtn.addEventListener('click', () => setSidebarOpen(false));
sidebarPeekBtn.addEventListener('click', () => setSidebarOpen(true));

let resizeDrag = null;
sidebarResizer.addEventListener('pointerdown', (event) => {
  if (event.button !== 0 || !sidebarOpen) return;
  event.preventDefault();
  resizeDrag = { pointerId: event.pointerId, startX: event.clientX, startW: sidebarWidth };
  workspaceEl.classList.add('resizing');
  sidebarResizer.setPointerCapture(event.pointerId);
});

sidebarResizer.addEventListener('pointermove', (event) => {
  if (!resizeDrag || !sidebarResizer.hasPointerCapture(event.pointerId)) return;
  const delta = resizeDrag.startX - event.clientX;
  sidebarWidth = Math.max(280, Math.min(680, Math.round(resizeDrag.startW + delta)));
  workspaceEl.style.setProperty('--sidebar-w', `${sidebarWidth}px`);
  drawMinimap();
});

const stopSidebarResize = () => {
  if (!resizeDrag) return;
  resizeDrag = null;
  workspaceEl.classList.remove('resizing');
  saveState();
};
sidebarResizer.addEventListener('pointerup', (event) => {
  if (sidebarResizer.hasPointerCapture(event.pointerId)) sidebarResizer.releasePointerCapture(event.pointerId);
  stopSidebarResize();
});
sidebarResizer.addEventListener('lostpointercapture', stopSidebarResize);
sidebarResizer.addEventListener('keydown', (event) => {
  if (event.key === 'ArrowLeft') {
    sidebarWidth = Math.min(680, sidebarWidth + 20);
  } else if (event.key === 'ArrowRight') {
    sidebarWidth = Math.max(280, sidebarWidth - 20);
  } else {
    return;
  }
  event.preventDefault();
  applySidebarState();
  saveState();
});

searchInput.addEventListener('input', updateSearchResults);
searchInput.addEventListener('focus', updateSearchResults);
document.addEventListener('click', (event) => {
  if (!event.target.closest('.search-wrap')) searchResults.hidden = true;
});

logLevelEl.addEventListener('change', renderLogs);
logNodeOnlyEl.addEventListener('change', renderLogs);
logPauseBtn.addEventListener('click', () => {
  logPaused = !logPaused;
  logPauseBtn.setAttribute('aria-pressed', logPaused ? 'true' : 'false');
  logPauseBtn.textContent = logPaused ? 'Resume' : 'Pause';
});
logClearBtn.addEventListener('click', () => {
  logs = [];
  nodeLogCounts.clear();
  fetch('/api/logs/clear', { method: 'POST' }).catch(() => {});
  renderLogs();
  render();
});

window.addEventListener('keydown', (event) => {
  if (event.ctrlKey && event.code === 'Space') {
    event.preventDefault();
    setSidebarOpen(!sidebarOpen);
    return;
  }
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'f') {
    event.preventDefault();
    searchInput.focus();
    searchInput.select();
    return;
  }
  if (event.target.matches('input, select, textarea')) {
    if (event.key === 'Escape') {
      searchResults.hidden = true;
      event.target.blur();
    }
    return;
  }
  if (event.key === '/') {
    event.preventDefault();
    searchInput.focus();
  } else if (event.key === '+' || event.key === '=') {
    zoomAround(1.18);
  } else if (event.key === '-') {
    zoomAround(1 / 1.18);
  } else if (event.key === '0') {
    view.scale = 1;
    applyViewportTransform();
    drawMinimap();
  } else if (event.key.toLowerCase() === 'f') {
    fitView();
  } else if (event.key.toLowerCase() === 'm') {
    toggleMapBtn.click();
  }
});

function applyGraph(next) {
  const incoming = next || { nodes: [], topics: [], links: [], diagnostics: [], error: 'empty graph' };
  const lostEveryone = incoming.error && (!incoming.nodes || incoming.nodes.length === 0);
  if (lostEveryone && graph.nodes && graph.nodes.length) {
    counts.textContent = incoming.error;
    live.classList.add('down');
    live.textContent = 'error';
    return;
  }
  graph = incoming;
  render();
}

function applyTopicSample(sample) {
  if (!sample) return;
  topicMonitor = sample;
  const matchesSelected = selected && (
    (selected.kind === 'wire' && selected.topic === sample.topic)
    || (selected.kind === 'topic' && selected.name === sample.topic)
  );
  if (topicStatusNode && matchesSelected && sample.active) {
    topicStatusNode.textContent = `${sample.hz} Hz · ${sample.kbps} KB/s · ${sample.count} msgs`;
    if (topicSamplePre) {
      topicSamplePre.hidden = false;
      topicSamplePre.textContent = sample.sample !== null ? JSON.stringify(sample.sample, null, 2) : '(waiting for message)';
    }
  }
}

loadState();
applySidebarState();
applyViewportTransform();

fetch('/api/graph')
  .then((response) => response.json())
  .then(applyGraph)
  .catch((error) => {
    graph = { nodes: [], topics: [], links: [], diagnostics: [], error: String(error) };
    render();
  });

fetch('/api/logs')
  .then((response) => response.json())
  .then((data) => {
    (data.logs || []).forEach(pushLogEntry);
  })
  .catch(() => {});

const stream = new EventSource('/api/stream');
stream.onmessage = (event) => applyGraph(JSON.parse(event.data));
stream.addEventListener('topic', (event) => applyTopicSample(JSON.parse(event.data)));
stream.addEventListener('log', (event) => pushLogEntry(JSON.parse(event.data)));
stream.onerror = () => {
  live.classList.add('down');
  live.textContent = 'reconnecting';
};
