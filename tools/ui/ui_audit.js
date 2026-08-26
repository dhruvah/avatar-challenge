/* UI audit: build a fake DOM with real per-id elements, load the designer's
   script, then click things and assert the resulting state. Catches the class
   of bug where a button leaves the model or the view inconsistent. */
const fs = require("fs");

// ---------------------------------------------------------------- fake DOM
// A real recording 2D context, so tests can count draw calls.
function mkCtx() {
  const noop = () => {};
  const c = {
    calls: { moveTo: 0, lineTo: 0, arc: 0, fillRect: 0, stroke: 0 },
    save: noop, restore: noop, setTransform: noop, clearRect: noop,
    strokeRect: noop, rect: noop, clip: noop, ellipse: noop, quadraticCurveTo: noop,
    bezierCurveTo: noop, textAlign: "left", textBaseline: "alphabetic", font: "",
    lineCap: "butt", lineJoin: "miter", lineWidth: 1, globalAlpha: 1,
    beginPath: noop, closePath: noop, fill: noop, setLineDash: noop,
    createLinearGradient: () => ({ addColorStop: noop }),
    measureText: () => ({ width: 10 }), fillText: noop, strokeText: noop,
    translate: noop, rotate: noop, scale: noop, drawImage: noop,
    moveTo() { this.calls.moveTo++; },
    lineTo() { this.calls.lineTo++; },
    arc() { this.calls.arc++; },
    fillRect() { this.calls.fillRect++; },
    stroke() { this.calls.stroke++; },
  };
  return c;
}

function mkEl(id, tag) {
  const el = {
    id: id || "", tagName: (tag || "DIV").toUpperCase(),
    style: {}, dataset: {}, value: "", checked: false,
    textContent: "", innerHTML: "", disabled: false, hidden: false,
    children: [], _attrs: {}, _listeners: {},
    classList: {
      _s: new Set(),
      add(c) { this._s.add(c); }, remove(c) { this._s.delete(c); },
      // the real classList.toggle returns the resulting state; code relies on it
      toggle(c, on) {
        const next = on === undefined ? !this._s.has(c) : !!on;
        next ? this._s.add(c) : this._s.delete(c);
        return next;
      },
      contains(c) { return this._s.has(c); },
    },
    setAttribute(k, v) { this._attrs[k] = String(v); },
    getAttribute(k) { return this._attrs[k] ?? null; },
    addEventListener(t, f) { (this._listeners[t] ||= []).push(f); },
    removeEventListener() {},
    appendChild(c) { this.children.push(c); return c; },
    remove() {}, select() {}, focus() {}, blur() {},
    getBoundingClientRect: () => ({ width: 900, height: 600, left: 0, top: 0 }),
    setPointerCapture() {},
    getContext: () => mkCtx(),
    querySelector(sel) { return REG.bySel(sel)[0] || mkEl("", "div"); },
    querySelectorAll(sel) { return REG.bySel(sel); },
    click() { this.onclick && this.onclick({ currentTarget: this, target: this,
                                             stopPropagation() {}, preventDefault() {} }); },
  };
  return el;
}

const REG = {
  ids: new Map(), groups: new Map(),
  get(id) { if (!this.ids.has(id)) this.ids.set(id, mkEl(id)); return this.ids.get(id); },
  group(sel, els) { this.groups.set(sel, els); },
  bySel(sel) { return this.groups.get(sel) || []; },
};

// tool rail buttons, mirroring the markup
const TOOLS = ["select", "path", "rect", "circle", "poly", "star", "pan"].map(t => {
  const e = mkEl("", "button"); e.dataset.tool = t; return e;
});
REG.group(".tool", TOOLS);
REG.group("#presets .chip", []);          // presets were removed
REG.group("#modeSw .chip", []);

global.document = {
  getElementById: id => REG.get(id),
  querySelector: sel => REG.bySel(sel)[0] || null,
  querySelectorAll: sel => REG.bySel(sel),
  createElement: tag => mkEl("", tag),
  documentElement: mkEl("html"),
  body: mkEl("body"),
  activeElement: null,
  addEventListener() {},
};
global.window = { devicePixelRatio: 1 };
global.addEventListener = () => {};
global.matchMedia = () => ({ matches: false, addEventListener() {} });
global.getComputedStyle = () => ({ getPropertyValue: () => "#000000" });
global.localStorage = { getItem: () => null, setItem() {} };
global.navigator = { clipboard: { writeText: async () => {} } };
global.clearTimeout = () => {};
global.setTimeout = (f) => 0;
global.setInterval = () => 0;
global.clearInterval = () => {};
global.fetch = () => Promise.reject(new Error("offline in tests"));
global.prompt = () => null;

// ---------------------------------------------------------------- load app
const src = fs.readFileSync(__dirname + "/shape_designer.html", "utf8");
const js = src.split("<script>")[1].split("</scr" + "ipt>")[0].replace('"use strict";', "");
eval(js);

// ---------------------------------------------------------------- harness
let fails = 0, checks = 0;
function chk(name, cond, extra) {
  checks++;
  if (!cond) { fails++; console.log("  FAIL:", name, extra !== undefined ? `(${extra})` : ""); }
}
function pickTool(name) { TOOLS.find(t => t.dataset.tool === name).click(); }
function press(id) { REG.get(id).click(); }
function nShapes() { return shapes.length; }

console.log("UI audit\n");

// --- baseline -------------------------------------------------------------
chk("starts with the seeded square", nShapes() === 1, nShapes());
chk("a shape is selected", sel() !== null);
chk("export is valid JSON", (() => { try { JSON.parse(buildJSON()); return true; } catch { return false; } })());

// --- tool switching -------------------------------------------------------
for (const t of ["path", "rect", "circle", "poly", "star", "pan", "select"]) {
  pickTool(t);
  chk(`tool '${t}' becomes active`, tool === t, tool);
}
pickTool("select");
chk("tool switching does not change shape count", nShapes() === 1, nShapes());
chk("tool switching keeps the selection", sel() !== null);

// --- duplicate ------------------------------------------------------------
const beforeDup = nShapes();
press("btnDup");
chk("duplicate adds one shape", nShapes() === beforeDup + 1, nShapes());
chk("duplicate selects the copy", sel() && /_copy$/.test(sel().name), sel() && sel().name);
chk("duplicate is independent of the original", (() => {
  const orig = shapes[0], copy = shapes[1];
  copy.verts[0].x += 5;
  const ok = orig.verts[0].x !== copy.verts[0].x;
  copy.verts[0].x -= 5;
  return ok;
})());
chk("duplicate does not share the pose object", shapes[0].pose !== shapes[1].pose);

// --- delete ---------------------------------------------------------------
const beforeDel = nShapes();
press("btnDel");
chk("delete removes one shape", nShapes() === beforeDel - 1, nShapes());
chk("delete leaves a valid selection", nShapes() === 0 || sel() !== null);
chk("export still valid after delete", (() => { try { JSON.parse(buildJSON()); return true; } catch { return false; } })());

// --- delete down to empty -------------------------------------------------
while (nShapes() > 0) press("btnDel");
chk("can delete every shape", nShapes() === 0, nShapes());
chk("selection is null when empty", sel() === null);
chk("export of an empty canvas is valid", (() => {
  try { return JSON.parse(buildJSON()).shapes.length === 0; } catch { return false; }
})());
let threw = null;
try { syncPanel(); draw(); press("btnReset"); press("btnDup"); press("btnDel"); }
catch (e) { threw = e; }
chk("buttons are safe with nothing selected", threw === null, threw && threw.message);

// --- import restores state ------------------------------------------------
const FIXTURE = JSON.stringify({ shapes: [
  { name: "sq", vertices: [[0,0],[0,0.1],[0.1,0.1],[0.1,0]], closed: true,
    start_pose: { position: [0.3,-0.05,0.25], rpy: [0,0,0.785] }, speed: 0.5 },
  { name: "tri", vertices: [[0,0],[0.09,0],[0.045,0.08]], closed: true,
    start_pose: { position: [0.34,0,0.3], rpy: [1.5708,0,0] }, speed: 1 },
]});
importJSON(FIXTURE);
chk("import loads both shapes", nShapes() === 2, nShapes());
chk("import selects the first", sel() && sel().name === "sq", sel() && sel().name);
chk("import round-trips speed", Math.abs(shapes[0].speed - 50) < 1, shapes[0].speed);
chk("import recovers tilt for a wall plane", Math.round(shapes[1].pose.tilt) === 90,
    shapes[1].pose.tilt);
chk("import recovers tilt for a table plane", Math.round(shapes[0].pose.tilt) === 0,
    shapes[0].pose.tilt);
chk("re-export matches the import", (() => {
  const a = JSON.parse(FIXTURE), b = JSON.parse(buildJSON());
  return a.shapes.every((sh, i) =>
    sh.start_pose.position.every((v, k) => Math.abs(v - b.shapes[i].start_pose.position[k]) < 1e-6));
})());

// --- selecting rows -------------------------------------------------------
selId = shapes[1].id; selVtx = -1; syncPanel(); draw();
chk("selecting the second shape works", sel().name === "tri");

// --- view reset -----------------------------------------------------------
view.ox = 123; view.oy = -77; view.scale = 5.5; cam.az = 2.2; cam.el = 1.1;
press("btnReset");
chk("reset restores the 3D camera azimuth", Math.abs(cam.az - (-0.6)) < 1e-9, cam.az);
chk("reset restores the 3D camera elevation", Math.abs(cam.el - 0.42) < 1e-9, cam.el);
chk("reset re-fits the canvas (scale changed)", view.scale !== 5.5, view.scale);
chk("reset re-centres the canvas", view.ox !== 123, view.ox);

// --- reach overlay toggle -------------------------------------------------
const before = showReach;
press("btnReach");
chk("reach overlay toggles", showReach === !before);
press("btnReach");
chk("reach overlay toggles back", showReach === before);

// --- orientation controls -------------------------------------------------
selId = shapes[0].id; syncPanel();
const s0 = sel();
REG.get("oTilt").value = "90"; REG.get("oTilt").oninput({ target: REG.get("oTilt") });
chk("tilt slider updates the model", Math.round(s0.pose.tilt) === 90, s0.pose.tilt);
chk("tilt updates the exported rpy", Math.abs(s0.pose.roll - 90) < 1e-6, s0.pose.roll);
REG.get("nTilt").value = "45"; REG.get("nTilt").oninput({ target: REG.get("nTilt") });
chk("tilt number box updates the model", Math.round(s0.pose.tilt) === 45, s0.pose.tilt);
chk("Facing is enabled when tilted", REG.get("oFace").disabled === false);
REG.get("oTilt").value = "0"; REG.get("oTilt").oninput({ target: REG.get("oTilt") });
chk("Facing is disabled on a flat plane", REG.get("oFace").disabled === true);
chk("a note explains why Facing is off", /flat/.test(REG.get("faceNote").textContent),
    REG.get("faceNote").textContent);
REG.get("oSpin").value = "30"; REG.get("oSpin").oninput({ target: REG.get("oSpin") });
chk("Spin still works on a flat plane", Math.round(s0.pose.spin) === 30, s0.pose.spin);

// --- shape options --------------------------------------------------------
REG.get("sClosed").checked = false; REG.get("sClosed").onchange({ target: REG.get("sClosed") });
chk("closed toggle reaches the export", JSON.parse(buildJSON()).shapes[0].closed === false);
REG.get("sClosed").checked = true; REG.get("sClosed").onchange({ target: REG.get("sClosed") });
REG.get("sSpeed").value = "25"; REG.get("sSpeed").oninput({ target: REG.get("sSpeed") });
chk("speed slider reaches the export",
    Math.abs(JSON.parse(buildJSON()).shapes[0].speed - 0.25) < 1e-9);
REG.get("sName").value = "renamed";
REG.get("sName")._listeners.input[0]({ target: REG.get("sName") });
chk("rename reaches the export", JSON.parse(buildJSON()).shapes[0].name === "renamed");

// --- clear all ------------------------------------------------------------
press("btnClear");
chk("clear all empties the canvas", nShapes() === 0, nShapes());
chk("clear all clears the selection", sel() === null);
threw = null; try { draw(); syncPanel(); } catch (e) { threw = e; }
chk("drawing an empty canvas is safe", threw === null, threw && threw.message);

// --- arm rendering --------------------------------------------------------
const ap = armPoints(HOME_Q);
chk("arm has a point per chain joint plus the base", ap.length === CHAIN.length + 1, ap.length);
chk("arm starts at the origin", ap[0].every(v => Math.abs(v) < 1e-12));
const reach = Math.hypot(...ap[ap.length - 1]);
chk("arm tip at home is inside the workspace", reach > 0.2 && reach < 0.9, reach.toFixed(3));
chk("arm moves when joints change", (() => {
  const a = armPoints(HOME_Q), b = armPoints(HOME_Q.map((v, i) => i === 0 ? v + 0.5 : v));
  return Math.hypot(...a[4].map((v, k) => v - b[4][k])) > 1e-3;
})());

// --- live trace overlay ---------------------------------------------------
importJSON(FIXTURE);
live.path = [[0,0],[10,10],[20,0]];
live.shape = "sq";
selId = shapes.find(x => x.name === "sq").id;
ctx.calls.moveTo = 0;
drawLive();
chk("live path draws over the shape it belongs to", ctx.calls.moveTo > 0, ctx.calls.moveTo);

// Regression: the robot reports the path relative to the shape's FIRST VERTEX
// (that is what start_pose pins), not the canvas origin. Drawing it without
// adding that offset back put the whole trace in the wrong place on screen.
(() => {
  const sq = shapes.find(x => x.name === "sq");
  selId = sq.id;
  // move the shape well away from the canvas origin
  const dx = 250, dy = -180;
  sq.verts.forEach(v => { v.x += dx; v.y += dy; });
  live.shape = "sq";
  live.path = [[0, 0], [0, 100], [100, 100]];   // as the robot reports it
  const seen = [];
  const realMove = ctx.moveTo, realLine = ctx.lineTo;
  ctx.moveTo = function (x, y) { seen.push([x, y]); };
  ctx.lineTo = function (x, y) { seen.push([x, y]); };
  drawLive();
  ctx.moveTo = realMove; ctx.lineTo = realLine;
  const expect = mm2px(sq.verts[0].x, sq.verts[0].y);
  const got = seen[0];
  const err = got ? Math.hypot(got[0] - expect[0], got[1] - expect[1]) : 1e9;
  chk("live path starts at the shape's first vertex, not the canvas origin",
      err < 0.5, err.toFixed(1) + "px off");
  const originPx = mm2px(0, 0);
  const atOrigin = got && Math.hypot(got[0] - originPx[0], got[1] - originPx[1]) < 0.5;
  chk("live path is NOT anchored to the canvas origin", !atOrigin);
  sq.verts.forEach(v => { v.x -= dx; v.y -= dy; });
})();
ctx.calls.moveTo = 0;
selId = shapes.find(x => x.name === "tri").id;
drawLive();
chk("live path is hidden over a different shape", ctx.calls.moveTo === 0, ctx.calls.moveTo);
ctx.calls.moveTo = 0;
shapes = []; selId = null;
drawLive();
chk("live path is safe with nothing selected", ctx.calls.moveTo === 0, ctx.calls.moveTo);

// --- disconnected shapes ---------------------------------------------------
importJSON(JSON.stringify({ shapes: [
  { name: "a", vertices: [[0,0],[0.08,0],[0.08,0.05],[0,0.05]], closed: true,
    start_pose: { position: [0.30,-0.10,0.26], rpy: [0,0,0] } },
  { name: "b", vertices: [[0,0],[0.07,0],[0.09,0.06],[0.01,0.05]], closed: true,
    start_pose: { position: [0.30,0.03,0.26], rpy: [0,0,0] } },
]}));
chk("two disconnected shapes both load", nShapes() === 2, nShapes());
const ex = JSON.parse(buildJSON());
chk("each disconnected shape keeps its own start_pose",
    ex.shapes[0].start_pose.position[1] !== ex.shapes[1].start_pose.position[1]);
chk("each disconnected shape starts at (0,0)",
    ex.shapes.every(sh => sh.vertices[0][0] === 0 && sh.vertices[0][1] === 0));
chk("shapes are exported as separate entries, not merged",
    ex.shapes.length === 2, ex.shapes.length);

// --- orientation defaults (regression) ------------------------------------
// tilt/facing/spin were once defaulted on the SHAPE rather than the POSE, so a
// new shape had pose.tilt === undefined. Moving one slider computed
// tfsToRpy(v, undefined, undefined) -> NaN, and the controls only appeared to
// work once all three had been touched.
(() => {
  shapes = []; selId = null;
  const s = newShape("fresh", [{x:0,y:0,kind:"line"},{x:50,y:0,kind:"line"},{x:50,y:40,kind:"line"}]);
  shapes.push(s); selId = s.id; syncPanel();
  for (const k of ["tilt", "facing", "spin", "roll", "pitch", "yaw", "x", "y", "z"]) {
    chk(`a new shape's pose defines ${k}`, typeof s.pose[k] === "number", s.pose[k]);
  }
  // move ONE control and make sure nothing becomes NaN
  const t = document.getElementById("oTilt");
  t.value = "69"; t.oninput({ target: t });
  chk("tilt alone keeps rpy finite",
      [s.pose.roll, s.pose.pitch, s.pose.yaw].every(Number.isFinite),
      [s.pose.roll, s.pose.pitch, s.pose.yaw].join(","));
  chk("tilt alone actually tilts the plane", Math.abs(s.pose.roll - 69) < 1e-6, s.pose.roll);
  const w = toWorld(50, 30, s.pose);
  chk("tilt alone keeps world coordinates finite", w.every(Number.isFinite), w.join(","));
  // facing alone, from a fresh shape
  const s2 = newShape("fresh2", [{x:0,y:0,kind:"line"},{x:50,y:0,kind:"line"},{x:50,y:40,kind:"line"}]);
  shapes.push(s2); selId = s2.id; syncPanel();
  s2.pose.tilt = 45; applyTfs(s2);
  const f = document.getElementById("oFace");
  f.value = "-64"; f.oninput({ target: f });
  chk("facing alone keeps rpy finite",
      [s2.pose.roll, s2.pose.pitch, s2.pose.yaw].every(Number.isFinite));
  chk("facing alone changes the plane", Math.abs(s2.pose.yaw + 64) < 1e-6 || s2.pose.yaw !== 0,
      s2.pose.yaw);
  chk("export from a fresh shape has finite rpy",
      JSON.parse(buildJSON()).shapes.every(sh => sh.start_pose.rpy.every(Number.isFinite)));
})();

// --- rpy readout must not mislabel units ----------------------------------
(() => {
  const s = sel();
  s.pose.tilt = 90; s.pose.facing = 0; s.pose.spin = 0; applyTfs(s); syncPanel();
  const txt = document.getElementById("rpyOut").textContent;
  const exported = JSON.parse(buildJSON()).shapes.slice(-1)[0].start_pose.rpy;
  chk("readout shows the exported radian value", txt.includes(exported[0].toFixed(4)), txt);
  chk("readout marks the degree values as degrees", txt.includes("\u00B0"), txt);
})();

// --- empty canvas keeps the workspace visible (regression) ----------------
// Clearing every shape used to take the reach overlay with it, because the
// overlay needs a plane to map canvas -> world and bailed with no selection.
(() => {
  importJSON(FIXTURE);
  const s = sel();
  s.pose.tilt = 40; s.pose.x = 350; applyTfs(s); syncPanel();
  ctx.calls.fillRect = 0; draw();
  const withShape = ctx.calls.fillRect;
  chk("workspace shading is drawn with a shape present", withShape > 0, withShape);

  press("btnClear");
  chk("clear all really empties the canvas", nShapes() === 0);
  ctx.calls.fillRect = 0; draw();
  const withoutShape = ctx.calls.fillRect;
  chk("workspace shading survives clear all", withoutShape > 0, withoutShape);
  chk("the remembered plane is the one last used",
      Math.abs(lastPose.tilt - 40) < 1e-9 && Math.abs(lastPose.x - 350) < 1e-9,
      `${lastPose.tilt}, ${lastPose.x}`);

  // and the panel should be blank rather than showing the deleted shape
  chk("shape controls are disabled when nothing is selected",
      document.getElementById("sName").disabled === true);
  chk("stale rpy text is replaced",
      /no shape selected/.test(document.getElementById("rpyOut").textContent),
      document.getElementById("rpyOut").textContent);
  chk("stale tilt wording is cleared",
      document.getElementById("tiltWord").textContent === "");

  // drawing again re-enables everything
  importJSON(FIXTURE);
  chk("controls come back when a shape is selected",
      document.getElementById("sName").disabled === false);
})();

// --- curve interiors are checked, not just the vertices --------------------
(() => {
  shapes = []; selId = null;
  // an arc whose endpoints are fine but which bulges far out of reach
  const sh = newShape("bulge", [
    { x: 0, y: 0, kind: "line" },
    { x: 100, y: 0, kind: "arc", cx: 50, cy: 0, cw: false },
  ], { x: 600, y: 0, z: 300, tilt: 0, facing: 0, spin: 0 });
  shapes.push(sh); selId = sh.id;
  const vertClasses = sh.verts.map(v => worldQuality(toWorld(v.x, v.y, sh.pose), sh.pose.tilt));
  const allClasses = flatten(sh).map(p => worldQuality(toWorld(p[0], p[1], sh.pose), sh.pose.tilt));
  chk("tessellated sampling sees at least as many points as the vertices",
      allClasses.length > vertClasses.length, `${allClasses.length} vs ${vertClasses.length}`);
  const worstVerts = vertClasses.slice().sort()[0];
  const worstAll = allClasses.slice().sort()[0];
  chk("outline sampling is never more optimistic than vertex sampling",
      worstAll <= worstVerts, `${worstAll} vs ${worstVerts}`);
  syncPanel();   // must not throw for a shape built from segments
})();

// --- measurements ----------------------------------------------------------
(() => {
  shapes = []; selId = null;
  const sq = newShape("m", [
    { x: 0, y: 0, kind: "line" }, { x: 100, y: 0, kind: "line" },
    { x: 100, y: 60, kind: "line" }, { x: 0, y: 60, kind: "line" }]);
  shapes.push(sq); selId = sq.id;
  const m = measure(sq);
  chk("bounding box width is right", Math.abs(m.w - 100) < 1e-9, m.w);
  chk("bounding box height is right", Math.abs(m.h - 60) < 1e-9, m.h);
  chk("closed perimeter is right", Math.abs(m.len - 320) < 1e-6, m.len);
  sq.closed = false;
  chk("open path length drops the closing edge",
      Math.abs(measure(sq).len - 260) < 1e-6, measure(sq).len);
  sq.closed = true;
  draw();          // the size readout is refreshed as part of rendering
  chk("status bar reports the size",
      /100 \u00D7 60 mm/.test(document.getElementById("stSize").textContent),
      document.getElementById("stSize").textContent);
})();

// --- Spin must be legible on a canvas that cannot itself rotate ------------
// The canvas is the plane's own frame, so the drawing never moves when the
// plane turns. The compass is what makes Spin visible; if it stops tracking,
// Spin silently looks broken again.
(() => {
  shapes = []; selId = null;
  const sh = newShape("c", [{x:0,y:0,kind:"line"},{x:80,y:0,kind:"line"},{x:80,y:50,kind:"line"}]);
  shapes.push(sh); selId = sh.id;
  sh.pose.tilt = 0; sh.pose.facing = 0; sh.pose.spin = 0; applyTfs(sh); syncPanel();

  const before = worldDirInPlane(sh.pose, 0);
  const pxBefore = JSON.stringify(sh.verts.map(v => mm2px(v.x, v.y)));
  sh.pose.spin = -81; applyTfs(sh); syncPanel();
  const after = worldDirInPlane(sh.pose, 0);
  const pxAfter = JSON.stringify(sh.verts.map(v => mm2px(v.x, v.y)));

  const dot = before[0]*after[0] + before[1]*after[1];
  const deg = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
  chk("compass turns by the spin angle", Math.abs(deg - 81) < 0.5, deg.toFixed(2));
  chk("the drawing itself does not move on canvas (it is the plane's frame)",
      pxBefore === pxAfter);
  chk("world coordinates DO move with spin", (() => {
    sh.pose.spin = 0; applyTfs(sh);
    const w0 = toWorld(50, 30, sh.pose);
    sh.pose.spin = -81; applyTfs(sh);
    const w1 = toWorld(50, 30, sh.pose);
    return Math.hypot(w0[0]-w1[0], w0[1]-w1[1]) > 0.01;
  })());
  chk("an edge-on axis yields null rather than NaN", (() => {
    sh.pose.tilt = 90; applyTfs(sh);
    return [0,1].every(i => { const d = worldDirInPlane(sh.pose, i);
      return d === null || d.every(Number.isFinite); });
  })());
  chk("the base arrow has a direction", baseDirInPlane(sh.pose) !== null);
})();

// --- Send is gated on reachability, not just on a robot being present ------
(() => {
  shapes = []; selId = null;
  const near = newShape("near", [{x:0,y:0,kind:"line"},{x:60,y:0,kind:"line"},
                                 {x:60,y:40,kind:"line"}],
                        {x:350, y:0, z:300, tilt:0, facing:0, spin:0});
  shapes.push(near); selId = near.id; applyTfs(near);
  robotOnline = true; syncPanel();
  const send = REG.get("btnSend");
  chk("reachable shape + robot -> Send enabled", send.disabled === false);
  chk("reason line says it is reachable",
      /reachable/i.test(REG.get("reachNote").textContent),
      REG.get("reachNote").textContent);

  // push it far outside the workspace
  near.pose.x = 1500; applyTfs(near); syncPanel();
  chk("unreachable shape -> Send disabled even with a robot", send.disabled === true);
  chk("reason names the shape and the count",
      /near:.*out of reach/.test(REG.get("reachNote").textContent),
      REG.get("reachNote").textContent);

  // back in range, but no robot
  near.pose.x = 350; applyTfs(near); robotOnline = false; syncPanel();
  chk("no robot -> Send disabled", send.disabled === true);
  chk("reason distinguishes 'no robot' from 'out of reach'",
      /waiting for a robot/i.test(REG.get("reachNote").textContent),
      REG.get("reachNote").textContent);

  shapes = []; selId = null; syncPanel();
  chk("empty canvas -> Send disabled", send.disabled === true);
})();

// --- Facing reads N/A on a flat plane -------------------------------------
(() => {
  shapes = []; selId = null;
  const sh = newShape("f", [{x:0,y:0,kind:"line"},{x:50,y:0,kind:"line"},
                            {x:50,y:30,kind:"line"}]);
  shapes.push(sh); selId = sh.id;
  sh.pose.tilt = 0; applyTfs(sh); syncPanel();
  chk("flat plane: Facing box is blank with an N/A placeholder",
      REG.get("nFace").value === "" && REG.get("nFace").placeholder === "N/A",
      `${REG.get("nFace").value}|${REG.get("nFace").placeholder}`);
  sh.pose.tilt = 45; applyTfs(sh); syncPanel();
  chk("tilted plane: Facing shows a number again",
      REG.get("nFace").placeholder === "" && REG.get("nFace").disabled === false);
})();

// --- camera presets and zoom ----------------------------------------------
(() => {
  cam.az = 9; cam.el = 9; cam.zoom = 9;
  Object.assign(cam, CAM_PRESETS.top);
  chk("Top preset looks down", cam.el > 1.0, cam.el);
  Object.assign(cam, CAM_PRESETS.front);
  chk("Front preset is level and along +X", Math.abs(cam.az) < 1e-9 && cam.el < 0.2);
  Object.assign(cam, CAM_PRESETS.side);
  chk("Side preset is a quarter turn round", Math.abs(cam.az + 1.57) < 1e-6);
  chk("presets reset zoom", cam.zoom === 1, cam.zoom);
})();

// --- preview expand ---------------------------------------------------------
(() => {
  const v = REG.get("view3d"), btn = REG.get("btnExpand");
  chk("preview starts collapsed", v.classList.contains("big") === false);
  btn.click();
  chk("Expand grows the preview", v.classList.contains("big") === true);
  chk("button offers to collapse", btn.textContent === "Collapse", btn.textContent);
  btn.click();
  chk("Collapse restores it", v.classList.contains("big") === false);
  chk("button offers to expand again", btn.textContent === "Expand", btn.textContent);
})();

console.log(`\n${checks} checks, ${fails} failure(s)`);
process.exit(fails ? 1 : 0);
