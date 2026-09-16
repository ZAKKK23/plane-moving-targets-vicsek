/* ===========================================================
   plane-moving-targets — page glue
   =========================================================== */

/* ---------- tabs ---------- */
const tabButtons = document.querySelectorAll("nav.tabs button");
const views = document.querySelectorAll(".view");
function showView(name) {
  tabButtons.forEach((b) => b.classList.toggle("active", b.dataset.view === name));
  views.forEach((v) => v.classList.toggle("active", v.id === "view-" + name));
  if (name === "sim") sim.canvas.dispatchEvent(new Event("resize-request"));
}
tabButtons.forEach((b) => b.addEventListener("click", () => showView(b.dataset.view)));
window.addEventListener("hashchange", () => {
  const h = location.hash.replace("#", "");
  if (h) showView(h);
});
if (location.hash) showView(location.hash.replace("#", ""));

/* ---------- simulation wiring ---------- */
const canvas = document.getElementById("simCanvas");
const sim = new VicsekSim(canvas, 2);

const statusLine = document.getElementById("statusLine");
const targetRows = document.getElementById("targetRows");
const cellSpdSlider = document.getElementById("cellSpd");
const cellSpdVal = document.getElementById("cellSpdVal");
const nuSlider = document.getElementById("nuSlider");
const nuVal = document.getElementById("nuVal");
const rSlider = document.getElementById("rSlider");
const rVal = document.getElementById("rVal");
const etaSlider = document.getElementById("etaSlider");
const etaVal = document.getElementById("etaVal");
const vxSlider = document.getElementById("vxSlider");
const vxVal = document.getElementById("vxVal");
const btnPause = document.getElementById("btnPause");
const btnReset = document.getElementById("btnReset");
const numTargetsSelect = document.getElementById("numTargets");

function buildTargetRows(targets) {
  const nRect = targets.filter((tg) => !tg.osc).length;
  numTargetsSelect.value = String(nRect);
  targetRows.innerHTML = "";
  targets.forEach((tg, k) => {
    const row = document.createElement("div");
    row.className = "target-row";
    const col = TARGET_COLORS[k % TARGET_COLORS.length];
    if (!tg.osc) {
      row.innerHTML = `
        <span class="tname" style="color:${col}">T${k + 1}</span>
        <div class="ctrl">Height:
          <input type="range" min="-15" max="15" step="0.1" value="${tg.height}" data-k="${k}" data-field="height" />
          <span class="readout" data-readout="height-${k}">${tg.height.toFixed(2)}</span>
        </div>`;
    } else {
      row.innerHTML = `
        <span class="tname" style="color:${col}">T${k + 1}~</span>
        <div class="ctrl">Height:
          <input type="range" min="-15" max="15" step="0.1" value="${tg.height}" data-k="${k}" data-field="height" />
          <span class="readout" data-readout="height-${k}">${tg.height.toFixed(2)}</span>
        </div>
        <div class="ctrl">Amp:
          <input type="range" min="0" max="15" step="0.1" value="${tg.amp}" data-k="${k}" data-field="amp" />
          <span class="readout" data-readout="amp-${k}">${tg.amp.toFixed(2)}</span>
        </div>
        <div class="ctrl">q (down):
          <input type="range" min="0" max="0.3" step="0.001" value="${tg.omega}" data-k="${k}" data-field="omega" />
          <span class="readout" data-readout="omega-${k}">${tg.omega.toFixed(3)}</span>
        </div>
        <div class="ctrl">n (up ×faster):
          <input type="range" min="0.1" max="10" step="0.1" value="${tg.n}" data-k="${k}" data-field="n" />
          <span class="readout" data-readout="n-${k}">${tg.n.toFixed(2)}</span>
        </div>`;
    }
    targetRows.appendChild(row);
  });
  targetRows.querySelectorAll("input[type=range]").forEach((inp) => {
    inp.addEventListener("input", () => {
      const k = +inp.dataset.k, field = inp.dataset.field;
      const val = +inp.value;
      sim.targets[k][field] = val;
      const ro = targetRows.querySelector(`[data-readout="${field}-${k}"]`);
      ro.textContent = field === "omega" ? val.toFixed(3) : val.toFixed(2);
    });
  });
}

sim.onSetupChange = buildTargetRows;
buildTargetRows(sim.targets);

cellSpdSlider.addEventListener("input", () => {
  sim.cellSpd = +cellSpdSlider.value;
  cellSpdVal.textContent = sim.cellSpd.toFixed(2);
});
nuSlider.addEventListener("input", () => {
  sim.nu = +nuSlider.value;
  nuVal.textContent = sim.nu.toFixed(2);
});
rSlider.addEventListener("input", () => {
  sim.R = +rSlider.value;
  rVal.textContent = sim.R.toFixed(2);
});
etaSlider.addEventListener("input", () => {
  sim.eta = +etaSlider.value;
  etaVal.textContent = sim.eta.toFixed(2);
});
vxSlider.addEventListener("input", () => {
  sim.vx = +vxSlider.value;
  vxVal.textContent = sim.vx.toFixed(3);
});

btnPause.addEventListener("click", () => {
  sim.paused = !sim.paused;
  btnPause.textContent = sim.paused ? "Resume" : "Pause";
});
btnReset.addEventListener("click", () => {
  const nRect = sim.nRect;
  sim.setup(nRect);
  cellSpdSlider.value = 1; sim.cellSpd = 1; cellSpdVal.textContent = "1.00";
  nuSlider.value = 2.5; sim.nu = 2.5; nuVal.textContent = "2.50";
  rSlider.value = 3; sim.R = 3; rVal.textContent = "3.00";
  etaSlider.value = 0.3; sim.eta = 0.3; etaVal.textContent = "0.30";
  vxSlider.value = 0.15; sim.vx = 0.15; vxVal.textContent = "0.150";
});
numTargetsSelect.addEventListener("change", () => {
  const n = clamp(Math.round(+numTargetsSelect.value) || 2, 0, 5);
  sim.setup(n);
});

/* ---------- animation loop ---------- */
function loop() {
  sim.step();
  sim.draw();
  const nRect = sim.nRect;
  statusLine.textContent = `t = ${sim.t} | ${sim.targets.length} aligned target${sim.targets.length === 1 ? "" : "s"} (${nRect} straight + 1 asym. sine) | vx = ${sim.vx.toFixed(3)} | Vicsek: R = ${sim.R.toFixed(2)}, \u03b7 = ${sim.eta.toFixed(2)}, \u03bd = ${sim.nu.toFixed(2)}`;
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

/* ---------- matlab file viewer ---------- */
const FILE_NOTES = {
  "dynamics.m": "Main entry point — sets up the figure/UI and runs the Vicsek simulation loop (metric-radius alignment + noise + homing, constant speed).",
  "Target.m": "Target class — target geometry and the homeToTarget homing-vector method. Generic geometry, not Voronoi-specific, so it's the one piece kept from the VTP version.",
  "nearestOnSegment.m": "Geometry helper used internally by Target.m for segment-shaped targets.",
};
const FILE_ORDER = ["dynamics.m", "Target.m", "nearestOnSegment.m"];

const fileList = document.getElementById("fileList");
const codeView = document.getElementById("codeView");
const fileHint = document.getElementById("fileHint");
let MATLAB_SRC = null;

async function loadMatlabSource() {
  try {
    const res = await fetch("assets/matlab_src.json");
    MATLAB_SRC = await res.json();
  } catch (e) {
    MATLAB_SRC = null;
  }
  FILE_ORDER.forEach((name, i) => {
    if (!MATLAB_SRC || !(name in MATLAB_SRC)) return;
    const btn = document.createElement("button");
    btn.textContent = name;
    btn.addEventListener("click", () => selectFile(name));
    fileList.appendChild(btn);
    if (i === 0) selectFile(name);
  });
}
function selectFile(name) {
  [...fileList.children].forEach((b) => b.classList.toggle("active", b.textContent === name));
  codeView.textContent = MATLAB_SRC[name];
  fileHint.textContent = FILE_NOTES[name] || "";
}
loadMatlabSource();
