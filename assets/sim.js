/* ===========================================================
   Vicsek — Aligned Moving Targets  |  live simulation
   Ported from the MATLAB model (dynamics.m / Target.m). The TARGET
   scenario is unchanged from the VTP version of this project
   (straight-line lanes + one asymmetrically-oscillating target, all
   sharing a single x-coordinate). What's different is the AGENT
   DYNAMICS: this replaces the VTP model (Voronoi/Delaunay-neighbor
   repulsion + alignment + homing, with a Voronoi-cell speed governor)
   with the classical Vicsek model:
     - Neighbors are METRIC, not topological: every agent within a
       fixed Euclidean radius R, full stop — no Delaunay triangulation,
       no Voronoi cells anywhere in this file.
     - Each agent's new heading is the vector-average heading of its
       neighbors (including itself) — the textbook Vicsek rule.
     - A uniform random angular kick of size eta is added every step —
       the classic Vicsek noise parameter driving the order/disorder
       transition.
     - Every agent moves at the SAME constant speed v0 every step —
       there is no crowding-based speed governor, because there are no
       Voronoi cells to measure.
   Homing toward the nearest target is kept as an extension on top of
   bare Vicsek (which has no notion of a target at all): each agent's
   heading is a blend of the neighbor-consensus direction and the
   direction to its nearest target, weighted by nu.
   =========================================================== */

const TARGET_COLORS = ["#4c8bf5", "#f5a742", "#4cd97d", "#f2685f", "#c792ea", "#3fd0c9"];

const PARAMS = {
  N: 170,             // number of agents ("cells")
  NU_DEFAULT: 2.5,     // default alignment-vs-homing blend weight (live-editable: sim.nu)
  R_DEFAULT: 3,         // default Vicsek interaction radius (live-editable: sim.R)
  ETA_DEFAULT: 0.3,      // default Vicsek noise strength, radians (live-editable: sim.eta)
  VX_DEFAULT: 0.15,    // default shared horizontal speed of every target
  X_START: -15,        // shared starting x for every target (same vertical line)
  HEIGHT_RANGE: 8,     // straight-line targets are spaced across [-HEIGHT_RANGE, HEIGHT_RANGE]
  OSC_AMP_DEFAULT: 4,
  OSC_OMEGA_DEFAULT: 0.05,   // q — the DOWN-phase angular frequency
  OSC_N_DEFAULT: 2,           // UP-phase is n times faster than the down-phase
};

function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

class VicsekSim {
  constructor(canvas, nRect = 2) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.cellSpd = 1.0;             // v0 — the CONSTANT speed of every agent
    this.nu = PARAMS.NU_DEFAULT;     // alignment-vs-homing blend — live-editable
    this.R = PARAMS.R_DEFAULT;       // Vicsek interaction radius — live-editable
    this.eta = PARAMS.ETA_DEFAULT;   // Vicsek noise strength — live-editable
    this.vx = PARAMS.VX_DEFAULT;     // shared horizontal speed of every target — live-editable
    this.paused = false;
    this.t = 0;
    this.cam = null; // {cx,cy,hw} smoothed camera
    this.setup(nRect);
  }

  // nRect = number of straight-line targets (0-5). One more target — the
  // oscillating one — is always added, so total targets = nRect + 1.
  setup(nRect) {
    nRect = clamp(Math.round(nRect), 0, 5);
    this.nRect = nRect;
    const N = PARAMS.N;

    // ---- agents ----
    const icRad = 0.5 * Math.sqrt((N * Math.PI) / 4 / 0.91);
    this.X = new Float64Array(N * 2);
    this.theta = new Float64Array(N);   // each agent's heading — the Vicsek state variable
    this.U = new Float64Array(N * 2);    // derived each step from theta, used for plotting
    for (let i = 0; i < N; i++) {
      this.X[2 * i] = icRad * (2 * Math.random() - 1);
      this.X[2 * i + 1] = icRad * (2 * Math.random() - 1);
      this.theta[i] = 2 * Math.PI * Math.random();
      this.U[2 * i] = this.cellSpd * Math.cos(this.theta[i]);
      this.U[2 * i + 1] = this.cellSpd * Math.sin(this.theta[i]);
    }

    // ---- targets: nRect straight horizontal lines + 1 oscillating ----
    this.targets = [];
    const H = PARAMS.HEIGHT_RANGE;
    for (let k = 0; k < nRect; k++) {
      const height = nRect === 1 ? 0 : -H + (2 * H * k) / (nRect - 1);
      this.targets.push({ osc: false, height });
    }
    this.targets.push({
      osc: true,
      height: 0,                          // oscillation center
      amp: PARAMS.OSC_AMP_DEFAULT,
      omega: PARAMS.OSC_OMEGA_DEFAULT,     // q — down-phase angular frequency
      n: PARAMS.OSC_N_DEFAULT,              // up-phase is n times faster
      goingDown: true,                      // true = descending, false = rising
      localPhase: 0,                        // phase within the current half-swing, in [0,pi)
    });
    this.oscIdx = this.targets.length - 1;

    this.xCommon = PARAMS.X_START;   // shared x — one value for every target
    this.t = 0;
    this.cam = null;
    this.onSetupChange && this.onSetupChange(this.targets);
  }

  targetPos(tg) {
    if (!tg.osc) return [this.xCommon, tg.height];
    const y = tg.goingDown
      ? tg.height + tg.amp * Math.cos(tg.localPhase)
      : tg.height - tg.amp * Math.cos(tg.localPhase);
    return [this.xCommon, y];
  }
  targetVel(tg) {
    if (!tg.osc) return [this.vx, 0];
    const vy = tg.goingDown
      ? -tg.amp * tg.omega * Math.sin(tg.localPhase)
      : tg.amp * tg.n * tg.omega * Math.sin(tg.localPhase);
    return [this.vx, vy];
  }

  step() {
    if (this.paused) return;
    this.t++;

    // Every target shares the SAME x — one accumulator, not one per
    // target — so they are aligned (same x at every instant) by
    // construction, not just at t=0.
    this.xCommon += this.vx;
    for (const tg of this.targets) {
      if (!tg.osc) continue;
      // Advance the current half-swing at its own rate (q going down,
      // n*q going up), and switch halves on overflow. Position is then
      // read from whichever half is CURRENT after any switch (see
      // targetPos/targetVel above) — not the pre-switch one — so the
      // join stays continuous even though the two halves run at
      // different speeds.
      if (tg.goingDown) {
        tg.localPhase += tg.omega;
        if (tg.localPhase >= Math.PI) { tg.localPhase = 0; tg.goingDown = false; }
      } else {
        tg.localPhase += tg.n * tg.omega;
        if (tg.localPhase >= Math.PI) { tg.localPhase = 0; tg.goingDown = true; }
      }
    }

    const N = PARAMS.N;
    const X = this.X, theta = this.theta;
    const tpos = this.targets.map((tg) => this.targetPos(tg));
    const R2 = this.R * this.R;

    const newTheta = new Float64Array(N);

    for (let i = 0; i < N; i++) {
      const xi = X[2 * i], yi = X[2 * i + 1];

      // Vicsek alignment: vector-average heading of every agent
      // (including self) within metric radius R — no Delaunay, no
      // Voronoi, just a plain distance check against everyone.
      let sumx = 0, sumy = 0;
      for (let j = 0; j < N; j++) {
        const dx = xi - X[2 * j], dy = yi - X[2 * j + 1];
        if (dx * dx + dy * dy <= R2) {
          sumx += Math.cos(theta[j]);
          sumy += Math.sin(theta[j]);
        }
      }
      let alignDir = [0, 0];
      const alignNorm = Math.hypot(sumx, sumy);
      if (alignNorm > 1e-12) alignDir = [sumx / alignNorm, sumy / alignNorm];

      // homing toward the nearest target
      let hbest = null, hbestD2 = Infinity;
      for (const tp of tpos) {
        const dx = tp[0] - xi, dy = tp[1] - yi;
        const d2 = dx * dx + dy * dy;
        if (d2 < hbestD2) { hbestD2 = d2; hbest = [dx, dy]; }
      }
      let hDir = [0, 0];
      const hNorm = Math.hypot(hbest[0], hbest[1]);
      if (hNorm > 1e-12) hDir = [hbest[0] / hNorm, hbest[1] / hNorm];

      // blend alignment consensus with homing bias (nu weights
      // alignment vs. homing, same role it always played)
      const bx = this.nu * alignDir[0] + hDir[0];
      const by = this.nu * alignDir[1] + hDir[1];
      const blendTheta = Math.atan2(by, bx);

      // Vicsek noise: independent uniform angular kick, in [-eta/2, +eta/2]
      const noiseKick = this.eta * (Math.random() - 0.5);
      newTheta[i] = blendTheta + noiseKick;
    }

    // Update agent state: CONSTANT speed v0 for every agent (Vicsek has
    // no crowding-based speed governor — only heading is stochastic).
    for (let i = 0; i < N; i++) {
      theta[i] = newTheta[i];
      const ux = Math.cos(theta[i]), uy = Math.sin(theta[i]);
      this.U[2 * i] = this.cellSpd * ux;
      this.U[2 * i + 1] = this.cellSpd * uy;
      X[2 * i] += this.U[2 * i];
      X[2 * i + 1] += this.U[2 * i + 1];
    }
  }

  updateCamera() {
    const N = PARAMS.N;
    const X = this.X;
    let sx = 0, sy = 0;
    for (let i = 0; i < N; i++) { sx += X[2 * i]; sy += X[2 * i + 1]; }
    const cx0 = sx / N, cy0 = sy / N;
    let ss = 0;
    for (let i = 0; i < N; i++) {
      const dx = X[2 * i] - cx0, dy = X[2 * i + 1] - cy0;
      ss += dx * dx + dy * dy;
    }
    const rmed = Math.sqrt(ss / N);
    let hw = Math.max(3 * rmed, PARAMS.HEIGHT_RANGE * 1.2);

    let xlo = cx0 - hw, xhi = cx0 + hw, ylo = cy0 - hw, yhi = cy0 + hw;
    for (const tg of this.targets) {
      const [tx, ty] = this.targetPos(tg);
      xlo = Math.min(xlo, tx - 2);
      xhi = Math.max(xhi, tx + 2);
      ylo = Math.min(ylo, ty - 2);
      yhi = Math.max(yhi, ty + 2);
    }
    const hw2 = Math.max(xhi - xlo, yhi - ylo) / 2;
    const cx2 = (xlo + xhi) / 2, cy2 = (ylo + yhi) / 2;

    if (!this.cam) this.cam = { cx: cx2, cy: cy2, hw: hw2 };
    const a = 0.05;
    this.cam.cx += (cx2 - this.cam.cx) * a;
    this.cam.cy += (cy2 - this.cam.cy) * a;
    this.cam.hw += (hw2 - this.cam.hw) * a;
  }

  draw() {
    const canvas = this.canvas, ctx = this.ctx;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr; canvas.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    this.updateCamera();
    const { cx, cy, hw } = this.cam;
    const scale = Math.min(w, h) / (2 * hw);
    const toPx = (x, y) => [w / 2 + (x - cx) * scale, h / 2 - (y - cy) * scale];

    // targets: filled dot, velocity arrow, label ("T3~" for the oscillating one)
    const rC = 0.7 * scale;
    for (let k = 0; k < this.targets.length; k++) {
      const tg = this.targets[k];
      const col = TARGET_COLORS[k % TARGET_COLORS.length];
      const [tgx, tgy] = this.targetPos(tg);
      const [tx, ty] = toPx(tgx, tgy);
      const [vx, vy] = this.targetVel(tg);

      // velocity arrow
      const vscale = 8;
      const [ax1, ay1] = toPx(tgx + vscale * vx, tgy + vscale * vy);
      ctx.strokeStyle = col; ctx.globalAlpha = 0.7; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(ax1, ay1); ctx.stroke();
      ctx.globalAlpha = 1;

      // target dot
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.arc(tx, ty, Math.max(rC, 9), 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#0a0c12";
      ctx.font = "700 9px " + getComputedStyle(document.body).getPropertyValue("--mono");
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText((tg.osc ? "T" + (k + 1) + "~" : "T" + (k + 1)), tx, ty + 0.5);
    }

    // agents: dot + velocity arrow
    const N = PARAMS.N;
    ctx.fillStyle = "#f4f5f7";
    for (let i = 0; i < N; i++) {
      const [px, py] = toPx(this.X[2 * i], this.X[2 * i + 1]);
      ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2); ctx.fill();
    }
    ctx.strokeStyle = "#5aa4f2"; ctx.fillStyle = "#5aa4f2"; ctx.lineWidth = 1;
    ctx.globalAlpha = 0.85;
    const vscale = 4.2;
    for (let i = 0; i < N; i++) {
      const x0 = this.X[2 * i], y0 = this.X[2 * i + 1];
      const ux = this.U[2 * i], uy = this.U[2 * i + 1];
      const [px0, py0] = toPx(x0, y0);
      const [px1, py1] = toPx(x0 + ux * vscale, y0 + uy * vscale);
      ctx.beginPath(); ctx.moveTo(px0, py0); ctx.lineTo(px1, py1); ctx.stroke();
      const ang = Math.atan2(py1 - py0, px1 - px0);
      ctx.beginPath();
      ctx.moveTo(px1, py1);
      ctx.lineTo(px1 - 4.5 * Math.cos(ang - 0.4), py1 - 4.5 * Math.sin(ang - 0.4));
      ctx.lineTo(px1 - 4.5 * Math.cos(ang + 0.4), py1 - 4.5 * Math.sin(ang + 0.4));
      ctx.closePath(); ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
}
