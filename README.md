# plane-moving-targets (aligned, Vicsek)

A Vicsek-model agent simulation with moving targets — a MATLAB model and a
dependency-light browser port of the same dynamics. The target scenario is
unchanged from the earlier VTP version of this project; what's different is
the **agent dynamics model itself**.

**[Live site →](#)** (enable GitHub Pages, see below)

## What it is

Instead of the VTP model (Voronoi/Delaunay-neighbor repulsion + alignment +
homing, with a Voronoi-cell-based speed governor), agents here follow the
classical **Vicsek model**:

- **Metric neighbors, not topological.** Every agent within a fixed radius
  **R** — full stop. No Delaunay triangulation, no Voronoi cells anywhere
  in this repo.
- **Alignment.** Each agent's new heading is the vector-average heading of
  its neighbors (including itself) — the textbook Vicsek consensus rule.
- **Noise.** A uniform random angular kick of size **&eta;** is added every
  step. This is the actual Vicsek order/disorder parameter: &eta;=0 gives
  clean consensus once aligned; large &eta; gives near-random headings.
  VTP never had an analogue of this at all.
- **Constant speed.** Every agent moves at the same fixed speed every step.
  There's no crowding-based speed governor, because there are no Voronoi
  cells to measure — a defining difference from VTP, where speed
  dynamically slows in crowded cells.

Since bare Vicsek has no notion of a target, homing toward the nearest
target is kept as an extension: each agent's heading is a blend of the
neighbor-consensus direction and the direction to its nearest target,
weighted by **&nu;** — the same role &nu; always played, just blending two
directions instead of three forces.

### Target motion (unchanged from the VTP version)

You choose a number of straight-line targets (0–5), each on its own
fixed-height horizontal lane. One more target is always added: it
oscillates in y, but **asymmetrically** — descending at angular frequency
**q** and rising back up at **n·q** (n times faster), both halves genuine
half-cosine curves joined continuously. Every target — including the
oscillating one — shares a single horizontal speed and a single shared
x-coordinate, so they're aligned (exactly the same x) at every instant.

## Structure

```
index.html              site shell — tabs for about / live simulation / matlab version
assets/style.css        site styling
assets/sim.js           the Vicsek simulation engine (metric-radius alignment + noise + homing, constant speed, canvas rendering) — no external libraries
assets/app.js           page wiring — tabs, sliders, MATLAB source viewer
assets/matlab_src.json  bundled MATLAB source (for the in-page code viewer)
matlab/                 original MATLAB implementation (just 3 files — see below)
```

Unlike the VTP variants of this project, this one needs no vendored
Delaunay library at all — Vicsek's metric-radius neighbor check is a plain
distance comparison, nothing more.

## Running the web version

No build step. Either open `index.html` directly, or serve the folder:

```
python3 -m http.server 8000
```

then visit `http://localhost:8000/`.

## Running the MATLAB version

Requires base MATLAB only — no toolboxes at all. Unlike some other
Vicsek implementations, the pairwise-distance computation here is plain
broadcasting (`X(:,1) - X(:,1)'`, etc.), not `pdist`/`squareform`, so it
doesn't even need the Statistics and Machine Learning Toolbox:

```
cd matlab
matlab -r dynamics
```

or open `matlab/dynamics.m` in the MATLAB editor and run it. You'll be
prompted for the number of straight-line targets (0–5); one
asymmetrically-oscillating target is always added on top. An interactive
figure then opens with a control panel (agents' constant speed v0, &nu;,
R, &eta;, shared horizontal speed, each straight target's height, and the
oscillating target's height/amplitude/q/n — Apply / Pause / Reset Targets).

This folder only ships three MATLAB files: `dynamics.m`, `Target.m`
(generic target geometry, not Voronoi-specific), and `nearestOnSegment.m`
(a small helper `Target.m` depends on internally). Every VTP-only file —
`neighborhoods.m`, `alignTo.m`, `transition.m`,
`voronoiProjectToBoundary.m`, and the Voronoi-diagnostic utilities
(`poly_area.m`, `voronoiPressure.m`, etc.) — has been removed, since none
of them are used or meaningful once the dynamics are Vicsek.

## Publishing to GitHub Pages

1. Create a new GitHub repository and push this folder to it (see commands
   below).
2. In the repo, go to **Settings → Pages**.
3. Under **Build and deployment**, set **Source** to `Deploy from a branch`,
   branch `main`, folder `/ (root)`.
4. Save — the site will be published at
   `https://<your-username>.github.io/<repo-name>/` within a minute or two.

```bash
cd plane-moving-targets-vicsek
git init
git add .
git commit -m "Initial commit: Vicsek aligned moving-targets site"
git branch -M main
git remote add origin https://github.com/<your-username>/<repo-name>.git
git push -u origin main
```

## Note on the JS port

The browser port implements the exact same Vicsek + homing model as
`dynamics.m` — metric-radius vector-average alignment, uniform noise, and
constant speed, blended with homing toward the nearest target weighted by
&nu; — ported line-for-line from MATLAB to JS, including the identical
down/up state machine for the oscillating target. There's no approximation
to note here (unlike the VTP variants' Voronoi-boundary-projection
shortcut): Vicsek's metric-radius neighbor check is exactly the same
computation in both languages.
