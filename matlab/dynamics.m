%% ============================================================
%  dynamics.m  —  VICSEK plane simulation with ALIGNED MOVING TARGETS
%
%  The TARGET scenario is unchanged from the VTP version (straight-line
%  lanes + one asymmetrically-oscillating target, all sharing a single
%  x-coordinate). What's different is the AGENT DYNAMICS: this file
%  replaces the VTP model (Voronoi/Delaunay-neighbor repulsion +
%  alignment + homing, with a Voronoi-cell-based speed governor) with
%  the classical Vicsek model:
%
%    - Neighbors are METRIC, not topological: agent i's neighbors are
%      every agent within a fixed Euclidean radius R, full stop — no
%      Delaunay triangulation, no Voronoi cells anywhere in this file.
%    - Each agent's new heading is the (vector) average heading of its
%      neighbors (including itself), same as the textbook Vicsek rule.
%    - A uniform random angular kick of size eta is added every step —
%      eta is the classic Vicsek noise parameter that drives the
%      order/disorder transition (eta=0: perfect consensus once
%      aligned; large eta: near-random headings).
%    - Every agent moves at the SAME constant speed v0 (the "Cells
%      Spd" slider) every step — there is no crowding-based speed
%      governor here, because there are no Voronoi cells to measure.
%
%  Homing toward the nearest target is kept as an extension on top of
%  the bare Vicsek rule (classic Vicsek has no notion of a target at
%  all): each agent's new heading is a blend of the neighbor-consensus
%  direction and the direction to its nearest target, weighted by nu —
%  same role nu always played, just blending two directions now
%  instead of three forces. Target.m (homeToTarget) is the only piece
%  of the old machinery still in use, since it's generic target
%  geometry, not Voronoi-specific.
%
%  A control panel lets you:
%    - Change the cells' (agents') constant speed v0
%    - Change nu (alignment-vs-homing blend), R (the Vicsek interaction
%      radius), and eta (noise strength)
%    - Change the shared horizontal speed (vx) of every target
%    - Change each straight-line target's height
%    - Change the oscillating target's height (oscillation center),
%      amplitude, down-frequency q, and the up-speed multiplier n
%    - Apply the above live, Pause/Resume, and Reset targets
%
%  IMPLEMENTATION NOTE: All button callbacks only set a flag via
%  setappdata on the figure; the flags are read back inside the main
%  loop below. This avoids relying on MATLAB's local-function variable
%  scoping in script files, which is not supported consistently across
%  MATLAB versions/configurations.
%% ============================================================

%% ---- Simulation parameters ----------------------------------
N      = 200;
R      = 3;          % Vicsek interaction radius — editable live (see cur_R below)
nu     = 2.5;         % alignment-vs-homing blend weight — editable live (see cur_nu below)
eta    = 0.3;         % Vicsek noise strength (radians) — editable live (see cur_eta below)
tmax   = 5000;

cell_spd0 = 1;      % v0 — the CONSTANT speed of every agent (Vicsek, not a speed cap)

fixframe   = false;
frameinrad = 50;

x_start = -15;      % shared starting x for every target (same vertical line)
vx0     = 0.15;      % default shared horizontal speed of every target

%% ---- Ask user for number of straight-line targets ------------
% Total targets = nRect (straight-line) + 1 (the oscillating one, added
% automatically). nRect may be 0 (just the oscillating target alone).
try
    nRect_answer = inputdlg('Number of straight-line targets (0-5):', 'Setup', 1, {'2'});
catch
    nRect_answer = {};
end
if isempty(nRect_answer)
    nRect = 2;   % default if dialog cancelled or unavailable
else
    nRect = round(str2double(nRect_answer{1}));
    if isnan(nRect), nRect = 2; end
end
nRect  = max(0, min(5, nRect));
nT     = nRect + 1;     % + the oscillating target
oscIdx = nT;            % the oscillating target is always the last one

%% ---- Initial agent conditions -------------------------------
ic_rad = 0.5 * sqrt(N*pi/4/0.91);
rng(2);
X     = ic_rad * (2*rand(N,2) - 1);
rng(18);
theta = 2*pi * rand(N,1);      % each agent's heading — the Vicsek state variable
U     = cell_spd0 * [cos(theta) sin(theta)];   % only used for the very first plotted frame

%% ---- Initial target parameters --------------------------------
% Straight-line targets: evenly spaced parallel horizontal lines.
if nRect == 0
    heights0 = zeros(0,1);
elseif nRect == 1
    heights0 = 0;
else
    heights0 = linspace(-8, 8, nRect)';
end

% Oscillating target: asymmetric sine — goes DOWN at angular frequency
% q ("Omega"), goes back UP at n*q ("n" times faster). Both halves are
% half-cosine curves, joined continuously at the top/bottom of the
% swing. osc_going_down / osc_localphase are the running state; y is
% recomputed from whichever half is CURRENT after any switch, every
% step, so the join is continuous with no jump.
H_osc0    = 0;
Amp0      = 4;
Omega0    = 0.05;   % q — the DOWN-phase angular frequency
n0        = 2;       % UP-phase is n times faster than the down-phase

osc_going_down = true;   % true = currently descending, false = currently rising
osc_localphase = 0;      % local phase within the current half-swing, in [0,pi)

xCommon   = x_start;    % shared x-coordinate of every target (accumulator)

Tpos = zeros(nT, 2);
for k = 1:nRect
    Tpos(k,:) = [xCommon, heights0(k)];
end
Tpos(oscIdx,:) = [xCommon, H_osc0 + Amp0*cos(osc_localphase)];

%% ---- Build initial Target object ----------------------------
tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

%% ---- Preallocate agent variables ----------------------------
U1 = zeros(N,2);

%% ============================================================
%  Build figure + UI
%% ============================================================
colors = lines(nT);

PANEL_H  = min(0.32 + nT*0.05, 0.62);   % panel grows with nT
fig = figure('Name', 'Vicsek - Aligned Moving Targets', ...
             'NumberTitle', 'off', ...
             'Position', [80 40 980 900]);

ax = axes('Parent', fig, ...
          'Position', [0.05  PANEL_H+0.03  0.90  0.94-PANEL_H]);

pan = uipanel('Parent', fig, ...
              'Title', 'Target Controls', ...
              'Position', [0.01 0.005 0.98 PANEL_H], ...
              'FontSize', 9);

% ---- Cells (agents) speed row ---------------------------------
uicontrol(pan, 'Style','text', ...
    'String', 'Cells', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.90  0.06  0.08]);

uicontrol(pan,'Style','text','String','Spd (v0):', ...
    'Units','normalized','Position',[0.07 0.90 0.08 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

sld_cell = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',3,'Value', cell_spd0, ...
    'Units','normalized','Position',[0.16  0.91  0.26  0.06]);

lbl_cell = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.2f', cell_spd0), ...
    'Units','normalized','Position',[0.43 0.90 0.07 0.08],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_cell,'Value','PostSet', ...
    @(~,~) set(lbl_cell,'String', sprintf('%.2f', sld_cell.Value)));

% ---- Dynamics constants (nu, R, eta) row ------------------------
% nu = alignment-vs-homing blend, R = Vicsek interaction radius (the
% metric neighbor cutoff — replaces the Voronoi/Delaunay neighborhood
% entirely), eta = Vicsek noise strength. All three are just weights/
% scales used every step, so they take effect immediately on Apply.
uicontrol(pan, 'Style','text', ...
    'String', 'Dynamics', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.82  0.09  0.08]);

uicontrol(pan,'Style','text','String','nu:', ...
    'Units','normalized','Position',[0.11 0.82 0.05 0.08],...
    'FontSize',8,'HorizontalAlignment','right');
edt_nu = uicontrol(pan,'Style','edit', ...
    'String', num2str(nu), ...
    'Units','normalized','Position',[0.17  0.825  0.11  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','R:', ...
    'Units','normalized','Position',[0.32 0.82 0.04 0.08],...
    'FontSize',8,'HorizontalAlignment','right');
edt_R = uicontrol(pan,'Style','edit', ...
    'String', num2str(R), ...
    'Units','normalized','Position',[0.37  0.825  0.11  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

uicontrol(pan,'Style','text','String','eta:', ...
    'Units','normalized','Position',[0.52 0.82 0.05 0.08],...
    'FontSize',8,'HorizontalAlignment','right');
edt_eta = uicontrol(pan,'Style','edit', ...
    'String', num2str(eta), ...
    'Units','normalized','Position',[0.58  0.825  0.11  0.065], ...
    'FontSize',8, 'BackgroundColor', [1 1 1]);

% ---- Motion row: shared horizontal speed (vx) ------------------
% Every target — straight-line or oscillating — uses this SAME speed,
% and they all started on the same vertical line, so they always
% share exactly the same x-coordinate at every step.
uicontrol(pan, 'Style','text', ...
    'String', 'Motion', ...
    'FontWeight','bold', 'FontSize', 9, ...
    'Units','normalized', ...
    'Position', [0.01  0.74  0.09  0.08]);

uicontrol(pan,'Style','text','String','Horiz. speed (vx, shared):', ...
    'Units','normalized','Position',[0.10 0.74 0.24 0.08],...
    'FontSize',8,'HorizontalAlignment','right');

sld_vx = uicontrol(pan,'Style','slider', ...
    'Min',0,'Max',1,'Value', vx0, ...
    'Units','normalized','Position',[0.35  0.75  0.30  0.06]);

lbl_vx = uicontrol(pan,'Style','text', ...
    'String', sprintf('%.3f', vx0), ...
    'Units','normalized','Position',[0.66 0.74 0.08 0.08],...
    'FontSize',8,'HorizontalAlignment','left');

addlistener(sld_vx,'Value','PostSet', ...
    @(~,~) set(lbl_vx,'String', sprintf('%.3f', sld_vx.Value)));

% ---- Per-target rows -------------------------------------------
% Rows 1..nRect: straight-line targets — label + Height slider.
% Row nT (last): the oscillating target — Height (center) + Amp + q + n.
row_h = 0.58 / nT;   % fractional height inside panel, shared by all rows

sld_height = gobjects(nRect,1);
lbl_height = gobjects(nRect,1);
sld_oscH = []; lbl_oscH = [];
sld_oscA = []; lbl_oscA = [];
sld_oscW = []; lbl_oscW = [];
sld_oscN = []; lbl_oscN = [];

for k = 1:nT
    y0 = 0.74 - k * row_h;   % bottom of this row

    if k <= nRect
        % ---- straight-line target: label + Height ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.04  row_h*0.7]);

        uicontrol(pan,'Style','text','String','Height:', ...
            'Units','normalized','Position',[0.05 y0 0.08 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');

        sld_height(k) = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', heights0(k), ...
            'Units','normalized','Position',[0.13  y0+0.01  0.35  row_h*0.55]);

        lbl_height(k) = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', heights0(k)), ...
            'Units','normalized','Position',[0.49 y0 0.10 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        kk = k;   % capture loop variable
        addlistener(sld_height(k),'Value','PostSet', ...
            @(~,~) set(lbl_height(kk),'String', sprintf('%.2f', sld_height(kk).Value)));
    else
        % ---- the oscillating target: Height (center) + Amp + q (down-freq) + n (up-speed multiplier) ----
        uicontrol(pan, 'Style','text', ...
            'String', sprintf('T%d~', k), ...
            'ForegroundColor', colors(k,:), ...
            'FontWeight','bold', 'FontSize', 9, ...
            'Units','normalized', ...
            'Position', [0.01  y0  0.045  row_h*0.7]);

        uicontrol(pan,'Style','text','String','H:', ...
            'Units','normalized','Position',[0.06 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscH = uicontrol(pan,'Style','slider', ...
            'Min',-15,'Max',15,'Value', H_osc0, ...
            'Units','normalized','Position',[0.09  y0+0.01  0.14  row_h*0.55]);
        lbl_oscH = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', H_osc0), ...
            'Units','normalized','Position',[0.23 y0 0.05 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','A:', ...
            'Units','normalized','Position',[0.29 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscA = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',15,'Value', Amp0, ...
            'Units','normalized','Position',[0.32  y0+0.01  0.14  row_h*0.55]);
        lbl_oscA = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', Amp0), ...
            'Units','normalized','Position',[0.46 y0 0.05 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','q:', ...
            'Units','normalized','Position',[0.52 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscW = uicontrol(pan,'Style','slider', ...
            'Min',0,'Max',0.3,'Value', Omega0, ...
            'Units','normalized','Position',[0.55  y0+0.01  0.14  row_h*0.55]);
        lbl_oscW = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.3f', Omega0), ...
            'Units','normalized','Position',[0.69 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        uicontrol(pan,'Style','text','String','n:', ...
            'Units','normalized','Position',[0.76 y0 0.03 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','right');
        sld_oscN = uicontrol(pan,'Style','slider', ...
            'Min',0.1,'Max',10,'Value', n0, ...
            'Units','normalized','Position',[0.79  y0+0.01  0.14  row_h*0.55]);
        lbl_oscN = uicontrol(pan,'Style','text', ...
            'String', sprintf('%.2f', n0), ...
            'Units','normalized','Position',[0.93 y0 0.06 row_h*0.7],...
            'FontSize',8,'HorizontalAlignment','left');

        addlistener(sld_oscH,'Value','PostSet', ...
            @(~,~) set(lbl_oscH,'String', sprintf('%.2f', sld_oscH.Value)));
        addlistener(sld_oscA,'Value','PostSet', ...
            @(~,~) set(lbl_oscA,'String', sprintf('%.2f', sld_oscA.Value)));
        addlistener(sld_oscW,'Value','PostSet', ...
            @(~,~) set(lbl_oscW,'String', sprintf('%.3f', sld_oscW.Value)));
        addlistener(sld_oscN,'Value','PostSet', ...
            @(~,~) set(lbl_oscN,'String', sprintf('%.2f', sld_oscN.Value)));
    end
end

% ---- Buttons at the bottom of the panel ----------------------
% Callbacks ONLY set a flag via setappdata; no nested functions, no
% closures over script variables needed.
setappdata(fig, 'apply_clicked', false);
setappdata(fig, 'reset_clicked', false);

btn_apply = uicontrol(pan,'Style','pushbutton','String','Apply', ...
    'FontSize',9,'FontWeight','bold', ...
    'Units','normalized','Position',[0.01 0.02 0.12 0.14], ...
    'Callback', @(src,evt) setappdata(fig,'apply_clicked',true));

btn_pause = uicontrol(pan,'Style','togglebutton','String','Pause', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.15 0.02 0.12 0.14]);

btn_reset = uicontrol(pan,'Style','pushbutton','String','Reset Targets', ...
    'FontSize',9, ...
    'Units','normalized','Position',[0.29 0.02 0.16 0.14], ...
    'Callback', @(src,evt) setappdata(fig,'reset_clicked',true));

%% ---- Mutable simulation variables (plain script variables) --
cur_cell_spd = cell_spd0;
cur_nu       = nu;
cur_R        = R;
cur_eta      = eta;
cur_vx       = vx0;
cur_height   = heights0;   % nRect x 1
cur_H_osc    = H_osc0;
cur_Amp      = Amp0;
cur_Omega    = Omega0;
cur_n        = n0;

%% ============================================================
%  Main simulation loop
%% ============================================================
t = 0;
while t < tmax && ishandle(fig)

    % Pause
    while ishandle(fig) && get(btn_pause,'Value')
        pause(0.05);
    end
    if ~ishandle(fig), break; end

    % ---- Check Apply button ----
    if getappdata(fig, 'apply_clicked')
        setappdata(fig, 'apply_clicked', false);
        cur_cell_spd = get(sld_cell, 'Value');
        cur_vx       = get(sld_vx, 'Value');
        for k = 1:nRect
            cur_height(k) = get(sld_height(k), 'Value');
        end
        if nRect < nT
            cur_H_osc = get(sld_oscH, 'Value');
            cur_Amp   = get(sld_oscA, 'Value');
            cur_Omega = get(sld_oscW, 'Value');
            cur_n     = get(sld_oscN, 'Value');
        end

        % ---- Alignment-vs-homing blend nu: applies immediately ----
        newNu = str2double(get(edt_nu, 'String'));
        if isnan(newNu) || newNu < 0
            newNu = cur_nu;   % invalid entry: keep current value
        end
        cur_nu = newNu;
        set(edt_nu, 'String', num2str(cur_nu));

        % ---- Vicsek interaction radius R: applies immediately too ----
        newR = str2double(get(edt_R, 'String'));
        if isnan(newR) || newR <= 0
            newR = cur_R;   % invalid entry: keep current value
        end
        cur_R = newR;
        set(edt_R, 'String', num2str(cur_R));

        % ---- Vicsek noise strength eta: applies immediately too ----
        newEta = str2double(get(edt_eta, 'String'));
        if isnan(newEta) || newEta < 0
            newEta = cur_eta;   % invalid entry: keep current value
        end
        cur_eta = newEta;
        set(edt_eta, 'String', num2str(cur_eta));
    end

    % ---- Check Reset button ----
    if getappdata(fig, 'reset_clicked')
        setappdata(fig, 'reset_clicked', false);

        xCommon        = x_start;
        osc_going_down = true;
        osc_localphase = 0;
        cur_cell_spd   = cell_spd0;
        cur_vx         = vx0;
        cur_height     = heights0;
        cur_H_osc      = H_osc0;
        cur_Amp        = Amp0;
        cur_Omega      = Omega0;
        cur_n          = n0;

        set(sld_cell,'Value', cell_spd0);
        set(lbl_cell,'String', sprintf('%.2f', cell_spd0));
        set(sld_vx,'Value', vx0);
        set(lbl_vx,'String', sprintf('%.3f', vx0));
        for k = 1:nRect
            set(sld_height(k),'Value', heights0(k));
            set(lbl_height(k),'String', sprintf('%.2f', heights0(k)));
        end
        if nRect < nT
            set(sld_oscH,'Value', H_osc0);
            set(lbl_oscH,'String', sprintf('%.2f', H_osc0));
            set(sld_oscA,'Value', Amp0);
            set(lbl_oscA,'String', sprintf('%.2f', Amp0));
            set(sld_oscW,'Value', Omega0);
            set(lbl_oscW,'String', sprintf('%.3f', Omega0));
            set(sld_oscN,'Value', n0);
            set(lbl_oscN,'String', sprintf('%.2f', n0));
        end
        t = 0;
    end

    t = t + 1;

    %% -- Move targets (aligned rectilinear + one asymmetric sinusoid) --
    % Every target shares the SAME x — one accumulator, not one per
    % target — so they are aligned (same x at every instant) by
    % construction, not just at t=0.
    xCommon = xCommon + cur_vx;

    % Asymmetric oscillation: advance the current half-swing at its own
    % rate (q going down, n*q going up), and switch halves on overflow.
    % y is then computed from whichever half is CURRENT after any
    % switch — not the pre-switch one — so the join stays continuous
    % (no jump) even though the two halves run at different speeds.
    if osc_going_down
        osc_localphase = osc_localphase + cur_Omega;
        if osc_localphase >= pi
            osc_localphase = 0;
            osc_going_down = false;
        end
    else
        osc_localphase = osc_localphase + cur_n*cur_Omega;
        if osc_localphase >= pi
            osc_localphase = 0;
            osc_going_down = true;
        end
    end
    if osc_going_down
        y_osc   = cur_H_osc + cur_Amp*cos(osc_localphase);
        vy_osc  = -cur_Amp*cur_Omega*sin(osc_localphase);
    else
        y_osc   = cur_H_osc - cur_Amp*cos(osc_localphase);
        vy_osc  =  cur_Amp*cur_n*cur_Omega*sin(osc_localphase);
    end

    for k = 1:nRect
        Tpos(k,:) = [xCommon, cur_height(k)];
    end
    Tpos(oscIdx,:) = [xCommon, y_osc];

    tar = Target( mat2cell(Tpos, ones(nT,1), 2) );

    % velocities used only for the on-screen arrows (analytic, not integrated)
    Tvel = zeros(nT,2);
    for k = 1:nRect
        Tvel(k,:) = [cur_vx, 0];
    end
    Tvel(oscIdx,:) = [cur_vx, vy_osc];

    %% -- Agent dynamics: VICSEK (metric-radius alignment + noise) ----
    %  plus homing, at CONSTANT speed. No Delaunay/Voronoi anywhere.

    % pairwise Euclidean distances via broadcasting (base MATLAB only —
    % avoids pdist/squareform, which need the Statistics Toolbox)
    dxp = X(:,1) - X(:,1)';
    dyp = X(:,2) - X(:,2)';
    Ddist = sqrt(dxp.^2 + dyp.^2);

    % metric neighborhood: everyone (including self) within radius R
    nbrMask = Ddist <= cur_R;

    % Vicsek alignment: (vector) average heading of all neighbors
    ux = cos(theta); uy = sin(theta);
    sumx = nbrMask * ux;
    sumy = nbrMask * uy;
    alignDir = [sumx, sumy];
    alignNorm = vecnorm(alignDir, 2, 2);
    alignDir  = alignDir ./ alignNorm;
    alignDir(alignNorm < eps, :) = 0;   % degenerate case: neighbors' headings exactly cancel

    % homing toward the nearest target (Target.m — generic geometry,
    % nothing Voronoi-specific about it)
    h0     = homeToTarget(tar, X);
    hNorm  = vecnorm(h0, 2, 2);
    hDir   = h0 ./ hNorm;
    hDir(hNorm < eps, :) = 0;   % degenerate case: sitting exactly on the target

    % blend alignment consensus with homing bias (nu weights alignment
    % vs. homing, same role it always played)
    blend      = cur_nu*alignDir + hDir;
    blendTheta = atan2(blend(:,2), blend(:,1));

    % Vicsek noise: independent uniform angular kick per agent, in
    % [-eta/2, +eta/2]
    noiseKick = cur_eta * (rand(N,1) - 0.5);
    theta     = blendTheta + noiseKick;

    U1 = [cos(theta), sin(theta)];   % new heading, also used for the plotted arrows

    %% -- Plot -------------------------------------------------
    if ishandle(fig)
        % Compute a square frame that contains both agents and targets
        if fixframe
            cx = 0; cy = 0; hw = frameinrad;
        else
            com  = mean(X);
            rmed = sqrt(median((X(:,1)-com(1)).^2 + (X(:,2)-com(2)).^2));
            cx   = com(1);  cy = com(2);  hw = 3*rmed;
        end
        all_pts = [X; Tpos];
        xlo = min(cx-hw, min(all_pts(:,1))-2);
        xhi = max(cx+hw, max(all_pts(:,1))+2);
        ylo = min(cy-hw, min(all_pts(:,2))-2);
        yhi = max(cy+hw, max(all_pts(:,2))+2);
        hw2 = max(xhi-xlo, yhi-ylo)/2;
        cx2 = (xlo+xhi)/2;  cy2 = (ylo+yhi)/2;

        cla(ax);
        set(ax, 'XLim', [cx2-hw2, cx2+hw2], ...
                'YLim', [cy2-hw2, cy2+hw2], ...
                'DataAspectRatio', [1 1 1], ...
                'NextPlot', 'add');

        % Agents
        scatter(ax, X(:,1), X(:,2), 4, 'k', 'filled');
        quiver(ax, X(:,1), X(:,2), U1(:,1), U1(:,2), ...
               'b', 'AutoScaleFactor', 0.5, 'MaxHeadSize', 0.3);

        % Moving targets
        theta_c = linspace(0, 2*pi, 40);
        r_c     = 0.7;
        vscale  = 8;
        for k = 1:nT
            fill(ax, Tpos(k,1) + r_c*cos(theta_c), ...
                     Tpos(k,2) + r_c*sin(theta_c), ...
                     colors(k,:), 'EdgeColor','none', 'FaceAlpha', 0.9);
            quiver(ax, Tpos(k,1), Tpos(k,2), ...
                       vscale*Tvel(k,1), vscale*Tvel(k,2), ...
                       'off', 'Color', colors(k,:)*0.55, 'LineWidth', 2, ...
                       'MaxHeadSize', 0.6);
            if k == oscIdx
                lbl = sprintf('T%d~', k);
            else
                lbl = sprintf('T%d', k);
            end
            text(ax, Tpos(k,1), Tpos(k,2), lbl, ...
                 'Color','w','FontWeight','bold', ...
                 'HorizontalAlignment','center','FontSize',7);
        end

        set(ax, 'NextPlot', 'replace');
        title(ax, sprintf('t = %d    |    %d aligned targets (%d straight + 1 asym. sine)    |    vx = %.3f, q = %.3f, n = %.2f    |    Vicsek: R = %.2f, eta = %.2f, \nu = %.2f', ...
              t, nT, nRect, cur_vx, cur_Omega, cur_n, cur_R, cur_eta, cur_nu), 'FontSize', 9);
        drawnow limitrate;
    end

    %% -- Update agent state: CONSTANT speed v0 for every agent -------
    % (Vicsek has no crowding-based speed governor — every agent always
    % moves at the same speed; only its heading is stochastic.)
    U = cur_cell_spd * U1;
    X = X + U;

end   % end main loop
