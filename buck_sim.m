%% =====================================================================
%  buck_sim.m
%  Simplified CLOSED-LOOP BUCK REGULATOR  (averaged, control-oriented model)
%  Project : Predictive Oscillation Detection for Switching Power Regulators
%  Tool    : GNU Octave (tested on 8.x)
%
%  WHAT THIS FILE DOES
%    1. Simulates VOUT(t) of a buck regulator under five conditions:
%         stable, transient, ringing, growing, oscillation
%    2. Plots VOUT vs time for each condition  (PNG files are saved too)
%    3. Writes one CSV per condition:   time_s, Vout_V, iL_A, duty, Rload_ohm
%
%  ALL RESULTS ARE SIMULATED.  Nothing here is measured hardware data.
%
%  HOW TO RUN
%    octave buck_sim.m            (from a terminal)
%    or open Octave and type:  buck_sim
%  Output goes to the folder  ./data/
%  =====================================================================
1;   % <- tells Octave this is a SCRIPT file that also contains functions

%% ---------------------------------------------------------------------
%  SECTION 1 : PHYSICAL PARAMETERS OF THE REGULATOR
%  ---------------------------------------------------------------------
function P = default_params()
  P.Vin   = 12;        % [V]   input voltage (e.g. 12 V supply)
  P.Vref  = 5;         % [V]   target output voltage (regulator setpoint)
  P.fsw   = 100e3;     % [Hz]  switching frequency (used for ripple estimate only)
  P.L     = 100e-6;    % [H]   inductor
  P.C     = 220e-6;    % [F]   output capacitor
  P.RL    = 0.05;      % [ohm] inductor winding resistance (DCR) - gives a little natural damping
  P.Dmax  = 0.95;      % duty-cycle limit (controller cannot ask for more)
  P.Ipos  = 2.0;       % [A]   peak inductor-current limit (protection circuit)
  P.Ineg  = -0.5;      % [A]   reverse-current limit (synchronous rectifier protection)
  P.Tf    = 16e-6;     % [s]   time-constant of the derivative filter (~10 kHz corner)
  P.R0    = 10;        % [ohm] nominal load  (5 V / 10 ohm = 0.5 A)

  % Small random jitter on the load current.  Real loads are never perfectly
  % constant.  It is also the "seed" that lets an unstable loop start to
  % oscillate (in a perfect model, an unstable loop sitting exactly at its
  % equilibrium would never move).
  P.i_noise_rms = 15e-3;  % [A]   RMS of load-current jitter (3% of 0.5 A)
  P.noise_hold  = 50e-6;  % [s]   jitter changes every 50 us
  P.seed        = 1;      % fixed seed -> results are repeatable

  % Simulation numerics
  P.dt   = 2e-6;       % [s]   integration step (500 kHz)  << resonance period (~0.7 ms)
  P.Tend = 0.200;      % [s]   length of each simulation
  P.dec  = 5;          % store every 5th step -> saved sample rate = 1/(5*dt) = 100 kS/s
end

%% ---------------------------------------------------------------------
%  SECTION 2 : THE SIMULATOR  (averaged buck + PID voltage controller)
%
%  AVERAGED MODEL  -  the two equations we actually integrate:
%
%      L * diL/dt = d*Vin - vC - RL*iL          (inductor:  voltage = L di/dt)
%      C * dvC/dt = iL - vC/R - i_jitter        (capacitor: current = C dv/dt)
%
%    d  = duty cycle (0..Dmax) = fraction of each switching period the
%         high-side switch is ON.  Averaged over one period, the switch node
%         voltage is d*Vin.  THIS AVERAGING IS THE SIMPLIFICATION.
%    iL = inductor current,  vC = capacitor voltage = VOUT (ESR neglected)
%
%  CONTROLLER (voltage-mode PID):
%      e = Vref - Vout
%      d = Kp*e + I + Kd * d/dt(e)         with   dI/dt = Ki*e
%    Kp : reacts to present error         Ki : removes the steady error
%    Kd : reacts to how fast the error changes -> ADDS DAMPING
%
%  Scenario struct S fields:
%    S.g0     = [Kp Ki Kd]  controller gains at the start ("healthy")
%    S.g1     = [Kp Ki Kd]  gains after degradation
%    S.t_deg  = [t0 t1]     gains change linearly from g0 to g1 between t0 and t1
%                           (t1 == t0 means an abrupt change)
%    S.load_t = [..]        times at which the load resistance changes
%    S.load_R = [..]        load resistance from that time onward
%% ---------------------------------------------------------------------
function out = simulate_buck(P, S)
  dt = P.dt;  N = round(P.Tend / dt);
  t  = (0:N-1) * dt;

   % --- gain schedule: alpha = 0 (healthy) ... 1 (degraded) -------------
  alpha = zeros(1, N);
  t0d = S.t_deg(1);
  t1d = S.t_deg(2);
  for k = 1:N
    tk = t(k);
    if t1d <= t0d
      % Abrupt change at t0d
      if tk >= t0d
        alpha(k) = 1;
      else
        alpha(k) = 0;
      end
    else
      if tk < t0d
        alpha(k) = 0;
      elseif tk >= t1d
        alpha(k) = 1;
      else
        alpha(k) = (tk - t0d) / (t1d - t0d);
      end
    end
  end
  Kp = S.g0(1) + alpha * (S.g1(1) - S.g0(1));
  Ki = S.g0(2) + alpha * (S.g1(2) - S.g0(2));
  Kd = S.g0(3) + alpha * (S.g1(3) - S.g0(3));

   % --- load resistance versus time -------------------------------------
  Rk = P.R0 * ones(1, N);
  lt = S.load_t;
  lR = S.load_R;
  for i = 1:numel(lt)
    ti = lt(i);
    Ri = lR(i);
    idx = find(t >= ti);
    if ~isempty(idx)
      Rk(idx) = Ri;
    end
  end

  % --- load-current jitter (piecewise constant, repeatable) -------------
  randn("state", P.seed);
  hold_n  = round(P.noise_hold / dt);
  jit_raw = P.i_noise_rms * randn(1, ceil(N / hold_n) + 1);
  jit     = jit_raw(floor((0:N-1) / hold_n) + 1);

  % --- initial state = steady state at the initial load ------------------
  vC = P.Vref;
  iL = P.Vref / Rk(1);
  I  = (P.Vref + P.RL * iL) / P.Vin;   % duty that holds Vref at steady state
  ef = 0;                              % filtered error (for derivative)

  % --- storage (every P.dec-th step) ------------------------------------
  M = ceil(N / P.dec);
  o_t = zeros(M,1); o_v = o_t; o_i = o_t; o_d = o_t; o_r = o_t;
  m = 0;

  for k = 1:N
    % ---- controller -----------------------------------------------------
    e     = P.Vref - vC;
    dterm = Kd(k) * (e - ef) / P.Tf;        % filtered derivative  s/(Tf*s+1)
    ef    = ef + dt * (e - ef) / P.Tf;
    I     = I + dt * Ki(k) * e;             % integrator
    I     = min(max(I, 0), P.Dmax);         % anti-windup clamp
    d     = Kp(k) * e + I + dterm;
    d     = min(max(d, 0), P.Dmax);         % duty limits: 0 ... Dmax  (SATURATION)

    % ---- power stage (averaged) -----------------------------------------
    % Semi-implicit Euler: update iL first, then use the NEW iL to update vC.
    % (Plain Euler slowly adds fake energy to an LC circuit; this ordering
    %  does not.)
    iL = iL + dt * (d * P.Vin - vC - P.RL * iL) / P.L;
    iL = min(max(iL, P.Ineg), P.Ipos);      % current limit: real chips have one,
                                            % so an oscillation cannot grow forever
    vC = vC + dt * (iL - vC / Rk(k) - jit(k)) / P.C;

    % ---- store ----------------------------------------------------------
    if mod(k-1, P.dec) == 0
      m = m + 1;
      o_t(m) = t(k);  o_v(m) = vC;  o_i(m) = iL;  o_d(m) = d;  o_r(m) = Rk(k);
    end
  end

  out.t = o_t(1:m);  out.vout = o_v(1:m);  out.iL = o_i(1:m);
  out.duty = o_d(1:m);  out.R = o_r(1:m);
end

%% ---------------------------------------------------------------------
%  SECTION 3 : LINEAR SANITY CHECK  (closed-loop poles)
%  Small-signal plant:  Vout/d = Vin / (L*C*s^2 + (L/R + RL*C)*s + 1 + RL/R)
%  If any closed-loop pole has a POSITIVE real part the loop is unstable.
%  Returns the least-damped complex pole pair (frequency and damping ratio).
%% ---------------------------------------------------------------------
function [f_hz, zeta, max_re] = least_damped_pole(P, R, g)
  a2 = P.L * P.C;  a1 = P.L / R + P.RL * P.C;  a0 = 1 + P.RL / R;
  Kp = g(1); Ki = g(2); Kd = g(3);
  nc = conv([Kp Ki], [P.Tf 1]) + [Kd 0 0];   % controller numerator
  dc = conv([1 0],    [P.Tf 1]);             % controller denominator
  ch = conv(dc, [a2 a1 a0]);                 % characteristic polynomial ...
  q  = P.Vin * nc;
  q  = [zeros(1, numel(ch) - numel(q)) q];
  p  = roots(ch + q);                        % ... its roots are the poles
  max_re = max(real(p));
  c = p(abs(imag(p)) > 1);                   % complex poles only
  if isempty(c)
    f_hz = NaN; zeta = NaN;
  else
    [~, i] = min(-real(c) ./ abs(c));        % smallest damping ratio
    f_hz = abs(c(i)) / (2*pi);
    zeta = -real(c(i)) / abs(c(i));
  end
end

%% =====================================================================
%  MAIN PROGRAM
%% =====================================================================
P = default_params();
if ~exist("data", "dir"), mkdir("data"); end
SHOW_PLOTS = true;      % set false to only save PNGs (no windows)

%% ---- Design numbers (printed so you can quote them in the viva) --------
D0    = P.Vref / P.Vin;                          % ideal duty cycle
f0    = 1 / (2*pi*sqrt(P.L * P.C));              % LC resonant frequency
Zc    = sqrt(P.L / P.C);                         % characteristic impedance
dIL   = (P.Vin - P.Vref) * D0 / (P.L * P.fsw);   % inductor ripple current (p-p)
dVsw  = dIL / (8 * P.C * P.fsw);                 % capacitor ripple voltage (p-p, ideal C)
printf("\n=== DESIGN NUMBERS (simulated regulator) ===\n");
printf("Vin = %g V, Vref = %g V, ideal duty D = Vref/Vin = %.3f\n", P.Vin, P.Vref, D0);
printf("fsw = %g kHz, L = %g uH, C = %g uF, nominal load = %g ohm (%.2f A)\n", ...
       P.fsw/1e3, P.L*1e6, P.C*1e6, P.R0, P.Vref/P.R0);
printf("LC resonant frequency f0 = 1/(2*pi*sqrt(LC)) = %.0f Hz\n", f0);
printf("Switching ripple that the AVERAGED model leaves out:\n");
printf("   inductor ripple current  = %.3f A p-p\n", dIL);
printf("   output ripple voltage    = %.2f mV p-p  (ideal capacitor)\n", dVsw*1e3);
printf("   fsw / f0 = %.0f  -> switching is far faster than the loop dynamics\n\n", P.fsw/f0);

%% ---- The five scenarios --------------------------------------------------
%  Gain sets [Kp Ki Kd]:
G_healthy = [0.05 200 2e-5];   % well damped (Kd provides damping)
G_ringing = [0.05  80 0   ];   % Kd removed -> LC resonance barely damped, still stable
G_marg    = [0.05 200 0   ];   % Kd removed, Ki as before -> loop just barely unstable
G_bad     = [0.05 300 0   ];   % Kd removed and Ki high -> clearly unstable

step_t = [0.050 0.120];        % load steps: 10 ohm -> 5 ohm at 50 ms, back to 10 ohm at 120 ms
step_R = [5     10   ];

S = struct("name", {}, "g0", {}, "g1", {}, "t_deg", {}, "load_t", {}, "load_R", {}, "title", {});

S(1).name = "stable";      S(1).g0 = G_healthy; S(1).g1 = G_healthy; S(1).t_deg = [0 0];
S(1).load_t = [];          S(1).load_R = [];
S(1).title = "1) Stable regulation (healthy loop, constant load)";

S(2).name = "transient";   S(2).g0 = G_healthy; S(2).g1 = G_healthy; S(2).t_deg = [0 0];
S(2).load_t = step_t;      S(2).load_R = step_R;
S(2).title = "2) Load transient (healthy loop, load steps at 50 ms and 120 ms)";

S(3).name = "ringing";     S(3).g0 = G_ringing; S(3).g1 = G_ringing; S(3).t_deg = [0 0];
S(3).load_t = step_t;      S(3).load_R = step_R;
S(3).title = "3) Damped ringing (poorly damped but stable loop, same load steps)";

S(4).name = "growing";     S(4).g0 = G_healthy; S(4).g1 = G_marg; S(4).t_deg = [0.030 0.130];
S(4).load_t = [];          S(4).load_R = [];
S(4).title = "4) Growing oscillation (damping slowly lost between 30 ms and 130 ms)";

S(5).name = "oscillation"; S(5).g0 = G_healthy; S(5).g1 = G_bad; S(5).t_deg = [0.050 0.050];
S(5).load_t = [];          S(5).load_R = [];
S(5).title = "5) Sustained oscillation (loop becomes unstable abruptly at 50 ms)";

%% ---- Run, check poles, save CSV, plot ------------------------------------
res = cell(1, numel(S));
for s = 1:numel(S)
  printf("Simulating %-12s ... ", S(s).name); fflush(stdout);
  tic; res{s} = simulate_buck(P, S(s)); printf("done (%.1f s)\n", toc);

  o = res{s};
  M = [o.t, o.vout, o.iL, o.duty, o.R];
  fid = fopen(fullfile("data", [S(s).name ".csv"]), "w");
  fprintf(fid, "time_s,Vout_V,iL_A,duty,Rload_ohm\n");
  fprintf(fid, "%.6f,%.6f,%.6f,%.5f,%.2f\n", M.');
  fclose(fid);

  [f_a, z_a, mr_a] = least_damped_pole(P, P.R0, S(s).g0);
  [f_b, z_b, mr_b] = least_damped_pole(P, P.R0, S(s).g1);
  printf("   linear check (R = %g ohm): start gains -> f=%.0f Hz, zeta=%.3f, max Re(pole)=%+.0f 1/s\n", ...
         P.R0, f_a, z_a, mr_a);
  if any(S(s).g1 ~= S(s).g0)
    printf("                             end gains   -> f=%.0f Hz, zeta=%.3f, max Re(pole)=%+.0f 1/s\n", ...
           f_b, z_b, mr_b);
  end
end
printf("\nCSV files written to ./data/  (SIMULATED data)\n");

%% ---- Plots ------------------------------------------------------------------
if ~SHOW_PLOTS, set(0, "defaultfigurevisible", "off"); end
fig = figure("position", [100 50 1000 1200]);
for s = 1:numel(S)
  o = res{s};
  subplot(numel(S), 1, s);
  plot(o.t*1e3, o.vout, "b", "linewidth", 1); hold on;
  plot([0 P.Tend*1e3], [P.Vref P.Vref], "k--");
  grid on; xlabel("time (ms)"); ylabel("V_{out} (V)");
  title([S(s).title "  [SIMULATED]"]);
  xlim([0 P.Tend*1e3]);
  % zoomed y-axis for the small-signal cases, full range for the saturated one
  span = max(o.vout) - min(o.vout);
  if span < 0.5, ylim([P.Vref - 0.5, P.Vref + 0.5]); end
end
print(fig, "-dpng", "-r110", fullfile("data", "vout_all_conditions.png"));

for s = 1:numel(S)
  o = res{s};
  f2 = figure("position", [100 100 900 350]);
  plot(o.t*1e3, o.vout, "b", "linewidth", 1); hold on;
  plot([0 P.Tend*1e3], [P.Vref P.Vref], "k--");
  grid on; xlabel("time (ms)"); ylabel("V_{out} (V)");
  title([S(s).title "  [SIMULATED]"]);
  print(f2, "-dpng", "-r100", fullfile("data", ["vout_" S(s).name ".png"]));
  if ~SHOW_PLOTS, close(f2); end
end
printf("Plots saved to ./data/*.png\n");
