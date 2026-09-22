#!/usr/bin/env python3
"""
detector.py  -  Predictive Oscillation Detection for Switching Power Regulators
================================================================================
Embedded-monitoring layer (Python prototype of what an ESP32 would run).

Pipeline (one block per section below):

  CSV Vout  ->  ADC emulation  ->  circular buffer  ->  DC removal
            ->  features (p-p, RMS, FFT peak, tone energy, trend)
            ->  risk score (0..1)  ->  state machine  NORMAL / WARNING / OSCILLATION

ALL INPUT DATA IS SIMULATED (produced by buck_sim.m).  Nothing here is measured
hardware data, and no claim is made about the exact time of a hardware failure.
"Predictive" here only means: raise WARNING on early indicators (growing ripple,
narrow-band ringing that persists) BEFORE the ripple becomes severe.

Usage
-----
  python detector.py                      # run all five scenarios in ./data
  python detector.py --file data/growing.csv
  python detector.py --noise-lsb 1.5      # noisier ADC
  python detector.py --no-show            # save PNGs only (no windows)

Requires: numpy, scipy (only for a Hann window), matplotlib
"""

import argparse
import os
from dataclasses import dataclass, field

import numpy as np
from scipy.signal.windows import hann
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

NORMAL, WARNING, OSCILLATION = "NORMAL", "WARNING", "OSCILLATION"
STATE_COLOR = {NORMAL: "#2ca02c", WARNING: "#ff9f0e", OSCILLATION: "#d62728"}


# =============================================================================
# 1. CONFIGURATION  (every threshold lives here -> nothing is hidden in code)
# =============================================================================
@dataclass
class ADCConfig:
    fs: float = 20_000.0       # ADC sampling rate [Hz]  (~15x the ~1.4 kHz resonance)
    bits: int = 12             # ESP32 ADC resolution
    v_ref: float = 3.3         # ADC full-scale [V]
    divider: float = 0.5       # resistor divider ratio: V_adc = 0.5 * Vout
    noise_lsb: float = 0.5     # RMS analogue noise at ADC input, in LSBs
    antialias: bool = True     # average the fast source samples (simple low-pass)
    seed: int = 0              # fixed noise seed -> repeatable results


@dataclass
class DetectorConfig:
    window: int = 128          # samples per analysis window  (6.4 ms at 20 kHz)
    hop: int = 64              # new window every 64 samples  (3.2 ms, 50 % overlap)
    band: tuple = (300.0, 5000.0)   # oscillation band [Hz] (LC resonance ~1.1-1.4 kHz)
    tone_halfwidth: int = 2    # bins each side of the peak counted as "the tone"
    trend_len: int = 5         # windows used for the growth (trend) estimate
    # --- risk-score weights (sum = 1) ---------------------------------------
    w_amp: float = 0.4         # how large is the ripple?
    w_tone: float = 0.4        # how much of it is ONE narrow-band oscillation?
    w_growth: float = 0.2      # is it getting bigger from window to window?
    growth_lo: float = 0.05    # ln-growth per window that starts to count
    growth_hi: float = 0.30    # ln-growth per window that counts fully
    # --- state machine (persistence, in windows) ----------------------------
    r_warn: float = 0.20       # risk needed to count toward WARNING
    r_osc: float = 0.55        # risk needed to count toward OSCILLATION
    n_warn: int = 4            # consecutive windows above r_warn  -> WARNING   (~13 ms)
    n_osc: int = 8             # consecutive windows above r_osc   -> OSCILLATION (~26 ms)
    n_clear: int = 6           # consecutive calm windows           -> step back down
    # --- calibration settings -----------------------------------------------
    k_sigma: float = 5.0       # noise-floor gate = mean + k_sigma * std (stable data)
    severe_pct: float = 3.0    # "severe" RMS ripple = 3 % of Vout  (design choice)


@dataclass
class Calibration:
    """Numbers that turn raw features into normalised 0..1 scores."""
    rms_lo: float = 0.005      # [V]  below this = noise floor
    rms_hi: float = 0.150      # [V]  at/above this = severe
    tone_lo: float = 0.002     # [V]
    tone_hi: float = 0.100     # [V]
    v_nominal: float = 5.0     # [V]


# =============================================================================
# 2. INPUT + ADC EMULATION      (ESP32:  analogue divider + ADC + hardware timer)
# =============================================================================
def load_csv(path):
    """Read the Octave CSV.  Needs columns time_s and Vout_V."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data["time_s"], data["Vout_V"]


def emulate_adc(t, vout, adc: ADCConfig):
    """
    Turn the 'analogue' Vout(t) from Octave into what firmware would receive:
    integer ADC codes at a fixed rate fs.   Steps:
      1. anti-alias: average the fast source samples inside one ADC period
      2. sample at t_k = k / fs   (a hardware timer does this on the ESP32)
      3. divider:  V_adc = divider * Vout
      4. add analogue noise (RMS = noise_lsb LSBs)
      5. clip to 0 ... v_ref  (ADC input range)
      6. quantise:  code = round(V_adc / v_ref * (2^bits - 1))
    Returns (t_adc, codes)   codes are integers, exactly as an MCU would see.
    """
    fs_src = 1.0 / np.median(np.diff(t))
    if adc.antialias:
        n = max(int(round(fs_src / adc.fs)), 1)
        # centred moving average; pad by repeating the edge value so the first/last
        # samples are not distorted by zero-padding
        pad_l, pad_r = n // 2, n - 1 - n // 2
        vp = np.pad(vout, (pad_l, pad_r), mode="edge")
        vout = np.convolve(vp, np.ones(n) / n, mode="valid")
    t_adc = np.arange(t[0], t[-1], 1.0 / adc.fs)
    v = np.interp(t_adc, t, vout)                       # value at the sample instants
    v_adc = adc.divider * v
    lsb = adc.v_ref / (2 ** adc.bits - 1)
    rng = np.random.default_rng(adc.seed)
    v_adc = v_adc + rng.normal(0.0, adc.noise_lsb * lsb, size=v_adc.shape)
    v_adc = np.clip(v_adc, 0.0, adc.v_ref)
    codes = np.round(v_adc / lsb).astype(np.int32)
    return t_adc, codes


def code_to_vout(codes, adc: ADCConfig):
    """Convert ADC codes back to volts at the regulator output (for readability)."""
    lsb = adc.v_ref / (2 ** adc.bits - 1)
    return codes * lsb / adc.divider


# =============================================================================
# 3. CIRCULAR BUFFER            (ESP32:  static float buf[N]; uint16_t head;)
# =============================================================================
class CircularBuffer:
    """Fixed-size ring: newest sample overwrites the oldest one. No allocation."""

    def __init__(self, size):
        self.buf = np.zeros(size)
        self.size = size
        self.head = 0          # index where the NEXT sample will be written
        self.count = 0         # total samples pushed (saturates at 'size' for fullness)

    def push(self, x):
        self.buf[self.head] = x
        self.head = (self.head + 1) % self.size
        self.count += 1

    @property
    def full(self):
        return self.count >= self.size

    def window(self):
        """Samples in time order, oldest first."""
        return np.concatenate((self.buf[self.head:], self.buf[:self.head]))


# =============================================================================
# 4. FEATURE EXTRACTION         (ESP32:  one C function called once per hop)
# =============================================================================
def extract_features(x, fs, cfg: DetectorConfig, w_hann):
    """
    x : one window of samples in volts at Vout (length cfg.window).
    Returns a dict of features.

    DC removal      x_ac = x - mean(x)                     (removes the 5 V level)
    p-p ripple      max(x_ac) - min(x_ac)
    RMS ripple      sqrt(mean(x_ac^2))
    FFT             Hann window (reduces spectral leakage), then rfft
    band RMS        RMS of the signal inside cfg.band (Parseval's theorem)
    dominant freq   bin of the largest peak inside the band (parabolic refinement)
    tone RMS        RMS of only the peak bin +/- tone_halfwidth  = 'oscillation energy'
    tonality        tone power / band power  (near 1 = one pure tone, small = noise)
    """
    n = len(x)
    x_ac = x - np.mean(x)
    p2p = float(np.max(x_ac) - np.min(x_ac))
    rms = float(np.sqrt(np.mean(x_ac ** 2)))

    X = np.fft.rfft(x_ac * w_hann)
    P = np.abs(X) ** 2
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    band = (freqs >= cfg.band[0]) & (freqs <= cfg.band[1])
    idx_band = np.where(band)[0]

    # Parseval: RMS of the un-windowed signal represented by a set of FFT bins
    #   rms = sqrt( 2 * sum(|X_k|^2) / (N * sum(w^2)) )     (one-sided spectrum)
    scale = 2.0 / (n * np.sum(w_hann ** 2))

    band_power = scale * np.sum(P[idx_band])
    k_pk = idx_band[np.argmax(P[idx_band])]
    lo, hi = max(k_pk - cfg.tone_halfwidth, 0), min(k_pk + cfg.tone_halfwidth + 1, len(P))
    tone_power = scale * np.sum(P[lo:hi])

    # parabolic interpolation of the peak position (sub-bin frequency estimate)
    if 0 < k_pk < len(P) - 1:
        a, b, c = np.log(P[k_pk - 1] + 1e-30), np.log(P[k_pk] + 1e-30), np.log(P[k_pk + 1] + 1e-30)
        delta = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
    else:
        delta = 0.0
    f_dom = float((k_pk + delta) * fs / n)

    return dict(
        p2p=p2p, rms=rms,
        band_rms=float(np.sqrt(band_power)),
        tone_rms=float(np.sqrt(tone_power)),
        tonality=float(tone_power / band_power) if band_power > 0 else 0.0,
        f_dom=f_dom,
        spectrum=np.sqrt(scale * P),          # amplitude-like spectrum (V rms per bin) for plots
        freqs=freqs,
    )


# =============================================================================
# 5. CALIBRATION FROM THE STABLE SIMULATION
# =============================================================================
def calibrate_from_stable(feature_list, cfg: DetectorConfig, v_nominal=5.0):
    """
    Choose the 'low' end of each normalisation from a run known to be healthy:

        floor = mean(feature over stable windows) + k_sigma * std(...)

    Anything below 'floor' is indistinguishable from ADC + load noise, so it
    scores zero.  The 'high' end (score = 1) is a DESIGN CHOICE, not something
    physics tells us: 'severe' = cfg.severe_pct % of Vout as RMS ripple.
    Change cfg.severe_pct to match your own specification.
    """
    rms = np.array([f["rms"] for f in feature_list])
    tone = np.array([f["tone_rms"] for f in feature_list])
    cal = Calibration(v_nominal=v_nominal)
    cal.rms_lo = float(rms.mean() + cfg.k_sigma * rms.std())
    cal.tone_lo = float(tone.mean() + cfg.k_sigma * tone.std())
    cal.rms_hi = cfg.severe_pct / 100.0 * v_nominal
    cal.tone_hi = cal.rms_hi
    return cal


# =============================================================================
# 6. RISK SCORE                 (ESP32:  a few multiplies, one log10, one logf)
# =============================================================================
def db_score(value, lo, hi):
    """
    Map 'value' to 0..1 on a DECIBEL scale between lo (-> 0) and hi (-> 1).
    A dB scale is used because oscillation amplitude can change by 100x:
        score = 20*log10(value/lo) / 20*log10(hi/lo)   clipped to [0, 1]
    """
    if value <= lo:
        return 0.0
    return float(np.clip(np.log(value / lo) / np.log(hi / lo), 0.0, 1.0))


def growth_slope(log_rms_hist, n):
    """
    Least-squares slope of ln(RMS) over the last n windows (per window).
    For n = 5 this reduces to  (-2*y0 - y1 + y3 + 2*y4) / 10   -> trivial on an MCU.
    Positive = ripple growing;  negative = decaying.
    """
    if len(log_rms_hist) < n:
        return 0.0
    y = np.array(log_rms_hist[-n:])
    i = np.arange(n) - (n - 1) / 2.0
    return float(np.sum(i * y) / np.sum(i * i))


def risk_score(feat, slope, cal: Calibration, cfg: DetectorConfig):
    """
    risk = w_amp*A + w_tone*E + w_growth*G      (each term in 0..1)

      A : ripple amplitude   (RMS above the noise floor, dB scale)
      E : oscillation energy (RMS of the single dominant spectral tone, dB scale)
      G : growth             (rising ln-RMS trend; only counts if A > 0, i.e. above noise)
    """
    A = db_score(feat["rms"], cal.rms_lo, cal.rms_hi)
    E = db_score(feat["tone_rms"], cal.tone_lo, cal.tone_hi)
    G = 0.0
    if A > 0.0:
        G = float(np.clip((slope - cfg.growth_lo) / (cfg.growth_hi - cfg.growth_lo), 0.0, 1.0))
    R = cfg.w_amp * A + cfg.w_tone * E + cfg.w_growth * G
    return R, A, E, G


# =============================================================================
# 7. STATE MACHINE with persistence + hysteresis   (ESP32:  enum + switch)
# =============================================================================
class StateMachine:
    """
    NORMAL      -> WARNING      after n_warn consecutive windows with risk >= r_warn
    WARNING     -> OSCILLATION  after n_osc  consecutive windows with risk >= r_osc
    OSCILLATION -> WARNING      after n_clear consecutive windows with risk <  r_osc
    WARNING     -> NORMAL       after n_clear consecutive windows with risk <  r_warn

    Persistence means a single load transient (which excites a few windows and
    then dies away) does not raise an alarm.  Hysteresis (different up/down
    conditions) stops the state flickering.
    """

    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg
        self.state = NORMAL
        self.c_warn = self.c_osc = 0
        self.c_calm_w = self.c_calm_o = 0

    def update(self, R):
        c = self.cfg
        self.c_warn = self.c_warn + 1 if R >= c.r_warn else 0
        self.c_osc = self.c_osc + 1 if R >= c.r_osc else 0
        self.c_calm_w = self.c_calm_w + 1 if R < c.r_warn else 0
        self.c_calm_o = self.c_calm_o + 1 if R < c.r_osc else 0

        if self.state == NORMAL:
            if self.c_warn >= c.n_warn:
                self.state = WARNING
        elif self.state == WARNING:
            if self.c_osc >= c.n_osc:
                self.state = OSCILLATION
            elif self.c_calm_w >= c.n_clear:
                self.state = NORMAL
        elif self.state == OSCILLATION:
            if self.c_calm_o >= c.n_clear:
                self.state = WARNING
                self.c_calm_w = 0 if R >= c.r_warn else self.c_calm_w
        return self.state


# =============================================================================
# 8. THE STREAMING DETECTOR     (this object is what becomes ESP32 firmware)
# =============================================================================
class Detector:
    def __init__(self, adc: ADCConfig, cfg: DetectorConfig, cal: Calibration):
        self.adc, self.cfg, self.cal = adc, cfg, cal
        self.buf = CircularBuffer(cfg.window)
        self.w = hann(cfg.window, sym=False)
        self.sm = StateMachine(cfg)
        self.log_rms = []
        self.since_hop = 0

    def on_sample(self, code, t_now):
        """Called once per ADC sample (= the timer interrupt / ADC-ready callback)."""
        self.buf.push(float(code_to_vout(code, self.adc)))
        self.since_hop += 1
        if self.buf.full and self.since_hop >= self.cfg.hop:
            self.since_hop = 0
            return self.process_window(t_now)
        return None

    def process_window(self, t_now):
        """Called once per hop (= the processing task)."""
        feat = extract_features(self.buf.window(), self.adc.fs, self.cfg, self.w)
        self.log_rms.append(np.log(max(feat["rms"], 1e-6)))
        slope = growth_slope(self.log_rms, self.cfg.trend_len)
        R, A, E, G = risk_score(feat, slope, self.cal, self.cfg)
        state = self.sm.update(R)
        return dict(t=t_now, R=R, A=A, E=E, G=G, slope=slope, state=state, **feat)


def run_stream(codes, t_adc, adc, cfg, cal):
    det = Detector(adc, cfg, cal)
    out = []
    for code, t in zip(codes, t_adc):        # <- this loop is the timer ISR on hardware
        r = det.on_sample(code, t)
        if r is not None:
            out.append(r)
    return out


def features_only(codes, adc, cfg):
    """Used for calibration: windows -> features (no risk / state)."""
    buf, w, out, since = CircularBuffer(cfg.window), hann(cfg.window, sym=False), [], 0
    for c in codes:
        buf.push(float(code_to_vout(c, adc)))
        since += 1
        if buf.full and since >= cfg.hop:
            since = 0
            out.append(extract_features(buf.window(), adc.fs, cfg, w))
    return out


# =============================================================================
# 9. REFERENCE: when did the ripple actually become severe? (for lead time)
# =============================================================================
def time_of_severe_ripple(t_adc, vout_v, adc, cfg, cal):
    """First window whose true RMS ripple exceeds cal.rms_hi (the 'severe' level)."""
    n, h = cfg.window, cfg.hop
    for i in range(0, len(vout_v) - n, h):
        seg = vout_v[i:i + n]
        if np.std(seg) >= cal.rms_hi:
            return t_adc[i + n - 1]
    return None


# =============================================================================
# 10. PLOTS + SUMMARY
# =============================================================================
def first_time(res, state):
    for r in res:
        if r["state"] == state:
            return r["t"]
    return None


def plot_result(name, t_adc, vout_v, res, cfg, cal, out_png, show):
    t_w = np.array([r["t"] for r in res])
    R = np.array([r["R"] for r in res])
    states = [r["state"] for r in res]
    hop_s = cfg.hop / (1.0 / np.median(np.diff(t_adc)))

    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True,
                           gridspec_kw={"height_ratios": [2, 2, 1.6, 1.6]})

    # (a) waveform + state colour bands
    ax[0].plot(t_adc * 1e3, vout_v, lw=0.7, color="navy")
    for tw, s in zip(t_w, states):
        ax[0].axvspan((tw - hop_s) * 1e3, tw * 1e3, color=STATE_COLOR[s], alpha=0.18, lw=0)
    ax[0].set_ylabel("Vout as seen by ADC (V)")
    ax[0].set_title(f"{name}: ADC-sampled waveform (SIMULATED data), background = detector state")
    ax[0].legend(handles=[Patch(color=STATE_COLOR[s], alpha=0.4, label=s) for s in STATE_COLOR],
                 loc="upper left", ncol=3, fontsize=8)
    ax[0].grid(alpha=0.3)

    # (b) spectrogram built from the per-window FFTs
    freqs = res[0]["freqs"]
    S = np.array([r["spectrum"] for r in res]).T * 1e3           # mV rms per bin
    fmax = 6000
    m = freqs <= fmax
    im = ax[1].pcolormesh(t_w * 1e3, freqs[m], np.log10(S[m] + 1e-3), shading="nearest", cmap="viridis")
    ax[1].plot(t_w * 1e3, [r["f_dom"] for r in res], "w.", ms=2.5, alpha=0.7)
    ax[1].axhspan(cfg.band[0], cfg.band[1], facecolor="none", edgecolor="w", ls=":", lw=0.8)
    ax[1].set_ylabel("frequency (Hz)")
    ax[1].set_title("FFT of every window (colour = log10 of amplitude in mV; white dots = dominant frequency)")
    fig.colorbar(im, ax=ax[1], pad=0.01, label="log10(mV)")

    # (c) features
    ax[2].semilogy(t_w * 1e3, [r["rms"] * 1e3 for r in res], label="RMS ripple (mV)")
    ax[2].semilogy(t_w * 1e3, [r["tone_rms"] * 1e3 for r in res], label="dominant-tone RMS (mV)")
    ax[2].semilogy(t_w * 1e3, [r["p2p"] * 1e3 for r in res], label="peak-to-peak (mV)", alpha=0.6)
    ax[2].axhline(cal.rms_lo * 1e3, color="gray", ls=":", label="noise-floor gate (calibrated)")
    ax[2].set_ylabel("feature (mV)")
    ax[2].legend(fontsize=8, ncol=4, loc="upper left")
    ax[2].grid(alpha=0.3, which="both")

    # (d) risk score
    ax[3].plot(t_w * 1e3, R, color="k", lw=1.3, label="risk score")
    ax[3].axhline(cfg.r_warn, color=STATE_COLOR[WARNING], ls="--", label=f"r_warn = {cfg.r_warn}")
    ax[3].axhline(cfg.r_osc, color=STATE_COLOR[OSCILLATION], ls="--", label=f"r_osc = {cfg.r_osc}")
    ax[3].set_ylim(0, 1.05)
    ax[3].set_ylabel("risk (0..1)")
    ax[3].set_xlabel("time (ms)")
    ax[3].legend(fontsize=8, ncol=3, loc="upper left")
    ax[3].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    if show:
        plt.show()
    plt.close(fig)


def plot_spectrum_snapshot(name, res, cfg, out_png, show):
    """Spectrum of the highest-risk window, with the band and dominant peak marked."""
    r = res[int(np.argmax([x["R"] for x in res]))]
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.semilogy(r["freqs"], r["spectrum"] * 1e3 + 1e-6)
    ax.axvspan(cfg.band[0], cfg.band[1], color="orange", alpha=0.12, label="oscillation band")
    ax.axvline(r["f_dom"], color="r", ls="--", label=f"dominant = {r['f_dom']:.0f} Hz")
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("amplitude (mV rms/bin)")
    ax.set_title(f"{name}: spectrum of the highest-risk window (t = {r['t']*1e3:.1f} ms, SIMULATED)")
    ax.legend(); ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    if show:
        plt.show()
    plt.close(fig)


def summarise(name, res, cfg, t_severe):
    n = len(res)
    frac = {s: sum(r["state"] == s for r in res) / n * 100 for s in (NORMAL, WARNING, OSCILLATION)}
    t_w, t_o = first_time(res, WARNING), first_time(res, OSCILLATION)
    fdom_osc = [r["f_dom"] for r in res if r["state"] != NORMAL]
    fmt = lambda v: f"{v*1e3:7.1f} ms" if v is not None else "      --   "
    lead = (f"{(t_severe - t_w)*1e3:6.1f} ms" if (t_w is not None and t_severe is not None) else "    --   ")
    print(f"{name:<12} {frac[NORMAL]:6.1f}% {frac[WARNING]:6.1f}% {frac[OSCILLATION]:6.1f}%   "
          f"{fmt(t_w)}  {fmt(t_o)}  {fmt(t_severe)}  {lead}   "
          f"{(np.median(fdom_osc) if fdom_osc else float('nan')):7.0f}   {res[-1]['state']}")


# =============================================================================
# 11. MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--file", help="analyse just this CSV")
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--noise-lsb", type=float, default=None, help="ADC input noise (RMS, in LSBs)")
    ap.add_argument("--bits", type=int, default=None)
    ap.add_argument("--fs", type=float, default=None, help="ADC sampling rate (Hz)")
    ap.add_argument("--no-show", action="store_true", help="save figures but do not open windows")
    a = ap.parse_args()

    if a.no_show:
        matplotlib.use("Agg")
    adc = ADCConfig()
    if a.noise_lsb is not None: adc.noise_lsb = a.noise_lsb
    if a.bits is not None: adc.bits = a.bits
    if a.fs is not None: adc.fs = a.fs
    cfg = DetectorConfig()
    os.makedirs(a.out_dir, exist_ok=True)

    # ---- calibrate on the stable simulation --------------------------------
    stable_csv = os.path.join(a.data_dir, "stable.csv")
    t, v = load_csv(stable_csv)
    t_adc, codes = emulate_adc(t, v, adc)
    feats = features_only(codes[int(0.01 * adc.fs):], adc, cfg)      # skip first 10 ms
    cal = calibrate_from_stable(feats, cfg, v_nominal=float(np.mean(v)))
    print("=" * 100)
    print("CALIBRATION from stable.csv (SIMULATED data)")
    print(f"  ADC: {adc.bits}-bit, fs = {adc.fs:.0f} Hz, divider = {adc.divider}, "
          f"LSB at Vout = {adc.v_ref/(2**adc.bits-1)/adc.divider*1e3:.2f} mV, noise = {adc.noise_lsb} LSB rms")
    print(f"  stable windows analysed  : {len(feats)}")
    print(f"  RMS ripple  floor gate   : {cal.rms_lo*1e3:6.2f} mV   (mean + {cfg.k_sigma:g} sigma of stable windows)")
    print(f"  tone RMS    floor gate   : {cal.tone_lo*1e3:6.2f} mV")
    print(f"  'severe' RMS (score = 1) : {cal.rms_hi*1e3:6.1f} mV   ({cfg.severe_pct:g} % of Vout - a design choice)")
    print("=" * 100)

    if a.file:
        files = [a.file]
    else:
        files = [os.path.join(a.data_dir, f"{n}.csv") for n in
                 ("stable", "transient", "ringing", "growing", "oscillation")]

    print(f"\n{'scenario':<12} {'NORMAL':>7} {'WARN':>7} {'OSC':>7}   {'1st WARN':>10}  {'1st OSC':>10}  "
          f"{'severe@':>10}  {'lead':>9}   {'f_alarm':>7}   final")
    print("-" * 118)
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        t, v = load_csv(path)
        t_adc, codes = emulate_adc(t, v, adc)
        res = run_stream(codes, t_adc, adc, cfg, cal)
        v_seen = code_to_vout(codes, adc)
        t_sev = time_of_severe_ripple(t_adc, v_seen, adc, cfg, cal)
        summarise(name, res, cfg, t_sev)
        plot_result(name, t_adc, v_seen, res, cfg, cal, os.path.join(a.out_dir, f"{name}_detector.png"),
                    show=not a.no_show)
        plot_spectrum_snapshot(name, res, cfg, os.path.join(a.out_dir, f"{name}_spectrum.png"),
                               show=not a.no_show)
    print("-" * 118)
    print("All results above come from SIMULATED waveforms.")
    print("severe@ = first time the true RMS ripple reached the 'severe' level; lead = severe@ - first WARN")
    print("(negative lead = the warning came AFTER the ripple was already severe, i.e. no early warning).")
    print("f_alarm = median dominant frequency over the windows that were not NORMAL.")
    print(f"Figures saved in ./{a.out_dir}/")


if __name__ == "__main__":
    main()
