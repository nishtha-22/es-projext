# Predictive Oscillation Detector for Switching Power Regulators

An embedded-style monitoring pipeline for a **12 V → 5 V voltage-mode buck converter**.
Simulated regulator waveforms are passed through an emulated ESP32 acquisition chain,
processed window-by-window, reduced to features, scored for risk, and classified as:

**`NORMAL` → `WARNING` → `OSCILLATION`**

> **All data in this repository is simulated** (`buck_sim.m` → CSV → `detector.py`).
> Nothing is measured hardware data. *Predictive* here means **early-warning detection
> from growing trend features**, not predicting an exact time-to-failure.

```text
REGULATOR  →  SENSE  →  ADC  →  ESP32  →  SIGNAL PROCESSING  →  FEATURES  →  RISK ASSESSMENT  →  NORMAL / WARNING / OSCILLATION
```

In one sentence:

> The regulator produces a voltage → we observe it → convert it into numbers →
> analyze those numbers → extract evidence of abnormal behavior → make a decision.

---

## Repository layout

| Path | Role |
|------|------|
| `buck_sim.m` | GNU Octave simulator: averaged closed-loop buck + voltage-mode PID, five scenarios |
| `detector.py` | Python prototype of the firmware: ADC emulation → circular buffer → features → risk → state machine |
| `data/*.csv` | Five simulated waveforms (`time_s,Vout_V,iL_A,duty,Rload_ohm`), 200 ms @ 100 kS/s |
| `data/*.png` | Simulator plots of `Vout(t)` per scenario |
| `results/` | Created at runtime: detector figures per scenario |

---

## 1. Regulator

The plant is a buck converter regulated to **5 V from 12 V** with a voltage-mode PID.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `Vin` | 12 V | supply |
| `Vref` | 5 V | setpoint → ideal duty `D ≈ Vref/Vin = 0.417` |
| `fsw` | 100 kHz | switching frequency (ripple estimate only — see caveat below) |
| `L` | 100 µH | power inductor (`RL = 0.05 Ω` DCR) |
| `C` | 220 µF | output capacitor (ESR neglected) |
| LC resonance `f0` | ≈ 1.07 kHz | `1/(2π√(LC))` — the band we actually watch |
| Nominal load | 10 Ω (0.5 A) | load-current jitter 15 mA RMS as noise seed |
| Integration | 500 kHz, 200 ms | stored every 5th step → **100 kS/s** in CSV |

The model is **averaged** (control-oriented): the switch is replaced by its duty
`d`, so the 100 kHz switching ripple is *not* in the waveform. What remains is the
loop dynamics — ripple, ringing, and oscillation around 5 V.

Controller: `d = Kp·e + ∫Ki·e + Kd·d/dt(e)` with filtered derivative, anti-windup,
duty limits `[0, 0.95]`, and inductor current limits `[-0.5, 2.0] A`.

A linear sanity check (`least_damped_pole`) reports the least-damped closed-loop
pole (frequency and damping ratio ζ) for each gain set — the code's stability
metric in place of a measured phase margin.

### Five scenarios

| Scenario | What changes | Intent |
|----------|--------------|--------|
| `stable` | healthy gains `[0.05, 200, 2e-5]`, constant load | baseline / calibration source |
| `transient` | load steps 10→5 Ω @ 50 ms, back @ 120 ms | brief excursion the detector must *not* alarm on |
| `ringing` | `Kd` removed, `Ki=80` + same load steps | poorly damped but still stable |
| `growing` | healthy → marginal gains ramped 30–130 ms | slowly lost damping → early-warning test |
| `oscillation` | healthy → bad gains abruptly at 50 ms | loop goes unstable → full alarm |

```bash
octave buck_sim.m     # writes data/*.csv and data/*.png
```

---

## 2. Sense (signal conditioning)

The ESP32 cannot accept `Vout` directly. A resistor divider scales it into the ADC
range; the emulation uses ratio **0.5**:

```text
V_ADC = 0.5 × Vout        →   5 V maps to 2.5 V
```

(Any `R1/R2` pair with `R2/(R1+R2) = 0.5`, e.g. 10 k/10 k, implements this.)

The stage must preserve the **variation** in voltage — ripple and oscillation are
the signal of interest, not just the 5 V average.

---

## 3. ADC emulation

`emulate_adc()` in `detector.py` reproduces what firmware would receive:

1. **Anti-alias** — centred moving average of the 100 kHz source (length 5)
2. **Sample** at `fs = 20 kHz` (on hardware: a timer / ADC-ready callback)
3. **Divider** — `V_adc = 0.5 × Vout`
4. **Noise** — Gaussian, 0.5 LSB RMS
5. **Clip** to `[0, 3.3 V]`, **quantise** to 12-bit codes (0…4095)

| ADC setting | Value |
|-------------|-------|
| Sampling rate `fs` | 20 kHz |
| Resolution | 12-bit, 3.3 V full scale |
| Effective LSB at Vout | ≈ 1.61 mV |
| Noise | 0.5 LSB RMS (CLI: `--noise-lsb`) |

**Why 20 kHz?** The features of interest live around the LC resonance (~1.1–1.4 kHz)
and the analysis band is 300–5000 Hz. Nyquist requires `fs > 2·f_max` → `fs > 10 kHz`
to cover the band; 20 kHz gives practical margin (~15× the resonance).

---

## 4. Streaming on the ESP32 (circular buffer)

Firmware cannot store an infinite stream, so a **fixed 128-sample ring buffer**
holds the most recent window (`static float buf[N]; uint16_t head;` in C terms).
Each new sample overwrites the oldest — no allocation, constant memory.

| Windowing | Value | At 20 kHz |
|-----------|-------|-----------|
| Window `N` | 128 samples | 6.4 ms |
| Hop | 64 samples | 3.2 ms (50 % overlap) |
| FFT bin width `fs/N` | — | 156.25 Hz |

`Detector.on_sample()` runs once per sample (the timer ISR role); every hop it
calls `process_window()` (the processing task role).

---

## 5. Signal processing

For each window `x[0…N-1]` (in volts at `Vout`):

**DC removal** — the interesting information is the variation around 5 V:

```text
x_ac[n] = x[n] − mean(x)
```

Then two time-domain measures and an FFT:

| Feature | Definition | Question it answers |
|---------|------------|---------------------|
| RMS ripple | `√(mean(x_ac²))` | How much is the output fluctuating overall? |
| Peak-to-peak | `max(x_ac) − min(x_ac)` | Total excursion of the waveform |
| Band RMS | Parseval RMS of FFT bins in **300–5000 Hz** | How much energy is in the oscillation band? |
| Tone RMS | RMS of peak bin ±2 inside the band | Energy of the single dominant oscillation |
| Dominant freq. `f_dom` | parabolic interpolation of the band peak | What frequency is the oscillation at? |
| Growth slope | least-squares slope of `ln(RMS)` over last **5 windows** | Is the ripple getting bigger over time? |

FFT details: Hann window (periodic) before `rfft`; `tonality = tone power / band
power` is computed for plots (near 1 = one pure tone) but is not part of the risk score.

### Switching-frequency caveat

A real buck switches at `fsw = 100 kHz` and deliberately puts energy there — that
is **not** a fault. This project watches the **control-loop band (300–5000 Hz)**
around the LC resonance, far below `fsw`. The averaged simulator removes switching
ripple entirely, so the detector can never mistake `fsw` for an oscillation; on
hardware the same separation comes from the band mask (plus the anti-alias filter).

---

## 6. Risk assessment

### Calibration (from the known-good run)

Before scoring, `detector.py` calibrates on `data/stable.csv` (first 10 ms skipped):

- **Noise-floor gate** = `mean + 5σ` of stable-window RMS / tone RMS → anything
  below this scores 0 (indistinguishable from ADC + load noise)
- **"Severe" level** = `3 %` of Vout as RMS ripple → **150 mV** (a design choice,
  `severe_pct` in config) → score 1

### Score

Each term is normalised 0…1 on a logarithmic (dB-style) scale — oscillation
amplitude can change by 100×:

```text
A = db_score(RMS ripple)        # amplitude
E = db_score(tone RMS)          # narrow-band oscillation energy
G = clip((slope − 0.05)/(0.30 − 0.05), 0, 1)   # growth; only counts if A > 0

R = 0.4·A + 0.4·E + 0.2·G       # weights sum to 1
```

Conceptually: **how large is the ripple + how much of it is one tone + is it growing**.

### State machine (persistence + hysteresis)

| From | To | Condition |
|------|----|-----------|
| NORMAL | WARNING | **4** consecutive windows with `R ≥ 0.20` (~13 ms) |
| WARNING | OSCILLATION | **8** consecutive windows with `R ≥ 0.55` (~26 ms) |
| WARNING | NORMAL | **6** consecutive calm windows (`R < 0.20`) |
| OSCILLATION | WARNING | **6** consecutive windows with `R < 0.55` |

Persistence means a single load transient (a few excited windows that die away)
does not raise an alarm; hysteresis (different up/down thresholds + separate calm
counters) stops the state from flickering. Escalation always passes through
`WARNING` — real systems degrade before they fully oscillate, and that intermediate
state is the early warning.

### What "predictive" means here

```text
NORMAL → ripple growing → tone energy rising → WARNING → persistent oscillation → OSCILLATION
```

The system flags **increasing abnormal behaviour before ripple becomes severe**.
The detector prints a **lead time**: `severe@ − first WARNING` (positive = warning
came first). It does **not** predict "failure in N seconds."

---

## 7. Expected behaviour per scenario

| Scenario | Expected classification |
|----------|-------------------------|
| `stable` | stays `NORMAL` |
| `transient` | brief activity absorbed by the 4-window persistence → mostly `NORMAL` |
| `ringing` | tone visible, risk may approach threshold, no sustained `OSCILLATION` |
| `growing` | `WARNING` during the 30–130 ms damping ramp (growth + amplitude terms) |
| `oscillation` | escalates to `OSCILLATION` after 8 windows ≥ 0.55 risk from 50 ms on |

Note: mean voltage can stay ≈ 5 V even while oscillating — DC measurement alone
cannot detect this; that is why frequency-domain and trend features exist.

---

## Running the detector

```bash
python -m venv venv && source venv/bin/activate
pip install numpy scipy matplotlib

python detector.py                 # all five scenarios → results/*.png
python detector.py --file data/growing.csv
python detector.py --noise-lsb 1.5 # noisier ADC
python detector.py --fs 10000      # try a different sampling rate
python detector.py --no-show       # save figures only
```

Console output per scenario: % time in each state, first `WARNING` / first
`OSCILLATION` time, `severe@` reference, **lead**, median alarm frequency
(`f_alarm`), and final state.

Figures written to `results/`:

- `<name>_detector.png` — waveform + state colour bands, per-window FFT spectrogram
  with dominant-frequency track and band markers, log-scale features with the
  calibrated noise-floor gate, risk score with `r_warn` / `r_osc` lines
- `<name>_spectrum.png` — spectrum of the highest-risk window

Every tunable threshold lives in `ADCConfig` / `DetectorConfig` at the top of
`detector.py` — nothing is hidden in the algorithm body.

---

## Mapping to embedded firmware

| Prototype code | ESP32 equivalent |
|----------------|------------------|
| `emulate_adc()` | resistor divider + ADC + hardware timer at `fs` |
| `CircularBuffer` | `static float buf[128]; uint16_t head;` |
| `on_sample()` loop | timer ISR / ADC-ready callback |
| `process_window()` | periodic DSP task every hop |
| `extract_features()` | one C function: mean, RMS, p-p, real FFT |
| `risk_score()` | a few multiplies + `logf` |
| `StateMachine` | `enum` + `switch` |
| prints / PNGs | OLED / LED / buzzer / serial |

The Python code is structured so each block can be translated 1:1 into firmware.
No ESP32 firmware is included yet — the prototype defines the algorithm and its
memory/timing budget (128-sample window, ~61 windows per 200 ms record).

---

## Advantages

- Early warning of deteriorating damping **before** ripple becomes severe
- Combines time-domain (RMS, p-p, trend) and frequency-domain (band/tone FFT) evidence
- Constant memory and MCU-friendly math (ring buffer, 128-point FFT, slope over 5 points)
- Threshold-based and interpretable — no machine learning required
- Calibrated against a known-good baseline (automatic noise-floor gating)

## Applications

- DC-DC converter health monitoring
- Embedded power-management systems
- Battery-powered and automotive electronics
- Distributed power systems / supply health assessment

## Limitations

- **Simulated only** — averaged model, no ESR/ESL, no component tolerances, no hardware ADC effects beyond quantisation + Gaussian noise
- Single operating point (12 V → 5 V, one load profile); input-voltage disturbances are not simulated
- Fixed thresholds and weights; severe level is a design choice, not a specification
- No phase-margin measurement in the detector — damping is created by gain scheduling and checked offline via closed-loop poles
- No closed-loop control action: the system **monitors and alerts**, it does not stabilize

## Future scope

- Real-time ESP32 firmware (divider → ADC timer → ISR ring buffer → FFT task → OLED/alert)
- Hardware validation against a physical buck board
- Closed-loop extension: `SENSE → ANALYZE → PREDICT → CONTROL → PWM adjustment → REGULATOR`
- Adaptive / learned thresholds (machine learning) on top of the same feature vector
- Multi-rail and input-voltage disturbance scenarios
