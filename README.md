# Project Handoff: Predictive Oscillation Detector for Switching Power Regulators

## 1. Project overview

An embedded-style monitoring pipeline for a **12 V → 5 V voltage-mode buck converter**.

The system monitors output-voltage behaviour, processes sampled waveforms, extracts signal features, calculates a risk score and classifies regulator behaviour into three states:

- NORMAL
- WARNING
- OSCILLATION

The purpose is to identify deteriorating regulator behaviour and detect growing oscillations before they become severe.

**Important:** The entire current project is simulation-based. No physical buck converter, ESP32 ADC acquisition or hardware measurements have been implemented. The ESP32 acquisition chain is emulated in software.

## 2. System architecture

REGULATOR → SENSE → ADC → ESP32 → SIGNAL PROCESSING → FEATURE EXTRACTION → RISK ASSESSMENT → STATE CLASSIFICATION

The project has two main implementation components:

| File | Purpose |
|---|---|
| `buck_sim.m` | GNU Octave simulation of the buck converter and its control-loop scenarios |
| `detector.py` | Python prototype of the embedded monitoring and classification pipeline |

Other directories:
- `data/`: five simulated CSV waveforms and simulator plots.
- `results/`: generated detector figures.

## 3. Converter model

The simulated plant is a 12 V to 5 V buck converter with voltage-mode PID regulation.

| Parameter | Value |
|---|---:|
| Input voltage | 12 V |
| Reference output voltage | 5 V |
| Switching frequency | 100 kHz |
| Inductance | 100 µH |
| Output capacitance | 220 µF |
| Inductor DCR | 0.05 Ω |
| Nominal load | 10 Ω |
| Nominal output current | 0.5 A |
| LC resonant frequency | Approximately 1.07 kHz |
| Simulation duration | 200 ms |
| Stored waveform sampling rate | 100 kS/s |

The model is averaged rather than switching-level. It represents control-loop dynamics and does not reproduce the actual 100 kHz switching ripple.

## 4. Five simulated operating scenarios

1. **Stable:** Healthy controller gains and constant load. Used as the baseline.
2. **Transient:** Load changes from 10 Ω to 5 Ω at 50 ms and returns at 120 ms. Tests rejection of temporary disturbances.
3. **Ringing:** Reduced derivative action and lower integral gain create poorly damped but stable behaviour.
4. **Growing:** Controller gains gradually become marginal between 30 and 130 ms, producing deteriorating damping.
5. **Oscillation:** Controller gains abruptly become unstable at 50 ms.

## 5. Sensing and ADC emulation

The proposed sensing stage uses a resistor divider to scale output voltage into the ESP32 ADC range.

- Divider ratio: 0.5
- 5 V output becomes 2.5 V at the ADC input.
- Example divider: 10 kΩ + 10 kΩ.

The Python ADC emulator performs:

1. Moving-average anti-alias filtering.
2. Sampling at 20 kHz.
3. Voltage-divider scaling.
4. Gaussian noise injection at 0.5 LSB RMS.
5. Clipping to 0–3.3 V.
6. Quantisation to 12-bit ADC codes (0–4095).

The 20 kHz sampling rate is selected to capture the 300–5000 Hz analysis band.

## 6. ESP32-style data acquisition

The prototype emulates an embedded acquisition pipeline using:

- 128-sample circular buffer.
- 20 kHz sampling frequency.
- 64-sample hop size.
- 50% overlapping windows.
- 6.4 ms window duration.
- 3.2 ms processing interval.

Each new sample overwrites the oldest sample in the ring buffer. Every 64 samples, the detector processes a new window.

## 7. Signal processing and feature extraction

For each window, the detector removes the DC component:

`x_ac[n] = x[n] - mean(x)`

It extracts six principal features:

| Feature | Purpose |
|---|---|
| RMS ripple | Measures overall output-voltage fluctuation |
| Peak-to-peak ripple | Measures maximum waveform excursion |
| Band RMS | Measures oscillation energy between 300–5000 Hz |
| Tone RMS | Measures energy around the dominant frequency |
| Dominant frequency | Identifies the main oscillation frequency |
| Growth slope | Determines whether RMS ripple is increasing over successive windows |

A Hann-windowed FFT is used for frequency-domain analysis.

## 8. Risk assessment

The detector calibrates against the stable scenario.

- Noise floor: mean + 5 standard deviations of baseline RMS/tone RMS.
- Severe RMS ripple reference: 3% of 5 V = 150 mV.

The normalised features are combined into a risk score:

`R = 0.4A + 0.4E + 0.2G`

Where:
- A = amplitude score.
- E = oscillation-tone energy score.
- G = ripple growth score.

The score ranges from 0 to 1.

## 9. Three-state classification

The detector uses persistence and hysteresis to avoid false alarms and rapid state switching.

| Transition | Condition |
|---|---|
| NORMAL → WARNING | 4 consecutive windows with risk ≥ 0.20 |
| WARNING → OSCILLATION | 8 consecutive windows with risk ≥ 0.55 |
| WARNING → NORMAL | 6 consecutive calm windows with risk below 0.20 |
| OSCILLATION → WARNING | 6 consecutive windows with risk below 0.55 |

The detector does not jump directly from NORMAL to OSCILLATION. It passes through WARNING to represent progressive deterioration.

### Interpretation of modes

- **NORMAL:** Output behaviour remains within the calibrated baseline.
- **WARNING:** Abnormal fluctuations or increasing oscillation risk are detected.
- **OSCILLATION:** Sustained high-risk oscillatory behaviour is detected.

## 10. Expected classification

| Scenario | Expected result |
|---|---|
| Stable | NORMAL |
| Transient | Mostly NORMAL; temporary disturbances should not trigger sustained alarms |
| Ringing | Visible frequency content, but no sustained OSCILLATION |
| Growing | WARNING as damping deteriorates |
| Oscillation | Escalation to OSCILLATION |

## 11. Implementation mapping

| Python prototype | Intended ESP32 implementation |
|---|---|
| `emulate_adc()` | Resistor divider + ADC + hardware timer |
| `CircularBuffer` | Fixed-size 128-sample array |
| `on_sample()` | Timer ISR / ADC-ready callback |
| `process_window()` | Periodic DSP task |
| `extract_features()` | C functions for RMS, peak-to-peak and FFT |
| `risk_score()` | Embedded risk calculation |
| `StateMachine` | C enum and switch-case state machine |
| Console output and plots | OLED, LED, buzzer or serial output |

No actual ESP32 firmware is included yet. The Python implementation is a prototype designed to map onto embedded firmware.

## 12. Advantages

- Detects deteriorating damping before severe oscillations.
- Combines time-domain and frequency-domain signal analysis.
- Uses interpretable features rather than machine learning.
- Uses a fixed-size circular buffer for predictable memory usage.
- Uses persistence and hysteresis to reduce false alarms.
- Calibrates risk against a known-good simulated baseline.

## 13. Limitations

- Entire project is simulation-based.
- Averaged converter model does not include switching ripple.
- No physical ADC or sensor measurements.
- No real-time ESP32 firmware.
- Fixed thresholds and weights.
- Only one input-voltage operating point and limited load scenarios.
- The detector monitors and alerts; it does not stabilise the converter or adjust PWM.

## 14. Future scope

- Implement the acquisition and DSP pipeline on an ESP32.
- Validate detection using a physical buck converter.
- Add OLED/LED/buzzer alerts for the three states.
- Extend the monitor into a closed-loop system with corrective PWM control.
- Explore adaptive thresholds and machine-learning-based classification.

## 15. Important distinction: P&O vs this project

Perturb & Observe (P&O) is an MPPT algorithm that adjusts converter duty cycle to maximise photovoltaic power.

It is **not the algorithm used in this project**.

This project uses signal processing, a weighted risk score and a three-state classification machine to detect regulator instability.

Do not mix the solar MPPT boost-converter project with this buck-converter oscillation detector in the presentation.
