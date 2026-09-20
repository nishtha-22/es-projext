# Predictive Oscillation Detector for Power Regulators

## Overview

Voltage-mode buck converters require a stable feedback control loop to ensure reliable operation.

Traditionally, stability is evaluated using:

- Loop Gain
- Crossover Frequency
- Phase Margin

As the phase margin decreases, the converter becomes less stable and exhibits increased ringing and oscillations in its transient response.

This project investigates whether transient ringing characteristics can be used to predict deteriorating loop stability. A simulation-based detector is developed to classify regulator behaviour into:

- Stable
- Warning Level
- Highly Oscillatory

### Key Concept

```text
Phase Margin ↓
       ↓
Transient Ringing ↑
       ↓
Stability Warning
```

---

## Objectives

- Design and simulate a voltage-mode buck converter.
- Obtain loop gain and Bode plot characteristics.
- Calculate crossover frequency and phase margin.
- Apply controlled input and load disturbances.
- Measure transient response characteristics.
- Extract ringing features from output voltage.
- Establish correlation between phase margin and ringing behaviour.
- Develop a threshold-based oscillation detection algorithm.
- Classify converter stability levels automatically.

---

## Background Questions

The project seeks to address the following questions:

1. What is a buck converter?
2. Why is feedback required?
3. What is loop gain?
4. What is a Bode plot?
5. What is crossover frequency?
6. What is phase margin?
7. Why does a lower phase margin cause ringing?
8. What disturbance is being applied?
9. How is ringing measured?
10. How can ringing indicate poor stability?
11. What exactly does the oscillation detector identify?
12. How is this different from conventional Bode analysis?
13. Why use simulation instead of hardware?
14. What are the limitations of the approach?
15. How can this be extended to a real embedded system?

---

## Methodology

### Step 1: Converter Design

Design a voltage-mode buck converter using SPICE or Simulink.

### Step 2: Stability Analysis

Obtain the loop-gain response and generate the Bode plot.

Determine:

- Loop Gain
- Crossover Frequency
- Phase Margin

### Step 3: Create Multiple Stability Conditions

Modify compensation parameters or operating conditions to generate different phase margins.

### Step 4: Apply Disturbances

Introduce controlled disturbances such as:

- Load current step changes
- Input voltage variations

### Step 5: Capture Transient Response

Record the output voltage response:

```text
Vout(t)
```

after each disturbance.

### Step 6: Extract Ringing Features

Measure:

- Peak Overshoot
- Ringing Amplitude
- Ringing Frequency
- Settling Time
- Number of Oscillation Cycles

### Step 7: Correlation Analysis

Compare transient ringing metrics against calculated phase margins.

Establish relationships such as:

```text
Lower Phase Margin
       ↓
Larger Overshoot
       ↓
Longer Settling Time
       ↓
More Ringing Cycles
```

### Step 8: Oscillation Detection Algorithm

Implement a threshold-based detector that classifies the converter as:

| Class | Description |
|---------|-------------|
| Stable | Minimal ringing, adequate phase margin |
| Warning | Noticeable ringing, reduced phase margin |
| Highly Oscillatory | Sustained oscillations, poor stability margin |

---

## Tools

### Circuit Simulation

- PSpice
- NGSpice

### Modelling and Control Analysis

- MATLAB
- Simulink

### Signal Processing and Automation

- MATLAB Scripts
- Python

---

## Detection Parameters

The oscillation detector uses transient-response features such as:

| Parameter | Purpose |
|------------|----------|
| Peak Overshoot | Indicates damping quality |
| Ringing Amplitude | Measures oscillation severity |
| Ringing Frequency | Identifies resonance behaviour |
| Settling Time | Reflects stability margin |
| Oscillation Count | Detects persistent ringing |

---

## Expected Results

The project is expected to demonstrate that:

- Reduced phase margin produces larger transient ringing.
- Time-domain measurements can indicate loop stability.
- Ringing features can predict deteriorating stability before complete oscillation occurs.
- A simple detector can classify system stability without continuously performing loop-gain analysis.

---

## Advantages

- Provides early warning of deteriorating stability.
- Connects frequency-domain metrics with time-domain behaviour.
- Enables predictive stability monitoring.
- Reduces dependence on repeated loop-gain calculations.
- Forms a foundation for future embedded implementation.

---

## Applications

- DC-DC Converters
- Embedded Power Management Systems
- Battery-Powered Devices
- Automotive Electronics
- Distributed Power Systems
- Adaptive Stability Monitoring
- Power Supply Health Assessment

---

## Future Scope

Potential extensions include:

- Real-time embedded implementation on a microcontroller.
- Machine learning
