import os
import sys

import matplotlib
matplotlib.use("Agg")

import numpy as np
from manim import *

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detector as det

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# --- unified palette (state colours mirror detector.py) -----------------------
C_NORMAL = det.STATE_COLOR[det.NORMAL]          # "#2ca02c"
C_WARN = det.STATE_COLOR[det.WARNING]          # "#ff9f0e"
C_OSC = det.STATE_COLOR[det.OSCILLATION]      # "#d62728"
C_CYAN = "#00c8d7"
C_VIOLET = "#8b5cf6"
C_GOLD = "#f5c542"
GREY_SUB = "#a8a8a8"

L = 100e-6          # H, buck inductor  (buck_sim.m)
C = 220e-6          # F, output cap     (buck_sim.m)
RESONANCE_HZ = 1.0 / (2 * np.pi * np.sqrt(L * C))     # ~1073 Hz


# --- data / detector wiring ---------------------------------------------------
def _init_detector():
    ADC = det.ADCConfig()
    CFG = det.DetectorConfig()
    t, v = det.load_csv(os.path.join(DATA, "stable.csv"))
    t_adc, codes = det.emulate_adc(t, v, ADC)
    feats = det.features_only(codes[int(0.01 * ADC.fs):], ADC, CFG)
    cal = det.calibrate_from_stable(feats, CFG, v_nominal=float(np.mean(v)))
    return ADC, CFG, cal


ADC, CFG, CAL = _init_detector()

HOP_MS = CFG.hop / ADC.fs * 1000.0
WIN_MS = CFG.window / ADC.fs * 1000.0
LSB_UV = ADC.v_ref / (2 ** ADC.bits - 1) / ADC.divider * 1e3


def scenario(name):
    t, v = det.load_csv(os.path.join(DATA, f"{name}.csv"))
    t_adc, codes = det.emulate_adc(t, v, ADC)
    res = det.run_stream(codes, t_adc, ADC, CFG, cal=CAL)
    v_seen = det.code_to_vout(codes, ADC)
    return {
        "name": name,
        "t": t_adc,
        "v": v_seen,
        "res": res,
        "tw": np.array([r["t"] for r in res]),
        "R": np.array([r["R"] for r in res]),
        "states": [r["state"] for r in res],
        "rms": np.array([r["rms"] for r in res]),
        "A": np.array([r["A"] for r in res]),
        "E": np.array([r["E"] for r in res]),
        "G": np.array([r["G"] for r in res]),
    }


GROW = scenario("growing")
STAB = scenario("stable")
OSCI = scenario("oscillation")


def downsample(t, y, n=1800):
    idx = np.linspace(0, len(t) - 1, n).astype(int)
    return t[idx], y[idx]


def window_at(scn, t_query, cfg=CFG):
    idx = int(np.argmin(np.abs(scn["tw"] - t_query)))
    res = scn["res"][idx]
    t0 = res["t"] - cfg.window / ADC.fs
    i0 = int(np.argmin(np.abs(scn["t"] - t0)))
    x = scn["v"][i0:i0 + cfg.window]
    return x, res


def code_values(t_query):
    t, v = det.load_csv(os.path.join(DATA, "growing.csv"))
    t_adc, codes = det.emulate_adc(t, v, ADC)
    i = int(np.argmin(np.abs(t_adc - t_query)))
    return t_adc[i:i + 8], codes[i:i + 8]


# --- shared layout helpers ----------------------------------------------------
def header(scene, text, color):
    t = Text(text, font_size=30, color=color, weight=BOLD).to_edge(UP, buff=0.28)
    rule = Line(LEFT * 3.4, RIGHT * 3.4, color=color, stroke_width=2.0).next_to(t, DOWN, buff=0.16)
    return VGroup(t, rule)


def footer(scene, text):
    return Text(text, font_size=15, color=GREY_C).to_edge(DOWN, buff=0.28)


def axes2d(x_range, y_range, x_len, y_len, **kw):
    cfg = {"color": GREY_B, "include_numbers": False, "stroke_width": 1.8}
    cfg.update(kw)
    return Axes(
        x_range=x_range, y_range=y_range,
        x_length=x_len, y_length=y_len,
        axis_config=cfg, tips=False,
    )


def axis_ticks(ax, xs, ys, font=12):
    """Manual numeric tick labels (cleaner control than include_numbers)."""
    out = VGroup()
    for v in xs:
        out.add(Text(f"{v:g}", font_size=font, color=GREY_B).next_to(ax.c2p(v, 0), DOWN, buff=0.09))
    for v in ys:
        out.add(Text(f"{v:g}", font_size=font, color=GREY_B).next_to(ax.c2p(0, v), LEFT, buff=0.10))
    return out


# =============================================================================
# Scene 1 - pipeline overview
# =============================================================================
class PipelineScene(Scene):
    def construct(self):
        title = Text(
            "Predictive Oscillation Detector",
            font_size=40, color=C_GOLD, weight=BOLD,
        )
        sub = Text(
            "for Switching Power Regulators",
            font_size=24, color=GREY_SUB,
        ).next_to(title, DOWN, buff=0.18)
        self.play(FadeIn(title, shift=DOWN * 0.3), FadeIn(sub, shift=DOWN * 0.2), run_time=1.0)
        self.wait(1.0)
        self.play(FadeOut(title), FadeOut(sub))

        blocks = [
            ("REGULATOR", "buck 12V → 5V", BLUE),
            ("SENSE", "voltage divider", TEAL),
            ("ADC", f"{ADC.bits}-bit @ {ADC.fs/1e3:.0f} kHz", GREEN),
            ("ESP32", "circular buffer", YELLOW),
            ("FEATURES", "RMS · Vpp · FFT", ORANGE),
            ("RISK", f"{CFG.w_amp:g}A+{CFG.w_tone:g}E+{CFG.w_growth:g}G", RED),
            ("STATE", "N · W · OSC", MAROON),
        ]

        boxes, labels, subs = VGroup(), VGroup(), VGroup()
        for name, small, col in blocks:
            box = RoundedRectangle(
                corner_radius=0.12, width=1.55, height=0.85,
                stroke_color=col, stroke_width=3,
                fill_color=col, fill_opacity=0.12,
            )
            lab = Text(name, font_size=17, color=col, weight=BOLD).move_to(box)
            sml = Text(small, font_size=11, color=GREY_B).next_to(box, DOWN, buff=0.12)
            boxes.add(box); labels.add(lab); subs.add(sml)

        row = VGroup(*[VGroup(boxes[i], labels[i], subs[i]) for i in range(len(blocks))])
        row.arrange(RIGHT, buff=0.28)
        row.scale_to_fit_width(config.frame_width - 1.8)

        arrows = VGroup()
        for i in range(len(row) - 1):
            a = Arrow(
                row[i][0].get_right(), row[i + 1][0].get_left(),
                buff=0.04, stroke_width=3,
                max_tip_length_to_length_ratio=0.18, color=GREY_A,
            )
            arrows.add(a)

        self.play(
            LaggedStart(*[FadeIn(m, shift=UP * 0.2) for m in row], lag_ratio=0.12),
            run_time=1.8,
        )
        self.play(Create(arrows), run_time=0.9)

        for i, a in enumerate(arrows):
            dot = Dot(radius=0.06, color=WHITE).move_to(row[i][0])
            self.play(
                MoveAlongPath(dot, a),
                boxes[i].animate.set_fill(opacity=0.35),
                boxes[i + 1].animate.set_fill(opacity=0.35),
                run_time=0.35,
            )
            self.play(boxes[i].animate.set_fill(opacity=0.12), FadeOut(dot), run_time=0.15)

        states = VGroup(
            Text("NORMAL", font_size=20, color=C_NORMAL, weight=BOLD),
            Text("WARNING", font_size=20, color=C_WARN, weight=BOLD),
            Text("OSCILLATION", font_size=20, color=C_OSC, weight=BOLD),
        ).arrange(RIGHT, buff=0.7)
        states.next_to(row, DOWN, buff=0.55)

        tagline = Text(
            "voltage → observe → digitize → analyze → decide",
            font_size=17, color=GREY_C,
        ).next_to(states, DOWN, buff=0.4)

        self.play(FadeIn(states, lag_ratio=0.2), run_time=0.8)
        self.play(Write(tagline), run_time=1.2)
        self.wait(2.5)
        self.play(*[FadeOut(m) for m in self.mobjects])


# =============================================================================
# Scene 2 - sense + ADC + circular buffer
# =============================================================================
class SenseAdcScene(Scene):
    def construct(self):
        hd = header(self, "SENSE  →  ADC  →  CIRCULAR BUFFER", C_CYAN)
        self.play(FadeIn(hd, shift=DOWN * 0.2))
        self.wait(0.4)

        t0, t1 = 0.100, 0.160
        m = (GROW["t"] >= t0) & (GROW["t"] <= t1)
        tw, vw = downsample(GROW["t"][m], GROW["v"][m], 1500)
        tw_ms = (tw - t0) * 1e3

        axes = axes2d(
            [0, 60, 10], [4.3, 5.7, 0.2], 7.0, 2.2,
        ).shift(UP * 1.15)
        xlabel = Text("time (ms)", font_size=15, color=GREY_B).next_to(axes.x_axis, DOWN, buff=0.18)
        ylabel = Text("Vout (V)", font_size=15, color=GREY_B).next_to(axes.y_axis, LEFT, buff=0.22)
        ticks = axis_ticks(axes, [0, 20, 40, 60], [4.4, 4.8, 5.2, 5.6])

        ref_line = DashedLine(
            axes.c2p(0, 5.0), axes.c2p(60, 5.0), color=C_GOLD, stroke_width=1.5, dash_length=0.08
        )
        ref_txt = Text("Vout = 5 V", font_size=14, color=C_GOLD).next_to(ref_line, RIGHT, buff=0.10)

        curve = axes.plot_line_graph(
            x_values=tw_ms, y_values=vw,
            add_vertex_dots=False, line_color=WHITE, stroke_width=2.2,
        )["line_graph"]

        self.play(Create(axes), FadeIn(ticks), FadeIn(xlabel), FadeIn(ylabel), run_time=0.7)
        self.play(Create(ref_line), FadeIn(ref_txt), run_time=0.4)
        self.play(Create(curve), run_time=2.2)

        note = footer(self, "growing scenario, 100–160 ms  (simulated)")
        self.play(FadeIn(note))
        self.wait(0.6)

        # spec strip: divider + ADC/Nyquist, left-aligned under the plot
        fs = ADC.fs
        spec1 = Text(
            f"SENSE:  V_adc = {ADC.divider:g} × Vout   (scaled to stay in 0…{ADC.v_ref:g} V)",
            font_size=16, color=TEAL,
        )
        spec2 = Text(
            f"ADC: {ADC.bits}-bit,  fs = {fs/1e3:.0f} kHz  (> 2 × {CFG.band[1]/1e3:.0f} kHz Nyquist  ·  LSB ≈ {LSB_UV:.1f} mV)",
            font_size=16, color=GREEN,
        )
        spec = VGroup(spec1, spec2).arrange(DOWN, aligned_edge=LEFT, buff=0.10)
        spec.align_to(axes, LEFT).shift(DOWN * 0.75)
        self.play(FadeIn(spec1, shift=RIGHT * 0.2), FadeIn(spec2, shift=RIGHT * 0.2))
        self.wait(0.8)

        # sampling demo — faint stream of ADC samples
        ts = np.arange(t0, t1, 1.0 / (fs * 8))
        ts_clip = ts[ts <= t1]
        vs = np.interp(ts_clip, GROW["t"], GROW["v"])

        stream_dots = VGroup(
            *[Dot(axes.c2p((ti - t0) * 1e3, vi), radius=0.02, color=C_CYAN, fill_opacity=0.35)
              for ti, vi in zip(ts_clip, vs)],
        )

        # the 8 codes actually captured at t = 132 ms (matches the displayed codes)
        h_t, codes = code_values(0.132)
        i0 = int(np.argmin(np.abs(GROW["t"] - h_t[0])))
        h_v = GROW["v"][i0:i0 + 8]
        highlight = VGroup()
        link = VGroup()
        for ti, vi in zip(h_t, h_v):
            p = axes.c2p((ti - t0) * 1e3, vi)
            link.add(Line(axes.c2p((ti - t0) * 1e3, 4.3), p, color=GREEN, stroke_width=1.4, stroke_opacity=0.5))
            highlight.add(Dot(p, radius=0.055, color=GREEN))

        self.play(FadeIn(stream_dots), run_time=1.2)
        samp_lbl = Text(f"ADC samples @ {fs/1e3:.0f} kHz  (every {(1/fs)*1e6:.0f} µs)", font_size=14, color=C_CYAN)
        samp_lbl.next_to(axes, UP, buff=0.10).align_to(axes, RIGHT)
        self.play(FadeIn(samp_lbl))
        self.wait(0.6)
        self.play(Create(link), FadeIn(highlight), run_time=1.0)

        code_txt = Text(
            "captured codes:  " + "  ".join(str(int(c)) for c in codes),
            font_size=16, color=GREEN,
        )
        code_txt.next_to(axes, DOWN, buff=0.10).align_to(axes, LEFT)
        self.play(FadeIn(code_txt, shift=UP * 0.15))
        self.wait(1.2)

        self.play(
            FadeOut(stream_dots), FadeOut(highlight), FadeOut(link), FadeOut(samp_lbl),
            FadeOut(code_txt), FadeOut(spec), FadeOut(ref_txt),
            run_time=0.6,
        )

        # circular buffer — 16 schematic cells for the 128-sample ring
        buf_head_txt = Text(
            f"circular buffer: {CFG.window} samples = {WIN_MS:.1f} ms,  newest overwrites oldest",
            font_size=17, color=YELLOW,
        )
        buf_head_txt.move_to([0, -0.55, 0])
        self.play(FadeIn(buf_head_txt))

        n_cells = 16
        cell_w, cell_h = 0.34, 0.34
        cells = VGroup()
        for i in range(n_cells):
            r = RoundedRectangle(
                corner_radius=0.04, width=cell_w, height=cell_h,
                stroke_color=GREY_A, stroke_width=2,
                fill_color=BLACK, fill_opacity=0.6,
            )
            cells.add(r)
        buf_row = VGroup(*cells).arrange(RIGHT, buff=0.04)
        buf_row.move_to([0, -1.65, 0])
        head_txt = Text("head", font_size=13, color=YELLOW).next_to(cells[0], UP, buff=0.16)
        arrow = Arrow(
            head_txt.get_bottom(), cells[0].get_top(),
            buff=0.02, stroke_width=2, color=YELLOW,
            max_tip_length_to_length_ratio=0.2,
        )

        self.play(FadeIn(buf_row), FadeIn(head_txt), Create(arrow))
        self.wait(0.4)

        n_fill = 12
        for i in range(n_fill):
            self.play(cells[i].animate.set_fill(YELLOW, 0.45), run_time=0.1)
        full_lbl = Text("window full → process", font_size=15, color=GREY_C)
        full_lbl.next_to(buf_row, RIGHT, buff=0.25)
        self.play(FadeIn(full_lbl))
        self.wait(0.5)

        h = n_fill
        for k in range(6):
            old = h % n_cells
            h += 1
            new = h % n_cells
            new_top = cells[new].get_top()
            self.play(
                cells[old].animate.set_fill(YELLOW, 0.1),
                head_txt.animate.move_to(new_top + UP * 0.3),
                arrow.animate.put_start_and_end_on(new_top + UP * 0.18, new_top),
                run_time=0.35,
            )
            self.play(cells[old].animate.set_fill(YELLOW, 0.45), run_time=0.2)

        self.wait(1.5)
        self.play(*[FadeOut(m) for m in self.mobjects])


# =============================================================================
# Scene 3 - features (DC removal, RMS/p-p, FFT)
# =============================================================================
class FeaturesScene(Scene):
    def construct(self):
        hd = header(self, "SIGNAL PROCESSING  →  FEATURES", ORANGE)
        self.play(FadeIn(hd, shift=DOWN * 0.2))
        self.wait(0.3)

        t_q = 0.145
        x, res = window_at(GROW, t_q)
        n = len(x)
        x_ac = x - float(np.mean(x))
        rms_v, p2p_v = float(res["rms"]), float(res["p2p"])

        # ---- phase A: raw window (around 5 V)
        left_raw = axes2d([0, n, 16], [4.3, 5.7, 0.2], 5.6, 2.2).shift(LEFT * 3.0 + UP * 0.7)
        lxl = Text("raw window (Vout)", font_size=15, color=GREY_B).next_to(left_raw, DOWN, buff=0.18)
        meta1 = Text(
            f"{CFG.window} samples  ·  {WIN_MS:.1f} ms",
            font_size=13, color=GREY_C,
        ).next_to(left_raw, UP, buff=0.08)
        raw_line = left_raw.plot_line_graph(
            x_values=np.arange(n), y_values=x,
            add_vertex_dots=False, line_color=WHITE, stroke_width=2.2,
        )["line_graph"]
        ref5 = DashedLine(
            left_raw.c2p(0, 5.0), left_raw.c2p(n, 5.0), color=C_GOLD, stroke_width=1.2, dash_length=0.06
        )

        self.play(Create(left_raw), FadeIn(lxl), FadeIn(meta1), run_time=0.6)
        self.play(Create(ref5), run_time=0.3)
        self.play(Create(raw_line), run_time=1.4)

        # ---- phase B: DC removed
        dc_lbl = Text(f"remove DC:  x[n] − mean(x)  →  rms = {rms_v*1e3:.0f} mV", font_size=16, color=C_GOLD)
        dc_lbl.next_to(lxl, DOWN, buff=0.18).align_to(lxl, LEFT)
        self.play(FadeIn(dc_lbl), run_time=0.5)
        self.wait(0.4)

        left_ac = axes2d([0, n, 16], [-0.7, 0.7, 0.35], 5.6, 2.2).shift(LEFT * 3.0 + UP * 0.7)
        acl = Text("V − DC (V)", font_size=14, color=GREY_B).next_to(left_ac.y_axis, LEFT, buff=0.12)
        ac_line = left_ac.plot_line_graph(
            x_values=np.arange(n), y_values=x_ac,
            add_vertex_dots=False, line_color=C_CYAN, stroke_width=2.4,
        )["line_graph"]
        zero = DashedLine(left_ac.c2p(0, 0), left_ac.c2p(n, 0), color=C_GOLD, stroke_width=1.2, dash_length=0.06)

        self.play(
            FadeOut(left_raw), FadeOut(raw_line), FadeOut(ref5), FadeOut(lxl), FadeOut(meta1),
            FadeIn(left_ac), FadeIn(acl), Create(ac_line), Create(zero),
            run_time=0.8,
        )

        rms_mark = DashedLine(left_ac.c2p(0, rms_v), left_ac.c2p(n, rms_v), color=C_OSC, stroke_width=1.8, dash_length=0.05)
        rms_txt = Text(f"RMS = {rms_v*1e3:.0f} mV", font_size=14, color=C_OSC)
        rms_txt.next_to(rms_mark, UP, buff=0.06).align_to(left_ac, LEFT)
        p2p_txt = Text(f"peak-to-peak = {p2p_v*1e3:.0f} mV", font_size=14, color=ORANGE)
        p2p_txt.next_to(dc_lbl, DOWN, buff=0.18).align_to(dc_lbl, LEFT)

        self.play(Create(rms_mark), FadeIn(rms_txt), FadeIn(p2p_txt), run_time=0.7)
        self.wait(1.2)

        # ---- FFT panel (right)
        fft_axes = axes2d([0, 6000, 1000], [0, 1.0, 0.25], 5.4, 2.2).shift(RIGHT * 3.6 + UP * 0.75)
        fxl = Text("frequency (Hz)", font_size=15, color=GREY_B).next_to(fft_axes, DOWN, buff=0.16)
        fyl = Text("amplitude", font_size=15, color=GREY_B).next_to(fft_axes.y_axis, LEFT, buff=0.14)
        fft_lbl = Text("Hann + rfft → one-sided spectrum", font_size=13, color=GREY_C).next_to(fxl, DOWN, buff=0.12)

        freqs, spec = res["freqs"], res["spectrum"]
        spec_n = spec / (spec.max() + 1e-12)
        band_lo, band_hi = CFG.band
        fdom = float(res["f_dom"])

        band_rect = Rectangle(
            width=fft_axes.c2p(band_hi, 0)[0] - fft_axes.c2p(band_lo, 0)[0],
            height=2.2, stroke_width=0, fill_color=C_VIOLET, fill_opacity=0.12,
        ).move_to(fft_axes.c2p((band_lo + band_hi) / 2, 0.5))
        band_txt = Text(f"band  {band_lo:.0f}–{band_hi:.0f} Hz", font_size=12, color=C_VIOLET)
        band_txt.next_to(fft_axes.c2p((band_lo + band_hi) / 2, 0), UP, buff=0.06)

        m = freqs <= 6000
        fft_curve = fft_axes.plot_line_graph(
            x_values=freqs[m], y_values=spec_n[m],
            add_vertex_dots=False, line_color=C_GOLD, stroke_width=2.2,
        )["line_graph"]
        fdom_line = DashedLine(
            fft_axes.c2p(fdom, 0), fft_axes.c2p(fdom, 1.0), color=C_OSC, stroke_width=1.8, dash_length=0.06
        )
        fdom_txt = Text(f"dominant ≈ {fdom:.0f} Hz", font_size=14, color=C_OSC)
        peak = np.argmax(spec_n[m])
        fdom_txt.next_to(fft_axes.c2p(freqs[m][peak], 1.0), UP, buff=0.08)

        self.play(Create(fft_axes), FadeIn(fxl), FadeIn(fyl), FadeIn(fft_lbl), run_time=0.6)
        self.play(FadeIn(band_rect), FadeIn(band_txt), run_time=0.5)
        self.play(Create(fft_curve), run_time=1.6)
        self.play(Create(fdom_line), FadeIn(fdom_txt), run_time=0.6)
        self.wait(0.5)

        f0_khz = RESONANCE_HZ / 1e3
        f0_txt = footer(
            self,
            f"LC resonance  f₀ = 1/(2π√LC) ≈ {f0_khz:.2f} kHz  —  live dominant ≈ {fdom:.0f} Hz  —  NOT the switching frequency",
        )
        self.play(Write(f0_txt), run_time=1.0)
        self.wait(1.0)

        trend = Text(
            f"trend: slope of ln(RMS) over {CFG.trend_len} windows → growth term G",
            font_size=16, color=MAROON,
        ).next_to(f0_txt, UP, buff=0.15)
        self.play(FadeIn(trend, shift=UP * 0.1))
        self.wait(2.0)
        self.play(*[FadeOut(m) for m in self.mobjects])


# =============================================================================
# Scene 4 - risk score + state machine
# =============================================================================
class RiskStateScene(Scene):
    def construct(self):
        hd = header(self, "RISK ASSESSMENT  →  STATE MACHINE", RED_B)
        self.play(FadeIn(hd, shift=DOWN * 0.2))
        self.wait(0.3)

        formula = Text(
            f"R = {CFG.w_amp:g}·A  +  {CFG.w_tone:g}·E  +  {CFG.w_growth:g}·G",
            font_size=24, color=WHITE, weight=BOLD,
        ).next_to(hd, DOWN, buff=0.28)
        formula_parts = VGroup(
            Text("A = ripple amplitude, log scale", font_size=15, color=C_OSC),
            Text("E = tone energy, log scale", font_size=15, color=C_VIOLET),
            Text("G = growth of ln(RMS)", font_size=15, color=C_GOLD),
        ).arrange(RIGHT, buff=0.55)
        formula_parts.next_to(formula, DOWN, buff=0.12)

        self.play(Write(formula), run_time=0.8)
        self.play(FadeIn(formula_parts, lag_ratio=0.15), run_time=0.7)

        sc_title = Text(
            "scenario: growing  (damping lost 30–130 ms)",
            font_size=16, color=GREY_C,
        ).next_to(formula_parts, DOWN, buff=0.18)

        axes = axes2d([0, 0.20, 0.05], [0, 1.05, 0.25], 8.6, 2.9).shift(DOWN * 0.5)
        ticks = axis_ticks(axes, [0.0, 0.05, 0.10, 0.15, 0.20], [0.0, 0.25, 0.50, 0.75, 1.0])
        xl = Text("time (s)", font_size=14, color=GREY_B).next_to(axes.x_axis, DOWN, buff=0.30)
        yl = Text("risk R", font_size=15, color=GREY_B).next_to(axes.y_axis, LEFT, buff=0.30)

        r_warn, r_osc = CFG.r_warn, CFG.r_osc
        warn_line = DashedLine(axes.c2p(0, r_warn), axes.c2p(0.20, r_warn), color=C_WARN, stroke_width=2, dash_length=0.07)
        osc_line = DashedLine(axes.c2p(0, r_osc), axes.c2p(0.20, r_osc), color=C_OSC, stroke_width=2, dash_length=0.07)
        warn_txt = Text(f"R ≥ {r_warn:.2f} → WARNING", font_size=14, color=C_WARN)
        warn_txt.next_to(warn_line, UP, buff=0.08).align_to(warn_line, LEFT)
        osc_txt = Text(f"R ≥ {r_osc:.2f} → OSCILLATION", font_size=14, color=C_OSC)
        osc_txt.next_to(osc_line, UP, buff=0.08).align_to(osc_line, LEFT)

        tw, R, states = GROW["tw"], GROW["R"], GROW["states"]
        colors = {det.NORMAL: C_NORMAL, det.WARNING: C_WARN, det.OSCILLATION: C_OSC}

        band_bottom = axes.get_bottom()[1] - 0.62
        bands = VGroup()
        for i in range(len(tw)):
            t_a = tw[i] - CFG.hop / ADC.fs
            t_b = tw[i]
            x0, x1 = axes.c2p(max(t_a, 0), 0), axes.c2p(t_b, 0)
            w = x1[0] - x0[0]
            if w <= 0:
                continue
            bands.add(Rectangle(
                width=w, height=0.16, stroke_width=0,
                fill_color=colors[states[i]], fill_opacity=0.6,
            ).move_to([(x0[0] + x1[0]) / 2, band_bottom, 0]))

        legend = VGroup(
            *[Square(side_length=0.16, fill_color=c, fill_opacity=0.7, stroke_width=0) for c in colors.values()],
            *[Text(k, font_size=13, color=colors[k], weight=BOLD) for k in colors],
        )
        pairs = [VGroup(legend[i], legend[3 + i]).arrange(RIGHT, buff=0.06) for i in range(3)]
        lg = VGroup(*pairs).arrange(RIGHT, buff=0.35)
        lg.next_to(bands, DOWN, buff=0.12)

        self.play(
            Create(axes), FadeIn(ticks), FadeIn(xl), FadeIn(yl), FadeIn(sc_title),
            run_time=0.7,
        )
        self.play(Create(warn_line), FadeIn(warn_txt), Create(osc_line), FadeIn(osc_txt), run_time=0.6)

        curve = axes.plot_line_graph(
            x_values=tw, y_values=R,
            add_vertex_dots=False, line_color=WHITE, stroke_width=2.6,
        )["line_graph"]
        self.play(Create(curve), run_time=2.4)
        self.play(FadeIn(bands), FadeIn(lg), run_time=0.7)

        badge = VGroup(
            RoundedRectangle(
                corner_radius=0.1, width=2.4, height=0.55,
                stroke_width=2.5, stroke_color=C_NORMAL, fill_color=C_NORMAL, fill_opacity=0.2,
            ),
            Text("NORMAL", font_size=22, color=C_NORMAL, weight=BOLD),
        )
        badge.to_edge(RIGHT, buff=0.35).shift(DOWN * 0.6)

        scan = Line(axes.c2p(0, 0), axes.c2p(0, 1.0), color=WHITE, stroke_width=2, stroke_opacity=0.8)
        self.play(FadeIn(badge, scale=0.8), Create(scan), run_time=0.5)

        i_warn = det.first_time(GROW["res"], det.WARNING)
        i_osc = det.first_time(GROW["res"], det.OSCILLATION)

        def next_badge(state):
            col = colors[state]
            nb = VGroup(
                RoundedRectangle(
                    corner_radius=0.1, width=2.4, height=0.55,
                    stroke_width=2.5, stroke_color=col, fill_color=col, fill_opacity=0.2,
                ).move_to(badge[0]),
                Text(state, font_size=22, color=col, weight=BOLD).move_to(badge[1]),
            )
            return nb

        self.play(scan.animate.move_to(axes.c2p(i_warn, 0.5)), run_time=2.0, rate_func=linear)
        nb = next_badge(det.WARNING)
        self.play(ReplacementTransform(badge, nb))
        badge = nb
        warn_mark = Text(f"WARN @ {i_warn*1e3:.0f} ms", font_size=15, color=C_WARN)
        warn_mark.next_to(axes.c2p(i_warn, 1.05), UP, buff=0.08)
        warn_mark.shift(LEFT * (warn_mark.get_width() / 2 - 0.3))
        self.play(FadeIn(warn_mark), run_time=0.5)

        self.play(scan.animate.move_to(axes.c2p(i_osc, 0.5)), run_time=1.2, rate_func=linear)
        nb = next_badge(det.OSCILLATION)
        self.play(ReplacementTransform(badge, nb))
        badge = nb
        osc_mark = Text(f"OSC @ {i_osc*1e3:.0f} ms", font_size=15, color=C_OSC)
        osc_mark.next_to(axes.c2p(i_osc, 1.05), UP, buff=0.08)
        osc_mark.shift(RIGHT * (osc_mark.get_width() / 2 - 0.3))
        self.play(FadeIn(osc_mark), run_time=0.5)

        self.play(scan.animate.move_to(axes.c2p(0.20, 0.5)), run_time=2.2, rate_func=linear)

        t_warn_ms = CFG.n_warn * HOP_MS
        t_osc_ms = CFG.n_osc * HOP_MS
        t_calm_ms = CFG.n_clear * HOP_MS
        pers = Text(
            f"{CFG.n_warn} windows ≥ {r_warn:.2f} → WARNING (≈{t_warn_ms:.0f} ms)   ·   "
            f"{CFG.n_osc} windows ≥ {r_osc:.2f} → OSCILLATION (≈{t_osc_ms:.0f} ms)   ·   "
            f"{CFG.n_clear} calm → step down",
            font_size=14, color=GREY_C,
        ).move_to([0, -3.45, 0])
        pred = Text(
            "predictive = early warning from trends, not time-to-failure",
            font_size=15, color=C_GOLD,
        ).move_to([0, -3.75, 0])
        self.play(FadeIn(pers), FadeIn(pred), run_time=0.8)
        self.wait(3.0)

        self.play(*[FadeOut(m) for m in self.mobjects])


# =============================================================================
# Scene 5 - closing summary
# =============================================================================
class ClosingScene(Scene):
    def construct(self):
        t1 = Text("Sense → Analyze → Predict (early) → Alert", font_size=34, color=C_GOLD, weight=BOLD)
        t2 = Text(
            "Embedded pipeline prototype for ESP32  ·  data simulated with buck_sim.m + detector.py",
            font_size=18, color=GREY_C,
        )
        t2.next_to(t1, DOWN, buff=0.3)

        chips = VGroup()
        for name, scn, verdict, col in [
            ("stable", STAB, "NORMAL", C_NORMAL),
            ("growing", GROW, "OSCILLATION", C_OSC),
            ("oscillation", OSCI, "OSCILLATION", C_OSC),
        ]:
            box = RoundedRectangle(
                corner_radius=0.12, width=3.6, height=2.0,
                stroke_color=col, stroke_width=2.5, fill_color=col, fill_opacity=0.08,
            )
            title = Text(name, font_size=18, color=WHITE, weight=BOLD).move_to(box.get_top() + DOWN * 0.25)
            ax = Axes(
                x_range=[0, 0.2, 0.1], y_range=[4.3, 5.7, 0.2],
                x_length=3.1, y_length=1.0,
                axis_config={"color": GREY_A, "stroke_width": 1.5, "include_numbers": False},
                tips=False,
            ).move_to(box.get_center() + DOWN * 0.02)
            td, vd = downsample(scn["t"], scn["v"], 700)
            g = ax.plot_line_graph(td, vd, add_vertex_dots=False, line_color=WHITE, stroke_width=1.6)["line_graph"]
            verd = Text(verdict, font_size=16, color=col, weight=BOLD).move_to(box.get_bottom() + UP * 0.22)
            chips.add(VGroup(box, title, ax, g, verd))

        chips.arrange(RIGHT, buff=0.35)
        group = VGroup(t1, chips, t2).arrange(DOWN, buff=0.45)

        self.play(FadeIn(t1, shift=UP * 0.3), run_time=0.9)
        self.play(LaggedStart(*[FadeIn(c, shift=UP * 0.2) for c in chips], lag_ratio=0.2), run_time=1.4)
        self.play(FadeIn(t2), run_time=0.6)
        self.wait(3.5)
        self.play(FadeOut(group), run_time=0.8)