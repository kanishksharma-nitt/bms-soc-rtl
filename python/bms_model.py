#!/usr/bin/env python3
"""Bit-exact golden model for the BMS state-of-charge estimation engine.

Fixed-point contract (identical to the RTL):
  pack current  I : Q7.9  signed (LSB = 1/512 A), +ve = discharge
  cell voltage  V : Q3.13 (LSB = 1/8192 V)
  temperature   T : Q7.1  signed (LSB = 0.5 degC)
  SoC accumulator : Q1.31 unsigned (1.0 = 2^31), output SoC = top 16 bits
  OCV table       : 33 entries, Q3.13, monotonic; linear interpolation
  inverse lookup  : 5-step binary search + fractional divide (mirrored as
                    integer //, the RTL uses a non-restoring divider)

Per-sample update order (the RTL strobe does the same combinational chain):
  1. sanity flags (V/T range, stuck current: same reading for STUCK_N
     samples while |I| > STUCK_I_TH)
  2. rest detector (|I| < REST_TH for REST_N samples)
  3. Coulomb count: soc -= (I*K_DSOC)>>8, charge efficiency on I<0
  4. if rested and not frozen: soc += (ALPHA*(soc_ocv_prev - soc))>>15
  5. soc_ocv_prev = inverse OCV lookup of this sample's V (used next sample)

Scenarios: constant discharge, pulse discharge, charge with rests,
current-sensor offset (shows OCV correction pulling SoC back; residual
error quantified), fault injection (V out of range, hot T, stuck sensor).

Stdlib only, deterministic. Emits rtl/ocv_lut.mem and test/bms_vectors.mem.
"""

import math
import random
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SEED = 20260704

# parameters (mirrored in RTL)
K_DSOC = 11930          # (0.01s * 2^31 / (Q_nom=900As * 512)) * 2^8
EFF = 32440             # 0.99 charge efficiency, Q1.15
ALPHA = 655             # 0.02 OCV blend gain, Q1.15
REST_TH = 154           # 0.3 A, above the worst-case sensor offset
REST_N = 300            # 3 s at 100 Hz
# stuck detection only above the rest band: below it a stuck reading is
# indistinguishable from genuine rest (and the OCV correction path is the
# defense against drift there anyway)
STUCK_I_TH = 154
STUCK_N = 200           # 2 s
V_MIN, V_MAX = 20480, 35226   # 2.5 .. 4.3 V in Q3.13
T_MIN, T_MAX = -40, 120       # -20 .. +60 C in Q7.1
FS = 100.0
FULL = 1 << 31

# Thevenin plant (floats); tau = R1*C1 = 3 s, so rests longer than ~10 s
# relax close to true OCV (the estimator only corrects while rested)
R0, R1, C1 = 0.05, 0.03, 100.0
Q_NOM_AS = 900.0


def ocv_f(s):
    return 3.0 + 0.75 * s + 0.35 * s ** 4 + 0.15 * (1 - math.exp(-15 * s))


OCV_LUT = [int(ocv_f(i / 32.0) * 8192 + 0.5) for i in range(33)]
assert all(OCV_LUT[i] < OCV_LUT[i + 1] for i in range(32)), "LUT not monotonic"


def ocv_interp(soc):
    """Forward OCV(SoC), Q1.31 -> Q3.13 (bit-exact)."""
    s = min(soc, FULL - 1)
    idx = (s >> 26) & 0x1F
    frac = (s >> 16) & 0x3FF
    lo, hi = OCV_LUT[idx], OCV_LUT[idx + 1]
    return lo + (((hi - lo) * frac) >> 10)


def ocv_inverse(v):
    """SoC(OCV) via binary search + interpolation divide (bit-exact)."""
    if v <= OCV_LUT[0]:
        return 0
    if v >= OCV_LUT[32]:
        return FULL
    i = 0
    for b in (16, 8, 4, 2, 1):
        if i + b <= 31 and OCV_LUT[i + b] <= v:
            i += b
    lo, hi = OCV_LUT[i], OCV_LUT[i + 1]
    frac = ((v - lo) << 10) // (hi - lo)      # RTL: non-restoring divider
    return (i << 26) + (frac << 16)


class Estimator:
    def __init__(self):
        self.soc = FULL              # start at 100%
        self.rest_cnt = 0
        self.stuck_run = 0
        self.last_i = None
        self.soc_ocv = 0
        self.ocv_valid = False

    def step(self, iq, vq, tq):
        # 1. sanity
        v_oor = vq < V_MIN or vq > V_MAX
        t_oor = tq < T_MIN or tq > T_MAX
        if self.last_i is not None and iq == self.last_i and \
                abs(iq) > STUCK_I_TH:
            self.stuck_run = min(self.stuck_run + 1, STUCK_N)
        else:
            self.stuck_run = 0
        self.last_i = iq
        i_stuck = self.stuck_run >= STUCK_N
        freeze = v_oor or t_oor or i_stuck

        # 2. rest detector
        if abs(iq) < REST_TH:
            self.rest_cnt = min(self.rest_cnt + 1, REST_N)
        else:
            self.rest_cnt = 0
        rested = self.rest_cnt >= REST_N

        # 3. Coulomb count
        delta = (iq * K_DSOC) >> 8
        if iq < 0:
            delta = (delta * EFF) >> 15
        self.soc = max(0, min(FULL, self.soc - delta))

        # 4. OCV correction
        if rested and self.ocv_valid and not freeze:
            corr = (ALPHA * (self.soc_ocv - self.soc)) >> 15
            self.soc = max(0, min(FULL, self.soc + corr))

        # 5. inverse lookup for next sample
        self.soc_ocv = ocv_inverse(vq)
        self.ocv_valid = True

        flags = (rested << 3) | (i_stuck << 2) | (t_oor << 1) | int(v_oor)
        return self.soc >> 16, flags


class Plant:
    def __init__(self):
        self.soc = 1.0
        self.vrc = 0.0

    def step(self, i_amp):
        self.soc = max(0.0, min(1.0, self.soc - i_amp / FS / Q_NOM_AS))
        self.vrc += (i_amp * R1 - self.vrc) / (R1 * C1) * (1.0 / FS)
        return ocv_f(self.soc) - i_amp * R0 - self.vrc


def build_scenario(kind, rng):
    """Return list of (i_amp_true, i_offset, v_force, t_c) per sample."""
    seg = []

    def add(seconds, amps, offset=0.0, vf=None, tc=25.0, dither=True):
        for _ in range(int(seconds * FS)):
            a = amps + (rng.randint(-3, 3) / 512.0 if dither and amps != 0
                        else 0.0)
            seg.append((a, offset, vf, tc))

    if kind == 0:                       # constant discharge + rest
        add(30, 2.0)
        add(15, 0.0)
    elif kind == 1:                     # pulse discharge (urban-ish)
        for _ in range(3):
            add(5, 2.0)
            add(5, 0.5)
            add(5, 0.0)
        add(10, 0.0)
    elif kind == 2:                     # charge with rest periods
        add(20, -1.5)
        add(10, 0.0)
        add(10, -1.5)
        add(10, 0.0)
    elif kind == 3:                     # sensor offset, rests pull SoC back
        # +0.2 A offset on the measured current throughout (REST_TH is
        # above it, so rest detection still works)
        for _ in range(3):
            add(10, 2.0, offset=0.2)
            add(15, 0.0, offset=0.2)
    else:                               # faults
        add(5, 1.5)
        add(3, 1.5, vf=36864 / 8192.0)  # V forced to 4.5 V -> v_oor
        add(3, 1.5, tc=70.0)            # hot -> t_oor
        add(4, 1.5, dither=False)       # frozen reading -> i_stuck
        add(8, 0.0)
    return seg


def main():
    rng = random.Random(SEED)

    lut_path = os.path.join(HERE, "..", "rtl", "ocv_lut.mem")
    os.makedirs(os.path.dirname(lut_path), exist_ok=True)
    with open(lut_path, "w") as f:
        for v in OCV_LUT:
            f.write(f"{v & 0xFFFF:04x}\n")

    names = ["cc_discharge", "pulse", "charge_rest", "sensor_offset",
             "faults"]
    words = [5]
    for kind in range(5):
        plant, est = Plant(), Estimator()
        est_nc = Estimator()            # coulomb-only comparison (offset run)
        stim, exp = [], []
        for i_true, i_off, v_force, t_c in build_scenario(kind, rng):
            v_true = plant.step(i_true)
            iq = max(-32768, min(32767, int((i_true + i_off) * 512)))
            vq = max(0, min(65535, int((v_force if v_force else v_true)
                                       * 8192 + 0.5)))
            tq = int(t_c * 2)
            soc15, flags = est.step(iq, vq, tq)
            if kind == 3:
                est_nc.rest_cnt = 0     # suppress correction in the shadow
                est_nc.step(iq, vq, tq)
            stim.append((iq, vq, tq))
            exp.append((soc15, flags))

        true15 = int(plant.soc * 32768)
        err_pct = abs(exp[-1][0] - true15) / 32768 * 100
        print(f"  [{names[kind]:14s}] {len(stim):5d} samples, final SoC "
              f"est {exp[-1][0]/327.68:6.2f}% vs true {true15/327.68:6.2f}% "
              f"(err {err_pct:.2f}%)")
        if kind == 3:
            nc_err = abs((est_nc.soc >> 16) - true15) / 32768 * 100
            print(f"                   coulomb-only drift with offset: "
                  f"{nc_err:.2f}% vs corrected {err_pct:.2f}%")
            assert err_pct < nc_err, "OCV correction must beat coulomb-only"
        if kind in (0, 1, 2):
            assert err_pct < 2.0, f"estimator error {err_pct:.2f}% too big"
        if kind == 4:
            assert any(e[1] & 1 for e in exp), "v_oor never flagged"
            assert any(e[1] & 2 for e in exp), "t_oor never flagged"
            assert any(e[1] & 4 for e in exp), "i_stuck never flagged"

        words.append(len(stim))
        for iq, vq, tq in stim:
            words += [iq & 0xFFFF, vq & 0xFFFF, tq & 0xFFFF]
        for soc15, flags in exp:
            words.append((flags << 16) | soc15)

    out = os.path.join(HERE, "..", "test", "bms_vectors.mem")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        for w in words:
            f.write(f"{w & 0xFFFFFFFF:08x}\n")
    print(f"wrote {len(words)} words -> test/bms_vectors.mem")


if __name__ == "__main__":
    main()
