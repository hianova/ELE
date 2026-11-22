#!/usr/bin/env python3
"""
Deterministic Industrial Benchmark Generator for ELE v9.0.0 (Pure Python Stdlib)
Generates canonical EM (.s2p, .s3p, .peec), Electronics (.sp), Motor (.motor),
and Power Grid (.grid) benchmarks across all three EEE pillars.
"""
import argparse
import cmath
import math
import os


def write_microstrip_s2p(path):
    """Lossy 50-ohm FR4 microstrip line (35 mm, dispersive skin + dielectric loss)."""
    z0 = 50.0
    l_m = 0.035
    er_eff = 3.25
    with open(path, "w") as f:
        f.write("! ELE v9.0.0 Benchmark: 50-ohm Lossy FR4 Microstrip Line (35 mm)\n")
        f.write("# GHZ S RI R 50.0\n")
        for m in range(31):
            f_ghz = 1.0 + 0.3 * m
            freq = f_ghz * 1e9
            omega = 2.0 * math.pi * freq
            # Skin + dielectric attenuation (strictly passive across 1..10 GHz)
            alpha = 0.35 + 0.45 * math.sqrt(f_ghz) + 0.12 * f_ghz
            beta = (omega / 299792458.0) * math.sqrt(er_eff)
            gamma = complex(alpha, beta)
            zc = complex(50.0 + 0.3 * f_ghz, -0.6 / math.sqrt(f_ghz))
            gl = gamma * l_m
            sh = cmath.sinh(gl)
            ch = cmath.cosh(gl)
            a = ch
            b = zc * sh
            c = sh / zc
            d = ch
            den = a + b / z0 + c * z0 + d
            s11 = (a + b / z0 - c * z0 - d) / den
            s21 = 2.0 / den
            f.write(
                f"{f_ghz:.4f} {s11.real:.10e} {s11.imag:.10e} "
                f"{s21.real:.10e} {s21.imag:.10e} "
                f"{s21.real:.10e} {s21.imag:.10e} "
                f"{s11.real:.10e} {s11.imag:.10e}\n"
            )


def _inv3x3_complex(M):
    """Invert a 3x3 complex matrix using Gauss-Jordan elimination (stdlib only)."""
    A = [[M[i][j] for j in range(3)] + [1.0 + 0j if i == j else 0j for j in range(3)] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(A[r][col]))
        A[col], A[piv] = A[piv], A[col]
        div = A[col][col]
        for j in range(6):
            A[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            f = A[r][col]
            for j in range(6):
                A[r][j] -= f * A[col][j]
    return [row[3:] for row in A]


def write_wilkinson_s3p(path):
    """Exact physical lossy 3-port Wilkinson power divider via nodal Y -> S (2..4 GHz)."""
    z0 = 50.0
    zc = z0 * math.sqrt(2.0)
    yc = 1.0 / zc
    g_iso = 1.0 / (2.0 * z0)
    g_pad = 1.0 / 2500.0  # slight dielectric/substrate shunt loss -> strictly passive
    f0 = 3.0e9
    alpha_l = 0.035       # arm attenuation (Np) -> ~0.3 dB realistic FR4/Rogers loss
    with open(path, "w") as f:
        f.write("! ELE v9.0.0 Benchmark: Physical 3-Port Wilkinson Power Divider (2..4 GHz)\n")
        f.write("# GHZ S RI R 50.0\n")
        for m in range(21):
            f_ghz = 2.0 + 0.1 * m
            freq = f_ghz * 1e9
            gl = complex(alpha_l * math.sqrt(freq / f0), math.pi * 0.5 * (freq / f0))
            sh = cmath.sinh(gl)
            ch = cmath.cosh(gl)
            y11 = yc * (ch / sh)
            y12 = -yc / sh
            Y = [
                [2.0 * y11 + g_pad, y12, y12],
                [y12, y11 + g_iso + g_pad, -g_iso],
                [y12, -g_iso, y11 + g_iso + g_pad],
            ]
            I_plus = [[(1.0 if i == j else 0.0) + z0 * Y[i][j] for j in range(3)] for i in range(3)]
            I_minus = [[(1.0 if i == j else 0.0) - z0 * Y[i][j] for j in range(3)] for i in range(3)]
            invP = _inv3x3_complex(I_plus)
            S = [[sum(I_minus[i][k] * invP[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
            f.write(f"{f_ghz:.4f}")
            for i in range(3):
                if i > 0:
                    f.write("      ")
                for j in range(3):
                    f.write(f" {S[i][j].real:.10e} {S[i][j].imag:.10e}")
                f.write("\n")


def write_nonpassive_s2p(path):
    """Adversarial AI/surrogate macromodel violating passivity at 5 GHz (sigma_max = 1.035)."""
    with open(path, "w") as f:
        f.write("! ELE v9.0.0 Benchmark: Non-Passive AI Surrogate Hallucination (MUST BE VETOED)\n")
        f.write("# GHZ S RI R 50.0\n")
        pts = [
            (1.0, (0.04, -0.01), (0.94, -0.10)),
            (2.0, (0.05, -0.02), (0.92, -0.15)),
            (3.0, (0.06, -0.03), (0.90, -0.18)),
            (4.0, (0.08, -0.02), (0.88, -0.20)),
            (5.0, (0.18,  0.00), (0.855, 0.00)),  # |S11 + S21| = 1.035 > 1!
            (6.0, (0.09, -0.04), (0.85, -0.22)),
        ]
        for f_ghz, s11, s21 in pts:
            f.write(
                f"{f_ghz:.4f} {s11[0]:.8e} {s11[1]:.8e} "
                f"{s21[0]:.8e} {s21[1]:.8e} "
                f"{s21[0]:.8e} {s21[1]:.8e} "
                f"{s11[0]:.8e} {s11[1]:.8e}\n"
            )


def write_nonpassive_wilkinson_s3p(path):
    """3-port Wilkinson AI surrogate violating passivity & reciprocity at 3.5 GHz (sigma_max ~ 1.048)."""
    z0 = 50.0
    zc = z0 * math.sqrt(2.0)
    yc = 1.0 / zc
    g_iso = 1.0 / (2.0 * z0)
    g_pad = 1.0 / 2500.0
    f0 = 3.0e9
    alpha_l = 0.035
    freqs = [1.0, 2.0, 3.0, 3.5, 4.0, 5.0]
    with open(path, "w") as f:
        f.write("! ELE v11.0.0 Benchmark: Non-Passive + Non-Reciprocal 3-Port AI Surrogate (MUST BE VETOED)\n")
        f.write("# GHZ S RI R 50.0\n")
        for f_ghz in freqs:
            freq = f_ghz * 1e9
            gl = complex(alpha_l * math.sqrt(freq / f0), math.pi * 0.5 * (freq / f0))
            sh = cmath.sinh(gl)
            ch = cmath.cosh(gl)
            y11 = yc * (ch / sh)
            y12 = -yc / sh
            Y = [
                [2.0 * y11 + g_pad, y12, y12],
                [y12, y11 + g_iso + g_pad, -g_iso],
                [y12, -g_iso, y11 + g_iso + g_pad],
            ]
            I_plus = [[(1.0 if i == j else 0.0) + z0 * Y[i][j] for j in range(3)] for i in range(3)]
            I_minus = [[(1.0 if i == j else 0.0) - z0 * Y[i][j] for j in range(3)] for i in range(3)]
            invP = _inv3x3_complex(I_plus)
            S = [[sum(I_minus[i][k] * invP[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
            if abs(f_ghz - 3.5) < 1e-6:
                for i in range(3):
                    for j in range(3):
                        S[i][j] *= 1.118
                S[1][0] += complex(-0.014, 0.004)  # inject non-reciprocity hallucination
            f.write(f"{f_ghz:.4f}")
            for i in range(3):
                if i > 0:
                    f.write("      ")
                for j in range(3):
                    f.write(f" {S[i][j].real:.10e} {S[i][j].imag:.10e}")
                f.write("\n")


def write_busbar_peec(path):
    with open(path, "w") as f:
        f.write("# ELE v9.0.0 Benchmark: 800V AI-Rack / EV Laminated Copper Busbar\n")
        f.write("length_m = 0.20\n")
        f.write("width_m = 0.04\n")
        f.write("thickness_m = 0.002\n")
        f.write("sep_m = 0.001\n")
        f.write("er = 4.2\n")
        f.write("sigma_s_m = 5.8e7\n")
        f.write("f_start_hz = 1e6\n")
        f.write("f_stop_hz = 1e8\n")
        f.write("n_freq = 41\n")
        f.write("z0_ohm = 50.0\n")


def write_ckt_benchmarks(ckt_dir):
    with open(os.path.join(ckt_dir, "rc_lowpass.sp"), "w") as f:
        f.write(
            "* ELE v9.0.0 Benchmark: First-Order RC Lowpass (fc = 159.155 Hz)\n"
            "V1 vin 0 DC 0 AC 1 PULSE(0 1 0 1n 1n 1 2)\n"
            "R1 vin vout 1k\n"
            "C1 vout 0 1u\n"
            ".AC DEC 125 1 100k VOUT=vout VIN=V1\n"
            ".TRAN 10u 1m PROBE=vout\n"
            ".END\n"
        )

    with open(os.path.join(ckt_dir, "rlc_pdn.sp"), "w") as f:
        f.write(
            "* ELE v9.0.0 Benchmark: Package-Die Power Distribution Network (Z_max < 0.25 Ohm)\n"
            "Rvrm vrm 0 0.01\n"
            "Lpkg vrm die 2n\n"
            "Cdie die 0 200n\n"
            "Resr die 0 0.08\n"
            "Iload die 0 DC 0 AC 1\n"
            ".AC DEC 125 10k 100MEG VOUT=die\n"
            ".PDN NODE=die ISRC=Iload ZMAX=0.25\n"
            ".END\n"
        )

    with open(os.path.join(ckt_dir, "diode_rectifier.sp"), "w") as f:
        f.write(
            "* ELE v9.0.0 Benchmark: Nonlinear Shockley Diode Clamp\n"
            "V1 vin 0 DC 5.0 AC 0.1\n"
            "R1 vin vd 100\n"
            "D1 vd 0 IS=1e-12 N=1.0 RS=0.05\n"
            ".OP\n"
            ".TRAN 1u 50u PROBE=vd\n"
            ".END\n"
        )

    with open(os.path.join(ckt_dir, "sic_halfbridge_800v.sp"), "w") as f:
        f.write(
            "* ELE v9.0.0 Benchmark: 800V SiC MOSFET Switching Stage + Snubber + PVT Corners\n"
            "Vdc vdc 0 DC 800\n"
            "R_bus vdc vbus 0.08 TOL=5\n"
            "L_bus vbus sw 15n TOL=10\n"
            "R_load vbus sw 16\n"
            "C_oss sw 0 1.2n TOL=10\n"
            "R_snub sw snub 4.7 TOL=5\n"
            "C_snub snub 0 4.7n TOL=10\n"
            "Vgate gate 0 PULSE(0 15 20n 8n 8n 80n 200n)\n"
            "M_ls sw gate 0 VTH=3.5 RON=12 ROFF=1e6\n"
            "D_fw 0 sw IS=1e-11 N=1.2 RS=0.05\n"
            ".TRAN 1n 220n PROBE=sw\n"
            ".SOA NODE=sw VMIN=-20 VMAX=1150\n"
            ".END\n"
        )

    with open(os.path.join(ckt_dir, "diff_pair_si.sp"), "w") as f:
        f.write(
            "* ELE v9.0.0 Benchmark: Coupled 2-Line RLGC PCB Trace (Crosstalk & Eye)\n"
            "Vvic vd 0 DC 0\n"
            "Vagg ad 0 DC 0\n"
            "Rsv vd vin 50\n"
            "Rsa ad ain 50\n"
            "Rlv vout 0 50\n"
            "Rla aout 0 50\n"
            "T1 vin vout ain aout 0 R=4.0 L=280n C=112p LM=33.6n CM=16p LEN=0.10 NSEC=8\n"
            ".TRAN 10p 4n PROBE=vout\n"
            ".END\n"
        )


def write_ee_benchmarks(ee_dir):
    with open(os.path.join(ee_dir, "ev_ipmsm_150kw.motor"), "w") as f:
        f.write(
            "# ELE v9.0.0 Benchmark: 800V 150kW EV Traction IPMSM (Cross-Saturated + 85C)\n"
            "poles = 8\n"
            "rs_ohm = 0.018\n"
            "ld0_h = 0.00018\n"
            "lq0_h = 0.00042\n"
            "psi_pm0_wb = 0.145\n"
            "sat_d = 1.2e-6\n"
            "sat_q = 2.8e-6\n"
            "sat_cross = 0.8e-6\n"
            "alpha_pm = -0.0012\n"
            "i_max_a = 400.0\n"
            "v_dc_v = 800.0\n"
            "j_kgm2 = 0.045\n"
            "b_viscous = 0.015\n"
            "k_core = 0.0012\n"
            "temp_c = 85.0\n"
            "rpm_max = 14000.0\n"
        )

    with open(os.path.join(ee_dir, "ieee3_test.grid"), "w") as f:
        f.write(
            "# ELE v9.0.0 Benchmark: 3-Bus Ring Grid (AC-PF + N-1 + Shor AC-OPF)\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 1.02 0.0 0.95 1.05\n"
            "2 2 50.0 15.0 0.0 0.0 1.02 0.0 0.95 1.05\n"
            "3 1 180.0 45.0 0.0 0.0 1.00 0.0 0.95 1.05\n"
            "GEN\n"
            "1 80.0 0.0 10.0 200.0 -100.0 100.0 1.02 0.11 5.0 0.0 6.0 0.08 1.5 0\n"
            "2 100.0 0.0 10.0 200.0 -100.0 100.0 1.02 0.085 1.2 0.0 4.5 0.10 1.5 1\n"
            "BRANCH\n"
            "1 2 0.015 0.08 0.02 1.0 220.0\n"
            "2 3 0.020 0.10 0.02 1.0 220.0\n"
            "1 3 0.025 0.12 0.02 1.0 220.0\n"
        )

    with open(os.path.join(ee_dir, "ieee9_wscc.grid"), "w") as f:
        f.write(
            "# ELE v9.0.0 Benchmark: WSCC 9-Bus 3-Machine/GFM System (MATPOWER case9)\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 1.04 0.0 0.90 1.10\n"
            "2 2 0.0 0.0 0.0 0.0 1.025 0.0 0.90 1.10\n"
            "3 2 0.0 0.0 0.0 0.0 1.025 0.0 0.90 1.10\n"
            "4 1 0.0 0.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "5 1 90.0 30.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "6 1 0.0 0.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "7 1 100.0 35.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "8 1 0.0 0.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "9 1 125.0 50.0 0.0 0.0 1.00 0.0 0.90 1.10\n"
            "GEN\n"
            "1 72.0 0.0 10.0 250.0 -300.0 300.0 1.04 0.11 5.0 150.0 23.64 0.0608 2.0 0\n"
            "2 163.0 0.0 10.0 300.0 -300.0 300.0 1.025 0.085 1.2 600.0 6.40 0.1198 1.5 0\n"
            "3 85.0 0.0 10.0 270.0 -300.0 300.0 1.025 0.1225 1.0 335.0 3.01 0.1813 1.2 1\n"
            "BRANCH\n"
            "1 4 0.0001 0.0576 0.000 1.0 300.0\n"
            "4 5 0.0170 0.0920 0.158 1.0 300.0\n"
            "5 6 0.0390 0.1700 0.358 1.0 300.0\n"
            "3 6 0.0001 0.0586 0.000 1.0 300.0\n"
            "6 7 0.0119 0.1008 0.209 1.0 300.0\n"
            "7 8 0.0085 0.0720 0.149 1.0 300.0\n"
            "8 2 0.0001 0.0625 0.000 1.0 300.0\n"
            "8 9 0.0320 0.1610 0.306 1.0 300.0\n"
            "9 4 0.0100 0.0850 0.176 1.0 300.0\n"
        )

    # Authentic IEEE 14-Bus Test System (MATPOWER case14)
    with open(os.path.join(ee_dir, "ieee14_grid.grid"), "w") as f:
        f.write(
            "# ELE v9.0.0 Benchmark: IEEE 14-Bus Power Grid (MATPOWER case14)\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 1.06 0.0 0.94 1.06\n"
            "2 2 21.7 12.7 0.0 0.0 1.045 -4.98 0.94 1.06\n"
            "3 2 94.2 19.0 0.0 0.0 1.010 -12.72 0.94 1.06\n"
            "4 1 47.8 -3.9 0.0 0.0 1.019 -10.33 0.94 1.06\n"
            "5 1 7.6 1.6 0.0 0.0 1.020 -8.78 0.94 1.06\n"
            "6 2 11.2 7.5 0.0 0.0 1.070 -14.22 0.94 1.08\n"
            "7 1 0.0 0.0 0.0 0.0 1.062 -13.37 0.94 1.08\n"
            "8 2 0.0 0.0 0.0 0.0 1.090 -13.36 0.94 1.10\n"
            "9 1 29.5 16.6 0.0 19.0 1.056 -14.94 0.94 1.08\n"
            "10 1 9.0 5.8 0.0 0.0 1.051 -15.10 0.94 1.08\n"
            "11 1 3.5 1.8 0.0 0.0 1.057 -14.79 0.94 1.08\n"
            "12 1 6.1 1.6 0.0 0.0 1.055 -15.07 0.94 1.08\n"
            "13 1 13.5 5.8 0.0 0.0 1.050 -15.16 0.94 1.08\n"
            "14 1 14.9 5.0 0.0 0.0 1.036 -16.04 0.94 1.08\n"
            "GEN\n"
            "1 232.4 -16.9 0.0 332.4 -99.0 99.0 1.060 0.043 20.0 0.0 5.148 0.08 1.5 0\n"
            "2 40.0 42.4 0.0 140.0 -40.0 50.0 1.045 0.25 20.0 0.0 6.54 0.12 1.5 0\n"
            "3 0.0 23.4 0.0 100.0 0.0 40.0 1.010 0.01 40.0 0.0 6.54 0.12 1.5 1\n"
            "6 0.0 12.2 0.0 100.0 -6.0 24.0 1.070 0.01 40.0 0.0 5.06 0.15 1.5 0\n"
            "8 0.0 17.4 0.0 100.0 -6.0 24.0 1.090 0.01 40.0 0.0 5.06 0.15 1.5 0\n"
            "BRANCH\n"
            "1 2 0.01938 0.05917 0.0528 1.0 250.0\n"
            "1 5 0.05403 0.22304 0.0492 1.0 150.0\n"
            "2 3 0.04699 0.19797 0.0438 1.0 150.0\n"
            "2 4 0.05811 0.17632 0.0340 1.0 150.0\n"
            "2 5 0.05695 0.17388 0.0346 1.0 150.0\n"
            "3 4 0.06701 0.17103 0.0128 1.0 150.0\n"
            "4 5 0.01335 0.04211 0.0000 1.0 150.0\n"
            "4 7 0.00000 0.20912 0.0000 0.978 150.0\n"
            "4 9 0.00000 0.55618 0.0000 0.969 150.0\n"
            "5 6 0.00000 0.25202 0.0000 0.932 150.0\n"
            "6 11 0.09498 0.19890 0.0000 1.0 150.0\n"
            "6 12 0.12291 0.25581 0.0000 1.0 150.0\n"
            "6 13 0.06615 0.13027 0.0000 1.0 150.0\n"
            "7 8 0.00000 0.17615 0.0000 1.0 150.0\n"
            "7 9 0.00000 0.11001 0.0000 1.0 150.0\n"
            "9 10 0.03181 0.08450 0.0000 1.0 150.0\n"
            "9 14 0.12711 0.27038 0.0000 1.0 150.0\n"
            "10 11 0.08205 0.19207 0.0000 1.0 150.0\n"
            "12 13 0.22092 0.19988 0.0000 1.0 150.0\n"
            "13 14 0.17093 0.34802 0.0000 1.0 150.0\n"
        )

    # Bukhsh et al. (2013) WB2: classic 2-bus Shor SDP non-zero duality gap benchmark
    with open(os.path.join(ee_dir, "wb2.grid"), "w") as f:
        f.write(
            "# ELE v10.0.0 Benchmark: Bukhsh WB2 2-Bus Non-Zero Shor Gap Network\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 0.95 0.0 0.95 1.05\n"
            "2 1 350.0 -350.0 0.0 0.0 0.976 -73.0 0.95 1.022\n"
            "GEN\n"
            "1 452.8 -100.0 0.0 600.0 -400.0 400.0 0.95 0.0 2.0 0.0 5.0 0.10 1.5 0\n"
            "BRANCH\n"
            "1 2 0.04 0.20 0.0 1.0 999.0 -85.0 85.0\n"
        )

    # Bukhsh WB3: 3-bus cycle with congested branch 1-2 & angle bounds
    with open(os.path.join(ee_dir, "wb3.grid"), "w") as f:
        f.write(
            "# ELE v10.0.0 Benchmark: 3-Bus Congested Cycle (WB3)\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 1.02 0.0 0.95 1.05\n"
            "2 2 110.0 40.0 0.0 0.0 1.02 0.0 0.95 1.05\n"
            "3 1 110.0 40.0 0.0 0.0 1.00 0.0 0.95 1.05\n"
            "GEN\n"
            "1 110.0 0.0 0.0 300.0 -150.0 150.0 1.02 0.05 5.0 0.0 6.0 0.08 1.5 0\n"
            "2 115.0 0.0 0.0 300.0 -150.0 150.0 1.02 0.02 1.2 0.0 4.5 0.10 1.5 1\n"
            "BRANCH\n"
            "1 2 0.015 0.08 0.02 1.0 50.0 -30.0 30.0\n"
            "2 3 0.020 0.10 0.02 1.0 200.0 -30.0 30.0\n"
            "1 3 0.025 0.12 0.02 1.0 200.0 -30.0 30.0\n"
        )

    # Bukhsh WB5: 5-bus meshed network with tight load voltage envelope
    with open(os.path.join(ee_dir, "wb5.grid"), "w") as f:
        f.write(
            "# ELE v10.0.0 Benchmark: 5-Bus Meshed Non-Zero Shor Gap Network (WB5)\n"
            "BASE_MVA 100.0\n"
            "BUS\n"
            "1 3 0.0 0.0 0.0 0.0 1.05 0.0 0.95 1.05\n"
            "2 2 0.0 0.0 0.0 0.0 1.05 0.0 0.95 1.05\n"
            "3 1 160.0 60.0 0.0 0.0 1.00 0.0 0.95 1.05\n"
            "4 1 180.0 -120.0 0.0 0.0 1.00 0.0 0.95 1.015\n"
            "5 2 80.0 20.0 0.0 0.0 1.04 0.0 0.95 1.05\n"
            "GEN\n"
            "1 180.0 0.0 0.0 400.0 -250.0 250.0 1.05 0.01 4.0 0.0 5.0 0.10 1.5 0\n"
            "2 150.0 0.0 0.0 300.0 -200.0 200.0 1.05 0.02 1.8 0.0 5.0 0.10 1.5 0\n"
            "5 100.0 0.0 0.0 250.0 -150.0 150.0 1.04 0.03 2.5 0.0 5.0 0.10 1.5 0\n"
            "BRANCH\n"
            "1 2 0.01 0.06 0.02 1.0 250.0 -30.0 30.0\n"
            "1 3 0.02 0.10 0.02 1.0 220.0 -30.0 30.0\n"
            "2 4 0.02 0.10 0.00 1.0 180.0 -30.0 30.0\n"
            "3 4 0.02 0.12 0.02 1.0 150.0 -30.0 30.0\n"
            "2 5 0.015 0.08 0.02 1.0 200.0 -30.0 30.0\n"
            "4 5 0.025 0.11 0.00 1.0 180.0 -30.0 30.0\n"
        )


def main():
    ap = argparse.ArgumentParser(description="Generate deterministic EEE benchmarks for ELE v9.0.0")
    ap.add_argument("--out-dir", default="benchmarks")
    args = ap.parse_args()

    em_dir = os.path.join(args.out_dir, "em")
    ckt_dir = os.path.join(args.out_dir, "ckt")
    ee_dir = os.path.join(args.out_dir, "ee")
    for d in (em_dir, ckt_dir, ee_dir):
        os.makedirs(d, exist_ok=True)

    write_microstrip_s2p(os.path.join(em_dir, "microstrip_50ohm.s2p"))
    write_wilkinson_s3p(os.path.join(em_dir, "wilkinson_3port.s3p"))
    write_nonpassive_s2p(os.path.join(em_dir, "nonpassive_ai_surrogate.s2p"))
    write_nonpassive_wilkinson_s3p(os.path.join(em_dir, "nonpassive_wilkinson_3port.s3p"))
    write_busbar_peec(os.path.join(em_dir, "busbar_800v.peec"))
    write_ckt_benchmarks(ckt_dir)
    write_ee_benchmarks(ee_dir)
    print(f"Generated 17 deterministic EEE benchmarks in {args.out_dir}/")


if __name__ == "__main__":
    main()
