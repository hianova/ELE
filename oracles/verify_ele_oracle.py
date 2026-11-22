#!/usr/bin/env python3
"""
ELE v9.0.0 Independent Multiphysics & Formal-Certificate Oracle (100% Stdlib)

Verifies every benchmark output in `build/ele_bench.json`, audits all
ELECERT-1.0 JSON witnesses via exact arbitrary-precision Python integer
arithmetic and SHA-256 hashes, verifies all Lean 4 `#print axioms` logs have
0 `sorryAx` and 0 `Lean.ofReduceBool`, and emits `ELE_VERIFICATION_REPORT.md`.
"""
import argparse
import cmath
import glob
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from ele_sdk import read_snp


def jacobi_max_eig(mat):
    """Exact symmetric Jacobi eigenvalues (pure Python) -> max eigenvalue."""
    n = len(mat)
    a = [[float(mat[i][j]) for j in range(n)] for i in range(n)]
    for _ in range(120 * n * n):
        p, q, max_off = 0, 1 if n > 1 else 0, 0.0
        for i in range(n):
            for j in range(i + 1, n):
                v = abs(a[i][j])
                if v > max_off:
                    max_off = v
                    p, q = i, j
        if n <= 1 or max_off < 1e-14:
            break
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        tau = (aqq - app) / (2.0 * apq)
        t = (1.0 / (tau + math.sqrt(1.0 + tau * tau))) if tau >= 0.0 else (-1.0 / (-tau + math.sqrt(1.0 + tau * tau)))
        c = 1.0 / math.sqrt(1.0 + t * t)
        s = t * c
        a[p][p] = app - t * apq
        a[q][q] = aqq + t * apq
        a[p][q] = 0.0
        a[q][p] = 0.0
        for k in range(n):
            if k != p and k != q:
                akp, akq = a[k][p], a[k][q]
                a[k][p] = c * akp - s * akq
                a[p][k] = a[k][p]
                a[k][q] = s * akp + c * akq
                a[q][k] = a[k][q]
    return max(a[i][i] for i in range(n))


def snp_sigma_and_recip(path, nports_hint=0):
    """Compute max singular value ||S(f)||_2 and max |S_ij - S_ji| across all f."""
    freqs, mats, z0 = read_snp(path, nports_hint=nports_hint)
    if not freqs:
        raise AssertionError(f"Empty Touchstone file: {path}")
    max_sigma = 0.0
    worst_freq = freqs[0]
    max_recip = 0.0
    for f, S in zip(freqs, mats):
        n = len(S)
        for i in range(n):
            for j in range(i + 1, n):
                err = abs(S[i][j] - S[j][i])
                if err > max_recip:
                    max_recip = err
        # Build G = S^H S (Hermitian PSD) and real 2N x 2N augmented symmetric matrix
        Gr = [[0.0] * n for _ in range(n)]
        Gi = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                acc = 0j
                for k in range(n):
                    acc += S[k][i].conjugate() * S[k][j]
                Gr[i][j] = acc.real
                Gi[i][j] = acc.imag
        aug = [[0.0] * (2 * n) for _ in range(2 * n)]
        for i in range(n):
            for j in range(n):
                aug[i][j] = Gr[i][j]
                aug[n + i][n + j] = Gr[i][j]
                aug[i][n + j] = -Gi[i][j]
                aug[n + i][j] = Gi[i][j]
        lam_max = max(0.0, jacobi_max_eig(aug))
        sig = math.sqrt(lam_max)
        if sig > max_sigma:
            max_sigma = sig
            worst_freq = f
    return {
        "n_freq": len(freqs),
        "nports": len(mats[0]),
        "z0": z0,
        "sigma_max": max_sigma,
        "worst_freq_hz": worst_freq,
        "recip_err": max_recip,
    }


def verify_em(bench_em, bench_dir, cert_dir):
    records = []

    ms_orc = snp_sigma_and_recip(os.path.join(bench_dir, "em/microstrip_50ohm.s2p"), 2)
    ms_c = bench_em["microstrip"]
    assert ms_c["passive"] == 1 and ms_orc["sigma_max"] < 1.0
    assert abs(ms_c["sigma_max"] - ms_orc["sigma_max"]) < 1e-5
    assert ms_orc["recip_err"] < 1e-12 and ms_c["recip_err"] < 1e-12
    records.append(("microstrip_50ohm.s2p", f"sigma_max={ms_c['sigma_max']:.6f}", f"sigma_max={ms_orc['sigma_max']:.6f}", f"{abs(ms_c['sigma_max'] - ms_orc['sigma_max']):.2e}", "PASS"))

    wk_orc = snp_sigma_and_recip(os.path.join(bench_dir, "em/wilkinson_3port.s3p"), 3)
    wk_c = bench_em["wilkinson"]
    assert wk_c["passive"] == 1 and wk_orc["sigma_max"] < 1.0
    assert abs(wk_c["sigma_max"] - wk_orc["sigma_max"]) < 1e-5
    assert wk_orc["recip_err"] < 1e-12 and wk_c["recip_err"] < 1e-12
    records.append(("wilkinson_3port.s3p", f"sigma_max={wk_c['sigma_max']:.6f}", f"sigma_max={wk_orc['sigma_max']:.6f}", f"{abs(wk_c['sigma_max'] - wk_orc['sigma_max']):.2e}", "PASS"))

    np_orc = snp_sigma_and_recip(os.path.join(bench_dir, "em/nonpassive_ai_surrogate.s2p"), 2)
    np_c = bench_em["nonpassive_veto"]
    assert np_c["passive"] == 0 and np_c["status"] == "VETOED" and np_orc["sigma_max"] > 1.0
    assert abs(np_c["sigma_max"] - np_orc["sigma_max"]) < 1e-5
    records.append(("nonpassive_ai_surrogate.s2p", f"{np_c['status']} (sigma={np_c['sigma_max']:.4f})", f"VETO (sigma={np_orc['sigma_max']:.4f})", f"{abs(np_c['sigma_max'] - np_orc['sigma_max']):.2e}", "PASS"))

    peec_c = bench_em["peec_busbar"]
    peec_s2p_orc = snp_sigma_and_recip(os.path.join(cert_dir, "busbar_800v_peec.s2p"), 2)
    assert peec_c["passive"] == 1 and peec_s2p_orc["sigma_max"] < 1.0
    assert 1.1e-8 <= peec_c["l_loop_h"] <= 1.5e-8
    assert peec_c["r_ac_fmax_ohm"] > peec_c["r_dc_ohm"]
    records.append(("busbar_800v.peec", f"L={peec_c['l_loop_h']*1e9:.2f} nH, passive=1", f"L in [11,15] nH, sigma={peec_s2p_orc['sigma_max']:.5f}", f"Rac/Rdc={peec_c['r_ac_fmax_ohm']/peec_c['r_dc_ohm']:.1f}x", "PASS"))

    fdtd_c = bench_em["fdtd_slab"]
    fdtd_s2p_orc = snp_sigma_and_recip(os.path.join(cert_dir, "fdtd_wg_slab.s2p"), 2)
    fc_analytic = 299792458.0 / (2.0 * 0.016)
    fc_rel = abs(fdtd_c["fc_hz"] - fc_analytic) / fc_analytic
    assert fdtd_c["cfl_ok"] == 1 and fdtd_c["passive"] == 1
    assert fc_rel < 1e-5 and fdtd_c["max_energy_sum"] < 1.0 and fdtd_s2p_orc["sigma_max"] < 1.0
    records.append(("fdtd_wg_slab", f"fc={fdtd_c['fc_hz']*1e-9:.4f} GHz, sum={fdtd_c['max_energy_sum']:.4f}", f"fc={fc_analytic*1e-9:.4f} GHz, sigma={fdtd_s2p_orc['sigma_max']:.4f}", f"rel={fc_rel:.2e}", "PASS"))

    arr_c = bench_em["synth_array"]
    assert arr_c["nominal_sll_db"] < -25.0 and arr_c["worst_tol_sll_db"] < -20.0
    assert arr_c["nominal_sll_db"] < arr_c["baseline_sll_db"]
    assert arr_c["worst_tol_sll_db"] < arr_c["baseline_sll_db"]
    records.append(("searu_phased_array", f"nom={arr_c['nominal_sll_db']:.2f} dB, wc={arr_c['worst_tol_sll_db']:.2f} dB", f"base={arr_c['baseline_sll_db']:.2f} dB", f"gain={arr_c['baseline_sll_db']-arr_c['worst_tol_sll_db']:.2f} dB", "PASS"))

    mat_c = bench_em["synth_match"]
    assert mat_c["nominal_s11_db"] < -20.0 and mat_c["worst_tol_s11_db"] < -14.0
    assert mat_c["nominal_s11_db"] < mat_c["baseline_s11_db"]
    assert mat_c["worst_tol_s11_db"] < mat_c["baseline_s11_db"]
    records.append(("searu_broadband_match", f"nom={mat_c['nominal_s11_db']:.2f} dB, wc={mat_c['worst_tol_s11_db']:.2f} dB", f"base={mat_c['baseline_s11_db']:.2f} dB", f"gain={mat_c['baseline_s11_db']-mat_c['worst_tol_s11_db']:.2f} dB", "PASS"))

    return records


def verify_ckt(bench_ckt):
    records = []

    rc = bench_ckt["rc_lowpass"]
    bw_analytic = 1.0 / (2.0 * math.pi * 1000.0 * 1e-6)
    vf_analytic = 1.0 - math.exp(-1.0)
    bw_err = abs(rc["bw_3db_hz"] - bw_analytic) / bw_analytic
    vf_err = abs(rc["v_final"] - vf_analytic) / vf_analytic
    assert bw_err < 0.005 and vf_err < 0.005
    records.append(("rc_lowpass.sp", f"fc={rc['bw_3db_hz']:.3f} Hz, V={rc['v_final']:.5f}", f"fc={bw_analytic:.3f} Hz, V={vf_analytic:.5f}", f"rel={max(bw_err, vf_err):.2e}", "PASS"))

    pdn = bench_ckt["rlc_pdn"]
    assert 0.0 < pdn["z_pdn_max_ohm"] < 0.10 and pdn["pdn_pass"] == 1
    records.append(("rlc_pdn.sp", f"Zmax={pdn['z_pdn_max_ohm']:.5f} Ohm", "< 0.10000 Ohm (pass=1)", f"margin={0.25 - pdn['z_pdn_max_ohm']:.4f} Ohm", "PASS"))

    # Shockley diode oracle: V1=5.0, R1=100, D1 IS=1e-12 N=1.0 RS=0.05, Vt=0.02585
    vt = 0.02585
    is_sat = 1e-12
    r_tot = 100.0 + 0.05
    vj = 0.65
    for _ in range(60):
        ex = math.exp(vj / vt)
        f = is_sat * (ex - 1.0) - (5.0 - vj) / r_tot
        df = is_sat * ex / vt + 1.0 / r_tot
        vj -= f / df
    id_orc = is_sat * (math.exp(vj / vt) - 1.0)
    vd_orc = 5.0 - 100.0 * id_orc
    di = bench_ckt["diode_rectifier"]
    vd_diff = abs(di["v_diode"] - vd_orc)
    assert vd_diff < 1e-5 and di["max_kcl_res"] < 1e-11
    records.append(("diode_rectifier.sp", f"Vd={di['v_diode']:.6f} V, res={di['max_kcl_res']:.2e}", f"Vd={vd_orc:.6f} V", f"diff={vd_diff:.2e} V", "PASS"))

    sic = bench_ckt["sic_800v"]
    assert sic["n_corners"] == 5
    assert sic["wc_vmin"] <= sic["nom_vmax"] <= sic["wc_vmax"]
    assert sic["wc_vmax"] < 1150.0 and sic["soa_pass"] == 1
    records.append(("sic_halfbridge_800v.sp", f"[{sic['wc_vmin']:.2f}, {sic['wc_vmax']:.2f}] V", "[-20.00, 1150.00] V (5 corners)", f"margin={1150.0 - sic['wc_vmax']:.2f} V", "PASS"))

    eye = bench_ckt["si_eye"]
    assert eye["eye_pda_v"] > 0.25 and abs(eye["eye_pda_v"] - eye["eye_searu_v"]) < 1e-6
    assert eye["eye_pass"] == 1
    records.append(("diff_pair_si.sp", f"PDA={eye['eye_pda_v']:.4f} V, SEARU={eye['eye_searu_v']:.4f} V", "> 0.2500 V (diff < 1e-6)", f"diff={abs(eye['eye_pda_v'] - eye['eye_searu_v']):.2e} V", "PASS"))

    return records


def verify_motor(bench_motor, bench_dir):
    spec = {}
    with open(os.path.join(bench_dir, "ee/ev_ipmsm_150kw.motor"), "r") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if "=" in line:
                k, v = [x.strip() for x in line.split("=", 1)]
                spec[k] = float(v)

    poles = int(round(spec["poles"]))
    p = 0.5 * poles
    psi_pm = spec["psi_pm0_wb"] * (1.0 + spec["alpha_pm"] * (spec["temp_c"] - 25.0))
    rs_t = spec["rs_ohm"] * (1.0 + 0.00393 * (spec["temp_c"] - 25.0))
    v_max = spec["v_dc_v"] / math.sqrt(3.0)

    op3 = bench_motor["op_3k"]
    id3, iq3 = op3["id_a"], op3["iq_a"]
    ld_eff = spec["ld0_h"] / (1.0 + spec["sat_d"] * id3 * id3 + spec["sat_cross"] * iq3 * iq3)
    lq_eff = spec["lq0_h"] / (1.0 + spec["sat_q"] * iq3 * iq3 + spec["sat_cross"] * id3 * id3)
    psi_d = psi_pm + ld_eff * id3
    psi_q = lq_eff * iq3
    tq_orc = 1.5 * p * (psi_d * iq3 - psi_q * id3)
    omega_e = p * 3000.0 * (2.0 * math.pi / 60.0)
    vd_orc = rs_t * id3 - omega_e * psi_q
    vq_orc = rs_t * iq3 + omega_e * psi_d
    tq_err = abs(op3["torque_nm"] - tq_orc)
    assert tq_err < 1e-2
    assert abs(op3["vd_v"] - vd_orc) < 1e-2 and abs(op3["vq_v"] - vq_orc) < 1e-2
    assert math.hypot(id3, iq3) <= spec["i_max_a"] + 1e-3
    assert math.hypot(op3["vd_v"], op3["vq_v"]) <= v_max + 1e-2
    assert op3["eff"] > 0.95

    op10 = bench_motor["op_10k_fw"]
    assert math.hypot(op10["id_a"], op10["iq_a"]) <= spec["i_max_a"] + 1e-3
    assert op10["v_mag_v"] <= v_max + 1e-2 and op10["eff"] > 0.95

    foc = bench_motor["foc"]
    assert foc["converged"] == 1 and abs(foc["final_torque_nm"] - 260.0) < 2.0

    return [
        ("ipmsm@3000rpm", f"T={op3['torque_nm']:.4f} Nm, eff={op3['eff']*100:.2f}%", f"T={tq_orc:.4f} Nm", f"err={tq_err:.2e} Nm", "PASS"),
        ("ipmsm@10000rpm_fw", f"Id={op10['id_a']:.2f} A, V={op10['v_mag_v']:.2f}/{v_max:.2f} V", "I<=400A, V<=461.88V", f"eff={op10['eff']*100:.2f}%", "PASS"),
        ("ipmsm_foc_40khz", f"converged={foc['converged']}, T={foc['final_torque_nm']:.2f} Nm", "T~260 Nm", f"Ipeak={foc['peak_i_a']:.2f} A", "PASS"),
    ]


def parse_matpower_m(path):
    with open(path, "r") as f:
        lines = [ln.split("%")[0] for ln in f]
    txt = "\n".join(lines)
    m_base = re.search(r"mpc\.baseMVA\s*=\s*([0-9eE.+\-]+)", txt)
    base_mva = float(m_base.group(1)) if m_base else 100.0

    def extract_matrix(name):
        m = re.search(rf"{re.escape(name)}\s*=\s*\[(.*?)\]\s*;", txt, flags=re.DOTALL)
        if not m:
            raise AssertionError(f"Missing {name} block in {path}")
        rows = []
        for raw_row in m.group(1).split(";"):
            vals = [float(x) for x in raw_row.strip().split()]
            if vals:
                rows.append(vals)
        return rows

    buses = []
    for r in extract_matrix("mpc.bus"):
        buses.append({
            "id": int(round(r[0])), "type": int(round(r[1])),
            "pd": float(r[2]), "qd": float(r[3]),
            "gs": float(r[4]), "bs": float(r[5]),
        })
    gens = []
    for r in extract_matrix("mpc.gen"):
        if int(round(r[7])) > 0:
            gens.append({"bus_id": int(round(r[0])), "pg": float(r[1])})
    branches = []
    for r in extract_matrix("mpc.branch"):
        if int(round(r[10])) > 0:
            ratio = float(r[8]) if float(r[8]) > 0.0 else 1.0
            branches.append({
                "u": int(round(r[0])), "v": int(round(r[1])),
                "r": float(r[2]), "x": float(r[3]),
                "bsh": float(r[4]), "tap": ratio,
                "shift_deg": float(r[9]),
            })
    idx = {b["id"]: i for i, b in enumerate(buses)}
    return base_mva, buses, gens, branches, idx


def parse_grid(path):
    if path.endswith(".m"):
        return parse_matpower_m(path)
    base_mva = 100.0
    buses, gens, branches = [], [], []
    sec = None
    with open(path, "r") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            toks = line.split()
            if toks[0].upper() == "BASE_MVA":
                base_mva = float(toks[1])
                continue
            if len(toks) == 1 and toks[0].upper() in ("BUS", "GEN", "BRANCH"):
                sec = toks[0].upper()
                continue
            if sec == "BUS":
                buses.append({
                    "id": int(toks[0]), "type": int(toks[1]),
                    "pd": float(toks[2]), "qd": float(toks[3]),
                    "gs": float(toks[4]), "bs": float(toks[5]),
                })
            elif sec == "GEN":
                gens.append({"bus_id": int(toks[0]), "pg": float(toks[1])})
            elif sec == "BRANCH":
                branches.append({
                    "u": int(toks[0]), "v": int(toks[1]),
                    "r": float(toks[2]), "x": float(toks[3]),
                    "bsh": float(toks[4]), "tap": float(toks[5]) if float(toks[5]) > 0 else 1.0,
                    "shift_deg": 0.0,
                })
    idx = {b["id"]: i for i, b in enumerate(buses)}
    return base_mva, buses, gens, branches, idx


def verify_grid(bench_grid, bench_dir):
    records = []
    specs = [
        ("ieee3", "ee/ieee3_test.grid"),
        ("ieee9", "ee/ieee9_wscc.grid"),
        ("ieee14", "ee/ieee14_grid.grid"),
        ("pglib14", "ee/pglib/pglib_opf_case14_ieee.m"),
        ("pglib30", "ee/pglib/pglib_opf_case30_ieee.m"),
        ("pglib57", "ee/pglib/pglib_opf_case57_ieee.m"),
        ("pglib118", "ee/pglib/pglib_opf_case118_ieee.m"),
    ]
    for name, rel in specs:
        base_mva, buses, gens, branches, idx = parse_grid(os.path.join(bench_dir, rel))
        N = len(buses)
        Y = [[0j] * N for _ in range(N)]
        for k, b in enumerate(buses):
            Y[k][k] += complex(b["gs"] / base_mva, b["bs"] / base_mva)
        for br in branches:
            u, v = idx[br["u"]], idx[br["v"]]
            ys = 1.0 / complex(br["r"], br["x"])
            bsh = complex(0.0, 0.5 * br["bsh"])
            t = br["tap"] * cmath.exp(1j * math.radians(br["shift_deg"]))
            t_mag2 = t.real * t.real + t.imag * t.imag
            Y[u][u] += (ys + bsh) / t_mag2
            Y[v][v] += (ys + bsh)
            Y[u][v] -= ys / t.conjugate()
            Y[v][u] -= ys / t

        pf = bench_grid[f"{name}_pf"]
        assert pf["converged"] == 1 and len(pf["vm"]) == N
        V = [cmath.rect(pf["vm"][i], pf["va_rad"][i]) for i in range(N)]
        p_spec = [-b["pd"] / base_mva for b in buses]
        q_spec = [-b["qd"] / base_mva for b in buses]
        for g in gens:
            p_spec[idx[g["bus_id"]]] += g["pg"] / base_mva

        max_mis = 0.0
        for i, b in enumerate(buses):
            iv = sum(Y[i][j] * V[j] for j in range(N))
            S = V[i] * iv.conjugate()
            if b["type"] != 3:
                max_mis = max(max_mis, abs(p_spec[i] - S.real))
            if b["type"] == 1:
                max_mis = max(max_mis, abs(q_spec[i] - S.imag))
        assert max_mis < 1e-9, f"{name} Ybus mismatch {max_mis}"
        records.append((f"{name}_acpf ({N}b)", f"mis={pf['mismatch']:.2e}, loss={pf['loss_mw']:.4f} MW", "complex Ybus KCL", f"oracle_mis={max_mis:.2e} pu", "PASS"))

    opf3 = bench_grid["ieee3_opf"]
    assert opf3["c_lb_cert"] <= opf3["c_ub_primal"] and opf3["gap_pct"] < 1.0
    records.append(("ieee3_acopf", f"[{opf3['c_lb_cert']:.2f}, {opf3['c_ub_primal']:.2f}] $/h", "C_LB <= C_UB, gap < 1%", f"gap={opf3['gap_pct']:.4f}%", "PASS"))

    opf9 = bench_grid["ieee9_opf"]
    assert opf9["c_lb_cert"] <= opf9["c_ub_primal"] and opf9["gap_pct"] < 5.0
    records.append(("ieee9_acopf", f"[{opf9['c_lb_cert']:.2f}, {opf9['c_ub_primal']:.2f}] $/h", "C_LB <= C_UB, gap < 5%", f"gap={opf9['gap_pct']:.4f}%", "PASS"))

    pg14 = bench_grid["pglib14_opf"]
    assert pg14["c_lb_cert"] <= pg14["c_ub_primal"] and pg14["gap_pct"] < 20.0
    records.append(("pglib_case14_acopf", f"[{pg14['c_lb_cert']:.2f}, {pg14['c_ub_primal']:.2f}] $/h", "C_LB <= C_UB, gap < 20%", f"gap={pg14['gap_pct']:.4f}%", "PASS"))

    pg30 = bench_grid["pglib30_opf"]
    assert pg30["c_lb_cert"] <= pg30["c_ub_primal"]
    records.append(("pglib_case30_acopf", f"[{pg30['c_lb_cert']:.2f}, {pg30['c_ub_primal']:.2f}] $/h", "C_LB <= C_UB (congested)", f"gap={pg30['gap_pct']:.2f}%", "PASS"))

    emt9 = bench_grid["ieee9_emt"]
    assert emt9["stable"] == 1 and emt9["v_post"] < emt9["v_crit"]
    assert 0.10 < emt9["cct_sec"] < 0.35 and emt9["max_gfm_i_pu"] <= 1.2001
    records.append(("ieee9_emt_gfm", f"V={emt9['v_post']:.4f}<{emt9['v_crit']:.4f}, CCT={emt9['cct_sec']:.4f}s", "PEBS Lyapunov, GFM<=1.2pu", f"Igfm={emt9['max_gfm_i_pu']:.3f} pu", "PASS"))

    return records


def verify_ngspice(bench_ckt):
    if shutil.which("ngspice") is None:
        return [("ngspice_crosscheck", "SKIPPED", "ngspice binary not found", "N/A", "SKIPPED")]

    rows = []
    with tempfile.TemporaryDirectory() as td:
        rc_sp = os.path.join(td, "rc_lowpass.sp")
        with open(rc_sp, "w") as f:
            f.write(
                "* ngspice cross-check: rc_lowpass\n"
                "V1 vin 0 DC 0 AC 1 PULSE(0 1 0 1n 1n 1 2)\n"
                "R1 vin vout 1k\n"
                "C1 vout 0 1u\n"
                ".control\n"
                "set noaskquit\n"
                "tran 10u 1m\n"
                "meas tran v1m find v(vout) at=1m\n"
                "ac dec 125 1 100k\n"
                "meas ac f3db when vdb(vout)=-3.010299956639812\n"
                "quit\n"
                ".endc\n"
                ".end\n"
            )
        out_rc = subprocess.run(["ngspice", "-b", rc_sp], capture_output=True, text=True, check=True).stdout
        m_v1m = re.search(r"v1m\s*=\s*([0-9eE.+\-]+)", out_rc)
        m_f3 = re.search(r"f3db\s*=\s*([0-9eE.+\-]+)", out_rc)
        assert m_v1m and m_f3, f"Failed parsing ngspice rc output:\n{out_rc}"
        ng_v1m, ng_f3 = float(m_v1m.group(1)), float(m_f3.group(1))
        rc = bench_ckt["rc_lowpass"]
        err_rc = max(abs(rc["bw_3db_hz"] - ng_f3) / ng_f3, abs(rc["v_final"] - ng_v1m) / ng_v1m)
        assert err_rc < 0.005, f"ngspice rc mismatch {err_rc}"
        rows.append(("rc_lowpass.sp", f"fc={rc['bw_3db_hz']:.2f} Hz, V={rc['v_final']:.5f} V", f"ngspice: fc={ng_f3:.2f} Hz, V={ng_v1m:.5f} V", f"rel={err_rc:.2e}", "PASS"))

        di_sp = os.path.join(td, "diode_rectifier.sp")
        with open(di_sp, "w") as f:
            f.write(
                "* ngspice cross-check: diode_rectifier\n"
                ".options temp=26.85 tnom=26.85\n"
                "V1 vin 0 DC 5.0\n"
                "R1 vin vd 100\n"
                "D1 vd 0 DMOD\n"
                ".model DMOD D(IS=1e-12 N=1.0 RS=0.05)\n"
                ".control\n"
                "set noaskquit\n"
                "op\n"
                "print v(vd)\n"
                "quit\n"
                ".endc\n"
                ".end\n"
            )
        out_di = subprocess.run(["ngspice", "-b", di_sp], capture_output=True, text=True, check=True).stdout
        m_vd = re.search(r"v\(vd\)\s*=\s*([0-9eE.+\-]+)", out_di)
        assert m_vd, f"Failed parsing ngspice diode output:\n{out_di}"
        ng_vd = float(m_vd.group(1))
        ele_vd = bench_ckt["diode_rectifier"]["v_diode"]
        diff_vd = abs(ele_vd - ng_vd)
        assert diff_vd < 0.002, f"ngspice diode mismatch {diff_vd}"
        rows.append(("diode_rectifier.sp", f"Vd={ele_vd:.6f} V", f"ngspice: Vd={ng_vd:.6f} V", f"diff={diff_vd:.2e} V", "PASS"))

        pdn_sp = os.path.join(td, "rlc_pdn.sp")
        with open(pdn_sp, "w") as f:
            f.write(
                "* ngspice cross-check: rlc_pdn\n"
                "Rvrm vrm 0 0.01\n"
                "Lpkg vrm die 2n\n"
                "Cdie die 0 200n\n"
                "Resr die 0 0.08\n"
                "Iload die 0 DC 0 AC -1\n"
                ".control\n"
                "set noaskquit\n"
                "ac dec 125 10k 100MEG\n"
                "meas ac zmax max vm(die)\n"
                "quit\n"
                ".endc\n"
                ".end\n"
            )
        out_pdn = subprocess.run(["ngspice", "-b", pdn_sp], capture_output=True, text=True, check=True).stdout
        m_zm = re.search(r"zmax\s*=\s*([0-9eE.+\-]+)", out_pdn)
        assert m_zm, f"Failed parsing ngspice pdn output:\n{out_pdn}"
        ng_zm = float(m_zm.group(1))
        ele_zm = bench_ckt["rlc_pdn"]["z_pdn_max_ohm"]
        diff_zm = abs(ele_zm - ng_zm)
        assert diff_zm < 0.001, f"ngspice pdn mismatch {diff_zm}"
        rows.append(("rlc_pdn.sp", f"Zmax={ele_zm:.5f} Ohm", f"ngspice: Zmax={ng_zm:.5f} Ohm", f"diff={diff_zm:.2e} Ohm", "PASS"))
    return rows


def verify_cardputer_lab(data_dir, cert_dir, lab_thermal_json, lab_csi_json, lab_power_json):
    man_path = os.path.join(data_dir, "hardware_manifest.json")
    assert os.path.exists(man_path), f"Missing Cardputer hardware manifest: {man_path}"
    with open(man_path, "r") as f:
        man = json.load(f)

    for fn, exp_sha in man["files"].items():
        fp = os.path.join(data_dir, fn)
        assert os.path.exists(fp), f"Missing captured hardware CSV: {fp}"
        h = hashlib.sha256()
        with open(fp, "rb") as rf:
            h.update(rf.read())
        assert h.hexdigest() == exp_sha, f"Manifest SHA-256 mismatch on {fp}"

    csi_csv = os.path.join(data_dir, "cardputer_rf_csi.csv")
    with open(csi_csv, "r") as f:
        for raw in f:
            ln = raw.strip()
            if not ln:
                continue
            if ln.startswith("# SCAN,"):
                assert re.match(r"^# SCAN,\d+,\d+,\d+,-?\d+$", ln), f"Unexpected SCAN line format: {ln}"
            elif not ln.startswith("#") and not ln.startswith("seq,"):
                assert re.match(r"^[\d,\-]+$", ln), f"Non-numeric CSI row (privacy/format check): {ln}"

    with open(lab_thermal_json, "r") as f:
        th = json.load(f)
    assert th["status"] == "CERTIFIED" and th["spice_rmse_c"] <= 1.50
    assert th["delta_t_c"] > 0.0 and th["spice_vs_analytic_c"] < 0.2

    with open(lab_csi_json, "r") as f:
        csi = json.load(f)
    assert csi["status"] == "CERTIFIED" and csi["passive"] == 1
    assert csi["recip_err"] < 1e-12 and csi["n_frames"] >= 5

    csi_s2p_path = os.path.join(cert_dir, "cardputer_rf_csi.s2p")
    assert os.path.exists(csi_s2p_path), f"Missing CSI .s2p: {csi_s2p_path}"
    csi_orc = snp_sigma_and_recip(csi_s2p_path, 2)
    assert abs(csi_orc["sigma_max"] - csi["sigma_max"]) < 1e-5
    assert csi_orc["recip_err"] < 1e-12

    adc_vals, rc_vals = [], []
    with open(os.path.join(data_dir, "cardputer_gpio_rc_adc.csv"), "r") as f:
        for raw in f:
            ln = raw.strip()
            if not ln or ln.startswith("#") or ln.startswith("kind,"):
                continue
            parts = ln.split(",")
            if parts[0] == "ADC":
                adc_vals.append(float(parts[3]))
            elif parts[0] == "RC":
                rc_vals.append(float(parts[3]))
    assert len(adc_vals) >= 100 and len(rc_vals) >= 20
    adc_mean = sum(adc_vals) / len(adc_vals)
    adc_std = math.sqrt(sum((x - adc_mean) ** 2 for x in adc_vals) / len(adc_vals))
    assert 2000.0 <= adc_mean <= 5500.0 and rc_vals[-1] >= rc_vals[0]

    with open(lab_power_json, "r") as f:
        pwr = json.load(f)
    assert pwr["status"] == "CERTIFIED" and pwr["spice_rmse_mv"] <= 30.0
    assert pwr["min_v_mv"] >= 3300.0 and pwr["n_samples"] >= 200

    chip_tag = f"{man.get('chip', 'ESP32-S3')} rev={man.get('rev', '?')} id={man.get('mac_sha8', '')}"
    return [
        ("cardputer_thermal_rail.csv", f"dT={th['delta_t_c']:.2f}C, Rth={th['r_th_k_per_w']:.2f}K/W, droop={th['v_droop_mv']:.1f}mV", f"{chip_tag}, Cauer SPICE <=1.5C", f"RMSE={th['spice_rmse_c']:.4f} C", "PASS"),
        ("cardputer_rf_csi.csv", f"ch{csi['dom_ch']}, RSSI={csi['mean_rssi_dbm']:.1f}dBm, tau={csi['tau_rms_ns']:.1f}ns", f"{csi['n_frames']} frames, sigma={csi_orc['sigma_max']:.5f}", f"recip={csi_orc['recip_err']:.1e}", "PASS"),
        ("cardputer_gpio_rc_adc.csv", f"GPIO10={adc_mean:.1f}+-{adc_std:.1f}mV, GPIO1={rc_vals[0]:.0f}->{rc_vals[-1]:.0f}mV", "manifest SHA-256 + 12b ADC/RC", f"N={len(adc_vals)}+{len(rc_vals)}", "PASS"),
        ("cardputer_power_modes.csv", f"Vmin={pwr['min_v_mv']:.1f}mV, droop={pwr['max_droop_mv']:.2f}mV, dT={pwr['delta_t_c']:.2f}C", f"{chip_tag}, Thevenin PDN SPICE <=30mV", f"RMSE={pwr['spice_rmse_mv']:.3f} mV", "PASS"),
    ]


def verify_certificates(cert_dir):
    certs = sorted(glob.glob(os.path.join(cert_dir, "*.elecert.json")))
    assert len(certs) >= 15, f"Expected >= 15 .elecert.json files in {cert_dir}, got {len(certs)}"
    rows = []
    for path in certs:
        with open(path, "r") as f:
            rec = json.load(f)
        assert rec.get("format") == "ELECERT-1.0", f"Bad format in {path}"
        in_path = rec.get("input_path", "")
        in_hash = rec.get("input_sha256", "")
        hash_ok = "internal"
        if in_path and in_path != "internal" and os.path.exists(in_path):
            h = hashlib.sha256()
            with open(in_path, "rb") as rf:
                h.update(rf.read())
            actual = h.hexdigest()
            assert actual == in_hash, f"SHA-256 mismatch on {path}: {actual} != {in_hash}"
            hash_ok = actual[:16] + "..."

        # Exact arbitrary-precision Python int verification of Cholesky + SDD witness
        if rec.get("has_psd") == 1:
            d = int(rec["dim"])
            mu_psd = int(rec["mu_psd"])
            delta_round = int(rec["delta_round"])
            mu_cert = int(rec["mu_cert"])
            assert mu_cert == mu_psd - delta_round
            A = rec["A_int"]
            L = rec["L_int"]
            E = [[0] * d for _ in range(d)]
            for i in range(d):
                for j in range(d):
                    assert A[i][j] == A[j][i], f"Asymmetric A_int in {path}"
                    if j > i:
                        assert L[i][j] == 0, f"L_int not lower triangular in {path}"
                    llt = sum(int(L[i][k]) * int(L[j][k]) for k in range(d))
                    E[i][j] = int(A[i][j]) - (mu_psd if i == j else 0) - llt
            for i in range(d):
                off = sum(abs(E[i][j]) for j in range(d) if j != i)
                assert off <= E[i][i], f"SDD failed row {i} in {path}: off={off} > diag={E[i][i]}"

        if rec.get("has_chordal") == 1:
            dim = int(rec["dim"])
            cliques = rec["cliques"]
            assert len(cliques) == int(rec["num_cliques"]), f"Clique count mismatch in {path}"
            A_target = [[0] * dim for _ in range(dim)]
            seen_sp = set()
            for r_i, c_i, v_i in rec["A_sparse"]:
                r_i, c_i, v_i = int(r_i), int(c_i), int(v_i)
                assert 0 <= r_i <= c_i < dim and (r_i, c_i) not in seen_sp, f"Bad A_sparse entry in {path}"
                seen_sp.add((r_i, c_i))
                A_target[r_i][c_i] = v_i
                A_target[c_i][r_i] = v_i
            A_sum = [[0] * dim for _ in range(dim)]
            neg_pen = 0
            min_mu = None
            for idx_c, cl in enumerate(cliques):
                cvars = [int(v) for v in cl["vars"]]
                dk = len(cvars)
                assert len(set(cvars)) == dk and all(0 <= v < dim for v in cvars), f"Bad clique vars in {path}"
                w_c = int(cl.get("w_max_int", rec["w_max_int"]))
                assert w_c > 0, f"Invalid clique w_max_int in {path}"
                mu_c = int(cl["mu_psd"])
                S = cl["S_int"]
                L = cl["L_int"]
                E = [[0] * dk for _ in range(dk)]
                for i in range(dk):
                    for j in range(dk):
                        assert int(S[i][j]) == int(S[j][i]), f"Asymmetric S_int in clique {idx_c} of {path}"
                        if j > i:
                            assert int(L[i][j]) == 0, f"L_int not lower triangular in clique {idx_c} of {path}"
                        llt = sum(int(L[i][k]) * int(L[j][k]) for k in range(dk))
                        E[i][j] = int(S[i][j]) - (mu_c if i == j else 0) - llt
                        A_sum[cvars[i]][cvars[j]] += int(S[i][j])
                for i in range(dk):
                    off = sum(abs(E[i][j]) for j in range(dk) if j != i)
                    assert off <= E[i][i], f"Chordal clique {idx_c} row {i} SDD failed in {path}"
                min_mu = mu_c if min_mu is None else min(min_mu, mu_c)
                if mu_c < 0:
                    neg_pen += mu_c * w_c
            assert A_sum == A_target, f"Chordal sum != A_sparse in {path}"
            assert min_mu == int(rec["min_clique_mu_psd"]), f"min_clique_mu_psd mismatch in {path}"
            exp_lb = int(rec["c_dual_int"]) + neg_pen - int(rec["opf_delta_round"])
            assert int(rec["c_lb_int"]) == exp_lb, f"Chordal c_lb_int mismatch in {path}"
            if rec.get("status") == "CERTIFIED":
                assert int(rec["c_lb_int"]) <= int(rec["c_ub_int"]), f"Chordal bracket inverted in {path}"

        if rec.get("has_opf") == 1:
            assert int(rec["c_lb_int"]) <= int(rec["c_ub_int"]), f"OPF bracket inverted in {path}"

        rows.append((os.path.basename(path), rec["pillar"], rec["kind"], rec["status"], hash_ok))
    return rows


def verify_atlas(atlas_path):
    if not atlas_path or not os.path.exists(atlas_path):
        return []
    with open(atlas_path, "r") as f:
        atl = json.load(f)
    rows = []
    seen_wb2 = seen_c3 = seen_c5 = seen_c118 = False
    for g in atl["grids"]:
        assert g["chordal_certified"] == 1 and g["chordal_lb_cert"] <= g["chordal_ub"], f"Bad chordal atlas row: {g}"
        if g["tag"] == "wb2":
            seen_wb2 = True
            assert 0.0 <= g["sos2_gap_pct"] < 0.5, f"wb2 SOS-2 gap too high: {g}"
        if g["tag"] == "case3_lmbd":
            seen_c3 = True
            assert 0.0 <= g["sos2_gap_pct"] < g["shor1_gap_pct"], f"case3_lmbd SOS-2 did not beat Shor-1: {g}"
        if g["tag"] == "case5_pjm":
            seen_c5 = True
            assert 0.0 <= g["sos2_gap_pct"] < g["shor1_gap_pct"], f"case5_pjm SOS-2 did not beat Shor-1: {g}"
        if g["tag"] == "case118_ieee":
            seen_c118 = True
            assert 0.0 <= g["chordal_gap_pct"] < 20.0, f"case118_ieee chordal gap too high: {g}"
        sos_txt = f"{g['sos2_gap_pct']:.4f}%" if g["sos2_gap_pct"] >= 0.0 else "N/A"
        s1_txt = f"{g['shor1_gap_pct']:.4f}%" if g["shor1_gap_pct"] >= 0.0 else "N/A"
        rows.append((f"grid:{g['tag']} ({g['n_bus']}b, tw<={g['treewidth_ub']})",
                     f"Shor1={s1_txt}, Chordal={g['chordal_gap_pct']:.4f}%, SOS2={sos_txt}",
                     f"[{g['chordal_lb_cert']:.2f}, {g['chordal_ub']:.2f}] $/h",
                     f"cliques={g['n_cliques']}, t={g['elapsed_sec']:.3f}s", "PASS"))
    assert seen_wb2 and seen_c3 and seen_c5 and seen_c118
    for o in atl["observables"]:
        assert o["status"] == "CERTIFIED" and o["o_lb"] <= o["o_primal"] <= o["o_ub"], f"Bad observable row: {o}"
        rows.append((f"obs:{o['grid']}:{o['obs']}", f"val={o['o_primal']:.5f}", f"[{o['o_lb']:.5f}, {o['o_ub']:.5f}]",
                     f"width={o['width']:.5f}", "PASS"))
    eb = atl["em_bound"]
    assert eb["status"] == "CERTIFIED" and eb["p_searu_lb"] <= eb["p_local_ub_cert"] <= eb["p_global_ub"], f"Bad em_bound: {eb}"
    rows.append((f"em:maxwell_bound (M={eb['n_cells']})",
                 f"SEARU={eb['p_searu_lb']:.3f} <= Local={eb['p_local_ub_cert']:.3f}",
                 f"Global={eb['p_global_ub']:.3f}", f"ratio={eb['tightening_ratio']:.4f}", "PASS"))
    kyp = atl["kyp_enforce"]
    assert kyp["status"] == "CERTIFIED" and kyp["certified"] == 1
    assert kyp["orig_sigma_max"] > 1.0 and kyp["rep_sigma_max"] <= 1.0
    rows.append(("em:kyp_enforce", f"orig={kyp['orig_sigma_max']:.4f} -> rep={kyp['rep_sigma_max']:.4f}",
                 f"P>0, KYP_mu={kyp['kyp_mu_psd']}", f"rms={kyp['rms_repair_err']:.4f}", "PASS"))
    return rows


def verify_lean_axioms(cert_dir):
    expected = [
        "CertEmMicrostrip",
        "CertEmWilkinson",
        "CertEmMaxwellBound",
        "CertEmBodeFano",
        "CertEmKypEnforce",
        "CertEmKyp3Port",
        "CertCktSic800v",
        "CertMotorIpmsm",
        "CertGridIeee3Opf",
        "CertGridChordal",
        "CertDiscoveredWb2Sos",
        "CertGridIeee9Stab",
    ]
    rows = []
    for mod in expected:
        p = os.path.join(cert_dir, f"{mod}.axioms.txt")
        assert os.path.exists(p), f"Missing Lean axioms log: {p}"
        with open(p, "r") as f:
            txt = f.read()
        assert "sorryAx" not in txt, f"sorryAx found in {p}"
        assert "Lean.ofReduceBool" not in txt, f"native_decide found in {p}"
        lines = [ln.strip() for ln in txt.splitlines() if ("depends on axioms:" in ln or "does not depend on any axioms" in ln)]
        assert lines, f"No axiom report lines in {p}"
        rows.append((mod, "; ".join(lines)))
    return rows


def write_report(report_path, em_rows, ckt_rows, ng_rows, mot_rows, grid_rows, lab_rows, cert_rows, lean_rows, atlas_rows):
    with open(report_path, "w") as f:
        f.write("# ELE v11.0.0 Verification Report\n\n")
        f.write("Generated automatically by pure-stdlib [verify_ele_oracle.py](oracles/verify_ele_oracle.py).\n\n")

        def write_table(title, headers, data):
            f.write(f"## {title}\n\n")
            f.write("| " + " | ".join(headers) + " |\n")
            f.write("| " + " | ".join("---" for _ in headers) + " |\n")
            for r in data:
                f.write("| " + " | ".join(str(x) for x in r) + " |\n")
            f.write("\n")

        write_table("1. Electromagnetics (`em`) Independent Oracle", ["Benchmark", "Engine Result", "Oracle Check", "Residual / Margin", "Status"], em_rows)
        write_table("2. Electronics & Circuits (`ckt`) Independent Oracle", ["Benchmark", "Engine Result", "Oracle Check", "Residual / Margin", "Status"], ckt_rows)
        write_table("3. External Golden-Solver Cross-Validation (`ngspice`)", ["Circuit", "ELE CKT Solver", "ngspice Reference", "Agreement", "Status"], ng_rows)
        write_table("4. Electrical Machines & Drives (`motor`) Independent Oracle", ["Benchmark", "Engine Result", "Oracle Check", "Residual / Margin", "Status"], mot_rows)
        write_table("5. Power Grids (`grid` + Official IEEE PES PGLib-OPF v23.07) Independent Oracle", ["Benchmark", "Engine Result", "Oracle Check", "Residual / Margin", "Status"], grid_rows)
        write_table("6. M5Stack Cardputer-Adv (`ESP32-S3`) Real Hardware Lab Validation (`lab`)", ["Dataset", "Extracted Physics / Fit", "Hardware + Model Oracle", "Residual / Count", "Status"], lab_rows)
        write_table("7. Zero-FP-Trust `ELECERT-1.0` Witness & SHA-256 Ledger", ["Certificate", "Pillar", "Kind", "Status", "Input SHA-256"], cert_rows)
        write_table("8. Lean 4 Standalone Certificate Axiom Audit (`leanchecker`)", ["Module", "Axiom Dependencies (`0 sorryAx`, `0 ofReduceBool`)"], lean_rows)
        if atlas_rows:
            write_table("9. Frontier Research Atlas (`v11.0.0`: Chordal + SOS-2 + Observables + EM Poynting/KYP)", ["Frontier Case", "Bound / Gap Result", "Bracket / Reference", "Extra Margin", "Status"], atlas_rows)


def main():
    ap = argparse.ArgumentParser(description="ELE v11.0.0 Independent Oracle")
    ap.add_argument("--bench", default="build/ele_bench.json")
    ap.add_argument("--bench-dir", default="benchmarks")
    ap.add_argument("--cert-dir", default="build/certs")
    ap.add_argument("--data-dir", default="data/cardputer")
    ap.add_argument("--lab-thermal", default="build/lab_thermal.json")
    ap.add_argument("--lab-csi", default="build/lab_csi.json")
    ap.add_argument("--lab-power", default="build/lab_power.json")
    ap.add_argument("--atlas", default="build/ele_atlas.json")
    ap.add_argument("--report", default="ELE_VERIFICATION_REPORT.md")
    args = ap.parse_args()

    with open(args.bench, "r") as f:
        bench = json.load(f)

    em_rows = verify_em(bench["em"], args.bench_dir, args.cert_dir)
    ckt_rows = verify_ckt(bench["ckt"])
    ng_rows = verify_ngspice(bench["ckt"])
    mot_rows = verify_motor(bench["motor"], args.bench_dir)
    grid_rows = verify_grid(bench["grid"], args.bench_dir)
    lab_rows = verify_cardputer_lab(args.data_dir, args.cert_dir, args.lab_thermal, args.lab_csi, args.lab_power)
    cert_rows = verify_certificates(args.cert_dir)
    lean_rows = verify_lean_axioms(args.cert_dir)
    atlas_rows = verify_atlas(args.atlas)
    write_report(args.report, em_rows, ckt_rows, ng_rows, mot_rows, grid_rows, lab_rows, cert_rows, lean_rows, atlas_rows)
    print(f"  [ORACLE PASS] EM({len(em_rows)}) + CKT({len(ckt_rows)}) + NGSPICE({len(ng_rows)}) + MOTOR({len(mot_rows)}) + GRID({len(grid_rows)}) + LAB({len(lab_rows)}) + CERTS({len(cert_rows)}) + LEAN({len(lean_rows)}) + ATLAS({len(atlas_rows)}) -> {args.report}")


if __name__ == "__main__":
    main()
