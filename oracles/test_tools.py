#!/usr/bin/env python3
"""
ELE v11.0.0 End-to-End CLI, Python SDK (`EleEngine` + `EleFastSession`), and Tamper-Rejection Test Suite
(100% Python Standard Library)
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from ele_sdk import EleEngine, EleFastSession, read_snp


def run_cmd(cmd):
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise AssertionError(f"Command failed ({res.returncode}): {' '.join(cmd)}\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}")
    return res


def run_expect_fail(cmd, desc):
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0:
        raise AssertionError(f"Expected non-zero exit for tamper test [{desc}], but got 0!\nCmd: {' '.join(cmd)}")


def test_cli_subcommands(cli_path, tmpdir):
    v = run_cmd([cli_path, "--version"])
    assert "ELE 11.0.0" in v.stdout
    h = run_cmd([cli_path, "--help"])
    for tok in ("field-to-circuit", "cert-verify", "chordal", "sos2", "observable",
                "enforce-passivity", "bode-fano", "kyp-sim", "validate-power", "atlas"):
        assert tok in h.stdout, f"Missing '{tok}' in --help"

    st = run_cmd([cli_path, "selftest"])
    assert "16 / 16 PASSED" in st.stdout

    synth_arr_j = os.path.join(tmpdir, "synth_arr.json")
    run_cmd([cli_path, "em", "synth", "--mode", "array", "--seed", "2026", "--json", synth_arr_j])
    with open(synth_arr_j) as f:
        sa = json.load(f)
    assert sa["nominal_sll_db"] < -25.0 and sa["worst_tol_sll_db"] < -20.0

    synth_mat_j = os.path.join(tmpdir, "synth_mat.json")
    run_cmd([cli_path, "em", "synth", "--mode", "match", "--seed", "2026", "--json", synth_mat_j])
    with open(synth_mat_j) as f:
        sm = json.load(f)
    assert sm["nominal_s11_db"] < -20.0 and sm["worst_tol_s11_db"] < -14.0

    run_cmd([cli_path, "em", "fdtd", "--er", "4.0", "--sigma", "0.04",
             "--s2p", os.path.join(tmpdir, "fdtd.s2p"),
             "--cert", os.path.join(tmpdir, "fdtd.elecert.json")])
    run_cmd([cli_path, "cert-verify", os.path.join(tmpdir, "fdtd.elecert.json")])


def test_sdk(cli_path, tmpdir):
    freqs2, mats2, z02 = read_snp("benchmarks/em/microstrip_50ohm.s2p", 2)
    assert len(freqs2) > 10 and len(mats2[0]) == 2 and z02 == 50.0
    freqs3, mats3, z03 = read_snp("benchmarks/em/wilkinson_3port.s3p", 3)
    assert len(freqs3) > 10 and len(mats3[0]) == 3 and z03 == 50.0

    eng = EleEngine(cli_path=cli_path)
    st = eng.selftest()
    assert st["passed"] == st["total"] == 16

    p_ms = eng.em_passivity("benchmarks/em/microstrip_50ohm.s2p", cert_path=os.path.join(tmpdir, "ms.elecert.json"))
    assert p_ms["passive"] == 1 and p_ms["sigma_max"] < 1.0
    assert eng.cert_verify(os.path.join(tmpdir, "ms.elecert.json"))

    peec = eng.em_peec("benchmarks/em/busbar_800v.peec",
                        s2p_path=os.path.join(tmpdir, "busbar.s2p"),
                        cert_path=os.path.join(tmpdir, "peec.elecert.json"))
    assert peec["passive"] == 1 and 1.1e-8 <= peec["l_loop_h"] <= 1.5e-8

    fdtd = eng.em_fdtd(er=4.0, sigma=0.04,
                        s2p_path=os.path.join(tmpdir, "sdk_fdtd.s2p"),
                        cert_path=os.path.join(tmpdir, "sdk_fdtd.elecert.json"))
    assert fdtd["cfl_ok"] == 1 and fdtd["passive"] == 1 and fdtd["min_s11_db"] < -25.0

    ckt = eng.ckt_run("benchmarks/ckt/sic_halfbridge_800v.sp", corners=True,
                      cert_path=os.path.join(tmpdir, "sic.elecert.json"))
    assert ckt["soa_pass"] == 1 and ckt["wc_vmax"] < 1150.0

    eye = eng.ckt_si_eye("benchmarks/ckt/diff_pair_si.sp", rate_gbps=4.0,
                         cert_path=os.path.join(tmpdir, "eye.elecert.json"))
    assert eye["eye_pass"] == 1 and eye["eye_pda_v"] > 0.25

    mot = eng.motor_run("benchmarks/ee/ev_ipmsm_150kw.motor", rpm=3000.0, torque=260.0,
                        sweep=True, cert_path=os.path.join(tmpdir, "mot.elecert.json"))
    assert mot["foc_ok"] == 1 and mot["eff"] > 0.95

    pf = eng.grid_pf("benchmarks/ee/ieee9_wscc.grid", n1=True)
    assert pf["converged"] == 1 and pf["mismatch"] < 1e-11 and "n1_secure" in pf

    pg14_pf = eng.grid_pf("benchmarks/ee/pglib/pglib_opf_case14_ieee.m", n1=False)
    assert pg14_pf["converged"] == 1 and pg14_pf["mismatch"] < 1e-11
    pg118_pf = eng.grid_pf("benchmarks/ee/pglib/pglib_opf_case118_ieee.m", n1=False)
    assert pg118_pf["converged"] == 1 and pg118_pf["mismatch"] < 1e-11

    pg14_opf = eng.grid_opf("benchmarks/ee/pglib/pglib_opf_case14_ieee.m",
                            cert_path=os.path.join(tmpdir, "pglib14.elecert.json"))
    assert pg14_opf["c_lb_cert"] <= pg14_opf["c_ub_primal"] and pg14_opf["gap_pct"] < 20.0
    assert eng.cert_verify(os.path.join(tmpdir, "pglib14.elecert.json"))

    th = eng.lab_validate_thermal("data/cardputer/cardputer_thermal_rail.csv",
                                  sp_path=os.path.join(tmpdir, "therm.sp"),
                                  cert_path=os.path.join(tmpdir, "therm.elecert.json"))
    assert th["status"] == "CERTIFIED" and th["spice_rmse_c"] <= 1.50
    assert eng.cert_verify(os.path.join(tmpdir, "therm.elecert.json"))

    csi = eng.lab_validate_csi("data/cardputer/cardputer_rf_csi.csv",
                               s2p_path=os.path.join(tmpdir, "csi.s2p"),
                               cert_path=os.path.join(tmpdir, "csi.elecert.json"))
    assert csi["status"] == "CERTIFIED" and csi["passive"] == 1
    assert eng.cert_verify(os.path.join(tmpdir, "csi.elecert.json"))

    opf = eng.grid_opf("benchmarks/ee/ieee3_test.grid", cert_path=os.path.join(tmpdir, "opf3.elecert.json"))
    assert opf["c_lb_cert"] <= opf["c_ub_primal"] and opf["gap_pct"] < 1.0

    emt = eng.grid_emt("benchmarks/ee/ieee9_wscc.grid", fault_bus=7, t_clear=0.12,
                       cert_path=os.path.join(tmpdir, "emt9.elecert.json"))
    assert emt["stable"] == 1 and emt["v_post"] < emt["v_crit"] and emt["max_gfm_i_pu"] <= 1.2001

    f2c = eng.field_to_circuit("benchmarks/em/busbar_800v.peec",
                               "benchmarks/ckt/sic_halfbridge_800v.sp",
                               "benchmarks/ee/ev_ipmsm_150kw.motor",
                               out_dir=tmpdir)
    assert f2c["ok"] == 1 and f2c["passive_certified"] == 1 and f2c["soa_pass"] == 1 and f2c["motor_in_ellipse"] == 1

    # v10.0.0 Frontier CLI/SDK checks
    ch_wb2 = eng.grid_chordal("benchmarks/ee/wb2.grid", cert_path=os.path.join(tmpdir, "ch_wb2.elecert.json"))
    assert ch_wb2["certified"] == 1 and ch_wb2["c_lb_cert"] <= ch_wb2["c_ub_primal"]
    assert eng.cert_verify(os.path.join(tmpdir, "ch_wb2.elecert.json"))

    ch_pg14 = eng.grid_chordal("benchmarks/ee/pglib/pglib_opf_case14_ieee.m",
                               cert_path=os.path.join(tmpdir, "ch_pg14.elecert.json"))
    assert ch_pg14["certified"] == 1 and ch_pg14["c_lb_cert"] <= ch_pg14["c_ub_primal"]
    assert eng.cert_verify(os.path.join(tmpdir, "ch_pg14.elecert.json"))

    sos_wb2 = eng.grid_sos2("benchmarks/ee/wb2.grid", cert_path=os.path.join(tmpdir, "sos_wb2.elecert.json"))
    assert sos_wb2["sos2_gap_pct"] < 0.5 and sos_wb2["sos2_lb_cert"] > sos_wb2["shor1_lb_cert"]
    assert eng.cert_verify(os.path.join(tmpdir, "sos_wb2.elecert.json"))

    sos_c3 = eng.grid_sos2("benchmarks/ee/pglib/pglib_opf_case3_lmbd.m",
                           cert_path=os.path.join(tmpdir, "sos_c3.elecert.json"))
    assert sos_c3["sos2_gap_pct"] < sos_c3["shor1_gap_pct"]
    assert eng.cert_verify(os.path.join(tmpdir, "sos_c3.elecert.json"))

    obs_wb2 = eng.grid_observable("benchmarks/ee/wb2.grid", "vm2:2", eps=5.0,
                                  cert_path=os.path.join(tmpdir, "obs_wb2.elecert.json"))
    assert obs_wb2["status"] == "CERTIFIED" and obs_wb2["o_lb_cert"] <= obs_wb2["o_primal"] <= obs_wb2["o_ub_cert"]
    assert eng.cert_verify(os.path.join(tmpdir, "obs_wb2.elecert.json"))

    emb = eng.em_bound(kl=2.0, zeta=10.0, xi0=0.0, cells=16, bw_frac=0.10, fdtd_check=True,
                       cert_path=os.path.join(tmpdir, "emb.elecert.json"))
    assert emb["status"] == "CERTIFIED" and emb["p_searu_lb"] <= emb["p_local_ub_cert"] <= emb["p_global_ub"]
    assert emb["p_band_avg_searu"] <= emb["p_band_avg_ub"] and emb["fdtd_below_ub"] == 1
    assert eng.cert_verify(os.path.join(tmpdir, "emb.elecert.json"))

    ebf = eng.em_bode_fano(z0=50.0, rl=50.0, cl_pf=2.0, flo_ghz=2.0, fhi_ghz=3.0,
                           cert_path=os.path.join(tmpdir, "ebf.elecert.json"))
    assert ebf["certified"] == 1 and ebf["max_gamma"] >= ebf["bf_gamma_limit"]
    assert eng.cert_verify(os.path.join(tmpdir, "ebf.elecert.json"))

    kyp = eng.em_enforce_passivity("benchmarks/em/nonpassive_ai_surrogate.s2p",
                                   out_snp_path=os.path.join(tmpdir, "kyp_rep.s2p"),
                                   cert_path=os.path.join(tmpdir, "kyp_rep.elecert.json"))
    assert kyp["certified"] == 1 and kyp["orig_sigma_max"] > 1.0 and kyp["rep_sigma_max"] <= 1.0
    assert eng.cert_verify(os.path.join(tmpdir, "kyp_rep.elecert.json"))

    ksim3 = eng.em_kyp_sim("benchmarks/em/nonpassive_wilkinson_3port.s3p",
                           cert_path=os.path.join(tmpdir, "ksim3.elecert.json"))
    assert ksim3["certified"] == 1 and ksim3["orig_blowup"] == 1 and ksim3["rep_stable"] == 1 and ksim3["rep_lyap_monotone"] == 1
    assert eng.cert_verify(os.path.join(tmpdir, "ksim3.elecert.json"))

    lpw = eng.lab_validate_power("data/cardputer/cardputer_power_modes.csv",
                                 cert_path=os.path.join(tmpdir, "lpw.elecert.json"))
    assert lpw["status"] == "CERTIFIED" and lpw["spice_rmse_mv"] <= 30.0 and lpw["min_v_mv"] >= 3300.0
    assert eng.cert_verify(os.path.join(tmpdir, "lpw.elecert.json"))

    sos_c5 = eng.grid_sos2("benchmarks/ee/pglib/pglib_opf_case5_pjm.m",
                           cert_path=os.path.join(tmpdir, "sos_c5.elecert.json"))
    assert sos_c5["sos2_gap_pct"] < sos_c5["shor1_gap_pct"]
    assert eng.cert_verify(os.path.join(tmpdir, "sos_c5.elecert.json"))

    with EleFastSession("benchmarks/ee/wb2.grid") as sess:
        s_opf = sess.solve_opf()
        s_sos = sess.solve_sos2()
        s_obs = sess.solve_obs("vm2", 2, eps=5.0)
        assert s_sos["sos2_lb_cert"] > s_opf["c_lb_cert"]
        assert s_obs["o_lb_cert"] <= s_obs["o_primal"] <= s_obs["o_ub_cert"]
        sess.set_bus_limits(2, 0.95, 1.05)
        s_opf2 = sess.solve_opf()
        assert s_opf2["c_ub_primal"] < s_opf["c_ub_primal"]


def test_tamper_rejection(cli_path, cert_dir, tmpdir):
    # 1. Corrupt A_int entry in em_microstrip.elecert.json
    c1 = os.path.join(tmpdir, "t1_a_int.elecert.json")
    with open(os.path.join(cert_dir, "em_microstrip.elecert.json")) as f:
        rec1 = json.load(f)
    rec1["A_int"][0][0] -= 500000000
    with open(c1, "w") as f:
        json.dump(rec1, f)
    run_expect_fail([cli_path, "cert-verify", c1], "tamper A_int")

    # 2. Inflate mu_psd and mu_cert consistently in em_microstrip.elecert.json -> Cholesky/SDD check fails
    c2 = os.path.join(tmpdir, "t2_mu.elecert.json")
    with open(os.path.join(cert_dir, "em_microstrip.elecert.json")) as f:
        rec2 = json.load(f)
    rec2["mu_psd"] += 100000000
    rec2["mu_cert"] += 100000000
    with open(c2, "w") as f:
        json.dump(rec2, f)
    run_expect_fail([cli_path, "cert-verify", c2], "inflate mu_psd/mu_cert")

    # 3. Flip VETOED -> CERTIFIED in em_nonpassive_veto.elecert.json
    c3 = os.path.join(tmpdir, "t3_veto_flip.elecert.json")
    with open(os.path.join(cert_dir, "em_nonpassive_veto.elecert.json")) as f:
        rec3 = json.load(f)
    rec3["status"] = "CERTIFIED"
    with open(c3, "w") as f:
        json.dump(rec3, f)
    run_expect_fail([cli_path, "cert-verify", c3], "flip VETOED->CERTIFIED")

    # 4. Mutate 1 byte of copied .s2p referenced by a fresh cert -> SHA-256 mismatch
    s2p_copy = os.path.join(tmpdir, "ms_copy.s2p")
    c4 = os.path.join(tmpdir, "t4_sha.elecert.json")
    shutil.copy2("benchmarks/em/microstrip_50ohm.s2p", s2p_copy)
    run_cmd([cli_path, "em", "passivity", s2p_copy, "--cert", c4])
    run_cmd([cli_path, "cert-verify", c4])
    with open(s2p_copy, "rb") as f:
        raw = bytearray(f.read())
    raw[-5] = ord("9") if raw[-5] != ord("9") else ord("0")
    with open(s2p_copy, "wb") as f:
        f.write(raw)
    run_expect_fail([cli_path, "cert-verify", c4], "input SHA-256 tamper")

    # 5. Inflate c_lb_int in grid_ieee3_opf.elecert.json
    c5 = os.path.join(tmpdir, "t5_opf_lb.elecert.json")
    with open(os.path.join(cert_dir, "grid_ieee3_opf.elecert.json")) as f:
        rec5 = json.load(f)
    rec5["c_lb_int"] += 10000000000
    with open(c5, "w") as f:
        json.dump(rec5, f)
    run_expect_fail([cli_path, "cert-verify", c5], "inflate c_lb_int")

    # 6. Tamper ub_int above limit_hi_int in lab_cardputer_thermal.elecert.json
    c6 = os.path.join(tmpdir, "t6_lab_therm.elecert.json")
    with open(os.path.join(cert_dir, "lab_cardputer_thermal.elecert.json")) as f:
        rec6 = json.load(f)
    rec6["ub_int"] = rec6["limit_hi_int"] + 500000
    with open(c6, "w") as f:
        json.dump(rec6, f)
    run_expect_fail([cli_path, "cert-verify", c6], "inflate lab thermal ub_int > limit_hi_int")

    # 7. Corrupt clique S_int in chordal_wb2.elecert.json -> clique SDD / sum fails
    ch_src = os.path.join(cert_dir, "chordal_wb2.elecert.json")
    if not os.path.exists(ch_src):
        ch_src = os.path.join(tmpdir, "ch_wb2_tamper_src.elecert.json")
        run_cmd([cli_path, "grid", "chordal", "benchmarks/ee/wb2.grid", "--cert", ch_src])
    c7 = os.path.join(tmpdir, "t7_chordal_s_int.elecert.json")
    with open(ch_src) as f:
        rec7 = json.load(f)
    rec7["cliques"][0]["S_int"][0][0] += 100
    with open(c7, "w") as f:
        json.dump(rec7, f)
    run_expect_fail([cli_path, "cert-verify", c7], "corrupt chordal clique S_int")

    # 8. Corrupt A_sparse in chordal_wb2.elecert.json -> clique lift sum != A_sparse fails
    c8 = os.path.join(tmpdir, "t8_chordal_a_sparse.elecert.json")
    with open(ch_src) as f:
        rec8 = json.load(f)
    rec8["A_sparse"][0][2] += 100
    with open(c8, "w") as f:
        json.dump(rec8, f)
    run_expect_fail([cli_path, "cert-verify", c8], "corrupt chordal A_sparse")

    # 9. Tamper per-clique w_max_int in chordal case5_pjm -> c_lb_int mismatch fails
    c5_src = os.path.join(cert_dir, "chordal_case5_pjm.elecert.json")
    if not os.path.exists(c5_src):
        c5_src = os.path.join(tmpdir, "ch_c5_src.elecert.json")
        run_cmd([cli_path, "grid", "chordal", "benchmarks/ee/pglib/pglib_opf_case5_pjm.m", "--cert", c5_src])
    c9 = os.path.join(tmpdir, "t9_chordal_w_clique.elecert.json")
    with open(c5_src) as f:
        rec9 = json.load(f)
    rec9["cliques"][0]["w_max_int"] = max(1, rec9["cliques"][0]["w_max_int"] // 2)
    with open(c9, "w") as f:
        json.dump(rec9, f)
    run_expect_fail([cli_path, "cert-verify", c9], "tamper per-clique w_max_int")

    # 10. Invert lb_int > ub_int in em_bode_fano.elecert.json
    bf_src = os.path.join(cert_dir, "em_bode_fano.elecert.json")
    if not os.path.exists(bf_src):
        bf_src = os.path.join(tmpdir, "bf_src.elecert.json")
        run_cmd([cli_path, "em", "bode-fano", "--cert", bf_src])
    c10 = os.path.join(tmpdir, "t10_bode_fano_inv.elecert.json")
    with open(bf_src) as f:
        rec10 = json.load(f)
    rec10["lb_int"] = rec10["ub_int"] + 50000
    with open(c10, "w") as f:
        json.dump(rec10, f)
    run_expect_fail([cli_path, "cert-verify", c10], "invert bode_fano lb_int > ub_int")

    # 11. Corrupt 3-port KYP A_int diagonal
    k3_src = os.path.join(cert_dir, "em_kyp_3port.elecert.json")
    if not os.path.exists(k3_src):
        k3_src = os.path.join(tmpdir, "k3_src.elecert.json")
        run_cmd([cli_path, "em", "enforce-passivity", "benchmarks/em/nonpassive_wilkinson_3port.s3p", "--cert", k3_src])
    c11 = os.path.join(tmpdir, "t11_kyp3_a.elecert.json")
    with open(k3_src) as f:
        rec11 = json.load(f)
    rec11["A_int"][0][0] -= 50000000
    with open(c11, "w") as f:
        json.dump(rec11, f)
    run_expect_fail([cli_path, "cert-verify", c11], "corrupt 3-port KYP A_int")

    # 12. Inflate lab_cardputer_power ub_int > limit_hi_int
    pw_src = os.path.join(cert_dir, "lab_cardputer_power.elecert.json")
    if not os.path.exists(pw_src):
        pw_src = os.path.join(tmpdir, "pw_src.elecert.json")
        run_cmd([cli_path, "lab", "validate-power", "data/cardputer/cardputer_power_modes.csv", "--cert", pw_src])
    c12 = os.path.join(tmpdir, "t12_lab_pwr.elecert.json")
    with open(pw_src) as f:
        rec12 = json.load(f)
    rec12["ub_int"] = rec12["limit_hi_int"] + 500000
    with open(c12, "w") as f:
        json.dump(rec12, f)
    run_expect_fail([cli_path, "cert-verify", c12], "inflate lab power ub_int > limit_hi_int")


def main():
    ap = argparse.ArgumentParser(description="ELE v11.0.0 CLI & SDK Tests")
    ap.add_argument("--cli", default="build/ele_cli")
    ap.add_argument("--cert-dir", default="build/certs")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as tmpdir:
        test_cli_subcommands(args.cli, tmpdir)
        test_sdk(args.cli, tmpdir)
        test_tamper_rejection(args.cli, args.cert_dir, tmpdir)

    print("  [TEST_TOOLS PASS] CLI subcommands + EleEngine/EleFastSession SDK + 12/12 Tamper-Rejection checks")


if __name__ == "__main__":
    main()
