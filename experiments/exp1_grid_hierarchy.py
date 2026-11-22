#!/usr/bin/env python3
"""
Experiment 1: Certified AC-OPF Hierarchy Benchmark (Shor-1 vs Chordal-SDSOS/SDD vs Lasserre SOS-2)
Outputs: experiments/exp1_grid_hierarchy.json (Pure Python stdlib)
"""
import json
import os
import sys
import tempfile
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "oracles"))

from ele_sdk import EleEngine  # noqa: E402


CASES = [
    ("wb2", "benchmarks/ee/wb2.grid", True, True),
    ("wb3", "benchmarks/ee/wb3.grid", True, True),
    ("wb5", "benchmarks/ee/wb5.grid", True, True),
    ("case3_lmbd", "benchmarks/ee/pglib/pglib_opf_case3_lmbd.m", True, True),
    ("case5_pjm", "benchmarks/ee/pglib/pglib_opf_case5_pjm.m", True, True),
    ("case14_ieee", "benchmarks/ee/pglib/pglib_opf_case14_ieee.m", True, False),
    ("case30_ieee", "benchmarks/ee/pglib/pglib_opf_case30_ieee.m", True, False),
    ("case30_ieee__api", "benchmarks/ee/pglib/pglib_opf_case30_ieee__api.m", True, False),
    ("case30_ieee__sad", "benchmarks/ee/pglib/pglib_opf_case30_ieee__sad.m", True, False),
    ("case57_ieee", "benchmarks/ee/pglib/pglib_opf_case57_ieee.m", False, False),
    ("case118_ieee", "benchmarks/ee/pglib/pglib_opf_case118_ieee.m", False, False),
]


def main():
    os.chdir(REPO_ROOT)
    eng = EleEngine("build/ele_cli")
    results = []
    certs_verified = 0

    with tempfile.TemporaryDirectory() as td:
        for tag, rel, do_shor1, do_sos2 in CASES:
            row = {"tag": tag, "path": rel}
            if do_shor1:
                t0 = time.perf_counter()
                c_shor = os.path.join(td, f"{tag}_shor1.elecert.json")
                r_shor = eng.grid_opf(rel, cert_path=c_shor)
                t_shor = time.perf_counter() - t0
                eng.cert_verify(c_shor)
                certs_verified += 1
                row["shor1"] = {
                    "c_lb_cert": r_shor["c_lb_cert"],
                    "c_ub_primal": r_shor["c_ub_primal"],
                    "gap_pct": r_shor["gap_pct"],
                    "mu_psd": r_shor["mu_psd"],
                    "elapsed_sec": round(t_shor, 6),
                }
            else:
                row["shor1"] = None

            t0 = time.perf_counter()
            c_ch = os.path.join(td, f"{tag}_chordal.elecert.json")
            r_ch = eng.grid_chordal(rel, cert_path=c_ch)
            t_ch = time.perf_counter() - t0
            eng.cert_verify(c_ch)
            certs_verified += 1
            row["chordal"] = {
                "n_bus": r_ch["n_bus"],
                "n_cliques": r_ch["n_cliques"],
                "max_clique_buses": r_ch["max_clique_buses"],
                "treewidth_ub": r_ch["treewidth_ub"],
                "c_lb_cert": r_ch["c_lb_cert"],
                "c_ub_primal": r_ch["c_ub_primal"],
                "gap_pct": r_ch["gap_pct"],
                "min_clique_mu_psd": r_ch["min_clique_mu_psd"],
                "certified": r_ch["certified"],
                "elapsed_sec": round(t_ch, 6),
            }

            if do_sos2:
                t0 = time.perf_counter()
                c_sos = os.path.join(td, f"{tag}_sos2.elecert.json")
                r_sos = eng.grid_sos2(rel, cert_path=c_sos)
                t_sos = time.perf_counter() - t0
                eng.cert_verify(c_sos)
                certs_verified += 1
                row["sos2"] = {
                    "d2_dim": r_sos["d2_dim"],
                    "n_monos4": r_sos["n_monos4"],
                    "sos2_lb_raw": r_sos["sos2_lb_raw"],
                    "sos2_lb_cert": r_sos["sos2_lb_cert"],
                    "c_ub_primal": r_sos["c_ub_primal"],
                    "sos2_gap_pct": r_sos["sos2_gap_pct"],
                    "improvement_abs": r_sos["improvement_abs"],
                    "q0_mu_psd": r_sos["q0_mu_psd"],
                    "elapsed_sec": round(t_sos, 6),
                }
            else:
                row["sos2"] = None

            results.append(row)

    payload = {
        "experiment": "exp1_grid_hierarchy",
        "certs_verified": certs_verified,
        "cases": results,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp1_grid_hierarchy.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP1] {len(results)} grids, {certs_verified} certs verified -> {out_path}")
    for r in results:
        ch = r["chordal"]
        s1 = f"{r['shor1']['gap_pct']:.4f}%" if r["shor1"] else "N/A"
        s2 = f"{r['sos2']['sos2_gap_pct']:.4f}% (+${r['sos2']['improvement_abs']:.2f}/h)" if r["sos2"] else "N/A"
        print(f"  {r['tag']:18s} tw<={ch['treewidth_ub']} Shor1={s1:9s} Chordal={ch['gap_pct']:7.4f}% SOS2={s2}")


if __name__ == "__main__":
    main()
