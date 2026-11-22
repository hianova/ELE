#!/usr/bin/env python3
"""
Experiment 2: Voltage & Thermal Congestion Bifurcation via In-Process EleFastSession
  - Sweep A: wb2.grid bus-2 Vmax from 0.95 to 1.10 (16 pts): Shor-1, SOS-2, Observable(vm2:2)
  - Sweep B: case3_lmbd.m branch-1 Smax from 35 to 80 MVA (10 pts): Shor-1, SOS-2, Observable(pflow:1)
Outputs: experiments/exp2_congestion_decoupling.json (Pure Python stdlib)
"""
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "oracles"))

from ele_sdk import EleFastSession  # noqa: E402


def main():
    os.chdir(REPO_ROOT)
    t_start = time.perf_counter()

    wb2_sweep = []
    with EleFastSession("benchmarks/ee/wb2.grid") as sess:
        for step in range(16):
            vmax2 = round(0.95 + 0.01 * step, 4)
            sess.set_bus_limits(2, 0.95, vmax2)
            opf = sess.solve_opf()
            sos = sess.solve_sos2()
            obs = sess.solve_obs("vm2", 2, eps=5.0)
            wb2_sweep.append({
                "vmax2_pu": vmax2,
                "shor1_lb_cert": opf["c_lb_cert"],
                "shor1_ub_primal": opf["c_ub_primal"],
                "shor1_gap_pct": opf["gap_pct"],
                "sos2_lb_cert": sos["sos2_lb_cert"],
                "sos2_gap_pct": sos["sos2_gap_pct"],
                "sos2_improvement_abs": sos["improvement_abs"],
                "obs_vm2_primal": obs["o_primal"],
                "obs_vm2_lb": obs["o_lb_cert"],
                "obs_vm2_ub": obs["o_ub_cert"],
                "obs_vm2_width": obs["width"],
            })

    case3_sweep = []
    with EleFastSession("benchmarks/ee/pglib/pglib_opf_case3_lmbd.m") as sess:
        for step in range(10):
            smax = round(35.0 + 5.0 * step, 2)
            sess.set_branch_limits(1, smax, shift_deg=0.0, angmin_deg=-30.0, angmax_deg=30.0)
            opf = sess.solve_opf()
            sos = sess.solve_sos2()
            obs = sess.solve_obs("pflow", 1, eps=5.0)
            case3_sweep.append({
                "branch1_smax_mva": smax,
                "shor1_lb_cert": opf["c_lb_cert"],
                "shor1_ub_primal": opf["c_ub_primal"],
                "shor1_gap_pct": opf["gap_pct"],
                "sos2_lb_cert": sos["sos2_lb_cert"],
                "sos2_gap_pct": sos["sos2_gap_pct"],
                "sos2_improvement_abs": sos["improvement_abs"],
                "obs_pflow1_primal": obs["o_primal"],
                "obs_pflow1_lb": obs["o_lb_cert"],
                "obs_pflow1_ub": obs["o_ub_cert"],
                "obs_pflow1_width": obs["width"],
            })

    elapsed = round(time.perf_counter() - t_start, 6)
    payload = {
        "experiment": "exp2_congestion_decoupling",
        "elapsed_sec": elapsed,
        "wb2_vmax_sweep": wb2_sweep,
        "case3_smax_sweep": case3_sweep,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp2_congestion_decoupling.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP2] wb2(16 pts) + case3_lmbd(10 pts) in {elapsed:.3f}s -> {out_path}")
    for r in wb2_sweep[::3]:
        print(f"  wb2 Vmax2={r['vmax2_pu']:.2f}: Shor1={r['shor1_gap_pct']:.4f}% -> SOS2={r['sos2_gap_pct']:.4f}%, vm2={r['obs_vm2_primal']:.4f} in [{r['obs_vm2_lb']:.4f}, {r['obs_vm2_ub']:.4f}]")
    for r in case3_sweep[::2]:
        print(f"  case3 Smax1={r['branch1_smax_mva']:4.1f} MVA: Shor1={r['shor1_gap_pct']:.4f}% -> SOS2={r['sos2_gap_pct']:.4f}%, Pflow1={r['obs_pflow1_primal']:.2f} in [{r['obs_pflow1_lb']:.2f}, {r['obs_pflow1_ub']:.2f}] MW")


if __name__ == "__main__":
    main()
