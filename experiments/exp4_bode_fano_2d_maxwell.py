#!/usr/bin/env python3
"""
Experiment 4: Multi-Frequency 2D Maxwell Poynting + 2D Yee FDTD Verification
              & Complex R||C Broadband Bode-Fano Match Closure (ELE v11.0.0)
Outputs: experiments/exp4_bode_fano_2d_maxwell.json (Pure Python stdlib)
"""
import json
import os
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "oracles"))

from ele_sdk import EleEngine  # noqa: E402


def main():
    os.chdir(REPO_ROOT)
    eng = EleEngine("build/ele_cli")
    certs_verified = 0
    fdtd_rows = []
    bf_rows = []

    with tempfile.TemporaryDirectory() as td:
        for kl in [1.0, 2.0, 3.0]:
            for zeta in [5.0, 10.0]:
                for bw in [0.0, 0.10, 0.20]:
                    cp = os.path.join(td, f"mb_{kl}_{zeta}_{bw}.elecert.json")
                    do_fdtd = (abs(bw) < 1e-12)
                    r = eng.em_bound(kl=kl, zeta=zeta, xi0=0.0, cells=16,
                                     bw_frac=bw, fdtd_check=do_fdtd, cert_path=cp)
                    eng.cert_verify(cp)
                    certs_verified += 1
                    assert r["p_searu_lb"] <= r["p_local_ub_cert"] <= r["p_global_ub"]
                    assert r["p_band_avg_searu"] <= r["p_band_avg_ub"]
                    if do_fdtd:
                        rel_diff = abs(r["fdtd_ext_power"] - r["p_searu_lb"]) / r["p_searu_lb"]
                        assert 0.0 < r["fdtd_ext_power"] <= r["p_global_ub"] and rel_diff < 0.06
                        if abs(kl - 2.0) < 1e-9 and abs(zeta - 10.0) < 1e-9:
                            assert r["fdtd_below_ub"] == 1
                    fdtd_rows.append(r)

        for cl_pf in [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]:
            for bw_ghz in [0.5, 1.0]:
                flo = 2.5 - 0.5 * bw_ghz
                fhi = 2.5 + 0.5 * bw_ghz
                cp = os.path.join(td, f"bf_{cl_pf}_{bw_ghz}.elecert.json")
                lp = os.path.join(td, f"CertBf_{str(cl_pf).replace('.', '_')}_{str(bw_ghz).replace('.', '_')}.lean")
                r = eng.em_bode_fano(z0=50.0, rl=50.0, cl_pf=cl_pf, flo_ghz=flo, fhi_ghz=fhi,
                                     cert_path=cp, lean_path=lp, mod_name="CertBfTmp")
                eng.cert_verify(cp)
                certs_verified += 1
                assert r["certified"] == 1 and r["max_gamma"] >= r["bf_gamma_limit"] - 1e-9
                assert 0.0 <= r["integral_ratio"] <= 1.0
                bf_rows.append(r)

    payload = {
        "experiment": "exp4_bode_fano_2d_maxwell",
        "certs_verified": certs_verified,
        "maxwell_2d_fdtd_checks": fdtd_rows,
        "bode_fano_rc_closure": bf_rows,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp4_bode_fano_2d_maxwell.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP4] {len(fdtd_rows)} 2D Maxwell+FDTD + {len(bf_rows)} Bode-Fano R||C sweeps ({certs_verified} certs verified) -> {out_path}")


if __name__ == "__main__":
    main()
