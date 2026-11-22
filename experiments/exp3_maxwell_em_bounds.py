#!/usr/bin/env python3
"""
Experiment 3: Maxwell-Poynting Dual Upper Bounds & KYP Bounded-Real Passivity Repair
Outputs: experiments/exp3_maxwell_em_bounds.json (Pure Python stdlib)
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
    bound_rows = []

    with tempfile.TemporaryDirectory() as td:
        for kl in [0.5, 1.0, 2.0, 4.0]:
            for zeta in [2.0, 5.0, 10.0, 20.0]:
                for cells in [4, 8, 16]:
                    cp = os.path.join(td, f"em_{kl}_{zeta}_{cells}.elecert.json")
                    res = eng.em_bound(kl=kl, zeta=zeta, xi0=0.0, cells=cells, cert_path=cp)
                    eng.cert_verify(cp)
                    certs_verified += 1
                    assert res["p_searu_lb"] <= res["p_local_ub_cert"] <= res["p_global_ub"]
                    bound_rows.append(res)

        kyp_cert = os.path.join(td, "kyp_repaired.elecert.json")
        kyp_s2p = os.path.join(td, "repaired.s2p")
        kyp_res = eng.em_enforce_passivity("benchmarks/em/nonpassive_ai_surrogate.s2p", out_snp_path=kyp_s2p, cert_path=kyp_cert)
        eng.cert_verify(kyp_cert)
        certs_verified += 1

    payload = {
        "experiment": "exp3_maxwell_em_bounds",
        "certs_verified": certs_verified,
        "maxwell_bounds": bound_rows,
        "kyp_passivity_repair": kyp_res,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp3_maxwell_em_bounds.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP3] {len(bound_rows)} EM bound configs + KYP repair ({certs_verified} certs verified) -> {out_path}")
    print(f"  KYP repair: orig_smax={kyp_res['orig_sigma_max']:.6f} -> rep_smax={kyp_res['rep_sigma_max']:.6f}, rms={kyp_res['rms_repair_err']:.6f}, mu_KYP={kyp_res['kyp_mu_psd']}, status={kyp_res['status']}")


if __name__ == "__main__":
    main()
