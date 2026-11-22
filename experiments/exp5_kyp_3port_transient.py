#!/usr/bin/env python3
"""
Experiment 5: 2-Port & 3-Port Non-Passive AI Surrogate KYP LMI Repair vs
              Causal Tustin Closed-Loop Transient Blow-Up & Lyapunov Decay (ELE v11.0.0)
Outputs: experiments/exp5_kyp_3port_transient.json (Pure Python stdlib)
"""
import json
import os
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "oracles"))

from ele_sdk import EleEngine  # noqa: E402


BENCHMARKS = [
    ("2port_surrogate", "benchmarks/em/nonpassive_ai_surrogate.s2p"),
    ("3port_wilkinson_surrogate", "benchmarks/em/nonpassive_wilkinson_3port.s3p"),
]


def main():
    os.chdir(REPO_ROOT)
    eng = EleEngine("build/ele_cli")
    certs_verified = 0
    kyp_rows = []
    sim_rows = []

    with tempfile.TemporaryDirectory() as td:
        for tag, rel in BENCHMARKS:
            kc = os.path.join(td, f"{tag}_kyp.elecert.json")
            ksnp = os.path.join(td, f"{tag}_rep.s{tag[0]}p")
            kres = eng.em_enforce_passivity(rel, out_snp_path=ksnp, cert_path=kc)
            eng.cert_verify(kc)
            certs_verified += 1
            assert kres["certified"] == 1 and kres["rep_sigma_max"] <= 1.0
            kyp_rows.append({"tag": tag, "path": rel, **kres})

            for rho in [0.90, 0.95, 0.985]:
                sc = os.path.join(td, f"{tag}_sim_{rho}.elecert.json")
                sres = eng.em_kyp_sim(rel, rho=rho, steps=400, cert_path=sc)
                eng.cert_verify(sc)
                certs_verified += 1
                assert sres["rep_stable"] == 1 and sres["rep_lyap_monotone"] == 1
                sim_rows.append({"tag": tag, "path": rel, **sres})

    payload = {
        "experiment": "exp5_kyp_3port_transient",
        "certs_verified": certs_verified,
        "kyp_repairs": kyp_rows,
        "transient_contrasts": sim_rows,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp5_kyp_3port_transient.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP5] {len(kyp_rows)} KYP repairs + {len(sim_rows)} Tustin closed-loop sweeps ({certs_verified} certs verified) -> {out_path}")


if __name__ == "__main__":
    main()
