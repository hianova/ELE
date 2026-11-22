#!/usr/bin/env python3
"""
Experiment 6: Real M5Stack Cardputer-Adv (ESP32-S3) Multi-Mode Power State &
              Causal Thevenin PDN RC SPICE Model Fit (ELE v11.0.0)
Outputs: experiments/exp6_cardputer_power_pdn.json (Pure Python stdlib)
"""
import hashlib
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
    csv_path = "data/cardputer/cardputer_power_modes.csv"
    man_path = "data/cardputer/hardware_manifest.json"
    with open(man_path, "r") as f:
        man = json.load(f)
    h = hashlib.sha256()
    with open(csv_path, "rb") as rf:
        h.update(rf.read())
    csv_sha = h.hexdigest()
    assert csv_sha == man["files"]["cardputer_power_modes.csv"], "CSV SHA-256 mismatch vs hardware_manifest.json"

    with tempfile.TemporaryDirectory() as td:
        sp = os.path.join(td, "pdn.sp")
        cp = os.path.join(td, "lab_pwr.elecert.json")
        res = eng.lab_validate_power(csv_path, sp_path=sp, cert_path=cp)
        eng.cert_verify(cp)

    payload = {
        "experiment": "exp6_cardputer_power_pdn",
        "board": man["board"],
        "chip": man["chip"],
        "mac_sha8": man["mac_sha8"],
        "csv_sha256": csv_sha,
        "pdn_validation": res,
    }
    out_path = os.path.join(REPO_ROOT, "experiments", "exp6_cardputer_power_pdn.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[EXP6] Cardputer-Adv 5-mode PDN SPICE fit verified -> RMSE={res['spice_rmse_mv']:.3f} mV, dT={res['delta_t_c']:.2f} C ({out_path})")


if __name__ == "__main__":
    main()
