# ELE v11.0.0 — Certified Multiphysics Electronics, Electrical Engineering & Electromagnetics Platform (`EM` + `CKT` + `EE` + `Silicon/Power IP`)

**Certified Multiphysics Simulation, Lasserre/Chordal/Poynting/KYP Dual Certificates (`ELECERT-1.0` + `12×` Lean 4 `EleCert.lean` `E1–E20` incl. `E13b`), and Univalent Silicon/Power IP Compilation built on [`FLOW`](../FLOW/README.md):**

- **Electromagnetics (`em`)**: 2D TMz Yee FDTD dielectric-slab waveguide (`cfl_ok = 1`, `fc = 9.3685 GHz`, `min S11 = -33.4698 dB`, `max S21 = -0.2607 dB`, `max_energy_sum = 0.980571`), 3D/2.5D PEEC laminated 800V busbar extractor (`L_loop = 12.87 nH`, `C_bus = 313.0 pF`, `R_ac/R_dc = 302.6x`), Touchstone `.sNp` I/O (`microstrip`, `wilkinson`), Zero-FP-Trust multi-port passivity & reciprocity certification + non-passive 2-port/3-port AI-surrogate vetoes (`nonpassive_ai_surrogate.s2p` `sigma_max = 1.0350`, `nonpassive_wilkinson_3port.s3p` `sigma_max = 1.0675`, `recip_err = 1.456e-2` $\to$ `VETOED`), **KYP Bounded-Real LMI Passivity Enforcement + Causal Tustin Closed-Loop Instability Contrast (`em enforce-passivity` + `em kyp-sim`: 2-port `1.0350 -> 0.9920` cuts passive-load blowup `1.120e+07 -> 1.27e-11`, 3-port `1.0675 -> 0.9920` cuts blowup `2.193e+08 -> 2.54e-06` with monotonic discrete Lyapunov decay, `CertEmKypEnforce` + `CertEmKyp3Port`)**, **Multi-Frequency Maxwell-Poynting Dual Bounds + Independent 2D TMz Yee FDTD Verification (`em bound --bw-frac 0.10 --fdtd-check`: `SEARU = 22.776 <= FDTD = 23.195 <= LocalCert = 23.767 <= Global = 33.686`)**, **Real `R || C` LC-Ladder Bode-Fano Closure (`em bode-fano`: `max |S11| = -15.57 dB >= BF = -43.43 dB`, `I/I_max = 0.4250`, `CertEmBodeFano`)**, and `SEARU × FLOW 2-Jet` phased-array & broadband matching synthesis.
- **Electronics & Circuits (`ckt`)**: SPICE `.sp` netlist parser (`R, L, C, K, V, I, D, M, E, G, T`), `Gmin`-stepping + Newton-Raphson nonlinear DC (`Vd = 0.635491 V`, KCL residual `5.00e-15 A`), AC small-signal (`RC -3dB = 159.00 Hz`) & PDN impedance (`Z_max = 0.07435 Ohm < 0.25 Ohm`), energy-stable Trapezoidal transient + 5-corner PVT SOA enclosure (`800V` SiC half-bridge `wc = [677.78, 935.23] V` inside `[-20, 1150] V`), coupled RLGC PCB crosstalk & `SEARU` worst-case SI eye (`PDA = SEARU = 0.311102 V`).
- **Electrical Engineering (`ee`)**: Nonlinear cross-saturated EV traction IPMSM (`ev_ipmsm_150kw.motor`, MTPA at `3000 rpm` `T = 260.00 Nm` `eff = 96.20%`, flux-weakening at `10000 rpm` `Id = -346.41 A` `eff = 97.85%`, `40 kHz` FOC + Lean 4 cert), Multi-bus Power Grid (`wb2/3/5`, `ieee3`, `ieee9`, `ieee14` + official IEEE PES PGLib-OPF v23.07 `case3_lmbd`, `case5_pjm`, `case14`, `case30` / `__api` / `__sad`, `case57`, `case118` MATPOWER `.m` sparse CSR `Y_bus`) Polar NR AC-PF (`<= 1.80e-12 p.u.`) + `N-1` screening, Certified Shor-1 AC-OPF, **Sparse Per-Clique Chordal Decomposition + Lean `E13b` (`grid chordal --emit-lean`: `case118_ieee` 109 cliques / `tw <= 4` gap `14.2958%`, `case30_ieee__api` `5.5488%`, `case14_ieee` `0.1349%`, standalone `CertGridChordal.lean` kernel-verified)**, **Clique-Sparse Lasserre Moment-SOS-2 Tightening (`grid sos2`: `wb2` `1.9485% -> 0.0727%`, `case3_lmbd` `0.5065% -> 0.1877%`, multi-gen `case5_pjm` `5.5581% -> 4.3025%` (`+$220.94/h`), Lean `E18` + `has_chordal: 1`)**, **Certified Grid Observable Envelopes (`grid observable`: `vm2` & `pflow`)**, and Kron-reduced / `1.200 p.u.` GFM EMT swing transient + PEBS Lyapunov margin.
- **In-Process `ctypes` Discovery Engine (`EleFastSession`) & Reproducible Experiments (`experiments/`)**: Zero-reload persistent C grid session + 6 pure-stdlib discovery experiments ([ELE_DISCOVERY_REPORT.md](experiments/ELE_DISCOVERY_REPORT.md)).
- **Real M5Stack Cardputer-Adv Hardware Lab (`lab`, `firmware/cardputer_ele_lab/`, `data/cardputer/`)**: Live USB CDC instrumentation on ESP32-S3 (`rev=2`, anonymized `mac_sha8=02c9b474`) capturing 2.4 GHz OFDM Wi-Fi CSI (`40` frames, `ch1` `2412 MHz`, `RSSI = -53.4 dBm`, `SNR = 43.6 dB`, `tau_rms = 99.1 ns`, certified passive `.s2p` `sigma_max = 0.10038`), silicon self-heating/cooling + rail droop (`N = 220`, `dT = +4.50 C`, `Rth = 6.14 K/W`, `tau = 1.24 s`, `droop = 1.4 mV`, Cauer SPICE transient `RMSE = 0.5209 C`), **5-mode ESP32-S3 DVFS/FPU/Wi-Fi Power & Causal Thevenin PDN SPICE Fit (`N = 240`, `80M_IDLE -> 240M_IDLE -> 240M_FPU -> 240M_WIFI -> 80M_RECOV`, `V_min = 4130 mV`, `dT_peak = +6.40 C`, `V_oc = 4.1489 V`, `R_pdn = 0.0200 Ohm`, `C_bulk = 0.70 F`, SPICE transient `RMSE = 8.582 mV <= 30 mV`, `CERTIFIED`)**, and calibrated 12-bit GPIO10/GPIO1 ADC/RC telemetry (`N = 200+40`).
- **Cross-Pillar `field-to-circuit` Coupling**: PEEC busbar extraction (`12.87 nH`) $\to$ coupled SiC half-bridge Transient SOA (`[683.4, 932.4] V`, `PASS`) $\to$ DC-link-derated traction IPMSM envelope check (`4000 rpm`, `260 Nm`, `eff = 97.09%`).
- **Preserved Silicon, Power IP & Univalent GDSII v6.0 Backends (`Ele.lean`, `rtl/`, `tools/`, `sscb800/`)**: Pure-config `Ele.lean` (`<= 200` lines, `0` theorems), 3× `576B` `.flow` PDKs (`sky130`, `euv7`, `sic800v`), 6 shared `.fir` graphs (byte-identical to `../QUA/qcc/build/`), `42`/`44`/`141`/`157`-pin RTL & `2-Jet` OPC/GDSII v6.0, 11-probe `0-ULP` Verilator wind-tunnel (`40,960` core vectors + `<0.7V` VPD + `200 ns` `800V` SiC SSCB + `1.20 p.u.` `GFM` + `ASIL-D` LFP BMS), and `12×` Yosys SAT `QED`.

---

## Repository Boundaries Across Sibling Projects

| Repository | Scope & Responsibility | Explicit Non-Overlap with `ELE v11.0.0` |
| :--- | :--- | :--- |
| **`FLOW`** | Domain-free Lean 4 math (`Flow.Interval`, `Flow.Bundle`, `Flow.Schema`, `Flow.Morphism`), Pure-GPR `flow.h` / `searu.h`, `.fir` synthesizer | Provides math & optimizer primitives only; no EM/CKT/EE/Silicon physics |
| **`ELE`** (**This Repo**) | **Applied Electronics (`ckt`), Electrical Engineering (`ee`), Electromagnetics (`em`), `field-to-circuit`, `EleCert` formal witnesses, and RTL/GDSII/Power silicon backends** | Does NOT duplicate `PHY`'s Lagrangian/gauge/QFT PDEs or `CHEM`'s DFT/MD |
| **`PHY`** | Fundamental theoretical physics (analytical mechanics, electrodynamics covariance, SM/GR/CMT) | `ELE` owns applied engineering solvers (FDTD waveguide, PEEC, SPICE MNA, IPMSM, AC-OPF, grid EMT, VLSI/Power) |
| **`CHEM`** | Computational chemistry & materials (`DFT`, `MD`, `SM` thermo) | Supplies material constants externally; `ELE` consumes macroscopic `sigma`, `er`, device/line parameters |
| **`QUA`** | Quantum compiler & fault-tolerant circuits | Shares byte-identical SEARU-synthesized GF256/LUT5 `.fir` hardware primitives |

---

## Architecture Overview (`3 + 1` Pillars)

```text
  FLOW Pure-Math & Optimizer SSOT (../FLOW/core/{flow.h,searu.h,lean/Flow/Interval.lean}, ../FLOW/sdk/lean/Flow.lean)
        │
        ├──► Lean 4 Formal Verification Layer (lean/EleCert.lean — imports ONLY Flow.Interval, 0 sorryAx, 0 ofReduceBool)
        │      • Core Algebra   : E1 sdd_quadratic_form_nonneg, E2 cholesky_sdd_certified_psd, E3 checkCholeskySDD_sound
        │      • EM / CKT / EE  : E4–E12 passivity, FDTD, reciprocity, Tellegen, Trap, SOA/Eye, IPMSM, OPF, EMT
        │      • Frontier E13–20: Chordal clique, thermal/angle dual, observable, Poynting bound, KYP, SOS-2, 3-cycle, Bode-Fano
        │
        ├──► Pure ISO C17 EEE Library & CLI (include/ele_{cert,em,ckt,ee,frontier}.h, src/ele_{cert,em,ckt,ee,frontier,cli}.c)
        │      • [1] Electromagnetics (`em`) : .sNp, Cholesky+SDD passivity/veto, KYP LMI repair, Maxwell-Poynting bound, FDTD, PEEC
        │      • [2] Electronics (`ckt`)     : .sp parser, Gmin+NR DC, AC/PDN, Trap tran + 5-corner PVT SOA, RLGC + SEARU eye
        │      • [3] Electrical Eng. (`ee`)  : Saturated IPMSM, Polar NR AC-PF, Shor-1, Chordal SDP, SOS-2 Newton, Observable, EMT
        │      • Zero-FP-Trust JSON (`ELECERT-1.0` + `has_chordal: 1`) + FIPS-180-4 SHA-256 + Lean 4 Cert Emitter (`Cert*.lean`)
        │
        ├──► Independent Python Stdlib Oracle & SDK (oracles/{gen_benchmarks,ele_sdk,verify_ele_oracle,test_tools}.py)
        │      • Pure stdlib (`0` numpy/scipy): EleEngine + ctypes EleFastSession + Int/Chordal SDD + Atlas Audit -> Report
        │
        └──► Preserved Silicon, Power IP & GDSII v6.0 Backend (Ele.lean <=200 lines, tools/{ele_bootstrap.c,ele_fab.h}, rtl/, sscb800/)
               • Build artifacts kept 100% compatible: build/ele_bootstrap, build/pdk_*.flow, 6x *.fir, RTL/GDSII/SSCB800 gates
```

---

## Verified EEE Multiphysics & Lean 4 Certification Results (`ELE_VERIFICATION_REPORT.md`)

All metrics below match live execution of `ele_cli bench` and independent pure-Python oracle audit in [ELE_VERIFICATION_REPORT.md](ELE_VERIFICATION_REPORT.md):

| Pillar | Benchmark | Key Measured Engine Metric | Independent Oracle / Witness Status |
| :--- | :--- | :--- | :--- |
| **`em`** | `microstrip_50ohm.s2p` | `sigma_max = 0.979608`, `recip_err = 0.0`, `mu_cert = 403,203,116` | Oracle `sigma_max` residual `2.89e-11`, `CERTIFIED` |
| **`em`** | `wilkinson_3port.s3p` | `sigma_max = 0.964712`, `recip_err = 0.0`, `mu_cert = 692,328,261` | Oracle `sigma_max` residual `1.20e-10`, `CERTIFIED` |
| **`em`** | `nonpassive_ai_surrogate.s2p` | `sigma_max = 1.035000`, `passive = 0`, `mu_cert = -712,662,217` | Correctly **`VETOED`** (`0.00e+00` diff) |
| **`em`** | `busbar_800v.peec` | `L_loop = 12.87 nH`, `C_bus = 313.0 pF`, `Rac/Rdc = 302.6x` | Synthesized `.s2p` `sigma_max = 0.99995`, `CERTIFIED` |
| **`em`** | `fdtd_wg_slab` (2D TMz + PML) | `fc = 9.3685 GHz`, `minS11 = -33.47 dB`, `sum = 0.9806` | `cfl_ok = 1`, `fc` rel err `0.00e+00`, `CERTIFIED` |
| **`em`** | `searu_phased_array` | `nom = -29.51 dB`, `wc_tol = -22.54 dB` (vs `-13.15 dB` unif.) | `9.38 dB` worst-case SLL suppression gain |
| **`em`** | `searu_broadband_match` | `nom = -24.60 dB`, `wc_tol = -16.34 dB` (vs `-6.02 dB` raw) | `10.32 dB` worst-case matching gain |
| **`ckt`** | `rc_lowpass.sp` | `fc = 158.996 Hz`, `v(RC) = 0.63027 V` | Analytic `159.155 Hz` / `0.63212 V` (`2.92e-3` rel err) |
| **`ckt`** | `rlc_pdn.sp` | `Zmax = 0.07435 Ohm` at `8.62 MHz` | Strictly below `0.25 Ohm` mask (`pdn_pass = 1`) |
| **`ckt`** | `diode_rectifier.sp` | `Vd = 0.635491 V`, KCL residual `5.00e-15 A` | Shockley NR `2.82e-10 V`; `ngspice 47` `0.635540 V` (`4.89e-05 V`) |
| **`ckt`** | `rc_lowpass.sp` / `rlc_pdn.sp` vs `ngspice 47` | `fc = 159.00 Hz`, `Zmax = 0.07435 Ohm` | `ngspice 47`: `159.16 Hz` (`2.92e-03`), `0.07440 Ohm` (`5.02e-05 Ohm`) |
| **`ckt`** | `sic_halfbridge_800v.sp` | 5-corner PVT SOA `wc = [677.78, 935.23] V` | Strictly inside `[-20, 1150] V` (`214.77 V` margin), `CERTIFIED` |
| **`ckt`** | `diff_pair_si.sp` | Coupled 8-section RLGC eye `PDA = SEARU = 0.3111 V` | Explicit worst-case pattern matches PDA (`0.00e+00` diff) |
| **`ee`** | `ev_ipmsm_150kw.motor` | `3krpm`: `260.00 Nm` (`eff=96.20%`); `10krpm FW`: `eff=97.85%`; `40kHz FOC` | Torque oracle residual `4.44e-07 Nm`, `CERTIFIED` |
| **`ee`** | `ieee3 / ieee9 / ieee14` AC-PF | Polar NR mismatch `2.94e-15` / `1.69e-14` / `1.06e-14 p.u.` | Complex `Y_bus` KCL oracle: `5.83e-15` / `3.60e-14` / `2.28e-14 pu` |
| **`ee`** | PGLib `case14 / case30 / case57 / case118` AC-PF | Sparse CSR `mis`: `9.53e-15` / `1.47e-14` / `1.80e-12` / `6.15e-14 pu` | Complex `Y_bus` KCL oracle: `<= 1.80e-12 pu` across all 4 grids |
| **`ee`** | `ieee3_test.grid` AC-OPF | Certified bracket `[3269.80, 3270.32] $/h` (`mu_psd = -932,007`) | **Gap = `0.0161%`** (`< 1%`), `CERTIFIED` |
| **`ee`** | `ieee9_wscc.grid` AC-OPF | Certified bracket `[5292.84, 5297.58] $/h` (`mu_psd = -48,968,474`) | **Gap = `0.0895%`** (`< 5%`), `CERTIFIED` |
| **`ee`** | PGLib `case14 / case30` AC-OPF | `case14`: `[2126.62, 2356.04] $/h`; `case30`: `[4206.21, 8288.42] $/h` | `case14` gap `9.7373%`; `case30` congested `C_LB <= C_UB`, `CERTIFIED` |
| **`ee`** | `ieee9_wscc.grid` EMT + GFM | `V_post = 0.4272 < V_crit = 0.7033`, `CCT = 0.2140 s`, `Igfm = 1.200 pu` | PEBS Lyapunov basin verified, `CERTIFIED` |
| **`lab`** | Cardputer `cardputer_thermal_rail.csv` | `N=220`, `dT=4.50 C`, `Rth=6.14 K/W`, `tau=1.24 s`, `droop=1.4 mV` | `ESP32-S3 rev=2 id=02c9b474`, Cauer SPICE `RMSE=0.5209 C`, `CERTIFIED` |
| **`lab`** | Cardputer `cardputer_rf_csi.csv` | `40` frames `ch1` (`2412 MHz`), `RSSI=-53.4 dBm`, `tau_rms=99.1 ns` | Synthesized `.s2p` `sigma_max=0.10038`, `recip=0.0e+00`, `CERTIFIED` |
| **`lab`** | Cardputer `cardputer_gpio_rc_adc.csv` | `GPIO10 = 4150.5 +- 8.7 mV` (`200` samp), `GPIO1 = 3160 mV` (`40` samp) | Manifest SHA-256 + privacy check verified (`PASS`) |

### Standalone Lean 4 Kernel Certificates (`leanchecker` Verified, `0 sorryAx`, `0 Lean.ofReduceBool`)

| Module | Certified Theorems (from [EleCert.lean](lean/EleCert.lean)) | Verified Axiom Dependencies |
| :--- | :--- | :--- |
| `CertEmMicrostrip` | `passivity_psd_sound`, `multiport_passivity_certified`, `reciprocity_certified` | `[propext, Classical.choice, Quot.sound]` / `[propext, Quot.sound]` |
| `CertEmWilkinson` | `passivity_psd_sound`, `multiport_passivity_certified`, `reciprocity_certified` | `[propext, Classical.choice, Quot.sound]` / `[propext, Quot.sound]` |
| `CertEmMaxwellBound` | `maxwell_psd_sound`, `maxwell_poynting_bound_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertEmBodeFano` | `bode_fano_match_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertEmKypEnforce` | `kyp_passivity_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertEmKyp3Port` | `kyp_passivity_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertCktSic800v` | `soa_corner_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertMotorIpmsm` | `ipmsm_certified` | **No axioms (`0` axioms)** |
| `CertGridIeee3Opf` | `ac_opf_psd_sound`, `ac_opf_bracket_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertGridChordal` | `grid_chordal_certified` (`E13b`) | `[propext, Classical.choice, Quot.sound]` |
| `CertDiscoveredWb2Sos` | `sos2_psd_sound`, `sos2_hierarchy_certified` | `[propext, Classical.choice, Quot.sound]` |
| `CertGridIeee9Stab` | `grid_stability_certified` | `[propext, Classical.choice, Quot.sound]` |

See [experiments/ELE_DISCOVERY_REPORT.md](experiments/ELE_DISCOVERY_REPORT.md) for full v11.0.0 discovery sweeps across all 11 power grids (`wb2/3/5`, `case3_lmbd`, `case5_pjm`, `case14`, `case30/__api/__sad`, `case57`, `case118`), `EleFastSession` `Vmax`/`Smax` bifurcation sweeps, 2D FDTD vs Maxwell-Poynting + `R||C` Bode-Fano closure, 2-port/3-port Tustin transient KYP contrast, and live ESP32-S3 Cardputer-Adv power/PDN SPICE validation.

---

## Preserved Silicon, Power IP & GDSII v6.0 Signoff (`v8.0.0` Hardware Backend)

Every hardware artifact, simulation harness, and formal proof from `v8.0.0` remains 100% intact and green inside `make test` and `make verify`:

- **4-Tier Physical RTL Hierarchy (`rtl/`)**: `ele_flow_core` (`141` pins: `33,277` cells / `237` lvls Profile A; `47,954` cells / `396` lvls canonical Profile B), `ele_flow_atlas_top` (`157` pins + `1× $mem_v2` 1RW SRAM), `ele_flow_qfn_top` (`42`-pin QFN-48, BI17 `<= 8/17`), `ele_power_qfn_top` (`44`-pin QFN-48 + `ele_dpwm_multiphase` `301` cells + `ele_sscb_trip_atom` `2,730` cells + BI5 `<= 2/5`).
- **`576B` `.flow` PDKs & Univalent `2-Jet` OPC/GDSII v6.0 (`Ele.lean` + `tools/ele_fab.h`)**: `pdk_sky130.flow` (`124,086 B` GDS, `11,253 ps`, `0` DRC / `0` LVS), `pdk_euv7.flow` (`126,774 B` GDS, `2` repeaters cutting critical path `146,724 -> 136,309 ps`), `pdk_sic800v.flow` (`124,150 B` GDS, `4,971 ps`), plus 6 `.fir` graphs byte-identical to `../QUA/qcc/build/`.
- **11-Probe `0-ULP` Verilator Wind-Tunnel (`rtl/ele_cosim_tb.sv`) & `sscb800/`**: `40,960` core `0-ULP` vectors, `<0.7V` 4-phase GPU VPD (`49.6 mV` undershoot saving `43.2%` MLCC vs PID), `800V DC` SiC SSCB (`200 ns` short trip, `0` hot-swap false trips), `1.20 p.u.` `GFM` ride-through (`max |I| = 1.198 p.u.`, `4 ms` resync vs `satSEP`), and `ASIL-D` LFP BMS (`0.00%` SoC secular drift).
- **Formal & Oracle Assurance**: `12×` Yosys unbounded `k`-induction SAT `QED`, `12×` Lean 4 `leanchecker` certificates (`0 sorryAx`, `0 Lean.ofReduceBool`), `ELECERT-1.0` + `has_chordal: 1` `__int128` SHA-256 witnesses, pure-stdlib independent physics oracle + optional `ngspice 47` cross-check + atlas audit ([verify_ele_oracle.py](oracles/verify_ele_oracle.py)), and `EleEngine` + `EleFastSession` SDK & `12/12` certificate tamper-rejection tests ([test_tools.py](oracles/test_tools.py)).

### Real M5Stack Cardputer-Adv Hardware Workflow & Factory Restore

The factory firmware image is backed up at `build/cardputer_factory_backup.bin` (`3,145,728` bytes, preserved automatically across `make clean`):

```sh
# Flash custom ELE lab firmware, capture real measurements into data/cardputer/, and validate
make cardputer-flash
make cardputer-capture
./build/ele_cli lab validate-thermal data/cardputer/cardputer_thermal_rail.csv --sp build/cardputer_thermal_rc.sp --cert build/lab_thermal.elecert.json
./build/ele_cli lab validate-csi data/cardputer/cardputer_rf_csi.csv --s2p build/cardputer_rf_csi.s2p --cert build/lab_csi.elecert.json
./build/ele_cli lab validate-power data/cardputer/cardputer_power_modes.csv --sp build/cardputer_power_pdn.sp --cert build/lab_power.elecert.json

# Restore M5Stack Cardputer-Adv back to original factory firmware
make cardputer-restore
# Or directly with esptool:
esptool --chip esp32s3 --port /dev/cu.usbmodem101 --baud 460800 write-flash 0x0 build/cardputer_factory_backup.bin
```

---

## Quick Start

Requires sibling [`FLOW`](../FLOW/README.md) (`../FLOW`), Lean 4 (`lean` / `leanchecker` / `elan`), a C17/C++20 compiler (`clang`/`gcc`), `verilator` ($\ge 5.0$), `yosys`, and `python3` (stdlib only):

```sh
# Build the EEE multiphysics CLI/SDK, Lean EleCert kernel, and hardware bootstrap
make clean && make eee-build

# Run the EEE multiphysics gate alone (selftest + bench + field-to-circuit + 6x Lean4 certs + Oracle + SDK/tamper tests)
make eee-gate

# Use the CLI or Python SDK directly
./build/ele_cli selftest
./build/ele_cli field-to-circuit --peec benchmarks/em/busbar_800v.peec \
  --sp benchmarks/ckt/sic_halfbridge_800v.sp --motor benchmarks/ee/ev_ipmsm_150kw.motor
python3 -c 'from oracles.ele_sdk import EleEngine; print(EleEngine().grid_opf("benchmarks/ee/ieee3_test.grid"))'

# Run ALL silicon + GDSII + SSCB800 + EEE verification gates
make clean && make test

# Run ALL silicon, GDSII, SSCB800 & EEE verification gates
make clean && make verify
```

---

## License

Released under the [MIT License](LICENSE).
