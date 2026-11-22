# ELE v11.0.0 Verification Report

Generated automatically by pure-stdlib [verify_ele_oracle.py](oracles/verify_ele_oracle.py).

## 1. Electromagnetics (`em`) Independent Oracle

| Benchmark | Engine Result | Oracle Check | Residual / Margin | Status |
| --- | --- | --- | --- | --- |
| microstrip_50ohm.s2p | sigma_max=0.979608 | sigma_max=0.979608 | 2.89e-11 | PASS |
| wilkinson_3port.s3p | sigma_max=0.964712 | sigma_max=0.964712 | 1.20e-10 | PASS |
| nonpassive_ai_surrogate.s2p | VETOED (sigma=1.0350) | VETO (sigma=1.0350) | 0.00e+00 | PASS |
| busbar_800v.peec | L=12.87 nH, passive=1 | L in [11,15] nH, sigma=0.99995 | Rac/Rdc=302.6x | PASS |
| fdtd_wg_slab | fc=9.3685 GHz, sum=0.9806 | fc=9.3685 GHz, sigma=0.9928 | rel=0.00e+00 | PASS |
| searu_phased_array | nom=-29.51 dB, wc=-22.54 dB | base=-13.15 dB | gain=9.38 dB | PASS |
| searu_broadband_match | nom=-24.60 dB, wc=-16.34 dB | base=-6.02 dB | gain=10.32 dB | PASS |

## 2. Electronics & Circuits (`ckt`) Independent Oracle

| Benchmark | Engine Result | Oracle Check | Residual / Margin | Status |
| --- | --- | --- | --- | --- |
| rc_lowpass.sp | fc=158.996 Hz, V=0.63027 | fc=159.155 Hz, V=0.63212 | rel=2.92e-03 | PASS |
| rlc_pdn.sp | Zmax=0.07435 Ohm | < 0.10000 Ohm (pass=1) | margin=0.1757 Ohm | PASS |
| diode_rectifier.sp | Vd=0.635491 V, res=5.00e-15 | Vd=0.635491 V | diff=2.82e-10 V | PASS |
| sic_halfbridge_800v.sp | [677.78, 935.23] V | [-20.00, 1150.00] V (5 corners) | margin=214.77 V | PASS |
| diff_pair_si.sp | PDA=0.3111 V, SEARU=0.3111 V | > 0.2500 V (diff < 1e-6) | diff=0.00e+00 V | PASS |

## 3. External Golden-Solver Cross-Validation (`ngspice`)

| Circuit | ELE CKT Solver | ngspice Reference | Agreement | Status |
| --- | --- | --- | --- | --- |
| rc_lowpass.sp | fc=159.00 Hz, V=0.63027 V | ngspice: fc=159.16 Hz, V=0.63212 V | rel=2.92e-03 | PASS |
| diode_rectifier.sp | Vd=0.635491 V | ngspice: Vd=0.635540 V | diff=4.89e-05 V | PASS |
| rlc_pdn.sp | Zmax=0.07435 Ohm | ngspice: Zmax=0.07440 Ohm | diff=5.02e-05 Ohm | PASS |

## 4. Electrical Machines & Drives (`motor`) Independent Oracle

| Benchmark | Engine Result | Oracle Check | Residual / Margin | Status |
| --- | --- | --- | --- | --- |
| ipmsm@3000rpm | T=260.0003 Nm, eff=96.20% | T=260.0003 Nm | err=4.44e-07 Nm | PASS |
| ipmsm@10000rpm_fw | Id=-346.41 A, V=457.28/461.88 V | I<=400A, V<=461.88V | eff=97.85% | PASS |
| ipmsm_foc_40khz | converged=1, T=260.26 Nm | T~260 Nm | Ipeak=322.96 A | PASS |

## 5. Power Grids (`grid` + Official IEEE PES PGLib-OPF v23.07) Independent Oracle

| Benchmark | Engine Result | Oracle Check | Residual / Margin | Status |
| --- | --- | --- | --- | --- |
| ieee3_acpf (3b) | mis=2.94e-15, loss=4.3571 MW | complex Ybus KCL | oracle_mis=5.83e-15 pu | PASS |
| ieee9_acpf (9b) | mis=1.69e-14, loss=4.6786 MW | complex Ybus KCL | oracle_mis=3.60e-14 pu | PASS |
| ieee14_acpf (14b) | mis=1.06e-14, loss=13.3933 MW | complex Ybus KCL | oracle_mis=2.28e-14 pu | PASS |
| pglib14_acpf (14b) | mis=9.53e-15, loss=16.6658 MW | complex Ybus KCL | oracle_mis=1.43e-14 pu | PASS |
| pglib30_acpf (30b) | mis=1.47e-14, loss=20.3588 MW | complex Ybus KCL | oracle_mis=2.21e-14 pu | PASS |
| pglib57_acpf (57b) | mis=1.80e-12, loss=29.9158 MW | complex Ybus KCL | oracle_mis=1.80e-12 pu | PASS |
| pglib118_acpf (118b) | mis=6.15e-14, loss=244.1480 MW | complex Ybus KCL | oracle_mis=1.12e-13 pu | PASS |
| ieee3_acopf | [3269.80, 3270.31] $/h | C_LB <= C_UB, gap < 1% | gap=0.0157% | PASS |
| ieee9_acopf | [5295.96, 5297.43] $/h | C_LB <= C_UB, gap < 5% | gap=0.0278% | PASS |
| pglib_case14_acopf | [2174.42, 2184.03] $/h | C_LB <= C_UB, gap < 20% | gap=0.4402% | PASS |
| pglib_case30_acopf | [7732.66, 8288.42] $/h | C_LB <= C_UB (congested) | gap=6.71% | PASS |
| ieee9_emt_gfm | V=0.4272<0.7033, CCT=0.2140s | PEBS Lyapunov, GFM<=1.2pu | Igfm=1.200 pu | PASS |

## 6. M5Stack Cardputer-Adv (`ESP32-S3`) Real Hardware Lab Validation (`lab`)

| Dataset | Extracted Physics / Fit | Hardware + Model Oracle | Residual / Count | Status |
| --- | --- | --- | --- | --- |
| cardputer_thermal_rail.csv | dT=4.50C, Rth=6.14K/W, droop=1.4mV | ESP32-S3 rev=2 id=02c9b474, Cauer SPICE <=1.5C | RMSE=0.5209 C | PASS |
| cardputer_rf_csi.csv | ch1, RSSI=-53.4dBm, tau=99.1ns | 40 frames, sigma=0.10038 | recip=0.0e+00 | PASS |
| cardputer_gpio_rc_adc.csv | GPIO10=4150.5+-8.7mV, GPIO1=3160->3160mV | manifest SHA-256 + 12b ADC/RC | N=200+40 | PASS |
| cardputer_power_modes.csv | Vmin=4130.0mV, droop=0.83mV, dT=6.40C | ESP32-S3 rev=2 id=02c9b474, Thevenin PDN SPICE <=30mV | RMSE=8.582 mV | PASS |

## 7. Zero-FP-Trust `ELECERT-1.0` Witness & SHA-256 Ledger

| Certificate | Pillar | Kind | Status | Input SHA-256 |
| --- | --- | --- | --- | --- |
| chordal_case118_ieee.elecert.json | grid | acopf_chordal | CERTIFIED | b1af0833849040c0... |
| chordal_case14_ieee.elecert.json | grid | acopf_chordal | CERTIFIED | bd5c568621de65e4... |
| chordal_case30_api.elecert.json | grid | acopf_chordal | CERTIFIED | 41f5a39b44607572... |
| chordal_case30_ieee.elecert.json | grid | acopf_chordal | CERTIFIED | cae3290639d98973... |
| chordal_case30_sad.elecert.json | grid | acopf_chordal | CERTIFIED | 221a62959b918b0e... |
| chordal_case3_lmbd.elecert.json | grid | acopf_chordal | CERTIFIED | 168f73f389aa8abd... |
| chordal_case57_ieee.elecert.json | grid | acopf_chordal | CERTIFIED | aa3b48f7cbaade2a... |
| chordal_case5_pjm.elecert.json | grid | acopf_chordal | CERTIFIED | cadf7501a15c2d50... |
| chordal_ieee3_test.elecert.json | grid | acopf_chordal | CERTIFIED | a8d092a42b4d624b... |
| chordal_ieee9_wscc.elecert.json | grid | acopf_chordal | CERTIFIED | 34902b40c83938a8... |
| chordal_wb2.elecert.json | grid | acopf_chordal | CERTIFIED | 51d60a7ab4fdd46c... |
| chordal_wb3.elecert.json | grid | acopf_chordal | CERTIFIED | 328998673bc8b774... |
| chordal_wb5.elecert.json | grid | acopf_chordal | CERTIFIED | c094625861ddde52... |
| ckt_si_eye.elecert.json | ckt | si_eye | CERTIFIED | 23aaf7f24d606d87... |
| ckt_sic_800v.elecert.json | ckt | soa_tran | CERTIFIED | cee7957375e232d1... |
| em_bode_fano.elecert.json | em | bode_fano_match | CERTIFIED | internal |
| em_fdtd_slab.elecert.json | em | passivity | CERTIFIED | 5293ebe9b078a5a0... |
| em_kyp_3port.elecert.json | em | kyp_passivity_enforce | CERTIFIED | ffbdae345aea138f... |
| em_kyp_repaired.elecert.json | em | kyp_passivity_enforce | CERTIFIED | 2e8119a024e35914... |
| em_kyp_sim_2port.elecert.json | em | kyp_transient_contrast | CERTIFIED | 2e8119a024e35914... |
| em_kyp_sim_3port.elecert.json | em | kyp_transient_contrast | CERTIFIED | ffbdae345aea138f... |
| em_maxwell_bound.elecert.json | em | maxwell_poynting_bound | CERTIFIED | internal |
| em_microstrip.elecert.json | em | passivity | CERTIFIED | 7b71796900cedf62... |
| em_nonpassive_veto.elecert.json | em | passivity | VETOED | 2e8119a024e35914... |
| em_peec_busbar.elecert.json | em | passivity | CERTIFIED | af98eee23634d69e... |
| em_wilkinson.elecert.json | em | passivity | CERTIFIED | 1aa585a1244bf171... |
| grid_ieee3_opf.elecert.json | grid | acopf | CERTIFIED | a8d092a42b4d624b... |
| grid_ieee9_emt.elecert.json | grid | emt_stab | CERTIFIED | 34902b40c83938a8... |
| grid_ieee9_opf.elecert.json | grid | acopf | CERTIFIED | 34902b40c83938a8... |
| grid_pglib14_opf.elecert.json | grid | acopf | CERTIFIED | bd5c568621de65e4... |
| grid_pglib30_opf.elecert.json | grid | acopf | CERTIFIED | cae3290639d98973... |
| lab_cardputer_csi.elecert.json | lab | cardputer_rf_csi | CERTIFIED | a2d5684a0cc00bbc... |
| lab_cardputer_power.elecert.json | lab | cardputer_power_pdn | CERTIFIED | 742b8e3d2c78ef8d... |
| lab_cardputer_thermal.elecert.json | lab | cardputer_thermal | CERTIFIED | 6c3bd15c5b725458... |
| motor_ipmsm.elecert.json | motor | ipmsm | CERTIFIED | 650bcdbc01214bbe... |
| obs_case3_pflow0.elecert.json | grid | observable_envelope | CERTIFIED | 168f73f389aa8abd... |
| obs_wb2_vm2.elecert.json | grid | observable_envelope | CERTIFIED | 51d60a7ab4fdd46c... |
| sos2_case3_lmbd.elecert.json | grid | acopf_sos2 | CERTIFIED | 168f73f389aa8abd... |
| sos2_case5_pjm.elecert.json | grid | acopf_sos2 | CERTIFIED | cadf7501a15c2d50... |
| sos2_ieee3_test.elecert.json | grid | acopf_sos2 | CERTIFIED | a8d092a42b4d624b... |
| sos2_wb2.elecert.json | grid | acopf_sos2 | CERTIFIED | 51d60a7ab4fdd46c... |
| sos2_wb3.elecert.json | grid | acopf_sos2 | CERTIFIED | 328998673bc8b774... |
| sos2_wb5.elecert.json | grid | acopf_sos2 | CERTIFIED | c094625861ddde52... |

## 8. Lean 4 Standalone Certificate Axiom Audit (`leanchecker`)

| Module | Axiom Dependencies (`0 sorryAx`, `0 ofReduceBool`) |
| --- | --- |
| CertEmMicrostrip | 'CertEmMicrostrip.passivity_psd_sound' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertEmMicrostrip.multiport_passivity_certified' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertEmMicrostrip.reciprocity_certified' depends on axioms: [propext, Quot.sound] |
| CertEmWilkinson | 'CertEmWilkinson.passivity_psd_sound' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertEmWilkinson.multiport_passivity_certified' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertEmWilkinson.reciprocity_certified' depends on axioms: [propext, Quot.sound] |
| CertEmMaxwellBound | 'CertEmMaxwellBound.maxwell_psd_sound' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertEmMaxwellBound.maxwell_poynting_bound_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertEmBodeFano | 'CertEmBodeFano.bode_fano_match_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertEmKypEnforce | 'CertEmKypEnforce.kyp_passivity_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertEmKyp3Port | 'CertEmKyp3Port.kyp_passivity_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertCktSic800v | 'CertCktSic800v.soa_corner_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertMotorIpmsm | 'CertMotorIpmsm.ipmsm_certified' does not depend on any axioms |
| CertGridIeee3Opf | 'CertGridIeee3Opf.ac_opf_psd_sound' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertGridIeee3Opf.ac_opf_bracket_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertGridChordal | 'CertGridChordal.grid_chordal_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertDiscoveredWb2Sos | 'CertDiscoveredWb2Sos.sos2_psd_sound' depends on axioms: [propext, Classical.choice, Quot.sound]; 'CertDiscoveredWb2Sos.sos2_hierarchy_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |
| CertGridIeee9Stab | 'CertGridIeee9Stab.grid_stability_certified' depends on axioms: [propext, Classical.choice, Quot.sound] |

## 9. Frontier Research Atlas (`v11.0.0`: Chordal + SOS-2 + Observables + EM Poynting/KYP)

| Frontier Case | Bound / Gap Result | Bracket / Reference | Extra Margin | Status |
| --- | --- | --- | --- | --- |
| grid:ieee3_test (3b, tw<=2) | Shor1=0.0157%, Chordal=0.0120%, SOS2=0.0157% | [3269.90, 3270.29] $/h | cliques=1, t=0.011s | PASS |
| grid:ieee9_wscc (9b, tw<=2) | Shor1=0.0278%, Chordal=0.0293%, SOS2=N/A | [5296.16, 5297.71] $/h | cliques=7, t=0.031s | PASS |
| grid:wb2 (2b, tw<=1) | Shor1=1.9485%, Chordal=1.9485%, SOS2=0.0727% | [888.08, 905.73] $/h | cliques=1, t=0.004s | PASS |
| grid:wb3 (3b, tw<=2) | Shor1=0.0174%, Chordal=0.0143%, SOS2=0.0174% | [1155.24, 1155.41] $/h | cliques=1, t=0.010s | PASS |
| grid:wb5 (5b, tw<=2) | Shor1=0.0237%, Chordal=0.0286%, SOS2=0.0237% | [2313.70, 2314.36] $/h | cliques=3, t=0.022s | PASS |
| grid:case3_lmbd (3b, tw<=2) | Shor1=0.5065%, Chordal=4.0054%, SOS2=0.1877% | [5789.86, 6031.45] $/h | cliques=1, t=0.012s | PASS |
| grid:case5_pjm (5b, tw<=2) | Shor1=5.5581%, Chordal=7.1185%, SOS2=4.3025% | [16616.30, 17889.80] $/h | cliques=3, t=0.023s | PASS |
| grid:case14_ieee (14b, tw<=2) | Shor1=0.4402%, Chordal=0.1349%, SOS2=N/A | [2175.60, 2178.54] $/h | cliques=12, t=0.064s | PASS |
| grid:case30_ieee (30b, tw<=3) | Shor1=6.7052%, Chordal=11.0239%, SOS2=N/A | [7315.90, 8222.32] $/h | cliques=26, t=0.179s | PASS |
| grid:case30_api (30b, tw<=3) | Shor1=16.2855%, Chordal=5.5488%, SOS2=N/A | [17101.60, 18106.28] $/h | cliques=26, t=0.172s | PASS |
| grid:case30_sad (30b, tw<=3) | Shor1=6.4806%, Chordal=11.0790%, SOS2=N/A | [7311.35, 8222.29] $/h | cliques=26, t=0.174s | PASS |
| grid:case57_ieee (57b, tw<=5) | Shor1=N/A, Chordal=7.2147%, SOS2=N/A | [36033.40, 38835.26] $/h | cliques=52, t=0.685s | PASS |
| grid:case118_ieee (118b, tw<=4) | Shor1=N/A, Chordal=14.2958%, SOS2=N/A | [96712.91, 112844.97] $/h | cliques=109, t=10.067s | PASS |
| obs:wb2:vm2:2 | val=0.95271 | [0.91947, 1.04449] | width=0.12502 | PASS |
| obs:case3_lmbd:pflow:0 | val=51.39298 | [48.32811, 53.37447] | width=5.04636 | PASS |
| em:maxwell_bound (M=16) | SEARU=22.776 <= Local=23.767 | Global=33.686 | ratio=0.7056 | PASS |
| em:kyp_enforce | orig=1.0350 -> rep=0.9920 | P>0, KYP_mu=279158 | rms=0.0088 | PASS |

