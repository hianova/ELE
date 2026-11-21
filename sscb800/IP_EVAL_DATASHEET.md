# `ele_sscb800_ip` — AI 800V DC Rack SiC Solid-State Circuit Breaker Control IP Datasheet

> **IP Revision ID:** `0xE1E80001` | **Interface:** 100 MHz FPGA Clock / 20 MSPS (`50 ns`) ADC (`0.25 A/LSB`) + Hardware DESAT Bypass + 32-bit BMC/PMBus CSR + 32-Entry Freeze Blackbox | **Zero Crypto / Zero Regulatory Cert Bloat**

---

## 1. Key Selling Points for 800V HVDC AI Power Rack Vendors (Delta / Lite-On / ODM)

1. **Sub-Microsecond Bolted Short Interrupt:** Trips `200 A/µs` 800V bus shorts in **`150.0 ns`** past nominal, holding peak fault current at **`440.0 A`** (vs `975.5 A` on `900 A + 400 ns` comparators) — **`25.4×` lower let-through $\int I^2 dt$ energy** (`0.0590` vs `1.4972 A^2·s`).
2. **Zero Capacitive Hot-Swap Nuisance Trips:** Rides cleanly through `1.0 mF / ΔV=280 V` sled hot-swap surges over `320 A` load (`30,000` samples / `1.5 ms`), peaking at **`599.2 A`** with **`658.0 A/µs`** peak rising slope (>3× the bolted short slope; `14` samples above both `I_nom` and `didt_thresh`): a naive fixed-`dI/dt` comparator **FALSE-TRIPS (`1`)** while `ele_sscb800_ip` logs **`0` false trips** via negative concavity ($d^2I/dt^2 < 0$).
3. **Hardware EMI Spike & Thermal Drift Immunity:** Integrated `ele_huber_median3_atom` (`Theorem 12`) absorbs **`50 / 50`** isolated `+1,800 A` SiC switching spikes with **`0` false trips** (`50 / 50` trips when bypassed), while `AUTOZERO` CSR calibration removes **`+35 A` (`140 LSB`)** Hall sensor drift cleanly.
4. **Mathematically Proven Safe by Unbounded $k$-Induction (Yosys QED):** Every release proves `0`-cycle hardware DESAT bypass, Theorem 12 single-spike ceiling, sticky overcurrent latch, and freeze-on-trip 32-entry Blackbox preservation across unbounded clock depth.

---

## 2. Measured FPGA Footprint (Yosys 0.70 Target Synthesis)

Fits comfortably in low-cost Lattice iCE40 or Xilinx Artix-7 FPGAs for instant oscilloscope evaluation or rack power-shelf integration (zero latches, zero external SRAM needed):

| Target FPGA Family | LUTs | Flip-Flops | DSP Blocks | Raw Synthesis Total Cells |
| :--- | ---: | ---: | ---: | ---: |
| **Lattice iCE40 (`synth_ice40`)** | `2395` (`SB_LUT4`) | `1314` (`SB_DFF*`) | `0` | `4073` |
| **Xilinx 7-Series (`synth_xilinx`)** | `1392` (`LUT*`) | `1314` (`FD*`) | `4` (`DSP48E1`) | `2905` |

---

## 3. Verilator 7-Scenario 800V AI-Rack Wind-Tunnel Results

| Scenario | Physical Stress | Measured Outcome | Status |
| :--- | :--- | :--- | :---: |
| **1. Bolted 800V Short** | `200 A/µs` (`4 µH` bus) from `380 A` rack load | Trips in **`150.0 ns`** past `400 A`, `I_peak=440.0 A`, **`25.4×` less $\int I^2 dt$** | **PASS** |
| **2. Capacitive Hot-Swap** | `520V->800V` (`ΔV=280V`), `C=1.0 mF, R=1.0 Ω, L=0.40 µH` over `320 A` (`30,000` samples / `1.5 ms`) | Peak `599.2 A`, max `658.0 A/µs` (`14` samples > `I_nom` & `didt_thresh`): naive `dI/dt` **FALSE-TRIPS (`1`)** vs **`0`** on `ele_sscb800_ip` ($d^2I/dt^2 < 0$) | **PASS** |
| **3. 50 V/ns SiC EMI Spikes** | `50×` isolated `+1800 A` bursts over `300 A` | Guard ON: **`0/50`** trips (`50` logged); Guard OFF: `50/50` | **PASS** |
| **4. 85°C Hall Zero-Drift** | `+35 A` (`140 LSB`) sensor offset | Uncal rearm `REJECTED` -> `AUTOZERO(140)` -> `REARMED` | **PASS** |
| **5. Sustained 1.6× Overload** | Continuous `640 A` above `400 A` nominal | Adiabatic $I^2 t$ thermal trip at tick `34` (`cause=3'b100`) | **PASS** |
| **6. Gate-Driver DESAT Bypass** | `desat_fault_i = 1` mid-load | `gate_en_o = 0` in **`0` clock cycles** + sticky `desat_latched` | **PASS** |
| **7. Fault Blackbox Readout** | Read `BB_RD_IDX` `0..31` after Scenario 1 | **`32/32`** pre-trip/trip words frozen & bit-exact | **PASS** |

---

## 4. BMC / PMBus CSR Register Map (`ele_sscb800_ip`)

| Offset | Name | Access | Field Summary |
| :--- | :--- | :---: | :--- |
| `0x00` | `IP_ID` | `RO` | `0xE1E80001` (`ELE` 800V SSCB v1) |
| `0x04` | `CTRL` | `RW` | `[0]` armed, `[1]` emi_guard_en, `[2]` autozero_pulse (W1P when open), `[3]` rearm_pulse |
| `0x08` | `CFG_CURR` | `RW` | `[15:0]` `i_nom` (`0.25 A/LSB`), `[31:16]` `i_hard_max` |
| `0x0C` | `CFG_JET2` | `RW` | `[15:0]` `didt_thresh`, `[31:16]` `d2_concave_tol` |
| `0x10` | `CFG_MISC` | `RW` | `[2:0]` `confirm_len`, `[18:3]` `spike_tol`, `[26:19]` `rearm_band` |
| `0x14` | `CFG_I2T_LIM` | `RW` | `[31:0]` adiabatic thermal $I^2 t$ trip limit |
| `0x18` | `STATUS` | `RO` | `[0]` gate_en, `[1]` trip, `[4:2]` cause, `[5]` desat, `[15:8]` emi_rej_cnt, `[31:16]` `i_offset` |
| `0x1C` | `TELEM_PEAK` | `RO` | `[15:0]` peak current, `[20:16]` `bb_wptr`, `[31:24]` `bb_count` (`0..32`) |
| `0x20` | `TELEM_I2T` | `RO` | `[31:0]` live adiabatic $I^2 t$ thermal accumulator |
| `0x24` | `BB_RD_IDX` | `RW` | `[4:0]` chronological Blackbox readout index (`0` = oldest, `31` = newest) |
| `0x28` | `BB_RD_DATA` | `RO` | Frozen `{i_raw[15:0], i_corr[15:0]}` waveform word |
