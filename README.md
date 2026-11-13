# ELE v6.0.0 — Self-Hosting Silicon Realization for `FLOW v6.0.0`

**Self-Hosting `FLOW v6.0.0` 2-Jet Silicon Bootstrap (`tools/ele_bootstrap.c` $\to$ Pure Synthesizable IEEE 1800 SystemVerilog): `< 160` Top-Level Physical Pins (`141` / `157`), True `$mem_v2` Synchronous 1RW 64-Bit SRAM, `33×33` Limb-Decomposed DSP Atoms, `7×` Yosys SAT `QED` Formal Proofs, Full Dual-Profile `0-ULP` 512-Bit Lockstep against Frozen `FLOW v6.0.0`, and `54 / 54` (`100.0%`) RTL Sabotage Kill Rate.**

---

## Self-Hosting Silicon Bootstrap Architecture

Instead of hand-translating C11 `int128` operators into an unroutable `2,226`-pin / `65×65` monolithic FSM with `8,192` loose flip-flops pretending to be SRAM, `ELE v6.0.0` decomposes hardware into **Primordial Physical Seed Atoms** ([`rtl/ele_primordial.sv`](rtl/ele_primordial.sv) + [`rtl/ele_em_phy.sv`](rtl/ele_em_phy.sv)) and uses **[`FLOW v6.0.0`](https://github.com/hianova/FLOW)'s own 2-Jet manifold solver (`flow_step_atlas`)** at build time ([`tools/ele_bootstrap.c`](tools/ele_bootstrap.c)) to schedule, pipeline-cut, and emit synthesizable IEEE 1800 SystemVerilog ([`rtl/ele_flow_core.sv`](rtl/ele_flow_core.sv)):

```text
  FLOW v6.0.0 (../FLOW/flow.h — Frozen Reference Pinned at ref: v6.0.0, 0 Bytes Modified)
        │
        ├──► Build-Time 2-Jet Coupled Lattice Synthesizer (tools/ele_bootstrap.c)
        │      • 33 operator nodes + Radix-2 unroll evolved as 64B flow_cell manifolds
        │      • Converges flow_step_atlas under (--dsp-count M, --max-depth D)
        │      • Emits pure IEEE 1800 SystemVerilog (rtl/ele_flow_core.sv)
        ▼
  ELE v6.0.0 Synthesizable Silicon IPs & Verification Gates (rtl/, tools/)
  ├── rtl/ele_primordial.sv        : Primordial Physical Seed Atoms
  │   ├── ele_mul33_atom           : Operand-isolated 33x33 -> 66 signed DSP slice (0 idle flips)
  │   ├── ele_rad2_step_atom       : Unified 1-subtractor radix-2 slice shared by isqrt & udiv
  │   └── ele_sram_1rw64           : True synchronous 1RW 64-bit Chart Arena SRAM ($mem_v2)
  ├── rtl/ele_em_phy.sv            : Charge-Minimal Physical Transceiver & Ternary MAC
  │   ├── ele_bus_invert65         : 65-wire Bus-Invert PHY (caps wire transitions <= 32/65)
  │   └── ele_ternary_mac_dualrail : Glitch-free dual monotonic rail 1.58-bit ternary MAC
  ├── rtl/ele_flow_core.sv         : FLOW-Synthesized 64-Bit Streaming Engine & Atlas Top
  │   ├── ele_flow_core            : 141 physical pins, 0-ULP 512-bit 2-Jet manifold core
  │   └── ele_flow_atlas_top       : 157 physical pins, 1RW SRAM arena + release-ordered @swap
  ├── rtl/ele_cosim_tb.sv          : Full dual-profile 0-ULP DPI-C wind-tunnel, analytical
  │                                  2^-120/2^-124 knife-edges, SEARU Epiphany & protocol torture
  └── tools/rtl_sabotage_check.py  : 54-mutant parallel SV single-fault RTL sabotage gate
```

---

## Measured Silicon Audit (`Yosys 0.70` + `ABC` + `ltp -noff`)

| Metric | Legacy Hand-Written SV | **Profile A (`--dsp-count 1 --max-depth 22`)** | **Profile B (`--dsp-count 3 --max-depth 36`, Canonical)** |
| :--- | :--- | :--- | :--- |
| **Top-Level Pins (`core` / `top`)** | `2,226` / `2,761` pins | **`141` / `157` pins (`-92.9%`)** | **`141` / `157` pins (`-92.9%`)** |
| **Chart Arena Storage (`16×512b`)** | `8,192` loose DFFs | **`1× $mem_v2` 1RW 64b SRAM (`0` warnings)** | **`1× $mem_v2` 1RW 64b SRAM (`0` warnings)** |
| **DSP / Radix-2 Hardware** | `1× 65×65` + 2 subtractors | `1× 33×33` DSP + `1×` Radix-2 (`K_rad=1`) | `3× 33×33` DSP + `2×` Radix-2 (`K_rad=2`) |
| **Pipeline Stages (`A/B/D/C`)** | Fixed hardcoded | `7 / 8 / 2 / 10` (`2,459` cut DFF bits) | `2 / 3 / 1 / 3` (`933` cut DFF bits) |
| **Flattened Core CMOS Cells** | `58,245` cells | **`35,480` cells (`-39.1%`)** | **`49,574` cells (`-14.9%`)** |
| **Critical Path (`ltp -noff`)** | Unbounded `65×65` cone | **`266` CMOS gate levels (`-38.4%`)** | **`432` CMOS gate levels** |
| **Step Latency (`quad / lin / idle`)** | `143 / 70 / 4` cyc | `155 / 83 / 8` cyc (`-98.5%` idle flips) | **`73 / 38 / 3` cyc (`1.96×` faster, `-98.8%` idle flips)** |

---

## Dual-Profile Silicon Wind-Tunnel, Formal `7× QED` & `100.0%` RTL Sabotage Gate

**Both** Area Profile A (`--dsp-count 1 --max-depth 22`, `155` cyc) and Canonical Profile B (`--dsp-count 3 --max-depth 36`, `73` cyc) execute the **100% full, un-shortened** X-poisoned co-simulation suite (`Verilator 5.054` `-Wall --x-assign unique --x-initial unique`, zero smoke shortcuts):

- **Probe 1 (`0-ULP` 512-Bit Differential)**: `20,480 / 20,480` vectors on Profile B + `20,480 / 20,480` vectors on Profile A (`40,960` total) bit-identical across `w0..w7` (`0` defects).
- **Probe 2 (Gauge Parity & $2^{-49}$ Fiber)**: `1,024` odd-mirror pairs (`0` cycle/value asymmetries) + `2,048`-generation autonomous sub-LSB fiber (`0-ULP` drift through `ele_sram_1rw64`) on both profiles.
- **Probe 3 (`ele_sram_1rw64` Atlas & Release-Ordered `@swap` Firewall)**: Hardware `cap_mask` slot clamping (`0xFF -> 15`), `1`-cycle firewall rejection (`0` SRAM writes on all `10` adversarial `@swap` vectors including exact `cap_mask + 1` boundaries), release-ordered `w6` commit on accepted swaps, and in-flight step immunity.
- **Probe 4 (CMOS Switching-Energy Telemetry)**:
  - `ele_flow_core` Profile B (`dsp=3`, `1,490` DFF bits): `73 cyc` (`1,781` flips) $\to$ `38 cyc` (`426` flips) $\to$ **`3 cyc` (`10` flips, `-98.8%` toggles across 16 steps: `369` vs `30,795`)**.
  - `ele_flow_core` Profile A (`dsp=1`, `3,018` DFF bits): `155 cyc` (`3,495` flips) $\to$ `83 cyc` (`989` flips) $\to$ **`8 cyc` (`20` flips, `-98.5%` toggles across 16 steps: `836` vs `58,306`)**.
  - `ele_bus_invert65`: **`max_flips = 32 / 65`**, **`-14.7%` wire toggles** (`29,435` vs `34,519`).
  - `ele_ternary_mac_dualrail`: **`0-ULP` bipolar match**, **`-8.2%` DFF toggles** (`4,655` vs `5,073`) + `0`-flip zero-skip.
- **Probe 5 (Analytical $2^{-120}$ / $2^{-124}$ Cross-Limb Knife-Edges — `3,700` Vectors / Profile)**: Ports all `3,688` analytical multipole knife-edges (`944` `isqrt` $k=2..60$ positive/negative gauge twins across `ele_mul33_atom` 33-bit limb & 64-bit carry/borrow cliffs checking `xp == 3` vs `xp == 2` + `2,700` `sym_div128_clamped` 64-stage `0.5 ULP` half-way rounding & `over_cap` saturation cliffs across $k=1..30$ + `42` `sub_valid` clamp/Schmitt/KanLock cliffs + `2` extreme 125-bit $\Delta \ge 2^{124}$ & inverted Schmitt cliffs) plus `12` exact sub-LSB Q49 convergence, isometric Lyapunov (`|G0_cand| == |G0|`), `G1 == 0, R2 < 0` gauge, and 128-bit `pre_sat` (`anum[127:64] == safe_den`) boundary vectors at `0-ULP` across both Pareto profiles.
- **Probe 6 (SEARU Zero-Memory 1-Bit `DualChaosState` + `Epiphany` Phase-Boundary Crucible)**: `4,096` closed-loop multi-chart `ele_flow_atlas_top` steps (`k=1..6` branchless SWAR XorShift `AND` cooling + Zipf singular jumps + `405` `Epiphany` phase-boundary lock/re-heat events) interleaved with `256` live `@swap` firewall checks (`142` accepted, `114` rejected) verified at `0-ULP` against `flow_step_atlas` & `flow_arena_try_swap` on both Pareto profiles.
- **Probe 7 (FSM Handshake Bubble Chaos, Busy Illegal-Write Immunity & Exhaustive Mid-Flight `rst_n` Recovery)**:
  - Irregular `0..17`-cycle `wr_valid_i` / `cmd_valid_i` backpressure bubbles with scrambled beat ordering and hostile invalid-cycle bus garbage (`64'hDEAD_BEEF_...`, `INT64_MIN`, `INT64_MAX`).
  - Active in-flight (`ready_o == 0`) hostile write & opcode hammering across `ele_flow_core` and `ele_flow_atlas_top` (`CMD_STEP` & `CMD_SWAP`), proving `100%` register & SRAM immunity (`0-ULP`).
  - Cycle-by-cycle mid-flight `rst_n = 0` abort & immediate `0-ULP` clean-slate recovery across **every** in-flight cycle offset: **`73` core (`1..73`) + `88` top (`1..80` step + `1..8` `@swap`) offsets** on Profile B (`dsp=3`), and **`155` core (`1..155`) + `170` top (`1..162` step + `1..8` `@swap`) offsets** on Profile A (`dsp=1`).
- **`7×` Yosys Formal SAT / Unbounded k-Induction `QED` (`make formal-gate`)**: `ele_bus_invert65`, `ele_ternary_mac_dualrail`, `ele_mul33_atom`, `ele_rad2_step_atom`, `ele_flow_core` (`dsp=3`), `ele_flow_core` (`dsp=1`), and `ele_flow_atlas_top` (formally proving reset safety, busy write-ignore invariance across all ingress & 512-bit state registers, `done_o -> ready_o`, SEU fault recovery, `cap_mask <= 15`, `@swap` firewall safety, and KanLock invariance).
- **Gate 6 — Adversarial SystemVerilog RTL Sabotage Audit (`make mutate-check` / `make verify`)**: `tools/rtl_sabotage_check.py` injects **`54` stratified single-fault SystemVerilog mutations** (`10` in `rtl/ele_primordial.sv` + `44` in `rtl/ele_flow_core.sv`) across isolated parallel Verilator sandboxes, requiring unique match, clean compilation (`compile_rc == 0`), and dynamic runtime kill (**`54 / 54 = 100.0%` mutant kill rate, `0` survivors**).

---

## Quick Start

Requires sibling [`FLOW`](https://github.com/hianova/FLOW) (`../FLOW` at `v6.0.0`, pinned via `ref: v6.0.0` in [`.github/workflows/ci.yml`](.github/workflows/ci.yml)), a C17 compiler (`clang`/`gcc`), `verilator` ($\ge 5.0$), `yosys`, and `python3`:

```sh
# Fast 5-gate sign-off (Hygiene, Bootstrap, Full Dual-Profile Co-Sim, 7x SAT QED, Synthesis/Pin/SRAM Audit)
make clean && make test

# Full 6-gate sign-off including 54-mutant SystemVerilog RTL sabotage kill-rate audit (100.0%)
make clean && make verify
```

---

## License

Released under the [MIT License](LICENSE).
