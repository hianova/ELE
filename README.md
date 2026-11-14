# ELE v7.0.1 — Self-Hosting `SEARU × FLOW` Silicon Realization

**Self-Hosting `SEARU × FLOW v7.0.1` Symbiotic 2-Jet Silicon Bootstrap (`tools/ele_bootstrap.c` $\to$ Pure Synthesizable IEEE 1800 SystemVerilog): Pure-GPR C17 Co-Synthesis (`flow_step_atlas` Continuous Retiming $\times$ `SEARU_SOLVE_INLINE` DSP Operand-Affinity Binding & Mux-Aware Register Coloring), `-127` Reclaimed Core/Atlas Sequential DFFs, Fine-Grained Regime Clock Gating & Coherent Chart-Bound Caching (`0` Steady-State SRAM Sense-Amp Reads), 3-Tier Physical Hierarchy (`42`-Pin QFN-48 ASIC Chip / `141`-Pin Core Macro / `157`-Pin 64b NoC Macro), Dual `ele_bus_invert17` Charge-Minimal Pad PHY (`<= 8/17` SSO Cap + Idle CSA Isolation), True `$mem_v2` Synchronous 1RW 64-Bit SRAM with Shadow-SRAM Zero-`cand_buf_q` `@swap`, `33×33` Limb-Decomposed DSP Atoms, `9×` Yosys SAT `QED` Formal Proofs, `64 / 64` (`100.0%`) RTL Sabotage Kill Rate, and Full Dual-Profile `0-ULP` 512-Bit Lockstep (including Live `searu_heal_and_swap` $\Delta < 0$ Barrier Healing) against Frozen `FLOW`.**

---

## Self-Hosting `SEARU × FLOW 2-Jet` Symbiotic Silicon Bootstrap & 3-Tier Physical Hierarchy

Instead of hand-translating C17 `int128` operators into an unroutable `2,226`-pin / `2,761`-pin / `65×65` monolithic FSM with `8,192` loose flip-flops pretending to be SRAM — or relying on greedy hand-coded HLS heuristics — `ELE v7.0.1` decomposes hardware into **Primordial Physical Seed Atoms** ([`rtl/ele_primordial.sv`](rtl/ele_primordial.sv) + [`rtl/ele_em_phy.sv`](rtl/ele_em_phy.sv)) and natively couples **[`FLOW`](https://github.com/hianova/FLOW)'s continuous 2-Jet manifold solver (`flow_step_atlas` in `../FLOW/flow.h`) with discrete `SEARU` phase-boundary synthesis (`SEARU_SOLVE_INLINE` in `../FLOW/searu.h` + `../FLOW/flow_opt_params.h`)** inside a pure-GPR (`-std=c17 -mgeneral-regs-only`, zero-float) build-time compiler ([`tools/ele_bootstrap.c`](tools/ele_bootstrap.c)) to schedule, bind, color, clock-gate, and emit synthesizable IEEE 1800 SystemVerilog ([`rtl/ele_flow_core.sv`](rtl/ele_flow_core.sv)):

```text
  FLOW + SEARU (../FLOW/{flow.h, searu.h, flow_opt_params.h} — Frozen v7.0.1 Reference, 0 Bytes Modified)
        │
        ├──► Build-Time SEARU × FLOW 2-Jet Symbiotic HLS Compiler (tools/ele_bootstrap.c, -mgeneral-regs-only)
        │      • Continuous Retiming  : Converges flow_step_atlas on SSA DAG under (--dsp-count M, --max-depth D)
        │      • Pass 1 (DSP Binding) : SEARU_SOLVE_INLINE searches per-stage dsp_slot permutations across
        │                               Cones A/B/C to minimize shared cut bits + unique 33b DSP operands + mux fan-in
        │      • Pass 2 (Cut Coloring): SEARU_SOLVE_INLINE refines heterogeneous-width live-range lane bindings
        │                               to minimize DFF bits + writer-mux fan-in with O(N^2) 0-overlap proof
        │      • Pass 3 (Power & DFF) : Folds d_mag into sr_acc_q (-64 DFFs), promotes op_mask/r2 to zero-DFF
        │                               invariants (-33 DFFs), gates Cone A/C/D regime clocks & DSP enables,
        │                               and emits coherent chart-bound caching (-30 Atlas DFFs, 0 steady SRAM reads)
        │      • Emits pure IEEE 1800 SystemVerilog (rtl/ele_flow_core.sv)
        ▼
  ELE v7.0.1 Synthesizable Silicon IPs & Verification Gates (rtl/, tools/)
  ├── rtl/ele_primordial.sv        : Primordial Physical Seed Atoms
  │   ├── ele_mul33_atom           : Operand-isolated 33x33 -> 66 signed DSP slice (0 idle flips)
  │   ├── ele_rad2_step_atom       : Unified 1-subtractor radix-2 slice shared by isqrt & udiv
  │   └── ele_sram_1rw64           : True synchronous 1RW 64-bit Chart Arena SRAM ($mem_v2)
  ├── rtl/ele_em_phy.sv            : Charge-Minimal Pad PHY, Ternary MAC & QFN-48 ASIC Top
  │   ├── ele_bus_invert65         : 65-wire on-die Bus-Invert PHY (<= 32/65 flips + idle CSA isolation)
  │   ├── ele_bus_invert17         : 17-wire narrow ASIC pad PHY (<= 8/17 SSO + idle CSA isolation)
  │   ├── ele_ternary_mac_dualrail : Glitch-free dual monotonic rail 1.58b ternary MAC + negator gating
  │   └── ele_flow_qfn_top         : Tier 3 — 42-pin 6x6 mm QFN-48 Charge-Minimal ASIC Chip
  ├── rtl/ele_flow_core.sv         : SEARU × FLOW Synthesized 64-Bit Streaming Engine & Atlas NoC Top
  │   ├── ele_flow_core            : Tier 1 — 141-pin 0-ULP 512-bit 2-Jet manifold core macro
  │   └── ele_flow_atlas_top       : Tier 2 — 157-pin 64b NoC macro + Bound-Cache + Shadow-SRAM @swap
  ├── rtl/ele_cosim_tb.sv          : Full 8-probe dual-profile 0-ULP DPI-C wind-tunnel, QFN-48 pad-SSO
  │                                  telemetry, 2^-120/2^-124 knife-edges & live searu_heal_and_swap
  └── tools/rtl_sabotage_check.py  : 64-mutant parallel SV single-fault RTL sabotage & kill-rate gate
```

### 3-Tier Physical Packaging & Pinout Hierarchy

| Hierarchy Tier | Module | Physical Pins | Target Integration & I/O Architecture |
| :--- | :--- | :--- | :--- |
| **Tier 1 (Core Macro)** | `ele_flow_core` | **`141` pins** | Hardened 64-bit streaming 2-Jet manifold compute block (`0` SRAM) |
| **Tier 2 (On-Die NoC)** | `ele_flow_atlas_top` | **`157` pins** | Wide 64-bit SoC/FPGA macro with `1× $mem_v2` 1RW SRAM + coherent chart-bound cache + Shadow-SRAM `@swap` |
| **Tier 3 (Discrete ASIC)** | **`ele_flow_qfn_top`** | **`42` pins** | **`6×6 mm` QFN-48 chip** (`42` signal + `6` VDD/VSS pins, **`-73.2%`** vs `157`, **`-98.5%`** vs `2,761`) with live 17-wire Bus-Invert Rx/Tx PHY (`<= 8/17` SSO) |

### Silicon Craftsmanship & Dynamic Power ($C V^2 f \alpha$) Architecture (`v7.0.1`)

- **`-127` Physical Flip-Flops Reclaimed (`-97` Core + `-30` Atlas Wrapper)**:
  - **`d_mag` Folded into `sr_acc_q` (`-64` DFFs)**: `isqrt` rounds its final 64-bit root directly into `sr_acc_q`, which holds `d_mag` across Cone D until `sym_div128_clamped` overwrites `sr_acc_q` with `init_quot`.
  - **Combinational Step Invariants (`-33` DFFs)**: `op_mask_q` (`1b`) and `r2_q` (`32b`) are derived directly from persistent `b1_q`, `b2_q`, and `state_w_q[2]` (which never mutate until the final commit beat), eliminating `33` redundant DFFs with zero critical-path penalty.
  - **Atlas Wrapper Register Compaction & Coherent Chart-Bound Cache (`-30` Net DFFs, `-100%` Steady-State SRAM Sense-Amp Reads)**: Eliminates `16`-bit `mode_hold_q` (forwarding `{cmd_aux_i, cap_mask_q}` directly into `u_core` on `CMD_STEP`), pre-clamps `u_ext_hold_q` from `64b` to `32b`, and tracks the resident chart slot (`bound_cache_valid_q` + `bound_cache_slot_q`) with automatic hardware invalidation on matching `CMD_WRITE_CHART` / `CMD_SWAP` commits — skipping all `6 × 64b` SRAM reads (`sram_ce_c = 0`) and saving `6` clock cycles per repeated-chart step.
- **Zero Spurious Datapath & PHY Switching**:
  - **Regime-Aware Clock & DSP Isolation**: Gating `a2_x_sq` (`b2 != 0`), `c1_dsq` (`r2 != 0`), `a7/a8` (`op_mask_q`), and `a9` (`~op_mask_q`) eliminates quadratic multiplier and cut-lane switching on linear/autonomous/converged steps (`0` datapath DFF toggles on steady converged steps; `-99.2%` / `-98.9%` idle vs active core flips).
  - **128-Bit `sr_aux_q` Upper-Half & `pre_sat` Divider Gating**: `sym_div128_clamped` initialization updates only `sr_aux_q[63:0]` (never clobbering `sr_aux_q[127:64]`) and completely gates `sr_rem_q`, `sr_acc_q`, and `sr_aux_q` writes whenever `pre_sat` triggers (`anum_hi >= den`).
  - **Idle PHY CSA & Negator Operand Isolation**: `ele_bus_invert65` and `ele_bus_invert17` isolate their 65-bit / 17-bit XOR-popcount CSA trees to `0` when `!valid_i`, and `ele_ternary_mac_dualrail` gates its 32-bit two's-complement magnitude negator when `!valid_i || !trit_nz_i`.

---

## Measured Silicon Audit (`Yosys 0.70` + `ABC` + `ltp -noff`)

| Metric | Legacy Hand-Written SV | **Profile A (`--dsp-count 1 --max-depth 22`)** | **Profile B (`--dsp-count 3 --max-depth 36`, Canonical)** |
| :--- | :--- | :--- | :--- |
| **Physical Pins (`qfn` / `core` / `atlas`)** | `2,761` / `2,226` / `2,761` | **`42` / `141` / `157` (`-98.5%` QFN-48)** | **`42` / `141` / `157` (`-98.5%` QFN-48)** |
| **Chart Arena Storage (`1RW SRAM`)** | `8,192` loose DFFs | **`1× $mem_v2` 1RW 64b SRAM (`0` steady reads on cache hit)** | **`1× $mem_v2` 1RW 64b SRAM (`0` steady reads on cache hit)** |
| **External Pad SSO Noise (`qfn_top`)** | Unbounded `100%` simultaneous flips | **`<= 8 / 17` wires (`< 50%` SSO, `-20.6%` pad toggles)** | **`<= 8 / 17` wires (`< 50%` SSO, `-22.7%` pad toggles)** |
| **DSP / Radix-2 Hardware** | `1× 65×65` + 2 subtractors | `1× 33×33` DSP + `1×` Radix-2 (`K_rad=1`) | `3× 33×33` DSP + `2×` Radix-2 (`K_rad=2`) |
| **Pipeline-Cut Bits (`VIRTUAL -> SHARED`)** | Unshared (`2,459` / `933` bits) | **`429` bits (`-82.6%` SEARU colored, `955b` mux)** | **`262` bits (`-71.9%` SEARU colored, `33` uniq DSP ops)** |
| **Internal Core Datapath DFF Bits** | Unshared (`> 2,500` bits) | **`925` bits (`-97` reclaimed DFFs)** | **`756` bits (`-97` reclaimed DFFs)** |
| **Synthesized Physical DFFs (`core` / `atlas`)** | `3,674` / `520` DFFs (+ `8,192` mem) | **`1,782` (`-51.5%`) / `131` (`-74.8%`, `0` `cand_buf_q`)** | **`1,593` (`-56.6%`) / `131` (`-74.8%`, `0` `cand_buf_q`)** |
| **Flattened Core CMOS Cells** | `58,245` cells | **`33,277` cells (`-42.9%` vs legacy)** | **`47,954` cells (`-17.7%` vs legacy)** |
| **Critical Path (`ltp -noff`)** | Unbounded `65×65` cone | **`237` CMOS gate levels (`-40.2%` vs Profile B)** | **`396` CMOS gate levels** |
| **Step Latency (`quad / lin / idle`)** | `143 / 70 / 4` cyc | `155 / 83 / 8` cyc (`20` idle flips, **`-99.2%`**) | **`73 / 38 / 3` cyc (`10` idle flips, **`-98.9%`**)** |

---

## Dual-Profile 8-Probe Wind-Tunnel, Formal `9× QED` & `100.0%` RTL Sabotage Gate

**Both** Area Profile A (`--dsp-count 1 --max-depth 22`) and Canonical Profile B (`--dsp-count 3 --max-depth 36`) execute the **100% full, un-shortened** 8-probe X-poisoned co-simulation suite (`Verilator 5.054` `-Wall --x-assign unique --x-initial unique`, zero smoke shortcuts):

- **Probe 1 (`0-ULP` 512-Bit Differential)**: `20,480 / 20,480` vectors on Profile B + `20,480 / 20,480` vectors on Profile A (`40,960` total) bit-identical across `w0..w7` (`0` defects).
- **Probe 2 (Gauge Parity & $2^{-49}$ Fiber)**: `1,024` odd-mirror pairs (`0` cycle/value asymmetries) + `2,048`-generation autonomous sub-LSB fiber (`0-ULP` drift through `ele_sram_1rw64`) on both profiles.
- **Probe 3 (`ele_sram_1rw64` Atlas, Coherent Chart-Bound Cache & Release-Ordered `@swap` Firewall)**: Hardware `cap_mask` slot clamping (`0xFF -> 15`), firewall rejection (`0` target SRAM writes on all `10` adversarial `@swap` vectors including exact `cap_mask + 1` boundaries), release-ordered `w6` Shadow-SRAM zero-`cand_buf_q` `@swap` commit, active-slot `bound_cache` coherence across live `CMD_WRITE_CHART` and `CMD_SWAP` overwrites, and in-flight step immunity.
- **Probe 4 (CMOS Switching-Energy Telemetry)**:
  - `ele_flow_core` tickless convergence + regime clock gating cuts active-vs-converged DFF bit flips by **`-99.2%`** on Profile A (`531` vs `68,176`, `20` flips/converged step) and **`-98.9%`** on Profile B (`355` vs `35,191`, `10` flips/converged step — `0` datapath flips).
  - `ele_bus_invert65`: **`max_flips = 32 / 65`**, **`-14.7%` wire toggles** (`29,435` vs `34,519`).
  - `ele_ternary_mac_dualrail`: **`0-ULP` bipolar match**, **`-8.2%` DFF toggles** (`4,655` vs `5,073`) + `0`-flip zero-skip.
- **Probe 5 (Analytical $2^{-120}$ / $2^{-124}$ Cross-Limb Knife-Edges — `3,700` Vectors / Profile)**: `3,688` analytical multipole knife-edges (`944` `isqrt` $k=2..60$ positive/negative gauge twins across `ele_mul33_atom` 33-bit limb & 64-bit carry/borrow cliffs + `2,700` `sym_div128_clamped` 64-stage `0.5 ULP` half-way rounding & `over_cap` cliffs + `42` `sub_valid`/Schmitt/KanLock cliffs + `2` extreme 125-bit $\Delta \ge 2^{124}$ & inverted Schmitt cliffs) plus `12` exact Q49 convergence, isometric Lyapunov, `G1 == 0, R2 < 0` gauge, and 128-bit `pre_sat` boundary vectors at `0-ULP`.
- **Probe 6 (SEARU Zero-Memory 1-Bit `DualChaosState` + `Epiphany` Crucible + Live `searu_heal_and_swap` Hardware Barrier Healing)**:
  - `4,096` closed-loop multi-chart `ele_flow_atlas_top` steps (`k=1..6` branchless SWAR XorShift `AND` cooling + Zipf singular jumps + `405` `Epiphany` phase-boundary lock/re-heat events) interleaved with `256` live `@swap` firewall checks (`142` accepted, `114` rejected) verified at `0-ULP` against `flow_step_atlas` & `flow_arena_try_swap`.
  - **Live `searu_heal_and_swap` $\Delta < 0$ Barrier Self-Healing**: Loads an unreachable discriminant-barrier chart (`b0 = 1,600,000, b1 = 0, b2 = 2 * 65536`) into `ele_flow_atlas_top`, verifies `0-ULP` hardware stall (`!CONVERGED && dt == 0`), invokes `searu.h`'s `searu_heal_and_swap` via DPI-C (`ele_dpi_searu_heal`) with a 24-term `searu_bilevel_term` actuator bank, commits the synthesized chart via live hardware `CMD_SWAP` (`top_try_swap`), and verifies `0-ULP` 512-bit hardware convergence to the exact target root (`CONVERGED == 1`, `x == 241,664`) plus pre/post-heal phase-gate no-op invariance.
- **Probe 7 (FSM Handshake Bubble Chaos, Busy Illegal-Write Immunity & Exhaustive Mid-Flight `rst_n` Recovery)**:
  - Irregular `0..17`-cycle `wr_valid_i` / `cmd_valid_i` backpressure bubbles with scrambled beat ordering and hostile invalid-cycle bus garbage.
  - Active in-flight (`ready_o == 0`) hostile write & opcode hammering across `ele_flow_core` and `ele_flow_atlas_top` (`CMD_STEP` & `CMD_SWAP`), proving `100%` register & SRAM immunity (`0-ULP`).
  - Cycle-by-cycle mid-flight `rst_n = 0` abort & immediate `0-ULP` clean-slate recovery across **every** in-flight cycle offset (`73`/`155` core + `94`/`176` top offsets) and dynamically measured accepted `@swap` commits.
- **Probe 8 (`42`-Pin QFN-48 Charge-Minimal ASIC Padframe Lockstep & 17-Wire Pad-SSO Telemetry)**:
  - Executes `256` multi-chart quadratic/linear/autonomous/Citardauq-tunnel steps (including full lossless 16-bit `mode_req` via `SET_AUX_HI` and busy flit-ignore hammering during both core solve and 4-beat Tx readout bursts) interleaved with `32` valid & adversarial `@swap` transactions (`12` accepted, `20` rejected) through `ele_flow_qfn_top` (`42` pins) at **`0-ULP` 512-bit lockstep** against `flow_step_atlas` & `ele_dpi_try_swap`.
  - Continuously verifies **`max_rx_flips = 8 / 17` (`<= 8`)** and **`max_tx_flips = 8 / 17` (`<= 8`)** (`< 50%` Simultaneous Switching Output pad noise ceiling on every single clock cycle), cutting total external QFN-48 pad toggles by **`-22.7%`** (`33,144` vs `42,889` on Profile B) and standalone `ele_bus_invert17` wire toggles by **`-25.6%`** (`6,880` vs `9,257`).
- **`9×` Yosys Formal SAT / Unbounded k-Induction `QED` (`make formal-gate`)**: `ele_bus_invert65`, `ele_bus_invert17`, `ele_ternary_mac_dualrail`, `ele_mul33_atom`, `ele_rad2_step_atom`, `ele_flow_core` (`dsp=3`), `ele_flow_core` (`dsp=1`), `ele_flow_atlas_top`, and **`ele_flow_qfn_top`** (formally proving 42-pin QFN-48 reset safety, coherent chart-bound cache SRAM idle gating, unconditional busy flit-ignore invariance, 4-beat Tx readout burst sequencing, `<= 8/17` external pad SSO ceiling, `done_o -> ready_o`, SEU fault recovery, `cap_mask <= 15`, `@swap` firewall safety, and KanLock invariance).
- **Gate 6 — Adversarial SystemVerilog RTL Sabotage Audit (`make mutate-check` / `make verify`)**: `tools/rtl_sabotage_check.py` injects `64` stratified single-fault SystemVerilog mutations (`8` QFN-48/PHY + `10` Primordial + `46` Core/Atlas) across isolated parallel Verilator sandboxes, requiring unique match, clean compilation (`compile_rc == 0`), and dynamic runtime kill (**`64 / 64 = 100.0%` mutant kill rate, `0` survivors**).

---

## Quick Start

Requires sibling [`FLOW`](https://github.com/hianova/FLOW) (`../FLOW` at `v7.0.1`, pinned via `ref: v7.0.1` in [`.github/workflows/ci.yml`](.github/workflows/ci.yml)), a C17 compiler (`clang`/`gcc`), `verilator` ($\ge 5.0$), `yosys`, and `python3`:

```sh
# Fast 5-gate sign-off (Hygiene, Bootstrap, Full 8-Probe Dual-Profile Co-Sim, 9x SAT QED, 42-Pin QFN/SRAM Audit)
make clean && make test

# Full 6-gate sign-off including parallel SystemVerilog RTL sabotage kill-rate audit (64/64 = 100.0%)
make clean && make verify
```

---

## License

Released under the [MIT License](LICENSE).
