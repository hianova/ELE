# ELE v2.0.0 — Verified Electronics, Electrical Power & Electromagnetic Silicon Layer

**Pure Lean 4 (`0 sorryAx`, `0` `.c`/`.h`/`.cpp` files) + Synthesizable IEEE 1800 SystemVerilog IPs (`0` latches, `0-ULP` 512-bit silicon co-verification) atop `FLOW v5.0.0` & `UNIT v2.0.0`.**

---

## Architecture

`ELE v2.0.0` eliminates legacy ad-hoc C firmware/header duplication and bespoke per-feature RTL state machines, unifying **電子 (Electronics)**, **電機 (Electrical Power)**, and **電磁 (Electromagnetics & Photonics)** on the `FLOW` $\to$ `UNIT` $\to$ `ELE` stack:

```text
  Ele.lean (import Unit — 0 sorryAx, 0 .c/.h/.cpp)
  ├── Ele.Electron  : <1ns AD-FLL zero-SDC envelope, C*V_dd^2 guardband savings,
  │                   LC Q>pi resonant clock, level-crossing ADC + Vernier TDC,
  │                   NBTI aging canary, Kelvin contact probe, `fllAtlas` ("EFLL")
  ├── Ele.Power     : 800V SiC SSCB 2-Jet RC-inrush vs short-circuit separation,
  │                   Miller soft turn-off V_ds clamp, 48V VRM phase-shedding / ZVS /
  │                   active pre-charge, 180-deg rack interleaving, 800V HVDC,
  │                   GFM synthetic inertia, CHF & Q^3 pump laws, `sscbAtlas` & `gfmAtlas`
  ├── Ele.MagWave   : Datasheet regRmw8 (`bv_decide`), 65-wire Bus-Invert (<=32/65 flips),
  │                   Single-Edge PPM, waveguide Z_0 drive, 2-Jet Zero-Bit Tx/Rx lock,
  │                   dual-rail 1.58-bit ternary MAC (`0-ULP`), BSPDN active damping,
  │                   2-cycle zero-drift spread-spectrum EMI, CPO Modulo-FSR, `cpoAtlas`
  └── Ele.Silicon   : `.flow` -> `.memh` FPGA/ASIC ROM emitter + 3-way `0-ULP`
                      512-bit (`w0..w7`) differential co-verifier
                              │
             ┌────────────────┴────────────────┐
             ▼                                 ▼
  Host / Edge CPU (`FLOW/build/flow`)   Synthesizable Silicon (`rtl/*.sv`)
  64B `.flow` Manifold Atlas binary     ├── `rtl/ele_flow_core.sv`
  + UTF-8 wire sieve (`UNIT`)           │     Universal silicon `.flow` engine
                                        │     (`ele_flow_core` + `ele_flow_atlas_top`
                                        │      with live guarded `@swap` & F_p sponge)
                                        └── `rtl/ele_em_phy.sv`
                                              Charge-minimal physical leaf IPs
                                              (`ele_bus_invert65` &
                                               `ele_ternary_mac_dualrail`)
```

---

## Reusable Synthesizable SystemVerilog IPs (`rtl/`)

Downstream FPGA/ASIC designers instantiate `ELE`'s hardware modules directly without reinventing RTL or writing C glue:

1. **`rtl/ele_flow_core.sv`**:
   - `ele_flow_core`: Cycle-accurate, `0-ULP` bit-identical silicon realization of `FLOW v5.0.0`'s `flow_step_chart` (`512`-bit `State` $\times$ `512`-bit `Chart` $\to$ `512`-bit `State`). Time-multiplexes **one** shared $64\times 64 \to 128$ unsigned multiplier across both the Goldilocks $\mathbb{F}_p = 2^{64}-2^{32}+1$ $x^7$ cryptographic sponge (`FLOWSTA5` / `FLOWBND5`) and signed `Q48.16` 2-Jet algebra, paired with a 32-step `isqrt_u64` unit and a 64-step restoring divider (`0` lookup tables, `0` combinational divider arrays, `0` FPU).
   - `ele_flow_atlas_top`: Power-of-two $2^k$ Chart Arena controller supporting cycle-accurate `STEP` execution and live guarded `@swap` (`flow_arena_try_swap` univalence domain tag, monotonic epoch, Schmitt hysteresis deadband, and Goldilocks seal firewall).
   - Any domain `.flow` Manifold Atlas minted in Lean (`fllAtlas`, `sscbAtlas`, `gfmAtlas`, `cpoAtlas`, or custom user Atlases) converts directly to an FPGA/ASIC `$readmemh` image via `Ele.Silicon.emitBundleMemh`.
2. **`rtl/ele_em_phy.sv`**:
   - `ele_bus_invert65`: Stateful 65-wire (`64` data + `1` polarity) charge-minimal transceiver strictly capping wire transitions at $\le 32 / 65$ ($< 50\%$ Maxwell displacement current $C \frac{dV}{dt}$).
   - `ele_ternary_mac_dualrail`: Glitch-free dual monotonic magnitude-rail `1.58-bit` balanced ternary MAC (`0-ULP` equivalent to signed accumulation without two's-complement sign-extension toggle storms).

---

## Repository Layout

```text
ELE/
├── Ele.lean             # L2 EEE theorems, Manifold Atlases & 3-way silicon co-verifier
├── TestEle.lean         # End-to-end EEE lifecycle + Verilator/FLOW 512-bit co-sim suite
├── rtl/
│   ├── ele_flow_core.sv # Universal synthesizable `.flow` silicon engine + `@swap` top
│   ├── ele_em_phy.sv    # Charge-minimal 65-wire Bus-Invert & dual-rail ternary MAC
│   └── ele_cosim_tb.sv  # Pure-SV differential testbench (`verilator --binary`, 0 .cpp)
├── Makefile             # 4-gate Lean + FLOW + Verilator + Yosys verification pipeline
└── VERSION              # 2.0.0
```

---

## Quick Start & Verification Gates

Requires sibling [`FLOW`](../FLOW) (`v5.0.0`), [`UNIT`](../UNIT) (`v2.0.0`), `lean` (Lean 4), `verilator` ($\ge 5.0$), and `yosys`:

```sh
make clean && make test
```

`make test` enforces all 4 hardware/formal gates:
- **Gate 1 (Formal Proof & Zero-C Gate)**: Compiles `Ele.lean` (`-DwarningAsError=true`), verifies `#print axioms` (`0 sorryAx`), and asserts `0` `.c`/`.h`/`.cpp` files in the repository.
- **Gate 2 (Strict RTL Lint & Pure-SV Build)**: Passes `verilator --lint-only -Wall` across all RTL modules and builds `build/verilator/Vele_cosim_tb` directly via `verilator --binary -Wall`.
- **Gate 3 (3-Way `0-ULP` 512-Bit Differential Co-Verification)**: Verifies `ele_em_phy.sv` over 256 hardware vectors and asserts bit-for-bit equality across all **512 bits (`w0..w7`, including Goldilocks $\mathbb{F}_p$ seals)** between `Flow.lean`/`Ele.Silicon`, `FLOW/build/flow` (C99), and `rtl/ele_flow_core.sv` (Verilator) across `EFLL`, `SSCB`, `EGFM`, `ECPO`, autonomous 2-Jet fiber pushforward, stationary instanton curvature tunneling, and live `@swap` hot-reloads.
- **Gate 4 (Yosys CMOS Gate-Level Synthesis & Zero-Latch DRC)**: Synthesizes `ele_flow_core`, `ele_bus_invert65`, and `ele_ternary_mac_dualrail` with `read_verilog -nolatches -sv`, `check -assert`, and `select -assert-none t:$_DLATCH* t:$dlatch*`.
