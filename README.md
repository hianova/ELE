# ELE v6.0.0 — Pure SystemVerilog Silicon Realization for `FLOW v6.0.0`

**100% Pure Synthesizable IEEE 1800 SystemVerilog (`0` `.lean` files, `0` `.c`/`.h`/`.cpp` files inside `ELE`, `0` `UNIT` dependency, `0` latches, `0-ULP` 512-bit silicon lockstep against frozen `FLOW v6.0.0`).**

---

## Architecture

`ELE v6.0.0` is the L0/L1 physical silicon landing node for [`FLOW v6.0.0`](../FLOW), decoupling hardware synthesis completely from L2 host text/unit tooling and realizing the exact 64-byte (`512`-bit) `FLOW v6.0.0` 2-Jet Manifold state machine alongside charge-minimal physical leaf IPs:

```text
  FLOW v6.0.0 (../FLOW/flow.h — Golden Frozen Reference)
        │
        │  IEEE 1800 DPI-C Direct 512-Bit Co-Verification (0 C/C++ glue files)
        ▼
  ELE v6.0.0 (Pure IEEE 1800 SystemVerilog)
  ├── rtl/ele_flow_core.sv
  │   ├── ele_flow_core      : 0-ULP 512-bit silicon engine for FLOW v6.0.0
  │   │                        • 128-bit Q78.49 Guard-Bit Wide-Accumulator (g0_q)
  │   │                        • Odd-symmetric rounding (sym_shr128_33, sym_shr64_16)
  │   │                        • Operand-isolated shared 65x65 -> 130 signed multiplier
  │   │                        • 64-step radix-2 isqrt_round_u128 + 64-step restoring divider
  │   │                        • 128-bit Lyapunov monotonicity guard & 2^-49 w7 fiber lock
  │   │                        • Tickless algebraic fast-paths (G0 == 0, R2 == 0, disc < 0)
  │   └── ele_flow_atlas_top : 2^k Chart Arena controller + 1-cycle deterministic @swap
  ├── rtl/ele_em_phy.sv
  │   ├── ele_bus_invert65         : 65-wire charge-minimal transceiver (<= 32/65 flips)
  │   └── ele_ternary_mac_dualrail : Dual monotonic rail 1.58-bit ternary MAC (0-ULP)
  └── rtl/ele_cosim_tb.sv    : 4-probe 512-bit wind-tunnel & CMOS bit-toggle energy meter
```

---

## Synthesizable SystemVerilog IPs (`rtl/`)

1. **`rtl/ele_flow_core.sv`**:
   - **`ele_flow_core`**: Cycle-accurate, `0-ULP` bit-identical silicon realization of `FLOW v6.0.0`'s `flow__step_chart_core` (`512`-bit `State` $\times$ `512`-bit `Chart` $\to$ `512`-bit `State`, `58,245` Yosys CMOS cells, `2,367` DFFs, `0` latches, `0` FPU, `0` combinational divider arrays).
   - **Operand Isolation & Tickless Convergence**: Zeroes multiplier inputs outside active multiply states (`0` combinational toggle storms) and bypasses redundant sequential stages algebraically (`143` cycles in full quadratic surge $\to$ `70` cycles when linear `R2 == 0` $\to$ `4` cycles / `7` DFF flips at converged equilibrium `G0 == 0`).
   - **`ele_flow_atlas_top`**: Parameterizable $2^k$ Chart Arena controller executing `flow_step_atlas` and **1-cycle deterministic atomic `@swap`** (`flow_arena_try_swap` monotonic epoch, `next_lo`/`next_hi` mask bounds, `dt_max > 0`, and `b_exit <= b_enter` hysteresis firewall).
2. **`rtl/ele_em_phy.sv`**:
   - **`ele_bus_invert65`**: Stateful 65-wire (`64` data + `1` polarity) bus transceiver strictly capping wire transitions at $\le 32 / 65$ (`< 50%` peak $C \frac{dV}{dt}$ switching current).
   - **`ele_ternary_mac_dualrail`**: Glitch-free dual monotonic magnitude-rail `1.58-bit` balanced ternary MAC (`0-ULP` bit-identical to signed two's-complement accumulation while eliminating sign-extension toggle storms and gating zero trits/activations).

---

## Silicon Wind-Tunnel & CMOS Switching-Energy Telemetry

Measured directly in pure SystemVerilog (`rtl/ele_cosim_tb.sv` on `Verilator 5.054` linked against `../FLOW/flow.h`):

| Verification / Energy Probe | Coverage | Measured Silicon Result |
| :--- | :--- | :--- |
| **Probe 1: `0-ULP` 512-Bit Differential** | `4,096` hostile singular corners + `16,384` LFSR vectors | **`20,480 / 20,480` bit-identical (`0` defects)** across `w0..w7` |
| **Probe 2: Mirror Parity & $2^{-49}$ Fiber** | `1,024` mirror pairs + `2,048` autonomous steps | **`0` gauge asymmetries**, **`0-ULP` sub-LSB drift** |
| **Probe 3: 4-Chart Atlas & 1-Cycle `@swap`** | Valid hot-swap + 4 adversarial rejections + 16-step surge | **1-cycle deterministic `@swap`**, **`0-ULP` C99 lockstep** |
| **Probe 4A: `ele_flow_core` Tickless Energy** | `1,677` internal datapath DFFs (`16` active vs `16` converged) | `143 cyc` (`3,146` flips) $\to$ `70 cyc` (`880` flips) $\to$ **`4 cyc` (`7` flips, `-98.9%` toggles)** |
| **Probe 4B: `ele_bus_invert65` Wire Energy** | `1,024` burst + LFSR 64-bit bus transfers | **`max_flips = 32 / 65`**, **`-14.7%` wire toggles** (`29,435` vs `34,519`) |
| **Probe 4C: `ele_ternary_mac_dualrail`** | `1,024` sparse bipolar ternary MAC cycles | **`0-ULP` match**, **`-16.4%` DFF toggles** (`4,359` vs `5,219`) |

---

## Repository Layout

```text
ELE/
├── rtl/
│   ├── ele_flow_core.sv # Synthesizable FLOW v6.0.0 silicon engine + 1-cycle @swap top
│   ├── ele_em_phy.sv    # Charge-minimal 65-wire Bus-Invert & dual-rail ternary MAC
│   └── ele_cosim_tb.sv  # Pure-SV 512-bit DPI-C wind-tunnel & CMOS bit-toggle meter
├── Makefile             # 4-gate pure-SV hygiene + Verilator + FLOW DPI-C + Yosys pipeline
└── VERSION              # 6.0.0
```

---

## Quick Start & Verification Gates

Requires sibling [`FLOW`](../FLOW) (`v6.0.0`), a C99 compiler (`clang`/`gcc`), `verilator` ($\ge 5.0$), and `yosys`:

```sh
make clean && make test
```

`make test` enforces all 4 silicon verification gates:
- **Gate 1 (Pure-SV Source Hygiene Gate)**: Asserts `0` `.lean`, `.c`, `.h`, `.cpp`, or `.cc` files inside `ELE` and verifies upstream `../FLOW/flow.h` presence.
- **Gate 2 (Strict RTL Lint & Direct `FLOW v6.0.0` DPI-C Build)**: Passes `verilator --lint-only -Wall` across `ele_flow_core`, `ele_flow_atlas_top`, `ele_bus_invert65`, and `ele_ternary_mac_dualrail`, then builds `build/verilator/Vele_cosim_tb` with `0` warnings.
- **Gate 3 (`0-ULP` 512-Bit Silicon Wind-Tunnel & CMOS Energy Telemetry)**: Executes all 4 hardware & switching-energy probes (`total_tapeout_defects = 0`).
- **Gate 4 (Yosys CMOS Gate-Level Synthesis & Zero-Latch DRC)**: Synthesizes `ele_flow_core`, `ele_flow_atlas_top`, `ele_bus_invert65`, and `ele_ternary_mac_dualrail` with `read_verilog -nolatches -sv`, `check -assert`, and `select -assert-none t:$_DLATCH* t:$dlatch*`.
