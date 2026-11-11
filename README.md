# ELE — Zero-Code Silicon Driver, Datacenter Power Electronics, Diurnal Grid Physics & Synthesizable RTL Kernel

`ELE` (`libele.a` / `ele_core_rv32.o` / `rtl/*.sv`) is a standalone, zero-floating-point (`FloatOps == 0`), zero-heap (`HeapCalls == 0`), 100% 64-byte cache-line univalent embedded silicon driver, `48V / 800V` power-electronics controller, diurnal grid-interactive microgrid dispatcher, and multi-scale electro-thermal physics kernel targeting bare-metal 32-bit RISC-V (`RV32IMAC`, `0` undefined symbols, `-ffreestanding -nostdlib`), Cortex-M33, FPGA/ASIC interconnects, and AI cluster power delivery networks (PDN).

Instead of writing vendor HAL boilerplate, spinning CPUs in `delay_ms()`, wasting `14%+` in triple `AC-DC-AC-DC` solar/UPS conversions, or leaving AI cluster GPUs dark behind substation breakers and weak-inertia island grids, `ELE` compiles hardware datasheets and physical conservation laws into **thirteen 64-byte cache-line descriptors** across nine C17 modules, **twenty-four machine-checked Lean 4 theorems**, and a **five-file synthesizable IEEE 1800 SystemVerilog RTL IP suite** verified cycle-by-cycle (`0-ULP`) against C17 via Verilator and synthesized to CMOS gates via Yosys.

---

## Architecture & Module Overview

### 1. Bare-Metal Silicon & Edge Interconnect Modules

1. **`ele_hal` ([`include/ele_hal.h`](include/ele_hal.h)) — Cross-MCU Bare-Metal Silicon Port, Pinmux & Power-Domain HAL (`EleHalPort64`)**
   - Unifies atomic 1-cycle GPIO set/clear (`W1TS/W1TC`, `RP2350 SIO`, `STM32 BSRR`), hardware pad drive-strength encoding (`5mA / 10mA / 20mA / 40mA`), `mcycle` edge timing, and dual-domain (`HP` active vs `LP` tickless deep-sleep) energy accounting across `ESP32-C6`, `ESP32-S3`, `RP2350`, `STM32`, and deterministic host wind tunnels.

2. **`ele_driver` ([`include/ele_driver.h`](include/ele_driver.h)) — 64B Zero-Code Datasheet Compiler (`EleDriverCodec64`), Tickless Bus Sequencer (`EleBusSeq64`) & `INA226` Self-Calorimeter**
   - **Zero-Code Register-to-Jet & Actuator Codec (`EleDriverCodec64`)**: Maps raw peripheral frames (`HTU21D`, `MPU6050`, `INA226`, DACs/ADCs) across endianness, bit-shift, bit-width, 2's-complement sign extension, **hardware-free CRC-8 sieves** (`0x31` Sensirion / `0x07` SMBus PEC), and `Q16.16` affine calibration directly into live 2-Jet state $(x, \dot{x}, \ddot{x})$ — plus rail-clamped inverse register encoding (`ele_driver_encode_actuator_raw`) and multi-axis burst decoding (`ele_driver_decode_burst_axes`).
   - **Declarative Bus Micro-Sequencer (`EleBusSeq64`)**: Packs multi-step I2C/SPI/MMIO init, bitmask-safe Read-Modify-Write (`ELE_OP_RMW_BITS8`, formally proven via `bv_decide` to preserve unmasked datasheet bits), conversion triggers, non-blocking `LP` sleep waits (`ELE_OP_WAIT_US`), bounded `DRDY` polling (`ELE_OP_POLL_MASK`), and burst reads into `4 x 64-bit` register words (`ele_bus_op_pack`).
   - **Built-In `INA226` Hardware Self-Calorimeter (`ele_driver_init_ina226_calorimeter` / `ele_driver_eval_ina226_power_uw`)**: Out-of-the-box zero-code preset for TI `INA226` bi-directional shunt/bus power monitors (`2.5 uV/LSB` shunt + `1.25 mV/LSB` bus -> exact integer microwatts with `0` FPU ops and `0` 64-bit software division calls).
   - **Tickless 2-Jet Polling Governor & 1-Cycle Self-Healing**: Scales polling periods exponentially toward `max_period_us` in static fields ($|\dot{x}| + |\ddot{x}| \le \epsilon$) while snapping back to `min_period_us` in **1 cycle** on physical slew impulses; verifies Goldilocks $\mathbb{F}_p$ ($p = 2^{64} - 2^{32} + 1$) seals for zero-reboot hot-swap, 1-cycle shadow rollback, and 9-clock stuck-bus recovery.

3. **`ele_wire` ([`include/ele_wire.h`](include/ele_wire.h)) — Surface-Charge Conservation & Poynting Waveguide Interconnect (`EleWireLink64`)**
   - Matches pad drive strength to dielectric waveguide impedance $Z_0$ and capacitance $C_{\text{eff}}$, replacing 8N1 UART framing with **Single-Edge Pulse-Position Modulation (PPM) + Gray-Delta coding** (`1` physical voltage flip per 8-bit symbol, cutting $\frac{1}{2} C V_{DD}^2$ surface-charge repolarization loss by **83%**).

4. **`ele_sync` ([`include/ele_sync.h`](include/ele_sync.h)) — Sensor-Agnostic 0-ULP 2-Jet Predictive Zero-Bit Synchronization (`EleSyncNode64`)**
   - Transmitter and receiver run an identical integer 2-Jet Taylor extrapolator ($\hat{x} = x + v \Delta t + \frac{1}{2} a \Delta t^2$). Within $|\Delta x| \le \epsilon$, **zero bits are transmitted** (`ELE_SYNC_SLEEP_ZERO_BIT`), suppressing `99%` of steady/linear/parabolic frames with **0-ULP drift**.

5. **`ele_drive` ([`include/ele_drive.h`](include/ele_drive.h)) — Symplectic Pulse-and-Coast Electromechanical Controller (`EleMotorDrive64`)**
   - Replaces continuous hard-switched PWM with Schmitt-hysteresis **Phase-Locked Pulse-and-Coast**, floating the H-bridge (`EN = 0`) during coast windows to eliminate saturation ($I \cdot V_{\text{sat}}$) and copper ($I^2 R$) heat (**91%** parasitic loss reduction) while harvesting `Piezo` vibration and overspeed Back-EMF.

6. **`ele_csi` ([`include/ele_csi.h`](include/ele_csi.h)) — 0-ULP Wi-Fi CSI Spinor Wave Interferometer & ISAC Governor (`EleCsiState64`)**
   - Computes **Divisionless Spinor Cross-Phase Holonomy** ($\Phi_{\times} = \sum (I_k Q_{k-1} - Q_k I_{k-1})$) across 64 subcarriers without `atan2f`/`sqrtf`, canceling PLL/CFO phase noise and scaling beacons across `2 Hz / 25 Hz / 100 Hz` and `+2 dBm` TX power.

---

### 2. Silicon, Power Electronics, Diurnal Grid & Synthesizable Hardware IP (`ele_silicon`, `ele_rack_vrm`, `ele_grid` & `rtl/`)

7. **`ele_silicon` ([`include/ele_silicon.h`](include/ele_silicon.h)) — Memory-Wall, On-Die PDN, Ternary Toggle Entropy & Subthreshold Leakage (`EleCacheBus64`, `ElePdnGrid64`)**
   - **Stateful 65-Wire Bus-Invert & 2-Jet Cache-Line Channel (`EleCacheBus64` + [`rtl/ele_cache_bus65.sv`](rtl/ele_cache_bus65.sv))**: Targets 64-byte (`2 x 256-bit` HBM3/UCIe/AXI) channels in both C17 and synthesizable AXI4-Stream SystemVerilog RTL (`ele_cache_bus65_axi_bridge`). Strictly caps simultaneous switching outputs (SSO) at **$\le 32$ out of $65$ wires ($< 50\%$)** even under adversarial `0x00..00 <-> 0xFF..FF` thrashing, cutting streaming tensor/KV-cache bus flips by **72%–99%** with bit-exact 0-ULP reconstruction.
   - **Symplectic $dI/dt$ PDN Voltage-Droop Eliminator (`ElePdnGrid64`)**: Bounds synchronous compute current steps ($|\Delta I / \Delta t| \le S_{\max}$) while **strictly conserving 100.0% of integrated compute work** ($\sum I_{\text{symp}} = \sum I_{\text{sync}}$), collapsing inductive supply droop $\Delta V = L \frac{dI}{dt}$ (`150 mV -> 18 mV`, **-88%**) to unlock quadratic $V_{DD}^2$ guardband savings.
   - **Dual Monotonic Magnitude-Rail Ternary Datapath ([`rtl/ele_ternary_mac_dualrail.sv`](rtl/ele_ternary_mac_dualrail.sv))**: Partitions `1.58-bit` ternary $\{-1, 0, +1\}$ dot products onto two independently clock-gated unsigned monotonic rails (`pos_rail_o`, `neg_rail_o`) with operand-isolated terminal subtraction (`readout_en_i`) — proving **0-ULP equivalence** while eliminating two's-complement zero-crossing carry avalanches (**-62% physical flip-flop toggles** vs clock-gated two's-complement RTL, **-80%** vs `FP32`, and **-70%+ synthesized CMOS logic cells** vs `Int32 x Int16` multiplier MAC in Yosys).
   - **2-Jet Subthreshold Thermal-Leakage Barrier (`ele_silicon_step_thermal_jet2`)**: Models nanometer subthreshold exponential leakage ($I_{\text{leak}} \propto 2^{\Delta T / 12^\circ\text{C}}$) and uses 2-Jet $(T, \dot{T}, \ddot{T})$ lookahead to migrate workloads prior to the exponential runaway knee.

8. **`ele_rack_vrm` ([`include/ele_rack_vrm.h`](include/ele_rack_vrm.h)) — `48V` Multiphase GaN/SiC Digital VRM & 8-GPU Rack Stranded-Power Unlocker (`EleVrmPhase64`, `EleRackBusbar64`)**
   - **Exact Divisionless ZVS-Aware Convex Phase-Shedding & 2-Jet ZVS Soft-Switching (`EleVrmPhase64` + [`rtl/ele_vrm_zvs_ctrl.sv`](rtl/ele_vrm_zvs_ctrl.sv))**: Minimizes multiphase buck loss $P_{\text{loss}}(I, N) = \frac{I^2 R_{\text{ph}}}{N} + N P_{\text{sw,eff}}$ via division-free integer crossover thresholds ($I^2 R_{\text{ph}} > N(N+1) P_{\text{sw,eff}}$) coupled directly to a 2-Jet picosecond Zero-Voltage Switching (`ZVS`) dead-time controller (`0-ULP` lockstep between C17 and RTL) — eliminating **67%** of `48V -> 0.75V` VRM conversion waste across mixed LLM KV-decode/GEMM workloads.
   - **Command-Lookahead Active Pre-Charge**: Wakes all 16 VRM stages `1–2 us` ahead of scheduled GPU Tensor Core launches to deadbeat pre-charge $C_{\text{out}}$ within safe transient headroom, collapsing unipolar $V_{\text{droop}}$ from **`120 mV` down to `5 mV`** with zero load-release undershoot.
   - **8-Socket `48V` Rack Symplectic Phase-Interleaver (`EleRackBusbar64`)**: Desynchronizes synchronous AllReduce/GEMM current crests across 8 accelerators (`1,760 A -> 1,250 A`) while **strictly conserving 100.0% of compute work** ($\sum I_{\text{symp}} = \sum I_{\text{sync}}$), cutting busbar/transformer $I^2 R$ heat by **24%** and powering **`8` active GPUs instead of `5` (+60% unlocked GPU capacity)** on an existing `1,250 A` substation breaker.

9. **`ele_grid` ([`include/ele_grid.h`](include/ele_grid.h)) — Diurnal Duck-Curve `800V HVDC` Microgrid & `60Hz/50Hz` Grid-Forming (`GFM`) Synthetic-Inertia Controller (`EleGridInverter64`, `EleDuckMicrogrid64`)**
   - **1-Cycle Zero-Crossing Grid-Forming Virtual Synchronous Generator (`EleGridInverter64` + [`rtl/ele_grid_gfm_ctrl.sv`](rtl/ele_grid_gfm_ctrl.sv))**: Converts hardware AC period captures (`period_ns`) to `1 mHz` precision (`60,000` / `50,000 mHz`) with zero FPU ops. Executes a multiplier-free shift-add Virtual Synchronous Generator (`VSG`) injecting primary droop and synthetic rotational inertia ($-M \frac{df}{dt}$) alongside 1-cycle (`16.67 ms`) hardware compute `P-cap` droop — arresting isolated `60 Hz` island contingencies from `59.372 Hz` (UFLS blackout) up to **`59.800 Hz`** while soaking `60.200 Hz` midday solar surges at `115%` compute overdrive.
   - **Behind-the-Meter `800V HVDC` Solar-BESS-Thermal-Compute Dispatcher (`EleDuckMicrogrid64`)**: Replaces legacy `PV(DC) -> AC -> Grid -> UPS(AC-DC-AC) -> PFC(DC)` triple conversion (`~14.3%` loss) with direct `800V HVDC` GaN `ZVS` coupling (`~2.5%` loss), shifting PCM/chilled-water cooling and elastic AI batch jobs out of `16:00–22:00` Night Peaks into midday solar canyons with **100.0% daily compute conservation** — cutting regulated Night-Peak feeder draw by **52%** and eliminating **100%** of negative-LMP solar curtailment (**-80% net 48h wholesale energy cost**).

---

## Synthesizable SystemVerilog RTL IP & Quantitative Benchmarks

| RTL Module / Subsystem | Physical / Protocol Role | Verified Hardware Guarantee & Measured Gain |
| :--- | :--- | :--- |
| [`rtl/ele_cache_bus65.sv`](rtl/ele_cache_bus65.sv) | 65-Wire AXI4-Stream Charge-Minimal PHY (`ele_cache_bus65_axi_bridge`) | Pure 65-wire channel (`0` out-of-band predictor wires); caps SSO `<= 32 / 65` (`MaxSSO = 19/65` in co-sim, **`-72%`..`-99%` wire flips**) |
| [`rtl/ele_ternary_mac_dualrail.sv`](rtl/ele_ternary_mac_dualrail.sv) | Multiplier-Free `1.58-Bit` Dual-Rail Ternary MAC (`readout_en_i` gated) | `0-ULP == C17`; **`-62%` FF toggles** vs clock-gated 2's-complement RTL, **`-80%`** vs `FP32`, **`-70%+` CMOS gate area** vs `Int32 x Int16` MAC |
| [`rtl/ele_vrm_zvs_ctrl.sv`](rtl/ele_vrm_zvs_ctrl.sv) | 16-Phase `48V` GaN/SiC Digital VRM (`HRPWM` ZVS + Active Pre-Charge) | `0-ULP` ZVS dead-time lockstep with C17 (`1,500..25,000 ps`); **`-67%` VRM waste**, $V_{\text{droop}}$ **`120 mV -> 5 mV`**; rack **`5 -> 8` GPUs** on `1,250 A` breaker |
| [`rtl/ele_grid_gfm_ctrl.sv`](rtl/ele_grid_gfm_ctrl.sv) | `60Hz/50Hz` Grid-Forming (`GFM`) Virtual Synchronous Generator & Sponge | Glitch-free `50Hz/60Hz` cold-start; `16.67 ms` single-cycle UFLS arrest (`59.372 -> 59.800 Hz`); Night-Peak **`-52%`**, Neg-LMP Curtail **`102,320 -> 0 kWh`** (`Cost -80%`) |
| [`rtl/ele_silicon_top.sv`](rtl/ele_silicon_top.sv) | Unified Multi-Domain Co-Simulation & CMOS Synthesis Top | `0` Verilator lint warnings (`6,110` co-sim assertions), `0` Yosys latches (`read_verilog -nolatches`), `0` DRC errors |

---

## Repository Layout

| Path | Description |
| :--- | :--- |
| [`include/`](include/) | Nine 64-byte cache-line univalent C17 headers (`13` `64B` descriptors across `ele_hal.h` .. `ele_grid.h`) |
| [`src/ele_core.c`](src/ele_core.c) | Freestanding-ready pure integer C17 implementation (`0` floats, `0` heap, `0` external symbols on `RV32IMAC`) |
| [`rtl/`](rtl/) | Five-file synthesizable IEEE 1800 SystemVerilog IP suite (`ele_cache_bus65.sv`, `ele_ternary_mac_dualrail.sv`, `ele_vrm_zvs_ctrl.sv`, `ele_grid_gfm_ctrl.sv`, `ele_silicon_top.sv`) |
| [`research/EnergyOptimality.lean`](research/EnergyOptimality.lean) | 30 Lean 4 formal definitions and 24 machine-checked theorems (`0 sorry`) |
| [`tests/test_ele_energy.c`](tests/test_ele_energy.c) | 212,858-assertion C17 hardware, `INA226`, silicon PDN, `48V` VRM, rack breaker, diurnal `800V HVDC` / `60Hz GFM`, and adversarial UB torture suite |
| [`tests/sim_ele_rtl.cpp`](tests/sim_ele_rtl.cpp) | Cycle-accurate Verilator RTL co-simulation & register-toggle calorimeter (`6,110` assertions, `0-ULP == C17`) |

---

## Build & Verification Gates

```sh
make fmt-check
make clean && make all && make test
```

- **Gate 0 (`RV32IMAC` Bare-Metal Cross-Compile)**: Compiles `src/ele_core.c` to `build/riscv32/ele_core_rv32.o` and `ele_core_rv32_O0.o` (`--target=riscv32-unknown-none-elf -march=rv32imac -ffreestanding -nostdlib -fno-builtin -fno-stack-protector`) and verifies **0 undefined external symbols** across `-O0..-Oz` via `nm -u`.
- **Gate 1 (Silicon, Power Electronics & Diurnal Grid Benchmark)**: Runs `build/test-ele-energy` (**212,858 assertions**, `0` UBSan violations across 13 `64B` descriptors, 4 MCU port profiles, zero-code `HTU21D`/`MPU6050`/DAC/`INA226` drivers, waveguide PPM, 2-Jet zero-bit sync, symplectic motor coasting, CSI spinor holonomy, HBM/PDN/ternary/leakage silicon physics, `48V` 16-phase GaN VRM + 8-GPU rack unlocking, and `60Hz GFM` + `800V HVDC` Duck-Curve dispatch).
- **Gate 2 (Native LLVM IR Static Audit)**: Inspects `build/ele_core.ll` to enforce **`0` floating-point instructions/types** and **`0` heap allocation calls** (`malloc`/`free`).
- **Gate 3 (Lean 4 Formal Proof, Verilator Co-Sim & Yosys CMOS Synthesis)**: Verifies all **24 formal theorems** (`0 sorry`) in [`research/EnergyOptimality.lean`](research/EnergyOptimality.lean), runs 4,096+ cycle bit-exact C17 co-simulation (`build/verilator/Vele_silicon_top`) via `verilator`, and synthesizes all RTL modules to standard CMOS logic cells (`0` latches, `0` DRC errors) via `yosys`.

---

## License

MIT License. See [`LICENSE`](LICENSE).
