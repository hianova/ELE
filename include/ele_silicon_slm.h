#ifndef ELE_SILICON_SLM_H
#define ELE_SILICON_SLM_H

/* ============================================================================
 * ELE_SILICON_SLM: Zero-Stall 2-Jet Flight Recorder, Aging Canary & Contact DFT
 * ============================================================================
 * In-field Silicon Lifecycle Management (SLM) & non-intrusive hardware debug
 * with 0 CPU stall cycles, 0 observer-effect Heisenbugs, and 0 external deps:
 *
 *   1. Zero-Stall 2-Jet Trace Encoder & 8-Deep Flight Recorder
 * (`EleTraceBox64`): Passively snoops 16-bit PC, 12-bit rail V_dd (mV), and
 * 4-bit fault flags with 0 pipeline stalls. Suppresses 95%-99% of predictable
 * sequential/loop PC and 1-Jet rail trajectories (0 trace pin flips), and
 * hardware-locks the last 8 nanosecond pre-crash snapshots in L1/SRAM on fault
 * trigger.
 *   2. NBTI/HCI Aging Canary -> AD-FLL Self-Healing (`EleAgingCanary64`):
 *      A shadow canary path delayed by `+canary_guard_ps` flags timing margin
 *      erosion strictly BEFORE the main datapath violates T_clk (0 SDC),
 *      closed-loop updating `EleIvrFll64.v_th_mv` to heal multi-year silicon
 * age.
 *   3. In-Situ Differential Micro-Ohm Contact/BGA Probe
 * (`ele_slm_probe_contact`): Evaluates R_diff = (V_low - V_high) / (I_high -
 * I_low) across live load steps, canceling static V_ref offsets to catch BGA
 * solder-joint fatigue and busbar oxidation before I^2*R thermal runaway.
 * ============================================================================
 */

#include "ele_ivr_fll.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pack 16-bit PC, 4-bit hardware flags, and 12-bit V_dd (mV) into 32-bit slot
 */
static inline uint32_t ele_trace_pack_snap32(uint16_t pc16, uint8_t flags4,
                                             uint16_t v_dd_mv) {
  uint32_t v12 = (v_dd_mv > 4095U) ? 4095U : (uint32_t)v_dd_mv;
  uint32_t f4 = (uint32_t)(flags4 & 0x0FU);
  return ((uint32_t)pc16 << 16U) | (f4 << 12U) | v12;
}

typedef struct __attribute__((aligned(64))) {
  uint32_t ring_snap[8];      /* 32B: [0..31]  8-cycle pre-trigger blackbox  */
  uint32_t suppressed_cycles; /*  4B: [32..35] Zero-pin-flip quiet cycles    */
  uint32_t emitted_packets;   /*  4B: [36..39] Emitted branch/PVT anchors    */
  uint16_t prev_pc16;         /*  2B: [40..41] Previous cycle PC [15:0]      */
  uint16_t pc_stride16;       /*  2B: [42..43] Modular PC stride [15:0]      */
  uint16_t prev_v_mv;         /*  2B: [44..45] Previous cycle V_dd [0..4095] */
  int16_t dv_mv_cyc;          /*  2B: [46..47] 1-Jet V_dd slope (mV/cyc)     */
  uint16_t v_eps_mv;          /*  2B: [48..49] Quiet V_dd prediction bound   */
  uint16_t trigger_count;     /*  2B: [50..51] Flight-recorder freeze count  */
  uint32_t full_trace_bits;   /*  4B: [52..55] Raw unfiltered trace bits     */
  uint32_t slm_trace_bits;    /*  4B: [56..59] Compressed single-wire bits   */
  uint8_t wr_idx;             /*  1B: [60]     Circular ring write head 0..7 */
  uint8_t freeze_idx;         /*  1B: [61]     Frozen crash index 0..7       */
  uint8_t frozen;             /*  1B: [62]     1 = pre-crash ring locked     */
  uint8_t primed;             /*  1B: [63]     1 = trace predictor primed    */
} EleTraceBox64;

ELE_STATIC_ASSERT(sizeof(EleTraceBox64) == 64U,
                  "EleTraceBox64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleTraceBox64) == 64U,
                  "EleTraceBox64 must be 64-byte aligned");

typedef struct __attribute__((aligned(64))) {
  uint32_t v_th_base_mv;         /*  4B: [0..3]   Fresh T=0 V_th (mV)         */
  uint32_t est_vth_shift_mv;     /*  4B: [4..7]   Compensated NBTI shift (mV) */
  uint32_t canary_guard_ps;      /*  4B: [8..11]  Canary guard delay (ps)     */
  uint32_t rem_slack_ps;         /*  4B: [12..15] Main-path timing slack (ps) */
  uint32_t r_baseline_uohm;      /*  4B: [16..19] Healthy joint R (uOhm)      */
  uint32_t r_warn_uohm;          /*  4B: [20..23] Fatigue/oxidation alarm R   */
  uint32_t r_measured_uohm;      /*  4B: [24..27] Latest dV/dI joint R (uOhm) */
  uint32_t prev_sample_v_uv;     /*  4B: [28..31] Low-load sense V (uV)       */
  uint32_t prev_sample_i_amp;    /*  4B: [32..35] Low-load current (Amps)     */
  uint32_t canary_pre_alarms;    /*  4B: [36..39] Pre-SDC canary catches      */
  uint32_t self_heal_steps;      /*  4B: [40..43] Closed-loop FLL updates     */
  uint32_t main_sdc_healed;      /*  4B: [44..47] Healed main SDC (== 0)      */
  uint32_t main_sdc_unmonitored; /*  4B: [48..51] Unmonitored aging SDCs      */
  uint32_t stress_kilohours;     /*  4B: [52..55] Cumulative silicon age (kh) */
  uint16_t max_aged_vth_mv;      /*  2B: [56..57] Safe V_th retirement limit  */
  uint16_t contact_fault_events; /*  2B: [58..59] BGA/busbar crack alarms     */
  uint8_t canary_alarm;          /*  1B: [60]     1 = canary slack warning    */
  uint8_t solder_fatigue_warn;   /*  1B: [61]     1 = joint dV/dI degraded    */
  uint8_t retire_recommended;    /*  1B: [62]     1 = silicon EOL reached     */
  uint8_t sample_primed;         /*  1B: [63]     1 = low-I VI anchor latched */
} EleAgingCanary64;

ELE_STATIC_ASSERT(sizeof(EleAgingCanary64) == 64U,
                  "EleAgingCanary64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleAgingCanary64) == 64U,
                  "EleAgingCanary64 must be 64-byte aligned");

/* Initialize 64B Zero-Stall 2-Jet Trace Encoder & 8-Deep Flight Recorder */
void ele_trace_box64_init(EleTraceBox64 *box, uint16_t v_eps_mv);

/* Re-arm flight recorder after post-mortem snapshot readout */
void ele_trace_box_rearm(EleTraceBox64 *box);

/* Step 1 non-intrusive trace cycle (0 CPU stall). Bit-for-bit identical (0-ULP)
 * to `rtl/ele_slm_canary_trace.sv`. Returns true iff a compressed trace anchor
 * packet is emitted on the single-wire debug port. */
bool ele_trace_step_cycle(EleTraceBox64 *box, uint16_t pc16, uint16_t v_dd_mv,
                          uint8_t hw_flags4, bool fault_trigger);

/* Read chronologically ordered snapshot `age_idx` in [0..7] (0 = oldest `t-7`,
 * 7 = crash cycle `t`) from a frozen or live `EleTraceBox64`. */
uint32_t ele_trace_read_ordered(const EleTraceBox64 *box, uint8_t age_idx);

/* Initialize 64B Silicon Aging Canary & Contact Impedance DFT monitor */
void ele_canary64_init(EleAgingCanary64 *can, uint32_t v_th_base_mv,
                       uint32_t canary_guard_ps, uint16_t max_aged_vth_mv,
                       uint32_t r_baseline_uohm, uint32_t r_warn_uohm);

/* Step aging canary against true aged threshold shift `true_nbti_shift_mv` and
 * closed-loop self-heal `EleIvrFll64` (`fll->v_th_mv`) before main-path SDC.
 * Returns remaining main-path timing slack in picoseconds. */
uint32_t ele_canary_step_and_heal(EleAgingCanary64 *can, EleIvrFll64 *fll,
                                  uint32_t load_amp,
                                  uint32_t true_nbti_shift_mv,
                                  uint32_t delta_kilohours);

/* In-situ differential micro-ohm contact / BGA solder-fatigue probe:
 * Computes R_diff = (V_low_uv - V_high_uv) / (I_high_a - I_low_a) across live
 * load transitions (canceling DC regulator offset). Returns R in uOhm. */
uint32_t ele_canary_probe_contact_uohm(EleAgingCanary64 *can,
                                       uint32_t sense_v_uv, uint32_t load_amp);

#ifdef __cplusplus
}
#endif

#endif /* ELE_SILICON_SLM_H */
