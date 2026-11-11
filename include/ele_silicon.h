#ifndef ELE_SILICON_H
#define ELE_SILICON_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 6: Silicon & Datacenter Electro-Thermal Physics Kernel
 * Targets the 4 Macro-Scale Compute Power Bottlenecks (Horowitz & PDN Laws):
 *   1. HBM / UCIe / CXL Capacitive Interconnect Wall (1/2 * C * V^2):
 *      - EleCacheBus64 combines 2-Jet/XOR temporal cache-line prediction with
 *        stateful 65-wire Bus-Invert polarity coding (64 data + 1 polarity
 *        wire per lane), mathematically capping physical transitions at <= 32
 *        per 64-bit beat (< 50% even on adversarial 0x00..00 <-> 0xFF..FF) and
 *        eliminating 75%..95% of memory-bus charge flips on tensor/KV streams.
 *   2. PDN Inductance Voltage-Droop Tax (dV = L * dI/dt & V_DD^2 Guardband):
 *      - ElePdnGrid64 smooths synchronous compute current steps via Symplectic
 *        Current-Slew Limiting while strictly conserving 100% of integrated
 *        compute work (sum I_symp == sum I_sync), collapsing L*dI/dt droop and
 *        unlocking 20%..30% V_DD^2 + I^2*R power savings.
 *   3. CMOS Gate-Toggle Switching Entropy (IEEE-754 FP32 vs 1.58-Bit Ternary):
 *      - Pure integer IEEE-754 binary32 bit-pattern synthesizer (0 FPU ops)
 *        quantifies exact mantissa/exponent carry-avalanche bit flips vs
 *        multiplier-free 1.58-bit ternary {-1, 0, +1} accumulation.
 *   4. Subthreshold Exponential Thermal-Leakage Runaway (I_leak ~ 2^(dT/12C)):
 *      - 2-Jet junction temperature (T, dT/dt, d2T/dt2) lookahead preempts
 *        exponential silicon leakage runaway prior to reactive OS throttling.
 * ============================================================================
 */

/* 64-Byte Cache-Line Univalent HBM / UCIe Charge-Minimal Bus Channel */
typedef struct __attribute__((aligned(64))) {
  uint64_t prev_bus_beat[4];    /* [0..31]  Physical wire state (256b beat) */
  uint64_t raw_unencoded_flips; /* [32..39] Baseline uncoded wire bit flips */
  uint64_t encoded_wire_flips;  /* [40..47] Actual data + polarity wire flips */
  int64_t energy_saved_pj_q16;  /* [48..55] Saved 1/2*C*V^2 energy (pJ, Q16) */
  uint32_t zero_beat_skips;     /* [56..59] Clock-gated 0-flip beats */
  uint16_t line_cap_ff;         /* [60..61] Per-wire capacitance (fF) */
  uint8_t vdd_mv_div10;         /* [62]     Swing voltage in 10mV (e.g. 40) */
  uint8_t prev_inv_mask4;       /* [63]     Stateful 4-lane polarity wires */
} EleCacheBus64;

ELE_STATIC_ASSERT(sizeof(EleCacheBus64) == 64U,
                  "EleCacheBus64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleCacheBus64) == 64U,
                  "EleCacheBus64 must be 64-byte aligned");

/* 64-Byte Cache-Line Univalent PDN dI/dt, Toggle-Entropy & Thermal Governor */
typedef struct __attribute__((aligned(64))) {
  uint32_t v_nom_mv;               /* [0..3]   Minimum droop-free V_DD (mV) */
  uint32_t l_droop_uv_per_a;       /* [4..7]   PDN L/dt droop (uV per A/tick) */
  uint32_t max_slew_amp_tick;      /* [8..11]  Symplectic dI/dt ceiling (A) */
  uint32_t cur_current_amp;        /* [12..15] Active smoothed current (A) */
  uint32_t backlog_work_amp_ticks; /* [16..19] Conserved deferred work queue */
  uint32_t peak_vdroop_sync_mv;    /* [20..23] Peak synchronous Vdroop (mV) */
  uint32_t peak_vdroop_symp_mv;    /* [24..27] Peak symplectic Vdroop (mV) */
  int32_t temp_mc;                 /* [28..31] 0-Jet junction temp (mC) */
  int32_t dtemp_mc_tick;           /* [32..35] 1-Jet heating rate (mC/tick) */
  int32_t ddtemp_mc_tick2;         /* [36..39] 2-Jet thermal accel (mC/tick2) */
  int32_t temp_knee_mc;            /* [40..43] Exponential leakage knee (mC) */
  uint32_t thermal_preempts;       /* [44..47] Pre-emptive 2-Jet migrations */
  int64_t pdn_energy_saved_mj;     /* [48..55] Saved V_DD^2 + leakage (mJ) */
  uint64_t gate_toggle_saved_bits; /* [56..63] Saved CMOS gate bit toggles */
} ElePdnGrid64;

ELE_STATIC_ASSERT(sizeof(ElePdnGrid64) == 64U,
                  "ElePdnGrid64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(ElePdnGrid64) == 64U,
                  "ElePdnGrid64 must be 64-byte aligned");

/* Initialize 64B HBM / UCIe charge-minimal bus channel */
void ele_silicon_bus64_init(EleCacheBus64 *bus, uint16_t line_cap_ff,
                            uint16_t vdd_mv);

/* Encode & transmit a 64-byte cache line (8 x 64b words = 2 x 256b HBM beats)
 * using optional 2-Jet/XOR prediction + stateful 65-wire Bus-Invert coding.
 * Returns the 8-bit lane polarity mask (1 bit per 64-bit word). */
uint8_t ele_silicon_tx_cacheline64(EleCacheBus64 *bus,
                                   const uint64_t next_line[8],
                                   const uint64_t pred_line[8],
                                   uint64_t out_wire[8]);

/* Losslessly reconstruct the exact 64-byte cache line (0-ULP) from wire words
 * and the 8-bit Bus-Invert polarity mask */
void ele_silicon_rx_cacheline64(const uint64_t wire_words[8],
                                const uint64_t pred_line[8], uint8_t inv_mask8,
                                uint64_t out_line[8]);

/* Initialize 64B PDN dI/dt slew & 2-Jet electro-thermal governor */
void ele_silicon_pdn64_init(ElePdnGrid64 *pdn, uint32_t v_nom_mv,
                            uint32_t l_droop_uv_per_a,
                            uint32_t max_slew_amp_tick, int32_t ambient_mc,
                            int32_t temp_knee_mc);

/* Advance 1 tick of Symplectic dI/dt current-slew shaping (conserving 100% of
 * requested compute work while bounding |dI/dt| <= max_slew_amp_tick) */
uint32_t ele_silicon_step_pdn_symplectic(ElePdnGrid64 *pdn,
                                         uint32_t prev_sync_amp,
                                         uint32_t req_sync_amp);

/* Evaluate subthreshold silicon leakage power in mW (doubles every 12 C above
 * 45 C in pure integer arithmetic, FloatOps == 0) */
uint32_t ele_silicon_eval_leakage_mw(int32_t temp_mc, uint32_t base_leak_mw);

/* Step 2-Jet predictive electro-thermal barrier vs reactive OS throttling;
 * returns recommended current cap (Amps) before exponential leakage runaway */
uint32_t ele_silicon_step_thermal_jet2(ElePdnGrid64 *pdn,
                                       uint32_t active_power_w,
                                       int32_t ambient_mc,
                                       uint32_t horizon_ticks);

/* Synthesize exact IEEE-754 binary32 bit-representation from Q16.16 in pure
 * integer arithmetic (0 FPU instructions) and compare CMOS register bit-flip
 * entropy against 1.58-bit ternary {-1, 0, +1} Gray-coded accumulation */
uint64_t ele_silicon_compare_dot_toggle_entropy(
    ElePdnGrid64 *pdn, const int32_t *inputs_q16, const int8_t *weights_trit,
    uint32_t len, uint64_t *out_fp32_toggles, uint64_t *out_ternary_toggles);

/* Sign-partitioned commutative 1.58-bit ternary inner product:
 * Accumulates positive (+1) and negative (-1) streams in monotonic
 * sign-coherent lanes prior to a single terminal subtraction, guaranteeing
 * 0-ULP exactness while eliminating two's-complement zero-crossing sign-bit
 * carry avalanches */
int64_t ele_silicon_dot_ternary_sign_grouped(const int32_t *inputs_q16,
                                             const int8_t *weights_trit,
                                             uint32_t len,
                                             uint64_t *out_grouped_toggles);

#ifdef __cplusplus
}
#endif

#endif /* ELE_SILICON_H */
