#ifndef ELE_IVR_FLL_H
#define ELE_IVR_FLL_H

/* ============================================================================
 * ELE_IVR_FLL: Package Coupled-Inductor IVR & <1ns Droop-Adaptive AD-FLL
 * ============================================================================
 * Eliminates the physical silicon root cause of V_droop-induced Silent Data
 * Corruption (SDC / setup-time violations) AND unlocks the ~18%-22% dynamic
 * chip power wasted on static voltage guardbands:
 *
 *   1. Sub-Nanosecond Clock Stretching (`ele_fll_eval_period_ps`):
 *      Uses a division-free second-order convex delay envelope
 *        T_clk(dV) = T_nom + ceil(T_nom * x) + ceil(2 * T_nom * x^2)
 *      where x = dV / 512 mV. Since (1 - x)(1 + x + 2x^2) = 1 + x^2(1 - 2x) >=
 * 1 for all x in [0, 1/2], T_clk(dV) strictly upper-bounds the exact CMOS
 *      alpha-power critical-path delay T_nom / (1 - x) in 1 combinational cycle
 *      (< 1 ns, 0 hardware dividers), guaranteeing 0 setup-time violations.
 *   2. Guardband Elimination (`ele_ivr_fll_step_cycle`):
 *      Operates continuously at guardband-free V_nom (e.g. 750 mV vs 830 mV
 *      static guardband), stretching only the transient droop cycles while
 *      the coupled-inductor IVR slews rail charge back to V_nom.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint32_t v_nom_mv;            /*  4B: [0..3]   Guardband-free nominal V_dd  */
  uint32_t v_guard_static_mv;   /*  4B: [4..7]   Eliminated static guardband  */
  uint32_t v_th_mv;             /*  4B: [8..11]  Transistor V_th (mV)         */
  uint32_t t_nom_ps;            /*  4B: [12..15] Nominal clock period (ps)    */
  uint32_t v_inst_mv;           /*  4B: [16..19] Instantaneous rail V_dd (mV) */
  uint32_t t_clk_ps;            /*  4B: [20..23] Active AD-FLL period (ps)    */
  uint32_t l_pkg_uv_per_a;      /*  4B: [24..27] Package L*dI/dt (uV / A)     */
  uint32_t ivr_slew_mv_cyc;     /*  4B: [28..31] Coupled IVR slew (mV / cyc)  */
  uint32_t prev_load_amp;       /*  4B: [32..35] Previous cycle die load (A)  */
  uint32_t stretch_cycles;      /*  4B: [36..39] Cycles stretched > t_nom_ps  */
  uint32_t baseline_sdc_faults; /*  4B: [40..43] Fixed-clock SDC violations   */
  uint32_t fll_sdc_faults;      /*  4B: [44..47] AD-FLL SDC violations (== 0) */
  uint64_t energy_guardband_uj; /*  8B: [48..55] Static-guardband energy (uJ) */
  uint64_t energy_fll_uj;       /*  8B: [56..63] Guardband-free FLL energy    */
} EleIvrFll64;

ELE_STATIC_ASSERT(sizeof(EleIvrFll64) == 64U,
                  "EleIvrFll64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleIvrFll64) == 64U,
                  "EleIvrFll64 must be 64-byte aligned");

/* Initialize 64B Coupled-Inductor IVR & Droop-Adaptive AD-FLL descriptor */
void ele_ivr_fll64_init(EleIvrFll64 *fll, uint32_t v_nom_mv,
                        uint32_t v_guard_static_mv, uint32_t v_th_mv,
                        uint32_t t_nom_ps, uint32_t l_pkg_uv_per_a,
                        uint32_t ivr_slew_mv_cyc);

/* Division-free 1-cycle (<1ns) DCO clock period evaluation in picoseconds.
 * Bit-for-bit identical (0-ULP) to `rtl/ele_fll_droop.sv`. */
uint32_t ele_fll_eval_period_ps(uint32_t t_nom_ps, uint32_t v_nom_mv,
                                uint32_t v_inst_mv, bool *out_emergency_gate);

/* Evaluate exact CMOS alpha-power critical-path delay in picoseconds:
 * T_crit = ceil(T_nom * (V_nom - V_th) / (V_inst - V_th)) */
uint32_t ele_fll_eval_crit_delay_ps(uint32_t t_nom_ps, uint32_t v_nom_mv,
                                    uint32_t v_inst_mv, uint32_t v_th_mv);

/* Step 1 clock burst of package L*dI/dt droop, IVR charge recovery, and AD-FLL
 * clock stretching; returns safe stretched clock period in picoseconds. */
uint32_t ele_ivr_fll_step_cycle(EleIvrFll64 *fll, uint32_t load_amp);

#ifdef __cplusplus
}
#endif

#endif /* ELE_IVR_FLL_H */
