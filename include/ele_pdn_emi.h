#ifndef ELE_PDN_EMI_H
#define ELE_PDN_EMI_H

/* ============================================================================
 * ELE_PDN_EMI: BSPDN Glass-Core Resonance Damper & Symplectic Spread-Spectrum
 * ============================================================================
 * Solves two high-frequency wave bottlenecks in Backside Power Delivery
 * (BSPDN) + Glass-Substrate 3D packages and 100 V/ns GaN/SiC stages:
 *
 *   1. Active -gamma * dV/dt Standing-Wave Anti-Resonance Damper:
 *      Ultra-low-R BSPDN + Through-Glass Vias (TGV) inflate package Q =
 *      (1/R)*sqrt(L/C), causing 5x-8x voltage magnification when compute loops
 *      hit f_res = 1/(2*pi*sqrt(L*C)). Zero-DC velocity feedback collapses
 *      resonant voltage standing waves by >75% with 0 static power waste.
 *   2. Paired (+delta, -delta) Galois-LFSR Symplectic Spread-Spectrum (`EMI`):
 *      Modulates consecutive cycle pairs as `(T_0 + d_k, T_0 - d_k)` (`0-ULP`
 *      2-cycle period & duty conservation!) to flatten narrowband GaN/clock
 *      EMI harmonic spikes by >10 dB without discrete common-mode chokes.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint64_t sum_period_base_ps; /*  8B: [0..7]   Cumulative fixed period (ps) */
  uint64_t sum_period_ss_ps;   /*  8B: [8..15]  Cumulative spread period ps  */
  int32_t v_osc_undamped_uv;   /*  4B: [16..19] Undamped BSPDN wave V (uV)   */
  int32_t i_osc_undamped_ma;   /*  4B: [20..23] Undamped BSPDN wave I (mA)   */
  int32_t v_osc_damped_uv;     /*  4B: [24..27] Active-damped wave V (uV)    */
  int32_t i_osc_damped_ma;     /*  4B: [28..31] Active-damped wave I (mA)    */
  uint32_t peak_v_undamped_uv; /*  4B: [32..35] Peak undamped swing (uV)     */
  uint32_t peak_v_damped_uv;   /*  4B: [36..39] Peak damped swing (uV)       */
  uint16_t bin_hits[8];        /* 16B: [40..55] 8-bin EMI spectral histogram */
  uint16_t lfsr_state;         /*  2B: [56..57] 16-bit Galois LFSR (0xB400)  */
  uint16_t nom_period_ps;      /*  2B: [58..59] Nominal period T_0 (ps)      */
  int8_t held_delta_ps;        /*  1B: [60]     Paired odd-step -delta (ps)  */
  uint8_t max_dither_ps;       /*  1B: [61]     Max dither amplitude (ps)    */
  uint8_t damping_gain_q8;     /*  1B: [62]     Active -gamma*dV gain Q0.8   */
  uint8_t pair_phase;          /*  1B: [63]     0 = +delta, 1 = -delta       */
} ElePdnEmi64;

ELE_STATIC_ASSERT(sizeof(ElePdnEmi64) == 64U,
                  "ElePdnEmi64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(ElePdnEmi64) == 64U,
                  "ElePdnEmi64 must be 64-byte aligned");

/* Initialize 64B BSPDN Active Resonance Damper & Symplectic Spread-Spectrum */
void ele_pdn_emi64_init(ElePdnEmi64 *emi, uint16_t nom_period_ps,
                        uint8_t max_dither_ps, uint8_t damping_gain_q8,
                        uint16_t seed);

/* Step 1 cycle of paired (+delta, -delta) Galois-LFSR spread-spectrum &
 * coupled BSPDN LC resonator under load excitation `di_excite_ma` (0-ULP
 * lockstep with `rtl/ele_pdn_emi_damping.sv`). Returns modulated period (ps).
 */
uint16_t ele_pdn_emi_step_cycle(ElePdnEmi64 *emi, int16_t di_excite_ma);

#ifdef __cplusplus
}
#endif

#endif /* ELE_PDN_EMI_H */
