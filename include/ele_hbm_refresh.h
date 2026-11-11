#ifndef ELE_HBM_REFRESH_H
#define ELE_HBM_REFRESH_H

/* ============================================================================
 * ELE_HBM_REFRESH: HBM4 Thermal-Refresh Breaker & Graphene Rowhammer Guard
 * ============================================================================
 * Solves two coupled physical walls in 12-Hi / 16-Hi 3D-stacked HBM4 DRAM:
 *
 *   1. Temperature-Retention-Refresh Runaway Breaker (`EleHbmStack64`):
 *      Breaks the positive feedback loop (`high T -> 2x/4x/8x JEDEC refresh ->
 *      refresh self-heating -> >95 C thermal runaway`) by eliding refresh on
 *      ephemeral/dead AI activation scratchpad banks (`live_bank_mask`).
 *   2. Constant-Space 4-Slot Graphene Spillover Rowhammer Guard (`DRFM`):
 *      Tracks hot aggressor rows in 4 registers + 1 spillover upper bound
 *      (`0-ULP` identical to `rtl/ele_hbm_rowhammer_guard.sv`), mathematically
 *      guaranteeing 0 undetected Rowhammer bit-flips across 65,536 rows.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint32_t ref_pulses_jedec;     /*  4B: [0..3]   Blanket 16-bank refreshes   */
  uint32_t ref_pulses_elided;    /*  4B: [4..7]   Transient-elided refreshes  */
  uint32_t energy_jedec_mj;      /*  4B: [8..11]  Blanket refresh energy (mJ) */
  uint32_t energy_symp_mj;       /*  4B: [12..15] Elided + DRFM energy (mJ)   */
  int32_t temp_jedec_mc;         /*  4B: [16..19] Blanket stack temp (mC)     */
  int32_t temp_symp_mc;          /*  4B: [20..23] Symplectic stack temp (mC)  */
  uint32_t drfm_mitigations;     /*  4B: [24..27] Targeted DRFM refresh fires */
  uint32_t preventive_refreshes; /*  4B: [28..31] Spillover-cap bank resets   */
  uint16_t rh_row[4];            /*  8B: [32..39] 4-slot Graphene row IDs     */
  uint16_t rh_cnt[4];            /*  8B: [40..47] 4-slot upper-bound counts   */
  uint16_t rh_spill_cnt;         /*  2B: [48..49] Untracked spillover bound   */
  uint16_t rh_drfm_thresh;       /*  2B: [50..51] DRFM trigger (< T_rh_flip)  */
  uint16_t last_drfm_row;        /*  2B: [52..53] Last mitigated aggressor    */
  uint16_t live_bank_mask;       /*  2B: [54..55] Persistent live bank mask   */
  uint16_t r_th_mc_per_w;        /*  2B: [56..57] 3D stack R_th (mC / Watt)   */
  uint16_t p_ref_bank_mw;        /*  2B: [58..59] 1x per-bank refresh mW      */
  uint8_t rh_valid_mask;         /*  1B: [60]     Slot valid mask [3:0]       */
  uint8_t cur_ref_mult;          /*  1B: [61]     Active refresh mult 1/2/4/8 */
  uint8_t runaway_jedec;         /*  1B: [62]     1 = blanket hit >= 95 C     */
  uint8_t runaway_symp;          /*  1B: [63]     1 = elided hit >= 95 C (0)  */
} EleHbmStack64;

ELE_STATIC_ASSERT(sizeof(EleHbmStack64) == 64U,
                  "EleHbmStack64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleHbmStack64) == 64U,
                  "EleHbmStack64 must be 64-byte aligned");

/* Initialize 64B HBM4 3D Thermal-Refresh & Graphene Rowhammer controller */
void ele_hbm_stack64_init(EleHbmStack64 *hbm, int32_t amb_temp_mc,
                          uint16_t r_th_mc_per_w, uint16_t p_ref_bank_mw,
                          uint16_t rh_drfm_thresh, uint16_t live_bank_mask);

/* Reset Rowhammer window counters at end of JEDEC tREFW window */
void ele_hbm_rowhammer_window_reset(EleHbmStack64 *hbm);

/* Step 1 DRAM row activation through the 4-slot Graphene Spillover Guard
 * (0-ULP == `rtl/ele_hbm_rowhammer_guard.sv`). Returns true if targeted DRFM
 * or preventive refresh fired on this activation. */
bool ele_hbm_step_row_activate(EleHbmStack64 *hbm, uint16_t row_id,
                               bool *out_preventive_ref);

/* Step 10 ms 3D HBM4 thermal-refresh coupled loop under base logic/IO power
 * `p_io_mw` and ambient coolant temperature `amb_temp_mc`. Returns active
 * refresh rate multiplier (1, 2, 4, or 8). */
uint8_t ele_hbm_step_thermal_10ms(EleHbmStack64 *hbm, int32_t amb_temp_mc,
                                  uint32_t p_io_mw, uint16_t live_bank_mask);

#ifdef __cplusplus
}
#endif

#endif /* ELE_HBM_REFRESH_H */
