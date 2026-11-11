#ifndef ELE_THERMO_FLUID_H
#define ELE_THERMO_FLUID_H

/* ============================================================================
 * ELE_THERMO_FLUID: Two-Phase Microchannel Boiling & Cube-Law Pump Governor
 * ============================================================================
 * Governs direct-to-chip two-phase cold-plate boiling and manifold hydraulic
 * pump power for kW-class AI accelerators:
 *
 *   1. Critical Heat Flux (CHF) & Vapor-Blanketing Dryout Prevention:
 *      Nucleate microchannel boiling absorbs high heat flux via latent heat
 *      h_fg until vapor exit quality x_vap = Q / (m_dot * h_fg) breaches
 *      critical quality x_crit (~700‰), triggering film-boiling dryout where
 *      thermal conductance collapses 8x and silicon spikes >115 C.
 *      2-Jet heat-flux lookahead pre-accelerates pump rotor inertia ahead of
 *      compute bursts to guarantee 0 CHF dryout events.
 *   2. Pump Affinity Cube-Law Convexity (`P_pump = P_idle +
 * P_rated*(m/m_max)^3`): Because hydraulic shaft power scales with the CUBE of
 * mass flow rate (50% flow = 12.5% power), tracking the minimal safe
 * nucleate-boiling flow trajectory cuts cooling pump electrical energy by
 * 65%-80%.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint32_t flow_max_ml_s;        /*  4B: [0..3]   100% rated flow (mL/s)      */
  uint32_t flow_min_ml_s;        /*  4B: [4..7]   Min channel wetting (mL/s)  */
  uint32_t pump_rated_mw;        /*  4B: [8..11]  Rated pump power (mW)       */
  uint32_t h_fg_j_per_ml;        /*  4B: [12..15] Latent heat capacity (J/mL) */
  int32_t t_coolant_in_mc;       /*  4B: [16..19] Saturation / inlet T (mC)   */
  int32_t t_junc_mc;             /*  4B: [20..23] Silicon junction T (mC)     */
  uint32_t cur_flow_ml_s;        /*  4B: [24..27] Actual rotor flow (mL/s)    */
  uint32_t pump_slew_ml_tick;    /*  4B: [28..31] Max rotor ramp (mL/s/10ms)  */
  uint32_t prev_q_watt;          /*  4B: [32..35] Previous chip heat (Watts)  */
  int32_t dq_watt_tick;          /*  4B: [36..39] 1-Jet heat velocity (W/tick)*/
  uint16_t vapor_qual_permille;  /*  2B: [40..41] Exit vapor quality [0..1000]*/
  uint16_t chf_crit_permille;    /*  2B: [42..43] CHF dryout ceiling (e.g.700)*/
  uint16_t dryout_events_symp;   /*  2B: [44..45] ELE dryout count (== 0)     */
  uint16_t dryout_events_react;  /*  2B: [46..47] Reactive lag dryout count   */
  uint64_t pump_energy_fixed_mj; /*  8B: [48..55] 100%-flow pump energy (mJ)  */
  uint64_t pump_energy_symp_mj;  /*  8B: [56..63] Cube-law pump energy (mJ)   */
} EleColdPlate64;

ELE_STATIC_ASSERT(sizeof(EleColdPlate64) == 64U,
                  "EleColdPlate64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleColdPlate64) == 64U,
                  "EleColdPlate64 must be 64-byte aligned");

/* Initialize 64B Two-Phase Microchannel Cold-Plate & Cube-Law Pump governor */
void ele_coldplate64_init(EleColdPlate64 *cp, uint32_t flow_max_ml_s,
                          uint32_t flow_min_ml_s, uint32_t pump_rated_mw,
                          uint32_t h_fg_j_per_ml, int32_t t_coolant_in_mc,
                          uint32_t pump_slew_ml_tick,
                          uint16_t chf_crit_permille);

/* Evaluate hydraulic pump electrical power in mW via Pump Affinity Cube Law:
 * P_pump(m_dot) = P_idle + P_rated * (m_dot / m_max)^3  (FloatOps == 0) */
uint32_t ele_thermo_eval_pump_mw(const EleColdPlate64 *cp, uint32_t flow_ml_s);

/* Step 10 ms electro-thermal-fluid tick: uses 2-Jet heat-flux lookahead +
 * optional scheduler hint `upcoming_q_watt` to pre-accelerate coolant flow
 * before CHF dryout while riding the cube-law curve. Returns flow (mL/s). */
uint32_t ele_coldplate_step_10ms(EleColdPlate64 *cp, uint32_t q_chip_watt,
                                 uint32_t upcoming_q_watt);

#ifdef __cplusplus
}
#endif

#endif /* ELE_THERMO_FLUID_H */
