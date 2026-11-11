#ifndef ELE_RESONANT_CLK_H
#define ELE_RESONANT_CLK_H

/* ============================================================================
 * ELE_RESONANT_CLK: LC Adiabatic Clock-Tree Charge Recovery & ZVS Tank Parking
 * ============================================================================
 * Breaks the irreversible C_clk * V_dd^2 hard-switching clock-tree power wall:
 *
 *   1. Spiral-Inductor LC Symplectic Charge Recycling (`EleResonantClk64`):
 *      Exchanges clock-tree electrostatic energy (1/2 C V^2) and magnetic
 *      inductor energy (1/2 L I^2) adiabatically, replenishing only (pi / Q)
 *      resistive tank dissipation at Zero-Voltage Switching (ZVS) crossings
 *      (>73% dynamic clock power saved at package inductor Q = 12).
 *   2. Zero-Current Crest Tank Parking (`rtl/ele_resonant_clk_zvs.sv`):
 *      Latches tank charge onto a DC hold state at voltage crest (I_L == 0)
 *      on clock-gate sleep and resumes oscillation in 1 cycle with 0 crowbar
 *      short-circuit loss and 0 LC ringing.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ELE_RCLK_PARKED = 0,
  ELE_RCLK_OSCILLATING = 1
} EleResonantClkState;

typedef struct __attribute__((aligned(64))) {
  uint64_t energy_cmos_pj;     /*  8B: [0..7]   Hard-switched C*V^2 (pJ)     */
  uint64_t energy_resonant_pj; /*  8B: [8..15]  LC charge-recovered (pJ)     */
  uint32_t total_cycles;       /*  4B: [16..19] Active resonant cycles       */
  uint32_t parked_cycles;      /*  4B: [20..23] Zero-power parked cycles     */
  uint32_t crowbar_faults;     /*  4B: [24..27] Non-ZVS crowbar faults (0)   */
  uint32_t zvs_lock_cycles;    /*  4B: [28..31] Cycles locked inside ZVS tol */
  uint32_t c_clk_ff;           /*  4B: [32..35] Clock-tree capacitance (fF)  */
  uint16_t v_dd_mv;            /*  2B: [36..37] Clock rail swing (mV)        */
  uint16_t q_factor_milli;     /*  2B: [38..39] Tank Q * 1000 (e.g. 12000)   */
  uint16_t sustain_pulse_ps;   /*  2B: [40..41] Sustaining ZVS pulse (ps)    */
  uint16_t target_pulse_ps;    /*  2B: [42..43] Nominal sustaining pulse ps  */
  int16_t phase_err_ps;        /*  2B: [44..45] ZVS zero-crossing error (ps) */
  int16_t prev_phase_err_ps;   /*  2B: [46..47] Previous ZVS error (ps)      */
  uint16_t tank_env_mv;        /*  2B: [48..49] Active LC amplitude (mV)     */
  uint16_t park_cap_mv;        /*  2B: [50..51] Parked crest hold V (mV)     */
  uint16_t freq_mhz;           /*  2B: [52..53] Tank resonant freq (MHz)     */
  uint16_t r_switch_mohm;      /*  2B: [54..55] Sustainer bridge R_on (mOhm) */
  uint16_t zvs_tol_ps;         /*  2B: [56..57] ZVS lock window (ps)         */
  uint16_t restart_events;     /*  2B: [58..59] Lossless 1-cycle restarts    */
  uint8_t state;               /*  1B: [60]     EleResonantClkState          */
  uint8_t zvs_locked;          /*  1B: [61]     1 = sustaining switch at ZVS */
  uint8_t park_ready;          /*  1B: [62]     1 = parked at I_L=0 crest    */
  uint8_t primed;              /*  1B: [63]     1 = ZVS phase loop primed    */
} EleResonantClk64;

ELE_STATIC_ASSERT(sizeof(EleResonantClk64) == 64U,
                  "EleResonantClk64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleResonantClk64) == 64U,
                  "EleResonantClk64 must be 64-byte aligned");

/* Initialize 64B LC Resonant Adiabatic Clock-Tree controller */
void ele_resonant_clk64_init(EleResonantClk64 *rclk, uint32_t c_clk_ff,
                             uint16_t v_dd_mv, uint16_t q_factor_milli,
                             uint16_t freq_mhz, uint16_t target_pulse_ps,
                             uint16_t zvs_tol_ps);

/* Step 1 resonant clock cycle or zero-current park transition (0-ULP == RTL
 * `rtl/ele_resonant_clk_zvs.sv`). Returns active sustaining pulse width (ps)
 * or 0 when lossless-parked. */
uint16_t ele_resonant_clk_step(EleResonantClk64 *rclk, bool clk_enable,
                               int16_t zvs_err_ps);

#ifdef __cplusplus
}
#endif

#endif /* ELE_RESONANT_CLK_H */
