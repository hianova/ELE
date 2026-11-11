#ifndef ELE_EVENT_ADC_H
#define ELE_EVENT_ADC_H

/* ============================================================================
 * ELE_EVENT_ADC: Asynchronous Level-Crossing ADC & Vernier Picosecond TDC
 * ============================================================================
 * Replaces wasteful fixed-clock Nyquist SAR polling with continuous-time
 * event-driven mixed-signal quantization & time-domain picosecond encoding:
 *
 *   1. Hysteresis-Guarded Send-on-Delta Quantizer (`EleEventAdc64`):
 *      Emits +/-1 delta pulses ONLY when `|V_in - V_dac| >= lsb + hyst`.
 *      Eliminates 100% of sub-LSB thermal-noise limit-cycle chatter and burns
 *      0 digital conversion power during steady physical intervals (>94%
 * saved).
 *   2. Differential Vernier Delay-Line TDC (`rtl/ele_level_crossing_adc.sv`):
 *      Quantizes sub-cycle crossing times via `n_tap * (t_slow - t_fast)`
 *      (e.g. `45 ps - 40 ps = 5 ps` resolution) without high-voltage op-amps.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint32_t nyquist_samples;    /*  4B: [0..3]   Equivalent Nyquist ticks     */
  uint32_t event_samples;      /*  4B: [4..7]   Emitted crossing events      */
  uint32_t chatter_suppressed; /*  4B: [8..11]  Hysteresis-blocked toggles   */
  uint32_t energy_nyquist_pj;  /*  4B: [12..15] Fixed Nyquist SAR energy pJ  */
  uint32_t energy_event_pj;    /*  4B: [16..19] Event + Vernier TDC pJ       */
  uint32_t last_event_time_ps; /*  4B: [20..23] Coarse + Vernier crossing ps */
  int32_t dac_ref_uv;          /*  4B: [24..27] Tracked DAC reference (uV)   */
  int32_t recon_sig_uv;        /*  4B: [28..31] Reconstructed output (uV)    */
  uint32_t lsb_step_uv;        /*  4B: [32..35] Quantization step (uV)       */
  uint32_t hyst_uv;            /*  4B: [36..39] Anti-chatter guard band (uV) */
  uint32_t coarse_clk_ps;      /*  4B: [40..43] Coarse tick period (ps)      */
  uint16_t t_slow_delay_ps;    /*  2B: [44..45] Vernier slow cell delay (ps) */
  uint16_t t_fast_delay_ps;    /*  2B: [46..47] Vernier fast cell delay (ps) */
  uint16_t vernier_res_ps;     /*  2B: [48..49] Sub-gate LSB (slow - fast)   */
  uint16_t last_vernier_steps; /*  2B: [50..51] Vernier coincidence tap idx  */
  uint16_t e_sar_conv_fj;      /*  2B: [52..53] Nyquist SAR conv energy (fJ) */
  uint16_t e_event_pulse_fj;   /*  2B: [54..55] Crossing + TDC energy (fJ)   */
  uint16_t peak_recon_err_uv;  /*  2B: [56..57] Max |V_in - V_recon| (uV)    */
  uint16_t quiet_ticks;        /*  2B: [58..59] Consecutive 0-power ticks    */
  int8_t last_dir;             /*  1B: [60]     -1 = down, 0 = quiet, +1 = up*/
  uint8_t event_fired;         /*  1B: [61]     1 = crossing pulse emitted   */
  uint8_t tdc_overflow;        /*  1B: [62]     1 = Vernier taps saturated   */
  uint8_t primed;              /*  1B: [63]     1 = initial DAC anchored     */
} EleEventAdc64;

ELE_STATIC_ASSERT(sizeof(EleEventAdc64) == 64U,
                  "EleEventAdc64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleEventAdc64) == 64U,
                  "EleEventAdc64 must be 64-byte aligned");

/* Initialize 64B Asynchronous Level-Crossing Event ADC + Vernier TDC */
void ele_event_adc64_init(EleEventAdc64 *adc, uint32_t lsb_step_uv,
                          uint32_t hyst_uv, uint32_t coarse_clk_ps,
                          uint16_t t_slow_delay_ps, uint16_t t_fast_delay_ps,
                          uint16_t e_sar_conv_fj, uint16_t e_event_pulse_fj);

/* Evaluate sub-cycle Vernier delay-line coincidence timestamp offset (ps)
 * from raw arrival lead `lead_ps` (`0-ULP` == `rtl/ele_level_crossing_adc.sv`)
 */
uint16_t ele_vernier_tdc_eval_ps(uint16_t lead_ps, uint16_t t_slow_ps,
                                 uint16_t t_fast_ps, uint16_t max_taps,
                                 uint16_t *out_taps, bool *out_overflow);

/* Step 1 asynchronous window against analog input `v_in_uv` and sub-cycle
 * arrival offset `crossing_lead_ps`. Returns -1, 0, or +1 delta event. */
int8_t ele_event_adc_step(EleEventAdc64 *adc, int32_t v_in_uv,
                          uint16_t crossing_lead_ps);

#ifdef __cplusplus
}
#endif

#endif /* ELE_EVENT_ADC_H */
