#ifndef ELE_PHOTONICS_CPO_H
#define ELE_PHOTONICS_CPO_H

/* ============================================================================
 * ELE_PHOTONICS_CPO: Silicon Photonics Microring Modulo-FSR & Thermal Lock
 * ============================================================================
 * Solves the thermo-optic & optical-bistability wall in Co-Packaged Optics
 * (CPO) Dense Wavelength-Division Multiplexing (DWDM) interconnects:
 *
 *   1. Modulo-FSR Vernier Wavelength Hopping (`12-bit` 2*pi optical phase):
 *      Silicon microring resonators drift ~80 pm/K, spanning multiple Free
 *      Spectral Ranges (FSR) across GPU die temperature swings. Instead of
 *      burning 3x-4x DC heater power to hold a fixed mode, modulo-FSR wraps
 *      micro-heater detuning onto `[0, FSR)` (`& 0x0FFF`), cutting tuning
 *      power by >60% while hopping cleanly to the adjacent resonance comb.
 *   2. 2-Jet Compute-Thermal Wave Feedforward & Bistability Clamp:
 *      Pre-biases the micro-heater 1 step ahead of adjacent XPU power surges
 *      and clamps two-photon cavity self-heating (`0` optical unlock drops).
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((aligned(64))) {
  uint32_t energy_conv_uj;      /*  4B: [0..3]   Unwrapped fixed-mode uJ      */
  uint32_t energy_hop_uj;       /*  4B: [4..7]   Modulo-FSR hopping heater uJ */
  uint32_t total_frames;        /*  4B: [8..11]  Total optical frames         */
  uint32_t unlock_drops_hop;    /*  4B: [12..15] Feedforward+Hop drops (== 0) */
  uint32_t unlock_drops_no_ff;  /*  4B: [16..19] Reactive baseline drops      */
  uint32_t fsr_hops;            /*  4B: [20..23] Modulo-FSR mode hop count    */
  int32_t die_temp_mc;          /*  4B: [24..27] Local PIC die temp (mC)      */
  int32_t d_temp_mc;            /*  4B: [28..31] 1-Jet thermal slope (mC/cyc) */
  uint16_t target_phase12;      /*  2B: [32..33] Target DWDM grid [0..4095]   */
  uint16_t heater_phase12;      /*  2B: [34..35] Modulo-FSR heater [0..4095]  */
  uint16_t prev_raw_phase12;    /*  2B: [36..37] Previous wrapped phase       */
  uint16_t lock_tol_phase12;    /*  2B: [38..39] Phase lock window [0..4095]  */
  uint16_t uw_per_phase_lsb;    /*  2B: [40..41] Heater uW per phase LSB      */
  uint16_t k_to_phase_per_c;    /*  2B: [42..43] Thermo-optic LSB per 1 C     */
  uint16_t k_gpu_ff_mdeg_per_w; /*  2B: [44..45] Thermal coupling mC / Watt   */
  uint16_t self_heat_phase12;   /*  2B: [46..47] Optical cavity self-heat LSB */
  uint16_t prev_gpu_watt;       /*  2B: [48..49] Previous adjacent XPU Watts  */
  uint16_t opt_power_uw;        /*  2B: [50..51] Input laser power (uW)       */
  uint16_t bistable_limit_uw;   /*  2B: [52..53] Safe optical bistable cap uW */
  uint16_t heater_power_uw;     /*  2B: [54..55] Active micro-heater power uW */
  uint16_t last_phase_err12;    /*  2B: [56..57] Post-lock phase error LSB    */
  uint16_t base_temp_c;         /*  2B: [58..59] Calibration baseline temp C  */
  uint8_t locked;               /*  1B: [60]     1 = DWDM carrier locked      */
  uint8_t hop_event;            /*  1B: [61]     1 = FSR mode hop this step   */
  uint8_t bistable_clamped;     /*  1B: [62]     1 = laser clamped below snap */
  uint8_t primed;               /*  1B: [63]     1 = 2-Jet thermal primed     */
} EleCpoMrr64;

ELE_STATIC_ASSERT(sizeof(EleCpoMrr64) == 64U,
                  "EleCpoMrr64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleCpoMrr64) == 64U,
                  "EleCpoMrr64 must be 64-byte aligned");

/* Initialize 64B CPO Silicon Photonics Microring Resonator controller */
void ele_cpo_mrr64_init(EleCpoMrr64 *mrr, uint16_t target_phase12,
                        uint16_t base_temp_c, uint16_t k_to_phase_per_c,
                        uint16_t uw_per_phase_lsb, uint16_t lock_tol_phase12,
                        uint16_t k_gpu_ff_mdeg_per_w,
                        uint16_t bistable_limit_uw);

/* Pure 12-bit Modulo-FSR + feedforward detuning step (0-ULP == RTL).
 * Computes wrapped heater detuning `[0..4095]` and hop flag. */
uint16_t ele_cpo_eval_heater_phase12(uint16_t target_phase12,
                                     uint16_t thermal_phase12,
                                     int16_t ff_delta_phase,
                                     uint16_t prev_heater_phase12,
                                     bool *out_hop);

/* Step 1 ms electro-optic / thermo-optic control loop under adjacent XPU power
 * `gpu_watt`, measured PIC temperature `meas_temp_mc`, and input laser power
 * `laser_in_uw`. Returns active micro-heater power in microwatts. */
uint32_t ele_cpo_mrr_step_ms(EleCpoMrr64 *mrr, int32_t meas_temp_mc,
                             uint16_t gpu_watt, uint16_t laser_in_uw);

#ifdef __cplusplus
}
#endif

#endif /* ELE_PHOTONICS_CPO_H */
