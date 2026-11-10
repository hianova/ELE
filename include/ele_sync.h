#ifndef ELE_SYNC_H
#define ELE_SYNC_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 2: 0-ULP 2-Jet Predictive Zero-Bit Synchronization (ele_sync)
 * Purpose:
 *   - Eliminates 95%+ of sensor polling, ADC conversion, and wire/RF packet
 *     transmissions between ESP32-C6 #1 (Tx), ESP32-C6 #2 (Rx), and Cardputer.
 *   - Both endpoints execute the identical integer 0-ULP 2-Jet Taylor operator:
 *       x_hat(t + dt) = x + v * dt + (1/2) * a * dt^2
 *   - When physical measurements (HTU21D Temp/RH, Piezo, Motor RPM) remain
 *     within the conservative inner deadband |x_meas - x_hat| <= epsilon,
 *     0 bits are transmitted and the 160MHz HP CPU stays in Deep Sleep.
 * ============================================================================
 */

typedef enum {
  ELE_SYNC_SLEEP_ZERO_BIT = 0, /* Within 2-Jet cone: 0 bits sent, HP sleeps */
  ELE_SYNC_WAKE_TRANSMIT_DELTA =
      1 /* Curvature surprise: wake & send residual */
} EleSyncDecision;

/* 64-Byte Univalent 2-Jet Predictive Sync & Calorimetric Node Descriptor */
typedef struct __attribute__((aligned(64))) {
  int64_t x_q16;       /* [0..7]   0-Jet state value (Q48.16) */
  int64_t v_q16;       /* [8..15]  1-Jet first derivative dx/dt (Q48.16) */
  int64_t a_q16;       /* [16..23] 2-Jet second derivative d2x/dt2 (Q48.16) */
  int64_t epsilon_q16; /* [24..31] Inner deadband threshold (Q48.16) */
  int32_t
      last_residual_q16; /* [32..35] Last transmitted surprise delta (Q16.16) */
  uint32_t
      ticks_since_sync; /* [36..39] Consecutive 0-bit ticks since last sync */
  int64_t
      energy_saved_uj_q16; /* [40..47] Cumulative saved energy in uJ (Q48.16) */
  uint32_t suppressed_frames;  /* [48..51] Zero-bit suppressed epochs */
  uint32_t transmitted_frames; /* [52..55] Active residual epochs */
  int32_t
      ambient_temp_c_q16; /* [56..59] HTU21D 0-ULP temperature (C, Q16.16) */
  uint32_t
      ambient_rh_pct_q16; /* [60..63] HTU21D 0-ULP humidity (%RH, Q16.16) */
} EleSyncNode64;

_Static_assert(sizeof(EleSyncNode64) == 64,
               "EleSyncNode64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleSyncNode64) == 64,
               "EleSyncNode64 must be 64-byte aligned");

/* Initialize 2-Jet predictive node */
void ele_sync_node64_init(EleSyncNode64 *node, int64_t initial_x_q16,
                          int64_t epsilon_q16);

/* Pure 0-ULP 2-Jet extrapolation: x + v*dt + (a*dt^2)/2 (dt in integer ticks)
 */
static inline int64_t ele_sync_predict_jet2_q16(const EleSyncNode64 *node,
                                                int64_t dt_ticks) {
  int64_t dt2 = dt_ticks * dt_ticks;
  return node->x_q16 + (node->v_q16 * dt_ticks) + ((node->a_q16 * dt2) >> 1);
}

/* Step Tx sensor gate: returns ELE_SYNC_SLEEP_ZERO_BIT or wakes to send delta
 */
EleSyncDecision ele_sync_step_tx_gate(EleSyncNode64 *tx_node,
                                      int64_t measured_x_q16,
                                      int64_t *out_delta_q16);

/* Step Rx mirror node: applies either 0-bit autonomous 2-Jet step or delta */
int64_t ele_sync_step_rx_mirror(EleSyncNode64 *rx_node,
                                EleSyncDecision decision,
                                int64_t received_delta_q16);

#ifdef __cplusplus
}
#endif

#endif /* ELE_SYNC_H */
