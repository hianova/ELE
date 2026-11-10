#ifndef ELE_CSI_H
#define ELE_CSI_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 4: 0-ULP ESP-NOW Wi-Fi CSI Wave Interferometer & ISAC Governor
 * Physical & Mathematical Basis:
 *   - ESP32-C6 (Wi-Fi 6 802.11ax HE-LTF) & ESP32-S3 (Cardputer HT20) expose
 *     per-subcarrier complex Channel State Information H_k = I_k + i Q_k as
 *     signed 8-bit integer pairs (int8_t buf[2 * N_sub]).
 *   - Traditional toolkits (esp-csi) waste massive CPU & UART power computing
 *     floating-point sqrtf(I^2+Q^2), atan2f(Q,I), and streaming 100+ pkt/s at
 *     fixed +20 dBm (100 mW) TX power.
 * ELE 0-ULP Improvements:
 *   1. Divisionless Spinor Cross-Phase Holonomy:
 *        Phi_cross = sum_{k=1}^{N-1} (I_k * Q_{k-1} - Q_k * I_{k-1})
 *      Eliminates random packet CFO/PLL phase offset without atan2f or float!
 *   2. Adaptive ESP-NOW Rate & TX Power Scaling (ISAC):
 *      - Scales ESP-NOW beacon rate between 2 Hz (idle static field) and
 *        100 Hz (active wave/motion/breathing tracking).
 *      - Scales Wi-Fi TX power register (0.25 dBm steps: 8 = +2 dBm [1.6 mW]
 *        up to 80 = +20 dBm [100 mW]) to minimum SNR-sufficient field strength.
 * ============================================================================
 */

#define ELE_CSI_SUBCARRIERS 64U

typedef enum {
  ELE_CSI_FIELD_STATIC_SLEEP = 0, /* Static room: 2 Hz beacon, Modem Sleep */
  ELE_CSI_FIELD_MICRO_WAVE = 1, /* Sub-cm wave (breathing / vibration): 25 Hz */
  ELE_CSI_FIELD_MACRO_MOTION = 2 /* Large boundary motion / gesture: 100 Hz */
} EleCsiFieldRegime;

/* 64-Byte Univalent ESP-NOW CSI Wave & ISAC Power Descriptor */
typedef struct __attribute__((aligned(64))) {
  int64_t baseline_energy_q16; /* [0..7]   0-Jet baseline subcarrier power */
  int64_t
      baseline_holonomy_q16;    /* [8..15]  0-Jet baseline spinor cross-phase */
  int64_t wave_variance_q16;    /* [16..23] Instantaneous wave perturbation */
  int64_t dwave_q16;            /* [24..31] 1-Jet wave velocity */
  int64_t energy_saved_uj_q16;  /* [32..39] Radio + CPU energy saved vs
                                   100Hz/20dBm */
  uint32_t frames_processed;    /* [40..43] Total ESP-NOW CSI frames ingested */
  uint32_t suppressed_beacons;  /* [44..47] Beacons skipped via adaptive duty */
  uint16_t recommended_rate_hz; /* [48..49] Optimal ESP-NOW rate (2..100 Hz) */
  uint8_t
      tx_power_qdbm;    /* [50]     ESP Wi-Fi TX power (8=+2dBm .. 80=+20dBm) */
  uint8_t field_regime; /* [51]     EleCsiFieldRegime state */
  int8_t last_rssi_dbm; /* [52]     Measured packet RSSI (dBm) */
  int8_t noise_floor_dbm;      /* [53]     Measured RF noise floor (dBm) */
  uint16_t active_subcarriers; /* [54..55] Subcarrier count (e.g. 52 or 64) */
  uint64_t
      coherence_signature; /* [56..63] 64-bit spatial multipath fingerprint */
} EleCsiState64;

_Static_assert(sizeof(EleCsiState64) == 64,
               "EleCsiState64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleCsiState64) == 64,
               "EleCsiState64 must be 64-byte aligned");

/* Initialize ESP-NOW CSI wave interferometer */
void ele_csi_state64_init(EleCsiState64 *csi, uint16_t num_subcarriers);

/* Ingest raw int8_t (I, Q) CSI buffer from esp_wifi_set_csi_rx_cb:
 *   - Computes divisionless spinor cross-phase & amplitude energy (0 float ops)
 *   - Updates field regime (STATIC_SLEEP / MICRO_WAVE / MACRO_MOTION)
 *   - Adjusts ESP-NOW beacon rate (2..100 Hz) and Wi-Fi TX power (+2..+20 dBm)
 */
EleCsiFieldRegime ele_csi_ingest_espnow_frame(EleCsiState64 *csi,
                                              const int8_t *iq_buf,
                                              uint16_t buf_len, int8_t rssi_dbm,
                                              int8_t noise_floor_dbm);

#ifdef __cplusplus
}
#endif

#endif /* ELE_CSI_H */
