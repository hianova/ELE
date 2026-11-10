#ifndef ELE_WIRE_H
#define ELE_WIRE_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 1: Surface-Charge Conservation & Poynting Waveguide Interconnect
 * Physical Basis:
 *   - Conductors act as waveguide boundaries; energy propagates in the
 *     surrounding dielectric via Poynting flux S = (1/mu_0)(E x B) steered by
 *     boundary surface-charge density gradients (nabla sigma_s).
 *   - Every 0<->1 voltage transition dissipates E_flip = (1/2) * C_eff * V_dd^2
 *     plus impedance-mismatch reflection energy.
 * Engineering Optimizations:
 *   1. Hardware FUN_DRV (5mA..40mA) impedance matching to eliminate ringing.
 *   2. Single-Edge Pulse-Position Modulation (PPM) & Gray-Delta coding:
 *      transmits an 8-bit payload using strictly 1 voltage transition instead
 *      of 8 UART/SPI bit transitions (up to 87.5% surface-charge saving).
 * ============================================================================
 */

typedef enum {
  ELE_GEOM_TWISTED_PAIR =
      0, /* Tightly coupled signal+GND (~105 Ohm, min S leakage) */
  ELE_GEOM_COPLANAR = 1, /* Adjacent ribbon/Dupont pair (~140 Ohm) */
  ELE_GEOM_OPEN_LOOP =
      2 /* Separated wire loop (~280+ Ohm, high radiative loss) */
} EleWaveguideGeometry;

/* 64-Byte Univalent Surface-Charge & Waveguide Link Descriptor */
typedef struct __attribute__((aligned(64))) {
  uint32_t z0_ohms_q16;      /* [0..3]   Waveguide characteristic Z0 (Q16.16) */
  uint32_t c_parasitic_ff;   /* [4..7]   Effective wire+pin capacitance (fF) */
  uint32_t vdd_mv;           /* [8..11]  Signaling voltage (e.g. 3300 mV) */
  uint32_t iomux_shadow_reg; /* [12..15] ESP32-C6 IO_MUX register image */
  uint32_t gpio_shadow_reg;  /* [16..19] ESP32-C6 GPIO_OUT register image */
  uint32_t ppm_base_cycles;  /* [20..23] Base guard cycles (160MHz 6.25ns) */
  uint64_t total_bits_sent;  /* [24..31] Total logical information bits sent */
  uint64_t
      actual_surface_flips;     /* [32..39] Actual physical 0<->1 transitions */
  uint64_t baseline_uart_flips; /* [40..47] Transitions standard 8N1 UART would
                                   use */
  int64_t energy_used_pj_q16;   /* [48..55] Dissipated energy in picojoules
                                   (Q48.16) */
  int64_t energy_saved_pj_q16;  /* [56..63] Saved energy vs standard 40mA UART
                                   (Q48.16) */
} EleWireLink64;

_Static_assert(sizeof(EleWireLink64) == 64,
               "EleWireLink64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleWireLink64) == 64,
               "EleWireLink64 must be 64-byte aligned");

/* Initialize link descriptor and automatically match ESP32-C6 FUN_DRV */
void ele_wire_link64_init(EleWireLink64 *link, uint32_t length_mm,
                          EleWaveguideGeometry geom, uint32_t vdd_mv);

/* Select optimal ESP32-C6 IO_MUX drive strength (5mA..40mA) for waveguide Z0 */
EleGpioDriveStrength ele_wire_select_optimal_drive(uint32_t z0_ohms_q16,
                                                   uint32_t c_parasitic_ff);

/* Gray-Delta bijection: maps sequential delta to minimal Hamming weight */
static inline uint8_t ele_wire_encode_gray8(uint8_t val) {
  return (uint8_t)(val ^ (val >> 1U));
}

static inline uint8_t ele_wire_decode_gray8(uint8_t gray) {
  uint8_t v = gray;
  v ^= (uint8_t)(v >> 4U);
  v ^= (uint8_t)(v >> 2U);
  v ^= (uint8_t)(v >> 1U);
  return v;
}

/* Encode 8-bit payload into a single-edge cycle delay (1 flip = 8 bits) */
uint32_t ele_wire_transmit_ppm_byte(EleWireLink64 *link, uint8_t pin,
                                    uint8_t prev_byte, uint8_t next_byte);

/* Decode single-edge cycle delay back to exact 8-bit payload (0-ULP lossless)
 */
uint8_t ele_wire_receive_ppm_byte(const EleWireLink64 *link, uint8_t prev_byte,
                                  uint32_t measured_cycles);

#ifdef __cplusplus
}
#endif

#endif /* ELE_WIRE_H */
