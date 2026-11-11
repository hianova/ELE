#ifndef ELE_HVDC_SSCB_H
#define ELE_HVDC_SSCB_H

/* ============================================================================
 * ELE_HVDC_SSCB: 800V/48V Microsecond SiC Solid-State Circuit Breaker (SSCB)
 * ============================================================================
 * Prevents a single shorted blade/GPU capacitor from collapsing an entire
 * 800V HVDC or 48V rack busbar (sympathetic cluster brownout & DC plasma arc):
 *
 *   1. 2-Jet Inrush vs Short-Circuit Discrimination (< 0.5 us detection):
 *      Hot-plugging a healthy capacitive blade produces high I and dI/dt > 0,
 *      but strictly negative 2-Jet acceleration (d2I/dt2 < -margin) as the
 *      filter capacitor charges. A metallic busbar short maintains pinned
 *      driving voltage (dI/dt >= di_crit AND d2I/dt2 >= -margin), arming and
 *      tripping in 500 ns with 0 false hot-plug trips.
 *   2. Active Miller-Plateau Soft Turn-Off (< 1.5 us arc-free isolation):
 *      Abruptly interrupting kiloamps across bus inductance L_bus spikes
 *      V_ds = V_bus + L_bus * |dI/dt| past SiC breakdown (>1,200 V). Controlled
 *      soft-ramp turn-off bounds peak V_ds below avalanche breakdown while
 *      isolating the faulty slot to 0 A in <= 1.5 us.
 * ============================================================================
 */

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ELE_SSCB_CLOSED = 0,
  ELE_SSCB_SOFT_TURNOFF = 1,
  ELE_SSCB_OPEN_ISOLATED = 2
} EleSscbState;

typedef struct __attribute__((aligned(64))) {
  uint32_t v_bus_nom_v;         /*  4B: [0..3]   Nominal DC bus (800V / 48V)  */
  uint32_t v_sw_breakdown_v;    /*  4B: [4..7]   SiC avalanche rating (V)     */
  uint32_t l_bus_nh;            /*  4B: [8..11]  Branch loop inductance (nH)  */
  uint32_t i_jet_arm_a;         /*  4B: [12..15] 2-Jet arming current (Amps)  */
  uint32_t i_hard_trip_a;       /*  4B: [16..19] Instant desat ceiling (Amps) */
  int32_t di_crit_a_tick;       /*  4B: [20..23] Critical dI/dt (A / 250ns)   */
  int32_t ddi_inrush_margin;    /*  4B: [24..27] RC deceleration margin       */
  uint32_t soft_off_a_tick;     /*  4B: [28..31] Soft turn-off ramp (A/250ns) */
  uint32_t cur_i_amp;           /*  4B: [32..35] Active branch current (Amps) */
  int32_t di_a_tick;            /*  4B: [36..39] 1-Jet velocity (A / 250ns)   */
  int32_t ddi_a_tick2;          /*  4B: [40..43] 2-Jet acceleration           */
  uint32_t peak_fault_i_amp;    /*  4B: [44..47] Peak let-through fault (A)   */
  uint32_t peak_v_sw_soft_v;    /*  4B: [48..51] Peak V_ds soft turn-off (V)  */
  uint32_t peak_v_sw_hard_v;    /*  4B: [52..55] Counterfactual hard V_ds (V) */
  uint16_t gate_drive_permille; /*  2B: [56..57] SiC gate drive [0..1000]‰    */
  uint8_t clear_ticks_250ns;    /*  1B: [58]     Fault clearing ticks (250ns) */
  uint8_t primed;               /*  1B: [59]     1 = 2-Jet history primed     */
  uint8_t state;                /*  1B: [60]     EleSscbState                 */
  uint8_t fault_streak;         /*  1B: [61]     Consecutive 2-Jet fault hits */
  uint8_t slot_healthy;         /*  1B: [62]     1 = healthy, 0 = isolated    */
  uint8_t inrush_ride_throughs; /*  1B: [63]     Hot-plug surges ridden thru  */
} EleSscb64;

ELE_STATIC_ASSERT(sizeof(EleSscb64) == 64U,
                  "EleSscb64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleSscb64) == 64U,
                  "EleSscb64 must be 64-byte aligned");

/* Initialize 64B 800V/48V SiC Solid-State Circuit Breaker descriptor */
void ele_sscb64_init(EleSscb64 *cb, uint32_t v_bus_nom_v,
                     uint32_t v_sw_breakdown_v, uint32_t l_bus_nh,
                     uint32_t i_jet_arm_a, uint32_t i_hard_trip_a,
                     int32_t di_crit_a_tick, int32_t ddi_inrush_margin,
                     uint32_t soft_off_a_tick);

/* Re-arm an isolated SSCB after physical blade replacement / fault clearance */
void ele_sscb_rearm(EleSscb64 *cb);

/* Step 1 sample tick (250 ns = 4 MHz): discriminates RC hot-plug inrush from
 * short-circuit fault via 2-Jet (I, dI, d2I) and executes soft turn-off.
 * Bit-for-bit identical (0-ULP) to `rtl/ele_sscb_trip.sv`.
 * Returns clamped branch current (Amps). */
uint32_t ele_sscb_step_250ns(EleSscb64 *cb, uint32_t measured_i_amp);

#ifdef __cplusplus
}
#endif

#endif /* ELE_HVDC_SSCB_H */
