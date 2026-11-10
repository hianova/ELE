#ifndef ELE_DRIVE_H
#define ELE_DRIVE_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 3: Symplectic Pulse-and-Coast Motor & Energy-Harvesting Controller
 * Hardware Targets:
 *   - Actuator: OT-RK-380PH Brushed DC Motor + L298N Dual H-Bridge
 *   - Feedback: Back-EMF (during High-Z Coast), Piezo Strain Harvester,
 *               Dual Microphones (Acoustic Resonance), HTU21D (Heatsink dT/dt)
 * Why Traditional L298N Fixed-PWM Wastes 40~60% Energy:
 *   1. L298N Darlington BJT drops V_sat (~1.8V) continuously during conduction.
 *   2. Blind PWM switches against rotor Back-EMF and brush commutation spikes,
 *      dumping (1/2) L I^2 inductive flyback into heat and mechanical rattle.
 * Symplectic Pulse-and-Coast Improvement:
 *   - Injects short full-conduction pulses only when Back-EMF drops below
 *     the symplectic target band and commutator/acoustic phase is quiet;
 *   - Immediately floats L298N (EN = 0, High-Z Coast) so I = 0 and V_sat loss
 *     drops to 0 W while rotor angular momentum carries the load;
 *   - Harvests Piezo mechanical vibration & inductive flyback during coast.
 * ============================================================================
 */

typedef enum {
  ELE_DRIVE_COAST_HIGH_Z =
      0, /* L298N EN=0: 0 current, rotor coasts & senses BEMF */
  ELE_DRIVE_SYMPLECTIC_PULSE = 1, /* Phase-locked optimal torque pulse */
  ELE_DRIVE_REGEN_HARVEST_BRAKE =
      2 /* Deceleration kinetic/piezo energy recovery */
} EleDrivePhase;

/* 64-Byte Univalent Symplectic Electromechanical Controller Descriptor */
typedef struct __attribute__((aligned(64))) {
  uint32_t v_supply_mv; /* [0..3]   Supply voltage (e.g. 7400 or 12000 mV) */
  uint32_t v_bemf_mv; /* [4..7]   Measured rotor Back-EMF (mV, prop. to RPM) */
  uint32_t target_bemf_mv; /* [8..11]  Target speed Back-EMF setpoint (mV) */
  uint32_t
      v_sat_drop_mv; /* [12..15] L298N Darlington saturation drop (~1800 mV) */
  uint32_t coil_r_mohm;     /* [16..19] OT-RK-380PH winding resistance (mOhm) */
  uint32_t coil_current_ma; /* [20..23] Instantaneous coil current (mA) */
  uint32_t piezo_peak_mv;   /* [24..27] Piezo vibration sensor voltage (mV) */
  uint32_t
      mic_diff_rms_uv; /* [28..31] Dual-microphone acoustic loss metric (uV) */
  int32_t
      heatsink_dtemp_mk_s; /* [32..35] HTU21D heatsink thermal slope (mK/s) */
  uint16_t
      pulse_duty_permille; /* [36..37] Active pulse duty (0..1000 permille) */
  uint8_t active_phase;    /* [38]     EleDrivePhase current state */
  uint8_t commutator_quiet_flag; /* [39]     1 = brush phase quiet, 0 = spark
                                    transient */
  int64_t energy_input_uj_q16;   /* [40..47] Cumulative electrical input energy
                                    (uJ, Q48.16) */
  int64_t energy_saved_uj_q16;  /* [48..55] Energy saved vs continuous fixed PWM
                                   (uJ, Q48.16) */
  int64_t piezo_harvest_uj_q16; /* [56..63] Harvested Piezo + flyback energy
                                   (uJ, Q48.16) */
} EleMotorDrive64;

_Static_assert(sizeof(EleMotorDrive64) == 64,
               "EleMotorDrive64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleMotorDrive64) == 64,
               "EleMotorDrive64 must be 64-byte aligned");

/* Initialize OT-RK-380PH + L298N symplectic drive controller */
void ele_drive_motor64_init(EleMotorDrive64 *drv, uint32_t v_supply_mv,
                            uint32_t target_bemf_mv, uint32_t coil_r_mohm);

/* Evaluate instantaneous parasitic waste power (mW):
 *   P_loss = I * V_sat(L298N) + I^2 * R_coil + P_vib(Piezo) + P_acoustic(Mics)
 */
uint32_t ele_drive_eval_waste_power_mw(const EleMotorDrive64 *drv,
                                       uint32_t current_ma);

/* Execute 1 deterministic control step (dt_us microseconds):
 *   - Chooses COAST_HIGH_Z vs SYMPLECTIC_PULSE vs REGEN_HARVEST_BRAKE
 *   - Updates rotor Back-EMF, harvested Piezo energy, and energy savings vs PWM
 */
EleDrivePhase ele_drive_step_symplectic(EleMotorDrive64 *drv,
                                        uint32_t measured_bemf_mv,
                                        uint32_t piezo_mv, uint32_t mic_diff_uv,
                                        int32_t heatsink_dtemp_mk_s,
                                        uint32_t dt_us);

#ifdef __cplusplus
}
#endif

#endif /* ELE_DRIVE_H */
