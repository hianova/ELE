#ifndef ELE_GRID_H
#define ELE_GRID_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 8: Diurnal Duck-Curve 800V HVDC Microgrid & 60Hz/50Hz
 *               Grid-Forming (GFM) Synthetic-Inertia Swing Controller
 *
 * Bridges the physical mismatch between daytime solar generation and 24x7
 * industrial / AI datacenter load across both market regimes:
 *   - Deregulated Wholesale LMP Grids (US ERCOT / CAISO):
 *       Midday solar canyons drive wholesale prices <= $0/MWh (curtailment),
 *       followed by steep sunset ramps and 10x..50x evening scarcity spikes.
 *   - Single-Buyer Regulated Island Grids (Taiwan Taipower 60 Hz TOU):
 *       Zero cross-border HVDC inertia makes high midday PV penetration prone
 *       to dangerous RoCoF (df/dt) swings on cloud shading or generator trips,
 *       while 16:00-22:00 "Night Peak" strains gas turbines & distribution
 *       feeders.
 *
 * Physical Breakthroughs:
 *   1. Behind-the-Meter 800V HVDC Direct Solar-BESS-Compute Busbar:
 *      Eliminates legacy PV(DC)->AC->Grid->UPS(AC-DC-AC)->PFC(DC) triple
 *      conversion (~14.3% loss -> ~2.5% single-stage ZVS DC-DC loss) and keeps
 *      midday PV off congested utility feeders.
 *   2. Thermal Cooling Phase-Shift + Conserved Elastic Compute Sponge:
 *      Charges PCM/chilled-water thermal storage and runs batch AI training
 *      during midday solar peaks; turns off chillers and shifts elastic batch
 *      load during 16:00-22:00 evening peaks with 100% daily work conservation.
 *   3. 1-Cycle Zero-Crossing Grid-Forming (GFM) Virtual Synchronous Generator:
 *      Converts hardware AC period capture (`period_ns`) to 1-mHz precision
 *      frequency without FPU ops, injecting synthetic rotational inertia
 *      (-M * df/dt) and 1-cycle compute P-cap droop within 16.67 ms.
 * ============================================================================
 */

/* 1. 64-Byte Grid-Forming (GFM) Virtual Synchronous Generator & RoCoF Node */
typedef struct __attribute__((aligned(64))) {
  uint32_t nom_freq_mhz;      /* [0..3]   Nominal AC frequency (60000/50000) */
  uint32_t cur_freq_mhz;      /* [4..7]   Measured zero-crossing freq (mHz) */
  int32_t rocof_mhz_s;        /* [8..11]  Measured RoCoF df/dt (mHz/s) */
  int32_t rotor_phase_mrad;   /* [12..15] Virtual synchronous angle (mrad) */
  uint32_t m_inertia_kw_s_hz; /* [16..19] Synthetic inertia gain M (kW*s/Hz) */
  uint32_t k_droop_kw_hz;     /* [20..23] Primary droop gain D (kW/Hz) */
  int32_t p_setpoint_kw;      /* [24..27] Dispatch active power setpoint (kW) */
  int32_t p_inject_kw;        /* [28..31] Instantaneous GFM power (+dis/-chg) */
  uint32_t max_power_kw;      /* [32..35] Bidirectional inverter rating (kW) */
  uint16_t compute_pcap_permille; /* [36..37] Compute P-cap (400..1150) */
  uint16_t deadband_mhz;          /* [38..39] Frequency deadband (mHz) */
  uint32_t nadir_gfl_mhz;         /* [40..43] Baseline GFL (H=0) worst nadir */
  uint32_t nadir_gfm_mhz;         /* [44..47] GFM synthetic-inertia nadir */
  uint64_t inertia_energy_kj;     /* [48..55] Cumulative inertia energy (kJ) */
  uint64_t cycles_stabilized;     /* [56..63] Total AC cycles governed */
} EleGridInverter64;

ELE_STATIC_ASSERT(sizeof(EleGridInverter64) == 64U,
                  "EleGridInverter64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleGridInverter64) == 64U,
                  "EleGridInverter64 must be 64-byte aligned");

/* 2. 64-Byte Diurnal Duck-Curve 800V HVDC Solar-BESS-Thermal-Compute Node */
typedef struct __attribute__((aligned(64))) {
  uint32_t bess_cap_kwh;          /* [0..3]   LFP battery capacity (kWh) */
  uint32_t bess_soc_kwh;          /* [4..7]   Current LFP battery SoC (kWh) */
  uint32_t thermal_cap_kwh;       /* [8..11]  PCM/chilled-water cap (kWh_th) */
  uint32_t thermal_soc_kwh;       /* [12..15] Current thermal cooling SoC */
  uint32_t grid_feeder_cap_kw;    /* [16..19] Utility feeder contract cap */
  uint32_t bess_max_rate_kw;      /* [20..23] Max BESS charge/discharge (kW) */
  uint32_t batch_backlog_kwh;     /* [24..27] Conserved elastic AI batch work */
  uint32_t peak_evening_base_kw;  /* [28..31] Legacy rigid evening peak (kW) */
  uint32_t peak_evening_symp_kw;  /* [32..35] Symplectic HVDC evening peak */
  uint32_t curtailed_pv_base_kwh; /* [36..39] Legacy curtailed/reverse PV */
  uint32_t curtailed_pv_symp_kwh; /* [40..43] Symplectic curtailed PV (kWh) */
  uint32_t ac_conv_waste_kwh;     /* [44..47] Saved AC-DC-AC-DC waste (kWh) */
  int64_t cost_baseline_milli;    /* [48..55] Legacy 24h tariff cost (signed) */
  int64_t cost_symplectic_milli;  /* [56..63] Symplectic HVDC 24h tariff cost */
} EleDuckMicrogrid64;

ELE_STATIC_ASSERT(sizeof(EleDuckMicrogrid64) == 64U,
                  "EleDuckMicrogrid64 must be exactly 64 bytes (1 cache line)");
ELE_STATIC_ASSERT(ELE_ALIGNOF(EleDuckMicrogrid64) == 64U,
                  "EleDuckMicrogrid64 must be 64-byte aligned");

/* Convert hardware zero-crossing period (ns) to integer millihertz (0 float) */
uint32_t ele_grid_period_ns_to_mhz(uint32_t period_ns);

/* Initialize 60Hz/50Hz Grid-Forming Virtual Synchronous Generator */
void ele_grid_inverter64_init(EleGridInverter64 *inv, uint32_t nom_freq_mhz,
                              uint32_t max_power_kw, uint32_t m_inertia_kw_s_hz,
                              uint32_t k_droop_kw_hz, uint16_t deadband_mhz);

/* Execute 1 AC-cycle GFM swing step + 1-cycle compute P-cap frequency droop */
int32_t ele_grid_step_gfm_cycle(EleGridInverter64 *inv, uint32_t period_ns,
                                int32_t grid_disturbance_kw,
                                uint32_t base_grid_stiffness_kw_hz);

/* Initialize Diurnal 800V HVDC Solar-BESS-Thermal-Compute Microgrid */
void ele_grid_duck64_init(EleDuckMicrogrid64 *mg, uint32_t bess_cap_kwh,
                          uint32_t bess_init_kwh, uint32_t thermal_cap_kwh,
                          uint32_t thermal_init_kwh,
                          uint32_t grid_feeder_cap_kw,
                          uint32_t bess_max_rate_kw);

/* Step 1-hour diurnal dispatch across PV, BESS, Thermal Storage, and AI Load.
 * Supports both negative wholesale LMPs (US ERCOT/CAISO) and regulated evening
 * Night-Peak TOU tariffs (Taiwan Taipower). Returns net utility grid import kW.
 */
uint32_t ele_grid_step_duck_hour(EleDuckMicrogrid64 *mg, uint32_t pv_avail_kw,
                                 uint32_t rigid_compute_kw,
                                 uint32_t elastic_batch_kw,
                                 uint32_t cooling_elec_kw,
                                 int32_t price_milli_per_kwh,
                                 bool is_evening_peak);

#ifdef __cplusplus
}
#endif

#endif /* ELE_GRID_H */
