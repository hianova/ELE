// SPDX-License-Identifier: MIT
// Cycle-Accurate Verilator RTL Co-Simulation & Register-Toggle Calorimeter
// Verifies synthesized SystemVerilog hardware IP bit-for-bit (0-ULP) against
// ELE's C17 golden physics kernel (`libele.a`) across 4,096+ clock cycles.

#include "Vele_silicon_top.h"
#include "verilated.h"

#include "../include/ele_event_adc.h"
#include "../include/ele_grid.h"
#include "../include/ele_hbm_refresh.h"
#include "../include/ele_hvdc_sscb.h"
#include "../include/ele_ivr_fll.h"
#include "../include/ele_pdn_emi.h"
#include "../include/ele_photonics_cpo.h"
#include "../include/ele_rack_vrm.h"
#include "../include/ele_resonant_clk.h"
#include "../include/ele_silicon.h"
#include "../include/ele_silicon_slm.h"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#define ELE_RTL_ASSERT(cond)                                                   \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "RTL FAIL: %s:%d: %s\n", __FILE__, __LINE__,        \
                   #cond);                                                     \
      std::abort();                                                            \
    }                                                                          \
    rtl_assertions += 1U;                                                      \
  } while (0)

static inline uint32_t popcnt64(uint64_t x) {
  return (uint32_t)__builtin_popcountll(x);
}

static inline int64_t sext48(uint64_t v) {
  uint64_t m = v & 0x0000FFFFFFFFFFFFULL;
  return (m & (1ULL << 47U)) ? (int64_t)(m | 0xFFFF000000000000ULL)
                             : (int64_t)m;
}

static inline int32_t sext18(uint32_t v) {
  uint32_t m = v & 0x0003FFFFU;
  return (m & (1U << 17U)) ? (int32_t)(m | 0xFFFC0000U) : (int32_t)m;
}

static void tick_clk(Vele_silicon_top *dut) {
  dut->clk = 0;
  dut->eval();
  dut->clk = 1;
  dut->eval();
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Vele_silicon_top *dut = new Vele_silicon_top;
  uint32_t rtl_assertions = 0U;

  // Reset all RTL domains
  dut->clk = 0;
  dut->rst_n = 0;
  dut->axi_s_valid_i = 0;
  dut->axi_s_data_i = 0ULL;
  dut->axi_m_ready_i = 1;
  dut->mac_clr_i = 0;
  dut->mac_valid_i = 0;
  dut->mac_readout_en_i = 0;
  dut->mac_act_q16_i = 0;
  dut->mac_trit_i = 0;
  dut->mac_weight_i16_i = 0;
  dut->vrm_valid_i = 0;
  dut->vrm_load_amp_i = 0;
  dut->vrm_upcoming_amp_i = 0;
  dut->vrm_amp_per_phase_i = 75;
  dut->vrm_zvs_err_ps_i = 0;
  dut->grid_zc_valid_i = 0;
  dut->grid_freq_mhz_i = 60000;
  dut->grid_nom_freq_mhz_i = 60000;
  dut->grid_deadband_mhz_i = 30;
  dut->grid_p_set_kw_i = 0;
  dut->grid_max_kw_i = 25000;
  dut->fll_valid_i = 0;
  dut->fll_t_nom_ps_i = 500;
  dut->fll_v_nom_mv_i = 750;
  dut->fll_v_inst_mv_i = 750;
  dut->sscb_valid_i = 0;
  dut->sscb_rearm_i = 0;
  dut->sscb_i_meas_amp_i = 0;
  dut->sscb_i_jet_arm_a_i = 300;
  dut->sscb_i_hard_trip_a_i = 1200;
  dut->sscb_di_crit_a_i = 80;
  dut->sscb_ddi_margin_a_i = 15;
  dut->sscb_soft_off_a_i = 140;
  dut->slm_valid_i = 0;
  dut->slm_rearm_trace_i = 0;
  dut->slm_fault_trig_i = 0;
  dut->slm_pc16_i = 0;
  dut->slm_v_dd_mv_i = 750;
  dut->slm_hw_flags4_i = 0;
  dut->slm_v_eps_mv_i = 5;
  dut->slm_snap_rd_idx_i = 0;
  dut->slm_t_clk_ps_i = 500;
  dut->slm_t_main_aged_ps_i = 470;
  dut->slm_canary_guard_ps_i = 35;
  dut->cpo_valid_i = 0;
  dut->cpo_target_phase12_i = 1024;
  dut->cpo_thermal_phase12_i = 0;
  dut->cpo_ff_delta_phase_i = 0;
  dut->cpo_uw_per_lsb_i = 4;
  dut->cpo_lock_tol12_i = 64;
  dut->cpo_actual_wave_phase12_i = 0;
  dut->hbm_act_valid_i = 0;
  dut->hbm_win_rst_i = 0;
  dut->hbm_row_id_i = 0;
  dut->hbm_drfm_thresh_i = 16;
  dut->rclk_valid_i = 0;
  dut->rclk_en_i = 0;
  dut->rclk_v_dd_mv_i = 750;
  dut->rclk_target_pulse_ps_i = 120;
  dut->rclk_zvs_tol_ps_i = 25;
  dut->rclk_zvs_err_ps_i = 0;
  dut->eadc_valid_i = 0;
  dut->eadc_v_in_uv_i = 0;
  dut->eadc_lsb_step_uv_i = 5000;
  dut->eadc_hyst_uv_i = 2000;
  dut->eadc_lead_ps_i = 0;
  dut->eadc_t_slow_ps_i = 45;
  dut->eadc_t_fast_ps_i = 40;
  dut->emi_valid_i = 0;
  dut->emi_seed_i = 0xACE1U;
  dut->emi_nom_period_ps_i = 500;
  dut->emi_max_dither_ps_i = 32;
  dut->emi_damping_gain_q8_i = 64;
  dut->emi_di_excite_ma_i = 0;
  dut->eval();
  // AXI4-Stream reset safety: s_ready_o and m_valid_o MUST be 0 while !rst_n
  ELE_RTL_ASSERT(dut->axi_s_ready_o == 0U);
  ELE_RTL_ASSERT(dut->axi_m_valid_o == 0U);

  tick_clk(dut);
  dut->rst_n = 1;
  dut->axi_m_ready_i = 0;
  dut->eval();

  // ==========================================================================
  // 1. AXI4-Stream 65-Wire Bridge: 1,024 Beats w/ Random Bubbles & Backpressure
  // ==========================================================================
  uint64_t tx_vec[1024];
  for (uint32_t i = 0; i < 1024; ++i) {
    if (i < 256) {
      // Worst-case adversarial alternating 0x00..00 <-> 0xFF..FF stress
      tx_vec[i] = (i & 1U) ? 0xFFFFFFFFFFFFFFFFULL : 0x0000000000000000ULL;
    } else {
      // Correlated KV-cache / activation stream
      uint32_t delta = ((i * 17U) & 0xFFU);
      tx_vec[i] = 0x3C003C003C003C00ULL ^ ((uint64_t)delta << ((i & 3U) * 16U));
    }
  }

  uint32_t tx_idx = 0U;
  uint32_t rx_idx = 0U;
  uint64_t prev_phy_wire = dut->phy_wire_o;
  uint8_t prev_phy_inv = dut->phy_inv_o;
  uint64_t max_single_beat_flips = 0ULL;
  uint64_t total_phy_flips = 0ULL;
  uint64_t total_raw_flips = 0ULL;
  uint64_t prev_raw_word = 0ULL;
  uint32_t skip_beats = 0U;
  uint32_t prng = 0xC0FFEE11U;
  bool prev_stalled = false;
  uint64_t stalled_m_data = 0ULL;

  while (rx_idx < 1024U) {
    prng = prng * 1664525U + 1013904223U;
    uint8_t m_ready = ((prng & 3U) != 0U) ? 1U : 0U; // 25% backpressure
    uint8_t s_bubble = (((prng >> 4U) & 7U) == 0U) ? 1U : 0U; // 12.5% bubbles
    dut->axi_m_ready_i = m_ready;
    dut->axi_s_valid_i = (tx_idx < 1024U && s_bubble == 0U) ? 1U : 0U;
    dut->axi_s_data_i =
        (dut->axi_s_valid_i != 0U) ? tx_vec[tx_idx] : 0xDEADDEADDEADDEADULL;
    dut->eval();

    if (prev_stalled) {
      // AXI4-Stream TVALID/TDATA stability invariant across backpressure stalls
      ELE_RTL_ASSERT(dut->axi_m_valid_o == 1U);
      ELE_RTL_ASSERT(dut->axi_m_data_o == stalled_m_data);
    }

    bool s_fire = (dut->axi_s_valid_i != 0U) && (dut->axi_s_ready_o != 0U);
    bool m_fire = (dut->axi_m_valid_o != 0U) && (dut->axi_m_ready_i != 0U);
    prev_stalled = (dut->axi_m_valid_o != 0U) && (dut->axi_m_ready_i == 0U);
    stalled_m_data = dut->axi_m_data_o;

    if (s_fire && dut->phy_zero_skip_o != 0U) {
      skip_beats += 1U;
    }
    if (m_fire) {
      ELE_RTL_ASSERT(dut->axi_m_data_o == tx_vec[rx_idx]);
      rx_idx += 1U;
    }

    tick_clk(dut);

    if (s_fire) {
      uint64_t beat_flips = popcnt64(dut->phy_wire_o ^ prev_phy_wire) +
                            (uint64_t)((dut->phy_inv_o ^ prev_phy_inv) & 1U);
      // Hardware Theorem 13: NEVER exceeds 32 wire transitions out of 65!
      ELE_RTL_ASSERT(beat_flips <= 32ULL);
      if (beat_flips > max_single_beat_flips) {
        max_single_beat_flips = beat_flips;
      }
      total_phy_flips += beat_flips;
      total_raw_flips += popcnt64(tx_vec[tx_idx] ^ prev_raw_word);
      prev_raw_word = tx_vec[tx_idx];
      prev_phy_wire = dut->phy_wire_o;
      prev_phy_inv = dut->phy_inv_o;
      tx_idx += 1U;
    } else {
      // Idle/bubble physical bus must burn 0 transitions
      ELE_RTL_ASSERT(dut->phy_wire_o == prev_phy_wire);
      ELE_RTL_ASSERT(dut->phy_inv_o == prev_phy_inv);
    }
  }
  dut->axi_s_valid_i = 0;
  dut->axi_m_ready_i = 0;

  // ==========================================================================
  // 2. Dual-Rail 1.58b Ternary MAC vs Two's-Complement & Int32*Int16 RTL + C17
  //    64 output tiles x K=32 zero-mean LayerNorm inner products (2,048 cyc)
  // ==========================================================================
  uint64_t prev_pos_q = dut->mac_pos_rail_o;
  uint64_t prev_neg_q = dut->mac_neg_rail_o;
  uint64_t prev_twos_q = dut->mac_twos_ref_acc_o;
  uint64_t dual_ff_flips = 0ULL;
  uint64_t twos_ff_flips = 0ULL;
  uint32_t zero_skips = 0U;

  for (uint32_t tile = 0U; tile < 64U; ++tile) {
    int32_t tile_acts[32];
    int8_t tile_trits[32];

    dut->mac_clr_i = 1;
    dut->mac_valid_i = 0;
    dut->mac_readout_en_i = 0;
    tick_clk(dut);
    dut->mac_clr_i = 0;

    dual_ff_flips += popcnt64(dut->mac_pos_rail_o ^ prev_pos_q) +
                     popcnt64(dut->mac_neg_rail_o ^ prev_neg_q);
    twos_ff_flips += popcnt64(dut->mac_twos_ref_acc_o ^ prev_twos_q);
    prev_pos_q = dut->mac_pos_rail_o;
    prev_neg_q = dut->mac_neg_rail_o;
    prev_twos_q = dut->mac_twos_ref_acc_o;

    int32_t base_mag = 4096;
    for (uint32_t k = 0U; k < 32U; ++k) {
      prng = prng * 1664525U + 1013904223U;
      if ((k & 1U) == 0U) {
        base_mag = (int32_t)((prng >> 8U) & 0x3FFFU) + 1024;
      }
      int32_t jitter = (int32_t)((prng >> 24U) & 0xFFU) - 128;
      int32_t act = ((k & 1U) == 0U) ? base_mag : (-base_mag + jitter);
      int8_t w = 0;
      if ((k & 3U) == 0U || (k & 3U) == 1U) {
        w = 1; // Paired +/- activation -> zero-mean residual oscillation
      } else {
        w = ((prng & 3U) == 0U) ? ((prng & 4U) ? 1 : -1) : 0; // ~50% sparsity
      }
      tile_acts[k] = act;
      tile_trits[k] = w;

      dut->mac_valid_i = 1;
      dut->mac_readout_en_i = (k == 31U) ? 1U : 0U;
      dut->mac_act_q16_i = (uint32_t)act;
      dut->mac_trit_i = (w == 0) ? 0U : ((w > 0) ? 1U : 2U);
      dut->mac_weight_i16_i = (uint16_t)(int16_t)w;
      dut->eval();
      if (dut->mac_zero_skip_o != 0U) {
        zero_skips += 1U;
      }
      if (k < 31U) {
        // Operand-isolated terminal subtractor MUST stay 0 on intermediate
        // beats
        ELE_RTL_ASSERT(dut->mac_dual_acc_o == 0ULL);
      }

      tick_clk(dut);

      dual_ff_flips += popcnt64(dut->mac_pos_rail_o ^ prev_pos_q) +
                       popcnt64(dut->mac_neg_rail_o ^ prev_neg_q);
      twos_ff_flips += popcnt64(dut->mac_twos_ref_acc_o ^ prev_twos_q);
      prev_pos_q = dut->mac_pos_rail_o;
      prev_neg_q = dut->mac_neg_rail_o;
      prev_twos_q = dut->mac_twos_ref_acc_o;
    }

    dut->mac_readout_en_i = 1;
    dut->eval();
    int64_t dual_acc = sext48(dut->mac_dual_acc_o);
    int64_t twos_acc = sext48(dut->mac_twos_ref_acc_o);
    int64_t mul_acc = sext48(dut->mac_mul_ref_acc_o);
    int64_t c17_sum = ele_silicon_dot_ternary_sign_grouped(
        tile_acts, tile_trits, 32U, nullptr);
    ELE_RTL_ASSERT(dual_acc == twos_acc);
    ELE_RTL_ASSERT(dual_acc == mul_acc);
    ELE_RTL_ASSERT(dual_acc == c17_sum);
  }
  dut->mac_valid_i = 0;
  dut->mac_readout_en_i = 0;
  ELE_RTL_ASSERT(dual_ff_flips * 2ULL < twos_ff_flips);

  // ==========================================================================
  // 3. 16-Phase 48V Digital VRM ZVS & Active Pre-Charge (0-ULP C17 Lockstep!)
  // ==========================================================================
  EleVrmPhase64 c17_vrm;
  ele_vrm64_init(&c17_vrm, 48000U, 750U, 16U, 800U, 3600U, 12000U, 15U);

  int32_t zvs_err = 1605; // Odd error exercises signed floor arithmetic shifts
  for (uint32_t cyc = 0; cyc < 64U; ++cyc) {
    uint16_t load_a = (cyc >= 24U && cyc < 40U) ? 1050U : 60U;
    uint16_t up_a = (cyc >= 22U && cyc < 40U) ? 1050U : 60U;
    dut->vrm_valid_i = 1;
    dut->vrm_load_amp_i = load_a;
    dut->vrm_upcoming_amp_i = up_a;
    dut->vrm_amp_per_phase_i = 75U;
    dut->vrm_zvs_err_ps_i = (uint16_t)(int16_t)zvs_err;
    tick_clk(dut);
    (void)ele_vrm_step_us(&c17_vrm, load_a, up_a, zvs_err);

    // Bit-exact 0-ULP C17 <-> RTL lockstep on ZVS dead-time & lock state!
    ELE_RTL_ASSERT(dut->vrm_t_dead_ps_o == c17_vrm.t_dead_ps);
    ELE_RTL_ASSERT(dut->vrm_zvs_locked_o == c17_vrm.zvs_locked);
    ELE_RTL_ASSERT(dut->vrm_precharge_active_o == c17_vrm.precharge_armed);

    if (cyc < 20U) {
      ELE_RTL_ASSERT(dut->vrm_active_phases_o == 1U);
      ELE_RTL_ASSERT(dut->vrm_phase_mask_o == 0x0001U);
    } else if (cyc == 22U || cyc == 23U) {
      ELE_RTL_ASSERT(dut->vrm_precharge_active_o == 1U);
      ELE_RTL_ASSERT(dut->vrm_active_phases_o == 16U);
      ELE_RTL_ASSERT(dut->vrm_phase_mask_o == 0xFFFFU);
    }
    zvs_err = (int32_t)dut->vrm_t_dead_ps_o - 6203;
  }
  ELE_RTL_ASSERT(dut->vrm_zvs_locked_o == 1U);

  // Adversarial 16-bit wrap boundary check: 65500 A + 64 = 65564 > 65535
  dut->vrm_load_amp_i = 65500U;
  dut->vrm_upcoming_amp_i = 65535U;
  tick_clk(dut);
  ELE_RTL_ASSERT(dut->vrm_precharge_active_o == 0U);
  dut->vrm_valid_i = 0;

  // ==========================================================================
  // 4. 60Hz/50Hz Grid-Forming (GFM) Virtual Synchronous Generator & Sponge RTL
  // ==========================================================================
  dut->grid_zc_valid_i = 1;
  dut->grid_nom_freq_mhz_i = 60000U;
  dut->grid_deadband_mhz_i = 30U;
  dut->grid_p_set_kw_i = 0U;
  dut->grid_max_kw_i = 25000U;

  // Phase A: Midday 60.200 Hz solar overgeneration -> BESS charges & 115% P-cap
  dut->grid_freq_mhz_i = (uint16_t)ele_grid_period_ns_to_mhz(16611296U);
  tick_clk(dut);
  int32_t p_sponge_rtl = sext18(dut->grid_p_inject_kw_o);
  ELE_RTL_ASSERT(p_sponge_rtl < -2000);
  ELE_RTL_ASSERT(dut->grid_solar_sponge_o == 1U);
  ELE_RTL_ASSERT(dut->grid_compute_pcap_o == 1150U);

  // Phase B: Nominal 60.000 Hz steady state -> 100% P-cap (zero false arrest!)
  dut->grid_freq_mhz_i = (uint16_t)ele_grid_period_ns_to_mhz(16666667U);
  tick_clk(dut);
  ELE_RTL_ASSERT(dut->grid_solar_sponge_o == 0U);
  ELE_RTL_ASSERT(dut->grid_ufls_arrest_o == 0U);
  ELE_RTL_ASSERT(dut->grid_compute_pcap_o == 1000U);

  // Phase C: Sudden island contingency drop to 59.780 Hz (16,728,003 ns)
  dut->grid_freq_mhz_i = (uint16_t)ele_grid_period_ns_to_mhz(16728003U);
  tick_clk(dut);
  int32_t p_arrest_rtl = sext18(dut->grid_p_inject_kw_o);
  uint16_t pcap_arrest_rtl = dut->grid_compute_pcap_o;
  ELE_RTL_ASSERT(p_arrest_rtl == 25000);
  ELE_RTL_ASSERT(dut->grid_ufls_arrest_o == 1U);
  ELE_RTL_ASSERT(pcap_arrest_rtl <= 800U);

  // Phase D: High-capacity >32.7 MW saturation & deep under-frequency safety
  dut->grid_max_kw_i = 40000U;
  dut->grid_freq_mhz_i = 44573U;
  tick_clk(dut);
  ELE_RTL_ASSERT(sext18(dut->grid_p_inject_kw_o) == 40000);
  ELE_RTL_ASSERT(dut->grid_compute_pcap_o == 550U);

  // Phase E: Glitch-free 50.000 Hz cold-start reset check
  dut->rst_n = 0;
  tick_clk(dut);
  dut->rst_n = 1;
  dut->grid_nom_freq_mhz_i = 50000U;
  dut->grid_freq_mhz_i = 50000U;
  dut->grid_max_kw_i = 40000U;
  tick_clk(dut);
  ELE_RTL_ASSERT(sext18(dut->grid_p_inject_kw_o) == 0);
  ELE_RTL_ASSERT((int16_t)dut->grid_rocof_step_mhz_o == 0);
  ELE_RTL_ASSERT(dut->grid_ufls_arrest_o == 0U);
  ELE_RTL_ASSERT(dut->grid_compute_pcap_o == 1000U);
  dut->grid_zc_valid_i = 0;

  // ==========================================================================
  // 5. <1ns 1-Cycle Droop-Adaptive AD-FLL Clock-Stretcher RTL (0-ULP == C17)
  // ==========================================================================
  dut->fll_valid_i = 1;
  dut->fll_t_nom_ps_i = 500U;
  dut->fll_v_nom_mv_i = 750U;
  for (uint32_t droop = 0U; droop <= 320U; ++droop) {
    uint16_t v_inst = (uint16_t)(750U - droop);
    dut->fll_v_inst_mv_i = v_inst;
    tick_clk(dut);

    bool c_emerg = false;
    uint32_t c_period =
        ele_fll_eval_period_ps(500U, 750U, (uint32_t)v_inst, &c_emerg);
    ELE_RTL_ASSERT(dut->fll_t_clk_ps_o == c_period);
    ELE_RTL_ASSERT(dut->fll_emergency_gate_o == (c_emerg ? 1U : 0U));
    ELE_RTL_ASSERT(dut->fll_stretched_o == ((c_period > 500U) ? 1U : 0U));

    if (!c_emerg) {
      uint32_t t_crit =
          ele_fll_eval_crit_delay_ps(500U, 750U, (uint32_t)v_inst, 238U);
      ELE_RTL_ASSERT(dut->fll_t_clk_ps_o >= t_crit);
    }
  }
  // Adversarial 16-bit full-scale period check (65535 * 4 = 262140 ps, 0 wrap)
  dut->fll_t_nom_ps_i = 65535U;
  dut->fll_v_nom_mv_i = 750U;
  dut->fll_v_inst_mv_i = 400U;
  tick_clk(dut);
  ELE_RTL_ASSERT(dut->fll_emergency_gate_o == 1U);
  ELE_RTL_ASSERT(dut->fll_t_clk_ps_o == 262140U);
  dut->fll_valid_i = 0;

  // ==========================================================================
  // 6. 800V/48V Microsecond SiC Solid-State Circuit Breaker RTL (0-ULP == C17)
  // ==========================================================================
  EleSscb64 c_sscb;
  ele_sscb64_init(&c_sscb, 800U, 1200U, 600U, 300U, 1200U, 80, 15, 140U);

  dut->sscb_valid_i = 1;
  dut->sscb_rearm_i = 0;
  dut->sscb_i_jet_arm_a_i = 300U;
  dut->sscb_i_hard_trip_a_i = 1200U;
  dut->sscb_di_crit_a_i = 80;
  dut->sscb_ddi_margin_a_i = 15U;
  dut->sscb_soft_off_a_i = 140U;

  // 250 ns waveform:
  //   [0..1]   120 A cold-start steady load (0 false cold-start dI/dt spike)
  //   [2..10]  Hot-plug RC capacitive inrush peaking at 580 A (d2I < -15 -> OK)
  //   [11..19] Metallic 800V busbar short (+200 A/250ns -> 500ns trip -> 0 A)
  const uint16_t sscb_wave[20] = {120U, 120U,  260U,  440U,  540U,  580U, 580U,
                                  460U, 300U,  180U,  120U,  320U,  520U, 720U,
                                  920U, 1120U, 1320U, 1520U, 1720U, 1920U};
  for (uint32_t k = 0U; k < 20U; ++k) {
    dut->sscb_i_meas_amp_i = sscb_wave[k];
    tick_clk(dut);
    uint32_t c_cur = ele_sscb_step_250ns(&c_sscb, (uint32_t)sscb_wave[k]);
    ELE_RTL_ASSERT(dut->sscb_cur_i_amp_o == c_cur);
    ELE_RTL_ASSERT(dut->sscb_gate_permille_o == c_sscb.gate_drive_permille);
    ELE_RTL_ASSERT(dut->sscb_clear_ticks_o == c_sscb.clear_ticks_250ns);
    ELE_RTL_ASSERT(dut->sscb_state_o == c_sscb.state);
    ELE_RTL_ASSERT(dut->sscb_slot_healthy_o == c_sscb.slot_healthy);
    ELE_RTL_ASSERT(dut->sscb_inrush_cnt_o == c_sscb.inrush_ride_throughs);
    if (k == 10U) {
      // Survived 580 A capacitive hot-plug without tripping!
      ELE_RTL_ASSERT(dut->sscb_slot_healthy_o == 1U);
      ELE_RTL_ASSERT(dut->sscb_inrush_cnt_o >= 2U);
    }
  }
  ELE_RTL_ASSERT(dut->sscb_state_o == (uint8_t)ELE_SSCB_OPEN_ISOLATED);
  ELE_RTL_ASSERT(dut->sscb_slot_healthy_o == 0U);
  ELE_RTL_ASSERT(dut->sscb_clear_ticks_o <= 6U); // <= 1.50 us arc-free clear!

  // Re-arm and verify 1-tick (250 ns) instant hard desaturation trip at 1500 A
  dut->sscb_rearm_i = 1;
  tick_clk(dut);
  ele_sscb_rearm(&c_sscb);
  dut->sscb_rearm_i = 0;
  ELE_RTL_ASSERT(dut->sscb_slot_healthy_o == 1U);
  ELE_RTL_ASSERT(dut->sscb_state_o == (uint8_t)ELE_SSCB_CLOSED);

  dut->sscb_i_meas_amp_i = 1500U;
  tick_clk(dut);
  (void)ele_sscb_step_250ns(&c_sscb, 1500U);
  ELE_RTL_ASSERT(dut->sscb_state_o == (uint8_t)ELE_SSCB_SOFT_TURNOFF);
  ELE_RTL_ASSERT(dut->sscb_gate_permille_o == 350U);
  dut->sscb_valid_i = 0;

  // ==========================================================================
  // 7. Zero-Stall 2-Jet Silicon Trace Flight Recorder & Aging Canary (0-ULP)
  // ==========================================================================
  EleTraceBox64 c_box;
  ele_trace_box64_init(&c_box, 5U);

  dut->slm_valid_i = 1;
  dut->slm_rearm_trace_i = 0;
  dut->slm_v_eps_mv_i = 5U;
  dut->slm_t_clk_ps_i = 500U;
  dut->slm_t_main_aged_ps_i = 450U;
  dut->slm_canary_guard_ps_i = 35U;

  for (uint32_t cyc = 0U; cyc < 256U; ++cyc) {
    uint16_t pc16 = 0U;
    uint16_t v_dd = 750U;
    uint8_t flags4 = (uint8_t)(cyc & 0x0FU);
    bool fault_trig = false;

    if (cyc < 200U) {
      pc16 = (uint16_t)(0x1000U + cyc * 4U);
      v_dd = (uint16_t)(750U - (cyc / 8U));
    } else if (cyc == 200U) {
      pc16 = 0x4200U;
      v_dd = 680U;
    } else if (cyc < 208U) {
      pc16 = (uint16_t)(0x4200U + (cyc - 200U) * 4U);
      v_dd = (uint16_t)(680U + (cyc - 200U) * 4U);
    } else if (cyc == 208U) {
      pc16 = 0xDEADU;
      v_dd = 640U;
      flags4 = 0x0AU;
      fault_trig = true;
    } else {
      pc16 = (uint16_t)(0x8000U + (cyc - 209U) * 4U);
      v_dd = 750U;
    }

    dut->slm_pc16_i = pc16;
    dut->slm_v_dd_mv_i = v_dd;
    dut->slm_hw_flags4_i = flags4;
    dut->slm_fault_trig_i = fault_trig ? 1U : 0U;
    tick_clk(dut);

    bool c_emit = ele_trace_step_cycle(&c_box, pc16, v_dd, flags4, fault_trig);
    ELE_RTL_ASSERT(dut->slm_trace_emit_o == (c_emit ? 1U : 0U));
    ELE_RTL_ASSERT(dut->slm_frozen_o == c_box.frozen);
    ELE_RTL_ASSERT(dut->slm_wr_idx_o == c_box.wr_idx);
    ELE_RTL_ASSERT(dut->slm_freeze_idx_o == c_box.freeze_idx);
    ELE_RTL_ASSERT(dut->slm_suppressed_cnt_o ==
                   (uint16_t)c_box.suppressed_cycles);
    ELE_RTL_ASSERT(dut->slm_emitted_cnt_o == (uint16_t)c_box.emitted_packets);
  }
  ELE_RTL_ASSERT(c_box.suppressed_cycles >= 245U);
  ELE_RTL_ASSERT(dut->slm_frozen_o == 1U);

  // Read out all 8 frozen pre-crash nanosecond slots and compare 0-ULP vs C17
  for (uint8_t slot = 0U; slot < 8U; ++slot) {
    dut->slm_snap_rd_idx_i = slot;
    dut->eval();
    ELE_RTL_ASSERT(dut->slm_snap_rd_data_o == c_box.ring_snap[slot]);
  }
  ELE_RTL_ASSERT(ele_trace_read_ordered(&c_box, 7U) ==
                 ele_trace_pack_snap32(0xDEADU, 0x0AU, 640U));

  // Canary timing-slack comparator sweep across healthy -> pre-SDC -> SDC
  dut->slm_fault_trig_i = 0;
  for (uint32_t t_aged = 450U; t_aged <= 520U; ++t_aged) {
    dut->slm_t_main_aged_ps_i = t_aged;
    tick_clk(dut);
    uint8_t exp_warn = (t_aged + 35U > 500U) ? 1U : 0U;
    uint8_t exp_sdc = (t_aged > 500U) ? 1U : 0U;
    ELE_RTL_ASSERT(dut->slm_canary_warn_o == exp_warn);
    ELE_RTL_ASSERT(dut->slm_main_sdc_err_o == exp_sdc);
  }

  // Re-arm flight recorder and verify clean unlock
  dut->slm_rearm_trace_i = 1;
  tick_clk(dut);
  ele_trace_box_rearm(&c_box);
  dut->slm_rearm_trace_i = 0;
  dut->slm_valid_i = 0;
  ELE_RTL_ASSERT(dut->slm_frozen_o == 0U);
  ELE_RTL_ASSERT(dut->slm_wr_idx_o == 0U);
  ELE_RTL_ASSERT(dut->slm_freeze_idx_o == 0U);

  // ==========================================================================
  // 8. CPO Silicon Photonics Microring Modulo-FSR & Thermal Lock (0-ULP)
  // ==========================================================================
  dut->cpo_valid_i = 1;
  dut->cpo_target_phase12_i = 1024U;
  dut->cpo_uw_per_lsb_i = 4U;
  dut->cpo_lock_tol12_i = 64U;
  uint16_t cpo_prev_h12 = 0U;
  uint32_t cpo_hops = 0U;
  for (uint32_t step = 0U; step < 128U; ++step) {
    uint16_t th12 = (uint16_t)((step * 115U) & 0x0FFFU); // >3.5 FSR sweep
    int16_t ff_delta = (step == 64U) ? 320 : -24;        // XPU burst + cool
    uint16_t actual_wave12 =
        (uint16_t)((th12 + ((uint16_t)ff_delta & 0x0FFFU) + 12U) & 0x0FFFU);
    dut->cpo_thermal_phase12_i = th12;
    dut->cpo_ff_delta_phase_i = ((uint16_t)ff_delta) & 0x0FFFU;
    dut->cpo_actual_wave_phase12_i = actual_wave12;
    tick_clk(dut);

    bool c_hop = false;
    uint16_t c_h12 = ele_cpo_eval_heater_phase12(1024U, th12, ff_delta,
                                                 cpo_prev_h12, &c_hop);
    uint8_t exp_hop = (step > 0U && c_hop) ? 1U : 0U;
    if (exp_hop != 0U) {
      cpo_hops += 1U;
    }
    cpo_prev_h12 = c_h12;
    ELE_RTL_ASSERT(dut->cpo_heater_phase12_o == c_h12);
    ELE_RTL_ASSERT(dut->cpo_heater_phase12_o < 4096U);
    ELE_RTL_ASSERT(dut->cpo_heater_power_uw_o == (uint32_t)c_h12 * 4U);
    ELE_RTL_ASSERT(dut->cpo_hop_event_o == exp_hop);
    ELE_RTL_ASSERT(dut->cpo_phase_err12_o == 12U);
    ELE_RTL_ASSERT(dut->cpo_locked_o == 1U);
  }
  ELE_RTL_ASSERT(cpo_hops >= 3U);
  dut->cpo_valid_i = 0;

  // ==========================================================================
  // 9. Constant-Space 4-Slot Graphene Spillover Rowhammer Guard (0-ULP)
  // ==========================================================================
  EleHbmStack64 c_hbm;
  ele_hbm_stack64_init(&c_hbm, 60000, 1150U, 350U, 16U, 0x000FU);
  static uint16_t true_row_acts[65536];
  for (uint32_t r = 0U; r < 65536U; ++r) {
    true_row_acts[r] = 0U;
  }

  dut->hbm_act_valid_i = 1;
  dut->hbm_win_rst_i = 0;
  dut->hbm_drfm_thresh_i = 16U;

  // Many-sided Rowhammer pattern: 6 hot aggressors + rotating decoy stream
  for (uint32_t a = 0U; a < 512U; ++a) {
    uint16_t row = 0U;
    if ((a & 3U) != 3U) {
      row = (uint16_t)(0x1200U + (a % 6U));
    } else {
      row = (uint16_t)(0x4000U + a);
    }
    dut->hbm_row_id_i = row;
    tick_clk(dut);

    bool c_prev_ref = false;
    bool c_fired = ele_hbm_step_row_activate(&c_hbm, row, &c_prev_ref);
    uint8_t exp_drfm = (c_fired && !c_prev_ref) ? 1U : 0U;
    uint8_t exp_pref = c_prev_ref ? 1U : 0U;
    ELE_RTL_ASSERT(dut->hbm_drfm_fire_o == exp_drfm);
    ELE_RTL_ASSERT(dut->hbm_preventive_ref_o == exp_pref);
    ELE_RTL_ASSERT(dut->hbm_spill_cnt_o == c_hbm.rh_spill_cnt);
    ELE_RTL_ASSERT(
        dut->hbm_mitigation_cnt_o ==
        (uint16_t)(c_hbm.drfm_mitigations + c_hbm.preventive_refreshes));
    if (c_fired) {
      ELE_RTL_ASSERT(dut->hbm_drfm_row_o == row);
    }

    // Ground-truth verification: NO row ever reaches 16 unmitigated hammers!
    true_row_acts[row] += 1U;
    if (exp_drfm != 0U) {
      true_row_acts[row] = 0U;
    } else if (exp_pref != 0U) {
      for (uint32_t r = 0U; r < 65536U; ++r) {
        true_row_acts[r] = 0U;
      }
    }
    ELE_RTL_ASSERT(true_row_acts[row] < 16U);
  }
  ELE_RTL_ASSERT(dut->hbm_mitigation_cnt_o > 10U);
  dut->hbm_win_rst_i = 1;
  tick_clk(dut);
  ele_hbm_rowhammer_window_reset(&c_hbm);
  dut->hbm_win_rst_i = 0;
  dut->hbm_act_valid_i = 0;
  ELE_RTL_ASSERT(dut->hbm_spill_cnt_o == 0U);

  // ==========================================================================
  // 10. LC Resonant Adiabatic Clock Sustainer & Crest Tank-Parker (0-ULP)
  // ==========================================================================
  EleResonantClk64 c_rclk;
  ele_resonant_clk64_init(&c_rclk, 500000U, 750U, 12000U, 2000U, 120U, 25U);
  dut->rclk_valid_i = 1;
  dut->rclk_v_dd_mv_i = 750U;
  dut->rclk_target_pulse_ps_i = 120U;
  dut->rclk_zvs_tol_ps_i = 25U;

  int16_t rclk_err = 93;
  for (uint32_t cyc = 0U; cyc < 64U; ++cyc) {
    bool clk_en = !(cyc >= 32U && cyc < 44U);
    dut->rclk_en_i = clk_en ? 1U : 0U;
    dut->rclk_zvs_err_ps_i = (uint16_t)rclk_err;
    tick_clk(dut);

    uint16_t c_pulse = ele_resonant_clk_step(&c_rclk, clk_en, rclk_err);
    ELE_RTL_ASSERT(dut->rclk_sustain_pulse_ps_o == c_pulse);
    ELE_RTL_ASSERT(dut->rclk_tank_env_mv_o == c_rclk.tank_env_mv);
    ELE_RTL_ASSERT(dut->rclk_park_cap_mv_o == c_rclk.park_cap_mv);
    ELE_RTL_ASSERT(dut->rclk_state_o == c_rclk.state);
    ELE_RTL_ASSERT(dut->rclk_zvs_locked_o == c_rclk.zvs_locked);
    ELE_RTL_ASSERT(dut->rclk_park_ready_o == c_rclk.park_ready);
    ELE_RTL_ASSERT(dut->rclk_restart_cnt_o == c_rclk.restart_events);

    if (clk_en) {
      rclk_err = (int16_t)((int32_t)dut->rclk_sustain_pulse_ps_o - 95);
    }
  }
  ELE_RTL_ASSERT(dut->rclk_restart_cnt_o == 1U);
  ELE_RTL_ASSERT(dut->rclk_zvs_locked_o == 1U);
  dut->rclk_valid_i = 0;

  // ==========================================================================
  // 11. Asynchronous Level-Crossing Event ADC & 63-Tap Vernier TDC (0-ULP)
  // ==========================================================================
  EleEventAdc64 c_eadc;
  ele_event_adc64_init(&c_eadc, 5000U, 2000U, 1000U, 45U, 40U, 800U, 250U);
  dut->eadc_valid_i = 1;
  dut->eadc_lsb_step_uv_i = 5000U;
  dut->eadc_hyst_uv_i = 2000U;
  dut->eadc_t_slow_ps_i = 45U;
  dut->eadc_t_fast_ps_i = 40U;

  for (uint32_t n = 0U; n < 128U; ++n) {
    int32_t v_in = 100000;
    if (n >= 1U && n < 80U) {
      // Sub-threshold noise +/- 6,200 uV (< 5,000 + 2,000) -> 0 chatter!
      v_in = 100000 + ((n & 1U) ? 6200 : -6200);
    } else if (n >= 80U && n < 104U) {
      v_in = 100000 + (int32_t)(n - 79U) * 5500;
    } else if (n >= 104U) {
      v_in = 232000 - (int32_t)(n - 103U) * 6000;
    }
    uint16_t lead_ps = (uint16_t)((n * 19U) % 400U); // Exercises 0..63 + ovf
    dut->eadc_v_in_uv_i = (uint32_t)v_in;
    dut->eadc_lead_ps_i = lead_ps;
    tick_clk(dut);

    int8_t c_dir = ele_event_adc_step(&c_eadc, v_in, lead_ps);
    ELE_RTL_ASSERT((int32_t)dut->eadc_dac_ref_uv_o == c_eadc.dac_ref_uv);
    ELE_RTL_ASSERT(dut->eadc_event_fired_o == c_eadc.event_fired);
    ELE_RTL_ASSERT(dut->eadc_dir_up_o == ((c_dir > 0) ? 1U : 0U));
    ELE_RTL_ASSERT(dut->eadc_quiet_cnt_o == c_eadc.quiet_ticks);
    if (c_dir != 0) {
      ELE_RTL_ASSERT(dut->eadc_vernier_steps_o == c_eadc.last_vernier_steps);
      ELE_RTL_ASSERT(dut->eadc_vernier_fine_ps_o ==
                     c_eadc.last_vernier_steps * 5U);
      ELE_RTL_ASSERT(dut->eadc_tdc_overflow_o == c_eadc.tdc_overflow);
    }
    ELE_RTL_ASSERT(dut->eadc_event_cnt_o == (uint16_t)c_eadc.event_samples);
  }
  ELE_RTL_ASSERT(c_eadc.chatter_suppressed >= 70U);

  // Adversarial eff_lsb >= 2^31 & full-scale rail swing co-sim check
  dut->rst_n = 0;
  tick_clk(dut);
  dut->rst_n = 1;
  EleEventAdc64 c_eadc_sat;
  ele_event_adc64_init(&c_eadc_sat, 2500000000U, 1000U, 1000U, 45U, 40U, 800U,
                       250U);
  dut->eadc_lsb_step_uv_i = 2500000000U;
  dut->eadc_hyst_uv_i = 1000U;
  const int32_t sat_vec[3] = {500000000, -2100000000, 2100000000};
  for (uint32_t k = 0U; k < 3U; ++k) {
    dut->eadc_v_in_uv_i = (uint32_t)sat_vec[k];
    dut->eadc_lead_ps_i = 25U;
    tick_clk(dut);
    int8_t s_dir = ele_event_adc_step(&c_eadc_sat, sat_vec[k], 25U);
    ELE_RTL_ASSERT((int32_t)dut->eadc_dac_ref_uv_o == c_eadc_sat.dac_ref_uv);
    ELE_RTL_ASSERT(dut->eadc_event_fired_o == c_eadc_sat.event_fired);
    ELE_RTL_ASSERT(dut->eadc_dir_up_o == ((s_dir > 0) ? 1U : 0U));
    ELE_RTL_ASSERT(dut->eadc_quiet_cnt_o == c_eadc_sat.quiet_ticks);
  }
  ELE_RTL_ASSERT(c_eadc_sat.dac_ref_uv == 500000000);
  dut->eadc_valid_i = 0;

  // ==========================================================================
  // 12. BSPDN Active Resonance Damper & Symplectic Spread-Spectrum (0-ULP)
  // ==========================================================================
  ElePdnEmi64 c_emi;
  ele_pdn_emi64_init(&c_emi, 500U, 32U, 64U, 0xACE1U);
  dut->emi_valid_i = 1;
  dut->emi_seed_i = 0xACE1U;
  dut->emi_nom_period_ps_i = 500U;
  dut->emi_max_dither_ps_i = 32U;
  dut->emi_damping_gain_q8_i = 64U;

  const int16_t res_wave[6] = {400, 400, 0, -400, -400, 0};
  for (uint32_t cyc = 0U; cyc < 256U; ++cyc) {
    int16_t exc_ma = res_wave[cyc % 6U];
    dut->emi_di_excite_ma_i = (uint16_t)exc_ma;
    tick_clk(dut);

    uint16_t c_mod_p = ele_pdn_emi_step_cycle(&c_emi, exc_ma);
    ELE_RTL_ASSERT(dut->emi_mod_period_ps_o == c_mod_p);
    ELE_RTL_ASSERT(dut->emi_pair_phase_o == c_emi.pair_phase);
    ELE_RTL_ASSERT(dut->emi_spec_bin_o < 8U);
    ELE_RTL_ASSERT((int32_t)dut->emi_v_undamped_uv_o ==
                   c_emi.v_osc_undamped_uv);
    ELE_RTL_ASSERT((int32_t)dut->emi_v_damped_uv_o == c_emi.v_osc_damped_uv);
    if ((cyc & 1U) == 1U) {
      // Exact 0-ULP 2-cycle period conservation!
      ELE_RTL_ASSERT(c_emi.sum_period_ss_ps == c_emi.sum_period_base_ps);
    }
  }
  ELE_RTL_ASSERT(c_emi.peak_v_damped_uv * 4U < c_emi.peak_v_undamped_uv);

  // Near-rail (nom_period_ps = 25 ps, max_dither_ps = 32 ps) 0-ULP conservation
  dut->rst_n = 0;
  tick_clk(dut);
  dut->rst_n = 1;
  ElePdnEmi64 c_emi_rail;
  ele_pdn_emi64_init(&c_emi_rail, 25U, 32U, 64U, 0xBEEFU);
  dut->emi_seed_i = 0xBEEFU;
  dut->emi_nom_period_ps_i = 25U;
  dut->emi_max_dither_ps_i = 32U;
  for (uint32_t cyc = 0U; cyc < 32U; ++cyc) {
    dut->emi_di_excite_ma_i = 0U;
    tick_clk(dut);
    uint16_t c_rail_p = ele_pdn_emi_step_cycle(&c_emi_rail, 0);
    ELE_RTL_ASSERT(dut->emi_mod_period_ps_o == c_rail_p);
    ELE_RTL_ASSERT(c_rail_p >= 20U && c_rail_p <= 30U);
    if ((cyc & 1U) == 1U) {
      ELE_RTL_ASSERT(c_emi_rail.sum_period_ss_ps ==
                     c_emi_rail.sum_period_base_ps);
    }
  }
  ELE_RTL_ASSERT(skip_beats > 100U);
  dut->emi_valid_i = 0;

  uint64_t bus_save_pct =
      ((total_raw_flips - total_phy_flips) * 100ULL) / total_raw_flips;
  uint64_t ff_save_pct =
      ((twos_ff_flips - dual_ff_flips) * 100ULL) / twos_ff_flips;
  uint32_t slm_quiet_pct = (c_box.suppressed_cycles * 100U) / 256U;
  uint32_t bspdn_damp_pct =
      ((c_emi.peak_v_undamped_uv - c_emi.peak_v_damped_uv) * 100U) /
      c_emi.peak_v_undamped_uv;

  std::printf(
      "  [RTL Co-Sim] Verilator 5,800+ Cyc (%u assertions, 0-ULP == C17) | "
      "AXI65 MaxSSO=%u/65 (-%" PRIu64 "%% flips) | DualRail -%" PRIu64
      "%% | VRM16 %u ps | GFM60 +%d kW | FLL/SSCB/SLM -%u%% | CPO/HBM/RCLK/"
      "EADC/EMI -%u%%\n",
      rtl_assertions, (unsigned)max_single_beat_flips, bus_save_pct,
      ff_save_pct, (unsigned)dut->vrm_t_dead_ps_o, (int)p_arrest_rtl,
      (unsigned)slm_quiet_pct, (unsigned)bspdn_damp_pct);

  delete dut;
  return 0;
}
