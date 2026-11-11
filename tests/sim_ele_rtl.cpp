// SPDX-License-Identifier: MIT
// Cycle-Accurate Verilator RTL Co-Simulation & Register-Toggle Calorimeter
// Verifies synthesized SystemVerilog hardware IP bit-for-bit (0-ULP) against
// ELE's C17 golden physics kernel (`libele.a`) across 4,096+ clock cycles.

#include "Vele_silicon_top.h"
#include "verilated.h"

#include "../include/ele_grid.h"
#include "../include/ele_rack_vrm.h"
#include "../include/ele_silicon.h"

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

  uint64_t bus_save_pct =
      ((total_raw_flips - total_phy_flips) * 100ULL) / total_raw_flips;
  uint64_t ff_save_pct =
      ((twos_ff_flips - dual_ff_flips) * 100ULL) / twos_ff_flips;

  std::printf(
      "  [RTL Co-Sim] Verilator 4,096+ Cyc (%u assertions, 0-ULP == C17) | "
      "AXI65 MaxSSO=%u/65 (-%" PRIu64 "%% flips, %u skips) | DualRail FF "
      "Toggles: %" PRIu64 " vs 2sComp %" PRIu64 " (-%" PRIu64
      "%%) | VRM16 ZVS (%u ps) | GFM60 +%d kW (Pcap %u‰)\n",
      rtl_assertions, (unsigned)max_single_beat_flips, bus_save_pct, skip_beats,
      dual_ff_flips, twos_ff_flips, ff_save_pct, (unsigned)dut->vrm_t_dead_ps_o,
      (int)p_arrest_rtl, (unsigned)pcap_arrest_rtl);

  delete dut;
  return 0;
}
