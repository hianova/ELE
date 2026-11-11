// SPDX-License-Identifier: MIT
// ELE Unified Silicon Benchmark Top (Cycle-Accurate Verilator Co-Sim Wrapper)
//
// Instantiates all 12 ELE synthesizable hardware IPs alongside conventional
// silicon baselines on the exact same clock & reset domain so Verilator and
// Yosys can directly measure:
//   1. `ele_cache_bus65_axi_bridge` (65-Wire AXI4-Stream Charge-Minimal PHY)
//   2. `ele_ternary_mac_dualrail` vs `ele_ternary_mac_twos_comp_ref`
//      vs `ele_int32_mul_mac_ref`
//   3. `ele_vrm_zvs_ctrl` (16-Phase 48V Digital GaN/SiC VRM ZVS + Pre-Charge)
//   4. `ele_grid_gfm_ctrl` (60Hz/50Hz Grid-Forming Synthetic-Inertia + Sponge)
//   5. `ele_fll_droop` (<1ns 1-Cycle Droop-Adaptive AD-FLL Clock-Stretcher)
//   6. `ele_sscb_trip` (800V/48V Microsecond SiC Solid-State Circuit Breaker)
//   7. `ele_slm_canary_trace` (Zero-Stall 2-Jet Flight Recorder & Aging Canary)
//   8. `ele_cpo_mrr_lock` (CPO Silicon Photonics Modulo-FSR & Thermal Lock)
//   9. `ele_hbm_rowhammer_guard` (4-Slot Graphene Spillover Rowhammer DRFM)
//  10. `ele_resonant_clk_zvs` (LC Adiabatic Clock Sustainer & Crest Parker)
//  11. `ele_level_crossing_adc` (Asynchronous Level-Crossing ADC + Vernier TDC)
//  12. `ele_pdn_emi_damping` (BSPDN Active Resonance Damper & Spread-Spectrum)

`default_nettype none

module ele_silicon_top (
    input  logic               clk,
    input  logic               rst_n,

    // 1. AXI4-Stream 65-Wire Charge-Minimal Interconnect Bridge
    input  logic               axi_s_valid_i,
    output logic               axi_s_ready_o,
    input  logic        [63:0] axi_s_data_i,
    output logic               axi_m_valid_o,
    input  logic               axi_m_ready_i,
    output logic        [63:0] axi_m_data_o,
    output logic        [63:0] phy_wire_o,
    output logic               phy_inv_o,
    output logic               phy_zero_skip_o,

    // 2. Dual-Rail 1.58-Bit Ternary MAC vs Two's-Complement & Int32*Int16 Baselines
    input  logic               mac_clr_i,
    input  logic               mac_valid_i,
    input  logic               mac_readout_en_i,
    input  logic signed [31:0] mac_act_q16_i,
    input  logic        [1:0]  mac_trit_i,
    input  logic signed [15:0] mac_weight_i16_i,
    output logic signed [47:0] mac_dual_acc_o,
    output logic        [47:0] mac_pos_rail_o,
    output logic        [47:0] mac_neg_rail_o,
    output logic               mac_zero_skip_o,
    output logic signed [47:0] mac_twos_ref_acc_o,
    output logic signed [47:0] mac_mul_ref_acc_o,

    // 3. 16-Phase 48V GaN/SiC Digital VRM ZVS & Active Pre-Charge Controller
    input  logic               vrm_valid_i,
    input  logic        [15:0] vrm_load_amp_i,
    input  logic        [15:0] vrm_upcoming_amp_i,
    input  logic        [15:0] vrm_amp_per_phase_i,
    input  logic signed [15:0] vrm_zvs_err_ps_i,
    output logic        [15:0] vrm_phase_mask_o,
    output logic        [4:0]  vrm_active_phases_o,
    output logic        [15:0] vrm_t_dead_ps_o,
    output logic               vrm_precharge_active_o,
    output logic               vrm_zvs_locked_o,

    // 4. 60Hz/50Hz Grid-Forming (GFM) Virtual Synchronous Generator & Sponge
    input  logic               grid_zc_valid_i,
    input  logic        [15:0] grid_freq_mhz_i,
    input  logic        [15:0] grid_nom_freq_mhz_i,
    input  logic        [15:0] grid_deadband_mhz_i,
    input  logic signed [17:0] grid_p_set_kw_i,
    input  logic        [15:0] grid_max_kw_i,
    output logic signed [17:0] grid_p_inject_kw_o,
    output logic        [10:0] grid_compute_pcap_o,
    output logic signed [15:0] grid_rocof_step_mhz_o,
    output logic               grid_ufls_arrest_o,
    output logic               grid_solar_sponge_o,

    // 5. <1ns 1-Cycle Droop-Adaptive All-Digital FLL Clock-Stretcher
    input  logic               fll_valid_i,
    input  logic        [15:0] fll_t_nom_ps_i,
    input  logic        [15:0] fll_v_nom_mv_i,
    input  logic        [15:0] fll_v_inst_mv_i,
    output logic        [17:0] fll_t_clk_ps_o,
    output logic               fll_stretched_o,
    output logic               fll_emergency_gate_o,

    // 6. 800V/48V Microsecond SiC Solid-State Circuit Breaker (SSCB)
    input  logic               sscb_valid_i,
    input  logic               sscb_rearm_i,
    input  logic        [15:0] sscb_i_meas_amp_i,
    input  logic        [15:0] sscb_i_jet_arm_a_i,
    input  logic        [15:0] sscb_i_hard_trip_a_i,
    input  logic signed [15:0] sscb_di_crit_a_i,
    input  logic        [15:0] sscb_ddi_margin_a_i,
    input  logic        [15:0] sscb_soft_off_a_i,
    output logic        [15:0] sscb_cur_i_amp_o,
    output logic        [9:0]  sscb_gate_permille_o,
    output logic        [7:0]  sscb_clear_ticks_o,
    output logic        [1:0]  sscb_state_o,
    output logic               sscb_slot_healthy_o,
    output logic        [7:0]  sscb_inrush_cnt_o,

    // 7. Zero-Stall 2-Jet Silicon Trace Flight Recorder & Aging Canary
    input  logic               slm_valid_i,
    input  logic               slm_rearm_trace_i,
    input  logic               slm_fault_trig_i,
    input  logic        [15:0] slm_pc16_i,
    input  logic        [11:0] slm_v_dd_mv_i,
    input  logic        [3:0]  slm_hw_flags4_i,
    input  logic        [11:0] slm_v_eps_mv_i,
    input  logic        [2:0]  slm_snap_rd_idx_i,
    input  logic        [17:0] slm_t_clk_ps_i,
    input  logic        [17:0] slm_t_main_aged_ps_i,
    input  logic        [15:0] slm_canary_guard_ps_i,
    output logic               slm_trace_emit_o,
    output logic               slm_frozen_o,
    output logic        [2:0]  slm_wr_idx_o,
    output logic        [2:0]  slm_freeze_idx_o,
    output logic        [15:0] slm_suppressed_cnt_o,
    output logic        [15:0] slm_emitted_cnt_o,
    output logic        [31:0] slm_snap_rd_data_o,
    output logic               slm_canary_warn_o,
    output logic               slm_main_sdc_err_o,

    // 8. CPO Silicon Photonics Microring Modulo-FSR & Thermal Lock
    input  logic               cpo_valid_i,
    input  logic        [11:0] cpo_target_phase12_i,
    input  logic        [11:0] cpo_thermal_phase12_i,
    input  logic signed [11:0] cpo_ff_delta_phase_i,
    input  logic        [11:0] cpo_uw_per_lsb_i,
    input  logic        [11:0] cpo_lock_tol12_i,
    input  logic        [11:0] cpo_actual_wave_phase12_i,
    output logic        [11:0] cpo_heater_phase12_o,
    output logic        [23:0] cpo_heater_power_uw_o,
    output logic        [11:0] cpo_phase_err12_o,
    output logic               cpo_hop_event_o,
    output logic               cpo_locked_o,

    // 9. Constant-Space 4-Slot Graphene Spillover Rowhammer Guard (DRFM)
    input  logic               hbm_act_valid_i,
    input  logic               hbm_win_rst_i,
    input  logic        [15:0] hbm_row_id_i,
    input  logic        [15:0] hbm_drfm_thresh_i,
    output logic               hbm_drfm_fire_o,
    output logic               hbm_preventive_ref_o,
    output logic        [15:0] hbm_drfm_row_o,
    output logic        [15:0] hbm_spill_cnt_o,
    output logic        [15:0] hbm_mitigation_cnt_o,

    // 10. LC Resonant Adiabatic Clock Sustainer & Crest Tank-Parker
    input  logic               rclk_valid_i,
    input  logic               rclk_en_i,
    input  logic        [15:0] rclk_v_dd_mv_i,
    input  logic        [15:0] rclk_target_pulse_ps_i,
    input  logic        [15:0] rclk_zvs_tol_ps_i,
    input  logic signed [15:0] rclk_zvs_err_ps_i,
    output logic        [15:0] rclk_sustain_pulse_ps_o,
    output logic        [15:0] rclk_tank_env_mv_o,
    output logic        [15:0] rclk_park_cap_mv_o,
    output logic               rclk_state_o,
    output logic               rclk_zvs_locked_o,
    output logic               rclk_park_ready_o,
    output logic        [15:0] rclk_restart_cnt_o,

    // 11. Asynchronous Level-Crossing Event ADC & 63-Tap Vernier TDC
    input  logic               eadc_valid_i,
    input  logic signed [31:0] eadc_v_in_uv_i,
    input  logic        [31:0] eadc_lsb_step_uv_i,
    input  logic        [31:0] eadc_hyst_uv_i,
    input  logic        [15:0] eadc_lead_ps_i,
    input  logic        [15:0] eadc_t_slow_ps_i,
    input  logic        [15:0] eadc_t_fast_ps_i,
    output logic signed [31:0] eadc_dac_ref_uv_o,
    output logic               eadc_event_fired_o,
    output logic               eadc_dir_up_o,
    output logic        [7:0]  eadc_vernier_steps_o,
    output logic        [15:0] eadc_vernier_fine_ps_o,
    output logic               eadc_tdc_overflow_o,
    output logic        [15:0] eadc_event_cnt_o,
    output logic        [15:0] eadc_quiet_cnt_o,

    // 12. BSPDN Active Resonance Damper & Symplectic Spread-Spectrum
    input  logic               emi_valid_i,
    input  logic        [15:0] emi_seed_i,
    input  logic        [15:0] emi_nom_period_ps_i,
    input  logic        [7:0]  emi_max_dither_ps_i,
    input  logic        [7:0]  emi_damping_gain_q8_i,
    input  logic signed [15:0] emi_di_excite_ma_i,
    output logic        [15:0] emi_mod_period_ps_o,
    output logic               emi_pair_phase_o,
    output logic        [2:0]  emi_spec_bin_o,
    output logic signed [31:0] emi_v_undamped_uv_o,
    output logic signed [31:0] emi_v_damped_uv_o
);
    ele_cache_bus65_axi_bridge u_axi_bus65 (
        .clk              (clk),
        .rst_n            (rst_n),
        .s_valid_i        (axi_s_valid_i),
        .s_ready_o        (axi_s_ready_o),
        .s_data_i         (axi_s_data_i),
        .m_valid_o        (axi_m_valid_o),
        .m_ready_i        (axi_m_ready_i),
        .m_data_o         (axi_m_data_o),
        .phy_wire_o       (phy_wire_o),
        .phy_inv_o        (phy_inv_o),
        .zero_flip_skip_o (phy_zero_skip_o)
    );

    ele_ternary_mac_dualrail u_mac_dual (
        .clk          (clk),
        .rst_n        (rst_n),
        .clr_i        (mac_clr_i),
        .valid_i      (mac_valid_i),
        .readout_en_i (mac_readout_en_i),
        .act_q16_i    (mac_act_q16_i),
        .trit_i       (mac_trit_i),
        .acc_o        (mac_dual_acc_o),
        .pos_rail_o   (mac_pos_rail_o),
        .neg_rail_o   (mac_neg_rail_o),
        .zero_skip_o  (mac_zero_skip_o)
    );

    ele_ternary_mac_twos_comp_ref u_mac_twos_ref (
        .clk       (clk),
        .rst_n     (rst_n),
        .clr_i     (mac_clr_i),
        .valid_i   (mac_valid_i),
        .act_q16_i (mac_act_q16_i),
        .trit_i    (mac_trit_i),
        .acc_o     (mac_twos_ref_acc_o)
    );

    ele_int32_mul_mac_ref u_mac_mul_ref (
        .clk          (clk),
        .rst_n        (rst_n),
        .clr_i        (mac_clr_i),
        .valid_i      (mac_valid_i),
        .act_q16_i    (mac_act_q16_i),
        .weight_i16_i (mac_weight_i16_i),
        .acc_o        (mac_mul_ref_acc_o)
    );

    ele_vrm_zvs_ctrl u_vrm_ctrl (
        .clk                (clk),
        .rst_n              (rst_n),
        .valid_i            (vrm_valid_i),
        .load_amp_i         (vrm_load_amp_i),
        .upcoming_amp_i     (vrm_upcoming_amp_i),
        .amp_per_phase_i    (vrm_amp_per_phase_i),
        .zvs_err_ps_i       (vrm_zvs_err_ps_i),
        .phase_mask_o       (vrm_phase_mask_o),
        .active_phases_o    (vrm_active_phases_o),
        .t_dead_ps_o        (vrm_t_dead_ps_o),
        .precharge_active_o (vrm_precharge_active_o),
        .zvs_locked_o       (vrm_zvs_locked_o)
    );

    ele_grid_gfm_ctrl u_grid_gfm (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .zc_valid_i              (grid_zc_valid_i),
        .freq_mhz_i              (grid_freq_mhz_i),
        .nom_freq_mhz_i          (grid_nom_freq_mhz_i),
        .deadband_mhz_i          (grid_deadband_mhz_i),
        .p_set_kw_i              (grid_p_set_kw_i),
        .max_kw_i                (grid_max_kw_i),
        .p_inject_kw_o           (grid_p_inject_kw_o),
        .compute_pcap_permille_o (grid_compute_pcap_o),
        .rocof_step_mhz_o        (grid_rocof_step_mhz_o),
        .ufls_arrest_o           (grid_ufls_arrest_o),
        .solar_sponge_o          (grid_solar_sponge_o)
    );

    ele_fll_droop u_fll_droop (
        .clk              (clk),
        .rst_n            (rst_n),
        .valid_i          (fll_valid_i),
        .t_nom_ps_i       (fll_t_nom_ps_i),
        .v_nom_mv_i       (fll_v_nom_mv_i),
        .v_inst_mv_i      (fll_v_inst_mv_i),
        .t_clk_ps_o       (fll_t_clk_ps_o),
        .stretched_o      (fll_stretched_o),
        .emergency_gate_o (fll_emergency_gate_o)
    );

    ele_sscb_trip u_sscb_trip (
        .clk               (clk),
        .rst_n             (rst_n),
        .valid_i           (sscb_valid_i),
        .rearm_i           (sscb_rearm_i),
        .i_meas_amp_i      (sscb_i_meas_amp_i),
        .i_jet_arm_a_i     (sscb_i_jet_arm_a_i),
        .i_hard_trip_a_i   (sscb_i_hard_trip_a_i),
        .di_crit_a_i       (sscb_di_crit_a_i),
        .ddi_margin_a_i    (sscb_ddi_margin_a_i),
        .soft_off_a_i      (sscb_soft_off_a_i),
        .cur_i_amp_o       (sscb_cur_i_amp_o),
        .gate_permille_o   (sscb_gate_permille_o),
        .clear_ticks_o     (sscb_clear_ticks_o),
        .state_o           (sscb_state_o),
        .slot_healthy_o    (sscb_slot_healthy_o),
        .inrush_ride_cnt_o (sscb_inrush_cnt_o)
    );

    ele_slm_canary_trace u_slm_canary_trace (
        .clk               (clk),
        .rst_n             (rst_n),
        .valid_i           (slm_valid_i),
        .rearm_trace_i     (slm_rearm_trace_i),
        .fault_trig_i      (slm_fault_trig_i),
        .pc16_i            (slm_pc16_i),
        .v_dd_mv_i         (slm_v_dd_mv_i),
        .hw_flags4_i       (slm_hw_flags4_i),
        .v_eps_mv_i        (slm_v_eps_mv_i),
        .snap_rd_idx_i     (slm_snap_rd_idx_i),
        .t_clk_ps_i        (slm_t_clk_ps_i),
        .t_main_aged_ps_i  (slm_t_main_aged_ps_i),
        .canary_guard_ps_i (slm_canary_guard_ps_i),
        .trace_emit_o      (slm_trace_emit_o),
        .frozen_o          (slm_frozen_o),
        .wr_idx_o          (slm_wr_idx_o),
        .freeze_idx_o      (slm_freeze_idx_o),
        .suppressed_cnt_o  (slm_suppressed_cnt_o),
        .emitted_cnt_o     (slm_emitted_cnt_o),
        .snap_rd_data_o    (slm_snap_rd_data_o),
        .canary_warn_o     (slm_canary_warn_o),
        .main_sdc_err_o    (slm_main_sdc_err_o)
    );

    ele_cpo_mrr_lock u_cpo_mrr_lock (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .valid_i               (cpo_valid_i),
        .target_phase12_i      (cpo_target_phase12_i),
        .thermal_phase12_i     (cpo_thermal_phase12_i),
        .ff_delta_phase_i      (cpo_ff_delta_phase_i),
        .uw_per_lsb_i          (cpo_uw_per_lsb_i),
        .lock_tol12_i          (cpo_lock_tol12_i),
        .actual_wave_phase12_i (cpo_actual_wave_phase12_i),
        .heater_phase12_o      (cpo_heater_phase12_o),
        .heater_power_uw_o     (cpo_heater_power_uw_o),
        .phase_err12_o         (cpo_phase_err12_o),
        .hop_event_o           (cpo_hop_event_o),
        .locked_o              (cpo_locked_o)
    );

    ele_hbm_rowhammer_guard u_hbm_rowhammer_guard (
        .clk              (clk),
        .rst_n            (rst_n),
        .act_valid_i      (hbm_act_valid_i),
        .win_rst_i        (hbm_win_rst_i),
        .row_id_i         (hbm_row_id_i),
        .drfm_thresh_i    (hbm_drfm_thresh_i),
        .drfm_fire_o      (hbm_drfm_fire_o),
        .preventive_ref_o (hbm_preventive_ref_o),
        .drfm_row_o       (hbm_drfm_row_o),
        .spill_cnt_o      (hbm_spill_cnt_o),
        .mitigation_cnt_o (hbm_mitigation_cnt_o)
    );

    ele_resonant_clk_zvs u_resonant_clk_zvs (
        .clk                (clk),
        .rst_n              (rst_n),
        .valid_i            (rclk_valid_i),
        .clk_en_i           (rclk_en_i),
        .v_dd_mv_i          (rclk_v_dd_mv_i),
        .target_pulse_ps_i  (rclk_target_pulse_ps_i),
        .zvs_tol_ps_i       (rclk_zvs_tol_ps_i),
        .zvs_err_ps_i       (rclk_zvs_err_ps_i),
        .sustain_pulse_ps_o (rclk_sustain_pulse_ps_o),
        .tank_env_mv_o      (rclk_tank_env_mv_o),
        .park_cap_mv_o      (rclk_park_cap_mv_o),
        .state_o            (rclk_state_o),
        .zvs_locked_o       (rclk_zvs_locked_o),
        .park_ready_o       (rclk_park_ready_o),
        .restart_cnt_o      (rclk_restart_cnt_o)
    );

    ele_level_crossing_adc u_level_crossing_adc (
        .clk               (clk),
        .rst_n             (rst_n),
        .valid_i           (eadc_valid_i),
        .v_in_uv_i         (eadc_v_in_uv_i),
        .lsb_step_uv_i     (eadc_lsb_step_uv_i),
        .hyst_uv_i         (eadc_hyst_uv_i),
        .lead_ps_i         (eadc_lead_ps_i),
        .t_slow_ps_i       (eadc_t_slow_ps_i),
        .t_fast_ps_i       (eadc_t_fast_ps_i),
        .dac_ref_uv_o      (eadc_dac_ref_uv_o),
        .event_fired_o     (eadc_event_fired_o),
        .dir_up_o          (eadc_dir_up_o),
        .vernier_steps_o   (eadc_vernier_steps_o),
        .vernier_fine_ps_o (eadc_vernier_fine_ps_o),
        .tdc_overflow_o    (eadc_tdc_overflow_o),
        .event_cnt_o       (eadc_event_cnt_o),
        .quiet_cnt_o       (eadc_quiet_cnt_o)
    );

    ele_pdn_emi_damping u_pdn_emi_damping (
        .clk               (clk),
        .rst_n             (rst_n),
        .valid_i           (emi_valid_i),
        .seed_i            (emi_seed_i),
        .nom_period_ps_i   (emi_nom_period_ps_i),
        .max_dither_ps_i   (emi_max_dither_ps_i),
        .damping_gain_q8_i (emi_damping_gain_q8_i),
        .di_excite_ma_i    (emi_di_excite_ma_i),
        .mod_period_ps_o   (emi_mod_period_ps_o),
        .pair_phase_o      (emi_pair_phase_o),
        .spec_bin_o        (emi_spec_bin_o),
        .v_undamped_uv_o   (emi_v_undamped_uv_o),
        .v_damped_uv_o     (emi_v_damped_uv_o)
    );
endmodule

`default_nettype wire
