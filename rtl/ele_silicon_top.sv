// SPDX-License-Identifier: MIT
// ELE Unified Silicon Benchmark Top (Cycle-Accurate Verilator Co-Sim Wrapper)
//
// Instantiates all 4 ELE synthesizable hardware IPs alongside conventional
// silicon baselines on the exact same clock & reset domain so Verilator and
// Yosys can directly measure:
//   1. `ele_cache_bus65_axi_bridge` (65-Wire AXI4-Stream Charge-Minimal PHY)
//   2. `ele_ternary_mac_dualrail` vs `ele_ternary_mac_twos_comp_ref`
//      vs `ele_int32_mul_mac_ref`
//   3. `ele_vrm_zvs_ctrl` (16-Phase 48V Digital GaN/SiC VRM ZVS + Pre-Charge)
//   4. `ele_grid_gfm_ctrl` (60Hz/50Hz Grid-Forming Synthetic-Inertia + Sponge)

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
    output logic               grid_solar_sponge_o
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
endmodule

`default_nettype wire
