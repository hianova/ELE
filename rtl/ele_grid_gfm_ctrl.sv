// SPDX-License-Identifier: MIT
// ELE Hardware IP: 60Hz/50Hz Grid-Forming (GFM) Virtual Synchronous Generator
//                  & 1-Cycle Zero-Crossing Compute Droop / Solar Sponge
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorems 22-23):
//   - Multiplier-free shift-add Virtual Synchronous Generator (VSG) executing on
//     every AC zero-crossing strobe (`zc_valid_i`, 16.67 ms @ 60 Hz / 20 ms @ 50 Hz):
//       P_inj = clamp(P_set - 16 * eff_df_mhz - 128 * d_freq_step_mhz, -P_max, +P_max)
//   - Glitch-free `freq_primed_q` cold-start supports both 50 Hz and 60 Hz grids
//     on Cycle 1 with zero false RoCoF spikes.
//   - Signed 18-bit `p_inject_kw_o` (`[-131,072 .. +131,071]` kW) and 22-bit
//     saturation-safe P-cap arithmetic eliminate wrap-around across the entire
//     `uint16_t` `max_kw_i` (`0..65,535` kW) and `freq_mhz_i` (`0..65,535` mHz) domain.

`default_nettype none

module ele_grid_gfm_ctrl (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               zc_valid_i,
    input  logic        [15:0] freq_mhz_i,
    input  logic        [15:0] nom_freq_mhz_i,
    input  logic        [15:0] deadband_mhz_i,
    input  logic signed [17:0] p_set_kw_i,
    input  logic        [15:0] max_kw_i,
    output logic signed [17:0] p_inject_kw_o,
    output logic        [10:0] compute_pcap_permille_o,
    output logic signed [15:0] rocof_step_mhz_o,
    output logic               ufls_arrest_o,
    output logic               solar_sponge_o
);
    logic               freq_primed_q;
    logic        [15:0] prev_freq_q;
    logic        [15:0] prev_eff_w;
    logic signed [17:0] df_w;
    logic signed [17:0] db_w;
    logic signed [17:0] eff_df_w;
    logic signed [17:0] d_step_w;
    logic signed [15:0] rocof_sat_w;
    logic signed [25:0] p_set_ext_w;
    logic signed [25:0] p_droop_w;
    logic signed [25:0] p_inert_w;
    logic signed [25:0] p_raw_w;
    logic signed [25:0] p_lim_w;
    logic signed [17:0] p_clamped_w;
    logic               arrest_w;
    logic               sponge_w;
    logic signed [17:0] neg_df_excess_w;
    logic signed [17:0] neg_step_excess_w;
    logic        [17:0] neg_df_u_w;
    logic        [17:0] neg_step_u_w;
    logic        [21:0] f_pen_w;
    logic        [21:0] r_pen_w;
    logic        [21:0] total_cut_w;
    logic        [10:0] pcap_next_w;

    assign prev_eff_w = freq_primed_q ? prev_freq_q : nom_freq_mhz_i;
    assign df_w       = $signed({2'b00, freq_mhz_i}) - $signed({2'b00, nom_freq_mhz_i});
    assign db_w       = $signed({2'b00, deadband_mhz_i});
    assign d_step_w   = $signed({2'b00, freq_mhz_i}) - $signed({2'b00, prev_eff_w});

    always_comb begin : rocof_saturator
        if (d_step_w > 18'sd32767) begin
            rocof_sat_w = 16'sd32767;
        end else if (d_step_w < -18'sd32768) begin
            rocof_sat_w = -16'sd32768;
        end else begin
            rocof_sat_w = d_step_w[15:0];
        end
    end

    always_comb begin : deadband_filter
        if (df_w > db_w) begin
            eff_df_w = df_w - db_w;
        end else if (df_w < -db_w) begin
            eff_df_w = df_w + db_w;
        end else begin
            eff_df_w = 18'sd0;
        end
    end

    // Multiplier-free VSG droop (-16 kW/mHz) + synthetic inertia (-128 kW/(mHz/cyc))
    assign p_set_ext_w = {{8{p_set_kw_i[17]}}, p_set_kw_i};
    assign p_droop_w   = -({{8{eff_df_w[17]}}, eff_df_w} <<< 4);
    assign p_inert_w   = -({{8{d_step_w[17]}}, d_step_w} <<< 7);
    assign p_raw_w     = p_set_ext_w + p_droop_w + p_inert_w;
    assign p_lim_w     = $signed({10'd0, max_kw_i});

    always_comb begin : power_saturator
        if (p_raw_w > p_lim_w) begin
            p_clamped_w = $signed({2'b00, max_kw_i});
        end else if (p_raw_w < -p_lim_w) begin
            p_clamped_w = -$signed({2'b00, max_kw_i});
        end else begin
            p_clamped_w = p_raw_w[17:0];
        end
    end

    assign arrest_w = (df_w < -18'sd150) || ((df_w < -db_w) && (d_step_w <= -18'sd7));
    assign sponge_w = (!arrest_w) && (df_w > 18'sd100);

    assign neg_df_excess_w   = -df_w - 18'sd150;
    assign neg_step_excess_w = -d_step_w - 18'sd7;
    assign neg_df_u_w        = neg_df_excess_w;
    assign neg_step_u_w      = neg_step_excess_w;
    assign f_pen_w           = (df_w < -18'sd150) ? {4'd0, neg_df_u_w} : 22'd0;
    assign r_pen_w           = (d_step_w <= -18'sd7) ? ({4'd0, neg_step_u_w} << 4) : 22'd0;
    assign total_cut_w       = 22'd150 + f_pen_w + r_pen_w;

    always_comb begin : pcap_resolver
        if (arrest_w) begin
            pcap_next_w = (total_cut_w >= 22'd450) ? 11'd550 : (11'd1000 - total_cut_w[10:0]);
        end else if (sponge_w) begin
            pcap_next_w = 11'd1150;
        end else begin
            pcap_next_w = 11'd1000;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_primed_q           <= 1'b0;
            prev_freq_q             <= 16'd60000;
            p_inject_kw_o           <= 18'sd0;
            compute_pcap_permille_o <= 11'd1000;
            rocof_step_mhz_o        <= 16'sd0;
            ufls_arrest_o           <= 1'b0;
            solar_sponge_o          <= 1'b0;
        end else if (zc_valid_i) begin
            freq_primed_q           <= 1'b1;
            prev_freq_q             <= freq_mhz_i;
            p_inject_kw_o           <= p_clamped_w;
            compute_pcap_permille_o <= pcap_next_w;
            rocof_step_mhz_o        <= rocof_sat_w;
            ufls_arrest_o           <= arrest_w;
            solar_sponge_o          <= sponge_w;
        end
    end
endmodule

`default_nettype wire
