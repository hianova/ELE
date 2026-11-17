// SPDX-License-Identifier: MIT
// ele_power_phy.sv — Closed-Loop Switched-Power, 800V DC SiC SSCB & 44-Pin Power QFN-48 ASIC — ELE v7.1.0
//
// Pure IEEE 1800 SystemVerilog synthesizable hardware primitives & ASIC wrapper
// integrating `ele_flow_atlas_top` (`SEARU x FLOW v7.1.0`) with switched-power control:
//   1. `ele_dpwm_multiphase` : 4-phase 90-deg interleaved HR-DPWM with hardware
//                              phase shedding (4->2->1), clamped dead-time interlock
//                              (0 shoot-through guaranteed by k-induction), and
//                              0-cycle `fault_kill_i` gate shutoff.
//   2. `ele_sscb_trip_atom`  : 800V DC SiC Solid-State Circuit Breaker (<1 us
//                              2-Jet dI/dt + d^2I/dt^2 concavity fault discriminator
//                              that rides through RC capacitive hot-swap inrush,
//                              plus adiabatic I^2*t thermal fuse via `ele_mul33_atom`,
//                              sticky trip latch & zero-current re-arm interlock).
//   3. `ele_power_qfn_top`   : 44-pin QFN-48 Autonomous Digital Power & Protection
//                              ASIC coupling `ele_flow_atlas_top`, `ele_dpwm_multiphase`,
//                              `ele_sscb_trip_atom`, 5-wire Bus-Invert host link
//                              (<= 2/5 SSO cap), and dual-channel 12b ADC hardware
//                              closed-loop + SSCB->DPWM 0-cycle fault reflex.

`default_nettype none
/* verilator lint_off DECLFILENAME */

module ele_dpwm_multiphase #(
    parameter int N_PHASES = 4,
    parameter int CNT_W    = 8,
    parameter int DT_W     = 4
) (
    input  logic                clk,
    input  logic                rst_n,
    input  logic                en_i,
    input  logic                fault_kill_i,
    input  logic [1:0]          active_minus1_i,
    input  logic [CNT_W-1:0]    duty_i,
    input  logic [DT_W-1:0]     deadtime_i,
    output logic [CNT_W-1:0]    phase0_cnt_o,
    output logic [N_PHASES-1:0] pwm_hi_o,
    output logic [N_PHASES-1:0] pwm_lo_o
);
    localparam int QUAD_SHIFT = 1 << (CNT_W - 2);

    logic [CNT_W-1:0]              cnt_q;
    logic [N_PHASES-1:0]           hi_q;
    logic [N_PHASES-1:0]           lo_q;
    logic [(N_PHASES*DT_W)-1:0]    dt_cnt_q;

    logic [DT_W-1:0]               dt_eff_c;
    logic [N_PHASES-1:0]           phase_en_c;
    logic [(N_PHASES*CNT_W)-1:0]   ph_cnt_c;

    assign dt_eff_c = (deadtime_i == {DT_W{1'b0}}) ? DT_W'(1) : deadtime_i;

    always_comb begin
        for (int p = 0; p < N_PHASES; p++) begin
            if (p == 0) begin
                phase_en_c[p] = 1'b1;
            end else if (p == 1) begin
                phase_en_c[p] = (active_minus1_i >= 2'd2);
            end else if (p == 2) begin
                phase_en_c[p] = (active_minus1_i >= 2'd1);
            end else begin
                phase_en_c[p] = (active_minus1_i >= 2'd3);
            end
            ph_cnt_c[p*CNT_W +: CNT_W] = cnt_q + CNT_W'(p * QUAD_SHIFT);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_q <= {CNT_W{1'b0}};
            hi_q  <= {N_PHASES{1'b0}};
            lo_q  <= {N_PHASES{1'b0}};
            for (int p = 0; p < N_PHASES; p++) begin
                dt_cnt_q[p*DT_W +: DT_W] <= {{(DT_W-1){1'b0}}, 1'b1};
            end
        end else if (!en_i || fault_kill_i) begin
            cnt_q <= {CNT_W{1'b0}};
            hi_q  <= {N_PHASES{1'b0}};
            lo_q  <= {N_PHASES{1'b0}};
            for (int p = 0; p < N_PHASES; p++) begin
                dt_cnt_q[p*DT_W +: DT_W] <= dt_eff_c;
            end
        end else begin
            cnt_q <= cnt_q + CNT_W'(1);
            for (int p = 0; p < N_PHASES; p++) begin
                if (!phase_en_c[p]) begin
                    hi_q[p]                  <= 1'b0;
                    lo_q[p]                  <= 1'b0;
                    dt_cnt_q[p*DT_W +: DT_W] <= dt_eff_c;
                end else if (ph_cnt_c[p*CNT_W +: CNT_W] < duty_i) begin
                    if (lo_q[p]) begin
                        hi_q[p]                  <= 1'b0;
                        lo_q[p]                  <= 1'b0;
                        dt_cnt_q[p*DT_W +: DT_W] <= dt_eff_c - DT_W'(1);
                    end else if (dt_cnt_q[p*DT_W +: DT_W] != {DT_W{1'b0}}) begin
                        hi_q[p]                  <= 1'b0;
                        lo_q[p]                  <= 1'b0;
                        dt_cnt_q[p*DT_W +: DT_W] <= dt_cnt_q[p*DT_W +: DT_W] - DT_W'(1);
                    end else begin
                        hi_q[p] <= 1'b1;
                        lo_q[p] <= 1'b0;
                    end
                end else begin
                    if (hi_q[p]) begin
                        hi_q[p]                  <= 1'b0;
                        lo_q[p]                  <= 1'b0;
                        dt_cnt_q[p*DT_W +: DT_W] <= dt_eff_c - DT_W'(1);
                    end else if (dt_cnt_q[p*DT_W +: DT_W] != {DT_W{1'b0}}) begin
                        hi_q[p]                  <= 1'b0;
                        lo_q[p]                  <= 1'b0;
                        dt_cnt_q[p*DT_W +: DT_W] <= dt_cnt_q[p*DT_W +: DT_W] - DT_W'(1);
                    end else begin
                        hi_q[p] <= 1'b0;
                        lo_q[p] <= 1'b1;
                    end
                end
            end
        end
    end

    assign phase0_cnt_o = cnt_q;
    assign pwm_hi_o     = (rst_n & en_i & (~fault_kill_i)) ? hi_q : {N_PHASES{1'b0}};
    assign pwm_lo_o     = (rst_n & en_i & (~fault_kill_i)) ? lo_q : {N_PHASES{1'b0}};

`ifdef FORMAL
    logic                f_past_valid_q;
    logic                f_past_rst_n_q;
    logic [N_PHASES-1:0] f_past_hi_q;
    logic [N_PHASES-1:0] f_past_lo_q;
    logic [N_PHASES-1:0] f_past_shed_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q <= 1'b1;
        f_past_rst_n_q <= rst_n;
        f_past_hi_q    <= hi_q;
        f_past_lo_q    <= lo_q;
        f_past_shed_q  <= (~phase_en_c) | {N_PHASES{(~en_i) | fault_kill_i}};
    end

    always_comb begin
        // Inductive shoot-through impossibility (both internal & pin-level)
        assert ((hi_q & lo_q) == {N_PHASES{1'b0}});
        assert ((pwm_hi_o & pwm_lo_o) == {N_PHASES{1'b0}});

        if (!rst_n || !en_i || fault_kill_i) begin
            assert (pwm_hi_o == {N_PHASES{1'b0}});
            assert (pwm_lo_o == {N_PHASES{1'b0}});
        end
        if (!rst_n) begin
            assert (cnt_q == {CNT_W{1'b0}});
            assert (hi_q == {N_PHASES{1'b0}});
            assert (lo_q == {N_PHASES{1'b0}});
        end else if (f_past_valid_q && f_past_rst_n_q) begin
            for (int p = 0; p < N_PHASES; p++) begin
                // Mandatory >=1 cycle dead-time separation between high-side & low-side
                assert (!(f_past_hi_q[p] && lo_q[p]));
                assert (!(f_past_lo_q[p] && hi_q[p]));
                // Shed or disabled phases transition cleanly to High-Z (0,0)
                if (f_past_shed_q[p]) begin
                    assert (!hi_q[p] && !lo_q[p]);
                end
            end
        end
    end
`endif
endmodule

module ele_sscb_trip_atom (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        armed_i,
    input  logic        sample_valid_i,
    input  logic        clr_rearm_i,
    input  logic [15:0] i_meas_i,
    input  logic [15:0] i_nom_i,
    input  logic [15:0] i_hard_max_i,
    input  logic [15:0] didt_thresh_i,
    input  logic [15:0] d2_concave_tol_i,
    input  logic [2:0]  confirm_len_i,
    input  logic [31:0] i2t_limit_i,
    output logic        gate_en_o,
    output logic        trip_latched_o,
    output logic [2:0]  trip_cause_o,
    output logic [31:0] i2t_acc_o
);
    logic               trip_latched_q;
    logic [2:0]         trip_cause_q;
    logic [31:0]        i2t_acc_q;
    logic [15:0]        i_prev_q;
    logic signed [16:0] j1_prev_q;
    logic [1:0]         jet_warm_q;
    logic [2:0]         jet_persist_q;

    logic               sample_active_c;
    logic [15:0]        i_over_c;
    logic signed [65:0] i2_prod_w;
    logic [31:0]        i2_step_c;
    logic [32:0]        i2t_sum_c;
    logic [31:0]        i2t_cool_c;
    logic [31:0]        i2t_next_c;

    logic signed [16:0] j1_c;
    logic signed [17:0] j2_c;
    logic signed [17:0] j2_concave_sum_c;
    logic               jet2_hit_c;
    logic [2:0]         persist_next_c;
    logic [2:0]         conf_eff_c;

    logic               cause_hard_c;
    logic               cause_jet2_c;
    logic               cause_i2t_c;
    logic               trip_now_c;
    logic               rearm_ok_c;

    assign sample_active_c = armed_i & sample_valid_i & (~trip_latched_q);
    assign i_over_c        = (i_meas_i > i_nom_i) ? (i_meas_i - i_nom_i) : 16'd0;

    ele_mul33_atom u_i2_mul (
        .en_i   (sample_active_c & (i_meas_i > i_nom_i)),
        .a_i    ($signed({17'd0, i_over_c})),
        .b_i    ($signed({17'd0, i_over_c})),
        .prod_o (i2_prod_w)
    );
    wire unused_i2_hi_ok = &{1'b0, i2_prod_w[65:32], i2_prod_w[3:0]};

    always_comb begin
        i2_step_c  = {4'd0, i2_prod_w[31:4]};
        i2t_sum_c  = {1'b0, i2t_acc_q} + {1'b0, i2_step_c};
        i2t_cool_c = (i2t_acc_q > 32'd64) ? (i2t_acc_q - 32'd64) : 32'd0;
        if (!sample_active_c) begin
            i2t_next_c = i2t_acc_q;
        end else if (i_meas_i > i_nom_i) begin
            i2t_next_c = i2t_sum_c[32] ? 32'hFFFF_FFFF : i2t_sum_c[31:0];
        end else begin
            i2t_next_c = i2t_cool_c;
        end

        j1_c             = $signed({1'b0, i_meas_i}) - $signed({1'b0, i_prev_q});
        j2_c             = $signed({j1_c[16], j1_c}) - $signed({j1_prev_q[16], j1_prev_q});
        j2_concave_sum_c = j2_c + $signed({2'b00, d2_concave_tol_i});
        jet2_hit_c       = (jet_warm_q == 2'd2) &
                           (i_meas_i > i_nom_i) &
                           (j1_c >= $signed({1'b0, didt_thresh_i})) &
                           (j2_concave_sum_c >= 18'sd0);

        persist_next_c = jet2_hit_c
                       ? ((jet_persist_q == 3'd7) ? 3'd7 : (jet_persist_q + 3'd1))
                       : 3'd0;
        conf_eff_c     = (confirm_len_i == 3'd0) ? 3'd1 : confirm_len_i;

        cause_hard_c = sample_active_c & (i_meas_i >= i_hard_max_i);
        cause_jet2_c = sample_active_c & jet2_hit_c & (persist_next_c >= conf_eff_c);
        cause_i2t_c  = sample_active_c & (i2t_next_c >= i2t_limit_i);
        trip_now_c   = cause_hard_c | cause_jet2_c | cause_i2t_c;
        rearm_ok_c   = clr_rearm_i & (i_meas_i == 16'd0);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trip_latched_q <= 1'b0;
            trip_cause_q   <= 3'd0;
            i2t_acc_q      <= 32'd0;
            i_prev_q       <= 16'd0;
            j1_prev_q      <= 17'sd0;
            jet_warm_q     <= 2'd0;
            jet_persist_q  <= 3'd0;
        end else if (trip_latched_q) begin
            if (rearm_ok_c) begin
                trip_latched_q <= 1'b0;
                trip_cause_q   <= 3'd0;
                i2t_acc_q      <= 32'd0;
                i_prev_q       <= 16'd0;
                j1_prev_q      <= 17'sd0;
                jet_warm_q     <= 2'd0;
                jet_persist_q  <= 3'd0;
            end
        end else if (!armed_i) begin
            i_prev_q      <= 16'd0;
            j1_prev_q     <= 17'sd0;
            jet_warm_q    <= 2'd0;
            jet_persist_q <= 3'd0;
            i2t_acc_q     <= (i2t_acc_q > 32'd64) ? (i2t_acc_q - 32'd64) : 32'd0;
        end else if (sample_valid_i) begin
            i2t_acc_q     <= i2t_next_c;
            i_prev_q      <= i_meas_i;
            j1_prev_q     <= j1_c;
            jet_warm_q    <= (jet_warm_q == 2'd2) ? 2'd2 : (jet_warm_q + 2'd1);
            jet_persist_q <= persist_next_c;
            if (trip_now_c) begin
                trip_latched_q <= 1'b1;
                trip_cause_q   <= {cause_i2t_c, cause_jet2_c, cause_hard_c};
            end
        end
    end

    assign gate_en_o      = rst_n & armed_i & (~trip_latched_q) & (~trip_now_c);
    assign trip_latched_o = rst_n & trip_latched_q;
    assign trip_cause_o   = trip_cause_q;
    assign i2t_acc_o      = i2t_acc_q;

`ifdef FORMAL
    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_trip_q;
    logic        f_past_armed_q;
    logic        f_past_sval_q;
    logic        f_past_clr_q;
    logic [15:0] f_past_i_q;
    logic [15:0] f_past_inom_q;
    logic [31:0] f_past_i2t_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q <= 1'b1;
        f_past_rst_n_q <= rst_n;
        f_past_trip_q  <= trip_latched_q;
        f_past_armed_q <= armed_i;
        f_past_sval_q  <= sample_valid_i;
        f_past_clr_q   <= clr_rearm_i;
        f_past_i_q     <= i_meas_i;
        f_past_inom_q  <= i_nom_i;
        f_past_i2t_q   <= i2t_acc_q;
    end

    always_comb begin
        // Fail-safe reset & fault de-energization invariants
        if (!rst_n || !armed_i || trip_latched_o) begin
            assert (!gate_en_o);
        end
        if (!rst_n) begin
            assert (!trip_latched_o && !trip_latched_q);
            assert (trip_cause_o == 3'd0);
            assert (i2t_acc_o == 32'd0);
        end
        // 0-cycle hard overcurrent shutoff
        if (rst_n && armed_i && sample_valid_i && !trip_latched_q && (i_meas_i >= i_hard_max_i)) begin
            assert (!gate_en_o);
        end
        // Concave d^2I/dt^2 < -tol capacitive hot-swap inrush NEVER asserts predictive short trip
        if (!jet2_hit_c) begin
            assert (!cause_jet2_c);
        end
        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin
            // Sticky trip latch cannot clear while live fault current > 0
            if (f_past_trip_q && !(f_past_clr_q && (f_past_i_q == 16'd0))) begin
                assert (trip_latched_o && !gate_en_o);
            end
            // Monotonic saturating I^2*t accumulation during active overload (never wraps)
            if (!f_past_trip_q && f_past_armed_q && f_past_sval_q && (f_past_i_q > f_past_inom_q)) begin
                assert (i2t_acc_o >= f_past_i2t_q);
            end
        end
    end
`endif
endmodule

// -----------------------------------------------------------------------------
// `ele_power_qfn_top` — 44-Pin QFN-48 Digital Power, SSCB & 2-Jet Manifold ASIC
// -----------------------------------------------------------------------------
// Physical Pin Budget: 2 + 16 + 14 + 12 = 44 signal pins (< 48 QFN-48).
//   * Host BI5 link (16 pins) : 5-wire Bus-Invert (<= 2/5 wire flips/cyc).
//       flit_ctrl_i == 0 : Data nibble rx_dec4 -> wbuf_q[nib_idx_q*4 +: 4], nib++
//       flit_ctrl_i == 1 : Control nibble rx_dec4:
//         4'h1..4'h6 : Execute Atlas cmd_op (LOAD_STATE..SWAP), nib_idx <= 0
//         4'h0       : Local config write indexed by cmd_word_q:
//                        w==0 : {slot_q[7:0], meta_q[15:0]} <= wbuf_q[23:0]
//                        w==1 : {auto_vpd_en_q, sscb_armed_q, dpwm_en_q,
//                                active_m1_man_q, deadtime_q, duty_man_q} <= wbuf_q[16:0]
//                        w==2 : {confirm_len_q, didt_thresh_q, i_hard_max_q,
//                                i_nom_q} <= wbuf_q[50:0]
//                        w==3 : {vpd_auto_mode_en_q, d2_concave_tol_q,
//                                i2t_limit_q} <= {wbuf_q[48], wbuf_q[47:0]}
//                        w==4 : tx_sscb_sel_q <= wbuf_q[0] (1=read SSCB status/i2t)
//         4'h7       : Start 16-beat 5-wire BI5 readout burst of word `cmd_word_q`
//         4'h8..4'hF : cmd_word_q <= rx_dec4[2:0]; nib_idx_q <= 4'd0
module ele_power_qfn_top #(
    parameter int MAX_CHARTS = 16
) (
    input  logic        clk,
    input  logic        rst_n,
    // 5-wire Charge-Minimal Bus-Invert Host Port (`16` pins)
    input  logic        flit_valid_i,
    input  logic        flit_ctrl_i,
    input  logic [4:0]  rx_bi5_i,
    output logic        ready_o,
    output logic        irq_done_o,
    output logic        swap_accepted_o,
    output logic        tx_valid_o,
    output logic [4:0]  tx_bi5_o,
    // Fast Power-Stage Dual-Channel 12-bit ADC Port (`14` pins)
    input  logic        adc_valid_i,
    input  logic        adc_ch_i,
    input  logic [11:0] adc_data_i,
    // 4-Phase Complementary DrMOS PWM + 800V DC SiC SSCB Hardware Pins (`12` pins)
    input  logic        ext_fault_i,
    input  logic        sscb_clr_i,
    output logic [3:0]  pwm_hi_o,
    output logic [3:0]  pwm_lo_o,
    output logic        sscb_gate_en_o,
    output logic        sscb_trip_o
);
    localparam logic [1:0] AUTO_IDLE       = 2'd0;
    localparam logic [1:0] AUTO_WAIT_STEP  = 2'd1;
    localparam logic [1:0] AUTO_READ_DUTY  = 2'd2;
    localparam logic [1:0] AUTO_READ_PHASE = 2'd3;

    // Ingress 5-wire Bus-Invert combinational decode
    logic [3:0] rx_dec4;
    assign rx_dec4 = rx_bi5_i[3:0] ^ {4{rx_bi5_i[4]}};

    // Host staging & configuration registers
    logic [63:0] wbuf_q;
    logic [3:0]  nib_idx_q;
    logic [2:0]  cmd_word_q;
    logic [7:0]  slot_q;
    logic [15:0] meta_q;
    logic        dpwm_en_q;
    logic        sscb_armed_q;
    logic        auto_vpd_en_q;
    logic [1:0]  active_m1_man_q;
    logic [3:0]  deadtime_q;
    logic [7:0]  duty_man_q;
    logic [15:0] i_nom_q;
    logic [15:0] i_hard_max_q;
    logic [15:0] didt_thresh_q;
    logic [15:0] d2_concave_tol_q;
    logic [2:0]  confirm_len_q;
    logic [31:0] i2t_limit_q;
    logic        vpd_auto_mode_en_q;
    logic        tx_sscb_sel_q;

    // Egress 16-beat 5-wire BI5 Tx state
    logic        tx_active_q;
    logic [3:0]  tx_idx_q;
    logic        tx_valid_q;
    logic [4:0]  tx_bi5_q;
    logic        swap_acc_lat_q;

    // Autonomous hardware VPD loop state (`adc_ch_i == 0`)
    logic [1:0]  auto_st_q;
    logic [7:0]  duty_auto_q;
    logic [1:0]  active_m1_auto_q;

    // Atlas macro wires
    logic        atlas_cmd_valid_c;
    logic [2:0]  atlas_cmd_op_c;
    logic [2:0]  atlas_cmd_word_c;
    logic [15:0] atlas_cmd_aux_c;
    logic [63:0] atlas_cmd_data_c;
    logic        atlas_ready_w;
    logic        atlas_step_done_w;
    logic        atlas_swap_done_w;
    logic        atlas_swap_acc_w;
    logic [63:0] atlas_rd_data_w;

    // DPWM + SSCB interconnect wires
    logic [7:0]  dpwm_cnt_w;
    logic        sscb_gate_en_w;
    logic        sscb_trip_lat_w;
    logic [2:0]  sscb_cause_w;
    logic [31:0] sscb_i2t_w;
    logic        sscb_trip_comb_c;
    logic        dpwm_fault_kill_c;
    logic [7:0]  dpwm_duty_c;
    logic [1:0]  dpwm_active_m1_c;

    assign ready_o = atlas_ready_w & (~tx_active_q) & (~tx_valid_q) & (auto_st_q == AUTO_IDLE);

    wire host_issue_atlas_c = flit_valid_i & ready_o & flit_ctrl_i &
                              (rx_dec4 >= 4'd1) & (rx_dec4 <= 4'd6);
    wire auto_vpd_fire_c    = ready_o & (~flit_valid_i) & auto_vpd_en_q &
                              adc_valid_i & (adc_ch_i == 1'b0);

    always_comb begin
        atlas_cmd_valid_c = host_issue_atlas_c | auto_vpd_fire_c;
        atlas_cmd_op_c    = auto_vpd_fire_c
                          ? 3'd5 // CMD_STEP
                          : (host_issue_atlas_c ? rx_dec4[2:0] : 3'd0);
        if (auto_st_q == AUTO_READ_DUTY) begin
            atlas_cmd_word_c = 3'd3; // state[3] = x_q16
        end else if (auto_st_q == AUTO_READ_PHASE) begin
            atlas_cmd_word_c = 3'd5; // state[5] = chart_idx / mode
        end else begin
            atlas_cmd_word_c = cmd_word_q;
        end
        atlas_cmd_aux_c = auto_vpd_fire_c
                        ? (vpd_auto_mode_en_q ? meta_q : 16'd0)
                        : ((host_issue_atlas_c && ((rx_dec4[2:0] == 3'd3) || (rx_dec4[2:0] == 3'd6)))
                           ? {8'd0, slot_q}
                           : meta_q);
        atlas_cmd_data_c = auto_vpd_fire_c
                         ? ({{52{adc_data_i[11]}}, adc_data_i} <<< 8)
                         : wbuf_q;
    end

    ele_flow_atlas_top #(
        .MAX_CHARTS (MAX_CHARTS)
    ) u_atlas (
        .clk             (clk),
        .rst_n           (rst_n),
        .cmd_valid_i     (atlas_cmd_valid_c),
        .cmd_op_i        (atlas_cmd_op_c),
        .cmd_word_i      (atlas_cmd_word_c),
        .cmd_aux_i       (atlas_cmd_aux_c),
        .cmd_data_i      (atlas_cmd_data_c),
        .ready_o         (atlas_ready_w),
        .step_done_o     (atlas_step_done_w),
        .swap_done_o     (atlas_swap_done_w),
        .swap_accepted_o (atlas_swap_acc_w),
        .rd_data_o       (atlas_rd_data_w)
    );

    ele_sscb_trip_atom u_sscb (
        .clk              (clk),
        .rst_n            (rst_n),
        .armed_i          (sscb_armed_q),
        .sample_valid_i   (adc_valid_i & adc_ch_i),
        .clr_rearm_i      (sscb_clr_i),
        .i_meas_i         ({adc_data_i, 4'd0}),
        .i_nom_i          (i_nom_q),
        .i_hard_max_i     (i_hard_max_q),
        .didt_thresh_i    (didt_thresh_q),
        .d2_concave_tol_i (d2_concave_tol_q),
        .confirm_len_i    (confirm_len_q),
        .i2t_limit_i      (i2t_limit_q),
        .gate_en_o        (sscb_gate_en_w),
        .trip_latched_o   (sscb_trip_lat_w),
        .trip_cause_o     (sscb_cause_w),
        .i2t_acc_o        (sscb_i2t_w)
    );

    // 0-cycle Hardware Protection Reflex: SSCB trip or external fault kills DPWM immediately
    assign sscb_trip_comb_c  = rst_n & (sscb_trip_lat_w | (sscb_armed_q & (~sscb_gate_en_w)));
    assign dpwm_fault_kill_c = ext_fault_i | sscb_trip_comb_c;
    assign dpwm_duty_c       = auto_vpd_en_q ? duty_auto_q : duty_man_q;
    assign dpwm_active_m1_c  = auto_vpd_en_q ? active_m1_auto_q : active_m1_man_q;

    ele_dpwm_multiphase #(
        .N_PHASES (4),
        .CNT_W    (8),
        .DT_W     (4)
    ) u_dpwm (
        .clk             (clk),
        .rst_n           (rst_n),
        .en_i            (dpwm_en_q),
        .fault_kill_i    (dpwm_fault_kill_c),
        .active_minus1_i (dpwm_active_m1_c),
        .duty_i          (dpwm_duty_c),
        .deadtime_i      (deadtime_q),
        .phase0_cnt_o    (dpwm_cnt_w),
        .pwm_hi_o        (pwm_hi_o),
        .pwm_lo_o        (pwm_lo_o)
    );

    // 5-Wire Stateful Charge-Minimal Bus-Invert Egress Encoder (<= 2/5 flips/cyc)
    logic [63:0] tx_word_c;
    logic [3:0]  tx_nib_c;
    logic [4:0]  tx_cand0_c;
    logic [4:0]  tx_diff0_c;
    logic [2:0]  tx_pop0_c;
    logic [4:0]  tx_next_bi5_c;

    function automatic [7:0] clamp_q16_duty(input logic [63:0] x_raw_q16);
        logic signed [63:0] x_round;
        begin
            x_round = $signed(x_raw_q16) + 64'sd32768;
            if (x_round <= 64'sd0) begin
                clamp_q16_duty = 8'd0;
            end else if (x_round >= 64'sd16711680) begin
                clamp_q16_duty = 8'd255;
            end else begin
                clamp_q16_duty = x_round[23:16];
            end
        end
    endfunction

    always_comb begin
        tx_word_c     = tx_sscb_sel_q
                      ? {29'd0, sscb_cause_w, sscb_i2t_w}
                      : atlas_rd_data_w;
        tx_nib_c      = tx_active_q ? tx_word_c[tx_idx_q*4 +: 4] : 4'd0;
        tx_cand0_c    = {1'b0, tx_nib_c};
        tx_diff0_c    = tx_bi5_q ^ tx_cand0_c;
        tx_pop0_c     = {2'd0, tx_diff0_c[0]} + {2'd0, tx_diff0_c[1]} +
                        {2'd0, tx_diff0_c[2]} + {2'd0, tx_diff0_c[3]} +
                        {2'd0, tx_diff0_c[4]};
        tx_next_bi5_c = tx_active_q
                      ? ((tx_pop0_c > 3'd2) ? {1'b1, ~tx_nib_c} : tx_cand0_c)
                      : tx_bi5_q;
    end

    wire unused_pwr_qfn_ok = &{1'b0, dpwm_cnt_w, wbuf_q[63:51]};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbuf_q             <= 64'd0;
            nib_idx_q          <= 4'd0;
            cmd_word_q         <= 3'd0;
            slot_q             <= 8'd0;
            meta_q             <= 16'd0;
            dpwm_en_q          <= 1'b0;
            sscb_armed_q       <= 1'b0;
            auto_vpd_en_q      <= 1'b0;
            active_m1_man_q    <= 2'd3;
            deadtime_q         <= 4'd1;
            duty_man_q         <= 8'd0;
            i_nom_q            <= 16'hFFFF;
            i_hard_max_q       <= 16'hFFFF;
            didt_thresh_q      <= 16'hFFFF;
            d2_concave_tol_q   <= 16'd0;
            confirm_len_q      <= 3'd1;
            i2t_limit_q        <= 32'hFFFF_FFFF;
            vpd_auto_mode_en_q <= 1'b0;
            tx_sscb_sel_q      <= 1'b0;
            tx_active_q        <= 1'b0;
            tx_idx_q           <= 4'd0;
            tx_valid_q         <= 1'b0;
            tx_bi5_q           <= 5'd0;
            swap_acc_lat_q     <= 1'b0;
            auto_st_q          <= AUTO_IDLE;
            duty_auto_q        <= 8'd0;
            active_m1_auto_q   <= 2'd0;
        end else begin
            tx_valid_q <= tx_active_q;
            if (atlas_swap_done_w) begin
                swap_acc_lat_q <= atlas_swap_acc_w;
            end

            if (tx_active_q) begin
                tx_bi5_q <= tx_next_bi5_c;
                if (tx_idx_q == 4'd15) begin
                    tx_active_q <= 1'b0;
                    tx_idx_q    <= 4'd0;
                end else begin
                    tx_idx_q <= tx_idx_q + 4'd1;
                end
            end else if (auto_st_q != AUTO_IDLE) begin
                case (auto_st_q)
                    AUTO_WAIT_STEP: begin
                        if (atlas_step_done_w) begin
                            auto_st_q <= AUTO_READ_DUTY;
                        end
                    end
                    AUTO_READ_DUTY: begin
                        duty_auto_q <= clamp_q16_duty(atlas_rd_data_w);
                        auto_st_q   <= AUTO_READ_PHASE;
                    end
                    AUTO_READ_PHASE: begin
                        active_m1_auto_q <= atlas_rd_data_w[17:16];
                        auto_st_q        <= AUTO_IDLE;
                    end
                    default: auto_st_q <= AUTO_IDLE;
                endcase
            end else if (auto_vpd_fire_c) begin
                auto_st_q <= AUTO_WAIT_STEP;
            end else if (flit_valid_i && ready_o) begin
                if (!flit_ctrl_i) begin
                    wbuf_q[nib_idx_q*4 +: 4] <= rx_dec4;
                    nib_idx_q                <= nib_idx_q + 4'd1;
                end else if (rx_dec4[3]) begin
                    cmd_word_q <= rx_dec4[2:0];
                    nib_idx_q  <= 4'd0;
                end else if (rx_dec4 == 4'd7) begin
                    tx_active_q <= 1'b1;
                    tx_idx_q    <= 4'd0;
                    nib_idx_q   <= 4'd0;
                end else if (rx_dec4 == 4'd0) begin
                    nib_idx_q <= 4'd0;
                    case (cmd_word_q)
                        3'd0: begin
                            meta_q <= wbuf_q[15:0];
                            slot_q <= wbuf_q[23:16];
                        end
                        3'd1: begin
                            duty_man_q      <= wbuf_q[7:0];
                            deadtime_q      <= wbuf_q[11:8];
                            active_m1_man_q <= wbuf_q[13:12];
                            dpwm_en_q       <= wbuf_q[14];
                            sscb_armed_q    <= wbuf_q[15];
                            auto_vpd_en_q   <= wbuf_q[16];
                        end
                        3'd2: begin
                            i_nom_q       <= wbuf_q[15:0];
                            i_hard_max_q  <= wbuf_q[31:16];
                            didt_thresh_q <= wbuf_q[47:32];
                            confirm_len_q <= wbuf_q[50:48];
                        end
                        3'd3: begin
                            i2t_limit_q        <= wbuf_q[31:0];
                            d2_concave_tol_q   <= wbuf_q[47:32];
                            vpd_auto_mode_en_q <= wbuf_q[48];
                        end
                        3'd4: begin
                            tx_sscb_sel_q <= wbuf_q[0];
                        end
                        default: begin end
                    endcase
                end else begin
                    // Commands 4'd1..4'd6 forwarded to `u_atlas`
                    nib_idx_q <= 4'd0;
                end
            end
        end
    end

    assign irq_done_o      = rst_n & (atlas_step_done_w | atlas_swap_done_w);
    assign swap_accepted_o = rst_n ? (atlas_swap_done_w ? atlas_swap_acc_w : swap_acc_lat_q) : 1'b0;
    assign tx_valid_o      = tx_valid_q;
    assign tx_bi5_o        = rst_n ? tx_bi5_q : 5'd0;
    assign sscb_gate_en_o  = sscb_gate_en_w;
    assign sscb_trip_o     = sscb_trip_comb_c;

`ifdef FORMAL
    function automatic [2:0] popcount5_pwr(input logic [4:0] v);
        begin
            popcount5_pwr = {2'd0, v[0]} + {2'd0, v[1]} + {2'd0, v[2]} + {2'd0, v[3]} + {2'd0, v[4]};
        end
    endfunction

    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_ready_q;
    logic        f_past_tx_act_q;
    logic [3:0]  f_past_tx_idx_q;
    logic [4:0]  f_past_tx5_q;
    logic [63:0] f_past_wbuf_q;
    logic [3:0]  f_past_nib_idx_q;
    logic [2:0]  f_past_cmd_word_q;
    logic [15:0] f_past_meta_q;
    logic [7:0]  f_past_slot_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q    <= 1'b1;
        f_past_rst_n_q    <= rst_n;
        f_past_ready_q    <= ready_o;
        f_past_tx_act_q   <= tx_active_q;
        f_past_tx_idx_q   <= tx_idx_q;
        f_past_tx5_q      <= tx_bi5_o;
        f_past_wbuf_q     <= wbuf_q;
        f_past_nib_idx_q  <= nib_idx_q;
        f_past_cmd_word_q <= cmd_word_q;
        f_past_meta_q     <= meta_q;
        f_past_slot_q     <= slot_q;
    end

    always_comb begin
        // Pin-level shoot-through impossibility & 0-cycle fault reflex
        assert ((pwm_hi_o & pwm_lo_o) == 4'b0000);
        if (!rst_n || ext_fault_i || sscb_trip_o) begin
            assert (pwm_hi_o == 4'b0000);
            assert (pwm_lo_o == 4'b0000);
        end
        if (!rst_n) begin
            assert (!sscb_gate_en_o && !sscb_trip_o);
            assert (!tx_valid_o && (tx_bi5_o == 5'd0));
            assert (!irq_done_o && !swap_accepted_o);
        end
        // Lossless combinational 5-wire BI5 ingress decode
        assert ((rx_dec4 ^ {4{rx_bi5_i[4]}}) == rx_bi5_i[3:0]);

        if (!ready_o) begin
            assert (!host_issue_atlas_c);
        end

        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin
            // Strict <= 2/5 (< 50%) pad SSO ceiling on every clock edge
            assert (popcount5_pwr(tx_bi5_o ^ f_past_tx5_q) <= 3'd2);
            if (!f_past_tx_act_q) begin
                assert (tx_bi5_o == f_past_tx5_q);
            end
            // Busy host write-ignore invariance
            if (!f_past_ready_q) begin
                assert (wbuf_q == f_past_wbuf_q);
                assert (nib_idx_q == f_past_nib_idx_q);
                assert (cmd_word_q == f_past_cmd_word_q);
                assert (meta_q == f_past_meta_q);
                assert (slot_q == f_past_slot_q);
            end
            // Deterministic 16-nibble Tx readout burst sequencing
            if (f_past_tx_act_q) begin
                assert (tx_valid_o);
                if (f_past_tx_idx_q == 4'd15) begin
                    assert (!tx_active_q);
                end else begin
                    assert (tx_active_q && (tx_idx_q == (f_past_tx_idx_q + 4'd1)));
                end
            end
        end
    end
`endif
endmodule

/* verilator lint_on DECLFILENAME */
`default_nettype wire
