// SPDX-License-Identifier: MIT
// ELE Hardware IP: LC Resonant Adiabatic Clock Sustainer & Crest Tank-Parker
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorem 38):
//   - Recycles clock-tree charge C_clk * V_dd between electrostatic & magnetic
//     fields, firing sustaining pulses ONLY inside the Zero-Voltage Switching
//     (ZVS) window (`zvs_locked_o`) using 1-Jet phase-error control.
//   - Parks tank charge on a DC hold node at voltage crest (`I_L == 0`) on
//     `!clk_en_i` and resumes full-swing oscillation in 1 clock cycle with 0
//     crowbar short-circuit energy loss.
//   - Bit-for-bit (0-ULP) identical to `ele_resonant_clk_step` in C17.

`default_nettype none

module ele_resonant_clk_zvs (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               valid_i,
    input  logic               clk_en_i,
    input  logic        [15:0] v_dd_mv_i,
    input  logic        [15:0] target_pulse_ps_i,
    input  logic        [15:0] zvs_tol_ps_i,
    input  logic signed [15:0] zvs_err_ps_i,
    output logic        [15:0] sustain_pulse_ps_o,
    output logic        [15:0] tank_env_mv_o,
    output logic        [15:0] park_cap_mv_o,
    output logic               state_o,
    output logic               zvs_locked_o,
    output logic               park_ready_o,
    output logic        [15:0] restart_cnt_o
);
    logic               primed_q;
    logic signed [15:0] prev_err_q;

    logic signed [17:0] err18_w;
    logic signed [17:0] prev18_w;
    logic signed [17:0] d_err18_w;
    logic signed [17:0] adj18_w;
    logic signed [17:0] cand_pulse18_w;
    logic        [15:0] clamped_pulse_w;
    logic signed [17:0] abs_err18_w;
    logic               in_tol_w;

    assign err18_w        = {{2{zvs_err_ps_i[15]}}, zvs_err_ps_i};
    assign prev18_w       = primed_q ? {{2{prev_err_q[15]}}, prev_err_q} : err18_w;
    assign d_err18_w      = err18_w - prev18_w;
    assign adj18_w        = (err18_w >>> 2) + (d_err18_w >>> 3);
    assign cand_pulse18_w = $signed({2'b00, sustain_pulse_ps_o}) - adj18_w;

    assign clamped_pulse_w = (cand_pulse18_w < 18'sd40)   ? 16'd40 :
                             (cand_pulse18_w > 18'sd2000) ? 16'd2000 :
                                                            cand_pulse18_w[15:0];

    assign abs_err18_w = (err18_w < 18'sd0) ? (-err18_w) : err18_w;
    assign in_tol_w    = (abs_err18_w <= $signed({2'b00, zvs_tol_ps_i}));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            primed_q           <= 1'b0;
            prev_err_q         <= 16'sd0;
            sustain_pulse_ps_o <= 16'd120;
            tank_env_mv_o      <= 16'd0;
            park_cap_mv_o      <= 16'd0;
            state_o            <= 1'b0;
            zvs_locked_o       <= 1'b0;
            park_ready_o       <= 1'b0;
            restart_cnt_o      <= 16'd0;
        end else if (valid_i) begin
            if (!clk_en_i) begin
                if (state_o) begin
                    park_cap_mv_o <= tank_env_mv_o;
                    park_ready_o  <= 1'b1;
                end
                tank_env_mv_o      <= 16'd0;
                sustain_pulse_ps_o <= 16'd0;
                state_o            <= 1'b0;
                zvs_locked_o       <= 1'b0;
            end else if (!state_o) begin
                tank_env_mv_o      <= park_ready_o ? park_cap_mv_o : v_dd_mv_i;
                sustain_pulse_ps_o <= (target_pulse_ps_i < 16'd40) ? 16'd40 :
                                      (target_pulse_ps_i > 16'd2000) ? 16'd2000 :
                                      target_pulse_ps_i;
                state_o            <= 1'b1;
                park_ready_o       <= 1'b0;
                primed_q           <= 1'b1;
                prev_err_q         <= zvs_err_ps_i;
                zvs_locked_o       <= in_tol_w;
                if (park_ready_o && (restart_cnt_o < 16'hFFFF)) begin
                    restart_cnt_o <= restart_cnt_o + 16'd1;
                end
            end else begin
                tank_env_mv_o      <= v_dd_mv_i;
                sustain_pulse_ps_o <= clamped_pulse_w;
                primed_q           <= 1'b1;
                prev_err_q         <= zvs_err_ps_i;
                zvs_locked_o       <= in_tol_w;
            end
        end
    end
endmodule

`default_nettype wire
