// SPDX-License-Identifier: MIT
// ELE Hardware IP: 16-Phase 48V GaN/SiC Digital VRM Controller (HRPWM)
//
// Synthesizable hardware controller implementing:
//   1. Parallel O(1)-depth thermometric phase-shedding (`phase_mask_o[15:0]`)
//      from `load_amp_i` vs per-phase crossover threshold `amp_per_phase_i`.
//   2. Wrap-safe 17-bit Command-Lookahead Active Pre-Charge: wakes all 16 power
//      stages (`16'hFFFF`) when `{1'b0, upcoming_amp_i} > {1'b0, load_amp_i} + 64`
//      prior to a synchronous GPU Tensor/GEMM launch.
//   3. 2-Jet picosecond Zero-Voltage Switching (ZVS) dead-time resonant loop
//      clamping `t_dead_ps_o` in `[16'd1500, 16'd25000]` ps (0-ULP == C17).

`default_nettype none

module ele_vrm_zvs_ctrl (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               valid_i,
    input  logic        [15:0] load_amp_i,
    input  logic        [15:0] upcoming_amp_i,
    input  logic        [15:0] amp_per_phase_i,
    input  logic signed [15:0] zvs_err_ps_i,
    output logic        [15:0] phase_mask_o,
    output logic        [4:0]  active_phases_o,
    output logic        [15:0] t_dead_ps_o,
    output logic               precharge_active_o,
    output logic               zvs_locked_o
);
    logic               precharge_w;
    logic        [19:0] step_w;
    logic        [19:0] load20_w;
    logic        [15:0] gt_thr_w;
    logic        [15:0] therm_mask_w;
    logic        [4:0]  req_phases_w;
    logic signed [17:0] zvs_vel_q;
    logic signed [17:0] err_ext_w;
    logic signed [17:0] d_err_w;
    logic signed [17:0] next_dead_w;
    logic        [15:0] clamped_dead_w;
    logic        [15:0] err_u_w;
    logic        [15:0] abs_err_w;

    // Parallel balanced popcount wires for active phase count
    logic [7:0][1:0] pc1_w;
    logic [3:0][2:0] pc2_w;
    logic [1:0][3:0] pc3_w;

    // 1. Wrap-safe 17-bit command-lookahead Active Pre-Charge comparator
    assign precharge_w = ({1'b0, upcoming_amp_i} > ({1'b0, load_amp_i} + 17'd64));

    // 2. Parallel multiplier-free thermometric phase comparators (zero serial chain)
    assign step_w   = (amp_per_phase_i == 16'd0) ? 20'd75 : {4'd0, amp_per_phase_i};
    assign load20_w = {4'd0, load_amp_i};

    assign gt_thr_w[0]  = 1'b1;
    assign gt_thr_w[1]  = (load20_w > step_w);
    assign gt_thr_w[2]  = (load20_w > (step_w << 1));
    assign gt_thr_w[3]  = (load20_w > ((step_w << 1) + step_w));
    assign gt_thr_w[4]  = (load20_w > (step_w << 2));
    assign gt_thr_w[5]  = (load20_w > ((step_w << 2) + step_w));
    assign gt_thr_w[6]  = (load20_w > ((step_w << 2) + (step_w << 1)));
    assign gt_thr_w[7]  = (load20_w > ((step_w << 3) - step_w));
    assign gt_thr_w[8]  = (load20_w > (step_w << 3));
    assign gt_thr_w[9]  = (load20_w > ((step_w << 3) + step_w));
    assign gt_thr_w[10] = (load20_w > ((step_w << 3) + (step_w << 1)));
    assign gt_thr_w[11] = (load20_w > ((step_w << 3) + (step_w << 1) + step_w));
    assign gt_thr_w[12] = (load20_w > ((step_w << 3) + (step_w << 2)));
    assign gt_thr_w[13] = (load20_w > ((step_w << 3) + (step_w << 2) + step_w));
    assign gt_thr_w[14] = (load20_w > ((step_w << 4) - (step_w << 1)));
    assign gt_thr_w[15] = (load20_w > ((step_w << 4) - step_w));

    assign therm_mask_w = precharge_w ? 16'hFFFF : gt_thr_w;

    genvar g;
    generate
        for (g = 0; g < 8; g = g + 1) begin : gen_vrm_p1
            assign pc1_w[g] = {1'b0, therm_mask_w[2*g]} + {1'b0, therm_mask_w[2*g + 1]};
        end
        for (g = 0; g < 4; g = g + 1) begin : gen_vrm_p2
            assign pc2_w[g] = {1'b0, pc1_w[2*g]} + {1'b0, pc1_w[2*g + 1]};
        end
        for (g = 0; g < 2; g = g + 1) begin : gen_vrm_p3
            assign pc3_w[g] = {1'b0, pc2_w[2*g]} + {1'b0, pc2_w[2*g + 1]};
        end
    endgenerate

    assign req_phases_w = {1'b0, pc3_w[0]} + {1'b0, pc3_w[1]};

    // 3. 2-Jet ZVS dead-time resonant feedback (0.5 * err + 0.25 * d_err damping)
    assign err_ext_w   = {{2{zvs_err_ps_i[15]}}, zvs_err_ps_i};
    assign d_err_w     = err_ext_w - zvs_vel_q;
    assign next_dead_w = $signed({2'b00, t_dead_ps_o}) - (err_ext_w >>> 1) - (d_err_w >>> 2);

    always_comb begin : dead_time_clamp
        if (next_dead_w < 18'sd1500) begin
            clamped_dead_w = 16'd1500;
        end else if (next_dead_w > 18'sd25000) begin
            clamped_dead_w = 16'd25000;
        end else begin
            clamped_dead_w = next_dead_w[15:0];
        end
    end

    assign err_u_w   = zvs_err_ps_i;
    assign abs_err_w = zvs_err_ps_i[15] ? (16'd0 - err_u_w) : err_u_w;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_mask_o       <= 16'hFFFF;
            active_phases_o    <= 5'd16;
            t_dead_ps_o        <= 16'd8000;
            zvs_vel_q          <= 18'sd0;
            precharge_active_o <= 1'b0;
            zvs_locked_o       <= 1'b0;
        end else if (valid_i) begin
            phase_mask_o       <= therm_mask_w;
            active_phases_o    <= req_phases_w;
            t_dead_ps_o        <= clamped_dead_w;
            zvs_vel_q          <= err_ext_w;
            precharge_active_o <= precharge_w;
            zvs_locked_o       <= (abs_err_w <= 16'd250);
        end
    end
endmodule

`default_nettype wire
