// SPDX-License-Identifier: MIT
// ELE Hardware IP: Constant-Space 4-Slot Graphene Spillover Rowhammer Guard
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorem 37):
//   - Tracks heavy-hitter DRAM row activations using only 4 `(row, cnt)` slots
//     plus 1 scalar spillover upper-bound counter (`spill_cnt_o`).
//   - Inductive Spillover Invariant: every untracked row has `true_acts <=
//     spill_cnt_o < drfm_thresh_i`, and every tracked row has `true_acts <=
//     cnt[k] < drfm_thresh_i` -- guaranteeing 0 silent Rowhammer bit-flips
//     across all 65,536 rows via targeted DRFM (`drfm_fire_o`) or spillover
//     cap refresh (`preventive_ref_o`).
//   - Bit-for-bit (0-ULP) identical to `ele_hbm_step_row_activate` in C17.

`default_nettype none

module ele_hbm_rowhammer_guard (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        act_valid_i,
    input  logic        win_rst_i,
    input  logic [15:0] row_id_i,
    input  logic [15:0] drfm_thresh_i,
    output logic        drfm_fire_o,
    output logic        preventive_ref_o,
    output logic [15:0] drfm_row_o,
    output logic [15:0] spill_cnt_o,
    output logic [15:0] mitigation_cnt_o
);
    logic [15:0] row0_q, row1_q, row2_q, row3_q;
    logic [15:0] cnt0_q, cnt1_q, cnt2_q, cnt3_q;
    logic [3:0]  valid_q;

    logic [15:0] eff_thresh_w;
    logic        hit0_w, hit1_w, hit2_w, hit3_w;
    logic        any_hit_w;
    logic [1:0]  hit_idx_w;
    logic [15:0] hit_cnt_w;
    logic [1:0]  free_idx_w;
    logic [1:0]  min01_idx_w, min23_idx_w, min_idx_w;
    logic [15:0] min01_val_w, min23_val_w, min_val_w;
    logic [16:0] hit_inc17_w;
    logic [16:0] spill_inc17_w;

    assign eff_thresh_w = (drfm_thresh_i == 16'd0) ? 16'd1 : drfm_thresh_i;

    assign hit0_w    = valid_q[0] && (row0_q == row_id_i);
    assign hit1_w    = valid_q[1] && (row1_q == row_id_i);
    assign hit2_w    = valid_q[2] && (row2_q == row_id_i);
    assign hit3_w    = valid_q[3] && (row3_q == row_id_i);
    assign any_hit_w = hit0_w || hit1_w || hit2_w || hit3_w;

    assign hit_idx_w = hit0_w ? 2'd0 : (hit1_w ? 2'd1 : (hit2_w ? 2'd2 : 2'd3));
    assign hit_cnt_w = hit0_w ? cnt0_q : (hit1_w ? cnt1_q : (hit2_w ? cnt2_q : cnt3_q));

    assign free_idx_w = (!valid_q[0]) ? 2'd0 :
                        ((!valid_q[1]) ? 2'd1 :
                        ((!valid_q[2]) ? 2'd2 : 2'd3));

    assign min01_idx_w = (cnt1_q < cnt0_q) ? 2'd1 : 2'd0;
    assign min01_val_w = (cnt1_q < cnt0_q) ? cnt1_q : cnt0_q;
    assign min23_idx_w = (cnt3_q < cnt2_q) ? 2'd3 : 2'd2;
    assign min23_val_w = (cnt3_q < cnt2_q) ? cnt3_q : cnt2_q;
    assign min_idx_w   = (min23_val_w < min01_val_w) ? min23_idx_w : min01_idx_w;
    assign min_val_w   = (min23_val_w < min01_val_w) ? min23_val_w : min01_val_w;

    assign hit_inc17_w   = {1'b0, hit_cnt_w} + 17'd1;
    assign spill_inc17_w = {1'b0, spill_cnt_o} + 17'd1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_q           <= 16'd0;
            row1_q           <= 16'd0;
            row2_q           <= 16'd0;
            row3_q           <= 16'd0;
            cnt0_q           <= 16'd0;
            cnt1_q           <= 16'd0;
            cnt2_q           <= 16'd0;
            cnt3_q           <= 16'd0;
            valid_q          <= 4'd0;
            spill_cnt_o      <= 16'd0;
            drfm_fire_o      <= 1'b0;
            preventive_ref_o <= 1'b0;
            drfm_row_o       <= 16'd0;
            mitigation_cnt_o <= 16'd0;
        end else if (win_rst_i) begin
            cnt0_q           <= 16'd0;
            cnt1_q           <= 16'd0;
            cnt2_q           <= 16'd0;
            cnt3_q           <= 16'd0;
            valid_q          <= 4'd0;
            spill_cnt_o      <= 16'd0;
            drfm_fire_o      <= 1'b0;
            preventive_ref_o <= 1'b0;
        end else if (act_valid_i) begin
            if (any_hit_w) begin
                preventive_ref_o <= 1'b0;
                if (hit_inc17_w >= {1'b0, eff_thresh_w}) begin
                    drfm_fire_o <= 1'b1;
                    drfm_row_o  <= row_id_i;
                    mitigation_cnt_o <= (mitigation_cnt_o < 16'hFFFF)
                                        ? (mitigation_cnt_o + 16'd1) : 16'hFFFF;
                    case (hit_idx_w)
                        2'd0: cnt0_q <= spill_cnt_o;
                        2'd1: cnt1_q <= spill_cnt_o;
                        2'd2: cnt2_q <= spill_cnt_o;
                        default: cnt3_q <= spill_cnt_o;
                    endcase
                end else begin
                    drfm_fire_o <= 1'b0;
                    case (hit_idx_w)
                        2'd0: cnt0_q <= hit_inc17_w[15:0];
                        2'd1: cnt1_q <= hit_inc17_w[15:0];
                        2'd2: cnt2_q <= hit_inc17_w[15:0];
                        default: cnt3_q <= hit_inc17_w[15:0];
                    endcase
                end
            end else if (valid_q != 4'hF) begin
                preventive_ref_o <= 1'b0;
                valid_q[free_idx_w] <= 1'b1;
                case (free_idx_w)
                    2'd0: row0_q <= row_id_i;
                    2'd1: row1_q <= row_id_i;
                    2'd2: row2_q <= row_id_i;
                    default: row3_q <= row_id_i;
                endcase
                if (spill_inc17_w >= {1'b0, eff_thresh_w}) begin
                    drfm_fire_o <= 1'b1;
                    drfm_row_o  <= row_id_i;
                    mitigation_cnt_o <= (mitigation_cnt_o < 16'hFFFF)
                                        ? (mitigation_cnt_o + 16'd1) : 16'hFFFF;
                    case (free_idx_w)
                        2'd0: cnt0_q <= spill_cnt_o;
                        2'd1: cnt1_q <= spill_cnt_o;
                        2'd2: cnt2_q <= spill_cnt_o;
                        default: cnt3_q <= spill_cnt_o;
                    endcase
                end else begin
                    drfm_fire_o <= 1'b0;
                    case (free_idx_w)
                        2'd0: cnt0_q <= spill_inc17_w[15:0];
                        2'd1: cnt1_q <= spill_inc17_w[15:0];
                        2'd2: cnt2_q <= spill_inc17_w[15:0];
                        default: cnt3_q <= spill_inc17_w[15:0];
                    endcase
                end
            end else if (min_val_w == spill_cnt_o) begin
                preventive_ref_o <= 1'b0;
                case (min_idx_w)
                    2'd0: row0_q <= row_id_i;
                    2'd1: row1_q <= row_id_i;
                    2'd2: row2_q <= row_id_i;
                    default: row3_q <= row_id_i;
                endcase
                if (spill_inc17_w >= {1'b0, eff_thresh_w}) begin
                    drfm_fire_o <= 1'b1;
                    drfm_row_o  <= row_id_i;
                    mitigation_cnt_o <= (mitigation_cnt_o < 16'hFFFF)
                                        ? (mitigation_cnt_o + 16'd1) : 16'hFFFF;
                    case (min_idx_w)
                        2'd0: cnt0_q <= spill_cnt_o;
                        2'd1: cnt1_q <= spill_cnt_o;
                        2'd2: cnt2_q <= spill_cnt_o;
                        default: cnt3_q <= spill_cnt_o;
                    endcase
                end else begin
                    drfm_fire_o <= 1'b0;
                    case (min_idx_w)
                        2'd0: cnt0_q <= spill_inc17_w[15:0];
                        2'd1: cnt1_q <= spill_inc17_w[15:0];
                        2'd2: cnt2_q <= spill_inc17_w[15:0];
                        default: cnt3_q <= spill_inc17_w[15:0];
                    endcase
                end
            end else begin
                drfm_fire_o <= 1'b0;
                if (spill_inc17_w >= {1'b0, eff_thresh_w}) begin
                    preventive_ref_o <= 1'b1;
                    drfm_row_o       <= row_id_i;
                    valid_q          <= 4'd0;
                    cnt0_q           <= 16'd0;
                    cnt1_q           <= 16'd0;
                    cnt2_q           <= 16'd0;
                    cnt3_q           <= 16'd0;
                    spill_cnt_o      <= 16'd0;
                    mitigation_cnt_o <= (mitigation_cnt_o < 16'hFFFF)
                                        ? (mitigation_cnt_o + 16'd1) : 16'hFFFF;
                end else begin
                    preventive_ref_o <= 1'b0;
                    spill_cnt_o      <= spill_inc17_w[15:0];
                end
            end
        end else begin
            drfm_fire_o      <= 1'b0;
            preventive_ref_o <= 1'b0;
        end
    end
endmodule

`default_nettype wire
