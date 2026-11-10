// SPDX-License-Identifier: MIT
// ELE Hardware IP: Stateful 65-Wire Charge-Minimal Bus-Invert + Residual PHY
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorem 13):
//   - Encodes a 64-bit cache/HBM/UCIe lane + 1 stateful polarity wire (65 wires).
//   - Given candidate word `cand = data_in ^ pred_in`, uninverted transmission
//     (inv_next = 0) flips `k = popcount64(cand ^ wire_q) + inv_q` wires, while
//     inverted transmission (inv_next = 1) flips `65 - k` wires.
//   - Because `k + (65 - k) == 65`, selecting `inv_next = (k > 32)` strictly
//     bounds physical simultaneous switching outputs (SSO) at <= 32 / 65 (< 50%)
//     on every beat, halving Simultaneous Switching Noise (SSN) and minimizing
//     1/2 * C * V_DDQ^2 wire-repolarization energy with 0-ULP reconstruction.

`default_nettype none

module ele_cache_bus65_tx (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_i,
    input  logic [63:0] data_i,
    input  logic [63:0] pred_i,
    output logic [63:0] wire_o,
    output logic        inv_o,
    output logic        zero_flip_skip_o
);
    logic [63:0] cand_w;
    logic [63:0] diff_p0_w;
    logic [6:0]  flips_p0_w;
    logic        invert_sel_w;
    logic [63:0] wire_next_w;
    logic        beat_changed_w;

    // 1. Temporal 2-Jet / XOR residual pre-conditioning (0-cycle combinational)
    assign cand_w    = data_i ^ pred_i;
    assign diff_p0_w = cand_w ^ wire_o;

    // 2. Balanced combinational 65-bit Hamming weight tree (64 data + 1 inv_o)
    always_comb begin
        logic [6:0] acc;
        acc = {6'd0, inv_o};
        for (int b = 0; b < 64; b++) begin
            acc = acc + {6'd0, diff_p0_w[b]};
        end
        flips_p0_w = acc;
    end

    // 3. Strict 65-wire majority voter: caps physical flips at <= 32 (< 50%)
    assign invert_sel_w   = (flips_p0_w > 7'd32);
    assign wire_next_w    = invert_sel_w ? ~cand_w : cand_w;
    assign beat_changed_w = (wire_next_w != wire_o) || (invert_sel_w != inv_o);
    assign zero_flip_skip_o = valid_i && !beat_changed_w;

    // 4. Clock-gated physical output driver flip-flops (0 dynamic clock power on skip)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wire_o <= 64'd0;
            inv_o  <= 1'b0;
        end else if (valid_i && beat_changed_w) begin
            wire_o <= wire_next_w;
            inv_o  <= invert_sel_w;
        end
    end
endmodule

module ele_cache_bus65_rx (
    input  logic [63:0] wire_i,
    input  logic        inv_i,
    input  logic [63:0] pred_i,
    output logic [63:0] data_o
);
    // Symmetric 0-ULP combinational reconstruction (< 2 LUT levels)
    logic [63:0] resid_w;
    assign resid_w = inv_i ? ~wire_i : wire_i;
    assign data_o  = resid_w ^ pred_i;
endmodule

`default_nettype wire
