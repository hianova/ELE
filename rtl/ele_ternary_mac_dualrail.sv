// SPDX-License-Identifier: MIT
// ELE Hardware IP: Multiplier-Free 1.58-Bit Ternary Dual Monotonic Rail MAC
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorem 16):
//   - Eliminates 32x32 hardware multipliers AND two's-complement zero-crossing
//     sign-extension carry avalanches (0x0000_0000 <-> 0xFFFF_FFFF).
//   - Partitions signed activations `act_q16_i` * ternary weights `trit_i` in
//     {-1, 0, +1} onto two monotonic unsigned 48-bit magnitude rails
//     (`pos_rail_o`, `neg_rail_o`), each independently clock-gated:
//       * `trit_i == 2'b00 || act_q16_i == 0` -> BOTH rails clock-gated (0 flips)
//       * `w * x > 0` -> ONLY `pos_rail_o` enabled (`neg_rail_o` 0 flips)
//       * `w * x < 0` -> ONLY `neg_rail_o` enabled (`pos_rail_o` 0 flips)
//   - Operand-isolated terminal subtraction (`readout_en_i`) holds `acc_o`
//     quiescent during intermediate vector cycles and evaluates exact 0-ULP
//     `pos_rail_o - neg_rail_o` once upon tile completion.

`default_nettype none

module ele_ternary_mac_dualrail (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               clr_i,
    input  logic               valid_i,
    input  logic               readout_en_i,
    input  logic signed [31:0] act_q16_i,
    input  logic        [1:0]  trit_i,       // 2'b00: 0, 2'b01: +1, 2'b10: -1
    output logic signed [47:0] acc_o,
    output logic        [47:0] pos_rail_o,
    output logic        [47:0] neg_rail_o,
    output logic               zero_skip_o
);
    logic        w_pos_w;
    logic        w_neg_w;
    logic        act_zero_w;
    logic        act_neg_w;
    logic [31:0] act_u_w;
    logic [31:0] mag_w;
    logic        en_pos_w;
    logic        en_neg_w;
    logic [47:0] base_pos_w;
    logic [47:0] base_neg_w;
    logic [47:0] term_pos_w;
    logic [47:0] term_neg_w;

    assign w_pos_w     = (trit_i == 2'b01);
    assign w_neg_w     = (trit_i == 2'b10);
    assign act_zero_w  = (act_q16_i == 32'sd0);
    assign act_neg_w   = act_q16_i[31];
    assign act_u_w     = act_q16_i;
    assign mag_w       = act_neg_w ? (32'd0 - act_u_w) : act_u_w;

    assign en_pos_w    = valid_i && !act_zero_w && ((w_pos_w && !act_neg_w) || (w_neg_w && act_neg_w));
    assign en_neg_w    = valid_i && !act_zero_w && ((w_pos_w && act_neg_w)  || (w_neg_w && !act_neg_w));
    assign zero_skip_o = valid_i && !en_pos_w && !en_neg_w;

    assign base_pos_w  = clr_i ? 48'd0 : pos_rail_o;
    assign base_neg_w  = clr_i ? 48'd0 : neg_rail_o;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_rail_o <= 48'd0;
        end else if (clr_i || en_pos_w) begin
            pos_rail_o <= base_pos_w + (en_pos_w ? {16'd0, mag_w} : 48'd0);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            neg_rail_o <= 48'd0;
        end else if (clr_i || en_neg_w) begin
            neg_rail_o <= base_neg_w + (en_neg_w ? {16'd0, mag_w} : 48'd0);
        end
    end

    // Operand-isolated terminal subtractor (0 intermediate borrow avalanches)
    assign term_pos_w = readout_en_i ? pos_rail_o : 48'd0;
    assign term_neg_w = readout_en_i ? neg_rail_o : 48'd0;
    assign acc_o      = $signed(term_pos_w - term_neg_w);
endmodule

// Reference Baseline 1: Conventional Two's-Complement Sequential Ternary MAC
// (Suffers repeated sign-extension carry avalanches across bits [47:16] on 0-crossings)
module ele_ternary_mac_twos_comp_ref (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               clr_i,
    input  logic               valid_i,
    input  logic signed [31:0] act_q16_i,
    input  logic        [1:0]  trit_i,
    output logic signed [47:0] acc_o
);
    logic signed [47:0] sext_act_w;
    logic signed [47:0] base_acc_w;
    assign sext_act_w = {{16{act_q16_i[31]}}, act_q16_i};
    assign base_acc_w = clr_i ? 48'sd0 : acc_o;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_o <= 48'sd0;
        end else if (valid_i) begin
            if (trit_i == 2'b01) begin
                acc_o <= base_acc_w + sext_act_w;
            end else if (trit_i == 2'b10) begin
                acc_o <= base_acc_w - sext_act_w;
            end else if (clr_i) begin
                acc_o <= 48'sd0;
            end
        end else if (clr_i) begin
            acc_o <= 48'sd0;
        end
    end
endmodule

// Reference Baseline 2: Conventional 32x16 Signed Multiplier-Accumulator (MAC)
// (Quantifies CMOS logic gate-area savings of multiplier-free ternary silicon)
module ele_int32_mul_mac_ref (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               clr_i,
    input  logic               valid_i,
    input  logic signed [31:0] act_q16_i,
    input  logic signed [15:0] weight_i16_i,
    output logic signed [47:0] acc_o
);
    logic signed [47:0] prod_w;
    logic signed [47:0] base_acc_w;
    assign prod_w     = act_q16_i * weight_i16_i;
    assign base_acc_w = clr_i ? 48'sd0 : acc_o;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_o <= 48'sd0;
        end else if (valid_i) begin
            acc_o <= base_acc_w + prod_w;
        end else if (clr_i) begin
            acc_o <= 48'sd0;
        end
    end
endmodule

`default_nettype wire
