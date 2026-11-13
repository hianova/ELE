// SPDX-License-Identifier: MIT
// ele_primordial.sv — Primordial Physical Silicon Seed Atoms for ELE v6.0.0
//
// Physically honest leaf silicon primitives targeted by the self-hosting
// FLOW 2-Jet lattice bootstrap synthesizer (`tools/ele_bootstrap.c`):
//   1. `ele_mul33_atom`     : Operand-isolated 33x33 -> 66 signed DSP multiplier
//                             aligned to FLOW_MAX_Q16 (+/-0x7FFFFFFF), unsigned
//                             32-bit multi-precision limbs, and 33-bit signed
//                             coordinate steps (`delta_cand`).
//   2. `ele_rad2_step_atom` : Unified operand-isolated 1-step radix-2 shift-subtract
//                             slice shared across 128-bit integer square root
//                             (`isqrt_round_u128`) and 128-by-64 restoring division
//                             (`sym_div128_clamped`), using a single 64-bit subtractor.
//   3. `ele_sram_1rw64`     : True synchronous single-port (1RW) 64-bit SRAM macro
//                             for Chart Arena storage (`$mem_v2` inference, 0 loose
//                             DFF array explosions, 64-bit physical bus alignment).

`default_nettype none
/* verilator lint_off DECLFILENAME */

module ele_mul33_atom (
    input  logic               en_i,
    input  logic signed [32:0] a_i,
    input  logic signed [32:0] b_i,
    output logic signed [65:0] prod_o
);
    logic signed [32:0] iso_a;
    logic signed [32:0] iso_b;

    always_comb begin
        iso_a  = en_i ? a_i : 33'sd0;
        iso_b  = en_i ? b_i : 33'sd0;
        prod_o = iso_a * iso_b;
    end

`ifdef FORMAL
    always_comb begin
        if (!en_i) begin
            assert (iso_a == 33'sd0);
            assert (iso_b == 33'sd0);
            assert (prod_o == 66'sd0);
        end else begin
            assert (iso_a == a_i);
            assert (iso_b == b_i);
        end
    end
`endif
endmodule

module ele_rad2_step_atom (
    input  logic         en_i,
    input  logic         is_div_i,
    input  logic [63:0]  rem_i,
    input  logic [63:0]  acc_i,
    input  logic [127:0] aux_i,
    output logic [63:0]  rem_o,
    output logic [63:0]  acc_o,
    output logic [127:0] aux_o
);
    logic [63:0] r_val;
    logic [63:0] sub_val;
    logic        ovf_hi;
    logic [64:0] diff;
    logic        ge;

    always_comb begin
        if (!en_i) begin
            r_val   = 64'd0;
            sub_val = 64'd0;
            ovf_hi  = 1'b0;
            diff    = 65'd0;
            ge      = 1'b0;
            rem_o   = rem_i;
            acc_o   = acc_i;
            aux_o   = aux_i;
        end else begin
            r_val   = is_div_i ? {rem_i[62:0], acc_i[63]} : {rem_i[61:0], aux_i[127:126]};
            sub_val = is_div_i ? aux_i[63:0]              : {acc_i[61:0], 2'b01};
            ovf_hi  = is_div_i ? rem_i[63]                : (rem_i[63:62] != 2'b00);
            diff    = {1'b0, r_val} - {1'b0, sub_val};
            ge      = ovf_hi | ~diff[64];
            rem_o   = ge ? diff[63:0] : r_val;
            acc_o   = {acc_i[62:0], ge};
            aux_o   = is_div_i ? aux_i : {aux_i[125:0], 2'b00};
        end
    end

`ifdef FORMAL
    always_comb begin
        if (!en_i) begin
            assert (r_val == 64'd0 && sub_val == 64'd0 && diff == 65'd0);
            assert (rem_o == rem_i && acc_o == acc_i && aux_o == aux_i);
        end else if (is_div_i) begin
            logic [63:0] ref_r;
            logic        ref_ge;
            ref_r  = (rem_i << 1) | (acc_i >> 63);
            ref_ge = (rem_i[63] != 1'b0) || (ref_r >= aux_i[63:0]);
            assert (rem_o == (ref_ge ? (ref_r - aux_i[63:0]) : ref_r));
            assert (acc_o == ((acc_i << 1) | {63'd0, ref_ge}));
            assert (aux_o == aux_i);
        end else begin
            logic [63:0] ref_r;
            logic [63:0] ref_cand;
            logic        ref_ge;
            ref_r    = (rem_i << 2) | (aux_i[127:126]);
            ref_cand = {acc_i[61:0], 2'b01};
            ref_ge   = (rem_i[63:62] != 2'b00) || (ref_r >= ref_cand);
            assert (rem_o == (ref_ge ? (ref_r - ref_cand) : ref_r));
            assert (acc_o == ((acc_i << 1) | {63'd0, ref_ge}));
            assert (aux_o == (aux_i << 2));
        end
    end
`endif
endmodule

module ele_sram_1rw64 #(
    parameter int DEPTH  = 128,
    parameter int ADDR_W = $clog2(DEPTH)
) (
    input  logic              clk,
    input  logic              ce_i,
    input  logic              we_i,
    input  logic [ADDR_W-1:0] addr_i,
    input  logic [63:0]       wdata_i,
    output logic [63:0]       rdata_o
);
    logic [63:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (ce_i) begin
            if (we_i) begin
                mem[addr_i] <= wdata_i;
            end
            rdata_o <= mem[addr_i];
        end
    end
endmodule

/* verilator lint_on DECLFILENAME */
`default_nettype wire
