// SPDX-License-Identifier: MIT
// ele_em_phy.sv — Synthesizable Electromagnetic & Charge-Minimal PHY Primitives
//
// Pure IEEE 1800 SystemVerilog leaf hardware primitives for ELE Pillar 3 (電磁):
//   1. `ele_bus_invert65`         : Stateful 65-wire (64 data + 1 polarity)
//                                   interconnect capping wire flips at <= 32/65
//                                   (< 50% Maxwell displacement current).
//   2. `ele_ternary_mac_dualrail` : Glitch-free dual monotonic magnitude-rail
//                                   1.58-bit balanced ternary MAC (`0-ULP`
//                                   equivalent to signed accumulation without
//                                   two's-complement sign-extension storms).

`default_nettype none
/* verilator lint_off DECLFILENAME */

module ele_bus_invert65 (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_i,
    input  logic [63:0] data_i,
    output logic [64:0] wire_o,
    output logic [6:0]  flips_o,
    output logic [63:0] rx_data_o
);
    function automatic [6:0] popcount65(input logic [64:0] v);
        logic [63:0] s1;
        logic [47:0] s2;
        logic [31:0] s3;
        logic [19:0] s4;
        logic [6:0]  s5;
        begin
            for (int i = 0; i < 32; i++) s1[2*i +: 2] = {1'b0, v[2*i]} + {1'b0, v[2*i+1]};
            for (int i = 0; i < 16; i++) s2[3*i +: 3] = {1'b0, s1[4*i +: 2]} + {1'b0, s1[4*i+2 +: 2]};
            for (int i = 0; i < 8;  i++) s3[4*i +: 4] = {1'b0, s2[6*i +: 3]} + {1'b0, s2[6*i+3 +: 3]};
            for (int i = 0; i < 4;  i++) s4[5*i +: 5] = {1'b0, s3[8*i +: 4]} + {1'b0, s3[8*i+4 +: 4]};
            s5         = ({2'd0, s4[4:0]} + {2'd0, s4[9:5]}) + ({2'd0, s4[14:10]} + {2'd0, s4[19:15]});
            popcount65 = s5 + {6'd0, v[64]};
        end
    endfunction

    logic [64:0] prev_wire_q;

    // Count Hamming distance across all 65 physical wires if sent uninverted (pol = 0)
    logic [64:0] cand_p0;
    logic [64:0] diff_p0;
    logic [6:0]  pop_p0;
    logic        invert_sel;
    logic [64:0] next_wire;

    always_comb begin
        cand_p0    = {1'b0, data_i};
        diff_p0    = prev_wire_q ^ cand_p0;
        pop_p0     = popcount65(diff_p0);
        invert_sel = (pop_p0 > 7'd32);
        next_wire  = invert_sel ? {1'b1, ~data_i} : cand_p0;
        flips_o    = invert_sel ? (7'd65 - pop_p0) : pop_p0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_wire_q <= 65'd0;
        end else if (valid_i) begin
            prev_wire_q <= next_wire;
        end
    end

    assign wire_o    = prev_wire_q;
    assign rx_data_o = prev_wire_q[63:0] ^ {64{prev_wire_q[64]}};

`ifdef FORMAL
    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_valid_i_q;
    logic [63:0] f_past_data_q;
    logic [64:0] f_past_wire_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q   <= 1'b1;
        f_past_rst_n_q   <= rst_n;
        f_past_valid_i_q <= valid_i;
        f_past_data_q    <= data_i;
        f_past_wire_q    <= wire_o;
    end

    always_comb begin
        assert (flips_o <= 7'd32);
        assert (popcount65(next_wire ^ prev_wire_q) == flips_o);
        assert ((wire_o[63:0] ^ {64{wire_o[64]}}) == rx_data_o);
        if (!rst_n) begin
            assert (wire_o == 65'd0);
            assert (rx_data_o == 64'd0);
        end else if (f_past_valid_q && f_past_rst_n_q) begin
            if (f_past_valid_i_q) begin
                assert (rx_data_o == f_past_data_q);
                assert (popcount65(wire_o ^ f_past_wire_q) <= 7'd32);
            end else begin
                assert (wire_o == f_past_wire_q);
            end
        end
    end
`endif
endmodule

module ele_ternary_mac_dualrail (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               clr_i,
    input  logic               valid_i,
    input  logic               trit_nz_i,
    input  logic               trit_neg_i,
    input  logic signed [31:0] act_q16_i,
    output logic        [47:0] pos_rail_o,
    output logic        [47:0] neg_rail_o,
    output logic signed [47:0] acc_q16_o,
    output logic               zero_skip_o
);
    logic [47:0] pos_q;
    logic [47:0] neg_q;

    logic        act_is_neg;
    logic [31:0] act_mag;
    logic        route_neg;

    always_comb begin
        act_is_neg  = act_q16_i[31];
        act_mag     = act_is_neg ? (32'd0 - $unsigned(act_q16_i)) : $unsigned(act_q16_i);
        route_neg   = trit_neg_i ^ act_is_neg;
        zero_skip_o = valid_i & (~trit_nz_i | (act_q16_i == 32'sd0));
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_q <= 48'd0;
            neg_q <= 48'd0;
        end else if (clr_i) begin
            pos_q <= 48'd0;
            neg_q <= 48'd0;
        end else if (valid_i && trit_nz_i && (act_q16_i != 32'sd0)) begin
            if (route_neg) begin
                neg_q <= neg_q + {16'd0, act_mag};
            end else begin
                pos_q <= pos_q + {16'd0, act_mag};
            end
        end
    end

    assign pos_rail_o = pos_q;
    assign neg_rail_o = neg_q;
    assign acc_q16_o  = $signed(pos_q) - $signed(neg_q);

`ifdef FORMAL
    logic signed [47:0] f_ref_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_ref_q <= 48'sd0;
        end else if (clr_i) begin
            f_ref_q <= 48'sd0;
        end else if (valid_i && trit_nz_i) begin
            if (trit_neg_i) begin
                f_ref_q <= f_ref_q - $signed({{16{act_q16_i[31]}}, act_q16_i});
            end else begin
                f_ref_q <= f_ref_q + $signed({{16{act_q16_i[31]}}, act_q16_i});
            end
        end
    end

    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_clr_q;
    logic        f_past_valid_i_q;
    logic        f_past_zskip_q;
    logic [47:0] f_past_pos_q;
    logic [47:0] f_past_neg_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q   <= 1'b1;
        f_past_rst_n_q   <= rst_n;
        f_past_clr_q     <= clr_i;
        f_past_valid_i_q <= valid_i;
        f_past_zskip_q   <= zero_skip_o;
        f_past_pos_q     <= pos_q;
        f_past_neg_q     <= neg_q;
    end

    always_comb begin
        assert (f_ref_q == ($signed(pos_q) - $signed(neg_q)));
        assert (acc_q16_o == f_ref_q);
        if (!rst_n) begin
            assert (pos_q == 48'd0 && neg_q == 48'd0 && f_ref_q == 48'sd0);
        end else if (f_past_valid_q && f_past_rst_n_q && !f_past_clr_q) begin
            if (!f_past_valid_i_q || f_past_zskip_q) begin
                assert (pos_q == f_past_pos_q && neg_q == f_past_neg_q);
            end
            assert ((pos_q == f_past_pos_q) || (neg_q == f_past_neg_q));
        end
    end
`endif
endmodule

/* verilator lint_on DECLFILENAME */
`default_nettype wire
