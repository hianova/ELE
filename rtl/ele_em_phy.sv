// SPDX-License-Identifier: MIT
// ele_em_phy.sv — Synthesizable Electromagnetic & Charge-Minimal PHY Primitives
//                 + 42-Pin QFN-48 Charge-Minimal ASIC Top (`ele_flow_qfn_top`) — ELE v7.1.0
//
// Pure IEEE 1800 SystemVerilog leaf hardware primitives & ASIC padframe wrapper
// with idle-cycle operand isolation on CSA popcount trees & magnitude negators:
//   1. `ele_bus_invert65`         : Stateful 65-wire (64 data + 1 polarity)
//                                   interconnect capping wire flips at <= 32/65
//                                   (< 50% Maxwell displacement current).
//   2. `ele_bus_invert17`         : Stateful 17-wire (16 data + 1 polarity)
//                                   narrow ASIC pad PHY capping wire transitions
//                                   at <= 8/17 (< 50% Simultaneous Switching
//                                   Output / SSO displacement current).
//   3. `ele_ternary_mac_dualrail` : Glitch-free dual monotonic magnitude-rail
//                                   1.58-bit balanced ternary MAC (`0-ULP`
//                                   equivalent to signed accumulation without
//                                   two's-complement sign-extension storms).
//   4. `ele_flow_qfn_top`         : 42-pin QFN-48 Charge-Minimal ASIC Padframe
//                                   Top wrapping `ele_flow_atlas_top` + live
//                                   17-wire Bus-Invert ingress/egress PHY.

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
        cand_p0    = valid_i ? {1'b0, data_i} : prev_wire_q;
        diff_p0    = valid_i ? (prev_wire_q ^ cand_p0) : 65'd0;
        pop_p0     = popcount65(diff_p0);
        invert_sel = (pop_p0 > 7'd32);
        next_wire  = valid_i ? (invert_sel ? {1'b1, ~data_i} : cand_p0) : prev_wire_q;
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

module ele_bus_invert17 (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_i,
    input  logic [15:0] data_i,
    output logic [16:0] wire_o,
    output logic [4:0]  flips_o,
    output logic [15:0] rx_data_o
);
    function automatic [4:0] popcount17(input logic [16:0] v);
        logic [15:0] s1;
        logic [11:0] s2;
        logic [7:0]  s3;
        logic [4:0]  s4;
        begin
            for (int i = 0; i < 8; i++) s1[2*i +: 2] = {1'b0, v[2*i]} + {1'b0, v[2*i+1]};
            for (int i = 0; i < 4; i++) s2[3*i +: 3] = {1'b0, s1[4*i +: 2]} + {1'b0, s1[4*i+2 +: 2]};
            for (int i = 0; i < 2; i++) s3[4*i +: 4] = {1'b0, s2[6*i +: 3]} + {1'b0, s2[6*i+3 +: 3]};
            s4         = {1'b0, s3[3:0]} + {1'b0, s3[7:4]};
            popcount17 = s4 + {4'd0, v[16]};
        end
    endfunction

    logic [16:0] prev_wire_q;

    // Count Hamming distance across all 17 physical pads if sent uninverted (pol = 0)
    logic [16:0] cand_p0;
    logic [16:0] diff_p0;
    logic [4:0]  pop_p0;
    logic        invert_sel;
    logic [16:0] next_wire;

    always_comb begin
        cand_p0    = valid_i ? {1'b0, data_i} : prev_wire_q;
        diff_p0    = valid_i ? (prev_wire_q ^ cand_p0) : 17'd0;
        pop_p0     = popcount17(diff_p0);
        invert_sel = (pop_p0 > 5'd8);
        next_wire  = valid_i ? (invert_sel ? {1'b1, ~data_i} : cand_p0) : prev_wire_q;
        flips_o    = invert_sel ? (5'd17 - pop_p0) : pop_p0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_wire_q <= 17'd0;
        end else if (valid_i) begin
            prev_wire_q <= next_wire;
        end
    end

    assign wire_o    = prev_wire_q;
    assign rx_data_o = prev_wire_q[15:0] ^ {16{prev_wire_q[16]}};

`ifdef FORMAL
    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_valid_i_q;
    logic [15:0] f_past_data_q;
    logic [16:0] f_past_wire_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q   <= 1'b1;
        f_past_rst_n_q   <= rst_n;
        f_past_valid_i_q <= valid_i;
        f_past_data_q    <= data_i;
        f_past_wire_q    <= wire_o;
    end

    always_comb begin
        assert (flips_o <= 5'd8);
        assert (popcount17(next_wire ^ prev_wire_q) == flips_o);
        assert ((wire_o[15:0] ^ {16{wire_o[16]}}) == rx_data_o);
        if (!rst_n) begin
            assert (wire_o == 17'd0);
            assert (rx_data_o == 16'd0);
        end else if (f_past_valid_q && f_past_rst_n_q) begin
            if (f_past_valid_i_q) begin
                assert (rx_data_o == f_past_data_q);
                assert (popcount17(wire_o ^ f_past_wire_q) <= 5'd8);
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

    wire mac_act_en = valid_i & trit_nz_i;

    always_comb begin
        act_is_neg  = mac_act_en & act_q16_i[31];
        act_mag     = mac_act_en ? (act_is_neg ? (32'd0 - $unsigned(act_q16_i)) : $unsigned(act_q16_i)) : 32'd0;
        route_neg   = mac_act_en & (trit_neg_i ^ act_is_neg);
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

// -----------------------------------------------------------------------------
// `ele_flow_qfn_top` — 42-Pin QFN-48 Charge-Minimal ASIC Padframe Top
// -----------------------------------------------------------------------------
// Physical Pin Budget: 1 + 1 + 1 + 1 + 17 + 1 + 1 + 1 + 1 + 17 = 42 signal pins
// (fits inside a standard 6x6 mm QFN-48 package with 6 VDD/VSS power/ground pins).
//
// Flit Protocol (when `flit_valid_i && ready_o`):
//   * `flit_ctrl_i == 1'b0` (Data Flit):
//       Writes decoded 16-bit ingress lane `rx_dec` into `wbuf_q[q_idx_q*16 +: 16]`
//       and advances `q_idx_q <= q_idx_q + 2'd1`.
//   * `flit_ctrl_i == 1'b1` (Control Flit `rx_dec[15:0]`):
//       - `rx_dec[15:14] == 2'b00` : Issue Atlas Command (`cmd_op = rx_dec[2:0]`,
//                                    `cmd_word = rx_dec[5:3]`,
//                                    `cmd_aux = {8'd0, rx_dec[13:6]}`, `q_idx <= 0`).
//       - `rx_dec[15:14] == 2'b01` : Issue Atlas Command with full 16-bit `cmd_aux`
//                                    (`cmd_aux = {aux_hi_q, rx_dec[13:6]}`, `q_idx <= 0`).
//       - `rx_dec[15:14] == 2'b10` : `SET_AUX_HI` (`aux_hi_q <= rx_dec[7:0]`).
//       - `rx_dec[15:14] == 2'b11` : Readout Request (`rd_word_q <= rx_dec[2:0]`),
//                                    streaming 4x16b slices of `rd_data_o` through
//                                    stateful `ele_bus_invert17` (`<= 8/17` flips/cyc).
module ele_flow_qfn_top #(
    parameter int MAX_CHARTS = 16
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        flit_valid_i,
    input  logic        flit_ctrl_i,
    input  logic [16:0] rx_bi_lane_i,
    output logic        ready_o,
    output logic        irq_done_o,
    output logic        swap_accepted_o,
    output logic        tx_valid_o,
    output logic [16:0] tx_bi_lane_o
);
    // Combinational 17-wire Bus-Invert ingress pad decoder
    logic [15:0] rx_dec;
    assign rx_dec = rx_bi_lane_i[15:0] ^ {16{rx_bi_lane_i[16]}};

    logic [63:0] wbuf_q;
    logic [1:0]  q_idx_q;
    logic [7:0]  aux_hi_q;
    logic [2:0]  rd_word_q;
    logic        tx_active_q;
    logic [1:0]  tx_idx_q;
    logic        tx_valid_q;
    logic        swap_acc_lat_q;

    logic        atlas_cmd_valid_c;
    logic [2:0]  atlas_cmd_op_c;
    logic [2:0]  atlas_cmd_word_c;
    logic [15:0] atlas_cmd_aux_c;
    logic        atlas_ready_w;
    logic        atlas_step_done_w;
    logic        atlas_swap_done_w;
    logic        atlas_swap_acc_w;
    logic [63:0] atlas_rd_data_w;

    assign ready_o = atlas_ready_w & (~tx_active_q) & (~tx_valid_q);

    wire issue_atlas_cmd_c = flit_valid_i & ready_o & flit_ctrl_i & (~rx_dec[15]);

    always_comb begin
        atlas_cmd_valid_c = issue_atlas_cmd_c;
        atlas_cmd_op_c    = issue_atlas_cmd_c ? rx_dec[2:0] : 3'd0;
        atlas_cmd_word_c  = tx_active_q ? rd_word_q : (issue_atlas_cmd_c ? rx_dec[5:3] : 3'd0);
        atlas_cmd_aux_c   = issue_atlas_cmd_c ? {(rx_dec[14] ? aux_hi_q : 8'd0), rx_dec[13:6]} : 16'd0;
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
        .cmd_data_i      (wbuf_q),
        .ready_o         (atlas_ready_w),
        .step_done_o     (atlas_step_done_w),
        .swap_done_o     (atlas_swap_done_w),
        .swap_accepted_o (atlas_swap_acc_w),
        .rd_data_o       (atlas_rd_data_w)
    );

    // Stateful 17-wire Charge-Minimal Bus-Invert egress pad transceiver
    logic [15:0] tx_slice_c;
    logic [4:0]  tx_phy_flips_w;
    logic [15:0] tx_phy_rx_unused_w;

    assign tx_slice_c = tx_active_q ? atlas_rd_data_w[tx_idx_q*16 +: 16] : 16'd0;

    ele_bus_invert17 u_tx_phy (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_i   (tx_active_q),
        .data_i    (tx_slice_c),
        .wire_o    (tx_bi_lane_o),
        .flips_o   (tx_phy_flips_w),
        .rx_data_o (tx_phy_rx_unused_w)
    );

    wire unused_qfn_ok = &{1'b0, tx_phy_flips_w, tx_phy_rx_unused_w};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wbuf_q         <= 64'd0;
            q_idx_q        <= 2'd0;
            aux_hi_q       <= 8'd0;
            rd_word_q      <= 3'd0;
            tx_active_q    <= 1'b0;
            tx_idx_q       <= 2'd0;
            tx_valid_q     <= 1'b0;
            swap_acc_lat_q <= 1'b0;
        end else begin
            tx_valid_q <= tx_active_q;
            if (atlas_swap_done_w) begin
                swap_acc_lat_q <= atlas_swap_acc_w;
            end

            if (tx_active_q) begin
                if (tx_idx_q == 2'd3) begin
                    tx_active_q <= 1'b0;
                    tx_idx_q    <= 2'd0;
                end else begin
                    tx_idx_q <= tx_idx_q + 2'd1;
                end
            end else if (flit_valid_i && ready_o) begin
                if (!flit_ctrl_i) begin
                    wbuf_q[q_idx_q*16 +: 16] <= rx_dec;
                    q_idx_q                  <= q_idx_q + 2'd1;
                end else begin
                    case (rx_dec[15:14])
                        2'b00, 2'b01: begin
                            q_idx_q <= 2'd0;
                        end
                        2'b10: begin
                            aux_hi_q <= rx_dec[7:0];
                        end
                        2'b11: begin
                            rd_word_q   <= rx_dec[2:0];
                            tx_active_q <= 1'b1;
                            tx_idx_q    <= 2'd0;
                            q_idx_q     <= 2'd0;
                        end
                    endcase
                end
            end
        end
    end

    assign irq_done_o      = rst_n & (atlas_step_done_w | atlas_swap_done_w);
    assign swap_accepted_o = rst_n ? (atlas_swap_done_w ? atlas_swap_acc_w : swap_acc_lat_q) : 1'b0;
    assign tx_valid_o      = tx_valid_q;

`ifdef FORMAL
    function automatic [4:0] popcount17_qfn(input logic [16:0] v);
        logic [15:0] s1;
        logic [11:0] s2;
        logic [7:0]  s3;
        logic [4:0]  s4;
        begin
            for (int i = 0; i < 8; i++) s1[2*i +: 2] = {1'b0, v[2*i]} + {1'b0, v[2*i+1]};
            for (int i = 0; i < 4; i++) s2[3*i +: 3] = {1'b0, s1[4*i +: 2]} + {1'b0, s1[4*i+2 +: 2]};
            for (int i = 0; i < 2; i++) s3[4*i +: 4] = {1'b0, s2[6*i +: 3]} + {1'b0, s2[6*i+3 +: 3]};
            s4             = {1'b0, s3[3:0]} + {1'b0, s3[7:4]};
            popcount17_qfn = s4 + {4'd0, v[16]};
        end
    endfunction

    logic        f_past_valid_q;
    logic        f_past_rst_n_q;
    logic        f_past_ready_q;
    logic        f_past_tx_act_q;
    logic [1:0]  f_past_tx_idx_q;
    logic [63:0] f_past_wbuf_q;
    logic [1:0]  f_past_q_idx_q;
    logic [7:0]  f_past_aux_hi_q;
    logic [2:0]  f_past_rd_word_q;
    logic [16:0] f_past_tx_lane_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q   <= 1'b1;
        f_past_rst_n_q   <= rst_n;
        f_past_ready_q   <= ready_o;
        f_past_tx_act_q  <= tx_active_q;
        f_past_tx_idx_q  <= tx_idx_q;
        f_past_wbuf_q    <= wbuf_q;
        f_past_q_idx_q   <= q_idx_q;
        f_past_aux_hi_q  <= aux_hi_q;
        f_past_rd_word_q <= rd_word_q;
        f_past_tx_lane_q <= tx_bi_lane_o;
    end

    always_comb begin
        // Lossless combinational Rx Bus-Invert decode
        assert ((rx_dec ^ {16{rx_bi_lane_i[16]}}) == rx_bi_lane_i[15:0]);

        if (!rst_n) begin
            assert (!tx_active_q && !tx_valid_o);
            assert (q_idx_q == 2'd0);
            assert (aux_hi_q == 8'd0);
            assert (!irq_done_o && !swap_accepted_o);
            assert (tx_bi_lane_o == 17'd0);
        end

        if (!atlas_ready_w || tx_active_q || tx_valid_o) begin
            assert (!ready_o);
        end
        if (!ready_o) begin
            assert (!atlas_cmd_valid_c);
        end

        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin
            // Strict pad SSO ceiling: <= 8/17 physical egress wires flip on EVERY clock edge
            assert (popcount17_qfn(tx_bi_lane_o ^ f_past_tx_lane_q) <= 5'd8);
            if (!f_past_tx_act_q) begin
                assert (tx_bi_lane_o == f_past_tx_lane_q);
            end
            // Unconditional busy write-ignore invariance
            if (!f_past_ready_q) begin
                assert (wbuf_q == f_past_wbuf_q);
                assert (q_idx_q == f_past_q_idx_q);
                assert (aux_hi_q == f_past_aux_hi_q);
                assert (rd_word_q == f_past_rd_word_q);
            end
            // Deterministic 4-beat Tx readout burst sequencing
            if (f_past_tx_act_q) begin
                assert (tx_valid_o);
                if (f_past_tx_idx_q == 2'd3) begin
                    assert (!tx_active_q);
                end else begin
                    assert (tx_active_q && (tx_idx_q == (f_past_tx_idx_q + 2'd1)));
                end
            end
        end
    end
`endif
endmodule

/* verilator lint_on DECLFILENAME */
`default_nettype wire
