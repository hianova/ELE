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

    // Balanced 6-stage binary Hamming-weight adder tree wires (O(log2 W) depth)
    logic [31:0][1:0] c1_w;
    logic [15:0][2:0] c2_w;
    logic [7:0][3:0]  c3_w;
    logic [3:0][4:0]  c4_w;
    logic [1:0][5:0]  c5_w;

    // 1. Operand-isolated XOR residual pre-conditioning (quiescent when !valid_i)
    assign cand_w    = valid_i ? (data_i ^ pred_i) : wire_o;
    assign diff_p0_w = cand_w ^ wire_o;

    // 2. Balanced 6-level binary popcount tree (64 data bits + 1 polarity wire)
    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : gen_pop_s1
            assign c1_w[g] = {1'b0, diff_p0_w[2*g]} + {1'b0, diff_p0_w[2*g + 1]};
        end
        for (g = 0; g < 16; g = g + 1) begin : gen_pop_s2
            assign c2_w[g] = {1'b0, c1_w[2*g]} + {1'b0, c1_w[2*g + 1]};
        end
        for (g = 0; g < 8; g = g + 1) begin : gen_pop_s3
            assign c3_w[g] = {1'b0, c2_w[2*g]} + {1'b0, c2_w[2*g + 1]};
        end
        for (g = 0; g < 4; g = g + 1) begin : gen_pop_s4
            assign c4_w[g] = {1'b0, c3_w[2*g]} + {1'b0, c3_w[2*g + 1]};
        end
        for (g = 0; g < 2; g = g + 1) begin : gen_pop_s5
            assign c5_w[g] = {1'b0, c4_w[2*g]} + {1'b0, c4_w[2*g + 1]};
        end
    endgenerate

    assign flips_p0_w = ({1'b0, c5_w[0]} + {1'b0, c5_w[1]}) + {6'd0, inv_o};

    // 3. Strict 65-wire majority voter + O(1) parallel zero-flip skip detector
    assign invert_sel_w     = (flips_p0_w > 7'd32);
    assign wire_next_w      = invert_sel_w ? ~cand_w : cand_w;
    assign beat_changed_w   = !((&{inv_o, diff_p0_w}) || (~|{inv_o, diff_p0_w}));
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

// AXI4-Stream / UCIe Valid-Ready Charge-Minimal 65-Wire Transceiver Bridge
// True physical 65-wire univalence: `rx_pred_q` updates solely from RX's own
// decoded `m_data_o` upon downstream handshake (`m_valid_o && m_ready_i`),
// requiring zero out-of-band predictor wires across the physical channel.
module ele_cache_bus65_axi_bridge (
    input  logic        clk,
    input  logic        rst_n,
    // Upstream AXI-Stream Source
    input  logic        s_valid_i,
    output logic        s_ready_o,
    input  logic [63:0] s_data_i,
    // Downstream AXI-Stream Sink
    output logic        m_valid_o,
    input  logic        m_ready_i,
    output logic [63:0] m_data_o,
    // Physical 65-Wire Interconnect Telemetry
    output logic [63:0] phy_wire_o,
    output logic        phy_inv_o,
    output logic        zero_flip_skip_o
);
    logic        s_fire_w;
    logic        m_fire_w;
    logic [63:0] tx_pred_q;
    logic [63:0] rx_pred_q;

    assign s_ready_o = rst_n && (m_ready_i || !m_valid_o);
    assign s_fire_w  = s_valid_i && s_ready_o;
    assign m_fire_w  = m_valid_o && m_ready_i;

    ele_cache_bus65_tx u_tx (
        .clk              (clk),
        .rst_n            (rst_n),
        .valid_i          (s_fire_w),
        .data_i           (s_data_i),
        .pred_i           (tx_pred_q),
        .wire_o           (phy_wire_o),
        .inv_o            (phy_inv_o),
        .zero_flip_skip_o (zero_flip_skip_o)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_valid_o <= 1'b0;
            tx_pred_q <= 64'd0;
            rx_pred_q <= 64'd0;
        end else begin
            if (m_fire_w) begin
                rx_pred_q <= m_data_o;
            end
            if (s_fire_w) begin
                m_valid_o <= 1'b1;
                tx_pred_q <= s_data_i;
            end else if (m_ready_i) begin
                m_valid_o <= 1'b0;
            end
        end
    end

    ele_cache_bus65_rx u_rx (
        .wire_i (phy_wire_o),
        .inv_i  (phy_inv_o),
        .pred_i (rx_pred_q),
        .data_o (m_data_o)
    );
endmodule

`default_nettype wire
