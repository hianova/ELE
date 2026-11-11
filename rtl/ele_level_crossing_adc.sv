// SPDX-License-Identifier: MIT
// ELE Hardware IP: Asynchronous Level-Crossing Event ADC & 63-Tap Vernier TDC
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorems 39-40):
//   - Suppresses sub-LSB thermal noise chattering (`|V_in - V_dac| < lsb + hyst`)
//     with 0 digital toggles (`event_fired_o = 0`) during quiet windows.
//   - Resolves sub-cycle crossing arrival times via a division-free 6-stage
//     Vernier delay-line coincidence encoder (`t_slow_ps_i - t_fast_ps_i`).
//   - Bit-for-bit (0-ULP) identical to `ele_event_adc_step` in C17.

`default_nettype none

module ele_level_crossing_adc (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               valid_i,
    input  logic signed [31:0] v_in_uv_i,
    input  logic        [31:0] lsb_step_uv_i,
    input  logic        [31:0] hyst_uv_i,
    input  logic        [15:0] lead_ps_i,
    input  logic        [15:0] t_slow_ps_i,
    input  logic        [15:0] t_fast_ps_i,
    output logic signed [31:0] dac_ref_uv_o,
    output logic               event_fired_o,
    output logic               dir_up_o,
    output logic        [7:0]  vernier_steps_o,
    output logic        [15:0] vernier_fine_ps_o,
    output logic               tdc_overflow_o,
    output logic        [15:0] event_cnt_o,
    output logic        [15:0] quiet_cnt_o
);
    logic               primed_q;
    logic        [31:0] eff_lsb_w;
    logic signed [34:0] vin35_w;
    logic signed [34:0] dac35_w;
    logic signed [34:0] lsb35_w;
    logic signed [34:0] diff35_w;
    logic signed [34:0] thresh35_w;
    logic signed [34:0] dac_up35_w;
    logic signed [34:0] dac_dn35_w;
    logic signed [31:0] dac_up_sat_w;
    logic signed [31:0] dac_dn_sat_w;
    logic               cross_up_w;
    logic               cross_dn_w;

    logic        [15:0] res_ps_w;
    logic        [21:0] lead22_w;
    logic        [21:0] res22_w;
    logic        [21:0] rem5_w, rem4_w, rem3_w, rem2_w, rem1_w;
    logic               b5_w, b4_w, b3_w, b2_w, b1_w, b0_w;
    logic               ovf_w;
    logic        [5:0]  taps6_w;
    logic        [21:0] fine_prod22_w;

    assign eff_lsb_w  = (lsb_step_uv_i == 32'd0) ? 32'd1 : lsb_step_uv_i;
    assign vin35_w    = $signed({{3{v_in_uv_i[31]}}, v_in_uv_i});
    assign dac35_w    = $signed({{3{dac_ref_uv_o[31]}}, dac_ref_uv_o});
    assign lsb35_w    = $signed({3'b000, eff_lsb_w});
    assign diff35_w   = vin35_w - dac35_w;
    assign thresh35_w = lsb35_w + $signed({3'b000, hyst_uv_i});

    assign cross_up_w = primed_q && (diff35_w >= thresh35_w);
    assign cross_dn_w = primed_q && (diff35_w <= (-thresh35_w));

    assign dac_up35_w   = dac35_w + lsb35_w;
    assign dac_dn35_w   = dac35_w - lsb35_w;
    assign dac_up_sat_w = (dac_up35_w > 35'sd2147483647) ? 32'sd2147483647 : dac_up35_w[31:0];
    assign dac_dn_sat_w = (dac_dn35_w < -35'sd2147483648) ? -32'sd2147483648 : dac_dn35_w[31:0];

    // Division-free 6-bit Vernier coincidence encoder (0..63 taps)
    assign res_ps_w = (t_slow_ps_i > t_fast_ps_i) ? (t_slow_ps_i - t_fast_ps_i) : 16'd1;
    assign lead22_w = {6'd0, lead_ps_i};
    assign res22_w  = {6'd0, res_ps_w};

    assign ovf_w  = (lead22_w >= (res22_w << 6));
    assign b5_w   = (lead22_w >= (res22_w << 5));
    assign rem5_w = b5_w ? (lead22_w - (res22_w << 5)) : lead22_w;
    assign b4_w   = (rem5_w   >= (res22_w << 4));
    assign rem4_w = b4_w ? (rem5_w   - (res22_w << 4)) : rem5_w;
    assign b3_w   = (rem4_w   >= (res22_w << 3));
    assign rem3_w = b3_w ? (rem4_w   - (res22_w << 3)) : rem4_w;
    assign b2_w   = (rem3_w   >= (res22_w << 2));
    assign rem2_w = b2_w ? (rem3_w   - (res22_w << 2)) : rem3_w;
    assign b1_w   = (rem2_w   >= (res22_w << 1));
    assign rem1_w = b1_w ? (rem2_w   - (res22_w << 1)) : rem2_w;
    assign b0_w   = (rem1_w   >= res22_w);

    assign taps6_w       = ovf_w ? 6'd63 : {b5_w, b4_w, b3_w, b2_w, b1_w, b0_w};
    assign fine_prod22_w = {16'd0, taps6_w} * res22_w;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            primed_q          <= 1'b0;
            dac_ref_uv_o      <= 32'sd0;
            event_fired_o     <= 1'b0;
            dir_up_o          <= 1'b0;
            vernier_steps_o   <= 8'd0;
            vernier_fine_ps_o <= 16'd0;
            tdc_overflow_o    <= 1'b0;
            event_cnt_o       <= 16'd0;
            quiet_cnt_o       <= 16'd0;
        end else if (valid_i) begin
            if (!primed_q) begin
                primed_q          <= 1'b1;
                dac_ref_uv_o      <= v_in_uv_i;
                event_fired_o     <= 1'b0;
                dir_up_o          <= 1'b0;
                vernier_steps_o   <= 8'd0;
                vernier_fine_ps_o <= 16'd0;
                tdc_overflow_o    <= 1'b0;
                quiet_cnt_o       <= (quiet_cnt_o < 16'hFFFF) ? (quiet_cnt_o + 16'd1) : 16'hFFFF;
            end else if (cross_up_w) begin
                dac_ref_uv_o      <= dac_up_sat_w;
                event_fired_o     <= 1'b1;
                dir_up_o          <= 1'b1;
                vernier_steps_o   <= {2'b00, taps6_w};
                vernier_fine_ps_o <= (fine_prod22_w > 22'd65535) ? 16'hFFFF : fine_prod22_w[15:0];
                tdc_overflow_o    <= ovf_w;
                event_cnt_o       <= (event_cnt_o < 16'hFFFF) ? (event_cnt_o + 16'd1) : 16'hFFFF;
                quiet_cnt_o       <= 16'd0;
            end else if (cross_dn_w) begin
                dac_ref_uv_o      <= dac_dn_sat_w;
                event_fired_o     <= 1'b1;
                dir_up_o          <= 1'b0;
                vernier_steps_o   <= {2'b00, taps6_w};
                vernier_fine_ps_o <= (fine_prod22_w > 22'd65535) ? 16'hFFFF : fine_prod22_w[15:0];
                tdc_overflow_o    <= ovf_w;
                event_cnt_o       <= (event_cnt_o < 16'hFFFF) ? (event_cnt_o + 16'd1) : 16'hFFFF;
                quiet_cnt_o       <= 16'd0;
            end else begin
                event_fired_o     <= 1'b0;
                dir_up_o          <= 1'b0;
                quiet_cnt_o       <= (quiet_cnt_o < 16'hFFFF) ? (quiet_cnt_o + 16'd1) : 16'hFFFF;
            end
        end
    end
endmodule

`default_nettype wire
