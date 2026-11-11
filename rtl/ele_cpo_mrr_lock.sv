// SPDX-License-Identifier: MIT
// ELE Hardware IP: CPO Silicon Photonics Microring Modulo-FSR & Thermal Lock
//
// Physical & Formal Guarantees (see research/EnergyOptimality.lean, Theorems 34-35):
//   - Wraps round-trip optical phase in 12-bit Free Spectral Range (FSR) units
//     (`[0..4095]` == `2*pi` rad == `1 FSR`), bounding micro-heater detuning
//     strictly `< 1 FSR` across arbitrary multi-FSR die temperature swings.
//   - Pre-biases the micro-heater using 2-Jet compute-thermal feedforward
//     (`ff_delta_phase_i`) to maintain carrier lock (`locked_o = 1`) during
//     sudden XPU power transients with 0 hardware dividers.
//   - Bit-for-bit (0-ULP) identical to `ele_cpo_eval_heater_phase12` in C17.

`default_nettype none

module ele_cpo_mrr_lock (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               valid_i,
    input  logic        [11:0] target_phase12_i,
    input  logic        [11:0] thermal_phase12_i,
    input  logic signed [11:0] ff_delta_phase_i,
    input  logic        [11:0] uw_per_lsb_i,
    input  logic        [11:0] lock_tol12_i,
    input  logic        [11:0] actual_wave_phase12_i,
    output logic        [11:0] heater_phase12_o,
    output logic        [23:0] heater_power_uw_o,
    output logic        [11:0] phase_err12_o,
    output logic               hop_event_o,
    output logic               locked_o
);
    logic        primed_q;
    logic [11:0] ff_mod12_w;
    logic [11:0] pred_phase12_w;
    logic [11:0] heater_next12_w;
    logic [11:0] raw_jump12_w;
    logic        hop_w;
    logic [11:0] eff_uw_lsb_w;
    logic [11:0] eff_tol12_w;
    logic [11:0] total_phase12_w;
    logic [11:0] diff12_w;
    logic [11:0] neg_diff12_w;
    logic [11:0] err12_w;
    logic [23:0] power_uw_w;

    assign eff_uw_lsb_w    = (uw_per_lsb_i == 12'd0) ? 12'd4 : uw_per_lsb_i;
    assign eff_tol12_w     = (lock_tol12_i == 12'd0) ? 12'd64 : lock_tol12_i;

    assign ff_mod12_w      = ff_delta_phase_i;
    assign pred_phase12_w  = thermal_phase12_i + ff_mod12_w;
    assign heater_next12_w = target_phase12_i - pred_phase12_w;

    assign raw_jump12_w    = (heater_next12_w >= heater_phase12_o)
                             ? (heater_next12_w - heater_phase12_o)
                             : (heater_phase12_o - heater_next12_w);
    assign hop_w           = primed_q && (raw_jump12_w > 12'd2048);

    assign total_phase12_w = actual_wave_phase12_i + heater_next12_w;
    assign diff12_w        = target_phase12_i - total_phase12_w;
    assign neg_diff12_w    = 12'd0 - diff12_w;
    assign err12_w         = (diff12_w > 12'd2048) ? neg_diff12_w : diff12_w;
    assign power_uw_w      = {12'd0, heater_next12_w} * {12'd0, eff_uw_lsb_w};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            primed_q          <= 1'b0;
            heater_phase12_o  <= 12'd0;
            heater_power_uw_o <= 24'd0;
            phase_err12_o     <= 12'd0;
            hop_event_o       <= 1'b0;
            locked_o          <= 1'b0;
        end else if (valid_i) begin
            primed_q          <= 1'b1;
            heater_phase12_o  <= heater_next12_w;
            heater_power_uw_o <= power_uw_w;
            phase_err12_o     <= err12_w;
            hop_event_o       <= hop_w;
            locked_o          <= (err12_w <= eff_tol12_w);
        end
    end
endmodule

`default_nettype wire
