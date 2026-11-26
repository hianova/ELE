// SPDX-License-Identifier: MIT
// ele_sscb800_ip.sv — AI 800V DC Rack SiC Solid-State Circuit Breaker IP (Commercial Eval)
//
// Builds directly atop formally verified `ele_primordial.sv` & `ele_power_phy.sv`:
//   1. `ele_huber_median3_atom` : 3-tap median EMI guard blocking arbitrary 1-sample switching
//                                spikes (`50 V/ns` SiC noise) while preserving monotonic ramps.
//   2. `ele_sscb800_ip`         : Full BMC/PMBus-ready 800V DC SSCB IP integrating:
//                                  - EMI spike guard + Auto-Zero thermal Hall offset subtraction
//                                  - 2-Jet (dI/dt + d^2I/dt^2 concavity) + adiabatic I^2*t core
//                                  - 0-cycle gate-driver DESAT emergency bypass kill
//                                  - 32-entry freeze-on-trip fault waveform Blackbox recorder
//                                  - 32-bit CSR register bank + embedded Yosys formal proofs

`default_nettype none
/* verilator lint_off DECLFILENAME */

module ele_huber_median3_atom (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        en_i,
    input  logic        emi_guard_en_i,
    input  logic        sample_valid_i,
    input  logic [15:0] x_i,
    input  logic [15:0] spike_tol_i,
    output logic [15:0] med_o,
    output logic [15:0] d1_o,
    output logic [15:0] d2_o,
    output logic [1:0]  warm_o,
    output logic        spike_rej_o
);
    logic [15:0] d1_q;
    logic [15:0] d2_q;
    logic [1:0]  warm_q;

    logic [15:0] lo_ab_c;
    logic [15:0] hi_ab_c;
    logic [15:0] med3_c;
    logic [15:0] abs_diff_c;

    always_comb begin
        lo_ab_c = (d1_q < d2_q) ? d1_q : d2_q;
        hi_ab_c = (d1_q < d2_q) ? d2_q : d1_q;
        if (x_i < lo_ab_c) begin
            med3_c = lo_ab_c;
        end else if (x_i > hi_ab_c) begin
            med3_c = hi_ab_c;
        end else begin
            med3_c = x_i;
        end

        if (emi_guard_en_i && (warm_q == 2'd2)) begin
            med_o = med3_c;
        end else begin
            med_o = x_i;
        end
    end

    assign abs_diff_c  = (x_i >= med_o) ? (x_i - med_o) : (med_o - x_i);
    assign spike_rej_o = rst_n & en_i & emi_guard_en_i & sample_valid_i &
                         (warm_q == 2'd2) & (abs_diff_c > spike_tol_i);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d1_q   <= 16'd0;
            d2_q   <= 16'd0;
            warm_q <= 2'd0;
        end else if (!en_i) begin
            d1_q   <= 16'd0;
            d2_q   <= 16'd0;
            warm_q <= 2'd0;
        end else if (sample_valid_i) begin
            d2_q   <= d1_q;
            d1_q   <= x_i;
            warm_q <= (warm_q == 2'd2) ? 2'd2 : (warm_q + 2'd1);
        end
    end

    assign d1_o   = d1_q;
    assign d2_o   = d2_q;
    assign warm_o = warm_q;

`ifdef FORMAL
    always_comb begin
        if (emi_guard_en_i && (warm_q == 2'd2)) begin
            // Median always bounded by window extrema and Theorem 12 single-spike ceiling
            assert ((med_o <= x_i) || (med_o <= d1_q) || (med_o <= d2_q));
            assert ((med_o >= x_i) || (med_o >= d1_q) || (med_o >= d2_q));
            assert (med_o <= hi_ab_c);
            assert (med_o >= lo_ab_c);
        end
    end
`endif
endmodule

module ele_sscb800_ip (
    input  logic        clk,
    input  logic        rst_n,
    // 100 MHz System Clock + 20 MSPS ADC (sample_valid_i) & Hardware SiC Gate-Driver Reflex Interface
    input  logic        sample_valid_i,
    input  logic [15:0] i_raw_i,
    input  logic        desat_fault_i,
    output logic        gate_en_o,
    output logic        trip_latched_o,
    output logic [2:0]  trip_cause_o,
    output logic        desat_latched_o,
    // 32-Bit BMC / PMBus-Style Control & Status Register (CSR) Bus
    input  logic        csr_wr_i,
    input  logic        csr_rd_i,
    input  logic [5:0]  csr_addr_i,
    input  logic [31:0] csr_wdata_i,
    output logic [31:0] csr_rdata_o
);
    localparam logic [31:0] IP_ID_CONST = 32'hE1E8_0001;

    // CSR Register File
    logic        armed_q;
    logic        emi_guard_en_q;
    logic [15:0] i_nom_q;
    logic [15:0] i_hard_max_q;
    logic [15:0] didt_thresh_q;
    logic [15:0] d2_concave_tol_q;
    logic [2:0]  confirm_len_q;
    logic [15:0] spike_tol_q;
    logic [7:0]  rearm_band_q;
    logic [31:0] i2t_limit_q;
    logic [15:0] i_offset_q;

    // Supervisory & Blackbox State
    logic        desat_latched_q;
    logic [15:0] peak_i_q;
    logic [7:0]  emi_rej_cnt_q;
    logic [4:0]  bb_wptr_q;
    logic [5:0]  bb_count_q;
    logic [4:0]  bb_rd_idx_q;
    logic [31:0] bb_mem_q [0:31];

    // Wire Connections
    logic [15:0] i_filt_w;
    logic [15:0] emi_d1_w;
    logic [15:0] emi_d2_w;
    logic [1:0]  emi_warm_w;
    logic        spike_rej_w;

    logic [15:0] i_corr_c;
    logic [15:0] i_atom_c;
    logic        clr_rearm_c;
    logic        autozero_cmd_c;
    logic        rearm_band_ok_c;
    logic        rearm_success_c;

    logic        atom_gate_en_w;
    logic        atom_trip_w;
    logic [2:0]  atom_cause_w;
    logic [31:0] atom_i2t_w;

    logic        trip_any_q;
    logic        trip_now_any_c;
    logic        bb_wr_en_c;
    logic [4:0]  bb_slot_c;
    logic [31:0] bb_word_c;

    // Suppress unused signal warnings for CSR reads & FORMAL-only EMI history taps
    wire unused_csr_ok = &{1'b0, csr_rd_i, csr_addr_i[1:0], csr_wdata_i[31:27],
                           trip_now_any_c, emi_d1_w, emi_d2_w, emi_warm_w};

    ele_huber_median3_atom u_emi_guard (
        .clk            (clk),
        .rst_n          (rst_n),
        .en_i           (armed_q & (~trip_any_q)),
        .emi_guard_en_i (emi_guard_en_q),
        .sample_valid_i (sample_valid_i),
        .x_i            (i_raw_i),
        .spike_tol_i    (spike_tol_q),
        .med_o          (i_filt_w),
        .d1_o           (emi_d1_w),
        .d2_o           (emi_d2_w),
        .warm_o         (emi_warm_w),
        .spike_rej_o    (spike_rej_w)
    );

    assign trip_any_q     = atom_trip_w | desat_latched_q;
    assign autozero_cmd_c = csr_wr_i & (csr_addr_i[5:2] == 4'h1) & csr_wdata_i[2];
    assign clr_rearm_c    = csr_wr_i & (csr_addr_i[5:2] == 4'h1) & csr_wdata_i[3];

    always_comb begin
        // Thermal zero-point offset removal (autozero_cmd captures Hall baseline while open)
        i_corr_c        = (i_filt_w > i_offset_q) ? (i_filt_w - i_offset_q) : 16'd0;
        rearm_band_ok_c = (i_corr_c <= {8'd0, rearm_band_q});
        i_atom_c        = rearm_band_ok_c ? 16'd0 : i_corr_c;
        rearm_success_c = trip_any_q & clr_rearm_c & rearm_band_ok_c;
    end

    ele_sscb_trip_atom u_core_sscb (
        .clk              (clk),
        .rst_n            (rst_n),
        .armed_i          (armed_q & (~desat_latched_q) & (~desat_fault_i)),
        .sample_valid_i   (sample_valid_i),
        .clr_rearm_i      (clr_rearm_c),
        .i_meas_i         (i_atom_c),
        .i_nom_i          (i_nom_q),
        .i_hard_max_i     (i_hard_max_q),
        .didt_thresh_i    (didt_thresh_q),
        .d2_concave_tol_i (d2_concave_tol_q),
        .confirm_len_i    (confirm_len_q),
        .i2t_limit_i      (i2t_limit_q),
        .gate_en_o        (atom_gate_en_w),
        .trip_latched_o   (atom_trip_w),
        .trip_cause_o     (atom_cause_w),
        .i2t_acc_o        (atom_i2t_w)
    );

    always_comb begin
        // 0-cycle gate-driver DESAT emergency bypass kill combined with 2-Jet core gate_en_o
        gate_en_o       = rst_n & armed_q & (~desat_fault_i) & (~desat_latched_q) & atom_gate_en_w;
        trip_latched_o  = rst_n & trip_any_q;
        trip_cause_o    = atom_cause_w;
        desat_latched_o = rst_n & desat_latched_q;

        // Capture waveform samples before/at fault, freeze immediately upon any trip
        trip_now_any_c = armed_q & (~gate_en_o) & sample_valid_i;
        bb_wr_en_c     = sample_valid_i & armed_q & (~trip_any_q);

        // Chronological Blackbox readout index (0 = oldest, 31 = newest when full)
        bb_slot_c = (bb_count_q >= 6'd32) ? (bb_wptr_q + bb_rd_idx_q) : bb_rd_idx_q;
        bb_word_c = bb_mem_q[bb_slot_c];

        csr_rdata_o = 32'd0;
        case (csr_addr_i[5:2])
            4'h0: csr_rdata_o = IP_ID_CONST;
            4'h1: csr_rdata_o = {30'd0, emi_guard_en_q, armed_q};
            4'h2: csr_rdata_o = {i_hard_max_q, i_nom_q};
            4'h3: csr_rdata_o = {d2_concave_tol_q, didt_thresh_q};
            4'h4: csr_rdata_o = {5'd0, rearm_band_q, spike_tol_q, confirm_len_q};
            4'h5: csr_rdata_o = i2t_limit_q;
            4'h6: csr_rdata_o = {i_offset_q, emi_rej_cnt_q, 2'd0, desat_latched_q,
                                 atom_cause_w, trip_latched_o, gate_en_o};
            4'h7: csr_rdata_o = {2'd0, bb_count_q, 3'd0, bb_wptr_q, peak_i_q};
            4'h8: csr_rdata_o = atom_i2t_w;
            4'h9: csr_rdata_o = {27'd0, bb_rd_idx_q};
            4'hA: csr_rdata_o = bb_word_c;
            default: csr_rdata_o = 32'd0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed_q          <= 1'b0;
            emi_guard_en_q   <= 1'b1;
            i_nom_q          <= 16'd1600;     // 400 A @ 0.25 A/LSB
            i_hard_max_q     <= 16'd3600;     // 900 A @ 0.25 A/LSB
            didt_thresh_q    <= 16'd20;       // 100 A/us @ 20 MSPS (50 ns) & 0.25 A/LSB
            d2_concave_tol_q <= 16'd1;
            confirm_len_q    <= 3'd3;         // 3 consecutive samples (150 ns)
            spike_tol_q      <= 16'd800;      // 200 A jump threshold for EMI log
            rearm_band_q     <= 8'd4;         // +/-1 A zero-current rearm band
            i2t_limit_q      <= 32'd500000000;
            i_offset_q       <= 16'd0;
            desat_latched_q  <= 1'b0;
            peak_i_q         <= 16'd0;
            emi_rej_cnt_q    <= 8'd0;
            bb_wptr_q        <= 5'd0;
            bb_count_q       <= 6'd0;
            bb_rd_idx_q      <= 5'd0;
            for (int k = 0; k < 32; k++) begin
                bb_mem_q[k] <= 32'd0;
            end
        end else begin
            // CSR Configuration Writes (AutoZero allowed whenever breaker is disarmed/open)
            if (autozero_cmd_c && !armed_q) begin
                i_offset_q <= i_raw_i;
            end
            if (csr_wr_i) begin
                case (csr_addr_i[5:2])
                    4'h1: begin
                        armed_q        <= csr_wdata_i[0];
                        emi_guard_en_q <= csr_wdata_i[1];
                    end
                    4'h2: begin
                        i_nom_q      <= csr_wdata_i[15:0];
                        i_hard_max_q <= csr_wdata_i[31:16];
                    end
                    4'h3: begin
                        didt_thresh_q    <= csr_wdata_i[15:0];
                        d2_concave_tol_q <= csr_wdata_i[31:16];
                    end
                    4'h4: begin
                        confirm_len_q <= csr_wdata_i[2:0];
                        spike_tol_q   <= csr_wdata_i[18:3];
                        rearm_band_q  <= csr_wdata_i[26:19];
                    end
                    4'h5: i2t_limit_q <= csr_wdata_i;
                    4'h9: bb_rd_idx_q <= csr_wdata_i[4:0];
                    default: begin end
                endcase
            end

            // DESAT sticky latch & zero-current rearm
            if (desat_latched_q) begin
                if (clr_rearm_c && rearm_band_ok_c && !desat_fault_i) begin
                    desat_latched_q <= 1'b0;
                end
            end else if (armed_q && desat_fault_i) begin
                desat_latched_q <= 1'b1;
            end

            // Clear peak & Blackbox on successful rearm
            if (rearm_success_c && (!desat_latched_q || !desat_fault_i)) begin
                peak_i_q      <= 16'd0;
                emi_rej_cnt_q <= 8'd0;
                bb_wptr_q     <= 5'd0;
                bb_count_q    <= 6'd0;
            end else begin
                if (sample_valid_i && armed_q && !trip_any_q) begin
                    if (i_corr_c > peak_i_q) begin
                        peak_i_q <= i_corr_c;
                    end
                    if (spike_rej_w && (emi_rej_cnt_q != 8'hFF)) begin
                        emi_rej_cnt_q <= emi_rej_cnt_q + 8'd1;
                    end
                    // Record {raw_i, corr_i} until fault trips; freeze identically after trip
                    if (bb_wr_en_c) begin
                        bb_mem_q[bb_wptr_q] <= {i_raw_i, i_corr_c};
                        bb_wptr_q           <= bb_wptr_q + 5'd1;
                        if (bb_count_q < 6'd32) begin
                            bb_count_q <= bb_count_q + 6'd1;
                        end
                    end
                end
            end
        end
    end

`ifdef FORMAL
    logic       f_past_valid_q;
    logic       f_past_rst_n_q;
    logic       f_past_trip_q;
    logic       f_past_clr_q;
    logic       f_past_rearm_ok_q;
    logic [4:0] f_past_wptr_q;

    initial f_past_valid_q = 1'b0;
    always @(posedge clk) begin
        f_past_valid_q    <= 1'b1;
        f_past_rst_n_q    <= rst_n;
        f_past_trip_q     <= trip_any_q;
        f_past_clr_q      <= clr_rearm_c;
        f_past_rearm_ok_q <= rearm_band_ok_c;
        f_past_wptr_q     <= bb_wptr_q;
    end

    always_comb begin
        // 1. Reset & 0-cycle hardware DESAT emergency kill
        if (!rst_n || !armed_q || desat_fault_i || trip_latched_o) begin
            assert (!gate_en_o);
        end
        if (!rst_n) begin
            assert (!trip_latched_o && !desat_latched_o);
            assert (bb_wptr_q == 5'd0);
        end

        // 2. Top-level Theorem 12 (EMI 1-sample spike ceiling invariant)
        if (emi_guard_en_q && (emi_warm_w == 2'd2) &&
            (emi_d1_w < i_hard_max_q) && (emi_d2_w < i_hard_max_q)) begin
            assert (i_filt_w < i_hard_max_q);
            assert (i_corr_c < i_hard_max_q);
        end

        // 3. Blackbox write is strictly gated off whenever tripped
        if (trip_any_q) begin
            assert (!bb_wr_en_c);
        end

        // 4. Blackbox pointer freeze & sticky latch under non-rearm cycles
        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin
            if (f_past_trip_q && !(f_past_clr_q && f_past_rearm_ok_q)) begin
                assert (trip_any_q);
                assert (!gate_en_o);
                assert (bb_wptr_q == f_past_wptr_q);
            end
        end
    end
`endif
endmodule
