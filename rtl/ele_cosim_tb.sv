// SPDX-License-Identifier: MIT
// ele_cosim_tb.sv — Pure-SV Hardware Wind-Tunnel & CMOS Toggle-Energy Inspector
//
// 100% Pure IEEE 1800 SystemVerilog (`0` `.lean` files, `0` `.c/.h/.cpp` files in ELE).
// Links directly against `../FLOW/flow.h` (`FLOW v6.0.0` golden substrate) via
// IEEE 1800 `import "DPI-C"` and executes 4 silicon sign-off probes:
//   Probe 1 : 20,480-vector 512-bit (`w0..w7`) 0-ULP Differential Wind-Tunnel
//             (4,096 hostile singular corners + 16,384 full-width LFSR fuzz)
//   Probe 2 : RTL Odd-Mirror Gauge Symmetry (`1,024` pairs) & 2,048-step
//             Autonomous `2^-49` Sub-LSB Phase Conservation
//   Probe 3 : Multi-Chart Atlas Gluing, Kan Mode Locks & 1-Cycle Atomic `@swap`
//   Probe 4 : CMOS Dynamic Switching-Energy Telemetry (Core DFF Bit-Toggles
//             from Nonlinear Transient -> Linear -> Tickless Equilibrium, plus
//             `ele_bus_invert65` & `ele_ternary_mac_dualrail` charge savings)

`default_nettype none

module ele_cosim_tb;
    localparam int MAX_CHARTS = 16;

    // Direct IEEE 1800 DPI-C binding to `../FLOW/flow.h` (`-DFLOW_EXPORT_KERNEL`)
    import "DPI-C" function void flow_step(
        output longint unsigned out_w[8],
        input  longint unsigned state_w[8],
        input  longint unsigned bound_w[8]
    );

    import "DPI-C" function void flow_step_atlas(
        output longint unsigned out_w[8],
        input  longint unsigned state_w[8],
        input  longint unsigned arena_w[128],
        input  longint unsigned cap_mask,
        input  longint          u_ext,
        input  shortint unsigned mode_req
    );

    logic clk;
    logic rst_n;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // --- Standalone Core DUT (`ele_flow_core`) ---
    logic         core_start;
    logic         core_ready;
    logic [511:0] core_state_i;
    logic [511:0] core_bound_i;
    logic [63:0]  core_cap_mask_i;
    logic [63:0]  core_u_ext_i;
    logic [15:0]  core_mode_req_i;
    logic         core_done;
    logic [511:0] core_state_o;

    ele_flow_core u_core_dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_i    (core_start),
        .ready_o    (core_ready),
        .state_i    (core_state_i),
        .bound_i    (core_bound_i),
        .cap_mask_i (core_cap_mask_i),
        .u_ext_i    (core_u_ext_i),
        .mode_req_i (core_mode_req_i),
        .done_o     (core_done),
        .state_o    (core_state_o)
    );

    // --- Multi-Chart Atlas Top DUT (`ele_flow_atlas_top`) ---
    logic         top_load_state;
    logic [511:0] top_init_state;
    logic [63:0]  top_cap_mask;
    logic         top_load_chart;
    logic [7:0]   top_load_slot;
    logic [511:0] top_load_bound;
    logic         top_step_start;
    logic [63:0]  top_u_ext;
    logic [15:0]  top_mode_req;
    logic         top_swap_start;
    logic [7:0]   top_swap_slot;
    logic [31:0]  top_swap_epoch;
    logic [511:0] top_swap_cand;
    logic         top_ready;
    logic         top_step_done;
    logic         top_swap_done;
    logic         top_swap_acc;
    logic [511:0] top_state_o;

    ele_flow_atlas_top #(
        .MAX_CHARTS (MAX_CHARTS)
    ) u_atlas_top (
        .clk             (clk),
        .rst_n           (rst_n),
        .load_state_i    (top_load_state),
        .init_state_i    (top_init_state),
        .cap_mask_i      (top_cap_mask),
        .load_chart_i    (top_load_chart),
        .load_slot_i     (top_load_slot),
        .load_bound_i    (top_load_bound),
        .step_start_i    (top_step_start),
        .u_ext_i         (top_u_ext),
        .mode_req_i      (top_mode_req),
        .swap_start_i    (top_swap_start),
        .swap_slot_i     (top_swap_slot),
        .swap_epoch_i    (top_swap_epoch),
        .swap_cand_i     (top_swap_cand),
        .ready_o         (top_ready),
        .step_done_o     (top_step_done),
        .swap_done_o     (top_swap_done),
        .swap_accepted_o (top_swap_acc),
        .state_o         (top_state_o)
    );

    // --- Charge-Minimal EM PHY DUTs (`ele_em_phy.sv`) ---
    logic        bi_valid;
    logic [63:0] bi_data;
    logic [64:0] bi_wire;
    logic [6:0]  bi_flips;
    logic [63:0] bi_rx;

    ele_bus_invert65 u_bus_invert (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_i   (bi_valid),
        .data_i    (bi_data),
        .wire_o    (bi_wire),
        .flips_o   (bi_flips),
        .rx_data_o (bi_rx)
    );

    logic               tm_clr;
    logic               tm_valid;
    logic               tm_nz;
    logic               tm_neg;
    logic signed [31:0] tm_act;
    logic        [47:0] tm_pos_rail;
    logic        [47:0] tm_neg_rail;
    logic signed [47:0] tm_acc;
    logic               tm_zero_skip;

    ele_ternary_mac_dualrail u_ternary_mac (
        .clk         (clk),
        .rst_n       (rst_n),
        .clr_i       (tm_clr),
        .valid_i     (tm_valid),
        .trit_nz_i   (tm_nz),
        .trit_neg_i  (tm_neg),
        .act_q16_i   (tm_act),
        .pos_rail_o  (tm_pos_rail),
        .neg_rail_o  (tm_neg_rail),
        .acc_q16_o   (tm_acc),
        .zero_skip_o (tm_zero_skip)
    );

    // --- Cycle-Accurate Core Datapath DFF Bit-Toggle Monitor (`1,677` DFF bits) ---
    logic [1676:0] core_dff_snap_c;
    logic [1676:0] core_dff_prev_q;
    longint unsigned core_toggle_acc_q;
    logic            toggle_meas_en;

    assign core_dff_snap_c = {
        u_core_dut.st_q,
        u_core_dut.cnt_q,
        u_core_dut.disc_neg_q,
        u_core_dut.div_neg_q,
        u_core_dut.b0_eff_q,
        u_core_dut.g0_q,
        u_core_dut.g1_q,
        u_core_dut.r2_q,
        u_core_dut.tmp64_q,
        u_core_dut.disc_q,
        u_core_dut.isqrt_rad_q,
        u_core_dut.sr_rem_q,
        u_core_dut.sr_acc_q,
        u_core_dut.d_mag_q,
        u_core_dut.div_den_q,
        u_core_dut.delta_cand_q,
        u_core_dut.delta_q,
        u_core_dut.xp_q,
        u_core_dut.out_w_q[7], u_core_dut.out_w_q[6], u_core_dut.out_w_q[5], u_core_dut.out_w_q[4],
        u_core_dut.out_w_q[3], u_core_dut.out_w_q[2], u_core_dut.out_w_q[1], u_core_dut.out_w_q[0]
    };

    function automatic int popcount1677(input logic [1676:0] diff);
        int cnt;
        begin
            cnt = 0;
            for (int i = 0; i < 1677; i++) begin
                if (diff[i]) cnt++;
            end
            popcount1677 = cnt;
        end
    endfunction

    function automatic int popcount64(input logic [63:0] v);
        int cnt;
        begin
            cnt = 0;
            for (int i = 0; i < 64; i++) begin
                if (v[i]) cnt++;
            end
            popcount64 = cnt;
        end
    endfunction

    function automatic int popcount65(input logic [64:0] v);
        int cnt;
        begin
            cnt = 0;
            for (int i = 0; i < 65; i++) begin
                if (v[i]) cnt++;
            end
            popcount65 = cnt;
        end
    endfunction

    function automatic int popcount48(input logic [47:0] v);
        int cnt;
        begin
            cnt = 0;
            for (int i = 0; i < 48; i++) begin
                if (v[i]) cnt++;
            end
            popcount48 = cnt;
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_dff_prev_q   <= 1677'd0;
            core_toggle_acc_q <= 64'd0;
        end else begin
            core_dff_prev_q <= core_dff_snap_c;
            if (toggle_meas_en) begin
                core_toggle_acc_q <= core_toggle_acc_q + 64'(popcount1677(core_dff_snap_c ^ core_dff_prev_q));
            end else begin
                core_toggle_acc_q <= 64'd0;
            end
        end
    end

    // --- Helpers ---
    function automatic [63:0] next_lfsr(input [63:0] s);
        logic [63:0] x;
        begin
            x = s ^ (s << 13);
            x = x ^ (x >> 7);
            x = x ^ (x << 17);
            next_lfsr = x;
        end
    endfunction

    function automatic [63:0] hostile_val(input int idx);
        begin
            case (idx & 15)
                0:  hostile_val = 64'h0000_0000_0000_0000;
                1:  hostile_val = 64'h0000_0000_0000_0001;
                2:  hostile_val = 64'hFFFF_FFFF_FFFF_FFFF;
                3:  hostile_val = 64'h0000_0000_0000_0004;
                4:  hostile_val = 64'hFFFF_FFFF_FFFF_FFFC;
                5:  hostile_val = 64'h0000_0000_0001_0000;
                6:  hostile_val = 64'hFFFF_FFFF_FFFF_0000;
                7:  hostile_val = 64'h0000_0000_7FFF_FFFF; // +FLOW_MAX_Q16
                8:  hostile_val = 64'hFFFF_FFFF_8000_0001; // -FLOW_MAX_Q16
                9:  hostile_val = 64'hFFFF_FFFF_8000_0000; // INT32_MIN sext
                10: hostile_val = 64'h1000_0000_0000_0000; // +FLOW_MAX_J0_Q16
                11: hostile_val = 64'hF000_0000_0000_0000; // -FLOW_MAX_J0_Q16
                12: hostile_val = 64'h0000_2000_0000_0000; // +FLOW_MAX_J1_Q16
                13: hostile_val = 64'hFFFF_E000_0000_0000; // -FLOW_MAX_J1_Q16
                14: hostile_val = 64'h7FFF_FFFF_FFFF_FFFF; // INT64_MAX
                default: hostile_val = 64'h8000_0000_0000_0000; // INT64_MIN
            endcase
        end
    endfunction

    function automatic [511:0] pack_cell(
        input [63:0] w0, input [63:0] w1, input [63:0] w2, input [63:0] w3,
        input [63:0] w4, input [63:0] w5, input [63:0] w6, input [63:0] w7
    );
        begin
            pack_cell = {w7, w6, w5, w4, w3, w2, w1, w0};
        end
    endfunction

    task automatic reset_all();
        begin
            rst_n           = 1'b0;
            toggle_meas_en  = 1'b0;
            core_start      = 1'b0;
            core_state_i    = 512'd0;
            core_bound_i    = 512'd0;
            core_cap_mask_i = 64'hFFFF;
            core_u_ext_i    = 64'd0;
            core_mode_req_i = 16'd0;

            top_load_state  = 1'b0;
            top_init_state  = 512'd0;
            top_cap_mask    = 64'd0;
            top_load_chart  = 1'b0;
            top_load_slot   = 8'd0;
            top_load_bound  = 512'd0;
            top_step_start  = 1'b0;
            top_u_ext       = 64'd0;
            top_mode_req    = 16'd0;
            top_swap_start  = 1'b0;
            top_swap_slot   = 8'd0;
            top_swap_epoch  = 32'd0;
            top_swap_cand   = 512'd0;

            bi_valid        = 1'b0;
            bi_data         = 64'd0;
            tm_clr          = 1'b0;
            tm_valid        = 1'b0;
            tm_nz           = 1'b0;
            tm_neg          = 1'b0;
            tm_act          = 32'sd0;

            repeat (3) @(posedge clk);
            #1;
            rst_n = 1'b1;
            @(posedge clk);
            #1;
        end
    endtask

    task automatic exec_core_step(
        input  logic [511:0] st_in,
        input  logic [511:0] bd_in,
        input  logic [63:0]  cap_in,
        input  logic [63:0]  u_in,
        input  logic [15:0]  mreq_in,
        output logic [511:0] st_out,
        output int           cycles_taken,
        output longint unsigned dff_toggles
    );
        int guard;
        begin
            guard = 0;
            while (!core_ready) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: core_ready timeout");
            end

            core_state_i    = st_in;
            core_bound_i    = bd_in;
            core_cap_mask_i = cap_in;
            core_u_ext_i    = u_in;
            core_mode_req_i = mreq_in;
            toggle_meas_en  = 1'b1;
            core_start      = 1'b1;
            @(posedge clk);
            #1;
            core_start      = 1'b0;
            cycles_taken    = 1;

            while (!core_done) begin
                @(posedge clk);
                #1;
                cycles_taken++;
                if (cycles_taken > 512) $fatal(1, "FAIL: core_done timeout");
            end
            st_out = core_state_o;
            @(posedge clk); // capture S_DONE -> S_IDLE transition toggles
            #1;
            dff_toggles    = core_toggle_acc_q;
            toggle_meas_en = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task automatic exec_top_step(
        input logic [63:0] u_in,
        input logic [15:0] mreq_in
    );
        int guard;
        begin
            guard = 0;
            while (!top_ready) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: top_ready timeout");
            end

            top_u_ext      = u_in;
            top_mode_req   = mreq_in;
            top_step_start = 1'b1;
            @(posedge clk);
            #1;
            top_step_start = 1'b0;

            guard = 0;
            while (!top_step_done) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: top_step_done timeout");
            end
        end
    endtask

    task automatic verify_core_against_c(
        input string        tag,
        input int           vec_id,
        input logic [511:0] st_in,
        input logic [511:0] bd_in,
        input logic [63:0]  cap_in,
        input logic [63:0]  u_in,
        input logic [15:0]  mreq_in
    );
        longint unsigned c_st[8];
        longint unsigned c_bd[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        logic [511:0]    rtl_out;
        logic [511:0]    c_packed;
        int              cyc;
        longint unsigned tog;
        begin
            for (int i = 0; i < 8; i++) begin
                c_st[i] = st_in[i*64 +: 64];
                c_bd[i] = bd_in[i*64 +: 64];
            end
            for (int s = 0; s < 16; s++) begin
                for (int i = 0; i < 8; i++) begin
                    c_arena[s*8 + i] = bd_in[i*64 +: 64];
                end
            end

            if (({48'd0, st_in[351:336]} & cap_in) < 64'd16) begin
                flow_step_atlas(c_out, c_st, c_arena, cap_in, longint'(u_in), mreq_in);
            end else if ((cap_in == 64'hFFFF) && (u_in == 64'd0) && (mreq_in == st_in[335:320])) begin
                flow_step(c_out, c_st, c_bd);
            end else begin
                $fatal(1, "FAIL [%s #%0d]: testbench arena index >= 16!", tag, vec_id);
            end
            c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                 c_out[4], c_out[5], c_out[6], c_out[7]);

            exec_core_step(st_in, bd_in, cap_in, u_in, mreq_in, rtl_out, cyc, tog);
            if ((cyc <= 0) || (tog == 64'hFFFF_FFFF_FFFF_FFFF)) begin
                $fatal(1, "FAIL [%s #%0d]: invalid cycle/toggle telemetry!", tag, vec_id);
            end

            if (rtl_out !== c_packed) begin
                $display("FAIL [%s #%0d]: 512-bit RTL != FLOW C99!", tag, vec_id);
                for (int w = 0; w < 8; w++) begin
                    if (rtl_out[w*64 +: 64] !== c_packed[w*64 +: 64]) begin
                        $display("  lane w%0d: rtl=%016x != c=%016x",
                                 w, rtl_out[w*64 +: 64], c_packed[w*64 +: 64]);
                    end
                end
                $fatal(1);
            end

            // When cap_in == 0xFFFF, u_in == 0, mreq_in == st_in.mode, also cross-check `flow_step`
            if ((cap_in == 64'hFFFF) && (u_in == 64'd0) && (mreq_in == st_in[335:320])) begin
                longint unsigned c_step_out[8];
                logic [511:0]    c_step_packed;
                flow_step(c_step_out, c_st, c_bd);
                c_step_packed = pack_cell(c_step_out[0], c_step_out[1], c_step_out[2], c_step_out[3],
                                          c_step_out[4], c_step_out[5], c_step_out[6], c_step_out[7]);
                if (rtl_out !== c_step_packed) begin
                    $display("FAIL [%s #%0d]: RTL != flow_step!", tag, vec_id);
                    $fatal(1);
                end
            end
        end
    endtask

    // =========================================================================
    // Probe 1: 20,480-Vector Singular & LFSR 512-Bit `0-ULP` Co-Verification
    // =========================================================================
    task automatic run_probe1_differential();
        logic [63:0]  rng;
        logic [511:0] st_v, bd_v;
        logic [63:0]  cap_v, u_v;
        logic [15:0]  mreq_v;
        int           vec_cnt;
        begin
            vec_cnt = 0;
            rng     = 64'hD1B5_4A32_D192_ED03;

            // 1A. 4,096 Structured Hostile Singular Corners (16 x 16 x 16)
            for (int ix = 0; ix < 16; ix++) begin
                for (int ib1 = 0; ib1 < 16; ib1++) begin
                    for (int ib2 = 0; ib2 < 16; ib2++) begin
                        rng = next_lfsr(rng);
                        st_v = pack_cell(
                            hostile_val((ix + ib1) & 15),           // raw_j0
                            hostile_val((ib1 + ib2) & 15),          // raw_j1
                            hostile_val(ib2),                       // raw_j2
                            hostile_val(ix),                        // x
                            rng,                                    // w4 ignored on input
                            {32'hDEAD_BEEF, 16'h0003, rng[15:0]},   // chart=3, mode
                            {rng[31:0], 32'hFFFF_FFFF},             // epoch, old status
                            rng                                     // sub-LSB w7
                        );
                        bd_v = pack_cell(
                            hostile_val((ix ^ ib2) & 15),           // b0
                            (vec_cnt % 4 == 0) ? 64'd0 : hostile_val(ib1), // b1 (0 every 4th)
                            (vec_cnt % 4 == 0) ? 64'd0 : hostile_val(ib2), // b2 (autonomous every 4th!)
                            {hostile_val((ix + 3) & 15)[31:0], hostile_val((ib1 + 7) & 15)[31:0]}, // b_exit, b_enter (incl. inverted!)
                            {hostile_val((ib2 + 5) & 15)[31:0], hostile_val((ix + 9) & 15)[31:0]}, // dt_max (incl. <=0!), u_gain
                            {32'hCAFE_BABE, rng[31:16], rng[15:0]}, // next_lo, next_hi, kan_lock
                            rng,
                            rng
                        );
                        cap_v  = (vec_cnt % 8 == 0) ? 64'hFFFF : {56'd0, (8'd1 << (vec_cnt & 3)) - 8'd1};
                        u_v    = (vec_cnt % 8 == 0) ? 64'd0 : hostile_val((ix + ib1 + ib2) & 15);
                        mreq_v = (vec_cnt % 8 == 0) ? st_v[335:320] : rng[47:32];

                        verify_core_against_c("HOSTILE", vec_cnt, st_v, bd_v, cap_v, u_v, mreq_v);
                        vec_cnt++;
                    end
                end
            end

            // 1B. 16,384 Full-Width LFSR Fuzz Vectors (operator, linear, tunnel, autonomous)
            for (int f = 0; f < 16384; f++) begin
                logic [63:0] r0, r1, r2, r3, r4, r5, r6, r7;
                logic [63:0] b0, b1, b2, b3, b4, b5;
                rng = next_lfsr(rng); r0 = rng;
                rng = next_lfsr(rng); r1 = rng;
                rng = next_lfsr(rng); r2 = rng;
                rng = next_lfsr(rng); r3 = (f % 3 == 0) ? {{33{rng[30]}}, rng[30:0]} : rng;
                rng = next_lfsr(rng); r4 = rng;
                rng = next_lfsr(rng); r5 = rng;
                rng = next_lfsr(rng); r6 = rng;
                rng = next_lfsr(rng); r7 = rng;

                rng = next_lfsr(rng); b0 = {{33{rng[30]}}, rng[30:0]};
                rng = next_lfsr(rng);
                b1 = (f % 5 == 0) ? 64'd0 : {{33{rng[30]}}, rng[30:0]};
                rng = next_lfsr(rng);
                b2 = ((f % 5 == 0) || (f % 5 == 1)) ? 64'd0 : {{33{rng[30]}}, rng[30:0]};
                rng = next_lfsr(rng); b3 = rng;
                rng = next_lfsr(rng); b4 = rng;
                rng = next_lfsr(rng); b5 = rng;

                st_v   = pack_cell(r0, r1, r2, r3, r4, r5, r6, r7);
                bd_v   = pack_cell(b0, b1, b2, b3, b4, b5, r0, r1);
                rng    = next_lfsr(rng);
                u_v    = (f % 4 == 0) ? 64'd0 : {{33{rng[30]}}, rng[30:0]};
                mreq_v = (f % 4 == 0) ? r5[15:0] : rng[47:32];
                cap_v  = (f % 4 == 0) ? 64'hFFFF : 64'h000F;

                verify_core_against_c("FUZZ", f, st_v, bd_v, cap_v, u_v, mreq_v);
                vec_cnt++;
            end

            $display("  [PASS] Probe 1 (512-bit 0-ULP Core vs FLOW C99): %0d / %0d vectors bit-identical (0 defects)",
                     vec_cnt, vec_cnt);
        end
    endtask

    // =========================================================================
    // Probe 2: RTL Odd-Mirror Gauge Parity & 2,048-Step Autonomous Phase Lock
    // =========================================================================
    task automatic run_probe2_symmetry_and_fiber();
        logic [63:0]     rng;
        logic [511:0]    st_p, bd_p, out_p;
        logic [511:0]    st_n, bd_n, out_n;
        logic [63:0]     x_v, b0_v, b1_v, b2_v, u_v;
        int              cyc_p, cyc_n;
        longint unsigned tog_p, tog_n;
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        logic [511:0]    c_packed;
        begin
            rng = 64'h9E37_79B9_7F4A_7C15;
            for (int k = 0; k < 1024; k++) begin
                rng  = next_lfsr(rng); x_v  = {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); b0_v = {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); b1_v = {{35{rng[28]}}, rng[28:0]} | 64'd1;
                rng  = next_lfsr(rng); b2_v = (k % 3 == 0) ? 64'd0 : {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); u_v  = {{35{rng[28]}}, rng[28:0]};

                // Under (x, b0, b2, u) -> (-x, -b0, -b2, -u) with b1, u_gain fixed:
                // G0(-x,-b0,-b2,-u) = -G0(x,b0,b2,u), G1(-...) = +G1(...), R2(-...) = -R2(...),
                // so step(-...) = -step(...), xp(-...) = -xp(...), j0(-...) = -j0(...), j1(-...) = +j1(...)!
                st_p = pack_cell(64'd0, 64'd0, 64'd0, x_v, 64'd0, 64'd0, 64'd0, 64'd0);
                bd_p = pack_cell(b0_v, b1_v, b2_v, {32'h8000_0001, 32'h7FFF_FFFF},
                                 {32'h0010_0000, 32'h0001_4000}, 64'd0, 64'd0, 64'd0);

                st_n = pack_cell(64'd0, 64'd0, 64'd0, 64'd0 - x_v, 64'd0, 64'd0, 64'd0, 64'd0);
                bd_n = pack_cell(64'd0 - b0_v, b1_v, 64'd0 - b2_v, {32'h8000_0001, 32'h7FFF_FFFF},
                                 {32'h0010_0000, 32'h0001_4000}, 64'd0, 64'd0, 64'd0);

                exec_core_step(st_p, bd_p, 64'hFFFF, u_v,         16'd0, out_p, cyc_p, tog_p);
                exec_core_step(st_n, bd_n, 64'hFFFF, 64'd0 - u_v, 16'd0, out_n, cyc_n, tog_n);

                if ((cyc_p != cyc_n)                                  || // symmetric latency!
                    (tog_p == 64'hFFFF_FFFF_FFFF_FFFF)                ||
                    (tog_n == 64'hFFFF_FFFF_FFFF_FFFF)                ||
                    (out_n[255:192] !== (64'd0 - out_p[255:192]))     || // xp
                    (out_n[63:0]    !== (64'd0 - out_p[63:0]))        || // j0
                    (out_n[127:64]  !== out_p[127:64])                || // j1
                    (out_n[191:128] !== (64'd0 - out_p[191:128]))     || // j2
                    (out_n[319:256] !== out_p[319:256])               || // dt_resolved
                    (out_n[511:320] !== out_p[511:320])) begin           // w5..w7
                    $display("FAIL [MIRROR #%0d]: gauge/mirror parity broken in RTL!", k);
                    $fatal(1);
                end
            end

            // 2,048-Step Autonomous 2-Jet Sub-LSB Fiber Conservation on `ele_flow_atlas_top`
            // Initial autonomous quadratic polynomial: J(x) = J0 + J1*(x-x0) + 0.5*J2*(x-x0)^2
            // with non-dyadic `J2 = 43691` (~2/3 in Q16.16) exercising `w7` 2^-49 phase conservation!
            st_p = pack_cell(
                64'sd327680,          // J0 = +5.0
                -64'sd196608,         // J1 = -3.0
                64'sd43691,           // J2 = 43691 / 65536 (~0.66667175)
                64'sd0,               // x0 = 0
                64'd0,
                64'd0,
                64'd0,
                64'd0
            );
            bd_p = pack_cell(
                64'sd0,               // b0 = 0
                64'sd0,               // b1 = 0 (autonomous!)
                64'sd0,               // b2 = 0 (autonomous!)
                {32'h8000_0001, 32'h7FFF_FFFF},
                {32'h0000_0111, 32'h0001_0000}, // small irrational-like dt_max = 273 ticks, u_gain = 1.0
                64'd0,
                {32'd1, 32'd0},
                64'd0
            );

            top_load_state = 1'b1;
            top_init_state = st_p;
            top_cap_mask   = 64'd1;
            @(posedge clk);
            #1;
            top_load_state = 1'b0;

            for (int s = 0; s < 2; s++) begin
                top_load_chart = 1'b1;
                top_load_slot  = 8'(s);
                top_load_bound = bd_p;
                @(posedge clk);
                #1;
            end
            top_load_chart = 1'b0;
            @(posedge clk);
            #1;

            for (int i = 0; i < 8; i++) c_st[i] = st_p[i*64 +: 64];
            for (int s = 0; s < 16; s++) begin
                for (int i = 0; i < 8; i++) c_arena[s*8 + i] = bd_p[i*64 +: 64];
            end

            for (int gen = 0; gen < 2048; gen++) begin
                // Oscillate external target every 256 steps so the autonomous fiber sloshes back & forth
                u_v = ((gen / 256) % 2 == 0) ? 64'sd131072 : -64'sd131072;

                exec_top_step(u_v, 16'd0);

                flow_step_atlas(c_out, c_st, c_arena, 64'd1, longint'(u_v), 16'd0);
                for (int i = 0; i < 8; i++) c_st[i] = c_out[i];
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);

                if (top_state_o !== c_packed) begin
                    $display("FAIL [FIBER gen=%0d]: autonomous 2^-49 sub-LSB drift!", gen);
                    $fatal(1);
                end
            end

            $display("  [PASS] Probe 2 (Odd-Mirror Parity & 2,048-Gen Autonomous 2^-49 Fiber): 0 asymmetries, 0-ULP drift");
        end
    endtask

    // =========================================================================
    // Probe 3: Multi-Chart Silicon Atlas + 1-Cycle Deterministic `@swap`
    // =========================================================================
    task automatic run_probe3_atlas_and_1cycle_swap();
        logic [511:0] chart0, chart1, chart2, chart3, init_st, cand_ok, cand_bad_ep, cand_bad_hi, cand_bad_dt, cand_bad_hyst;
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        logic [511:0]    c_packed;
        begin
            // 4-chart SiC/FLL control manifold:
            //   0: Linear nominal tracker (escalates > +30.0 -> slot 1)
            //   1: Quadratic Miller soft-clamp (recovers < +15.0 -> 0, trips > +55.0 -> 2, locks bit 1)
            //   2: Barrier tunnel hop (disc < 0 -> vertex -> slot 3)
            //   3: Shock-isolated safe drain (u_gain = 0, locks bits 0 & 1, recovers < +12.0 -> 0)
            chart0 = pack_cell(64'sd0, 64'sd65536, 64'sd0,
                               {-32'sd2000000, 32'sd1966080}, {32'sd1638400, 32'sd65536},
                               {32'd0, 8'd0, 8'd1, 16'h0000}, {32'd1, 32'd0}, 64'd0);
            chart1 = pack_cell(64'sd0, 64'sd65536, 64'sd2621,
                               {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                               {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd1, 32'd0}, 64'd0);
            chart2 = pack_cell(64'sd262144, 64'sd0, 64'sd131072,
                               {32'sd655360, 32'sd4587520}, {32'sd3276800, 32'sd0},
                               {32'd0, 8'd3, 8'd3, 16'h0002}, {32'd1, 32'd0}, 64'd0);
            chart3 = pack_cell(-64'sd655360, 64'sd65536, 64'sd0,
                               {32'sd786432, 32'h7FFF_FFFF}, {32'sd1966080, 32'sd0},
                               {32'd0, 8'd0, 8'd3, 16'h0003}, {32'd1, 32'd0}, 64'd0);
            init_st = pack_cell(64'd0, 64'd0, 64'd0, 64'sd655360, 64'd0,
                                {32'd0, 16'd0, 16'h0003}, 64'd0, 64'd0);

            top_load_state = 1'b1;
            top_init_state = init_st;
            top_cap_mask   = 64'd3;
            @(posedge clk);
            #1;
            top_load_state = 1'b0;

            for (int s = 0; s < 4; s++) begin
                top_load_chart = 1'b1;
                top_load_slot  = 8'(s);
                case (s)
                    0: top_load_bound = chart0;
                    1: top_load_bound = chart1;
                    2: top_load_bound = chart2;
                    default: top_load_bound = chart3;
                endcase
                @(posedge clk);
                #1;
            end
            top_load_chart = 1'b0;
            @(posedge clk);
            #1;

            for (int i = 0; i < 8; i++) begin
                c_st[i]         = init_st[i*64 +: 64];
                c_arena[0*8+i]  = chart0[i*64 +: 64];
                c_arena[1*8+i]  = chart1[i*64 +: 64];
                c_arena[2*8+i]  = chart2[i*64 +: 64];
                c_arena[3*8+i]  = chart3[i*64 +: 64];
            end

            // Verify 1-Cycle Atomic `@swap` Acceptance & Rejections
            cand_ok       = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                      {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                                      {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd2, 32'd0}, 64'd0);
            cand_bad_ep   = cand_ok;
            cand_bad_hi   = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                      {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                                      {32'd0, 8'd0, 8'd9, 16'h0002}, {32'd5, 32'd0}, 64'd0); // next_hi=9 > cap_mask=3
            cand_bad_dt   = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                      {32'sd983040, 32'sd3604480}, {32'd0, 32'sd65536},       // dt_max=0
                                      {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_bad_hyst = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                      {32'sd3604480, 32'sd983040}, {32'sd2293760, 32'sd65536}, // b_exit > b_enter
                                      {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);

            // Valid 1-cycle swap into slot 1 at epoch 2 -> MUST complete on the VERY NEXT posedge!
            top_swap_slot  = 8'd1;
            top_swap_epoch = 32'd2;
            top_swap_cand  = cand_ok;
            top_swap_start = 1'b1;
            @(posedge clk);
            #1;
            top_swap_start = 1'b0;
            if (!top_swap_done || !top_swap_acc) begin
                $fatal(1, "FAIL: valid @swap did not complete in 1 deterministic cycle!");
            end
            for (int i = 0; i < 6; i++) c_arena[1*8+i] = cand_ok[i*64 +: 64];
            c_arena[1*8+6] = {32'd2, 32'd0};
            c_arena[1*8+7] = 64'd0;

            // Reject stale epoch=2 replay, bad next_hi, bad dt_max, bad deadband (all in 1 cycle)
            for (int r = 0; r < 4; r++) begin
                top_swap_slot  = 8'd1;
                top_swap_epoch = (r == 0) ? 32'd2 : 32'd5;
                case (r)
                    0: top_swap_cand = cand_bad_ep;
                    1: top_swap_cand = cand_bad_hi;
                    2: top_swap_cand = cand_bad_dt;
                    default: top_swap_cand = cand_bad_hyst;
                endcase
                top_swap_start = 1'b1;
                @(posedge clk);
                #1;
                top_swap_start = 1'b0;
                if (!top_swap_done || top_swap_acc) begin
                    $fatal(1, "FAIL: invalid @swap #%0d was not rejected in 1 cycle!", r);
                end
            end

            // Drive 16 closed-loop surge/recovery steps through all 4 regimes and assert 0-ULP C lockstep
            for (int s = 0; s < 16; s++) begin
                logic [63:0] u_stim;
                logic [15:0] m_stim;
                u_stim = (s < 4) ? (64'sd1638400 + longint'(s) * 64'sd1310720) :
                         (s < 10) ? 64'sd9830400 : 64'sd655360;
                m_stim = (s % 2 == 0) ? 16'h0007 : 16'h0000; // attempts illegal Kan unlocks!

                exec_top_step(u_stim, m_stim);

                flow_step_atlas(c_out, c_st, c_arena, 64'd3, longint'(u_stim), m_stim);
                for (int i = 0; i < 8; i++) c_st[i] = c_out[i];
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);

                if (top_state_o !== c_packed) begin
                    $fatal(1, "FAIL [ATLAS step=%0d]: 512-bit RTL != flow_step_atlas!", s);
                end
            end

            $display("  [PASS] Probe 3 (4-Chart Silicon Lifecycle & 1-Cycle Atomic @swap): 0-ULP C99 lockstep");
        end
    endtask

    // =========================================================================
    // Probe 4: Pure-SV CMOS Switching-Activity (Bit-Toggle Energy) Telemetry
    // =========================================================================
    task automatic run_probe4_cmos_energy_telemetry();
        logic [511:0] st_q_cur, bd_quad, bd_lin, st_next;
        int           cyc_quad, cyc_lin, cyc_idle;
        longint unsigned tog_quad, tog_lin, tog_idle;
        longint unsigned quad_tog_sum, idle_tog_sum;

        logic [63:0] lfsr;
        logic [63:0] vec, raw_prev_wire;
        int          bi_max_flips;
        longint unsigned raw_bus_toggles, inv_bus_toggles;

        logic signed [47:0] ref_acc, ref_acc_prev;
        logic        [47:0] prev_pos, prev_neg;
        longint unsigned    twos_mac_toggles, dual_mac_toggles;
        begin
            // 4A. `ele_flow_core` Datapath Switching-Activity across Regimes
            bd_quad = pack_cell(
                -64'sd1638400,        // b0 = -25.0 (setpoint +25.0 when u=0)
                64'sd65536,           // b1 = 1.0
                64'sd6554,            // b2 = ~0.1
                {-32'sd3276800, 32'sd3276800},
                {32'sd983040, 32'sd65536},
                64'd0, 64'd0, 64'd0
            );
            bd_lin = pack_cell(
                -64'sd1638400,        // b0 = -25.0 (exact setpoint x = +25.0)
                64'sd65536,           // b1 = 1.0
                64'sd0,               // b2 = 0.0 (linear!)
                {-32'sd3276800, 32'sd3276800},
                {32'sd1966080, 32'sd65536},
                64'd0, 64'd0, 64'd0
            );

            // Step 1: Active nonlinear quadratic surge (`x = -18.0 -> target`)
            st_q_cur = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648, 64'd0, 64'd0, 64'd0, 64'd0);
            exec_core_step(st_q_cur, bd_quad, 64'hFFFF, 64'sd393216, 16'd0, st_next, cyc_quad, tog_quad);

            // Step 2: Active linear step (`b2 == 0`: skips 64-cycle `isqrt`!)
            exec_core_step(st_q_cur, bd_lin, 64'hFFFF, 64'sd0, 16'd0, st_next, cyc_lin, tog_lin);

            // Drive linear manifold to exact fixed-point (`x = +25.0`, `G0 == 0`), then measure steady-state!
            st_q_cur = st_next;
            for (int warm = 0; warm < 4; warm++) begin
                exec_core_step(st_q_cur, bd_lin, 64'hFFFF, 64'sd0, 16'd0, st_q_cur, cyc_idle, tog_idle);
            end
            // Measure 16 steady-state equilibrium steps vs 16 active nonlinear transient steps
            quad_tog_sum = 64'd0;
            idle_tog_sum = 64'd0;
            for (int m = 0; m < 16; m++) begin
                int c_tmp;
                longint unsigned t_tmp;
                logic [511:0] s_tmp;
                s_tmp = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648 + longint'(m) * 64'sd13107,
                                  64'd0, 64'd0, {32'(m), 32'd0}, 64'd0);
                exec_core_step(s_tmp, bd_quad, 64'hFFFF, 64'sd524288, 16'd0, s_tmp, c_tmp, t_tmp);
                if (c_tmp <= 0) $fatal(1);
                quad_tog_sum += t_tmp;
            end
            for (int m = 0; m < 16; m++) begin
                exec_core_step(st_q_cur, bd_lin, 64'hFFFF, 64'sd0, 16'd0, st_q_cur, cyc_idle, tog_idle);
                idle_tog_sum += tog_idle;
            end

            if ((cyc_idle >= cyc_lin) || (cyc_lin >= cyc_quad) ||
                (tog_idle >= tog_lin) || (tog_lin >= tog_quad) ||
                (idle_tog_sum * 10 >= quad_tog_sum)) begin
                $fatal(1, "FAIL [ENERGY]: tickless FLOW convergence did not yield expected cycle/toggle cut!");
            end

            // 4B. `ele_bus_invert65` vs Uncoded 64-Wire Bus (`1,024` vectors)
            lfsr            = 64'hCAFE_BABE_DEAD_BEEF;
            bi_max_flips    = 0;
            raw_prev_wire   = 64'd0;
            raw_bus_toggles = 64'd0;
            inv_bus_toggles = 64'd0;

            for (int i = 0; i < 1024; i++) begin
                logic [64:0] prev_bi_wire;
                prev_bi_wire = bi_wire;
                if (i < 512) begin
                    // Adversarial alternating / burst bus patterns (common in wide SIMD/DMA buses)
                    lfsr = next_lfsr(lfsr);
                    vec  = (i % 2 == 0) ? (lfsr & 64'hFFFF_FFFF_FFFF_FF00)
                                        : (~(lfsr & 64'hFFFF_FFFF_FFFF_FF00));
                end else begin
                    lfsr = next_lfsr(lfsr);
                    vec  = lfsr;
                end

                bi_valid = 1'b1;
                bi_data  = vec;
                #1;
                if (bi_flips > 7'd32) begin
                    $fatal(1, "FAIL [PHY]: bus_invert flips=%0d > 32!", bi_flips);
                end
                if (int'(bi_flips) > bi_max_flips) bi_max_flips = int'(bi_flips);

                @(posedge clk);
                #1;
                if (bi_rx !== vec) begin
                    $fatal(1, "FAIL [PHY]: bus_invert rx mismatch!");
                end
                raw_bus_toggles += 64'(popcount64(vec ^ raw_prev_wire));
                inv_bus_toggles += 64'(popcount65(bi_wire ^ prev_bi_wire));
                raw_prev_wire    = vec;
            end
            bi_valid = 1'b0;

            // 4C. `ele_ternary_mac_dualrail` vs 48-Bit Two's-Complement Signed Accumulator (`1,024` steps)
            tm_clr   = 1'b1;
            tm_valid = 1'b0;
            @(posedge clk);
            #1;
            tm_clr           = 1'b0;
            ref_acc          = 48'sd0;
            ref_acc_prev     = 48'sd0;
            prev_pos         = 48'd0;
            prev_neg         = 48'd0;
            twos_mac_toggles = 64'd0;
            dual_mac_toggles = 64'd0;

            for (int k = 0; k < 1024; k++) begin
                logic exp_skip;
                lfsr     = next_lfsr(lfsr);
                tm_valid = 1'b1;
                tm_nz    = (k % 3 != 0); // 33% ternary sparsity
                tm_neg   = (k % 2 == 1); // alternating bipolar signs trigger 2's-comp sign-bit storms!
                tm_act   = (k % 11 == 0) ? 32'sd0 : $signed({20'd0, lfsr[11:0]});
                #1;
                exp_skip = (~tm_nz) | (tm_act == 32'sd0);
                if (tm_zero_skip !== exp_skip) begin
                    $fatal(1, "FAIL [PHY]: ternary zero_skip mismatch!");
                end
                @(posedge clk);
                #1;
                if ((tm_pos_rail < prev_pos) || (tm_neg_rail < prev_neg)) begin
                    $fatal(1, "FAIL [PHY]: non-monotonic dual rail!");
                end
                if (tm_nz && (tm_act != 32'sd0)) begin
                    if (tm_neg) ref_acc = ref_acc - 48'($signed(tm_act));
                    else        ref_acc = ref_acc + 48'($signed(tm_act));
                end
                if (tm_acc !== ref_acc) begin
                    $fatal(1, "FAIL [PHY]: ternary acc=%0d != ref=%0d!", tm_acc, ref_acc);
                end

                twos_mac_toggles += 64'(popcount48(ref_acc ^ ref_acc_prev));
                dual_mac_toggles += 64'(popcount48(tm_pos_rail ^ prev_pos)) +
                                    64'(popcount48(tm_neg_rail ^ prev_neg));
                ref_acc_prev      = ref_acc;
                prev_pos          = tm_pos_rail;
                prev_neg          = tm_neg_rail;
            end
            tm_valid = 1'b0;

            if ((inv_bus_toggles >= raw_bus_toggles) || (dual_mac_toggles >= twos_mac_toggles)) begin
                $fatal(1, "FAIL [ENERGY]: PHY charge-minimal modules did not cut switching toggles!");
            end

            $display("  [PASS] Probe 4 (CMOS Switching-Energy & Charge-Minimal PHY Telemetry):");
            $display("         • FLOW Core Latency/Flips : quad=%0d cyc (%0d flips) -> linear=%0d cyc (%0d flips) -> converged=%0d cyc (%0d flips)",
                     cyc_quad, tog_quad, cyc_lin, tog_lin, cyc_idle, tog_idle);
            $display("         • FLOW Core DFF Toggles   : active_16steps=%0d flips vs converged_16steps=%0d flips (-%0d.%0d%%)",
                     quad_tog_sum, idle_tog_sum,
                     ((quad_tog_sum - idle_tog_sum) * 1000 / quad_tog_sum) / 10,
                     ((quad_tog_sum - idle_tog_sum) * 1000 / quad_tog_sum) % 10);
            $display("         • 65-Wire Bus-Invert PHY  : max_flips=%0d/65 (<=32 cap), wire_toggles=%0d vs raw=%0d (-%0d.%0d%%)",
                     bi_max_flips, inv_bus_toggles, raw_bus_toggles,
                     ((raw_bus_toggles - inv_bus_toggles) * 1000 / raw_bus_toggles) / 10,
                     ((raw_bus_toggles - inv_bus_toggles) * 1000 / raw_bus_toggles) % 10);
            $display("         • Dual-Rail Ternary MAC   : rail_dff_toggles=%0d vs 2s_comp_acc=%0d (-%0d.%0d%%)",
                     dual_mac_toggles, twos_mac_toggles,
                     ((twos_mac_toggles - dual_mac_toggles) * 1000 / twos_mac_toggles) / 10,
                     ((twos_mac_toggles - dual_mac_toggles) * 1000 / twos_mac_toggles) % 10);
        end
    endtask

    initial begin
        $display("=== ELE v6.0.0 Silicon Wind-Tunnel (RTL vs FLOW v6.0.0 Golden C99) ===");
        reset_all();
        run_probe1_differential();
        reset_all();
        run_probe2_symmetry_and_fiber();
        reset_all();
        run_probe3_atlas_and_1cycle_swap();
        reset_all();
        run_probe4_cmos_energy_telemetry();
        $display("=== ALL SILICON & ENERGY PROBES PASSED: total_tapeout_defects = 0 ===");
        $finish;
    end
endmodule

`default_nettype wire
