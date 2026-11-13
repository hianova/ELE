// SPDX-License-Identifier: MIT
// ele_cosim_tb.sv — Pure-SV Hardware Wind-Tunnel & CMOS Toggle-Energy Inspector
//
// Links directly against `../FLOW/flow.h` (`FLOW v6.0.0` golden substrate) via
// IEEE 1800 `import "DPI-C"` with Verilator X-propagation poisoning
// (`--x-assign unique --x-initial unique`) across the `< 160`-pin 64-bit
// streaming bus interfaces of `ele_flow_core` (141 pins) and
// `ele_flow_atlas_top` (157 pins, `ele_sram_1rw64` 1RW SRAM macro):
//   Probe 1 : 512-bit (`w0..w7`) 0-ULP Differential Wind-Tunnel (20,480 vectors)
//   Probe 2 : Odd-Mirror Gauge Symmetry & Autonomous `2^-49` Sub-LSB Fiber Lock
//   Probe 3 : Multi-Chart `ele_sram_1rw64` Atlas Gluing, Hardware Slot-Aliasing
//             Clamp, Kan Mode Locks, Release-Ordered `@swap` Firewall &
//             In-Flight Step Immunity
//   Probe 4 : CMOS Dynamic Switching-Energy Telemetry & Full Bipolar Ternary MAC
//   Probe 5 : 3,688 Analytical Multipole `2^-120` / `2^-124` Cross-Limb Knife-Edges
//             + 8 Exact `2^-49` Convergence Cliffs (isqrt, udiv, sub_valid, Schmitt)
//   Probe 6 : SEARU Zero-Memory 1-Bit `DualChaosState` + `Epiphany` Phase-Boundary
//             Re-Heating Crucible (Multi-Chart `ele_sram_1rw64` + `@swap` Lockstep)
//   Probe 7 : FSM Handshake Bubble Chaos (`0..17` cyc), Busy Illegal-Write Hammering
//             & Exhaustive Cycle-by-Cycle Mid-Flight `rst_n` Abort/0-ULP Recovery

`default_nettype none

module ele_cosim_tb;
    localparam int MAX_CHARTS = 16;

    // 64-bit Atlas Bus Command Opcodes (matching `ele_flow_atlas_top`)
    localparam logic [2:0] CMD_NOP         = 3'd0;
    localparam logic [2:0] CMD_LOAD_STATE  = 3'd1;
    localparam logic [2:0] CMD_SET_CAP     = 3'd2;
    localparam logic [2:0] CMD_WRITE_CHART = 3'd3;
    localparam logic [2:0] CMD_STAGE_CAND  = 3'd4;
    localparam logic [2:0] CMD_STEP        = 3'd5;
    localparam logic [2:0] CMD_SWAP        = 3'd6;

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

    import "DPI-C" function byte unsigned ele_dpi_try_swap(
        inout longint unsigned arena_w[128],
        input longint unsigned idx,
        input longint unsigned cap_mask,
        input int unsigned     new_epoch,
        input longint unsigned cand_w[8]
    );

    logic clk;
    logic rst_n;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // --- Standalone Core DUT (`ele_flow_core`, 141 physical pins) ---
    logic        core_wr_valid;
    logic [1:0]  core_wr_kind;
    logic [2:0]  core_wr_idx;
    logic [63:0] core_wr_data;
    logic        core_ready;
    logic        core_done;
    logic [2:0]  core_rd_idx;
    logic [63:0] core_rd_data;

    ele_flow_core u_core_dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .wr_valid_i (core_wr_valid),
        .wr_kind_i  (core_wr_kind),
        .wr_idx_i   (core_wr_idx),
        .wr_data_i  (core_wr_data),
        .ready_o    (core_ready),
        .done_o     (core_done),
        .rd_idx_i   (core_rd_idx),
        .rd_data_o  (core_rd_data)
    );

    // --- Multi-Chart Atlas Top DUT (`ele_flow_atlas_top`, 157 physical pins) ---
    logic        top_cmd_valid;
    logic [2:0]  top_cmd_op;
    logic [2:0]  top_cmd_word;
    logic [15:0] top_cmd_aux;
    logic [63:0] top_cmd_data;
    logic        top_ready;
    logic        top_step_done;
    logic        top_swap_done;
    logic        top_swap_acc;
    logic [63:0] top_rd_data;

    ele_flow_atlas_top #(
        .MAX_CHARTS (MAX_CHARTS)
    ) u_atlas_top (
        .clk             (clk),
        .rst_n           (rst_n),
        .cmd_valid_i     (top_cmd_valid),
        .cmd_op_i        (top_cmd_op),
        .cmd_word_i      (top_cmd_word),
        .cmd_aux_i       (top_cmd_aux),
        .cmd_data_i      (top_cmd_data),
        .ready_o         (top_ready),
        .step_done_o     (top_step_done),
        .swap_done_o     (top_swap_done),
        .swap_accepted_o (top_swap_acc),
        .rd_data_o       (top_rd_data)
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

    // --- Cycle-Accurate Core Datapath DFF Bit-Toggle Monitor ---
    longint unsigned core_toggle_acc_q;
    logic            toggle_meas_en;
    logic [4095:0]   core_dff_snap_c;
    logic [4095:0]   core_dff_prev_q;

    assign core_dff_snap_c = u_core_dut.sim_dff_snap_w;

    function automatic int popcount4096(input logic [4095:0] diff);
        int cnt;
        begin
            cnt = 0;
            for (int w = 0; w < 64; w++) begin
                cnt += $countones(diff[w*64 +: 64]);
            end
            popcount4096 = cnt;
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_dff_prev_q   <= 4096'd0;
            core_toggle_acc_q <= 64'd0;
        end else begin
            core_dff_prev_q <= core_dff_snap_c;
            if (toggle_meas_en) begin
                core_toggle_acc_q <= core_toggle_acc_q + 64'(popcount4096(core_dff_snap_c ^ core_dff_prev_q));
            end else begin
                core_toggle_acc_q <= 64'd0;
            end
        end
    end

    function automatic int popcount64(input logic [63:0] v);
        begin
            popcount64 = $countones(v);
        end
    endfunction

    function automatic int popcount65(input logic [64:0] v);
        begin
            popcount65 = $countones(v);
        end
    endfunction

    function automatic int popcount48(input logic [47:0] v);
        begin
            popcount48 = $countones(v);
        end
    endfunction

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

    function automatic [63:0] lattice16_val(input int idx);
        begin
            case (idx & 15)
                0:  lattice16_val = 64'h0000_0000_0000_0000; // 0
                1:  lattice16_val = 64'h0000_0000_0000_0001; // +1
                2:  lattice16_val = 64'hFFFF_FFFF_FFFF_FFFF; // -1
                3:  lattice16_val = 64'h0000_0000_0000_0004; // +4 (FLOW_CONVERGE_EPS_Q16)
                4:  lattice16_val = 64'hFFFF_FFFF_FFFF_FFFC; // -4
                5:  lattice16_val = 64'h0000_0000_0000_0064; // +100
                6:  lattice16_val = 64'hFFFF_FFFF_FFFF_FF9C; // -100
                7:  lattice16_val = 64'h0000_0000_0001_0000; // +1.0 Q16
                8:  lattice16_val = 64'hFFFF_FFFF_FFFF_0000; // -1.0 Q16
                9:  lattice16_val = 64'h0000_0000_7FFF_FFFF; // +FLOW_MAX_Q16
                10: lattice16_val = 64'hFFFF_FFFF_8000_0001; // -FLOW_MAX_Q16
                11: lattice16_val = 64'hFFFF_FFFF_8000_0000; // INT32_MIN
                12: lattice16_val = 64'h0000_4000_0000_0000; // +2^46
                13: lattice16_val = 64'hFFFF_C000_0000_0000; // -2^46
                14: lattice16_val = 64'h7FFF_FFFF_FFFF_FFFF; // INT64_MAX
                default: lattice16_val = 64'h8000_0000_0000_0000; // INT64_MIN
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
            rst_n          = 1'b0;
            toggle_meas_en = 1'b0;
            core_wr_valid  = 1'b0;
            core_wr_kind   = 2'd0;
            core_wr_idx    = 3'd0;
            core_wr_data   = 64'd0;
            core_rd_idx    = 3'd0;

            top_cmd_valid  = 1'b0;
            top_cmd_op     = CMD_NOP;
            top_cmd_word   = 3'd0;
            top_cmd_aux    = 16'd0;
            top_cmd_data   = 64'd0;

            bi_valid       = 1'b0;
            bi_data        = 64'd0;
            tm_clr         = 1'b0;
            tm_valid       = 1'b0;
            tm_nz          = 1'b0;
            tm_neg         = 1'b0;
            tm_act         = 32'sd0;

            repeat (3) @(posedge clk);
            #1;
            rst_n = 1'b1;
            @(posedge clk);
            #1;
        end
    endtask

    // -------------------------------------------------------------------------
    // 64-Bit Streaming Bus Tasks for `ele_flow_core` (141 pins)
    // -------------------------------------------------------------------------
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
            if (&{1'b0, cap_in[63:16]}) begin end
            guard = 0;
            while (!core_ready) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: core_ready timeout");
            end

            // Stream 8x64b `state_w` words (`wr_kind = 0`)
            for (int w = 0; w < 8; w++) begin
                core_wr_valid = 1'b1;
                core_wr_kind  = 2'd0;
                core_wr_idx   = 3'(w);
                core_wr_data  = st_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end

            // Stream 6x64b `bound_w[0..5]` words (`wr_kind = 1`)
            for (int w = 0; w < 6; w++) begin
                core_wr_valid = 1'b1;
                core_wr_kind  = 2'd1;
                core_wr_idx   = 3'(w);
                core_wr_data  = bd_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end

            // Write `{mode_req, cap_mask[15:0]}` (`wr_kind = 2, wr_idx = 0`)
            core_wr_valid = 1'b1;
            core_wr_kind  = 2'd2;
            core_wr_idx   = 3'd0;
            core_wr_data  = {32'd0, mreq_in, cap_in[15:0]};
            @(posedge clk);
            #1;

            // Bracket datapath switching telemetry across ONLY the core step execution
            toggle_meas_en = 1'b1;
            core_wr_valid  = 1'b1;
            core_wr_kind   = 2'd2;
            core_wr_idx    = 3'd1;
            core_wr_data   = u_in;
            @(posedge clk);
            #1;
            core_wr_valid  = 1'b0;
            cycles_taken   = 0;

            while (!core_done) begin
                @(posedge clk);
                #1;
                cycles_taken++;
                if (cycles_taken > 512) $fatal(1, "FAIL: core_done timeout");
            end

            // Capture final commit -> ST_IDLE transition in `core_toggle_acc_q`
            @(posedge clk);
            #1;
            dff_toggles    = core_toggle_acc_q;
            toggle_meas_en = 1'b0;

            // Read back 8x64b output manifold cell
            for (int w = 0; w < 8; w++) begin
                core_rd_idx = 3'(w);
                #1;
                st_out[w*64 +: 64] = core_rd_data;
            end

            @(posedge clk);
            #1;
        end
    endtask

    // -------------------------------------------------------------------------
    // 64-Bit Streaming Bus Tasks for `ele_flow_atlas_top` (157 pins + SRAM)
    // -------------------------------------------------------------------------
    task automatic top_wait_ready();
        int guard;
        begin
            guard = 0;
            while (!top_ready) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: top_ready timeout");
            end
        end
    endtask

    task automatic top_load_state_512(input logic [511:0] st_in, input logic [63:0] cap_in);
        begin
            top_wait_ready();
            top_cmd_valid = 1'b1;
            top_cmd_op    = CMD_SET_CAP;
            top_cmd_word  = 3'd0;
            top_cmd_aux   = 16'd0;
            top_cmd_data  = cap_in;
            @(posedge clk);
            #1;
            for (int w = 0; w < 8; w++) begin
                top_cmd_valid = 1'b1;
                top_cmd_op    = CMD_LOAD_STATE;
                top_cmd_word  = 3'(w);
                top_cmd_aux   = 16'd0;
                top_cmd_data  = st_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;
        end
    endtask

    task automatic top_write_chart_512(input logic [7:0] slot, input logic [511:0] bd_in);
        begin
            top_wait_ready();
            for (int w = 0; w < 8; w++) begin
                top_cmd_valid = 1'b1;
                top_cmd_op    = CMD_WRITE_CHART;
                top_cmd_word  = 3'(w);
                top_cmd_aux   = {8'd0, slot};
                top_cmd_data  = bd_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;
        end
    endtask

    task automatic top_stage_cand_512(input logic [511:0] cand_in);
        begin
            for (int w = 0; w < 6; w++) begin
                top_cmd_valid = 1'b1;
                top_cmd_op    = CMD_STAGE_CAND;
                top_cmd_word  = 3'(w);
                top_cmd_aux   = 16'd0;
                top_cmd_data  = cand_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;
        end
    endtask

    task automatic top_read_state_512(output logic [511:0] st_out);
        begin
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;
            for (int w = 0; w < 8; w++) begin
                top_cmd_word = 3'(w);
                #1;
                st_out[w*64 +: 64] = top_rd_data;
            end
        end
    endtask

    task automatic top_try_swap(
        input  logic [7:0]   slot,
        input  logic [31:0]  epoch,
        input  logic [511:0] cand_in,
        output logic         accepted,
        output int           swap_cycles
    );
        begin
            top_wait_ready();
            top_stage_cand_512(cand_in);
            top_cmd_valid = 1'b1;
            top_cmd_op    = CMD_SWAP;
            top_cmd_word  = 3'd0;
            top_cmd_aux   = {8'd0, slot};
            top_cmd_data  = {32'd0, epoch};
            @(posedge clk);
            #1;
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;
            swap_cycles   = 0;

            while (!top_swap_done) begin
                @(posedge clk);
                #1;
                swap_cycles++;
                if (swap_cycles > 32) $fatal(1, "FAIL: top_swap_done timeout");
            end
            accepted = top_swap_acc;
        end
    endtask

    task automatic exec_top_step(
        input  logic [63:0]  u_in,
        input  logic [15:0]  mreq_in,
        output logic [511:0] st_out
    );
        int guard;
        begin
            top_wait_ready();
            top_cmd_valid = 1'b1;
            top_cmd_op    = CMD_STEP;
            top_cmd_word  = 3'd0;
            top_cmd_aux   = mreq_in;
            top_cmd_data  = u_in;
            @(posedge clk);
            #1;
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;

            guard = 0;
            while (!top_step_done) begin
                @(posedge clk);
                #1;
                guard++;
                if (guard > 512) $fatal(1, "FAIL: top_step_done timeout");
            end
            top_read_state_512(st_out);
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
                $display("FAIL [%s #%0d]: 512-bit SV != FLOW C99!", tag, vec_id);
                for (int w = 0; w < 8; w++) begin
                    if (rtl_out[w*64 +: 64] !== c_packed[w*64 +: 64]) begin
                        $display("  lane w%0d: sv=%016x != c=%016x",
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
                    $display("FAIL [%s #%0d]: SV != flow_step!", tag, vec_id);
                    $fatal(1);
                end
            end
        end
    endtask

    // =========================================================================
    // Probe 1: Singular & LFSR 512-Bit `0-ULP` Co-Verification (Full on ALL Profiles)
    // =========================================================================
    task automatic run_probe1_differential();
`ifdef RTL_MUTANT_FAST
        localparam int N_IB   = 4;
        localparam int N_FUZZ = 256;
`else
        localparam int N_IB   = 16;
        localparam int N_FUZZ = 16384;
`endif
        logic [63:0]  rng;
        logic [511:0] st_v, bd_v;
        logic [63:0]  cap_v, u_v;
        logic [15:0]  mreq_v;
        int           vec_cnt;
        begin
            vec_cnt = 0;
            rng     = 64'hD1B5_4A32_D192_ED03;

            // 1A. Structured Hostile Singular Corners (4,096 canonical; 1,024 smoke)
            for (int ix = 0; ix < 16; ix++) begin
                for (int ib1 = 0; ib1 < N_IB; ib1++) begin
                    for (int ib2 = 0; ib2 < N_IB; ib2++) begin
                        int h_b1, h_b2;
                        h_b1 = (N_IB == 16) ? ib1 : ((ib1 << 1) ^ (ix & 1));
                        h_b2 = (N_IB == 16) ? ib2 : ((ib2 << 1) ^ ((ix >> 1) & 1));
                        rng  = next_lfsr(rng);
                        st_v = pack_cell(
                            hostile_val((ix + h_b1) & 15),
                            hostile_val((h_b1 + h_b2) & 15),
                            hostile_val(h_b2),
                            hostile_val(ix),
                            rng,
                            {32'hDEAD_BEEF, 16'h0003, rng[15:0]},
                            {rng[31:0], 32'hFFFF_FFFF},
                            rng
                        );
                        bd_v = pack_cell(
                            hostile_val((ix ^ h_b2) & 15),
                            (vec_cnt % 4 == 0) ? 64'd0 : hostile_val(h_b1),
                            (vec_cnt % 4 == 0) ? 64'd0 : hostile_val(h_b2),
                            {hostile_val((ix + 3) & 15)[31:0], hostile_val((h_b1 + 7) & 15)[31:0]},
                            {hostile_val((h_b2 + 5) & 15)[31:0], hostile_val((ix + 9) & 15)[31:0]},
                            {32'hCAFE_BABE, rng[31:16], rng[15:0]},
                            rng,
                            rng
                        );
                        cap_v  = (vec_cnt % 8 == 0) ? 64'hFFFF : {56'd0, (8'd1 << (vec_cnt & 3)) - 8'd1};
                        u_v    = (vec_cnt % 8 == 0) ? 64'd0 : hostile_val((ix + h_b1 + h_b2) & 15);
                        mreq_v = (vec_cnt % 8 == 0) ? st_v[335:320] : rng[47:32];

                        verify_core_against_c("HOSTILE", vec_cnt, st_v, bd_v, cap_v, u_v, mreq_v);
                        vec_cnt++;
                    end
                end
            end

            // 1B. Full-Width LFSR Fuzz Vectors (operator, linear, tunnel, autonomous)
            for (int f = 0; f < N_FUZZ; f++) begin
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
    // Probe 2: Odd-Mirror Gauge Parity & Autonomous 2^-49 Sub-LSB Phase Lock
    // =========================================================================
    task automatic run_probe2_symmetry_and_fiber();
`ifdef RTL_MUTANT_FAST
        localparam int N_MIRROR = 64;
        localparam int N_FIBER  = 128;
        localparam int HALF_PER = 64;
`else
        localparam int N_MIRROR = 1024;
        localparam int N_FIBER  = 2048;
        localparam int HALF_PER = 256;
`endif
        logic [63:0]     rng;
        logic [511:0]    st_p, bd_p, out_p;
        logic [511:0]    st_n, bd_n, out_n;
        logic [511:0]    top_state_cur;
        logic [63:0]     x_v, b0_v, b1_v, b2_v, u_v;
        int              cyc_p, cyc_n;
        longint unsigned tog_p, tog_n;
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        logic [511:0]    c_packed;
        begin
            rng = 64'h9E37_79B9_7F4A_7C15;
            for (int k = 0; k < N_MIRROR; k++) begin
                rng  = next_lfsr(rng); x_v  = {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); b0_v = {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); b1_v = {{35{rng[28]}}, rng[28:0]} | 64'd1;
                rng  = next_lfsr(rng); b2_v = (k % 3 == 0) ? 64'd0 : {{35{rng[28]}}, rng[28:0]};
                rng  = next_lfsr(rng); u_v  = {{35{rng[28]}}, rng[28:0]};

                st_p = pack_cell(64'd0, 64'd0, 64'd0, x_v, 64'd0, 64'd0, 64'd0, 64'd0);
                bd_p = pack_cell(b0_v, b1_v, b2_v, {32'h8000_0001, 32'h7FFF_FFFF},
                                 {32'h0010_0000, 32'h0001_4000}, 64'd0, 64'd0, 64'd0);

                st_n = pack_cell(64'd0, 64'd0, 64'd0, 64'd0 - x_v, 64'd0, 64'd0, 64'd0, 64'd0);
                bd_n = pack_cell(64'd0 - b0_v, b1_v, 64'd0 - b2_v, {32'h8000_0001, 32'h7FFF_FFFF},
                                 {32'h0010_0000, 32'h0001_4000}, 64'd0, 64'd0, 64'd0);

                exec_core_step(st_p, bd_p, 64'hFFFF, u_v,         16'd0, out_p, cyc_p, tog_p);
                exec_core_step(st_n, bd_n, 64'hFFFF, 64'd0 - u_v, 16'd0, out_n, cyc_n, tog_n);

                if ((cyc_p != cyc_n)                                  ||
                    (tog_p == 64'hFFFF_FFFF_FFFF_FFFF)                ||
                    (tog_n == 64'hFFFF_FFFF_FFFF_FFFF)                ||
                    (out_n[255:192] !== (64'd0 - out_p[255:192]))     ||
                    (out_n[63:0]    !== (64'd0 - out_p[63:0]))        ||
                    (out_n[127:64]  !== out_p[127:64])                ||
                    (out_n[191:128] !== (64'd0 - out_p[191:128]))     ||
                    (out_n[319:256] !== out_p[319:256])               ||
                    (out_n[511:320] !== out_p[511:320])) begin
                    $display("FAIL [MIRROR #%0d]: gauge/mirror parity broken!", k);
                    $fatal(1);
                end
            end

            // Autonomous 2-Jet Sub-LSB Fiber Conservation on `ele_flow_atlas_top` (`ele_sram_1rw64`)
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
                {32'h0000_0111, 32'h0001_0000}, // dt_max = 273 ticks, u_gain = 1.0
                64'd0,
                {32'd1, 32'd0},
                64'd0
            );

            top_load_state_512(st_p, 64'd1);
            for (int s = 0; s < 2; s++) begin
                top_write_chart_512(8'(s), bd_p);
            end

            for (int i = 0; i < 8; i++) c_st[i] = st_p[i*64 +: 64];
            for (int s = 0; s < 16; s++) begin
                for (int i = 0; i < 8; i++) c_arena[s*8 + i] = bd_p[i*64 +: 64];
            end

            for (int gen = 0; gen < N_FIBER; gen++) begin
                u_v = ((gen / HALF_PER) % 2 == 0) ? 64'sd131072 : -64'sd131072;

                exec_top_step(u_v, 16'd0, top_state_cur);

                flow_step_atlas(c_out, c_st, c_arena, 64'd1, longint'(u_v), 16'd0);
                for (int i = 0; i < 8; i++) c_st[i] = c_out[i];
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);

                if (top_state_cur !== c_packed) begin
                    $display("FAIL [FIBER gen=%0d]: autonomous 2^-49 sub-LSB drift!", gen);
                    $fatal(1);
                end
            end

            $display("  [PASS] Probe 2 (%0d Mirror Pairs & %0d-Gen Autonomous 2^-49 Fiber): 0 asymmetries, 0-ULP drift",
                     N_MIRROR, N_FIBER);
        end
    endtask

    // =========================================================================
    // Probe 3: Multi-Chart `ele_sram_1rw64` Atlas + Release-Ordered `@swap` + In-Flight Immunity
    // =========================================================================
    task automatic run_probe3_atlas_and_sram_swap();
        logic [511:0] chart0, chart1, chart2, chart3, chart_alias, init_st, alias_st, top_state_cur;
        logic [511:0] cand_ok, cand_bad_ep, cand_bad_lo, cand_bad_hi;
        logic [511:0] cand_bad_dt0, cand_bad_dt_min, cand_bad_hyst;
        logic [511:0] cand_cap15_ok, cand_cap16_lo, cand_cap16_hi, cand_poison;
        logic         sw_acc;
        int           sw_cyc;
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        logic [511:0]    c_packed;
        begin
            // -----------------------------------------------------------------
            // 3A. Hardware Slot-Aliasing Clamp (`cap_mask = 64'hFF -> 15`) &
            //     `epoch == 0` when `cur_ep == 0` Rejection on Uninitialized SRAM
            // -----------------------------------------------------------------
            alias_st = pack_cell(64'd0, 64'd0, 64'd0, 64'sd655360, 64'd0,
                                 {32'd0, 16'h0022, 16'h0001}, 64'd0, 64'd0);
            top_load_state_512(alias_st, 64'h0000_0000_0000_00FF);

            cand_cap15_ok = pack_cell(64'sd0, 64'sd65536, 64'sd0,
                                      {-32'sd2000000, 32'sd1966080}, {32'sd1638400, 32'sd65536},
                                      {32'd0, 8'd15, 8'd15, 16'h0001}, {32'd1, 32'd0}, 64'd0);
            cand_cap16_lo = pack_cell(64'sd0, 64'sd65536, 64'sd0,
                                      {-32'sd2000000, 32'sd1966080}, {32'sd1638400, 32'sd65536},
                                      {32'd0, 8'd16, 8'd0, 16'h0001}, {32'd1, 32'd0}, 64'd0);
            cand_cap16_hi = pack_cell(64'sd0, 64'sd65536, 64'sd0,
                                      {-32'sd2000000, 32'sd1966080}, {32'sd1638400, 32'sd65536},
                                      {32'd0, 8'd0, 8'd16, 16'h0001}, {32'd1, 32'd0}, 64'd0);

            // Slot 0 has `slot_valid_q[0] == 0` -> `cur_ep == 0`: `epoch == 0` MUST be rejected in 1 firewall cycle!
            top_try_swap(8'd0, 32'd0, cand_cap15_ok, sw_acc, sw_cyc);
            if (sw_acc || (sw_cyc != 1)) begin
                $fatal(1, "FAIL: epoch==0 when cur_ep==0 was not rejected in 1 firewall cycle (cyc=%0d)!", sw_cyc);
            end

            // With `cap_mask=0xFF` clamped to 15, `next_lo=16` and `next_hi=16` MUST be rejected in 1 firewall cycle!
            for (int a = 0; a < 2; a++) begin
                top_try_swap(8'd0, 32'd1, (a == 0) ? cand_cap16_lo : cand_cap16_hi, sw_acc, sw_cyc);
                if (sw_acc || (sw_cyc != 1)) begin
                    $fatal(1, "FAIL: out-of-capacity @swap #%0d (cap_mask=0xFF->15) was not rejected!", a);
                end
            end

            // Valid @swap with `next_lo=15, next_hi=15` at `epoch=1` MUST commit in 8 deterministic SRAM beats (`w6` last)!
            top_try_swap(8'd0, 32'd1, cand_cap15_ok, sw_acc, sw_cyc);
            if (!sw_acc || (sw_cyc != 8)) begin
                $fatal(1, "FAIL: valid @swap at hardware capacity boundary 15 failed (acc=%0b, cyc=%0d)!", sw_acc, sw_cyc);
            end

            // Load slots 0..15 with `next_hi = 8'h3F (63)` and verify core step masks `chart_next` to 15 (0-ULP vs C)
            chart_alias = pack_cell(64'sd0, 64'sd65536, 64'sd0,
                                    {-32'sd2000000, 32'sd983040}, {32'sd1638400, 32'sd65536},
                                    {32'd0, 8'd0, 8'h3F, 16'h0001}, {32'd1, 32'd0}, 64'd0);
            for (int s = 0; s < 16; s++) begin
                top_write_chart_512(8'(s), chart_alias);
                for (int i = 0; i < 8; i++) c_arena[s*8 + i] = chart_alias[i*64 +: 64];
            end

            for (int i = 0; i < 8; i++) c_st[i] = alias_st[i*64 +: 64];
            exec_top_step(64'sd3276800, 16'h00FF, top_state_cur);
            flow_step_atlas(c_out, c_st, c_arena, 64'd15, 64'sd3276800, 16'h00FF);
            c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                 c_out[4], c_out[5], c_out[6], c_out[7]);
            if ((top_state_cur !== c_packed) || (top_state_cur[351:336] !== 16'd15)) begin
                $fatal(1, "FAIL: cap_mask=0xFF did not clamp step chart_next to 15!");
            end

            // -----------------------------------------------------------------
            // 3B. 4-Chart SiC/FLL Control Manifold + Adversarial `@swap` Suite
            // -----------------------------------------------------------------
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

            top_load_state_512(init_st, 64'd3);
            top_write_chart_512(8'd0, chart0);
            top_write_chart_512(8'd1, chart1);
            top_write_chart_512(8'd2, chart2);
            top_write_chart_512(8'd3, chart3);

            for (int i = 0; i < 8; i++) begin
                c_st[i]        = init_st[i*64 +: 64];
                c_arena[0*8+i] = chart0[i*64 +: 64];
                c_arena[1*8+i] = chart1[i*64 +: 64];
                c_arena[2*8+i] = chart2[i*64 +: 64];
                c_arena[3*8+i] = chart3[i*64 +: 64];
            end

            cand_ok         = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                                        {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd2, 32'd0}, 64'd0);
            cand_bad_ep     = cand_ok;
            cand_bad_lo     = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                                        {32'd0, 8'd4, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_bad_hi     = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd983040, 32'sd3604480}, {32'sd2293760, 32'sd65536},
                                        {32'd0, 8'd0, 8'd9, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_bad_dt0    = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd983040, 32'sd3604480}, {32'd0, 32'sd65536},
                                        {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_bad_dt_min = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd983040, 32'sd3604480}, {32'h8000_0000, 32'sd65536},
                                        {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_bad_hyst   = pack_cell(64'sd0, 64'sd81920, 64'sd5242,
                                        {32'sd3604480, 32'sd983040}, {32'sd2293760, 32'sd65536},
                                        {32'd0, 8'd0, 8'd2, 16'h0002}, {32'd5, 32'd0}, 64'd0);
            cand_poison     = pack_cell(-64'sd9999999, 64'sd131072, 64'sd32768,
                                        {-32'sd100000, 32'sd100000}, {32'sd65536, 32'sd65536},
                                        {32'd0, 8'd3, 8'd3, 16'hFFFF}, {32'd99, 32'd0}, 64'd0);

            // Valid release-ordered swap into slot 1 at epoch 2 (`w0..w5`, `w7=0`, `w6={2,0}` last)
            top_try_swap(8'd1, 32'd2, cand_ok, sw_acc, sw_cyc);
            if (!sw_acc || (sw_cyc != 8)) begin
                $fatal(1, "FAIL: valid @swap into slot 1 failed (acc=%0b, cyc=%0d)!", sw_acc, sw_cyc);
            end
            @(posedge clk);
            #1;
            if (top_swap_done || top_swap_acc) begin
                $fatal(1, "FAIL: swap_done/swap_acc did not clear after 1-cycle pulse!");
            end
            for (int i = 0; i < 6; i++) c_arena[1*8+i] = cand_ok[i*64 +: 64];
            c_arena[1*8+6] = {32'd2, 32'd0};
            c_arena[1*8+7] = 64'd0;

            // Reject all 7 adversarial `@swap` vectors in 1 firewall cycle (`0` SRAM writes)
            for (int r = 0; r < 7; r++) begin
                logic [31:0] ep_try;
                logic [511:0] cd_try;
                ep_try = (r == 0) ? 32'd2 : ((r == 1) ? 32'd1 : 32'd5);
                case (r)
                    0: cd_try = cand_bad_ep;
                    1: cd_try = cand_bad_ep;
                    2: cd_try = cand_bad_lo;
                    3: cd_try = cand_bad_hi;
                    4: cd_try = cand_bad_dt0;
                    5: cd_try = cand_bad_dt_min;
                    default: cd_try = cand_bad_hyst;
                endcase
                top_try_swap(8'd1, ep_try, cd_try, sw_acc, sw_cyc);
                if (sw_acc || (sw_cyc != 1)) begin
                    $fatal(1, "FAIL: invalid @swap #%0d was not rejected in 1 firewall cycle (cyc=%0d)!", r, sw_cyc);
                end
            end

            // -----------------------------------------------------------------
            // 3C. In-Flight Step Immunity + 16-Step Closed-Loop Surge & KanLock
            // -----------------------------------------------------------------
            top_read_state_512(top_state_cur);
            for (int s = 0; s < 16; s++) begin
                logic [63:0] u_stim;
                logic [15:0] m_stim;
                logic [15:0] prev_chart, prev_mode, cur_lock;
                u_stim     = (s < 4) ? (64'sd1638400 + longint'(s) * 64'sd1310720) :
                             (s < 10) ? 64'sd9830400 : 64'sd655360;
                m_stim     = (s % 2 == 0) ? 16'h0007 : 16'h0000;
                prev_chart = top_state_cur[351:336] & 16'd3;
                prev_mode  = top_state_cur[335:320];
                cur_lock   = c_arena[int'(prev_chart)*8 + 5][15:0];

                if (s < 2) begin
                    int guard;
                    top_wait_ready();
                    // Stage poison candidate into shadow buffer, then launch step and attempt illegal mid-step @swap
                    top_stage_cand_512(cand_poison);
                    top_cmd_valid = 1'b1;
                    top_cmd_op    = CMD_STEP;
                    top_cmd_word  = 3'd0;
                    top_cmd_aux   = m_stim;
                    top_cmd_data  = u_stim;
                    @(posedge clk);
                    #1;
                    top_cmd_valid = 1'b0;
                    top_cmd_op    = CMD_NOP;

                    if (top_ready || top_step_done) begin
                        $fatal(1, "FAIL: expected multi-cycle in-flight step on s=%0d!", s);
                    end

                    for (int p = 0; p < 3; p++) begin
                        top_cmd_valid = 1'b1;
                        top_cmd_op    = CMD_SWAP;
                        top_cmd_word  = 3'd0;
                        top_cmd_aux   = 16'd1;
                        top_cmd_data  = {32'd0, 32'd99};
                        @(posedge clk);
                        #1;
                        if (top_swap_done || top_swap_acc) begin
                            $fatal(1, "FAIL: in-flight @swap was not blocked while top_ready==0!");
                        end
                    end
                    top_cmd_valid = 1'b0;
                    top_cmd_op    = CMD_NOP;

                    guard = 0;
                    while (!top_step_done) begin
                        if (top_swap_done || top_swap_acc) begin
                            $fatal(1, "FAIL: delayed @swap fired mid-step!");
                        end
                        @(posedge clk);
                        #1;
                        guard++;
                        if (guard > 512) $fatal(1, "FAIL: top_step_done timeout after in-flight swap");
                    end
                    top_read_state_512(top_state_cur);
                end else begin
                    exec_top_step(u_stim, m_stim, top_state_cur);
                end

                flow_step_atlas(c_out, c_st, c_arena, 64'd3, longint'(u_stim), m_stim);
                for (int i = 0; i < 8; i++) c_st[i] = c_out[i];
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);

                if (top_state_cur !== c_packed) begin
                    $fatal(1, "FAIL [ATLAS step=%0d]: 512-bit SV != flow_step_atlas!", s);
                end
                if ((top_state_cur[335:320] & cur_lock) !== (prev_mode & cur_lock)) begin
                    $fatal(1, "FAIL [KANLOCK step=%0d]: locked mode bits mutated!", s);
                end
            end

            $display("  [PASS] Probe 3 (16-Slot SRAM Clamp, 10 Adversarial @swap Firewall Checks, In-Flight Immunity & 4-Chart Surge): 0-ULP C99 lockstep");
        end
    endtask

    // =========================================================================
    // Probe 4: Pure-SV CMOS Switching-Activity & Bipolar PHY Telemetry
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
        int                 neg_act_cnt, pos_act_cnt, zero_act_cnt;
        begin
            // 4A. `ele_flow_core` Latency & Datapath Switching-Activity across Regimes
            bd_quad = pack_cell(
                -64'sd1638400,        // b0 = -25.0
                64'sd65536,           // b1 = 1.0
                64'sd6554,            // b2 = ~0.1
                {-32'sd3276800, 32'sd3276800},
                {32'sd983040, 32'sd65536},
                64'd0, 64'd0, 64'd0
            );
            bd_lin = pack_cell(
                -64'sd1638400,        // b0 = -25.0
                64'sd65536,           // b1 = 1.0
                64'sd0,               // b2 = 0.0 (linear!)
                {-32'sd3276800, 32'sd3276800},
                {32'sd1966080, 32'sd65536},
                64'd0, 64'd0, 64'd0
            );

            st_q_cur = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648, 64'd0, 64'd0, 64'd0, 64'd0);
            exec_core_step(st_q_cur, bd_quad, 64'hFFFF, 64'sd393216, 16'd0, st_next, cyc_quad, tog_quad);
            exec_core_step(st_q_cur, bd_lin,  64'hFFFF, 64'sd0,      16'd0, st_next, cyc_lin,  tog_lin);

            st_q_cur = st_next;
            for (int warm = 0; warm < 4; warm++) begin
                exec_core_step(st_q_cur, bd_lin, 64'hFFFF, 64'sd0, 16'd0, st_q_cur, cyc_idle, tog_idle);
            end

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

            if ((cyc_idle >= cyc_lin) || (cyc_lin >= cyc_quad)) begin
                $fatal(1, "FAIL [LATENCY]: tickless FLOW convergence did not cut step latency (quad=%0d, lin=%0d, idle=%0d)!",
                       cyc_quad, cyc_lin, cyc_idle);
            end
            if ((tog_idle >= tog_lin) || (tog_lin >= tog_quad) ||
                (idle_tog_sum * 10 >= quad_tog_sum)) begin
                $fatal(1, "FAIL [ENERGY]: tickless FLOW convergence did not yield expected toggle cut (quad=%0d, lin=%0d, idle=%0d)!",
                       tog_quad, tog_lin, tog_idle);
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

            // 4C-1. `ele_ternary_mac_dualrail` Exhaustive Bipolar & Extreme Corner Matrix (64 cycles)
            tm_clr   = 1'b1;
            tm_valid = 1'b0;
            @(posedge clk);
            #1;
            tm_clr   = 1'b0;
            ref_acc  = 48'sd0;
            prev_pos = 48'd0;
            prev_neg = 48'd0;

            for (int c_act = 0; c_act < 8; c_act++) begin
                for (int c_ctl = 0; c_ctl < 8; c_ctl++) begin
                    logic exp_skip;
                    logic signed [47:0] act_sext48;
                    tm_valid = c_ctl[2];
                    tm_nz    = c_ctl[1];
                    tm_neg   = c_ctl[0];
                    case (c_act)
                        0: tm_act = 32'sd0;
                        1: tm_act = 32'sd1;
                        2: tm_act = -32'sd1;
                        3: tm_act = 32'sh7FFF_FFFF;
                        4: tm_act = -32'sh7FFF_FFFF;
                        5: tm_act = 32'sh8000_0000;
                        6: tm_act = 32'sh0001_0000;
                        default: tm_act = -32'sh0001_0000;
                    endcase
                    #1;
                    exp_skip = tm_valid & ((~tm_nz) | (tm_act == 32'sd0));
                    if (tm_zero_skip !== exp_skip) begin
                        $fatal(1, "FAIL [PHY-CORNER]: ternary zero_skip mismatch!");
                    end
                    @(posedge clk);
                    #1;
                    if (!tm_valid || exp_skip) begin
                        if ((tm_pos_rail !== prev_pos) || (tm_neg_rail !== prev_neg)) begin
                            $fatal(1, "FAIL [PHY-CORNER]: rail toggled during idle/zero_skip!");
                        end
                    end else begin
                        if ((tm_pos_rail < prev_pos) || (tm_neg_rail < prev_neg) ||
                            ((tm_pos_rail != prev_pos) && (tm_neg_rail != prev_neg))) begin
                            $fatal(1, "FAIL [PHY-CORNER]: non-monotonic or dual-rail simultaneous toggle!");
                        end
                        act_sext48 = $signed({{16{tm_act[31]}}, tm_act});
                        if (tm_neg) ref_acc = ref_acc - act_sext48;
                        else        ref_acc = ref_acc + act_sext48;
                    end
                    if (tm_acc !== ref_acc) begin
                        $fatal(1, "FAIL [PHY-CORNER]: ternary acc=%0d != ref=%0d (act=%08x, neg=%0b)!",
                               tm_acc, ref_acc, tm_act, tm_neg);
                    end
                    prev_pos = tm_pos_rail;
                    prev_neg = tm_neg_rail;
                end
            end

            // 4C-2. `ele_ternary_mac_dualrail` Full Bipolar Signed Switching-Energy Sweep (`1,024` steps)
            tm_clr   = 1'b1;
            tm_valid = 1'b0;
            @(posedge clk);
            #1;
            if ((tm_pos_rail !== 48'd0) || (tm_neg_rail !== 48'd0) || (tm_acc !== 48'sd0)) begin
                $fatal(1, "FAIL [PHY]: tm_clr did not clear dual rails to 0!");
            end
            tm_clr           = 1'b0;
            ref_acc          = 48'sd0;
            ref_acc_prev     = 48'sd0;
            prev_pos         = 48'd0;
            prev_neg         = 48'd0;
            twos_mac_toggles = 64'd0;
            dual_mac_toggles = 64'd0;
            neg_act_cnt      = 0;
            pos_act_cnt      = 0;
            zero_act_cnt     = 0;

            for (int k = 0; k < 1024; k++) begin
                logic exp_skip;
                logic signed [47:0] act_sext48;
                lfsr     = next_lfsr(lfsr);
                tm_valid = 1'b1;

                case (k)
                    64: begin tm_nz = 1'b1; tm_neg = 1'b0; tm_act = 32'sh7FFF_FFFF;  end
                    65: begin tm_nz = 1'b1; tm_neg = 1'b1; tm_act = 32'sh7FFF_FFFF;  end
                    66: begin tm_nz = 1'b1; tm_neg = 1'b0; tm_act = 32'sh8000_0000;  end
                    67: begin tm_nz = 1'b1; tm_neg = 1'b1; tm_act = 32'sh8000_0000;  end
                    68: begin tm_nz = 1'b1; tm_neg = 1'b0; tm_act = -32'sh7FFF_FFFF; end
                    69: begin tm_nz = 1'b1; tm_neg = 1'b0; tm_act = 32'sh7FFF_FFFF;  end
                    default: begin
                        logic signed [31:0] mag12;
                        mag12  = $signed({20'd0, lfsr[11:0] | 12'd1});
                        tm_nz  = (k % 3 != 0);
                        tm_neg = (k % 2 == 1);
                        if (k % 11 == 0) begin
                            tm_act = 32'sd0;
                        end else if ((((k >> 1) ^ int'(lfsr[12])) & 1) != 0) begin
                            tm_act = -mag12;
                        end else begin
                            tm_act = mag12;
                        end
                    end
                endcase

                if (tm_act < 32'sd0)       neg_act_cnt++;
                else if (tm_act > 32'sd0)  pos_act_cnt++;
                else                       zero_act_cnt++;

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
                    act_sext48 = $signed({{16{tm_act[31]}}, tm_act});
                    if (tm_neg) ref_acc = ref_acc - act_sext48;
                    else        ref_acc = ref_acc + act_sext48;
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

            if ((neg_act_cnt == 0) || (pos_act_cnt == 0) || (zero_act_cnt == 0)) begin
                $fatal(1, "FAIL [PHY]: bipolar activation coverage gap!");
            end
            if ((inv_bus_toggles >= raw_bus_toggles) || (dual_mac_toggles >= twos_mac_toggles)) begin
                $fatal(1, "FAIL [ENERGY]: PHY charge-minimal modules did not cut switching toggles!");
            end

            $display("  [PASS] Probe 4 (CMOS Switching-Energy & Charge-Minimal PHY Telemetry [datapath_dff=%0d bits]):",
                     u_core_dut.CORE_DFF_BITS);
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
            $display("         • Dual-Rail Ternary MAC   : rail_dff_toggles=%0d vs 2s_comp_acc=%0d (-%0d.%0d%%) [pos=%0d, neg=%0d, zero=%0d]",
                     dual_mac_toggles, twos_mac_toggles,
                     ((twos_mac_toggles - dual_mac_toggles) * 1000 / twos_mac_toggles) / 10,
                     ((twos_mac_toggles - dual_mac_toggles) * 1000 / twos_mac_toggles) % 10,
                     pos_act_cnt, neg_act_cnt, zero_act_cnt);
        end
    endtask

    // =========================================================================
    // Helper: Core Step + Dual `flow_step_atlas` / `flow_step` 512-Bit Oracle
    // =========================================================================
    task automatic eval_and_verify_core(
        input  string        tag,
        input  int           vec_id,
        input  logic [511:0] st_in,
        input  logic [511:0] bd_in,
        input  logic [63:0]  cap_in,
        input  logic [63:0]  u_in,
        input  logic [15:0]  mreq_in,
        output logic [511:0] rtl_out
    );
        longint unsigned c_st[8];
        longint unsigned c_bd[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
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
                $fatal(1, "FAIL [%s #%0d]: arena index >= 16!", tag, vec_id);
            end
            c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                 c_out[4], c_out[5], c_out[6], c_out[7]);

            exec_core_step(st_in, bd_in, cap_in, u_in, mreq_in, rtl_out, cyc, tog);
            if ((cyc <= 0) || (tog == 64'hFFFF_FFFF_FFFF_FFFF)) begin
                $fatal(1, "FAIL [%s #%0d]: invalid cycle/toggle telemetry!", tag, vec_id);
            end
            if (rtl_out !== c_packed) begin
                $display("FAIL [%s #%0d]: 512-bit SV != FLOW C99!", tag, vec_id);
                for (int w = 0; w < 8; w++) begin
                    if (rtl_out[w*64 +: 64] !== c_packed[w*64 +: 64]) begin
                        $display("  lane w%0d: sv=%016x != c=%016x",
                                 w, rtl_out[w*64 +: 64], c_packed[w*64 +: 64]);
                    end
                end
                $fatal(1);
            end

            if ((cap_in == 64'hFFFF) && (u_in == 64'd0) && (mreq_in == st_in[335:320])) begin
                longint unsigned c_step_out[8];
                logic [511:0]    c_step_packed;
                flow_step(c_step_out, c_st, c_bd);
                c_step_packed = pack_cell(c_step_out[0], c_step_out[1], c_step_out[2], c_step_out[3],
                                          c_step_out[4], c_step_out[5], c_step_out[6], c_step_out[7]);
                if (rtl_out !== c_step_packed) begin
                    $fatal(1, "FAIL [%s #%0d]: SV != flow_step!", tag, vec_id);
                end
            end
        end
    endtask

    // =========================================================================
    // Probe 5: 3,688 Analytical Multipole 2^-120 / 2^-124 Cross-Limb Knife-Edges
    //          + 8 Exact Q49 Sub-LSB Convergence Cliffs (Full on BOTH Profiles)
    // =========================================================================
    task automatic pack_au_knife_state(
        input  longint signed g0_s,
        input  longint signed g1_s,
        input  longint signed r2_s,
        input  longint signed x_s,
        output logic [511:0]  st_out
    );
        logic signed [63:0] g0_v, g1_v;
        logic [63:0]        j0_in, j1_in, s7_v;
        begin
            g0_v  = g0_s;
            g1_v  = g1_s;
            j0_in = $unsigned(g0_v >>> 33);
            j1_in = $unsigned(g1_v >>> 16);
            s7_v  = {g1_v[15:0], 15'd0, g0_v[32:0]};
            st_out = pack_cell(
                j0_in,
                j1_in,
                $unsigned(64'(r2_s)),
                $unsigned(64'(x_s)),
                64'd0,
                64'h0000_0000_0002_BEEF,
                64'h0000_0007_0000_0000,
                s7_v
            );
        end
    endtask

    task automatic run_probe5_knife_edges();
`ifdef RTL_MUTANT_FAST
        localparam int K_DIV_STEP = 6;
`else
        localparam int K_DIV_STEP = 1;
`endif
        logic [511:0]    bd_au_max, bd_au_256, bd_sch, bd_inv, bd_conv_p, bd_conv_n;
        logic [511:0]    st_v, rtl_out;
        longint unsigned q_tbl[5];
        longint signed   j0_tbl[7];
        longint signed   j1_tbl[5];
        longint signed   sch_tbl[7];
        int              multipole_cnt, conv_cnt;
        begin
            multipole_cnt = 0;
            conv_cnt      = 0;

            bd_au_max = pack_cell(
                64'd0, 64'd0, 64'd0,
                {32'h8000_0001, 32'h7FFF_FFFF},
                {32'h7FFF_FFFF, 32'h0000_0000},
                64'd0, {32'd1, 32'd0}, 64'd0
            );
            bd_au_256 = pack_cell(
                64'd0, 64'd0, 64'd0,
                {32'h8000_0001, 32'h7FFF_FFFF},
                {32'h0100_0000, 32'h0000_0000},
                64'd0, {32'd1, 32'd0}, 64'd0
            );

            // --- Family A (944 vectors): 120-Bit `isqrt` + 33b Limb Carry/Borrow Knife-Edges (k=2..60) ---
            for (int k = 2; k <= 60; k++) begin
                for (int v = 0; v < 2; v++) begin
                    longint unsigned m_val, m4, m5;
                    longint signed   g1_pos, g0_pos;
                    m_val = (64'd1 << k);
                    if (v == 1) begin
                        m_val = m_val + (64'd1 << (k >> 1)) + 64'd1;
                    end
                    g1_pos = longint'(m_val) - 64'sd2;
                    m4     = m_val << 2;
                    m5     = m4 + m_val;

                    for (int d_idx = 0; d_idx < 4; d_idx++) begin
                        case (d_idx)
                            0: g0_pos = 64'sd5 - longint'(m4); // D = M^2 - 1 (rounds up to M)
                            1: g0_pos = 64'sd4 - longint'(m4); // D = M^2     (exact square M)
                            2: g0_pos = 64'sd4 - longint'(m5); // D = M^2 + M (isqrt=M   -> xp MUST == 3)
                            default: g0_pos = 64'sd3 - longint'(m5); // D = M^2+M+1 (isqrt=M+1 -> xp MUST == 2)
                        endcase

                        // Sign 0: (+G0_pos, +G1_pos, R2 = +1, x = 0)
                        pack_au_knife_state(g0_pos, g1_pos, 64'sd1, 64'sd0, st_v);
                        eval_and_verify_core("KNIFE-A+", multipole_cnt, st_v, bd_au_max,
                                             64'hFFFF, 64'd0, st_v[335:320], rtl_out);
                        if ((d_idx == 2) && ($signed(rtl_out[255:192]) !== 64'sd3)) begin
                            $fatal(1, "FAIL [KNIFE-A+ k=%0d v=%0d]: D=M^2+M expected xp=3, got %0d!",
                                   k, v, $signed(rtl_out[255:192]));
                        end
                        if ((d_idx == 3) && ($signed(rtl_out[255:192]) !== 64'sd2)) begin
                            $fatal(1, "FAIL [KNIFE-A+ k=%0d v=%0d]: D=M^2+M+1 expected xp=2, got %0d!",
                                   k, v, $signed(rtl_out[255:192]));
                        end
                        multipole_cnt++;

                        // Sign 1 (Gauge Twin): (-G0_pos, -G1_pos, R2 = -1, x = 0)
                        pack_au_knife_state(-g0_pos, -g1_pos, -64'sd1, 64'sd0, st_v);
                        eval_and_verify_core("KNIFE-A-", multipole_cnt, st_v, bd_au_max,
                                             64'hFFFF, 64'd0, st_v[335:320], rtl_out);
                        if ((d_idx == 2) && ($signed(rtl_out[255:192]) !== 64'sd3)) begin
                            $fatal(1, "FAIL [KNIFE-A- k=%0d v=%0d]: gauge twin D=M^2+M expected xp=3, got %0d!",
                                   k, v, $signed(rtl_out[255:192]));
                        end
                        if ((d_idx == 3) && ($signed(rtl_out[255:192]) !== 64'sd2)) begin
                            $fatal(1, "FAIL [KNIFE-A- k=%0d v=%0d]: gauge twin D=M^2+M+1 expected xp=2, got %0d!",
                                   k, v, $signed(rtl_out[255:192]));
                        end
                        multipole_cnt++;
                    end
                end
            end

            // --- Family B (2,700 vectors): 64-Stage `sym_div128_clamped` 0.5-ULP & Cap Cliffs (k=1..30) ---
            q_tbl[0] = 64'd0;
            q_tbl[1] = 64'd1;
            q_tbl[2] = 64'd37;
            q_tbl[3] = 64'h00FF_FFFF; // dt_max - 1
            q_tbl[4] = 64'h0100_0000; // dt_max exact saturation cliff

            for (int k = 1; k <= 30; k += K_DIV_STEP) begin
                for (int d_off = 0; d_off < 3; d_off++) begin
                    longint unsigned d_val;
                    if (k < 30) begin
                        d_val = (64'd1 << k) + 64'(d_off) - 64'd1;
                    end else begin
                        d_val = 64'h7FFF_FFFF - 64'(d_off);
                    end

                    for (int q_idx = 0; q_idx < 5; q_idx++) begin
                        for (int r_idx = 0; r_idx < 3; r_idx++) begin
                            longint unsigned r_val, g1_mag;
                            logic signed [63:0] g1_neg, g1_pos_b;
                            if (r_idx == 0) begin
                                r_val = 64'd0;
                            end else if (r_idx == 1) begin
                                r_val = (d_val - 64'd1) >> 1;
                            end else begin
                                r_val = ((d_val - 64'd1) >> 1) + 64'd1;
                                if (r_val > (d_val - 64'd1)) r_val = d_val - 64'd1;
                            end
                            g1_mag   = q_tbl[q_idx] * d_val + r_val;
                            g1_neg   = -longint'(g1_mag);
                            g1_pos_b = longint'(g1_mag);

                            // Negative tangent G1 = -g1_mag with j0_in = 1<<58 (disc < 0 tunnel branch)
                            st_v = pack_cell(
                                64'h0400_0000_0000_0000,
                                $unsigned(g1_neg >>> 16),
                                d_val,
                                64'd0,
                                64'd0, 64'd0, 64'd0,
                                {g1_neg[15:0], 48'd0}
                            );
                            eval_and_verify_core("KNIFE-B-", multipole_cnt, st_v, bd_au_256,
                                                 64'hFFFF, 64'd0, 16'd0, rtl_out);
                            multipole_cnt++;

                            // Positive tangent G1 = +g1_mag
                            st_v = pack_cell(
                                64'h0400_0000_0000_0000,
                                $unsigned(g1_pos_b >>> 16),
                                d_val,
                                64'd0,
                                64'd0, 64'd0, 64'd0,
                                {g1_pos_b[15:0], 48'd0}
                            );
                            eval_and_verify_core("KNIFE-B+", multipole_cnt, st_v, bd_au_256,
                                                 64'hFFFF, 64'd0, 16'd0, rtl_out);
                            multipole_cnt++;
                        end
                    end
                end
            end

            // --- Family C (42 vectors): `sub_valid` Saturation Cliffs (35) + Schmitt/KanLock Cliffs (7) ---
            j0_tbl[0] = 64'sd0;
            j0_tbl[1] = 64'sh1000_0000_0000_0000;  // +FLOW_MAX_J0_Q16 (sub_valid == 1)
            j0_tbl[2] = 64'sh1000_0000_0000_0001;  // +FLOW_MAX_J0_Q16 + 1 (sub_valid == 0!)
            j0_tbl[3] = -64'sh1000_0000_0000_0000; // -FLOW_MAX_J0_Q16 (sub_valid == 1)
            j0_tbl[4] = -64'sh1000_0000_0000_0001; // -FLOW_MAX_J0_Q16 - 1 (sub_valid == 0!)
            j0_tbl[5] = 64'sh7FFF_FFFF_FFFF_FFFF;  // INT64_MAX
            j0_tbl[6] = -64'sh8000_0000_0000_0000; // INT64_MIN

            j1_tbl[0] = 64'sd0;
            j1_tbl[1] = 64'sh0000_2000_0000_0000;  // +FLOW_MAX_J1_Q16 (sub_valid == 1)
            j1_tbl[2] = 64'sh0000_2000_0000_0001;  // +FLOW_MAX_J1_Q16 + 1 (sub_valid == 0!)
            j1_tbl[3] = -64'sh0000_2000_0000_0000; // -FLOW_MAX_J1_Q16 (sub_valid == 1)
            j1_tbl[4] = -64'sh0000_2000_0000_0001; // -FLOW_MAX_J1_Q16 - 1 (sub_valid == 0!)

            for (int i0 = 0; i0 < 7; i0++) begin
                for (int i1 = 0; i1 < 5; i1++) begin
                    st_v = pack_cell(
                        $unsigned(64'(j0_tbl[i0])),
                        $unsigned(64'(j1_tbl[i1])),
                        64'd65536,
                        64'd0,
                        64'd0, 64'd0, 64'd0,
                        64'hFFFF_0001_FFFF_FFFF
                    );
                    eval_and_verify_core("KNIFE-C-SUB", multipole_cnt, st_v, bd_au_256,
                                         64'hFFFF, 64'd0, 16'd0, rtl_out);
                    multipole_cnt++;
                end
            end

            bd_sch = pack_cell(
                64'd0, 64'd0, 64'd0,
                {-32'sd100, 32'sd100},
                {32'sd65536, 32'sd0},
                {32'd0, 8'd1, 8'd2, 16'h5AA5},
                {32'd1, 32'd0}, 64'd0
            );
            sch_tbl[0] = -64'sd101;
            sch_tbl[1] = -64'sd100;
            sch_tbl[2] = -64'sd99;
            sch_tbl[3] = 64'sd0;
            sch_tbl[4] = 64'sd99;
            sch_tbl[5] = 64'sd100;
            sch_tbl[6] = 64'sd101;

            for (int s = 0; s < 7; s++) begin
                logic [15:0] exp_chart;
                st_v = pack_cell(64'd0, 64'd0, 64'd0, $unsigned(64'(sch_tbl[s])),
                                 64'd0, 64'd0, 64'd0, 64'd0);
                eval_and_verify_core("KNIFE-C-SCH", multipole_cnt, st_v, bd_sch,
                                     64'd3, 64'd0, 16'hF00F, rtl_out);
                exp_chart = (s == 0) ? 16'd1 : ((s == 6) ? 16'd2 : 16'd0);
                if ((rtl_out[351:336] !== exp_chart) ||
                    (rtl_out[335:320] !== (16'hF00F & (~16'h5AA5)))) begin
                    $fatal(1, "FAIL [KNIFE-C-SCH s=%0d]: Schmitt/KanLock cliff violated!", s);
                end
                multipole_cnt++;
            end

            // --- Family D (2 vectors): 125-Bit Extreme Discriminant (>= 2^124) & Inverted Schmitt Band ---
            st_v = pack_cell(
                $unsigned(-64'sh1000_0000_0000_0000), // j0_in = -(1LL << 60) => G0 = -2^93
                $unsigned(-64'sh0000_2000_0000_0000), // j1_in = -(1LL << 45) => G1 = -2^61
                64'h0000_0000_7FFF_FFFF,              // j2_in = +0x7FFFFFFF  => disc >= 2^124 + 2^122
                64'd0,
                64'd0, 64'd0, 64'd0, 64'd0
            );
            eval_and_verify_core("KNIFE-D-125B", multipole_cnt, st_v, bd_au_max,
                                 64'hFFFF, 64'd0, 16'd0, rtl_out);
            multipole_cnt++;

            bd_inv = pack_cell(
                64'd0, 64'd0, 64'd0,
                {32'sd100, -32'sd100}, // Inverted Schmitt: raw_exit = +100 > raw_enter = -100
                {32'sd65536, 32'sd0},
                {32'd0, 8'd1, 8'd2, 16'h0000},
                {32'd1, 32'd0}, 64'd0
            );
            st_v = pack_cell(64'd0, 64'd0, 64'd0, 64'sd50, 64'd0, 64'd0, 64'd0, 64'd0);
            eval_and_verify_core("KNIFE-D-INV", multipole_cnt, st_v, bd_inv,
                                 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if (rtl_out[351:336] !== 16'd1) begin
                $fatal(1, "FAIL [KNIFE-D-INV]: inverted Schmitt band glued to %0d instead of next_lo=1!",
                       rtl_out[351:336]);
            end
            multipole_cnt++;

`ifndef RTL_MUTANT_FAST
            if (multipole_cnt != 3688) begin
                $fatal(1, "FAIL [KNIFE-COUNT]: expected exactly 3688 multipole vectors, ran %0d!", multipole_cnt);
            end
`endif

            // --- Family E (8 vectors): Exact `2^-49` Q49 `FLOW_STATUS_CONVERGED` Sub-LSB Cliffs ---
            // E1: Autonomous j0=+4, sub=0 -> |G0_post| == (4<<33) -> out_j0==+4, converged==1
            st_v = pack_cell(64'sd4, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0);
            eval_and_verify_core("KNIFE-E1", 0, st_v, bd_au_256, 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== 64'sd4) || (rtl_out[415:384] !== 32'd1)) $fatal(1, "FAIL E1");
            conv_cnt++;

            // E2: Autonomous j0=+4, sub=1 -> |G0_post| == (4<<33)+1 -> out_j0==+4, converged==0!
            st_v = pack_cell(64'sd4, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd1);
            eval_and_verify_core("KNIFE-E2", 1, st_v, bd_au_256, 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== 64'sd4) || (rtl_out[415:384] !== 32'd0)) $fatal(1, "FAIL E2");
            conv_cnt++;

            // E3: Autonomous j0=-4, sub=0 -> |G0_post| == (4<<33) -> out_j0==-4, converged==1
            st_v = pack_cell(-64'sd4, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0);
            eval_and_verify_core("KNIFE-E3", 2, st_v, bd_au_256, 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== -64'sd4) || (rtl_out[415:384] !== 32'd1)) $fatal(1, "FAIL E3");
            conv_cnt++;

            // E4: Autonomous j0=-5, sub=0x1FFFFFFFF -> G0_post == -(4<<33)-1 -> converged==0!
            st_v = pack_cell(-64'sd5, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'h0000_0001_FFFF_FFFF);
            eval_and_verify_core("KNIFE-E4", 3, st_v, bd_au_256, 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if (rtl_out[415:384] !== 32'd0) $fatal(1, "FAIL E4");
            conv_cnt++;

            // E5 & E6: Driven Operator Barrier b0=+4, b1=0, b2=+1000, dt=65536, u_gain=+1, x=0
            bd_conv_p = pack_cell(64'sd4, 64'sd0, 64'sd1000,
                                  {32'h8000_0001, 32'h7FFF_FFFF}, {32'sd65536, 32'sd1},
                                  64'd0, {32'd1, 32'd0}, 64'd0);
            st_v = pack_cell(64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0, 64'd0);
            eval_and_verify_core("KNIFE-E5", 4, st_v, bd_conv_p, 64'd0, 64'sd1, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== 64'sd4) || (rtl_out[415:384] !== 32'd1)) $fatal(1, "FAIL E5");
            conv_cnt++;

            eval_and_verify_core("KNIFE-E6", 5, st_v, bd_conv_p, 64'd0, -64'sd1, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== 64'sd4) || (rtl_out[415:384] !== 32'd0)) $fatal(1, "FAIL E6");
            conv_cnt++;

            // E7 & E8: Mirror Negative Driven Barrier b0=-4, b1=0, b2=-1000, dt=65536, u_gain=+1, x=0
            bd_conv_n = pack_cell(-64'sd4, 64'sd0, -64'sd1000,
                                  {32'h8000_0001, 32'h7FFF_FFFF}, {32'sd65536, 32'sd1},
                                  64'd0, {32'd1, 32'd0}, 64'd0);
            eval_and_verify_core("KNIFE-E7", 6, st_v, bd_conv_n, 64'd0, -64'sd1, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== -64'sd4) || (rtl_out[415:384] !== 32'd1)) $fatal(1, "FAIL E7");
            conv_cnt++;

            eval_and_verify_core("KNIFE-E8", 7, st_v, bd_conv_n, 64'd0, 64'sd1, 16'd0, rtl_out);
            if (($signed(rtl_out[63:0]) !== -64'sd4) || (rtl_out[415:384] !== 32'd0)) $fatal(1, "FAIL E8");
            conv_cnt++;

            // E9 & E10: Exact Isometric 128-Bit Lyapunov Boundary (`|G0_cand| == |G0| == 2`, `delta_cand = +-1`)
            // G0 = -2 (`j0=-1, j0_sub=2^33-2`), G1 = +2 (`j1=0, j1_sub=2`), R2 = 0 -> step=+1, G0_cand=+2!
            pack_au_knife_state(-64'sd2, 64'sd2, 64'sd0, 64'sd0, st_v);
            eval_and_verify_core("KNIFE-E9", 8, st_v, bd_au_256, 64'hFFFF, 64'd0, st_v[335:320], rtl_out);
            if ($signed(rtl_out[255:192]) !== 64'sd1) $fatal(1, "FAIL E9 isometric Lyapunov +1");
            conv_cnt++;

            pack_au_knife_state(64'sd2, 64'sd2, 64'sd0, 64'sd0, st_v);
            eval_and_verify_core("KNIFE-E10", 9, st_v, bd_au_256, 64'hFFFF, 64'd0, st_v[335:320], rtl_out);
            if ($signed(rtl_out[255:192]) !== -64'sd1) $fatal(1, "FAIL E10 isometric Lyapunov -1");
            conv_cnt++;

            // E11: Exact `G1 == 0, R2 < 0, G0 > 0` Gauge-Invariant Citardauq Branch (`d_neg == 1` at `G1 == 0`)
            pack_au_knife_state(64'sd16, 64'sd0, -64'sd1, 64'sd0, st_v);
            eval_and_verify_core("KNIFE-E11", 10, st_v, bd_au_256, 64'hFFFF, 64'd0, st_v[335:320], rtl_out);
            if ($signed(rtl_out[255:192]) !== 64'sd4) $fatal(1, "FAIL E11 G1==0 R2<0 gauge sign");
            conv_cnt++;

            // E12: Exact 128-Bit Divider `pre_sat` Boundary (`anum128[127:64] == safe_den == 2`, `quot == 2^64` wrap trap!)
            st_v = pack_cell(
                $unsigned(-(64'sd1 <<< 32)), // j0_in = -2^32 -> G0 = -2^65 -> |div_num| = 2 * 2^64
                64'd0,                       // j1_in = 0
                64'd0,                       // R2 = 0 (linear: div_den = 2 * G1 = 2)
                64'd0,
                64'd0, 64'd0, 64'd0,
                {16'd1, 48'd0}               // j1_sub = 1 -> G1 = +1 -> safe_den = 2 == anum[127:64]!
            );
            eval_and_verify_core("KNIFE-E12", 11, st_v, bd_au_256, 64'hFFFF, 64'd0, 16'd0, rtl_out);
            if ($signed(rtl_out[255:192]) !== 64'sh0000_0000_0100_0000) $fatal(1, "FAIL E12 pre_sat wrap");
            if (&{1'b0, rtl_out}) begin end
            conv_cnt++;

            $display("  [PASS] Probe 5 (Analytical Multipole Knife-Edges): %0d multipole + %0d exact cliff vectors = %0d total (0-ULP)",
                     multipole_cnt, conv_cnt, multipole_cnt + conv_cnt);
        end
    endtask

    // =========================================================================
    // Probe 6: SEARU Zero-Memory 1-Bit `DualChaosState` + `Epiphany` Crucible
    // =========================================================================
    task automatic seed_searu_chart_slot(
        input  int           slot,
        inout  logic [63:0]  rng,
        input  int unsigned  ep_in,
        output logic [511:0] chart_out
    );
        logic signed [63:0] w0_s, w1_s, w2_s;
        logic [63:0]        w0, w1, w2, w3, w4, w5;
        begin
            rng  = next_lfsr(rng); w0_s = $signed(rng);
            rng  = next_lfsr(rng); w1_s = $signed(rng);
            rng  = next_lfsr(rng); w2_s = $signed(rng);

            case (slot & 3)
                0: begin // Autonomous Sub-LSB Fiber (b1=0, b2=0)
                    w0 = $unsigned(w0_s >>> 34);
                    w1 = 64'd0;
                    w2 = 64'd0;
                end
                1: begin // Linear Operator (b1 != 0, b2 = 0)
                    w0 = $unsigned(w0_s >>> 34);
                    w1 = $unsigned((w1_s >>> 36) | 64'sd1);
                    w2 = 64'd0;
                end
                2: begin // Quadratic Citardauq / Barrier (b2 != 0)
                    w0 = $unsigned(w0_s >>> 34);
                    w1 = $unsigned(w1_s >>> 35);
                    w2 = $unsigned((w2_s >>> 36) | 64'sd1);
                end
                default: begin // Full 64-Bit Unclamped Hostile Bound
                    w0 = $unsigned(w0_s);
                    w1 = $unsigned(w1_s);
                    w2 = $unsigned(w2_s);
                end
            endcase

            rng = next_lfsr(rng); w3 = rng;
            rng = next_lfsr(rng); w4 = rng;
            if ((slot & 3) != 3) begin
                logic [31:0] dt_pos;
                dt_pos = (w4[63:32] & 32'h7FFF_FFFF) | 32'h0010_0000;
                w4     = {dt_pos, w4[31:0]};
            end
            rng = next_lfsr(rng); w5 = rng;
            chart_out = pack_cell(w0, w1, w2, w3, w4, w5, {ep_in, 32'd0}, 64'd0);
        end
    endtask

    task automatic run_probe6_searu_epiphany_crucible();
`ifdef RTL_MUTANT_FAST
        localparam int N_CRUCIBLE = 256;
`else
        localparam int N_CRUCIBLE = 4096;
`endif
        logic [63:0]     rng, swar_mask, cap_raw, cap_eff, u_ext;
        logic [15:0]     mode_req;
        logic [511:0]    st_cur, rtl_out, chart_tmp, cand_cell;
        logic [63:0]     cap_tbl[8];
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        longint unsigned c_cand[8];
        logic [511:0]    c_packed;
        int unsigned     ep_seq;
        int              k_cool, epiphany_ttl, epiphany_events;
        int              swap_acc_cnt, swap_rej_cnt;
        logic            prev_conv, prev_hor_sat, prev_lyap_rej;
        begin
            // Deterministic ChaosEngram master seed (zero history buffer)
            rng             = 64'h9E37_79B9_7F4A_7C15;
            ep_seq          = 32'd2;
            epiphany_ttl    = 0;
            epiphany_events = 0;
            swap_acc_cnt    = 0;
            swap_rej_cnt    = 0;
            prev_conv       = 1'b0;
            prev_hor_sat    = 1'b0;
            prev_lyap_rej   = 1'b0;

            cap_tbl[0] = 64'd0;
            cap_tbl[1] = 64'd1;
            cap_tbl[2] = 64'd3;
            cap_tbl[3] = 64'd7;
            cap_tbl[4] = 64'd15;
            cap_tbl[5] = 64'd31;
            cap_tbl[6] = 64'd63;
            cap_tbl[7] = 64'd255;

            // Seed all 16 hardware SRAM chart slots with 4-regime manifold diversity
            for (int s = 0; s < 16; s++) begin
                seed_searu_chart_slot(s, rng, 32'd1, chart_tmp);
                top_write_chart_512(8'(s), chart_tmp);
                for (int w = 0; w < 8; w++) begin
                    c_arena[s*8 + w] = chart_tmp[w*64 +: 64];
                end
            end

            // Initial state & control inputs
            for (int w = 0; w < 8; w++) begin
                rng = next_lfsr(rng);
                st_cur[w*64 +: 64] = rng;
                c_st[w]            = rng;
            end
            cap_raw  = 64'd15;
            cap_eff  = 64'd15;
            u_ext    = 64'd0;
            mode_req = 16'hA55A;
            top_load_state_512(st_cur, cap_raw);

            for (int step = 0; step < N_CRUCIBLE; step++) begin
                int   dim;
                logic is_black_swan, state_dirty;

                state_dirty = 1'b0;

                // 1. DualChaosState SWAR Cooling Schedule + Epiphany Re-Heat / 1-Bit Edge Lock
                if (epiphany_ttl > 0) begin
                    epiphany_ttl--;
                    // Re-heat burst on transition 4->3 to explore across newly crossed phase boundary,
                    // otherwise lock at k=6 (1-bit ultra-fine Bernoulli mask) right on the cliff!
                    k_cool = (epiphany_ttl == 3) ? 2 : 6;
                end else begin
                    k_cool = (((step & 255) * 6) >> 8) + 1;

                    // 2. Macro Regime Leap every 128 steps when not in Epiphany lock
                    if ((step & 127) == 0) begin
                        int reseed_slot;
                        rng         = next_lfsr(rng);
                        cap_raw     = cap_tbl[rng[10:8]];
                        cap_eff     = cap_raw & 64'd15;
                        reseed_slot = int'(rng[3:0]);
                        ep_seq++;
                        seed_searu_chart_slot(reseed_slot, rng, ep_seq, chart_tmp);
                        top_write_chart_512(8'(reseed_slot), chart_tmp);
                        for (int w = 0; w < 8; w++) begin
                            c_arena[reseed_slot*8 + w] = chart_tmp[w*64 +: 64];
                        end

                        // Alternate interior `sub_valid == 1` vs raw 64-bit hostile state reseeds
                        if ((step & 128) == 0) begin
                            logic signed [63:0] s_tmp;
                            rng = next_lfsr(rng); s_tmp = $signed(rng); st_cur[63:0]    = $unsigned(s_tmp >>> 8);
                            rng = next_lfsr(rng); s_tmp = $signed(rng); st_cur[127:64]  = $unsigned(s_tmp >>> 20);
                            rng = next_lfsr(rng); s_tmp = $signed(rng); st_cur[191:128] = $unsigned(s_tmp >>> 33);
                            rng = next_lfsr(rng); s_tmp = $signed(rng); st_cur[255:192] = $unsigned(s_tmp >>> 33);
                            for (int w = 4; w < 8; w++) begin
                                rng = next_lfsr(rng);
                                st_cur[w*64 +: 64] = rng;
                            end
                        end else begin
                            for (int w = 0; w < 8; w++) begin
                                rng = next_lfsr(rng);
                                st_cur[w*64 +: 64] = rng;
                            end
                        end
                        state_dirty = 1'b1;
                    end
                end

                // 3. Periodic Live `@swap` Firewall Lockstep against `flow_arena_try_swap`
                if ((step & 15) == 7) begin
                    logic [7:0]   sw_slot;
                    int unsigned  sw_ep;
                    logic         rtl_sw_acc;
                    int           rtl_sw_cyc;
                    byte unsigned c_sw_acc;

                    rng     = next_lfsr(rng);
                    sw_slot = rng[7:0];
                    ep_seq++;
                    seed_searu_chart_slot({28'd0, sw_slot[3:0]}, rng, ep_seq, cand_cell);

                    // Mix valid monotonically increasing swaps with hostile firewall traps
                    if (rng[11:9] == 3'd0) begin
                        sw_ep = 32'd0; // stale epoch trap
                    end else if (rng[11:9] == 3'd1) begin
                        sw_ep = ep_seq;
                        cand_cell[351:320] = {16'd0, 8'd200, 8'd200}; // out-of-capacity trap
                    end else if (rng[11:9] == 3'd2) begin
                        sw_ep = ep_seq;
                        cand_cell[255:192] = {32'sd500, -32'sd500};   // inverted Schmitt trap
                    end else if (rng[11:9] == 3'd3) begin
                        sw_ep = ep_seq;
                        cand_cell[319:256] = {32'd0, cand_cell[287:256]}; // dt_max == 0 trap
                    end else begin
                        sw_ep = ep_seq;
                        cand_cell[255:192] = {-32'sd100000, 32'sd100000};
                        cand_cell[319:288] = 32'sd524288;
                        cand_cell[351:336] = {cand_cell[351:344] & cap_eff[7:0],
                                              cand_cell[343:336] & cap_eff[7:0]};
                    end

                    for (int w = 0; w < 8; w++) c_cand[w] = cand_cell[w*64 +: 64];

                    if (state_dirty) begin
                        top_load_state_512(st_cur, cap_raw);
                        state_dirty = 1'b0;
                    end
                    top_try_swap(sw_slot, sw_ep, cand_cell, rtl_sw_acc, rtl_sw_cyc);
                    c_sw_acc = ele_dpi_try_swap(c_arena, {56'd0, sw_slot}, cap_eff, sw_ep, c_cand);

                    if ((rtl_sw_acc !== (c_sw_acc != 8'd0)) ||
                        (rtl_sw_acc && (rtl_sw_cyc != 8))   ||
                        ((!rtl_sw_acc) && (rtl_sw_cyc != 1))) begin
                        $fatal(1, "FAIL [SEARU-SWAP step=%0d]: rtl_acc=%0b (cyc=%0d) != c_acc=%0d!",
                               step, rtl_sw_acc, rtl_sw_cyc, c_sw_acc);
                    end
                    if (rtl_sw_acc) swap_acc_cnt++;
                    else            swap_rej_cnt++;
                end

                // 4. Branchless k-Way SWAR XorShift AND Mask + 1/32 Zipf Singular Black-Swan Jump
                swar_mask = 64'hFFFF_FFFF_FFFF_FFFF;
                for (int b = 0; b < k_cool; b++) begin
                    rng       = next_lfsr(rng);
                    swar_mask = swar_mask & rng;
                end

                rng           = next_lfsr(rng);
                dim           = int'(rng[3:0]);
                is_black_swan = (rng[8:4] == 5'd0);

                if (dim < 8) begin
                    if (is_black_swan) begin
                        st_cur[dim*64 +: 64] = lattice16_val(int'(rng[12:9]));
                    end else begin
                        st_cur[dim*64 +: 64] = st_cur[dim*64 +: 64] ^ swar_mask;
                    end
                    state_dirty = 1'b1;
                end else if (dim < 14) begin
                    int b_idx, act_slot;
                    b_idx    = dim - 8;
                    act_slot = {16'd0, st_cur[351:336] & cap_eff[15:0]};
                    // Keep autonomous b1=b2=0 intact on `slot % 4 == 0` so 2^-49 fiber stays hot
                    if (!(((act_slot & 3) == 0) && ((b_idx == 1) || (b_idx == 2)))) begin
                        for (int w = 0; w < 8; w++) begin
                            chart_tmp[w*64 +: 64] = c_arena[act_slot*8 + w];
                        end
                        if (is_black_swan) begin
                            chart_tmp[b_idx*64 +: 64] = lattice16_val(int'(rng[12:9]));
                        end else begin
                            chart_tmp[b_idx*64 +: 64] = chart_tmp[b_idx*64 +: 64] ^ swar_mask;
                        end
                        c_arena[act_slot*8 + b_idx] = chart_tmp[b_idx*64 +: 64];
                        top_write_chart_512(8'(act_slot), chart_tmp);
                    end
                end else if (dim == 14) begin
                    u_ext = is_black_swan ? lattice16_val(int'(rng[12:9])) : (u_ext ^ swar_mask);
                end else begin
                    mode_req = mode_req ^ swar_mask[15:0];
                end

                if (state_dirty) begin
                    top_load_state_512(st_cur, cap_raw);
                end

                // 5. Execute Hardware `ele_flow_atlas_top` Step & Verify 512-Bit `0-ULP` Parity vs C99
                for (int w = 0; w < 8; w++) c_st[w] = st_cur[w*64 +: 64];
                exec_top_step(u_ext, mode_req, rtl_out);
                flow_step_atlas(c_out, c_st, c_arena, cap_eff, longint'(u_ext), mode_req);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);

                if (rtl_out !== c_packed) begin
                    $display("FAIL [SEARU-CRUCIBLE step=%0d seed=0x9e3779b97f4a7c15]: 512-bit SV != FLOW C99!", step);
                    for (int w = 0; w < 8; w++) begin
                        if (rtl_out[w*64 +: 64] !== c_packed[w*64 +: 64]) begin
                            $display("  lane w%0d: sv=%016x != c=%016x",
                                     w, rtl_out[w*64 +: 64], c_packed[w*64 +: 64]);
                        end
                    end
                    $fatal(1);
                end

                // 6. SEARU `Epiphany` Phase-Boundary Detector (chart / convergence / horizon / Lyapunov cliffs)
                begin
                    int          act_s;
                    logic signed [31:0] raw_dt32;
                    longint unsigned    dt_eff64;
                    logic               cur_conv, cur_hor_sat, cur_lyap_rej, near_q49, chart_hop;

                    act_s    = {16'd0, st_cur[351:336] & cap_eff[15:0]};
                    raw_dt32 = $signed(c_arena[act_s*8 + 4][63:32]);
                    if (raw_dt32 == 32'sh8000_0000)      dt_eff64 = 64'd1;
                    else if (raw_dt32 < 32'sd1)          dt_eff64 = 64'd1;
                    else                                 dt_eff64 = 64'(unsigned'(raw_dt32));

                    chart_hop    = (rtl_out[351:336] != st_cur[351:336]);
                    cur_conv     = rtl_out[384];
                    cur_hor_sat  = (rtl_out[319:256] == dt_eff64);
                    cur_lyap_rej = (rtl_out[319:256] == 64'd0) && (!cur_conv);
                    near_q49     = ($signed(rtl_out[63:0]) >= -64'sd8) &&
                                   ($signed(rtl_out[63:0]) <= 64'sd8)  &&
                                   (rtl_out[319:256] != 64'd0);

                    if ((epiphany_ttl == 0) &&
                        (chart_hop || near_q49 || (cur_conv != prev_conv) ||
                         (cur_hor_sat != prev_hor_sat) || (cur_lyap_rej != prev_lyap_rej))) begin
                        epiphany_events++;
                        epiphany_ttl = 8;
                    end
                    prev_conv     = cur_conv;
                    prev_hor_sat  = cur_hor_sat;
                    prev_lyap_rej = cur_lyap_rej;
                end

                // 7. Closed-Loop Manifold Trajectory Chaining on 75% of steps (retaining live RTL state_w_q!)
                if ((step & 3) != 0) begin
                    st_cur = rtl_out;
                end else begin
                    top_load_state_512(st_cur, cap_raw);
                end
            end

            if ((epiphany_events == 0) || (swap_acc_cnt == 0) || (swap_rej_cnt == 0)) begin
                $fatal(1, "FAIL [SEARU-CRUCIBLE]: insufficient phase-boundary coverage (epi=%0d, acc=%0d, rej=%0d)!",
                       epiphany_events, swap_acc_cnt, swap_rej_cnt);
            end

            $display("  [PASS] Probe 6 (SEARU DualChaosState + Epiphany Crucible): %0d transitions, %0d epiphanies, %0d/%0d @swap acc/rej (0-ULP)",
                     N_CRUCIBLE, epiphany_events, swap_acc_cnt, swap_rej_cnt);
        end
    endtask

    // =========================================================================
    // Probe 7: FSM Handshake Bubble Chaos (`0..17` cyc), Busy Illegal-Write
    //          Immunity & Exhaustive Cycle-by-Cycle Mid-Flight `rst_n` Recovery
    // =========================================================================
    task automatic drive_core_idle_bubbles(input int n_bubbles, inout logic [63:0] rng);
        begin
            for (int b = 0; b < n_bubbles; b++) begin
                rng           = next_lfsr(rng);
                core_wr_valid = 1'b0; // Invalid! Bus carries hostile garbage that MUST be ignored
                core_wr_kind  = rng[1:0];
                core_wr_idx   = rng[4:2];
                core_wr_data  = (b % 3 == 0) ? 64'hDEAD_BEEF_CAFE_F00D :
                                ((b % 3 == 1) ? 64'h8000_0000_0000_0000 : 64'h7FFF_FFFF_FFFF_FFFF);
                @(posedge clk);
                #1;
                if (!core_ready || core_done) begin
                    $fatal(1, "FAIL [BUBBLE-CORE]: spurious busy/done during wr_valid_i==0 idle gap!");
                end
            end
            core_wr_data = 64'd0;
        end
    endtask

    task automatic drive_top_idle_bubbles(input int n_bubbles, inout logic [63:0] rng);
        begin
            for (int b = 0; b < n_bubbles; b++) begin
                rng           = next_lfsr(rng);
                top_cmd_valid = 1'b0; // Invalid! Bus carries hostile opcodes/garbage that MUST be ignored
                top_cmd_op    = (b % 2 == 0) ? CMD_SWAP : rng[2:0];
                top_cmd_word  = rng[5:3];
                top_cmd_aux   = rng[21:6];
                top_cmd_data  = (b % 3 == 0) ? 64'hDEAD_BEEF_DEAD_BEEF :
                                ((b % 3 == 1) ? 64'h8000_0000_0000_0000 : 64'h7FFF_FFFF_FFFF_FFFF);
                @(posedge clk);
                #1;
                if (!top_ready || top_step_done || top_swap_done || top_swap_acc) begin
                    $fatal(1, "FAIL [BUBBLE-TOP]: spurious busy/done during cmd_valid_i==0 idle gap!");
                end
            end
            top_cmd_op   = CMD_NOP;
            top_cmd_data = 64'd0;
        end
    endtask

    task automatic run_probe7_protocol_chaos_and_reset();
`ifdef RTL_MUTANT_FAST
        localparam int N_BUBBLE_CORE = 12;
        localparam int N_BUBBLE_TOP  = 12;
        localparam int N_HAMMER      = 8;
        localparam int RST_STRIDE    = 7;
`else
        localparam int N_BUBBLE_CORE = 64;
        localparam int N_BUBBLE_TOP  = 48;
        localparam int N_HAMMER      = 32;
        localparam int RST_STRIDE    = 1;
`endif
        logic [63:0]     rng;
        logic [511:0]    st_in, bd_in, rtl_out, c_packed, bd_quad_ref, bd_lin_ref, bd_au_ref;
        logic [511:0]    cand_valid;
        longint unsigned c_st[8];
        longint unsigned c_arena[128];
        longint unsigned c_out[8];
        longint unsigned c_cand[8];
        int              max_quad_cyc, max_top_cyc;
        int              core_rst_swept, top_rst_swept;
        longint unsigned dummy_tog;
        begin
            rng = 64'hF00D_CAFE_1234_5678;

            bd_quad_ref = pack_cell(
                -64'sd1638400, 64'sd65536, 64'sd6554,
                {-32'sd3276800, 32'sd3276800},
                {32'sd983040, 32'sd65536},
                {32'd0, 8'd1, 8'd2, 16'h00F0},
                {32'd1, 32'd0}, 64'd0
            );
            bd_lin_ref = pack_cell(
                -64'sd1638400, 64'sd65536, 64'sd0,
                {-32'sd3276800, 32'sd3276800},
                {32'sd1966080, 32'sd65536},
                {32'd0, 8'd0, 8'd1, 16'h0F00},
                {32'd1, 32'd0}, 64'd0
            );
            bd_au_ref = pack_cell(
                64'sd0, 64'sd0, 64'sd0,
                {32'h8000_0001, 32'h7FFF_FFFF},
                {32'h0000_0111, 32'h0001_0000},
                {32'd0, 8'd1, 8'd0, 16'h55AA},
                {32'd1, 32'd0}, 64'd0
            );

            // -----------------------------------------------------------------
            // 7A-1. `ele_flow_core` Intermittent `wr_valid_i` Bubbles (`0..17` cyc)
            //       with Scrambled Word Arrival & Hostile Invalid-Cycle Garbage
            // -----------------------------------------------------------------
            reset_all();
            for (int t = 0; t < N_BUBBLE_CORE; t++) begin
                logic [63:0] u_v, cap_v;
                logic [15:0] mreq_v;
                int          perm;

                for (int w = 0; w < 8; w++) begin
                    rng = next_lfsr(rng);
                    st_in[w*64 +: 64] = (w == 3) ? {{33{rng[30]}}, rng[30:0]} : rng;
                    c_st[w]           = st_in[w*64 +: 64];
                end
                st_in[351:336] = {12'd0, rng[3:0]};
                c_st[5]        = st_in[383:320];

                bd_in = (t % 3 == 0) ? bd_quad_ref : ((t % 3 == 1) ? bd_lin_ref : bd_au_ref);
                for (int s = 0; s < 16; s++) begin
                    for (int w = 0; w < 8; w++) c_arena[s*8 + w] = bd_in[w*64 +: 64];
                end
                rng    = next_lfsr(rng);
                cap_v  = 64'h000F;
                u_v    = {{33{rng[30]}}, rng[30:0]};
                mreq_v = rng[47:32];
                perm   = int'(rng[50:48]);

                // Stream 8 state words in scrambled order `(w ^ perm)` with `0..17` cycle bubbles
                for (int w = 0; w < 8; w++) begin
                    int idx_p;
                    idx_p = (w ^ perm) & 7;
                    rng   = next_lfsr(rng);
                    drive_core_idle_bubbles(int'(rng % 64'd18), rng);
                    core_wr_valid = 1'b1;
                    core_wr_kind  = 2'd0;
                    core_wr_idx   = 3'(idx_p);
                    core_wr_data  = st_in[idx_p*64 +: 64];
                    @(posedge clk);
                    #1;
                end

                // Stream 6 bound words in reverse/permuted order with `0..17` cycle bubbles
                for (int w = 0; w < 6; w++) begin
                    int idx_b;
                    idx_b = (t % 2 == 0) ? (5 - w) : w;
                    rng   = next_lfsr(rng);
                    drive_core_idle_bubbles(int'(rng % 64'd18), rng);
                    core_wr_valid = 1'b1;
                    core_wr_kind  = 2'd1;
                    core_wr_idx   = 3'(idx_b);
                    core_wr_data  = bd_in[idx_b*64 +: 64];
                    @(posedge clk);
                    #1;
                end

                rng = next_lfsr(rng);
                drive_core_idle_bubbles(int'(rng % 64'd18), rng);
                core_wr_valid = 1'b1;
                core_wr_kind  = 2'd2;
                core_wr_idx   = 3'd0;
                core_wr_data  = {32'd0, mreq_v, cap_v[15:0]};
                @(posedge clk);
                #1;

                rng = next_lfsr(rng);
                drive_core_idle_bubbles(int'(rng % 64'd18), rng);
                core_wr_valid = 1'b1;
                core_wr_kind  = 2'd2;
                core_wr_idx   = 3'd1;
                core_wr_data  = u_v;
                @(posedge clk);
                #1;
                core_wr_valid = 1'b0;

                while (!core_done) begin
                    @(posedge clk);
                    #1;
                end

                // Idle bubble gap `0..17` cycles AFTER `done_o` before reading `rd_data_o`!
                rng = next_lfsr(rng);
                drive_core_idle_bubbles(int'(rng % 64'd18), rng);
                for (int w = 0; w < 8; w++) begin
                    core_rd_idx = 3'(w);
                    #1;
                    rtl_out[w*64 +: 64] = core_rd_data;
                end

                flow_step_atlas(c_out, c_st, c_arena, cap_v, longint'(u_v), mreq_v);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);
                if (rtl_out !== c_packed) begin
                    $fatal(1, "FAIL [BUBBLE-CORE t=%0d]: 512-bit mismatch after 0..17 cycle handshake bubbles!", t);
                end
            end

            // -----------------------------------------------------------------
            // 7A-2. `ele_flow_atlas_top` Intermittent `cmd_valid_i` Bubbles (`0..17` cyc)
            //       with Interleaved `@swap` and Multi-Chart Step Transactions
            // -----------------------------------------------------------------
            reset_all();
            st_in = pack_cell(64'sd327680, -64'sd196608, 64'sd43691, -64'sd524288,
                              64'd0, {32'd0, 16'd0, 16'h1234}, 64'd0, 64'd0);
            for (int w = 0; w < 8; w++) c_st[w] = st_in[w*64 +: 64];

            // Load cap_mask = 3 and 4 charts with `0..17` cycle bubbles between every single 64b beat!
            drive_top_idle_bubbles(5, rng);
            top_cmd_valid = 1'b1;
            top_cmd_op    = CMD_SET_CAP;
            top_cmd_word  = 3'd0;
            top_cmd_aux   = 16'd0;
            top_cmd_data  = 64'd3;
            @(posedge clk);
            #1;
            for (int w = 0; w < 8; w++) begin
                rng = next_lfsr(rng);
                drive_top_idle_bubbles(int'(rng % 64'd18), rng);
                top_cmd_valid = 1'b1;
                top_cmd_op    = CMD_LOAD_STATE;
                top_cmd_word  = 3'(w);
                top_cmd_aux   = 16'd0;
                top_cmd_data  = st_in[w*64 +: 64];
                @(posedge clk);
                #1;
            end
            for (int s = 0; s < 4; s++) begin
                bd_in = (s == 0) ? bd_quad_ref : ((s == 1) ? bd_lin_ref : bd_au_ref);
                for (int w = 0; w < 8; w++) begin
                    c_arena[s*8 + w] = bd_in[w*64 +: 64];
                    rng              = next_lfsr(rng);
                    drive_top_idle_bubbles(int'(rng % 64'd9), rng);
                    top_cmd_valid = 1'b1;
                    top_cmd_op    = CMD_WRITE_CHART;
                    top_cmd_word  = 3'(w);
                    top_cmd_aux   = {8'd0, 8'(s)};
                    top_cmd_data  = bd_in[w*64 +: 64];
                    @(posedge clk);
                    #1;
                end
            end
            top_cmd_valid = 1'b0;
            top_cmd_op    = CMD_NOP;

            for (int t = 0; t < N_BUBBLE_TOP; t++) begin
                rng = next_lfsr(rng);
                drive_top_idle_bubbles(int'(rng % 64'd18), rng);

                if ((t % 3) == 1) begin
                    // Interleaved `@swap` with `0..17` cycle bubbles between candidate staging beats!
                    logic [7:0]   sw_slot;
                    int unsigned  sw_ep;
                    byte unsigned c_acc;
                    sw_slot    = 8'(t & 3);
                    sw_ep      = (t % 2 == 1) ? int'(32'd10 + 32'(t)) : 32'd0; // mix accepted & rejected
                    cand_valid = pack_cell(
                        -64'sd655360 + longint'(t) * 64'sd4096,
                        64'sd65536,
                        (t % 4 == 0) ? 64'sd0 : 64'sd4096,
                        {-32'sd2000000, 32'sd2000000},
                        {32'sd1048576, 32'sd65536},
                        {32'd0, 8'((t + 1) & 3), 8'((t + 2) & 3), 16'h00FF},
                        {sw_ep, 32'd0}, 64'd0
                    );
                    for (int w = 0; w < 8; w++) c_cand[w] = cand_valid[w*64 +: 64];
                    for (int w = 0; w < 6; w++) begin
                        rng = next_lfsr(rng);
                        drive_top_idle_bubbles(int'(rng % 64'd18), rng);
                        top_cmd_valid = 1'b1;
                        top_cmd_op    = CMD_STAGE_CAND;
                        top_cmd_word  = 3'(w);
                        top_cmd_aux   = 16'd0;
                        top_cmd_data  = cand_valid[w*64 +: 64];
                        @(posedge clk);
                        #1;
                    end
                    rng = next_lfsr(rng);
                    drive_top_idle_bubbles(int'(rng % 64'd18), rng);
                    top_cmd_valid = 1'b1;
                    top_cmd_op    = CMD_SWAP;
                    top_cmd_word  = 3'd0;
                    top_cmd_aux   = {8'd0, sw_slot};
                    top_cmd_data  = {32'd0, sw_ep};
                    @(posedge clk);
                    #1;
                    top_cmd_valid = 1'b0;
                    top_cmd_op    = CMD_NOP;

                    while (!top_swap_done) begin
                        @(posedge clk);
                        #1;
                    end
                    c_acc = ele_dpi_try_swap(c_arena, {56'd0, sw_slot}, 64'd3, sw_ep, c_cand);
                    if (top_swap_acc !== (c_acc != 8'd0)) begin
                        $fatal(1, "FAIL [BUBBLE-TOP-SWAP t=%0d]: swap parity mismatch!", t);
                    end
                end else begin
                    logic [63:0] u_v;
                    logic [15:0] mreq_v;
                    u_v    = {{33{rng[30]}}, rng[30:0]};
                    mreq_v = rng[47:32];

                    top_cmd_valid = 1'b1;
                    top_cmd_op    = CMD_STEP;
                    top_cmd_word  = 3'd0;
                    top_cmd_aux   = mreq_v;
                    top_cmd_data  = u_v;
                    @(posedge clk);
                    #1;
                    top_cmd_valid = 1'b0;
                    top_cmd_op    = CMD_NOP;

                    while (!top_step_done) begin
                        @(posedge clk);
                        #1;
                    end
                    rng = next_lfsr(rng);
                    drive_top_idle_bubbles(int'(rng % 64'd18), rng);
                    top_read_state_512(rtl_out);

                    flow_step_atlas(c_out, c_st, c_arena, 64'd3, longint'(u_v), mreq_v);
                    for (int w = 0; w < 8; w++) c_st[w] = c_out[w];
                    c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                         c_out[4], c_out[5], c_out[6], c_out[7]);
                    if (rtl_out !== c_packed) begin
                        $fatal(1, "FAIL [BUBBLE-TOP-STEP t=%0d]: 512-bit mismatch!", t);
                    end
                end
            end

            // -----------------------------------------------------------------
            // 7B. Busy-State (`ready_o == 0`) Hostile Write & Input Hammering
            // -----------------------------------------------------------------
            // 7B-1. Hammer `ele_flow_core` on EVERY in-flight cycle (`ready_o == 0`)
            for (int h = 0; h < N_HAMMER; h++) begin
                logic [63:0] u_v1, u_v2;
                logic [15:0] mreq_v;
                rng    = next_lfsr(rng);
                bd_in  = (h % 3 == 0) ? bd_quad_ref : ((h % 3 == 1) ? bd_lin_ref : bd_au_ref);
                st_in  = pack_cell(
                    {{35{rng[28]}}, rng[28:0]},
                    {{35{rng[60]}}, rng[60:32]},
                    64'sd43691,
                    -64'sd1179648 + longint'(h) * 64'sd32768,
                    64'd0,
                    {32'd0, 16'd1, rng[15:0]},
                    {32'(h), 32'd0},
                    rng
                );
                u_v1   = 64'sd393216 + longint'(h) * 64'sd4096;
                u_v2   = -64'sd262144;
                mreq_v = ~rng[15:0];

                for (int w = 0; w < 8; w++) c_st[w] = st_in[w*64 +: 64];
                for (int s = 0; s < 16; s++) begin
                    for (int w = 0; w < 8; w++) c_arena[s*8 + w] = bd_in[w*64 +: 64];
                end

                // Load clean inputs while `core_ready == 1`
                for (int w = 0; w < 8; w++) begin
                    core_wr_valid = 1'b1; core_wr_kind = 2'd0; core_wr_idx = 3'(w);
                    core_wr_data  = st_in[w*64 +: 64];
                    @(posedge clk); #1;
                end
                for (int w = 0; w < 6; w++) begin
                    core_wr_valid = 1'b1; core_wr_kind = 2'd1; core_wr_idx = 3'(w);
                    core_wr_data  = bd_in[w*64 +: 64];
                    @(posedge clk); #1;
                end
                core_wr_valid = 1'b1; core_wr_kind = 2'd2; core_wr_idx = 3'd0;
                core_wr_data  = {32'd0, mreq_v, 16'h0003};
                @(posedge clk); #1;

                // Trigger step -> transitions to `core_ready == 0`
                core_wr_valid = 1'b1; core_wr_kind = 2'd2; core_wr_idx = 3'd1;
                core_wr_data  = u_v1;
                @(posedge clk); #1;

                if (core_ready || core_done) begin
                    $fatal(1, "FAIL [HAMMER-CORE h=%0d]: core was not busy after start!", h);
                end

                // Hammer `wr_valid_i=1` with hostile garbage on EVERY SINGLE BUSY CYCLE!
                while (!core_ready) begin
                    rng           = next_lfsr(rng);
                    core_wr_valid = 1'b1;
                    core_wr_kind  = rng[1:0];
                    core_wr_idx   = rng[4:2];
                    case (rng[6:5])
                        2'd0: core_wr_data = 64'hDEAD_BEEF_DEAD_BEEF;
                        2'd1: core_wr_data = 64'h8000_0000_0000_0000; // INT64_MIN
                        2'd2: core_wr_data = 64'h7FFF_FFFF_FFFF_FFFF; // INT64_MAX
                        default: core_wr_data = 64'hFFFF_FFFF_FFFF_FFFF ^ rng;
                    endcase
                    @(posedge clk);
                    #1;
                end
                core_wr_valid = 1'b0;
                if (!core_done) begin
                    $fatal(1, "FAIL [HAMMER-CORE h=%0d]: core_done missing on core_ready recovery!", h);
                end

                for (int w = 0; w < 8; w++) begin
                    core_rd_idx = 3'(w);
                    #1;
                    rtl_out[w*64 +: 64] = core_rd_data;
                end
                flow_step_atlas(c_out, c_st, c_arena, 64'h0003, longint'(u_v1), mreq_v);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);
                if (rtl_out !== c_packed) begin
                    $fatal(1, "FAIL [HAMMER-CORE h=%0d]: busy illegal writes corrupted in-flight result!", h);
                end

                // Immediately trigger a back-to-back step WITHOUT reloading bound_w or ctrl0:
                // proves 100% of stored bound/cap/mode_req registers survived busy hammering!
                for (int w = 0; w < 8; w++) c_st[w] = c_out[w];
                core_wr_valid = 1'b1; core_wr_kind = 2'd2; core_wr_idx = 3'd1;
                core_wr_data  = u_v2;
                @(posedge clk); #1;
                core_wr_valid = 1'b0;
                while (!core_done) begin
                    @(posedge clk); #1;
                end
                for (int w = 0; w < 8; w++) begin
                    core_rd_idx = 3'(w);
                    #1;
                    rtl_out[w*64 +: 64] = core_rd_data;
                end
                flow_step_atlas(c_out, c_st, c_arena, 64'h0003, longint'(u_v2), mreq_v);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);
                if (rtl_out !== c_packed) begin
                    $fatal(1, "FAIL [HAMMER-CORE-RETENTION h=%0d]: busy writes corrupted stored bound/ctrl!", h);
                end
            end

            // 7B-2. Hammer `ele_flow_atlas_top` on EVERY busy cycle during BOTH `CMD_STEP` & `CMD_SWAP`
            reset_all();
            st_in = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648, 64'd0,
                              {32'd0, 16'd0, 16'h00AA}, 64'd0, 64'd0);
            top_load_state_512(st_in, 64'd3);
            for (int s = 0; s < 4; s++) begin
                top_write_chart_512(8'(s), bd_quad_ref);
                for (int w = 0; w < 8; w++) c_arena[s*8 + w] = bd_quad_ref[w*64 +: 64];
            end
            for (int w = 0; w < 8; w++) c_st[w] = st_in[w*64 +: 64];

            // Pre-stage a valid candidate into `cand_buf_q` BEFORE launching busy `CMD_STEP`
            cand_valid = pack_cell(
                -64'sd819200, 64'sd65536, 64'sd3277,
                {-32'sd3276800, 32'sd3276800},
                {32'sd983040, 32'sd65536},
                {32'd0, 8'd0, 8'd1, 16'h000F},
                {32'd5, 32'd0}, 64'd0
            );
            for (int w = 0; w < 8; w++) c_cand[w] = cand_valid[w*64 +: 64];
            top_stage_cand_512(cand_valid);

            // Launch `CMD_STEP` and hammer ALL opcodes (`CMD_WRITE_CHART`, `CMD_STAGE_CAND`, `CMD_SWAP`, `CMD_SET_CAP`, `CMD_LOAD_STATE`) while `top_ready == 0`!
            top_cmd_valid = 1'b1; top_cmd_op = CMD_STEP; top_cmd_word = 3'd0;
            top_cmd_aux   = 16'h55FF; top_cmd_data = 64'sd393216;
            @(posedge clk); #1;
            while (!top_ready) begin
                rng           = next_lfsr(rng);
                top_cmd_valid = 1'b1;
                top_cmd_op    = 3'((rng[2:0] == 3'd0) ? CMD_STAGE_CAND : rng[2:0]);
                top_cmd_word  = rng[5:3];
                top_cmd_aux   = rng[21:6];
                top_cmd_data  = (rng[6]) ? 64'hDEAD_BEEF_CAFE_BABE : 64'h8000_0000_0000_0000;
                if (top_swap_done || top_swap_acc) begin
                    $fatal(1, "FAIL [HAMMER-TOP]: spurious swap_done/acc while step busy!");
                end
                @(posedge clk); #1;
            end
            top_cmd_valid = 1'b0; top_cmd_op = CMD_NOP;
            if (!top_step_done) $fatal(1, "FAIL [HAMMER-TOP]: missing step_done!");
            top_read_state_512(rtl_out);

            flow_step_atlas(c_out, c_st, c_arena, 64'd3, 64'sd393216, 16'h55FF);
            for (int w = 0; w < 8; w++) c_st[w] = c_out[w];
            c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                 c_out[4], c_out[5], c_out[6], c_out[7]);
            if (rtl_out !== c_packed) begin
                $fatal(1, "FAIL [HAMMER-TOP]: busy bus hammering corrupted in-flight atlas step!");
            end

            // Commit the pre-staged `cand_buf_q` WITHOUT re-staging, AND hammer during the 8-cycle SRAM commit!
            top_cmd_valid = 1'b1; top_cmd_op = CMD_SWAP; top_cmd_word = 3'd0;
            top_cmd_aux   = 16'd0; top_cmd_data = {32'd0, 32'd5};
            @(posedge clk); #1;
            while (!top_ready) begin
                rng           = next_lfsr(rng);
                top_cmd_valid = 1'b1;
                top_cmd_op    = (rng[0]) ? CMD_STAGE_CAND : CMD_WRITE_CHART;
                top_cmd_word  = rng[3:1];
                top_cmd_aux   = 16'd0;
                top_cmd_data  = 64'h7FFF_FFFF_FFFF_FFFF;
                @(posedge clk); #1;
            end
            top_cmd_valid = 1'b0; top_cmd_op = CMD_NOP;
            if (!top_swap_done || !top_swap_acc) begin
                $fatal(1, "FAIL [HAMMER-TOP-SWAP]: pre-staged cand_buf_q failed to commit after busy hammering!");
            end
            if (ele_dpi_try_swap(c_arena, 64'd0, 64'd3, 32'd5, c_cand) == 8'd0) $fatal(1);

            exec_top_step(-64'sd262144, 16'hFFFF, rtl_out);
            flow_step_atlas(c_out, c_st, c_arena, 64'd3, -64'sd262144, 16'hFFFF);
            c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                 c_out[4], c_out[5], c_out[6], c_out[7]);
            if (rtl_out !== c_packed) begin
                $fatal(1, "FAIL [HAMMER-TOP-POST]: SRAM or cand_buf_q was corrupted during busy hammering!");
            end

            // -----------------------------------------------------------------
            // 7C. Exhaustive Mid-Flight `rst_n` Abort & 0-ULP Recovery Sweep
            // -----------------------------------------------------------------
            reset_all();
            st_in = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648, 64'd0, 64'd0, 64'd0, 64'd0);
            exec_core_step(st_in, bd_quad_ref, 64'hFFFF, 64'sd393216, 16'd0, rtl_out, max_quad_cyc, dummy_tog);
            if (&{1'b0, dummy_tog}) begin end
            if ((u_core_dut.PROFILE_DSP_COUNT == 3) && (max_quad_cyc != 73)) begin
                $fatal(1, "FAIL [RST-SWEEP]: expected Canonical dsp=3 max_quad_cyc=73, got %0d!", max_quad_cyc);
            end
            if ((u_core_dut.PROFILE_DSP_COUNT == 1) && (max_quad_cyc != 155)) begin
                $fatal(1, "FAIL [RST-SWEEP]: expected Area dsp=1 max_quad_cyc=155, got %0d!", max_quad_cyc);
            end

            // 7C-1. Sweep EVERY SINGLE in-flight cycle `c = 1 .. max_quad_cyc` on `ele_flow_core`
            core_rst_swept = 0;
            for (int c = 1; c <= max_quad_cyc; c += RST_STRIDE) begin
                logic [511:0] dirty_st, clean_st, clean_bd;
                logic [63:0]  clean_u;
                logic [15:0]  clean_m;

                // Launch hostile full-quadratic solve with extreme values
                dirty_st = pack_cell(
                    -64'sh1000_0000_0000_0000,
                    -64'sh0000_2000_0000_0000,
                    64'h0000_0000_7FFF_FFFF,
                    -64'sd1179648,
                    64'hDEAD_BEEF_DEAD_BEEF,
                    64'hFFFF_FFFF_FFFF_FFFF,
                    64'hCAFE_BABE_0000_0001,
                    64'hFFFF_0001_FFFF_FFFF
                );
                for (int w = 0; w < 8; w++) begin
                    core_wr_valid = 1'b1; core_wr_kind = 2'd0; core_wr_idx = 3'(w);
                    core_wr_data  = dirty_st[w*64 +: 64];
                    @(posedge clk); #1;
                end
                for (int w = 0; w < 6; w++) begin
                    core_wr_valid = 1'b1; core_wr_kind = 2'd1; core_wr_idx = 3'(w);
                    core_wr_data  = bd_quad_ref[w*64 +: 64];
                    @(posedge clk); #1;
                end
                core_wr_valid = 1'b1; core_wr_kind = 2'd2; core_wr_idx = 3'd0;
                core_wr_data  = 64'h0000_0000_AAAA_0003;
                @(posedge clk); #1;
                core_wr_valid = 1'b1; core_wr_kind = 2'd2; core_wr_idx = 3'd1;
                core_wr_data  = 64'sd393216;
                @(posedge clk); #1;
                core_wr_valid = 1'b0;

                // Advance to exact in-flight cycle offset `c` (`1 .. max_quad_cyc`)
                for (int adv = 1; adv < c; adv++) begin
                    if (core_ready || core_done) begin
                        $fatal(1, "FAIL [RST-CORE c=%0d]: premature completion at adv=%0d!", c, adv);
                    end
                    @(posedge clk);
                    #1;
                end
                if (core_ready || core_done) begin
                    $fatal(1, "FAIL [RST-CORE c=%0d]: core was not in-flight at abort point!", c);
                end

                // Assert mid-cycle asynchronous `rst_n = 0` and verify immediate safe outputs & 0 X-leakage
                #2;
                rst_n = 1'b0;
                #1;
                if ((core_ready !== 1'b1) || (core_done !== 1'b0) || ((^core_rd_data) === 1'bx)) begin
                    $fatal(1, "FAIL [RST-CORE c=%0d]: unsafe or X output during rst_n==0!", c);
                end
                repeat (1 + (c & 1)) @(posedge clk);
                #1;
                rst_n = 1'b1;
                @(posedge clk);
                #1;
                if ((core_ready !== 1'b1) || (core_done !== 1'b0)) begin
                    $fatal(1, "FAIL [RST-CORE c=%0d]: deadlock or spurious done after rst_n release!", c);
                end
                for (int w = 0; w < 8; w++) begin
                    core_rd_idx = 3'(w);
                    #1;
                    if (core_rd_data !== 64'd0) begin
                        $fatal(1, "FAIL [RST-CORE c=%0d]: state_w_q[%0d] not zeroed by reset!", c, w);
                    end
                end

                // Immediately verify 512-bit 0-ULP parity on the very next transaction across all regimes!
                rng      = next_lfsr(rng);
                clean_bd = (c % 3 == 0) ? bd_quad_ref : ((c % 3 == 1) ? bd_lin_ref : bd_au_ref);
                clean_st = pack_cell(
                    64'sd327680, -64'sd196608, 64'sd43691,
                    {{35{rng[28]}}, rng[28:0]},
                    64'd0, {32'd0, 16'd1, rng[15:0]}, {32'(c), 32'd0}, rng
                );
                clean_u  = {{35{rng[60]}}, rng[60:32]};
                clean_m  = rng[31:16];
                verify_core_against_c("RST-CORE-REC", c, clean_st, clean_bd, 64'h000F, clean_u, clean_m);
                core_rst_swept++;
            end

            // 7C-2. Sweep EVERY SINGLE in-flight cycle on `ele_flow_atlas_top` (`1 .. max_quad_cyc + 7` + `@swap` beats `1..8`)
            reset_all();
            max_top_cyc   = max_quad_cyc + 7; // 6 SRAM fetch beats + 1 core start beat + max_quad_cyc
            top_rst_swept = 0;

            for (int c = 1; c <= max_top_cyc; c += RST_STRIDE) begin
                st_in = pack_cell(64'd0, 64'd0, 64'd0, -64'sd1179648, 64'd0,
                                  {32'd0, 16'd0, 16'h0055}, 64'd0, 64'd0);
                top_load_state_512(st_in, 64'd3);
                top_write_chart_512(8'd0, bd_quad_ref);

                top_cmd_valid = 1'b1; top_cmd_op = CMD_STEP; top_cmd_word = 3'd0;
                top_cmd_aux   = 16'hFFFF; top_cmd_data = 64'sd393216;
                @(posedge clk); #1;
                top_cmd_valid = 1'b0; top_cmd_op = CMD_NOP;

                for (int adv = 1; adv < c; adv++) begin
                    if (top_ready || top_step_done) begin
                        $fatal(1, "FAIL [RST-TOP c=%0d]: premature top_ready/step_done at adv=%0d!", c, adv);
                    end
                    @(posedge clk); #1;
                end
                if (top_ready || top_step_done) begin
                    $fatal(1, "FAIL [RST-TOP c=%0d]: atlas_top was not busy at abort point!", c);
                end

                #2;
                rst_n = 1'b0;
                #1;
                if ((top_ready !== 1'b1) || top_step_done || top_swap_done || top_swap_acc ||
                    ((^top_rd_data) === 1'bx)) begin
                    $fatal(1, "FAIL [RST-TOP c=%0d]: unsafe or X output during rst_n==0!", c);
                end
                @(posedge clk); #1;
                rst_n = 1'b1;
                @(posedge clk); #1;
                if ((top_ready !== 1'b1) || top_step_done || top_swap_done || top_swap_acc) begin
                    $fatal(1, "FAIL [RST-TOP c=%0d]: deadlock or spurious pulse after rst_n release!", c);
                end

                // Verify immediate post-reset step parity (`slot_valid_q == 0` -> zeroed chart + clean recovery)
                bd_in = (c % 2 == 0) ? bd_quad_ref : bd_lin_ref;
                top_write_chart_512(8'd1, bd_in);
                st_in = pack_cell(64'sd131072, -64'sd65536, 64'd0, -64'sd524288 + longint'(c) * 64'sd2048,
                                  64'd0, {32'd0, 16'd1, 16'h1234}, 64'd0, 64'd0);
                top_load_state_512(st_in, 64'd3);
                exec_top_step(64'sd262144, 16'hA5A5, rtl_out);

                for (int w = 0; w < 8; w++) c_st[w] = st_in[w*64 +: 64];
                for (int s = 0; s < 16; s++) begin
                    for (int w = 0; w < 8; w++) c_arena[s*8 + w] = bd_in[w*64 +: 64];
                end
                flow_step_atlas(c_out, c_st, c_arena, 64'd3, 64'sd262144, 16'hA5A5);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);
                if (rtl_out !== c_packed) begin
                    $fatal(1, "FAIL [RST-TOP-REC c=%0d]: post-reset atlas step mismatch!", c);
                end
                top_rst_swept++;
            end

            // 7C-3. Sweep aborting an accepted `@swap` at EVERY beat `b = 1 .. 8` (`TOP_SWAP_CHK` + 7 SRAM beats)
            for (int b = 1; b <= 8; b++) begin
                cand_valid = pack_cell(
                    -64'sd1638400, 64'sd65536, 64'sd6554,
                    {-32'sd3276800, 32'sd3276800},
                    {32'sd983040, 32'sd65536},
                    {32'd0, 8'd0, 8'd0, 16'hFFFF},
                    {32'd9, 32'd0}, 64'd0
                );
                top_stage_cand_512(cand_valid);
                top_cmd_valid = 1'b1; top_cmd_op = CMD_SWAP; top_cmd_word = 3'd0;
                top_cmd_aux   = 16'd2; top_cmd_data = {32'd0, 32'd9};
                @(posedge clk); #1;
                top_cmd_valid = 1'b0; top_cmd_op = CMD_NOP;

                for (int adv = 1; adv < b; adv++) begin
                    if (top_ready || top_swap_done) begin
                        $fatal(1, "FAIL [RST-SWAP b=%0d]: premature swap_done at adv=%0d!", b, adv);
                    end
                    @(posedge clk); #1;
                end
                #2;
                rst_n = 1'b0;
                #1;
                if ((top_ready !== 1'b1) || top_swap_done || top_swap_acc) begin
                    $fatal(1, "FAIL [RST-SWAP b=%0d]: unsafe output during mid-swap rst_n==0!", b);
                end
                @(posedge clk); #1;
                rst_n = 1'b1;
                @(posedge clk); #1;

                // Without writing slot 2 post-reset, `slot_valid_q[2] == 0` MUST mask any torn SRAM words to 0!
                st_in = pack_cell(64'sd65536, -64'sd32768, 64'sd1024, 64'sd4096,
                                  64'd0, {32'd0, 16'd2, 16'h0000}, 64'd0, 64'd0);
                top_load_state_512(st_in, 64'd3);
                exec_top_step(64'sd0, 16'hBEEF, rtl_out);
                for (int w = 0; w < 8; w++) c_st[w] = st_in[w*64 +: 64];
                for (int s = 0; s < 16; s++) begin
                    for (int w = 0; w < 8; w++) c_arena[s*8 + w] = 64'd0;
                end
                flow_step_atlas(c_out, c_st, c_arena, 64'd3, 64'sd0, 16'hBEEF);
                c_packed = pack_cell(c_out[0], c_out[1], c_out[2], c_out[3],
                                     c_out[4], c_out[5], c_out[6], c_out[7]);
                if (rtl_out !== c_packed) begin
                    $fatal(1, "FAIL [RST-SWAP b=%0d]: torn mid-swap SRAM state leaked across reset!", b);
                end
                top_rst_swept++;
            end

            $display("  [PASS] Probe 7 (Protocol Bubbles [0..17 cyc], Busy Hammering & Mid-Flight Reset): %0d core + %0d top reset offsets recovered at 0-ULP",
                     core_rst_swept, top_rst_swept);
        end
    endtask

    initial begin
        $display("=== ELE v6.0.0 Full Silicon Wind-Tunnel (`--dsp-count %0d --max-depth %0d`, dff=%0d bits + X-Poisoning vs FLOW v6.0.0 C99) ===",
                 u_core_dut.PROFILE_DSP_COUNT, u_core_dut.PROFILE_MAX_DEPTH, u_core_dut.CORE_DFF_BITS);
        reset_all();
        run_probe5_knife_edges();
        reset_all();
        run_probe3_atlas_and_sram_swap();
        reset_all();
        run_probe7_protocol_chaos_and_reset();
        reset_all();
        run_probe2_symmetry_and_fiber();
        reset_all();
        run_probe6_searu_epiphany_crucible();
        reset_all();
        run_probe4_cmos_energy_telemetry();
        reset_all();
        run_probe1_differential();
        $display("=== ALL 7 SILICON, KNIFE-EDGE, SEARU EPIPHANY, PROTOCOL & ENERGY PROBES PASSED: total_tapeout_defects = 0 ===");
        $finish;
    end
endmodule

`default_nettype wire
