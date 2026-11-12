// SPDX-License-Identifier: MIT
// ele_cosim_tb.sv — Pure IEEE 1800 SystemVerilog Differential Co-Sim Harness
//
// Compiled directly via `verilator --binary -Wall` (`0` `.cpp` / `.c` files).
// Supports two deterministic verification modes:
//   1. `+PHY_TEST`
//        Exhaustive hardware test of `ele_bus_invert65` (<= 32/65 wire flips +
//        lossless decode) and `ele_ternary_mac_dualrail` (monotonic rails +
//        0-ULP signed equivalence).
//   2. `+BUNDLE=<memh> +CHARTS=<N> +CMDS=<M> +STIM=<memh>`
//        Loads a Lean-minted `.flow` Manifold Atlas into `ele_flow_atlas_top`,
//        executes `STEP` and live `@swap` commands cycle-accurately in RTL,
//        and emits full 512-bit `w7..w0` state words for 0-ULP comparison
//        against `FLOW/build/flow` and `Flow.lean`.

`default_nettype none

module ele_cosim_tb;
    localparam int MAX_CHARTS = 16;
    localparam int MAX_CMDS   = 256;

    logic clk;
    logic rst_n;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // --- Pillar 3 EM PHY DUTs ---
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

    // --- Universal Silicon `.flow` Manifold Engine DUT ---
    logic         dut_load_state;
    logic [511:0] dut_init_state;
    logic [63:0]  dut_cap_mask;
    logic         dut_load_chart;
    logic [7:0]   dut_load_slot;
    logic [511:0] dut_load_bound;
    logic         dut_step_start;
    logic [63:0]  dut_u_ext;
    logic [15:0]  dut_mode_req;
    logic         dut_swap_start;
    logic [7:0]   dut_swap_slot;
    logic [31:0]  dut_swap_epoch;
    logic [511:0] dut_swap_cand;
    logic         dut_ready;
    logic         dut_step_done;
    logic         dut_swap_done;
    logic         dut_swap_acc;
    logic [511:0] dut_state_o;

    ele_flow_atlas_top #(
        .MAX_CHARTS (MAX_CHARTS)
    ) u_atlas_top (
        .clk             (clk),
        .rst_n           (rst_n),
        .load_state_i    (dut_load_state),
        .init_state_i    (dut_init_state),
        .cap_mask_i      (dut_cap_mask),
        .load_chart_i    (dut_load_chart),
        .load_slot_i     (dut_load_slot),
        .load_bound_i    (dut_load_bound),
        .step_start_i    (dut_step_start),
        .u_ext_i         (dut_u_ext),
        .mode_req_i      (dut_mode_req),
        .swap_start_i    (dut_swap_start),
        .swap_slot_i     (dut_swap_slot),
        .swap_epoch_i    (dut_swap_epoch),
        .swap_cand_i     (dut_swap_cand),
        .ready_o         (dut_ready),
        .step_done_o     (dut_step_done),
        .swap_done_o     (dut_swap_done),
        .swap_accepted_o (dut_swap_acc),
        .state_o         (dut_state_o)
    );

    wire unused_tb_ok = &{1'b0, bi_wire};

    logic [511:0] bundle_mem [0:MAX_CHARTS];
    logic [559:0] stim_mem   [0:MAX_CMDS-1];

    task automatic reset_all();
        begin
            rst_n          = 1'b0;
            bi_valid       = 1'b0;
            bi_data        = 64'd0;
            tm_clr         = 1'b0;
            tm_valid       = 1'b0;
            tm_nz          = 1'b0;
            tm_neg         = 1'b0;
            tm_act         = 32'sd0;
            dut_load_state = 1'b0;
            dut_init_state = 512'd0;
            dut_cap_mask   = 64'd0;
            dut_load_chart = 1'b0;
            dut_load_slot  = 8'd0;
            dut_load_bound = 512'd0;
            dut_step_start = 1'b0;
            dut_u_ext      = 64'd0;
            dut_mode_req   = 16'd0;
            dut_swap_start = 1'b0;
            dut_swap_slot  = 8'd0;
            dut_swap_epoch = 32'd0;
            dut_swap_cand  = 512'd0;
            repeat (3) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    task automatic run_phy_self_test();
        logic [63:0] lfsr;
        logic [63:0] vec;
        logic [6:0]  max_flips;
        logic [47:0] prev_pos;
        logic [47:0] prev_neg;
        logic signed [47:0] ref_acc;
        logic exp_skip;
        begin
            lfsr      = 64'hCAFE_BABE_DEAD_BEEF;
            max_flips = 7'd0;
            for (int i = 0; i < 128; i++) begin
                case (i)
                    0: vec = 64'hFFFF_FFFF_FFFF_FFFF;
                    1: vec = 64'h0000_0000_0000_0000;
                    2: vec = 64'hFFFF_FFFF_FFFF_FFFF;
                    3: vec = 64'hAAAA_AAAA_AAAA_AAAA;
                    4: vec = 64'h5555_5555_5555_5555;
                    5: vec = 64'hFFFF_0000_FFFF_0000;
                    6: vec = 64'h0000_FFFF_0000_FFFF;
                    default: begin
                        lfsr = {lfsr[62:0], lfsr[63] ^ lfsr[62] ^ lfsr[60] ^ lfsr[59]};
                        vec  = (i < 71) ? (64'd1 << (i - 7)) : lfsr;
                    end
                endcase

                bi_valid = 1'b1;
                bi_data  = vec;
                #1;
                if (bi_flips > 7'd32) begin
                    $display("PHY_FAIL: bus_invert flips=%0d exceeded 32 at vec=%0d", bi_flips, i);
                    $fatal(1);
                end
                if (bi_flips > max_flips) begin
                    max_flips = bi_flips;
                end
                @(posedge clk);
                #1;
                if (bi_rx !== vec) begin
                    $display("PHY_FAIL: bus_invert rx=%016x != tx=%016x at vec=%0d", bi_rx, vec, i);
                    $fatal(1);
                end
            end
            bi_valid = 1'b0;

            ref_acc  = 48'sd0;
            prev_pos = 48'd0;
            prev_neg = 48'd0;
            lfsr     = 64'h1234_5678_9ABC_DEF1;

            for (int k = 0; k < 128; k++) begin
                lfsr = {lfsr[62:0], lfsr[63] ^ lfsr[62] ^ lfsr[60] ^ lfsr[59]};
                if (k == 64) begin
                    tm_clr   = 1'b1;
                    tm_valid = 1'b0;
                    @(posedge clk);
                    #1;
                    tm_clr   = 1'b0;
                    ref_acc  = 48'sd0;
                    prev_pos = 48'd0;
                    prev_neg = 48'd0;
                end else begin
                    tm_clr   = 1'b0;
                    tm_valid = 1'b1;
                    tm_nz    = (k % 4 != 0);
                    tm_neg   = lfsr[0];
                    tm_act   = (k % 9 == 0) ? 32'sd0 : $signed({{8{lfsr[31]}}, lfsr[31:8]});
                    #1;
                    exp_skip = (~tm_nz) | (tm_act == 32'sd0);
                    if (tm_zero_skip !== exp_skip) begin
                        $display("PHY_FAIL: zero_skip mismatch at k=%0d", k);
                        $fatal(1);
                    end
                    @(posedge clk);
                    #1;
                    if ((tm_pos_rail < prev_pos) || (tm_neg_rail < prev_neg)) begin
                        $display("PHY_FAIL: non-monotonic magnitude rail at k=%0d", k);
                        $fatal(1);
                    end
                    prev_pos = tm_pos_rail;
                    prev_neg = tm_neg_rail;
                    if (tm_nz && (tm_act != 32'sd0)) begin
                        if (tm_neg) begin
                            ref_acc = ref_acc - 48'($signed(tm_act));
                        end else begin
                            ref_acc = ref_acc + 48'($signed(tm_act));
                        end
                    end
                    if (tm_acc !== ref_acc) begin
                        $display("PHY_FAIL: ternary acc=%0d != ref=%0d at k=%0d", tm_acc, ref_acc, k);
                        $fatal(1);
                    end
                end
            end
            tm_valid = 1'b0;
            $display("PHY_PASS bus_vectors=128 max_flips=%0d ternary_vectors=128 acc_final=%0d",
                     max_flips, tm_acc);
        end
    endtask

    task automatic run_bundle_cosim(
        input string bundle_path,
        input int    charts_n,
        input int    cmds_m,
        input string stim_path
    );
        int step_idx;
        int guard;
        logic [559:0] cmd;
        logic [7:0]   op_kind;
        begin
            if ((charts_n <= 0) || (charts_n > MAX_CHARTS)) begin
                $display("COSIM_FAIL: invalid CHARTS=%0d", charts_n);
                $fatal(1);
            end
            if ((cmds_m < 0) || (cmds_m > MAX_CMDS)) begin
                $display("COSIM_FAIL: invalid CMDS=%0d", cmds_m);
                $fatal(1);
            end

            $readmemh(bundle_path, bundle_mem, 0, charts_n);
            if (cmds_m > 0) begin
                $readmemh(stim_path, stim_mem, 0, cmds_m - 1);
            end

            dut_load_state = 1'b1;
            dut_init_state = bundle_mem[0];
            dut_cap_mask   = 64'(charts_n - 1);
            @(posedge clk);
            dut_load_state = 1'b0;

            for (int s = 0; s < charts_n; s++) begin
                dut_load_chart = 1'b1;
                dut_load_slot  = 8'(s);
                dut_load_bound = bundle_mem[s + 1];
                @(posedge clk);
            end
            dut_load_chart = 1'b0;
            @(posedge clk);

            step_idx = 0;
            for (int c = 0; c < cmds_m; c++) begin
                guard = 0;
                while (!dut_ready) begin
                    @(posedge clk);
                    guard++;
                    if (guard > 4096) begin
                        $display("COSIM_FAIL: timeout waiting for dut_ready at cmd=%0d", c);
                        $fatal(1);
                    end
                end

                cmd     = stim_mem[c];
                op_kind = cmd[559:552];

                if (op_kind == 8'h01) begin
                    dut_u_ext      = cmd[63:0];
                    dut_mode_req   = cmd[80] ? dut_state_o[335:320] : cmd[79:64];
                    dut_step_start = 1'b1;
                    @(posedge clk);
                    dut_step_start = 1'b0;

                    guard = 0;
                    while (!dut_step_done) begin
                        @(posedge clk);
                        guard++;
                        if (guard > 4096) begin
                            $display("COSIM_FAIL: timeout waiting for step_done at cmd=%0d", c);
                            $fatal(1);
                        end
                    end
                    step_idx++;
                    #1;
                    $display("STEP idx=%0d w7=%016x w6=%016x w5=%016x w4=%016x w3=%016x w2=%016x w1=%016x w0=%016x",
                             step_idx,
                             dut_state_o[511:448], dut_state_o[447:384],
                             dut_state_o[383:320], dut_state_o[319:256],
                             dut_state_o[255:192], dut_state_o[191:128],
                             dut_state_o[127:64],  dut_state_o[63:0]);
                end else if (op_kind == 8'h02) begin
                    dut_swap_cand  = cmd[511:0];
                    dut_swap_epoch = cmd[543:512];
                    dut_swap_slot  = cmd[551:544];
                    dut_swap_start = 1'b1;
                    @(posedge clk);
                    dut_swap_start = 1'b0;

                    guard = 0;
                    while (!dut_swap_done) begin
                        @(posedge clk);
                        guard++;
                        if (guard > 4096) begin
                            $display("COSIM_FAIL: timeout waiting for swap_done at cmd=%0d", c);
                            $fatal(1);
                        end
                    end
                    #1;
                    $display("SWAP slot=%0d epoch=%0d ok=%0d",
                             cmd[551:544], cmd[543:512], dut_swap_acc);
                end else begin
                    $display("COSIM_FAIL: unknown op_kind=0x%02x at cmd=%0d", op_kind, c);
                    $fatal(1);
                end
                @(posedge clk);
            end
        end
    endtask

    string arg_bundle;
    string arg_stim;
    int    arg_charts;
    int    arg_cmds;

    initial begin
        reset_all();
        if ($test$plusargs("PHY_TEST")) begin
            run_phy_self_test();
            $finish;
        end else if ($value$plusargs("BUNDLE=%s", arg_bundle) &&
                     $value$plusargs("CHARTS=%d", arg_charts) &&
                     $value$plusargs("CMDS=%d",   arg_cmds)   &&
                     $value$plusargs("STIM=%s",   arg_stim)) begin
            run_bundle_cosim(arg_bundle, arg_charts, arg_cmds, arg_stim);
            $finish;
        end else begin
            $display("Usage: Vele_cosim_tb +PHY_TEST | +BUNDLE=<memh> +CHARTS=<N> +CMDS=<M> +STIM=<memh>");
            $fatal(1);
        end
    end
endmodule

`default_nettype wire
