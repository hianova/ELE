// SPDX-License-Identifier: MIT
// ele_flow_core.sv — Universal Synthesizable Silicon `.flow` Manifold Engine
//
// Cycle-accurate, 0-ULP bit-identical IEEE 1800 SystemVerilog hardware
// realization of `FLOW v5.0.0` (`flow_step_chart` / `flow_step_atlas` /
// `flow_arena_try_swap`).
//
// Silicon Microarchitecture:
//   • Single shared 64x64->128 unsigned multiplier (serves both Goldilocks
//     F_p = 2^64 - 2^32 + 1 x^7 sponge and all Q48.16 signed multiplications)
//   • Single 32-step shift-subtract `isqrt_u64` unit (`0` tables, `0` FPU)
//   • Single 64-step restoring unsigned divider (`0` combinational div arrays)
//   • Strict `always_comb` / `always_ff` separation (`0` latches in Yosys).

`default_nettype none

module ele_flow_core (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         start_i,
    input  logic         verify_only_i,
    output logic         ready_o,
    input  logic [511:0] state_i,
    input  logic [511:0] bound_i,
    input  logic [63:0]  cap_mask_i,
    input  logic [63:0]  u_ext_i,
    input  logic [15:0]  mode_req_i,
    output logic         done_o,
    output logic         bound_ok_o,
    output logic [511:0] state_o
);
    localparam logic [63:0] FP_MOD          = 64'hFFFFFFFF_00000001;
    localparam logic [63:0] FLOW_GOLDEN_64  = 64'h9E3779B9_7F4A7C15;
    localparam logic [63:0] FLOW_BOUND_SEED = 64'h464C4F57_424E4435;
    localparam logic [63:0] FLOW_STATE_SEED = 64'h464C4F57_53544135;

    localparam logic [4:0] S_IDLE     = 5'd0;
    localparam logic [4:0] S_SPONGE   = 5'd1;
    localparam logic [4:0] S_JMUL_0   = 5'd2;
    localparam logic [4:0] S_JMUL_1   = 5'd3;
    localparam logic [4:0] S_JMUL_2   = 5'd4;
    localparam logic [4:0] S_JMUL_3   = 5'd5;
    localparam logic [4:0] S_JMUL_4   = 5'd6;
    localparam logic [4:0] S_JMUL_5   = 5'd7;
    localparam logic [4:0] S_JMUL_6   = 5'd8;
    localparam logic [4:0] S_ISQRT    = 5'd9;
    localparam logic [4:0] S_DIV_INIT = 5'd10;
    localparam logic [4:0] S_DIV_STEP = 5'd11;
    localparam logic [4:0] S_POST_0   = 5'd12;
    localparam logic [4:0] S_POST_1   = 5'd13;
    localparam logic [4:0] S_POST_2   = 5'd14;
    localparam logic [4:0] S_POST_3   = 5'd15;
    localparam logic [4:0] S_POST_4   = 5'd16;
    localparam logic [4:0] S_POST_5   = 5'd17;
    localparam logic [4:0] S_POST_6   = 5'd18;
    localparam logic [4:0] S_POST_7   = 5'd19;
    localparam logic [4:0] S_DONE     = 5'd20;

    function automatic [63:0] fp_reduce64(input [63:0] x);
        begin
            fp_reduce64 = (x >= FP_MOD) ? (x - FP_MOD) : x;
        end
    endfunction

    function automatic [63:0] fp_add(input [63:0] a, input [63:0] b);
        reg [64:0] sum65;
        reg [63:0] folded;
        begin
            sum65  = {1'b0, a} + {1'b0, b};
            folded = sum65[63:0] + (sum65[64] ? 64'h00000000_FFFFFFFF : 64'd0);
            fp_add = fp_reduce64(folded);
        end
    endfunction

    function automatic [63:0] fp_sub(input [63:0] a, input [63:0] b);
        begin
            fp_sub = (a < b) ? ((a - b) + FP_MOD) : (a - b);
        end
    endfunction

    function automatic [63:0] fp_mul_reduce(input [127:0] prod);
        reg [63:0] z_lo, z_hi, h0, h1, t0, t1, v;
        begin
            z_lo = prod[63:0];
            z_hi = prod[127:64];
            h0   = {32'd0, z_hi[31:0]};
            h1   = {32'd0, z_hi[63:32]};
            t0   = fp_reduce64(z_lo);
            t1   = fp_sub(t0, h1);
            v    = {h0[31:0], 32'd0} - h0;
            fp_mul_reduce = fp_add(t1, v);
        end
    endfunction

    function automatic [64:0] qmul_extract(
        input [63:0]  a,
        input [63:0]  b,
        input [127:0] prod
    );
        reg [63:0] ulo, uhi, hi, shifted, sign_fill;
        reg        ov;
        begin
            ulo       = prod[63:0];
            uhi       = prod[127:64];
            hi        = uhi - (a[63] ? b : 64'd0) - (b[63] ? a : 64'd0);
            shifted   = (ulo >> 16) | (hi << 48);
            sign_fill = shifted[63] ? 64'hFFFFFFFF_FFFFFFFF : 64'd0;
            ov        = (((hi ^ sign_fill) >> 15) != 64'd0);
            qmul_extract = {ov, shifted};
        end
    endfunction

    function automatic [64:0] qadd64(input [63:0] a, input [63:0] b);
        reg [63:0] s;
        reg        ov;
        begin
            s  = a + b;
            ov = (~(a[63] ^ b[63])) & (a[63] ^ s[63]);
            qadd64 = {ov, s};
        end
    endfunction

    function automatic [64:0] qsub64(input [63:0] a, input [63:0] b);
        reg [63:0] d;
        reg        ov;
        begin
            d  = a - b;
            ov = (a[63] ^ b[63]) & (a[63] ^ d[63]);
            qsub64 = {ov, d};
        end
    endfunction

    function automatic [63:0] abs_i64(input [63:0] x);
        begin
            abs_i64 = x[63] ? (64'd0 - x) : x;
        end
    endfunction

    // Registered state & datapath lanes
    logic [4:0]  st_q;
    logic        verify_only_q;
    logic [63:0] s_w_q   [0:7];
    logic [63:0] b_w_q   [0:7];
    logic [63:0] out_w_q [0:7];
    logic [15:0] cap_mask_q;
    logic [63:0] u_ext_q;
    logic [15:0] mode_req_q;
    wire         unused_cap_hi = &{1'b0, cap_mask_i[63:16]};

    // Sponge registers
    logic [1:0]  sp_phase_q;
    logic [2:0]  sp_lane_q;
    logic [2:0]  sp_sub_q;
    logic [63:0] sp_st_q;
    logic [63:0] sp_base_q;
    logic [63:0] sp_acc_q;
    logic [63:0] sp_whi_q;
    logic        state_seal_ok_q;
    logic        bound_seal_ok_q;
    logic        bound_ok_q;

    // 2-Jet & solver registers
    logic        ovf_q;
    logic        op_ovf_q;
    logic        auto_ovf_q;
    logic        post_op_ovf_q;
    logic        push_ovf_q;
    logic [63:0] b0_eff_q;
    logic [63:0] f0_q;
    logic [63:0] x_sq_q;
    logic [63:0] quad_term_q;
    logic [63:0] g0_q;
    logic [63:0] r0_q, r1_q, r2_q;
    logic [63:0] r1_sq_q;
    logic        disc_neg_q;
    logic        disc_ovf_q;
    logic        qcit_ovf_q;

    // isqrt registers
    logic        isqrt_hi_q;
    logic [63:0] isqrt_rem_q;
    logic [63:0] isqrt_root_q;
    logic [5:0]  isqrt_sh_q;
    logic [63:0] d_mag_q;
    logic [63:0] q_cit_q;

    // Restoring 64-bit unsigned divider registers
    logic [1:0]  div_idx_q;
    logic [6:0]  div_cnt_q;
    logic [63:0] div_rem_q;
    logic [63:0] div_quot_q;
    logic [63:0] div_den_q;
    logic        div_pre_ovf_q;
    logic        div_neg_q;
    logic [63:0] root_a_q;
    logic [63:0] root_b_q;
    logic [63:0] step_lin_q;

    // Post-step registers
    logic [63:0] delta_q;
    logic [63:0] dt_res_q;
    logic [63:0] xp_q;
    logic [63:0] xp_sq_q;
    logic [63:0] post_quad_q;
    logic [63:0] j0_op_q;
    logic [63:0] j1_op_q;
    logic [63:0] d_sq_q;
    logic [63:0] push_quad_q;
    logic [63:0] j0_au_q;

    // Combinational datapath wires
    logic [63:0]  b_exit_c, b_enter_c, dt_max_c, u_gain_c, safe_dt_max_c;
    logic         op_en_c;
    logic [63:0]  sp_cur_word_c, sp_t_init_c, sp_final_seal_c;
    logic         sp_b_ok_c;
    logic [63:0]  mul_a_c, mul_b_c;
    logic [127:0] mul_prod_c;
    logic [63:0]  fp_prod_c;
    logic [64:0]  qmul_res_c;

    // Combinational stage results
    logic [64:0] j0_sub_b0_c, j0_sub_f0_c;
    logic [64:0] j3_lq_c, j3_g0_c;
    logic [64:0] j4_g1_c;
    logic        j4_op_ovf_tot_c;
    logic [64:0] j6_two_r0_r2_c, j6_disc_sub_c;
    logic [63:0] j6_disc_c, j6_abs_disc_c;
    logic        j6_hi_r_c;
    logic [63:0] isq_b_bit_c, isq_cand_c, isq_rem_n_c, isq_root_n_c, isq_d_mag_c, isq_signed_d_c;
    logic [64:0] isq_qcit_add_c;

    logic        descend_neg_c;
    logic [64:0] two_r0_c, neg_two_r0_c, neg_qcit_c, neg_r0_c, neg_r1_c, tun_num_c;
    logic        num_ovf_a_c, num_ovf_b_c, num_ovf_l_c, num_ovf_t_c;
    logic [63:0] v_num_c, tun_d_c, abs_r2_c, abs_r2_i64_c;
    logic [63:0] div_num_sel_c, div_den_sel_c;
    logic [63:0] div_anum_c, div_aden_c, div_safe_den_c;
    logic        div_den_zero_c, div_pre_ovf_c;
    logic [64:0] div_r_sh_c, div_r_diff_c;
    logic [63:0] div_r_nx_c, div_q_nx_c, div_clamped_c, div_res_c;

    logic        d_pick_a_c, d_has_r2_c, d_real_roots_c, d_active_c, d_zero_quant_c, d_clamp_c;
    logic        d_quad_ovf_c, d_solver_ovf_c;
    logic [63:0] d_step_quad_c, d_grad_dir_c, d_raw_step_c, d_nz_step_c, d_raw_abs_c;
    logic [64:0] d_neg_dt_c, d_xp_c;
    logic [63:0] d_signed_cap_c, d_delta_c;

    logic [64:0] p2_s1_c, p2_j0_c;
    logic [64:0] p3_j1_c;
    logic [64:0] p6_s1_c, p6_j0_c;
    logic [64:0] p7_j1_c, p7_conv_sub_c;
    logic        p7_push_tot_c, p7_tot_ovf_c, p7_converged_c, p7_kan_ok_c;
    logic        p7_go_lo_c, p7_go_hi_c, p7_seal_ok_c, p7_dom_ok_c, p7_horizon_ok_c, p7_commit_c;
    logic [63:0] p7_j0_next_c, p7_j1_next_c, p7_res_after_c, p7_aux_next_c, p7_omega_next_c;
    logic [15:0] p7_chart_orig_c, p7_mode_orig_c, p7_next_lo_c, p7_next_hi_c, p7_kan_lock_c;
    logic [15:0] p7_mode_proj_c, p7_raw_chart1_c, p7_raw_chart2_c, p7_chart_next_c;
    logic [31:0] p7_req_sieve_c, p7_live_sieve_c, p7_eff_req_c, p7_epoch_next_c;

    always_comb begin
        b_exit_c      = {{32{b_w_q[3][63]}}, b_w_q[3][63:32]};
        b_enter_c     = {{32{b_w_q[3][31]}}, b_w_q[3][31:0]};
        dt_max_c      = {{32{b_w_q[4][63]}}, b_w_q[4][63:32]};
        u_gain_c      = {{32{b_w_q[4][31]}}, b_w_q[4][31:0]};
        safe_dt_max_c = ($signed(dt_max_c) > 64'sd0) ? dt_max_c : 64'd1;
        op_en_c       = (b_w_q[1] != 64'd0) || (b_w_q[2] != 64'd0);

        sp_cur_word_c = 64'd0;
        if (sp_lane_q < 3'd7) begin
            case (sp_phase_q)
                2'd0: sp_cur_word_c = s_w_q[sp_lane_q];
                2'd1: sp_cur_word_c = (sp_lane_q == 3'd6) ? {32'd0, b_w_q[6][31:0]} : b_w_q[sp_lane_q];
                default: sp_cur_word_c = out_w_q[sp_lane_q];
            endcase
        end
        sp_t_init_c = fp_add(sp_st_q, fp_reduce64(sp_cur_word_c));

        mul_a_c = 64'd0;
        mul_b_c = 64'd0;
        case (st_q)
            S_SPONGE: begin
                if (sp_lane_q < 3'd7) begin
                    case (sp_sub_q)
                        3'd0: begin mul_a_c = sp_t_init_c; mul_b_c = sp_t_init_c; end
                        3'd1: begin mul_a_c = sp_acc_q;    mul_b_c = sp_base_q;   end
                        3'd2: begin mul_a_c = sp_acc_q;    mul_b_c = sp_acc_q;    end
                        3'd3: begin mul_a_c = sp_acc_q;    mul_b_c = sp_base_q;   end
                        default: begin mul_a_c = sp_acc_q; mul_b_c = FLOW_GOLDEN_64; end
                    endcase
                end else begin
                    case (sp_sub_q)
                        3'd0: begin mul_a_c = sp_st_q;  mul_b_c = sp_st_q;    end
                        3'd1: begin mul_a_c = sp_acc_q; mul_b_c = sp_base_q;  end
                        3'd2: begin mul_a_c = sp_acc_q; mul_b_c = sp_acc_q;   end
                        default: begin mul_a_c = sp_acc_q; mul_b_c = sp_base_q; end
                    endcase
                end
            end
            S_JMUL_0: begin mul_a_c = u_gain_c; mul_b_c = u_ext_q;  end
            S_JMUL_1: begin mul_a_c = s_w_q[3]; mul_b_c = s_w_q[3]; end
            S_JMUL_2: begin mul_a_c = b_w_q[2]; mul_b_c = x_sq_q;   end
            S_JMUL_3: begin mul_a_c = b_w_q[1]; mul_b_c = s_w_q[3]; end
            S_JMUL_4: begin mul_a_c = b_w_q[2]; mul_b_c = s_w_q[3]; end
            S_JMUL_5: begin mul_a_c = r1_q;     mul_b_c = r1_q;     end
            S_JMUL_6: begin mul_a_c = r0_q;     mul_b_c = r2_q;     end
            S_POST_0: begin mul_a_c = xp_q;     mul_b_c = xp_q;     end
            S_POST_1: begin mul_a_c = b_w_q[2]; mul_b_c = xp_sq_q;  end
            S_POST_2: begin mul_a_c = b_w_q[1]; mul_b_c = xp_q;     end
            S_POST_3: begin mul_a_c = b_w_q[2]; mul_b_c = xp_q;     end
            S_POST_4: begin mul_a_c = delta_q;  mul_b_c = delta_q;  end
            S_POST_5: begin mul_a_c = r2_q;     mul_b_c = d_sq_q;   end
            S_POST_6: begin mul_a_c = r1_q;     mul_b_c = delta_q;  end
            S_POST_7: begin mul_a_c = r2_q;     mul_b_c = delta_q;  end
            default:  begin mul_a_c = 64'd0;    mul_b_c = 64'd0;    end
        endcase

        mul_prod_c      = {64'd0, mul_a_c} * {64'd0, mul_b_c};
        fp_prod_c       = fp_mul_reduce(mul_prod_c);
        qmul_res_c      = qmul_extract(mul_a_c, mul_b_c, mul_prod_c);
        sp_final_seal_c = fp_prod_c | {63'd0, (fp_prod_c == 64'd0)};
        sp_b_ok_c       = (b_w_q[7] != 64'd0) && (b_w_q[7] == sp_final_seal_c);

        // Pre-root 2-Jet combinational paths
        j0_sub_b0_c     = qsub64(b_w_q[0], qmul_res_c[63:0]);
        j0_sub_f0_c     = qsub64(s_w_q[0], j0_sub_b0_c[63:0]);
        j3_lq_c         = qadd64(qmul_res_c[63:0], quad_term_q);
        j3_g0_c         = qadd64(b0_eff_q, j3_lq_c[63:0]);
        j4_g1_c         = qadd64(b_w_q[1], qmul_res_c[63:0]);
        j4_op_ovf_tot_c = op_ovf_q | qmul_res_c[64] | j4_g1_c[64];

        j6_two_r0_r2_c  = qadd64(qmul_res_c[63:0], qmul_res_c[63:0]);
        j6_disc_sub_c   = qsub64(r1_sq_q, j6_two_r0_r2_c[63:0]);
        j6_disc_c       = j6_disc_sub_c[63:0];
        j6_abs_disc_c   = abs_i64(j6_disc_c);
        j6_hi_r_c       = ((j6_abs_disc_c >> 48) != 64'd0);

        // isqrt step
        isq_b_bit_c     = 64'd1 << isqrt_sh_q;
        isq_cand_c      = isqrt_root_q + isq_b_bit_c;
        isq_rem_n_c     = (isqrt_rem_q >= isq_cand_c) ? (isqrt_rem_q - isq_cand_c) : isqrt_rem_q;
        isq_root_n_c    = (isqrt_rem_q >= isq_cand_c) ? ((isqrt_root_q >> 1) + isq_b_bit_c) : (isqrt_root_q >> 1);
        isq_d_mag_c     = isqrt_hi_q ? (isq_root_n_c << 8) : isq_root_n_c;
        isq_signed_d_c  = r1_q[63] ? (64'd0 - isq_d_mag_c) : isq_d_mag_c;
        isq_qcit_add_c  = qadd64(r1_q, isq_signed_d_c);

        // Divider candidate & step
        descend_neg_c   = (r1_q != 64'd0) && (($signed(r0_q) > 64'sd0) == ($signed(r1_q) > 64'sd0));
        two_r0_c        = qadd64(r0_q, r0_q);
        neg_two_r0_c    = qsub64(64'd0, two_r0_c[63:0]);
        num_ovf_a_c     = qcit_ovf_q | two_r0_c[64] | neg_two_r0_c[64];
        neg_qcit_c      = qsub64(64'd0, q_cit_q);
        num_ovf_b_c     = qcit_ovf_q | neg_qcit_c[64];
        neg_r0_c        = qsub64(64'd0, r0_q);
        num_ovf_l_c     = neg_r0_c[64];

        neg_r1_c        = qsub64(64'd0, r1_q);
        v_num_c         = r2_q[63] ? r1_q : neg_r1_c[63:0];
        tun_d_c         = v_num_c[63] ? (64'd0 - d_mag_q) : d_mag_q;
        tun_num_c       = qadd64(v_num_c, tun_d_c);
        num_ovf_t_c     = (r2_q[63] ? 1'b0 : neg_r1_c[64]) | tun_num_c[64];
        abs_r2_c        = abs_i64(r2_q);
        abs_r2_i64_c    = (abs_r2_c > 64'h7FFFFFFF_FFFFFFFF) ? 64'h7FFFFFFF_FFFFFFFF : abs_r2_c;

        case (div_idx_q)
            2'd0: begin
                div_num_sel_c = neg_two_r0_c[63:0];
                div_den_sel_c = q_cit_q;
            end
            2'd1: begin
                div_num_sel_c = neg_qcit_c[63:0];
                div_den_sel_c = r2_q;
            end
            2'd2: begin
                div_num_sel_c = neg_r0_c[63:0];
                div_den_sel_c = r1_q;
            end
            default: begin
                div_num_sel_c = tun_num_c[63:0];
                div_den_sel_c = abs_r2_i64_c;
            end
        endcase

        div_anum_c     = abs_i64(div_num_sel_c);
        div_aden_c     = abs_i64(div_den_sel_c);
        div_den_zero_c = (div_aden_c == 64'd0);
        div_safe_den_c = div_aden_c | (div_den_zero_c ? 64'd1 : 64'd0);
        div_pre_ovf_c  = div_den_zero_c || ((div_anum_c >> 48) >= div_safe_den_c);

        div_r_sh_c     = {div_rem_q, div_quot_q[63]};
        div_r_diff_c   = div_r_sh_c - {1'b0, div_den_q};
        if (div_r_sh_c >= {1'b0, div_den_q}) begin
            div_r_nx_c = div_r_diff_c[63:0] ^ {63'd0, div_r_diff_c[64] & 1'b0};
            div_q_nx_c = {div_quot_q[62:0], 1'b1};
        end else begin
            div_r_nx_c = div_r_sh_c[63:0];
            div_q_nx_c = {div_quot_q[62:0], 1'b0};
        end
        div_clamped_c  = (div_pre_ovf_q || (div_q_nx_c > 64'h7FFFFFFF_FFFFFFFF)) ? 64'h7FFFFFFF_FFFFFFFF : div_q_nx_c;
        div_res_c      = div_neg_q ? (64'd0 - div_clamped_c) : div_clamped_c;

        // Step selection & horizon clamping
        d_pick_a_c     = (abs_i64(root_a_q) <= abs_i64(root_b_q));
        d_step_quad_c  = d_pick_a_c ? root_a_q : root_b_q;
        d_quad_ovf_c   = num_ovf_a_c | num_ovf_b_c;
        d_grad_dir_c   = descend_neg_c ? 64'hFFFFFFFF_FFFFFFFF : 64'd1;
        d_has_r2_c     = (r2_q != 64'd0);
        d_real_roots_c = ~disc_neg_q;
        d_raw_step_c   = d_has_r2_c ? (d_real_roots_c ? d_step_quad_c : div_res_c) : step_lin_q;
        d_solver_ovf_c = d_has_r2_c ? (disc_ovf_q | (d_real_roots_c ? d_quad_ovf_c : num_ovf_t_c)) : num_ovf_l_c;
        d_active_c     = (r0_q != 64'd0) && ((r1_q != 64'd0) || (r2_q != 64'd0));
        d_zero_quant_c = d_active_c && (d_raw_step_c == 64'd0);
        d_nz_step_c    = (~d_active_c) ? 64'd0 : (d_zero_quant_c ? d_grad_dir_c : d_raw_step_c);
        d_raw_abs_c    = abs_i64(d_nz_step_c);
        d_clamp_c      = (d_raw_abs_c > safe_dt_max_c);
        d_neg_dt_c     = qsub64(64'd0, safe_dt_max_c);
        d_signed_cap_c = d_nz_step_c[63] ? d_neg_dt_c[63:0] : safe_dt_max_c;
        d_delta_c      = d_clamp_c ? d_signed_cap_c : d_nz_step_c;
        d_xp_c         = qadd64(s_w_q[3], d_delta_c);

        // Post-step Jet & Omega sieve assembly
        p2_s1_c         = qadd64(qmul_res_c[63:0], post_quad_q);
        p2_j0_c         = qadd64(b0_eff_q, p2_s1_c[63:0]);
        p3_j1_c         = qadd64(b_w_q[1], qmul_res_c[63:0]);
        p6_s1_c         = qadd64(qmul_res_c[63:0], push_quad_q);
        p6_j0_c         = qadd64(s_w_q[0], p6_s1_c[63:0]);
        p7_j1_c         = qadd64(s_w_q[1], qmul_res_c[63:0]);

        p7_push_tot_c   = push_ovf_q | qmul_res_c[64] | p7_j1_c[64];
        p7_tot_ovf_c    = ovf_q | (op_en_c ? post_op_ovf_q : p7_push_tot_c);
        p7_j0_next_c    = op_en_c ? j0_op_q : j0_au_q;
        p7_j1_next_c    = op_en_c ? j1_op_q : p7_j1_c[63:0];
        p7_conv_sub_c   = qsub64(p7_j0_next_c, b0_eff_q);
        p7_res_after_c  = op_en_c ? p7_j0_next_c : p7_conv_sub_c[63:0];
        p7_converged_c  = (~(p7_tot_ovf_c | ((~op_en_c) & p7_conv_sub_c[64]))) && (abs_i64(p7_res_after_c) <= 64'd4);

        p7_chart_orig_c = s_w_q[5][31:16];
        p7_mode_orig_c  = s_w_q[5][15:0];
        p7_next_lo_c    = {8'd0, b_w_q[5][31:24]};
        p7_next_hi_c    = {8'd0, b_w_q[5][23:16]};
        p7_kan_lock_c   = b_w_q[5][15:0];
        p7_req_sieve_c  = b_w_q[6][31:0];

        p7_kan_ok_c     = (((p7_mode_orig_c ^ mode_req_q) & p7_kan_lock_c) == 16'd0);
        p7_mode_proj_c  = (mode_req_q & ~p7_kan_lock_c) | (p7_mode_orig_c & p7_kan_lock_c);

        p7_go_lo_c      = ($signed(xp_q) < $signed(b_exit_c));
        p7_go_hi_c      = ($signed(xp_q) > $signed(b_enter_c));
        p7_raw_chart1_c = p7_go_lo_c ? p7_next_lo_c : p7_chart_orig_c;
        p7_raw_chart2_c = p7_go_hi_c ? p7_next_hi_c : p7_raw_chart1_c;
        p7_chart_next_c = p7_raw_chart2_c & cap_mask_q;

        p7_seal_ok_c    = state_seal_ok_q & bound_seal_ok_q;
        p7_dom_ok_c     = (s_w_q[5][63:32] == b_w_q[5][63:32]);
        p7_horizon_ok_c = ($signed(dt_max_c) > 64'sd0) && ($signed(b_exit_c) <= $signed(b_enter_c));

        p7_live_sieve_c = {26'd0, p7_converged_c, p7_horizon_ok_c, p7_kan_ok_c, ~p7_tot_ovf_c, p7_dom_ok_c, p7_seal_ok_c};
        p7_eff_req_c    = p7_req_sieve_c | 32'h0000_0003;
        p7_commit_c     = ((p7_live_sieve_c & p7_eff_req_c) == p7_eff_req_c);

        p7_aux_next_c   = {s_w_q[5][63:32], p7_chart_next_c, p7_mode_proj_c};
        p7_epoch_next_c = s_w_q[6][63:32] + 32'd1;
        p7_omega_next_c = {p7_epoch_next_c, p7_live_sieve_c};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_q            <= S_IDLE;
            verify_only_q   <= 1'b0;
            cap_mask_q      <= 16'd0;
            u_ext_q         <= 64'd0;
            mode_req_q      <= 16'd0;
            sp_phase_q      <= 2'd0;
            sp_lane_q       <= 3'd0;
            sp_sub_q        <= 3'd0;
            sp_st_q         <= 64'd0;
            sp_base_q       <= 64'd0;
            sp_acc_q        <= 64'd0;
            sp_whi_q        <= 64'd0;
            state_seal_ok_q <= 1'b0;
            bound_seal_ok_q <= 1'b0;
            bound_ok_q      <= 1'b0;
            ovf_q           <= 1'b0;
            op_ovf_q        <= 1'b0;
            auto_ovf_q      <= 1'b0;
            post_op_ovf_q   <= 1'b0;
            push_ovf_q      <= 1'b0;
            b0_eff_q        <= 64'd0;
            f0_q            <= 64'd0;
            x_sq_q          <= 64'd0;
            quad_term_q     <= 64'd0;
            g0_q            <= 64'd0;
            r0_q            <= 64'd0;
            r1_q            <= 64'd0;
            r2_q            <= 64'd0;
            r1_sq_q         <= 64'd0;
            disc_neg_q      <= 1'b0;
            disc_ovf_q      <= 1'b0;
            qcit_ovf_q      <= 1'b0;
            isqrt_hi_q      <= 1'b0;
            isqrt_rem_q     <= 64'd0;
            isqrt_root_q    <= 64'd0;
            isqrt_sh_q      <= 6'd0;
            d_mag_q         <= 64'd0;
            q_cit_q         <= 64'd0;
            div_idx_q       <= 2'd0;
            div_cnt_q       <= 7'd0;
            div_rem_q       <= 64'd0;
            div_quot_q      <= 64'd0;
            div_den_q       <= 64'd1;
            div_pre_ovf_q   <= 1'b0;
            div_neg_q       <= 1'b0;
            root_a_q        <= 64'd0;
            root_b_q        <= 64'd0;
            step_lin_q      <= 64'd0;
            delta_q         <= 64'd0;
            dt_res_q        <= 64'd0;
            xp_q            <= 64'd0;
            xp_sq_q         <= 64'd0;
            post_quad_q     <= 64'd0;
            j0_op_q         <= 64'd0;
            j1_op_q         <= 64'd0;
            d_sq_q          <= 64'd0;
            push_quad_q     <= 64'd0;
            j0_au_q         <= 64'd0;
            for (int i = 0; i < 8; i++) begin
                s_w_q[i]   <= 64'd0;
                b_w_q[i]   <= 64'd0;
                out_w_q[i] <= 64'd0;
            end
        end else begin
            case (st_q)
                S_IDLE: begin
                    if (start_i) begin
                        for (int i = 0; i < 8; i++) begin
                            s_w_q[i] <= state_i[i*64 +: 64];
                            b_w_q[i] <= bound_i[i*64 +: 64];
                        end
                        verify_only_q <= verify_only_i;
                        cap_mask_q    <= cap_mask_i[15:0];
                        u_ext_q       <= u_ext_i;
                        mode_req_q    <= mode_req_i;
                        sp_lane_q     <= 3'd0;
                        sp_sub_q      <= 3'd0;
                        if (verify_only_i) begin
                            sp_phase_q <= 2'd1;
                            sp_st_q    <= FLOW_BOUND_SEED;
                        end else begin
                            sp_phase_q <= 2'd0;
                            sp_st_q    <= FLOW_STATE_SEED;
                        end
                        st_q <= S_SPONGE;
                    end
                end

                S_SPONGE: begin
                    if (sp_lane_q < 3'd7) begin
                        case (sp_sub_q)
                            3'd0: begin
                                sp_base_q <= sp_t_init_c;
                                sp_acc_q  <= fp_prod_c;
                                sp_whi_q  <= (sp_cur_word_c >= FP_MOD) ? 64'd1 : 64'd0;
                                sp_sub_q  <= 3'd1;
                            end
                            3'd1: begin
                                sp_acc_q <= fp_prod_c;
                                sp_sub_q <= 3'd2;
                            end
                            3'd2: begin
                                sp_acc_q <= fp_prod_c;
                                sp_sub_q <= 3'd3;
                            end
                            3'd3: begin
                                sp_acc_q <= fp_prod_c;
                                sp_sub_q <= 3'd4;
                            end
                            default: begin
                                sp_st_q   <= fp_add(fp_prod_c, sp_whi_q + {61'd0, sp_lane_q} + 64'd1);
                                sp_lane_q <= sp_lane_q + 3'd1;
                                sp_sub_q  <= 3'd0;
                            end
                        endcase
                    end else begin
                        case (sp_sub_q)
                            3'd0: begin
                                sp_base_q <= sp_st_q;
                                sp_acc_q  <= fp_prod_c;
                                sp_sub_q  <= 3'd1;
                            end
                            3'd1: begin
                                sp_acc_q <= fp_prod_c;
                                sp_sub_q <= 3'd2;
                            end
                            3'd2: begin
                                sp_acc_q <= fp_prod_c;
                                sp_sub_q <= 3'd3;
                            end
                            default: begin
                                if (sp_phase_q == 2'd0) begin
                                    state_seal_ok_q <= (s_w_q[7] != 64'd0) && (s_w_q[7] == sp_final_seal_c);
                                    sp_phase_q      <= 2'd1;
                                    sp_st_q         <= FLOW_BOUND_SEED;
                                    sp_lane_q       <= 3'd0;
                                    sp_sub_q        <= 3'd0;
                                end else if (sp_phase_q == 2'd1) begin
                                    bound_seal_ok_q <= sp_b_ok_c;
                                    bound_ok_q      <= sp_b_ok_c;
                                    if (verify_only_q) begin
                                        st_q <= S_DONE;
                                    end else begin
                                        st_q <= S_JMUL_0;
                                    end
                                end else begin
                                    out_w_q[7] <= state_seal_ok_q ? sp_final_seal_c : 64'd0;
                                    st_q       <= S_DONE;
                                end
                            end
                        endcase
                    end
                end

                S_JMUL_0: begin
                    b0_eff_q   <= j0_sub_b0_c[63:0];
                    f0_q       <= j0_sub_f0_c[63:0];
                    auto_ovf_q <= j0_sub_f0_c[64];
                    ovf_q      <= qmul_res_c[64] | j0_sub_b0_c[64];
                    st_q       <= S_JMUL_1;
                end

                S_JMUL_1: begin
                    x_sq_q   <= qmul_res_c[63:0];
                    op_ovf_q <= qmul_res_c[64];
                    st_q     <= S_JMUL_2;
                end

                S_JMUL_2: begin
                    quad_term_q <= {qmul_res_c[63], qmul_res_c[63:1]};
                    op_ovf_q    <= op_ovf_q | qmul_res_c[64];
                    st_q        <= S_JMUL_3;
                end

                S_JMUL_3: begin
                    g0_q     <= j3_g0_c[63:0];
                    op_ovf_q <= op_ovf_q | qmul_res_c[64] | j3_lq_c[64] | j3_g0_c[64];
                    st_q     <= S_JMUL_4;
                end

                S_JMUL_4: begin
                    ovf_q <= ovf_q | (op_en_c ? j4_op_ovf_tot_c : auto_ovf_q);
                    r0_q  <= op_en_c ? g0_q          : f0_q;
                    r1_q  <= op_en_c ? j4_g1_c[63:0] : s_w_q[1];
                    r2_q  <= op_en_c ? b_w_q[2]      : s_w_q[2];
                    st_q  <= S_JMUL_5;
                end

                S_JMUL_5: begin
                    r1_sq_q    <= qmul_res_c[63:0];
                    disc_ovf_q <= qmul_res_c[64];
                    st_q       <= S_JMUL_6;
                end

                S_JMUL_6: begin
                    disc_neg_q   <= j6_disc_c[63];
                    disc_ovf_q   <= disc_ovf_q | qmul_res_c[64] | j6_two_r0_r2_c[64] | j6_disc_sub_c[64];
                    isqrt_hi_q   <= j6_hi_r_c;
                    isqrt_rem_q  <= j6_hi_r_c ? j6_abs_disc_c : (j6_abs_disc_c << 16);
                    isqrt_root_q <= 64'd0;
                    isqrt_sh_q   <= 6'd62;
                    st_q         <= S_ISQRT;
                end

                S_ISQRT: begin
                    if (isqrt_sh_q == 6'd0) begin
                        d_mag_q    <= isq_d_mag_c;
                        q_cit_q    <= isq_qcit_add_c[63:0];
                        qcit_ovf_q <= isq_qcit_add_c[64];
                        div_idx_q  <= 2'd0;
                        st_q       <= S_DIV_INIT;
                    end else begin
                        isqrt_rem_q  <= isq_rem_n_c;
                        isqrt_root_q <= isq_root_n_c;
                        isqrt_sh_q   <= isqrt_sh_q - 6'd2;
                    end
                end

                S_DIV_INIT: begin
                    div_quot_q    <= (div_anum_c << 16);
                    div_den_q     <= div_safe_den_c;
                    div_rem_q     <= div_pre_ovf_c ? 64'd0 : (div_anum_c >> 48);
                    div_pre_ovf_q <= div_pre_ovf_c;
                    div_neg_q     <= (div_num_sel_c[63] != div_den_sel_c[63]);
                    div_cnt_q     <= 7'd64;
                    st_q          <= S_DIV_STEP;
                end

                S_DIV_STEP: begin
                    if (div_cnt_q == 7'd1) begin
                        case (div_idx_q)
                            2'd0: begin
                                root_a_q  <= div_res_c;
                                div_idx_q <= 2'd1;
                                st_q      <= S_DIV_INIT;
                            end
                            2'd1: begin
                                root_b_q  <= div_res_c;
                                div_idx_q <= 2'd2;
                                st_q      <= S_DIV_INIT;
                            end
                            2'd2: begin
                                step_lin_q <= div_res_c;
                                div_idx_q  <= 2'd3;
                                st_q       <= S_DIV_INIT;
                            end
                            default: begin
                                delta_q  <= d_delta_c;
                                dt_res_q <= abs_i64(d_delta_c);
                                xp_q     <= d_xp_c[63:0];
                                ovf_q    <= ovf_q | (d_active_c & d_solver_ovf_c) | d_neg_dt_c[64] | d_xp_c[64];
                                st_q     <= S_POST_0;
                            end
                        endcase
                    end else begin
                        div_rem_q  <= div_r_nx_c;
                        div_quot_q <= div_q_nx_c;
                        div_cnt_q  <= div_cnt_q - 7'd1;
                    end
                end

                S_POST_0: begin
                    xp_sq_q       <= qmul_res_c[63:0];
                    post_op_ovf_q <= qmul_res_c[64];
                    st_q          <= S_POST_1;
                end

                S_POST_1: begin
                    post_quad_q   <= {qmul_res_c[63], qmul_res_c[63:1]};
                    post_op_ovf_q <= post_op_ovf_q | qmul_res_c[64];
                    st_q          <= S_POST_2;
                end

                S_POST_2: begin
                    j0_op_q       <= p2_j0_c[63:0];
                    post_op_ovf_q <= post_op_ovf_q | qmul_res_c[64] | p2_s1_c[64] | p2_j0_c[64];
                    st_q          <= S_POST_3;
                end

                S_POST_3: begin
                    j1_op_q       <= p3_j1_c[63:0];
                    post_op_ovf_q <= post_op_ovf_q | qmul_res_c[64] | p3_j1_c[64];
                    st_q          <= S_POST_4;
                end

                S_POST_4: begin
                    d_sq_q     <= qmul_res_c[63:0];
                    push_ovf_q <= qmul_res_c[64];
                    st_q       <= S_POST_5;
                end

                S_POST_5: begin
                    push_quad_q <= {qmul_res_c[63], qmul_res_c[63:1]};
                    push_ovf_q  <= push_ovf_q | qmul_res_c[64];
                    st_q        <= S_POST_6;
                end

                S_POST_6: begin
                    j0_au_q    <= p6_j0_c[63:0];
                    push_ovf_q <= push_ovf_q | qmul_res_c[64] | p6_s1_c[64] | p6_j0_c[64];
                    st_q       <= S_POST_7;
                end

                S_POST_7: begin
                    out_w_q[0] <= p7_commit_c ? p7_j0_next_c : s_w_q[0];
                    out_w_q[1] <= p7_commit_c ? p7_j1_next_c : s_w_q[1];
                    out_w_q[2] <= p7_commit_c ? r2_q         : s_w_q[2];
                    out_w_q[3] <= p7_commit_c ? xp_q         : s_w_q[3];
                    out_w_q[4] <= p7_commit_c ? dt_res_q     : 64'd0;
                    out_w_q[5] <= p7_commit_c ? p7_aux_next_c : s_w_q[5];
                    out_w_q[6] <= p7_omega_next_c;
                    out_w_q[7] <= 64'd0;

                    sp_phase_q <= 2'd2;
                    sp_st_q    <= FLOW_STATE_SEED;
                    sp_lane_q  <= 3'd0;
                    sp_sub_q   <= 3'd0;
                    st_q       <= S_SPONGE;
                end

                S_DONE: begin
                    st_q <= S_IDLE;
                end

                default: st_q <= S_IDLE;
            endcase
        end
    end

    assign ready_o    = (st_q == S_IDLE);
    assign done_o     = (st_q == S_DONE);
    assign bound_ok_o = bound_ok_q;
    assign state_o    = {
        out_w_q[7], out_w_q[6], out_w_q[5], out_w_q[4],
        out_w_q[3], out_w_q[2], out_w_q[1], out_w_q[0]
    };
endmodule

/* verilator lint_off DECLFILENAME */
// Top-level 2^k Atlas Controller with live guarded `@swap` (`flow_arena_try_swap`)
module ele_flow_atlas_top #(
    parameter int MAX_CHARTS = 16
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         load_state_i,
    input  logic [511:0] init_state_i,
    input  logic [63:0]  cap_mask_i,
    input  logic         load_chart_i,
    input  logic [7:0]   load_slot_i,
    input  logic [511:0] load_bound_i,
    input  logic         step_start_i,
    input  logic [63:0]  u_ext_i,
    input  logic [15:0]  mode_req_i,
    input  logic         swap_start_i,
    input  logic [7:0]   swap_slot_i,
    input  logic [31:0]  swap_epoch_i,
    input  logic [511:0] swap_cand_i,
    output logic         ready_o,
    output logic         step_done_o,
    output logic         swap_done_o,
    output logic         swap_accepted_o,
    output logic [511:0] state_o
);
    localparam int SLOT_W = $clog2(MAX_CHARTS);

    logic [511:0] state_q;
    logic [63:0]  cap_mask_q;
    logic [511:0] arena_q [0:MAX_CHARTS-1];

    logic         swap_pending_q;
    logic [7:0]   swap_slot_q;
    logic [31:0]  swap_epoch_q;
    logic [511:0] swap_cand_q;
    logic         swap_done_q;
    logic         swap_acc_q;
    logic         step_done_q;
    wire          unused_top_ok = &{1'b0, load_slot_i[7:SLOT_W], swap_cand_q[447:416]};

    logic         core_start;
    logic         core_verify_only;
    logic         core_ready;
    logic [511:0] core_bound_in;
    logic         core_done;
    logic         core_bound_ok;
    logic [511:0] core_state_out;

    logic [SLOT_W-1:0]      masked_slot_c;
    logic [SLOT_W-1:0]      sw_idx_c;
    logic [31:0]            sw_cur_ep_c, sw_exp_dom_c, sw_cand_dom_c, sw_req_sv_c;
    logic [63:0]            sw_n_lo_c, sw_n_hi_c;
    logic signed [31:0]     sw_b_ex32_c, sw_b_en32_c, sw_dt_m32_c;
    logic                   sw_ok_c;

    always_comb begin
        masked_slot_c    = state_q[336 +: SLOT_W] & cap_mask_q[SLOT_W-1:0];
        core_start       = 1'b0;
        core_verify_only = 1'b0;
        core_bound_in    = arena_q[masked_slot_c];

        if (core_ready && !swap_pending_q) begin
            if (swap_start_i) begin
                core_start       = 1'b1;
                core_verify_only = 1'b1;
                core_bound_in    = swap_cand_i;
            end else if (step_start_i) begin
                core_start       = 1'b1;
                core_verify_only = 1'b0;
                core_bound_in    = arena_q[masked_slot_c];
            end
        end

        sw_idx_c      = swap_slot_q[SLOT_W-1:0] & cap_mask_q[SLOT_W-1:0];
        sw_cur_ep_c   = arena_q[sw_idx_c][447:416];
        sw_exp_dom_c  = state_q[383:352];
        sw_cand_dom_c = swap_cand_q[383:352];
        sw_n_lo_c     = {56'd0, swap_cand_q[351:344]};
        sw_n_hi_c     = {56'd0, swap_cand_q[343:336]};
        sw_b_ex32_c   = $signed(swap_cand_q[255:224]);
        sw_b_en32_c   = $signed(swap_cand_q[223:192]);
        sw_dt_m32_c   = $signed(swap_cand_q[319:288]);
        sw_req_sv_c   = swap_cand_q[415:384];

        sw_ok_c = core_bound_ok &&
                  (sw_cand_dom_c == sw_exp_dom_c) &&
                  (swap_epoch_q > sw_cur_ep_c) &&
                  ({56'd0, swap_slot_q} <= cap_mask_q) &&
                  (sw_n_lo_c <= cap_mask_q) &&
                  (sw_n_hi_c <= cap_mask_q) &&
                  (sw_dt_m32_c > 32'sd0) &&
                  (sw_b_ex32_c <= sw_b_en32_c) &&
                  ((sw_req_sv_c & 32'h0000_0003) == 32'h0000_0003);
    end

    ele_flow_core u_core (
        .clk           (clk),
        .rst_n         (rst_n),
        .start_i       (core_start),
        .verify_only_i (core_verify_only),
        .ready_o       (core_ready),
        .state_i       (state_q),
        .bound_i       (core_bound_in),
        .cap_mask_i    (cap_mask_q),
        .u_ext_i       (u_ext_i),
        .mode_req_i    (mode_req_i),
        .done_o        (core_done),
        .bound_ok_o    (core_bound_ok),
        .state_o       (core_state_out)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q        <= 512'd0;
            cap_mask_q     <= 64'd0;
            swap_pending_q <= 1'b0;
            swap_slot_q    <= 8'd0;
            swap_epoch_q   <= 32'd0;
            swap_cand_q    <= 512'd0;
            swap_done_q    <= 1'b0;
            swap_acc_q     <= 1'b0;
            step_done_q    <= 1'b0;
            for (int i = 0; i < MAX_CHARTS; i++) begin
                arena_q[i] <= 512'd0;
            end
        end else begin
            swap_done_q <= 1'b0;
            step_done_q <= 1'b0;

            if (load_state_i) begin
                state_q    <= init_state_i;
                cap_mask_q <= cap_mask_i;
            end
            if (load_chart_i) begin
                arena_q[load_slot_i[SLOT_W-1:0]] <= load_bound_i;
            end

            if (core_ready && !swap_pending_q && swap_start_i) begin
                swap_pending_q <= 1'b1;
                swap_slot_q    <= swap_slot_i;
                swap_epoch_q   <= swap_epoch_i;
                swap_cand_q    <= swap_cand_i;
            end else if (core_done) begin
                if (swap_pending_q) begin
                    if (sw_ok_c) begin
                        arena_q[sw_idx_c] <= {
                            swap_cand_q[511:448],
                            swap_epoch_q,
                            sw_req_sv_c,
                            swap_cand_q[383:0]
                        };
                    end
                    swap_pending_q <= 1'b0;
                    swap_done_q    <= 1'b1;
                    swap_acc_q     <= sw_ok_c;
                end else begin
                    state_q     <= core_state_out;
                    step_done_q <= 1'b1;
                end
            end
        end
    end

    assign ready_o         = core_ready & ~swap_pending_q;
    assign step_done_o     = step_done_q;
    assign swap_done_o     = swap_done_q;
    assign swap_accepted_o = swap_acc_q;
    assign state_o         = state_q;
endmodule
/* verilator lint_on DECLFILENAME */

`default_nettype wire
