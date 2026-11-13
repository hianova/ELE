#!/usr/bin/env python3
"""
rtl_sabotage_check.py — SystemVerilog Single-Fault RTL Mutation & Kill-Rate Gate for ELE v6.0.0
===============================================================================================
Empirically proves functional & protocol completeness of ELE's pure-SV co-simulation wind-tunnel
by injecting 54 stratified single-fault SystemVerilog hardware mutations across BOTH:
  - `rtl/ele_primordial.sv` (10 leaf-silicon mutants across `ele_mul33_atom`, `ele_rad2_step_atom`,
    and `ele_sram_1rw64`), and
  - `rtl/ele_flow_core.sv` (44 datapath, FSM, reset, Schmitt/KanLock, and `@swap` firewall mutants
    across `ele_flow_core` and `ele_flow_atlas_top`).

Every mutant is required to:
  1. Match uniquely (`count == 1`) in the golden RTL source (0 silent no-op mutants).
  2. Elaborate and compile cleanly into a Verilator executable (`compile_rc == 0`, proving genuine
     semantic RTL fault injection rather than trivial syntax/lint errors).
  3. Be killed dynamically at simulation runtime (`sim_rc != 0`, 100.0% mutant kill rate, 0 survivors).

All mutated builds execute in isolated temporary directories (`tempfile.mkdtemp`) so `ELE/build`
is never modified or polluted.
"""

from __future__ import annotations

import concurrent.futures
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RtlMutant:
    name: str
    category: str
    rel_file: str
    target: str
    replacement: str


def build_rtl_mutants() -> list[RtlMutant]:
    return [
        # --- Category 1: Primordial DSP33 & 1RW SRAM Leaf Atoms (`rtl/ele_primordial.sv`, 4 mutants) ---
        RtlMutant(
            "PRIM_MUL33_NEGATE_B",
            "primordial_mul33_sram",
            "rtl/ele_primordial.sv",
            "iso_b  = en_i ? b_i : 33'sd0;",
            "iso_b  = en_i ? -b_i : 33'sd0;",
        ),
        RtlMutant(
            "PRIM_MUL33_DROP_SIGN_A",
            "primordial_mul33_sram",
            "rtl/ele_primordial.sv",
            "iso_a  = en_i ? a_i : 33'sd0;",
            "iso_a  = en_i ? $signed({1'b0, a_i[31:0]}) : 33'sd0;",
        ),
        RtlMutant(
            "PRIM_MUL33_DROP_SIGN_B",
            "primordial_mul33_sram",
            "rtl/ele_primordial.sv",
            "prod_o = iso_a * iso_b;",
            "prod_o = iso_a * $signed({1'b0, iso_b[31:0]});",
        ),
        RtlMutant(
            "PRIM_SRAM_WE_INVERT",
            "primordial_mul33_sram",
            "rtl/ele_primordial.sv",
            "if (we_i) begin",
            "if (!we_i) begin",
        ),
        # --- Category 2: Primordial Unified Radix-2 `isqrt`/`udiv` Atom (`rtl/ele_primordial.sv`, 6 mutants) ---
        RtlMutant(
            "PRIM_RAD2_DIV_SHIFT_LSB_DROP",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            "r_val   = is_div_i ? {rem_i[62:0], acc_i[63]} : {rem_i[61:0], aux_i[127:126]};",
            "r_val   = is_div_i ? {rem_i[62:0], 1'b0}      : {rem_i[61:0], aux_i[127:126]};",
        ),
        RtlMutant(
            "PRIM_RAD2_ISQRT_TRIAL_CONST",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            "sub_val = is_div_i ? aux_i[63:0]              : {acc_i[61:0], 2'b01};",
            "sub_val = is_div_i ? aux_i[63:0]              : {acc_i[61:0], 2'b11};",
        ),
        RtlMutant(
            "PRIM_RAD2_DIV_OVF_HI_DROP",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            "ovf_hi  = is_div_i ? rem_i[63]                : (rem_i[63:62] != 2'b00);",
            "ovf_hi  = is_div_i ? 1'b0                     : (rem_i[63:62] != 2'b00);",
        ),
        RtlMutant(
            "PRIM_RAD2_ISQRT_OVF_BIT62_DROP",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            ": (rem_i[63:62] != 2'b00);",
            ": (rem_i[63] != 1'b0);",
        ),
        RtlMutant(
            "PRIM_RAD2_BORROW_POLARITY_FLIP",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            "ge      = ovf_hi | ~diff[64];",
            "ge      = ovf_hi | diff[64];",
        ),
        RtlMutant(
            "PRIM_RAD2_RESTORE_MUX_FLIP",
            "primordial_rad2",
            "rtl/ele_primordial.sv",
            "rem_o   = ge ? diff[63:0] : r_val;",
            "rem_o   = (~ge) ? diff[63:0] : r_val;",
        ),
        # --- Category 3: FSM Handshake, Busy Write-Ignore & Mid-Flight Reset (`rtl/ele_flow_core.sv`, 10 mutants) ---
        RtlMutant(
            "FSM_CORE_READY_PREMATURE",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "assign ready_o   = (st_q == 4'd0);",
            "assign ready_o   = (st_q <= 4'd1);",
        ),
        RtlMutant(
            "FSM_CORE_WR_VALID_UNCHECKED",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "if (wr_valid_i) begin",
            "if (1'b1) begin",
        ),
        RtlMutant(
            "FSM_CORE_BUSY_WRITE_STATE_LEAK",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "done_q <= 1'b0;\n            case (st_q)",
            "done_q <= 1'b0;\n            if (wr_valid_i && (st_q != 4'd0) && (wr_kind_i == 2'd0)) state_w_q[wr_idx_i] <= wr_data_i;\n            case (st_q)",
        ),
        RtlMutant(
            "FSM_CORE_FAST_PATH_INVERT",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "if (!n_a10_active_c) begin",
            "if (n_a10_active_c) begin",
        ),
        RtlMutant(
            "FSM_CORE_LINEAR_BYPASS_INVERT",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "end else if (!n_a10_has_r2_c) begin",
            "end else if (n_a10_has_r2_c) begin",
        ),
        RtlMutant(
            "FSM_CORE_ISQRT_ITER_OFF1",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "sr_aux_q <= $unsigned(n_b8_disc_c);\n                        iter_q   <= 5'd31;",
            "sr_aux_q <= $unsigned(n_b8_disc_c);\n                        iter_q   <= 5'd30;",
        ),
        RtlMutant(
            "FSM_CORE_RST_ST_NONIDLE",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "st_q         <= 4'd0;\n            iter_q       <= 5'd0;",
            "st_q         <= 4'd2;\n            iter_q       <= 5'd0;",
        ),
        RtlMutant(
            "FSM_CORE_RST_STATE_UNCLEARED",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "state_w_q    <= 512'd0;",
            "state_w_q    <= {8{64'hDEAD_BEEF_CAFE_BABE}};",
        ),
        RtlMutant(
            "FSM_TOP_IDLE_HANDSHAKE_BYPASS",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "if (cmd_valid_i && core_ready_w) begin\n                        case (cmd_op_i)\n                            CMD_SET_CAP:",
            "if (1'b1) begin\n                        case (cmd_op_i)\n                            CMD_SET_CAP:",
        ),
        RtlMutant(
            "FSM_TOP_RST_SLOT_VALID_STUCK1",
            "fsm_handshake_reset",
            "rtl/ele_flow_core.sv",
            "slot_valid_q <= {MAX_CHARTS{1'b0}};",
            "slot_valid_q <= {MAX_CHARTS{1'b1}};",
        ),
        # --- Category 4: 33-Bit Cross-Limb Carry & `2^-120` Exact Rounding (`rtl/ele_flow_core.sv`, 10 mutants) ---
        RtlMutant(
            "LIMB_G1_LO_FALSE_SIGN_EXT",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "dsp_a_c[0]  = {1'b0, g1_q[31:0]};",
            "dsp_a_c[0]  = {g1_q[31], g1_q[31:0]};",
        ),
        RtlMutant(
            "LIMB_G0_MID_FALSE_SIGN_EXT",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "dsp_a_c[1]  = {1'b0, g0_q[63:32]};",
            "dsp_a_c[1]  = {g0_q[63], g0_q[63:32]};",
        ),
        RtlMutant(
            "LIMB_G1_CROSS_SHIFT_32_VS_33",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "($signed({{62{n_b1_g1_hl_c[65]}}, n_b1_g1_hl_c}) <<< 33) +",
            "($signed({{62{n_b1_g1_hl_c[65]}}, n_b1_g1_hl_c}) <<< 32) +",
        ),
        RtlMutant(
            "LIMB_DISC_ADD_VS_SUB",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "n_b8_disc_c   = n_b6_g1_sq_q - n_b7_g0_r2_q;",
            "n_b8_disc_c   = n_b6_g1_sq_q + n_b7_g0_r2_q;",
        ),
        RtlMutant(
            "ROUND_ISQRT_GT_TO_GE",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "d_mag_q <= rad2_acc_w[2] + {63'd0, (rad2_rem_w[2] > rad2_acc_w[2])};",
            "d_mag_q <= rad2_acc_w[2] + {63'd0, (rad2_rem_w[2] >= rad2_acc_w[2])};",
        ),
        RtlMutant(
            "ROUND_ISQRT_DROP_ROUND_BIT",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "(rad2_rem_w[2] > rad2_acc_w[2])",
            "1'b0",
        ),
        RtlMutant(
            "ROUND_UDIV_HALFWAY_GT_TO_GE",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "round_up  = (~pre_sat_q) & (sr_rem_q > ((sr_aux_q[63:0] - 64'd1) >> 1));",
            "round_up  = (~pre_sat_q) & (sr_rem_q >= ((sr_aux_q[63:0] - 64'd1) >> 1));",
        ),
        RtlMutant(
            "ROUND_UDIV_OVER_CAP_GE_TO_GT",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "over_cap  = pre_sat_q | (sr_acc_q >= cap64);",
            "over_cap  = pre_sat_q | (sr_acc_q > cap64);",
        ),
        RtlMutant(
            "ROUND_SYM_SHR128_33_TRUNC",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "mag = 64'((abs128(v) + 128'h0000_0000_0000_0000_0000_0001_0000_0000) >> 33);",
            "mag = 64'(abs128(v) >> 33);",
        ),
        RtlMutant(
            "ROUND_SYM_SHR64_16_TRUNC",
            "datapath_limb_carry_round",
            "rtl/ele_flow_core.sv",
            "mag = ((v[63] ? (64'd0 - $unsigned(v)) : $unsigned(v)) + 64'h0000_0000_0000_8000) >> 16;",
            "mag = (v[63] ? (64'd0 - $unsigned(v)) : $unsigned(v)) >> 16;",
        ),
        # --- Category 5: 128-Bit Citardauq, Lyapunov & `2^-49` Jet Predicates (`rtl/ele_flow_core.sv`, 11 mutants) ---
        RtlMutant(
            "CIT_GAUGE_SIGN_AT_G1_ZERO",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "d_neg            = (g1_q < 64'sd0) | ((g1_q == 64'sd0) & (r2_q < 32'sd0));",
            "d_neg            = (g1_q < 64'sd0);",
        ),
        RtlMutant(
            "CIT_QCIT_CANCELLATION_BRANCH",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "q_cit66          = $signed({{2{g1_q[63]}}, g1_q}) + $signed({{2{signed_d[63]}}, signed_d});",
            "q_cit66          = $signed({{2{g1_q[63]}}, g1_q}) - $signed({{2{signed_d[63]}}, signed_d});",
        ),
        RtlMutant(
            "CIT_TUNNEL_NUM_MUX_INVERT",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "n_d0_div_num_c   = (has_r2 & (~real_roots)) ? (128'sd0 - $signed({{64{g1_q[63]}}, g1_q})) : neg_g0;",
            "n_d0_div_num_c   = (has_r2 & real_roots) ? (128'sd0 - $signed({{64{g1_q[63]}}, g1_q})) : neg_g0;",
        ),
        RtlMutant(
            "CIT_DIV_NEG_XOR_TO_AND",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "n_d0_div_neg_c   = n_d0_div_num_c[127] ^ div_den66[65];",
            "n_d0_div_neg_c   = n_d0_div_num_c[127] & div_den66[65];",
        ),
        RtlMutant(
            "CIT_PRE_SAT_GE_TO_GT",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "n_d1_pre_sat_c   = den_zero | (anum128[127:64] >= n_d1_safe_den_c);",
            "n_d1_pre_sat_c   = den_zero | (anum128[127:64] > n_d1_safe_den_c);",
        ),
        RtlMutant(
            "LYAP_ISOMETRIC_LE_TO_LT",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "lyap_ok         = (~fast_path_q) & (abs128(n_c7_g0_cand_q) <= abs128(g0_q));",
            "lyap_ok         = (~fast_path_q) & (abs128(n_c7_g0_cand_q) < abs128(g0_q));",
        ),
        RtlMutant(
            "LYAP_MONOTONICITY_DISABLED",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "(abs128(n_c7_g0_cand_q) <= abs128(g0_q))",
            "1'b1",
        ),
        RtlMutant(
            "JET_SUB_VALID_AND_TO_OR",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "sub_valid     = ($signed(state_w_q[0]) == j0_in) & ($signed(state_w_q[1]) == j1_in);",
            "sub_valid     = ($signed(state_w_q[0]) == j0_in) | ($signed(state_w_q[1]) == j1_in);",
        ),
        RtlMutant(
            "JET_CLAMP_SYM32_INT32_MIN_LEAK",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "clamp_sym32 = (raw == 32'h8000_0000) ? -32'sh7FFF_FFFF : $signed(raw);",
            "clamp_sym32 = $signed(raw);",
        ),
        RtlMutant(
            "JET_CONV_Q49_LE_TO_LT",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "n_c10_conv_c       = (abs128(n_c8_g0_post_c) <= 128'h0000_0000_0000_0000_0000_0008_0000_0000);",
            "n_c10_conv_c       = (abs128(n_c8_g0_post_c) < 128'h0000_0000_0000_0000_0000_0008_0000_0000);",
        ),
        RtlMutant(
            "JET_CONV_Q49_PLUS_1ULP",
            "citardauq_lyap_jet",
            "rtl/ele_flow_core.sv",
            "128'h0000_0000_0000_0000_0000_0008_0000_0000",
            "128'h0000_0000_0000_0000_0000_0008_0000_0001",
        ),
        # --- Category 6: Schmitt Hysteresis & Cubical Kan Mode Lock Gluing (`rtl/ele_flow_core.sv`, 5 mutants) ---
        RtlMutant(
            "SCHMITT_INVERTED_BAND_UNCLAMPED",
            "schmitt_kan_gluing",
            "rtl/ele_flow_core.sv",
            "b_enter_q <= (en_c < ex_c) ? ex_c : en_c;",
            "b_enter_q <= en_c;",
        ),
        RtlMutant(
            "SCHMITT_ENTER_GT_TO_GE",
            "schmitt_kan_gluing",
            "rtl/ele_flow_core.sv",
            "raw_chart          = (n_c8_xp_c > b_enter_q) ? {8'd0, next_hi_q}",
            "raw_chart          = (n_c8_xp_c >= b_enter_q) ? {8'd0, next_hi_q}",
        ),
        RtlMutant(
            "SCHMITT_EXIT_LT_TO_LE",
            "schmitt_kan_gluing",
            "rtl/ele_flow_core.sv",
            ": ((n_c8_xp_c < b_exit_q) ? {8'd0, next_lo_q} : state_w_q[5][31:16]);",
            ": ((n_c8_xp_c <= b_exit_q) ? {8'd0, next_lo_q} : state_w_q[5][31:16]);",
        ),
        RtlMutant(
            "KAN_LOCK_MASK_INVERT",
            "schmitt_kan_gluing",
            "rtl/ele_flow_core.sv",
            "n_c10_mode_proj_c  = (mode_req_q & (~kan_lock_q)) | (state_w_q[5][15:0] & kan_lock_q);",
            "n_c10_mode_proj_c  = (mode_req_q & kan_lock_q) | (state_w_q[5][15:0] & (~kan_lock_q));",
        ),
        RtlMutant(
            "CHART_CAP_MASK_BYPASS",
            "schmitt_kan_gluing",
            "rtl/ele_flow_core.sv",
            "n_c10_chart_next_c = raw_chart & cap_mask16_q;",
            "n_c10_chart_next_c = raw_chart;",
        ),
        # --- Category 7: Release-Ordered `@swap` Epoch & Geometry Firewall (`rtl/ele_flow_core.sv`, 8 mutants) ---
        RtlMutant(
            "SWAP_FW_EPOCH_GT_TO_GE",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "(swap_epoch_q > swap_cur_ep_c) &",
            "(swap_epoch_q >= swap_cur_ep_c) &",
        ),
        RtlMutant(
            "SWAP_FW_NEXT_LO_CAP_OFF1",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "({8'd0, cand_buf_q[5][31:24]} <= cap_mask_q) &",
            "({8'd0, cand_buf_q[5][31:24]} <= (cap_mask_q + 16'd1)) &",
        ),
        RtlMutant(
            "SWAP_FW_NEXT_LO_LE_TO_LT",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "({8'd0, cand_buf_q[5][31:24]} <= cap_mask_q) &",
            "({8'd0, cand_buf_q[5][31:24]} < cap_mask_q) &",
        ),
        RtlMutant(
            "SWAP_FW_NEXT_HI_CAP_OFF1",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "({8'd0, cand_buf_q[5][23:16]} <= cap_mask_q) &",
            "({8'd0, cand_buf_q[5][23:16]} <= (cap_mask_q + 16'd1)) &",
        ),
        RtlMutant(
            "SWAP_FW_NEXT_HI_LE_TO_LT",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "({8'd0, cand_buf_q[5][23:16]} <= cap_mask_q) &",
            "({8'd0, cand_buf_q[5][23:16]} < cap_mask_q) &",
        ),
        RtlMutant(
            "SWAP_FW_DT_MAX_GT_TO_GE",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "($signed(cand_buf_q[4][63:32]) > 32'sd0) &",
            "($signed(cand_buf_q[4][63:32]) >= 32'sd0) &",
        ),
        RtlMutant(
            "SWAP_FW_HYSTERESIS_INVERT",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "($signed(cand_buf_q[3][63:32]) <= $signed(cand_buf_q[3][31:0]));",
            "($signed(cand_buf_q[3][63:32]) >= $signed(cand_buf_q[3][31:0]));",
        ),
        RtlMutant(
            "SWAP_COMMIT_EPOCH_LANE_SHIFT",
            "swap_firewall_commit",
            "rtl/ele_flow_core.sv",
            "sram_wdata_c = {swap_epoch_q, 32'd0};",
            "sram_wdata_c = {32'd0, swap_epoch_q};",
        ),
    ]


def run_verilator_sim(
    verilator: str,
    env: dict[str, str],
    mdir: Path,
    phy_sv: Path,
    prim_sv: Path,
    core_sv: Path,
    tb_sv: Path,
    flow_obj: Path,
    fast_mode: bool,
) -> tuple[bool, bool, int, str]:
    """Compiles and executes a mutated RTL model inside `mdir`.

    Returns `(compiled_ok, killed, rc, detail)`.
    """
    # Force clean recompilation of all DUT object files while preserving Verilator runtime/PCH in `mdir`
    for stale in mdir.glob("Vele_cosim_tb*"):
        if stale.suffix in (".o", ".a") or stale.name == "Vele_cosim_tb_mut":
            stale.unlink(missing_ok=True)

    cmd = [
        verilator,
        "--binary",
        "-j",
        "1",
        "-Wno-UNUSEDSIGNAL",
        "--x-assign",
        "unique",
        "--x-initial",
        "unique",
        "-CFLAGS",
        "-O0",
        "-Mdir",
        str(mdir),
        "--top-module",
        "ele_cosim_tb",
    ]
    if fast_mode:
        cmd.append("-DRTL_MUTANT_FAST")
    cmd.extend(
        [
            str(phy_sv),
            str(prim_sv),
            str(core_sv),
            str(tb_sv),
            str(flow_obj),
            "-o",
            "Vele_cosim_tb_mut",
        ]
    )

    bld = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=120)
    mut_bin = mdir / "Vele_cosim_tb_mut"
    if bld.returncode != 0 or not mut_bin.exists():
        return (False, False, bld.returncode, (bld.stderr or bld.stdout)[-400:])

    try:
        sim = subprocess.run([str(mut_bin)], capture_output=True, text=True, env=env, timeout=20)
        killed = sim.returncode != 0
        return (True, killed, sim.returncode, "")
    except subprocess.TimeoutExpired:
        # Infinite loop / FSM deadlock caused by mutant is a genuine dynamic kill
        return (True, True, -9, "TIMEOUT_DEADLOCK")


def eval_rtl_mutant(
    m: RtlMutant,
    sources: dict[str, str],
    root: Path,
    flow_obj: Path,
    verilator: str,
    env: dict[str, str],
    worker_q: queue.Queue[int],
    tmpdir: Path,
) -> tuple[RtlMutant, bool, bool, int, str]:
    wid = worker_q.get()
    try:
        wdir = tmpdir / f"worker_{wid}"
        mdir = wdir / "mdir_fast"
        mdir.mkdir(parents=True, exist_ok=True)

        prim_text = sources["rtl/ele_primordial.sv"]
        core_text = sources["rtl/ele_flow_core.sv"]
        if m.rel_file == "rtl/ele_primordial.sv":
            prim_text = prim_text.replace(m.target, m.replacement, 1)
        else:
            core_text = core_text.replace(m.target, m.replacement, 1)

        prim_path = wdir / "ele_primordial.sv"
        core_path = wdir / "ele_flow_core.sv"
        prim_path.write_text(prim_text, encoding="utf-8")
        core_path.write_text(core_text, encoding="utf-8")

        phy_sv = root / "rtl" / "ele_em_phy.sv"
        tb_sv = root / "rtl" / "ele_cosim_tb.sv"

        compiled_ok, killed, rc, detail = run_verilator_sim(
            verilator, env, mdir, phy_sv, prim_path, core_path, tb_sv, flow_obj, fast_mode=True
        )
        if not compiled_ok:
            return (m, False, False, rc, detail)

        # Fallback: if a subtle mutant survives the fast first-pass filter, run the FULL un-shortened wind-tunnel
        if not killed:
            mdir_full = wdir / f"mdir_full_{m.name}"
            mdir_full.mkdir(parents=True, exist_ok=True)
            compiled_ok, killed, rc, detail = run_verilator_sim(
                verilator, env, mdir_full, phy_sv, prim_path, core_path, tb_sv, flow_obj, fast_mode=False
            )

        return (m, compiled_ok, killed, rc, detail)
    finally:
        worker_q.put(wid)


def main() -> int:
    t0 = time.monotonic()
    root = Path(__file__).resolve().parent.parent
    env = os.environ.copy()
    extra_paths = ["/opt/homebrew/opt/llvm/bin", "/opt/homebrew/bin", "/usr/local/bin"]
    env["PATH"] = ":".join(extra_paths + [env.get("PATH", "")])
    verilator = env.get("VERILATOR", "verilator")

    canon_bin = root / "build" / "verilator_canon" / "Vele_cosim_tb"
    area_bin = root / "build" / "verilator_area" / "Vele_cosim_tb_area"
    flow_obj = root / "build" / "flow_kernel.o"

    if not (canon_bin.exists() and area_bin.exists() and flow_obj.exists()):
        print("ERROR: Missing Verilator/FLOW build artifacts; run `make cosim-gate` first.", file=sys.stderr)
        return 1

    # 1. Verify BOTH unmutated golden Pareto profiles pass (`exit 0`)
    for label, bin_path in [("Canonical (dsp=3)", canon_bin), ("Area (dsp=1)", area_bin)]:
        base = subprocess.run([str(bin_path)], capture_output=True, text=True, env=env)
        if base.returncode != 0:
            print(f"ERROR: Unmutated golden {label} failed wind-tunnel!", file=sys.stderr)
            print(base.stdout[-1000:])
            print(base.stderr[-1000:], file=sys.stderr)
            return 1

    sources = {
        "rtl/ele_primordial.sv": (root / "rtl" / "ele_primordial.sv").read_text(encoding="utf-8"),
        "rtl/ele_flow_core.sv": (root / "rtl" / "ele_flow_core.sv").read_text(encoding="utf-8"),
    }

    mutants = build_rtl_mutants()

    # 2. Strict uniqueness check on every mutant target (zero silent no-ops allowed!)
    seen_names: set[str] = set()
    for m in mutants:
        if m.name in seen_names:
            raise RuntimeError(f"Duplicate mutant name: {m.name}")
        seen_names.add(m.name)
        src = sources[m.rel_file]
        cnt = src.count(m.target)
        if cnt != 1:
            raise RuntimeError(
                f"Mutant {m.name} target in {m.rel_file} matched {cnt} times (expected exactly 1): {m.target!r}"
            )
        if m.target == m.replacement:
            raise RuntimeError(f"Mutant {m.name} replacement is identical to target!")

    prim_cnt = sum(1 for m in mutants if m.rel_file == "rtl/ele_primordial.sv")
    core_cnt = sum(1 for m in mutants if m.rel_file == "rtl/ele_flow_core.sv")

    print("================================================================================")
    print("  ELE v6.0.0 — Adversarial SystemVerilog Single-Fault RTL Sabotage Audit")
    print("================================================================================")
    print("  golden_profiles_status : PASS (Canonical dsp=3 & Area dsp=1 both exit 0)")
    print(f"  adversarial_rtl_mutants: {len(mutants)} single-fault SV mutants ({prim_cnt} primordial + {core_cnt} core/top)")

    tmpdir = Path(tempfile.mkdtemp(prefix="ele_rtl_sabotage_"))
    compile_errors: list[tuple[RtlMutant, str]] = []
    survivors: list[RtlMutant] = []
    by_cat: dict[str, list[bool]] = {}

    try:
        workers = min(8, os.cpu_count() or 4)
        worker_q: queue.Queue[int] = queue.Queue()
        for wid in range(workers):
            worker_q.put(wid)

        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
            futs = [
                pool.submit(
                    eval_rtl_mutant, m, sources, root, flow_obj.resolve(), verilator, env, worker_q, tmpdir
                )
                for m in mutants
            ]
            for fut in concurrent.futures.as_completed(futs):
                m, compiled_ok, killed, _, detail = fut.result()
                by_cat.setdefault(m.category, []).append(compiled_ok and killed)
                if not compiled_ok:
                    compile_errors.append((m, detail))
                elif not killed:
                    survivors.append(m)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    for cat in sorted(by_cat):
        res = by_cat[cat]
        print(f"  cat_{cat:<25}: {sum(res):2d} / {len(res):2d} killed dynamically")

    killed_total = len(mutants) - len(compile_errors) - len(survivors)
    pct = (100.0 * killed_total / len(mutants)) if mutants else 0.0
    elapsed = time.monotonic() - t0
    print(f"\n  rtl_mutant_kill_rate   : {killed_total} / {len(mutants)} ({pct:.1f}%) in {elapsed:.1f}s")

    if compile_errors:
        print("================================================================================")
        print("  [MUTATION AUDIT ERROR] MUTANTS FAILED TO COMPILE (INVALID FAULT INJECTION!):")
        for m, det in compile_errors:
            print(f"    - {m.name} ({m.rel_file}): {det}")
        print("================================================================================")
        return 1

    if survivors:
        print("================================================================================")
        print("  [MUTATION AUDIT FAIL] SURVIVING RTL MUTANTS DETECTED (VERIFICATION BLIND SPOT!):")
        for s in survivors:
            print(f"    - {s.name} ({s.rel_file} [{s.category}]): {s.target!r} -> {s.replacement!r}")
        print("================================================================================")
        return 1

    print("================================================================================")
    print("  [RTL MUTATION AUDIT PASS] 100.0% DYNAMIC KILL RATE — ZERO SURVIVING MUTANTS")
    print("================================================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
