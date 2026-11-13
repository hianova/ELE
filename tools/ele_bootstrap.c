/* SPDX-License-Identifier: MIT
 * tools/ele_bootstrap.c — Self-Hosting FLOW v6.0.0 2-Jet Silicon Synthesizer
 *
 * Build-time hardware DAG scheduler, pipeline-cut retimer, and IEEE 1800
 * SystemVerilog emitter driven directly by `../FLOW/flow.h` (`flow_step_atlas`).
 *
 * Zero external library dependencies (C17 libc + `../FLOW/flow.h` only).
 * Given physical silicon constraints (`--dsp-count M`, `--max-depth D`),
 * constructs a coupled 64-byte `flow_cell` lattice over all 33 operator nodes
 * of the decomposed 33x33-limb FLOW 2-Jet manifold plus the radix-2 unroll
 * controller, evolves `flow_step_atlas` to `FLOW_STATUS_CONVERGED`, and emits
 * synthesizable `< 160`-pin SystemVerilog (`ele_flow_core` + `ele_flow_atlas_top`)
 * targeting the primordial leaf atoms in `rtl/ele_primordial.sv`.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flow.h"

#define MAX_CONE_NODES 16
#define MAX_PREDS      6
#define MAX_DSPS       8

typedef enum {
    PHASE_PRE_LOGIC  = 0,
    PHASE_DSP33      = 1,
    PHASE_POST_LOGIC = 2
} node_phase_t;

typedef struct {
    const char  *name;
    node_phase_t phase;
    int          delay_lvls;
    int          reg_bits;
    int          num_preds;
    int          preds[MAX_PREDS];
    /* Solved by FLOW 2-Jet lattice (`flow_step_atlas`) */
    int          stage;
    int          end_frac_q16;
    int          dsp_slot;
    bool         needs_reg;
    uint32_t     flow_epochs;
    uint16_t     final_chart;
} dag_node_t;

typedef struct {
    const char *cone_name;
    int         num_nodes;
    dag_node_t  nodes[MAX_CONE_NODES];
    int         num_stages;
    int         sweeps_taken;
    int         cut_reg_bits;
} dag_cone_t;

/* Quantize Q16.16 coordinate upward to multiples of 1024 (`2^10`) so that
 * quadratic 2-Jet curvature `b2 = 8192 (2^13)` satisfies `(x_req^2 >> 20) << 33
 * == b2 * x_req^2` to exact `2^-49` guard-bit precision (`0-ULP` root). */
static int64_t quantize_q16_up(int64_t v) {
    if (v <= 0) {
        return 0;
    }
    return ((v + 1023) / 1024) * 1024;
}

static int64_t quadratic_drive_for_root(int64_t x_root_q16) {
    /* Root of: -(u_in << 33) + (x << 33) + 8192 * x^2 == 0
     * => u_in = x_root + ((x_root * x_root) >> 20) */
    int64_t sq = x_root_q16 * x_root_q16;
    return x_root_q16 + (sq >> 20U);
}

static void init_2jet_node_arena(flow_cell arena[2], bool pin_anchor) {
    /* Chart 0: Stage-0 combinational packing basin (`x <= 1.0` cycle)
     * Chart 1: Multi-cycle pipeline-cut basin (`x > 1.0` cycle), entered via
     *          HoTT Schmitt hysteresis (`b_exit = 0.5`, `b_enter = 1.0`). */
    const uint64_t b0_c0[8] = {
        0ULL,
        (uint64_t)FLOW_Q16_ONE,
        8192ULL, /* Non-zero positive 2-Jet curvature -> exercises Citardauq + isqrt! */
        ((uint64_t)(uint32_t)(-32768) << 32U) | (uint64_t)(uint32_t)65536,
        ((uint64_t)(uint32_t)98304 << 32U) | (uint64_t)(uint32_t)FLOW_Q16_ONE,
        (0ULL << 24U) | (1ULL << 16U) | (uint64_t)(pin_anchor ? 0xFFFFU : 0x0001U),
        0ULL,
        0ULL,
    };
    const uint64_t b0_c1[8] = {
        0ULL,
        (uint64_t)FLOW_Q16_ONE,
        8192ULL,
        ((uint64_t)(uint32_t)32768 << 32U) | (uint64_t)(uint32_t)0x7FFFFFFF,
        ((uint64_t)(uint32_t)131072 << 32U) | (uint64_t)(uint32_t)FLOW_Q16_ONE,
        (0ULL << 24U) | (1ULL << 16U) | (uint64_t)(pin_anchor ? 0xFFFFU : 0x0003U),
        0ULL,
        0ULL,
    };
    flow_arena_publish_bound(arena, 0U, 1U, 1U, b0_c0);
    flow_arena_publish_bound(arena, 1U, 1U, 1U, b0_c1);
}

static int solve_radix_unroll_2jet(int max_depth, uint32_t *out_epochs) {
    /* Each chained `ele_rad2_step_atom` slice costs 13 gate levels + 5 mux overhead.
     * Legal power-of-two unrolls: K_rad in {1, 2, 4}. */
    int target_k = 1;
    if (5 + 4 * 13 <= max_depth) {
        target_k = 4;
    } else if (5 + 2 * 13 <= max_depth) {
        target_k = 2;
    }

    flow_cell arena[2];
    flow_cell state = {0ULL, 0ULL, 0ULL, 0ULL, 0ULL, 0x0001ULL, 0ULL, 0ULL};
    init_2jet_node_arena(arena, false);

    int64_t x_target = (int64_t)target_k * FLOW_Q16_ONE;
    int64_t u_drive  = quadratic_drive_for_root(x_target);

    for (int iter = 0; iter < 64; ++iter) {
        flow_cell next_s;
        flow_step_atlas(next_s, state, arena, 1ULL, u_drive, 0x0007U);
        memcpy(state, next_s, sizeof(flow_cell));
        if ((state[6] & FLOW_STATUS_CONVERGED) != 0ULL && (int64_t)state[3] == x_target) {
            break;
        }
    }
    if ((state[6] & FLOW_STATUS_CONVERGED) == 0ULL) {
        fprintf(stderr, "FATAL: FLOW 2-Jet radix unroll cell failed to converge!\n");
        exit(1);
    }
    *out_epochs = (uint32_t)(state[6] >> 32U);
    int resolved_k = (int)(((int64_t)state[3] + 32768) / FLOW_Q16_ONE);
    if (resolved_k != 1 && resolved_k != 2 && resolved_k != 4) {
        fprintf(stderr, "FATAL: invalid converged radix unroll K=%d\n", resolved_k);
        exit(1);
    }
    return resolved_k;
}

static void solve_cone_with_flow_2jet(dag_cone_t *cone, int dsp_count, int max_depth) {
    flow_cell cells[MAX_CONE_NODES];
    flow_cell arenas[MAX_CONE_NODES][2];

    for (int i = 0; i < cone->num_nodes; ++i) {
        memset(cells[i], 0, sizeof(flow_cell));
        cells[i][5] = (i == 0) ? 0x0005ULL : 0x0001ULL;
        init_2jet_node_arena(arenas[i], (i == 0 && cone->nodes[i].num_preds == 0));
    }

    bool all_converged = false;
    int  sweep         = 0;

    for (sweep = 1; sweep <= 128; ++sweep) {
        int dsp_occ[64];
        memset(dsp_occ, 0, sizeof(dsp_occ));
        all_converged = true;

        for (int i = 0; i < cone->num_nodes; ++i) {
            dag_node_t *nd = &cone->nodes[i];
            int64_t dur_q16 = quantize_q16_up(((int64_t)nd->delay_lvls * FLOW_Q16_ONE) / max_depth);
            if (dur_q16 > FLOW_Q16_ONE) {
                dur_q16 = FLOW_Q16_ONE;
            }
            if (dur_q16 < 1024) {
                dur_q16 = 1024;
            }

            /* 1. Evaluate causal timing & max-depth pipeline-cut bounds from predecessors */
            int64_t req_x = 0;
            for (int p = 0; p < nd->num_preds; ++p) {
                int         u      = nd->preds[p];
                dag_node_t *pu     = &cone->nodes[u];
                int64_t     xu     = (int64_t)cells[u][3];
                if (xu < 0) {
                    xu = 0;
                }
                int64_t su         = xu / FLOW_Q16_ONE;
                int64_t frac_u     = xu % FLOW_Q16_ONE;
                int64_t dur_u      = quantize_q16_up(((int64_t)pu->delay_lvls * FLOW_Q16_ONE) / max_depth);
                if (dur_u > FLOW_Q16_ONE) {
                    dur_u = FLOW_Q16_ONE;
                }
                int64_t end_frac_u = frac_u + dur_u;

                /* Can `(u -> nd)` chain combinationally inside clock stage `su`?
                 * Forbidden if:
                 *   (a) Both `u` and `nd` are 33x33 Booth DSP atoms (`DSP -> DSP`), or
                 *   (b) `pu->phase > nd->phase` (violates PRE -> DSP -> POST stratification), or
                 *   (c) Accumulated gate depth `end_frac_u + dur_q16 > 1.0` (`> max_depth`). */
                bool can_chain = !(pu->phase == PHASE_DSP33 && nd->phase == PHASE_DSP33) &&
                                 (pu->phase <= nd->phase) &&
                                 ((end_frac_u + dur_q16) <= FLOW_Q16_ONE);

                int64_t cand_x = can_chain ? (su * FLOW_Q16_ONE + end_frac_u)
                                           : ((su + 1) * FLOW_Q16_ONE);
                if (cand_x > req_x) {
                    req_x = cand_x;
                }
            }

            /* 2. Enforce physical DSP33 contention capacity (`dsp_count`) per stage */
            int cand_stage = (int)(req_x / FLOW_Q16_ONE);
            if (nd->phase == PHASE_DSP33) {
                while (cand_stage < 63 && dsp_occ[cand_stage] >= dsp_count) {
                    ++cand_stage;
                    req_x = (int64_t)cand_stage * FLOW_Q16_ONE;
                }
            }

            req_x = quantize_q16_up(req_x);
            if ((req_x % FLOW_Q16_ONE) + dur_q16 > FLOW_Q16_ONE) {
                ++cand_stage;
                req_x = (int64_t)cand_stage * FLOW_Q16_ONE;
                if (nd->phase == PHASE_DSP33) {
                    while (cand_stage < 63 && dsp_occ[cand_stage] >= dsp_count) {
                        ++cand_stage;
                        req_x = (int64_t)cand_stage * FLOW_Q16_ONE;
                    }
                }
            }

            /* 3. Advance node `i` via `flow_step_atlas` (`../FLOW/flow.h`) */
            int64_t   u_drive = quadratic_drive_for_root(req_x);
            flow_cell next_c;
            flow_step_atlas(next_c, cells[i], arenas[i], 1ULL, u_drive, 0x000FU);
            memcpy(cells[i], next_c, sizeof(flow_cell));

            int cur_stage = (int)(((int64_t)cells[i][3] < 0 ? 0 : (int64_t)cells[i][3]) / FLOW_Q16_ONE);
            if (nd->phase == PHASE_DSP33 && cur_stage >= 0 && cur_stage < 64) {
                dsp_occ[cur_stage]++;
            }

            bool node_conv = ((cells[i][6] & FLOW_STATUS_CONVERGED) != 0ULL) &&
                             ((int64_t)cells[i][3] == req_x);
            if (!node_conv) {
                all_converged = false;
            }
        }

        if (all_converged && sweep >= 2) {
            break;
        }
    }

    if (!all_converged) {
        fprintf(stderr, "FATAL: FLOW 2-Jet lattice did not converge on cone '%s'!\n", cone->cone_name);
        exit(1);
    }

    /* Extract discrete stage schedule, verify 0 constraint violations, bind DSP slots */
    int stage_dsp_next[64];
    memset(stage_dsp_next, 0, sizeof(stage_dsp_next));
    int max_stage = 0;

    for (int i = 0; i < cone->num_nodes; ++i) {
        dag_node_t *nd = &cone->nodes[i];
        int64_t     xi = (int64_t)cells[i][3];
        nd->stage         = (int)(xi / FLOW_Q16_ONE);
        nd->end_frac_q16  = (int)(xi % FLOW_Q16_ONE);
        nd->flow_epochs   = (uint32_t)(cells[i][6] >> 32U);
        nd->final_chart   = (uint16_t)((cells[i][5] >> 16U) & 0xFFFFU);
        nd->needs_reg     = false;
        if (nd->stage > max_stage) {
            max_stage = nd->stage;
        }
        if (nd->phase == PHASE_DSP33) {
            nd->dsp_slot = stage_dsp_next[nd->stage]++;
            if (nd->dsp_slot >= dsp_count) {
                fprintf(stderr, "FATAL: DSP overflow in cone '%s' stage %d!\n",
                        cone->cone_name, nd->stage);
                exit(1);
            }
        } else {
            nd->dsp_slot = -1;
        }

        /* Verify causal precedence */
        for (int p = 0; p < nd->num_preds; ++p) {
            int u = nd->preds[p];
            if (cone->nodes[u].stage > nd->stage) {
                fprintf(stderr, "FATAL: causal violation %s (s=%d) -> %s (s=%d)\n",
                        cone->nodes[u].name, cone->nodes[u].stage, nd->name, nd->stage);
                exit(1);
            }
            if (cone->nodes[u].stage < nd->stage) {
                cone->nodes[u].needs_reg = true;
            }
        }
    }

    /* Compact any empty stage gaps while preserving strict order */
    int used[64] = {0};
    for (int i = 0; i < cone->num_nodes; ++i) {
        used[cone->nodes[i].stage] = 1;
    }
    int remap[64] = {0};
    int compact_s = 0;
    for (int s = 0; s <= max_stage; ++s) {
        if (used[s]) {
            remap[s] = compact_s++;
        }
    }
    cone->cut_reg_bits = 0;
    for (int i = 0; i < cone->num_nodes; ++i) {
        cone->nodes[i].stage     = remap[cone->nodes[i].stage];
        cone->nodes[i].needs_reg = false;
    }
    for (int i = 0; i < cone->num_nodes; ++i) {
        for (int p = 0; p < cone->nodes[i].num_preds; ++p) {
            int u = cone->nodes[i].preds[p];
            if (cone->nodes[u].stage < cone->nodes[i].stage) {
                cone->nodes[u].needs_reg = true;
            }
        }
    }
    for (int i = 0; i < cone->num_nodes; ++i) {
        if (cone->nodes[i].needs_reg) {
            cone->cut_reg_bits += cone->nodes[i].reg_bits;
        }
    }
    cone->num_stages   = compact_s;
    cone->sweeps_taken = sweep;
}

static void add_node(dag_cone_t *c, const char *name, node_phase_t phase,
                     int delay_lvls, int reg_bits, int np,
                     int p0, int p1, int p2, int p3) {
    int idx = c->num_nodes++;
    dag_node_t *n = &c->nodes[idx];
    memset(n, 0, sizeof(*n));
    n->name       = name;
    n->phase      = phase;
    n->delay_lvls = delay_lvls;
    n->reg_bits   = reg_bits;
    n->num_preds  = np;
    n->preds[0]   = p0;
    n->preds[1]   = p1;
    n->preds[2]   = p2;
    n->preds[3]   = p3;
}

static void build_flow_2jet_cones(dag_cone_t *cone_a, dag_cone_t *cone_b,
                                  dag_cone_t *cone_d, dag_cone_t *cone_c) {
    memset(cone_a, 0, sizeof(*cone_a));
    cone_a->cone_name = "CONE_A_RESIDUAL";
    /*  0 */ add_node(cone_a, "a0_ug_u",      PHASE_DSP33,      17,  64, 0, 0, 0, 0, 0);
    /*  1 */ add_node(cone_a, "a1_b1_x",      PHASE_DSP33,      17,  64, 0, 0, 0, 0, 0);
    /*  2 */ add_node(cone_a, "a2_x_sq",      PHASE_DSP33,      17,  64, 0, 0, 0, 0, 0);
    /*  3 */ add_node(cone_a, "a3_b2_x",      PHASE_DSP33,      17,  64, 0, 0, 0, 0, 0);
    /*  4 */ add_node(cone_a, "a4_b0_eff",    PHASE_POST_LOGIC, 10, 128, 1, 0, 0, 0, 0);
    /*  5 */ add_node(cone_a, "a5_b2xsq_lo",  PHASE_DSP33,      17,  66, 1, 2, 0, 0, 0);
    /*  6 */ add_node(cone_a, "a6_b2xsq_hi",  PHASE_DSP33,      17,  66, 1, 2, 0, 0, 0);
    /*  7 */ add_node(cone_a, "a7_g0_op",     PHASE_POST_LOGIC, 13, 128, 4, 1, 4, 5, 6);
    /*  8 */ add_node(cone_a, "a8_g1_op",     PHASE_POST_LOGIC,  8,  64, 1, 3, 0, 0, 0);
    /*  9 */ add_node(cone_a, "a9_jet_au",    PHASE_POST_LOGIC, 11, 192, 1, 4, 0, 0, 0);
    /* 10 */ add_node(cone_a, "a10_jet_mux",  PHASE_POST_LOGIC,  5,   0, 4, 4, 7, 8, 9);
    /* Architectural outputs of Cone A (`b0_eff` from A4 is latched at end of Cone A if A4 < A10) */

    memset(cone_b, 0, sizeof(*cone_b));
    cone_b->cone_name = "CONE_B_DISC";
    /*  0 */ add_node(cone_b, "b0_g1_ll",     PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  1 */ add_node(cone_b, "b1_g1_hl",     PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  2 */ add_node(cone_b, "b2_g1_hh",     PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  3 */ add_node(cone_b, "b3_g0r2_0",    PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  4 */ add_node(cone_b, "b4_g0r2_1",    PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  5 */ add_node(cone_b, "b5_g0r2_2",    PHASE_DSP33,      17,  66, 0, 0, 0, 0, 0);
    /*  6 */ add_node(cone_b, "b6_g1_sq",     PHASE_POST_LOGIC, 12, 128, 3, 0, 1, 2, 0);
    /*  7 */ add_node(cone_b, "b7_g0_r2",     PHASE_POST_LOGIC, 12, 128, 3, 3, 4, 5, 0);
    /*  8 */ add_node(cone_b, "b8_disc_sub",  PHASE_POST_LOGIC, 11,   0, 2, 6, 7, 0, 0);

    memset(cone_d, 0, sizeof(*cone_d));
    cone_d->cone_name = "CONE_D_CITARDAUQ";
    /*  0 */ add_node(cone_d, "d0_cit_sel",   PHASE_POST_LOGIC, 14, 194, 0, 0, 0, 0, 0);
    /*  1 */ add_node(cone_d, "d1_div_norm",  PHASE_POST_LOGIC, 14,   0, 1, 0, 0, 0, 0);

    memset(cone_c, 0, sizeof(*cone_c));
    cone_c->cone_name = "CONE_C_LYAP_COMMIT";
    /*  0 */ add_node(cone_c, "c0_delta_cand", PHASE_PRE_LOGIC,  13,  33, 0, 0, 0, 0, 0);
    /*  1 */ add_node(cone_c, "c1_d_sq",       PHASE_DSP33,      17,  64, 1, 0, 0, 0, 0);
    /*  2 */ add_node(cone_c, "c2_g1dc_lo",    PHASE_DSP33,      17,  66, 1, 0, 0, 0, 0);
    /*  3 */ add_node(cone_c, "c3_g1dc_hi",    PHASE_DSP33,      17,  66, 1, 0, 0, 0, 0);
    /*  4 */ add_node(cone_c, "c4_r2_dc",      PHASE_DSP33,      17,  64, 1, 0, 0, 0, 0);
    /*  5 */ add_node(cone_c, "c5_r2dsq_lo",   PHASE_DSP33,      17,  66, 1, 1, 0, 0, 0);
    /*  6 */ add_node(cone_c, "c6_r2dsq_hi",   PHASE_DSP33,      17,  66, 1, 1, 0, 0, 0);
    /*  7 */ add_node(cone_c, "c7_g0_cand",    PHASE_POST_LOGIC, 13, 128, 4, 2, 3, 5, 6);
    /*  8 */ add_node(cone_c, "c8_lyap_guard", PHASE_POST_LOGIC, 14, 288, 3, 0, 4, 7, 0);
    /*  9 */ add_node(cone_c, "c9_post_round", PHASE_POST_LOGIC, 13, 192, 1, 8, 0, 0, 0);
    /* 10 */ add_node(cone_c, "c10_kan_commit",PHASE_POST_LOGIC,  7,   0, 2, 8, 9, 0, 0);
}

/* Helper: return SV identifier read by consumer node `dst` from producer `src`
 * within the same cone (`_c` if same stage, `_q` if across a pipeline cut). */
static const char *ref_sig(const dag_cone_t *c, int src, int dst,
                           const char *comb_id, const char *reg_id) {
    return (c->nodes[src].stage == c->nodes[dst].stage) ? comb_id : reg_id;
}

typedef struct {
    int cone_id; /* 0=A, 1=B, 2=C */
    int node_idx;
    int global_state;
    const char *op_a_expr;
    const char *op_b_expr;
} dsp_bind_t;

static void emit_sv(FILE *fp, int dsp_count, int max_depth, int k_rad, uint32_t rad_epochs,
                    const dag_cone_t *ca, const dag_cone_t *cb,
                    const dag_cone_t *cd, const dag_cone_t *cc) {
    /* Assign sequential FSM states cleanly:
     *   0                               : ST_IDLE
     *   1 .. ca->num_stages             : ST_CA_0 .. ST_CA_{na-1}
     *   base_cb .. base_cb + nb - 1     : ST_CB_0 .. ST_CB_{nb-1}
     *   st_isqrt                        : ST_ISQRT_LOOP
     *   base_cd .. base_cd + nd - 1     : ST_CD_0 .. ST_CD_{nd-1}
     *   st_udiv                         : ST_UDIV_LOOP
     *   base_cc .. base_cc + nc - 1     : ST_CC_0 .. ST_CC_{nc-1}
     *     (Tickless `!active` fast path sets `fast_path_q=1` and jumps directly
     *      from Cone A's final stage to `base_cc + cc->nodes[9].stage`, sharing
     *      Cone C's rounding & Kan commit hardware with 0 duplicate gates.)
     */
    const int st_idle   = 0;
    const int base_ca   = 1;
    const int base_cb   = base_ca + ca->num_stages;
    const int st_isqrt  = base_cb + cb->num_stages;
    const int base_cd   = st_isqrt + 1;
    const int st_udiv   = base_cd + cd->num_stages;
    const int base_cc   = st_udiv + 1;
    const int num_st    = base_cc + cc->num_stages;
    int       st_w      = 1;
    while ((1 << st_w) < num_st) {
        ++st_w;
    }

    const int rad_iters = 64 / k_rad;
    int       iter_w    = 1;
    while ((1 << iter_w) < rad_iters) {
        ++iter_w;
    }

    const int quad_cycles = ca->num_stages + cb->num_stages + rad_iters +
                            cd->num_stages + rad_iters + cc->num_stages;
    const int lin_cycles  = ca->num_stages + cd->num_stages + rad_iters + cc->num_stages;
    const int idle_cycles = ca->num_stages + (cc->num_stages - cc->nodes[9].stage);
    const int total_cut_bits = ca->cut_reg_bits + cb->cut_reg_bits +
                               cd->cut_reg_bits + cc->cut_reg_bits;

    fprintf(fp, "// SPDX-License-Identifier: MIT\n");
    fprintf(fp, "// AUTO-GENERATED BY `tools/ele_bootstrap.c` VIA `FLOW v%s` (`flow_step_atlas`)\n",
            FLOW_VERSION_STRING);
    fprintf(fp, "// DO NOT HAND-EDIT — Re-synthesize via `./build/ele_bootstrap`.\n");
    fprintf(fp, "//\n");
    fprintf(fp, "// Physical Pareto Constraints & Converged FLOW 2-Jet Lattice Schedule:\n");
    fprintf(fp, "//   * dsp_count (`ele_mul33_atom` instances) : %d\n", dsp_count);
    fprintf(fp, "//   * max_depth (target gate lvls / cycle)   : %d\n", max_depth);
    fprintf(fp, "//   * k_rad (`ele_rad2_step_atom` chain)     : %d (%d cyc/loop, %u FLOW epochs)\n",
            k_rad, rad_iters, rad_epochs);
    fprintf(fp, "//   * Cone A / B / D / C pipeline stages     : %d / %d / %d / %d (%d FSM states)\n",
            ca->num_stages, cb->num_stages, cd->num_stages, cc->num_stages, num_st);
    fprintf(fp, "//   * FLOW lattice sweeps (A/B/D/C)          : %d / %d / %d / %d (100%% CONVERGED)\n",
            ca->sweeps_taken, cb->sweeps_taken, cd->sweeps_taken, cc->sweeps_taken);
    fprintf(fp, "//   * Inserted pipeline-cut DFF bits         : %d bits\n", total_cut_bits);
    fprintf(fp, "//   * Step latency (quad / linear / idle)    : %d / %d / %d cycles\n",
            quad_cycles, lin_cycles, idle_cycles);
    fprintf(fp, "//   * Top-level physical pin count           : ele_flow_core=141, ele_flow_atlas_top=157 (<160)\n\n");

    fprintf(fp, "`default_nettype none\n");
    fprintf(fp, "/* verilator lint_off DECLFILENAME */\n\n");

    /* ======================================================================
     * MODULE 1: `ele_flow_core` (141 physical pins, 64-bit streaming bus)
     * ====================================================================== */
    fprintf(fp, "module ele_flow_core (\n");
    fprintf(fp, "    input  logic        clk,\n");
    fprintf(fp, "    input  logic        rst_n,\n");
    fprintf(fp, "    input  logic        wr_valid_i,\n");
    fprintf(fp, "    input  logic [1:0]  wr_kind_i,\n");
    fprintf(fp, "    input  logic [2:0]  wr_idx_i,\n");
    fprintf(fp, "    input  logic [63:0] wr_data_i,\n");
    fprintf(fp, "    output logic        ready_o,\n");
    fprintf(fp, "    output logic        done_o,\n");
    fprintf(fp, "    input  logic [2:0]  rd_idx_i,\n");
    fprintf(fp, "    output logic [63:0] rd_data_o\n");
    fprintf(fp, ");\n");
    /* Pure branchless combinational helpers matching `flow.h` */
    fprintf(fp, "    function automatic logic signed [31:0] clamp_sym64_to_32(input logic [63:0] raw);\n");
    fprintf(fp, "        logic signed [63:0] sv;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            sv = $signed(raw);\n");
    fprintf(fp, "            if (sv > 64'sh0000_0000_7FFF_FFFF)      clamp_sym64_to_32 = 32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else if (sv < -64'sh0000_0000_7FFF_FFFF) clamp_sym64_to_32 = -32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else                                    clamp_sym64_to_32 = raw[31:0];\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [31:0] clamp_sym32(input logic [31:0] raw);\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            clamp_sym32 = (raw == 32'h8000_0000) ? -32'sh7FFF_FFFF : $signed(raw);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [63:0] clamp_sym64(\n");
    fprintf(fp, "        input logic signed [63:0] v,\n");
    fprintf(fp, "        input logic signed [63:0] lim\n");
    fprintf(fp, "    );\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            if (v > lim)       clamp_sym64 = lim;\n");
    fprintf(fp, "            else if (v < -lim) clamp_sym64 = -lim;\n");
    fprintf(fp, "            else               clamp_sym64 = v;\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic [127:0] abs128(input logic signed [127:0] v);\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            abs128 = v[127] ? (128'd0 - $unsigned(v)) : $unsigned(v);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [63:0] sym_shr128_33(input logic signed [127:0] v);\n");
    fprintf(fp, "        logic [63:0] mag;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            mag = 64'((abs128(v) + 128'h0000_0000_0000_0000_0000_0001_0000_0000) >> 33);\n");
    fprintf(fp, "            sym_shr128_33 = v[127] ? $signed(64'd0 - mag) : $signed(mag);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [63:0] sym_shr64_16(input logic signed [63:0] v);\n");
    fprintf(fp, "        logic [63:0] mag;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            mag = ((v[63] ? (64'd0 - $unsigned(v)) : $unsigned(v)) + 64'h0000_0000_0000_8000) >> 16;\n");
    fprintf(fp, "            sym_shr64_16 = v[63] ? $signed(64'd0 - mag) : $signed(mag);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    /* Architectural state & clamped 32-bit ingress registers */
    fprintf(fp, "    logic [%d:0]         st_q;\n", st_w - 1);
    fprintf(fp, "    logic [%d:0]         iter_q;\n", iter_w - 1);
    fprintf(fp, "    logic                done_q;\n");
    fprintf(fp, "    logic [7:0][63:0]    state_w_q;\n");
    fprintf(fp, "    logic signed [31:0]  b0_q, b1_q, b2_q;\n");
    fprintf(fp, "    logic signed [31:0]  b_exit_q, b_enter_q;\n");
    fprintf(fp, "    logic signed [31:0]  dt_max_q, u_gain_q, u_in_q;\n");
    fprintf(fp, "    logic [7:0]          next_lo_q, next_hi_q;\n");
    fprintf(fp, "    logic [15:0]         kan_lock_q, cap_mask16_q, mode_req_q;\n\n");

    /* Cross-cone architectural registers */
    fprintf(fp, "    logic                op_mask_q;\n");
    fprintf(fp, "    logic                fast_path_q;\n");
    fprintf(fp, "    logic signed [31:0]  x_clamped_q;\n");
    fprintf(fp, "    logic signed [127:0] b0_eff_q;\n");
    fprintf(fp, "    logic signed [127:0] g0_q;\n");
    fprintf(fp, "    logic signed [63:0]  g1_q;\n");
    fprintf(fp, "    logic signed [31:0]  r2_q;\n");
    fprintf(fp, "    logic [63:0]         d_mag_q;\n");
    fprintf(fp, "    logic                div_neg_q;\n");
    fprintf(fp, "    logic                pre_sat_q;\n");
    fprintf(fp, "    logic [63:0]         sr_rem_q;\n");
    fprintf(fp, "    logic [63:0]         sr_acc_q;\n");
    fprintf(fp, "    logic [127:0]        sr_aux_q;\n\n");

    /* Emit combinational wires & FLOW-scheduled pipeline cut registers per node */
    fprintf(fp, "    // --- Cone A Wires & FLOW Pipeline Cut Registers ---\n");
    fprintf(fp, "    logic signed [63:0]  n_a0_ug_u_c%s;\n", ca->nodes[0].needs_reg ? ", n_a0_ug_u_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_a1_b1_x_c%s;\n", ca->nodes[1].needs_reg ? ", n_a1_b1_x_q" : "");
    fprintf(fp, "    logic [63:0]         n_a2_x_sq_c%s;\n", ca->nodes[2].needs_reg ? ", n_a2_x_sq_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_a3_b2_x_c%s;\n", ca->nodes[3].needs_reg ? ", n_a3_b2_x_q" : "");
    fprintf(fp, "    logic signed [127:0] n_a4_b0_eff_c%s;\n", ca->nodes[4].needs_reg ? ", n_a4_b0_eff_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_a5_b2xsq_lo_c%s;\n", ca->nodes[5].needs_reg ? ", n_a5_b2xsq_lo_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_a6_b2xsq_hi_c%s;\n", ca->nodes[6].needs_reg ? ", n_a6_b2xsq_hi_q" : "");
    fprintf(fp, "    logic signed [127:0] n_a7_g0_op_c%s;\n", ca->nodes[7].needs_reg ? ", n_a7_g0_op_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_a8_g1_op_c%s;\n", ca->nodes[8].needs_reg ? ", n_a8_g1_op_q" : "");
    fprintf(fp, "    logic signed [127:0] n_a9_g0_au_c%s;\n", ca->nodes[9].needs_reg ? ", n_a9_g0_au_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_a9_j1_q32_c%s;\n", ca->nodes[9].needs_reg ? ", n_a9_j1_q32_q" : "");
    fprintf(fp, "    logic                n_a10_op_mask_c, n_a10_active_c, n_a10_has_r2_c;\n");
    fprintf(fp, "    logic signed [127:0] n_a10_g0_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_a10_g1_c;\n");
    fprintf(fp, "    logic signed [31:0]  n_a10_r2_c;\n\n");

    fprintf(fp, "    // --- Cone B Wires & FLOW Pipeline Cut Registers ---\n");
    fprintf(fp, "    logic signed [65:0]  n_b0_g1_ll_c%s;\n", cb->nodes[0].needs_reg ? ", n_b0_g1_ll_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_b1_g1_hl_c%s;\n", cb->nodes[1].needs_reg ? ", n_b1_g1_hl_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_b2_g1_hh_c%s;\n", cb->nodes[2].needs_reg ? ", n_b2_g1_hh_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_b3_g0r2_0_c%s;\n", cb->nodes[3].needs_reg ? ", n_b3_g0r2_0_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_b4_g0r2_1_c%s;\n", cb->nodes[4].needs_reg ? ", n_b4_g0r2_1_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_b5_g0r2_2_c%s;\n", cb->nodes[5].needs_reg ? ", n_b5_g0r2_2_q" : "");
    fprintf(fp, "    logic signed [127:0] n_b6_g1_sq_c%s;\n", cb->nodes[6].needs_reg ? ", n_b6_g1_sq_q" : "");
    fprintf(fp, "    logic signed [127:0] n_b7_g0_r2_c%s;\n", cb->nodes[7].needs_reg ? ", n_b7_g0_r2_q" : "");
    fprintf(fp, "    logic signed [127:0] n_b8_disc_c;\n");
    fprintf(fp, "    logic                n_b8_pos_nz_c;\n\n");

    fprintf(fp, "    // --- Cone D Wires & FLOW Pipeline Cut Registers ---\n");
    fprintf(fp, "    logic signed [127:0] n_d0_div_num_c%s;\n", cd->nodes[0].needs_reg ? ", n_d0_div_num_q" : "");
    fprintf(fp, "    logic        [64:0]  n_d0_div_den_c%s;\n", cd->nodes[0].needs_reg ? ", n_d0_div_den_q" : "");
    fprintf(fp, "    logic                n_d0_div_neg_c%s;\n", cd->nodes[0].needs_reg ? ", n_d0_div_neg_q" : "");
    fprintf(fp, "    logic [63:0]         n_d1_safe_den_c, n_d1_init_rem_c, n_d1_init_quot_c;\n");
    fprintf(fp, "    logic                n_d1_pre_sat_c;\n\n");

    fprintf(fp, "    // --- Cone C Wires & FLOW Pipeline Cut Registers ---\n");
    fprintf(fp, "    logic signed [32:0]  n_c0_dc_c%s;\n", cc->nodes[0].needs_reg ? ", n_c0_dc_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_c1_dsq_c%s;\n", cc->nodes[1].needs_reg ? ", n_c1_dsq_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_c2_g1dc_lo_c%s;\n", cc->nodes[2].needs_reg ? ", n_c2_g1dc_lo_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_c3_g1dc_hi_c%s;\n", cc->nodes[3].needs_reg ? ", n_c3_g1dc_hi_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_c4_r2_dc_c%s;\n", cc->nodes[4].needs_reg ? ", n_c4_r2_dc_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_c5_r2dsq_lo_c%s;\n", cc->nodes[5].needs_reg ? ", n_c5_r2dsq_lo_q" : "");
    fprintf(fp, "    logic signed [65:0]  n_c6_r2dsq_hi_c%s;\n", cc->nodes[6].needs_reg ? ", n_c6_r2dsq_hi_q" : "");
    fprintf(fp, "    logic signed [127:0] n_c7_g0_cand_c%s;\n", cc->nodes[7].needs_reg ? ", n_c7_g0_cand_q" : "");
    fprintf(fp, "    logic signed [127:0] n_c8_g0_post_c%s;\n", cc->nodes[8].needs_reg ? ", n_c8_g0_post_q" : "");
    fprintf(fp, "    logic signed [63:0]  n_c8_g1_post_c%s;\n", cc->nodes[8].needs_reg ? ", n_c8_g1_post_q" : "");
    fprintf(fp, "    logic signed [31:0]  n_c8_xp_c%s;\n", cc->nodes[8].needs_reg ? ", n_c8_xp_q" : "");
    fprintf(fp, "    logic [63:0]         n_c8_dt_res_c%s;\n", cc->nodes[8].needs_reg ? ", n_c8_dt_res_q" : "");
    fprintf(fp, "    logic [63:0]         n_c9_j0_next_c%s;\n", cc->nodes[9].needs_reg ? ", n_c9_j0_next_q" : "");
    fprintf(fp, "    logic [63:0]         n_c9_j1_next_c%s;\n", cc->nodes[9].needs_reg ? ", n_c9_j1_next_q" : "");
    fprintf(fp, "    logic [63:0]         n_c9_sub_next_c%s;\n", cc->nodes[9].needs_reg ? ", n_c9_sub_next_q" : "");
    fprintf(fp, "    logic [15:0]         n_c10_chart_next_c, n_c10_mode_proj_c;\n");
    fprintf(fp, "    logic                n_c10_conv_c;\n\n");

    /* Phase 0 (`PRE_LOGIC`): `C0_STEP_CLAMP` */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 0 (`PRE_LOGIC`): Post-Divide Step Rounding & Coordinate Delta\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    logic signed [31:0] x_in_clamped_c;\n");
    fprintf(fp, "    assign x_in_clamped_c = clamp_sym64_to_32(state_w_q[3]);\n\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        logic        round_up;\n");
    fprintf(fp, "        logic [63:0] cap64;\n");
    fprintf(fp, "        logic        over_cap;\n");
    fprintf(fp, "        logic [63:0] mag;\n");
    fprintf(fp, "        logic signed [63:0] raw_step;\n");
    fprintf(fp, "        logic signed [31:0] xp_cand;\n");
    fprintf(fp, "        round_up  = (~pre_sat_q) & (sr_rem_q > ((sr_aux_q[63:0] - 64'd1) >> 1));\n");
    fprintf(fp, "        cap64     = {32'd0, $unsigned(dt_max_q)};\n");
    fprintf(fp, "        over_cap  = pre_sat_q | (sr_acc_q >= cap64);\n");
    fprintf(fp, "        mag       = over_cap ? cap64 : (sr_acc_q + {63'd0, round_up});\n");
    fprintf(fp, "        raw_step  = div_neg_q ? $signed(64'd0 - mag) : $signed(mag);\n");
    fprintf(fp, "        xp_cand   = clamp_sym64_to_32($unsigned($signed({{32{x_clamped_q[31]}}, x_clamped_q}) + raw_step));\n");
    fprintf(fp, "        n_c0_dc_c = $signed({xp_cand[31], xp_cand}) - $signed({x_clamped_q[31], x_clamped_q});\n");
    fprintf(fp, "    end\n\n");

    /* Phase 1 (`DSP33`): Instantiate `dsp_count` `ele_mul33_atom` units and drive muxes */
    const char *x_sq_for_a5  = ref_sig(ca, 2, 5, "n_a2_x_sq_c",  "n_a2_x_sq_q");
    const char *x_sq_for_a6  = ref_sig(ca, 2, 6, "n_a2_x_sq_c",  "n_a2_x_sq_q");
    const char *dc_for_c1    = ref_sig(cc, 0, 1, "n_c0_dc_c",    "n_c0_dc_q");
    const char *dc_for_c2    = ref_sig(cc, 0, 2, "n_c0_dc_c",    "n_c0_dc_q");
    const char *dc_for_c3    = ref_sig(cc, 0, 3, "n_c0_dc_c",    "n_c0_dc_q");
    const char *dc_for_c4    = ref_sig(cc, 0, 4, "n_c0_dc_c",    "n_c0_dc_q");
    const char *dsq_for_c5   = ref_sig(cc, 1, 5, "n_c1_dsq_c",   "n_c1_dsq_q");
    const char *dsq_for_c6   = ref_sig(cc, 1, 6, "n_c1_dsq_c",   "n_c1_dsq_q");

    char op_a5_b[64], op_a6_b[64], op_c5_b[64], op_c6_b[64];
    char op_c1_d[64], op_c2_d[64], op_c3_d[64], op_c4_d[64];
    snprintf(op_a5_b, sizeof(op_a5_b), "{1'b0, %s[31:0]}", x_sq_for_a5);
    snprintf(op_a6_b, sizeof(op_a6_b), "{1'b0, %s[63:32]}", x_sq_for_a6);
    snprintf(op_c5_b, sizeof(op_c5_b), "{1'b0, %s[31:0]}", dsq_for_c5);
    snprintf(op_c6_b, sizeof(op_c6_b), "{%s[63], %s[63:32]}", dsq_for_c6, dsq_for_c6);
    snprintf(op_c1_d, sizeof(op_c1_d), "$unsigned(%s)", dc_for_c1);
    snprintf(op_c2_d, sizeof(op_c2_d), "$unsigned(%s)", dc_for_c2);
    snprintf(op_c3_d, sizeof(op_c3_d), "$unsigned(%s)", dc_for_c3);
    snprintf(op_c4_d, sizeof(op_c4_d), "$unsigned(%s)", dc_for_c4);

    const dsp_bind_t binds[18] = {
        {0, 0, base_ca + ca->nodes[0].stage, "{u_gain_q[31], u_gain_q}", "{u_in_q[31], u_in_q}"},
        {0, 1, base_ca + ca->nodes[1].stage, "{b1_q[31], b1_q}",         "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 2, base_ca + ca->nodes[2].stage, "{x_in_clamped_c[31], x_in_clamped_c}", "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 3, base_ca + ca->nodes[3].stage, "{b2_q[31], b2_q}",         "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 5, base_ca + ca->nodes[5].stage, "{b2_q[31], b2_q}",         op_a5_b},
        {0, 6, base_ca + ca->nodes[6].stage, "{b2_q[31], b2_q}",         op_a6_b},

        {1, 0, base_cb + cb->nodes[0].stage, "{1'b0, g1_q[31:0]}",       "{1'b0, g1_q[31:0]}"},
        {1, 1, base_cb + cb->nodes[1].stage, "{g1_q[63], g1_q[63:32]}",  "{1'b0, g1_q[31:0]}"},
        {1, 2, base_cb + cb->nodes[2].stage, "{g1_q[63], g1_q[63:32]}",  "{g1_q[63], g1_q[63:32]}"},
        {1, 3, base_cb + cb->nodes[3].stage, "{1'b0, g0_q[31:0]}",       "{r2_q[31], r2_q}"},
        {1, 4, base_cb + cb->nodes[4].stage, "{1'b0, g0_q[63:32]}",      "{r2_q[31], r2_q}"},
        {1, 5, base_cb + cb->nodes[5].stage, "{g0_q[95], g0_q[95:64]}",  "{r2_q[31], r2_q}"},

        {2, 1, base_cc + cc->nodes[1].stage, op_c1_d,                    op_c1_d},
        {2, 2, base_cc + cc->nodes[2].stage, "{1'b0, g1_q[31:0]}",       op_c2_d},
        {2, 3, base_cc + cc->nodes[3].stage, "{g1_q[63], g1_q[63:32]}",  op_c3_d},
        {2, 4, base_cc + cc->nodes[4].stage, "{r2_q[31], r2_q}",         op_c4_d},
        {2, 5, base_cc + cc->nodes[5].stage, "{r2_q[31], r2_q}",         op_c5_b},
        {2, 6, base_cc + cc->nodes[6].stage, "{r2_q[31], r2_q}",         op_c6_b},
    };

    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 1 (`DSP33`): %d Operand-Isolated `ele_mul33_atom` DSP Slices\n", dsp_count);
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    logic [%d:0]        dsp_en_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][32:0]  dsp_a_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][32:0]  dsp_b_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][65:0]  dsp_prod_w;\n\n", dsp_count - 1);

    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        dsp_en_c = %d'd0;\n", dsp_count);
    fprintf(fp, "        dsp_a_c  = %d'd0;\n", dsp_count * 33);
    fprintf(fp, "        dsp_b_c  = %d'd0;\n", dsp_count * 33);
    fprintf(fp, "        case (st_q)\n");
    for (int s = 1; s < num_st; ++s) {
        bool any = false;
        for (int b = 0; b < 18; ++b) {
            if (binds[b].global_state == s) {
                any = true;
                break;
            }
        }
        if (!any) {
            continue;
        }
        fprintf(fp, "            %d'd%d: begin\n", st_w, s);
        for (int b = 0; b < 18; ++b) {
            if (binds[b].global_state != s) {
                continue;
            }
            const dag_cone_t *cone = (binds[b].cone_id == 0) ? ca : ((binds[b].cone_id == 1) ? cb : cc);
            int slot = cone->nodes[binds[b].node_idx].dsp_slot;
            if (binds[b].cone_id == 2) {
                fprintf(fp, "                dsp_en_c[%d] = ~fast_path_q;\n", slot);
            } else {
                fprintf(fp, "                dsp_en_c[%d] = 1'b1;\n", slot);
            }
            fprintf(fp, "                dsp_a_c[%d]  = %s;\n", slot, binds[b].op_a_expr);
            fprintf(fp, "                dsp_b_c[%d]  = %s;\n", slot, binds[b].op_b_expr);
        }
        fprintf(fp, "            end\n");
    }
    fprintf(fp, "            default: begin end\n");
    fprintf(fp, "        endcase\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    genvar gd;\n");
    fprintf(fp, "    generate\n");
    fprintf(fp, "        for (gd = 0; gd < %d; gd++) begin : g_dsp33\n", dsp_count);
    fprintf(fp, "            logic signed [32:0] a_s, b_s;\n");
    fprintf(fp, "            logic signed [65:0] prod_s;\n");
    fprintf(fp, "            assign a_s = $signed(dsp_a_c[gd]);\n");
    fprintf(fp, "            assign b_s = $signed(dsp_b_c[gd]);\n");
    fprintf(fp, "            ele_mul33_atom u_mul33 (\n");
    fprintf(fp, "                .en_i   (dsp_en_c[gd]),\n");
    fprintf(fp, "                .a_i    (a_s),\n");
    fprintf(fp, "                .b_i    (b_s),\n");
    fprintf(fp, "                .prod_o (prod_s)\n");
    fprintf(fp, "            );\n");
    fprintf(fp, "            assign dsp_prod_w[gd] = $unsigned(prod_s);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endgenerate\n\n");

    /* Route DSP outputs to their corresponding node `_c` wires */
    fprintf(fp, "    assign n_a0_ug_u_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[0].dsp_slot);
    fprintf(fp, "    assign n_a1_b1_x_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[1].dsp_slot);
    fprintf(fp, "    assign n_a2_x_sq_c     = dsp_prod_w[%d][63:0];\n", ca->nodes[2].dsp_slot);
    fprintf(fp, "    assign n_a3_b2_x_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[3].dsp_slot);
    fprintf(fp, "    assign n_a5_b2xsq_lo_c = $signed(dsp_prod_w[%d]);\n", ca->nodes[5].dsp_slot);
    fprintf(fp, "    assign n_a6_b2xsq_hi_c = $signed(dsp_prod_w[%d]);\n", ca->nodes[6].dsp_slot);

    fprintf(fp, "    assign n_b0_g1_ll_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[0].dsp_slot);
    fprintf(fp, "    assign n_b1_g1_hl_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[1].dsp_slot);
    fprintf(fp, "    assign n_b2_g1_hh_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[2].dsp_slot);
    fprintf(fp, "    assign n_b3_g0r2_0_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[3].dsp_slot);
    fprintf(fp, "    assign n_b4_g0r2_1_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[4].dsp_slot);
    fprintf(fp, "    assign n_b5_g0r2_2_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[5].dsp_slot);

    fprintf(fp, "    assign n_c1_dsq_c      = $signed(dsp_prod_w[%d][63:0]);\n", cc->nodes[1].dsp_slot);
    fprintf(fp, "    assign n_c2_g1dc_lo_c  = $signed(dsp_prod_w[%d]);\n", cc->nodes[2].dsp_slot);
    fprintf(fp, "    assign n_c3_g1dc_hi_c  = $signed(dsp_prod_w[%d]);\n", cc->nodes[3].dsp_slot);
    fprintf(fp, "    assign n_c4_r2_dc_c    = $signed(dsp_prod_w[%d][63:0]);\n", cc->nodes[4].dsp_slot);
    fprintf(fp, "    assign n_c5_r2dsq_lo_c = $signed(dsp_prod_w[%d]);\n", cc->nodes[5].dsp_slot);
    fprintf(fp, "    assign n_c6_r2dsq_hi_c = $signed(dsp_prod_w[%d]);\n\n", cc->nodes[6].dsp_slot);

    /* Instantiate `k_rad` chained `ele_rad2_step_atom` slices */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Radix-2 Unroll Chain (`K_rad = %d` `ele_rad2_step_atom` slices / cycle)\n", k_rad);
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    logic                rad2_en_c;\n");
    fprintf(fp, "    logic                rad2_is_div_c;\n");
    fprintf(fp, "    logic [%d:0][63:0]   rad2_rem_w;\n", k_rad);
    fprintf(fp, "    logic [%d:0][63:0]   rad2_acc_w;\n", k_rad);
    fprintf(fp, "    logic [%d:0][127:0]  rad2_aux_w;\n\n", k_rad);
    fprintf(fp, "    assign rad2_en_c     = (st_q == %d'd%d) | (st_q == %d'd%d);\n",
            st_w, st_isqrt, st_w, st_udiv);
    fprintf(fp, "    assign rad2_is_div_c = (st_q == %d'd%d);\n", st_w, st_udiv);
    fprintf(fp, "    assign rad2_rem_w[0] = sr_rem_q;\n");
    fprintf(fp, "    assign rad2_acc_w[0] = sr_acc_q;\n");
    fprintf(fp, "    assign rad2_aux_w[0] = sr_aux_q;\n\n");
    fprintf(fp, "    genvar gr;\n");
    fprintf(fp, "    generate\n");
    fprintf(fp, "        for (gr = 0; gr < %d; gr++) begin : g_rad2\n", k_rad);
    fprintf(fp, "            ele_rad2_step_atom u_rad2 (\n");
    fprintf(fp, "                .en_i     (rad2_en_c),\n");
    fprintf(fp, "                .is_div_i (rad2_is_div_c),\n");
    fprintf(fp, "                .rem_i    (rad2_rem_w[gr]),\n");
    fprintf(fp, "                .acc_i    (rad2_acc_w[gr]),\n");
    fprintf(fp, "                .aux_i    (rad2_aux_w[gr]),\n");
    fprintf(fp, "                .rem_o    (rad2_rem_w[gr+1]),\n");
    fprintf(fp, "                .acc_o    (rad2_acc_w[gr+1]),\n");
    fprintf(fp, "                .aux_o    (rad2_aux_w[gr+1])\n");
    fprintf(fp, "            );\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endgenerate\n\n");

    /* Phase 2 (`POST_LOGIC`): Topological combinational logic for Cones A, B, D, C + Fast Commit */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 2 (`POST_LOGIC`): Topological Cone Combinators\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        logic signed [63:0]  j0_in, j1_in;\n");
    fprintf(fp, "        logic signed [31:0]  j2_in;\n");
    fprintf(fp, "        logic                sub_valid;\n");
    fprintf(fp, "        logic [32:0]         j0_sub_in;\n");
    fprintf(fp, "        logic [15:0]         j1_sub_in;\n");
    fprintf(fp, "        logic signed [127:0] j0_q49, b2_xsq128;\n");
    fprintf(fp, "        logic                d_neg;\n");
    fprintf(fp, "        logic signed [63:0]  signed_d;\n");
    fprintf(fp, "        logic signed [65:0]  q_cit66, div_den66;\n");
    fprintf(fp, "        logic signed [127:0] neg_g0;\n");
    fprintf(fp, "        logic                has_r2, real_roots;\n");
    fprintf(fp, "        logic [127:0]        anum128;\n");
    fprintf(fp, "        logic [63:0]         aden64;\n");
    fprintf(fp, "        logic                den_zero;\n");
        fprintf(fp, "        logic signed [127:0] g1_dc128, r2_dsq128, push_q49, j0_au_next;\n");
        fprintf(fp, "        logic                lyap_ok;\n");
        fprintf(fp, "        logic signed [32:0]  delta_act;\n");
        fprintf(fp, "        logic signed [63:0]  j0_op_out, j1_op_out, j0_au_out, j1_au_out;\n");
        fprintf(fp, "        logic [15:0]         raw_chart;\n\n");

    /* Cone A POST_LOGIC */
    fprintf(fp, "        // Cone A (`A4`, `A7`, `A8`, `A9`, `A10`)\n");
    fprintf(fp, "        n_a4_b0_eff_c = ($signed({{96{b0_q[31]}}, b0_q}) <<< 33) -\n");
    fprintf(fp, "                        ($signed({{64{%s[63]}}, %s}) <<< 17);\n",
            ref_sig(ca, 0, 4, "n_a0_ug_u_c", "n_a0_ug_u_q"),
            ref_sig(ca, 0, 4, "n_a0_ug_u_c", "n_a0_ug_u_q"));
    fprintf(fp, "        b2_xsq128     = ($signed({{62{%s[65]}}, %s}) <<< 32) +\n",
            ref_sig(ca, 6, 7, "n_a6_b2xsq_hi_c", "n_a6_b2xsq_hi_q"),
            ref_sig(ca, 6, 7, "n_a6_b2xsq_hi_c", "n_a6_b2xsq_hi_q"));
    fprintf(fp, "                        $signed({{62{%s[65]}}, %s});\n",
            ref_sig(ca, 5, 7, "n_a5_b2xsq_lo_c", "n_a5_b2xsq_lo_q"),
            ref_sig(ca, 5, 7, "n_a5_b2xsq_lo_c", "n_a5_b2xsq_lo_q"));
    fprintf(fp, "        n_a7_g0_op_c  = %s + ($signed({{64{%s[63]}}, %s}) <<< 17) + b2_xsq128;\n",
            ref_sig(ca, 4, 7, "n_a4_b0_eff_c", "n_a4_b0_eff_q"),
            ref_sig(ca, 1, 7, "n_a1_b1_x_c", "n_a1_b1_x_q"),
            ref_sig(ca, 1, 7, "n_a1_b1_x_c", "n_a1_b1_x_q"));
    fprintf(fp, "        n_a8_g1_op_c  = ($signed({{32{b1_q[31]}}, b1_q}) <<< 16) + %s;\n",
            ref_sig(ca, 3, 8, "n_a3_b2_x_c", "n_a3_b2_x_q"));
    fprintf(fp, "        j0_in         = clamp_sym64($signed(state_w_q[0]), 64'sh1000_0000_0000_0000);\n");
    fprintf(fp, "        j1_in         = clamp_sym64($signed(state_w_q[1]), 64'sh0000_2000_0000_0000);\n");
    fprintf(fp, "        j2_in         = clamp_sym64_to_32(state_w_q[2]);\n");
    fprintf(fp, "        sub_valid     = ($signed(state_w_q[0]) == j0_in) & ($signed(state_w_q[1]) == j1_in);\n");
    fprintf(fp, "        j0_sub_in     = sub_valid ? state_w_q[7][32:0]  : 33'd0;\n");
    fprintf(fp, "        j1_sub_in     = sub_valid ? state_w_q[7][63:48] : 16'd0;\n");
    fprintf(fp, "        j0_q49        = ($signed({{64{j0_in[63]}}, j0_in}) <<< 33) + $signed({95'd0, j0_sub_in});\n");
    fprintf(fp, "        n_a9_j1_q32_c = (j1_in <<< 16) + $signed({48'd0, j1_sub_in});\n");
    fprintf(fp, "        n_a9_g0_au_c  = j0_q49 - %s;\n",
            ref_sig(ca, 4, 9, "n_a4_b0_eff_c", "n_a4_b0_eff_q"));
    fprintf(fp, "        n_a10_op_mask_c = (b1_q != 32'sd0) | (b2_q != 32'sd0);\n");
    fprintf(fp, "        n_a10_g0_c      = n_a10_op_mask_c ? %s : %s;\n",
            ref_sig(ca, 7, 10, "n_a7_g0_op_c", "n_a7_g0_op_q"),
            ref_sig(ca, 9, 10, "n_a9_g0_au_c", "n_a9_g0_au_q"));
    fprintf(fp, "        n_a10_g1_c      = n_a10_op_mask_c ? %s : %s;\n",
            ref_sig(ca, 8, 10, "n_a8_g1_op_c", "n_a8_g1_op_q"),
            ref_sig(ca, 9, 10, "n_a9_j1_q32_c", "n_a9_j1_q32_q"));
    fprintf(fp, "        n_a10_r2_c      = n_a10_op_mask_c ? b2_q : j2_in;\n");
    fprintf(fp, "        n_a10_has_r2_c  = (n_a10_r2_c != 32'sd0);\n");
    fprintf(fp, "        n_a10_active_c  = (n_a10_g0_c != 128'sd0) & ((n_a10_g1_c != 64'sd0) | n_a10_has_r2_c);\n\n");

    /* Cone B POST_LOGIC */
    fprintf(fp, "        // Cone B (`B6`, `B7`, `B8`)\n");
    fprintf(fp, "        n_b6_g1_sq_c  = ($signed({{62{%s[65]}}, %s}) <<< 64) +\n",
            ref_sig(cb, 2, 6, "n_b2_g1_hh_c", "n_b2_g1_hh_q"),
            ref_sig(cb, 2, 6, "n_b2_g1_hh_c", "n_b2_g1_hh_q"));
    fprintf(fp, "                        ($signed({{62{%s[65]}}, %s}) <<< 33) +\n",
            ref_sig(cb, 1, 6, "n_b1_g1_hl_c", "n_b1_g1_hl_q"),
            ref_sig(cb, 1, 6, "n_b1_g1_hl_c", "n_b1_g1_hl_q"));
    fprintf(fp, "                        $signed({{62{%s[65]}}, %s});\n",
            ref_sig(cb, 0, 6, "n_b0_g1_ll_c", "n_b0_g1_ll_q"),
            ref_sig(cb, 0, 6, "n_b0_g1_ll_c", "n_b0_g1_ll_q"));
    fprintf(fp, "        n_b7_g0_r2_c  = ($signed({{62{%s[65]}}, %s}) <<< 64) +\n",
            ref_sig(cb, 5, 7, "n_b5_g0r2_2_c", "n_b5_g0r2_2_q"),
            ref_sig(cb, 5, 7, "n_b5_g0r2_2_c", "n_b5_g0r2_2_q"));
    fprintf(fp, "                        ($signed({{62{%s[65]}}, %s}) <<< 32) +\n",
            ref_sig(cb, 4, 7, "n_b4_g0r2_1_c", "n_b4_g0r2_1_q"),
            ref_sig(cb, 4, 7, "n_b4_g0r2_1_c", "n_b4_g0r2_1_q"));
    fprintf(fp, "                        $signed({{62{%s[65]}}, %s});\n",
            ref_sig(cb, 3, 7, "n_b3_g0r2_0_c", "n_b3_g0r2_0_q"),
            ref_sig(cb, 3, 7, "n_b3_g0r2_0_c", "n_b3_g0r2_0_q"));
    fprintf(fp, "        n_b8_disc_c   = %s - %s;\n",
            ref_sig(cb, 6, 8, "n_b6_g1_sq_c", "n_b6_g1_sq_q"),
            ref_sig(cb, 7, 8, "n_b7_g0_r2_c", "n_b7_g0_r2_q"));
    fprintf(fp, "        n_b8_pos_nz_c = (~n_b8_disc_c[127]) & (n_b8_disc_c != 128'sd0);\n\n");

    /* Cone D POST_LOGIC (`real_roots` tracked cleanly via `d_mag_q` / `real_roots_q`) */
    fprintf(fp, "        // Cone D (`D0`, `D1`)\n");
    fprintf(fp, "        has_r2           = (r2_q != 32'sd0);\n");
    fprintf(fp, "        real_roots       = ~d_mag_q[63];\n");
    fprintf(fp, "        d_neg            = (g1_q < 64'sd0) | ((g1_q == 64'sd0) & (r2_q < 32'sd0));\n");
    fprintf(fp, "        signed_d         = d_neg ? $signed(64'd0 - {1'b0, d_mag_q[62:0]}) : $signed({1'b0, d_mag_q[62:0]});\n");
    fprintf(fp, "        q_cit66          = $signed({{2{g1_q[63]}}, g1_q}) + $signed({{2{signed_d[63]}}, signed_d});\n");
    fprintf(fp, "        neg_g0           = 128'sd0 - g0_q;\n");
    fprintf(fp, "        n_d0_div_num_c   = (has_r2 & (~real_roots)) ? (128'sd0 - $signed({{64{g1_q[63]}}, g1_q})) : neg_g0;\n");
    fprintf(fp, "        div_den66        = has_r2 ? (real_roots ? q_cit66 : $signed({{34{r2_q[31]}}, r2_q}))\n");
    fprintf(fp, "                                  : ($signed({{2{g1_q[63]}}, g1_q}) <<< 1);\n");
    fprintf(fp, "        n_d0_div_den_c   = {div_den66[65], div_den66[63:0]};\n");
    fprintf(fp, "        n_d0_div_neg_c   = n_d0_div_num_c[127] ^ div_den66[65];\n");
    fprintf(fp, "        if (div_den66[64]) begin end\n");
    const char *dden_d1 = ref_sig(cd, 0, 1, "n_d0_div_den_c", "n_d0_div_den_q");
    fprintf(fp, "        anum128          = abs128(%s);\n",
            ref_sig(cd, 0, 1, "n_d0_div_num_c", "n_d0_div_num_q"));
    fprintf(fp, "        aden64           = %s[64] ? (64'd0 - %s[63:0]) : %s[63:0];\n",
            dden_d1, dden_d1, dden_d1);
    fprintf(fp, "        den_zero         = (aden64 == 64'd0);\n");
    fprintf(fp, "        n_d1_safe_den_c  = aden64 | {63'd0, den_zero};\n");
    fprintf(fp, "        n_d1_pre_sat_c   = den_zero | (anum128[127:64] >= n_d1_safe_den_c);\n");
    fprintf(fp, "        n_d1_init_rem_c  = n_d1_pre_sat_c ? 64'd0 : anum128[127:64];\n");
    fprintf(fp, "        n_d1_init_quot_c = anum128[63:0];\n\n");

    /* Cone C POST_LOGIC (`C7`, `C8`, `C9`, `C10`) */
    fprintf(fp, "        // Cone C (`C7`, `C8`, `C9`, `C10`)\n");
    fprintf(fp, "        g1_dc128        = ($signed({{62{%s[65]}}, %s}) <<< 32) +\n",
            ref_sig(cc, 3, 7, "n_c3_g1dc_hi_c", "n_c3_g1dc_hi_q"),
            ref_sig(cc, 3, 7, "n_c3_g1dc_hi_c", "n_c3_g1dc_hi_q"));
    fprintf(fp, "                          $signed({{62{%s[65]}}, %s});\n",
            ref_sig(cc, 2, 7, "n_c2_g1dc_lo_c", "n_c2_g1dc_lo_q"),
            ref_sig(cc, 2, 7, "n_c2_g1dc_lo_c", "n_c2_g1dc_lo_q"));
    fprintf(fp, "        r2_dsq128       = ($signed({{62{%s[65]}}, %s}) <<< 32) +\n",
            ref_sig(cc, 6, 7, "n_c6_r2dsq_hi_c", "n_c6_r2dsq_hi_q"),
            ref_sig(cc, 6, 7, "n_c6_r2dsq_hi_c", "n_c6_r2dsq_hi_q"));
    fprintf(fp, "                          $signed({{62{%s[65]}}, %s});\n",
            ref_sig(cc, 5, 7, "n_c5_r2dsq_lo_c", "n_c5_r2dsq_lo_q"),
            ref_sig(cc, 5, 7, "n_c5_r2dsq_lo_c", "n_c5_r2dsq_lo_q"));
    fprintf(fp, "        push_q49        = (g1_dc128 <<< 1) + r2_dsq128;\n");
    fprintf(fp, "        n_c7_g0_cand_c  = g0_q + push_q49;\n");
    fprintf(fp, "        lyap_ok         = (~fast_path_q) & (abs128(%s) <= abs128(g0_q));\n",
            ref_sig(cc, 7, 8, "n_c7_g0_cand_c", "n_c7_g0_cand_q"));
    fprintf(fp, "        delta_act       = lyap_ok ? %s : 33'sd0;\n",
            ref_sig(cc, 0, 8, "n_c0_dc_c", "n_c0_dc_q"));
    fprintf(fp, "        n_c8_xp_c       = $signed(x_clamped_q + delta_act[31:0]);\n");
    fprintf(fp, "        n_c8_dt_res_c   = delta_act[32] ? (64'd0 - {{31{delta_act[32]}}, $unsigned(delta_act)})\n");
    fprintf(fp, "                                        : {31'd0, $unsigned(delta_act)};\n");
    fprintf(fp, "        n_c8_g0_post_c  = lyap_ok ? %s : g0_q;\n",
            ref_sig(cc, 7, 8, "n_c7_g0_cand_c", "n_c7_g0_cand_q"));
    fprintf(fp, "        n_c8_g1_post_c  = g1_q + (lyap_ok ? %s : 64'sd0);\n",
            ref_sig(cc, 4, 8, "n_c4_r2_dc_c", "n_c4_r2_dc_q"));

    const char *g0p_c9 = ref_sig(cc, 8, 9, "n_c8_g0_post_c", "n_c8_g0_post_q");
    const char *g1p_c9 = ref_sig(cc, 8, 9, "n_c8_g1_post_c", "n_c8_g1_post_q");
    fprintf(fp, "        j0_op_out       = sym_shr128_33(%s);\n", g0p_c9);
    fprintf(fp, "        j1_op_out       = sym_shr64_16(%s);\n", g1p_c9);
    fprintf(fp, "        j0_au_next      = %s + b0_eff_q;\n", g0p_c9);
    fprintf(fp, "        j0_au_out       = clamp_sym64($signed(j0_au_next[96:33]), 64'sh1000_0000_0000_0000);\n");
    fprintf(fp, "        j1_au_out       = clamp_sym64($signed({{16{%s[63]}}, %s[63:16]}), 64'sh0000_2000_0000_0000);\n",
            g1p_c9, g1p_c9);
    fprintf(fp, "        n_c9_j0_next_c  = $unsigned(op_mask_q ? j0_op_out : j0_au_out);\n");
    fprintf(fp, "        n_c9_j1_next_c  = $unsigned(op_mask_q ? j1_op_out : j1_au_out);\n");
    fprintf(fp, "        n_c9_sub_next_c = op_mask_q ? 64'd0 : {%s[15:0], 15'd0, j0_au_next[32:0]};\n", g1p_c9);

    const char *xp_c10  = ref_sig(cc, 8, 10, "n_c8_xp_c",      "n_c8_xp_q");
    const char *g0p_c10 = ref_sig(cc, 8, 10, "n_c8_g0_post_c", "n_c8_g0_post_q");
    fprintf(fp, "        raw_chart          = (%s > b_enter_q) ? {8'd0, next_hi_q}\n", xp_c10);
    fprintf(fp, "                           : ((%s < b_exit_q) ? {8'd0, next_lo_q} : state_w_q[5][31:16]);\n", xp_c10);
    fprintf(fp, "        n_c10_chart_next_c = raw_chart & cap_mask16_q;\n");
    fprintf(fp, "        n_c10_mode_proj_c  = (mode_req_q & (~kan_lock_q)) | (state_w_q[5][15:0] & kan_lock_q);\n");
    fprintf(fp, "        n_c10_conv_c       = (abs128(%s) <= 128'h0000_0000_0000_0000_0000_0008_0000_0000);\n", g0p_c10);
    fprintf(fp, "        if (&{1'b0, j0_au_next[127:97]}) begin end\n");
    fprintf(fp, "    end\n\n");

    /* Sequential FSM & Pipeline Register Transitions */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Sequential Pipeline & Tickless Manifold State Machine\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    always_ff @(posedge clk or negedge rst_n) begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            st_q         <= %d'd0;\n", st_w);
    fprintf(fp, "            iter_q       <= %d'd0;\n", iter_w);
    fprintf(fp, "            done_q       <= 1'b0;\n");
    fprintf(fp, "            state_w_q    <= 512'd0;\n");
    fprintf(fp, "            b0_q         <= 32'sd0;\n");
    fprintf(fp, "            b1_q         <= 32'sd0;\n");
    fprintf(fp, "            b2_q         <= 32'sd0;\n");
    fprintf(fp, "            b_exit_q     <= 32'sd0;\n");
    fprintf(fp, "            b_enter_q    <= 32'sd0;\n");
    fprintf(fp, "            dt_max_q     <= 32'sd1;\n");
    fprintf(fp, "            u_gain_q     <= 32'sd0;\n");
    fprintf(fp, "            u_in_q       <= 32'sd0;\n");
    fprintf(fp, "            next_lo_q    <= 8'd0;\n");
    fprintf(fp, "            next_hi_q    <= 8'd0;\n");
    fprintf(fp, "            kan_lock_q   <= 16'd0;\n");
    fprintf(fp, "            cap_mask16_q <= 16'hFFFF;\n");
    fprintf(fp, "            mode_req_q   <= 16'd0;\n");
    fprintf(fp, "            op_mask_q    <= 1'b0;\n");
    fprintf(fp, "            fast_path_q  <= 1'b0;\n");
    fprintf(fp, "            x_clamped_q  <= 32'sd0;\n");
    fprintf(fp, "            b0_eff_q     <= 128'sd0;\n");
    fprintf(fp, "            g0_q         <= 128'sd0;\n");
    fprintf(fp, "            g1_q         <= 64'sd0;\n");
    fprintf(fp, "            r2_q         <= 32'sd0;\n");
    fprintf(fp, "            d_mag_q      <= 64'd0;\n");
    fprintf(fp, "            div_neg_q    <= 1'b0;\n");
    fprintf(fp, "            pre_sat_q    <= 1'b0;\n");
    fprintf(fp, "            sr_rem_q     <= 64'd0;\n");
    fprintf(fp, "            sr_acc_q     <= 64'd0;\n");
    fprintf(fp, "            sr_aux_q     <= 128'd0;\n");

    if (ca->nodes[0].needs_reg) fprintf(fp, "            n_a0_ug_u_q      <= 64'sd0;\n");
    if (ca->nodes[1].needs_reg) fprintf(fp, "            n_a1_b1_x_q      <= 64'sd0;\n");
    if (ca->nodes[2].needs_reg) fprintf(fp, "            n_a2_x_sq_q      <= 64'd0;\n");
    if (ca->nodes[3].needs_reg) fprintf(fp, "            n_a3_b2_x_q      <= 64'sd0;\n");
    if (ca->nodes[4].needs_reg) fprintf(fp, "            n_a4_b0_eff_q    <= 128'sd0;\n");
    if (ca->nodes[5].needs_reg) fprintf(fp, "            n_a5_b2xsq_lo_q  <= 66'sd0;\n");
    if (ca->nodes[6].needs_reg) fprintf(fp, "            n_a6_b2xsq_hi_q  <= 66'sd0;\n");
    if (ca->nodes[7].needs_reg) fprintf(fp, "            n_a7_g0_op_q     <= 128'sd0;\n");
    if (ca->nodes[8].needs_reg) fprintf(fp, "            n_a8_g1_op_q     <= 64'sd0;\n");
    if (ca->nodes[9].needs_reg) {
        fprintf(fp, "            n_a9_g0_au_q     <= 128'sd0;\n");
        fprintf(fp, "            n_a9_j1_q32_q    <= 64'sd0;\n");
    }
    if (cb->nodes[0].needs_reg) fprintf(fp, "            n_b0_g1_ll_q     <= 66'sd0;\n");
    if (cb->nodes[1].needs_reg) fprintf(fp, "            n_b1_g1_hl_q     <= 66'sd0;\n");
    if (cb->nodes[2].needs_reg) fprintf(fp, "            n_b2_g1_hh_q     <= 66'sd0;\n");
    if (cb->nodes[3].needs_reg) fprintf(fp, "            n_b3_g0r2_0_q    <= 66'sd0;\n");
    if (cb->nodes[4].needs_reg) fprintf(fp, "            n_b4_g0r2_1_q    <= 66'sd0;\n");
    if (cb->nodes[5].needs_reg) fprintf(fp, "            n_b5_g0r2_2_q    <= 66'sd0;\n");
    if (cb->nodes[6].needs_reg) fprintf(fp, "            n_b6_g1_sq_q     <= 128'sd0;\n");
    if (cb->nodes[7].needs_reg) fprintf(fp, "            n_b7_g0_r2_q     <= 128'sd0;\n");
    if (cd->nodes[0].needs_reg) {
        fprintf(fp, "            n_d0_div_num_q   <= 128'sd0;\n");
        fprintf(fp, "            n_d0_div_den_q   <= 65'd0;\n");
        fprintf(fp, "            n_d0_div_neg_q   <= 1'b0;\n");
    }
    if (cc->nodes[0].needs_reg) fprintf(fp, "            n_c0_dc_q        <= 33'sd0;\n");
    if (cc->nodes[1].needs_reg) fprintf(fp, "            n_c1_dsq_q       <= 64'sd0;\n");
    if (cc->nodes[2].needs_reg) fprintf(fp, "            n_c2_g1dc_lo_q   <= 66'sd0;\n");
    if (cc->nodes[3].needs_reg) fprintf(fp, "            n_c3_g1dc_hi_q   <= 66'sd0;\n");
    if (cc->nodes[4].needs_reg) fprintf(fp, "            n_c4_r2_dc_q     <= 64'sd0;\n");
    if (cc->nodes[5].needs_reg) fprintf(fp, "            n_c5_r2dsq_lo_q  <= 66'sd0;\n");
    if (cc->nodes[6].needs_reg) fprintf(fp, "            n_c6_r2dsq_hi_q  <= 66'sd0;\n");
    if (cc->nodes[7].needs_reg) fprintf(fp, "            n_c7_g0_cand_q   <= 128'sd0;\n");
    if (cc->nodes[8].needs_reg) {
        fprintf(fp, "            n_c8_g0_post_q   <= 128'sd0;\n");
        fprintf(fp, "            n_c8_g1_post_q   <= 64'sd0;\n");
        fprintf(fp, "            n_c8_xp_q        <= 32'sd0;\n");
        fprintf(fp, "            n_c8_dt_res_q    <= 64'd0;\n");
    }
    if (cc->nodes[9].needs_reg) {
        fprintf(fp, "            n_c9_j0_next_q   <= 64'd0;\n");
        fprintf(fp, "            n_c9_j1_next_q   <= 64'd0;\n");
        fprintf(fp, "            n_c9_sub_next_q  <= 64'd0;\n");
    }

    fprintf(fp, "        end else begin\n");
    fprintf(fp, "            done_q <= 1'b0;\n");
    fprintf(fp, "            case (st_q)\n");

    /* ST_IDLE (0): 64-bit streaming ingress */
    fprintf(fp, "                %d'd%d: begin\n", st_w, st_idle);
    fprintf(fp, "                    if (wr_valid_i) begin\n");
    fprintf(fp, "                        case (wr_kind_i)\n");
    fprintf(fp, "                            2'd0: state_w_q[wr_idx_i] <= wr_data_i;\n");
    fprintf(fp, "                            2'd1: begin\n");
    fprintf(fp, "                                case (wr_idx_i)\n");
    fprintf(fp, "                                    3'd0: b0_q <= clamp_sym64_to_32(wr_data_i);\n");
    fprintf(fp, "                                    3'd1: b1_q <= clamp_sym64_to_32(wr_data_i);\n");
    fprintf(fp, "                                    3'd2: b2_q <= clamp_sym64_to_32(wr_data_i);\n");
    fprintf(fp, "                                    3'd3: begin\n");
    fprintf(fp, "                                        logic signed [31:0] ex_c, en_c;\n");
    fprintf(fp, "                                        ex_c      = clamp_sym32(wr_data_i[63:32]);\n");
    fprintf(fp, "                                        en_c      = clamp_sym32(wr_data_i[31:0]);\n");
    fprintf(fp, "                                        b_exit_q  <= ex_c;\n");
    fprintf(fp, "                                        b_enter_q <= (en_c < ex_c) ? ex_c : en_c;\n");
    fprintf(fp, "                                    end\n");
    fprintf(fp, "                                    3'd4: begin\n");
    fprintf(fp, "                                        logic signed [31:0] dt_c;\n");
    fprintf(fp, "                                        dt_c     = clamp_sym32(wr_data_i[63:32]);\n");
    fprintf(fp, "                                        dt_max_q <= (dt_c < 32'sd1) ? 32'sd1 : dt_c;\n");
    fprintf(fp, "                                        u_gain_q <= clamp_sym32(wr_data_i[31:0]);\n");
    fprintf(fp, "                                    end\n");
    fprintf(fp, "                                    3'd5: begin\n");
    fprintf(fp, "                                        next_lo_q  <= wr_data_i[31:24];\n");
    fprintf(fp, "                                        next_hi_q  <= wr_data_i[23:16];\n");
    fprintf(fp, "                                        kan_lock_q <= wr_data_i[15:0];\n");
    fprintf(fp, "                                    end\n");
    fprintf(fp, "                                    default: begin end\n");
    fprintf(fp, "                                endcase\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            2'd2: begin\n");
    fprintf(fp, "                                if (wr_idx_i == 3'd0) begin\n");
    fprintf(fp, "                                    cap_mask16_q <= wr_data_i[15:0];\n");
    fprintf(fp, "                                    mode_req_q   <= wr_data_i[31:16];\n");
    fprintf(fp, "                                end else if (wr_idx_i == 3'd1) begin\n");
    fprintf(fp, "                                    u_in_q      <= clamp_sym64_to_32(wr_data_i);\n");
    fprintf(fp, "                                    x_clamped_q <= x_in_clamped_c;\n");
    fprintf(fp, "                                    fast_path_q <= 1'b0;\n");
    fprintf(fp, "                                    st_q        <= %d'd%d;\n", st_w, base_ca);
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            default: begin end\n");
    fprintf(fp, "                        endcase\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    /* Cone A Stages */
    static const char *ca_lhs[10] = {
        "n_a0_ug_u_q     <= n_a0_ug_u_c;",
        "n_a1_b1_x_q     <= n_a1_b1_x_c;",
        "n_a2_x_sq_q     <= n_a2_x_sq_c;",
        "n_a3_b2_x_q     <= n_a3_b2_x_c;",
        "n_a4_b0_eff_q   <= n_a4_b0_eff_c;",
        "n_a5_b2xsq_lo_q <= n_a5_b2xsq_lo_c;",
        "n_a6_b2xsq_hi_q <= n_a6_b2xsq_hi_c;",
        "n_a7_g0_op_q    <= n_a7_g0_op_c;",
        "n_a8_g1_op_q    <= n_a8_g1_op_c;",
        "n_a9_g0_au_q    <= n_a9_g0_au_c;\n                    n_a9_j1_q32_q   <= n_a9_j1_q32_c;",
    };
    for (int s = 0; s < ca->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_ca + s);
        for (int n = 0; n < 10; ++n) {
            if (ca->nodes[n].stage == s && ca->nodes[n].needs_reg) {
                fprintf(fp, "                    %s\n", ca_lhs[n]);
            }
        }
        if (s == ca->nodes[4].stage) {
            fprintf(fp, "                    b0_eff_q <= n_a4_b0_eff_c;\n");
        }
        if (s < ca->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_ca + s + 1);
        } else {
            fprintf(fp, "                    op_mask_q <= n_a10_op_mask_c;\n");
            fprintf(fp, "                    g0_q      <= n_a10_g0_c;\n");
            fprintf(fp, "                    g1_q      <= n_a10_g1_c;\n");
            fprintf(fp, "                    r2_q      <= n_a10_r2_c;\n");
            fprintf(fp, "                    if (!n_a10_active_c) begin\n");
            fprintf(fp, "                        fast_path_q <= 1'b1;\n");
            if (cc->nodes[8].needs_reg) {
                fprintf(fp, "                        n_c8_g0_post_q <= n_a10_g0_c;\n");
                fprintf(fp, "                        n_c8_g1_post_q <= n_a10_g1_c;\n");
                fprintf(fp, "                        n_c8_xp_q      <= x_clamped_q;\n");
                fprintf(fp, "                        n_c8_dt_res_q  <= 64'd0;\n");
            }
            fprintf(fp, "                        st_q        <= %d'd%d;\n", st_w, base_cc + cc->nodes[9].stage);
            fprintf(fp, "                    end else if (!n_a10_has_r2_c) begin\n");
            fprintf(fp, "                        d_mag_q <= 64'd0;\n");
            fprintf(fp, "                        st_q    <= %d'd%d;\n", st_w, base_cd);
            fprintf(fp, "                    end else begin\n");
            fprintf(fp, "                        st_q <= %d'd%d;\n", st_w, base_cb);
            fprintf(fp, "                    end\n");
        }
        fprintf(fp, "                end\n");
    }

    /* Cone B Stages (`Discriminant`)
     * Note on `d_mag_q[63]`: Since `disc < 2^126`, `isqrt(disc) < 2^63` (`bit 63 == 0` for all real roots!).
     * Encoding `d_mag_q = 64'h8000_0000_0000_0000` when `disc < 0` carries `!real_roots` in bit 63
     * with ZERO extra flip-flops! */
    static const char *cb_lhs[8] = {
        "n_b0_g1_ll_q  <= n_b0_g1_ll_c;",
        "n_b1_g1_hl_q  <= n_b1_g1_hl_c;",
        "n_b2_g1_hh_q  <= n_b2_g1_hh_c;",
        "n_b3_g0r2_0_q <= n_b3_g0r2_0_c;",
        "n_b4_g0r2_1_q <= n_b4_g0r2_1_c;",
        "n_b5_g0r2_2_q <= n_b5_g0r2_2_c;",
        "n_b6_g1_sq_q  <= n_b6_g1_sq_c;",
        "n_b7_g0_r2_q  <= n_b7_g0_r2_c;",
    };
    for (int s = 0; s < cb->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_cb + s);
        for (int n = 0; n < 8; ++n) {
            if (cb->nodes[n].stage == s && cb->nodes[n].needs_reg) {
                fprintf(fp, "                    %s\n", cb_lhs[n]);
            }
        }
        if (s < cb->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_cb + s + 1);
        } else {
            fprintf(fp, "                    if (n_b8_disc_c[127]) begin\n");
            fprintf(fp, "                        d_mag_q <= 64'h8000_0000_0000_0000;\n");
            fprintf(fp, "                        st_q    <= %d'd%d;\n", st_w, base_cd);
            fprintf(fp, "                    end else if (!n_b8_pos_nz_c) begin\n");
            fprintf(fp, "                        d_mag_q <= 64'd0;\n");
            fprintf(fp, "                        st_q    <= %d'd%d;\n", st_w, base_cd);
            fprintf(fp, "                    end else begin\n");
            fprintf(fp, "                        sr_rem_q <= 64'd0;\n");
            fprintf(fp, "                        sr_acc_q <= 64'd0;\n");
            fprintf(fp, "                        sr_aux_q <= $unsigned(n_b8_disc_c);\n");
            fprintf(fp, "                        iter_q   <= %d'd%d;\n", iter_w, rad_iters - 1);
            fprintf(fp, "                        st_q     <= %d'd%d;\n", st_w, st_isqrt);
            fprintf(fp, "                    end\n");
        }
        fprintf(fp, "                end\n");
    }

    /* ST_ISQRT_LOOP */
    fprintf(fp, "                %d'd%d: begin\n", st_w, st_isqrt);
    fprintf(fp, "                    sr_rem_q <= rad2_rem_w[%d];\n", k_rad);
    fprintf(fp, "                    sr_acc_q <= rad2_acc_w[%d];\n", k_rad);
    fprintf(fp, "                    sr_aux_q <= rad2_aux_w[%d];\n", k_rad);
    fprintf(fp, "                    if (iter_q == %d'd0) begin\n", iter_w);
    fprintf(fp, "                        d_mag_q <= rad2_acc_w[%d] + {63'd0, (rad2_rem_w[%d] > rad2_acc_w[%d])};\n",
            k_rad, k_rad, k_rad);
    fprintf(fp, "                        st_q    <= %d'd%d;\n", st_w, base_cd);
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        iter_q <= iter_q - %d'd1;\n", iter_w);
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    /* Cone D Stages (`Citardauq & Divider Normalization`) */
    for (int s = 0; s < cd->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_cd + s);
        if (cd->nodes[0].stage == s && cd->nodes[0].needs_reg) {
            fprintf(fp, "                    n_d0_div_num_q <= n_d0_div_num_c;\n");
            fprintf(fp, "                    n_d0_div_den_q <= n_d0_div_den_c;\n");
            fprintf(fp, "                    n_d0_div_neg_q <= n_d0_div_neg_c;\n");
        }
        if (s < cd->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_cd + s + 1);
        } else {
            const char *dneg_src = ref_sig(cd, 0, 1, "n_d0_div_neg_c", "n_d0_div_neg_q");
            fprintf(fp, "                    div_neg_q <= %s;\n", dneg_src);
            fprintf(fp, "                    pre_sat_q <= n_d1_pre_sat_c;\n");
            fprintf(fp, "                    sr_rem_q  <= n_d1_init_rem_c;\n");
            fprintf(fp, "                    sr_acc_q  <= n_d1_init_quot_c;\n");
            fprintf(fp, "                    sr_aux_q  <= {64'd0, n_d1_safe_den_c};\n");
            fprintf(fp, "                    if (n_d1_pre_sat_c) begin\n");
            fprintf(fp, "                        st_q   <= %d'd%d;\n", st_w, base_cc);
            fprintf(fp, "                    end else begin\n");
            fprintf(fp, "                        iter_q <= %d'd%d;\n", iter_w, rad_iters - 1);
            fprintf(fp, "                        st_q   <= %d'd%d;\n", st_w, st_udiv);
            fprintf(fp, "                    end\n");
        }
        fprintf(fp, "                end\n");
    }

    /* ST_UDIV_LOOP */
    fprintf(fp, "                %d'd%d: begin\n", st_w, st_udiv);
    fprintf(fp, "                    sr_rem_q <= rad2_rem_w[%d];\n", k_rad);
    fprintf(fp, "                    sr_acc_q <= rad2_acc_w[%d];\n", k_rad);
    fprintf(fp, "                    if (iter_q == %d'd0) begin\n", iter_w);
    fprintf(fp, "                        st_q <= %d'd%d;\n", st_w, base_cc);
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        iter_q <= iter_q - %d'd1;\n", iter_w);
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    /* Cone C Stages (`Post-Div, Lyapunov, Rounding & Kan Commit`) */
    for (int s = 0; s < cc->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_cc + s);
        if (cc->nodes[0].stage == s && cc->nodes[0].needs_reg)
            fprintf(fp, "                    n_c0_dc_q       <= n_c0_dc_c;\n");
        if (cc->nodes[1].stage == s && cc->nodes[1].needs_reg)
            fprintf(fp, "                    n_c1_dsq_q      <= n_c1_dsq_c;\n");
        if (cc->nodes[2].stage == s && cc->nodes[2].needs_reg)
            fprintf(fp, "                    n_c2_g1dc_lo_q  <= n_c2_g1dc_lo_c;\n");
        if (cc->nodes[3].stage == s && cc->nodes[3].needs_reg)
            fprintf(fp, "                    n_c3_g1dc_hi_q  <= n_c3_g1dc_hi_c;\n");
        if (cc->nodes[4].stage == s && cc->nodes[4].needs_reg)
            fprintf(fp, "                    n_c4_r2_dc_q    <= n_c4_r2_dc_c;\n");
        if (cc->nodes[5].stage == s && cc->nodes[5].needs_reg)
            fprintf(fp, "                    n_c5_r2dsq_lo_q <= n_c5_r2dsq_lo_c;\n");
        if (cc->nodes[6].stage == s && cc->nodes[6].needs_reg)
            fprintf(fp, "                    n_c6_r2dsq_hi_q <= n_c6_r2dsq_hi_c;\n");
        if (cc->nodes[7].stage == s && cc->nodes[7].needs_reg)
            fprintf(fp, "                    n_c7_g0_cand_q  <= n_c7_g0_cand_c;\n");
        if (cc->nodes[8].stage == s && cc->nodes[8].needs_reg) {
            fprintf(fp, "                    n_c8_g0_post_q  <= n_c8_g0_post_c;\n");
            fprintf(fp, "                    n_c8_g1_post_q  <= n_c8_g1_post_c;\n");
            fprintf(fp, "                    n_c8_xp_q       <= n_c8_xp_c;\n");
            fprintf(fp, "                    n_c8_dt_res_q   <= n_c8_dt_res_c;\n");
        }
        if (cc->nodes[9].stage == s && cc->nodes[9].needs_reg) {
            fprintf(fp, "                    n_c9_j0_next_q  <= n_c9_j0_next_c;\n");
            fprintf(fp, "                    n_c9_j1_next_q  <= n_c9_j1_next_c;\n");
            fprintf(fp, "                    n_c9_sub_next_q <= n_c9_sub_next_c;\n");
        }
        if (s < cc->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_cc + s + 1);
        } else {
            const char *j0_fin  = ref_sig(cc, 9, 10, "n_c9_j0_next_c",  "n_c9_j0_next_q");
            const char *j1_fin  = ref_sig(cc, 9, 10, "n_c9_j1_next_c",  "n_c9_j1_next_q");
            const char *sub_fin = ref_sig(cc, 9, 10, "n_c9_sub_next_c", "n_c9_sub_next_q");
            const char *xp_fin  = ref_sig(cc, 8, 10, "n_c8_xp_c",       "n_c8_xp_q");
            const char *dt_fin  = ref_sig(cc, 8, 10, "n_c8_dt_res_c",   "n_c8_dt_res_q");
            fprintf(fp, "                    state_w_q[0] <= %s;\n", j0_fin);
            fprintf(fp, "                    state_w_q[1] <= %s;\n", j1_fin);
            fprintf(fp, "                    state_w_q[2] <= {{32{r2_q[31]}}, $unsigned(r2_q)};\n");
            fprintf(fp, "                    state_w_q[3] <= {{32{%s[31]}}, $unsigned(%s)};\n", xp_fin, xp_fin);
            fprintf(fp, "                    state_w_q[4] <= %s;\n", dt_fin);
            fprintf(fp, "                    state_w_q[5] <= {32'd0, n_c10_chart_next_c, n_c10_mode_proj_c};\n");
            fprintf(fp, "                    state_w_q[6] <= {state_w_q[6][63:32] + 32'd1, 31'd0, n_c10_conv_c};\n");
            fprintf(fp, "                    state_w_q[7] <= %s;\n", sub_fin);
            fprintf(fp, "                    fast_path_q  <= 1'b0;\n");
            fprintf(fp, "                    done_q       <= 1'b1;\n");
            fprintf(fp, "                    st_q         <= %d'd%d;\n", st_w, st_idle);
        }
        fprintf(fp, "                end\n");
    }

    /* SEU Fault Recovery Default */
    fprintf(fp, "                default: begin\n");
    fprintf(fp, "                    fast_path_q <= 1'b0;\n");
    fprintf(fp, "                    st_q        <= %d'd0;\n", st_w);
    fprintf(fp, "                end\n");
    fprintf(fp, "            endcase\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    assign ready_o   = (st_q == %d'd0);\n", st_w);
    fprintf(fp, "    assign done_o    = done_q;\n");
    fprintf(fp, "    assign rd_data_o = state_w_q[rd_idx_i];\n\n");

    /* Internal Datapath DFF Toggle Snapshot for Wind-Tunnel Energy Telemetry */
    const int base_dff_bits = st_w + iter_w + 1 + 1 + 128 + 64 + 32 + 64 + 1 + 1 + 64 + 64 + 128;
    const int snap_bits     = base_dff_bits + total_cut_bits;
    const int pad_bits      = 4096 - snap_bits;
    fprintf(fp, "`ifndef SYNTHESIS\n");
    fprintf(fp, "    /* verilator lint_off UNUSEDPARAM */\n");
    fprintf(fp, "    /* verilator lint_off UNUSEDSIGNAL */\n");
    fprintf(fp, "    localparam int PROFILE_DSP_COUNT = %d;\n", dsp_count);
    fprintf(fp, "    localparam int PROFILE_MAX_DEPTH = %d;\n", max_depth);
    fprintf(fp, "    localparam int CORE_DFF_BITS     = %d;\n", snap_bits);
    fprintf(fp, "    logic [4095:0] sim_dff_snap_w;\n");
    fprintf(fp, "    assign sim_dff_snap_w = {\n");
    fprintf(fp, "        %d'd0,\n", pad_bits);
    fprintf(fp, "        st_q, iter_q, op_mask_q, fast_path_q, g0_q, g1_q, r2_q, d_mag_q,\n");
    fprintf(fp, "        div_neg_q, pre_sat_q, sr_rem_q, sr_acc_q, sr_aux_q");
    if (ca->nodes[0].needs_reg) fprintf(fp, ",\n        n_a0_ug_u_q");
    if (ca->nodes[1].needs_reg) fprintf(fp, ",\n        n_a1_b1_x_q");
    if (ca->nodes[2].needs_reg) fprintf(fp, ",\n        n_a2_x_sq_q");
    if (ca->nodes[3].needs_reg) fprintf(fp, ",\n        n_a3_b2_x_q");
    if (ca->nodes[4].needs_reg) fprintf(fp, ",\n        n_a4_b0_eff_q");
    if (ca->nodes[5].needs_reg) fprintf(fp, ",\n        n_a5_b2xsq_lo_q");
    if (ca->nodes[6].needs_reg) fprintf(fp, ",\n        n_a6_b2xsq_hi_q");
    if (ca->nodes[7].needs_reg) fprintf(fp, ",\n        n_a7_g0_op_q");
    if (ca->nodes[8].needs_reg) fprintf(fp, ",\n        n_a8_g1_op_q");
    if (ca->nodes[9].needs_reg) fprintf(fp, ",\n        n_a9_g0_au_q, n_a9_j1_q32_q");
    if (cb->nodes[0].needs_reg) fprintf(fp, ",\n        n_b0_g1_ll_q");
    if (cb->nodes[1].needs_reg) fprintf(fp, ",\n        n_b1_g1_hl_q");
    if (cb->nodes[2].needs_reg) fprintf(fp, ",\n        n_b2_g1_hh_q");
    if (cb->nodes[3].needs_reg) fprintf(fp, ",\n        n_b3_g0r2_0_q");
    if (cb->nodes[4].needs_reg) fprintf(fp, ",\n        n_b4_g0r2_1_q");
    if (cb->nodes[5].needs_reg) fprintf(fp, ",\n        n_b5_g0r2_2_q");
    if (cb->nodes[6].needs_reg) fprintf(fp, ",\n        n_b6_g1_sq_q");
    if (cb->nodes[7].needs_reg) fprintf(fp, ",\n        n_b7_g0_r2_q");
    if (cd->nodes[0].needs_reg) fprintf(fp, ",\n        n_d0_div_num_q, n_d0_div_den_q, n_d0_div_neg_q");
    if (cc->nodes[0].needs_reg) fprintf(fp, ",\n        n_c0_dc_q");
    if (cc->nodes[1].needs_reg) fprintf(fp, ",\n        n_c1_dsq_q");
    if (cc->nodes[2].needs_reg) fprintf(fp, ",\n        n_c2_g1dc_lo_q");
    if (cc->nodes[3].needs_reg) fprintf(fp, ",\n        n_c3_g1dc_hi_q");
    if (cc->nodes[4].needs_reg) fprintf(fp, ",\n        n_c4_r2_dc_q");
    if (cc->nodes[5].needs_reg) fprintf(fp, ",\n        n_c5_r2dsq_lo_q");
    if (cc->nodes[6].needs_reg) fprintf(fp, ",\n        n_c6_r2dsq_hi_q");
    if (cc->nodes[7].needs_reg) fprintf(fp, ",\n        n_c7_g0_cand_q");
    if (cc->nodes[8].needs_reg) fprintf(fp, ",\n        n_c8_g0_post_q, n_c8_g1_post_q, n_c8_xp_q, n_c8_dt_res_q");
    if (cc->nodes[9].needs_reg) fprintf(fp, ",\n        n_c9_j0_next_q, n_c9_j1_next_q, n_c9_sub_next_q");
    fprintf(fp, "\n    };\n");
    fprintf(fp, "    /* verilator lint_on UNUSEDSIGNAL */\n");
    fprintf(fp, "    /* verilator lint_on UNUSEDPARAM */\n");
    fprintf(fp, "`endif\n\n");

    /* Formal SAT assertions for `ele_flow_core` */
    fprintf(fp, "`ifdef FORMAL\n");
    fprintf(fp, "    localparam int NUM_STATES = %d;\n", num_st);
    fprintf(fp, "    logic               f_past_valid_q;\n");
    fprintf(fp, "    logic               f_past_rst_n_q;\n");
    fprintf(fp, "    logic               f_past_ready_q;\n");
    fprintf(fp, "    logic [%d:0]        f_past_st_q;\n", st_w - 1);
    fprintf(fp, "    logic [511:0]       f_past_state_w_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_b0_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_b1_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_b2_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_b_exit_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_b_enter_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_dt_max_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_u_gain_q;\n");
    fprintf(fp, "    logic signed [31:0] f_past_u_in_q;\n");
    fprintf(fp, "    logic [7:0]         f_past_next_lo_q;\n");
    fprintf(fp, "    logic [7:0]         f_past_next_hi_q;\n");
    fprintf(fp, "    logic [15:0]        f_past_cap_q;\n");
    fprintf(fp, "    logic [15:0]        f_past_mreq_q;\n");
    fprintf(fp, "    logic [15:0]        f_past_mode_q;\n");
    fprintf(fp, "    logic [15:0]        f_past_lock_q;\n");
    fprintf(fp, "    initial f_past_valid_q = 1'b0;\n");
    fprintf(fp, "    always @(posedge clk) begin\n");
    fprintf(fp, "        f_past_valid_q   <= 1'b1;\n");
    fprintf(fp, "        f_past_rst_n_q   <= rst_n;\n");
    fprintf(fp, "        f_past_ready_q   <= ready_o;\n");
    fprintf(fp, "        f_past_st_q      <= st_q;\n");
    fprintf(fp, "        f_past_state_w_q <= state_w_q;\n");
    fprintf(fp, "        f_past_b0_q      <= b0_q;\n");
    fprintf(fp, "        f_past_b1_q      <= b1_q;\n");
    fprintf(fp, "        f_past_b2_q      <= b2_q;\n");
    fprintf(fp, "        f_past_b_exit_q  <= b_exit_q;\n");
    fprintf(fp, "        f_past_b_enter_q <= b_enter_q;\n");
    fprintf(fp, "        f_past_dt_max_q  <= dt_max_q;\n");
    fprintf(fp, "        f_past_u_gain_q  <= u_gain_q;\n");
    fprintf(fp, "        f_past_u_in_q    <= u_in_q;\n");
    fprintf(fp, "        f_past_next_lo_q <= next_lo_q;\n");
    fprintf(fp, "        f_past_next_hi_q <= next_hi_q;\n");
    fprintf(fp, "        f_past_cap_q     <= cap_mask16_q;\n");
    fprintf(fp, "        f_past_mreq_q    <= mode_req_q;\n");
    fprintf(fp, "        f_past_mode_q    <= state_w_q[5][15:0];\n");
    fprintf(fp, "        f_past_lock_q    <= kan_lock_q;\n");
    fprintf(fp, "    end\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            assert (st_q == %d'd0);\n", st_w);
    fprintf(fp, "            assert (ready_o && !done_o && !fast_path_q);\n");
    fprintf(fp, "            assert (dt_max_q == 32'sd1);\n");
    fprintf(fp, "            assert (cap_mask16_q == 16'hFFFF);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if (ready_o) begin\n");
    for (int d = 0; d < dsp_count; ++d) {
        fprintf(fp, "            assert (dsp_en_c[%d] == 1'b0);\n", d);
    }
    fprintf(fp, "            assert (rad2_en_c == 1'b0);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin\n");
    fprintf(fp, "            if (!f_past_ready_q) begin\n");
    fprintf(fp, "                assert (b0_q == f_past_b0_q);\n");
    fprintf(fp, "                assert (b1_q == f_past_b1_q);\n");
    fprintf(fp, "                assert (b2_q == f_past_b2_q);\n");
    fprintf(fp, "                assert (b_exit_q == f_past_b_exit_q);\n");
    fprintf(fp, "                assert (b_enter_q == f_past_b_enter_q);\n");
    fprintf(fp, "                assert (dt_max_q == f_past_dt_max_q);\n");
    fprintf(fp, "                assert (u_gain_q == f_past_u_gain_q);\n");
    fprintf(fp, "                assert (u_in_q == f_past_u_in_q);\n");
    fprintf(fp, "                assert (next_lo_q == f_past_next_lo_q);\n");
    fprintf(fp, "                assert (next_hi_q == f_past_next_hi_q);\n");
    fprintf(fp, "                assert (kan_lock_q == f_past_lock_q);\n");
    fprintf(fp, "                assert (cap_mask16_q == f_past_cap_q);\n");
    fprintf(fp, "                assert (mode_req_q == f_past_mreq_q);\n");
    fprintf(fp, "                if (int'(f_past_st_q) != (NUM_STATES - 1)) begin\n");
    fprintf(fp, "                    assert (!done_o);\n");
    fprintf(fp, "                    assert (state_w_q == f_past_state_w_q);\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "            if (int'(f_past_st_q) >= NUM_STATES) begin\n");
    fprintf(fp, "                assert (st_q == %d'd0);\n", st_w);
    fprintf(fp, "                assert (ready_o && !done_o && !fast_path_q);\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "            if (done_o) begin\n");
    fprintf(fp, "                assert (ready_o);\n");
    fprintf(fp, "                assert (int'(f_past_st_q) == (NUM_STATES - 1));\n");
    fprintf(fp, "                assert ((state_w_q[5][15:0] & f_past_lock_q) == (f_past_mode_q & f_past_lock_q));\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    end\n");
    fprintf(fp, "`endif\n");
    fprintf(fp, "endmodule\n\n");

    /* ======================================================================
     * MODULE 2: `ele_flow_atlas_top` (157 physical pins, `ele_sram_1rw64` Arena)
     * ====================================================================== */
    fprintf(fp, "module ele_flow_atlas_top #(\n");
    fprintf(fp, "    parameter int MAX_CHARTS = 16\n");
    fprintf(fp, ") (\n");
    fprintf(fp, "    input  logic        clk,\n");
    fprintf(fp, "    input  logic        rst_n,\n");
    fprintf(fp, "    input  logic        cmd_valid_i,\n");
    fprintf(fp, "    input  logic [2:0]  cmd_op_i,\n");
    fprintf(fp, "    input  logic [2:0]  cmd_word_i,\n");
    fprintf(fp, "    input  logic [15:0] cmd_aux_i,\n");
    fprintf(fp, "    input  logic [63:0] cmd_data_i,\n");
    fprintf(fp, "    output logic        ready_o,\n");
    fprintf(fp, "    output logic        step_done_o,\n");
    fprintf(fp, "    output logic        swap_done_o,\n");
    fprintf(fp, "    output logic        swap_accepted_o,\n");
    fprintf(fp, "    output logic [63:0] rd_data_o\n");
    fprintf(fp, ");\n");
    fprintf(fp, "    localparam int SLOT_W     = (MAX_CHARTS <= 2) ? 1 : $clog2(MAX_CHARTS);\n");
    fprintf(fp, "    localparam int SRAM_DEPTH = MAX_CHARTS * 8;\n");
    fprintf(fp, "    localparam int SRAM_AW    = $clog2(SRAM_DEPTH);\n");
    fprintf(fp, "    localparam logic [15:0] HW_MASK16 = 16'(MAX_CHARTS - 1);\n\n");
    fprintf(fp, "    // 64-bit Bus Command Opcodes\n");
    fprintf(fp, "    localparam logic [2:0] CMD_NOP         = 3'd0;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_LOAD_STATE  = 3'd1;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_SET_CAP     = 3'd2;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_WRITE_CHART = 3'd3;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_STAGE_CAND  = 3'd4;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_STEP        = 3'd5;\n");
    fprintf(fp, "    localparam logic [2:0] CMD_SWAP        = 3'd6;\n\n");
    fprintf(fp, "    localparam logic [2:0] TOP_IDLE        = 3'd0;\n");
    fprintf(fp, "    localparam logic [2:0] TOP_FETCH_BOUND = 3'd1;\n");
    fprintf(fp, "    localparam logic [2:0] TOP_START_CORE  = 3'd2;\n");
    fprintf(fp, "    localparam logic [2:0] TOP_WAIT_CORE   = 3'd3;\n");
    fprintf(fp, "    localparam logic [2:0] TOP_SWAP_CHK    = 3'd4;\n");
    fprintf(fp, "    localparam logic [2:0] TOP_SWAP_COMMIT = 3'd5;\n\n");

    fprintf(fp, "    logic [2:0]            top_st_q;\n");
    fprintf(fp, "    logic [2:0]            beat_q;\n");
    fprintf(fp, "    logic [15:0]           cap_mask_q;\n");
    fprintf(fp, "    logic [MAX_CHARTS-1:0] slot_valid_q;\n");
    fprintf(fp, "    logic [SLOT_W-1:0]     act_slot_q;\n");
    fprintf(fp, "    logic [15:0]           mode_hold_q;\n");
    fprintf(fp, "    logic [63:0]           u_ext_hold_q;\n");
    fprintf(fp, "    logic [5:0][63:0]      cand_buf_q;\n");
    fprintf(fp, "    logic [SLOT_W-1:0]     swap_slot_q;\n");
    fprintf(fp, "    logic [31:0]           swap_epoch_q;\n");
    fprintf(fp, "    logic                  step_done_q;\n");
    fprintf(fp, "    logic                  swap_done_q;\n");
    fprintf(fp, "    logic                  swap_acc_q;\n\n");

    /* SRAM interface wires */
    fprintf(fp, "    logic                  sram_ce_c;\n");
    fprintf(fp, "    logic                  sram_we_c;\n");
    fprintf(fp, "    logic [SRAM_AW-1:0]    sram_addr_c;\n");
    fprintf(fp, "    logic [63:0]           sram_wdata_c;\n");
    fprintf(fp, "    logic [63:0]           sram_rdata_w;\n\n");

    fprintf(fp, "    ele_sram_1rw64 #(\n");
    fprintf(fp, "        .DEPTH  (SRAM_DEPTH),\n");
    fprintf(fp, "        .ADDR_W (SRAM_AW)\n");
    fprintf(fp, "    ) u_chart_sram (\n");
    fprintf(fp, "        .clk     (clk),\n");
    fprintf(fp, "        .ce_i    (sram_ce_c),\n");
    fprintf(fp, "        .we_i    (sram_we_c),\n");
    fprintf(fp, "        .addr_i  (sram_addr_c),\n");
    fprintf(fp, "        .wdata_i (sram_wdata_c),\n");
    fprintf(fp, "        .rdata_o (sram_rdata_w)\n");
    fprintf(fp, "    );\n\n");

    /* Core interface wires */
    fprintf(fp, "    logic        core_wr_valid_c;\n");
    fprintf(fp, "    logic [1:0]  core_wr_kind_c;\n");
    fprintf(fp, "    logic [2:0]  core_wr_idx_c;\n");
    fprintf(fp, "    logic [63:0] core_wr_data_c;\n");
    fprintf(fp, "    logic        core_ready_w;\n");
    fprintf(fp, "    logic        core_done_w;\n");
    fprintf(fp, "    logic [2:0]  core_rd_idx_c;\n");
    fprintf(fp, "    logic [63:0] core_rd_data_w;\n\n");

    fprintf(fp, "    ele_flow_core u_core (\n");
    fprintf(fp, "        .clk        (clk),\n");
    fprintf(fp, "        .rst_n      (rst_n),\n");
    fprintf(fp, "        .wr_valid_i (core_wr_valid_c),\n");
    fprintf(fp, "        .wr_kind_i  (core_wr_kind_c),\n");
    fprintf(fp, "        .wr_idx_i   (core_wr_idx_c),\n");
    fprintf(fp, "        .wr_data_i  (core_wr_data_c),\n");
    fprintf(fp, "        .ready_o    (core_ready_w),\n");
    fprintf(fp, "        .done_o     (core_done_w),\n");
    fprintf(fp, "        .rd_idx_i   (core_rd_idx_c),\n");
    fprintf(fp, "        .rd_data_o  (core_rd_data_w)\n");
    fprintf(fp, "    );\n\n");

    /* Firewall check combinational signals */
    fprintf(fp, "    logic [31:0] swap_cur_ep_c;\n");
    fprintf(fp, "    logic        swap_fw_ok_c;\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        swap_cur_ep_c = slot_valid_q[swap_slot_q] ? sram_rdata_w[63:32] : 32'd0;\n");
    fprintf(fp, "        swap_fw_ok_c  = (swap_epoch_q > swap_cur_ep_c) &\n");
    fprintf(fp, "                        ({8'd0, cand_buf_q[5][31:24]} <= cap_mask_q) &\n");
    fprintf(fp, "                        ({8'd0, cand_buf_q[5][23:16]} <= cap_mask_q) &\n");
    fprintf(fp, "                        ($signed(cand_buf_q[4][63:32]) > 32'sd0) &\n");
    fprintf(fp, "                        ($signed(cand_buf_q[3][63:32]) <= $signed(cand_buf_q[3][31:0]));\n");
    fprintf(fp, "    end\n\n");

    /* Combinational control for `u_chart_sram` and `u_core` */
    fprintf(fp, "    logic [SLOT_W-1:0] step_slot_c;\n");
    fprintf(fp, "    logic [SLOT_W-1:0] swap_req_slot_c;\n");
    fprintf(fp, "    logic [SLOT_W-1:0] wr_chart_slot_c;\n");
    fprintf(fp, "    wire               unused_aux_hi_ok = &{1'b0, cmd_aux_i[15:SLOT_W]};\n\n");

    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        core_rd_idx_c   = (top_st_q == TOP_IDLE && cmd_valid_i && cmd_op_i == CMD_STEP) ? 3'd5 : cmd_word_i;\n");
    fprintf(fp, "        step_slot_c     = core_rd_data_w[16 +: SLOT_W] & cap_mask_q[SLOT_W-1:0];\n");
    fprintf(fp, "        swap_req_slot_c = cmd_aux_i[SLOT_W-1:0] & cap_mask_q[SLOT_W-1:0];\n");
    fprintf(fp, "        wr_chart_slot_c = cmd_aux_i[SLOT_W-1:0];\n\n");
    fprintf(fp, "        sram_ce_c       = 1'b0;\n");
    fprintf(fp, "        sram_we_c       = 1'b0;\n");
    fprintf(fp, "        sram_addr_c     = {SRAM_AW{1'b0}};\n");
    fprintf(fp, "        sram_wdata_c    = 64'd0;\n\n");
    fprintf(fp, "        core_wr_valid_c = 1'b0;\n");
    fprintf(fp, "        core_wr_kind_c  = 2'd0;\n");
    fprintf(fp, "        core_wr_idx_c   = 3'd0;\n");
    fprintf(fp, "        core_wr_data_c  = 64'd0;\n\n");

    fprintf(fp, "        case (top_st_q)\n");
    fprintf(fp, "            TOP_IDLE: begin\n");
    fprintf(fp, "                if (cmd_valid_i && core_ready_w) begin\n");
    fprintf(fp, "                    case (cmd_op_i)\n");
    fprintf(fp, "                        CMD_LOAD_STATE: begin\n");
    fprintf(fp, "                            core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                            core_wr_kind_c  = 2'd0;\n");
    fprintf(fp, "                            core_wr_idx_c   = cmd_word_i;\n");
    fprintf(fp, "                            core_wr_data_c  = cmd_data_i;\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_WRITE_CHART: begin\n");
    fprintf(fp, "                            sram_ce_c    = 1'b1;\n");
    fprintf(fp, "                            sram_we_c    = 1'b1;\n");
    fprintf(fp, "                            sram_addr_c  = {wr_chart_slot_c, cmd_word_i};\n");
    fprintf(fp, "                            sram_wdata_c = cmd_data_i;\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_STEP: begin\n");
    fprintf(fp, "                            sram_ce_c       = 1'b1;\n");
    fprintf(fp, "                            sram_we_c       = 1'b0;\n");
    fprintf(fp, "                            sram_addr_c     = {step_slot_c, 3'd0};\n");
    fprintf(fp, "                            core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                            core_wr_kind_c  = 2'd2;\n");
    fprintf(fp, "                            core_wr_idx_c   = 3'd0;\n");
    fprintf(fp, "                            core_wr_data_c  = {32'd0, cmd_aux_i, cap_mask_q};\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_SWAP: begin\n");
    fprintf(fp, "                            sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                            sram_we_c   = 1'b0;\n");
    fprintf(fp, "                            sram_addr_c = {swap_req_slot_c, 3'd6};\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        default: begin end\n");
    fprintf(fp, "                    endcase\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    /* TOP_FETCH_BOUND: `beat_q = 0..5` receives SRAM read of word `beat_q` and issues read `beat_q + 1` */
    fprintf(fp, "            TOP_FETCH_BOUND: begin\n");
    fprintf(fp, "                core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                core_wr_kind_c  = 2'd1;\n");
    fprintf(fp, "                core_wr_idx_c   = beat_q;\n");
    fprintf(fp, "                core_wr_data_c  = slot_valid_q[act_slot_q] ? sram_rdata_w : 64'd0;\n");
    fprintf(fp, "                if (beat_q < 3'd5) begin\n");
    fprintf(fp, "                    sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                    sram_we_c   = 1'b0;\n");
    fprintf(fp, "                    sram_addr_c = {act_slot_q, beat_q + 3'd1};\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    fprintf(fp, "            TOP_START_CORE: begin\n");
    fprintf(fp, "                core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                core_wr_kind_c  = 2'd2;\n");
    fprintf(fp, "                core_wr_idx_c   = 3'd1;\n");
    fprintf(fp, "                core_wr_data_c  = u_ext_hold_q;\n");
    fprintf(fp, "            end\n");

    fprintf(fp, "            TOP_SWAP_CHK: begin\n");
    fprintf(fp, "                if (swap_fw_ok_c) begin\n");
    fprintf(fp, "                    sram_ce_c    = 1'b1;\n");
    fprintf(fp, "                    sram_we_c    = 1'b1;\n");
    fprintf(fp, "                    sram_addr_c  = {swap_slot_q, 3'd0};\n");
    fprintf(fp, "                    sram_wdata_c = cand_buf_q[0];\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    /* Release-ordered 64b SRAM commit: beat 1..5 -> w1..w5, beat 6 -> w7=0, beat 7 -> w6={epoch,0} LAST! */
    fprintf(fp, "            TOP_SWAP_COMMIT: begin\n");
    fprintf(fp, "                sram_ce_c = 1'b1;\n");
    fprintf(fp, "                sram_we_c = 1'b1;\n");
    fprintf(fp, "                if (beat_q < 3'd6) begin\n");
    fprintf(fp, "                    sram_addr_c  = {swap_slot_q, beat_q};\n");
    fprintf(fp, "                    sram_wdata_c = cand_buf_q[beat_q];\n");
    fprintf(fp, "                end else if (beat_q == 3'd6) begin\n");
    fprintf(fp, "                    sram_addr_c  = {swap_slot_q, 3'd7};\n");
    fprintf(fp, "                    sram_wdata_c = 64'd0;\n");
    fprintf(fp, "                end else begin\n");
    fprintf(fp, "                    sram_addr_c  = {swap_slot_q, 3'd6};\n");
    fprintf(fp, "                    sram_wdata_c = {swap_epoch_q, 32'd0};\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    fprintf(fp, "            default: begin end\n");
    fprintf(fp, "        endcase\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    always_ff @(posedge clk or negedge rst_n) begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            top_st_q     <= TOP_IDLE;\n");
    fprintf(fp, "            beat_q       <= 3'd0;\n");
    fprintf(fp, "            cap_mask_q   <= HW_MASK16;\n");
    fprintf(fp, "            slot_valid_q <= {MAX_CHARTS{1'b0}};\n");
    fprintf(fp, "            act_slot_q   <= {SLOT_W{1'b0}};\n");
    fprintf(fp, "            mode_hold_q  <= 16'd0;\n");
    fprintf(fp, "            u_ext_hold_q <= 64'd0;\n");
    fprintf(fp, "            cand_buf_q   <= 384'd0;\n");
    fprintf(fp, "            swap_slot_q  <= {SLOT_W{1'b0}};\n");
    fprintf(fp, "            swap_epoch_q <= 32'd0;\n");
    fprintf(fp, "            step_done_q  <= 1'b0;\n");
    fprintf(fp, "            swap_done_q  <= 1'b0;\n");
    fprintf(fp, "            swap_acc_q   <= 1'b0;\n");
    fprintf(fp, "        end else begin\n");
    fprintf(fp, "            step_done_q <= 1'b0;\n");
    fprintf(fp, "            swap_done_q <= 1'b0;\n");
    fprintf(fp, "            swap_acc_q  <= 1'b0;\n\n");

    fprintf(fp, "            case (top_st_q)\n");
    fprintf(fp, "                TOP_IDLE: begin\n");
    fprintf(fp, "                    if (cmd_valid_i && core_ready_w) begin\n");
    fprintf(fp, "                        case (cmd_op_i)\n");
    fprintf(fp, "                            CMD_SET_CAP: begin\n");
    fprintf(fp, "                                cap_mask_q <= cmd_data_i[15:0] & HW_MASK16;\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_STAGE_CAND: begin\n");
    fprintf(fp, "                                if (cmd_word_i < 3'd6) begin\n");
    fprintf(fp, "                                    cand_buf_q[cmd_word_i] <= cmd_data_i;\n");
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_WRITE_CHART: begin\n");
    fprintf(fp, "                                if (cmd_word_i == 3'd7) begin\n");
    fprintf(fp, "                                    slot_valid_q[wr_chart_slot_c] <= 1'b1;\n");
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_STEP: begin\n");
    fprintf(fp, "                                act_slot_q   <= step_slot_c;\n");
    fprintf(fp, "                                mode_hold_q  <= cmd_aux_i;\n");
    fprintf(fp, "                                u_ext_hold_q <= cmd_data_i;\n");
    fprintf(fp, "                                beat_q       <= 3'd0;\n");
    fprintf(fp, "                                top_st_q     <= TOP_FETCH_BOUND;\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_SWAP: begin\n");
    fprintf(fp, "                                swap_slot_q  <= swap_req_slot_c;\n");
    fprintf(fp, "                                swap_epoch_q <= cmd_data_i[31:0];\n");
    fprintf(fp, "                                top_st_q     <= TOP_SWAP_CHK;\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            default: begin end\n");
    fprintf(fp, "                        endcase\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_FETCH_BOUND: begin\n");
    fprintf(fp, "                    if (beat_q == 3'd5) begin\n");
    fprintf(fp, "                        top_st_q <= TOP_START_CORE;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        beat_q <= beat_q + 3'd1;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_START_CORE: begin\n");
    fprintf(fp, "                    top_st_q <= TOP_WAIT_CORE;\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_WAIT_CORE: begin\n");
    fprintf(fp, "                    if (core_done_w) begin\n");
    fprintf(fp, "                        step_done_q <= 1'b1;\n");
    fprintf(fp, "                        top_st_q    <= TOP_IDLE;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_SWAP_CHK: begin\n");
    fprintf(fp, "                    if (!swap_fw_ok_c) begin\n");
    fprintf(fp, "                        swap_done_q <= 1'b1;\n");
    fprintf(fp, "                        swap_acc_q  <= 1'b0;\n");
    fprintf(fp, "                        top_st_q    <= TOP_IDLE;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        beat_q   <= 3'd1;\n");
    fprintf(fp, "                        top_st_q <= TOP_SWAP_COMMIT;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_SWAP_COMMIT: begin\n");
    fprintf(fp, "                    if (beat_q == 3'd7) begin\n");
    fprintf(fp, "                        slot_valid_q[swap_slot_q] <= 1'b1;\n");
    fprintf(fp, "                        swap_done_q               <= 1'b1;\n");
    fprintf(fp, "                        swap_acc_q                <= 1'b1;\n");
    fprintf(fp, "                        top_st_q                  <= TOP_IDLE;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        beat_q <= beat_q + 3'd1;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                default: top_st_q <= TOP_IDLE;\n");
    fprintf(fp, "            endcase\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    wire unused_mode_hold_ok = &{1'b0, mode_hold_q, CMD_NOP};\n");
    fprintf(fp, "    assign ready_o         = (top_st_q == TOP_IDLE) & core_ready_w;\n");
    fprintf(fp, "    assign step_done_o     = step_done_q;\n");
    fprintf(fp, "    assign swap_done_o     = swap_done_q;\n");
    fprintf(fp, "    assign swap_accepted_o = swap_acc_q;\n");
    fprintf(fp, "    assign rd_data_o       = core_rd_data_w;\n\n");

    /* Formal SAT assertions for `ele_flow_atlas_top` */
    fprintf(fp, "`ifdef FORMAL\n");
    fprintf(fp, "    logic        f_past_valid_q;\n");
    fprintf(fp, "    logic        f_past_rst_n_q;\n");
    fprintf(fp, "    logic        f_past_ready_q;\n");
    fprintf(fp, "    logic [2:0]  f_past_st_q;\n");
    fprintf(fp, "    logic        f_past_fw_ok_q;\n");
    fprintf(fp, "    logic [15:0] f_past_cap_q;\n");
    fprintf(fp, "    logic [63:0] f_past_cand0_q;\n");
    fprintf(fp, "    logic [63:0] f_past_cand5_q;\n");
    fprintf(fp, "    initial f_past_valid_q = 1'b0;\n");
    fprintf(fp, "    always @(posedge clk) begin\n");
    fprintf(fp, "        f_past_valid_q <= 1'b1;\n");
    fprintf(fp, "        f_past_rst_n_q <= rst_n;\n");
    fprintf(fp, "        f_past_ready_q <= ready_o;\n");
    fprintf(fp, "        f_past_st_q    <= top_st_q;\n");
    fprintf(fp, "        f_past_fw_ok_q <= swap_fw_ok_c;\n");
    fprintf(fp, "        f_past_cap_q   <= cap_mask_q;\n");
    fprintf(fp, "        f_past_cand0_q <= cand_buf_q[0];\n");
    fprintf(fp, "        f_past_cand5_q <= cand_buf_q[5];\n");
    fprintf(fp, "    end\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            assert (top_st_q == TOP_IDLE);\n");
    fprintf(fp, "            assert (!step_done_o && !swap_done_o && !swap_accepted_o);\n");
    fprintf(fp, "            assert (cap_mask_q == HW_MASK16);\n");
    fprintf(fp, "            assert (slot_valid_q == {MAX_CHARTS{1'b0}});\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        assert ((cap_mask_q & (~HW_MASK16)) == 16'd0);\n");
    fprintf(fp, "        if ((top_st_q == TOP_IDLE) && (!core_ready_w)) begin\n");
    fprintf(fp, "            assert (!ready_o && !sram_ce_c && !sram_we_c && !core_wr_valid_c);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_FETCH_BOUND) || (top_st_q == TOP_START_CORE) || (top_st_q == TOP_WAIT_CORE)) begin\n");
    fprintf(fp, "            assert (sram_we_c == 1'b0);\n");
    fprintf(fp, "            assert (!ready_o);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_SWAP_CHK) && (!swap_fw_ok_c)) begin\n");
    fprintf(fp, "            assert (sram_we_c == 1'b0);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin\n");
    fprintf(fp, "            if (!f_past_ready_q) begin\n");
    fprintf(fp, "                assert (cap_mask_q == f_past_cap_q);\n");
    fprintf(fp, "                assert (cand_buf_q[0] == f_past_cand0_q);\n");
    fprintf(fp, "                assert (cand_buf_q[5] == f_past_cand5_q);\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "            if ((f_past_st_q == TOP_WAIT_CORE)) begin\n");
    fprintf(fp, "                assert (!swap_done_o && !swap_accepted_o);\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "            if ((f_past_st_q == TOP_SWAP_CHK) && (!f_past_fw_ok_q)) begin\n");
    fprintf(fp, "                assert (swap_done_o && !swap_accepted_o);\n");
    fprintf(fp, "            end\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    end\n");
    fprintf(fp, "`endif\n");
    fprintf(fp, "endmodule\n\n");

    fprintf(fp, "/* verilator lint_on DECLFILENAME */\n");
    fprintf(fp, "`default_nettype wire\n");
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [--dsp-count <1..6>] [--max-depth <18..64>] -o <out.sv>\n", prog);
}

int main(int argc, char **argv) {
    int         dsp_count = 3;
    int         max_depth = 36;
    const char *out_path  = NULL;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--dsp-count") == 0 && i + 1 < argc) {
            dsp_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--max-depth") == 0 && i + 1 < argc) {
            max_depth = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (out_path == NULL || dsp_count < 1 || dsp_count > 6 || max_depth < 18 || max_depth > 64) {
        usage(argv[0]);
        return 1;
    }

    uint32_t   rad_epochs = 0;
    int        k_rad      = solve_radix_unroll_2jet(max_depth, &rad_epochs);
    dag_cone_t ca, cb, cd, cc;
    build_flow_2jet_cones(&ca, &cb, &cd, &cc);

    solve_cone_with_flow_2jet(&ca, dsp_count, max_depth);
    solve_cone_with_flow_2jet(&cb, dsp_count, max_depth);
    solve_cone_with_flow_2jet(&cd, dsp_count, max_depth);
    solve_cone_with_flow_2jet(&cc, dsp_count, max_depth);

    FILE *fp = fopen(out_path, "w");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    emit_sv(fp, dsp_count, max_depth, k_rad, rad_epochs, &ca, &cb, &cd, &cc);
    fclose(fp);

    const int rad_iters   = 64 / k_rad;
    const int quad_cycles = ca.num_stages + cb.num_stages + rad_iters +
                            cd.num_stages + rad_iters + cc.num_stages;
    const int cut_bits    = ca.cut_reg_bits + cb.cut_reg_bits +
                            cd.cut_reg_bits + cc.cut_reg_bits;
    printf("  [FLOW-BOOTSTRAP] out=%s | dsp=%d max_depth=%d -> K_rad=%d, stages=(%d+%d+%d+%d), cut_dff=%d bits, quad_cyc=%d (CONVERGED)\n",
           out_path, dsp_count, max_depth, k_rad,
           ca.num_stages, cb.num_stages, cd.num_stages, cc.num_stages,
           cut_bits, quad_cycles);
    return 0;
}
