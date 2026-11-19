/* SPDX-License-Identifier: MIT
 * tools/ele_bootstrap.c — Self-Hosting SEARU × FLOW v8.0.0 Symbiotic Silicon Compiler
 *
 * Build-time 3-pass SSA hardware compiler combining continuous FLOW 2-Jet lattice
 * retiming (`../FLOW/core/flow.h`) with discrete SEARU Sparse-k / Kan-pin / Crucible
 * combinatorial optimization (`../FLOW/core/searu.h` + `../FLOW/core/flow_opt_params.h`).
 *
 * Zero external library dependencies (pure GPR C17 + `flow.h` + `searu.h` only):
 *   Pass 1 : Declarative 33-node SSA operator IR (`ssa_op_spec_t`) retimed across
 *            physical constraints (`--dsp-count M`, `--max-depth D`) by evolving a
 *            coupled `flow_cell` 2-Jet manifold to `CONVERGED`, followed by
 *            `SEARU_SOLVE_INLINE` per-stage DSP slot permutation synthesis to
 *            jointly minimize shared cut bits, 33-bit DSP operand fan-in, and
 *            shared-lane mux fan-in.
 *   Pass 2 : Global FSM live-interval analysis `[birth_state, last_read]` seeded by
 *            chordal coloring and refined via `SEARU_SOLVE_INLINE` heterogeneous-width
 *            + mux-affinity register allocation with formal O(N^2) disjointness proof,
 *            plus regime & zero-limb clock-enable / DSP operand isolation.
 *   Pass 3 : Critical-path (`ltp -noff`) carry-select decoupling, O(log W)
 *            power-of-two symmetric range trees, sequential DFF reclamation
 *            (`d_mag_q` folded into `sr_acc_q`, combinational `r2`/`op_mask`,
 *            `pre_sat` + `sr_aux_q[63:0]` divider gating), Active Chart-Bound
 *            Caching (`0` steady-state SRAM reads), and Shadow-SRAM `@swap` staging.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flow.h"
#include "searu.h"
#include "flow_ir.h"
#include "flow_ir_io.h"
#include "ele_fab.h"

#define MAX_CONE_NODES  16
#define MAX_PREDS       6
#define MAX_DSPS        8
#define NUM_SSA_OPS     33
#define MAX_SCALAR_REQS 64
#define MAX_PHYS_LANES  32

typedef enum {
    CONE_A    = 0,
    CONE_B    = 1,
    CONE_D    = 2,
    CONE_C    = 3,
    NUM_CONES = 4
} cone_id_t;

typedef enum {
    PHASE_PRE_LOGIC  = 0,
    PHASE_DSP33      = 1,
    PHASE_POST_LOGIC = 2
} node_phase_t;

typedef enum {
    EXPR_A0_UG_U = 0,
    EXPR_A1_B1_X,
    EXPR_A2_X_SQ,
    EXPR_A3_B2_X,
    EXPR_A4_B0_EFF,
    EXPR_A5_B2XSQ_LO,
    EXPR_A6_B2XSQ_HI,
    EXPR_A7_G0_OP,
    EXPR_A8_G1_OP,
    EXPR_A9_JET_AU,
    EXPR_A10_JET_MUX,
    EXPR_B0_G1_LL,
    EXPR_B1_G1_HL,
    EXPR_B2_G1_HH,
    EXPR_B3_G0R2_0,
    EXPR_B4_G0R2_1,
    EXPR_B5_G0R2_2,
    EXPR_B6_G1_SQ,
    EXPR_B7_G0_R2,
    EXPR_B8_DISC_SUB,
    EXPR_D0_CIT_SEL,
    EXPR_D1_DIV_NORM,
    EXPR_C0_DELTA_CAND,
    EXPR_C1_D_SQ,
    EXPR_C2_G1DC_LO,
    EXPR_C3_G1DC_HI,
    EXPR_C4_R2_DC,
    EXPR_C5_R2DSQ_LO,
    EXPR_C6_R2DSQ_HI,
    EXPR_C7_G0_CAND,
    EXPR_C8_LYAP_GUARD,
    EXPR_C9_POST_ROUND,
    EXPR_C10_KAN_COMMIT
} ssa_expr_id_t;

/* Node descriptors are supplied externally by Lean SSOT `.fir` (`build_cones_from_fir`) */

typedef struct {
    const char   *name;
    node_phase_t  phase;
    int           delay_lvls;
    int           reg_bits;
    int           num_preds;
    int           preds[MAX_PREDS];
    ssa_expr_id_t expr_id;
    /* Solved by FLOW 2-Jet lattice (`flow_step_atlas`) */
    int           stage;
    int           end_frac_q16;
    int           dsp_slot;
    bool          needs_reg;
    uint32_t      flow_epochs;
    uint16_t      final_chart;
} dag_node_t;

typedef struct {
    const char *cone_name;
    int         num_nodes;
    dag_node_t  nodes[MAX_CONE_NODES];
    int         num_stages;
    int         sweeps_taken;
    int         cut_reg_bits;
} dag_cone_t;

/* Pass-2 Global Live-Range Scalar Slice & Physical Shared Register Pool */
typedef struct {
    const char *tag;
    int         birth_state;
    int         last_read_state;
    int         bit_width;        /* Exact active bits in {31, 33, 34, 35, 64, 66} */
    const char *we_guard;         /* Fine-grained clock-enable predicate on write */
    const char *val_expr;         /* Exact `bit_width`-wide unsigned RHS expression */
    int         phys_lane;        /* Assigned physical lane [0 .. num_lanes-1] */
} ssa_scalar_live_t;

typedef struct {
    ssa_scalar_live_t reqs[MAX_SCALAR_REQS];
    int               num_reqs;
    int               num_lanes;
    int               lane_width[MAX_PHYS_LANES];
    int               lane_writers[MAX_PHYS_LANES];
    int               shared_cut_bits;
} reg_alloc_t;

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
    int64_t sq = x_root_q16 * x_root_q16;
    return x_root_q16 + (sq >> 20U);
}

static void init_2jet_node_arena(flow_cell arena[2], bool pin_anchor) {
    const uint64_t b0_c0[8] = {
        0ULL,
        (uint64_t)FLOW_Q16_ONE,
        8192ULL,
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

static void build_cones_from_fir(dag_cone_t cones[NUM_CONES], const flow_ir_prog_t *fir) {
    static const char *k_cone_names[NUM_CONES] = {
        "CONE_A_RESIDUAL",
        "CONE_B_DISC",
        "CONE_D_CITARDAUQ",
        "CONE_C_LYAP_COMMIT",
    };
    static const uint32_t k_cone_base_val[NUM_CONES] = {1U, 12U, 21U, 23U};
    if (fir->mult != FLOW_IR_MULT_CARTESIAN || fir->num_ops != NUM_SSA_OPS ||
        fir->num_inputs != 1U) {
        fprintf(stderr, "FATAL: invalid core .fir morphism metadata\n");
        exit(1);
    }
    memset(cones, 0, sizeof(dag_cone_t) * NUM_CONES);
    for (int c = 0; c < NUM_CONES; ++c) {
        cones[c].cone_name = k_cone_names[c];
    }
    for (uint32_t i = 0U; i < fir->num_ops; ++i) {
        const flow_ir_op_t *op = &fir->ops[i];
        if (op->kind != FLOW_IR_OP_STAGE_PIPE) {
            fprintf(stderr, "FATAL: unexpected op kind %u in core .fir\n", (unsigned)op->kind);
            exit(1);
        }
        uint32_t cone      = (uint32_t)(op->p0 & 0xFFULL);
        uint32_t local_idx = (uint32_t)((op->p0 >> 8U) & 0xFFULL);
        uint32_t phase     = (uint32_t)((op->p0 >> 16U) & 0xFFULL);
        uint32_t expr_id   = (uint32_t)((op->p0 >> 24U) & 0xFFULL);
        if (cone >= NUM_CONES || phase > PHASE_POST_LOGIC || expr_id >= NUM_SSA_OPS) {
            fprintf(stderr, "FATAL: corrupt metadata in core .fir op %u\n", (unsigned)i);
            exit(1);
        }
        dag_cone_t *cn = &cones[cone];
        if ((int)local_idx != cn->num_nodes || cn->num_nodes >= MAX_CONE_NODES) {
            fprintf(stderr, "FATAL: SSA cone index mismatch in core .fir\n");
            exit(1);
        }
        dag_node_t *nd = &cn->nodes[cn->num_nodes++];
        memset(nd, 0, sizeof(*nd));
        nd->name       = fir->vals[op->out_val].name;
        nd->phase      = (node_phase_t)phase;
        nd->delay_lvls = (int)op->p1;
        nd->reg_bits   = (int)op->p2;
        nd->num_preds  = (int)op->num_ins;
        nd->expr_id    = (ssa_expr_id_t)expr_id;
        for (uint16_t p = 0U; p < op->num_ins; ++p) {
            uint32_t src_v = op->ins[p];
            if (src_v < k_cone_base_val[cone]) {
                fprintf(stderr, "FATAL: cross-cone pred in core .fir\n");
                exit(1);
            }
            nd->preds[p] = (int)(src_v - k_cone_base_val[cone]);
        }
    }
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

                bool can_chain = !(pu->phase == PHASE_DSP33 && nd->phase == PHASE_DSP33) &&
                                 (pu->phase <= nd->phase) &&
                                 ((end_frac_u + dur_q16) <= FLOW_Q16_ONE);

                int64_t cand_x = can_chain ? (su * FLOW_Q16_ONE + end_frac_u)
                                           : ((su + 1) * FLOW_Q16_ONE);
                if (cand_x > req_x) {
                    req_x = cand_x;
                }
            }

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

    int stage_dsp_next[64];
    memset(stage_dsp_next, 0, sizeof(stage_dsp_next));
    int max_stage = 0;

    for (int i = 0; i < cone->num_nodes; ++i) {
        dag_node_t *nd    = &cone->nodes[i];
        int64_t     xi    = (int64_t)cells[i][3];
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

/* ============================================================================
 * Pass 2: Global FSM Live-Range Analysis & Chordal Interval-Graph Allocation
 * ============================================================================ */

static int max_consumer_stage(const dag_cone_t *cone, int src_idx) {
    int max_s = cone->nodes[src_idx].stage;
    for (int v = 0; v < cone->num_nodes; ++v) {
        for (int p = 0; p < cone->nodes[v].num_preds; ++p) {
            if (cone->nodes[v].preds[p] == src_idx && cone->nodes[v].stage > max_s) {
                max_s = cone->nodes[v].stage;
            }
        }
    }
    return max_s;
}

static int add_scalar_req(reg_alloc_t *ra, const char *tag, int birth, int last_read,
                          int bit_width, const char *we_guard, const char *val_expr) {
    if (birth >= last_read) {
        return -1;
    }
    if (ra->num_reqs >= MAX_SCALAR_REQS) {
        fprintf(stderr, "FATAL: scalar live-range table overflow!\n");
        exit(1);
    }
    int idx = ra->num_reqs++;
    ssa_scalar_live_t *r = &ra->reqs[idx];
    r->tag             = tag;
    r->birth_state     = birth;
    r->last_read_state = last_read;
    r->bit_width       = bit_width;
    r->we_guard        = we_guard;
    r->val_expr        = val_expr;
    r->phys_lane       = -1;
    return idx;
}

typedef struct {
    int a0, a1, a2, a3, a5, a6, a7_lo, a7_hi, a8, a9_lo, a9_hi, a9_j1;
    int b0, b1, b2, b3, b4, b5, b6_lo, b6_hi, b7_lo, b7_hi;
    int d0_lo, d0_hi, d0_den;
    int c0, c1, c2, c3, c4, c5, c6, c7_lo, c7_hi;
    int c8_lo, c8_hi, c8_g1, c8_xpdt;
    int c9_j0, c9_j1, c9_sub;
} scalar_map_t;

typedef struct {
    uint32_t dsp_uniq_ops_init;
    uint32_t dsp_uniq_ops_best;
    uint32_t p1_iters;
    uint32_t p1_epiphanies;
    uint32_t p1_swans;
    uint64_t p1_energy_init;
    uint64_t p1_energy_best;
    int      cut_bits_init;
    int      cut_bits_best;
    uint32_t lane_mux_bits_init;
    uint32_t lane_mux_bits_best;
    uint32_t p2_iters;
    uint32_t p2_epiphanies;
    uint32_t p2_swans;
    uint64_t p2_energy_init;
    uint64_t p2_energy_best;
} searu_hls_stats_t;

/* Helper: return SV identifier read by consumer node `dst` from producer `src`
 * within the same cone (`_c` if same stage, `_q` alias if across a pipeline cut). */
static const char *ref_sig(const dag_cone_t *c, int src, int dst,
                           const char *comb_id, const char *reg_id) {
    return (c->nodes[src].stage == c->nodes[dst].stage) ? comb_id : reg_id;
}

typedef struct {
    int         cone_id; /* 0=A, 1=B, 2=C */
    int         node_idx;
    int         global_state;
    const char *en_expr;
    const char *op_a_expr;
    const char *op_b_expr;
} dsp_bind_t;

typedef struct {
    char op_a5_b[64];
    char op_a6_b[64];
    char op_c5_b[64];
    char op_c6_b[64];
    char op_c1_d[64];
    char op_c2_d[64];
    char op_c3_d[64];
    char op_c4_d[64];
} dsp_expr_scratch_t;

static void build_dsp_binds(const dag_cone_t cones[NUM_CONES],
                            int base_ca, int base_cb, int base_cc,
                            dsp_expr_scratch_t *sc, dsp_bind_t binds[18]) {
    const dag_cone_t *ca = &cones[CONE_A];
    const dag_cone_t *cb = &cones[CONE_B];
    const dag_cone_t *cc = &cones[CONE_C];

    const char *x_sq_for_a5 = ref_sig(ca, 2, 5, "n_a2_x_sq_c", "n_a2_x_sq_q");
    const char *x_sq_for_a6 = ref_sig(ca, 2, 6, "n_a2_x_sq_c", "n_a2_x_sq_q");
    const char *dc_for_c1   = ref_sig(cc, 0, 1, "n_c0_dc_c",   "n_c0_dc_q");
    const char *dc_for_c2   = ref_sig(cc, 0, 2, "n_c0_dc_c",   "n_c0_dc_q");
    const char *dc_for_c3   = ref_sig(cc, 0, 3, "n_c0_dc_c",   "n_c0_dc_q");
    const char *dc_for_c4   = ref_sig(cc, 0, 4, "n_c0_dc_c",   "n_c0_dc_q");
    const char *dsq_for_c5  = ref_sig(cc, 1, 5, "n_c1_dsq_c",  "n_c1_dsq_q");
    const char *dsq_for_c6  = ref_sig(cc, 1, 6, "n_c1_dsq_c",  "n_c1_dsq_q");

    snprintf(sc->op_a5_b, sizeof(sc->op_a5_b), "{1'b0, %s[31:0]}", x_sq_for_a5);
    snprintf(sc->op_a6_b, sizeof(sc->op_a6_b), "{1'b0, %s[63:32]}", x_sq_for_a6);
    snprintf(sc->op_c5_b, sizeof(sc->op_c5_b), "{1'b0, %s[31:0]}", dsq_for_c5);
    snprintf(sc->op_c6_b, sizeof(sc->op_c6_b), "{%s[63], %s[63:32]}", dsq_for_c6, dsq_for_c6);
    snprintf(sc->op_c1_d, sizeof(sc->op_c1_d), "$unsigned(%s)", dc_for_c1);
    snprintf(sc->op_c2_d, sizeof(sc->op_c2_d), "$unsigned(%s)", dc_for_c2);
    snprintf(sc->op_c3_d, sizeof(sc->op_c3_d), "$unsigned(%s)", dc_for_c3);
    snprintf(sc->op_c4_d, sizeof(sc->op_c4_d), "$unsigned(%s)", dc_for_c4);

    const dsp_bind_t tmp[18] = {
        {0, 0, base_ca + ca->nodes[0].stage, "1'b1",     "{u_gain_q[31], u_gain_q}", "{u_in_q[31], u_in_q}"},
        {0, 1, base_ca + ca->nodes[1].stage, "1'b1",     "{b1_q[31], b1_q}",         "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 2, base_ca + ca->nodes[2].stage, "b2_nz_c",  "{x_in_clamped_c[31], x_in_clamped_c}", "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 3, base_ca + ca->nodes[3].stage, "1'b1",     "{b2_q[31], b2_q}",         "{x_in_clamped_c[31], x_in_clamped_c}"},
        {0, 5, base_ca + ca->nodes[5].stage, "1'b1",     "{b2_q[31], b2_q}",         sc->op_a5_b},
        {0, 6, base_ca + ca->nodes[6].stage, "1'b1",     "{b2_q[31], b2_q}",         sc->op_a6_b},

        {1, 0, base_cb + cb->nodes[0].stage, "1'b1",     "{1'b0, g1_q[31:0]}",       "{1'b0, g1_q[31:0]}"},
        {1, 1, base_cb + cb->nodes[1].stage, "1'b1",     "{g1_q[63], g1_q[63:32]}",  "{1'b0, g1_q[31:0]}"},
        {1, 2, base_cb + cb->nodes[2].stage, "1'b1",     "{g1_q[63], g1_q[63:32]}",  "{g1_q[63], g1_q[63:32]}"},
        {1, 3, base_cb + cb->nodes[3].stage, "1'b1",     "{1'b0, g0_q[31:0]}",       "{r2_q[31], r2_q}"},
        {1, 4, base_cb + cb->nodes[4].stage, "1'b1",     "{1'b0, g0_q[63:32]}",      "{r2_q[31], r2_q}"},
        {1, 5, base_cb + cb->nodes[5].stage, "1'b1",     "{g0_q[95], g0_q[95:64]}",  "{r2_q[31], r2_q}"},

        {2, 1, base_cc + cc->nodes[1].stage, "r2_nz_c",  sc->op_c1_d,                sc->op_c1_d},
        {2, 2, base_cc + cc->nodes[2].stage, "1'b1",     sc->op_c2_d,                "{1'b0, g1_q[31:0]}"},
        {2, 3, base_cc + cc->nodes[3].stage, "1'b1",     sc->op_c3_d,                "{g1_q[63], g1_q[63:32]}"},
        {2, 4, base_cc + cc->nodes[4].stage, "1'b1",     "{r2_q[31], r2_q}",         sc->op_c4_d},
        {2, 5, base_cc + cc->nodes[5].stage, "1'b1",     "{r2_q[31], r2_q}",         sc->op_c5_b},
        {2, 6, base_cc + cc->nodes[6].stage, "1'b1",     "{r2_q[31], r2_q}",         sc->op_c6_b},
    };
    memcpy(binds, tmp, sizeof(tmp));
}

static uint32_t count_unique_dsp_operands(const dag_cone_t cones[NUM_CONES],
                                          int dsp_count, const dsp_bind_t binds[18]) {
    const dag_cone_t *ca = &cones[CONE_A];
    const dag_cone_t *cb = &cones[CONE_B];
    const dag_cone_t *cc = &cones[CONE_C];
    uint32_t total_uniq  = 0U;

    for (int d = 0; d < dsp_count; ++d) {
        const char *ua[18];
        const char *ub[18];
        int na = 0;
        int nb = 0;
        for (int b = 0; b < 18; ++b) {
            const dag_cone_t *ci = (binds[b].cone_id == 0) ? ca : ((binds[b].cone_id == 1) ? cb : cc);
            if (ci->nodes[binds[b].node_idx].dsp_slot != d) {
                continue;
            }
            bool seen_a = false;
            for (int k = 0; k < na; ++k) {
                if (strcmp(ua[k], binds[b].op_a_expr) == 0) {
                    seen_a = true;
                    break;
                }
            }
            if (!seen_a) {
                ua[na++] = binds[b].op_a_expr;
            }
            bool seen_b = false;
            for (int k = 0; k < nb; ++k) {
                if (strcmp(ub[k], binds[b].op_b_expr) == 0) {
                    seen_b = true;
                    break;
                }
            }
            if (!seen_b) {
                ub[nb++] = binds[b].op_b_expr;
            }
        }
        total_uniq += (uint32_t)(na + nb);
    }
    return total_uniq;
}

static uint32_t count_lane_extra_mux_bits(const reg_alloc_t *ra, int lane_uniq_out[MAX_PHYS_LANES]) {
    uint32_t mux_bits = 0U;
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        int         u_w[MAX_SCALAR_REQS];
        const char *u_expr[MAX_SCALAR_REQS];
        int         num_u = 0;
        for (int r = 0; r < ra->num_reqs; ++r) {
            const ssa_scalar_live_t *rq = &ra->reqs[r];
            if (rq->phys_lane != ln) {
                continue;
            }
            bool seen = false;
            for (int u = 0; u < num_u; ++u) {
                if (u_w[u] == rq->bit_width && strcmp(u_expr[u], rq->val_expr) == 0) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                u_w[num_u]    = rq->bit_width;
                u_expr[num_u] = rq->val_expr;
                ++num_u;
            }
        }
        if (lane_uniq_out != NULL) {
            lane_uniq_out[ln] = num_u;
        }
        if (num_u > 1) {
            mux_bits += (uint32_t)(num_u - 1) * (uint32_t)ra->lane_width[ln];
        }
    }
    return mux_bits;
}

static void populate_and_color_cut_registers_greedy(const dag_cone_t cones[NUM_CONES],
                                                    int base_ca, int base_cb, int base_cd, int base_cc,
                                                    reg_alloc_t *ra, scalar_map_t *smap) {
    static char dsp_expr_buf[18][32];
    memset(ra, 0, sizeof(*ra));
    memset(smap, -1, sizeof(*smap));

    const dag_cone_t *ca = &cones[CONE_A];
    const dag_cone_t *cb = &cones[CONE_B];
    const dag_cone_t *cd = &cones[CONE_D];
    const dag_cone_t *cc = &cones[CONE_C];

    snprintf(dsp_expr_buf[0],  sizeof(dsp_expr_buf[0]),  "dsp_prod_w[%d]", ca->nodes[0].dsp_slot);
    snprintf(dsp_expr_buf[1],  sizeof(dsp_expr_buf[1]),  "dsp_prod_w[%d]", ca->nodes[1].dsp_slot);
    snprintf(dsp_expr_buf[2],  sizeof(dsp_expr_buf[2]),  "dsp_prod_w[%d]", ca->nodes[2].dsp_slot);
    snprintf(dsp_expr_buf[3],  sizeof(dsp_expr_buf[3]),  "dsp_prod_w[%d]", ca->nodes[3].dsp_slot);
    snprintf(dsp_expr_buf[4],  sizeof(dsp_expr_buf[4]),  "dsp_prod_w[%d]", ca->nodes[5].dsp_slot);
    snprintf(dsp_expr_buf[5],  sizeof(dsp_expr_buf[5]),  "dsp_prod_w[%d]", ca->nodes[6].dsp_slot);
    snprintf(dsp_expr_buf[6],  sizeof(dsp_expr_buf[6]),  "dsp_prod_w[%d]", cb->nodes[0].dsp_slot);
    snprintf(dsp_expr_buf[7],  sizeof(dsp_expr_buf[7]),  "dsp_prod_w[%d]", cb->nodes[1].dsp_slot);
    snprintf(dsp_expr_buf[8],  sizeof(dsp_expr_buf[8]),  "dsp_prod_w[%d]", cb->nodes[2].dsp_slot);
    snprintf(dsp_expr_buf[9],  sizeof(dsp_expr_buf[9]),  "dsp_prod_w[%d]", cb->nodes[3].dsp_slot);
    snprintf(dsp_expr_buf[10], sizeof(dsp_expr_buf[10]), "dsp_prod_w[%d]", cb->nodes[4].dsp_slot);
    snprintf(dsp_expr_buf[11], sizeof(dsp_expr_buf[11]), "dsp_prod_w[%d]", cb->nodes[5].dsp_slot);
    snprintf(dsp_expr_buf[12], sizeof(dsp_expr_buf[12]), "dsp_prod_w[%d]", cc->nodes[1].dsp_slot);
    snprintf(dsp_expr_buf[13], sizeof(dsp_expr_buf[13]), "dsp_prod_w[%d]", cc->nodes[2].dsp_slot);
    snprintf(dsp_expr_buf[14], sizeof(dsp_expr_buf[14]), "dsp_prod_w[%d]", cc->nodes[3].dsp_slot);
    snprintf(dsp_expr_buf[15], sizeof(dsp_expr_buf[15]), "dsp_prod_w[%d]", cc->nodes[4].dsp_slot);
    snprintf(dsp_expr_buf[16], sizeof(dsp_expr_buf[16]), "dsp_prod_w[%d]", cc->nodes[5].dsp_slot);
    snprintf(dsp_expr_buf[17], sizeof(dsp_expr_buf[17]), "dsp_prod_w[%d]", cc->nodes[6].dsp_slot);

    /* Cone A scalar cut slices (`a4_b0_eff` is unified directly with `b0_eff64_q`).
     * All DSP33 cuts bind directly to 66-bit `dsp_prod_w[slot]` so same-slot DSP cuts
     * across Cones A/B/C share physical lanes with ZERO data-steering muxes, while
     * zero-operand & regime clock-enables eliminate redundant lane thrashing! */
    if (ca->nodes[0].needs_reg)
        smap->a0 = add_scalar_req(ra, "a0_ug_u", base_ca + ca->nodes[0].stage,
                                  base_ca + max_consumer_stage(ca, 0), 66, "1'b1", dsp_expr_buf[0]);
    if (ca->nodes[1].needs_reg)
        smap->a1 = add_scalar_req(ra, "a1_b1_x", base_ca + ca->nodes[1].stage,
                                  base_ca + max_consumer_stage(ca, 1), 66, "1'b1", dsp_expr_buf[1]);
    if (ca->nodes[2].needs_reg)
        smap->a2 = add_scalar_req(ra, "a2_x_sq", base_ca + ca->nodes[2].stage,
                                  base_ca + max_consumer_stage(ca, 2), 66, "1'b1", dsp_expr_buf[2]);
    if (ca->nodes[3].needs_reg)
        smap->a3 = add_scalar_req(ra, "a3_b2_x", base_ca + ca->nodes[3].stage,
                                  base_ca + max_consumer_stage(ca, 3), 66, "1'b1", dsp_expr_buf[3]);
    if (ca->nodes[5].needs_reg)
        smap->a5 = add_scalar_req(ra, "a5_b2xsq_lo", base_ca + ca->nodes[5].stage,
                                  base_ca + max_consumer_stage(ca, 5), 66, "1'b1", dsp_expr_buf[4]);
    if (ca->nodes[6].needs_reg)
        smap->a6 = add_scalar_req(ra, "a6_b2xsq_hi", base_ca + ca->nodes[6].stage,
                                  base_ca + max_consumer_stage(ca, 6), 66, "1'b1", dsp_expr_buf[5]);
    if (ca->nodes[7].needs_reg) {
        int b = base_ca + ca->nodes[7].stage, r = base_ca + max_consumer_stage(ca, 7);
        smap->a7_lo = add_scalar_req(ra, "a7_g0_op_lo", b, r, 64, "op_mask_q",
                                     "$unsigned(n_a7_g0_op_c[63:0])");
        smap->a7_hi = add_scalar_req(ra, "a7_g0_op_hi", b, r, 34, "op_mask_q",
                                     "$unsigned(n_a7_g0_op_c[97:64])");
    }
    if (ca->nodes[8].needs_reg)
        smap->a8 = add_scalar_req(ra, "a8_g1_op", base_ca + ca->nodes[8].stage,
                                  base_ca + max_consumer_stage(ca, 8), 64, "op_mask_q",
                                  "$unsigned(n_a8_g1_op_c)");
    if (ca->nodes[9].needs_reg) {
        int b = base_ca + ca->nodes[9].stage, r = base_ca + max_consumer_stage(ca, 9);
        smap->a9_lo = add_scalar_req(ra, "a9_g0_au_lo", b, r, 64, "~op_mask_q",
                                     "$unsigned(n_a9_g0_au_c[63:0])");
        smap->a9_hi = add_scalar_req(ra, "a9_g0_au_hi", b, r, 31, "~op_mask_q",
                                     "$unsigned(n_a9_g0_au_c[94:64])");
        smap->a9_j1 = add_scalar_req(ra, "a9_j1_q32", b, r, 64, "~op_mask_q",
                                     "$unsigned(n_a9_j1_q32_c)");
    }

    /* Cone B scalar cut slices */
    if (cb->nodes[0].needs_reg)
        smap->b0 = add_scalar_req(ra, "b0_g1_ll", base_cb + cb->nodes[0].stage,
                                  base_cb + max_consumer_stage(cb, 0), 66, "1'b1", dsp_expr_buf[6]);
    if (cb->nodes[1].needs_reg)
        smap->b1 = add_scalar_req(ra, "b1_g1_hl", base_cb + cb->nodes[1].stage,
                                  base_cb + max_consumer_stage(cb, 1), 66, "1'b1", dsp_expr_buf[7]);
    if (cb->nodes[2].needs_reg)
        smap->b2 = add_scalar_req(ra, "b2_g1_hh", base_cb + cb->nodes[2].stage,
                                  base_cb + max_consumer_stage(cb, 2), 66, "1'b1", dsp_expr_buf[8]);
    if (cb->nodes[3].needs_reg)
        smap->b3 = add_scalar_req(ra, "b3_g0r2_0", base_cb + cb->nodes[3].stage,
                                  base_cb + max_consumer_stage(cb, 3), 66, "1'b1", dsp_expr_buf[9]);
    if (cb->nodes[4].needs_reg)
        smap->b4 = add_scalar_req(ra, "b4_g0r2_1", base_cb + cb->nodes[4].stage,
                                  base_cb + max_consumer_stage(cb, 4), 66, "1'b1", dsp_expr_buf[10]);
    if (cb->nodes[5].needs_reg)
        smap->b5 = add_scalar_req(ra, "b5_g0r2_2", base_cb + cb->nodes[5].stage,
                                  base_cb + max_consumer_stage(cb, 5), 66, "1'b1", dsp_expr_buf[11]);
    if (cb->nodes[6].needs_reg) {
        int b = base_cb + cb->nodes[6].stage, r = base_cb + max_consumer_stage(cb, 6);
        smap->b6_lo = add_scalar_req(ra, "b6_g1_sq_lo", b, r, 64, "1'b1",
                                     "$unsigned(n_b6_g1_sq_c[63:0])");
        smap->b6_hi = add_scalar_req(ra, "b6_g1_sq_hi", b, r, 64, "1'b1",
                                     "$unsigned(n_b6_g1_sq_c[127:64])");
    }
    if (cb->nodes[7].needs_reg) {
        int b = base_cb + cb->nodes[7].stage, r = base_cb + max_consumer_stage(cb, 7);
        smap->b7_lo = add_scalar_req(ra, "b7_g0_r2_lo", b, r, 64, "1'b1",
                                     "$unsigned(n_b7_g0_r2_c[63:0])");
        smap->b7_hi = add_scalar_req(ra, "b7_g0_r2_hi", b, r, 64, "1'b1",
                                     "$unsigned(n_b7_g0_r2_c[127:64])");
    }

    /* Cone D scalar cut slices (`99b` exact signed num + `66b` `{div_neg, div_den}`) */
    if (cd->nodes[0].needs_reg) {
        int b = base_cd + cd->nodes[0].stage, r = base_cd + max_consumer_stage(cd, 0);
        smap->d0_lo  = add_scalar_req(ra, "d0_div_num_lo", b, r, 64, "1'b1",
                                      "$unsigned(n_d0_div_num_c[63:0])");
        smap->d0_hi  = add_scalar_req(ra, "d0_div_num_hi", b, r, 35, "1'b1",
                                      "$unsigned(n_d0_div_num_c[98:64])");
        smap->d0_den = add_scalar_req(ra, "d0_div_den_neg", b, r, 66, "1'b1",
                                      "{n_d0_div_neg_c, n_d0_div_den_c}");
    }

    /* Cone C scalar cut slices (`ele_mul33_atom` already outputs 0 when `r2 == 0`) */
    if (cc->nodes[0].needs_reg)
        smap->c0 = add_scalar_req(ra, "c0_dc", base_cc + cc->nodes[0].stage,
                                  base_cc + max_consumer_stage(cc, 0), 33, "1'b1",
                                  "$unsigned(n_c0_dc_c)");
    if (cc->nodes[1].needs_reg)
        smap->c1 = add_scalar_req(ra, "c1_dsq", base_cc + cc->nodes[1].stage,
                                  base_cc + max_consumer_stage(cc, 1), 66, "1'b1", dsp_expr_buf[12]);
    if (cc->nodes[2].needs_reg)
        smap->c2 = add_scalar_req(ra, "c2_g1dc_lo", base_cc + cc->nodes[2].stage,
                                  base_cc + max_consumer_stage(cc, 2), 66, "1'b1", dsp_expr_buf[13]);
    if (cc->nodes[3].needs_reg)
        smap->c3 = add_scalar_req(ra, "c3_g1dc_hi", base_cc + cc->nodes[3].stage,
                                  base_cc + max_consumer_stage(cc, 3), 66, "1'b1", dsp_expr_buf[14]);
    if (cc->nodes[4].needs_reg)
        smap->c4 = add_scalar_req(ra, "c4_r2_dc", base_cc + cc->nodes[4].stage,
                                  base_cc + max_consumer_stage(cc, 4), 66, "1'b1", dsp_expr_buf[15]);
    if (cc->nodes[5].needs_reg)
        smap->c5 = add_scalar_req(ra, "c5_r2dsq_lo", base_cc + cc->nodes[5].stage,
                                  base_cc + max_consumer_stage(cc, 5), 66, "1'b1", dsp_expr_buf[16]);
    if (cc->nodes[6].needs_reg)
        smap->c6 = add_scalar_req(ra, "c6_r2dsq_hi", base_cc + cc->nodes[6].stage,
                                  base_cc + max_consumer_stage(cc, 6), 66, "1'b1", dsp_expr_buf[17]);
    if (cc->nodes[7].needs_reg) {
        int b = base_cc + cc->nodes[7].stage, r = base_cc + max_consumer_stage(cc, 7);
        smap->c7_lo = add_scalar_req(ra, "c7_g0_cand_lo", b, r, 64, "1'b1",
                                     "$unsigned(n_c7_g0_cand_c[63:0])");
        smap->c7_hi = add_scalar_req(ra, "c7_g0_cand_hi", b, r, 35, "1'b1",
                                     "$unsigned(n_c7_g0_cand_c[98:64])");
    }
    if (cc->nodes[8].needs_reg) {
        int b     = base_cc + cc->nodes[8].stage;
        int r_s9  = base_cc + cc->nodes[9].stage;
        int r_s10 = base_cc + cc->nodes[10].stage;
        smap->c8_lo   = add_scalar_req(ra, "c8_g0_post_lo", b, r_s10, 64, "1'b1",
                                       "$unsigned(n_c8_g0_post_c[63:0])");
        smap->c8_hi   = add_scalar_req(ra, "c8_g0_post_hi", b, r_s10, 35, "1'b1",
                                       "$unsigned(n_c8_g0_post_c[98:64])");
        smap->c8_g1   = add_scalar_req(ra, "c8_g1_post", b, r_s9, 64, "1'b1",
                                       "$unsigned(n_c8_g1_post_c)");
        smap->c8_xpdt = add_scalar_req(ra, "c8_xp_dt", b, r_s10, 64, "1'b1",
                                       "{$unsigned(n_c8_xp_c), n_c8_dt_res_c[31:0]}");
    }
    if (cc->nodes[9].needs_reg) {
        int b = base_cc + cc->nodes[9].stage, r = base_cc + cc->nodes[10].stage;
        smap->c9_j0  = add_scalar_req(ra, "c9_j0_next", b, r, 64, "1'b1",
                                      "n_c9_j0_next_c");
        smap->c9_j1  = add_scalar_req(ra, "c9_j1_next", b, r, 64, "1'b1",
                                      "n_c9_j1_next_c");
        smap->c9_sub = add_scalar_req(ra, "c9_sub_next", b, r, 64, "1'b1",
                                      "n_c9_sub_next_c");
    }

    /* Seed Chordal Interval-Graph Coloring (Linear Scan sorted by birth_state):
     * Synchronous non-strict reuse `last_read_state <= birth_state`. */
    int order[MAX_SCALAR_REQS];
    for (int i = 0; i < ra->num_reqs; ++i) {
        order[i] = i;
    }
    for (int i = 0; i < ra->num_reqs; ++i) {
        for (int j = i + 1; j < ra->num_reqs; ++j) {
            const ssa_scalar_live_t *ri = &ra->reqs[order[i]];
            const ssa_scalar_live_t *rj = &ra->reqs[order[j]];
            bool swap_ij = false;
            if (rj->birth_state < ri->birth_state) {
                swap_ij = true;
            } else if (rj->birth_state == ri->birth_state && rj->bit_width > ri->bit_width) {
                swap_ij = true;
            } else if (rj->birth_state == ri->birth_state && rj->bit_width == ri->bit_width &&
                       rj->last_read_state < ri->last_read_state) {
                swap_ij = true;
            }
            if (swap_ij) {
                int tmp  = order[i];
                order[i] = order[j];
                order[j] = tmp;
            }
        }
    }

    int lane_free_at[MAX_PHYS_LANES];
    memset(lane_free_at, 0, sizeof(lane_free_at));

    for (int k = 0; k < ra->num_reqs; ++k) {
        ssa_scalar_live_t *r = &ra->reqs[order[k]];
        int best_lane = -1;
        for (int ln = 0; ln < ra->num_lanes; ++ln) {
            if (lane_free_at[ln] > r->birth_state) {
                continue;
            }
            bool same_expr = false;
            for (int p = 0; p < k; ++p) {
                const ssa_scalar_live_t *pr = &ra->reqs[order[p]];
                if (pr->phys_lane == ln && pr->bit_width == r->bit_width &&
                    strcmp(pr->val_expr, r->val_expr) == 0) {
                    same_expr = true;
                    break;
                }
            }
            if (same_expr) {
                best_lane = ln;
                break;
            }
        }
        if (best_lane < 0) {
            for (int ln = 0; ln < ra->num_lanes; ++ln) {
                if (lane_free_at[ln] > r->birth_state || ra->lane_width[ln] < r->bit_width) {
                    continue;
                }
                const char *u0 = NULL;
                int uniq = 0;
                bool is_dsp0 = false;
                for (int p = 0; p < k; ++p) {
                    const ssa_scalar_live_t *pr = &ra->reqs[order[p]];
                    if (pr->phys_lane != ln) continue;
                    if (u0 == NULL) {
                        u0 = pr->val_expr;
                        uniq = 1;
                        is_dsp0 = (strncmp(u0, "dsp_prod_w[", 11) == 0);
                    } else if (strcmp(u0, pr->val_expr) != 0) {
                        uniq++;
                    }
                }
                if (!(uniq == 1 && is_dsp0)) {
                    best_lane = ln;
                    break;
                }
            }
        }
        if (best_lane < 0) {
            for (int ln = 0; ln < ra->num_lanes; ++ln) {
                if (lane_free_at[ln] <= r->birth_state && ra->lane_width[ln] >= r->bit_width) {
                    best_lane = ln;
                    break;
                }
            }
        }
        if (best_lane < 0) {
            for (int ln = 0; ln < ra->num_lanes; ++ln) {
                if (lane_free_at[ln] <= r->birth_state) {
                    best_lane = ln;
                    break;
                }
            }
        }
        if (best_lane < 0) {
            if (ra->num_lanes >= MAX_PHYS_LANES) {
                fprintf(stderr, "FATAL: physical lane pool overflow!\n");
                exit(1);
            }
            best_lane = ra->num_lanes++;
        }
        r->phys_lane            = best_lane;
        lane_free_at[best_lane] = r->last_read_state;
        if (r->bit_width > ra->lane_width[best_lane]) {
            ra->lane_width[best_lane] = r->bit_width;
        }
        ra->lane_writers[best_lane]++;
    }

    ra->shared_cut_bits = 0;
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        ra->shared_cut_bits += ra->lane_width[ln];
    }
}

/* ============================================================================
 * Pass 1B: `SEARU_SOLVE_INLINE` DSP Operand-Affinity & Cut-Mux Binding
 * ============================================================================ */

typedef struct {
    int cone_idx;
    int stage;
    int num_ops;
    int node_indices[MAX_DSPS];
} dsp_stage_group_t;

static inline void apply_dsp_group_permutations(dag_cone_t cones[NUM_CONES],
                                                const dsp_stage_group_t *groups,
                                                int num_groups,
                                                int dsp_count,
                                                const int *pair_p,
                                                const int *pair_q,
                                                int pairs_per_grp,
                                                const uint64_t canvas[4]) {
    for (int g = 0; g < num_groups; ++g) {
        int slots[MAX_DSPS];
        for (int d = 0; d < dsp_count; ++d) {
            slots[d] = d;
        }
        for (int t = 0; t < pairs_per_grp; ++t) {
            uint32_t v = (uint32_t)(g * pairs_per_grp + t);
            if (((canvas[v >> 6U] >> (v & 63U)) & UINT64_C(1)) != UINT64_C(0)) {
                int p    = pair_p[t];
                int q    = pair_q[t];
                int tmp  = slots[p];
                slots[p] = slots[q];
                slots[q] = tmp;
            }
        }
        dag_cone_t *cn = &cones[groups[g].cone_idx];
        for (int k = 0; k < groups[g].num_ops; ++k) {
            cn->nodes[groups[g].node_indices[k]].dsp_slot = slots[k];
        }
    }
}

static inline uint64_t eval_p1_dsp_cand(dag_cone_t cones[NUM_CONES],
                                        const dsp_stage_group_t *groups,
                                        int num_groups,
                                        int dsp_count,
                                        const int *pair_p,
                                        const int *pair_q,
                                        int pairs_per_grp,
                                        int base_ca, int base_cb, int base_cd, int base_cc,
                                        const dsp_bind_t binds[18],
                                        const uint64_t kan_pin[4],
                                        const uint64_t cand[4],
                                        uint64_t frust[4]) {
    apply_dsp_group_permutations(cones, groups, num_groups, dsp_count,
                                 pair_p, pair_q, pairs_per_grp, cand);

    reg_alloc_t  ra_tmp;
    scalar_map_t smap_tmp;
    int          lane_uniq[MAX_PHYS_LANES];
    memset(lane_uniq, 0, sizeof(lane_uniq));

    populate_and_color_cut_registers_greedy(cones, base_ca, base_cb, base_cd, base_cc,
                                            &ra_tmp, &smap_tmp);
    uint32_t uniq_ops = count_unique_dsp_operands(cones, dsp_count, binds);
    uint32_t mux_bits = count_lane_extra_mux_bits(&ra_tmp, lane_uniq);

    /* Frustration compass: mark stage groups whose DSP ops either have an operand
     * appearing on another DSP slot or feed a multi-writer shared cut lane. */
    const dag_cone_t *ca = &cones[CONE_A];
    const dag_cone_t *cb = &cones[CONE_B];
    const dag_cone_t *cc = &cones[CONE_C];
    for (int g = 0; g < num_groups; ++g) {
        bool grp_frust = false;
        int  c_idx     = groups[g].cone_idx;
        int  bind_cid  = (c_idx == CONE_A) ? 0 : ((c_idx == CONE_B) ? 1 : 2);
        for (int b = 0; b < 18 && !grp_frust; ++b) {
            if (binds[b].cone_id != bind_cid) {
                continue;
            }
            const dag_cone_t *ci = &cones[c_idx];
            if (ci->nodes[binds[b].node_idx].stage != groups[g].stage) {
                continue;
            }
            int sl_b = ci->nodes[binds[b].node_idx].dsp_slot;
            for (int j = 0; j < 18; ++j) {
                if (j == b) continue;
                const dag_cone_t *cj = (binds[j].cone_id == 0) ? ca : ((binds[j].cone_id == 1) ? cb : cc);
                int sl_j = cj->nodes[binds[j].node_idx].dsp_slot;
                if (sl_j != sl_b &&
                    (strcmp(binds[b].op_a_expr, binds[j].op_a_expr) == 0 ||
                     strcmp(binds[b].op_b_expr, binds[j].op_b_expr) == 0)) {
                    grp_frust = true;
                    break;
                }
            }
            if (!grp_frust && ci->nodes[binds[b].node_idx].needs_reg) {
                for (int r = 0; r < ra_tmp.num_reqs; ++r) {
                    if (ra_tmp.reqs[r].birth_state == binds[b].global_state &&
                        lane_uniq[ra_tmp.reqs[r].phys_lane] > 1) {
                        grp_frust = true;
                        break;
                    }
                }
            }
        }
        if (grp_frust) {
            for (int t = 0; t < pairs_per_grp; ++t) {
                uint32_t v = (uint32_t)(g * pairs_per_grp + t);
                uint32_t w = v >> 6U;
                frust[w] |= (UINT64_C(1) << (v & 63U)) & ~kan_pin[w];
            }
        }
    }

    return ((uint64_t)ra_tmp.shared_cut_bits * UINT64_C(16)) +
           ((uint64_t)uniq_ops * UINT64_C(33)) +
           (uint64_t)mux_bits;
}

static void optimize_dsp_slots_searu(dag_cone_t cones[NUM_CONES],
                                     int dsp_count, int max_depth,
                                     int base_ca, int base_cb, int base_cd, int base_cc,
                                     searu_hls_stats_t *stats) {
    dsp_expr_scratch_t sc;
    dsp_bind_t         binds[18];
    build_dsp_binds(cones, base_ca, base_cb, base_cc, &sc, binds);

    reg_alloc_t  ra_init;
    scalar_map_t smap_init;
    populate_and_color_cut_registers_greedy(cones, base_ca, base_cb, base_cd, base_cc,
                                            &ra_init, &smap_init);
    stats->dsp_uniq_ops_init = count_unique_dsp_operands(cones, dsp_count, binds);
    uint32_t init_mux        = count_lane_extra_mux_bits(&ra_init, NULL);
    stats->p1_energy_init    = ((uint64_t)ra_init.shared_cut_bits * UINT64_C(16)) +
                               ((uint64_t)stats->dsp_uniq_ops_init * UINT64_C(33)) +
                               (uint64_t)init_mux;

    if (dsp_count <= 1) {
        stats->dsp_uniq_ops_best = stats->dsp_uniq_ops_init;
        stats->p1_energy_best    = stats->p1_energy_init;
        return;
    }

    int pair_p[32];
    int pair_q[32];
    int pairs_per_grp = 0;
    for (int p = 0; p < dsp_count; ++p) {
        for (int q = p + 1; q < dsp_count; ++q) {
            pair_p[pairs_per_grp] = p;
            pair_q[pairs_per_grp] = q;
            ++pairs_per_grp;
        }
    }
    for (int p = 0; p + 1 < dsp_count; ++p) {
        pair_p[pairs_per_grp] = p;
        pair_q[pairs_per_grp] = p + 1;
        ++pairs_per_grp;
    }

    dsp_stage_group_t groups[24];
    int               num_groups = 0;
    const int         cone_order[3] = {CONE_A, CONE_B, CONE_C};
    for (int ci = 0; ci < 3; ++ci) {
        int               c_idx = cone_order[ci];
        const dag_cone_t *cn    = &cones[c_idx];
        for (int s = 0; s < cn->num_stages; ++s) {
            if ((num_groups + 1) * pairs_per_grp > (int)SEARU_CANVAS_BITS || num_groups >= 24) {
                break;
            }
            dsp_stage_group_t grp;
            memset(&grp, 0, sizeof(grp));
            grp.cone_idx = c_idx;
            grp.stage    = s;
            for (int n = 0; n < cn->num_nodes; ++n) {
                if (cn->nodes[n].phase == PHASE_DSP33 && cn->nodes[n].stage == s) {
                    grp.node_indices[grp.num_ops++] = n;
                }
            }
            if (grp.num_ops > 0) {
                groups[num_groups++] = grp;
            }
        }
    }

    uint64_t init_canvas[4] = {UINT64_C(0), UINT64_C(0), UINT64_C(0), UINT64_C(0)};
    uint64_t kan_pin[4]     = {UINT64_C(0), UINT64_C(0), UINT64_C(0), UINT64_C(0)};

    /* Kan-pin `b0_g1_ll -> slot 0` and `b4_g0r2_1 -> slot 1` so exact cross-limb
     * verification anchors stay locked while remaining DSP slots permute freely. */
    for (int g = 0; g < num_groups; ++g) {
        if (groups[g].cone_idx != CONE_B) {
            continue;
        }
        for (int k = 0; k < groups[g].num_ops; ++k) {
            int nd_idx   = groups[g].node_indices[k];
            int pin_slot = -1;
            if (nd_idx == 0 && k == 0) {
                pin_slot = 0;
            } else if (nd_idx == 4 && k == 1 && dsp_count >= 2) {
                pin_slot = 1;
            }
            if (pin_slot >= 0) {
                for (int t = 0; t < pairs_per_grp; ++t) {
                    if (pair_p[t] == pin_slot || pair_q[t] == pin_slot) {
                        uint32_t v = (uint32_t)(g * pairs_per_grp + t);
                        kan_pin[v >> 6U] |= UINT64_C(1) << (v & 63U);
                    }
                }
            }
        }
    }

    uint32_t     nvars = (uint32_t)(num_groups * pairs_per_grp);
    uint64_t     seed  = UINT64_C(0x5EA20D59) ^ ((uint64_t)dsp_count << 16U) ^ (uint64_t)max_depth;
    searu_result res;
    SEARU_SOLVE_INLINE(&res, nvars, init_canvas, kan_pin, seed, 16384U, cand, frust, energy, {
        energy = eval_p1_dsp_cand(cones, groups, num_groups, dsp_count,
                                  pair_p, pair_q, pairs_per_grp,
                                  base_ca, base_cb, base_cd, base_cc,
                                  binds, kan_pin, cand, frust);
    });

    apply_dsp_group_permutations(cones, groups, num_groups, dsp_count,
                                 pair_p, pair_q, pairs_per_grp, res.canvas);
    stats->dsp_uniq_ops_best = count_unique_dsp_operands(cones, dsp_count, binds);
    stats->p1_iters          = res.iterations_run;
    stats->p1_epiphanies     = res.epiphanies;
    stats->p1_swans          = res.black_swans;
    stats->p1_energy_best    = res.best_energy;
}

/* ============================================================================
 * Pass 2B: `SEARU_SOLVE_INLINE` Heterogeneous-Width + Mux-Affinity Allocation
 * ============================================================================ */

static inline uint32_t extract_req_lane_code(const uint64_t canvas[4],
                                             int req_idx, int bits_per_req) {
    uint32_t code = 0U;
    for (int b = 0; b < bits_per_req; ++b) {
        uint32_t v   = (uint32_t)(req_idx * bits_per_req + b);
        uint32_t bit = (uint32_t)((canvas[v >> 6U] >> (v & 63U)) & UINT64_C(1));
        code |= bit << (uint32_t)b;
    }
    return code;
}

static inline void set_req_frust_bits(uint64_t frust[4], int req_idx, int bits_per_req) {
    for (int b = 0; b < bits_per_req; ++b) {
        uint32_t v = (uint32_t)(req_idx * bits_per_req + b);
        frust[v >> 6U] |= UINT64_C(1) << (v & 63U);
    }
}

static inline uint64_t eval_p2_reg_cand(const reg_alloc_t *ra,
                                        int pool_lanes,
                                        int bits_per_req,
                                        uint64_t overlap_penalty_floor,
                                        const uint64_t cand[4],
                                        uint64_t frust[4]) {
    int  cand_lane[MAX_SCALAR_REQS];
    bool req_ovf[MAX_SCALAR_REQS];
    int  lane_max_w[MAX_PHYS_LANES];
    int  lane_min_w[MAX_PHYS_LANES];
    int  lane_num_u[MAX_PHYS_LANES];

    memset(req_ovf, 0, sizeof(req_ovf));
    for (int ln = 0; ln < pool_lanes; ++ln) {
        lane_max_w[ln] = 0;
        lane_min_w[ln] = 1024;
        lane_num_u[ln] = 0;
    }

    for (int r = 0; r < ra->num_reqs; ++r) {
        uint32_t code = extract_req_lane_code(cand, r, bits_per_req);
        int      ln   = (int)(code % (uint32_t)pool_lanes);
        cand_lane[r]  = ln;
        int w         = ra->reqs[r].bit_width;
        if (w > lane_max_w[ln]) lane_max_w[ln] = w;
        if (w < lane_min_w[ln]) lane_min_w[ln] = w;
    }

    uint32_t overlaps = 0U;
    for (int i = 0; i < ra->num_reqs; ++i) {
        for (int j = i + 1; j < ra->num_reqs; ++j) {
            if (cand_lane[i] == cand_lane[j]) {
                bool disjoint = (ra->reqs[i].last_read_state <= ra->reqs[j].birth_state) ||
                                (ra->reqs[j].last_read_state <= ra->reqs[i].birth_state);
                if (!disjoint) {
                    ++overlaps;
                    req_ovf[i] = true;
                    req_ovf[j] = true;
                }
            }
        }
    }

    uint64_t total_lane_bits = UINT64_C(0);
    uint64_t extra_mux_bits  = UINT64_C(0);
    for (int ln = 0; ln < pool_lanes; ++ln) {
        if (lane_max_w[ln] == 0) {
            continue;
        }
        total_lane_bits += (uint64_t)lane_max_w[ln];
        int         u_w[MAX_SCALAR_REQS];
        const char *u_expr[MAX_SCALAR_REQS];
        int         num_u = 0;
        for (int r = 0; r < ra->num_reqs; ++r) {
            if (cand_lane[r] != ln) continue;
            bool seen = false;
            for (int u = 0; u < num_u; ++u) {
                if (u_w[u] == ra->reqs[r].bit_width &&
                    strcmp(u_expr[u], ra->reqs[r].val_expr) == 0) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                u_w[num_u]    = ra->reqs[r].bit_width;
                u_expr[num_u] = ra->reqs[r].val_expr;
                ++num_u;
            }
        }
        lane_num_u[ln] = num_u;
        if (num_u > 1) {
            extra_mux_bits += (uint64_t)(num_u - 1) * (uint64_t)lane_max_w[ln];
        }
    }

    if (overlaps > 0U) {
        for (int r = 0; r < ra->num_reqs; ++r) {
            if (req_ovf[r]) {
                set_req_frust_bits(frust, r, bits_per_req);
            }
        }
        return overlap_penalty_floor +
               ((uint64_t)overlaps * UINT64_C(4096)) +
               (total_lane_bits * UINT64_C(16)) +
               extra_mux_bits;
    }

    for (int r = 0; r < ra->num_reqs; ++r) {
        int ln = cand_lane[r];
        if (lane_min_w[ln] < lane_max_w[ln] || lane_num_u[ln] > 1) {
            set_req_frust_bits(frust, r, bits_per_req);
        }
    }
    return (total_lane_bits * UINT64_C(16)) + extra_mux_bits;
}

static void allocate_shared_cut_registers(const dag_cone_t cones[NUM_CONES],
                                          int dsp_count, int max_depth,
                                          int base_ca, int base_cb, int base_cd, int base_cc,
                                          reg_alloc_t *ra, scalar_map_t *smap,
                                          searu_hls_stats_t *stats) {
    populate_and_color_cut_registers_greedy(cones, base_ca, base_cb, base_cd, base_cc, ra, smap);

    stats->cut_bits_init      = ra->shared_cut_bits;
    stats->lane_mux_bits_init = count_lane_extra_mux_bits(ra, NULL);
    stats->p2_energy_init     = ((uint64_t)ra->shared_cut_bits * UINT64_C(16)) +
                                (uint64_t)stats->lane_mux_bits_init;
    stats->p2_energy_best     = stats->p2_energy_init;

    const int pool_lanes = ra->num_lanes;
    if (ra->num_reqs > 0 && pool_lanes >= 2) {
        int bits_per_req = 1;
        while ((1 << bits_per_req) < pool_lanes) {
            ++bits_per_req;
        }
        if (ra->num_reqs * bits_per_req <= (int)SEARU_CANVAS_BITS) {
            uint64_t init_canvas[4] = {UINT64_C(0), UINT64_C(0), UINT64_C(0), UINT64_C(0)};
            for (int r = 0; r < ra->num_reqs; ++r) {
                uint32_t code = (uint32_t)ra->reqs[r].phys_lane;
                for (int b = 0; b < bits_per_req; ++b) {
                    if (((code >> (uint32_t)b) & 1U) != 0U) {
                        uint32_t v = (uint32_t)(r * bits_per_req + b);
                        init_canvas[v >> 6U] |= UINT64_C(1) << (v & 63U);
                    }
                }
            }

            uint64_t     overlap_floor = stats->p2_energy_init + UINT64_C(65536);
            uint32_t     nvars         = (uint32_t)(ra->num_reqs * bits_per_req);
            uint64_t     seed          = UINT64_C(0x5EA20C07) ^
                                         ((uint64_t)dsp_count << 16U) ^
                                         (uint64_t)max_depth;
            searu_result res;
            SEARU_SOLVE_INLINE(&res, nvars, init_canvas, NULL, seed, 16384U,
                               cand, frust, energy, {
                energy = eval_p2_reg_cand(ra, pool_lanes, bits_per_req,
                                          overlap_floor, cand, frust);
            });

            /* Decode winning canvas and compact active physical lanes */
            int raw_lane[MAX_SCALAR_REQS];
            int lane_remap[MAX_PHYS_LANES];
            memset(lane_remap, -1, sizeof(lane_remap));
            int compact_lanes = 0;

            for (int r = 0; r < ra->num_reqs; ++r) {
                uint32_t code = extract_req_lane_code(res.canvas, r, bits_per_req);
                raw_lane[r]   = (int)(code % (uint32_t)pool_lanes);
            }
            for (int ln = 0; ln < pool_lanes; ++ln) {
                bool used = false;
                for (int r = 0; r < ra->num_reqs; ++r) {
                    if (raw_lane[r] == ln) {
                        used = true;
                        break;
                    }
                }
                if (used) {
                    lane_remap[ln] = compact_lanes++;
                }
            }

            ra->num_lanes = compact_lanes;
            memset(ra->lane_width, 0, sizeof(ra->lane_width));
            memset(ra->lane_writers, 0, sizeof(ra->lane_writers));
            for (int r = 0; r < ra->num_reqs; ++r) {
                int cl                  = lane_remap[raw_lane[r]];
                ra->reqs[r].phys_lane   = cl;
                ra->lane_writers[cl]++;
                if (ra->reqs[r].bit_width > ra->lane_width[cl]) {
                    ra->lane_width[cl] = ra->reqs[r].bit_width;
                }
            }

            stats->p2_iters       = res.iterations_run;
            stats->p2_epiphanies  = res.epiphanies;
            stats->p2_swans       = res.black_swans;
            stats->p2_energy_best = res.best_energy;
        }
    }

    /* Post-allocation O(N^2) formal disjointness proof across all assigned intervals */
    for (int i = 0; i < ra->num_reqs; ++i) {
        for (int j = i + 1; j < ra->num_reqs; ++j) {
            const ssa_scalar_live_t *a = &ra->reqs[i];
            const ssa_scalar_live_t *b = &ra->reqs[j];
            if (a->phys_lane == b->phys_lane) {
                bool disjoint = (a->last_read_state <= b->birth_state) ||
                                (b->last_read_state <= a->birth_state);
                if (!disjoint) {
                    fprintf(stderr, "FATAL: live-range overlap on sh_lane%d: %s [%d,%d] vs %s [%d,%d]\n",
                            a->phys_lane, a->tag, a->birth_state, a->last_read_state,
                            b->tag, b->birth_state, b->last_read_state);
                    exit(1);
                }
            }
        }
    }

    ra->shared_cut_bits = 0;
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        ra->shared_cut_bits += ra->lane_width[ln];
    }
    stats->cut_bits_best      = ra->shared_cut_bits;
    stats->lane_mux_bits_best = count_lane_extra_mux_bits(ra, NULL);
}

static void emit_lane_rhs(FILE *fp, int lane_w, int val_w, const char *val_expr) {
    if (lane_w == val_w) {
        fprintf(fp, "%s", val_expr);
    } else {
        fprintf(fp, "{%d'd0, %s}", lane_w - val_w, val_expr);
    }
}

static void emit_sv(FILE *fp, int dsp_count, int max_depth, int k_rad, uint32_t rad_epochs,
                    const dag_cone_t cones[NUM_CONES],
                    const reg_alloc_t *ra, const scalar_map_t *smap,
                    const searu_hls_stats_t *hls) {
    const dag_cone_t *ca = &cones[CONE_A];
    const dag_cone_t *cb = &cones[CONE_B];
    const dag_cone_t *cd = &cones[CONE_D];
    const dag_cone_t *cc = &cones[CONE_C];

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

    const int quad_cycles      = ca->num_stages + cb->num_stages + rad_iters +
                                 cd->num_stages + rad_iters + cc->num_stages;
    const int lin_cycles       = ca->num_stages + cd->num_stages + rad_iters + cc->num_stages;
    const int idle_cycles      = ca->num_stages + (cc->num_stages - cc->nodes[9].stage);
    const int virtual_cut_bits = ca->cut_reg_bits + cb->cut_reg_bits +
                                 cd->cut_reg_bits + cc->cut_reg_bits;
    const int shared_cut_bits  = ra->shared_cut_bits;
    const int base_dff_bits    = st_w + iter_w + 1 + 64 + 98 + 64 + 1 + 1 + 64 + 64 + 128;
    const int core_dff_bits    = base_dff_bits + shared_cut_bits;

    fprintf(fp, "// SPDX-License-Identifier: MIT\n");
    fprintf(fp, "// AUTO-GENERATED BY `tools/ele_bootstrap.c` VIA `SEARU x FLOW v%s` (`SEARU_SOLVE_INLINE` + `flow_step_atlas`)\n",
            FLOW_VERSION_STRING);
    fprintf(fp, "// DO NOT HAND-EDIT — Re-synthesize via `./build/ele_bootstrap`.\n");
    fprintf(fp, "//\n");
    fprintf(fp, "// Symbiotic SEARU x FLOW 2-Jet HLS Schedule, DSP Operand Affinity & Register Sharing:\n");
    fprintf(fp, "//   * dsp_count (`ele_mul33_atom` instances) : %d\n", dsp_count);
    fprintf(fp, "//   * max_depth (target gate lvls / cycle)   : %d\n", max_depth);
    fprintf(fp, "//   * k_rad (`ele_rad2_step_atom` chain)     : %d (%d cyc/loop, %u FLOW epochs)\n",
            k_rad, rad_iters, rad_epochs);
    fprintf(fp, "//   * Cone A / B / D / C pipeline stages     : %d / %d / %d / %d (%d FSM states)\n",
            ca->num_stages, cb->num_stages, cd->num_stages, cc->num_stages, num_st);
    fprintf(fp, "//   * FLOW lattice sweeps (A/B/D/C)          : %d / %d / %d / %d (100%% CONVERGED)\n",
            ca->sweeps_taken, cb->sweeps_taken, cd->sweeps_taken, cc->sweeps_taken);
    fprintf(fp, "//   * SEARU Pass-1 DSP operand fan-in        : %u -> %u unique 33b ops (%u iters, %u epiphanies)\n",
            hls->dsp_uniq_ops_init, hls->dsp_uniq_ops_best, hls->p1_iters, hls->p1_epiphanies);
    fprintf(fp, "//   * SEARU Pass-2 cut DFFs & lane mux bits  : %d -> %d cut bits (%d lanes), %u -> %u mux bits (%u iters, %u epiphanies)\n",
            virtual_cut_bits, shared_cut_bits, ra->num_lanes,
            hls->lane_mux_bits_init, hls->lane_mux_bits_best, hls->p2_iters, hls->p2_epiphanies);
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
    fprintf(fp, "    /* verilator lint_off UNUSEDPARAM */\n");
    fprintf(fp, "    localparam int PROFILE_DSP_COUNT = %d;\n", dsp_count);
    fprintf(fp, "    localparam int PROFILE_MAX_DEPTH = %d;\n", max_depth);
    fprintf(fp, "    localparam int VIRTUAL_CUT_BITS  = %d;\n", virtual_cut_bits);
    fprintf(fp, "    localparam int SHARED_CUT_BITS   = %d;\n", shared_cut_bits);
    fprintf(fp, "    localparam int CORE_DFF_BITS     = %d;\n", core_dff_bits);
    fprintf(fp, "    /* verilator lint_on UNUSEDPARAM */\n\n");

    /* O(log W) carry-free power-of-two symmetric range trees & helpers matching `flow.h` */
    fprintf(fp, "    function automatic logic signed [31:0] clamp_sym64_to_32(input logic [63:0] raw);\n");
    fprintf(fp, "        logic ovf_pos, ovf_neg;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            ovf_pos = (~raw[63]) & (|raw[62:31]);\n");
    fprintf(fp, "            ovf_neg = raw[63] & ((~(&raw[62:31])) | (raw[30:0] == 31'd0));\n");
    fprintf(fp, "            if (ovf_pos)      clamp_sym64_to_32 = 32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else if (ovf_neg) clamp_sym64_to_32 = -32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else              clamp_sym64_to_32 = $signed(raw[31:0]);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [31:0] clamp_sym34_to_32(input logic signed [33:0] v);\n");
    fprintf(fp, "        logic [33:0] uv;\n");
    fprintf(fp, "        logic        ovf_pos, ovf_neg;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            uv      = $unsigned(v);\n");
    fprintf(fp, "            ovf_pos = (~uv[33]) & (|uv[32:31]);\n");
    fprintf(fp, "            ovf_neg = uv[33] & ((~(&uv[32:31])) | (uv[30:0] == 31'd0));\n");
    fprintf(fp, "            if (ovf_pos)      clamp_sym34_to_32 = 32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else if (ovf_neg) clamp_sym34_to_32 = -32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else              clamp_sym34_to_32 = $signed(uv[31:0]);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [31:0] clamp_sym32(input logic [31:0] raw);\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            clamp_sym32 = (raw == 32'h8000_0000) ? -32'sh7FFF_FFFF : $signed(raw);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [63:0] clamp_j0_sym64(input logic signed [63:0] v);\n");
    fprintf(fp, "        logic [63:0] uv;\n");
    fprintf(fp, "        logic        ovf_pos, ovf_neg;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            uv      = $unsigned(v);\n");
    fprintf(fp, "            ovf_pos = (~uv[63]) & ((|uv[62:61]) | (uv[60] & (|uv[59:0])));\n");
    fprintf(fp, "            ovf_neg = uv[63] & (~(&uv[62:60]));\n");
    fprintf(fp, "            if (ovf_pos)      clamp_j0_sym64 = 64'sh1000_0000_0000_0000;\n");
    fprintf(fp, "            else if (ovf_neg) clamp_j0_sym64 = -64'sh1000_0000_0000_0000;\n");
    fprintf(fp, "            else              clamp_j0_sym64 = v;\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic signed [63:0] clamp_j1_sym64(input logic signed [63:0] v);\n");
    fprintf(fp, "        logic [63:0] uv;\n");
    fprintf(fp, "        logic        ovf_pos, ovf_neg;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            uv      = $unsigned(v);\n");
    fprintf(fp, "            ovf_pos = (~uv[63]) & ((|uv[62:46]) | (uv[45] & (|uv[44:0])));\n");
    fprintf(fp, "            ovf_neg = uv[63] & (~(&uv[62:45]));\n");
    fprintf(fp, "            if (ovf_pos)      clamp_j1_sym64 = 64'sh0000_2000_0000_0000;\n");
    fprintf(fp, "            else if (ovf_neg) clamp_j1_sym64 = -64'sh0000_2000_0000_0000;\n");
    fprintf(fp, "            else              clamp_j1_sym64 = v;\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    fprintf(fp, "    function automatic logic [127:0] abs128(input logic signed [127:0] v);\n");
    fprintf(fp, "        logic [98:0] mag99;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            if (&{1'b0, v[126:99]}) begin end\n");
    fprintf(fp, "            mag99  = v[127] ? (99'd0 - $unsigned(v[98:0])) : $unsigned(v[98:0]);\n");
    fprintf(fp, "            abs128 = {29'd0, mag99};\n");
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

    /* Cross-cone architectural registers (`b0_eff64_q` is exact 64b; `g0_s98_q` is exact 98b Q49;
     * `d_mag` is folded into `sr_acc_q`, and `op_mask`/`r2` are combinational invariants: -97 DFFs!) */
    fprintf(fp, "    logic                fast_path_q;\n");
    fprintf(fp, "    logic signed [63:0]  b0_eff64_q;\n");
    fprintf(fp, "    logic signed [97:0]  g0_s98_q;\n");
    fprintf(fp, "    wire  signed [127:0] g0_q = $signed({{30{g0_s98_q[97]}}, g0_s98_q});\n");
    fprintf(fp, "    logic signed [63:0]  g1_q;\n");
    fprintf(fp, "    logic                div_neg_q;\n");
    fprintf(fp, "    logic                pre_sat_q;\n");
    fprintf(fp, "    logic [63:0]         sr_rem_q;\n");
    fprintf(fp, "    logic [63:0]         sr_acc_q;\n");
    fprintf(fp, "    logic [127:0]        sr_aux_q;\n\n");

    /* Pass-2 Shared Physical Pipeline-Cut Register Pool (`sh_lane*_q`) */
    fprintf(fp, "    // --- Pass-2 Live-Range Shared Physical Pipeline-Cut Lanes (%d bits vs %d virtual) ---\n",
            shared_cut_bits, virtual_cut_bits);
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, "    logic [%d:0]         sh_lane%d_q, sh_lane%d_d_c;\n",
                ra->lane_width[ln] - 1, ln, ln);
        fprintf(fp, "    logic                sh_lane%d_we_c;\n", ln);
    }
    fprintf(fp, "\n");

    fprintf(fp, "    logic signed [31:0]  x_in_clamped_c;\n");
    fprintf(fp, "    logic signed [31:0]  wr_clamped32_c;\n");
    fprintf(fp, "    logic signed [31:0]  ex_c, en_c, dt_c;\n");
    fprintf(fp, "    assign x_in_clamped_c = clamp_sym64_to_32(state_w_q[3]);\n");
    fprintf(fp, "    assign wr_clamped32_c = clamp_sym64_to_32(wr_data_i);\n");
    fprintf(fp, "    assign ex_c           = clamp_sym32(wr_data_i[63:32]);\n");
    fprintf(fp, "    assign en_c           = clamp_sym32(wr_data_i[31:0]);\n");
    fprintf(fp, "    assign dt_c           = ex_c;\n\n");

    /* Regime DSP isolation predicates + reclaimed combinational invariants (`-97` DFFs) */
    fprintf(fp, "    wire  signed [31:0]  j2_in_c      = clamp_sym64_to_32(state_w_q[2]);\n");
    fprintf(fp, "    wire                 b1_nz_c      = (b1_q != 32'sd0);\n");
    fprintf(fp, "    wire                 b2_nz_c      = (b2_q != 32'sd0);\n");
    fprintf(fp, "    wire                 op_mask_q    = b1_nz_c | b2_nz_c;\n");
    fprintf(fp, "    wire  signed [31:0]  r2_q         = op_mask_q ? b2_q : j2_in_c;\n");
    fprintf(fp, "    wire                 r2_nz_c      = (r2_q != 32'sd0);\n");
    fprintf(fp, "    wire         [63:0]  d_mag_w      = sr_acc_q;\n\n");

    /* Combinational SSA node wires & zero-gate virtual read aliases onto `sh_lane*_q` */
    fprintf(fp, "    // --- SSA Combinational Wires & Zero-Gate Shared-Lane Read Aliases ---\n");
    if (!ca->nodes[0].needs_reg) fprintf(fp, "    logic signed [63:0]  n_a0_ug_u_c;\n");
    if (!ca->nodes[1].needs_reg) fprintf(fp, "    logic signed [63:0]  n_a1_b1_x_c;\n");
    if (!ca->nodes[2].needs_reg) fprintf(fp, "    logic [63:0]         n_a2_x_sq_c;\n");
    if (!ca->nodes[3].needs_reg) fprintf(fp, "    logic signed [63:0]  n_a3_b2_x_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_a4_b0_eff64_c;\n");
    if (!ca->nodes[5].needs_reg) fprintf(fp, "    logic signed [65:0]  n_a5_b2xsq_lo_c;\n");
    if (!ca->nodes[6].needs_reg) fprintf(fp, "    logic signed [65:0]  n_a6_b2xsq_hi_c;\n");
    fprintf(fp, "    logic signed [127:0] n_a7_g0_op_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_a8_g1_op_c;\n");
    fprintf(fp, "    logic signed [127:0] n_a9_g0_au_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_a9_j1_q32_c;\n");
    fprintf(fp, "    logic                n_a10_op_mask_c, n_a10_active_c, n_a10_has_r2_c;\n");
    fprintf(fp, "    logic signed [127:0] n_a10_g0_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_a10_g1_c;\n");

    if (smap->a0 >= 0) fprintf(fp, "    wire  signed [63:0]  n_a0_ug_u_q     = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->a0].phys_lane);
    if (smap->a1 >= 0) fprintf(fp, "    wire  signed [63:0]  n_a1_b1_x_q     = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->a1].phys_lane);
    if (smap->a2 >= 0) fprintf(fp, "    wire         [63:0]  n_a2_x_sq_q     = sh_lane%d_q[63:0];\n", ra->reqs[smap->a2].phys_lane);
    if (smap->a3 >= 0) fprintf(fp, "    wire  signed [63:0]  n_a3_b2_x_q     = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->a3].phys_lane);
    if (smap->a5 >= 0) fprintf(fp, "    wire  signed [65:0]  n_a5_b2xsq_lo_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->a5].phys_lane);
    if (smap->a6 >= 0) fprintf(fp, "    wire  signed [65:0]  n_a6_b2xsq_hi_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->a6].phys_lane);
    if (smap->a7_lo >= 0)
        fprintf(fp, "    wire  signed [127:0] n_a7_g0_op_q    = $signed({{30{sh_lane%d_q[33]}}, sh_lane%d_q[33:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->a7_hi].phys_lane, ra->reqs[smap->a7_hi].phys_lane, ra->reqs[smap->a7_lo].phys_lane);
    if (smap->a8 >= 0) fprintf(fp, "    wire  signed [63:0]  n_a8_g1_op_q    = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->a8].phys_lane);
    if (smap->a9_lo >= 0) {
        fprintf(fp, "    wire  signed [127:0] n_a9_g0_au_q    = $signed({{33{sh_lane%d_q[30]}}, sh_lane%d_q[30:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->a9_hi].phys_lane, ra->reqs[smap->a9_hi].phys_lane, ra->reqs[smap->a9_lo].phys_lane);
        fprintf(fp, "    wire  signed [63:0]  n_a9_j1_q32_q   = $signed(sh_lane%d_q[63:0]);\n",
                ra->reqs[smap->a9_j1].phys_lane);
    }

    if (!cb->nodes[0].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b0_g1_ll_c;\n");
    if (!cb->nodes[1].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b1_g1_hl_c;\n");
    if (!cb->nodes[2].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b2_g1_hh_c;\n");
    if (!cb->nodes[3].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b3_g0r2_0_c;\n");
    if (!cb->nodes[4].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b4_g0r2_1_c;\n");
    if (!cb->nodes[5].needs_reg) fprintf(fp, "    logic signed [65:0]  n_b5_g0r2_2_c;\n");
    fprintf(fp, "    logic signed [127:0] n_b6_g1_sq_c, n_b7_g0_r2_c, n_b8_disc_c;\n");
    fprintf(fp, "    logic                n_b8_pos_nz_c;\n");
    if (smap->b0 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b0_g1_ll_q  = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b0].phys_lane);
    if (smap->b1 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b1_g1_hl_q  = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b1].phys_lane);
    if (smap->b2 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b2_g1_hh_q  = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b2].phys_lane);
    if (smap->b3 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b3_g0r2_0_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b3].phys_lane);
    if (smap->b4 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b4_g0r2_1_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b4].phys_lane);
    if (smap->b5 >= 0) fprintf(fp, "    wire  signed [65:0]  n_b5_g0r2_2_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->b5].phys_lane);
    if (smap->b6_lo >= 0)
        fprintf(fp, "    wire  signed [127:0] n_b6_g1_sq_q  = $signed({sh_lane%d_q[63:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->b6_hi].phys_lane, ra->reqs[smap->b6_lo].phys_lane);
    if (smap->b7_lo >= 0)
        fprintf(fp, "    wire  signed [127:0] n_b7_g0_r2_q  = $signed({sh_lane%d_q[63:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->b7_hi].phys_lane, ra->reqs[smap->b7_lo].phys_lane);

    fprintf(fp, "    logic signed [127:0] n_d0_div_num_c;\n");
    fprintf(fp, "    logic        [64:0]  n_d0_div_den_c;\n");
    fprintf(fp, "    logic                n_d0_div_neg_c;\n");
    fprintf(fp, "    logic [63:0]         n_d1_safe_den_c, n_d1_init_rem_c, n_d1_init_quot_c;\n");
    fprintf(fp, "    logic                n_d1_pre_sat_c;\n");
    if (smap->d0_lo >= 0) {
        fprintf(fp, "    wire  signed [127:0] n_d0_div_num_q = $signed({{29{sh_lane%d_q[34]}}, sh_lane%d_q[34:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->d0_hi].phys_lane, ra->reqs[smap->d0_hi].phys_lane, ra->reqs[smap->d0_lo].phys_lane);
        fprintf(fp, "    wire         [64:0]  n_d0_div_den_q = sh_lane%d_q[64:0];\n",
                ra->reqs[smap->d0_den].phys_lane);
        fprintf(fp, "    wire                 n_d0_div_neg_q = sh_lane%d_q[65];\n",
                ra->reqs[smap->d0_den].phys_lane);
    }

    fprintf(fp, "    logic signed [32:0]  n_c0_dc_c;\n");
    if (!cc->nodes[1].needs_reg) fprintf(fp, "    logic signed [63:0]  n_c1_dsq_c;\n");
    if (!cc->nodes[4].needs_reg) fprintf(fp, "    logic signed [63:0]  n_c4_r2_dc_c;\n");
    if (!cc->nodes[2].needs_reg) fprintf(fp, "    logic signed [65:0]  n_c2_g1dc_lo_c;\n");
    if (!cc->nodes[3].needs_reg) fprintf(fp, "    logic signed [65:0]  n_c3_g1dc_hi_c;\n");
    if (!cc->nodes[5].needs_reg) fprintf(fp, "    logic signed [65:0]  n_c5_r2dsq_lo_c;\n");
    if (!cc->nodes[6].needs_reg) fprintf(fp, "    logic signed [65:0]  n_c6_r2dsq_hi_c;\n");
    fprintf(fp, "    logic signed [127:0] n_c7_g0_cand_c, n_c8_g0_post_c;\n");
    fprintf(fp, "    logic signed [63:0]  n_c8_g1_post_c;\n");
    fprintf(fp, "    logic signed [31:0]  n_c8_xp_c;\n");
    fprintf(fp, "    logic [63:0]         n_c8_dt_res_c, n_c9_j0_next_c, n_c9_j1_next_c, n_c9_sub_next_c;\n");
    fprintf(fp, "    logic [15:0]         n_c10_chart_next_c, n_c10_mode_proj_c;\n");
    fprintf(fp, "    logic                n_c10_conv_c;\n");

    if (smap->c0 >= 0) fprintf(fp, "    wire  signed [32:0]  n_c0_dc_q       = $signed(sh_lane%d_q[32:0]);\n", ra->reqs[smap->c0].phys_lane);
    if (smap->c1 >= 0) fprintf(fp, "    wire  signed [63:0]  n_c1_dsq_q      = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->c1].phys_lane);
    if (smap->c2 >= 0) fprintf(fp, "    wire  signed [65:0]  n_c2_g1dc_lo_q  = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->c2].phys_lane);
    if (smap->c3 >= 0) fprintf(fp, "    wire  signed [65:0]  n_c3_g1dc_hi_q  = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->c3].phys_lane);
    if (smap->c4 >= 0) fprintf(fp, "    wire  signed [63:0]  n_c4_r2_dc_q    = $signed(sh_lane%d_q[63:0]);\n", ra->reqs[smap->c4].phys_lane);
    if (smap->c5 >= 0) fprintf(fp, "    wire  signed [65:0]  n_c5_r2dsq_lo_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->c5].phys_lane);
    if (smap->c6 >= 0) fprintf(fp, "    wire  signed [65:0]  n_c6_r2dsq_hi_q = $signed(sh_lane%d_q[65:0]);\n", ra->reqs[smap->c6].phys_lane);
    if (smap->c7_lo >= 0)
        fprintf(fp, "    wire  signed [127:0] n_c7_g0_cand_q  = $signed({{29{sh_lane%d_q[34]}}, sh_lane%d_q[34:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->c7_hi].phys_lane, ra->reqs[smap->c7_hi].phys_lane, ra->reqs[smap->c7_lo].phys_lane);
    if (smap->c8_lo >= 0) {
        fprintf(fp, "    wire  signed [127:0] n_c8_g0_post_q  = fast_path_q ? g0_q           : $signed({{29{sh_lane%d_q[34]}}, sh_lane%d_q[34:0], sh_lane%d_q[63:0]});\n",
                ra->reqs[smap->c8_hi].phys_lane, ra->reqs[smap->c8_hi].phys_lane, ra->reqs[smap->c8_lo].phys_lane);
        fprintf(fp, "    wire  signed [63:0]  n_c8_g1_post_q  = fast_path_q ? g1_q           : $signed(sh_lane%d_q[63:0]);\n",
                ra->reqs[smap->c8_g1].phys_lane);
        fprintf(fp, "    wire  signed [31:0]  n_c8_xp_q       = fast_path_q ? x_in_clamped_c : $signed(sh_lane%d_q[63:32]);\n",
                ra->reqs[smap->c8_xpdt].phys_lane);
        fprintf(fp, "    wire         [63:0]  n_c8_dt_res_q   = fast_path_q ? 64'd0          : {32'd0, sh_lane%d_q[31:0]};\n",
                ra->reqs[smap->c8_xpdt].phys_lane);
    }
    if (smap->c9_j0 >= 0) {
        fprintf(fp, "    wire         [63:0]  n_c9_j0_next_q  = sh_lane%d_q[63:0];\n", ra->reqs[smap->c9_j0].phys_lane);
        fprintf(fp, "    wire         [63:0]  n_c9_j1_next_q  = sh_lane%d_q[63:0];\n", ra->reqs[smap->c9_j1].phys_lane);
        fprintf(fp, "    wire         [63:0]  n_c9_sub_next_q = sh_lane%d_q[63:0];\n", ra->reqs[smap->c9_sub].phys_lane);
    }
    fprintf(fp, "\n");

    /* Phase 0 (`PRE_LOGIC`): Carry-Select Increment + Single-Path Clamp (`c0_delta_cand`) */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 0 (`PRE_LOGIC`): Speculative Carry-Select Step Clamp (`c0_delta_cand`)\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        logic               round_up;\n");
    fprintf(fp, "        logic        [63:0] cap64;\n");
    fprintf(fp, "        logic               over_cap;\n");
    fprintf(fp, "        logic        [31:0] mag_r1, mag32;\n");
    fprintf(fp, "        logic signed [33:0] step34;\n");
    fprintf(fp, "        logic signed [31:0] xp32;\n");
    fprintf(fp, "        round_up  = (~pre_sat_q) & (sr_rem_q > ((sr_aux_q[63:0] - 64'd1) >> 1));\n");
    fprintf(fp, "        cap64     = {32'd0, $unsigned(dt_max_q)};\n");
    fprintf(fp, "        over_cap  = pre_sat_q | (sr_acc_q >= cap64);\n");
    fprintf(fp, "        mag_r1    = sr_acc_q[31:0] + 32'd1;\n");
    fprintf(fp, "        mag32     = over_cap ? $unsigned(dt_max_q) : (round_up ? mag_r1 : sr_acc_q[31:0]);\n");
    fprintf(fp, "        step34    = div_neg_q ? -$signed({2'b00, mag32}) : $signed({2'b00, mag32});\n");
    fprintf(fp, "        xp32      = clamp_sym34_to_32($signed({{2{x_in_clamped_c[31]}}, x_in_clamped_c}) + step34);\n");
    fprintf(fp, "        n_c0_dc_c = $signed({xp32[31], xp32}) - $signed({x_in_clamped_c[31], x_in_clamped_c});\n");
    fprintf(fp, "    end\n\n");

    /* Phase 1 (`DSP33`): Instantiate `dsp_count` `ele_mul33_atom` units and drive muxes */
    dsp_expr_scratch_t dsp_sc;
    dsp_bind_t         binds[18];
    build_dsp_binds(cones, base_ca, base_cb, base_cc, &dsp_sc, binds);

    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 1 (`DSP33`): %d Operand-Isolated `ele_mul33_atom` DSP Slices\n", dsp_count);
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    logic [%d:0]        dsp_en_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][32:0]  dsp_a_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][32:0]  dsp_b_c;\n", dsp_count - 1);
    fprintf(fp, "    logic [%d:0][65:0]  dsp_prod_w;\n\n", dsp_count - 1);

    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        dsp_en_c = %d'd0;\n", dsp_count);
    for (int d = 0; d < dsp_count; ++d) {
        const char *best_a = "33'd0";
        const char *best_b = "33'd0";
        int best_ca = 0, best_cb = 0;
        for (int i = 0; i < 18; ++i) {
            const dag_cone_t *ci = (binds[i].cone_id == 0) ? ca : ((binds[i].cone_id == 1) ? cb : cc);
            if (ci->nodes[binds[i].node_idx].dsp_slot != d) continue;
            int cnt_a = 0, cnt_b = 0;
            for (int j = 0; j < 18; ++j) {
                const dag_cone_t *cj = (binds[j].cone_id == 0) ? ca : ((binds[j].cone_id == 1) ? cb : cc);
                if (cj->nodes[binds[j].node_idx].dsp_slot != d) continue;
                if (strcmp(binds[i].op_a_expr, binds[j].op_a_expr) == 0) cnt_a++;
                if (strcmp(binds[i].op_b_expr, binds[j].op_b_expr) == 0) cnt_b++;
            }
            if (cnt_a > best_ca) { best_ca = cnt_a; best_a = binds[i].op_a_expr; }
            if (cnt_b > best_cb) { best_cb = cnt_b; best_b = binds[i].op_b_expr; }
        }
        fprintf(fp, "        dsp_a_c[%d] = %s;\n", d, best_a);
        fprintf(fp, "        dsp_b_c[%d] = %s;\n", d, best_b);
    }
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
            fprintf(fp, "                dsp_en_c[%d] = %s;\n", slot, binds[b].en_expr);
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

    if (!ca->nodes[0].needs_reg) fprintf(fp, "    assign n_a0_ug_u_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[0].dsp_slot);
    if (!ca->nodes[1].needs_reg) fprintf(fp, "    assign n_a1_b1_x_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[1].dsp_slot);
    if (!ca->nodes[2].needs_reg) fprintf(fp, "    assign n_a2_x_sq_c     = dsp_prod_w[%d][63:0];\n", ca->nodes[2].dsp_slot);
    if (!ca->nodes[3].needs_reg) fprintf(fp, "    assign n_a3_b2_x_c     = $signed(dsp_prod_w[%d][63:0]);\n", ca->nodes[3].dsp_slot);
    if (!ca->nodes[5].needs_reg) fprintf(fp, "    assign n_a5_b2xsq_lo_c = $signed(dsp_prod_w[%d]);\n", ca->nodes[5].dsp_slot);
    if (!ca->nodes[6].needs_reg) fprintf(fp, "    assign n_a6_b2xsq_hi_c = $signed(dsp_prod_w[%d]);\n", ca->nodes[6].dsp_slot);

    if (!cb->nodes[0].needs_reg) fprintf(fp, "    assign n_b0_g1_ll_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[0].dsp_slot);
    if (!cb->nodes[1].needs_reg) fprintf(fp, "    assign n_b1_g1_hl_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[1].dsp_slot);
    if (!cb->nodes[2].needs_reg) fprintf(fp, "    assign n_b2_g1_hh_c    = $signed(dsp_prod_w[%d]);\n", cb->nodes[2].dsp_slot);
    if (!cb->nodes[3].needs_reg) fprintf(fp, "    assign n_b3_g0r2_0_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[3].dsp_slot);
    if (!cb->nodes[4].needs_reg) fprintf(fp, "    assign n_b4_g0r2_1_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[4].dsp_slot);
    if (!cb->nodes[5].needs_reg) fprintf(fp, "    assign n_b5_g0r2_2_c   = $signed(dsp_prod_w[%d]);\n", cb->nodes[5].dsp_slot);

    if (!cc->nodes[1].needs_reg) fprintf(fp, "    assign n_c1_dsq_c      = $signed(dsp_prod_w[%d][63:0]);\n", cc->nodes[1].dsp_slot);
    if (!cc->nodes[2].needs_reg) fprintf(fp, "    assign n_c2_g1dc_lo_c  = $signed(dsp_prod_w[%d]);\n", cc->nodes[2].dsp_slot);
    if (!cc->nodes[3].needs_reg) fprintf(fp, "    assign n_c3_g1dc_hi_c  = $signed(dsp_prod_w[%d]);\n", cc->nodes[3].dsp_slot);
    if (!cc->nodes[4].needs_reg) fprintf(fp, "    assign n_c4_r2_dc_c    = $signed(dsp_prod_w[%d][63:0]);\n", cc->nodes[4].dsp_slot);
    if (!cc->nodes[5].needs_reg) fprintf(fp, "    assign n_c5_r2dsq_lo_c = $signed(dsp_prod_w[%d]);\n", cc->nodes[5].dsp_slot);
    if (!cc->nodes[6].needs_reg) fprintf(fp, "    assign n_c6_r2dsq_hi_c = $signed(dsp_prod_w[%d]);\n", cc->nodes[6].dsp_slot);
    fprintf(fp, "\n");

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

    /* Phase 2 (`POST_LOGIC`): Topological combinational logic for Cones A, B, D, C */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Phase 2 (`POST_LOGIC`): Topological Cone Combinators\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        logic signed [63:0]  j0_in, j1_in;\n");
    fprintf(fp, "        logic                sub_valid;\n");
    fprintf(fp, "        logic [32:0]         j0_sub_in;\n");
    fprintf(fp, "        logic [15:0]         j1_sub_in;\n");
    fprintf(fp, "        logic signed [66:0]  b2_xsq_hi67, g1_dc_hi67, r2_dsq_hi67;\n");
    fprintf(fp, "        logic signed [64:0]  b0_b1_q32;\n");
    fprintf(fp, "        logic signed [80:0]  g0_op_hi81;\n");
    fprintf(fp, "        logic signed [77:0]  g0_au_hi78;\n");
    fprintf(fp, "        logic signed [95:0]  b7_hi96;\n");
    fprintf(fp, "        logic signed [97:0]  push_hi98;\n");
    fprintf(fp, "        logic signed [98:0]  cand_s99;\n");
    fprintf(fp, "        logic        [96:0]  j0_au_next97;\n");
    fprintf(fp, "        logic signed [94:0]  j0_q49;\n");
    fprintf(fp, "        logic signed [127:0] b2_xsq128;\n");
    fprintf(fp, "        logic                d_neg;\n");
    fprintf(fp, "        logic signed [63:0]  signed_d;\n");
    fprintf(fp, "        logic signed [65:0]  q_cit66, div_den66;\n");
    fprintf(fp, "        logic signed [98:0]  neg_g0_s99;\n");
    fprintf(fp, "        logic signed [127:0] neg_g0;\n");
    fprintf(fp, "        logic                has_r2, real_roots;\n");
    fprintf(fp, "        logic [127:0]        anum128;\n");
    fprintf(fp, "        logic [63:0]         aden64;\n");
    fprintf(fp, "        logic                den_zero;\n");
    fprintf(fp, "        logic signed [127:0] g1_dc128, r2_dsq128;\n");
    fprintf(fp, "        logic                lyap_ok;\n");
    fprintf(fp, "        logic signed [32:0]  delta_act;\n");
    fprintf(fp, "        logic [32:0]         delta_mag33;\n");
    fprintf(fp, "        logic signed [63:0]  j0_op_out, j1_op_out, j0_au_out, j1_au_out;\n");
    fprintf(fp, "        logic [15:0]         raw_chart;\n\n");

    /* Cone A POST_LOGIC (`b0_eff` computed in exact 64b `(b0<<16) - ug_u`, zero-carry low limbs) */
    const char *a5_a7      = ref_sig(ca, 5, 7, "n_a5_b2xsq_lo_c", "n_a5_b2xsq_lo_q");
    const char *a6_a7      = ref_sig(ca, 6, 7, "n_a6_b2xsq_hi_c", "n_a6_b2xsq_hi_q");
    const char *b0e_a7     = ref_sig(ca, 4, 7, "n_a4_b0_eff64_c", "b0_eff64_q");
    const char *b1x_a7     = ref_sig(ca, 1, 7, "n_a1_b1_x_c",     "n_a1_b1_x_q");
    const char *b0e_a9     = ref_sig(ca, 4, 9, "n_a4_b0_eff64_c", "b0_eff64_q");
    fprintf(fp, "        // Cone A (`A4`, `A7`, `A8`, `A9`, `A10`)\n");
    fprintf(fp, "        n_a4_b0_eff64_c = $signed({{16{b0_q[31]}}, b0_q, 16'd0}) - %s;\n",
            ref_sig(ca, 0, 4, "n_a0_ug_u_c", "n_a0_ug_u_q"));
    fprintf(fp, "        b2_xsq_hi67     = $signed({%s[65], %s}) + $signed({{33{%s[65]}}, %s[65:32]});\n",
            a6_a7, a6_a7, a5_a7, a5_a7);
    fprintf(fp, "        b2_xsq128       = $signed({{29{b2_xsq_hi67[66]}}, b2_xsq_hi67, %s[31:0]});\n", a5_a7);
    fprintf(fp, "        b0_b1_q32       = $signed({%s[63], %s}) + $signed({%s[63], %s});\n",
            b0e_a7, b0e_a7, b1x_a7, b1x_a7);
    fprintf(fp, "        g0_op_hi81      = $signed(b2_xsq128[97:17]) + $signed({{16{b0_b1_q32[64]}}, b0_b1_q32});\n");
    fprintf(fp, "        n_a7_g0_op_c    = $signed({{30{g0_op_hi81[80]}}, g0_op_hi81, b2_xsq128[16:0]});\n");
    fprintf(fp, "        n_a8_g1_op_c    = $signed({{16{b1_q[31]}}, b1_q, 16'd0}) + %s;\n",
            ref_sig(ca, 3, 8, "n_a3_b2_x_c", "n_a3_b2_x_q"));
    fprintf(fp, "        j0_in           = clamp_j0_sym64($signed(state_w_q[0]));\n");
    fprintf(fp, "        j1_in           = clamp_j1_sym64($signed(state_w_q[1]));\n");
    fprintf(fp, "        sub_valid     = ($signed(state_w_q[0]) == j0_in) & ($signed(state_w_q[1]) == j1_in);\n");
    fprintf(fp, "        j0_sub_in       = sub_valid ? state_w_q[7][32:0]  : 33'd0;\n");
    fprintf(fp, "        j1_sub_in       = sub_valid ? state_w_q[7][63:48] : 16'd0;\n");
    fprintf(fp, "        j0_q49          = $signed({j0_in[61:0], j0_sub_in});\n");
    fprintf(fp, "        n_a9_j1_q32_c   = $signed({j1_in[47:0], j1_sub_in});\n");
    fprintf(fp, "        g0_au_hi78      = $signed(j0_q49[94:17]) - $signed({{14{%s[63]}}, %s});\n",
            b0e_a9, b0e_a9);
    fprintf(fp, "        n_a9_g0_au_c    = $signed({{33{g0_au_hi78[77]}}, g0_au_hi78, j0_q49[16:0]});\n");
    fprintf(fp, "        n_a10_op_mask_c = op_mask_q;\n");
    fprintf(fp, "        n_a10_g0_c      = n_a10_op_mask_c ? %s : %s;\n",
            ref_sig(ca, 7, 10, "n_a7_g0_op_c", "n_a7_g0_op_q"),
            ref_sig(ca, 9, 10, "n_a9_g0_au_c", "n_a9_g0_au_q"));
    fprintf(fp, "        n_a10_g1_c      = n_a10_op_mask_c ? %s : %s;\n",
            ref_sig(ca, 8, 10, "n_a8_g1_op_c", "n_a8_g1_op_q"),
            ref_sig(ca, 9, 10, "n_a9_j1_q32_c", "n_a9_j1_q32_q"));
    fprintf(fp, "        n_a10_has_r2_c  = r2_nz_c;\n");
    fprintf(fp, "        n_a10_active_c  = (n_a10_g0_c != 128'sd0) & ((n_a10_g1_c != 64'sd0) | n_a10_has_r2_c);\n\n");

    /* Cone B POST_LOGIC (`ll < 2^64` folds `(hh <<< 64) + ll` to `{hh[63:0], ll[63:0]}`) */
    const char *b0_b6 = ref_sig(cb, 0, 6, "n_b0_g1_ll_c",  "n_b0_g1_ll_q");
    const char *b1_b6 = ref_sig(cb, 1, 6, "n_b1_g1_hl_c",  "n_b1_g1_hl_q");
    const char *b2_b6 = ref_sig(cb, 2, 6, "n_b2_g1_hh_c",  "n_b2_g1_hh_q");
    const char *b3_b7 = ref_sig(cb, 3, 7, "n_b3_g0r2_0_c", "n_b3_g0r2_0_q");
    const char *b4_b7 = ref_sig(cb, 4, 7, "n_b4_g0r2_1_c", "n_b4_g0r2_1_q");
    const char *b5_b7 = ref_sig(cb, 5, 7, "n_b5_g0r2_2_c", "n_b5_g0r2_2_q");
    fprintf(fp, "        // Cone B (`B6`, `B7`, `B8`)\n");
    fprintf(fp, "        n_b6_g1_sq_c  = ($signed({{62{%s[65]}}, %s}) <<< 33) +\n", b1_b6, b1_b6);
    fprintf(fp, "                        $signed({%s[63:0], %s[63:0]});\n", b2_b6, b0_b6);
    fprintf(fp, "        b7_hi96       = $signed({%s[63:0], 32'd0}) +\n", b5_b7);
    fprintf(fp, "                        $signed({{30{%s[65]}}, %s}) +\n", b4_b7, b4_b7);
    fprintf(fp, "                        $signed({{62{%s[65]}}, %s[65:32]});\n", b3_b7, b3_b7);
    fprintf(fp, "        n_b7_g0_r2_c  = $signed({b7_hi96, %s[31:0]});\n", b3_b7);
    fprintf(fp, "        n_b8_disc_c   = %s - %s;\n",
            ref_sig(cb, 6, 8, "n_b6_g1_sq_c", "n_b6_g1_sq_q"),
            ref_sig(cb, 7, 8, "n_b7_g0_r2_c", "n_b7_g0_r2_q"));
    fprintf(fp, "        n_b8_pos_nz_c = (~n_b8_disc_c[127]) & (n_b8_disc_c != 128'sd0);\n\n");

    /* Cone D POST_LOGIC (`d_mag_w` aliases `sr_acc_q`, saving 64 dedicated DFFs!) */
    fprintf(fp, "        // Cone D (`D0`, `D1`)\n");
    fprintf(fp, "        has_r2           = r2_nz_c;\n");
    fprintf(fp, "        real_roots       = ~d_mag_w[63];\n");
    fprintf(fp, "        d_neg            = (g1_q < 64'sd0) | ((g1_q == 64'sd0) & (r2_q < 32'sd0));\n");
    fprintf(fp, "        signed_d         = d_neg ? $signed(64'd0 - {1'b0, d_mag_w[62:0]}) : $signed({1'b0, d_mag_w[62:0]});\n");
    fprintf(fp, "        q_cit66          = $signed({{2{g1_q[63]}}, g1_q}) + $signed({{2{signed_d[63]}}, signed_d});\n");
    fprintf(fp, "        neg_g0_s99       = 99'sd0 - $signed({g0_s98_q[97], g0_s98_q});\n");
    fprintf(fp, "        neg_g0           = $signed({{29{neg_g0_s99[98]}}, neg_g0_s99});\n");
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
    const char *c2_c7 = ref_sig(cc, 2, 7, "n_c2_g1dc_lo_c",  "n_c2_g1dc_lo_q");
    const char *c3_c7 = ref_sig(cc, 3, 7, "n_c3_g1dc_hi_c",  "n_c3_g1dc_hi_q");
    const char *c5_c7 = ref_sig(cc, 5, 7, "n_c5_r2dsq_lo_c", "n_c5_r2dsq_lo_q");
    const char *c6_c7 = ref_sig(cc, 6, 7, "n_c6_r2dsq_hi_c", "n_c6_r2dsq_hi_q");
    fprintf(fp, "        // Cone C (`C7`, `C8`, `C9`, `C10`)\n");
    fprintf(fp, "        g1_dc_hi67      = $signed({%s[65], %s}) + $signed({{33{%s[65]}}, %s[65:32]});\n",
            c3_c7, c3_c7, c2_c7, c2_c7);
    fprintf(fp, "        g1_dc128        = $signed({{29{g1_dc_hi67[66]}}, g1_dc_hi67, %s[31:0]});\n", c2_c7);
    fprintf(fp, "        r2_dsq_hi67     = $signed({%s[65], %s}) + $signed({{33{%s[65]}}, %s[65:32]});\n",
            c6_c7, c6_c7, c5_c7, c5_c7);
    fprintf(fp, "        r2_dsq128       = $signed({{29{r2_dsq_hi67[66]}}, r2_dsq_hi67, %s[31:0]});\n", c5_c7);
    fprintf(fp, "        push_hi98       = $signed({g1_dc128[96:0], 1'b0}) + $signed(r2_dsq128[97:0]);\n");
    fprintf(fp, "        cand_s99        = $signed({g0_s98_q[97], g0_s98_q}) + $signed({push_hi98[97], push_hi98});\n");
    fprintf(fp, "        n_c7_g0_cand_c  = $signed({{29{cand_s99[98]}}, cand_s99});\n");
    fprintf(fp, "        lyap_ok         = (~fast_path_q) & (abs128(%s) <= abs128(g0_q));\n",
            ref_sig(cc, 7, 8, "n_c7_g0_cand_c", "n_c7_g0_cand_q"));
    fprintf(fp, "        delta_act       = lyap_ok ? %s : 33'sd0;\n",
            ref_sig(cc, 0, 8, "n_c0_dc_c", "n_c0_dc_q"));
    fprintf(fp, "        n_c8_xp_c       = $signed(x_in_clamped_c + delta_act[31:0]);\n");
    fprintf(fp, "        delta_mag33     = delta_act[32] ? (33'd0 - $unsigned(delta_act)) : $unsigned(delta_act);\n");
    fprintf(fp, "        n_c8_dt_res_c   = {31'd0, delta_mag33};\n");
    fprintf(fp, "        n_c8_g0_post_c  = lyap_ok ? %s : g0_q;\n",
            ref_sig(cc, 7, 8, "n_c7_g0_cand_c", "n_c7_g0_cand_q"));
    fprintf(fp, "        n_c8_g1_post_c  = g1_q + (lyap_ok ? %s : 64'sd0);\n",
            ref_sig(cc, 4, 8, "n_c4_r2_dc_c", "n_c4_r2_dc_q"));

    const char *g0p_c9 = ref_sig(cc, 8, 9, "n_c8_g0_post_c", "n_c8_g0_post_q");
    const char *g1p_c9 = ref_sig(cc, 8, 9, "n_c8_g1_post_c", "n_c8_g1_post_q");
    fprintf(fp, "        j0_op_out       = sym_shr128_33(%s);\n", g0p_c9);
    fprintf(fp, "        j1_op_out       = sym_shr64_16(%s);\n", g1p_c9);
    fprintf(fp, "        j0_au_next97    = {$unsigned(%s[96:17]) + {{16{b0_eff64_q[63]}}, $unsigned(b0_eff64_q)}, %s[16:0]};\n",
            g0p_c9, g0p_c9);
    fprintf(fp, "        j0_au_out       = clamp_j0_sym64($signed(j0_au_next97[96:33]));\n");
    fprintf(fp, "        j1_au_out       = clamp_j1_sym64($signed({{16{%s[63]}}, %s[63:16]}));\n",
            g1p_c9, g1p_c9);
    fprintf(fp, "        n_c9_j0_next_c  = $unsigned(op_mask_q ? j0_op_out : j0_au_out);\n");
    fprintf(fp, "        n_c9_j1_next_c  = $unsigned(op_mask_q ? j1_op_out : j1_au_out);\n");
    fprintf(fp, "        n_c9_sub_next_c = op_mask_q ? 64'd0 : {%s[15:0], 15'd0, j0_au_next97[32:0]};\n", g1p_c9);

    const char *xp_c10  = ref_sig(cc, 8, 10, "n_c8_xp_c",      "n_c8_xp_q");
    const char *g0p_c10 = ref_sig(cc, 8, 10, "n_c8_g0_post_c", "n_c8_g0_post_q");
    fprintf(fp, "        raw_chart          = (%s > b_enter_q) ? {8'd0, next_hi_q}\n", xp_c10);
    fprintf(fp, "                           : ((%s < b_exit_q) ? {8'd0, next_lo_q} : state_w_q[5][31:16]);\n", xp_c10);
    fprintf(fp, "        n_c10_chart_next_c = raw_chart & cap_mask16_q;\n");
    fprintf(fp, "        n_c10_mode_proj_c  = (mode_req_q & (~kan_lock_q)) | (state_w_q[5][15:0] & kan_lock_q);\n");
    fprintf(fp, "        n_c10_conv_c       = (abs128(%s) <= 128'h0000_0000_0000_0000_0000_0008_0000_0000);\n", g0p_c10);
    fprintf(fp, "        if (&{1'b0, j0_in[63:62], %s[65:64], %s[65:64], %s[65:64], b2_xsq128[127:98], g1_dc128[127:97], r2_dsq128[127:98], n_a7_g0_op_c[127:98], n_a9_g0_au_c[127:95], n_a10_g0_c[127:98], n_d0_div_num_c[126:99], n_c7_g0_cand_c[127:99], n_c8_g0_post_c[127:99], delta_mag33[32], n_c8_dt_res_c[63:32]}) begin end\n",
            b0_b6, b2_b6, b5_b7);
    fprintf(fp, "    end\n\n");

    /* Pass-2 Shared Physical Lane Write-Enable & Deduplicated Data Steering Muxes */
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    // Pass-2 Clock-Enabled Shared Physical Cut Register Steering (`0` idle flips)\n");
    fprintf(fp, "    // =========================================================================\n");
    fprintf(fp, "    always_comb begin\n");
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, "        sh_lane%d_we_c = 1'b0;\n", ln);
    }
    fprintf(fp, "        case (st_q)\n");
    for (int s = 1; s < num_st; ++s) {
        bool any_wr = false;
        for (int r = 0; r < ra->num_reqs; ++r) {
            if (ra->reqs[r].birth_state == s) {
                any_wr = true;
                break;
            }
        }
        if (!any_wr) {
            continue;
        }
        fprintf(fp, "            %d'd%d: begin\n", st_w, s);
        for (int r = 0; r < ra->num_reqs; ++r) {
            const ssa_scalar_live_t *rq = &ra->reqs[r];
            if (rq->birth_state != s) {
                continue;
            }
            fprintf(fp, "                // %s [%d -> %d] -> sh_lane%d\n",
                    rq->tag, rq->birth_state, rq->last_read_state, rq->phys_lane);
            fprintf(fp, "                sh_lane%d_we_c = %s;\n",
                    rq->phys_lane, rq->we_guard);
        }
        fprintf(fp, "            end\n");
    }
    fprintf(fp, "            default: begin end\n");
    fprintf(fp, "        endcase\n");
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        int         lw = ra->lane_width[ln];
        int         u_w[MAX_SCALAR_REQS];
        const char *u_expr[MAX_SCALAR_REQS];
        int         num_u = 0;
        for (int r = 0; r < ra->num_reqs; ++r) {
            const ssa_scalar_live_t *rq = &ra->reqs[r];
            if (rq->phys_lane != ln) continue;
            bool seen = false;
            for (int u = 0; u < num_u; ++u) {
                if (u_w[u] == rq->bit_width && strcmp(u_expr[u], rq->val_expr) == 0) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                u_w[num_u]    = rq->bit_width;
                u_expr[num_u] = rq->val_expr;
                num_u++;
            }
        }
        if (num_u == 1) {
            fprintf(fp, "        sh_lane%d_d_c = ", ln);
            emit_lane_rhs(fp, lw, u_w[0], u_expr[0]);
            fprintf(fp, ";\n");
        } else {
            for (int u = 0; u < num_u - 1; ++u) {
                fprintf(fp, "        %sif (", (u == 0) ? "" : "else ");
                bool first_st = true;
                for (int r = 0; r < ra->num_reqs; ++r) {
                    const ssa_scalar_live_t *rq = &ra->reqs[r];
                    if (rq->phys_lane == ln && rq->bit_width == u_w[u] &&
                        strcmp(rq->val_expr, u_expr[u]) == 0) {
                        if (!first_st) fprintf(fp, " || ");
                        fprintf(fp, "(st_q == %d'd%d)", st_w, rq->birth_state);
                        first_st = false;
                    }
                }
                fprintf(fp, ") sh_lane%d_d_c = ", ln);
                emit_lane_rhs(fp, lw, u_w[u], u_expr[u]);
                fprintf(fp, ";\n");
            }
            fprintf(fp, "        else sh_lane%d_d_c = ", ln);
            emit_lane_rhs(fp, lw, u_w[num_u - 1], u_expr[num_u - 1]);
            fprintf(fp, ";\n");
        }
    }
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
    fprintf(fp, "            fast_path_q  <= 1'b0;\n");
    fprintf(fp, "            b0_eff64_q   <= 64'sd0;\n");
    fprintf(fp, "            g0_s98_q     <= 98'sd0;\n");
    fprintf(fp, "            g1_q         <= 64'sd0;\n");
    fprintf(fp, "            div_neg_q    <= 1'b0;\n");
    fprintf(fp, "            pre_sat_q    <= 1'b0;\n");
    fprintf(fp, "            sr_rem_q     <= 64'd0;\n");
    fprintf(fp, "            sr_acc_q     <= 64'd0;\n");
    fprintf(fp, "            sr_aux_q     <= 128'd0;\n");
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, "            sh_lane%d_q   <= %d'd0;\n", ln, ra->lane_width[ln]);
    }
    fprintf(fp, "        end else begin\n");
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, "            if (sh_lane%d_we_c) sh_lane%d_q <= sh_lane%d_d_c;\n", ln, ln, ln);
    }
    fprintf(fp, "            done_q <= 1'b0;\n");
    fprintf(fp, "            case (st_q)\n");

    /* ST_IDLE (0): 64-bit streaming ingress (constant-index word enables kill 512b barrel shifter) */
    fprintf(fp, "                %d'd%d: begin\n", st_w, st_idle);
    fprintf(fp, "                    if (wr_valid_i) begin\n");
    fprintf(fp, "                        case (wr_kind_i)\n");
    fprintf(fp, "                            2'd0: begin\n");
    fprintf(fp, "                                case (wr_idx_i)\n");
    for (int wi = 0; wi < 8; ++wi) {
        fprintf(fp, "                                    3'd%d: state_w_q[%d] <= wr_data_i;\n", wi, wi);
    }
    fprintf(fp, "                                endcase\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            2'd1: begin\n");
    fprintf(fp, "                                case (wr_idx_i)\n");
    fprintf(fp, "                                    3'd0: b0_q <= wr_clamped32_c;\n");
    fprintf(fp, "                                    3'd1: b1_q <= wr_clamped32_c;\n");
    fprintf(fp, "                                    3'd2: b2_q <= wr_clamped32_c;\n");
    fprintf(fp, "                                    3'd3: begin\n");
    fprintf(fp, "                                        b_exit_q  <= ex_c;\n");
    fprintf(fp, "                                        b_enter_q <= (en_c < ex_c) ? ex_c : en_c;\n");
    fprintf(fp, "                                    end\n");
    fprintf(fp, "                                    3'd4: begin\n");
    fprintf(fp, "                                        dt_max_q <= (dt_c < 32'sd1) ? 32'sd1 : dt_c;\n");
    fprintf(fp, "                                        u_gain_q <= en_c;\n");
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
    fprintf(fp, "                                    u_in_q      <= wr_clamped32_c;\n");
    fprintf(fp, "                                    fast_path_q <= 1'b0;\n");
    fprintf(fp, "                                    st_q        <= %d'd%d;\n", st_w, base_ca);
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            default: begin end\n");
    fprintf(fp, "                        endcase\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    /* Cone A Stages */
    for (int s = 0; s < ca->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_ca + s);
        if (s == ca->nodes[4].stage) {
            fprintf(fp, "                    b0_eff64_q <= n_a4_b0_eff64_c;\n");
        }
        if (s < ca->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_ca + s + 1);
        } else {
            fprintf(fp, "                    g0_s98_q <= n_a10_g0_c[97:0];\n");
            fprintf(fp, "                    g1_q     <= n_a10_g1_c;\n");
            fprintf(fp, "                    if (!n_a10_active_c) begin\n");
            fprintf(fp, "                        fast_path_q <= 1'b1;\n");
            fprintf(fp, "                        st_q        <= %d'd%d;\n", st_w, base_cc + cc->nodes[9].stage);
            fprintf(fp, "                    end else if (!n_a10_has_r2_c) begin\n");
            fprintf(fp, "                        st_q        <= %d'd%d;\n", st_w, base_cd);
            fprintf(fp, "                    end else begin\n");
            fprintf(fp, "                        st_q        <= %d'd%d;\n", st_w, base_cb);
            fprintf(fp, "                    end\n");
        }
        fprintf(fp, "                end\n");
    }

    /* Cone B Stages (`Discriminant`) */
    for (int s = 0; s < cb->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_cb + s);
        if (s < cb->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_cb + s + 1);
        } else {
            fprintf(fp, "                    if (n_b8_disc_c[127]) begin\n");
            fprintf(fp, "                        sr_acc_q <= 64'h8000_0000_0000_0000;\n");
            fprintf(fp, "                        st_q     <= %d'd%d;\n", st_w, base_cd);
            fprintf(fp, "                    end else if (!n_b8_pos_nz_c) begin\n");
            fprintf(fp, "                        sr_acc_q <= 64'd0;\n");
            fprintf(fp, "                        st_q     <= %d'd%d;\n", st_w, base_cd);
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

    /* ST_ISQRT_LOOP (`d_mag` rounded directly into `sr_acc_q` on final iteration!) */
    fprintf(fp, "                %d'd%d: begin\n", st_w, st_isqrt);
    fprintf(fp, "                    sr_rem_q <= rad2_rem_w[%d];\n", k_rad);
    fprintf(fp, "                    sr_aux_q <= rad2_aux_w[%d];\n", k_rad);
    fprintf(fp, "                    if (iter_q == %d'd0) begin\n", iter_w);
    fprintf(fp, "                        sr_acc_q <= rad2_acc_w[%d] + {63'd0, (rad2_rem_w[%d] > rad2_acc_w[%d])};\n",
            k_rad, k_rad, k_rad);
    fprintf(fp, "                        st_q     <= %d'd%d;\n", st_w, base_cd);
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        sr_acc_q <= rad2_acc_w[%d];\n", k_rad);
    fprintf(fp, "                        iter_q   <= iter_q - %d'd1;\n", iter_w);
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    /* Cone D Stages (`Citardauq & Divider Normalization`: `pre_sat` & `sr_aux_q[63:0]` clock-gated!) */
    for (int s = 0; s < cd->num_stages; ++s) {
        fprintf(fp, "                %d'd%d: begin\n", st_w, base_cd + s);
        if (s < cd->num_stages - 1) {
            fprintf(fp, "                    st_q <= %d'd%d;\n", st_w, base_cd + s + 1);
        } else {
            const char *dneg_src = ref_sig(cd, 0, 1, "n_d0_div_neg_c", "n_d0_div_neg_q");
            fprintf(fp, "                    div_neg_q <= %s;\n", dneg_src);
            fprintf(fp, "                    pre_sat_q <= n_d1_pre_sat_c;\n");
            fprintf(fp, "                    if (n_d1_pre_sat_c) begin\n");
            fprintf(fp, "                        st_q           <= %d'd%d;\n", st_w, base_cc);
            fprintf(fp, "                    end else begin\n");
            fprintf(fp, "                        sr_rem_q       <= n_d1_init_rem_c;\n");
            fprintf(fp, "                        sr_acc_q       <= n_d1_init_quot_c;\n");
            fprintf(fp, "                        sr_aux_q[63:0] <= n_d1_safe_den_c;\n");
            fprintf(fp, "                        iter_q         <= %d'd%d;\n", iter_w, rad_iters - 1);
            fprintf(fp, "                        st_q           <= %d'd%d;\n", st_w, st_udiv);
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

    /* Internal Datapath DFF Toggle Snapshot (`ONLY` true physical registers!) */
    const int pad_bits = 4096 - core_dff_bits;
    fprintf(fp, "`ifndef SYNTHESIS\n");
    fprintf(fp, "    /* verilator lint_off UNUSEDSIGNAL */\n");
    fprintf(fp, "    logic [4095:0] sim_dff_snap_w;\n");
    fprintf(fp, "    assign sim_dff_snap_w = {\n");
    fprintf(fp, "        %d'd0,\n", pad_bits);
    fprintf(fp, "        st_q, iter_q, fast_path_q, b0_eff64_q,\n");
    fprintf(fp, "        g0_s98_q, g1_q, div_neg_q, pre_sat_q, sr_rem_q, sr_acc_q, sr_aux_q");
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, ",\n        sh_lane%d_q", ln);
    }
    fprintf(fp, "\n    };\n");
    fprintf(fp, "    /* verilator lint_on UNUSEDSIGNAL */\n");
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
    for (int ln = 0; ln < ra->num_lanes; ++ln) {
        fprintf(fp, "            assert (sh_lane%d_we_c == 1'b0);\n", ln);
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
     * MODULE 2: `ele_flow_atlas_top` (157 physical pins, Shadow-SRAM `@swap`)
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
    fprintf(fp, "    /* verilator lint_off UNUSEDPARAM */\n");
    fprintf(fp, "    localparam int SLOT_W             = (MAX_CHARTS <= 2) ? 1 : $clog2(MAX_CHARTS);\n");
    fprintf(fp, "    localparam int SRAM_DEPTH         = (MAX_CHARTS * 2) * 8;\n");
    fprintf(fp, "    localparam int SRAM_AW            = $clog2(SRAM_DEPTH);\n");
    fprintf(fp, "    localparam int SWAP_COMMIT_CYCLES = 14;\n");
    fprintf(fp, "    localparam logic [SLOT_W:0] SHADOW_SLOT = {1'b1, {SLOT_W{1'b0}}};\n");
    fprintf(fp, "    localparam logic [15:0]     HW_MASK16   = 16'(MAX_CHARTS - 1);\n");
    fprintf(fp, "    /* verilator lint_on UNUSEDPARAM */\n\n");

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

    /* Symmetric 64->32 Q16.16 clamp for `u_ext` (`-32` wrapper DFFs!) */
    fprintf(fp, "    function automatic logic signed [31:0] clamp_top64_to_32(input logic [63:0] raw);\n");
    fprintf(fp, "        logic ovf_pos, ovf_neg;\n");
    fprintf(fp, "        begin\n");
    fprintf(fp, "            ovf_pos = (~raw[63]) & (|raw[62:31]);\n");
    fprintf(fp, "            ovf_neg = raw[63] & ((~(&raw[62:31])) | (raw[30:0] == 31'd0));\n");
    fprintf(fp, "            if (ovf_pos)      clamp_top64_to_32 = 32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else if (ovf_neg) clamp_top64_to_32 = -32'sh7FFF_FFFF;\n");
    fprintf(fp, "            else              clamp_top64_to_32 = $signed(raw[31:0]);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    endfunction\n\n");

    /* Compact 24-bit candidate firewall metadata + Active Chart-Bound Cache (`0` steady-state SRAM reads!) */
    fprintf(fp, "    logic [2:0]             top_st_q;\n");
    fprintf(fp, "    logic [3:0]             sub_beat_q;\n");
    fprintf(fp, "    logic [15:0]            cap_mask_q;\n");
    fprintf(fp, "    logic [MAX_CHARTS-1:0]  slot_valid_q;\n");
    fprintf(fp, "    logic [SLOT_W-1:0]      act_slot_q;\n");
    fprintf(fp, "    logic                   bound_cache_valid_q;\n");
    fprintf(fp, "    logic [SLOT_W-1:0]      bound_cache_slot_q;\n");
    fprintf(fp, "    logic signed [31:0]     u_ext_hold32_q;\n");
    fprintf(fp, "    logic [5:0]             cand_staged_mask_q;\n");
    fprintf(fp, "    logic                   cand_b_ok_q;\n");
    fprintf(fp, "    logic                   cand_dt_ok_q;\n");
    fprintf(fp, "    logic [7:0]             cand_next_lo_q;\n");
    fprintf(fp, "    logic [7:0]             cand_next_hi_q;\n");
    fprintf(fp, "    logic [SLOT_W-1:0]      swap_slot_q;\n");
    fprintf(fp, "    logic [31:0]            swap_epoch_q;\n");
    fprintf(fp, "    logic                   step_done_q;\n");
    fprintf(fp, "    logic                   swap_done_q;\n");
    fprintf(fp, "    logic                   swap_acc_q;\n\n");

    /* Unified 1RW 64-bit Chart + Shadow-Staging SRAM (`$mem_v2`, 0 loose DFFs) */
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

    /* 24-bit Metadata Firewall Check (`1`-cycle zero-SRAM-write reject) */
    fprintf(fp, "    logic [31:0] swap_cur_ep_c;\n");
    fprintf(fp, "    logic        swap_fw_ok_c;\n");
    fprintf(fp, "    logic [7:0]  cand_staged_pad_c;\n");
    fprintf(fp, "    assign cand_staged_pad_c = {2'b00, cand_staged_mask_q};\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        swap_cur_ep_c = slot_valid_q[swap_slot_q] ? sram_rdata_w[63:32] : 32'd0;\n");
    fprintf(fp, "        swap_fw_ok_c  = (swap_epoch_q > swap_cur_ep_c) &\n");
    fprintf(fp, "                        ({8'd0, cand_next_lo_q} <= cap_mask_q) &\n");
    fprintf(fp, "                        ({8'd0, cand_next_hi_q} <= cap_mask_q) &\n");
    fprintf(fp, "                        cand_dt_ok_q &\n");
    fprintf(fp, "                        cand_b_ok_q;\n");
    fprintf(fp, "    end\n\n");

    /* Combinational control for `u_chart_sram` and `u_core` */
    fprintf(fp, "    logic [SLOT_W-1:0] step_slot_c;\n");
    fprintf(fp, "    logic              step_cache_hit_c;\n");
    fprintf(fp, "    logic [SLOT_W-1:0] swap_req_slot_c;\n");
    fprintf(fp, "    logic [SLOT_W-1:0] wr_chart_slot_c;\n");
    fprintf(fp, "    wire               unused_aux_hi_ok = &{1'b0, cmd_aux_i[15:SLOT_W]};\n\n");

    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        core_rd_idx_c    = (top_st_q == TOP_IDLE && cmd_valid_i && cmd_op_i == CMD_STEP) ? 3'd5 : cmd_word_i;\n");
    fprintf(fp, "        step_slot_c      = core_rd_data_w[16 +: SLOT_W] & cap_mask_q[SLOT_W-1:0];\n");
    fprintf(fp, "        step_cache_hit_c = bound_cache_valid_q & (step_slot_c == bound_cache_slot_q);\n");
    fprintf(fp, "        swap_req_slot_c  = cmd_aux_i[SLOT_W-1:0] & cap_mask_q[SLOT_W-1:0];\n");
    fprintf(fp, "        wr_chart_slot_c  = cmd_aux_i[SLOT_W-1:0];\n\n");
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
    fprintf(fp, "                        CMD_STAGE_CAND: begin\n");
    fprintf(fp, "                            if (cmd_word_i < 3'd6) begin\n");
    fprintf(fp, "                                sram_ce_c    = 1'b1;\n");
    fprintf(fp, "                                sram_we_c    = 1'b1;\n");
    fprintf(fp, "                                sram_addr_c  = {SHADOW_SLOT, cmd_word_i};\n");
    fprintf(fp, "                                sram_wdata_c = cmd_data_i;\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_WRITE_CHART: begin\n");
    fprintf(fp, "                            sram_ce_c    = 1'b1;\n");
    fprintf(fp, "                            sram_we_c    = 1'b1;\n");
    fprintf(fp, "                            sram_addr_c  = {1'b0, wr_chart_slot_c, cmd_word_i};\n");
    fprintf(fp, "                            sram_wdata_c = cmd_data_i;\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_STEP: begin\n");
    fprintf(fp, "                            if (!step_cache_hit_c) begin\n");
    fprintf(fp, "                                sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                                sram_we_c   = 1'b0;\n");
    fprintf(fp, "                                sram_addr_c = {1'b0, step_slot_c, 3'd0};\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                            core_wr_kind_c  = 2'd2;\n");
    fprintf(fp, "                            core_wr_idx_c   = 3'd0;\n");
    fprintf(fp, "                            core_wr_data_c  = {32'd0, cmd_aux_i, cap_mask_q};\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        CMD_SWAP: begin\n");
    fprintf(fp, "                            sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                            sram_we_c   = 1'b0;\n");
    fprintf(fp, "                            sram_addr_c = {1'b0, swap_req_slot_c, 3'd6};\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        default: begin end\n");
    fprintf(fp, "                    endcase\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n\n");

    fprintf(fp, "            TOP_FETCH_BOUND: begin\n");
    fprintf(fp, "                core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                core_wr_kind_c  = 2'd1;\n");
    fprintf(fp, "                core_wr_idx_c   = sub_beat_q[2:0];\n");
    fprintf(fp, "                core_wr_data_c  = slot_valid_q[act_slot_q] ? sram_rdata_w : 64'd0;\n");
    fprintf(fp, "                if (sub_beat_q < 4'd5) begin\n");
    fprintf(fp, "                    sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                    sram_we_c   = 1'b0;\n");
    fprintf(fp, "                    sram_addr_c = {1'b0, act_slot_q, sub_beat_q[2:0] + 3'd1};\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n\n");

    fprintf(fp, "            TOP_START_CORE: begin\n");
    fprintf(fp, "                core_wr_valid_c = 1'b1;\n");
    fprintf(fp, "                core_wr_kind_c  = 2'd2;\n");
    fprintf(fp, "                core_wr_idx_c   = 3'd1;\n");
    fprintf(fp, "                core_wr_data_c  = {{32{u_ext_hold32_q[31]}}, $unsigned(u_ext_hold32_q)};\n");
    fprintf(fp, "            end\n");

    /* On accept, `TOP_SWAP_CHK` immediately reads `{SHADOW_SLOT, 3'd0}` (0 SRAM writes!) */
    fprintf(fp, "            TOP_SWAP_CHK: begin\n");
    fprintf(fp, "                if (swap_fw_ok_c) begin\n");
    fprintf(fp, "                    sram_ce_c   = 1'b1;\n");
    fprintf(fp, "                    sram_we_c   = 1'b0;\n");
    fprintf(fp, "                    sram_addr_c = {SHADOW_SLOT, 3'd0};\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    /* Shadow-to-Slot Ping-Pong Commit (`sub_beat_q` 0..12, `sram_rdata_w` holds read word):
     *   even 0..10 : write `{1'b0, swap_slot_q, w}` from `sram_rdata_w` (masked by `cand_staged_mask_q`)
     *   odd  1..9  : read  `{SHADOW_SLOT, w+1}` into `sram_rdata_w`
     *   11         : write `{1'b0, swap_slot_q, 3'd7} <= 64'd0`
     *   >= 12      : release-ordered final commit `{1'b0, swap_slot_q, 3'd6} <= {swap_epoch_q, 32'd0}` */
    fprintf(fp, "            TOP_SWAP_COMMIT: begin\n");
    fprintf(fp, "                sram_ce_c = 1'b1;\n");
    fprintf(fp, "                if (sub_beat_q < 4'd11) begin\n");
    fprintf(fp, "                    if (!sub_beat_q[0]) begin\n");
    fprintf(fp, "                        sram_we_c    = 1'b1;\n");
    fprintf(fp, "                        sram_addr_c  = {1'b0, swap_slot_q, sub_beat_q[3:1]};\n");
    fprintf(fp, "                        sram_wdata_c = cand_staged_pad_c[sub_beat_q[3:1]] ? sram_rdata_w : 64'd0;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        sram_we_c   = 1'b0;\n");
    fprintf(fp, "                        sram_addr_c = {SHADOW_SLOT, sub_beat_q[3:1] + 3'd1};\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end else if (sub_beat_q == 4'd11) begin\n");
    fprintf(fp, "                    sram_we_c    = 1'b1;\n");
    fprintf(fp, "                    sram_addr_c  = {1'b0, swap_slot_q, 3'd7};\n");
    fprintf(fp, "                    sram_wdata_c = 64'd0;\n");
    fprintf(fp, "                end else begin\n");
    fprintf(fp, "                    sram_we_c    = 1'b1;\n");
    fprintf(fp, "                    sram_addr_c  = {1'b0, swap_slot_q, 3'd6};\n");
    fprintf(fp, "                    sram_wdata_c = {swap_epoch_q, 32'd0};\n");
    fprintf(fp, "                end\n");
    fprintf(fp, "            end\n");

    fprintf(fp, "            default: begin end\n");
    fprintf(fp, "        endcase\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    always_ff @(posedge clk or negedge rst_n) begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            top_st_q            <= TOP_IDLE;\n");
    fprintf(fp, "            sub_beat_q          <= 4'd0;\n");
    fprintf(fp, "            cap_mask_q          <= HW_MASK16;\n");
    fprintf(fp, "            slot_valid_q <= {MAX_CHARTS{1'b0}};\n");
    fprintf(fp, "            act_slot_q          <= {SLOT_W{1'b0}};\n");
    fprintf(fp, "            bound_cache_valid_q <= 1'b0;\n");
    fprintf(fp, "            bound_cache_slot_q  <= {SLOT_W{1'b0}};\n");
    fprintf(fp, "            u_ext_hold32_q      <= 32'sd0;\n");
    fprintf(fp, "            cand_staged_mask_q  <= 6'd0;\n");
    fprintf(fp, "            cand_b_ok_q         <= 1'b1;\n");
    fprintf(fp, "            cand_dt_ok_q        <= 1'b0;\n");
    fprintf(fp, "            cand_next_lo_q      <= 8'd0;\n");
    fprintf(fp, "            cand_next_hi_q      <= 8'd0;\n");
    fprintf(fp, "            swap_slot_q         <= {SLOT_W{1'b0}};\n");
    fprintf(fp, "            swap_epoch_q        <= 32'd0;\n");
    fprintf(fp, "            step_done_q         <= 1'b0;\n");
    fprintf(fp, "            swap_done_q         <= 1'b0;\n");
    fprintf(fp, "            swap_acc_q          <= 1'b0;\n");
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
    fprintf(fp, "                                    cand_staged_mask_q[cmd_word_i] <= 1'b1;\n");
    fprintf(fp, "                                    if (cmd_word_i == 3'd3) begin\n");
    fprintf(fp, "                                        cand_b_ok_q <= ($signed(cmd_data_i[63:32]) <= $signed(cmd_data_i[31:0]));\n");
    fprintf(fp, "                                    end else if (cmd_word_i == 3'd4) begin\n");
    fprintf(fp, "                                        cand_dt_ok_q <= ($signed(cmd_data_i[63:32]) > 32'sd0);\n");
    fprintf(fp, "                                    end else if (cmd_word_i == 3'd5) begin\n");
    fprintf(fp, "                                        cand_next_lo_q <= cmd_data_i[31:24];\n");
    fprintf(fp, "                                        cand_next_hi_q <= cmd_data_i[23:16];\n");
    fprintf(fp, "                                    end\n");
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_WRITE_CHART: begin\n");
    fprintf(fp, "                                if (wr_chart_slot_c == bound_cache_slot_q) begin\n");
    fprintf(fp, "                                    bound_cache_valid_q <= 1'b0;\n");
    fprintf(fp, "                                end\n");
    fprintf(fp, "                                if (cmd_word_i == 3'd7) begin\n");
    fprintf(fp, "                                    slot_valid_q[wr_chart_slot_c] <= 1'b1;\n");
    fprintf(fp, "                                end\n");
    fprintf(fp, "                            end\n");
    fprintf(fp, "                            CMD_STEP: begin\n");
    fprintf(fp, "                                act_slot_q     <= step_slot_c;\n");
    fprintf(fp, "                                u_ext_hold32_q <= clamp_top64_to_32(cmd_data_i);\n");
    fprintf(fp, "                                sub_beat_q     <= 4'd0;\n");
    fprintf(fp, "                                top_st_q       <= step_cache_hit_c ? TOP_START_CORE : TOP_FETCH_BOUND;\n");
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
    fprintf(fp, "                    if (sub_beat_q >= 4'd5) begin\n");
    fprintf(fp, "                        bound_cache_valid_q <= 1'b1;\n");
    fprintf(fp, "                        bound_cache_slot_q  <= act_slot_q;\n");
    fprintf(fp, "                        top_st_q            <= TOP_START_CORE;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        sub_beat_q <= sub_beat_q + 4'd1;\n");
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
    fprintf(fp, "                        sub_beat_q <= 4'd0;\n");
    fprintf(fp, "                        top_st_q   <= TOP_SWAP_COMMIT;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                TOP_SWAP_COMMIT: begin\n");
    fprintf(fp, "                    if (sub_beat_q >= 4'd12) begin\n");
    fprintf(fp, "                        if (swap_slot_q == bound_cache_slot_q) begin\n");
    fprintf(fp, "                            bound_cache_valid_q <= 1'b0;\n");
    fprintf(fp, "                        end\n");
    fprintf(fp, "                        slot_valid_q[swap_slot_q] <= 1'b1;\n");
    fprintf(fp, "                        swap_done_q               <= 1'b1;\n");
    fprintf(fp, "                        swap_acc_q                <= 1'b1;\n");
    fprintf(fp, "                        top_st_q                  <= TOP_IDLE;\n");
    fprintf(fp, "                    end else begin\n");
    fprintf(fp, "                        sub_beat_q <= sub_beat_q + 4'd1;\n");
    fprintf(fp, "                    end\n");
    fprintf(fp, "                end\n");

    fprintf(fp, "                default: top_st_q <= TOP_IDLE;\n");
    fprintf(fp, "            endcase\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "    end\n\n");

    fprintf(fp, "    wire unused_mode_hold_ok = &{1'b0, CMD_NOP, cand_staged_pad_c[7:6]};\n");
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
    fprintf(fp, "    logic [5:0]  f_past_mask_q;\n");
    fprintf(fp, "    logic        f_past_b_ok_q;\n");
    fprintf(fp, "    logic        f_past_dt_ok_q;\n");
    fprintf(fp, "    logic [7:0]  f_past_nlo_q;\n");
    fprintf(fp, "    logic [7:0]  f_past_nhi_q;\n");
    fprintf(fp, "    initial f_past_valid_q = 1'b0;\n");
    fprintf(fp, "    always @(posedge clk) begin\n");
    fprintf(fp, "        f_past_valid_q <= 1'b1;\n");
    fprintf(fp, "        f_past_rst_n_q <= rst_n;\n");
    fprintf(fp, "        f_past_ready_q <= ready_o;\n");
    fprintf(fp, "        f_past_st_q    <= top_st_q;\n");
    fprintf(fp, "        f_past_fw_ok_q <= swap_fw_ok_c;\n");
    fprintf(fp, "        f_past_cap_q   <= cap_mask_q;\n");
    fprintf(fp, "        f_past_mask_q  <= cand_staged_mask_q;\n");
    fprintf(fp, "        f_past_b_ok_q  <= cand_b_ok_q;\n");
    fprintf(fp, "        f_past_dt_ok_q <= cand_dt_ok_q;\n");
    fprintf(fp, "        f_past_nlo_q   <= cand_next_lo_q;\n");
    fprintf(fp, "        f_past_nhi_q   <= cand_next_hi_q;\n");
    fprintf(fp, "    end\n");
    fprintf(fp, "    always_comb begin\n");
    fprintf(fp, "        if (!rst_n) begin\n");
    fprintf(fp, "            assert (top_st_q == TOP_IDLE);\n");
    fprintf(fp, "            assert (!step_done_o && !swap_done_o && !swap_accepted_o);\n");
    fprintf(fp, "            assert (cap_mask_q == HW_MASK16);\n");
    fprintf(fp, "            assert (slot_valid_q == {MAX_CHARTS{1'b0}});\n");
    fprintf(fp, "            assert (!bound_cache_valid_q);\n");
    fprintf(fp, "            assert (cand_staged_mask_q == 6'd0 && cand_b_ok_q && !cand_dt_ok_q);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        assert ((cap_mask_q & (~HW_MASK16)) == 16'd0);\n");
    fprintf(fp, "        if ((top_st_q == TOP_IDLE) && (!core_ready_w)) begin\n");
    fprintf(fp, "            assert (!ready_o && !sram_ce_c && !sram_we_c && !core_wr_valid_c);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_IDLE) && cmd_valid_i && core_ready_w && (cmd_op_i == CMD_STEP) && step_cache_hit_c) begin\n");
    fprintf(fp, "            assert (!sram_ce_c && !sram_we_c);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_IDLE) && cmd_valid_i && core_ready_w && (cmd_op_i == CMD_STAGE_CAND) && (cmd_word_i < 3'd6)) begin\n");
    fprintf(fp, "            assert (sram_ce_c && sram_we_c && (sram_addr_c == {SHADOW_SLOT, cmd_word_i}));\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_FETCH_BOUND) || (top_st_q == TOP_START_CORE) || (top_st_q == TOP_WAIT_CORE)) begin\n");
    fprintf(fp, "            assert (sram_we_c == 1'b0);\n");
    fprintf(fp, "            assert (!ready_o);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if (top_st_q == TOP_SWAP_CHK) begin\n");
    fprintf(fp, "            assert (sram_we_c == 1'b0);\n");
    fprintf(fp, "            if (!swap_fw_ok_c) assert (sram_ce_c == 1'b0);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if ((top_st_q == TOP_SWAP_COMMIT) && sram_we_c) begin\n");
    fprintf(fp, "            assert (sram_addr_c[SRAM_AW-1] == 1'b0);\n");
    fprintf(fp, "        end\n");
    fprintf(fp, "        if (f_past_valid_q && f_past_rst_n_q && rst_n) begin\n");
    fprintf(fp, "            if (!f_past_ready_q) begin\n");
    fprintf(fp, "                assert (cap_mask_q == f_past_cap_q);\n");
    fprintf(fp, "                assert (cand_staged_mask_q == f_past_mask_q);\n");
    fprintf(fp, "                assert (cand_b_ok_q == f_past_b_ok_q && cand_dt_ok_q == f_past_dt_ok_q);\n");
    fprintf(fp, "                assert (cand_next_lo_q == f_past_nlo_q && cand_next_hi_q == f_past_nhi_q);\n");
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

static ele_fab_ws_t   g_ele_fab_ws;
static uint8_t        g_ele_fab_ref_buf[ELE_FAB_MAX_GDS_BYTES];
static uint8_t        g_ele_fab_disk_buf[ELE_FAB_MAX_GDS_BYTES];
static flow_ir_prog_t g_ele_fir_prog;
static uint8_t        g_ele_fir_raw[FLOW_IR_MAX_FILE_BYTES];

static void emit_sv_resized_rhs(FILE *fp, uint32_t vid, uint16_t src_w, uint16_t dst_w) {
    if (src_w == dst_w) {
        fprintf(fp, "w_%u", (unsigned)vid);
    } else if (src_w > dst_w) {
        fprintf(fp, "w_%u[%u:0]", (unsigned)vid, (unsigned)(dst_w - 1U));
    } else {
        fprintf(fp, "{{%u{1'b0}}, w_%u}", (unsigned)(dst_w - src_w), (unsigned)vid);
    }
}

static void emit_sv_gf2m_funcs(FILE *fp, uint32_t tag, uint16_t deg, uint64_t poly, bool emit_inv) {
    uint64_t poly_low = poly & flow_ir_mask_u64(deg);
    fprintf(fp,
            "    function automatic logic [%u:0] gf2m_mul_fn_%u(\n"
            "        input logic [%u:0] fa,\n"
            "        input logic [%u:0] fb\n"
            "    );\n"
            "        logic [%u:0] res;\n"
            "        logic [%u:0] cur;\n"
            "        integer k;\n"
            "        begin\n"
            "            res = %u'h0;\n"
            "            cur = fa;\n"
            "            for (k = 0; k < %u; k = k + 1) begin\n"
            "                if (fb[k]) res = res ^ cur;\n",
            (unsigned)(deg - 1U), (unsigned)tag, (unsigned)(deg - 1U), (unsigned)(deg - 1U),
            (unsigned)(deg - 1U), (unsigned)(deg - 1U), (unsigned)deg, (unsigned)deg);
    if (deg == 1U) {
        fprintf(fp, "                cur = %u'h0;\n", (unsigned)deg);
    } else {
        fprintf(fp,
                "                if (cur[%u]) cur = {cur[%u:0], 1'b0} ^ %u'h%" PRIx64 ";\n"
                "                else          cur = {cur[%u:0], 1'b0};\n",
                (unsigned)(deg - 1U), (unsigned)(deg - 2U), (unsigned)deg, poly_low,
                (unsigned)(deg - 2U));
    }
    fprintf(fp,
            "            end\n"
            "            gf2m_mul_fn_%u = res;\n"
            "        end\n"
            "    endfunction\n",
            (unsigned)tag);

    if (emit_inv) {
        uint64_t exp = flow_ir_mask_u64(deg) - UINT64_C(1);
        fprintf(fp,
                "    function automatic logic [%u:0] gf2m_inv_fn_%u(input logic [%u:0] fx);\n"
                "        logic [%u:0] res;\n"
                "        logic [%u:0] base;\n"
                "        begin\n"
                "            if (fx == %u'h0) begin\n"
                "                gf2m_inv_fn_%u = %u'h0;\n"
                "            end else begin\n"
                "                res  = %u'h1;\n"
                "                base = fx;\n",
                (unsigned)(deg - 1U), (unsigned)tag, (unsigned)(deg - 1U), (unsigned)(deg - 1U),
                (unsigned)(deg - 1U), (unsigned)deg, (unsigned)tag, (unsigned)deg, (unsigned)deg);
        for (uint16_t b = 0U; b < deg; ++b) {
            if (((exp >> b) & UINT64_C(1)) != UINT64_C(0)) {
                fprintf(fp, "                res  = gf2m_mul_fn_%u(res, base);\n", (unsigned)tag);
            }
            if (b + 1U < deg) {
                fprintf(fp, "                base = gf2m_mul_fn_%u(base, base);\n", (unsigned)tag);
            }
        }
        fprintf(fp,
                "                gf2m_inv_fn_%u = res;\n"
                "            end\n"
                "        end\n"
                "    endfunction\n",
                (unsigned)tag);
    }
}

static void emit_fir_sv(FILE *fp, const flow_ir_prog_t *p) {
    fprintf(fp, "// SPDX-License-Identifier: MIT\n");
    fprintf(fp, "// AUTO-GENERATED BY `tools/ele_bootstrap.c --fir-synth` FROM Unified `.fir` (`%s`)\n",
            p->name);
    fprintf(fp, "`default_nettype none\n");
    fprintf(fp, "/* verilator lint_off DECLFILENAME */\n\n");
    fprintf(fp, "module %s (\n", p->name);

    uint32_t total_ports = p->num_inputs + p->num_outputs;
    uint32_t port_idx    = 0U;
    for (uint32_t i = 0U; i < p->num_inputs; ++i) {
        uint32_t vid = p->inputs[i];
        uint16_t w   = p->vals[vid].width;
        ++port_idx;
        fprintf(fp, "    input  logic [%u:0] %s%s\n", (unsigned)(w - 1U), p->vals[vid].name,
                (port_idx < total_ports) ? "," : "");
    }
    for (uint32_t i = 0U; i < p->num_outputs; ++i) {
        uint32_t vid = p->outputs[i];
        uint16_t w   = p->vals[vid].width;
        ++port_idx;
        fprintf(fp, "    output logic [%u:0] %s%s\n", (unsigned)(w - 1U), p->vals[vid].name,
                (port_idx < total_ports) ? "," : "");
    }
    fprintf(fp, ");\n");
    fprintf(fp, "/* verilator lint_off UNUSEDSIGNAL */\n");

    for (uint32_t i = 0U; i < p->num_vals; ++i) {
        fprintf(fp, "    logic [%u:0] w_%u;\n", (unsigned)(p->vals[i].width - 1U), (unsigned)i);
    }
    for (uint32_t i = 0U; i < p->num_inputs; ++i) {
        uint32_t vid = p->inputs[i];
        fprintf(fp, "    assign w_%u = %s;\n", (unsigned)vid, p->vals[vid].name);
    }

    for (uint32_t i = 0U; i < p->num_ops; ++i) {
        const flow_ir_op_t *op = &p->ops[i];
        uint16_t k = op->kind;
        if (k == FLOW_IR_OP_GF2M_MUL || k == FLOW_IR_OP_GF2M_INV) {
            emit_sv_gf2m_funcs(fp, i, (uint16_t)op->p0, op->p1, (k == FLOW_IR_OP_GF2M_INV));
        }
    }

    for (uint32_t i = 0U; i < p->num_ops; ++i) {
        const flow_ir_op_t *op = &p->ops[i];
        uint32_t oid           = op->out_val;
        uint16_t ow            = p->vals[oid].width;
        uint16_t k             = op->kind;

        if (k == FLOW_IR_OP_CONST_U64) {
            uint64_t mv = op->p0 & flow_ir_mask_u64(ow);
            fprintf(fp, "    assign w_%u = %u'h%" PRIx64 ";\n", (unsigned)oid, (unsigned)ow, mv);
        } else if (k == FLOW_IR_OP_SLICE) {
            uint32_t iv = op->ins[0];
            uint16_t iw = p->vals[iv].width;
            uint16_t lo = (uint16_t)op->p0;
            if (lo + ow <= iw) {
                fprintf(fp, "    assign w_%u = w_%u[%u:%u];\n", (unsigned)oid, (unsigned)iv,
                        (unsigned)(lo + ow - 1U), (unsigned)lo);
            } else {
                fprintf(fp, "    assign w_%u = ", (unsigned)oid);
                emit_sv_resized_rhs(fp, iv, iw, ow);
                fprintf(fp, ";\n");
            }
        } else if (k == FLOW_IR_OP_CONCAT) {
            fprintf(fp, "    assign w_%u = {", (unsigned)oid);
            for (int32_t j = (int32_t)op->num_ins - 1; j >= 0; --j) {
                fprintf(fp, "w_%u%s", (unsigned)op->ins[j], (j > 0) ? ", " : "");
            }
            fprintf(fp, "};\n");
        } else if (k == FLOW_IR_OP_XOR_VEC || k == FLOW_IR_OP_AND_VEC || k == FLOW_IR_OP_ADD ||
                   k == FLOW_IR_OP_SUB) {
            const char *sym = (k == FLOW_IR_OP_XOR_VEC) ? "^"
                              : (k == FLOW_IR_OP_AND_VEC) ? "&"
                              : (k == FLOW_IR_OP_ADD)     ? "+"
                                                          : "-";
            uint32_t a = op->ins[0], b = op->ins[1];
            fprintf(fp, "    assign w_%u = ", (unsigned)oid);
            emit_sv_resized_rhs(fp, a, p->vals[a].width, ow);
            fprintf(fp, " %s ", sym);
            emit_sv_resized_rhs(fp, b, p->vals[b].width, ow);
            fprintf(fp, ";\n");
        } else if (k == FLOW_IR_OP_NOT_VEC) {
            uint32_t a = op->ins[0];
            fprintf(fp, "    assign w_%u = ~(", (unsigned)oid);
            emit_sv_resized_rhs(fp, a, p->vals[a].width, ow);
            fprintf(fp, ");\n");
        } else if (k == FLOW_IR_OP_OR_REDUCE) {
            fprintf(fp, "    assign w_%u = |w_%u;\n", (unsigned)oid, (unsigned)op->ins[0]);
        } else if (k == FLOW_IR_OP_MUL_FXP || k == FLOW_IR_OP_SQR_FXP) {
            uint32_t a  = op->ins[0];
            uint32_t b  = (k == FLOW_IR_OP_SQR_FXP) ? op->ins[0] : op->ins[1];
            uint16_t wa = p->vals[a].width;
            uint16_t wb = p->vals[b].width;
            uint16_t sh = (uint16_t)((k == FLOW_IR_OP_SQR_FXP)
                                         ? ((2ULL * op->p0 > op->p2) ? (2ULL * op->p0 - op->p2) : 0ULL)
                                         : ((op->p0 + op->p1 > op->p2) ? (op->p0 + op->p1 - op->p2) : 0ULL));
            uint16_t fw = (uint16_t)((wa + wb > sh + ow) ? (wa + wb) : (sh + ow));
            fprintf(fp, "    wire [%u:0] mul_a_%u = ", (unsigned)(fw - 1U), (unsigned)i);
            emit_sv_resized_rhs(fp, a, wa, fw);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] mul_b_%u = ", (unsigned)(fw - 1U), (unsigned)i);
            emit_sv_resized_rhs(fp, b, wb, fw);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] mul_p_%u = mul_a_%u * mul_b_%u;\n",
                    (unsigned)(fw - 1U), (unsigned)i, (unsigned)i, (unsigned)i);
            fprintf(fp, "    assign w_%u = mul_p_%u[%u:%u];\n", (unsigned)oid, (unsigned)i,
                    (unsigned)(sh + ow - 1U), (unsigned)sh);
        } else if (k == FLOW_IR_OP_GF2M_MUL) {
            uint16_t deg = (uint16_t)op->p0;
            fprintf(fp, "    assign w_%u = gf2m_mul_fn_%u(w_%u[%u:0], w_%u[%u:0]);\n",
                    (unsigned)oid, (unsigned)i, (unsigned)op->ins[0], (unsigned)(deg - 1U),
                    (unsigned)op->ins[1], (unsigned)(deg - 1U));
        } else if (k == FLOW_IR_OP_GF2M_INV) {
            uint16_t deg = (uint16_t)op->p0;
            fprintf(fp, "    assign w_%u = gf2m_inv_fn_%u(w_%u[%u:0]);\n",
                    (unsigned)oid, (unsigned)i, (unsigned)op->ins[0], (unsigned)(deg - 1U));
        } else if (k == FLOW_IR_OP_CTRL_SEL) {
            uint32_t sb = op->ins[0], vf = op->ins[1], vt = op->ins[2];
            fprintf(fp, "    assign w_%u = w_%u[0] ? ", (unsigned)oid, (unsigned)sb);
            emit_sv_resized_rhs(fp, vt, p->vals[vt].width, ow);
            fprintf(fp, " : ");
            emit_sv_resized_rhs(fp, vf, p->vals[vf].width, ow);
            fprintf(fp, ";\n");
        } else if (k == FLOW_IR_OP_SHL_VAR || k == FLOW_IR_OP_SHR_VAR) {
            const char *op_s = (k == FLOW_IR_OP_SHL_VAR) ? "<<" : ">>";
            uint32_t a = op->ins[0], shv = op->ins[1];
            fprintf(fp, "    assign w_%u = ", (unsigned)oid);
            emit_sv_resized_rhs(fp, a, p->vals[a].width, ow);
            fprintf(fp, " %s w_%u;\n", op_s, (unsigned)shv);
        } else if (k == FLOW_IR_OP_EQ_ZERO) {
            uint32_t a = op->ins[0];
            fprintf(fp, "    assign w_%u = (w_%u == %u'h0) ? 1'b1 : 1'b0;\n",
                    (unsigned)oid, (unsigned)a, (unsigned)p->vals[a].width);
        } else if (k == FLOW_IR_OP_LUT_ROM) {
            uint32_t a = op->ins[0];
            fprintf(fp, "    always_comb begin\n        case (w_%u)\n", (unsigned)a);
            for (uint32_t l = 0U; l < op->lut_len; ++l) {
                uint64_t lv = p->lut_pool[op->lut_off + l] & flow_ir_mask_u64(ow);
                fprintf(fp, "            %u'h%x: w_%u = %u'h%" PRIx64 ";\n",
                        (unsigned)p->vals[a].width, (unsigned)l, (unsigned)oid, (unsigned)ow, lv);
            }
            fprintf(fp, "            default: w_%u = %u'h0;\n        endcase\n    end\n",
                    (unsigned)oid, (unsigned)ow);
        }
    }

    for (uint32_t i = 0U; i < p->num_outputs; ++i) {
        uint32_t vid = p->outputs[i];
        fprintf(fp, "    assign %s = w_%u;\n", p->vals[vid].name, (unsigned)vid);
    }
    fprintf(fp, "/* verilator lint_on UNUSEDSIGNAL */\n");
    fprintf(fp, "endmodule\n");
    fprintf(fp, "/* verilator lint_on DECLFILENAME */\n");
    fprintf(fp, "`default_nettype wire\n");
}

typedef struct {
    uint32_t val_stage[FLOW_IR_MAX_VALS];
    uint32_t max_emitted_stage[FLOW_IR_MAX_VALS];
    uint32_t op_mask_off[FLOW_IR_MAX_OPS];
    uint32_t total_mask_bits;
    uint32_t total_lat;
} ele_dom_ws_t;

static ele_dom_ws_t g_ele_dom_ws;

static bool dom_op_is_supported(uint16_t k) {
    return (k == FLOW_IR_OP_CONST_U64) || (k == FLOW_IR_OP_SLICE) || (k == FLOW_IR_OP_CONCAT) ||
           (k == FLOW_IR_OP_XOR_VEC) || (k == FLOW_IR_OP_NOT_VEC) || (k == FLOW_IR_OP_AND_VEC) ||
           (k == FLOW_IR_OP_CTRL_SEL);
}

static bool dom_val_is_const(const flow_ir_prog_t *p, uint32_t vid) {
    if (vid < p->num_inputs) {
        return false;
    }
    uint32_t oi = vid - p->num_inputs;
    return (oi < p->num_ops) && (p->ops[oi].kind == FLOW_IR_OP_CONST_U64);
}

static void emit_dom_sv_resized_rhs(FILE *fp, const char *dom_tag, uint32_t vid, uint32_t st,
                                    uint16_t src_w, uint16_t dst_w) {
    if (src_w == dst_w) {
        fprintf(fp, "w_%s_%u_s%u", dom_tag, (unsigned)vid, (unsigned)st);
    } else if (src_w > dst_w) {
        fprintf(fp, "w_%s_%u_s%u[%u:0]", dom_tag, (unsigned)vid, (unsigned)st,
                (unsigned)(dst_w - 1U));
    } else {
        fprintf(fp, "{{%u{1'b0}}, w_%s_%u_s%u}", (unsigned)(dst_w - src_w), dom_tag,
                (unsigned)vid, (unsigned)st);
    }
}

static void ensure_dom_stage(FILE *fp, const flow_ir_prog_t *p, uint32_t vid, uint32_t target_st) {
    uint16_t w = p->vals[vid].width;
    while (g_ele_dom_ws.max_emitted_stage[vid] < target_st) {
        uint32_t s_prev = g_ele_dom_ws.max_emitted_stage[vid];
        uint32_t s_next = s_prev + 1U;
        if (dom_val_is_const(p, vid)) {
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = w_dom0_%u_s%u;\n", (unsigned)(w - 1U),
                    (unsigned)vid, (unsigned)s_next, (unsigned)vid, (unsigned)s_prev);
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = w_dom1_%u_s%u;\n", (unsigned)(w - 1U),
                    (unsigned)vid, (unsigned)s_next, (unsigned)vid, (unsigned)s_prev);
        } else {
            fprintf(fp,
                    "    logic [%u:0] dly_dom0_%u_s%u;\n"
                    "    always_ff @(posedge clk_i or negedge rst_ni) begin\n"
                    "        if (!rst_ni) dly_dom0_%u_s%u <= '0;\n"
                    "        else         dly_dom0_%u_s%u <= w_dom0_%u_s%u;\n"
                    "    end\n"
                    "    wire [%u:0] w_dom0_%u_s%u = dly_dom0_%u_s%u;\n",
                    (unsigned)(w - 1U), (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next, (unsigned)vid, (unsigned)s_prev,
                    (unsigned)(w - 1U), (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next);
            fprintf(fp,
                    "    logic [%u:0] dly_dom1_%u_s%u;\n"
                    "    always_ff @(posedge clk_i or negedge rst_ni) begin\n"
                    "        if (!rst_ni) dly_dom1_%u_s%u <= '0;\n"
                    "        else         dly_dom1_%u_s%u <= w_dom1_%u_s%u;\n"
                    "    end\n"
                    "    wire [%u:0] w_dom1_%u_s%u = dly_dom1_%u_s%u;\n",
                    (unsigned)(w - 1U), (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next, (unsigned)vid, (unsigned)s_prev,
                    (unsigned)(w - 1U), (unsigned)vid, (unsigned)s_next,
                    (unsigned)vid, (unsigned)s_next);
        }
        g_ele_dom_ws.max_emitted_stage[vid] = s_next;
    }
}

static bool emit_fir_dom_sv(FILE *fp, const flow_ir_prog_t *p) {
    memset(&g_ele_dom_ws, 0, sizeof(g_ele_dom_ws));

    for (uint32_t i = 0U; i < p->num_ops; ++i) {
        const flow_ir_op_t *op = &p->ops[i];
        if (!dom_op_is_supported(op->kind)) {
            fprintf(stderr,
                    "FATAL: --dom-mask only supports factored F2/AND/SEL graphs (found kind=%u in %s)\n",
                    (unsigned)op->kind, p->name);
            return false;
        }
        uint32_t oid       = op->out_val;
        uint32_t max_in_st = 0U;
        for (uint16_t j = 0U; j < op->num_ins && j < FLOW_IR_MAX_INS; ++j) {
            uint32_t iv = op->ins[j];
            if (g_ele_dom_ws.val_stage[iv] > max_in_st) {
                max_in_st = g_ele_dom_ws.val_stage[iv];
            }
        }
        if (op->kind == FLOW_IR_OP_AND_VEC || op->kind == FLOW_IR_OP_CTRL_SEL) {
            g_ele_dom_ws.val_stage[oid]   = max_in_st + 2U;
            g_ele_dom_ws.op_mask_off[i]   = g_ele_dom_ws.total_mask_bits;
            g_ele_dom_ws.total_mask_bits += (uint32_t)p->vals[oid].width;
        } else {
            g_ele_dom_ws.val_stage[oid] = max_in_st;
        }
    }

    uint32_t total_lat = 0U;
    for (uint32_t i = 0U; i < p->num_outputs; ++i) {
        uint32_t vid = p->outputs[i];
        if (g_ele_dom_ws.val_stage[vid] > total_lat) {
            total_lat = g_ele_dom_ws.val_stage[vid];
        }
    }
    g_ele_dom_ws.total_lat = total_lat;
    uint32_t mask_hi = (g_ele_dom_ws.total_mask_bits > 0U) ? (g_ele_dom_ws.total_mask_bits - 1U) : 0U;

    fprintf(fp, "// SPDX-License-Identifier: MIT\n");
    fprintf(fp, "// AUTO-GENERATED BY `tools/ele_bootstrap.c --fir-synth --dom-mask` FROM `%s`\n",
            p->name);
    fprintf(fp, "`default_nettype none\n");
    fprintf(fp, "/* verilator lint_off DECLFILENAME */\n\n");

    fprintf(fp,
            "module ele_dom_and_stage #(\n"
            "    parameter int unsigned W = 1\n"
            ") (\n"
            "    input  logic         clk_i,\n"
            "    input  logic         rst_ni,\n"
            "    input  logic [W-1:0] a0_i,\n"
            "    input  logic [W-1:0] a1_i,\n"
            "    input  logic [W-1:0] b0_i,\n"
            "    input  logic [W-1:0] b1_i,\n"
            "    input  logic [W-1:0] r_ref_i,\n"
            "    input  logic [W-1:0] z_blind_i,\n"
            "    output logic [W-1:0] q0_o,\n"
            "    output logic [W-1:0] q1_o\n"
            ");\n"
            "    logic [W-1:0] a0_q, a1_q, hb0_q, hb1_q;\n"
            "    always_ff @(posedge clk_i or negedge rst_ni) begin\n"
            "        if (!rst_ni) begin\n"
            "            a0_q  <= '0;\n"
            "            a1_q  <= '0;\n"
            "            hb0_q <= '0;\n"
            "            hb1_q <= '0;\n"
            "        end else begin\n"
            "            a0_q  <= a0_i;\n"
            "            a1_q  <= a1_i;\n"
            "            hb0_q <= b0_i ^ r_ref_i;\n"
            "            hb1_q <= b1_i ^ r_ref_i;\n"
            "        end\n"
            "    end\n"
            "    wire [W-1:0] p00 = a0_q & hb0_q;\n"
            "    wire [W-1:0] c01 = (a0_q & hb1_q) ^ z_blind_i;\n"
            "    wire [W-1:0] c10 = (a1_q & hb0_q) ^ z_blind_i;\n"
            "    wire [W-1:0] p11 = a1_q & hb1_q;\n"
            "    logic [W-1:0] p00_q, c01_q, c10_q, p11_q;\n"
            "    always_ff @(posedge clk_i or negedge rst_ni) begin\n"
            "        if (!rst_ni) begin\n"
            "            p00_q <= '0;\n"
            "            c01_q <= '0;\n"
            "            c10_q <= '0;\n"
            "            p11_q <= '0;\n"
            "        end else begin\n"
            "            p00_q <= p00;\n"
            "            c01_q <= c01;\n"
            "            c10_q <= c10;\n"
            "            p11_q <= p11;\n"
            "        end\n"
            "    end\n"
            "    assign q0_o = p00_q ^ c01_q;\n"
            "    assign q1_o = c10_q ^ p11_q;\n"
            "endmodule\n\n");

    fprintf(fp,
            "`ifdef FORMAL\n"
            "module %s_dom_atom_qed (\n"
            "    input logic [7:0] x_a,\n"
            "    input logic [7:0] y_a,\n"
            "    input logic [7:0] x_b,\n"
            "    input logic [7:0] y_b,\n"
            "    input logic [7:0] m_x,\n"
            "    input logic [7:0] m_y,\n"
            "    input logic [7:0] r_ref,\n"
            "    input logic [7:0] z_m\n"
            ");\n"
            "    wire [7:0] a0_A = m_x;\n"
            "    wire [7:0] a1_A = x_a ^ m_x;\n"
            "    wire [7:0] b0_A = m_y ^ m_x;\n"
            "    wire [7:0] b1_A = y_a ^ b0_A;\n"
            "    wire [7:0] hb0_A = b0_A ^ r_ref;\n"
            "    wire [7:0] hb1_A = b1_A ^ r_ref;\n"
            "    wire [7:0] p00_A = a0_A & hb0_A;\n"
            "    wire [7:0] p01_A = a0_A & hb1_A;\n"
            "    wire [7:0] c01_A = p01_A ^ z_m;\n"
            "    wire [7:0] p10_A = a1_A & hb0_A;\n"
            "    wire [7:0] c10_A = p10_A ^ z_m;\n"
            "    wire [7:0] p11_A = a1_A & hb1_A;\n"
            "    wire [7:0] q0_A  = p00_A ^ c01_A;\n"
            "    wire [7:0] q1_A  = c10_A ^ p11_A;\n"
            "    wire [7:0] dy     = y_a ^ y_b;\n"
            "    wire [7:0] zm_B   = z_m ^ (a0_A & dy);\n"
            "    wire [7:0] b0_B   = m_y ^ m_x;\n"
            "    wire [7:0] b1_B   = y_b ^ b0_B;\n"
            "    wire [7:0] hb0_B  = b0_B ^ r_ref;\n"
            "    wire [7:0] hb1_B  = b1_B ^ r_ref;\n"
            "    wire [7:0] p00_B  = a0_A & hb0_B;\n"
            "    wire [7:0] c01_B  = (a0_A & hb1_B) ^ zm_B;\n"
            "    wire [7:0] q0_B   = p00_B ^ c01_B;\n"
            "    wire [7:0] p01_Bp = a0_A & (b1_B ^ (r_ref ^ dy));\n"
            "    wire [7:0] dx     = x_a ^ x_b;\n"
            "    wire [7:0] mx_C   = m_x ^ dx;\n"
            "    wire [7:0] a1_C   = x_b ^ mx_C;\n"
            "    wire [7:0] b0_C   = m_y ^ mx_C;\n"
            "    wire [7:0] b1_C   = y_b ^ b0_C;\n"
            "    wire [7:0] rb_C   = r_ref ^ dx ^ dy;\n"
            "    wire [7:0] hb0_C  = b0_C ^ rb_C;\n"
            "    wire [7:0] hb1_C  = b1_C ^ rb_C;\n"
            "    wire [7:0] zm_C   = z_m ^ (a1_A & (hb0_A ^ hb0_C));\n"
            "    wire [7:0] p11_C  = a1_C & hb1_C;\n"
            "    wire [7:0] c10_C  = (a1_C & hb0_C) ^ zm_C;\n"
            "    wire [7:0] q1_C   = c10_C ^ p11_C;\n"
            "    always_comb begin\n"
            "        assert((q0_A ^ q1_A) == (x_a & y_a));\n"
            "        assert((zm_B ^ (a0_A & dy)) == z_m);\n"
            "        assert(p00_A == p00_B);\n"
            "        assert(c01_A == c01_B);\n"
            "        assert(q0_A  == q0_B);\n"
            "        assert(p01_A == p01_Bp);\n"
            "        assert(a1_C  == a1_A);\n"
            "        assert(p11_A == p11_C);\n"
            "        assert(c10_A == c10_C);\n"
            "        assert(q1_A  == q1_C);\n"
            "    end\n"
            "endmodule\n"
            "`endif\n\n",
            p->name);

    fprintf(fp, "module %s_dom (\n", p->name);
    fprintf(fp, "    input  logic clk_i,\n");
    fprintf(fp, "    input  logic rst_ni,\n");
    for (uint32_t i = 0U; i < p->num_inputs; ++i) {
        uint32_t vid = p->inputs[i];
        uint16_t w   = p->vals[vid].width;
        fprintf(fp, "    input  logic [%u:0] %s_dom0_i,\n", (unsigned)(w - 1U), p->vals[vid].name);
        fprintf(fp, "    input  logic [%u:0] %s_dom1_i,\n", (unsigned)(w - 1U), p->vals[vid].name);
    }
    fprintf(fp, "    input  logic [%u:0] r_ref_i,\n", (unsigned)mask_hi);
    fprintf(fp, "    input  logic [%u:0] z_blind_i,\n", (unsigned)mask_hi);
    for (uint32_t i = 0U; i < p->num_outputs; ++i) {
        uint32_t vid = p->outputs[i];
        uint16_t w   = p->vals[vid].width;
        const char *comma = (i + 1U < p->num_outputs) ? "," : "";
        fprintf(fp, "    output logic [%u:0] %s_dom0_o,\n", (unsigned)(w - 1U), p->vals[vid].name);
        fprintf(fp, "    output logic [%u:0] %s_dom1_o%s\n", (unsigned)(w - 1U), p->vals[vid].name, comma);
    }
    fprintf(fp, ");\n");
    fprintf(fp, "    /* verilator lint_off UNUSEDPARAM */\n");
    fprintf(fp, "    localparam int unsigned DOM_LATENCY_CYC = %u;\n", (unsigned)total_lat);
    fprintf(fp, "    localparam int unsigned DOM_MASK_BITS   = %u;\n",
            (unsigned)g_ele_dom_ws.total_mask_bits);
    fprintf(fp, "    /* verilator lint_on UNUSEDPARAM */\n\n");
    fprintf(fp, "    /* verilator lint_off UNUSEDSIGNAL */\n");

    if (total_lat == 0U || g_ele_dom_ws.total_mask_bits == 0U) {
        fprintf(fp, "    /* verilator lint_off UNUSEDSIGNAL */\n");
        fprintf(fp, "    wire _unused_dom_ctrl = &{1'b0, clk_i, rst_ni, r_ref_i, z_blind_i};\n");
        fprintf(fp, "    /* verilator lint_on UNUSEDSIGNAL */\n");
    }

    for (uint32_t i = 0U; i < p->num_inputs; ++i) {
        uint32_t vid = p->inputs[i];
        uint16_t w   = p->vals[vid].width;
        fprintf(fp, "    wire [%u:0] w_dom0_%u_s0 = %s_dom0_i;\n", (unsigned)(w - 1U),
                (unsigned)vid, p->vals[vid].name);
        fprintf(fp, "    wire [%u:0] w_dom1_%u_s0 = %s_dom1_i;\n", (unsigned)(w - 1U),
                (unsigned)vid, p->vals[vid].name);
        g_ele_dom_ws.max_emitted_stage[vid] = 0U;
    }

    for (uint32_t i = 0U; i < p->num_ops; ++i) {
        const flow_ir_op_t *op = &p->ops[i];
        uint32_t oid           = op->out_val;
        uint16_t ow            = p->vals[oid].width;
        uint16_t k             = op->kind;
        uint32_t st_out        = g_ele_dom_ws.val_stage[oid];
        uint32_t max_in_st     = 0U;
        for (uint16_t j = 0U; j < op->num_ins && j < FLOW_IR_MAX_INS; ++j) {
            uint32_t iv = op->ins[j];
            if (g_ele_dom_ws.val_stage[iv] > max_in_st) {
                max_in_st = g_ele_dom_ws.val_stage[iv];
            }
        }
        for (uint16_t j = 0U; j < op->num_ins && j < FLOW_IR_MAX_INS; ++j) {
            ensure_dom_stage(fp, p, op->ins[j], max_in_st);
        }

        if (k == FLOW_IR_OP_CONST_U64) {
            uint64_t mv = op->p0 & flow_ir_mask_u64(ow);
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = %u'h%" PRIx64 ";\n", (unsigned)(ow - 1U),
                    (unsigned)oid, (unsigned)st_out, (unsigned)ow, mv);
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = %u'h0;\n", (unsigned)(ow - 1U),
                    (unsigned)oid, (unsigned)st_out, (unsigned)ow);
        } else if (k == FLOW_IR_OP_SLICE) {
            uint32_t iv = op->ins[0];
            uint16_t iw = p->vals[iv].width;
            uint16_t lo = (uint16_t)op->p0;
            if (lo + ow <= iw) {
                fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = w_dom0_%u_s%u[%u:%u];\n",
                        (unsigned)(ow - 1U), (unsigned)oid, (unsigned)st_out, (unsigned)iv,
                        (unsigned)max_in_st, (unsigned)(lo + ow - 1U), (unsigned)lo);
                fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = w_dom1_%u_s%u[%u:%u];\n",
                        (unsigned)(ow - 1U), (unsigned)oid, (unsigned)st_out, (unsigned)iv,
                        (unsigned)max_in_st, (unsigned)(lo + ow - 1U), (unsigned)lo);
            } else {
                fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = ", (unsigned)(ow - 1U),
                        (unsigned)oid, (unsigned)st_out);
                emit_dom_sv_resized_rhs(fp, "dom0", iv, max_in_st, iw, ow);
                fprintf(fp, ";\n");
                fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = ", (unsigned)(ow - 1U),
                        (unsigned)oid, (unsigned)st_out);
                emit_dom_sv_resized_rhs(fp, "dom1", iv, max_in_st, iw, ow);
                fprintf(fp, ";\n");
            }
        } else if (k == FLOW_IR_OP_CONCAT) {
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = {", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            for (int32_t j = (int32_t)op->num_ins - 1; j >= 0; --j) {
                fprintf(fp, "w_dom0_%u_s%u%s", (unsigned)op->ins[j], (unsigned)max_in_st,
                        (j > 0) ? ", " : "");
            }
            fprintf(fp, "};\n");
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = {", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            for (int32_t j = (int32_t)op->num_ins - 1; j >= 0; --j) {
                fprintf(fp, "w_dom1_%u_s%u%s", (unsigned)op->ins[j], (unsigned)max_in_st,
                        (j > 0) ? ", " : "");
            }
            fprintf(fp, "};\n");
        } else if (k == FLOW_IR_OP_XOR_VEC) {
            uint32_t a = op->ins[0], b = op->ins[1];
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = ", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            emit_dom_sv_resized_rhs(fp, "dom0", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, " ^ ");
            emit_dom_sv_resized_rhs(fp, "dom0", b, max_in_st, p->vals[b].width, ow);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = ", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            emit_dom_sv_resized_rhs(fp, "dom1", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, " ^ ");
            emit_dom_sv_resized_rhs(fp, "dom1", b, max_in_st, p->vals[b].width, ow);
            fprintf(fp, ";\n");
        } else if (k == FLOW_IR_OP_NOT_VEC) {
            uint32_t a = op->ins[0];
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = ~(", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            emit_dom_sv_resized_rhs(fp, "dom0", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, ");\n");
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = ", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            emit_dom_sv_resized_rhs(fp, "dom1", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, ";\n");
        } else if (k == FLOW_IR_OP_AND_VEC) {
            uint32_t a   = op->ins[0], b = op->ins[1];
            uint32_t mlo = g_ele_dom_ws.op_mask_off[i];
            uint32_t mhi = mlo + (uint32_t)ow - 1U;
            fprintf(fp, "    wire [%u:0] and_a_dom0_%u = ", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom0", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] and_a_dom1_%u = ", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom1", a, max_in_st, p->vals[a].width, ow);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] and_b_dom0_%u = ", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom0", b, max_in_st, p->vals[b].width, ow);
            fprintf(fp, ";\n");
            fprintf(fp, "    wire [%u:0] and_b_dom1_%u = ", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom1", b, max_in_st, p->vals[b].width, ow);
            fprintf(fp, ";\n");
            fprintf(fp, "    logic [%u:0] w_dom0_%u_s%u;\n", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            fprintf(fp, "    logic [%u:0] w_dom1_%u_s%u;\n", (unsigned)(ow - 1U), (unsigned)oid,
                    (unsigned)st_out);
            fprintf(fp,
                    "    ele_dom_and_stage #(.W(%u)) u_dom_%u (\n"
                    "        .clk_i(clk_i), .rst_ni(rst_ni),\n"
                    "        .a0_i(and_a_dom0_%u), .a1_i(and_a_dom1_%u),\n"
                    "        .b0_i(and_b_dom0_%u), .b1_i(and_b_dom1_%u),\n"
                    "        .r_ref_i(r_ref_i[%u:%u]), .z_blind_i(z_blind_i[%u:%u]),\n"
                    "        .q0_o(w_dom0_%u_s%u), .q1_o(w_dom1_%u_s%u)\n"
                    "    );\n",
                    (unsigned)ow, (unsigned)oid, (unsigned)oid, (unsigned)oid, (unsigned)oid,
                    (unsigned)oid, (unsigned)mhi, (unsigned)mlo, (unsigned)mhi, (unsigned)mlo,
                    (unsigned)oid, (unsigned)st_out, (unsigned)oid, (unsigned)st_out);
        } else if (k == FLOW_IR_OP_CTRL_SEL) {
            uint32_t sb = op->ins[0], vf = op->ins[1], vt = op->ins[2];
            uint32_t mlo = g_ele_dom_ws.op_mask_off[i];
            uint32_t mhi = mlo + (uint32_t)ow - 1U;
            if (ow == 1U) {
                fprintf(fp, "    wire [0:0] sel_rep_dom0_%u = w_dom0_%u_s%u[0:0];\n",
                        (unsigned)oid, (unsigned)sb, (unsigned)max_in_st);
                fprintf(fp, "    wire [0:0] sel_rep_dom1_%u = w_dom1_%u_s%u[0:0];\n",
                        (unsigned)oid, (unsigned)sb, (unsigned)max_in_st);
            } else {
                fprintf(fp, "    wire [%u:0] sel_rep_dom0_%u = {%u{w_dom0_%u_s%u[0]}};\n",
                        (unsigned)(ow - 1U), (unsigned)oid, (unsigned)ow, (unsigned)sb,
                        (unsigned)max_in_st);
                fprintf(fp, "    wire [%u:0] sel_rep_dom1_%u = {%u{w_dom1_%u_s%u[0]}};\n",
                        (unsigned)(ow - 1U), (unsigned)oid, (unsigned)ow, (unsigned)sb,
                        (unsigned)max_in_st);
            }
            fprintf(fp, "    wire [%u:0] sel_diff_dom0_%u = (", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom0", vf, max_in_st, p->vals[vf].width, ow);
            fprintf(fp, ") ^ (");
            emit_dom_sv_resized_rhs(fp, "dom0", vt, max_in_st, p->vals[vt].width, ow);
            fprintf(fp, ");\n");
            fprintf(fp, "    wire [%u:0] sel_diff_dom1_%u = (", (unsigned)(ow - 1U), (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom1", vf, max_in_st, p->vals[vf].width, ow);
            fprintf(fp, ") ^ (");
            emit_dom_sv_resized_rhs(fp, "dom1", vt, max_in_st, p->vals[vt].width, ow);
            fprintf(fp, ");\n");
            fprintf(fp, "    logic [%u:0] sel_p_dom0_%u;\n", (unsigned)(ow - 1U), (unsigned)oid);
            fprintf(fp, "    logic [%u:0] sel_p_dom1_%u;\n", (unsigned)(ow - 1U), (unsigned)oid);
            fprintf(fp,
                    "    ele_dom_and_stage #(.W(%u)) u_dom_%u (\n"
                    "        .clk_i(clk_i), .rst_ni(rst_ni),\n"
                    "        .a0_i(sel_rep_dom0_%u), .a1_i(sel_rep_dom1_%u),\n"
                    "        .b0_i(sel_diff_dom0_%u), .b1_i(sel_diff_dom1_%u),\n"
                    "        .r_ref_i(r_ref_i[%u:%u]), .z_blind_i(z_blind_i[%u:%u]),\n"
                    "        .q0_o(sel_p_dom0_%u), .q1_o(sel_p_dom1_%u)\n"
                    "    );\n",
                    (unsigned)ow, (unsigned)oid, (unsigned)oid, (unsigned)oid, (unsigned)oid,
                    (unsigned)oid, (unsigned)mhi, (unsigned)mlo, (unsigned)mhi, (unsigned)mlo,
                    (unsigned)oid, (unsigned)oid);
            ensure_dom_stage(fp, p, vf, st_out);
            fprintf(fp, "    wire [%u:0] w_dom0_%u_s%u = sel_p_dom0_%u ^ (", (unsigned)(ow - 1U),
                    (unsigned)oid, (unsigned)st_out, (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom0", vf, st_out, p->vals[vf].width, ow);
            fprintf(fp, ");\n");
            fprintf(fp, "    wire [%u:0] w_dom1_%u_s%u = sel_p_dom1_%u ^ (", (unsigned)(ow - 1U),
                    (unsigned)oid, (unsigned)st_out, (unsigned)oid);
            emit_dom_sv_resized_rhs(fp, "dom1", vf, st_out, p->vals[vf].width, ow);
            fprintf(fp, ");\n");
        }
        g_ele_dom_ws.max_emitted_stage[oid] = st_out;
    }

    for (uint32_t i = 0U; i < p->num_outputs; ++i) {
        uint32_t vid = p->outputs[i];
        ensure_dom_stage(fp, p, vid, total_lat);
        fprintf(fp, "    assign %s_dom0_o = w_dom0_%u_s%u;\n", p->vals[vid].name, (unsigned)vid,
                (unsigned)total_lat);
        fprintf(fp, "    assign %s_dom1_o = w_dom1_%u_s%u;\n", p->vals[vid].name, (unsigned)vid,
                (unsigned)total_lat);
    }
    fprintf(fp, "    /* verilator lint_on UNUSEDSIGNAL */\n");
    fprintf(fp, "endmodule\n");
    fprintf(fp, "/* verilator lint_on DECLFILENAME */\n");
    fprintf(fp, "`default_nettype wire\n");
    return true;
}

static int run_fir_synth_cmd(const char *fir_path, const char *sv_path,
                             const char *pdk_path, const char *gds_path,
                             bool verify_fir, bool dom_mask) {
    if (!flow_ir_load_file(&g_ele_fir_prog, fir_path, g_ele_fir_raw, sizeof(g_ele_fir_raw))) {
        fprintf(stderr, "FATAL: failed to load .fir morphism %s\n", fir_path);
        return 1;
    }
    if (g_ele_fir_prog.mult != FLOW_IR_MULT_LINEAR) {
        fprintf(stderr, "FATAL: --fir-synth expects .linear morphism (got mult=%u)\n",
                (unsigned)g_ele_fir_prog.mult);
        return 1;
    }

    flow_ir_dag_stats_t dag_st;
    if (!flow_ir_analyze_dag(&g_ele_fir_prog, &dag_st)) {
        fprintf(stderr, "FATAL: flow_ir_analyze_dag failed on %s\n", fir_path);
        return 1;
    }
    printf("  [FLOW-FIR-DAG] mod=%s | ops=%u (affine=%u, non_affine=%u) mult_bits=%u mult_depth=%u\n",
           g_ele_fir_prog.name, (unsigned)g_ele_fir_prog.num_ops,
           (unsigned)dag_st.affine_ops, (unsigned)dag_st.non_affine_ops,
           (unsigned)dag_st.mult_bits, (unsigned)dag_st.mult_depth);

    FILE *fp = fopen(sv_path, "w");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    if (dom_mask) {
        if (!emit_fir_dom_sv(fp, &g_ele_fir_prog)) {
            fclose(fp);
            return 1;
        }
    } else {
        emit_fir_sv(fp, &g_ele_fir_prog);
    }
    fclose(fp);

    if (verify_fir && g_ele_fir_prog.num_inputs <= FLOW_IR_MAX_IO &&
        g_ele_fir_prog.num_outputs <= FLOW_IR_MAX_IO) {
        uint64_t in_v[FLOW_IR_MAX_IO];
        uint64_t out_v[FLOW_IR_MAX_IO];
        uint64_t chk = 0ULL;
        for (uint32_t v = 0U; v < 256U; ++v) {
            for (uint32_t k = 0U; k < g_ele_fir_prog.num_inputs; ++k) {
                uint64_t x = (uint64_t)((v + k * 37U) ^ (v >> 1U));
                in_v[k]    = x & flow_ir_mask_u64(g_ele_fir_prog.vals[g_ele_fir_prog.inputs[k]].width);
            }
            if (!flow_ir_eval_u64(&g_ele_fir_prog, in_v, out_v)) {
                fprintf(stderr, "FATAL: flow_ir_eval_u64 failed on %s\n", fir_path);
                return 1;
            }
            chk = (chk * UINT64_C(1315423911)) ^ out_v[0];
        }
        printf("  [FLOW-FIR-EVAL] mod=%s | 256-vec GPR eval PASS (fnv=0x%016" PRIX64 ")\n",
               g_ele_fir_prog.name, chk);
    }

    if (pdk_path != NULL && gds_path != NULL) {
        ele_fab_pdk_t pdk;
        ele_fab_fiber_t fiber = ele_fab_load_pdk(pdk_path, &pdk);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fir-synth failed to load PDK %s (%s, code=%d)\n",
                    pdk_path, ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_compile_fir_die(&g_ele_fab_ws, &pdk, &g_ele_fir_prog);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fir-synth compile_fir_die failed (%s, code=%d)\n",
                    ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_apply_opc_2jet(&g_ele_fab_ws);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fir-synth OPC failed (%s, code=%d)\n",
                    ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_write_gds(&g_ele_fab_ws, gds_path);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fir-synth write_gds failed (%s, code=%d)\n",
                    ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_verify_gds_bytes(&g_ele_fab_ws, g_ele_fab_ws.gds_buf, g_ele_fab_ws.gds_len);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN ||
            g_ele_fab_ws.last_fiber != ELE_FIBER_SIGNOFF_CLEAN ||
            g_ele_fab_ws.fiber_witness_mask != UINT64_C(0)) {
            fprintf(stderr, "FATAL: --fir-synth DRC/LVS failed on %s (%s, code=%d, mask=0x%" PRIX64 ")\n",
                    gds_path, ele_fab_fiber_name(fiber), (int)fiber, g_ele_fab_ws.fiber_witness_mask);
            return 1;
        }
        printf("  [FLOW-FIR-FAB] mod=%s pdk=%-14s | wire=%3dum ir=%2dmV OPC=%2d->%2dnm | gds=%s (%uB, DRC=0, LVS=0)\n",
               g_ele_fir_prog.name, pdk.name, g_ele_fab_ws.total_wire_um,
               g_ele_fab_ws.max_ir_drop_mv, g_ele_fab_ws.epe_naive_max_nm,
               g_ele_fab_ws.epe_opc_max_nm, gds_path, g_ele_fab_ws.gds_len);
    }
    return 0;
}

static int run_single_fab_proj(const char *pdk_path, const char *gds_path) {
    ele_fab_pdk_t pdk;
    ele_fab_fiber_t fiber = ele_fab_load_pdk(pdk_path, &pdk);
    if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
        fprintf(stderr, "FATAL: failed to load PDK .flow atlas %s (%s, code=%d)\n",
                pdk_path, ele_fab_fiber_name(fiber), (int)fiber);
        return 1;
    }
    fiber = ele_fab_compile_die(&g_ele_fab_ws, &pdk, true);
    if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
        fprintf(stderr, "FATAL: compile_die failed (%s, code=%d)\n",
                ele_fab_fiber_name(fiber), (int)fiber);
        return 1;
    }
    fiber = ele_fab_apply_opc_2jet(&g_ele_fab_ws);
    if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
        fprintf(stderr, "FATAL: apply_opc_2jet failed (%s, code=%d)\n",
                ele_fab_fiber_name(fiber), (int)fiber);
        return 1;
    }
    fiber = ele_fab_write_gds(&g_ele_fab_ws, gds_path);
    if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
        fprintf(stderr, "FATAL: failed to write GDSII stream %s (%s, code=%d)\n",
                gds_path, ele_fab_fiber_name(fiber), (int)fiber);
        return 1;
    }
    fiber = ele_fab_verify_gds_bytes(&g_ele_fab_ws, g_ele_fab_ws.gds_buf, g_ele_fab_ws.gds_len);
    if (fiber != ELE_FIBER_SIGNOFF_CLEAN ||
        g_ele_fab_ws.last_fiber != ELE_FIBER_SIGNOFF_CLEAN ||
        g_ele_fab_ws.fiber_witness_mask != UINT64_C(0)) {
        fprintf(stderr, "FATAL: DRC/LVS/GDSII sign-off failed on %s (%s, code=%d, mask=0x%" PRIX64 ")\n",
                gds_path, ele_fab_fiber_name(fiber), (int)fiber, g_ele_fab_ws.fiber_witness_mask);
        return 1;
    }
    printf("  [FLOW-FAB] pdk=%-16s | pitch=%4dnm g=%2dnm | reps=%d crit=%4dps wire=%3dum "
           "ir=%2dmV | opc_epe=%2d->%2dnm (%d serifs) | gds=%s (%uB, polys=%d, fnv=0x%016" PRIX64 ")\n",
           pdk.name, pdk.pitch_nm, pdk.grid_nm,
           g_ele_fab_ws.repeaters_inserted, g_ele_fab_ws.crit_delay_ps,
           g_ele_fab_ws.total_wire_um, g_ele_fab_ws.max_ir_drop_mv,
           g_ele_fab_ws.epe_naive_max_nm, g_ele_fab_ws.epe_opc_max_nm,
           g_ele_fab_ws.opc_polys_added, gds_path,
           g_ele_fab_ws.gds_len, g_ele_fab_ws.parsed_count, g_ele_fab_ws.gds_fnv1a);
    return 0;
}

static int run_fab_verify(const char *build_dir) {
    static const char *k_pdk_names[3] = {"sky130", "euv7", "sic800v"};
    uint64_t gds_hashes[3] = {0ULL, 0ULL, 0ULL};
    int32_t  reps[3]       = {0, 0, 0};

    for (int p = 0; p < 3; ++p) {
        char pdk_path[512];
        char gds_path[512];
        snprintf(pdk_path, sizeof(pdk_path), "%s/pdk_%s.flow", build_dir, k_pdk_names[p]);
        snprintf(gds_path, sizeof(gds_path), "%s/ele_power_%s.gds", build_dir, k_pdk_names[p]);

        ele_fab_pdk_t pdk;
        ele_fab_fiber_t fiber = ele_fab_load_pdk(pdk_path, &pdk);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fab-verify failed loading %s (%s, code=%d)\n",
                    pdk_path, ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }

        /* 1. Unified HoTT SEARU x FLOW 2-Jet co-evolution + 2-Jet OPC + GDSII write */
        fiber = ele_fab_compile_die(&g_ele_fab_ws, &pdk, true);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fab-verify compile_die failed for %s (%s, code=%d)\n",
                    k_pdk_names[p], ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_apply_opc_2jet(&g_ele_fab_ws);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fab-verify OPC failed for %s (%s, code=%d)\n",
                    k_pdk_names[p], ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }
        fiber = ele_fab_write_gds(&g_ele_fab_ws, gds_path);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN) {
            fprintf(stderr, "FATAL: --fab-verify write_gds failed for %s (%s, code=%d)\n",
                    gds_path, ele_fab_fiber_name(fiber), (int)fiber);
            return 1;
        }

        uint32_t ref_len    = g_ele_fab_ws.gds_len;
        uint64_t ref_hash   = g_ele_fab_ws.gds_fnv1a;
        int32_t  unif_reps  = g_ele_fab_ws.repeaters_inserted;
        int32_t  unif_delay = g_ele_fab_ws.crit_delay_ps;
        int32_t  unif_wire  = g_ele_fab_ws.total_wire_um;
        int32_t  unif_ir    = g_ele_fab_ws.max_ir_drop_mv;
        int64_t  unif_E     = g_ele_fab_ws.total_energy;
        int32_t  epe_naive  = g_ele_fab_ws.epe_naive_max_nm;
        int32_t  epe_opc    = g_ele_fab_ws.epe_opc_max_nm;
        int32_t  n_serifs   = g_ele_fab_ws.opc_polys_added;
        if (ref_len == 0U || ref_len > ELE_FAB_MAX_GDS_BYTES) {
            fprintf(stderr, "FATAL: invalid GDS byte count %u\n", ref_len);
            return 1;
        }
        memcpy(g_ele_fab_ref_buf, g_ele_fab_ws.gds_buf, ref_len);

        /* 2. Raw disk readback + independent binary GDSII/DRC/LVS/OPC verification */
        FILE *fp = fopen(gds_path, "rb");
        if (!fp) {
            fprintf(stderr, "FATAL: cannot reopen written GDS %s\n", gds_path);
            return 1;
        }
        size_t rd = fread(g_ele_fab_disk_buf, 1U, sizeof(g_ele_fab_disk_buf), fp);
        fclose(fp);
        if (rd != (size_t)ref_len) {
            fprintf(stderr, "FATAL: disk GDS size mismatch on %s\n", gds_path);
            return 1;
        }
        fiber = ele_fab_verify_gds_bytes(&g_ele_fab_ws, g_ele_fab_disk_buf, (uint32_t)rd);
        if (fiber != ELE_FIBER_SIGNOFF_CLEAN ||
            g_ele_fab_ws.last_fiber != ELE_FIBER_SIGNOFF_CLEAN ||
            g_ele_fab_ws.fiber_witness_mask != UINT64_C(0)) {
            fprintf(stderr, "FATAL: physical DRC/LVS/GDSII sign-off failed on %s (%s, code=%d, mask=0x%" PRIX64 ")\n",
                    gds_path, ele_fab_fiber_name(fiber), (int)fiber, g_ele_fab_ws.fiber_witness_mask);
            return 1;
        }
        int32_t unif_polys = g_ele_fab_ws.parsed_count;

        /* 3. Bit-Exact Determinism Gate: re-compile from scratch and verify memcmp == 0 */
        (void)ele_fab_compile_die(&g_ele_fab_ws, &pdk, true);
        (void)ele_fab_apply_opc_2jet(&g_ele_fab_ws);
        (void)ele_fab_write_gds(&g_ele_fab_ws, NULL);
        if (g_ele_fab_ws.gds_len != ref_len ||
            g_ele_fab_ws.gds_fnv1a != ref_hash ||
            memcmp(g_ele_fab_ws.gds_buf, g_ele_fab_ref_buf, ref_len) != 0) {
            fprintf(stderr, "FATAL: Bit-exact GDSII determinism violated on %s!\n", k_pdk_names[p]);
            return 1;
        }

        /* 4. Front-End / Back-End Unification Ablation on interconnect-dominated 7nm EUV */
        if (p == 1) { /* euv7 */
            ele_fab_compile_die(&g_ele_fab_ws, &pdk, false);
            int32_t split_delay = g_ele_fab_ws.crit_delay_ps;
            int64_t split_E     = g_ele_fab_ws.total_energy;
            if (unif_reps <= 0 || unif_delay >= split_delay || unif_E >= split_E) {
                fprintf(stderr,
                        "FATAL: Univalent 7nm EUV ablation failed (reps=%d, delay %d vs split %d, E %" PRId64 " vs %" PRId64 ")\n",
                        unif_reps, unif_delay, split_delay, unif_E, split_E);
                return 1;
            }
            printf("  [UNIVALENCE-EUV7] Split(no-rep): crit=%dps E=%" PRId64
                   " -> Unified(rep=%d): crit=%dps E=%" PRId64 " (DELAY & CLOSURE IMPROVED)\n",
                   split_delay, split_E, unif_reps, unif_delay, unif_E);
        }

        if (epe_naive <= 0 || epe_opc >= epe_naive || n_serifs <= 0) {
            fprintf(stderr, "FATAL: 2-Jet OPC EPE contraction failed on %s (%d -> %d nm, serifs=%d)\n",
                    k_pdk_names[p], epe_naive, epe_opc, n_serifs);
            return 1;
        }

        gds_hashes[p] = ref_hash;
        reps[p]       = unif_reps;
        printf("  [FAB-SIGNOFF %-7s] bits=0x%016" PRIX64 " (%uB, %d polys) | reps=%d crit=%4dps "
               "wire=%3dum ir=%2dmV | OPC=%2d->%2dnm (%d serifs) | DRC=0 LVS=0-open/0-short\n",
               k_pdk_names[p], ref_hash, ref_len, unif_polys,
               unif_reps, unif_delay, unif_wire, unif_ir,
               epe_naive, epe_opc, n_serifs);
    }

    if (gds_hashes[0] == gds_hashes[1] || gds_hashes[1] == gds_hashes[2] ||
        gds_hashes[0] == gds_hashes[2] || reps[0] == reps[1]) {
        fprintf(stderr, "FATAL: Multi-PDK polymorphism check failed (identical GDS or repeaters)!\n");
        return 1;
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [--dsp-count <1..6>] [--max-depth <18..64>] --fir <core.fir> -o <out.sv> [--pdk <pdk.flow> --gds-out <out.gds>]\n"
            "       %s --fir-synth <in.fir> -o <out.sv> [--pdk <pdk.flow> --gds-out <out.gds>] [--verify-fir] [--dom-mask]\n"
            "       %s --fab-verify <build_dir>\n",
            prog, prog, prog);
}

int main(int argc, char **argv) {
    int         dsp_count    = 3;
    int         max_depth    = 36;
    const char *out_path     = NULL;
    const char *pdk_path     = NULL;
    const char *gds_out_path = NULL;
    const char *fab_verify_d = NULL;
    const char *fir_path     = NULL;
    const char *fir_synth_p  = NULL;
    bool        verify_fir   = false;
    bool        dom_mask     = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--dsp-count") == 0 && i + 1 < argc) {
            dsp_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--max-depth") == 0 && i + 1 < argc) {
            max_depth = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--pdk") == 0 && i + 1 < argc) {
            pdk_path = argv[++i];
        } else if (strcmp(argv[i], "--gds-out") == 0 && i + 1 < argc) {
            gds_out_path = argv[++i];
        } else if (strcmp(argv[i], "--fab-verify") == 0 && i + 1 < argc) {
            fab_verify_d = argv[++i];
        } else if (strcmp(argv[i], "--fir") == 0 && i + 1 < argc) {
            fir_path = argv[++i];
        } else if (strcmp(argv[i], "--fir-synth") == 0 && i + 1 < argc) {
            fir_synth_p = argv[++i];
        } else if (strcmp(argv[i], "--verify-fir") == 0) {
            verify_fir = true;
        } else if (strcmp(argv[i], "--dom-mask") == 0) {
            dom_mask = true;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (fab_verify_d != NULL) {
        return run_fab_verify(fab_verify_d);
    }

    if (fir_synth_p != NULL) {
        if (out_path == NULL) {
            usage(argv[0]);
            return 1;
        }
        return run_fir_synth_cmd(fir_synth_p, out_path, pdk_path, gds_out_path, verify_fir, dom_mask);
    }

    if (out_path == NULL || fir_path == NULL || dsp_count < 1 || dsp_count > 6 || max_depth < 18 || max_depth > 64) {
        usage(argv[0]);
        return 1;
    }

    /* Pass 1: Unified Lean SSOT `.fir` + FLOW 2-Jet Retiming + SEARU DSP Binding */
    uint32_t   rad_epochs = 0;
    int        k_rad      = solve_radix_unroll_2jet(max_depth, &rad_epochs);
    dag_cone_t cones[NUM_CONES];
    if (!flow_ir_load_file(&g_ele_fir_prog, fir_path, g_ele_fir_raw, sizeof(g_ele_fir_raw))) {
        fprintf(stderr, "FATAL: failed loading unified core IR %s\n", fir_path);
        return 1;
    }
    build_cones_from_fir(cones, &g_ele_fir_prog);
    for (int c = 0; c < NUM_CONES; ++c) {
        solve_cone_with_flow_2jet(&cones[c], dsp_count, max_depth);
    }

    const int base_ca  = 1;
    const int base_cb  = base_ca + cones[CONE_A].num_stages;
    const int st_isqrt = base_cb + cones[CONE_B].num_stages;
    const int base_cd  = st_isqrt + 1;
    const int st_udiv  = base_cd + cones[CONE_D].num_stages;
    const int base_cc  = st_udiv + 1;

    searu_hls_stats_t hls_stats;
    memset(&hls_stats, 0, sizeof(hls_stats));
    optimize_dsp_slots_searu(cones, dsp_count, max_depth,
                             base_ca, base_cb, base_cd, base_cc, &hls_stats);

    /* Pass 2: Global FSM Live-Range Analysis & SEARU Heterogeneous-Width Register Allocation */
    reg_alloc_t  ra;
    scalar_map_t smap;
    allocate_shared_cut_registers(cones, dsp_count, max_depth,
                                  base_ca, base_cb, base_cd, base_cc, &ra, &smap, &hls_stats);

    /* Pass 3: Critical-Path Optimized RTL & Shadow-SRAM Emitter */
    FILE *fp = fopen(out_path, "w");
    if (!fp) {
        perror("fopen");
        return 1;
    }
    emit_sv(fp, dsp_count, max_depth, k_rad, rad_epochs, cones, &ra, &smap, &hls_stats);
    fclose(fp);

    const int rad_iters   = 64 / k_rad;
    const int quad_cycles = cones[CONE_A].num_stages + cones[CONE_B].num_stages + rad_iters +
                            cones[CONE_D].num_stages + rad_iters + cones[CONE_C].num_stages;
    const int virt_bits   = cones[CONE_A].cut_reg_bits + cones[CONE_B].cut_reg_bits +
                            cones[CONE_D].cut_reg_bits + cones[CONE_C].cut_reg_bits;
    printf("  [FLOW-BOOTSTRAP] out=%s | dsp=%d max_depth=%d -> K_rad=%d, stages=(%d+%d+%d+%d), "
           "dsp_ops=%u->%u, cut_dff=%d->%d bits (%d lanes, mux=%u->%ub), "
           "searu_it=%u+%u (epi=%u+%u), quad_cyc=%d (CONVERGED)\n",
           out_path, dsp_count, max_depth, k_rad,
           cones[CONE_A].num_stages, cones[CONE_B].num_stages,
           cones[CONE_D].num_stages, cones[CONE_C].num_stages,
           hls_stats.dsp_uniq_ops_init, hls_stats.dsp_uniq_ops_best,
           virt_bits, ra.shared_cut_bits, ra.num_lanes,
           hls_stats.lane_mux_bits_init, hls_stats.lane_mux_bits_best,
           hls_stats.p1_iters, hls_stats.p2_iters,
           hls_stats.p1_epiphanies, hls_stats.p2_epiphanies,
           quad_cycles);

    if (pdk_path != NULL && gds_out_path != NULL) {
        if (run_single_fab_proj(pdk_path, gds_out_path) != 0) {
            return 1;
        }
    }
    return 0;
}
