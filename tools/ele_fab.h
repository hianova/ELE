/* SPDX-License-Identifier: MIT
 * tools/ele_fab.h — Pure-GPR Univalent HoTT Spec-to-Polygon, 2-Jet OPC/ILT & GDSII Engine (ELE v8.0.0)
 *
 * Pure C17 / C++20-clean single-header physical silicon compiler coupled to FLOW v8.0.0:
 *   1. `.flow` Process Metric Atlas Loader (decodes Lean 4-certified 320B PDK Atlases).
 *   2. Unified SEARU × FLOW 2-Jet Die Co-Evolution (eliminating the front-end / back-end split):
 *      - Discrete SEARU topology + univalent repeater splitting + slot ordering + layer selection,
 *      - Coupled FLOW 2-Jet continuous coordinate relaxation + Abacus grid-aligned legalization,
 *      - Constructive Kan multi-layer Manhattan Steiner (RSMT) routing (`M1`/`VIA12`/`M2`/`VIA23`/`M3`),
 *      - Power-rail `IR-Drop` (`flow_step_atlas` Chart 2) + Elmore quadratic interconnect closure.
 *   3. Pure-GPR 2D Nanometer Optical Diffraction Kernel & `2-Jet` Inverse Lithography (`OPC/ILT`)
 *      with MRC-safe line-end Hammerhead / Serif emission (`Layer 70`).
 *   4. Standard Binary `GDSII` v6.0 Writer (IBM Excess-64 Base-16 `Real8` 1nm DBU) + Independent
 *      Raw-Binary GDSII Parser, Exhaustive Polygon DRC & 3D Union-Find LVS Sign-Off Verifier.
 *
 * Zero floats, zero heap, zero indirect calls (`blr`/`br`), `-mgeneral-regs-only` & UB-trap clean.
 */
#ifndef ELE_FAB_H
#define ELE_FAB_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flow.h"
#include "searu.h"
#include "flow_ir.h"
#include "ele_pdk_abi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_FAB_PDK_BYTES       ELE_PDK_ABI_BUNDLE_BYTES
#define ELE_FAB_NUM_ROWS        4
#define ELE_FAB_BASE_CELLS      48
#define ELE_FAB_NUM_PADS        44
#define ELE_FAB_MAX_REPEATERS   8
#define ELE_FAB_MAX_NODES       112
#define ELE_FAB_MAX_NETS        80
#define ELE_FAB_MAX_NET_PINS    10
#define ELE_FAB_MAX_PINS        (ELE_FAB_MAX_NETS * ELE_FAB_MAX_NET_PINS)
#define ELE_FAB_MAX_POLYS       4096
#define ELE_FAB_MAX_GDS_BYTES   393216
#define ELE_FAB_MAX_LINE_ENDS   256
#define ELE_FAB_MAX_TRACKS      192

/* GDSII Layer Map (nm database unit = 1e-9 m) */
#define ELE_LAYER_HV_GUARD      10
#define ELE_LAYER_CELL_BOUND    20
#define ELE_LAYER_POLY_GATE     30
#define ELE_LAYER_POWER_RAIL    40 /* dt 0 = VDD, dt 1 = VSS */
#define ELE_LAYER_M1            51
#define ELE_LAYER_VIA12         52
#define ELE_LAYER_M2            53
#define ELE_LAYER_VIA23         54
#define ELE_LAYER_M3            55
#define ELE_LAYER_PAD           60
#define ELE_LAYER_OPC_SERIF     70 /* dt 1 = 2-Jet OPC hammerhead/serif */

/* Independent Sign-Off Error Codes */
enum {
    ELE_FAB_OK                     = 0,
    ELE_FAB_ERR_PDK_IO             = 1,
    ELE_FAB_ERR_PDK_FORMAT         = 2,
    ELE_FAB_ERR_GDS_HEADER         = 10,
    ELE_FAB_ERR_GDS_UNITS          = 11,
    ELE_FAB_ERR_GDS_BOUNDARY       = 12,
    ELE_FAB_ERR_GDS_TAIL           = 13,
    ELE_FAB_ERR_DRC_GRID           = 20,
    ELE_FAB_ERR_DRC_WIDTH          = 21,
    ELE_FAB_ERR_DRC_CELL_OVERLAP   = 22,
    ELE_FAB_ERR_DRC_RAIL_SHORT     = 23,
    ELE_FAB_ERR_DRC_VIA_ENCLOSURE  = 24,
    ELE_FAB_ERR_DRC_SPACE          = 25,
    ELE_FAB_ERR_DRC_HV_SPACE       = 26,
    ELE_FAB_ERR_DRC_OPC_MRC        = 27,
    ELE_FAB_ERR_LVS_PIN_MISS       = 30,
    ELE_FAB_ERR_LVS_OPEN           = 31,
    ELE_FAB_ERR_LVS_SHORT          = 32,
    ELE_FAB_ERR_LVS_PAD_COUNT      = 33,
    ELE_FAB_ERR_LITHO_PULLBACK     = 40
};

typedef struct {
    char     name[24];
    int32_t  grid_nm;
    int32_t  pitch_nm;
    int32_t  min_width_nm;
    int32_t  min_space_nm;
    int32_t  via_size_nm;
    int32_t  via_enc_nm;
    int32_t  hv_space_nm;
    int32_t  r_sheet_mohm;
    int32_t  c_unit_af;
    int32_t  lambda_nm;
    int32_t  opc_gain_q16;
    uint16_t kan_lock;
    flow_cell state_cell;
    flow_cell chart_cells[4];
} ele_fab_pdk_t;

typedef struct {
    char    name[24];
    uint8_t is_pad;
    uint8_t is_hv;
    uint8_t is_repeater;
    int8_t  pad_edge;     /* 0=N, 1=E, 2=S, 3=W, -1=core */
    int32_t row;          /* 0..3 for core cells, 0..10 pad index along edge */
    int32_t slot_pref;
    int32_t x0_nm;
    int32_t y0_nm;
    int32_t x1_nm;
    int32_t y1_nm;
    int32_t power_ua;
} ele_fab_node_t;

typedef struct {
    char    name[24];
    uint8_t is_hv;
    uint8_t is_critical;
    uint8_t can_split;
    uint8_t split_active;
    uint8_t prefer_m3;
    int32_t num_pins;
    int32_t pin_node[ELE_FAB_MAX_NET_PINS];
    int32_t pin_slot[ELE_FAB_MAX_NET_PINS]; /* 0,1,2 pin offset inside node */
    int32_t trunk_y_track;
    int32_t trunk_y_nm;
    int32_t trunk_x0_nm;
    int32_t trunk_x1_nm;
    int32_t wire_len_nm;
    int32_t delay_ps;
} ele_fab_net_t;

typedef struct {
    int16_t layer;
    int16_t datatype;
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t net_hint;
} ele_fab_poly_t;

typedef struct {
    int32_t net_id;
    int16_t layer;
    int8_t  dir;          /* 0 = left x0-end, 1 = right x1-end */
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t mrc_gap_nm;
    int32_t epe_naive_nm;
    int32_t epe_opc_nm;
    int32_t opc_bias_nm;
} ele_fab_line_end_t;

typedef struct {
    int32_t node_idx;
    int32_t net_idx;
    int32_t pin_slot;
    int32_t x_nm;
    int32_t y_nm;
    int16_t layer;
} ele_fab_pin_loc_t;

typedef struct {
    int32_t col;
    int32_t y0;
    int32_t y1;
    int32_t net_id;
    uint8_t is_hv;
} ele_fab_vseg_t;

typedef struct {
    ele_fab_pdk_t      pdk;
    int32_t            num_nodes;
    int32_t            num_base_nets;
    int32_t            num_nets;
    ele_fab_node_t     nodes[ELE_FAB_MAX_NODES];
    ele_fab_net_t      nets[ELE_FAB_MAX_NETS];

    int32_t            die_x0;
    int32_t            die_y0;
    int32_t            die_x1;
    int32_t            die_y1;
    int32_t            core_x0;
    int32_t            core_y0;
    int32_t            core_x1;
    int32_t            core_y1;
    int32_t            row_y0[ELE_FAB_NUM_ROWS];
    int32_t            row_y1[ELE_FAB_NUM_ROWS];

    /* Active routed outputs */
    int32_t            num_polys;
    ele_fab_poly_t     polys[ELE_FAB_MAX_POLYS];
    int32_t            num_pin_locs;
    ele_fab_pin_loc_t  pin_locs[ELE_FAB_MAX_PINS];
    int32_t            num_line_ends;
    ele_fab_line_end_t line_ends[ELE_FAB_MAX_LINE_ENDS];

    /* Physical sign-off & Pareto telemetry */
    int32_t            repeaters_inserted;
    int32_t            crit_delay_ps;
    int32_t            total_wire_um;
    int32_t            max_ir_drop_mv;
    int64_t            total_energy;
    int32_t            epe_naive_max_nm;
    int32_t            epe_opc_max_nm;
    int32_t            opc_polys_added;
    uint32_t           searu_iters;
    uint32_t           searu_epiphanies;

    /* Scratch occupancy intervals for constructive Kan routing */
    ele_fab_vseg_t     vsegs[ELE_FAB_MAX_PINS];
    int32_t            vseg_count;

    int32_t            m1_seg_trk[ELE_FAB_MAX_PINS];
    int32_t            m1_seg_x0[ELE_FAB_MAX_PINS];
    int32_t            m1_seg_x1[ELE_FAB_MAX_PINS];
    int32_t            m1_seg_net[ELE_FAB_MAX_PINS];
    int32_t            m1_seg_count;

    int32_t            m3_seg_trk[ELE_FAB_MAX_NETS];
    int32_t            m3_seg_x0[ELE_FAB_MAX_NETS];
    int32_t            m3_seg_x1[ELE_FAB_MAX_NETS];
    int32_t            m3_seg_net[ELE_FAB_MAX_NETS];
    int32_t            m3_seg_count;

    /* GDSII serialized output & independent raw parser scratch */
    uint32_t           gds_len;
    uint64_t           gds_fnv1a;
    uint8_t            gds_buf[ELE_FAB_MAX_GDS_BYTES];

    int32_t            parsed_count;
    ele_fab_poly_t     parsed_polys[ELE_FAB_MAX_POLYS];
    uint16_t           uf_parent[ELE_FAB_MAX_POLYS];
    uint8_t            uf_is_hv[ELE_FAB_MAX_POLYS];

    /* Optional custom FIR spec nets preserved across SEARU iterations */
    uint8_t            custom_spec;
    int32_t            saved_num_nets;
    ele_fab_net_t      saved_nets[ELE_FAB_MAX_NETS];
} ele_fab_ws_t;

static inline uint64_t ele_fab_read_le64(const uint8_t *p) {
    return ((uint64_t)p[0])       | ((uint64_t)p[1] << 8)  |
           ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static inline uint16_t ele_fab_read_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline int32_t ele_fab_read_be32(const uint8_t *p) {
    uint32_t u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8)  | ((uint32_t)p[3]);
    return (int32_t)u;
}

static inline uint64_t ele_fab_read_be64(const uint8_t *p) {
    return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
           ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
           ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
           ((uint64_t)p[6] << 8)  | ((uint64_t)p[7]);
}

static inline void ele_fab_write_be16(uint8_t *dst, uint32_t *pos, uint16_t v) {
    if (*pos + 2U <= ELE_FAB_MAX_GDS_BYTES) {
        dst[*pos]      = (uint8_t)((v >> 8) & 0xFFU);
        dst[*pos + 1U] = (uint8_t)(v & 0xFFU);
        *pos += 2U;
    }
}

static inline void ele_fab_write_be32(uint8_t *dst, uint32_t *pos, int32_t v) {
    uint32_t u = (uint32_t)v;
    if (*pos + 4U <= ELE_FAB_MAX_GDS_BYTES) {
        dst[*pos]      = (uint8_t)((u >> 24) & 0xFFU);
        dst[*pos + 1U] = (uint8_t)((u >> 16) & 0xFFU);
        dst[*pos + 2U] = (uint8_t)((u >> 8)  & 0xFFU);
        dst[*pos + 3U] = (uint8_t)(u & 0xFFU);
        *pos += 4U;
    }
}

static inline void ele_fab_write_be64(uint8_t *dst, uint32_t *pos, uint64_t v) {
    if (*pos + 8U <= ELE_FAB_MAX_GDS_BYTES) {
        dst[*pos]      = (uint8_t)((v >> 56) & 0xFFU);
        dst[*pos + 1U] = (uint8_t)((v >> 48) & 0xFFU);
        dst[*pos + 2U] = (uint8_t)((v >> 40) & 0xFFU);
        dst[*pos + 3U] = (uint8_t)((v >> 32) & 0xFFU);
        dst[*pos + 4U] = (uint8_t)((v >> 24) & 0xFFU);
        dst[*pos + 5U] = (uint8_t)((v >> 16) & 0xFFU);
        dst[*pos + 6U] = (uint8_t)((v >> 8)  & 0xFFU);
        dst[*pos + 7U] = (uint8_t)(v & 0xFFU);
        *pos += 8U;
    }
}

static inline int32_t ele_fab_i32_min(int32_t a, int32_t b) { return (a < b) ? a : b; }
static inline int32_t ele_fab_i32_max(int32_t a, int32_t b) { return (a > b) ? a : b; }
static inline int32_t ele_fab_i32_abs(int32_t a) { return (a < 0) ? -a : a; }

static inline int32_t ele_fab_snap_down(int32_t x, int32_t g) {
    if (g <= 1) {
        return x;
    }
    int32_t rem = x % g;
    if (rem < 0) {
        rem += g;
    }
    return x - rem;
}

static inline uint64_t ele_fab_fnv1a(const uint8_t *buf, uint32_t len) {
    uint64_t h = UINT64_C(0xCBF29CE484222325);
    for (uint32_t i = 0U; i < len; ++i) {
        h ^= (uint64_t)buf[i];
        h *= UINT64_C(0x00000100000001B3);
    }
    return h;
}

/* ============================================================================
 * 1. Process Physics Metric .flow Loader
 * ============================================================================ */

static inline int ele_fab_load_pdk(const char *path, ele_fab_pdk_t *out) {
    if (!path || !out) {
        return ELE_FAB_ERR_PDK_IO;
    }
    memset(out, 0, sizeof(*out));
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return ELE_FAB_ERR_PDK_IO;
    }
    uint8_t raw[ELE_FAB_PDK_BYTES + 16U];
    size_t n = fread(raw, 1U, sizeof(raw), fp);
    fclose(fp);
    if (n != ELE_FAB_PDK_BYTES) {
        return ELE_FAB_ERR_PDK_FORMAT;
    }

    for (uint32_t w = 0U; w < 8U; ++w) {
        out->state_cell[w] = ele_fab_read_le64(raw + (size_t)w * 8U);
    }
    flow_cell all_charts[ELE_PDK_ABI_CHART_COUNT];
    for (size_t c = 0U; c < ELE_PDK_ABI_CHART_COUNT; ++c) {
        for (uint32_t w = 0U; w < 8U; ++w) {
            all_charts[c][w] = ele_fab_read_le64(raw + (c + 1U) * 64U + (size_t)w * 8U);
        }
        uint32_t epoch = (uint32_t)(all_charts[c][6] >> 32U);
        uint32_t tag   = (uint32_t)(all_charts[c][6] & UINT64_C(0xFFFFFFFF));
        int32_t b_exit  = (int32_t)(uint32_t)(all_charts[c][3] >> 32U);
        int32_t b_enter = (int32_t)(uint32_t)(all_charts[c][3] & UINT64_C(0xFFFFFFFF));
        int32_t dt_max  = (int32_t)(uint32_t)(all_charts[c][4] >> 32U);
        if (epoch < 1U || tag != 0U || b_exit > b_enter || dt_max < 1) {
            return ELE_FAB_ERR_PDK_FORMAT;
        }
        if (c < 4U) {
            memcpy(out->chart_cells[c], all_charts[c], sizeof(flow_cell));
        }
    }

    ele_pdk_abi_t abi;
    if (!ele_pdk_abi_unpack(all_charts, ELE_PDK_ABI_CHART_COUNT, &abi)) {
        return ELE_FAB_ERR_PDK_FORMAT;
    }

    out->grid_nm      = (int32_t)abi.grid_nm;
    out->pitch_nm     = (int32_t)abi.pitch_nm;
    out->min_width_nm = (int32_t)abi.min_width_nm;
    out->min_space_nm = (int32_t)abi.min_space_nm;
    out->via_size_nm  = (int32_t)abi.via_size_nm;
    out->via_enc_nm   = (int32_t)abi.via_enc_nm;
    out->hv_space_nm  = (int32_t)abi.hv_space_nm;
    out->r_sheet_mohm = (int32_t)abi.r_sheet_mohm;
    out->c_unit_af    = (int32_t)abi.c_unit_af;
    out->lambda_nm    = (int32_t)abi.lambda_nm;
    out->opc_gain_q16 = (int32_t)abi.opc_gain_q16;
    out->kan_lock     = (uint16_t)abi.kan_lock;

    const char *base = path;
    for (const char *s = path; *s != '\0'; ++s) {
        if (*s == '/') {
            base = s + 1;
        }
    }
    snprintf(out->name, sizeof(out->name), "%s", base);

    int32_t w_pad = out->via_size_nm + 2 * out->via_enc_nm;
    if (out->grid_nm <= 0 || out->min_width_nm < out->grid_nm || out->min_space_nm <= 0 ||
        out->via_size_nm <= 0 || out->via_enc_nm < 0 ||
        w_pad + out->min_space_nm > out->pitch_nm ||
        w_pad + out->hv_space_nm > 2 * out->pitch_nm ||
        (out->pitch_nm % out->grid_nm) != 0 ||
        (out->min_width_nm % out->grid_nm) != 0 ||
        (out->min_space_nm % out->grid_nm) != 0 ||
        (out->via_size_nm % out->grid_nm) != 0 ||
        (out->via_enc_nm % out->grid_nm) != 0 ||
        out->r_sheet_mohm <= 0 || out->c_unit_af <= 0 || out->lambda_nm <= 0) {
        return ELE_FAB_ERR_PDK_FORMAT;
    }
    return ELE_FAB_OK;
}

/* ============================================================================
 * 2. Unified ELE v8.0.0 Silicon Graph Specification (Core + Power + QFN-48 Pads)
 * ============================================================================ */

static inline void ele_fab_add_pin_to_net(ele_fab_ws_t *ws, int32_t net_idx,
                                          int32_t node_idx, int32_t pin_slot) {
    if (net_idx < 0 || net_idx >= ws->num_base_nets) {
        return;
    }
    ele_fab_net_t *net = &ws->nets[net_idx];
    if (net->num_pins >= ELE_FAB_MAX_NET_PINS) {
        return;
    }
    int32_t k = net->num_pins++;
    net->pin_node[k] = node_idx;
    net->pin_slot[k] = pin_slot;
}

static inline void ele_fab_init_base_nets(ele_fab_ws_t *ws) {
    ws->num_base_nets = 52;
    memset(ws->nets, 0, (size_t)ws->num_base_nets * sizeof(ws->nets[0]));
    for (int n = 0; n < ws->num_base_nets; ++n) {
        ele_fab_net_t *net = &ws->nets[n];
        snprintf(net->name, sizeof(net->name), "N_%02d", n);
        net->is_critical = (n < 12) ? 1U : 0U;
        net->can_split   = (n < ELE_FAB_MAX_REPEATERS) ? 1U : 0U;
        net->is_hv       = (n >= 46) ? 1U : 0U;
        net->prefer_m3   = 1U;

        if (n < 8) {
            int32_t src = (n * 5) % 36;
            int32_t dst_mid = (n * 7 + 14) % 36;
            int32_t dst_far = 36 + (n % 12);
            if (dst_mid == src) {
                dst_mid = (src + 1) % 36;
            }
            ele_fab_add_pin_to_net(ws, n, src, 2);
            ele_fab_add_pin_to_net(ws, n, dst_mid, 0);
            ele_fab_add_pin_to_net(ws, n, dst_far, 1);
            ele_fab_add_pin_to_net(ws, n, ELE_FAB_BASE_CELLS + n, 0);
        } else if (n < 38) {
            int32_t src = n % 42;
            int32_t dst = (src + 1 + (n % 5)) % 42;
            if (dst == src) {
                dst = (src + 1) % 42;
            }
            ele_fab_add_pin_to_net(ws, n, src, 2);
            ele_fab_add_pin_to_net(ws, n, dst, 1);
            ele_fab_add_pin_to_net(ws, n, ELE_FAB_BASE_CELLS + n, 0);
        } else if (n < 46) {
            int32_t src = (n - 38) * 4;
            int32_t dst = src + 2;
            ele_fab_add_pin_to_net(ws, n, src, 2);
            ele_fab_add_pin_to_net(ws, n, dst, 0);
        } else {
            int32_t hv_idx = n - 46;
            int32_t src = 42 + hv_idx;
            int32_t dst = 36 + hv_idx;
            int32_t pad = ELE_FAB_BASE_CELLS + 38 + hv_idx;
            ele_fab_add_pin_to_net(ws, n, src, 2);
            ele_fab_add_pin_to_net(ws, n, dst, 0);
            ele_fab_add_pin_to_net(ws, n, pad, 0);
        }
    }
    ws->num_nets = ws->num_base_nets;
}

static inline void ele_fab_build_chip_spec(ele_fab_ws_t *ws, const ele_fab_pdk_t *pdk) {
    memset(ws, 0, sizeof(*ws));
    ws->pdk = *pdk;
    static const char *k_core_names[ELE_FAB_BASE_CELLS] = {
        "a0_ug_u", "a1_b1_x", "a2_x_sq", "a3_b2_x", "a4_b0_eff", "a5_b2x_l",
        "a6_b2x_h", "a7_g0_op", "a8_g1_op", "a9_jet_a", "a10_jmux", "b0_g1_ll",
        "b1_g1_hl", "b2_g1_hh", "b3_g0r_0", "b4_g0r_1", "b5_g0r_2", "b6_g1_sq",
        "b7_g0_r2", "b8_disc", "d0_cits", "d1_dnorm", "c0_dcand", "c1_dsq",
        "c2_g1d_l", "c3_g1d_h", "c4_r2dc", "c5_r2d_l", "c6_r2d_h", "c7_g0c",
        "c8_lyap", "c9_prnd", "c10_kan", "prim_dsp", "prim_rad", "prim_srm",
        "dpwm_p0", "dpwm_p1", "dpwm_p2", "dpwm_p3", "dpwm_dt", "dpwm_shd",
        "sscb_didt", "sscb_d2i", "sscb_i2t", "sscb_ocp", "sscb_arm", "sscb_kil"
    };

    for (int i = 0; i < ELE_FAB_BASE_CELLS; ++i) {
        ele_fab_node_t *nd = &ws->nodes[ws->num_nodes++];
        snprintf(nd->name, sizeof(nd->name), "%s", k_core_names[i]);
        nd->row       = i / 12;
        nd->slot_pref = i % 12;
        nd->pad_edge  = -1;
        nd->is_pad    = 0U;
        nd->is_hv     = (i >= 42) ? 1U : 0U;
        nd->power_ua  = (i >= 36) ? 420 : ((i >= 33) ? 680 : 180);
    }

    for (int p = 0; p < ELE_FAB_NUM_PADS; ++p) {
        ele_fab_node_t *pad = &ws->nodes[ws->num_nodes++];
        snprintf(pad->name, sizeof(pad->name), "PAD_%02d", p);
        pad->is_pad   = 1U;
        pad->is_hv    = (p >= 38) ? 1U : 0U;
        pad->pad_edge = (int8_t)(p / 11);
        pad->row      = p % 11;
    }

    ele_fab_init_base_nets(ws);
}

/* ============================================================================
 * 3. Constructive Kan Legalizer, Interval-Graph Router, IR-Drop & 2-Jet OPC
 * ============================================================================ */

static inline int32_t ele_fab_alloc_m2_col(const ele_fab_vseg_t *vsegs, int32_t vcnt,
                                           int32_t pref_col, int32_t min_col, int32_t max_col,
                                           int32_t y0, int32_t y1, int32_t net_id,
                                           uint8_t is_hv, int32_t y_guard) {
    const int32_t q_y0 = y0 - y_guard;
    const int32_t q_y1 = y1 + y_guard;
    int32_t span = max_col - min_col + 1;
    if (span < 1) {
        span = 1;
    }
    for (int32_t d = 0; d <= span; ++d) {
        for (int32_t s = 0; s < 2; ++s) {
            if (d == 0 && s == 1) {
                continue;
            }
            int32_t cand = pref_col + ((s == 0) ? d : -d);
            if (cand < min_col || cand > max_col) {
                continue;
            }
            bool ok = true;
            for (int32_t i = 0; i < vcnt; ++i) {
                if (vsegs[i].net_id == net_id) {
                    continue;
                }
                int32_t col_need = (is_hv || vsegs[i].is_hv) ? 2 : 1;
                if (ele_fab_i32_abs(cand - vsegs[i].col) < col_need) {
                    if (!(q_y1 < vsegs[i].y0 || vsegs[i].y1 < q_y0)) {
                        ok = false;
                        break;
                    }
                }
            }
            if (ok) {
                return cand;
            }
        }
    }
    return pref_col;
}

static inline void ele_fab_emit_poly(ele_fab_ws_t *ws, int16_t layer, int16_t datatype,
                                     int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                                     int32_t net_hint) {
    if (ws->num_polys >= ELE_FAB_MAX_POLYS || x1 <= x0 || y1 <= y0) {
        return;
    }
    ele_fab_poly_t *p = &ws->polys[ws->num_polys++];
    p->layer    = layer;
    p->datatype = datatype;
    p->x0       = x0;
    p->y0       = y0;
    p->x1       = x1;
    p->y1       = y1;
    p->net_hint = net_hint;
}

static inline void ele_fab_record_line_end(ele_fab_ws_t *ws, int32_t net_id, int16_t layer,
                                           int8_t dir, int32_t x0, int32_t y0,
                                           int32_t x1, int32_t y1) {
    if (ws->num_line_ends >= ELE_FAB_MAX_LINE_ENDS) {
        return;
    }
    ele_fab_line_end_t *le = &ws->line_ends[ws->num_line_ends++];
    le->net_id = net_id;
    le->layer  = layer;
    le->dir    = dir;
    le->x0     = x0;
    le->y0     = y0;
    le->x1     = x1;
    le->y1     = y1;
}

static inline int ele_fab_legalize_and_route(ele_fab_ws_t *ws,
                                             const uint8_t perm_row[ELE_FAB_BASE_CELLS],
                                             const int32_t cell_target_x[ELE_FAB_BASE_CELLS],
                                             uint8_t split_mask,
                                             bool emit_geometry,
                                             bool relax_2jet) {
    const ele_fab_pdk_t *pdk = &ws->pdk;
    const int32_t P = pdk->pitch_nm;
    const int32_t W = pdk->min_width_nm;
    const int32_t V = pdk->via_size_nm;
    const int32_t E = pdk->via_enc_nm;

    const int32_t row_tracks  = 34; /* 16 M1 internal row tracks at 2*P stride */
    const int32_t chan_tracks = 36; /* M3 horizontal tracks per channel (5 * 16 = 80 tracks) */
    const int32_t cell_cols   = 6;
    const int32_t pad_margin  = 40 * P;

    ws->core_x0 = pad_margin;
    ws->core_y0 = pad_margin;
    ws->core_x1 = ws->core_x0 + 132 * P;

    for (int r = 0; r < ELE_FAB_NUM_ROWS; ++r) {
        int32_t y0 = ws->core_y0 + chan_tracks * P + r * (row_tracks + chan_tracks) * P;
        ws->row_y0[r] = y0;
        ws->row_y1[r] = y0 + row_tracks * P;
    }
    ws->core_y1 = ws->row_y1[ELE_FAB_NUM_ROWS - 1] + chan_tracks * P;
    ws->die_x0  = 0;
    ws->die_y0  = 0;
    ws->die_x1  = ws->core_x1 + pad_margin;
    ws->die_y1  = ws->core_y1 + pad_margin;

    /* Reset repeater nodes & restore clean base nets before applying univalent repeater cloning */
    ws->num_nodes = ELE_FAB_BASE_CELLS + ELE_FAB_NUM_PADS;
    if (ws->custom_spec != 0U && ws->saved_num_nets > 0 && ws->saved_num_nets <= ELE_FAB_MAX_NETS) {
        ws->num_base_nets = ws->saved_num_nets;
        ws->num_nets      = ws->saved_num_nets;
        memcpy(ws->nets, ws->saved_nets, (size_t)ws->saved_num_nets * sizeof(ws->nets[0]));
    } else {
        ele_fab_init_base_nets(ws);
    }
    ws->repeaters_inserted = 0;

    int32_t node_row[ELE_FAB_MAX_NODES];
    int32_t node_x_req[ELE_FAB_MAX_NODES];
    for (int i = 0; i < ELE_FAB_BASE_CELLS; ++i) {
        node_row[i] = perm_row[i] & 3;
        int32_t xq = cell_target_x[i];
        if (relax_2jet) {
            flow_cell st = {0ULL, 0ULL, 0ULL, (uint64_t)(int64_t)i * 65536ULL, 0ULL, 0ULL, 0ULL, 0ULL};
            flow_cell next_s;
            flow_step_atlas(next_s, st, pdk->chart_cells, 3ULL,
                            (int64_t)xq * 65536, pdk->kan_lock);
            int32_t x_flow = (int32_t)((int64_t)next_s[3] >> 16);
            xq = (xq + x_flow) / 2;
        }
        node_x_req[i] = xq;
    }

    /* Step A: Insert Univalent HoTT Repeaters at geometric midpoints BEFORE Abacus legalization */
    int32_t active_core_cells = ELE_FAB_BASE_CELLS;
    for (int k = 0; k < ELE_FAB_MAX_REPEATERS; ++k) {
        if ((split_mask & (uint8_t)(1U << (unsigned)k)) == 0U ||
            ws->nets[k].num_pins < 3 ||
            ws->num_nodes >= ELE_FAB_MAX_NODES ||
            ws->num_nets >= ELE_FAB_MAX_NETS) {
            continue;
        }
        ele_fab_net_t *orig = &ws->nets[k];
        int32_t src_node = orig->pin_node[0];
        int32_t far_idx  = 1;
        int32_t max_dist = -1;
        for (int p = 1; p < orig->num_pins; ++p) {
            int32_t dn = orig->pin_node[p];
            if (dn >= ELE_FAB_BASE_CELLS) {
                continue;
            }
            int32_t dist = ele_fab_i32_abs(node_x_req[dn] - node_x_req[src_node]) +
                           ele_fab_i32_abs(node_row[dn] - node_row[src_node]) * 24;
            if (dist > max_dist) {
                max_dist = dist;
                far_idx  = p;
            }
        }
        int32_t rep_id = ws->num_nodes++;
        ele_fab_node_t *rep = &ws->nodes[rep_id];
        memset(rep, 0, sizeof(*rep));
        snprintf(rep->name, sizeof(rep->name), "UREP_%d", k);
        rep->is_repeater = 1U;
        rep->pad_edge    = -1;
        rep->power_ua    = 90;

        int32_t far_node = orig->pin_node[far_idx];
        int32_t far_slot = orig->pin_slot[far_idx];
        node_row[rep_id]   = (node_row[src_node] + node_row[far_node]) / 2;
        node_x_req[rep_id] = (node_x_req[src_node] + node_x_req[far_node]) / 2;
        rep->row           = node_row[rep_id];

        int32_t sub_idx  = ws->num_nets++;
        ele_fab_net_t *sub = &ws->nets[sub_idx];
        memset(sub, 0, sizeof(*sub));
        snprintf(sub->name, sizeof(sub->name), "N_%02d_R", k);
        sub->is_hv       = orig->is_hv;
        sub->is_critical = 1U;
        sub->prefer_m3   = 1U;
        sub->num_pins    = 2;
        sub->pin_node[0] = rep_id;
        sub->pin_slot[0] = 2;
        sub->pin_node[1] = far_node;
        sub->pin_slot[1] = far_slot;

        orig->split_active      = 1U;
        orig->pin_node[far_idx] = rep_id;
        orig->pin_slot[far_idx] = 0;

        /* Move any far/perimeter pad sink on orig into post-repeater sub-net so both spans halve */
        for (int p = 1; p < orig->num_pins; ++p) {
            int32_t pn = orig->pin_node[p];
            if (pn >= ELE_FAB_BASE_CELLS && pn < ELE_FAB_BASE_CELLS + ELE_FAB_NUM_PADS &&
                sub->num_pins < ELE_FAB_MAX_NET_PINS) {
                int32_t ps = orig->pin_slot[p];
                sub->pin_node[sub->num_pins]   = pn;
                sub->pin_slot[sub->num_pins++] = ps;
                for (int q = p; q + 1 < orig->num_pins; ++q) {
                    orig->pin_node[q] = orig->pin_node[q + 1];
                    orig->pin_slot[q] = orig->pin_slot[q + 1];
                }
                orig->num_pins--;
                break;
            }
        }
        ws->repeaters_inserted++;
    }

    /* Step B: Abacus monotonic left-to-right row legalization (base cells + midpoint repeaters) */
    for (int r = 0; r < ELE_FAB_NUM_ROWS; ++r) {
        int32_t idx_list[ELE_FAB_MAX_NODES];
        int32_t cnt = 0;
        for (int i = 0; i < ws->num_nodes; ++i) {
            if (!ws->nodes[i].is_pad && node_row[i] == r) {
                idx_list[cnt++] = i;
            }
        }
        for (int a = 1; a < cnt; ++a) {
            int32_t key = idx_list[a];
            int32_t b = a - 1;
            while (b >= 0 && node_x_req[idx_list[b]] > node_x_req[key]) {
                idx_list[b + 1] = idx_list[b];
                --b;
            }
            idx_list[b + 1] = key;
        }
        int32_t cursor_col = 4;
        for (int a = 0; a < cnt; ++a) {
            int32_t id = idx_list[a];
            int32_t pref_col = ele_fab_i32_max(4, node_x_req[id]);
            int32_t col = (pref_col > cursor_col) ? pref_col : cursor_col;
            int32_t c_width = ws->nodes[id].is_repeater ? 4 :
                              ((id >= 33 && id < 36) ? (cell_cols + 2) : cell_cols);
            ws->nodes[id].row   = r;
            ws->nodes[id].x0_nm = ws->core_x0 + col * P;
            ws->nodes[id].x1_nm = ws->core_x0 + (col + c_width) * P;
            ws->nodes[id].y0_nm = ws->row_y0[r];
            ws->nodes[id].y1_nm = ws->row_y1[r];
            cursor_col = col + c_width + (ws->nodes[id].is_hv ? 2 : 1);
            if (ws->nodes[id].x1_nm + 4 * P > ws->core_x1) {
                ws->core_x1 = ws->nodes[id].x1_nm + 8 * P;
                ws->die_x1  = ws->core_x1 + pad_margin;
            }
        }
    }
    (void)active_core_cells;

    /* Step C: Position 44 QFN-48 perimeter bond pads AFTER final core_x1 sizing */
    int32_t core_w = ws->core_x1 - ws->core_x0;
    int32_t core_h = ws->core_y1 - ws->core_y0;
    int32_t pad_w  = 4 * P;
    int32_t step_x = ele_fab_snap_down((core_w - 8 * P) / 11, P);
    int32_t step_y = ele_fab_snap_down((core_h - 8 * P) / 11, P);
    if (step_x < 6 * P) {
        step_x = 6 * P;
    }
    if (step_y < 6 * P) {
        step_y = 6 * P;
    }
    for (int p = 0; p < ELE_FAB_NUM_PADS; ++p) {
        ele_fab_node_t *pad = &ws->nodes[ELE_FAB_BASE_CELLS + p];
        int32_t side = p / 11;
        int32_t k    = p % 11;
        if (side == 0) { /* North */
            pad->x0_nm = ws->core_x0 + 4 * P + k * step_x;
            pad->y0_nm = ws->core_y1 + 28 * P;
        } else if (side == 1) { /* East (outside core_x1 + 24*P M2 escape tracks) */
            pad->x0_nm = ws->core_x1 + 30 * P;
            pad->y0_nm = ws->core_y0 + 4 * P + k * step_y;
        } else if (side == 2) { /* South */
            pad->x0_nm = ws->core_x0 + 4 * P + k * step_x;
            pad->y0_nm = ws->core_y0 - 32 * P;
        } else { /* West (outside core_x0 - 24*P M2 escape tracks) */
            pad->x0_nm = ws->core_x0 - 34 * P;
            pad->y0_nm = ws->core_y0 + 4 * P + k * step_y;
        }
        pad->x1_nm = pad->x0_nm + pad_w;
        pad->y1_nm = pad->y0_nm + pad_w;
    }

    /* Step D: Constructive Kan Multi-Layer RSMT Routing & Interval Track Allocation */
    ws->num_polys     = 0;
    ws->num_pin_locs  = 0;
    ws->num_line_ends = 0;
    ws->vseg_count    = 0;
    ws->m1_seg_count  = 0;
    ws->m3_seg_count  = 0;

    int32_t global_y_tracks[ELE_FAB_MAX_TRACKS];
    int32_t num_global_tracks = 0;
    for (int ch = 0; ch < ELE_FAB_NUM_ROWS + 1; ++ch) {
        int32_t ch_base = (ch == 0) ? (ws->core_y0 + 2 * P)
                                    : (ws->row_y1[ch - 1] + 2 * P);
        int32_t cnt_t = (chan_tracks - 4) / 2;
        for (int t = 0; t < cnt_t && num_global_tracks < ELE_FAB_MAX_TRACKS; ++t) {
            global_y_tracks[num_global_tracks++] = ch_base + t * 2 * P;
        }
    }

    int32_t core_cols = (ws->core_x1 - ws->core_x0) / P;
    if (core_cols < 80) {
        core_cols = 80;
    }

    /* Pre-reserve N/S pad outer drop M2 columns so interior core columns stay disjoint */
    int32_t pad_net_map[ELE_FAB_NUM_PADS];
    for (int p = 0; p < ELE_FAB_NUM_PADS; ++p) {
        pad_net_map[p] = -1;
    }
    for (int n = 0; n < ws->num_nets; ++n) {
        for (int p = 0; p < ws->nets[n].num_pins; ++p) {
            int32_t nid = ws->nets[n].pin_node[p];
            if (nid >= ELE_FAB_BASE_CELLS && nid < ELE_FAB_BASE_CELLS + ELE_FAB_NUM_PADS) {
                pad_net_map[nid - ELE_FAB_BASE_CELLS] = n;
            }
        }
    }
    for (int p = 0; p < ELE_FAB_NUM_PADS; ++p) {
        if (pad_net_map[p] < 0) {
            continue;
        }
        const ele_fab_node_t *pad = &ws->nodes[ELE_FAB_BASE_CELLS + p];
        if (pad->pad_edge == 0 || pad->pad_edge == 2) {
            int32_t px     = pad->x0_nm + P;
            int32_t py     = pad->y0_nm + P;
            int32_t k      = pad->row;
            int32_t y_jog  = (pad->pad_edge == 0) ? (ws->core_y1 + (2 + 2 * k) * P)
                                                  : (ws->core_y0 - (2 + 2 * k) * P);
            int32_t y_lo   = ele_fab_i32_min(py, y_jog);
            int32_t y_hi   = ele_fab_i32_max(py, y_jog);
            if (ws->vseg_count < ELE_FAB_MAX_PINS) {
                ws->vsegs[ws->vseg_count].col    = (px - ws->core_x0) / P;
                ws->vsegs[ws->vseg_count].y0     = y_lo - 3 * P;
                ws->vsegs[ws->vseg_count].y1     = y_hi + 3 * P;
                ws->vsegs[ws->vseg_count].net_id = pad_net_map[p];
                ws->vsegs[ws->vseg_count].is_hv  = pad->is_hv;
                ws->vseg_count++;
            }
        }
    }

    int32_t total_wire_nm    = 0;
    int32_t crit_delay_sum   = 0;
    int32_t max_single_delay = 0;

    for (int n = 0; n < ws->num_nets; ++n) {
        ele_fab_net_t *net = &ws->nets[n];
        if (net->num_pins < 2) {
            continue;
        }

        int32_t x_min = 0x3FFFFFFF, x_max = -0x3FFFFFFF;
        int64_t y_sum = 0;
        for (int p = 0; p < net->num_pins; ++p) {
            const ele_fab_node_t *nd = &ws->nodes[net->pin_node[p]];
            int32_t px, py;
            if (nd->is_pad) {
                py = nd->y0_nm + P;
                if (nd->pad_edge == 1) {
                    px = ws->core_x0 + (core_cols + 4 + 2 * nd->row) * P;
                } else if (nd->pad_edge == 3) {
                    px = ws->core_x0 - (4 + 2 * nd->row) * P;
                } else {
                    px = nd->x0_nm + P;
                }
            } else {
                px = nd->x0_nm + (1 + net->pin_slot[p]) * P;
                py = nd->y0_nm + 4 * P;
            }
            if (px < x_min) {
                x_min = px;
            }
            if (px > x_max) {
                x_max = px;
            }
            y_sum += (int64_t)py;
        }
        int32_t y_cent = (int32_t)(y_sum / (int64_t)net->num_pins);

        int32_t best_trk  = 0;
        int32_t best_dist = 0x3FFFFFFF;
        const int32_t x_guard = 12 * P;
        for (int t = 0; t < num_global_tracks; ++t) {
            bool conflict = false;
            for (int s = 0; s < ws->m3_seg_count; ++s) {
                int32_t td = ele_fab_i32_abs(t - ws->m3_seg_trk[s]);
                int32_t req_td = (net->is_hv || ws->nets[ws->m3_seg_net[s]].is_hv) ? 2 : 1;
                if (td < req_td) {
                    if (!(x_max + x_guard < ws->m3_seg_x0[s] || ws->m3_seg_x1[s] + x_guard < x_min)) {
                        conflict = true;
                        break;
                    }
                }
            }
            if (!conflict) {
                int32_t d = ele_fab_i32_abs(global_y_tracks[t] - y_cent);
                if (d < best_dist) {
                    best_dist = d;
                    best_trk  = t;
                }
            }
        }
        net->trunk_y_track = best_trk;
        net->trunk_y_nm    = global_y_tracks[best_trk];

        int32_t min_tcol_nm = 0x3FFFFFFF, max_tcol_nm = -0x3FFFFFFF;
        int32_t net_span_nm = 0;

        for (int p = 0; p < net->num_pins; ++p) {
            const ele_fab_node_t *nd = &ws->nodes[net->pin_node[p]];
            if (nd->is_pad) {
                int32_t px = nd->x0_nm + P;
                int32_t py = nd->y0_nm + P;
                int32_t k  = nd->row;
                int32_t col, x_col, y_jog;

                if (nd->pad_edge == 1) { /* East perimeter: dedicated unique column outside core */
                    col   = core_cols + 4 + 2 * k;
                    x_col = ws->core_x0 + col * P;
                    y_jog = py;
                } else if (nd->pad_edge == 3) { /* West perimeter: dedicated unique column left of core */
                    col   = -4 - 2 * k;
                    x_col = ws->core_x0 + col * P;
                    y_jog = py;
                } else { /* North/South perimeter: dedicated horizontal M1 jog track -> core col */
                    y_jog = (nd->pad_edge == 0) ? (ws->core_y1 + (2 + 2 * k) * P)
                                                : (ws->core_y0 - (2 + 2 * k) * P);
                    int32_t pref_c = (px - ws->core_x0) / P;
                    int32_t cy_lo = ele_fab_i32_min(y_jog, net->trunk_y_nm);
                    int32_t cy_hi = ele_fab_i32_max(y_jog, net->trunk_y_nm);
                    col   = ele_fab_alloc_m2_col(ws->vsegs, ws->vseg_count, pref_c, 1, core_cols - 1,
                                                 cy_lo, cy_hi, n, net->is_hv, 3 * P);
                    x_col = ws->core_x0 + col * P;
                }

                int32_t y_lo_drop = ele_fab_i32_min(y_jog, net->trunk_y_nm);
                int32_t y_hi_drop = ele_fab_i32_max(y_jog, net->trunk_y_nm);
                if (ws->vseg_count < ELE_FAB_MAX_PINS) {
                    ws->vsegs[ws->vseg_count].col    = col;
                    ws->vsegs[ws->vseg_count].y0     = y_lo_drop - 3 * P;
                    ws->vsegs[ws->vseg_count].y1     = y_hi_drop + 3 * P;
                    ws->vsegs[ws->vseg_count].net_id = n;
                    ws->vsegs[ws->vseg_count].is_hv  = net->is_hv;
                    ws->vseg_count++;
                }
                if (x_col < min_tcol_nm) {
                    min_tcol_nm = x_col;
                }
                if (x_col > max_tcol_nm) {
                    max_tcol_nm = x_col;
                }
                net_span_nm += ele_fab_i32_abs(px - x_col) + (y_hi_drop - y_lo_drop);

                if (emit_geometry) {
                    if (ws->num_pin_locs < ELE_FAB_MAX_PINS) {
                        ele_fab_pin_loc_t *pl = &ws->pin_locs[ws->num_pin_locs++];
                        pl->node_idx = net->pin_node[p];
                        pl->net_idx  = n;
                        pl->pin_slot = 0;
                        pl->x_nm     = px + (V / 2);
                        pl->y_nm     = py + (V / 2);
                        pl->layer    = ELE_LAYER_PAD;
                    }
                    /* Pad PAD -> M3 -> VIA23 -> M2 -> VIA12 -> M1 perimeter fanout bridge -> M2 drop -> VIA23 -> M3 trunk */
                    ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, px - E, py - E, px + V + E, py + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA23, 0, px, py, px + V, py + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, px - E, py - E, px + V + E, py + V + E, n);
                    if (nd->pad_edge == 0 || nd->pad_edge == 2) {
                        int32_t py_lo = ele_fab_i32_min(py, y_jog);
                        int32_t py_hi = ele_fab_i32_max(py, y_jog);
                        ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, px, py_lo, px + W, py_hi + W, n);
                        ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, px - E, y_jog - E, px + V + E, y_jog + V + E, n);
                    }
                    /* VIA12 down to private perimeter M1 feeder track at y_jog */
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA12, 0, px, y_jog, px + V, y_jog + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, px - E, y_jog - E, px + V + E, y_jog + V + E, n);
                    int32_t jx0 = ele_fab_i32_min(px, x_col);
                    int32_t jx1 = ele_fab_i32_max(px, x_col) + W;
                    ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, jx0, y_jog, jx1, y_jog + W, n);
                    /* VIA12 back up to M2 vertical drop column x_col */
                    ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, x_col - E, y_jog - E, x_col + V + E, y_jog + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA12, 0, x_col, y_jog, x_col + V, y_jog + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col - E, y_jog - E, x_col + V + E, y_jog + V + E, n);
                    /* Vertical M2 drop onto net's M3 trunk */
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col, y_lo_drop, x_col + W, y_hi_drop + W, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col - E, net->trunk_y_nm - E,
                                      x_col + V + E, net->trunk_y_nm + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA23, 0, x_col, net->trunk_y_nm,
                                      x_col + V, net->trunk_y_nm + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, x_col - E, net->trunk_y_nm - E,
                                      x_col + V + E, net->trunk_y_nm + V + E, n);
                }
            } else {
                int32_t r  = nd->row;
                int32_t px = nd->x0_nm + (1 + net->pin_slot[p]) * P;
                int32_t pref_col = (px - ws->core_x0) / P;
                int32_t y_env_lo = ele_fab_i32_min(ws->row_y0[r], net->trunk_y_nm);
                int32_t y_env_hi = ele_fab_i32_max(ws->row_y1[r], net->trunk_y_nm);

                int32_t col = ele_fab_alloc_m2_col(ws->vsegs, ws->vseg_count, pref_col, 1, core_cols - 1,
                                                   y_env_lo, y_env_hi, n, net->is_hv, 3 * P);
                int32_t x_col = ws->core_x0 + col * P;
                int32_t stub_x0 = ele_fab_i32_min(px, x_col);
                int32_t stub_x1 = ele_fab_i32_max(px, x_col) + W;

                int32_t m1_trk = -1;
                for (int t = 0; t < 16; ++t) {
                    int32_t cand_trk = (r * 16) + ((net->pin_slot[p] * 3 + t) & 15);
                    bool ok = true;
                    for (int s = 0; s < ws->m1_seg_count; ++s) {
                        if (ws->m1_seg_trk[s] == cand_trk && ws->m1_seg_net[s] != n) {
                            if (!(stub_x1 + 3 * P < ws->m1_seg_x0[s] || ws->m1_seg_x1[s] + 3 * P < stub_x0)) {
                                ok = false;
                                break;
                            }
                        }
                    }
                    if (ok) {
                        m1_trk = cand_trk;
                        break;
                    }
                }
                if (m1_trk < 0) {
                    m1_trk = r * 16 + (n & 15);
                }
                int32_t y_stub = ws->row_y0[r] + (2 + (m1_trk & 15) * 2) * P;
                int32_t y_lo   = ele_fab_i32_min(y_stub, net->trunk_y_nm);
                int32_t y_hi   = ele_fab_i32_max(y_stub, net->trunk_y_nm);

                if (ws->vseg_count < ELE_FAB_MAX_PINS) {
                    ws->vsegs[ws->vseg_count].col    = col;
                    ws->vsegs[ws->vseg_count].y0     = y_lo - 3 * P;
                    ws->vsegs[ws->vseg_count].y1     = y_hi + 3 * P;
                    ws->vsegs[ws->vseg_count].net_id = n;
                    ws->vsegs[ws->vseg_count].is_hv  = net->is_hv;
                    ws->vseg_count++;
                }
                if (ws->m1_seg_count < ELE_FAB_MAX_PINS) {
                    int32_t s = ws->m1_seg_count++;
                    ws->m1_seg_trk[s] = m1_trk;
                    ws->m1_seg_x0[s]  = stub_x0 - 2 * P;
                    ws->m1_seg_x1[s]  = stub_x1 + 2 * P;
                    ws->m1_seg_net[s] = n;
                }

                if (x_col < min_tcol_nm) {
                    min_tcol_nm = x_col;
                }
                if (x_col > max_tcol_nm) {
                    max_tcol_nm = x_col;
                }
                net_span_nm += (stub_x1 - stub_x0) + (y_hi - y_lo);

                if (emit_geometry) {
                    if (ws->num_pin_locs < ELE_FAB_MAX_PINS) {
                        ele_fab_pin_loc_t *pl = &ws->pin_locs[ws->num_pin_locs++];
                        pl->node_idx = net->pin_node[p];
                        pl->net_idx  = n;
                        pl->pin_slot = net->pin_slot[p];
                        pl->x_nm     = x_col + (V / 2);
                        pl->y_nm     = y_stub + (V / 2);
                        pl->layer    = ELE_LAYER_M1;
                    }
                    if (stub_x1 > stub_x0) {
                        ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, stub_x0, y_stub, stub_x1, y_stub + W, n);
                    }
                    ele_fab_emit_poly(ws, ELE_LAYER_M1, 0, x_col - E, y_stub - E,
                                      x_col + V + E, y_stub + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA12, 0, x_col, y_stub, x_col + V, y_stub + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col - E, y_stub - E,
                                      x_col + V + E, y_stub + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col, y_lo, x_col + W, y_hi + W, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M2, 0, x_col - E, net->trunk_y_nm - E,
                                      x_col + V + E, net->trunk_y_nm + V + E, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_VIA23, 0, x_col, net->trunk_y_nm,
                                      x_col + V, net->trunk_y_nm + V, n);
                    ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, x_col - E, net->trunk_y_nm - E,
                                      x_col + V + E, net->trunk_y_nm + V + E, n);
                }
            }
        }

        if (max_tcol_nm < min_tcol_nm) {
            min_tcol_nm = ws->core_x0;
            max_tcol_nm = ws->core_x0 + P;
        }
        net->trunk_x0_nm = min_tcol_nm;
        net->trunk_x1_nm = max_tcol_nm + W;
        if (net->trunk_x1_nm - net->trunk_x0_nm < W) {
            net->trunk_x1_nm = net->trunk_x0_nm + W;
        }
        if (ws->m3_seg_count < ELE_FAB_MAX_NETS) {
            int32_t s = ws->m3_seg_count++;
            ws->m3_seg_trk[s] = best_trk;
            ws->m3_seg_x0[s]  = min_tcol_nm - E;
            ws->m3_seg_x1[s]  = max_tcol_nm + V + E;
            ws->m3_seg_net[s] = n;
        }

        if (emit_geometry) {
            ele_fab_emit_poly(ws, ELE_LAYER_M3, 0,
                              net->trunk_x0_nm, net->trunk_y_nm,
                              net->trunk_x1_nm, net->trunk_y_nm + W, n);
            int32_t outer_x0 = min_tcol_nm - E;
            int32_t outer_x1 = max_tcol_nm + V + E;
            ele_fab_record_line_end(ws, n, ELE_LAYER_M3, 0,
                                    outer_x0, net->trunk_y_nm - E,
                                    outer_x1, net->trunk_y_nm + V + E);
            ele_fab_record_line_end(ws, n, ELE_LAYER_M3, 1,
                                    outer_x0, net->trunk_y_nm - E,
                                    outer_x1, net->trunk_y_nm + V + E);
        }

        net_span_nm += (net->trunk_x1_nm - net->trunk_x0_nm);
        net->wire_len_nm = net_span_nm;
        total_wire_nm += net_span_nm;

        int64_t span_tracks = (int64_t)(net_span_nm / P);
        if (span_tracks < 1) {
            span_tracks = 1;
        }
        int64_t rc_prod = (int64_t)pdk->r_sheet_mohm * (int64_t)pdk->c_unit_af;
        int64_t elmore  = (rc_prod * span_tracks * span_tracks) / INT64_C(3000000) +
                          ((int64_t)pdk->c_unit_af * span_tracks) / INT64_C(120);
        net->delay_ps = (int32_t)elmore;
        if (net->is_critical) {
            crit_delay_sum += net->delay_ps;
            if (net->delay_ps > max_single_delay) {
                max_single_delay = net->delay_ps;
            }
        }
    }

    int32_t gate_buf_ps = (pdk->grid_nm * pdk->grid_nm * 8) + (pdk->pitch_nm / 2);
    crit_delay_sum += ws->repeaters_inserted * gate_buf_ps;

    int32_t row_span_um = (ws->core_x1 - ws->core_x0) / 1000;
    if (row_span_um < 1) {
        row_span_um = 1;
    }
    int64_t ir_u = ((int64_t)pdk->r_sheet_mohm * (int64_t)(row_span_um + 4)) / 100;
    flow_cell ir_st = {0ULL, 0ULL, 0ULL, 0ULL, 0ULL, 0x0002ULL << 16U, 0ULL, 0ULL};
    flow_cell ir_nx;
    flow_step_atlas(ir_nx, ir_st, pdk->chart_cells, 3ULL, ir_u * 65536, pdk->kan_lock);
    int32_t ir_drop_mv = ele_fab_i32_abs((int32_t)((int64_t)ir_nx[3] >> 16)) +
                         (pdk->r_sheet_mohm / 120) + 2;

    ws->total_wire_um  = total_wire_nm / 1000;
    ws->crit_delay_ps  = crit_delay_sum + max_single_delay / 2;
    ws->max_ir_drop_mv = ir_drop_mv;
    ws->total_energy   = (int64_t)ws->crit_delay_ps * 4 + (int64_t)ws->total_wire_um +
                         (int64_t)ws->max_ir_drop_mv * 8 + (int64_t)ws->repeaters_inserted * 40;

    if (emit_geometry) {
        for (int i = 0; i < ws->num_nodes; ++i) {
            const ele_fab_node_t *nd = &ws->nodes[i];
            if (nd->is_pad) {
                ele_fab_emit_poly(ws, ELE_LAYER_PAD, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);
                ele_fab_emit_poly(ws, ELE_LAYER_M3, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);
            } else {
                ele_fab_emit_poly(ws, ELE_LAYER_CELL_BOUND, 0, nd->x0_nm, nd->y0_nm, nd->x1_nm, nd->y1_nm, -1);
                for (int32_t gx = nd->x0_nm + P; gx + W <= nd->x1_nm - P; gx += 2 * P) {
                    ele_fab_emit_poly(ws, ELE_LAYER_POLY_GATE, 0,
                                      gx, nd->y0_nm + P, gx + W, nd->y1_nm - P, -1);
                }
            }
        }
        for (int r = 0; r <= ELE_FAB_NUM_ROWS; ++r) {
            int16_t rail_dt = (int16_t)(r & 1);
            int32_t ry0 = (r < ELE_FAB_NUM_ROWS) ? ws->row_y0[r] : ws->row_y1[ELE_FAB_NUM_ROWS - 1];
            ele_fab_emit_poly(ws, ELE_LAYER_POWER_RAIL, rail_dt,
                              ws->core_x0, ry0, ws->core_x1, ry0 + P, -1);
        }
        if ((pdk->kan_lock & 0x0800U) != 0U) {
            ele_fab_emit_poly(ws, ELE_LAYER_HV_GUARD, 0,
                              ws->core_x0 - 2 * P, ws->row_y0[3] - 2 * P,
                              ws->core_x1 + 2 * P, ws->row_y1[3] + 2 * P, -1);
        }
    }
    return ELE_FAB_OK;
}

static inline uint8_t ele_fab_decode_genome(const uint64_t canvas[4],
                                            bool allow_univalent_split,
                                            uint8_t perm_row[ELE_FAB_BASE_CELLS],
                                            int32_t target_x[ELE_FAB_BASE_CELLS]) {
    for (int i = 0; i < ELE_FAB_BASE_CELLS; ++i) {
        uint32_t r_bit  = (uint32_t)i * 2U;
        uint32_t r_word = r_bit >> 6U;
        uint32_t r_sh   = r_bit & 63U;
        uint32_t r_val  = (uint32_t)((canvas[r_word] >> r_sh) & UINT64_C(3));

        uint32_t x_bit  = 96U + (uint32_t)i * 2U;
        uint32_t x_word = x_bit >> 6U;
        uint32_t x_sh   = x_bit & 63U;
        uint32_t x_val  = (uint32_t)((canvas[x_word] >> x_sh) & UINT64_C(3));

        perm_row[i] = (uint8_t)r_val;
        target_x[i] = (i % 12) * 8 + (int32_t)x_val * 4;
    }
    if (!allow_univalent_split) {
        return 0U;
    }
    return (uint8_t)(canvas[3] & UINT64_C(0xFF));
}

static inline int ele_fab_run_coevolution(ele_fab_ws_t *ws, const ele_fab_pdk_t *pdk,
                                          bool allow_univalent_split) {
    const uint32_t n_vars = 200U;
    uint64_t init_canvas[4] = {UINT64_C(0), UINT64_C(0), UINT64_C(0), UINT64_C(0)};
    uint64_t kan_pin[4]     = {UINT64_C(0), UINT64_C(0), UINT64_C(0), UINT64_C(0xFF)};
    uint8_t  perm_row[ELE_FAB_BASE_CELLS];
    int32_t  target_x[ELE_FAB_BASE_CELLS];

    for (uint32_t i = 0U; i < (uint32_t)ELE_FAB_BASE_CELLS; ++i) {
        uint64_t r_init = (uint64_t)(i / 12U);
        uint32_t r_bit  = i * 2U;
        uint32_t r_word = r_bit >> 6U;
        uint32_t r_sh   = r_bit & 63U;
        init_canvas[r_word] |= (r_init << r_sh);
        if ((pdk->kan_lock & 0x0800U) != 0U && i >= 42U) {
            kan_pin[r_word] |= (UINT64_C(3) << r_sh);
        }
    }

    searu_result sres;
    memset(&sres, 0, sizeof(sres));
    uint64_t rng = UINT64_C(0xE1E8000F4B5E4201) ^ ((uint64_t)pdk->pitch_nm << 16U) ^
                   (uint64_t)pdk->r_sheet_mohm;

    /* Pass 1: Continuous 2-Jet + SEARU row/slot convergence (canonical gate-minimal baseline) */
    SEARU_SOLVE_INLINE(&sres, 192U, init_canvas, kan_pin, rng, 192U, cand, frust, energy, {
        uint8_t sm = ele_fab_decode_genome(cand, false, perm_row, target_x);
        ele_fab_legalize_and_route(ws, perm_row, target_x, sm, false, true);
        energy = (ws->total_energy > 0) ? (uint64_t)ws->total_energy : UINT64_C(1);
        for (int w = 0; w < 3; ++w) {
            frust[w] = UINT64_C(0xFFFFFFFFFFFFFFFF);
        }
    });

    /* Pass 2: Univalent HoTT Co-Evolution (monotonic tunneling across repeater + placement sectors) */
    if (allow_univalent_split) {
        uint64_t seed2_canvas[4];
        uint64_t pin2[4];
        for (int w = 0; w < 4; ++w) {
            seed2_canvas[w] = sres.canvas[w];
            pin2[w]         = kan_pin[w];
        }
        seed2_canvas[3] = UINT64_C(0);
        pin2[3]         = UINT64_C(0);
        searu_result sres2;
        memset(&sres2, 0, sizeof(sres2));
        SEARU_SOLVE_INLINE(&sres2, n_vars, seed2_canvas, pin2, rng ^ UINT64_C(0x9E3779B97F4A7C15),
                           128U, cand, frust, energy, {
            uint8_t sm = ele_fab_decode_genome(cand, true, perm_row, target_x);
            ele_fab_legalize_and_route(ws, perm_row, target_x, sm, false, true);
            energy = (ws->total_energy > 0) ? (uint64_t)ws->total_energy : UINT64_C(1);
            frust[0] = UINT64_C(0x00000000FFFFFFFF);
            frust[1] = UINT64_C(0);
            frust[2] = UINT64_C(0);
            frust[3] = UINT64_C(0xFF);
        });
        if (sres2.best_energy < sres.best_energy) {
            sres = sres2;
        }
    }

    uint8_t best_split_mask = ele_fab_decode_genome(sres.canvas, allow_univalent_split,
                                                    perm_row, target_x);
    ele_fab_legalize_and_route(ws, perm_row, target_x, best_split_mask, true, true);
    ws->searu_iters      = sres.iterations_run;
    ws->searu_epiphanies = sres.epiphanies;
    return ELE_FAB_OK;
}

static inline int ele_fab_compile_die(ele_fab_ws_t *ws, const ele_fab_pdk_t *pdk,
                                      bool allow_univalent_split) {
    ele_fab_build_chip_spec(ws, pdk);
    return ele_fab_run_coevolution(ws, pdk, allow_univalent_split);
}

static inline int ele_fab_compile_fir_die(ele_fab_ws_t *ws, const ele_fab_pdk_t *pdk,
                                          const flow_ir_prog_t *fir) {
    if (!ws || !pdk || !fir || fir->num_inputs == 0U || fir->num_outputs == 0U) {
        return ELE_FAB_ERR_PDK_FORMAT;
    }
    memset(ws, 0, sizeof(*ws));
    ws->pdk         = *pdk;
    ws->custom_spec = 1U;

    for (int i = 0; i < ELE_FAB_BASE_CELLS; ++i) {
        ele_fab_node_t *nd = &ws->nodes[ws->num_nodes++];
        if ((uint32_t)i < fir->num_ops) {
            uint32_t ov = fir->ops[i].out_val;
            snprintf(nd->name, sizeof(nd->name), "F_%s", fir->vals[ov].name);
        } else {
            snprintf(nd->name, sizeof(nd->name), "%s_%02d", fir->name, i);
        }
        nd->row       = i / 12;
        nd->slot_pref = i % 12;
        nd->pad_edge  = -1;
        nd->is_pad    = 0U;
        nd->is_hv     = (i >= 42) ? 1U : 0U;
        nd->power_ua  = (i < (int)fir->num_ops) ? 260 : 140;
    }

    int pad_idx = 0;
    for (uint32_t i = 0U; i < fir->num_inputs && pad_idx < ELE_FAB_NUM_PADS; ++i) {
        uint32_t vid = fir->inputs[i];
        for (uint16_t b = 0U; b < fir->vals[vid].width && pad_idx < ELE_FAB_NUM_PADS; ++b) {
            ele_fab_node_t *pad = &ws->nodes[ws->num_nodes++];
            snprintf(pad->name, sizeof(pad->name), "%s_%u", fir->vals[vid].name, (unsigned)b);
            pad->is_pad   = 1U;
            pad->is_hv    = (pad_idx >= 38) ? 1U : 0U;
            pad->pad_edge = (int8_t)(pad_idx / 11);
            pad->row      = pad_idx % 11;
            ++pad_idx;
        }
    }
    for (uint32_t i = 0U; i < fir->num_outputs && pad_idx < ELE_FAB_NUM_PADS; ++i) {
        uint32_t vid = fir->outputs[i];
        for (uint16_t b = 0U; b < fir->vals[vid].width && pad_idx < ELE_FAB_NUM_PADS; ++b) {
            ele_fab_node_t *pad = &ws->nodes[ws->num_nodes++];
            snprintf(pad->name, sizeof(pad->name), "%s_%u", fir->vals[vid].name, (unsigned)b);
            pad->is_pad   = 1U;
            pad->is_hv    = (pad_idx >= 38) ? 1U : 0U;
            pad->pad_edge = (int8_t)(pad_idx / 11);
            pad->row      = pad_idx % 11;
            ++pad_idx;
        }
    }
    while (pad_idx < ELE_FAB_NUM_PADS) {
        ele_fab_node_t *pad = &ws->nodes[ws->num_nodes++];
        snprintf(pad->name, sizeof(pad->name), "IO_%02d", pad_idx);
        pad->is_pad   = 1U;
        pad->is_hv    = (pad_idx >= 38) ? 1U : 0U;
        pad->pad_edge = (int8_t)(pad_idx / 11);
        pad->row      = pad_idx % 11;
        ++pad_idx;
    }

    ele_fab_init_base_nets(ws);
    for (int n = 0; n < 32 && (uint32_t)n < fir->num_ops; ++n) {
        const flow_ir_op_t *op = &fir->ops[n];
        if (op->num_ins > 0U) {
            int32_t dst_cell = n % 36;
            int32_t src_val  = (int32_t)op->ins[0];
            int32_t src_cell = (src_val >= (int32_t)fir->num_inputs)
                                   ? ((src_val - (int32_t)fir->num_inputs) % 36)
                                   : ((src_val * 5 + 3) % 36);
            if (src_cell != dst_cell && n < 8) {
                ws->nets[n].pin_node[0] = src_cell;
                ws->nets[n].pin_node[1] = dst_cell;
            }
        }
    }
    ws->saved_num_nets = ws->num_base_nets;
    memcpy(ws->saved_nets, ws->nets, (size_t)ws->saved_num_nets * sizeof(ws->nets[0]));

    return ele_fab_run_coevolution(ws, pdk, true);
}

/* ============================================================================
 * 4. Pure-GPR 2D Nanometer Optical Diffraction Kernel & 2-Jet OPC/ILT
 * ============================================================================ */

static inline int32_t ele_fab_litho_sigma(const ele_fab_pdk_t *pdk) {
    int32_t sigma_nm = (pdk->lambda_nm * 65) / 100;
    if (sigma_nm < pdk->grid_nm * 6) {
        sigma_nm = pdk->grid_nm * 6;
    }
    int32_t w_pad = pdk->via_size_nm + 2 * pdk->via_enc_nm;
    if (sigma_nm > w_pad) {
        sigma_nm = w_pad;
    }
    return sigma_nm;
}

/* serif_mode: -2 = ignore all OPC_SERIF (naive mask), -1 = include all OPC_SERIF, >=0 = match net */
static inline int32_t ele_fab_sample_aerial_q16(const ele_fab_poly_t *polys, int32_t npolys,
                                                int16_t layer_mask, int32_t serif_mode,
                                                int32_t feat_h, int32_t sx, int32_t sy,
                                                int32_t sigma_nm) {
    int32_t R = 2 * sigma_nm;
    if (R < 12) {
        R = 12;
    }
    int32_t step = R / 7;
    if (step < 1) {
        step = 1;
    }
    int64_t r2_max = (int64_t)R * (int64_t)R + 1;
    int64_t w_ref  = 0;
    int64_t w_lit  = 0;
    int32_t half_h = feat_h / 2;

    for (int32_t dy = -R; dy <= R; dy += step) {
        for (int32_t dx = -R; dx <= R; dx += step) {
            int64_t d2 = (int64_t)dx * (int64_t)dx + (int64_t)dy * (int64_t)dy;
            if (d2 >= r2_max) {
                continue;
            }
            int64_t kern = r2_max - d2;
            if (dy >= -half_h && dy <= half_h) {
                w_ref += kern;
            }
            int32_t px = sx + dx;
            int32_t py = sy + dy;
            for (int32_t i = 0; i < npolys; ++i) {
                const ele_fab_poly_t *p = &polys[i];
                bool match = (p->layer == layer_mask) ||
                             (p->layer == ELE_LAYER_OPC_SERIF &&
                              (serif_mode == -1 || (serif_mode >= 0 && p->net_hint == serif_mode)));
                if (!match) {
                    continue;
                }
                if (px >= p->x0 && px <= p->x1 && py >= p->y0 && py <= p->y1) {
                    w_lit += kern;
                    break;
                }
            }
        }
    }
    if (w_ref <= 0) {
        return 0;
    }
    int64_t iq = (w_lit * INT64_C(65536)) / w_ref;
    if (iq > INT64_C(65536)) {
        iq = INT64_C(65536);
    }
    return (int32_t)iq;
}

static inline int32_t ele_fab_measure_line_end_epe_nm(const ele_fab_poly_t *polys, int32_t npolys,
                                                      const ele_fab_line_end_t *le,
                                                      int32_t sigma_nm, int32_t serif_mode) {
    int32_t ym     = (le->y0 + le->y1) / 2;
    int32_t feat_h = le->y1 - le->y0;
    int16_t mask_layer = le->layer;
    int32_t max_scan = sigma_nm * 2;
    if (max_scan < 12) {
        max_scan = 12;
    }
    int32_t R_pad = 2 * sigma_nm + max_scan + 4;
    int32_t xe    = (le->dir == 0) ? le->x0 : le->x1;
    ele_fab_poly_t local_p[64];
    int32_t local_n = 0;
    for (int32_t i = 0; i < npolys && local_n < 64; ++i) {
        const ele_fab_poly_t *p = &polys[i];
        bool keep = (p->layer == mask_layer) ||
                    (p->layer == ELE_LAYER_OPC_SERIF &&
                     (serif_mode == -1 || (serif_mode >= 0 && p->net_hint == serif_mode)));
        if (!keep) {
            continue;
        }
        if (p->x1 < xe - R_pad || p->x0 > xe + R_pad ||
            p->y1 < ym - R_pad || p->y0 > ym + R_pad) {
            continue;
        }
        local_p[local_n++] = *p;
    }

    int32_t step = (max_scan >= 80) ? 4 : ((max_scan >= 30) ? 2 : 1);
    int32_t half_len = (le->x1 - le->x0) / 2;
    if (half_len < step) {
        half_len = step;
    }
    if (half_len > max_scan) {
        half_len = max_scan;
    }
    const int32_t thresh_q16 = 39321; /* 0.60 dose-to-size aerial printing threshold */

    if (le->dir == 0) {
        int32_t best_x = le->x0 + half_len;
        for (int32_t dx = -max_scan; dx <= half_len; dx += step) {
            int32_t iq = ele_fab_sample_aerial_q16(local_p, local_n, mask_layer,
                                                   serif_mode, feat_h, le->x0 + dx, ym, sigma_nm);
            if (iq >= thresh_q16) {
                best_x = le->x0 + dx;
                break;
            }
        }
        return ele_fab_i32_abs(best_x - le->x0);
    }
    int32_t best_x = le->x1 - half_len;
    for (int32_t dx = max_scan; dx >= -half_len; dx -= step) {
        int32_t iq = ele_fab_sample_aerial_q16(local_p, local_n, mask_layer,
                                               serif_mode, feat_h, le->x1 + dx, ym, sigma_nm);
        if (iq >= thresh_q16) {
            best_x = le->x1 + dx;
            break;
        }
    }
    return ele_fab_i32_abs(le->x1 - best_x);
}

static inline int32_t ele_fab_nearest_collinear_gap(const ele_fab_ws_t *ws,
                                                    const ele_fab_line_end_t *le) {
    int32_t min_gap = 1000000;
    for (int32_t i = 0; i < ws->num_polys; ++i) {
        const ele_fab_poly_t *p = &ws->polys[i];
        if (p->layer != le->layer || p->net_hint == le->net_id) {
            continue;
        }
        int32_t dy_gap = 0;
        if (p->y1 < le->y0 - ws->pdk.pitch_nm) {
            dy_gap = le->y0 - ws->pdk.pitch_nm - p->y1;
        } else if (le->y1 + ws->pdk.pitch_nm < p->y0) {
            dy_gap = p->y0 - (le->y1 + ws->pdk.pitch_nm);
        }
        if (dy_gap > 0) {
            continue;
        }
        int32_t dx_gap = 0;
        if (le->dir == 0 && p->x1 <= le->x0) {
            dx_gap = le->x0 - p->x1;
        } else if (le->dir == 1 && p->x0 >= le->x1) {
            dx_gap = p->x0 - le->x1;
        } else {
            continue;
        }
        if (dx_gap < min_gap) {
            min_gap = dx_gap;
        }
    }
    return min_gap;
}

static inline int ele_fab_apply_opc_2jet(ele_fab_ws_t *ws) {
    const ele_fab_pdk_t *pdk = &ws->pdk;
    int32_t sigma_nm = ele_fab_litho_sigma(pdk);

    int32_t base_polys = ws->num_polys;
    int32_t max_naive  = 0;
    int32_t max_opc    = 0;
    int32_t added      = 0;

    for (int32_t i = 0; i < ws->num_line_ends; ++i) {
        ele_fab_line_end_t *le = &ws->line_ends[i];
        int32_t epe_n = ele_fab_measure_line_end_epe_nm(ws->polys, base_polys, le, sigma_nm, -2);
        le->epe_naive_nm = epe_n;
        if (epe_n > max_naive) {
            max_naive = epe_n;
        }

        /* Invoke FLOW 2-Jet Chart 3 (OPC inverse lithography manifold) in dimensionless EPE/sigma units */
        int64_t u_drive_q16 = (((int64_t)epe_n * 5) * INT64_C(65536)) / ((int64_t)sigma_nm * 4);
        if (u_drive_q16 < INT64_C(16384)) {
            u_drive_q16 = INT64_C(16384);
        }
        flow_cell st = {0ULL, 0ULL, 0ULL, 0ULL, 0ULL, 0x0003ULL << 16U, 0ULL, 0ULL};
        for (int it = 0; it < 4; ++it) {
            flow_cell nx;
            flow_step_atlas(nx, st, pdk->chart_cells, 3ULL, u_drive_q16, pdk->kan_lock);
            memcpy(st, nx, sizeof(flow_cell));
        }
        int32_t bias_raw = (int32_t)(((int64_t)st[3] * (int64_t)sigma_nm) >> 16);

        int32_t mrc_gap  = ele_fab_nearest_collinear_gap(ws, le);
        le->mrc_gap_nm   = mrc_gap;
        int32_t max_safe = (mrc_gap - pdk->min_space_nm) / 3;
        int32_t ext_len  = ele_fab_i32_min(bias_raw, max_safe);
        ext_len = ele_fab_snap_down(ext_len, pdk->grid_nm);
        if (ext_len > pdk->min_space_nm) {
            ext_len = ele_fab_snap_down(pdk->min_space_nm, pdk->grid_nm);
        }
        if (ext_len <= 0) {
            le->epe_opc_nm = epe_n;
            continue;
        }
        le->opc_bias_nm = ext_len;

        int32_t pad_y = 0;
        int32_t sy0   = le->y0 - pad_y;
        int32_t sy1   = le->y1 + pad_y;
        if (le->dir == 0) {
            ele_fab_emit_poly(ws, ELE_LAYER_OPC_SERIF, 1,
                              le->x0 - ext_len, sy0, le->x0 + pdk->grid_nm, sy1, le->net_id);
        } else {
            ele_fab_emit_poly(ws, ELE_LAYER_OPC_SERIF, 1,
                              le->x1 - pdk->grid_nm, sy0, le->x1 + ext_len, sy1, le->net_id);
        }
        added++;
    }

    for (int32_t i = 0; i < ws->num_line_ends; ++i) {
        ele_fab_line_end_t *le = &ws->line_ends[i];
        le->epe_opc_nm = ele_fab_measure_line_end_epe_nm(ws->polys, ws->num_polys, le, sigma_nm, le->net_id);
        if (le->epe_opc_nm > max_opc) {
            max_opc = le->epe_opc_nm;
        }
    }

    ws->epe_naive_max_nm = max_naive;
    ws->epe_opc_max_nm   = max_opc;
    ws->opc_polys_added  = added;
    return ELE_FAB_OK;
}

/* ============================================================================
 * 5. Standard Binary GDSII v6.0 Stream Writer
 * ============================================================================ */

static inline void ele_fab_gds_str_rec(uint8_t *dst, uint32_t *pos, uint16_t rec_type,
                                       const char *str) {
    uint32_t slen = (uint32_t)strlen(str);
    uint32_t plen = (slen + 1U) & ~1U;
    ele_fab_write_be16(dst, pos, (uint16_t)(4U + plen));
    ele_fab_write_be16(dst, pos, rec_type);
    for (uint32_t i = 0U; i < plen; ++i) {
        if (*pos < ELE_FAB_MAX_GDS_BYTES) {
            dst[*pos] = (i < slen) ? (uint8_t)str[i] : 0U;
            *pos += 1U;
        }
    }
}

static inline int ele_fab_write_gds(ele_fab_ws_t *ws, const char *path) {
    uint8_t *b   = ws->gds_buf;
    uint32_t pos = 0U;

    /* HEADER v6.00 (600 = 0x0258) */
    ele_fab_write_be16(b, &pos, 6U);
    ele_fab_write_be16(b, &pos, 0x0002U);
    ele_fab_write_be16(b, &pos, 0x0258U);

    /* BGNLIB (fixed deterministic timestamp for 100% bit-exact reproducibility) */
    ele_fab_write_be16(b, &pos, 28U);
    ele_fab_write_be16(b, &pos, 0x0102U);
    static const uint16_t k_stamp[12] = {2026U, 1U, 1U, 0U, 0U, 0U, 2026U, 1U, 1U, 0U, 0U, 0U};
    for (int i = 0; i < 12; ++i) {
        ele_fab_write_be16(b, &pos, k_stamp[i]);
    }

    ele_fab_gds_str_rec(b, &pos, 0x0206U, "ELE_V8_LIB");

    /* UNITS: exact IBM Excess-64 Base-16 Real8 constants (1e-3 user / 1e-9 m = 1nm DBU) */
    ele_fab_write_be16(b, &pos, 20U);
    ele_fab_write_be16(b, &pos, 0x0305U);
    ele_fab_write_be64(b, &pos, UINT64_C(0x3E4189374BC6A7F0));
    ele_fab_write_be64(b, &pos, UINT64_C(0x3944B82FA09B5A54));

    /* BGNSTR + STRNAME */
    ele_fab_write_be16(b, &pos, 28U);
    ele_fab_write_be16(b, &pos, 0x0502U);
    for (int i = 0; i < 12; ++i) {
        ele_fab_write_be16(b, &pos, k_stamp[i]);
    }
    ele_fab_gds_str_rec(b, &pos, 0x0606U, "ele_power_qfn_top");

    for (int32_t i = 0; i < ws->num_polys; ++i) {
        const ele_fab_poly_t *p = &ws->polys[i];
        ele_fab_write_be16(b, &pos, 4U);
        ele_fab_write_be16(b, &pos, 0x0800U); /* BOUNDARY */
        ele_fab_write_be16(b, &pos, 6U);
        ele_fab_write_be16(b, &pos, 0x0D02U); /* LAYER */
        ele_fab_write_be16(b, &pos, (uint16_t)p->layer);
        ele_fab_write_be16(b, &pos, 6U);
        ele_fab_write_be16(b, &pos, 0x0E02U); /* DATATYPE */
        ele_fab_write_be16(b, &pos, (uint16_t)p->datatype);
        ele_fab_write_be16(b, &pos, 44U);
        ele_fab_write_be16(b, &pos, 0x1003U); /* XY (5 closed vertices) */
        ele_fab_write_be32(b, &pos, p->x0);
        ele_fab_write_be32(b, &pos, p->y0);
        ele_fab_write_be32(b, &pos, p->x1);
        ele_fab_write_be32(b, &pos, p->y0);
        ele_fab_write_be32(b, &pos, p->x1);
        ele_fab_write_be32(b, &pos, p->y1);
        ele_fab_write_be32(b, &pos, p->x0);
        ele_fab_write_be32(b, &pos, p->y1);
        ele_fab_write_be32(b, &pos, p->x0);
        ele_fab_write_be32(b, &pos, p->y0);
        ele_fab_write_be16(b, &pos, 4U);
        ele_fab_write_be16(b, &pos, 0x1100U); /* ENDEL */
    }

    for (int i = ELE_FAB_BASE_CELLS; i < ELE_FAB_BASE_CELLS + ELE_FAB_NUM_PADS; ++i) {
        const ele_fab_node_t *pad = &ws->nodes[i];
        ele_fab_write_be16(b, &pos, 4U);
        ele_fab_write_be16(b, &pos, 0x0C00U); /* TEXT */
        ele_fab_write_be16(b, &pos, 6U);
        ele_fab_write_be16(b, &pos, 0x0D02U); /* LAYER */
        ele_fab_write_be16(b, &pos, (uint16_t)ELE_LAYER_PAD);
        ele_fab_write_be16(b, &pos, 6U);
        ele_fab_write_be16(b, &pos, 0x1602U); /* TEXTTYPE */
        ele_fab_write_be16(b, &pos, 0U);
        ele_fab_write_be16(b, &pos, 12U);
        ele_fab_write_be16(b, &pos, 0x1003U); /* XY */
        ele_fab_write_be32(b, &pos, (pad->x0_nm + pad->x1_nm) / 2);
        ele_fab_write_be32(b, &pos, (pad->y0_nm + pad->y1_nm) / 2);
        ele_fab_gds_str_rec(b, &pos, 0x1906U, pad->name);
        ele_fab_write_be16(b, &pos, 4U);
        ele_fab_write_be16(b, &pos, 0x1100U);
    }

    ele_fab_write_be16(b, &pos, 4U);
    ele_fab_write_be16(b, &pos, 0x0700U); /* ENDSTR */
    ele_fab_write_be16(b, &pos, 4U);
    ele_fab_write_be16(b, &pos, 0x0400U); /* ENDLIB */

    ws->gds_len   = pos;
    ws->gds_fnv1a = ele_fab_fnv1a(b, pos);

    if (path != NULL) {
        FILE *fp = fopen(path, "wb");
        if (!fp) {
            return ELE_FAB_ERR_PDK_IO;
        }
        size_t wr = fwrite(b, 1U, (size_t)pos, fp);
        fclose(fp);
        if (wr != (size_t)pos) {
            return ELE_FAB_ERR_PDK_IO;
        }
    }
    return ELE_FAB_OK;
}

/* ============================================================================
 * 6. Independent Raw-Binary GDSII Parser, Exhaustive DRC & 3D Union-Find LVS
 * ============================================================================ */

static inline uint16_t ele_fab_uf_find(uint16_t *parent, uint16_t x) {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

static inline void ele_fab_uf_union(uint16_t *parent, uint16_t a, uint16_t b) {
    uint16_t ra = ele_fab_uf_find(parent, a);
    uint16_t rb = ele_fab_uf_find(parent, b);
    if (ra != rb) {
        parent[rb] = ra;
    }
}

static inline int32_t ele_fab_rect_gap(const ele_fab_poly_t *a, const ele_fab_poly_t *b) {
    int32_t gx = 0;
    if (a->x1 < b->x0) {
        gx = b->x0 - a->x1;
    } else if (b->x1 < a->x0) {
        gx = a->x0 - b->x1;
    }
    int32_t gy = 0;
    if (a->y1 < b->y0) {
        gy = b->y0 - a->y1;
    } else if (b->y1 < a->y0) {
        gy = a->y0 - b->y1;
    }
    return (gx > gy) ? gx : gy;
}

static inline bool ele_fab_rects_touch(const ele_fab_poly_t *a, const ele_fab_poly_t *b) {
    return (a->x0 <= b->x1 && b->x0 <= a->x1 && a->y0 <= b->y1 && b->y0 <= a->y1);
}

static inline bool ele_fab_via_enclosed(const ele_fab_poly_t *via, const ele_fab_poly_t *metal,
                                        int32_t enc_nm) {
    return (metal->x0 <= via->x0 - enc_nm &&
            metal->x1 >= via->x1 + enc_nm &&
            metal->y0 <= via->y0 - enc_nm &&
            metal->y1 >= via->y1 + enc_nm);
}

static inline bool ele_fab_is_conducting_layer(int16_t layer) {
    return (layer == ELE_LAYER_M1 || layer == ELE_LAYER_VIA12 ||
            layer == ELE_LAYER_M2 || layer == ELE_LAYER_VIA23 ||
            layer == ELE_LAYER_M3 || layer == ELE_LAYER_PAD ||
            layer == ELE_LAYER_OPC_SERIF);
}

static inline int ele_fab_verify_gds_bytes(ele_fab_ws_t *ws, const uint8_t *buf, uint32_t len) {
    const ele_fab_pdk_t *pdk = &ws->pdk;
    if (!buf || len < 64U) {
        return ELE_FAB_ERR_GDS_HEADER;
    }

    uint32_t pos = 0U;
    bool seen_header   = false;
    bool seen_units    = false;
    bool seen_endlib   = false;
    bool in_boundary   = false;
    bool have_layer    = false;
    bool have_dt       = false;
    bool have_xy       = false;
    ele_fab_poly_t cur = {0, 0, 0, 0, 0, 0, -1};
    ws->parsed_count   = 0;

    while (pos + 4U <= len) {
        uint16_t rlen = ele_fab_read_be16(buf + pos);
        uint16_t rtyp = ele_fab_read_be16(buf + pos + 2U);
        if (rlen < 4U || (rlen & 1U) != 0U || pos + (uint32_t)rlen > len) {
            return ELE_FAB_ERR_GDS_HEADER;
        }
        const uint8_t *pay = buf + pos + 4U;
        uint32_t plen = (uint32_t)rlen - 4U;

        if (rtyp == 0x0002U) {
            if (plen != 2U || ele_fab_read_be16(pay) != 0x0258U) {
                return ELE_FAB_ERR_GDS_HEADER;
            }
            seen_header = true;
        } else if (rtyp == 0x0305U) {
            if (plen != 16U ||
                ele_fab_read_be64(pay) != UINT64_C(0x3E4189374BC6A7F0) ||
                ele_fab_read_be64(pay + 8U) != UINT64_C(0x3944B82FA09B5A54)) {
                return ELE_FAB_ERR_GDS_UNITS;
            }
            seen_units = true;
        } else if (rtyp == 0x0800U) {
            in_boundary = true;
            have_layer  = false;
            have_dt     = false;
            have_xy     = false;
            memset(&cur, 0, sizeof(cur));
            cur.net_hint = -1;
        } else if (rtyp == 0x0D02U && in_boundary) {
            if (plen != 2U) {
                return ELE_FAB_ERR_GDS_BOUNDARY;
            }
            cur.layer  = (int16_t)ele_fab_read_be16(pay);
            have_layer = true;
        } else if (rtyp == 0x0E02U && in_boundary) {
            if (plen != 2U) {
                return ELE_FAB_ERR_GDS_BOUNDARY;
            }
            cur.datatype = (int16_t)ele_fab_read_be16(pay);
            have_dt      = true;
        } else if (rtyp == 0x1003U && in_boundary) {
            if (plen != 40U) {
                return ELE_FAB_ERR_GDS_BOUNDARY;
            }
            int32_t xs[5], ys[5];
            int32_t min_x = 0x3FFFFFFF, max_x = -0x3FFFFFFF;
            int32_t min_y = 0x3FFFFFFF, max_y = -0x3FFFFFFF;
            for (int k = 0; k < 5; ++k) {
                xs[k] = ele_fab_read_be32(pay + (uint32_t)k * 8U);
                ys[k] = ele_fab_read_be32(pay + (uint32_t)k * 8U + 4U);
                if (xs[k] < min_x) {
                    min_x = xs[k];
                }
                if (xs[k] > max_x) {
                    max_x = xs[k];
                }
                if (ys[k] < min_y) {
                    min_y = ys[k];
                }
                if (ys[k] > max_y) {
                    max_y = ys[k];
                }
            }
            if (xs[4] != xs[0] || ys[4] != ys[0] || max_x <= min_x || max_y <= min_y ||
                xs[0] >= xs[1] || ys[0] >= ys[2]) {
                return ELE_FAB_ERR_GDS_BOUNDARY;
            }
            cur.x0  = min_x;
            cur.y0  = min_y;
            cur.x1  = max_x;
            cur.y1  = max_y;
            have_xy = true;
        } else if (rtyp == 0x1100U) {
            if (in_boundary) {
                if (!have_layer || !have_dt || !have_xy) {
                    return ELE_FAB_ERR_GDS_BOUNDARY;
                }
                if (ws->parsed_count < ELE_FAB_MAX_POLYS) {
                    ws->parsed_polys[ws->parsed_count++] = cur;
                }
                in_boundary = false;
            }
        } else if (rtyp == 0x0400U) {
            seen_endlib = true;
        }
        pos += (uint32_t)rlen;
    }

    if (!seen_header || !seen_units || !seen_endlib || pos != len || in_boundary) {
        return ELE_FAB_ERR_GDS_TAIL;
    }

    const int32_t N = ws->parsed_count;
    const ele_fab_poly_t *P = ws->parsed_polys;

    /* Gate DRC-1 & DRC-2: Exact Manufacturing Grid Alignment & Minimum Width */
    for (int32_t i = 0; i < N; ++i) {
        int32_t rx0 = P[i].x0 % pdk->grid_nm;
        int32_t ry0 = P[i].y0 % pdk->grid_nm;
        int32_t rx1 = P[i].x1 % pdk->grid_nm;
        int32_t ry1 = P[i].y1 % pdk->grid_nm;
        if (rx0 != 0 || ry0 != 0 || rx1 != 0 || ry1 != 0) {
            return ELE_FAB_ERR_DRC_GRID;
        }
        int32_t w = P[i].x1 - P[i].x0;
        int32_t h = P[i].y1 - P[i].y0;
        if (P[i].layer == ELE_LAYER_M1 || P[i].layer == ELE_LAYER_M2 || P[i].layer == ELE_LAYER_M3) {
            if (w < pdk->min_width_nm || h < pdk->min_width_nm) {
                return ELE_FAB_ERR_DRC_WIDTH;
            }
        } else if (P[i].layer == ELE_LAYER_VIA12 || P[i].layer == ELE_LAYER_VIA23) {
            if (w != pdk->via_size_nm || h != pdk->via_size_nm) {
                return ELE_FAB_ERR_DRC_WIDTH;
            }
        }
    }

    /* Gate DRC-3: Standard-cell zero overlap & Power rail VDD/VSS polarity separation */
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = i + 1; j < N; ++j) {
            if (P[i].layer == ELE_LAYER_CELL_BOUND && P[j].layer == ELE_LAYER_CELL_BOUND) {
                if (ele_fab_rect_gap(&P[i], &P[j]) < pdk->min_space_nm) {
                    return ELE_FAB_ERR_DRC_CELL_OVERLAP;
                }
            } else if (P[i].layer == ELE_LAYER_POWER_RAIL && P[j].layer == ELE_LAYER_POWER_RAIL) {
                if (P[i].datatype != P[j].datatype &&
                    ele_fab_rect_gap(&P[i], &P[j]) < pdk->min_space_nm) {
                    return ELE_FAB_ERR_DRC_RAIL_SHORT;
                }
            }
        }
    }

    /* Gate DRC-4: 4-Sided Metal Landing Enclosure on every VIA12 and VIA23 cut */
    for (int32_t i = 0; i < N; ++i) {
        if (P[i].layer != ELE_LAYER_VIA12 && P[i].layer != ELE_LAYER_VIA23) {
            continue;
        }
        int16_t lo_m = (P[i].layer == ELE_LAYER_VIA12) ? ELE_LAYER_M1 : ELE_LAYER_M2;
        int16_t hi_m = (P[i].layer == ELE_LAYER_VIA12) ? ELE_LAYER_M2 : ELE_LAYER_M3;
        bool ok_lo = false, ok_hi = false;
        for (int32_t j = 0; j < N; ++j) {
            if (P[j].layer == lo_m && ele_fab_via_enclosed(&P[i], &P[j], pdk->via_enc_nm)) {
                ok_lo = true;
            }
            if (P[j].layer == hi_m && ele_fab_via_enclosed(&P[i], &P[j], pdk->via_enc_nm)) {
                ok_hi = true;
            }
            if (ok_lo && ok_hi) {
                break;
            }
        }
        if (!ok_lo || !ok_hi) {
            return ELE_FAB_ERR_DRC_VIA_ENCLOSURE;
        }
    }

    /* Reconstruct 3D Connected Electrical Components via Union-Find on raw parsed polygons */
    for (int32_t i = 0; i < N; ++i) {
        ws->uf_parent[i] = (uint16_t)i;
        ws->uf_is_hv[i]  = 0U;
    }
    for (int32_t i = 0; i < N; ++i) {
        if (!ele_fab_is_conducting_layer(P[i].layer)) {
            continue;
        }
        for (int32_t j = i + 1; j < N; ++j) {
            if (!ele_fab_is_conducting_layer(P[j].layer) || !ele_fab_rects_touch(&P[i], &P[j])) {
                continue;
            }
            int16_t la = P[i].layer;
            int16_t lb = P[j].layer;
            bool bridge = (la == lb) ||
                          (la == ELE_LAYER_VIA12 && (lb == ELE_LAYER_M1 || lb == ELE_LAYER_M2)) ||
                          (lb == ELE_LAYER_VIA12 && (la == ELE_LAYER_M1 || la == ELE_LAYER_M2)) ||
                          (la == ELE_LAYER_VIA23 && (lb == ELE_LAYER_M2 || lb == ELE_LAYER_M3)) ||
                          (lb == ELE_LAYER_VIA23 && (la == ELE_LAYER_M2 || la == ELE_LAYER_M3)) ||
                          (la == ELE_LAYER_PAD && lb == ELE_LAYER_M3) ||
                          (lb == ELE_LAYER_PAD && la == ELE_LAYER_M3) ||
                          (la == ELE_LAYER_OPC_SERIF && lb == ELE_LAYER_M3) ||
                          (lb == ELE_LAYER_OPC_SERIF && la == ELE_LAYER_M3) ||
                          (la == ELE_LAYER_OPC_SERIF && lb == ELE_LAYER_M2) ||
                          (lb == ELE_LAYER_OPC_SERIF && la == ELE_LAYER_M2);
            if (bridge) {
                ele_fab_uf_union(ws->uf_parent, (uint16_t)i, (uint16_t)j);
            }
        }
    }

    /* Gate LVS-1: Map every schematic terminal onto raw parsed polygons & check 0 opens / 0 shorts */
    uint16_t net_comp[ELE_FAB_MAX_NETS];
    for (int n = 0; n < ws->num_nets; ++n) {
        net_comp[n] = UINT16_C(0xFFFF);
    }
    int32_t pad_hit_count = 0;
    uint8_t pad_seen[ELE_FAB_NUM_PADS];
    memset(pad_seen, 0, sizeof(pad_seen));

    for (int32_t k = 0; k < ws->num_pin_locs; ++k) {
        const ele_fab_pin_loc_t *pl = &ws->pin_locs[k];
        int32_t hit_poly = -1;
        for (int32_t i = 0; i < N; ++i) {
            if (P[i].layer == pl->layer &&
                pl->x_nm >= P[i].x0 && pl->x_nm <= P[i].x1 &&
                pl->y_nm >= P[i].y0 && pl->y_nm <= P[i].y1) {
                hit_poly = i;
                break;
            }
        }
        if (hit_poly < 0) {
            return ELE_FAB_ERR_LVS_PIN_MISS;
        }
        uint16_t root = ele_fab_uf_find(ws->uf_parent, (uint16_t)hit_poly);
        if (net_comp[pl->net_idx] == UINT16_C(0xFFFF)) {
            net_comp[pl->net_idx] = root;
        } else if (net_comp[pl->net_idx] != root) {
            return ELE_FAB_ERR_LVS_OPEN;
        }
        if (ws->nets[pl->net_idx].is_hv) {
            ws->uf_is_hv[root] = 1U;
        }
        if (pl->node_idx >= ELE_FAB_BASE_CELLS &&
            pl->node_idx < ELE_FAB_BASE_CELLS + ELE_FAB_NUM_PADS) {
            int32_t pid = pl->node_idx - ELE_FAB_BASE_CELLS;
            if (pad_seen[pid] == 0U) {
                pad_seen[pid] = 1U;
                pad_hit_count++;
            }
        }
    }
    if (pad_hit_count != ELE_FAB_NUM_PADS) {
        return ELE_FAB_ERR_LVS_PAD_COUNT;
    }

    for (int a = 0; a < ws->num_nets; ++a) {
        if (net_comp[a] == UINT16_C(0xFFFF)) {
            return ELE_FAB_ERR_LVS_OPEN;
        }
        for (int b = a + 1; b < ws->num_nets; ++b) {
            if (net_comp[a] == net_comp[b]) {
                return ELE_FAB_ERR_LVS_SHORT;
            }
        }
    }

    /* Gate LVS-2: Univalent repeater buffer continuity (must have distinct driven input & output) */
    for (int i = 0; i < ws->num_nodes; ++i) {
        if (ws->nodes[i].is_repeater == 0U) {
            continue;
        }
        int32_t in_net = -1, out_net = -1;
        for (int32_t k = 0; k < ws->num_pin_locs; ++k) {
            if (ws->pin_locs[k].node_idx == i) {
                if (ws->pin_locs[k].pin_slot == 0) {
                    in_net = ws->pin_locs[k].net_idx;
                } else if (ws->pin_locs[k].pin_slot == 2) {
                    out_net = ws->pin_locs[k].net_idx;
                }
            }
        }
        if (in_net < 0 || out_net < 0) {
            return ELE_FAB_ERR_LVS_OPEN;
        }
        if (net_comp[in_net] == net_comp[out_net]) {
            return ELE_FAB_ERR_LVS_SHORT;
        }
    }

    /* Gate DRC-5: Exhaustive pairwise cross-component same-layer spacing + HV creepage + OPC MRC */
    for (int32_t i = 0; i < N; ++i) {
        if (!ele_fab_is_conducting_layer(P[i].layer)) {
            continue;
        }
        uint16_t ri = ele_fab_uf_find(ws->uf_parent, (uint16_t)i);
        uint8_t hvi = ws->uf_is_hv[ri];
        for (int32_t j = i + 1; j < N; ++j) {
            if (!ele_fab_is_conducting_layer(P[j].layer)) {
                continue;
            }
            bool same_or_opc = (P[i].layer == P[j].layer) ||
                               ((P[i].layer == ELE_LAYER_OPC_SERIF || P[j].layer == ELE_LAYER_OPC_SERIF) &&
                                (P[i].layer == ELE_LAYER_M3 || P[j].layer == ELE_LAYER_M3));
            if (!same_or_opc) {
                continue;
            }
            uint16_t rj = ele_fab_uf_find(ws->uf_parent, (uint16_t)j);
            if (ri == rj) {
                continue;
            }
            int32_t gap = ele_fab_rect_gap(&P[i], &P[j]);
            if (P[i].layer == ELE_LAYER_OPC_SERIF || P[j].layer == ELE_LAYER_OPC_SERIF) {
                if (gap < pdk->min_space_nm) {
                    return ELE_FAB_ERR_DRC_OPC_MRC;
                }
            }
            if (gap < pdk->min_space_nm) {
                return ELE_FAB_ERR_DRC_SPACE;
            }
            if ((hvi || ws->uf_is_hv[rj]) && gap < pdk->hv_space_nm) {
                return ELE_FAB_ERR_DRC_HV_SPACE;
            }
        }
    }

    /* Gate LITHO-1: Independent optical diffraction re-check directly on parsed GDSII polygons */
    int32_t opc_poly_cnt = 0;
    for (int32_t i = 0; i < N; ++i) {
        if (P[i].layer == ELE_LAYER_OPC_SERIF) {
            opc_poly_cnt++;
        }
    }
    if (opc_poly_cnt <= 0) {
        return ELE_FAB_ERR_LITHO_PULLBACK;
    }
    int32_t sigma_nm = ele_fab_litho_sigma(pdk);
    int32_t p_naive_max = 0;
    int32_t p_opc_max   = 0;
    int32_t checked_le  = (ws->num_line_ends < 8) ? ws->num_line_ends : 8;
    for (int32_t i = 0; i < checked_le; ++i) {
        int32_t en = ele_fab_measure_line_end_epe_nm(P, N, &ws->line_ends[i], sigma_nm, -2);
        int32_t eo = ele_fab_measure_line_end_epe_nm(P, N, &ws->line_ends[i], sigma_nm, -1);
        if (en > p_naive_max) {
            p_naive_max = en;
        }
        if (eo > p_opc_max) {
            p_opc_max = eo;
        }
    }
    if (p_naive_max <= 0 || p_opc_max >= p_naive_max ||
        ws->opc_polys_added <= 0 || ws->epe_opc_max_nm >= ws->epe_naive_max_nm) {
        return ELE_FAB_ERR_LITHO_PULLBACK;
    }

    return ELE_FAB_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* ELE_FAB_H */
