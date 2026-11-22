/* SPDX-License-Identifier: MIT */
#ifndef ELE_CKT_H
#define ELE_CKT_H

#include "ele_cert.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_CKT_MAX_NODES   128
#define ELE_CKT_MAX_ELEMS   256
#define ELE_CKT_MAX_MNA     224
#define ELE_CKT_MAX_AC_PTS  256
#define ELE_CKT_MAX_TRAN_PT 8192

typedef enum {
    ELE_CKT_R = 0,
    ELE_CKT_C = 1,
    ELE_CKT_L = 2,
    ELE_CKT_K = 3,
    ELE_CKT_V = 4,
    ELE_CKT_I = 5,
    ELE_CKT_D = 6,
    ELE_CKT_M = 7,
    ELE_CKT_E = 8,
    ELE_CKT_G = 9
} ele_ckt_elem_type_t;

typedef enum {
    ELE_SRC_DC = 0,
    ELE_SRC_PULSE = 1,
    ELE_SRC_SIN = 2
} ele_ckt_src_kind_t;

typedef struct {
    char name[32];
    ele_ckt_elem_type_t type;
    int n1;
    int n2;
    int n3; /* ctrl+ for E/G, gate for M */
    int n4; /* ctrl- for E/G */
    double val;
    double tol_frac; /* e.g. 0.05 for 5% */
    double ic;
    int has_ic;

    /* Mutual coupling K */
    int l1_idx;
    int l2_idx;
    char l1_name[32];
    char l2_name[32];

    /* Independent source V/I */
    ele_ckt_src_kind_t src_kind;
    double dc_val;
    double ac_mag;
    double ac_phase_deg;
    double p_v1, p_v2, p_td, p_tr, p_tf, p_pw, p_per;
    double s_vo, s_va, s_freq, s_td;

    /* Diode D */
    double d_is;
    double d_n;
    double d_rs;

    /* Switch/MOSFET M */
    double m_vth;
    double m_ron;
    double m_roff;
    double m_kp;

    /* MNA branch index (-1 if none) */
    int branch_idx;
} ele_ckt_elem_t;

typedef struct {
    int n_nodes; /* including 0 ("0"/"gnd") */
    char node_names[ELE_CKT_MAX_NODES][32];
    int n_elems;
    ele_ckt_elem_t elems[ELE_CKT_MAX_ELEMS];

    /* Directives */
    int has_op;
    int has_ac;
    int ac_is_dec;
    int ac_pts;
    double ac_f1;
    double ac_f2;
    int ac_vout_node;
    int ac_vin_elem;

    int has_tran;
    double tran_tstep;
    double tran_tstop;
    int tran_probe_node;

    int has_soa;
    int soa_node;
    double soa_vmin;
    double soa_vmax;

    int has_pdn;
    int pdn_node;
    int pdn_isrc_elem;
    double pdn_zmax;

    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];
} ele_ckt_netlist_t;

typedef struct {
    int converged;
    int nr_iters;
    double max_kcl_res;
    int n_nodes;
    double node_v[ELE_CKT_MAX_NODES];
    int n_branches;
    double branch_i[ELE_CKT_MAX_MNA];
    int is_pure_passive_r;
    double total_power_w;
} ele_ckt_dc_res_t;

typedef struct {
    int n_pts;
    double freq_hz[ELE_CKT_MAX_AC_PTS];
    double mag_db[ELE_CKT_MAX_AC_PTS];
    double phase_deg[ELE_CKT_MAX_AC_PTS];
    double z_pdn_ohm[ELE_CKT_MAX_AC_PTS];
    double bw_3db_hz;
    double z_pdn_max_ohm;
    double z_pdn_fpeak_hz;
    int pdn_pass;
} ele_ckt_ac_res_t;

typedef struct {
    int converged;
    int n_steps;
    double dt;
    double max_kcl_res;
    int probe_node;
    double probe_v_init;
    double probe_v_final;
    double probe_v_min;
    double probe_v_max;
    double node_vmin[ELE_CKT_MAX_NODES];
    double node_vmax[ELE_CKT_MAX_NODES];
    int soa_pass;
    /* Sampled trace (downsampled to <= ELE_CKT_MAX_TRAN_PT) */
    int trace_len;
    double time_s[ELE_CKT_MAX_TRAN_PT];
    double trace_v[ELE_CKT_MAX_TRAN_PT];
} ele_ckt_tran_res_t;

typedef struct {
    int run_corners;
    int n_corners;
    int soa_pass;
    double wc_vmin;
    double wc_vmax;
    double nom_vmin;
    double nom_vmax;
    ele_ckt_dc_res_t dc;
    ele_ckt_ac_res_t ac;
    ele_ckt_tran_res_t tran;
    ele_cert_record_t cert;
} ele_ckt_run_res_t;

typedef struct {
    double bit_rate_gbps;
    double ui_sec;
    double cursor_time_s;
    double pulse_peak_v;
    double xtalk_peak_v;
    double eye_pda_v;
    double eye_searu_v;
    double eye_fulltran_v;
    uint32_t worst_vic_bits;
    uint32_t worst_agg_bits;
    int eye_pass;
    ele_cert_record_t cert;
} ele_ckt_eye_res_t;

int ele_ckt_load(const char *path, ele_ckt_netlist_t *out);
int ele_ckt_find_node(const ele_ckt_netlist_t *nl, const char *name);

int ele_ckt_solve_dc(const ele_ckt_netlist_t *nl, ele_ckt_dc_res_t *out);
int ele_ckt_solve_ac(const ele_ckt_netlist_t *nl, const ele_ckt_dc_res_t *dc,
                     ele_ckt_ac_res_t *out);
int ele_ckt_solve_tran(const ele_ckt_netlist_t *nl, const ele_ckt_dc_res_t *dc,
                       ele_ckt_tran_res_t *out);
int ele_ckt_run_all(const ele_ckt_netlist_t *nl, int with_corners, ele_ckt_run_res_t *out);
int ele_ckt_si_eye(const ele_ckt_netlist_t *nl, double bit_rate_gbps, ele_ckt_eye_res_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ELE_CKT_H */
