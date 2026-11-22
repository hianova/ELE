/* SPDX-License-Identifier: MIT */
#ifndef ELE_EE_H
#define ELE_EE_H

#include "ele_cert.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_GRID_MAX_BUS    256
#define ELE_GRID_MAX_BRANCH 512
#define ELE_GRID_MAX_GEN    128

/* ============================================================================
 * 1. Nonlinear Magnetic-Saturated IPMSM Machine & FOC Engine
 * ============================================================================ */

typedef struct {
    int poles;          /* total poles (p = poles / 2 pole pairs) */
    double rs_ohm;
    double ld0_h;
    double lq0_h;
    double psi_pm0_wb;
    double sat_d;       /* d-axis saturation coefficient (1/A^2) */
    double sat_q;       /* q-axis saturation coefficient (1/A^2) */
    double sat_cross;   /* dq cross-saturation coupling */
    double alpha_pm;    /* magnet thermal coefficient (1/degC, e.g. -0.0012) */
    double i_max_a;
    double v_dc_v;
    double j_kgm2;
    double b_viscous;
    double k_core;      /* Steinmetz eddy/hysteresis core-loss factor */
    double temp_c;
    double rpm_max;
    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];
} ele_motor_spec_t;

typedef struct {
    double rpm;
    double omega_e;
    double id_a;
    double iq_a;
    double i_mag_a;
    double vd_v;
    double vq_v;
    double v_mag_v;
    double v_max_v;
    double psi_d_wb;
    double psi_q_wb;
    double torque_nm;
    double p_mech_w;
    double p_cu_w;
    double p_core_w;
    double efficiency;
    int in_current_circle;
    int in_voltage_ellipse;
    int field_weakening;
    ele_cert_record_t cert;
} ele_motor_op_res_t;

typedef struct {
    int n_pts;
    double rpm[32];
    double max_torque_nm[32];
    double power_w[32];
    double efficiency[32];
    int foc_converged;
    double foc_final_torque_nm;
    double foc_peak_i_a;
    double foc_final_rpm;
} ele_motor_sweep_res_t;

int ele_motor_load(const char *path, ele_motor_spec_t *out);
void ele_motor_eval_flux(const ele_motor_spec_t *m, double id, double iq,
                         double *psi_d, double *psi_q);
int ele_motor_solve_op(const ele_motor_spec_t *m, double rpm, double torque_ref,
                       ele_motor_op_res_t *out);
int ele_motor_sweep_and_foc(const ele_motor_spec_t *m, double torque_cmd,
                            ele_motor_sweep_res_t *out);

/* ============================================================================
 * 2. Multi-Bus Power Grid (AC-PF, N-1, Certified AC-OPF, EMT Swing)
 * ============================================================================ */

typedef struct {
    int id;         /* external bus ID */
    int type;       /* 1=PQ, 2=PV, 3=Slack */
    double pd_mw;
    double qd_mvar;
    double gs_mw;
    double bs_mvar;
    double vm_init;
    double va_init_deg;
    double vmin;
    double vmax;
} ele_grid_bus_t;

typedef struct {
    int bus_id;
    int bus_idx;
    double pg_mw;
    double qg_mvar;
    double pmin_mw;
    double pmax_mw;
    double qmin_mvar;
    double qmax_mvar;
    double vset;
    double c2;
    double c1;
    double c0;
    double h_sec;
    double xdp_pu;
    double d_pu;
    int is_gfm;
} ele_grid_gen_t;

typedef struct {
    int from_id;
    int to_id;
    int from_idx;
    int to_idx;
    double r_pu;
    double x_pu;
    double b_sh_pu;
    double tap;
    double shift_deg;
    double smax_mva;
    double angmin_deg;
    double angmax_deg;
} ele_grid_branch_t;

typedef struct {
    int n;
    int nnz;
    int *row_ptr;
    int *col_idx;
    double *g_val;
    double *b_val;
} ele_grid_csr_t;

typedef struct {
    double base_mva;
    int n_bus;
    int n_gen;
    int n_branch;
    int slack_idx;
    ele_grid_bus_t bus[ELE_GRID_MAX_BUS];
    ele_grid_gen_t gen[ELE_GRID_MAX_GEN];
    ele_grid_branch_t branch[ELE_GRID_MAX_BRANCH];
    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];
} ele_grid_t;

typedef struct {
    int converged;
    int nr_iters;
    double max_mismatch_pu;
    double total_p_gen_mw;
    double total_q_gen_mvar;
    double total_p_load_mw;
    double total_p_loss_mw;
    double vmin_pu;
    double vmax_pu;
    double max_branch_loading;
    int limits_ok;
    double vm[ELE_GRID_MAX_BUS];
    double va_rad[ELE_GRID_MAX_BUS];
    double p_inj_pu[ELE_GRID_MAX_BUS];
    double q_inj_pu[ELE_GRID_MAX_BUS];
    double branch_s_mva[ELE_GRID_MAX_BRANCH];
} ele_grid_pf_res_t;

typedef struct {
    int n_contingencies;
    int n_islands;
    int n_converged;
    int n_violations;
    int worst_branch_idx;
    double worst_vmin;
    double worst_vmax;
    double worst_loading;
    int n1_secure;
} ele_grid_n1_res_t;

typedef struct {
    int converged;
    double c_lb_dual_raw;
    double c_lb_cert;
    double c_ub_primal;
    double gap_abs;
    double gap_pct;
    int64_t mu_psd_int;
    int64_t delta_round_int;
    double gen_pg_mw[ELE_GRID_MAX_GEN];
    double gen_vset[ELE_GRID_MAX_GEN];
    ele_grid_pf_res_t ub_pf;
    ele_cert_record_t cert;
} ele_grid_opf_res_t;

typedef struct {
    int fault_bus_idx;
    double t_clear_sec;
    double max_delta_coi_deg;
    double final_delta_spread_deg;
    double v_kin;
    double v_pot;
    double v_post;
    double v_crit;
    double stability_margin;
    double cct_sec;
    double max_gfm_i_pu;
    int stable;
    ele_cert_record_t cert;
} ele_grid_emt_res_t;

int ele_grid_load(const char *path, ele_grid_t *out);
int ele_grid_build_ybus(const ele_grid_t *g, int skip_branch_idx,
                        double *G_out, double *B_out);
int ele_grid_build_ybus_csr(const ele_grid_t *g, int skip_branch_idx,
                            ele_grid_csr_t *csr_out);
void ele_grid_csr_free(ele_grid_csr_t *csr);
int ele_grid_acpf(const ele_grid_t *g, int skip_branch_idx, ele_grid_pf_res_t *out);
int ele_grid_n1(const ele_grid_t *g, ele_grid_n1_res_t *out);
int ele_grid_acopf(const ele_grid_t *g, ele_grid_opf_res_t *out);
int ele_grid_emt(const ele_grid_t *g, int fault_bus_id, double t_clear_sec,
                 int trip_branch_idx, ele_grid_emt_res_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ELE_EE_H */
