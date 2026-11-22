/* SPDX-License-Identifier: MIT */
#ifndef ELE_FRONTIER_H
#define ELE_FRONTIER_H

#include "ele_cert.h"
#include "ele_ee.h"
#include "ele_em.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_FRONTIER_MAX_CLIQUES     256
#define ELE_FRONTIER_MAX_CLIQUE_BUS  12
#define ELE_FRONTIER_MAX_OBS         64

/* ============================================================================
 * 1. Chordal Clique Decomposition Certified AC-OPF (up to 256 Buses)
 * ============================================================================ */

typedef struct {
    int n_bus;
    int n_branch;
    int n_cliques;
    int max_clique_buses;
    int treewidth_ub;
    int converged;
    double c_lb_dual_raw;
    double c_lb_cert;
    double c_ub_primal;
    double gap_abs;
    double gap_pct;
    double elapsed_sec;
    int64_t min_clique_mu_psd;
    int64_t c_lb_int;
    int64_t c_ub_int;
    int certified;
    ele_grid_pf_res_t ub_pf;
    double gen_pg_mw[ELE_GRID_MAX_GEN];
    double gen_vset[ELE_GRID_MAX_GEN];
} ele_grid_chordal_res_t;

int ele_grid_acopf_chordal(const ele_grid_t *g, const char *cert_out_path,
                           ele_grid_chordal_res_t *out);

/* ============================================================================
 * 2. 2nd-Order Lasserre Moment-SOS AC-OPF Tightener
 * ============================================================================ */

typedef struct {
    int n_bus;
    int d2_dim;
    int n_monos4;
    int converged;
    double shor1_lb_cert;
    double sos2_lb_raw;
    double sos2_lb_cert;
    double c_ub_primal;
    double shor1_gap_pct;
    double sos2_gap_pct;
    double improvement_abs;
    int64_t q0_mu_psd;
    ele_cert_record_t cert;
} ele_grid_sos2_res_t;

int ele_grid_acopf_sos2(const ele_grid_t *g, ele_grid_sos2_res_t *out);
int ele_grid_acopf_sos2_cert(const ele_grid_t *g, const char *cert_out_path, ele_grid_sos2_res_t *out);

/* ============================================================================
 * 3. Certified Grid Observable Envelope (Voltage^2 / Branch Flow)
 * ============================================================================ */

typedef enum {
    ELE_OBS_VM2 = 0,
    ELE_OBS_PFLOW = 1
} ele_grid_obs_kind_t;

typedef struct {
    ele_grid_obs_kind_t kind;
    int target_id;    /* external bus ID for VM2, 0-based branch idx for PFLOW */
    double cost_cap;  /* C_UB + eps */
    double o_primal;
    double o_lb_cert;
    double o_ub_cert;
    double width;
    double c_ub_primal;
    double c_lb_cert;
    double cost_gap_abs;
    ele_cert_record_t cert;
} ele_grid_obs_res_t;

int ele_grid_observable(const ele_grid_t *g, ele_grid_obs_kind_t kind,
                        int target_id, double cost_eps, ele_grid_obs_res_t *out);

/* ============================================================================
 * 4. Computational EM Physical Bounds (Maxwell Poynting Dual + Bode-Fano)
 * ============================================================================ */

typedef struct {
    double kl;
    double zeta;
    double xi0;
    int n_cells;
    double p_global_ub;
    double p_local_ub_raw;
    double p_local_ub_cert;
    double p_searu_lb;
    double tightening_ratio;
    double efficiency_gap_pct;
    uint32_t searu_best_mask;
    int64_t mu_psd_int;
    double bf_gamma_limit;
    double bf_s11_limit_db;
    ele_cert_record_t cert;
} ele_em_bound_res_t;

int ele_em_maxwell_bound(double kl, double zeta, double xi0, int n_cells,
                         ele_em_bound_res_t *out);

double ele_em_bode_fano_bound(double r_ohm, double c_farad,
                              double f_low_hz, double f_high_hz);

/* ============================================================================
 * 5. KYP Bounded-Real LMI Passivity Enforcement
 * ============================================================================ */

typedef struct {
    int nports;
    int n_state;
    int nfreq;
    double orig_sigma_max;
    int orig_passive;
    double rep_sigma_max;
    double rep_recip_err;
    double rms_repair_err;
    double kyp_gamma_sq;
    int64_t kyp_mu_psd;
    int64_t p_min_eig_int;
    int certified;
    ele_cert_record_t cert;
} ele_em_kyp_res_t;

int ele_em_enforce_passivity(const ele_snp_t *in_snp, const char *out_s2p_path,
                             ele_em_kyp_res_t *out);

/* ============================================================================
 * 6. Persistent In-Process Grid Session API (for Python ctypes EleFastSession)
 * ============================================================================ */

typedef struct ele_grid_session ele_grid_session_t;

ele_grid_session_t *ele_grid_session_create(const char *path);
void ele_grid_session_destroy(ele_grid_session_t *s);
int ele_grid_session_set_bus_limits(ele_grid_session_t *s, int bus_id, double vmin, double vmax);
int ele_grid_session_set_branch_limits(ele_grid_session_t *s, int branch_idx,
                                       double smax_mva, double shift_deg,
                                       double angmin_deg, double angmax_deg);
int ele_grid_session_solve_opf(ele_grid_session_t *s, ele_grid_opf_res_t *out);
int ele_grid_session_solve_sos2(ele_grid_session_t *s, ele_grid_sos2_res_t *out);
int ele_grid_session_solve_obs(ele_grid_session_t *s, int kind, int target_id,
                               double eps, ele_grid_obs_res_t *out);

/* ============================================================================
 * 7. Multi-Frequency Maxwell + 2D FDTD & Real Complex-Load Bode-Fano (v11.0.0)
 * ============================================================================ */

typedef struct {
    ele_em_bound_res_t base;
    double bw_frac;
    int n_freq_pts;
    double p_band_avg_ub;
    double p_band_avg_searu;
    int fdtd_checked;
    double fdtd_ext_power;
    int fdtd_below_ub;
} ele_em_bound_ext_res_t;

int ele_em_maxwell_bound_ext(double kl, double zeta, double xi0, int n_cells,
                             double bw_frac, int run_fdtd_check,
                             ele_em_bound_ext_res_t *out);

typedef struct {
    double z0;
    double rl;
    double cl_f;
    double f_lo_hz;
    double f_hi_hz;
    double l1_nh;
    double c1_pf;
    double l2_nh;
    double c2_pf;
    double bf_gamma_limit;
    double bf_s11_limit_db;
    double max_gamma;
    double max_s11_db;
    double margin_db;
    double integral_meas;
    double integral_bf_max;
    double integral_ratio;
    int n_bins;
    int64_t bin_resp_int[32];
    int64_t bin_cap_int;
    int64_t i_meas_int;
    int64_t i_bf_int;
    int certified;
    ele_cert_record_t cert;
} ele_em_bode_fano_res_t;

int ele_em_bode_fano_synth(double z0, double rl, double cl_f,
                           double f_lo_hz, double f_hi_hz, uint64_t seed,
                           ele_em_bode_fano_res_t *out);

int ele_em_bode_fano_emit_lean(const char *out_path, const char *mod_name,
                               const ele_em_bode_fano_res_t *res);

/* ============================================================================
 * 8. KYP Time-Domain Wave/Lyapunov Instability Contrast Simulation (v11.0.0)
 * ============================================================================ */

typedef struct {
    int nports;
    int n_state;
    int n_steps;
    double rho_load;
    double dt_s;
    double orig_sigma_max;
    double rep_sigma_max;
    double orig_peak_ratio;
    double orig_final_ratio;
    double rep_peak_ratio;
    double rep_final_ratio;
    int orig_blowup;
    int rep_stable;
    int rep_lyap_monotone;
    int certified;
    ele_cert_record_t cert;
} ele_em_kyp_sim_res_t;

int ele_em_kyp_transient_contrast(const ele_snp_t *in_snp, double rho_load,
                                  int n_steps, ele_em_kyp_sim_res_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ELE_FRONTIER_H */
