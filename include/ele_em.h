/* SPDX-License-Identifier: MIT */
#ifndef ELE_EM_H
#define ELE_EM_H

#include "ele_cert.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_EM_MAX_FREQ   512
#define ELE_EM_MAX_PORTS  8

typedef struct {
    int nports;
    int nfreq;
    double z0;
    double freq_hz[ELE_EM_MAX_FREQ];
    /* Row-major complex S[m][i*nports + j] */
    double sr[ELE_EM_MAX_FREQ][ELE_EM_MAX_PORTS * ELE_EM_MAX_PORTS];
    double si[ELE_EM_MAX_FREQ][ELE_EM_MAX_PORTS * ELE_EM_MAX_PORTS];
    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];
} ele_snp_t;

typedef struct {
    int passive_certified;     /* 1 iff min_mu_cert >= 0 across all freqs */
    int reciprocal_certified;  /* 1 iff max_recip_err <= 1e-3 */
    int worst_freq_idx;
    double worst_freq_hz;
    double sigma_max;
    double min_eig_passive;    /* min eig of (I - S^H S) */
    double max_recip_err;
    int64_t min_mu_psd;
    int64_t worst_delta_q;
    int64_t min_mu_cert;
    ele_cert_record_t cert;
} ele_em_passivity_res_t;

typedef struct {
    int nx;
    int ny;
    int nt;
    double dx_m;
    double dy_m;
    double dt_s;
    double slab_er;
    double slab_sigma;
    double fc_hz;
    int cfl_ok;
    int64_t c_dt_sq_scaled;
    int64_t dx_sq_scaled;
    int64_t dy_sq_scaled;
    double min_s11_db;
    double max_s21_db;
    double max_energy_sum;     /* max(|S11|^2 + |S21|^2) */
    ele_snp_t snp;
    ele_em_passivity_res_t passivity;
} ele_em_fdtd_res_t;

typedef struct {
    double length_m;
    double width_m;
    double thickness_m;
    double sep_m;
    double er;
    double sigma_s_m;
    double f_start_hz;
    double f_stop_hz;
    int n_freq;
    double z0_ohm;
    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];
} ele_peec_spec_t;

typedef struct {
    double l_self_h;
    double m_12_h;
    double l_loop_h;
    double c_bus_f;
    double r_dc_ohm;
    double r_ac_fmax_ohm;
    double z0_char_ohm;
    ele_snp_t snp;
    ele_em_passivity_res_t passivity;
} ele_em_peec_res_t;

typedef struct {
    int n_elem;
    double theta0_deg;
    int weights[16];
    double baseline_sll_db;
    double nominal_sll_db;
    double worst_tol_sll_db;
} ele_em_array_res_t;

typedef struct {
    double z0_ohm;
    double zl_ohm;
    double f_low_hz;
    double f_high_hz;
    double z_sec[3];
    double baseline_s11_db;
    double nominal_s11_db;
    double worst_tol_s11_db;
} ele_em_match_res_t;

/* Touchstone .sNp parser & writer */
int ele_snp_load(const char *path, int nports_hint, ele_snp_t *out);
int ele_snp_save(const char *path, const ele_snp_t *snp);

/* Zero-FP-Trust Passivity & Reciprocity Certifier */
int ele_em_certify_snp(const ele_snp_t *snp, ele_em_passivity_res_t *out);

/* 2D TMz Yee FDTD Waveguide/Dielectric-Slab Solver */
int ele_em_fdtd_run(double slab_er, double slab_sigma, int nx, int ny, int nt,
                    const char *out_s2p_path, ele_em_fdtd_res_t *out);

/* PEEC Quasi-Static Busbar Extractor */
int ele_em_peec_load(const char *path, ele_peec_spec_t *out);
int ele_em_peec_extract(const ele_peec_spec_t *spec, const char *out_s2p_path,
                        ele_em_peec_res_t *out);

/* SEARU x FLOW 2-Jet EM Inverse Synthesis */
int ele_em_synth_array(uint64_t seed, ele_em_array_res_t *out);
int ele_em_synth_match(uint64_t seed, ele_em_match_res_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ELE_EM_H */
