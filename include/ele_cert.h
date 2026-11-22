/* SPDX-License-Identifier: MIT */
#ifndef ELE_CERT_H
#define ELE_CERT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELE_VERSION_STRING "11.0.0"
#define ELE_CERT_MAX_DIM   64
#define ELE_CERT_MAX_PORTS 8
#define ELE_CERT_PATH_MAX  512

typedef struct {
    int dim;
    int64_t scale_q;
    int64_t mu_psd;
    int64_t delta_round;
    int64_t mu_cert;
    int ok;
    int64_t A[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    int64_t L[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
} ele_psd_cert_t;

typedef struct {
    char pillar[16];       /* "em", "ckt", "motor", "grid" */
    char kind[32];         /* e.g. "passivity", "soa", "ipmsm", "acopf", "emt" */
    char status[16];       /* "CERTIFIED" or "VETOED" */
    char input_path[ELE_CERT_PATH_MAX];
    char input_sha256[65];

    /* Optional Cholesky+SDD matrix witness */
    int has_psd;
    ele_psd_cert_t psd;

    /* Optional S-parameter passivity block witness (N ports at scale Q) */
    int has_sparam;
    int nports;
    int64_t s_scale_q;
    int64_t R_q[ELE_CERT_MAX_PORTS * ELE_CERT_MAX_PORTS];
    int64_t X_q[ELE_CERT_MAX_PORTS * ELE_CERT_MAX_PORTS];
    int64_t recip_eps_q;
    double worst_freq_hz;
    double sigma_max;
    double max_recip_err;

    /* Optional 1D Interval / SOA / Eye / Bracket witness */
    int has_bracket;
    int64_t bracket_scale;
    int64_t lb_int;
    int64_t ub_int;
    int64_t limit_lo_int;
    int64_t limit_hi_int;
    double lb_val;
    double ub_val;

    /* Optional Motor integer operating point */
    int has_motor;
    int64_t motor_scale;
    int64_t id_int;
    int64_t iq_int;
    int64_t vd_int;
    int64_t vq_int;
    int64_t psid_int;
    int64_t psiq_int;
    int64_t imax_int;
    int64_t vmax_int;
    int64_t two_p;
    int64_t torque_int;

    /* Optional Grid AC-OPF / EMT extra integer witness */
    int has_opf;
    int64_t opf_scale;
    int64_t c_dual_int;
    int64_t w_max_int;
    int64_t opf_delta_round;
    int64_t c_lb_int;
    int64_t c_ub_int;

    int has_emt;
    int64_t emt_scale;
    int64_t v_kin_int;
    int64_t v_pot_int;
    int64_t v_crit_int;
    int64_t v_err_int;
    double cct_sec;
    double t_clear_sec;
} ele_cert_record_t;

/* FIPS-180-4 SHA-256 */
void ele_sha256_bytes(const uint8_t *data, size_t len, char hex_out[65]);
int ele_sha256_file(const char *path, char hex_out[65]);

/* Portable dense real & complex linear algebra */
int ele_lu_solve_real(int n, const double *A, const double *b, double *x);
int ele_lu_solve_complex(int n, const double *Ar, const double *Ai,
                         const double *br, const double *bi,
                         double *xr, double *xi);
int ele_sym_jacobi_eigs(int n, const double *A, double *evals, double *evecs);

/* Zero-FP-Trust 128-bit integer Cholesky + SDD certifier & independent verifier */
int ele_cert_cholesky_sdd_i64(int n, const int64_t *A_sym,
                              int64_t *L_out, int64_t *mu_psd_out);
int ele_verify_cholesky_sdd_i64(int n, const int64_t *A_sym,
                                const int64_t *L_tri, int64_t mu_psd);

/* Canonical .elecert.json writer & independent integer verifier */
int ele_cert_write_json(const char *out_path, const ele_cert_record_t *rec);
int ele_cert_verify_file(const char *cert_path, int verify_input_hash);

/* Standalone Lean 4 module emitters (import EleCert; 0 sorry, 0 native_decide) */
int ele_cert_emit_lean(const char *out_path, const char *mod_name,
                       const ele_cert_record_t *rec);
int ele_cert_emit_chordal_lean(const char *cert_json_path,
                               const char *lean_out_path,
                               const char *mod_name);

#ifdef __cplusplus
}
#endif

#endif /* ELE_CERT_H */
