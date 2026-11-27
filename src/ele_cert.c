/* SPDX-License-Identifier: MIT */
#include "ele_cert.h"
#include "flow_cert_linalg.h"
#include "flow_cholesky_sdd.h"
#include "flow_ir_wire.h"
#include "flow_sha256_io.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef ELE_USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

/* ============================================================================
 * 1. FIPS-180-4 SHA-256 (`FLOW/sdk/flow_sha256_io.h`)
 * ============================================================================ */

void ele_sha256_bytes(const uint8_t *data, size_t len, char hex_out[65]) {
    flow_sha256_bytes_hex(data, len, hex_out);
}

int ele_sha256_file(const char *path, char hex_out[65]) {
    return flow_sha256_file_hex(path, hex_out);
}

/* ============================================================================
 * 2. Portable Dense Real & Complex Linear Algebra
 * ============================================================================ */

int ele_lu_solve_real(int n, const double *A, const double *b, double *x) {
    return flow_lu_solve_real_tol(n, A, b, x, 1e-18);
}

int ele_lu_solve_complex(int n, const double *Ar, const double *Ai,
                         const double *br, const double *bi,
                         double *xr, double *xi) {
    if (n <= 0 || Ar == NULL || Ai == NULL || br == NULL || bi == NULL ||
        xr == NULL || xi == NULL) return -1;
    size_t nn = (size_t)n;
    double *Mr = (double *)malloc(nn * nn * sizeof(double));
    double *Mi = (double *)malloc(nn * nn * sizeof(double));
    double *rr = (double *)malloc(nn * sizeof(double));
    double *ri = (double *)malloc(nn * sizeof(double));
    if (Mr == NULL || Mi == NULL || rr == NULL || ri == NULL) {
        free(Mr); free(Mi); free(rr); free(ri);
        return -1;
    }
    memcpy(Mr, Ar, nn * nn * sizeof(double));
    memcpy(Mi, Ai, nn * nn * sizeof(double));
    memcpy(rr, br, nn * sizeof(double));
    memcpy(ri, bi, nn * sizeof(double));

    for (int k = 0; k < n; ++k) {
        int piv = k;
        double maxm2 = Mr[(size_t)k * nn + (size_t)k] * Mr[(size_t)k * nn + (size_t)k] +
                       Mi[(size_t)k * nn + (size_t)k] * Mi[(size_t)k * nn + (size_t)k];
        for (int i = k + 1; i < n; ++i) {
            double ar = Mr[(size_t)i * nn + (size_t)k];
            double ai = Mi[(size_t)i * nn + (size_t)k];
            double m2 = ar * ar + ai * ai;
            if (m2 > maxm2) {
                maxm2 = m2;
                piv = i;
            }
        }
        if (maxm2 < 1e-32 || !isfinite(maxm2)) {
            free(Mr); free(Mi); free(rr); free(ri);
            return -1;
        }
        if (piv != k) {
            for (int j = k; j < n; ++j) {
                double tr = Mr[(size_t)k * nn + (size_t)j];
                double ti = Mi[(size_t)k * nn + (size_t)j];
                Mr[(size_t)k * nn + (size_t)j] = Mr[(size_t)piv * nn + (size_t)j];
                Mi[(size_t)k * nn + (size_t)j] = Mi[(size_t)piv * nn + (size_t)j];
                Mr[(size_t)piv * nn + (size_t)j] = tr;
                Mi[(size_t)piv * nn + (size_t)j] = ti;
            }
            double trb = rr[k], tib = ri[k];
            rr[k] = rr[piv]; ri[k] = ri[piv];
            rr[piv] = trb; ri[piv] = tib;
        }
        double dr = Mr[(size_t)k * nn + (size_t)k];
        double di = Mi[(size_t)k * nn + (size_t)k];
        double d2 = dr * dr + di * di;
        for (int i = k + 1; i < n; ++i) {
            double ar = Mr[(size_t)i * nn + (size_t)k];
            double ai = Mi[(size_t)i * nn + (size_t)k];
            if (ar == 0.0 && ai == 0.0) continue;
            double fr = (ar * dr + ai * di) / d2;
            double fi = (ai * dr - ar * di) / d2;
            Mr[(size_t)i * nn + (size_t)k] = 0.0;
            Mi[(size_t)i * nn + (size_t)k] = 0.0;
            for (int j = k + 1; j < n; ++j) {
                double kr = Mr[(size_t)k * nn + (size_t)j];
                double ki = Mi[(size_t)k * nn + (size_t)j];
                Mr[(size_t)i * nn + (size_t)j] -= (fr * kr - fi * ki);
                Mi[(size_t)i * nn + (size_t)j] -= (fr * ki + fi * kr);
            }
            rr[i] -= (fr * rr[k] - fi * ri[k]);
            ri[i] -= (fr * ri[k] + fi * rr[k]);
        }
    }

    for (int i = n - 1; i >= 0; --i) {
        double sr = rr[i], si = ri[i];
        for (int j = i + 1; j < n; ++j) {
            double ar = Mr[(size_t)i * nn + (size_t)j];
            double ai = Mi[(size_t)i * nn + (size_t)j];
            sr -= (ar * xr[j] - ai * xi[j]);
            si -= (ar * xi[j] + ai * xr[j]);
        }
        double dr = Mr[(size_t)i * nn + (size_t)i];
        double di = Mi[(size_t)i * nn + (size_t)i];
        double d2 = dr * dr + di * di;
        xr[i] = (sr * dr + si * di) / d2;
        xi[i] = (si * dr - sr * di) / d2;
        if (!isfinite(xr[i]) || !isfinite(xi[i])) {
            free(Mr); free(Mi); free(rr); free(ri);
            return -1;
        }
    }
    free(Mr); free(Mi); free(rr); free(ri);
    return 0;
}

int ele_sym_jacobi_eigs(int n, const double *A, double *evals, double *evecs) {
    if (n <= 0 || n > ELE_CERT_MAX_DIM || A == NULL || evals == NULL) return -1;
    size_t nn = (size_t)n;
#ifdef ELE_USE_ACCELERATE
    if (n >= 10) {
        double *Z = (double *)malloc(nn * nn * sizeof(double));
        if (Z == NULL) return -1;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                /* Column-major symmetric matrix for LAPACK */
                Z[(size_t)j * nn + (size_t)i] = 0.5 * (A[(size_t)i * nn + (size_t)j] + A[(size_t)j * nn + (size_t)i]);
            }
        }
        __LAPACK_int lap_n = (__LAPACK_int)n;
        __LAPACK_int lwork = -1, liwork = -1, info = 0;
        double wq = 0.0;
        __LAPACK_int iwq = 0;
        const char *jobz = (evecs != NULL) ? "V" : "N";
        dsyevd_(jobz, "U", &lap_n, Z, &lap_n, evals, &wq, &lwork, &iwq, &liwork, &info);
        if (info == 0) {
            lwork = (__LAPACK_int)wq + 16;
            liwork = iwq + 8;
            double *wk = (double *)malloc((size_t)lwork * sizeof(double));
            __LAPACK_int *iwk = (__LAPACK_int *)malloc((size_t)liwork * sizeof(__LAPACK_int));
            if (wk != NULL && iwk != NULL) {
                dsyevd_(jobz, "U", &lap_n, Z, &lap_n, evals, wk, &lwork, iwk, &liwork, &info);
                if (info == 0) {
                    if (evecs != NULL) {
                        for (int r = 0; r < n; ++r) {
                            for (int i = 0; i < n; ++i) {
                                evecs[(size_t)i * nn + (size_t)r] = Z[(size_t)r * nn + (size_t)i];
                            }
                        }
                    }
                    free(wk); free(iwk); free(Z);
                    return 0;
                }
            }
            free(wk); free(iwk);
        }
        free(Z);
    }
#endif
    double *M = (double *)malloc(nn * nn * sizeof(double));
    double *V = (double *)malloc(nn * nn * sizeof(double));
    if (M == NULL || V == NULL) {
        free(M); free(V);
        return -1;
    }
    double scale = 0.0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double v = 0.5 * (A[(size_t)i * nn + (size_t)j] + A[(size_t)j * nn + (size_t)i]);
            M[(size_t)i * nn + (size_t)j] = v;
            V[(size_t)i * nn + (size_t)j] = (i == j) ? 1.0 : 0.0;
            if (fabs(v) > scale) scale = fabs(v);
        }
    }
    if (scale == 0.0) scale = 1.0;

    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double apq = M[(size_t)p * nn + (size_t)q];
                off += apq * apq;
            }
        }
        if (sqrt(2.0 * off) <= 1e-14 * scale) break;
        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double g = M[(size_t)p * nn + (size_t)q];
                if (fabs(g) <= 1e-16 * scale) continue;
                double app = M[(size_t)p * nn + (size_t)p];
                double aqq = M[(size_t)q * nn + (size_t)q];
                double tau = (aqq - app) / (2.0 * g);
                double t = (tau >= 0.0)
                               ? (1.0 / (tau + sqrt(1.0 + tau * tau)))
                               : (-1.0 / (-tau + sqrt(1.0 + tau * tau)));
                double c = 1.0 / sqrt(1.0 + t * t);
                double s = t * c;
                double s2 = s / (1.0 + c);

                M[(size_t)p * nn + (size_t)p] = app - t * g;
                M[(size_t)q * nn + (size_t)q] = aqq + t * g;
                M[(size_t)p * nn + (size_t)q] = 0.0;
                M[(size_t)q * nn + (size_t)p] = 0.0;

                for (int r = 0; r < n; ++r) {
                    if (r == p || r == q) continue;
                    double arp = M[(size_t)r * nn + (size_t)p];
                    double arq = M[(size_t)r * nn + (size_t)q];
                    double np = arp - s * (arq + s2 * arp);
                    double nq = arq + s * (arp - s2 * arq);
                    M[(size_t)r * nn + (size_t)p] = np;
                    M[(size_t)p * nn + (size_t)r] = np;
                    M[(size_t)r * nn + (size_t)q] = nq;
                    M[(size_t)q * nn + (size_t)r] = nq;
                }
                for (int r = 0; r < n; ++r) {
                    double vrp = V[(size_t)r * nn + (size_t)p];
                    double vrq = V[(size_t)r * nn + (size_t)q];
                    V[(size_t)r * nn + (size_t)p] = c * vrp - s * vrq;
                    V[(size_t)r * nn + (size_t)q] = s * vrp + c * vrq;
                }
            }
        }
    }

    int perm[ELE_CERT_MAX_DIM];
    for (int i = 0; i < n; ++i) {
        evals[i] = M[(size_t)i * nn + (size_t)i];
        perm[i] = i;
    }
    for (int i = 0; i < n - 1; ++i) {
        int best = i;
        for (int j = i + 1; j < n; ++j) {
            if (evals[j] < evals[best]) best = j;
        }
        if (best != i) {
            double tv = evals[i]; evals[i] = evals[best]; evals[best] = tv;
            int tp = perm[i]; perm[i] = perm[best]; perm[best] = tp;
        }
    }
    if (evecs != NULL) {
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                evecs[(size_t)r * nn + (size_t)c] = V[(size_t)r * nn + (size_t)perm[c]];
            }
        }
    }
    free(M);
    free(V);
    return 0;
}

/* ============================================================================
 * 3. Zero-FP-Trust 128-Bit Integer Cholesky + SDD Certifier
 * ============================================================================ */

__extension__ typedef __int128 ele_i128;

int ele_verify_cholesky_sdd_i64(int n, const int64_t *A_sym,
                                const int64_t *L_tri, int64_t mu_psd) {
    return flow_verify_cholesky_sdd_i64(n, ELE_CERT_MAX_DIM, A_sym, L_tri, mu_psd);
}

int ele_cert_cholesky_sdd_i64(int n, const int64_t *A_sym,
                              int64_t *L_out, int64_t *mu_psd_out) {
    return flow_cert_cholesky_sdd_host_i64(n, ELE_CERT_MAX_DIM, A_sym, L_out, mu_psd_out,
                                           ele_sym_jacobi_eigs);
}

/* ============================================================================
 * 4. Canonical .elecert.json Writer & Independent Verifier
 * ============================================================================ */

static void write_i64_matrix_json(FILE *fp, const char *name, int n, const int64_t *M, int comma) {
    fprintf(fp, "  \"%s\": [\n", name);
    for (int i = 0; i < n; ++i) {
        fprintf(fp, "    [");
        for (int j = 0; j < n; ++j) {
            fprintf(fp, "%s%" PRId64, (j > 0) ? ", " : "", M[(size_t)i * (size_t)n + (size_t)j]);
        }
        fprintf(fp, "]%s\n", (i + 1 < n) ? "," : "");
    }
    fprintf(fp, "  ]%s\n", comma ? "," : "");
}

int ele_cert_write_json(const char *out_path, const ele_cert_record_t *rec) {
    if (out_path == NULL || rec == NULL) return -1;
    FILE *fp = fopen(out_path, "w");
    if (fp == NULL) return -1;
    fprintf(fp, "{\n");
    fprintf(fp, "  \"format\": \"ELECERT-1.0\",\n");
    fprintf(fp, "  \"version\": \"%s\",\n", ELE_VERSION_STRING);
    fprintf(fp, "  \"pillar\": \"%s\",\n", rec->pillar);
    fprintf(fp, "  \"kind\": \"%s\",\n", rec->kind);
    fprintf(fp, "  \"status\": \"%s\",\n", rec->status);
    fprintf(fp, "  \"input_path\": \"%s\",\n", rec->input_path);
    fprintf(fp, "  \"input_sha256\": \"%s\",\n", rec->input_sha256);
    fprintf(fp, "  \"has_psd\": %d,\n", rec->has_psd);
    fprintf(fp, "  \"has_sparam\": %d,\n", rec->has_sparam);
    fprintf(fp, "  \"has_bracket\": %d,\n", rec->has_bracket);
    fprintf(fp, "  \"has_motor\": %d,\n", rec->has_motor);
    fprintf(fp, "  \"has_opf\": %d,\n", rec->has_opf);
    fprintf(fp, "  \"has_emt\": %d", rec->has_emt);

    if (rec->has_psd) {
        fprintf(fp, ",\n  \"dim\": %d,\n", rec->psd.dim);
        fprintf(fp, "  \"scale_q\": %" PRId64 ",\n", rec->psd.scale_q);
        fprintf(fp, "  \"mu_psd\": %" PRId64 ",\n", rec->psd.mu_psd);
        fprintf(fp, "  \"delta_round\": %" PRId64 ",\n", rec->psd.delta_round);
        fprintf(fp, "  \"mu_cert\": %" PRId64 ",\n", rec->psd.mu_cert);
        write_i64_matrix_json(fp, "A_int", rec->psd.dim, rec->psd.A, 1);
        write_i64_matrix_json(fp, "L_int", rec->psd.dim, rec->psd.L, 0);
    }
    if (rec->has_sparam) {
        fprintf(fp, ",\n  \"nports\": %d,\n", rec->nports);
        fprintf(fp, "  \"s_scale_q\": %" PRId64 ",\n", rec->s_scale_q);
        fprintf(fp, "  \"recip_eps_q\": %" PRId64 ",\n", rec->recip_eps_q);
        fprintf(fp, "  \"worst_freq_hz\": %.9e,\n", rec->worst_freq_hz);
        fprintf(fp, "  \"sigma_max\": %.12f,\n", rec->sigma_max);
        fprintf(fp, "  \"max_recip_err\": %.12e,\n", rec->max_recip_err);
        write_i64_matrix_json(fp, "R_q", rec->nports, rec->R_q, 1);
        write_i64_matrix_json(fp, "X_q", rec->nports, rec->X_q, 0);
    }
    if (rec->has_bracket) {
        fprintf(fp, ",\n  \"bracket_scale\": %" PRId64 ",\n", rec->bracket_scale);
        fprintf(fp, "  \"lb_int\": %" PRId64 ",\n", rec->lb_int);
        fprintf(fp, "  \"ub_int\": %" PRId64 ",\n", rec->ub_int);
        fprintf(fp, "  \"limit_lo_int\": %" PRId64 ",\n", rec->limit_lo_int);
        fprintf(fp, "  \"limit_hi_int\": %" PRId64 ",\n", rec->limit_hi_int);
        fprintf(fp, "  \"lb_val\": %.12g,\n", rec->lb_val);
        fprintf(fp, "  \"ub_val\": %.12g", rec->ub_val);
    }
    if (rec->has_motor) {
        fprintf(fp, ",\n  \"motor_scale\": %" PRId64 ",\n", rec->motor_scale);
        fprintf(fp, "  \"id_int\": %" PRId64 ",\n", rec->id_int);
        fprintf(fp, "  \"iq_int\": %" PRId64 ",\n", rec->iq_int);
        fprintf(fp, "  \"vd_int\": %" PRId64 ",\n", rec->vd_int);
        fprintf(fp, "  \"vq_int\": %" PRId64 ",\n", rec->vq_int);
        fprintf(fp, "  \"psid_int\": %" PRId64 ",\n", rec->psid_int);
        fprintf(fp, "  \"psiq_int\": %" PRId64 ",\n", rec->psiq_int);
        fprintf(fp, "  \"imax_int\": %" PRId64 ",\n", rec->imax_int);
        fprintf(fp, "  \"vmax_int\": %" PRId64 ",\n", rec->vmax_int);
        fprintf(fp, "  \"two_p\": %" PRId64 ",\n", rec->two_p);
        fprintf(fp, "  \"torque_int\": %" PRId64, rec->torque_int);
    }
    if (rec->has_opf) {
        fprintf(fp, ",\n  \"opf_scale\": %" PRId64 ",\n", rec->opf_scale);
        fprintf(fp, "  \"c_dual_int\": %" PRId64 ",\n", rec->c_dual_int);
        fprintf(fp, "  \"w_max_int\": %" PRId64 ",\n", rec->w_max_int);
        fprintf(fp, "  \"opf_delta_round\": %" PRId64 ",\n", rec->opf_delta_round);
        fprintf(fp, "  \"c_lb_int\": %" PRId64 ",\n", rec->c_lb_int);
        fprintf(fp, "  \"c_ub_int\": %" PRId64, rec->c_ub_int);
    }
    if (rec->has_emt) {
        fprintf(fp, ",\n  \"emt_scale\": %" PRId64 ",\n", rec->emt_scale);
        fprintf(fp, "  \"v_kin_int\": %" PRId64 ",\n", rec->v_kin_int);
        fprintf(fp, "  \"v_pot_int\": %" PRId64 ",\n", rec->v_pot_int);
        fprintf(fp, "  \"v_crit_int\": %" PRId64 ",\n", rec->v_crit_int);
        fprintf(fp, "  \"v_err_int\": %" PRId64 ",\n", rec->v_err_int);
        fprintf(fp, "  \"cct_sec\": %.9f,\n", rec->cct_sec);
        fprintf(fp, "  \"t_clear_sec\": %.9f", rec->t_clear_sec);
    }
    fprintf(fp, "\n}\n");
    int ferr = ferror(fp);
    fclose(fp);
    return (ferr == 0) ? 0 : -1;
}

static const char *find_json_key(const char *json, const char *key) {
    char pat[128];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    size_t pat_len = strlen(pat);
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        if (p == json || p[-1] == '{' || p[-1] == ',' || isspace((unsigned char)p[-1])) {
            const char *q = p + pat_len;
            while (*q != '\0' && isspace((unsigned char)*q)) ++q;
            if (*q == ':') {
                ++q;
                while (*q != '\0' && isspace((unsigned char)*q)) ++q;
                return q;
            }
        }
        p += pat_len;
    }
    return NULL;
}

static int json_get_str(const char *json, const char *key, char *out, size_t cap) {
    const char *v = find_json_key(json, key);
    if (v == NULL || *v != '"') return -1;
    ++v;
    size_t k = 0;
    while (*v != '\0' && *v != '"') {
        if (*v == '\\' && v[1] != '\0') {
            ++v;
        }
        if (k + 1U < cap) out[k++] = *v;
        ++v;
    }
    if (*v != '"') return -1;
    out[k] = '\0';
    return 0;
}

static int json_get_i64(const char *json, const char *key, int64_t *out) {
    const char *v = find_json_key(json, key);
    if (v == NULL) return -1;
    char *endp = NULL;
    long long val = strtoll(v, &endp, 10);
    if (endp == v) return -1;
    *out = (int64_t)val;
    return 0;
}

static int json_get_f64(const char *json, const char *key, double *out) {
    const char *v = find_json_key(json, key);
    if (v == NULL) return -1;
    char *endp = NULL;
    double val = strtod(v, &endp);
    if (endp == v || !isfinite(val)) return -1;
    *out = val;
    return 0;
}

static int json_get_i64_mat(const char *json, const char *key, int n, int64_t *M) {
    const char *v = find_json_key(json, key);
    if (v == NULL || *v != '[') return -1;
    ++v;
    for (int i = 0; i < n; ++i) {
        while (*v != '\0' && *v != '[') ++v;
        if (*v != '[') return -1;
        ++v;
        for (int j = 0; j < n; ++j) {
            while (*v != '\0' && (isspace((unsigned char)*v) || *v == ',')) ++v;
            char *endp = NULL;
            long long val = strtoll(v, &endp, 10);
            if (endp == v) return -1;
            M[(size_t)i * (size_t)n + (size_t)j] = (int64_t)val;
            v = endp;
        }
        while (*v != '\0' && *v != ']') ++v;
        if (*v != ']') return -1;
        ++v;
    }
    return 0;
}

static int json_get_i64_list_bounded(const char *json, const char *key,
                                     int max_n, int *out_n, int *out_vals) {
    const char *v = find_json_key(json, key);
    if (v == NULL || *v != '[') return -1;
    ++v;
    int n = 0;
    while (*v != '\0') {
        while (*v != '\0' && (isspace((unsigned char)*v) || *v == ',')) ++v;
        if (*v == ']') {
            *out_n = n;
            return 0;
        }
        if (n >= max_n) return -1;
        char *endp = NULL;
        long long val = strtoll(v, &endp, 10);
        if (endp == v) return -1;
        out_vals[n++] = (int)val;
        v = endp;
    }
    return -1;
}

#define ELE_CHORDAL_MAX_DIM     512
#define ELE_CHORDAL_MAX_CLIQUES 256
#define ELE_CHORDAL_MAX_CL_DIM  32

#define ELEC_FLAG_PSD          UINT16_C(0x0001)
#define ELEC_FLAG_CHORDAL      UINT16_C(0x0002)
#define ELEC_FLAG_SPARAM       UINT16_C(0x0004)
#define ELEC_FLAG_BRACKET      UINT16_C(0x0008)
#define ELEC_FLAG_MOTOR        UINT16_C(0x0010)
#define ELEC_FLAG_OPF          UINT16_C(0x0020)
#define ELEC_FLAG_EMT          UINT16_C(0x0040)
#define ELEC_FLAG_BRACKET_VALS UINT16_C(0x0080)
#define ELEC_FLAGS_KNOWN_MASK  UINT16_C(0x00FF)

#define ELEC_TRAILER_BYTES 40U /* 32B SHA-256 + 8B FNV-1a-64 */

typedef struct {
    int dk;
    int vars[ELE_CHORDAL_MAX_CL_DIM];
    int64_t mu_psd;
    int64_t w_max_int;
    int64_t S_int[ELE_CHORDAL_MAX_CL_DIM * ELE_CHORDAL_MAX_CL_DIM];
    int64_t L_int[ELE_CHORDAL_MAX_CL_DIM * ELE_CHORDAL_MAX_CL_DIM];
} ele_chordal_clique_t;

typedef struct {
    int r;
    int c;
    int64_t val;
} ele_sparse_entry_t;

typedef struct {
    char version[32];
    ele_cert_record_t rec;
    int has_chordal;
    int has_bracket_vals;
    int chordal_dim;
    int num_cliques;
    int max_clique_dim;
    int64_t min_clique_mu_psd;
    int n_sparse;
    int sparse_cap;
    ele_sparse_entry_t *A_sparse;
    ele_chordal_clique_t *cliques;
} ele_full_cert_t;

static void ele_full_cert_init(ele_full_cert_t *fc) {
    memset(fc, 0, sizeof(*fc));
    snprintf(fc->version, sizeof(fc->version), "%s", ELE_VERSION_STRING);
}

static void ele_full_cert_free(ele_full_cert_t *fc) {
    if (fc == NULL) return;
    free(fc->A_sparse);
    free(fc->cliques);
    fc->A_sparse = NULL;
    fc->cliques = NULL;
    fc->n_sparse = 0;
    fc->sparse_cap = 0;
}

static int parse_chordal_json(char *buf, ele_full_cert_t *fc) {
    int64_t dim64 = 0, ncl64 = 0, min_mu_rec = 0, max_cl_dim64 = 0;
    if (json_get_i64(buf, "dim", &dim64) != 0 ||
        json_get_i64(buf, "num_cliques", &ncl64) != 0 ||
        json_get_i64(buf, "min_clique_mu_psd", &min_mu_rec) != 0) {
        return -1;
    }
    if (dim64 < 2 || dim64 > ELE_CHORDAL_MAX_DIM ||
        ncl64 < 1 || ncl64 > ELE_CHORDAL_MAX_CLIQUES) {
        return -1;
    }
    fc->chordal_dim = (int)dim64;
    fc->num_cliques = (int)ncl64;
    fc->min_clique_mu_psd = min_mu_rec;
    if (json_get_i64(buf, "max_clique_dim", &max_cl_dim64) == 0 &&
        max_cl_dim64 >= 2 && max_cl_dim64 <= ELE_CHORDAL_MAX_CL_DIM) {
        fc->max_clique_dim = (int)max_cl_dim64;
    }

    const char *as = find_json_key(buf, "A_sparse");
    if (as == NULL || *as != '[') return -1;
    ++as;
    int max_entries = fc->chordal_dim * fc->chordal_dim;
    fc->sparse_cap = 256;
    fc->n_sparse = 0;
    fc->A_sparse = (ele_sparse_entry_t *)calloc((size_t)fc->sparse_cap, sizeof(ele_sparse_entry_t));
    if (fc->A_sparse == NULL) return -1;

    while (*as != '\0') {
        while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
        if (*as == ']') break;
        if (*as != '[') return -1;
        ++as;
        int64_t triple[3] = {0, 0, 0};
        for (int t = 0; t < 3; ++t) {
            while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
            char *endp = NULL;
            long long val = strtoll(as, &endp, 10);
            if (endp == as) return -1;
            triple[t] = (int64_t)val;
            as = endp;
        }
        while (*as != '\0' && isspace((unsigned char)*as)) ++as;
        if (*as != ']') return -1;
        ++as;
        if (fc->n_sparse >= max_entries) return -1;
        if (fc->n_sparse >= fc->sparse_cap) {
            int new_cap = fc->sparse_cap * 2;
            if (new_cap > max_entries) new_cap = max_entries;
            ele_sparse_entry_t *np = (ele_sparse_entry_t *)realloc(
                fc->A_sparse, (size_t)new_cap * sizeof(ele_sparse_entry_t));
            if (np == NULL) return -1;
            fc->A_sparse = np;
            fc->sparse_cap = new_cap;
        }
        if (triple[0] < 0 || triple[0] >= fc->chordal_dim ||
            triple[1] < 0 || triple[1] >= fc->chordal_dim) {
            return -1;
        }
        fc->A_sparse[fc->n_sparse].r = (int)triple[0];
        fc->A_sparse[fc->n_sparse].c = (int)triple[1];
        fc->A_sparse[fc->n_sparse].val = triple[2];
        fc->n_sparse += 1;
    }
    if (*as != ']') return -1;

    fc->cliques = (ele_chordal_clique_t *)calloc((size_t)fc->num_cliques, sizeof(ele_chordal_clique_t));
    if (fc->cliques == NULL) return -1;

    const char *cl = find_json_key(buf, "cliques");
    if (cl == NULL || *cl != '[') return -1;
    char *cur = (char *)(uintptr_t)(cl + 1);
    int computed_max_dk = 0;

    for (int c = 0; c < fc->num_cliques; ++c) {
        while (*cur != '\0' && *cur != '{') ++cur;
        if (*cur != '{') return -1;
        char *obj_start = cur;
        int depth = 0, in_str = 0;
        char *obj_end = cur;
        while (*obj_end != '\0') {
            if (*obj_end == '"') in_str = !in_str;
            else if (!in_str) {
                if (*obj_end == '{') ++depth;
                else if (*obj_end == '}') {
                    --depth;
                    if (depth == 0) break;
                }
            }
            ++obj_end;
        }
        if (*obj_end != '}') return -1;
        *obj_end = '\0';

        ele_chordal_clique_t *clq = &fc->cliques[c];
        clq->w_max_int = fc->rec.w_max_int;
        (void)json_get_i64(obj_start, "w_max_int", &clq->w_max_int);
        if (json_get_i64_list_bounded(obj_start, "vars", ELE_CHORDAL_MAX_CL_DIM,
                                      &clq->dk, clq->vars) != 0 ||
            clq->dk < 2 || clq->dk > ELE_CHORDAL_MAX_CL_DIM ||
            json_get_i64(obj_start, "mu_psd", &clq->mu_psd) != 0 ||
            json_get_i64_mat(obj_start, "S_int", clq->dk, clq->S_int) != 0 ||
            json_get_i64_mat(obj_start, "L_int", clq->dk, clq->L_int) != 0) {
            *obj_end = '}';
            return -1;
        }
        *obj_end = '}';
        cur = obj_end + 1;
        if (clq->dk > computed_max_dk) computed_max_dk = clq->dk;
    }
    if (fc->max_clique_dim == 0) fc->max_clique_dim = computed_max_dk;
    return 0;
}

static int parse_json_full_cert(char *buf, ele_full_cert_t *fc) {
    ele_full_cert_init(fc);
    char fmt[32] = {0};
    if (json_get_str(buf, "format", fmt, sizeof(fmt)) != 0 ||
        strcmp(fmt, "ELECERT-1.0") != 0) {
        return -1;
    }
    (void)json_get_str(buf, "version", fc->version, sizeof(fc->version));
    if (json_get_str(buf, "pillar", fc->rec.pillar, sizeof(fc->rec.pillar)) != 0 ||
        json_get_str(buf, "kind", fc->rec.kind, sizeof(fc->rec.kind)) != 0 ||
        json_get_str(buf, "status", fc->rec.status, sizeof(fc->rec.status)) != 0 ||
        json_get_str(buf, "input_path", fc->rec.input_path, sizeof(fc->rec.input_path)) != 0 ||
        json_get_str(buf, "input_sha256", fc->rec.input_sha256, sizeof(fc->rec.input_sha256)) != 0) {
        return -1;
    }

    int64_t tmp = 0;
    if (json_get_i64(buf, "has_psd", &tmp) == 0) fc->rec.has_psd = (int)tmp;
    if (json_get_i64(buf, "has_chordal", &tmp) == 0) fc->has_chordal = (int)tmp;
    if (json_get_i64(buf, "has_sparam", &tmp) == 0) fc->rec.has_sparam = (int)tmp;
    if (json_get_i64(buf, "has_bracket", &tmp) == 0) fc->rec.has_bracket = (int)tmp;
    if (json_get_i64(buf, "has_motor", &tmp) == 0) fc->rec.has_motor = (int)tmp;
    if (json_get_i64(buf, "has_opf", &tmp) == 0) fc->rec.has_opf = (int)tmp;
    if (json_get_i64(buf, "has_emt", &tmp) == 0) fc->rec.has_emt = (int)tmp;

    if (fc->rec.has_psd) {
        int64_t dim64 = 0;
        if (json_get_i64(buf, "dim", &dim64) != 0 || dim64 <= 0 || dim64 > ELE_CERT_MAX_DIM ||
            json_get_i64(buf, "scale_q", &fc->rec.psd.scale_q) != 0 ||
            json_get_i64(buf, "mu_psd", &fc->rec.psd.mu_psd) != 0 ||
            json_get_i64(buf, "delta_round", &fc->rec.psd.delta_round) != 0 ||
            json_get_i64(buf, "mu_cert", &fc->rec.psd.mu_cert) != 0) {
            return -1;
        }
        fc->rec.psd.dim = (int)dim64;
        fc->rec.psd.ok = (strcmp(fc->rec.status, "CERTIFIED") == 0) ? 1 : 0;
        if (json_get_i64_mat(buf, "A_int", fc->rec.psd.dim, fc->rec.psd.A) != 0 ||
            json_get_i64_mat(buf, "L_int", fc->rec.psd.dim, fc->rec.psd.L) != 0) {
            return -1;
        }
    }

    if (fc->rec.has_sparam) {
        int64_t np64 = 0;
        if (json_get_i64(buf, "nports", &np64) != 0 || np64 <= 0 || np64 > ELE_CERT_MAX_PORTS ||
            json_get_i64(buf, "s_scale_q", &fc->rec.s_scale_q) != 0 ||
            json_get_i64(buf, "recip_eps_q", &fc->rec.recip_eps_q) != 0) {
            return -1;
        }
        fc->rec.nports = (int)np64;
        (void)json_get_f64(buf, "worst_freq_hz", &fc->rec.worst_freq_hz);
        (void)json_get_f64(buf, "sigma_max", &fc->rec.sigma_max);
        (void)json_get_f64(buf, "max_recip_err", &fc->rec.max_recip_err);
        if (json_get_i64_mat(buf, "R_q", fc->rec.nports, fc->rec.R_q) != 0 ||
            json_get_i64_mat(buf, "X_q", fc->rec.nports, fc->rec.X_q) != 0) {
            return -1;
        }
    }

    if (fc->rec.has_bracket) {
        fc->rec.bracket_scale = 1;
        (void)json_get_i64(buf, "bracket_scale", &fc->rec.bracket_scale);
        if (json_get_i64(buf, "lb_int", &fc->rec.lb_int) != 0 ||
            json_get_i64(buf, "ub_int", &fc->rec.ub_int) != 0 ||
            json_get_i64(buf, "limit_lo_int", &fc->rec.limit_lo_int) != 0 ||
            json_get_i64(buf, "limit_hi_int", &fc->rec.limit_hi_int) != 0) {
            return -1;
        }
        if (json_get_f64(buf, "lb_val", &fc->rec.lb_val) == 0 &&
            json_get_f64(buf, "ub_val", &fc->rec.ub_val) == 0) {
            fc->has_bracket_vals = 1;
        }
    }

    if (fc->rec.has_motor) {
        fc->rec.motor_scale = 1;
        (void)json_get_i64(buf, "motor_scale", &fc->rec.motor_scale);
        if (json_get_i64(buf, "id_int", &fc->rec.id_int) != 0 ||
            json_get_i64(buf, "iq_int", &fc->rec.iq_int) != 0 ||
            json_get_i64(buf, "vd_int", &fc->rec.vd_int) != 0 ||
            json_get_i64(buf, "vq_int", &fc->rec.vq_int) != 0 ||
            json_get_i64(buf, "psid_int", &fc->rec.psid_int) != 0 ||
            json_get_i64(buf, "psiq_int", &fc->rec.psiq_int) != 0 ||
            json_get_i64(buf, "imax_int", &fc->rec.imax_int) != 0 ||
            json_get_i64(buf, "vmax_int", &fc->rec.vmax_int) != 0 ||
            json_get_i64(buf, "two_p", &fc->rec.two_p) != 0 ||
            json_get_i64(buf, "torque_int", &fc->rec.torque_int) != 0) {
            return -1;
        }
    }

    if (fc->rec.has_opf) {
        if (json_get_i64(buf, "opf_scale", &fc->rec.opf_scale) != 0 ||
            json_get_i64(buf, "c_dual_int", &fc->rec.c_dual_int) != 0 ||
            json_get_i64(buf, "w_max_int", &fc->rec.w_max_int) != 0 ||
            json_get_i64(buf, "opf_delta_round", &fc->rec.opf_delta_round) != 0 ||
            json_get_i64(buf, "c_lb_int", &fc->rec.c_lb_int) != 0 ||
            json_get_i64(buf, "c_ub_int", &fc->rec.c_ub_int) != 0) {
            return -1;
        }
    }

    if (fc->rec.has_emt) {
        fc->rec.emt_scale = 1;
        (void)json_get_i64(buf, "emt_scale", &fc->rec.emt_scale);
        if (json_get_i64(buf, "v_kin_int", &fc->rec.v_kin_int) != 0 ||
            json_get_i64(buf, "v_pot_int", &fc->rec.v_pot_int) != 0 ||
            json_get_i64(buf, "v_crit_int", &fc->rec.v_crit_int) != 0 ||
            json_get_i64(buf, "v_err_int", &fc->rec.v_err_int) != 0) {
            return -1;
        }
        (void)json_get_f64(buf, "cct_sec", &fc->rec.cct_sec);
        (void)json_get_f64(buf, "t_clear_sec", &fc->rec.t_clear_sec);
    }

    if (fc->has_chordal) {
        if (parse_chordal_json(buf, fc) != 0) {
            ele_full_cert_free(fc);
            return -1;
        }
    }
    return 0;
}

/* --- Canonical "ELEC" Binary Witness Wire Codec --- */

typedef struct {
    uint8_t *data;
    size_t len;
    size_t cap;
} elec_buf_t;

static int elec_buf_reserve(elec_buf_t *b, size_t extra) {
    if (b->len > SIZE_MAX - extra) return -1;
    size_t need = b->len + extra;
    if (need <= b->cap) return 0;
    size_t ncap = (b->cap == 0U) ? 512U : b->cap;
    while (ncap < need) {
        if (ncap > SIZE_MAX / 2U) return -1;
        ncap *= 2U;
    }
    uint8_t *np = (uint8_t *)realloc(b->data, ncap);
    if (np == NULL) return -1;
    b->data = np;
    b->cap = ncap;
    return 0;
}

static int elec_put_u8(elec_buf_t *b, uint8_t v) {
    if (elec_buf_reserve(b, 1U) != 0) return -1;
    b->data[b->len++] = v;
    return 0;
}

static int elec_put_le16(elec_buf_t *b, uint16_t v) {
    if (elec_buf_reserve(b, 2U) != 0) return -1;
    flow_ir_wire_write_le16(b->data + b->len, v);
    b->len += 2U;
    return 0;
}

static int elec_put_le32(elec_buf_t *b, uint32_t v) {
    if (elec_buf_reserve(b, 4U) != 0) return -1;
    flow_ir_wire_write_le32(b->data + b->len, v);
    b->len += 4U;
    return 0;
}

static int elec_put_le64(elec_buf_t *b, uint64_t v) {
    if (elec_buf_reserve(b, 8U) != 0) return -1;
    flow_ir_wire_write_le64(b->data + b->len, v);
    b->len += 8U;
    return 0;
}

static int elec_put_f64(elec_buf_t *b, double d) {
    uint64_t u = 0U;
    memcpy(&u, &d, sizeof(u));
    return elec_put_le64(b, u);
}

static size_t elec_varu64_size(uint64_t v) {
    size_t n = 1U;
    while (v >= UINT64_C(0x80)) {
        v >>= 7U;
        n += 1U;
    }
    return n;
}

static int elec_put_varu64(elec_buf_t *b, uint64_t v) {
    if (elec_buf_reserve(b, 10U) != 0) return -1;
    for (uint32_t i = 0U; i < 10U; ++i) {
        uint8_t lo = (uint8_t)(v & UINT64_C(0x7F));
        v >>= 7U;
        if (v == UINT64_C(0)) {
            b->data[b->len++] = lo;
            return 0;
        }
        b->data[b->len++] = (uint8_t)(lo | UINT8_C(0x80));
    }
    return -1;
}

static uint64_t elec_zz_enc(int64_t x) {
    return ((uint64_t)x << 1U) ^ ((x < 0) ? ~UINT64_C(0) : UINT64_C(0));
}

static int64_t elec_zz_dec(uint64_t u) {
    return (int64_t)((u >> 1U) ^ (~(u & UINT64_C(1)) + UINT64_C(1)));
}

static int elec_put_zz64(elec_buf_t *b, int64_t x) {
    return elec_put_varu64(b, elec_zz_enc(x));
}

static int elec_put_str(elec_buf_t *b, const char *s) {
    size_t slen = strlen(s);
    if (elec_put_varu64(b, (uint64_t)slen) != 0) return -1;
    if (slen > 0U) {
        if (elec_buf_reserve(b, slen) != 0) return -1;
        memcpy(b->data + b->len, s, slen);
        b->len += slen;
    }
    return 0;
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
}

static int elec_put_sha256_str(elec_buf_t *b, const char *hex_str) {
    size_t slen = strlen(hex_str);
    if (slen == 64U) {
        uint8_t raw[32];
        int all_lower_hex = 1;
        for (size_t i = 0U; i < 32U; ++i) {
            int hi = hex_nibble(hex_str[2U * i]);
            int lo = hex_nibble(hex_str[2U * i + 1U]);
            if (hi < 0 || lo < 0) {
                all_lower_hex = 0;
                break;
            }
            raw[i] = (uint8_t)(((unsigned)hi << 4U) | (unsigned)lo);
        }
        if (all_lower_hex) {
            if (elec_put_u8(b, UINT8_C(0)) != 0) return -1;
            if (elec_buf_reserve(b, 32U) != 0) return -1;
            memcpy(b->data + b->len, raw, 32U);
            b->len += 32U;
            return 0;
        }
    }
    if (elec_put_u8(b, UINT8_C(1)) != 0) return -1;
    return elec_put_str(b, hex_str);
}

static int elec_get_u8(const uint8_t *buf, size_t end, size_t *pos, uint8_t *out) {
    if (*pos >= end) return -1;
    *out = buf[(*pos)++];
    return 0;
}

static int elec_get_f64(const uint8_t *buf, size_t end, size_t *pos, double *out) {
    if (*pos + 8U > end) return -1;
    uint64_t u = flow_ir_wire_read_le64(buf + *pos);
    *pos += 8U;
    double d = 0.0;
    memcpy(&d, &u, sizeof(d));
    if (!isfinite(d)) return -1;
    *out = d;
    return 0;
}

static int elec_get_varu64(const uint8_t *buf, size_t end, size_t *pos, uint64_t *out) {
    size_t p = *pos;
    uint64_t acc = UINT64_C(0);
    uint32_t shift = 0U;
    for (uint32_t i = 0U; i < 10U; ++i) {
        if (p >= end) return -1;
        uint8_t b = buf[p++];
        if (i == 9U) {
            if (b != UINT8_C(1)) return -1;
            acc |= UINT64_C(1) << 63U;
            *pos = p;
            *out = acc;
            return 0;
        }
        if (i > 0U && b == UINT8_C(0)) return -1;
        acc |= (uint64_t)(b & UINT8_C(0x7F)) << shift;
        if ((b & UINT8_C(0x80)) == UINT8_C(0)) {
            *pos = p;
            *out = acc;
            return 0;
        }
        shift += 7U;
    }
    return -1;
}

static int elec_get_varu32(const uint8_t *buf, size_t end, size_t *pos, uint32_t *out) {
    uint64_t v = UINT64_C(0);
    if (elec_get_varu64(buf, end, pos, &v) != 0 || v > (uint64_t)UINT32_MAX) return -1;
    *out = (uint32_t)v;
    return 0;
}

static int elec_get_zz64(const uint8_t *buf, size_t end, size_t *pos, int64_t *out) {
    uint64_t u = UINT64_C(0);
    if (elec_get_varu64(buf, end, pos, &u) != 0) return -1;
    *out = elec_zz_dec(u);
    return 0;
}

static int elec_get_str(const uint8_t *buf, size_t end, size_t *pos, char *out, size_t cap) {
    uint32_t slen = 0U;
    if (elec_get_varu32(buf, end, pos, &slen) != 0) return -1;
    if ((size_t)slen >= cap || *pos + (size_t)slen > end) return -1;
    for (size_t i = 0U; i < (size_t)slen; ++i) {
        if (buf[*pos + i] == 0U) return -1;
    }
    if (slen > 0U) memcpy(out, buf + *pos, (size_t)slen);
    out[slen] = '\0';
    *pos += (size_t)slen;
    return 0;
}

static int elec_get_sha256_str(const uint8_t *buf, size_t end, size_t *pos, char out[65]) {
    uint8_t mode = 0U;
    if (elec_get_u8(buf, end, pos, &mode) != 0) return -1;
    if (mode == UINT8_C(0)) {
        if (*pos + 32U > end) return -1;
        flow_sha256_digest_to_hex(buf + *pos, out);
        *pos += 32U;
        return 0;
    }
    if (mode == UINT8_C(1)) {
        if (elec_get_str(buf, end, pos, out, 65U) != 0) return -1;
        /* Canonical rule: 64 lowercase hex chars must use mode 0 */
        if (strlen(out) == 64U) {
            int all_hex = 1;
            for (size_t i = 0U; i < 64U; ++i) {
                if (hex_nibble(out[i]) < 0) { all_hex = 0; break; }
            }
            if (all_hex) return -1;
        }
        return 0;
    }
    return -1;
}

enum {
    ELEC_MAT_SHAPE_FULL = 0,
    ELEC_MAT_SHAPE_LTRI = 1,
    ELEC_MAT_SHAPE_SYM  = 2
};

enum {
    ELEC_MAT_ENC_ZERO    = 0,
    ELEC_MAT_ENC_SPARSE  = 1,
    ELEC_MAT_ENC_BITMASK = 2
};

static int mat_is_ltri(int n, const int64_t *M) {
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (M[(size_t)i * (size_t)n + (size_t)j] != 0) return 0;
        }
    }
    return 1;
}

static int mat_is_sym(int n, const int64_t *M) {
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (M[(size_t)i * (size_t)n + (size_t)j] !=
                M[(size_t)j * (size_t)n + (size_t)i]) return 0;
        }
    }
    return 1;
}

static size_t mat_slot_rc(int n, int shape, size_t slot, int *out_r, int *out_c) {
    (void)slot; (void)n; (void)shape; (void)out_r; (void)out_c;
    return 0U;
}

static size_t mat_total_slots(int n, int shape) {
    if (shape == ELEC_MAT_SHAPE_FULL) return (size_t)n * (size_t)n;
    return ((size_t)n * (size_t)(n + 1)) / 2U;
}

static int mat_slot_val(int n, int shape, const int64_t *M, size_t slot, int *r_out, int *c_out, int64_t *v_out) {
    (void)mat_slot_rc;
    if (n <= 0 || M == NULL) return -1;
    if (shape == ELEC_MAT_SHAPE_FULL) {
        size_t base = 0U;
        size_t row_len = (size_t)n;
        for (int i = 0; i < n; ++i) {
            if (slot < base + row_len) {
                int j = (int)(slot - base);
                if (r_out) *r_out = i;
                if (c_out) *c_out = j;
                if (v_out) *v_out = M[(size_t)i * row_len + (size_t)j];
                return 0;
            }
            base += row_len;
        }
        return -1;
    }
    if (shape == ELEC_MAT_SHAPE_LTRI) {
        size_t base = 0U;
        for (int i = 0; i < n; ++i) {
            size_t row_len = (size_t)(i + 1);
            if (slot < base + row_len) {
                int j = (int)(slot - base);
                if (r_out) *r_out = i;
                if (c_out) *c_out = j;
                if (v_out) *v_out = M[(size_t)i * (size_t)n + (size_t)j];
                return 0;
            }
            base += row_len;
        }
        return -1;
    }
    if (shape == ELEC_MAT_SHAPE_SYM) {
        size_t base = 0U;
        for (int i = 0; i < n; ++i) {
            size_t row_len = (size_t)(n - i);
            if (slot < base + row_len) {
                int j = i + (int)(slot - base);
                if (r_out) *r_out = i;
                if (c_out) *c_out = j;
                if (v_out) *v_out = M[(size_t)i * (size_t)n + (size_t)j];
                return 0;
            }
            base += row_len;
        }
        return -1;
    }
    return -1;
}

static int elec_put_i64_mat(elec_buf_t *b, int n, const int64_t *M) {
    if (n <= 0 || M == NULL) return -1;
    int shape = ELEC_MAT_SHAPE_FULL;
    if (mat_is_ltri(n, M)) shape = ELEC_MAT_SHAPE_LTRI;
    else if (mat_is_sym(n, M)) shape = ELEC_MAT_SHAPE_SYM;

    size_t total = mat_total_slots(n, shape);
    size_t nnz = 0U;
    size_t sparse_idx_bytes = 0U;
    size_t next_min = 0U;

    for (size_t s = 0U; s < total; ++s) {
        int64_t v = 0;
        if (mat_slot_val(n, shape, M, s, NULL, NULL, &v) != 0) return -1;
        if (v != 0) {
            nnz += 1U;
            sparse_idx_bytes += elec_varu64_size((uint64_t)(s - next_min));
            next_min = s + 1U;
        }
    }

    if (nnz == 0U) {
        uint8_t hdr = (uint8_t)(((unsigned)shape << 2U) | (unsigned)ELEC_MAT_ENC_ZERO);
        return elec_put_u8(b, hdr);
    }

    sparse_idx_bytes += elec_varu64_size((uint64_t)nnz);
    size_t bitmap_bytes = (total + 7U) / 8U;
    int enc = (bitmap_bytes < sparse_idx_bytes) ? ELEC_MAT_ENC_BITMASK : ELEC_MAT_ENC_SPARSE;
    uint8_t hdr = (uint8_t)(((unsigned)shape << 2U) | (unsigned)enc);
    if (elec_put_u8(b, hdr) != 0) return -1;

    if (enc == ELEC_MAT_ENC_SPARSE) {
        if (elec_put_varu64(b, (uint64_t)nnz) != 0) return -1;
        size_t cur_min = 0U;
        for (size_t s = 0U; s < total; ++s) {
            int64_t v = 0;
            if (mat_slot_val(n, shape, M, s, NULL, NULL, &v) != 0) return -1;
            if (v != 0) {
                if (elec_put_varu64(b, (uint64_t)(s - cur_min)) != 0 ||
                    elec_put_zz64(b, v) != 0) return -1;
                cur_min = s + 1U;
            }
        }
        return 0;
    }

    size_t bm_start = b->len;
    if (elec_buf_reserve(b, bitmap_bytes) != 0) return -1;
    memset(b->data + bm_start, 0, bitmap_bytes);
    b->len += bitmap_bytes;
    for (size_t s = 0U; s < total; ++s) {
        int64_t v = 0;
        if (mat_slot_val(n, shape, M, s, NULL, NULL, &v) != 0) return -1;
        if (v != 0) {
            b->data[bm_start + (s >> 3U)] |= (uint8_t)(1U << (s & 7U));
            if (elec_put_zz64(b, v) != 0) return -1;
        }
    }
    return 0;
}

static int elec_get_i64_mat(const uint8_t *buf, size_t end, size_t *pos, int n, int64_t *M) {
    if (n <= 0 || M == NULL) return -1;
    uint8_t hdr = 0U;
    if (elec_get_u8(buf, end, pos, &hdr) != 0) return -1;
    if ((hdr >> 4U) != 0U) return -1;
    int shape = (int)((hdr >> 2U) & 3U);
    int enc = (int)(hdr & 3U);
    if (shape > ELEC_MAT_SHAPE_SYM || enc > ELEC_MAT_ENC_BITMASK) return -1;

    memset(M, 0, (size_t)n * (size_t)n * sizeof(int64_t));
    size_t total = mat_total_slots(n, shape);
    size_t nnz = 0U;
    size_t sparse_idx_bytes = 0U;

    if (enc == ELEC_MAT_ENC_ZERO) {
        if (shape != ELEC_MAT_SHAPE_LTRI) return -1; /* zero matrix is always LTRI canonically */
        return 0;
    } else if (enc == ELEC_MAT_ENC_SPARSE) {
        uint32_t nnz32 = 0U;
        if (elec_get_varu32(buf, end, pos, &nnz32) != 0 || nnz32 == 0U || (size_t)nnz32 > total) {
            return -1;
        }
        nnz = (size_t)nnz32;
        sparse_idx_bytes = elec_varu64_size((uint64_t)nnz);
        size_t next_min = 0U;
        for (size_t k = 0U; k < nnz; ++k) {
            uint32_t delta = 0U;
            int64_t v = 0;
            if (elec_get_varu32(buf, end, pos, &delta) != 0) return -1;
            if ((size_t)delta >= total - next_min) return -1;
            size_t slot = next_min + (size_t)delta;
            if (elec_get_zz64(buf, end, pos, &v) != 0 || v == 0) return -1;
            sparse_idx_bytes += elec_varu64_size((uint64_t)delta);
            int r = 0, c = 0;
            if (mat_slot_val(n, shape, M, slot, &r, &c, NULL) != 0) return -1;
            M[(size_t)r * (size_t)n + (size_t)c] = v;
            if (shape == ELEC_MAT_SHAPE_SYM && r != c) {
                M[(size_t)c * (size_t)n + (size_t)r] = v;
            }
            next_min = slot + 1U;
        }
    } else {
        size_t bitmap_bytes = (total + 7U) / 8U;
        if (*pos + bitmap_bytes > end) return -1;
        const uint8_t *bm = buf + *pos;
        *pos += bitmap_bytes;
        /* Reject trailing non-zero padding bits in last bitmap byte */
        size_t rem_bits = total & 7U;
        if (rem_bits != 0U) {
            uint8_t mask = (uint8_t)((1U << rem_bits) - 1U);
            if ((bm[bitmap_bytes - 1U] & (uint8_t)(~mask)) != 0U) return -1;
        }
        size_t next_min = 0U;
        for (size_t s = 0U; s < total; ++s) {
            if ((bm[s >> 3U] & (uint8_t)(1U << (s & 7U))) != 0U) {
                int64_t v = 0;
                if (elec_get_zz64(buf, end, pos, &v) != 0 || v == 0) return -1;
                nnz += 1U;
                sparse_idx_bytes += elec_varu64_size((uint64_t)(s - next_min));
                next_min = s + 1U;
                int r = 0, c = 0;
                if (mat_slot_val(n, shape, M, s, &r, &c, NULL) != 0) return -1;
                M[(size_t)r * (size_t)n + (size_t)c] = v;
                if (shape == ELEC_MAT_SHAPE_SYM && r != c) {
                    M[(size_t)c * (size_t)n + (size_t)r] = v;
                }
            }
        }
        if (nnz == 0U) return -1;
        sparse_idx_bytes += elec_varu64_size((uint64_t)nnz);
    }

    /* Enforce canonical shape & encoding choice */
    if (shape == ELEC_MAT_SHAPE_FULL && (mat_is_ltri(n, M) || mat_is_sym(n, M))) return -1;
    if (shape == ELEC_MAT_SHAPE_SYM && mat_is_ltri(n, M)) return -1;
    size_t bitmap_bytes = (total + 7U) / 8U;
    int expected_enc = (bitmap_bytes < sparse_idx_bytes) ? ELEC_MAT_ENC_BITMASK : ELEC_MAT_ENC_SPARSE;
    if (enc != expected_enc) return -1;
    return 0;
}

static int sparse_is_strictly_sorted_upper(int dim, int n_sparse, const ele_sparse_entry_t *entries) {
    int64_t prev_lin = -1;
    for (int i = 0; i < n_sparse; ++i) {
        int r = entries[i].r, c = entries[i].c;
        if (r < 0 || r >= dim || c < r || c >= dim) return 0;
        int64_t lin = (int64_t)r * (int64_t)dim + (int64_t)c;
        if (lin <= prev_lin) return 0;
        prev_lin = lin;
    }
    return 1;
}

static int elec_encode_full(const ele_full_cert_t *fc, uint8_t **out_buf, size_t *out_len) {
    elec_buf_t b;
    memset(&b, 0, sizeof(b));

    uint16_t flags = 0U;
    if (fc->rec.has_psd) flags |= ELEC_FLAG_PSD;
    if (fc->has_chordal) flags |= ELEC_FLAG_CHORDAL;
    if (fc->rec.has_sparam) flags |= ELEC_FLAG_SPARAM;
    if (fc->rec.has_bracket) flags |= ELEC_FLAG_BRACKET;
    if (fc->rec.has_motor) flags |= ELEC_FLAG_MOTOR;
    if (fc->rec.has_opf) flags |= ELEC_FLAG_OPF;
    if (fc->rec.has_emt) flags |= ELEC_FLAG_EMT;
    if (fc->rec.has_bracket && fc->has_bracket_vals) flags |= ELEC_FLAG_BRACKET_VALS;

    if (elec_put_le32(&b, ELE_CERT_WIRE_MAGIC) != 0 ||
        elec_put_le16(&b, (uint16_t)ELE_CERT_WIRE_VERSION) != 0 ||
        elec_put_le16(&b, flags) != 0 ||
        elec_put_str(&b, fc->version) != 0 ||
        elec_put_str(&b, fc->rec.pillar) != 0 ||
        elec_put_str(&b, fc->rec.kind) != 0 ||
        elec_put_str(&b, fc->rec.status) != 0 ||
        elec_put_str(&b, fc->rec.input_path) != 0 ||
        elec_put_sha256_str(&b, fc->rec.input_sha256) != 0) {
        free(b.data);
        return -1;
    }

    if (flags & ELEC_FLAG_PSD) {
        if (fc->rec.psd.dim <= 0 || fc->rec.psd.dim > ELE_CERT_MAX_DIM) {
            free(b.data); return -1;
        }
        if (elec_put_varu64(&b, (uint64_t)fc->rec.psd.dim) != 0 ||
            elec_put_zz64(&b, fc->rec.psd.scale_q) != 0 ||
            elec_put_zz64(&b, fc->rec.psd.mu_psd) != 0 ||
            elec_put_zz64(&b, fc->rec.psd.delta_round) != 0 ||
            elec_put_zz64(&b, fc->rec.psd.mu_cert) != 0 ||
            elec_put_i64_mat(&b, fc->rec.psd.dim, fc->rec.psd.A) != 0 ||
            elec_put_i64_mat(&b, fc->rec.psd.dim, fc->rec.psd.L) != 0) {
            free(b.data); return -1;
        }
    }

    if (flags & ELEC_FLAG_CHORDAL) {
        if (fc->chordal_dim < 2 || fc->chordal_dim > ELE_CHORDAL_MAX_DIM ||
            fc->num_cliques < 1 || fc->num_cliques > ELE_CHORDAL_MAX_CLIQUES ||
            fc->max_clique_dim < 2 || fc->max_clique_dim > ELE_CHORDAL_MAX_CL_DIM) {
            free(b.data); return -1;
        }
        if (elec_put_varu64(&b, (uint64_t)fc->chordal_dim) != 0 ||
            elec_put_varu64(&b, (uint64_t)fc->num_cliques) != 0 ||
            elec_put_varu64(&b, (uint64_t)fc->max_clique_dim) != 0 ||
            elec_put_zz64(&b, fc->min_clique_mu_psd) != 0) {
            free(b.data); return -1;
        }
        int sorted_upper = sparse_is_strictly_sorted_upper(
            fc->chordal_dim, fc->n_sparse, fc->A_sparse);
        uint8_t sp_mode = sorted_upper ? UINT8_C(0) : UINT8_C(1);
        if (elec_put_u8(&b, sp_mode) != 0 ||
            elec_put_varu64(&b, (uint64_t)fc->n_sparse) != 0) {
            free(b.data); return -1;
        }
        if (sp_mode == UINT8_C(0)) {
            uint64_t next_min = 0U;
            for (int i = 0; i < fc->n_sparse; ++i) {
                uint64_t lin = (uint64_t)fc->A_sparse[i].r * (uint64_t)fc->chordal_dim +
                               (uint64_t)fc->A_sparse[i].c;
                if (elec_put_varu64(&b, lin - next_min) != 0 ||
                    elec_put_zz64(&b, fc->A_sparse[i].val) != 0) {
                    free(b.data); return -1;
                }
                next_min = lin + 1U;
            }
        } else {
            for (int i = 0; i < fc->n_sparse; ++i) {
                if (fc->A_sparse[i].r < 0 || fc->A_sparse[i].c < 0) {
                    free(b.data); return -1;
                }
                if (elec_put_varu64(&b, (uint64_t)fc->A_sparse[i].r) != 0 ||
                    elec_put_varu64(&b, (uint64_t)fc->A_sparse[i].c) != 0 ||
                    elec_put_zz64(&b, fc->A_sparse[i].val) != 0) {
                    free(b.data); return -1;
                }
            }
        }
        for (int c = 0; c < fc->num_cliques; ++c) {
            const ele_chordal_clique_t *clq = &fc->cliques[c];
            if (clq->dk < 2 || clq->dk > ELE_CHORDAL_MAX_CL_DIM) {
                free(b.data); return -1;
            }
            if (elec_put_varu64(&b, (uint64_t)clq->dk) != 0) {
                free(b.data); return -1;
            }
            for (int a = 0; a < clq->dk; ++a) {
                if (clq->vars[a] < 0 || elec_put_varu64(&b, (uint64_t)clq->vars[a]) != 0) {
                    free(b.data); return -1;
                }
            }
            if (elec_put_zz64(&b, clq->mu_psd) != 0 ||
                elec_put_zz64(&b, clq->w_max_int) != 0 ||
                elec_put_i64_mat(&b, clq->dk, clq->S_int) != 0 ||
                elec_put_i64_mat(&b, clq->dk, clq->L_int) != 0) {
                free(b.data); return -1;
            }
        }
    }

    if (flags & ELEC_FLAG_SPARAM) {
        if (fc->rec.nports <= 0 || fc->rec.nports > ELE_CERT_MAX_PORTS) {
            free(b.data); return -1;
        }
        if (elec_put_varu64(&b, (uint64_t)fc->rec.nports) != 0 ||
            elec_put_zz64(&b, fc->rec.s_scale_q) != 0 ||
            elec_put_zz64(&b, fc->rec.recip_eps_q) != 0 ||
            elec_put_f64(&b, fc->rec.worst_freq_hz) != 0 ||
            elec_put_f64(&b, fc->rec.sigma_max) != 0 ||
            elec_put_f64(&b, fc->rec.max_recip_err) != 0 ||
            elec_put_i64_mat(&b, fc->rec.nports, fc->rec.R_q) != 0 ||
            elec_put_i64_mat(&b, fc->rec.nports, fc->rec.X_q) != 0) {
            free(b.data); return -1;
        }
    }

    if (flags & ELEC_FLAG_BRACKET) {
        if (elec_put_zz64(&b, fc->rec.bracket_scale) != 0 ||
            elec_put_zz64(&b, fc->rec.lb_int) != 0 ||
            elec_put_zz64(&b, fc->rec.ub_int) != 0 ||
            elec_put_zz64(&b, fc->rec.limit_lo_int) != 0 ||
            elec_put_zz64(&b, fc->rec.limit_hi_int) != 0) {
            free(b.data); return -1;
        }
        if (flags & ELEC_FLAG_BRACKET_VALS) {
            if (elec_put_f64(&b, fc->rec.lb_val) != 0 ||
                elec_put_f64(&b, fc->rec.ub_val) != 0) {
                free(b.data); return -1;
            }
        }
    }

    if (flags & ELEC_FLAG_MOTOR) {
        if (elec_put_zz64(&b, fc->rec.motor_scale) != 0 ||
            elec_put_zz64(&b, fc->rec.id_int) != 0 ||
            elec_put_zz64(&b, fc->rec.iq_int) != 0 ||
            elec_put_zz64(&b, fc->rec.vd_int) != 0 ||
            elec_put_zz64(&b, fc->rec.vq_int) != 0 ||
            elec_put_zz64(&b, fc->rec.psid_int) != 0 ||
            elec_put_zz64(&b, fc->rec.psiq_int) != 0 ||
            elec_put_zz64(&b, fc->rec.imax_int) != 0 ||
            elec_put_zz64(&b, fc->rec.vmax_int) != 0 ||
            elec_put_zz64(&b, fc->rec.two_p) != 0 ||
            elec_put_zz64(&b, fc->rec.torque_int) != 0) {
            free(b.data); return -1;
        }
    }

    if (flags & ELEC_FLAG_OPF) {
        if (elec_put_zz64(&b, fc->rec.opf_scale) != 0 ||
            elec_put_zz64(&b, fc->rec.c_dual_int) != 0 ||
            elec_put_zz64(&b, fc->rec.w_max_int) != 0 ||
            elec_put_zz64(&b, fc->rec.opf_delta_round) != 0 ||
            elec_put_zz64(&b, fc->rec.c_lb_int) != 0 ||
            elec_put_zz64(&b, fc->rec.c_ub_int) != 0) {
            free(b.data); return -1;
        }
    }

    if (flags & ELEC_FLAG_EMT) {
        if (elec_put_zz64(&b, fc->rec.emt_scale) != 0 ||
            elec_put_zz64(&b, fc->rec.v_kin_int) != 0 ||
            elec_put_zz64(&b, fc->rec.v_pot_int) != 0 ||
            elec_put_zz64(&b, fc->rec.v_crit_int) != 0 ||
            elec_put_zz64(&b, fc->rec.v_err_int) != 0 ||
            elec_put_f64(&b, fc->rec.cct_sec) != 0 ||
            elec_put_f64(&b, fc->rec.t_clear_sec) != 0) {
            free(b.data); return -1;
        }
    }

    /* Integrity trailer: 32-byte FIPS-180-4 SHA-256 + 8-byte FNV-1a-64 */
    if (elec_buf_reserve(&b, ELEC_TRAILER_BYTES) != 0) {
        free(b.data); return -1;
    }
    size_t body_len = b.len;
    flow_sha256_ctx_t sctx;
    uint8_t sha_digest[32];
    flow_sha256_init(&sctx);
    flow_sha256_update(&sctx, b.data, body_len);
    flow_sha256_final(&sctx, sha_digest);
    memcpy(b.data + body_len, sha_digest, 32U);
    uint64_t fnv = flow_ir_wire_fnv1a64(b.data, body_len + 32U);
    flow_ir_wire_write_le64(b.data + body_len + 32U, fnv);
    b.len = body_len + ELEC_TRAILER_BYTES;

    *out_buf = b.data;
    *out_len = b.len;
    return 0;
}

static int elec_decode_full(const uint8_t *buf, size_t len, ele_full_cert_t *fc) {
    ele_full_cert_init(fc);
    if (buf == NULL || len < 8U + ELEC_TRAILER_BYTES) return -1;
    if (flow_ir_wire_read_le32(buf + 0U) != ELE_CERT_WIRE_MAGIC) return -1;
    uint16_t wire_ver = flow_ir_wire_read_le16(buf + 4U);
    if (wire_ver != (uint16_t)ELE_CERT_WIRE_VERSION) return -1;

    size_t body_len = len - ELEC_TRAILER_BYTES;
    uint64_t got_fnv = flow_ir_wire_read_le64(buf + body_len + 32U);
    uint64_t exp_fnv = flow_ir_wire_fnv1a64(buf, body_len + 32U);
    if (got_fnv != exp_fnv) return -21;

    flow_sha256_ctx_t sctx;
    uint8_t exp_sha[32];
    flow_sha256_init(&sctx);
    flow_sha256_update(&sctx, buf, body_len);
    flow_sha256_final(&sctx, exp_sha);
    if (memcmp(buf + body_len, exp_sha, 32U) != 0) return -22;

    uint16_t flags = flow_ir_wire_read_le16(buf + 6U);
    if ((flags & (uint16_t)(~ELEC_FLAGS_KNOWN_MASK)) != 0U) return -1;
    if ((flags & ELEC_FLAG_BRACKET_VALS) != 0U && (flags & ELEC_FLAG_BRACKET) == 0U) return -1;

    size_t pos = 8U;
    if (elec_get_str(buf, body_len, &pos, fc->version, sizeof(fc->version)) != 0 ||
        elec_get_str(buf, body_len, &pos, fc->rec.pillar, sizeof(fc->rec.pillar)) != 0 ||
        elec_get_str(buf, body_len, &pos, fc->rec.kind, sizeof(fc->rec.kind)) != 0 ||
        elec_get_str(buf, body_len, &pos, fc->rec.status, sizeof(fc->rec.status)) != 0 ||
        elec_get_str(buf, body_len, &pos, fc->rec.input_path, sizeof(fc->rec.input_path)) != 0 ||
        elec_get_sha256_str(buf, body_len, &pos, fc->rec.input_sha256) != 0) {
        return -1;
    }

    fc->rec.has_psd = (flags & ELEC_FLAG_PSD) ? 1 : 0;
    fc->has_chordal = (flags & ELEC_FLAG_CHORDAL) ? 1 : 0;
    fc->rec.has_sparam = (flags & ELEC_FLAG_SPARAM) ? 1 : 0;
    fc->rec.has_bracket = (flags & ELEC_FLAG_BRACKET) ? 1 : 0;
    fc->rec.has_motor = (flags & ELEC_FLAG_MOTOR) ? 1 : 0;
    fc->rec.has_opf = (flags & ELEC_FLAG_OPF) ? 1 : 0;
    fc->rec.has_emt = (flags & ELEC_FLAG_EMT) ? 1 : 0;
    fc->has_bracket_vals = (flags & ELEC_FLAG_BRACKET_VALS) ? 1 : 0;

    if (fc->rec.has_psd) {
        uint32_t dim32 = 0U;
        if (elec_get_varu32(buf, body_len, &pos, &dim32) != 0 ||
            dim32 == 0U || dim32 > (uint32_t)ELE_CERT_MAX_DIM ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psd.scale_q) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psd.mu_psd) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psd.delta_round) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psd.mu_cert) != 0) {
            goto fail;
        }
        fc->rec.psd.dim = (int)dim32;
        fc->rec.psd.ok = (strcmp(fc->rec.status, "CERTIFIED") == 0) ? 1 : 0;
        if (elec_get_i64_mat(buf, body_len, &pos, fc->rec.psd.dim, fc->rec.psd.A) != 0 ||
            elec_get_i64_mat(buf, body_len, &pos, fc->rec.psd.dim, fc->rec.psd.L) != 0) {
            goto fail;
        }
    }

    if (fc->has_chordal) {
        uint32_t cdim = 0U, ncl = 0U, mcdim = 0U, nsp = 0U;
        uint8_t sp_mode = 0U;
        if (elec_get_varu32(buf, body_len, &pos, &cdim) != 0 ||
            cdim < 2U || cdim > (uint32_t)ELE_CHORDAL_MAX_DIM ||
            elec_get_varu32(buf, body_len, &pos, &ncl) != 0 ||
            ncl < 1U || ncl > (uint32_t)ELE_CHORDAL_MAX_CLIQUES ||
            elec_get_varu32(buf, body_len, &pos, &mcdim) != 0 ||
            mcdim < 2U || mcdim > (uint32_t)ELE_CHORDAL_MAX_CL_DIM ||
            elec_get_zz64(buf, body_len, &pos, &fc->min_clique_mu_psd) != 0 ||
            elec_get_u8(buf, body_len, &pos, &sp_mode) != 0 ||
            elec_get_varu32(buf, body_len, &pos, &nsp) != 0 ||
            nsp > cdim * cdim) {
            goto fail;
        }
        fc->chordal_dim = (int)cdim;
        fc->num_cliques = (int)ncl;
        fc->max_clique_dim = (int)mcdim;
        fc->n_sparse = (int)nsp;
        fc->sparse_cap = (nsp > 0U) ? (int)nsp : 1;
        fc->A_sparse = (ele_sparse_entry_t *)calloc((size_t)fc->sparse_cap, sizeof(ele_sparse_entry_t));
        fc->cliques = (ele_chordal_clique_t *)calloc((size_t)fc->num_cliques, sizeof(ele_chordal_clique_t));
        if (fc->A_sparse == NULL || fc->cliques == NULL) goto fail;

        if (sp_mode == UINT8_C(0)) {
            uint64_t next_min = 0U;
            uint64_t max_lin = (uint64_t)cdim * (uint64_t)cdim;
            for (int i = 0; i < fc->n_sparse; ++i) {
                uint64_t delta = 0U;
                int64_t val = 0;
                if (elec_get_varu64(buf, body_len, &pos, &delta) != 0 ||
                    delta >= max_lin - next_min ||
                    elec_get_zz64(buf, body_len, &pos, &val) != 0) {
                    goto fail;
                }
                uint64_t lin = next_min + delta;
                int r = (int)(lin / (uint64_t)cdim);
                int c = (int)(lin % (uint64_t)cdim);
                if (c < r) goto fail;
                fc->A_sparse[i].r = r;
                fc->A_sparse[i].c = c;
                fc->A_sparse[i].val = val;
                next_min = lin + 1U;
            }
        } else if (sp_mode == UINT8_C(1)) {
            for (int i = 0; i < fc->n_sparse; ++i) {
                uint32_t r = 0U, c = 0U;
                int64_t val = 0;
                if (elec_get_varu32(buf, body_len, &pos, &r) != 0 ||
                    elec_get_varu32(buf, body_len, &pos, &c) != 0 ||
                    r >= cdim || c >= cdim ||
                    elec_get_zz64(buf, body_len, &pos, &val) != 0) {
                    goto fail;
                }
                fc->A_sparse[i].r = (int)r;
                fc->A_sparse[i].c = (int)c;
                fc->A_sparse[i].val = val;
            }
            if (sparse_is_strictly_sorted_upper(fc->chordal_dim, fc->n_sparse, fc->A_sparse)) {
                goto fail; /* Canonical mode 0 was required */
            }
        } else {
            goto fail;
        }

        for (int c = 0; c < fc->num_cliques; ++c) {
            ele_chordal_clique_t *clq = &fc->cliques[c];
            uint32_t dk32 = 0U;
            if (elec_get_varu32(buf, body_len, &pos, &dk32) != 0 ||
                dk32 < 2U || dk32 > (uint32_t)ELE_CHORDAL_MAX_CL_DIM) {
                goto fail;
            }
            clq->dk = (int)dk32;
            for (int a = 0; a < clq->dk; ++a) {
                uint32_t v32 = 0U;
                if (elec_get_varu32(buf, body_len, &pos, &v32) != 0 || v32 >= cdim) goto fail;
                clq->vars[a] = (int)v32;
            }
            if (elec_get_zz64(buf, body_len, &pos, &clq->mu_psd) != 0 ||
                elec_get_zz64(buf, body_len, &pos, &clq->w_max_int) != 0 ||
                elec_get_i64_mat(buf, body_len, &pos, clq->dk, clq->S_int) != 0 ||
                elec_get_i64_mat(buf, body_len, &pos, clq->dk, clq->L_int) != 0) {
                goto fail;
            }
        }
    }

    if (fc->rec.has_sparam) {
        uint32_t np32 = 0U;
        if (elec_get_varu32(buf, body_len, &pos, &np32) != 0 ||
            np32 == 0U || np32 > (uint32_t)ELE_CERT_MAX_PORTS ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.s_scale_q) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.recip_eps_q) != 0 ||
            elec_get_f64(buf, body_len, &pos, &fc->rec.worst_freq_hz) != 0 ||
            elec_get_f64(buf, body_len, &pos, &fc->rec.sigma_max) != 0 ||
            elec_get_f64(buf, body_len, &pos, &fc->rec.max_recip_err) != 0) {
            goto fail;
        }
        fc->rec.nports = (int)np32;
        if (elec_get_i64_mat(buf, body_len, &pos, fc->rec.nports, fc->rec.R_q) != 0 ||
            elec_get_i64_mat(buf, body_len, &pos, fc->rec.nports, fc->rec.X_q) != 0) {
            goto fail;
        }
    }

    if (fc->rec.has_bracket) {
        if (elec_get_zz64(buf, body_len, &pos, &fc->rec.bracket_scale) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.lb_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.ub_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.limit_lo_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.limit_hi_int) != 0) {
            goto fail;
        }
        if (fc->has_bracket_vals) {
            if (elec_get_f64(buf, body_len, &pos, &fc->rec.lb_val) != 0 ||
                elec_get_f64(buf, body_len, &pos, &fc->rec.ub_val) != 0) {
                goto fail;
            }
        }
    }

    if (fc->rec.has_motor) {
        if (elec_get_zz64(buf, body_len, &pos, &fc->rec.motor_scale) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.id_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.iq_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.vd_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.vq_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psid_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.psiq_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.imax_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.vmax_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.two_p) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.torque_int) != 0) {
            goto fail;
        }
    }

    if (fc->rec.has_opf) {
        if (elec_get_zz64(buf, body_len, &pos, &fc->rec.opf_scale) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.c_dual_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.w_max_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.opf_delta_round) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.c_lb_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.c_ub_int) != 0) {
            goto fail;
        }
    }

    if (fc->rec.has_emt) {
        if (elec_get_zz64(buf, body_len, &pos, &fc->rec.emt_scale) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.v_kin_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.v_pot_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.v_crit_int) != 0 ||
            elec_get_zz64(buf, body_len, &pos, &fc->rec.v_err_int) != 0 ||
            elec_get_f64(buf, body_len, &pos, &fc->rec.cct_sec) != 0 ||
            elec_get_f64(buf, body_len, &pos, &fc->rec.t_clear_sec) != 0) {
            goto fail;
        }
    }

    if (pos != body_len) goto fail;
    return 0;

fail:
    ele_full_cert_free(fc);
    return -1;
}

static int verify_chordal_full(const ele_full_cert_t *fc) {
    int dim = fc->chordal_dim;
    int n_cliques = fc->num_cliques;
    if (dim < 2 || dim > ELE_CHORDAL_MAX_DIM ||
        n_cliques < 1 || n_cliques > ELE_CHORDAL_MAX_CLIQUES) {
        return -1;
    }
    if (!fc->rec.has_opf || fc->rec.w_max_int <= 0 || fc->rec.opf_delta_round < 0) return -1;

    size_t mat_sz = (size_t)dim * (size_t)dim;
    ele_i128 *A_target = (ele_i128 *)calloc(mat_sz, sizeof(ele_i128));
    ele_i128 *A_sum = (ele_i128 *)calloc(mat_sz, sizeof(ele_i128));
    uint8_t *seen = (uint8_t *)calloc(mat_sz, sizeof(uint8_t));
    if (A_target == NULL || A_sum == NULL || seen == NULL) {
        free(A_target); free(A_sum); free(seen);
        return -1;
    }

    for (int i = 0; i < fc->n_sparse; ++i) {
        int r_i = fc->A_sparse[i].r;
        int c_i = fc->A_sparse[i].c;
        int64_t val = fc->A_sparse[i].val;
        if (r_i < 0 || r_i >= dim || c_i < r_i || c_i >= dim) {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        size_t r = (size_t)r_i, c = (size_t)c_i;
        if (seen[r * (size_t)dim + c]) {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        seen[r * (size_t)dim + c] = 1U;
        A_target[r * (size_t)dim + c] = (ele_i128)val;
        A_target[c * (size_t)dim + r] = (ele_i128)val;
    }

    int64_t min_mu = 0;
    ele_i128 neg_pen = 0;
    for (int c = 0; c < n_cliques; ++c) {
        const ele_chordal_clique_t *clq = &fc->cliques[c];
        int dk = clq->dk;
        if (dk < 2 || dk > ELE_CHORDAL_MAX_CL_DIM || clq->w_max_int <= 0) {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        for (int a = 0; a < dk; ++a) {
            if (clq->vars[a] < 0 || clq->vars[a] >= dim) {
                free(A_target); free(A_sum); free(seen);
                return -1;
            }
            for (int b = a + 1; b < dk; ++b) {
                if (clq->vars[a] == clq->vars[b]) {
                    free(A_target); free(A_sum); free(seen);
                    return -1;
                }
            }
        }
        if (ele_verify_cholesky_sdd_i64(dk, clq->S_int, clq->L_int, clq->mu_psd) != 0) {
            free(A_target); free(A_sum); free(seen);
            return -16;
        }
        for (int a = 0; a < dk; ++a) {
            size_t va = (size_t)clq->vars[a];
            for (int b = 0; b < dk; ++b) {
                size_t vb = (size_t)clq->vars[b];
                A_sum[va * (size_t)dim + vb] += (ele_i128)clq->S_int[(size_t)a * (size_t)dk + (size_t)b];
            }
        }
        if (c == 0 || clq->mu_psd < min_mu) min_mu = clq->mu_psd;
        if (clq->mu_psd < 0) neg_pen += (ele_i128)clq->mu_psd * (ele_i128)clq->w_max_int;
    }

    for (size_t p = 0; p < mat_sz; ++p) {
        if (A_sum[p] != A_target[p]) {
            free(A_target); free(A_sum); free(seen);
            return -17;
        }
    }
    free(A_target); free(A_sum); free(seen);

    if (min_mu != fc->min_clique_mu_psd) return -18;
    ele_i128 expected_lb = (ele_i128)fc->rec.c_dual_int +
                           neg_pen -
                           (ele_i128)fc->rec.opf_delta_round;
    if ((ele_i128)fc->rec.c_lb_int != expected_lb) return -19;
    if (strcmp(fc->rec.status, "CERTIFIED") == 0 && fc->rec.c_lb_int > fc->rec.c_ub_int) return -20;
    return 0;
}

static int verify_full_cert(const ele_full_cert_t *fc, int verify_input_hash) {
    const ele_cert_record_t *rec = &fc->rec;

    if (verify_input_hash && strlen(rec->input_path) > 0U &&
        strcmp(rec->input_path, "internal") != 0 &&
        strlen(rec->input_sha256) == 64U) {
        char actual_hash[65];
        if (ele_sha256_file(rec->input_path, actual_hash) == 0) {
            if (strcmp(actual_hash, rec->input_sha256) != 0) {
                return -2;
            }
        }
    }

    if (rec->has_psd) {
        if (rec->psd.dim <= 0 || rec->psd.dim > ELE_CERT_MAX_DIM) return -1;
        if (rec->psd.mu_cert != rec->psd.mu_psd - rec->psd.delta_round) return -3;
        if (ele_verify_cholesky_sdd_i64(rec->psd.dim, rec->psd.A, rec->psd.L, rec->psd.mu_psd) != 0) {
            return -4;
        }
    }

    if (rec->has_sparam) {
        if (rec->nports <= 0 || rec->nports > ELE_CERT_MAX_PORTS) return -1;
        if (rec->has_psd) {
            if (rec->psd.dim != 2 * rec->nports) return -5;
            int n = rec->nports;
            int d = 2 * n;
            ele_i128 Q2 = (ele_i128)rec->s_scale_q * (ele_i128)rec->s_scale_q;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    ele_i128 aij = (i == j) ? Q2 : 0;
                    ele_i128 bij = 0;
                    for (int k = 0; k < n; ++k) {
                        ele_i128 rki = rec->R_q[(size_t)k * (size_t)n + (size_t)i];
                        ele_i128 rkj = rec->R_q[(size_t)k * (size_t)n + (size_t)j];
                        ele_i128 xki = rec->X_q[(size_t)k * (size_t)n + (size_t)i];
                        ele_i128 xkj = rec->X_q[(size_t)k * (size_t)n + (size_t)j];
                        aij -= (rki * rkj + xki * xkj);
                        bij += (rki * xkj - xki * rkj);
                    }
                    if ((ele_i128)rec->psd.A[(size_t)i * (size_t)d + (size_t)j] != aij ||
                        (ele_i128)rec->psd.A[(size_t)(n + i) * (size_t)d + (size_t)(n + j)] != aij ||
                        (ele_i128)rec->psd.A[(size_t)i * (size_t)d + (size_t)(n + j)] != bij ||
                        (ele_i128)rec->psd.A[(size_t)(n + i) * (size_t)d + (size_t)j] != -bij) {
                        return -6;
                    }
                }
            }
            if (strcmp(rec->status, "CERTIFIED") == 0 && rec->psd.mu_cert < 0) return -7;
            if (strcmp(rec->status, "VETOED") == 0 && rec->psd.mu_cert >= 0) return -8;
        }
    }

    if (rec->has_bracket) {
        if (rec->lb_int > rec->ub_int) return -9;
        if (strcmp(rec->status, "CERTIFIED") == 0 &&
            (rec->limit_lo_int > rec->lb_int || rec->ub_int > rec->limit_hi_int)) {
            return -10;
        }
    }

    if (rec->has_motor) {
        ele_i128 c2 = (ele_i128)rec->id_int * rec->id_int + (ele_i128)rec->iq_int * rec->iq_int;
        ele_i128 cm = (ele_i128)rec->imax_int * rec->imax_int;
        ele_i128 v2 = (ele_i128)rec->vd_int * rec->vd_int + (ele_i128)rec->vq_int * rec->vq_int;
        ele_i128 vm = (ele_i128)rec->vmax_int * rec->vmax_int;
        ele_i128 tq = (ele_i128)rec->two_p *
                      ((ele_i128)rec->psid_int * rec->iq_int - (ele_i128)rec->psiq_int * rec->id_int);
        if (c2 > cm || v2 > vm) return -11;
        if (rec->has_bracket && (tq < rec->lb_int || tq > rec->ub_int)) return -12;
    }

    if (rec->has_opf) {
        if (rec->has_psd) {
            ele_i128 neg_mu = (rec->psd.mu_psd < 0) ? (ele_i128)rec->psd.mu_psd : 0;
            ele_i128 expected_lb = (ele_i128)rec->c_dual_int +
                                   neg_mu * (ele_i128)rec->w_max_int -
                                   (ele_i128)rec->opf_delta_round;
            if ((ele_i128)rec->c_lb_int != expected_lb || rec->c_lb_int > rec->c_ub_int) {
                return -13;
            }
        }
    }

    if (rec->has_emt) {
        if (rec->v_kin_int < 0 || rec->v_err_int < 0) return -14;
        if (strcmp(rec->status, "CERTIFIED") == 0 &&
            (ele_i128)rec->v_kin_int + rec->v_pot_int + rec->v_err_int >= rec->v_crit_int) {
            return -15;
        }
    }

    if (fc->has_chordal) {
        int cr = verify_chordal_full(fc);
        if (cr != 0) return cr;
    }

    return 0;
}

static int write_full_cert_json(const char *out_path, const ele_full_cert_t *fc) {
    if (out_path == NULL || fc == NULL) return -1;
    if (!fc->has_chordal) {
        return ele_cert_write_json(out_path, &fc->rec);
    }
    FILE *cf = fopen(out_path, "w");
    if (cf == NULL) return -1;
    fprintf(cf, "{\n");
    fprintf(cf, "  \"format\": \"ELECERT-1.0\",\n");
    fprintf(cf, "  \"version\": \"%s\",\n", fc->version);
    fprintf(cf, "  \"pillar\": \"%s\",\n", fc->rec.pillar);
    fprintf(cf, "  \"kind\": \"%s\",\n", fc->rec.kind);
    fprintf(cf, "  \"status\": \"%s\",\n", fc->rec.status);
    fprintf(cf, "  \"input_path\": \"%s\",\n", fc->rec.input_path);
    fprintf(cf, "  \"input_sha256\": \"%s\",\n", fc->rec.input_sha256);
    fprintf(cf, "  \"has_psd\": %d,\n", fc->rec.has_psd);
    fprintf(cf, "  \"has_chordal\": 1,\n");
    fprintf(cf, "  \"has_sparam\": %d,\n", fc->rec.has_sparam);
    fprintf(cf, "  \"has_bracket\": %d,\n", fc->rec.has_bracket);
    fprintf(cf, "  \"has_motor\": %d,\n", fc->rec.has_motor);
    fprintf(cf, "  \"has_opf\": %d,\n", fc->rec.has_opf);
    fprintf(cf, "  \"has_emt\": %d,\n", fc->rec.has_emt);
    fprintf(cf, "  \"dim\": %d,\n", fc->chordal_dim);
    fprintf(cf, "  \"num_cliques\": %d,\n", fc->num_cliques);
    fprintf(cf, "  \"max_clique_dim\": %d,\n", fc->max_clique_dim);
    fprintf(cf, "  \"min_clique_mu_psd\": %" PRId64 ",\n", fc->min_clique_mu_psd);
    if (fc->rec.has_opf) {
        fprintf(cf, "  \"opf_scale\": %" PRId64 ",\n", fc->rec.opf_scale);
        fprintf(cf, "  \"c_dual_int\": %" PRId64 ",\n", fc->rec.c_dual_int);
        fprintf(cf, "  \"w_max_int\": %" PRId64 ",\n", fc->rec.w_max_int);
        fprintf(cf, "  \"opf_delta_round\": %" PRId64 ",\n", fc->rec.opf_delta_round);
        fprintf(cf, "  \"c_lb_int\": %" PRId64 ",\n", fc->rec.c_lb_int);
        fprintf(cf, "  \"c_ub_int\": %" PRId64 ",\n", fc->rec.c_ub_int);
    }
    if (fc->rec.has_bracket) {
        fprintf(cf, "  \"bracket_scale\": %" PRId64 ",\n", fc->rec.bracket_scale);
        fprintf(cf, "  \"limit_lo_int\": %" PRId64 ",\n", fc->rec.limit_lo_int);
        fprintf(cf, "  \"lb_int\": %" PRId64 ",\n", fc->rec.lb_int);
        fprintf(cf, "  \"ub_int\": %" PRId64 ",\n", fc->rec.ub_int);
        fprintf(cf, "  \"limit_hi_int\": %" PRId64 ",\n", fc->rec.limit_hi_int);
        if (fc->has_bracket_vals) {
            fprintf(cf, "  \"lb_val\": %.12g,\n", fc->rec.lb_val);
            fprintf(cf, "  \"ub_val\": %.12g,\n", fc->rec.ub_val);
        }
    }
    fprintf(cf, "  \"A_sparse\": [\n");
    for (int i = 0; i < fc->n_sparse; ++i) {
        fprintf(cf, "%s    [%d, %d, %" PRId64 "]", (i > 0) ? ",\n" : "",
                fc->A_sparse[i].r, fc->A_sparse[i].c, fc->A_sparse[i].val);
    }
    fprintf(cf, "\n  ],\n");
    fprintf(cf, "  \"cliques\": [\n");
    for (int c = 0; c < fc->num_cliques; ++c) {
        const ele_chordal_clique_t *clq = &fc->cliques[c];
        int dk = clq->dk;
        fprintf(cf, "    {\n");
        fprintf(cf, "      \"vars\": [");
        for (int a = 0; a < dk; ++a) fprintf(cf, "%s%d", a ? ", " : "", clq->vars[a]);
        fprintf(cf, "],\n");
        fprintf(cf, "      \"mu_psd\": %" PRId64 ",\n", clq->mu_psd);
        fprintf(cf, "      \"w_max_int\": %" PRId64 ",\n", clq->w_max_int);
        fprintf(cf, "      \"S_int\": [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(cf, "        [");
            for (int b = 0; b < dk; ++b) {
                fprintf(cf, "%s%" PRId64, b ? ", " : "", clq->S_int[(size_t)a * (size_t)dk + (size_t)b]);
            }
            fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(cf, "      ],\n");
        fprintf(cf, "      \"L_int\": [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(cf, "        [");
            for (int b = 0; b < dk; ++b) {
                fprintf(cf, "%s%" PRId64, b ? ", " : "", clq->L_int[(size_t)a * (size_t)dk + (size_t)b]);
            }
            fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(cf, "      ]\n");
        fprintf(cf, "    }%s\n", (c + 1 < fc->num_cliques) ? "," : "");
    }
    fprintf(cf, "  ]\n");
    fprintf(cf, "}\n");
    int ferr = ferror(cf);
    fclose(cf);
    return (ferr == 0) ? 0 : -1;
}

static int read_entire_file(const char *path, uint8_t **out_buf, size_t *out_len) {
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) return -1;
    if (fseek(fp, 0L, SEEK_END) != 0) { fclose(fp); return -1; }
    long fsz = ftell(fp);
    if (fsz <= 0 || fsz > 16L * 1024L * 1024L) { fclose(fp); return -1; }
    rewind(fp);
    uint8_t *buf = (uint8_t *)malloc((size_t)fsz + 1U);
    if (buf == NULL) { fclose(fp); return -1; }
    size_t nr = fread(buf, 1U, (size_t)fsz, fp);
    int ferr = ferror(fp);
    fclose(fp);
    if (ferr != 0 || nr != (size_t)fsz) { free(buf); return -1; }
    buf[nr] = 0U;
    *out_buf = buf;
    *out_len = nr;
    return 0;
}

static int write_entire_file(const char *path, const uint8_t *buf, size_t len) {
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) return -1;
    size_t nw = (len > 0U) ? fwrite(buf, 1U, len, fp) : 0U;
    int ferr = ferror(fp);
    int cls = fclose(fp);
    return (ferr == 0 && cls == 0 && nw == len) ? 0 : -1;
}

static int has_bin_suffix(const char *path) {
    if (path == NULL) return 0;
    size_t n = strlen(path);
    return (n >= 4U && strcmp(path + n - 4U, ".bin") == 0);
}

static int load_full_cert_auto(const char *cert_path, ele_full_cert_t *fc) {
    uint8_t *buf = NULL;
    size_t len = 0U;
    if (read_entire_file(cert_path, &buf, &len) != 0) return -1;
    int rc = 0;
    if (len >= 4U && flow_ir_wire_read_le32(buf) == ELE_CERT_WIRE_MAGIC) {
        rc = elec_decode_full(buf, len, fc);
    } else {
        rc = parse_json_full_cert((char *)buf, fc);
    }
    free(buf);
    return rc;
}

int ele_cert_write_bin(const char *out_path, const ele_cert_record_t *rec) {
    if (out_path == NULL || rec == NULL) return -1;
    ele_full_cert_t fc;
    ele_full_cert_init(&fc);
    fc.rec = *rec;
    fc.has_bracket_vals = rec->has_bracket ? 1 : 0;
    uint8_t *wire = NULL;
    size_t wire_len = 0U;
    if (elec_encode_full(&fc, &wire, &wire_len) != 0) return -1;
    int rc = write_entire_file(out_path, wire, wire_len);
    free(wire);
    return rc;
}

int ele_cert_write_auto(const char *out_path, const ele_cert_record_t *rec) {
    if (has_bin_suffix(out_path)) return ele_cert_write_bin(out_path, rec);
    return ele_cert_write_json(out_path, rec);
}

int ele_cert_pack_file(const char *json_in_path, const char *bin_out_path) {
    if (json_in_path == NULL || bin_out_path == NULL) return -1;
    uint8_t *buf = NULL;
    size_t len = 0U;
    if (read_entire_file(json_in_path, &buf, &len) != 0) return -1;
    ele_full_cert_t fc;
    int prc = parse_json_full_cert((char *)buf, &fc);
    free(buf);
    if (prc != 0) return -1;

    uint8_t *wire = NULL;
    size_t wire_len = 0U;
    int erc = elec_encode_full(&fc, &wire, &wire_len);
    ele_full_cert_free(&fc);
    if (erc != 0) return -1;
    int wrc = write_entire_file(bin_out_path, wire, wire_len);
    free(wire);
    return wrc;
}

int ele_cert_unpack_file(const char *bin_in_path, const char *json_out_path) {
    if (bin_in_path == NULL || json_out_path == NULL) return -1;
    uint8_t *buf = NULL;
    size_t len = 0U;
    if (read_entire_file(bin_in_path, &buf, &len) != 0) return -1;
    ele_full_cert_t fc;
    int drc = elec_decode_full(buf, len, &fc);
    free(buf);
    if (drc != 0) return drc;
    int wrc = write_full_cert_json(json_out_path, &fc);
    ele_full_cert_free(&fc);
    return wrc;
}

int ele_cert_verify_file(const char *cert_path, int verify_input_hash) {
    if (cert_path == NULL) return -1;
    ele_full_cert_t fc;
    int lrc = load_full_cert_auto(cert_path, &fc);
    if (lrc != 0) return lrc;
    int vrc = verify_full_cert(&fc, verify_input_hash);
    ele_full_cert_free(&fc);
    return vrc;
}

/* ============================================================================
 * 5. Standalone Lean 4 Certificate Emitter
 * ============================================================================ */

static void emit_lean_i64_matrix(FILE *fp, const char *name, int n, const int64_t *M) {
    fprintf(fp, "def %s : List (List Int) := [\n", name);
    for (int i = 0; i < n; ++i) {
        fprintf(fp, "  [");
        for (int j = 0; j < n; ++j) {
            fprintf(fp, "%s%" PRId64, (j > 0) ? ", " : "", M[(size_t)i * (size_t)n + (size_t)j]);
        }
        fprintf(fp, "]%s\n", (i + 1 < n) ? "," : "");
    }
    fprintf(fp, "]\n\n");
}

int ele_cert_emit_lean(const char *out_path, const char *mod_name,
                       const ele_cert_record_t *rec) {
    if (out_path == NULL || mod_name == NULL || rec == NULL) return -1;
    FILE *fp = fopen(out_path, "w");
    if (fp == NULL) return -1;

    fprintf(fp, "/-\n  Standalone Zero-FP-Trust Certificate: %s\n  Pillar: %s | Kind: %s | SHA256: %s\n-/\n",
            mod_name, rec->pillar, rec->kind, rec->input_sha256);
    fprintf(fp, "import EleCert\nimport Flow.Interval\n\nnamespace %s\n\nopen Flow.Interval\n\n", mod_name);

    if (rec->has_sparam && rec->has_psd && strcmp(rec->status, "CERTIFIED") == 0) {
        int n = rec->nports;
        int d = 2 * n;
        fprintf(fp, "def nPorts : Nat := %d\n", n);
        fprintf(fp, "def scaleQ : Int := %" PRId64 "\n", rec->s_scale_q);
        fprintf(fp, "def muPsd : Int := %" PRId64 "\n", rec->psd.mu_psd);
        fprintf(fp, "def deltaQ : Int := %" PRId64 "\n", rec->psd.delta_round);
        fprintf(fp, "def recipEps : Int := %" PRId64 "\n\n", rec->recip_eps_q);
        emit_lean_i64_matrix(fp, "matR", n, rec->R_q);
        emit_lean_i64_matrix(fp, "matX", n, rec->X_q);
        emit_lean_i64_matrix(fp, "matM", d, rec->psd.A);
        emit_lean_i64_matrix(fp, "matL", d, rec->psd.L);

        fprintf(fp,
                "theorem passivity_psd_sound : ∀ v : Fin %d → Int,\n"
                "    muPsd * EleCert.vecNormSq %d v ≤ EleCert.matQuad %d (EleCert.matAt matM) v :=\n"
                "  EleCert.checkCholeskySDD_sound %d matM matL muPsd (by decide)\n\n",
                d, d, d, d);
        fputs(
            "theorem multiport_passivity_certified : ∀ u v : Fin nPorts → Int,\n"
            "    EleCert.vecNormSq nPorts (fun k => EleCert.matMulVec nPorts (EleCert.matAt matR) u k - EleCert.matMulVec nPorts (EleCert.matAt matX) v k) +\n"
            "    EleCert.vecNormSq nPorts (fun k => EleCert.matMulVec nPorts (EleCert.matAt matX) u k + EleCert.matMulVec nPorts (EleCert.matAt matR) v k) ≤\n"
            "      (scaleQ * scaleQ - muPsd) * (EleCert.vecNormSq nPorts u + EleCert.vecNormSq nPorts v) ∧\n"
            "    EleCert.vecNormSq nPorts (fun k => EleCert.matMulVec nPorts (EleCert.matAt matR) u k - EleCert.matMulVec nPorts (EleCert.matAt matX) v k) +\n"
            "    EleCert.vecNormSq nPorts (fun k => EleCert.matMulVec nPorts (EleCert.matAt matX) u k + EleCert.matMulVec nPorts (EleCert.matAt matR) v k) ≤\n"
            "      scaleQ * scaleQ * (EleCert.vecNormSq nPorts u + EleCert.vecNormSq nPorts v) :=\n"
            "  EleCert.certified_multiport_passivity nPorts scaleQ muPsd deltaQ\n"
            "    (EleCert.matAt matR) (EleCert.matAt matX) matM matL\n"
            "    (by decide)\n"
            "    (EleCert.checkPassivityMat_sound nPorts scaleQ matR matX matM (by decide))\n"
            "    (by decide) (by decide)\n\n",
            fp);
        fprintf(fp,
                "theorem reciprocity_certified : ∀ i j : Fin nPorts,\n"
                "    EleCert.absInt (EleCert.matAt matR i j - EleCert.matAt matR j i) ≤ recipEps ∧\n"
                "    EleCert.absInt (EleCert.matAt matX i j - EleCert.matAt matX j i) ≤ recipEps :=\n"
                "  EleCert.reciprocity_symmetry_bound nPorts matR matX recipEps (by decide)\n\n"
                "#print axioms passivity_psd_sound\n"
                "#print axioms multiport_passivity_certified\n"
                "#print axioms reciprocity_certified\n");
    } else if (strcmp(rec->pillar, "ckt") == 0 && rec->has_bracket) {
        fprintf(fp,
                "def ivCorner : Interval1D := { lo := %" PRId64 ", hi := %" PRId64 " }\n"
                "def vLoLimit : Int := %" PRId64 "\n"
                "def vHiLimit : Int := %" PRId64 "\n"
                "def ivEyeLo  : Interval1D := { lo := 0, hi := 0 }\n"
                "def ivEyeHi  : Interval1D := { lo := 1, hi := 1 }\n\n"
                "theorem soa_corner_certified :\n"
                "    (∀ x, ivCorner.contains x → vLoLimit ≤ x ∧ x ≤ vHiLimit) ∧\n"
                "    (∀ xLo xHi, ivEyeLo.contains xLo → ivEyeHi.contains xHi → (0 : Int) < xHi - xLo) :=\n"
                "  EleCert.soa_and_eye_enclosure_cert ivCorner ivEyeLo ivEyeHi vLoLimit vHiLimit 0\n"
                "    ⟨by decide, by decide⟩ (by decide)\n\n"
                "#print axioms soa_corner_certified\n",
                rec->lb_int, rec->ub_int, rec->limit_lo_int, rec->limit_hi_int);
    } else if (rec->has_motor) {
        fprintf(fp,
                "def tBracket : Interval1D := { lo := %" PRId64 ", hi := %" PRId64 " }\n\n"
                "theorem ipmsm_certified :\n"
                "    (%" PRId64 " : Int) * %" PRId64 " + %" PRId64 " * %" PRId64 " ≤ %" PRId64 " * %" PRId64 " ∧\n"
                "    (%" PRId64 " : Int) * %" PRId64 " + %" PRId64 " * %" PRId64 " ≤ %" PRId64 " * %" PRId64 " ∧\n"
                "    tBracket.lo ≤ %" PRId64 " * (%" PRId64 " * %" PRId64 " - %" PRId64 " * %" PRId64 ") ∧\n"
                "    %" PRId64 " * (%" PRId64 " * %" PRId64 " - %" PRId64 " * %" PRId64 ") ≤ tBracket.hi :=\n"
                "  EleCert.ipmsm_operating_envelope_cert\n"
                "    (%" PRId64 ") (%" PRId64 ") (%" PRId64 ") (%" PRId64 ") (%" PRId64 ") (%" PRId64 ")\n"
                "    (%" PRId64 ") (%" PRId64 ") (%" PRId64 ") tBracket\n"
                "    (by decide) (by decide) ⟨by decide, by decide⟩\n\n"
                "#print axioms ipmsm_certified\n",
                rec->lb_int, rec->ub_int,
                rec->id_int, rec->id_int, rec->iq_int, rec->iq_int, rec->imax_int, rec->imax_int,
                rec->vd_int, rec->vd_int, rec->vq_int, rec->vq_int, rec->vmax_int, rec->vmax_int,
                rec->two_p, rec->psid_int, rec->iq_int, rec->psiq_int, rec->id_int,
                rec->two_p, rec->psid_int, rec->iq_int, rec->psiq_int, rec->id_int,
                rec->id_int, rec->iq_int, rec->vd_int, rec->vq_int,
                rec->psid_int, rec->psiq_int, rec->imax_int, rec->vmax_int, rec->two_p);
    } else if (strcmp(rec->kind, "maxwell_poynting_bound") == 0 &&
               rec->has_psd && rec->has_bracket && strcmp(rec->status, "CERTIFIED") == 0) {
        int d = rec->psd.dim;
        int k = d - 1;
        fprintf(fp, "def nVar : Nat := %d\n", k);
        fprintf(fp, "def muPsd : Int := %" PRId64 "\n", rec->psd.mu_psd);
        fprintf(fp, "def localUbQ : Int := %" PRId64 "\n", rec->lb_int);
        fprintf(fp, "def globalUbQ : Int := %" PRId64 "\n\n", rec->ub_int);
        emit_lean_i64_matrix(fp, "matM", d, rec->psd.A);
        emit_lean_i64_matrix(fp, "matL", d, rec->psd.L);
        fprintf(fp,
                "theorem maxwell_psd_sound : ∀ v : Fin %d → Int,\n"
                "    muPsd * EleCert.vecNormSq %d v ≤ EleCert.matQuad %d (EleCert.matAt matM) v :=\n"
                "  EleCert.checkCholeskySDD_sound %d matM matL muPsd (by decide)\n\n"
                "theorem maxwell_poynting_bound_certified :\n"
                "    (∀ t : Fin nVar → Int,\n"
                "      -(EleCert.sumFin nVar (fun j => 2 * EleCert.matAt matM ⟨0, Nat.add_pos_left Nat.zero_lt_one nVar⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) -\n"
                "      EleCert.sumFin nVar (fun i => EleCert.sumFin nVar (fun j =>\n"
                "        t i * EleCert.matAt matM ⟨1 + i.val, Nat.add_lt_add_left i.isLt 1⟩ ⟨1 + j.val, Nat.add_lt_add_left j.isLt 1⟩ * t j)) ≤\n"
                "        EleCert.matAt matM ⟨0, Nat.add_pos_left Nat.zero_lt_one nVar⟩ ⟨0, Nat.add_pos_left Nat.zero_lt_one nVar⟩) ∧\n"
                "    localUbQ ≤ globalUbQ :=\n"
                "  EleCert.certified_maxwell_poynting_bound nVar matM matL muPsd localUbQ globalUbQ\n"
                "    (by decide) (by decide) (by decide)\n\n"
                "#print axioms maxwell_psd_sound\n"
                "#print axioms maxwell_poynting_bound_certified\n",
                d, d, d, d);
    } else if (strcmp(rec->kind, "kyp_passivity_enforce") == 0 &&
               rec->has_psd && rec->has_bracket && strcmp(rec->status, "CERTIFIED") == 0) {
        int d = rec->psd.dim;
        fprintf(fp, "def dimKyp : Nat := %d\n", d);
        fprintf(fp, "def muPsd : Int := %" PRId64 "\n", rec->psd.mu_psd);
        fprintf(fp, "def sigmaRepQ : Int := %" PRId64 "\n", rec->lb_int);
        fprintf(fp, "def gammaQ : Int := %" PRId64 "\n\n", rec->ub_int);
        emit_lean_i64_matrix(fp, "matM", d, rec->psd.A);
        emit_lean_i64_matrix(fp, "matL", d, rec->psd.L);
        fprintf(fp,
                "theorem kyp_passivity_certified :\n"
                "    (∀ w : Fin dimKyp → Int, 0 ≤ EleCert.matQuad dimKyp (EleCert.matAt matM) w) ∧\n"
                "    sigmaRepQ ≤ gammaQ :=\n"
                "  EleCert.certified_kyp_passivity dimKyp matM matL muPsd sigmaRepQ gammaQ\n"
                "    (by decide) (by decide) (by decide)\n\n"
                "#print axioms kyp_passivity_certified\n");
    } else if (strcmp(rec->kind, "acopf_sos2") == 0 &&
               rec->has_opf && rec->has_psd && rec->has_bracket) {
        int d = rec->psd.dim;
        fprintf(fp, "def shor1LB : Int := %" PRId64 "\n", rec->limit_lo_int);
        fprintf(fp, "def cDual   : Int := %" PRId64 "\n", rec->c_dual_int);
        fprintf(fp, "def muPsd   : Int := %" PRId64 "\n", rec->psd.mu_psd);
        fprintf(fp, "def wMax    : Int := %" PRId64 "\n", rec->w_max_int);
        fprintf(fp, "def dRnd    : Int := %" PRId64 "\n", rec->opf_delta_round);
        fprintf(fp, "def cLB     : Int := %" PRId64 "\n", rec->c_lb_int);
        fprintf(fp, "def cUB     : Int := %" PRId64 "\n\n", rec->c_ub_int);
        emit_lean_i64_matrix(fp, "matA", d, rec->psd.A);
        emit_lean_i64_matrix(fp, "matL", d, rec->psd.L);
        fprintf(fp,
                "theorem sos2_psd_sound : ∀ v : Fin %d → Int,\n"
                "    muPsd * EleCert.vecNormSq %d v ≤ EleCert.matQuad %d (EleCert.matAt matA) v :=\n"
                "  EleCert.checkCholeskySDD_sound %d matA matL muPsd (by decide)\n\n"
                "theorem sos2_hierarchy_certified (z : Fin %d → Int) (C_primal : Int)\n"
                "    (hLag : cDual + EleCert.matQuad %d (EleCert.matAt matA) z ≤ C_primal)\n"
                "    (hNorm : EleCert.vecNormSq %d z ≤ wMax)\n"
                "    (hPrimalUb : C_primal ≤ cUB) :\n"
                "    shor1LB ≤ cLB ∧ ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal :=\n"
                "  EleCert.certified_sos2_lower_bound %d shor1LB cDual muPsd wMax dRnd cLB cUB matA matL z C_primal\n"
                "    (by decide) (by decide) (by decide) hLag (EleCert.vecNormSq_nonneg %d z) hNorm (by decide) hPrimalUb\n\n"
                "#print axioms sos2_psd_sound\n"
                "#print axioms sos2_hierarchy_certified\n",
                d, d, d, d, d, d, d, d, d);
    } else if (rec->has_opf && rec->has_psd) {
        int d = rec->psd.dim;
        fprintf(fp, "def cDual : Int := %" PRId64 "\n", rec->c_dual_int);
        fprintf(fp, "def muPsd : Int := %" PRId64 "\n", rec->psd.mu_psd);
        fprintf(fp, "def wMax  : Int := %" PRId64 "\n", rec->w_max_int);
        fprintf(fp, "def dRnd  : Int := %" PRId64 "\n", rec->opf_delta_round);
        fprintf(fp, "def cLB   : Int := %" PRId64 "\n", rec->c_lb_int);
        fprintf(fp, "def cUB   : Int := %" PRId64 "\n\n", rec->c_ub_int);
        emit_lean_i64_matrix(fp, "matA", d, rec->psd.A);
        emit_lean_i64_matrix(fp, "matL", d, rec->psd.L);
        fprintf(fp,
                "theorem ac_opf_psd_sound : ∀ v : Fin %d → Int,\n"
                "    muPsd * EleCert.vecNormSq %d v ≤ EleCert.matQuad %d (EleCert.matAt matA) v :=\n"
                "  EleCert.checkCholeskySDD_sound %d matA matL muPsd (by decide)\n\n"
                "theorem ac_opf_bracket_certified (x : Fin %d → Int) (C_primal : Int)\n"
                "    (hLag : cDual + EleCert.matQuad %d (EleCert.matAt matA) x ≤ C_primal)\n"
                "    (hNorm : EleCert.vecNormSq %d x ≤ wMax)\n"
                "    (hPrimalUb : C_primal ≤ cUB) :\n"
                "    ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal :=\n"
                "  EleCert.ac_opf_bracket_valid cLB C_primal cUB\n"
                "    (by\n"
                "      have hlb := EleCert.ac_opf_anytime_dual_lower_bound\n"
                "        C_primal cDual muPsd (EleCert.vecNormSq %d x) wMax dRnd\n"
                "        (EleCert.matQuad %d (EleCert.matAt matA) x)\n"
                "        hLag (ac_opf_psd_sound x) (EleCert.vecNormSq_nonneg %d x) hNorm (by decide)\n"
                "      have hEq : cLB = cDual + EleCert.negPart muPsd * wMax - dRnd := by decide\n"
                "      omega)\n"
                "    hPrimalUb\n\n"
                "#print axioms ac_opf_psd_sound\n"
                "#print axioms ac_opf_bracket_certified\n",
                d, d, d, d, d, d, d, d, d, d);
    } else if (rec->has_emt && strcmp(rec->status, "CERTIFIED") == 0) {
        fprintf(fp,
                "def vKin  : Int := %" PRId64 "\n"
                "def vPot  : Int := %" PRId64 "\n"
                "def vCrit : Int := %" PRId64 "\n"
                "def vErr  : Int := %" PRId64 "\n\n"
                "theorem grid_stability_certified :\n"
                "    vPot < vCrit ∧ 0 < vCrit - (vKin + vPot + vErr) :=\n"
                "  EleCert.grid_lyapunov_stability_basin vKin vPot vCrit vErr\n"
                "    (by decide) (by decide) (by decide)\n\n"
                "#print axioms grid_stability_certified\n",
                rec->v_kin_int, rec->v_pot_int, rec->v_crit_int, rec->v_err_int);
    } else {
        fclose(fp);
        return -1;
    }

    fprintf(fp, "\nend %s\n", mod_name);
    int ferr = ferror(fp);
    fclose(fp);
    return (ferr == 0) ? 0 : -1;
}

int ele_cert_emit_chordal_lean(const char *cert_path,
                               const char *lean_out_path,
                               const char *mod_name) {
    if (cert_path == NULL || lean_out_path == NULL || mod_name == NULL) return -1;
    ele_full_cert_t fc;
    if (load_full_cert_auto(cert_path, &fc) != 0) return -1;

    if (!fc.has_chordal || !fc.rec.has_opf ||
        strcmp(fc.rec.status, "CERTIFIED") != 0 ||
        fc.chordal_dim < 2 || fc.chordal_dim > 64 ||
        fc.num_cliques < 1 || fc.num_cliques > 64) {
        ele_full_cert_free(&fc);
        return -1;
    }

    int dim = fc.chordal_dim;
    int n_cl = fc.num_cliques;
    int64_t *A_full = (int64_t *)calloc((size_t)dim * (size_t)dim, sizeof(int64_t));
    if (A_full == NULL) { ele_full_cert_free(&fc); return -1; }

    for (int i = 0; i < fc.n_sparse; ++i) {
        int r = fc.A_sparse[i].r, c = fc.A_sparse[i].c;
        if (r >= 0 && r < dim && c >= 0 && c < dim) {
            A_full[(size_t)r * (size_t)dim + (size_t)c] = fc.A_sparse[i].val;
            A_full[(size_t)c * (size_t)dim + (size_t)r] = fc.A_sparse[i].val;
        }
    }

    FILE *fp = fopen(lean_out_path, "w");
    if (fp == NULL) { free(A_full); ele_full_cert_free(&fc); return -1; }
    fprintf(fp, "/-\n  Standalone Zero-FP-Trust Chordal Certificate: %s\n  Pillar: %s | Kind: %s | SHA256: %s\n-/\n",
            mod_name, fc.rec.pillar, fc.rec.kind, fc.rec.input_sha256);
    fprintf(fp, "import EleCert\nimport Flow.Interval\n\nnamespace %s\n\nopen Flow.Interval\n\n", mod_name);
    fprintf(fp, "def dimN : Nat := %d\n", dim);
    fprintf(fp, "def cDual : Int := %" PRId64 "\n", fc.rec.c_dual_int);
    fprintf(fp, "def dRnd  : Int := %" PRId64 "\n", fc.rec.opf_delta_round);
    fprintf(fp, "def cLB   : Int := %" PRId64 "\n", fc.rec.c_lb_int);
    fprintf(fp, "def cUB   : Int := %" PRId64 "\n\n", fc.rec.c_ub_int);
    emit_lean_i64_matrix(fp, "matA", dim, A_full);
    free(A_full);

    fprintf(fp, "def cliques : List EleCert.CliqueCert := [\n");
    for (int c = 0; c < n_cl; ++c) {
        const ele_chordal_clique_t *clq = &fc.cliques[c];
        int dk = clq->dk;
        if (dk < 2 || dk > 32) {
            fclose(fp); ele_full_cert_free(&fc); return -1;
        }
        fprintf(fp, "  { vars := [");
        for (int a = 0; a < dk; ++a) fprintf(fp, "%s%d", a ? ", " : "", clq->vars[a]);
        fprintf(fp, "],\n    sMat := [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(fp, "      [");
            for (int b = 0; b < dk; ++b) fprintf(fp, "%s%" PRId64, b ? ", " : "", clq->S_int[a * dk + b]);
            fprintf(fp, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(fp, "    ],\n    lMat := [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(fp, "      [");
            for (int b = 0; b < dk; ++b) fprintf(fp, "%s%" PRId64, b ? ", " : "", clq->L_int[a * dk + b]);
            fprintf(fp, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(fp, "    ],\n    muPsd := %" PRId64 ",\n    wMax := %" PRId64 " }%s\n",
                clq->mu_psd, clq->w_max_int, (c + 1 < n_cl) ? "," : "");
    }
    fprintf(fp, "]\n\n");
    fprintf(fp,
            "theorem grid_chordal_certified (v : Fin dimN → Int) (C_primal : Int)\n"
            "    (hLag : cDual + EleCert.matQuad dimN (EleCert.matAt matA) v ≤ C_primal)\n"
            "    (hNorm : ∀ c : Fin cliques.length,\n"
            "      EleCert.vecNormSq (cliques[c].vars.length) (EleCert.projCliqueVec dimN cliques[c] v) ≤ cliques[c].wMax)\n"
            "    (hPrimalUb : C_primal ≤ cUB) :\n"
            "    ({ lo := cLB, hi := cUB } : Interval1D).contains C_primal :=\n"
            "  EleCert.certified_chordal_acopf dimN matA cliques cDual dRnd cLB cUB v C_primal\n"
            "    (by decide) (by decide) hLag hNorm (by decide) hPrimalUb\n\n"
            "#print axioms grid_chordal_certified\n\n"
            "end %s\n", mod_name);

    int ferr = ferror(fp);
    fclose(fp);
    ele_full_cert_free(&fc);
    return (ferr == 0) ? 0 : -1;
}
