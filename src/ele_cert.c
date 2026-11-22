/* SPDX-License-Identifier: MIT */
#include "ele_cert.h"
#include "flow_cholesky_sdd.h"
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
    if (n <= 0 || A == NULL || b == NULL || x == NULL) return -1;
    size_t nn = (size_t)n;
    double *M = (double *)malloc(nn * nn * sizeof(double));
    double *rhs = (double *)malloc(nn * sizeof(double));
    if (M == NULL || rhs == NULL) {
        free(M);
        free(rhs);
        return -1;
    }
    memcpy(M, A, nn * nn * sizeof(double));
    memcpy(rhs, b, nn * sizeof(double));

    for (int k = 0; k < n; ++k) {
        int piv = k;
        double maxv = fabs(M[(size_t)k * nn + (size_t)k]);
        for (int i = k + 1; i < n; ++i) {
            double v = fabs(M[(size_t)i * nn + (size_t)k]);
            if (v > maxv) {
                maxv = v;
                piv = i;
            }
        }
        if (maxv < 1e-18 || !isfinite(maxv)) {
            free(M);
            free(rhs);
            return -1;
        }
        if (piv != k) {
            for (int j = k; j < n; ++j) {
                double tmp = M[(size_t)k * nn + (size_t)j];
                M[(size_t)k * nn + (size_t)j] = M[(size_t)piv * nn + (size_t)j];
                M[(size_t)piv * nn + (size_t)j] = tmp;
            }
            double tb = rhs[k];
            rhs[k] = rhs[piv];
            rhs[piv] = tb;
        }
        double invd = 1.0 / M[(size_t)k * nn + (size_t)k];
        for (int i = k + 1; i < n; ++i) {
            double mik = M[(size_t)i * nn + (size_t)k];
            if (fabs(mik) < 1e-18) {
                M[(size_t)i * nn + (size_t)k] = 0.0;
                continue;
            }
            double f = mik * invd;
            M[(size_t)i * nn + (size_t)k] = 0.0;
            for (int j = k + 1; j < n; ++j) {
                double mkj = M[(size_t)k * nn + (size_t)j];
                if (fabs(mkj) < 1e-18) continue;
                M[(size_t)i * nn + (size_t)j] -= f * mkj;
            }
            rhs[i] -= f * rhs[k];
        }
    }

    for (int i = n - 1; i >= 0; --i) {
        double s = rhs[i];
        for (int j = i + 1; j < n; ++j) {
            s -= M[(size_t)i * nn + (size_t)j] * x[j];
        }
        x[i] = s / M[(size_t)i * nn + (size_t)i];
        if (!isfinite(x[i])) {
            free(M);
            free(rhs);
            return -1;
        }
    }
    free(M);
    free(rhs);
    return 0;
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
    if (n <= 0 || n > ELE_CERT_MAX_DIM || A_sym == NULL ||
        L_out == NULL || mu_psd_out == NULL) return -1;
    size_t nn = (size_t)n;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (A_sym[(size_t)i * nn + (size_t)j] != A_sym[(size_t)j * nn + (size_t)i]) {
                return -1;
            }
        }
    }

    double A_fp[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double L_fp[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double evals[ELE_CERT_MAX_DIM];
    double max_diag = 0.0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double v = (double)A_sym[(size_t)i * nn + (size_t)j];
            A_fp[(size_t)i * nn + (size_t)j] = v;
            if (i == j && fabs(v) > max_diag) max_diag = fabs(v);
        }
    }
    if (ele_sym_jacobi_eigs(n, A_fp, evals, NULL) != 0) return -1;
    double lmin = evals[0];
    double eps = fmax(1.0, 1e-7 * (max_diag + 1.0));

    int chol_ok = 0;
    for (int attempt = 0; attempt < 8; ++attempt) {
        double shift = lmin - eps;
        memset(L_fp, 0, nn * nn * sizeof(double));
        chol_ok = 1;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                double s = A_fp[(size_t)i * nn + (size_t)j];
                if (i == j) s -= shift;
                for (int k = 0; k < j; ++k) {
                    s -= L_fp[(size_t)i * nn + (size_t)k] * L_fp[(size_t)j * nn + (size_t)k];
                }
                if (i == j) {
                    if (s <= 0.0) {
                        chol_ok = 0;
                        break;
                    }
                    L_fp[(size_t)i * nn + (size_t)j] = sqrt(s);
                } else {
                    L_fp[(size_t)i * nn + (size_t)j] = s / L_fp[(size_t)j * nn + (size_t)j];
                }
            }
            if (!chol_ok) break;
        }
        if (chol_ok) break;
        eps *= 4.0;
    }

    if (!chol_ok) {
        memset(L_out, 0, nn * nn * sizeof(int64_t));
    } else {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                L_out[(size_t)i * nn + (size_t)j] =
                    (j <= i) ? (int64_t)llround(L_fp[(size_t)i * nn + (size_t)j]) : 0;
            }
        }
    }

    return flow_cholesky_sdd_floor_i64(n, ELE_CERT_MAX_DIM, A_sym, L_out, mu_psd_out);
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
    const char *p = json;
    while ((p = strstr(p, pat)) != NULL) {
        const char *q = p + strlen(pat);
        while (*q != '\0' && isspace((unsigned char)*q)) ++q;
        if (*q == ':') {
            ++q;
            while (*q != '\0' && isspace((unsigned char)*q)) ++q;
            return q;
        }
        p = q;
    }
    return NULL;
}

static int json_get_str(const char *json, const char *key, char *out, size_t cap) {
    const char *v = find_json_key(json, key);
    if (v == NULL || *v != '"') return -1;
    ++v;
    size_t k = 0;
    while (*v != '\0' && *v != '"') {
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

static int verify_chordal_cert(char *buf, const ele_cert_record_t *rec) {
    int64_t dim64 = 0, ncl64 = 0, min_mu_rec = 0;
    if (json_get_i64(buf, "dim", &dim64) != 0 ||
        json_get_i64(buf, "num_cliques", &ncl64) != 0 ||
        json_get_i64(buf, "min_clique_mu_psd", &min_mu_rec) != 0) {
        return -1;
    }
    if (dim64 < 2 || dim64 > 512 || ncl64 < 1 || ncl64 > 256) return -1;
    int dim = (int)dim64;
    int n_cliques = (int)ncl64;

    if (!rec->has_opf || rec->w_max_int <= 0 || rec->opf_delta_round < 0) return -1;

    size_t mat_sz = (size_t)dim * (size_t)dim;
    ele_i128 *A_target = (ele_i128 *)calloc(mat_sz, sizeof(ele_i128));
    ele_i128 *A_sum = (ele_i128 *)calloc(mat_sz, sizeof(ele_i128));
    uint8_t *seen = (uint8_t *)calloc(mat_sz, sizeof(uint8_t));
    if (A_target == NULL || A_sum == NULL || seen == NULL) {
        free(A_target); free(A_sum); free(seen);
        return -1;
    }

    const char *as = find_json_key(buf, "A_sparse");
    if (as == NULL || *as != '[') {
        free(A_target); free(A_sum); free(seen);
        return -1;
    }
    ++as;
    while (*as != '\0') {
        while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
        if (*as == ']') break;
        if (*as != '[') {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        ++as;
        int64_t triple[3] = {0, 0, 0};
        for (int t = 0; t < 3; ++t) {
            while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
            char *endp = NULL;
            long long val = strtoll(as, &endp, 10);
            if (endp == as) {
                free(A_target); free(A_sum); free(seen);
                return -1;
            }
            triple[t] = (int64_t)val;
            as = endp;
        }
        while (*as != '\0' && isspace((unsigned char)*as)) ++as;
        if (*as != ']') {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        ++as;
        if (triple[0] < 0 || triple[0] >= dim || triple[1] < triple[0] || triple[1] >= dim) {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        size_t r = (size_t)triple[0], c = (size_t)triple[1];
        if (seen[r * (size_t)dim + c]) {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        seen[r * (size_t)dim + c] = 1U;
        A_target[r * (size_t)dim + c] = (ele_i128)triple[2];
        A_target[c * (size_t)dim + r] = (ele_i128)triple[2];
    }
    if (*as != ']') {
        free(A_target); free(A_sum); free(seen);
        return -1;
    }

    const char *cl = find_json_key(buf, "cliques");
    if (cl == NULL || *cl != '[') {
        free(A_target); free(A_sum); free(seen);
        return -1;
    }
    char *cur = (char *)(uintptr_t)(cl + 1);
    int64_t min_mu = 0;
    ele_i128 neg_pen = 0;
    int64_t S_int[32 * 32];
    int64_t L_int[32 * 32];
    int vars[32];

    for (int c = 0; c < n_cliques; ++c) {
        while (*cur != '\0' && *cur != '{') ++cur;
        if (*cur != '{') {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
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
        if (*obj_end != '}') {
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        *obj_end = '\0';

        int dk = 0;
        int64_t mu_c = 0;
        int64_t w_c = rec->w_max_int;
        (void)json_get_i64(obj_start, "w_max_int", &w_c);
        if (w_c <= 0 ||
            json_get_i64_list_bounded(obj_start, "vars", 32, &dk, vars) != 0 ||
            dk < 2 || dk > 32 ||
            json_get_i64(obj_start, "mu_psd", &mu_c) != 0 ||
            json_get_i64_mat(obj_start, "S_int", dk, S_int) != 0 ||
            json_get_i64_mat(obj_start, "L_int", dk, L_int) != 0) {
            *obj_end = '}';
            free(A_target); free(A_sum); free(seen);
            return -1;
        }
        *obj_end = '}';
        cur = obj_end + 1;

        for (int a = 0; a < dk; ++a) {
            if (vars[a] < 0 || vars[a] >= dim) {
                free(A_target); free(A_sum); free(seen);
                return -1;
            }
            for (int b = a + 1; b < dk; ++b) {
                if (vars[a] == vars[b]) {
                    free(A_target); free(A_sum); free(seen);
                    return -1;
                }
            }
        }
        if (ele_verify_cholesky_sdd_i64(dk, S_int, L_int, mu_c) != 0) {
            free(A_target); free(A_sum); free(seen);
            return -16;
        }
        for (int a = 0; a < dk; ++a) {
            size_t va = (size_t)vars[a];
            for (int b = 0; b < dk; ++b) {
                size_t vb = (size_t)vars[b];
                A_sum[va * (size_t)dim + vb] += (ele_i128)S_int[(size_t)a * (size_t)dk + (size_t)b];
            }
        }
        if (c == 0 || mu_c < min_mu) min_mu = mu_c;
        if (mu_c < 0) neg_pen += (ele_i128)mu_c * (ele_i128)w_c;
    }

    for (size_t p = 0; p < mat_sz; ++p) {
        if (A_sum[p] != A_target[p]) {
            free(A_target); free(A_sum); free(seen);
            return -17;
        }
    }
    free(A_target); free(A_sum); free(seen);

    if (min_mu != min_mu_rec) return -18;
    ele_i128 expected_lb = (ele_i128)rec->c_dual_int +
                           neg_pen -
                           (ele_i128)rec->opf_delta_round;
    if ((ele_i128)rec->c_lb_int != expected_lb) return -19;
    if (strcmp(rec->status, "CERTIFIED") == 0 && rec->c_lb_int > rec->c_ub_int) return -20;
    return 0;
}

int ele_cert_verify_file(const char *cert_path, int verify_input_hash) {
    if (cert_path == NULL) return -1;
    FILE *fp = fopen(cert_path, "rb");
    if (fp == NULL) return -1;
    if (fseek(fp, 0L, SEEK_END) != 0) { fclose(fp); return -1; }
    long fsz = ftell(fp);
    if (fsz <= 0 || fsz > 16L * 1024L * 1024L) { fclose(fp); return -1; }
    rewind(fp);
    char *buf = (char *)malloc((size_t)fsz + 1U);
    if (buf == NULL) { fclose(fp); return -1; }
    size_t nr = fread(buf, 1U, (size_t)fsz, fp);
    fclose(fp);
    buf[nr] = '\0';

    ele_cert_record_t rec;
    memset(&rec, 0, sizeof(rec));
    char fmt[32] = {0};
    if (json_get_str(buf, "format", fmt, sizeof(fmt)) != 0 ||
        strcmp(fmt, "ELECERT-1.0") != 0) {
        free(buf);
        return -1;
    }
    if (json_get_str(buf, "pillar", rec.pillar, sizeof(rec.pillar)) != 0 ||
        json_get_str(buf, "kind", rec.kind, sizeof(rec.kind)) != 0 ||
        json_get_str(buf, "status", rec.status, sizeof(rec.status)) != 0 ||
        json_get_str(buf, "input_path", rec.input_path, sizeof(rec.input_path)) != 0 ||
        json_get_str(buf, "input_sha256", rec.input_sha256, sizeof(rec.input_sha256)) != 0) {
        free(buf);
        return -1;
    }

    if (verify_input_hash && strlen(rec.input_path) > 0U &&
        strcmp(rec.input_path, "internal") != 0 &&
        strlen(rec.input_sha256) == 64U) {
        char actual_hash[65];
        if (ele_sha256_file(rec.input_path, actual_hash) == 0) {
            if (strcmp(actual_hash, rec.input_sha256) != 0) {
                free(buf);
                return -2;
            }
        }
    }

    int64_t tmp = 0;
    int has_chordal = 0;
    if (json_get_i64(buf, "has_psd", &tmp) == 0) rec.has_psd = (int)tmp;
    if (json_get_i64(buf, "has_chordal", &tmp) == 0) has_chordal = (int)tmp;
    if (json_get_i64(buf, "has_sparam", &tmp) == 0) rec.has_sparam = (int)tmp;
    if (json_get_i64(buf, "has_bracket", &tmp) == 0) rec.has_bracket = (int)tmp;
    if (json_get_i64(buf, "has_motor", &tmp) == 0) rec.has_motor = (int)tmp;
    if (json_get_i64(buf, "has_opf", &tmp) == 0) rec.has_opf = (int)tmp;
    if (json_get_i64(buf, "has_emt", &tmp) == 0) rec.has_emt = (int)tmp;

    if (rec.has_psd) {
        int64_t dim64 = 0;
        if (json_get_i64(buf, "dim", &dim64) != 0 || dim64 <= 0 || dim64 > ELE_CERT_MAX_DIM ||
            json_get_i64(buf, "scale_q", &rec.psd.scale_q) != 0 ||
            json_get_i64(buf, "mu_psd", &rec.psd.mu_psd) != 0 ||
            json_get_i64(buf, "delta_round", &rec.psd.delta_round) != 0 ||
            json_get_i64(buf, "mu_cert", &rec.psd.mu_cert) != 0) {
            free(buf);
            return -1;
        }
        rec.psd.dim = (int)dim64;
        if (rec.psd.mu_cert != rec.psd.mu_psd - rec.psd.delta_round) {
            free(buf);
            return -3;
        }
        if (json_get_i64_mat(buf, "A_int", rec.psd.dim, rec.psd.A) != 0 ||
            json_get_i64_mat(buf, "L_int", rec.psd.dim, rec.psd.L) != 0) {
            free(buf);
            return -1;
        }
        if (ele_verify_cholesky_sdd_i64(rec.psd.dim, rec.psd.A, rec.psd.L, rec.psd.mu_psd) != 0) {
            free(buf);
            return -4;
        }
    }

    if (rec.has_sparam) {
        int64_t np64 = 0;
        if (json_get_i64(buf, "nports", &np64) != 0 || np64 <= 0 || np64 > ELE_CERT_MAX_PORTS ||
            json_get_i64(buf, "s_scale_q", &rec.s_scale_q) != 0 ||
            json_get_i64(buf, "recip_eps_q", &rec.recip_eps_q) != 0) {
            free(buf);
            return -1;
        }
        rec.nports = (int)np64;
        if (json_get_i64_mat(buf, "R_q", rec.nports, rec.R_q) != 0 ||
            json_get_i64_mat(buf, "X_q", rec.nports, rec.X_q) != 0) {
            free(buf);
            return -1;
        }
        if (rec.has_psd) {
            if (rec.psd.dim != 2 * rec.nports) { free(buf); return -5; }
            int n = rec.nports;
            int d = 2 * n;
            ele_i128 Q2 = (ele_i128)rec.s_scale_q * (ele_i128)rec.s_scale_q;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    ele_i128 aij = (i == j) ? Q2 : 0;
                    ele_i128 bij = 0;
                    for (int k = 0; k < n; ++k) {
                        ele_i128 rki = rec.R_q[(size_t)k * (size_t)n + (size_t)i];
                        ele_i128 rkj = rec.R_q[(size_t)k * (size_t)n + (size_t)j];
                        ele_i128 xki = rec.X_q[(size_t)k * (size_t)n + (size_t)i];
                        ele_i128 xkj = rec.X_q[(size_t)k * (size_t)n + (size_t)j];
                        aij -= (rki * rkj + xki * xkj);
                        bij += (rki * xkj - xki * rkj);
                    }
                    if ((ele_i128)rec.psd.A[(size_t)i * (size_t)d + (size_t)j] != aij ||
                        (ele_i128)rec.psd.A[(size_t)(n + i) * (size_t)d + (size_t)(n + j)] != aij ||
                        (ele_i128)rec.psd.A[(size_t)i * (size_t)d + (size_t)(n + j)] != bij ||
                        (ele_i128)rec.psd.A[(size_t)(n + i) * (size_t)d + (size_t)j] != -bij) {
                        free(buf);
                        return -6;
                    }
                }
            }
            if (strcmp(rec.status, "CERTIFIED") == 0 && rec.psd.mu_cert < 0) {
                free(buf);
                return -7;
            }
            if (strcmp(rec.status, "VETOED") == 0 && rec.psd.mu_cert >= 0) {
                free(buf);
                return -8;
            }
        }
    }

    if (rec.has_bracket) {
        if (json_get_i64(buf, "lb_int", &rec.lb_int) != 0 ||
            json_get_i64(buf, "ub_int", &rec.ub_int) != 0 ||
            json_get_i64(buf, "limit_lo_int", &rec.limit_lo_int) != 0 ||
            json_get_i64(buf, "limit_hi_int", &rec.limit_hi_int) != 0) {
            free(buf);
            return -1;
        }
        if (rec.lb_int > rec.ub_int) { free(buf); return -9; }
        if (strcmp(rec.status, "CERTIFIED") == 0 &&
            (rec.limit_lo_int > rec.lb_int || rec.ub_int > rec.limit_hi_int)) {
            free(buf);
            return -10;
        }
    }

    if (rec.has_motor) {
        if (json_get_i64(buf, "id_int", &rec.id_int) != 0 ||
            json_get_i64(buf, "iq_int", &rec.iq_int) != 0 ||
            json_get_i64(buf, "vd_int", &rec.vd_int) != 0 ||
            json_get_i64(buf, "vq_int", &rec.vq_int) != 0 ||
            json_get_i64(buf, "psid_int", &rec.psid_int) != 0 ||
            json_get_i64(buf, "psiq_int", &rec.psiq_int) != 0 ||
            json_get_i64(buf, "imax_int", &rec.imax_int) != 0 ||
            json_get_i64(buf, "vmax_int", &rec.vmax_int) != 0 ||
            json_get_i64(buf, "two_p", &rec.two_p) != 0 ||
            json_get_i64(buf, "torque_int", &rec.torque_int) != 0) {
            free(buf);
            return -1;
        }
        ele_i128 c2 = (ele_i128)rec.id_int * rec.id_int + (ele_i128)rec.iq_int * rec.iq_int;
        ele_i128 cm = (ele_i128)rec.imax_int * rec.imax_int;
        ele_i128 v2 = (ele_i128)rec.vd_int * rec.vd_int + (ele_i128)rec.vq_int * rec.vq_int;
        ele_i128 vm = (ele_i128)rec.vmax_int * rec.vmax_int;
        ele_i128 tq = (ele_i128)rec.two_p *
                      ((ele_i128)rec.psid_int * rec.iq_int - (ele_i128)rec.psiq_int * rec.id_int);
        if (c2 > cm || v2 > vm) { free(buf); return -11; }
        if (rec.has_bracket && (tq < rec.lb_int || tq > rec.ub_int)) { free(buf); return -12; }
    }

    if (rec.has_opf) {
        if (json_get_i64(buf, "opf_scale", &rec.opf_scale) != 0 ||
            json_get_i64(buf, "c_dual_int", &rec.c_dual_int) != 0 ||
            json_get_i64(buf, "w_max_int", &rec.w_max_int) != 0 ||
            json_get_i64(buf, "opf_delta_round", &rec.opf_delta_round) != 0 ||
            json_get_i64(buf, "c_lb_int", &rec.c_lb_int) != 0 ||
            json_get_i64(buf, "c_ub_int", &rec.c_ub_int) != 0) {
            free(buf);
            return -1;
        }
        if (rec.has_psd) {
            ele_i128 neg_mu = (rec.psd.mu_psd < 0) ? (ele_i128)rec.psd.mu_psd : 0;
            ele_i128 expected_lb = (ele_i128)rec.c_dual_int +
                                   neg_mu * (ele_i128)rec.w_max_int -
                                   (ele_i128)rec.opf_delta_round;
            if ((ele_i128)rec.c_lb_int != expected_lb || rec.c_lb_int > rec.c_ub_int) {
                free(buf);
                return -13;
            }
        }
    }

    if (rec.has_emt) {
        if (json_get_i64(buf, "v_kin_int", &rec.v_kin_int) != 0 ||
            json_get_i64(buf, "v_pot_int", &rec.v_pot_int) != 0 ||
            json_get_i64(buf, "v_crit_int", &rec.v_crit_int) != 0 ||
            json_get_i64(buf, "v_err_int", &rec.v_err_int) != 0) {
            free(buf);
            return -1;
        }
        if (rec.v_kin_int < 0 || rec.v_err_int < 0) { free(buf); return -14; }
        if (strcmp(rec.status, "CERTIFIED") == 0 &&
            (ele_i128)rec.v_kin_int + rec.v_pot_int + rec.v_err_int >= rec.v_crit_int) {
            free(buf);
            return -15;
        }
    }

    if (has_chordal) {
        int cr = verify_chordal_cert(buf, &rec);
        if (cr != 0) {
            free(buf);
            return cr;
        }
    }

    free(buf);
    return 0;
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

int ele_cert_emit_chordal_lean(const char *cert_json_path,
                               const char *lean_out_path,
                               const char *mod_name) {
    if (cert_json_path == NULL || lean_out_path == NULL || mod_name == NULL) return -1;
    FILE *rfp = fopen(cert_json_path, "rb");
    if (rfp == NULL) return -1;
    if (fseek(rfp, 0L, SEEK_END) != 0) { fclose(rfp); return -1; }
    long fsz = ftell(rfp);
    if (fsz <= 0 || fsz > 16L * 1024L * 1024L) { fclose(rfp); return -1; }
    rewind(rfp);
    char *buf = (char *)malloc((size_t)fsz + 1U);
    if (buf == NULL) { fclose(rfp); return -1; }
    size_t nr = fread(buf, 1U, (size_t)fsz, rfp);
    fclose(rfp);
    buf[nr] = '\0';

    char pillar[32] = {0}, kind[64] = {0}, status[32] = {0}, sha[65] = {0};
    int64_t has_ch = 0, has_opf = 0, dim64 = 0, ncl64 = 0;
    int64_t c_dual = 0, d_rnd = 0, c_lb = 0, c_ub = 0, w_max_def = 0;
    if (json_get_str(buf, "pillar", pillar, sizeof(pillar)) != 0 ||
        json_get_str(buf, "kind", kind, sizeof(kind)) != 0 ||
        json_get_str(buf, "status", status, sizeof(status)) != 0 ||
        json_get_str(buf, "input_sha256", sha, sizeof(sha)) != 0 ||
        json_get_i64(buf, "has_chordal", &has_ch) != 0 || has_ch != 1 ||
        json_get_i64(buf, "has_opf", &has_opf) != 0 || has_opf != 1 ||
        strcmp(status, "CERTIFIED") != 0 ||
        json_get_i64(buf, "dim", &dim64) != 0 || dim64 < 2 || dim64 > 64 ||
        json_get_i64(buf, "num_cliques", &ncl64) != 0 || ncl64 < 1 || ncl64 > 64 ||
        json_get_i64(buf, "c_dual_int", &c_dual) != 0 ||
        json_get_i64(buf, "w_max_int", &w_max_def) != 0 ||
        json_get_i64(buf, "opf_delta_round", &d_rnd) != 0 ||
        json_get_i64(buf, "c_lb_int", &c_lb) != 0 ||
        json_get_i64(buf, "c_ub_int", &c_ub) != 0) {
        free(buf);
        return -1;
    }

    int dim = (int)dim64;
    int n_cl = (int)ncl64;
    int64_t *A_full = (int64_t *)calloc((size_t)dim * (size_t)dim, sizeof(int64_t));
    if (A_full == NULL) { free(buf); return -1; }

    const char *as = find_json_key(buf, "A_sparse");
    if (as == NULL || *as != '[') { free(A_full); free(buf); return -1; }
    ++as;
    while (*as != '\0') {
        while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
        if (*as == ']') break;
        if (*as != '[') { free(A_full); free(buf); return -1; }
        ++as;
        int64_t tr[3] = {0, 0, 0};
        for (int t = 0; t < 3; ++t) {
            while (*as != '\0' && (isspace((unsigned char)*as) || *as == ',')) ++as;
            char *endp = NULL;
            long long val = strtoll(as, &endp, 10);
            if (endp == as) { free(A_full); free(buf); return -1; }
            tr[t] = (int64_t)val;
            as = endp;
        }
        while (*as != '\0' && isspace((unsigned char)*as)) ++as;
        if (*as != ']') { free(A_full); free(buf); return -1; }
        ++as;
        if (tr[0] >= 0 && tr[0] < dim && tr[1] >= 0 && tr[1] < dim) {
            A_full[(size_t)tr[0] * (size_t)dim + (size_t)tr[1]] = tr[2];
            A_full[(size_t)tr[1] * (size_t)dim + (size_t)tr[0]] = tr[2];
        }
    }

    FILE *fp = fopen(lean_out_path, "w");
    if (fp == NULL) { free(A_full); free(buf); return -1; }
    fprintf(fp, "/-\n  Standalone Zero-FP-Trust Chordal Certificate: %s\n  Pillar: %s | Kind: %s | SHA256: %s\n-/\n",
            mod_name, pillar, kind, sha);
    fprintf(fp, "import EleCert\nimport Flow.Interval\n\nnamespace %s\n\nopen Flow.Interval\n\n", mod_name);
    fprintf(fp, "def dimN : Nat := %d\n", dim);
    fprintf(fp, "def cDual : Int := %" PRId64 "\n", c_dual);
    fprintf(fp, "def dRnd  : Int := %" PRId64 "\n", d_rnd);
    fprintf(fp, "def cLB   : Int := %" PRId64 "\n", c_lb);
    fprintf(fp, "def cUB   : Int := %" PRId64 "\n\n", c_ub);
    emit_lean_i64_matrix(fp, "matA", dim, A_full);
    free(A_full);

    const char *cl = find_json_key(buf, "cliques");
    if (cl == NULL || *cl != '[') { fclose(fp); free(buf); return -1; }
    char *cur = (char *)(uintptr_t)(cl + 1);
    fprintf(fp, "def cliques : List EleCert.CliqueCert := [\n");
    for (int c = 0; c < n_cl; ++c) {
        while (*cur != '\0' && *cur != '{') ++cur;
        if (*cur != '{') { fclose(fp); free(buf); return -1; }
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
        if (*obj_end != '}') { fclose(fp); free(buf); return -1; }
        *obj_end = '\0';

        int dk = 0;
        int64_t mu_c = 0, w_c = w_max_def;
        int vars[32] = {0};
        int64_t S_int[32 * 32] = {0}, L_int[32 * 32] = {0};
        (void)json_get_i64(obj_start, "w_max_int", &w_c);
        if (json_get_i64_list_bounded(obj_start, "vars", 32, &dk, vars) != 0 ||
            dk < 2 || dk > 32 ||
            json_get_i64(obj_start, "mu_psd", &mu_c) != 0 ||
            json_get_i64_mat(obj_start, "S_int", dk, S_int) != 0 ||
            json_get_i64_mat(obj_start, "L_int", dk, L_int) != 0) {
            *obj_end = '}';
            fclose(fp); free(buf); return -1;
        }
        *obj_end = '}';
        cur = obj_end + 1;

        fprintf(fp, "  { vars := [");
        for (int a = 0; a < dk; ++a) fprintf(fp, "%s%d", a ? ", " : "", vars[a]);
        fprintf(fp, "],\n    sMat := [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(fp, "      [");
            for (int b = 0; b < dk; ++b) fprintf(fp, "%s%" PRId64, b ? ", " : "", S_int[a * dk + b]);
            fprintf(fp, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(fp, "    ],\n    lMat := [\n");
        for (int a = 0; a < dk; ++a) {
            fprintf(fp, "      [");
            for (int b = 0; b < dk; ++b) fprintf(fp, "%s%" PRId64, b ? ", " : "", L_int[a * dk + b]);
            fprintf(fp, "]%s\n", (a + 1 < dk) ? "," : "");
        }
        fprintf(fp, "    ],\n    muPsd := %" PRId64 ",\n    wMax := %" PRId64 " }%s\n",
                mu_c, w_c, (c + 1 < n_cl) ? "," : "");
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
    free(buf);
    return (ferr == 0) ? 0 : -1;
}
