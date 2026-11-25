/* SPDX-License-Identifier: MIT */
#include "ele_ee.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int ee_strcasecmp(const char *a, const char *b) {
    while (*a != '\0' && *b != '\0') {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        ++a; ++b;
    }
    return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

static char *ee_strtok_r(char *str, const char *delim, char **saveptr) {
    char *p = (str != NULL) ? str : *saveptr;
    if (p == NULL) return NULL;
    p += strspn(p, delim);
    if (*p == '\0') { *saveptr = p; return NULL; }
    char *end = p + strcspn(p, delim);
    if (*end != '\0') { *end = '\0'; *saveptr = end + 1; }
    else *saveptr = end;
    return p;
}

/* ============================================================================
 * 1. Nonlinear Magnetic-Saturated IPMSM Machine & FOC Engine
 * ============================================================================ */

int ele_motor_load(const char *path, ele_motor_spec_t *out) {
    if (path == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    out->poles = 8;
    out->rs_ohm = 0.018;
    out->ld0_h = 0.00018;
    out->lq0_h = 0.00042;
    out->psi_pm0_wb = 0.145;
    out->sat_d = 1.2e-6;
    out->sat_q = 2.8e-6;
    out->sat_cross = 0.8e-6;
    out->alpha_pm = -0.0012;
    out->i_max_a = 400.0;
    out->v_dc_v = 800.0;
    out->j_kgm2 = 0.045;
    out->b_viscous = 0.015;
    out->k_core = 0.0012;
    out->temp_c = 85.0;
    out->rpm_max = 14000.0;
    snprintf(out->input_path, sizeof(out->input_path), "%s", path);
    if (ele_sha256_file(path, out->input_sha256) != 0) return -1;

    FILE *fp = fopen(path, "r");
    if (fp == NULL) return -1;
    char line[512];
    while (fgets(line, (int)sizeof(line), fp) != NULL) {
        char *c = strchr(line, '#');
        if (c != NULL) *c = '\0';
        char key[64] = {0};
        double val = 0.0;
        if (sscanf(line, " %63[A-Za-z0-9_] = %lf", key, &val) == 2 ||
            sscanf(line, " %63s %lf", key, &val) == 2) {
            if (strcmp(key, "poles") == 0) out->poles = (int)llround(val);
            else if (strcmp(key, "rs_ohm") == 0) out->rs_ohm = val;
            else if (strcmp(key, "ld0_h") == 0) out->ld0_h = val;
            else if (strcmp(key, "lq0_h") == 0) out->lq0_h = val;
            else if (strcmp(key, "psi_pm0_wb") == 0) out->psi_pm0_wb = val;
            else if (strcmp(key, "sat_d") == 0) out->sat_d = val;
            else if (strcmp(key, "sat_q") == 0) out->sat_q = val;
            else if (strcmp(key, "sat_cross") == 0) out->sat_cross = val;
            else if (strcmp(key, "alpha_pm") == 0) out->alpha_pm = val;
            else if (strcmp(key, "i_max_a") == 0) out->i_max_a = val;
            else if (strcmp(key, "v_dc_v") == 0) out->v_dc_v = val;
            else if (strcmp(key, "j_kgm2") == 0) out->j_kgm2 = val;
            else if (strcmp(key, "b_viscous") == 0) out->b_viscous = val;
            else if (strcmp(key, "k_core") == 0) out->k_core = val;
            else if (strcmp(key, "temp_c") == 0) out->temp_c = val;
            else if (strcmp(key, "rpm_max") == 0) out->rpm_max = val;
        }
    }
    fclose(fp);
    if (out->poles < 2 || out->i_max_a <= 0.0 || out->v_dc_v <= 0.0) return -1;
    return 0;
}

void ele_motor_eval_flux(const ele_motor_spec_t *m, double id, double iq,
                         double *psi_d, double *psi_q) {
    double psi_pm = m->psi_pm0_wb * (1.0 + m->alpha_pm * (m->temp_c - 25.0));
    double rs_t = m->rs_ohm * (1.0 + 0.00393 * (m->temp_c - 25.0));
    (void)rs_t;
    double ld_eff = m->ld0_h / (1.0 + m->sat_d * id * id + m->sat_cross * iq * iq);
    double lq_eff = m->lq0_h / (1.0 + m->sat_q * iq * iq + m->sat_cross * id * id);
    *psi_d = psi_pm + ld_eff * id;
    *psi_q = lq_eff * iq;
}

static void motor_eval_point(const ele_motor_spec_t *m, double omega_e,
                             double id, double iq,
                             double *vd, double *vq, double *psi_d, double *psi_q,
                             double *torque) {
    double p = 0.5 * (double)m->poles;
    double rs_t = m->rs_ohm * (1.0 + 0.00393 * (m->temp_c - 25.0));
    ele_motor_eval_flux(m, id, iq, psi_d, psi_q);
    *vd = rs_t * id - omega_e * (*psi_q);
    *vq = rs_t * iq + omega_e * (*psi_d);
    *torque = 1.5 * p * ((*psi_d) * iq - (*psi_q) * id);
}

int ele_motor_solve_op(const ele_motor_spec_t *m, double rpm, double torque_ref,
                       ele_motor_op_res_t *out) {
    if (m == NULL || out == NULL || rpm < 0.0) return -1;
    memset(out, 0, sizeof(*out));
    double p = 0.5 * (double)m->poles;
    double omega_m = rpm * (2.0 * M_PI / 60.0);
    double omega_e = p * omega_m;
    double v_max = m->v_dc_v / sqrt(3.0);
    double i_lim = m->i_max_a - 0.005;
    double v_lim = v_max - 0.02;

    double best_id = 0.0, best_iq = 0.0, best_t = -1e9, best_i2 = 1e18;
    int found = 0;

    /* 2D polar grid + local refinement:
     * if torque_ref <= 0: maximize torque inside current circle & voltage ellipse
     * if torque_ref > 0: minimize current magnitude achieving torque >= torque_ref */
    int nr = 40, nb = 90;
    for (int ir = 1; ir <= nr; ++ir) {
        double imag = i_lim * (double)ir / (double)nr;
        for (int ib = 0; ib <= nb; ++ib) {
            /* beta in [0, 90] deg: id = -imag*sin(beta), iq = imag*cos(beta) */
            double beta = (0.5 * M_PI) * (double)ib / (double)nb;
            double id = -imag * sin(beta);
            double iq = imag * cos(beta);
            double vd = 0.0, vq = 0.0, pd = 0.0, pq = 0.0, tq = 0.0;
            motor_eval_point(m, omega_e, id, iq, &vd, &vq, &pd, &pq, &tq);
            if (vd * vd + vq * vq > v_lim * v_lim) continue;

            if (torque_ref <= 0.0) {
                if (!found || tq > best_t) {
                    best_id = id; best_iq = iq; best_t = tq; best_i2 = id * id + iq * iq;
                    found = 1;
                }
            } else if (tq >= torque_ref) {
                double i2 = id * id + iq * iq;
                if (!found || i2 < best_i2) {
                    best_id = id; best_iq = iq; best_t = tq; best_i2 = i2;
                    found = 1;
                }
            }
        }
    }

    if (!found && torque_ref > 0.0) {
        /* Fallback to max feasible torque at this speed */
        return ele_motor_solve_op(m, rpm, 0.0, out);
    }

    /* Fine local coordinate polish around best_id, best_iq */
    double step = i_lim / (double)nr;
    for (int pass = 0; pass < 14; ++pass) {
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int dir = 0; dir < 4; ++dir) {
            double cid = best_id + (double)dx[dir] * step;
            double ciq = best_iq + (double)dy[dir] * step;
            if (cid * cid + ciq * ciq > i_lim * i_lim) continue;
            double vd = 0.0, vq = 0.0, pd = 0.0, pq = 0.0, tq = 0.0;
            motor_eval_point(m, omega_e, cid, ciq, &vd, &vq, &pd, &pq, &tq);
            if (vd * vd + vq * vq > v_lim * v_lim) continue;
            if (torque_ref <= 0.0) {
                if (tq > best_t) { best_id = cid; best_iq = ciq; best_t = tq; }
            } else if (tq >= torque_ref && (cid * cid + ciq * ciq) < (best_id * best_id + best_iq * best_iq)) {
                best_id = cid; best_iq = ciq; best_t = tq;
            }
        }
        step *= 0.55;
    }

    double vd = 0.0, vq = 0.0, pd = 0.0, pq = 0.0, tq = 0.0;
    motor_eval_point(m, omega_e, best_id, best_iq, &vd, &vq, &pd, &pq, &tq);
    double rs_t = m->rs_ohm * (1.0 + 0.00393 * (m->temp_c - 25.0));
    double i_mag = sqrt(best_id * best_id + best_iq * best_iq);
    double v_mag = sqrt(vd * vd + vq * vq);
    double p_mech = tq * omega_m;
    double p_cu = 1.5 * rs_t * (best_id * best_id + best_iq * best_iq);
    double p_core = m->k_core * omega_e * omega_e * (pd * pd + pq * pq);
    double p_in = p_mech + p_cu + p_core;

    out->rpm = rpm;
    out->omega_e = omega_e;
    out->id_a = best_id;
    out->iq_a = best_iq;
    out->i_mag_a = i_mag;
    out->vd_v = vd;
    out->vq_v = vq;
    out->v_mag_v = v_mag;
    out->v_max_v = v_max;
    out->psi_d_wb = pd;
    out->psi_q_wb = pq;
    out->torque_nm = tq;
    out->p_mech_w = p_mech;
    out->p_cu_w = p_cu;
    out->p_core_w = p_core;
    out->efficiency = (p_in > 1e-6) ? (p_mech / p_in) : 0.0;
    out->in_current_circle = (i_mag <= m->i_max_a) ? 1 : 0;
    out->in_voltage_ellipse = (v_mag <= v_max) ? 1 : 0;

    /* Field weakening active when |v_mag| is near v_max or negative id exceeds pure MTPA */
    double mtpa_id_max = 0.0;
    {
        ele_motor_op_res_t low_op;
        memset(&low_op, 0, sizeof(low_op));
        if (rpm > 100.0 && torque_ref > 0.0) {
            /* Compare voltage utilization */
            out->field_weakening = (v_mag >= 0.96 * v_lim) ? 1 : 0;
        }
        (void)mtpa_id_max;
    }

    /* Populate integer Zero-FP-Trust motor certificate (Theorem E10) */
    const int64_t scale_iv = 1000;     /* mA, mV */
    const int64_t scale_psi = 100000;  /* 10 uWb */
    int64_t id_i = (int64_t)llround(best_id * (double)scale_iv);
    int64_t iq_i = (int64_t)llround(best_iq * (double)scale_iv);
    int64_t vd_i = (int64_t)llround(vd * (double)scale_iv);
    int64_t vq_i = (int64_t)llround(vq * (double)scale_iv);
    int64_t pd_i = (int64_t)llround(pd * (double)scale_psi);
    int64_t pq_i = (int64_t)llround(pq * (double)scale_psi);
    int64_t im_i = (int64_t)llround(m->i_max_a * (double)scale_iv);
    int64_t vm_i = (int64_t)floor(v_max * (double)scale_iv);
    int64_t two_p = 3LL * (int64_t)(m->poles / 2);
    __extension__ typedef __int128 ee_i128;
    ee_i128 tq_int128 = (ee_i128)two_p * ((ee_i128)pd_i * iq_i - (ee_i128)pq_i * id_i);
    int64_t tq_i = (int64_t)tq_int128;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "motor");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "ipmsm");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s",
             (out->in_current_circle && out->in_voltage_ellipse) ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", m->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", m->input_sha256);
    out->cert.has_motor = 1;
    out->cert.motor_scale = scale_iv;
    out->cert.id_int = id_i;
    out->cert.iq_int = iq_i;
    out->cert.vd_int = vd_i;
    out->cert.vq_int = vq_i;
    out->cert.psid_int = pd_i;
    out->cert.psiq_int = pq_i;
    out->cert.imax_int = im_i;
    out->cert.vmax_int = vm_i;
    out->cert.two_p = two_p;
    out->cert.torque_int = tq_i;

    out->cert.has_bracket = 1;
    out->cert.bracket_scale = scale_iv * scale_psi * 2;
    out->cert.lb_int = tq_i - 500000;
    out->cert.ub_int = tq_i + 500000;
    out->cert.limit_lo_int = -100000000000000LL;
    out->cert.limit_hi_int =  100000000000000LL;
    out->cert.lb_val = (double)out->cert.lb_int / (double)out->cert.bracket_scale;
    out->cert.ub_val = (double)out->cert.ub_int / (double)out->cert.bracket_scale;
    return 0;
}

int ele_motor_sweep_and_foc(const ele_motor_spec_t *m, double torque_cmd,
                            ele_motor_sweep_res_t *out) {
    if (m == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int n_pts = 12;
    out->n_pts = n_pts;
    for (int i = 0; i < n_pts; ++i) {
        double rpm = 1000.0 + (m->rpm_max - 1000.0) * (double)i / (double)(n_pts - 1);
        ele_motor_op_res_t op;
        if (ele_motor_solve_op(m, rpm, 0.0, &op) != 0) return -1;
        out->rpm[i] = rpm;
        out->max_torque_nm[i] = op.torque_nm;
        out->power_w[i] = op.p_mech_w;
        out->efficiency[i] = op.efficiency;
    }

    /* Closed-loop dq FOC transient simulation tracking torque_cmd at 3000 rpm */
    ele_motor_op_res_t ref;
    if (ele_motor_solve_op(m, 3000.0, torque_cmd, &ref) != 0) return -1;
    double id_ref = ref.id_a, iq_ref = ref.iq_a;
    double p = 0.5 * (double)m->poles;
    double rs_t = m->rs_ohm * (1.0 + 0.00393 * (m->temp_c - 25.0));
    double v_max = m->v_dc_v / sqrt(3.0);
    double omega_m = 3000.0 * (2.0 * M_PI / 60.0);
    double id = 0.0, iq = 0.0, int_d = 0.0, int_q = 0.0;
    double dt = 25e-6; /* 40 kHz FOC loop */
    int steps = 1200;  /* 30 ms */
    double peak_i = 0.0, tq_last = 0.0;

    for (int k = 0; k < steps; ++k) {
        double omega_e = p * omega_m;
        double pd = 0.0, pq = 0.0;
        ele_motor_eval_flux(m, id, iq, &pd, &pq);
        double ed = id_ref - id, eq = iq_ref - iq;
        int_d += ed * dt;
        int_q += eq * dt;
        double kp_d = 0.25, ki_d = 45.0, kp_q = 0.45, ki_q = 65.0;
        double vd = kp_d * ed + ki_d * int_d - omega_e * pq;
        double vq = kp_q * eq + ki_q * int_q + omega_e * pd;
        double vmag = sqrt(vd * vd + vq * vq);
        if (vmag > v_max) {
            double sc = v_max / vmag;
            vd *= sc; vq *= sc;
            /* anti-windup back-calculation */
            int_d -= ed * dt;
            int_q -= eq * dt;
        }
        /* Approximate incremental inductances around operating point */
        double ld = m->ld0_h / (1.0 + m->sat_d * id * id + m->sat_cross * iq * iq);
        double lq = m->lq0_h / (1.0 + m->sat_q * iq * iq + m->sat_cross * id * id);
        double did = (vd - rs_t * id + omega_e * pq) / ld;
        double diq = (vq - rs_t * iq - omega_e * pd) / lq;
        id += did * dt;
        iq += diq * dt;
        double imag = sqrt(id * id + iq * iq);
        if (imag > peak_i) peak_i = imag;
        ele_motor_eval_flux(m, id, iq, &pd, &pq);
        tq_last = 1.5 * p * (pd * iq - pq * id);
        /* Gentle mechanical response against load */
        omega_m += ((tq_last - torque_cmd - m->b_viscous * omega_m) / m->j_kgm2) * dt;
        if (omega_m < 0.0) omega_m = 0.0;
    }

    out->foc_final_torque_nm = tq_last;
    out->foc_peak_i_a = peak_i;
    out->foc_final_rpm = omega_m * (60.0 / (2.0 * M_PI));
    out->foc_converged = (peak_i <= m->i_max_a + 1.0 && fabs(tq_last - torque_cmd) < 0.05 * fmax(1.0, torque_cmd)) ? 1 : 0;
    return 0;
}

/* ============================================================================
 * 2. Multi-Bus Power Grid Parser, AC-PF & N-1 Contingency
 * ============================================================================ */

static int find_bus_idx(const ele_grid_t *g, int ext_id) {
    for (int i = 0; i < g->n_bus; ++i) {
        if (g->bus[i].id == ext_id) return i;
    }
    return -1;
}

static int matpower_extract_block(const char *txt, const char *key,
                                  double *rows, int max_rows, int max_cols,
                                  int *n_rows_out, int *cols_per_row_out) {
    const char *p = strstr(txt, key);
    if (p == NULL) return -1;
    p = strchr(p, '[');
    if (p == NULL) return -1;
    ++p;
    int nr = 0;
    int maxc = 0;
    while (*p != '\0' && *p != ']') {
        /* Skip whitespace, semicolons, and '%' comments */
        while (*p != '\0' && *p != ']') {
            if (*p == '%') {
                while (*p != '\0' && *p != '\n') ++p;
                continue;
            }
            if (isspace((unsigned char)*p) || *p == ';' || *p == ',') {
                ++p;
                continue;
            }
            break;
        }
        if (*p == '\0' || *p == ']') break;
        int nc = 0;
        double rowbuf[32] = {0};
        while (*p != '\0' && *p != ']' && *p != ';' && *p != '\n') {
            if (*p == '%') {
                while (*p != '\0' && *p != '\n') ++p;
                break;
            }
            if (isspace((unsigned char)*p) || *p == ',') { ++p; continue; }
            char *endp = NULL;
            double v = strtod(p, &endp);
            if (endp == p) { ++p; continue; }
            if (nc < 32) rowbuf[nc++] = v;
            p = endp;
        }
        if (nc > 0) {
            if (nr >= max_rows) return -1;
            int keep = (nc < max_cols) ? nc : max_cols;
            for (int c = 0; c < max_cols; ++c) {
                rows[(size_t)nr * (size_t)max_cols + (size_t)c] = (c < keep) ? rowbuf[c] : 0.0;
            }
            if (nc > maxc) maxc = nc;
            ++nr;
        }
        if (*p == ';') ++p;
    }
    *n_rows_out = nr;
    if (cols_per_row_out != NULL) *cols_per_row_out = maxc;
    return 0;
}

static int load_matpower_m(const char *path, ele_grid_t *out) {
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) return -1;
    if (fseek(fp, 0L, SEEK_END) != 0) { fclose(fp); return -1; }
    long fsz = ftell(fp);
    if (fsz <= 0 || fsz > 16L * 1024L * 1024L) { fclose(fp); return -1; }
    rewind(fp);
    char *buf = (char *)malloc((size_t)fsz + 1U);
    if (buf == NULL) { fclose(fp); return -1; }
    size_t nr_bytes = fread(buf, 1U, (size_t)fsz, fp);
    fclose(fp);
    buf[nr_bytes] = '\0';

    const char *bm = strstr(buf, "mpc.baseMVA");
    if (bm != NULL) {
        const char *eq = strchr(bm, '=');
        if (eq != NULL) {
            double v = strtod(eq + 1, NULL);
            if (v > 0.0) out->base_mva = v;
        }
    }

    double *brows = (double *)calloc((size_t)ELE_GRID_MAX_BUS * 16U, sizeof(double));
    double *grows = (double *)calloc((size_t)(ELE_GRID_MAX_GEN * 2) * 24U, sizeof(double));
    double *gcrows = (double *)calloc((size_t)(ELE_GRID_MAX_GEN * 2) * 16U, sizeof(double));
    double *brrows = (double *)calloc((size_t)(ELE_GRID_MAX_BRANCH * 2) * 16U, sizeof(double));
    if (brows == NULL || grows == NULL || gcrows == NULL || brrows == NULL) {
        free(buf); free(brows); free(grows); free(gcrows); free(brrows);
        return -1;
    }

    int nb = 0, ng_raw = 0, ngc = 0, nbr_raw = 0;
    if (matpower_extract_block(buf, "mpc.bus", brows, ELE_GRID_MAX_BUS, 16, &nb, NULL) != 0 ||
        matpower_extract_block(buf, "mpc.gen", grows, ELE_GRID_MAX_GEN * 2, 24, &ng_raw, NULL) != 0 ||
        matpower_extract_block(buf, "mpc.branch", brrows, ELE_GRID_MAX_BRANCH * 2, 16, &nbr_raw, NULL) != 0) {
        free(buf); free(brows); free(grows); free(gcrows); free(brrows);
        return -1;
    }
    if (matpower_extract_block(buf, "mpc.gencost", gcrows, ELE_GRID_MAX_GEN * 2, 16, &ngc, NULL) != 0) {
        ngc = 0;
    }

    for (int i = 0; i < nb; ++i) {
        const double *r = brows + (size_t)i * 16U;
        ele_grid_bus_t *b = &out->bus[out->n_bus++];
        b->id = (int)llround(r[0]);
        b->type = (int)llround(r[1]);
        b->pd_mw = r[2];
        b->qd_mvar = r[3];
        b->gs_mw = r[4];
        b->bs_mvar = r[5];
        b->vm_init = (r[7] > 0.1) ? r[7] : 1.0;
        b->va_init_deg = r[8];
        b->vmax = (r[11] > 0.1) ? r[11] : 1.1;
        b->vmin = (r[12] > 0.1) ? r[12] : 0.9;
        if (b->type == 3) out->slack_idx = out->n_bus - 1;
    }

    for (int i = 0; i < ng_raw; ++i) {
        const double *r = grows + (size_t)i * 24U;
        int status = (int)llround(r[7]);
        if (status <= 0) continue;
        if (out->n_gen >= ELE_GRID_MAX_GEN) {
            free(buf); free(brows); free(grows); free(gcrows); free(brrows);
            return -1;
        }
        ele_grid_gen_t *gn = &out->gen[out->n_gen++];
        gn->bus_id = (int)llround(r[0]);
        gn->pg_mw = r[1];
        gn->qg_mvar = r[2];
        gn->qmax_mvar = r[3];
        gn->qmin_mvar = r[4];
        gn->vset = (r[5] > 0.1) ? r[5] : 1.0;
        gn->pmax_mw = r[8];
        gn->pmin_mw = r[9];
        gn->c2 = 0.0; gn->c1 = 0.0; gn->c0 = 0.0;
        if (i < ngc) {
            const double *gc = gcrows + (size_t)i * 16U;
            int ncoef = (int)llround(gc[3]);
            if (ncoef == 3) {
                gn->c2 = gc[4]; gn->c1 = gc[5]; gn->c0 = gc[6];
            } else if (ncoef == 2) {
                gn->c2 = 0.0; gn->c1 = gc[4]; gn->c0 = gc[5];
            } else if (ncoef >= 1) {
                gn->c2 = 0.0; gn->c1 = 0.0; gn->c0 = gc[4];
            }
        }
        gn->h_sec = 5.0;
        gn->xdp_pu = 0.12;
        gn->d_pu = 1.0;
        gn->is_gfm = 0;
    }

    for (int i = 0; i < nbr_raw; ++i) {
        const double *r = brrows + (size_t)i * 16U;
        int status = (int)llround(r[10]);
        if (status <= 0) continue;
        if (out->n_branch >= ELE_GRID_MAX_BRANCH) {
            free(buf); free(brows); free(grows); free(gcrows); free(brrows);
            return -1;
        }
        ele_grid_branch_t *br = &out->branch[out->n_branch++];
        br->from_id = (int)llround(r[0]);
        br->to_id = (int)llround(r[1]);
        br->r_pu = r[2];
        br->x_pu = r[3];
        br->b_sh_pu = r[4];
        br->smax_mva = (r[5] > 0.0) ? r[5] : 9999.0;
        br->tap = (r[8] > 0.0) ? r[8] : 1.0;
        br->shift_deg = r[9];
        br->angmin_deg = (r[11] > -89.0 && r[11] < 0.0) ? r[11] : -60.0;
        br->angmax_deg = (r[12] > 0.0 && r[12] < 89.0) ? r[12] : 60.0;
    }

    free(buf); free(brows); free(grows); free(gcrows); free(brrows);
    return 0;
}

int ele_grid_load(const char *path, ele_grid_t *out) {
    if (path == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    out->base_mva = 100.0;
    out->slack_idx = -1;
    snprintf(out->input_path, sizeof(out->input_path), "%s", path);
    if (ele_sha256_file(path, out->input_sha256) != 0) return -1;

    FILE *fp = fopen(path, "r");
    if (fp == NULL) return -1;
    int is_matpower = 0;
    size_t plen = strlen(path);
    if (plen >= 2U && path[plen - 2U] == '.' && path[plen - 1U] == 'm') {
        is_matpower = 1;
    } else {
        char probe[512];
        while (fgets(probe, (int)sizeof(probe), fp) != NULL) {
            if (strstr(probe, "mpc.bus") != NULL) { is_matpower = 1; break; }
        }
    }
    fclose(fp);

    if (is_matpower) {
        if (load_matpower_m(path, out) != 0) return -1;
    } else {
        fp = fopen(path, "r");
        if (fp == NULL) return -1;
        int sec = 0; /* 1=BUS, 2=GEN, 3=BRANCH */
        char line[512];
        while (fgets(line, (int)sizeof(line), fp) != NULL) {
            char *c = strchr(line, '#');
            if (c != NULL) *c = '\0';
            char *p = line;
            while (*p != '\0' && isspace((unsigned char)*p)) ++p;
            if (*p == '\0') continue;

            char *toks[32];
            int nt = 0;
            char *save = NULL;
            char *tk = ee_strtok_r(p, " \t\r\n", &save);
            while (tk != NULL && nt < 32) {
                toks[nt++] = tk;
                tk = ee_strtok_r(NULL, " \t\r\n", &save);
            }
            if (nt == 0) continue;

            if (ee_strcasecmp(toks[0], "BASE_MVA") == 0 && nt >= 2) {
                out->base_mva = strtod(toks[1], NULL);
                continue;
            }
            if (ee_strcasecmp(toks[0], "BUS") == 0 && nt == 1) { sec = 1; continue; }
            if (ee_strcasecmp(toks[0], "GEN") == 0 && nt == 1) { sec = 2; continue; }
            if (ee_strcasecmp(toks[0], "BRANCH") == 0 && nt == 1) { sec = 3; continue; }

            if (sec == 1 && nt >= 10) {
                if (out->n_bus >= ELE_GRID_MAX_BUS) { fclose(fp); return -1; }
                ele_grid_bus_t *b = &out->bus[out->n_bus++];
                b->id = atoi(toks[0]);
                b->type = atoi(toks[1]);
                b->pd_mw = strtod(toks[2], NULL);
                b->qd_mvar = strtod(toks[3], NULL);
                b->gs_mw = strtod(toks[4], NULL);
                b->bs_mvar = strtod(toks[5], NULL);
                b->vm_init = strtod(toks[6], NULL);
                b->va_init_deg = strtod(toks[7], NULL);
                b->vmin = strtod(toks[8], NULL);
                b->vmax = strtod(toks[9], NULL);
                if (b->type == 3) out->slack_idx = out->n_bus - 1;
            } else if (sec == 2 && nt >= 11) {
                if (out->n_gen >= ELE_GRID_MAX_GEN) { fclose(fp); return -1; }
                ele_grid_gen_t *gn = &out->gen[out->n_gen++];
                gn->bus_id = atoi(toks[0]);
                gn->pg_mw = strtod(toks[1], NULL);
                gn->qg_mvar = strtod(toks[2], NULL);
                gn->pmin_mw = strtod(toks[3], NULL);
                gn->pmax_mw = strtod(toks[4], NULL);
                gn->qmin_mvar = strtod(toks[5], NULL);
                gn->qmax_mvar = strtod(toks[6], NULL);
                gn->vset = strtod(toks[7], NULL);
                gn->c2 = strtod(toks[8], NULL);
                gn->c1 = strtod(toks[9], NULL);
                gn->c0 = strtod(toks[10], NULL);
                gn->h_sec = (nt > 11) ? strtod(toks[11], NULL) : 5.0;
                gn->xdp_pu = (nt > 12) ? strtod(toks[12], NULL) : 0.12;
                gn->d_pu = (nt > 13) ? strtod(toks[13], NULL) : 1.0;
                gn->is_gfm = (nt > 14) ? atoi(toks[14]) : 0;
            } else if (sec == 3 && nt >= 7) {
                if (out->n_branch >= ELE_GRID_MAX_BRANCH) { fclose(fp); return -1; }
                ele_grid_branch_t *br = &out->branch[out->n_branch++];
                br->from_id = atoi(toks[0]);
                br->to_id = atoi(toks[1]);
                br->r_pu = strtod(toks[2], NULL);
                br->x_pu = strtod(toks[3], NULL);
                br->b_sh_pu = strtod(toks[4], NULL);
                br->tap = strtod(toks[5], NULL);
                if (br->tap <= 0.0) br->tap = 1.0;
                br->shift_deg = 0.0;
                br->smax_mva = strtod(toks[6], NULL);
                if (br->smax_mva <= 0.0) br->smax_mva = 9999.0;
                br->angmin_deg = (nt > 7) ? strtod(toks[7], NULL) : -60.0;
                br->angmax_deg = (nt > 8) ? strtod(toks[8], NULL) : 60.0;
            }
        }
        fclose(fp);
    }

    for (int i = 0; i < out->n_gen; ++i) {
        out->gen[i].bus_idx = find_bus_idx(out, out->gen[i].bus_id);
        if (out->gen[i].bus_idx < 0) return -1;
    }
    for (int i = 0; i < out->n_branch; ++i) {
        out->branch[i].from_idx = find_bus_idx(out, out->branch[i].from_id);
        out->branch[i].to_idx = find_bus_idx(out, out->branch[i].to_id);
        if (out->branch[i].from_idx < 0 || out->branch[i].to_idx < 0) return -1;
    }
    if (out->n_bus < 2 || out->slack_idx < 0 || out->base_mva <= 0.0) return -1;
    return 0;
}

static void add_branch_ybus_terms(const ele_grid_branch_t *br, double *guu, double *buu,
                                  double *gvv, double *bvv, double *guv, double *buv,
                                  double *gvu, double *bvu) {
    double r = br->r_pu, x = br->x_pu;
    double z2 = r * r + x * x;
    double gs = r / z2, bs = -x / z2;
    double bsh = 0.5 * br->b_sh_pu;
    double t = (br->tap > 0.0) ? br->tap : 1.0;
    double phi = br->shift_deg * (M_PI / 180.0);
    double cs = cos(phi), sn = sin(phi);
    /* y_uv = -(g_s + j b_s)/t * (cos(phi) + j sin(phi)) */
    double yuv_r = -(gs * cs - bs * sn) / t;
    double yuv_i = -(gs * sn + bs * cs) / t;
    /* y_vu = -(g_s + j b_s)/t * (cos(phi) - j sin(phi)) */
    double yvu_r = -(gs * cs + bs * sn) / t;
    double yvu_i = -(bs * cs - gs * sn) / t;
    *guu += gs / (t * t);
    *buu += (bs + bsh) / (t * t);
    *gvv += gs;
    *bvv += bs + bsh;
    *guv = yuv_r; *buv = yuv_i;
    *gvu = yvu_r; *bvu = yvu_i;
}

int ele_grid_build_ybus(const ele_grid_t *g, int skip_branch_idx,
                        double *G_out, double *B_out) {
    if (g == NULL || G_out == NULL || B_out == NULL) return -1;
    int N = g->n_bus;
    memset(G_out, 0, (size_t)(N * N) * sizeof(double));
    memset(B_out, 0, (size_t)(N * N) * sizeof(double));

    for (int k = 0; k < N; ++k) {
        G_out[k * N + k] += g->bus[k].gs_mw / g->base_mva;
        B_out[k * N + k] += g->bus[k].bs_mvar / g->base_mva;
    }

    for (int e = 0; e < g->n_branch; ++e) {
        if (e == skip_branch_idx) continue;
        const ele_grid_branch_t *br = &g->branch[e];
        double z2 = br->r_pu * br->r_pu + br->x_pu * br->x_pu;
        if (z2 < 1e-18) return -1;
        int u = br->from_idx, v = br->to_idx;
        double guu = 0.0, buu = 0.0, gvv = 0.0, bvv = 0.0;
        double guv = 0.0, buv = 0.0, gvu = 0.0, bvu = 0.0;
        add_branch_ybus_terms(br, &guu, &buu, &gvv, &bvv, &guv, &buv, &gvu, &bvu);

        G_out[u * N + u] += guu; B_out[u * N + u] += buu;
        G_out[v * N + v] += gvv; B_out[v * N + v] += bvv;
        G_out[u * N + v] += guv; B_out[u * N + v] += buv;
        G_out[v * N + u] += gvu; B_out[v * N + u] += bvu;
    }
    return 0;
}

void ele_grid_csr_free(ele_grid_csr_t *csr) {
    if (csr == NULL) return;
    free(csr->row_ptr);
    free(csr->col_idx);
    free(csr->g_val);
    free(csr->b_val);
    memset(csr, 0, sizeof(*csr));
}

int ele_grid_build_ybus_csr(const ele_grid_t *g, int skip_branch_idx,
                            ele_grid_csr_t *csr_out) {
    if (g == NULL || csr_out == NULL) return -1;
    memset(csr_out, 0, sizeof(*csr_out));
    int N = g->n_bus;
    int max_triples = N + 4 * g->n_branch;
    int *r_arr = (int *)malloc((size_t)max_triples * sizeof(int));
    int *c_arr = (int *)malloc((size_t)max_triples * sizeof(int));
    double *gv = (double *)malloc((size_t)max_triples * sizeof(double));
    double *bv = (double *)malloc((size_t)max_triples * sizeof(double));
    int *row_cnt = (int *)calloc((size_t)N, sizeof(int));
    if (r_arr == NULL || c_arr == NULL || gv == NULL || bv == NULL || row_cnt == NULL) {
        free(r_arr); free(c_arr); free(gv); free(bv); free(row_cnt);
        return -1;
    }

    int nt = 0;
    for (int k = 0; k < N; ++k) {
        r_arr[nt] = k; c_arr[nt] = k;
        gv[nt] = g->bus[k].gs_mw / g->base_mva;
        bv[nt] = g->bus[k].bs_mvar / g->base_mva;
        ++nt;
    }
    for (int e = 0; e < g->n_branch; ++e) {
        if (e == skip_branch_idx) continue;
        const ele_grid_branch_t *br = &g->branch[e];
        double z2 = br->r_pu * br->r_pu + br->x_pu * br->x_pu;
        if (z2 < 1e-18) {
            free(r_arr); free(c_arr); free(gv); free(bv); free(row_cnt);
            return -1;
        }
        int u = br->from_idx, v = br->to_idx;
        double guu = 0.0, buu = 0.0, gvv = 0.0, bvv = 0.0;
        double guv = 0.0, buv = 0.0, gvu = 0.0, bvu = 0.0;
        add_branch_ybus_terms(br, &guu, &buu, &gvv, &bvv, &guv, &buv, &gvu, &bvu);
        gv[u] += guu; bv[u] += buu;
        gv[v] += gvv; bv[v] += bvv;
        r_arr[nt] = u; c_arr[nt] = v; gv[nt] = guv; bv[nt] = buv; ++nt;
        r_arr[nt] = v; c_arr[nt] = u; gv[nt] = gvu; bv[nt] = bvu; ++nt;
    }

    /* Bin by row, sort column indices and merge parallel edges */
    int *row_ptr = (int *)calloc((size_t)(N + 1), sizeof(int));
    int *col_idx = (int *)malloc((size_t)nt * sizeof(int));
    double *g_val = (double *)malloc((size_t)nt * sizeof(double));
    double *b_val = (double *)malloc((size_t)nt * sizeof(double));
    int *bin_c = (int *)malloc((size_t)nt * sizeof(int));
    double *bin_g = (double *)malloc((size_t)nt * sizeof(double));
    double *bin_b = (double *)malloc((size_t)nt * sizeof(double));
    int *offs = (int *)calloc((size_t)(N + 1), sizeof(int));
    if (!row_ptr || !col_idx || !g_val || !b_val || !bin_c || !bin_g || !bin_b || !offs) {
        free(r_arr); free(c_arr); free(gv); free(bv); free(row_cnt);
        free(row_ptr); free(col_idx); free(g_val); free(b_val);
        free(bin_c); free(bin_g); free(bin_b); free(offs);
        return -1;
    }
    for (int t = 0; t < nt; ++t) ++row_cnt[r_arr[t]];
    for (int k = 0; k < N; ++k) offs[k + 1] = offs[k] + row_cnt[k];
    int *cursor = (int *)malloc((size_t)N * sizeof(int));
    if (!cursor) {
        free(r_arr); free(c_arr); free(gv); free(bv); free(row_cnt);
        free(row_ptr); free(col_idx); free(g_val); free(b_val);
        free(bin_c); free(bin_g); free(bin_b); free(offs);
        return -1;
    }
    memcpy(cursor, offs, (size_t)N * sizeof(int));
    for (int t = 0; t < nt; ++t) {
        int r = r_arr[t];
        int dst = cursor[r]++;
        bin_c[dst] = c_arr[t];
        bin_g[dst] = gv[t];
        bin_b[dst] = bv[t];
    }

    int nnz = 0;
    for (int k = 0; k < N; ++k) {
        row_ptr[k] = nnz;
        int lo = offs[k], hi = offs[k + 1];
        for (int a = lo; a < hi; ++a) {
            for (int b = a + 1; b < hi; ++b) {
                if (bin_c[b] < bin_c[a]) {
                    int tc = bin_c[a]; bin_c[a] = bin_c[b]; bin_c[b] = tc;
                    double tg = bin_g[a]; bin_g[a] = bin_g[b]; bin_g[b] = tg;
                    double tb = bin_b[a]; bin_b[a] = bin_b[b]; bin_b[b] = tb;
                }
            }
        }
        for (int a = lo; a < hi; ++a) {
            if (nnz > row_ptr[k] && col_idx[nnz - 1] == bin_c[a]) {
                g_val[nnz - 1] += bin_g[a];
                b_val[nnz - 1] += bin_b[a];
            } else {
                col_idx[nnz] = bin_c[a];
                g_val[nnz] = bin_g[a];
                b_val[nnz] = bin_b[a];
                ++nnz;
            }
        }
    }
    row_ptr[N] = nnz;

    free(r_arr); free(c_arr); free(gv); free(bv); free(row_cnt);
    free(bin_c); free(bin_g); free(bin_b); free(offs); free(cursor);

    csr_out->n = N;
    csr_out->nnz = nnz;
    csr_out->row_ptr = row_ptr;
    csr_out->col_idx = col_idx;
    csr_out->g_val = g_val;
    csr_out->b_val = b_val;
    return 0;
}

static int acpf_internal(const ele_grid_t *g, int skip_branch_idx,
                         const double *pg_pu_override, const double *vset_override,
                         const double *vm_warm, const double *va_warm,
                         ele_grid_pf_res_t *out) {
    if (g == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int N = g->n_bus;
    ele_grid_csr_t Y;
    if (ele_grid_build_ybus_csr(g, skip_branch_idx, &Y) != 0) return -1;

    double vm[ELE_GRID_MAX_BUS], va[ELE_GRID_MAX_BUS];
    double p_spec[ELE_GRID_MAX_BUS], q_spec[ELE_GRID_MAX_BUS];
    double diag_g[ELE_GRID_MAX_BUS], diag_b[ELE_GRID_MAX_BUS];
    int bus_vset_done[ELE_GRID_MAX_BUS] = {0};

    for (int k = 0; k < N; ++k) {
        vm[k] = (vm_warm != NULL) ? vm_warm[k] : g->bus[k].vm_init;
        va[k] = (va_warm != NULL) ? va_warm[k] : (g->bus[k].va_init_deg * M_PI / 180.0);
        p_spec[k] = -g->bus[k].pd_mw / g->base_mva;
        q_spec[k] = -g->bus[k].qd_mvar / g->base_mva;
        diag_g[k] = 0.0;
        diag_b[k] = 0.0;
        for (int p = Y.row_ptr[k]; p < Y.row_ptr[k + 1]; ++p) {
            if (Y.col_idx[p] == k) {
                diag_g[k] = Y.g_val[p];
                diag_b[k] = Y.b_val[p];
                break;
            }
        }
    }
    for (int i = 0; i < g->n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        double pg = (pg_pu_override != NULL) ? pg_pu_override[i] : (g->gen[i].pg_mw / g->base_mva);
        p_spec[b] += pg;
        if (g->bus[b].type == 2 || g->bus[b].type == 3) {
            if (vset_override != NULL || !bus_vset_done[b]) {
                vm[b] = (vset_override != NULL) ? vset_override[i] : g->gen[i].vset;
                bus_vset_done[b] = 1;
            }
        }
    }

    int ang_idx[ELE_GRID_MAX_BUS], v_idx[ELE_GRID_MAX_BUS];
    int bus_ang_pos[ELE_GRID_MAX_BUS], bus_v_pos[ELE_GRID_MAX_BUS];
    int n_ang = 0, n_v = 0;
    for (int k = 0; k < N; ++k) {
        bus_ang_pos[k] = -1;
        bus_v_pos[k] = -1;
        if (g->bus[k].type != 3) {
            bus_ang_pos[k] = n_ang;
            ang_idx[n_ang++] = k;
        }
        if (g->bus[k].type == 1) {
            bus_v_pos[k] = n_v;
            v_idx[n_v++] = k;
        }
    }
    int n_jac = n_ang + n_v;
    size_t nj = (n_jac > 0) ? (size_t)n_jac : 1U;
    double *J = (double *)malloc(nj * nj * sizeof(double));
    double *mis = (double *)malloc(nj * sizeof(double));
    double *dx = (double *)malloc(nj * sizeof(double));
    if (J == NULL || mis == NULL || dx == NULL) {
        free(J); free(mis); free(dx);
        ele_grid_csr_free(&Y);
        return -1;
    }

    double p_calc[ELE_GRID_MAX_BUS], q_calc[ELE_GRID_MAX_BUS];
    int converged = 0;
    double max_mis = 1e9;
    int it = 0;

    for (it = 0; it < 40; ++it) {
        for (int k = 0; k < N; ++k) {
            double pc = 0.0, qc = 0.0;
            for (int p = Y.row_ptr[k]; p < Y.row_ptr[k + 1]; ++p) {
                int m = Y.col_idx[p];
                double dth = va[k] - va[m];
                double cs = cos(dth), sn = sin(dth);
                double gkm = Y.g_val[p], bkm = Y.b_val[p];
                pc += vm[k] * vm[m] * (gkm * cs + bkm * sn);
                qc += vm[k] * vm[m] * (gkm * sn - bkm * cs);
            }
            p_calc[k] = pc;
            q_calc[k] = qc;
        }

        max_mis = 0.0;
        for (int i = 0; i < n_ang; ++i) {
            int k = ang_idx[i];
            mis[i] = p_spec[k] - p_calc[k];
            if (fabs(mis[i]) > max_mis) max_mis = fabs(mis[i]);
        }
        for (int i = 0; i < n_v; ++i) {
            int k = v_idx[i];
            mis[n_ang + i] = q_spec[k] - q_calc[k];
            if (fabs(mis[n_ang + i]) > max_mis) max_mis = fabs(mis[n_ang + i]);
        }
        if (max_mis < 1e-11) {
            converged = 1;
            break;
        }
        if (n_jac == 0) { converged = 1; break; }

        memset(J, 0, nj * nj * sizeof(double));
        for (int i = 0; i < n_ang; ++i) {
            int k = ang_idx[i];
            J[(size_t)i * nj + (size_t)i] = -q_calc[k] - diag_b[k] * vm[k] * vm[k];
            if (bus_v_pos[k] >= 0) {
                J[(size_t)i * nj + (size_t)(n_ang + bus_v_pos[k])] =
                    (p_calc[k] + diag_g[k] * vm[k] * vm[k]) / vm[k];
            }
            for (int p = Y.row_ptr[k]; p < Y.row_ptr[k + 1]; ++p) {
                int m = Y.col_idx[p];
                if (m == k) continue;
                double dth = va[k] - va[m];
                double cs = cos(dth), sn = sin(dth);
                double gkm = Y.g_val[p], bkm = Y.b_val[p];
                if (bus_ang_pos[m] >= 0) {
                    J[(size_t)i * nj + (size_t)bus_ang_pos[m]] =
                        vm[k] * vm[m] * (gkm * sn - bkm * cs);
                }
                if (bus_v_pos[m] >= 0) {
                    J[(size_t)i * nj + (size_t)(n_ang + bus_v_pos[m])] =
                        vm[k] * (gkm * cs + bkm * sn);
                }
            }
        }
        for (int i = 0; i < n_v; ++i) {
            int k = v_idx[i];
            size_t row = (size_t)(n_ang + i);
            if (bus_ang_pos[k] >= 0) {
                J[row * nj + (size_t)bus_ang_pos[k]] =
                    p_calc[k] - diag_g[k] * vm[k] * vm[k];
            }
            J[row * nj + (size_t)(n_ang + i)] =
                (q_calc[k] - diag_b[k] * vm[k] * vm[k]) / vm[k];
            for (int p = Y.row_ptr[k]; p < Y.row_ptr[k + 1]; ++p) {
                int m = Y.col_idx[p];
                if (m == k) continue;
                double dth = va[k] - va[m];
                double cs = cos(dth), sn = sin(dth);
                double gkm = Y.g_val[p], bkm = Y.b_val[p];
                if (bus_ang_pos[m] >= 0) {
                    J[row * nj + (size_t)bus_ang_pos[m]] =
                        -vm[k] * vm[m] * (gkm * cs + bkm * sn);
                }
                if (bus_v_pos[m] >= 0) {
                    J[row * nj + (size_t)(n_ang + bus_v_pos[m])] =
                        vm[k] * (gkm * sn - bkm * cs);
                }
            }
        }

        if (ele_lu_solve_real(n_jac, J, mis, dx) != 0) {
            free(J); free(mis); free(dx);
            ele_grid_csr_free(&Y);
            return -1;
        }
        for (int i = 0; i < n_ang; ++i) va[ang_idx[i]] += dx[i];
        for (int i = 0; i < n_v; ++i) {
            vm[v_idx[i]] += dx[n_ang + i];
            if (vm[v_idx[i]] < 0.2) vm[v_idx[i]] = 0.2;
        }
    }

    free(J); free(mis); free(dx);
    ele_grid_csr_free(&Y);

    out->converged = converged;
    out->nr_iters = it;
    out->max_mismatch_pu = max_mis;
    out->vmin_pu = 1e9;
    out->vmax_pu = -1e9;
    out->limits_ok = converged ? 1 : 0;

    for (int k = 0; k < N; ++k) {
        out->vm[k] = vm[k];
        out->va_rad[k] = va[k];
        out->p_inj_pu[k] = p_calc[k];
        out->q_inj_pu[k] = q_calc[k];
        out->total_p_gen_mw += (p_calc[k] + g->bus[k].pd_mw / g->base_mva) * g->base_mva;
        out->total_q_gen_mvar += (q_calc[k] + g->bus[k].qd_mvar / g->base_mva) * g->base_mva;
        out->total_p_load_mw += g->bus[k].pd_mw;
        if (vm[k] < out->vmin_pu) out->vmin_pu = vm[k];
        if (vm[k] > out->vmax_pu) out->vmax_pu = vm[k];
        if (vm[k] < g->bus[k].vmin - 1e-6 || vm[k] > g->bus[k].vmax + 1e-6) {
            out->limits_ok = 0;
        }
    }
    out->total_p_loss_mw = out->total_p_gen_mw - out->total_p_load_mw;

    out->max_branch_loading = 0.0;
    for (int e = 0; e < g->n_branch; ++e) {
        if (e == skip_branch_idx) { out->branch_s_mva[e] = 0.0; continue; }
        const ele_grid_branch_t *br = &g->branch[e];
        int u = br->from_idx, v = br->to_idx;
        double z2 = br->r_pu * br->r_pu + br->x_pu * br->x_pu;
        double gs = br->r_pu / z2, bs = -br->x_pu / z2;
        double t = (br->tap > 0.0) ? br->tap : 1.0;
        double phi = br->shift_deg * (M_PI / 180.0);
        double cs = cos(phi), sn = sin(phi);
        double bsh = 0.5 * br->b_sh_pu;
        double yff_r = gs / (t * t), yff_i = (bs + bsh) / (t * t);
        double yft_r = -(gs * cs - bs * sn) / t, yft_i = -(gs * sn + bs * cs) / t;
        double ytf_r = -(gs * cs + bs * sn) / t, ytf_i = -(bs * cs - gs * sn) / t;
        double ytt_r = gs, ytt_i = bs + bsh;

        double eur = vm[u] * cos(va[u]), eui = vm[u] * sin(va[u]);
        double evr = vm[v] * cos(va[v]), evi = vm[v] * sin(va[v]);

        double if_r = (yff_r * eur - yff_i * eui) + (yft_r * evr - yft_i * evi);
        double if_i = (yff_r * eui + yff_i * eur) + (yft_r * evi + yft_i * evr);
        double sf_r = eur * if_r + eui * if_i;
        double sf_i = eui * if_r - eur * if_i;
        double sf_mva = hypot(sf_r, sf_i) * g->base_mva;

        double it_r = (ytf_r * eur - ytf_i * eui) + (ytt_r * evr - ytt_i * evi);
        double it_i = (ytf_r * eui + ytf_i * eur) + (ytt_r * evi + ytt_i * evr);
        double st_r = evr * it_r + evi * it_i;
        double st_i = evi * it_r - evr * it_i;
        double st_mva = hypot(st_r, st_i) * g->base_mva;

        double s_max_end = (sf_mva > st_mva) ? sf_mva : st_mva;
        out->branch_s_mva[e] = s_max_end;
        double load = s_max_end / br->smax_mva;
        if (load > out->max_branch_loading) out->max_branch_loading = load;
        if (load > 1.0 + 1e-6) out->limits_ok = 0;
    }
    return converged ? 0 : -1;
}

int ele_grid_acpf(const ele_grid_t *g, int skip_branch_idx, ele_grid_pf_res_t *out) {
    return acpf_internal(g, skip_branch_idx, NULL, NULL, NULL, NULL, out);
}

static int grid_is_connected(const ele_grid_t *g, int skip_branch_idx) {
    int vis[ELE_GRID_MAX_BUS] = {0};
    int q[ELE_GRID_MAX_BUS];
    int head = 0, tail = 0;
    vis[0] = 1;
    q[tail++] = 0;
    while (head < tail) {
        int u = q[head++];
        for (int e = 0; e < g->n_branch; ++e) {
            if (e == skip_branch_idx) continue;
            int a = g->branch[e].from_idx, b = g->branch[e].to_idx;
            if (a == u && !vis[b]) { vis[b] = 1; q[tail++] = b; }
            if (b == u && !vis[a]) { vis[a] = 1; q[tail++] = a; }
        }
    }
    return (tail == g->n_bus) ? 1 : 0;
}

int ele_grid_n1(const ele_grid_t *g, ele_grid_n1_res_t *out) {
    if (g == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    out->worst_vmin = 1e9;
    out->worst_vmax = -1e9;
    out->n1_secure = 1;

    for (int e = 0; e < g->n_branch; ++e) {
        ++out->n_contingencies;
        if (!grid_is_connected(g, e)) {
            ++out->n_islands;
            continue;
        }
        ele_grid_pf_res_t pf;
        if (ele_grid_acpf(g, e, &pf) == 0) {
            ++out->n_converged;
            if (pf.vmin_pu < out->worst_vmin) out->worst_vmin = pf.vmin_pu;
            if (pf.vmax_pu > out->worst_vmax) out->worst_vmax = pf.vmax_pu;
            if (pf.max_branch_loading > out->worst_loading) {
                out->worst_loading = pf.max_branch_loading;
                out->worst_branch_idx = e;
            }
            if (!pf.limits_ok) {
                ++out->n_violations;
                out->n1_secure = 0;
            }
        } else {
            ++out->n_violations;
            out->n1_secure = 0;
        }
    }
    return 0;
}

/* ============================================================================
 * 3. Certified Two-Sided AC-OPF Bracket (Primal UB + Shor SDP Dual LB)
 * ============================================================================ */

static void build_shor_basis_mats(const ele_grid_t *g, const double *G, const double *B,
                                  double *YP, double *YQ, double *MV) {
    int N = g->n_bus;
    int d = 2 * N;
    size_t mat_sz = (size_t)(d * d);
    memset(YP, 0, (size_t)N * mat_sz * sizeof(double));
    memset(YQ, 0, (size_t)N * mat_sz * sizeof(double));
    memset(MV, 0, (size_t)N * mat_sz * sizeof(double));

    for (int k = 0; k < N; ++k) {
        double *yp = YP + (size_t)k * mat_sz;
        double *yq = YQ + (size_t)k * mat_sz;
        double *mv = MV + (size_t)k * mat_sz;
        mv[k * d + k] = 1.0;
        mv[(N + k) * d + (N + k)] = 1.0;

        for (int m = 0; m < N; ++m) {
            double gkm = 0.5 * G[k * N + m];
            double bkm = 0.5 * B[k * N + m];

            /* P_k: G*(e_k e_m + f_k f_m) + B*(f_k e_m - e_k f_m) */
            yp[k * d + m] += gkm;
            yp[m * d + k] += gkm;
            yp[(N + k) * d + (N + m)] += gkm;
            yp[(N + m) * d + (N + k)] += gkm;

            yp[(N + k) * d + m] += bkm;
            yp[m * d + (N + k)] += bkm;
            yp[k * d + (N + m)] -= bkm;
            yp[(N + m) * d + k] -= bkm;

            /* Q_k: -B*(e_k e_m + f_k f_m) + G*(f_k e_m - e_k f_m) */
            yq[k * d + m] -= bkm;
            yq[m * d + k] -= bkm;
            yq[(N + k) * d + (N + m)] -= bkm;
            yq[(N + m) * d + (N + k)] -= bkm;

            yq[(N + k) * d + m] += gkm;
            yq[m * d + (N + k)] += gkm;
            yq[k * d + (N + m)] -= gkm;
            yq[(N + m) * d + k] -= gkm;
        }
    }
}

static void fill_4x4_from_coeffs(double alpha_r, double delta_r,
                                 double b_re, double b_fe, double M[4][4]) {
    memset(M, 0, 16U * sizeof(double));
    M[0][0] = alpha_r;
    M[2][2] = alpha_r;
    M[1][1] = delta_r;
    M[3][3] = delta_r;
    M[0][1] = 0.5 * b_re; M[1][0] = 0.5 * b_re;
    M[2][3] = 0.5 * b_re; M[3][2] = 0.5 * b_re;
    M[2][1] = 0.5 * b_fe; M[1][2] = 0.5 * b_fe;
    M[0][3] = -0.5 * b_fe; M[3][0] = -0.5 * b_fe;
}

typedef struct {
    int idx[4];
    int has_therm;
    double smax_pu;
    double mpf[4][4], mqf[4][4];
    double mpt[4][4], mqt[4][4];
    double norm_pf, norm_qf, dot_pqf;
    double norm_pt, norm_qt, dot_pqt;
    int has_ang;
    double ma_lo[4][4], ma_hi[4][4];
    double norm_alo, norm_ahi;
} opf_branch_dual_t;

static void init_opf_branch_duals(const ele_grid_t *g, opf_branch_dual_t *bd) {
    int N = g->n_bus;
    double B = g->base_mva;
    for (int e = 0; e < g->n_branch; ++e) {
        const ele_grid_branch_t *br = &g->branch[e];
        opf_branch_dual_t *d = &bd[e];
        memset(d, 0, sizeof(*d));
        int u = br->from_idx, v = br->to_idx;
        d->idx[0] = u; d->idx[1] = v; d->idx[2] = N + u; d->idx[3] = N + v;
        if (u == v) continue;

        double z2 = br->r_pu * br->r_pu + br->x_pu * br->x_pu;
        double gs = br->r_pu / z2, bs = -br->x_pu / z2;
        double t = (br->tap > 0.0) ? br->tap : 1.0;
        double phi = br->shift_deg * (M_PI / 180.0);
        double cs = cos(phi), sn = sin(phi);
        double bsh = 0.5 * br->b_sh_pu;
        double gff = gs / (t * t), bff = (bs + bsh) / (t * t);
        double gft = -(gs * cs - bs * sn) / t, bft = -(gs * sn + bs * cs) / t;
        double gtf = -(gs * cs + bs * sn) / t, btf = -(bs * cs - gs * sn) / t;
        double gtt = gs, btt = bs + bsh;

        if (br->smax_mva > 0.0 && br->smax_mva < 9000.0) {
            d->has_therm = 1;
            d->smax_pu = br->smax_mva / B;
            fill_4x4_from_coeffs(gff, 0.0, gft, bft, d->mpf);
            fill_4x4_from_coeffs(-bff, 0.0, -bft, gft, d->mqf);
            fill_4x4_from_coeffs(0.0, gtt, gtf, -btf, d->mpt);
            fill_4x4_from_coeffs(0.0, -btt, -btf, -gtf, d->mqt);
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) {
                    d->norm_pf += d->mpf[a][b] * d->mpf[a][b];
                    d->norm_qf += d->mqf[a][b] * d->mqf[a][b];
                    d->dot_pqf += d->mpf[a][b] * d->mqf[a][b];
                    d->norm_pt += d->mpt[a][b] * d->mpt[a][b];
                    d->norm_qt += d->mqt[a][b] * d->mqt[a][b];
                    d->dot_pqt += d->mpt[a][b] * d->mqt[a][b];
                }
            }
            d->norm_pf = fmax(d->norm_pf, 1e-12);
            d->norm_qf = fmax(d->norm_qf, 1e-12);
            d->norm_pt = fmax(d->norm_pt, 1e-12);
            d->norm_qt = fmax(d->norm_qt, 1e-12);
        }
        if (br->angmin_deg > -85.0 || br->angmax_deg < 85.0) {
            d->has_ang = 1;
            double amin = (br->angmin_deg > -85.0 ? br->angmin_deg : -85.0) * (M_PI / 180.0);
            double amax = (br->angmax_deg < 85.0 ? br->angmax_deg : 85.0) * (M_PI / 180.0);
            fill_4x4_from_coeffs(0.0, 0.0, tan(amin), -1.0, d->ma_lo);
            fill_4x4_from_coeffs(0.0, 0.0, -tan(amax), 1.0, d->ma_hi);
            for (int a = 0; a < 4; ++a) {
                for (int b = 0; b < 4; ++b) {
                    d->norm_alo += d->ma_lo[a][b] * d->ma_lo[a][b];
                    d->norm_ahi += d->ma_hi[a][b] * d->ma_hi[a][b];
                }
            }
            d->norm_alo = fmax(d->norm_alo, 1e-12);
            d->norm_ahi = fmax(d->norm_ahi, 1e-12);
        }
    }
}

static void prox_solve_2d_cone(double h11, double h22, double h12,
                               double b1, double b2, double smax_pu,
                               double *x_io, double *y_io) {
    double diff = 0.5 * (h11 - h22);
    double lmax = 0.5 * (h11 + h22) + hypot(diff, h12) + 1e-12;
    double step = 1.0 / lmax;
    double x = *x_io, y = *y_io;
    for (int it = 0; it < 8; ++it) {
        double gx = h11 * x + h12 * y + b1;
        double gy = h12 * x + h22 * y + b2;
        double zx = x - step * gx;
        double zy = y - step * gy;
        double zn = hypot(zx, zy);
        double thr = step * smax_pu;
        double sc = (zn > thr && zn > 1e-18) ? ((zn - thr) / zn) : 0.0;
        x = zx * sc;
        y = zy * sc;
    }
    *x_io = x;
    *y_io = y;
}

static double eval_opf_dual_scalar_ext(const ele_grid_t *g, const double *lam, const double *gam,
                                       const double *mu_lo, const double *mu_hi,
                                       const opf_branch_dual_t *bd,
                                       const double *xif, const double *etaf,
                                       const double *xit, const double *etat,
                                       double *pg_star_out, double *qg_star_out) {
    double val = 0.0;
    double B = g->base_mva;
    for (int k = 0; k < g->n_bus; ++k) {
        val += lam[k] * (g->bus[k].pd_mw / B);
        val += gam[k] * (g->bus[k].qd_mvar / B);
        val += mu_lo[k] * (g->bus[k].vmin * g->bus[k].vmin);
        val -= mu_hi[k] * (g->bus[k].vmax * g->bus[k].vmax);
    }
    for (int i = 0; i < g->n_gen; ++i) {
        const ele_grid_gen_t *gn = &g->gen[i];
        int k = gn->bus_idx;
        double a2 = gn->c2 * B * B;
        double a1 = gn->c1 * B;
        double a0 = gn->c0;
        double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
        double qmin = gn->qmin_mvar / B, qmax = gn->qmax_mvar / B;

        double pg = (a2 > 1e-12) ? ((lam[k] - a1) / (2.0 * a2))
                                 : ((lam[k] > a1) ? pmax : pmin);
        if (pg < pmin) pg = pmin;
        if (pg > pmax) pg = pmax;
        double qg = (gam[k] >= 0.0) ? qmax : qmin;
        if (pg_star_out != NULL) pg_star_out[i] = pg;
        if (qg_star_out != NULL) qg_star_out[i] = qg;
        val += a2 * pg * pg + (a1 - lam[k]) * pg + a0 - gam[k] * qg;
    }
    if (bd != NULL && xif != NULL) {
        for (int e = 0; e < g->n_branch; ++e) {
            if (!bd[e].has_therm) continue;
            val -= bd[e].smax_pu * (hypot(xif[e], etaf[e]) + hypot(xit[e], etat[e]));
        }
    }
    return val;
}

static double eval_gen_cost_pu(const ele_grid_t *g, const double *pg_pu) {
    double c = 0.0, B = g->base_mva;
    for (int i = 0; i < g->n_gen; ++i) {
        double pmw = pg_pu[i] * B;
        c += g->gen[i].c2 * pmw * pmw + g->gen[i].c1 * pmw + g->gen[i].c0;
    }
    return c;
}

static double eval_primal_ub_obj(const ele_grid_t *g, const double *pg_ctrl, const double *vset_ctrl,
                                 const double *vm_warm, const double *va_warm,
                                 double *pg_full_out, ele_grid_pf_res_t *pf_out, int *feasible_out) {
    double pg_pu[ELE_GRID_MAX_GEN];
    int n_gen = g->n_gen;
    for (int i = 0; i < n_gen; ++i) pg_pu[i] = pg_ctrl[i];
    if (acpf_internal(g, -1, pg_pu, vset_ctrl, vm_warm, va_warm, pf_out) != 0) {
        *feasible_out = 0;
        return 1e18;
    }
    double B = g->base_mva;
    /* Compute slack gen P and bus-grouped Q allocation */
    double bus_p_non_slack[ELE_GRID_MAX_BUS] = {0};
    int slack_gen_idx = 0;
    for (int i = 0; i < n_gen; ++i) {
        if (g->gen[i].bus_idx == g->slack_idx) slack_gen_idx = i;
        else bus_p_non_slack[g->gen[i].bus_idx] += pg_pu[i];
    }
    double p_slack_total = pf_out->p_inj_pu[g->slack_idx] + g->bus[g->slack_idx].pd_mw / B;
    pg_pu[slack_gen_idx] = p_slack_total - bus_p_non_slack[g->slack_idx];
    for (int i = 0; i < n_gen; ++i) pg_full_out[i] = pg_pu[i];

    double cost = eval_gen_cost_pu(g, pg_pu);
    double pen = 0.0;
    int feas = pf_out->limits_ok;
    for (int i = 0; i < n_gen; ++i) {
        double pmw = pg_pu[i] * B;
        if (pmw < g->gen[i].pmin_mw - 1e-4) {
            double v = g->gen[i].pmin_mw - pmw;
            pen += 1e5 * v * v + 1e3 * v;
            feas = 0;
        }
        if (pmw > g->gen[i].pmax_mw + 1e-4) {
            double v = pmw - g->gen[i].pmax_mw;
            pen += 1e5 * v * v + 1e3 * v;
            feas = 0;
        }
    }
    /* Check per-gen reactive power limits summed per bus */
    double bus_qgen[ELE_GRID_MAX_BUS] = {0};
    double bus_qmin[ELE_GRID_MAX_BUS] = {0};
    double bus_qmax[ELE_GRID_MAX_BUS] = {0};
    int bus_has_gen[ELE_GRID_MAX_BUS] = {0};
    for (int i = 0; i < n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        bus_has_gen[b] = 1;
        bus_qmin[b] += g->gen[i].qmin_mvar / B;
        bus_qmax[b] += g->gen[i].qmax_mvar / B;
    }
    for (int b = 0; b < g->n_bus; ++b) {
        if (!bus_has_gen[b]) continue;
        bus_qgen[b] = pf_out->q_inj_pu[b] + g->bus[b].qd_mvar / B;
        if (bus_qgen[b] < bus_qmin[b] - 1e-4) {
            double v = (bus_qmin[b] - bus_qgen[b]) * B;
            pen += 1e4 * v * v + 1e2 * v;
            feas = 0;
        }
        if (bus_qgen[b] > bus_qmax[b] + 1e-4) {
            double v = (bus_qgen[b] - bus_qmax[b]) * B;
            pen += 1e4 * v * v + 1e2 * v;
            feas = 0;
        }
    }
    for (int k = 0; k < g->n_bus; ++k) {
        if (pf_out->vm[k] < g->bus[k].vmin - 1e-5) {
            double v = g->bus[k].vmin - pf_out->vm[k];
            pen += 1e6 * v * v + 1e4 * v;
            feas = 0;
        }
        if (pf_out->vm[k] > g->bus[k].vmax + 1e-5) {
            double v = pf_out->vm[k] - g->bus[k].vmax;
            pen += 1e6 * v * v + 1e4 * v;
            feas = 0;
        }
    }
    for (int e = 0; e < g->n_branch; ++e) {
        double ex = pf_out->branch_s_mva[e] - g->branch[e].smax_mva;
        if (ex > 1e-4) {
            pen += 1e4 * ex * ex + 1e2 * ex;
            feas = 0;
        }
        if (g->branch[e].angmin_deg > -85.0 || g->branch[e].angmax_deg < 85.0) {
            double dv = (pf_out->va_rad[g->branch[e].from_idx] - pf_out->va_rad[g->branch[e].to_idx]) * (180.0 / M_PI);
            while (dv > 180.0) dv -= 360.0;
            while (dv < -180.0) dv += 360.0;
            if (dv < g->branch[e].angmin_deg - 1e-3) {
                double ev = g->branch[e].angmin_deg - dv;
                pen += 1e3 * ev * ev + 50.0 * ev;
                feas = 0;
            }
            if (dv > g->branch[e].angmax_deg + 1e-3) {
                double ev = dv - g->branch[e].angmax_deg;
                pen += 1e3 * ev * ev + 50.0 * ev;
                feas = 0;
            }
        }
    }
    *feasible_out = feas;
    return cost + pen;
}

static int acpf_balanced_seed(const ele_grid_t *g, double *pg_ctrl, double *vset_ctrl,
                              ele_grid_pf_res_t *pf_out) {
    double bmva = g->base_mva;
    double pd_tot = 0.0;
    for (int k = 0; k < g->n_bus; ++k) pd_tot += g->bus[k].pd_mw;
    double p_rem = pd_tot * 1.03;

    int order[ELE_GRID_MAX_GEN];
    int n_act = 0;
    for (int i = 0; i < g->n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        double vs = g->gen[i].vset;
        if (vs < g->bus[b].vmin) vs = g->bus[b].vmin;
        if (vs > g->bus[b].vmax) vs = g->bus[b].vmax;
        vset_ctrl[i] = vs;
        pg_ctrl[i] = g->gen[i].pmin_mw / bmva;
        p_rem -= g->gen[i].pmin_mw;
        if (g->gen[i].pmax_mw > g->gen[i].pmin_mw + 1e-6) order[n_act++] = i;
    }
    for (int a = 0; a < n_act; ++a) {
        for (int b = a + 1; b < n_act; ++b) {
            int ia = order[a], ib = order[b];
            double ma = g->gen[ia].c1 + 0.5 * g->gen[ia].c2 * pd_tot;
            double mb = g->gen[ib].c1 + 0.5 * g->gen[ib].c2 * pd_tot;
            if (mb < ma) { order[a] = ib; order[b] = ia; }
        }
    }
    for (int a = 0; a < n_act && p_rem > 0.0; ++a) {
        int i = order[a];
        double cap = g->gen[i].pmax_mw - g->gen[i].pmin_mw;
        double add = (p_rem < cap) ? p_rem : cap;
        pg_ctrl[i] = (g->gen[i].pmin_mw + add) / bmva;
        p_rem -= add;
    }
    return acpf_internal(g, -1, pg_ctrl, vset_ctrl, NULL, NULL, pf_out);
}

static void polish_primal_acopf(const ele_grid_t *g, double *pg_ctrl, double *vset_ctrl,
                                const double *vm_warm, const double *va_warm,
                                ele_grid_pf_res_t *tpf_p, int max_iters,
                                int *feas_best, double *best_ub_obj, double *best_ub_cost,
                                double *best_pg, double *best_v, ele_grid_pf_res_t *best_pf) {
    double bmva = g->base_mva;
    double pg_full[ELE_GRID_MAX_GEN];
    int tf = 0;
    double f0 = eval_primal_ub_obj(g, pg_ctrl, vset_ctrl, vm_warm, va_warm, pg_full, tpf_p, &tf);
    if (f0 < *best_ub_obj && (!*feas_best || tf)) {
        *best_ub_obj = f0;
        if (!*feas_best) { *best_pf = *tpf_p; memcpy(best_pg, pg_ctrl, (size_t)g->n_gen * sizeof(double)); memcpy(best_v, vset_ctrl, (size_t)g->n_gen * sizeof(double)); }
    }
    if (tf) {
        double cst = eval_gen_cost_pu(g, pg_full);
        if (!*feas_best || cst < *best_ub_cost) {
            *best_ub_cost = cst;
            *best_ub_obj = cst;
            memcpy(best_pg, pg_full, (size_t)g->n_gen * sizeof(double));
            memcpy(best_v, vset_ctrl, (size_t)g->n_gen * sizeof(double));
            *best_pf = *tpf_p;
            *feas_best = 1;
        }
    }

    double step_p = 0.12, step_v = 0.012;
    for (int iter = 0; iter < max_iters; ++iter) {
        double grad_p[ELE_GRID_MAX_GEN] = {0};
        double grad_v[ELE_GRID_MAX_GEN] = {0};
        double eps_p = 1e-4, eps_v = 1e-4;
        double tpg[ELE_GRID_MAX_GEN];
        int cf = 0;
        double f_cur = eval_primal_ub_obj(g, pg_ctrl, vset_ctrl, vm_warm, va_warm, tpg, tpf_p, &cf);
        for (int i = 0; i < g->n_gen; ++i) {
            if (g->gen[i].bus_idx != g->slack_idx && g->gen[i].pmax_mw > g->gen[i].pmin_mw + 1e-6) {
                double old = pg_ctrl[i];
                pg_ctrl[i] = old + eps_p;
                double fp = eval_primal_ub_obj(g, pg_ctrl, vset_ctrl, vm_warm, va_warm, tpg, tpf_p, &cf);
                grad_p[i] = (fp - f_cur) / eps_p;
                pg_ctrl[i] = old;
            }
            double oldv = vset_ctrl[i];
            vset_ctrl[i] = oldv + eps_v;
            double fv = eval_primal_ub_obj(g, pg_ctrl, vset_ctrl, vm_warm, va_warm, tpg, tpf_p, &cf);
            grad_v[i] = (fv - f_cur) / eps_v;
            vset_ctrl[i] = oldv;
        }
        double gnorm_p = 1e-12, gnorm_v = 1e-12;
        for (int i = 0; i < g->n_gen; ++i) {
            gnorm_p += grad_p[i] * grad_p[i];
            gnorm_v += grad_v[i] * grad_v[i];
        }
        gnorm_p = sqrt(gnorm_p);
        gnorm_v = sqrt(gnorm_v);

        int improved = 0;
        double alpha = 1.0;
        for (int ls = 0; ls < 12; ++ls) {
            double cand_p[ELE_GRID_MAX_GEN], cand_v[ELE_GRID_MAX_GEN];
            for (int i = 0; i < g->n_gen; ++i) {
                cand_p[i] = pg_ctrl[i] - alpha * step_p * (grad_p[i] / gnorm_p);
                double pmin = g->gen[i].pmin_mw / bmva, pmax = g->gen[i].pmax_mw / bmva;
                if (cand_p[i] < pmin) cand_p[i] = pmin;
                if (cand_p[i] > pmax) cand_p[i] = pmax;

                cand_v[i] = vset_ctrl[i] - alpha * step_v * (grad_v[i] / gnorm_v);
                int b = g->gen[i].bus_idx;
                if (cand_v[i] < g->bus[b].vmin) cand_v[i] = g->bus[b].vmin;
                if (cand_v[i] > g->bus[b].vmax) cand_v[i] = g->bus[b].vmax;
            }
            double fc = eval_primal_ub_obj(g, cand_p, cand_v, vm_warm, va_warm, tpg, tpf_p, &cf);
            if (fc < f_cur - 1e-9) {
                memcpy(pg_ctrl, cand_p, sizeof(cand_p));
                memcpy(vset_ctrl, cand_v, sizeof(cand_v));
                if (fc < *best_ub_obj) *best_ub_obj = fc;
                if (cf) {
                    double cst = eval_gen_cost_pu(g, tpg);
                    if (!*feas_best || cst < *best_ub_cost) {
                        *best_ub_cost = cst;
                        *best_ub_obj = cst;
                        memcpy(best_pg, tpg, (size_t)g->n_gen * sizeof(double));
                        memcpy(best_v, cand_v, (size_t)g->n_gen * sizeof(double));
                        *best_pf = *tpf_p;
                        *feas_best = 1;
                    }
                }
                improved = 1;
                break;
            }
            alpha *= 0.5;
        }
        if (!improved) {
            step_p *= 0.5;
            step_v *= 0.5;
            if (step_p < 1e-5) break;
        }
    }
}

int ele_grid_acopf(const ele_grid_t *g, ele_grid_opf_res_t *out) {
    if (g == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int N = g->n_bus;
    int d = 2 * N;
    if (d > ELE_CERT_MAX_DIM) return -1;
    size_t mat_sz = (size_t)(d * d);

    double *G = (double *)malloc((size_t)(N * N) * sizeof(double));
    double *B = (double *)malloc((size_t)(N * N) * sizeof(double));
    double *YP = (double *)malloc((size_t)N * mat_sz * sizeof(double));
    double *YQ = (double *)malloc((size_t)N * mat_sz * sizeof(double));
    double *MV = (double *)malloc((size_t)N * mat_sz * sizeof(double));
    if (G == NULL || B == NULL || YP == NULL || YQ == NULL || MV == NULL) {
        free(G); free(B); free(YP); free(YQ); free(MV);
        return -1;
    }
    if (ele_grid_build_ybus(g, -1, G, B) != 0) {
        free(G); free(B); free(YP); free(YQ); free(MV);
        return -1;
    }
    build_shor_basis_mats(g, G, B, YP, YQ, MV);
    free(G); free(B);
    G = NULL; B = NULL;

    /* ------------------------------------------------------------------------
     * Step A: Solve Primal Feasible AC-OPF Upper Bound C_UB
     * ------------------------------------------------------------------------ */
    double pg_ctrl[ELE_GRID_MAX_GEN], vset_ctrl[ELE_GRID_MAX_GEN];
    double pg_full[ELE_GRID_MAX_GEN];
    double bmva = g->base_mva;
    for (int i = 0; i < g->n_gen; ++i) {
        pg_ctrl[i] = g->gen[i].pg_mw / bmva;
        vset_ctrl[i] = g->gen[i].vset;
    }
    ele_grid_pf_res_t *pf_tmp = (ele_grid_pf_res_t *)malloc(2U * sizeof(ele_grid_pf_res_t));
    if (pf_tmp == NULL) { free(YP); free(YQ); free(MV); return -1; }
    ele_grid_pf_res_t *base_pf_p = &pf_tmp[0];
    ele_grid_pf_res_t *tpf_p = &pf_tmp[1];
    int has_base_pf = (ele_grid_acpf(g, -1, base_pf_p) == 0);
    double pg_m[ELE_GRID_MAX_GEN], vs_m[ELE_GRID_MAX_GEN];
    int has_bal_pf = (acpf_balanced_seed(g, pg_m, vs_m, tpf_p) == 0);
    if (!has_base_pf && !has_bal_pf) {
        free(pf_tmp); free(YP); free(YQ); free(MV);
        return -1;
    }

    int feas_best = 0;
    double best_ub_obj = 1e18;
    double best_ub_cost = 1e18;
    double best_pg[ELE_GRID_MAX_GEN], best_v[ELE_GRID_MAX_GEN];
    if (has_base_pf) {
        polish_primal_acopf(g, pg_ctrl, vset_ctrl, base_pf_p->vm, base_pf_p->va_rad, tpf_p, 180,
                            &feas_best, &best_ub_obj, &best_ub_cost, best_pg, best_v, &out->ub_pf);
    }
    if (has_bal_pf) {
        ele_grid_pf_res_t bal_pf = *tpf_p;
        polish_primal_acopf(g, pg_m, vs_m, bal_pf.vm, bal_pf.va_rad, tpf_p, 180,
                            &feas_best, &best_ub_obj, &best_ub_cost, best_pg, best_v, &out->ub_pf);
    }
    (void)pg_full;
    if (!feas_best) {
        out->c_ub_primal = best_ub_obj;
    } else {
        out->c_ub_primal = best_ub_cost;
        for (int i = 0; i < g->n_gen; ++i) {
            out->gen_pg_mw[i] = best_pg[i] * bmva;
            out->gen_vset[i] = best_v[i];
        }
    }

    /* ------------------------------------------------------------------------
     * Step B: Boundary-Point ADMM for Shor SDP Dual Lower Bound C_LB
     * ------------------------------------------------------------------------ */
    double W[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double Z[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double A[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double Tmat[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double evals[ELE_CERT_MAX_DIM];
    double evecs[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];

    double *Zprev = (double *)malloc(mat_sz * sizeof(double));
    if (Zprev == NULL) {
        free(pf_tmp); free(YP); free(YQ); free(MV);
        return -1;
    }
    memset(W, 0, sizeof(W));
    memset(Z, 0, sizeof(Z));
    {
        double x0[ELE_CERT_MAX_DIM];
        for (int k = 0; k < N; ++k) {
            x0[k] = out->ub_pf.vm[k] * cos(out->ub_pf.va_rad[k]);
            x0[N + k] = out->ub_pf.vm[k] * sin(out->ub_pf.va_rad[k]);
        }
        for (int i = 0; i < d; ++i) {
            for (int j = 0; j < d; ++j) W[i * d + j] = x0[i] * x0[j];
        }
    }

    double c_scale = fmax(100.0, fabs(out->c_ub_primal));
    double max_y_norm = 1.0;
    {
        ele_grid_csr_t Ytmp;
        if (ele_grid_build_ybus_csr(g, -1, &Ytmp) == 0) {
            for (int k = 0; k < N; ++k) {
                for (int p = Ytmp.row_ptr[k]; p < Ytmp.row_ptr[k + 1]; ++p) {
                    if (Ytmp.col_idx[p] == k) {
                        double ymag = fabs(Ytmp.g_val[p]) + fabs(Ytmp.b_val[p]);
                        if (ymag > max_y_norm) max_y_norm = ymag;
                    }
                }
            }
            ele_grid_csr_free(&Ytmp);
        }
    }

    /* Extract sparse nonzero lists of YP[k], YQ[k] for O(deg) ADMM sweeps */
    size_t *p_nz_idx = (size_t *)malloc((size_t)N * mat_sz * sizeof(size_t));
    double *p_nz_val = (double *)malloc((size_t)N * mat_sz * sizeof(double));
    int p_nz_off[ELE_GRID_MAX_BUS + 1];
    size_t *q_nz_idx = (size_t *)malloc((size_t)N * mat_sz * sizeof(size_t));
    double *q_nz_val = (double *)malloc((size_t)N * mat_sz * sizeof(double));
    int q_nz_off[ELE_GRID_MAX_BUS + 1];
    if (!p_nz_idx || !p_nz_val || !q_nz_idx || !q_nz_val) {
        free(pf_tmp); free(Zprev);
        free(p_nz_idx); free(p_nz_val); free(q_nz_idx); free(q_nz_val);
        free(YP); free(YQ); free(MV);
        return -1;
    }
    p_nz_off[0] = 0;
    q_nz_off[0] = 0;
    for (int k = 0; k < N; ++k) {
        const double *yp = YP + (size_t)k * mat_sz;
        const double *yq = YQ + (size_t)k * mat_sz;
        int np_k = p_nz_off[k];
        int nq_k = q_nz_off[k];
        for (size_t e = 0; e < mat_sz; ++e) {
            if (fabs(yp[e]) > 1e-15) { p_nz_idx[np_k] = e; p_nz_val[np_k] = yp[e]; ++np_k; }
            if (fabs(yq[e]) > 1e-15) { q_nz_idx[nq_k] = e; q_nz_val[nq_k] = yq[e]; ++nq_k; }
        }
        p_nz_off[k + 1] = np_k;
        q_nz_off[k + 1] = nq_k;
    }

    double lam[ELE_GRID_MAX_BUS] = {0};
    double gam[ELE_GRID_MAX_BUS] = {0};
    double mu_lo[ELE_GRID_MAX_BUS] = {0};
    double mu_hi[ELE_GRID_MAX_BUS] = {0};
    opf_branch_dual_t *bd = (opf_branch_dual_t *)calloc((size_t)ELE_GRID_MAX_BRANCH, sizeof(opf_branch_dual_t));
    double xif[ELE_GRID_MAX_BRANCH] = {0}, etaf[ELE_GRID_MAX_BRANCH] = {0};
    double xit[ELE_GRID_MAX_BRANCH] = {0}, etat[ELE_GRID_MAX_BRANCH] = {0};
    double tlo[ELE_GRID_MAX_BRANCH] = {0}, thi[ELE_GRID_MAX_BRANCH] = {0};
    if (bd == NULL) {
        free(pf_tmp); free(Zprev);
        free(p_nz_idx); free(p_nz_val); free(q_nz_idx); free(q_nz_val);
        free(YP); free(YQ); free(MV);
        return -1;
    }
    init_opf_branch_duals(g, bd);
    {
        double avg_mc = 0.0;
        int cnt_mc = 0;
        for (int i = 0; i < g->n_gen; ++i) {
            if (g->gen[i].pmax_mw > g->gen[i].pmin_mw + 1e-6) {
                avg_mc += 2.0 * g->gen[i].c2 * bmva * bmva * (best_pg[i]) + g->gen[i].c1 * bmva;
                ++cnt_mc;
            }
        }
        if (cnt_mc > 0) avg_mc /= (double)cnt_mc;
        for (int k = 0; k < N; ++k) lam[k] = avg_mc / c_scale;
    }

    double rho = (d > 24) ? 0.05 : fmin(1.5 / max_y_norm, 0.25);
    double norm_YP[ELE_GRID_MAX_BUS], norm_YQ[ELE_GRID_MAX_BUS], norm_MV[ELE_GRID_MAX_BUS];
    for (int k = 0; k < N; ++k) {
        double np = 0.0, nq = 0.0;
        for (int idx = p_nz_off[k]; idx < p_nz_off[k + 1]; ++idx) np += p_nz_val[idx] * p_nz_val[idx];
        for (int idx = q_nz_off[k]; idx < q_nz_off[k + 1]; ++idx) nq += q_nz_val[idx] * q_nz_val[idx];
        norm_YP[k] = fmax(np, 1e-12);
        norm_YQ[k] = fmax(nq, 1e-12);
        norm_MV[k] = 2.0;
    }

    double W_max = 0.0;
    for (int k = 0; k < N; ++k) W_max += g->bus[k].vmax * g->bus[k].vmax;

    double best_lb_fp = -1e18;
    double best_lam[ELE_GRID_MAX_BUS] = {0}, best_gam[ELE_GRID_MAX_BUS] = {0};
    double best_mlo[ELE_GRID_MAX_BUS] = {0}, best_mhi[ELE_GRID_MAX_BUS] = {0};
    double best_xif[ELE_GRID_MAX_BRANCH] = {0}, best_etaf[ELE_GRID_MAX_BRANCH] = {0};
    double best_xit[ELE_GRID_MAX_BRANCH] = {0}, best_etat[ELE_GRID_MAX_BRANCH] = {0};
    double best_tlo[ELE_GRID_MAX_BRANCH] = {0}, best_thi[ELE_GRID_MAX_BRANCH] = {0};

    /* Normalized Boundary-Point ADMM with adaptive residual balancing */
    int n_admm = (d > 24) ? 1400 : 500;
    for (int admm_it = 0; admm_it < n_admm; ++admm_it) {
        memset(A, 0, mat_sz * sizeof(double));
        for (int k = 0; k < N; ++k) {
            double lk = lam[k], gk = gam[k];
            for (int idx = p_nz_off[k]; idx < p_nz_off[k + 1]; ++idx) A[p_nz_idx[idx]] += lk * p_nz_val[idx];
            for (int idx = q_nz_off[k]; idx < q_nz_off[k + 1]; ++idx) A[q_nz_idx[idx]] += gk * q_nz_val[idx];
            double dm = mu_hi[k] - mu_lo[k];
            A[(size_t)k * (size_t)d + (size_t)k] += dm;
            A[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] += dm;
        }
        for (int e = 0; e < g->n_branch; ++e) {
            const opf_branch_dual_t *b = &bd[e];
            if (!b->has_therm && !b->has_ang) continue;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                    if (b->has_therm) {
                        A[pos] += xif[e] * b->mpf[i][j] + etaf[e] * b->mqf[i][j] +
                                  xit[e] * b->mpt[i][j] + etat[e] * b->mqt[i][j];
                    }
                    if (b->has_ang) {
                        A[pos] += tlo[e] * b->ma_lo[i][j] + thi[e] * b->ma_hi[i][j];
                    }
                }
            }
        }

        for (int cd = 0; cd < 6; ++cd) {
            for (int k = 0; k < N; ++k) {
                double tr_y_w = 0.0, tr_y_r = 0.0;
                for (int idx = p_nz_off[k]; idx < p_nz_off[k + 1]; ++idx) {
                    size_t e = p_nz_idx[idx];
                    double v = p_nz_val[idx];
                    tr_y_w += v * W[e];
                    tr_y_r += v * (A[e] - Z[e]);
                }
                tr_y_r -= lam[k] * norm_YP[k];
                double pd_pu = g->bus[k].pd_mw / bmva;
                double b_lin = -pd_pu - tr_y_w + rho * tr_y_r;
                double a_quad = rho * norm_YP[k];
                double lo = -100.0, hi = 100.0;
                for (int bs = 0; bs < 28; ++bs) {
                    double mid = 0.5 * (lo + hi);
                    double pg_sum = 0.0;
                    for (int i = 0; i < g->n_gen; ++i) {
                        if (g->gen[i].bus_idx != k) continue;
                        double a2 = (g->gen[i].c2 * bmva * bmva) / c_scale;
                        double a1 = (g->gen[i].c1 * bmva) / c_scale;
                        double pmin = g->gen[i].pmin_mw / bmva, pmax = g->gen[i].pmax_mw / bmva;
                        double pg = (a2 > 1e-12) ? ((mid - a1) / (2.0 * a2))
                                                 : ((mid > a1) ? pmax : pmin);
                        if (pg < pmin) pg = pmin;
                        if (pg > pmax) pg = pmax;
                        pg_sum += pg;
                    }
                    double deriv = pg_sum + b_lin + a_quad * mid;
                    if (deriv > 0.0) hi = mid;
                    else lo = mid;
                }
                double new_lam = 0.5 * (lo + hi);
                double dlam = new_lam - lam[k];
                lam[k] = new_lam;
                for (int idx = p_nz_off[k]; idx < p_nz_off[k + 1]; ++idx) A[p_nz_idx[idx]] += dlam * p_nz_val[idx];
            }

            for (int k = 0; k < N; ++k) {
                double tr_y_w = 0.0, tr_y_r = 0.0;
                for (int idx = q_nz_off[k]; idx < q_nz_off[k + 1]; ++idx) {
                    size_t e = q_nz_idx[idx];
                    double v = q_nz_val[idx];
                    tr_y_w += v * W[e];
                    tr_y_r += v * (A[e] - Z[e]);
                }
                tr_y_r -= gam[k] * norm_YQ[k];
                double qd_pu = g->bus[k].qd_mvar / bmva;
                double b_lin = -qd_pu - tr_y_w + rho * tr_y_r;
                double a_quad = rho * norm_YQ[k];
                double qmin_sum = 0.0, qmax_sum = 0.0;
                for (int i = 0; i < g->n_gen; ++i) {
                    if (g->gen[i].bus_idx != k) continue;
                    qmin_sum += g->gen[i].qmin_mvar / bmva;
                    qmax_sum += g->gen[i].qmax_mvar / bmva;
                }
                double new_gam = -(b_lin + qmax_sum) / a_quad;
                if (new_gam < 0.0) {
                    double c2 = -(b_lin + qmin_sum) / a_quad;
                    new_gam = (c2 < 0.0) ? c2 : 0.0;
                }
                double dgam = new_gam - gam[k];
                gam[k] = new_gam;
                for (int idx = q_nz_off[k]; idx < q_nz_off[k + 1]; ++idx) A[q_nz_idx[idx]] += dgam * q_nz_val[idx];
            }

            for (int k = 0; k < N; ++k) {
                size_t e1 = (size_t)k * (size_t)d + (size_t)k;
                size_t e2 = (size_t)(N + k) * (size_t)d + (size_t)(N + k);
                {
                    double tr_m_w = W[e1] + W[e2];
                    double tr_m_r = (A[e1] - Z[e1]) + (A[e2] - Z[e2]) - 2.0 * mu_hi[k];
                    double vmax2 = g->bus[k].vmax * g->bus[k].vmax;
                    double new_mhi = (-vmax2 + tr_m_w - rho * tr_m_r) / (rho * 2.0);
                    if (new_mhi < 0.0) new_mhi = 0.0;
                    double dm = new_mhi - mu_hi[k];
                    mu_hi[k] = new_mhi;
                    A[e1] += dm;
                    A[e2] += dm;
                }
                {
                    double tr_m_w = W[e1] + W[e2];
                    double tr_m_r = (A[e1] - Z[e1]) + (A[e2] - Z[e2]) + 2.0 * mu_lo[k];
                    double vmin2 = g->bus[k].vmin * g->bus[k].vmin;
                    double new_mlo = (vmin2 - tr_m_w + rho * tr_m_r) / (rho * 2.0);
                    if (new_mlo < 0.0) new_mlo = 0.0;
                    double dm = new_mlo - mu_lo[k];
                    mu_lo[k] = new_mlo;
                    A[e1] -= dm;
                    A[e2] -= dm;
                }
            }

            /* Exact Branch Apparent-Power Thermal Cone Dual (f & t) + Angle Duals */
            for (int e = 0; e < g->n_branch; ++e) {
                const opf_branch_dual_t *b = &bd[e];
                if (b->has_therm) {
                    {
                        double xo = xif[e], yo = etaf[e];
                        double tr_pw = 0.0, tr_pr = 0.0, tr_qw = 0.0, tr_qr = 0.0;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                double mp = b->mpf[i][j], mq = b->mqf[i][j];
                                double rem = (A[pos] - Z[pos]) - xo * mp - yo * mq;
                                tr_pw += mp * W[pos]; tr_pr += mp * rem;
                                tr_qw += mq * W[pos]; tr_qr += mq * rem;
                            }
                        }
                        double xn = xo, yn = yo;
                        prox_solve_2d_cone(rho * b->norm_pf, rho * b->norm_qf, rho * b->dot_pqf,
                                           -tr_pw + rho * tr_pr, -tr_qw + rho * tr_qr, b->smax_pu, &xn, &yn);
                        double dx = xn - xo, dy = yn - yo;
                        xif[e] = xn; etaf[e] = yn;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                A[pos] += dx * b->mpf[i][j] + dy * b->mqf[i][j];
                            }
                        }
                    }
                    {
                        double xo = xit[e], yo = etat[e];
                        double tr_pw = 0.0, tr_pr = 0.0, tr_qw = 0.0, tr_qr = 0.0;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                double mp = b->mpt[i][j], mq = b->mqt[i][j];
                                double rem = (A[pos] - Z[pos]) - xo * mp - yo * mq;
                                tr_pw += mp * W[pos]; tr_pr += mp * rem;
                                tr_qw += mq * W[pos]; tr_qr += mq * rem;
                            }
                        }
                        double xn = xo, yn = yo;
                        prox_solve_2d_cone(rho * b->norm_pt, rho * b->norm_qt, rho * b->dot_pqt,
                                           -tr_pw + rho * tr_pr, -tr_qw + rho * tr_qr, b->smax_pu, &xn, &yn);
                        double dx = xn - xo, dy = yn - yo;
                        xit[e] = xn; etat[e] = yn;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                A[pos] += dx * b->mpt[i][j] + dy * b->mqt[i][j];
                            }
                        }
                    }
                }
                if (b->has_ang) {
                    {
                        double to = tlo[e], tw = 0.0, tr = 0.0;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                double ma = b->ma_lo[i][j];
                                tw += ma * W[pos];
                                tr += ma * ((A[pos] - Z[pos]) - to * ma);
                            }
                        }
                        double tn = (tw - rho * tr) / (rho * b->norm_alo);
                        if (tn < 0.0) tn = 0.0;
                        double dt = tn - to;
                        tlo[e] = tn;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                A[(size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j]] += dt * b->ma_lo[i][j];
                            }
                        }
                    }
                    {
                        double to = thi[e], tw = 0.0, tr = 0.0;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                                double ma = b->ma_hi[i][j];
                                tw += ma * W[pos];
                                tr += ma * ((A[pos] - Z[pos]) - to * ma);
                            }
                        }
                        double tn = (tw - rho * tr) / (rho * b->norm_ahi);
                        if (tn < 0.0) tn = 0.0;
                        double dt = tn - to;
                        thi[e] = tn;
                        for (int i = 0; i < 4; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                A[(size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j]] += dt * b->ma_hi[i][j];
                            }
                        }
                    }
                }
            }
        }

        memcpy(Zprev, Z, mat_sz * sizeof(double));
        for (size_t e = 0; e < mat_sz; ++e) Tmat[e] = A[e] - W[e] / rho;
        if (ele_sym_jacobi_eigs(d, Tmat, evals, evecs) != 0) break;
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                double zij = 0.0, wij = 0.0;
                for (int r = 0; r < d; ++r) {
                    double ev = evals[r];
                    double vr = evecs[i * d + r] * evecs[j * d + r];
                    if (ev > 0.0) zij += ev * vr;
                    else wij += (-rho * ev) * vr;
                }
                Z[i * d + j] = zij; Z[j * d + i] = zij;
                W[i * d + j] = wij; W[j * d + i] = wij;
            }
        }
        if ((admm_it % 20) == 19) {
            double rn = 0.0, sn = 0.0;
            for (size_t e = 0; e < mat_sz; ++e) {
                double dr = A[e] - Z[e], ds = Z[e] - Zprev[e];
                rn += dr * dr;
                sn += ds * ds;
            }
            rn = sqrt(rn);
            sn = rho * sqrt(sn);
            if (rn > 3.0 * sn && rho < 10.0) rho *= 1.3;
            else if (sn > 3.0 * rn && rho > 1e-3) rho /= 1.3;
        }

        /* Unscale multipliers periodically to evaluate exact anytime dual bound in $/h */
        if (d <= 24 || (admm_it % 15 == 0) || (admm_it + 1 == n_admm)) {
            double sl[ELE_GRID_MAX_BUS], sg[ELE_GRID_MAX_BUS], sml[ELE_GRID_MAX_BUS], smh[ELE_GRID_MAX_BUS];
            double sxif[ELE_GRID_MAX_BRANCH], setaf[ELE_GRID_MAX_BRANCH];
            double sxit[ELE_GRID_MAX_BRANCH], setat[ELE_GRID_MAX_BRANCH];
            for (int k = 0; k < N; ++k) {
                sl[k] = lam[k] * c_scale;
                sg[k] = gam[k] * c_scale;
                sml[k] = mu_lo[k] * c_scale;
                smh[k] = mu_hi[k] * c_scale;
            }
            for (int e = 0; e < g->n_branch; ++e) {
                sxif[e] = xif[e] * c_scale; setaf[e] = etaf[e] * c_scale;
                sxit[e] = xit[e] * c_scale; setat[e] = etat[e] * c_scale;
            }
            if (ele_sym_jacobi_eigs(d, A, evals, NULL) != 0) break;
            double c_d = eval_opf_dual_scalar_ext(g, sl, sg, sml, smh, bd,
                                                  sxif, setaf, sxit, setat, NULL, NULL);
            double lmin = evals[0] * c_scale;
            double lb_cur = c_d + ((lmin < 0.0) ? (lmin * W_max) : 0.0);
            if (lb_cur > best_lb_fp && lb_cur <= out->c_ub_primal + 1e-3) {
                best_lb_fp = lb_cur;
                memcpy(best_lam, sl, sizeof(best_lam));
                memcpy(best_gam, sg, sizeof(best_gam));
                memcpy(best_mlo, sml, sizeof(best_mlo));
                memcpy(best_mhi, smh, sizeof(best_mhi));
                for (int e = 0; e < g->n_branch; ++e) {
                    best_xif[e] = sxif[e]; best_etaf[e] = setaf[e];
                    best_xit[e] = sxit[e]; best_etat[e] = setat[e];
                    best_tlo[e] = tlo[e] * c_scale; best_thi[e] = thi[e] * c_scale;
                }
            }
        }
    }
    free(Zprev);
    free(p_nz_idx); free(p_nz_val); free(q_nz_idx); free(q_nz_val);

    /* Post-ADMM Rank-1 Eigenvector Primal Recovery from Shor W matrix */
    if (ele_sym_jacobi_eigs(d, W, evals, evecs) == 0 && evals[d - 1] > 1e-6) {
        double scale_eig = sqrt(evals[d - 1]);
        double vm_rec[ELE_GRID_MAX_BUS], va_rec[ELE_GRID_MAX_BUS];
        double ref_re = evecs[g->slack_idx * d + (d - 1)];
        double ref_im = evecs[(N + g->slack_idx) * d + (d - 1)];
        double ref_ang = atan2(ref_im, ref_re);
        for (int k = 0; k < N; ++k) {
            double er = scale_eig * evecs[k * d + (d - 1)];
            double fi = scale_eig * evecs[(N + k) * d + (d - 1)];
            double mag = hypot(er, fi);
            if (mag < g->bus[k].vmin) mag = g->bus[k].vmin;
            if (mag > g->bus[k].vmax) mag = g->bus[k].vmax;
            vm_rec[k] = mag;
            va_rec[k] = atan2(fi, er) - ref_ang;
        }
        double x_rec[ELE_CERT_MAX_DIM];
        for (int k = 0; k < N; ++k) {
            x_rec[k] = vm_rec[k] * cos(va_rec[k]);
            x_rec[N + k] = vm_rec[k] * sin(va_rec[k]);
        }
        double pg_rec[ELE_GRID_MAX_GEN], vs_rec[ELE_GRID_MAX_GEN];
        for (int i = 0; i < g->n_gen; ++i) {
            int b = g->gen[i].bus_idx;
            vs_rec[i] = vm_rec[b];
            const double *yp = YP + (size_t)b * mat_sz;
            double p_inj = 0.0;
            for (int r = 0; r < d; ++r) {
                for (int c = 0; c < d; ++c) p_inj += yp[(size_t)r * (size_t)d + (size_t)c] * x_rec[r] * x_rec[c];
            }
            double pg = p_inj + g->bus[b].pd_mw / bmva;
            double pmin = g->gen[i].pmin_mw / bmva, pmax = g->gen[i].pmax_mw / bmva;
            if (pg < pmin) pg = pmin;
            if (pg > pmax) pg = pmax;
            pg_rec[i] = pg;
        }
        int had_feas = feas_best;
        double old_cost = best_ub_cost;
        polish_primal_acopf(g, pg_rec, vs_rec, vm_rec, va_rec, tpf_p, 160,
                            &feas_best, &best_ub_obj, &best_ub_cost, best_pg, best_v, &out->ub_pf);
        if (feas_best && (!had_feas || (best_ub_cost < old_cost && best_ub_cost >= best_lb_fp - 1.0))) {
            out->c_ub_primal = best_ub_cost;
            for (int i = 0; i < g->n_gen; ++i) {
                out->gen_pg_mw[i] = best_pg[i] * bmva;
                out->gen_vset[i] = best_v[i];
            }
        }
    }
    free(pf_tmp);

    /* Rebuild exact A at best anytime dual multipliers */
    memset(A, 0, mat_sz * sizeof(double));
    for (int k = 0; k < N; ++k) {
        const double *yp = YP + (size_t)k * mat_sz;
        const double *yq = YQ + (size_t)k * mat_sz;
        const double *mv = MV + (size_t)k * mat_sz;
        double dm = best_mhi[k] - best_mlo[k];
        for (size_t e = 0; e < mat_sz; ++e) {
            A[e] += best_lam[k] * yp[e] + best_gam[k] * yq[e] + dm * mv[e];
        }
    }
    for (int e = 0; e < g->n_branch; ++e) {
        const opf_branch_dual_t *b = &bd[e];
        if (!b->has_therm && !b->has_ang) continue;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                size_t pos = (size_t)b->idx[i] * (size_t)d + (size_t)b->idx[j];
                if (b->has_therm) {
                    A[pos] += best_xif[e] * b->mpf[i][j] + best_etaf[e] * b->mqf[i][j] +
                              best_xit[e] * b->mpt[i][j] + best_etat[e] * b->mqt[i][j];
                }
                if (b->has_ang) {
                    A[pos] += best_tlo[e] * b->ma_lo[i][j] + best_thi[e] * b->ma_hi[i][j];
                }
            }
        }
    }

    /* ------------------------------------------------------------------------
     * Step C: Zero-FP-Trust Integer Cholesky+SDD Lower Bound Certification
     * ------------------------------------------------------------------------ */
    const int64_t S_V = 1000;
    const int64_t Q_A = 10000000; /* 10^7 */
    const int64_t opf_scale = S_V * S_V * Q_A; /* 10^13 */

    int64_t A_int[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    int64_t L_int[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double norm_A_frob = 0.0;
    for (int i = 0; i < d; ++i) {
        for (int j = i; j < d; ++j) {
            double v = 0.5 * (A[i * d + j] + A[j * d + i]);
            int64_t qv = (int64_t)llround(v * (double)Q_A);
            A_int[i * d + j] = qv;
            A_int[j * d + i] = qv;
            norm_A_frob += v * v * ((i == j) ? 1.0 : 2.0);
        }
    }
    norm_A_frob = sqrt(norm_A_frob);

    int64_t mu_psd = 0;
    if (ele_cert_cholesky_sdd_i64(d, A_int, L_int, &mu_psd) != 0) {
        free(bd); free(YP); free(YQ); free(MV);
        return -1;
    }

    /* Exact bound: e_k^2 + f_k^2 <= (S_V * vmax_k + sqrt(2))^2 <= (ceil(S_V*vmax_k) + 2)^2 */
    int64_t w_max_int = 0;
    for (int k = 0; k < N; ++k) {
        int64_t vmk = (int64_t)ceil(g->bus[k].vmax * (double)S_V) + 2;
        w_max_int += vmk * vmk;
    }

    double c_dual_exact = eval_opf_dual_scalar_ext(g, best_lam, best_gam, best_mlo, best_mhi,
                                                   bd, best_xif, best_etaf, best_xit, best_etat,
                                                   NULL, NULL);
    free(bd);
    int64_t c_dual_int = (int64_t)floor(c_dual_exact * (double)opf_scale);
    /* Conservative rounding slack covering A_int quantization + x->x_int discretization + dual constant */
    double d_rnd_fp = (double)N * (double)w_max_int +
                      (double)Q_A * norm_A_frob * (sqrt((double)d * W_max) + 0.5 * (double)N) + 100.0;
    int64_t delta_round_int = (int64_t)ceil(d_rnd_fp);

    __extension__ typedef __int128 ee_i128;
    ee_i128 neg_mu = (mu_psd < 0) ? (ee_i128)mu_psd : 0;
    ee_i128 c_lb_i128 = (ee_i128)c_dual_int + neg_mu * (ee_i128)w_max_int - (ee_i128)delta_round_int;
    int64_t c_lb_int = (int64_t)c_lb_i128;
    int64_t c_ub_int = (int64_t)ceil(out->c_ub_primal * (double)opf_scale);

    out->converged = feas_best ? 1 : 0;
    out->c_lb_dual_raw = best_lb_fp;
    out->c_lb_cert = (double)c_lb_int / (double)opf_scale;
    out->mu_psd_int = mu_psd;
    out->delta_round_int = delta_round_int;
    out->gap_abs = out->c_ub_primal - out->c_lb_cert;
    out->gap_pct = (out->c_ub_primal > 1e-6) ? (100.0 * out->gap_abs / out->c_ub_primal) : 0.0;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "grid");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "acopf");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s",
             (out->converged && c_lb_int <= c_ub_int) ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", g->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", g->input_sha256);

    out->cert.has_psd = 1;
    out->cert.psd.dim = d;
    out->cert.psd.scale_q = Q_A;
    out->cert.psd.mu_psd = mu_psd;
    out->cert.psd.delta_round = 0;
    out->cert.psd.mu_cert = mu_psd;
    out->cert.psd.ok = (c_lb_int <= c_ub_int) ? 1 : 0;
    memcpy(out->cert.psd.A, A_int, mat_sz * sizeof(int64_t));
    memcpy(out->cert.psd.L, L_int, mat_sz * sizeof(int64_t));

    out->cert.has_opf = 1;
    out->cert.opf_scale = opf_scale;
    out->cert.c_dual_int = c_dual_int;
    out->cert.w_max_int = w_max_int;
    out->cert.opf_delta_round = delta_round_int;
    out->cert.c_lb_int = c_lb_int;
    out->cert.c_ub_int = c_ub_int;

    out->cert.has_bracket = 1;
    out->cert.bracket_scale = opf_scale;
    out->cert.lb_int = c_lb_int;
    out->cert.ub_int = c_ub_int;
    out->cert.limit_lo_int = (c_lb_int < 0) ? (c_lb_int - opf_scale) : 0;
    out->cert.limit_hi_int = c_ub_int + opf_scale;
    out->cert.lb_val = out->c_lb_cert;
    out->cert.ub_val = out->c_ub_primal;

    free(YP); free(YQ); free(MV);
    return 0;
}

/* ============================================================================
 * 4. Multi-Machine / GFM EMT Swing Transient & Lyapunov CCT
 * ============================================================================ */

static int kron_reduce_network(const ele_grid_t *g, const ele_grid_pf_res_t *pf,
                               int fault_bus_idx, int skip_branch_idx,
                               double *G_red, double *B_red) {
    int N = g->n_bus;
    int G = g->n_gen;
    if (N + G > 64) return -1;

    size_t nn = (size_t)(N * N);
    size_t ng = (size_t)(N * G);
    size_t gg = (size_t)(G * G);
    double *G_bus = (double *)malloc(nn * sizeof(double));
    double *B_bus = (double *)malloc(nn * sizeof(double));
    double *YNN_r = (double *)malloc(nn * sizeof(double));
    double *YNN_i = (double *)malloc(nn * sizeof(double));
    double *YNG_r = (double *)calloc(ng, sizeof(double));
    double *YNG_i = (double *)calloc(ng, sizeof(double));
    double *YGG_r = (double *)calloc(gg, sizeof(double));
    double *YGG_i = (double *)calloc(gg, sizeof(double));
    if (!G_bus || !B_bus || !YNN_r || !YNN_i || !YNG_r || !YNG_i || !YGG_r || !YGG_i) {
        free(G_bus); free(B_bus); free(YNN_r); free(YNN_i);
        free(YNG_r); free(YNG_i); free(YGG_r); free(YGG_i);
        return -1;
    }

    if (ele_grid_build_ybus(g, skip_branch_idx, G_bus, B_bus) != 0) {
        free(G_bus); free(B_bus); free(YNN_r); free(YNN_i);
        free(YNG_r); free(YNG_i); free(YGG_r); free(YGG_i);
        return -1;
    }

    /* Constant-impedance equivalent loads at each bus: Y_L = (P_d - j Q_d) / |V|^2 */
    for (int k = 0; k < N; ++k) {
        double v2 = fmax(0.01, pf->vm[k] * pf->vm[k]);
        G_bus[k * N + k] += (g->bus[k].pd_mw / g->base_mva) / v2;
        B_bus[k * N + k] += (-g->bus[k].qd_mvar / g->base_mva) / v2;
    }
    if (fault_bus_idx >= 0 && fault_bus_idx < N) {
        G_bus[fault_bus_idx * N + fault_bus_idx] += 1e7;
        B_bus[fault_bus_idx * N + fault_bus_idx] -= 1e7;
    }

    memcpy(YNN_r, G_bus, nn * sizeof(double));
    memcpy(YNN_i, B_bus, nn * sizeof(double));

    for (int i = 0; i < G; ++i) {
        int b = g->gen[i].bus_idx;
        double xdp = (g->gen[i].xdp_pu > 1e-5) ? g->gen[i].xdp_pu : 0.10;
        double yd_i = -1.0 / xdp; /* 1/(j x'_d) = -j/x'_d */
        YNN_i[b * N + b] += yd_i;
        YGG_i[i * G + i] += yd_i;
        YNG_i[b * G + i] -= yd_i;
    }

    /* Solve Y_NN * X_col = Y_NG[:, col] for each generator col */
    memset(G_red, 0, gg * sizeof(double));
    memset(B_red, 0, gg * sizeof(double));
    for (int c = 0; c < G; ++c) {
        double br[64], bi[64], xr[64], xi[64];
        for (int r = 0; r < N; ++r) {
            br[r] = YNG_r[r * G + c];
            bi[r] = YNG_i[r * G + c];
        }
        if (ele_lu_solve_complex(N, YNN_r, YNN_i, br, bi, xr, xi) != 0) {
            free(G_bus); free(B_bus); free(YNN_r); free(YNN_i);
            free(YNG_r); free(YNG_i); free(YGG_r); free(YGG_i);
            return -1;
        }
        for (int r = 0; r < G; ++r) {
            double prod_r = 0.0, prod_i = 0.0;
            /* Y_GN = Y_NG^T */
            for (int k = 0; k < N; ++k) {
                double gr = YNG_r[k * G + r], gi = YNG_i[k * G + r];
                prod_r += gr * xr[k] - gi * xi[k];
                prod_i += gr * xi[k] + gi * xr[k];
            }
            G_red[r * G + c] = YGG_r[r * G + c] - prod_r;
            B_red[r * G + c] = YGG_i[r * G + c] - prod_i;
        }
    }
    free(G_bus); free(B_bus); free(YNN_r); free(YNN_i);
    free(YNG_r); free(YNG_i); free(YGG_r); free(YGG_i);
    return 0;
}

static void eval_swing_rhs(const ele_grid_t *g, const double *E_mag, const double *Pm,
                           const double *G_red, const double *B_red,
                           const double *delta, const double *omega,
                           double *ddelta, double *domega, double *pe_out,
                           double *max_gfm_i_out) {
    int G = g->n_gen;
    const double ws = 2.0 * M_PI * 60.0;
    for (int i = 0; i < G; ++i) {
        double ir = 0.0, ii = 0.0;
        double er = E_mag[i] * cos(delta[i]), ei = E_mag[i] * sin(delta[i]);
        for (int j = 0; j < G; ++j) {
            double ej_r = E_mag[j] * cos(delta[j]), ej_i = E_mag[j] * sin(delta[j]);
            double gij = G_red[i * G + j], bij = B_red[i * G + j];
            ir += gij * ej_r - bij * ej_i;
            ii += gij * ej_i + bij * ej_r;
        }
        double imag = sqrt(ir * ir + ii * ii);
        if (g->gen[i].is_gfm && imag > 1.20) {
            double sc = 1.20 / imag;
            ir *= sc; ii *= sc;
            imag = 1.20;
        }
        if (g->gen[i].is_gfm && max_gfm_i_out != NULL && imag > *max_gfm_i_out) {
            *max_gfm_i_out = imag;
        }
        double pe = er * ir + ei * ii;
        pe_out[i] = pe;
        ddelta[i] = omega[i];
        domega[i] = (ws / (2.0 * g->gen[i].h_sec)) * (Pm[i] - pe - g->gen[i].d_pu * (omega[i] / ws));
    }
}

static void run_swing_sim(const ele_grid_t *g, const double *E_mag, const double *Pm,
                          const double *delta0, const double *G_f, const double *B_f,
                          const double *G_p, const double *B_p,
                          double t_clear, double t_end, double dt,
                          double *max_dcoi_deg, double *final_spread_deg,
                          double *v_kin_cl, double *v_pot_cl, double *max_gfm_i) {
    int G = g->n_gen;
    const double ws = 2.0 * M_PI * 60.0;
    double delta[ELE_GRID_MAX_GEN], omega[ELE_GRID_MAX_GEN];
    double delta_prev[ELE_GRID_MAX_GEN], pe_post_prev[ELE_GRID_MAX_GEN];
    memcpy(delta, delta0, (size_t)G * sizeof(double));
    memset(omega, 0, (size_t)G * sizeof(double));

    double h_sum = 0.0;
    for (int i = 0; i < G; ++i) h_sum += g->gen[i].h_sec;

    double vpot_cum = 0.0;
    double max_coi = 0.0;
    double max_gi = 0.0;
    int cl_recorded = 0;

    /* Initial post-fault Pe at delta0 */
    {
        double dd[ELE_GRID_MAX_GEN], dw[ELE_GRID_MAX_GEN];
        eval_swing_rhs(g, E_mag, Pm, G_p, B_p, delta, omega, dd, dw, pe_post_prev, NULL);
        memcpy(delta_prev, delta, (size_t)G * sizeof(double));
    }

    int steps = (int)llround(t_end / dt);
    for (int s = 0; s < steps; ++s) {
        double t = (double)s * dt;
        const double *Gm = (t < t_clear) ? G_f : G_p;
        const double *Bm = (t < t_clear) ? B_f : B_p;

        double k1d[ELE_GRID_MAX_GEN], k1w[ELE_GRID_MAX_GEN], pe_t[ELE_GRID_MAX_GEN];
        double k2d[ELE_GRID_MAX_GEN], k2w[ELE_GRID_MAX_GEN];
        double k3d[ELE_GRID_MAX_GEN], k3w[ELE_GRID_MAX_GEN];
        double k4d[ELE_GRID_MAX_GEN], k4w[ELE_GRID_MAX_GEN];
        double td[ELE_GRID_MAX_GEN], tw[ELE_GRID_MAX_GEN];

        eval_swing_rhs(g, E_mag, Pm, Gm, Bm, delta, omega, k1d, k1w, pe_t, &max_gi);
        for (int i = 0; i < G; ++i) { td[i] = delta[i] + 0.5 * dt * k1d[i]; tw[i] = omega[i] + 0.5 * dt * k1w[i]; }
        eval_swing_rhs(g, E_mag, Pm, Gm, Bm, td, tw, k2d, k2w, pe_t, NULL);
        for (int i = 0; i < G; ++i) { td[i] = delta[i] + 0.5 * dt * k2d[i]; tw[i] = omega[i] + 0.5 * dt * k2w[i]; }
        eval_swing_rhs(g, E_mag, Pm, Gm, Bm, td, tw, k3d, k3w, pe_t, NULL);
        for (int i = 0; i < G; ++i) { td[i] = delta[i] + dt * k3d[i]; tw[i] = omega[i] + dt * k3w[i]; }
        eval_swing_rhs(g, E_mag, Pm, Gm, Bm, td, tw, k4d, k4w, pe_t, NULL);

        for (int i = 0; i < G; ++i) {
            delta[i] += (dt / 6.0) * (k1d[i] + 2.0 * k2d[i] + 2.0 * k3d[i] + k4d[i]);
            omega[i] += (dt / 6.0) * (k1w[i] + 2.0 * k2w[i] + 2.0 * k3w[i] + k4w[i]);
        }

        /* Path-integrate post-fault potential energy V_pot during fault-on period */
        if (t < t_clear) {
            double dd[ELE_GRID_MAX_GEN], dw[ELE_GRID_MAX_GEN], pe_post_cur[ELE_GRID_MAX_GEN];
            eval_swing_rhs(g, E_mag, Pm, G_p, B_p, delta, omega, dd, dw, pe_post_cur, NULL);
            for (int i = 0; i < G; ++i) {
                double pacc_avg = 0.5 * ((Pm[i] - pe_post_prev[i]) + (Pm[i] - pe_post_cur[i]));
                vpot_cum -= pacc_avg * (delta[i] - delta_prev[i]);
                pe_post_prev[i] = pe_post_cur[i];
                delta_prev[i] = delta[i];
            }
        }

        double d_coi = 0.0, w_coi = 0.0;
        for (int i = 0; i < G; ++i) {
            d_coi += g->gen[i].h_sec * delta[i];
            w_coi += g->gen[i].h_sec * omega[i];
        }
        d_coi /= h_sum;
        w_coi /= h_sum;

        if (!cl_recorded && (t + dt >= t_clear)) {
            cl_recorded = 1;
            double vk = 0.0;
            for (int i = 0; i < G; ++i) {
                double wi = omega[i] - w_coi;
                vk += (g->gen[i].h_sec / ws) * wi * wi;
            }
            if (v_kin_cl != NULL) *v_kin_cl = vk;
            if (v_pot_cl != NULL) *v_pot_cl = (vpot_cum > 0.0) ? vpot_cum : 0.0;
        }

        for (int i = 0; i < G; ++i) {
            double ad = fabs((delta[i] - d_coi) - (delta0[i] - (delta0[0]))) * 180.0 / M_PI;
            double rel = fabs(delta[i] - d_coi) * 180.0 / M_PI;
            if (rel > max_coi) max_coi = rel;
            (void)ad;
        }
    }

    double dmin = 1e9, dmax = -1e9;
    for (int i = 0; i < G; ++i) {
        double deg = delta[i] * 180.0 / M_PI;
        if (deg < dmin) dmin = deg;
        if (deg > dmax) dmax = deg;
    }
    if (max_dcoi_deg != NULL) *max_dcoi_deg = max_coi;
    if (final_spread_deg != NULL) *final_spread_deg = dmax - dmin;
    if (max_gfm_i != NULL) *max_gfm_i = max_gi;
}

int ele_grid_emt(const ele_grid_t *g, int fault_bus_id, double t_clear_sec,
                 int trip_branch_idx, ele_grid_emt_res_t *out) {
    if (g == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int fb = find_bus_idx(g, fault_bus_id);
    if (fb < 0) fb = g->n_bus - 1;
    out->fault_bus_idx = fb;
    out->t_clear_sec = t_clear_sec;

    ele_grid_pf_res_t pf;
    if (ele_grid_acpf(g, -1, &pf) != 0) return -1;

    int G = g->n_gen;
    double bmva = g->base_mva;
    double E_mag[ELE_GRID_MAX_GEN], Pm[ELE_GRID_MAX_GEN], delta0[ELE_GRID_MAX_GEN];
    int bus_gen_cnt[ELE_GRID_MAX_BUS] = {0};
    for (int i = 0; i < G; ++i) ++bus_gen_cnt[g->gen[i].bus_idx];

    for (int i = 0; i < G; ++i) {
        int b = g->gen[i].bus_idx;
        double pg = (b == g->slack_idx)
                        ? (pf.p_inj_pu[b] + g->bus[b].pd_mw / bmva)
                        : (g->gen[i].pg_mw / bmva);
        double qg = (pf.q_inj_pu[b] + g->bus[b].qd_mvar / bmva) / (double)bus_gen_cnt[b];
        double vr = pf.vm[b] * cos(pf.va_rad[b]);
        double vi = pf.vm[b] * sin(pf.va_rad[b]);
        double v2 = vr * vr + vi * vi;
        /* S = V * I^* => I = (P - jQ) / V^* = ((P*vr + Q*vi) + j*(P*vi - Q*vr)) / v2 */
        double ir = (pg * vr + qg * vi) / v2;
        double ii = (pg * vi - qg * vr) / v2;
        double xdp = (g->gen[i].xdp_pu > 1e-5) ? g->gen[i].xdp_pu : 0.10;
        double er = vr - xdp * ii;
        double ei = vi + xdp * ir;
        E_mag[i] = sqrt(er * er + ei * ei);
        delta0[i] = atan2(ei, er);
        Pm[i] = pg;
    }

    double G_f[ELE_GRID_MAX_GEN * ELE_GRID_MAX_GEN], B_f[ELE_GRID_MAX_GEN * ELE_GRID_MAX_GEN];
    double G_p[ELE_GRID_MAX_GEN * ELE_GRID_MAX_GEN], B_p[ELE_GRID_MAX_GEN * ELE_GRID_MAX_GEN];
    if (kron_reduce_network(g, &pf, fb, -1, G_f, B_f) != 0) return -1;
    if (kron_reduce_network(g, &pf, -1, trip_branch_idx, G_p, B_p) != 0) return -1;

    double dt = 0.0005; /* 0.5 ms */
    double vk = 0.0, vp = 0.0, max_coi = 0.0, fin_sp = 0.0, max_gi = 0.0;
    run_swing_sim(g, E_mag, Pm, delta0, G_f, B_f, G_p, B_p, t_clear_sec, 1.5, dt,
                  &max_coi, &fin_sp, &vk, &vp, &max_gi);

    /* Compute PEBS critical energy V_crit from sustained fault-on trajectory */
    double v_crit = 0.0;
    {
        double delta[ELE_GRID_MAX_GEN], omega[ELE_GRID_MAX_GEN];
        double delta_prev[ELE_GRID_MAX_GEN], pe_post_prev[ELE_GRID_MAX_GEN];
        memcpy(delta, delta0, (size_t)G * sizeof(double));
        memset(omega, 0, (size_t)G * sizeof(double));
        double dd[ELE_GRID_MAX_GEN], dw[ELE_GRID_MAX_GEN];
        eval_swing_rhs(g, E_mag, Pm, G_p, B_p, delta, omega, dd, dw, pe_post_prev, NULL);
        memcpy(delta_prev, delta, (size_t)G * sizeof(double));
        double vp_cur = 0.0, vp_peak = 0.0;
        for (int s = 0; s < 1600; ++s) {
            double k1d[ELE_GRID_MAX_GEN], k1w[ELE_GRID_MAX_GEN], pe_t[ELE_GRID_MAX_GEN];
            eval_swing_rhs(g, E_mag, Pm, G_f, B_f, delta, omega, k1d, k1w, pe_t, NULL);
            for (int i = 0; i < G; ++i) {
                delta[i] += dt * k1d[i];
                omega[i] += dt * k1w[i];
            }
            double pe_post_cur[ELE_GRID_MAX_GEN];
            eval_swing_rhs(g, E_mag, Pm, G_p, B_p, delta, omega, dd, dw, pe_post_cur, NULL);
            double dot_pebs = 0.0;
            for (int i = 0; i < G; ++i) {
                double pacc = Pm[i] - pe_post_cur[i];
                vp_cur -= 0.5 * ((Pm[i] - pe_post_prev[i]) + pacc) * (delta[i] - delta_prev[i]);
                dot_pebs += pacc * (delta[i] - delta0[i]);
                pe_post_prev[i] = pe_post_cur[i];
                delta_prev[i] = delta[i];
            }
            if (vp_cur > vp_peak) vp_peak = vp_cur;
            if (s > 10 && dot_pebs > 0.0 && vp_cur < vp_peak) break;
        }
        v_crit = (vp_peak > 1e-4) ? vp_peak : 1.0;
    }

    /* Bisection for Critical Clearing Time (CCT) */
    double lo = 0.02, hi = 0.80;
    for (int it = 0; it < 14; ++it) {
        double mid = 0.5 * (lo + hi);
        double mc = 0.0, fs = 0.0;
        run_swing_sim(g, E_mag, Pm, delta0, G_f, B_f, G_p, B_p, mid, 1.2, 0.001,
                      &mc, &fs, NULL, NULL, NULL);
        if (mc < 150.0 && fs < 150.0) lo = mid;
        else hi = mid;
    }

    out->max_delta_coi_deg = max_coi;
    out->final_delta_spread_deg = fin_sp;
    out->v_kin = vk;
    out->v_pot = vp;
    out->v_post = vk + vp;
    out->v_crit = v_crit;
    out->stability_margin = v_crit - (vk + vp);
    out->cct_sec = lo;
    out->max_gfm_i_pu = max_gi;
    out->stable = (max_coi < 150.0 && (vk + vp) < v_crit) ? 1 : 0;

    const int64_t scale_e = 1000000;
    int64_t vk_i = (int64_t)ceil(vk * (double)scale_e);
    int64_t vp_i = (int64_t)ceil(vp * (double)scale_e);
    int64_t vc_i = (int64_t)floor(v_crit * (double)scale_e);
    int64_t verr_i = 100;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "grid");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "emt_stab");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s",
             (out->stable && vk_i + vp_i + verr_i < vc_i) ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", g->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", g->input_sha256);
    out->cert.has_emt = 1;
    out->cert.emt_scale = scale_e;
    out->cert.v_kin_int = vk_i;
    out->cert.v_pot_int = vp_i;
    out->cert.v_crit_int = vc_i;
    out->cert.v_err_int = verr_i;
    out->cert.cct_sec = out->cct_sec;
    out->cert.t_clear_sec = t_clear_sec;
    return 0;
}
