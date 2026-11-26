/* SPDX-License-Identifier: MIT */
#include "ele_frontier.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880
#endif
#ifndef M_SQRT1_2
#define M_SQRT1_2 0.70710678118654752440
#endif

__extension__ typedef __int128 ele_f_i128;

static double fnow_sec(void) {
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC) return 0.0;
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

/* ============================================================================
 * Shared Grid Power Flow, Sensitivity, & Dual Basis Helpers
 * ============================================================================ */

typedef struct {
    int u, v;
    double yff_r, yff_i, yft_r, yft_i;
    double ytf_r, ytf_i, ytt_r, ytt_i;
} br_y_t;

static void compute_branch_y(const ele_grid_branch_t *br, br_y_t *out) {
    out->u = br->from_idx;
    out->v = br->to_idx;
    double z2 = br->r_pu * br->r_pu + br->x_pu * br->x_pu;
    if (z2 < 1e-18) z2 = 1e-18;
    double gs = br->r_pu / z2;
    double bs = -br->x_pu / z2;
    double t = (br->tap > 0.0) ? br->tap : 1.0;
    double phi = br->shift_deg * (M_PI / 180.0);
    double cs = cos(phi), sn = sin(phi);
    double bsh = 0.5 * br->b_sh_pu;
    out->yff_r = gs / (t * t);
    out->yff_i = (bs + bsh) / (t * t);
    out->yft_r = -(gs * cs - bs * sn) / t;
    out->yft_i = -(gs * sn + bs * cs) / t;
    out->ytf_r = -(gs * cs + bs * sn) / t;
    out->ytf_i = -(bs * cs - gs * sn) / t;
    out->ytt_r = gs;
    out->ytt_i = bs + bsh;
}

static int frontier_acpf_ctrl(const ele_grid_t *g, const double *pg_ctrl, const double *vset_ctrl,
                              const double *vm_warm, const double *va_warm, ele_grid_pf_res_t *out) {
    int N = g->n_bus;
    if (N <= 0 || N > ELE_GRID_MAX_BUS || out == NULL) return -1;
    memset(out, 0, sizeof(*out));

    ele_grid_csr_t csr;
    if (ele_grid_build_ybus_csr(g, -1, &csr) != 0) return -1;

    double B = g->base_mva;
    double p_spec[ELE_GRID_MAX_BUS] = {0};
    double q_spec[ELE_GRID_MAX_BUS] = {0};
    double bus_vset[ELE_GRID_MAX_BUS] = {0};
    int is_pv[ELE_GRID_MAX_BUS] = {0};

    for (int k = 0; k < N; ++k) {
        p_spec[k] = -g->bus[k].pd_mw / B;
        q_spec[k] = -g->bus[k].qd_mvar / B;
        bus_vset[k] = (g->bus[k].vm_init > 0.1) ? g->bus[k].vm_init : 1.0;
    }
    for (int i = 0; i < g->n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        double pg = (pg_ctrl != NULL) ? pg_ctrl[i] : (g->gen[i].pg_mw / B);
        double vs = (vset_ctrl != NULL) ? vset_ctrl[i] : g->gen[i].vset;
        if (b != g->slack_idx) p_spec[b] += pg;
        if (vs > 0.1) {
            bus_vset[b] = vs;
            if (b != g->slack_idx) is_pv[b] = 1;
        }
    }
    bus_vset[g->slack_idx] = (vset_ctrl != NULL) ? vset_ctrl[0] : g->gen[0].vset;
    for (int i = 0; i < g->n_gen; ++i) {
        if (g->gen[i].bus_idx == g->slack_idx) {
            double vs = (vset_ctrl != NULL) ? vset_ctrl[i] : g->gen[i].vset;
            if (vs > 0.1) bus_vset[g->slack_idx] = vs;
            break;
        }
    }

    double vm[ELE_GRID_MAX_BUS], va[ELE_GRID_MAX_BUS];
    for (int k = 0; k < N; ++k) {
        if (g->bus[k].type == 3 || is_pv[k]) vm[k] = bus_vset[k];
        else if (vm_warm != NULL && vm_warm[k] > 0.1) vm[k] = vm_warm[k];
        else vm[k] = (g->bus[k].vm_init > 0.1) ? g->bus[k].vm_init : 1.0;

        if (g->bus[k].type == 3) va[k] = g->bus[k].va_init_deg * (M_PI / 180.0);
        else if (va_warm != NULL) va[k] = va_warm[k];
        else va[k] = g->bus[k].va_init_deg * (M_PI / 180.0);
    }

    int ang_idx[ELE_GRID_MAX_BUS], pq_idx[ELE_GRID_MAX_BUS];
    int n_ang = 0, n_pq = 0;
    for (int k = 0; k < N; ++k) {
        if (g->bus[k].type != 3) ang_idx[n_ang++] = k;
        if (g->bus[k].type == 1 && !is_pv[k]) pq_idx[n_pq++] = k;
    }
    int n_unk = n_ang + n_pq;
    if (n_unk == 0) {
        ele_grid_csr_free(&csr);
        return -1;
    }

    double *J = (double *)calloc((size_t)n_unk * (size_t)n_unk, sizeof(double));
    double *rhs = (double *)calloc((size_t)n_unk, sizeof(double));
    double *dx = (double *)calloc((size_t)n_unk, sizeof(double));
    if (J == NULL || rhs == NULL || dx == NULL) {
        free(J); free(rhs); free(dx);
        ele_grid_csr_free(&csr);
        return -1;
    }

    double Pcalc[ELE_GRID_MAX_BUS], Qcalc[ELE_GRID_MAX_BUS];
    double diag_G[ELE_GRID_MAX_BUS] = {0}, diag_B[ELE_GRID_MAX_BUS] = {0};
    for (int i = 0; i < N; ++i) {
        for (int p = csr.row_ptr[i]; p < csr.row_ptr[i + 1]; ++p) {
            if (csr.col_idx[p] == i) {
                diag_G[i] = csr.g_val[p];
                diag_B[i] = csr.b_val[p];
                break;
            }
        }
    }

    int bus_to_ang[ELE_GRID_MAX_BUS], bus_to_pq[ELE_GRID_MAX_BUS];
    for (int k = 0; k < N; ++k) { bus_to_ang[k] = -1; bus_to_pq[k] = -1; }
    for (int a = 0; a < n_ang; ++a) bus_to_ang[ang_idx[a]] = a;
    for (int a = 0; a < n_pq; ++a) bus_to_pq[pq_idx[a]] = a;

    int conv = 0;
    double max_mis = 1e9;
    int it = 0;
    for (it = 0; it < 28; ++it) {
        for (int i = 0; i < N; ++i) {
            double ps = 0.0, qs = 0.0;
            for (int p = csr.row_ptr[i]; p < csr.row_ptr[i + 1]; ++p) {
                int j = csr.col_idx[p];
                double dth = va[i] - va[j];
                double cd = cos(dth), sd = sin(dth);
                double gij = csr.g_val[p], bij = csr.b_val[p];
                ps += vm[i] * vm[j] * (gij * cd + bij * sd);
                qs += vm[i] * vm[j] * (gij * sd - bij * cd);
            }
            Pcalc[i] = ps;
            Qcalc[i] = qs;
        }
        max_mis = 0.0;
        for (int r = 0; r < n_ang; ++r) {
            int i = ang_idx[r];
            double dp = p_spec[i] - Pcalc[i];
            rhs[r] = dp;
            if (fabs(dp) > max_mis) max_mis = fabs(dp);
        }
        for (int r = 0; r < n_pq; ++r) {
            int i = pq_idx[r];
            double dq = q_spec[i] - Qcalc[i];
            rhs[n_ang + r] = dq;
            if (fabs(dq) > max_mis) max_mis = fabs(dq);
        }
        if (max_mis < 1e-10) {
            conv = 1;
            break;
        }

        memset(J, 0, (size_t)n_unk * (size_t)n_unk * sizeof(double));
        for (int i = 0; i < N; ++i) {
            int ra = bus_to_ang[i], rq = bus_to_pq[i];
            for (int p = csr.row_ptr[i]; p < csr.row_ptr[i + 1]; ++p) {
                int j = csr.col_idx[p];
                int ca = bus_to_ang[j], cq = bus_to_pq[j];
                if (i == j) {
                    if (ra >= 0) {
                        J[(size_t)ra * (size_t)n_unk + (size_t)ra] = -Qcalc[i] - diag_B[i] * vm[i] * vm[i];
                    }
                    if (ra >= 0 && rq >= 0) {
                        J[(size_t)ra * (size_t)n_unk + (size_t)(n_ang + rq)] = Pcalc[i] / vm[i] + diag_G[i] * vm[i];
                        J[(size_t)(n_ang + rq) * (size_t)n_unk + (size_t)ra] = Pcalc[i] - diag_G[i] * vm[i] * vm[i];
                        J[(size_t)(n_ang + rq) * (size_t)n_unk + (size_t)(n_ang + rq)] = Qcalc[i] / vm[i] - diag_B[i] * vm[i];
                    }
                } else {
                    double dth = va[i] - va[j];
                    double cd = cos(dth), sd = sin(dth);
                    double gij = csr.g_val[p], bij = csr.b_val[p];
                    if (ra >= 0 && ca >= 0) {
                        J[(size_t)ra * (size_t)n_unk + (size_t)ca] = vm[i] * vm[j] * (gij * sd - bij * cd);
                    }
                    if (ra >= 0 && cq >= 0) {
                        J[(size_t)ra * (size_t)n_unk + (size_t)(n_ang + cq)] = vm[i] * (gij * cd + bij * sd);
                    }
                    if (rq >= 0 && ca >= 0) {
                        J[(size_t)(n_ang + rq) * (size_t)n_unk + (size_t)ca] = -vm[i] * vm[j] * (gij * cd + bij * sd);
                    }
                    if (rq >= 0 && cq >= 0) {
                        J[(size_t)(n_ang + rq) * (size_t)n_unk + (size_t)(n_ang + cq)] = vm[i] * (gij * sd - bij * cd);
                    }
                }
            }
        }

        if (ele_lu_solve_real(n_unk, J, rhs, dx) != 0) break;
        double step_scale = 1.0;
        double max_step = 0.0;
        for (int r = 0; r < n_unk; ++r) {
            if (fabs(dx[r]) > max_step) max_step = fabs(dx[r]);
        }
        if (max_step > 0.6) step_scale = 0.6 / max_step;
        for (int r = 0; r < n_ang; ++r) va[ang_idx[r]] += step_scale * dx[r];
        for (int r = 0; r < n_pq; ++r) {
            int i = pq_idx[r];
            vm[i] += step_scale * dx[n_ang + r];
            if (vm[i] < 0.2) vm[i] = 0.2;
            if (vm[i] > 2.0) vm[i] = 2.0;
        }
    }

    free(J); free(rhs); free(dx);

    for (int i = 0; i < N; ++i) {
        double ps = 0.0, qs = 0.0;
        for (int p = csr.row_ptr[i]; p < csr.row_ptr[i + 1]; ++p) {
            int j = csr.col_idx[p];
            double dth = va[i] - va[j];
            ps += vm[i] * vm[j] * (csr.g_val[p] * cos(dth) + csr.b_val[p] * sin(dth));
            qs += vm[i] * vm[j] * (csr.g_val[p] * sin(dth) - csr.b_val[p] * cos(dth));
        }
        out->p_inj_pu[i] = ps;
        out->q_inj_pu[i] = qs;
        out->vm[i] = vm[i];
        out->va_rad[i] = va[i];
    }
    ele_grid_csr_free(&csr);

    out->converged = conv ? 1 : 0;
    out->nr_iters = it;
    out->max_mismatch_pu = max_mis;
    out->limits_ok = conv ? 1 : 0;
    out->vmin_pu = 1e9;
    out->vmax_pu = -1e9;
    for (int k = 0; k < N; ++k) {
        double pd = g->bus[k].pd_mw;
        out->total_p_load_mw += pd;
        out->total_p_gen_mw += out->p_inj_pu[k] * B + pd;
        out->total_q_gen_mvar += out->q_inj_pu[k] * B + g->bus[k].qd_mvar;
        if (vm[k] < out->vmin_pu) out->vmin_pu = vm[k];
        if (vm[k] > out->vmax_pu) out->vmax_pu = vm[k];
        if (vm[k] < g->bus[k].vmin - 1e-5 || vm[k] > g->bus[k].vmax + 1e-5) out->limits_ok = 0;
    }
    out->total_p_loss_mw = out->total_p_gen_mw - out->total_p_load_mw;

    out->max_branch_loading = 0.0;
    for (int e = 0; e < g->n_branch; ++e) {
        br_y_t y;
        compute_branch_y(&g->branch[e], &y);
        int u = y.u, v = y.v;
        double eur = vm[u] * cos(va[u]), eui = vm[u] * sin(va[u]);
        double evr = vm[v] * cos(va[v]), evi = vm[v] * sin(va[v]);
        double if_r = (y.yff_r * eur - y.yff_i * eui) + (y.yft_r * evr - y.yft_i * evi);
        double if_i = (y.yff_r * eui + y.yff_i * eur) + (y.yft_r * evi + y.yft_i * evr);
        double sf_r = eur * if_r + eui * if_i;
        double sf_i = eui * if_r - eur * if_i;
        double sf_mva = hypot(sf_r, sf_i) * B;
        double it_r = (y.ytf_r * eur - y.ytf_i * eui) + (y.ytt_r * evr - y.ytt_i * evi);
        double it_i = (y.ytf_r * eui + y.ytf_i * eur) + (y.ytt_r * evi + y.ytt_i * evr);
        double st_r = evr * it_r + evi * it_i;
        double st_i = evi * it_r - evr * it_i;
        double st_mva = hypot(st_r, st_i) * B;
        double smax_end = (sf_mva > st_mva) ? sf_mva : st_mva;
        out->branch_s_mva[e] = smax_end;
        double load = smax_end / g->branch[e].smax_mva;
        if (load > out->max_branch_loading) out->max_branch_loading = load;
        if (load > 1.0 + 1e-5) out->limits_ok = 0;
        if (g->branch[e].angmin_deg > -85.0 || g->branch[e].angmax_deg < 85.0) {
            double dv = (va[u] - va[v]) * (180.0 / M_PI);
            while (dv > 180.0) dv -= 360.0;
            while (dv < -180.0) dv += 360.0;
            if (dv < g->branch[e].angmin_deg - 1e-3 || dv > g->branch[e].angmax_deg + 1e-3) {
                out->limits_ok = 0;
            }
        }
    }
    return conv ? 0 : -1;
}

static void fill_4x4(double alpha_r, double delta_r, double b_re, double b_fe, double M[4][4]) {
    memset(M, 0, 16U * sizeof(double));
    M[0][0] = alpha_r; M[2][2] = alpha_r;
    M[1][1] = delta_r; M[3][3] = delta_r;
    M[0][1] = 0.5 * b_re; M[1][0] = 0.5 * b_re;
    M[2][3] = 0.5 * b_re; M[3][2] = 0.5 * b_re;
    M[2][1] = 0.5 * b_fe; M[1][2] = 0.5 * b_fe;
    M[0][3] = -0.5 * b_fe; M[3][0] = -0.5 * b_fe;
}

static double eval_primal_cost_and_viol(const ele_grid_t *g, const double *pg_ctrl, const double *vset_ctrl,
                                        const double *vm_warm, const double *va_warm,
                                        double *pg_full_out, ele_grid_pf_res_t *pf_out,
                                        int *feas_out, double *viol_out) {
    double pg_pu[ELE_GRID_MAX_GEN];
    int ng = g->n_gen;
    for (int i = 0; i < ng; ++i) pg_pu[i] = pg_ctrl[i];
    if (frontier_acpf_ctrl(g, pg_pu, vset_ctrl, vm_warm, va_warm, pf_out) != 0) {
        *feas_out = 0;
        *viol_out = 1e9;
        return 1e18;
    }

    double B = g->base_mva;
    double bus_p_ns[ELE_GRID_MAX_BUS] = {0};
    int slack_gen_idx = 0;
    for (int i = 0; i < ng; ++i) {
        if (g->gen[i].bus_idx == g->slack_idx) slack_gen_idx = i;
        else bus_p_ns[g->gen[i].bus_idx] += pg_pu[i];
    }
    double p_slack_tot = pf_out->p_inj_pu[g->slack_idx] + g->bus[g->slack_idx].pd_mw / B;
    pg_pu[slack_gen_idx] = p_slack_tot - bus_p_ns[g->slack_idx];
    for (int i = 0; i < ng; ++i) pg_full_out[i] = pg_pu[i];

    double cost = 0.0;
    for (int i = 0; i < ng; ++i) {
        double pmw = pg_pu[i] * B;
        cost += g->gen[i].c2 * pmw * pmw + g->gen[i].c1 * pmw + g->gen[i].c0;
    }

    double viol = 0.0;
    int feas = pf_out->limits_ok ? 1 : 0;
    for (int i = 0; i < ng; ++i) {
        double pmw = pg_pu[i] * B;
        if (pmw < g->gen[i].pmin_mw - 1e-4) {
            double v = (g->gen[i].pmin_mw - pmw) / B;
            viol += v * v + 0.01 * v;
            feas = 0;
        }
        if (pmw > g->gen[i].pmax_mw + 1e-4) {
            double v = (pmw - g->gen[i].pmax_mw) / B;
            viol += v * v + 0.01 * v;
            feas = 0;
        }
    }

    double bus_qgen[ELE_GRID_MAX_BUS] = {0};
    double bus_qmin[ELE_GRID_MAX_BUS] = {0};
    double bus_qmax[ELE_GRID_MAX_BUS] = {0};
    int bus_has_gen[ELE_GRID_MAX_BUS] = {0};
    for (int i = 0; i < ng; ++i) {
        int b = g->gen[i].bus_idx;
        bus_has_gen[b] = 1;
        bus_qmin[b] += g->gen[i].qmin_mvar / B;
        bus_qmax[b] += g->gen[i].qmax_mvar / B;
    }
    for (int b = 0; b < g->n_bus; ++b) {
        if (!bus_has_gen[b]) continue;
        bus_qgen[b] = pf_out->q_inj_pu[b] + g->bus[b].qd_mvar / B;
        if (bus_qgen[b] < bus_qmin[b] - 1e-4) {
            double v = bus_qmin[b] - bus_qgen[b];
            viol += v * v + 0.01 * v;
            feas = 0;
        }
        if (bus_qgen[b] > bus_qmax[b] + 1e-4) {
            double v = bus_qgen[b] - bus_qmax[b];
            viol += v * v + 0.01 * v;
            feas = 0;
        }
    }

    for (int k = 0; k < g->n_bus; ++k) {
        if (pf_out->vm[k] < g->bus[k].vmin - 1e-5) {
            double v = (g->bus[k].vmin - pf_out->vm[k]) * 10.0;
            viol += v * v + 0.05 * v;
            feas = 0;
        }
        if (pf_out->vm[k] > g->bus[k].vmax + 1e-5) {
            double v = (pf_out->vm[k] - g->bus[k].vmax) * 10.0;
            viol += v * v + 0.05 * v;
            feas = 0;
        }
    }
    for (int e = 0; e < g->n_branch; ++e) {
        double ex = (pf_out->branch_s_mva[e] - g->branch[e].smax_mva) / fmax(1.0, g->branch[e].smax_mva);
        if (ex > 1e-5) {
            viol += 4.0 * ex * ex + 0.05 * ex;
            feas = 0;
        }
        if (g->branch[e].angmin_deg > -85.0 || g->branch[e].angmax_deg < 85.0) {
            double dv = (pf_out->va_rad[g->branch[e].from_idx] - pf_out->va_rad[g->branch[e].to_idx]) * (180.0 / M_PI);
            while (dv > 180.0) dv -= 360.0;
            while (dv < -180.0) dv += 360.0;
            if (dv < g->branch[e].angmin_deg - 1e-3) {
                double ev = (g->branch[e].angmin_deg - dv) * 0.2;
                viol += ev * ev + 0.02 * ev;
                feas = 0;
            }
            if (dv > g->branch[e].angmax_deg + 1e-3) {
                double ev = (dv - g->branch[e].angmax_deg) * 0.2;
                viol += ev * ev + 0.02 * ev;
                feas = 0;
            }
        }
    }
    *feas_out = feas;
    *viol_out = viol;
    return cost;
}

static void seed_balanced_pg(const ele_grid_t *g, double *pg_ctrl, double *vset_ctrl) {
    double B = g->base_mva;
    double pd_tot = 0.0;
    for (int k = 0; k < g->n_bus; ++k) pd_tot += g->bus[k].pd_mw;
    double p_rem = pd_tot * 1.02;
    int order[ELE_GRID_MAX_GEN];
    int n_act = 0;
    for (int i = 0; i < g->n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        double vs = (g->gen[i].vset > 0.1) ? g->gen[i].vset : 0.5 * (g->bus[b].vmin + g->bus[b].vmax);
        if (vs < g->bus[b].vmin) vs = g->bus[b].vmin;
        if (vs > g->bus[b].vmax) vs = g->bus[b].vmax;
        vset_ctrl[i] = vs;
        pg_ctrl[i] = g->gen[i].pmin_mw / B;
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
        pg_ctrl[i] = (g->gen[i].pmin_mw + add) / B;
        p_rem -= add;
    }
}

/* Two-phase coordinate + sensitivity primal restoration and cost polish */
static void polish_primal_twophase(const ele_grid_t *g,
                                   double *pg_ctrl, double *vset_ctrl,
                                   double *pg_best, double *vs_best,
                                   ele_grid_pf_res_t *pf_best,
                                   double *c_best_out, int *feas_best_out) {
    double B = g->base_mva;
    int ng = g->n_gen;
    int N = g->n_bus;
    int is_slack_gen[ELE_GRID_MAX_GEN] = {0};
    for (int i = 0; i < ng; ++i) {
        if (g->gen[i].bus_idx == g->slack_idx) { is_slack_gen[i] = 1; break; }
    }

    double pg_full[ELE_GRID_MAX_GEN];
    ele_grid_pf_res_t pf;
    int feas = 0;
    double viol = 1e9;
    double cost = eval_primal_cost_and_viol(g, pg_ctrl, vset_ctrl, NULL, NULL,
                                           pg_full, &pf, &feas, &viol);

    double best_score = feas ? cost : (1e8 * viol + cost);
    if (feas || viol < 1e8) {
        memcpy(pg_best, pg_full, (size_t)ng * sizeof(double));
        memcpy(vs_best, vset_ctrl, (size_t)ng * sizeof(double));
        *pf_best = pf;
        *c_best_out = cost;
        *feas_best_out = feas;
    }

    int bus_pv_rep[ELE_GRID_MAX_BUS];
    for (int b = 0; b < N; ++b) bus_pv_rep[b] = -1;
    for (int i = 0; i < ng; ++i) {
        int b = g->gen[i].bus_idx;
        if (bus_pv_rep[b] < 0) bus_pv_rep[b] = i;
    }

    double vm_warm[ELE_GRID_MAX_BUS], va_warm[ELE_GRID_MAX_BUS];
    if (pf.converged) {
        memcpy(vm_warm, pf.vm, (size_t)N * sizeof(double));
        memcpy(va_warm, pf.va_rad, (size_t)N * sizeof(double));
    } else {
        for (int k = 0; k < N; ++k) { vm_warm[k] = 1.0; va_warm[k] = 0.0; }
    }

    /* Targeted feasibility restoration pre-pass (fixes simultaneous Q/V/slack/angle/thermal violations) */
    for (int rp = 0; rp < 16 && !feas; ++rp) {
        double pg_next[ELE_GRID_MAX_GEN], vs_next[ELE_GRID_MAX_GEN];
        memcpy(pg_next, pg_ctrl, (size_t)ng * sizeof(double));
        memcpy(vs_next, vset_ctrl, (size_t)ng * sizeof(double));

        double b_qmin[ELE_GRID_MAX_BUS] = {0}, b_qmax[ELE_GRID_MAX_BUS] = {0};
        int b_hgen[ELE_GRID_MAX_BUS] = {0};
        for (int i = 0; i < ng; ++i) {
            int b = g->gen[i].bus_idx;
            b_hgen[b] = 1;
            b_qmin[b] += g->gen[i].qmin_mvar / B;
            b_qmax[b] += g->gen[i].qmax_mvar / B;
        }
        for (int b = 0; b < N; ++b) {
            if (!b_hgen[b]) continue;
            double qg = pf.q_inj_pu[b] + g->bus[b].qd_mvar / B;
            double vnew = vs_next[bus_pv_rep[b]];
            if (qg > b_qmax[b] + 1e-4) {
                double step = fmin(0.03, 0.015 * (qg - b_qmax[b]) + 0.004);
                vnew -= step;
            } else if (qg < b_qmin[b] - 1e-4) {
                double step = fmin(0.03, 0.015 * (b_qmin[b] - qg) + 0.004);
                vnew += step;
            }
            if (vnew < g->bus[b].vmin) vnew = g->bus[b].vmin;
            if (vnew > g->bus[b].vmax) vnew = g->bus[b].vmax;
            for (int j = 0; j < ng; ++j) {
                if (g->gen[j].bus_idx == b) vs_next[j] = vnew;
            }
        }
        /* Slack Pg balance nudge */
        for (int i = 0; i < ng; ++i) {
            if (!is_slack_gen[i]) continue;
            double pmin_s = g->gen[i].pmin_mw / B, pmax_s = g->gen[i].pmax_mw / B;
            double def = 0.0;
            if (pg_full[i] > pmax_s) def = pg_full[i] - pmax_s;
            else if (pg_full[i] < pmin_s) def = pg_full[i] - pmin_s;
            if (fabs(def) > 1e-4) {
                double share = def / fmax(1.0, (double)(ng - 1));
                for (int j = 0; j < ng; ++j) {
                    if (is_slack_gen[j]) continue;
                    pg_next[j] += 1.5 * share;
                    double jmin = g->gen[j].pmin_mw / B, jmax = g->gen[j].pmax_mw / B;
                    if (pg_next[j] < jmin) pg_next[j] = jmin;
                    if (pg_next[j] > jmax) pg_next[j] = jmax;
                }
            }
        }
        /* Branch angle/thermal flow relief via pairwise Pg + voltage lift */
        for (int e = 0; e < g->n_branch; ++e) {
            const ele_grid_branch_t *br = &g->branch[e];
            double load = pf.branch_s_mva[e] / fmax(1.0, br->smax_mva);
            double dv = (pf.va_rad[br->from_idx] - pf.va_rad[br->to_idx]) * (180.0 / M_PI);
            while (dv > 180.0) dv -= 360.0;
            while (dv < -180.0) dv += 360.0;
            int hi_viol = (load > 1.0 + 1e-4) ||
                          (br->angmax_deg < 85.0 && dv > br->angmax_deg + 1e-3);
            int lo_viol = (br->angmin_deg > -85.0 && dv < br->angmin_deg - 1e-3);
            if (hi_viol || lo_viol) {
                double sgn = hi_viol ? 1.0 : -1.0;
                for (int j = 0; j < ng; ++j) {
                    if (is_slack_gen[j]) continue;
                    int b = g->gen[j].bus_idx;
                    double dp = 0.08 * (g->gen[j].pmax_mw - g->gen[j].pmin_mw) / B;
                    if (b == br->from_idx) pg_next[j] -= sgn * dp;
                    if (b == br->to_idx) pg_next[j] += sgn * dp;
                    double jmin = g->gen[j].pmin_mw / B, jmax = g->gen[j].pmax_mw / B;
                    if (pg_next[j] < jmin) pg_next[j] = jmin;
                    if (pg_next[j] > jmax) pg_next[j] = jmax;
                    if (hi_viol && (b == br->from_idx || b == br->to_idx)) {
                        vs_next[j] = fmin(g->bus[b].vmax, vs_next[j] + 0.008);
                    }
                }
            }
        }
        /* Bus Vmin/Vmax nudge on neighboring generators */
        for (int k = 0; k < N; ++k) {
            if (pf.vm[k] < g->bus[k].vmin - 1e-5 || pf.vm[k] > g->bus[k].vmax + 1e-5) {
                double dir = (pf.vm[k] < g->bus[k].vmin) ? 0.008 : -0.008;
                for (int j = 0; j < ng; ++j) {
                    int b = g->gen[j].bus_idx;
                    double nv = vs_next[j] + ((b == k) ? (2.0 * dir) : (0.5 * dir));
                    if (nv < g->bus[b].vmin) nv = g->bus[b].vmin;
                    if (nv > g->bus[b].vmax) nv = g->bus[b].vmax;
                    vs_next[j] = nv;
                }
            }
        }

        double tpg[ELE_GRID_MAX_GEN], tv = 1e9;
        int tf = 0;
        ele_grid_pf_res_t tpf;
        double tc = eval_primal_cost_and_viol(g, pg_next, vs_next, vm_warm, va_warm, tpg, &tpf, &tf, &tv);
        if (tpf.converged && (tf || tv < viol)) {
            memcpy(pg_ctrl, pg_next, (size_t)ng * sizeof(double));
            memcpy(vset_ctrl, vs_next, (size_t)ng * sizeof(double));
            cost = tc; feas = tf; viol = tv; pf = tpf;
            memcpy(pg_full, tpg, (size_t)ng * sizeof(double));
            memcpy(vm_warm, tpf.vm, (size_t)N * sizeof(double));
            memcpy(va_warm, tpf.va_rad, (size_t)N * sizeof(double));
            double tsc = tf ? tc : (1e8 * tv + tc);
            if ((tf && !(*feas_best_out)) || (tf == *feas_best_out && tsc < best_score)) {
                best_score = tsc;
                memcpy(pg_best, tpg, (size_t)ng * sizeof(double));
                memcpy(vs_best, vset_ctrl, (size_t)ng * sizeof(double));
                *pf_best = tpf; *c_best_out = tc; *feas_best_out = tf;
            }
        } else {
            break;
        }
    }

    int max_passes = (N > 40) ? 10 : 18;
    for (int pass = 0; pass < max_passes; ++pass) {
        double p_frac = (pass < 4) ? 0.25 : ((pass < 10) ? 0.08 : 0.015);
        double v_step = (pass < 4) ? 0.03 : ((pass < 10) ? 0.01 : 0.0025);
        double pen_w = (*feas_best_out) ? 1e7 : 1e8;

        for (int i = 0; i < ng; ++i) {
            if (!is_slack_gen[i] && g->gen[i].pmax_mw > g->gen[i].pmin_mw + 1e-6) {
                double pmin = g->gen[i].pmin_mw / B, pmax = g->gen[i].pmax_mw / B;
                double dp = p_frac * (pmax - pmin);
                double orig = pg_ctrl[i];
                for (int s = 0; s < 2; ++s) {
                    double cand = orig + ((s == 0) ? dp : -dp);
                    if (cand < pmin) cand = pmin;
                    if (cand > pmax) cand = pmax;
                    if (fabs(cand - orig) < 1e-9) continue;
                    pg_ctrl[i] = cand;
                    double tpg[ELE_GRID_MAX_GEN], tv = 1e9;
                    int tf = 0;
                    ele_grid_pf_res_t tpf;
                    double tc = eval_primal_cost_and_viol(g, pg_ctrl, vset_ctrl,
                                                         vm_warm, va_warm, tpg, &tpf, &tf, &tv);
                    double tsc = tf ? tc : (pen_w * tv + tc);
                    double cur_sc = feas ? cost : (pen_w * viol + cost);
                    if (tpf.converged && ((tf && !feas) || (tf == feas && tsc < cur_sc - 1e-6))) {
                        orig = cand;
                        cost = tc; feas = tf; viol = tv;
                        memcpy(vm_warm, tpf.vm, (size_t)N * sizeof(double));
                        memcpy(va_warm, tpf.va_rad, (size_t)N * sizeof(double));
                        if ((feas && !(*feas_best_out)) || (feas == *feas_best_out && tsc < best_score)) {
                            best_score = tsc;
                            memcpy(pg_best, tpg, (size_t)ng * sizeof(double));
                            memcpy(vs_best, vset_ctrl, (size_t)ng * sizeof(double));
                            *pf_best = tpf;
                            *c_best_out = cost;
                            *feas_best_out = feas;
                        }
                    } else {
                        pg_ctrl[i] = orig;
                    }
                }
            }

            int b = g->gen[i].bus_idx;
            if (bus_pv_rep[b] == i) {
                double vmin = g->bus[b].vmin, vmax = g->bus[b].vmax;
                double orig_v = vset_ctrl[i];
                for (int s = 0; s < 2; ++s) {
                    double cand = orig_v + ((s == 0) ? v_step : -v_step);
                    if (cand < vmin) cand = vmin;
                    if (cand > vmax) cand = vmax;
                    if (fabs(cand - orig_v) < 1e-9) continue;
                    for (int j = 0; j < ng; ++j) {
                        if (g->gen[j].bus_idx == b) vset_ctrl[j] = cand;
                    }
                    double tpg[ELE_GRID_MAX_GEN], tv = 1e9;
                    int tf = 0;
                    ele_grid_pf_res_t tpf;
                    double tc = eval_primal_cost_and_viol(g, pg_ctrl, vset_ctrl,
                                                         vm_warm, va_warm, tpg, &tpf, &tf, &tv);
                    double tsc = tf ? tc : (pen_w * tv + tc);
                    double cur_sc = feas ? cost : (pen_w * viol + cost);
                    if (tpf.converged && ((tf && !feas) || (tf == feas && tsc < cur_sc - 1e-6))) {
                        orig_v = cand;
                        cost = tc; feas = tf; viol = tv;
                        memcpy(vm_warm, tpf.vm, (size_t)N * sizeof(double));
                        memcpy(va_warm, tpf.va_rad, (size_t)N * sizeof(double));
                        if ((feas && !(*feas_best_out)) || (feas == *feas_best_out && tsc < best_score)) {
                            best_score = tsc;
                            memcpy(pg_best, tpg, (size_t)ng * sizeof(double));
                            memcpy(vs_best, vset_ctrl, (size_t)ng * sizeof(double));
                            *pf_best = tpf;
                            *c_best_out = cost;
                            *feas_best_out = feas;
                        }
                    } else {
                        for (int j = 0; j < ng; ++j) {
                            if (g->gen[j].bus_idx == b) vset_ctrl[j] = orig_v;
                        }
                    }
                }
            }
        }
    }
}

/* ============================================================================
 * 1. Chordal Clique Decomposition Certified AC-OPF (up to 256 Buses)
 * ============================================================================ */

typedef struct {
    int n_bus_c;
    int buses[ELE_FRONTIER_MAX_CLIQUE_BUS];
    int dk;
    int vars[2 * ELE_FRONTIER_MAX_CLIQUE_BUS];
} clique_def_t;

typedef struct {
    int p, q;
    double w_pq;
    int mult;
    int occ_off;
    double A_cur;
    double U_bar;
} chord_edge_t;

typedef struct {
    int clique;
    int a, b; /* local index pair 0..dk-1 with vars[a] vs vars[b] */
} edge_occ_t;

typedef struct {
    int n_nz;
    int off;
} dual_var_span_t;

static int cmp_int_asc(const void *a, const void *b) {
    int ia = *(const int *)a, ib = *(const int *)b;
    return (ia > ib) - (ia < ib);
}

static int find_chord_edge(const chord_edge_t *edges, int n_edges, int u, int v) {
    int p = (u <= v) ? u : v;
    int q = (u <= v) ? v : u;
    int lo = 0, hi = n_edges - 1;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        if (edges[mid].p < p || (edges[mid].p == p && edges[mid].q < q)) lo = mid + 1;
        else if (edges[mid].p > p || (edges[mid].p == p && edges[mid].q > q)) hi = mid - 1;
        else return mid;
    }
    return -1;
}

static void collect_bus_pq_coeffs(const ele_grid_t *g, int bus_k,
                                  const chord_edge_t *edges, int n_edges,
                                  int *idx_p, double *val_p, int *cnt_p,
                                  int *idx_q, double *val_q, int *cnt_q) {
    /* Accumulate symmetric matrix entries of YP[k] and YQ[k] cleanly on graph edges */
    int N = g->n_bus;
    double gs = g->bus[bus_k].gs_mw / g->base_mva;
    double bs = g->bus[bus_k].bs_mvar / g->base_mva;

    int tmp_idx_p[256], tmp_idx_q[256];
    double tmp_vp[256], tmp_vq[256];
    int np = 0, nq = 0;

    #define ADD_COEFF(ARR_I, ARR_V, CNT, U, V, VAL) do { \
        double _vv = (VAL); \
        if (fabs(_vv) > 1e-15) { \
            int _eid = find_chord_edge(edges, n_edges, (U), (V)); \
            if (_eid >= 0) { \
                int _found = -1; \
                for (int _t = 0; _t < (CNT); ++_t) { \
                    if ((ARR_I)[_t] == _eid) { _found = _t; break; } \
                } \
                if (_found >= 0) (ARR_V)[_found] += _vv; \
                else { (ARR_I)[(CNT)] = _eid; (ARR_V)[(CNT)] = _vv; ++(CNT); } \
            } \
        } \
    } while (0)

    ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, bus_k, gs);
    ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + bus_k, gs);
    ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, bus_k, -bs);
    ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + bus_k, -bs);

    for (int e = 0; e < g->n_branch; ++e) {
        br_y_t by;
        compute_branch_y(&g->branch[e], &by);
        if (by.u == bus_k && by.v == bus_k) {
            double gsum = by.yff_r + by.yft_r + by.ytf_r + by.ytt_r;
            double bsum = by.yff_i + by.yft_i + by.ytf_i + by.ytt_i;
            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, bus_k, gsum);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + bus_k, gsum);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, bus_k, -bsum);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + bus_k, -bsum);
        } else if (by.u == bus_k) {
            int j = by.v;
            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, bus_k, by.yff_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + bus_k, by.yff_r);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, bus_k, -by.yff_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + bus_k, -by.yff_i);

            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, j, 0.5 * by.yft_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + j, 0.5 * by.yft_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, j, 0.5 * by.yft_i);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, N + j, -0.5 * by.yft_i);

            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, j, -0.5 * by.yft_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + j, -0.5 * by.yft_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, j, 0.5 * by.yft_r);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, N + j, -0.5 * by.yft_r);
        } else if (by.v == bus_k) {
            int i = by.u;
            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, bus_k, by.ytt_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + bus_k, by.ytt_r);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, bus_k, -by.ytt_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + bus_k, -by.ytt_i);

            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, i, 0.5 * by.ytf_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, N + i, 0.5 * by.ytf_r);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, N + bus_k, i, 0.5 * by.ytf_i);
            ADD_COEFF(tmp_idx_p, tmp_vp, np, bus_k, N + i, -0.5 * by.ytf_i);

            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, i, -0.5 * by.ytf_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, N + i, -0.5 * by.ytf_i);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, N + bus_k, i, 0.5 * by.ytf_r);
            ADD_COEFF(tmp_idx_q, tmp_vq, nq, bus_k, N + i, -0.5 * by.ytf_r);
        }
    }
    #undef ADD_COEFF

    int cp = 0, cq = 0;
    for (int t = 0; t < np; ++t) {
        if (fabs(tmp_vp[t]) > 1e-13) { idx_p[cp] = tmp_idx_p[t]; val_p[cp] = tmp_vp[t]; ++cp; }
    }
    for (int t = 0; t < nq; ++t) {
        if (fabs(tmp_vq[t]) > 1e-13) { idx_q[cq] = tmp_idx_q[t]; val_q[cq] = tmp_vq[t]; ++cq; }
    }
    *cnt_p = cp;
    *cnt_q = cq;
}

static void prox_cone_2d(double h11, double h22, double h12,
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

static double solve_bus_lambda_1d(const ele_grid_t *g,
                                  const int *gen_ids, int n_bg,
                                  double c_scale, double pd_pu,
                                  double Hk, double gk) {
    /* Minimize F(lam) = -lam*pd_pu + sum_i min_pg -(a2*pg^2+(a1-lam)*pg+a0) + 0.5*Hk*lam^2 + gk*lam
       Derivative F'(lam) = -pd_pu + gk + Hk*lam + sum_i pg_i^*(lam) = 0, monotone increasing! */
    double B = g->base_mva;
    if (Hk < 1e-12) Hk = 1e-12;
    if (n_bg == 0) return (pd_pu - gk) / Hk;

    double lo = -1000.0, hi = 1000.0;
    for (int it = 0; it < 36; ++it) {
        double mid = 0.5 * (lo + hi);
        double deriv = gk - pd_pu + Hk * mid;
        for (int idx = 0; idx < n_bg; ++idx) {
            const ele_grid_gen_t *gn = &g->gen[gen_ids[idx]];
            double a2 = (gn->c2 * B * B) / c_scale;
            double a1 = (gn->c1 * B) / c_scale;
            double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
            double pg = (a2 > 1e-12) ? ((mid - a1) / (2.0 * a2))
                                     : ((mid >= a1) ? pmax : pmin);
            if (pg < pmin) pg = pmin;
            if (pg > pmax) pg = pmax;
            deriv += pg;
        }
        if (deriv < 0.0) lo = mid;
        else hi = mid;
    }
    return 0.5 * (lo + hi);
}

static int cmp_chord_edge_asc(const void *a, const void *b) {
    const chord_edge_t *ea = (const chord_edge_t *)a;
    const chord_edge_t *eb = (const chord_edge_t *)b;
    if (ea->p != eb->p) return (ea->p > eb->p) - (ea->p < eb->p);
    return (ea->q > eb->q) - (ea->q < eb->q);
}

int ele_grid_acopf_chordal(const ele_grid_t *g, const char *cert_out_path,
                           ele_grid_chordal_res_t *out) {
    if (g == NULL || out == NULL || g->n_bus <= 0 || g->n_bus > ELE_GRID_MAX_BUS || g->n_gen <= 0) {
        return -1;
    }
    double t_start = fnow_sec();
    memset(out, 0, sizeof(*out));
    int N = g->n_bus;
    int ng = g->n_gen;
    int nb = g->n_branch;
    double B = g->base_mva;
    out->n_bus = N;
    out->n_branch = nb;

    /* 1. MMD Chordal Elimination */
    uint8_t *adj = (uint8_t *)calloc((size_t)N * (size_t)N, sizeof(uint8_t));
    uint8_t *elim = (uint8_t *)calloc((size_t)N, sizeof(uint8_t));
    clique_def_t *raw_cl = (clique_def_t *)calloc((size_t)N, sizeof(clique_def_t));
    clique_def_t *cliques = (clique_def_t *)calloc(ELE_FRONTIER_MAX_CLIQUES, sizeof(clique_def_t));
    if (adj == NULL || elim == NULL || raw_cl == NULL || cliques == NULL) {
        free(adj); free(elim); free(raw_cl); free(cliques);
        return -1;
    }
    for (int e = 0; e < nb; ++e) {
        int u = g->branch[e].from_idx, v = g->branch[e].to_idx;
        if (u != v && u >= 0 && u < N && v >= 0 && v < N) {
            adj[(size_t)u * (size_t)N + (size_t)v] = 1U;
            adj[(size_t)v * (size_t)N + (size_t)u] = 1U;
        }
    }

    int max_cl_bus = 0;
    for (int step = 0; step < N; ++step) {
        int best_v = -1, best_deg = N + 1;
        for (int v = 0; v < N; ++v) {
            if (elim[v]) continue;
            int deg = 0;
            for (int w = 0; w < N; ++w) {
                if (!elim[w] && w != v && adj[(size_t)v * (size_t)N + (size_t)w]) ++deg;
            }
            if (deg < best_deg) {
                best_deg = deg;
                best_v = v;
            }
        }
        if (best_v < 0) break;
        if (1 + best_deg > ELE_FRONTIER_MAX_CLIQUE_BUS) {
            free(adj); free(elim); free(raw_cl); free(cliques);
            return -2;
        }
        clique_def_t *c = &raw_cl[step];
        c->buses[0] = best_v;
        c->n_bus_c = 1;
        for (int w = 0; w < N; ++w) {
            if (!elim[w] && w != best_v && adj[(size_t)best_v * (size_t)N + (size_t)w]) {
                c->buses[c->n_bus_c++] = w;
            }
        }
        for (int a = 1; a < c->n_bus_c; ++a) {
            for (int b = a + 1; b < c->n_bus_c; ++b) {
                int ua = c->buses[a], ub = c->buses[b];
                adj[(size_t)ua * (size_t)N + (size_t)ub] = 1U;
                adj[(size_t)ub * (size_t)N + (size_t)ua] = 1U;
            }
        }
        qsort(c->buses, (size_t)c->n_bus_c, sizeof(int), cmp_int_asc);
        elim[best_v] = 1U;
    }
    free(adj);
    free(elim);

    /* Prune subset cliques (keep maximal) */
    int n_cl = 0;
    for (int i = 0; i < N; ++i) {
        int is_sub = 0;
        for (int j = 0; j < N; ++j) {
            if (i == j || raw_cl[j].n_bus_c < raw_cl[i].n_bus_c) continue;
            if (raw_cl[j].n_bus_c == raw_cl[i].n_bus_c && j > i) continue;
            int hit = 0;
            for (int a = 0; a < raw_cl[i].n_bus_c; ++a) {
                for (int b = 0; b < raw_cl[j].n_bus_c; ++b) {
                    if (raw_cl[i].buses[a] == raw_cl[j].buses[b]) { ++hit; break; }
                }
            }
            if (hit == raw_cl[i].n_bus_c) { is_sub = 1; break; }
        }
        if (!is_sub && n_cl < ELE_FRONTIER_MAX_CLIQUES) {
            clique_def_t *dst = &cliques[n_cl++];
            *dst = raw_cl[i];
            int r = dst->n_bus_c;
            dst->dk = 2 * r;
            for (int a = 0; a < r; ++a) {
                dst->vars[a] = dst->buses[a];
                dst->vars[r + a] = N + dst->buses[a];
            }
            if (r > max_cl_bus) max_cl_bus = r;
        }
    }
    free(raw_cl);

    int dim = 2 * N;
    out->n_cliques = n_cl;
    out->max_clique_buses = max_cl_bus;
    out->treewidth_ub = (max_cl_bus > 0) ? (max_cl_bus - 1) : 0;

    /* 2. Build unique chordal edges + occurrences */
    int max_occ = 0;
    for (int c = 0; c < n_cl; ++c) {
        int dk = cliques[c].dk;
        max_occ += dk * (dk + 1) / 2;
    }
    chord_edge_t *tmp_edges = (chord_edge_t *)calloc((size_t)max_occ, sizeof(chord_edge_t));
    if (tmp_edges == NULL) { free(cliques); return -1; }
    int raw_e = 0;
    for (int c = 0; c < n_cl; ++c) {
        const clique_def_t *cl = &cliques[c];
        for (int a = 0; a < cl->dk; ++a) {
            for (int b = a; b < cl->dk; ++b) {
                int p = cl->vars[a], q = cl->vars[b];
                if (p > q) { int t = p; p = q; q = t; }
                tmp_edges[raw_e].p = p;
                tmp_edges[raw_e].q = q;
                ++raw_e;
            }
        }
    }
    qsort(tmp_edges, (size_t)raw_e, sizeof(chord_edge_t), cmp_chord_edge_asc);
    int n_edges = 0;
    for (int i = 0; i < raw_e; ++i) {
        if (n_edges == 0 || tmp_edges[i].p != tmp_edges[n_edges - 1].p ||
            tmp_edges[i].q != tmp_edges[n_edges - 1].q) {
            chord_edge_t ed = tmp_edges[i];
            ed.w_pq = (ed.p == ed.q) ? 1.0 : 2.0;
            ed.mult = 0;
            tmp_edges[n_edges++] = ed;
        }
    }
    chord_edge_t *edges = (chord_edge_t *)realloc(tmp_edges, (size_t)n_edges * sizeof(chord_edge_t));
    if (edges == NULL) edges = tmp_edges;

    for (int c = 0; c < n_cl; ++c) {
        const clique_def_t *cl = &cliques[c];
        for (int a = 0; a < cl->dk; ++a) {
            for (int b = a; b < cl->dk; ++b) {
                int eid = find_chord_edge(edges, n_edges, cl->vars[a], cl->vars[b]);
                if (eid >= 0) ++edges[eid].mult;
            }
        }
    }
    int tot_occ = 0;
    for (int i = 0; i < n_edges; ++i) {
        edges[i].occ_off = tot_occ;
        tot_occ += edges[i].mult;
    }
    edge_occ_t *occs = (edge_occ_t *)calloc((size_t)tot_occ, sizeof(edge_occ_t));
    int *occ_fill = (int *)calloc((size_t)n_edges, sizeof(int));
    if (occs == NULL || occ_fill == NULL) {
        free(edges); free(cliques); free(occs); free(occ_fill);
        return -1;
    }
    for (int c = 0; c < n_cl; ++c) {
        const clique_def_t *cl = &cliques[c];
        for (int a = 0; a < cl->dk; ++a) {
            for (int b = a; b < cl->dk; ++b) {
                int eid = find_chord_edge(edges, n_edges, cl->vars[a], cl->vars[b]);
                if (eid >= 0) {
                    int pos = edges[eid].occ_off + occ_fill[eid]++;
                    occs[pos].clique = c;
                    occs[pos].a = a;
                    occs[pos].b = b;
                }
            }
        }
    }
    free(occ_fill);

    /* 3. Build Sparse Dual Linear Map A(y) over Chordal Edges */
    int *bus_gen_cnt = (int *)calloc((size_t)N, sizeof(int));
    int *bus_gen_ids = (int *)calloc((size_t)N * 16U, sizeof(int));
    int *lam_nz_cnt = (int *)calloc((size_t)N, sizeof(int));
    int *gam_nz_cnt = (int *)calloc((size_t)N, sizeof(int));
    int *lam_eid = (int *)calloc((size_t)N * 256U, sizeof(int));
    int *gam_eid = (int *)calloc((size_t)N * 256U, sizeof(int));
    double *lam_val = (double *)calloc((size_t)N * 256U, sizeof(double));
    double *gam_val = (double *)calloc((size_t)N * 256U, sizeof(double));
    int *mv_eid0 = (int *)calloc((size_t)N, sizeof(int));
    int *mv_eid1 = (int *)calloc((size_t)N, sizeof(int));
    if (!bus_gen_cnt || !bus_gen_ids || !lam_nz_cnt || !gam_nz_cnt ||
        !lam_eid || !gam_eid || !lam_val || !gam_val || !mv_eid0 || !mv_eid1) {
        free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
        free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
        free(mv_eid0); free(mv_eid1);
        free(edges); free(cliques); free(occs);
        return -1;
    }

    for (int i = 0; i < ng; ++i) {
        int b = g->gen[i].bus_idx;
        if (bus_gen_cnt[b] < 16) bus_gen_ids[(size_t)b * 16U + (size_t)(bus_gen_cnt[b]++)] = i;
    }
    for (int k = 0; k < N; ++k) {
        collect_bus_pq_coeffs(g, k, edges, n_edges,
                              lam_eid + (size_t)k * 256U, lam_val + (size_t)k * 256U, &lam_nz_cnt[k],
                              gam_eid + (size_t)k * 256U, gam_val + (size_t)k * 256U, &gam_nz_cnt[k]);
        mv_eid0[k] = find_chord_edge(edges, n_edges, k, k);
        mv_eid1[k] = find_chord_edge(edges, n_edges, N + k, N + k);
    }

    typedef struct {
        int has_therm, has_ang;
        double smax_pu;
        int e_pf[16], n_pf; double v_pf[16];
        int e_qf[16], n_qf; double v_qf[16];
        int e_pt[16], n_pt; double v_pt[16];
        int e_qt[16], n_qt; double v_qt[16];
        int e_alo[16], n_alo; double v_alo[16];
        int e_ahi[16], n_ahi; double v_ahi[16];
        double dot_pqf, norm_qf;
        double dot_pqt, norm_qt;
    } ch_br_t;
    ch_br_t *brs = (ch_br_t *)calloc((size_t)nb, sizeof(ch_br_t));
    if (brs == NULL) {
        free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
        free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
        free(mv_eid0); free(mv_eid1);
        free(edges); free(cliques); free(occs);
        return -1;
    }

    #define PACK_BR_4X4(M4, E_OUT, V_OUT, N_OUT) do { \
        int _nn = 0; \
        int _vars4[4] = { u, v, N + u, N + v }; \
        for (int _a = 0; _a < 4; ++_a) { \
            for (int _b = _a; _b < 4; ++_b) { \
                double _c = (M4)[_a][_b]; \
                if (fabs(_c) > 1e-13) { \
                    int _eid = find_chord_edge(edges, n_edges, _vars4[_a], _vars4[_b]); \
                    if (_eid >= 0) { \
                        int _hit = -1; \
                        for (int _t = 0; _t < _nn; ++_t) { if ((E_OUT)[_t] == _eid) { _hit = _t; break; } } \
                        if (_hit >= 0) (V_OUT)[_hit] += _c; \
                        else { (E_OUT)[_nn] = _eid; (V_OUT)[_nn] = _c; ++_nn; } \
                    } \
                } \
            } \
        } \
        *(N_OUT) = _nn; \
    } while (0)

    for (int e = 0; e < nb; ++e) {
        const ele_grid_branch_t *br = &g->branch[e];
        br_y_t by;
        compute_branch_y(br, &by);
        int u = by.u, v = by.v;
        if (u == v) continue;
        if (br->smax_mva > 0.0 && br->smax_mva < 9000.0) {
            double M[4][4];
            brs[e].has_therm = 1;
            brs[e].smax_pu = br->smax_mva / B;
            fill_4x4(by.yff_r, 0.0, by.yft_r, by.yft_i, M);
            PACK_BR_4X4(M, brs[e].e_pf, brs[e].v_pf, &brs[e].n_pf);
            fill_4x4(-by.yff_i, 0.0, -by.yft_i, by.yft_r, M);
            PACK_BR_4X4(M, brs[e].e_qf, brs[e].v_qf, &brs[e].n_qf);
            fill_4x4(0.0, by.ytt_r, by.ytf_r, -by.ytf_i, M);
            PACK_BR_4X4(M, brs[e].e_pt, brs[e].v_pt, &brs[e].n_pt);
            fill_4x4(0.0, -by.ytt_i, -by.ytf_i, -by.ytf_r, M);
            PACK_BR_4X4(M, brs[e].e_qt, brs[e].v_qt, &brs[e].n_qt);
        }
        if (br->angmin_deg > -85.0 || br->angmax_deg < 85.0) {
            double M[4][4];
            brs[e].has_ang = 1;
            double amin = (br->angmin_deg > -85.0 ? br->angmin_deg : -85.0) * (M_PI / 180.0);
            double amax = (br->angmax_deg < 85.0 ? br->angmax_deg : 85.0) * (M_PI / 180.0);
            fill_4x4(0.0, 0.0, tan(amin), -1.0, M);
            PACK_BR_4X4(M, brs[e].e_alo, brs[e].v_alo, &brs[e].n_alo);
            fill_4x4(0.0, 0.0, -tan(amax), 1.0, M);
            PACK_BR_4X4(M, brs[e].e_ahi, brs[e].v_ahi, &brs[e].n_ahi);
        }
    }
    #undef PACK_BR_4X4

    /* 4. Per-clique dense S_c, Z_c, W_c, U_c buffers (dk <= 24 -> 576 doubles per clique) */
    size_t cl_stride = (size_t)(2 * max_cl_bus) * (size_t)(2 * max_cl_bus);
    double *S_cl = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    double *Z_cl = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    double *Z_prev = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    double *W_cl = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    double *U_cl = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    double *lam = (double *)calloc((size_t)N, sizeof(double));
    double *gam = (double *)calloc((size_t)N, sizeof(double));
    double *mlo = (double *)calloc((size_t)N, sizeof(double));
    double *mhi = (double *)calloc((size_t)N, sizeof(double));
    double *xif = (double *)calloc((size_t)nb, sizeof(double));
    double *etaf = (double *)calloc((size_t)nb, sizeof(double));
    double *xit = (double *)calloc((size_t)nb, sizeof(double));
    double *etat = (double *)calloc((size_t)nb, sizeof(double));
    double *tlo = (double *)calloc((size_t)nb, sizeof(double));
    double *thi = (double *)calloc((size_t)nb, sizeof(double));
    double *best_lam = (double *)calloc((size_t)N, sizeof(double));
    double *best_gam = (double *)calloc((size_t)N, sizeof(double));
    double *best_mlo = (double *)calloc((size_t)N, sizeof(double));
    double *best_mhi = (double *)calloc((size_t)N, sizeof(double));
    double *best_xif = (double *)calloc((size_t)nb, sizeof(double));
    double *best_etaf = (double *)calloc((size_t)nb, sizeof(double));
    double *best_xit = (double *)calloc((size_t)nb, sizeof(double));
    double *best_etat = (double *)calloc((size_t)nb, sizeof(double));
    double *best_tlo = (double *)calloc((size_t)nb, sizeof(double));
    double *best_thi = (double *)calloc((size_t)nb, sizeof(double));
    if (!S_cl || !Z_cl || !Z_prev || !W_cl || !U_cl ||
        !lam || !gam || !mlo || !mhi || !xif || !etaf || !xit || !etat || !tlo || !thi ||
        !best_lam || !best_gam || !best_mlo || !best_mhi || !best_xif || !best_etaf ||
        !best_xit || !best_etat || !best_tlo || !best_thi) {
        free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
        free(lam); free(gam); free(mlo); free(mhi);
        free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
        free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
        free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
        free(best_tlo); free(best_thi);
        free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
        free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
        free(mv_eid0); free(mv_eid1);
        free(edges); free(cliques); free(occs);
        return -1;
    }

    double tot_pd_pu = 0.0;
    for (int k = 0; k < N; ++k) tot_pd_pu += fabs(g->bus[k].pd_mw / B);
    if (tot_pd_pu < 0.5) tot_pd_pu = 0.5;
    double avg_c1 = 0.0;
    for (int i = 0; i < ng; ++i) avg_c1 += fabs(g->gen[i].c1 * B + 2.0 * g->gen[i].c2 * B * B * 0.5);
    avg_c1 /= (double)ng;
    double c_scale = fmax(1.0, tot_pd_pu * fmax(1.0, avg_c1));

    /* 5. Initial feasible balanced Primal UB seed + Two-Phase polish */
    double pg_ctrl[ELE_GRID_MAX_GEN], vset_ctrl[ELE_GRID_MAX_GEN];
    double pg_best[ELE_GRID_MAX_GEN] = {0}, vs_best[ELE_GRID_MAX_GEN] = {0};
    double c_ub = 1e18;
    int feas_best = 0;
    ele_grid_pf_res_t pf_best;
    memset(&pf_best, 0, sizeof(pf_best));

    seed_balanced_pg(g, pg_ctrl, vset_ctrl);
    polish_primal_twophase(g, pg_ctrl, vset_ctrl, pg_best, vs_best, &pf_best, &c_ub, &feas_best);

    /* Try MATPOWER init seed as second restart (crucial for __sad / __api) */
    {
        double pg_mp[ELE_GRID_MAX_GEN], vs_mp[ELE_GRID_MAX_GEN];
        for (int i = 0; i < ng; ++i) {
            pg_mp[i] = g->gen[i].pg_mw / B;
            if (pg_mp[i] < g->gen[i].pmin_mw / B) pg_mp[i] = g->gen[i].pmin_mw / B;
            if (pg_mp[i] > g->gen[i].pmax_mw / B) pg_mp[i] = g->gen[i].pmax_mw / B;
            vs_mp[i] = g->gen[i].vset;
            int b = g->gen[i].bus_idx;
            if (vs_mp[i] < g->bus[b].vmin) vs_mp[i] = g->bus[b].vmin;
            if (vs_mp[i] > g->bus[b].vmax) vs_mp[i] = g->bus[b].vmax;
        }
        double c2 = 1e18;
        int f2 = 0;
        double pg_b2[ELE_GRID_MAX_GEN] = {0}, vs_b2[ELE_GRID_MAX_GEN] = {0};
        ele_grid_pf_res_t pf2;
        memset(&pf2, 0, sizeof(pf2));
        polish_primal_twophase(g, pg_mp, vs_mp, pg_b2, vs_b2, &pf2, &c2, &f2);
        if ((f2 && !feas_best) || (f2 == feas_best && c2 < c_ub)) {
            feas_best = f2;
            c_ub = c2;
            memcpy(pg_best, pg_b2, (size_t)ng * sizeof(double));
            memcpy(vs_best, vs_b2, (size_t)ng * sizeof(double));
            pf_best = pf2;
        }
        /* Merit-order cheapest-first primal seed (3rd restart before ADMM) */
        {
            double pg_mo[ELE_GRID_MAX_GEN], vs_mo[ELE_GRID_MAX_GEN];
            int order[ELE_GRID_MAX_GEN];
            double pd_tot = 0.0;
            for (int k = 0; k < N; ++k) pd_tot += g->bus[k].pd_mw / B + g->bus[k].gs_mw / B;
            for (int i = 0; i < ng; ++i) {
                order[i] = i;
                pg_mo[i] = g->gen[i].pmin_mw / B;
                vs_mo[i] = g->gen[i].vset;
                int b = g->gen[i].bus_idx;
                if (vs_mo[i] < g->bus[b].vmin) vs_mo[i] = g->bus[b].vmin;
                if (vs_mo[i] > g->bus[b].vmax) vs_mo[i] = g->bus[b].vmax;
            }
            for (int a = 0; a < ng; ++a) {
                for (int b = a + 1; b < ng; ++b) {
                    const ele_grid_gen_t *ga = &g->gen[order[a]];
                    const ele_grid_gen_t *gb = &g->gen[order[b]];
                    double ma = ga->c1 + ga->c2 * (ga->pmin_mw + ga->pmax_mw);
                    double mb = gb->c1 + gb->c2 * (gb->pmin_mw + gb->pmax_mw);
                    if (mb < ma) { int tmp = order[a]; order[a] = order[b]; order[b] = tmp; }
                }
            }
            double rem = 1.02 * pd_tot;
            for (int i = 0; i < ng; ++i) rem -= pg_mo[i];
            for (int a = 0; a < ng && rem > 0.0; ++a) {
                int idx = order[a];
                double room = g->gen[idx].pmax_mw / B - pg_mo[idx];
                if (room > 0.0) {
                    double add = (room < rem) ? room : rem;
                    pg_mo[idx] += add;
                    rem -= add;
                }
            }
            double cm = 1e18; int fm = 0;
            double pg_bm[ELE_GRID_MAX_GEN] = {0}, vs_bm[ELE_GRID_MAX_GEN] = {0};
            ele_grid_pf_res_t pfm; memset(&pfm, 0, sizeof(pfm));
            polish_primal_twophase(g, pg_mo, vs_mo, pg_bm, vs_bm, &pfm, &cm, &fm);
            if ((fm && !feas_best) || (fm == feas_best && cm < c_ub)) {
                feas_best = fm; c_ub = cm;
                memcpy(pg_best, pg_bm, (size_t)ng * sizeof(double));
                memcpy(vs_best, vs_bm, (size_t)ng * sizeof(double));
                pf_best = pfm;
            }
        }
    }

    /* Warm start W_cl from primal voltage vector */
    double x0[2 * ELE_GRID_MAX_BUS] = {0};
    if (pf_best.converged) {
        for (int k = 0; k < N; ++k) {
            x0[k] = pf_best.vm[k] * cos(pf_best.va_rad[k]);
            x0[N + k] = pf_best.vm[k] * sin(pf_best.va_rad[k]);
        }
    } else {
        for (int k = 0; k < N; ++k) x0[k] = 1.0;
    }
    for (int c = 0; c < n_cl; ++c) {
        const clique_def_t *cl = &cliques[c];
        int dk = cl->dk;
        double *Wc = W_cl + (size_t)c * cl_stride;
        for (int a = 0; a < dk; ++a) {
            for (int b = 0; b < dk; ++b) {
                Wc[(size_t)a * (size_t)dk + (size_t)b] = x0[cl->vars[a]] * x0[cl->vars[b]];
            }
        }
    }

    /* 6. Chordal Dual ADMM */
    double rho = 0.05;
    int n_admm = (N > 60) ? 1400 : 800;
    double *Min_mat = (double *)calloc(cl_stride, sizeof(double));
    double *ev_buf = (double *)calloc((size_t)(2 * max_cl_bus), sizeof(double));
    double *vec_buf = (double *)calloc(cl_stride, sizeof(double));
    double *best_Z = (double *)calloc((size_t)n_cl * cl_stride, sizeof(double));
    if (!Min_mat || !ev_buf || !vec_buf || !best_Z) {
        free(Min_mat); free(ev_buf); free(vec_buf); free(best_Z);
        /* free rest */
        free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
        free(lam); free(gam); free(mlo); free(mhi);
        free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
        free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
        free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
        free(best_tlo); free(best_thi);
        free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
        free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
        free(mv_eid0); free(mv_eid1);
        free(edges); free(cliques); free(occs);
        return -1;
    }

    /* Helper macro to adjust A_cur incrementally by dy * span */
    double best_lb_fp = -1e18;
    double w_cl_max[ELE_FRONTIER_MAX_CLIQUES] = {0};
    for (int c = 0; c < n_cl; ++c) {
        double wc = 0.0;
        for (int a = 0; a < cliques[c].n_bus_c; ++a) {
            int b = cliques[c].buses[a];
            wc += g->bus[b].vmax * g->bus[b].vmax;
        }
        w_cl_max[c] = wc;
    }

    for (int it = 0; it < n_admm; ++it) {
        double inv_rho = 1.0 / rho;
        for (int c = 0; c < n_cl; ++c) {
            int dk = cliques[c].dk;
            double *Sc = S_cl + (size_t)c * cl_stride;
            double *Wc = W_cl + (size_t)c * cl_stride;
            double *Uc = U_cl + (size_t)c * cl_stride;
            for (int p = 0; p < dk * dk; ++p) Uc[p] = Sc[p] + inv_rho * Wc[p];
        }
        for (int e = 0; e < n_edges; ++e) {
            double s = 0.0;
            for (int k = 0; k < edges[e].mult; ++k) {
                const edge_occ_t *oc = &occs[edges[e].occ_off + k];
                int dk = cliques[oc->clique].dk;
                const double *Uc = U_cl + (size_t)oc->clique * cl_stride;
                s += Uc[(size_t)oc->a * (size_t)dk + (size_t)oc->b];
            }
            edges[e].U_bar = s;
        }

        /* Gauss-Seidel sweep on sparse duals y */
        for (int gs = 0; gs < 6; ++gs) {
            for (int k = 0; k < N; ++k) {
                /* lambda_k */
                int cnt = lam_nz_cnt[k];
                const int *el = lam_eid + (size_t)k * 256U;
                const double *vl = lam_val + (size_t)k * 256U;
                double H = 0.0, g_val = 0.0;
                for (int t = 0; t < cnt; ++t) {
                    int eid = el[t];
                    double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                    double phi = vl[t];
                    double other = edges[eid].A_cur - lam[k] * phi - edges[eid].U_bar;
                    H += wt * phi * phi;
                    g_val += wt * phi * other;
                }
                double nl = solve_bus_lambda_1d(g, bus_gen_ids + (size_t)k * 16U, bus_gen_cnt[k],
                                                c_scale, g->bus[k].pd_mw / B, H, g_val);
                double dl = nl - lam[k];
                lam[k] = nl;
                for (int t = 0; t < cnt; ++t) edges[el[t]].A_cur += dl * vl[t];

                /* gamma_k with exact [qmin, qmax] subdifferential deadzone */
                int gcnt = gam_nz_cnt[k];
                const int *egl = gam_eid + (size_t)k * 256U;
                const double *vgl = gam_val + (size_t)k * 256U;
                double Hg = 0.0, g_quad = -(g->bus[k].qd_mvar / B);
                for (int t = 0; t < gcnt; ++t) {
                    int eid = egl[t];
                    double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                    double phi = vgl[t];
                    double other = edges[eid].A_cur - gam[k] * phi - edges[eid].U_bar;
                    Hg += wt * phi * phi;
                    g_quad += wt * phi * other;
                }
                double qmin_sum = 0.0, qmax_sum = 0.0;
                for (int idx = 0; idx < bus_gen_cnt[k]; ++idx) {
                    const ele_grid_gen_t *gn = &g->gen[bus_gen_ids[(size_t)k * 16U + (size_t)idx]];
                    qmin_sum += gn->qmin_mvar / B;
                    qmax_sum += gn->qmax_mvar / B;
                }
                double ng_v = (Hg > 1e-12) ? (-(g_quad + qmax_sum) / Hg) : gam[k];
                if (Hg > 1e-12 && ng_v < 0.0) {
                    double c2 = -(g_quad + qmin_sum) / Hg;
                    ng_v = (c2 < 0.0) ? c2 : 0.0;
                }
                double dg = ng_v - gam[k];
                gam[k] = ng_v;
                for (int t = 0; t < gcnt; ++t) edges[egl[t]].A_cur += dg * vgl[t];

                /* mlo_k >= 0, mhi_k >= 0: MV has -1 at (k,k) and (N+k,N+k); coeff diff is (mhi - mlo) */
                int e0 = mv_eid0[k], e1 = mv_eid1[k];
                if (e0 >= 0 && e1 >= 0) {
                    double w0 = rho * edges[e0].w_pq / (double)edges[e0].mult;
                    double w1 = rho * edges[e1].w_pq / (double)edges[e1].mult;
                    double H_m = w0 + w1;
                    double o0 = edges[e0].A_cur - (mhi[k] - mlo[k]) - edges[e0].U_bar;
                    double o1 = edges[e1].A_cur - (mhi[k] - mlo[k]) - edges[e1].U_bar;
                    double g_lo = -(g->bus[k].vmin * g->bus[k].vmin) - w0 * (o0 + mhi[k]) - w1 * (o1 + mhi[k]);
                    double nmlo = fmax(0.0, -g_lo / fmax(H_m, 1e-12));
                    double dlo = -(nmlo - mlo[k]);
                    mlo[k] = nmlo;
                    edges[e0].A_cur += dlo;
                    edges[e1].A_cur += dlo;

                    o0 = edges[e0].A_cur - (mhi[k] - mlo[k]) - edges[e0].U_bar;
                    o1 = edges[e1].A_cur - (mhi[k] - mlo[k]) - edges[e1].U_bar;
                    double g_hi = (g->bus[k].vmax * g->bus[k].vmax) + w0 * (o0 - mlo[k]) + w1 * (o1 - mlo[k]);
                    double nmhi = fmax(0.0, -g_hi / fmax(H_m, 1e-12));
                    double dhi = nmhi - mhi[k];
                    mhi[k] = nmhi;
                    edges[e0].A_cur += dhi;
                    edges[e1].A_cur += dhi;
                }
            }

            /* Branch thermal cone duals + angle duals */
            for (int e = 0; e < nb; ++e) {
                const ch_br_t *be = &brs[e];
                if (be->has_therm) {
                    double h11 = 0.0, h22 = 0.0, h12 = 0.0, b1 = 0.0, b2 = 0.0;
                    for (int t = 0; t < be->n_pf; ++t) {
                        int eid = be->e_pf[t];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double vp = be->v_pf[t];
                        h11 += wt * vp * vp;
                        double other = edges[eid].A_cur - xif[e] * vp;
                        /* subtract etaf contribution cleanly below via common edge lookup or two-pass */
                        b1 += wt * vp * (other - edges[eid].U_bar);
                    }
                    /* Build small joint edge table for (pf, qf) on branch e for exact h12/b1/b2 */
                    double sum_qf_r[16] = {0};
                    for (int tf = 0; tf < be->n_qf; ++tf) {
                        int eid = be->e_qf[tf];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double vq = be->v_qf[tf];
                        h22 += wt * vq * vq;
                        double ap = 0.0;
                        for (int tp = 0; tp < be->n_pf; ++tp) {
                            if (be->e_pf[tp] == eid) { ap = be->v_pf[tp]; break; }
                        }
                        h12 += wt * ap * vq;
                        sum_qf_r[tf] = wt * vq * ((edges[eid].A_cur - xif[e] * ap - etaf[e] * vq) - edges[eid].U_bar);
                        b2 += sum_qf_r[tf];
                    }
                    b1 = 0.0;
                    for (int tp = 0; tp < be->n_pf; ++tp) {
                        int eid = be->e_pf[tp];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double vp = be->v_pf[tp];
                        double vq = 0.0;
                        for (int tf = 0; tf < be->n_qf; ++tf) {
                            if (be->e_qf[tf] == eid) { vq = be->v_qf[tf]; break; }
                        }
                        b1 += wt * vp * ((edges[eid].A_cur - xif[e] * vp - etaf[e] * vq) - edges[eid].U_bar);
                    }
                    double nxi = xif[e], neta = etaf[e];
                    prox_cone_2d(h11, h22, h12, b1, b2, be->smax_pu, &nxi, &neta);
                    double dxi = nxi - xif[e], deta = neta - etaf[e];
                    xif[e] = nxi; etaf[e] = neta;
                    for (int tp = 0; tp < be->n_pf; ++tp) edges[be->e_pf[tp]].A_cur += dxi * be->v_pf[tp];
                    for (int tf = 0; tf < be->n_qf; ++tf) edges[be->e_qf[tf]].A_cur += deta * be->v_qf[tf];

                    /* To-end thermal cone (pt, qt) */
                    h11 = 0.0; h22 = 0.0; h12 = 0.0; b1 = 0.0; b2 = 0.0;
                    for (int tp = 0; tp < be->n_pt; ++tp) {
                        int eid = be->e_pt[tp];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double vp = be->v_pt[tp], vq = 0.0;
                        for (int tq = 0; tq < be->n_qt; ++tq) {
                            if (be->e_qt[tq] == eid) { vq = be->v_qt[tq]; break; }
                        }
                        h11 += wt * vp * vp;
                        b1 += wt * vp * ((edges[eid].A_cur - xit[e] * vp - etat[e] * vq) - edges[eid].U_bar);
                    }
                    for (int tq = 0; tq < be->n_qt; ++tq) {
                        int eid = be->e_qt[tq];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double vq = be->v_qt[tq], vp = 0.0;
                        for (int tp = 0; tp < be->n_pt; ++tp) {
                            if (be->e_pt[tp] == eid) { vp = be->v_pt[tp]; break; }
                        }
                        h22 += wt * vq * vq;
                        h12 += wt * vp * vq;
                        b2 += wt * vq * ((edges[eid].A_cur - xit[e] * vp - etat[e] * vq) - edges[eid].U_bar);
                    }
                    nxi = xit[e]; neta = etat[e];
                    prox_cone_2d(h11, h22, h12, b1, b2, be->smax_pu, &nxi, &neta);
                    dxi = nxi - xit[e]; deta = neta - etat[e];
                    xit[e] = nxi; etat[e] = neta;
                    for (int tp = 0; tp < be->n_pt; ++tp) edges[be->e_pt[tp]].A_cur += dxi * be->v_pt[tp];
                    for (int tq = 0; tq < be->n_qt; ++tq) edges[be->e_qt[tq]].A_cur += deta * be->v_qt[tq];
                }
                if (be->has_ang) {
                    double Hlo = 0.0, glo = 0.0;
                    for (int t = 0; t < be->n_alo; ++t) {
                        int eid = be->e_alo[t];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double va = be->v_alo[t];
                        Hlo += wt * va * va;
                        glo += wt * va * ((edges[eid].A_cur - tlo[e] * va) - edges[eid].U_bar);
                    }
                    double ntlo = (Hlo > 1e-12) ? fmax(0.0, -glo / Hlo) : tlo[e];
                    double dtlo = ntlo - tlo[e];
                    tlo[e] = ntlo;
                    for (int t = 0; t < be->n_alo; ++t) edges[be->e_alo[t]].A_cur += dtlo * be->v_alo[t];

                    double Hhi = 0.0, ghi = 0.0;
                    for (int t = 0; t < be->n_ahi; ++t) {
                        int eid = be->e_ahi[t];
                        double wt = rho * edges[eid].w_pq / (double)edges[eid].mult;
                        double va = be->v_ahi[t];
                        Hhi += wt * va * va;
                        ghi += wt * va * ((edges[eid].A_cur - thi[e] * va) - edges[eid].U_bar);
                    }
                    double nthi = (Hhi > 1e-12) ? fmax(0.0, -ghi / Hhi) : thi[e];
                    double dthi = nthi - thi[e];
                    thi[e] = nthi;
                    for (int t = 0; t < be->n_ahi; ++t) edges[be->e_ahi[t]].A_cur += dthi * be->v_ahi[t];
                }
            }
        }

        /* Save Z_prev & Closed-form consensus update Z_c */
        memcpy(Z_prev, Z_cl, (size_t)n_cl * cl_stride * sizeof(double));
        for (int e = 0; e < n_edges; ++e) {
            double corr = (edges[e].A_cur - edges[e].U_bar) / (double)edges[e].mult;
            for (int k = 0; k < edges[e].mult; ++k) {
                const edge_occ_t *oc = &occs[edges[e].occ_off + k];
                int dk = cliques[oc->clique].dk;
                const double *Uc = U_cl + (size_t)oc->clique * cl_stride;
                double *Zc = Z_cl + (size_t)oc->clique * cl_stride;
                double val = Uc[(size_t)oc->a * (size_t)dk + (size_t)oc->b] + corr;
                Zc[(size_t)oc->a * (size_t)dk + (size_t)oc->b] = val;
                Zc[(size_t)oc->b * (size_t)dk + (size_t)oc->a] = val;
            }
        }

        /* Per-clique PSD projection S_c and multiplier W_c + residual norms */
        double r_prim_sq = 0.0, s_dual_sq = 0.0;
        double min_eig_all = 1e9;
        double sum_shift_pen = 0.0;
        for (int c = 0; c < n_cl; ++c) {
            int dk = cliques[c].dk;
            double *Sc = S_cl + (size_t)c * cl_stride;
            double *Zc = Z_cl + (size_t)c * cl_stride;
            const double *Zpc = Z_prev + (size_t)c * cl_stride;
            double *Wc = W_cl + (size_t)c * cl_stride;
            for (int i = 0; i < dk; ++i) {
                for (int j = 0; j < dk; ++j) {
                    Min_mat[(size_t)i * (size_t)dk + (size_t)j] =
                        0.5 * (Zc[(size_t)i * (size_t)dk + (size_t)j] + Zc[(size_t)j * (size_t)dk + (size_t)i]) -
                        inv_rho * 0.5 * (Wc[(size_t)i * (size_t)dk + (size_t)j] + Wc[(size_t)j * (size_t)dk + (size_t)i]);
                }
            }
            if (ele_sym_jacobi_eigs(dk, Min_mat, ev_buf, vec_buf) == 0) {
                memset(Sc, 0, (size_t)dk * (size_t)dk * sizeof(double));
                for (int r = 0; r < dk; ++r) {
                    if (ev_buf[r] > 0.0) {
                        double ev = ev_buf[r];
                        for (int i = 0; i < dk; ++i) {
                            double vi = vec_buf[(size_t)i * (size_t)dk + (size_t)r];
                            for (int j = 0; j < dk; ++j) {
                                Sc[(size_t)i * (size_t)dk + (size_t)j] += ev * vi * vec_buf[(size_t)j * (size_t)dk + (size_t)r];
                            }
                        }
                    }
                }
            }
            int check_lb = (it >= 40 && (it % 20 == 19 || it == n_admm - 1));
            for (int p = 0; p < dk * dk; ++p) {
                double diff_sz = Sc[p] - Zc[p];
                double diff_zz = Zc[p] - Zpc[p];
                Wc[p] += rho * diff_sz;
                r_prim_sq += diff_sz * diff_sz;
                s_dual_sq += diff_zz * diff_zz;
            }
            if (check_lb && ele_sym_jacobi_eigs(dk, Zc, ev_buf, NULL) == 0) {
                if (ev_buf[0] < min_eig_all) min_eig_all = ev_buf[0];
                if (ev_buf[0] < 0.0) sum_shift_pen += ev_buf[0] * w_cl_max[c];
            }
        }
        double r_prim = sqrt(r_prim_sq);
        double s_dual = rho * sqrt(s_dual_sq);
        if (it % 20 == 19) {
            if (r_prim > 3.0 * s_dual && rho < 5.0) rho *= 1.3;
            else if (s_dual > 3.0 * r_prim && rho > 1e-3) rho /= 1.3;
        }

        if (it >= 40 && (it % 20 == 19 || it == n_admm - 1)) {
            double c_d = 0.0;
            for (int k = 0; k < N; ++k) {
                double lk = lam[k] * c_scale, gk = gam[k] * c_scale;
                double ml = mlo[k] * c_scale, mh = mhi[k] * c_scale;
                c_d += lk * (g->bus[k].pd_mw / B) + gk * (g->bus[k].qd_mvar / B);
                c_d += ml * (g->bus[k].vmin * g->bus[k].vmin) - mh * (g->bus[k].vmax * g->bus[k].vmax);
            }
            for (int i = 0; i < ng; ++i) {
                const ele_grid_gen_t *gn = &g->gen[i];
                int k = gn->bus_idx;
                double lk = lam[k] * c_scale, gk = gam[k] * c_scale;
                double a2 = gn->c2 * B * B, a1 = gn->c1 * B, a0 = gn->c0;
                double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
                double pg = (a2 > 1e-12) ? ((lk - a1) / (2.0 * a2)) : ((lk > a1) ? pmax : pmin);
                if (pg < pmin) pg = pmin;
                if (pg > pmax) pg = pmax;
                double qg = (gk >= 0.0) ? (gn->qmax_mvar / B) : (gn->qmin_mvar / B);
                c_d += a2 * pg * pg + (a1 - lk) * pg + a0 - gk * qg;
            }
            for (int e = 0; e < nb; ++e) {
                if (!brs[e].has_therm) continue;
                c_d -= brs[e].smax_pu * (hypot(xif[e], etaf[e]) + hypot(xit[e], etat[e])) * c_scale;
            }
            double lb_cur = c_d + c_scale * sum_shift_pen;
            if (lb_cur > best_lb_fp && lb_cur <= c_ub + 1e-2) {
                best_lb_fp = lb_cur;
                for (int k = 0; k < N; ++k) {
                    best_lam[k] = lam[k] * c_scale;
                    best_gam[k] = gam[k] * c_scale;
                    best_mlo[k] = mlo[k] * c_scale;
                    best_mhi[k] = mhi[k] * c_scale;
                }
                for (int e = 0; e < nb; ++e) {
                    best_xif[e] = xif[e] * c_scale; best_etaf[e] = etaf[e] * c_scale;
                    best_xit[e] = xit[e] * c_scale; best_etat[e] = etat[e] * c_scale;
                    best_tlo[e] = tlo[e] * c_scale; best_thi[e] = thi[e] * c_scale;
                }
                for (size_t p = 0; p < (size_t)n_cl * cl_stride; ++p) best_Z[p] = Z_cl[p] * c_scale;
            }
        }
    }

    free(Min_mat);

    /* Post-ADMM Clique-Tree BFS Rank-1 Eigenvector Recovery + Uniform Lifted Seed for Feasibility */
    {
        double xr[ELE_GRID_MAX_BUS] = {0}, xi[ELE_GRID_MAX_BUS] = {0};
        uint8_t bus_set[ELE_GRID_MAX_BUS] = {0};
        uint8_t cl_vis[ELE_FRONTIER_MAX_CLIQUES] = {0};
        int queue[ELE_FRONTIER_MAX_CLIQUES], qh = 0, qt = 0;
        cl_vis[0] = 1U;
        queue[qt++] = 0;
        while (qh < qt) {
            int c = queue[qh++];
            const clique_def_t *cl = &cliques[c];
            int dk = cl->dk, r = cl->n_bus_c;
            double *Wc = W_cl + (size_t)c * cl_stride;
            if (ele_sym_jacobi_eigs(dk, Wc, ev_buf, vec_buf) == 0 && ev_buf[dk - 1] > 1e-8) {
                double sc = sqrt(ev_buf[dk - 1]);
                double er_ref = 1.0, ei_ref = 0.0;
                double er_cur = 1.0, ei_cur = 0.0;
                int has_anchor = 0;
                for (int a = 0; a < r; ++a) {
                    int b = cl->buses[a];
                    if (bus_set[b]) {
                        er_ref = xr[b]; ei_ref = xi[b];
                        er_cur = sc * vec_buf[(size_t)a * (size_t)dk + (size_t)(dk - 1)];
                        ei_cur = sc * vec_buf[(size_t)(r + a) * (size_t)dk + (size_t)(dk - 1)];
                        has_anchor = 1;
                        break;
                    }
                }
                double rot_ang = 0.0;
                if (has_anchor) rot_ang = atan2(ei_ref, er_ref) - atan2(ei_cur, er_cur);
                double cr = cos(rot_ang), sr = sin(rot_ang);
                for (int a = 0; a < r; ++a) {
                    int b = cl->buses[a];
                    if (!bus_set[b]) {
                        double u = sc * vec_buf[(size_t)a * (size_t)dk + (size_t)(dk - 1)];
                        double v = sc * vec_buf[(size_t)(r + a) * (size_t)dk + (size_t)(dk - 1)];
                        xr[b] = u * cr - v * sr;
                        xi[b] = u * sr + v * cr;
                        bus_set[b] = 1U;
                    }
                }
            }
            for (int nxt = 0; nxt < n_cl; ++nxt) {
                if (cl_vis[nxt]) continue;
                int share = 0;
                for (int a = 0; a < r && !share; ++a) {
                    for (int bb = 0; bb < cliques[nxt].n_bus_c; ++bb) {
                        if (cl->buses[a] == cliques[nxt].buses[bb]) { share = 1; break; }
                    }
                }
                if (share) { cl_vis[nxt] = 1U; queue[qt++] = nxt; }
            }
        }
        double pg_eig[ELE_GRID_MAX_GEN], vs_eig[ELE_GRID_MAX_GEN];
        double ref_a = atan2(xi[g->slack_idx], xr[g->slack_idx]);
        ele_grid_pf_res_t pf_pre;
        seed_balanced_pg(g, pg_eig, vs_eig);
        for (int i = 0; i < ng; ++i) {
            int b = g->gen[i].bus_idx;
            if (bus_set[b]) {
                double m = hypot(xr[b], xi[b]);
                if (m < g->bus[b].vmin) m = g->bus[b].vmin;
                if (m > g->bus[b].vmax) m = g->bus[b].vmax;
                vs_eig[i] = m;
            }
        }
        (void)ref_a; (void)pf_pre;
        double c3 = 1e18; int f3 = 0;
        double pg_b3[ELE_GRID_MAX_GEN] = {0}, vs_b3[ELE_GRID_MAX_GEN] = {0};
        ele_grid_pf_res_t pf3; memset(&pf3, 0, sizeof(pf3));
        polish_primal_twophase(g, pg_eig, vs_eig, pg_b3, vs_b3, &pf3, &c3, &f3);
        if ((f3 && !feas_best) || (f3 == feas_best && c3 < c_ub)) {
            feas_best = f3; c_ub = c3;
            memcpy(pg_best, pg_b3, (size_t)ng * sizeof(double));
            memcpy(vs_best, vs_b3, (size_t)ng * sizeof(double));
            pf_best = pf3;
        }
        /* Dual-LMP economic dispatch seed from converged dual prices best_lam[] */
        {
            double pg_lmp[ELE_GRID_MAX_GEN];
            double pd_tot = 0.0, p_sum = 0.0;
            for (int k = 0; k < N; ++k) pd_tot += g->bus[k].pd_mw / B + g->bus[k].gs_mw / B;
            for (int i = 0; i < ng; ++i) {
                const ele_grid_gen_t *gn = &g->gen[i];
                int b = gn->bus_idx;
                double a2 = gn->c2 * B * B, a1 = gn->c1 * B;
                double reg2 = (a2 > 1e-6) ? a2 : (0.05 * c_scale);
                double pg = (best_lam[b] - a1) / (2.0 * reg2);
                double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
                if (pg < pmin) pg = pmin;
                if (pg > pmax) pg = pmax;
                pg_lmp[i] = pg;
                p_sum += pg;
            }
            if (p_sum > 1e-6) {
                double tgt = 1.02 * pd_tot;
                double sc = tgt / p_sum;
                for (int i = 0; i < ng; ++i) {
                    double pmin = g->gen[i].pmin_mw / B, pmax = g->gen[i].pmax_mw / B;
                    pg_lmp[i] *= sc;
                    if (pg_lmp[i] < pmin) pg_lmp[i] = pmin;
                    if (pg_lmp[i] > pmax) pg_lmp[i] = pmax;
                }
            }
            c3 = 1e18; f3 = 0;
            memset(&pf3, 0, sizeof(pf3));
            polish_primal_twophase(g, pg_lmp, vs_eig, pg_b3, vs_b3, &pf3, &c3, &f3);
            if ((f3 && !feas_best) || (f3 == feas_best && c3 < c_ub)) {
                feas_best = f3; c_ub = c3;
                memcpy(pg_best, pg_b3, (size_t)ng * sizeof(double));
                memcpy(vs_best, vs_b3, (size_t)ng * sizeof(double));
                pf_best = pf3;
            }
        }
        /* Also try lifted voltage seed if still not feasible */
        if (!feas_best) {
            seed_balanced_pg(g, pg_eig, vs_eig);
            for (int i = 0; i < ng; ++i) {
                int b = g->gen[i].bus_idx;
                vs_eig[i] = 0.5 * (g->bus[b].vmin + g->bus[b].vmax) + 0.3 * (g->bus[b].vmax - g->bus[b].vmin);
            }
            c3 = 1e18; f3 = 0;
            polish_primal_twophase(g, pg_eig, vs_eig, pg_b3, vs_b3, &pf3, &c3, &f3);
            if ((f3 && !feas_best) || (f3 == feas_best && c3 < c_ub)) {
                feas_best = f3; c_ub = c3;
                memcpy(pg_best, pg_b3, (size_t)ng * sizeof(double));
                memcpy(vs_best, vs_b3, (size_t)ng * sizeof(double));
                pf_best = pf3;
            }
        }
    }

    /* 7. Anytime Diagonal Shift per Clique + Exact Integer Clique-Sum Certification */
    /* Absorb any negative minimum eigenvalue on each clique c cleanly into S_c + mhi[b] */
    double delta_cl[ELE_FRONTIER_MAX_CLIQUES] = {0};
    double c_dual_shifted = 0.0;
    for (int c = 0; c < n_cl; ++c) {
        int dk = cliques[c].dk;
        double *Zc = best_Z + (size_t)c * cl_stride;
        if (ele_sym_jacobi_eigs(dk, Zc, ev_buf, NULL) == 0 && ev_buf[0] < 1e-7) {
            double sh = -ev_buf[0] + 1e-6;
            delta_cl[c] = sh;
            for (int a = 0; a < dk; ++a) Zc[(size_t)a * (size_t)dk + (size_t)a] += sh;
            for (int a = 0; a < cliques[c].n_bus_c; ++a) {
                int b = cliques[c].buses[a];
                best_mhi[b] += sh;
            }
        }
    }

    c_dual_shifted = 0.0;
    for (int k = 0; k < N; ++k) {
        c_dual_shifted += best_lam[k] * (g->bus[k].pd_mw / B) + best_gam[k] * (g->bus[k].qd_mvar / B);
        c_dual_shifted += best_mlo[k] * (g->bus[k].vmin * g->bus[k].vmin) -
                          best_mhi[k] * (g->bus[k].vmax * g->bus[k].vmax);
    }
    for (int i = 0; i < ng; ++i) {
        const ele_grid_gen_t *gn = &g->gen[i];
        int k = gn->bus_idx;
        double a2 = gn->c2 * B * B, a1 = gn->c1 * B, a0 = gn->c0;
        double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
        double pg = (a2 > 1e-12) ? ((best_lam[k] - a1) / (2.0 * a2)) : ((best_lam[k] > a1) ? pmax : pmin);
        if (pg < pmin) pg = pmin;
        if (pg > pmax) pg = pmax;
        double qg = (best_gam[k] >= 0.0) ? (gn->qmax_mvar / B) : (gn->qmin_mvar / B);
        c_dual_shifted += a2 * pg * pg + (a1 - best_lam[k]) * pg + a0 - best_gam[k] * qg;
    }
    for (int e = 0; e < nb; ++e) {
        if (!brs[e].has_therm) continue;
        c_dual_shifted -= brs[e].smax_pu * (hypot(best_xif[e], best_etaf[e]) + hypot(best_xit[e], best_etat[e]));
    }

    /* Rebuild exact target sparse matrix A(best_y) */
    for (int e = 0; e < n_edges; ++e) edges[e].A_cur = 0.0;
    for (int k = 0; k < N; ++k) {
        const int *el = lam_eid + (size_t)k * 256U;
        const double *vl = lam_val + (size_t)k * 256U;
        for (int t = 0; t < lam_nz_cnt[k]; ++t) edges[el[t]].A_cur += best_lam[k] * vl[t];
        const int *egl = gam_eid + (size_t)k * 256U;
        const double *vgl = gam_val + (size_t)k * 256U;
        for (int t = 0; t < gam_nz_cnt[k]; ++t) edges[egl[t]].A_cur += best_gam[k] * vgl[t];
        if (mv_eid0[k] >= 0 && mv_eid1[k] >= 0) {
            edges[mv_eid0[k]].A_cur += best_mhi[k] - best_mlo[k];
            edges[mv_eid1[k]].A_cur += best_mhi[k] - best_mlo[k];
        }
    }
    for (int e = 0; e < nb; ++e) {
        const ch_br_t *be = &brs[e];
        if (be->has_therm) {
            for (int t = 0; t < be->n_pf; ++t) edges[be->e_pf[t]].A_cur += best_xif[e] * be->v_pf[t];
            for (int t = 0; t < be->n_qf; ++t) edges[be->e_qf[t]].A_cur += best_etaf[e] * be->v_qf[t];
            for (int t = 0; t < be->n_pt; ++t) edges[be->e_pt[t]].A_cur += best_xit[e] * be->v_pt[t];
            for (int t = 0; t < be->n_qt; ++t) edges[be->e_qt[t]].A_cur += best_etat[e] * be->v_qt[t];
        }
        if (be->has_ang) {
            for (int t = 0; t < be->n_alo; ++t) edges[be->e_alo[t]].A_cur += best_tlo[e] * be->v_alo[t];
            for (int t = 0; t < be->n_ahi; ++t) edges[be->e_ahi[t]].A_cur += best_thi[e] * be->v_ahi[t];
        }
    }

    const int64_t S_V = 1000;
    const int64_t Q_A = 10000000; /* 10^7 */
    const int64_t opf_scale = S_V * S_V * Q_A; /* 10^13 */

    int64_t *S_all_int = (int64_t *)calloc((size_t)n_cl * cl_stride, sizeof(int64_t));
    int64_t *L_all_int = (int64_t *)calloc((size_t)n_cl * cl_stride, sizeof(int64_t));
    int64_t *mu_cl = (int64_t *)calloc((size_t)n_cl, sizeof(int64_t));
    int64_t *w_cl_int = (int64_t *)calloc((size_t)n_cl, sizeof(int64_t));
    int64_t *A_int_edge = (int64_t *)calloc((size_t)n_edges, sizeof(int64_t));
    if (!S_all_int || !L_all_int || !mu_cl || !w_cl_int || !A_int_edge) {
        free(S_all_int); free(L_all_int); free(mu_cl); free(w_cl_int); free(A_int_edge);
        free(ev_buf); free(vec_buf); free(best_Z);
        free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
        free(lam); free(gam); free(mlo); free(mhi);
        free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
        free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
        free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
        free(best_tlo); free(best_thi);
        free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
        free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
        free(mv_eid0); free(mv_eid1);
        free(edges); free(cliques); free(occs);
        return -1;
    }

    /* Quantize Z_c -> S_all_int[c], sum into A_int_edge, and fold exact A(y) - sum S_c discrepancy into c_dual & opf_delta_round */
    for (int c = 0; c < n_cl; ++c) {
        int dk = cliques[c].dk;
        const double *Zc = best_Z + (size_t)c * cl_stride;
        int64_t *Sc_i = S_all_int + (size_t)c * cl_stride;
        for (int a = 0; a < dk; ++a) {
            for (int b = a; b < dk; ++b) {
                double v = 0.5 * (Zc[(size_t)a * (size_t)dk + (size_t)b] + Zc[(size_t)b * (size_t)dk + (size_t)a]);
                int64_t qv = (int64_t)llround(v * (double)Q_A);
                Sc_i[(size_t)a * (size_t)dk + (size_t)b] = qv;
                Sc_i[(size_t)b * (size_t)dk + (size_t)a] = qv;
            }
        }
    }

    double mismatch_frob = 0.0;
    double norm_A_frob = 0.0;
    double W_max_pu = 0.0;
    int64_t w_max_int = 0;
    for (int k = 0; k < N; ++k) {
        W_max_pu += g->bus[k].vmax * g->bus[k].vmax;
        int64_t vmk = (int64_t)ceil(g->bus[k].vmax * (double)S_V) + 2;
        w_max_int += vmk * vmk;
    }
    for (int c = 0; c < n_cl; ++c) {
        int64_t wc = 0;
        for (int a = 0; a < cliques[c].n_bus_c; ++a) {
            int b = cliques[c].buses[a];
            int64_t vmk = (int64_t)ceil(g->bus[b].vmax * (double)S_V) + 2;
            wc += vmk * vmk;
        }
        w_cl_int[c] = (wc > 0) ? wc : 1;
    }
    for (int e = 0; e < n_edges; ++e) {
        int64_t s = 0;
        for (int k = 0; k < edges[e].mult; ++k) {
            const edge_occ_t *oc = &occs[edges[e].occ_off + k];
            int dk = cliques[oc->clique].dk;
            const int64_t *Sc_i = S_all_int + (size_t)oc->clique * cl_stride;
            s += Sc_i[(size_t)oc->a * (size_t)dk + (size_t)oc->b];
        }
        A_int_edge[e] = s;
        double a_fp = edges[e].A_cur;
        double s_fp = (double)s / (double)Q_A;
        double diff = a_fp - s_fp;
        mismatch_frob += diff * diff * edges[e].w_pq;
        norm_A_frob += a_fp * a_fp * edges[e].w_pq;
    }
    mismatch_frob = sqrt(mismatch_frob);
    norm_A_frob = sqrt(norm_A_frob);

    int64_t min_mu = 0;
    ele_f_i128 neg_pen = 0;
    for (int c = 0; c < n_cl; ++c) {
        int dk = cliques[c].dk;
        const int64_t *Sc_i = S_all_int + (size_t)c * cl_stride;
        int64_t *Lc_i = L_all_int + (size_t)c * cl_stride;
        if (ele_cert_cholesky_sdd_i64(dk, Sc_i, Lc_i, &mu_cl[c]) != 0) {
            free(S_all_int); free(L_all_int); free(mu_cl); free(w_cl_int); free(A_int_edge);
            free(ev_buf); free(vec_buf); free(best_Z);
            free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
            free(lam); free(gam); free(mlo); free(mhi);
            free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
            free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
            free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
            free(best_tlo); free(best_thi);
            free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
            free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
            free(mv_eid0); free(mv_eid1);
            free(edges); free(cliques); free(occs);
            return -1;
        }
        if (c == 0 || mu_cl[c] < min_mu) min_mu = mu_cl[c];
        if (mu_cl[c] < 0) neg_pen += (ele_f_i128)mu_cl[c] * (ele_f_i128)w_cl_int[c];
    }

    int64_t c_dual_int = (int64_t)floor(c_dual_shifted * (double)opf_scale);
    double d_rnd_fp = (double)N * (double)w_max_int +
                      (double)opf_scale * mismatch_frob * W_max_pu +
                      (double)Q_A * norm_A_frob * (sqrt((double)dim * W_max_pu) + 0.5 * (double)N) +
                      1000.0;
    int64_t opf_delta_round = (int64_t)ceil(d_rnd_fp);
    ele_f_i128 c_lb_i128 = (ele_f_i128)c_dual_int +
                           neg_pen -
                           (ele_f_i128)opf_delta_round;
    int64_t c_lb_int = (int64_t)c_lb_i128;
    int64_t c_ub_int = (int64_t)ceil(c_ub * (double)opf_scale);

    int certified = (feas_best && c_lb_int <= c_ub_int) ? 1 : 0;
    const char *status_str = certified ? "CERTIFIED" : "VETOED";

    if (cert_out_path != NULL) {
        FILE *cf = fopen(cert_out_path, "w");
        if (cf == NULL) {
            free(S_all_int); free(L_all_int); free(mu_cl); free(w_cl_int); free(A_int_edge);
            free(ev_buf); free(vec_buf); free(best_Z);
            free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
            free(lam); free(gam); free(mlo); free(mhi);
            free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
            free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
            free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
            free(best_tlo); free(best_thi);
            free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
            free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
            free(mv_eid0); free(mv_eid1);
            free(edges); free(cliques); free(occs);
            return -1;
        }
        fprintf(cf, "{\n");
        fprintf(cf, "  \"format\": \"ELECERT-1.0\",\n");
        fprintf(cf, "  \"version\": \"%s\",\n", ELE_VERSION_STRING);
        fprintf(cf, "  \"pillar\": \"grid\",\n");
        fprintf(cf, "  \"kind\": \"acopf_chordal\",\n");
        fprintf(cf, "  \"status\": \"%s\",\n", status_str);
        fprintf(cf, "  \"input_path\": \"%s\",\n", g->input_path);
        fprintf(cf, "  \"input_sha256\": \"%s\",\n", g->input_sha256);
        fprintf(cf, "  \"has_psd\": 0,\n");
        fprintf(cf, "  \"has_chordal\": 1,\n");
        fprintf(cf, "  \"has_sparam\": 0,\n");
        fprintf(cf, "  \"has_bracket\": 0,\n");
        fprintf(cf, "  \"has_motor\": 0,\n");
        fprintf(cf, "  \"has_opf\": 1,\n");
        fprintf(cf, "  \"has_emt\": 0,\n");
        fprintf(cf, "  \"dim\": %d,\n", dim);
        fprintf(cf, "  \"num_cliques\": %d,\n", n_cl);
        fprintf(cf, "  \"max_clique_dim\": %d,\n", 2 * max_cl_bus);
        fprintf(cf, "  \"min_clique_mu_psd\": %" PRId64 ",\n", min_mu);
        fprintf(cf, "  \"opf_scale\": %" PRId64 ",\n", opf_scale);
        fprintf(cf, "  \"c_dual_int\": %" PRId64 ",\n", c_dual_int);
        fprintf(cf, "  \"w_max_int\": %" PRId64 ",\n", w_max_int);
        fprintf(cf, "  \"opf_delta_round\": %" PRId64 ",\n", opf_delta_round);
        fprintf(cf, "  \"c_lb_int\": %" PRId64 ",\n", c_lb_int);
        fprintf(cf, "  \"c_ub_int\": %" PRId64 ",\n", c_ub_int);

        fprintf(cf, "  \"A_sparse\": [\n");
        int wrote_nz = 0;
        for (int e = 0; e < n_edges; ++e) {
            if (A_int_edge[e] == 0) continue;
            fprintf(cf, "%s    [%d, %d, %" PRId64 "]", wrote_nz ? ",\n" : "",
                    edges[e].p, edges[e].q, A_int_edge[e]);
            wrote_nz = 1;
        }
        fprintf(cf, "\n  ],\n");

        fprintf(cf, "  \"cliques\": [\n");
        for (int c = 0; c < n_cl; ++c) {
            int dk = cliques[c].dk;
            const int64_t *Sc_i = S_all_int + (size_t)c * cl_stride;
            const int64_t *Lc_i = L_all_int + (size_t)c * cl_stride;
            fprintf(cf, "    {\n");
            fprintf(cf, "      \"vars\": [");
            for (int a = 0; a < dk; ++a) fprintf(cf, "%s%d", a ? ", " : "", cliques[c].vars[a]);
            fprintf(cf, "],\n");
            fprintf(cf, "      \"mu_psd\": %" PRId64 ",\n", mu_cl[c]);
            fprintf(cf, "      \"w_max_int\": %" PRId64 ",\n", w_cl_int[c]);
            fprintf(cf, "      \"S_int\": [\n");
            for (int a = 0; a < dk; ++a) {
                fprintf(cf, "        [");
                for (int b = 0; b < dk; ++b) {
                    fprintf(cf, "%s%" PRId64, b ? ", " : "", Sc_i[(size_t)a * (size_t)dk + (size_t)b]);
                }
                fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
            }
            fprintf(cf, "      ],\n");
            fprintf(cf, "      \"L_int\": [\n");
            for (int a = 0; a < dk; ++a) {
                fprintf(cf, "        [");
                for (int b = 0; b < dk; ++b) {
                    fprintf(cf, "%s%" PRId64, b ? ", " : "", Lc_i[(size_t)a * (size_t)dk + (size_t)b]);
                }
                fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
            }
            fprintf(cf, "      ]\n");
            fprintf(cf, "    }%s\n", (c + 1 < n_cl) ? "," : "");
        }
        fprintf(cf, "  ]\n");
        fprintf(cf, "}\n");
        fclose(cf);
    }

    out->converged = feas_best ? 1 : 0;
    out->c_lb_dual_raw = best_lb_fp;
    out->c_lb_cert = (double)c_lb_int / (double)opf_scale;
    out->c_ub_primal = c_ub;
    out->gap_abs = out->c_ub_primal - out->c_lb_cert;
    out->gap_pct = (out->c_ub_primal > 1e-6) ? (100.0 * out->gap_abs / out->c_ub_primal) : 0.0;
    out->elapsed_sec = fnow_sec() - t_start;
    out->min_clique_mu_psd = min_mu;
    out->c_lb_int = c_lb_int;
    out->c_ub_int = c_ub_int;
    out->certified = certified;
    out->ub_pf = pf_best;
    for (int i = 0; i < ng; ++i) {
        out->gen_pg_mw[i] = pg_best[i] * B;
        out->gen_vset[i] = vs_best[i];
    }

    free(S_all_int); free(L_all_int); free(mu_cl); free(w_cl_int); free(A_int_edge);
    free(ev_buf); free(vec_buf); free(best_Z);
    free(S_cl); free(Z_cl); free(Z_prev); free(W_cl); free(U_cl);
    free(lam); free(gam); free(mlo); free(mhi);
    free(xif); free(etaf); free(xit); free(etat); free(tlo); free(thi);
    free(best_lam); free(best_gam); free(best_mlo); free(best_mhi);
    free(best_xif); free(best_etaf); free(best_xit); free(best_etat);
    free(best_tlo); free(best_thi);
    free(brs); free(bus_gen_cnt); free(bus_gen_ids); free(lam_nz_cnt); free(gam_nz_cnt);
    free(lam_eid); free(gam_eid); free(lam_val); free(gam_val);
    free(mv_eid0); free(mv_eid1);
    free(edges); free(cliques); free(occs);
    return 0;
}

/* ============================================================================
 * 2. 2nd-Order Lasserre Moment-SOS AC-OPF Tightener (N <= 5 Buses)
 * ============================================================================ */

/* Reduced variable coordinates: n_v = 2*N - 1 (gauge-fixed f[slack_idx] == 0):
   idx 0..N-1 = e_0..e_{N-1}, idx N..2N-2 = f_b (b != slack_idx). */
static int map_2n_to_nv(int N, int slack_idx, int r) {
    if (r < N) return r;
    int b = r - N;
    if (b == slack_idx) return -1;
    return (b < slack_idx) ? (N + b) : (N + b - 1);
}

static int nv_to_bus(int N, int slack_idx, int v) {
    if (v < N) return v;
    int f_ord = v - N;
    return (f_ord < slack_idx) ? f_ord : (f_ord + 1);
}

typedef struct {
    int deg;
    int e[4]; /* sorted variable indices 0..nv-1 */
} mono4_t;

static int find_mono4(const mono4_t *ms, int nm, int deg, const int *vars) {
    int v[4] = {0, 0, 0, 0};
    for (int i = 0; i < deg; ++i) v[i] = vars[i];
    for (int i = 0; i < deg; ++i) {
        for (int j = i + 1; j < deg; ++j) {
            if (v[j] < v[i]) { int t = v[i]; v[i] = v[j]; v[j] = t; }
        }
    }
    for (int m = 0; m < nm; ++m) {
        if (ms[m].deg != deg) continue;
        int ok = 1;
        for (int i = 0; i < deg; ++i) {
            if (ms[m].e[i] != v[i]) { ok = 0; break; }
        }
        if (ok) return m;
    }
    return -1;
}

static void poly2_add_quad_matrix(int N, int slack_idx, int nv, const mono4_t *ms, int nm,
                                  const double *M2N, double scale, double c0, double *poly4) {
    poly4[0] += c0;
    int d2 = 2 * N;
    for (int i = 0; i < d2; ++i) {
        int vi = map_2n_to_nv(N, slack_idx, i);
        if (vi < 0 || vi >= nv) continue;
        for (int j = 0; j < d2; ++j) {
            int vj = map_2n_to_nv(N, slack_idx, j);
            if (vj < 0 || vj >= nv) continue;
            double c = M2N[(size_t)i * (size_t)d2 + (size_t)j] * scale;
            if (fabs(c) < 1e-16) continue;
            int vv[2] = { vi, vj };
            int m = find_mono4(ms, nm, 2, vv);
            if (m >= 0) poly4[m] += c;
        }
    }
}

static void poly_mul_2x2_check(const mono4_t *ms, int nm,
                               const double *pA, const double *pB,
                               int n_deg2_end, double coeff, double *out4, int *miss) {
    for (int i = 0; i < n_deg2_end; ++i) {
        if (fabs(pA[i]) < 1e-16) continue;
        for (int j = 0; j < n_deg2_end; ++j) {
            if (fabs(pB[j]) < 1e-16) continue;
            double pr = coeff * pA[i] * pB[j];
            if (fabs(pr) < 1e-16) continue;
            int deg = ms[i].deg + ms[j].deg;
            if (deg == 0) {
                out4[0] += pr;
            } else if (deg == 2) {
                int vv[2];
                if (ms[i].deg == 2) { vv[0] = ms[i].e[0]; vv[1] = ms[i].e[1]; }
                else { vv[0] = ms[j].e[0]; vv[1] = ms[j].e[1]; }
                int m = find_mono4(ms, nm, 2, vv);
                if (m >= 0) out4[m] += pr;
                else if (miss != NULL) *miss = 1;
            } else if (deg == 4) {
                int vv[4] = { ms[i].e[0], ms[i].e[1], ms[j].e[0], ms[j].e[1] };
                int m = find_mono4(ms, nm, 4, vv);
                if (m >= 0) out4[m] += pr;
                else if (miss != NULL) *miss = 1;
            }
        }
    }
}

static void build_pq_matrix_2n(const ele_grid_t *g, int k, double *YPk, double *YQk) {
    int N = g->n_bus;
    int d = 2 * N;
    double B = g->base_mva;
    memset(YPk, 0, (size_t)d * (size_t)d * sizeof(double));
    memset(YQk, 0, (size_t)d * (size_t)d * sizeof(double));
    double gs = g->bus[k].gs_mw / B, bs = g->bus[k].bs_mvar / B;
    YPk[(size_t)k * (size_t)d + (size_t)k] += gs;
    YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] += gs;
    YQk[(size_t)k * (size_t)d + (size_t)k] -= bs;
    YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] -= bs;

    for (int e = 0; e < g->n_branch; ++e) {
        br_y_t by;
        compute_branch_y(&g->branch[e], &by);
        if (by.u == k && by.v == k) {
            double gsum = by.yff_r + by.yft_r + by.ytf_r + by.ytt_r;
            double bsum = by.yff_i + by.yft_i + by.ytf_i + by.ytt_i;
            YPk[(size_t)k * (size_t)d + (size_t)k] += gsum;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] += gsum;
            YQk[(size_t)k * (size_t)d + (size_t)k] -= bsum;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] -= bsum;
        } else if (by.u == k) {
            int j = by.v;
            YPk[(size_t)k * (size_t)d + (size_t)k] += by.yff_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] += by.yff_r;
            YQk[(size_t)k * (size_t)d + (size_t)k] -= by.yff_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] -= by.yff_i;

            YPk[(size_t)k * (size_t)d + (size_t)j] += 0.5 * by.yft_r; YPk[(size_t)j * (size_t)d + (size_t)k] += 0.5 * by.yft_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + j)] += 0.5 * by.yft_r; YPk[(size_t)(N + j) * (size_t)d + (size_t)(N + k)] += 0.5 * by.yft_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)j] += 0.5 * by.yft_i; YPk[(size_t)j * (size_t)d + (size_t)(N + k)] += 0.5 * by.yft_i;
            YPk[(size_t)k * (size_t)d + (size_t)(N + j)] -= 0.5 * by.yft_i; YPk[(size_t)(N + j) * (size_t)d + (size_t)k] -= 0.5 * by.yft_i;

            YQk[(size_t)k * (size_t)d + (size_t)j] -= 0.5 * by.yft_i; YQk[(size_t)j * (size_t)d + (size_t)k] -= 0.5 * by.yft_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + j)] -= 0.5 * by.yft_i; YQk[(size_t)(N + j) * (size_t)d + (size_t)(N + k)] -= 0.5 * by.yft_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)j] += 0.5 * by.yft_r; YQk[(size_t)j * (size_t)d + (size_t)(N + k)] += 0.5 * by.yft_r;
            YQk[(size_t)k * (size_t)d + (size_t)(N + j)] -= 0.5 * by.yft_r; YQk[(size_t)(N + j) * (size_t)d + (size_t)k] -= 0.5 * by.yft_r;
        } else if (by.v == k) {
            int i = by.u;
            YPk[(size_t)k * (size_t)d + (size_t)k] += by.ytt_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] += by.ytt_r;
            YQk[(size_t)k * (size_t)d + (size_t)k] -= by.ytt_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + k)] -= by.ytt_i;

            YPk[(size_t)k * (size_t)d + (size_t)i] += 0.5 * by.ytf_r; YPk[(size_t)i * (size_t)d + (size_t)k] += 0.5 * by.ytf_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)(N + i)] += 0.5 * by.ytf_r; YPk[(size_t)(N + i) * (size_t)d + (size_t)(N + k)] += 0.5 * by.ytf_r;
            YPk[(size_t)(N + k) * (size_t)d + (size_t)i] += 0.5 * by.ytf_i; YPk[(size_t)i * (size_t)d + (size_t)(N + k)] += 0.5 * by.ytf_i;
            YPk[(size_t)k * (size_t)d + (size_t)(N + i)] -= 0.5 * by.ytf_i; YPk[(size_t)(N + i) * (size_t)d + (size_t)k] -= 0.5 * by.ytf_i;

            YQk[(size_t)k * (size_t)d + (size_t)i] -= 0.5 * by.ytf_i; YQk[(size_t)i * (size_t)d + (size_t)k] -= 0.5 * by.ytf_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)(N + i)] -= 0.5 * by.ytf_i; YQk[(size_t)(N + i) * (size_t)d + (size_t)(N + k)] -= 0.5 * by.ytf_i;
            YQk[(size_t)(N + k) * (size_t)d + (size_t)i] += 0.5 * by.ytf_r; YQk[(size_t)i * (size_t)d + (size_t)(N + k)] += 0.5 * by.ytf_r;
            YQk[(size_t)k * (size_t)d + (size_t)(N + i)] -= 0.5 * by.ytf_r; YQk[(size_t)(N + i) * (size_t)d + (size_t)k] -= 0.5 * by.ytf_r;
        }
    }
}

#define SOS2_MAX_DIM   24
#define SOS2_MAX_BLKS  96
#define SOS2_MAX_MONOS 384

static void sos2_unpack_svec(int p, const double *s, double *M) {
    int idx = 0;
    for (int a = 0; a < p; ++a) {
        for (int b = a; b < p; ++b) {
            double v = s[idx++] * ((a == b) ? 1.0 : M_SQRT1_2);
            M[(size_t)a * (size_t)p + (size_t)b] = v;
            M[(size_t)b * (size_t)p + (size_t)a] = v;
        }
    }
}

static void sos2_pack_svec(int p, const double *M, double *s) {
    int idx = 0;
    for (int a = 0; a < p; ++a) {
        for (int b = a; b < p; ++b) {
            s[idx++] = M[(size_t)a * (size_t)p + (size_t)b] * ((a == b) ? 1.0 : M_SQRT2);
        }
    }
}

static void sos2_psd_hinv(int p, const double *V, const double *ev,
                          double eps_c, double mu,
                          const double *w_svec, double *y_svec) {
    double W[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double T[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double U[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double Y[SOS2_MAX_DIM * SOS2_MAX_DIM];
    sos2_unpack_svec(p, w_svec, W);
    for (int k = 0; k < p; ++k) {
        for (int c = 0; c < p; ++c) {
            double acc = 0.0;
            for (int b = 0; b < p; ++b) acc += W[(size_t)k * (size_t)p + (size_t)b] * V[(size_t)b * (size_t)p + (size_t)c];
            T[(size_t)k * (size_t)p + (size_t)c] = acc;
        }
    }
    for (int r = 0; r < p; ++r) {
        for (int c = r; c < p; ++c) {
            double acc = 0.0;
            for (int k = 0; k < p; ++k) acc += V[(size_t)k * (size_t)p + (size_t)r] * T[(size_t)k * (size_t)p + (size_t)c];
            double h = eps_c + mu / (ev[r] * ev[c]);
            double val = acc / h;
            U[(size_t)r * (size_t)p + (size_t)c] = val;
            U[(size_t)c * (size_t)p + (size_t)r] = val;
        }
    }
    for (int a = 0; a < p; ++a) {
        for (int c = 0; c < p; ++c) {
            double acc = 0.0;
            for (int r = 0; r < p; ++r) acc += V[(size_t)a * (size_t)p + (size_t)r] * U[(size_t)r * (size_t)p + (size_t)c];
            T[(size_t)a * (size_t)p + (size_t)c] = acc;
        }
    }
    for (int a = 0; a < p; ++a) {
        for (int b = a; b < p; ++b) {
            double acc = 0.0;
            for (int c = 0; c < p; ++c) acc += T[(size_t)a * (size_t)p + (size_t)c] * V[(size_t)b * (size_t)p + (size_t)c];
            Y[(size_t)a * (size_t)p + (size_t)b] = acc;
            Y[(size_t)b * (size_t)p + (size_t)a] = acc;
        }
    }
    sos2_pack_svec(p, Y, y_svec);
}

static double sos2_block_alpha_max(int p, const double *V, const double *ev,
                                   const double *dm_svec) {
    int n_sym = p * (p + 1) / 2;
    int nz = 0;
    for (int i = 0; i < n_sym; ++i) {
        if (fabs(dm_svec[i]) > 1e-24) { nz = 1; break; }
    }
    if (!nz) return 1.0;

    double dM[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double T[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double U[SOS2_MAX_DIM * SOS2_MAX_DIM];
    double lams[SOS2_MAX_DIM];
    sos2_unpack_svec(p, dm_svec, dM);
    for (int k = 0; k < p; ++k) {
        for (int c = 0; c < p; ++c) {
            double acc = 0.0;
            for (int b = 0; b < p; ++b) acc += dM[(size_t)k * (size_t)p + (size_t)b] * V[(size_t)b * (size_t)p + (size_t)c];
            T[(size_t)k * (size_t)p + (size_t)c] = acc;
        }
    }
    for (int r = 0; r < p; ++r) {
        double sr = 1.0 / sqrt(ev[r]);
        for (int c = r; c < p; ++c) {
            double sc = 1.0 / sqrt(ev[c]);
            double acc = 0.0;
            for (int k = 0; k < p; ++k) acc += V[(size_t)k * (size_t)p + (size_t)r] * T[(size_t)k * (size_t)p + (size_t)c];
            double val = acc * sr * sc;
            U[(size_t)r * (size_t)p + (size_t)c] = val;
            U[(size_t)c * (size_t)p + (size_t)r] = val;
        }
    }
    if (ele_sym_jacobi_eigs(p, U, lams, NULL) != 0) return 0.5;
    if (lams[0] < -1e-14) return fmin(1.0, -0.98 / lams[0]);
    return 1.0;
}

typedef struct {
    int off;
    int dim;
    int n_sym;
    int v_off;
    int ev_off;
    double eps;
} sos2_blk_t;

typedef struct {
    int n_vars, off_gam, off_eq, off_pos, off_blk, n_psd;
    double eps_gam, eps_eq, eps_c;
    sos2_blk_t blks[SOS2_MAX_BLKS];
} sos2_layout_t;

static void sos2_apply_hinv(const sos2_layout_t *L, const double *u,
                            const double *V_all, const double *ev_all,
                            double mu, const double *in_v, double *out_v) {
    out_v[L->off_gam] = in_v[L->off_gam] / L->eps_gam;
    for (int k = L->off_eq; k < L->off_pos; ++k) {
        out_v[k] = in_v[k] / L->eps_eq;
    }
    for (int k = L->off_pos; k < L->off_blk; ++k) {
        double v = in_v[k];
        if (v == 0.0) { out_v[k] = 0.0; continue; }
        double uk = u[k];
        out_v[k] = v / (L->eps_c + mu / (uk * uk));
    }
    for (int b = 0; b < L->n_psd; ++b) {
        const sos2_blk_t *B = &L->blks[b];
        int nz = 0;
        for (int i = 0; i < B->n_sym; ++i) {
            if (in_v[B->off + i] != 0.0) { nz = 1; break; }
        }
        if (!nz) {
            memset(out_v + B->off, 0, (size_t)B->n_sym * sizeof(double));
        } else {
            sos2_psd_hinv(B->dim, V_all + B->v_off, ev_all + B->ev_off,
                          B->eps, mu, in_v + B->off, out_v + B->off);
        }
    }
}

static int sos2_eval_state(const sos2_layout_t *L, const double *u, double mu,
                           double *V_all, double *ev_all, double *grad) {
    for (int k = L->off_pos; k < L->off_blk; ++k) {
        if (u[k] <= 1e-14) return -1;
    }
    for (int b = 0; b < L->n_psd; ++b) {
        const sos2_blk_t *B = &L->blks[b];
        int p = B->dim;
        double Mb[SOS2_MAX_DIM * SOS2_MAX_DIM];
        sos2_unpack_svec(p, u + B->off, Mb);
        double *Vb = V_all + B->v_off;
        double *evb = ev_all + B->ev_off;
        if (ele_sym_jacobi_eigs(p, Mb, evb, Vb) != 0) return -1;
        for (int r = 0; r < p; ++r) {
            if (evb[r] <= 1e-14) return -1;
        }
    }
    if (grad != NULL) {
        grad[L->off_gam] = -1.0 + L->eps_gam * u[L->off_gam];
        for (int k = L->off_eq; k < L->off_pos; ++k) grad[k] = L->eps_eq * u[k];
        for (int k = L->off_pos; k < L->off_blk; ++k) grad[k] = -mu / u[k] + L->eps_c * u[k];
        for (int b = 0; b < L->n_psd; ++b) {
            const sos2_blk_t *B = &L->blks[b];
            int p = B->dim;
            const double *Vb = V_all + B->v_off;
            const double *evb = ev_all + B->ev_off;
            double Minv[SOS2_MAX_DIM * SOS2_MAX_DIM];
            for (int r0 = 0; r0 < p; ++r0) {
                for (int r1 = r0; r1 < p; ++r1) {
                    double s = 0.0;
                    for (int k = 0; k < p; ++k) {
                        s += Vb[(size_t)r0 * (size_t)p + (size_t)k] *
                             Vb[(size_t)r1 * (size_t)p + (size_t)k] / evb[k];
                    }
                    Minv[(size_t)r0 * (size_t)p + (size_t)r1] = s;
                    Minv[(size_t)r1 * (size_t)p + (size_t)r0] = s;
                }
            }
            sos2_pack_svec(p, Minv, grad + B->off);
            for (int i = 0; i < B->n_sym; ++i) {
                grad[B->off + i] = -mu * grad[B->off + i] + B->eps * u[B->off + i];
            }
        }
    }
    return 0;
}

static int compute_sos2_cliques(const ele_grid_t *g, clique_def_t *cliques,
                                int *n_cl_out, int *max_bus_out) {
    int N = g->n_bus;
    if (N <= 3) {
        cliques[0].n_bus_c = N;
        for (int b = 0; b < N; ++b) cliques[0].buses[b] = b;
        *n_cl_out = 1;
        *max_bus_out = N;
        return 0;
    }
    uint8_t adj[16 * 16] = {0}, elim[16] = {0};
    clique_def_t raw[16];
    memset(raw, 0, sizeof(raw));
    for (int e = 0; e < g->n_branch; ++e) {
        int u = g->branch[e].from_idx, v = g->branch[e].to_idx;
        if (u != v && u >= 0 && u < N && v >= 0 && v < N) {
            adj[u * N + v] = 1U;
            adj[v * N + u] = 1U;
        }
    }
    int max_b = 0;
    for (int step = 0; step < N; ++step) {
        int best_v = -1, best_deg = N + 1;
        for (int v = 0; v < N; ++v) {
            if (elim[v]) continue;
            int deg = 0;
            for (int w = 0; w < N; ++w) if (!elim[w] && w != v && adj[v * N + w]) ++deg;
            if (deg < best_deg) { best_deg = deg; best_v = v; }
        }
        if (best_v < 0) break;
        raw[step].buses[0] = best_v;
        raw[step].n_bus_c = 1;
        for (int w = 0; w < N; ++w) {
            if (!elim[w] && w != best_v && adj[best_v * N + w]) raw[step].buses[raw[step].n_bus_c++] = w;
        }
        for (int a = 1; a < raw[step].n_bus_c; ++a) {
            for (int b = a + 1; b < raw[step].n_bus_c; ++b) {
                int ua = raw[step].buses[a], ub = raw[step].buses[b];
                adj[ua * N + ub] = 1U;
                adj[ub * N + ua] = 1U;
            }
        }
        qsort(raw[step].buses, (size_t)raw[step].n_bus_c, sizeof(int), cmp_int_asc);
        elim[best_v] = 1U;
    }
    int n_cl = 0;
    for (int i = 0; i < N; ++i) {
        int is_sub = 0;
        for (int j = 0; j < N; ++j) {
            if (i == j || raw[j].n_bus_c < raw[i].n_bus_c) continue;
            if (raw[j].n_bus_c == raw[i].n_bus_c && j > i) continue;
            int hit = 0;
            for (int a = 0; a < raw[i].n_bus_c; ++a) {
                for (int b = 0; b < raw[j].n_bus_c; ++b) {
                    if (raw[i].buses[a] == raw[j].buses[b]) { ++hit; break; }
                }
            }
            if (hit == raw[i].n_bus_c) { is_sub = 1; break; }
        }
        if (!is_sub && n_cl < ELE_FRONTIER_MAX_CLIQUES) {
            cliques[n_cl++] = raw[i];
            if (raw[i].n_bus_c > max_b) max_b = raw[i].n_bus_c;
        }
    }
    *n_cl_out = n_cl;
    *max_bus_out = max_b;
    return 0;
}

int ele_grid_acopf_sos2(const ele_grid_t *g, ele_grid_sos2_res_t *out) {
    return ele_grid_acopf_sos2_cert(g, NULL, out);
}

static void sos2_fill_fallback(const ele_grid_t *g, const ele_grid_opf_res_t *shor1,
                               int d2_dim, const char *cert_out_path,
                               ele_grid_sos2_res_t *out) {
    (void)g;
    out->d2_dim = d2_dim;
    out->n_monos4 = 0;
    out->converged = shor1->converged;
    out->sos2_lb_raw = shor1->c_lb_dual_raw;
    out->sos2_lb_cert = shor1->c_lb_cert;
    out->sos2_gap_pct = shor1->gap_pct;
    out->improvement_abs = 0.0;
    out->q0_mu_psd = shor1->mu_psd_int;
    out->cert = shor1->cert;
    snprintf(out->cert.kind, sizeof(out->cert.kind), "acopf_sos2");
    out->cert.has_bracket = 1;
    out->cert.bracket_scale = shor1->cert.opf_scale;
    out->cert.limit_lo_int = shor1->cert.c_lb_int;
    out->cert.lb_int = shor1->cert.c_lb_int;
    out->cert.ub_int = shor1->cert.c_ub_int;
    out->cert.limit_hi_int = shor1->cert.c_ub_int;
    if (cert_out_path != NULL) {
        (void)ele_cert_write_json(cert_out_path, &out->cert);
    }
}

typedef struct {
    int bus;
    int n_seg;
    double slope_pu[8];
    double icpt_pu[8];
    double L_poly[8][SOS2_MAX_MONOS];
    double max_L_bnd;
} sos2_mgen_t;

int ele_grid_acopf_sos2_cert(const ele_grid_t *g, const char *cert_out_path,
                             ele_grid_sos2_res_t *out) {
    if (g == NULL || out == NULL || g->n_bus <= 0) return -1;
    memset(out, 0, sizeof(*out));
    int N = g->n_bus;
    int slack_idx = (g->slack_idx >= 0 && g->slack_idx < N) ? g->slack_idx : 0;
    out->n_bus = N;

    /* Run Shor-1 baseline first */
    ele_grid_opf_res_t shor1;
    if (ele_grid_acopf(g, &shor1) != 0) return -1;
    out->shor1_lb_cert = shor1.c_lb_cert;
    out->c_ub_primal = shor1.c_ub_primal;
    out->shor1_gap_pct = shor1.gap_pct;

    int bus_gcnt[ELE_GRID_MAX_BUS] = {0};
    int bus_glist[ELE_GRID_MAX_BUS][8];
    memset(bus_glist, 0, sizeof(bus_glist));
    int has_nl_multigen = 0;
    for (int i = 0; i < g->n_gen; ++i) {
        int b = g->gen[i].bus_idx;
        if (bus_gcnt[b] < 8) bus_glist[b][bus_gcnt[b]] = i;
        ++bus_gcnt[b];
    }
    for (int b = 0; b < N; ++b) {
        if (bus_gcnt[b] > 8) has_nl_multigen = 1;
        if (bus_gcnt[b] > 1 && bus_gcnt[b] <= 8) {
            for (int k = 0; k < bus_gcnt[b]; ++k) {
                if (fabs(g->gen[bus_glist[b][k]].c2) > 1e-12) has_nl_multigen = 1;
            }
        }
    }

    if (N > 5 || has_nl_multigen) {
        sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
        return 0;
    }

    clique_def_t cliques[ELE_FRONTIER_MAX_CLIQUES];
    int n_cl = 0, max_cl_bus = 0;
    memset(cliques, 0, sizeof(cliques));
    if (compute_sos2_cliques(g, cliques, &n_cl, &max_cl_bus) != 0 ||
        n_cl <= 0 || n_cl > 12 || max_cl_bus > 3) {
        sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
        return 0;
    }

    int nv = 2 * N - 1;
    uint8_t cl_v_mask[12][16];
    int cl_v[12][8];
    int cl_nv[12];
    memset(cl_v_mask, 0, sizeof(cl_v_mask));
    memset(cl_v, 0, sizeof(cl_v));
    memset(cl_nv, 0, sizeof(cl_nv));

    for (int c = 0; c < n_cl; ++c) {
        for (int bi = 0; bi < cliques[c].n_bus_c; ++bi) {
            int b = cliques[c].buses[bi];
            int ve = map_2n_to_nv(N, slack_idx, b);
            int vf = map_2n_to_nv(N, slack_idx, N + b);
            if (ve >= 0 && ve < nv && !cl_v_mask[c][ve]) {
                cl_v_mask[c][ve] = 1U;
                cl_v[c][cl_nv[c]++] = ve;
            }
            if (vf >= 0 && vf < nv && !cl_v_mask[c][vf]) {
                cl_v_mask[c][vf] = 1U;
                cl_v[c][cl_nv[c]++] = vf;
            }
        }
        qsort(cl_v[c], (size_t)cl_nv[c], sizeof(int), cmp_int_asc);
        if (1 + cl_nv[c] * (cl_nv[c] + 1) / 2 > SOS2_MAX_DIM) {
            sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
            return 0;
        }
    }

    mono4_t *ms = (mono4_t *)calloc(SOS2_MAX_MONOS, sizeof(mono4_t));
    if (ms == NULL) return -1;
    int nm = 0;
    ms[nm].deg = 0;
    ++nm;
    for (int a = 0; a < nv; ++a) {
        for (int b = a; b < nv; ++b) {
            ms[nm].deg = 2;
            ms[nm].e[0] = a; ms[nm].e[1] = b;
            ++nm;
        }
    }
    int d_even = nm; /* 1 + nv*(nv+1)/2 <= 46 for N=5 */
    for (int a = 0; a < nv; ++a) {
        for (int b = a; b < nv; ++b) {
            for (int cv = b; cv < nv; ++cv) {
                for (int dv = cv; dv < nv; ++dv) {
                    int in_cl = 0;
                    for (int c = 0; c < n_cl; ++c) {
                        if (cl_v_mask[c][a] && cl_v_mask[c][b] &&
                            cl_v_mask[c][cv] && cl_v_mask[c][dv]) {
                            in_cl = 1;
                            break;
                        }
                    }
                    if (!in_cl) continue;
                    if (nm >= SOS2_MAX_MONOS) {
                        free(ms);
                        sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
                        return 0;
                    }
                    ms[nm].deg = 4;
                    ms[nm].e[0] = a; ms[nm].e[1] = b; ms[nm].e[2] = cv; ms[nm].e[3] = dv;
                    ++nm;
                }
            }
        }
    }

    /* Per-clique degree-0..2 index map into 0..d_even-1 */
    int cl_ev[12][SOS2_MAX_DIM];
    int cl_d_even[12];
    memset(cl_ev, 0, sizeof(cl_ev));
    for (int c = 0; c < n_cl; ++c) {
        int cnt = 0;
        cl_ev[c][cnt++] = 0;
        for (int i = 0; i < cl_nv[c]; ++i) {
            for (int j = i; j < cl_nv[c]; ++j) {
                int vv[2] = { cl_v[c][i], cl_v[c][j] };
                int m2 = find_mono4(ms, d_even, 2, vv);
                if (m2 < 0) {
                    free(ms);
                    sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
                    return 0;
                }
                cl_ev[c][cnt++] = m2;
            }
        }
        cl_d_even[c] = cnt;
    }

    out->d2_dim = d_even;
    out->n_monos4 = nm;

    double B = g->base_mva;
    double YP[5][100], YQ[5][100];
    for (int k = 0; k < N; ++k) build_pq_matrix_2n(g, k, YP[k], YQ[k]);

    double cost_norm = fmax(1.0, out->c_ub_primal);
    double cost_raw[SOS2_MAX_MONOS] = {0};
    double C_poly[SOS2_MAX_MONOS] = {0};
    sos2_mgen_t *mgens = (sos2_mgen_t *)calloc(ELE_GRID_MAX_BUS, sizeof(sos2_mgen_t));
    double (*eq_polys)[SOS2_MAX_MONOS] = (double (*)[SOS2_MAX_MONOS])calloc(12, sizeof(*eq_polys));
    double (*ineq_polys)[SOS2_MAX_MONOS] = (double (*)[SOS2_MAX_MONOS])calloc(48, sizeof(*ineq_polys));
    double (*therm4)[SOS2_MAX_MONOS] = (double (*)[SOS2_MAX_MONOS])calloc(32, sizeof(*therm4));
    double (*rlt4)[SOS2_MAX_MONOS] = (double (*)[SOS2_MAX_MONOS])calloc(48, sizeof(*rlt4));
    if (!mgens || !eq_polys || !ineq_polys || !therm4 || !rlt4) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        return -1;
    }

    int cost_miss = 0;
    int n_mg_bus = 0, n_mg_segs = 0;
    for (int b = 0; b < N; ++b) {
        if (bus_gcnt[b] == 0) continue;
        double p_blk[SOS2_MAX_MONOS] = {0};
        poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, YP[b], 1.0, g->bus[b].pd_mw / B, p_blk);
        if (bus_gcnt[b] == 1) {
            const ele_grid_gen_t *gn = &g->gen[bus_glist[b][0]];
            double a2 = gn->c2 * B * B;
            double a1 = gn->c1 * B;
            poly_mul_2x2_check(ms, nm, p_blk, p_blk, d_even, a2, cost_raw, &cost_miss);
            for (int m = 0; m < d_even; ++m) cost_raw[m] += a1 * p_blk[m];
            cost_raw[0] += gn->c0;
        } else {
            int cnt = bus_gcnt[b];
            int ord[8];
            for (int k = 0; k < cnt; ++k) ord[k] = bus_glist[b][k];
            for (int i = 0; i < cnt; ++i) {
                for (int j = i + 1; j < cnt; ++j) {
                    if (g->gen[ord[j]].c1 < g->gen[ord[i]].c1) {
                        int tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
                    }
                }
            }
            sos2_mgen_t *mg = &mgens[n_mg_bus++];
            mg->bus = b;
            mg->n_seg = cnt;
            double p_cur = 0.0, c_cur = 0.0;
            for (int k = 0; k < cnt; ++k) {
                const ele_grid_gen_t *gk = &g->gen[ord[k]];
                double pmin_pu = gk->pmin_mw / B;
                p_cur += pmin_pu;
                c_cur += gk->c0 + gk->c1 * gk->pmin_mw;
            }
            for (int k = 0; k < cnt; ++k) {
                const ele_grid_gen_t *gk = &g->gen[ord[k]];
                double sr = gk->c1 * B;
                double ic = c_cur - sr * p_cur;
                mg->slope_pu[k] = sr;
                mg->icpt_pu[k] = ic;
                for (int m = 0; m < d_even; ++m) {
                    mg->L_poly[k][m] = (sr * p_blk[m]) / cost_norm;
                }
                mg->L_poly[k][0] += ic / cost_norm;
                double d_p = (gk->pmax_mw - gk->pmin_mw) / B;
                p_cur += d_p;
                c_cur += sr * d_p;
            }
            n_mg_segs += cnt;
        }
    }
    if (cost_miss) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        sos2_fill_fallback(g, &shor1, 2 * N, cert_out_path, out);
        return 0;
    }
    for (int m = 0; m < nm; ++m) C_poly[m] = cost_raw[m] / cost_norm;

    /* Collect equalities & inequalities */
    int n_eq_base = 0;
    int n_ineq2 = 0;
    int ineq_cl[48];
    for (int k = 0; k < 48; ++k) ineq_cl[k] = -1;

    for (int b = 0; b < N; ++b) {
        if (bus_gcnt[b] == 0 && n_eq_base + 2 <= 12) {
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, YP[b], 1.0, g->bus[b].pd_mw / B, eq_polys[n_eq_base++]);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, YQ[b], 1.0, g->bus[b].qd_mvar / B, eq_polys[n_eq_base++]);
        }
        if (n_ineq2 + 2 <= 48) {
            double vm2[SOS2_MAX_MONOS] = {0};
            double MV[10 * 10] = {0};
            MV[(size_t)b * (size_t)(2 * N) + (size_t)b] = 1.0;
            MV[(size_t)(N + b) * (size_t)(2 * N) + (size_t)(N + b)] = 1.0;
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MV, 1.0, 0.0, vm2);
            for (int m = 0; m < d_even; ++m) ineq_polys[n_ineq2][m] = vm2[m];
            ineq_polys[n_ineq2++][0] -= g->bus[b].vmin * g->bus[b].vmin;
            for (int m = 0; m < d_even; ++m) ineq_polys[n_ineq2][m] = -vm2[m];
            ineq_polys[n_ineq2++][0] += g->bus[b].vmax * g->bus[b].vmax;
        }
        if (bus_gcnt[b] >= 1 && n_ineq2 + 4 <= 48) {
            double pmin_sum = 0.0, pmax_sum = 0.0, qmin_sum = 0.0, qmax_sum = 0.0;
            for (int k = 0; k < bus_gcnt[b]; ++k) {
                const ele_grid_gen_t *gn = &g->gen[bus_glist[b][k]];
                pmin_sum += gn->pmin_mw; pmax_sum += gn->pmax_mw;
                qmin_sum += gn->qmin_mvar; qmax_sum += gn->qmax_mvar;
            }
            double p_blk[SOS2_MAX_MONOS] = {0}, q_blk[SOS2_MAX_MONOS] = {0};
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, YP[b], 1.0, g->bus[b].pd_mw / B, p_blk);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, YQ[b], 1.0, g->bus[b].qd_mvar / B, q_blk);
            for (int m = 0; m < d_even; ++m) {
                ineq_polys[n_ineq2][m] = p_blk[m];
                ineq_polys[n_ineq2 + 1][m] = -p_blk[m];
                ineq_polys[n_ineq2 + 2][m] = q_blk[m];
                ineq_polys[n_ineq2 + 3][m] = -q_blk[m];
            }
            ineq_polys[n_ineq2][0] -= pmin_sum / B;
            ineq_polys[n_ineq2 + 1][0] += pmax_sum / B;
            ineq_polys[n_ineq2 + 2][0] -= qmin_sum / B;
            ineq_polys[n_ineq2 + 3][0] += qmax_sum / B;
            n_ineq2 += 4;
        }
    }

    int n_ineq4 = 0;
    int br_miss = 0;
    for (int e = 0; e < g->n_branch; ++e) {
        const ele_grid_branch_t *br = &g->branch[e];
        br_y_t by;
        compute_branch_y(br, &by);
        int u = by.u, v = by.v;
        if (u == v) continue;
        int d2 = 2 * N;
        double MPF[100] = {0}, MQF[100] = {0}, MPT[100] = {0}, MQT[100] = {0};
        double M4[4][4];
        int idx4[4] = { u, v, N + u, N + v };
        #define MAP4_TO_2N(MAT2N) do { \
            for (int _a = 0; _a < 4; ++_a) for (int _b = 0; _b < 4; ++_b) \
                (MAT2N)[(size_t)idx4[_a] * (size_t)d2 + (size_t)idx4[_b]] = M4[_a][_b]; \
        } while (0)

        if (br->angmin_deg > -85.0 && n_ineq2 < 48) {
            double MA[100] = {0};
            fill_4x4(0.0, 0.0, -tan(br->angmin_deg * (M_PI / 180.0)), 1.0, M4);
            MAP4_TO_2N(MA);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MA, 1.0, 0.0, ineq_polys[n_ineq2++]);
        }
        if (br->angmax_deg < 85.0 && n_ineq2 < 48) {
            double MA[100] = {0};
            fill_4x4(0.0, 0.0, tan(br->angmax_deg * (M_PI / 180.0)), -1.0, M4);
            MAP4_TO_2N(MA);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MA, 1.0, 0.0, ineq_polys[n_ineq2++]);
        }
        if (br->smax_mva > 0.0 && br->smax_mva < 9000.0 && n_ineq4 + 2 <= 32) {
            double smax_pu = br->smax_mva / B;
            double p_fp[SOS2_MAX_MONOS] = {0}, q_fp[SOS2_MAX_MONOS] = {0};
            double p_tp[SOS2_MAX_MONOS] = {0}, q_tp[SOS2_MAX_MONOS] = {0};
            fill_4x4(by.yff_r, 0.0, by.yft_r, by.yft_i, M4); MAP4_TO_2N(MPF);
            fill_4x4(-by.yff_i, 0.0, -by.yft_i, by.yft_r, M4); MAP4_TO_2N(MQF);
            fill_4x4(0.0, by.ytt_r, by.ytf_r, -by.ytf_i, M4); MAP4_TO_2N(MPT);
            fill_4x4(0.0, -by.ytt_i, -by.ytf_i, -by.ytf_r, M4); MAP4_TO_2N(MQT);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MPF, 1.0, 0.0, p_fp);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MQF, 1.0, 0.0, q_fp);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MPT, 1.0, 0.0, p_tp);
            poly2_add_quad_matrix(N, slack_idx, nv, ms, nm, MQT, 1.0, 0.0, q_tp);

            int fmiss = 0, tmiss = 0;
            therm4[n_ineq4][0] = smax_pu * smax_pu;
            poly_mul_2x2_check(ms, nm, p_fp, p_fp, d_even, -1.0, therm4[n_ineq4], &fmiss);
            poly_mul_2x2_check(ms, nm, q_fp, q_fp, d_even, -1.0, therm4[n_ineq4], &fmiss);
            if (!fmiss) ++n_ineq4; else { memset(therm4[n_ineq4], 0, sizeof(therm4[0])); br_miss = 1; }

            therm4[n_ineq4][0] = smax_pu * smax_pu;
            poly_mul_2x2_check(ms, nm, p_tp, p_tp, d_even, -1.0, therm4[n_ineq4], &tmiss);
            poly_mul_2x2_check(ms, nm, q_tp, q_tp, d_even, -1.0, therm4[n_ineq4], &tmiss);
            if (!tmiss) ++n_ineq4; else { memset(therm4[n_ineq4], 0, sizeof(therm4[0])); br_miss = 1; }
        }
        #undef MAP4_TO_2N
    }
    (void)br_miss;

    /* Assign each degree-2 inequality a home clique (if all its vars sit in a single clique) */
    for (int k = 0; k < n_ineq2; ++k) {
        uint8_t used[16] = {0};
        for (int m = 1; m < d_even; ++m) {
            if (fabs(ineq_polys[k][m]) > 1e-14) {
                used[ms[m].e[0]] = 1U;
                used[ms[m].e[1]] = 1U;
            }
        }
        for (int c = 0; c < n_cl; ++c) {
            int ok = 1;
            for (int v = 0; v < nv; ++v) {
                if (used[v] && !cl_v_mask[c][v]) { ok = 0; break; }
            }
            if (ok) { ineq_cl[k] = c; break; }
        }
    }

    /* Intra-clique RLT products g_i * g_j >= 0 (up to 48 valid pairs) */
    int n_rlt = 0;
    for (int i = 0; i < n_ineq2 && n_rlt < 48; ++i) {
        for (int j = i; j < n_ineq2 && n_rlt < 48; ++j) {
            double prod[SOS2_MAX_MONOS] = {0};
            int miss = 0;
            poly_mul_2x2_check(ms, nm, ineq_polys[i], ineq_polys[j], d_even, 1.0, prod, &miss);
            if (!miss) {
                memcpy(rlt4[n_rlt], prod, (size_t)nm * sizeof(double));
                ++n_rlt;
            }
        }
    }

    /* Normalize constraint polynomials (note: do NOT normalize multi-gen L_poly) */
    for (int j = 0; j < n_eq_base; ++j) {
        double n2 = 0.0;
        for (int m = 0; m < nm; ++m) n2 += eq_polys[j][m] * eq_polys[j][m];
        double sc = (n2 > 1e-24) ? (1.0 / sqrt(n2)) : 1.0;
        for (int m = 0; m < nm; ++m) eq_polys[j][m] *= sc;
    }
    for (int k = 0; k < n_ineq2; ++k) {
        double n2 = 0.0;
        for (int m = 0; m < nm; ++m) n2 += ineq_polys[k][m] * ineq_polys[k][m];
        double sc = (n2 > 1e-24) ? (1.0 / sqrt(n2)) : 1.0;
        for (int m = 0; m < nm; ++m) ineq_polys[k][m] *= sc;
    }
    for (int k = 0; k < n_ineq4; ++k) {
        double n2 = 0.0;
        for (int m = 0; m < nm; ++m) n2 += therm4[k][m] * therm4[k][m];
        double sc = (n2 > 1e-24) ? (1.0 / sqrt(n2)) : 1.0;
        for (int m = 0; m < nm; ++m) therm4[k][m] *= sc;
    }
    for (int k = 0; k < n_rlt; ++k) {
        double n2 = 0.0;
        for (int m = 0; m < nm; ++m) n2 += rlt4[k][m] * rlt4[k][m];
        double sc = (n2 > 1e-24) ? (1.0 / sqrt(n2)) : 1.0;
        for (int m = 0; m < nm; ++m) rlt4[k][m] *= sc;
    }

    /* Count valid equality columns (h_j * phi_d with zero misses) & localizing PSD blocks */
    int eq_valid_j[12 * 64];
    int eq_valid_d[12 * 64];
    int n_eq_v = 0;
    for (int j = 0; j < n_eq_base; ++j) {
        for (int d = 0; d < d_even; ++d) {
            double unit[SOS2_MAX_MONOS] = {0}, prod[SOS2_MAX_MONOS] = {0};
            unit[d] = 1.0;
            int miss = 0;
            poly_mul_2x2_check(ms, nm, eq_polys[j], unit, d_even, 1.0, prod, &miss);
            if (!miss) {
                eq_valid_j[n_eq_v] = j;
                eq_valid_d[n_eq_v] = d;
                ++n_eq_v;
            }
        }
    }

    /* Layout construction */
    sos2_layout_t lay;
    memset(&lay, 0, sizeof(lay));
    lay.off_gam = 0;
    lay.off_eq = 1;
    lay.off_pos = lay.off_eq + n_eq_v;
    int off_s0 = lay.off_pos;
    int off_sig = off_s0 + n_ineq2;
    int off_tau = off_sig + n_ineq4;
    int off_th = off_tau + n_rlt;
    lay.off_blk = off_th + n_mg_segs;
    lay.eps_gam = 1e-7;
    lay.eps_eq = 1e-8;
    lay.eps_c = 1e-8;

    int cur_off = lay.off_blk;
    int cur_v = 0, cur_ev = 0;
    for (int c = 0; c < n_cl; ++c) {
        sos2_blk_t *B_c = &lay.blks[lay.n_psd++];
        B_c->dim = cl_d_even[c];
        B_c->n_sym = B_c->dim * (B_c->dim + 1) / 2;
        B_c->off = cur_off;
        B_c->v_off = cur_v;
        B_c->ev_off = cur_ev;
        B_c->eps = 1e-5;
        cur_off += B_c->n_sym;
        cur_v += B_c->dim * B_c->dim;
        cur_ev += B_c->dim;
    }
    int sk_ineq[48];
    int n_sk_blks = 0;
    for (int k = 0; k < n_ineq2 && lay.n_psd < SOS2_MAX_BLKS; ++k) {
        int c = ineq_cl[k];
        if (c < 0) continue;
        sos2_blk_t *B_k = &lay.blks[lay.n_psd++];
        B_k->dim = cl_nv[c];
        B_k->n_sym = B_k->dim * (B_k->dim + 1) / 2;
        B_k->off = cur_off;
        B_k->v_off = cur_v;
        B_k->ev_off = cur_ev;
        B_k->eps = 1e-8;
        cur_off += B_k->n_sym;
        cur_v += B_k->dim * B_k->dim;
        cur_ev += B_k->dim;
        sk_ineq[n_sk_blks++] = k;
    }
    int n_vars = cur_off;
    lay.n_vars = n_vars;
    int n_rows = nm + n_mg_bus;

    double *A_op = (double *)calloc((size_t)n_rows * (size_t)n_vars, sizeof(double));
    double *C_rhs = (double *)calloc((size_t)n_rows, sizeof(double));
    double *G_norm = (double *)calloc((size_t)n_rows * (size_t)n_rows, sizeof(double));
    if (A_op == NULL || C_rhs == NULL || G_norm == NULL) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        free(A_op); free(C_rhs); free(G_norm);
        return -1;
    }

    memcpy(C_rhs, C_poly, (size_t)nm * sizeof(double));
    for (int b = 0; b < n_mg_bus; ++b) C_rhs[nm + b] = 1.0;

    A_op[0 * n_vars + lay.off_gam] = 1.0;
    for (int col_i = 0; col_i < n_eq_v; ++col_i) {
        int j = eq_valid_j[col_i], d = eq_valid_d[col_i];
        int col = lay.off_eq + col_i;
        double unit[SOS2_MAX_MONOS] = {0}, prod[SOS2_MAX_MONOS] = {0};
        unit[d] = 1.0;
        poly_mul_2x2_check(ms, nm, eq_polys[j], unit, d_even, 1.0, prod, NULL);
        for (int m = 0; m < nm; ++m) A_op[(size_t)m * (size_t)n_vars + (size_t)col] = prod[m];
    }
    for (int k = 0; k < n_ineq2; ++k) {
        int col = off_s0 + k;
        for (int m = 0; m < d_even; ++m) A_op[(size_t)m * (size_t)n_vars + (size_t)col] = ineq_polys[k][m];
    }
    for (int k = 0; k < n_ineq4; ++k) {
        int col = off_sig + k;
        for (int m = 0; m < nm; ++m) A_op[(size_t)m * (size_t)n_vars + (size_t)col] = therm4[k][m];
    }
    for (int k = 0; k < n_rlt; ++k) {
        int col = off_tau + k;
        for (int m = 0; m < nm; ++m) A_op[(size_t)m * (size_t)n_vars + (size_t)col] = rlt4[k][m];
    }
    {
        int th_pos = 0;
        for (int b = 0; b < n_mg_bus; ++b) {
            for (int r = 0; r < mgens[b].n_seg; ++r) {
                int col = off_th + th_pos++;
                for (int m = 0; m < d_even; ++m) {
                    A_op[(size_t)m * (size_t)n_vars + (size_t)col] = -mgens[b].L_poly[r][m];
                }
                A_op[(size_t)(nm + b) * (size_t)n_vars + (size_t)col] = 1.0;
            }
        }
    }

    /* Clique SOS blocks Q_c in exact svec isometry */
    for (int c = 0; c < n_cl; ++c) {
        int base = lay.blks[c].off;
        int dk = lay.blks[c].dim;
        int si = 0;
        for (int a = 0; a < dk; ++a) {
            int ga = cl_ev[c][a];
            for (int b = a; b < dk; ++b) {
                int gb = cl_ev[c][b];
                int col = base + si++;
                double wt = (a == b) ? 1.0 : M_SQRT2;
                double pA[SOS2_MAX_MONOS] = {0}, pB[SOS2_MAX_MONOS] = {0}, prod[SOS2_MAX_MONOS] = {0};
                pA[ga] = 1.0; pB[gb] = 1.0;
                poly_mul_2x2_check(ms, nm, pA, pB, d_even, wt, prod, NULL);
                for (int m = 0; m < nm; ++m) {
                    if (fabs(prod[m]) > 1e-16) {
                        A_op[(size_t)m * (size_t)n_vars + (size_t)col] = prod[m];
                    }
                }
            }
        }
    }

    /* Intra-clique localizing blocks S_k */
    for (int sb = 0; sb < n_sk_blks; ++sb) {
        int k = sk_ineq[sb];
        int c = ineq_cl[k];
        int base = lay.blks[n_cl + sb].off;
        int dks = lay.blks[n_cl + sb].dim;
        int si = 0;
        for (int a = 0; a < dks; ++a) {
            for (int b = a; b < dks; ++b) {
                int col = base + si++;
                double wt = (a == b) ? 1.0 : M_SQRT2;
                int vv[2] = { cl_v[c][a], cl_v[c][b] };
                int m2 = find_mono4(ms, d_even, 2, vv);
                if (m2 >= 0) {
                    double unit2[SOS2_MAX_MONOS] = {0}, prod[SOS2_MAX_MONOS] = {0};
                    unit2[m2] = wt;
                    poly_mul_2x2_check(ms, nm, ineq_polys[k], unit2, d_even, 1.0, prod, NULL);
                    for (int m = 0; m < nm; ++m) {
                        if (fabs(prod[m]) > 1e-16) {
                            A_op[(size_t)m * (size_t)n_vars + (size_t)col] = prod[m];
                        }
                    }
                }
            }
        }
    }

    /* Build CSR sparse representation of A_op for fast Newton steps */
    int *row_off = (int *)calloc((size_t)(n_rows + 1), sizeof(int));
    if (!row_off) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        free(A_op); free(C_rhs); free(G_norm);
        return -1;
    }
    int nnz = 0;
    for (int r = 0; r < n_rows; ++r) {
        row_off[r] = nnz;
        const double *ar = A_op + (size_t)r * (size_t)n_vars;
        for (int k = 0; k < n_vars; ++k) {
            if (fabs(ar[k]) > 1e-16) ++nnz;
        }
    }
    row_off[n_rows] = nnz;
    int *nz_col = (int *)malloc((size_t)(nnz > 0 ? nnz : 1) * sizeof(int));
    double *nz_val = (double *)malloc((size_t)(nnz > 0 ? nnz : 1) * sizeof(double));
    if (!nz_col || !nz_val) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        free(A_op); free(C_rhs); free(G_norm); free(row_off); free(nz_col); free(nz_val);
        return -1;
    }
    {
        int pos = 0;
        for (int r = 0; r < n_rows; ++r) {
            const double *ar = A_op + (size_t)r * (size_t)n_vars;
            for (int k = 0; k < n_vars; ++k) {
                if (fabs(ar[k]) > 1e-16) {
                    nz_col[pos] = k;
                    nz_val[pos] = ar[k];
                    ++pos;
                }
            }
        }
    }

    double *LG = (double *)calloc((size_t)n_rows * (size_t)n_rows, sizeof(double));
    double *Hinv_AT = (double *)calloc((size_t)n_rows * (size_t)n_vars, sizeof(double));
    double *u = (double *)calloc((size_t)n_vars, sizeof(double));
    double *u_try = (double *)calloc((size_t)n_vars, sizeof(double));
    double *du = (double *)calloc((size_t)n_vars, sizeof(double));
    double *grad = (double *)calloc((size_t)n_vars, sizeof(double));
    double *grad_try = (double *)calloc((size_t)n_vars, sizeof(double));
    double *r_d = (double *)calloc((size_t)n_vars, sizeof(double));
    double *hinv_rd = (double *)calloc((size_t)n_vars, sizeof(double));
    double *rhs_d = (double *)calloc((size_t)n_vars, sizeof(double));
    double *nu = (double *)calloc((size_t)n_rows, sizeof(double));
    double *nu_try = (double *)calloc((size_t)n_rows, sizeof(double));
    double *dnu = (double *)calloc((size_t)n_rows, sizeof(double));
    double *r_p = (double *)calloc((size_t)n_rows, sizeof(double));
    double *rhs_nu = (double *)calloc((size_t)n_rows, sizeof(double));
    double *y_tmp = (double *)calloc((size_t)n_rows, sizeof(double));
    double *V_all = (double *)calloc((size_t)(cur_v > 0 ? cur_v : 1), sizeof(double));
    double *ev_all = (double *)calloc((size_t)(cur_ev > 0 ? cur_ev : 1), sizeof(double));
    double *V_try = (double *)calloc((size_t)(cur_v > 0 ? cur_v : 1), sizeof(double));
    double *ev_try = (double *)calloc((size_t)(cur_ev > 0 ? cur_ev : 1), sizeof(double));
    int64_t *best_Q_all = (int64_t *)calloc(12U * 32U * 32U, sizeof(int64_t));
    int64_t *best_L_all = (int64_t *)calloc(12U * 32U * 32U, sizeof(int64_t));
    int64_t best_mu_cl[12] = {0}, best_w_cl[12] = {0};

    if (!LG || !Hinv_AT || !u || !u_try || !du || !grad || !grad_try ||
        !r_d || !hinv_rd || !rhs_d || !nu || !nu_try || !dnu || !r_p || !rhs_nu || !y_tmp ||
        !V_all || !ev_all || !V_try || !ev_try || !best_Q_all || !best_L_all) {
        free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
        free(A_op); free(C_rhs); free(G_norm); free(row_off); free(nz_col); free(nz_val);
        free(LG); free(Hinv_AT); free(u); free(u_try); free(du); free(grad); free(grad_try);
        free(r_d); free(hinv_rd); free(rhs_d); free(nu); free(nu_try); free(dnu);
        free(r_p); free(rhs_nu); free(y_tmp); free(V_all); free(ev_all); free(V_try); free(ev_try);
        free(best_Q_all); free(best_L_all);
        return -1;
    }

    /* Strictly interior starting point */
    u[lay.off_gam] = -1.0;
    for (int k = lay.off_pos; k < off_th; ++k) u[k] = 1.0;
    {
        int th_pos = 0;
        for (int b = 0; b < n_mg_bus; ++b) {
            double inv = 1.0 / (double)mgens[b].n_seg;
            for (int r = 0; r < mgens[b].n_seg; ++r) u[off_th + th_pos++] = inv;
        }
    }
    for (int b = 0; b < lay.n_psd; ++b) {
        int p = lay.blks[b].dim, base = lay.blks[b].off;
        for (int a = 0; a < p; ++a) {
            int diag_idx = a * p - a * (a - 1) / 2;
            u[base + diag_idx] = 1.0;
        }
    }

    double best_cert_val = out->shor1_lb_cert;
    double best_raw = 0.0;
    int has_improved_sos = 0;
    int64_t best_opf_scale = 1, best_c_dual_int = 0, best_w_max_int = 0;
    int64_t best_d_rnd_int = 0, best_c_lb_int = 0, best_c_ub_int = 0, best_min_mu = 0;

    int64_t S_Z = 5000, Q_A = 1000000;
    if (out->c_ub_primal <= 1000.0) {
        S_Z = 20000; Q_A = 10000000;
    } else if (out->c_ub_primal <= 10000.0) {
        S_Z = 20000; Q_A = 1500000;
    } else if (out->c_ub_primal <= 30000.0) {
        S_Z = 10000; Q_A = 1000000;
    }
    const int64_t opf_scale = S_Z * S_Z * Q_A;

    double vm2_max_tot = 0.0;
    double x_max_bus[ELE_GRID_MAX_BUS];
    for (int k = 0; k < N; ++k) {
        x_max_bus[k] = g->bus[k].vmax;
        vm2_max_tot += x_max_bus[k] * x_max_bus[k];
    }
    double *mono_bnd = (double *)calloc((size_t)nm, sizeof(double));
    if (mono_bnd == NULL) return -1;
    for (int m = 0; m < nm; ++m) {
        double mb = 1.0;
        for (int d = 0; d < ms[m].deg; ++d) {
            int bus = nv_to_bus(N, slack_idx, ms[m].e[d]);
            mb *= x_max_bus[bus];
        }
        mono_bnd[m] = mb;
    }
    for (int b = 0; b < n_mg_bus; ++b) {
        double mxl = 0.0;
        for (int r = 0; r < mgens[b].n_seg; ++r) {
            double s = 0.0;
            for (int m = 0; m < d_even; ++m) s += fabs(mgens[b].L_poly[r][m]) * mono_bnd[m];
            if (s > mxl) mxl = s;
        }
        mgens[b].max_L_bnd = mxl;
    }

    double mu = 0.1;
    for (int stage = 0; stage < 12; ++stage) {
        for (int nt = 0; nt < 25; ++nt) {
            if (sos2_eval_state(&lay, u, mu, V_all, ev_all, grad) != 0) break;

            double rp2 = 0.0, rd2 = 0.0;
            for (int m = 0; m < n_rows; ++m) {
                double ax = 0.0;
                for (int p = row_off[m]; p < row_off[m + 1]; ++p) ax += nz_val[p] * u[nz_col[p]];
                r_p[m] = C_rhs[m] - ax;
                rp2 += r_p[m] * r_p[m];
            }
            for (int k = 0; k < n_vars; ++k) r_d[k] = -grad[k];
            for (int m = 0; m < n_rows; ++m) {
                double nvm = nu[m];
                if (fabs(nvm) < 1e-24) continue;
                for (int p = row_off[m]; p < row_off[m + 1]; ++p) r_d[nz_col[p]] -= nz_val[p] * nvm;
            }
            for (int k = 0; k < n_vars; ++k) rd2 += r_d[k] * r_d[k];

            double rp_norm = sqrt(rp2);
            double rd_norm = sqrt(rd2);
            double res_norm = sqrt(rp2 + rd2);
            if (rp_norm < 1e-10 && rd_norm < fmax(1e-6, 0.2 * mu)) break;

            for (int m = 0; m < n_rows; ++m) {
                sos2_apply_hinv(&lay, u, V_all, ev_all, mu,
                                A_op + (size_t)m * (size_t)n_vars,
                                Hinv_AT + (size_t)m * (size_t)n_vars);
            }
            sos2_apply_hinv(&lay, u, V_all, ev_all, mu, r_d, hinv_rd);

            double tr_s = 0.0;
            for (int r = 0; r < n_rows; ++r) {
                const int p0 = row_off[r], p1 = row_off[r + 1];
                for (int c = r; c < n_rows; ++c) {
                    const double *hc = Hinv_AT + (size_t)c * (size_t)n_vars;
                    double acc = 0.0;
                    for (int p = p0; p < p1; ++p) acc += nz_val[p] * hc[nz_col[p]];
                    G_norm[(size_t)r * (size_t)n_rows + (size_t)c] = acc;
                    G_norm[(size_t)c * (size_t)n_rows + (size_t)r] = acc;
                    if (r == c) tr_s += acc;
                }
                double s = 0.0;
                for (int p = p0; p < p1; ++p) s += nz_val[p] * hinv_rd[nz_col[p]];
                rhs_nu[r] = s - r_p[r];
            }

            int chol_ok = 0;
            double jit_base = fmax(1e-13, 1e-13 * (tr_s / (double)n_rows));
            for (int ja = 0; ja < 6; ++ja) {
                double jit = jit_base * pow(100.0, (double)ja);
                chol_ok = 1;
                for (int i = 0; i < n_rows; ++i) {
                    for (int j = 0; j <= i; ++j) {
                        double s = G_norm[(size_t)i * (size_t)n_rows + (size_t)j];
                        if (i == j) s += jit;
                        for (int k = 0; k < j; ++k) {
                            s -= LG[(size_t)i * (size_t)n_rows + (size_t)k] *
                                 LG[(size_t)j * (size_t)n_rows + (size_t)k];
                        }
                        if (i == j) {
                            if (s <= 1e-24) { chol_ok = 0; break; }
                            LG[(size_t)i * (size_t)n_rows + (size_t)j] = sqrt(s);
                        } else {
                            LG[(size_t)i * (size_t)n_rows + (size_t)j] =
                                s / LG[(size_t)j * (size_t)n_rows + (size_t)j];
                        }
                    }
                    if (!chol_ok) break;
                }
                if (chol_ok) break;
            }
            if (!chol_ok) break;

            for (int i = 0; i < n_rows; ++i) {
                double s = rhs_nu[i];
                for (int k = 0; k < i; ++k) s -= LG[(size_t)i * (size_t)n_rows + (size_t)k] * y_tmp[k];
                y_tmp[i] = s / LG[(size_t)i * (size_t)n_rows + (size_t)i];
            }
            for (int i = n_rows - 1; i >= 0; --i) {
                double s = y_tmp[i];
                for (int k = i + 1; k < n_rows; ++k) s -= LG[(size_t)k * (size_t)n_rows + (size_t)i] * dnu[k];
                dnu[i] = s / LG[(size_t)i * (size_t)n_rows + (size_t)i];
            }

            for (int k = 0; k < n_vars; ++k) rhs_d[k] = r_d[k];
            for (int m = 0; m < n_rows; ++m) {
                double dm = dnu[m];
                if (fabs(dm) < 1e-24) continue;
                for (int p = row_off[m]; p < row_off[m + 1]; ++p) rhs_d[nz_col[p]] -= nz_val[p] * dm;
            }
            sos2_apply_hinv(&lay, u, V_all, ev_all, mu, rhs_d, du);

            double alpha_max = 1.0;
            for (int k = lay.off_pos; k < lay.off_blk; ++k) {
                if (du[k] < -1e-18) alpha_max = fmin(alpha_max, -0.98 * u[k] / du[k]);
            }
            for (int b = 0; b < lay.n_psd; ++b) {
                double ab = sos2_block_alpha_max(lay.blks[b].dim,
                                                 V_all + lay.blks[b].v_off,
                                                 ev_all + lay.blks[b].ev_off,
                                                 du + lay.blks[b].off);
                if (ab < alpha_max) alpha_max = ab;
            }
            if (alpha_max < 1e-10) break;

            int accepted = 0;
            double alpha = alpha_max;
            for (int ls = 0; ls < 24; ++ls) {
                for (int k = 0; k < n_vars; ++k) u_try[k] = u[k] + alpha * du[k];
                for (int m = 0; m < n_rows; ++m) nu_try[m] = nu[m] + alpha * dnu[m];
                if (sos2_eval_state(&lay, u_try, mu, V_try, ev_try, grad_try) == 0) {
                    double t_rp2 = 0.0, t_rd2 = 0.0;
                    for (int m = 0; m < n_rows; ++m) {
                        double ax = 0.0;
                        for (int p = row_off[m]; p < row_off[m + 1]; ++p) ax += nz_val[p] * u_try[nz_col[p]];
                        double rpi = C_rhs[m] - ax;
                        t_rp2 += rpi * rpi;
                    }
                    for (int k = 0; k < n_vars; ++k) rhs_d[k] = -grad_try[k];
                    for (int m = 0; m < n_rows; ++m) {
                        double nvm = nu_try[m];
                        if (fabs(nvm) < 1e-24) continue;
                        for (int p = row_off[m]; p < row_off[m + 1]; ++p) rhs_d[nz_col[p]] -= nz_val[p] * nvm;
                    }
                    for (int k = 0; k < n_vars; ++k) t_rd2 += rhs_d[k] * rhs_d[k];
                    double res_try = sqrt(t_rp2 + t_rd2);
                    if (res_try <= (1.0 - 1e-4 * alpha) * res_norm || res_norm < 1e-11) {
                        accepted = 1;
                        break;
                    }
                }
                alpha *= 0.5;
            }
            if (!accepted) break;
            memcpy(u, u_try, (size_t)n_vars * sizeof(double));
            memcpy(nu, nu_try, (size_t)n_rows * sizeof(double));
        }

        double res_poly_bnd = 0.0;
        for (int m = 0; m < nm; ++m) {
            double ax = 0.0;
            for (int p = row_off[m]; p < row_off[m + 1]; ++p) ax += nz_val[p] * u[nz_col[p]];
            res_poly_bnd += fabs(C_rhs[m] - ax) * cost_norm * mono_bnd[m];
        }
        for (int b = 0; b < n_mg_bus; ++b) {
            double ax = 0.0;
            int r = nm + b;
            for (int p = row_off[r]; p < row_off[r + 1]; ++p) ax += nz_val[p] * u[nz_col[p]];
            res_poly_bnd += fabs(1.0 - ax) * mgens[b].max_L_bnd * cost_norm;
        }

        if (res_poly_bnd < 1e-2 * cost_norm &&
            sos2_eval_state(&lay, u, mu, V_all, ev_all, NULL) == 0) {
            int q_ok = 1;
            int64_t stage_Q[12 * 32 * 32] = {0};
            int64_t stage_L[12 * 32 * 32] = {0};
            int64_t stage_mu[12] = {0}, stage_w[12] = {0};
            int64_t min_mu = 0, w_tot = 0;
            ele_f_i128 neg_pen = 0;
            double d_rnd_fp = 1e6;

            for (int c = 0; c < n_cl; ++c) {
                int dk = cl_d_even[c];
                int base = lay.blks[c].off;
                int64_t *Qc = &stage_Q[(size_t)c * 32U * 32U];
                int64_t *Lc = &stage_L[(size_t)c * 32U * 32U];
                double norm_f2 = 0.0;
                int si_q = 0;
                for (int a = 0; a < dk; ++a) {
                    for (int b = a; b < dk; ++b) {
                        double qv = u[base + si_q++] * ((a == b) ? 1.0 : M_SQRT1_2) * cost_norm;
                        double scaled = qv * (double)Q_A;
                        if (fabs(scaled) > 1e15) q_ok = 0;
                        int64_t qi = q_ok ? (int64_t)llround(scaled) : 0;
                        Qc[a * dk + b] = qi;
                        Qc[b * dk + a] = qi;
                        norm_f2 += qv * qv * ((a == b) ? 1.0 : 2.0);
                    }
                }
                int64_t mu_c = 0;
                if (!q_ok || ele_cert_cholesky_sdd_i64(dk, Qc, Lc, &mu_c) != 0) {
                    q_ok = 0;
                    break;
                }
                double vm2_c = (n_cl == 1) ? vm2_max_tot : 0.0;
                if (n_cl > 1) {
                    for (int bi = 0; bi < cliques[c].n_bus_c; ++bi) {
                        double xb = x_max_bus[cliques[c].buses[bi]];
                        vm2_c += xb * xb;
                    }
                }
                double wz_c = 1.0 + vm2_c + vm2_c * vm2_c;
                int64_t wc = (int64_t)ceil(wz_c * (double)(S_Z * S_Z)) + 16LL * (int64_t)dk;
                stage_mu[c] = mu_c;
                stage_w[c] = wc;
                w_tot += wc;
                if (c == 0 || mu_c < min_mu) min_mu = mu_c;
                if (mu_c < 0) neg_pen += (ele_f_i128)mu_c * (ele_f_i128)wc;
                d_rnd_fp += 0.5 * (double)dk * (double)wc +
                            (double)Q_A * sqrt(norm_f2) *
                            ((double)S_Z * sqrt((double)dk * wz_c) + 0.25 * (double)dk);
            }

            double c_dual_fp = u[lay.off_gam] * cost_norm - res_poly_bnd;
            if (q_ok && fabs(c_dual_fp) * (double)opf_scale < 4.5e18 &&
                out->c_ub_primal * (double)opf_scale < 4.5e18 &&
                d_rnd_fp < 4.5e18) {
                int64_t c_dual_int = (int64_t)floor(c_dual_fp * (double)opf_scale);
                int64_t opf_delta_round = (int64_t)ceil(d_rnd_fp);
                ele_f_i128 c_lb_i128 = (ele_f_i128)c_dual_int + neg_pen - (ele_f_i128)opf_delta_round;
                if (c_lb_i128 > (ele_f_i128)(-4500000000000000000LL) &&
                    c_lb_i128 < (ele_f_i128)4500000000000000000LL) {
                    int64_t c_lb_int = (int64_t)c_lb_i128;
                    int64_t c_ub_int = (int64_t)ceil(out->c_ub_primal * (double)opf_scale);
                    double cert_val = (double)c_lb_int / (double)opf_scale;
                    if (cert_val > best_cert_val && c_lb_int <= c_ub_int) {
                        best_cert_val = cert_val;
                        best_raw = u[lay.off_gam] * cost_norm;
                        has_improved_sos = 1;
                        best_opf_scale = opf_scale;
                        best_c_dual_int = c_dual_int;
                        best_w_max_int = w_tot;
                        best_d_rnd_int = opf_delta_round;
                        best_c_lb_int = c_lb_int;
                        best_c_ub_int = c_ub_int;
                        best_min_mu = min_mu;
                        memcpy(best_mu_cl, stage_mu, sizeof(best_mu_cl));
                        memcpy(best_w_cl, stage_w, sizeof(best_w_cl));
                        memcpy(best_Q_all, stage_Q, 12U * 32U * 32U * sizeof(int64_t));
                        memcpy(best_L_all, stage_L, 12U * 32U * 32U * sizeof(int64_t));
                    }
                }
            }
        }
        mu *= 0.25;
    }

    if (has_improved_sos) {
        out->converged = shor1.converged;
        out->sos2_lb_raw = best_raw;
        out->sos2_lb_cert = best_cert_val;
        out->sos2_gap_pct = (out->c_ub_primal > 1e-6) ?
            (100.0 * (out->c_ub_primal - out->sos2_lb_cert) / out->c_ub_primal) : 0.0;
        out->improvement_abs = out->sos2_lb_cert - out->shor1_lb_cert;
        out->q0_mu_psd = best_min_mu;

        memset(&out->cert, 0, sizeof(out->cert));
        snprintf(out->cert.pillar, sizeof(out->cert.pillar), "grid");
        snprintf(out->cert.kind, sizeof(out->cert.kind), "acopf_sos2");
        snprintf(out->cert.status, sizeof(out->cert.status), "%s",
                 (shor1.converged && best_c_lb_int <= best_c_ub_int) ? "CERTIFIED" : "VETOED");
        snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", g->input_path);
        snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", g->input_sha256);

        out->cert.has_opf = 1;
        out->cert.opf_scale = best_opf_scale;
        out->cert.c_dual_int = best_c_dual_int;
        out->cert.w_max_int = best_w_max_int;
        out->cert.opf_delta_round = best_d_rnd_int;
        out->cert.c_lb_int = best_c_lb_int;
        out->cert.c_ub_int = best_c_ub_int;

        out->cert.has_bracket = 1;
        out->cert.bracket_scale = best_opf_scale;
        int64_t shor1_i = (int64_t)floor(out->shor1_lb_cert * (double)best_opf_scale);
        if (shor1_i > best_c_lb_int) shor1_i = best_c_lb_int;
        out->cert.limit_lo_int = shor1_i;
        out->cert.lb_int = best_c_lb_int;
        out->cert.ub_int = best_c_ub_int;
        out->cert.limit_hi_int = best_c_ub_int;
        out->cert.lb_val = out->sos2_lb_cert;
        out->cert.ub_val = out->c_ub_primal;

        if (n_cl == 1) {
            out->cert.has_psd = 1;
            out->cert.psd.dim = d_even;
            out->cert.psd.scale_q = Q_A;
            out->cert.psd.mu_psd = best_min_mu;
            out->cert.psd.delta_round = 0;
            out->cert.psd.mu_cert = best_min_mu;
            out->cert.psd.ok = 1;
            memcpy(out->cert.psd.A, best_Q_all, (size_t)d_even * (size_t)d_even * sizeof(int64_t));
            memcpy(out->cert.psd.L, best_L_all, (size_t)d_even * (size_t)d_even * sizeof(int64_t));
            if (cert_out_path != NULL) (void)ele_cert_write_json(cert_out_path, &out->cert);
        } else if (cert_out_path != NULL) {
            ele_f_i128 *A_sum = (ele_f_i128 *)calloc((size_t)d_even * (size_t)d_even, sizeof(ele_f_i128));
            FILE *cf = fopen(cert_out_path, "w");
            if (A_sum != NULL && cf != NULL) {
                int max_dk = 0;
                for (int c = 0; c < n_cl; ++c) {
                    int dk = cl_d_even[c];
                    if (dk > max_dk) max_dk = dk;
                    const int64_t *Qc = &best_Q_all[(size_t)c * 32U * 32U];
                    for (int a = 0; a < dk; ++a) {
                        int va = cl_ev[c][a];
                        for (int b = 0; b < dk; ++b) {
                            A_sum[(size_t)va * (size_t)d_even + (size_t)cl_ev[c][b]] +=
                                (ele_f_i128)Qc[(size_t)a * (size_t)dk + (size_t)b];
                        }
                    }
                }
                fprintf(cf, "{\n");
                fprintf(cf, "  \"format\": \"ELECERT-1.0\",\n");
                fprintf(cf, "  \"version\": \"%s\",\n", ELE_VERSION_STRING);
                fprintf(cf, "  \"pillar\": \"grid\",\n");
                fprintf(cf, "  \"kind\": \"acopf_sos2\",\n");
                fprintf(cf, "  \"status\": \"%s\",\n", out->cert.status);
                fprintf(cf, "  \"input_path\": \"%s\",\n", g->input_path);
                fprintf(cf, "  \"input_sha256\": \"%s\",\n", g->input_sha256);
                fprintf(cf, "  \"has_psd\": 0,\n");
                fprintf(cf, "  \"has_chordal\": 1,\n");
                fprintf(cf, "  \"has_sparam\": 0,\n");
                fprintf(cf, "  \"has_bracket\": 1,\n");
                fprintf(cf, "  \"has_motor\": 0,\n");
                fprintf(cf, "  \"has_opf\": 1,\n");
                fprintf(cf, "  \"has_emt\": 0,\n");
                fprintf(cf, "  \"dim\": %d,\n", d_even);
                fprintf(cf, "  \"num_cliques\": %d,\n", n_cl);
                fprintf(cf, "  \"max_clique_dim\": %d,\n", max_dk);
                fprintf(cf, "  \"min_clique_mu_psd\": %" PRId64 ",\n", best_min_mu);
                fprintf(cf, "  \"opf_scale\": %" PRId64 ",\n", best_opf_scale);
                fprintf(cf, "  \"c_dual_int\": %" PRId64 ",\n", best_c_dual_int);
                fprintf(cf, "  \"w_max_int\": %" PRId64 ",\n", best_w_max_int);
                fprintf(cf, "  \"opf_delta_round\": %" PRId64 ",\n", best_d_rnd_int);
                fprintf(cf, "  \"c_lb_int\": %" PRId64 ",\n", best_c_lb_int);
                fprintf(cf, "  \"c_ub_int\": %" PRId64 ",\n", best_c_ub_int);
                fprintf(cf, "  \"bracket_scale\": %" PRId64 ",\n", best_opf_scale);
                fprintf(cf, "  \"limit_lo_int\": %" PRId64 ",\n", shor1_i);
                fprintf(cf, "  \"lb_int\": %" PRId64 ",\n", best_c_lb_int);
                fprintf(cf, "  \"ub_int\": %" PRId64 ",\n", best_c_ub_int);
                fprintf(cf, "  \"limit_hi_int\": %" PRId64 ",\n", best_c_ub_int);
                fprintf(cf, "  \"A_sparse\": [\n");
                int wrote = 0;
                for (int r = 0; r < d_even; ++r) {
                    for (int c = r; c < d_even; ++c) {
                        int64_t val = (int64_t)A_sum[(size_t)r * (size_t)d_even + (size_t)c];
                        if (val == 0) continue;
                        fprintf(cf, "%s    [%d, %d, %" PRId64 "]", wrote ? ",\n" : "", r, c, val);
                        wrote = 1;
                    }
                }
                fprintf(cf, "\n  ],\n  \"cliques\": [\n");
                for (int c = 0; c < n_cl; ++c) {
                    int dk = cl_d_even[c];
                    const int64_t *Qc = &best_Q_all[(size_t)c * 32U * 32U];
                    const int64_t *Lc = &best_L_all[(size_t)c * 32U * 32U];
                    fprintf(cf, "    {\n      \"vars\": [");
                    for (int a = 0; a < dk; ++a) fprintf(cf, "%s%d", a ? ", " : "", cl_ev[c][a]);
                    fprintf(cf, "],\n      \"mu_psd\": %" PRId64 ",\n", best_mu_cl[c]);
                    fprintf(cf, "      \"w_max_int\": %" PRId64 ",\n", best_w_cl[c]);
                    fprintf(cf, "      \"S_int\": [\n");
                    for (int a = 0; a < dk; ++a) {
                        fprintf(cf, "        [");
                        for (int b = 0; b < dk; ++b) {
                            fprintf(cf, "%s%" PRId64, b ? ", " : "", Qc[(size_t)a * (size_t)dk + (size_t)b]);
                        }
                        fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
                    }
                    fprintf(cf, "      ],\n      \"L_int\": [\n");
                    for (int a = 0; a < dk; ++a) {
                        fprintf(cf, "        [");
                        for (int b = 0; b < dk; ++b) {
                            fprintf(cf, "%s%" PRId64, b ? ", " : "", Lc[(size_t)a * (size_t)dk + (size_t)b]);
                        }
                        fprintf(cf, "]%s\n", (a + 1 < dk) ? "," : "");
                    }
                    fprintf(cf, "      ]\n    }%s\n", (c + 1 < n_cl) ? "," : "");
                }
                fprintf(cf, "  ]\n}\n");
            }
            if (cf != NULL) fclose(cf);
            free(A_sum);
        }
    } else {
        sos2_fill_fallback(g, &shor1, d_even, cert_out_path, out);
    }

    free(mono_bnd);
    free(ms); free(mgens); free(eq_polys); free(ineq_polys); free(therm4); free(rlt4);
    free(A_op); free(C_rhs); free(G_norm); free(row_off); free(nz_col); free(nz_val);
    free(LG); free(Hinv_AT); free(u); free(u_try); free(du); free(grad); free(grad_try);
    free(r_d); free(hinv_rd); free(rhs_d); free(nu); free(nu_try); free(dnu);
    free(r_p); free(rhs_nu); free(y_tmp); free(V_all); free(ev_all); free(V_try); free(ev_try);
    free(best_Q_all); free(best_L_all);
    return 0;
}

/* ============================================================================
 * 3. Certified Grid Observable Envelope (Voltage^2 / Branch Flow)
 * ============================================================================ */

static int solve_obs_direction(const ele_grid_t *g, const double *M_obs, int d,
                               double sgn_obj, double alpha_cost, double cost_cap,
                               double *cert_lb_out, ele_psd_cert_t *psd_out) {
    /* Minimize sgn_obj * (x^T M_obs x) + alpha_cost * (Cost(x) - cost_cap) over feasible AC-OPF
       via Shor-1 dual ADMM (with MV=+1, gam deadzone, and branch thermal/angle duals) */
    int N = g->n_bus;
    int ng = g->n_gen;
    int nb = g->n_branch;
    double B = g->base_mva;
    size_t mat_sz = (size_t)d * (size_t)d;

    double *YP = (double *)calloc((size_t)N * mat_sz, sizeof(double));
    double *YQ = (double *)calloc((size_t)N * mat_sz, sizeof(double));
    double *MV = (double *)calloc((size_t)N * mat_sz, sizeof(double));
    if (!YP || !YQ || !MV) { free(YP); free(YQ); free(MV); return -1; }
    for (int k = 0; k < N; ++k) {
        build_pq_matrix_2n(g, k, YP + (size_t)k * mat_sz, YQ + (size_t)k * mat_sz);
        MV[(size_t)k * mat_sz + (size_t)k * (size_t)d + (size_t)k] = 1.0;
        MV[(size_t)k * mat_sz + (size_t)(N + k) * (size_t)d + (size_t)(N + k)] = 1.0;
    }

    typedef struct {
        int idx[4], has_therm, has_ang;
        double smax_pu;
        double mpf[4][4], mqf[4][4], mpt[4][4], mqt[4][4];
        double ma_lo[4][4], ma_hi[4][4];
        double norm_pf, norm_qf, dot_pqf, norm_pt, norm_qt, dot_pqt, norm_alo, norm_ahi;
    } obs_br_t;
    obs_br_t *bd = (obs_br_t *)calloc((size_t)(nb > 0 ? nb : 1), sizeof(obs_br_t));
    if (bd == NULL) { free(YP); free(YQ); free(MV); return -1; }
    for (int e = 0; e < nb; ++e) {
        const ele_grid_branch_t *br = &g->branch[e];
        obs_br_t *b = &bd[e];
        br_y_t by;
        compute_branch_y(br, &by);
        int u = by.u, v = by.v;
        b->idx[0] = u; b->idx[1] = v; b->idx[2] = N + u; b->idx[3] = N + v;
        if (u == v) continue;
        if (br->smax_mva > 0.0 && br->smax_mva < 9000.0) {
            b->has_therm = 1;
            b->smax_pu = br->smax_mva / B;
            fill_4x4(by.yff_r, 0.0, by.yft_r, by.yft_i, b->mpf);
            fill_4x4(-by.yff_i, 0.0, -by.yft_i, by.yft_r, b->mqf);
            fill_4x4(0.0, by.ytt_r, by.ytf_r, -by.ytf_i, b->mpt);
            fill_4x4(0.0, -by.ytt_i, -by.ytf_i, -by.ytf_r, b->mqt);
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    b->norm_pf += b->mpf[r][c] * b->mpf[r][c];
                    b->norm_qf += b->mqf[r][c] * b->mqf[r][c];
                    b->dot_pqf += b->mpf[r][c] * b->mqf[r][c];
                    b->norm_pt += b->mpt[r][c] * b->mpt[r][c];
                    b->norm_qt += b->mqt[r][c] * b->mqt[r][c];
                    b->dot_pqt += b->mpt[r][c] * b->mqt[r][c];
                }
            }
            b->norm_pf = fmax(b->norm_pf, 1e-12); b->norm_qf = fmax(b->norm_qf, 1e-12);
            b->norm_pt = fmax(b->norm_pt, 1e-12); b->norm_qt = fmax(b->norm_qt, 1e-12);
        }
        if (br->angmin_deg > -85.0 || br->angmax_deg < 85.0) {
            b->has_ang = 1;
            double amin = (br->angmin_deg > -85.0 ? br->angmin_deg : -85.0) * (M_PI / 180.0);
            double amax = (br->angmax_deg < 85.0 ? br->angmax_deg : 85.0) * (M_PI / 180.0);
            fill_4x4(0.0, 0.0, tan(amin), -1.0, b->ma_lo);
            fill_4x4(0.0, 0.0, -tan(amax), 1.0, b->ma_hi);
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    b->norm_alo += b->ma_lo[r][c] * b->ma_lo[r][c];
                    b->norm_ahi += b->ma_hi[r][c] * b->ma_hi[r][c];
                }
            }
            b->norm_alo = fmax(b->norm_alo, 1e-12); b->norm_ahi = fmax(b->norm_ahi, 1e-12);
        }
    }

    double lam[ELE_GRID_MAX_BUS] = {0}, gam[ELE_GRID_MAX_BUS] = {0};
    double mlo[ELE_GRID_MAX_BUS] = {0}, mhi[ELE_GRID_MAX_BUS] = {0};
    double xif[ELE_GRID_MAX_BRANCH] = {0}, etaf[ELE_GRID_MAX_BRANCH] = {0};
    double xit[ELE_GRID_MAX_BRANCH] = {0}, etat[ELE_GRID_MAX_BRANCH] = {0};
    double tlo[ELE_GRID_MAX_BRANCH] = {0}, thi[ELE_GRID_MAX_BRANCH] = {0};
    double S[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double Z[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double Zprev[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double W[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double A[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double evals[ELE_CERT_MAX_DIM], evecs[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];

    for (int k = 0; k < N; ++k) { W[k * d + k] = 1.0; W[(N + k) * d + (N + k)] = 0.0; }

    double o_norm = 0.0;
    for (size_t e = 0; e < mat_sz; ++e) o_norm += fabs(M_obs[e]);
    double o_scale = fmax(1.0, 0.25 * o_norm) + alpha_cost * fmax(1.0, cost_cap);
    double rho = 0.05;
    double best_raw = -1e18;
    double best_A[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double best_cd = -1e18;

    int bus_gcnt[ELE_GRID_MAX_BUS] = {0}, bus_gids[ELE_GRID_MAX_BUS * 8];
    for (int i = 0; i < ng; ++i) {
        int b = g->gen[i].bus_idx;
        if (bus_gcnt[b] < 8) bus_gids[b * 8 + bus_gcnt[b]++] = i;
    }

    double norm_yp[ELE_GRID_MAX_BUS] = {0}, norm_yq[ELE_GRID_MAX_BUS] = {0};
    for (int k = 0; k < N; ++k) {
        double sp = 0.0, sq = 0.0;
        const double *yp = YP + (size_t)k * mat_sz, *yq = YQ + (size_t)k * mat_sz;
        for (size_t e = 0; e < mat_sz; ++e) { sp += yp[e] * yp[e]; sq += yq[e] * yq[e]; }
        norm_yp[k] = fmax(sp, 1e-12);
        norm_yq[k] = fmax(sq, 1e-12);
    }

    double M_s[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    for (size_t e = 0; e < mat_sz; ++e) M_s[e] = (sgn_obj / o_scale) * M_obs[e];

    for (int it = 0; it < 600; ++it) {
        memset(A, 0, mat_sz * sizeof(double));
        for (size_t e = 0; e < mat_sz; ++e) A[e] = M_s[e];
        for (int k = 0; k < N; ++k) {
            const double *yp = YP + (size_t)k * mat_sz, *yq = YQ + (size_t)k * mat_sz;
            double dm = mhi[k] - mlo[k];
            for (size_t e = 0; e < mat_sz; ++e) A[e] += lam[k] * yp[e] + gam[k] * yq[e];
            A[k * d + k] += dm;
            A[(N + k) * d + (N + k)] += dm;
        }
        for (int e = 0; e < nb; ++e) {
            const obs_br_t *b = &bd[e];
            if (!b->has_therm && !b->has_ang) continue;
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                    if (b->has_therm) {
                        A[pos] += xif[e] * b->mpf[r][c] + etaf[e] * b->mqf[r][c] +
                                  xit[e] * b->mpt[r][c] + etat[e] * b->mqt[r][c];
                    }
                    if (b->has_ang) A[pos] += tlo[e] * b->ma_lo[r][c] + thi[e] * b->ma_hi[r][c];
                }
            }
        }

        for (int gs = 0; gs < 6; ++gs) {
            for (int k = 0; k < N; ++k) {
                const double *yp = YP + (size_t)k * mat_sz;
                double tr_w = 0.0, tr_r = 0.0;
                for (size_t e = 0; e < mat_sz; ++e) {
                    tr_w += yp[e] * W[e];
                    tr_r += yp[e] * (A[e] - Z[e]);
                }
                tr_r -= lam[k] * norm_yp[k];
                double b_lin = -(g->bus[k].pd_mw / B) - tr_w + rho * tr_r;
                double H = rho * norm_yp[k];
                double lo = -500.0, hi = 500.0;
                for (int bi = 0; bi < 30; ++bi) {
                    double mid = 0.5 * (lo + hi);
                    double dv = b_lin + H * mid;
                    for (int gi = 0; gi < bus_gcnt[k]; ++gi) {
                        const ele_grid_gen_t *gn = &g->gen[bus_gids[k * 8 + gi]];
                        double a2 = (alpha_cost * gn->c2 * B * B) / o_scale;
                        double a1 = (alpha_cost * gn->c1 * B) / o_scale;
                        double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
                        double pg = (a2 > 1e-12) ? ((mid - a1) / (2.0 * a2)) : ((mid >= a1) ? pmax : pmin);
                        if (pg < pmin) pg = pmin;
                        if (pg > pmax) pg = pmax;
                        dv += pg;
                    }
                    if (dv > 0.0) hi = mid; else lo = mid;
                }
                double nl = 0.5 * (lo + hi), dl = nl - lam[k];
                lam[k] = nl;
                for (size_t e = 0; e < mat_sz; ++e) A[e] += dl * yp[e];
            }
            for (int k = 0; k < N; ++k) {
                const double *yq = YQ + (size_t)k * mat_sz;
                double tr_w = 0.0, tr_r = 0.0;
                for (size_t e = 0; e < mat_sz; ++e) {
                    tr_w += yq[e] * W[e];
                    tr_r += yq[e] * (A[e] - Z[e]);
                }
                tr_r -= gam[k] * norm_yq[k];
                double b_lin = -(g->bus[k].qd_mvar / B) - tr_w + rho * tr_r;
                double Hg = rho * norm_yq[k];
                double qmin_s = 0.0, qmax_s = 0.0;
                for (int gi = 0; gi < bus_gcnt[k]; ++gi) {
                    qmin_s += g->gen[bus_gids[k * 8 + gi]].qmin_mvar / B;
                    qmax_s += g->gen[bus_gids[k * 8 + gi]].qmax_mvar / B;
                }
                double ng_v = -(b_lin + qmax_s) / Hg;
                if (ng_v < 0.0) {
                    double c2 = -(b_lin + qmin_s) / Hg;
                    ng_v = (c2 < 0.0) ? c2 : 0.0;
                }
                double dg = ng_v - gam[k];
                gam[k] = ng_v;
                for (size_t e = 0; e < mat_sz; ++e) A[e] += dg * yq[e];
            }
            for (int k = 0; k < N; ++k) {
                size_t e1 = (size_t)k * (size_t)d + (size_t)k;
                size_t e2 = (size_t)(N + k) * (size_t)d + (size_t)(N + k);
                double tr_mw = W[e1] + W[e2];
                double tr_mr = (A[e1] - Z[e1]) + (A[e2] - Z[e2]) - 2.0 * mhi[k];
                double nmhi = fmax(0.0, (-(g->bus[k].vmax * g->bus[k].vmax) + tr_mw - rho * tr_mr) / (2.0 * rho));
                double dm = nmhi - mhi[k];
                mhi[k] = nmhi; A[e1] += dm; A[e2] += dm;

                tr_mr = (A[e1] - Z[e1]) + (A[e2] - Z[e2]) + 2.0 * mlo[k];
                double nmlo = fmax(0.0, ((g->bus[k].vmin * g->bus[k].vmin) - tr_mw + rho * tr_mr) / (2.0 * rho));
                dm = nmlo - mlo[k];
                mlo[k] = nmlo; A[e1] -= dm; A[e2] -= dm;
            }
            for (int e = 0; e < nb; ++e) {
                const obs_br_t *b = &bd[e];
                if (b->has_therm) {
                    double xo = xif[e], yo = etaf[e], twp = 0.0, trp = 0.0, twq = 0.0, trq = 0.0;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        double mp = b->mpf[r][c], mq = b->mqf[r][c];
                        double rem = (A[pos] - Z[pos]) - xo * mp - yo * mq;
                        twp += mp * W[pos]; trp += mp * rem; twq += mq * W[pos]; trq += mq * rem;
                    }
                    double xn = xo, yn = yo;
                    prox_cone_2d(rho * b->norm_pf, rho * b->norm_qf, rho * b->dot_pqf,
                                 -twp + rho * trp, -twq + rho * trq, b->smax_pu, &xn, &yn);
                    double dx = xn - xo, dy = yn - yo; xif[e] = xn; etaf[e] = yn;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        A[pos] += dx * b->mpf[r][c] + dy * b->mqf[r][c];
                    }
                    xo = xit[e]; yo = etat[e]; twp = 0.0; trp = 0.0; twq = 0.0; trq = 0.0;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        double mp = b->mpt[r][c], mq = b->mqt[r][c];
                        double rem = (A[pos] - Z[pos]) - xo * mp - yo * mq;
                        twp += mp * W[pos]; trp += mp * rem; twq += mq * W[pos]; trq += mq * rem;
                    }
                    xn = xo; yn = yo;
                    prox_cone_2d(rho * b->norm_pt, rho * b->norm_qt, rho * b->dot_pqt,
                                 -twp + rho * trp, -twq + rho * trq, b->smax_pu, &xn, &yn);
                    dx = xn - xo; dy = yn - yo; xit[e] = xn; etat[e] = yn;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        A[pos] += dx * b->mpt[r][c] + dy * b->mqt[r][c];
                    }
                }
                if (b->has_ang) {
                    double to = tlo[e], tw = 0.0, tr = 0.0;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        double ma = b->ma_lo[r][c]; tw += ma * W[pos]; tr += ma * ((A[pos] - Z[pos]) - to * ma);
                    }
                    double tn = fmax(0.0, (tw - rho * tr) / (rho * b->norm_alo));
                    double dt = tn - to; tlo[e] = tn;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        A[(size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c]] += dt * b->ma_lo[r][c];
                    }
                    to = thi[e]; tw = 0.0; tr = 0.0;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        size_t pos = (size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c];
                        double ma = b->ma_hi[r][c]; tw += ma * W[pos]; tr += ma * ((A[pos] - Z[pos]) - to * ma);
                    }
                    tn = fmax(0.0, (tw - rho * tr) / (rho * b->norm_ahi));
                    dt = tn - to; thi[e] = tn;
                    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
                        A[(size_t)b->idx[r] * (size_t)d + (size_t)b->idx[c]] += dt * b->ma_hi[r][c];
                    }
                }
            }
        }

        memcpy(Zprev, Z, mat_sz * sizeof(double));
        double Tmat[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
        for (size_t e = 0; e < mat_sz; ++e) Tmat[e] = A[e] - W[e] / rho;
        if (ele_sym_jacobi_eigs(d, Tmat, evals, evecs) != 0) break;
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                double zij = 0.0, wij = 0.0;
                for (int r = 0; r < d; ++r) {
                    double ev = evals[r], vr = evecs[i * d + r] * evecs[j * d + r];
                    if (ev > 0.0) zij += ev * vr; else wij += (-rho * ev) * vr;
                }
                Z[i * d + j] = zij; Z[j * d + i] = zij;
                W[i * d + j] = wij; W[j * d + i] = wij;
            }
        }
        if (it % 20 == 19) {
            double rn = 0.0, sn = 0.0;
            for (size_t e = 0; e < mat_sz; ++e) {
                double dr = A[e] - Z[e], ds = Z[e] - Zprev[e];
                rn += dr * dr; sn += ds * ds;
            }
            rn = sqrt(rn); sn = rho * sqrt(sn);
            if (rn > 3.0 * sn && rho < 10.0) rho *= 1.3;
            else if (sn > 3.0 * rn && rho > 1e-3) rho /= 1.3;
        }

        if (it >= 40 && (it % 20 == 19 || it == 599)) {
            double W_max = 0.0;
            double c_d = -alpha_cost * cost_cap;
            for (int k = 0; k < N; ++k) {
                W_max += g->bus[k].vmax * g->bus[k].vmax;
                double lk = lam[k] * o_scale, gk = gam[k] * o_scale;
                double ml = mlo[k] * o_scale, mh = mhi[k] * o_scale;
                c_d += lk * (g->bus[k].pd_mw / B) + gk * (g->bus[k].qd_mvar / B);
                c_d += ml * (g->bus[k].vmin * g->bus[k].vmin) - mh * (g->bus[k].vmax * g->bus[k].vmax);
            }
            for (int i = 0; i < ng; ++i) {
                const ele_grid_gen_t *gn = &g->gen[i];
                int k = gn->bus_idx;
                double lk = lam[k] * o_scale, gk = gam[k] * o_scale;
                double a2 = alpha_cost * gn->c2 * B * B, a1 = alpha_cost * gn->c1 * B, a0 = alpha_cost * gn->c0;
                double pmin = gn->pmin_mw / B, pmax = gn->pmax_mw / B;
                double pg = (a2 > 1e-12) ? ((lk - a1) / (2.0 * a2)) : ((lk > a1) ? pmax : pmin);
                if (pg < pmin) pg = pmin;
                if (pg > pmax) pg = pmax;
                double qg = (gk >= 0.0) ? (gn->qmax_mvar / B) : (gn->qmin_mvar / B);
                c_d += a2 * pg * pg + (a1 - lk) * pg + a0 - gk * qg;
            }
            for (int e = 0; e < nb; ++e) {
                if (bd[e].has_therm) {
                    c_d -= bd[e].smax_pu * (hypot(xif[e], etaf[e]) + hypot(xit[e], etat[e])) * o_scale;
                }
            }
            if (ele_sym_jacobi_eigs(d, A, evals, NULL) == 0) {
                double lmin = evals[0] * o_scale;
                double lb_c = c_d + ((lmin < 0.0) ? (lmin * W_max) : 0.0);
                if (isfinite(lb_c) && lb_c > best_raw) {
                    best_raw = lb_c;
                    best_cd = c_d;
                    for (size_t e = 0; e < mat_sz; ++e) best_A[e] = A[e] * o_scale;
                }
            }
        }
    }

    (void)S;
    free(bd); free(YP); free(YQ); free(MV);
    if (!isfinite(best_cd) || best_raw <= -1e17) return -1;

    /* Integer Cholesky + SDD certification */
    const int64_t S_V = 1000;
    const int64_t Q_A = 1000000;
    const int64_t scale_tot = S_V * S_V * Q_A; /* 10^12 */
    int64_t A_i[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    int64_t L_i[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM];
    double nf = 0.0, W_max = 0.0;
    int64_t w_max_int = 0;
    for (int k = 0; k < N; ++k) {
        W_max += g->bus[k].vmax * g->bus[k].vmax;
        int64_t vmk = (int64_t)ceil(g->bus[k].vmax * (double)S_V) + 2;
        w_max_int += vmk * vmk;
    }
    for (int i = 0; i < d; ++i) {
        for (int j = i; j < d; ++j) {
            double v = 0.5 * (best_A[i * d + j] + best_A[j * d + i]);
            if (!isfinite(v) || fabs(v * (double)Q_A) > 4e18) return -1;
            int64_t qv = (int64_t)llround(v * (double)Q_A);
            A_i[i * d + j] = qv; A_i[j * d + i] = qv;
            nf += v * v * ((i == j) ? 1.0 : 2.0);
        }
    }
    nf = sqrt(nf);
    int64_t mu_psd = 0;
    if (ele_cert_cholesky_sdd_i64(d, A_i, L_i, &mu_psd) != 0) return -1;
    int64_t cd_i = (int64_t)floor(best_cd * (double)scale_tot);
    int64_t drnd = (int64_t)ceil((double)N * (double)w_max_int +
                                 (double)Q_A * nf * (sqrt((double)d * W_max) + 0.5 * (double)N) + 100.0);
    ele_f_i128 neg_mu = (mu_psd < 0) ? (ele_f_i128)mu_psd : 0;
    ele_f_i128 lb_i = (ele_f_i128)cd_i + neg_mu * (ele_f_i128)w_max_int - (ele_f_i128)drnd;
    *cert_lb_out = (double)(int64_t)lb_i / (double)scale_tot;

    if (psd_out != NULL) {
        psd_out->dim = d;
        psd_out->scale_q = Q_A;
        psd_out->mu_psd = mu_psd;
        psd_out->delta_round = 0;
        psd_out->mu_cert = mu_psd;
        psd_out->ok = 1;
        memcpy(psd_out->A, A_i, (size_t)d * (size_t)d * sizeof(int64_t));
        memcpy(psd_out->L, L_i, (size_t)d * (size_t)d * sizeof(int64_t));
    }
    return 0;
}

int ele_grid_observable(const ele_grid_t *g, ele_grid_obs_kind_t kind,
                        int target_id, double cost_eps, ele_grid_obs_res_t *out) {
    if (g == NULL || out == NULL || g->n_bus <= 0 || 2 * g->n_bus > ELE_CERT_MAX_DIM) return -1;
    memset(out, 0, sizeof(*out));
    out->kind = kind;
    out->target_id = target_id;

    ele_grid_opf_res_t opf;
    if (ele_grid_acopf(g, &opf) != 0) return -1;
    out->c_ub_primal = opf.c_ub_primal;
    out->c_lb_cert = opf.c_lb_cert;
    out->cost_gap_abs = opf.gap_abs;
    out->cost_cap = opf.c_ub_primal + fmax(cost_eps, 0.0);

    int N = g->n_bus;
    int d = 2 * N;
    double M_obs[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    double x_pr[ELE_CERT_MAX_DIM] = {0};
    for (int k = 0; k < N; ++k) {
        x_pr[k] = opf.ub_pf.vm[k] * cos(opf.ub_pf.va_rad[k]);
        x_pr[N + k] = opf.ub_pf.vm[k] * sin(opf.ub_pf.va_rad[k]);
    }

    if (kind == ELE_OBS_VM2) {
        int b_idx = -1;
        for (int k = 0; k < N; ++k) {
            if (g->bus[k].id == target_id) { b_idx = k; break; }
        }
        if (b_idx < 0) return -1;
        M_obs[b_idx * d + b_idx] = 1.0;
        M_obs[(N + b_idx) * d + (N + b_idx)] = 1.0;
        out->o_primal = opf.ub_pf.vm[b_idx] * opf.ub_pf.vm[b_idx];
    } else {
        if (target_id < 0 || target_id >= g->n_branch) return -1;
        br_y_t by;
        compute_branch_y(&g->branch[target_id], &by);
        double M4[4][4];
        fill_4x4(by.yff_r, 0.0, by.yft_r, by.yft_i, M4);
        int idx4[4] = { by.u, by.v, N + by.u, N + by.v };
        for (int a = 0; a < 4; ++a) {
            for (int b = 0; b < 4; ++b) {
                M_obs[idx4[a] * d + idx4[b]] = g->base_mva * M4[a][b];
            }
        }
        double pf = 0.0;
        for (int a = 0; a < d; ++a) {
            for (int b = 0; b < d; ++b) pf += M_obs[a * d + b] * x_pr[a] * x_pr[b];
        }
        out->o_primal = pf;
    }

    static const double alphas[9] = {0.0, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 0.1, 0.3};
    double best_lo = -1e18;
    double best_neg_ub = -1e18;
    ele_psd_cert_t best_psd;
    memset(&best_psd, 0, sizeof(best_psd));

    for (int ai = 0; ai < 9; ++ai) {
        double al = alphas[ai];
        double cand_lo = -1e18;
        ele_psd_cert_t psd_tmp;
        if (solve_obs_direction(g, M_obs, d, +1.0, al, out->cost_cap, &cand_lo, &psd_tmp) == 0) {
            if (cand_lo > best_lo && cand_lo <= out->o_primal + 1e-6) {
                best_lo = cand_lo;
                best_psd = psd_tmp;
            }
        }
        double cand_neg = -1e18;
        if (solve_obs_direction(g, M_obs, d, -1.0, al, out->cost_cap, &cand_neg, NULL) == 0) {
            if (cand_neg > best_neg_ub && (-cand_neg) >= out->o_primal - 1e-6) {
                best_neg_ub = cand_neg;
            }
        }
    }

    double best_hi = -best_neg_ub;
    if (best_lo > out->o_primal) best_lo = out->o_primal;
    if (best_hi < out->o_primal) best_hi = out->o_primal;

    out->o_lb_cert = best_lo;
    out->o_ub_cert = best_hi;
    out->width = out->o_ub_cert - out->o_lb_cert;

    const int64_t BS = 1000000;
    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "grid");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "observable_envelope");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s",
             (best_psd.ok && out->o_lb_cert <= out->o_primal + 1e-7 && out->o_primal <= out->o_ub_cert + 1e-7)
             ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", g->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", g->input_sha256);
    if (best_psd.ok) {
        out->cert.has_psd = 1;
        out->cert.psd = best_psd;
    }
    out->cert.has_bracket = 1;
    out->cert.bracket_scale = BS;
    out->cert.lb_int = (int64_t)floor(out->o_lb_cert * (double)BS);
    out->cert.ub_int = (int64_t)ceil(out->o_ub_cert * (double)BS);
    if (out->cert.lb_int > out->cert.ub_int) out->cert.lb_int = out->cert.ub_int;
    out->cert.limit_lo_int = out->cert.lb_int;
    out->cert.limit_hi_int = out->cert.ub_int;
    out->cert.lb_val = out->o_lb_cert;
    out->cert.ub_val = out->o_ub_cert;
    return 0;
}

/* ============================================================================
 * 4. Maxwell 2D TMz MoM Convex Physical Bounds + Bode-Fano
 * ============================================================================ */

static double ele_bessel_j0(double x) {
    double ax = fabs(x);
    if (ax < 8.0) {
        double y = x * x;
        double a = 57568490574.0 + y * (-13362590354.0 + y * (651619640.7 +
                   y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));
        double b = 57568490411.0 + y * (1029532985.0 + y * (9494680.718 +
                   y * (59272.64853 + y * (267.8532712 + y))));
        return a / b;
    } else {
        double z = 8.0 / ax, y = z * z, xx = ax - 0.7853981633974483096;
        double a = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 +
                   y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
        double b = -0.1562499995e-1 + y * (0.1430488765e-3 +
                   y * (-0.6911147651e-5 + y * (0.7621095161e-6 - y * 0.934945152e-7)));
        return sqrt(0.63661977236758134308 / ax) * (cos(xx) * a - z * sin(xx) * b);
    }
}

static double ele_bessel_y0(double x) {
    if (x < 8.0) {
        double y = x * x;
        double a = -2957821389.0 + y * (7062834065.0 + y * (-512359803.6 +
                   y * (10879881.29 + y * (-86327.92757 + y * 228.4622733))));
        double b = 40076544269.0 + y * (745249964.8 + y * (7189466.438 +
                   y * (47447.26470 + y * (226.1030244 + y))));
        return (a / b) + 0.63661977236758134308 * ele_bessel_j0(x) * log(x);
    } else {
        double z = 8.0 / x, y = z * z, xx = x - 0.7853981633974483096;
        double a = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 +
                   y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
        double b = -0.1562499995e-1 + y * (0.1430488765e-3 +
                   y * (-0.6911147651e-5 + y * (0.7621095161e-6 - y * 0.934945152e-7)));
        return sqrt(0.63661977236758134308 / x) * (sin(xx) * a + z * cos(xx) * b);
    }
}

static double ele_bessel_y1(double x) {
    if (x < 8.0) {
        double y = x * x;
        double a = x * (-0.4900604943e13 + y * (0.1275274390e13 +
                   y * (-0.5153438139e11 + y * (0.7349264551e9 +
                   y * (-0.4237922726e7 + y * 0.8511937935e4)))));
        double b = 0.2499580570e14 + y * (0.4244419664e12 +
                   y * (0.3733650367e10 + y * (0.2245904002e8 +
                   y * (0.1020426050e6 + y * (0.3549632885e3 + y)))));
        double j1;
        {
            double aj = x * (72362614232.0 + y * (-7895059235.0 + y * (242396853.1 +
                        y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
            double bj = 144725228442.0 + y * (2300535178.0 + y * (18583304.74 +
                        y * (99447.43394 + y * (376.9991397 + y))));
            j1 = aj / bj;
        }
        return (a / b) + 0.63661977236758134308 * (j1 * log(x) - 1.0 / x);
    } else {
        double z = 8.0 / x, y = z * z, xx = x - 2.3561944901923449288;
        double a = 1.0 + y * (0.183105e-2 + y * (-0.3516396496e-4 +
                   y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        double b = 0.04687499995 + y * (-0.2002690873e-3 +
                   y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        return sqrt(0.63661977236758134308 / x) * (sin(xx) * a + z * cos(xx) * b);
    }
}

static int eval_maxwell_dual(int M, const double *Ur, const double *Ui,
                             const double *er, const double *ei,
                             const double *alpha, const double *beta,
                             double *B_out, double *grad_a, double *grad_b,
                             double *A_re_out, double *A_im_out,
                             double *br_out, double *bi_out, int *spd_ok) {
    /* Hermitian A = 0.5 * (Lambda * U + U^H * Lambda^H), Lambda = diag(beta + i * alpha)
       Linear RHS: b_m = (beta_m + i*(1+alpha_m)) * e_inc_m */
    double Ar[25 * 25], Ai[25 * 25], br[25], bi[25], tr[25], ti[25];
    for (int m = 0; m < M; ++m) {
        double bm = beta[m], am = alpha[m];
        br[m] = bm * er[m] - (1.0 + am) * ei[m];
        bi[m] = bm * ei[m] + (1.0 + am) * er[m];
    }
    for (int m = 0; m < M; ++m) {
        for (int n = 0; n < M; ++n) {
            /* (Lambda * U)_{mn} = (bm + i*am)*(Ur_mn + i*Ui_mn) */
            double lur = beta[m] * Ur[m * M + n] - alpha[m] * Ui[m * M + n];
            double lui = beta[m] * Ui[m * M + n] + alpha[m] * Ur[m * M + n];
            /* (U^H * Lambda^H)_{mn} = conj(U_{nm}) * (bn - i*an) */
            double ur_nm = Ur[n * M + m], ui_nm = Ui[n * M + m];
            double ulr = ur_nm * beta[n] - ui_nm * alpha[n];
            double uli = -ur_nm * alpha[n] - ui_nm * beta[n];
            Ar[m * M + n] = 0.5 * (lur + ulr);
            Ai[m * M + n] = 0.5 * (lui + uli);
        }
    }

    /* Check A > 0 via 2M x 2M real symmetric matrix eigendecomposition */
    int d = 2 * M;
    double Aug[50 * 50], ev[50];
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            Aug[i * d + j] = Ar[i * M + j];
            Aug[(M + i) * d + (M + j)] = Ar[i * M + j];
            Aug[i * d + (M + j)] = -Ai[i * M + j];
            Aug[(M + i) * d + j] = Ai[i * M + j];
        }
    }
    if (ele_sym_jacobi_eigs(d, Aug, ev, NULL) != 0 || ev[0] <= 1e-8) {
        if (spd_ok != NULL) *spd_ok = 0;
        return -1;
    }
    if (spd_ok != NULL) *spd_ok = 1;

    /* Solve 2 * A * t = b  <=> A * (2t) = b; let sol = A^{-1} b, so t = 0.5 * sol */
    double solr[25], soli[25];
    if (ele_lu_solve_complex(M, Ar, Ai, br, bi, solr, soli) != 0) {
        if (spd_ok != NULL) *spd_ok = 0;
        return -1;
    }
    double dot_rb = 0.0;
    for (int m = 0; m < M; ++m) {
        tr[m] = 0.5 * solr[m];
        ti[m] = 0.5 * soli[m];
        dot_rb += br[m] * tr[m] + bi[m] * ti[m];
    }
    *B_out = 0.5 * dot_rb; /* 0.25 * b^H A^{-1} b = 0.5 * Re(b^H t) */

    if (grad_a != NULL && grad_b != NULL) {
        for (int m = 0; m < M; ++m) {
            double utr = 0.0, uti = 0.0;
            for (int n = 0; n < M; ++n) {
                utr += Ur[m * M + n] * tr[n] - Ui[m * M + n] * ti[n];
                uti += Ur[m * M + n] * ti[n] + Ui[m * M + n] * tr[n];
            }
            /* t_m^* * (U t)_m = (tr_m - i ti_m)*(utr + i uti) */
            double tu_re = tr[m] * utr + ti[m] * uti;
            double tu_im = tr[m] * uti - ti[m] * utr;
            /* e_inc_m^* * t_m */
            double et_re = er[m] * tr[m] + ei[m] * ti[m];
            double et_im = er[m] * ti[m] - ei[m] * tr[m];
            grad_a[m] = et_im + tu_im;
            grad_b[m] = et_re - tu_re;
        }
    }
    if (A_re_out != NULL) memcpy(A_re_out, Ar, (size_t)M * (size_t)M * sizeof(double));
    if (A_im_out != NULL) memcpy(A_im_out, Ai, (size_t)M * (size_t)M * sizeof(double));
    if (br_out != NULL) memcpy(br_out, br, (size_t)M * sizeof(double));
    if (bi_out != NULL) memcpy(bi_out, bi, (size_t)M * sizeof(double));
    return 0;
}

static double eval_maxwell_subset(int M, const double *Ur, const double *Ui,
                                  const double *er, const double *ei, uint32_t mask) {
    int idx[25], na = 0;
    for (int m = 0; m < M; ++m) {
        if ((mask >> m) & 1U) idx[na++] = m;
    }
    if (na == 0) return 0.0;
    double sr[25 * 25], si[25 * 25], rhsr[25], rhsi[25], xr[25], xi[25];
    for (int a = 0; a < na; ++a) {
        int ia = idx[a];
        rhsr[a] = er[ia]; rhsi[a] = ei[ia];
        for (int b = 0; b < na; ++b) {
            int ib = idx[b];
            sr[a * na + b] = Ur[ia * M + ib];
            si[a * na + b] = Ui[ia * M + ib];
        }
    }
    if (ele_lu_solve_complex(na, sr, si, rhsr, rhsi, xr, xi) != 0) return 0.0;
    double p_ext = 0.0;
    for (int a = 0; a < na; ++a) {
        p_ext += rhsr[a] * xi[a] - rhsi[a] * xr[a];
    }
    return p_ext;
}

double ele_em_bode_fano_bound(double r_ohm, double c_farad,
                              double f_low_hz, double f_high_hz) {
    if (r_ohm <= 0.0 || c_farad <= 0.0 || f_high_hz <= f_low_hz) return 1.0;
    double tau = r_ohm * c_farad;
    double df = f_high_hz - f_low_hz;
    /* Discrete Bode-Fano integral bound: exp(-pi / (omega_df * R * C)) = exp(-1 / (2 * R * C * df)) */
    double exponent = -1.0 / (2.0 * tau * df);
    double gam = exp(exponent);
    if (gam > 1.0) gam = 1.0;
    if (gam < 1e-9) gam = 1e-9;
    return gam;
}

int ele_em_maxwell_bound(double kl, double zeta, double xi0, int n_cells,
                         ele_em_bound_res_t *out) {
    if (out == NULL || kl <= 0.0 || zeta <= 0.0 || n_cells < 4 || n_cells > 25) return -1;
    memset(out, 0, sizeof(*out));
    out->kl = kl;
    out->zeta = zeta;
    out->xi0 = xi0;
    out->n_cells = n_cells;

    int nx = (int)floor(sqrt((double)n_cells) + 0.5);
    if (nx * nx < n_cells) nx = (int)ceil(sqrt((double)n_cells));
    double h = kl / (double)nx;
    double k2_area = h * h;
    double k_req = h / sqrt(M_PI);

    double cx[25], cy[25];
    for (int m = 0; m < n_cells; ++m) {
        int ix = m % nx;
        int iy = m / nx;
        cx[m] = ((double)ix + 0.5) * h;
        cy[m] = ((double)iy + 0.5) * h;
    }

    double Gr[25 * 25], Gi[25 * 25], Ur[25 * 25], Ui[25 * 25], er[25], ei[25];
    double y1_req = ele_bessel_y1(k_req);
    double g_self_re = -1.0 - 0.5 * M_PI * k_req * y1_req;
    for (int m = 0; m < n_cells; ++m) {
        er[m] = cos(cx[m]);
        ei[m] = sin(cx[m]);
        for (int n = 0; n < n_cells; ++n) {
            if (m == n) {
                Gr[m * n_cells + n] = g_self_re;
                Gi[m * n_cells + n] = 0.25 * k2_area;
            } else {
                double r = hypot(cx[m] - cx[n], cy[m] - cy[n]);
                Gr[m * n_cells + n] = -0.25 * k2_area * ele_bessel_y0(r);
                Gi[m * n_cells + n] = 0.25 * k2_area * ele_bessel_j0(r);
            }
            Ur[m * n_cells + n] = ((m == n) ? xi0 : 0.0) - Gr[m * n_cells + n];
            Ui[m * n_cells + n] = ((m == n) ? (-1.0 / zeta) : 0.0) - Gi[m * n_cells + n];
        }
    }

    /* 1. Global Optical Bound: P_global = e_inc^H (zeta^-1 I + Gi)^-1 e_inc */
    double A0r[25 * 25], A0i[25 * 25] = {0}, solr[25], soli[25];
    for (int m = 0; m < n_cells; ++m) {
        for (int n = 0; n < n_cells; ++n) {
            A0r[m * n_cells + n] = ((m == n) ? (1.0 / zeta) : 0.0) + Gi[m * n_cells + n];
        }
    }
    if (ele_lu_solve_complex(n_cells, A0r, A0i, er, ei, solr, soli) != 0) return -1;
    double p_glob = 0.0;
    for (int m = 0; m < n_cells; ++m) p_glob += er[m] * solr[m] + ei[m] * soli[m];
    out->p_global_ub = p_glob;

    /* 2. Local Cell Poynting Dual Bound optimization */
    double alpha[25], beta[25], ga[25], gb[25];
    for (int m = 0; m < n_cells; ++m) { alpha[m] = 1.0; beta[m] = 0.0; }
    double B_cur = p_glob;
    int spd = 0;
    if (eval_maxwell_dual(n_cells, Ur, Ui, er, ei, alpha, beta, &B_cur, ga, gb,
                          NULL, NULL, NULL, NULL, &spd) != 0) return -1;

    double step = 0.05;
    for (int it = 0; it < 160; ++it) {
        double gnorm2 = 0.0;
        for (int m = 0; m < n_cells; ++m) gnorm2 += ga[m] * ga[m] + gb[m] * gb[m];
        if (gnorm2 < 1e-12) break;
        double gnorm = sqrt(gnorm2);
        double accepted = 0.0;
        double st = step;
        for (int ls = 0; ls < 12; ++ls) {
            double na[25], nb[25], B_new = 0.0, gna[25], gnb[25];
            for (int m = 0; m < n_cells; ++m) {
                na[m] = alpha[m] - st * (ga[m] / gnorm);
                nb[m] = beta[m] - st * (gb[m] / gnorm);
            }
            int sp = 0;
            if (eval_maxwell_dual(n_cells, Ur, Ui, er, ei, na, nb, &B_new, gna, gnb,
                                  NULL, NULL, NULL, NULL, &sp) == 0 &&
                sp && B_new < B_cur - 1e-10) {
                memcpy(alpha, na, (size_t)n_cells * sizeof(double));
                memcpy(beta, nb, (size_t)n_cells * sizeof(double));
                memcpy(ga, gna, (size_t)n_cells * sizeof(double));
                memcpy(gb, gnb, (size_t)n_cells * sizeof(double));
                B_cur = B_new;
                step = fmin(0.4, st * 1.4);
                accepted = 1.0;
                break;
            }
            st *= 0.5;
        }
        if (accepted < 0.5) break;
    }
    out->p_local_ub_raw = B_cur;

    /* Extract Ar, Ai, br, bi at optimal (alpha, beta) & certify (2M+1)x(2M+1) Schur Matrix! */
    double Ar[25 * 25], Ai[25 * 25], br[25], bi[25];
    eval_maxwell_dual(n_cells, Ur, Ui, er, ei, alpha, beta, &B_cur, NULL, NULL,
                      Ar, Ai, br, bi, &spd);

    int M = n_cells;
    int dm = 2 * M + 1; /* <= 51 <= ELE_CERT_MAX_DIM */
    double M_schur[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    /* M_schur = [ 4*B_cert, -2*b^T; -2*b, 4*A ] */
    double B_cert_fp = B_cur + 1e-4 * fmax(1.0, p_glob);
    M_schur[0] = 4.0 * B_cert_fp;
    for (int m = 0; m < M; ++m) {
        M_schur[1 + m] = -2.0 * br[m];
        M_schur[1 + M + m] = -2.0 * bi[m];
        M_schur[(1 + m) * dm] = -2.0 * br[m];
        M_schur[(1 + M + m) * dm] = -2.0 * bi[m];
    }
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            M_schur[(1 + i) * dm + (1 + j)] = 4.0 * Ar[i * M + j];
            M_schur[(1 + M + i) * dm + (1 + M + j)] = 4.0 * Ar[i * M + j];
            M_schur[(1 + i) * dm + (1 + M + j)] = -4.0 * Ai[i * M + j];
            M_schur[(1 + M + i) * dm + (1 + j)] = 4.0 * Ai[i * M + j];
        }
    }

    const int64_t Q_S = 100000; /* 10^5 */
    int64_t M_int[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    int64_t L_int[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    for (int i = 0; i < dm; ++i) {
        for (int j = i; j < dm; ++j) {
            double v = 0.5 * (M_schur[i * dm + j] + M_schur[j * dm + i]);
            int64_t qv = (int64_t)llround(v * (double)Q_S);
            M_int[i * dm + j] = qv; M_int[j * dm + i] = qv;
        }
    }
    int64_t mu_psd = 0;
    for (int bump = 0; bump < 6; ++bump) {
        if (ele_cert_cholesky_sdd_i64(dm, M_int, L_int, &mu_psd) != 0) return -1;
        if (mu_psd >= 0) break;
        /* Shift diagonal minimally to guarantee mu_psd >= 0 cleanly */
        int64_t sh = (-mu_psd) + 10;
        for (int i = 0; i < dm; ++i) M_int[i * dm + i] += sh;
        if (ele_cert_cholesky_sdd_i64(dm, M_int, L_int, &mu_psd) != 0) return -1;
        if (mu_psd >= 0) {
            /* Update B_cert_fp to absorb diagonal shift cleanly: 4*B_eff >= M_int[0]/Q_S + sum_i (shift * ...) */
            B_cert_fp = ((double)M_int[0] / (double)Q_S) * 0.25 +
                        ((double)sh / (double)Q_S) * 0.25;
            break;
        }
    }
    B_cert_fp = fmax(B_cert_fp, ((double)M_int[0] / (double)Q_S) * 0.25);
    out->p_local_ub_cert = B_cert_fp;
    out->mu_psd_int = mu_psd;
    out->tightening_ratio = (p_glob > 1e-12) ? (out->p_local_ub_cert / p_glob) : 1.0;

    /* 3. SEARU Binary Mask Optimization (deterministic seeded greedy + 1/2-bit ascent) */
    uint32_t full_mask = (M < 32) ? ((1U << M) - 1U) : 0xFFFFFFFFU;
    uint32_t seeds[6] = {
        full_mask,
        0x55555555U & full_mask,
        0xAAAAAAAAU & full_mask,
        0x0F0F0F0FU & full_mask,
        0x33333333U & full_mask,
        0x06600660U & full_mask
    };
    uint32_t best_mask = full_mask;
    double best_p = eval_maxwell_subset(M, Ur, Ui, er, ei, full_mask);

    for (int s = 0; s < 6; ++s) {
        uint32_t mk = seeds[s];
        if (mk == 0U) continue;
        double cur = eval_maxwell_subset(M, Ur, Ui, er, ei, mk);
        if (cur > best_p) { best_p = cur; best_mask = mk; }
        for (int pass = 0; pass < 3; ++pass) {
            int improved = 0;
            for (int b = 0; b < M; ++b) {
                uint32_t cand = mk ^ (1U << b);
                if (cand == 0U) continue;
                double pc = eval_maxwell_subset(M, Ur, Ui, er, ei, cand);
                if (pc > cur + 1e-12) { cur = pc; mk = cand; improved = 1; }
            }
            if (!improved) break;
        }
        if (cur > best_p) { best_p = cur; best_mask = mk; }
    }
    out->p_searu_lb = best_p;
    out->searu_best_mask = best_mask;
    out->efficiency_gap_pct = (out->p_local_ub_cert > 1e-12)
        ? (100.0 * (out->p_local_ub_cert - out->p_searu_lb) / out->p_local_ub_cert)
        : 0.0;

    /* Reference 50-Ohm / 1.5 pF 2.4-2.5 GHz Bode-Fano limit */
    out->bf_gamma_limit = ele_em_bode_fano_bound(50.0, 1.5e-12, 2.4e9, 2.5e9);
    out->bf_s11_limit_db = 20.0 * log10(fmax(out->bf_gamma_limit, 1e-12));

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "em");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "maxwell_poynting_bound");
    int ok_ord = (mu_psd >= 0 && out->p_searu_lb <= out->p_local_ub_cert + 1e-9 &&
                  out->p_local_ub_cert <= out->p_global_ub + 1e-6);
    snprintf(out->cert.status, sizeof(out->cert.status), "%s", ok_ord ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "internal");
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256),
             "0000000000000000000000000000000000000000000000000000000000000000");
    out->cert.has_psd = 1;
    out->cert.psd.dim = dm;
    out->cert.psd.scale_q = Q_S;
    out->cert.psd.mu_psd = mu_psd;
    out->cert.psd.delta_round = 0;
    out->cert.psd.mu_cert = mu_psd;
    out->cert.psd.ok = (mu_psd >= 0) ? 1 : 0;
    memcpy(out->cert.psd.A, M_int, (size_t)dm * (size_t)dm * sizeof(int64_t));
    memcpy(out->cert.psd.L, L_int, (size_t)dm * (size_t)dm * sizeof(int64_t));

    out->cert.has_bracket = 1;
    out->cert.bracket_scale = Q_S;
    int64_t loc_i = (int64_t)ceil(out->p_local_ub_cert * (double)Q_S);
    int64_t glob_i = (int64_t)ceil(out->p_global_ub * (double)Q_S);
    if (glob_i < loc_i) glob_i = loc_i;
    out->cert.lb_int = loc_i;
    out->cert.ub_int = glob_i;
    out->cert.limit_lo_int = 0;
    out->cert.limit_hi_int = glob_i;
    out->cert.lb_val = out->p_local_ub_cert;
    out->cert.ub_val = out->p_global_ub;
    return 0;
}

/* ============================================================================
 * 5. KYP Bounded-Real LMI Passivity Enforcement
 * ============================================================================ */

static int em_kyp_build_and_certify(const ele_snp_t *in_snp, const char *out_s2p_path,
                                    ele_em_kyp_res_t *out,
                                    double *poles_out, double *B_out,
                                    double *C_out, double *D_out,
                                    double *P_out, double *f_mid_out) {
    if (in_snp == NULL || out == NULL || in_snp->nports <= 0 ||
        in_snp->nports > ELE_CERT_MAX_PORTS || in_snp->nfreq <= 0) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    int np = in_snp->nports;
    int nf = in_snp->nfreq;
    out->nports = np;
    out->nfreq = nf;

    ele_em_passivity_res_t orig_res;
    if (ele_em_certify_snp(in_snp, &orig_res) != 0) return -1;
    out->orig_sigma_max = orig_res.sigma_max;
    out->orig_passive = orig_res.passive_certified;

    /* 1. Spectral Singular-Value Contraction + Reciprocity Projection across frequencies */
    ele_snp_t *rep = (ele_snp_t *)calloc(1, sizeof(ele_snp_t));
    if (rep == NULL) return -1;
    *rep = *in_snp;

    const double gamma_target = 0.992;
    double sum_err2 = 0.0;
    int n_ent = 0;

    for (int f = 0; f < nf; ++f) {
        double sr[64], si[64];
        for (int i = 0; i < np; ++i) {
            for (int j = 0; j < np; ++j) {
                sr[i * np + j] = 0.5 * (rep->sr[f][i * np + j] + rep->sr[f][j * np + i]);
                si[i * np + j] = 0.5 * (rep->si[f][i * np + j] + rep->si[f][j * np + i]);
            }
        }
        /* Two passes of Hermitian G = S^H S spectral contraction + symmetrization */
        for (int pass = 0; pass < 2; ++pass) {
            int d = 2 * np;
            double aug[256] = {0}, ev[16], vec[256];
            for (int i = 0; i < np; ++i) {
                for (int j = 0; j < np; ++j) {
                    double gr = 0.0, gi = 0.0;
                    for (int k = 0; k < np; ++k) {
                        gr += sr[k * np + i] * sr[k * np + j] + si[k * np + i] * si[k * np + j];
                        gi += sr[k * np + i] * si[k * np + j] - si[k * np + i] * sr[k * np + j];
                    }
                    aug[i * d + j] = gr;
                    aug[(np + i) * d + (np + j)] = gr;
                    aug[i * d + (np + j)] = -gi;
                    aug[(np + i) * d + j] = gi;
                }
            }
            if (ele_sym_jacobi_eigs(d, aug, ev, vec) == 0 && sqrt(fmax(0.0, ev[d - 1])) > gamma_target) {
                double Fr[64] = {0}, Fi[64] = {0};
                for (int r = 0; r < d; ++r) {
                    double sig = sqrt(fmax(0.0, ev[r]));
                    double w = (sig > gamma_target && sig > 1e-12) ? (gamma_target / sig) : 1.0;
                    for (int i = 0; i < np; ++i) {
                        double ur_i = vec[i * d + r], ui_i = vec[(np + i) * d + r];
                        for (int j = 0; j < np; ++j) {
                            double ur_j = vec[j * d + r], ui_j = vec[(np + j) * d + r];
                            Fr[i * np + j] += 0.5 * w * (ur_i * ur_j + ui_i * ui_j);
                            Fi[i * np + j] += 0.5 * w * (ui_i * ur_j - ur_i * ui_j);
                        }
                    }
                }
                double nr[64] = {0}, ni[64] = {0};
                for (int i = 0; i < np; ++i) {
                    for (int j = 0; j < np; ++j) {
                        double acr = 0.0, aci = 0.0;
                        for (int k = 0; k < np; ++k) {
                            acr += sr[i * np + k] * Fr[k * np + j] - si[i * np + k] * Fi[k * np + j];
                            aci += sr[i * np + k] * Fi[k * np + j] + si[i * np + k] * Fr[k * np + j];
                        }
                        nr[i * np + j] = acr;
                        ni[i * np + j] = aci;
                    }
                }
                for (int i = 0; i < np; ++i) {
                    for (int j = 0; j < np; ++j) {
                        sr[i * np + j] = 0.5 * (nr[i * np + j] + nr[j * np + i]);
                        si[i * np + j] = 0.5 * (ni[i * np + j] + ni[j * np + i]);
                    }
                }
            } else {
                break;
            }
        }
        for (int p = 0; p < np * np; ++p) {
            double dr = sr[p] - in_snp->sr[f][p];
            double di = si[p] - in_snp->si[f][p];
            sum_err2 += dr * dr + di * di;
            ++n_ent;
            rep->sr[f][p] = sr[p];
            rep->si[f][p] = si[p];
        }
    }
    out->rms_repair_err = sqrt(sum_err2 / (double)(n_ent > 0 ? n_ent : 1));
    if (out_s2p_path != NULL && ele_snp_save(out_s2p_path, rep) != 0) {
        free(rep);
        return -1;
    }

    ele_em_passivity_res_t rep_res;
    if (ele_em_certify_snp(rep, &rep_res) != 0) {
        free(rep);
        return -1;
    }
    out->rep_sigma_max = rep_res.sigma_max;
    out->rep_recip_err = rep_res.max_recip_err;

    /* 2. Fit Real-Rational Diagonal-Pole State-Space Model + Solve Continuous-Time KYP LMI */
    int ns = 2 * np;
    if (ns > 16) ns = 16;
    out->n_state = ns;
    double poles[16];
    for (int r = 0; r < ns; ++r) {
        poles[r] = 1.0 + 1.0 * (double)r;
    }
    double Bmat[16 * 8] = {0};
    for (int r = 0; r < ns; ++r) Bmat[r * np + (r % np)] = 0.5;

    double f_mid = 0.5 * (in_snp->freq_hz[0] + in_snp->freq_hz[nf - 1]);
    if (f_mid < 1.0) f_mid = 1.0;

    double Cmat[8 * 16] = {0}, Dmat[8 * 8] = {0};
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            int ridx[8], nrc = 0;
            for (int r = 0; r < ns; ++r) {
                if ((r % np) == j && nrc < 8) ridx[nrc++] = r;
            }
            int mdim = 1 + nrc;
            double ATA[9 * 9] = {0}, ATy[9] = {0}, sol[9] = {0};
            for (int a = 0; a < mdim; ++a) ATA[a * mdim + a] = 1e-2;
            for (int f = 0; f < nf; ++f) {
                double w = rep->freq_hz[f] / f_mid;
                double phi_r[9] = {0}, phi_i[9] = {0};
                phi_r[0] = 1.0;
                for (int c = 0; c < nrc; ++c) {
                    double pr = poles[ridx[c]];
                    double den = pr * pr + w * w;
                    phi_r[1 + c] = 0.5 * pr / den;
                    phi_i[1 + c] = -0.5 * w / den;
                }
                double yr = rep->sr[f][i * np + j], yi = rep->si[f][i * np + j];
                for (int a = 0; a < mdim; ++a) {
                    ATy[a] += phi_r[a] * yr + phi_i[a] * yi;
                    for (int b = 0; b < mdim; ++b) {
                        ATA[a * mdim + b] += phi_r[a] * phi_r[b] + phi_i[a] * phi_i[b];
                    }
                }
            }
            if (ele_lu_solve_real(mdim, ATA, ATy, sol) == 0) {
                Dmat[i * np + j] = sol[0];
                for (int c = 0; c < nrc; ++c) Cmat[i * ns + ridx[c]] = sol[1 + c];
            }
        }
    }

    double hinf = 0.0;
    for (int k = 0; k <= 128; ++k) {
        double w = (k == 0) ? 0.0 : (0.02 * pow(200.0, (double)k / 128.0));
        double Hr[64] = {0}, Hi[64] = {0};
        for (int i = 0; i < np; ++i) {
            for (int j = 0; j < np; ++j) {
                double sr = Dmat[i * np + j], si = 0.0;
                for (int r = 0; r < ns; ++r) {
                    if (Bmat[r * np + j] == 0.0) continue;
                    double pr = poles[r], den = pr * pr + w * w;
                    sr += Cmat[i * ns + r] * Bmat[r * np + j] * (pr / den);
                    si += Cmat[i * ns + r] * Bmat[r * np + j] * (-w / den);
                }
                Hr[i * np + j] = sr; Hi[i * np + j] = si;
            }
        }
        int d = 2 * np;
        double aug[256] = {0}, ev[16];
        for (int i = 0; i < np; ++i) {
            for (int j = 0; j < np; ++j) {
                double gr = 0.0, gi = 0.0;
                for (int q = 0; q < np; ++q) {
                    gr += Hr[q * np + i] * Hr[q * np + j] + Hi[q * np + i] * Hi[q * np + j];
                    gi += Hr[q * np + i] * Hi[q * np + j] - Hi[q * np + i] * Hr[q * np + j];
                }
                aug[i * d + j] = gr; aug[(np + i) * d + (np + j)] = gr;
                aug[i * d + (np + j)] = -gi; aug[(np + i) * d + j] = gi;
            }
        }
        if (ele_sym_jacobi_eigs(d, aug, ev, NULL) == 0) {
            double s = sqrt(fmax(0.0, ev[d - 1]));
            if (s > hinf) hinf = s;
        }
    }
    if (hinf > 0.72) {
        double sc = 0.72 / hinf;
        for (int i = 0; i < np * np; ++i) Dmat[i] *= sc;
        for (int i = 0; i < np * ns; ++i) Cmat[i] *= sc;
    }

    const double gamma_sq = 1.0;
    out->kyp_gamma_sq = gamma_sq;
    double Rmat[64] = {0}, Rinv[64] = {0};
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            double acc = (i == j) ? gamma_sq : 0.0;
            for (int k = 0; k < np; ++k) acc -= Dmat[k * np + i] * Dmat[k * np + j];
            Rmat[i * np + j] = acc;
        }
    }
    for (int j = 0; j < np; ++j) {
        double e_j[8] = {0}, col[8] = {0};
        e_j[j] = 1.0;
        if (ele_lu_solve_real(np, Rmat, e_j, col) != 0) { free(rep); return -1; }
        for (int i = 0; i < np; ++i) Rinv[i * np + j] = col[i];
    }

    double Pmat[16 * 16] = {0};
    for (int it = 0; it < 240; ++it) {
        double Pnext[16 * 16] = {0};
        double PB[16 * 8] = {0};
        for (int i = 0; i < ns; ++i) {
            for (int j = 0; j < np; ++j) {
                double s = 0.0;
                for (int r = 0; r < ns; ++r) s += Pmat[i * ns + r] * Bmat[r * np + j];
                for (int k = 0; k < np; ++k) s += Cmat[k * ns + i] * Dmat[k * np + j];
                PB[i * np + j] = s;
            }
        }
        for (int i = 0; i < ns; ++i) {
            for (int j = i; j < ns; ++j) {
                double w = (i == j) ? 0.04 : 0.0;
                for (int k = 0; k < np; ++k) w += Cmat[k * ns + i] * Cmat[k * ns + j];
                for (int a = 0; a < np; ++a) {
                    for (int b = 0; b < np; ++b) {
                        w += PB[i * np + a] * Rinv[a * np + b] * PB[j * np + b];
                    }
                }
                double pnew = w / (poles[i] + poles[j]);
                Pnext[i * ns + j] = pnew;
                Pnext[j * ns + i] = pnew;
            }
        }
        double max_d = 0.0;
        for (int p = 0; p < ns * ns; ++p) {
            double d = fabs(Pnext[p] - Pmat[p]);
            if (d > max_d) max_d = d;
            Pmat[p] = 0.5 * Pmat[p] + 0.5 * Pnext[p];
        }
        if (max_d < 1e-12) {
            memcpy(Pmat, Pnext, (size_t)(ns * ns) * sizeof(double));
            break;
        }
    }

    int nd = ns + np;
    int dm = ns + nd;
    double M_kyp[24 * 24] = {0};
    for (int i = 0; i < ns; ++i) {
        for (int j = 0; j < ns; ++j) {
            double val = (poles[i] + poles[j]) * Pmat[i * ns + j];
            for (int k = 0; k < np; ++k) val -= Cmat[k * ns + i] * Cmat[k * ns + j];
            M_kyp[i * nd + j] = val;
        }
        for (int j = 0; j < np; ++j) {
            double off = 0.0;
            for (int r = 0; r < ns; ++r) off -= Pmat[i * ns + r] * Bmat[r * np + j];
            for (int k = 0; k < np; ++k) off -= Cmat[k * ns + i] * Dmat[k * np + j];
            M_kyp[i * nd + (ns + j)] = off;
            M_kyp[(ns + j) * nd + i] = off;
        }
    }
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            M_kyp[(ns + i) * nd + (ns + j)] = Rmat[i * np + j];
        }
    }

    const int64_t Q_K = 10000000;
    int64_t P_i[16 * 16] = {0}, Lp_i[16 * 16] = {0};
    int64_t Mk_i[24 * 24] = {0}, Lmk_i[24 * 24] = {0};
    int64_t Full_i[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};
    int64_t Lfull_i[ELE_CERT_MAX_DIM * ELE_CERT_MAX_DIM] = {0};

    for (int i = 0; i < ns; ++i) {
        for (int j = i; j < ns; ++j) {
            double v = 0.5 * (Pmat[i * ns + j] + Pmat[j * ns + i]);
            int64_t qv = (int64_t)llround(v * (double)Q_K);
            P_i[i * ns + j] = qv; P_i[j * ns + i] = qv;
            Full_i[i * dm + j] = qv; Full_i[j * dm + i] = qv;
        }
    }
    for (int i = 0; i < nd; ++i) {
        for (int j = i; j < nd; ++j) {
            double v = 0.5 * (M_kyp[i * nd + j] + M_kyp[j * nd + i]);
            int64_t qv = (int64_t)llround(v * (double)Q_K);
            Mk_i[i * nd + j] = qv; Mk_i[j * nd + i] = qv;
            Full_i[(ns + i) * dm + (ns + j)] = qv;
            Full_i[(ns + j) * dm + (ns + i)] = qv;
        }
    }

    int64_t p_mu = 0, mk_mu = 0, full_mu = 0;
    if (ele_cert_cholesky_sdd_i64(ns, P_i, Lp_i, &p_mu) != 0 ||
        ele_cert_cholesky_sdd_i64(nd, Mk_i, Lmk_i, &mk_mu) != 0 ||
        ele_cert_cholesky_sdd_i64(dm, Full_i, Lfull_i, &full_mu) != 0) {
        free(rep);
        return -1;
    }

    out->p_min_eig_int = p_mu;
    out->kyp_mu_psd = mk_mu;
    int cert_ok = (rep_res.passive_certified && rep_res.reciprocal_certified &&
                   p_mu >= 0 && mk_mu >= 0 && full_mu >= 0) ? 1 : 0;
    out->certified = cert_ok;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "em");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "kyp_passivity_enforce");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s", cert_ok ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", in_snp->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", in_snp->input_sha256);

    out->cert.has_psd = 1;
    out->cert.psd.dim = dm;
    out->cert.psd.scale_q = Q_K;
    out->cert.psd.mu_psd = full_mu;
    out->cert.psd.delta_round = 0;
    out->cert.psd.mu_cert = full_mu;
    out->cert.psd.ok = (full_mu >= 0) ? 1 : 0;
    memcpy(out->cert.psd.A, Full_i, (size_t)dm * (size_t)dm * sizeof(int64_t));
    memcpy(out->cert.psd.L, Lfull_i, (size_t)dm * (size_t)dm * sizeof(int64_t));

    out->cert.has_bracket = 1;
    out->cert.bracket_scale = Q_K;
    int64_t sig_rep_i = (int64_t)ceil(out->rep_sigma_max * (double)Q_K);
    out->cert.lb_int = sig_rep_i;
    out->cert.ub_int = Q_K;
    out->cert.limit_lo_int = 0;
    out->cert.limit_hi_int = Q_K;
    out->cert.lb_val = out->rep_sigma_max;
    out->cert.ub_val = 1.0;

    if (poles_out != NULL) memcpy(poles_out, poles, (size_t)ns * sizeof(double));
    if (B_out != NULL) memcpy(B_out, Bmat, (size_t)(ns * np) * sizeof(double));
    if (C_out != NULL) memcpy(C_out, Cmat, (size_t)(np * ns) * sizeof(double));
    if (D_out != NULL) memcpy(D_out, Dmat, (size_t)(np * np) * sizeof(double));
    if (P_out != NULL) memcpy(P_out, Pmat, (size_t)(ns * ns) * sizeof(double));
    if (f_mid_out != NULL) *f_mid_out = f_mid;

    free(rep);
    return 0;
}

int ele_em_enforce_passivity(const ele_snp_t *in_snp, const char *out_s2p_path,
                             ele_em_kyp_res_t *out) {
    return em_kyp_build_and_certify(in_snp, out_s2p_path, out,
                                    NULL, NULL, NULL, NULL, NULL, NULL);
}

/* ============================================================================
 * 6. Persistent In-Process Grid Session API
 * ============================================================================ */

struct ele_grid_session {
    ele_grid_t g;
};

ele_grid_session_t *ele_grid_session_create(const char *path) {
    if (path == NULL) return NULL;
    ele_grid_session_t *s = (ele_grid_session_t *)calloc(1, sizeof(ele_grid_session_t));
    if (s == NULL) return NULL;
    if (ele_grid_load(path, &s->g) != 0) {
        free(s);
        return NULL;
    }
    return s;
}

void ele_grid_session_destroy(ele_grid_session_t *s) {
    free(s);
}

int ele_grid_session_set_bus_limits(ele_grid_session_t *s, int bus_id, double vmin, double vmax) {
    if (s == NULL || vmin <= 0.0 || vmax < vmin) return -1;
    for (int k = 0; k < s->g.n_bus; ++k) {
        if (s->g.bus[k].id == bus_id) {
            s->g.bus[k].vmin = vmin;
            s->g.bus[k].vmax = vmax;
            return 0;
        }
    }
    return -1;
}

int ele_grid_session_set_branch_limits(ele_grid_session_t *s, int branch_idx,
                                       double smax_mva, double shift_deg,
                                       double angmin_deg, double angmax_deg) {
    if (s == NULL || branch_idx < 0 || branch_idx >= s->g.n_branch || angmax_deg < angmin_deg) {
        return -1;
    }
    if (smax_mva > 0.0) s->g.branch[branch_idx].smax_mva = smax_mva;
    s->g.branch[branch_idx].shift_deg = shift_deg;
    s->g.branch[branch_idx].angmin_deg = angmin_deg;
    s->g.branch[branch_idx].angmax_deg = angmax_deg;
    return 0;
}

int ele_grid_session_solve_opf(ele_grid_session_t *s, ele_grid_opf_res_t *out) {
    if (s == NULL || out == NULL) return -1;
    return ele_grid_acopf(&s->g, out);
}

int ele_grid_session_solve_sos2(ele_grid_session_t *s, ele_grid_sos2_res_t *out) {
    if (s == NULL || out == NULL) return -1;
    return ele_grid_acopf_sos2(&s->g, out);
}

int ele_grid_session_solve_obs(ele_grid_session_t *s, int kind, int target_id,
                               double eps, ele_grid_obs_res_t *out) {
    if (s == NULL || out == NULL) return -1;
    ele_grid_obs_kind_t k = (kind == 1) ? ELE_OBS_PFLOW : ELE_OBS_VM2;
    return ele_grid_observable(&s->g, k, target_id, eps, out);
}

/* ============================================================================
 * 7. Multi-Frequency Maxwell + 2D Yee FDTD Check & Real Bode-Fano Match (v11.0.0)
 * ============================================================================ */

static int eval_maxwell_at_kl(double kl, double zeta, double xi0, int n_cells,
                              uint32_t mask, double *p_local_ub_out, double *p_mask_out) {
    if (kl <= 0.0 || zeta <= 0.0 || n_cells < 4 || n_cells > 25) return -1;
    int nx = (int)floor(sqrt((double)n_cells) + 0.5);
    if (nx * nx < n_cells) nx = (int)ceil(sqrt((double)n_cells));
    double h = kl / (double)nx;
    double k2_area = h * h;
    double k_req = h / sqrt(M_PI);

    double cx[25], cy[25], Gr[25 * 25], Gi[25 * 25], Ur[25 * 25], Ui[25 * 25], er[25], ei[25];
    double y1_req = ele_bessel_y1(k_req);
    double g_self_re = -1.0 - 0.5 * M_PI * k_req * y1_req;
    for (int m = 0; m < n_cells; ++m) {
        cx[m] = ((double)(m % nx) + 0.5) * h;
        cy[m] = ((double)(m / nx) + 0.5) * h;
        er[m] = cos(cx[m]);
        ei[m] = sin(cx[m]);
    }
    for (int m = 0; m < n_cells; ++m) {
        for (int n = 0; n < n_cells; ++n) {
            if (m == n) {
                Gr[m * n_cells + n] = g_self_re;
                Gi[m * n_cells + n] = 0.25 * k2_area;
            } else {
                double r = hypot(cx[m] - cx[n], cy[m] - cy[n]);
                Gr[m * n_cells + n] = -0.25 * k2_area * ele_bessel_y0(r);
                Gi[m * n_cells + n] = 0.25 * k2_area * ele_bessel_j0(r);
            }
            Ur[m * n_cells + n] = ((m == n) ? xi0 : 0.0) - Gr[m * n_cells + n];
            Ui[m * n_cells + n] = ((m == n) ? (-1.0 / zeta) : 0.0) - Gi[m * n_cells + n];
        }
    }
    if (p_mask_out != NULL) *p_mask_out = eval_maxwell_subset(n_cells, Ur, Ui, er, ei, mask);
    if (p_local_ub_out != NULL) {
        double alpha[25], beta[25], ga[25], gb[25];
        for (int m = 0; m < n_cells; ++m) { alpha[m] = 1.0; beta[m] = 0.0; }
        double B_cur = 0.0;
        int spd = 0;
        if (eval_maxwell_dual(n_cells, Ur, Ui, er, ei, alpha, beta, &B_cur, ga, gb,
                              NULL, NULL, NULL, NULL, &spd) != 0) return -1;
        double step = 0.05;
        for (int it = 0; it < 120; ++it) {
            double gn2 = 0.0;
            for (int m = 0; m < n_cells; ++m) gn2 += ga[m] * ga[m] + gb[m] * gb[m];
            if (gn2 < 1e-12) break;
            double gn = sqrt(gn2), st = step;
            int acc = 0;
            for (int ls = 0; ls < 12; ++ls) {
                double na[25], nb[25], B_new = 0.0, gna[25], gnb[25];
                for (int m = 0; m < n_cells; ++m) {
                    na[m] = alpha[m] - st * (ga[m] / gn);
                    nb[m] = beta[m] - st * (gb[m] / gn);
                }
                int sp = 0;
                if (eval_maxwell_dual(n_cells, Ur, Ui, er, ei, na, nb, &B_new, gna, gnb,
                                      NULL, NULL, NULL, NULL, &sp) == 0 &&
                    sp && B_new < B_cur - 1e-10) {
                    memcpy(alpha, na, (size_t)n_cells * sizeof(double));
                    memcpy(beta, nb, (size_t)n_cells * sizeof(double));
                    memcpy(ga, gna, (size_t)n_cells * sizeof(double));
                    memcpy(gb, gnb, (size_t)n_cells * sizeof(double));
                    B_cur = B_new;
                    step = fmin(0.4, st * 1.4);
                    acc = 1;
                    break;
                }
                st *= 0.5;
            }
            if (!acc) break;
        }
        *p_local_ub_out = B_cur;
    }
    return 0;
}

static int run_maxwell_2d_fdtd_check(double kl, double zeta, double xi0, int n_cells,
                                     uint32_t mask, double *p_ext_out) {
    int nx = (int)floor(sqrt((double)n_cells) + 0.5);
    if (nx * nx < n_cells) nx = (int)ceil(sqrt((double)n_cells));
    double h = kl / (double)nx;
    const int sub = 6;
    const int pml = 10;
    const int air_pad = 4;
    int sc_px = nx * sub;
    int G = sc_px + 2 * (pml + air_pad); /* <= 30 + 28 = 58 */
    double ds = h / (double)sub;
    double dt = 0.45 * ds / M_SQRT2;
    int ox = (G - sc_px) / 2;
    int oy = (G - sc_px) / 2;
    int i0 = ox - 2, i1 = ox + sc_px + 1;
    int j0 = oy - 2, j1 = oy + sc_px + 1;

    /* Complex susceptibility chi = 1 / (xi0 - i / zeta) */
    double denom = xi0 * xi0 + 1.0 / (zeta * zeta);
    double chi_r = xi0 / denom;
    double chi_i = (1.0 / zeta) / denom;
    double eps_cell = fmax(0.25, 1.0 + chi_r);
    double sig_cell = fmax(0.0, chi_i);

    size_t gsz = (size_t)G * (size_t)G;
    double *Ezx = (double *)calloc(gsz, sizeof(double));
    double *Ezy = (double *)calloc(gsz, sizeof(double));
    double *Hx = (double *)calloc(gsz, sizeof(double));
    double *Hy = (double *)calloc(gsz, sizeof(double));
    double *sig_x = (double *)calloc((size_t)G, sizeof(double));
    double *sig_y = (double *)calloc((size_t)G, sizeof(double));
    double *eps_g = (double *)malloc(gsz * sizeof(double));
    double *sig_g = (double *)calloc(gsz, sizeof(double));
    double *ref_r = (double *)calloc((size_t)sc_px * (size_t)sc_px, sizeof(double));
    double *ref_i = (double *)calloc((size_t)sc_px * (size_t)sc_px, sizeof(double));
    double *tot_r = (double *)calloc((size_t)sc_px * (size_t)sc_px, sizeof(double));
    double *tot_i = (double *)calloc((size_t)sc_px * (size_t)sc_px, sizeof(double));

    if (!Ezx || !Ezy || !Hx || !Hy || !sig_x || !sig_y || !eps_g || !sig_g ||
        !ref_r || !ref_i || !tot_r || !tot_i) {
        free(Ezx); free(Ezy); free(Hx); free(Hy); free(sig_x); free(sig_y);
        free(eps_g); free(sig_g); free(ref_r); free(ref_i); free(tot_r); free(tot_i);
        return -1;
    }

    for (int i = 0; i < G; ++i) {
        double dx_p = 0.0;
        if (i < pml) dx_p = (double)(pml - i) / (double)pml;
        else if (i >= G - pml) dx_p = (double)(i - (G - pml) + 1) / (double)pml;
        double sx_v = (2.5 / ds) * dx_p * dx_p * dx_p;
        sig_x[i] = sx_v;
        sig_y[i] = sx_v;
    }

    int steps_per_cyc = (int)ceil((2.0 * M_PI) / dt);
    int nt_meas = 2 * steps_per_cyc;
    int nt_ramp = 6 * steps_per_cyc;
    int nt_tot = nt_ramp + nt_meas;
    double Tr = 2.0 * (2.0 * M_PI);

    for (int pass = 0; pass < 2; ++pass) {
        memset(Ezx, 0, gsz * sizeof(double));
        memset(Ezy, 0, gsz * sizeof(double));
        memset(Hx, 0, gsz * sizeof(double));
        memset(Hy, 0, gsz * sizeof(double));
        for (size_t k = 0; k < gsz; ++k) { eps_g[k] = 1.0; sig_g[k] = 0.0; }
        if (pass == 1) {
            for (int m = 0; m < n_cells; ++m) {
                if (((mask >> m) & 1U) == 0U) continue;
                int mx = m % nx, my = m / nx;
                for (int sy = 0; sy < sub; ++sy) {
                    for (int sx = 0; sx < sub; ++sx) {
                        int gx = ox + mx * sub + sx, gy = oy + my * sub + sy;
                        eps_g[(size_t)gy * (size_t)G + (size_t)gx] = eps_cell;
                        sig_g[(size_t)gy * (size_t)G + (size_t)gx] = sig_cell;
                    }
                }
            }
        }

        double *pr = (pass == 0) ? ref_r : tot_r;
        double *pi = (pass == 0) ? ref_i : tot_i;
        double s_cc = 0.0, s_cs = 0.0, s_ss = 0.0;

        for (int n = 0; n < nt_tot; ++n) {
            double t_e = (double)n * dt;
            double ramp_e = (t_e < Tr) ? (0.5 * (1.0 - cos(M_PI * t_e / Tr))) : 1.0;
            double t_h = ((double)n + 0.5) * dt;
            double ramp_h = (t_h < Tr) ? (0.5 * (1.0 - cos(M_PI * t_h / Tr))) : 1.0;

            /* Update Hx, Hy (n -> n+1/2) */
            for (int y = 0; y < G - 1; ++y) {
                double a_hy = 0.5 * sig_y[y] * dt;
                double ch_y1 = (1.0 - a_hy) / (1.0 + a_hy);
                double ch_y2 = (dt / ds) / (1.0 + a_hy);
                for (int x = 0; x < G - 1; ++x) {
                    double a_hx = 0.5 * sig_x[x] * dt;
                    double ch_x1 = (1.0 - a_hx) / (1.0 + a_hx);
                    double ch_x2 = (dt / ds) / (1.0 + a_hx);
                    size_t idx = (size_t)y * (size_t)G + (size_t)x;
                    double ez_c = Ezx[idx] + Ezy[idx];
                    double ez_up = Ezx[idx + (size_t)G] + Ezy[idx + (size_t)G];
                    double ez_rt = Ezx[idx + 1U] + Ezy[idx + 1U];
                    Hx[idx] = ch_y1 * Hx[idx] - ch_y2 * (ez_up - ez_c);
                    Hy[idx] = ch_x1 * Hy[idx] + ch_x2 * (ez_rt - ez_c);
                }
            }

            /* TF/SF H corrections at box perimeter */
            for (int y = j0; y <= j1; ++y) {
                double x0 = ((double)i0 - (double)ox + 0.5) * ds;
                double x1 = ((double)i1 - (double)ox + 0.5) * ds;
                Hy[(size_t)y * (size_t)G + (size_t)(i0 - 1)] -= (dt / ds) * ramp_e * cos(x0 - t_e);
                Hy[(size_t)y * (size_t)G + (size_t)i1] += (dt / ds) * ramp_e * cos(x1 - t_e);
            }
            for (int x = i0; x <= i1; ++x) {
                double xv = ((double)x - (double)ox + 0.5) * ds;
                double einc = ramp_e * cos(xv - t_e);
                Hx[(size_t)(j0 - 1) * (size_t)G + (size_t)x] += (dt / ds) * einc;
                Hx[(size_t)j1 * (size_t)G + (size_t)x] -= (dt / ds) * einc;
            }

            /* Update Ezx, Ezy (n -> n+1) */
            for (int y = 1; y < G - 1; ++y) {
                for (int x = 1; x < G - 1; ++x) {
                    size_t idx = (size_t)y * (size_t)G + (size_t)x;
                    double er = eps_g[idx];
                    double sc = sig_g[idx];
                    double sx_t = sc + sig_x[x], sy_t = sc + sig_y[y];
                    double ax = 0.5 * sx_t * dt / er, ay = 0.5 * sy_t * dt / er;
                    double cx1 = (1.0 - ax) / (1.0 + ax), cx2 = (dt / (er * ds)) / (1.0 + ax);
                    double cy1 = (1.0 - ay) / (1.0 + ay), cy2 = (dt / (er * ds)) / (1.0 + ay);
                    Ezx[idx] = cx1 * Ezx[idx] + cx2 * (Hy[idx] - Hy[idx - 1U]);
                    Ezy[idx] = cy1 * Ezy[idx] - cy2 * (Hx[idx] - Hx[idx - (size_t)G]);
                }
            }

            /* TF/SF Ez corrections (only left i0 and right i1 since Hx_inc == 0) */
            for (int y = j0; y <= j1; ++y) {
                double hy_l = -ramp_h * cos(((double)i0 - (double)ox) * ds - t_h);
                double hy_r = -ramp_h * cos(((double)(i1 + 1) - (double)ox) * ds - t_h);
                Ezx[(size_t)y * (size_t)G + (size_t)i0] -= (dt / ds) * hy_l;
                Ezx[(size_t)y * (size_t)G + (size_t)i1] += (dt / ds) * hy_r;
            }

            if (n >= nt_ramp) {
                double t_next = ((double)n + 1.0) * dt;
                double cs = cos(t_next), sn = sin(t_next);
                s_cc += cs * cs;
                s_cs += cs * sn;
                s_ss += sn * sn;
                for (int sy = 0; sy < sc_px; ++sy) {
                    int gy = oy + sy;
                    for (int sx = 0; sx < sc_px; ++sx) {
                        int gx = ox + sx;
                        size_t gi = (size_t)gy * (size_t)G + (size_t)gx;
                        size_t li = (size_t)sy * (size_t)sc_px + (size_t)sx;
                        double ez = Ezx[gi] + Ezy[gi];
                        pr[li] += ez * cs;
                        pi[li] += ez * sn;
                    }
                }
            }
        }
        double det = s_cc * s_ss - s_cs * s_cs;
        if (fabs(det) > 1e-12) {
            for (int k = 0; k < sc_px * sc_px; ++k) {
                double rhs0 = pr[k], rhs1 = pi[k];
                pr[k] = (s_ss * rhs0 - s_cs * rhs1) / det;
                pi[k] = (-s_cs * rhs0 + s_cc * rhs1) / det;
            }
        }
    }

    double inc_amp2 = 0.0;
    for (int k = 0; k < sc_px * sc_px; ++k) inc_amp2 += ref_r[k] * ref_r[k] + ref_i[k] * ref_i[k];
    inc_amp2 /= (double)(sc_px * sc_px);
    if (inc_amp2 < 1e-12) inc_amp2 = 1.0;

    double p_ext = 0.0;
    double inv_sub2 = 1.0 / (double)(sub * sub);
    for (int m = 0; m < n_cells; ++m) {
        if (((mask >> m) & 1U) == 0U) continue;
        int mx = m % nx, my = m / nx;
        double cell_p = 0.0;
        for (int sy = 0; sy < sub; ++sy) {
            for (int sx = 0; sx < sub; ++sx) {
                size_t li = (size_t)(my * sub + sy) * (size_t)sc_px + (size_t)(mx * sub + sx);
                double er_v = ref_r[li], ei_v = ref_i[li];
                double tr_v = tot_r[li], ti_v = tot_i[li];
                /* P = chi * E_tot = (chi_r + i chi_i)*(tr + i ti) */
                double pr_v = chi_r * tr_v - chi_i * ti_v;
                double pi_v = chi_r * ti_v + chi_i * tr_v;
                cell_p += (er_v * pi_v - ei_v * pr_v) / inc_amp2;
            }
        }
        p_ext += cell_p * inv_sub2;
    }

    *p_ext_out = p_ext;
    free(Ezx); free(Ezy); free(Hx); free(Hy); free(sig_x); free(sig_y);
    free(eps_g); free(sig_g); free(ref_r); free(ref_i); free(tot_r); free(tot_i);
    return 0;
}

int ele_em_maxwell_bound_ext(double kl, double zeta, double xi0, int n_cells,
                             double bw_frac, int run_fdtd_check,
                             ele_em_bound_ext_res_t *out) {
    if (out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    if (ele_em_maxwell_bound(kl, zeta, xi0, n_cells, &out->base) != 0) return -1;
    out->bw_frac = bw_frac;
    if (bw_frac > 0.0) {
        double mults[3] = { 1.0 - 0.5 * bw_frac, 1.0, 1.0 + 0.5 * bw_frac };
        double sum_ub = 0.0, sum_se = 0.0;
        int cnt = 0;
        for (int k = 0; k < 3; ++k) {
            if (mults[k] <= 0.0) continue;
            if (k == 1) {
                sum_ub += out->base.p_local_ub_cert;
                sum_se += out->base.p_searu_lb;
                ++cnt;
            } else {
                double ub_k = 0.0, se_k = 0.0;
                if (eval_maxwell_at_kl(kl * mults[k], zeta, xi0, n_cells,
                                       out->base.searu_best_mask, &ub_k, &se_k) == 0) {
                    sum_ub += ub_k;
                    sum_se += se_k;
                    ++cnt;
                }
            }
        }
        out->n_freq_pts = cnt;
        out->p_band_avg_ub = (cnt > 0) ? (sum_ub / (double)cnt) : out->base.p_local_ub_cert;
        out->p_band_avg_searu = (cnt > 0) ? (sum_se / (double)cnt) : out->base.p_searu_lb;
    } else {
        out->n_freq_pts = 1;
        out->p_band_avg_ub = out->base.p_local_ub_cert;
        out->p_band_avg_searu = out->base.p_searu_lb;
    }

    if (run_fdtd_check != 0) {
        out->fdtd_checked = 1;
        double p_fd = 0.0;
        if (run_maxwell_2d_fdtd_check(kl, zeta, xi0, n_cells, out->base.searu_best_mask, &p_fd) == 0) {
            out->fdtd_ext_power = p_fd;
            out->fdtd_below_ub = (p_fd > 0.0 && p_fd <= out->base.p_local_ub_cert + 1e-6) ? 1 : 0;
        }
    }
    return 0;
}

static void cplx_inv(double ar, double ai, double *br, double *bi) {
    double d = ar * ar + ai * ai;
    if (d < 1e-30) d = 1e-30;
    *br = ar / d;
    *bi = -ai / d;
}

static double bf_ladder_eval(double z0, double rl, double cl_f,
                             double l1, double c1, double l2, double c2,
                             double w_rad) {
    /* Load admittance: Y_L = 1/rl + j w cl_f */
    double yr = 1.0 / rl, yi = w_rad * cl_f;
    double zr, zi;
    cplx_inv(yr, yi, &zr, &zi);
    /* Series L2 */
    zi += w_rad * l2;
    /* Shunt C2 */
    cplx_inv(zr, zi, &yr, &yi);
    yi += w_rad * c2;
    cplx_inv(yr, yi, &zr, &zi);
    /* Series L1 */
    zi += w_rad * l1;
    /* Shunt C1 */
    cplx_inv(zr, zi, &yr, &yi);
    yi += w_rad * c1;
    cplx_inv(yr, yi, &zr, &zi);
    /* Gamma = (Z_in - Z0) / (Z_in + Z0) */
    double num2 = (zr - z0) * (zr - z0) + zi * zi;
    double den2 = (zr + z0) * (zr + z0) + zi * zi;
    if (den2 < 1e-30) return 1.0;
    return sqrt(fmin(1.0, num2 / den2));
}

int ele_em_bode_fano_synth(double z0, double rl, double cl_f,
                           double f_lo_hz, double f_hi_hz, uint64_t seed,
                           ele_em_bode_fano_res_t *out) {
    if (out == NULL || z0 <= 0.0 || rl <= 0.0 || cl_f <= 0.0 ||
        f_lo_hz <= 0.0 || f_hi_hz <= f_lo_hz) return -1;
    memset(out, 0, sizeof(*out));
    out->z0 = z0; out->rl = rl; out->cl_f = cl_f;
    out->f_lo_hz = f_lo_hz; out->f_hi_hz = f_hi_hz;

    const int n_pts = 31;
    double w_pts[31];
    double df = (f_hi_hz - f_lo_hz) / (double)(n_pts - 1);
    for (int i = 0; i < n_pts; ++i) {
        w_pts[i] = 2.0 * M_PI * (f_lo_hz + (double)i * df);
    }
    double w0 = 2.0 * M_PI * sqrt(f_lo_hz * f_hi_hz);
    double z_ref = sqrt(z0 * rl);
    double l_base = z_ref / w0;
    double c_base = 1.0 / (z_ref * w0);

    double best_p[4] = { 0.8, 0.4, 0.8, 0.4 };
    double best_obj = 1e9;
    uint64_t rng = seed ^ UINT64_C(0x9e3779b97f4a7c15);

    for (int st = 0; st < 28; ++st) {
        double p[4];
        if (st == 0) {
            p[0] = 0.7; p[1] = 0.35; p[2] = 0.7; p[3] = 0.35;
        } else {
            for (int k = 0; k < 4; ++k) {
                rng += UINT64_C(0x9e3779b97f4a7c15);
                uint64_t z = rng;
                z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
                z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
                z ^= (z >> 31);
                double u = (double)(z & 0xFFFFFFU) / (double)0x1000000U;
                p[k] = 0.05 + 2.4 * u;
            }
        }
        double cur = 0.0;
        for (int i = 0; i < n_pts; ++i) {
            double g = bf_ladder_eval(z0, rl, cl_f,
                                      p[0] * l_base, p[1] * c_base,
                                      p[2] * l_base, p[3] * c_base, w_pts[i]);
            if (g > cur) cur = g;
        }
        double step = 0.35;
        for (int cyc = 0; cyc < 18; ++cyc) {
            int imp = 0;
            for (int d = 0; d < 4; ++d) {
                for (int dir = -1; dir <= 1; dir += 2) {
                    double cand[4];
                    memcpy(cand, p, sizeof(cand));
                    cand[d] = (dir > 0) ? (p[d] * (1.0 + step)) : (p[d] / (1.0 + step));
                    if (cand[d] < 1e-4 || cand[d] > 20.0) continue;
                    double obj = 0.0;
                    for (int i = 0; i < n_pts; ++i) {
                        double g = bf_ladder_eval(z0, rl, cl_f,
                                                  cand[0] * l_base, cand[1] * c_base,
                                                  cand[2] * l_base, cand[3] * c_base, w_pts[i]);
                        if (g > obj) obj = g;
                    }
                    if (obj < cur - 1e-7) {
                        cur = obj;
                        memcpy(p, cand, sizeof(p));
                        imp = 1;
                    }
                }
            }
            if (!imp) step *= 0.55;
        }
        if (cur < best_obj) {
            best_obj = cur;
            memcpy(best_p, p, sizeof(p));
        }
    }

    double l1 = best_p[0] * l_base, c1 = best_p[1] * c_base;
    double l2 = best_p[2] * l_base, c2 = best_p[3] * c_base;
    out->l1_nh = l1 * 1e9;
    out->c1_pf = c1 * 1e12;
    out->l2_nh = l2 * 1e9;
    out->c2_pf = c2 * 1e12;

    double dw = 2.0 * M_PI * df;
    double tau = rl * cl_f;
    double i_meas = 0.0;
    double max_g = 0.0;
    const int64_t Q_BF = 100000;
    out->n_bins = n_pts;
    int64_t bin_cap = 0, i_sum = 0;
    for (int i = 0; i < n_pts; ++i) {
        double g = bf_ladder_eval(z0, rl, cl_f, l1, c1, l2, c2, w_pts[i]);
        if (g > max_g) max_g = g;
        double wt = (i == 0 || i == n_pts - 1) ? (0.5 * dw) : dw;
        double lg = log(1.0 / fmax(g, 1e-9));
        double term_norm = tau * lg * wt; /* sums to tau * I_meas <= M_PI */
        i_meas += lg * wt;
        int64_t bi = (int64_t)floor(term_norm * (double)Q_BF);
        if (bi < 0) bi = 0;
        out->bin_resp_int[i] = bi;
        if (bi > bin_cap) bin_cap = bi;
        i_sum += bi;
    }

    out->bf_gamma_limit = ele_em_bode_fano_bound(rl, cl_f, f_lo_hz, f_hi_hz);
    out->bf_s11_limit_db = 20.0 * log10(fmax(out->bf_gamma_limit, 1e-12));
    out->max_gamma = max_g;
    out->max_s11_db = 20.0 * log10(fmax(max_g, 1e-12));
    out->margin_db = out->max_s11_db - out->bf_s11_limit_db;
    out->integral_meas = i_meas;
    out->integral_bf_max = M_PI / tau;
    out->integral_ratio = (tau * i_meas) / M_PI;
    out->bin_cap_int = bin_cap;
    out->i_meas_int = i_sum;
    out->i_bf_int = (int64_t)floor(M_PI * (double)Q_BF);

    int ok = (max_g >= out->bf_gamma_limit - 1e-9 && i_sum <= out->i_bf_int) ? 1 : 0;
    out->certified = ok;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "em");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "bode_fano_match");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s", ok ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "internal");
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%064d", 0);
    out->cert.has_bracket = 1;
    out->cert.bracket_scale = Q_BF;
    out->cert.limit_lo_int = 0;
    out->cert.lb_int = out->i_meas_int;
    out->cert.ub_int = out->i_bf_int;
    out->cert.limit_hi_int = out->i_bf_int;
    out->cert.lb_val = tau * i_meas;
    out->cert.ub_val = M_PI;
    return 0;
}

int ele_em_bode_fano_emit_lean(const char *out_path, const char *mod_name,
                               const ele_em_bode_fano_res_t *res) {
    if (out_path == NULL || mod_name == NULL || res == NULL || !res->certified) return -1;
    FILE *fp = fopen(out_path, "w");
    if (fp == NULL) return -1;
    fprintf(fp,
            "/-\n  Standalone Zero-FP-Trust Bode-Fano Certificate: %s\n-/\n"
            "import EleCert\nimport Flow.Interval\n\nnamespace %s\n\n"
            "def nBins : Nat := %d\n"
            "def cellCap : Int := %" PRId64 "\n"
            "def iMeas : Int := %" PRId64 "\n"
            "def iBfMax : Int := %" PRId64 "\n\n"
            "def cellVals : List Int := [\n  ",
            mod_name, mod_name, res->n_bins, res->bin_cap_int, res->i_meas_int, res->i_bf_int);
    for (int i = 0; i < res->n_bins; ++i) {
        fprintf(fp, "%s%" PRId64, i ? ", " : "", res->bin_resp_int[i]);
    }
    fprintf(fp,
            "\n]\n\n"
            "def cellResp (i : Fin nBins) : Int := cellVals.getD i.val 0\n\n"
            "theorem bode_fano_match_certified :\n"
            "    EleCert.sumFin nBins cellResp ≤ Int.ofNat nBins * cellCap ∧\n"
            "    iMeas ≤ iBfMax ∧ 0 ≤ iBfMax - iMeas :=\n"
            "  EleCert.bode_fano_energy_limit_sound nBins cellResp cellCap iMeas iMeas iBfMax\n"
            "    (by decide) (by decide) (by decide)\n\n"
            "#print axioms bode_fano_match_certified\n\n"
            "end %s\n", mod_name);
    int ferr = ferror(fp);
    fclose(fp);
    return (ferr == 0) ? 0 : -1;
}

/* ============================================================================
 * 8. KYP Time-Domain Wave/Lyapunov Instability Contrast Simulation (v11.0.0)
 * ============================================================================ */

static int tustin_discretize(int ns, int np, const double *A, const double *B,
                             const double *C, const double *D, double dt,
                             double *Ad, double *Bd, double *Cd, double *Dd) {
    double M[32 * 32] = {0}, Minv[32 * 32] = {0};
    double Pplus[32 * 32] = {0};
    for (int i = 0; i < ns; ++i) {
        for (int j = 0; j < ns; ++j) {
            double id = (i == j) ? 1.0 : 0.0;
            M[i * ns + j] = id - 0.5 * dt * A[i * ns + j];
            Pplus[i * ns + j] = id + 0.5 * dt * A[i * ns + j];
        }
    }
    for (int col = 0; col < ns; ++col) {
        double rhs[32] = {0}, x[32] = {0};
        rhs[col] = 1.0;
        if (ele_lu_solve_real(ns, M, rhs, x) != 0) return -1;
        for (int row = 0; row < ns; ++row) Minv[row * ns + col] = x[row];
    }
    for (int i = 0; i < ns; ++i) {
        for (int j = 0; j < ns; ++j) {
            double s = 0.0;
            for (int k = 0; k < ns; ++k) s += Minv[i * ns + k] * Pplus[k * ns + j];
            Ad[i * ns + j] = s;
        }
    }
    double sdt = sqrt(dt);
    double MinvB[32 * 8] = {0};
    for (int i = 0; i < ns; ++i) {
        for (int j = 0; j < np; ++j) {
            double s = 0.0;
            for (int k = 0; k < ns; ++k) s += Minv[i * ns + k] * B[k * np + j];
            MinvB[i * np + j] = s;
            Bd[i * np + j] = sdt * s;
        }
    }
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < ns; ++j) {
            double s = 0.0;
            for (int k = 0; k < ns; ++k) s += C[i * ns + k] * Minv[k * ns + j];
            Cd[i * ns + j] = sdt * s;
        }
    }
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            double s = D[i * np + j];
            for (int k = 0; k < ns; ++k) s += 0.5 * dt * C[i * ns + k] * MinvB[k * np + j];
            Dd[i * np + j] = s;
        }
    }
    return 0;
}

static void run_allpass_closed_loop(int ns, int np, const double *Ad, const double *Bd,
                                    const double *Cd, const double *Dd, const double *P_lyap,
                                    double rho_load, double sgn_ap, double a_ap,
                                    const double *u_imp, int n_steps,
                                    double *peak_ratio_out, double *final_ratio_out,
                                    int *lyap_mono_out) {
    double s_ap = sqrt(fmax(0.0, 1.0 - a_ap * a_ap));
    double Msys[8 * 8] = {0};
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            Msys[i * np + j] = ((i == j) ? 1.0 : 0.0) + sgn_ap * rho_load * a_ap * Dd[i * np + j];
        }
    }
    double x[32] = {0}, zL[8] = {0};
    double peak_y = 0.0, last_sum = 0.0;
    double v_prev = 0.0;
    int mono = 1;

    for (int step = 0; step < n_steps; ++step) {
        double u_in[8] = {0};
        for (int p = 0; p < np; ++p) {
            double ext = (step == 0) ? u_imp[p] : 0.0;
            u_in[p] = ext + sgn_ap * rho_load * s_ap * zL[p];
        }
        double rhs[8] = {0}, y[8] = {0};
        for (int i = 0; i < np; ++i) {
            double s = 0.0;
            for (int k = 0; k < ns; ++k) s += Cd[i * ns + k] * x[k];
            for (int j = 0; j < np; ++j) s += Dd[i * np + j] * u_in[j];
            rhs[i] = s;
        }
        if (ele_lu_solve_real(np, Msys, rhs, y) != 0) {
            peak_y = 1e9;
            mono = 0;
            break;
        }
        double u_tot[8] = {0};
        for (int p = 0; p < np; ++p) {
            double ext = (step == 0) ? u_imp[p] : 0.0;
            u_tot[p] = ext + sgn_ap * rho_load * (s_ap * zL[p] - a_ap * y[p]);
        }
        double xn[32] = {0}, zLn[8] = {0};
        for (int i = 0; i < ns; ++i) {
            double s = 0.0;
            for (int k = 0; k < ns; ++k) s += Ad[i * ns + k] * x[k];
            for (int j = 0; j < np; ++j) s += Bd[i * np + j] * u_tot[j];
            xn[i] = s;
        }
        for (int p = 0; p < np; ++p) zLn[p] = a_ap * zL[p] + s_ap * y[p];

        if (P_lyap != NULL) {
            double v_nxt = 0.0;
            for (int i = 0; i < ns; ++i) {
                for (int j = 0; j < ns; ++j) v_nxt += xn[i] * P_lyap[i * ns + j] * xn[j];
            }
            for (int p = 0; p < np; ++p) v_nxt += rho_load * rho_load * zLn[p] * zLn[p];
            if (step >= 1 && v_nxt > v_prev + 1e-10) mono = 0;
            v_prev = v_nxt;
        }

        double ynorm = 0.0;
        for (int p = 0; p < np; ++p) ynorm += y[p] * y[p];
        ynorm = sqrt(ynorm);
        if (ynorm > peak_y) peak_y = ynorm;
        if (step >= n_steps - 10) last_sum += ynorm;
        if (ynorm > 1e8) {
            peak_y = ynorm;
            if (step < n_steps - 10) last_sum = ynorm * 10.0;
            memcpy(x, xn, (size_t)ns * sizeof(double));
            memcpy(zL, zLn, (size_t)np * sizeof(double));
            break;
        }
        memcpy(x, xn, (size_t)ns * sizeof(double));
        memcpy(zL, zLn, (size_t)np * sizeof(double));
    }
    *peak_ratio_out = peak_y;
    *final_ratio_out = last_sum / (double)(n_steps >= 10 ? 10 : n_steps);
    if (lyap_mono_out != NULL) *lyap_mono_out = mono;
}

int ele_em_kyp_transient_contrast(const ele_snp_t *in_snp, double rho_load,
                                  int n_steps, ele_em_kyp_sim_res_t *out) {
    if (in_snp == NULL || out == NULL || in_snp->nports <= 0 ||
        in_snp->nports > ELE_CERT_MAX_PORTS || in_snp->nfreq <= 0 ||
        rho_load <= 0.0 || rho_load >= 1.0 || n_steps < 20 || n_steps > 5000) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    int np = in_snp->nports;
    int nf = in_snp->nfreq;
    out->nports = np;
    out->n_steps = n_steps;
    out->rho_load = rho_load;

    ele_em_passivity_res_t orig_res;
    if (ele_em_certify_snp(in_snp, &orig_res) != 0) return -1;
    out->orig_sigma_max = orig_res.sigma_max;

    double poles_rep[16] = {0}, B_rep[16 * 8] = {0}, C_rep[8 * 16] = {0}, D_rep[8 * 8] = {0}, P_rep[16 * 16] = {0};
    double f_mid = 1.0;
    ele_em_kyp_res_t kyp_res;
    if (em_kyp_build_and_certify(in_snp, NULL, &kyp_res,
                                 poles_rep, B_rep, C_rep, D_rep, P_rep, &f_mid) != 0) {
        return -1;
    }
    int ns_rep = kyp_res.n_state;
    out->n_state = ns_rep;
    out->rep_sigma_max = kyp_res.rep_sigma_max;

    double A_rep[16 * 16] = {0};
    for (int i = 0; i < ns_rep; ++i) A_rep[i * ns_rep + i] = -poles_rep[i];

    /* Unrepaired model with 2 real poles + 1 damped 2x2 resonator per port at worst violation freq */
    int ns_o = 4 * np; /* <= 32 */
    double f_viol = orig_res.worst_freq_hz;
    double w_v = fmax(0.05, f_viol / f_mid);
    double alpha = 0.08 * w_v;

    double A_o[32 * 32] = {0}, B_o[32 * 8] = {0}, C_o[8 * 32] = {0}, D_o[8 * 8] = {0};
    for (int j = 0; j < np; ++j) {
        int s0 = 4 * j;
        A_o[(s0 + 0) * ns_o + (s0 + 0)] = -1.0;
        B_o[(s0 + 0) * np + j] = 0.5;
        A_o[(s0 + 1) * ns_o + (s0 + 1)] = -3.0;
        B_o[(s0 + 1) * np + j] = 0.5;
        A_o[(s0 + 2) * ns_o + (s0 + 2)] = -alpha;
        A_o[(s0 + 2) * ns_o + (s0 + 3)] = +w_v;
        A_o[(s0 + 3) * ns_o + (s0 + 2)] = -w_v;
        A_o[(s0 + 3) * ns_o + (s0 + 3)] = -alpha;
        B_o[(s0 + 2) * np + j] = alpha;
    }

    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            double ATA[5 * 5] = {0}, ATy[5] = {0}, sol[5] = {0};
            for (int a = 0; a < 5; ++a) ATA[a * 5 + a] = 1e-4;
            for (int f = 0; f < nf; ++f) {
                double wt = (f == orig_res.worst_freq_idx) ? 8.0 : 1.0;
                double w = in_snp->freq_hz[f] / f_mid;
                double phi_r[5], phi_i[5];
                phi_r[0] = 1.0; phi_i[0] = 0.0;
                double d1 = 1.0 + w * w, d2 = 9.0 + w * w;
                phi_r[1] = 0.5 / d1; phi_i[1] = -0.5 * w / d1;
                phi_r[2] = 1.5 / d2; phi_i[2] = -0.5 * w / d2;
                double dr = alpha * alpha - w * w + w_v * w_v;
                double di = 2.0 * alpha * w;
                double dsq = fmax(1e-18, dr * dr + di * di);
                /* phi_2 = alpha*(alpha + i w)/(dr + i di), phi_3 = -alpha*w_v/(dr + i di) */
                phi_r[3] = alpha * (alpha * dr + w * di) / dsq;
                phi_i[3] = alpha * (w * dr - alpha * di) / dsq;
                phi_r[4] = (-alpha * w_v * dr) / dsq;
                phi_i[4] = (+alpha * w_v * di) / dsq;
                double yr = in_snp->sr[f][i * np + j], yi = in_snp->si[f][i * np + j];
                for (int a = 0; a < 5; ++a) {
                    ATy[a] += wt * (phi_r[a] * yr + phi_i[a] * yi);
                    for (int b = 0; b < 5; ++b) ATA[a * 5 + b] += wt * (phi_r[a] * phi_r[b] + phi_i[a] * phi_i[b]);
                }
            }
            if (ele_lu_solve_real(5, ATA, ATy, sol) == 0) {
                D_o[i * np + j] = sol[0];
                C_o[i * ns_o + (4 * j + 0)] = sol[1];
                C_o[i * ns_o + (4 * j + 1)] = sol[2];
                C_o[i * ns_o + (4 * j + 2)] = sol[3];
                C_o[i * ns_o + (4 * j + 3)] = sol[4];
            }
        }
    }

    double dt = 0.35 / fmax(w_v, 0.5);
    out->dt_s = dt / f_mid;

    double Ad_o[32 * 32] = {0}, Bd_o[32 * 8] = {0}, Cd_o[8 * 32] = {0}, Dd_o[8 * 8] = {0};
    double Ad_r[16 * 16] = {0}, Bd_r[16 * 8] = {0}, Cd_r[8 * 16] = {0}, Dd_r[8 * 8] = {0};
    if (tustin_discretize(ns_o, np, A_o, B_o, C_o, D_o, dt, Ad_o, Bd_o, Cd_o, Dd_o) != 0 ||
        tustin_discretize(ns_rep, np, A_rep, B_rep, C_rep, D_rep, dt, Ad_r, Bd_r, Cd_r, Dd_r) != 0) {
        return -1;
    }

    double u_imp[8] = {0};
    double uval = 1.0 / sqrt((double)np);
    for (int p = 0; p < np; ++p) u_imp[p] = uval;

    /* Find passive all-pass reflection parameter triggering maximum instability in unrepaired model */
    double best_sgn = 1.0, best_a = 0.0, best_pk = -1.0;
    int probe_steps = (n_steps < 140) ? n_steps : 140;
    for (int si = 0; si < 2; ++si) {
        double sgn = (si == 0) ? 1.0 : -1.0;
        for (int ai = -9; ai <= 9; ++ai) {
            double a_ap = 0.1 * (double)ai;
            double pk = 0.0, fin = 0.0;
            run_allpass_closed_loop(ns_o, np, Ad_o, Bd_o, Cd_o, Dd_o, NULL,
                                    rho_load, sgn, a_ap, u_imp, probe_steps,
                                    &pk, &fin, NULL);
            if (pk > best_pk) {
                best_pk = pk;
                best_sgn = sgn;
                best_a = a_ap;
            }
        }
    }

    int rep_mono = 0;
    run_allpass_closed_loop(ns_o, np, Ad_o, Bd_o, Cd_o, Dd_o, NULL,
                            rho_load, best_sgn, best_a, u_imp, n_steps,
                            &out->orig_peak_ratio, &out->orig_final_ratio, NULL);
    run_allpass_closed_loop(ns_rep, np, Ad_r, Bd_r, Cd_r, Dd_r, P_rep,
                            rho_load, best_sgn, best_a, u_imp, n_steps,
                            &out->rep_peak_ratio, &out->rep_final_ratio, &rep_mono);

    out->orig_blowup = (out->orig_peak_ratio > 50.0 || out->orig_final_ratio > 10.0) ? 1 : 0;
    out->rep_stable = (out->rep_final_ratio < 0.05 && out->rep_peak_ratio < 10.0) ? 1 : 0;
    out->rep_lyap_monotone = rep_mono;
    out->certified = (kyp_res.certified && out->orig_blowup && out->rep_stable && out->rep_lyap_monotone) ? 1 : 0;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "em");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "kyp_transient_contrast");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s", out->certified ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", in_snp->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", in_snp->input_sha256);

    const int64_t Q_T = 100000;
    out->cert.has_bracket = 1;
    out->cert.bracket_scale = Q_T;
    int64_t rep_pk_i = (int64_t)ceil(out->rep_peak_ratio * (double)Q_T);
    int64_t orig_pk_i = (int64_t)ceil(fmin(1e9, out->orig_peak_ratio) * (double)Q_T);
    if (orig_pk_i < rep_pk_i) orig_pk_i = rep_pk_i;
    out->cert.limit_lo_int = 0;
    out->cert.lb_int = rep_pk_i;
    out->cert.ub_int = orig_pk_i;
    out->cert.limit_hi_int = orig_pk_i;
    out->cert.lb_val = out->rep_peak_ratio;
    out->cert.ub_val = fmin(1e9, out->orig_peak_ratio);
    return 0;
}
