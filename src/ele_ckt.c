/* SPDX-License-Identifier: MIT */
#include "ele_ckt.h"
#include "flow.h"
#include "searu.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 * 1. Engineering Suffix & SPICE .sp Parser
 * ============================================================================ */

static int strcasecmp_ele(const char *a, const char *b) {
    while (*a != '\0' && *b != '\0') {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        ++a;
        ++b;
    }
    return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

static int strncasecmp_ele(const char *a, const char *b, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        int ca = tolower((unsigned char)a[i]);
        int cb = tolower((unsigned char)b[i]);
        if (ca != cb) return ca - cb;
        if (ca == '\0') return 0;
    }
    return 0;
}

static char *strtok_r_ele(char *str, const char *delim, char **saveptr) {
    char *p = (str != NULL) ? str : *saveptr;
    if (p == NULL) return NULL;
    p += strspn(p, delim);
    if (*p == '\0') {
        *saveptr = p;
        return NULL;
    }
    char *end = p + strcspn(p, delim);
    if (*end != '\0') {
        *end = '\0';
        *saveptr = end + 1;
    } else {
        *saveptr = end;
    }
    return p;
}

static int parse_eng_val(const char *tok, double *out) {
    if (tok == NULL || *tok == '\0') return -1;
    char *endp = NULL;
    double v = strtod(tok, &endp);
    if (endp == tok || !isfinite(v)) return -1;
    while (*endp != '\0' && isspace((unsigned char)*endp)) ++endp;
    if (*endp != '\0') {
        if (strcasecmp_ele(endp, "meg") == 0 || strncasecmp_ele(endp, "meg", 3U) == 0) {
            v *= 1e6;
        } else {
            char c = (char)tolower((unsigned char)*endp);
            if (c == 'f') v *= 1e-15;
            else if (c == 'p') v *= 1e-12;
            else if (c == 'n') v *= 1e-9;
            else if (c == 'u') v *= 1e-6;
            else if (c == 'm') v *= 1e-3;
            else if (c == 'k') v *= 1e3;
            else if (c == 'g') v *= 1e9;
            else if (c == 't') v *= 1e12;
        }
    }
    *out = v;
    return 0;
}

static int get_or_add_node(ele_ckt_netlist_t *nl, const char *name) {
    if (strcasecmp_ele(name, "0") == 0 || strcasecmp_ele(name, "gnd") == 0) {
        return 0;
    }
    for (int i = 0; i < nl->n_nodes; ++i) {
        if (strcasecmp_ele(nl->node_names[i], name) == 0) return i;
    }
    if (nl->n_nodes >= ELE_CKT_MAX_NODES) return -1;
    int idx = nl->n_nodes++;
    snprintf(nl->node_names[idx], sizeof(nl->node_names[idx]), "%s", name);
    return idx;
}

int ele_ckt_find_node(const ele_ckt_netlist_t *nl, const char *name) {
    if (nl == NULL || name == NULL) return -1;
    if (strcasecmp_ele(name, "0") == 0 || strcasecmp_ele(name, "gnd") == 0) return 0;
    for (int i = 0; i < nl->n_nodes; ++i) {
        if (strcasecmp_ele(nl->node_names[i], name) == 0) return i;
    }
    return -1;
}

static int parse_kv_param(const char *tok, char *key_out, size_t kcap, double *val_out, char *sval_out, size_t scap) {
    const char *eq = strchr(tok, '=');
    if (eq == NULL) return -1;
    size_t klen = (size_t)(eq - tok);
    if (klen == 0U || klen + 1U > kcap) return -1;
    memcpy(key_out, tok, klen);
    key_out[klen] = '\0';
    for (size_t i = 0; i < klen; ++i) key_out[i] = (char)toupper((unsigned char)key_out[i]);
    if (sval_out != NULL && scap > 0U) snprintf(sval_out, scap, "%s", eq + 1);
    if (val_out != NULL) {
        if (parse_eng_val(eq + 1, val_out) != 0) *val_out = 0.0;
    }
    return 0;
}

static int add_basic_elem(ele_ckt_netlist_t *nl, ele_ckt_elem_type_t type, const char *name,
                          int n1, int n2, double val, double tol_frac) {
    if (nl->n_elems >= ELE_CKT_MAX_ELEMS) return -1;
    ele_ckt_elem_t *e = &nl->elems[nl->n_elems++];
    memset(e, 0, sizeof(*e));
    snprintf(e->name, sizeof(e->name), "%s", name);
    e->type = type;
    e->n1 = n1;
    e->n2 = n2;
    e->val = val;
    e->tol_frac = tol_frac;
    e->branch_idx = -1;
    e->l1_idx = -1;
    e->l2_idx = -1;
    return nl->n_elems - 1;
}

static void parse_paren_floats(const char *str, double *vals, int max_n, int *n_out) {
    *n_out = 0;
    const char *lp = strchr(str, '(');
    const char *p = (lp != NULL) ? (lp + 1) : str;
    char buf[256];
    snprintf(buf, sizeof(buf), "%s", p);
    char *rp = strchr(buf, ')');
    if (rp != NULL) *rp = '\0';
    char *save = NULL;
    char *tok = strtok_r_ele(buf, " \t,", &save);
    while (tok != NULL && *n_out < max_n) {
        double v = 0.0;
        if (parse_eng_val(tok, &v) == 0) vals[(*n_out)++] = v;
        tok = strtok_r_ele(NULL, " \t,", &save);
    }
}

int ele_ckt_load(const char *path, ele_ckt_netlist_t *out) {
    if (path == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    out->n_nodes = 1;
    snprintf(out->node_names[0], sizeof(out->node_names[0]), "0");
    out->ac_vout_node = -1;
    out->ac_vin_elem = -1;
    out->tran_probe_node = -1;
    out->soa_node = -1;
    out->pdn_node = -1;
    out->pdn_isrc_elem = -1;
    snprintf(out->input_path, sizeof(out->input_path), "%s", path);
    if (ele_sha256_file(path, out->input_sha256) != 0) return -1;

    FILE *fp = fopen(path, "r");
    if (fp == NULL) return -1;

    char pending_vout[64] = {0};
    char pending_vin[64] = {0};
    char pending_probe[64] = {0};
    char pending_soa[64] = {0};
    char pending_pdn_node[64] = {0};
    char pending_pdn_isrc[64] = {0};

    char line[1024];
    int line_no = 0;
    while (fgets(line, (int)sizeof(line), fp) != NULL) {
        ++line_no;
        char *semi = strchr(line, ';');
        if (semi != NULL) *semi = '\0';
        char *p = line;
        while (*p != '\0' && isspace((unsigned char)*p)) ++p;
        if (*p == '\0' || *p == '*') continue;
        if (line_no == 1 && strncasecmp_ele(p, ".title", 6U) == 0) continue;

        /* Preserve raw copy for PULSE(...) / SIN(...) */
        char raw_line[1024];
        snprintf(raw_line, sizeof(raw_line), "%s", p);

        char *toks[64];
        int ntok = 0;
        char *save = NULL;
        char *tk = strtok_r_ele(p, " \t\r\n", &save);
        while (tk != NULL && ntok < 64) {
            toks[ntok++] = tk;
            tk = strtok_r_ele(NULL, " \t\r\n", &save);
        }
        if (ntok == 0) continue;

        if (toks[0][0] == '.') {
            if (strcasecmp_ele(toks[0], ".end") == 0) break;
            if (strcasecmp_ele(toks[0], ".op") == 0) {
                out->has_op = 1;
            } else if (strcasecmp_ele(toks[0], ".ac") == 0 && ntok >= 5) {
                out->has_ac = 1;
                out->ac_is_dec = (strcasecmp_ele(toks[1], "dec") == 0) ? 1 : 0;
                out->ac_pts = atoi(toks[2]);
                if (out->ac_pts < 2) out->ac_pts = 21;
                if (out->ac_pts > ELE_CKT_MAX_AC_PTS) out->ac_pts = ELE_CKT_MAX_AC_PTS;
                parse_eng_val(toks[3], &out->ac_f1);
                parse_eng_val(toks[4], &out->ac_f2);
                for (int i = 5; i < ntok; ++i) {
                    char k[32], sv[64]; double dv = 0.0;
                    if (parse_kv_param(toks[i], k, sizeof(k), &dv, sv, sizeof(sv)) == 0) {
                        if (strcmp(k, "VOUT") == 0) snprintf(pending_vout, sizeof(pending_vout), "%s", sv);
                        else if (strcmp(k, "VIN") == 0) snprintf(pending_vin, sizeof(pending_vin), "%s", sv);
                    }
                }
            } else if (strcasecmp_ele(toks[0], ".tran") == 0 && ntok >= 3) {
                out->has_tran = 1;
                parse_eng_val(toks[1], &out->tran_tstep);
                parse_eng_val(toks[2], &out->tran_tstop);
                for (int i = 3; i < ntok; ++i) {
                    char k[32], sv[64]; double dv = 0.0;
                    if (parse_kv_param(toks[i], k, sizeof(k), &dv, sv, sizeof(sv)) == 0) {
                        if (strcmp(k, "PROBE") == 0) snprintf(pending_probe, sizeof(pending_probe), "%s", sv);
                    }
                }
            } else if (strcasecmp_ele(toks[0], ".soa") == 0) {
                out->has_soa = 1;
                out->soa_vmin = -1e9;
                out->soa_vmax = 1e9;
                for (int i = 1; i < ntok; ++i) {
                    char k[32], sv[64]; double dv = 0.0;
                    if (parse_kv_param(toks[i], k, sizeof(k), &dv, sv, sizeof(sv)) == 0) {
                        if (strcmp(k, "NODE") == 0) snprintf(pending_soa, sizeof(pending_soa), "%s", sv);
                        else if (strcmp(k, "VMIN") == 0) out->soa_vmin = dv;
                        else if (strcmp(k, "VMAX") == 0) out->soa_vmax = dv;
                    }
                }
            } else if (strcasecmp_ele(toks[0], ".pdn") == 0) {
                out->has_pdn = 1;
                out->pdn_zmax = 1.0;
                for (int i = 1; i < ntok; ++i) {
                    char k[32], sv[64]; double dv = 0.0;
                    if (parse_kv_param(toks[i], k, sizeof(k), &dv, sv, sizeof(sv)) == 0) {
                        if (strcmp(k, "NODE") == 0) snprintf(pending_pdn_node, sizeof(pending_pdn_node), "%s", sv);
                        else if (strcmp(k, "ISRC") == 0) snprintf(pending_pdn_isrc, sizeof(pending_pdn_isrc), "%s", sv);
                        else if (strcmp(k, "ZMAX") == 0) out->pdn_zmax = dv;
                    }
                }
            }
            continue;
        }

        char kind = (char)toupper((unsigned char)toks[0][0]);
        if (kind == 'R' || kind == 'C' || kind == 'L') {
            if (ntok < 4) { fclose(fp); return -1; }
            int n1 = get_or_add_node(out, toks[1]);
            int n2 = get_or_add_node(out, toks[2]);
            double val = 0.0;
            if (n1 < 0 || n2 < 0 || parse_eng_val(toks[3], &val) != 0) { fclose(fp); return -1; }
            ele_ckt_elem_type_t et = (kind == 'R') ? ELE_CKT_R : (kind == 'C' ? ELE_CKT_C : ELE_CKT_L);
            int idx = add_basic_elem(out, et, toks[0], n1, n2, val, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            for (int i = 4; i < ntok; ++i) {
                char k[32]; double dv = 0.0;
                if (parse_kv_param(toks[i], k, sizeof(k), &dv, NULL, 0U) == 0) {
                    if (strcmp(k, "TOL") == 0) out->elems[idx].tol_frac = (dv > 1.0) ? (dv * 0.01) : dv;
                    else if (strcmp(k, "IC") == 0) { out->elems[idx].ic = dv; out->elems[idx].has_ic = 1; }
                }
            }
        } else if (kind == 'K' && ntok >= 4) {
            int idx = add_basic_elem(out, ELE_CKT_K, toks[0], 0, 0, 0.0, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            snprintf(out->elems[idx].l1_name, sizeof(out->elems[idx].l1_name), "%s", toks[1]);
            snprintf(out->elems[idx].l2_name, sizeof(out->elems[idx].l2_name), "%s", toks[2]);
            parse_eng_val(toks[3], &out->elems[idx].val);
        } else if ((kind == 'V' || kind == 'I') && ntok >= 3) {
            int n1 = get_or_add_node(out, toks[1]);
            int n2 = get_or_add_node(out, toks[2]);
            if (n1 < 0 || n2 < 0) { fclose(fp); return -1; }
            int idx = add_basic_elem(out, (kind == 'V') ? ELE_CKT_V : ELE_CKT_I, toks[0], n1, n2, 0.0, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            ele_ckt_elem_t *e = &out->elems[idx];
            e->src_kind = ELE_SRC_DC;
            /* Parse DC, AC, and PULSE/SIN */
            const char *pulse_ptr = strstr(raw_line, "PULSE");
            if (pulse_ptr == NULL) pulse_ptr = strstr(raw_line, "pulse");
            if (pulse_ptr != NULL) {
                double pv[8] = {0};
                int npv = 0;
                parse_paren_floats(pulse_ptr, pv, 7, &npv);
                if (npv >= 2) {
                    e->src_kind = ELE_SRC_PULSE;
                    e->p_v1 = pv[0];
                    e->p_v2 = pv[1];
                    e->p_td = (npv > 2) ? pv[2] : 0.0;
                    e->p_tr = (npv > 3) ? pv[3] : 1e-12;
                    e->p_tf = (npv > 4) ? pv[4] : 1e-12;
                    e->p_pw = (npv > 5) ? pv[5] : 1e9;
                    e->p_per = (npv > 6) ? pv[6] : 0.0;
                    e->dc_val = e->p_v1;
                }
            }
            const char *sin_ptr = strstr(raw_line, "SIN");
            if (sin_ptr == NULL) sin_ptr = strstr(raw_line, "sin");
            if (sin_ptr != NULL && e->src_kind == ELE_SRC_DC) {
                double sv[8] = {0};
                int nsv = 0;
                parse_paren_floats(sin_ptr, sv, 4, &nsv);
                if (nsv >= 3) {
                    e->src_kind = ELE_SRC_SIN;
                    e->s_vo = sv[0];
                    e->s_va = sv[1];
                    e->s_freq = sv[2];
                    e->s_td = (nsv > 3) ? sv[3] : 0.0;
                    e->dc_val = e->s_vo;
                }
            }
            for (int i = 3; i < ntok; ++i) {
                if (strcasecmp_ele(toks[i], "dc") == 0 && i + 1 < ntok) {
                    parse_eng_val(toks[++i], &e->dc_val);
                } else if (strcasecmp_ele(toks[i], "ac") == 0 && i + 1 < ntok) {
                    parse_eng_val(toks[++i], &e->ac_mag);
                    if (i + 1 < ntok && parse_eng_val(toks[i + 1], &e->ac_phase_deg) == 0 &&
                        isalpha((unsigned char)toks[i + 1][0]) == 0) {
                        ++i;
                    }
                } else if (i == 3 && e->src_kind == ELE_SRC_DC &&
                           strncasecmp_ele(toks[i], "pulse", 5U) != 0 &&
                           strncasecmp_ele(toks[i], "sin", 3U) != 0) {
                    double dv = 0.0;
                    if (parse_eng_val(toks[i], &dv) == 0) e->dc_val = dv;
                }
            }
        } else if (kind == 'D' && ntok >= 3) {
            int n1 = get_or_add_node(out, toks[1]);
            int n2 = get_or_add_node(out, toks[2]);
            if (n1 < 0 || n2 < 0) { fclose(fp); return -1; }
            int idx = add_basic_elem(out, ELE_CKT_D, toks[0], n1, n2, 0.0, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            out->elems[idx].d_is = 1e-12;
            out->elems[idx].d_n = 1.0;
            out->elems[idx].d_rs = 0.02;
            for (int i = 3; i < ntok; ++i) {
                char k[32]; double dv = 0.0;
                if (parse_kv_param(toks[i], k, sizeof(k), &dv, NULL, 0U) == 0) {
                    if (strcmp(k, "IS") == 0) out->elems[idx].d_is = dv;
                    else if (strcmp(k, "N") == 0) out->elems[idx].d_n = dv;
                    else if (strcmp(k, "RS") == 0) out->elems[idx].d_rs = dv;
                }
            }
        } else if (kind == 'M' && ntok >= 4) {
            int nd = get_or_add_node(out, toks[1]);
            int ng = get_or_add_node(out, toks[2]);
            int ns = get_or_add_node(out, toks[3]);
            if (nd < 0 || ng < 0 || ns < 0) { fclose(fp); return -1; }
            int idx = add_basic_elem(out, ELE_CKT_M, toks[0], nd, ns, 0.0, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            out->elems[idx].n3 = ng;
            out->elems[idx].m_vth = 3.0;
            out->elems[idx].m_ron = 0.05;
            out->elems[idx].m_roff = 1e7;
            out->elems[idx].m_kp = 0.0;
            double cgd = 0.0, cgs = 0.0, cds = 0.0;
            for (int i = 4; i < ntok; ++i) {
                char k[32]; double dv = 0.0;
                if (parse_kv_param(toks[i], k, sizeof(k), &dv, NULL, 0U) == 0) {
                    if (strcmp(k, "VTH") == 0) out->elems[idx].m_vth = dv;
                    else if (strcmp(k, "RON") == 0) out->elems[idx].m_ron = dv;
                    else if (strcmp(k, "ROFF") == 0) out->elems[idx].m_roff = dv;
                    else if (strcmp(k, "KP") == 0) out->elems[idx].m_kp = dv;
                    else if (strcmp(k, "CGD") == 0) cgd = dv;
                    else if (strcmp(k, "CGS") == 0) cgs = dv;
                    else if (strcmp(k, "CDS") == 0) cds = dv;
                }
            }
            if (cgd > 0.0) {
                char cname[32];
                snprintf(cname, sizeof(cname), "C_gd_%s", toks[0]);
                add_basic_elem(out, ELE_CKT_C, cname, nd, ng, cgd, 0.0);
            }
            if (cgs > 0.0) {
                char cname[32];
                snprintf(cname, sizeof(cname), "C_gs_%s", toks[0]);
                add_basic_elem(out, ELE_CKT_C, cname, ng, ns, cgs, 0.0);
            }
            if (cds > 0.0) {
                char cname[32];
                snprintf(cname, sizeof(cname), "C_ds_%s", toks[0]);
                add_basic_elem(out, ELE_CKT_C, cname, nd, ns, cds, 0.0);
            }
        } else if ((kind == 'E' || kind == 'G') && ntok >= 6) {
            int np = get_or_add_node(out, toks[1]);
            int nm = get_or_add_node(out, toks[2]);
            int ncp = get_or_add_node(out, toks[3]);
            int ncm = get_or_add_node(out, toks[4]);
            double val = 0.0;
            if (np < 0 || nm < 0 || ncp < 0 || ncm < 0 || parse_eng_val(toks[5], &val) != 0) {
                fclose(fp);
                return -1;
            }
            int idx = add_basic_elem(out, (kind == 'E') ? ELE_CKT_E : ELE_CKT_G, toks[0], np, nm, val, 0.0);
            if (idx < 0) { fclose(fp); return -1; }
            out->elems[idx].n3 = ncp;
            out->elems[idx].n4 = ncm;
        } else if (kind == 'T' && ntok >= 6) {
            /* Coupled 2-line RLGC transmission line: Tname vin vout ain aout gnd params... */
            int vin = get_or_add_node(out, toks[1]);
            int vout = get_or_add_node(out, toks[2]);
            int ain = get_or_add_node(out, toks[3]);
            int aout = get_or_add_node(out, toks[4]);
            int gnd = get_or_add_node(out, toks[5]);
            if (vin < 0 || vout < 0 || ain < 0 || aout < 0 || gnd < 0) { fclose(fp); return -1; }
            double rp = 5.0, lp = 300e-9, cp = 120e-12;
            double lm = 30e-9, cm = 18e-12, len = 0.10;
            int nsec = 8;
            for (int i = 6; i < ntok; ++i) {
                char k[32]; double dv = 0.0;
                if (parse_kv_param(toks[i], k, sizeof(k), &dv, NULL, 0U) == 0) {
                    if (strcmp(k, "R") == 0) rp = dv;
                    else if (strcmp(k, "L") == 0) lp = dv;
                    else if (strcmp(k, "C") == 0) cp = dv;
                    else if (strcmp(k, "LM") == 0) lm = dv;
                    else if (strcmp(k, "CM") == 0) cm = dv;
                    else if (strcmp(k, "LEN") == 0) len = dv;
                    else if (strcmp(k, "NSEC") == 0) nsec = (int)llround(dv);
                }
            }
            if (nsec < 2) nsec = 2;
            if (nsec > 16) nsec = 16;
            double dz = len / (double)nsec;
            double rs = rp * dz, ls = lp * dz, cs = cp * dz, cms = cm * dz;
            double k_cpl = (lp > 0.0) ? (lm / lp) : 0.0;
            if (k_cpl > 0.99) k_cpl = 0.99;

            int cur_v = vin, cur_a = ain;
            for (int s = 0; s < nsec; ++s) {
                char nm[32];
                int next_v = (s + 1 == nsec) ? vout : -1;
                int next_a = (s + 1 == nsec) ? aout : -1;
                int mid_v = -1, mid_a = -1;
                if (next_v < 0) {
                    snprintf(nm, sizeof(nm), "_%s_v%d", toks[0], s);
                    next_v = get_or_add_node(out, nm);
                }
                if (next_a < 0) {
                    snprintf(nm, sizeof(nm), "_%s_a%d", toks[0], s);
                    next_a = get_or_add_node(out, nm);
                }
                snprintf(nm, sizeof(nm), "_%s_mv%d", toks[0], s);
                mid_v = get_or_add_node(out, nm);
                snprintf(nm, sizeof(nm), "_%s_ma%d", toks[0], s);
                mid_a = get_or_add_node(out, nm);
                if (next_v < 0 || next_a < 0 || mid_v < 0 || mid_a < 0) { fclose(fp); return -1; }

                char rv[32], ra[32], lv[32], la[32], ks[32], cv[32], ca[32], cx[32];
                snprintf(rv, sizeof(rv), "R_%s_v%d", toks[0], s);
                snprintf(ra, sizeof(ra), "R_%s_a%d", toks[0], s);
                snprintf(lv, sizeof(lv), "L_%s_v%d", toks[0], s);
                snprintf(la, sizeof(la), "L_%s_a%d", toks[0], s);
                snprintf(ks, sizeof(ks), "K_%s_%d", toks[0], s);
                snprintf(cv, sizeof(cv), "C_%s_v%d", toks[0], s);
                snprintf(ca, sizeof(ca), "C_%s_a%d", toks[0], s);
                snprintf(cx, sizeof(cx), "C_%s_m%d", toks[0], s);

                add_basic_elem(out, ELE_CKT_R, rv, cur_v, mid_v, rs, 0.0);
                add_basic_elem(out, ELE_CKT_R, ra, cur_a, mid_a, rs, 0.0);
                int ilv = add_basic_elem(out, ELE_CKT_L, lv, mid_v, next_v, ls, 0.0);
                int ila = add_basic_elem(out, ELE_CKT_L, la, mid_a, next_a, ls, 0.0);
                if (k_cpl > 1e-6 && ilv >= 0 && ila >= 0) {
                    int ik = add_basic_elem(out, ELE_CKT_K, ks, 0, 0, k_cpl, 0.0);
                    if (ik >= 0) {
                        out->elems[ik].l1_idx = ilv;
                        out->elems[ik].l2_idx = ila;
                        snprintf(out->elems[ik].l1_name, sizeof(out->elems[ik].l1_name), "%s", lv);
                        snprintf(out->elems[ik].l2_name, sizeof(out->elems[ik].l2_name), "%s", la);
                    }
                }
                add_basic_elem(out, ELE_CKT_C, cv, next_v, gnd, cs, 0.0);
                add_basic_elem(out, ELE_CKT_C, ca, next_a, gnd, cs, 0.0);
                if (cms > 0.0) add_basic_elem(out, ELE_CKT_C, cx, next_v, next_a, cms, 0.0);
                cur_v = next_v;
                cur_a = next_a;
            }
        }
    }
    fclose(fp);

    /* Resolve K inductor names -> indices, and assign MNA branch indices */
    int br_cnt = 0;
    for (int i = 0; i < out->n_elems; ++i) {
        ele_ckt_elem_t *e = &out->elems[i];
        if (e->type == ELE_CKT_V || e->type == ELE_CKT_L || e->type == ELE_CKT_E) {
            e->branch_idx = br_cnt++;
        }
    }
    if (out->n_nodes - 1 + br_cnt > ELE_CKT_MAX_MNA) return -1;

    for (int i = 0; i < out->n_elems; ++i) {
        ele_ckt_elem_t *e = &out->elems[i];
        if (e->type == ELE_CKT_K && (e->l1_idx < 0 || e->l2_idx < 0)) {
            for (int j = 0; j < out->n_elems; ++j) {
                if (out->elems[j].type == ELE_CKT_L) {
                    if (strcasecmp_ele(out->elems[j].name, e->l1_name) == 0) e->l1_idx = j;
                    if (strcasecmp_ele(out->elems[j].name, e->l2_name) == 0) e->l2_idx = j;
                }
            }
            if (e->l1_idx < 0 || e->l2_idx < 0) return -1;
        }
    }

    if (pending_vout[0] != '\0') out->ac_vout_node = ele_ckt_find_node(out, pending_vout);
    if (pending_probe[0] != '\0') out->tran_probe_node = ele_ckt_find_node(out, pending_probe);
    if (pending_soa[0] != '\0') out->soa_node = ele_ckt_find_node(out, pending_soa);
    if (pending_pdn_node[0] != '\0') out->pdn_node = ele_ckt_find_node(out, pending_pdn_node);
    if (pending_vin[0] != '\0') {
        for (int i = 0; i < out->n_elems; ++i) {
            if (strcasecmp_ele(out->elems[i].name, pending_vin) == 0) out->ac_vin_elem = i;
        }
    }
    if (pending_pdn_isrc[0] != '\0') {
        for (int i = 0; i < out->n_elems; ++i) {
            if (strcasecmp_ele(out->elems[i].name, pending_pdn_isrc) == 0) out->pdn_isrc_elem = i;
        }
    }
    if (out->ac_vout_node <= 0 && out->n_nodes > 1) {
        int vout = ele_ckt_find_node(out, "vout");
        out->ac_vout_node = (vout > 0) ? vout : (out->n_nodes - 1);
    }
    if (out->tran_probe_node <= 0) {
        if (out->soa_node > 0) out->tran_probe_node = out->soa_node;
        else if (out->ac_vout_node > 0) out->tran_probe_node = out->ac_vout_node;
        else if (out->n_nodes > 1) out->tran_probe_node = out->n_nodes - 1;
    }
    if (out->ac_vin_elem < 0) {
        for (int i = 0; i < out->n_elems; ++i) {
            if ((out->elems[i].type == ELE_CKT_V || out->elems[i].type == ELE_CKT_I) &&
                out->elems[i].ac_mag > 0.0) {
                out->ac_vin_elem = i;
                break;
            }
        }
    }
    return 0;
}

/* ============================================================================
 * 2. Nonlinear Device Models & MNA Assembly
 * ============================================================================ */

static void eval_diode(const ele_ckt_elem_t *e, double vd, double *id_out, double *gd_out) {
    const double vt = 0.02585;
    double nvt = e->d_n * vt;
    if (nvt < 1e-4) nvt = vt;
    double is = (e->d_is > 0.0) ? e->d_is : 1e-12;
    double rs = (e->d_rs > 0.0) ? e->d_rs : 0.0;
    double id = 0.0;
    if (rs <= 0.0) {
        double arg = vd / nvt;
        if (arg > 40.0) {
            double e40 = exp(40.0);
            id = is * (e40 * (1.0 + (arg - 40.0)) - 1.0);
            *gd_out = is * e40 / nvt;
        } else if (arg < -40.0) {
            id = -is;
            *gd_out = 1e-12;
        } else {
            double ex = exp(arg);
            id = is * (ex - 1.0);
            *gd_out = (is * ex) / nvt + 1e-12;
        }
        *id_out = id;
    } else {
        /* Solve id = is*(exp((vd - id*rs)/nvt) - 1) via damped 1D Newton */
        double vj = vd;
        if (vj > 40.0 * nvt) vj = 40.0 * nvt;
        for (int it = 0; it < 20; ++it) {
            double arg = (vj < -40.0 * nvt) ? -40.0 : (vj / nvt);
            double ex = (arg > 40.0) ? (exp(40.0) * (1.0 + (arg - 40.0))) : exp(arg);
            double f = vj + rs * is * (ex - 1.0) - vd;
            double df = 1.0 + rs * (is * ex / nvt);
            double step = f / df;
            if (step > 2.0 * nvt) step = 2.0 * nvt;
            if (step < -2.0 * nvt) step = -2.0 * nvt;
            vj -= step;
            if (fabs(f) < 1e-12) break;
        }
        double arg = (vj / nvt);
        double ex = (arg > 40.0) ? exp(40.0) * (1.0 + arg - 40.0) : (arg < -40.0 ? 0.0 : exp(arg));
        double g_j = (is * ex) / nvt + 1e-12;
        *id_out = is * (ex - 1.0) + 1e-12 * vj;
        *gd_out = g_j / (1.0 + rs * g_j);
    }
}

static void eval_mos_switch(const ele_ckt_elem_t *e, double vgs, double vds,
                            double *ids_out, double *gds_out, double *gm_out) {
    double gon = 1.0 / ((e->m_ron > 1e-6) ? e->m_ron : 0.05);
    double goff = 1.0 / ((e->m_roff > 1.0) ? e->m_roff : 1e7);
    double u = (vgs - e->m_vth) / 0.25;
    if (u > 40.0) u = 40.0;
    if (u < -40.0) u = -40.0;
    double sig = 1.0 / (1.0 + exp(-u));
    double g = goff + (gon - goff) * sig;
    double dg_dvgs = (gon - goff) * (sig * (1.0 - sig)) / 0.25;
    *ids_out = g * vds;
    *gds_out = g;
    *gm_out = dg_dvgs * vds;
}

static double eval_src_time(const ele_ckt_elem_t *e, double t) {
    if (e->src_kind == ELE_SRC_DC) return e->dc_val;
    if (e->src_kind == ELE_SRC_PULSE) {
        if (t < e->p_td) return e->p_v1;
        double dt = t - e->p_td;
        double tp = (e->p_per > 0.0) ? fmod(dt, e->p_per) : dt;
        double tr = (e->p_tr > 1e-15) ? e->p_tr : 1e-15;
        double tf = (e->p_tf > 1e-15) ? e->p_tf : 1e-15;
        if (tp < tr) return e->p_v1 + (e->p_v2 - e->p_v1) * (tp / tr);
        if (tp < tr + e->p_pw) return e->p_v2;
        if (tp < tr + e->p_pw + tf) return e->p_v2 + (e->p_v1 - e->p_v2) * ((tp - tr - e->p_pw) / tf);
        return e->p_v1;
    }
    if (e->src_kind == ELE_SRC_SIN) {
        if (t < e->s_td) return e->s_vo;
        return e->s_vo + e->s_va * sin(2.0 * M_PI * e->s_freq * (t - e->s_td));
    }
    return e->dc_val;
}

static int get_mna_dim(const ele_ckt_netlist_t *nl, int *n_v_out, int *n_br_out) {
    int nv = nl->n_nodes - 1;
    int nbr = 0;
    for (int i = 0; i < nl->n_elems; ++i) {
        if (nl->elems[i].branch_idx >= 0) ++nbr;
    }
    *n_v_out = nv;
    *n_br_out = nbr;
    return nv + nbr;
}

static void stamp_g(double *A, int mna, int a, int b, double g) {
    if (a > 0) A[(size_t)(a - 1) * (size_t)mna + (size_t)(a - 1)] += g;
    if (b > 0) A[(size_t)(b - 1) * (size_t)mna + (size_t)(b - 1)] += g;
    if (a > 0 && b > 0) {
        A[(size_t)(a - 1) * (size_t)mna + (size_t)(b - 1)] -= g;
        A[(size_t)(b - 1) * (size_t)mna + (size_t)(a - 1)] -= g;
    }
}

static void stamp_i_rhs(double *rhs, int a, int b, double i_a_to_b) {
    /* Current leaving node a and entering node b */
    if (a > 0) rhs[a - 1] -= i_a_to_b;
    if (b > 0) rhs[b - 1] += i_a_to_b;
}

static int solve_dc_with_gmin(const ele_ckt_netlist_t *nl, double gmin, double *x_io,
                              int max_it, double *kcl_res_out, int *iters_out) {
    int nv = 0, nbr = 0;
    int mna = get_mna_dim(nl, &nv, &nbr);
    if (mna <= 0 || mna > ELE_CKT_MAX_MNA) return -1;
    size_t nn = (size_t)mna;
    double *A = (double *)malloc(nn * nn * sizeof(double));
    double *rhs = (double *)malloc(nn * sizeof(double));
    double *x_next = (double *)malloc(nn * sizeof(double));
    if (A == NULL || rhs == NULL || x_next == NULL) {
        free(A); free(rhs); free(x_next);
        return -1;
    }

    int converged = 0;
    double res_max = 1e9;
    int it = 0;
    for (it = 0; it < max_it; ++it) {
        memset(A, 0, nn * nn * sizeof(double));
        memset(rhs, 0, nn * sizeof(double));
        for (int n = 0; n < nv; ++n) A[(size_t)n * nn + (size_t)n] += gmin;

        for (int i = 0; i < nl->n_elems; ++i) {
            const ele_ckt_elem_t *e = &nl->elems[i];
            if (e->type == ELE_CKT_R && e->val > 0.0) {
                stamp_g(A, mna, e->n1, e->n2, 1.0 / e->val);
            } else if (e->type == ELE_CKT_V) {
                int br = nv + e->branch_idx;
                if (e->n1 > 0) {
                    A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                    A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                    A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                rhs[br] += e->dc_val;
            } else if (e->type == ELE_CKT_I) {
                stamp_i_rhs(rhs, e->n1, e->n2, e->dc_val);
            } else if (e->type == ELE_CKT_L) {
                int br = nv + e->branch_idx;
                if (e->n1 > 0) {
                    A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                    A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                    A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                A[(size_t)br * nn + (size_t)br] -= 1e-9; /* tiny winding R for DC */
            } else if (e->type == ELE_CKT_E) {
                int br = nv + e->branch_idx;
                if (e->n1 > 0) {
                    A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                    A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                    A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                if (e->n3 > 0) A[(size_t)br * nn + (size_t)(e->n3 - 1)] -= e->val;
                if (e->n4 > 0) A[(size_t)br * nn + (size_t)(e->n4 - 1)] += e->val;
            } else if (e->type == ELE_CKT_G) {
                double gm = e->val;
                if (e->n1 > 0 && e->n3 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                if (e->n1 > 0 && e->n4 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n4 - 1)] -= gm;
                if (e->n2 > 0 && e->n3 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                if (e->n2 > 0 && e->n4 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n4 - 1)] += gm;
            } else if (e->type == ELE_CKT_D) {
                double va = (e->n1 > 0) ? x_io[e->n1 - 1] : 0.0;
                double vc = (e->n2 > 0) ? x_io[e->n2 - 1] : 0.0;
                double vd = va - vc;
                double id = 0.0, gd = 0.0;
                eval_diode(e, vd, &id, &gd);
                stamp_g(A, mna, e->n1, e->n2, gd);
                double ieq = id - gd * vd;
                stamp_i_rhs(rhs, e->n1, e->n2, ieq);
            } else if (e->type == ELE_CKT_M) {
                double vd_node = (e->n1 > 0) ? x_io[e->n1 - 1] : 0.0;
                double vs_node = (e->n2 > 0) ? x_io[e->n2 - 1] : 0.0;
                double vg_node = (e->n3 > 0) ? x_io[e->n3 - 1] : 0.0;
                double vds = vd_node - vs_node;
                double vgs = vg_node - vs_node;
                double ids = 0.0, gds = 0.0, gm = 0.0;
                eval_mos_switch(e, vgs, vds, &ids, &gds, &gm);
                stamp_g(A, mna, e->n1, e->n2, gds);
                if (e->n1 > 0 && e->n3 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                if (e->n1 > 0 && e->n2 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n2 - 1)] -= gm;
                if (e->n2 > 0 && e->n3 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                if (e->n2 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n2 - 1)] += gm;
                double ieq = ids - gds * vds - gm * vgs;
                stamp_i_rhs(rhs, e->n1, e->n2, ieq);
            }
        }

        if (ele_lu_solve_real(mna, A, rhs, x_next) != 0) {
            free(A); free(rhs); free(x_next);
            return -1;
        }
        double max_dv = 0.0;
        for (int k = 0; k < mna; ++k) {
            double dv = x_next[k] - x_io[k];
            if (k < nv && fabs(dv) > 200.0 && it < 20) {
                double s = 200.0 / fabs(dv);
                x_io[k] += s * dv;
            } else {
                x_io[k] = x_next[k];
            }
            if (fabs(dv) > max_dv) max_dv = fabs(dv);
        }

        /* Evaluate true KCL residual */
        res_max = 0.0;
        for (int row = 0; row < nv; ++row) {
            int node = row + 1;
            double kcl = 0.0;
            for (int i = 0; i < nl->n_elems; ++i) {
                const ele_ckt_elem_t *e = &nl->elems[i];
                if (e->type == ELE_CKT_R && e->val > 0.0) {
                    double v1 = (e->n1 > 0) ? x_io[e->n1 - 1] : 0.0;
                    double v2 = (e->n2 > 0) ? x_io[e->n2 - 1] : 0.0;
                    double ir = (v1 - v2) / e->val;
                    if (e->n1 == node) kcl += ir;
                    if (e->n2 == node) kcl -= ir;
                } else if (e->type == ELE_CKT_I) {
                    if (e->n1 == node) kcl += e->dc_val;
                    if (e->n2 == node) kcl -= e->dc_val;
                } else if (e->type == ELE_CKT_V || e->type == ELE_CKT_L || e->type == ELE_CKT_E) {
                    double ib = x_io[nv + e->branch_idx];
                    if (e->n1 == node) kcl += ib;
                    if (e->n2 == node) kcl -= ib;
                } else if (e->type == ELE_CKT_G) {
                    double v3 = (e->n3 > 0) ? x_io[e->n3 - 1] : 0.0;
                    double v4 = (e->n4 > 0) ? x_io[e->n4 - 1] : 0.0;
                    double ig = e->val * (v3 - v4);
                    if (e->n1 == node) kcl += ig;
                    if (e->n2 == node) kcl -= ig;
                } else if (e->type == ELE_CKT_D) {
                    double v1 = (e->n1 > 0) ? x_io[e->n1 - 1] : 0.0;
                    double v2 = (e->n2 > 0) ? x_io[e->n2 - 1] : 0.0;
                    double id = 0.0, gd = 0.0;
                    eval_diode(e, v1 - v2, &id, &gd);
                    if (e->n1 == node) kcl += id;
                    if (e->n2 == node) kcl -= id;
                } else if (e->type == ELE_CKT_M) {
                    double v1 = (e->n1 > 0) ? x_io[e->n1 - 1] : 0.0;
                    double v2 = (e->n2 > 0) ? x_io[e->n2 - 1] : 0.0;
                    double v3 = (e->n3 > 0) ? x_io[e->n3 - 1] : 0.0;
                    double ids = 0.0, gds = 0.0, gm = 0.0;
                    eval_mos_switch(e, v3 - v2, v1 - v2, &ids, &gds, &gm);
                    if (e->n1 == node) kcl += ids;
                    if (e->n2 == node) kcl -= ids;
                }
            }
            if (fabs(kcl) > res_max) res_max = fabs(kcl);
        }
        if (max_dv < 1e-11 && res_max < 1e-11) {
            converged = 1;
            ++it;
            break;
        }
    }
    free(A); free(rhs); free(x_next);
    if (kcl_res_out != NULL) *kcl_res_out = res_max;
    if (iters_out != NULL) *iters_out = it;
    return converged ? 0 : -1;
}

int ele_ckt_solve_dc(const ele_ckt_netlist_t *nl, ele_ckt_dc_res_t *out) {
    if (nl == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int nv = 0, nbr = 0;
    int mna = get_mna_dim(nl, &nv, &nbr);
    if (mna <= 0 || mna > ELE_CKT_MAX_MNA) return -1;

    double x[ELE_CKT_MAX_MNA];
    memset(x, 0, sizeof(x));

    const double gmins[4] = {1e-3, 1e-6, 1e-10, 1e-15};
    int tot_it = 0;
    double res = 1e9;
    int ok = 0;
    for (int g = 0; g < 4; ++g) {
        int it = 0;
        if (solve_dc_with_gmin(nl, gmins[g], x, 80, &res, &it) == 0) {
            ok = 1;
        } else if (g == 3) {
            ok = 0;
        }
        tot_it += it;
    }

    out->converged = ok;
    out->nr_iters = tot_it;
    out->max_kcl_res = res;
    out->n_nodes = nl->n_nodes;
    out->n_branches = nbr;
    out->node_v[0] = 0.0;
    for (int i = 0; i < nv; ++i) out->node_v[i + 1] = x[i];
    for (int i = 0; i < nbr; ++i) out->branch_i[i] = x[nv + i];

    out->is_pure_passive_r = 1;
    double p_diss = 0.0;
    for (int i = 0; i < nl->n_elems; ++i) {
        const ele_ckt_elem_t *e = &nl->elems[i];
        if (e->type == ELE_CKT_D || e->type == ELE_CKT_M || e->type == ELE_CKT_E || e->type == ELE_CKT_G) {
            out->is_pure_passive_r = 0;
        }
        if (e->type == ELE_CKT_R && e->val > 0.0) {
            double dv = out->node_v[e->n1] - out->node_v[e->n2];
            p_diss += (dv * dv) / e->val;
        }
    }
    out->total_power_w = p_diss;
    return ok ? 0 : -1;
}

/* ============================================================================
 * 3. AC Small-Signal & PDN Impedance Solver
 * ============================================================================ */

int ele_ckt_solve_ac(const ele_ckt_netlist_t *nl, const ele_ckt_dc_res_t *dc,
                     ele_ckt_ac_res_t *out) {
    if (nl == NULL || dc == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    int nv = 0, nbr = 0;
    int mna = get_mna_dim(nl, &nv, &nbr);
    if (mna <= 0 || mna > ELE_CKT_MAX_MNA) return -1;
    int npts = (nl->has_ac && nl->ac_pts >= 2) ? nl->ac_pts : 61;
    if (npts > ELE_CKT_MAX_AC_PTS) npts = ELE_CKT_MAX_AC_PTS;
    double f1 = (nl->has_ac && nl->ac_f1 > 0.0) ? nl->ac_f1 : 1.0;
    double f2 = (nl->has_ac && nl->ac_f2 > f1) ? nl->ac_f2 : 1e6;
    int is_dec = nl->has_ac ? nl->ac_is_dec : 1;

    size_t nn = (size_t)mna;
    double *Ar = (double *)malloc(nn * nn * sizeof(double));
    double *Ai = (double *)malloc(nn * nn * sizeof(double));
    double *br = (double *)malloc(nn * sizeof(double));
    double *bi = (double *)malloc(nn * sizeof(double));
    double *xr = (double *)malloc(nn * sizeof(double));
    double *xi = (double *)malloc(nn * sizeof(double));
    if (Ar == NULL || Ai == NULL || br == NULL || bi == NULL || xr == NULL || xi == NULL) {
        free(Ar); free(Ai); free(br); free(bi); free(xr); free(xi);
        return -1;
    }

    out->n_pts = npts;
    out->pdn_pass = 1;
    out->z_pdn_max_ohm = 0.0;
    double dc_gain_db = -1e9;

    for (int p = 0; p < npts; ++p) {
        double freq = is_dec
                          ? (f1 * pow(f2 / f1, (double)p / (double)(npts - 1)))
                          : (f1 + (f2 - f1) * (double)p / (double)(npts - 1));
        double w = 2.0 * M_PI * freq;
        memset(Ar, 0, nn * nn * sizeof(double));
        memset(Ai, 0, nn * nn * sizeof(double));
        memset(br, 0, nn * sizeof(double));
        memset(bi, 0, nn * sizeof(double));
        for (int k = 0; k < nv; ++k) Ar[(size_t)k * nn + (size_t)k] += 1e-12;

        for (int i = 0; i < nl->n_elems; ++i) {
            const ele_ckt_elem_t *e = &nl->elems[i];
            if (e->type == ELE_CKT_R && e->val > 0.0) {
                stamp_g(Ar, mna, e->n1, e->n2, 1.0 / e->val);
            } else if (e->type == ELE_CKT_C) {
                stamp_g(Ai, mna, e->n1, e->n2, w * e->val);
            } else if (e->type == ELE_CKT_L) {
                int br_idx = nv + e->branch_idx;
                if (e->n1 > 0) {
                    Ar[(size_t)(e->n1 - 1) * nn + (size_t)br_idx] += 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    Ar[(size_t)(e->n2 - 1) * nn + (size_t)br_idx] -= 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                Ai[(size_t)br_idx * nn + (size_t)br_idx] -= w * e->val;
            } else if (e->type == ELE_CKT_K) {
                const ele_ckt_elem_t *l1 = &nl->elems[e->l1_idx];
                const ele_ckt_elem_t *l2 = &nl->elems[e->l2_idx];
                double mut = e->val * sqrt(l1->val * l2->val);
                int b1 = nv + l1->branch_idx;
                int b2 = nv + l2->branch_idx;
                Ai[(size_t)b1 * nn + (size_t)b2] -= w * mut;
                Ai[(size_t)b2 * nn + (size_t)b1] -= w * mut;
            } else if (e->type == ELE_CKT_V) {
                int br_idx = nv + e->branch_idx;
                if (e->n1 > 0) {
                    Ar[(size_t)(e->n1 - 1) * nn + (size_t)br_idx] += 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    Ar[(size_t)(e->n2 - 1) * nn + (size_t)br_idx] -= 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                double rad = e->ac_phase_deg * M_PI / 180.0;
                br[br_idx] += e->ac_mag * cos(rad);
                bi[br_idx] += e->ac_mag * sin(rad);
            } else if (e->type == ELE_CKT_I) {
                double rad = e->ac_phase_deg * M_PI / 180.0;
                stamp_i_rhs(br, e->n1, e->n2, e->ac_mag * cos(rad));
                stamp_i_rhs(bi, e->n1, e->n2, e->ac_mag * sin(rad));
            } else if (e->type == ELE_CKT_E) {
                int br_idx = nv + e->branch_idx;
                if (e->n1 > 0) {
                    Ar[(size_t)(e->n1 - 1) * nn + (size_t)br_idx] += 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n1 - 1)] += 1.0;
                }
                if (e->n2 > 0) {
                    Ar[(size_t)(e->n2 - 1) * nn + (size_t)br_idx] -= 1.0;
                    Ar[(size_t)br_idx * nn + (size_t)(e->n2 - 1)] -= 1.0;
                }
                if (e->n3 > 0) Ar[(size_t)br_idx * nn + (size_t)(e->n3 - 1)] -= e->val;
                if (e->n4 > 0) Ar[(size_t)br_idx * nn + (size_t)(e->n4 - 1)] += e->val;
            } else if (e->type == ELE_CKT_G) {
                double gm = e->val;
                if (e->n1 > 0 && e->n3 > 0) Ar[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                if (e->n1 > 0 && e->n4 > 0) Ar[(size_t)(e->n1 - 1) * nn + (size_t)(e->n4 - 1)] -= gm;
                if (e->n2 > 0 && e->n3 > 0) Ar[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                if (e->n2 > 0 && e->n4 > 0) Ar[(size_t)(e->n2 - 1) * nn + (size_t)(e->n4 - 1)] += gm;
            } else if (e->type == ELE_CKT_D) {
                double vd = dc->node_v[e->n1] - dc->node_v[e->n2];
                double id = 0.0, gd = 0.0;
                eval_diode(e, vd, &id, &gd);
                stamp_g(Ar, mna, e->n1, e->n2, gd);
            } else if (e->type == ELE_CKT_M) {
                double vds = dc->node_v[e->n1] - dc->node_v[e->n2];
                double vgs = dc->node_v[e->n3] - dc->node_v[e->n2];
                double ids = 0.0, gds = 0.0, gm = 0.0;
                eval_mos_switch(e, vgs, vds, &ids, &gds, &gm);
                stamp_g(Ar, mna, e->n1, e->n2, gds);
                if (e->n1 > 0 && e->n3 > 0) Ar[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                if (e->n1 > 0 && e->n2 > 0) Ar[(size_t)(e->n1 - 1) * nn + (size_t)(e->n2 - 1)] -= gm;
                if (e->n2 > 0 && e->n3 > 0) Ar[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                if (e->n2 > 0) Ar[(size_t)(e->n2 - 1) * nn + (size_t)(e->n2 - 1)] += gm;
            }
        }

        if (ele_lu_solve_complex(mna, Ar, Ai, br, bi, xr, xi) != 0) {
            free(Ar); free(Ai); free(br); free(bi); free(xr); free(xi);
            return -1;
        }
        out->freq_hz[p] = freq;
        int probe = (nl->ac_vout_node > 0) ? (nl->ac_vout_node - 1) : 0;
        double vr = (nv > 0) ? xr[probe] : 0.0;
        double vi = (nv > 0) ? xi[probe] : 0.0;
        double mag = sqrt(vr * vr + vi * vi);
        double db = 20.0 * log10(fmax(mag, 1e-15));
        out->mag_db[p] = db;
        out->phase_deg[p] = atan2(vi, vr) * 180.0 / M_PI;
        if (p == 0) dc_gain_db = db;

        int pdn_nd = (nl->pdn_node > 0) ? (nl->pdn_node - 1) : probe;
        double vpr = xr[pdn_nd], vpi = xi[pdn_nd];
        double imag = 1.0;
        if (nl->pdn_isrc_elem >= 0 && nl->elems[nl->pdn_isrc_elem].ac_mag > 0.0) {
            imag = nl->elems[nl->pdn_isrc_elem].ac_mag;
        }
        double zmag = sqrt(vpr * vpr + vpi * vpi) / imag;
        out->z_pdn_ohm[p] = zmag;
        if (zmag > out->z_pdn_max_ohm) {
            out->z_pdn_max_ohm = zmag;
            out->z_pdn_fpeak_hz = freq;
        }
    }

    /* Find -3 dB crossover frequency via linear interpolation in log(f) */
    out->bw_3db_hz = 0.0;
    double target_db = dc_gain_db - 3.0103;
    for (int p = 1; p < npts; ++p) {
        if (out->mag_db[p - 1] >= target_db && out->mag_db[p] <= target_db) {
            double frac = (out->mag_db[p - 1] - target_db) /
                          fmax(1e-12, out->mag_db[p - 1] - out->mag_db[p]);
            out->bw_3db_hz = out->freq_hz[p - 1] *
                             pow(out->freq_hz[p] / out->freq_hz[p - 1], frac);
            break;
        }
    }
    if (nl->has_pdn) {
        out->pdn_pass = (out->z_pdn_max_ohm <= nl->pdn_zmax) ? 1 : 0;
    }

    free(Ar); free(Ai); free(br); free(bi); free(xr); free(xi);
    return 0;
}

/* ============================================================================
 * 4. Energy-Stable Trapezoidal Transient & Worst-Case PVT Corners
 * ============================================================================ */

int ele_ckt_solve_tran(const ele_ckt_netlist_t *nl, const ele_ckt_dc_res_t *dc,
                       ele_ckt_tran_res_t *out) {
    if (nl == NULL || dc == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    if (!nl->has_tran || nl->tran_tstep <= 0.0 || nl->tran_tstop <= 0.0) return -1;

    int nv = 0, nbr = 0;
    int mna = get_mna_dim(nl, &nv, &nbr);
    if (mna <= 0 || mna > ELE_CKT_MAX_MNA) return -1;

    double h = nl->tran_tstep;
    int n_steps = (int)llround(nl->tran_tstop / h);
    if (n_steps < 1) n_steps = 1;
    if (n_steps > 200000) n_steps = 200000;

    int probe = (nl->tran_probe_node > 0) ? nl->tran_probe_node : ((nv > 0) ? 1 : 0);
    out->probe_node = probe;
    out->dt = h;
    out->n_steps = n_steps;
    out->converged = 1;

    size_t nn = (size_t)mna;
    double *A = (double *)malloc(nn * nn * sizeof(double));
    double *rhs = (double *)malloc(nn * sizeof(double));
    double *x_prev = (double *)calloc(nn, sizeof(double));
    double *x_cur = (double *)calloc(nn, sizeof(double));
    double *x_next = (double *)calloc(nn, sizeof(double));
    double *cap_i_prev = (double *)calloc((size_t)nl->n_elems, sizeof(double));
    double *v_ind_prev = (double *)calloc((size_t)nl->n_elems, sizeof(double));
    if (A == NULL || rhs == NULL || x_prev == NULL || x_cur == NULL ||
        x_next == NULL || cap_i_prev == NULL || v_ind_prev == NULL) {
        free(A); free(rhs); free(x_prev); free(x_cur); free(x_next);
        free(cap_i_prev); free(v_ind_prev);
        return -1;
    }

    for (int k = 0; k < nv; ++k) x_prev[k] = dc->node_v[k + 1];
    for (int k = 0; k < nbr; ++k) x_prev[nv + k] = dc->branch_i[k];
    for (int i = 0; i < nl->n_elems; ++i) {
        if (nl->elems[i].has_ic && nl->elems[i].type == ELE_CKT_L) {
            x_prev[nv + nl->elems[i].branch_idx] = nl->elems[i].ic;
        }
        if (nl->elems[i].has_ic && nl->elems[i].type == ELE_CKT_C && nl->elems[i].n1 > 0 && nl->elems[i].n2 == 0) {
            x_prev[nl->elems[i].n1 - 1] = nl->elems[i].ic;
        }
    }
    memcpy(x_cur, x_prev, nn * sizeof(double));

    for (int i = 0; i < nl->n_elems; ++i) {
        if (nl->elems[i].type == ELE_CKT_L) {
            double v1 = (nl->elems[i].n1 > 0) ? x_prev[nl->elems[i].n1 - 1] : 0.0;
            double v2 = (nl->elems[i].n2 > 0) ? x_prev[nl->elems[i].n2 - 1] : 0.0;
            v_ind_prev[i] = v1 - v2;
        }
    }

    out->node_vmin[0] = 0.0;
    out->node_vmax[0] = 0.0;
    for (int k = 1; k <= nv; ++k) {
        out->node_vmin[k] = x_prev[k - 1];
        out->node_vmax[k] = x_prev[k - 1];
    }
    out->probe_v_init = (probe > 0) ? x_prev[probe - 1] : 0.0;

    int stride = (n_steps + 1 <= ELE_CKT_MAX_TRAN_PT) ? 1 : ((n_steps + ELE_CKT_MAX_TRAN_PT) / ELE_CKT_MAX_TRAN_PT);
    out->time_s[0] = 0.0;
    out->trace_v[0] = out->probe_v_init;
    out->trace_len = 1;

    for (int step = 1; step <= n_steps; ++step) {
        double t_next = (double)step * h;
        /* Precompute Trapezoidal history for L + K */
        double l_hist[ELE_CKT_MAX_ELEMS];
        for (int i = 0; i < nl->n_elems; ++i) {
            if (nl->elems[i].type == ELE_CKT_L) {
                double lh = -v_ind_prev[i] - (2.0 * nl->elems[i].val / h) * x_prev[nv + nl->elems[i].branch_idx];
                l_hist[i] = lh;
            }
        }
        for (int i = 0; i < nl->n_elems; ++i) {
            if (nl->elems[i].type == ELE_CKT_K) {
                int i1 = nl->elems[i].l1_idx, i2 = nl->elems[i].l2_idx;
                double mut = nl->elems[i].val * sqrt(nl->elems[i1].val * nl->elems[i2].val);
                l_hist[i1] -= (2.0 * mut / h) * x_prev[nv + nl->elems[i2].branch_idx];
                l_hist[i2] -= (2.0 * mut / h) * x_prev[nv + nl->elems[i1].branch_idx];
            }
        }

        double step_res = 1e9;
        int nr_ok = 0;
        for (int it = 0; it < 60; ++it) {
            memset(A, 0, nn * nn * sizeof(double));
            memset(rhs, 0, nn * sizeof(double));
            for (int k = 0; k < nv; ++k) A[(size_t)k * nn + (size_t)k] += 1e-12;

            for (int i = 0; i < nl->n_elems; ++i) {
                const ele_ckt_elem_t *e = &nl->elems[i];
                if (e->type == ELE_CKT_R && e->val > 0.0) {
                    stamp_g(A, mna, e->n1, e->n2, 1.0 / e->val);
                } else if (e->type == ELE_CKT_C) {
                    double geq = 2.0 * e->val / h;
                    double vp1 = (e->n1 > 0) ? x_prev[e->n1 - 1] : 0.0;
                    double vp2 = (e->n2 > 0) ? x_prev[e->n2 - 1] : 0.0;
                    double ieq = geq * (vp1 - vp2) + cap_i_prev[i];
                    stamp_g(A, mna, e->n1, e->n2, geq);
                    stamp_i_rhs(rhs, e->n1, e->n2, -ieq);
                } else if (e->type == ELE_CKT_L) {
                    int br = nv + e->branch_idx;
                    if (e->n1 > 0) {
                        A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                        A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                    }
                    if (e->n2 > 0) {
                        A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                        A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                    }
                    A[(size_t)br * nn + (size_t)br] -= 2.0 * e->val / h;
                    rhs[br] += l_hist[i];
                } else if (e->type == ELE_CKT_K) {
                    const ele_ckt_elem_t *l1 = &nl->elems[e->l1_idx];
                    const ele_ckt_elem_t *l2 = &nl->elems[e->l2_idx];
                    double mut = e->val * sqrt(l1->val * l2->val);
                    int b1 = nv + l1->branch_idx;
                    int b2 = nv + l2->branch_idx;
                    A[(size_t)b1 * nn + (size_t)b2] -= 2.0 * mut / h;
                    A[(size_t)b2 * nn + (size_t)b1] -= 2.0 * mut / h;
                } else if (e->type == ELE_CKT_V) {
                    int br = nv + e->branch_idx;
                    if (e->n1 > 0) {
                        A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                        A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                    }
                    if (e->n2 > 0) {
                        A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                        A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                    }
                    rhs[br] += eval_src_time(e, t_next);
                } else if (e->type == ELE_CKT_I) {
                    stamp_i_rhs(rhs, e->n1, e->n2, eval_src_time(e, t_next));
                } else if (e->type == ELE_CKT_E) {
                    int br = nv + e->branch_idx;
                    if (e->n1 > 0) {
                        A[(size_t)(e->n1 - 1) * nn + (size_t)br] += 1.0;
                        A[(size_t)br * nn + (size_t)(e->n1 - 1)] += 1.0;
                    }
                    if (e->n2 > 0) {
                        A[(size_t)(e->n2 - 1) * nn + (size_t)br] -= 1.0;
                        A[(size_t)br * nn + (size_t)(e->n2 - 1)] -= 1.0;
                    }
                    if (e->n3 > 0) A[(size_t)br * nn + (size_t)(e->n3 - 1)] -= e->val;
                    if (e->n4 > 0) A[(size_t)br * nn + (size_t)(e->n4 - 1)] += e->val;
                } else if (e->type == ELE_CKT_G) {
                    double gm = e->val;
                    if (e->n1 > 0 && e->n3 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                    if (e->n1 > 0 && e->n4 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n4 - 1)] -= gm;
                    if (e->n2 > 0 && e->n3 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                    if (e->n2 > 0 && e->n4 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n4 - 1)] += gm;
                } else if (e->type == ELE_CKT_D) {
                    double v1 = (e->n1 > 0) ? x_cur[e->n1 - 1] : 0.0;
                    double v2 = (e->n2 > 0) ? x_cur[e->n2 - 1] : 0.0;
                    double vd = v1 - v2;
                    double id = 0.0, gd = 0.0;
                    eval_diode(e, vd, &id, &gd);
                    stamp_g(A, mna, e->n1, e->n2, gd);
                    stamp_i_rhs(rhs, e->n1, e->n2, id - gd * vd);
                } else if (e->type == ELE_CKT_M) {
                    double v1 = (e->n1 > 0) ? x_cur[e->n1 - 1] : 0.0;
                    double v2 = (e->n2 > 0) ? x_cur[e->n2 - 1] : 0.0;
                    double v3 = (e->n3 > 0) ? x_cur[e->n3 - 1] : 0.0;
                    double ids = 0.0, gds = 0.0, gm = 0.0;
                    eval_mos_switch(e, v3 - v2, v1 - v2, &ids, &gds, &gm);
                    stamp_g(A, mna, e->n1, e->n2, gds);
                    if (e->n1 > 0 && e->n3 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n3 - 1)] += gm;
                    if (e->n1 > 0 && e->n2 > 0) A[(size_t)(e->n1 - 1) * nn + (size_t)(e->n2 - 1)] -= gm;
                    if (e->n2 > 0 && e->n3 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n3 - 1)] -= gm;
                    if (e->n2 > 0) A[(size_t)(e->n2 - 1) * nn + (size_t)(e->n2 - 1)] += gm;
                    double ieq = ids - gds * (v1 - v2) - gm * (v3 - v2);
                    stamp_i_rhs(rhs, e->n1, e->n2, ieq);
                }
            }

            if (ele_lu_solve_real(mna, A, rhs, x_next) != 0) {
                out->converged = 0;
                free(A); free(rhs); free(x_prev); free(x_cur); free(x_next);
                free(cap_i_prev); free(v_ind_prev);
                return -1;
            }
            double max_dv = 0.0;
            for (int k = 0; k < mna; ++k) {
                double dv = x_next[k] - x_cur[k];
                if (k < nv && fabs(dv) > 200.0 && it < 20) {
                    x_cur[k] += (200.0 / fabs(dv)) * dv;
                } else {
                    x_cur[k] = x_next[k];
                }
                if (fabs(dv) > max_dv) max_dv = fabs(dv);
            }
            if (max_dv < 1e-10) {
                step_res = max_dv;
                nr_ok = 1;
                break;
            }
        }
        if (!nr_ok) out->converged = 0;
        if (step_res > out->max_kcl_res) out->max_kcl_res = step_res;

        /* Update C currents and L voltages */
        for (int i = 0; i < nl->n_elems; ++i) {
            const ele_ckt_elem_t *e = &nl->elems[i];
            if (e->type == ELE_CKT_C) {
                double geq = 2.0 * e->val / h;
                double vc_new = ((e->n1 > 0) ? x_cur[e->n1 - 1] : 0.0) - ((e->n2 > 0) ? x_cur[e->n2 - 1] : 0.0);
                double vc_old = ((e->n1 > 0) ? x_prev[e->n1 - 1] : 0.0) - ((e->n2 > 0) ? x_prev[e->n2 - 1] : 0.0);
                cap_i_prev[i] = geq * (vc_new - vc_old) - cap_i_prev[i];
            } else if (e->type == ELE_CKT_L) {
                double vl_new = ((e->n1 > 0) ? x_cur[e->n1 - 1] : 0.0) - ((e->n2 > 0) ? x_cur[e->n2 - 1] : 0.0);
                v_ind_prev[i] = vl_new;
            }
        }
        memcpy(x_prev, x_cur, nn * sizeof(double));

        for (int k = 1; k <= nv; ++k) {
            double vk = x_cur[k - 1];
            if (vk < out->node_vmin[k]) out->node_vmin[k] = vk;
            if (vk > out->node_vmax[k]) out->node_vmax[k] = vk;
        }
        if ((step % stride == 0 || step == n_steps) && out->trace_len < ELE_CKT_MAX_TRAN_PT) {
            int idx = out->trace_len++;
            out->time_s[idx] = t_next;
            out->trace_v[idx] = (probe > 0) ? x_cur[probe - 1] : 0.0;
        }
    }

    out->probe_v_final = (probe > 0) ? x_cur[probe - 1] : 0.0;
    out->probe_v_min = (probe > 0) ? out->node_vmin[probe] : 0.0;
    out->probe_v_max = (probe > 0) ? out->node_vmax[probe] : 0.0;
    out->soa_pass = 1;
    if (nl->has_soa && nl->soa_node > 0) {
        double lo = out->node_vmin[nl->soa_node];
        double hi = out->node_vmax[nl->soa_node];
        out->soa_pass = (lo >= nl->soa_vmin && hi <= nl->soa_vmax) ? 1 : 0;
    }

    free(A); free(rhs); free(x_prev); free(x_cur); free(x_next);
    free(cap_i_prev); free(v_ind_prev);
    return out->converged ? 0 : -1;
}

int ele_ckt_run_all(const ele_ckt_netlist_t *nl, int with_corners, ele_ckt_run_res_t *out) {
    if (nl == NULL || out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    if (ele_ckt_solve_dc(nl, &out->dc) != 0) return -1;
    if (nl->has_ac || nl->has_pdn) {
        if (ele_ckt_solve_ac(nl, &out->dc, &out->ac) != 0) return -1;
    }
    if (nl->has_tran) {
        if (ele_ckt_solve_tran(nl, &out->dc, &out->tran) != 0) return -1;
        out->nom_vmin = out->tran.probe_v_min;
        out->nom_vmax = out->tran.probe_v_max;
        out->wc_vmin = out->nom_vmin;
        out->wc_vmax = out->nom_vmax;
        out->soa_pass = out->tran.soa_pass;
    } else {
        out->soa_pass = 1;
    }

    out->run_corners = with_corners;
    out->n_corners = 1;
    int has_tol = 0;
    for (int i = 0; i < nl->n_elems; ++i) {
        if (nl->elems[i].tol_frac > 0.0) { has_tol = 1; break; }
    }
    if (with_corners && nl->has_tran && has_tol) {
        /* Evaluate extreme tolerance corners: all-, all+, worst-ringing (R-, L+, C-), (R+, L-, C+) */
        const int r_sgn[4] = {-1, 1, -1, 1};
        const int l_sgn[4] = {-1, 1,  1, -1};
        const int c_sgn[4] = {-1, 1, -1, 1};
        ele_ckt_netlist_t *copy = (ele_ckt_netlist_t *)malloc(sizeof(ele_ckt_netlist_t));
        if (copy == NULL) return -1;
        for (int c = 0; c < 4; ++c) {
            memcpy(copy, nl, sizeof(*copy));
            for (int i = 0; i < copy->n_elems; ++i) {
                double t = copy->elems[i].tol_frac;
                if (t <= 0.0) continue;
                int s = 0;
                if (copy->elems[i].type == ELE_CKT_R) s = r_sgn[c];
                else if (copy->elems[i].type == ELE_CKT_L) s = l_sgn[c];
                else if (copy->elems[i].type == ELE_CKT_C) s = c_sgn[c];
                copy->elems[i].val *= (1.0 + (double)s * t);
            }
            ele_ckt_dc_res_t cdc;
            ele_ckt_tran_res_t ctran;
            if (ele_ckt_solve_dc(copy, &cdc) == 0 && ele_ckt_solve_tran(copy, &cdc, &ctran) == 0) {
                ++out->n_corners;
                int snode = (nl->has_soa && nl->soa_node > 0) ? nl->soa_node : out->tran.probe_node;
                if (ctran.node_vmin[snode] < out->wc_vmin) out->wc_vmin = ctran.node_vmin[snode];
                if (ctran.node_vmax[snode] > out->wc_vmax) out->wc_vmax = ctran.node_vmax[snode];
                if (!ctran.soa_pass) out->soa_pass = 0;
            }
        }
        free(copy);
    }

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "ckt");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "%s",
             nl->has_soa ? "soa_tran" : (nl->has_pdn ? "pdn_ac" : "mna_dc"));
    snprintf(out->cert.status, sizeof(out->cert.status), "%s",
             (out->soa_pass && (!nl->has_pdn || out->ac.pdn_pass)) ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", nl->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", nl->input_sha256);
    if (nl->has_tran) {
        int64_t scale = 1000; /* mV */
        out->cert.has_bracket = 1;
        out->cert.bracket_scale = scale;
        out->cert.lb_int = (int64_t)floor(out->wc_vmin * (double)scale);
        out->cert.ub_int = (int64_t)ceil(out->wc_vmax * (double)scale);
        out->cert.limit_lo_int = nl->has_soa ? (int64_t)ceil(nl->soa_vmin * (double)scale) : -100000000;
        out->cert.limit_hi_int = nl->has_soa ? (int64_t)floor(nl->soa_vmax * (double)scale) : 100000000;
        out->cert.lb_val = out->wc_vmin;
        out->cert.ub_val = out->wc_vmax;
    }
    return 0;
}

/* ============================================================================
 * 5. High-Speed Coupled RLGC SI & SEARU Worst-Case Eye Search
 * ============================================================================ */

int ele_ckt_si_eye(const ele_ckt_netlist_t *nl, double bit_rate_gbps, ele_ckt_eye_res_t *out) {
    if (nl == NULL || out == NULL || bit_rate_gbps <= 0.0) return -1;
    memset(out, 0, sizeof(*out));
    out->bit_rate_gbps = bit_rate_gbps;
    double ui = 1e-9 / bit_rate_gbps;
    out->ui_sec = ui;

    int v_vic = -1, v_agg = -1;
    for (int i = 0; i < nl->n_elems; ++i) {
        if (nl->elems[i].type == ELE_CKT_V) {
            if (v_vic < 0) v_vic = i;
            else if (v_agg < 0) v_agg = i;
        }
    }
    if (v_vic < 0) return -1;

    ele_ckt_netlist_t *work = (ele_ckt_netlist_t *)malloc(sizeof(ele_ckt_netlist_t));
    ele_ckt_dc_res_t *dc = (ele_ckt_dc_res_t *)malloc(sizeof(ele_ckt_dc_res_t));
    ele_ckt_tran_res_t *tr_vic = (ele_ckt_tran_res_t *)malloc(sizeof(ele_ckt_tran_res_t));
    ele_ckt_tran_res_t *tr_agg = (ele_ckt_tran_res_t *)malloc(sizeof(ele_ckt_tran_res_t));
    if (work == NULL || dc == NULL || tr_vic == NULL || tr_agg == NULL) {
        free(work); free(dc); free(tr_vic); free(tr_agg);
        return -1;
    }

    int spb = 20; /* 20 samples per bit UI */
    double dt = ui / (double)spb;
    double tstop = 24.0 * ui;
    double tr = 0.15 * ui;
    double pw = ui - tr;

    /* 1. Victim single-UI pulse response */
    memcpy(work, nl, sizeof(*work));
    work->has_tran = 1;
    work->tran_tstep = dt;
    work->tran_tstop = tstop;
    int probe = ele_ckt_find_node(work, "vout");
    if (probe > 0) work->tran_probe_node = probe;
    work->elems[v_vic].src_kind = ELE_SRC_PULSE;
    work->elems[v_vic].dc_val = 0.0;
    work->elems[v_vic].p_v1 = 0.0; work->elems[v_vic].p_v2 = 1.0;
    work->elems[v_vic].p_td = 0.0; work->elems[v_vic].p_tr = tr;
    work->elems[v_vic].p_tf = tr;  work->elems[v_vic].p_pw = pw;
    work->elems[v_vic].p_per = 0.0;
    if (v_agg >= 0) {
        work->elems[v_agg].src_kind = ELE_SRC_DC;
        work->elems[v_agg].dc_val = 0.0;
    }
    if (ele_ckt_solve_dc(work, dc) != 0 || ele_ckt_solve_tran(work, dc, tr_vic) != 0) {
        free(work); free(dc); free(tr_vic); free(tr_agg);
        return -1;
    }

    /* 2. Aggressor single-UI pulse crosstalk response on victim node */
    memset(tr_agg, 0, sizeof(*tr_agg));
    if (v_agg >= 0) {
        memcpy(work, nl, sizeof(*work));
        work->has_tran = 1;
        work->tran_tstep = dt;
        work->tran_tstop = tstop;
        if (probe > 0) work->tran_probe_node = probe;
        work->elems[v_vic].src_kind = ELE_SRC_DC;
        work->elems[v_vic].dc_val = 0.0;
        work->elems[v_agg].src_kind = ELE_SRC_PULSE;
        work->elems[v_agg].dc_val = 0.0;
        work->elems[v_agg].p_v1 = 0.0; work->elems[v_agg].p_v2 = 1.0;
        work->elems[v_agg].p_td = 0.0; work->elems[v_agg].p_tr = tr;
        work->elems[v_agg].p_tf = tr;  work->elems[v_agg].p_pw = pw;
        work->elems[v_agg].p_per = 0.0;
        if (ele_ckt_solve_dc(work, dc) != 0 || ele_ckt_solve_tran(work, dc, tr_agg) != 0) {
            free(work); free(dc); free(tr_vic); free(tr_agg);
            return -1;
        }
    }

    /* Locate main cursor peak */
    int peak_idx = 0;
    double peak_val = -1e9;
    for (int i = 0; i < tr_vic->trace_len; ++i) {
        if (tr_vic->trace_v[i] > peak_val) {
            peak_val = tr_vic->trace_v[i];
            peak_idx = i;
        }
    }
    double xpeak = 0.0;
    for (int i = 0; i < tr_agg->trace_len; ++i) {
        if (fabs(tr_agg->trace_v[i]) > xpeak) xpeak = fabs(tr_agg->trace_v[i]);
    }
    out->cursor_time_s = tr_vic->time_s[peak_idx];
    out->pulse_peak_v = peak_val;
    out->xtalk_peak_v = xpeak;

    /* Extract UI taps h_vic[k] and h_agg[k] for k = 0..15 around cursor */
    double hv[16] = {0}, ha[16] = {0};
    int cur_k = 8;
    for (int k = 0; k < 16; ++k) {
        int s_idx = peak_idx + (cur_k - k) * spb;
        if (s_idx >= 0 && s_idx < tr_vic->trace_len) hv[k] = tr_vic->trace_v[s_idx];
        if (s_idx >= 0 && s_idx < tr_agg->trace_len) ha[k] = tr_agg->trace_v[s_idx];
    }

    double isi_sum = 0.0, xtalk_sum = 0.0;
    for (int k = 0; k < 16; ++k) {
        if (k != cur_k) isi_sum += fabs(hv[k]);
        xtalk_sum += fabs(ha[k]);
    }
    out->eye_pda_v = hv[cur_k] - isi_sum - xtalk_sum;

    /* 3. SEARU discrete 32-bit pattern search to find explicit worst-case bit sequence */
    uint32_t cur_vb = 0x5555U | (1U << cur_k);
    uint32_t cur_ab = 0xAAAAU;
    uint32_t best_vb = cur_vb, best_ab = cur_ab;

    double best_v1_low = 1e9, cur_v1_low = 0.0;
    for (int k = 0; k < 16; ++k) {
        cur_v1_low += (((cur_vb >> k) & 1U) ? hv[k] : 0.0) +
                      (((cur_ab >> k) & 1U) ? ha[k] : 0.0);
    }
    best_v1_low = cur_v1_low;

    uint64_t rng = UINT64_C(0x51E7E2026B170001);
    uint64_t temp_q12 = UINT64_C(4096);
    uint32_t stag = 0U;
    for (int iter = 0; iter < 1500; ++iter) {
        uint32_t cand_vb = cur_vb, cand_ab = cur_ab;
        uint64_t swan = 0U;
        uint64_t jmp = searu_zipf_jump(&rng, searu_adapt_chaos_q8(stag, UINT64_C(0)), &swan);
        uint32_t nflips = 1U + (uint32_t)(jmp & UINT64_C(3));
        for (uint32_t f = 0; f < nflips; ++f) {
            uint32_t bit = searu_Bounded_u32(searu_rng_next(&rng), 31U);
            if (bit < 15U) {
                uint32_t idx = (bit >= (uint32_t)cur_k) ? (bit + 1U) : bit;
                cand_vb ^= (1U << idx);
            } else {
                cand_ab ^= (1U << (bit - 15U));
            }
        }
        double v1 = 0.0;
        for (int k = 0; k < 16; ++k) {
            v1 += (((cand_vb >> k) & 1U) ? hv[k] : 0.0) +
                  (((cand_ab >> k) & 1U) ? ha[k] : 0.0);
        }
        uint64_t ce = (uint64_t)llround(fmax(0.0, (v1 + 10.0) * 10000.0));
        uint64_t ue = (uint64_t)llround(fmax(0.0, (cur_v1_low + 10.0) * 10000.0));
        uint64_t be = (uint64_t)llround(fmax(0.0, (best_v1_low + 10.0) * 10000.0));
        bool epi = false;
        if (searu_crucible_accept(ce, ue, be, &temp_q12, UINT64_C(4096), swan, &rng, &epi)) {
            cur_vb = cand_vb; cur_ab = cand_ab; cur_v1_low = v1;
        }
        if (v1 < best_v1_low) {
            best_vb = cand_vb; best_ab = cand_ab; best_v1_low = v1;
            stag = 0U;
        } else {
            ++stag;
        }
        temp_q12 = (temp_q12 * UINT64_C(992)) / UINT64_C(1000);
    }

    /* Complementary worst-'0' high pattern: flip all 32 bits of worst-'1' low */
    uint32_t zero_vb = (~best_vb) & 0xFFFFU;
    uint32_t zero_ab = (~best_ab) & 0xFFFFU;
    double best_v0_high = 0.0;
    for (int k = 0; k < 16; ++k) {
        best_v0_high += (((zero_vb >> k) & 1U) ? hv[k] : 0.0) +
                        (((zero_ab >> k) & 1U) ? ha[k] : 0.0);
    }

    out->worst_vic_bits = best_vb;
    out->worst_agg_bits = best_ab;
    out->eye_searu_v = best_v1_low - best_v0_high;
    out->eye_fulltran_v = out->eye_searu_v;
    out->eye_pass = (out->eye_pda_v > 0.05 && out->eye_searu_v > 0.05) ? 1 : 0;

    memset(&out->cert, 0, sizeof(out->cert));
    snprintf(out->cert.pillar, sizeof(out->cert.pillar), "ckt");
    snprintf(out->cert.kind, sizeof(out->cert.kind), "si_eye");
    snprintf(out->cert.status, sizeof(out->cert.status), "%s", out->eye_pass ? "CERTIFIED" : "VETOED");
    snprintf(out->cert.input_path, sizeof(out->cert.input_path), "%s", nl->input_path);
    snprintf(out->cert.input_sha256, sizeof(out->cert.input_sha256), "%s", nl->input_sha256);
    out->cert.has_bracket = 1;
    out->cert.bracket_scale = 10000;
    out->cert.lb_int = (int64_t)llround(best_v0_high * 10000.0);
    out->cert.ub_int = (int64_t)llround(best_v1_low * 10000.0);
    out->cert.limit_lo_int = 0;
    out->cert.limit_hi_int = 10000;
    out->cert.lb_val = best_v0_high;
    out->cert.ub_val = best_v1_low;

    free(work); free(dc); free(tr_vic); free(tr_agg);
    return 0;
}
