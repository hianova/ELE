/* SPDX-License-Identifier: MIT */
#include "ele_cert.h"
#include "ele_em.h"
#include "ele_ckt.h"
#include "ele_ee.h"
#include "ele_frontier.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int cli_strcasecmp(const char *a, const char *b) {
    while (*a != '\0' && *b != '\0') {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        ++a; ++b;
    }
    return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

static int cli_strncasecmp(const char *a, const char *b, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        int ca = tolower((unsigned char)a[i]);
        int cb = tolower((unsigned char)b[i]);
        if (ca != cb) return ca - cb;
        if (ca == '\0') return 0;
    }
    return 0;
}

static void print_usage(void) {
    printf(
        "ELE v%s — Certified Multiphysics Electronics, Electrical Engineering & Electromagnetics\n"
        "Usage:\n"
        "  ele_cli --version | --help\n"
        "  ele_cli selftest [--json <out.json>]\n"
        "  ele_cli em passivity <file.sNp> [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli em fdtd [--er <val>] [--sigma <val>] [--nx <n>] [--ny <n>] [--nt <n>] [--s2p <out.s2p>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli em peec <file.peec> [--s2p <out.s2p>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli em synth --mode array|match [--seed <u64>] [--json <out.json>]\n"
        "  ele_cli em bound [--kl <v>] [--zeta <v>] [--xi0 <v>] [--cells <n>] [--bw-frac <f>] [--fdtd-check] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli em bode-fano [--z0 <v>] [--rl <v>] [--cl-pf <v>] [--flo-ghz <v>] [--fhi-ghz <v>] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli em enforce-passivity <in.sNp> [--out <rep.sNp>] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli em kyp-sim <in.sNp> [--rho <v>] [--steps <n>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli ckt run <file.sp> [--corners] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli ckt si-eye <file.sp> [--rate-gbps <val>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli motor run <file.motor> [--rpm <val>] [--torque <val>] [--sweep] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli grid pf <file.grid> [--n1] [--json <out.json>]\n"
        "  ele_cli grid opf <file.grid> [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli grid chordal <file.grid> [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli grid sos2 <file.grid> [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli grid observable <file.grid> --obs <vm2:ID|pflow:IDX> [--eps <v>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli grid emt <file.grid> --fault-bus <id> --t-clear <sec> [--trip-branch <idx>] [--cert <out.json>] [--emit-lean <out.lean> --name <Mod>] [--json <out.json>]\n"
        "  ele_cli lab validate-thermal <thermal.csv> [--sp <out.sp>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli lab validate-csi <csi.csv> [--s2p <out.s2p>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli lab validate-power <power.csv> [--sp <out.sp>] [--cert <out.json>] [--json <out.json>]\n"
        "  ele_cli cert-verify <cert.elecert.json>\n"
        "  ele_cli field-to-circuit --peec <file.peec> --sp <file.sp> --motor <file.motor> [--out-dir <dir>] [--json <out.json>]\n"
        "  ele_cli bench --dir <benchmarks> --out <bench.json> --cert-dir <certs>\n"
        "  ele_cli atlas [--dir <benchmarks>] [--out <atlas.json>] [--cert-dir <certs>]\n",
        ELE_VERSION_STRING);
}

static int file_exists_readable(const char *path) {
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) return 0;
    fclose(fp);
    return 1;
}

static void build_test_ieee3(ele_grid_t *g) {
    memset(g, 0, sizeof(*g));
    g->base_mva = 100.0;
    g->n_bus = 3;
    g->n_gen = 2;
    g->n_branch = 3;
    g->slack_idx = 0;
    snprintf(g->input_path, sizeof(g->input_path), "internal_ieee3");
    ele_sha256_bytes((const uint8_t *)"ieee3", 5U, g->input_sha256);

    g->bus[0] = (ele_grid_bus_t){1, 3, 0.0, 0.0, 0.0, 0.0, 1.02, 0.0, 0.95, 1.05};
    g->bus[1] = (ele_grid_bus_t){2, 2, 50.0, 15.0, 0.0, 0.0, 1.02, 0.0, 0.95, 1.05};
    g->bus[2] = (ele_grid_bus_t){3, 1, 180.0, 45.0, 0.0, 0.0, 1.00, 0.0, 0.95, 1.05};

    g->gen[0] = (ele_grid_gen_t){1, 0, 80.0, 0.0, 10.0, 200.0, -100.0, 100.0, 1.02, 0.11, 5.0, 0.0, 6.0, 0.08, 1.5, 0};
    g->gen[1] = (ele_grid_gen_t){2, 1, 100.0, 0.0, 10.0, 200.0, -100.0, 100.0, 1.02, 0.085, 1.2, 0.0, 4.5, 0.10, 1.5, 1};

    g->branch[0] = (ele_grid_branch_t){1, 2, 0, 1, 0.015, 0.08, 0.02, 1.0, 0.0, 220.0, -60.0, 60.0};
    g->branch[1] = (ele_grid_branch_t){2, 3, 1, 2, 0.020, 0.10, 0.02, 1.0, 0.0, 220.0, -60.0, 60.0};
    g->branch[2] = (ele_grid_branch_t){1, 3, 0, 2, 0.025, 0.12, 0.02, 1.0, 0.0, 220.0, -60.0, 60.0};
}

static void build_test_ieee9(ele_grid_t *g) {
    memset(g, 0, sizeof(*g));
    g->base_mva = 100.0;
    g->n_bus = 9;
    g->n_gen = 3;
    g->n_branch = 9;
    g->slack_idx = 0;
    snprintf(g->input_path, sizeof(g->input_path), "internal_ieee9");
    ele_sha256_bytes((const uint8_t *)"ieee9", 5U, g->input_sha256);

    g->bus[0] = (ele_grid_bus_t){1, 3, 0.0, 0.0, 0.0, 0.0, 1.04, 0.0, 0.90, 1.10};
    g->bus[1] = (ele_grid_bus_t){2, 2, 0.0, 0.0, 0.0, 0.0, 1.025, 0.0, 0.90, 1.10};
    g->bus[2] = (ele_grid_bus_t){3, 2, 0.0, 0.0, 0.0, 0.0, 1.025, 0.0, 0.90, 1.10};
    g->bus[3] = (ele_grid_bus_t){4, 1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};
    g->bus[4] = (ele_grid_bus_t){5, 1, 90.0, 30.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};
    g->bus[5] = (ele_grid_bus_t){6, 1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};
    g->bus[6] = (ele_grid_bus_t){7, 1, 100.0, 35.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};
    g->bus[7] = (ele_grid_bus_t){8, 1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};
    g->bus[8] = (ele_grid_bus_t){9, 1, 125.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.90, 1.10};

    g->gen[0] = (ele_grid_gen_t){1, 0, 72.0, 0.0, 10.0, 250.0, -300.0, 300.0, 1.04, 0.11, 5.0, 150.0, 23.64, 0.0608, 2.0, 0};
    g->gen[1] = (ele_grid_gen_t){2, 1, 163.0, 0.0, 10.0, 300.0, -300.0, 300.0, 1.025, 0.085, 1.2, 600.0, 6.40, 0.1198, 1.5, 0};
    g->gen[2] = (ele_grid_gen_t){3, 2, 85.0, 0.0, 10.0, 270.0, -300.0, 300.0, 1.025, 0.1225, 1.0, 335.0, 3.01, 0.1813, 1.2, 1};

    g->branch[0] = (ele_grid_branch_t){1, 4, 0, 3, 0.0001, 0.0576, 0.000, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[1] = (ele_grid_branch_t){4, 5, 3, 4, 0.0170, 0.0920, 0.158, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[2] = (ele_grid_branch_t){5, 6, 4, 5, 0.0390, 0.1700, 0.358, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[3] = (ele_grid_branch_t){3, 6, 2, 5, 0.0001, 0.0586, 0.000, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[4] = (ele_grid_branch_t){6, 7, 5, 6, 0.0119, 0.1008, 0.209, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[5] = (ele_grid_branch_t){7, 8, 6, 7, 0.0085, 0.0720, 0.149, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[6] = (ele_grid_branch_t){8, 2, 7, 1, 0.0001, 0.0625, 0.000, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[7] = (ele_grid_branch_t){8, 9, 7, 8, 0.0320, 0.1610, 0.306, 1.0, 0.0, 300.0, -60.0, 60.0};
    g->branch[8] = (ele_grid_branch_t){9, 4, 8, 3, 0.0100, 0.0850, 0.176, 1.0, 0.0, 300.0, -60.0, 60.0};
}

static int run_selftest(const char *json_out) {
    int passed = 0, failed = 0;

    /* T01: SHA-256 NIST KAT */
    {
        char h[65];
        ele_sha256_bytes((const uint8_t *)"abc", 3U, h);
        int ok = (strcmp(h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);
        printf("  [T01] SHA-256 NIST KAT                : %s\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed; else ++failed;
    }

    /* T02: Integer Cholesky+SDD PSD Certifier & Tamper Rejection */
    {
        int64_t A[9] = {500, 120, -40, 120, 600, 80, -40, 80, 450};
        int64_t L[9] = {0};
        int64_t mu = 0;
        int ok1 = (ele_cert_cholesky_sdd_i64(3, A, L, &mu) == 0 && mu > -100 &&
                   ele_verify_cholesky_sdd_i64(3, A, L, mu) == 0);
        int ok2 = (ele_verify_cholesky_sdd_i64(3, A, L, mu + 50000) != 0);
        int ok = ok1 && ok2;
        printf("  [T02] 128b Cholesky+SDD & Tamper Trap : %s (mu=%" PRId64 ")\n", ok ? "PASS" : "FAIL", mu);
        if (ok) ++passed; else ++failed;
    }

    /* T03: Touchstone .s2p ordering & round-trip */
    {
        ele_snp_t s1, s2;
        memset(&s1, 0, sizeof(s1));
        s1.nports = 2; s1.nfreq = 2; s1.z0 = 50.0;
        s1.freq_hz[0] = 1e9; s1.freq_hz[1] = 2e9;
        s1.sr[0][0] = 0.05; s1.si[0][0] = -0.02;
        s1.sr[0][1] = 0.01; s1.si[0][1] = -0.92; /* S12 */
        s1.sr[0][2] = 0.02; s1.si[0][2] = -0.90; /* S21 (asymmetric test for order!) */
        s1.sr[0][3] = 0.04; s1.si[0][3] = -0.01;
        s1.sr[1][0] = 0.06; s1.sr[1][1] = 0.02; s1.sr[1][2] = 0.03; s1.sr[1][3] = 0.05;
        const char *tmpf = "build/.ele_st_t03.s2p";
        if (ele_snp_save(tmpf, &s1) != 0) tmpf = ".ele_st_t03.s2p";
        int ok = (ele_snp_save(tmpf, &s1) == 0 && ele_snp_load(tmpf, 2, &s2) == 0 &&
                  fabs(s2.si[0][1] - (-0.92)) < 1e-8 && fabs(s2.si[0][2] - (-0.90)) < 1e-8);
        remove(tmpf);
        printf("  [T03] Touchstone v1.1 S11/S21/S12/S22 : %s\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed; else ++failed;
    }

    /* T04: Passive 2-port & 3-port Zero-FP-Trust Passivity + Reciprocity */
    {
        ele_snp_t s3;
        memset(&s3, 0, sizeof(s3));
        s3.nports = 3; s3.nfreq = 3; s3.z0 = 50.0;
        double invr2 = 0.685; /* slightly lossy Wilkinson divider (-3.28 dB) */
        for (int f = 0; f < 3; ++f) {
            s3.freq_hz[f] = (2.0 + 0.5 * (double)f) * 1e9;
            s3.sr[f][0] = 0.02; s3.si[f][0] = -0.01;
            s3.sr[f][1] = 0.0;  s3.si[f][1] = -invr2;
            s3.sr[f][2] = 0.0;  s3.si[f][2] = -invr2;
            s3.sr[f][3] = 0.0;  s3.si[f][3] = -invr2;
            s3.sr[f][4] = 0.02; s3.si[f][4] = -0.01;
            s3.sr[f][5] = 0.03; s3.si[f][5] = 0.01;
            s3.sr[f][6] = 0.0;  s3.si[f][6] = -invr2;
            s3.sr[f][7] = 0.03; s3.si[f][7] = 0.01;
            s3.sr[f][8] = 0.02; s3.si[f][8] = -0.01;
        }
        ele_em_passivity_res_t pr;
        int ok = (ele_em_certify_snp(&s3, &pr) == 0 && pr.passive_certified == 1 &&
                  pr.reciprocal_certified == 1 && pr.sigma_max < 1.0);
        printf("  [T04] 3-Port Wilkinson Passivity Cert : %s (smax=%.4f)\n", ok ? "PASS" : "FAIL", pr.sigma_max);
        if (ok) ++passed; else ++failed;
    }

    /* T05: Adversarial Non-Passive Surrogate Veto */
    {
        ele_snp_t snp;
        memset(&snp, 0, sizeof(snp));
        snp.nports = 2; snp.nfreq = 2; snp.z0 = 50.0;
        snp.freq_hz[0] = 1e9; snp.freq_hz[1] = 5e9;
        snp.sr[0][0] = 0.10; snp.sr[0][1] = 0.90; snp.sr[0][2] = 0.90; snp.sr[0][3] = 0.10;
        snp.sr[1][0] = 0.18; snp.sr[1][1] = 0.855; snp.sr[1][2] = 0.855; snp.sr[1][3] = 0.18; /* smax=1.035 > 1 */
        ele_em_passivity_res_t pr;
        int ok = (ele_em_certify_snp(&snp, &pr) == 0 && pr.passive_certified == 0 &&
                  pr.sigma_max > 1.03 && strcmp(pr.cert.status, "VETOED") == 0);
        printf("  [T05] Non-Passive Surrogate Veto      : %s (smax=%.4f -> VETOED)\n", ok ? "PASS" : "FAIL", pr.sigma_max);
        if (ok) ++passed; else ++failed;
    }

    /* T06: 2D TMz Yee FDTD Dielectric-Slab Waveguide */
    {
        ele_em_fdtd_res_t fd;
        int ok = (ele_em_fdtd_run(4.0, 0.04, 300, 32, 2800, NULL, &fd) == 0 &&
                  fd.cfl_ok == 1 && fd.passivity.passive_certified == 1 &&
                  fd.min_s11_db < -10.0 && fd.max_s21_db > -2.0 && fd.max_energy_sum <= 1.001);
        printf("  [T06] 2D Yee FDTD Waveguide & CFL     : %s (minS11=%.1f dB, maxS21=%.2f dB, smax=%.4f)\n",
               ok ? "PASS" : "FAIL", fd.min_s11_db, fd.max_s21_db, fd.passivity.sigma_max);
        if (ok) ++passed; else ++failed;
    }

    /* T07: PEEC 800V Laminated Busbar Extraction */
    {
        ele_peec_spec_t ps = {0.20, 0.04, 0.002, 0.001, 4.2, 5.8e7, 1e6, 1e8, 41, 50.0, "internal_peec", ""};
        ele_sha256_bytes((const uint8_t *)"peec", 4U, ps.input_sha256);
        ele_em_peec_res_t pr;
        int ok = (ele_em_peec_extract(&ps, NULL, &pr) == 0 &&
                  pr.l_loop_h > 1e-9 && pr.l_loop_h < 50e-9 &&
                  pr.r_ac_fmax_ohm > pr.r_dc_ohm * 5.0 &&
                  pr.c_bus_f > 1e-10 && pr.passivity.passive_certified == 1);
        printf("  [T07] PEEC 800V Busbar Extraction     : %s (L=%.2f nH, C=%.1f pF, Rac/Rdc=%.1fx)\n",
               ok ? "PASS" : "FAIL", pr.l_loop_h * 1e9, pr.c_bus_f * 1e12, pr.r_ac_fmax_ohm / pr.r_dc_ohm);
        if (ok) ++passed; else ++failed;
    }

    /* T08: SEARU x FLOW Phased Array Synthesis */
    {
        ele_em_array_res_t ar;
        int ok = (ele_em_synth_array(2026U, &ar) == 0 &&
                  ar.nominal_sll_db <= -26.0 && ar.worst_tol_sll_db <= -21.0 &&
                  ar.baseline_sll_db > -14.0);
        printf("  [T08] SEARU Array Inverse Design      : %s (nom=%.1f dB, wc=%.1f dB vs unif=%.1f dB)\n",
               ok ? "PASS" : "FAIL", ar.nominal_sll_db, ar.worst_tol_sll_db, ar.baseline_sll_db);
        if (ok) ++passed; else ++failed;
    }

    /* T09: SEARU x FLOW 3-Section Broadband Match */
    {
        ele_em_match_res_t mr;
        int ok = (ele_em_synth_match(2026U, &mr) == 0 &&
                  mr.nominal_s11_db <= -18.0 && mr.worst_tol_s11_db <= -14.0 &&
                  mr.baseline_s11_db > -7.0);
        printf("  [T09] SEARU 3-Sec Broadband Match     : %s (nom=%.1f dB, wc=%.1f dB vs raw=%.1f dB)\n",
               ok ? "PASS" : "FAIL", mr.nominal_s11_db, mr.worst_tol_s11_db, mr.baseline_s11_db);
        if (ok) ++passed; else ++failed;
    }

    /* T10: MNA RC Analytical Step & -3 dB AC Bandwidth */
    {
        ele_ckt_netlist_t nl;
        memset(&nl, 0, sizeof(nl));
        nl.n_nodes = 3;
        snprintf(nl.node_names[0], 32, "0");
        snprintf(nl.node_names[1], 32, "vin");
        snprintf(nl.node_names[2], 32, "vout");
        nl.n_elems = 3;
        nl.elems[0] = (ele_ckt_elem_t){.name="V1", .type=ELE_CKT_V, .n1=1, .n2=0, .src_kind=ELE_SRC_PULSE,
            .dc_val=0.0, .ac_mag=1.0, .p_v1=0.0, .p_v2=1.0, .p_td=0.0, .p_tr=1e-9, .p_tf=1e-9, .p_pw=1.0, .branch_idx=0};
        nl.elems[1] = (ele_ckt_elem_t){.name="R1", .type=ELE_CKT_R, .n1=1, .n2=2, .val=1000.0, .branch_idx=-1};
        nl.elems[2] = (ele_ckt_elem_t){.name="C1", .type=ELE_CKT_C, .n1=2, .n2=0, .val=1e-6, .branch_idx=-1};
        nl.has_ac = 1; nl.ac_is_dec = 1; nl.ac_pts = 81; nl.ac_f1 = 1.0; nl.ac_f2 = 1e5; nl.ac_vout_node = 2;
        nl.has_tran = 1; nl.tran_tstep = 1e-5; nl.tran_tstop = 1e-3; nl.tran_probe_node = 2;
        ele_ckt_run_res_t rr;
        double fc_exact = 1.0 / (2.0 * M_PI * 1000.0 * 1e-6);
        double step_exact = 1.0 - exp(-1.0);
        int ok = (ele_ckt_run_all(&nl, 0, &rr) == 0 &&
                  fabs(rr.ac.bw_3db_hz - fc_exact) / fc_exact < 0.02 &&
                  fabs(rr.tran.probe_v_final - step_exact) < 0.002);
        printf("  [T10] MNA RC Step & -3dB BW           : %s (f3db=%.2f Hz, v(RC)=%.5f)\n",
               ok ? "PASS" : "FAIL", rr.ac.bw_3db_hz, rr.tran.probe_v_final);
        if (ok) ++passed; else ++failed;
    }

    /* T11: MNA Shockley Diode Nonlinear DC & RLC PDN Peak */
    {
        ele_ckt_netlist_t nl;
        memset(&nl, 0, sizeof(nl));
        nl.n_nodes = 3;
        snprintf(nl.node_names[0], 32, "0");
        snprintf(nl.node_names[1], 32, "vin");
        snprintf(nl.node_names[2], 32, "vd");
        nl.n_elems = 3;
        nl.elems[0] = (ele_ckt_elem_t){.name="V1", .type=ELE_CKT_V, .n1=1, .n2=0, .dc_val=5.0, .branch_idx=0};
        nl.elems[1] = (ele_ckt_elem_t){.name="R1", .type=ELE_CKT_R, .n1=1, .n2=2, .val=100.0, .branch_idx=-1};
        nl.elems[2] = (ele_ckt_elem_t){.name="D1", .type=ELE_CKT_D, .n1=2, .n2=0, .d_is=1e-12, .d_n=1.0, .d_rs=0.05, .branch_idx=-1};
        ele_ckt_dc_res_t dc;
        int ok1 = (ele_ckt_solve_dc(&nl, &dc) == 0 && dc.max_kcl_res < 1e-11 &&
                   dc.node_v[2] > 0.55 && dc.node_v[2] < 0.78);

        ele_ckt_netlist_t pdn;
        memset(&pdn, 0, sizeof(pdn));
        pdn.n_nodes = 3;
        snprintf(pdn.node_names[0], 32, "0"); snprintf(pdn.node_names[1], 32, "vrm"); snprintf(pdn.node_names[2], 32, "die");
        pdn.n_elems = 5;
        pdn.elems[0] = (ele_ckt_elem_t){.name="Rvrm", .type=ELE_CKT_R, .n1=1, .n2=0, .val=0.01, .branch_idx=-1};
        pdn.elems[1] = (ele_ckt_elem_t){.name="Lpkg", .type=ELE_CKT_L, .n1=1, .n2=2, .val=2e-9, .branch_idx=0};
        pdn.elems[2] = (ele_ckt_elem_t){.name="Cdie", .type=ELE_CKT_C, .n1=2, .n2=0, .val=200e-9, .branch_idx=-1};
        pdn.elems[3] = (ele_ckt_elem_t){.name="Resr", .type=ELE_CKT_R, .n1=2, .n2=0, .val=0.08, .branch_idx=-1};
        pdn.elems[4] = (ele_ckt_elem_t){.name="Iload", .type=ELE_CKT_I, .n1=2, .n2=0, .ac_mag=1.0, .branch_idx=-1};
        pdn.has_ac = 1; pdn.ac_is_dec = 1; pdn.ac_pts = 81; pdn.ac_f1 = 1e4; pdn.ac_f2 = 1e8;
        pdn.ac_vout_node = 2; pdn.has_pdn = 1; pdn.pdn_node = 2; pdn.pdn_isrc_elem = 4; pdn.pdn_zmax = 0.25;
        ele_ckt_run_res_t prr;
        int ok2 = (ele_ckt_run_all(&pdn, 0, &prr) == 0 && prr.ac.pdn_pass == 1 && prr.ac.z_pdn_max_ohm < 0.25);
        int ok = ok1 && ok2;
        printf("  [T11] Diode Nonlinear DC & PDN Z(f)   : %s (Vd=%.4f V, Zmax=%.4f Ohm)\n",
               ok ? "PASS" : "FAIL", dc.node_v[2], prr.ac.z_pdn_max_ohm);
        if (ok) ++passed; else ++failed;
    }

    /* T12: MNA 800V SiC Half-Bridge Switching + PVT Corner SOA */
    {
        ele_ckt_netlist_t hb;
        memset(&hb, 0, sizeof(hb));
        hb.n_nodes = 6;
        snprintf(hb.node_names[0], 32, "0"); snprintf(hb.node_names[1], 32, "vdc");
        snprintf(hb.node_names[2], 32, "vbus"); snprintf(hb.node_names[3], 32, "sw");
        snprintf(hb.node_names[4], 32, "gate"); snprintf(hb.node_names[5], 32, "snub");
        hb.n_elems = 10;
        hb.elems[0] = (ele_ckt_elem_t){.name="Vdc", .type=ELE_CKT_V, .n1=1, .n2=0, .dc_val=800.0, .branch_idx=0};
        hb.elems[1] = (ele_ckt_elem_t){.name="R_bus", .type=ELE_CKT_R, .n1=1, .n2=2, .val=0.08, .tol_frac=0.05, .branch_idx=-1};
        hb.elems[2] = (ele_ckt_elem_t){.name="L_bus", .type=ELE_CKT_L, .n1=2, .n2=3, .val=15e-9, .tol_frac=0.10, .branch_idx=1};
        hb.elems[3] = (ele_ckt_elem_t){.name="R_load", .type=ELE_CKT_R, .n1=2, .n2=3, .val=16.0, .branch_idx=-1};
        hb.elems[4] = (ele_ckt_elem_t){.name="C_oss", .type=ELE_CKT_C, .n1=3, .n2=0, .val=1.2e-9, .tol_frac=0.10, .branch_idx=-1};
        hb.elems[5] = (ele_ckt_elem_t){.name="R_snub", .type=ELE_CKT_R, .n1=3, .n2=5, .val=4.7, .tol_frac=0.05, .branch_idx=-1};
        hb.elems[6] = (ele_ckt_elem_t){.name="C_snub", .type=ELE_CKT_C, .n1=5, .n2=0, .val=4.7e-9, .tol_frac=0.10, .branch_idx=-1};
        hb.elems[7] = (ele_ckt_elem_t){.name="Vgate", .type=ELE_CKT_V, .n1=4, .n2=0, .src_kind=ELE_SRC_PULSE,
            .dc_val=0.0, .p_v1=0.0, .p_v2=15.0, .p_td=20e-9, .p_tr=8e-9, .p_tf=8e-9, .p_pw=80e-9, .p_per=200e-9, .branch_idx=2};
        hb.elems[8] = (ele_ckt_elem_t){.name="M_ls", .type=ELE_CKT_M, .n1=3, .n2=0, .n3=4, .m_vth=3.5, .m_ron=12.0, .m_roff=1e6, .branch_idx=-1};
        hb.elems[9] = (ele_ckt_elem_t){.name="D_fw", .type=ELE_CKT_D, .n1=0, .n2=3, .d_is=1e-11, .d_n=1.2, .d_rs=0.05, .branch_idx=-1};
        hb.has_tran = 1; hb.tran_tstep = 1e-9; hb.tran_tstop = 220e-9; hb.tran_probe_node = 3;
        hb.has_soa = 1; hb.soa_node = 3; hb.soa_vmin = -20.0; hb.soa_vmax = 1150.0;
        ele_ckt_run_res_t rr;
        int ok = (ele_ckt_run_all(&hb, 1, &rr) == 0 && rr.n_corners == 5 &&
                  rr.soa_pass == 1 && rr.wc_vmax >= 800.0 && rr.wc_vmax < 1150.0);
        printf("  [T12] 800V SiC Half-Bridge + Corners  : %s (Vds_wc=[%.1f, %.1f] V)\n",
               ok ? "PASS" : "FAIL", rr.wc_vmin, rr.wc_vmax);
        if (ok) ++passed; else ++failed;
    }

    /* T13: Coupled RLGC SI & SEARU Eye Search */
    {
        ele_ckt_netlist_t si;
        memset(&si, 0, sizeof(si));
        snprintf(si.node_names[0], 32, "0");
        snprintf(si.node_names[1], 32, "vd");
        snprintf(si.node_names[2], 32, "ad");
        int vn[9], an[9];
        vn[0] = 3; snprintf(si.node_names[3], 32, "vin");
        an[0] = 4; snprintf(si.node_names[4], 32, "ain");
        int nidx = 5;
        for (int s = 1; s < 8; ++s) {
            vn[s] = nidx; snprintf(si.node_names[nidx], 32, "vm%d", s); ++nidx;
            an[s] = nidx; snprintf(si.node_names[nidx], 32, "am%d", s); ++nidx;
        }
        vn[8] = nidx; snprintf(si.node_names[nidx], 32, "vout"); ++nidx;
        an[8] = nidx; snprintf(si.node_names[nidx], 32, "aout"); ++nidx;
        si.n_nodes = nidx;

        int e = 0, br = 2;
        si.elems[e++] = (ele_ckt_elem_t){.name="Vvic", .type=ELE_CKT_V, .n1=1, .n2=0, .branch_idx=0};
        si.elems[e++] = (ele_ckt_elem_t){.name="Vagg", .type=ELE_CKT_V, .n1=2, .n2=0, .branch_idx=1};
        si.elems[e++] = (ele_ckt_elem_t){.name="Rsv", .type=ELE_CKT_R, .n1=1, .n2=vn[0], .val=50.0, .branch_idx=-1};
        si.elems[e++] = (ele_ckt_elem_t){.name="Rsa", .type=ELE_CKT_R, .n1=2, .n2=an[0], .val=50.0, .branch_idx=-1};
        si.elems[e++] = (ele_ckt_elem_t){.name="Rlv", .type=ELE_CKT_R, .n1=vn[8], .n2=0, .val=50.0, .branch_idx=-1};
        si.elems[e++] = (ele_ckt_elem_t){.name="Rla", .type=ELE_CKT_R, .n1=an[8], .n2=0, .val=50.0, .branch_idx=-1};
        for (int s = 0; s < 8; ++s) {
            int iv = e; si.elems[e++] = (ele_ckt_elem_t){.name="Lv", .type=ELE_CKT_L, .n1=vn[s], .n2=vn[s+1], .val=3.5e-9, .branch_idx=br++};
            int ia = e; si.elems[e++] = (ele_ckt_elem_t){.name="La", .type=ELE_CKT_L, .n1=an[s], .n2=an[s+1], .val=3.5e-9, .branch_idx=br++};
            si.elems[e++] = (ele_ckt_elem_t){.name="K", .type=ELE_CKT_K, .val=0.12, .l1_idx=iv, .l2_idx=ia, .branch_idx=-1};
            si.elems[e++] = (ele_ckt_elem_t){.name="Cv", .type=ELE_CKT_C, .n1=vn[s+1], .n2=0, .val=1.4e-12, .branch_idx=-1};
            si.elems[e++] = (ele_ckt_elem_t){.name="Ca", .type=ELE_CKT_C, .n1=an[s+1], .n2=0, .val=1.4e-12, .branch_idx=-1};
            si.elems[e++] = (ele_ckt_elem_t){.name="Cm", .type=ELE_CKT_C, .n1=vn[s+1], .n2=an[s+1], .val=0.20e-12, .branch_idx=-1};
        }
        si.n_elems = e;
        ele_ckt_eye_res_t er;
        int ok = (ele_ckt_si_eye(&si, 2.5, &er) == 0 && er.eye_pass == 1 &&
                  fabs(er.eye_searu_v - er.eye_pda_v) < 0.02);
        printf("  [T13] Coupled RLGC SI & SEARU Eye     : %s (PDA=%.3f V, SEARU=%.3f V)\n",
               ok ? "PASS" : "FAIL", er.eye_pda_v, er.eye_searu_v);
        if (ok) ++passed; else ++failed;
    }

    /* T14: Saturated IPMSM MTPA/FW + FOC + Integer Cert */
    {
        ele_motor_spec_t ms = {8, 0.018, 0.00018, 0.00042, 0.145, 1.2e-6, 2.8e-6, 0.8e-6,
                               -0.0012, 400.0, 800.0, 0.045, 0.015, 0.0012, 85.0, 14000.0, "internal_motor", ""};
        ele_sha256_bytes((const uint8_t *)"motor", 5U, ms.input_sha256);
        ele_motor_op_res_t op_lo, op_hi;
        ele_motor_sweep_res_t sw;
        int ok = (ele_motor_solve_op(&ms, 3000.0, 260.0, &op_lo) == 0 &&
                  ele_motor_solve_op(&ms, 10000.0, 0.0, &op_hi) == 0 &&
                  ele_motor_sweep_and_foc(&ms, 260.0, &sw) == 0 &&
                  op_lo.in_current_circle && op_lo.in_voltage_ellipse &&
                  op_hi.in_current_circle && op_hi.in_voltage_ellipse &&
                  op_hi.id_a < op_lo.id_a && sw.foc_converged);
        printf("  [T14] IPMSM Saturated MTPA/FW & FOC   : %s (T@3k=%.1f Nm, Id@10k=%.1f A, eff=%.1f%%)\n",
               ok ? "PASS" : "FAIL", op_lo.torque_nm, op_hi.id_a, 100.0 * op_lo.efficiency);
        if (ok) ++passed; else ++failed;
    }

    /* T15: IEEE 3-Bus & WSCC 9-Bus AC-PF, N-1 & Certified AC-OPF */
    {
        ele_grid_t g3, g9;
        build_test_ieee3(&g3);
        build_test_ieee9(&g9);
        ele_grid_pf_res_t pf3, pf9;
        ele_grid_n1_res_t n1_9;
        ele_grid_opf_res_t opf3, opf9;
        int ok3 = (ele_grid_acpf(&g3, -1, &pf3) == 0 && pf3.max_mismatch_pu < 1e-11 &&
                   ele_grid_acopf(&g3, &opf3) == 0 && opf3.c_lb_cert <= opf3.c_ub_primal && opf3.gap_pct < 1.0);
        int ok9 = (ele_grid_acpf(&g9, -1, &pf9) == 0 && pf9.max_mismatch_pu < 1e-11 &&
                   ele_grid_n1(&g9, &n1_9) == 0 && n1_9.n_converged == 6 &&
                   ele_grid_acopf(&g9, &opf9) == 0 && opf9.c_lb_cert <= opf9.c_ub_primal && opf9.gap_pct < 5.0);
        int ok = ok3 && ok9;
        printf("  [T15] IEEE 3/9 AC-PF, N-1 & AC-OPF    : %s (3b=[%.1f,%.1f] %.3f%%, 9b=[%.1f,%.1f] %.3f%%)\n",
               ok ? "PASS" : "FAIL", opf3.c_lb_cert, opf3.c_ub_primal, opf3.gap_pct,
               opf9.c_lb_cert, opf9.c_ub_primal, opf9.gap_pct);
        if (ok) ++passed; else ++failed;
    }

    /* T16: WSCC 9-Bus Multi-Machine / GFM EMT & Lyapunov CCT */
    {
        ele_grid_t g9;
        build_test_ieee9(&g9);
        ele_grid_emt_res_t emt;
        int ok = (ele_grid_emt(&g9, 7, 0.12, -1, &emt) == 0 &&
                  emt.stable == 1 && emt.v_post < emt.v_crit &&
                  emt.cct_sec > 0.12 && emt.max_gfm_i_pu <= 1.201);
        printf("  [T16] WSCC 9-Bus EMT & PEBS CCT       : %s (V=%.4f<%.4f, CCT=%.3fs, GFM_I=%.3f pu)\n",
               ok ? "PASS" : "FAIL", emt.v_post, emt.v_crit, emt.cct_sec, emt.max_gfm_i_pu);
        if (ok) ++passed; else ++failed;
    }

    printf("  SELFTEST SUMMARY: %d / %d PASSED\n", passed, passed + failed);
    if (json_out != NULL) {
        FILE *fp = fopen(json_out, "w");
        if (fp != NULL) {
            fprintf(fp, "{\n  \"passed\": %d,\n  \"failed\": %d,\n  \"total\": %d\n}\n",
                    passed, failed, passed + failed);
            fclose(fp);
        }
    }
    return (failed == 0) ? 0 : 1;
}

static void replace_sp_value(const char *line_in, char *line_out, size_t cap,
                             const char *elem_prefix, double new_val) {
    char copy[1024];
    snprintf(copy, sizeof(copy), "%s", line_in);
    char t0[64] = {0}, t1[64] = {0}, t2[64] = {0}, t3[64] = {0}, rest[512] = {0};
    int n = sscanf(copy, " %63s %63s %63s %63s %511[^\n]", t0, t1, t2, t3, rest);
    if (n >= 4 && cli_strcasecmp(t0, elem_prefix) == 0) {
        if (n == 5) snprintf(line_out, cap, "%s %s %s %.9e %s\n", t0, t1, t2, new_val, rest);
        else snprintf(line_out, cap, "%s %s %s %.9e\n", t0, t1, t2, new_val);
    } else {
        snprintf(line_out, cap, "%s", line_in);
    }
}

static int run_field_to_circuit(const char *peec_path, const char *sp_path,
                                  const char *motor_path, const char *out_dir,
                                  const char *json_out) {
    if (peec_path == NULL || sp_path == NULL || motor_path == NULL) return -1;
    char s2p_path[512], em_cert_path[512], ckt_sp_path[512], ckt_cert_path[512];
    const char *dir = (out_dir != NULL) ? out_dir : "build";
    snprintf(s2p_path, sizeof(s2p_path), "%s/field_busbar.s2p", dir);
    snprintf(em_cert_path, sizeof(em_cert_path), "%s/field_busbar.elecert.json", dir);
    snprintf(ckt_sp_path, sizeof(ckt_sp_path), "%s/coupled_sic_halfbridge.sp", dir);
    snprintf(ckt_cert_path, sizeof(ckt_cert_path), "%s/coupled_sic_soa.elecert.json", dir);

    ele_peec_spec_t pspec;
    ele_em_peec_res_t pres;
    if (ele_em_peec_load(peec_path, &pspec) != 0 ||
        ele_em_peec_extract(&pspec, s2p_path, &pres) != 0 ||
        !pres.passivity.passive_certified) {
        fprintf(stderr, "ERROR: PEEC extraction or passivity certification failed!\n");
        return -1;
    }
    ele_cert_write_json(em_cert_path, &pres.passivity.cert);

    /* Rewrite SPICE netlist injecting extracted L_loop, R_dc/ac, C_bus */
    FILE *fi = fopen(sp_path, "r");
    FILE *fo = fopen(ckt_sp_path, "w");
    if (fi == NULL || fo == NULL) {
        if (fi != NULL) fclose(fi);
        if (fo != NULL) fclose(fo);
        return -1;
    }
    char line[1024], outl[1024];
    while (fgets(line, (int)sizeof(line), fi) != NULL) {
        if (cli_strncasecmp(line, "L_bus", 5U) == 0 || cli_strncasecmp(line, "Lbus", 4U) == 0) {
            char nm[64] = {0};
            sscanf(line, " %63s", nm);
            replace_sp_value(line, outl, sizeof(outl), nm, pres.l_loop_h);
            fputs(outl, fo);
        } else if (cli_strncasecmp(line, "R_bus", 5U) == 0 || cli_strncasecmp(line, "Rbus", 4U) == 0) {
            char nm[64] = {0};
            sscanf(line, " %63s", nm);
            replace_sp_value(line, outl, sizeof(outl), nm, fmax(0.01, pres.r_ac_fmax_ohm));
            fputs(outl, fo);
        } else {
            fputs(line, fo);
        }
    }
    fclose(fi);
    fclose(fo);

    ele_ckt_netlist_t nl;
    ele_ckt_run_res_t ckt_res;
    if (ele_ckt_load(ckt_sp_path, &nl) != 0 || ele_ckt_run_all(&nl, 1, &ckt_res) != 0) {
        fprintf(stderr, "ERROR: Coupled SiC half-bridge MNA failed!\n");
        return -1;
    }
    ele_cert_write_json(ckt_cert_path, &ckt_res.cert);

    ele_motor_spec_t mspec;
    ele_motor_op_res_t mop;
    if (ele_motor_load(motor_path, &mspec) != 0) return -1;
    double overshoot = fmax(0.0, ckt_res.wc_vmax - 800.0);
    double effective_vdc = mspec.v_dc_v - 0.05 * overshoot;
    if (effective_vdc < 600.0) effective_vdc = 600.0;
    mspec.v_dc_v = effective_vdc;
    if (ele_motor_solve_op(&mspec, 4000.0, 260.0, &mop) != 0) return -1;

    double didt_fault_a_us = (800.0 / pres.l_loop_h) * 1e-6;
    int ok = (pres.passivity.passive_certified && ckt_res.soa_pass &&
              mop.in_current_circle && mop.in_voltage_ellipse);

    printf("  [FIELD-TO-CIRCUIT] PEEC L_loop=%.2f nH, C_bus=%.1f pF (Passive=%d)\n"
           "                     SiC Vds_wc=[%.1f, %.1f] V (SOA=%s) | Short dI/dt=%.1f kA/us\n"
           "                     IPMSM@4krpm T=%.1f Nm, V=%.1f/%.1f V, eff=%.2f%%\n",
           pres.l_loop_h * 1e9, pres.c_bus_f * 1e12, pres.passivity.passive_certified,
           ckt_res.wc_vmin, ckt_res.wc_vmax, ckt_res.soa_pass ? "PASS" : "FAIL",
           didt_fault_a_us * 1e-3, mop.torque_nm, mop.v_mag_v, mop.v_max_v, 100.0 * mop.efficiency);

    if (json_out != NULL) {
        FILE *jf = fopen(json_out, "w");
        if (jf != NULL) {
            fprintf(jf,
                    "{\n"
                    "  \"ok\": %d,\n"
                    "  \"l_loop_h\": %.9e,\n"
                    "  \"c_bus_f\": %.9e,\n"
                    "  \"r_ac_fmax_ohm\": %.9e,\n"
                    "  \"passive_certified\": %d,\n"
                    "  \"wc_vmin_v\": %.6f,\n"
                    "  \"wc_vmax_v\": %.6f,\n"
                    "  \"soa_pass\": %d,\n"
                    "  \"short_didt_a_per_us\": %.3f,\n"
                    "  \"motor_torque_nm\": %.3f,\n"
                    "  \"motor_eff\": %.6f,\n"
                    "  \"motor_in_ellipse\": %d\n"
                    "}\n",
                    ok, pres.l_loop_h, pres.c_bus_f, pres.r_ac_fmax_ohm,
                    pres.passivity.passive_certified, ckt_res.wc_vmin, ckt_res.wc_vmax,
                    ckt_res.soa_pass, didt_fault_a_us, mop.torque_nm, mop.efficiency,
                    mop.in_voltage_ellipse);
            fclose(jf);
        }
    }
    return ok ? 0 : 1;
}

typedef struct {
    double t_s[2048];
    double v_rail_mv[2048];
    double t_die_c[2048];
    int load_on[2048];
    double u[2048];
    double u_best[2048];
} lab_therm_buf_t;

static double eval_spice_trace_at(const ele_ckt_tran_res_t *tr, double t, int *cursor) {
    int tl = tr->trace_len;
    if (tl <= 0) return 0.0;
    if (t <= tr->time_s[0]) return tr->trace_v[0];
    int j = (cursor != NULL) ? *cursor : 0;
    while (j + 1 < tl && tr->time_s[j + 1] < t) ++j;
    if (cursor != NULL) *cursor = j;
    if (j + 1 >= tl) return tr->trace_v[tl - 1];
    double t0 = tr->time_s[j], t1 = tr->time_s[j + 1];
    double w = (t1 > t0 + 1e-12) ? ((t - t0) / (t1 - t0)) : 0.0;
    return tr->trace_v[j] + w * (tr->trace_v[j + 1] - tr->trace_v[j]);
}

static double eval_cauer_sse(const lab_therm_buf_t *tb, int n, double t_amb_c, double tau,
                             double *dt_ss_out, double *u_work) {
    u_work[0] = 0.0;
    for (int k = 0; k + 1 < n; ++k) {
        double dt = tb->t_s[k + 1] - tb->t_s[k];
        if (dt < 1e-6) dt = 0.1;
        double a = exp(-dt / tau);
        u_work[k + 1] = u_work[k] * a + (double)tb->load_on[k] * (1.0 - a);
    }
    double num = 0.0, den = 0.0;
    for (int k = 0; k < n; ++k) {
        double y = tb->t_die_c[k] - t_amb_c;
        num += u_work[k] * y;
        den += u_work[k] * u_work[k];
    }
    double dt_ss = (den > 1e-12) ? (num / den) : 0.0;
    if (dt_ss < 0.01) dt_ss = 0.01;
    double sse = 0.0;
    for (int k = 0; k < n; ++k) {
        double err = (tb->t_die_c[k] - t_amb_c) - dt_ss * u_work[k];
        sse += err * err;
    }
    if (dt_ss_out != NULL) *dt_ss_out = dt_ss;
    return sse;
}

static int cmd_lab_thermal(const char *csv_path, const char *sp_path,
                           const char *cert_path, const char *json_path) {
    if (csv_path == NULL) return 1;
    FILE *fp = fopen(csv_path, "r");
    if (fp == NULL) {
        fprintf(stderr, "ERROR: Cannot open thermal CSV: %s\n", csv_path);
        return 1;
    }
    lab_therm_buf_t *tb = (lab_therm_buf_t *)calloc(1U, sizeof(lab_therm_buf_t));
    ele_ckt_netlist_t *nl = (ele_ckt_netlist_t *)calloc(1U, sizeof(ele_ckt_netlist_t));
    ele_ckt_run_res_t *res = (ele_ckt_run_res_t *)calloc(1U, sizeof(ele_ckt_run_res_t));
    if (tb == NULL || nl == NULL || res == NULL) {
        fclose(fp); free(tb); free(nl); free(res);
        return 1;
    }

    int n = 0;
    char line[512];
    while (fgets(line, (int)sizeof(line), fp) != NULL && n < 2048) {
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#' || *p == '\n' || *p == '\r') continue;
        int idx_v = 0, lo_v = 0;
        double ts_v = 0.0, vr_v = 0.0, td_v = 0.0;
        if (sscanf(p, "%d,%lf,%lf,%lf,%d", &idx_v, &ts_v, &vr_v, &td_v, &lo_v) == 5) {
            tb->t_s[n] = ts_v;
            tb->v_rail_mv[n] = vr_v;
            tb->t_die_c[n] = td_v;
            tb->load_on[n] = lo_v ? 1 : 0;
            ++n;
        }
    }
    fclose(fp);
    if (n < 20) {
        fprintf(stderr, "ERROR: Insufficient thermal samples (%d < 20) in %s\n", n, csv_path);
        free(tb); free(nl); free(res);
        return 1;
    }

    int first_on = -1, last_on = -1;
    for (int k = 0; k < n; ++k) {
        if (tb->load_on[k] != 0) {
            if (first_on < 0) first_on = k;
            last_on = k;
        }
    }
    if (first_on < 0) {
        fprintf(stderr, "ERROR: No load step (load_on=1) found in %s\n", csv_path);
        free(tb); free(nl); free(res);
        return 1;
    }

    double sum_tamb = 0.0, sum_vidle = 0.0, sum_vload = 0.0;
    int cnt_idle = 0, cnt_load = 0;
    double t_peak_c = tb->t_die_c[0];
    for (int k = 0; k < n; ++k) {
        if (tb->t_die_c[k] > t_peak_c) t_peak_c = tb->t_die_c[k];
        if (k < first_on) {
            sum_tamb += tb->t_die_c[k];
            sum_vidle += tb->v_rail_mv[k];
            ++cnt_idle;
        }
        if (tb->load_on[k] != 0) {
            sum_vload += tb->v_rail_mv[k];
            ++cnt_load;
        }
    }
    double t_amb_c = (cnt_idle > 0) ? (sum_tamb / (double)cnt_idle) : tb->t_die_c[0];
    double v_idle_mv = (cnt_idle > 0) ? (sum_vidle / (double)cnt_idle) : tb->v_rail_mv[0];
    double v_load_mv = (cnt_load > 0) ? (sum_vload / (double)cnt_load) : tb->v_rail_mv[first_on];
    double v_droop_mv = v_idle_mv - v_load_mv;
    double delta_t_c = t_peak_c - t_amb_c;

    double dt_avg = (tb->t_s[n - 1] - tb->t_s[0]) / (double)(n - 1);
    if (dt_avg < 1e-4) dt_avg = 0.1;
    double t_load_start = tb->t_s[first_on];
    double t_load_end = (last_on + 1 < n) ? tb->t_s[last_on + 1] : (tb->t_s[last_on] + dt_avg);

    double best_tau = 5.0, best_sse = 1e30, best_dt_ss = 1.0;
    for (int i = 0; i < 200; ++i) {
        double tau = 0.5 * pow(60.0 / 0.5, (double)i / 199.0);
        double dt_ss = 0.0;
        double sse = eval_cauer_sse(tb, n, t_amb_c, tau, &dt_ss, tb->u);
        if (sse < best_sse) {
            best_sse = sse;
            best_tau = tau;
            best_dt_ss = dt_ss;
        }
    }
    double g_lo = fmax(0.5, best_tau * 0.7);
    double g_hi = fmin(60.0, best_tau * 1.3);
    const double phi = 0.6180339887498949;
    double x1 = g_hi - phi * (g_hi - g_lo);
    double x2 = g_lo + phi * (g_hi - g_lo);
    double f1 = eval_cauer_sse(tb, n, t_amb_c, x1, NULL, tb->u);
    double f2 = eval_cauer_sse(tb, n, t_amb_c, x2, NULL, tb->u);
    for (int it = 0; it < 30; ++it) {
        if (f1 < f2) {
            g_hi = x2; x2 = x1; f2 = f1;
            x1 = g_hi - phi * (g_hi - g_lo);
            f1 = eval_cauer_sse(tb, n, t_amb_c, x1, NULL, tb->u);
        } else {
            g_lo = x1; x1 = x2; f1 = f2;
            x2 = g_lo + phi * (g_hi - g_lo);
            f2 = eval_cauer_sse(tb, n, t_amb_c, x2, NULL, tb->u);
        }
    }
    best_tau = 0.5 * (g_lo + g_hi);
    best_sse = eval_cauer_sse(tb, n, t_amb_c, best_tau, &best_dt_ss, tb->u_best);
    double analytic_rmse_c = sqrt(best_sse / (double)n);

    const double p_step = 0.65;
    double r_th = fmax(0.1, best_dt_ss / p_step);
    double c_th = best_tau / r_th;
    double pw = fmax(0.1, t_load_end - t_load_start - 0.002);
    double t_end = tb->t_s[n - 1] + 0.05;

    const char *use_sp = (sp_path != NULL) ? sp_path : ".ele_lab_thermal_tmp.sp";
    FILE *sfp = fopen(use_sp, "w");
    if (sfp == NULL) {
        fprintf(stderr, "ERROR: Cannot write thermal SPICE deck: %s\n", use_sp);
        free(tb); free(nl); free(res);
        return 1;
    }
    fprintf(sfp,
            "* ELE v%s Cardputer-Adv Cauer 1-Pole Thermal-Electrical Analog\n"
            "I_heat 0 v_dt DC 0 PULSE(0 %.6f %.6f 0.001 0.001 %.6f 1000.0)\n"
            "R_th v_dt 0 %.6f\n"
            "C_th v_dt 0 %.6f\n"
            ".TRAN 0.05 %.6f PROBE=v_dt\n"
            ".END\n",
            ELE_VERSION_STRING, p_step, t_load_start, pw, r_th, c_th, t_end);
    fclose(sfp);

    int load_rc = ele_ckt_load(use_sp, nl);
    int run_rc = (load_rc == 0) ? ele_ckt_run_all(nl, 0, res) : -1;
    if (sp_path == NULL) remove(use_sp);
    if (load_rc != 0 || run_rc != 0 || !res->tran.converged || res->tran.trace_len < 2) {
        fprintf(stderr, "ERROR: Thermal Cauer SPICE transient solve failed!\n");
        free(tb); free(nl); free(res);
        return 1;
    }

    double sse_spice = 0.0, spice_vs_analytic_c = 0.0;
    int j = 0;
    for (int k = 0; k < n; ++k) {
        double t = tb->t_s[k];
        double v_sp = eval_spice_trace_at(&res->tran, t, &j);
        double err_m = (tb->t_die_c[k] - t_amb_c) - v_sp;
        sse_spice += err_m * err_m;
        double err_sa = fabs(v_sp - best_dt_ss * tb->u_best[k]);
        if (err_sa > spice_vs_analytic_c) spice_vs_analytic_c = err_sa;
    }
    double spice_rmse_c = sqrt(sse_spice / (double)n);

    ele_cert_record_t cert;
    memset(&cert, 0, sizeof(cert));
    snprintf(cert.pillar, sizeof(cert.pillar), "lab");
    snprintf(cert.kind, sizeof(cert.kind), "cardputer_thermal");
    snprintf(cert.input_path, sizeof(cert.input_path), "%s", csv_path);
    if (ele_sha256_file(csv_path, cert.input_sha256) != 0) {
        free(tb); free(nl); free(res);
        return 1;
    }
    int pass = (res->tran.converged && spice_rmse_c <= 1.50 && delta_t_c > 0.0);
    snprintf(cert.status, sizeof(cert.status), "%s", pass ? "CERTIFIED" : "VETOED");
    cert.has_bracket = 1;
    cert.bracket_scale = INT64_C(1000000);
    cert.lb_int = 0;
    cert.ub_int = (int64_t)llround(spice_rmse_c * 1000000.0);
    cert.limit_lo_int = 0;
    cert.limit_hi_int = INT64_C(1500000);
    cert.lb_val = 0.0;
    cert.ub_val = spice_rmse_c;

    if (cert_path != NULL && ele_cert_write_json(cert_path, &cert) != 0) {
        free(tb); free(nl); free(res);
        return 1;
    }
    if (json_path != NULL) {
        FILE *jf = fopen(json_path, "w");
        if (jf != NULL) {
            fprintf(jf,
                    "{\n"
                    "  \"n_samples\": %d,\n"
                    "  \"t_amb_c\": %.6f,\n"
                    "  \"t_peak_c\": %.6f,\n"
                    "  \"delta_t_c\": %.6f,\n"
                    "  \"v_idle_mv\": %.3f,\n"
                    "  \"v_load_mv\": %.3f,\n"
                    "  \"v_droop_mv\": %.3f,\n"
                    "  \"r_th_k_per_w\": %.6f,\n"
                    "  \"c_th_j_per_k\": %.6f,\n"
                    "  \"tau_th_s\": %.6f,\n"
                    "  \"analytic_rmse_c\": %.6f,\n"
                    "  \"spice_rmse_c\": %.6f,\n"
                    "  \"spice_vs_analytic_c\": %.6f,\n"
                    "  \"status\": \"%s\"\n"
                    "}\n",
                    n, t_amb_c, t_peak_c, delta_t_c, v_idle_mv, v_load_mv, v_droop_mv,
                    r_th, c_th, best_tau, analytic_rmse_c, spice_rmse_c,
                    spice_vs_analytic_c, cert.status);
            fclose(jf);
        }
    }
    printf("[LAB THERMAL] %s -> N=%d, dT=%.2f C (Rth=%.2f K/W, tau=%.2f s, droop=%.1f mV), SPICE RMSE=%.4f C (%s)\n",
           csv_path, n, delta_t_c, r_th, best_tau, v_droop_mv, spice_rmse_c, cert.status);
    free(tb); free(nl); free(res);
    return pass ? 0 : 1;
}

typedef struct {
    double t_s[2048];
    double v_rail_mv[2048];
    double t_die_c[2048];
    int mode_id[2048];
    int cpu_mhz[2048];
} lab_pwr_buf_t;

static double eval_pdn_analytic_sse(const lab_pwr_buf_t *pb, int n, double v_oc_v, double r_pdn,
                                    double tau, double i_base, const double dI_step[3],
                                    double t1s, double t1e, double t2s, double t2e, double t3s, double t3e) {
    double vc = v_oc_v - r_pdn * i_base;
    double sse = 0.0;
    for (int k = 0; k < n; ++k) {
        double t = pb->t_s[k];
        if (k > 0) {
            double dt = t - pb->t_s[k - 1];
            if (dt < 1e-6) dt = 0.05;
            double i_step = i_base;
            if (t >= t1s && t <= t1e) i_step += dI_step[0];
            if (t >= t2s && t <= t2e) i_step += dI_step[1];
            if (t >= t3s && t <= t3e) i_step += dI_step[2];
            double v_inf = v_oc_v - r_pdn * i_step;
            double a = exp(-dt / tau);
            vc = v_inf + (vc - v_inf) * a;
        }
        double err_mv = pb->v_rail_mv[k] - vc * 1000.0;
        sse += err_mv * err_mv;
    }
    return sse;
}

static int cmd_lab_power(const char *csv_path, const char *sp_path,
                         const char *cert_path, const char *json_path) {
    if (csv_path == NULL) return 1;
    FILE *fp = fopen(csv_path, "r");
    if (fp == NULL) {
        fprintf(stderr, "ERROR: Cannot open power-mode CSV: %s\n", csv_path);
        return 1;
    }
    lab_pwr_buf_t *pb = (lab_pwr_buf_t *)calloc(1U, sizeof(lab_pwr_buf_t));
    ele_ckt_netlist_t *nl = (ele_ckt_netlist_t *)calloc(1U, sizeof(ele_ckt_netlist_t));
    ele_ckt_run_res_t *res = (ele_ckt_run_res_t *)calloc(1U, sizeof(ele_ckt_run_res_t));
    if (!pb || !nl || !res) {
        fclose(fp); free(pb); free(nl); free(res);
        return 1;
    }

    int n = 0;
    char line[512];
    while (fgets(line, (int)sizeof(line), fp) != NULL && n < 2048) {
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#' || *p == '\n' || *p == '\r') continue;
        int idx_v = 0, mhz_v = 0, v_mv = 0;
        double t_ms = 0.0, t_c = 0.0;
        char mode_str[32] = {0};
        if (sscanf(p, "%d,%lf,%31[^,],%d,%d,%lf", &idx_v, &t_ms, mode_str, &mhz_v, &v_mv, &t_c) == 6) {
            int mid = 0;
            if (strcmp(mode_str, "80M_IDLE") == 0) mid = 0;
            else if (strcmp(mode_str, "240M_IDLE") == 0) mid = 1;
            else if (strcmp(mode_str, "240M_FPU") == 0) mid = 2;
            else if (strcmp(mode_str, "240M_WIFI") == 0) mid = 3;
            else if (strcmp(mode_str, "80M_RECOV") == 0) mid = 4;
            else continue;
            pb->t_s[n] = t_ms * 1e-3;
            pb->mode_id[n] = mid;
            pb->cpu_mhz[n] = mhz_v;
            pb->v_rail_mv[n] = (double)v_mv;
            pb->t_die_c[n] = t_c;
            ++n;
        }
    }
    fclose(fp);
    if (n < 50) {
        fprintf(stderr, "ERROR: Insufficient power-mode samples (%d < 50) in %s\n", n, csv_path);
        free(pb); free(nl); free(res);
        return 1;
    }

    double sum_v[5] = {0}, sum_t[5] = {0};
    int cnt_m[5] = {0};
    double t_min_m[5] = {1e9, 1e9, 1e9, 1e9, 1e9};
    double t_max_m[5] = {0, 0, 0, 0, 0};
    double min_v_mv = pb->v_rail_mv[0], max_v_mv = pb->v_rail_mv[0];
    double t_die_peak = pb->t_die_c[0];

    for (int k = 0; k < n; ++k) {
        int m = pb->mode_id[k];
        sum_v[m] += pb->v_rail_mv[k];
        sum_t[m] += pb->t_die_c[k];
        ++cnt_m[m];
        if (pb->t_s[k] < t_min_m[m]) t_min_m[m] = pb->t_s[k];
        if (pb->t_s[k] > t_max_m[m]) t_max_m[m] = pb->t_s[k];
        if (pb->v_rail_mv[k] < min_v_mv) min_v_mv = pb->v_rail_mv[k];
        if (pb->v_rail_mv[k] > max_v_mv) max_v_mv = pb->v_rail_mv[k];
        if (pb->t_die_c[k] > t_die_peak) t_die_peak = pb->t_die_c[k];
    }
    if (cnt_m[0] == 0 || cnt_m[1] == 0 || cnt_m[2] == 0 || cnt_m[3] == 0 || cnt_m[4] == 0) {
        fprintf(stderr, "ERROR: Missing one of the 5 power modes in %s\n", csv_path);
        free(pb); free(nl); free(res);
        return 1;
    }

    double v_m[5], t_m[5];
    for (int m = 0; m < 5; ++m) {
        v_m[m] = sum_v[m] / (double)cnt_m[m];
        t_m[m] = sum_t[m] / (double)cnt_m[m];
    }
    double max_droop_mv = 0.0;
    for (int m = 1; m <= 3; ++m) {
        double d = v_m[0] - v_m[m];
        if (d > max_droop_mv) max_droop_mv = d;
    }
    double delta_t_c = t_die_peak - t_m[0];

    const double i_base = 0.04;
    const double i_act_nom = 0.25;
    double dv_peak_v = max_droop_mv * 1e-3;
    double r_pdn = (dv_peak_v > 1e-4) ? fmax(0.02, dv_peak_v / i_act_nom) : 0.02;
    double v_oc_v = (v_m[0] * 1e-3) + r_pdn * i_base;

    double dI_step[3];
    for (int m = 0; m < 3; ++m) {
        double dvm = fmax(0.0, (v_m[0] - v_m[1 + m]) * 1e-3);
        dI_step[m] = dvm / r_pdn;
    }

    double t_start[3] = { t_min_m[1], t_min_m[2], t_min_m[3] };
    double t_end_m[3] = { t_max_m[1] + 0.02, t_max_m[2] + 0.02, t_max_m[3] + 0.02 };

    double best_tau = 0.15, best_sse = 1e30;
    for (int i = 0; i < 60; ++i) {
        double tau = 0.02 * pow(2.0 / 0.02, (double)i / 59.0);
        double sse = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, tau, i_base, dI_step,
                                           t_start[0], t_end_m[0], t_start[1], t_end_m[1],
                                           t_start[2], t_end_m[2]);
        if (sse < best_sse) { best_sse = sse; best_tau = tau; }
    }
    double g_lo = fmax(0.01, best_tau * 0.7), g_hi = fmin(2.5, best_tau * 1.3);
    const double phi = 0.6180339887498949;
    double x1 = g_hi - phi * (g_hi - g_lo), x2 = g_lo + phi * (g_hi - g_lo);
    double f1 = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, x1, i_base, dI_step,
                                      t_start[0], t_end_m[0], t_start[1], t_end_m[1], t_start[2], t_end_m[2]);
    double f2 = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, x2, i_base, dI_step,
                                      t_start[0], t_end_m[0], t_start[1], t_end_m[1], t_start[2], t_end_m[2]);
    for (int it = 0; it < 25; ++it) {
        if (f1 < f2) {
            g_hi = x2; x2 = x1; f2 = f1;
            x1 = g_hi - phi * (g_hi - g_lo);
            f1 = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, x1, i_base, dI_step,
                                       t_start[0], t_end_m[0], t_start[1], t_end_m[1], t_start[2], t_end_m[2]);
        } else {
            g_lo = x1; x1 = x2; f1 = f2;
            x2 = g_lo + phi * (g_hi - g_lo);
            f2 = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, x2, i_base, dI_step,
                                       t_start[0], t_end_m[0], t_start[1], t_end_m[1], t_start[2], t_end_m[2]);
        }
    }
    best_tau = 0.5 * (g_lo + g_hi);
    best_sse = eval_pdn_analytic_sse(pb, n, v_oc_v, r_pdn, best_tau, i_base, dI_step,
                                     t_start[0], t_end_m[0], t_start[1], t_end_m[1], t_start[2], t_end_m[2]);
    double analytic_rmse_mv = sqrt(best_sse / (double)n);
    double c_bulk = best_tau / r_pdn;
    double t_sim_end = pb->t_s[n - 1] + 0.05;

    const char *use_sp = (sp_path != NULL) ? sp_path : ".ele_lab_power_tmp.sp";
    FILE *sfp = fopen(use_sp, "w");
    if (sfp == NULL) {
        free(pb); free(nl); free(res);
        return 1;
    }
    fprintf(sfp,
            "* ELE v%s Cardputer-Adv Causal Thevenin PDN RC Model\n"
            "V_oc n_oc 0 DC %.6f\n"
            "R_pdn n_oc v_rail %.6f\n"
            "C_bulk v_rail 0 %.6f\n"
            "I_base v_rail 0 DC %.6f\n",
            ELE_VERSION_STRING, v_oc_v, r_pdn, c_bulk, i_base);
    for (int m = 0; m < 3; ++m) {
        if (dI_step[m] > 1e-6) {
            double pw = fmax(0.05, t_end_m[m] - t_start[m] - 0.004);
            fprintf(sfp,
                    "I_m%d v_rail 0 DC 0 PULSE(0 %.6f %.6f 0.002 0.002 %.6f 1000.0)\n",
                    m + 1, dI_step[m], t_start[m], pw);
        }
    }
    fprintf(sfp, ".TRAN 0.025 %.6f PROBE=v_rail\n.END\n", t_sim_end);
    fclose(sfp);

    int load_rc = ele_ckt_load(use_sp, nl);
    int run_rc = (load_rc == 0) ? ele_ckt_run_all(nl, 0, res) : -1;
    if (sp_path == NULL) remove(use_sp);
    if (load_rc != 0 || run_rc != 0 || !res->tran.converged || res->tran.trace_len < 2) {
        fprintf(stderr, "ERROR: PDN SPICE transient solve failed!\n");
        free(pb); free(nl); free(res);
        return 1;
    }

    double sse_sp = 0.0;
    int j = 0;
    for (int k = 0; k < n; ++k) {
        double v_sp_v = eval_spice_trace_at(&res->tran, pb->t_s[k], &j);
        double err_mv = pb->v_rail_mv[k] - v_sp_v * 1000.0;
        sse_sp += err_mv * err_mv;
    }
    double spice_rmse_mv = sqrt(sse_sp / (double)n);

    ele_cert_record_t cert;
    memset(&cert, 0, sizeof(cert));
    snprintf(cert.pillar, sizeof(cert.pillar), "lab");
    snprintf(cert.kind, sizeof(cert.kind), "cardputer_power_pdn");
    snprintf(cert.input_path, sizeof(cert.input_path), "%s", csv_path);
    if (ele_sha256_file(csv_path, cert.input_sha256) != 0) {
        free(pb); free(nl); free(res);
        return 1;
    }
    int pass = (res->tran.converged && spice_rmse_mv <= 30.0 && min_v_mv >= 3300.0);
    snprintf(cert.status, sizeof(cert.status), "%s", pass ? "CERTIFIED" : "VETOED");
    cert.has_bracket = 1;
    cert.bracket_scale = INT64_C(1000000);
    cert.lb_int = 0;
    cert.ub_int = (int64_t)llround(spice_rmse_mv * 1000000.0);
    cert.limit_lo_int = 0;
    cert.limit_hi_int = INT64_C(30000000);
    cert.lb_val = 0.0;
    cert.ub_val = spice_rmse_mv;

    if (cert_path != NULL && ele_cert_write_json(cert_path, &cert) != 0) {
        free(pb); free(nl); free(res);
        return 1;
    }
    if (json_path != NULL) {
        FILE *jf = fopen(json_path, "w");
        if (jf != NULL) {
            fprintf(jf,
                    "{\n"
                    "  \"n_samples\": %d,\n"
                    "  \"v_80_idle_mv\": %.3f,\n"
                    "  \"v_240_idle_mv\": %.3f,\n"
                    "  \"v_240_fpu_mv\": %.3f,\n"
                    "  \"v_240_wifi_mv\": %.3f,\n"
                    "  \"v_80_recov_mv\": %.3f,\n"
                    "  \"t_80_idle_c\": %.3f,\n"
                    "  \"t_240_idle_c\": %.3f,\n"
                    "  \"t_240_fpu_c\": %.3f,\n"
                    "  \"t_240_wifi_c\": %.3f,\n"
                    "  \"t_80_recov_c\": %.3f,\n"
                    "  \"min_v_mv\": %.3f,\n"
                    "  \"max_v_mv\": %.3f,\n"
                    "  \"max_droop_mv\": %.3f,\n"
                    "  \"delta_t_c\": %.3f,\n"
                    "  \"v_oc_v\": %.6f,\n"
                    "  \"r_pdn_ohm\": %.6f,\n"
                    "  \"c_bulk_f\": %.6f,\n"
                    "  \"tau_pdn_s\": %.6f,\n"
                    "  \"analytic_rmse_mv\": %.6f,\n"
                    "  \"spice_rmse_mv\": %.6f,\n"
                    "  \"status\": \"%s\"\n"
                    "}\n",
                    n, v_m[0], v_m[1], v_m[2], v_m[3], v_m[4],
                    t_m[0], t_m[1], t_m[2], t_m[3], t_m[4],
                    min_v_mv, max_v_mv, max_droop_mv, delta_t_c,
                    v_oc_v, r_pdn, c_bulk, best_tau, analytic_rmse_mv, spice_rmse_mv,
                    cert.status);
            fclose(jf);
        }
    }
    printf("[LAB POWER] %s -> N=%d, Vmin=%.1f mV, droop=%.2f mV, dT=%.2f C (Voc=%.4f V, Rpdn=%.4f Ohm, C=%.2f F), SPICE RMSE=%.3f mV (%s)\n",
           csv_path, n, min_v_mv, max_droop_mv, delta_t_c, v_oc_v, r_pdn, c_bulk, spice_rmse_mv, cert.status);
    free(pb); free(nl); free(res);
    return pass ? 0 : 1;
}

typedef struct {
    int ch;
    int rssi;
    int nf;
    int len;
    int n_sc;
    int8_t b[128];
} lab_csi_frame_t;

static int cmd_lab_csi(const char *csv_path, const char *s2p_path,
                       const char *cert_path, const char *json_path) {
    if (csv_path == NULL) return 1;
    FILE *fp = fopen(csv_path, "r");
    if (fp == NULL) {
        fprintf(stderr, "ERROR: Cannot open CSI CSV: %s\n", csv_path);
        return 1;
    }
    lab_csi_frame_t *frames = (lab_csi_frame_t *)calloc(256U, sizeof(lab_csi_frame_t));
    ele_snp_t *snp = (ele_snp_t *)calloc(1U, sizeof(ele_snp_t));
    ele_em_passivity_res_t *pres = (ele_em_passivity_res_t *)calloc(1U, sizeof(ele_em_passivity_res_t));
    if (frames == NULL || snp == NULL || pres == NULL) {
        fclose(fp); free(frames); free(snp); free(pres);
        return 1;
    }

    int nf_tot = 0;
    char line[2048];
    while (fgets(line, (int)sizeof(line), fp) != NULL && nf_tot < 256) {
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#' || *p == '\n' || *p == '\r') continue;
        if (!isdigit((unsigned char)*p) && *p != '-') continue;
        long hdr[6] = {0};
        int tok_idx = 0;
        char *cur = p;
        int ok_row = 1;
        while (tok_idx < 6) {
            char *endp = NULL;
            hdr[tok_idx] = strtol(cur, &endp, 10);
            if (endp == cur) { ok_row = 0; break; }
            ++tok_idx;
            cur = endp;
            while (*cur == ',' || *cur == ' ') ++cur;
        }
        if (!ok_row) continue;
        int len = (int)hdr[5];
        if (len < 104) continue;
        int cap_len = (len > 128) ? 128 : len;
        lab_csi_frame_t *fr = &frames[nf_tot];
        fr->ch = (int)hdr[2];
        fr->rssi = (int)hdr[3];
        fr->nf = (int)hdr[4];
        fr->len = cap_len;
        fr->n_sc = cap_len / 2;
        int nb = 0;
        while (nb < cap_len && *cur != '\0' && *cur != '\n' && *cur != '\r') {
            char *endp = NULL;
            long bv = strtol(cur, &endp, 10);
            if (endp == cur) break;
            if (bv < -128) bv = -128;
            if (bv > 127) bv = 127;
            fr->b[nb++] = (int8_t)bv;
            cur = endp;
            while (*cur == ',' || *cur == ' ') ++cur;
        }
        if (nb >= 104) {
            fr->len = nb;
            fr->n_sc = nb / 2;
            ++nf_tot;
        }
    }
    fclose(fp);
    if (nf_tot < 5) {
        fprintf(stderr, "ERROR: Insufficient valid CSI frames (%d < 5) in %s\n", nf_tot, csv_path);
        free(frames); free(snp); free(pres);
        return 1;
    }

    int ch_cnt[16] = {0};
    double ch_rssi[16] = {0};
    for (int f = 0; f < nf_tot; ++f) {
        int c = frames[f].ch;
        if (c >= 1 && c <= 14) {
            ++ch_cnt[c];
            ch_rssi[c] += (double)frames[f].rssi;
        }
    }
    int dom_ch = 1, best_cnt = -1;
    for (int c = 1; c <= 14; ++c) {
        if (ch_cnt[c] > best_cnt || (ch_cnt[c] == best_cnt && ch_cnt[c] > 0 && ch_rssi[c] > ch_rssi[dom_ch])) {
            best_cnt = ch_cnt[c];
            dom_ch = c;
        }
    }

    int sc_hist[65] = {0}, dom_nsc = 64, max_sc_cnt = 0;
    double sum_rssi = 0.0, sum_nf = 0.0;
    int n_used = 0;
    double avg_mag[64] = {0};
    for (int f = 0; f < nf_tot; ++f) {
        if (frames[f].ch == dom_ch && frames[f].n_sc <= 64) {
            ++sc_hist[frames[f].n_sc];
            if (sc_hist[frames[f].n_sc] > max_sc_cnt) {
                max_sc_cnt = sc_hist[frames[f].n_sc];
                dom_nsc = frames[f].n_sc;
            }
        }
    }
    for (int f = 0; f < nf_tot; ++f) {
        if (frames[f].ch != dom_ch || frames[f].n_sc != dom_nsc) continue;
        sum_rssi += (double)frames[f].rssi;
        sum_nf += (double)frames[f].nf;
        for (int k = 0; k < dom_nsc; ++k) {
            double im = (double)frames[f].b[2 * k];
            double re = (double)frames[f].b[2 * k + 1];
            avg_mag[k] += hypot(re, im);
        }
        ++n_used;
    }
    if (n_used < 5) {
        fprintf(stderr, "ERROR: Insufficient frames on dominant channel %d\n", dom_ch);
        free(frames); free(snp); free(pres);
        return 1;
    }
    double mean_rssi_dbm = sum_rssi / (double)n_used;
    double mean_nf_dbm = sum_nf / (double)n_used;
    double snr_db = mean_rssi_dbm - mean_nf_dbm;

    int k_ref = 0;
    for (int k = 1; k < dom_nsc; ++k) {
        if (avg_mag[k] > avg_mag[k_ref]) k_ref = k;
    }

    double mean_re[64] = {0}, mean_im[64] = {0};
    int valid_coh = 0;
    for (int f = 0; f < nf_tot; ++f) {
        if (frames[f].ch != dom_ch || frames[f].n_sc != dom_nsc) continue;
        double rim = (double)frames[f].b[2 * k_ref];
        double rre = (double)frames[f].b[2 * k_ref + 1];
        double rmag = hypot(rre, rim);
        double ux = (rmag > 1e-9) ? (rre / rmag) : 1.0;
        double uy = (rmag > 1e-9) ? (-rim / rmag) : 0.0;
        for (int k = 0; k < dom_nsc; ++k) {
            double im = (double)frames[f].b[2 * k];
            double re = (double)frames[f].b[2 * k + 1];
            mean_re[k] += re * ux - im * uy;
            mean_im[k] += re * uy + im * ux;
        }
        ++valid_coh;
    }
    double max_h_mag = 0.0;
    double h_mag[64] = {0};
    for (int k = 0; k < dom_nsc; ++k) {
        mean_re[k] /= (double)valid_coh;
        mean_im[k] /= (double)valid_coh;
        h_mag[k] = hypot(mean_re[k], mean_im[k]);
        if (h_mag[k] > max_h_mag) max_h_mag = h_mag[k];
    }
    if (max_h_mag < 1e-6) {
        fprintf(stderr, "ERROR: Degenerate zero CSI amplitude\n");
        free(frames); free(snp); free(pres);
        return 1;
    }

    int is_act[64] = {0};
    int m_idx[64];
    int act_bins[64];
    int n_active = 0;
    for (int k = 0; k < dom_nsc; ++k) {
        m_idx[k] = (dom_nsc == 64) ? ((k < 32) ? k : (k - 64)) : (k - dom_nsc / 2);
        if (h_mag[k] > 0.05 * max_h_mag) {
            is_act[k] = 1;
            act_bins[n_active++] = k;
        }
    }
    if (n_active < 10) {
        fprintf(stderr, "ERROR: Too few active CSI subcarriers (%d < 10)\n", n_active);
        free(frames); free(snp); free(pres);
        return 1;
    }
    for (int i = 0; i < n_active; ++i) {
        for (int j = i + 1; j < n_active; ++j) {
            if (m_idx[act_bins[j]] < m_idx[act_bins[i]]) {
                int tmp = act_bins[i];
                act_bins[i] = act_bins[j];
                act_bins[j] = tmp;
            }
        }
    }

    /* 64-pt discrete IFFT power delay profile (dt = 50 ns) */
    double p64[64];
    int n_peak = 0;
    double max_p = -1.0;
    for (int n = 0; n < 64; ++n) {
        double hre = 0.0, him = 0.0;
        for (int k = 0; k < dom_nsc; ++k) {
            if (!is_act[k]) continue;
            double ang = 2.0 * M_PI * (double)(m_idx[k] * n) / 64.0;
            double ca = cos(ang), sa = sin(ang);
            hre += mean_re[k] * ca - mean_im[k] * sa;
            him += mean_re[k] * sa + mean_im[k] * ca;
        }
        hre /= 64.0;
        him /= 64.0;
        p64[n] = hre * hre + him * him;
        if (p64[n] > max_p) { max_p = p64[n]; n_peak = n; }
    }
    double p_tot = 0.0, tau_num = 0.0, tau_sq = 0.0;
    for (int m = 0; m < 24; ++m) {
        int idx = (n_peak + m) % 64;
        double pm = (p64[idx] >= 0.005 * max_p) ? p64[idx] : 0.0;
        double tm = 50.0 * (double)m;
        p_tot += pm;
        tau_num += tm * pm;
        tau_sq += tm * tm * pm;
    }
    double tau_mean = (p_tot > 0.0) ? (tau_num / p_tot) : 0.0;
    double var_tau = (p_tot > 0.0) ? fmax(0.0, tau_sq / p_tot - tau_mean * tau_mean) : 0.0;
    double tau_rms_ns = fmax(5.0, sqrt(var_tau));
    double bc_mhz = 1000.0 / (2.0 * M_PI * tau_rms_ns);

    double center_mhz = (double)(2407 + 5 * dom_ch);
    double pl_db = fmin(fmax(15.0 - mean_rssi_dbm, 15.0), 80.0);
    double g_path = pow(10.0, -pl_db / 20.0);

    snp->nports = 2;
    snp->nfreq = n_active;
    snp->z0 = 50.0;
    for (int i = 0; i < n_active; ++i) {
        int k = act_bins[i];
        snp->freq_hz[i] = center_mhz * 1e6 + (double)m_idx[k] * 312500.0;
        double s21_r = g_path * (mean_re[k] / max_h_mag);
        double s21_i = g_path * (mean_im[k] / max_h_mag);
        /* Row-major 2x2: (0,0)=S11, (0,1)=S12, (1,0)=S21, (1,1)=S22 */
        snp->sr[i][0] = 0.10; snp->si[i][0] = 0.0;
        snp->sr[i][1] = s21_r; snp->si[i][1] = s21_i;
        snp->sr[i][2] = s21_r; snp->si[i][2] = s21_i;
        snp->sr[i][3] = 0.10; snp->si[i][3] = 0.0;
    }
    if (s2p_path != NULL && ele_snp_save(s2p_path, snp) != 0) {
        fprintf(stderr, "ERROR: Failed saving CSI Touchstone .s2p to %s\n", s2p_path);
        free(frames); free(snp); free(pres);
        return 1;
    }
    snprintf(snp->input_path, sizeof(snp->input_path), "%s", csv_path);
    if (ele_sha256_file(csv_path, snp->input_sha256) != 0 ||
        ele_em_certify_snp(snp, pres) != 0) {
        free(frames); free(snp); free(pres);
        return 1;
    }
    snprintf(pres->cert.pillar, sizeof(pres->cert.pillar), "lab");
    snprintf(pres->cert.kind, sizeof(pres->cert.kind), "cardputer_rf_csi");

    if (cert_path != NULL && ele_cert_write_json(cert_path, &pres->cert) != 0) {
        free(frames); free(snp); free(pres);
        return 1;
    }
    if (json_path != NULL) {
        FILE *jf = fopen(json_path, "w");
        if (jf != NULL) {
            fprintf(jf,
                    "{\n"
                    "  \"n_frames\": %d,\n"
                    "  \"dom_ch\": %d,\n"
                    "  \"center_freq_mhz\": %.1f,\n"
                    "  \"mean_rssi_dbm\": %.3f,\n"
                    "  \"mean_nf_dbm\": %.3f,\n"
                    "  \"snr_db\": %.3f,\n"
                    "  \"n_active_sc\": %d,\n"
                    "  \"tau_rms_ns\": %.3f,\n"
                    "  \"bc_mhz\": %.3f,\n"
                    "  \"path_loss_db\": %.3f,\n"
                    "  \"sigma_max\": %.9f,\n"
                    "  \"recip_err\": %.9e,\n"
                    "  \"passive\": %d,\n"
                    "  \"mu_cert\": %" PRId64 ",\n"
                    "  \"status\": \"%s\"\n"
                    "}\n",
                    n_used, dom_ch, center_mhz, mean_rssi_dbm, mean_nf_dbm, snr_db,
                    n_active, tau_rms_ns, bc_mhz, pl_db, pres->sigma_max,
                    pres->max_recip_err, pres->passive_certified, pres->min_mu_cert,
                    pres->cert.status);
            fclose(jf);
        }
    }
    int ok = (pres->passive_certified && pres->reciprocal_certified && n_used >= 5);
    printf("[LAB CSI] %s -> frames=%d, ch%d (%.0f MHz), RSSI=%.1f dBm (SNR=%.1f dB), active=%d SC, tau_rms=%.1f ns, sigma=%.5f (%s)\n",
           csv_path, n_used, dom_ch, center_mhz, mean_rssi_dbm, snr_db, n_active,
           tau_rms_ns, pres->sigma_max, pres->cert.status);
    free(frames); free(snp); free(pres);
    return ok ? 0 : 1;
}

static int run_bench(const char *dir, const char *out_json, const char *cert_dir) {
    char p_micro[512], p_wilk[512], p_nonp[512], p_peec[512];
    char p_rc[512], p_pdn[512], p_diode[512], p_sic[512], p_si[512];
    char p_mot[512], p_g3[512], p_g9[512], p_g14[512];

    snprintf(p_micro, sizeof(p_micro), "%s/em/microstrip_50ohm.s2p", dir);
    snprintf(p_wilk, sizeof(p_wilk), "%s/em/wilkinson_3port.s3p", dir);
    snprintf(p_nonp, sizeof(p_nonp), "%s/em/nonpassive_ai_surrogate.s2p", dir);
    snprintf(p_peec, sizeof(p_peec), "%s/em/busbar_800v.peec", dir);

    snprintf(p_rc, sizeof(p_rc), "%s/ckt/rc_lowpass.sp", dir);
    snprintf(p_pdn, sizeof(p_pdn), "%s/ckt/rlc_pdn.sp", dir);
    snprintf(p_diode, sizeof(p_diode), "%s/ckt/diode_rectifier.sp", dir);
    snprintf(p_sic, sizeof(p_sic), "%s/ckt/sic_halfbridge_800v.sp", dir);
    snprintf(p_si, sizeof(p_si), "%s/ckt/diff_pair_si.sp", dir);

    snprintf(p_mot, sizeof(p_mot), "%s/ee/ev_ipmsm_150kw.motor", dir);
    snprintf(p_g3, sizeof(p_g3), "%s/ee/ieee3_test.grid", dir);
    snprintf(p_g9, sizeof(p_g9), "%s/ee/ieee9_wscc.grid", dir);
    snprintf(p_g14, sizeof(p_g14), "%s/ee/ieee14_grid.grid", dir);

    ele_snp_t s_micro, s_wilk, s_nonp;
    ele_em_passivity_res_t pr_micro, pr_wilk, pr_nonp;
    if (ele_snp_load(p_micro, 2, &s_micro) != 0 || ele_em_certify_snp(&s_micro, &pr_micro) != 0) return -1;
    if (ele_snp_load(p_wilk, 3, &s_wilk) != 0 || ele_em_certify_snp(&s_wilk, &pr_wilk) != 0) return -1;
    if (ele_snp_load(p_nonp, 2, &s_nonp) != 0 || ele_em_certify_snp(&s_nonp, &pr_nonp) != 0) return -1;

    ele_peec_spec_t ps;
    ele_em_peec_res_t pres;
    char peec_s2p[512];
    snprintf(peec_s2p, sizeof(peec_s2p), "%s/busbar_800v_peec.s2p", cert_dir);
    if (ele_em_peec_load(p_peec, &ps) != 0 || ele_em_peec_extract(&ps, peec_s2p, &pres) != 0) return -1;

    ele_em_fdtd_res_t fd;
    char fdtd_s2p[512];
    snprintf(fdtd_s2p, sizeof(fdtd_s2p), "%s/fdtd_wg_slab.s2p", cert_dir);
    if (ele_em_fdtd_run(4.0, 0.04, 160, 32, 1600, fdtd_s2p, &fd) != 0) return -1;

    ele_em_array_res_t ar;
    ele_em_match_res_t mr;
    if (ele_em_synth_array(2026U, &ar) != 0 || ele_em_synth_match(2026U, &mr) != 0) return -1;

    ele_ckt_netlist_t nl_rc, nl_pdn, nl_di, nl_sic, nl_si;
    ele_ckt_run_res_t r_rc, r_pdn, r_di, r_sic;
    ele_ckt_eye_res_t r_eye;
    if (ele_ckt_load(p_rc, &nl_rc) != 0 || ele_ckt_run_all(&nl_rc, 0, &r_rc) != 0) return -1;
    if (ele_ckt_load(p_pdn, &nl_pdn) != 0 || ele_ckt_run_all(&nl_pdn, 0, &r_pdn) != 0) return -1;
    if (ele_ckt_load(p_diode, &nl_di) != 0 || ele_ckt_run_all(&nl_di, 0, &r_di) != 0) return -1;
    if (ele_ckt_load(p_sic, &nl_sic) != 0 || ele_ckt_run_all(&nl_sic, 1, &r_sic) != 0) return -1;
    if (ele_ckt_load(p_si, &nl_si) != 0 || ele_ckt_si_eye(&nl_si, 4.0, &r_eye) != 0) return -1;

    ele_motor_spec_t ms;
    ele_motor_op_res_t mop_3k, mop_10k;
    ele_motor_sweep_res_t msw;
    if (ele_motor_load(p_mot, &ms) != 0 ||
        ele_motor_solve_op(&ms, 3000.0, 260.0, &mop_3k) != 0 ||
        ele_motor_solve_op(&ms, 10000.0, 0.0, &mop_10k) != 0 ||
        ele_motor_sweep_and_foc(&ms, 260.0, &msw) != 0) return -1;

    ele_grid_t g3, g9, g14;
    ele_grid_pf_res_t pf3, pf9, pf14;
    ele_grid_n1_res_t n1_3, n1_9, n1_14;
    ele_grid_opf_res_t opf3, opf9;
    ele_grid_emt_res_t emt9;
    if (ele_grid_load(p_g3, &g3) != 0 || ele_grid_load(p_g9, &g9) != 0 || ele_grid_load(p_g14, &g14) != 0) return -1;
    if (ele_grid_acpf(&g3, -1, &pf3) != 0 || ele_grid_acpf(&g9, -1, &pf9) != 0 || ele_grid_acpf(&g14, -1, &pf14) != 0) return -1;
    if (ele_grid_n1(&g3, &n1_3) != 0 || ele_grid_n1(&g9, &n1_9) != 0 || ele_grid_n1(&g14, &n1_14) != 0) return -1;
    if (ele_grid_acopf(&g3, &opf3) != 0 || ele_grid_acopf(&g9, &opf9) != 0) return -1;
    if (ele_grid_emt(&g9, 7, 0.12, -1, &emt9) != 0) return -1;

    /* Official IEEE PES PGLib-OPF v23.07 sparse power-flow & certified OPF benchmarks */
    const char *pg_rel[4] = {
        "ee/pglib/pglib_opf_case14_ieee.m",
        "ee/pglib/pglib_opf_case30_ieee.m",
        "ee/pglib/pglib_opf_case57_ieee.m",
        "ee/pglib/pglib_opf_case118_ieee.m"
    };
    const char *pg_names[4] = {"pglib14", "pglib30", "pglib57", "pglib118"};
    ele_grid_t *pg = (ele_grid_t *)calloc(1U, sizeof(ele_grid_t));
    ele_grid_pf_res_t *ppf = (ele_grid_pf_res_t *)calloc(4U, sizeof(ele_grid_pf_res_t));
    ele_grid_opf_res_t *popf = (ele_grid_opf_res_t *)calloc(2U, sizeof(ele_grid_opf_res_t));
    if (pg == NULL || ppf == NULL || popf == NULL) {
        free(pg); free(ppf); free(popf);
        return -1;
    }
    int pg_nbus[4] = {0}, pg_nbr[4] = {0}, pg_ngen[4] = {0};
    for (int i = 0; i < 4; ++i) {
        char p_pg[512];
        snprintf(p_pg, sizeof(p_pg), "%s/%s", dir, pg_rel[i]);
        if (ele_grid_load(p_pg, pg) != 0 || ele_grid_acpf(pg, -1, &ppf[i]) != 0) {
            free(pg); free(ppf); free(popf);
            return -1;
        }
        pg_nbus[i] = pg->n_bus;
        pg_nbr[i] = pg->n_branch;
        pg_ngen[i] = pg->n_gen;
        if (i < 2) {
            if (ele_grid_acopf(pg, &popf[i]) != 0) {
                free(pg); free(ppf); free(popf);
                return -1;
            }
        }
    }

    /* Write certificates into cert_dir */
    char cpath[512];
    snprintf(cpath, sizeof(cpath), "%s/em_microstrip.elecert.json", cert_dir); ele_cert_write_json(cpath, &pr_micro.cert);
    snprintf(cpath, sizeof(cpath), "%s/em_wilkinson.elecert.json", cert_dir); ele_cert_write_json(cpath, &pr_wilk.cert);
    snprintf(cpath, sizeof(cpath), "%s/em_nonpassive_veto.elecert.json", cert_dir); ele_cert_write_json(cpath, &pr_nonp.cert);
    snprintf(cpath, sizeof(cpath), "%s/em_peec_busbar.elecert.json", cert_dir); ele_cert_write_json(cpath, &pres.passivity.cert);
    snprintf(cpath, sizeof(cpath), "%s/em_fdtd_slab.elecert.json", cert_dir); ele_cert_write_json(cpath, &fd.passivity.cert);
    snprintf(cpath, sizeof(cpath), "%s/ckt_sic_800v.elecert.json", cert_dir); ele_cert_write_json(cpath, &r_sic.cert);
    snprintf(cpath, sizeof(cpath), "%s/ckt_si_eye.elecert.json", cert_dir); ele_cert_write_json(cpath, &r_eye.cert);
    snprintf(cpath, sizeof(cpath), "%s/motor_ipmsm.elecert.json", cert_dir); ele_cert_write_json(cpath, &mop_3k.cert);
    snprintf(cpath, sizeof(cpath), "%s/grid_ieee3_opf.elecert.json", cert_dir); ele_cert_write_json(cpath, &opf3.cert);
    snprintf(cpath, sizeof(cpath), "%s/grid_ieee9_opf.elecert.json", cert_dir); ele_cert_write_json(cpath, &opf9.cert);
    snprintf(cpath, sizeof(cpath), "%s/grid_ieee9_emt.elecert.json", cert_dir); ele_cert_write_json(cpath, &emt9.cert);
    snprintf(cpath, sizeof(cpath), "%s/grid_pglib14_opf.elecert.json", cert_dir); ele_cert_write_json(cpath, &popf[0].cert);
    snprintf(cpath, sizeof(cpath), "%s/grid_pglib30_opf.elecert.json", cert_dir); ele_cert_write_json(cpath, &popf[1].cert);

    /* Write bench JSON */
    FILE *fp = fopen(out_json, "w");
    if (fp == NULL) { free(pg); free(ppf); free(popf); return -1; }
    fprintf(fp, "{\n");
    fprintf(fp, "  \"version\": \"%s\",\n", ELE_VERSION_STRING);
    fprintf(fp, "  \"em\": {\n");
    fprintf(fp, "    \"microstrip\": {\"passive\": %d, \"sigma_max\": %.9f, \"recip_err\": %.9e, \"mu_cert\": %" PRId64 "},\n",
            pr_micro.passive_certified, pr_micro.sigma_max, pr_micro.max_recip_err, pr_micro.min_mu_cert);
    fprintf(fp, "    \"wilkinson\": {\"passive\": %d, \"sigma_max\": %.9f, \"recip_err\": %.9e, \"mu_cert\": %" PRId64 "},\n",
            pr_wilk.passive_certified, pr_wilk.sigma_max, pr_wilk.max_recip_err, pr_wilk.min_mu_cert);
    fprintf(fp, "    \"nonpassive_veto\": {\"passive\": %d, \"sigma_max\": %.9f, \"status\": \"%s\", \"mu_cert\": %" PRId64 "},\n",
            pr_nonp.passive_certified, pr_nonp.sigma_max, pr_nonp.cert.status, pr_nonp.min_mu_cert);
    fprintf(fp, "    \"peec_busbar\": {\"l_loop_h\": %.9e, \"c_bus_f\": %.9e, \"r_dc_ohm\": %.9e, \"r_ac_fmax_ohm\": %.9e, \"passive\": %d},\n",
            pres.l_loop_h, pres.c_bus_f, pres.r_dc_ohm, pres.r_ac_fmax_ohm, pres.passivity.passive_certified);
    fprintf(fp, "    \"fdtd_slab\": {\"cfl_ok\": %d, \"fc_hz\": %.3f, \"min_s11_db\": %.4f, \"max_s21_db\": %.4f, \"max_energy_sum\": %.6f, \"passive\": %d},\n",
            fd.cfl_ok, fd.fc_hz, fd.min_s11_db, fd.max_s21_db, fd.max_energy_sum, fd.passivity.passive_certified);
    fprintf(fp, "    \"synth_array\": {\"baseline_sll_db\": %.4f, \"nominal_sll_db\": %.4f, \"worst_tol_sll_db\": %.4f},\n",
            ar.baseline_sll_db, ar.nominal_sll_db, ar.worst_tol_sll_db);
    fprintf(fp, "    \"synth_match\": {\"baseline_s11_db\": %.4f, \"nominal_s11_db\": %.4f, \"worst_tol_s11_db\": %.4f}\n",
            mr.baseline_s11_db, mr.nominal_s11_db, mr.worst_tol_s11_db);
    fprintf(fp, "  },\n");

    fprintf(fp, "  \"ckt\": {\n");
    fprintf(fp, "    \"rc_lowpass\": {\"bw_3db_hz\": %.6f, \"v_final\": %.9f},\n",
            r_rc.ac.bw_3db_hz, r_rc.tran.probe_v_final);
    fprintf(fp, "    \"rlc_pdn\": {\"z_pdn_max_ohm\": %.9f, \"fpeak_hz\": %.3f, \"pdn_pass\": %d},\n",
            r_pdn.ac.z_pdn_max_ohm, r_pdn.ac.z_pdn_fpeak_hz, r_pdn.ac.pdn_pass);
    fprintf(fp, "    \"diode_rectifier\": {\"max_kcl_res\": %.6e, \"v_diode\": %.9f, \"v_probe_max\": %.9f},\n",
            r_di.dc.max_kcl_res, r_di.dc.node_v[nl_di.n_nodes - 1], r_di.tran.probe_v_max);
    fprintf(fp, "    \"sic_800v\": {\"n_corners\": %d, \"nom_vmax\": %.6f, \"wc_vmin\": %.6f, \"wc_vmax\": %.6f, \"soa_pass\": %d},\n",
            r_sic.n_corners, r_sic.nom_vmax, r_sic.wc_vmin, r_sic.wc_vmax, r_sic.soa_pass);
    fprintf(fp, "    \"si_eye\": {\"rate_gbps\": %.3f, \"eye_pda_v\": %.6f, \"eye_searu_v\": %.6f, \"eye_pass\": %d}\n",
            r_eye.bit_rate_gbps, r_eye.eye_pda_v, r_eye.eye_searu_v, r_eye.eye_pass);
    fprintf(fp, "  },\n");

    fprintf(fp, "  \"motor\": {\n");
    fprintf(fp, "    \"op_3k\": {\"id_a\": %.6f, \"iq_a\": %.6f, \"psi_d\": %.9f, \"psi_q\": %.9f, \"torque_nm\": %.6f, \"vd_v\": %.6f, \"vq_v\": %.6f, \"v_max_v\": %.6f, \"eff\": %.6f},\n",
            mop_3k.id_a, mop_3k.iq_a, mop_3k.psi_d_wb, mop_3k.psi_q_wb, mop_3k.torque_nm, mop_3k.vd_v, mop_3k.vq_v, mop_3k.v_max_v, mop_3k.efficiency);
    fprintf(fp, "    \"op_10k_fw\": {\"id_a\": %.6f, \"iq_a\": %.6f, \"torque_nm\": %.6f, \"v_mag_v\": %.6f, \"v_max_v\": %.6f, \"eff\": %.6f},\n",
            mop_10k.id_a, mop_10k.iq_a, mop_10k.torque_nm, mop_10k.v_mag_v, mop_10k.v_max_v, mop_10k.efficiency);
    fprintf(fp, "    \"foc\": {\"converged\": %d, \"final_torque_nm\": %.4f, \"peak_i_a\": %.4f}\n",
            msw.foc_converged, msw.foc_final_torque_nm, msw.foc_peak_i_a);
    fprintf(fp, "  },\n");

    fprintf(fp, "  \"grid\": {\n");
    struct { const char *name; const ele_grid_t *g; const ele_grid_pf_res_t *pf; const ele_grid_n1_res_t *n1; } pfs[3] = {
        {"ieee3", &g3, &pf3, &n1_3},
        {"ieee9", &g9, &pf9, &n1_9},
        {"ieee14", &g14, &pf14, &n1_14}
    };
    for (int i = 0; i < 3; ++i) {
        fprintf(fp, "    \"%s_pf\": {\"converged\": %d, \"mismatch\": %.6e, \"loss_mw\": %.6f, \"n1_converged\": %d, \"n1_secure\": %d, \"vm\": [",
                pfs[i].name, pfs[i].pf->converged, pfs[i].pf->max_mismatch_pu, pfs[i].pf->total_p_loss_mw,
                pfs[i].n1->n_converged, pfs[i].n1->n1_secure);
        for (int k = 0; k < pfs[i].g->n_bus; ++k) fprintf(fp, "%s%.15f", (k > 0) ? ", " : "", pfs[i].pf->vm[k]);
        fprintf(fp, "], \"va_rad\": [");
        for (int k = 0; k < pfs[i].g->n_bus; ++k) fprintf(fp, "%s%.15f", (k > 0) ? ", " : "", pfs[i].pf->va_rad[k]);
        fprintf(fp, "]},\n");
    }
    fprintf(fp, "    \"ieee3_opf\": {\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_pct\": %.6f, \"mu_psd\": %" PRId64 "},\n",
            opf3.c_lb_cert, opf3.c_ub_primal, opf3.gap_pct, opf3.mu_psd_int);
    fprintf(fp, "    \"ieee9_opf\": {\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_pct\": %.6f, \"mu_psd\": %" PRId64 "},\n",
            opf9.c_lb_cert, opf9.c_ub_primal, opf9.gap_pct, opf9.mu_psd_int);
    fprintf(fp, "    \"ieee9_emt\": {\"stable\": %d, \"v_post\": %.6f, \"v_crit\": %.6f, \"cct_sec\": %.6f, \"max_gfm_i_pu\": %.6f},\n",
            emt9.stable, emt9.v_post, emt9.v_crit, emt9.cct_sec, emt9.max_gfm_i_pu);
    for (int i = 0; i < 4; ++i) {
        fprintf(fp, "    \"%s_pf\": {\"n_bus\": %d, \"n_branch\": %d, \"n_gen\": %d, \"converged\": %d, \"mismatch\": %.6e, \"loss_mw\": %.6f, \"vm\": [",
                pg_names[i], pg_nbus[i], pg_nbr[i], pg_ngen[i],
                ppf[i].converged, ppf[i].max_mismatch_pu, ppf[i].total_p_loss_mw);
        for (int k = 0; k < pg_nbus[i]; ++k) fprintf(fp, "%s%.15f", (k > 0) ? ", " : "", ppf[i].vm[k]);
        fprintf(fp, "], \"va_rad\": [");
        for (int k = 0; k < pg_nbus[i]; ++k) fprintf(fp, "%s%.15f", (k > 0) ? ", " : "", ppf[i].va_rad[k]);
        fprintf(fp, "]},\n");
    }
    fprintf(fp, "    \"pglib14_opf\": {\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_pct\": %.6f, \"mu_psd\": %" PRId64 "},\n",
            popf[0].c_lb_cert, popf[0].c_ub_primal, popf[0].gap_pct, popf[0].mu_psd_int);
    fprintf(fp, "    \"pglib30_opf\": {\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_pct\": %.6f, \"mu_psd\": %" PRId64 "}\n",
            popf[1].c_lb_cert, popf[1].c_ub_primal, popf[1].gap_pct, popf[1].mu_psd_int);
    fprintf(fp, "  }\n}\n");
    fclose(fp);
    free(pg); free(ppf); free(popf);
    printf("  [BENCH] Verified EM + CKT + Motor + Grid (incl. PGLib 14/30/57/118) suite -> %s\n", out_json);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) { print_usage(); return 1; }
    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
        print_usage();
        return 0;
    }
    if (strcmp(argv[1], "--version") == 0 || strcmp(argv[1], "-v") == 0) {
        printf("ELE %s\n", ELE_VERSION_STRING);
        return 0;
    }
    if (strcmp(argv[1], "selftest") == 0) {
        const char *jout = NULL;
        for (int i = 2; i + 1 < argc; ++i) {
            if (strcmp(argv[i], "--json") == 0) jout = argv[++i];
        }
        return run_selftest(jout);
    }
    if (strcmp(argv[1], "cert-verify") == 0 && argc >= 3) {
        int rc = ele_cert_verify_file(argv[2], 1);
        printf("cert-verify %s -> %s (rc=%d)\n", argv[2], (rc == 0) ? "PASS" : "FAIL", rc);
        return (rc == 0) ? 0 : 1;
    }

    if (strcmp(argv[1], "em") == 0 && argc >= 3) {
        if (strcmp(argv[2], "passivity") == 0 && argc >= 4) {
            const char *in_p = argv[3], *cert_p = NULL, *lean_p = NULL, *mod_n = "CertEm", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_snp_t snp;
            ele_em_passivity_res_t res;
            if (ele_snp_load(in_p, 0, &snp) != 0 || ele_em_certify_snp(&snp, &res) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &res.cert) != 0) return 1;
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &res.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"status\": \"%s\", \"passive\": %d, \"sigma_max\": %.9f, \"recip_err\": %.9e, \"mu_cert\": %" PRId64 "}\n",
                            res.cert.status, res.passive_certified, res.sigma_max, res.max_recip_err, res.min_mu_cert);
                    fclose(fp);
                }
            }
            printf("[EM PASSIVITY] %s -> %s (sigma_max=%.5f, recip_err=%.3e, mu_cert=%" PRId64 ")\n",
                   in_p, res.cert.status, res.sigma_max, res.max_recip_err, res.min_mu_cert);
            return 0;
        }
        if (strcmp(argv[2], "fdtd") == 0) {
            double er = 4.0, sig = 0.04;
            int nx = 160, ny = 32, nt = 1600;
            const char *s2p_p = NULL, *cert_p = NULL, *json_p = NULL;
            for (int i = 3; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--er") == 0) er = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--sigma") == 0) sig = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--nx") == 0) nx = atoi(argv[++i]);
                else if (strcmp(argv[i], "--ny") == 0) ny = atoi(argv[++i]);
                else if (strcmp(argv[i], "--nt") == 0) nt = atoi(argv[++i]);
                else if (strcmp(argv[i], "--s2p") == 0) s2p_p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_em_fdtd_res_t res;
            if (ele_em_fdtd_run(er, sig, nx, ny, nt, s2p_p, &res) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &res.passivity.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"cfl_ok\": %d, \"fc_hz\": %.3f, \"min_s11_db\": %.4f, \"max_s21_db\": %.4f, \"passive\": %d}\n",
                            res.cfl_ok, res.fc_hz, res.min_s11_db, res.max_s21_db, res.passivity.passive_certified);
                    fclose(fp);
                }
            }
            printf("[EM FDTD] fc=%.3f GHz, minS11=%.2f dB, maxS21=%.2f dB, passive=%d\n",
                   res.fc_hz * 1e-9, res.min_s11_db, res.max_s21_db, res.passivity.passive_certified);
            return 0;
        }
        if (strcmp(argv[2], "peec") == 0 && argc >= 4) {
            const char *in_p = argv[3], *s2p_p = NULL, *cert_p = NULL, *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--s2p") == 0) s2p_p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_peec_spec_t ps;
            ele_em_peec_res_t res;
            if (ele_em_peec_load(in_p, &ps) != 0 || ele_em_peec_extract(&ps, s2p_p, &res) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &res.passivity.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"l_loop_h\": %.9e, \"c_bus_f\": %.9e, \"r_dc_ohm\": %.9e, \"r_ac_fmax_ohm\": %.9e, \"passive\": %d}\n",
                            res.l_loop_h, res.c_bus_f, res.r_dc_ohm, res.r_ac_fmax_ohm, res.passivity.passive_certified);
                    fclose(fp);
                }
            }
            printf("[EM PEEC] L_loop=%.3f nH, C_bus=%.1f pF, R_dc=%.3f mOhm, R_ac=%.3f mOhm, passive=%d\n",
                   res.l_loop_h * 1e9, res.c_bus_f * 1e12, res.r_dc_ohm * 1e3, res.r_ac_fmax_ohm * 1e3, res.passivity.passive_certified);
            return 0;
        }
        if (strcmp(argv[2], "synth") == 0) {
            const char *mode = "array", *json_p = NULL;
            uint64_t seed = 2026U;
            for (int i = 3; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--mode") == 0) mode = argv[++i];
                else if (strcmp(argv[i], "--seed") == 0) seed = strtoull(argv[++i], NULL, 10);
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            if (strcmp(mode, "array") == 0) {
                ele_em_array_res_t ar;
                if (ele_em_synth_array(seed, &ar) != 0) return 1;
                if (json_p != NULL) {
                    FILE *fp = fopen(json_p, "w");
                    if (fp != NULL) {
                        fprintf(fp, "{\"baseline_sll_db\": %.4f, \"nominal_sll_db\": %.4f, \"worst_tol_sll_db\": %.4f}\n",
                                ar.baseline_sll_db, ar.nominal_sll_db, ar.worst_tol_sll_db);
                        fclose(fp);
                    }
                }
                printf("[EM SYNTH ARRAY] baseline=%.2f dB -> nom=%.2f dB, wc_tol=%.2f dB\n",
                       ar.baseline_sll_db, ar.nominal_sll_db, ar.worst_tol_sll_db);
                return 0;
            }
            if (strcmp(mode, "match") == 0) {
                ele_em_match_res_t mr;
                if (ele_em_synth_match(seed, &mr) != 0) return 1;
                if (json_p != NULL) {
                    FILE *fp = fopen(json_p, "w");
                    if (fp != NULL) {
                        fprintf(fp, "{\"baseline_s11_db\": %.4f, \"nominal_s11_db\": %.4f, \"worst_tol_s11_db\": %.4f, \"z_sec\": [%.3f, %.3f, %.3f]}\n",
                                mr.baseline_s11_db, mr.nominal_s11_db, mr.worst_tol_s11_db, mr.z_sec[0], mr.z_sec[1], mr.z_sec[2]);
                        fclose(fp);
                    }
                }
                printf("[EM SYNTH MATCH] Z=(%.2f, %.2f, %.2f) | raw=%.2f dB -> nom=%.2f dB, wc_tol=%.2f dB\n",
                       mr.z_sec[0], mr.z_sec[1], mr.z_sec[2], mr.baseline_s11_db, mr.nominal_s11_db, mr.worst_tol_s11_db);
                return 0;
            }
        }
        if (strcmp(argv[2], "bound") == 0) {
            double kl = 2.0, zeta = 10.0, xi0 = 0.0, bw_frac = 0.0;
            int n_cells = 16, fdtd_chk = 0;
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertEmMaxwellBound", *json_p = NULL;
            for (int i = 3; i < argc; ++i) {
                if (strcmp(argv[i], "--kl") == 0 && i + 1 < argc) kl = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--zeta") == 0 && i + 1 < argc) zeta = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--xi0") == 0 && i + 1 < argc) xi0 = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--cells") == 0 && i + 1 < argc) n_cells = atoi(argv[++i]);
                else if (strcmp(argv[i], "--bw-frac") == 0 && i + 1 < argc) bw_frac = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--fdtd-check") == 0) fdtd_chk = 1;
                else if (strcmp(argv[i], "--cert") == 0 && i + 1 < argc) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0 && i + 1 < argc) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0 && i + 1 < argc) json_p = argv[++i];
            }
            ele_em_bound_ext_res_t *ex = (ele_em_bound_ext_res_t *)calloc(1, sizeof(ele_em_bound_ext_res_t));
            if (ex == NULL) return 1;
            if (ele_em_maxwell_bound_ext(kl, zeta, xi0, n_cells, bw_frac, fdtd_chk, ex) != 0) {
                free(ex);
                return 1;
            }
            ele_em_bound_res_t *bres = &ex->base;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &bres->cert) != 0) { free(ex); return 1; }
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &bres->cert) != 0) { free(ex); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"kl\": %.6f, \"zeta\": %.6f, \"xi0\": %.6f, \"n_cells\": %d, "
                            "\"p_global_ub\": %.9f, \"p_local_ub_raw\": %.9f, \"p_local_ub_cert\": %.9f, "
                            "\"p_searu_lb\": %.9f, \"tightening_ratio\": %.9f, \"efficiency_gap_pct\": %.6f, "
                            "\"searu_best_mask\": %u, \"mu_psd\": %" PRId64 ", "
                            "\"bf_gamma_limit\": %.9f, \"bf_s11_limit_db\": %.6f, "
                            "\"bw_frac\": %.6f, \"n_freq_pts\": %d, \"p_band_avg_ub\": %.9f, "
                            "\"p_band_avg_searu\": %.9f, \"fdtd_checked\": %d, \"fdtd_ext_power\": %.9f, "
                            "\"fdtd_below_ub\": %d, \"status\": \"%s\"}\n",
                            bres->kl, bres->zeta, bres->xi0, bres->n_cells,
                            bres->p_global_ub, bres->p_local_ub_raw, bres->p_local_ub_cert,
                            bres->p_searu_lb, bres->tightening_ratio, bres->efficiency_gap_pct,
                            bres->searu_best_mask, bres->mu_psd_int,
                            bres->bf_gamma_limit, bres->bf_s11_limit_db,
                            ex->bw_frac, ex->n_freq_pts, ex->p_band_avg_ub,
                            ex->p_band_avg_searu, ex->fdtd_checked, ex->fdtd_ext_power,
                            ex->fdtd_below_ub, bres->cert.status);
                    fclose(fp);
                }
            }
            printf("[EM BOUND] kL=%.2f zeta=%.1f M=%d -> SEARU=%.6f <= LocalCert=%.6f <= Global=%.6f (ratio=%.4f",
                   bres->kl, bres->zeta, bres->n_cells,
                   bres->p_searu_lb, bres->p_local_ub_cert, bres->p_global_ub, bres->tightening_ratio);
            if (bw_frac > 0.0) {
                printf(", band(%d)=[%.4f<=%.4f]", ex->n_freq_pts, ex->p_band_avg_searu, ex->p_band_avg_ub);
            }
            if (ex->fdtd_checked) {
                printf(", FDTD=%.6f(%s)", ex->fdtd_ext_power, ex->fdtd_below_ub ? "BELOW_UB" : "OVER");
            }
            printf(", %s)\n", bres->cert.status);
            free(ex);
            return 0;
        }
        if (strcmp(argv[2], "bode-fano") == 0) {
            double z0 = 50.0, rl = 50.0, cl_pf = 2.0, flo_ghz = 2.0, fhi_ghz = 3.0;
            uint64_t seed = 20261122ULL;
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertEmBodeFano", *json_p = NULL;
            for (int i = 3; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--z0") == 0) z0 = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--rl") == 0) rl = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--cl-pf") == 0) cl_pf = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--flo-ghz") == 0) flo_ghz = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--fhi-ghz") == 0) fhi_ghz = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--seed") == 0) seed = (uint64_t)strtoull(argv[++i], NULL, 10);
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_em_bode_fano_res_t *bf = (ele_em_bode_fano_res_t *)calloc(1, sizeof(ele_em_bode_fano_res_t));
            if (bf == NULL) return 1;
            if (ele_em_bode_fano_synth(z0, rl, cl_pf * 1e-12, flo_ghz * 1e9, fhi_ghz * 1e9, seed, bf) != 0) {
                free(bf);
                return 1;
            }
            if (cert_p != NULL && ele_cert_write_json(cert_p, &bf->cert) != 0) { free(bf); return 1; }
            if (lean_p != NULL && ele_em_bode_fano_emit_lean(lean_p, mod_n, bf) != 0) { free(bf); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"z0\": %.4f, \"rl\": %.4f, \"cl_pf\": %.6f, \"flo_ghz\": %.6f, \"fhi_ghz\": %.6f, "
                            "\"l1_nh\": %.6f, \"c1_pf\": %.6f, \"l2_nh\": %.6f, \"c2_pf\": %.6f, "
                            "\"bf_gamma_limit\": %.9f, \"bf_s11_limit_db\": %.6f, "
                            "\"max_gamma\": %.9f, \"max_s11_db\": %.6f, \"margin_db\": %.6f, "
                            "\"integral_ratio\": %.6f, \"i_meas_int\": %" PRId64 ", \"i_bf_int\": %" PRId64 ", "
                            "\"certified\": %d, \"status\": \"%s\"}\n",
                            bf->z0, bf->rl, bf->cl_f * 1e12, bf->f_lo_hz * 1e-9, bf->f_hi_hz * 1e-9,
                            bf->l1_nh, bf->c1_pf, bf->l2_nh, bf->c2_pf,
                            bf->bf_gamma_limit, bf->bf_s11_limit_db,
                            bf->max_gamma, bf->max_s11_db, bf->margin_db,
                            bf->integral_ratio, bf->i_meas_int, bf->i_bf_int,
                            bf->certified, bf->cert.status);
                    fclose(fp);
                }
            }
            printf("[EM BODE-FANO] Z0=%.0f R=%.0f C=%.2fpF [%.2f,%.2f]GHz -> max|S11|=%.2f dB >= BF=%.2f dB (I/Imax=%.4f, %s)\n",
                   bf->z0, bf->rl, bf->cl_f * 1e12, bf->f_lo_hz * 1e-9, bf->f_hi_hz * 1e-9,
                   bf->max_s11_db, bf->bf_s11_limit_db, bf->integral_ratio, bf->cert.status);
            free(bf);
            return 0;
        }
        if (strcmp(argv[2], "enforce-passivity") == 0 && argc >= 4) {
            const char *in_p = argv[3], *out_s2p = NULL, *cert_p = NULL, *lean_p = NULL;
            const char *mod_n = "CertEmKypEnforce", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--out") == 0) out_s2p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_snp_t snp;
            ele_em_kyp_res_t *kres = (ele_em_kyp_res_t *)calloc(1, sizeof(ele_em_kyp_res_t));
            if (kres == NULL) return 1;
            if (ele_snp_load(in_p, 0, &snp) != 0 ||
                ele_em_enforce_passivity(&snp, out_s2p, kres) != 0) {
                free(kres);
                return 1;
            }
            if (cert_p != NULL && ele_cert_write_json(cert_p, &kres->cert) != 0) { free(kres); return 1; }
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &kres->cert) != 0) { free(kres); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"nports\": %d, \"n_state\": %d, \"nfreq\": %d, "
                            "\"orig_sigma_max\": %.9f, \"orig_passive\": %d, "
                            "\"rep_sigma_max\": %.9f, \"rep_recip_err\": %.9e, \"rms_repair_err\": %.9f, "
                            "\"p_min_eig_int\": %" PRId64 ", \"kyp_mu_psd\": %" PRId64 ", "
                            "\"certified\": %d, \"status\": \"%s\"}\n",
                            kres->nports, kres->n_state, kres->nfreq,
                            kres->orig_sigma_max, kres->orig_passive,
                            kres->rep_sigma_max, kres->rep_recip_err, kres->rms_repair_err,
                            kres->p_min_eig_int, kres->kyp_mu_psd,
                            kres->certified, kres->cert.status);
                    fclose(fp);
                }
            }
            printf("[EM ENFORCE KYP] %s -> orig=%.4f -> rep=%.6f (rms=%.4f, P_mu=%" PRId64 ", KYP_mu=%" PRId64 ", %s)\n",
                   in_p, kres->orig_sigma_max, kres->rep_sigma_max, kres->rms_repair_err,
                   kres->p_min_eig_int, kres->kyp_mu_psd, kres->cert.status);
            free(kres);
            return 0;
        }
        if (strcmp(argv[2], "kyp-sim") == 0 && argc >= 4) {
            const char *in_p = argv[3], *cert_p = NULL, *json_p = NULL;
            double rho_load = 0.985;
            int n_steps = 400;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--rho") == 0) rho_load = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--steps") == 0) n_steps = atoi(argv[++i]);
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_snp_t snp;
            ele_em_kyp_sim_res_t *sim = (ele_em_kyp_sim_res_t *)calloc(1, sizeof(ele_em_kyp_sim_res_t));
            if (sim == NULL) return 1;
            if (ele_snp_load(in_p, 0, &snp) != 0 ||
                ele_em_kyp_transient_contrast(&snp, rho_load, n_steps, sim) != 0) {
                free(sim);
                return 1;
            }
            if (cert_p != NULL && ele_cert_write_json(cert_p, &sim->cert) != 0) { free(sim); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"nports\": %d, \"n_state\": %d, \"n_steps\": %d, \"rho_load\": %.4f, "
                            "\"orig_sigma_max\": %.9f, \"rep_sigma_max\": %.9f, "
                            "\"orig_peak_ratio\": %.9e, \"orig_final_ratio\": %.9e, "
                            "\"rep_peak_ratio\": %.9f, \"rep_final_ratio\": %.9e, "
                            "\"orig_blowup\": %d, \"rep_stable\": %d, \"rep_lyap_monotone\": %d, "
                            "\"certified\": %d, \"status\": \"%s\"}\n",
                            sim->nports, sim->n_state, sim->n_steps, sim->rho_load,
                            sim->orig_sigma_max, sim->rep_sigma_max,
                            sim->orig_peak_ratio, sim->orig_final_ratio,
                            sim->rep_peak_ratio, sim->rep_final_ratio,
                            sim->orig_blowup, sim->rep_stable, sim->rep_lyap_monotone,
                            sim->certified, sim->cert.status);
                    fclose(fp);
                }
            }
            printf("[EM KYP-SIM] %s (P=%d, rho=%.3f, N=%d) -> UNREP peak=%.3e fin=%.3e (blowup=%d) | REP peak=%.3f fin=%.2e (stable=%d, LyapMono=%d, %s)\n",
                   in_p, sim->nports, sim->rho_load, sim->n_steps,
                   sim->orig_peak_ratio, sim->orig_final_ratio, sim->orig_blowup,
                   sim->rep_peak_ratio, sim->rep_final_ratio, sim->rep_stable, sim->rep_lyap_monotone,
                   sim->cert.status);
            free(sim);
            return 0;
        }
    }

    if (strcmp(argv[1], "ckt") == 0 && argc >= 4) {
        if (strcmp(argv[2], "run") == 0) {
            const char *in_p = argv[3], *cert_p = NULL, *lean_p = NULL, *mod_n = "CertCkt", *json_p = NULL;
            int corners = 0;
            for (int i = 4; i < argc; ++i) {
                if (strcmp(argv[i], "--corners") == 0) corners = 1;
                else if (strcmp(argv[i], "--cert") == 0 && i + 1 < argc) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0 && i + 1 < argc) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0 && i + 1 < argc) json_p = argv[++i];
            }
            ele_ckt_netlist_t nl;
            ele_ckt_run_res_t res;
            if (ele_ckt_load(in_p, &nl) != 0 || ele_ckt_run_all(&nl, corners, &res) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &res.cert) != 0) return 1;
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &res.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"dc_kcl_res\": %.6e, \"bw_3db_hz\": %.6f, \"z_pdn_max\": %.6f, \"wc_vmin\": %.6f, \"wc_vmax\": %.6f, \"soa_pass\": %d}\n",
                            res.dc.max_kcl_res, res.ac.bw_3db_hz, res.ac.z_pdn_max_ohm, res.wc_vmin, res.wc_vmax, res.soa_pass);
                    fclose(fp);
                }
            }
            printf("[CKT RUN] %s -> DC_kcl=%.2e | BW=%.2f Hz | Zpdn=%.4f Ohm | wc=[%.2f, %.2f] V | SOA=%s\n",
                   in_p, res.dc.max_kcl_res, res.ac.bw_3db_hz, res.ac.z_pdn_max_ohm,
                   res.wc_vmin, res.wc_vmax, res.soa_pass ? "PASS" : "FAIL");
            return 0;
        }
        if (strcmp(argv[2], "si-eye") == 0) {
            const char *in_p = argv[3], *cert_p = NULL, *json_p = NULL;
            double rate = 4.0;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--rate-gbps") == 0) rate = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_ckt_netlist_t nl;
            ele_ckt_eye_res_t res;
            if (ele_ckt_load(in_p, &nl) != 0 || ele_ckt_si_eye(&nl, rate, &res) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &res.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"rate_gbps\": %.3f, \"eye_pda_v\": %.6f, \"eye_searu_v\": %.6f, \"eye_pass\": %d}\n",
                            res.bit_rate_gbps, res.eye_pda_v, res.eye_searu_v, res.eye_pass);
                    fclose(fp);
                }
            }
            printf("[CKT SI-EYE] %.2f Gbps | PDA=%.4f V | SEARU=%.4f V | pass=%d\n",
                   res.bit_rate_gbps, res.eye_pda_v, res.eye_searu_v, res.eye_pass);
            return 0;
        }
    }

    if (strcmp(argv[1], "motor") == 0 && argc >= 4 && strcmp(argv[2], "run") == 0) {
        const char *in_p = argv[3], *cert_p = NULL, *lean_p = NULL, *mod_n = "CertMotor", *json_p = NULL;
        double rpm = 3000.0, tq = 260.0;
        int do_sweep = 0;
        for (int i = 4; i < argc; ++i) {
            if (strcmp(argv[i], "--sweep") == 0) do_sweep = 1;
            else if (strcmp(argv[i], "--rpm") == 0 && i + 1 < argc) rpm = strtod(argv[++i], NULL);
            else if (strcmp(argv[i], "--torque") == 0 && i + 1 < argc) tq = strtod(argv[++i], NULL);
            else if (strcmp(argv[i], "--cert") == 0 && i + 1 < argc) cert_p = argv[++i];
            else if (strcmp(argv[i], "--emit-lean") == 0 && i + 1 < argc) lean_p = argv[++i];
            else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) mod_n = argv[++i];
            else if (strcmp(argv[i], "--json") == 0 && i + 1 < argc) json_p = argv[++i];
        }
        ele_motor_spec_t ms;
        ele_motor_op_res_t op;
        ele_motor_sweep_res_t sw;
        memset(&sw, 0, sizeof(sw));
        if (ele_motor_load(in_p, &ms) != 0 || ele_motor_solve_op(&ms, rpm, tq, &op) != 0) return 1;
        if (do_sweep && ele_motor_sweep_and_foc(&ms, tq, &sw) != 0) return 1;
        if (cert_p != NULL && ele_cert_write_json(cert_p, &op.cert) != 0) return 1;
        if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &op.cert) != 0) return 1;
        if (json_p != NULL) {
            FILE *fp = fopen(json_p, "w");
            if (fp != NULL) {
                fprintf(fp, "{\"rpm\": %.2f, \"id_a\": %.4f, \"iq_a\": %.4f, \"torque_nm\": %.4f, \"v_mag_v\": %.4f, \"eff\": %.6f, \"foc_ok\": %d}\n",
                        op.rpm, op.id_a, op.iq_a, op.torque_nm, op.v_mag_v, op.efficiency, sw.foc_converged);
                fclose(fp);
            }
        }
        printf("[MOTOR RUN] %.0f rpm -> Id=%.2f A, Iq=%.2f A, T=%.2f Nm, V=%.1f/%.1f V, eff=%.2f%%\n",
               op.rpm, op.id_a, op.iq_a, op.torque_nm, op.v_mag_v, op.v_max_v, 100.0 * op.efficiency);
        return 0;
    }

    if (strcmp(argv[1], "grid") == 0 && argc >= 4) {
        const char *sub = argv[2], *in_p = argv[3];
        if (strcmp(sub, "pf") == 0) {
            int do_n1 = 0;
            const char *json_p = NULL;
            for (int i = 4; i < argc; ++i) {
                if (strcmp(argv[i], "--n1") == 0) do_n1 = 1;
                else if (strcmp(argv[i], "--json") == 0 && i + 1 < argc) json_p = argv[++i];
            }
            ele_grid_t g;
            ele_grid_pf_res_t pf;
            ele_grid_n1_res_t n1;
            memset(&n1, 0, sizeof(n1));
            if (ele_grid_load(in_p, &g) != 0 || ele_grid_acpf(&g, -1, &pf) != 0) return 1;
            if (do_n1 && ele_grid_n1(&g, &n1) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"converged\": %d, \"mismatch\": %.6e, \"loss_mw\": %.6f, \"n1_secure\": %d}\n",
                            pf.converged, pf.max_mismatch_pu, pf.total_p_loss_mw, n1.n1_secure);
                    fclose(fp);
                }
            }
            printf("[GRID PF] %s -> conv=%d, mis=%.2e, Pgen=%.2f MW, Ploss=%.3f MW, N-1=%s\n",
                   in_p, pf.converged, pf.max_mismatch_pu, pf.total_p_gen_mw, pf.total_p_loss_mw,
                   do_n1 ? (n1.n1_secure ? "SECURE" : "WARN") : "N/A");
            return 0;
        }
        if (strcmp(sub, "opf") == 0) {
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertGridOpf", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_grid_t g;
            ele_grid_opf_res_t opf;
            if (ele_grid_load(in_p, &g) != 0 || ele_grid_acopf(&g, &opf) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &opf.cert) != 0) return 1;
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &opf.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_pct\": %.6f, \"mu_psd\": %" PRId64 "}\n",
                            opf.c_lb_cert, opf.c_ub_primal, opf.gap_pct, opf.mu_psd_int);
                    fclose(fp);
                }
            }
            printf("[GRID OPF] %s -> [%.2f, %.2f] $/h (gap=%.4f%%, mu_psd=%" PRId64 ")\n",
                   in_p, opf.c_lb_cert, opf.c_ub_primal, opf.gap_pct, opf.mu_psd_int);
            return 0;
        }
        if (strcmp(sub, "chordal") == 0) {
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertGridChordal", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            const char *use_cert = cert_p;
            char tmp_cert[512] = {0};
            if (use_cert == NULL && lean_p != NULL) {
                snprintf(tmp_cert, sizeof(tmp_cert), "%s.tmp_chordal.json", lean_p);
                use_cert = tmp_cert;
            }
            ele_grid_t *g = (ele_grid_t *)calloc(1, sizeof(ele_grid_t));
            ele_grid_chordal_res_t *ch = (ele_grid_chordal_res_t *)calloc(1, sizeof(ele_grid_chordal_res_t));
            if (g == NULL || ch == NULL) { free(g); free(ch); return 1; }
            if (ele_grid_load(in_p, g) != 0 || ele_grid_acopf_chordal(g, use_cert, ch) != 0) {
                free(g); free(ch);
                return 1;
            }
            if (lean_p != NULL && ele_cert_emit_chordal_lean(use_cert, lean_p, mod_n) != 0) {
                if (tmp_cert[0] != '\0') remove(tmp_cert);
                free(g); free(ch);
                return 1;
            }
            if (tmp_cert[0] != '\0') remove(tmp_cert);
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"n_bus\": %d, \"n_branch\": %d, \"n_cliques\": %d, \"max_clique_buses\": %d, "
                            "\"treewidth_ub\": %d, \"converged\": %d, \"c_lb_dual_raw\": %.6f, "
                            "\"c_lb_cert\": %.6f, \"c_ub_primal\": %.6f, \"gap_abs\": %.6f, "
                            "\"gap_pct\": %.6f, \"elapsed_sec\": %.6f, \"min_clique_mu_psd\": %" PRId64 ", "
                            "\"certified\": %d}\n",
                            ch->n_bus, ch->n_branch, ch->n_cliques, ch->max_clique_buses,
                            ch->treewidth_ub, ch->converged, ch->c_lb_dual_raw,
                            ch->c_lb_cert, ch->c_ub_primal, ch->gap_abs,
                            ch->gap_pct, ch->elapsed_sec, ch->min_clique_mu_psd, ch->certified);
                    fclose(fp);
                }
            }
            printf("[GRID CHORDAL] %s -> buses=%d cliques=%d tw<=%d [%.2f, %.2f] $/h (gap=%.4f%%, min_mu=%" PRId64 ", t=%.3fs, %s)\n",
                   in_p, ch->n_bus, ch->n_cliques, ch->treewidth_ub,
                   ch->c_lb_cert, ch->c_ub_primal, ch->gap_pct,
                   ch->min_clique_mu_psd, ch->elapsed_sec,
                   ch->certified ? "CERTIFIED" : "VETOED");
            free(g); free(ch);
            return 0;
        }
        if (strcmp(sub, "sos2") == 0) {
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertGridSos2", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_grid_t *g = (ele_grid_t *)calloc(1, sizeof(ele_grid_t));
            ele_grid_sos2_res_t *sr = (ele_grid_sos2_res_t *)calloc(1, sizeof(ele_grid_sos2_res_t));
            if (g == NULL || sr == NULL) { free(g); free(sr); return 1; }
            if (ele_grid_load(in_p, g) != 0 || ele_grid_acopf_sos2_cert(g, cert_p, sr) != 0) {
                free(g); free(sr);
                return 1;
            }
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &sr->cert) != 0) { free(g); free(sr); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"n_bus\": %d, \"d2_dim\": %d, \"n_monos4\": %d, \"converged\": %d, "
                            "\"shor1_lb_cert\": %.6f, \"sos2_lb_raw\": %.6f, \"sos2_lb_cert\": %.6f, "
                            "\"c_ub_primal\": %.6f, \"shor1_gap_pct\": %.6f, \"sos2_gap_pct\": %.6f, "
                            "\"improvement_abs\": %.6f, \"q0_mu_psd\": %" PRId64 "}\n",
                            sr->n_bus, sr->d2_dim, sr->n_monos4, sr->converged,
                            sr->shor1_lb_cert, sr->sos2_lb_raw, sr->sos2_lb_cert,
                            sr->c_ub_primal, sr->shor1_gap_pct, sr->sos2_gap_pct,
                            sr->improvement_abs, sr->q0_mu_psd);
                    fclose(fp);
                }
            }
            printf("[GRID SOS2] %s -> Shor1=%.2f (%.4f%%) -> SOS2=%.2f (%.4f%%) <= UB=%.2f $/h (+%.2f $/h, dim=%d)\n",
                   in_p, sr->shor1_lb_cert, sr->shor1_gap_pct,
                   sr->sos2_lb_cert, sr->sos2_gap_pct, sr->c_ub_primal,
                   sr->improvement_abs, sr->d2_dim);
            free(g); free(sr);
            return 0;
        }
        if (strcmp(sub, "observable") == 0) {
            const char *obs_str = "vm2:1", *cert_p = NULL, *json_p = NULL;
            double eps = 1.0;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--obs") == 0) obs_str = argv[++i];
                else if (strcmp(argv[i], "--eps") == 0) eps = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_grid_obs_kind_t okind = ELE_OBS_VM2;
            int tid = 1;
            if (strncmp(obs_str, "vm2:", 4) == 0) {
                okind = ELE_OBS_VM2; tid = atoi(obs_str + 4);
            } else if (strncmp(obs_str, "pflow:", 6) == 0) {
                okind = ELE_OBS_PFLOW; tid = atoi(obs_str + 6);
            } else {
                fprintf(stderr, "ERROR: --obs must be vm2:<bus_id> or pflow:<branch_idx>\n");
                return 1;
            }
            ele_grid_t *g = (ele_grid_t *)calloc(1, sizeof(ele_grid_t));
            ele_grid_obs_res_t *ores = (ele_grid_obs_res_t *)calloc(1, sizeof(ele_grid_obs_res_t));
            if (g == NULL || ores == NULL) { free(g); free(ores); return 1; }
            if (ele_grid_load(in_p, g) != 0 || ele_grid_observable(g, okind, tid, eps, ores) != 0) {
                free(g); free(ores);
                return 1;
            }
            if (cert_p != NULL && ele_cert_write_json(cert_p, &ores->cert) != 0) { free(g); free(ores); return 1; }
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp,
                            "{\"kind\": \"%s\", \"target_id\": %d, \"cost_cap\": %.6f, "
                            "\"o_primal\": %.9f, \"o_lb_cert\": %.9f, \"o_ub_cert\": %.9f, "
                            "\"width\": %.9f, \"c_ub_primal\": %.6f, \"c_lb_cert\": %.6f, "
                            "\"status\": \"%s\"}\n",
                            (okind == ELE_OBS_VM2) ? "vm2" : "pflow", tid, ores->cost_cap,
                            ores->o_primal, ores->o_lb_cert, ores->o_ub_cert,
                            ores->width, ores->c_ub_primal, ores->c_lb_cert, ores->cert.status);
                    fclose(fp);
                }
            }
            printf("[GRID OBS] %s (%s) -> primal=%.6f in [%.6f, %.6f] (width=%.6f, cap=%.2f, %s)\n",
                   in_p, obs_str, ores->o_primal, ores->o_lb_cert, ores->o_ub_cert,
                   ores->width, ores->cost_cap, ores->cert.status);
            free(g); free(ores);
            return 0;
        }
        if (strcmp(sub, "emt") == 0) {
            int fault_bus = 7, trip_br = -1;
            double tc = 0.12;
            const char *cert_p = NULL, *lean_p = NULL, *mod_n = "CertGridEmt", *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--fault-bus") == 0) fault_bus = atoi(argv[++i]);
                else if (strcmp(argv[i], "--t-clear") == 0) tc = strtod(argv[++i], NULL);
                else if (strcmp(argv[i], "--trip-branch") == 0) trip_br = atoi(argv[++i]);
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--emit-lean") == 0) lean_p = argv[++i];
                else if (strcmp(argv[i], "--name") == 0) mod_n = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            ele_grid_t g;
            ele_grid_emt_res_t emt;
            if (ele_grid_load(in_p, &g) != 0 || ele_grid_emt(&g, fault_bus, tc, trip_br, &emt) != 0) return 1;
            if (cert_p != NULL && ele_cert_write_json(cert_p, &emt.cert) != 0) return 1;
            if (lean_p != NULL && ele_cert_emit_lean(lean_p, mod_n, &emt.cert) != 0) return 1;
            if (json_p != NULL) {
                FILE *fp = fopen(json_p, "w");
                if (fp != NULL) {
                    fprintf(fp, "{\"stable\": %d, \"v_post\": %.6f, \"v_crit\": %.6f, \"cct_sec\": %.6f, \"max_gfm_i_pu\": %.6f}\n",
                            emt.stable, emt.v_post, emt.v_crit, emt.cct_sec, emt.max_gfm_i_pu);
                    fclose(fp);
                }
            }
            printf("[GRID EMT] %s fault@bus%d tc=%.3fs -> stable=%d, V=%.4f<%.4f, CCT=%.3fs\n",
                   in_p, fault_bus, tc, emt.stable, emt.v_post, emt.v_crit, emt.cct_sec);
            return 0;
        }
    }

    if (strcmp(argv[1], "lab") == 0 && argc >= 4) {
        const char *sub = argv[2], *in_p = argv[3];
        if (strcmp(sub, "validate-thermal") == 0) {
            const char *sp_p = NULL, *cert_p = NULL, *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--sp") == 0) sp_p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            return cmd_lab_thermal(in_p, sp_p, cert_p, json_p);
        }
        if (strcmp(sub, "validate-csi") == 0) {
            const char *s2p_p = NULL, *cert_p = NULL, *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--s2p") == 0) s2p_p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            return cmd_lab_csi(in_p, s2p_p, cert_p, json_p);
        }
        if (strcmp(sub, "validate-power") == 0) {
            const char *sp_p = NULL, *cert_p = NULL, *json_p = NULL;
            for (int i = 4; i + 1 < argc; ++i) {
                if (strcmp(argv[i], "--sp") == 0) sp_p = argv[++i];
                else if (strcmp(argv[i], "--cert") == 0) cert_p = argv[++i];
                else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
            }
            return cmd_lab_power(in_p, sp_p, cert_p, json_p);
        }
    }

    if (strcmp(argv[1], "field-to-circuit") == 0) {
        const char *peec_p = NULL, *sp_p = NULL, *mot_p = NULL, *out_d = "build", *json_p = NULL;
        for (int i = 2; i + 1 < argc; ++i) {
            if (strcmp(argv[i], "--peec") == 0) peec_p = argv[++i];
            else if (strcmp(argv[i], "--sp") == 0) sp_p = argv[++i];
            else if (strcmp(argv[i], "--motor") == 0) mot_p = argv[++i];
            else if (strcmp(argv[i], "--out-dir") == 0) out_d = argv[++i];
            else if (strcmp(argv[i], "--json") == 0) json_p = argv[++i];
        }
        return run_field_to_circuit(peec_p, sp_p, mot_p, out_d, json_p);
    }

    if (strcmp(argv[1], "bench") == 0) {
        const char *dir = "benchmarks", *out_j = "build/ele_bench.json", *cert_d = "build/certs";
        for (int i = 2; i + 1 < argc; ++i) {
            if (strcmp(argv[i], "--dir") == 0) dir = argv[++i];
            else if (strcmp(argv[i], "--out") == 0) out_j = argv[++i];
            else if (strcmp(argv[i], "--cert-dir") == 0) cert_d = argv[++i];
        }
        if (!file_exists_readable(dir) && 0) return 1;
        return (run_bench(dir, out_j, cert_d) == 0) ? 0 : 1;
    }

    if (strcmp(argv[1], "atlas") == 0) {
        const char *dir = "benchmarks", *out_j = "build/ele_atlas.json", *cert_d = "build/certs";
        for (int i = 2; i + 1 < argc; ++i) {
            if (strcmp(argv[i], "--dir") == 0) dir = argv[++i];
            else if (strcmp(argv[i], "--out") == 0) out_j = argv[++i];
            else if (strcmp(argv[i], "--cert-dir") == 0) cert_d = argv[++i];
        }
        static const struct { const char *tag; const char *rel; int do_shor1; int do_sos2; } cases[13] = {
            {"ieee3_test", "ee/ieee3_test.grid", 1, 1},
            {"ieee9_wscc", "ee/ieee9_wscc.grid", 1, 0},
            {"wb2", "ee/wb2.grid", 1, 1},
            {"wb3", "ee/wb3.grid", 1, 1},
            {"wb5", "ee/wb5.grid", 1, 1},
            {"case3_lmbd", "ee/pglib/pglib_opf_case3_lmbd.m", 1, 1},
            {"case5_pjm", "ee/pglib/pglib_opf_case5_pjm.m", 1, 1},
            {"case14_ieee", "ee/pglib/pglib_opf_case14_ieee.m", 1, 0},
            {"case30_ieee", "ee/pglib/pglib_opf_case30_ieee.m", 1, 0},
            {"case30_api", "ee/pglib/pglib_opf_case30_ieee__api.m", 1, 0},
            {"case30_sad", "ee/pglib/pglib_opf_case30_ieee__sad.m", 1, 0},
            {"case57_ieee", "ee/pglib/pglib_opf_case57_ieee.m", 0, 0},
            {"case118_ieee", "ee/pglib/pglib_opf_case118_ieee.m", 0, 0}
        };
        FILE *af = fopen(out_j, "w");
        if (af == NULL) return 1;
        fprintf(af, "{\n  \"version\": \"%s\",\n  \"grids\": [\n", ELE_VERSION_STRING);
        for (int c = 0; c < 13; ++c) {
            char gp[512], cp[512];
            snprintf(gp, sizeof(gp), "%s/%s", dir, cases[c].rel);
            snprintf(cp, sizeof(cp), "%s/chordal_%s.elecert.json", cert_d, cases[c].tag);
            ele_grid_t *g = (ele_grid_t *)calloc(1, sizeof(ele_grid_t));
            ele_grid_chordal_res_t *ch = (ele_grid_chordal_res_t *)calloc(1, sizeof(ele_grid_chordal_res_t));
            ele_grid_opf_res_t *opf = (ele_grid_opf_res_t *)calloc(1, sizeof(ele_grid_opf_res_t));
            ele_grid_sos2_res_t *sr = (ele_grid_sos2_res_t *)calloc(1, sizeof(ele_grid_sos2_res_t));
            if (!g || !ch || !opf || !sr) { free(g); free(ch); free(opf); free(sr); fclose(af); return 1; }
            if (ele_grid_load(gp, g) != 0 || ele_grid_acopf_chordal(g, cp, ch) != 0) {
                free(g); free(ch); free(opf); free(sr); fclose(af); return 1;
            }
            if (cases[c].do_shor1) {
                if (ele_grid_acopf(g, opf) != 0) opf->c_lb_cert = ch->c_lb_cert;
            }
            if (cases[c].do_sos2) {
                char sp[512];
                snprintf(sp, sizeof(sp), "%s/sos2_%s.elecert.json", cert_d, cases[c].tag);
                if (ele_grid_acopf_sos2_cert(g, sp, sr) != 0) { free(g); free(ch); free(opf); free(sr); fclose(af); return 1; }
            }
            fprintf(af,
                    "    {\"tag\": \"%s\", \"n_bus\": %d, \"n_cliques\": %d, \"treewidth_ub\": %d, "
                    "\"shor1_gap_pct\": %.6f, \"chordal_lb_cert\": %.6f, \"chordal_ub\": %.6f, "
                    "\"chordal_gap_pct\": %.6f, \"sos2_lb_cert\": %.6f, \"sos2_gap_pct\": %.6f, "
                    "\"chordal_certified\": %d, \"elapsed_sec\": %.6f}%s\n",
                    cases[c].tag, ch->n_bus, ch->n_cliques, ch->treewidth_ub,
                    cases[c].do_shor1 ? opf->gap_pct : -1.0,
                    ch->c_lb_cert, ch->c_ub_primal, ch->gap_pct,
                    cases[c].do_sos2 ? sr->sos2_lb_cert : -1.0,
                    cases[c].do_sos2 ? sr->sos2_gap_pct : -1.0,
                    ch->certified, ch->elapsed_sec, (c + 1 < 13) ? "," : "");
            free(g); free(ch); free(opf); free(sr);
        }
        fprintf(af, "  ],\n");

        /* Observable envelopes */
        char obs_wb2_p[512], obs_wb2_c[512], obs_c3_p[512], obs_c3_c[512];
        snprintf(obs_wb2_p, sizeof(obs_wb2_p), "%s/ee/wb2.grid", dir);
        snprintf(obs_wb2_c, sizeof(obs_wb2_c), "%s/obs_wb2_vm2.elecert.json", cert_d);
        snprintf(obs_c3_p, sizeof(obs_c3_p), "%s/ee/pglib/pglib_opf_case3_lmbd.m", dir);
        snprintf(obs_c3_c, sizeof(obs_c3_c), "%s/obs_case3_pflow0.elecert.json", cert_d);
        ele_grid_t *gobs = (ele_grid_t *)calloc(1, sizeof(ele_grid_t));
        ele_grid_obs_res_t *owb2 = (ele_grid_obs_res_t *)calloc(1, sizeof(ele_grid_obs_res_t));
        ele_grid_obs_res_t *oc3 = (ele_grid_obs_res_t *)calloc(1, sizeof(ele_grid_obs_res_t));
        if (!gobs || !owb2 || !oc3 ||
            ele_grid_load(obs_wb2_p, gobs) != 0 || ele_grid_observable(gobs, ELE_OBS_VM2, 2, 5.0, owb2) != 0 ||
            ele_cert_write_json(obs_wb2_c, &owb2->cert) != 0 ||
            ele_grid_load(obs_c3_p, gobs) != 0 || ele_grid_observable(gobs, ELE_OBS_PFLOW, 0, 5.0, oc3) != 0 ||
            ele_cert_write_json(obs_c3_c, &oc3->cert) != 0) {
            free(gobs); free(owb2); free(oc3); fclose(af); return 1;
        }
        fprintf(af,
                "  \"observables\": [\n"
                "    {\"grid\": \"wb2\", \"obs\": \"vm2:2\", \"o_primal\": %.6f, \"o_lb\": %.6f, \"o_ub\": %.6f, \"width\": %.6f, \"status\": \"%s\"},\n"
                "    {\"grid\": \"case3_lmbd\", \"obs\": \"pflow:0\", \"o_primal\": %.6f, \"o_lb\": %.6f, \"o_ub\": %.6f, \"width\": %.6f, \"status\": \"%s\"}\n"
                "  ],\n",
                owb2->o_primal, owb2->o_lb_cert, owb2->o_ub_cert, owb2->width, owb2->cert.status,
                oc3->o_primal, oc3->o_lb_cert, oc3->o_ub_cert, oc3->width, oc3->cert.status);
        free(gobs); free(owb2); free(oc3);

        /* Maxwell EM bound + KYP repair */
        ele_em_bound_res_t *eb = (ele_em_bound_res_t *)calloc(1, sizeof(ele_em_bound_res_t));
        ele_em_kyp_res_t *ek = (ele_em_kyp_res_t *)calloc(1, sizeof(ele_em_kyp_res_t));
        ele_snp_t *snp_np = (ele_snp_t *)calloc(1, sizeof(ele_snp_t));
        char em_c[512], np_p[512], rep_s2p[512], kyp_c[512];
        snprintf(em_c, sizeof(em_c), "%s/em_maxwell_bound.elecert.json", cert_d);
        snprintf(np_p, sizeof(np_p), "%s/em/nonpassive_ai_surrogate.s2p", dir);
        snprintf(rep_s2p, sizeof(rep_s2p), "%s/repaired_ai_surrogate.s2p", cert_d);
        snprintf(kyp_c, sizeof(kyp_c), "%s/em_kyp_repaired.elecert.json", cert_d);
        if (!eb || !ek || !snp_np ||
            ele_em_maxwell_bound(2.0, 10.0, 0.0, 16, eb) != 0 || ele_cert_write_json(em_c, &eb->cert) != 0 ||
            ele_snp_load(np_p, 2, snp_np) != 0 || ele_em_enforce_passivity(snp_np, rep_s2p, ek) != 0 ||
            ele_cert_write_json(kyp_c, &ek->cert) != 0) {
            free(eb); free(ek); free(snp_np); fclose(af); return 1;
        }
        fprintf(af,
                "  \"em_bound\": {\"kl\": %.2f, \"zeta\": %.2f, \"n_cells\": %d, \"p_global_ub\": %.6f, "
                "\"p_local_ub_cert\": %.6f, \"p_searu_lb\": %.6f, \"tightening_ratio\": %.6f, "
                "\"bf_s11_limit_db\": %.4f, \"status\": \"%s\"},\n",
                eb->kl, eb->zeta, eb->n_cells, eb->p_global_ub, eb->p_local_ub_cert,
                eb->p_searu_lb, eb->tightening_ratio, eb->bf_s11_limit_db, eb->cert.status);
        fprintf(af,
                "  \"kyp_enforce\": {\"orig_sigma_max\": %.6f, \"rep_sigma_max\": %.6f, "
                "\"rms_repair_err\": %.6f, \"p_min_eig_int\": %" PRId64 ", \"kyp_mu_psd\": %" PRId64 ", "
                "\"certified\": %d, \"status\": \"%s\"}\n",
                ek->orig_sigma_max, ek->rep_sigma_max, ek->rms_repair_err,
                ek->p_min_eig_int, ek->kyp_mu_psd, ek->certified, ek->cert.status);
        fprintf(af, "}\n");
        fclose(af);
        free(eb); free(ek); free(snp_np);
        printf("  [ATLAS] ELE v%s Frontier Atlas -> %s\n", ELE_VERSION_STRING, out_j);
        return 0;
    }

    print_usage();
    return 1;
}
