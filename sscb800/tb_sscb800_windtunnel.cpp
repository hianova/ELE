// tb_sscb800_windtunnel.cpp — 7-Scenario AI 800V DC Rack SiC SSCB Verilator Wind-Tunnel
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "Vele_sscb800_ip.h"
#include "verilated.h"

namespace {

constexpr double kSampleNs      = 50.0;      // 20 MSPS ADC sampling (100 MHz FPGA clk / 5)
constexpr int kClksPerSample    = 5;         // 5 sys-clk cycles per 50 ns ADC sample
constexpr double kAmpsPerLsb    = 0.25;      // 16-bit full-scale up to 16,383.75 A
constexpr uint16_t kNomLsb      = 1600;      // 400 A
constexpr uint16_t kHardMax     = 3600;      // 900 A
constexpr uint16_t kDidtThr     = 20;        // 100 A/us (20 LSB per 50 ns)
constexpr uint16_t kConcTol     = 1;         // concave tolerance
constexpr uint8_t kConfirmLen   = 3;         // 3 consecutive samples (150 ns)
constexpr uint32_t kI2tLim      = 500000000U;// allows 1.0 mF hot-swap surge, trips sustained overload

inline uint16_t amps_to_lsb(double amps) {
    if (amps <= 0.0) {
        return 0U;
    }
    double v = std::llround(amps / kAmpsPerLsb);
    return (v > 65535.0) ? 65535U : static_cast<uint16_t>(v);
}

inline double lsb_to_amps(uint16_t lsb) {
    return static_cast<double>(lsb) * kAmpsPerLsb;
}

class DutHarness {
  public:
    Vele_sscb800_ip *dut;

    DutHarness() { dut = new Vele_sscb800_ip; }
    ~DutHarness() {
        dut->final();
        delete dut;
    }

    void eval() { dut->eval(); }

    void tick() {
        dut->clk = 0;
        dut->eval();
        dut->clk = 1;
        dut->eval();
    }

    void reset_and_config(bool emi_guard_en = true) {
        dut->rst_n          = 0;
        dut->sample_valid_i = 0;
        dut->i_raw_i        = 0;
        dut->desat_fault_i  = 0;
        dut->csr_wr_i       = 0;
        dut->csr_rd_i       = 0;
        dut->csr_addr_i     = 0;
        dut->csr_wdata_i    = 0;
        for (int i = 0; i < 4; ++i) {
            tick();
        }
        dut->rst_n = 1;
        tick();

        write_csr(0x08, (static_cast<uint32_t>(kHardMax) << 16) | kNomLsb);
        write_csr(0x0C, (static_cast<uint32_t>(kConcTol) << 16) | kDidtThr);
        write_csr(0x10, (4U << 19) | (800U << 3) | kConfirmLen);
        write_csr(0x14, kI2tLim);
        write_csr(0x04, (emi_guard_en ? 2U : 0U) | 1U); // armed=1, emi_guard_en
    }

    void write_csr(uint8_t addr, uint32_t wdata) {
        dut->csr_wr_i    = 1;
        dut->csr_addr_i  = addr & 0x3FU;
        dut->csr_wdata_i = wdata;
        tick();
        dut->csr_wr_i    = 0;
        dut->csr_wdata_i = 0;
    }

    uint32_t read_csr(uint8_t addr) {
        dut->csr_rd_i   = 1;
        dut->csr_addr_i = addr & 0x3FU;
        dut->eval();
        uint32_t v = dut->csr_rdata_o;
        dut->csr_rd_i = 0;
        return v;
    }

    void feed_adc(uint16_t raw_lsb, uint8_t desat = 0) {
        dut->sample_valid_i = 1;
        dut->i_raw_i        = raw_lsb;
        dut->desat_fault_i  = desat;
        tick();
        dut->sample_valid_i = 0;
        dut->desat_fault_i  = 0;
        for (int i = 1; i < kClksPerSample; ++i) {
            tick();
        }
    }
};

struct RecordedSample {
    uint16_t raw;
    uint16_t corr;
};

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const char *json_path = nullptr;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--json") == 0) {
            json_path = argv[i + 1];
        }
    }

    DutHarness h;

    // ------------------------------------------------------------------------
    // Verify IP_ID CSR
    // ------------------------------------------------------------------------
    h.reset_and_config(true);
    uint32_t ip_id = h.read_csr(0x00);
    if (ip_id != 0xE1E80001U) {
        std::fprintf(stderr, "FAIL: unexpected IP_ID 0x%08x\n", ip_id);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 1 + 7: Bolted 800V DC Short-Circuit (200 A/us, L=4 uH) + Blackbox
    // ------------------------------------------------------------------------
    std::vector<RecordedSample> trace1;
    uint16_t m_d1 = 0, m_d2 = 0;
    size_t m_cnt = 0;
    auto push_trace = [&](uint16_t raw_lsb) {
        uint16_t filt = raw_lsb;
        if (m_cnt >= 2U) {
            uint16_t lo = (m_d1 < m_d2) ? m_d1 : m_d2;
            uint16_t hi = (m_d1 < m_d2) ? m_d2 : m_d1;
            filt = (raw_lsb < lo) ? lo : ((raw_lsb > hi) ? hi : raw_lsb);
        }
        m_d2 = m_d1;
        m_d1 = raw_lsb;
        m_cnt += 1U;
        trace1.push_back({raw_lsb, filt});
    };
    for (int i = 0; i < 40; ++i) {
        uint16_t lsb = amps_to_lsb(380.0); // steady 380 A nominal AI rack load
        push_trace(lsb);
        h.feed_adc(lsb);
        if (!h.dut->gate_en_o || h.dut->trip_latched_o) {
            std::fprintf(stderr, "FAIL Scen1: false trip during steady 380A warmup!\n");
            return 1;
        }
    }

    int s1_trip_tick      = -1;
    int s1_over_nom_tick  = -1;
    double s1_peak_amps   = 0.0;
    double s1_i2t_let     = 0.0;
    double ref_i2t_let    = 0.0;
    double ref_peak_amps  = 0.0; // computed prior to 900 A + 400 ns comparator trip
    int ref_trip_tick     = -1;

    for (int t = 0; t < 500; ++t) {
        double ts     = static_cast<double>(t) * (kSampleNs * 1e-9);
        double i_amps = 380.0 + (800.0 / 0.02) * (1.0 - std::exp(-0.02 * ts / 4e-6));
        uint16_t lsb  = amps_to_lsb(i_amps);
        if (s1_over_nom_tick < 0 && lsb > kNomLsb) {
            s1_over_nom_tick = t;
        }
        if (ref_trip_tick < 0 && i_amps >= 900.0) {
            ref_trip_tick = t + 8; // +400 ns comparator + blanking delay (8 samples @ 50 ns)
        }
        if (ref_trip_tick < 0 || t < ref_trip_tick) {
            ref_i2t_let += i_amps * i_amps * (kSampleNs * 1e-9);
            if (i_amps > ref_peak_amps) {
                ref_peak_amps = i_amps;
            }
        }

        if (s1_trip_tick < 0) {
            push_trace(lsb);
            h.feed_adc(lsb);
            s1_i2t_let += i_amps * i_amps * (kSampleNs * 1e-9);
            if (i_amps > s1_peak_amps) {
                s1_peak_amps = i_amps;
            }
            if (h.dut->trip_latched_o) {
                s1_trip_tick = t;
            }
        }
        if (s1_trip_tick >= 0 && ref_trip_tick >= 0 && t >= ref_trip_tick) {
            break;
        }
    }

    if (s1_trip_tick < 0 || h.dut->trip_cause_o != 0x2U) {
        std::fprintf(stderr, "FAIL Scen1: expected 2-Jet trip (cause=2), got tick=%d cause=%u\n",
                     s1_trip_tick, h.dut->trip_cause_o);
        return 1;
    }

    // Feed 10 bogus post-trip high spikes; verify Blackbox remains 100% frozen!
    for (int i = 0; i < 10; ++i) {
        h.feed_adc(5000U);
        if (h.dut->gate_en_o || !h.dut->trip_latched_o) {
            std::fprintf(stderr, "FAIL Scen1: gate not latched off post-trip!\n");
            return 1;
        }
    }

    // Scenario 7: Read all 32 frozen Blackbox entries via CSR
    uint32_t telem = h.read_csr(0x1C);
    uint32_t bb_cnt = (telem >> 24) & 0x3FU;
    if (bb_cnt != 32U || trace1.size() < 32U) {
        std::fprintf(stderr, "FAIL Scen7: expected 32 BB entries, got %u\n", bb_cnt);
        return 1;
    }
    const size_t base_off = trace1.size() - 32U;
    for (uint32_t idx = 0; idx < 32U; ++idx) {
        h.write_csr(0x24, idx);
        uint32_t word = h.read_csr(0x28);
        uint16_t raw_r = static_cast<uint16_t>((word >> 16) & 0xFFFFU);
        uint16_t cor_r = static_cast<uint16_t>(word & 0xFFFFU);
        if (raw_r != trace1[base_off + idx].raw || cor_r != trace1[base_off + idx].corr) {
            std::fprintf(stderr,
                         "FAIL Scen7: BB slot %u mismatch: got (%u,%u) vs exp (%u,%u)\n",
                         idx, raw_r, cor_r, trace1[base_off + idx].raw, trace1[base_off + idx].corr);
            return 1;
        }
    }

    double s1_trip_ns    = static_cast<double>(s1_trip_tick - s1_over_nom_tick) * kSampleNs;
    double let_ratio     = ref_i2t_let / ((s1_i2t_let > 1e-9) ? s1_i2t_let : 1e-9);
    if (s1_trip_ns > 250.0 || let_ratio < 10.0) {
        std::fprintf(stderr, "FAIL Scen1 timing/let-through: dt=%.1f ns ratio=%.2fx\n",
                     s1_trip_ns, let_ratio);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 2: Capacitive Hot-Swap Inrush (520V -> 800V RLC concave surge over 320A load)
    //   C = 1.0 mF, R = 1.0 ohm, L = 0.40 uH, dV = 280 V -> peak ~599 A, slope > 600 A/us
    //   Naive fixed-dI/dt comparator FALSE-TRIPS; ele_sscb800_ip rides through via d^2I/dt^2 < 0!
    // ------------------------------------------------------------------------
    h.reset_and_config(true);
    const double R = 1.0, L = 0.40e-6, C = 1.0e-3, dV = 280.0, i0 = 320.0;
    double alpha = R / (2.0 * L);
    double w0_sq = 1.0 / (L * C);
    double wd    = std::sqrt(alpha * alpha - w0_sq);
    double s1    = -alpha + wd;
    double s2    = -alpha - wd;
    double c1    = (dV / L) / (s1 - s2);
    double c2    = -c1;

    uint16_t m2_d1 = 0, m2_d2 = 0, n_i_prev = 0;
    size_t m2_cnt = 0;
    uint8_t n_warm = 0, n_persist = 0;
    bool n_tripped            = false;
    int s2_naive_over_samples = 0;
    int s2_naive_false_trips  = 0;
    auto step_naive = [&](uint16_t raw_lsb, bool count_surge) {
        uint16_t corr = raw_lsb;
        if (m2_cnt >= 2U) {
            uint16_t lo = (m2_d1 < m2_d2) ? m2_d1 : m2_d2;
            uint16_t hi = (m2_d1 < m2_d2) ? m2_d2 : m2_d1;
            corr = (raw_lsb < lo) ? lo : ((raw_lsb > hi) ? hi : raw_lsb);
        }
        m2_d2 = m2_d1;
        m2_d1 = raw_lsb;
        m2_cnt += 1U;

        int32_t j1 = static_cast<int32_t>(corr) - static_cast<int32_t>(n_i_prev);
        bool hit   = (n_warm == 2U) && (corr > kNomLsb) && (j1 >= static_cast<int32_t>(kDidtThr));
        if (count_surge && hit) {
            s2_naive_over_samples += 1;
        }
        n_persist = hit ? static_cast<uint8_t>(n_persist + 1U) : 0U;
        if (count_surge && !n_tripped && (corr >= kHardMax || (hit && n_persist >= kConfirmLen))) {
            n_tripped = true;
            s2_naive_false_trips += 1;
        }
        n_i_prev = corr;
        if (n_warm < 2U) {
            n_warm += 1U;
        }
    };

    for (int i = 0; i < 40; ++i) {
        uint16_t lsb = amps_to_lsb(i0);
        step_naive(lsb, false);
        h.feed_adc(lsb);
        if (!h.dut->gate_en_o || h.dut->trip_latched_o) {
            std::fprintf(stderr, "FAIL Scen2: false trip during steady 320A warmup!\n");
            return 1;
        }
    }

    const int s2_total_samples = 30000; // 1.5 ms full charge until current returns < 400 A
    double s2_peak_amps = 0.0;
    int s2_false_trips  = 0;
    double s2_max_slope = 0.0;
    double prev_i       = i0;
    for (int t = 0; t < s2_total_samples; ++t) {
        double ts     = static_cast<double>(t) * (kSampleNs * 1e-9);
        double i_amps = i0 + c1 * std::exp(s1 * ts) + c2 * std::exp(s2 * ts);
        if (t > 0) {
            double sl = (i_amps - prev_i) / (kSampleNs * 1e-3); // A/us
            if (sl > s2_max_slope) {
                s2_max_slope = sl;
            }
        }
        prev_i = i_amps;
        if (i_amps > s2_peak_amps) {
            s2_peak_amps = i_amps;
        }
        uint16_t raw_lsb = amps_to_lsb(i_amps);
        step_naive(raw_lsb, true);
        h.feed_adc(raw_lsb);
        if (h.dut->trip_latched_o) {
            s2_false_trips += 1;
        }
    }
    if (s2_false_trips != 0 || s2_naive_false_trips < 1 || s2_naive_over_samples < kConfirmLen) {
        std::fprintf(stderr,
                     "FAIL Scen2: ele_false=%d (want 0), naive_false=%d (want >=1), naive_over=%d\n",
                     s2_false_trips, s2_naive_false_trips, s2_naive_over_samples);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 3: 50 V/ns SiC Switching EMI Spikes (50 isolated +1800 A bursts)
    // ------------------------------------------------------------------------
    int s3_guarded_trips = 0;
    uint32_t s3_rej_log  = 0U;
    h.reset_and_config(true);
    for (int k = 0; k < 50; ++k) {
        for (int w = 0; w < 6; ++w) {
            h.feed_adc(amps_to_lsb(300.0));
        }
        h.feed_adc(amps_to_lsb(1800.0)); // huge 1-sample EMI switching spike
        for (int w = 0; w < 3; ++w) {
            h.feed_adc(amps_to_lsb(300.0));
        }
        if (h.dut->trip_latched_o) {
            s3_guarded_trips += 1;
        }
    }
    s3_rej_log = (h.read_csr(0x18) >> 8) & 0xFFU;

    int s3_unguarded_trips = 0;
    for (int k = 0; k < 50; ++k) {
        h.reset_and_config(false); // emi_guard_en = 0
        for (int w = 0; w < 6; ++w) {
            h.feed_adc(amps_to_lsb(300.0));
        }
        h.feed_adc(amps_to_lsb(1800.0));
        if (h.dut->trip_latched_o) {
            s3_unguarded_trips += 1;
        }
    }
    if (s3_guarded_trips != 0 || s3_rej_log != 50U || s3_unguarded_trips != 50) {
        std::fprintf(stderr, "FAIL Scen3 EMI: guarded=%d rej=%u unguarded=%d\n",
                     s3_guarded_trips, s3_rej_log, s3_unguarded_trips);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 4: 85C Hall Sensor Thermal Offset Drift (+35 A = 140 LSB) + AutoZero
    // ------------------------------------------------------------------------
    const uint16_t drift_lsb = amps_to_lsb(35.0);
    h.reset_and_config(true);
    // Trip first under fault with drifted sensor reading +35 A
    for (int w = 0; w < 5; ++w) {
        h.feed_adc(amps_to_lsb(200.0) + drift_lsb);
    }
    h.feed_adc(amps_to_lsb(950.0) + drift_lsb);
    h.feed_adc(amps_to_lsb(950.0) + drift_lsb);
    if (!h.dut->trip_latched_o) {
        std::fprintf(stderr, "FAIL Scen4: setup trip failed\n");
        return 1;
    }
    // Attempt rearm at true zero bus current (sensor shows +35 A) -> MUST be rejected!
    h.dut->i_raw_i = drift_lsb;
    h.write_csr(0x04, (1U << 3) | 2U | 1U); // rearm=1, emi=1, armed=1
    bool uncal_rearm_rejected = (h.dut->trip_latched_o != 0);

    // Disarm, pulse AUTOZERO while true bus current = 0 A (sensor reads +35 A), re-enable, rearm
    h.write_csr(0x04, 2U); // armed=0
    h.dut->i_raw_i = drift_lsb;
    h.write_csr(0x04, (1U << 2) | 2U); // autozero_pulse while disarmed
    uint16_t captured_off = static_cast<uint16_t>((h.read_csr(0x18) >> 16) & 0xFFFFU);
    h.write_csr(0x04, (1U << 3) | 2U | 1U); // rearm=1
    bool cal_rearm_accepted = (h.dut->trip_latched_o == 0);
    if (!uncal_rearm_rejected || !cal_rearm_accepted || captured_off != drift_lsb) {
        std::fprintf(stderr, "FAIL Scen4 AutoZero: rej=%d acc=%d off=%u\n",
                     uncal_rearm_rejected, cal_rearm_accepted, captured_off);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 5: Sustained 1.6x Overload (640 A) Adiabatic I^2*t Thermal Trip
    // ------------------------------------------------------------------------
    h.reset_and_config(true);
    h.write_csr(0x14, 2000000U); // lower I^2*t budget for fast clean thermal test
    int s5_trip_tick = -1;
    uint16_t ov_lsb  = amps_to_lsb(640.0); // 2560 LSB (over 400A nom=1600 -> over=960 LSB)
    for (int t = 0; t < 200; ++t) {
        h.feed_adc(ov_lsb);
        if (h.dut->trip_latched_o) {
            s5_trip_tick = t;
            break;
        }
    }
    if (s5_trip_tick < 0 || h.dut->trip_cause_o != 0x4U) {
        std::fprintf(stderr, "FAIL Scen5 I2t: tick=%d cause=%u\n", s5_trip_tick, h.dut->trip_cause_o);
        return 1;
    }

    // ------------------------------------------------------------------------
    // Scenario 6: Hardware SiC Gate-Driver DESAT 0-Cycle Combinational Kill
    // ------------------------------------------------------------------------
    h.reset_and_config(true);
    for (int w = 0; w < 5; ++w) {
        h.feed_adc(amps_to_lsb(250.0));
    }
    if (!h.dut->gate_en_o) {
        std::fprintf(stderr, "FAIL Scen6: gate_en_o not high before DESAT\n");
        return 1;
    }
    h.dut->desat_fault_i = 1;
    h.eval(); // PURE COMBINATIONAL (0 clock cycles elapsed!)
    bool zero_cyc_killed = (h.dut->gate_en_o == 0);
    h.tick();
    h.dut->desat_fault_i = 0;
    h.eval();
    bool desat_sticky = (h.dut->desat_latched_o != 0 && h.dut->trip_latched_o != 0 && h.dut->gate_en_o == 0);
    if (!zero_cyc_killed || !desat_sticky) {
        std::fprintf(stderr, "FAIL Scen6 DESAT: zero_cyc=%d sticky=%d\n", zero_cyc_killed, desat_sticky);
        return 1;
    }

    std::printf("\n=== AI 800V DC SiC SSCB IP Wind-Tunnel Summary (100 MHz Clk / 20 MSPS 50 ns ADC) ===\n");
    std::printf("  [PASS Scen 1] Bolted Short (200 A/us): trip=%.1f ns (I_peak=%.1f A vs %.1f A base, %.1fx lower I^2*t)\n",
                s1_trip_ns, s1_peak_amps, ref_peak_amps, let_ratio);
    std::printf("  [PASS Scen 2] Capacitive Hot-Swap (1.0mF, 280V step over 320A): peak=%.1f A, slope_max=%.1f A/us, naive_over=%d -> naive_false=%d vs ele_false=%d/%d\n",
                s2_peak_amps, s2_max_slope, s2_naive_over_samples, s2_naive_false_trips, s2_false_trips, s2_total_samples);
    std::printf("  [PASS Scen 3] 50V/ns SiC EMI (50x +1800 A spikes): guarded=%d/50 (logged=%u), unguarded=%d/50 trips\n",
                s3_guarded_trips, s3_rej_log, s3_unguarded_trips);
    std::printf("  [PASS Scen 4] 85C Hall Drift (+35 A = %u LSB): uncal_rearm=REJECTED -> AutoZero(%u) -> REARMED\n",
                drift_lsb, captured_off);
    std::printf("  [PASS Scen 5] Sustained 1.6x Overload (640 A): adiabatic I^2*t trip at tick=%d (cause=3'b100)\n",
                s5_trip_tick);
    std::printf("  [PASS Scen 6] Gate-Driver DESAT Bypass: 0-cycle combinational kill=%d, sticky=%d\n",
                zero_cyc_killed, desat_sticky);
    std::printf("  [PASS Scen 7] Post-Mortem Blackbox: 32/32 pre-trip/trip words frozen & bit-exact via CSR\n");

    if (json_path != nullptr) {
        std::FILE *fp = std::fopen(json_path, "w");
        if (fp == nullptr) {
            std::fprintf(stderr, "ERROR: cannot write %s\n", json_path);
            return 1;
        }
        std::fprintf(fp, "{\n");
        std::fprintf(fp, "  \"ip_id\": \"0x%08X\",\n", ip_id);
        std::fprintf(fp, "  \"sys_clk_mhz\": 100,\n");
        std::fprintf(fp, "  \"adc_msps\": 20,\n");
        std::fprintf(fp, "  \"scen1_trip_ns\": %.1f,\n", s1_trip_ns);
        std::fprintf(fp, "  \"scen1_peak_a\": %.2f,\n", s1_peak_amps);
        std::fprintf(fp, "  \"scen1_base_peak_a\": %.2f,\n", ref_peak_amps);
        std::fprintf(fp, "  \"scen1_let_through_a2s\": %.4f,\n", s1_i2t_let);
        std::fprintf(fp, "  \"scen1_base_let_through_a2s\": %.4f,\n", ref_i2t_let);
        std::fprintf(fp, "  \"scen1_i2t_reduction_x\": %.2f,\n", let_ratio);
        std::fprintf(fp, "  \"scen2_hotswap_peak_a\": %.2f,\n", s2_peak_amps);
        std::fprintf(fp, "  \"scen2_hotswap_max_slope_aus\": %.2f,\n", s2_max_slope);
        std::fprintf(fp, "  \"scen2_total_samples\": %d,\n", s2_total_samples);
        std::fprintf(fp, "  \"scen2_naive_over_samples\": %d,\n", s2_naive_over_samples);
        std::fprintf(fp, "  \"scen2_naive_false_trips\": %d,\n", s2_naive_false_trips);
        std::fprintf(fp, "  \"scen2_false_trips\": %d,\n", s2_false_trips);
        std::fprintf(fp, "  \"scen3_emi_guarded_trips\": %d,\n", s3_guarded_trips);
        std::fprintf(fp, "  \"scen3_emi_unguarded_trips\": %d,\n", s3_unguarded_trips);
        std::fprintf(fp, "  \"scen3_emi_logged\": %u,\n", s3_rej_log);
        std::fprintf(fp, "  \"scen4_drift_lsb\": %u,\n", drift_lsb);
        std::fprintf(fp, "  \"scen4_captured_offset_lsb\": %u,\n", captured_off);
        std::fprintf(fp, "  \"scen5_overload_trip_ticks\": %d,\n", s5_trip_tick);
        std::fprintf(fp, "  \"scen6_desat_zero_cycle_kill\": true,\n");
        std::fprintf(fp, "  \"scen7_blackbox_exact_words\": 32\n");
        std::fprintf(fp, "}\n");
        std::fclose(fp);
    }

    return 0;
}
