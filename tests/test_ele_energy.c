#include "../include/ele_csi.h"
#include "../include/ele_drive.h"
#include "../include/ele_driver.h"
#include "../include/ele_grid.h"
#include "../include/ele_hal.h"
#include "../include/ele_rack_vrm.h"
#include "../include/ele_silicon.h"
#include "../include/ele_sync.h"
#include "../include/ele_wire.h"
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t g_assertions_passed = 0U;

#define ELE_ASSERT(cond)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
      assert(cond);                                                            \
    }                                                                          \
    g_assertions_passed += 1U;                                                 \
  } while (0)

static void test_suite1_hal_and_wire_surface_charge(void) {
  /* 1. Cross-MCU Bare-Metal Silicon Port Profiles (EleHalPort64) */
  EleHalPort64 port_c6;
  EleHalPort64 port_s3;
  EleHalPort64 port_rp2350;
  EleHalPort64 port_stm32;
  ele_hal_port64_init(&port_c6, ELE_MCU_ESP32_C6_RV32);
  ele_hal_port64_init(&port_s3, ELE_MCU_ESP32_S3_ULP);
  ele_hal_port64_init(&port_rp2350, ELE_MCU_RP2350_RV32);
  ele_hal_port64_init(&port_stm32, ELE_MCU_STM32_CM33);

  ELE_ASSERT(sizeof(EleHalPort64) == 64U);
  ELE_ASSERT(port_c6.gpio_base_addr == 0x60091000UL &&
             port_c6.w1ts_set_off == 0x0008U);
  ELE_ASSERT(port_s3.gpio_base_addr == 0x60004000UL &&
             port_s3.hp_cpu_hz == 240000000UL);
  ELE_ASSERT(port_rp2350.gpio_base_addr == 0xD0000000UL &&
             port_rp2350.w1ts_set_off == 0x0018U);
  ELE_ASSERT(port_stm32.gpio_base_addr == 0x42020000UL &&
             port_stm32.w1tc_clr_off == 0x0028U);

  ele_hal_port_gpio_write(&port_c6, 4U, 1U);
  ele_hal_port_gpio_write(&port_c6, 7U, 1U);
  ELE_ASSERT(port_c6.gpio_shadow_out == ((1UL << 4U) | (1UL << 7U)));
  ele_hal_port_gpio_write(&port_c6, 4U, 0U);
  ELE_ASSERT(port_c6.gpio_shadow_out == (1UL << 7U));

  /* 2. Waveguide Impedance Matching on IO_MUX Pad Drive Strength */
  EleWireLink64 link_twisted;
  EleWireLink64 link_open;
  ele_wire_link64_init(&link_twisted, 200U, ELE_GEOM_TWISTED_PAIR, 3300U);
  ele_wire_link64_init(&link_open, 200U, ELE_GEOM_OPEN_LOOP, 3300U);

  uint32_t drv_twisted =
      (link_twisted.iomux_shadow_reg >> ELE_IO_MUX_DRV_SHIFT) &
      ELE_IO_MUX_DRV_MASK;
  uint32_t drv_open = (link_open.iomux_shadow_reg >> ELE_IO_MUX_DRV_SHIFT) &
                      ELE_IO_MUX_DRV_MASK;
  ELE_ASSERT(drv_twisted == (uint32_t)ELE_DRV_10MA);
  ELE_ASSERT(drv_open == (uint32_t)ELE_DRV_5MA);

  /* 3. Exhaustive 65,536 (256 x 256) PPM Single-Edge Round-Trip & Jitter Test
   */
  for (uint32_t prev = 0U; prev < 256U; ++prev) {
    for (uint32_t next = 0U; next < 256U; ++next) {
      uint32_t cyc = ele_wire_transmit_ppm_byte(&link_twisted, 4U,
                                                (uint8_t)prev, (uint8_t)next);
      uint8_t dec0 =
          ele_wire_receive_ppm_byte(&link_twisted, (uint8_t)prev, cyc);
      uint8_t dec_plus1 =
          ele_wire_receive_ppm_byte(&link_twisted, (uint8_t)prev, cyc + 1U);
      uint8_t dec_minus1 =
          ele_wire_receive_ppm_byte(&link_twisted, (uint8_t)prev, cyc - 1U);
      ELE_ASSERT(dec0 == (uint8_t)next);
      ELE_ASSERT(dec_plus1 == (uint8_t)next);
      ELE_ASSERT(dec_minus1 == (uint8_t)next);
    }
  }

  ELE_ASSERT(link_twisted.actual_surface_flips == 65536ULL);
  ELE_ASSERT(link_twisted.baseline_uart_flips >
             link_twisted.actual_surface_flips * 5ULL);
  ELE_ASSERT(link_twisted.energy_saved_pj_q16 >
             link_twisted.energy_used_pj_q16 * 4LL);

  uint64_t flip_saving_pct =
      ((link_twisted.baseline_uart_flips - link_twisted.actual_surface_flips) *
       100ULL) /
      link_twisted.baseline_uart_flips;
  printf("  [Suite 1] ele_hal + ele_wire: 4 MCU Ports + 65,536-pair PPM | "
         "Flips: %" PRIu64 " vs UART %" PRIu64 " (-%" PRIu64 "%%) | Saved: "
         "%" PRId64 " pJ\n",
         link_twisted.actual_surface_flips, link_twisted.baseline_uart_flips,
         flip_saving_pct, link_twisted.energy_saved_pj_q16 >> 16U);
}

static void test_suite2_zero_code_datasheet_driver_and_tickless_bus(void) {
  EleHalPort64 port;
  ele_hal_port64_init(&port, ELE_MCU_ESP32_C6_RV32);

  /* =========================================================================
   * Part A: 100% Zero-Code HTU21D (0x40) via EleBusSeq64 + CRC-8 (Poly 0x31)
   *   T(C) = -46.85 + 175.72 * (raw14 / 2^14)
   *   scale_q16 = 11516002, offset_q16 = -3070362
   * =========================================================================
   */
  EleDriverCodec64 htu_drv;
  ele_driver_codec64_init(&htu_drv, 1U, 0x40U, 0xE3U, 2U, 14U,
                          ELE_CODEC_BIG_ENDIAN | ELE_CODEC_CRC8_POLY31,
                          11516002, -3070362, (5 << 16));
  ELE_ASSERT(sizeof(EleDriverCodec64) == 64U);
  ELE_ASSERT(sizeof(EleBusSeq64) == 64U);

  const uint64_t htu_ops[4] = {
      /* PC=0 (Init): Soft Reset 0xFE + 15ms non-blocking LP sleep */
      ele_bus_op_pack(ELE_OP_CMD_ONLY, 0xFEU, 0U, 0xFEU, 0U, 1500U),
      /* PC=1 (Loop Start): Trigger Temp No-Hold 0xF3 */
      ele_bus_op_pack(ELE_OP_CMD_ONLY, 0xF3U, 0U, 0xF3U, 0U, 0U),
      /* PC=2: Non-blocking 50ms LP conversion sleep (0 CPU spin-wait) */
      ele_bus_op_pack(ELE_OP_WAIT_US, 0U, 0U, 0U, 0U, 5000U),
      /* PC=3: Burst Read 3 bytes (MSB, LSB, Sensirion CRC-8) */
      ele_bus_op_pack(ELE_OP_BURST_READ, 0x00U, 0U, 0U, 3U, 0U)};

  EleBusSeq64 htu_seq;
  ele_bus_seq64_init(&htu_seq, ELE_BUS_PROTO_I2C, 0x40U, 1U, 50000U, 400000U,
                     htu_ops);

  uint8_t htu_regs[256];
  memset(htu_regs, 0, sizeof(htu_regs));
  htu_regs[0] = 0x68U;
  htu_regs[1] = 0x3AU; /* ~24.68 C */
  htu_regs[2] = ele_crc8_compute(htu_regs, 2U, 0x31U, 0x00U);

  /* Execute Init -> Sleep -> Trigger -> 50ms LP Sleep -> Burst Read + CRC-8 */
  ELE_ASSERT(ele_bus_seq_step(&htu_seq, &htu_drv, &port, htu_regs, 0U, 2048,
                              true) == ELE_SEQ_STEP_ISSUED);
  ELE_ASSERT(htu_regs[0xFEU] == 0xFEU);
  ELE_ASSERT(ele_bus_seq_step(&htu_seq, &htu_drv, &port, htu_regs, 10000U, 2048,
                              true) == ELE_SEQ_SLEEP_TICKLESS);
  ELE_ASSERT(ele_bus_seq_step(&htu_seq, &htu_drv, &port, htu_regs, 5000U, 2048,
                              true) == ELE_SEQ_STEP_ISSUED);
  ELE_ASSERT(ele_bus_seq_step(&htu_seq, &htu_drv, &port, htu_regs, 0U, 2048,
                              true) == ELE_SEQ_SLEEP_TICKLESS);
  ELE_ASSERT(ele_bus_seq_step(&htu_seq, &htu_drv, &port, htu_regs, 50000U, 2048,
                              true) == ELE_SEQ_DATA_READY);

  ELE_ASSERT((htu_drv.jet_pos_q16 >> 16) == 24);
  ELE_ASSERT(htu_drv.bus_health_state == ELE_BUS_HEALTHY);
  ELE_ASSERT(port.sleep_us_accum == 65000ULL);
  ELE_ASSERT(port.energy_saved_uj_q16 > 0LL);

  /* Hardware-Free CRC-8 Sieve: 1-bit wire corruption rejected, 2-Jet coasts! */
  int32_t clean_temp_q16 = htu_drv.jet_pos_q16;
  uint8_t bad_crc_frame[4] = {0xFFU, 0xFFU, (uint8_t)(htu_regs[2] ^ 0x01U), 0U};
  int32_t coast_crc =
      ele_driver_decode_and_heal_step(&htu_drv, bad_crc_frame, true);
  ELE_ASSERT((htu_drv.bus_health_state & ELE_BUS_CRC_REJECTED) != 0U);
  ELE_ASSERT((htu_drv.bus_health_state & ELE_BUS_COASTING_2JET) != 0U);
  ELE_ASSERT(coast_crc == clean_temp_q16);

  /* Valid 1-Cycle Zero-Reboot Hot-Swap (v1 -> v2) + Faulty 10x Auto-Rollback */
  uint8_t valid_f2[4] = {0x68U, 0x4AU, 0U, 0U};
  valid_f2[2] = ele_crc8_compute(valid_f2, 2U, 0x31U, 0x00U);
  (void)ele_driver_decode_and_heal_step(&htu_drv, valid_f2, true);

  uint8_t htu_flags = ELE_CODEC_BIG_ENDIAN | ELE_CODEC_CRC8_POLY31;
  uint64_t seal_v2 = ele_driver_seal_fp64(2U, 0x40U, 0xE3U, 11516100, -3070000,
                                          2U, 14U, htu_flags);
  ELE_ASSERT(
      ele_driver_hotswap_atomic64(&htu_drv, 2U, 11516100, -3070000, seal_v2));
  int32_t t_v2 = ele_driver_decode_and_heal_step(&htu_drv, valid_f2, true);
  ELE_ASSERT(htu_drv.driver_version == 2U && htu_drv.hot_swap_rollbacks == 0U);

  /* Reject downgrade & tampered seal */
  ELE_ASSERT(
      !ele_driver_hotswap_atomic64(&htu_drv, 2U, 11516100, -3070000, seal_v2));
  ELE_ASSERT(!ele_driver_hotswap_atomic64(&htu_drv, 3U, 11516100, -3070000,
                                          seal_v2 ^ 1ULL));

  /* Faulty 10x gain -> 1-Cycle Automatic Shadow Rollback */
  uint64_t seal_bad = ele_driver_seal_fp64(3U, 0x40U, 0xE3U, 115161000,
                                           -3070000, 2U, 14U, htu_flags);
  ELE_ASSERT(
      ele_driver_hotswap_atomic64(&htu_drv, 3U, 115161000, -3070000, seal_bad));
  int32_t t_rb = ele_driver_decode_and_heal_step(&htu_drv, valid_f2, true);
  ELE_ASSERT(htu_drv.hot_swap_rollbacks == 1U && htu_drv.driver_version == 2U);
  ELE_ASSERT(htu_drv.bus_health_state == ELE_BUS_ROLLED_BACK);
  ELE_ASSERT(t_rb == t_v2);

  /* Stuck SDA Bus -> 9-Clock Recovery + 2-Jet Coasting */
  (void)ele_driver_decode_and_heal_step(&htu_drv, NULL, false);
  ELE_ASSERT(htu_drv.recovery_pulses_sent == 9U);
  ELE_ASSERT((htu_drv.bus_health_state & ELE_BUS_9CLK_RECOVERED) != 0U);

  /* =========================================================================
   * Part B: Zero-Code MPU6050 (0x68) RMW + DRDY Poll + Tickless 2-Jet Governor
   * =========================================================================
   */
  EleDriverCodec64 imu_axes[3];
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    ele_driver_codec64_init(
        &imu_axes[axis], 1U, 0x68U, (uint16_t)(0x3BU + axis * 2U), 0U, 16U,
        ELE_CODEC_BIG_ENDIAN | ELE_CODEC_SIGNED_2C, (16 << 16), 0, (32 << 16));
  }

  const uint64_t imu_ops[4] = {
      /* PC=0: Wake PWR_MGMT_1 (0x6B = 0x00) */
      ele_bus_op_pack(ELE_OP_WRITE_REG8, 0x6BU, 0U, 0x00U, 0U, 0U),
      /* PC=1: RMW ACCEL_CONFIG (0x1C) bits [4:3] = 0b10 (+/-8g), keep reserved
       */
      ele_bus_op_pack(ELE_OP_RMW_BITS8, 0x1CU, 0x18U, 0x10U, 0U, 0U),
      /* PC=2 (Loop): Bounded DRDY Poll on INT_STATUS (0x3A & 0x01 == 0x01) */
      ele_bus_op_pack(ELE_OP_POLL_MASK, 0x3AU, 0x01U, 0x01U, 0U, 100U),
      /* PC=3: Burst Read ACCEL_XOUT_H..L (2 bytes) */
      ele_bus_op_pack(ELE_OP_BURST_READ, 0x3BU, 0U, 0U, 2U, 0U)};

  EleBusSeq64 imu_seq;
  ele_bus_seq64_init(&imu_seq, ELE_BUS_PROTO_I2C, 0x68U, 2U, 10000U, 320000U,
                     imu_ops);

  uint8_t imu_regs[256];
  memset(imu_regs, 0, sizeof(imu_regs));
  imu_regs[0x6BU] = 0x40U; /* Default sleep bit set */
  imu_regs[0x1CU] = 0xE3U; /* Factory trim bits [7:5]=111, [2:0]=011 */
  imu_regs[0x3AU] = 0x00U; /* DRDY not ready yet */
  imu_regs[0x3BU] = 0x00U;
  imu_regs[0x3CU] = 0x00U;

  ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 0U, 512,
                              true) == ELE_SEQ_STEP_ISSUED);
  ELE_ASSERT(imu_regs[0x6BU] == 0x00U);
  ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 0U, 512,
                              true) == ELE_SEQ_STEP_ISSUED);
  /* Unmasked datasheet trim bits [7:5, 2:0] MUST remain strictly 0xE3 | 0x10!
   */
  ELE_ASSERT(imu_regs[0x1CU] == 0xF3U);

  /* DRDY == 0 -> Non-blocking LP sleep; DRDY == 1 -> Advances to burst read */
  ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 0U, 512,
                              true) == ELE_SEQ_SLEEP_TICKLESS);
  imu_regs[0x3AU] = 0x01U;

  /* Static IMU over 6 cycles -> Tickless Governor scales 10ms -> 320ms (32x
   * sleep)! */
  for (uint32_t c = 0U; c < 6U; ++c) {
    uint32_t wait = imu_seq.wake_countdown_us;
    ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, wait,
                                512, true) == ELE_SEQ_STEP_ISSUED);
    ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 0U,
                                512, true) == ELE_SEQ_DATA_READY);
  }
  ELE_ASSERT(imu_seq.current_period_us == 320000U);
  ELE_ASSERT(imu_seq.suppressed_wakeups >= 50U);

  /* Inject sudden physical acceleration impulse -> 1-Cycle Snapback to 10ms! */
  imu_regs[0x3BU] = 0x20U; /* +8192 raw (+2g) */
  imu_regs[0x3CU] = 0x00U;
  ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 320000U,
                              512, true) == ELE_SEQ_STEP_ISSUED);
  ELE_ASSERT(ele_bus_seq_step(&imu_seq, &imu_axes[0], &port, imu_regs, 0U, 512,
                              true) == ELE_SEQ_DATA_READY);
  ELE_ASSERT(imu_seq.current_period_us == 10000U);

  /* Multi-Axis 6-Byte Signed 2C Burst Decode (Ax = +4096, Ay = -512, Az =
   * +16384) */
  const uint8_t imu_burst6[6] = {0x10U, 0x00U, 0xFEU, 0x00U, 0x40U, 0x00U};
  int32_t xyz_q16[3] = {0, 0, 0};
  ele_driver_decode_burst_axes(imu_axes, 3U, imu_burst6, 2U, true, xyz_q16);
  ELE_ASSERT(xyz_q16[0] == 65536);  /* +1.0g in Q16.16 */
  ELE_ASSERT(xyz_q16[1] == -8192);  /* -0.125g in Q16.16 */
  ELE_ASSERT(xyz_q16[2] == 262144); /* +4.0g in Q16.16 */

  /* =========================================================================
   * Part C: Bidirectional Clamped Actuator Encoder (12-bit DAC, shift=4, 0..4V)
   * =========================================================================
   */
  EleDriverCodec64 dac_drv;
  ele_driver_codec64_init(&dac_drv, 1U, 0x60U, 0x40U, 4U, 12U,
                          ELE_CODEC_BIG_ENDIAN | ELE_CODEC_ACTUATOR_CLAMP,
                          (4 << 16), 0, (5 << 16));
  uint8_t dac_bytes[4] = {0U, 0U, 0U, 0U};
  uint32_t raw_mid =
      ele_driver_encode_actuator_raw(&dac_drv, (2 << 16), dac_bytes);
  ELE_ASSERT((raw_mid >> 4U) == 2048U);
  int32_t dac_rt = ele_driver_decode_and_heal_step(&dac_drv, dac_bytes, true);
  ELE_ASSERT(dac_rt == (2 << 16));

  /* Verify upper & lower hardware rail clamping */
  uint32_t raw_hi =
      ele_driver_encode_actuator_raw(&dac_drv, (10 << 16), dac_bytes);
  uint32_t raw_lo =
      ele_driver_encode_actuator_raw(&dac_drv, -(2 << 16), dac_bytes);
  ELE_ASSERT((raw_hi >> 4U) == 4095U);
  ELE_ASSERT((raw_lo >> 4U) == 0U);

  /* =========================================================================
   * Part D: Zero-Code TI INA226 Self-Calorimeter (100 mA @ 3.30 V, 100 mOhm)
   * =========================================================================
   */
  EleDriverCodec64 ina_shunt;
  EleDriverCodec64 ina_bus;
  EleBusSeq64 ina_seq;
  ele_driver_init_ina226_calorimeter(&ina_shunt, &ina_bus, &ina_seq, 0x44U);
  uint8_t ina_shunt_raw[4] = {0x0FU, 0xA0U, 0U, 0U}; /* 4000 * 2.5uV = 10.0mV */
  uint8_t ina_bus_raw[4] = {0x0AU, 0x50U, 0U, 0U};   /* 2640 * 1.25mV = 3.30V */
  int32_t v_shunt_mv_q16 =
      ele_driver_decode_and_heal_step(&ina_shunt, ina_shunt_raw, true);
  int32_t v_bus_v_q16 =
      ele_driver_decode_and_heal_step(&ina_bus, ina_bus_raw, true);
  uint32_t p_active_uw =
      ele_driver_eval_ina226_power_uw(v_shunt_mv_q16, v_bus_v_q16, 100U);
  /* Exact active rail power = 330,000 uW (within < 10 uW fixed-point bound) */
  ELE_ASSERT(p_active_uw >= 329990U && p_active_uw <= 330010U);

  printf("  [Suite 2] ele_driver + EleBusSeq64: Zero-Code HTU21D+MPU6050+DAC+"
         "INA226(%u uW) | Tickless Saved: %u | LP Saved: %" PRId64 " uJ\n",
         p_active_uw, imu_seq.suppressed_wakeups,
         port.energy_saved_uj_q16 >> 16U);
}

static void test_suite3_sync_zero_bit_jet2(void) {
  /* Sensor-agnostic 1,000-epoch 2-Jet Zero-Bit Sync between Tx & Rx nodes */
  EleSyncNode64 tx_node;
  EleSyncNode64 rx_node;
  int64_t initial_q16 = (24LL << 16U) + 44564LL; /* ~24.68 C from codec */
  int64_t eps_q16 = 3276LL;                      /* ~0.05 unit deadband */
  ele_sync_node64_init(&tx_node, initial_q16, eps_q16);
  ele_sync_node64_init(&rx_node, initial_q16, eps_q16);

  int64_t true_x = initial_q16;
  int64_t true_v = 256LL;
  int64_t true_a = 4LL;

  for (uint32_t step = 0U; step < 1000U; ++step) {
    if (step == 200U || step == 500U || step == 800U) {
      true_v += (step == 500U) ? -4096LL : 6144LL;
    }
    true_x += true_v + (true_a >> 1);
    true_v += true_a;

    int64_t delta_wire = 0LL;
    EleSyncDecision dec = ele_sync_step_tx_gate(&tx_node, true_x, &delta_wire);
    int64_t rx_est = ele_sync_step_rx_mirror(&rx_node, dec, delta_wire);

    /* Core invariant: Tx and Rx 2-Jet states are 0-ULP bit-identical */
    ELE_ASSERT(tx_node.x_q16 == rx_est);
    ELE_ASSERT(tx_node.v_q16 == rx_node.v_q16);
    ELE_ASSERT(tx_node.a_q16 == rx_node.a_q16);

    int64_t err = true_x - rx_est;
    if (err < 0LL) {
      err = -err;
    }
    ELE_ASSERT(err <= eps_q16);
  }

  uint32_t total_frames =
      tx_node.suppressed_frames + tx_node.transmitted_frames;
  uint32_t zero_bit_pct = (tx_node.suppressed_frames * 100U) / total_frames;
  ELE_ASSERT(zero_bit_pct >= 95U);

  printf("  [Suite 3] ele_sync: 1,000-epoch 2-Jet Sync | Zero-Bit Sleep: %u/%u "
         "(%u%%) | Saved: %" PRId64 " uJ\n",
         tx_node.suppressed_frames, total_frames, zero_bit_pct,
         tx_node.energy_saved_uj_q16 >> 16U);
}

static void test_suite4_drive_symplectic_motor(void) {
  EleMotorDrive64 drv;
  ele_drive_motor64_init(&drv, 7400U, 4200U, 1500U);

  uint32_t rotor_bemf_mv = 0U;
  uint32_t coast_steps = 0U;
  uint32_t pulse_steps = 0U;
  uint32_t regen_steps = 0U;

  for (uint32_t step = 0U; step < 2000U; ++step) {
    uint32_t piezo_mv =
        (drv.active_phase == ELE_DRIVE_SYMPLECTIC_PULSE) ? 450U : 180U;
    uint32_t mic_uv =
        (drv.active_phase == ELE_DRIVE_SYMPLECTIC_PULSE) ? 12000U : 3000U;
    int32_t dtemp_mk_s = 120;

    if (step == 1200U) {
      rotor_bemf_mv = 4650U;
    }

    EleDrivePhase phase = ele_drive_step_symplectic(
        &drv, rotor_bemf_mv, piezo_mv, mic_uv, dtemp_mk_s, 100U);

    if (phase == ELE_DRIVE_SYMPLECTIC_PULSE) {
      pulse_steps += 1U;
      rotor_bemf_mv += 38U;
    } else if (phase == ELE_DRIVE_COAST_HIGH_Z) {
      coast_steps += 1U;
      if (rotor_bemf_mv >= 6U) {
        rotor_bemf_mv -= 6U;
      }
      ELE_ASSERT(ele_drive_eval_waste_power_mw(&drv, drv.coil_current_ma) <
                 50U);
    } else {
      regen_steps += 1U;
      if (rotor_bemf_mv >= 25U) {
        rotor_bemf_mv -= 25U;
      }
    }
  }

  ELE_ASSERT(coast_steps > pulse_steps * 3U);
  ELE_ASSERT(regen_steps > 0U);
  ELE_ASSERT(drv.energy_saved_uj_q16 > drv.energy_input_uj_q16);
  ELE_ASSERT(drv.piezo_harvest_uj_q16 > 0LL);

  int64_t used_uj = drv.energy_input_uj_q16 >> 16U;
  int64_t saved_uj = drv.energy_saved_uj_q16 >> 16U;
  int64_t total_pwm_uj = used_uj + saved_uj;
  int64_t motor_saving_pct =
      (saved_uj * 100LL) / (total_pwm_uj > 0 ? total_pwm_uj : 1);

  printf("  [Suite 4] ele_drive: RK-380PH + L298N | Coast: %u, Pulse: %u, "
         "Regen: %u | Energy Saved vs PWM: %" PRId64 "%% (%" PRId64
         " uJ) | Piezo/Regen Harvested: %" PRId64 " uJ\n",
         coast_steps, pulse_steps, regen_steps, motor_saving_pct, saved_uj,
         drv.piezo_harvest_uj_q16 >> 16U);
}

static void test_suite5_espnow_csi_wave(void) {
  EleCsiState64 csi;
  ele_csi_state64_init(&csi, 64U);

  int8_t iq_buf[128];
  uint32_t static_cnt = 0U;
  uint32_t micro_cnt = 0U;
  uint32_t macro_cnt = 0U;

  for (uint32_t step = 0U; step < 300U; ++step) {
    for (uint16_t k = 0U; k < 64U; ++k) {
      int32_t base_i = 24 + (int32_t)(k & 7U);
      int32_t base_q = -18 + (int32_t)((k >> 1U) & 7U);

      if (step >= 100U && step < 180U) {
        int32_t rot = (step & 1U) ? 2 : -2;
        base_i += rot;
        base_q -= rot;
      } else if (step >= 220U && step < 260U) {
        int32_t shift = (int32_t)((step & 3U) * 9U) - 14;
        base_i += shift;
        base_q -= (shift * 2);
      }
      iq_buf[k * 2U] = (int8_t)base_i;
      iq_buf[k * 2U + 1U] = (int8_t)base_q;
    }

    EleCsiFieldRegime reg =
        ele_csi_ingest_espnow_frame(&csi, iq_buf, 128U, -48, -92);
    if (reg == ELE_CSI_FIELD_STATIC_SLEEP) {
      static_cnt += 1U;
      ELE_ASSERT(csi.recommended_rate_hz == 2U);
    } else if (reg == ELE_CSI_FIELD_MICRO_WAVE) {
      micro_cnt += 1U;
      ELE_ASSERT(csi.recommended_rate_hz == 25U);
    } else {
      macro_cnt += 1U;
      ELE_ASSERT(csi.recommended_rate_hz == 100U);
    }
    if (step > 0U) {
      ELE_ASSERT(csi.tx_power_qdbm == 8U);
    }
  }

  ELE_ASSERT(static_cnt > 150U);
  ELE_ASSERT(micro_cnt > 20U);
  ELE_ASSERT(macro_cnt > 15U);
  ELE_ASSERT(csi.energy_saved_uj_q16 > 0LL);

  printf("  [Suite 5] ele_csi: ESP-NOW 64-Subcarrier Wave ISAC | Static(2Hz): "
         "%u, MicroWave(25Hz): %u, Macro(100Hz): %u | TX Power: +2 dBm | "
         "Saved: %" PRId64 " uJ\n",
         static_cnt, micro_cnt, macro_cnt, csi.energy_saved_uj_q16 >> 16U);
}

static void test_suite6_silicon_datacenter_power_physics(void) {
  ELE_ASSERT(sizeof(EleCacheBus64) == 64U);
  ELE_ASSERT(sizeof(ElePdnGrid64) == 64U);

  /* 1. Adversarial 64B Bus Thrashing (0x00..00 <-> 0xFF..FF):
   *    Uncoded flips = 512 bits/line; 65-wire Bus-Invert caps <= 256 flips!
   */
  EleCacheBus64 adv_bus;
  ele_silicon_bus64_init(&adv_bus, 1800U, 1100U);
  for (uint32_t step = 0U; step < 64U; ++step) {
    uint64_t pat = (step & 1U) ? 0xFFFFFFFFFFFFFFFFULL : 0x0000000000000000ULL;
    uint64_t tx_line[8] = {pat, pat, pat, pat, pat, pat, pat, pat};
    uint64_t wire_words[8];
    uint64_t rx_line[8];
    uint64_t prev_enc = adv_bus.encoded_wire_flips;
    uint8_t inv =
        ele_silicon_tx_cacheline64(&adv_bus, tx_line, NULL, wire_words);
    ele_silicon_rx_cacheline64(wire_words, NULL, inv, rx_line);
    ELE_ASSERT(memcmp(tx_line, rx_line, sizeof(tx_line)) == 0);
    ELE_ASSERT((adv_bus.encoded_wire_flips - prev_enc) <= 256ULL);
  }
  ELE_ASSERT(adv_bus.encoded_wire_flips * 10ULL <
             adv_bus.raw_unencoded_flips * 6ULL);

  /* 2. HBM3E Tensor / KV-Cache 64B Stream (2-Jet Residual + Bus-Invert) */
  EleCacheBus64 hbm_bus;
  ele_silicon_bus64_init(&hbm_bus, 1200U, 400U);
  uint64_t pred_line[8];
  uint64_t cur_line[8];
  for (uint32_t k = 0U; k < 8U; ++k) {
    pred_line[k] =
        0xA5A55A5AF0F00F0FULL ^ ((uint64_t)k * 0x1111111111111111ULL);
  }
  for (uint32_t step = 0U; step < 256U; ++step) {
    for (uint32_t k = 0U; k < 8U; ++k) {
      /* Alternate high-entropy tensor base while 7/8 lanes match prediction */
      uint64_t base = (step & 1U) ? ~pred_line[k] : pred_line[k];
      uint64_t sparse_delta = (k == (step & 7U) && (step & 3U) == 0U)
                                  ? ((uint64_t)step + 1ULL)
                                  : 0ULL;
      cur_line[k] = base ^ sparse_delta;
    }
    uint64_t cur_pred[8];
    for (uint32_t k = 0U; k < 8U; ++k) {
      cur_pred[k] = (step & 1U) ? ~pred_line[k] : pred_line[k];
    }
    uint64_t wire_w[8];
    uint64_t dec_w[8];
    uint8_t mask8 =
        ele_silicon_tx_cacheline64(&hbm_bus, cur_line, cur_pred, wire_w);
    ele_silicon_rx_cacheline64(wire_w, cur_pred, mask8, dec_w);
    ELE_ASSERT(memcmp(cur_line, dec_w, sizeof(cur_line)) == 0);
  }
  uint64_t hbm_flip_save_pct =
      ((hbm_bus.raw_unencoded_flips - hbm_bus.encoded_wire_flips) * 100ULL) /
      hbm_bus.raw_unencoded_flips;
  ELE_ASSERT(hbm_flip_save_pct >= 90ULL);
  ELE_ASSERT(hbm_bus.zero_beat_skips > 200U);

  /* 3. PDN L*dI/dt Voltage-Droop Tax & 100% Compute-Work Conservation */
  ElePdnGrid64 pdn;
  ele_silicon_pdn64_init(&pdn, 700U, 250U, 75U, 45000, 76000);
  uint64_t sum_sync_work = 0ULL;
  uint64_t sum_symp_work = 0ULL;
  uint32_t prev_sync = 0U;

  for (uint32_t t = 0U; t < 240U; ++t) {
    /* Synchronous GEMM / AllReduce square wave (0 A <-> 600 A) for t < 180,
     * followed by drain tail so 100% of queued work finishes */
    uint32_t req_sync = (t < 180U && ((t / 12U) & 1U) == 0U) ? 600U : 0U;
    uint32_t symp_amp =
        ele_silicon_step_pdn_symplectic(&pdn, prev_sync, req_sync);
    prev_sync = req_sync;
    sum_sync_work += (uint64_t)req_sync;
    sum_symp_work += (uint64_t)symp_amp;
  }
  ELE_ASSERT(pdn.backlog_work_amp_ticks == 0U);
  ELE_ASSERT(sum_symp_work ==
             sum_sync_work); /* 100.0% identical compute work! */
  ELE_ASSERT(pdn.peak_vdroop_sync_mv == 150U);
  ELE_ASSERT(pdn.peak_vdroop_symp_mv <= 19U); /* -87% inductive Vdroop! */
  ELE_ASSERT(pdn.pdn_energy_saved_mj > 10000LL);

  /* 4. CMOS Gate-Toggle Entropy: IEEE-754 FP32 vs 1.58-Bit Ternary vs
   *    Dual Monotonic Magnitude-Rail Sign-Grouped Ternary */
  int32_t x_vec[512];
  int8_t w_trit[512];
  int64_t ref_seq_sum = 0LL;
  for (uint32_t i = 0U; i < 512U; ++i) {
    int32_t sign = (i & 1U) ? -1 : 1;
    x_vec[i] = sign * (int32_t)((i * 317U + 19U) & 0x3FFFFU);
    w_trit[i] = (int8_t)((i % 3U == 0U) ? 0 : ((i & 2U) ? 1 : -1));
    ref_seq_sum += (int64_t)w_trit[i] * (int64_t)x_vec[i];
  }
  uint64_t fp32_tog = 0ULL;
  uint64_t tern_tog = 0ULL;
  uint64_t grouped_tog = 0ULL;
  (void)ele_silicon_compare_dot_toggle_entropy(&pdn, x_vec, w_trit, 512U,
                                               &fp32_tog, &tern_tog);
  int64_t grouped_sum =
      ele_silicon_dot_ternary_sign_grouped(x_vec, w_trit, 512U, &grouped_tog);
  ELE_ASSERT(grouped_sum == ref_seq_sum); /* 0-ULP exact algebraic identity! */
  ELE_ASSERT(fp32_tog > tern_tog * 3ULL);
  ELE_ASSERT(grouped_tog < tern_tog);
  uint64_t tog_save_pct = ((fp32_tog - grouped_tog) * 100ULL) / fp32_tog;

  /* 5. Subthreshold Exponential Thermal-Leakage Runaway (2-Die Spatial Routing
   *    at 100% Identical 450W Active Compute & Identical RC Physics) */
  ELE_ASSERT(ele_silicon_eval_leakage_mw(45000, 15000U) == 15000U);
  ELE_ASSERT(ele_silicon_eval_leakage_mw(57000, 15000U) == 30000U);
  ELE_ASSERT(ele_silicon_eval_leakage_mw(81000, 15000U) == 120000U);

  ElePdnGrid64 die_react[2];
  ElePdnGrid64 die_jet2[2];
  ele_silicon_pdn64_init(&die_react[0], 700U, 250U, 75U, 45000, 95000);
  ele_silicon_pdn64_init(&die_react[1], 700U, 250U, 75U, 45000, 95000);
  ele_silicon_pdn64_init(&die_jet2[0], 700U, 250U, 75U, 45000, 68000);
  ele_silicon_pdn64_init(&die_jet2[1], 700U, 250U, 75U, 45000, 68000);
  uint64_t leak_reactive_mj = 0ULL;
  uint64_t leak_jet2_mj = 0ULL;
  uint8_t active_r = 0U;
  uint8_t active_j = 0U;

  for (uint32_t step = 0U; step < 120U; ++step) {
    /* Both schedulers execute 450W on EVERY tick (100% identical throughput) */
    uint8_t next_r = active_r;
    uint8_t next_j = active_j;
    for (uint8_t d = 0U; d < 2U; ++d) {
      uint32_t rw = (d == active_r) ? 450U : 0U;
      leak_reactive_mj +=
          (uint64_t)(ele_silicon_eval_leakage_mw(die_react[d].temp_mc, 15000U) /
                     1000U);
      (void)ele_silicon_step_thermal_jet2(&die_react[d], rw, 45000, 1U);
      if (d == active_r && die_react[d].temp_mc >= 95000) {
        next_r = (uint8_t)(1U - active_r);
      }

      uint32_t jw = (d == active_j) ? 450U : 0U;
      leak_jet2_mj +=
          (uint64_t)(ele_silicon_eval_leakage_mw(die_jet2[d].temp_mc, 15000U) /
                     1000U);
      uint32_t cap = ele_silicon_step_thermal_jet2(&die_jet2[d], jw, 45000, 4U);
      if (d == active_j && cap < 600U) {
        next_j = (uint8_t)(1U - active_j);
      }
    }
    active_r = next_r;
    active_j = next_j;
  }
  ELE_ASSERT(die_jet2[0].thermal_preempts + die_jet2[1].thermal_preempts > 0U);
  ELE_ASSERT(leak_jet2_mj * 10ULL < leak_reactive_mj * 7ULL);

  printf("  [Suite 6] ele_silicon: HBM64 Flips -%" PRIu64 "%% | PDN Vdroop "
         "%umV->%umV (Saved %" PRId64 " mJ, 100%% Work) | Ternary Gate Toggles "
         "-%" PRIu64 "%% | 2-Jet Leakage: %" PRIu64 " vs %" PRIu64 " mJ\n",
         hbm_flip_save_pct, pdn.peak_vdroop_sync_mv, pdn.peak_vdroop_symp_mv,
         pdn.pdn_energy_saved_mj, tog_save_pct, leak_jet2_mj, leak_reactive_mj);
}

/* ============================================================================
 * Suite 7: 48V Multiphase GaN/SiC Digital VRM (Phase-Shedding, ZVS & Active
 *          Pre-Charge) + 8-Socket Rack Busbar Stranded-Megawatt Unlocker
 * ============================================================================
 */
static void test_suite7_rack_vrm_zvs_and_stranded_power(void) {
  EleVrmPhase64 vrm;
  /* 48V -> 0.75V @ 16-phase GaN/DrMOS VRM: 800 uOhm/phase, 3600 mW/phase hard
   * switching loss, 12,000 uF output decoupling, 15 A/us per-phase slew */
  ele_vrm64_init(&vrm, 48000U, 750U, 16U, 800U, 3600U, 12000U, 15U);

  /* 1. Exhaustive discrete convex optimality across [0..1200 A]:
   *    ele_vrm_optimal_phases must match or beat EVERY possible N in [1..16] */
  for (uint32_t amp = 0U; amp <= 1200U; amp += 5U) {
    uint8_t n_opt = ele_vrm_optimal_phases(&vrm, amp);
    ELE_ASSERT(n_opt >= 1U && n_opt <= 16U);
    uint32_t opt_loss = ele_vrm_eval_loss_mw(&vrm, amp, n_opt, false);
    for (uint8_t n_any = 1U; n_any <= 16U; ++n_any) {
      uint32_t any_loss = ele_vrm_eval_loss_mw(&vrm, amp, n_any, false);
      ELE_ASSERT(opt_loss <= any_loss);
    }
  }
  ELE_ASSERT(ele_vrm_optimal_phases(&vrm, 90U) == 1U);
  ELE_ASSERT(ele_vrm_optimal_phases(&vrm, 1050U) >= 15U);

  /* Also verify ZVS-aware discrete convex optimality across [0..1200 A] */
  vrm.zvs_locked = 1U;
  for (uint32_t amp = 0U; amp <= 1200U; amp += 5U) {
    uint8_t n_zvs = ele_vrm_optimal_phases(&vrm, amp);
    ELE_ASSERT(n_zvs >= 1U && n_zvs <= 16U);
    uint32_t zvs_opt_loss = ele_vrm_eval_loss_mw(&vrm, amp, n_zvs, true);
    for (uint8_t n_any = 1U; n_any <= 16U; ++n_any) {
      uint32_t any_loss = ele_vrm_eval_loss_mw(&vrm, amp, n_any, true);
      ELE_ASSERT(zvs_opt_loss <= any_loss);
    }
  }
  vrm.zvs_locked = 0U;

  /* 2. 1,000 us mixed LLM workload: 80% memory-bound KV decode (90 A) +
   *    20% Tensor GEMM bursts (1050 A) with 2-us Active Pre-Charge lookahead +
   *    2-Jet picosecond ZVS dead-time resonant convergence */
  vrm.inductor_current_a = 90U;
  int32_t zvs_err_ps = 1800; /* Initial +1.8 ns hard-switching timing offset */

  for (uint32_t us = 0U; us < 1000U; ++us) {
    uint32_t slot = us % 50U;
    uint32_t load_a = (slot >= 40U && slot < 50U) ? 1050U : 90U;
    uint32_t upcoming_a = (slot >= 38U && slot < 50U) ? 1050U : 90U;

    uint8_t phases = ele_vrm_step_us(&vrm, load_a, upcoming_a, zvs_err_ps);
    ELE_ASSERT(phases >= 1U && phases <= 16U);
    if (slot == 38U || slot == 39U) {
      ELE_ASSERT(vrm.precharge_armed == 1U);
      ELE_ASSERT(phases == 16U);
    }
    /* Physical LC switch-node zero-crossing tracks dead-time adjustment */
    zvs_err_ps = (int32_t)vrm.t_dead_ps - 6200;
  }

  ELE_ASSERT(vrm.zvs_locked == 1U);
  ELE_ASSERT(vrm.max_droop_base_mv >= 110U);
  ELE_ASSERT(vrm.max_droop_active_mv * 2U < vrm.max_droop_base_mv);
  ELE_ASSERT(vrm.loss_symplectic_nj * 100ULL < vrm.loss_baseline_nj * 40ULL);
  uint64_t vrm_save_pct =
      ((vrm.loss_baseline_nj - vrm.loss_symplectic_nj) * 100ULL) /
      vrm.loss_baseline_nj;

  /* 3. 8-Socket 48V Rack Busbar Symplectic Phase-Interleaving:
   *    8 GPUs oscillate synchronously between 45 A (AllReduce) and 220 A (GEMM)
   *    on a 1,250 A substation breaker (500 uOhm busbar).
   *    Uncoordinated peak = 8 * 220 = 1,760 A (trips breaker; caps rack at 5
   *    GPUs). Symplectic interleaving caps draw <= 1,250 A with 100% conserved
   *    compute work, unlocking all 8 GPUs (+60% active GPUs on same breaker).
   */
  EleRackBusbar64 rack;
  ele_rack_busbar64_init(&rack, 1250U, 500U);
  uint64_t total_req_work = 0ULL;
  uint64_t total_symp_work = 0ULL;

  for (uint32_t tick = 0U; tick < 208U; ++tick) {
    uint16_t req[8];
    bool gemm_wave = (tick < 200U) && ((tick % 10U) < 4U);
    for (uint8_t k = 0U; k < 8U; ++k) {
      req[k] = (tick < 200U) ? (gemm_wave ? 220U : 45U) : 0U;
      total_req_work += (uint64_t)req[k];
    }
    uint32_t rack_a = ele_rack_step_interleave8(&rack, req, 90U);
    ELE_ASSERT(rack_a <= rack.breaker_cap_a);
    total_symp_work += (uint64_t)rack_a;
  }

  for (uint8_t k = 0U; k < 8U; ++k) {
    ELE_ASSERT(rack.slot_backlog_work[k] == 0U);
  }
  ELE_ASSERT(total_symp_work == total_req_work); /* 100% Work Conservation! */
  ELE_ASSERT(rack.peak_sync_rack_a == 1760U);
  ELE_ASSERT(rack.peak_interleaved_a <= 1250U);
  ELE_ASSERT(rack.bus_loss_symp_uj < rack.bus_loss_sync_uj);

  uint32_t sync_max_gpus = 0U;
  uint32_t unlocked_gpus =
      ele_rack_unlocked_gpu_slots(&rack, 8U, &sync_max_gpus);
  ELE_ASSERT(sync_max_gpus == 5U);
  ELE_ASSERT(unlocked_gpus >= 8U);
  uint64_t bus_i2r_save_pct =
      ((rack.bus_loss_sync_uj - rack.bus_loss_symp_uj) * 100ULL) /
      rack.bus_loss_sync_uj;

  printf("  [Suite 7] ele_rack_vrm: 48V 16ph GaN VRM Waste -%" PRIu64
         "%%, Vdroop %umV->%umV (ZVS Locked) | 8-GPU Rack Peak %uA->%uA "
         "(Breaker 1250A: %u->%u GPUs Powered, Bus I^2R -%" PRIu64 "%%)\n",
         vrm_save_pct, vrm.max_droop_base_mv, vrm.max_droop_active_mv,
         rack.peak_sync_rack_a, rack.peak_interleaved_a, sync_max_gpus,
         unlocked_gpus, bus_i2r_save_pct);
}

/* ============================================================================
 * Suite 8: Diurnal Duck-Curve 800V HVDC & 60Hz GFM Synthetic-Inertia Physics
 * ============================================================================
 */
static void test_suite8_grid_duck_curve_and_gfm_inertia(void) {
  /* 1. Hardware Zero-Crossing Period-to-mHz & Taiwan 60 Hz Island Contingency:
   *    -22 MW cloud-edge / generator trip on 35 MW/Hz isolated island feeder.
   *    GFL (H=0) plunges to 59.372 Hz (< 59.500 Hz UFLS blackout relay!).
   *    GFM Virtual Synchronous Generator + 1-cycle (16.67ms) compute P-cap
   *    arrests nadir > 59.850 Hz and absorbs 60.200 Hz midday PV surge. */
  ELE_ASSERT(ele_grid_period_ns_to_mhz(16666667U) == 60000U);
  ELE_ASSERT(ele_grid_period_ns_to_mhz(20000000U) == 50000U);

  EleGridInverter64 gfm;
  ele_grid_inverter64_init(&gfm, 60000U, 25000U, 18000U, 25000U, 30U);

  /* Step A: Midday 60.200 Hz (16,611,296 ns) solar overgeneration -> Sponge! */
  int32_t p_sponge = ele_grid_step_gfm_cycle(&gfm, 16611296U, 0, 35000U);
  ELE_ASSERT(gfm.cur_freq_mhz == 60200U);
  ELE_ASSERT(p_sponge < 0); /* Absorbing excess power into 800V BESS */
  ELE_ASSERT(gfm.compute_pcap_permille == 1150U);

  /* Return to nominal 60.000 Hz before contingency */
  (void)ele_grid_step_gfm_cycle(&gfm, 16666667U, 0, 35000U);
  ELE_ASSERT(gfm.compute_pcap_permille == 1000U);

  /* Step B: Sudden -22,000 kW island contingency (59.800 Hz -> 16,722,408 ns)
   */
  int32_t p_arrest = ele_grid_step_gfm_cycle(&gfm, 16722408U, -22000, 35000U);
  ELE_ASSERT(p_arrest > 18000);
  ELE_ASSERT(gfm.compute_pcap_permille <= 800U);
  ELE_ASSERT(gfm.nadir_gfl_mhz < 59500U);  /* GFL trips 59.5 Hz UFLS relay! */
  ELE_ASSERT(gfm.nadir_gfm_mhz >= 59800U); /* GFM arrests safely >= 59.8 Hz! */
  ELE_ASSERT(gfm.inertia_energy_kj > 0ULL);

  /* 2. Taiwan Taipower 48h TOU + 16:00-22:00 Night-Peak + Feeder Congestion:
   *    26 MW colocated PV peak on a constrained 10 MW utility feeder, 6 MW
   *    rigid + 2.5 MW elastic AI compute + 3 MW chiller cooling, 28 MWh BESS +
   *    18 MWh_th PCM thermal storage */
  static const uint32_t pv_profile_kw[24] = {
      0U,    0U,     0U,     0U,     0U,     0U,     800U,   3500U,
      9000U, 16500U, 22500U, 25500U, 26000U, 24500U, 20000U, 12000U,
      3000U, 0U,     0U,     0U,     0U,     0U,     0U,     0U};

  EleDuckMicrogrid64 tw_mg;
  ele_grid_duck64_init(&tw_mg, 28000U, 12000U, 18000U, 12000U, 10000U, 10000U);

  for (uint32_t step = 0U; step < 48U; ++step) {
    uint32_t hr = step % 24U;
    bool night_peak = (hr >= 16U && hr < 22U);
    int32_t tou_ntd_milli =
        night_peak ? 9600 : ((hr >= 11U && hr < 15U) ? 2100 : 3200);
    uint32_t batch_kw = (step < 44U) ? 2500U : 0U;
    uint32_t cool_kw = (hr >= 10U && hr < 22U) ? 3000U : 1800U;
    (void)ele_grid_step_duck_hour(&tw_mg, pv_profile_kw[hr], 6000U, batch_kw,
                                  cool_kw, tou_ntd_milli, night_peak);
  }

  ELE_ASSERT(tw_mg.batch_backlog_kwh == 0U); /* 100% Compute Conservation */
  ELE_ASSERT(tw_mg.curtailed_pv_base_kwh > 0U);
  ELE_ASSERT(tw_mg.curtailed_pv_symp_kwh < tw_mg.curtailed_pv_base_kwh);
  ELE_ASSERT(tw_mg.peak_evening_symp_kw * 2U < tw_mg.peak_evening_base_kw);
  ELE_ASSERT(tw_mg.ac_conv_waste_kwh > 40000U);
  ELE_ASSERT(tw_mg.cost_symplectic_milli * 2LL < tw_mg.cost_baseline_milli);

  /* 3. US ERCOT / CAISO 48h Deregulated Negative-LMP Canyon Arbitrage:
   *    Midday 10:00-15:00 wholesale LMP = -$35/MWh (-35,000 milli/kWh),
   *    Evening 17:00-21:00 scarcity spike = +$280/MWh (+280,000 milli/kWh) */
  EleDuckMicrogrid64 us_mg;
  ele_grid_duck64_init(&us_mg, 68000U, 8000U, 24000U, 8000U, 14000U, 16000U);

  for (uint32_t step = 0U; step < 48U; ++step) {
    uint32_t hr = step % 24U;
    bool scarcity_peak = (hr >= 16U && hr < 22U);
    int32_t lmp_milli = 42000;
    if (hr >= 10U && hr < 15U) {
      lmp_milli = -35000; /* Negative midday solar canyon price! */
    } else if (hr >= 17U && hr < 21U) {
      lmp_milli = 280000; /* Sunset ramp scarcity spike! */
    }
    uint32_t batch_kw = (step < 44U) ? 2500U : 0U;
    uint32_t cool_kw = (hr >= 10U && hr < 22U) ? 3000U : 1800U;
    (void)ele_grid_step_duck_hour(&us_mg, pv_profile_kw[hr], 6000U, batch_kw,
                                  cool_kw, lmp_milli, scarcity_peak);
  }

  ELE_ASSERT(us_mg.batch_backlog_kwh == 0U);
  ELE_ASSERT(us_mg.curtailed_pv_base_kwh > 50000U);
  ELE_ASSERT(us_mg.curtailed_pv_symp_kwh == 0U);
  ELE_ASSERT(us_mg.cost_symplectic_milli * 4LL < us_mg.cost_baseline_milli);

  uint32_t tw_peak_cut_pct =
      ((tw_mg.peak_evening_base_kw - tw_mg.peak_evening_symp_kw) * 100U) /
      tw_mg.peak_evening_base_kw;
  int64_t us_cost_cut_pct =
      ((us_mg.cost_baseline_milli - us_mg.cost_symplectic_milli) * 100LL) /
      us_mg.cost_baseline_milli;

  printf("  [Suite 8] ele_grid: 60Hz GFM Nadir %u->%u mHz (16.7ms Arrest) | "
         "TW Night-Peak %u->%u kW (-%u%%, HVDC Saved %u kWh) | US Neg-LMP "
         "Curtail %u->%u kWh (Cost -%" PRId64 "%%)\n",
         gfm.nadir_gfl_mhz, gfm.nadir_gfm_mhz, tw_mg.peak_evening_base_kw,
         tw_mg.peak_evening_symp_kw, tw_peak_cut_pct, tw_mg.ac_conv_waste_kwh,
         us_mg.curtailed_pv_base_kwh, us_mg.curtailed_pv_symp_kwh,
         us_cost_cut_pct);
}

static void run_live_dual_chip_hil(uint32_t c6_phase, uint32_t s3_phase,
                                   uint32_t gpio_loop_ok) {
  ELE_ASSERT(gpio_loop_ok == 1U);
  ELE_ASSERT(c6_phase > 10000U);
  ELE_ASSERT(s3_phase > 10000U);

  EleWireLink64 wire;
  ele_wire_link64_init(&wire, 200U, ELE_GEOM_TWISTED_PAIR, 3300U);

  EleSyncNode64 c6_tx;
  EleSyncNode64 s3_rx;
  int64_t base_q16 = (int64_t)c6_phase << 8U;
  ele_sync_node64_init(&c6_tx, base_q16, (64LL << 8U));
  ele_sync_node64_init(&s3_rx, base_q16, (64LL << 8U));

  int32_t phase_diff = (int32_t)s3_phase - (int32_t)c6_phase;
  uint8_t ppm_payload = (uint8_t)(phase_diff & 0xFF);
  uint32_t cyc = ele_wire_transmit_ppm_byte(&wire, 4U, 0xA5U, ppm_payload);
  uint8_t rx_payload = ele_wire_receive_ppm_byte(&wire, 0xA5U, cyc);
  ELE_ASSERT(rx_payload == ppm_payload);

  printf("  [Live HIL] ESP32-C6 (98:A3:16:9C:2A:9C) <-> Cardputer Adv S3 "
         "(E8:F6:0A:98:C3:78) | C6_Phase=%u cyc, S3_Phase=%u cyc (Delta=%d) | "
         "GPIO4->5 5mA Loopback=OK | PPM Byte=0x%02X (%u cyc)\n",
         c6_phase, s3_phase, phase_diff, rx_payload, cyc);
}

static void test_suite0_abi64_univalence(void) {
  ELE_ASSERT(sizeof(EleHalPort64) == 64U && ELE_ALIGNOF(EleHalPort64) == 64U);
  ELE_ASSERT(sizeof(EleDriverCodec64) == 64U &&
             ELE_ALIGNOF(EleDriverCodec64) == 64U);
  ELE_ASSERT(sizeof(EleBusSeq64) == 64U && ELE_ALIGNOF(EleBusSeq64) == 64U);
  ELE_ASSERT(sizeof(EleWireLink64) == 64U && ELE_ALIGNOF(EleWireLink64) == 64U);
  ELE_ASSERT(sizeof(EleSyncNode64) == 64U && ELE_ALIGNOF(EleSyncNode64) == 64U);
  ELE_ASSERT(sizeof(EleMotorDrive64) == 64U &&
             ELE_ALIGNOF(EleMotorDrive64) == 64U);
  ELE_ASSERT(sizeof(EleCsiState64) == 64U && ELE_ALIGNOF(EleCsiState64) == 64U);
  ELE_ASSERT(sizeof(EleCacheBus64) == 64U && ELE_ALIGNOF(EleCacheBus64) == 64U);
  ELE_ASSERT(sizeof(ElePdnGrid64) == 64U && ELE_ALIGNOF(ElePdnGrid64) == 64U);
  ELE_ASSERT(sizeof(EleVrmPhase64) == 64U && ELE_ALIGNOF(EleVrmPhase64) == 64U);
  ELE_ASSERT(sizeof(EleRackBusbar64) == 64U &&
             ELE_ALIGNOF(EleRackBusbar64) == 64U);
  ELE_ASSERT(sizeof(EleGridInverter64) == 64U &&
             ELE_ALIGNOF(EleGridInverter64) == 64U);
  ELE_ASSERT(sizeof(EleDuckMicrogrid64) == 64U &&
             ELE_ALIGNOF(EleDuckMicrogrid64) == 64U);
}

static void test_suite9_adversarial_ub_and_fault_torture(void) {
  /* 1. INT32_MIN / INT32_MAX & 32-bit signed actuator clamping (0 shift UB) */
  EleDriverCodec64 act32;
  ele_driver_codec64_init(&act32, 1U, 0x62U, 0x10U, 0U, 32U,
                          ELE_CODEC_SIGNED_2C | ELE_CODEC_ACTUATOR_CLAMP,
                          -65536, 2147483647, (5 << 16));
  uint8_t b4[4] = {0U, 0U, 0U, 0U};
  uint32_t w_ext =
      ele_driver_encode_actuator_raw(&act32, (-2147483647 - 1), b4);
  ELE_ASSERT(w_ext == 0x7FFFFFFFU);

  /* 2. High-power 48V INA226 shunt monitor without 32-bit numerator wrap:
   *    V_shunt = 100 mV, V_bus = 48 V, R_shunt = 2 mOhm => 2.400 kW */
  uint32_t p_48v_uw = ele_driver_eval_ina226_power_uw(100 << 16, 48 << 16, 2U);
  ELE_ASSERT(p_48v_uw == 2400000000U);
  ELE_ASSERT(ele_driver_eval_ina226_power_uw(100 << 16, 48 << 16, 1U) ==
             0xFFFFFFFFU);

  /* 3. Ternary dot-product entropy with INT32_MIN & INT32_MAX activations */
  const int32_t ext_in[4] = {(-2147483647 - 1), 2147483647, (-2147483647 - 1),
                             0};
  const int8_t ext_w[4] = {-1, 1, 1, -1};
  uint64_t fp_t = 0ULL;
  uint64_t tr_t = 0ULL;
  (void)ele_silicon_compare_dot_toggle_entropy(NULL, ext_in, ext_w, 4U, &fp_t,
                                               &tr_t);
  int64_t ext_sum =
      ele_silicon_dot_ternary_sign_grouped(ext_in, ext_w, 4U, NULL);
  ELE_ASSERT(ext_sum == 2147483647LL);

  /* 4. Uninitialized / zeroed c_out_uf == 0 VRM step & extreme droop safety */
  EleVrmPhase64 z_vrm;
  for (size_t i = 0; i < sizeof(z_vrm); ++i) {
    ((unsigned char *)&z_vrm)[i] = 0U;
  }
  (void)ele_vrm_step_us(&z_vrm, 1000000U, 2000000U, (-2147483647 - 1));
  ELE_ASSERT(z_vrm.active_phases >= 1U);
  ELE_ASSERT(z_vrm.t_dead_ps >= 1500U && z_vrm.t_dead_ps <= 25000U);

  /* 5. Grid EMI spike (period_ns = 1 ns) & INT32_MIN disturbance safety */
  ELE_ASSERT(ele_grid_period_ns_to_mhz(1U) == 0xFFFFFFFFU);
  EleGridInverter64 g_ext;
  ele_grid_inverter64_init(&g_ext, 50000U, 40000U, 18000U, 25000U, 30U);
  int32_t p_emi =
      ele_grid_step_gfm_cycle(&g_ext, 50U, (-2147483647 - 1), 35000U);
  ELE_ASSERT(p_emi == -40000);
  ELE_ASSERT(g_ext.rotor_phase_mrad >= -3142 && g_ext.rotor_phase_mrad <= 3142);
}

int main(int argc, char **argv) {
  printf(
      "=== ELE Zero-Code Silicon Driver & Energy-Optimality Benchmark ===\n");
  test_suite0_abi64_univalence();
  test_suite1_hal_and_wire_surface_charge();
  test_suite2_zero_code_datasheet_driver_and_tickless_bus();
  test_suite3_sync_zero_bit_jet2();
  test_suite4_drive_symplectic_motor();
  test_suite5_espnow_csi_wave();
  test_suite6_silicon_datacenter_power_physics();
  test_suite7_rack_vrm_zvs_and_stranded_power();
  test_suite8_grid_duck_curve_and_gfm_inertia();
  test_suite9_adversarial_ub_and_fault_torture();

  if (argc >= 5 && strcmp(argv[1], "--hil") == 0) {
    uint32_t c6_p = (uint32_t)strtoul(argv[2], NULL, 10);
    uint32_t s3_p = (uint32_t)strtoul(argv[3], NULL, 10);
    uint32_t gpio_ok = (uint32_t)strtoul(argv[4], NULL, 10);
    run_live_dual_chip_hil(c6_p, s3_p, gpio_ok);
  }

  printf("  ✓ ALL ELE SUITES PASSED (%u assertions, 0-ULP, 0 float, 0 heap)\n",
         g_assertions_passed);
  return 0;
}
