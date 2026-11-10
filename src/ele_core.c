#include "../include/ele_csi.h"
#include "../include/ele_drive.h"
#include "../include/ele_driver.h"
#include "../include/ele_hal.h"
#include "../include/ele_silicon.h"
#include "../include/ele_sync.h"
#include "../include/ele_wire.h"

static inline void ele_zero64(void *ptr) {
  uint64_t *p = (uint64_t *)ptr;
  p[0] = 0ULL;
  p[1] = 0ULL;
  p[2] = 0ULL;
  p[3] = 0ULL;
  p[4] = 0ULL;
  p[5] = 0ULL;
  p[6] = 0ULL;
  p[7] = 0ULL;
}

static inline uint32_t ele_popcount8(uint8_t x) {
  uint32_t c = 0U;
  while (x != 0U) {
    x &= (uint8_t)(x - 1U);
    c += 1U;
  }
  return c;
}

/* ============================================================================
 * 0. ele_hal: Cross-MCU Bare-Metal Silicon Port, Pinmux & Power-Domain HAL
 * ============================================================================
 */

void ele_hal_port64_init(EleHalPort64 *port, EleMcuArch arch) {
  if (!port) {
    return;
  }
  ele_zero64(port);
  port->mcu_arch_id = (uint16_t)arch;

  switch (arch) {
  case ELE_MCU_ESP32_C6_RV32:
    port->gpio_base_addr = 0x60091000UL;
    port->iomux_base_addr = 0x60090000UL;
    port->w1ts_set_off = 0x0008U;
    port->w1tc_clr_off = 0x000CU;
    port->gpio_in_off = 0x003CU;
    port->hp_cpu_hz = 160000000UL;
    port->lp_cpu_hz = 20000000UL;
    port->lp_sleep_power_uw = 30U;
    port->hp_active_power_uw = 85000U;
    break;
  case ELE_MCU_ESP32_S3_ULP:
    port->gpio_base_addr = 0x60004000UL;
    port->iomux_base_addr = 0x60009000UL;
    port->w1ts_set_off = 0x0008U;
    port->w1tc_clr_off = 0x000CU;
    port->gpio_in_off = 0x003CU;
    port->hp_cpu_hz = 240000000UL;
    port->lp_cpu_hz = 17500000UL;
    port->lp_sleep_power_uw = 45U;
    port->hp_active_power_uw = 98000U;
    break;
  case ELE_MCU_RP2350_RV32:
    /* RP2350 SIO atomic GPIO_OUT_SET (0x018) / GPIO_OUT_CLR (0x020) */
    port->gpio_base_addr = 0xD0000000UL;
    port->iomux_base_addr = 0x40038000UL;
    port->w1ts_set_off = 0x0018U;
    port->w1tc_clr_off = 0x0020U;
    port->gpio_in_off = 0x0004U;
    port->hp_cpu_hz = 150000000UL;
    port->lp_cpu_hz = 12000000UL;
    port->lp_sleep_power_uw = 80U;
    port->hp_active_power_uw = 26000U;
    break;
  case ELE_MCU_STM32_CM33:
    /* STM32 GPIOA BSRR (0x18) / BRR (0x28) */
    port->gpio_base_addr = 0x42020000UL;
    port->iomux_base_addr = 0x42020000UL;
    port->w1ts_set_off = 0x0018U;
    port->w1tc_clr_off = 0x0028U;
    port->gpio_in_off = 0x0010U;
    port->hp_cpu_hz = 160000000UL;
    port->lp_cpu_hz = 32768UL;
    port->lp_sleep_power_uw = 12U;
    port->hp_active_power_uw = 19000U;
    break;
  case ELE_MCU_HOST_SHADOW:
  default:
    port->hp_cpu_hz = 160000000UL;
    port->lp_cpu_hz = 20000000UL;
    port->lp_sleep_power_uw = 30U;
    port->hp_active_power_uw = 85000U;
    break;
  }
}

void ele_hal_record_sleep_us(EleHalPort64 *port, uint32_t sleep_us) {
  if (!port || sleep_us == 0U) {
    return;
  }
  port->sleep_us_accum += (uint64_t)sleep_us;
  uint32_t net_power_saved_uw =
      (port->hp_active_power_uw > port->lp_sleep_power_uw)
          ? (port->hp_active_power_uw - port->lp_sleep_power_uw)
          : 0U;
  /* Saved energy (uJ, Q48.16) = P_saved(uW) * t(us) / 10^6 */
  int64_t saved_uj_q16 =
      (int64_t)(((uint64_t)net_power_saved_uw * (uint64_t)sleep_us * 65536ULL) /
                1000000ULL);
  port->energy_saved_uj_q16 += saved_uj_q16;
}

/* ============================================================================
 * 1. ele_driver & ele_bus_seq: 64B Zero-Code Datasheet Compiler, CRC-8 Sieve,
 *    1-Cycle Atomic Hot-Swap & Tickless 2-Jet Energy-Proportional Bus Governor
 * ============================================================================
 */

#define ELE_GOLDILOCKS_PRIME 0xFFFFFFFF00000001ULL
#define ELE_SPONGE_ROUND_CONST 0x9E3779B97F4A7C15ULL

static inline uint64_t ele_fp64_add(uint64_t a, uint64_t b) {
  uint64_t p = ELE_GOLDILOCKS_PRIME;
  uint64_t a_red = (a >= p) ? (a - p) : a;
  uint64_t b_red = (b >= p) ? (b - p) : b;
  uint64_t sum = a_red + b_red;
  if (sum < a_red || sum >= p) {
    sum -= p;
  }
  return sum;
}

static inline uint64_t ele_fp64_mul(uint64_t a, uint64_t b) {
  uint64_t p = ELE_GOLDILOCKS_PRIME;
  uint64_t a_red = (a >= p) ? (a - p) : a;
  uint64_t b_red = (b >= p) ? (b - p) : b;

  uint64_t a0 = a_red & 0xFFFFFFFFULL;
  uint64_t a1 = a_red >> 32U;
  uint64_t b0 = b_red & 0xFFFFFFFFULL;
  uint64_t b1 = b_red >> 32U;

  uint64_t p00 = a0 * b0;
  uint64_t p01 = a0 * b1;
  uint64_t p10 = a1 * b0;
  uint64_t p11 = a1 * b1;

  uint64_t mid = (p00 >> 32U) + (p01 & 0xFFFFFFFFULL) + (p10 & 0xFFFFFFFFULL);
  uint64_t lo = (p00 & 0xFFFFFFFFULL) | (mid << 32U);
  uint64_t hi = p11 + (p01 >> 32U) + (p10 >> 32U) + (mid >> 32U);

  uint64_t w2 = hi & 0xFFFFFFFFULL;
  uint64_t w3 = hi >> 32U;
  uint64_t w2_term = (w2 << 32U) - w2;

  uint64_t acc = ele_fp64_add(lo, w2_term);
  uint64_t neg_sub = ele_fp64_add(w3, 0ULL);
  return (acc >= neg_sub) ? (acc - neg_sub) : (acc + p - neg_sub);
}

static inline uint64_t ele_fp64_sponge_step(uint64_t state, uint64_t word) {
  uint64_t mixed =
      ele_fp64_add(state, ele_fp64_add(word & 0x7FFFFFFFFFFFFFFFULL,
                                       ELE_SPONGE_ROUND_CONST));
  uint64_t x2 = ele_fp64_mul(mixed, mixed);
  uint64_t x3 = ele_fp64_mul(x2, mixed);
  uint64_t x6 = ele_fp64_mul(x3, x3);
  return ele_fp64_mul(x6, mixed);
}

uint8_t ele_crc8_compute(const uint8_t *data, uint8_t len, uint8_t poly,
                         uint8_t init_val) {
  if (!data) {
    return init_val;
  }
  uint8_t crc = init_val;
  for (uint8_t i = 0U; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0U; b < 8U; ++b) {
      uint8_t msb = (uint8_t)(crc & 0x80U);
      crc = (uint8_t)(crc << 1U);
      if (msb != 0U) {
        crc ^= poly;
      }
    }
  }
  return crc;
}

uint64_t ele_driver_seal_fp64(uint32_t version, uint16_t bus_dev_addr,
                              uint16_t reg_addr, int32_t scale_q16,
                              int32_t offset_q16, uint8_t bit_shift,
                              uint8_t bit_width, uint8_t codec_flags) {
  uint64_t w0 = ((uint64_t)version << 32U) | ((uint64_t)bus_dev_addr << 16U) |
                (uint64_t)reg_addr;
  uint64_t w1 =
      ((uint64_t)(uint32_t)scale_q16 << 32U) | (uint64_t)(uint32_t)offset_q16;
  uint64_t w2 = ((uint64_t)bit_shift << 16U) | ((uint64_t)bit_width << 8U) |
                (uint64_t)(codec_flags & ELE_CODEC_INVARIANT_MASK);
  uint64_t st = ele_fp64_sponge_step(0ULL, w0);
  st = ele_fp64_sponge_step(st, w1);
  return ele_fp64_sponge_step(st, w2);
}

void ele_driver_codec64_init(EleDriverCodec64 *drv, uint32_t version,
                             uint16_t bus_dev_addr, uint16_t reg_addr,
                             uint8_t bit_shift, uint8_t bit_width,
                             uint8_t codec_flags, int32_t scale_q16,
                             int32_t offset_q16, int32_t max_slew_jump_q16) {
  if (!drv) {
    return;
  }
  ele_zero64(drv);
  uint8_t clean_flags = (uint8_t)(codec_flags & ELE_CODEC_INVARIANT_MASK);
  drv->scale_q16 = scale_q16;
  drv->offset_q16 = offset_q16;
  drv->max_slew_jump_q16 =
      (max_slew_jump_q16 > 0) ? max_slew_jump_q16 : (10 << 16);
  drv->driver_version = version;
  drv->shadow_version = version;
  drv->bus_dev_addr = bus_dev_addr;
  drv->reg_addr = reg_addr;
  drv->bit_shift = (bit_shift < 32U) ? bit_shift : 0U;
  drv->bit_width = (bit_width >= 1U && bit_width <= 32U) ? bit_width : 16U;
  drv->codec_flags = clean_flags;
  drv->bus_health_state = ELE_BUS_HEALTHY;
  drv->shadow_backup_word =
      ((uint64_t)(uint32_t)scale_q16 << 32U) | (uint64_t)(uint32_t)offset_q16;
  drv->driver_seal_fp64 = ele_driver_seal_fp64(
      version, bus_dev_addr, reg_addr, scale_q16, offset_q16, drv->bit_shift,
      drv->bit_width, clean_flags);
}

bool ele_driver_hotswap_atomic64(EleDriverCodec64 *drv, uint32_t new_version,
                                 int32_t new_scale_q16, int32_t new_offset_q16,
                                 uint64_t expected_seal_fp64) {
  if (!drv || new_version <= drv->driver_version) {
    if (drv) {
      drv->bus_health_state |= ELE_BUS_TAMPER_REJECTED;
    }
    return false;
  }
  uint8_t base_flags = (uint8_t)(drv->codec_flags & ELE_CODEC_INVARIANT_MASK);
  uint64_t calc_seal = ele_driver_seal_fp64(
      new_version, drv->bus_dev_addr, drv->reg_addr, new_scale_q16,
      new_offset_q16, drv->bit_shift, drv->bit_width, base_flags);
  if (calc_seal != expected_seal_fp64) {
    drv->bus_health_state |= ELE_BUS_TAMPER_REJECTED;
    return false;
  }

  /* Save working parameters into 64-bit shadow word for 1-cycle rollback */
  drv->shadow_backup_word = ((uint64_t)(uint32_t)drv->scale_q16 << 32U) |
                            (uint64_t)(uint32_t)drv->offset_q16;
  drv->shadow_version = drv->driver_version;

  /* 1-Cycle Atomic Swap: live 2-Jet state (jet_pos, jet_vel, jet_acc) untouched
   */
  drv->scale_q16 = new_scale_q16;
  drv->offset_q16 = new_offset_q16;
  drv->driver_version = new_version;
  drv->driver_seal_fp64 = calc_seal;
  drv->codec_flags |= ELE_CODEC_HOTSWAP_PENDING;
  return true;
}

static inline int32_t ele_driver_eval_affine_q16(const EleDriverCodec64 *drv,
                                                 const uint8_t raw_bytes[4],
                                                 int32_t scale_q16,
                                                 int32_t offset_q16) {
  uint32_t word = 0U;
  if ((drv->codec_flags & ELE_CODEC_BIG_ENDIAN) != 0U) {
    if (drv->bit_width + drv->bit_shift <= 16U) {
      word = ((uint32_t)raw_bytes[0] << 8U) | (uint32_t)raw_bytes[1];
    } else {
      word = ((uint32_t)raw_bytes[0] << 24U) | ((uint32_t)raw_bytes[1] << 16U) |
             ((uint32_t)raw_bytes[2] << 8U) | (uint32_t)raw_bytes[3];
    }
  } else {
    word = (uint32_t)raw_bytes[0] | ((uint32_t)raw_bytes[1] << 8U) |
           ((uint32_t)raw_bytes[2] << 16U) | ((uint32_t)raw_bytes[3] << 24U);
  }

  uint32_t shifted = word >> drv->bit_shift;
  uint32_t mask =
      (drv->bit_width >= 32U) ? 0xFFFFFFFFU : ((1U << drv->bit_width) - 1U);
  uint32_t masked = shifted & mask;

  int32_t signed_raw = (int32_t)masked;
  if ((drv->codec_flags & ELE_CODEC_SIGNED_2C) != 0U && drv->bit_width < 32U) {
    uint32_t sign_bit = 1U << (drv->bit_width - 1U);
    if ((masked & sign_bit) != 0U) {
      signed_raw = (int32_t)(masked | ~mask);
    }
  }

  int64_t scaled = ((int64_t)signed_raw * (int64_t)scale_q16) >> drv->bit_width;
  return (int32_t)scaled + offset_q16;
}

static inline int32_t ele_driver_coast_jet2_step(EleDriverCodec64 *drv,
                                                 uint8_t fault_flags) {
  drv->bus_health_state = (uint8_t)(ELE_BUS_COASTING_2JET | fault_flags);
  int32_t extrap =
      drv->jet_pos_q16 + drv->jet_vel_q16 + (drv->jet_acc_q16 >> 1);
  drv->jet_pos_q16 = extrap;
  drv->jet_vel_q16 = (drv->jet_vel_q16 * 3) >> 2;
  drv->jet_acc_q16 = drv->jet_acc_q16 >> 1;
  return extrap;
}

int32_t ele_driver_decode_and_heal_step(EleDriverCodec64 *drv,
                                        const uint8_t raw_bytes[4],
                                        bool bus_transfer_ok) {
  if (!drv) {
    return 0;
  }

  /* 1. Stuck I2C / Disconnected Wire: 9-Clock Bus Recovery + 2-Jet Coasting */
  if (!bus_transfer_ok || !raw_bytes) {
    drv->recovery_pulses_sent += 9U;
    return ele_driver_coast_jet2_step(drv, ELE_BUS_9CLK_RECOVERED);
  }

  /* 2. Hardware-Free Frame Integrity Sieve (CRC-8 over raw_bytes[0..1] vs [2])
   */
  if ((drv->codec_flags & (ELE_CODEC_CRC8_POLY31 | ELE_CODEC_CRC8_SMBUS07)) !=
      0U) {
    uint8_t poly =
        ((drv->codec_flags & ELE_CODEC_CRC8_POLY31) != 0U) ? 0x31U : 0x07U;
    uint8_t calc_crc = ele_crc8_compute(raw_bytes, 2U, poly, 0x00U);
    if (calc_crc != raw_bytes[2]) {
      return ele_driver_coast_jet2_step(drv, ELE_BUS_CRC_REJECTED);
    }
  }

  int32_t x_cal = ele_driver_eval_affine_q16(drv, raw_bytes, drv->scale_q16,
                                             drv->offset_q16);

  /* 3. Post-Hot-Swap 2-Jet Continuity Self-Check & 1-Cycle Shadow Auto-Rollback
   */
  if ((drv->codec_flags & ELE_CODEC_INITIALIZED) != 0U &&
      (drv->codec_flags & ELE_CODEC_HOTSWAP_PENDING) != 0U) {
    int32_t pred_x =
        drv->jet_pos_q16 + drv->jet_vel_q16 + (drv->jet_acc_q16 >> 1);
    int32_t diff = x_cal - pred_x;
    int32_t abs_diff = (diff >= 0) ? diff : -diff;
    drv->codec_flags &= (uint8_t)~ELE_CODEC_HOTSWAP_PENDING;

    if (abs_diff > drv->max_slew_jump_q16) {
      drv->scale_q16 = (int32_t)(uint32_t)(drv->shadow_backup_word >> 32U);
      drv->offset_q16 =
          (int32_t)(uint32_t)(drv->shadow_backup_word & 0xFFFFFFFFULL);
      drv->driver_version = drv->shadow_version;
      drv->hot_swap_rollbacks += 1U;
      drv->bus_health_state = ELE_BUS_ROLLED_BACK;
      x_cal = ele_driver_eval_affine_q16(drv, raw_bytes, drv->scale_q16,
                                         drv->offset_q16);
    } else {
      drv->bus_health_state = ELE_BUS_HEALTHY;
    }
  } else {
    drv->bus_health_state = ELE_BUS_HEALTHY;
  }

  /* 4. Update 2-Jet state (x, dx/dt, d2x/dt2) */
  if ((drv->codec_flags & ELE_CODEC_INITIALIZED) == 0U) {
    drv->jet_pos_q16 = x_cal;
    drv->jet_vel_q16 = 0;
    drv->jet_acc_q16 = 0;
    drv->codec_flags |= ELE_CODEC_INITIALIZED;
  } else {
    int32_t new_vel = x_cal - drv->jet_pos_q16;
    int32_t new_acc = new_vel - drv->jet_vel_q16;
    drv->jet_pos_q16 = x_cal;
    drv->jet_vel_q16 = (drv->jet_vel_q16 + new_vel) >> 1;
    drv->jet_acc_q16 = (drv->jet_acc_q16 + new_acc) >> 1;
  }

  return drv->jet_pos_q16;
}

void ele_driver_decode_burst_axes(EleDriverCodec64 *axes, uint8_t num_axes,
                                  const uint8_t *burst_buf,
                                  uint8_t stride_bytes, bool bus_transfer_ok,
                                  int32_t *out_q16) {
  if (!axes || num_axes == 0U || stride_bytes == 0U) {
    return;
  }
  for (uint8_t k = 0U; k < num_axes; ++k) {
    uint8_t frame[4] = {0U, 0U, 0U, 0U};
    if (bus_transfer_ok && burst_buf) {
      const uint8_t *src = burst_buf + ((uint32_t)k * (uint32_t)stride_bytes);
      for (uint8_t b = 0U; b < stride_bytes && b < 4U; ++b) {
        frame[b] = src[b];
      }
    }
    int32_t val =
        ele_driver_decode_and_heal_step(&axes[k], frame, bus_transfer_ok);
    if (out_q16) {
      out_q16[k] = val;
    }
  }
}

uint32_t ele_driver_encode_actuator_raw(const EleDriverCodec64 *drv,
                                        int32_t target_q16,
                                        uint8_t out_bytes[4]) {
  if (!drv || drv->scale_q16 == 0) {
    return 0U;
  }
  /* Inverse affine mapping using 32-bit hardware DIV (0 libgcc __divdi3):
   * target_q16 = ((raw * scale_q16) >> bit_width) + offset_q16
   * => raw = ((target_q16 - offset_q16) << bit_width) / scale_q16
   */
  int32_t delta = target_q16 - drv->offset_q16;
  uint8_t bw =
      (drv->bit_width >= 1U && drv->bit_width <= 24U) ? drv->bit_width : 16U;
  int32_t scale_s8 = (drv->scale_q16 >> 8);
  if (scale_s8 == 0) {
    scale_s8 = (drv->scale_q16 > 0) ? 1 : -1;
  }
  int32_t num_s8 = (bw >= 8U) ? (delta << (bw - 8U)) : (delta >> (8U - bw));
  int32_t raw_i32 = num_s8 / scale_s8;

  int32_t min_raw =
      ((drv->codec_flags & ELE_CODEC_SIGNED_2C) != 0U) ? -(1 << (bw - 1U)) : 0;
  int32_t max_raw = ((drv->codec_flags & ELE_CODEC_SIGNED_2C) != 0U)
                        ? ((1 << (bw - 1U)) - 1)
                        : ((int32_t)((1U << bw) - 1U));
  if (raw_i32 < min_raw) {
    raw_i32 = min_raw;
  } else if (raw_i32 > max_raw) {
    raw_i32 = max_raw;
  }

  uint32_t mask = (1U << bw) - 1U;
  uint32_t word = ((uint32_t)raw_i32 & mask) << drv->bit_shift;

  if (out_bytes) {
    if ((drv->codec_flags & ELE_CODEC_BIG_ENDIAN) != 0U) {
      if (bw + drv->bit_shift <= 16U) {
        out_bytes[0] = (uint8_t)((word >> 8U) & 0xFFU);
        out_bytes[1] = (uint8_t)(word & 0xFFU);
        out_bytes[2] = 0U;
        out_bytes[3] = 0U;
      } else {
        out_bytes[0] = (uint8_t)((word >> 24U) & 0xFFU);
        out_bytes[1] = (uint8_t)((word >> 16U) & 0xFFU);
        out_bytes[2] = (uint8_t)((word >> 8U) & 0xFFU);
        out_bytes[3] = (uint8_t)(word & 0xFFU);
      }
    } else {
      out_bytes[0] = (uint8_t)(word & 0xFFU);
      out_bytes[1] = (uint8_t)((word >> 8U) & 0xFFU);
      out_bytes[2] = (uint8_t)((word >> 16U) & 0xFFU);
      out_bytes[3] = (uint8_t)((word >> 24U) & 0xFFU);
    }
  }
  return word;
}

void ele_bus_seq64_init(EleBusSeq64 *seq, EleBusProto proto, uint8_t dev_addr,
                        uint8_t loop_start_pc, uint32_t min_period_us,
                        uint32_t max_period_us, const uint64_t ops[4]) {
  if (!seq) {
    return;
  }
  ele_zero64(seq);
  uint64_t seal = 0ULL;
  for (uint8_t i = 0U; i < 4U; ++i) {
    uint64_t w = ops ? ops[i] : 0ULL;
    seq->micro_ops[i] = w;
    seal = ele_fp64_sponge_step(seal, w);
  }
  uint32_t min_p = (min_period_us > 0U) ? min_period_us : 10000U;
  uint32_t max_p = (max_period_us >= min_p) ? max_period_us : min_p;
  uint64_t meta = ((uint64_t)min_p << 32U) | (uint64_t)max_p;
  seq->seq_seal_fp64 = ele_fp64_sponge_step(seal, meta);
  seq->min_period_us = min_p;
  seq->max_period_us = max_p;
  seq->current_period_us = min_p;
  seq->wake_countdown_us = 0U;
  seq->bus_proto = (uint8_t)proto;
  seq->dev_addr = dev_addr;
  seq->pc_state = (uint8_t)((loop_start_pc & 0x03U) << 2U);
  seq->last_verdict = (uint8_t)ELE_SEQ_SLEEP_TICKLESS;
}

uint32_t ele_bus_govern_period_jet2(EleBusSeq64 *seq,
                                    const EleDriverCodec64 *drv,
                                    int32_t quiet_eps_q16) {
  if (!seq || !drv) {
    return 0U;
  }
  int32_t abs_v =
      (drv->jet_vel_q16 >= 0) ? drv->jet_vel_q16 : -drv->jet_vel_q16;
  int32_t abs_a =
      (drv->jet_acc_q16 >= 0) ? drv->jet_acc_q16 : -drv->jet_acc_q16;
  int32_t activity = abs_v + abs_a;
  int32_t eps = (quiet_eps_q16 >= 0) ? quiet_eps_q16 : 0;

  if (activity > eps || drv->bus_health_state != ELE_BUS_HEALTHY) {
    /* Physical slew surprise or bus fault -> 1-Cycle Snapback to Max Rate */
    seq->current_period_us = seq->min_period_us;
  } else {
    /* Quiescent physical field -> Exponential Tickless Period Expansion */
    uint32_t doubled = (seq->current_period_us <= (seq->max_period_us >> 1U))
                           ? (seq->current_period_us << 1U)
                           : seq->max_period_us;
    if (doubled < seq->min_period_us) {
      doubled = seq->min_period_us;
    }
    seq->current_period_us = doubled;
    if (seq->current_period_us > seq->min_period_us) {
      seq->suppressed_wakeups +=
          (seq->current_period_us / seq->min_period_us) - 1U;
    }
  }
  return seq->current_period_us;
}

EleSeqVerdict ele_bus_seq_step(EleBusSeq64 *seq, EleDriverCodec64 *drv,
                               EleHalPort64 *port, uint8_t periph_regs[256],
                               uint32_t dt_us, int32_t quiet_eps_q16,
                               bool bus_link_ok) {
  if (!seq || !drv) {
    return ELE_SEQ_FAULT_COASTING;
  }

  /* 1. Non-blocking LP Deep-Sleep Gate: zero spin-wait while awaiting timer */
  if (seq->wake_countdown_us > dt_us) {
    seq->wake_countdown_us -= dt_us;
    if (port) {
      ele_hal_record_sleep_us(port, dt_us);
    }
    seq->last_verdict = (uint8_t)ELE_SEQ_SLEEP_TICKLESS;
    return ELE_SEQ_SLEEP_TICKLESS;
  }
  if (seq->wake_countdown_us > 0U && port) {
    ele_hal_record_sleep_us(port, seq->wake_countdown_us);
  }
  seq->wake_countdown_us = 0U;

  if (!bus_link_ok || !periph_regs) {
    (void)ele_driver_decode_and_heal_step(drv, NULL, false);
    seq->wake_countdown_us = seq->min_period_us;
    seq->last_verdict = (uint8_t)ELE_SEQ_FAULT_COASTING;
    return ELE_SEQ_FAULT_COASTING;
  }

  uint8_t pc = (uint8_t)(seq->pc_state & 0x03U);
  uint8_t loop_pc = (uint8_t)((seq->pc_state >> 2U) & 0x03U);
  uint64_t op_word = seq->micro_ops[pc];

  uint8_t opcode = (uint8_t)((op_word >> 56U) & 0xFFU);
  uint8_t reg_addr = (uint8_t)((op_word >> 40U) & 0xFFU);
  uint8_t mask = (uint8_t)((op_word >> 32U) & 0xFFU);
  uint8_t imm = (uint8_t)((op_word >> 24U) & 0xFFU);
  uint8_t burst_len = (uint8_t)((op_word >> 16U) & 0xFFU);
  uint32_t wait_us = (uint32_t)(op_word & 0xFFFFULL) * 10U;

  switch ((EleBusOpcode)opcode) {
  case ELE_OP_WRITE_REG8:
    periph_regs[reg_addr] = imm;
    if (port) {
      port->bus_tx_count += 1U;
    }
    seq->wake_countdown_us = wait_us;
    pc = (uint8_t)((pc + 1U) & 0x03U);
    seq->pc_state = (uint8_t)((seq->pc_state & 0xFCU) | pc);
    seq->last_verdict = (uint8_t)ELE_SEQ_STEP_ISSUED;
    return ELE_SEQ_STEP_ISSUED;

  case ELE_OP_RMW_BITS8:
    periph_regs[reg_addr] = ele_reg_rmw8(periph_regs[reg_addr], mask, imm);
    if (port) {
      port->bus_tx_count += 2U; /* Read + Write */
    }
    seq->wake_countdown_us = wait_us;
    pc = (uint8_t)((pc + 1U) & 0x03U);
    seq->pc_state = (uint8_t)((seq->pc_state & 0xFCU) | pc);
    seq->last_verdict = (uint8_t)ELE_SEQ_STEP_ISSUED;
    return ELE_SEQ_STEP_ISSUED;

  case ELE_OP_CMD_ONLY:
    periph_regs[reg_addr] = imm;
    if (port) {
      port->bus_tx_count += 1U;
    }
    seq->wake_countdown_us = wait_us;
    pc = (uint8_t)((pc + 1U) & 0x03U);
    seq->pc_state = (uint8_t)((seq->pc_state & 0xFCU) | pc);
    seq->last_verdict = (uint8_t)ELE_SEQ_STEP_ISSUED;
    return ELE_SEQ_STEP_ISSUED;

  case ELE_OP_WAIT_US:
    seq->wake_countdown_us = wait_us;
    pc = (uint8_t)((pc + 1U) & 0x03U);
    seq->pc_state = (uint8_t)((seq->pc_state & 0xFCU) | pc);
    seq->last_verdict = (uint8_t)ELE_SEQ_SLEEP_TICKLESS;
    return ELE_SEQ_SLEEP_TICKLESS;

  case ELE_OP_POLL_MASK:
    if (port) {
      port->bus_tx_count += 1U;
    }
    if ((periph_regs[reg_addr] & mask) == (imm & mask)) {
      pc = (uint8_t)((pc + 1U) & 0x03U);
      seq->pc_state = (uint8_t)((seq->pc_state & 0xFCU) | pc);
      seq->last_verdict = (uint8_t)ELE_SEQ_STEP_ISSUED;
      return ELE_SEQ_STEP_ISSUED;
    }
    seq->wake_countdown_us = (wait_us > 0U) ? wait_us : 1000U;
    seq->last_verdict = (uint8_t)ELE_SEQ_SLEEP_TICKLESS;
    return ELE_SEQ_SLEEP_TICKLESS;

  case ELE_OP_BURST_READ: {
    uint8_t frame[4] = {0U, 0U, 0U, 0U};
    uint8_t n = (burst_len >= 1U && burst_len <= 4U) ? burst_len : 4U;
    for (uint8_t b = 0U; b < n; ++b) {
      frame[b] = periph_regs[(uint8_t)(reg_addr + b)];
    }
    if (port) {
      port->bus_tx_count += 1U;
    }
    (void)ele_driver_decode_and_heal_step(drv, frame, true);
    uint32_t next_sleep = ele_bus_govern_period_jet2(seq, drv, quiet_eps_q16);
    seq->wake_countdown_us = next_sleep;
    seq->pc_state = (uint8_t)(0x80U | ((loop_pc & 0x03U) << 2U) | loop_pc);
    if (drv->bus_health_state == ELE_BUS_HEALTHY ||
        drv->bus_health_state == ELE_BUS_ROLLED_BACK) {
      seq->last_verdict = (uint8_t)ELE_SEQ_DATA_READY;
      return ELE_SEQ_DATA_READY;
    }
    seq->last_verdict = (uint8_t)ELE_SEQ_FAULT_COASTING;
    return ELE_SEQ_FAULT_COASTING;
  }

  case ELE_OP_END:
  default:
    seq->pc_state = (uint8_t)(0x80U | ((loop_pc & 0x03U) << 2U) | loop_pc);
    seq->wake_countdown_us = seq->current_period_us;
    seq->last_verdict = (uint8_t)ELE_SEQ_SLEEP_TICKLESS;
    return ELE_SEQ_SLEEP_TICKLESS;
  }
}

void ele_driver_init_ina226_calorimeter(EleDriverCodec64 *shunt_mv_drv,
                                        EleDriverCodec64 *bus_v_drv,
                                        EleBusSeq64 *seq, uint8_t i2c_addr) {
  uint8_t addr = (i2c_addr != 0U) ? i2c_addr : 0x40U;
  if (shunt_mv_drv) {
    /* Reg 0x01 SHUNT_VOLTAGE: Signed 16b BE, 2.5 uV/LSB -> 163.84 mV FS
     * scale_q16 = round(163.84 * 65536) = 10737418 (Q16.16 millivolts) */
    ele_driver_codec64_init(shunt_mv_drv, 1U, addr, 0x01U, 0U, 16U,
                            ELE_CODEC_BIG_ENDIAN | ELE_CODEC_SIGNED_2C,
                            10737418, 0, (40 << 16));
  }
  if (bus_v_drv) {
    /* Reg 0x02 BUS_VOLTAGE: Unsigned 16b BE, 1.25 mV/LSB -> 81.92 V FS
     * scale_q16 = round(81.92 * 65536) = 5368709 (Q16.16 volts) */
    ele_driver_codec64_init(bus_v_drv, 1U, addr, 0x02U, 0U, 16U,
                            ELE_CODEC_BIG_ENDIAN, 5368709, 0, (12 << 16));
  }
  if (seq) {
    const uint64_t ina_ops[4] = {
        /* PC=0: Configure AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, Cont Shunt+Bus */
        ele_bus_op_pack(ELE_OP_RMW_BITS8, 0x00U, 0x0FU, 0x07U, 0U, 0U),
        /* PC=1 (Loop): Poll MASK_ENABLE (0x06) bit 3 (CVRF == 1), 1.1ms LP */
        ele_bus_op_pack(ELE_OP_POLL_MASK, 0x06U, 0x08U, 0x08U, 0U, 110U),
        /* PC=2: Burst Read SHUNT_VOLTAGE (0x01..0x02, 2 bytes) */
        ele_bus_op_pack(ELE_OP_BURST_READ, 0x01U, 0U, 0U, 2U, 0U),
        ele_bus_op_pack(ELE_OP_END, 0U, 0U, 0U, 0U, 0U)};
    ele_bus_seq64_init(seq, ELE_BUS_PROTO_I2C, addr, 1U, 5000U, 250000U,
                       ina_ops);
  }
}

uint32_t ele_driver_eval_ina226_power_uw(int32_t shunt_mv_q16,
                                         int32_t bus_v_q16,
                                         uint32_t r_shunt_mohm) {
  if (bus_v_q16 <= 0 || r_shunt_mohm == 0U) {
    return 0U;
  }
  uint32_t abs_shunt_q16 = (shunt_mv_q16 < 0) ? (0U - (uint32_t)shunt_mv_q16)
                                              : (uint32_t)shunt_mv_q16;
  uint32_t pos_bus_q16 = (uint32_t)bus_v_q16;
  /* prod_mv_v_q16 = V_shunt(mV) * V_bus(V) * 2^16 */
  uint64_t prod_mv_v_q16 =
      ((uint64_t)abs_shunt_q16 * (uint64_t)pos_bus_q16) >> 16U;
  /* P_uW * R_mOhm = V_shunt(mV) * V_bus(V) * 10^6 */
  uint32_t num_uw_mohm = (uint32_t)((prod_mv_v_q16 * 1000000ULL) >> 16U);
  return num_uw_mohm / r_shunt_mohm;
}

/* ============================================================================
 * 2. ele_wire: Surface-Charge Conservation & Waveguide Impedance Matching
 * ============================================================================
 */

EleGpioDriveStrength ele_wire_select_optimal_drive(uint32_t z0_ohms_q16,
                                                   uint32_t c_parasitic_ff) {
  uint32_t z0_ohms = z0_ohms_q16 >> 16U;
  if (z0_ohms >= 200U || c_parasitic_ff <= 4000U) {
    return ELE_DRV_5MA;
  }
  if (z0_ohms >= 95U && c_parasitic_ff <= 16000U) {
    return ELE_DRV_10MA;
  }
  if (c_parasitic_ff <= 35000U) {
    return ELE_DRV_20MA;
  }
  return ELE_DRV_40MA;
}

void ele_wire_link64_init(EleWireLink64 *link, uint32_t length_mm,
                          EleWaveguideGeometry geom, uint32_t vdd_mv) {
  if (!link) {
    return;
  }
  ele_zero64(link);

  uint32_t z0_ohms = 105U;
  uint32_t c_per_mm_ff = 45U;
  if (geom == ELE_GEOM_COPLANAR) {
    z0_ohms = 145U;
    c_per_mm_ff = 38U;
  } else if (geom == ELE_GEOM_OPEN_LOOP) {
    z0_ohms = 290U;
    c_per_mm_ff = 28U;
  }

  link->z0_ohms_q16 = (z0_ohms << 16U);
  link->c_parasitic_ff = 2500U + (length_mm * c_per_mm_ff);
  link->vdd_mv = (vdd_mv > 0U) ? vdd_mv : 3300U;
  link->ppm_base_cycles = 16U;

  EleGpioDriveStrength drv =
      ele_wire_select_optimal_drive(link->z0_ohms_q16, link->c_parasitic_ff);
  link->iomux_shadow_reg = ele_hal_gpio_encode_iomux_drv(0U, drv);
}

uint32_t ele_wire_transmit_ppm_byte(EleWireLink64 *link, uint8_t pin,
                                    uint8_t prev_byte, uint8_t next_byte) {
  if (!link) {
    return 0U;
  }
  uint8_t delta = (uint8_t)(next_byte ^ prev_byte);
  uint8_t gray = ele_wire_encode_gray8(delta);
  uint32_t delay_cycles = link->ppm_base_cycles + ((uint32_t)gray << 2U);

  uint32_t cur_level = (link->gpio_shadow_reg >> (pin & 31U)) & 1U;
  ele_hal_gpio_write(&link->gpio_shadow_reg, pin, (uint8_t)(cur_level ^ 1U));

  link->total_bits_sent += 8ULL;
  link->actual_surface_flips += 1ULL;

  uint32_t uart_flips =
      2U + ele_popcount8((uint8_t)(next_byte ^ (next_byte << 1U)));
  if (uart_flips < 2U) {
    uart_flips = 2U;
  }
  link->baseline_uart_flips += (uint64_t)uart_flips;

  uint64_t v_decivolt = (uint64_t)(link->vdd_mv / 100U);
  int64_t e_single_flip_pj_q16 =
      (int64_t)(((uint64_t)link->c_parasitic_ff * v_decivolt * v_decivolt *
                 65536ULL) /
                200000ULL);
  if (e_single_flip_pj_q16 < 65536LL) {
    e_single_flip_pj_q16 = 65536LL;
  }

  int64_t e_uart_byte_pj_q16 =
      (e_single_flip_pj_q16 * (int64_t)uart_flips * 135LL) / 100LL;
  int64_t geom_factor_num =
      ((link->z0_ohms_q16 >> 16U) >= 250U) ? 150LL : 100LL;
  int64_t e_ppm_byte_pj_q16 = (e_single_flip_pj_q16 * geom_factor_num) / 100LL;

  link->energy_used_pj_q16 += e_ppm_byte_pj_q16;
  if (e_uart_byte_pj_q16 > e_ppm_byte_pj_q16) {
    link->energy_saved_pj_q16 += (e_uart_byte_pj_q16 - e_ppm_byte_pj_q16);
  }

  return delay_cycles;
}

uint8_t ele_wire_receive_ppm_byte(const EleWireLink64 *link, uint8_t prev_byte,
                                  uint32_t measured_cycles) {
  if (!link || measured_cycles < link->ppm_base_cycles) {
    return prev_byte;
  }
  uint32_t offset = measured_cycles - link->ppm_base_cycles;
  uint8_t gray = (uint8_t)(((offset + 2U) >> 2U) & 0xFFU);
  uint8_t delta = ele_wire_decode_gray8(gray);
  return (uint8_t)(prev_byte ^ delta);
}

/* ============================================================================
 * 3. ele_sync: Sensor-Agnostic 0-ULP 2-Jet Predictive Zero-Bit Synchronization
 * ============================================================================
 */

void ele_sync_node64_init(EleSyncNode64 *node, int64_t initial_x_q16,
                          int64_t epsilon_q16) {
  if (!node) {
    return;
  }
  ele_zero64(node);
  node->x_q16 = initial_x_q16;
  node->epsilon_q16 = (epsilon_q16 > 0LL) ? epsilon_q16 : 6553LL;
  node->ambient_temp_c_q16 = (25 << 16);
  node->ambient_rh_pct_q16 = (50U << 16U);
}

EleSyncDecision ele_sync_step_tx_gate(EleSyncNode64 *tx_node,
                                      int64_t measured_x_q16,
                                      int64_t *out_delta_q16) {
  if (!tx_node) {
    return ELE_SYNC_SLEEP_ZERO_BIT;
  }
  int64_t pred_x = ele_sync_predict_jet2_q16(tx_node, 1LL);
  int64_t residual = measured_x_q16 - pred_x;
  int64_t abs_res = (residual >= 0LL) ? residual : -residual;

  if (abs_res <= tx_node->epsilon_q16) {
    tx_node->x_q16 = pred_x;
    tx_node->v_q16 = tx_node->v_q16 + tx_node->a_q16;
    tx_node->last_residual_q16 = 0;
    tx_node->ticks_since_sync += 1U;
    tx_node->suppressed_frames += 1U;
    tx_node->energy_saved_uj_q16 += (145LL << 16U);
    if (out_delta_q16) {
      *out_delta_q16 = 0LL;
    }
    return ELE_SYNC_SLEEP_ZERO_BIT;
  }

  int32_t res32 = (int32_t)residual;
  int32_t dt32 = (int32_t)(tx_node->ticks_since_sync + 1U);
  int32_t dv32 = res32 / dt32;
  int32_t da32 = 0;
  if (tx_node->transmitted_frames >= 1U && dt32 >= 4) {
    da32 = (res32 * 2) / (dt32 * (dt32 + 1));
    dv32 = (res32 * 2) / (dt32 + 1);
  }
  tx_node->x_q16 = measured_x_q16;
  tx_node->v_q16 = tx_node->v_q16 + tx_node->a_q16 + (int64_t)dv32;
  tx_node->a_q16 = tx_node->a_q16 + (int64_t)da32;
  tx_node->last_residual_q16 = res32;
  tx_node->ticks_since_sync = 0U;
  tx_node->transmitted_frames += 1U;
  if (out_delta_q16) {
    *out_delta_q16 = residual;
  }
  return ELE_SYNC_WAKE_TRANSMIT_DELTA;
}

int64_t ele_sync_step_rx_mirror(EleSyncNode64 *rx_node,
                                EleSyncDecision decision,
                                int64_t received_delta_q16) {
  if (!rx_node) {
    return 0LL;
  }
  int64_t pred_x = ele_sync_predict_jet2_q16(rx_node, 1LL);

  if (decision == ELE_SYNC_SLEEP_ZERO_BIT) {
    rx_node->x_q16 = pred_x;
    rx_node->v_q16 = rx_node->v_q16 + rx_node->a_q16;
    rx_node->last_residual_q16 = 0;
    rx_node->ticks_since_sync += 1U;
    rx_node->suppressed_frames += 1U;
    return rx_node->x_q16;
  }

  int32_t res32 = (int32_t)received_delta_q16;
  int32_t dt32 = (int32_t)(rx_node->ticks_since_sync + 1U);
  int32_t dv32 = res32 / dt32;
  int32_t da32 = 0;
  if (rx_node->transmitted_frames >= 1U && dt32 >= 4) {
    da32 = (res32 * 2) / (dt32 * (dt32 + 1));
    dv32 = (res32 * 2) / (dt32 + 1);
  }
  int64_t corrected_x = pred_x + received_delta_q16;
  rx_node->x_q16 = corrected_x;
  rx_node->v_q16 = rx_node->v_q16 + rx_node->a_q16 + (int64_t)dv32;
  rx_node->a_q16 = rx_node->a_q16 + (int64_t)da32;
  rx_node->last_residual_q16 = res32;
  rx_node->ticks_since_sync = 0U;
  rx_node->transmitted_frames += 1U;
  return rx_node->x_q16;
}

/* ============================================================================
 * 4. ele_drive: Symplectic Pulse-and-Coast Motor & Harvester Controller
 * ============================================================================
 */

void ele_drive_motor64_init(EleMotorDrive64 *drv, uint32_t v_supply_mv,
                            uint32_t target_bemf_mv, uint32_t coil_r_mohm) {
  if (!drv) {
    return;
  }
  ele_zero64(drv);
  drv->v_supply_mv = (v_supply_mv > 0U) ? v_supply_mv : 7400U;
  drv->target_bemf_mv = target_bemf_mv;
  drv->v_sat_drop_mv = 1800U;
  drv->coil_r_mohm = (coil_r_mohm > 0U) ? coil_r_mohm : 1500U;
  drv->commutator_quiet_flag = 1U;
}

uint32_t ele_drive_eval_waste_power_mw(const EleMotorDrive64 *drv,
                                       uint32_t current_ma) {
  if (!drv) {
    return 0U;
  }
  uint64_t p_bjt_mw =
      ((uint64_t)current_ma * (uint64_t)drv->v_sat_drop_mv) / 1000ULL;
  uint64_t p_copper_mw = ((uint64_t)current_ma * (uint64_t)current_ma *
                          (uint64_t)drv->coil_r_mohm) /
                         1000000ULL;
  uint64_t p_vib_mw = (uint64_t)(drv->piezo_peak_mv / 25U) +
                      (uint64_t)(drv->mic_diff_rms_uv / 500U);
  return (uint32_t)(p_bjt_mw + p_copper_mw + p_vib_mw);
}

EleDrivePhase ele_drive_step_symplectic(EleMotorDrive64 *drv,
                                        uint32_t measured_bemf_mv,
                                        uint32_t piezo_mv, uint32_t mic_diff_uv,
                                        int32_t heatsink_dtemp_mk_s,
                                        uint32_t dt_us) {
  if (!drv) {
    return ELE_DRIVE_COAST_HIGH_Z;
  }
  drv->v_bemf_mv = measured_bemf_mv;
  drv->piezo_peak_mv = piezo_mv;
  drv->mic_diff_rms_uv = mic_diff_uv;
  drv->heatsink_dtemp_mk_s = heatsink_dtemp_mk_s;

  if (piezo_mv > 100U) {
    int64_t e_piezo_uj_q16 =
        (int64_t)((15ULL * (uint64_t)piezo_mv * (uint64_t)piezo_mv * 65536ULL) /
                  2000000000ULL);
    if (e_piezo_uj_q16 < 1024LL) {
      e_piezo_uj_q16 = 1024LL;
    }
    drv->piezo_harvest_uj_q16 += e_piezo_uj_q16;
  }

  uint32_t v_eff_drive_mv = (drv->v_supply_mv > drv->v_sat_drop_mv)
                                ? (drv->v_supply_mv - drv->v_sat_drop_mv)
                                : 0U;
  uint32_t pwm_headroom_mv = (v_eff_drive_mv > drv->target_bemf_mv)
                                 ? (v_eff_drive_mv - drv->target_bemf_mv)
                                 : 500U;
  uint32_t baseline_pwm_ma = (pwm_headroom_mv * 1000U) / drv->coil_r_mohm;
  uint64_t baseline_power_mw =
      (((uint64_t)baseline_pwm_ma * (uint64_t)drv->v_supply_mv) * 125ULL) /
      100000ULL;
  int64_t baseline_energy_uj_q16 =
      (int64_t)((baseline_power_mw * (uint64_t)dt_us * 65536ULL) / 1000ULL);

  uint32_t hyst_mv = 80U;
  if (measured_bemf_mv > drv->target_bemf_mv + (3U * hyst_mv)) {
    drv->active_phase = (uint8_t)ELE_DRIVE_REGEN_HARVEST_BRAKE;
    drv->pulse_duty_permille = 0U;
    drv->coil_current_ma = 0U;
    uint32_t excess_mv = measured_bemf_mv - drv->target_bemf_mv;
    int64_t regen_uj_q16 =
        (int64_t)(((uint64_t)excess_mv * (uint64_t)dt_us * 65536ULL) /
                  50000ULL);
    drv->piezo_harvest_uj_q16 += regen_uj_q16;
    drv->energy_saved_uj_q16 += baseline_energy_uj_q16;
    return ELE_DRIVE_REGEN_HARVEST_BRAKE;
  }

  uint32_t coast_threshold_mv =
      (drv->active_phase == (uint8_t)ELE_DRIVE_SYMPLECTIC_PULSE)
          ? (drv->target_bemf_mv + hyst_mv)
          : (drv->target_bemf_mv - hyst_mv);

  if (measured_bemf_mv >= coast_threshold_mv || heatsink_dtemp_mk_s > 600 ||
      drv->commutator_quiet_flag == 0U) {
    drv->active_phase = (uint8_t)ELE_DRIVE_COAST_HIGH_Z;
    drv->pulse_duty_permille = 0U;
    drv->coil_current_ma = 0U;
    drv->energy_saved_uj_q16 += baseline_energy_uj_q16;
    return ELE_DRIVE_COAST_HIGH_Z;
  }

  drv->active_phase = (uint8_t)ELE_DRIVE_SYMPLECTIC_PULSE;
  uint32_t deficit_mv = drv->target_bemf_mv - measured_bemf_mv;
  uint32_t duty =
      (deficit_mv * 1000U) / (v_eff_drive_mv > 0U ? v_eff_drive_mv : 1U);
  uint32_t vib_penalty = (piezo_mv / 20U) + (mic_diff_uv / 200U);
  if (duty > vib_penalty + 150U) {
    duty -= vib_penalty;
  }
  if (duty > 850U) {
    duty = 850U;
  }
  if (duty < 120U) {
    duty = 120U;
  }
  drv->pulse_duty_permille = (uint16_t)duty;

  uint32_t net_v_mv = (v_eff_drive_mv > measured_bemf_mv)
                          ? (v_eff_drive_mv - measured_bemf_mv)
                          : 200U;
  drv->coil_current_ma = ((net_v_mv * 1000U) / drv->coil_r_mohm) * duty / 1000U;

  uint64_t pulse_power_mw =
      ((uint64_t)drv->coil_current_ma * (uint64_t)drv->v_supply_mv) / 1000ULL;
  int64_t pulse_energy_uj_q16 =
      (int64_t)((pulse_power_mw * (uint64_t)dt_us * 65536ULL) / 1000ULL);

  drv->energy_input_uj_q16 += pulse_energy_uj_q16;
  if (baseline_energy_uj_q16 > pulse_energy_uj_q16) {
    drv->energy_saved_uj_q16 += (baseline_energy_uj_q16 - pulse_energy_uj_q16);
  }
  return ELE_DRIVE_SYMPLECTIC_PULSE;
}

/* ============================================================================
 * 5. ele_csi: 0-ULP ESP-NOW Wi-Fi CSI Wave Interferometer & ISAC Governor
 * ============================================================================
 */

void ele_csi_state64_init(EleCsiState64 *csi, uint16_t num_subcarriers) {
  if (!csi) {
    return;
  }
  ele_zero64(csi);
  csi->active_subcarriers =
      (num_subcarriers > 0U && num_subcarriers <= 128U) ? num_subcarriers : 64U;
  csi->recommended_rate_hz = 2U;
  csi->tx_power_qdbm = 20U;
  csi->field_regime = (uint8_t)ELE_CSI_FIELD_STATIC_SLEEP;
  csi->last_rssi_dbm = -55;
  csi->noise_floor_dbm = -92;
}

EleCsiFieldRegime ele_csi_ingest_espnow_frame(EleCsiState64 *csi,
                                              const int8_t *iq_buf,
                                              uint16_t buf_len, int8_t rssi_dbm,
                                              int8_t noise_floor_dbm) {
  if (!csi || !iq_buf || buf_len < 4U) {
    return ELE_CSI_FIELD_STATIC_SLEEP;
  }
  csi->last_rssi_dbm = rssi_dbm;
  csi->noise_floor_dbm = noise_floor_dbm;

  uint16_t pairs = buf_len >> 1U;
  if (pairs > csi->active_subcarriers) {
    pairs = csi->active_subcarriers;
  }

  int64_t total_energy = 0LL;
  int64_t cross_holonomy = 0LL;
  uint64_t sig = 0x9E3779B97F4A7C15ULL;

  int32_t prev_i = (int32_t)iq_buf[0];
  int32_t prev_q = (int32_t)iq_buf[1];
  total_energy += (int64_t)(prev_i * prev_i + prev_q * prev_q);

  for (uint16_t k = 1U; k < pairs; ++k) {
    int32_t cur_i = (int32_t)iq_buf[(uint16_t)(k * 2U)];
    int32_t cur_q = (int32_t)iq_buf[(uint16_t)(k * 2U + 1U)];
    int32_t e_k = cur_i * cur_i + cur_q * cur_q;
    int32_t cross_k = cur_i * prev_q - cur_q * prev_i;

    total_energy += (int64_t)e_k;
    cross_holonomy += (int64_t)cross_k;
    sig ^=
        (uint64_t)(uint32_t)(cross_k + (e_k << 8)) + (sig << 6U) + (sig >> 2U);

    prev_i = cur_i;
    prev_q = cur_q;
  }

  csi->coherence_signature = sig;
  int64_t energy_q16 = total_energy << 8U;
  int64_t holonomy_q16 = cross_holonomy << 8U;

  if (csi->frames_processed == 0U) {
    csi->baseline_energy_q16 = energy_q16;
    csi->baseline_holonomy_q16 = holonomy_q16;
    csi->frames_processed = 1U;
    return ELE_CSI_FIELD_STATIC_SLEEP;
  }

  int64_t d_energy = energy_q16 - csi->baseline_energy_q16;
  int64_t d_holo = holonomy_q16 - csi->baseline_holonomy_q16;
  if (d_energy < 0LL) {
    d_energy = -d_energy;
  }
  if (d_holo < 0LL) {
    d_holo = -d_holo;
  }

  int64_t prev_wave = csi->wave_variance_q16;
  int64_t wave_now = (d_energy >> 2U) + d_holo;
  csi->dwave_q16 = wave_now - prev_wave;
  csi->wave_variance_q16 = wave_now;

  uint32_t adapt_shift = (csi->dwave_q16 < 0LL) ? 1U : 6U;
  csi->baseline_energy_q16 +=
      (energy_q16 - csi->baseline_energy_q16) >> adapt_shift;
  csi->baseline_holonomy_q16 +=
      (holonomy_q16 - csi->baseline_holonomy_q16) >> adapt_shift;
  csi->frames_processed += 1U;

  int32_t snr_db = (int32_t)rssi_dbm - (int32_t)noise_floor_dbm;
  if (snr_db > 35) {
    csi->tx_power_qdbm = 8U;
  } else if (snr_db > 25) {
    csi->tx_power_qdbm = 24U;
  } else if (snr_db > 16) {
    csi->tx_power_qdbm = 48U;
  } else {
    csi->tx_power_qdbm = 80U;
  }

  int64_t micro_thresh_q16 = 1200LL << 8U;
  int64_t macro_thresh_q16 = 12000LL << 8U;

  if (wave_now >= macro_thresh_q16) {
    csi->field_regime = (uint8_t)ELE_CSI_FIELD_MACRO_MOTION;
    csi->recommended_rate_hz = 100U;
  } else if (wave_now >= micro_thresh_q16) {
    csi->field_regime = (uint8_t)ELE_CSI_FIELD_MICRO_WAVE;
    csi->recommended_rate_hz = 25U;
    csi->suppressed_beacons += 3U;
  } else {
    csi->field_regime = (uint8_t)ELE_CSI_FIELD_STATIC_SLEEP;
    csi->recommended_rate_hz = 2U;
    csi->suppressed_beacons += 49U;
  }

  int64_t saved_per_step_uj = (int64_t)(100U - csi->recommended_rate_hz) * 9LL +
                              (int64_t)(80U - csi->tx_power_qdbm) * 4LL;
  csi->energy_saved_uj_q16 += (saved_per_step_uj << 16U);

  return (EleCsiFieldRegime)csi->field_regime;
}

/* ============================================================================
 * 6. ele_silicon: HBM/UCIe Charge-Minimal Bus, Symplectic PDN dI/dt Governor,
 *    CMOS Gate-Toggle Entropy Calorimeter & 2-Jet Thermal-Leakage Barrier
 * ============================================================================
 */

static inline uint32_t ele_popcount64(uint64_t x) {
  x -= (x >> 1U) & 0x5555555555555555ULL;
  x = (x & 0x3333333333333333ULL) + ((x >> 2U) & 0x3333333333333333ULL);
  x = (x + (x >> 4U)) & 0x0F0F0F0F0F0F0F0FULL;
  uint32_t lo = (uint32_t)x + (uint32_t)(x >> 32U);
  lo += lo >> 8U;
  lo += lo >> 16U;
  return lo & 0x7FU;
}

static inline uint32_t ele_popcount32(uint32_t x) {
  x -= (x >> 1U) & 0x55555555U;
  x = (x & 0x33333333U) + ((x >> 2U) & 0x33333333U);
  x = (x + (x >> 4U)) & 0x0F0F0F0FU;
  x += x >> 8U;
  x += x >> 16U;
  return x & 0x3FU;
}

static inline uint32_t ele_clz32(uint32_t x) {
  if (x == 0U) {
    return 32U;
  }
  uint32_t n = 0U;
  if ((x & 0xFFFF0000U) == 0U) {
    n += 16U;
    x <<= 16U;
  }
  if ((x & 0xFF000000U) == 0U) {
    n += 8U;
    x <<= 8U;
  }
  if ((x & 0xF0000000U) == 0U) {
    n += 4U;
    x <<= 4U;
  }
  if ((x & 0xC0000000U) == 0U) {
    n += 2U;
    x <<= 2U;
  }
  if ((x & 0x80000000U) == 0U) {
    n += 1U;
  }
  return n;
}

void ele_silicon_bus64_init(EleCacheBus64 *bus, uint16_t line_cap_ff,
                            uint16_t vdd_mv) {
  if (!bus) {
    return;
  }
  ele_zero64(bus);
  bus->line_cap_ff = (line_cap_ff > 0U) ? line_cap_ff : 1200U;
  uint32_t v10 = (vdd_mv > 0U) ? (vdd_mv / 10U) : 75U;
  bus->vdd_mv_div10 = (uint8_t)((v10 > 255U) ? 255U : v10);
}

uint8_t ele_silicon_tx_cacheline64(EleCacheBus64 *bus,
                                   const uint64_t next_line[8],
                                   const uint64_t pred_line[8],
                                   uint64_t out_wire[8]) {
  if (!bus || !next_line) {
    return 0U;
  }
  uint8_t inv_mask8 = 0U;
  uint32_t e_flip_pj_q16 =
      ((uint32_t)bus->line_cap_ff * (uint32_t)bus->vdd_mv_div10 *
       (uint32_t)bus->vdd_mv_div10) /
      305U;
  if (e_flip_pj_q16 == 0U) {
    e_flip_pj_q16 = 64U;
  }

  /* Process 64B cache line as 2 x 256-bit HBM3 pseudo-channel beats */
  for (uint8_t beat = 0U; beat < 2U; ++beat) {
    uint32_t beat_raw_flips = 0U;
    uint32_t beat_enc_flips = 0U;
    uint8_t next_inv4 = 0U;

    for (uint8_t lane = 0U; lane < 4U; ++lane) {
      uint8_t idx = (uint8_t)(beat * 4U + lane);
      uint64_t raw_w = next_line[idx];
      uint64_t prev_w = bus->prev_bus_beat[lane];

      /* Baseline uncoded wire flips vs previous physical beat */
      beat_raw_flips += ele_popcount64(raw_w ^ prev_w);

      /* 1. Optional 2-Jet / XOR temporal cache-line residual */
      uint64_t cand0 = pred_line ? (raw_w ^ pred_line[idx]) : raw_w;

      /* 2. Stateful 65-wire Bus-Invert (64 data + 1 polarity wire):
       *    flips(p=0) + flips(p=1) == 65 => min <= 32 strictly!
       */
      uint8_t prev_p = (uint8_t)((bus->prev_inv_mask4 >> lane) & 1U);
      uint32_t flips_p0 = ele_popcount64(cand0 ^ prev_w) + (uint32_t)prev_p;
      uint64_t chosen_w = cand0;
      uint32_t chosen_flips = flips_p0;

      if (flips_p0 > 32U) {
        chosen_w = ~cand0;
        chosen_flips = 65U - flips_p0;
        next_inv4 |= (uint8_t)(1U << lane);
        inv_mask8 |= (uint8_t)(1U << idx);
      }

      bus->prev_bus_beat[lane] = chosen_w;
      if (out_wire) {
        out_wire[idx] = chosen_w;
      }
      beat_enc_flips += chosen_flips;
    }

    bus->prev_inv_mask4 = next_inv4;
    bus->raw_unencoded_flips += (uint64_t)beat_raw_flips;
    bus->encoded_wire_flips += (uint64_t)beat_enc_flips;
    if (beat_enc_flips == 0U) {
      bus->zero_beat_skips += 1U;
    }
    if (beat_raw_flips > beat_enc_flips) {
      uint32_t saved_flips = beat_raw_flips - beat_enc_flips;
      bus->energy_saved_pj_q16 += (int64_t)saved_flips * (int64_t)e_flip_pj_q16;
    }
  }

  return inv_mask8;
}

void ele_silicon_rx_cacheline64(const uint64_t wire_words[8],
                                const uint64_t pred_line[8], uint8_t inv_mask8,
                                uint64_t out_line[8]) {
  if (!wire_words || !out_line) {
    return;
  }
  for (uint8_t i = 0U; i < 8U; ++i) {
    uint64_t w = ((inv_mask8 >> i) & 1U) ? ~wire_words[i] : wire_words[i];
    out_line[i] = pred_line ? (w ^ pred_line[i]) : w;
  }
}

void ele_silicon_pdn64_init(ElePdnGrid64 *pdn, uint32_t v_nom_mv,
                            uint32_t l_droop_uv_per_a,
                            uint32_t max_slew_amp_tick, int32_t ambient_mc,
                            int32_t temp_knee_mc) {
  if (!pdn) {
    return;
  }
  ele_zero64(pdn);
  pdn->v_nom_mv = (v_nom_mv > 0U) ? v_nom_mv : 700U;
  pdn->l_droop_uv_per_a = (l_droop_uv_per_a > 0U) ? l_droop_uv_per_a : 250U;
  pdn->max_slew_amp_tick = (max_slew_amp_tick > 0U) ? max_slew_amp_tick : 80U;
  pdn->temp_mc = ambient_mc;
  pdn->temp_knee_mc = (temp_knee_mc > ambient_mc) ? temp_knee_mc : 78000;
}

uint32_t ele_silicon_step_pdn_symplectic(ElePdnGrid64 *pdn,
                                         uint32_t prev_sync_amp,
                                         uint32_t req_sync_amp) {
  if (!pdn) {
    return 0U;
  }
  /* 1. Unscheduled synchronous inductive rise Vdroop = L * max(0, dI/dt) */
  uint32_t sync_rise =
      (req_sync_amp > prev_sync_amp) ? (req_sync_amp - prev_sync_amp) : 0U;
  uint32_t sync_droop_mv = (sync_rise * pdn->l_droop_uv_per_a) / 1000U;
  if (sync_droop_mv > pdn->peak_vdroop_sync_mv) {
    pdn->peak_vdroop_sync_mv = sync_droop_mv;
  }

  /* 2. Symplectic slew-bounded current shaping with 100% work conservation */
  uint32_t total_work = pdn->backlog_work_amp_ticks + req_sync_amp;
  uint32_t hi_lim = pdn->cur_current_amp + pdn->max_slew_amp_tick;
  uint32_t lo_lim = (pdn->cur_current_amp > pdn->max_slew_amp_tick)
                        ? (pdn->cur_current_amp - pdn->max_slew_amp_tick)
                        : 0U;
  uint32_t next_amp = total_work;
  if (next_amp > hi_lim) {
    next_amp = hi_lim;
  } else if (next_amp < lo_lim && total_work >= lo_lim) {
    next_amp = lo_lim;
  }

  uint32_t symp_rise = (next_amp > pdn->cur_current_amp)
                           ? (next_amp - pdn->cur_current_amp)
                           : 0U;
  uint32_t symp_droop_mv = (symp_rise * pdn->l_droop_uv_per_a) / 1000U;
  if (symp_droop_mv > pdn->peak_vdroop_symp_mv) {
    pdn->peak_vdroop_symp_mv = symp_droop_mv;
  }

  pdn->backlog_work_amp_ticks = total_work - next_amp;
  pdn->cur_current_amp = next_amp;

  /* 3. Dynamic V_DD^2 guardband + PDN copper I^2*R (0.5 mOhm) energy (1 ms) */
  uint32_t v_sync = pdn->v_nom_mv + pdn->peak_vdroop_sync_mv;
  uint32_t v_symp = pdn->v_nom_mv + pdn->peak_vdroop_symp_mv;
  uint32_t v_sync_eff = (v_sync * v_sync) / pdn->v_nom_mv;
  uint32_t v_symp_eff = (v_symp * v_symp) / pdn->v_nom_mv;

  int64_t e_sync_mj = (int64_t)((req_sync_amp * v_sync_eff) / 1000U) +
                      (int64_t)((req_sync_amp * req_sync_amp) / 2000U);
  int64_t e_symp_mj = (int64_t)((next_amp * v_symp_eff) / 1000U) +
                      (int64_t)((next_amp * next_amp) / 2000U);
  pdn->pdn_energy_saved_mj += (e_sync_mj - e_symp_mj);

  return next_amp;
}

uint32_t ele_silicon_eval_leakage_mw(int32_t temp_mc, uint32_t base_leak_mw) {
  uint32_t base = (base_leak_mw > 0U) ? base_leak_mw : 15000U;
  if (temp_mc <= 45000) {
    return base;
  }
  /* Subthreshold silicon leakage doubles every ~12 C above 45 C:
   *   2^(oct + rem/12000) approximated cleanly via (12000 + rem) / 12000
   */
  uint32_t dt_mc = (uint32_t)(temp_mc - 45000);
  uint32_t octaves = dt_mc / 12000U;
  uint32_t rem_mc = dt_mc - (octaves * 12000U);
  if (octaves > 6U) {
    octaves = 6U;
  }
  uint32_t scaled = (base * (12000U + rem_mc)) / 12000U;
  return scaled << octaves;
}

uint32_t ele_silicon_step_thermal_jet2(ElePdnGrid64 *pdn,
                                       uint32_t active_power_w,
                                       int32_t ambient_mc,
                                       uint32_t horizon_ticks) {
  if (!pdn) {
    return 0U;
  }
  uint32_t leak_mw = ele_silicon_eval_leakage_mw(pdn->temp_mc, 15000U);
  uint32_t total_w = active_power_w + (leak_mw / 1000U);

  /* Silicon junction RC step: heating +8 mC/W, cooling (T - T_amb) / 12 */
  int32_t cooling_mc = (pdn->temp_mc - ambient_mc) / 12;
  int32_t next_temp_mc = pdn->temp_mc + (int32_t)(total_w * 8U) - cooling_mc;
  int32_t new_vel = next_temp_mc - pdn->temp_mc;
  int32_t new_acc = new_vel - pdn->dtemp_mc_tick;

  pdn->temp_mc = next_temp_mc;
  pdn->dtemp_mc_tick = new_vel;
  pdn->ddtemp_mc_tick2 = (pdn->ddtemp_mc_tick2 + new_acc) >> 1;

  int32_t h = (int32_t)((horizon_ticks > 0U) ? horizon_ticks : 4U);
  int32_t pos_acc = (pdn->ddtemp_mc_tick2 > 0) ? pdn->ddtemp_mc_tick2 : 0;
  int32_t pred_temp_mc =
      pdn->temp_mc + (pdn->dtemp_mc_tick * h) + ((pos_acc * h * h) >> 1);

  if (pred_temp_mc >= pdn->temp_knee_mc) {
    pdn->thermal_preempts += 1U;
    return 180U; /* Pre-emptive migration / cool-carrier current cap (Amps) */
  }
  return 600U;
}

/* Pure integer IEEE-754 binary32 bit-pattern packer from Q16.16 (0 FPU ops!) */
static inline uint32_t ele_pack_ieee754_f32_bits_q16(int32_t val_q16) {
  if (val_q16 == 0) {
    return 0U;
  }
  uint32_t sign = (val_q16 < 0) ? 0x80000000U : 0U;
  uint32_t mag = (val_q16 < 0) ? (0U - (uint32_t)val_q16) : (uint32_t)val_q16;
  uint32_t lz = ele_clz32(mag);
  uint32_t msb_pos = 31U - lz;
  /* Value is mag * 2^-16 => unbiased exponent = msb_pos - 16 */
  uint32_t exp_biased = (msb_pos + 127U) - 16U;
  uint32_t norm = (mag << lz) & 0x7FFFFFFFU; /* Strip implicit leading 1 */
  uint32_t mant23 = norm >> 8U;
  return sign | (exp_biased << 23U) | mant23;
}

uint64_t ele_silicon_compare_dot_toggle_entropy(
    ElePdnGrid64 *pdn, const int32_t *inputs_q16, const int8_t *weights_trit,
    uint32_t len, uint64_t *out_fp32_toggles, uint64_t *out_ternary_toggles) {
  uint64_t fp32_flips = 0ULL;
  uint64_t tern_flips = 0ULL;
  if (!inputs_q16 || !weights_trit) {
    if (out_fp32_toggles) {
      *out_fp32_toggles = 0ULL;
    }
    if (out_ternary_toggles) {
      *out_ternary_toggles = 0ULL;
    }
    return 0ULL;
  }

  uint32_t prev_fp_in = 0U;
  uint32_t prev_fp_w = 0U;
  uint32_t prev_fp_prod = 0U;
  uint32_t prev_fp_acc = 0U;
  int32_t acc_q16 = 0;

  uint32_t prev_gray_acc = 0U;
  uint8_t prev_trit_code = 0U;

  for (uint32_t i = 0U; i < len; ++i) {
    int32_t x = inputs_q16[i];
    int8_t w = weights_trit[i];
    int32_t prod_q16 = (w > 0) ? x : ((w < 0) ? -x : 0);
    acc_q16 += prod_q16;

    /* 1. IEEE-754 FP32 datapath register toggles (input, weight, FMA, acc) */
    uint32_t fp_in = ele_pack_ieee754_f32_bits_q16(x);
    uint32_t fp_w = ele_pack_ieee754_f32_bits_q16((int32_t)w * 65536 +
                                                  ((int32_t)(i & 3U) * 64));
    uint32_t fp_prod = ele_pack_ieee754_f32_bits_q16(prod_q16);
    uint32_t fp_acc = ele_pack_ieee754_f32_bits_q16(acc_q16);

    fp32_flips += (uint64_t)ele_popcount32(fp_in ^ prev_fp_in);
    fp32_flips += (uint64_t)ele_popcount32(fp_w ^ prev_fp_w);
    fp32_flips += (uint64_t)ele_popcount32(fp_prod ^ prev_fp_prod);
    fp32_flips += (uint64_t)ele_popcount32(fp_acc ^ prev_fp_acc);

    prev_fp_in = fp_in;
    prev_fp_w = fp_w;
    prev_fp_prod = fp_prod;
    prev_fp_acc = fp_acc;

    /* 2. 1.58-Bit Ternary (0 multipliers, clock-gated on w == 0, Gray acc) */
    uint8_t trit_code = (w > 0) ? 1U : ((w < 0) ? 2U : 0U);
    tern_flips +=
        (uint64_t)ele_popcount8((uint8_t)(trit_code ^ prev_trit_code));
    prev_trit_code = trit_code;
    if (w != 0) {
      uint32_t uacc = (uint32_t)acc_q16;
      uint32_t gray_acc = uacc ^ (uacc >> 1U);
      tern_flips += (uint64_t)ele_popcount32(gray_acc ^ prev_gray_acc);
      prev_gray_acc = gray_acc;
    }
  }

  if (out_fp32_toggles) {
    *out_fp32_toggles = fp32_flips;
  }
  if (out_ternary_toggles) {
    *out_ternary_toggles = tern_flips;
  }
  uint64_t saved = (fp32_flips > tern_flips) ? (fp32_flips - tern_flips) : 0ULL;
  if (pdn) {
    pdn->gate_toggle_saved_bits += saved;
  }
  return saved;
}

int64_t ele_silicon_dot_ternary_sign_grouped(const int32_t *inputs_q16,
                                             const int8_t *weights_trit,
                                             uint32_t len,
                                             uint64_t *out_grouped_toggles) {
  if (!inputs_q16 || !weights_trit) {
    if (out_grouped_toggles) {
      *out_grouped_toggles = 0ULL;
    }
    return 0LL;
  }

  /* Dual monotonic unsigned magnitude rails:
   *   - Pass 1 accumulates strictly positive contributions (w * x > 0)
   *   - Pass 2 accumulates strictly negative magnitudes (w * x < 0)
   *   - Zero weights (w == 0) are skipped (0 toggles)
   * Because neither unsigned rail ever crosses zero, high-order sign-extension
   * bits [31:20] never experience alternating 0<->1 two's-complement carry
   * avalanches, while (int64_t)pos_rail - (int64_t)neg_rail is 0-ULP exact. */
  uint64_t pos_rail = 0ULL;
  uint64_t neg_rail = 0ULL;
  uint64_t toggles = 0ULL;
  uint32_t prev_gray_pos = 0U;
  uint32_t prev_gray_neg = 0U;

  for (uint32_t i = 0U; i < len; ++i) {
    int8_t w = weights_trit[i];
    int32_t x = inputs_q16[i];
    if (w == 0 || x == 0) {
      continue;
    }
    bool term_pos = ((w > 0) && (x > 0)) || ((w < 0) && (x < 0));
    uint32_t mag = (x < 0) ? (0U - (uint32_t)x) : (uint32_t)x;
    if (term_pos) {
      pos_rail += (uint64_t)mag;
      uint32_t upos = (uint32_t)pos_rail;
      uint32_t gpos = upos ^ (upos >> 1U);
      toggles += (uint64_t)ele_popcount32(gpos ^ prev_gray_pos);
      prev_gray_pos = gpos;
    } else {
      neg_rail += (uint64_t)mag;
      uint32_t uneg = (uint32_t)neg_rail;
      uint32_t gneg = uneg ^ (uneg >> 1U);
      toggles += (uint64_t)ele_popcount32(gneg ^ prev_gray_neg);
      prev_gray_neg = gneg;
    }
  }

  /* Single terminal subtraction (1 sign-domain merge over the entire vector) */
  int64_t exact_sum = (int64_t)pos_rail - (int64_t)neg_rail;
  uint32_t ufinal = (uint32_t)exact_sum;
  uint32_t gfinal = ufinal ^ (ufinal >> 1U);
  toggles += (uint64_t)ele_popcount32(gfinal ^ prev_gray_pos);

  if (out_grouped_toggles) {
    *out_grouped_toggles = toggles;
  }
  return exact_sum;
}
