#include "../include/ele_csi.h"
#include "../include/ele_drive.h"
#include "../include/ele_driver.h"
#include "../include/ele_event_adc.h"
#include "../include/ele_grid.h"
#include "../include/ele_hal.h"
#include "../include/ele_hbm_refresh.h"
#include "../include/ele_hvdc_sscb.h"
#include "../include/ele_ivr_fll.h"
#include "../include/ele_pdn_emi.h"
#include "../include/ele_photonics_cpo.h"
#include "../include/ele_rack_vrm.h"
#include "../include/ele_resonant_clk.h"
#include "../include/ele_silicon.h"
#include "../include/ele_silicon_slm.h"
#include "../include/ele_sync.h"
#include "../include/ele_thermo_fluid.h"
#include "../include/ele_wire.h"

#if defined(__GNUC__) || defined(__clang__)
typedef uint64_t __attribute__((__may_alias__)) ele_u64_alias_t;
#else
typedef uint64_t ele_u64_alias_t;
#endif

static inline void ele_zero64(void *ptr) {
  volatile ele_u64_alias_t *p = (volatile ele_u64_alias_t *)ptr;
  p[0] = 0ULL;
  p[1] = 0ULL;
  p[2] = 0ULL;
  p[3] = 0ULL;
  p[4] = 0ULL;
  p[5] = 0ULL;
  p[6] = 0ULL;
  p[7] = 0ULL;
}

static inline uint32_t ele_abs_i32_u32(int32_t x) {
  return (x < 0) ? (0U - (uint32_t)x) : (uint32_t)x;
}

static inline int32_t ele_sat_i64_i32(int64_t x) {
  if (x > 2147483647LL) {
    return 2147483647;
  }
  if (x < (-2147483647LL - 1LL)) {
    return (-2147483647 - 1);
  }
  return (int32_t)x;
}

/* 32-bit RV32I-native 64-bit logical shifts (eliminates __ashldi3 / __lshrdi3
 * across all optimization levels including -Oz). */
static inline uint64_t ele_shl64(uint64_t x, uint32_t sh) {
  if (sh == 0U) {
    return x;
  }
  if (sh >= 64U) {
    return 0ULL;
  }
  uint32_t lo = (uint32_t)x;
  uint32_t hi = (uint32_t)(x >> 32U);
  if (sh >= 32U) {
    return (uint64_t)(lo << (sh - 32U)) << 32U;
  }
  uint32_t out_hi = (hi << sh) | (lo >> (32U - sh));
  uint32_t out_lo = lo << sh;
  return ((uint64_t)out_hi << 32U) | (uint64_t)out_lo;
}

static inline uint64_t ele_shr64(uint64_t x, uint32_t sh) {
  if (sh == 0U) {
    return x;
  }
  if (sh >= 64U) {
    return 0ULL;
  }
  uint32_t lo = (uint32_t)x;
  uint32_t hi = (uint32_t)(x >> 32U);
  if (sh >= 32U) {
    return (uint64_t)(hi >> (sh - 32U));
  }
  uint32_t out_hi = hi >> sh;
  uint32_t out_lo = (lo >> sh) | (hi << (32U - sh));
  return ((uint64_t)out_hi << 32U) | (uint64_t)out_lo;
}

/* Portable ISO C17 arithmetic right shift (exact floor division by 2^sh,
 * bit-for-bit identical to SystemVerilog signed `>>>` with zero UB). */
static inline int64_t ele_asr64(int64_t x, uint32_t sh) {
  if (sh == 0U) {
    return x;
  }
  if (x >= 0LL) {
    return (int64_t)ele_shr64((uint64_t)x, sh);
  }
  uint64_t bias = ele_shl64(1ULL, sh) - 1ULL;
  return -(int64_t)ele_shr64((0ULL - (uint64_t)x) + bias, sh);
}

static inline uint32_t ele_popcount32(uint32_t x) {
  x -= (x >> 1U) & 0x55555555U;
  x = (x & 0x33333333U) + ((x >> 2U) & 0x33333333U);
  x = (x + (x >> 4U)) & 0x0F0F0F0FU;
#if defined(__riscv)
  __asm__ volatile("" : "+r"(x));
#endif
  x += x >> 8U;
  x += x >> 16U;
  return x & 0x3FU;
}

static inline uint32_t ele_popcount8(uint8_t x) {
  return ele_popcount32((uint32_t)x);
}

/* 0-ULP 64-by-32 unsigned division using only 32-bit RV32M hardware DIVU and
 * 32-bit shifts (eliminates __udivdi3 / __divdi3 across all Clang/GCC versions
 * and optimization levels -O0..-O3/-Os/-Oz). */
static inline uint64_t ele_div_u64_u32(uint64_t num, uint32_t den) {
  if (den == 0U) {
    return 0ULL;
  }
  uint32_t hi = (uint32_t)(num >> 32U);
  uint32_t lo = (uint32_t)num;
  if (hi == 0U) {
    return (uint64_t)(lo / den);
  }
  uint32_t q_hi = hi / den;
  uint32_t rem = hi - (q_hi * den);
  if (den <= 0xFFFFU) {
    uint32_t mid = (rem << 16U) | (lo >> 16U);
    uint32_t q_mid = mid / den;
    uint32_t r_mid = mid - (q_mid * den);
    uint32_t bot = (r_mid << 16U) | (lo & 0xFFFFU);
    uint32_t q_bot = bot / den;
    return ((uint64_t)q_hi << 32U) | (uint64_t)((q_mid << 16U) | q_bot);
  }
  uint32_t q_lo = 0U;
  for (uint32_t i = 0U; i < 32U; ++i) {
    uint32_t carry = rem >> 31U;
    rem = (rem << 1U) | (lo >> 31U);
    lo <<= 1U;
    q_lo <<= 1U;
    if (carry != 0U || rem >= den) {
      rem -= den;
      q_lo |= 1U;
    }
  }
  return ((uint64_t)q_hi << 32U) | (uint64_t)q_lo;
}

static inline int64_t ele_sdiv_i64_u32(int64_t num, uint32_t den) {
  if (num == 0LL || den == 0U) {
    return 0LL;
  }
  if (num > 0LL) {
    return (int64_t)ele_div_u64_u32((uint64_t)num, den);
  }
  uint64_t mag = 0ULL - (uint64_t)num;
  return -(int64_t)ele_div_u64_u32(mag, den);
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
  /* Saved energy (uJ, Q48.16) = P_saved(uW) * t(us) / 10^6 without 64b wrap */
  uint64_t prod_uw_us = (uint64_t)net_power_saved_uw * (uint64_t)sleep_us;
  uint64_t q_whole = ele_div_u64_u32(prod_uw_us, 1000000U);
  uint64_t rem = prod_uw_us - (q_whole * 1000000ULL);
  uint64_t q_frac = ele_div_u64_u32(rem << 16U, 1000000U);
  int64_t saved_uj_q16 = (int64_t)((q_whole << 16U) + q_frac);
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

  int64_t prod = (int64_t)signed_raw * (int64_t)scale_q16;
  uint8_t bw = drv->bit_width;
  int32_t scaled;
  if (bw == 0U) {
    scaled = (int32_t)prod;
  } else if (bw >= 32U) {
    scaled = (int32_t)(prod >> 32U);
  } else {
    uint32_t lo = (uint32_t)((uint64_t)prod);
    uint32_t hi = (uint32_t)((uint64_t)prod >> 32U);
    scaled = (int32_t)((lo >> bw) | (hi << (32U - bw)));
  }
  return ele_sat_i64_i32((int64_t)scaled + (int64_t)offset_q16);
}

static inline int32_t ele_driver_coast_jet2_step(EleDriverCodec64 *drv,
                                                 uint8_t fault_flags) {
  drv->bus_health_state = (uint8_t)(ELE_BUS_COASTING_2JET | fault_flags);
  int32_t extrap =
      ele_sat_i64_i32((int64_t)drv->jet_pos_q16 + (int64_t)drv->jet_vel_q16 +
                      ele_asr64((int64_t)drv->jet_acc_q16, 1U));
  drv->jet_pos_q16 = extrap;
  drv->jet_vel_q16 = (int32_t)ele_asr64((int64_t)drv->jet_vel_q16 * 3LL, 2U);
  drv->jet_acc_q16 = (int32_t)ele_asr64((int64_t)drv->jet_acc_q16, 1U);
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
    int64_t pred_x64 = (int64_t)drv->jet_pos_q16 + (int64_t)drv->jet_vel_q16 +
                       ele_asr64((int64_t)drv->jet_acc_q16, 1U);
    int64_t diff64 = (int64_t)x_cal - pred_x64;
    int64_t abs_diff64 = (diff64 >= 0LL) ? diff64 : -diff64;
    drv->codec_flags &= (uint8_t)~ELE_CODEC_HOTSWAP_PENDING;

    if (abs_diff64 > (int64_t)drv->max_slew_jump_q16) {
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
    int32_t new_vel =
        ele_sat_i64_i32((int64_t)x_cal - (int64_t)drv->jet_pos_q16);
    int32_t new_acc =
        ele_sat_i64_i32((int64_t)new_vel - (int64_t)drv->jet_vel_q16);
    drv->jet_pos_q16 = x_cal;
    drv->jet_vel_q16 =
        (int32_t)ele_asr64((int64_t)drv->jet_vel_q16 + (int64_t)new_vel, 1U);
    drv->jet_acc_q16 =
        (int32_t)ele_asr64((int64_t)drv->jet_acc_q16 + (int64_t)new_acc, 1U);
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
      frame[0] = src[0];
      if (stride_bytes > 1U) {
        frame[1] = src[1];
      }
      if (stride_bytes > 2U) {
        frame[2] = src[2];
      }
      if (stride_bytes > 3U) {
        frame[3] = src[3];
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
  /* UB-free inverse affine mapping via 64-by-32 unsigned division (0 __divdi3):
   * target_q16 = ((raw * scale_q16) >> bit_width) + offset_q16
   * => raw = ((target_q16 - offset_q16) << bit_width) / scale_q16
   */
  int64_t delta64 = (int64_t)target_q16 - (int64_t)drv->offset_q16;
  uint8_t bw =
      (drv->bit_width >= 1U && drv->bit_width <= 32U) ? drv->bit_width : 16U;
  uint8_t sh = (drv->bit_shift < 32U) ? drv->bit_shift : 0U;
  bool neg = (delta64 < 0LL) != (drv->scale_q16 < 0);
  uint64_t mag_delta =
      (delta64 < 0LL) ? (0ULL - (uint64_t)delta64) : (uint64_t)delta64;
  uint32_t mag_scale = ele_abs_i32_u32(drv->scale_q16);
  uint64_t max_shift_in =
      (bw >= 32U) ? 0xFFFFFFFFULL : ele_shr64(0xFFFFFFFFFFFFFFFFULL, bw);
  uint64_t raw_mag = 0ULL;
  if (mag_delta > max_shift_in) {
    raw_mag = 0xFFFFFFFFULL;
  } else {
    raw_mag = ele_div_u64_u32(ele_shl64(mag_delta, bw), mag_scale);
    if (raw_mag > 0xFFFFFFFFULL) {
      raw_mag = 0xFFFFFFFFULL;
    }
  }
  int64_t raw_i64 = neg ? -(int64_t)raw_mag : (int64_t)raw_mag;

  int64_t min_raw = 0LL;
  int64_t max_raw = 0LL;
  if ((drv->codec_flags & ELE_CODEC_SIGNED_2C) != 0U) {
    uint64_t half_range = (uint64_t)(1U << (bw - 1U));
    min_raw = -(int64_t)half_range;
    max_raw = (int64_t)(half_range - 1ULL);
  } else {
    min_raw = 0LL;
    max_raw = (bw >= 32U) ? 0xFFFFFFFFLL : (int64_t)((1U << bw) - 1U);
  }
  if (raw_i64 < min_raw) {
    raw_i64 = min_raw;
  } else if (raw_i64 > max_raw) {
    raw_i64 = max_raw;
  }

  uint32_t mask = (bw >= 32U) ? 0xFFFFFFFFU : ((1U << bw) - 1U);
  uint32_t word = ((uint32_t)(int32_t)raw_i64 & mask) << sh;

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
  uint64_t activity = (uint64_t)ele_abs_i32_u32(drv->jet_vel_q16) +
                      (uint64_t)ele_abs_i32_u32(drv->jet_acc_q16);
  uint64_t eps = (quiet_eps_q16 > 0) ? (uint64_t)(uint32_t)quiet_eps_q16 : 0ULL;

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
    uint64_t ina_ops[4];
    /* PC=0: Configure AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, Cont Shunt+Bus */
    ina_ops[0] = ele_bus_op_pack(ELE_OP_RMW_BITS8, 0x00U, 0x0FU, 0x07U, 0U, 0U);
    /* PC=1 (Loop): Poll MASK_ENABLE (0x06) bit 3 (CVRF == 1), 1.1ms LP */
    ina_ops[1] =
        ele_bus_op_pack(ELE_OP_POLL_MASK, 0x06U, 0x08U, 0x08U, 0U, 110U);
    /* PC=2: Burst Read SHUNT_VOLTAGE (0x01..0x02, 2 bytes) */
    ina_ops[2] = ele_bus_op_pack(ELE_OP_BURST_READ, 0x01U, 0U, 0U, 2U, 0U);
    ina_ops[3] = ele_bus_op_pack(ELE_OP_END, 0U, 0U, 0U, 0U, 0U);
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
  uint64_t num_uw_mohm = (prod_mv_v_q16 * 1000000ULL) >> 16U;
  uint64_t p_uw = ele_div_u64_u32(num_uw_mohm, r_shunt_mohm);
  return (p_uw > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)p_uw;
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
  uint64_t e_single_flip_u64 = ele_div_u64_u32(
      ((uint64_t)link->c_parasitic_ff * v_decivolt * v_decivolt) << 16U,
      200000U);
  if (e_single_flip_u64 < 65536ULL) {
    e_single_flip_u64 = 65536ULL;
  }

  int64_t e_uart_byte_pj_q16 = (int64_t)ele_div_u64_u32(
      e_single_flip_u64 * (uint64_t)uart_flips * 135ULL, 100U);
  uint64_t geom_factor_num =
      ((link->z0_ohms_q16 >> 16U) >= 250U) ? 150ULL : 100ULL;
  int64_t e_ppm_byte_pj_q16 =
      (int64_t)ele_div_u64_u32(e_single_flip_u64 * geom_factor_num, 100U);

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

  int64_t clamped_res = residual;
  if (clamped_res > 1073741823LL) {
    clamped_res = 1073741823LL;
  } else if (clamped_res < -1073741823LL) {
    clamped_res = -1073741823LL;
  }
  int32_t res32 = (int32_t)clamped_res;
  uint64_t dt64 = (uint64_t)tx_node->ticks_since_sync + 1ULL;
  int32_t dt32 = (dt64 > 32767ULL) ? 32767 : (int32_t)dt64;
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

  int64_t clamped_res = received_delta_q16;
  if (clamped_res > 1073741823LL) {
    clamped_res = 1073741823LL;
  } else if (clamped_res < -1073741823LL) {
    clamped_res = -1073741823LL;
  }
  int32_t res32 = (int32_t)clamped_res;
  uint64_t dt64 = (uint64_t)rx_node->ticks_since_sync + 1ULL;
  int32_t dt32 = (dt64 > 32767ULL) ? 32767 : (int32_t)dt64;
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
  uint64_t p_bjt_mw = ele_div_u64_u32(
      (uint64_t)current_ma * (uint64_t)drv->v_sat_drop_mv, 1000U);
  uint64_t p_copper_mw = ele_div_u64_u32(
      (uint64_t)current_ma * (uint64_t)current_ma * (uint64_t)drv->coil_r_mohm,
      1000000U);
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
    int64_t e_piezo_uj_q16 = (int64_t)ele_div_u64_u32(
        (15ULL * (uint64_t)piezo_mv * (uint64_t)piezo_mv) << 16U, 2000000000U);
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
  uint64_t baseline_power_mw = ele_div_u64_u32(
      ((uint64_t)baseline_pwm_ma * (uint64_t)drv->v_supply_mv) * 125ULL,
      100000U);
  int64_t baseline_energy_uj_q16 = (int64_t)ele_div_u64_u32(
      (baseline_power_mw * (uint64_t)dt_us) << 16U, 1000U);

  uint32_t hyst_mv = 80U;
  if (measured_bemf_mv > drv->target_bemf_mv + (3U * hyst_mv)) {
    drv->active_phase = (uint8_t)ELE_DRIVE_REGEN_HARVEST_BRAKE;
    drv->pulse_duty_permille = 0U;
    drv->coil_current_ma = 0U;
    uint32_t excess_mv = measured_bemf_mv - drv->target_bemf_mv;
    int64_t regen_uj_q16 = (int64_t)ele_div_u64_u32(
        ((uint64_t)excess_mv * (uint64_t)dt_us) << 16U, 50000U);
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

  uint64_t pulse_power_mw = ele_div_u64_u32(
      (uint64_t)drv->coil_current_ma * (uint64_t)drv->v_supply_mv, 1000U);
  int64_t pulse_energy_uj_q16 = (int64_t)ele_div_u64_u32(
      (pulse_power_mw * (uint64_t)dt_us) << 16U, 1000U);

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
  int64_t energy_q16 = total_energy * 256LL;
  int64_t holonomy_q16 = cross_holonomy * 256LL;

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
      ele_asr64(energy_q16 - csi->baseline_energy_q16, adapt_shift);
  csi->baseline_holonomy_q16 +=
      ele_asr64(holonomy_q16 - csi->baseline_holonomy_q16, adapt_shift);
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

  int64_t saved_per_step_uj =
      (int64_t)((csi->recommended_rate_hz < 100U)
                    ? (100U - csi->recommended_rate_hz)
                    : 0U) *
          9LL +
      (int64_t)((csi->tx_power_qdbm < 80U) ? (80U - csi->tx_power_qdbm) : 0U) *
          4LL;
  csi->energy_saved_uj_q16 += (saved_per_step_uj * 65536LL);

  return (EleCsiFieldRegime)csi->field_regime;
}

/* ============================================================================
 * 6. ele_silicon: HBM/UCIe Charge-Minimal Bus, Symplectic PDN dI/dt Governor,
 *    CMOS Gate-Toggle Entropy Calorimeter & 2-Jet Thermal-Leakage Barrier
 * ============================================================================
 */

static inline uint32_t ele_popcount64(uint64_t x) {
  return ele_popcount32((uint32_t)x) + ele_popcount32((uint32_t)(x >> 32U));
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
      uint8_t prev_p = (uint8_t)((bus->prev_inv_mask4 >> lane) & 1U);

      /* Baseline uncoded wire flips vs previous unencoded data beat */
      uint64_t prev_raw_w = (!pred_line && prev_p != 0U) ? ~prev_w : prev_w;
      beat_raw_flips += ele_popcount64(raw_w ^ prev_raw_w);

      /* 1. Optional 2-Jet / XOR temporal cache-line residual */
      uint64_t cand0 = pred_line ? (raw_w ^ pred_line[idx]) : raw_w;

      /* 2. Stateful 65-wire Bus-Invert (64 data + 1 polarity wire):
       *    flips(p=0) + flips(p=1) == 65 => min <= 32 strictly!
       */
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
  uint64_t scaled = ele_shl64(
      ele_div_u64_u32((uint64_t)base * (uint64_t)(12000U + rem_mc), 12000U),
      octaves);
  return (scaled > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)scaled;
}

uint32_t ele_silicon_step_thermal_jet2(ElePdnGrid64 *pdn,
                                       uint32_t active_power_w,
                                       int32_t ambient_mc,
                                       uint32_t horizon_ticks) {
  if (!pdn) {
    return 0U;
  }
  uint32_t leak_mw = ele_silicon_eval_leakage_mw(pdn->temp_mc, 15000U);
  uint64_t total_w = (uint64_t)active_power_w + (uint64_t)(leak_mw / 1000U);
  if (total_w > 100000ULL) {
    total_w = 100000ULL;
  }

  /* Silicon junction RC step: heating +8 mC/W, cooling (T - T_amb) / 12 */
  int64_t cooling_mc =
      ele_sdiv_i64_u32((int64_t)pdn->temp_mc - (int64_t)ambient_mc, 12U);
  int32_t next_temp_mc = ele_sat_i64_i32(
      (int64_t)pdn->temp_mc + (int64_t)(total_w * 8ULL) - cooling_mc);
  int32_t new_vel =
      ele_sat_i64_i32((int64_t)next_temp_mc - (int64_t)pdn->temp_mc);
  int32_t new_acc =
      ele_sat_i64_i32((int64_t)new_vel - (int64_t)pdn->dtemp_mc_tick);

  pdn->temp_mc = next_temp_mc;
  pdn->dtemp_mc_tick = new_vel;
  pdn->ddtemp_mc_tick2 =
      (int32_t)ele_asr64((int64_t)pdn->ddtemp_mc_tick2 + (int64_t)new_acc, 1U);

  int64_t h = (int64_t)((horizon_ticks > 0U)
                            ? ((horizon_ticks > 1024U) ? 1024U : horizon_ticks)
                            : 4U);
  int64_t pos_acc =
      (pdn->ddtemp_mc_tick2 > 0) ? (int64_t)pdn->ddtemp_mc_tick2 : 0LL;
  int64_t pred_temp_mc = (int64_t)pdn->temp_mc +
                         ((int64_t)pdn->dtemp_mc_tick * h) +
                         ((pos_acc * h * h) >> 1U);

  if (pred_temp_mc >= (int64_t)pdn->temp_knee_mc) {
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
    int64_t prod64 = (w > 0) ? (int64_t)x : ((w < 0) ? -(int64_t)x : 0LL);
    int32_t prod_q16 = ele_sat_i64_i32(prod64);
    acc_q16 = (int32_t)((uint32_t)acc_q16 + (uint32_t)prod_q16);

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

/* ============================================================================
 * 7. ele_rack_vrm: 48V Multiphase GaN/SiC Digital VRM (Phase-Shedding, ZVS &
 *    Active Pre-Charge) + 8-Socket Rack Busbar Symplectic Phase-Interleaver
 * ============================================================================
 */

void ele_vrm64_init(EleVrmPhase64 *vrm, uint32_t v_in_mv, uint32_t v_out_nom_mv,
                    uint8_t num_phases_max, uint32_t r_phase_uohm,
                    uint32_t p_sw_phase_mw, uint32_t c_out_uf,
                    uint32_t slew_phase_a_us) {
  if (!vrm) {
    return;
  }
  ele_zero64(vrm);
  uint8_t n_max =
      (num_phases_max >= 1U && num_phases_max <= 16U) ? num_phases_max : 16U;
  vrm->v_in_mv = (v_in_mv > 0U) ? v_in_mv : 48000U;
  vrm->v_out_nom_mv = (v_out_nom_mv > 0U) ? v_out_nom_mv : 750U;
  vrm->r_phase_uohm = (r_phase_uohm > 0U) ? r_phase_uohm : 800U;
  vrm->p_sw_phase_mw = p_sw_phase_mw;
  vrm->c_out_uf = (c_out_uf > 0U) ? c_out_uf : 12000U;
  vrm->slew_phase_a_us = (slew_phase_a_us > 0U) ? slew_phase_a_us : 15U;
  vrm->t_dead_ps = 8000U;
  vrm->num_phases_max = n_max;
  vrm->active_phases = n_max;
}

uint8_t ele_vrm_optimal_phases(const EleVrmPhase64 *vrm, uint32_t load_amp) {
  if (!vrm || vrm->num_phases_max <= 1U) {
    return 1U;
  }
  uint32_t p_sw_eff =
      (vrm->zvs_locked != 0U) ? (vrm->p_sw_phase_mw >> 2U) : vrm->p_sw_phase_mw;
  if (p_sw_eff == 0U) {
    return vrm->num_phases_max;
  }
  /* Exact discrete convex minimizer of P(I, N) = I^2*R/(1000*N) + N*P_sw_eff:
   * P(I, N+1) < P(I, N)  <=>  I^2 * r_uohm > N * (N + 1) * 1000 * p_sw_eff */
  uint64_t i2_r =
      (uint64_t)load_amp * (uint64_t)load_amp * (uint64_t)vrm->r_phase_uohm;
  uint64_t p_sw_scaled = 1000ULL * (uint64_t)p_sw_eff;
  uint8_t best_n = 1U;
  for (uint8_t n = 1U; n < vrm->num_phases_max; ++n) {
    uint64_t rhs = (uint64_t)n * (uint64_t)(n + 1U) * p_sw_scaled;
    if (i2_r > rhs) {
      best_n = (uint8_t)(n + 1U);
    } else {
      break;
    }
  }
  return best_n;
}

uint32_t ele_vrm_eval_loss_mw(const EleVrmPhase64 *vrm, uint32_t load_amp,
                              uint8_t n_phases, bool zvs_soft_switched) {
  if (!vrm) {
    return 0U;
  }
  uint32_t n = (n_phases >= 1U) ? (uint32_t)n_phases : 1U;
  uint64_t cond_mw = ele_div_u64_u32((uint64_t)load_amp * (uint64_t)load_amp *
                                         (uint64_t)vrm->r_phase_uohm,
                                     1000U * n);
  /* Under ZVS resonant soft-switching, 1/2 * C_oss * V^2 turn-on energy (~75%
   * of 48V GaN stage switching loss) is recovered prior to gate turn-on,
   * leaving only gate-drive Q_g * V_drv (~25%). */
  uint32_t p_sw_eff =
      zvs_soft_switched ? (vrm->p_sw_phase_mw >> 2U) : vrm->p_sw_phase_mw;
  uint64_t sw_mw = (uint64_t)n * (uint64_t)p_sw_eff;
  return (uint32_t)(cond_mw + sw_mw);
}

static inline uint16_t ele_vrm_eval_reactive_droop_mv(const EleVrmPhase64 *vrm,
                                                      uint32_t step_amp) {
  uint64_t max_slew =
      (uint64_t)vrm->num_phases_max * (uint64_t)vrm->slew_phase_a_us;
  if (max_slew == 0ULL || (uint64_t)step_amp <= max_slew ||
      vrm->c_out_uf == 0U) {
    return 0U;
  }
  uint64_t k_ticks = ele_div_u64_u32((uint64_t)step_amp, (uint32_t)max_slew);
  uint64_t tri = (k_ticks * (k_ticks + 1ULL)) >> 1U;
  uint64_t q_def_uc = (k_ticks * (uint64_t)step_amp) - (max_slew * tri);
  uint64_t droop_mv = ele_div_u64_u32(q_def_uc * 1000ULL, vrm->c_out_uf);
  return (droop_mv > 0xFFFFULL) ? 0xFFFFU : (uint16_t)droop_mv;
}

uint8_t ele_vrm_step_us(EleVrmPhase64 *vrm, uint32_t load_amp,
                        uint32_t upcoming_load_amp,
                        int32_t measured_zvs_err_ps) {
  if (!vrm) {
    return 0U;
  }
  uint32_t c_uf = (vrm->c_out_uf > 0U) ? vrm->c_out_uf : 1U;
  uint8_t n_max = (vrm->num_phases_max >= 1U && vrm->num_phases_max <= 16U)
                      ? vrm->num_phases_max
                      : 1U;

  /* 1. Record uncompensated baseline droop if a sharp current step arrives */
  uint32_t ref_step = 0U;
  if (upcoming_load_amp > load_amp) {
    ref_step = upcoming_load_amp - load_amp;
  }
  if (load_amp > vrm->inductor_current_a && vrm->precharge_armed == 0U) {
    uint32_t direct_step = load_amp - vrm->inductor_current_a;
    if (direct_step > ref_step) {
      ref_step = direct_step;
    }
  }
  uint16_t base_droop = ele_vrm_eval_reactive_droop_mv(vrm, ref_step);
  if (base_droop > vrm->max_droop_base_mv) {
    vrm->max_droop_base_mv = base_droop;
  }

  /* 2. 2-Jet picosecond Zero-Voltage Switching (ZVS) dead-time resonant loop
   *    (exact arithmetic right-shift floor match with rtl/ele_vrm_zvs_ctrl.sv)
   */
  int64_t err64 = (int64_t)measured_zvs_err_ps;
  int64_t d_err64 = err64 - (int64_t)vrm->zvs_phase_vel_ps;
  vrm->zvs_phase_vel_ps = measured_zvs_err_ps;
  int64_t next_dead =
      (int64_t)vrm->t_dead_ps - ele_asr64(err64, 1U) - ele_asr64(d_err64, 2U);
  if (next_dead < 1500LL) {
    next_dead = 1500LL;
  } else if (next_dead > 25000LL) {
    next_dead = 25000LL;
  }
  vrm->t_dead_ps = (uint32_t)next_dead;

  uint32_t abs_err = ele_abs_i32_u32(measured_zvs_err_ps);
  vrm->zvs_locked = (abs_err <= 250U) ? 1U : 0U;

  /* 3. Command-lookahead Active Pre-Charge + ZVS-Aware Phase-Shedding */
  bool precharge = ((uint64_t)upcoming_load_amp > ((uint64_t)load_amp + 64ULL));
  vrm->precharge_armed = precharge ? 1U : 0U;

  uint8_t n_act = n_max;
  uint32_t i_gap = (load_amp >= vrm->inductor_current_a)
                       ? (load_amp - vrm->inductor_current_a)
                       : (vrm->inductor_current_a - load_amp);
  if (!precharge && i_gap <= vrm->slew_phase_a_us &&
      vrm->v_cap_delta_mv >= -5 && vrm->v_cap_delta_mv <= 5) {
    n_act = ele_vrm_optimal_phases(vrm, load_amp);
  }
  vrm->active_phases = n_act;

  /* 4. Inductor current slew & output capacitor ODE (1 us integration step) */
  int64_t max_slew = (int64_t)n_act * (int64_t)vrm->slew_phase_a_us;
  int64_t target_i = (int64_t)load_amp;
  if (precharge) {
    /* Deadbeat pre-charge up to +45 mV (+6% transient margin on 750 mV rail) */
    int64_t headroom_mv = 45LL - (int64_t)vrm->v_cap_delta_mv;
    if (headroom_mv > 0LL) {
      int64_t q_head_uc = ele_sdiv_i64_u32(headroom_mv * (int64_t)c_uf, 1000U);
      int64_t pre_i = (int64_t)load_amp + q_head_uc;
      target_i = (pre_i < (int64_t)upcoming_load_amp)
                     ? pre_i
                     : (int64_t)upcoming_load_amp;
    }
  } else if (vrm->v_cap_delta_mv != 0) {
    /* Critically-damped deadbeat charge restoration bounded by max_slew/2 */
    int64_t q_err_uc =
        ele_sdiv_i64_u32((int64_t)vrm->v_cap_delta_mv * (int64_t)c_uf, 1000U);
    int64_t half_slew = max_slew >> 1U;
    if (q_err_uc > half_slew) {
      q_err_uc = half_slew;
    } else if (q_err_uc < -half_slew) {
      q_err_uc = -half_slew;
    }
    target_i = (int64_t)load_amp - q_err_uc;
    if (target_i < 0LL) {
      target_i = 0LL;
    }
  }

  int64_t cur_i = (int64_t)vrm->inductor_current_a;
  int64_t diff_i = target_i - cur_i;
  if (diff_i > max_slew) {
    cur_i += max_slew;
  } else if (diff_i < -max_slew) {
    cur_i -= max_slew;
  } else {
    cur_i = target_i;
  }
  if (cur_i < 0LL) {
    cur_i = 0LL;
  } else if (cur_i > 0xFFFFFFFFLL) {
    cur_i = 0xFFFFFFFFLL;
  }
  vrm->inductor_current_a = (uint32_t)cur_i;

  int64_t dq_uc = cur_i - (int64_t)load_amp;
  int64_t dv_mv = ele_sdiv_i64_u32(dq_uc * 1000LL, c_uf);
  int64_t next_v_delta = (int64_t)vrm->v_cap_delta_mv + dv_mv;
  int64_t min_v_delta =
      -(int64_t)((vrm->v_out_nom_mv > 0U && vrm->v_out_nom_mv <= 0x7FFFFFFFU)
                     ? vrm->v_out_nom_mv
                     : 750U);
  if (next_v_delta < min_v_delta) {
    next_v_delta = min_v_delta;
  } else if (next_v_delta > 5000LL) {
    next_v_delta = 5000LL;
  }
  vrm->v_cap_delta_mv = (int32_t)next_v_delta;

  if (vrm->v_cap_delta_mv < 0) {
    uint32_t droop32 = ele_abs_i32_u32(vrm->v_cap_delta_mv);
    uint16_t droop = (droop32 > 0xFFFFU) ? 0xFFFFU : (uint16_t)droop32;
    if (droop > vrm->max_droop_active_mv) {
      vrm->max_droop_active_mv = droop;
    }
  }

  /* 5. Exact nanojoule calorimetry (1 mW * 1 us == 1 nJ) */
  uint32_t base_mw = ele_vrm_eval_loss_mw(vrm, load_amp, n_max, false);
  uint32_t symp_mw =
      ele_vrm_eval_loss_mw(vrm, load_amp, n_act, vrm->zvs_locked != 0U);
  vrm->loss_baseline_nj += (uint64_t)base_mw;
  vrm->loss_symplectic_nj += (uint64_t)symp_mw;

  return n_act;
}

void ele_rack_busbar64_init(EleRackBusbar64 *rack, uint32_t breaker_cap_a,
                            uint32_t r_busbar_uohm) {
  if (!rack) {
    return;
  }
  ele_zero64(rack);
  rack->breaker_cap_a = (breaker_cap_a > 0U) ? breaker_cap_a : 1400U;
  rack->r_busbar_uohm = (r_busbar_uohm > 0U) ? r_busbar_uohm : 500U;
}

uint32_t ele_rack_step_interleave8(EleRackBusbar64 *rack,
                                   const uint16_t req_slot_amp[8],
                                   uint16_t per_slot_slew_a) {
  if (!rack || !req_slot_amp) {
    return 0U;
  }

  /* 1. Uncoordinated synchronous rack draw & harmonic I^2*R busbar waste */
  uint32_t sync_sum = 0U;
  uint32_t tank[8];
  uint32_t alloc[8];
  uint32_t symp_sum = 0U;

  for (uint8_t k = 0U; k < 8U; ++k) {
    sync_sum += (uint32_t)req_slot_amp[k];
    tank[k] = (uint32_t)rack->slot_backlog_work[k] + (uint32_t)req_slot_amp[k];
    uint32_t des = tank[k];
    if (per_slot_slew_a > 0U) {
      uint32_t max_up =
          (uint32_t)rack->slot_current_a[k] + (uint32_t)per_slot_slew_a;
      uint32_t lo_lim =
          (rack->slot_current_a[k] > per_slot_slew_a)
              ? (uint32_t)(rack->slot_current_a[k] - per_slot_slew_a)
              : 0U;
      if (des > max_up) {
        des = max_up;
      } else if (des < lo_lim && tank[k] >= lo_lim) {
        des = lo_lim;
      }
    }
    if (des > 0xFFFFU) {
      des = 0xFFFFU;
    }
    alloc[k] = des;
    symp_sum += des;
  }

  if (sync_sum > rack->peak_sync_rack_a) {
    rack->peak_sync_rack_a = sync_sum;
  }

  /* 2. Symplectic water-filling ceiling under rack/substation breaker_cap_a:
   *    Shaves peak crests uniformly with rotating socket fairness. */
  if (rack->breaker_cap_a > 0U && symp_sum > rack->breaker_cap_a) {
    uint32_t excess = symp_sum - rack->breaker_cap_a;
    uint8_t rot_base = (uint8_t)(rack->bus_loss_symp_uj & 7ULL);
    for (uint8_t pass = 0U; pass < 8U && excess > 0U; ++pass) {
      uint32_t active_slots = 0U;
      for (uint8_t k = 0U; k < 8U; ++k) {
        if (alloc[k] > 0U) {
          active_slots += 1U;
        }
      }
      if (active_slots == 0U) {
        break;
      }
      uint32_t cut_each = (excess + active_slots - 1U) / active_slots;
      for (uint8_t ki = 0U; ki < 8U && excess > 0U; ++ki) {
        uint8_t k = (uint8_t)((ki + rot_base + pass) & 7U);
        uint32_t cut = (alloc[k] < cut_each) ? alloc[k] : cut_each;
        if (cut > excess) {
          cut = excess;
        }
        alloc[k] -= cut;
        excess -= cut;
      }
    }
    symp_sum = 0U;
    for (uint8_t k = 0U; k < 8U; ++k) {
      symp_sum += alloc[k];
    }
  }

  for (uint8_t k = 0U; k < 8U; ++k) {
    rack->slot_current_a[k] =
        (alloc[k] > 0xFFFFU) ? 0xFFFFU : (uint16_t)alloc[k];
    uint32_t rem = tank[k] - alloc[k];
    rack->slot_backlog_work[k] = (rem > 0xFFFFU) ? 0xFFFFU : (uint16_t)rem;
  }

  if (symp_sum > rack->peak_interleaved_a) {
    rack->peak_interleaved_a = symp_sum;
  }

  /* 3. Busbar & transformer winding I^2*R loss (uJ per 1ms wave tick) */
  rack->bus_loss_sync_uj += ele_div_u64_u32(
      (uint64_t)sync_sum * (uint64_t)sync_sum * (uint64_t)rack->r_busbar_uohm,
      1000U);
  rack->bus_loss_symp_uj += ele_div_u64_u32(
      (uint64_t)symp_sum * (uint64_t)symp_sum * (uint64_t)rack->r_busbar_uohm,
      1000U);

  return symp_sum;
}

uint32_t ele_rack_unlocked_gpu_slots(const EleRackBusbar64 *rack,
                                     uint32_t base_gpu_count,
                                     uint32_t *out_sync_max_gpus) {
  if (!rack || base_gpu_count == 0U || rack->peak_sync_rack_a == 0U ||
      rack->peak_interleaved_a == 0U) {
    if (out_sync_max_gpus) {
      *out_sync_max_gpus = 0U;
    }
    return 0U;
  }
  /* Exact integer capacity without intermediate per-GPU ceiling rounding */
  uint32_t sync_gpus = (uint32_t)ele_div_u64_u32((uint64_t)rack->breaker_cap_a *
                                                     (uint64_t)base_gpu_count,
                                                 rack->peak_sync_rack_a);
  uint32_t symp_gpus = (uint32_t)ele_div_u64_u32((uint64_t)rack->breaker_cap_a *
                                                     (uint64_t)base_gpu_count,
                                                 rack->peak_interleaved_a);
  if (out_sync_max_gpus) {
    *out_sync_max_gpus = sync_gpus;
  }
  return symp_gpus;
}

/* ============================================================================
 * 8. ele_grid: Diurnal Duck-Curve 800V HVDC & 60Hz/50Hz GFM Synthetic Inertia
 * ============================================================================
 */

uint32_t ele_grid_period_ns_to_mhz(uint32_t period_ns) {
  if (period_ns == 0U) {
    return 0U;
  }
  uint64_t num = 1000000000000ULL + (uint64_t)(period_ns >> 1U);
  uint64_t mhz = ele_div_u64_u32(num, period_ns);
  return (mhz > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)mhz;
}

void ele_grid_inverter64_init(EleGridInverter64 *inv, uint32_t nom_freq_mhz,
                              uint32_t max_power_kw, uint32_t m_inertia_kw_s_hz,
                              uint32_t k_droop_kw_hz, uint16_t deadband_mhz) {
  if (!inv) {
    return;
  }
  ele_zero64(inv);
  uint32_t f0 = (nom_freq_mhz >= 40000U && nom_freq_mhz <= 100000U)
                    ? nom_freq_mhz
                    : 60000U;
  inv->nom_freq_mhz = f0;
  inv->cur_freq_mhz = f0;
  inv->max_power_kw = (max_power_kw > 0U) ? max_power_kw : 25000U;
  inv->m_inertia_kw_s_hz = m_inertia_kw_s_hz;
  inv->k_droop_kw_hz = k_droop_kw_hz;
  inv->deadband_mhz = deadband_mhz;
  inv->compute_pcap_permille = 1000U;
  inv->nadir_gfl_mhz = f0;
  inv->nadir_gfm_mhz = f0;
}

static inline int32_t ele_scaled_signed_kw(int32_t signed_val,
                                           uint32_t gain_kw) {
  if (signed_val == 0 || gain_kw == 0U) {
    return 0;
  }
  bool neg = (signed_val < 0);
  uint32_t mag = ele_abs_i32_u32(signed_val);
  uint64_t kw = ele_div_u64_u32((uint64_t)mag * (uint64_t)gain_kw, 1000U);
  int32_t clamped = (kw > 0x7FFFFFFFULL) ? 0x7FFFFFFF : (int32_t)kw;
  return neg ? -clamped : clamped;
}

int32_t ele_grid_step_gfm_cycle(EleGridInverter64 *inv, uint32_t period_ns,
                                int32_t grid_disturbance_kw,
                                uint32_t base_grid_stiffness_kw_hz) {
  if (!inv) {
    return 0;
  }
  uint32_t f_nom = (inv->nom_freq_mhz >= 1000U) ? inv->nom_freq_mhz : 60000U;
  uint32_t prev_mhz =
      (inv->cur_freq_mhz >= 10000U && inv->cur_freq_mhz <= 100000U)
          ? inv->cur_freq_mhz
          : f_nom;
  uint32_t meas_mhz =
      (period_ns > 0U) ? ele_grid_period_ns_to_mhz(period_ns) : f_nom;
  /* Hardware zero-crossing de-glitch window: [10.000 Hz, 100.000 Hz] */
  if (meas_mhz < 10000U) {
    meas_mhz = 10000U;
  } else if (meas_mhz > 100000U) {
    meas_mhz = 100000U;
  }
  inv->cur_freq_mhz = meas_mhz;

  int32_t df_mhz = (int32_t)meas_mhz - (int32_t)f_nom;
  int32_t d_step_mhz = (int32_t)meas_mhz - (int32_t)prev_mhz;
  int64_t cycles_per_s = (int64_t)(f_nom / 1000U);
  int32_t rocof = ele_sat_i64_i32((int64_t)d_step_mhz * cycles_per_s);
  inv->rocof_mhz_s = rocof;

  /* 1. Deadband-filtered primary droop + 2-Jet synthetic rotational inertia */
  int32_t eff_df = 0;
  int32_t db = (int32_t)inv->deadband_mhz;
  if (df_mhz > db) {
    eff_df = df_mhz - db;
  } else if (df_mhz < -db) {
    eff_df = df_mhz + db;
  }

  int32_t p_droop = -ele_scaled_signed_kw(eff_df, inv->k_droop_kw_hz);
  int32_t p_inert = -ele_scaled_signed_kw(rocof, inv->m_inertia_kw_s_hz);
  int64_t p_cmd64 =
      (int64_t)inv->p_setpoint_kw + (int64_t)p_droop + (int64_t)p_inert;
  int64_t p_lim64 = (inv->max_power_kw > 0x7FFFFFFFU)
                        ? 0x7FFFFFFFLL
                        : (int64_t)inv->max_power_kw;
  if (p_cmd64 > p_lim64) {
    p_cmd64 = p_lim64;
  } else if (p_cmd64 < -p_lim64) {
    p_cmd64 = -p_lim64;
  }
  int32_t p_cmd = (int32_t)p_cmd64;
  inv->p_inject_kw = p_cmd;

  /* 2. Symplectic virtual synchronous rotor angle update wrapped in [-pi, +pi]
   *    (O(1) 32-bit RV32M hardware division via ele_sdiv_i64_u32, 0 __udivdi3)
   */
  int64_t d_theta = ele_sdiv_i64_u32((int64_t)df_mhz * 6283LL, f_nom);
  int64_t raw_theta = (int64_t)inv->rotor_phase_mrad + d_theta;
  int64_t turns = ele_sdiv_i64_u32(
      raw_theta + ((raw_theta >= 0LL) ? 3141LL : -3141LL), 6283U);
  inv->rotor_phase_mrad = (int32_t)(raw_theta - (turns * 6283LL));

  /* 3. 1-Cycle Zero-Crossing Hardware Compute P-Cap Droop / Solar Sponge
   *    (RoCoF arrest only triggers below nominal deadband, never when
   *     recovering downward from over-frequency back toward 60.000 Hz!) */
  if (df_mhz < -150 || (df_mhz < -db && rocof < -400)) {
    uint32_t f_pen = (df_mhz < -150) ? (uint32_t)(-df_mhz - 150) : 0U;
    uint32_t r_pen =
        (rocof < -400) ? ((ele_abs_i32_u32(rocof) - 400U) >> 2U) : 0U;
    uint64_t cut = 150ULL + (uint64_t)f_pen + (uint64_t)r_pen;
    inv->compute_pcap_permille =
        (cut >= 450ULL) ? 550U : (uint16_t)(1000ULL - cut);
  } else if (df_mhz > 100) {
    inv->compute_pcap_permille = 1150U;
  } else {
    inv->compute_pcap_permille = 1000U;
  }

  /* 4. Weak-grid frequency nadir audit: GFL (H=0) vs GFM synthetic inertia */
  if (grid_disturbance_kw < 0 && base_grid_stiffness_kw_hz > 0U) {
    uint32_t loss_kw = ele_abs_i32_u32(grid_disturbance_kw);
    uint32_t gfl_drop_mhz = (uint32_t)ele_div_u64_u32(
        (uint64_t)loss_kw * 1000ULL, base_grid_stiffness_kw_hz);
    uint32_t f_gfl = (f_nom > gfl_drop_mhz) ? (f_nom - gfl_drop_mhz) : 0U;
    if (f_gfl < inv->nadir_gfl_mhz) {
      inv->nadir_gfl_mhz = f_gfl;
    }

    uint32_t sup_kw = (p_cmd > 0) ? (uint32_t)p_cmd : 0U;
    uint32_t rem_kw = (loss_kw > sup_kw) ? (loss_kw - sup_kw) : 0U;
    uint32_t gfm_drop_mhz = (uint32_t)ele_div_u64_u32(
        (uint64_t)rem_kw * 1000ULL, base_grid_stiffness_kw_hz);
    uint32_t f_gfm = (f_nom > gfm_drop_mhz) ? (f_nom - gfm_drop_mhz) : 0U;
    if (f_gfm < inv->nadir_gfm_mhz) {
      inv->nadir_gfm_mhz = f_gfm;
    }
  }
  if (meas_mhz < inv->nadir_gfm_mhz) {
    inv->nadir_gfm_mhz = meas_mhz;
  }

  if (p_cmd > 0) {
    inv->inertia_energy_kj +=
        ele_div_u64_u32((uint64_t)(uint32_t)p_cmd * 1000ULL, f_nom);
  }
  inv->cycles_stabilized += 1ULL;
  return p_cmd;
}

void ele_grid_duck64_init(EleDuckMicrogrid64 *mg, uint32_t bess_cap_kwh,
                          uint32_t bess_init_kwh, uint32_t thermal_cap_kwh,
                          uint32_t thermal_init_kwh,
                          uint32_t grid_feeder_cap_kw,
                          uint32_t bess_max_rate_kw) {
  if (!mg) {
    return;
  }
  ele_zero64(mg);
  mg->bess_cap_kwh = bess_cap_kwh;
  mg->bess_soc_kwh =
      (bess_init_kwh <= bess_cap_kwh) ? bess_init_kwh : bess_cap_kwh;
  mg->thermal_cap_kwh = thermal_cap_kwh;
  mg->thermal_soc_kwh = (thermal_init_kwh <= thermal_cap_kwh) ? thermal_init_kwh
                                                              : thermal_cap_kwh;
  mg->grid_feeder_cap_kw = grid_feeder_cap_kw;
  mg->bess_max_rate_kw = bess_max_rate_kw;
}

uint32_t ele_grid_step_duck_hour(EleDuckMicrogrid64 *mg, uint32_t pv_avail_kw,
                                 uint32_t rigid_compute_kw,
                                 uint32_t elastic_batch_kw,
                                 uint32_t cooling_elec_kw,
                                 int32_t price_milli_per_kwh,
                                 bool is_evening_peak) {
  if (!mg) {
    return 0U;
  }

  /* 1. Legacy AC-coupled rigid baseline (PV->AC->Grid->UPS->PFC: +12% IT loss,
   *    4% PV inverter loss, zero thermal shift, zero compute phase-shift) */
  uint32_t base_it_kw = rigid_compute_kw + elastic_batch_kw;
  uint32_t base_conv_loss_kw = (base_it_kw * 120U) / 1000U;
  uint32_t pv_inv_loss_kw = (pv_avail_kw * 40U) / 1000U;
  uint32_t usable_ac_pv_kw = pv_avail_kw - pv_inv_loss_kw;
  uint32_t base_total_kw = base_it_kw + cooling_elec_kw + base_conv_loss_kw;

  if (usable_ac_pv_kw >= base_total_kw) {
    uint32_t excess_pv = usable_ac_pv_kw - base_total_kw;
    if (price_milli_per_kwh <= 0) {
      mg->curtailed_pv_base_kwh += excess_pv;
    } else if (mg->grid_feeder_cap_kw > 0U &&
               excess_pv > mg->grid_feeder_cap_kw) {
      mg->curtailed_pv_base_kwh += (excess_pv - mg->grid_feeder_cap_kw);
    }
  } else {
    uint32_t base_import_kw = base_total_kw - usable_ac_pv_kw;
    if (is_evening_peak && base_import_kw > mg->peak_evening_base_kw) {
      mg->peak_evening_base_kw = base_import_kw;
    }
    mg->cost_baseline_milli +=
        (int64_t)base_import_kw * (int64_t)price_milli_per_kwh;
  }

  /* 2. Symplectic 800V HVDC Direct Busbar + Thermal PCM + Compute Sponge */
  uint32_t batch_tank = mg->batch_backlog_kwh + elastic_batch_kw;
  uint32_t run_batch_kw = 0U;
  uint32_t fallback_cap =
      (rigid_compute_kw > 0U)
          ? rigid_compute_kw
          : ((mg->grid_feeder_cap_kw > 0U) ? mg->grid_feeder_cap_kw
                                           : batch_tank);
  bool solar_rich = (pv_avail_kw > (rigid_compute_kw + cooling_elec_kw)) ||
                    (price_milli_per_kwh <= 0);

  if (is_evening_peak) {
    /* Defer elastic batch out of 16:00-22:00 evening peak */
    run_batch_kw = 0U;
  } else if (solar_rich) {
    /* Midday solar canyon / negative LMP: open 3x compute sponge */
    uint32_t cap_batch =
        (elastic_batch_kw > 0U) ? (elastic_batch_kw * 3U) : fallback_cap;
    run_batch_kw = (batch_tank < cap_batch) ? batch_tank : cap_batch;
  } else {
    /* Off-peak night hours: drain conserved backlog steadily */
    uint32_t cap_batch =
        (elastic_batch_kw > 0U) ? (elastic_batch_kw * 2U) : fallback_cap;
    run_batch_kw = (batch_tank < cap_batch) ? batch_tank : cap_batch;
  }
  mg->batch_backlog_kwh = batch_tank - run_batch_kw;

  /* Thermal cooling phase-shift: discharge PCM/ice tank during evening peak */
  uint32_t active_cool_kw = cooling_elec_kw;
  if (is_evening_peak && mg->thermal_soc_kwh > 0U) {
    uint32_t dis_th = (mg->thermal_soc_kwh < active_cool_kw)
                          ? mg->thermal_soc_kwh
                          : active_cool_kw;
    active_cool_kw -= dis_th;
    mg->thermal_soc_kwh -= dis_th;
  }

  /* Direct 800V HVDC ZVS DC-DC overhead (~2.5% vs legacy ~12% + 4% PV inv) */
  uint32_t symp_it_kw = rigid_compute_kw + run_batch_kw;
  uint32_t hvdc_loss_kw = (symp_it_kw * 25U) / 1000U;
  uint32_t legacy_equiv_loss = base_conv_loss_kw + pv_inv_loss_kw;
  if (legacy_equiv_loss > hvdc_loss_kw) {
    mg->ac_conv_waste_kwh += (legacy_equiv_loss - hvdc_loss_kw);
  }

  uint32_t dc_demand_kw = symp_it_kw + hvdc_loss_kw + active_cool_kw;
  uint32_t symp_import_kw = 0U;

  if (pv_avail_kw >= dc_demand_kw) {
    uint32_t surp = pv_avail_kw - dc_demand_kw;
    /* Priority A: Charge Thermal Storage (1/5th cost of Li-ion, 0 degradation)
     */
    uint32_t th_room = mg->thermal_cap_kwh - mg->thermal_soc_kwh;
    uint32_t chg_th = (surp < th_room) ? surp : th_room;
    mg->thermal_soc_kwh += chg_th;
    surp -= chg_th;

    /* Priority B: Charge 800V LFP BESS directly on DC busbar */
    uint32_t b_room = mg->bess_cap_kwh - mg->bess_soc_kwh;
    if (b_room > mg->bess_max_rate_kw) {
      b_room = mg->bess_max_rate_kw;
    }
    uint32_t chg_b = (surp < b_room) ? surp : b_room;
    mg->bess_soc_kwh += chg_b;
    surp -= chg_b;

    /* Negative wholesale LMP bonus: soak negative-priced grid power into lower
     * 25% of BESS while reserving upper 75% for behind-the-meter PV peak */
    uint32_t low_bess = mg->bess_cap_kwh >> 2U;
    if (price_milli_per_kwh < 0 && surp == 0U && mg->bess_soc_kwh < low_bess) {
      uint32_t rem_b = low_bess - mg->bess_soc_kwh;
      uint32_t rate_left =
          (mg->bess_max_rate_kw > chg_b) ? (mg->bess_max_rate_kw - chg_b) : 0U;
      uint32_t grid_sponge = (rem_b < rate_left) ? rem_b : rate_left;
      if (mg->grid_feeder_cap_kw > 0U && grid_sponge > mg->grid_feeder_cap_kw) {
        grid_sponge = mg->grid_feeder_cap_kw;
      }
      mg->bess_soc_kwh += grid_sponge;
      symp_import_kw = grid_sponge;
    } else if (surp > 0U) {
      if (price_milli_per_kwh <= 0) {
        mg->curtailed_pv_symp_kwh += surp;
      } else if (mg->grid_feeder_cap_kw > 0U && surp > mg->grid_feeder_cap_kw) {
        mg->curtailed_pv_symp_kwh += (surp - mg->grid_feeder_cap_kw);
      }
    }
  } else {
    uint32_t def_kw = dc_demand_kw - pv_avail_kw;
    if (is_evening_peak ||
        (mg->grid_feeder_cap_kw > 0U && def_kw > mg->grid_feeder_cap_kw)) {
      uint32_t avail_b = (mg->bess_soc_kwh < mg->bess_max_rate_kw)
                             ? mg->bess_soc_kwh
                             : mg->bess_max_rate_kw;
      uint32_t dis_b = (def_kw < avail_b) ? def_kw : avail_b;
      def_kw -= dis_b;
      mg->bess_soc_kwh -= dis_b;
    } else if (pv_avail_kw == 0U && price_milli_per_kwh > 0 &&
               mg->bess_soc_kwh > (mg->bess_cap_kwh / 5U)) {
      /* Post-peak dark hours: drain surplus solar SoC down to 20% reserve so
       * full tank headroom is ready before sunrise */
      uint32_t surp_soc = mg->bess_soc_kwh - (mg->bess_cap_kwh / 5U);
      uint32_t avail_b =
          (surp_soc < mg->bess_max_rate_kw) ? surp_soc : mg->bess_max_rate_kw;
      uint32_t dis_b = (def_kw < avail_b) ? def_kw : avail_b;
      def_kw -= dis_b;
      mg->bess_soc_kwh -= dis_b;
    } else if (!is_evening_peak && price_milli_per_kwh < 0 &&
               mg->bess_soc_kwh < (mg->bess_cap_kwh >> 2U)) {
      /* Negative night wind LMP: charge lower 25% of BESS while getting paid,
       * preserving upper 75% solar sponge headroom before sunrise */
      uint32_t low_bess = mg->bess_cap_kwh >> 2U;
      uint32_t rem_b = low_bess - mg->bess_soc_kwh;
      if (rem_b > mg->bess_max_rate_kw) {
        rem_b = mg->bess_max_rate_kw;
      }
      if (mg->grid_feeder_cap_kw > 0U &&
          def_kw + rem_b > mg->grid_feeder_cap_kw) {
        rem_b = (mg->grid_feeder_cap_kw > def_kw)
                    ? (mg->grid_feeder_cap_kw - def_kw)
                    : 0U;
      }
      mg->bess_soc_kwh += rem_b;
      def_kw += rem_b;
    }
    symp_import_kw = def_kw;
  }

  if (is_evening_peak && symp_import_kw > mg->peak_evening_symp_kw) {
    mg->peak_evening_symp_kw = symp_import_kw;
  }
  mg->cost_symplectic_milli +=
      (int64_t)symp_import_kw * (int64_t)price_milli_per_kwh;
  return symp_import_kw;
}

/* ============================================================================
 * 10. Package Coupled-Inductor IVR & <1ns Droop-Adaptive AD-FLL (`ele_ivr_fll`)
 * ============================================================================
 */

void ele_ivr_fll64_init(EleIvrFll64 *fll, uint32_t v_nom_mv,
                        uint32_t v_guard_static_mv, uint32_t v_th_mv,
                        uint32_t t_nom_ps, uint32_t l_pkg_uv_per_a,
                        uint32_t ivr_slew_mv_cyc) {
  if (!fll) {
    return;
  }
  ele_zero64(fll);
  fll->v_nom_mv = (v_nom_mv >= 300U) ? v_nom_mv : 750U;
  fll->v_guard_static_mv = (v_guard_static_mv > 0U) ? v_guard_static_mv : 80U;
  fll->v_th_mv = (v_th_mv < fll->v_nom_mv) ? v_th_mv : 238U;
  fll->t_nom_ps = (t_nom_ps >= 50U) ? t_nom_ps : 500U;
  fll->v_inst_mv = fll->v_nom_mv;
  fll->t_clk_ps = fll->t_nom_ps;
  fll->l_pkg_uv_per_a = (l_pkg_uv_per_a > 0U) ? l_pkg_uv_per_a : 250U;
  fll->ivr_slew_mv_cyc = (ivr_slew_mv_cyc > 0U) ? ivr_slew_mv_cyc : 20U;
}

uint32_t ele_fll_eval_period_ps(uint32_t t_nom_ps, uint32_t v_nom_mv,
                                uint32_t v_inst_mv, bool *out_emergency_gate) {
  if (v_inst_mv >= v_nom_mv) {
    if (out_emergency_gate) {
      *out_emergency_gate = false;
    }
    return t_nom_ps;
  }
  uint32_t droop_mv = v_nom_mv - v_inst_mv;
  if (droop_mv > 256U) {
    if (out_emergency_gate) {
      *out_emergency_gate = true;
    }
    uint64_t p4 = (uint64_t)t_nom_ps * 4ULL;
    return (p4 > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)p4;
  }
  if (out_emergency_gate) {
    *out_emergency_gate = false;
  }
  uint64_t lin_num = (uint64_t)t_nom_ps * (uint64_t)droop_mv + 511ULL;
  uint32_t dt1_ps = (uint32_t)ele_shr64(lin_num, 9U);
  uint32_t droop_sq = droop_mv * droop_mv;
  uint64_t quad_num = (uint64_t)t_nom_ps * (uint64_t)droop_sq + 131071ULL;
  uint32_t dt2_ps = (uint32_t)ele_shr64(quad_num, 17U);
  uint64_t total_ps = (uint64_t)t_nom_ps + (uint64_t)dt1_ps + (uint64_t)dt2_ps;
  return (total_ps > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)total_ps;
}

uint32_t ele_fll_eval_crit_delay_ps(uint32_t t_nom_ps, uint32_t v_nom_mv,
                                    uint32_t v_inst_mv, uint32_t v_th_mv) {
  if (v_nom_mv <= v_th_mv) {
    return t_nom_ps;
  }
  if (v_inst_mv <= v_th_mv) {
    return 0xFFFFFFFFU;
  }
  uint32_t vod_nom = v_nom_mv - v_th_mv;
  uint32_t vod_inst = v_inst_mv - v_th_mv;
  uint64_t num =
      (uint64_t)t_nom_ps * (uint64_t)vod_nom + (uint64_t)(vod_inst - 1U);
  uint64_t res = ele_div_u64_u32(num, vod_inst);
  return (res > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)res;
}

uint32_t ele_ivr_fll_step_cycle(EleIvrFll64 *fll, uint32_t load_amp) {
  if (!fll) {
    return 0U;
  }
  uint32_t v_nom = (fll->v_nom_mv >= 300U)
                       ? ((fll->v_nom_mv > 65535U) ? 65535U : fll->v_nom_mv)
                       : 750U;
  uint32_t v_guard =
      (fll->v_guard_static_mv > 65535U) ? 65535U : fll->v_guard_static_mv;
  uint32_t v_th = (fll->v_th_mv < v_nom) ? fll->v_th_mv : 238U;
  uint32_t t_nom = (fll->t_nom_ps >= 50U)
                       ? ((fll->t_nom_ps > 65535U) ? 65535U : fll->t_nom_ps)
                       : 500U;
  uint32_t slew_mv =
      (fll->ivr_slew_mv_cyc > 0U)
          ? ((fll->ivr_slew_mv_cyc > 65535U) ? 65535U : fll->ivr_slew_mv_cyc)
          : 20U;
  uint32_t v_cur = (fll->v_inst_mv > 0U && fll->v_inst_mv <= 65535U)
                       ? fll->v_inst_mv
                       : v_nom;

  /* 1. Coupled-inductor IVR charge restoration toward v_nom */
  if (v_cur < v_nom) {
    uint32_t gap = v_nom - v_cur;
    v_cur += (gap < slew_mv) ? gap : slew_mv;
  } else if (v_cur > v_nom) {
    uint32_t over = v_cur - v_nom;
    v_cur -= (over < slew_mv) ? over : slew_mv;
  }

  /* 2. Sub-nanosecond package L_pkg * max(0, dI/dt) voltage droop */
  uint32_t di_rise =
      (load_amp > fll->prev_load_amp) ? (load_amp - fll->prev_load_amp) : 0U;
  uint64_t droop_mv64 =
      ele_div_u64_u32((uint64_t)di_rise * (uint64_t)fll->l_pkg_uv_per_a, 1000U);
  uint32_t droop_step_mv =
      (droop_mv64 > 0xFFFFULL) ? 0xFFFFU : (uint32_t)droop_mv64;

  uint32_t v_floor = v_th + 64U;
  if (v_floor > v_nom) {
    v_floor = v_nom;
  }
  if (v_cur > v_floor + droop_step_mv) {
    v_cur -= droop_step_mv;
  } else {
    v_cur = v_floor;
  }
  fll->v_inst_mv = v_cur;
  fll->prev_load_amp = load_amp;

  /* 3. Normalize overdrive to 512 mV reference if near-threshold vod < 512 */
  uint32_t vod_nom = v_nom - v_th;
  uint32_t v_eff_inst = v_cur;
  if (v_cur < v_nom && vod_nom > 0U && vod_nom < 512U) {
    uint32_t raw_droop = v_nom - v_cur;
    uint64_t scaled_droop = ele_div_u64_u32(
        (uint64_t)raw_droop * 512ULL + (uint64_t)(vod_nom - 1U), vod_nom);
    v_eff_inst = (scaled_droop >= (uint64_t)v_nom)
                     ? 0U
                     : (v_nom - (uint32_t)scaled_droop);
  }

  bool emerg = false;
  uint32_t t_clk = ele_fll_eval_period_ps(t_nom, v_nom, v_eff_inst, &emerg);
  fll->t_clk_ps = t_clk;
  if (t_clk > t_nom) {
    fll->stretch_cycles += 1U;
  }

  /* 4. Audit setup-time violations (SDC) vs fixed-clock static guardband */
  uint32_t t_crit_fll = ele_fll_eval_crit_delay_ps(t_nom, v_nom, v_cur, v_th);
  if (!emerg && t_crit_fll > t_clk) {
    fll->fll_sdc_faults += 1U;
  }

  uint64_t v_static_inst64 = (uint64_t)v_cur + (uint64_t)v_guard;
  uint32_t v_static_inst =
      (v_static_inst64 > 0xFFFFULL) ? 0xFFFFU : (uint32_t)v_static_inst64;
  uint32_t t_crit_static =
      ele_fll_eval_crit_delay_ps(t_nom, v_nom, v_static_inst, v_th);
  if (t_crit_static > t_nom) {
    fll->baseline_sdc_faults += 1U;
  }

  /* 5. Dynamic energy per 1,000-cycle burst: E_dyn = C_eff * V_dd^2 (in uJ) */
  uint32_t clamped_load = (load_amp > 100000U) ? 100000U : load_amp;
  uint64_t v_static_nom = (uint64_t)v_nom + (uint64_t)v_guard;
  uint64_t e_guard = ele_div_u64_u32(
      (uint64_t)clamped_load * v_static_nom * v_static_nom, v_nom * 1000U);
  uint64_t e_fll = ele_div_u64_u32((uint64_t)clamped_load * (uint64_t)v_cur *
                                       (uint64_t)v_cur,
                                   v_nom * 1000U);
  fll->energy_guardband_uj += e_guard;
  fll->energy_fll_uj += e_fll;

  return t_clk;
}

/* ============================================================================
 * 11. 800V/48V Microsecond SiC Solid-State Circuit Breaker (`ele_hvdc_sscb`)
 * ============================================================================
 */

void ele_sscb64_init(EleSscb64 *cb, uint32_t v_bus_nom_v,
                     uint32_t v_sw_breakdown_v, uint32_t l_bus_nh,
                     uint32_t i_jet_arm_a, uint32_t i_hard_trip_a,
                     int32_t di_crit_a_tick, int32_t ddi_inrush_margin,
                     uint32_t soft_off_a_tick) {
  if (!cb) {
    return;
  }
  ele_zero64(cb);
  cb->v_bus_nom_v = (v_bus_nom_v > 0U) ? v_bus_nom_v : 800U;
  cb->v_sw_breakdown_v =
      (v_sw_breakdown_v > cb->v_bus_nom_v) ? v_sw_breakdown_v : 1200U;
  cb->l_bus_nh = (l_bus_nh > 0U) ? l_bus_nh : 600U;
  cb->i_jet_arm_a = (i_jet_arm_a > 0U) ? i_jet_arm_a : 300U;
  cb->i_hard_trip_a =
      (i_hard_trip_a > cb->i_jet_arm_a)
          ? i_hard_trip_a
          : ((cb->i_jet_arm_a <= 0x3FFFFFFFU) ? (cb->i_jet_arm_a * 4U)
                                              : 0xFFFFFFFFU);
  cb->di_crit_a_tick = (di_crit_a_tick > 0) ? di_crit_a_tick : 80;
  cb->ddi_inrush_margin = (ddi_inrush_margin >= 0) ? ddi_inrush_margin : 15;
  cb->soft_off_a_tick = (soft_off_a_tick > 0U) ? soft_off_a_tick : 140U;
  cb->gate_drive_permille = 1000U;
  cb->state = (uint8_t)ELE_SSCB_CLOSED;
  cb->slot_healthy = 1U;
}

void ele_sscb_rearm(EleSscb64 *cb) {
  if (!cb) {
    return;
  }
  cb->cur_i_amp = 0U;
  cb->di_a_tick = 0;
  cb->ddi_a_tick2 = 0;
  cb->gate_drive_permille = 1000U;
  cb->clear_ticks_250ns = 0U;
  cb->primed = 0U;
  cb->state = (uint8_t)ELE_SSCB_CLOSED;
  cb->fault_streak = 0U;
  cb->slot_healthy = 1U;
}

uint32_t ele_sscb_step_250ns(EleSscb64 *cb, uint32_t measured_i_amp) {
  if (!cb) {
    return 0U;
  }
  if (cb->state == (uint8_t)ELE_SSCB_OPEN_ISOLATED) {
    cb->cur_i_amp = 0U;
    cb->di_a_tick = 0;
    cb->ddi_a_tick2 = 0;
    cb->gate_drive_permille = 0U;
    cb->slot_healthy = 0U;
    return 0U;
  }

  uint32_t soft_step = (cb->soft_off_a_tick > 0U) ? cb->soft_off_a_tick : 140U;

  if (cb->state == (uint8_t)ELE_SSCB_SOFT_TURNOFF) {
    uint32_t ramped_i =
        (cb->cur_i_amp > soft_step) ? (cb->cur_i_amp - soft_step) : 0U;
    uint32_t next_i = (measured_i_amp < ramped_i) ? measured_i_amp : ramped_i;
    int32_t new_di = ele_sat_i64_i32((int64_t)next_i - (int64_t)cb->cur_i_amp);
    int32_t new_ddi = ele_sat_i64_i32((int64_t)new_di - (int64_t)cb->di_a_tick);
    cb->cur_i_amp = next_i;
    cb->di_a_tick = new_di;
    cb->ddi_a_tick2 = new_ddi;
    if (cb->clear_ticks_250ns < 255U) {
      cb->clear_ticks_250ns += 1U;
    }
    if (next_i == 0U) {
      cb->state = (uint8_t)ELE_SSCB_OPEN_ISOLATED;
      cb->gate_drive_permille = 0U;
      cb->slot_healthy = 0U;
    }
    return next_i;
  }

  /* State == ELE_SSCB_CLOSED: 2-Jet Inrush vs Short-Circuit Discriminator */
  int32_t new_di = 0;
  int32_t new_ddi = 0;
  if (cb->primed == 0U) {
    cb->primed = 1U;
  } else {
    new_di = ele_sat_i64_i32((int64_t)measured_i_amp - (int64_t)cb->cur_i_amp);
    new_ddi = ele_sat_i64_i32((int64_t)new_di - (int64_t)cb->di_a_tick);
  }

  uint32_t i_arm = (cb->i_jet_arm_a > 0U) ? cb->i_jet_arm_a : 300U;
  uint32_t i_hard = (cb->i_hard_trip_a > i_arm)
                        ? cb->i_hard_trip_a
                        : ((i_arm <= 0x3FFFFFFFU) ? (i_arm * 4U) : 0xFFFFFFFFU);
  int64_t margin =
      (cb->ddi_inrush_margin >= 0) ? (int64_t)cb->ddi_inrush_margin : 0LL;
  int64_t neg_margin = -margin;

  bool hard_fault = (measured_i_amp >= i_hard);
  bool jet_fault = (measured_i_amp >= i_arm) &&
                   (new_di >= cb->di_crit_a_tick) &&
                   ((int64_t)new_ddi >= neg_margin);
  bool rc_inrush = (measured_i_amp >= i_arm) && (new_di > 0) &&
                   ((int64_t)new_ddi < neg_margin);

  cb->cur_i_amp = measured_i_amp;
  cb->di_a_tick = new_di;
  cb->ddi_a_tick2 = new_ddi;

  if (rc_inrush && cb->inrush_ride_throughs < 255U) {
    cb->inrush_ride_throughs += 1U;
  }

  uint8_t next_streak = 0U;
  if (jet_fault) {
    next_streak =
        (cb->fault_streak < 255U) ? (uint8_t)(cb->fault_streak + 1U) : 255U;
  }
  cb->fault_streak = next_streak;

  if (hard_fault || next_streak >= 2U) {
    cb->state = (uint8_t)ELE_SSCB_SOFT_TURNOFF;
    cb->gate_drive_permille = 350U;
    cb->clear_ticks_250ns = 1U;
    if (measured_i_amp > cb->peak_fault_i_amp) {
      cb->peak_fault_i_amp = measured_i_amp;
    }
    uint64_t v_soft =
        (uint64_t)cb->v_bus_nom_v +
        ele_div_u64_u32((uint64_t)cb->l_bus_nh * (uint64_t)soft_step, 250U);
    uint64_t v_hard =
        (uint64_t)cb->v_bus_nom_v +
        ele_div_u64_u32((uint64_t)cb->l_bus_nh * (uint64_t)measured_i_amp, 25U);
    uint32_t vs32 = (v_soft > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)v_soft;
    uint32_t vh32 = (v_hard > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)v_hard;
    if (vs32 > cb->peak_v_sw_soft_v) {
      cb->peak_v_sw_soft_v = vs32;
    }
    if (vh32 > cb->peak_v_sw_hard_v) {
      cb->peak_v_sw_hard_v = vh32;
    }
  }

  return measured_i_amp;
}

/* ============================================================================
 * 12. Two-Phase Microchannel Boiling & Cube-Law Pump (`ele_thermo_fluid`)
 * ============================================================================
 */

void ele_coldplate64_init(EleColdPlate64 *cp, uint32_t flow_max_ml_s,
                          uint32_t flow_min_ml_s, uint32_t pump_rated_mw,
                          uint32_t h_fg_j_per_ml, int32_t t_coolant_in_mc,
                          uint32_t pump_slew_ml_tick,
                          uint16_t chf_crit_permille) {
  if (!cp) {
    return;
  }
  ele_zero64(cp);
  cp->flow_max_ml_s = (flow_max_ml_s >= 5U) ? flow_max_ml_s : 50U;
  cp->flow_min_ml_s = (flow_min_ml_s > 0U && flow_min_ml_s <= cp->flow_max_ml_s)
                          ? flow_min_ml_s
                          : 10U;
  cp->pump_rated_mw = (pump_rated_mw > 0U) ? pump_rated_mw : 60000U;
  cp->h_fg_j_per_ml = (h_fg_j_per_ml > 0U) ? h_fg_j_per_ml : 40U;
  cp->t_coolant_in_mc = t_coolant_in_mc;
  cp->t_junc_mc = ele_sat_i64_i32((int64_t)t_coolant_in_mc + 2000LL);
  cp->cur_flow_ml_s = cp->flow_min_ml_s;
  cp->pump_slew_ml_tick = (pump_slew_ml_tick > 0U) ? pump_slew_ml_tick : 6U;
  cp->chf_crit_permille =
      (chf_crit_permille >= 100U && chf_crit_permille <= 950U)
          ? chf_crit_permille
          : 700U;
}

uint32_t ele_thermo_eval_pump_mw(const EleColdPlate64 *cp, uint32_t flow_ml_s) {
  if (!cp) {
    return 0U;
  }
  uint32_t f_max = (cp->flow_max_ml_s > 0U) ? cp->flow_max_ml_s : 50U;
  uint32_t p_rated = (cp->pump_rated_mw > 0U) ? cp->pump_rated_mw : 60000U;
  uint32_t f = (flow_ml_s > f_max) ? f_max : flow_ml_s;

  /* 4% static bearing/controller tare + 96% hydraulic shaft cube law */
  uint32_t p_idle = p_rated / 25U;
  uint32_t p_shaft = p_rated - p_idle;
  uint32_t u_permille = (uint32_t)ele_div_u64_u32((uint64_t)f * 1000ULL, f_max);
  uint32_t u_cubed = u_permille * u_permille * u_permille;
  uint32_t dyn_mw = (uint32_t)ele_div_u64_u32(
      (uint64_t)p_shaft * (uint64_t)u_cubed, 1000000000U);
  return p_idle + dyn_mw;
}

uint32_t ele_coldplate_step_10ms(EleColdPlate64 *cp, uint32_t q_chip_watt,
                                 uint32_t upcoming_q_watt) {
  if (!cp) {
    return 0U;
  }
  uint32_t f_max =
      (cp->flow_max_ml_s >= 5U)
          ? ((cp->flow_max_ml_s > 65535U) ? 65535U : cp->flow_max_ml_s)
          : 50U;
  uint32_t f_min = (cp->flow_min_ml_s > 0U && cp->flow_min_ml_s <= f_max)
                       ? cp->flow_min_ml_s
                       : 10U;
  uint32_t h_fg =
      (cp->h_fg_j_per_ml > 0U)
          ? ((cp->h_fg_j_per_ml > 65535U) ? 65535U : cp->h_fg_j_per_ml)
          : 40U;
  uint32_t slew =
      (cp->pump_slew_ml_tick > 0U)
          ? ((cp->pump_slew_ml_tick > f_max) ? f_max : cp->pump_slew_ml_tick)
          : 6U;
  uint16_t x_crit =
      (cp->chf_crit_permille >= 100U && cp->chf_crit_permille <= 950U)
          ? cp->chf_crit_permille
          : 700U;

  uint32_t q_now = (q_chip_watt > 50000U) ? 50000U : q_chip_watt;
  uint32_t q_up = (upcoming_q_watt > 50000U) ? 50000U : upcoming_q_watt;
  uint32_t q_prev = (cp->prev_q_watt > 50000U) ? 50000U : cp->prev_q_watt;

  /* 1. 2-Jet thermal heat-flux lookahead across 5-tick (50 ms) rotor inertia */
  int32_t new_dq = (int32_t)q_now - (int32_t)q_prev;
  int32_t ddq = ele_sat_i64_i32((int64_t)new_dq - (int64_t)cp->dq_watt_tick);
  cp->prev_q_watt = q_now;
  cp->dq_watt_tick = new_dq;

  int64_t pos_dq = (new_dq > 0) ? (int64_t)new_dq : 0LL;
  int64_t pos_ddq = (ddq > 0) ? (int64_t)ddq : 0LL;
  uint64_t q_pred = (uint64_t)q_now + (uint64_t)(pos_dq * 5LL) +
                    (uint64_t)((pos_ddq * 25LL) >> 1U);
  if ((uint64_t)q_up > q_pred) {
    q_pred = (uint64_t)q_up;
  }

  /* 2. Target 75% of critical vapor quality for safe nucleate boiling */
  uint32_t x_target = ((uint32_t)x_crit * 3U) >> 2U;
  if (x_target == 0U) {
    x_target = 500U;
  }
  uint32_t denom = h_fg * x_target;
  uint64_t req_flow64 =
      ele_div_u64_u32(q_pred * 1000ULL + (uint64_t)(denom - 1U), denom);
  uint32_t target_flow =
      (req_flow64 > (uint64_t)f_max)
          ? f_max
          : ((req_flow64 < (uint64_t)f_min) ? f_min : (uint32_t)req_flow64);

  /* Counterfactual reactive thermostat flow (lags behind burst onset) */
  uint64_t prev_req64 = ele_div_u64_u32(
      (uint64_t)q_prev * 1000ULL + (uint64_t)(denom - 1U), denom);
  uint32_t react_base =
      (prev_req64 > (uint64_t)f_max)
          ? f_max
          : ((prev_req64 < (uint64_t)f_min) ? f_min : (uint32_t)prev_req64);
  uint32_t react_flow =
      (react_base + slew > f_max) ? f_max : (react_base + slew);

  /* 3. Mechanical rotor slew-rate evolution */
  uint32_t f_cur =
      (cp->cur_flow_ml_s >= f_min)
          ? ((cp->cur_flow_ml_s > f_max) ? f_max : cp->cur_flow_ml_s)
          : f_min;
  if (f_cur < target_flow) {
    uint32_t up = target_flow - f_cur;
    f_cur += (up < slew) ? up : slew;
  } else if (f_cur > target_flow) {
    uint32_t down_slew = (slew + 1U) >> 1U;
    uint32_t down = f_cur - target_flow;
    f_cur -= (down < down_slew) ? down : down_slew;
  }
  cp->cur_flow_ml_s = f_cur;

  /* 4. Evaluate microchannel exit vapor quality & CHF dryout crisis */
  uint64_t x_symp64 = ele_div_u64_u32((uint64_t)q_now * 1000ULL, f_cur * h_fg);
  uint64_t x_react64 =
      ele_div_u64_u32((uint64_t)q_now * 1000ULL, react_flow * h_fg);
  uint16_t x_symp = (x_symp64 > 1000ULL) ? 1000U : (uint16_t)x_symp64;
  cp->vapor_qual_permille = x_symp;

  if (x_react64 >= (uint64_t)x_crit && cp->dryout_events_react < 65535U) {
    cp->dryout_events_react += 1U;
  }

  int64_t t_sat_mc = (int64_t)cp->t_coolant_in_mc + 8000LL;
  int64_t t_eq_mc = t_sat_mc;
  if (x_symp >= x_crit) {
    /* Film-boiling vapor blanketing: conductance collapses 8x (200 mC/W) */
    if (cp->dryout_events_symp < 65535U) {
      cp->dryout_events_symp += 1U;
    }
    t_eq_mc = t_sat_mc + (int64_t)q_now * 200LL;
  } else {
    /* Nucleate boiling: high two-phase conductance (22 mC/W) */
    t_eq_mc = t_sat_mc + (int64_t)q_now * 22LL;
  }
  int64_t step_mc = ele_asr64(t_eq_mc - (int64_t)cp->t_junc_mc, 2U);
  cp->t_junc_mc = ele_sat_i64_i32((int64_t)cp->t_junc_mc + step_mc);

  /* 5. Accumulate 10 ms (0.01 s) pump energy in mJ: E_mJ = P_mW / 100 */
  uint32_t p_fixed_mw = ele_thermo_eval_pump_mw(cp, f_max);
  uint32_t p_symp_mw = ele_thermo_eval_pump_mw(cp, f_cur);
  cp->pump_energy_fixed_mj += (uint64_t)((p_fixed_mw + 50U) / 100U);
  cp->pump_energy_symp_mj += (uint64_t)((p_symp_mw + 50U) / 100U);

  return f_cur;
}

/* ============================================================================
 * 13. Silicon Lifecycle Management, Zero-Stall Trace & Canary DFT (`ele_slm`)
 * ============================================================================
 */

void ele_trace_box64_init(EleTraceBox64 *box, uint16_t v_eps_mv) {
  if (!box) {
    return;
  }
  ele_zero64(box);
  box->v_eps_mv = (v_eps_mv > 4095U) ? 4095U : v_eps_mv;
  box->pc_stride16 = 4U;
}

void ele_trace_box_rearm(EleTraceBox64 *box) {
  if (!box) {
    return;
  }
  box->wr_idx = 0U;
  box->freeze_idx = 0U;
  box->frozen = 0U;
}

bool ele_trace_step_cycle(EleTraceBox64 *box, uint16_t pc16, uint16_t v_dd_mv,
                          uint8_t hw_flags4, bool fault_trigger) {
  if (!box) {
    return false;
  }
  uint16_t v12 = (v_dd_mv > 4095U) ? 4095U : v_dd_mv;
  uint16_t eps12 = (box->v_eps_mv > 4095U) ? 4095U : box->v_eps_mv;
  uint32_t snap = ele_trace_pack_snap32(pc16, hw_flags4, v12);

  /* 1. Pre-trigger 8-slot circular flight recorder (0-stall SRAM/FF ring) */
  if (box->frozen == 0U) {
    uint8_t cur_wr = (uint8_t)(box->wr_idx & 7U);
    box->ring_snap[cur_wr] = snap;
    if (fault_trigger) {
      box->frozen = 1U;
      box->freeze_idx = cur_wr;
      if (box->trigger_count < 65535U) {
        box->trigger_count += 1U;
      }
    } else {
      box->wr_idx = (uint8_t)((cur_wr + 1U) & 7U);
    }
  }

  /* 2. Zero-stall 2-Jet PVT + modular PC-stride trace compressor */
  bool quiet = false;
  if (box->primed == 0U) {
    box->primed = 1U;
    box->prev_pc16 = pc16;
    box->pc_stride16 = 4U;
    box->prev_v_mv = v12;
    box->dv_mv_cyc = 0;
  } else {
    uint16_t expected_pc =
        (uint16_t)((uint32_t)box->prev_pc16 + (uint32_t)box->pc_stride16);
    int32_t pred_v = (int32_t)box->prev_v_mv + (int32_t)box->dv_mv_cyc;
    int32_t v_err = (int32_t)v12 - pred_v;
    uint32_t abs_v_err = ele_abs_i32_u32(v_err);

    quiet = (!fault_trigger) && (pc16 == expected_pc) &&
            (abs_v_err <= (uint32_t)eps12);

    uint16_t new_stride = (uint16_t)((uint32_t)pc16 - (uint32_t)box->prev_pc16);
    int16_t new_dv = (int16_t)((int32_t)v12 - (int32_t)box->prev_v_mv);
    box->prev_pc16 = pc16;
    box->pc_stride16 = new_stride;
    box->prev_v_mv = v12;
    box->dv_mv_cyc = new_dv;
  }

  if (box->full_trace_bits <= 0xFFFFFFFFU - 32U) {
    box->full_trace_bits += 32U;
  } else {
    box->full_trace_bits = 0xFFFFFFFFU;
  }

  if (quiet) {
    if (box->suppressed_cycles < 0xFFFFFFFFU) {
      box->suppressed_cycles += 1U;
    }
  } else {
    if (box->emitted_packets < 0xFFFFFFFFU) {
      box->emitted_packets += 1U;
    }
    if (box->slm_trace_bits <= 0xFFFFFFFFU - 32U) {
      box->slm_trace_bits += 32U;
    } else {
      box->slm_trace_bits = 0xFFFFFFFFU;
    }
  }
  return !quiet;
}

uint32_t ele_trace_read_ordered(const EleTraceBox64 *box, uint8_t age_idx) {
  if (!box) {
    return 0U;
  }
  uint8_t k = (uint8_t)(age_idx & 7U);
  uint8_t slot = (box->frozen != 0U)
                     ? (uint8_t)((box->freeze_idx + 1U + k) & 7U)
                     : (uint8_t)((box->wr_idx + k) & 7U);
  return box->ring_snap[slot];
}

void ele_canary64_init(EleAgingCanary64 *can, uint32_t v_th_base_mv,
                       uint32_t canary_guard_ps, uint16_t max_aged_vth_mv,
                       uint32_t r_baseline_uohm, uint32_t r_warn_uohm) {
  if (!can) {
    return;
  }
  ele_zero64(can);
  can->v_th_base_mv =
      (v_th_base_mv > 0U && v_th_base_mv <= 2000U) ? v_th_base_mv : 238U;
  can->canary_guard_ps = (canary_guard_ps > 0U && canary_guard_ps <= 5000U)
                             ? canary_guard_ps
                             : 35U;
  can->max_aged_vth_mv = (max_aged_vth_mv > (uint16_t)can->v_th_base_mv)
                             ? max_aged_vth_mv
                             : (uint16_t)(can->v_th_base_mv + 120U);
  can->r_baseline_uohm = (r_baseline_uohm > 0U) ? r_baseline_uohm : 200U;
  can->r_warn_uohm = (r_warn_uohm > can->r_baseline_uohm) ? r_warn_uohm : 600U;
  can->r_measured_uohm = can->r_baseline_uohm;
}

uint32_t ele_canary_step_and_heal(EleAgingCanary64 *can, EleIvrFll64 *fll,
                                  uint32_t load_amp,
                                  uint32_t true_nbti_shift_mv,
                                  uint32_t delta_kilohours) {
  if (!can || !fll) {
    return 0U;
  }
  if (can->stress_kilohours <= 0xFFFFFFFFU - delta_kilohours) {
    can->stress_kilohours += delta_kilohours;
  } else {
    can->stress_kilohours = 0xFFFFFFFFU;
  }

  uint32_t v_nom = (fll->v_nom_mv >= 300U)
                       ? ((fll->v_nom_mv > 65535U) ? 65535U : fll->v_nom_mv)
                       : 750U;
  uint32_t t_nom = (fll->t_nom_ps >= 50U)
                       ? ((fll->t_nom_ps > 65535U) ? 65535U : fll->t_nom_ps)
                       : 500U;
  uint32_t v_th0 = (can->v_th_base_mv < v_nom) ? can->v_th_base_mv : 238U;
  uint32_t guard_ps =
      (can->canary_guard_ps <= 5000U) ? can->canary_guard_ps : 35U;
  uint32_t nbti_mv = (true_nbti_shift_mv > 256U) ? 256U : true_nbti_shift_mv;

  /* 1. Step IVR rail voltage using fresh v_th0 baseline, then add compensated
   *    NBTI shift `est_vth_shift_mv` as effective overdrive droop into AD-FLL!
   */
  fll->v_th_mv = v_th0;
  (void)ele_ivr_fll_step_cycle(fll, load_amp);

  uint32_t v_inst = fll->v_inst_mv;
  uint32_t comp_shift =
      (can->est_vth_shift_mv > 256U) ? 256U : can->est_vth_shift_mv;
  uint32_t v_fll_eff = (v_inst > comp_shift) ? (v_inst - comp_shift) : 0U;
  uint32_t vod_fresh = (v_nom > v_th0) ? (v_nom - v_th0) : 512U;
  if (v_fll_eff < v_nom && vod_fresh > 0U && vod_fresh < 512U) {
    uint32_t raw_droop = v_nom - v_fll_eff;
    uint64_t scaled = ele_div_u64_u32(
        (uint64_t)raw_droop * 512ULL + (uint64_t)(vod_fresh - 1U), vod_fresh);
    v_fll_eff = (scaled >= (uint64_t)v_nom) ? 0U : (v_nom - (uint32_t)scaled);
  }
  bool emerg = false;
  uint32_t t_clk_healed =
      ele_fll_eval_period_ps(t_nom, v_nom, v_fll_eff, &emerg);
  fll->t_clk_ps = t_clk_healed;

  /* 2. True aged critical-path delay: T_nom * (V_nom - V_th0) / (V_inst -
   * V_th_aged) */
  uint32_t v_th_aged = v_th0 + nbti_mv;
  uint32_t t_main_aged = 0xFFFFFFFFU;
  if (v_inst > v_th_aged) {
    uint32_t vod_aged = v_inst - v_th_aged;
    uint64_t num =
        (uint64_t)t_nom * (uint64_t)vod_fresh + (uint64_t)(vod_aged - 1U);
    uint64_t d64 = ele_div_u64_u32(num, vod_aged);
    t_main_aged = (d64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)d64;
  }

  /* Counterfactual unmonitored AD-FLL (unaware of NBTI V_th shift) */
  bool emerg_unmon = false;
  uint32_t t_clk_unmon =
      ele_fll_eval_period_ps(t_nom, v_nom, v_inst, &emerg_unmon);
  if (!emerg_unmon && t_main_aged > t_clk_unmon) {
    can->main_sdc_unmonitored += 1U;
  }

  /* Audit healed main-path SDC (0 violations while canary guards margin) */
  if (!emerg && t_main_aged > t_clk_healed) {
    can->main_sdc_healed += 1U;
  }

  /* 3. Canary shadow path (`t_main_aged + guard_ps`) & closed-loop healing */
  uint64_t t_canary64 = (uint64_t)t_main_aged + (uint64_t)guard_ps;
  if (t_canary64 > (uint64_t)t_clk_healed) {
    can->canary_alarm = 1U;
    can->canary_pre_alarms += 1U;
    if (can->est_vth_shift_mv + 8U <= 256U) {
      can->est_vth_shift_mv += 8U;
      can->self_heal_steps += 1U;
    }
  } else {
    can->canary_alarm = 0U;
  }

  if (v_th0 + can->est_vth_shift_mv >= (uint32_t)can->max_aged_vth_mv) {
    can->retire_recommended = 1U;
  }
  fll->v_th_mv = v_th0 + can->est_vth_shift_mv;

  can->rem_slack_ps =
      (t_clk_healed > t_main_aged) ? (t_clk_healed - t_main_aged) : 0U;
  return can->rem_slack_ps;
}

uint32_t ele_canary_probe_contact_uohm(EleAgingCanary64 *can,
                                       uint32_t sense_v_uv, uint32_t load_amp) {
  if (!can) {
    return 0U;
  }
  if (can->sample_primed == 0U || load_amp <= can->prev_sample_i_amp) {
    can->prev_sample_v_uv = sense_v_uv;
    can->prev_sample_i_amp = load_amp;
    can->sample_primed = 1U;
    return can->r_measured_uohm;
  }

  uint32_t di_a = load_amp - can->prev_sample_i_amp;
  if (di_a >= 20U && can->prev_sample_v_uv >= sense_v_uv) {
    uint32_t dv_uv = can->prev_sample_v_uv - sense_v_uv;
    uint32_t r_uohm = dv_uv / di_a;
    can->r_measured_uohm = r_uohm;
    uint32_t r_warn = (can->r_warn_uohm > 0U) ? can->r_warn_uohm : 600U;
    if (r_uohm >= r_warn) {
      can->solder_fatigue_warn = 1U;
      if (can->contact_fault_events < 65535U) {
        can->contact_fault_events += 1U;
      }
    } else {
      can->solder_fatigue_warn = 0U;
    }
  }
  return can->r_measured_uohm;
}

/* ============================================================================
 * 14. CPO Silicon Photonics Microring Modulo-FSR & Lock (`ele_photonics_cpo`)
 * ============================================================================
 */

void ele_cpo_mrr64_init(EleCpoMrr64 *mrr, uint16_t target_phase12,
                        uint16_t base_temp_c, uint16_t k_to_phase_per_c,
                        uint16_t uw_per_phase_lsb, uint16_t lock_tol_phase12,
                        uint16_t k_gpu_ff_mdeg_per_w,
                        uint16_t bistable_limit_uw) {
  if (!mrr) {
    return;
  }
  ele_zero64(mrr);
  mrr->target_phase12 = target_phase12 & 0x0FFFU;
  mrr->base_temp_c =
      (base_temp_c > 0U && base_temp_c <= 150U) ? base_temp_c : 25U;
  mrr->k_to_phase_per_c =
      (k_to_phase_per_c > 0U) ? k_to_phase_per_c : 512U; /* 8 C per FSR */
  mrr->uw_per_phase_lsb = (uw_per_phase_lsb > 0U) ? uw_per_phase_lsb : 4U;
  mrr->lock_tol_phase12 = (lock_tol_phase12 > 0U && lock_tol_phase12 <= 2048U)
                              ? lock_tol_phase12
                              : 64U;
  mrr->k_gpu_ff_mdeg_per_w =
      (k_gpu_ff_mdeg_per_w > 0U) ? k_gpu_ff_mdeg_per_w : 25U;
  mrr->bistable_limit_uw = (bistable_limit_uw > 0U) ? bistable_limit_uw : 4000U;
  mrr->die_temp_mc = (int32_t)mrr->base_temp_c * 1000;
}

uint16_t ele_cpo_eval_heater_phase12(uint16_t target_phase12,
                                     uint16_t thermal_phase12,
                                     int16_t ff_delta_phase,
                                     uint16_t prev_heater_phase12,
                                     bool *out_hop) {
  uint16_t tgt12 = target_phase12 & 0x0FFFU;
  uint16_t th12 = thermal_phase12 & 0x0FFFU;
  uint16_t ff12 = ((uint16_t)ff_delta_phase) & 0x0FFFU;
  uint16_t pred12 = (uint16_t)((th12 + ff12) & 0x0FFFU);
  uint16_t next12 = (uint16_t)((tgt12 - pred12) & 0x0FFFU);
  uint16_t prev12 = prev_heater_phase12 & 0x0FFFU;
  uint16_t jump12 = (next12 >= prev12) ? (next12 - prev12) : (prev12 - next12);
  if (out_hop) {
    *out_hop = (jump12 > 2048U);
  }
  return next12;
}

uint32_t ele_cpo_mrr_step_ms(EleCpoMrr64 *mrr, int32_t meas_temp_mc,
                             uint16_t gpu_watt, uint16_t laser_in_uw) {
  if (!mrr) {
    return 0U;
  }
  uint16_t k_to = (mrr->k_to_phase_per_c > 0U) ? mrr->k_to_phase_per_c : 512U;
  uint16_t uw_lsb = (mrr->uw_per_phase_lsb > 0U) ? mrr->uw_per_phase_lsb : 4U;
  uint16_t tol12 = (mrr->lock_tol_phase12 > 0U) ? mrr->lock_tol_phase12 : 64U;
  uint16_t bi_cap =
      (mrr->bistable_limit_uw > 0U) ? mrr->bistable_limit_uw : 4000U;

  /* 1. Clamp input optical power strictly below two-photon bistability knee */
  if (laser_in_uw > bi_cap) {
    mrr->opt_power_uw = bi_cap;
    mrr->bistable_clamped = 1U;
  } else {
    mrr->opt_power_uw = laser_in_uw;
    mrr->bistable_clamped = 0U;
  }
  mrr->self_heat_phase12 = (uint16_t)(((uint32_t)mrr->opt_power_uw * 3U) >> 6U);

  /* 2. Compute unwrapped thermo-optic phase shift relative to base_temp_c */
  int64_t base_mc = (int64_t)mrr->base_temp_c * 1000LL;
  int64_t dt_mc = (int64_t)meas_temp_mc - base_mc;
  uint64_t abs_dt_mc =
      (dt_mc < 0LL) ? (0ULL - (uint64_t)dt_mc) : (uint64_t)dt_mc;
  uint64_t mag_phase = ele_div_u64_u32(abs_dt_mc * (uint64_t)k_to, 1000U);
  uint16_t th_phase12 = (dt_mc < 0LL)
                            ? (uint16_t)((0U - (uint32_t)mag_phase) & 0x0FFFU)
                            : (uint16_t)((uint32_t)mag_phase & 0x0FFFU);
  th_phase12 = (uint16_t)((th_phase12 + mrr->self_heat_phase12) & 0x0FFFU);

  /* 3. 2-Jet thermal velocity + adjacent XPU power-step wave feedforward */
  int32_t d_temp =
      (mrr->primed != 0U)
          ? ele_sat_i64_i32((int64_t)meas_temp_mc - (int64_t)mrr->die_temp_mc)
          : 0;
  int32_t d_gpu_w = (mrr->primed != 0U)
                        ? ((int32_t)gpu_watt - (int32_t)mrr->prev_gpu_watt)
                        : 0;
  int64_t wave_mc =
      (int64_t)d_temp + (int64_t)d_gpu_w * (int64_t)mrr->k_gpu_ff_mdeg_per_w;
  uint64_t abs_wave_mc =
      (wave_mc < 0LL) ? (0ULL - (uint64_t)wave_mc) : (uint64_t)wave_mc;
  uint64_t wave_mag = ele_div_u64_u32(abs_wave_mc * (uint64_t)k_to, 1000U);
  if (wave_mag > 2047ULL) {
    wave_mag = 2047ULL;
  }
  int16_t ff_phase = (wave_mc < 0LL) ? -(int16_t)wave_mag : (int16_t)wave_mag;

  bool hop = false;
  uint16_t next_h12 = ele_cpo_eval_heater_phase12(
      mrr->target_phase12, th_phase12, ff_phase, mrr->heater_phase12, &hop);
  if (mrr->primed != 0U && hop) {
    mrr->hop_event = 1U;
    if (mrr->fsr_hops < 0xFFFFFFFFU) {
      mrr->fsr_hops += 1U;
    }
  } else {
    mrr->hop_event = 0U;
  }

  /* 4. Audit carrier phase error at end of 1 ms frame as heat wave arrives */
  uint16_t actual_wave12 =
      (uint16_t)((th_phase12 + ((uint16_t)ff_phase & 0x0FFFU)) & 0x0FFFU);
  uint16_t comp_total12 = (uint16_t)((actual_wave12 + next_h12) & 0x0FFFU);
  uint16_t diff12 =
      (uint16_t)(((mrr->target_phase12 & 0x0FFFU) - comp_total12) & 0x0FFFU);
  uint16_t err12 = (diff12 > 2048U) ? (uint16_t)(4096U - diff12) : diff12;
  mrr->last_phase_err12 = err12;
  if (err12 <= tol12) {
    mrr->locked = 1U;
  } else {
    mrr->locked = 0U;
    if (mrr->unlock_drops_hop < 0xFFFFFFFFU) {
      mrr->unlock_drops_hop += 1U;
    }
  }

  /* Counterfactual reactive loop without 2-Jet / XPU feedforward (ff = 0) */
  uint16_t no_ff_h12 = ele_cpo_eval_heater_phase12(
      mrr->target_phase12, th_phase12, 0, mrr->heater_phase12, NULL);
  uint16_t no_ff_tot12 = (uint16_t)((actual_wave12 + no_ff_h12) & 0x0FFFU);
  uint16_t no_ff_diff12 =
      (uint16_t)(((mrr->target_phase12 & 0x0FFFU) - no_ff_tot12) & 0x0FFFU);
  uint16_t no_ff_err12 =
      (no_ff_diff12 > 2048U) ? (uint16_t)(4096U - no_ff_diff12) : no_ff_diff12;
  if (no_ff_err12 > tol12 && mrr->unlock_drops_no_ff < 0xFFFFFFFFU) {
    mrr->unlock_drops_no_ff += 1U;
  }

  /* 5. Heater power & 1 ms energy (1 mW * 1 ms = 1 uJ): Modulo-FSR vs Fixed */
  uint32_t p_hop_uw = (uint32_t)next_h12 * (uint32_t)uw_lsb;
  mrr->heater_power_uw = (p_hop_uw > 65535U) ? 65535U : (uint16_t)p_hop_uw;

  /* Fixed-mode heater must hold ring above a 4-FSR (16,384 LSB) ceiling */
  uint64_t fixed_lsb =
      (mag_phase < 16384ULL) ? (16384ULL - mag_phase) : 4096ULL;
  uint64_t p_conv_uw = fixed_lsb * (uint64_t)uw_lsb;
  uint32_t d_hop_uj = (p_hop_uw + 500U) / 1000U;
  uint32_t d_conv_uj = (uint32_t)ele_div_u64_u32(p_conv_uw + 500ULL, 1000U);

  if (mrr->energy_hop_uj <= 0xFFFFFFFFU - d_hop_uj) {
    mrr->energy_hop_uj += d_hop_uj;
  } else {
    mrr->energy_hop_uj = 0xFFFFFFFFU;
  }
  if (mrr->energy_conv_uj <= 0xFFFFFFFFU - d_conv_uj) {
    mrr->energy_conv_uj += d_conv_uj;
  } else {
    mrr->energy_conv_uj = 0xFFFFFFFFU;
  }
  if (mrr->total_frames < 0xFFFFFFFFU) {
    mrr->total_frames += 1U;
  }

  mrr->heater_phase12 = next_h12;
  mrr->prev_raw_phase12 = th_phase12;
  mrr->die_temp_mc = meas_temp_mc;
  mrr->d_temp_mc = d_temp;
  mrr->prev_gpu_watt = gpu_watt;
  mrr->primed = 1U;
  return p_hop_uw;
}

/* ============================================================================
 * 15. HBM4 3D Thermal-Refresh Breaker & Rowhammer Guard (`ele_hbm_refresh`)
 * ============================================================================
 */

static inline uint8_t ele_hbm_eval_jedec_mult(int32_t temp_mc) {
  if (temp_mc >= 95000) {
    return 8U;
  }
  if (temp_mc >= 85000) {
    return 4U;
  }
  if (temp_mc >= 75000) {
    return 2U;
  }
  return 1U;
}

void ele_hbm_stack64_init(EleHbmStack64 *hbm, int32_t amb_temp_mc,
                          uint16_t r_th_mc_per_w, uint16_t p_ref_bank_mw,
                          uint16_t rh_drfm_thresh, uint16_t live_bank_mask) {
  if (!hbm) {
    return;
  }
  ele_zero64(hbm);
  hbm->temp_jedec_mc = amb_temp_mc;
  hbm->temp_symp_mc = amb_temp_mc;
  hbm->r_th_mc_per_w = (r_th_mc_per_w > 0U) ? r_th_mc_per_w : 1150U;
  hbm->p_ref_bank_mw = (p_ref_bank_mw > 0U) ? p_ref_bank_mw : 350U;
  hbm->rh_drfm_thresh = (rh_drfm_thresh > 0U) ? rh_drfm_thresh : 256U;
  hbm->live_bank_mask = live_bank_mask;
  hbm->cur_ref_mult = ele_hbm_eval_jedec_mult(amb_temp_mc);
}

void ele_hbm_rowhammer_window_reset(EleHbmStack64 *hbm) {
  if (!hbm) {
    return;
  }
  hbm->rh_cnt[0] = 0U;
  hbm->rh_cnt[1] = 0U;
  hbm->rh_cnt[2] = 0U;
  hbm->rh_cnt[3] = 0U;
  hbm->rh_spill_cnt = 0U;
  hbm->rh_valid_mask = 0U;
}

bool ele_hbm_step_row_activate(EleHbmStack64 *hbm, uint16_t row_id,
                               bool *out_preventive_ref) {
  if (out_preventive_ref) {
    *out_preventive_ref = false;
  }
  if (!hbm) {
    return false;
  }
  uint16_t eff_thresh = (hbm->rh_drfm_thresh == 0U) ? 1U : hbm->rh_drfm_thresh;

  /* 1. Check 4-slot associative match (priority 0 -> 1 -> 2 -> 3) */
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (((hbm->rh_valid_mask >> i) & 1U) != 0U && hbm->rh_row[i] == row_id) {
      uint32_t inc = (uint32_t)hbm->rh_cnt[i] + 1U;
      if (inc >= (uint32_t)eff_thresh) {
        hbm->rh_cnt[i] = hbm->rh_spill_cnt;
        hbm->last_drfm_row = row_id;
        if (hbm->drfm_mitigations < 0xFFFFFFFFU) {
          hbm->drfm_mitigations += 1U;
        }
        return true;
      }
      hbm->rh_cnt[i] = (uint16_t)inc;
      return false;
    }
  }

  /* 2. Allocate free slot if any available */
  if ((hbm->rh_valid_mask & 0x0FU) != 0x0FU) {
    uint8_t free_idx = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
      if (((hbm->rh_valid_mask >> i) & 1U) == 0U) {
        free_idx = i;
        break;
      }
    }
    hbm->rh_valid_mask |= (uint8_t)(1U << free_idx);
    hbm->rh_row[free_idx] = row_id;
    uint32_t inc = (uint32_t)hbm->rh_spill_cnt + 1U;
    if (inc >= (uint32_t)eff_thresh) {
      hbm->rh_cnt[free_idx] = hbm->rh_spill_cnt;
      hbm->last_drfm_row = row_id;
      if (hbm->drfm_mitigations < 0xFFFFFFFFU) {
        hbm->drfm_mitigations += 1U;
      }
      return true;
    }
    hbm->rh_cnt[free_idx] = (uint16_t)inc;
    return false;
  }

  /* 3. Exact 2-level tournament min-slot selection (0-ULP == RTL) */
  uint8_t min01_idx = (hbm->rh_cnt[1] < hbm->rh_cnt[0]) ? 1U : 0U;
  uint16_t min01_val = hbm->rh_cnt[min01_idx];
  uint8_t min23_idx = (hbm->rh_cnt[3] < hbm->rh_cnt[2]) ? 3U : 2U;
  uint16_t min23_val = hbm->rh_cnt[min23_idx];
  uint8_t min_idx = (min23_val < min01_val) ? min23_idx : min01_idx;
  uint16_t min_val = hbm->rh_cnt[min_idx];

  if (min_val == hbm->rh_spill_cnt) {
    hbm->rh_row[min_idx] = row_id;
    uint32_t inc = (uint32_t)hbm->rh_spill_cnt + 1U;
    if (inc >= (uint32_t)eff_thresh) {
      hbm->rh_cnt[min_idx] = hbm->rh_spill_cnt;
      hbm->last_drfm_row = row_id;
      if (hbm->drfm_mitigations < 0xFFFFFFFFU) {
        hbm->drfm_mitigations += 1U;
      }
      return true;
    }
    hbm->rh_cnt[min_idx] = (uint16_t)inc;
    return false;
  }

  /* 4. All 4 slots strictly exceed spill_cnt -> advance spillover bound */
  uint32_t next_spill = (uint32_t)hbm->rh_spill_cnt + 1U;
  if (next_spill >= (uint32_t)eff_thresh) {
    if (out_preventive_ref) {
      *out_preventive_ref = true;
    }
    hbm->last_drfm_row = row_id;
    hbm->rh_valid_mask = 0U;
    hbm->rh_cnt[0] = 0U;
    hbm->rh_cnt[1] = 0U;
    hbm->rh_cnt[2] = 0U;
    hbm->rh_cnt[3] = 0U;
    hbm->rh_spill_cnt = 0U;
    if (hbm->preventive_refreshes < 0xFFFFFFFFU) {
      hbm->preventive_refreshes += 1U;
    }
    return true;
  }
  hbm->rh_spill_cnt = (uint16_t)next_spill;
  return false;
}

uint8_t ele_hbm_step_thermal_10ms(EleHbmStack64 *hbm, int32_t amb_temp_mc,
                                  uint32_t p_io_mw, uint16_t live_bank_mask) {
  if (!hbm) {
    return 1U;
  }
  hbm->live_bank_mask = live_bank_mask;
  uint32_t r_th = (hbm->r_th_mc_per_w > 0U) ? hbm->r_th_mc_per_w : 1150U;
  uint32_t p_bank = (hbm->p_ref_bank_mw > 0U) ? hbm->p_ref_bank_mw : 350U;
  uint32_t clamped_io = (p_io_mw > 200000U) ? 200000U : p_io_mw;
  uint32_t live_banks = ele_popcount32((uint32_t)live_bank_mask & 0xFFFFU);

  uint8_t mult_j = ele_hbm_eval_jedec_mult(hbm->temp_jedec_mc);
  uint8_t mult_s = ele_hbm_eval_jedec_mult(hbm->temp_symp_mc);
  hbm->cur_ref_mult = mult_s;

  uint32_t p_ref_j_mw = 16U * (uint32_t)mult_j * p_bank;
  uint32_t p_ref_s_mw = live_banks * (uint32_t)mult_s * p_bank;

  uint64_t rise_j_mc = ele_div_u64_u32(
      (uint64_t)(clamped_io + p_ref_j_mw) * (uint64_t)r_th, 1000U);
  uint64_t rise_s_mc = ele_div_u64_u32(
      (uint64_t)(clamped_io + p_ref_s_mw) * (uint64_t)r_th, 1000U);

  int64_t eq_j = (int64_t)amb_temp_mc + (int64_t)rise_j_mc;
  int64_t eq_s = (int64_t)amb_temp_mc + (int64_t)rise_s_mc;

  hbm->temp_jedec_mc =
      ele_sat_i64_i32((int64_t)hbm->temp_jedec_mc +
                      ele_asr64(eq_j - (int64_t)hbm->temp_jedec_mc, 2U));
  hbm->temp_symp_mc =
      ele_sat_i64_i32((int64_t)hbm->temp_symp_mc +
                      ele_asr64(eq_s - (int64_t)hbm->temp_symp_mc, 2U));

  if (hbm->temp_jedec_mc >= 95000) {
    hbm->runaway_jedec = 1U;
  }
  if (hbm->temp_symp_mc >= 95000) {
    hbm->runaway_symp = 1U;
  }

  uint32_t pulses_j = 16U * (uint32_t)mult_j * 256U;
  uint32_t pulses_s = live_banks * (uint32_t)mult_s * 256U;
  if (hbm->ref_pulses_jedec <= 0xFFFFFFFFU - pulses_j) {
    hbm->ref_pulses_jedec += pulses_j;
  } else {
    hbm->ref_pulses_jedec = 0xFFFFFFFFU;
  }
  if (hbm->ref_pulses_elided <= 0xFFFFFFFFU - pulses_s) {
    hbm->ref_pulses_elided += pulses_s;
  } else {
    hbm->ref_pulses_elided = 0xFFFFFFFFU;
  }

  /* 10 ms * mW = 0.01 mJ -> accumulate in mJ */
  uint32_t dj_mj = (p_ref_j_mw + 50U) / 100U;
  uint32_t ds_mj = (p_ref_s_mw + 50U) / 100U;
  if (hbm->energy_jedec_mj <= 0xFFFFFFFFU - dj_mj) {
    hbm->energy_jedec_mj += dj_mj;
  } else {
    hbm->energy_jedec_mj = 0xFFFFFFFFU;
  }
  if (hbm->energy_symp_mj <= 0xFFFFFFFFU - ds_mj) {
    hbm->energy_symp_mj += ds_mj;
  } else {
    hbm->energy_symp_mj = 0xFFFFFFFFU;
  }
  return hbm->cur_ref_mult;
}

/* ============================================================================
 * 16. LC Resonant Adiabatic Clock & Crest Tank-Parker (`ele_resonant_clk`)
 * ============================================================================
 */

void ele_resonant_clk64_init(EleResonantClk64 *rclk, uint32_t c_clk_ff,
                             uint16_t v_dd_mv, uint16_t q_factor_milli,
                             uint16_t freq_mhz, uint16_t target_pulse_ps,
                             uint16_t zvs_tol_ps) {
  if (!rclk) {
    return;
  }
  ele_zero64(rclk);
  rclk->c_clk_ff = (c_clk_ff > 0U) ? c_clk_ff : 500000U; /* 500 pF mesh */
  rclk->v_dd_mv = (v_dd_mv > 0U) ? v_dd_mv : 750U;
  rclk->q_factor_milli = (q_factor_milli > 0U) ? q_factor_milli : 12000U;
  rclk->freq_mhz = (freq_mhz > 0U) ? freq_mhz : 2000U;
  rclk->target_pulse_ps = (target_pulse_ps >= 40U && target_pulse_ps <= 2000U)
                              ? target_pulse_ps
                              : 120U;
  rclk->sustain_pulse_ps = rclk->target_pulse_ps;
  rclk->zvs_tol_ps = (zvs_tol_ps > 0U) ? zvs_tol_ps : 25U;
  rclk->r_switch_mohm = 50U;
}

uint16_t ele_resonant_clk_step(EleResonantClk64 *rclk, bool clk_enable,
                               int16_t zvs_err_ps) {
  if (!rclk) {
    return 0U;
  }
  uint32_t abs_err = ele_abs_i32_u32((int32_t)zvs_err_ps);
  bool in_tol = (abs_err <= (uint32_t)rclk->zvs_tol_ps);

  if (!clk_enable) {
    if (rclk->state == (uint8_t)ELE_RCLK_OSCILLATING) {
      rclk->park_cap_mv = rclk->tank_env_mv;
      rclk->park_ready = 1U;
    }
    rclk->tank_env_mv = 0U;
    rclk->sustain_pulse_ps = 0U;
    rclk->state = (uint8_t)ELE_RCLK_PARKED;
    rclk->zvs_locked = 0U;
    if (rclk->parked_cycles < 0xFFFFFFFFU) {
      rclk->parked_cycles += 1U;
    }
    return 0U;
  }

  if (rclk->state == (uint8_t)ELE_RCLK_PARKED) {
    rclk->tank_env_mv =
        (rclk->park_ready != 0U) ? rclk->park_cap_mv : rclk->v_dd_mv;
    uint16_t tp = rclk->target_pulse_ps;
    rclk->sustain_pulse_ps = (tp < 40U) ? 40U : ((tp > 2000U) ? 2000U : tp);
    if (rclk->park_ready != 0U && rclk->restart_events < 65535U) {
      rclk->restart_events += 1U;
    }
    rclk->state = (uint8_t)ELE_RCLK_OSCILLATING;
    rclk->park_ready = 0U;
    rclk->primed = 1U;
    rclk->phase_err_ps = zvs_err_ps;
    rclk->prev_phase_err_ps = zvs_err_ps;
    rclk->zvs_locked = in_tol ? 1U : 0U;
  } else {
    int64_t err = (int64_t)zvs_err_ps;
    int64_t prev =
        (rclk->primed != 0U) ? (int64_t)rclk->prev_phase_err_ps : err;
    int64_t d_err = err - prev;
    int64_t adj = ele_asr64(err, 2U) + ele_asr64(d_err, 3U);
    int64_t cand = (int64_t)rclk->sustain_pulse_ps - adj;
    if (cand < 40LL) {
      rclk->sustain_pulse_ps = 40U;
    } else if (cand > 2000LL) {
      rclk->sustain_pulse_ps = 2000U;
    } else {
      rclk->sustain_pulse_ps = (uint16_t)cand;
    }
    rclk->tank_env_mv = rclk->v_dd_mv;
    rclk->primed = 1U;
    rclk->phase_err_ps = zvs_err_ps;
    rclk->prev_phase_err_ps = zvs_err_ps;
    rclk->zvs_locked = in_tol ? 1U : 0U;
  }

  if (rclk->total_cycles < 0xFFFFFFFFU) {
    rclk->total_cycles += 1U;
  }
  if (rclk->zvs_locked != 0U && rclk->zvs_lock_cycles < 0xFFFFFFFFU) {
    rclk->zvs_lock_cycles += 1U;
  }

  /* Dynamic clock energy (pJ): E_cmos = C_clk(fF) * V_dd(mV)^2 / 10^9 */
  uint32_t c_ff = (rclk->c_clk_ff > 0U) ? rclk->c_clk_ff : 500000U;
  uint32_t v_mv = (rclk->v_dd_mv > 0U) ? rclk->v_dd_mv : 750U;
  uint32_t q_milli =
      (rclk->q_factor_milli > 0U) ? rclk->q_factor_milli : 12000U;
  uint64_t cv2_fj = ele_div_u64_u32(
      (uint64_t)c_ff * (uint64_t)v_mv * (uint64_t)v_mv, 1000000U);
  uint64_t e_cmos_pj = ele_div_u64_u32(cv2_fj + 500ULL, 1000U);
  if (e_cmos_pj == 0ULL) {
    e_cmos_pj = 1ULL;
  }
  /* LC tank dissipates (pi / Q) * E_cmos at ZVS; non-ZVS adds phase-error loss
   */
  uint64_t pi_eff =
      (rclk->zvs_locked != 0U) ? 3142ULL : (3142ULL + (uint64_t)(abs_err * 8U));
  uint64_t e_res_pj =
      ele_div_u64_u32(e_cmos_pj * pi_eff + (uint64_t)(q_milli - 1U), q_milli);

  if (rclk->energy_cmos_pj <= 0xFFFFFFFFFFFFFFFFULL - e_cmos_pj) {
    rclk->energy_cmos_pj += e_cmos_pj;
  } else {
    rclk->energy_cmos_pj = 0xFFFFFFFFFFFFFFFFULL;
  }
  if (rclk->energy_resonant_pj <= 0xFFFFFFFFFFFFFFFFULL - e_res_pj) {
    rclk->energy_resonant_pj += e_res_pj;
  } else {
    rclk->energy_resonant_pj = 0xFFFFFFFFFFFFFFFFULL;
  }
  return rclk->sustain_pulse_ps;
}

/* ============================================================================
 * 17. Asynchronous Level-Crossing Event ADC + Vernier TDC (`ele_event_adc`)
 * ============================================================================
 */

void ele_event_adc64_init(EleEventAdc64 *adc, uint32_t lsb_step_uv,
                          uint32_t hyst_uv, uint32_t coarse_clk_ps,
                          uint16_t t_slow_delay_ps, uint16_t t_fast_delay_ps,
                          uint16_t e_sar_conv_fj, uint16_t e_event_pulse_fj) {
  if (!adc) {
    return;
  }
  ele_zero64(adc);
  adc->lsb_step_uv = (lsb_step_uv > 0U) ? lsb_step_uv : 5000U;
  adc->hyst_uv = hyst_uv;
  adc->coarse_clk_ps = (coarse_clk_ps > 0U) ? coarse_clk_ps : 1000U;
  adc->t_slow_delay_ps = (t_slow_delay_ps > 0U) ? t_slow_delay_ps : 45U;
  adc->t_fast_delay_ps =
      (t_fast_delay_ps > 0U && t_fast_delay_ps < adc->t_slow_delay_ps)
          ? t_fast_delay_ps
          : (uint16_t)(adc->t_slow_delay_ps - 1U);
  adc->vernier_res_ps = (uint16_t)(adc->t_slow_delay_ps - adc->t_fast_delay_ps);
  adc->e_sar_conv_fj = (e_sar_conv_fj > 0U) ? e_sar_conv_fj : 800U;
  adc->e_event_pulse_fj = (e_event_pulse_fj > 0U) ? e_event_pulse_fj : 250U;
}

uint16_t ele_vernier_tdc_eval_ps(uint16_t lead_ps, uint16_t t_slow_ps,
                                 uint16_t t_fast_ps, uint16_t max_taps,
                                 uint16_t *out_taps, bool *out_overflow) {
  uint16_t res_ps =
      (t_slow_ps > t_fast_ps) ? (uint16_t)(t_slow_ps - t_fast_ps) : 1U;
  uint16_t cap_taps = (max_taps > 0U) ? max_taps : 63U;
  uint32_t raw_taps = (uint32_t)lead_ps / (uint32_t)res_ps;
  bool ovf = (raw_taps > (uint32_t)cap_taps);
  uint16_t taps = ovf ? cap_taps : (uint16_t)raw_taps;
  if (out_taps) {
    *out_taps = taps;
  }
  if (out_overflow) {
    *out_overflow = ovf;
  }
  uint32_t fine_ps = (uint32_t)taps * (uint32_t)res_ps;
  return (fine_ps > 65535U) ? 65535U : (uint16_t)fine_ps;
}

int8_t ele_event_adc_step(EleEventAdc64 *adc, int32_t v_in_uv,
                          uint16_t crossing_lead_ps) {
  if (!adc) {
    return 0;
  }
  uint32_t eff_lsb = (adc->lsb_step_uv == 0U) ? 1U : adc->lsb_step_uv;
  uint64_t thresh64 = (uint64_t)eff_lsb + (uint64_t)adc->hyst_uv;

  if (adc->nyquist_samples < 0xFFFFFFFFU) {
    adc->nyquist_samples += 1U;
  }
  uint32_t d_nyq_pj = ((uint32_t)adc->e_sar_conv_fj + 5U) / 10U;
  if (d_nyq_pj == 0U && adc->e_sar_conv_fj > 0U) {
    d_nyq_pj = 1U;
  }
  if (adc->energy_nyquist_pj <= 0xFFFFFFFFU - d_nyq_pj) {
    adc->energy_nyquist_pj += d_nyq_pj;
  } else {
    adc->energy_nyquist_pj = 0xFFFFFFFFU;
  }

  if (adc->primed == 0U) {
    adc->primed = 1U;
    adc->dac_ref_uv = v_in_uv;
    adc->recon_sig_uv = v_in_uv;
    adc->last_dir = 0;
    adc->event_fired = 0U;
    adc->last_vernier_steps = 0U;
    adc->tdc_overflow = 0U;
    if (adc->quiet_ticks < 65535U) {
      adc->quiet_ticks += 1U;
    }
    return 0;
  }

  int64_t diff64 = (int64_t)v_in_uv - (int64_t)adc->dac_ref_uv;
  uint64_t abs_diff64 =
      (diff64 < 0LL) ? (0ULL - (uint64_t)diff64) : (uint64_t)diff64;

  int8_t dir = 0;
  if (diff64 >= 0LL && abs_diff64 >= thresh64) {
    dir = 1;
    adc->dac_ref_uv =
        ele_sat_i64_i32((int64_t)adc->dac_ref_uv + (int64_t)eff_lsb);
  } else if (diff64 < 0LL && abs_diff64 >= thresh64) {
    dir = -1;
    adc->dac_ref_uv =
        ele_sat_i64_i32((int64_t)adc->dac_ref_uv - (int64_t)eff_lsb);
  } else {
    if (abs_diff64 >= (uint64_t)eff_lsb &&
        adc->chatter_suppressed < 0xFFFFFFFFU) {
      adc->chatter_suppressed += 1U;
    }
  }

  if (dir != 0) {
    adc->recon_sig_uv = adc->dac_ref_uv;
    adc->last_dir = dir;
    adc->event_fired = 1U;
    adc->quiet_ticks = 0U;
    if (adc->event_samples < 0xFFFFFFFFU) {
      adc->event_samples += 1U;
    }

    uint16_t taps = 0U;
    bool ovf = false;
    uint16_t fine_ps =
        ele_vernier_tdc_eval_ps(crossing_lead_ps, adc->t_slow_delay_ps,
                                adc->t_fast_delay_ps, 63U, &taps, &ovf);
    adc->last_vernier_steps = taps;
    adc->tdc_overflow = ovf ? 1U : 0U;
    uint64_t t_coarse =
        (uint64_t)adc->nyquist_samples * (uint64_t)adc->coarse_clk_ps;
    uint64_t t_evt =
        (t_coarse > (uint64_t)fine_ps) ? (t_coarse - (uint64_t)fine_ps) : 0ULL;
    adc->last_event_time_ps =
        (t_evt > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)t_evt;

    uint32_t d_evt_pj = ((uint32_t)adc->e_event_pulse_fj + 5U) / 10U;
    if (d_evt_pj == 0U && adc->e_event_pulse_fj > 0U) {
      d_evt_pj = 1U;
    }
    if (adc->energy_event_pj <= 0xFFFFFFFFU - d_evt_pj) {
      adc->energy_event_pj += d_evt_pj;
    } else {
      adc->energy_event_pj = 0xFFFFFFFFU;
    }
  } else {
    adc->last_dir = 0;
    adc->event_fired = 0U;
    if (adc->quiet_ticks < 65535U) {
      adc->quiet_ticks += 1U;
    }
  }

  int64_t err64 = (int64_t)v_in_uv - (int64_t)adc->recon_sig_uv;
  uint64_t abs_err = (err64 < 0LL) ? (0ULL - (uint64_t)err64) : (uint64_t)err64;
  uint16_t err16 = (abs_err > 65535ULL) ? 65535U : (uint16_t)abs_err;
  if (err16 > adc->peak_recon_err_uv) {
    adc->peak_recon_err_uv = err16;
  }
  return dir;
}

/* ============================================================================
 * 18. BSPDN Active Resonance Damper & Spread-Spectrum (`ele_pdn_emi`)
 * ============================================================================
 */

void ele_pdn_emi64_init(ElePdnEmi64 *emi, uint16_t nom_period_ps,
                        uint8_t max_dither_ps, uint8_t damping_gain_q8,
                        uint16_t seed) {
  if (!emi) {
    return;
  }
  ele_zero64(emi);
  emi->nom_period_ps = (nom_period_ps >= 20U) ? nom_period_ps : 500U;
  emi->max_dither_ps = (max_dither_ps > 127U) ? 127U : max_dither_ps;
  emi->damping_gain_q8 = (damping_gain_q8 > 0U) ? damping_gain_q8 : 64U;
  emi->lfsr_state = (seed != 0U) ? seed : 0xACE1U;
}

uint16_t ele_pdn_emi_step_cycle(ElePdnEmi64 *emi, int16_t di_excite_ma) {
  if (!emi) {
    return 0U;
  }
  uint16_t cur_lfsr = (emi->lfsr_state != 0U) ? emi->lfsr_state : 0xACE1U;
  uint16_t next_lfsr =
      (uint16_t)((cur_lfsr >> 1U) ^ (((cur_lfsr & 1U) != 0U) ? 0xB400U : 0U));

  uint16_t eff_nom = (emi->nom_period_ps >= 20U) ? emi->nom_period_ps : 500U;
  uint16_t head_lo = (uint16_t)(eff_nom - 20U);
  uint16_t head_hi = (uint16_t)(65535U - eff_nom);
  uint16_t sym_cap = (head_lo < head_hi) ? head_lo : head_hi;
  uint8_t cap_dither = (emi->max_dither_ps > 127U) ? 127U : emi->max_dither_ps;
  uint8_t eff_dither =
      ((uint16_t)cap_dither > sym_cap) ? (uint8_t)sym_cap : cap_dither;
  uint32_t raw_prod =
      ((uint32_t)(cur_lfsr & 0x7FU) * (uint32_t)eff_dither) >> 7U;
  uint8_t scaled_mag = (raw_prod > 127U) ? 127U : (uint8_t)raw_prod;
  int16_t even_delta =
      ((cur_lfsr & 0x80U) != 0U) ? -(int16_t)scaled_mag : (int16_t)scaled_mag;
  int16_t cur_delta =
      (emi->pair_phase != 0U) ? (int16_t)emi->held_delta_ps : even_delta;

  int32_t cand_period = (int32_t)eff_nom + (int32_t)cur_delta;
  uint16_t mod_period =
      (cand_period < 20)
          ? 20U
          : ((cand_period > 65535) ? 65535U : (uint16_t)cand_period);

  uint8_t spec_bin =
      (uint8_t)((cur_lfsr & 0x07U) ^ (uint16_t)(emi->pair_phase & 1U));
  if (emi->bin_hits[spec_bin] < 65535U) {
    emi->bin_hits[spec_bin] += 1U;
  }

  if (emi->pair_phase == 0U) {
    emi->held_delta_ps = (int8_t)(-even_delta);
    emi->pair_phase = 1U;
    emi->lfsr_state = next_lfsr;
  } else {
    emi->pair_phase = 0U;
    emi->lfsr_state = cur_lfsr;
  }

  emi->sum_period_base_ps += (uint64_t)eff_nom;
  emi->sum_period_ss_ps += (uint64_t)mod_period;

  /* Symplectic Semi-Implicit Euler BSPDN Coupled LC Resonator (0-ULP == RTL) */
  int64_t vu = (int64_t)emi->v_osc_undamped_uv;
  int64_t iu = (int64_t)emi->i_osc_undamped_ma;
  int64_t vd = (int64_t)emi->v_osc_damped_uv;
  int64_t id = (int64_t)emi->i_osc_damped_ma;
  int64_t exc = (int64_t)di_excite_ma;

  int64_t vu_next = vu - ele_asr64(vu, 6U) + (iu * 4LL) + (exc * 8LL);
  int64_t iu_next = iu - ele_asr64(vu_next, 2U);

  int64_t damp_term = ele_asr64(vd * (int64_t)emi->damping_gain_q8, 8U);
  int64_t vd_next =
      vd - ele_asr64(vd, 6U) - damp_term + (id * 4LL) + (exc * 8LL);
  int64_t id_next = id - ele_asr64(vd_next, 2U);

  emi->v_osc_undamped_uv = ele_sat_i64_i32(vu_next);
  emi->i_osc_undamped_ma = ele_sat_i64_i32(iu_next);
  emi->v_osc_damped_uv = ele_sat_i64_i32(vd_next);
  emi->i_osc_damped_ma = ele_sat_i64_i32(id_next);

  uint32_t abs_vu = ele_abs_i32_u32(emi->v_osc_undamped_uv);
  uint32_t abs_vd = ele_abs_i32_u32(emi->v_osc_damped_uv);
  if (abs_vu > emi->peak_v_undamped_uv) {
    emi->peak_v_undamped_uv = abs_vu;
  }
  if (abs_vd > emi->peak_v_damped_uv) {
    emi->peak_v_damped_uv = abs_vd;
  }

  return mod_period;
}
