#ifndef ELE_HAL_H
#define ELE_HAL_H

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 0: Cross-MCU Bare-Metal Silicon Port, Pinmux & Power-Domain HAL
 * Targets:
 *   - ESP32-C6 (RISC-V RV32IMAC @ 160 MHz HP / 20 MHz LP)
 *   - ESP32-S3 (Dual LX7 @ 240 MHz HP / ULP RISC-V @ 17.5 MHz)
 *   - RP2350   (Dual Hazard3 RV32IMAC / Cortex-M33 @ 150 MHz, SIO GPIO)
 *   - STM32U5  (Cortex-M33 Low-Power @ 160 MHz, BSRR Atomic GPIO)
 *   - Host Deterministic Shadow (ARM64 / x86_64 wind-tunnel verification)
 * Invariants:
 *   - FloatOps == 0, HeapCalls == 0, 0 vtables / function pointers, 64B Aligned
 * ============================================================================
 */

#define ELE_IO_MUX_DRV_SHIFT 10U
#define ELE_IO_MUX_DRV_MASK 0x3U

typedef enum {
  ELE_MCU_HOST_SHADOW = 0,
  ELE_MCU_ESP32_C6_RV32 = 1,
  ELE_MCU_ESP32_S3_ULP = 2,
  ELE_MCU_RP2350_RV32 = 3,
  ELE_MCU_STM32_CM33 = 4
} EleMcuArch;

/* Hardware Pad Drive Strength Levels (Impedance Matched to Waveguide Z0) */
typedef enum {
  ELE_DRV_5MA = 0,  /* ~5 mA (2mA..5mA class): High Z0 / short trace / 0 ring */
  ELE_DRV_10MA = 1, /* ~10 mA: Twisted-pair ~100 Ohm optimal match */
  ELE_DRV_20MA = 2, /* ~20 mA: Standard CMOS load */
  ELE_DRV_40MA = 3  /* ~40 mA: Heavy capacitive load */
} EleGpioDriveStrength;

/* Power-Domain States for Dual-Domain / Tickless Embedded Silicon */
typedef enum {
  ELE_POWER_DEEP_SLEEP_LP_ONLY = 0, /* HP gated off, LP timer/bus gate active */
  ELE_POWER_LP_SENSE_GATE = 1,      /* LP peripheral burst check */
  ELE_POWER_HP_ACTIVE_BURST = 2     /* Full HP core active burst */
} ElePowerState;

/* 64-Byte Cache-Line Univalent Cross-MCU Bare-Metal Silicon Port Descriptor */
typedef struct __attribute__((aligned(64))) {
  uint32_t gpio_base_addr;    /* [0..3]   MMIO GPIO peripheral base address */
  uint32_t iomux_base_addr;   /* [4..7]   MMIO Pinmux / pad control base */
  uint16_t w1ts_set_off;      /* [8..9]   Atomic 1-cycle pin SET offset */
  uint16_t w1tc_clr_off;      /* [10..11] Atomic 1-cycle pin CLEAR offset */
  uint16_t gpio_in_off;       /* [12..13] GPIO input sample offset */
  uint16_t mcu_arch_id;       /* [14..15] EleMcuArch silicon profile ID */
  uint32_t hp_cpu_hz;         /* [16..19] High-performance clock (Hz) */
  uint32_t lp_cpu_hz;         /* [20..23] Low-power tickless sleep clock (Hz) */
  uint32_t lp_sleep_power_uw; /* [24..27] Deep-sleep domain power (uW) */
  uint32_t hp_active_power_uw; /* [28..31] Active HP domain power (uW) */
  uint32_t gpio_shadow_out;    /* [32..35] Deterministic GPIO output image */
  uint32_t iomux_shadow_reg;   /* [36..39] Deterministic Pinmux drive image */
  uint32_t cycle_counter;      /* [40..43] Monotonic cycle counter */
  uint32_t bus_tx_count;       /* [44..47] Physical I2C/SPI/MMIO transactions */
  uint64_t sleep_us_accum;     /* [48..55] Cumulative tickless LP sleep (us) */
  int64_t
      energy_saved_uj_q16; /* [56..63] Sleep energy saved vs active poll (uJ) */
} EleHalPort64;

_Static_assert(sizeof(EleHalPort64) == 64U,
               "EleHalPort64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleHalPort64) == 64U,
               "EleHalPort64 must be 64-byte aligned");

/* Initialize canonical bare-metal MMIO & power profile for target MCU */
void ele_hal_port64_init(EleHalPort64 *port, EleMcuArch arch);

/* Record non-blocking tickless LP sleep window and accumulate saved energy */
void ele_hal_record_sleep_us(EleHalPort64 *port, uint32_t sleep_us);

/* Read hardware cycle counter (RISC-V mcycle) or deterministic port counter */
static inline uint32_t ele_hal_read_cycles32(uint32_t *sim_cycle_counter) {
#if defined(__riscv)
  uint32_t cyc = 0U;
  __asm__ volatile("csrr %0, mcycle" : "=r"(cyc));
  if (sim_cycle_counter) {
    *sim_cycle_counter = cyc;
  }
  return cyc;
#else
  if (sim_cycle_counter) {
    *sim_cycle_counter += 1U;
    return *sim_cycle_counter;
  }
  return 0U;
#endif
}

/* Single-cycle atomic GPIO set/clear via port MMIO offsets or host shadow */
static inline void ele_hal_port_gpio_write(EleHalPort64 *port, uint8_t pin,
                                           uint8_t level) {
  if (!port) {
    return;
  }
  uint32_t mask = (1UL << (pin & 31U));
#if defined(__riscv) && defined(ELE_BARE_METAL_MMIO)
  if (port->gpio_base_addr != 0U) {
    uint32_t off =
        level ? (uint32_t)port->w1ts_set_off : (uint32_t)port->w1tc_clr_off;
    volatile uint32_t *reg =
        (volatile uint32_t *)(uintptr_t)(port->gpio_base_addr + off);
    *reg = mask;
  }
#endif
  if (level) {
    port->gpio_shadow_out |= mask;
  } else {
    port->gpio_shadow_out &= ~mask;
  }
}

/* Direct shadow helper for standalone link descriptors */
static inline void ele_hal_gpio_write(uint32_t *gpio_shadow_reg, uint8_t pin,
                                      uint8_t level) {
  uint32_t mask = (1UL << (pin & 31U));
#if defined(__riscv) && defined(ELE_BARE_METAL_MMIO)
  volatile uint32_t *reg =
      (volatile uint32_t *)(uintptr_t)(0x60091000UL +
                                       (level ? 0x0008UL : 0x000CUL));
  *reg = mask;
#endif
  if (gpio_shadow_reg) {
    if (level) {
      *gpio_shadow_reg |= mask;
    } else {
      *gpio_shadow_reg &= ~mask;
    }
  }
}

/* Encode hardware pad drive strength bits (0..3 -> 5mA..40mA) */
static inline uint32_t ele_hal_gpio_encode_iomux_drv(uint32_t prev_iomux,
                                                     EleGpioDriveStrength drv) {
  uint32_t clean = prev_iomux & ~(ELE_IO_MUX_DRV_MASK << ELE_IO_MUX_DRV_SHIFT);
  return clean |
         (((uint32_t)drv & ELE_IO_MUX_DRV_MASK) << ELE_IO_MUX_DRV_SHIFT);
}

#ifdef __cplusplus
}
#endif

#endif /* ELE_HAL_H */
