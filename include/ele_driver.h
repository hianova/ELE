#ifndef ELE_DRIVER_H
#define ELE_DRIVER_H

#include "ele_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * ELE Module 1: 64-Byte Zero-Code Datasheet Compiler, Bus Micro-Sequencer,
 *               Hardware CRC-8 Sieve, 1-Cycle Hot-Swap & Tickless Bus Governor
 * Principles:
 *   1. Complete Datasheet-to-Descriptor Abstraction:
 *      - EleBusSeq64 encodes multi-step I2C/SPI/MMIO initialization, bitmasked
 *        Read-Modify-Write (preserving reserved IC bits), trigger-conversion,
 *        non-blocking LP sleep waits (0 delay_ms spin), bounded DRDY polling,
 *        and burst reads in 4 x 64-bit register-packed micro-ops.
 *      - EleDriverCodec64 decodes raw frames (endianness, bit-shift, bit-width,
 *        2's complement, hardware-free CRC-8 sieve, Q16.16 affine calibration)
 *        into live 2-Jet state (x, dx/dt, d2x/dt2), and inversely encodes
 *        clamped actuator register commands.
 *   2. Tickless 2-Jet Energy-Proportional Bus Governor:
 *      - Automatically scales peripheral polling period between min_period_us
 *        (e.g. 100 Hz active) and max_period_us (e.g. 2 Hz static sleep) based
 *        on live 2-Jet derivatives |v| + |a|, snapping back in 1 cycle on any
 *        physical slew surprise.
 *   3. Zero-Reboot 1-Cycle Atomic Hot-Swap, Slew Rollback & 9-Clock Bus Heal:
 *      - Verified by Goldilocks F_p seals; rolls back faulty updates in 1 cycle
 *        and coasts on 2-Jet prediction across stuck-SDA / CRC bus faults.
 * ============================================================================
 */

enum {
  ELE_CODEC_LITTLE_ENDIAN = 0x00U,
  ELE_CODEC_BIG_ENDIAN = (1U << 0),
  ELE_CODEC_SIGNED_2C = (1U << 1),
  ELE_CODEC_HOTSWAP_PENDING = (1U << 2),
  ELE_CODEC_INITIALIZED = (1U << 3),
  ELE_CODEC_CRC8_POLY31 = (1U << 4),  /* Sensirion/HTU21D/SHT3x poly 0x31 */
  ELE_CODEC_CRC8_SMBUS07 = (1U << 5), /* SMBus/PMBus PEC poly 0x07 */
  ELE_CODEC_ACTUATOR_CLAMP = (1U << 6)
};

#define ELE_CODEC_INVARIANT_MASK                                               \
  (ELE_CODEC_BIG_ENDIAN | ELE_CODEC_SIGNED_2C | ELE_CODEC_CRC8_POLY31 |        \
   ELE_CODEC_CRC8_SMBUS07 | ELE_CODEC_ACTUATOR_CLAMP)

enum {
  ELE_BUS_HEALTHY = 0U,
  ELE_BUS_COASTING_2JET = (1U << 0),
  ELE_BUS_9CLK_RECOVERED = (1U << 1),
  ELE_BUS_ROLLED_BACK = (1U << 2),
  ELE_BUS_TAMPER_REJECTED = (1U << 3),
  ELE_BUS_CRC_REJECTED = (1U << 4)
};

typedef enum {
  ELE_BUS_PROTO_I2C = 0,
  ELE_BUS_PROTO_SPI = 1,
  ELE_BUS_PROTO_MMIO = 2
} EleBusProto;

typedef enum {
  ELE_OP_END = 0,
  ELE_OP_WRITE_REG8 = 1, /* reg[addr] = imm_val */
  ELE_OP_RMW_BITS8 = 2,  /* reg[addr] = (reg[addr] & ~mask) | (imm & mask) */
  ELE_OP_CMD_ONLY = 3,   /* issue command byte + optional non-blocking wait */
  ELE_OP_WAIT_US = 4,    /* non-blocking LP sleep wait (10 us units) */
  ELE_OP_POLL_MASK = 5,  /* bounded DRDY poll: (reg[addr] & mask) == imm */
  ELE_OP_BURST_READ = 6  /* burst read [1..16] bytes from reg[addr] */
} EleBusOpcode;

typedef enum {
  ELE_SEQ_SLEEP_TICKLESS = 0, /* CPU & bus asleep in LP domain */
  ELE_SEQ_STEP_ISSUED = 1,    /* Init/Config/Trigger micro-op executed */
  ELE_SEQ_DATA_READY = 2,     /* Burst read completed & 2-Jet updated */
  ELE_SEQ_FAULT_COASTING = 3  /* Bus/CRC/timeout fault -> 2-Jet coasting */
} EleSeqVerdict;

/* Pack a declarative datasheet bus micro-op into a single 64-bit word:
 *   [63:56] opcode | [55:40] reg_addr | [39:32] mask | [31:24] imm_val
 *   [23:16] burst_len | [15:0] wait_ticks_10us (0..65535 -> up to 655.35 ms)
 */
static inline uint64_t ele_bus_op_pack(EleBusOpcode op, uint16_t reg_addr,
                                       uint8_t mask, uint8_t imm_val,
                                       uint8_t burst_len,
                                       uint16_t wait_ticks_10us) {
  return ((uint64_t)(op & 0xFFU) << 56U) | ((uint64_t)reg_addr << 40U) |
         ((uint64_t)mask << 32U) | ((uint64_t)imm_val << 24U) |
         ((uint64_t)burst_len << 16U) | (uint64_t)wait_ticks_10us;
}

/* 64-Byte Cache-Line Univalent Declarative Driver & Hot-Swap Descriptor */
typedef struct __attribute__((aligned(64))) {
  uint64_t driver_seal_fp64;   /* [0..7]   Goldilocks F_p manifest seal */
  uint64_t shadow_backup_word; /* [8..15]  Packed previous-good (scale, offset)
                                  for 1-cycle rollback */
  int32_t scale_q16;           /* [16..19] Affine transfer gain in Q16.16 */
  int32_t offset_q16;          /* [20..23] Affine transfer bias in Q16.16 */
  int32_t max_slew_jump_q16;   /* [24..27] Max physical 1-tick jump before
                                  rollback */
  int32_t jet_pos_q16;         /* [28..31] Calibrated 0-Jet physical state x */
  int32_t jet_vel_q16;         /* [32..35] Calibrated 1-Jet velocity dx/dt */
  int32_t jet_acc_q16;      /* [36..39] Calibrated 2-Jet acceleration d2x/dt2 */
  uint32_t driver_version;  /* [40..43] Active monotonic driver revision */
  uint32_t shadow_version;  /* [44..47] Previous known-good shadow revision */
  uint16_t bus_dev_addr;    /* [48..49] I2C / SPI / MMIO peripheral address */
  uint16_t reg_addr;        /* [50..51] Hardware register offset */
  uint8_t bit_shift;        /* [52]     Right-shift bits to strip status/CRC */
  uint8_t bit_width;        /* [53]     Effective bit width (1..32) */
  uint8_t codec_flags;      /* [54]     Endian, sign, CRC8 & hot-swap flags */
  uint8_t bus_health_state; /* [55]     Health, CRC & rollback status */
  uint32_t recovery_pulses_sent; /* [56..59] Cumulative 9-clock heal pulses */
  uint32_t hot_swap_rollbacks;   /* [60..63] Cumulative 1-cycle rollbacks */
} EleDriverCodec64;

_Static_assert(sizeof(EleDriverCodec64) == 64U,
               "EleDriverCodec64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleDriverCodec64) == 64U,
               "EleDriverCodec64 must be 64-byte aligned");

/* 64-Byte Cache-Line Univalent Bus Sequencer & Tickless 2-Jet Governor */
typedef struct __attribute__((aligned(64))) {
  uint64_t micro_ops[4];       /* [0..31]  4 x 64b datasheet bus micro-ops */
  uint64_t seq_seal_fp64;      /* [32..39] Goldilocks F_p micro-program seal */
  uint32_t wake_countdown_us;  /* [40..43] Non-blocking sleep countdown (us) */
  uint32_t current_period_us;  /* [44..47] Adaptive 2-Jet poll period (us) */
  uint32_t min_period_us;      /* [48..51] Fast active period (e.g. 10000 us) */
  uint32_t max_period_us;      /* [52..55] Static sleep period (e.g. 500000) */
  uint32_t suppressed_wakeups; /* [56..59] Bus wakeups saved by 2-Jet sleep */
  uint8_t bus_proto;           /* [60]     EleBusProto (I2C, SPI, MMIO) */
  uint8_t dev_addr;            /* [61]     7-bit I2C addr or SPI CS pin */
  uint8_t pc_state;            /* [62]     Bits [1:0]=PC, [5:2]=loop_pc,
                                           [7]=init_done */
  uint8_t last_verdict;        /* [63]     EleSeqVerdict status */
} EleBusSeq64;

_Static_assert(sizeof(EleBusSeq64) == 64U,
               "EleBusSeq64 must be exactly 64 bytes (1 cache line)");
_Static_assert(_Alignof(EleBusSeq64) == 64U,
               "EleBusSeq64 must be 64-byte aligned");

/* Hardware-free CRC-8 calculator (e.g. poly=0x31 Sensirion, poly=0x07 SMBus) */
uint8_t ele_crc8_compute(const uint8_t *data, uint8_t len, uint8_t poly,
                         uint8_t init_val);

/* Safe datasheet register Read-Modify-Write preserving unmasked reserved bits
 */
static inline uint8_t ele_reg_rmw8(uint8_t old_reg, uint8_t mask,
                                   uint8_t new_bits) {
  return (uint8_t)((old_reg & (uint8_t)~mask) | (new_bits & mask));
}

/* Compute Goldilocks F_p cryptographic seal over declarative driver parameters
 */
uint64_t ele_driver_seal_fp64(uint32_t version, uint16_t bus_dev_addr,
                              uint16_t reg_addr, int32_t scale_q16,
                              int32_t offset_q16, uint8_t bit_shift,
                              uint8_t bit_width, uint8_t codec_flags);

/* Initialize a 64B zero-code declarative driver descriptor */
void ele_driver_codec64_init(EleDriverCodec64 *drv, uint32_t version,
                             uint16_t bus_dev_addr, uint16_t reg_addr,
                             uint8_t bit_shift, uint8_t bit_width,
                             uint8_t codec_flags, int32_t scale_q16,
                             int32_t offset_q16, int32_t max_slew_jump_q16);

/* Zero-Reboot 1-Cycle Atomic Driver Hot-Swap */
bool ele_driver_hotswap_atomic64(EleDriverCodec64 *drv, uint32_t new_version,
                                 int32_t new_scale_q16, int32_t new_offset_q16,
                                 uint64_t expected_seal_fp64);

/* Universal Frame Decode, CRC-8 Sieve, 9-Clock Bus Heal & 1-Cycle Auto-Rollback
 */
int32_t ele_driver_decode_and_heal_step(EleDriverCodec64 *drv,
                                        const uint8_t raw_bytes[4],
                                        bool bus_transfer_ok);

/* Decode multi-axis contiguous burst buffer (e.g. 6-byte 3-axis IMU / VI) */
void ele_driver_decode_burst_axes(EleDriverCodec64 *axes, uint8_t num_axes,
                                  const uint8_t *burst_buf,
                                  uint8_t stride_bytes, bool bus_transfer_ok,
                                  int32_t *out_q16);

/* Bidirectional Actuator Encoder: Q16.16 setpoint -> clamped raw register bytes
 */
uint32_t ele_driver_encode_actuator_raw(const EleDriverCodec64 *drv,
                                        int32_t target_q16,
                                        uint8_t out_bytes[4]);

/* Initialize 64B declarative bus micro-sequencer & Tickless 2-Jet Governor */
void ele_bus_seq64_init(EleBusSeq64 *seq, EleBusProto proto, uint8_t dev_addr,
                        uint8_t loop_start_pc, uint32_t min_period_us,
                        uint32_t max_period_us, const uint64_t ops[4]);

/* Tickless 2-Jet Energy-Proportional Polling Governor:
 * Scales period exponentially toward max_period_us when |v| + |a| <= eps_q16,
 * or snaps back to min_period_us in 1 cycle on physical slew surprise. */
uint32_t ele_bus_govern_period_jet2(EleBusSeq64 *seq,
                                    const EleDriverCodec64 *drv,
                                    int32_t quiet_eps_q16);

/* Execute 1 non-blocking tick of the 64B bus micro-sequencer + codec + HAL */
EleSeqVerdict ele_bus_seq_step(EleBusSeq64 *seq, EleDriverCodec64 *drv,
                               EleHalPort64 *port, uint8_t periph_regs[256],
                               uint32_t dt_us, int32_t quiet_eps_q16,
                               bool bus_link_ok);

/* Zero-Code Hardware Self-Calorimetry Preset for TI INA226 Power Monitor:
 *   - shunt_mv_drv: Reg 0x01 (Signed 16b BE, 2.5 uV/LSB -> Q16.16 mV)
 *   - bus_v_drv:    Reg 0x02 (Unsigned 16b BE, 1.25 mV/LSB -> Q16.16 V)
 *   - seq:          Configures averaging, polls CVRF (0x06 & 0x08), burst reads
 */
void ele_driver_init_ina226_calorimeter(EleDriverCodec64 *shunt_mv_drv,
                                        EleDriverCodec64 *bus_v_drv,
                                        EleBusSeq64 *seq, uint8_t i2c_addr);

/* Evaluate instantaneous rail power in microwatts (uW) from calibrated INA226
 * Q16.16 shunt millivolts, Q16.16 bus volts, and shunt resistance (mOhm) */
uint32_t ele_driver_eval_ina226_power_uw(int32_t shunt_mv_q16,
                                         int32_t bus_v_q16,
                                         uint32_t r_shunt_mohm);

#ifdef __cplusplus
}
#endif

#endif /* ELE_DRIVER_H */
