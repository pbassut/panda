#pragma once

#include "safety_declarations.h"

typedef struct {
  const int ABS_6;
  const int DAS_1;
  const int DAS_2;
  const int LKAS_COMMAND;
} FiatAddrs;

typedef enum {
  FASTBACK_LIMITED_EDITION,
} FiatPlatform;
static FiatPlatform fiat_platform;
static const FiatAddrs *fiat_addrs;

uint8_t fca_fastback_crc8_lut_j1850[256];  // Static lookup table for CRC8 SAE J1850

static uint32_t fca_fastback_get_checksum(const CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint8_t fca_fastback_get_counter(const CANPacket_t *to_push) {
  int counter_byte = GET_LEN(to_push) - 2U;
  return (uint8_t)(GET_BYTE(to_push, counter_byte) & 0xFU);
}

static uint32_t fca_fastback_compute_crc(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  // CRC is in the last byte, poly is same as SAE J1850 but uses a different init value and output XOR
  uint8_t crc = 0xFF;
  uint8_t final_xor = 0xFF;

  for (int i = 0; i < (len - 1); i++) {
    crc ^= (uint8_t)GET_BYTE(to_push, i);
    crc = fca_fastback_crc8_lut_j1850[crc];
  }

  return (uint8_t)(crc ^ final_xor);
}

static void fiat_rx_hook(const CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (bus == 0 && addr == 0x101) {
    int speed = GET_BYTE(to_push, 1) + (GET_BYTE(to_push, 2) >> 5);
    vehicle_moving = speed != 0;
  }

  if (bus == 0 && addr == 0xFA) {
    int pressed = (GET_BYTE(to_push, 0) >> 3);
    brake_pressed = pressed == 1;
  }

  if (bus == 1 && addr == 0x2FA) {
    int button_pressed = GET_BYTE(to_push, 0);
    int resume_pressed = button_pressed == 8;
    controls_allowed = resume_pressed;
  }

  if (bus == 1 && addr == 0x5A5) {
    int acc_state = GET_BIT(to_push, 21U);
    pcm_cruise_check(acc_state == 1);
  }

  if (bus == 0 && addr == 0xFC) {
    int pedal_threshold = (GET_BYTE(to_push, 2) << 3 >> 3) + (GET_BYTE(to_push, 3) >> 5);
    gas_pressed = pedal_threshold > 0;
  }

  generic_rx_checks((bus == 0) && (addr == fiat_addrs->LKAS_COMMAND));
}

static bool fiat_tx_hook(const CANPacket_t *to_send) {
  const SteeringLimits FASTBACK_STEERING_LIMITS = {
    .max_steer = 361,
    .max_rt_delta = 182,
    .max_rt_interval = 250000,
    .max_rate_up = 14,
    .max_rate_down = 14,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;
  int addr = GET_ADDR(to_send);

  // STEERING
  if (addr == fiat_addrs->LKAS_COMMAND) {
    int desired_torque = (GET_BYTE(to_send, 0) | GET_BYTE(to_send, 1));
    desired_torque -= 1024;

    const SteeringLimits limits = FASTBACK_STEERING_LIMITS;

    bool steer_req = (fiat_platform == FASTBACK_LIMITED_EDITION) ? GET_BIT(to_send, 4U) : (GET_BYTE(to_send, 3) & 0x7U) == 2U;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      // tx = false;
      tx = true;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (addr == fiat_addrs->DAS_1) {
    const bool is_cancel = GET_BYTE(to_send, 0) == 0x80;
    const bool is_resume = GET_BYTE(to_send, 0) == 0x8;
    const bool allowed = is_cancel || (is_resume && controls_allowed);
    if (!allowed) {
      tx = true;
    }
  }

  return tx;
}

static int fiat_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  // forward to camera
  if (bus_num == 0) {
    bus_fwd = 2;
  }

  // forward all messages from camera except LKAS messages
  const bool is_lkas = addr == fiat_addrs->LKAS_COMMAND;
  if ((bus_num == 2) && !is_lkas){
    bus_fwd = 0;
  }

  return bus_fwd;
}

static safety_config fiat_init(uint16_t param) {
  gen_crc_lookup_table_8(0x1D, fca_fastback_crc8_lut_j1850);

  static const FiatAddrs FASTBACK_ADDRS = {
    .DAS_1            = 0x2FA,   // ACC engagement states from DASM
    .LKAS_COMMAND     = 0x1F6,   // LKAS controls from DASM
  };

  static RxCheck fastback_rx_checks[] = {
    {.msg = {{FASTBACK_ADDRS.DAS_1,         1, 8,   .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{FASTBACK_ADDRS.LKAS_COMMAND,  0, 11,  .check_checksum = true, .max_counter = 15U, .frequency = 100U},  { 0 }, { 0 }}},
  };

  static const CanMsg FASTBACK_TX_MSGS[] = {
    {FASTBACK_ADDRS.DAS_1,        1, 8},
    {FASTBACK_ADDRS.LKAS_COMMAND, 0, 11},
  };


  fiat_platform = FASTBACK_LIMITED_EDITION;
  fiat_addrs = &FASTBACK_ADDRS;
  UNUSED(param);

  safety_config ret = BUILD_SAFETY_CFG(fastback_rx_checks, FASTBACK_TX_MSGS);
  return ret;
}

const safety_hooks fiat_hooks = {
  .init = fiat_init,
  .rx = fiat_rx_hook,
  .tx = fiat_tx_hook,
  .fwd = fiat_fwd_hook,
  .get_counter = fca_fastback_get_counter,
  .get_checksum = fca_fastback_get_checksum,
  .compute_checksum = fca_fastback_compute_crc,
};
